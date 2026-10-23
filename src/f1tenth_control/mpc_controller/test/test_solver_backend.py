"""
mpc_solver_impl:=py|rust (main-jazzy-rust R2): the solver core, the switch, the refusal.

solve_mpc_step sends an rti tick without a stage_spec either to
mpc_solver._solve_rti (py, the default) or to the Rust solver core in
f1tenth_mpc_core through mpc_native.solve_rti (rust). What is pinned here:

  - the two return the SAME (u0, info), bit for bit -- keys, key order, value
    types, every array element, status, message, iterations, cost -- over a grid
    of both speed models, obstacles, soft / hard / no boundary rows, warm starts
    (none, short, non-finite), the signed turn branch, a target line, the stage
    heading term and a prediction step longer than the control period; and on
    every 7th tick of the R2.0 fixture when the repository has it. The full
    parity (QP bytes, OSQP alone, all fixture and live ticks, closed loop) is
    tools/rust_parity/mpc, output/rust/parity_mpc_solver.md;
  - the same exception type for the same bad input;
  - OSQP: the Rust core runs OSQP C 1.0.0 with exactly the settings osqp-python
    hands it for _solve_rti, field by field;
  - the dispatch: rust only for rti without stage_spec (slsqp and the reverse
    candidates stay Python), DEFAULT_IMPL from stack_params unless
    F1TENTH_MPC_SOLVER_IMPL overrides it, _mpc_solve passes the node's impl;
  - the capture hook (F1TENTH_MPC_CAPTURE) writes a tick that replays to the
    same outputs;
  - the node: the parameter defaults to that; rust that does not load refuses
    the start, and so does an unknown value; read-only at run time.

The Rust cases skip in a fixture (never at import, which would skip the whole
directory under launch_testing) when the library is not installed.

Run standalone: python3 -m pytest test/test_solver_backend.py -v
"""

import gzip
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys

from f1tenth_params import corridor_geometry as cg
from f1tenth_params.object_geometry import SOLVER_FRONT_POINT_OFFSETS_M
from f1tenth_params.param_defaults import get_value
from mpc_controller import mpc_native, mpc_solver, solver_backend
import numpy as np
import pytest

_FIXTURE = (Path(__file__).resolve().parents[4] / 'tools' / 'rust_parity' / 'mpc' / 'data'
            / 'rti_ticks.jsonl.gz')

PARAMS = {'L': 0.305, 'lr': 0.17}
LIMITS = {'delta_min': -0.283, 'delta_max': 0.278, 'a_min': -2.0, 'a_max': 3.0,
          'dDeltaMin': -0.5, 'dDeltaMax': 0.5, 'dAMin': -2.0, 'dAMax': 2.0,
          'vMin': 0.0, 'vMax': 1.0}
WEIGHTS = {'w_term': 9.0, 'w_v': 4.2857, 'w_psi': 4.5, 'w_psi_stage': 4.7736,
           'w_corr': 3.5714, 'w_u_a': 0.0, 'w_du_delta': 8.5714, 'w_du_a': 0.0,
           'w_delta0': 0.0, 'w_obs': 8.0, 'w_line': 1.1662}


@pytest.fixture
def rust():
    if not mpc_native.available():
        pytest.skip(f'f1tenth_mpc_core not loadable: {mpc_native.load_error()}')
    return mpc_native


def _bits(x):
    return struct.pack('<d', float(x))


def assert_same(a, b, path='info'):
    """Bit-for-bit equality of two solver results, types included (NaN == NaN)."""
    assert type(a) is type(b), f'{path}: {type(a).__name__} != {type(b).__name__}'
    if isinstance(a, dict):
        assert list(a) == list(b), f'{path}: keys {list(a)} != {list(b)}'
        for key in a:
            assert_same(a[key], b[key], f'{path}[{key!r}]')
    elif isinstance(a, np.ndarray):
        assert a.dtype == b.dtype and a.shape == b.shape, path
        assert a.tobytes() == b.tobytes(), (
            f'{path}: {int(np.sum(a.view(np.uint64) != b.view(np.uint64)))} elements differ')
    elif isinstance(a, (list, tuple)):
        assert len(a) == len(b), path
        for i, (x, y) in enumerate(zip(a, b)):
            assert_same(x, y, f'{path}[{i}]')
    elif isinstance(a, float):
        assert _bits(a) == _bits(b) or (math.isnan(a) and math.isnan(b)), \
            f'{path}: {a!r} != {b!r}'
    else:
        assert a == b, f'{path}: {a!r} != {b!r}'


def _corridor(psi_end=0.6, length=3.0, obstacles=(), turn=None, line=False):
    c = cg.corridor_curves(0.0, 0.0, 0.0, psi_end, length, 120, r_min=0.923)
    corridor = {k: c[k] for k in ('xc', 'yc', 'nx', 'ny', 'halfWidth')}
    corridor.update({'psiRef': psi_end, 'obstacles_world': list(obstacles),
                     'car_radius': 0.20, 'avoidance_margin': 0.12})
    if turn is not None:
        corridor.update({'psiRefTurn': turn, 'psiStart': 0.05})
    if line:
        corridor['targetLine'] = {'p': [2.5, 0.9], 'psi': 0.5}
    return corridor


def _speed_model(n, v_hi=True):
    return {'alpha': 0.39, 'delay': 0, 'd': 0.03,
            'c_lo': np.full(n, 0.0), 'c_hi': np.full(n, 0.6),
            'v_hi': (np.array([0.5] * (n - 2) + [math.inf, 0.45]) if v_hi else None),
            'slack_weight': 1.0e3}


#: The grid's obstacle sets, boundary rows and warm starts (N = 20).
_OBSTACLE_SETS = ((), ((1.4, 0.15, 0.18),),
                  ((1.4, 0.15, 0.18), (2.2, -0.5, 0.2), (0.8, 0.7, 0.15)))
_BOUNDARIES = {'none': [], 'soft': [(0.0, 1.0, 0.9), (0.6, -0.8, 1.5)],
               'hard': [(0.0, 1.0, 1.4)]}
_WARM = {'none': None, 'plan': np.tile([0.05, 0.2], 20), 'short': np.zeros(7),
         'nan': np.full(40, np.nan)}


def _cases():
    """(name, kwargs) of solve_mpc_step, rti, no stage_spec."""
    n = 20
    out = []
    for speed in ('legacy', 'offset_free'):
        for obs in _OBSTACLE_SETS:
            for bnd, rows in _BOUNDARIES.items():
                for warm, warm_z in _WARM.items():
                    x0 = [0.05, -0.02, 0.03, 0.35] + ([0.4] if speed == 'offset_free' else [])
                    kw = {
                        'x0': x0, 'last_u': np.array([0.02, 0.1]),
                        'pref_nom': np.array([2.6, 1.0]), 'corridor': _corridor(obstacles=obs),
                        'horizon': n, 'ts': 0.1, 'params': PARAMS, 'limits': LIMITS,
                        'weights': dict(WEIGHTS), 'obstacles': list(obs), 'dmin': 0.32,
                        'vdes': 0.5, 'solver': 'rti', 'boundaries': rows,
                        'boundary_max_sources': 4, 'boundary_hard': bnd == 'hard',
                        'boundary_slack_weight': 2.0e6, 'boundary_slack_max': 0.5,
                        'warm_start_z': warm_z, 'control_period': 0.1,
                        'speed_model': _speed_model(n) if speed == 'offset_free' else None}
                    out.append((f'{speed}/obs{len(obs)}/{bnd}/{warm}', kw))
    base = dict(out[4][1])
    out.append(('turn', dict(base, corridor=_corridor(turn=1.2))))
    out.append(('line', dict(base, corridor=_corridor(line=True))))
    out.append(('ts0.2_cp0.1', dict(base, ts=0.2, control_period=0.1)))
    out.append(('cp_none', dict(base, control_period=None)))
    out.append(('no_optional_weights', dict(base, weights={
        k: v for k, v in WEIGHTS.items() if k not in ('w_psi', 'w_corr', 'w_line',
                                                      'w_psi_stage')})))
    out.append(('psi_near_pi', dict(base, x0=[0.05, -0.02, 3.13, 0.35],
                                    corridor=_corridor(psi_end=-3.1))))
    out.append(('no_v_hi', dict(out[60][1], speed_model=_speed_model(20, v_hi=False))))
    out.append(('delay', dict(out[60][1], speed_model=dict(_speed_model(20), delay=1))))
    return out


@pytest.mark.parametrize('name, kwargs', _cases(), ids=[c[0] for c in _cases()])
def test_rust_and_python_return_the_same_tick(rust, name, kwargs):
    u_py, info_py = mpc_solver.solve_mpc_step(**kwargs, impl='py')
    u_rs, info_rs = mpc_solver.solve_mpc_step(**kwargs, impl='rust')
    assert_same(u_py, u_rs, f'{name}: u0')
    assert_same(info_py, info_rs, f'{name}: info')


def _fixture_ticks():
    if not _FIXTURE.is_file():
        return []
    with gzip.open(_FIXTURE, 'rt', encoding='utf-8') as fh:
        lines = [line for line in fh if line.strip()]
    return lines[1::7]


def _decode(obj):
    if isinstance(obj, dict):
        if len(obj) == 1 and '$float' in obj:
            return float(obj['$float'])
        return {k: _decode(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_decode(v) for v in obj]
    return obj


def _kwargs(record):
    """rti_fixture.solve_kwargs, inlined: arrays as float64, tuples for rows."""
    kw = _decode(record['inputs'])
    for key in ('x0', 'last_u', 'pref_nom', 'warm_start_z'):
        if kw.get(key) is not None:
            kw[key] = np.asarray(kw[key], dtype=float)
    for key in ('obstacles', 'boundaries'):
        if kw.get(key) is not None:
            kw[key] = [tuple(row) for row in kw[key]]
    cor = kw['corridor']
    for key in ('xc', 'yc', 'nx', 'ny', 'halfWidth'):
        cor[key] = np.asarray(cor[key], dtype=float)
    cor['obstacles_world'] = [tuple(row) for row in cor['obstacles_world']]
    if kw.get('speed_model'):
        for key in ('c_lo', 'c_hi', 'v_hi'):
            if kw['speed_model'].get(key) is not None:
                kw['speed_model'][key] = np.asarray(kw['speed_model'][key], dtype=float)
    return kw


def test_the_fixture_ticks_solve_the_same_on_both(rust):
    lines = _fixture_ticks()
    if not lines:
        pytest.skip(f'{_FIXTURE} not in this checkout')
    for line in lines:
        record = json.loads(line)
        kw = _kwargs(record)
        u_py, info_py = mpc_solver.solve_mpc_step(**kw, impl='py')
        u_rs, info_rs = mpc_solver.solve_mpc_step(**kw, impl='rust')
        assert_same(u_py, u_rs, f"{record['id']}: u0")
        assert_same(info_py, info_rs, f"{record['id']}: info")


def _bad(kwargs, **changes):
    out = dict(kwargs)
    out.update(changes)
    return out


@pytest.mark.parametrize('change, error', [
    ({'x0': [0.0, 0.0, 0.0]}, ValueError),
    ({'x0': [0.0, 0.0, math.inf, 0.3]}, ValueError),
    ({'params': {'L': 0.0, 'lr': 0.17}}, ZeroDivisionError),
    ({'corridor': dict(_corridor(), xc=np.array([]), yc=np.array([]), nx=np.array([]),
                       ny=np.array([]), halfWidth=np.array([]))}, ValueError),
    ({'corridor': {k: v for k, v in _corridor().items() if k != 'obstacles_world'}},
     KeyError),
    ({'corridor': {k: v for k, v in _corridor().items() if k != 'psiRef'}}, KeyError),
    ({'speed_model': dict(_speed_model(20), c_lo=[0.0] * 5),
      'x0': [0.0, 0.0, 0.0, 0.3, 0.3]}, IndexError),
])
def test_the_same_inputs_raise_the_same_exception(rust, change, error):
    kwargs = _bad(_cases()[0][1], **change)
    with pytest.raises(error) as py_err:
        mpc_solver.solve_mpc_step(**kwargs, impl='py')
    with pytest.raises(error) as rs_err:
        mpc_solver.solve_mpc_step(**kwargs, impl='rust')
    if error is not IndexError:
        assert str(py_err.value) == str(rs_err.value)


def test_a_qp_beyond_the_size_cap_is_refused_not_allocated(rust):
    # 6 x 1001 variables > MAX_QP_VARIABLES: the dense P would be 290 MB+;
    # Rust refuses (ValueError) instead of risking an allocation abort.
    kwargs = dict(_cases()[0][1], horizon=1001)
    with pytest.raises(ValueError, match='more than 6000 variables'):
        mpc_solver.solve_mpc_step(**kwargs, impl='rust')
    assert rust.MAX_QP_VARIABLES == 6000


def test_the_core_runs_osqp_1_0_0_with_the_pythons_settings(rust):
    osqp = pytest.importorskip('osqp')
    import scipy.sparse as sp
    prob = osqp.OSQP()
    prob.setup(sp.csc_matrix(np.eye(2)), np.zeros(2), sp.csc_matrix(np.eye(2)),
               -np.ones(2), np.ones(2),
               warm_starting=True, verbose=False, polishing=True, max_iter=200)
    py = {k: getattr(prob.settings, k) for k in dir(prob.settings) if not k.startswith('_')}
    py = {k: (int(v) if not isinstance(v, (int, float, bool)) else v) for k, v in py.items()}
    ours = rust.osqp_settings('solver')
    assert set(py) <= set(ours)
    for key, value in py.items():
        assert ours[key] == value, (key, ours[key], value)
    # the two the binding does not expose: OSQP C's defaults
    default = rust.osqp_settings('default')
    assert ours['allocate_solution'] == default['allocate_solution'] == 1
    assert ours['profiler_level'] == default['profiler_level'] == 0
    assert rust.osqp_version() == '1.0.0'


# ---------------------------------------------------------------------------
# the dispatch
# ---------------------------------------------------------------------------

def test_the_dispatcher_picks_the_implementation(rust, monkeypatch):
    calls = []
    real = mpc_native.solve_rti
    monkeypatch.setattr(mpc_native, 'solve_rti',
                        lambda *a, **k: calls.append('rust') or real(*a, **k))
    kwargs = _cases()[13][1]
    mpc_solver.solve_mpc_step(**kwargs, impl='py')
    assert calls == []
    mpc_solver.solve_mpc_step(**kwargs, impl='rust')
    assert calls == ['rust']
    # SLSQP stays Python, whatever impl says
    legacy = _cases()[1][1]
    mpc_solver.solve_mpc_step(**dict(legacy, solver='slsqp'), impl='rust')
    assert calls == ['rust']
    with pytest.raises(ValueError, match='mpc_solver_impl must be one of'):
        mpc_solver.solve_mpc_step(**kwargs, impl='cpp')


def test_a_stage_spec_tick_stays_python(rust, monkeypatch):
    calls = []
    monkeypatch.setattr(mpc_native, 'solve_rti', lambda *a, **k: calls.append(1))
    n = 20
    kwargs = dict(_cases()[1][1])
    spec = {'v_lo': [0.0] * n, 'v_hi': [1.0] * n, 'vdes': [0.5] * n}
    _u, info = mpc_solver.solve_mpc_step(**kwargs, stage_spec=spec, impl='rust')
    assert calls == [] and 'rev_slack' in info


def test_a_call_without_impl_gets_the_default(rust, monkeypatch):
    calls = []
    real = mpc_native.solve_rti
    monkeypatch.setattr(mpc_native, 'solve_rti',
                        lambda *a, **k: calls.append(1) or real(*a, **k))
    kwargs = _cases()[1][1]
    monkeypatch.setattr(solver_backend, 'DEFAULT_IMPL', 'rust')
    mpc_solver.solve_mpc_step(**kwargs)
    assert calls == [1]
    monkeypatch.setattr(solver_backend, 'DEFAULT_IMPL', 'py')
    mpc_solver.solve_mpc_step(**kwargs)
    assert calls == [1]


def test_mpc_solve_passes_the_nodes_impl_and_nothing_without_one(monkeypatch):
    from types import SimpleNamespace

    from mpc_controller import MPC_corr
    seen = []
    monkeypatch.setattr(MPC_corr, 'solve_mpc_step', lambda **kw: seen.append(kw) or (0, {}))
    node = SimpleNamespace(allow_reverse=False, reverse_planner=None)
    MPC_corr.MPCController._mpc_solve(node, x0=1)
    node.mpc_solver_impl = 'rust'
    MPC_corr.MPCController._mpc_solve(node, x0=1)
    assert seen == [{'x0': 1}, {'x0': 1, 'impl': 'rust'}]


def test_require_and_check():
    assert solver_backend.require('py') == 'py'
    with pytest.raises(ValueError):
        solver_backend.require('rs')
    assert solver_backend.IMPLS == ('py', 'rust')
    assert get_value('mpc_solver_impl') == 'py', 'the shipped default stays py'
    assert solver_backend.runs_rust('rust', 'rti', None)
    assert not solver_backend.runs_rust('rust', 'slsqp', None)
    assert not solver_backend.runs_rust('rust', 'rti', {'v_lo': []})
    assert not solver_backend.runs_rust('py', 'rti', None)


def test_rust_that_does_not_load_is_refused(monkeypatch):
    monkeypatch.setattr(mpc_native, 'available', lambda: False)
    monkeypatch.setattr(mpc_native, 'load_error', lambda: 'OSError: not here')
    with pytest.raises(RuntimeError, match='not here'):
        solver_backend.require('rust')


def _python(code, **env):
    full = dict(os.environ)
    full.pop(solver_backend.ENV_IMPL, None)
    full.update(env)
    return subprocess.run([sys.executable, '-c', code], env=full,
                          capture_output=True, text=True, timeout=120)


def test_default_impl_is_stack_params_unless_the_test_hook_says_otherwise():
    code = 'from mpc_controller import solver_backend as b; print(b.DEFAULT_IMPL)'
    assert _python(code).stdout.strip() == get_value('mpc_solver_impl')
    assert _python(code, F1TENTH_MPC_SOLVER_IMPL='rust').stdout.strip() == 'rust'
    bad = _python(code, F1TENTH_MPC_SOLVER_IMPL='fast')
    assert bad.returncode != 0 and 'mpc_solver_impl must be one of' in bad.stderr


def test_the_capture_hook_writes_ticks_that_replay_the_same(rust, tmp_path, monkeypatch):
    monkeypatch.setenv(solver_backend.ENV_CAPTURE, str(tmp_path))
    kwargs = _cases()[61][1]
    u0, info = mpc_solver.solve_mpc_step(**kwargs, impl='py')
    monkeypatch.delenv(solver_backend.ENV_CAPTURE)
    files = list(tmp_path.glob('rti_capture_*.jsonl.gz'))
    assert len(files) == 1
    with gzip.open(files[0], 'rt', encoding='utf-8') as fh:
        records = [json.loads(line) for line in fh]
    assert len(records) == 1 and records[0]['impl'] == 'py'
    replay = _kwargs(records[0])
    assert replay['weights'] == kwargs['weights'], 'the UNSCALED weights are recorded'
    u_rs, info_rs = mpc_solver.solve_mpc_step(**replay, impl='rust')
    assert_same(u0, u_rs, 'u0')
    assert_same(info, info_rs, 'info')


def test_the_library_reports_its_solver(rust):
    assert 'opt-level 3' in rust.build_info()
    assert 'OSQP C 1.0.0' in solver_backend.describe('rust')
    assert solver_backend.describe('py') == 'py (mpc_solver._solve_rti)'
    assert tuple(SOLVER_FRONT_POINT_OFFSETS_M) == (0.10, 0.25, 0.40)


# ---------------------------------------------------------------------------
# the node
# ---------------------------------------------------------------------------

@pytest.fixture
def make_node(monkeypatch):
    """Build an MPCController with `overrides` (ROS args), torn down after the test."""
    rclpy = pytest.importorskip('rclpy')
    for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'ROS_STATIC_PEERS',
                'FASTRTPS_DEFAULT_PROFILES_FILE'):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '148'))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    made = []

    def _make(**overrides):
        from mpc_controller.MPC_corr import MPCController
        args = ['--ros-args', '--log-level', 'fatal']
        for key, value in overrides.items():
            args += ['-p', f'{key}:={value}']
        if rclpy.ok():
            rclpy.shutdown()
        rclpy.init(args=args)
        node = MPCController()
        made.append(node)
        return node

    yield _make
    for node in made:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_the_node_defaults_to_the_backend_default(make_node):
    node = make_node()
    assert node.mpc_solver_impl == solver_backend.default_impl()


def test_the_node_runs_rust_when_asked(rust, make_node):
    assert make_node(mpc_solver_impl='rust').mpc_solver_impl == 'rust'


def test_the_switch_is_read_only_at_run_time(make_node):
    from rclpy.parameter import Parameter
    node = make_node()
    result = node.set_parameters([Parameter('mpc_solver_impl', value='rust')])[0]
    assert not result.successful
    assert node.mpc_solver_impl == solver_backend.default_impl()


def test_the_node_refuses_rust_it_cannot_load(make_node, monkeypatch):
    monkeypatch.setattr(mpc_native, 'available', lambda: False)
    monkeypatch.setattr(mpc_native, 'load_error', lambda: 'OSError: no library')
    with pytest.raises(RuntimeError, match='no library'):
        make_node(mpc_solver_impl='rust')


def test_the_node_refuses_an_unknown_impl(make_node):
    with pytest.raises(ValueError, match='mpc_solver_impl must be one of'):
        make_node(mpc_solver_impl='cpp')
