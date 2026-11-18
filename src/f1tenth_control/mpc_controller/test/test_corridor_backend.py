"""
corridor_impl:=py|rust (main-jazzy-rust R1): the backend, the switch, the refusal.

corridor_backend.py sends MPC_corr's two geometry calls either to
f1tenth_params.corridor_geometry (py, the default) or to the Rust library
f1tenth_mpc_core through mpc_native.py (rust). What is pinned here:

  - the two return the SAME dict, bit for bit -- keys, key order, value types,
    every array element, 'ny' being the 'tx' array -- on the 24 archived
    object rebuilds and a grid of shapes, walls and clamps. The full parity
    (captured calls, a 10k edge-case sweep) is tools/rust_parity/corridor,
    output/rust/parity_corridor.md;
  - the same exception for the same bad input;
  - build_straight_corridor follows a stand-in's corridor_impl, and
    corridor_backend.DEFAULT_IMPL without one;
  - DEFAULT_IMPL is stack_params.yaml's corridor_impl unless
    F1TENTH_CORRIDOR_IMPL overrides it (the test hook);
  - the node: the parameter defaults to that; rust that does not load refuses
    the start, and so does an unknown value.

The Rust cases skip in a fixture (never at import, which would skip the whole
directory under launch_testing) when the library is not installed: a
workspace built without colcon-ros-cargo has none.

Run standalone: python3 -m pytest test/test_corridor_backend.py -v
"""

import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys

from f1tenth_params import corridor_geometry as cg
from f1tenth_params.param_defaults import get_value
from mpc_controller import corridor_backend, mpc_native
import numpy as np
import pytest

_REBUILDS = (Path(__file__).resolve().parents[3] / 'f1tenth_params' / 'test' / 'data'
             / 'object_rebuilds.json')


@pytest.fixture
def rust():
    if not mpc_native.available():
        pytest.skip(f'f1tenth_mpc_core not loadable: {mpc_native.load_error()}')
    return mpc_native


def _bits(x):
    return struct.pack('<d', float(x))


def assert_same(a, b, path='corridor'):
    """Bit-for-bit equality of two corridor values, types included."""
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
        assert _bits(a) == _bits(b), f'{path}: {a!r} != {b!r}'
    else:
        assert a == b, f'{path}: {a!r} != {b!r}'


def _rebuilds():
    if not _REBUILDS.is_file():
        return []
    return json.loads(_REBUILDS.read_text())['rebuilds']


def _cases():
    """Return (function name, args, kwargs): the planner's shapes on the archived data."""
    out = []
    for rec in _rebuilds():
        car, goal = rec['car'], rec['goal']
        origin = rec['lead_in_origin']
        dpsi = cg.wrap_pi(rec['psi_c'] - rec['yaw'])
        length = float(np.hypot(goal[0] - car[0], goal[1] - car[1]))
        r_min = 0.923 if dpsi >= 0.0 else 0.955
        for walls in cg.CORRIDOR_WALL_SHAPES:
            # the straight corridor on the target line
            line = float(np.hypot(goal[0] - origin[0], goal[1] - origin[1]))
            out.append(('corridor_curves',
                        (origin[0], origin[1], rec['psi_c'], rec['psi_c'], line, 120),
                        {'dpsi': 0.0, 'u_start': 0.0, 'u_end': 0.40, 'w0': 0.4333,
                         'w1': 0.7667, 'walls': walls}))
            # the arc from the car, with both clamps
            out.append(('corridor_curves',
                        (car[0], car[1], rec['yaw'], rec['psi_c'], max(length, 1e-3), 120),
                        {'dpsi': dpsi, 'u_start': 0.0, 'u_end': 0.40, 'w0': 0.4333,
                         'w1': 0.7667, 'r_min': r_min, 'length_ref': 3.0, 'walls': walls}))
            # the pose arc ending on the goal
            out.append(('corridor_curves_to_pose',
                        (car[0], car[1], rec['yaw'], goal[0], goal[1], rec['psi_c'], 120),
                        {'w0': 0.4333, 'w1': 0.7667, 'r_min': r_min, 'walls': walls}))
    # a grid of the clamps and the defaults
    for psi_end in (-3.0, -0.5, 0.0, 0.7, math.pi):
        for length in (0.0, 0.4, 3.0):
            for kwargs in ({}, {'r_min': 0.923}, {'length_ref': 3.0},
                           {'dpsi': 4.7, 'r_min': 0.955}, {'walls': 'trapezoid'}):
                out.append(('corridor_curves', (0.3, -1.2, 0.2, psi_end, length, 120), kwargs))
                out.append(('corridor_curves_to_pose',
                            (0.3, -1.2, 0.2, 0.3 + length, -1.0, psi_end, 120),
                            {k: v for k, v in kwargs.items() if k != 'dpsi'}))
    out.append(('corridor_curves', (0.0, 0.0, 0.0, 1.0, 2.0, 2), {}))
    out.append(('corridor_curves_to_pose', (1.0, 1.0, 0.0, 1.0, 1.0, 2.0, 2), {}))
    return out


def test_the_archived_rebuilds_are_there():
    assert len(_rebuilds()) == 24


def test_rust_and_python_build_the_same_dict(rust):
    cases = _cases()
    assert len(cases) > 200
    for name, args, kwargs in cases:
        expected = getattr(cg, name)(*args, **kwargs)
        got = getattr(rust, name)(*args, **kwargs)
        assert_same(expected, got, f'{name}{args}{kwargs}')
        assert got['ny'] is got['tx'] and expected['ny'] is expected['tx']


@pytest.mark.parametrize('name, args, kwargs, error', [
    ('corridor_curves', (0, 0, 0, 0, 3.0, 1), {}, ValueError),
    ('corridor_curves', (0, 0, 0, 0, 3.0, 1.0), {}, ValueError),
    ('corridor_curves', (0, 0, 0, 0, 3.0, 10), {'walls': 'zigzag'}, ValueError),
    ('corridor_curves', (0, 0, math.inf, 0, 3.0, 10), {}, ValueError),
    ('corridor_curves', (0, 0, -1e308, 1e308, 3.0, 10), {}, ValueError),
    ('corridor_curves_to_pose', (0, 0, 0, 1, 0, 0, 1), {}, ValueError),
    ('corridor_curves_to_pose', (0, 0, 0, 1, 0, -math.inf, 10), {}, ValueError),
    ('corridor_curves_to_pose', (0, 0, 0, 1, 0, 0, 10), {'r_min': 0.0}, ZeroDivisionError),
    ('corridor_curves_to_pose', (0, 0, -1e308, 1, 0, 1e308, 10), {}, ValueError),
])
def test_the_same_inputs_raise_the_same_exception(rust, name, args, kwargs, error):
    with pytest.raises(error) as py_err:
        getattr(cg, name)(*args, **kwargs)
    with pytest.raises(error) as rs_err:
        getattr(rust, name)(*args, **kwargs)
    assert str(py_err.value) == str(rs_err.value)


def test_the_dispatcher_picks_the_implementation(rust, monkeypatch):
    calls = []
    real = mpc_native.corridor_curves
    monkeypatch.setattr(mpc_native, 'corridor_curves',
                        lambda *a, **k: calls.append('rust') or real(*a, **k))
    py = corridor_backend.corridor_curves(0, 0, 0, 0.5, 3.0, 120, impl='py')
    rs = corridor_backend.corridor_curves(0, 0, 0, 0.5, 3.0, 120, impl='rust')
    assert calls == ['rust']
    assert_same(py, rs)
    with pytest.raises(ValueError, match='corridor_impl must be one of'):
        corridor_backend.corridor_curves(0, 0, 0, 0.5, 3.0, 120, impl='cpp')
    with pytest.raises(ValueError, match='corridor_impl must be one of'):
        corridor_backend.corridor_curves_to_pose(0, 0, 0, 1, 1, 0, 120, impl='')


def test_require_and_check():
    assert corridor_backend.require('py') == 'py'
    with pytest.raises(ValueError):
        corridor_backend.require('rs')
    assert corridor_backend.IMPLS == ('py', 'rust')
    assert get_value('corridor_impl') == 'py', 'the shipped default stays py'


def test_rust_that_does_not_load_is_refused(monkeypatch):
    monkeypatch.setattr(mpc_native, 'available', lambda: False)
    monkeypatch.setattr(mpc_native, 'load_error', lambda: 'OSError: not here')
    with pytest.raises(RuntimeError, match='not here'):
        corridor_backend.require('rust')


def _python(code, **env):
    full = dict(os.environ)
    full.pop(corridor_backend.ENV_IMPL, None)
    full.update(env)
    return subprocess.run([sys.executable, '-c', code], env=full,
                          capture_output=True, text=True, timeout=120)


def test_default_impl_is_stack_params_unless_the_test_hook_says_otherwise():
    code = 'from mpc_controller import corridor_backend as b; print(b.DEFAULT_IMPL)'
    assert _python(code).stdout.strip() == get_value('corridor_impl')
    assert _python(code, F1TENTH_CORRIDOR_IMPL='rust').stdout.strip() == 'rust'
    bad = _python(code, F1TENTH_CORRIDOR_IMPL='fast')
    assert bad.returncode != 0 and 'corridor_impl must be one of' in bad.stderr


def test_a_missing_library_is_reported_with_its_path():
    code = ('from mpc_controller import mpc_native as m; '
            'print(m.available()); print(m.load_error()); print(m.library_path())')
    out = _python(code, F1TENTH_MPC_CORE_LIB='/nonexistent/libf1tenth_mpc_core.so')
    lines = out.stdout.splitlines()
    assert lines[0] == 'False'
    assert 'OSError' in lines[1]
    assert lines[2] == '/nonexistent/libf1tenth_mpc_core.so'


def test_the_library_reports_an_optimised_build(rust):
    assert rust.version() == '0.1.0'
    assert 'opt-level 3' in rust.build_info()


# ---------------------------------------------------------------------------
# the call sites: build_straight_corridor on a stand-in
# ---------------------------------------------------------------------------

class _FakeLogger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass


class _FakeMPC:
    """Just enough of MPCController for build_straight_corridor (test_corridor_width's)."""

    def __init__(self, goal_pose_xy=None, impl=None):
        self.front_distance = 10.0
        self.goal_pose_xy = goal_pose_xy
        self.corr_L_base = 3.0
        self.corr_N = 120
        self.corr_wmin = 0.65
        self.corr_wmax = 0.65 * cg.CORRIDOR_WIDTH_RATIO
        self.psi_init_corridor = 0.0
        self.goal_start_xy = None
        self.goal_anchor_odom = None
        self.corr_turn_u_start = 0.0
        self.corr_turn_u_end = 0.40
        if impl is not None:
            self.corridor_impl = impl

    def get_logger(self):
        return _FakeLogger()


def _build(fake, state=(0.0, 0.0, 0.3, 0.5)):
    from mpc_controller.MPC_corr import MPCController
    return MPCController.build_straight_corridor(fake, list(state))


@pytest.mark.parametrize('goal', [None, (2.0, 0.5), (1.2, -0.3), (6.0, 2.0)])
def test_build_straight_corridor_is_the_same_on_both(rust, goal):
    assert_same(_build(_FakeMPC(goal, impl='py')), _build(_FakeMPC(goal, impl='rust')))


def test_a_stand_in_without_the_field_gets_the_default(rust, monkeypatch):
    calls = []
    real = mpc_native.corridor_curves
    monkeypatch.setattr(mpc_native, 'corridor_curves',
                        lambda *a, **k: calls.append(1) or real(*a, **k))
    monkeypatch.setattr(corridor_backend, 'DEFAULT_IMPL', 'rust')
    _build(_FakeMPC())
    assert calls == [1]
    monkeypatch.setattr(corridor_backend, 'DEFAULT_IMPL', 'py')
    _build(_FakeMPC())
    assert calls == [1]


def test_a_bad_impl_on_the_object_fails_the_build():
    with pytest.raises(ValueError, match='corridor_impl must be one of'):
        _build(_FakeMPC(impl='cython'))


# ---------------------------------------------------------------------------
# the node
# ---------------------------------------------------------------------------

@pytest.fixture
def make_node(monkeypatch):
    """Build an MPCController with `overrides` (ROS args), torn down after the test."""
    rclpy = pytest.importorskip('rclpy')
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '147'))
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
    assert node.corridor_impl == corridor_backend.default_impl()


def test_the_node_runs_rust_when_asked(rust, make_node):
    assert make_node(corridor_impl='rust').corridor_impl == 'rust'


def test_the_switch_is_read_only_at_run_time(make_node):
    from rclpy.parameter import Parameter
    node = make_node()
    result = node.set_parameters([Parameter('corridor_impl', value='rust')])[0]
    assert not result.successful
    assert node.corridor_impl == corridor_backend.default_impl()


def test_the_node_refuses_rust_it_cannot_load(make_node, monkeypatch):
    monkeypatch.setattr(mpc_native, 'available', lambda: False)
    monkeypatch.setattr(mpc_native, 'load_error', lambda: 'OSError: no library')
    with pytest.raises(RuntimeError, match='no library'):
        make_node(corridor_impl='rust')


def test_the_node_refuses_an_unknown_impl(make_node):
    with pytest.raises(ValueError, match='corridor_impl must be one of'):
        make_node(corridor_impl='cpp')
