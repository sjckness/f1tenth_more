#!/usr/bin/env python3
"""Where the MPC tick's time goes: a per-part profile of mpc_solver._solve_rti (R2.0).

Measures, on representative closed-loop ticks, how long each part of the RTI
solve takes, so the Rust port (R2.1) starts where the time is. It also
records the machine and library versions, dumps the OSQP settings the Python
path really runs with, and writes the captured ticks as the parity fixture
(rti_fixture.py documents the format).

TICKS. Closed loops, not synthetic calls:
  matrix    test/speed_channel_harness.run (stack_params weights and limits,
            N = 20, mpc_prediction_dt_s, the real SpeedChannel, the stop
            ramp, a plant that is not the solver's model) for a 4 m
            goal_distance move at 0.5 m/s, for both speed models (legacy,
            offset_free) x 0 / 1 / 8 obstacles x 0 / 4 live wall rows. The
            obstacle sets are test_obstacles_v2's timing set; the wall rows
            are four (nx, ny, offset) half-planes in the odom frame, passed
            with the shipped boundary_max_sources (4), boundary_hard (false),
            wall_row_slack_weight and boundary_slack_max. With 0 live rows
            the 4 padded all-zero rows per stage are still there (as live).
  approach  test_object_approach_closed_loop.run_approach (the real
            build_straight_corridor / compute_local_target: curved object
            corridors, deflection), legacy speed model, the target as a
            0.25 m disk. Profiled as its own rows.
  edge      matrix ticks with one input changed (no / short / non-finite
            warm start, hard rows, infeasible rows, saturated slack,
            psiRefTurn, a target line, 8 slots, ts 0.2 with a 0.1 s control
            period): fixture only, not profiled.

HOW THE TIME IS SPLIT. solve_mpc_step and _solve_rti are re-compiled from
their own source (inspect.getsource) into a private copy of the module's
globals with a timestamp probe inserted before fixed anchor statements, and
with _linearize_obstacle_cost_rti / _nearest_corridor_frame /
corridor_heading_at wrapped while the stage loop runs. Nothing under src/ is
touched and the real module is never patched. Every instrumented solve is
checked bit-identical to the real one. A changed anchor fails loudly.
Per tick the minimum over --repeats replays is taken (robust against other
load on the machine); configs are summarised over their ticks.

OSQP ON ITS OWN. On each tick's captured QP: fresh OSQP() + setup +
warm_start + solve (as _solve_rti does), with OSQP C's own setup/solve/
polish times and the time osqp-python spends converting the matrices; and a
persistent workspace (setup once per run, then update(Px, Ax, q, l, u) +
warm_start + solve per tick, rho carried or reset), with the numerical
difference it makes.

Usage (bash; workspace sourced, e.g. `source output/rust_main/env.sh`):
  tools/rust_parity/mpc/profile_rti.py [--repeats 7] [--out FILE.json]
      [--fixture FILE.jsonl.gz | --no-fixture] [--cpu N] [--quick]
  tools/rust_parity/mpc/profile_rti.py --compare RUN1.json RUN2.json [--tolerance 0.10]
  tools/rust_parity/mpc/profile_rti.py --report RUN1.json [RUN2.json] --md FILE.md

Gate (R2.0): runs end to end; two runs agree within 10 % (--compare).
"""

import argparse
import contextlib
import copy
import ctypes
import gc
import hashlib
import inspect
import json
import math
import os
import platform
import statistics
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
MPC_PKG = os.path.join(REPO, 'src', 'f1tenth_control', 'mpc_controller')
TEST_DIR = os.path.join(MPC_PKG, 'test')
DEFAULT_FIXTURE = os.path.join(HERE, 'data', 'rti_ticks.jsonl.gz')
DEFAULT_OUT = os.path.join(REPO, 'output', 'rust', 'mpc', 'rti_profile.json')

sys.path.insert(0, HERE)
import rti_fixture  # noqa: E402

# ------------------------------------------------------------------ scenarios
SPEED_MODELS = ('legacy', 'offset_free')
OBSTACLE_COUNTS = (0, 1, 8)
WALL_COUNTS = (0, 4)

# test_obstacles_v2.py's timing set: one 0.36 m cluster 2 m ahead, 0.1 m off
# the path, then wall-side clutter alternating left/right.
OBSTACLE_SETS = {
    0: [],
    1: [(2.0, 0.1, 0.18)],
    8: [(2.0, 0.1, 0.18)] + [(2.5 + 0.6 * k, (-1.0) ** k * (1.0 + 0.1 * k), 0.15)
                             for k in range(7)],
}
# Four live wall rows (odom frame, normal . p <= offset, the solver then
# subtracts car_radius + avoidance_margin): left and right walls of a 1.75 m
# hallway, a far end wall and an oblique one ahead-left. The right wall can
# bind while the car passes the obstacle.
WALL_SETS = {
    0: [],
    4: [(0.0, 1.0, 0.95), (0.0, -1.0, 0.80), (1.0, 0.0, 8.0), (0.6, 0.8, 4.0)],
}
MATRIX_RUN = dict(vdes=0.5, goal_distance=4.0, duration=14.0)
MATRIX_SCENARIO = 'goal_distance 4 m at 0.5 m/s'
APPROACH_RUNS = (('45deg_4m', '45 deg off, 4 m'), ('70deg_2.5m', '70 deg off, 2.5 m'),
                 ('jitter_4m', 'jitter 5 cm @ 12.5 Hz'))
# The fixture keeps every approach and edge tick and every FIXTURE_MATRIX_STRIDE-th
# tick of each matrix run (~800 in all); every tick is profiled.
FIXTURE_MATRIX_STRIDE = 2
APPROACH_OBSTACLE_R = 0.25

# _solve_rti's own setup call (mpc_solver.py); checked against the settings
# the real path produced, so a change there fails the run.
SETUP_KWARGS = dict(warm_starting=True, verbose=False, polishing=True, max_iter=200)

# (anchor at indentation 4, segment label that STARTS there)
STEP_ANCHORS = (
    ('weights = scale_stage_weights(weights, horizon)', 'step_prep'),
)
RTI_ANCHORS = (
    ('if not OSQP_AVAILABLE:', 'prep'),
    ('x_ref = [x0.copy()]', 'model_ref'),
    ('A_list, B_list, c_list = [], [], []', 'model_jac'),
    ('padded_boundaries = pad_boundary_constraints(', 'layout'),
    ('# ---- Dynamics equality constraints', 'rows_dynamics'),
    ('# ---- Control box + rate constraints', 'rows_control'),
    ('# ---- Per-step cost/constraint terms', 'stage_terms'),
    ('# ---- boundary slack: box', 'slack_terminal'),
    ('# ---- assemble + solve.', 'rows_to_array'),
    ('P = 0.5 * (P + P.T)', 'p_symmetrize'),
    ('import scipy.sparse as sp', 'csc'),
    ('z_guess = np.concatenate([', 'z_guess'),
    ('prob = osqp.OSQP()', 'osqp_ctor'),
    ('prob.setup(P_csc, q, A_csc, l_vec, u_vec,', 'osqp_setup_call'),
    ('prob.warm_start(x=z_guess)', 'osqp_warm_start'),
    ('results = prob.solve(raise_error=False)', 'osqp_solve'),
    ('status_val = results.info.status_val', 'result_check'),
    ('x_pred = _rollout_true_dynamics(', 'rollout'),
    ('v_cmd_pred = None', 'result_split'),
    ('true_cost = planner_cost_corridor(', 'true_cost'),
    ('info = {', 'info_dict'),
    ('return u0, info', 'end'),
)
# Called only from _solve_rti's stage loop; timed there and taken out of it.
CHILD_WRAPS = (
    ('_linearize_obstacle_cost_rti', 'obstacle_lin'),
    ('_nearest_corridor_frame', 'corridor_query'),
    ('corridor_heading_at', 'corridor_query'),
)
CHILD_PARENT = 'stage_terms'

# The parts the plan names, as sums of segments.
PARTS = (
    ('row_assembly', ('layout', 'rows_dynamics', 'rows_control', 'stage_terms',
                      'slack_terminal', 'rows_to_array', 'p_symmetrize', 'z_guess')),
    ('csc_conversion', ('csc',)),
    ('osqp_setup', ('osqp_ctor', 'osqp_setup_call')),
    ('osqp_solve_polish', ('osqp_warm_start', 'osqp_solve')),
    ('obstacle_linearisation', ('obstacle_lin',)),
    ('model_jacobians', ('model_ref', 'model_jac')),
    ('rollout', ('rollout',)),
    ('planner_cost_corridor', ('true_cost',)),
    ('corridor_queries', ('corridor_query',)),
    ('other', ('step_prep', 'prep', 'result_check', 'result_split', 'info_dict')),
)
ROW_ASSEMBLY_SEGMENTS = PARTS[0][1]
OSQP_ALONE_KEYS = ('setup_wall', 'ctor_wall', 'setup_call_wall', 'ctor_builtin_wall',
                   'warm_start_wall', 'solve_wall', 'py_convert', 'c_setup', 'c_solve',
                   'c_polish', 'c_run')


def log(msg):
    print(msg, file=sys.stderr, flush=True)


# ------------------------------------------------------------------ helpers
@contextlib.contextmanager
def quiet_c_stdout():
    """Send fd 1 to /dev/null (OSQP C prints 'Polishing not needed' there)."""
    libc = ctypes.CDLL(None)
    sys.stdout.flush()
    libc.fflush(None)
    saved = os.dup(1)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 1)
    try:
        yield
    finally:
        sys.stdout.flush()
        libc.fflush(None)
        os.dup2(saved, 1)
        os.close(saved)
        os.close(devnull)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        h.update(fh.read())
    return h.hexdigest()


def pct(values, q):
    return float(np.percentile(np.asarray(values, dtype=float), q)) if len(values) else math.nan


def summary(values):
    v = np.asarray(values, dtype=float)
    if not v.size:
        return {'n': 0}
    return {'n': int(v.size), 'mean': float(v.mean()), 'p50': float(np.median(v)),
            'p95': float(np.percentile(v, 95)), 'max': float(v.max()), 'min': float(v.min())}


def git_head():
    try:
        out = subprocess.run(['git', '-C', REPO, 'rev-parse', 'HEAD'], capture_output=True,
                             text=True, timeout=10)
        return out.stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def blas_libraries():
    """Shared libraries mapped into this process whose name says BLAS/LAPACK."""
    libs = set()
    try:
        with open('/proc/self/maps') as fh:
            for line in fh:
                path = line.split()[-1] if len(line.split()) >= 6 else ''
                base = os.path.basename(path).lower()
                if base.startswith('lib') and any(
                        k in base for k in ('blas', 'lapack', 'mkl', 'atlas', 'flexiblas')):
                    libs.add(os.path.realpath(path))
    except OSError:
        pass
    return sorted(libs)


def machine_info():
    import osqp
    import scipy
    info = {
        'python': sys.version.split()[0],
        'platform': platform.platform(),
        'numpy': np.__version__,
        'scipy': scipy.__version__,
        'osqp_python': osqp.__version__,
        'osqp_c': None,
        'numpy_blas': None,
        'blas_libraries_loaded': blas_libraries(),
        'cpu_model': None,
        'cpu_count': os.cpu_count(),
        'cpu_max_mhz': None,
        'cpufreq_governor': None,
        'affinity': sorted(os.sched_getaffinity(0)),
        'loadavg_start': list(os.getloadavg()),
    }
    try:
        import osqp.ext_builtin as ext
        lib = ctypes.CDLL(ext.__file__)
        lib.osqp_version.restype = ctypes.c_char_p
        info['osqp_c'] = lib.osqp_version().decode()
        info['osqp_c_float64'] = int(ext.OSQP_USE_FLOAT) == 0
        info['osqp_c_int64'] = int(ext.OSQP_USE_LONG) == 1
    except (OSError, AttributeError, ImportError):
        pass
    try:
        cfg = np.show_config(mode='dicts')
        info['numpy_blas'] = cfg.get('Build Dependencies', {}).get('blas')
    except TypeError:
        pass
    try:
        with open('/proc/cpuinfo') as fh:
            for line in fh:
                if line.startswith('model name'):
                    info['cpu_model'] = line.split(':', 1)[1].strip()
                    break
    except OSError:
        pass
    for key, path in (('cpu_max_mhz', '/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq'),
                      ('cpufreq_governor',
                       '/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor')):
        try:
            with open(path) as fh:
                val = fh.read().strip()
            info[key] = int(val) // 1000 if key == 'cpu_max_mhz' else val
        except (OSError, ValueError):
            pass
    try:
        info['dpkg_libblas3'] = subprocess.run(
            ['dpkg-query', '-W', '-f=${Version}', 'libblas3'], capture_output=True,
            text=True, timeout=10).stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        pass
    return info


# ------------------------------------------------------------------ instrumentation
def _insert_probes(func, anchors, probe_name):
    """Source of func with `probe_name(label)` lines before each anchor (indent 4)."""
    lines = inspect.getsource(func).splitlines()
    # Skip the signature and the docstring: anchors are only searched in code.
    start = 0
    for i, line in enumerate(lines):
        if line.strip().startswith(('"""', "'''")):
            quote = line.strip()[:3]
            rest = line.strip()[3:]
            if quote in rest:
                start = i + 1
            else:
                for j in range(i + 1, len(lines)):
                    if quote in lines[j]:
                        start = j + 1
                        break
            break
    found = {}
    out = lines[:start]
    for line in lines[start:]:
        indent = len(line) - len(line.lstrip(' '))
        if indent == 4 and line.strip():
            for anchor, label in anchors:
                if line.strip().startswith(anchor):
                    if anchor in found:
                        raise RuntimeError(f'anchor {anchor!r} found twice in {func.__name__}')
                    found[anchor] = label
                    out.append(f'    {probe_name}({label!r})')
                    break
        out.append(line)
    missing = [a for a, _ in anchors if a not in found]
    if missing:
        raise RuntimeError(
            f'{func.__name__} changed: anchors not found {missing}; update RTI_ANCHORS')
    return '\n'.join(out) + '\n'


class Instrumented:
    """A probed copy of solve_mpc_step/_solve_rti over a COPY of mpc_solver's globals."""

    def __init__(self, mpc_solver):
        self.mod = mpc_solver
        self.marks = []
        self.children = []
        self.capture = None
        g = dict(mpc_solver.__dict__)
        pc = time.perf_counter
        mark_append = self.marks.append
        child_append = self.children.append
        state = self

        def fast_mark(label, _a=mark_append, _pc=pc):
            _a((label, _pc()))

        def capture_mark(label, _a=mark_append, _pc=pc):
            if label == 'result_check':
                state.capture = dict(sys._getframe(1).f_locals)
            _a((label, _pc()))

        def wrap(fn, label):
            def wrapped(*a, **k):
                t0 = pc()
                r = fn(*a, **k)
                child_append((label, pc() - t0))
                return r
            wrapped.__wrapped__ = fn
            return wrapped

        self._fast_mark, self._capture_mark = fast_mark, capture_mark
        g['__rti_mark__'] = fast_mark
        for name, label in CHILD_WRAPS:
            g[name] = wrap(getattr(mpc_solver, name), label)
        src_rti = _insert_probes(mpc_solver._solve_rti, RTI_ANCHORS, '__rti_mark__')
        src_step = _insert_probes(mpc_solver.solve_mpc_step, STEP_ANCHORS, '__rti_mark__')
        exec(compile(src_rti, '<instrumented _solve_rti>', 'exec'), g)
        exec(compile(src_step, '<instrumented solve_mpc_step>', 'exec'), g)
        self.g = g
        self.solve = g['solve_mpc_step']

    def run(self, kwargs, capture=False):
        """Return (u0, info, wall_s, segments{label: s}) for one solve."""
        del self.marks[:]
        del self.children[:]
        self.capture = None
        self.g['__rti_mark__'] = self._capture_mark if capture else self._fast_mark
        t0 = time.perf_counter()
        u0, info = self.solve(**kwargs)
        wall = time.perf_counter() - t0
        marks = self.marks
        seg = {}
        for (label, t), (_next, t_next) in zip(marks, marks[1:]):
            seg[label] = seg.get(label, 0.0) + (t_next - t)
        child_total = 0.0
        for label, dt in self.children:
            seg[label] = seg.get(label, 0.0) + dt
            child_total += dt
        seg[CHILD_PARENT] = seg.get(CHILD_PARENT, 0.0) - child_total
        seg['_total'] = marks[-1][1] - marks[0][1]
        return u0, info, wall, seg


# ------------------------------------------------------------------ capture
class Recorder:
    def __init__(self, mpc_solver):
        self.sig = inspect.signature(mpc_solver.solve_mpc_step)
        self.records = []
        self._counts = {}

    def explicit(self, kwargs):
        bound = self.sig.bind(**kwargs)
        bound.apply_defaults()
        return dict(bound.arguments)

    def wrap(self, solve, group, config, inject=None):
        """A solve_mpc_step stand-in that injects kwargs, solves and records."""
        def recorded(**kwargs):
            if inject:
                kwargs = inject(dict(kwargs))
            full = self.explicit(kwargs)
            snapshot = copy.deepcopy(full)
            t0 = time.perf_counter()
            u0, info = solve(**full)
            dt = time.perf_counter() - t0
            key = json.dumps(config, sort_keys=True)
            tick = self._counts.get(key, 0)
            self._counts[key] = tick + 1
            self.records.append({
                'group': group, 'config': dict(config), 'tick': tick,
                'kwargs': snapshot, 'u0': np.array(u0, dtype=float),
                'info': copy.deepcopy(info), 'closed_loop_s': dt})
            return u0, info
        return recorded


def config_name(cfg):
    if cfg.get('group') == 'approach':
        return f"approach/{cfg['scenario']}"
    return f"{cfg['speed_model']}/obs{cfg['obstacles']}/walls{cfg['walls']}"


def tick_id(rec):
    cfg = rec['config']
    if rec['group'] == 'edge':
        return f"edge/{cfg['speed_model']}/{cfg['scenario']}/from{cfg['source_tick']:04d}"
    if rec['group'] == 'approach':
        return f"approach/{cfg['scenario']}/{rec['tick']:04d}"
    return f"matrix/{config_name(cfg)}/{rec['tick']:04d}"


def capture_matrix(recorder, harness, mpc_solver, get_value, quick=False):
    bms = int(get_value('boundary_max_sources'))
    hard = bool(get_value('boundary_hard'))
    w_wall = float(get_value('wall_row_slack_weight'))
    s_max = float(get_value('boundary_slack_max'))
    run_kw = dict(MATRIX_RUN)
    if quick:
        run_kw['duration'] = 4.0
    for speed in SPEED_MODELS:
        for n_obs in OBSTACLE_COUNTS:
            for n_walls in WALL_COUNTS:
                obs = [tuple(o) for o in OBSTACLE_SETS[n_obs]]
                walls = [tuple(w) for w in WALL_SETS[n_walls]]
                cfg = {'group': 'matrix', 'speed_model': speed, 'obstacles': n_obs,
                       'walls': n_walls, 'scenario': MATRIX_SCENARIO}

                def inject(kw, obs=obs, walls=walls):
                    kw['obstacles'] = list(obs)
                    kw['boundaries'] = list(walls)
                    kw['boundary_max_sources'] = bms
                    kw['boundary_hard'] = hard
                    kw['boundary_slack_weight'] = w_wall
                    kw['boundary_slack_max'] = s_max
                    return kw

                orig_corr, orig_solve = harness.straight_corridor, harness.solve_mpc_step

                def corridor(*a, _orig=orig_corr, _obs=obs, **k):
                    c = _orig(*a, **k)
                    c['obstacles_world'] = list(_obs)
                    return c

                harness.straight_corridor = corridor
                harness.solve_mpc_step = recorder.wrap(mpc_solver.solve_mpc_step, 'matrix',
                                                       cfg, inject)
                try:
                    harness.run(mode=speed, **run_kw)
                finally:
                    harness.straight_corridor, harness.solve_mpc_step = orig_corr, orig_solve
                n = sum(1 for r in recorder.records if r['config'] == cfg)
                log(f'  captured {config_name(cfg)}: {n} ticks')


def capture_approach(recorder, rig, mpc_solver, quick=False):
    for short, scenario in APPROACH_RUNS:
        cfg = {'group': 'approach', 'speed_model': 'legacy', 'obstacles': 1, 'walls': 0,
               'scenario': short}
        kwargs = dict(rig.SCENARIOS[scenario])
        if quick:
            kwargs['duration'] = 3.0
        orig = rig.solve_mpc_step
        rig.solve_mpc_step = recorder.wrap(mpc_solver.solve_mpc_step, 'approach', cfg)
        try:
            rig.run_approach(standoff=1.0, obstacle_r=APPROACH_OBSTACLE_R, **kwargs)
        finally:
            rig.solve_mpc_step = orig
        n = sum(1 for r in recorder.records if r['config'] == cfg)
        log(f'  captured {config_name(cfg)}: {n} ticks')


def edge_variants(kw):
    """(name, modified kwargs) for one matrix tick with 4 live walls."""
    x0 = np.asarray(kw['x0'], dtype=float)
    out = []
    v = copy.deepcopy(kw)
    v['warm_start_z'] = None
    out.append(('no_warm_start', v))
    if kw['warm_start_z'] is not None:
        v = copy.deepcopy(kw)
        v['warm_start_z'] = np.asarray(kw['warm_start_z'], dtype=float)[:-2].copy()
        out.append(('short_warm_start', v))
        v = copy.deepcopy(kw)
        w = np.asarray(kw['warm_start_z'], dtype=float).copy()
        w[3] = math.nan
        v['warm_start_z'] = w
        out.append(('nonfinite_warm_start', v))
    v = copy.deepcopy(kw)
    v['boundary_hard'] = True
    out.append(('hard_rows', v))
    v = copy.deepcopy(kw)
    v['boundary_hard'] = True
    v['boundaries'] = list(kw['boundaries'][:2]) + [(1.0, 0.0, float(x0[0]) - 0.3)]
    out.append(('hard_infeasible', v))
    v = copy.deepcopy(kw)
    v['boundaries'] = list(kw['boundaries'][:3]) + [(1.0, 0.0, float(x0[0]) - 1.0)]
    out.append(('soft_saturated', v))
    v = copy.deepcopy(kw)
    v['boundary_max_sources'] = 8
    out.append(('slots8', v))
    v = copy.deepcopy(kw)
    v['corridor']['psiRefTurn'] = 3.3
    v['corridor']['psiStart'] = float(x0[2]) - 0.2
    out.append(('psi_ref_turn', v))
    v = copy.deepcopy(kw)
    v['corridor']['targetLine'] = {'p': [float(x0[0]), float(x0[1]) + 0.15],
                                   'psi': float(x0[2]) + 0.05}
    out.append(('target_line', v))
    # stack_params runs ts == control_period (0.1); a 0.2 s prediction step
    # reaches rate_weight_factor != 1 and the separate first rate row.
    v = copy.deepcopy(kw)
    v['ts'] = 0.2
    v['control_period'] = 0.1
    out.append(('ts0.2_cp0.1', v))
    return out


def capture_edge(recorder, mpc_solver):
    picks = (10, 40)
    for speed in SPEED_MODELS:
        src_cfg = {'group': 'matrix', 'speed_model': speed, 'obstacles': 1, 'walls': 4,
                   'scenario': MATRIX_SCENARIO}
        src = [r for r in recorder.records if r['config'] == src_cfg]
        for idx in picks:
            if idx >= len(src):
                continue
            for name, kw in edge_variants(src[idx]['kwargs']):
                cfg = {'group': 'edge', 'speed_model': speed, 'obstacles': 1, 'walls': 4,
                       'scenario': name, 'source_tick': idx}
                recorder.wrap(mpc_solver.solve_mpc_step, 'edge', cfg)(**kw)
    n = sum(1 for r in recorder.records if r['group'] == 'edge')
    log(f'  captured edge: {n} ticks')


# ------------------------------------------------------------------ QP capture
def _csc_hash_parts(m):
    return [np.asarray(m.indptr, dtype='<i8'), np.asarray(m.indices, dtype='<i8'),
            np.asarray(m.data, dtype='<f8')]


def qp_from_capture(loc):
    """Everything about the QP of one solve, from _solve_rti's frame locals."""
    prob, results = loc['prob'], loc['results']
    cache = prob._derivative_cache
    P_c, A_c = cache['P'], cache['A']
    h = hashlib.sha256()
    for arr in (_csc_hash_parts(P_c) + [np.asarray(cache['q'], dtype='<f8')]
                + _csc_hash_parts(A_c)
                + [np.asarray(cache['l'], dtype='<f8'), np.asarray(cache['u'], dtype='<f8')]):
        h.update(np.ascontiguousarray(arr).tobytes())
    info = results.info
    settings = {k: getattr(prob.settings, k) for k in dir(prob.settings) if not k.startswith('_')}
    settings = {k: (int(v) if not isinstance(v, (int, float, bool)) else v)
                for k, v in settings.items()}
    return {
        'P_full': loc['P_csc'].copy(), 'A': loc['A_csc'].copy(),
        'q': np.array(loc['q'], dtype=float), 'l': np.array(loc['l_vec'], dtype=float),
        'u': np.array(loc['u_vec'], dtype=float), 'z_guess': np.array(loc['z_guess'], dtype=float),
        'P_c': P_c.copy(), 'A_c': A_c.copy(),
        'q_c': np.array(cache['q'], dtype=float), 'l_c': np.array(cache['l'], dtype=float),
        'u_c': np.array(cache['u'], dtype=float),
        'x': None if results.x is None else np.array(results.x, dtype=float),
        'settings': settings,
        'summary': {
            'n': int(prob.n), 'm': int(prob.m), 'nnz_P': int(P_c.nnz), 'nnz_A': int(A_c.nnz),
            'iter': int(info.iter), 'status_val': int(info.status_val),
            'status_polish': int(info.status_polish), 'rho_updates': int(info.rho_updates),
            'rho_estimate': float(info.rho_estimate), 'obj_val': float(info.obj_val),
            'prim_res': float(info.prim_res), 'dual_res': float(info.dual_res),
            'qp_sha256': h.hexdigest(),
        },
    }


# ------------------------------------------------------------------ OSQP alone
def masked_csc(m, mask):
    """m (sparse) on the pattern `mask` (dense bool), explicit zeros kept."""
    import scipy.sparse as sp
    dense = m.toarray()
    cols, rows = np.nonzero(mask.T)
    data = dense[rows, cols]
    indptr = np.concatenate([[0], np.cumsum(mask.sum(axis=0))]).astype(np.int32)
    return sp.csc_matrix((data, rows.astype(np.int32), indptr), shape=dense.shape)


def osqp_fresh(qps, repeats):
    """Per tick: _solve_rti's own OSQP calls, timed; C-side timers; py conversion."""
    import osqp
    rows = []
    for qp in qps:
        best = {}
        x_ok = True
        for _ in range(repeats):
            t0 = time.perf_counter()
            prob = osqp.OSQP()
            ta = time.perf_counter()
            prob.setup(qp['P_full'], qp['q'], qp['A'], qp['l'], qp['u'], **SETUP_KWARGS)
            t1 = time.perf_counter()
            prob.warm_start(x=qp['z_guess'])
            t2 = time.perf_counter()
            res = prob.solve(raise_error=False)
            t3 = time.perf_counter()
            prob._infer_mnpqalu(P=qp['P_full'], q=qp['q'], A=qp['A'], l=qp['l'], u=qp['u'])
            t5 = time.perf_counter()
            osqp.OSQP(algebra='builtin')
            t6 = time.perf_counter()
            vals = {'setup_wall': t1 - t0, 'ctor_wall': ta - t0, 'setup_call_wall': t1 - ta,
                    'ctor_builtin_wall': t6 - t5,
                    'warm_start_wall': t2 - t1, 'solve_wall': t3 - t2,
                    'py_convert': t5 - t3, 'c_setup': res.info.setup_time,
                    'c_solve': res.info.solve_time, 'c_polish': res.info.polish_time,
                    'c_run': res.info.run_time}
            for k, v in vals.items():
                best[k] = min(best.get(k, math.inf), float(v))
            if qp['x'] is not None and (res.x is None or not np.array_equal(res.x, qp['x'])):
                x_ok = False
        settings = {k: getattr(prob.settings, k) for k in dir(prob.settings)
                    if not k.startswith('_')}
        settings = {k: (int(v) if not isinstance(v, (int, float, bool)) else v)
                    for k, v in settings.items()}
        best['x_identical'] = x_ok
        best['settings_match'] = settings == qp['settings']
        rows.append(best)
    return rows


def osqp_persistent(qps, repeats, rho0):
    """One workspace per run: setup on tick 0, then update + warm_start + solve."""
    import osqp
    shapes = {(q['P_c'].shape, q['A_c'].shape) for q in qps}
    if len(shapes) != 1:
        return None
    mask_p = np.zeros(qps[0]['P_c'].shape, dtype=bool)
    mask_a = np.zeros(qps[0]['A_c'].shape, dtype=bool)
    for q in qps:
        mask_p |= q['P_c'].toarray() != 0.0
        mask_a |= q['A_c'].toarray() != 0.0
    pm = [masked_csc(q['P_c'], mask_p) for q in qps]
    am = [masked_csc(q['A_c'], mask_a) for q in qps]
    same_pattern = sum(
        1 for a, b in zip(qps[1:], qps[:-1])
        if (np.array_equal(a['P_c'].indices, b['P_c'].indices)
            and np.array_equal(a['P_c'].indptr, b['P_c'].indptr)
            and np.array_equal(a['A_c'].indices, b['A_c'].indices)
            and np.array_equal(a['A_c'].indptr, b['A_c'].indptr)))
    out = {'ticks': len(qps), 'nnz_P_union': int(mask_p.sum()), 'nnz_A_union': int(mask_a.sum()),
           'consecutive_same_pattern': same_pattern}
    # carry_rho: plain update(); reset_rho: + rho back to its setup value;
    # reset_all: + the dual iterate y back to 0, as a fresh solver starts.
    for variant in ('carry_rho', 'reset_rho', 'reset_all'):
        per_tick = [dict() for _ in qps]
        numerics = []
        for rep in range(repeats):
            prob = osqp.OSQP()
            for i, q in enumerate(qps):
                t0 = time.perf_counter()
                if i == 0:
                    # Copies: osqp-python caches the matrix objects setup() is
                    # given and update(Px=..) REBINDS their .data, so passing
                    # pm[0] itself would overwrite tick 0 for the next run.
                    prob.setup(pm[0].copy(), q['q'], am[0].copy(), q['l'], q['u'],
                               **SETUP_KWARGS)
                else:
                    prob.update(q=q['q'], l=q['l'], u=q['u'], Px=pm[i].data, Ax=am[i].data)
                t1 = time.perf_counter()
                if variant != 'carry_rho' and i > 0:
                    prob.update_settings(rho=rho0)
                t2 = time.perf_counter()
                if variant == 'reset_all':
                    prob.warm_start(x=q['z_guess'], y=np.zeros(prob.m))
                else:
                    prob.warm_start(x=q['z_guess'])
                res = prob.solve(raise_error=False)
                t3 = time.perf_counter()
                vals = {'update_wall': (t1 - t0) * 1e3, 'rho_reset_wall': (t2 - t1) * 1e3,
                        'solve_wall': (t3 - t2) * 1e3, 'total_wall': (t3 - t0) * 1e3}
                for k, v in vals.items():
                    per_tick[i][k] = min(per_tick[i].get(k, math.inf), v)
                if rep == 0:
                    x = None if res.x is None else np.array(res.x, dtype=float)
                    ref = q['x']
                    dx = (math.inf if x is None or ref is None
                          else float(np.max(np.abs(x - ref))))
                    numerics.append({
                        'iter': int(res.info.iter), 'iter_fresh': q['summary']['iter'],
                        'status': int(res.info.status_val),
                        'status_fresh': q['summary']['status_val'],
                        'max_abs_dx': dx,
                        'identical': x is not None and ref is not None
                        and bool(np.array_equal(x, ref))})
        steady = per_tick[1:] if len(per_tick) > 1 else per_tick
        out[variant] = {   # wall times in ms
            'first_setup_wall': per_tick[0]['update_wall'],
            'update_wall': summary([t['update_wall'] for t in steady]),
            'rho_reset_wall': summary([t['rho_reset_wall'] for t in steady]),
            'solve_wall': summary([t['solve_wall'] for t in steady]),
            'total_wall': summary([t['total_wall'] for t in steady]),
            'iter_delta': summary([n['iter'] - n['iter_fresh'] for n in numerics]),
            'status_changed': sum(1 for n in numerics if n['status'] != n['status_fresh']),
            'max_abs_dx': summary([n['max_abs_dx'] for n in numerics
                                   if math.isfinite(n['max_abs_dx'])]),
            'identical_ticks': sum(1 for n in numerics if n['identical']),
        }
    return out


# ------------------------------------------------------------------ main run
def run_profile(args):
    if args.cpu is not None:
        os.sched_setaffinity(0, {int(args.cpu)})
    sys.path.insert(0, TEST_DIR)
    from mpc_controller import mpc_solver
    from f1tenth_params.param_defaults import get_value
    import osqp
    import osqp.ext_builtin
    import speed_channel_harness as harness
    import test_object_approach_closed_loop as approach_rig

    meta = machine_info()
    meta['git_head'] = git_head()
    meta['repeats'] = args.repeats
    meta['quick'] = bool(args.quick)
    meta['command'] = ' '.join(sys.argv)
    sources = {}
    for name, path in (('mpc_solver.py', mpc_solver.__file__),
                       ('vehicle_model.py', sys.modules['mpc_controller.vehicle_model'].__file__),
                       ('object_geometry.py',
                        sys.modules['f1tenth_params.object_geometry'].__file__)):
        real = os.path.realpath(path)
        sources[name] = {'path': os.path.relpath(real, REPO) if real.startswith(REPO) else real,
                         'sha256': sha256_file(real)}
    meta['sources'] = sources
    stack = {k: get_value(k) for k in (
        'mpc_prediction_dt_s', 'boundary_max_sources', 'boundary_hard', 'wall_row_slack_weight',
        'boundary_slack_max', 'boundary_slack_weight', 'speed_command_mode', 'speed_lag_tau_s',
        'offset_free_speed_slack_weight', 'mpc_wheelbase_m', 'mpc_obstacles_max',
        'max_wall_constraints')}
    log(f'machine: {meta["cpu_model"]}, numpy {meta["numpy"]}, osqp {meta["osqp_python"]} '
        f'(C {meta["osqp_c"]}), load {meta["loadavg_start"]}')

    # ---- 1. capture closed-loop ticks
    log('capture (closed loops, real solve_mpc_step)')
    recorder = Recorder(mpc_solver)
    with quiet_c_stdout():
        capture_matrix(recorder, harness, mpc_solver, get_value, quick=args.quick)
        capture_approach(recorder, approach_rig, mpc_solver, quick=args.quick)
        capture_edge(recorder, mpc_solver)
    records = recorder.records

    # ---- 2. encode -> decode, replay real and instrumented, capture the QP
    log(f'replay {len(records)} ticks from the fixture encoding (real and instrumented)')
    inst = Instrumented(mpc_solver)
    encoded = []
    qps = []
    replay_identical = inst_identical = 0
    settings_seen = []
    with quiet_c_stdout():
        for i, rec in enumerate(records):
            cfg = rec['config']
            enc = {
                'id': tick_id(rec),
                'group': rec['group'],
                'config': {k: v for k, v in cfg.items() if k != 'group'},
                'tick': rec['tick'],
                'inputs': rti_fixture.encode_inputs(rec['kwargs']),
                'outputs': rti_fixture.encode_outputs(rec['u0'], rec['info']),
            }
            # Round trip through JSON text: exactly what the fixture file holds.
            enc = json.loads(json.dumps(enc, allow_nan=False), object_hook=rti_fixture._hook)
            kw = rti_fixture.solve_kwargs(enc)
            rec['replay_kwargs'] = kw
            u0, info = mpc_solver.solve_mpc_step(**kw)
            same = rti_fixture.compare_outputs(
                enc['outputs'], rti_fixture.outputs_of(u0, info))['identical']
            replay_identical += same
            u0i, infoi, _, _ = inst.run(rti_fixture.solve_kwargs(enc), capture=True)
            same_i = rti_fixture.compare_outputs(
                enc['outputs'], rti_fixture.outputs_of(u0i, infoi))['identical']
            inst_identical += same_i
            qp = qp_from_capture(inst.capture)
            enc['osqp'] = qp['summary']
            if rec['group'] != 'edge':
                qps.append(qp)
            else:
                qps.append(None)
            settings_seen.append(qp['settings'])
            encoded.append(enc)
    log(f'  replay bit-identical: real {replay_identical}/{len(records)}, '
        f'instrumented {inst_identical}/{len(records)}')
    settings0 = settings_seen[0]
    settings_constant = all(s == settings0 for s in settings_seen)

    # ---- 3. fixture
    fixture_info = None
    if not args.no_fixture:
        kept = [e for e in encoded
                if e['group'] != 'matrix' or e['tick'] % FIXTURE_MATRIX_STRIDE == 0]
        groups = {}
        for e in kept:
            groups[e['group']] = groups.get(e['group'], 0) + 1
        header = {
            'format': rti_fixture.FORMAT, 'version': rti_fixture.VERSION,
            'created_by': 'tools/rust_parity/mpc/profile_rti.py',
            'git_head': meta['git_head'],
            'sources': {k: v['sha256'] for k, v in sources.items()},
            'solver_front_point_offsets_m': list(
                sys.modules['f1tenth_params.object_geometry'].SOLVER_FRONT_POINT_OFFSETS_M),
            'stack_params': rti_fixture.jsonable(stack),
            'osqp_settings': rti_fixture.jsonable(settings0),
            'osqp_setup_kwargs': SETUP_KWARGS,
            'osqp': {'python': meta['osqp_python'], 'c': meta['osqp_c'],
                     'infty': float(osqp.OSQP().constant('OSQP_INFTY'))},
            'numpy': meta['numpy'], 'scipy': meta['scipy'],
            'count': len(kept), 'groups': groups,
            'matrix_stride': FIXTURE_MATRIX_STRIDE,
        }
        os.makedirs(os.path.dirname(os.path.abspath(args.fixture)), exist_ok=True)
        data = rti_fixture.write(args.fixture, header, kept)
        # Read the file back and solve every tick from it: the fixture alone
        # must reproduce the recorded outputs bit for bit.
        _, back = rti_fixture.load(args.fixture)
        with quiet_c_stdout():
            reread_ok = (len(back) == len(kept) and all(
                a['id'] == b['id']
                and rti_fixture.compare_outputs(a['outputs'], b['outputs'])['identical']
                and rti_fixture.compare_outputs(
                    rti_fixture.outputs_of(
                        *mpc_solver.solve_mpc_step(**rti_fixture.solve_kwargs(a))),
                    b['outputs'])['identical']
                for a, b in zip(back, kept)))
        fixture_info = {'path': _shown(args.fixture),
                        'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
                        'count': len(kept), 'groups': groups, 'reread_identical': reread_ok}
        log(f'fixture: {fixture_info["path"]} {len(kept)} ticks, {len(data) / 1e6:.2f} MB, '
            f'sha256 {fixture_info["sha256"][:16]}')

    # ---- 4. timing: uninstrumented and instrumented replays
    timed = [i for i, r in enumerate(records) if r['group'] != 'edge']
    log(f'timing {len(timed)} ticks x {args.repeats} repeats (uninstrumented + instrumented)')
    uninst = {i: [] for i in timed}
    inst_wall = {i: [] for i in timed}
    segs = {i: {} for i in timed}
    solve = mpc_solver.solve_mpc_step
    pc = time.perf_counter
    with quiet_c_stdout():
        for i in timed[:min(len(timed), 40)]:          # warm-up
            solve(**records[i]['replay_kwargs'])
        for rep in range(args.repeats):
            gc.collect()
            for i in timed:
                kw = records[i]['replay_kwargs']
                t0 = pc()
                solve(**kw)
                uninst[i].append(pc() - t0)
            gc.collect()
            for i in timed:
                _, _, wall, seg = inst.run(records[i]['replay_kwargs'])
                inst_wall[i].append(wall)
                for k, v in seg.items():
                    segs[i][k] = min(segs[i].get(k, math.inf), v)
            log(f'  repeat {rep + 1}/{args.repeats} done')
    # the timing replays must not have changed anything
    with quiet_c_stdout():
        post_ok = all(rti_fixture.compare_outputs(
            encoded[i]['outputs'],
            rti_fixture.outputs_of(*solve(**records[i]['replay_kwargs'])))['identical']
            for i in timed[::10])

    # ---- 5. OSQP alone
    log('OSQP alone: fresh setup per tick, persistent workspace')
    rho0 = float(settings0.get('rho', 0.1))
    by_cfg = {}
    for i in timed:
        by_cfg.setdefault(config_name(records[i]['config']), []).append(i)
    osqp_rows = {}
    persistent = {}
    with quiet_c_stdout():
        for name, idx in by_cfg.items():
            rows = osqp_fresh([qps[i] for i in idx], args.repeats)
            for i, row in zip(idx, rows):
                osqp_rows[i] = row
            persistent[name] = osqp_persistent([qps[i] for i in idx],
                                               max(1, min(args.repeats, 5)), rho0)
    # ---- 5b. two explanatory micro-benchmarks quoted by the report
    import timeit
    m4, b4 = np.zeros((4, 4)), np.zeros(4)
    micro = {
        # _solve_rti pads the 4x4 obstacle model to 5x5 at every stage when
        # the speed model adds the command state (offset_free only).
        'np_pad_pair_us': min(timeit.repeat(
            lambda: (np.pad(m4, ((0, 1), (0, 1))), np.pad(b4, (0, 1))),
            number=2000, repeat=5)) / 2000 * 1e6,
        'osqp_ctor_default_us': min(timeit.repeat(osqp.OSQP, number=200, repeat=5)) / 200 * 1e6,
        'osqp_ctor_builtin_us': min(timeit.repeat(
            lambda: osqp.OSQP(algebra='builtin'), number=200, repeat=5)) / 200 * 1e6,
        'sys_path_entries': len(sys.path),
    }
    meta['loadavg_end'] = list(os.getloadavg())

    # ---- 6. summarise
    configs = {}
    for name, idx in by_cfg.items():
        cfg = records[idx[0]]['config']
        tick_parts = []
        for i in idx:
            s = segs[i]
            tick_parts.append({p: sum(s.get(lbl, 0.0) for lbl in lbls) for p, lbls in PARTS})
        seg_names = sorted({k for i in idx for k in segs[i] if not k.startswith('_')})
        qsum = [qps[i]['summary'] for i in idx]
        status_counts = {}
        for q in qsum:
            status_counts[str(q['status_val'])] = status_counts.get(str(q['status_val']), 0) + 1
        polish_counts = {}
        for q in qsum:
            key = str(q['status_polish'])
            polish_counts[key] = polish_counts.get(key, 0) + 1
        slack_used = sum(1 for i in idx if any(
            v > 1e-9 for v in records[i]['info'].get('boundary_slack', [])))
        configs[name] = {
            'group': cfg['group'], 'speed_model': cfg['speed_model'],
            'obstacles': cfg['obstacles'], 'walls': cfg['walls'], 'ticks': len(idx),
            'qp': {k: summary([q[k] for q in qsum]) for k in ('n', 'm', 'nnz_P', 'nnz_A')},
            'iterations': summary([q['iter'] for q in qsum]),
            'rho_updates': summary([q['rho_updates'] for q in qsum]),
            'status_counts': status_counts, 'polish_counts': polish_counts,
            'ticks_with_boundary_slack': slack_used,
            'closed_loop_ms': summary([records[i]['closed_loop_s'] * 1e3 for i in idx]),
            'solve_mpc_step_ms': summary([min(uninst[i]) * 1e3 for i in idx]),
            'solve_mpc_step_median_of_repeats_ms': summary(
                [statistics.median(uninst[i]) * 1e3 for i in idx]),
            'instrumented_ms': summary([min(inst_wall[i]) * 1e3 for i in idx]),
            'instrumented_probe_total_ms': summary([segs[i]['_total'] * 1e3 for i in idx]),
            'parts_ms': {p: summary([t[p] * 1e3 for t in tick_parts]) for p, _ in PARTS},
            'segments_ms': {s: summary([segs[i].get(s, 0.0) * 1e3 for i in idx])
                            for s in seg_names},
            'osqp_alone_ms': {k: summary([osqp_rows[i][k] * 1e3 for i in idx])
                              for k in OSQP_ALONE_KEYS},
            'osqp_alone_x_identical': sum(1 for i in idx if osqp_rows[i]['x_identical']),
            'osqp_alone_settings_match': sum(1 for i in idx if osqp_rows[i]['settings_match']),
            'persistent': persistent[name],
        }
    # aggregate over the profiled matrix
    agg = {}
    for group in ('matrix', 'approach'):
        idx = [i for i in timed if records[i]['group'] == group]
        if not idx:
            continue
        tick_parts = [{p: sum(segs[i].get(lbl, 0.0) for lbl in lbls) for p, lbls in PARTS}
                      for i in idx]
        agg[group] = {
            'ticks': len(idx),
            'solve_mpc_step_ms': summary([min(uninst[i]) * 1e3 for i in idx]),
            'instrumented_ms': summary([min(inst_wall[i]) * 1e3 for i in idx]),
            'parts_ms': {p: summary([t[p] * 1e3 for t in tick_parts]) for p, _ in PARTS},
            'segments_ms': {s: summary([segs[i].get(s, 0.0) * 1e3 for i in idx])
                            for s in sorted({k for i in idx for k in segs[i]
                                             if not k.startswith('_')})},
            'osqp_alone_ms': {k: summary([osqp_rows[i][k] * 1e3 for i in idx])
                              for k in OSQP_ALONE_KEYS},
        }
    import scipy.sparse  # noqa: F401  (version recorded in meta)
    result = {
        'meta': meta,
        'stack_params': rti_fixture.jsonable(stack),
        'osqp': {
            'setup_kwargs': SETUP_KWARGS,
            'effective_settings': rti_fixture.jsonable(settings0),
            'settings_constant_over_ticks': settings_constant,
            'infty': float(osqp.OSQP().constant('OSQP_INFTY')),
            'status_enum': {k: int(v.value) for k, v in osqp.SolverStatus.__members__.items()},
            'infeasible_statuses': sorted(int(s.value)
                                          for s in mpc_solver._OSQP_INFEASIBLE_STATUSES),
            'linsys_solver_enum': {k: int(v) for k, v in
                                   osqp.ext_builtin.osqp_linsys_solver_type.__members__.items()},
        },
        'checks': {
            'ticks': len(records), 'replay_identical': replay_identical,
            'instrumented_identical': inst_identical, 'post_timing_identical': post_ok,
        },
        'fixture': fixture_info,
        'micro': micro,
        'configs': configs,
        'aggregate': agg,
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as fh:
        json.dump(rti_fixture.jsonable(result), fh, indent=1, allow_nan=False)
    log(f'wrote {args.out}')
    ok = (replay_identical == len(records) and inst_identical == len(records) and post_ok
          and settings_constant and (fixture_info is None or fixture_info['reread_identical']))
    for name, c in configs.items():
        log(f'  {name:28s} {c["ticks"]:4d} ticks  solve_mpc_step mean '
            f'{c["solve_mpc_step_ms"]["mean"]:6.3f} ms p95 {c["solve_mpc_step_ms"]["p95"]:6.3f}  '
            f'osqp setup {c["parts_ms"]["osqp_setup"]["mean"]:.3f} solve '
            f'{c["parts_ms"]["osqp_solve_polish"]["mean"]:.3f} rows '
            f'{c["parts_ms"]["row_assembly"]["mean"]:.3f}')
    if not ok:
        log('CHECKS FAILED (see "checks" in the JSON)')
    return 0 if ok else 1


# ------------------------------------------------------------------ compare
def gated_metrics(run):
    """{metric name: value} the reproducibility gate compares."""
    out = {}
    for name, c in run['configs'].items():
        total = c['solve_mpc_step_ms']['mean']
        out[f'{name}: solve_mpc_step mean'] = total
        out[f'{name}: solve_mpc_step p95'] = c['solve_mpc_step_ms']['p95']
        for part, s in c['parts_ms'].items():
            if s['mean'] >= 0.05 * total:
                out[f'{name}: {part}'] = s['mean']
        for k in ('setup_wall', 'solve_wall'):
            out[f'{name}: osqp alone {k}'] = c['osqp_alone_ms'][k]['mean']
    for group, a in run['aggregate'].items():
        out[f'ALL {group}: solve_mpc_step mean'] = a['solve_mpc_step_ms']['mean']
        for part, s in a['parts_ms'].items():
            out[f'ALL {group}: {part}'] = s['mean']
    return out


def compare(path_a, path_b, tol):
    with open(path_a) as fh:
        a = json.load(fh)
    with open(path_b) as fh:
        b = json.load(fh)
    ma, mb = gated_metrics(a), gated_metrics(b)
    rows = []
    worst = 0.0
    for key in sorted(set(ma) & set(mb)):
        va, vb = ma[key], mb[key]
        dev = abs(vb / va - 1.0) if va > 0 else (0.0 if vb == 0 else math.inf)
        # parts under 0.05 ms are reported, never gated: timer resolution and
        # cache effects dominate there.
        gated = max(va, vb) >= 0.05
        rows.append((key, va, vb, dev, gated))
        if gated:
            worst = max(worst, dev)
    fails = [r for r in rows if r[4] and r[3] > tol]
    print(f'{"metric":60s} {"run A ms":>10s} {"run B ms":>10s} {"dev":>7s}')
    for key, va, vb, dev, gated in rows:
        flag = '' if gated else '  (not gated, < 0.05 ms)'
        if gated and dev > tol:
            flag = '  FAIL'
        print(f'{key:60s} {va:10.4f} {vb:10.4f} {dev * 100:6.2f}%{flag}')
    fa, fb = a.get('fixture') or {}, b.get('fixture') or {}
    same_fixture = fa.get('sha256') is not None and fa.get('sha256') == fb.get('sha256')
    print(f'\ngated metrics: {sum(1 for r in rows if r[4])}, worst deviation '
          f'{worst * 100:.2f} %, tolerance {tol * 100:.0f} %: '
          f'{"PASS" if not fails else "FAIL (" + str(len(fails)) + ")"}')
    print(f'fixture sha256 identical across runs: {same_fixture}')
    return rows, worst, fails, same_fixture


# ------------------------------------------------------------------ report
BEGIN, END = '<!-- BEGIN GENERATED (profile_rti.py --report) -->', '<!-- END GENERATED -->'


def _shown(path):
    path = os.path.abspath(path)
    return os.path.relpath(path, REPO) if path.startswith(REPO + os.sep) else path


def _f(v, nd=3):
    return '-' if v is None or (isinstance(v, float) and not math.isfinite(v)) else f'{v:.{nd}f}'


def _row(cells):
    return '| ' + ' | '.join(str(c) for c in cells) + ' |'


def _rule(n):
    return '|' + '---|' * n


def report(paths, md_path, tol):
    """Write the generated block of the markdown report from one or two runs."""
    runs = []
    for p in paths:
        with open(p) as fh:
            runs.append(json.load(fh))
    r = runs[0]
    m = r['meta']
    L = []
    L.append(f'_Generated from `{_shown(paths[0])}`'
             + (f' and `{_shown(paths[1])}`' if len(paths) > 1 else '')
             + f' (tables: the first). Times in ms; per tick the minimum of {m["repeats"]} '
             'replays, then mean / p95 over the ticks of a config._')
    L.append('')
    L.append('### Machine and libraries')
    L.append('')
    blas = m.get('numpy_blas') or {}
    loaded = ', '.join(f'`{x}`' for x in m['blas_libraries_loaded'])
    L += [_row(['', '']), _rule(2)]
    L.append(_row(['CPU', f'{m["cpu_model"]}, {m["cpu_count"]} CPUs, max {m["cpu_max_mhz"]} MHz, '
                          f'governor `{m["cpufreq_governor"]}`, affinity {m["affinity"]}']))
    L.append(_row(['Load average (start / end)',
                   ', '.join(_f(x, 2) for x in m['loadavg_start']) + ' / '
                   + ', '.join(_f(x, 2) for x in m['loadavg_end'])]))
    L.append(_row(['Python', f'{m["python"]} ({m["platform"]}), {r["micro"]["sys_path_entries"]} '
                             'sys.path entries']))
    L.append(_row(['numpy / scipy', f'{m["numpy"]} / {m["scipy"]} (Ubuntu packages)']))
    L.append(_row(['BLAS', f'numpy built against `{blas.get("name")}` '
                           f'({blas.get("detection method")}); loaded {loaded}: reference '
                           f'BLAS/LAPACK, libblas3 {m.get("dpkg_libblas3")}']))
    L.append(_row(['OSQP', f'osqp-python {m["osqp_python"]} (pip), OSQP C {m["osqp_c"]} '
                           f'(float64 {m.get("osqp_c_float64")}, int64 indices '
                           f'{m.get("osqp_c_int64")}), QDLDL direct solver']))
    L.append(_row(['Source', f'git {m["git_head"]}; mpc_solver.py sha256 '
                             f'`{m["sources"]["mpc_solver.py"]["sha256"][:16]}`']))
    L.append('')
    L.append('### Per config: totals and QP size')
    L.append('')
    L.append(_row(['config', 'ticks', 'n', 'm', 'nnz P (upper)', 'nnz A', 'iter p50 / max',
                   'status (count)', 'closed loop p50 / p95',
                   'solve_mpc_step mean / p50 / p95']))
    L.append(_rule(10))
    for name, c in r['configs'].items():
        st = ', '.join(f'{k}: {v}' for k, v in sorted(c['status_counts'].items()))
        q, sm = c['qp'], c['solve_mpc_step_ms']
        L.append(_row([
            name, c['ticks'], f'{q["n"]["max"]:.0f}', f'{q["m"]["max"]:.0f}',
            f'{q["nnz_P"]["min"]:.0f}-{q["nnz_P"]["max"]:.0f}',
            f'{q["nnz_A"]["min"]:.0f}-{q["nnz_A"]["max"]:.0f}',
            f'{c["iterations"]["p50"]:.0f} / {c["iterations"]["max"]:.0f}', st,
            f'{_f(c["closed_loop_ms"]["p50"], 2)} / {_f(c["closed_loop_ms"]["p95"], 2)}',
            f'{_f(sm["mean"], 2)} / {_f(sm["p50"], 2)} / {_f(sm["p95"], 2)}']))
    for g, a in r['aggregate'].items():
        sm = a['solve_mpc_step_ms']
        L.append(_row([f'**all {g}**', a['ticks'], '', '', '', '', '', '', '',
                       f'{_f(sm["mean"], 2)} / {_f(sm["p50"], 2)} / {_f(sm["p95"], 2)}']))
    L.append('')
    L.append('OSQP status: ' + ', '.join(
        f'{v} {k.replace("OSQP_", "").lower()}' for k, v in r['osqp']['status_enum'].items()
        if v <= 7) + '. "closed loop" is the single timed call inside the rig; '
        '"solve_mpc_step" the uninstrumented replay of the same tick.')
    L.append('')
    L.append('### Per config: where the time goes (mean ms per tick, share of the sum)')
    L.append('')
    part_names = [p for p, _ in PARTS]
    L.append(_row(['config'] + [p.replace('_', ' ') for p in part_names]
                  + ['sum of parts', 'uninstrumented']))
    L.append(_rule(len(part_names) + 3))
    rows_src = (list(r['configs'].items())
                + [(f'**all {g}**', a) for g, a in r['aggregate'].items()])
    for name, c in rows_src:
        total = sum(c['parts_ms'][p]['mean'] for p in part_names)
        cells = [f'{c["parts_ms"][p]["mean"]:.3f} ({100 * c["parts_ms"][p]["mean"] / total:.0f} %)'
                 for p in part_names]
        L.append(_row([name] + cells + [f'{total:.3f}',
                                        f'{c["solve_mpc_step_ms"]["mean"]:.3f}']))
    L.append('')
    L.append('Parts, as segments of the instrumented copy: '
             + '; '.join(f'**{p.replace("_", " ")}** = {" + ".join(s)}' for p, s in PARTS)
             + '. obstacle_lin and corridor_query are the calls made from the stage loop, '
             'taken out of stage_terms; planner_cost_corridor and the rollout include their '
             'own model steps and corridor queries.')
    L.append('')
    L.append('### By segment (all matrix ticks)')
    L.append('')
    a = r['aggregate']['matrix']
    L += [_row(['segment', 'mean', 'p95']), _rule(3)]
    for s in ROW_ASSEMBLY_SEGMENTS + ('model_ref', 'model_jac', 'obstacle_lin',
                                      'corridor_query', 'rollout', 'true_cost', 'csc',
                                      'osqp_ctor', 'osqp_setup_call', 'osqp_warm_start',
                                      'osqp_solve'):
        if s in a['segments_ms']:
            seg = a['segments_ms'][s]
            L.append(_row([s, f'{seg["mean"]:.3f}', f'{seg["p95"]:.3f}']))
    L.append('')
    L.append("### OSQP alone, on each tick's captured QP (mean ms per tick)")
    L.append('')
    L.append(_row(['config', 'OSQP()', "OSQP(algebra='builtin')", 'setup()',
                   'of which py conversion', 'C setup', 'warm_start', 'solve()', 'C solve',
                   'C polish', 'x bit-identical']))
    L.append(_rule(11))
    for name, c in list(r['configs'].items()) + [(f'**all {g}**', a)
                                                 for g, a in r['aggregate'].items()]:
        o = c['osqp_alone_ms']
        ident = (f'{c["osqp_alone_x_identical"]}/{c["ticks"]}'
                 if 'osqp_alone_x_identical' in c else '')
        L.append(_row([name] + [f'{o[k]["mean"]:.3f}' for k in (
            'ctor_wall', 'ctor_builtin_wall', 'setup_call_wall', 'py_convert', 'c_setup',
            'warm_start_wall', 'solve_wall', 'c_solve', 'c_polish')] + [ident]))
    L.append('')
    mi = r['micro']
    L.append(f'Micro: `osqp.OSQP()` {mi["osqp_ctor_default_us"]:.0f} us against '
             f'{mi["osqp_ctor_builtin_us"]:.1f} us with `algebra=\'builtin\'` (the default '
             'constructor tries to import the cuda and mkl backends first, and a failed import '
             f'walks all {mi["sys_path_entries"]} sys.path entries, every tick); one '
             f'`np.pad` pair (4x4 -> 5x5 obstacle model, offset_free only) '
             f'{mi["np_pad_pair_us"]:.1f} us, 20 per tick.')
    L.append('')
    L.append('### Persistent OSQP workspace '
             '(setup once per run, then update + warm_start + solve)')
    L.append('')
    L.append("Pattern: the union of every tick's sparsity in the run, explicit zeros kept. "
             '"fresh" is today\'s OSQP() + setup + warm_start + solve. dx is the largest '
             'absolute difference of the full OSQP solution x from the fresh solve.')
    L.append('')
    L.append(_row(['config', 'fresh', 'update()', 'rho carried: update + solve',
                   'iter change p50 / min / max', 'status changed', 'max abs dx p50 / max',
                   'rho reset: status changed', 'max abs dx p50',
                   'rho reset + y = 0: update + solve', 'iter change min / max',
                   'status changed', 'max abs dx max']))
    L.append(_rule(13))
    for name, c in r['configs'].items():
        p = c.get('persistent') or {}
        if not p:
            continue
        o = c['osqp_alone_ms']
        fresh = o['setup_wall']['mean'] + o['warm_start_wall']['mean'] + o['solve_wall']['mean']
        cr, rr, ra = p['carry_rho'], p['reset_rho'], p.get('reset_all')
        cells = [
            name, f'{fresh:.3f}', f'{cr["update_wall"]["mean"]:.3f}',
            f'{cr["total_wall"]["mean"]:.3f}',
            f'{cr["iter_delta"]["p50"]:+.0f} / {cr["iter_delta"]["min"]:+.0f} / '
            f'{cr["iter_delta"]["max"]:+.0f}',
            f'{cr["status_changed"]}/{p["ticks"]}',
            f'{cr["max_abs_dx"]["p50"]:.1e} / {cr["max_abs_dx"]["max"]:.1e}',
            f'{rr["status_changed"]}/{p["ticks"]}', f'{rr["max_abs_dx"]["p50"]:.1e}']
        if ra:
            cells += [f'{ra["total_wall"]["mean"]:.3f}',
                      f'{ra["iter_delta"]["min"]:+.0f} / {ra["iter_delta"]["max"]:+.0f}',
                      f'{ra["status_changed"]}/{p["ticks"]}', f'{ra["max_abs_dx"]["max"]:.1e}']
        else:
            cells += ['-'] * 4
        L.append(_row(cells))
    L.append('')
    L.append('### OSQP settings the Python path runs with (`prob.settings` after setup)')
    L.append('')
    kw = ', '.join(f'{k}={v}' for k, v in r['osqp']['setup_kwargs'].items())
    L.append(f'`_solve_rti` calls `osqp.OSQP().setup(P, q, A, l, u, {kw})`, then '
             '`warm_start(x=z_guess)` (x only, no y) and `solve(raise_error=False)`; '
             "everything else is OSQP's default. Identical on every tick: "
             f'{r["osqp"]["settings_constant_over_ticks"]}.')
    L.append('')
    L += [_row(['setting', 'value']), _rule(2)]
    for k, v in sorted(r['osqp']['effective_settings'].items()):
        L.append(_row([f'`{k}`', f'`{v}`']))
    L.append('')
    L.append('Before OSQP C: osqp-python keeps only the upper triangle of P '
             '(`scipy.sparse.triu` of the symmetric CSC `_solve_rti` passes; zeros already '
             'dropped by `csc_matrix`, indices sorted) and clamps l/u to +-OSQP_INFTY = '
             f'{r["osqp"]["infty"]:.0e}. linsys_solver enum: {r["osqp"]["linsys_solver_enum"]}. '
             'Statuses `_solve_rti` treats as an infeasibility certificate (success False, '
             f'warm start held): {r["osqp"]["infeasible_statuses"]}.')
    L.append('')
    L.append('### Checks')
    L.append('')
    ch = r['checks']
    L.append(f'- replay of all {ch["ticks"]} ticks from the fixture encoding bit-identical to the '
             f'closed-loop solve: {ch["replay_identical"]}/{ch["ticks"]}; instrumented copy '
             f'bit-identical: {ch["instrumented_identical"]}/{ch["ticks"]}; still identical '
             f'after the timing replays: {ch["post_timing_identical"]}.')
    fx = r.get('fixture')
    if fx:
        L.append(f'- fixture `{fx["path"]}`: {fx["count"]} ticks ({fx["groups"]}), '
                 f'{fx["bytes"] / 1e6:.2f} MB, sha256 `{fx["sha256"]}`; read back and solved '
                 f'from the file, bit-identical: {fx["reread_identical"]}.')
    if len(runs) > 1:
        rows, worst, fails, same_fixture = compare(paths[0], paths[1], tol)
        n_gated = sum(1 for x in rows if x[4])
        L.append(f'- reproducibility (`--compare`): {n_gated} gated metrics, worst deviation '
                 f'{worst * 100:.2f} % (tolerance {tol * 100:.0f} %): '
                 f'**{"PASS" if not fails else "FAIL"}**; fixture sha256 identical across '
                 f'runs: {same_fixture}.')
        L.append('')
        L += [_row(['metric', 'run 1', 'run 2', 'deviation']), _rule(4)]
        for key, va, vb, dev, gated in rows:
            if gated and ('solve_mpc_step mean' in key or key.startswith('ALL')):
                L.append(_row([key, f'{va:.3f}', f'{vb:.3f}', f'{dev * 100:.2f} %']))
        worst_rows = sorted((x for x in rows if x[4]), key=lambda x: -x[3])[:5]
        L.append('')
        L.append('Largest deviations: ' + '; '.join(
            f'{k} {va:.3f} -> {vb:.3f} ({d * 100:.1f} %)' for k, va, vb, d, _ in worst_rows)
            + '.')
    block = '\n'.join(L)
    if os.path.exists(md_path):
        with open(md_path) as fh:
            text = fh.read()
        if BEGIN in text and END in text:
            head, rest = text.split(BEGIN, 1)
            _, tail = rest.split(END, 1)
            text = head + BEGIN + '\n' + block + '\n' + END + tail
        else:
            text = text.rstrip('\n') + '\n\n' + BEGIN + '\n' + block + '\n' + END + '\n'
    else:
        text = ('# MPC RTI solve: per-part profile (R2.0)\n\n' + BEGIN + '\n' + block + '\n'
                + END + '\n')
    os.makedirs(os.path.dirname(os.path.abspath(md_path)), exist_ok=True)
    with open(md_path, 'w') as fh:
        fh.write(text)
    log(f'wrote {md_path}')
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--repeats', type=int, default=7)
    ap.add_argument('--out', default=DEFAULT_OUT)
    ap.add_argument('--fixture', default=DEFAULT_FIXTURE)
    ap.add_argument('--no-fixture', action='store_true')
    ap.add_argument('--cpu', type=int, default=None, help='pin to this CPU')
    ap.add_argument('--quick', action='store_true', help='short runs, for development')
    ap.add_argument('--compare', nargs=2, metavar=('RUN1', 'RUN2'))
    ap.add_argument('--tolerance', type=float, default=0.10)
    ap.add_argument('--report', nargs='+', metavar='RUN')
    ap.add_argument('--md', default=os.path.join(REPO, 'output', 'rust', 'mpc', 'rti_profile.md'))
    args = ap.parse_args(argv)
    if args.compare:
        _, _, fails, _ = compare(args.compare[0], args.compare[1], args.tolerance)
        return 1 if fails else 0
    if args.report:
        return report(args.report, args.md, args.tolerance)
    return run_profile(args)


if __name__ == '__main__':
    sys.exit(main())
