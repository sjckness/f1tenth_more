#!/usr/bin/env python3
"""
Deterministic micro-benchmarks of the stack's pure functions (perf baseline, level A).

Measure only: this file imports production modules and calls them; it changes
none of them. Inputs are fixed (seeded), come from the existing test fixtures
and harnesses where they exist and from scripts/perf/perf_scene.py otherwise.

Run (from the repo root, workspace built and sourced; on linus also
~/dev_ws/jc_env.sh for osqp):

    python3 scripts/perf/bench_micro.py --out output/perf/<ts>            # measure + golden
    python3 scripts/perf/bench_micro.py --out /tmp/x --only 'mpc.*'       # a subset
    python3 scripts/perf/bench_micro.py --check-golden output/perf/<ts>/golden
    python3 scripts/perf/bench_micro.py --list

What a run does, per case:
  1. timing: a fresh setup, --warmup calls, then --iters calls timed one by one with
     time.perf_counter_ns (p50 / p95 / max in ms);
  2. allocations: a fresh setup, 20 calls under tracemalloc, the largest per-call peak;
  3. golden: a fresh setup, the first n_golden calls, their inputs and outputs written
     to <out>/golden/<case>.npz (see GOLDEN FILES);
  4. for the --profile-top costliest cases (by cost per 100 ms control cycle): cProfile
     over 50 calls, top 10 by cumulative time, in <out>/profiles/<case>.txt.

The process re-executes itself under `taskset -c <core>` (--core, default: the
least busy core) with BLAS/OpenMP limited to one thread, so a run is one core's
worth of work and nothing else.

"calls per control cycle" is the number of calls in one 100 ms MPC period: the
rate the node calls the function at, divided by 10 Hz. For the solver's inner
functions it is the count measured while one real solve runs.

GOLDEN FILES. <case>.npz holds
  out/<i>/<path>   every numeric leaf of the i-th call's return value;
  in/<i>/<path>    the small inputs of that call (poses, seeds, parameters);
  in/pickle        (replay cases only) the pickled argument list of every call;
  meta             JSON: target, file:line, the scene arrays the case read
                   (stored once in _scene.npz), a digest of its inputs.
--check-golden rebuilds the inputs (or, for replay cases, loads them from the
file), calls the functions again and compares every leaf with numpy.allclose
(--rtol / --atol); a case whose input digest changed is reported as such and not
compared. This is the numerical-equivalence test for phase 2.
"""

import os
import sys

for _var in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ.setdefault(_var, '1')

import argparse  # noqa: E402
import array  # noqa: E402
import collections  # noqa: E402
import copy  # noqa: E402
import cProfile  # noqa: E402
import csv  # noqa: E402
import dataclasses  # noqa: E402
import enum  # noqa: E402
import fnmatch  # noqa: E402
import functools  # noqa: E402
import hashlib  # noqa: E402
import inspect  # noqa: E402
import io  # noqa: E402
import itertools  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import pathlib  # noqa: E402
import pickle  # noqa: E402
import platform  # noqa: E402
import pstats  # noqa: E402
import time  # noqa: E402
import tracemalloc  # noqa: E402
import types  # noqa: E402

REPO = pathlib.Path(__file__).resolve().parents[2]
CONTROL_HZ = 10.0
ALLOC_CALLS = 20
PROFILE_CALLS = 50


def _repin(core):
    """Re-exec under taskset on one core (once)."""
    if os.environ.get('F1T_PERF_PINNED') == '1':
        return
    if core == 'auto':
        import psutil
        busy = psutil.cpu_percent(interval=1.0, percpu=True)
        core = str(min(range(len(busy)), key=lambda c: busy[c]))
    env = dict(os.environ, F1T_PERF_PINNED='1', F1T_PERF_CORE=str(core))
    os.execvpe('taskset', ['taskset', '-c', str(core), sys.executable] + sys.argv, env)


# ----------------------------------------------------------------- flattening

def flatten(obj, prefix='', out=None, depth=0, seen=None):
    """Every numeric leaf of obj as {path: ndarray}; strings and None as arrays too."""
    import numpy as np
    out = {} if out is None else out
    seen = set() if seen is None else seen
    key = prefix or 'value'
    if depth > 8:
        return out
    if obj is None:
        out[key] = np.array(np.nan)
    elif isinstance(obj, (bool, int, float, np.generic)):
        out[key] = np.asarray(obj)
    elif isinstance(obj, str):
        out[key] = np.array(obj)
    elif isinstance(obj, enum.Enum):
        flatten(obj.value, key, out, depth + 1, seen)
    elif isinstance(obj, np.ndarray):
        out[key] = obj if obj.dtype != object else np.array(repr(obj))
    elif isinstance(obj, (array.array, bytes, bytearray)):
        out[key] = np.asarray(obj)
    elif isinstance(obj, dict):
        for k in obj:
            flatten(obj[k], f'{prefix}/{k}' if prefix else str(k), out, depth + 1, seen)
    elif isinstance(obj, (list, tuple, set, frozenset)):
        items = sorted(obj, key=repr) if isinstance(obj, (set, frozenset)) else list(obj)
        try:
            arr = np.asarray(items)
            numeric = arr.dtype != object and arr.dtype.kind in 'biuf' and arr.ndim >= 1
        except Exception:
            numeric = False
        if numeric:
            out[key] = arr
        else:
            out[f'{key}/len'] = np.asarray(len(items))
            for i, item in enumerate(items):
                flatten(item, f'{key}/{i}', out, depth + 1, seen)
    elif callable(obj) or isinstance(obj, types.ModuleType):
        pass
    else:
        if id(obj) in seen:
            return out
        seen.add(id(obj))
        if dataclasses.is_dataclass(obj):
            names = [f.name for f in dataclasses.fields(obj)]
        elif hasattr(obj, '__dict__'):
            names = [n for n in vars(obj) if not n.startswith('__')]
        elif hasattr(obj, '__slots__'):
            names = list(obj.__slots__)
        elif hasattr(obj, '_fields'):
            names = list(obj._fields)
        else:
            out[key] = np.array(repr(obj))
            return out
        for n in names:
            try:
                flatten(getattr(obj, n), f'{key}/{n.lstrip("_")}', out, depth + 1, seen)
            except Exception:
                pass
    return out


def digest(arrays):
    import numpy as np
    h = hashlib.sha256()
    for k in sorted(arrays):
        a = np.ascontiguousarray(arrays[k])
        h.update(k.encode())
        h.update(str(a.dtype).encode())
        h.update(a.tobytes() if a.dtype.kind != 'U' else str(a).encode())
    return h.hexdigest()[:16]


def capture_calls(module, name, during, limit=4000):
    """Arguments of every call of module.name made while during() runs."""
    orig = getattr(module, name)
    calls = []

    @functools.wraps(orig)
    def spy(*args, **kwargs):
        if len(calls) < limit:
            calls.append((copy.deepcopy(args), copy.deepcopy(kwargs)))
        return orig(*args, **kwargs)

    setattr(module, name, spy)
    try:
        during()
    finally:
        setattr(module, name, orig)
    return calls


# ----------------------------------------------------------------- the cases

CASES = []


class Case:
    """One benchmarked call. Subclasses fill name, target, rate_hz and run()."""

    name = ''
    target = None          # the function whose file:line the row reports
    rate_hz = CONTROL_HZ   # how often production calls it
    calls_per_cycle = None  # overrides rate_hz / CONTROL_HZ when set
    note = ''
    in_default_stack = True
    n_golden = 8
    replay = False         # True: golden stores the pickled arguments and the check replays them
    scene_refs = ()

    def setup(self):
        pass

    def prepare(self, i):
        pass

    def run(self, i):
        raise NotImplementedError

    def inputs(self, i):
        return {}

    def summarise(self, out):
        return flatten(out)

    def per_cycle(self):
        return self.rate_hz / CONTROL_HZ if self.calls_per_cycle is None else self.calls_per_cycle

    def where(self):
        fn = inspect.unwrap(self.target)
        path = pathlib.Path(os.path.realpath(inspect.getsourcefile(fn)))
        try:
            path = path.relative_to(REPO)
        except ValueError:
            pass
        return f'{path}:{inspect.getsourcelines(fn)[1]}'


def register(cls):
    CASES.append(cls)
    return cls


class Scene:
    """perf_scene's arrays, built once per process and shared by the cases."""

    K_IMAGE = 4
    K_SCAN = 8
    N_DRIVE = 240

    def __init__(self):
        import perf_scene as ps
        self.ps = ps
        self._cache = {}

    def _get(self, key, make):
        if key not in self._cache:
            self._cache[key] = make()
        return self._cache[key]

    def pose(self, i):
        return self.ps.car_pose(i % self.K_SCAN)

    def scan(self, i):
        k = i % self.K_SCAN
        return self._get(('scan', k), lambda: self.ps.scan(self.ps.car_pose(k), seed=k))

    def frame(self, i):
        """(depth, instance mask, pose index) of image variation i."""
        k = i % self.K_IMAGE
        return self._get(('frame', k),
                         lambda: self.ps.depth_and_masks(self.ps.car_pose(k), seed=k)) + (k,)

    def grid(self):
        return self._get('grid', self.ps.occupancy_grid)

    def drive_scan(self, i):
        k = i % self.N_DRIVE
        return self._get(('drive', k), lambda: self.ps.scan(self.ps.drive_pose(k), seed=1000 + k))

    def warm(self, drive=False):
        """Build every array now, so that none is ever generated inside a timed call."""
        self.grid()
        for k in range(self.K_SCAN):
            self.scan(k)
        for k in range(self.K_IMAGE):
            self.frame(k)
        if drive:
            for k in range(self.N_DRIVE):
                self.drive_scan(k)

    def arrays(self):
        """The shared arrays for golden/_scene.npz."""
        import numpy as np
        out = {'grid': np.asarray(self.grid(), dtype=np.int8)}
        for k in range(self.K_SCAN):
            out[f'scan/{k}'] = self.scan(k)
            out[f'pose/{k}'] = np.asarray(self.pose(k))
        for k in range(self.K_IMAGE):
            depth, mask, _ = self.frame(k)
            out[f'depth/{k}'], out[f'mask/{k}'] = depth, mask
        return out


SCENE = None


def scene():
    global SCENE
    if SCENE is None:
        SCENE = Scene()
    return SCENE


def _add_test_paths():
    for sub in ('scripts/perf', 'src/f1tenth_control/mpc_controller/test',
                'src/f1tenth_perception/test', 'src/f1tenth_costmap/test'):
        p = str(REPO / sub)
        if p not in sys.path:
            sys.path.insert(0, p)


# ---- MPC ---------------------------------------------------------------------

class _Mpc:
    """Solver inputs captured from the shipped closed-loop harness, built once."""

    _built = None

    @classmethod
    def get(cls):
        if cls._built is None:
            cls._built = cls._build()
        return cls._built

    @staticmethod
    def _build():
        import numpy as np
        from mpc_controller import mpc_solver
        import reverse_harness as rh
        import speed_channel_harness as sch
        walls = [(0.0, 1.0, 1.0), (0.0, -1.0, 1.0)]
        obstacles = [(4.0, 0.45, 0.15), (8.0, -0.45, 0.20)]
        calls = []
        orig = sch.solve_mpc_step

        def spy(**kw):
            kw = dict(kw)
            corridor = dict(kw['corridor'])
            corridor['obstacles_world'] = list(obstacles)
            kw.update(corridor=corridor, obstacles=list(obstacles), boundaries=list(walls),
                      boundary_max_sources=3, boundary_slack_weight=rh.SLACK_W)
            calls.append(copy.deepcopy(kw))
            return orig(**kw)

        sch.solve_mpc_step = spy
        try:
            sch.run(mode='offset_free', vdes=0.5, duration=26.0, seed=0)
        finally:
            sch.solve_mpc_step = orig
        inner = {}
        probe = calls[60]
        for fn in ('_linearize_obstacle_cost_rti', '_rollout_true_dynamics', '_model_jacobian',
                   '_model_step', 'planner_cost_corridor', '_nearest_corridor_frame',
                   'corridor_heading_at', 'nearest_centerline_index'):
            if hasattr(mpc_solver, fn):
                inner[fn] = capture_calls(
                    mpc_solver, fn, lambda: mpc_solver.solve_mpc_step(**copy.deepcopy(probe)))
        return types.SimpleNamespace(calls=calls, inner=inner, np=np)


def _solve_inputs(kw):
    import numpy as np
    return {'x0': np.asarray(kw['x0'], float), 'last_u': np.asarray(kw['last_u'], float),
            'pref_nom': np.asarray(kw['pref_nom'], float), 'vdes': np.asarray(kw['vdes']),
            'warm': np.zeros(0) if kw.get('warm_start_z') is None
            else np.asarray(kw['warm_start_z'], float)}


def _solve_summary(out):
    u0, info = out
    keep = ('zopt', 'x_pred', 'success', 'true_cost', 'cost', 'v_cmd_pred', 'boundary_slack')
    return flatten({'u0': u0, **{k: info[k] for k in keep if k in info}})


@register
class MpcSolveForward(Case):
    name = 'mpc.solve_mpc_step[forward rti N=20, offset-free, 2 walls, 2 obstacles]'
    note = ('closed-loop inputs captured from test/speed_channel_harness.py with wall rows and '
            'two obstacles added; the QP mpc_corr solves every tick')
    n_golden = 40
    replay = True

    def setup(self):
        from mpc_controller import mpc_solver
        self.target = mpc_solver.solve_mpc_step
        self.fn = mpc_solver.solve_mpc_step
        self.calls = _Mpc.get().calls

    def args(self, i):
        return copy.deepcopy(self.calls[i % len(self.calls)])

    def prepare(self, i):
        self._kw = self.args(i)

    def run(self, i):
        return self.fn(**self._kw)

    def inputs(self, i):
        return _solve_inputs(self.calls[i % len(self.calls)])

    summarise = staticmethod(_solve_summary)


@register
class MpcSolveRear(Case):
    name = 'mpc.RearSolver.solve[reverse QP]'
    note = ('test/bench_reverse_manoeuvre.py "rear_alone"; runs only while the forward solve '
            'is stuck (reverse_solver_mode lazy)')
    calls_per_cycle = 0.0

    def setup(self):
        import numpy as np
        from f1tenth_params.object_geometry import SOLVER_FRONT_POINT_OFFSETS_M
        from mpc_controller.reverse_manoeuvre import rear_corridor, RearSolver
        from mpc_controller.reverse_mpc import rear_probe_offsets
        import reverse_harness as rh
        import reverse_manoeuvre_harness as hm
        self.rh, self.rear_corridor = rh, rear_corridor
        self.cfg = hm.shipped_manoeuvre_config()
        off = rear_probe_offsets(rh.WHEELBASE, rh.FRONT_X, rh.REAR_X, SOLVER_FRONT_POINT_OFFSETS_M)
        self.rear = RearSolver(self.cfg, hm.rear_weights(), rh.LIMITS, rh.PARAMS, off,
                               boundary_opts={'boundary_max_sources': 3,
                                              'boundary_slack_weight': rh.SLACK_W})
        self.target = RearSolver.solve
        rng = np.random.default_rng(20261020)
        n = 256
        self.states = np.column_stack([rng.uniform(0.0, 3.0, n), rng.uniform(-0.1, 0.1, n),
                                       rng.uniform(-0.1, 0.1, n), rng.uniform(0.0, 0.5, n)])
        self.walls = [(0.0, 1.0, 0.5), (0.0, -1.0, 0.5)]

    def prepare(self, i):
        x = self.states[i % len(self.states)]
        self._x = x
        self._corridor = self.rear_corridor(x[0], x[1], x[2], self.cfg)

    def run(self, i):
        rh = self.rh
        return self.rear.solve(self._x, 0.0, self._corridor, [], self.walls, rh.CAR_RADIUS,
                               rh.AVOIDANCE_MARGIN, rh.TS)

    def inputs(self, i):
        return {'x': self.states[i % len(self.states)]}

    def summarise(self, out):
        steer, info, ok, _seconds = out
        return flatten({'steer': steer, 'ok': ok,
                        'info': {k: info[k] for k in ('zopt', 'x_pred', 'success') if k in info}})


class _MpcInner(Case):
    """An inner function of the solver, replayed with the arguments of one real solve."""

    fn_name = ''

    def setup(self):
        from mpc_controller import mpc_solver
        self.target = getattr(mpc_solver, self.fn_name)
        self.fn = self.target
        self.calls = _Mpc.get().inner[self.fn_name]
        self.calls_per_cycle = float(len(self.calls))

    def prepare(self, i):
        a, k = self.calls[i % len(self.calls)]
        self._a, self._k = copy.deepcopy(a), copy.deepcopy(k)

    def run(self, i):
        return self.fn(*self._a, **self._k)

    def inputs(self, i):
        a, _k = self.calls[i % len(self.calls)]
        return flatten([x for x in a if not isinstance(x, dict)], 'args')


for _fn, _label in (('_linearize_obstacle_cost_rti', 'RTI obstacle-cost linearisation'),
                    ('_model_jacobian', 'RTI dynamics Jacobian'),
                    ('_model_step', 'nonlinear model step'),
                    ('_rollout_true_dynamics', 'true-dynamics rollout'),
                    ('_nearest_corridor_frame', 'corridor frame lookup'),
                    ('planner_cost_corridor', 'true cost of the solution')):
    register(type(f'MpcInner{_fn}', (_MpcInner,), {
        'name': f'mpc.{_fn}', 'fn_name': _fn,
        'note': f'{_label}; arguments and call count captured from one real forward solve'}))


@register
class MpcScaleWeights(Case):
    name = 'mpc.scale_stage_weights'
    calls_per_cycle = 1.0

    def setup(self):
        from mpc_controller import mpc_solver
        import reverse_harness as rh
        self.target = self.fn = mpc_solver.scale_stage_weights
        self.w = dict(rh.WEIGHTS)

    def run(self, i):
        return self.fn(self.w, 20)


class _Corridor(Case):
    def setup(self):
        import numpy as np
        from mpc_controller.MPC_corr import MPCController
        import reverse_harness as rh
        self.rh, self.ctl = rh, MPCController
        self.fake = rh._Stand()
        self.fake.goal_distance = 6.0
        self.fake.goal_start_xy = (0.0, 0.0)
        self.fake.goal_anchor_odom = (0.0, 0.0, 0.0)
        self.fake.psi_init_corridor = 0.0
        rng = np.random.default_rng(20261020)
        n = 256
        self.states = np.column_stack([rng.uniform(0.0, 3.0, n), rng.uniform(-0.1, 0.1, n),
                                       rng.uniform(-0.1, 0.1, n), rng.uniform(0.0, 0.5, n)])

    def state(self, i):
        return self.states[i % len(self.states)]

    def build(self, x):
        rh = self.rh
        self.fake.smoothed_target = None
        corridor = self.ctl.build_straight_corridor(self.fake, x)
        corridor['obstacles_world'] = []
        corridor['d_safe'] = rh.CAR_RADIUS + rh.AVOIDANCE_MARGIN
        corridor['car_radius'] = rh.CAR_RADIUS
        corridor['avoidance_margin'] = rh.AVOIDANCE_MARGIN
        return corridor

    def inputs(self, i):
        return {'x': self.state(i)}


@register
class MpcBuildCorridor(_Corridor):
    name = 'mpc.MPCController.build_straight_corridor[goal_distance]'
    rate_hz = 1.0
    note = 'rebuilt every corridor_update_period (1 s); stand-in node of test/reverse_harness.py'

    def setup(self):
        super().setup()
        self.target = self.ctl.build_straight_corridor

    def prepare(self, i):
        self.fake.smoothed_target = None

    def run(self, i):
        return self.ctl.build_straight_corridor(self.fake, self.state(i))

    def summarise(self, out):
        return flatten({k: v for k, v in out.items() if k != 't'})


@register
class MpcLocalTarget(_Corridor):
    name = 'mpc.MPCController.compute_local_target'
    note = 'every tick, on a corridor that changes once a second'

    def setup(self):
        super().setup()
        self.target = self.ctl.compute_local_target

    def prepare(self, i):
        self._c = self.build(self.state(i))
        self.fake.smoothed_target = None

    def run(self, i):
        return self.ctl.compute_local_target(self.fake, self.state(i), self._c)


# ---- LiDAR: walls, doors -------------------------------------------------------

class _ScanCase(Case):
    rate_hz = 40.0
    scene_refs = ('scan', 'pose')

    def setup(self):
        self.s = scene()
        self.ps = self.s.ps
        self.s.warm()

    def inputs(self, i):
        import numpy as np
        return {'scan_index': np.asarray(i % Scene.K_SCAN)}


@register
class ScenePointsCase(_ScanCase):
    name = 'walls.scene_labels.scan_points'
    note = 'lidar_front_wall_node._scene_fit, every scan: bearings, sin/cos, arctan2 per beam'

    def setup(self):
        super().setup()
        from f1tenth_perception import scene_labels
        self.target = self.fn = scene_labels.scan_points

    def run(self, i):
        ps = self.ps
        return self.fn(self.s.scan(i), ps.ANGLE_MIN, ps.ANGLE_INC, ps.RANGE_MIN, ps.RANGE_MAX,
                       ps.LASER, math.radians(60.0), (), with_bearings=True)


@register
class SequentialLinesCase(_ScanCase):
    name = 'walls.scene_labels.sequential_lines+heading_wall'
    rate_hz = 10.0
    note = 'wall planes at wall_planes_rate_hz (10 Hz), every other beam of the forward sector'

    def setup(self):
        super().setup()
        from f1tenth_perception import scene_labels
        self.sl = scene_labels
        self.target = scene_labels.sequential_lines

    def prepare(self, i):
        ps = self.ps
        self._pts, _u, _x, self._b = self.sl.scan_points(
            self.s.scan(i), ps.ANGLE_MIN, ps.ANGLE_INC, ps.RANGE_MIN, ps.RANGE_MAX, ps.LASER,
            math.radians(60.0), (), with_bearings=True)

    def run(self, i):
        lines = self.sl.sequential_lines(self._pts[::2], 0.03, 0.8, 15, 3)
        return lines, self.sl.heading_wall(lines, math.radians(45.0), self._pts, self._b)


@register
class RefitWallCase(_ScanCase):
    name = 'walls.scene_labels.refit_wall+wall_ahead'
    note = 'refit of the tracked wall on the full-resolution returns, every scan'

    def setup(self):
        super().setup()
        from f1tenth_perception import scene_labels
        self.sl = scene_labels
        self.target = scene_labels.refit_wall

    def prepare(self, i):
        ps = self.ps
        self._pts, _u, _x, self._b = self.sl.scan_points(
            self.s.scan(i), ps.ANGLE_MIN, ps.ANGLE_INC, ps.RANGE_MIN, ps.RANGE_MAX, ps.LASER,
            math.radians(60.0), (), with_bearings=True)
        lines = self.sl.sequential_lines(self._pts[::2], 0.03, 0.8, 15, 3)
        self._wall = self.sl.heading_wall(lines, math.radians(45.0), self._pts, self._b)
        if self._wall is None and lines:
            # With the tracker on the node refits the tracker's front wall; here: the
            # most frontal of the scan's own lines.
            self._wall = min(lines, key=lambda w: abs(w.normal_angle))

    def run(self, i):
        wall = self.sl.refit_wall(self._pts, self._wall, 0.03, 0.09)
        return wall, (None if wall is None else self.sl.wall_ahead(wall, self._pts, self._b))


@register
class FitForwardLineCase(_ScanCase):
    name = 'walls.lidar_front_wall.fit_forward_line'
    in_default_stack = False
    note = 'the pre-scene-labels sector fit; only with enable_scene_labels false'

    def setup(self):
        super().setup()
        from f1tenth_perception import lidar_front_wall
        self.target = self.fn = lidar_front_wall.fit_forward_line

    def run(self, i):
        ps = self.ps
        return self.fn(self.s.scan(i), ps.ANGLE_MIN, ps.ANGLE_INC, ps.RANGE_MIN, ps.RANGE_MAX,
                       ps.LASER, math.radians(10.0), 0.03, 800)


class _Extract(_ScanCase):
    rate_hz = 10.0
    backend = 'c'

    def setup(self):
        super().setup()
        from f1tenth_perception import multi_wall_tracker as mwt
        self.mwt = mwt
        self.target = mwt.extract_lines
        self.params = mwt.extraction_params(backend=self.backend)

    def run(self, i):
        ps = self.ps
        return self.mwt.extract_lines(self.s.scan(i), ps.ANGLE_MIN, ps.ANGLE_INC, 0.06,
                                      ps.RANGE_MAX, ps.LASER, (), params=self.params)


@register
class ExtractC(_Extract):
    name = 'walls.multi_wall_tracker.extract_lines[backend=c]'
    note = 'wall proposal from the scan, one scan in four (tracker decimation); the default backend'


@register
class ExtractPython(_Extract):
    name = 'walls.multi_wall_tracker.extract_lines[backend=python]'
    backend = 'python'
    in_default_stack = False
    note = 'reference implementation; used only if libextract_lines.so is missing'


class _Drive(Case):
    """A slow drive down the corridor: one scan, its lines and observations per step."""

    rate_hz = 10.0
    n_golden = 40

    def setup(self):
        from corridor_perception.geometry import Pose2D
        from f1tenth_perception import multi_wall_tracker as mwt
        import test_multi_wall_tracker as tmt
        self.s, self.mwt, self.tmt, self.Pose2D = scene(), mwt, tmt, Pose2D
        self.ps = self.s.ps
        self.s.warm(drive=True)
        self.tracker = mwt.MultiWallTracker()
        self.params = mwt.extraction_params()

    def step_inputs(self, i):
        ps = self.ps
        ranges = self.s.drive_scan(i)
        car = self.Pose2D(*ps.drive_pose(i % Scene.N_DRIVE))
        lines = self.mwt.extract_lines(ranges, ps.ANGLE_MIN, ps.ANGLE_INC, 0.06, ps.RANGE_MAX,
                                       ps.LASER, (), params=self.params)
        return car, self.tmt.observe(lines), self.tmt.scan_of(ranges), 0.1 * i

    def track(self, car, obs, scan, t):
        self.tracker.laser_xy = self.ps.LASER[:2]
        self.tracker.step(obs, t, car, scan=scan, defer_roles=True)
        self.tracker.update_roles(car)
        return self.tracker.published(car), self.tracker.front_track(car, 0.405)

    def inputs(self, i):
        import numpy as np
        return {'step': np.asarray(i)}


def _track_table(tracks):
    import numpy as np
    rows = [[t.id, float(t.x[0]), float(t.x[1]), float(t.t0), float(t.t1)] for t in tracks]
    return np.asarray(sorted(rows), dtype=float).reshape(-1, 5)


@register
class TrackerStep(_Drive):
    name = 'walls.MultiWallTracker.step+update_roles+published'
    note = 'one tracker cycle on the observations of one scan (extraction and fusion not included)'

    def setup(self):
        super().setup()
        self.target = self.mwt.MultiWallTracker.step

    def prepare(self, i):
        self._in = self.step_inputs(i)

    def run(self, i):
        return self.track(*self._in)

    def summarise(self, out):
        tracks, front = out
        return {'tracks': _track_table(tracks),
                'front_id': flatten(None if front is None else front.id)['value']}


@register
class DoorCycle(_Drive):
    name = 'doors.door_candidates+DoorTracker.step'
    in_default_stack = False
    note = ('one door cycle on the tracker state of the same drive; lidar door detector is off '
            'by default on this branch (door_source freespace)')

    def setup(self):
        super().setup()
        from f1tenth_perception import door_detection as dd
        self.dd = dd
        self.target = dd.door_candidates
        self.p = dd.DoorParams()
        self.prior = dd.ManhattanPrior(self.p)
        self.doors = dd.DoorTracker(self.p)

    def prepare(self, i):
        car, obs, scan, t = self.step_inputs(i)
        self.track(car, obs, scan, t)
        self._in = (car, scan, t)

    def run(self, i):
        car, scan, t = self._in
        self.prior.update(self.tracker.tracks, t)
        cands = self.dd.door_candidates(self.tracker.tracks, car, scan, self.p,
                                        stereo=None, manhattan=self.prior)
        return cands, self.doors.step(cands, car, t, (), scan=scan)

    def summarise(self, out):
        import numpy as np
        cands, doors = out
        return {'n_candidates': np.asarray(len(cands)),
                'codes': np.array(','.join(sorted(c.code for c in cands))),
                'widths': np.asarray(sorted(float(c.width) for c in cands)),
                'n_doors': np.asarray(len(doors))}


# ---- camera: scene mask, labels, fusion ------------------------------------------

class _Frame(Case):
    rate_hz = 5.0
    scene_refs = ('depth', 'mask', 'pose', 'scan')
    n_golden = 4

    def setup(self):
        self.s = scene()
        self.ps = self.s.ps
        self.s.warm()
        ps = self.ps
        self.k = (ps.FX, ps.FY, ps.CX, ps.CY)

    def classify(self, i):
        from f1tenth_perception import scene_mask
        ps = self.ps
        depth, mask, _k = self.s.frame(i)
        labels, _z = scene_mask.classify(depth, *self.k, ps.ROT, ps.TRANS, self_mask=None,
                                         object_mask=mask > 0, floor_tol=0.05, ceiling_min_z=2.0,
                                         min_depth=0.2, max_depth=20.0)
        return depth, mask, labels

    def inputs(self, i):
        import numpy as np
        return {'frame_index': np.asarray(i % Scene.K_IMAGE)}


@register
class SceneMaskClassify(_Frame):
    name = 'camera.scene_mask.classify'
    note = 'whole-frame depth classification, scene mask tick (5 Hz)'

    def setup(self):
        super().setup()
        from f1tenth_perception import scene_mask
        self.target = self.fn = scene_mask.classify

    def run(self, i):
        ps = self.ps
        depth, mask, _k = self.s.frame(i)
        return self.fn(depth, *self.k, ps.ROT, ps.TRANS, self_mask=None, object_mask=mask > 0,
                       floor_tol=0.05, ceiling_min_z=2.0, min_depth=0.2, max_depth=20.0)


@register
class InferredFloorLut(_Frame):
    name = 'camera.inferred_floor.build_lut'
    calls_per_cycle = 0.0
    n_golden = 1
    note = 'cached by the node; rebuilt only when the image size, intrinsics or mount change'

    def setup(self):
        super().setup()
        from f1tenth_perception import inferred_floor
        self.target = self.fn = inferred_floor.build_lut

    def run(self, i):
        ps = self.ps
        return self.fn((ps.H, ps.W), *self.k, ps.ROT, ps.TRANS, ps.LASER, ps.ANGLE_MIN,
                       ps.ANGLE_INC, ps.N_BEAMS, r_max=3.0, self_mask=None)

    def summarise(self, out):
        import numpy as np
        return {'n_valid': np.asarray(out.n_valid)}


@register
class InferredFloorApply(_Frame):
    name = 'camera.inferred_floor.effective_ranges+apply'
    note = 'LiDAR-inferred floor on the class image, scene mask tick (5 Hz)'

    def setup(self):
        super().setup()
        from f1tenth_perception import inferred_floor
        ps = self.ps
        self.fl = inferred_floor
        self.target = inferred_floor.apply
        self.lut = inferred_floor.build_lut(
            (ps.H, ps.W), *self.k, ps.ROT, ps.TRANS, ps.LASER, ps.ANGLE_MIN, ps.ANGLE_INC,
            ps.N_BEAMS, r_max=3.0, self_mask=None)

    def prepare(self, i):
        _d, _m, labels = self.classify(i)
        self._labels = labels.copy()

    def run(self, i):
        ps = self.ps
        r_eff = self.fl.effective_ranges(self.s.scan(i % Scene.K_IMAGE), ps.RANGE_MIN,
                                         ps.RANGE_MAX, far_value=3.0 + 0.15 + 1.0, window_k=3,
                                         sigma_beams=1.5)
        return self.fl.apply(self._labels, self.lut, r_eff, margin=0.15)


@register
class StereoColumns(_Frame):
    name = 'camera.scene_mask.wall_points+wall_fusion.column_ranges+column_extent'
    note = 'the per-column stereo profile the wall fusion consumes, scene mask tick (5 Hz)'

    def setup(self):
        super().setup()
        from f1tenth_perception import scene_mask, wall_fusion
        self.sm, self.wf = scene_mask, wall_fusion
        self.target = scene_mask.wall_points

    def prepare(self, i):
        self._d, _m, self._l = self.classify(i)

    def run(self, i):
        ps = self.ps
        points, rows, cols = self.sm.wall_points(self._d, self._l, *self.k, ps.ROT, ps.TRANS)
        dist, n_px = self.wf.column_ranges(cols, self._d[rows, cols], ps.W, percentile=20.0,
                                           min_px=5)
        z_lo, z_hi = self.wf.column_extent(cols, points[:, 2], ps.W)
        return {'n_points': len(points), 'distance': dist, 'n_px': n_px, 'z_lo': z_lo,
                'z_hi': z_hi}


@register
class SceneMaskStats(_Frame):
    name = 'camera.scene_mask.class_stats'
    note = 'class counts and the too-close strip, scene mask tick (5 Hz)'

    def setup(self):
        super().setup()
        from f1tenth_perception import scene_mask
        self.sm = scene_mask
        self.target = scene_mask.class_stats

    def prepare(self, i):
        ps = self.ps
        self._d, _m, self._l = self.classify(i)
        self._p, _r, _c = self.sm.wall_points(self._d, self._l, *self.k, ps.ROT, ps.TRANS)

    def run(self, i):
        return self.sm.class_stats(self._l, depth=self._d, points=self._p, front_x=0.405,
                                   strip_half_width_m=0.3)


@register
class SceneLabelsTick(_Frame):
    name = 'camera.scene_labels[back_project+fit_ground+label_points+cluster_other]'
    note = ('front_clearance_node._label_scene on the 160x90 grid, 5 Hz; node defaults '
            '(60 RANSAC iterations)')

    def setup(self):
        super().setup()
        import numpy as np
        from f1tenth_perception import ground_plane, scene_labels
        self.np, self.gp, self.sl = np, ground_plane, scene_labels
        self.target = scene_labels.label_points
        ps = self.ps
        gh, gw = 90, 160
        rows = np.arange(gh) * ps.H // gh + ps.H // (2 * gh)
        cols = np.arange(gw) * ps.W // gw + ps.W // (2 * gw)
        rr, cc = np.meshgrid(rows, cols, indexing='ij')
        self.rr, self.cc = rr.ravel(), cc.ravel()
        self.ceiling = scene_labels.CeilingTracker()

    def prepare(self, i):
        self._rng = self.np.random.default_rng(0)

    def run(self, i):
        np, sl, ps = self.np, self.sl, self.ps
        depth, mask, _k = self.s.frame(i)
        z = depth[self.rr, self.cc]
        valid = np.isfinite(z) & (z > 0)
        rv, cv = self.rr[valid], self.cc[valid]
        pts = self.gp.back_project(depth, rv, cv, *self.k, ps.ROT, ps.TRANS)
        in_mask = mask[rv, cv] > 0
        ground, _fit = sl.fit_ground(pts[~in_mask], prior_z=0.0, candidate_band_m=0.15,
                                     inlier_m=0.03, max_angle_deg=5.0, max_offset_m=0.05,
                                     iters=60, min_inliers=100, rng=self._rng)
        heights = ground.height(pts)
        ceiling_h = self.ceiling.update(heights[~in_mask])
        labels, wall_idx = sl.label_points(pts, in_mask, ground, ceiling_h, [],
                                           ground_band_m=0.04, ceiling_band_m=0.08,
                                           max_relevant_height_m=1.0, wall_band_m=0.06)
        clusters = sl.cluster_other(pts, labels, in_mask)
        return {'labels': labels, 'n_clusters': len(clusters)}


@register
class StereoCrossCheck(_Frame):
    name = 'walls.wall_fusion.cross_check_profile+observations_from_lines'
    rate_hz = 10.0
    note = ('per tracker cycle: scan returns projected into the image, compared with the '
            'stereo column profile, lines turned into observations')

    def setup(self):
        super().setup()
        import numpy as np
        from f1tenth_perception import multi_wall_tracker as mwt, scene_labels, scene_mask
        from f1tenth_perception import wall_fusion
        self.np, self.mwt, self.sl, self.sm, self.wf = np, mwt, scene_labels, scene_mask, wall_fusion
        self.target = wall_fusion.cross_check_profile
        self.params = mwt.extraction_params()

    def prepare(self, i):
        ps = self.ps
        k = i % Scene.K_IMAGE
        depth, _m, labels = self.classify(k)
        points, rows, cols = self.sm.wall_points(depth, labels, *self.k, ps.ROT, ps.TRANS)
        self._dist, _n = self.wf.column_ranges(cols, depth[rows, cols], ps.W)
        self._zlo, self._zhi = self.wf.column_extent(cols, points[:, 2], ps.W)
        ranges = self.s.scan(k)
        self._pts = self.sl.scan_points(ranges, ps.ANGLE_MIN, ps.ANGLE_INC, ps.RANGE_MIN,
                                        ps.RANGE_MAX, ps.LASER, math.radians(60.0), ())[0]
        lines = self.mwt.extract_lines(ranges, ps.ANGLE_MIN, ps.ANGLE_INC, 0.06, ps.RANGE_MAX,
                                       ps.LASER, (), params=self.params)
        self._lines = [self.mwt.line_to_wall_line(ln) for ln in lines]

    def run(self, i):
        np, ps = self.np, self.ps

        def project(p):
            return self.sm.project_to_image(p, *self.k, ps.ROT, ps.TRANS)

        xyz = np.column_stack([self._pts, np.full(len(self._pts), ps.LASER_Z)])
        uv = project(xyz)
        scan_depth = ((xyz - ps.TRANS) @ ps.ROT)[:, 2]
        check = self.wf.cross_check_profile(uv[:, 0], scan_depth, self._dist, ps.W)
        obs, rejected = self.wf.observations_from_lines(self._lines, check, project, ps.W,
                                                        self._zlo, self._zhi)
        return {'status': check['status'], 'n_obs': len(obs), 'rejected': rejected,
                'confidence': [float(o.confidence) for o in obs]}


@register
class VerifyRectangles(_Frame):
    name = 'walls.wall_fusion.verify_rectangles'
    rate_hz = 10.0
    note = 'stereo verification of the tracked walls (wall_rect_verify), per tracker cycle'

    def setup(self):
        super().setup()
        from f1tenth_perception import wall_fusion
        self.target = self.fn = wall_fusion.verify_rectangles

    def prepare(self, i):
        self._d, _m, self._l = self.classify(i)
        self._segs = self.ps.segments_in_base(self.s.pose(i % Scene.K_IMAGE))

    def run(self, i):
        ps = self.ps
        return self.fn(self._segs, self._l, self._d, *self.k, ps.ROT, ps.TRANS,
                       laser_z=ps.LASER_Z)


@register
class LidarToImage(_Frame):
    name = 'camera.scene_mask.scan_to_base+project_to_image'
    rate_hz = 5.0
    in_default_stack = False
    note = 'LiDAR -> image projection of a whole scan; drawn only when the overlay is subscribed'

    def setup(self):
        super().setup()
        from f1tenth_perception import scene_mask
        self.sm = scene_mask
        self.target = scene_mask.project_to_image
        import numpy as np
        c, s = math.cos(self.ps.LASER[2]), math.sin(self.ps.LASER[2])
        self.rot_l = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        self.t_l = np.array([self.ps.LASER[0], self.ps.LASER[1], self.ps.LASER_Z])

    def run(self, i):
        ps = self.ps
        pts, _r = self.sm.scan_to_base(self.s.scan(i), ps.ANGLE_MIN, ps.ANGLE_INC, ps.RANGE_MIN,
                                       ps.RANGE_MAX, self.rot_l, self.t_l)
        return self.sm.project_to_image(pts, *self.k, ps.ROT, ps.TRANS)


@register
class SceneOverlay(_Frame):
    name = 'camera.scene_mask.overlay'
    in_default_stack = False
    note = 'debug overlay image; only when /perception/scene_labels/image is subscribed'

    def setup(self):
        super().setup()
        import numpy as np
        from f1tenth_perception import scene_mask
        self.sm = scene_mask
        self.target = scene_mask.overlay
        self.img = np.random.default_rng(3).integers(0, 255, (self.ps.H, self.ps.W, 3),
                                                     dtype=np.uint8)

    def prepare(self, i):
        _d, _m, self._l = self.classify(i)

    def run(self, i):
        return self.sm.overlay(self.img, self._l)


@register
class MaskMedianDepth(_Frame):
    name = 'camera.Detection3DNode._mask_median_depth'
    rate_hz = 15.0
    calls_per_cycle = 3.0
    note = ('mask + depth fusion, once per detection (2 per frame here) at up to 15 Hz; '
            'a full-frame boolean operation each')

    def setup(self):
        super().setup()
        from f1tenth_perception import detection_3d_node as d3
        cls = next(v for v in vars(d3).values()
                   if inspect.isclass(v) and hasattr(v, '_mask_median_depth'))
        self.cls = cls
        self.target = cls._mask_median_depth
        log = types.SimpleNamespace(warn=lambda *a, **k: None)
        self.fake = types.SimpleNamespace(
            scene_enabled=False, _stats=collections.defaultdict(int), center_fraction=0.5,
            get_logger=lambda: log)
        self.fake._median_depth = lambda *a: cls._median_depth(self.fake, *a)

    def run(self, i):
        ps = self.ps
        depth, mask, _k = self.s.frame(i)
        return [self.cls._mask_median_depth(self.fake, depth, mask, j, 320.0, 180.0, 80.0, 160.0,
                                            ps.W, ps.H) for j in (0, 1)]

    def per_cycle(self):
        return 1.5


@register
class CvBridgeDepthIn(_Frame):
    name = 'camera.cv_bridge.imgmsg_to_cv2[32FC1 640x360]'
    rate_hz = 30.0
    calls_per_cycle = 6.0
    note = ('one depth frame into numpy; done at the camera rate by front_clearance_node and '
            'by detection_3d_node (2 x 30 Hz configured)')

    def setup(self):
        super().setup()
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        self.bridge = CvBridge()
        self.target = CvBridge.imgmsg_to_cv2
        self.msgs = []
        for k in range(Scene.K_IMAGE):
            depth, _m, _ = self.s.frame(k)
            msg = Image(height=self.ps.H, width=self.ps.W, encoding='32FC1', step=self.ps.W * 4)
            msg.data = depth.tobytes()
            self.msgs.append(msg)

    def run(self, i):
        return self.bridge.imgmsg_to_cv2(self.msgs[i % len(self.msgs)], desired_encoding='32FC1')


@register
class CvBridgeMaskOut(_Frame):
    name = 'camera.cv_bridge.cv2_to_imgmsg[mono8 640x360]'
    rate_hz = 15.0
    note = 'the label image yolo publishes per processed frame'

    def setup(self):
        super().setup()
        from cv_bridge import CvBridge
        self.bridge = CvBridge()
        self.target = CvBridge.cv2_to_imgmsg

    def run(self, i):
        return self.bridge.cv2_to_imgmsg(self.s.frame(i)[1], encoding='mono8')

    def summarise(self, out):
        import numpy as np
        return {'data': np.frombuffer(bytes(out.data), dtype=np.uint8),
                'hw': np.asarray([out.height, out.width])}


# ---- costmap ---------------------------------------------------------------------

class _Grid(Case):
    scene_refs = ('grid', 'pose')

    def setup(self):
        self.s = scene()
        self.ps = self.s.ps
        self.s.warm()
        ps = self.ps
        self.g = (self.s.grid(), ps.GRID_W, ps.GRID_H, ps.GRID_RES, ps.GRID_ORIGIN[0],
                  ps.GRID_ORIGIN[1])

    def inputs(self, i):
        import numpy as np
        return {'pose_index': np.asarray(i % Scene.K_SCAN)}


@register
class BoundaryExtract(_Grid):
    name = 'costmap.costmap_boundary.extract_boundary_constraints'
    note = 'boundary extraction from the SLAM grid, every 100 ms, map changed or not'

    def setup(self):
        super().setup()
        from f1tenth_costmap import costmap_boundary
        self.target = self.fn = costmap_boundary.extract_boundary_constraints

    def run(self, i):
        return self.fn(*self.g, *self.s.pose(i), math.radians(35.0), math.radians(45.0),
                       math.radians(135.0), 65, 5.0)


@register
class BoundaryStrip(_Grid):
    name = 'costmap.costmap_boundary.nearest_occupied_in_strip'
    note = 'front clearance for the behaviour tree, same tick as the boundary extraction'

    def setup(self):
        super().setup()
        from f1tenth_costmap import costmap_boundary
        self.target = self.fn = costmap_boundary.nearest_occupied_in_strip

    def run(self, i):
        return self.fn(*self.g, *self.s.pose(i), 0.405, 0.30, 65, 5.0)


@register
class BoundaryComponents(_Grid):
    name = 'costmap.costmap_boundary.occupied_components+filter_occupied_cells'
    note = 'only with component_filter or exclude_tracked_object_cells set'
    in_default_stack = False

    def setup(self):
        super().setup()
        from f1tenth_costmap import costmap_boundary
        self.cb = costmap_boundary
        self.target = costmap_boundary.filter_occupied_cells

    def run(self, i):
        labels, comps = self.cb.occupied_components(*self.g, 65)
        return self.cb.filter_occupied_cells(*self.g, 65, min_component_length_m=0.3,
                                             components=comps, labels=labels)

    def summarise(self, out):
        import numpy as np
        data, report = out
        return {'data': np.asarray(data), **flatten(report, 'report')}


@register
class SafeCorridor(_Grid):
    name = 'costmap.safe_corridor.build_safe_corridor'
    in_default_stack = False
    note = 'convex safe corridor incl. classify_local_cells; use_convex_polytope is off by default'

    def setup(self):
        super().setup()
        import numpy as np
        from f1tenth_costmap import safe_corridor
        self.np = np
        self.target = self.fn = safe_corridor.build_safe_corridor

    def run(self, i):
        x, y, yaw = self.s.pose(i)
        s = self.np.linspace(0.0, 3.0, 60)
        line = self.np.column_stack([x + s * math.cos(yaw), y + s * math.sin(yaw)])
        return self.fn(*self.g, line, 3.0, 65)


@register
class ClassifyLocalCells(_Grid):
    name = 'costmap.safe_corridor.classify_local_cells'
    in_default_stack = False
    note = 'first stage of the convex safe corridor (off by default)'

    def setup(self):
        super().setup()
        from f1tenth_costmap import safe_corridor
        self.target = self.fn = safe_corridor.classify_local_cells

    def run(self, i):
        x, y, _yaw = self.s.pose(i)
        return self.fn(*self.g, x, y, 3.0, 65)


@register
class CostmapRender(_Grid):
    name = 'costmap.costmap_renderer.render_costmap_rgb+rgb_array_to_image_msg'
    rate_hz = 2.0
    note = 'visualisation image, 2 Hz, rendered whether or not anyone subscribes'

    def setup(self):
        super().setup()
        from builtin_interfaces.msg import Time
        from f1tenth_costmap import costmap_renderer
        self.cr, self.stamp = costmap_renderer, Time()
        self.target = costmap_renderer.render_costmap_rgb

    def run(self, i):
        rgb = self.cr.render_costmap_rgb(*self.g, [])
        return rgb, self.cr.rgb_array_to_image_msg(rgb, 'map', self.stamp)

    def summarise(self, out):
        return {'rgb': out[0]}


@register
class WallRaster(_Grid):
    name = 'costmap.wall_raster.rasterize_walls[240x240]'
    rate_hz = 5.0
    note = 'semantic layer wall cells, 5 Hz; no live consumer found'

    def setup(self):
        super().setup()
        from f1tenth_costmap import wall_raster
        self.target = self.fn = wall_raster.rasterize_walls
        self.walls = [(j + 1, 'side' if j < 3 else 'front', a, b)
                      for j, (a, b) in enumerate(self.ps.SEGMENTS[:4])]

    def run(self, i):
        x, y, _yaw = self.s.pose(i)
        return self.fn(self.walls, x - 6.0, y - 6.0, 0.05, 240, 240)


@register
class SemanticTracks(Case):
    name = 'costmap.semantic_layer.update_tracks_batch'
    n_golden = 40
    note = 'one detection batch (4 objects) folded into the semantic tracks, 10 Hz'

    def setup(self):
        import numpy as np
        from f1tenth_costmap import semantic_layer
        self.np = np
        self.target = self.fn = semantic_layer.update_tracks_batch
        self.tracks = []
        self.ids = itertools.count()
        self.rng = np.random.default_rng(7)
        self.objs = [('person', 5.0, -0.35), ('chair', 3.2, 0.45), ('bottle', 6.5, 0.2),
                     ('potted plant', 7.5, -0.7)]

    def prepare(self, i):
        self._dets = [(c, x + self.rng.normal(0, 0.05), y + self.rng.normal(0, 0.05), 0.8, 0.1)
                      for c, x, y in self.objs]

    def run(self, i):
        self.tracks = self.fn(self.tracks, self._dets, 0.1 * i, 0.5, 0.3, 3, 5,
                              lambda: next(self.ids))
        return self.tracks

    def summarise(self, out):
        np = self.np
        return {'n': np.asarray(len(out)),
                'xy': np.asarray(sorted((float(t.x_map), float(t.y_map)) for t in out)
                                 ).reshape(-1, 2)}


@register
class FreespaceDoors(_Grid):
    name = 'doors.freespace_doors[scan_profile+occupancy_window+candidates]'
    note = ('freespace_door_node._detect, 10 Hz: full-map copy, morphology and distance '
            'transform on the window, then the candidates')
    scene_refs = ('grid', 'pose', 'scan')

    def setup(self):
        super().setup()
        import numpy as np
        from f1tenth_costmap import freespace_doors as fd
        self.np, self.fd = np, fd
        self.target = fd.occupancy_window
        self.p = fd.FreespaceParams()
        ps = self.ps
        self.bearings = ps.ANGLE_MIN + np.arange(ps.N_BEAMS, dtype=float) * ps.ANGLE_INC

    def run(self, i):
        fd, ps, p = self.fd, self.ps, self.p
        x, y, yaw = self.s.pose(i)
        laser = (x + ps.LASER[0] * math.cos(yaw), y + ps.LASER[0] * math.sin(yaw), yaw)
        profile = fd.scan_profile(self.bearings, self.s.scan(i), ps.RANGE_MIN, ps.RANGE_MAX,
                                  laser, p)
        window = fd.occupancy_window(*self.g, (x, y), p)
        if p.use_skeleton:
            skel = fd.skeleton(window, p)
            cands = fd.skeleton_candidates(window, skel, fd.ScanBeams(profile.points,
                                                                      profile.laser), (x, y), p)
        else:
            cands = fd.scan_candidates(profile, window, (x, y), p, debug=[])
        return cands, len(fd.jump_points(profile, p))

    def summarise(self, out):
        np = self.np
        cands, jumps = out
        return {'n': np.asarray(len(cands)), 'jumps': np.asarray(jumps),
                'codes': np.array(','.join(sorted(str(getattr(c, 'code', '')) for c in cands)))}


# ----------------------------------------------------------------- the runner

def _stats(ns):
    import numpy as np
    ms = np.asarray(ns, dtype=float) / 1e6
    return {'p50_ms': float(np.percentile(ms, 50)), 'p95_ms': float(np.percentile(ms, 95)),
            'max_ms': float(ms.max()), 'min_ms': float(ms.min()), 'mean_ms': float(ms.mean())}


def time_case(case, warmup, iters):
    case.setup()
    for i in range(warmup):
        case.prepare(i)
        case.run(i)
    ns = []
    clock = time.perf_counter_ns
    for i in range(warmup, warmup + iters):
        case.prepare(i)
        t0 = clock()
        case.run(i)
        ns.append(clock() - t0)
    return _stats(ns)


def alloc_case(case):
    case.setup()
    case.prepare(0)
    case.run(0)
    tracemalloc.start()
    peak = 0
    for i in range(1, 1 + ALLOC_CALLS):
        case.prepare(i)
        base = tracemalloc.get_traced_memory()[0]
        tracemalloc.reset_peak()
        out = case.run(i)
        peak = max(peak, tracemalloc.get_traced_memory()[1] - base)
        del out
    tracemalloc.stop()
    return peak / 1024.0


BIG_FLOAT = 20000     # elements; larger float arrays are stored subsampled
BIG_ANY = 300000
STRIDE = 17


def compact(arrays):
    """Large arrays as a strided sample plus [sum, min, max, n_nan, size].

    Keeps the golden files small (a 640x360 float image is 0.9 MB) while still
    comparing with a tolerance: every 17th element and four whole-array statistics.
    """
    import numpy as np
    out = {}
    for k, v in arrays.items():
        a = np.asarray(v)
        big = a.size > BIG_ANY or (a.size > BIG_FLOAT and a.dtype.kind == 'f')
        if not big or a.dtype.kind in 'US':
            out[k] = a
            continue
        f = a.astype(float).ravel()
        fin = f[np.isfinite(f)]
        out[k + '#sample'] = a.ravel()[::STRIDE]
        out[k + '#stats'] = np.array([fin.sum() if fin.size else 0.0,
                                      fin.min() if fin.size else np.nan,
                                      fin.max() if fin.size else np.nan,
                                      f.size - fin.size, f.size, *a.shape])
    return out


def golden_case(case, from_file=None):
    """(inputs, outputs, pickle-or-None) of the case's first n_golden calls."""
    case.setup()
    replay = None
    if case.replay:
        if from_file is not None:
            replay = pickle.loads(bytes(from_file['in/pickle']))
            case.calls = replay
        else:
            replay = [case.args(i) for i in range(case.n_golden)]
    ins, outs = {}, {}
    for i in range(case.n_golden):
        case.prepare(i)
        for k, v in case.inputs(i).items():
            ins[f'in/{i}/{k}'] = v
        for k, v in case.summarise(case.run(i)).items():
            outs[f'out/{i}/{k}'] = v
    return ins, compact(outs), replay


def profile_case(case, path):
    case.setup()
    for i in range(5):
        case.prepare(i)
        case.run(i)
    prof = cProfile.Profile()
    for i in range(5, 5 + PROFILE_CALLS):
        case.prepare(i)
        prof.enable()
        case.run(i)
        prof.disable()
    buf = io.StringIO()
    stats = pstats.Stats(prof, stream=buf).strip_dirs().sort_stats('cumulative')
    stats.print_stats(10)
    text = buf.getvalue()
    path.write_text(f'# {case.name}\n# cProfile over {PROFILE_CALLS} calls, top 10 by cumulative '
                    f'time (seconds are totals over those calls)\n{text}')
    return text


def osqp_phases(warmup, iters):
    """
    Split the forward solve into Python assembly and the three OSQP calls.

    Timed directly around osqp.OSQP(), setup() and solve() through a thin proxy
    class, not read from cProfile: OSQP's methods are pybind11 callables, which
    cProfile does not see as C calls, so the profile's cumulative column around
    them is not usable.
    """
    import numpy as np
    from mpc_controller import mpc_solver
    real = mpc_solver.osqp
    acc = {'init': 0, 'setup': 0, 'solve': 0}

    class Timed:
        def __init__(self, *a, **k):
            t0 = time.perf_counter_ns()
            self._p = real.OSQP(*a, **k)
            acc['init'] += time.perf_counter_ns() - t0

        def setup(self, *a, **k):
            t0 = time.perf_counter_ns()
            out = self._p.setup(*a, **k)
            acc['setup'] += time.perf_counter_ns() - t0
            return out

        def solve(self, *a, **k):
            t0 = time.perf_counter_ns()
            out = self._p.solve(*a, **k)
            acc['solve'] += time.perf_counter_ns() - t0
            return out

        def __getattr__(self, name):
            return getattr(self._p, name)

    proxy = types.SimpleNamespace(**{k: getattr(real, k) for k in dir(real)
                                     if not k.startswith('__')})
    proxy.OSQP = Timed
    case = MpcSolveForward()
    case.setup()
    rows = {'total': [], 'init': [], 'setup': [], 'solve': [], 'python': []}
    iters_osqp = []
    mpc_solver.osqp = proxy
    try:
        for i in range(warmup + iters):
            case.prepare(i)
            acc['init'] = acc['setup'] = acc['solve'] = 0
            t0 = time.perf_counter_ns()
            _u, info = case.run(i)
            total = time.perf_counter_ns() - t0
            if i >= warmup:
                rows['total'].append(total)
                for key in ('init', 'setup', 'solve'):
                    rows[key].append(acc[key])
                rows['python'].append(total - acc['init'] - acc['setup'] - acc['solve'])
                iters_osqp.append(float(info.get('iterations', np.nan)))
    finally:
        mpc_solver.osqp = real
    out = {k: _stats(v) for k, v in rows.items()}
    it = np.asarray(iters_osqp)
    out['osqp_iterations'] = {'p50': float(np.nanpercentile(it, 50)),
                              'p95': float(np.nanpercentile(it, 95)), 'max': float(np.nanmax(it))}
    return out, case


def _save_npz(path, arrays):
    import numpy as np
    np.savez_compressed(path, **{k: np.asarray(v) for k, v in arrays.items()})


def compare(saved, fresh, rtol, atol):
    """(n_leaves, n_bad, worst abs diff, first bad key)."""
    import numpy as np
    n = bad = 0
    worst, first = 0.0, ''
    for key in sorted(k for k in saved.files if k.startswith('out/')):
        n += 1
        a = saved[key]
        b = fresh.get(key)
        ok = b is not None
        if ok:
            b = np.asarray(b)
            if a.dtype.kind in 'US' or b.dtype.kind in 'US':
                ok = str(a) == str(b)
            elif a.shape != b.shape:
                ok = False
            else:
                af, bf = a.astype(float), b.astype(float)
                ok = bool(np.allclose(af, bf, rtol=rtol, atol=atol, equal_nan=True))
                both = np.isfinite(af) & np.isfinite(bf)
                if both.any():
                    worst = max(worst, float(np.max(np.abs(af[both] - bf[both]))))
        if not ok:
            bad += 1
            first = first or key
    extra = [k for k in fresh if k not in saved.files]
    if extra:
        bad += len(extra)
        first = first or extra[0]
    return n, bad, worst, first


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--out', help='output directory (micro.csv, golden/, profiles/)')
    ap.add_argument('--only', action='append', default=[], help='fnmatch pattern on the case name')
    ap.add_argument('--warmup', type=int, default=20)
    ap.add_argument('--iters', type=int, default=200)
    ap.add_argument('--core', default='auto', help="CPU to pin to ('auto' = least busy)")
    ap.add_argument('--profile-top', type=int, default=5)
    ap.add_argument('--no-golden', action='store_true')
    ap.add_argument('--golden-only', action='store_true',
                    help='(re)write <out>/golden without timing anything')
    ap.add_argument('--csv-name', default='micro.csv')
    ap.add_argument('--check-golden', metavar='DIR')
    ap.add_argument('--rtol', type=float, default=1e-9)
    ap.add_argument('--atol', type=float, default=1e-12)
    ap.add_argument('--list', action='store_true')
    args = ap.parse_args(argv)
    if not args.list:
        _repin(args.core)
    _add_test_paths()
    import numpy as np

    cases = [c() for c in CASES
             if not args.only or any(fnmatch.fnmatch(c.name, p) for p in args.only)]
    if args.list:
        for c in cases:
            print(c.name)
        return 0

    if args.check_golden:
        gdir = pathlib.Path(args.check_golden)
        failed = 0
        rows = []
        scene_file = gdir / '_scene.npz'
        if scene_file.exists():
            saved_scene = np.load(scene_file, allow_pickle=False)
            fresh_scene = scene().arrays()
            changed = [k for k in saved_scene.files
                       if k not in fresh_scene
                       or not np.array_equal(saved_scene[k], fresh_scene[k], equal_nan=True)]
            if changed:
                print(f'SCENE_CHANGED: perf_scene.py no longer reproduces {changed[:5]} '
                      f'({len(changed)} arrays): the scan/camera/grid cases below are not '
                      'comparing like with like')
                failed += 1
        for c in cases:
            path = gdir / (_slug(c.name) + '.npz')
            if not path.exists():
                rows.append((c.name, 'MISSING', 0, 0, 0.0, ''))
                failed += 1
                continue
            saved = np.load(path, allow_pickle=False)
            meta = json.loads(str(saved['meta']))
            ins, outs, _r = golden_case(c, from_file=saved if c.replay else None)
            if not c.replay and digest(ins) != meta['input_digest']:
                rows.append((c.name, 'INPUTS_CHANGED', 0, 0, 0.0, ''))
                failed += 1
                continue
            n, bad, worst, first = compare(saved, outs, args.rtol, args.atol)
            rows.append((c.name, 'OK' if bad == 0 else 'DIFF', n, bad, worst, first))
            failed += bad != 0
        for name, status, n, bad, worst, first in rows:
            print(f'{status:15s} {name}  leaves={n} bad={bad} worst_abs={worst:.3g} {first}')
        if args.out:
            out = pathlib.Path(args.out)
            out.mkdir(parents=True, exist_ok=True)
            with open(out / 'golden_check.csv', 'w', newline='') as fh:
                w = csv.writer(fh)
                w.writerow(['function', 'status', 'leaves', 'bad', 'worst_abs_diff', 'first_bad'])
                w.writerows(rows)
        print(f'{len(rows) - failed} of {len(rows)} cases equal (rtol {args.rtol}, atol {args.atol})')
        return 1 if failed else 0

    if not args.out:
        ap.error('--out is required to measure')
    out = pathlib.Path(args.out)
    (out / 'golden').mkdir(parents=True, exist_ok=True)
    (out / 'profiles').mkdir(parents=True, exist_ok=True)
    rows = []
    osqp_iterations = None
    if args.golden_only:
        for c in cases:
            _write_golden(out, c)
            print(f'golden  {c.name}', flush=True)
        if SCENE is not None:
            _save_npz(out / 'golden' / '_scene.npz', SCENE.arrays())
        return 0
    for c in cases:
        t0 = time.time()
        try:
            st = time_case(c, args.warmup, args.iters)
            alloc_kb = alloc_case(type(c)())
        except Exception as exc:  # a case that cannot run is reported, not fatal
            print(f'FAILED  {c.name}: {exc!r}', flush=True)
            rows.append({'function': c.name, 'error': repr(exc)})
            continue
        row = {'function': c.name, 'file_line': c.where(), **{k: round(v, 4) for k, v in st.items()},
               'alloc_peak_kb': round(alloc_kb, 1), 'calls_per_cycle': round(c.per_cycle(), 3),
               'ms_per_cycle': round(st['p50_ms'] * c.per_cycle(), 4),
               'in_default_stack': c.in_default_stack, 'n': args.iters, 'note': c.note}
        rows.append(row)
        if not args.no_golden:
            _write_golden(out, c, row['file_line'])
        print(f"{row['p50_ms']:9.3f} ms p50 {row['p95_ms']:9.3f} p95 {row['max_ms']:9.3f} max "
              f"x{row['calls_per_cycle']:<5} {c.name}  [{time.time() - t0:.1f}s]", flush=True)

    if any(isinstance(c, MpcSolveForward) for c in cases):
        phases, fc = osqp_phases(args.warmup, args.iters)
        osqp_iterations = phases.pop('osqp_iterations')
        for key, label in (('python', 'Python: linearisation, dense P/q/A assembly, dense->csc, '
                                      'rollout, cost'),
                           ('init', 'osqp.OSQP(): a new solver object every tick'),
                           ('setup', 'OSQP.setup(): KKT factorisation from scratch'),
                           ('solve', 'OSQP.solve() incl. polish')):
            st = phases[key]
            rows.append({'function': f'mpc.solve_mpc_step/phase[{key}]', 'file_line': fc.where(),
                         **{k: round(v, 4) for k, v in st.items()}, 'alloc_peak_kb': '',
                         'calls_per_cycle': 1.0, 'ms_per_cycle': round(st['p50_ms'], 4),
                         'in_default_stack': True, 'n': args.iters,
                         'note': f'part of the forward solve, not an extra cost: {label}'})
            print(f"{st['p50_ms']:9.3f} ms p50 {st['p95_ms']:9.3f} p95  phase {key}", flush=True)

    if not args.no_golden and SCENE is not None:
        _save_npz(out / 'golden' / '_scene.npz', SCENE.arrays())

    ok = [r for r in rows if 'error' not in r and '/phase[' not in r['function']]
    top = sorted(ok, key=lambda r: -r['ms_per_cycle'])[:args.profile_top]
    by_name = {c.name: c for c in cases}
    for r in top:
        profile_case(type(by_name[r['function']])(), out / 'profiles' / (_slug(r['function']) + '.txt'))

    fields = ['function', 'file_line', 'p50_ms', 'p95_ms', 'max_ms', 'min_ms', 'mean_ms',
              'alloc_peak_kb', 'calls_per_cycle', 'ms_per_cycle', 'in_default_stack', 'n',
              'note', 'error']
    with open(out / args.csv_name, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=fields, restval='')
        w.writeheader()
        w.writerows(rows)
    meta = {'host': platform.node(), 'machine': platform.machine(), 'python': platform.python_version(),
            'numpy': np.__version__, 'core': os.environ.get('F1T_PERF_CORE'),
            'warmup': args.warmup, 'iters': args.iters, 'loadavg': os.getloadavg(),
            'time': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'osqp_iterations': osqp_iterations,
            'profiled': [r['function'] for r in top]}
    (out / (pathlib.Path(args.csv_name).stem + '.meta.json')).write_text(json.dumps(meta, indent=1))
    print(f'wrote {out / args.csv_name} ({len(rows)} rows), core {meta["core"]}, '
          f'load {meta["loadavg"][0]:.1f}')
    return 0


def _write_golden(out, c, file_line=None):
    import numpy as np
    g = type(c)()
    ins, outs, replay = golden_case(g)
    file_line = file_line or g.where()
    arrays = dict(ins)
    arrays.update(outs)
    if replay is not None:
        arrays['in/pickle'] = np.frombuffer(pickle.dumps(replay, protocol=4), dtype=np.uint8)
    arrays['meta'] = np.array(json.dumps({
        'case': c.name, 'target': file_line, 'n_calls': g.n_golden, 'replay': g.replay,
        'scene_refs': list(g.scene_refs), 'input_digest': digest(ins)}))
    _save_npz(out / 'golden' / (_slug(c.name) + '.npz'), arrays)


def _slug(name):
    return ''.join(ch if ch.isalnum() or ch in '._-' else '_' for ch in name).strip('_')


if __name__ == '__main__':
    sys.exit(main())
