#!/usr/bin/env python3
"""
Closed-loop parity of the Rust solver core (main-jazzy-rust R2, layer L4).

Not a test module (no test_ prefix): run it as a script,

    python3 test/closed_loop_parity.py [--json OUT] [--quick]

Every closed-loop rig of this package is driven twice, with solve_mpc_step's
implementation 'py' and then 'rust' (solver_backend.DEFAULT_IMPL, which every
rig's call picks up: they pass no impl=). Each solve is recorded (x0, u0, the
plan, status, cost), and per run the two are compared:

  - OUTCOME: the rig's own result -- the object approach's tick count, final
    flags and terminal state, corridor steps; the person pass's metrics; the
    speed channel's trace length, rest position and events -- identical;
  - END TICK: the number of solves -- identical;
  - FINAL POSE (the state the last solve started from): |dx|, |dy| <= 1 cm,
    |dpsi| <= 0.01 rad (the gate; on linus the expectation is bit identity,
    reported separately as an identical per-run SHA-256 of every solve).

The rigs: the object approach (test_object_approach_closed_loop, every scenario
x no obstacle / footprint), the person pass (test_person_pass_closed_loop),
the speed-channel harness (speed_channel_harness.run, a 4 m goal_distance move
at 0.5 m/s, legacy and offset_free x 0 / 1 / 8 obstacles x 0 / 4 live wall
rows, the R2.0 profile's matrix) and closed_loop_golden's direct solves.

THE SHA GOLDEN (closed_loop_golden.py, test/data/closed_loop_golden.json) stays
the Python reference: it is read, never rewritten, and its digests are also
recomputed with the Rust core and compared with it.

Exit 0 when every run passes.
"""

import argparse
import contextlib
import hashlib
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

POS_TOL_M = 0.01
YAW_TOL_RAD = 0.01

OBSTACLE_SETS = {
    0: [],
    1: [(2.0, 0.1, 0.18)],
    8: [(2.0, 0.1, 0.18)] + [(2.5 + 0.6 * k, (-1.0) ** k * (1.0 + 0.1 * k), 0.15)
                             for k in range(7)],
}
WALL_SETS = {
    0: [],
    4: [(0.0, 1.0, 0.95), (0.0, -1.0, 0.80), (1.0, 0.0, 8.0), (0.6, 0.8, 4.0)],
}


@contextlib.contextmanager
def patched(modules, solve):
    saved = [(m, m.solve_mpc_step) for m in modules]
    for m in modules:
        m.solve_mpc_step = solve
    try:
        yield
    finally:
        for m, orig in saved:
            m.solve_mpc_step = orig


class Recorder:
    """A solve_mpc_step stand-in that solves with the module and records."""

    def __init__(self, mpc_solver, inject=None):
        self.mpc_solver = mpc_solver
        self.inject = inject
        self.ticks = []
        self.h = hashlib.sha256()

    def __call__(self, **kw):
        if self.inject is not None:
            kw = self.inject(dict(kw))
        u0, info = self.mpc_solver.solve_mpc_step(**kw)
        x0 = np.asarray(kw['x0'], dtype=float)
        self.ticks.append({'x0': x0.copy(), 'u0': np.asarray(u0, dtype=float).copy(),
                           'status': info.get('status'), 'success': info.get('success')})
        for arr in (x0, u0, info.get('zopt', []), info.get('x_pred', []),
                    [info.get('cost', math.nan)]):
            self.h.update(np.ascontiguousarray(np.asarray(arr, dtype=float)).tobytes())
        return u0, info


def _plain(obj):
    """Return a JSON-comparable form of a rig's outcome."""
    return json.loads(json.dumps(obj, default=repr, sort_keys=True))


def _close(a, b, tol=POS_TOL_M):
    """Discrete parts equal, floats within `tol` (1 cm / 0.01 rad), recursively."""
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_close(a[k], b[k], tol) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_close(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, float) and isinstance(b, float):
        if math.isnan(a) or math.isnan(b):
            return math.isnan(a) and math.isnan(b)
        return a == b or abs(a - b) <= tol
    return a == b


def runs(quick):
    """(name, modules to patch, run function -> outcome, inject or None)."""
    import closed_loop_golden as golden
    import speed_channel_harness as harness
    import test_object_approach_closed_loop as approach_rig
    import test_person_pass_closed_loop as person_rig

    out = []
    for name, kwargs in golden.APPROACH_RUNS:
        def fn(kwargs=kwargs):
            res = approach_rig.run_approach(**kwargs)
            return {'ticks': len(res.xs) - 1, 'steps': len(res.steps),
                    'flags_last': res.flags[-1] if res.flags else None,
                    'terminal_last': res.terminal[-1] if res.terminal else None,
                    'deflected': sum(bool(d) for d in res.deflected),
                    'final_state': [float(v) for v in res.xs[-1]]}
        out.append((name, [approach_rig], fn, None))
    for name, kwargs in golden.PERSON_RUNS:
        def fn(kwargs=kwargs):
            return person_rig.run_pass(kwargs['obstacle_r'], kwargs['offset'])
        out.append((name, [person_rig], fn, None))

    from f1tenth_params.param_defaults import get_value
    bms = int(get_value('boundary_max_sources'))
    hard = bool(get_value('boundary_hard'))
    w_wall = float(get_value('wall_row_slack_weight'))
    s_max = float(get_value('boundary_slack_max'))
    for speed in ('legacy', 'offset_free'):
        for n_obs in (0, 1, 8):
            for n_walls in (0, 4):
                obs, walls = OBSTACLE_SETS[n_obs], WALL_SETS[n_walls]

                def inject(kw, obs=obs, walls=walls):
                    kw['obstacles'] = list(obs)
                    kw['boundaries'] = list(walls)
                    kw['boundary_max_sources'] = bms
                    kw['boundary_hard'] = hard
                    kw['boundary_slack_weight'] = w_wall
                    kw['boundary_slack_max'] = s_max
                    kw['corridor'] = dict(kw['corridor'], obstacles_world=list(obs))
                    return kw

                def fn(speed=speed):
                    trace = harness.run(mode=speed, vdes=0.5, goal_distance=4.0,
                                        duration=6.0 if quick else 14.0)
                    return {'ticks': len(trace.t), 'rest_x': trace.rest_x,
                            'events': trace.events,
                            'sources': sorted(set(trace.source))}
                out.append((f'speed/{speed}/obs{n_obs}/walls{n_walls}', [harness], fn,
                            inject))

    for name, kwargs in golden._direct_cases():
        def fn(kwargs=kwargs):
            from mpc_controller import mpc_solver
            return {'n': 1, 'u0': [float(v) for v in
                                   mpc_solver.solve_mpc_step(**kwargs)[0]]}
        out.append((name, [], fn, None))
    return out


def drive(name, modules, fn, inject, impl, mpc_solver, solver_backend):
    solver_backend.DEFAULT_IMPL = impl
    rec = Recorder(mpc_solver, inject)
    t0 = time.perf_counter()
    if modules:
        with patched(modules, rec):
            outcome = fn()
    else:
        outcome = fn()
        rec.ticks.append({'x0': np.zeros(1), 'u0': np.asarray(outcome['u0']),
                          'status': None, 'success': None})
        rec.h.update(np.asarray(outcome['u0'], dtype=float).tobytes())
    return {'ticks': rec.ticks, 'sha': rec.h.hexdigest(), 'outcome': _plain(outcome),
            'seconds': time.perf_counter() - t0}


def compare(py, rs):
    n_py, n_rs = len(py['ticks']), len(rs['ticks'])
    row = {'solves_py': n_py, 'solves_rust': n_rs, 'end_tick_same': n_py == n_rs,
           'outcome_same': _close(py['outcome'], rs['outcome']),
           'outcome_identical': py['outcome'] == rs['outcome'],
           'sha_same': py['sha'] == rs['sha']}
    n = min(n_py, n_rs)
    if n:
        dx = [np.max(np.abs(a['x0'] - b['x0'])) if a['x0'].shape == b['x0'].shape
              else math.inf for a, b in zip(py['ticks'][:n], rs['ticks'][:n])]
        pairs = list(zip(py['ticks'][:n], rs['ticks'][:n]))
        du = [np.max(np.abs(a['u0'] - b['u0'])) for a, b in pairs]
        row['max_state_diff'] = float(max(dx))
        row['max_u0_diff'] = float(max(du))
        row['status_same'] = all(a['status'] == b['status'] and a['success'] == b['success']
                                 for a, b in zip(py['ticks'][:n], rs['ticks'][:n]))
        a, b = py['ticks'][n - 1]['x0'], rs['ticks'][n - 1]['x0']
        if a.size >= 3 and b.size >= 3:
            row['final_pos_diff_m'] = float(math.hypot(a[0] - b[0], a[1] - b[1]))
            row['final_yaw_diff_rad'] = float(abs(a[2] - b[2]))
        else:
            row['final_pos_diff_m'] = row['final_yaw_diff_rad'] = 0.0
    row['pass'] = bool(row['end_tick_same'] and row['outcome_same']
                       and row.get('final_pos_diff_m', 0.0) <= POS_TOL_M
                       and row.get('final_yaw_diff_rad', 0.0) <= YAW_TOL_RAD
                       and row.get('status_same', True))
    return row


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--json')
    ap.add_argument('--quick', action='store_true', help='shorter speed-channel runs')
    args = ap.parse_args(argv)
    os.environ.pop('F1TENTH_MPC_SOLVER_IMPL', None)
    os.environ.pop('F1TENTH_MPC_CAPTURE', None)
    from mpc_controller import mpc_native, mpc_solver, solver_backend
    if not mpc_native.available():
        raise SystemExit(f'Rust library not loadable: {mpc_native.load_error()}')
    import closed_loop_golden as golden

    table = {}
    seconds = {'py': 0.0, 'rust': 0.0}
    for name, modules, fn, inject in runs(args.quick):
        res = {impl: drive(name, modules, fn, inject, impl, mpc_solver, solver_backend)
               for impl in ('py', 'rust')}
        for impl in res:
            seconds[impl] += res[impl]['seconds']
        row = compare(res['py'], res['rust'])
        table[name] = row
        print(f"{'PASS' if row['pass'] else 'FAIL'}  {name:<34} solves {row['solves_py']:4d}/"
              f"{row['solves_rust']:4d}  sha {'=' if row['sha_same'] else '!='}  "
              f"pose {row.get('final_pos_diff_m', 0.0):.2e} m  "
              f"yaw {row.get('final_yaw_diff_rad', 0.0):.2e}", flush=True)

    # the SHA golden: the Python reference, compared with the Rust digests
    expected = golden.load_golden()
    solver_backend.DEFAULT_IMPL = 'rust'
    got_rust = golden.digests()
    solver_backend.DEFAULT_IMPL = 'py'
    got_py = golden.digests()
    golden_rust = {k: got_rust.get(k) == v for k, v in expected.items()}
    golden_py = {k: got_py.get(k) == v for k, v in expected.items()}
    print(f'golden SHA (py reference, {len(expected)} runs): py '
          f'{sum(golden_py.values())}/{len(expected)}, rust {sum(golden_rust.values())}/'
          f'{len(expected)} equal')
    ok = all(r['pass'] for r in table.values())
    out = {'runs': table, 'n_runs': len(table),
           'pass_runs': sum(r['pass'] for r in table.values()),
           'bit_identical_runs': sum(r['sha_same'] for r in table.values()),
           'seconds_in_rigs': seconds,
           'golden': {'path': str(golden.GOLDEN_PATH), 'runs': len(expected),
                      'equal_py': sum(golden_py.values()),
                      'equal_rust': sum(golden_rust.values()),
                      'differing_rust': sorted(k for k, v in golden_rust.items() if not v)},
           'tolerances': {'pos_m': POS_TOL_M, 'yaw_rad': YAW_TOL_RAD}, 'pass': ok}
    print(f"{out['pass_runs']}/{out['n_runs']} runs pass, {out['bit_identical_runs']} "
          f"bit-identical; time in the rigs py {seconds['py']:.1f} s, rust "
          f"{seconds['rust']:.1f} s")
    print('PASS' if ok else 'FAIL')
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, indent=1, default=repr)
            fh.write('\n')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
