"""
Offline timing of the reverse manoeuvre: forward alone, rear alone, both, per tick.

Not a test (no test_ prefix). Run from src/f1tenth_control/mpc_controller with
the workspace sourced:

    python3 test/bench_reverse_manoeuvre.py --n 1000

On the Orin (Humble):

    source /opt/ros/humble/setup.bash && source ~/dev_ws/f1tenth_more/install/setup.bash
    cd ~/dev_ws/f1tenth_more/src/f1tenth_control/mpc_controller
    python3 test/bench_reverse_manoeuvre.py --n 1000

Each configuration solves --n synthetic ticks: states drawn around a 1 m
corridor (x 0..3 m, |y| <= 0.1, |psi| <= 0.1, v 0..0.5) with the shipped
weights, limits, horizons and the side boundary rows, so every solve is a
fresh, realistic problem rather than the same one cached. A "tick" is what
MPC_corr's control loop spends in these pieces: corridor + target + forward
solve + the supervisor (pre-check, and the rear solve when it runs). ROS
callbacks, logging and publishing are NOT in it. The budget check is p95 and
max against the 100 ms control period.
"""

import argparse
import json
import os
import sys
import time

from f1tenth_params.object_geometry import SOLVER_FRONT_POINT_OFFSETS_M
from mpc_controller.MPC_corr import MPCController
from mpc_controller.mpc_solver import solve_mpc_step
from mpc_controller.reverse_manoeuvre import (
    rear_corridor, RearEnv, RearSolver, ReverseSupervisor)
from mpc_controller.reverse_mpc import rear_probe_offsets
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import reverse_harness as rh  # noqa: E402
import reverse_manoeuvre_harness as H  # noqa: E402

BUDGET_MS = 100.0


def _states(n, seed=20261020):
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(0.0, 3.0, n), rng.uniform(-0.1, 0.1, n),
                            rng.uniform(-0.1, 0.1, n), rng.uniform(0.0, 0.5, n)])


def _stats(ms):
    ms = np.asarray(ms)
    return {'n': int(ms.size), 'p50_ms': round(float(np.percentile(ms, 50)), 2),
            'p95_ms': round(float(np.percentile(ms, 95)), 2),
            'max_ms': round(float(ms.max()), 2),
            'within_budget': bool(np.percentile(ms, 95) < BUDGET_MS and ms.max() < BUDGET_MS)}


def run(n):
    cfg = H.shipped_manoeuvre_config()
    rear_off = rear_probe_offsets(rh.WHEELBASE, rh.FRONT_X, rh.REAR_X,
                                  SOLVER_FRONT_POINT_OFFSETS_M)
    walls = [(0.0, 1.0, 0.5), (0.0, -1.0, 0.5)]
    env = RearEnv(grid=H.free_grid(), map_to_odom=(0.0, 0.0, 0.0), objects=[],
                  boundaries=walls, bt_allow=False, bt_allow_age_s=0.1)
    fake = rh._Stand()
    fake.goal_distance = 6.0
    fake.goal_start_xy = (0.0, 0.0)
    fake.goal_anchor_odom = (0.0, 0.0, 0.0)
    fake.psi_init_corridor = 0.0
    states = _states(n)

    def forward(x):
        corridor = MPCController.build_straight_corridor(fake, x)
        corridor['obstacles_world'] = []
        corridor['d_safe'] = rh.CAR_RADIUS + rh.AVOIDANCE_MARGIN
        corridor['car_radius'] = rh.CAR_RADIUS
        corridor['avoidance_margin'] = rh.AVOIDANCE_MARGIN
        pref = MPCController.compute_local_target(fake, x, corridor)
        return solve_mpc_step(
            x0=x, last_u=np.zeros(2), pref_nom=pref, corridor=corridor, horizon=rh.N,
            ts=rh.TS_PRED, params=rh.PARAMS, limits=rh.LIMITS, weights=dict(rh.WEIGHTS),
            obstacles=[], dmin=rh.CAR_RADIUS + rh.AVOIDANCE_MARGIN, vdes=rh.VDES,
            solver='rti', boundaries=walls, boundary_max_sources=3,
            boundary_slack_weight=rh.SLACK_W, control_period=rh.TS)

    def supervisor(mode):
        rear = RearSolver(cfg, H.rear_weights(), rh.LIMITS, rh.PARAMS, rear_off,
                          boundary_opts={'boundary_max_sources': 3,
                                         'boundary_slack_weight': rh.SLACK_W})
        c = H.shipped_manoeuvre_config(solver_mode=mode)
        return ReverseSupervisor(c, rear, rh.TS), rear

    out = {}
    # Corridor builds.
    t = []
    for x in states:
        fake.smoothed_target = None
        t0 = time.perf_counter()
        MPCController.build_straight_corridor(fake, x)
        t.append((time.perf_counter() - t0) * 1e3)
    out['forward_corridor_build'] = _stats(t)
    t = []
    for x in states:
        t0 = time.perf_counter()
        rear_corridor(x[0], x[1], x[2], cfg)
        t.append((time.perf_counter() - t0) * 1e3)
    out['rear_corridor_build'] = _stats(t)
    # Forward alone (today's tick).
    t = []
    for x in states:
        fake.smoothed_target = None
        t0 = time.perf_counter()
        forward(x)
        t.append((time.perf_counter() - t0) * 1e3)
    out['forward_alone'] = _stats(t)
    # Rear alone.
    sup, rear = supervisor('lazy')
    t = []
    for x in states:
        corridor = rear_corridor(x[0], x[1], x[2], cfg)
        t0 = time.perf_counter()
        rear.solve(x, 0.0, corridor, [], walls, rh.CAR_RADIUS, rh.AVOIDANCE_MARGIN, rh.TS)
        t.append((time.perf_counter() - t0) * 1e3)
    out['rear_alone'] = _stats(t)
    # Full ticks: normal driving (lazy, not stuck), stuck window (lazy, the
    # rear runs), "always" mode.
    for name, mode, stuck in (('tick_normal_driving_lazy', 'lazy', False),
                              ('tick_stuck_window_lazy', 'lazy', True),
                              ('tick_always_mode', 'always', False)):
        sup, rear = supervisor(mode)
        calls0 = rear.calls
        t = []
        for k, x in enumerate(states):
            fake.smoothed_target = None
            t0 = time.perf_counter()
            sup.before_forward(k * rh.TS, x, env)
            _u, info = forward(x)
            ok = bool(info.get('success')) and not stuck
            sup.after_forward(k * rh.TS, x, ok, info.get('x_pred'), True, False, None, env)
            t.append((time.perf_counter() - t0) * 1e3)
        out[name] = dict(_stats(t), rear_solves=rear.calls - calls0)
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description='Reverse manoeuvre timing')
    parser.add_argument('--n', type=int, default=1000)
    parser.add_argument('--json', help='also write the result here')
    args = parser.parse_args(argv)
    import platform
    res = {'host': platform.node(), 'machine': platform.machine(),
           'python': platform.python_version(), 'n': args.n, 'budget_ms': BUDGET_MS,
           'results': run(args.n)}
    for k, v in res['results'].items():
        print(f'{k:28s} {json.dumps(v)}')
    ok = all(v['within_budget'] for k, v in res['results'].items() if k.startswith('tick_'))
    print(f'BUDGET (p95 and max < {BUDGET_MS:.0f} ms on every tick configuration): '
          f'{"PASS" if ok else "FAIL"}')
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(res, fh, indent=1)
            fh.write('\n')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
