#!/usr/bin/env python3
"""Solve-time benchmark for mpc_corr's RTI solve, old vs new horizon.

Closed loop on the nominal model: a straight corridor, the car starting at
rest, optionally an obstacle centred in the corridor (R011-like: a 0.30 m
plant 1.4 m ahead). Every tick runs mpc_solver.solve_mpc_step exactly as
MPC_corr.control_loop does (warm start shifted by control_period / dt,
control_period passed for the first rate row) and times it with
time.perf_counter(). Only the solve is timed -- not the corridor build,
not the publishing.

Configurations (all N = 20):
  legacy   prediction dt 0.1, control period 0.1, lookahead 1.5 m
  campaign prediction dt 0.2, control period 0.1, lookahead 2.55 m

Run on the machine you care about (the Orin is slower than linus):
  source install/setup.bash
  python3 tools/mpc_solve_benchmark.py            # both configs, both scenes
  python3 tools/mpc_solve_benchmark.py --ticks 300 --repeat 5

Needs the 'osqp' Python package (the stack's RTI solver needs it too).
"""

import argparse
import time

import numpy as np

from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta

WHEELBASE = 0.305
LR = 0.17
PARAMS = {'L': WHEELBASE, 'lr': LR}
LIMITS = {
    'delta_min': -0.309, 'delta_max': 0.319,
    'a_min': -2.0, 'a_max': 3.0,
    'dDeltaMin': -0.5, 'dDeltaMax': 0.5,
    'dAMin': -2.0, 'dAMax': 2.0,
    'vMin': -1.0, 'vMax': 3.0,
}
# Shipping weights (stack_params.yaml mpc_w_*), before the per-stage scaling
# solve_mpc_step applies itself.
WEIGHTS = {
    'w_term': 9.0, 'w_v': 4.2857, 'w_psi': 4.5, 'w_psi_stage': 4.7736,
    'w_corr': 3.5714, 'w_u_a': 0.0, 'w_du_delta': 8.33, 'w_du_a': 0.0,
    'w_delta0': 0.0, 'w_obs': 8.0,
}
CAR_RADIUS = 0.20
MARGIN = 0.12

CONFIGS = {
    'legacy': {'N': 20, 'dt': 0.1, 'period': 0.1, 'lookahead': 1.5},
    'campaign': {'N': 20, 'dt': 0.2, 'period': 0.1, 'lookahead': 2.55},
}


def corridor_at(x0, half_width, length=3.0, n=120):
    s = np.linspace(0.0, length, n) + x0
    zeros = np.zeros(n)
    return {
        'xc': s, 'yc': zeros.copy(), 'xL': s, 'yL': zeros + half_width,
        'xR': s, 'yR': zeros - half_width,
        'tx': np.ones(n), 'ty': zeros.copy(), 'nx': zeros.copy(), 'ny': np.ones(n),
        'halfWidth': zeros + half_width, 'psiRef': 0.0, 'psiRefTurn': None,
        'L': length, 'Pend': np.array([x0 + length, 0.0]),
        'car_radius': CAR_RADIUS, 'avoidance_margin': MARGIN,
    }


def run(cfg, ticks, half_width, obstacles, vdes=0.4):
    x = np.zeros(4)
    last_u = np.zeros(2)
    warm = None
    times = []
    failures = 0
    for _ in range(ticks):
        corridor = corridor_at(x[0], half_width)
        corridor['obstacles_world'] = obstacles
        pref = np.array([x[0] + cfg['lookahead'], 0.0])
        t0 = time.perf_counter()
        u0, info = solve_mpc_step(
            x0=x, last_u=last_u, pref_nom=pref, corridor=corridor,
            horizon=cfg['N'], ts=cfg['dt'], params=PARAMS, limits=LIMITS,
            weights=dict(WEIGHTS), obstacles=obstacles,
            dmin=CAR_RADIUS + MARGIN, vdes=vdes, solver='rti',
            warm_start_z=warm, control_period=cfg['period'])
        times.append(time.perf_counter() - t0)
        if info.get('success'):
            warm = shift_warm_start(info['zopt'], cfg['N'],
                                    shift=cfg['period'] / cfg['dt'])
        else:
            failures += 1
            warm = None
        v_cmd = min(max(x[3] + u0[1] * cfg['period'], 0.0), 0.5)
        u_plant = np.array([u0[0], (v_cmd - x[3]) / cfg['period']])
        x = np.array(f1tenth_state_fcn_dt_beta(x, u_plant, cfg['period'], WHEELBASE, LR))
        last_u = np.asarray(u0, dtype=float)
    return np.asarray(times) * 1e3, failures


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--ticks', type=int, default=150)
    ap.add_argument('--repeat', type=int, default=3)
    ap.add_argument('--half-width', type=float, default=0.6,
                    help='corridor half-width [m] (0.4333 = pre-campaign)')
    args = ap.parse_args()

    scenes = {
        'free': [],
        'plant_1.4m': [(1.4, 0.0, 0.15)],
    }
    print(f'{"config":9s} {"scene":11s} {"N":>3s} {"dt":>5s} {"horizon":>8s} '
          f'{"reach@0.4":>9s} {"mean":>7s} {"p95":>7s} {"max":>7s} {"fail":>5s}  [ms]')
    for name, cfg in CONFIGS.items():
        for scene, obstacles in scenes.items():
            all_t = []
            fails = 0
            for _ in range(args.repeat):
                t, f = run(cfg, args.ticks, args.half_width, obstacles)
                all_t.append(t[1:])  # first tick includes imports/JIT warm-up
                fails += f
            t = np.concatenate(all_t)
            horizon = cfg['N'] * cfg['dt']
            print(f'{name:9s} {scene:11s} {cfg["N"]:3d} {cfg["dt"]:5.2f} '
                  f'{horizon:7.1f}s {horizon * 0.4:8.2f}m {t.mean():7.2f} '
                  f'{np.percentile(t, 95):7.2f} {t.max():7.2f} {fails:5d}')


if __name__ == '__main__':
    main()
