#!/usr/bin/env python3
"""Reproduce docs/analysis/2026-09-16_chase_overspeed.md's rig numbers.

The unbounded lateral chase from the object-approach closed-loop rig: a person
walking sideways at 0.3 m/s past a car approaching at standoff 1.0, footprint
radius in the obstacle list. Prints peak forward/reverse speed for the
baseline, one cost term removed at a time, and with the /drive clamp.

Nominal model only. Run from the repo root:
    python3 docs/analysis/chase_overspeed_ablation.py
"""

import pathlib
import sys

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src/f1tenth_control/mpc_controller/test'))
import test_object_approach_closed_loop as rig  # noqa: E402

BASE = dict(rig.WEIGHTS)


def run(label, weights=None, obstacle_r=rig.PERSON_RADIUS['footprint'], speed_limits=None):
    rig.WEIGHTS.clear()
    rig.WEIGHTS.update(BASE)
    rig.WEIGHTS.update(weights or {})
    res = rig.run_approach(target_fn=lambda t: (4.0, 0.3 * t), duration=40.0, standoff=1.0,
                           obstacle_r=obstacle_r, stop_at_rest=False,
                           speed_limits=speed_limits)
    v = np.array([s[3] for s in res.xs])
    print(f'| {label} | {v.max():+.3f} (tick {int(v.argmax())}) | '
          f'{v.min():+.3f} (tick {int(v.argmin())}) | {int((v > 1.0).sum())} | '
          f'{int((v < -0.01).sum())} |')


if __name__ == '__main__':
    print('| run | peak forward m/s | peak reverse m/s | ticks > 1.0 | ticks < -0.01 |')
    print('|---|---|---|---|---|')
    run('baseline (footprint, standoff 1.0)')
    run('w_obs = 0', {'w_obs': 0.0})
    run('no obstacle in the list', obstacle_r=None)
    run('w_term = 0', {'w_term': 0.0})
    run('w_corr = 0', {'w_corr': 0.0})
    run('w_psi = 0 and w_psi_stage = 0', {'w_psi': 0.0, 'w_psi_stage': 0.0})
    run('w_v x 10', {'w_v': BASE['w_v'] * 10.0})
    run('legacy radius', obstacle_r=rig.PERSON_RADIUS['legacy'])
    run('baseline + /drive clamp (+1.0 / -0.0)', speed_limits=(1.0, 0.0))
    rig.WEIGHTS.clear()
    rig.WEIGHTS.update(BASE)
