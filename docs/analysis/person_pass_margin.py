#!/usr/bin/env python3
"""Reproduce docs/analysis/2026-09-17_person_pass_margin.md's rig tables.

Runs mpc_controller's test_person_pass_closed_loop.run_pass for legacy and
footprint radii at class margins 0.0 / 0.2 / 0.3 / 0.4 and lateral offsets
0.0 / 0.3 / 0.6 m, with the camera's real half field of view and, for
attribution, with the person never leaving view. Nominal model only.

    python3 docs/analysis/person_pass_margin.py
"""

import math
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src/f1tenth_control/mpc_controller/test'))
import test_person_pass_closed_loop as passing  # noqa: E402


def table(title, half_fov):
    print(f'\n### {title}\n')
    print('| obstacle | offset m | min centre m | min body gap m | dropout tick | '
          'peak cross-track m | peak speed m/s | goal reached |')
    print('|---|---|---|---|---|---|---|---|')
    for label, radius in passing.RADIUS_ROWS:
        for offset in passing.OFFSETS_M:
            res = passing.run_pass(radius, offset, half_fov=half_fov)
            dropout = '-' if res['dropout_tick'] is None else res['dropout_tick']
            reached = 'no' if res['reached_s'] is None else f'{res["reached_s"]:.1f} s'
            print(f'| {label} | {offset:.1f} | {res["min_centre"]:.3f} | '
                  f'{res["min_body_gap"]:+.3f} | {dropout} | {res["peak_cross_track"]:.3f} | '
                  f'{res["peak_speed"]:.2f} | {reached} |')


if __name__ == '__main__':
    table(f'With dropout at the real half-FOV '
          f'({math.degrees(passing.HALF_FOV_RAD):.2f} deg)', passing.HALF_FOV_RAD)
    table('Attribution only: the person never leaves view', math.pi)
