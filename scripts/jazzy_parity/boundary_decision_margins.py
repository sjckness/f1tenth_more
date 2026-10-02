#!/usr/bin/env python3
"""Phase 2 follow-up: how close is each frozen tick to a nearest-cell flip?

nearest_occupied_in_window() makes three discrete decisions per window:
which occupied cell is nearest (argmin), whether a cell's bearing is inside
the window, and whether its distance is inside max_range_m. A 1-ULP
platform difference can only change the chosen cell if one of those is
within a few ULP of a tie. This script recomputes, for every frozen tick and
direction, using the same array maths as costmap_boundary.py:

  gap_m       second-nearest minus nearest candidate distance (exact ties = 0)
  angle_m     min |bearing - window edge| over occupied in-range cells
  range_m     min |dist - max_range_m| over occupied in-window cells

and reports the smallest of each. If all are many orders of magnitude above
the ~1e-16 m scale of a ULP at these coordinates, no ULP-level numerics
difference can flip a decision on these inputs, independent of platform.

Plain Python 3 + NumPy, no ROS.
"""
import argparse
import math
import types

import numpy as np

import run_boundary_frozen as frozen


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--repo', required=True)
    ap.add_argument('--inputs', required=True)
    args = ap.parse_args()

    import os
    mod = frozen.load_module(os.path.join(os.path.abspath(args.repo), frozen.MODULE_REL))
    inp = dict(np.load(args.inputs, allow_pickle=False))
    fmax = float(inp['param_front_facing_max_rad'])
    smin = float(inp['param_side_window_min_rad'])
    smax = float(inp['param_side_window_max_rad'])
    occ = int(inp['param_occupied_threshold'])
    rng = float(inp['param_max_range_m'])
    windows = {'front': (-fmax, fmax), 'left': (smin, smax), 'right': (-smax, -smin)}

    worst = {k: (math.inf, None) for k in ('gap', 'angle', 'range')}
    n_exact_ties = 0
    for i in range(len(inp['tick_bag_index'])):
        m = int(inp['tick_map_idx'][i])
        w, h = int(inp['map_width'][m]), int(inp['map_height'][m])
        res = float(inp['map_resolution'][m])
        ox, oy = float(inp['map_origin_x'][m]), float(inp['map_origin_y'][m])
        rx, ry = float(inp['tick_pose_x'][i]), float(inp['tick_pose_y'][i])
        q = [float(v) for v in inp['tick_pose_quat_xyzw'][i]]
        yaw = mod.yaw_from_quaternion(types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]))
        off = inp['map_offsets']
        arr = inp['map_data'][int(off[m]):int(off[m + 1])].astype(np.int16).reshape(h, w)

        c0, c1, r0, r1 = mod._crop_indices(w, h, res, ox, oy, rx, ry, rng)
        sub = arr[r0:r1, c0:c1]
        gx, gy = np.meshgrid(ox + (np.arange(c0, c1) + 0.5) * res,
                             oy + (np.arange(r0, r1) + 0.5) * res)
        dx, dy = gx - rx, gy - ry
        cy, sy = math.cos(yaw), math.sin(yaw)
        cdx, cdy = cy * dx + sy * dy, -sy * dx + cy * dy
        dist = np.hypot(cdx, cdy)
        ang = np.arctan2(cdy, cdx)
        occm = sub >= occ

        for d, (lo, hi) in windows.items():
            inwin = (ang >= lo) & (ang <= hi)
            inrng = dist <= rng
            cand = np.sort(dist[occm & inwin & inrng])
            if len(cand) >= 2:
                g = float(cand[1] - cand[0])
                if g == 0.0:
                    n_exact_ties += 1
                if g < worst['gap'][0]:
                    worst['gap'] = (g, (i, d))
            sel = occm & inrng
            if sel.any():
                a = float(np.min(np.minimum(np.abs(ang[sel] - lo), np.abs(ang[sel] - hi))))
                if a < worst['angle'][0]:
                    worst['angle'] = (a, (i, d))
            sel = occm & inwin
            if sel.any():
                r = float(np.min(np.abs(dist[sel] - rng)))
                if r < worst['range'][0]:
                    worst['range'] = (r, (i, d))

    print('=== decision margins over %d frozen ticks x 3 windows ===' % len(inp['tick_bag_index']))
    print('exact distance ties between the two nearest candidates: %d' % n_exact_ties)
    for k, label, unit in (('gap', 'nearest vs 2nd-nearest distance gap', 'm'),
                           ('angle', 'occupied cell bearing to window edge', 'rad'),
                           ('range', 'occupied cell distance to max_range_m', 'm')):
        v, where = worst[k]
        print('min %-40s %.3e %s  (tick %s, %s)' % (label + ':', v, unit,
                                                     where[0] if where else '-',
                                                     where[1] if where else '-'))


if __name__ == '__main__':
    main()
