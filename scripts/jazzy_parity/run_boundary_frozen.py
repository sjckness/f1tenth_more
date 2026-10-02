#!/usr/bin/env python3
"""Phase 2 follow-up: run costmap_boundary.py's extraction on FROZEN inputs.

Plain Python 3 + NumPy only -- no ROS, no rclpy, no rosbag2 -- so the exact
same script runs on Thor (Jazzy, Python 3.12) and on the Orin (Humble,
Python 3.10, whatever NumPy it has). Keep it that way: no f-string tricks
newer than 3.6, no NumPy API newer than ~1.17.

Input: the .npz written by freeze_boundary_inputs.py -- for every recorded
/costmap/boundaries tick, the exact map grid + metadata, pose and
parameters that were passed to extract_boundary_constraints() /
front_clearance_from_extraction(). Nothing is re-derived from a bag here,
so two platforms running this on the same .npz see bit-identical inputs,
and any output difference is a difference in the code or the numerics.

costmap_boundary.py is loaded straight from its file in the given repo
checkout (not via the f1tenth_costmap package, whose __init__ may pull in
ROS). Its sha256 is recorded in the output so the comparison can confirm
both platforms ran the same source.

Output .npz, one row per tick:
  yaw                      yaw_from_quaternion() of the frozen quaternion
  {d}_present              extraction[d] is not None   (d = front/left/right)
  {d}_nx, {d}_ny, {d}_off  extraction[d]               (NaN when absent)
  {d}_car_dx/_car_dy/_dist nearest_occupied_in_window() raw hit
  {d}_cell_col/_cell_row   the grid cell that hit lies in (-1 when absent)
  n_constraints, front_clearance
plus meta_* strings (python/numpy versions, module sha256, git HEAD, ...).

Usage:
  python3 run_boundary_frozen.py --repo ~/dev_ws/f1tenth_more \\
      --inputs boundary_frozen_inputs.npz --out boundary_orin_outputs.npz
  (add --module path/to/costmap_boundary.py to run a specific copy of the
  source instead of the one in --repo's working tree)
"""
import argparse
import array
import hashlib
import importlib.util
import math
import os
import platform
import subprocess
import sys
import types

import numpy as np

MODULE_REL = os.path.join('src', 'f1tenth_costmap', 'f1tenth_costmap', 'costmap_boundary.py')
DIRECTIONS = ('front', 'left', 'right')


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def load_module(path):
    spec = importlib.util.spec_from_file_location('costmap_boundary_frozen', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def git_head(repo):
    try:
        out = subprocess.check_output(
            ['git', '-C', repo, 'rev-parse', 'HEAD'], stderr=subprocess.DEVNULL)
        return out.decode().strip()
    except Exception:
        return 'unknown'


def cell_of_hit(car_dx, car_dy, robot_x, robot_y, yaw, origin_x, origin_y, resolution):
    """Grid (col, row) the nearest-occupied hit lies in. The hit is a cell
    CENTER (costmap_boundary.py uses +0.5), so after rotating back to map
    frame it sits half a cell from every edge and floor() is unambiguous."""
    c, s = math.cos(yaw), math.sin(yaw)
    mx = robot_x + c * car_dx - s * car_dy
    my = robot_y + s * car_dx + c * car_dy
    return (int(math.floor((mx - origin_x) / resolution)),
            int(math.floor((my - origin_y) / resolution)))


def grid_for_map(inp, m):
    """The production node passes OccupancyGrid.data, which rclpy delivers
    as array.array('b'). Hand the module the same type."""
    off = inp['map_offsets']
    flat = inp['map_data'][int(off[m]):int(off[m + 1])]
    return array.array('b', flat.astype(np.int8).tobytes())


def run(inp, mod):
    n = len(inp['tick_bag_index'])
    fmax = float(inp['param_front_facing_max_rad'])
    smin = float(inp['param_side_window_min_rad'])
    smax = float(inp['param_side_window_max_rad'])
    occ = int(inp['param_occupied_threshold'])
    rng = float(inp['param_max_range_m'])
    windows = {'front': (-fmax, fmax), 'left': (smin, smax), 'right': (-smax, -smin)}

    out = {'tick_bag_index': np.asarray(inp['tick_bag_index'], dtype=np.int64),
           'yaw': np.full(n, np.nan),
           'n_constraints': np.zeros(n, dtype=np.int64),
           'front_clearance': np.full(n, np.nan)}
    for d in DIRECTIONS:
        out[d + '_present'] = np.zeros(n, dtype=bool)
        for k in ('nx', 'ny', 'off', 'car_dx', 'car_dy', 'dist'):
            out[d + '_' + k] = np.full(n, np.nan)
        out[d + '_cell_col'] = np.full(n, -1, dtype=np.int64)
        out[d + '_cell_row'] = np.full(n, -1, dtype=np.int64)

    grid_cache = {}
    for i in range(n):
        m = int(inp['tick_map_idx'][i])
        if m not in grid_cache:
            grid_cache = {m: grid_for_map(inp, m)}  # ticks are map-sorted runs
        grid = grid_cache[m]
        w = int(inp['map_width'][m])
        h = int(inp['map_height'][m])
        res = float(inp['map_resolution'][m])
        ox = float(inp['map_origin_x'][m])
        oy = float(inp['map_origin_y'][m])
        rx = float(inp['tick_pose_x'][i])
        ry = float(inp['tick_pose_y'][i])
        qx, qy, qz, qw = (float(v) for v in inp['tick_pose_quat_xyzw'][i])
        yaw = mod.yaw_from_quaternion(types.SimpleNamespace(x=qx, y=qy, z=qz, w=qw))
        out['yaw'][i] = yaw

        extraction = mod.extract_boundary_constraints(
            grid, w, h, res, ox, oy, rx, ry, yaw, fmax, smin, smax, occ, rng)
        out['front_clearance'][i] = mod.front_clearance_from_extraction(extraction, rng)

        for d in DIRECTIONS:
            r = extraction[d]
            if r is None:
                continue
            out['n_constraints'][i] += 1
            out[d + '_present'][i] = True
            out[d + '_nx'][i], out[d + '_ny'][i], out[d + '_off'][i] = r
            lo, hi = windows[d]
            hit = mod.nearest_occupied_in_window(
                grid, w, h, res, ox, oy, rx, ry, yaw, lo, hi, occ, rng)
            cdx, cdy, dist = hit
            out[d + '_car_dx'][i], out[d + '_car_dy'][i], out[d + '_dist'][i] = cdx, cdy, dist
            col, row = cell_of_hit(cdx, cdy, rx, ry, yaw, ox, oy, res)
            out[d + '_cell_col'][i] = col
            out[d + '_cell_row'][i] = row
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--repo', required=True, help='repo checkout to import costmap_boundary.py from')
    ap.add_argument('--module', help='explicit path to costmap_boundary.py (overrides --repo\'s copy, '
                                     'e.g. one extracted with `git show <rev>:<path>`)')
    ap.add_argument('--inputs', required=True, help='boundary_frozen_inputs.npz')
    ap.add_argument('--out', required=True, help='output .npz')
    ap.add_argument('--label', default=platform.node(), help='platform label stored in the output')
    args = ap.parse_args()

    repo = os.path.abspath(os.path.expanduser(args.repo))
    mod_path = os.path.abspath(os.path.expanduser(args.module)) if args.module \
        else os.path.join(repo, MODULE_REL)
    mod = load_module(mod_path)
    inp = dict(np.load(args.inputs, allow_pickle=False))

    out = run(inp, mod)
    meta = {
        'meta_label': args.label,
        'meta_python': sys.version,
        'meta_numpy': np.__version__,
        'meta_platform': platform.platform(),
        'meta_machine': platform.machine(),
        'meta_module_path': mod_path,
        'meta_module_sha256': sha256_file(mod_path),
        'meta_git_head': git_head(repo),
        'meta_inputs_sha256': sha256_file(args.inputs),
    }
    for k, v in meta.items():
        out[k] = np.array(v)
    np.savez_compressed(args.out, **out)

    n = len(out['tick_bag_index'])
    print('ticks run:        %d' % n)
    print('constraints:      %d' % int(out['n_constraints'].sum()))
    for k, v in meta.items():
        print('%-17s %s' % (k[5:] + ':', v.replace('\n', ' ')))
    print('wrote %s' % args.out)


if __name__ == '__main__':
    main()
