#!/usr/bin/env python3
"""Parity: costmap_boundary_node (Python) vs costmap_boundary_node (Rust).

IDENTICAL inputs (an OccupancyGrid and a map-frame pose) go to
  - the Python: the REAL CostmapBoundaryNode._extraction_tick (f1tenth_costmap,
    use_convex_polytope false), its two publishers replaced by recorders, and
  - the Rust: costmap_boundary_replay (f1tenth_costmap_boundary_rs::extraction,
    the code the node runs).
Compared per pose: constraint count, each (normal_x, normal_y, offset) in the
published order front/left/right, and /costmap/front_clearance (Float32);
tolerances below. Header stamp/frame_id are not part of the extraction: the
live test (live_costmap_test.py) checks them on the running nodes.

Cases: synthetic maps (empty, unknown, walls, corridor, corners, clutter,
values around the threshold, ties, odd resolutions/origins, the car outside
the mapped area, malformed grids) and every /slam/map of the recorded bags
with the /ekf_global/odometry/filtered poses received while it was current.

Usage (workspace sourced): parity_costmap_boundary.py [--bags N] [--out FILE]
"""

import argparse
import glob
import json
import math
import os
import random
import subprocess
import sys
import time

import numpy as np

# Tolerances. Same libm, same order of operations: expected bit-exact; any
# difference above these fails the gate.
TOL_NORMAL = 1e-12
TOL_OFFSET = 1e-12          # m
TOL_CLEARANCE_ULP = 0       # Float32 ulps

ARCHIVES = ['~/f1tenth_archive/mission_row', '~/f1tenth_sim_archive/mission_row']


class _Recorder:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class PythonRef:
    """One CostmapBoundaryNode, ticked by hand."""

    def __init__(self):
        import rclpy
        from f1tenth_costmap.costmap_boundary_node import CostmapBoundaryNode
        if not rclpy.ok():
            rclpy.init()
        self.node = CostmapBoundaryNode()
        self.node.timer.cancel()
        self.boundaries = self.node.boundary_pub = _Recorder()
        self.clearance = self.node.clearance_pub = _Recorder()
        self.tick_s = []

    def params(self):
        from f1tenth_description.vehicle_frames import (
            base_front_x, front_side_limit_m, MIN_FRONT_DISTANCE_M)
        n = self.node
        return dict(front_facing_max_deg=math.degrees(n.front_facing_max_rad),
                    side_window_min_deg=math.degrees(n.side_window_min_rad),
                    side_window_max_deg=math.degrees(n.side_window_max_rad),
                    occupied_threshold=n.occupied_threshold, max_range_m=n.max_range_m,
                    front_x=base_front_x(), front_side_limit_m=front_side_limit_m(),
                    min_front_distance_m=MIN_FRONT_DISTANCE_M)

    def run(self, grid_msg, pose):
        self.node._latest_grid = grid_msg
        self.node._latest_pose_xy_yaw = tuple(pose)
        self.boundaries.msgs.clear()
        self.clearance.msgs.clear()
        t0 = time.perf_counter()
        try:
            self.node._extraction_tick()
        except Exception as exc:  # the node would die here; recorded, compared below
            return dict(raised=f'{type(exc).__name__}: {exc}'[:200])
        finally:
            self.tick_s.append(time.perf_counter() - t0)
        arr = self.boundaries.msgs[-1]
        return dict(raised=None, frame_id=arr.header.frame_id,
                    constraints=[[float(c.normal[0]), float(c.normal[1]), float(c.offset)]
                                 for c in arr.constraints],
                    front_clearance=(float(self.clearance.msgs[-1].data)
                                     if self.clearance.msgs else None))


def grid_msg(data, width, height, resolution, ox, oy):
    from nav_msgs.msg import OccupancyGrid
    m = OccupancyGrid()
    m.header.frame_id = 'map'
    m.info.width, m.info.height = int(width), int(height)
    m.info.resolution = float(resolution)
    m.info.origin.position.x, m.info.origin.position.y = float(ox), float(oy)
    m.info.origin.orientation.w = 1.0
    m.data = [int(v) for v in data]
    return m


def _enc(x):
    x = float(x)
    if math.isnan(x):
        return 'nan'
    if math.isinf(x):
        return 'inf' if x > 0 else '-inf'
    return x


def _dec(x):
    return float(x) if x is not None else None


# ---------------------------------------------------------------- synthetic
def synthetic_maps(rng):
    """(name, data, width, height, resolution, ox, oy, poses)."""
    out = []

    def poses_inside(w, h, res, ox, oy, k):
        return [(ox + rng.uniform(0.1, 0.9) * w * res, oy + rng.uniform(0.1, 0.9) * h * res,
                 rng.uniform(-math.pi, math.pi)) for _ in range(k)]

    W = H = 160
    res, ox, oy = 0.05, -4.0, -4.0
    empty = np.zeros((H, W), dtype=np.int16)
    out.append(('empty', empty, res, ox, oy, poses_inside(W, H, res, ox, oy, 30)))
    out.append(('all_unknown', np.full((H, W), -1, dtype=np.int16), res, ox, oy,
                poses_inside(W, H, res, ox, oy, 10)))
    out.append(('all_occupied', np.full((H, W), 100, dtype=np.int16), res, ox, oy,
                poses_inside(W, H, res, ox, oy, 10)))
    room = empty.copy()
    room[0:3, :] = room[-3:, :] = 100
    room[:, 0:3] = room[:, -3:] = 100
    out.append(('room_walls', room, res, ox, oy, poses_inside(W, H, res, ox, oy, 120)
                + [(0.0, 0.0, 0.0), (0.0, 0.0, math.pi), (0.0, 0.0, -math.pi),
                   (0.0, 0.0, math.pi / 2), (0.025, 0.025, 0.0)]))
    corridor = np.full((H, W), -1, dtype=np.int16)
    corridor[60:100, :] = 0
    corridor[58:60, :] = 100
    corridor[100:102, :] = 100
    out.append(('corridor', corridor, res, ox, oy,
                [(ox + x * res, 0.0, yaw) for x in range(10, 150, 7)
                 for yaw in (0.0, 0.3, -0.3, 1.2, math.pi)]))
    corner = empty.copy()
    corner[80:83, 80:] = 100
    corner[80:, 80:83] = 100
    out.append(('corner', corner, res, ox, oy, poses_inside(W, H, res, ox, oy, 120)))
    clutter = empty.copy()
    for _ in range(300):
        r, c = rng.randrange(H), rng.randrange(W)
        clutter[r:r + rng.randint(1, 4), c:c + rng.randint(1, 4)] = rng.choice([100, 70, 65, 64, -1])
    out.append(('clutter', clutter, res, ox, oy, poses_inside(W, H, res, ox, oy, 200)))
    thr = empty.copy()
    for i, v in enumerate((63, 64, 65, 66, 99, 100, -1, 0)):
        thr[70 + 2 * i, 90:100] = v
    out.append(('values_around_threshold', thr, res, ox, oy, [(0.0, 0.0, 0.0), (0.0, 0.2, 0.0),
                                                               (0.3, 0.1, 0.5)]))
    # Exact ties: one cell at each of the 4 diagonals / axes, robot at a cell centre.
    tie = empty.copy()
    for dr, dc in ((10, 10), (-10, 10), (10, -10), (-10, -10), (0, 20), (20, 0), (0, -20), (-20, 0)):
        tie[80 + dr, 80 + dc] = 100
    centre = (ox + 80.5 * res, oy + 80.5 * res)
    out.append(('ties', tie, res, ox, oy, [(centre[0], centre[1], yaw)
                                           for yaw in (0.0, math.pi / 4, math.pi / 2, math.pi,
                                                       -math.pi / 2, -3 * math.pi / 4)]))
    # The car outside the mapped area: far away, partly overlapping, at the edge.
    out.append(('outside_far', room, res, ox, oy, [(50.0, 50.0, 0.0), (-50.0, 3.0, 1.0),
                                                    (1e6, -1e6, 0.0)]))
    out.append(('outside_partial', room, res, ox, oy,
                [(ox - 1.0, 0.0, 0.0), (ox - 1.0, 0.0, math.pi), (ox - 4.9, 0.0, 0.0),
                 (ox - 5.1, 0.0, 0.0), (ox + W * res + 2.0, oy - 2.0, 2.5), (ox, oy, 0.7)]))
    # Odd resolutions and origins.
    for r2, o2 in ((0.1, (-3.0, -2.0)), (0.025, (-2.0, -2.0)), (0.05, (0.0123, -7.77)),
                   (0.0499999, (-4.0, -4.0))):
        n = int(8.0 / r2)
        g = np.zeros((n, n), dtype=np.int16)
        g[n // 3, :] = 100
        g[:, 2 * n // 3] = 100
        out.append((f'res{r2}_origin{o2}', g, r2, o2[0], o2[1],
                    poses_inside(n, n, r2, o2[0], o2[1], 40)))
    # Non-square map.
    g = np.zeros((40, 300), dtype=np.int16)
    g[0, :] = g[-1, :] = 100
    out.append(('non_square', g, 0.05, -1.0, -1.0, [(x, 0.0, 0.0) for x in np.linspace(-1, 14, 31)]))
    return out


def malformed_cases(rng):
    """Grids/poses the Python cannot process: (name, data, w, h, res, ox, oy, poses)."""
    base = np.zeros(100, dtype=np.int16)
    return [
        ('data_too_short', base[:99], 10, 10, 0.1, 0.0, 0.0, [(0.5, 0.5, 0.0)]),
        ('data_too_long', np.zeros(101, dtype=np.int16), 10, 10, 0.1, 0.0, 0.0, [(0.5, 0.5, 0.0)]),
        ('resolution_zero', base, 10, 10, 0.0, 0.0, 0.0, [(0.5, 0.5, 0.0)]),
        ('pose_nan', base, 10, 10, 0.1, 0.0, 0.0, [(math.nan, 0.5, 0.0), (0.5, 0.5, math.nan)]),
        ('pose_inf', base, 10, 10, 0.1, 0.0, 0.0, [(math.inf, 0.5, 0.0)]),
        ('empty_grid', np.zeros(0, dtype=np.int16), 0, 0, 0.05, 0.0, 0.0, [(0.0, 0.0, 0.0)]),
    ]


# ---------------------------------------------------------------- bags
def find_bags():
    import yaml
    bags = []
    for root in ARCHIVES:
        for meta in sorted(glob.glob(os.path.join(os.path.expanduser(root), '*', 'bag',
                                                  'metadata.yaml'))):
            try:
                info = yaml.safe_load(open(meta))['rosbag2_bagfile_information']
            except Exception:
                continue
            c = {t['topic_metadata']['name']: t['message_count']
                 for t in info.get('topics_with_message_count', [])}
            if c.get('/slam/map', 0) and c.get('/ekf_global/odometry/filtered', 0):
                bags.append((os.path.dirname(meta), info['storage_identifier']))
    return bags


def bag_maps(bag_dir, storage, max_poses_per_map):
    """[(name, OccupancyGrid, [poses received while it was the current map])]."""
    import rosbag2_py
    from costmap_yaw import yaw
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.serialization import deserialize_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/slam/map', '/ekf_global/odometry/filtered']))
    out, cur, poses = [], None, []
    name = os.path.basename(os.path.dirname(bag_dir))
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/slam/map':
            if cur is not None and poses:
                out.append((f'{name}#map{len(out)}', cur, poses[:max_poses_per_map]))
            cur, poses = deserialize_message(data, OccupancyGrid), []
        elif cur is not None:
            p = deserialize_message(data, Odometry).pose.pose
            poses.append((p.position.x, p.position.y, yaw(p.orientation)))
    if cur is not None and poses:
        out.append((f'{name}#map{len(out)}', cur, poses[:max_poses_per_map]))
    return out


# ---------------------------------------------------------------- compare
def run_rust(replay, lines):
    t0 = time.perf_counter()
    p = subprocess.run([replay], input='\n'.join(lines) + '\n', capture_output=True,
                       text=True, check=False)
    dt = time.perf_counter() - t0
    if p.returncode != 0:
        raise RuntimeError(f'replay failed: {p.stderr[:2000]}')
    return [json.loads(line) for line in p.stdout.splitlines() if line.strip()], dt


def f32_ulps(a, b):
    ia = np.float32(a).view(np.int32).astype(np.int64)
    ib = np.float32(b).view(np.int32).astype(np.int64)
    return int(abs(ia - ib))


def compare_case(name, group, msg, poses, py, replay, params, stats):
    line = json.dumps(dict(id=name, width=msg.info.width, height=msg.info.height,
                           resolution=float(msg.info.resolution),
                           origin_x=msg.info.origin.position.x,
                           origin_y=msg.info.origin.position.y,
                           data=[int(v) for v in msg.data],
                           poses=[[_enc(v) for v in p] for p in poses], **params))
    rust, dt = run_rust(replay, [line])
    stats['rs_time_s'] += dt
    if len(rust) != len(poses):
        raise RuntimeError(f'{name}: {len(rust)} Rust results for {len(poses)} poses')
    g = stats['groups'].setdefault(group, dict(maps=0, poses=0, diffs=0, py_raised=0,
                                               constraints=0, empty=0))
    g['maps'] += 1
    for k, (pose, rs) in enumerate(zip(poses, rust)):
        g['poses'] += 1
        ref = py.run(msg, pose)
        if ref['raised']:
            g['py_raised'] += 1
            stats['py_raised'].append(dict(case=f'{name}[{k}]', python=ref['raised'],
                                           rust=rs['error']))
            if rs['error'] is None or rs['constraints'] or rs['front_clearance'] is not None:
                stats['diffs'].append(dict(case=f'{name}[{k}]', why='python raised, rust '
                                           'did not publish empty/withheld', rust=rs))
            continue
        why = []
        if (rs['error'] is not None and not ref['constraints']
                and ref['front_clearance'] == params['max_range_m'] + 1.0):
            # INTENDED: the Python did not raise but published the fail-OPEN
            # pair (no constraint, "clear" max_range + 1) for an input it
            # cannot have processed (a NaN yaw: every distance NaN, no cell
            # passes dist <= max_range). The Rust publishes empty + withheld.
            g['intended'] = g.get('intended', 0) + 1
            stats['intended'].append(dict(case=f'{name}[{k}]', pose=pose, rust=rs['error'],
                                          python='no constraints, front_clearance '
                                                 f'{ref["front_clearance"]}'))
            continue
        if rs['error'] is not None:
            why.append(f'rust error {rs["error"]}')
        pc, rc = ref['constraints'], rs['constraints']
        if len(pc) != len(rc):
            why.append(f'count {len(pc)} vs {len(rc)}')
        else:
            for a, b in zip(pc, rc):
                b = [_dec(v) if not isinstance(v, str) else float(v) for v in b]
                dn = max(abs(a[0] - b[0]), abs(a[1] - b[1]))
                do = abs(a[2] - b[2]) if math.isfinite(a[2]) else (0.0 if a[2] == b[2] else math.inf)
                if dn > stats['max_normal_diff']:
                    stats['max_normal_diff'] = dn
                    stats['max_normal_case'] = dict(case=f'{name}[{k}]', pose=pose, py=a, rs=b)
                stats['max_offset_diff'] = max(stats['max_offset_diff'], do)
                stats['bitexact_constraints'] += int(a == b)
                stats['compared_constraints'] += 1
                if dn > TOL_NORMAL or do > TOL_OFFSET:
                    why.append(f'constraint {a} vs {b}')
        g['constraints'] += len(pc)
        g['empty'] += int(not pc)
        pcl = ref['front_clearance']
        rcl = rs['front_clearance']
        rcl = float(rcl) if rcl is not None else None
        if (pcl is None) != (rcl is None):
            why.append(f'clearance presence {pcl} vs {rcl}')
        elif pcl is not None:
            u = f32_ulps(pcl, rcl)
            stats['max_clearance_ulps'] = max(stats['max_clearance_ulps'], u)
            if u > TOL_CLEARANCE_ULP:
                why.append(f'clearance {pcl} vs {rcl}')
            stats['clearance_values'].setdefault(group, []).append(pcl)
        if ref['frame_id'] != 'base_link':
            why.append(f'python frame_id {ref["frame_id"]}')
        if why:
            g['diffs'] += 1
            if len(stats['diffs']) < 50:
                stats['diffs'].append(dict(case=f'{name}[{k}]', pose=pose, why=why))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--replay', default=None)
    ap.add_argument('--bags', type=int, default=0, help='max bags (0: all)')
    ap.add_argument('--max-poses-per-map', type=int, default=400)
    ap.add_argument('--seed', type=int, default=20261018)
    ap.add_argument('--out', default='output/rust/parity_costmap_boundary.json')
    a = ap.parse_args()
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    replay = a.replay
    if replay is None:
        from ament_index_python.packages import get_package_prefix
        replay = os.path.join(get_package_prefix('f1tenth_costmap_boundary_rs'), 'lib',
                              'f1tenth_costmap_boundary_rs', 'costmap_boundary_replay')
    rng = random.Random(a.seed)
    py = PythonRef()
    params = py.params()
    stats = dict(groups={}, diffs=[], py_raised=[], intended=[], max_normal_diff=0.0, max_offset_diff=0.0,
                 max_clearance_ulps=0, bitexact_constraints=0, compared_constraints=0,
                 clearance_values={}, rs_time_s=0.0, params=params, bags=[],
                 tolerances=dict(normal=TOL_NORMAL, offset_m=TOL_OFFSET,
                                 clearance_f32_ulps=TOL_CLEARANCE_ULP))
    for name, g, res, ox, oy, poses in synthetic_maps(rng):
        h, w = g.shape
        compare_case(name, 'synthetic', grid_msg(g.ravel(), w, h, res, ox, oy), poses, py,
                     replay, params, stats)
    for name, data, w, h, res, ox, oy, poses in malformed_cases(rng):
        compare_case(name, 'malformed', grid_msg(data, w, h, res, ox, oy), poses, py, replay,
                     params, stats)
    print('synthetic+malformed:', json.dumps({k: v for k, v in stats['groups'].items()}),
          flush=True)
    bags = find_bags()
    if a.bags:
        bags = bags[:a.bags]
    for n, (bag, storage) in enumerate(bags):
        try:
            maps = bag_maps(bag, storage, a.max_poses_per_map)
        except Exception as exc:
            stats['bags'].append(dict(bag=bag, error=repr(exc)))
            continue
        for name, msg, poses in maps:
            compare_case(name, 'bag', msg, poses, py, replay, params, stats)
        stats['bags'].append(dict(bag=bag, maps=len(maps), poses=sum(len(p) for _, _, p in maps)))
        if n % 20 == 0:
            print(f'bag {n + 1}/{len(bags)}: {json.dumps(stats["groups"].get("bag", {}))}',
                  flush=True)
    stats['py_tick_ms'] = dict(
        p50=float(np.percentile(py.tick_s, 50) * 1e3), p95=float(np.percentile(py.tick_s, 95) * 1e3),
        max=float(np.max(py.tick_s) * 1e3), n=len(py.tick_s))
    stats['clearance_values'] = {k: dict(n=len(v), min=min(v), max=max(v),
                                         n_clear=sum(1 for x in v if x >= params['max_range_m'] + 1.0))
                                 for k, v in stats['clearance_values'].items()}
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    with open(a.out, 'w') as f:
        json.dump(stats, f, indent=1, default=str)
    total_diffs = sum(g['diffs'] for g in stats['groups'].values())
    print(json.dumps({k: stats[k] for k in ('groups', 'max_normal_diff', 'max_offset_diff',
                                             'max_clearance_ulps', 'bitexact_constraints',
                                             'compared_constraints', 'py_tick_ms')}, indent=1))
    print(f'TOTAL diffs {total_diffs}; python raised on {len(stats["py_raised"])} poses')
    return 0 if total_diffs == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
