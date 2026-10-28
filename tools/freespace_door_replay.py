#!/usr/bin/env python3
"""Run freespace_doors.py over a recorded bag, offline, with no ROS running.

The grid door detector's cycle is a pure function of (/slam/map,
/ekf_global/odometry/filtered, /scan, /tf_static), and a bag has all four --
so the whole detector can be replayed without a car, a simulator or an rclpy
node. That is the only way it has been exercised against a REAL slam_toolbox
grid so far.

Usage (no ROS environment needed; numpy, scipy, pyyaml, rosbags):

    python3 tools/freespace_door_replay.py ~/bags/<run> [--out output/doors]
                                           [--rate-hz 2.0] [--window-m 8.0]

Parameters come from stack_params.yaml, the car's own values, so a replay
answers "what would the stack have found", not "what can this code find if
tuned for this bag". Overrides exist for the two knobs worth sweeping.

Same sqlite3 + rosbags decoding as tools/semantic_door_replay.py and
tools/obstacles_v2_replay.py, whose helpers this reuses rather than
reimplementing (TOPICS is swapped for ours first -- an old bag's other topics
may not decode against today's .msg files, and reading them would fail the
whole bag).

Writes <out>/freespace_replay.md (the summary) and
<out>/freespace_cycles.csv (one row per cycle) next to the semantic
detector's own replay output.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src/f1tenth_costmap'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import obstacles_v2_replay as v2  # noqa: E402
from f1tenth_costmap import freespace_doors as fd  # noqa: E402
from f1tenth_costmap import semantic_doors as sd  # noqa: E402
from f1tenth_costmap.semantic_layer import find_nearest_pose_by_stamp  # noqa: E402

#: Only what this replay reads. See the module docstring.
TOPICS = ('/slam/map', '/scan', '/tf_static',
          '/ekf_global/odometry/filtered', '/odometry/filtered')


def freespace_params(params, **overrides):
    """A FreespaceParams from stack_params.yaml -- the car's own values."""
    def p(key):
        return params['freespace_doors_' + key]
    kw = dict(
        window_m=float(p('window_m')),
        occupied_threshold=int(p('occupied_threshold')),
        cleanup_iterations=int(p('cleanup_iterations')),
        width_min_m=float(p('width_min_m')),
        width_max_m=float(p('width_max_m')),
        rise_ratio=float(p('rise_ratio')),
        rise_within_m=float(p('rise_within_m')),
        spur_prune_m=float(p('spur_prune_m')),
        local_min_radius_m=float(p('local_min_radius_m')),
        tangent_span_m=float(p('tangent_span_m')),
        symmetry_tol_m=float(p('symmetry_tol_m')),
        jamb_radius_m=float(p('jamb_radius_m')),
        jamb_min_cells=int(p('jamb_min_cells')),
        min_beams=int(p('min_beams')),
        beyond_m=float(p('beyond_m')),
        jamb_exclude_m=float(p('jamb_exclude_m')),
        assoc_max_m=float(p('assoc_max_m')),
        confirm_hits=int(p('confirm_hits')),
        smooth_alpha=float(p('smooth_alpha')),
        candidate_forget_s=float(p('candidate_forget_s')),
        forget_unseen_s=float(p('forget_unseen_s')),
        max_candidates=int(p('max_candidates')))
    kw.update({k: v for k, v in overrides.items() if v is not None})
    return fd.FreespaceParams(**kw)


def laser_in_base(resolve, frame_id):
    """(x, y, yaw) of the scan frame in base_link, from /tf_static, or None."""
    m = resolve(frame_id)
    if m is None:
        return None
    return (float(m[0, 3]), float(m[1, 3]), math.atan2(float(m[1, 0]), float(m[0, 0])))


def compose(pose, offset):
    """(x, y, yaw) of `offset` (base_link) in map, given the car's map pose."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * offset[0] - s * offset[1],
            y + s * offset[0] + c * offset[1],
            yaw + offset[2])


def beams_from_scan(scan, laser_map):
    """Map-frame beam endpoints. SAME POLICY as the node: no return -> range_max."""
    r = np.asarray(scan.ranges, dtype=float)
    if r.size == 0:
        return fd.ScanBeams(np.zeros((0, 2)), laser_map[:2])
    bearings = scan.angle_min + np.arange(r.size, dtype=float) * scan.angle_increment
    no_return = ~np.isfinite(r) | (r >= scan.range_max)
    r = np.where(no_return, scan.range_max, r)
    keep = no_return | (r >= scan.range_min)
    r, bearings = r[keep], bearings[keep]
    th = laser_map[2] + bearings
    return fd.ScanBeams(
        np.column_stack([laser_map[0] + r * np.cos(th),
                         laser_map[1] + r * np.sin(th)]), laser_map[:2])


def replay(bag, out_dir, params, rate_hz, window_m):
    v2.TOPICS = TOPICS
    store = v2.typestore(bag)
    data = v2.read_bag(bag, store)
    resolve = v2.tf_to_base(data['/tf_static'])

    pose_topic = ('/ekf_global/odometry/filtered'
                  if data['/ekf_global/odometry/filtered'] else '/odometry/filtered')
    poses = [(v2.stamp(m), v2.planar(m)) for _t, m in data[pose_topic]]
    grids = data['/slam/map']
    scans = data['/scan']
    if not grids:
        raise SystemExit('this bag has no /slam/map: nothing for this detector to read')
    if not poses:
        raise SystemExit(f'this bag has no {pose_topic}: no map-frame pose')

    p = freespace_params(params, window_m=window_m)
    tracker = sd.DoorTracker(p.door_params())
    period = 1.0 / float(rate_hz)

    t0 = min(grids[0][0], scans[0][0] if scans else grids[0][0])
    t_end = max(grids[-1][0], scans[-1][0] if scans else grids[-1][0])
    gi, si = 0, 0
    rows, code_totals, ms = [], {}, []
    confirmed_ever = {}
    t = t0
    while t <= t_end:
        while gi + 1 < len(grids) and grids[gi + 1][0] <= t:
            gi += 1
        while si + 1 < len(scans) and scans[si + 1][0] <= t:
            si += 1
        if grids[gi][0] > t:
            t += period
            continue
        grid = grids[gi][1]
        scan = scans[si][1] if scans and scans[si][0] <= t else None
        pose = find_nearest_pose_by_stamp(poses, t)
        if pose is None:
            t += period
            continue

        c0 = time.perf_counter()
        info = grid.info
        window = fd.occupancy_window(
            grid.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y, pose[:2], p)
        skel = fd.skeleton(window, p)
        beams = None
        if scan is not None:
            lb = laser_in_base(resolve, scan.header.frame_id)
            if lb is not None:
                beams = beams_from_scan(scan, compose(pose, lb))
        cands = fd.candidate_doors(window, skel, beams, pose[:2], p)
        tracker.update(cands, (pose[0], pose[1]), t)
        cycle_ms = (time.perf_counter() - c0) * 1e3
        ms.append(cycle_ms)

        codes = {}
        for c in cands:
            key = c.code or 'ok'
            codes[key] = codes.get(key, 0) + 1
            code_totals[key] = code_totals.get(key, 0) + 1
        for d in tracker.confirmed():
            confirmed_ever[d.id] = d
        rows.append({
            't': round(t - t0, 2),
            'grid_age_s': round(t - grids[gi][0], 2),
            'free_cells': int(window.free.sum()),
            'skeleton_cells': int(skel.sum()),
            'candidates': len(cands),
            'confirmed': len(tracker.confirmed()),
            'ms': round(cycle_ms, 2),
            'codes': json.dumps(codes, sort_keys=True),
            'car_x': round(pose[0], 3),
            'car_y': round(pose[1], 3),
        })
        t += period

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / 'freespace_cycles.csv'
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ['t'])
        w.writeheader()
        w.writerows(rows)

    doors = [confirmed_ever[k] for k in sorted(confirmed_ever)]
    summary = {
        'bag': str(bag),
        'pose_topic': pose_topic,
        'grids': len(grids),
        'scans': len(scans),
        'cycles': len(rows),
        'rate_hz': rate_hz,
        'window_m': p.window_m,
        'ms_median': round(float(np.median(ms)), 2) if ms else None,
        'ms_p95': round(float(np.percentile(ms, 95)), 2) if ms else None,
        'ms_max': round(float(np.max(ms)), 2) if ms else None,
        'codes': dict(sorted(code_totals.items(), key=lambda kv: -kv[1])),
        'confirmed_doors': [
            {'id': d.id, 'centre': [round(d.centre[0], 3), round(d.centre[1], 3)],
             'width_m': round(d.width, 3),
             'yaw_deg': round(math.degrees(d.yaw()), 1),
             'hits': d.hits}
            for d in doors],
    }
    (out_dir / 'freespace_replay.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary, csv_path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag')
    ap.add_argument('--out', default='output/doors')
    ap.add_argument('--rate-hz', type=float, default=None)
    ap.add_argument('--window-m', type=float, default=None)
    args = ap.parse_args()
    params = v2.stack_params()
    rate = args.rate_hz if args.rate_hz is not None else float(params['freespace_doors_rate_hz'])
    summary, csv_path = replay(args.bag, args.out, params, rate, args.window_m)
    print(json.dumps(summary, indent=2))
    print(f'\nper-cycle rows: {csv_path}')


if __name__ == '__main__':
    main()
