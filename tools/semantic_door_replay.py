#!/usr/bin/env python3
"""
semantic_door_replay.py: the semantic-layer door detector, replayed on a bag.

Usage (no ROS environment needed; numpy, pyyaml, rosbags):
    python3 tools/semantic_door_replay.py --out output/doors BAG [BAG ...]

WHAT IS REPLAYED. f1tenth_costmap/semantic_doors.py as semantic_layer_node
hosts it: at each /scan (the node's LiDAR cycle), the newest
/perception/walls message's CONFIRMED tracks are moved into the map frame
through the nearest pose by stamp, the scan is turned into a ScanView in
base_link, and candidate_doors + DoorTracker run with stack_params.yaml's
semantic_doors_* values. Never rclpy; the bag is read with
tools/obstacles_v2_replay.py's own sqlite3 + rosbags decoding.

TWO HONEST LIMITS OF THIS REPLAY, both reported in the summary:

  THE OBJECT VETO HAS NOTHING TO VETO WITH. The in-the-gap object test takes
  the semantic layer's own confirmed tracks, which exist only once that
  layer has run. Rebuilding them here would be a replay of a different node
  (tools/semantic_merge_replay.py is that one). So `objects` is empty and
  every door reported here passed the other three gates only. On a bag with
  someone standing in a doorway, this replay says door and the car would
  not.

  "MAP" IS WHATEVER POSE THE BAG CARRIES. With /ekf_global/odometry/filtered
  recorded, the frame is the map and the coordinates are the car's. With
  only /odometry/filtered -- which is what the October corridor bags have --
  it is the odom frame, so the poses below are odom poses. Nothing in the
  geometry changes (a door is found in the same place either way); only what
  the numbers are relative to. The summary names the topic used.

OUTPUTS in --out, per bag:
  <name>.candidates.csv  every gap evaluated on every cycle, with its code
  <name>.doors.csv       one row per confirmed door per cycle
  <name>.summary.json    the counts, the codes, and the final door list
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
sys.path.insert(0, str(REPO / 'tools'))
for sub in ('src/f1tenth_costmap', 'src/f1tenth_params'):
    sys.path.insert(0, str(REPO / sub))

import obstacles_v2_replay as v2  # noqa: E402
from f1tenth_costmap import semantic_doors as sd  # noqa: E402
from f1tenth_costmap.semantic_layer import find_nearest_pose_by_stamp  # noqa: E402
from f1tenth_costmap.wall_raster import transform_segment  # noqa: E402

#: Only what this replay reads. Other topics of an old bag may not decode
#: against today's .msg files, and reading them would fail the whole bag.
TOPICS = ('/scan', '/perception/walls', '/tf_static',
          '/ekf_global/odometry/filtered', '/odometry/filtered')

#: MultiWallTrack.STATE_CONFIRMED / the semantic layer's own filter.
STATE_CONFIRMED = 1


def door_params(params):
    """A DoorParams from stack_params.yaml -- the car's own values."""
    def p(key):
        return params['semantic_doors_' + key]
    return sd.DoorParams(
        min_wall_level=sd.LEVEL_BY_NAME[str(p('min_wall_level'))],
        width_min_m=float(p('width_min_m')),
        width_max_m=float(p('width_max_m')),
        parallel_angle_deg=float(p('parallel_angle_deg')),
        parallel_offset_m=float(p('parallel_offset_m')),
        perp_tol_deg=float(p('perp_tol_deg')),
        foot_inset_m=float(p('foot_inset_m')),
        end_fov_margin_rad=math.radians(float(p('end_fov_margin_deg'))),
        end_range_margin_m=float(p('end_range_margin_m')),
        end_occlusion_margin_m=float(p('end_occlusion_margin_m')),
        end_probe_beams=int(p('end_probe_beams')),
        min_beams=int(p('min_beams')),
        through_frac=float(p('through_frac')),
        behind_margin_m=float(p('behind_margin_m')),
        gap_clear_m=float(p('gap_clear_m')),
        jamb_exclude_m=float(p('jamb_exclude_m')),
        assoc_max_m=float(p('assoc_max_m')),
        confirm_hits=int(p('confirm_hits')),
        smooth_alpha=float(p('smooth_alpha')),
        candidate_forget_s=float(p('candidate_forget_s')),
        forget_unseen_s=float(p('forget_unseen_s')),
        max_walls=int(p('max_walls')))


def laser_xy_yaw(resolve, frame_id):
    """(x, y, yaw) of the scan frame in base_link, from /tf_static, or None."""
    m = resolve(frame_id)
    if m is None:
        return None
    return float(m[0, 3]), float(m[1, 3]), math.atan2(float(m[1, 0]), float(m[0, 0]))


def to_base_link(xy, pose):
    """A map point to base_link, given the car's (x, y, yaw) in map."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = xy[0] - pose[0], xy[1] - pose[1]
    return (c * dx + s * dy, -s * dx + c * dy)


def scan_view(scan, laser, p):
    """A sd.ScanView of one LaserScan, base_link."""
    r = np.asarray(scan.ranges, dtype=float)
    lx, ly, lyaw = laser
    bearings = lyaw + scan.angle_min + np.arange(r.size) * scan.angle_increment
    valid = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
    return sd.ScanView(bearings, np.where(valid, r, np.inf), (lx, ly), scan.range_max)


def replay(bag, out_dir, params):
    name = Path(bag).name
    v2.TOPICS = TOPICS
    store = v2.typestore()
    data = v2.read_bag(bag, store)
    resolve = v2.tf_to_base(data['/tf_static'])

    pose_topic = ('/ekf_global/odometry/filtered'
                  if data['/ekf_global/odometry/filtered'] else '/odometry/filtered')
    poses = [(v2.stamp(m), v2.planar(m)) for _t, m in data[pose_topic]]
    frame = 'map' if pose_topic.startswith('/ekf_global') else 'odom'

    p = door_params(params)
    tracker = sd.DoorTracker(p)
    walls_msgs = data['/perception/walls']
    wi = 0
    latest_walls = None
    laser = None

    cand_rows, door_rows = [], []
    codes = {}
    cycles = 0
    cycle_ms = []
    for _t, scan in data['/scan']:
        stamp = v2.stamp(scan)
        while wi < len(walls_msgs) and v2.stamp(walls_msgs[wi][1]) <= stamp:
            latest_walls = walls_msgs[wi][1]
            wi += 1
        if latest_walls is None:
            continue
        pose = find_nearest_pose_by_stamp(poses, stamp)
        if pose is None:
            continue
        if laser is None:
            laser = laser_xy_yaw(resolve, scan.header.frame_id)
            if laser is None:
                continue
        cycles += 1
        walls = []
        for t in latest_walls.tracks:
            p0, p1 = transform_segment((t.p0_bl.x, t.p0_bl.y),
                                       (t.p1_bl.x, t.p1_bl.y), pose)
            walls.append(sd.Wall(t.id, p0, p1,
                                 level=sd.wall_level(t.state, t.source),
                                 role=t.role or t.label))
        t0 = time.perf_counter()
        view = scan_view(scan, laser, p)
        # objects: EMPTY -- see the module docstring.
        cands = sd.candidate_doors(walls, view, [], p,
                                   to_base_link=lambda xy: to_base_link(xy, pose))
        tracker.update(cands, (pose[0], pose[1]), stamp)
        cycle_ms.append((time.perf_counter() - t0) * 1e3)
        for c in cands:
            code = c.code or 'ok'
            codes[code] = codes.get(code, 0) + 1
            cand_rows.append({
                'stamp': f'{stamp:.3f}', 'case': c.case,
                'wall0': c.wall_id0, 'wall1': c.wall_id1,
                'width_m': f'{c.width:.3f}',
                'centre_x': f'{c.centre[0]:.3f}', 'centre_y': f'{c.centre[1]:.3f}',
                'code': code, 'group': sd.code_group(c.code),
                'n_beams': c.n_beams, 'through_frac': f'{c.through_frac:.3f}'})
        for d in tracker.confirmed():
            j0, j1 = d.jambs()
            door_rows.append({
                'stamp': f'{stamp:.3f}', 'door_id': d.id, 'frame': d.frame_id(),
                'case': d.case, 'hits': d.hits,
                'centre_x': f'{d.centre[0]:.3f}', 'centre_y': f'{d.centre[1]:.3f}',
                'yaw_deg': f'{math.degrees(d.yaw()):.1f}',
                'width_m': f'{d.width:.3f}',
                'jamb0_x': f'{j0[0]:.3f}', 'jamb0_y': f'{j0[1]:.3f}',
                'jamb1_x': f'{j1[0]:.3f}', 'jamb1_y': f'{j1[1]:.3f}'})

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(out_dir / f'{name}.candidates.csv', cand_rows)
    _write_csv(out_dir / f'{name}.doors.csv', door_rows)

    summary = {
        'bag': name,
        'frame': frame,
        'pose_topic': pose_topic,
        'object_veto': 'not exercised: no semantic-layer objects in this replay',
        'scans': len(data['/scan']),
        'wall_messages': len(walls_msgs),
        'cycles_run': cycles,
        'candidates': len(cand_rows),
        # On this machine, not the Orin: a cost ORDER, not a budget.
        'cycle_ms': {
            'median': round(float(np.median(cycle_ms)), 3) if cycle_ms else None,
            'p95': round(float(np.percentile(cycle_ms, 95)), 3) if cycle_ms else None,
            'max': round(float(np.max(cycle_ms)), 3) if cycle_ms else None,
        },
        'codes': dict(sorted(codes.items(), key=lambda kv: -kv[1])),
        'doors_confirmed': len(tracker.confirmed()),
        'doors': [{
            'id': d.id, 'frame': d.frame_id(), 'case': d.case, 'hits': d.hits,
            'centre': [round(d.centre[0], 3), round(d.centre[1], 3)],
            'yaw_deg': round(math.degrees(d.yaw()), 1),
            'width_m': round(d.width, 3),
            'walls': [d.wall_id0, d.wall_id1],
        } for d in tracker.confirmed()],
        'tracks_total': len(tracker.tracks),
        'params': {
            'min_wall_level': sd.LEVEL_NAMES[p.min_wall_level],
            'width_m': [p.width_min_m, p.width_max_m],
            'confirm_hits': p.confirm_hits,
        },
    }
    (out_dir / f'{name}.summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def _write_csv(path, rows):
    if not rows:
        path.write_text('')
        return
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--out', default='output/doors')
    args = ap.parse_args()
    params = v2.stack_params()
    for bag in args.bags:
        s = replay(bag, args.out, params)
        print(json.dumps(s, indent=2))


if __name__ == '__main__':
    main()
