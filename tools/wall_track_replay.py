#!/usr/bin/env python3
"""Replay mpc_controller.wall_tracker against an archived run's bag.

Usage (after `source install/setup.bash`):
    python3 tools/wall_track_replay.py ~/f1tenth_archive/complete/<run> [commit_at_m] [refit_period_s]

The archive carries neither /perception/front_distance nor /mpc/goal_drive, so
the commit instant is simulated (see COMMIT_AT below). This is the tool that
surfaced the two selection defects wall_tracker.py's docstring records (the
16 m scattered-return "wall" and the 10 m end wall behind an obstacle); rerun
it on the archive before changing a gate.

Commit is simulated: the first scan whose dead-ahead LiDAR range (median of
+-2 deg) drops to <= COMMIT_AT metres, standing in for the mission's
front_clearance <= 4.0 stop condition that hands over to the wall_turn (the
bag has neither /perception/front_distance nor /mpc/goal_drive). From then on:
selection at commit, refit at REBUILD_PERIOD, d_wall evaluated at every
odometry message. Prints the selection verdicts and a trace, writes a CSV.
"""

import csv
import math
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from mpc_controller.wall_tracker import (
    PROVENANCE_NAMES, WallTracker, scan_to_odom_points)

COMMIT_AT = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
# Refit cadence: the control tick (0.1 s). Pass 1.0 to see the per-rebuild sawtooth.
REBUILD_PERIOD = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
PARAMS = dict(normal_tol_rad=0.39, min_span_m=0.5, min_inliers=8, assoc_dist_m=0.3,
              dfront_slack_m=0.5, bumper_x_m=0.443, inlier_distance_m=0.03)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = ['/scan', '/odometry/filtered', '/tf_static', '/mission/status',
              '/behavior/tree_status', '/mpc/corridor_markers']
    reader.set_filter(rosbag2_py.StorageFilter(topics=[w for w in wanted if w in types]))
    out = {w: [] for w in wanted}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        out[topic].append((t_ns * 1e-9, deserialize_message(raw, get_message(types[topic]))))
    return out


def dead_ahead(msg, laser_pose):
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment + laser_pose[2]
    sel = (np.abs(angles) <= math.radians(2.0)) & np.isfinite(ranges) & (ranges > msg.range_min)
    if not sel.any():
        return math.nan
    return float(np.median(ranges[sel])) + laser_pose[0]   # from base_link origin


def main():
    run = Path(sys.argv[1])
    data = read(run / 'bag')
    laser_pose = None
    for _t, tf in data['/tf_static']:
        for tr in tf.transforms:
            if tr.header.frame_id == 'base_link' and tr.child_frame_id == 'laser':
                laser_pose = (tr.transform.translation.x, tr.transform.translation.y,
                              yaw_of(tr.transform.rotation))
    print(f'run {run.name}: laser pose {laser_pose}, {len(data["/scan"])} scans, '
          f'{len(data["/odometry/filtered"])} odom')
    for t, m in data['/mission/status']:
        print(f'  mission/status t={t - data["/scan"][0][0]:7.2f} state={m.state}')

    odom = [(t, m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
            for t, m in data['/odometry/filtered']]
    odom_t = np.array([o[0] for o in odom])

    def pose_at(t):
        i = int(np.clip(np.searchsorted(odom_t, t) - 1, 0, len(odom) - 1))
        return odom[i][1:]

    tracker = WallTracker(**PARAMS)
    t0 = data['/scan'][0][0]
    committed = False
    psi_commit = None
    last_rebuild = None
    rows = []
    scans = data['/scan']
    for k, (t, scan) in enumerate(scans):
        pose = pose_at(t)
        ahead = dead_ahead(scan, laser_pose)
        points = scan_to_odom_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                     scan.range_min, scan.range_max, laser_pose, pose)
        if not committed:
            if math.isfinite(ahead) and ahead <= COMMIT_AT and k > 5:
                committed = True
                psi_commit = pose[2]
                tracker.commit(psi_commit, ahead)
                print(f'\nCOMMIT at t={t - t0:.2f}s dead_ahead={ahead:.2f} psi={psi_commit:+.3f} '
                      f'pose=({pose[0]:+.2f},{pose[1]:+.2f}) window={tracker.window_m}')
                sel = tracker.select(points, pose, t)
                for c in sel.candidates:
                    verdict = ('ACCEPTED' if c is sel.accepted else
                               'passed-not-best' if c.reason == 'accepted' else c.reason)
                    print(f'  {verdict:16s} d={c.distance_m:5.2f} '
                          f'angle_err={math.degrees(c.angle_err_rad):5.1f} deg '
                          f'inliers={c.inlier_count:4d} span={c.span_m:5.2f} '
                          f'rms={c.rms_m * 1e3:4.1f}mm ahead={c.ahead}')
                if sel.accepted is None:
                    print('  NO CANDIDATE ACCEPTED')
                last_rebuild = t
            continue
        step = None
        if tracker.has_wall and t - last_rebuild >= REBUILD_PERIOD:
            step = tracker.update(points, pose, t)
            last_rebuild = t
        rows.append((t - t0, math.degrees(pose[2] - psi_commit), tracker.d_wall(pose), ahead,
                     PROVENANCE_NAMES[tracker.provenance],
                     step.reason if step else '', step.inlier_count if step else '',
                     f'{step.span_m:.2f}' if step and math.isfinite(step.span_m) else ''))

    out = run.name + '.wall_track.csv'
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'rot_deg', 'd_wall', 'dead_ahead', 'provenance', 'refit', 'inliers',
                    'span'])
        w.writerows(rows)
    print(f'\ntrace ({len(rows)} rows, every ~0.5 s; refit rows marked) -> {out}')
    print(f'{"t":>6} {"rot":>6} {"d_wall":>7} {"ahead":>6} {"prov":>13} refit')
    last_print = -1.0
    for r in rows:
        if r[5] or r[0] - last_print >= 0.5:
            print(f'{r[0]:6.2f} {r[1]:+6.1f} {r[2]:7.3f} {r[3]:6.2f} {r[4]:>13} '
                  f'{r[5]} {r[6]} {r[7]}')
            last_print = r[0]
    if rows:
        d = np.array([r[2] for r in rows])
        print(f'\nd_wall: first {d[0]:.3f} last {d[-1]:.3f} max step between odom msgs '
              f'{np.abs(np.diff(d)).max():.4f} m; rotation {rows[-1][1]:+.1f} deg')


if __name__ == '__main__':
    main()
