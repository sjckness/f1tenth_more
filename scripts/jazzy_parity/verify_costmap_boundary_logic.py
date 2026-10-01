#!/usr/bin/env python3
"""Phase 2 addendum (item A): test costmap_boundary_node's EXTRACTION LOGIC
in isolation from all ROS timing -- no node, no timer, no spin, no replay.

The Phase 2 report's original costmap_boundary parity pass replayed the
node live and found large Jazzy-vs-Humble outliers (normal-angle RMS
6.3-6.5 deg, up to 89 deg) alongside a smaller but nonzero Jazzy-vs-Jazzy
noise floor (up to 7 deg between two IDENTICAL replay runs), and
attributed this to the node's own 20Hz-periodic/5s-map-update timer
racing against message arrival -- a plausible but, as originally reported,
UNPROVEN explanation. This script removes timing from the question
entirely: reconstruct, from the ORIGINAL Humble bag's own recv (bag-write)
timestamps, which /slam/map and /ekf_global/odometry/filtered message the
node most plausibly had cached at each recorded /costmap/boundaries tick
(the latest of each with recv time <= the output's own recv time -- the
real causal ordering, not the output's header.stamp, which the node sets
to its own publish-time clock read per costmap_boundary_node.py's
_extraction_tick(), not derived from the inputs), then call
extract_boundary_constraints()/front_clearance_from_extraction() DIRECTLY
-- pure functions, no rclpy -- and diff against what Humble actually
published for that exact tick. If the timer-race hypothesis is right,
this should be near-exact; if not, there's a real logic/numerics
difference to chase before Phase 3.
"""
import bisect
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from bag_read import read_topic
from compare_runs import wrap

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                        / 'src/f1tenth_costmap/f1tenth_costmap'))
from costmap_boundary import (  # noqa: E402
    extract_boundary_constraints, front_clearance_from_extraction, yaw_from_quaternion)

BAG = '/home/andre/bags/humble_reference/humble_obstacle_run/bag'

# Production defaults -- costmap_boundary_node.py's own declare_parameter
# defaults, identical to costmap.launch.py's (checked in the original
# Phase 2 pass, Step 3).
FRONT_FACING_MAX_RAD = math.radians(35.0)
SIDE_WINDOW_MIN_RAD = math.radians(45.0)
SIDE_WINDOW_MAX_RAD = math.radians(135.0)
OCCUPIED_THRESHOLD = 65
MAX_RANGE_M = 5.0
_DIRECTIONS = ('front', 'left', 'right')


def load_with_recv(topic):
    return sorted(read_topic(BAG, topic), key=lambda r: r[0])


def latest_before(series, t, idx_offset=0):
    """series sorted by recv time (ns). Return the entry at or before `t`,
    `idx_offset` further back (0 = latest, 1 = second-latest, for the
    off-by-one cache check), or None."""
    times = [r[0] for r in series]
    i = bisect.bisect_right(times, t) - 1 - idx_offset
    if i < 0:
        return None
    return series[i][1]


def reconstruct(map_msg, pose_msg):
    pos = pose_msg.pose.pose.position
    yaw = yaw_from_quaternion(pose_msg.pose.pose.orientation)
    extraction = extract_boundary_constraints(
        map_msg.data, map_msg.info.width, map_msg.info.height, map_msg.info.resolution,
        map_msg.info.origin.position.x, map_msg.info.origin.position.y,
        pos.x, pos.y, yaw,
        FRONT_FACING_MAX_RAD, SIDE_WINDOW_MIN_RAD, SIDE_WINDOW_MAX_RAD,
        OCCUPIED_THRESHOLD, MAX_RANGE_M)
    constraints = []
    for d in _DIRECTIONS:
        if extraction[d] is not None:
            nx, ny, off = extraction[d]
            constraints.append((nx, ny, off))
    clearance = front_clearance_from_extraction(extraction, MAX_RANGE_M)
    return constraints, clearance


def diff_constraints(recorded, recon):
    if len(recorded) != len(recon):
        return None  # count mismatch -- reported separately
    angs, offs = [], []
    for (rnx, rny, roff), (cnx, cny, coff) in zip(recorded, recon):
        dot = max(-1.0, min(1.0, rnx * cnx + rny * cny))
        angs.append(math.degrees(math.acos(dot)))
        offs.append(abs(roff - coff))
    return angs, offs


def main():
    maps = load_with_recv('/slam/map')
    poses = load_with_recv('/ekf_global/odometry/filtered')
    boundaries = load_with_recv('/costmap/boundaries')
    clearances = load_with_recv('/costmap/front_clearance')
    clearance_by_t = {t: msg.data for t, msg in clearances}
    clearance_times = sorted(clearance_by_t)

    n_total = len(boundaries)
    n_no_map_or_pose = 0
    n_count_mismatch = 0
    all_angs, all_offs = [], []
    mismatched_ticks = []  # count-mismatch ticks, for the off-by-one check
    large_err_ticks = []   # same-count but angle err > 5deg ticks, same check
    clearance_errs = []

    for t_out, msg in boundaries:
        map_msg = latest_before(maps, t_out)
        pose_msg = latest_before(poses, t_out)
        if map_msg is None or pose_msg is None:
            n_no_map_or_pose += 1
            continue

        recorded = [(c.normal[0], c.normal[1], c.offset) for c in msg.constraints]
        recon, clearance = reconstruct(map_msg, pose_msg)

        d = diff_constraints(recorded, recon)
        if d is None:
            n_count_mismatch += 1
            mismatched_ticks.append((t_out, recorded, pose_msg, map_msg))
            continue
        angs, offs = d
        all_angs.extend(angs)
        all_offs.extend(offs)
        if max(angs) > 5.0:
            large_err_ticks.append((t_out, recorded, pose_msg, map_msg, max(angs)))

        # nearest front_clearance sample to this tick's own output time
        ci = bisect.bisect_left(clearance_times, t_out)
        cand = [i for i in (ci - 1, ci) if 0 <= i < len(clearance_times)]
        if cand:
            best = min(cand, key=lambda i: abs(clearance_times[i] - t_out))
            if abs(clearance_times[best] - t_out) < int(0.05 * 1e9):
                clearance_errs.append(abs(clearance_by_t[clearance_times[best]] - clearance))

    print(f'=== costmap_boundary logic-only verification (latest-cached inputs) ===')
    print(f'total /costmap/boundaries messages: {n_total}')
    print(f'  no map/pose cached yet (skipped):  {n_no_map_or_pose}')
    print(f'  constraint-count mismatch:          {n_count_mismatch}')
    print(f'  compared directly:                  {len(all_angs) and "see below" or 0}')
    print()

    if all_angs:
        a = np.array(all_angs)
        o = np.array(all_offs)
        print('normal angle error (deg): '
              f'median={np.median(a):.4f} p95={np.percentile(a,95):.4f} max={np.max(a):.4f} '
              f'frac>1deg={np.mean(a>1):.4f} frac>5deg={np.mean(a>5):.4f}')
        print('offset error (m):         '
              f'median={np.median(o):.5f} p95={np.percentile(o,95):.5f} max={np.max(o):.5f} '
              f'frac>0.01m={np.mean(o>0.01):.4f} frac>0.1m={np.mean(o>0.1):.4f}')
    if clearance_errs:
        c = np.array(clearance_errs)
        print('front_clearance error (m):'
              f' median={np.median(c):.5f} p95={np.percentile(c,95):.5f} max={np.max(c):.5f} '
              f'frac>0.01m={np.mean(c>0.01):.4f} frac>0.1m={np.mean(c>0.1):.4f}  n={len(c)}')
    print()

    # Off-by-one cache check on the count-mismatch ticks: do they resolve
    # with the SECOND-latest map or pose instead?
    print(f'=== off-by-one cache check on {len(mismatched_ticks)} count-mismatched ticks ===')
    resolved_prev_map = 0
    resolved_prev_pose = 0
    still_mismatched = 0
    for t_out, recorded, pose_msg, map_msg in mismatched_ticks:
        prev_map = latest_before(maps, t_out, idx_offset=1)
        prev_pose = latest_before(poses, t_out, idx_offset=1)
        resolved = False
        if prev_map is not None:
            recon, _ = reconstruct(prev_map, pose_msg)
            if len(recon) == len(recorded):
                resolved_prev_map += 1
                resolved = True
        if not resolved and prev_pose is not None:
            recon, _ = reconstruct(map_msg, prev_pose)
            if len(recon) == len(recorded):
                resolved_prev_pose += 1
                resolved = True
        if not resolved:
            still_mismatched += 1
    print(f'  resolved with previous /slam/map:      {resolved_prev_map}')
    print(f'  resolved with previous pose:            {resolved_prev_pose}')
    print(f'  still mismatched (neither explains it): {still_mismatched}')

    # The check that actually matters for the >5deg outliers: same
    # constraint COUNT, but the nearest-cell choice itself differs enough
    # to swing the normal by >5deg. Does the PREVIOUS map or pose resolve
    # it (angle error drops under 1deg)?
    print(f'\n=== off-by-one cache check on {len(large_err_ticks)} large-angle-error (>5deg) ticks ===')
    resolved_prev_map2 = 0
    resolved_prev_pose2 = 0
    still_large = 0
    for t_out, recorded, pose_msg, map_msg, orig_err in large_err_ticks:
        prev_map = latest_before(maps, t_out, idx_offset=1)
        prev_pose = latest_before(poses, t_out, idx_offset=1)
        resolved = False
        if prev_map is not None:
            recon, _ = reconstruct(prev_map, pose_msg)
            d = diff_constraints(recorded, recon)
            if d is not None and max(d[0]) < 1.0:
                resolved_prev_map2 += 1
                resolved = True
        if not resolved and prev_pose is not None:
            recon, _ = reconstruct(map_msg, prev_pose)
            d = diff_constraints(recorded, recon)
            if d is not None and max(d[0]) < 1.0:
                resolved_prev_pose2 += 1
                resolved = True
        if not resolved:
            still_large += 1
    print(f'  resolved (<1deg) with previous /slam/map: {resolved_prev_map2}')
    print(f'  resolved (<1deg) with previous pose:       {resolved_prev_pose2}')
    print(f'  still large (neither explains it):          {still_large}')


if __name__ == '__main__':
    main()
