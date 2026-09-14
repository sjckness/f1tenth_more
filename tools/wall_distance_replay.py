#!/usr/bin/env python3
"""Replay wall_distance_node's logic against an archived run's bag.

Usage (after `source install/setup.bash`):
    python3 tools/wall_distance_replay.py ~/f1tenth_archive/complete/<run> [commit_at_m]

    # the highest-value bag in the archive, 55.9 s with 2228 scans:
    python3 tools/wall_distance_replay.py \
        ~/f1tenth_archive/complete/2026-09-11T13-49-54_mission-wall_turn

Companion to tools/wall_track_replay.py, which replays mpc_controller's
wall_tracker. This one replays f1tenth_perception's glass_detect +
wall_distance: the phase machine, the tracked wall, the coast caps and the
correction.


WHAT THIS CAN AND CANNOT VALIDATE -- READ THIS BEFORE BELIEVING THE OUTPUT
=========================================================================
NO BAG IN THE ARCHIVE CONTAINS /mpc/wall_track. Every one of the 14 wall_turn
and drive_turn_180 runs predates the wall tracker, and /perception/front_distance
and /mpc/goal_drive are absent from all of them too (the same gap
tools/wall_track_replay.py already works around). So the phase machine's input
is SYNTHESISED here, by running plan_wall_turn_step's commit rule over the bag's
real /scan and /odometry/filtered at the real tick timing.

That buys a lot:
  * real scan geometry, real returns, real glass, real clutter
  * real odometry, with its real 17-20% distance bias baked in
  * the real 10 Hz control cadence and the real 40 Hz scan cadence
  * a real 90 degree turn, with the wall really sweeping out of incidence

And it does NOT buy the thing the phase machine is most exposed on:
  * REAL TURN-EXIT SILENCE. On the car, exit is observable only as silence, and
    that silence is overloaded five ways (no wall_turn, /mpc/hold, no odom,
    wall_track_enable false, dead node). Here the silence is manufactured by
    this script's own stop rule, so it is clean by construction -- exactly the
    thing real silence is not.
  * REAL DROPPED MESSAGES and real jitter on /mpc/wall_track.
  * REAL /mpc/hold TIMING. The bags DO carry /mpc/hold, and this script reads it
    (see --hold), which is the closest available proxy: a hold that lands
    mid-turn is the case that must not read as "turn finished".

So: a clean pass here means NOT YET FALSIFIED against real geometry. It does not
mean the silence logic is validated. Capturing a bag WITH /mpc/wall_track is
Stage 4 of docs/bringup_checklist.md and is the single highest-value thing the
first powered session produces.


WHAT IS SIMULATED, PRECISELY
============================
  * COMMIT. The first scan whose dead-ahead LiDAR range (median of +-2 deg)
    drops to <= commit_at metres, standing in for the mission's
    front_clearance <= 4.0 stop condition that hands over to the wall_turn.
    Same rule and same default as tools/wall_track_replay.py, so the two
    replays agree about when the turn started.
  * THE TURN'S END. When the car has accumulated turn_total of unwrapped yaw
    since commit -- which is what f1tenth_behavior's orientation_delta
    stop_condition actually does (condition_eval.py), not something invented
    here. From that tick on, /mpc/wall_track goes silent.
  * psi_commit. NaN until the commit tick, the pose yaw after it, matching
    MPC_corr._wall_track_tick.

Everything downstream of those three is the real code: glass_detect.detect(),
GlassTracker, WallDistanceTracker, PhaseMachine, psi_correction.
"""

import csv
import math
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from f1tenth_perception.glass_detect import DetectorConfig, GlassTracker, detect
from f1tenth_perception.wall_distance import (
    geometric_candidates,
    PHASE_CORRIDOR,
    PHASE_NAMES,
    PROVENANCE_NAMES,
    CorrectionConfig,
    PhaseMachine,
    WallDistanceTracker,
    gate_margin,
    psi_correction,
    wrap_to_pi,
)

COMMIT_AT = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
TURN_TOTAL_DEG = 90.0
TICK = 0.1          # MPC_corr's control tick, and wall_distance_node's publish rate.


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = ['/scan', '/odometry/filtered', '/tf_static', '/mission/status', '/mpc/hold']
    present = [w for w in wanted if w in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=present))
    out = {w: [] for w in wanted}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        out[topic].append((t_ns * 1e-9, deserialize_message(raw, get_message(types[topic]))))
    return out, present


def dead_ahead(msg, laser_pose):
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment + laser_pose[2]
    sel = (np.abs(angles) <= math.radians(2.0)) & np.isfinite(ranges) & (ranges > msg.range_min)
    if not sel.any():
        return math.nan
    return float(np.median(ranges[sel])) + laser_pose[0]


def main():
    run = Path(sys.argv[1])
    data, present = read(run / 'bag')
    print(f'run {run.name}')
    print(f'  topics present: {", ".join(present)}')
    missing = [t for t in ('/mpc/wall_track',) if t not in present]
    if missing:
        print("  /mpc/wall_track ABSENT (expected -- every archived bag predates it). "
              "The phase feed below is SYNTHESISED; see this file's docstring.")

    laser_pose = None
    for _t, tf in data['/tf_static']:
        for tr in tf.transforms:
            if tr.header.frame_id == 'base_link' and tr.child_frame_id == 'laser':
                laser_pose = (tr.transform.translation.x, tr.transform.translation.y,
                              yaw_of(tr.transform.rotation))
    if laser_pose is None:
        print('  no base_link <- laser on /tf_static; cannot unproject. Aborting.')
        return 1
    print(f'  laser pose {laser_pose}, {len(data["/scan"])} scans, '
          f'{len(data["/odometry/filtered"])} odom, {len(data["/mpc/hold"])} hold msgs')

    odom = [(t, m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
            for t, m in data['/odometry/filtered']]
    if not odom or not data['/scan']:
        print('  bag has no scans or no odometry. Aborting.')
        return 1
    odom_t = np.array([o[0] for o in odom])

    def pose_at(t):
        i = int(np.clip(np.searchsorted(odom_t, t) - 1, 0, len(odom) - 1))
        return odom[i][1:]

    holds = [(t, bool(m.data)) for t, m in data['/mpc/hold']]

    def held_at(t):
        state = False
        for ht, val in holds:
            if ht <= t:
                state = val
            else:
                break
        return state

    cfg = DetectorConfig()
    tracker = WallDistanceTracker(
        glass_tracker=GlassTracker(
            persistence_window=8, persistence_hits=3,
            match_endpoint_tol=0.20, match_angle_tol_deg=10.0,
            expect_return_tol_deg=60.0, fov_half_angle_rad=math.pi / 2,
            max_range_m=5.0, geometry_only_multiplier=2),
        max_coast_distance=0.5, max_coast_yaw=0.35)
    phase = PhaseMachine(silence_ticks=3)
    correction = CorrectionConfig(
        d_ref=0.60, convergence_length_m=3.0, max_psi_correction=0.20,
        max_psi_rate=0.5, deadband_floor=0.02, deadband_k=2.0,
        fade_start_age=0.3, fade_zero_age=1.0, stale_inflate_per_s=0.15)

    scans = data['/scan']
    t0 = scans[0][0]
    t_end = scans[-1][0]
    scan_t = np.array([s[0] for s in scans])
    turn_total = math.radians(TURN_TOTAL_DEG)

    committed = False
    psi_commit = None
    turn_accum = 0.0
    last_yaw = None
    turn_done = False
    dpsi = 0.0
    rows = []
    transitions = []
    next_scan = 0

    t = t0
    while t <= t_end:
        pose = pose_at(t)
        # Every odom message between the last tick and this one, so the
        # odometer integrates PATH LENGTH rather than tick-to-tick chords.
        tracker.odometer.update(pose)

        # ONE FIT PER TICK, FROM THE NEWEST SCAN IN THE WINDOW -- matching
        # wall_distance_node._scan_cb, which stores the newest scan and fits it
        # on the publish tick. Fitting every scan here would make the replay
        # four times more thorough than the node and hide the very thing that
        # forced the node's design: the full per-scan path costs 26.5 ms on
        # this Jetson, so at 40 Hz it is 107% of one core.
        newest = None
        while next_scan < len(scans) and scan_t[next_scan] <= t:
            newest = next_scan
            next_scan += 1
        if newest is not None:
            st, scan = scans[newest]
            spose = pose_at(st)
            intensities = scan.intensities if len(scan.intensities) else None
            scan_args = (scan.ranges, scan.angle_min, scan.angle_increment,
                         scan.range_min, scan.range_max, laser_pose, spose)
            cands, used = detect(*scan_args, cfg, intensities=intensities)
            # Both sources, as the node runs them -- see
            # wall_distance.geometric_candidates on why glass alone tracks
            # nothing on an opaque wall.
            cands = list(cands) + geometric_candidates(
                *scan_args, min_range_m=cfg.min_range_m, max_range_m=5.0,
                inlier_distance_m=0.03, min_inliers=40, min_span_m=1.0,
                min_distance_m=0.5, max_candidates=4)
            tracker.update(cands, spose, st, used)

        # ---- the synthesised /mpc/wall_track feed ----
        ahead = dead_ahead(scans[min(next_scan, len(scans) - 1)][1], laser_pose)
        if last_yaw is not None and committed:
            turn_accum += wrap_to_pi(pose[2] - last_yaw)
        last_yaw = pose[2]
        if not committed and math.isfinite(ahead) and ahead <= COMMIT_AT and t - t0 > 0.5:
            committed = True
            psi_commit = pose[2]
            print(f'\nSIMULATED COMMIT at t={t - t0:.2f}s dead_ahead={ahead:.2f} '
                  f'psi={psi_commit:+.3f}')
        if committed and abs(turn_accum) >= turn_total:
            if not turn_done:
                print(f'SIMULATED TURN END at t={t - t0:.2f}s '
                      f'(orientation_delta {math.degrees(turn_accum):+.1f} deg) '
                      f'-> /mpc/wall_track goes silent')
            turn_done = True
        held = held_at(t)
        if not committed or turn_done or held:
            # Silent: before the move, after the BT ends it, or on a hold --
            # /mpc/hold makes control_loop return BEFORE _wall_track_tick, so
            # a held car publishes nothing. THIS is the case that must not read
            # as "turn finished".
            feed = None
        else:
            feed = psi_commit if committed else math.nan

        obs = tracker.observation(pose, t)
        transition = phase.tick(feed, track_valid=obs.valid)
        if transition is not None:
            transitions.append((t - t0, transition))
            print(f'  PHASE t={t - t0:7.2f}  {transition}')

        applicable = obs.valid and phase.phase == PHASE_CORRIDOR
        dpsi = psi_correction(obs.d_wall, cfg=correction, fit_rms=obs.fit_rms,
                              age=obs.age, prev_psi=dpsi, dt=TICK, valid=applicable)
        rows.append((
            round(t - t0, 3), PHASE_NAMES[phase.phase], int(obs.track_id),
            int(obs.valid), round(obs.d_wall, 4), round(obs.heading_rel, 4),
            round(obs.fit_rms, 5), int(obs.inlier_count), round(obs.age, 3),
            PROVENANCE_NAMES[obs.provenance], round(obs.coast_distance, 3),
            round(obs.coast_yaw, 3), round(dpsi, 5),
            round(gate_margin(obs.age, correction.stale_inflate_per_s), 4),
            int(held), round(ahead, 2) if math.isfinite(ahead) else '',
            round(math.degrees(turn_accum), 1), int(obs.coast_cap_first_hit)))
        t += TICK

    out = run.name + '.d_wall.csv'
    header = ['t', 'phase', 'track_id', 'valid', 'd_wall', 'heading_rel', 'fit_rms',
              'inliers', 'age', 'provenance', 'coast_m', 'coast_rad', 'dpsi',
              'gate_margin', 'held', 'dead_ahead', 'turn_deg', 'cap_hit']
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print(f'\ntrace ({len(rows)} ticks) -> {out}')

    # ---- the summary that says whether anything happened at all ----
    phases = [r[1] for r in rows]
    ids = {r[2] for r in rows if r[3]}
    provs = [r[9] for r in rows]
    valid = [r for r in rows if r[3]]
    print('\nSUMMARY')
    print(f'  phases seen      : {sorted(set(phases))}')
    print(f'  transitions      : {len(transitions)}')
    for ts, tr in transitions:
        print(f'                     t={ts:7.2f}  {PHASE_NAMES[tr.old]} -> '
              f'{PHASE_NAMES[tr.new]}')
    print(f'  ticks in CORRIDOR: {phases.count("CORRIDOR")} of {len(phases)}')
    print(f'  track_ids used   : {sorted(ids) or "NONE -- no valid track all run"}')
    print(f'  provenance counts: '
          f'{ {p: provs.count(p) for p in sorted(set(provs))} }')
    if valid:
        d = np.array([r[4] for r in valid])
        print(f'  d_wall           : min {d.min():+.3f} max {d.max():+.3f} '
              f'mean {d.mean():+.3f} m; sides '
              f'{"BOTH" if (d > 0).any() and (d < 0).any() else "one"}')
        rms = np.array([r[6] for r in valid])
        print(f'  fit_rms          : min {rms.min() * 1e3:.1f} max {rms.max() * 1e3:.1f} mm')
    dp = np.array([r[12] for r in rows])
    print(f'  dpsi             : min {dp.min():+.4f} max {dp.max():+.4f} rad '
          f'(zero unless CORRIDOR)')
    print(f'  ticks held        : {sum(r[14] for r in rows)}')
    caps = sum(r[17] for r in rows)
    print(f'  coast caps fired  : {caps}'
          + ('  <-- each one breaks track identity, by design: the caps are set by'
             '\n                      the measured odometry bias, and a 90 deg turn cannot be'
             '\n                      coasted end to end. See stack_params.yaml'
             '\n                      wall_distance_max_coast_yaw.' if caps else ''))
    if caps and 'CORRIDOR' not in [r[1] for r in rows]:
        print('  CONSEQUENCE: a cap firing mid-turn clears the phase machine\'s'
              '\n               "valid track held throughout" condition, so this turn could'
              '\n               not have reached CORRIDOR even with a move after it. This is'
              '\n               the open design question in f1tenth_perception/README.md.')
    if 'CORRIDOR' not in phases:
        print('\n  NOTE no CORRIDOR ticks. Expected on a TERMINAL wall_turn mission: the '
              '\n  BT publishes /mpc/hold, which is silence WITHOUT a valid exit, so the '
              '\n  phase machine goes to UNKNOWN -- which is the designed behaviour, not a '
              '\n  failure. Replay a mission with a move after the turn to see CORRIDOR.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
