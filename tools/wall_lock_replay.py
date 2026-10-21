#!/usr/bin/env python3
"""What the mission wall stop's FRONT-WALL LOCK would have done on a recording.

Usage (after `source install/setup.bash`):
    python3 tools/wall_lock_replay.py [--target M] [--debounce N] [--front-x M] BAG

Feeds the recorded /perception/virtual_wall, /perception/virtual_wall/debug,
/perception/walls and /odometry/filtered through f1tenth_behavior's
FrontWallLock (mission/wall_lock.py) with the stack_params.yaml settings, one
lock step per debug payload, by bag time. Prints every phase change and event
(acquired, re-acquired, refused) and where a front_clearance stop at --target
would fire on the raw wall topic and on the locked value (strict arm only,
--debounce consecutive steps). The rest-and-tolerance arm is not modelled.
Each line also says what mpc_corr's deceleration ramp aims at on that step
(mpc_controller/ramp_wall_source.py, fed the id the BT would publish on
/mpc/stop_wall_id), and the summary counts the steps where that is not the
raw wall value of the same publish.

THE TWO STOP COLUMNS PAIR TWO TOPICS BY BAG TIME, and the recorder does not
keep them paired: corridor10m_20261019T131815 holds 716
/perception/virtual_wall messages against 730 payloads and writes them in
bursts, so "raw fires at" can be a later, smaller value than the live stop saw
(there: raw 78.52 s / 1.734 m, locked 77.14 s / 1.98 m; the Float32s carrying
1.98 and 1.89 are not in the bag). The ramp column reads the payload alone and
has no such skew.

BAG is a bag directory or an archive run folder (the one holding bag/); the
storage plugin is detected. Read-only, no rclpy node, no graph.

Needs a recording from a stack that writes `front_id` in the debug payload
(campaign-final since 2026-10-16, block 3). On an older one the lock never
engages and the two stops agree by construction; one from before 2026-10-15
does not even decode (/perception/walls grew fields then), and the tool says
so and stops.

Measured 2026-10-21 (output/front_wall_lock/design.md): the M02 mission bf7938
(COMPLETE) and corridor10m_20261019T131815 decide as before; on bf7938 the
first version of the lock (any FRONT locked) refused the real wall.
"""

import argparse
import math
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from f1tenth_behavior.behaviours.check_stop_condition import (
    front_from_debug, walls_from_message)
from f1tenth_behavior.mission import wall_lock
from f1tenth_description.vehicle_frames import base_front_x
from f1tenth_params.param_defaults import get_value
from mpc_controller import ramp_wall_source

TOPICS = ('/perception/virtual_wall', '/perception/virtual_wall/debug',
          '/perception/walls', '/odometry/filtered')


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def replay(bag, target, debounce, front_x):
    if os.path.isdir(os.path.join(bag, 'bag')):
        bag = os.path.join(bag, 'bag')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in TOPICS if t not in types]
    if missing:
        raise SystemExit(f'{bag}: not recorded: {", ".join(missing)}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))

    max_age = float(get_value('wall_distance_max_age_s'))
    lock = wall_lock.FrontWallLock(wall_lock.LockParams(
        gate_m=float(get_value('wall_follow_gate_m')),
        gate_rad=math.radians(float(get_value('wall_follow_gate_deg'))),
        coast_max_sec=float(get_value('lidar_front_wall_wall_tracker_front_max_coast_sec')),
        coast_max_m=float(get_value('lidar_front_wall_wall_tracker_front_max_coast_m'))))
    clear = float(get_value('wall_clear_distance_m'))
    ramp_counts = {}
    ramp_not_raw = 0
    raw = walls = walls_t = pose = prev_xy = t0 = phase = None
    odometer = 0.0
    streak = {'raw': 0, 'lock': 0}
    fired = {'raw': None, 'lock': None}
    counts = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns * 1e-9
        t0 = t if t0 is None else t0
        try:
            msg = deserialize_message(data, get_message(types[topic]))
        except Exception as exc:    # rclpy's RMWError, by name only
            raise SystemExit(f'{bag}: {topic} does not decode with the installed '
                             f'{types[topic]} (an older definition?): {exc}')
        if topic == '/perception/virtual_wall':
            raw = float(msg.data)
        elif topic == '/perception/walls':
            walls, walls_t = walls_from_message(msg), t
        elif topic == '/odometry/filtered':
            p = msg.pose.pose
            if prev_xy is not None:
                odometer += math.hypot(p.position.x - prev_xy[0], p.position.y - prev_xy[1])
            prev_xy = (p.position.x, p.position.y)
            pose = (p.position.x, p.position.y, _yaw(p.orientation))
        else:
            front = front_from_debug(msg.data)
            tracks = walls if walls_t is not None and t - walls_t <= max_age else None
            bumper = None if pose is None else (pose[0] + front_x * math.cos(pose[2]),
                                                pose[1] + front_x * math.sin(pose[2]))
            step = lock.update(t, raw, front, tracks, bumper, odometer)
            counts[step.phase] = counts.get(step.phase, 0) + 1
            # The ramp's raw value: this payload's own distance_m, through
            # MPC_corr._fresh_front_distance's filter (no -1.0, no CLEAR). Not
            # the last recorded Float32: the recorder writes the two topics in
            # different bursts and misses some (corridor10m_20261019T131815:
            # 369 of 730 payloads follow a Float32 that is not theirs), while
            # live the Float32 carries this same publish's value.
            d = None if front is None else front.distance
            raw_d = d if d is not None and math.isfinite(d) and 0.0 < d < clear else None
            ramp = ramp_wall_source.select_ramp_distance(
                raw_d, wall_lock.ramp_wall_id(step), front, clear)
            ramp_counts[ramp.source] = ramp_counts.get(ramp.source, 0) + 1
            ramp_not_raw += ramp.distance != raw_d
            if step.event or step.phase != phase:
                print(f'{t - t0:8.2f} s  {step.phase:10s} track={step.track_id} '
                      f'event={step.event} raw={raw} locked={step.distance} '
                      f'ramp={ramp.distance} ({ramp.source})  {step.detail}')
                phase = step.phase
            for name, value in (('raw', raw), ('lock', step.distance)):
                hit = value is not None and 0.0 < value < target
                streak[name] = streak[name] + 1 if hit else 0
                if fired[name] is None and streak[name] >= debounce:
                    fired[name] = (round(t - t0, 2), round(value, 3))
    print(f'\nphases (lock steps): {counts}')
    print(f'ramp sources (lock steps): {ramp_counts}; the ramp read other than the '
          f'raw wall value of the same publish on {ramp_not_raw} step(s)')
    print(f'stop < {target} m, {debounce} step(s): raw fires at {fired["raw"]}, '
          f'locked fires at {fired["lock"]}  (bag s, distance m)')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('bag')
    parser.add_argument('--target', type=float, default=2.0, help='stop distance [m]')
    parser.add_argument('--debounce', type=int, default=1, help='consecutive steps')
    parser.add_argument('--front-x', type=float, default=None,
                        help='base_link to bumper [m]; default: the URDF base_front')
    args = parser.parse_args()
    front_x = base_front_x() if args.front_x is None else args.front_x
    replay(args.bag, args.target, args.debounce, front_x)


if __name__ == '__main__':
    main()
