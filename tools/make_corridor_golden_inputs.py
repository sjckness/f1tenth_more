#!/usr/bin/env python3
"""Cut the recorded input window the corridor goldens are replayed from.

Usage (after `source install/setup.bash`):
    python3 tools/make_corridor_golden_inputs.py <run_dir> <out.json.gz> [t_start_s] [n_ticks]

<run_dir> is an archived run (its bag/ is opened READ_ONLY by rosbag2). The
output is the fixture test_corridor_golden.py replays mpc_corr's control loop
against: one input bundle per 10 Hz control tick, each message the LATEST one
the bag had received by that tick's clock, stored as its original CDR bytes so
the replay feeds back exactly what was recorded.

What is NOT in the bag and is therefore derived here, deterministically:
  * /perception/front_distance -- the archive does not record it. Stood in for
    by the scan's dead-ahead range (median over +-2 deg) re-referenced to the
    ZED's origin (base_link -> zed2_camera_link from the same /tf_static),
    because that topic is the camera's depth to the wall, measured from the
    lens. tools/wall_track_replay.py uses the same median but from base_link;
    on this car laser and camera sit at the same x, so the camera-referenced
    value is the LiDAR range itself.
  * a world-fixed obstacle for the obstacle scenario, expressed per tick in
    the recorded robot frame (Obstacle2DArray), at OBSTACLE_ODOM below.

Everything else -- odometry, /scan, map -> odom, base_link -> laser,
/perception/obstacles_2d, /costmap/boundaries -- is the recording itself.
"""

import base64
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

from f1tenth_messages.msg import Obstacle2D, Obstacle2DArray
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

ODOM_TOPIC = '/odometry/filtered'
TOPICS = [ODOM_TOPIC, '/scan', '/tf', '/tf_static', '/perception/obstacles_2d',
          '/costmap/boundaries']
TICK_NS = 100_000_000
# Where the 1.5 m lookahead target crosses it on the frozen-heading corridor
# of a move started at tick 2, about 0.3 m off the recorded path.
OBSTACLE_ODOM = (4.40, -1.00, 0.15)


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _read(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    out = {t: [] for t in TOPICS}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        out[topic].append((t_ns, bytes(raw), deserialize_message(raw, get_message(types[topic]))))
    return out, types


def _latest(stream, t_ns):
    best = None
    for item in stream:
        if item[0] > t_ns:
            break
        best = item
    return best


def _dead_ahead(scan, laser_pose, camera_x):
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment + laser_pose[2]
    sel = (np.abs(angles) <= math.radians(2.0)) & np.isfinite(ranges) & (ranges > scan.range_min)
    if not sel.any():
        return -1.0
    return float(np.median(ranges[sel])) + laser_pose[0] - camera_x


def main():
    run = Path(sys.argv[1])
    out_path = Path(sys.argv[2])
    t_start_s = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    n_ticks = int(sys.argv[4]) if len(sys.argv) > 4 else 40

    data, types = _read(run / 'bag')

    static = None
    camera_x = None
    for _t, raw, msg in data['/tf_static']:
        for tr in msg.transforms:
            if tr.header.frame_id == 'base_link' and tr.child_frame_id == 'laser':
                static = tr
            if tr.header.frame_id == 'base_link' and tr.child_frame_id == 'zed2_camera_link':
                camera_x = tr.transform.translation.x
    laser_pose = (static.transform.translation.x, static.transform.translation.y,
                  _yaw(static.transform.rotation))
    map_odom = [(t, tr) for t, _raw, msg in data['/tf'] for tr in msg.transforms
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom']

    blobs = {}

    def blob(type_name, raw):
        key = hashlib.sha256(raw).hexdigest()[:16]
        blobs.setdefault(key, {'type': type_name,
                               'cdr': base64.b64encode(raw).decode('ascii')})
        return key

    t0 = data[ODOM_TOPIC][0][0] + int(round(t_start_s * 1e9))
    ticks = []
    for k in range(n_ticks):
        clock_ns = t0 + k * TICK_NS
        odom = _latest(data[ODOM_TOPIC], clock_ns)
        scan = _latest(data['/scan'], clock_ns)
        tf = _latest(map_odom, clock_ns)
        obstacles = _latest(data['/perception/obstacles_2d'], clock_ns)
        boundaries = _latest(data['/costmap/boundaries'], clock_ns)

        pose = odom[2].pose.pose
        x, y, yaw = pose.position.x, pose.position.y, _yaw(pose.orientation)
        dx, dy = OBSTACLE_ODOM[0] - x, OBSTACLE_ODOM[1] - y
        synthetic = Obstacle2DArray()
        synthetic.obstacles = [Obstacle2D(
            x=float(math.cos(yaw) * dx + math.sin(yaw) * dy),
            y=float(-math.sin(yaw) * dx + math.cos(yaw) * dy),
            r=float(OBSTACLE_ODOM[2]))]

        ticks.append({
            'clock_ns': clock_ns,
            'odom': blob(types[ODOM_TOPIC], odom[1]),
            'scan': blob(types['/scan'], scan[1]) if scan else None,
            'map_odom': blob('geometry_msgs/msg/TransformStamped',
                             serialize_message(tf[1])) if tf else None,
            'obstacles': (blob(types['/perception/obstacles_2d'], obstacles[1])
                          if obstacles else None),
            'boundaries': (blob(types['/costmap/boundaries'], boundaries[1])
                           if boundaries else None),
            'front_distance': _dead_ahead(scan[2], laser_pose, camera_x) if scan else -1.0,
            'synthetic_obstacle': blob('f1tenth_messages/msg/Obstacle2DArray',
                                       serialize_message(synthetic)),
        })

    fixture = {
        'source_run': run.name,
        'source_bag_sha256': hashlib.sha256(
            (run / 'bag' / 'bag_0.db3').read_bytes()).hexdigest(),
        'window': {'t_start_s': t_start_s, 'n_ticks': n_ticks, 'tick_ns': TICK_NS,
                   'clock_origin': f'first {ODOM_TOPIC} receive time + t_start_s'},
        'derived': {
            'front_distance': ('scan dead-ahead median +-2 deg, from the camera origin '
                               f'(laser x {laser_pose[0]}, camera x {camera_x})'),
            'synthetic_obstacle_odom': list(OBSTACLE_ODOM),
        },
        'laser_static': blob('geometry_msgs/msg/TransformStamped', serialize_message(static)),
        'ticks': ticks,
        'blobs': blobs,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.GzipFile(out_path, 'wb', mtime=0) as f:
        f.write(json.dumps(fixture, sort_keys=True).encode('utf-8'))
    print(f'{out_path}: {n_ticks} ticks, {len(blobs)} distinct messages, '
          f'{out_path.stat().st_size} bytes')


if __name__ == '__main__':
    main()
