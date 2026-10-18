#!/usr/bin/env python3
"""Offline cost of heavy-message ingest in Python, per message, on real data.

For each heavy message type of the stack, on recorded messages (car bag
2026-10-15T15-18-27) and a synthetic ZED RGB frame (the raw RGB is not
recorded), times:
  deserialize  rclpy.serialization.deserialize_message (what rclpy's take does)
  convert      the node's own conversion call (cv_bridge imgmsg_to_cv2 with
               the node's encoding, np.frombuffer, read_points_numpy,
               np.asarray(ranges)), as found in f1tenth_perception & co.
Prints a JSON report: bytes, p50/p95 ms per step.

Usage (workspace sourced): bench_ingest.py [--bag DIR] [--n N] [--out FILE]
"""

import argparse
import json
import os
import time

from cv_bridge import CvBridge
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy

DEFAULT_BAG = os.path.expanduser(
    '~/f1tenth_archive/mission_row/2026-10-15T15-18-27_mission-llm_86cff518e42d/bag')
BRIDGE = CvBridge()


def conversions():
    """(topic or synthetic name, type, [(label, function of msg)])."""
    def asarray_ranges(m):
        return np.asarray(m.ranges, dtype=np.float64)

    def bytes_copy_frombuffer(m):   # swept_clearance_node.py:92
        return np.frombuffer(bytes(m.data), dtype=np.float32)

    return [
        ('/scan', LaserScan, [('np.asarray(ranges) [IsProximityTooClose, lidar_front_wall, '
                               'wall_distance, front_clearance, ...]', asarray_ranges)]),
        ('/zed2/zed_node/depth/depth_registered_5hz', Image, [
            ("imgmsg_to_cv2('32FC1') [detection_3d, front_clearance, front_depth_monitor]",
             lambda m: BRIDGE.imgmsg_to_cv2(m, desired_encoding='32FC1')),
            ('np.frombuffer(bytes(data)) [swept_clearance]', bytes_copy_frombuffer)]),
        ('/camera/detection_masks', Image, [
            ("imgmsg_to_cv2('mono8') [detection_3d, front_clearance]",
             lambda m: BRIDGE.imgmsg_to_cv2(m, desired_encoding='mono8'))]),
        ('/perception/scene_labels', Image, [
            ("imgmsg_to_cv2('mono8') [yolo debug]",
             lambda m: BRIDGE.imgmsg_to_cv2(m, 'mono8')),
            ('np.frombuffer [lidar_front_wall]',
             lambda m: np.frombuffer(m.data, dtype=np.uint8))]),
        ('/perception/wall_points', PointCloud2, [
            ('read_points_numpy(x,y,z) [lidar_front_wall]',
             lambda m: read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True))]),
        ('/camera/camera_info', CameraInfo, [('(fields only)', lambda m: m.k[0])]),
        ('synthetic ZED RGB 640x360 bgra8', Image, [
            ("imgmsg_to_cv2('bgr8') [yolo_detector, front_clearance debug]",
             lambda m: BRIDGE.imgmsg_to_cv2(m, desired_encoding='bgr8'))]),
        ('synthetic ZED depth 640x360 32FC1 (full rate topic)', Image, [
            ("imgmsg_to_cv2('32FC1')",
             lambda m: BRIDGE.imgmsg_to_cv2(m, desired_encoding='32FC1'))]),
    ]


def synthetic(name):
    m = Image()
    m.height, m.width = 360, 640
    if 'RGB' in name:
        m.encoding, m.step = 'bgra8', 640 * 4
        m.data = np.random.default_rng(0).integers(0, 255, 360 * 640 * 4, dtype=np.uint8).tobytes()
    else:
        m.encoding, m.step = '32FC1', 640 * 4
        m.data = np.random.default_rng(0).random(360 * 640, dtype=np.float32).tobytes()
    return serialize_message(m)


def stats(samples):
    v = np.array(samples) * 1e3
    return dict(p50=round(float(np.percentile(v, 50)), 4), p95=round(float(np.percentile(v, 95)), 4))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', default=DEFAULT_BAG)
    ap.add_argument('--n', type=int, default=200)
    ap.add_argument('--out', default='output/rust/bench_ingest.json')
    a = ap.parse_args()
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    raw = {}
    while r.has_next():
        topic, data, _ = r.read_next()
        raw.setdefault(topic, []).append(data)
    out = []
    for name, typ, convs in conversions():
        msgs = raw.get(name) or [synthetic(name)]
        des, conv = [], {label: [] for label, _ in convs}
        for i in range(a.n):
            data = msgs[i % len(msgs)]
            t0 = time.perf_counter()
            m = deserialize_message(data, typ)
            des.append(time.perf_counter() - t0)
            for label, fn in convs:
                t0 = time.perf_counter()
                fn(m)
                conv[label].append(time.perf_counter() - t0)
        out.append(dict(topic=name, type=typ.__name__, serialized_bytes=len(msgs[0]),
                        recorded=name in raw, deserialize_ms=stats(des),
                        convert_ms={k: stats(v) for k, v in conv.items()}))
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    with open(a.out, 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
