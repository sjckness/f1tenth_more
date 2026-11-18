#!/usr/bin/env python3
"""rclpy twin of ingest_probe_rs: no-op subscribers on heavy topics, counted.

    ingest_probe_py.py KIND:TOPIC ... [--touch] [--seconds S]

KIND: image | info | cloud | scan. Same QoS as the Rust probe (sensor data,
depth 2). --touch converts each message the way the nodes do
(np.frombuffer / np.asarray) and sums it, so every byte is read. Prints one
JSON line: message counts and bytes per topic. CPU is read from /proc by
the caller.
"""

import json
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2

TYPES = {'image': Image, 'info': CameraInfo, 'cloud': PointCloud2, 'scan': LaserScan}


def main():
    args = sys.argv[1:]
    touch = '--touch' in args
    seconds = float(args[args.index('--seconds') + 1]) if '--seconds' in args else 30.0
    rclpy.init()
    node = rclpy.create_node('ingest_probe_py')
    qos = QoSProfile(depth=2, reliability=qos_profile_sensor_data.reliability,
                     durability=qos_profile_sensor_data.durability)
    counts = {}
    for spec in (a for a in args if ':' in a):
        kind, topic = spec.split(':', 1)
        c = counts[topic] = {'msgs': 0, 'bytes': 0}

        def cb(msg, c=c, kind=kind):
            c['msgs'] += 1
            if kind in ('image', 'cloud'):
                c['bytes'] += len(msg.data)
                if touch:
                    np.frombuffer(msg.data, dtype=np.uint8).sum()
            elif kind == 'scan':
                c['bytes'] += 4 * len(msg.ranges)
                if touch:
                    np.asarray(msg.ranges, dtype=np.float64).sum()
        node.create_subscription(TYPES[kind], topic, cb, qos)
    t0 = time.monotonic()
    end = t0 + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(json.dumps({'wall_s': round(time.monotonic() - t0, 3), 'topics': counts}))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
