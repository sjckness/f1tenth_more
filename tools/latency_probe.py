#!/usr/bin/env python3
"""Arrival age (receipt minus header.stamp) and rate of the camera pipeline topics.

Best effort, depth 1, so it never back-pressures a publisher. A plain rclpy
node: it registers with the Discovery Server, so it sees the live stack where
the ros2 CLI does not. Usage (after `source install/setup.bash`):
    python3 tools/latency_probe.py DURATION_S [--images]
--images also subscribes to the raw image and depth (about 1 MB per message each).
Prints one JSON line at the end, plus the publishers' QoS.
"""
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2DArray, Detection3DArray

SMALL = [('/camera/detections', Detection2DArray), ('/camera/detections_3d', Detection3DArray),
         ('/camera/debug_image/compressed', CompressedImage)]
BIG = [('/camera/image_raw', Image), ('/zed2/zed_node/depth/depth_registered', Image),
       ('/camera/detection_masks', Image), ('/camera/image_annotated', Image)]


def main():
    duration = float(sys.argv[1])
    topics = SMALL + (BIG if '--images' in sys.argv else [])
    rclpy.init()
    node = Node('latency_probe')
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT)
    data = {t: [] for t, _ in topics}

    def cb(topic, msg):
        now = time.time()
        s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        data[topic].append((now, s))
    for topic, typ in topics:
        node.create_subscription(typ, topic, lambda m, t=topic: cb(t, m), qos)
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    out = {}
    for topic, rows in data.items():
        pubs = node.get_publishers_info_by_topic(topic)
        qos_txt = [f'{p.node_name}:{p.qos_profile.reliability.name}/'
                   f'{p.qos_profile.history.name}{p.qos_profile.depth}' for p in pubs]
        if len(rows) < 3:
            out[topic] = {'n': len(rows), 'pub_qos': qos_txt}
            continue
        arr = np.array(rows)
        age = (arr[:, 0] - arr[:, 1]) * 1e3
        span = arr[-1, 0] - arr[0, 0]
        stamps = np.unique(arr[:, 1])
        out[topic] = {'n': len(rows), 'rx_hz': round((len(rows) - 1) / span, 2),
                      'stamp_hz': round((len(stamps) - 1) / (stamps[-1] - stamps[0]), 2),
                      'age_p50': round(float(np.percentile(age, 50)), 1),
                      'age_p95': round(float(np.percentile(age, 95)), 1),
                      'pub_qos': qos_txt}
    print('PROBE ' + json.dumps(out))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
