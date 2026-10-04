#!/usr/bin/env python3
"""Fix batch 5: receipt wall times of topics, until killed.

  rx_probe.py --out FILE.json TOPIC:TYPE...

Raw BEST_EFFORT/VOLATILE subscriptions (no deserialization). Writes
{topic: [receipt epoch, ...]} on exit. Used for /slam/map's real interval.
"""
import argparse
import json
import signal
import time

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('topics', nargs='+')
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('rx_probe')
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     durability=QoSDurabilityPolicy.VOLATILE)
    times = {}
    for spec in a.topics:
        topic, type_name = spec.split(':', 1)
        times[topic] = []
        node.create_subscription(get_message(type_name), topic,
                                 lambda _m, t=topic: times[t].append(round(time.time(), 3)),
                                 qos, raw=True)
    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    while not stop[0]:
        rclpy.spin_once(node, timeout_sec=0.2)
    with open(a.out, 'w') as f:
        json.dump(times, f)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
