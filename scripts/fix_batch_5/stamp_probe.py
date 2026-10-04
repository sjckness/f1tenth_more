#!/usr/bin/env python3
"""Fix batch 5: receipt count vs distinct header.stamp count on Odometry topics.

Answers "does this EKF keep publishing after its input stops, and if so, does
its header.stamp freeze?" -- the premise of the watchdog's new_stamp check.

  stamp_probe.py --duration S [--bin B] TOPIC... -> JSON on stdout
      per topic: messages and distinct stamps overall, and per B-second bin
      (default 0.5 s) since the probe started: [messages, new stamps,
      header.stamp of the last message in the bin]
"""
import argparse
import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=5.0)
    ap.add_argument('--bin', type=float, default=0.5)
    ap.add_argument('topics', nargs='+')
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('stamp_probe')
    seen = {t: {'msgs': 0, 'stamps': set(), 'bins': {}} for t in a.topics}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    t0 = time.monotonic()

    def cb(m, t):
        s = seen[t]
        stamp = (m.header.stamp.sec, m.header.stamp.nanosec)
        b = s['bins'].setdefault(int((time.monotonic() - t0) / a.bin), [0, 0, None])
        b[0] += 1
        if stamp not in s['stamps']:
            b[1] += 1
        b[2] = round(stamp[0] + stamp[1] * 1e-9, 3)
        s['msgs'] += 1
        s['stamps'].add(stamp)

    for t in a.topics:
        node.create_subscription(Odometry, t, lambda m, t=t: cb(m, t), qos)
    end = time.monotonic() + a.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    nbins = int(a.duration / a.bin) + 1
    print(json.dumps({t: {'msgs': v['msgs'], 'distinct_stamps': len(v['stamps']),
                          'bin_s': a.bin,
                          'bins': [v['bins'].get(i, [0, 0, None]) for i in range(nbins)]}
                      for t, v in seen.items()}))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
