#!/usr/bin/env python3
"""Fix batch 5 follow-up (B6, backlog M19): wall time of the first receipts of
/scan and /slam/map, until killed.

slam_toolbox publishes its first map one map_update_interval (5 s of the
node's clock: sim time with use_sim_time) after it has processed a scan, plus
the grid rebuild. This records that delay from the outside, in wall time, as
the watchdog sees it: the input to the `once` check's settle_sec default.

  first_rx.py --out FILE.json

Writes FILE.json on every first receipt and at exit:
  {"<topic>": {"first": <unix s>, "count": n}, ...}
Run as a super client (ROS_SUPER_CLIENT=TRUE) on the stack's domain.
"""
import argparse
import json
import signal
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

TOPICS = {'/scan': LaserScan, '/slam/map': OccupancyGrid}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('first_rx_probe')
    # Best effort + volatile, as the supervisor's watchdog reads: matches any
    # publisher, and a latched old map is not replayed as a fresh receipt.
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     durability=QoSDurabilityPolicy.VOLATILE)
    state = {t: {'first': None, 'count': 0} for t in TOPICS}

    def dump():
        with open(a.out, 'w') as f:
            json.dump(state, f, indent=1)

    def cb_for(topic):
        def cb(_msg):
            s = state[topic]
            s['count'] += 1
            if s['first'] is None:
                s['first'] = time.time()
                dump()
        return cb

    for t, typ in TOPICS.items():
        node.create_subscription(typ, t, cb_for(t), qos, raw=True)
    stop = [False]
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    dump()
    while not stop[0]:
        rclpy.spin_once(node, timeout_sec=0.2)
    dump()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
