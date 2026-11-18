#!/usr/bin/env python3
"""
Publish one /mission/status (MissionStatus) the way MissionLoader does.

RELIABLE, TRANSIENT_LOCAL, KEEP_LAST depth 1 (loader.py MISSION_STATUS_QOS),
with the publisher kept alive HOLD_SEC so the mission logger receives it.

Runs on: the Orin host (native Humble), the harness container, linus.
Used by logger_check.sh to drive mission_logger_node through RUNNING ->
COMPLETE without a behaviour tree.

The message is published once a subscriber has matched (up to WAIT_SEC,
default 20: a fresh participant can need seconds to match on a loaded host),
then the publisher stays alive HOLD_SEC (transient-local history is only
served while the writer lives).

Usage: publish_mission_status.py STATE JSON_PATH [HOLD_SEC] [WAIT_SEC]
"""
import sys
import time

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                 history=QoSHistoryPolicy.KEEP_LAST)


def main():
    if len(sys.argv) < 3 or sys.argv[1] in ('-h', '--help'):
        print(__doc__)
        return 0 if len(sys.argv) > 1 and sys.argv[1] in ('-h', '--help') else 2
    from f1tenth_messages.msg import MissionStatus
    state, json_path = sys.argv[1], sys.argv[2]
    hold = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    wait = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0
    rclpy.init()
    node = rclpy.create_node('phase4_mission_status_driver')
    pub = node.create_publisher(MissionStatus, '/mission/status', QOS)
    msg = MissionStatus()
    msg.state = state
    msg.json_path = json_path
    msg.emergency_stop_active = False
    t0 = time.time()
    while time.time() - t0 < wait and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.05)
    matched_after = time.time() - t0
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    end = time.time() + hold
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    print('published %s (%s), subscribers=%d (matched after %.1f s)'
          % (state, json_path, pub.get_subscription_count(), matched_after))
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
