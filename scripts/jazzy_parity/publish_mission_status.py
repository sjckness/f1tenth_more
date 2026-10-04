#!/usr/bin/env python3
"""Publish one /mission/status (MissionStatus) the way MissionLoader does --
RELIABLE, TRANSIENT_LOCAL, KEEP_LAST depth 1 (loader.py MISSION_STATUS_QOS) --
and keep the publisher alive HOLD_SEC so the mission logger receives it.

Phase 4 logger check: drives mission_logger_node through RUNNING -> COMPLETE
without a behaviour tree. Usage: publish_mission_status.py STATE JSON_PATH [HOLD_SEC]
"""
import sys
import time

import rclpy
from f1tenth_messages.msg import MissionStatus
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                 history=QoSHistoryPolicy.KEEP_LAST)


def main():
    state, json_path = sys.argv[1], sys.argv[2]
    hold = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    rclpy.init()
    node = rclpy.create_node('phase4_mission_status_driver')
    pub = node.create_publisher(MissionStatus, '/mission/status', QOS)
    msg = MissionStatus()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.state = state
    msg.json_path = json_path
    msg.emergency_stop_active = False
    end = time.time() + hold
    published = False
    while time.time() < end:
        if not published and pub.get_subscription_count() > 0:
            pub.publish(msg)
            published = True
        rclpy.spin_once(node, timeout_sec=0.05)
    if not published:
        pub.publish(msg)
    print('published %s (%s), subscribers=%d' % (state, json_path, pub.get_subscription_count()))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
