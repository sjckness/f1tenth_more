#!/usr/bin/env python3
"""Publish test messages on the topics fix batch 1 added to
mission_logger_node's _DEFAULT_TOPICS, each with its production publisher's
QoS, so logger_check.sh can show they land in the recording.

Every production publisher of these topics uses depth 10 with rclpy/rclcpp
defaults (RELIABLE, VOLATILE, KEEP_LAST); the source line is next to each
topic. /imu has no publisher in this workspace (the ZED's publish_imu is
false); it is published here with the subscriber's own QoS (MPC_corr.py,
depth 10). None of the source bags contains any of these topics, so a topic
in the recording came from here.

Usage: publish_logger_topics.py DURATION_S [RATE_HZ]
"""
import sys
import time

import rclpy
from f1tenth_messages.msg import DriveCommand
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, String

RELIABLE_10 = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST)

TOPICS = [
    # topic, type, QoS, production publisher
    ('/mpc/goal_drive', DriveCommand, RELIABLE_10,
     'f1tenth_behavior/behaviours/publish_move_goal.py:108'),
    ('/imu', Imu, RELIABLE_10, 'no publisher in workspace; subscriber MPC_corr.py:1718'),
    ('/joint_states', JointState, RELIABLE_10,
     'joint_state_publisher (apt) joint_state_publisher.py:425'),
    ('/perception/front_distance', Float32, RELIABLE_10,
     'f1tenth_perception/front_clearance_node.py:508'),
    ('/perception/d_wall/psi_correction', Float32, RELIABLE_10,
     'f1tenth_perception/wall_distance_node.py:268'),
    ('/mpc/status', String, RELIABLE_10, 'mpc_controller/MPC_corr.py:1805'),
    ('/sensors/imu/raw', Imu, RELIABLE_10, 'vesc_driver/src/vesc_driver.cpp:123'),
]


def main():
    duration = float(sys.argv[1])
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    rclpy.init()
    node = rclpy.create_node('fix_batch_1_logger_topics')
    pubs = [(t, typ, node.create_publisher(typ, t, qos)) for t, typ, qos, _ in TOPICS]
    sent = 0
    t_end = time.monotonic() + duration
    while time.monotonic() < t_end:
        for _, typ, pub in pubs:
            pub.publish(typ())
        sent += 1
        rclpy.spin_once(node, timeout_sec=1.0 / rate)
    print(f'published {sent} messages on each of {len(pubs)} topics')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
