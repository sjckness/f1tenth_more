#!/usr/bin/env python3
"""Fix batch 3: run the stack's graph-reading calls against the running full
stack, as the process that makes them would (plain Discovery Server client
unless the environment says otherwise), and print what they return.

  graph_calls_live.py OUT_JSON

1. ekf_cost_observer_node: reads its published DiagnosticStatus on
   /diagnostics for DIAG_SEC and reports, per EKF, the fields that show
   whether it subscribed to anything (ticks_selfcount, meas_delivered, ...).
2. steering_offset_calibration_node: constructs the REAL node class (it only
   creates its publishers/subscriptions; nothing is commanded -- run_preflight,
   the nudge and the drive are never called) and runs its stage-1 graph
   checks: _participants_visible() (node names) and _check_estop_path()
   (count_subscribers(/calibration_drive), count_publishers(/safety_stop)).
3. The generic API classes, from one bare node: node names, foreign topic
   types, count_publishers/count_subscribers on a topic with and without an
   endpoint of our own, a publisher's get_subscription_count(),
   get_service_names_and_types(), wait_for_service().

Environment as inherited; ROS_SUPER_CLIENT is recorded in the output.
"""
import json
import os
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray

DIAG_SEC = 12.0


def spin(node, sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def ekf_cost(node):
    seen = {}

    def cb(msg):
        for st in msg.status:
            if 'ekf_cost' in st.name:
                seen[st.name] = {'level': int.from_bytes(st.level, 'little')
                                 if isinstance(st.level, bytes) else int(st.level),
                                 'message': st.message,
                                 **{kv.key: kv.value for kv in st.values}}
    node.create_subscription(DiagnosticArray, '/diagnostics', cb, 50)
    spin(node, DIAG_SEC)
    return seen


def steering_calibration():
    from f1tenth_diagnostics.steering_offset_calibration_node import (
        SteeringOffsetCalibrationNode)
    node = SteeringOffsetCalibrationNode()
    spin(node, 10.0)  # discovery settles, as the node's own preflight window does
    res = {
        'participants_visible': node._participants_visible(),
        'count_subscribers(/calibration_drive)': node.count_subscribers(node.drive_topic),
        'count_publishers(/safety_stop)': node.count_publishers(node.safety_stop_topic),
        'require_estop_publisher': node.require_estop_publisher,
    }
    res['_check_estop_path()'] = node._check_estop_path()
    node.destroy_node()
    return res


def generic(node):
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    pub = node.create_publisher(String, '/fb3_probe_own', 10)  # nobody else uses it
    from ackermann_msgs.msg import AckermannDriveStamped
    drive_pub = node.create_publisher(AckermannDriveStamped, '/calibration_drive', 10)
    cli = node.create_client(Trigger, '/mission/start_mission')
    spin(node, 10.0)
    types = dict(node.get_topic_names_and_types())
    svcs = dict(node.get_service_names_and_types())
    return {
        'node_names': len(node.get_node_names()),
        'topic_names_and_types': len(types),
        'foreign topic /odometry/filtered in get_topic_names_and_types': '/odometry/filtered' in types,
        'count_publishers(/odometry/filtered), no own endpoint': node.count_publishers('/odometry/filtered'),
        'count_subscribers(/calibration_drive), own publisher': node.count_subscribers('/calibration_drive'),
        'publisher.get_subscription_count() on /calibration_drive': drive_pub.get_subscription_count(),
        'count_publishers(/safety_stop), no own endpoint': node.count_publishers('/safety_stop'),
        'service_names_and_types': len(svcs),
        '/restart_component in get_service_names_and_types': '/restart_component' in svcs,
        'wait_for_service(/mission/start_mission, 5 s)': cli.wait_for_service(timeout_sec=5.0),
        '_unused': pub.topic_name,
    }


def main():
    out = {'ROS_SUPER_CLIENT': os.environ.get('ROS_SUPER_CLIENT', '<unset>'),
           'ROS_DISCOVERY_SERVER': os.environ.get('ROS_DISCOVERY_SERVER', '<unset>')}
    rclpy.init()
    probe = rclpy.create_node('fb3_graph_probe')
    out['ekf_cost_observer_diagnostics'] = ekf_cost(probe)
    out['generic'] = generic(probe)
    probe.destroy_node()
    out['steering_offset_calibration'] = steering_calibration()
    rclpy.try_shutdown()
    json.dump(out, open(sys.argv[1], 'w'), indent=2, default=str)
    print(json.dumps(out, indent=2, default=str))


if __name__ == '__main__':
    main()
