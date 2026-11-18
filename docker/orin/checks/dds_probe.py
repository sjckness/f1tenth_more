#!/usr/bin/env python3
"""
DDS talker and listener: a topic flowing through the current DDS setup.

Runs on: the Orin host (native Humble), inside the container (car or harness
role), linus. From the spike gate G5 (docker/orin/spike/gates/dds_probe.py on
the jazzy line), with arguments, so ds_probe.sh and lease_probe.py can use it
instead of demo_nodes_cpp (not in the image).

  dds_probe.py talker   SECONDS [--topic T] [--node N] [--rate HZ]
  dds_probe.py listener SECONDS [--topic T] [--node N]
  dds_probe.py idle     SECONDS [--node N]     a participant, nothing else

The talker publishes a counter at --rate (50 Hz) on --topic (/p5d_probe).
The listener prints one JSON line: messages received, the counter span,
gaps (lost or reordered samples) and the time to the first message. Pass
criterion where a check uses it: received > 0 and, on one host, gaps == 0.
The environment decides everything else (ROS_DOMAIN_ID, ROS_DISCOVERY_SERVER,
ROS_SUPER_CLIENT, FASTRTPS_DEFAULT_PROFILES_FILE), as for any node.
"""
import argparse
import json
import sys
import time

import rclpy
from std_msgs.msg import String


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('  dds_probe.py'):])
    ap.add_argument('role', choices=['talker', 'listener', 'idle'])
    ap.add_argument('seconds', type=float)
    ap.add_argument('--topic', default='/p5d_probe')
    ap.add_argument('--node', default='')
    ap.add_argument('--rate', type=float, default=50.0)
    a = ap.parse_args(argv)
    rclpy.init()
    node = rclpy.create_node(a.node or 'p5d_probe_' + a.role)
    count = [0]
    got = []
    if a.role == 'talker':
        pub = node.create_publisher(String, a.topic, 10)

        def tick():
            msg = String()
            msg.data = str(count[0])
            pub.publish(msg)
            count[0] += 1
        node.create_timer(1.0 / a.rate, tick)
    elif a.role == 'listener':
        node.create_subscription(String, a.topic, lambda m: got.append(int(m.data)), 10)
    t_start = time.time()
    end = t_start + a.seconds
    first_rx = None
    try:
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if got and first_rx is None:
                first_rx = time.time()
    except KeyboardInterrupt:
        pass
    if a.role == 'talker':
        print(json.dumps({'role': 'talker', 'node': node.get_name(), 'sent': count[0]}))
    elif a.role == 'listener':
        gaps = sum(1 for x, y in zip(got, got[1:]) if y != x + 1)
        print(json.dumps({
            'role': 'listener', 'node': node.get_name(), 'received': len(got),
            'span': (got[-1] - got[0] + 1) if got else 0, 'gaps': gaps,
            'first_rx_after_s': round(first_rx - t_start, 2) if first_rx else None}))
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
