#!/usr/bin/env python3
"""G5 (DDS): talker / listener on /p5d_probe through the Discovery Server.
usage: dds_probe.py talker|listener SECONDS"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

role, seconds = sys.argv[1], float(sys.argv[2])
rclpy.init()
node = Node(f'p5d_probe_{role}')
count = [0]
got = []
if role == 'talker':
    pub = node.create_publisher(String, '/p5d_probe', 10)

    def tick():
        msg = String()
        msg.data = str(count[0])
        pub.publish(msg)
        count[0] += 1
    node.create_timer(0.02, tick)  # 50 Hz
else:
    node.create_subscription(String, '/p5d_probe', lambda m: got.append(int(m.data)), 10)
end = time.time() + seconds
first_rx = None
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
    if got and first_rx is None:
        first_rx = time.time()
if role == 'talker':
    print(f'talker sent {count[0]}')
else:
    gaps = sum(1 for a, b in zip(got, got[1:]) if b != a + 1)
    span = (got[-1] - got[0] + 1) if got else 0
    print(f'listener received {len(got)} span {span} gaps {gaps} '
          f'first_rx_after_s {round(first_rx - (end - seconds), 2) if first_rx else None}')
node.destroy_node()
rclpy.shutdown()
