#!/usr/bin/env python3
"""A ROS node that only EXISTS under a given name -- no publishers, no
subscriptions, no parameters.

Phase 4 BT replay: /mission/start_mission runs a preflight
(f1tenth_behavior/mission/preflight.py) that refuses to start unless nodes
named `mpc_corr` and `ackermann_to_vesc_node` (and, for a front_clearance
stop condition, `costmap_boundary_node`) are in the graph. The BT layer is
replayed in isolation, like every other layer in this harness, and
ackermann_to_vesc_node is a hardware driver that must never run here. A stub
with the right name satisfies the node-existence check and nothing else; it
cannot command anything because it publishes nothing.

Usage: graph_stub_node.py NODE_NAME [--ros-args ...]
"""
import sys

import rclpy


def main():
    name = sys.argv[1]
    rclpy.init(args=sys.argv[2:])
    node = rclpy.create_node(name)
    try:
        rclpy.spin(node)
    except BaseException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
