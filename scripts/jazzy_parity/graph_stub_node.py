#!/usr/bin/env python3
"""
A ROS node that only EXISTS under a given name.

No publishers, no subscriptions, no parameters.

Runs on: the Orin host (native Humble), the harness container, linus.
Used by phase5_bringup.sh (FEED=bag) and replay_localization.sh.

/mission/start_mission runs a preflight (f1tenth_behavior/mission/
preflight.py) that refuses to start unless nodes named `mpc_corr` and
`ackermann_to_vesc_node` (and lidar_front_wall_node for a front_clearance
stop) are in the graph. A bag-fed run has no hardware component, and
ackermann_to_vesc_node is a hardware driver that must never run there. A
stub with the right name satisfies the node-existence check and nothing
else; it cannot command anything because it publishes nothing.

Usage: graph_stub_node.py NODE_NAME [--ros-args ...]
"""
import sys

import rclpy


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help'):
        print(__doc__)
        return 0 if len(sys.argv) > 1 else 2
    rclpy.init(args=sys.argv[2:])
    node = rclpy.create_node(sys.argv[1])
    try:
        rclpy.spin(node)
    except BaseException:  # noqa: B036 - KeyboardInterrupt / ExternalShutdownException
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
