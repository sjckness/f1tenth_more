"""Broadcast map -> odom as a direct, unfiltered mirror of /odom's pose.

Fallback map -> odom source for localization_source:='raw_odom' (see
f1tenth_bringup/launch/stack_bringup.launch.py), used in place of the EKF
(f1tenth_localization/launch/ekf.launch.py) while EKF tuning is in progress.
No filtering, no timer -- every /odom message is republished as map -> odom
verbatim (pose.pose taken as-is), on receipt.
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


class RawOdomMapTfNode(Node):
    def __init__(self):
        super().__init__('raw_odom_map_tf_node')

        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info(
            '[raw_odom_map_tf] Mirroring /odom onto map -> odom (unfiltered fallback).')

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = RawOdomMapTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
