"""Broadcast the odom -> base_link transform from /odom.

Subscribes to /odom (nav_msgs/Odometry) and re-publishes the pose as a TF
transform (parent: odom, child: base_link), passing the orientation quaternion
through unchanged.

NOTE: in the default f1tenth stack, vesc_to_odom_node already broadcasts this
exact transform (vesc.yaml `publish_tf: true`). This node is therefore an
*alternative* odom->base_link source for setups where vesc_to_odom's TF is
disabled (or a different odom source is used). Running it alongside
vesc_to_odom would publish two transforms for the same frame pair — so it is
launched disabled by default (see map_server_launch.py `publish_odom_tf`).
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


class OdomToTfNode(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.odom_frame = str(self.declare_parameter('odom_frame', 'odom').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)

        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info(
            f'odom_tf_broadcaster up: {self.odom_frame} -> {self.base_frame} from /odom'
        )

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        # Reuse the odom message stamp so TF and odometry are time-aligned.
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        # Orientation passes through directly (already a unit quaternion).
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfNode()
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
