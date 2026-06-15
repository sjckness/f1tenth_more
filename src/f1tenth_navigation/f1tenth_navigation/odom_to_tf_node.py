"""Broadcast the odom -> base_link transform from /odom (relative to start).

Subscribes to /odom (nav_msgs/Odometry) and publishes the TF transform
(parent: odom, child: base_link) as the displacement of the car RELATIVE to
the pose recorded on the first /odom message. So at startup base_link is the
identity in the odom frame, and the absolute starting pose is carried entirely
by the static map -> odom transform (see map_server_launch.py). This keeps the
two concerns separate: map->odom = where the car started, odom->base_link =
how far it has moved since.

NOTE: in the default f1tenth stack, vesc_to_odom_node already broadcasts this
exact transform (vesc.yaml `publish_tf: true`). This node is therefore an
*alternative* odom->base_link source for setups where vesc_to_odom's TF is
disabled (or a different odom source is used). Running both publishes two
transforms for the same frame pair, so it is launched disabled by default
(see map_server_launch.py `publish_odom_tf`).
"""

import math

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

        # Origin (first-message pose) — recorded lazily on the first /odom msg.
        self.origin_set = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0

        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info('[odom_tf] Waiting for first /odom message...')

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        curr_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # Record the odom origin on the first message; nothing to broadcast yet.
        if not self.origin_set:
            self.origin_x = curr_x
            self.origin_y = curr_y
            self.origin_yaw = curr_yaw
            self.origin_set = True
            self.get_logger().info(
                f'[odom_tf] Origin set: x={self.origin_x:.4f} '
                f'y={self.origin_y:.4f} yaw={self.origin_yaw:.4f}'
            )

        # Displacement since the origin, rotated into the odom frame (i.e.
        # unrotated by the origin yaw) so that a car starting at any heading
        # begins at identity in odom.
        dx = curr_x - self.origin_x
        dy = curr_y - self.origin_y
        dyaw = curr_yaw - self.origin_yaw

        c = math.cos(-self.origin_yaw)
        s = math.sin(-self.origin_yaw)
        rel_x = c * dx - s * dy
        rel_y = s * dx + c * dy
        rel_yaw = dyaw

        qz_out = math.sin(rel_yaw / 2.0)
        qw_out = math.cos(rel_yaw / 2.0)

        t = TransformStamped()
        # Reuse the odom message stamp so TF and odometry are time-aligned.
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = rel_x
        t.transform.translation.y = rel_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz_out
        t.transform.rotation.w = qw_out

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
