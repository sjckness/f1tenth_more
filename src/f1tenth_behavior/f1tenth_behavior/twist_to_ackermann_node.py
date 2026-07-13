"""Bridges nav2_regulated_pure_pursuit_controller's Twist output (Nav2 has no
Ackermann-native controller plugin in this ROS distro) onto the ackermann_mux's
existing "navigation" lane (topic "drive"), the same lane the MPC node publishes on --
Nav2's controller becomes an alternative source for that lane, mutually exclusive with
the MPC via stack_bringup's use_behavior_tree arg.
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TwistToAckermannNode(Node):

    def __init__(self):
        super().__init__('twist_to_ackermann_node')

        # Real-car wheelbase (f1tenth_bringup/config/vesc.yaml,
        # vesc_to_odom_node.wheelbase).
        self.wheelbase = self.declare_parameter('wheelbase', 0.25).value
        # Back-solved from vesc.yaml's servo calibration (servo = gain*angle + offset,
        # gain=-1.2135, offset=0.5304, servo_min=0.15, servo_max=0.85) -- the real usable
        # range is asymmetric, not a guessed symmetric constant.
        self.min_steering_angle = self.declare_parameter('min_steering_angle', -0.264).value
        self.max_steering_angle = self.declare_parameter('max_steering_angle', 0.314).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value

        self.sub = self.create_subscription(
            Twist, 'cmd_vel_nav', self._twist_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)

    def _twist_callback(self, msg):
        # Bicycle-model inverse of the forward relation already used in
        # vesc_to_odom.cpp/vesc_to_odom_backup.cpp (angular_velocity = speed *
        # tan(steering_angle) / wheelbase): steering_angle = atan(angular.z * wheelbase
        # / v). Degenerate at v==0 (a car can't rotate in place); guard it to 0 steering.
        steering_angle = 0.0
        if abs(msg.linear.x) > 1e-3:
            steering_angle = math.atan2(msg.angular.z * self.wheelbase, msg.linear.x)
        steering_angle = max(
            self.min_steering_angle, min(self.max_steering_angle, steering_angle))

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.drive.speed = float(msg.linear.x)
        out.drive.steering_angle = float(steering_angle)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistToAckermannNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
