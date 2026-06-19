"""Drive bridge: real-stack /drive contract <-> sim Ackermann controller.

On the real F1TENTH the MPC / mux publish ackermann_msgs/AckermannDriveStamped
on /drive (speed + steering_angle). In sim the ros2_control
ackermann_steering_controller instead takes a body-velocity TwistStamped
reference and publishes its odometry under its own namespace.

This node makes the simulator a drop-in for the real drive contract:

  /drive (AckermannDriveStamped)
        --> /ackermann_steering_controller/reference (TwistStamped)
            linear.x  = speed
            angular.z = speed * tan(steering_angle) / wheelbase   (bicycle model)

  /ackermann_steering_controller/odometry (nav_msgs/Odometry)
        --> /odom    (so ekf.yaml's odom0:=odom can be reused verbatim)

The EKF owns the odom->base_link TF (the controller's enable_odom_tf is false),
matching the real stack.
"""
import math

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class DriveBridge(Node):

    def __init__(self):
        super().__init__('f1tenth_sim_drive_bridge')

        # Must match the URDF geometry / controllers.yaml wheelbase.
        self.declare_parameter('wheelbase', 0.325)
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter(
            'reference_topic', '/ackermann_steering_controller/reference')
        self.declare_parameter(
            'controller_odom_topic', '/ackermann_steering_controller/odometry')
        self.declare_parameter('odom_topic', '/odom')

        self.wheelbase = self.get_parameter('wheelbase').value

        self.ref_pub = self.create_publisher(
            TwistStamped,
            self.get_parameter('reference_topic').value,
            10)
        self.create_subscription(
            AckermannDriveStamped,
            self.get_parameter('drive_topic').value,
            self.on_drive,
            10)

        # Republish controller odometry on the canonical /odom topic for the EKF.
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self.create_subscription(
            Odometry,
            self.get_parameter('controller_odom_topic').value,
            self.on_odom,
            10)

        self.get_logger().info(
            'drive_bridge up: /drive -> reference (TwistStamped), '
            'controller odometry -> /odom (wheelbase=%.3f m)' % self.wheelbase)

    def on_drive(self, msg: AckermannDriveStamped):
        v = msg.drive.speed
        delta = msg.drive.steering_angle
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = v
        # Bicycle model: yaw rate from forward speed + steering angle.
        out.twist.angular.z = v * math.tan(delta) / self.wheelbase
        self.ref_pub.publish(out)

    def on_odom(self, msg: Odometry):
        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = DriveBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
