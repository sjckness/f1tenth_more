import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class MPCNode(Node):

    def __init__(self):
        super().__init__('mpc_controller')

        self.declare_parameter('qn', 20.0)
        self.declare_parameter('qv', 10.0)
        self.declare_parameter('qalpha', 7.0)
        self.declare_parameter('qac', 0.01)
        self.declare_parameter('qddelta', 0.1)
        self.declare_parameter('alat_max', 10.0)
        self.declare_parameter('a_min', -5.0)
        self.declare_parameter('a_max', 5.0)
        self.declare_parameter('v_min', 1.0)
        self.declare_parameter('v_max', 5.0)
        self.declare_parameter('track_safety_margin', 0.45)
        self.declare_parameter('v_ref', 3.0)

        self.current_velocity = 0.0

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info('Basic MPC controller started')

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x

    def control_loop(self):

        qv = self.get_parameter('qv').value
        a_min = self.get_parameter('a_min').value
        a_max = self.get_parameter('a_max').value
        v_min = self.get_parameter('v_min').value
        v_max = self.get_parameter('v_max').value
        v_ref = self.get_parameter('v_ref').value

        velocity_error = v_ref - self.current_velocity

        acceleration_cmd = 0.1 * qv * velocity_error

        acceleration_cmd = max(
            min(acceleration_cmd, a_max),
            a_min
        )

        target_speed = self.current_velocity + acceleration_cmd

        target_speed = max(
            min(target_speed, v_max),
            v_min
        )

        drive_msg = AckermannDriveStamped()

        drive_msg.header.stamp = self.get_clock().now().to_msg()

        drive_msg.drive.speed = float(target_speed)
        drive_msg.drive.steering_angle = 0.0

        self.drive_pub.publish(drive_msg)

        self.get_logger().info(
            f'v={self.current_velocity:.2f}, '
            f'v_ref={v_ref:.2f}, '
            f'qv={qv:.2f}, '
            f'cmd_speed={target_speed:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = MPCNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
