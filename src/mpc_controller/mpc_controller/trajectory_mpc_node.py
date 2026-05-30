import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class TrajectoryMPCNode(Node):
    def __init__(self):
        super().__init__('trajectory_mpc_controller')

        self.v_ref = 0.4          # m/s, basso per test
        self.steer_turn = 0.25    # rad, piccolo turn
        self.distance = 0.0
        self.last_x = None
        self.last_y = None

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('Trajectory MPC-like node started')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is not None:
            dx = x - self.last_x
            dy = y - self.last_y
            self.distance += math.sqrt(dx * dx + dy * dy)

        self.last_x = x
        self.last_y = y

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        if self.distance < 2.5:
            cmd.drive.speed = self.v_ref
            cmd.drive.steering_angle = 0.0
        elif self.distance < 3.0:
            cmd.drive.speed  = 0.4
            cmd.drive.steering_angle =-self.steer_turn
        else:
            cmd.drive.speed = 0.0
            cmd.drive.steering_angle = 0.0

        self.pub.publish(cmd)

        self.get_logger().info(
            f'd={self.distance:.2f} m, '
            f'speed={cmd.drive.speed:.2f}, '
            f'steer={cmd.drive.steering_angle:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
