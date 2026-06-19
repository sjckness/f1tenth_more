"""Startup sequence: steer right, steer left, center.

Publishes AckermannDriveStamped to the high-priority mux input topic
(`/teleop` by default) so the sequence preempts whatever the MPC publishes
on `/drive` (mux gives joystick priority 100 vs navigation 10).
"""

import threading

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class StackStartupSequence(Node):

    def __init__(self):
        super().__init__('stack_startup_sequence')

        self.declare_parameter('startup_delay', 5.0)
        self.declare_parameter('right_duration', 2.0)
        self.declare_parameter('left_duration', 2.0)
        self.declare_parameter('center_duration', 1.0)
        self.declare_parameter('max_steering_angle', 0.18)
        self.declare_parameter('command_topic', '/teleop')
        self.declare_parameter('publish_rate_hz', 20.0)

        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.right_duration = float(self.get_parameter('right_duration').value)
        self.left_duration = float(self.get_parameter('left_duration').value)
        self.center_duration = float(self.get_parameter('center_duration').value)
        self.max_steer = float(self.get_parameter('max_steering_angle').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)

        self.pub = self.create_publisher(
            AckermannDriveStamped,
            self.command_topic,
            10,
        )

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _publish_cmd(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.pub.publish(msg)

    def _hold(self, steering, speed, duration):
        period = 1.0 / max(self.publish_rate, 1.0)
        end = self.get_clock().now().nanoseconds + int(duration * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            self._publish_cmd(steering, speed)
            threading.Event().wait(period)

    def _run(self):
        self.get_logger().info(
            f'[stack_bringup] Waiting {self.startup_delay:.1f}s for nodes to come up...'
        )
        threading.Event().wait(self.startup_delay)

        self.get_logger().info('[stack_bringup] Steering RIGHT (max)...')
        self._hold(-self.max_steer, 0.0, self.right_duration)

        self.get_logger().info('[stack_bringup] Steering LEFT (max)...')
        self._hold(self.max_steer, 0.0, self.left_duration)

        self.get_logger().info('[stack_bringup] Centering steering...')
        self._hold(0.0, 0.0, self.center_duration)

        self._publish_cmd(0.0, 0.0)
        self.get_logger().info('[stack_bringup] Startup sequence complete.')


def main(args=None):
    rclpy.init(args=args)
    node = StackStartupSequence()
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