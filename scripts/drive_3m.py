#!/usr/bin/env python3
"""
drive_straight_3m.py -- drive straight until a target distance is covered,
measured from /odom, then stop.

For steering offset calibration: publishes steering_angle 0.0 and a fixed
speed, integrates distance travelled from odom position, and cuts the
command at the target. Reports the actual distance covered (including any
coast after the stop command) so it can be fed to the offset calculation
instead of an assumed 3.0 m.

Usage:
    python3 drive_straight_3m.py                 # 3.0 m at 0.5 m/s
    python3 drive_straight_3m.py 3.0 0.3         # explicit distance, speed
"""

import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


class DriveStraight(Node):
    def __init__(self, target_m, speed):
        super().__init__('drive_straight_3m')
        self.target_m = target_m
        self.speed = speed

        self.start_xy = None
        self.dist = 0.0
        self.start_yaw = None
        self.last_yaw = None
        self.stopping = False
        self.stop_ticks = 0

        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)  # 20 Hz

        self.get_logger().info(f'Driving straight {target_m:.2f} m at {speed:.2f} m/s')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if self.start_xy is None:
            self.start_xy = (p.x, p.y)
            self.start_yaw = yaw

        dx = p.x - self.start_xy[0]
        dy = p.y - self.start_xy[1]
        self.dist = math.hypot(dx, dy)
        self.last_yaw = yaw

    def tick(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = 0.0

        if self.start_xy is None:
            # No odom yet -- do not move.
            msg.drive.speed = 0.0
            self.pub.publish(msg)
            return

        if not self.stopping and self.dist < self.target_m:
            msg.drive.speed = float(self.speed)
            self.pub.publish(msg)
            return

        # Target reached: hold zero for a moment so any coast is captured
        # in the final distance report.
        self.stopping = True
        msg.drive.speed = 0.0
        self.pub.publish(msg)
        self.stop_ticks += 1

        if self.stop_ticks >= 20:  # 1 s of zero command
            yaw_drift = math.degrees(self.last_yaw - self.start_yaw)
            # normalize to (-180, 180]
            yaw_drift = (yaw_drift + 180.0) % 360.0 - 180.0
            self.get_logger().info(
                f'DONE. odom distance {self.dist:.3f} m '
                f'(target {self.target_m:.2f}), odom yaw drift {yaw_drift:+.2f} deg')
            self.get_logger().info(
                'Measure lateral drift with a tape and report BOTH that and '
                'the odom distance above -- do not assume 3.00 m.')
            raise SystemExit


def main():
    target = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

    rclpy.init()
    node = DriveStraight(target, speed)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Best-effort stop on any exit path.
        try:
            stop = AckermannDriveStamped()
            stop.drive.speed = 0.0
            stop.drive.steering_angle = 0.0
            for _ in range(5):
                node.pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()