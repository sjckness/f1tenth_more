"""Measures the VESC IMU's gyro (angular_velocity) static bias.

Run with the car completely stationary and level. Subscribes to the raw IMU topic
(default /sensors/imu/raw, matching f1tenth_bringup/config/ekf.yaml's imu0 source),
averages angular_velocity.x/y/z over a fixed sampling window, then logs the mean and
standard deviation per axis and shuts itself down. Read-only: this node never
publishes anything and never modifies live parameters on any other node -- it is
strictly a measurement tool, safe to run at any time without affecting the rest of
the stack.

See f1tenth_diagnostics/README.md for how to apply the measured bias.
"""

import statistics

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class GyroBiasCalibrationNode(Node):

    def __init__(self):
        super().__init__('gyro_bias_calibration_node')

        self.imu_topic = self.declare_parameter('imu_topic', '/zed/zed_node/imu/data').value
        self.sample_duration_sec = self.declare_parameter('sample_duration_sec', 30.0).value
        self.min_samples = self.declare_parameter('min_samples', 300).value

        self._samples_x = []
        self._samples_y = []
        self._samples_z = []

        self._sub = self.create_subscription(Imu, self.imu_topic, self._imu_callback, 50)
        self._timer = self.create_timer(self.sample_duration_sec, self._finish)

        self.get_logger().info(
            f'Sampling gyro bias on "{self.imu_topic}" for '
            f'{self.sample_duration_sec:.1f}s -- keep the car completely stationary.')

    def _imu_callback(self, msg: Imu):
        self._samples_x.append(msg.angular_velocity.x)
        self._samples_y.append(msg.angular_velocity.y)
        self._samples_z.append(msg.angular_velocity.z)

    def _finish(self):
        self._timer.cancel()
        count = len(self._samples_z)

        if count < 2:
            self.get_logger().error(
                f'Only {count} sample(s) received on "{self.imu_topic}" -- is the IMU '
                'publishing? No result to report; Ctrl+C to exit.')
            return

        if count < self.min_samples:
            self.get_logger().warning(
                f'Only {count} samples received (min_samples={self.min_samples}) -- '
                'result may be noisy. Consider a longer sample_duration_sec.')

        def report(axis, samples):
            mean = statistics.mean(samples)
            stdev = statistics.stdev(samples)
            self.get_logger().info(
                f'  angular_velocity.{axis}: mean={mean:+.6f} rad/s  stdev={stdev:.6f} rad/s')
            return mean

        self.get_logger().info(f'Gyro bias over {count} samples:')
        report('x', self._samples_x)
        report('y', self._samples_y)
        vyaw_bias = report('z', self._samples_z)
        self.get_logger().info(
            f'vyaw (z) bias = {vyaw_bias:+.6f} rad/s -- this is the value that matters '
            'for the planar EKF fusion (imu0_config keeps vyaw only). '
            'See f1tenth_diagnostics/README.md for where to apply it. Done -- Ctrl+C to exit.')


def main():
    rclpy.init()
    node = GyroBiasCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
