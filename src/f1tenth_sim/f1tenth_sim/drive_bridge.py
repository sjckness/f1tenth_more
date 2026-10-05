"""Drive bridge: makes the sim a drop-in for the car's VESC drivers.

On the car, ackermann_mux outputs ackermann_msgs/AckermannDriveStamped on
/ackermann_drive, which ackermann_to_vesc turns into motor/servo commands, and
the VESC side publishes /odom (vesc_to_odom) and /sensors/imu/raw (vesc_driver).
In sim the ros2_control ackermann_steering_controller takes a body-velocity
TwistStamped reference and publishes odometry under its own namespace, and the
gz IMU arrives through ros_gz_bridge. This node adapts both directions:

  /ackermann_drive (AckermannDriveStamped)          [behind ackermann_mux, D1]
        --> /ackermann_steering_controller/reference (TwistStamped)
            steering_angle clamped to the real servo envelope   [D2]
            linear.x  = speed
            angular.z = speed * tan(steering_angle) / wheelbase   (bicycle model)

  /ackermann_steering_controller/odometry (nav_msgs/Odometry)
        --> /odom    (what vesc_to_odom publishes on the car)

  /sim/imu_raw (sensor_msgs/Imu, from ros_gz_bridge)
        --> /sensors/imu/raw   frame_id "" and the vesc.yaml covariances,
            exactly like vesc_driver on the car  [D3]

Decisions D1-D3: output/sim_port_report.md. Nothing here touches /tf: the
Thor's EKF owns odom->base_link (the controller's enable_odom_tf is false).
"""
import signal

from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_sim.kinematics import (
    ackermann_to_twist,
    STEERING_MAX_RAD,
    STEERING_MIN_RAD,
)
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class DriveBridge(Node):

    def __init__(self):
        super().__init__('f1tenth_sim_drive_bridge')

        # Must match the URDF geometry / controllers.yaml wheelbase.
        self.declare_parameter('wheelbase', 0.325)
        # Real servo envelope as steering angle; see kinematics.py for the
        # derivation from steering_calibration.yaml.
        self.declare_parameter('steering_min', STEERING_MIN_RAD)
        self.declare_parameter('steering_max', STEERING_MAX_RAD)
        self.declare_parameter('drive_topic', '/ackermann_drive')
        self.declare_parameter(
            'reference_topic', '/ackermann_steering_controller/reference')
        self.declare_parameter(
            'controller_odom_topic', '/ackermann_steering_controller/odometry')
        self.declare_parameter('odom_topic', '/odom')

        # IMU relay. Defaults copied from f1tenth_bringup/config/vesc.yaml
        # (gyro_variance_* / accel_variance_*), which vesc_driver writes into
        # the covariance diagonals. imu_frame_id stays "" like the real driver,
        # which never sets header.frame_id (backlog B1 is to fix both).
        self.declare_parameter('sim_imu_topic', '/sim/imu_raw')
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('imu_frame_id', '')
        self.declare_parameter('gyro_variance_x', 0.030539653513912038)
        self.declare_parameter('gyro_variance_y', 0.005006719643874733)
        self.declare_parameter('gyro_variance_z', 1.746756e-06)
        self.declare_parameter('accel_variance_x', 6.838468054307688e-06)
        self.declare_parameter('accel_variance_y', 7.457561226365246e-06)
        self.declare_parameter('accel_variance_z', 3.9237810323728924e-05)

        p = self.get_parameter
        self.wheelbase = p('wheelbase').value
        self.steering_min = p('steering_min').value
        self.steering_max = p('steering_max').value
        self.imu_frame_id = p('imu_frame_id').value
        gx, gy, gz = (p('gyro_variance_' + a).value for a in 'xyz')
        ax, ay, az = (p('accel_variance_' + a).value for a in 'xyz')
        self.gyro_cov = [gx, 0.0, 0.0, 0.0, gy, 0.0, 0.0, 0.0, gz]
        self.accel_cov = [ax, 0.0, 0.0, 0.0, ay, 0.0, 0.0, 0.0, az]

        self.ref_pub = self.create_publisher(
            TwistStamped, p('reference_topic').value, 10)
        self.create_subscription(
            AckermannDriveStamped, p('drive_topic').value, self.on_drive, 10)

        self.odom_pub = self.create_publisher(
            Odometry, p('odom_topic').value, 10)
        self.create_subscription(
            Odometry, p('controller_odom_topic').value, self.on_odom,
            qos_profile_sensor_data)

        self.imu_pub = self.create_publisher(Imu, p('imu_topic').value, 10)
        self.create_subscription(
            Imu, p('sim_imu_topic').value, self.on_imu, qos_profile_sensor_data)

        self.get_logger().info(
            'drive_bridge up: %s -> reference (wheelbase=%.3f m, steering '
            'clamped to [%.4f, %.4f] rad); controller odometry -> %s; '
            '%s -> %s (frame_id=%r)' % (
                p('drive_topic').value, self.wheelbase, self.steering_min,
                self.steering_max, p('odom_topic').value,
                p('sim_imu_topic').value, p('imu_topic').value,
                self.imu_frame_id))

    def on_drive(self, msg: AckermannDriveStamped):
        v, omega = ackermann_to_twist(
            msg.drive.speed, msg.drive.steering_angle, self.wheelbase,
            self.steering_min, self.steering_max)
        out = TwistStamped()
        # The controller drops references older than reference_timeout, so the
        # stamp is "now" on the sim clock, not the incoming header's.
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = v
        out.twist.angular.z = omega
        self.ref_pub.publish(out)

    def on_odom(self, msg: Odometry):
        self.odom_pub.publish(msg)

    def on_imu(self, msg: Imu):
        msg.header.frame_id = self.imu_frame_id
        msg.angular_velocity_covariance = self.gyro_cov
        msg.linear_acceleration_covariance = self.accel_cov
        # vesc_driver leaves orientation_covariance at its all-zero default.
        msg.orientation_covariance = [0.0] * 9
        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = DriveBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal Ctrl-C reaches every process in the group and ros2 launch
        # forwards its own SIGINT on top, so a second one used to land inside
        # destroy_node() as a KeyboardInterrupt. Teardown is short; finish it.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        # On SIGINT/SIGTERM Jazzy's rclpy signal handler has already shut the
        # context down, and a second rclpy.shutdown() raises RCLError, so the
        # node exited with code 1 on every clean launch shutdown.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
