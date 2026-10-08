"""Decide where to point the camera and publish the pan COMMAND.

camera_pan_controller_node is hardware-agnostic: it never touches TF or a servo
directly; the sim pan bridge (f1tenth_sim) or the future real driver consume
/camera_pan/command and report the measured angle on /camera_pan/joint_state,
and camera_pan_tf_node turns THAT into TF.

mode (param):
  track_heading  aim at the look-ahead point on the travel arc (aim_law.py)
  fixed          hold fixed_angle_rad (for bring-up / TF testing)
  scan           RESERVED -- logs not-implemented once, falls back to fixed 0.0
                 (the sweep generator is deliberately not implemented yet)

Inputs: local EKF odometry (v, omega_z) on get_odom_topic(), and the commanded
steering on /ackermann_drive (the mux output; delta is only used at low speed,
where omega_z/v is untrustworthy -- see aim_law.curvature). All aim-law tuning
comes from stack_params.yaml so the law stays a single source of truth.
"""
from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_camera_pan.aim_law import aim_pan, AimParams, PanSmoother
from f1tenth_params.param_defaults import get_odom_topic, get_value
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


class CameraPanController(Node):

    def __init__(self):
        super().__init__('camera_pan_controller_node')

        self.mode = self.declare_parameter('mode', 'track_heading').value
        self.fixed_angle = float(self.declare_parameter('fixed_angle_rad', 0.0).value)
        rate_hz = float(self.declare_parameter('publish_rate_hz', 50.0).value)
        # The pivot is per-machine (sim 0.36 vs car 0.12), so it is a launch
        # param set from the same source as camera_pan_tf_node's pivot, NOT a
        # shared stack param. Everything else below is machine-agnostic tuning.
        pivot_x = float(self.declare_parameter('pivot_x_m', 0.12).value)
        pivot_y = float(self.declare_parameter('pivot_y_m', 0.0).value)

        # Aim-law tuning: stack_params.yaml is the single source of truth.
        self.params = AimParams(
            wheelbase_m=float(get_value('swept_clearance_wheelbase_m')),
            max_pan_rad=float(get_value('camera_pan_max_rad')),
            t_lookahead_s=float(get_value('camera_pan_lookahead_s')),
            s_min_m=float(get_value('camera_pan_lookahead_min_m')),
            s_max_m=float(get_value('camera_pan_lookahead_max_m')),
            pivot_x_m=pivot_x,
            pivot_y_m=pivot_y,
            v_curv_min_mps=float(get_value('camera_pan_v_curv_min_mps')),
            v_curv_full_mps=float(get_value('camera_pan_v_curv_full_mps')),
            v_stop_mps=float(get_value('camera_pan_v_stop_mps')),
            reverse_aims_zero=bool(get_value('camera_pan_reverse_aims_zero')),
            track_when_stopped=bool(
                self.declare_parameter('track_when_stopped', False).value),
        )
        self.smoother = PanSmoother(
            deadband_rad=float(get_value('camera_pan_deadband_rad')),
            lp_tau_s=float(get_value('camera_pan_lowpass_tau_s')),
            rate_max_radps=float(get_value('camera_pan_cmd_rate_max_radps')),
        )

        self._v = 0.0
        self._omega = 0.0
        self._delta = 0.0
        self._scan_warned = False

        self.pub = self.create_publisher(Float64, '/camera_pan/command', 10)
        self.create_subscription(Odometry, get_odom_topic(), self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(AckermannDriveStamped, '/ackermann_drive',
                                 self._on_drive, 10)
        self._dt = 1.0 / rate_hz
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'camera_pan_controller up: mode={self.mode}, publishing /camera_pan/command '
            f'at {rate_hz:g} Hz (max +-{self.params.max_pan_rad:.4f} rad).')

    def _on_odom(self, msg: Odometry):
        self._v = msg.twist.twist.linear.x
        self._omega = msg.twist.twist.angular.z

    def _on_drive(self, msg: AckermannDriveStamped):
        self._delta = msg.drive.steering_angle

    def _target(self) -> float:
        if self.mode == 'track_heading':
            return aim_pan(self._v, self._omega, self._delta, self.params)
        if self.mode == 'fixed':
            m = self.params.max_pan_rad
            return min(m, max(-m, self.fixed_angle))
        # scan reserved -> not implemented yet, fall back to fixed 0.0
        if not self._scan_warned:
            self.get_logger().warn(
                'mode "scan" is reserved and not implemented -- holding 0.0. '
                'Implement a sweep generator here when scanning is added.')
            self._scan_warned = True
        return 0.0

    def _tick(self):
        cmd = self.smoother.update(self._target(), self._dt)
        self.pub.publish(Float64(data=cmd))


def main():
    rclpy.init()
    node = CameraPanController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
