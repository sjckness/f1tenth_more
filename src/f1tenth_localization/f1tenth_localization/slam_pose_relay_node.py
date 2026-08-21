"""Relay slam_toolbox's raw /slam/pose onto /slam/pose_calibrated, stamping
in a CALIBRATED covariance -- the real, functional mechanism the dual-EKF
pass's own ekf_global.yaml pose0 calibration actually takes effect through.

Why this exists at all (found live while building the dual-EKF pass, not
assumed going in): robot_localization has NO static per-sensor covariance-
override parameter (checked against /opt/ros/humble/share/robot_localization/
params/ekf.yaml, the package's own reference config) -- a pose0 source's
covariance always comes from whatever the MESSAGE ITSELF carries
(PoseWithCovarianceStamped.pose.covariance), never from a yaml param. slam_
toolbox's own /slam/pose covariance is whatever it happens to publish (as of
this pass, /slam/pose does not publish anything at all -- see f1tenth_
diagnostics/slam_pose_covariance_calibration_node.py's own docstring for that
still-open, separate issue), not something this workspace controls or has
validated. Rather than trust that value blindly (or lack thereof), this node
sits between slam_toolbox and the global EKF (f1tenth_bringup/config/
ekf_global.yaml's own ekf_global_filter_node), OVERWRITING the covariance
diagonal with values slam_pose_covariance_calibration_node measures from
real data and writes into THIS node's own params block (ekf_global.yaml's
slam_pose_relay_node section, same file) -- the position/orientation payload
itself passes through unmodified.

Only x/y/yaw (indices 0, 7, 35 of the row-major 6x6 covariance matrix -- x,
y, yaw=rot_z) are stamped with the calibrated values; every other diagonal
entry (z, roll, pitch cross terms) is set to a large, clearly-not-trusted
placeholder (1e6) -- inert regardless, since ekf_global_filter_node's own
pose0_config excludes those axes from fusion entirely (2D vehicle, see that
file's own comment), but set explicitly rather than left at whatever
slam_toolbox happened to publish, for hygiene. Off-diagonal terms are zeroed
(no cross-axis correlation assumed) -- a REASONED SIMPLIFICATION, not a
measurement in itself; if slam_toolbox's own scan-matching produces
meaningfully correlated x/y/yaw errors, this discards that structure. Flagged
deliberately, same discipline this codebase's other reasoned-starting-point
parameters use elsewhere.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped

_UNUSED_AXIS_VARIANCE = 1e6  # z, roll, pitch -- see module docstring


class SlamPoseRelayNode(Node):
    def __init__(self):
        super().__init__('slam_pose_relay_node')

        self.declare_parameter('input_topic', '/slam/pose')
        self.declare_parameter('output_topic', '/slam/pose_calibrated')
        self.declare_parameter('pose_variance_x', 0.1)
        self.declare_parameter('pose_variance_y', 0.1)
        self.declare_parameter('pose_variance_yaw', 0.05)

        p = self.get_parameter
        self.pose_variance_x = float(p('pose_variance_x').value)
        self.pose_variance_y = float(p('pose_variance_y').value)
        self.pose_variance_yaw = float(p('pose_variance_yaw').value)

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, p('input_topic').value, self._pose_cb, 10)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, p('output_topic').value, 10)

        self.get_logger().info(
            f'slam_pose_relay_node started: "{p("input_topic").value}" -> '
            f'"{p("output_topic").value}", covariance stamped with '
            f'var_x={self.pose_variance_x:.6f} var_y={self.pose_variance_y:.6f} '
            f'var_yaw={self.pose_variance_yaw:.6f}')

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose.pose = msg.pose.pose

        # Row-major 6x6, state order [x, y, z, roll, pitch, yaw] -- diagonal
        # indices are row*6 + row: x=0, y=7, z=14, roll=21, pitch=28, yaw=35.
        cov = [0.0] * 36
        cov[0] = self.pose_variance_x
        cov[7] = self.pose_variance_y
        cov[14] = _UNUSED_AXIS_VARIANCE
        cov[21] = _UNUSED_AXIS_VARIANCE
        cov[28] = _UNUSED_AXIS_VARIANCE
        cov[35] = self.pose_variance_yaw
        out.pose.covariance = cov

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SlamPoseRelayNode()
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
