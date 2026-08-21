"""Measures message-level (Level 1) covariance for slam_toolbox's own
/slam/pose (x, y, yaw), then writes the result into f1tenth_bringup/config/
ekf_global.yaml's slam_pose_relay_node section and exits -- the real,
functional place that value takes effect (see slam_pose_relay_node.py's own
docstring for why: robot_localization has no static per-sensor covariance-
override parameter, so a small relay node stamps this measured covariance
onto slam_toolbox's raw pose before the global EKF ever sees it).

Mirrors gyro_bias_calibration_node.py/sensor_covariance_calibration_node.py's
own 'stationary' mode pattern exactly (same package, same StationaryGate,
same Welford accumulator, same auto-write-then-exit discipline, same real-
src-not-install-space write-path fix already established via calibration_
common.resolve_source_config_path) -- stationary mode ONLY in this pass; see
STUB comment near the bottom of this file for the documented, unimplemented
in-motion mode.

Two gates run in sequence before sampling starts, same shape as sensor_
covariance_calibration_node.py's own stationary mode:
  1. Stationary check: confirms the car is actually still via raw ERPM
     telemetry (VescStateStamped.state.speed on state_topic, default
     /sensors/core), held continuously below stationary_erpm_threshold for
     stationary_confirm_sec, or aborts (EXIT_NOT_STATIONARY) after
     stationary_timeout_sec without ever sampling.
  2. First-message gate: once stationary is confirmed, subscribes to
     pose_topic and waits for at least one message before starting the
     sample_duration_sec timer (or aborts, EXIT_INSUFFICIENT_SAMPLES, after
     first_message_timeout_sec) -- so the full window counts real samples,
     and so a /slam/pose that never publishes at all is detected here,
     specifically, rather than silently sampling nothing for the whole
     window and only noticing at the very end.

THE CURRENT REALITY THIS MUST HANDLE HONESTLY (explicit per this pass's own
task): /slam/pose does not publish at all as of this writing (a separate,
already-flagged, still-open issue -- see the SLAM/costmap investigation this
follows). Both VESC and the lidar are ALSO physically disconnected as of this
same pass, so even the STATIONARY gate itself has no state_topic data to work
with right now -- meaning a live run today would fail at EXIT_NOT_STATIONARY
(state_topic never publishes) before ever reaching the pose_topic gate at
all, not specifically at "pose_topic never publishes" -- both are honest,
correctly-distinguished, zero-write failure paths (see calibration_common.
EXIT_REASONS), neither hangs, neither writes garbage. Once hardware is
reconnected, if VESC alone comes back but /slam/pose still doesn't, THAT
specific path (stationary confirmed, then pose_topic times out at
EXIT_INSUFFICIENT_SAMPLES) is what would actually run.

Exit codes: see calibration_common.EXIT_* -- 0 success, 1 insufficient
samples, 2 not stationary, 3 ruamel.yaml missing.
"""

import math
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from vesc_msgs.msg import VescStateStamped

from f1tenth_diagnostics.calibration_common import (
    EXIT_INSUFFICIENT_SAMPLES,
    EXIT_MISSING_DEPENDENCY,
    EXIT_NOT_STATIONARY,
    EXIT_SUCCESS,
    StationaryGate,
    Welford,
    YAML,
    resolve_source_config_path,
    write_yaml_config,
)

# The one fixed section this node ever writes into -- see module docstring
# and slam_pose_relay_node.py's own docstring for why THIS is the real,
# functional destination (not a per-key routing table like vesc.yaml's own
# _KEY_SECTIONS -- there is only ever one section here).
_TARGET_SECTION = 'slam_pose_relay_node'


def _yaw_from_quaternion(q) -> float:
    """Same planar-yaw-from-quaternion formula used throughout this codebase
    (MPC_corr.py's own quaternion_to_yaw(), check_stop_condition.py's own
    _quaternion_to_yaw(), f1tenth_costmap/semantic_layer.py's own pose_to_
    xytheta()) -- reimplemented inline here rather than imported, matching
    this codebase's own established precedent (MPC_corr.py's inline cpu_
    affinity copy, behavior_executor_node.py's own inline copy) for a single-
    consumer package pulling in a cross-package dependency for a few lines of
    math; f1tenth_diagnostics has no existing dependency on f1tenth_costmap
    or mpc_controller and this isn't worth adding one for."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class SlamPoseCovarianceCalibrationNode(Node):

    def __init__(self, **kwargs):
        # **kwargs forwarded to rclpy.node.Node (e.g. parameter_overrides=)
        # -- lets tests construct this node with specific parameter values
        # directly (no live launch/params-file needed), same as any other
        # rclpy Node subclass; main() below still just calls this with no
        # arguments, unaffected.
        super().__init__('slam_pose_covariance_calibration_node', **kwargs)

        # Raw /slam/pose -- upstream of slam_pose_relay_node's own covariance
        # stamping, deliberately: this node needs to measure the ACTUAL,
        # unmodified noise slam_toolbox itself produces, not whatever
        # placeholder/previously-calibrated value the relay is currently
        # stamping on top of it.
        self.pose_topic = str(self.declare_parameter('pose_topic', '/slam/pose').value)
        self.sample_duration_sec = float(
            self.declare_parameter('sample_duration_sec', 60.0).value)
        self.min_samples = int(self.declare_parameter('min_samples', 30).value)

        # ---- stationary-check knobs -- same defaults/reasoning as sensor_
        # covariance_calibration_node.py's own stationary mode (see that
        # file's own comments for the erpm_deadband/settle-window precedent
        # this reuses, not re-derived here).
        self.state_topic = str(self.declare_parameter('state_topic', '/sensors/core').value)
        self.stationary_erpm_threshold = float(
            self.declare_parameter('stationary_erpm_threshold', 500.0).value)
        self.stationary_confirm_sec = float(
            self.declare_parameter('stationary_confirm_sec', 2.0).value)
        self.stationary_timeout_sec = float(
            self.declare_parameter('stationary_timeout_sec', 10.0).value)
        self.first_message_timeout_sec = float(
            self.declare_parameter('first_message_timeout_sec', 10.0).value)

        # Own fallback default (unchanged pattern from gyro_bias_calibration_
        # node.py/sensor_covariance_calibration_node.py): only resolves
        # correctly under --symlink-install. calibration.launch.py overrides
        # this explicitly via calibration_common.resolve_source_config_path()
        # -- see that function's own docstring.
        default_yaml_path = resolve_source_config_path(
            'f1tenth_bringup', 'config', 'ekf_global.yaml')
        self.ekf_global_yaml_path = str(
            self.declare_parameter('ekf_global_yaml_path', default_yaml_path).value)

        self.exit_code = EXIT_INSUFFICIENT_SAMPLES
        self.done = False

        self._accumulators = {
            'pose_variance_x': Welford(),
            'pose_variance_y': Welford(),
            'pose_variance_yaw': Welford(),
        }

        self._pose_sub = None
        self._timer = None
        self._first_pose_received = False

        # ---- Phase 1: confirm stationary before sampling ----------------------
        self._gate = StationaryGate(
            threshold=self.stationary_erpm_threshold,
            confirm_sec=self.stationary_confirm_sec,
            timeout_sec=self.stationary_timeout_sec,
            now_fn=lambda: self.get_clock().now().nanoseconds / 1e9,
        )
        self._state_sub = self.create_subscription(
            VescStateStamped, self.state_topic, self._state_callback, 50)
        self._stationary_watchdog = self.create_timer(0.5, self._check_stationary_watchdog)

        self.get_logger().info(
            f'Confirming stationary on "{self.state_topic}" (|speed| <= '
            f'{self.stationary_erpm_threshold:.0f} ERPM for '
            f'{self.stationary_confirm_sec:.1f}s continuously, timeout '
            f'{self.stationary_timeout_sec:.1f}s) before sampling covariance on '
            f'"{self.pose_topic}" for {self.sample_duration_sec:.1f}s.')

    # ==========================================================================
    # gates
    # ==========================================================================
    def _state_callback(self, msg: VescStateStamped):
        if self._timer is not None or self._pose_sub is not None:
            return  # already armed/sampling
        self._gate.on_speed_sample(msg.state.speed)
        if self._gate.confirmed:
            self._arm_sampling()

    def _check_stationary_watchdog(self):
        if self._timer is not None or self._pose_sub is not None or self._gate.confirmed:
            return
        if self._gate.timed_out():
            self._on_not_stationary()

    def _on_not_stationary(self):
        self._stationary_watchdog.cancel()
        self._state_sub.destroy()
        self.get_logger().error(
            f'Stationary check failed: car was not confirmed stationary (|speed| <= '
            f'{self.stationary_erpm_threshold:.0f} ERPM continuously for '
            f'{self.stationary_confirm_sec:.1f}s) within {self.stationary_timeout_sec:.1f}s '
            f'-- either the car is moving, or "{self.state_topic}" never published '
            '(e.g. VESC disconnected). Aborting without sampling or writing '
            f'{self.ekf_global_yaml_path}.')
        self.exit_code = EXIT_NOT_STATIONARY
        self.done = True

    def _arm_sampling(self):
        """Stationary confirmed -- subscribe to pose_topic and wait for at
        least one message before starting the fixed-duration sample timer
        (same "don't silently eat driver-startup latency out of the real
        sampling window" fix sensor_covariance_calibration_node.py's own
        _arm_sampling already established)."""
        self._stationary_watchdog.cancel()
        self._state_sub.destroy()
        self.get_logger().info(
            f'Confirmed stationary for {self.stationary_confirm_sec:.1f}s -- waiting for '
            f'first message on "{self.pose_topic}" before starting the '
            f'{self.sample_duration_sec:.1f}s sampling window.')
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self._pose_callback, 50)
        self._first_message_watchdog = self.create_timer(
            0.5, self._check_first_message_watchdog)
        self._first_message_deadline = (
            self.get_clock().now().nanoseconds / 1e9 + self.first_message_timeout_sec)

    def _check_first_message_watchdog(self):
        if self._timer is not None:
            return
        if self._first_pose_received:
            return
        if self.get_clock().now().nanoseconds / 1e9 >= self._first_message_deadline:
            self._on_first_message_timeout()

    def _on_first_message_timeout(self):
        self._first_message_watchdog.cancel()
        self.get_logger().error(
            f'No message received on "{self.pose_topic}" within '
            f'{self.first_message_timeout_sec:.1f}s of confirming stationary -- is '
            'slam_toolbox running and actually publishing a pose (see this workspace\'s '
            'own known-open issue: /slam/pose currently does not publish at all)? '
            f'Aborting without sampling or writing {self.ekf_global_yaml_path}.')
        self._pose_sub.destroy()
        self.exit_code = EXIT_INSUFFICIENT_SAMPLES
        self.done = True

    # ==========================================================================
    # sampling
    # ==========================================================================
    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        pos = msg.pose.pose.position
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._accumulators['pose_variance_x'].update(pos.x)
        self._accumulators['pose_variance_y'].update(pos.y)
        self._accumulators['pose_variance_yaw'].update(yaw)

        if not self._first_pose_received:
            self._first_pose_received = True
            self._first_message_watchdog.cancel()
            self.get_logger().info(
                f'First message received on "{self.pose_topic}" -- starting the '
                f'{self.sample_duration_sec:.1f}s sampling window now.')
            self._timer = self.create_timer(self.sample_duration_sec, self._finish)

    def _finish(self):
        self._timer.cancel()
        self._pose_sub.destroy()

        insufficient = [k for k, acc in self._accumulators.items() if acc.n < 2]
        if insufficient:
            self.get_logger().error(
                f'Not enough samples on: {insufficient} (need >=2 each, have '
                f'n={self._accumulators["pose_variance_x"].n}) -- is "{self.pose_topic}" '
                f'still publishing? Aborting without writing {self.ekf_global_yaml_path}.')
            self.exit_code = EXIT_INSUFFICIENT_SAMPLES
            self.done = True
            return

        count = self._accumulators['pose_variance_x'].n
        if count < self.min_samples:
            self.get_logger().warning(
                f'Only {count} samples received (min_samples={self.min_samples}) -- '
                'result may be noisy. Consider a longer sample_duration_sec.')

        self.get_logger().info(f'slam pose covariance over {count} samples:')
        results = {}
        for key, acc in self._accumulators.items():
            variance = acc.variance()
            results[key] = variance
            self.get_logger().info(
                f'  {key} = {variance:.8f}  (mean={acc.mean:+.6f}, n={acc.n})')

        if YAML is None:
            self.get_logger().error(
                'ruamel.yaml is not installed (pip install ruamel.yaml or apt install '
                f'python3-ruamel.yaml) -- cannot write {self.ekf_global_yaml_path}. Apply '
                'the values above manually to that file\'s own slam_pose_relay_node '
                'section.')
            self.exit_code = EXIT_MISSING_DEPENDENCY
            self.done = True
            return

        write_yaml_config(
            self.ekf_global_yaml_path, _TARGET_SECTION, results, self.get_logger())
        self.exit_code = EXIT_SUCCESS
        self.done = True

    # ==========================================================================
    # STUB: future in-motion mode -- NOT implemented this pass (see this
    # package's own task scope: "leave a documented stub, don't implement
    # it"). Stationary-mode covariance (above) measures slam_toolbox's own
    # scan-matching NOISE around a fixed true pose -- it says nothing about
    # how that noise behaves under real motion (faster scan-to-scan
    # decorrelation, motion-blur-adjacent effects on the lidar return itself,
    # etc.), which a real deployment would eventually want a second,
    # independent measurement for -- same "stationary first, in-motion later,
    # explicitly deferred" shape as sensor_covariance_calibration_node.py's
    # own light_motion mode relative to its own stationary mode. Whoever
    # implements this: it would need its own confirm_light_motion_start()-
    # style human-confirmation gate (see sensor_covariance_calibration_node.py
    # for that exact pattern) and can't share the StationaryGate used above
    # (same reasoning that file's own light_motion mode already documents --
    # it inherently drives the car on purpose, so a stationary check would be
    # actively wrong for it).
    # ==========================================================================


def main():
    rclpy.init()
    node = SlamPoseCovarianceCalibrationNode()
    try:
        # NOT rclpy.spin(node): same reasoning as gyro_bias_calibration_node.py/
        # sensor_covariance_calibration_node.py -- calling rclpy.shutdown() from
        # inside a callback running under the executor deadlocks. Callbacks only
        # set node.done; shutdown happens here, in the main thread, once the loop
        # notices it.
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
