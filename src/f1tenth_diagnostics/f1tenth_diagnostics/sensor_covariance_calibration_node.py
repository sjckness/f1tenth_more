"""Measures message-level (Level 1) covariance for the VESC IMU driver and
vesc_to_odom_node_backup, and writes the result directly into the shared
source-tree config both nodes read at startup (f1tenth_bringup/config/vesc.yaml).

Two calibration_mode values:

- 'stationary' (default): run with the car completely stationary and level, for
  sample_duration_sec (default 60s). Before that window starts, two gates run
  in sequence (both new as of the automatic-calibration pass):
    1. Stationary check: confirms the car is actually still via raw ERPM
       telemetry (VescStateStamped.state.speed on state_topic, default
       /sensors/core) held continuously below stationary_erpm_threshold for
       stationary_confirm_sec, or aborts (EXIT_NOT_STATIONARY) after
       stationary_timeout_sec without ever sampling. A single near-zero
       reading is not treated as proof of stillness -- see
       calibration_common.StationaryGate.
    2. First-message gate: once stationary is confirmed, subscribes to
       imu_topic/odom_topic and waits for at least one message on each before
       starting the sample_duration_sec timer (or aborts, EXIT_INSUFFICIENT_
       SAMPLES, after first_message_timeout_sec) -- so the full window counts
       real samples instead of silently losing a couple seconds to driver
       startup latency.
  Once armed, computes the variance of each channel with Welford's online
  algorithm (O(1) memory per channel, no sample buffer):
    * angular_velocity.x/y/z    -> gyro_variance_x/y/z
    * linear_acceleration.x/y/z -> accel_variance_x/y/z
    * twist.twist.linear.x      -> vx_variance
  twist.twist.linear.y is intentionally NOT calibrated: vesc_to_odom_node_backup
  hardcodes it to the constant 0.0 (Ackermann kinematics assumption), never
  derived from a sensor, so its variance is always exactly 0.0 by construction
  -- not a meaningful measurement.

  vx_variance from THIS mode is structurally meaningless while genuinely
  stationary: vesc_to_odom_node_backup's erpm_deadband (see vesc.yaml) forces
  vx to exactly 0.0 at rest, so a stationary sample has zero variance by
  construction, not "no noise found." Use 'light_motion' below for a real
  vx_variance measurement.

- 'light_motion' (new): human-confirmed, constant-velocity straight-line drive
  (mirrors vesc_tuning's steering_calibration_node.py pattern exactly --
  command publishers created and an immediate stop published as soon as the
  node comes up, a blocking input() confirmation gate before any non-zero
  (driving) command is published, continuous command republishing while
  driving, always-stop again on exit/Ctrl-C) at
  light_motion_target_speed (default 0.3 m/s -- comfortably above
  vesc.yaml's erpm_deadband of 500 erpm / speed_to_erpm_gain of ~5499, i.e.
  ~0.09 m/s, so the deadband never masks real motion) for
  light_motion_duration_sec (default 5.0s) after a light_motion_settle_sec
  (default 1.0s) ramp-up window that's driven but NOT sampled (avoids counting
  the accelerate-from-rest transient as steady-state noise). Reuses the exact
  same Welford accumulator and _odom_callback plumbing as the stationary mode
  -- variance is computed around whatever the running mean is, which just
  happens to be nonzero here instead of (structurally) zero. Only vx_variance
  is touched in this mode; gyro_variance_*/accel_variance_* are left exactly
  as the stationary path already correctly measured them -- not recomputed,
  not overwritten. Deliberately has NO stationary check of its own (it
  inherently drives the car on purpose -- a stationary check would be
  actively wrong here).

  MUST be started with `ros2 run` directly, never `ros2 launch` --
  confirm_light_motion_start()'s input() confirmation gate needs a real
  stdin, which `ros2 launch` does not reliably forward to a node process
  (same reason vesc_tuning's steering_calibration_node.py /
  speed_sweep_diagnostic_node.py have no launch file at all). Because of
  this, calibration.launch.py no longer exposes calibration_mode or any
  light_motion_* argument -- it only ever runs 'stationary' (its own node
  default). See confirm_light_motion_start()'s docstring and
  f1tenth_diagnostics/README.md for the exact `ros2 run` invocation.

Unlike gyro_bias_calibration_node's ORIGINAL behavior (read-only, reports
only, never exits on its own), this node has always WRITTEN its result and
exited cleanly with a real exit code -- gyro_bias_calibration_node was
restructured to match this node's pattern (sample-then-exit, then write) as
part of the automatic-calibration pass, rather than the other way around.
This node backs up vesc.yaml (timestamped copy, same directory) and then
patches only the relevant variance keys in place via ruamel.yaml's round-trip
mode, preserving all existing comments/formatting/ordering. Both nodes now
share this logic via calibration_common.write_vesc_yaml (including its
backup-retention pruning) instead of each having their own copy. Requires
`ruamel.yaml` (pip install ruamel.yaml or apt install python3-ruamel.yaml --
now a real rosdep-tracked exec_depend, see package.xml).

Exit codes: see calibration_common.EXIT_* -- 0 success, 1 insufficient
samples, 2 not stationary, 3 ruamel.yaml missing. vesc.launch.py's
calibration_exit_handler branches on these to log a specific reason and
fall back to leaving vesc.yaml untouched (last-known-good) on any nonzero
code, rather than blocking startup.

Path resolution for vesc_yaml_path -- two layers, deliberately kept separate:
  1. This node's OWN default (unchanged, kept as a fallback for anyone who
     runs it directly via `ros2 run` or launches it in a workspace actually
     built with `colcon build --symlink-install`): follows the INSTALLED
     vesc.yaml's symlink back to its source-tree file via
     get_package_share_directory() + realpath(). This only resolves correctly
     when that installed copy really is a symlink; a copy-install silently
     patches the install-space copy instead (the node logs a warning when the
     resolved path doesn't contain "/src/", but still proceeds -- see
     __init__ below).
  2. calibration.launch.py and vesc.launch.py both pass an EXPLICIT
     vesc_yaml_path parameter (overriding #1) computed by
     calibration_common.resolve_source_vesc_yaml_path() -- see that
     function's own docstring for the full reasoning (colcon's install/src
     sibling-directory convention, verified live on this workspace to not use
     --symlink-install).
"""

import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

from f1tenth_diagnostics.calibration_common import (
    EXIT_INSUFFICIENT_SAMPLES,
    EXIT_MISSING_DEPENDENCY,
    EXIT_NOT_STATIONARY,
    EXIT_SUCCESS,
    StationaryGate,
    Welford,
    YAML,
    resolve_source_vesc_yaml_path,
    write_vesc_yaml,
)

# Re-exported for backward compatibility -- calibration.launch.py originally
# imported this name from this module; it now lives in calibration_common
# (shared with gyro_bias_calibration_node) and calibration.launch.py has been
# updated to import it from there directly, but keeping this re-export costs
# nothing and covers any other direct `ros2 run` / script usage.
__all__ = ['resolve_source_vesc_yaml_path', 'SensorCovarianceCalibrationNode', 'main']


class SensorCovarianceCalibrationNode(Node):

    def __init__(self):
        super().__init__('sensor_covariance_calibration_node')

        self.calibration_mode = str(
            self.declare_parameter('calibration_mode', 'stationary').value)
        if self.calibration_mode not in ('stationary', 'light_motion'):
            raise ValueError(
                f'calibration_mode must be "stationary" or "light_motion", '
                f'got {self.calibration_mode!r}')

        self.imu_topic = str(self.declare_parameter('imu_topic', '/sensors/imu/raw').value)
        self.odom_topic = str(self.declare_parameter('odom_topic', '/odom').value)
        self.sample_duration_sec = float(
            self.declare_parameter('sample_duration_sec', 60.0).value)

        # Own fallback default (unchanged) -- see module docstring's "Path
        # resolution" section for why calibration.launch.py/vesc.launch.py now
        # override this with an explicit, reliably-source-anchored value
        # instead of relying on this symlink-following resolution actually
        # finding a symlink.
        default_yaml_path = os.path.realpath(os.path.join(
            get_package_share_directory('f1tenth_bringup'), 'config', 'vesc.yaml'))
        self.vesc_yaml_path = str(
            self.declare_parameter('vesc_yaml_path', default_yaml_path).value)

        # Unconditional -- lets "did the fix land" be confirmed by reading the
        # log at startup without running a full calibration (see chat).
        self.get_logger().info(f'Resolved vesc_yaml_path = "{self.vesc_yaml_path}"')
        if '/src/' not in self.vesc_yaml_path:
            self.get_logger().warning(
                f'Resolved vesc_yaml_path="{self.vesc_yaml_path}" does not look like a '
                'source-tree path (no "/src/" segment) -- the workspace may not be '
                'built with --symlink-install, so this will patch an install-space '
                'copy rather than the real source file. Pass vesc_yaml_path explicitly '
                'if that matters.')

        self.exit_code = EXIT_INSUFFICIENT_SAMPLES
        self.done = False

        self._accumulators = {
            'gyro_variance_x': Welford(),
            'gyro_variance_y': Welford(),
            'gyro_variance_z': Welford(),
            'accel_variance_x': Welford(),
            'accel_variance_y': Welford(),
            'accel_variance_z': Welford(),
            'vx_variance': Welford(),
        }

        if self.calibration_mode == 'stationary':
            # ---- stationary-check knobs (new) ------------------------------
            self.state_topic = str(
                self.declare_parameter('state_topic', '/sensors/core').value)
            # Matches vesc.yaml's erpm_deadband (500.0) -- vesc_driver_node
            # already zeroes any |speed| below that before publishing, so
            # this reuses the same already-established "notionally at rest"
            # threshold for this hardware.
            self.stationary_erpm_threshold = float(
                self.declare_parameter('stationary_erpm_threshold', 500.0).value)
            # Matches this codebase's existing short-settle-window precedent
            # (vesc.launch.py's _TEARDOWN_BUFFER_SEC, battery_voltage_check_
            # node's sample_window_sec -- both 2.0s).
            self.stationary_confirm_sec = float(
                self.declare_parameter('stationary_confirm_sec', 2.0).value)
            self.stationary_timeout_sec = float(
                self.declare_parameter('stationary_timeout_sec', 10.0).value)
            # Fix for blocker #7: bounds the "waiting for first message on
            # imu_topic/odom_topic" phase after stationary is confirmed.
            self.first_message_timeout_sec = float(
                self.declare_parameter('first_message_timeout_sec', 10.0).value)

            self._imu_sub = None
            self._odom_sub = None
            self._timer = None
            self._first_imu_received = False
            self._first_odom_received = False

            self._gate = StationaryGate(
                threshold=self.stationary_erpm_threshold,
                confirm_sec=self.stationary_confirm_sec,
                timeout_sec=self.stationary_timeout_sec,
                now_fn=lambda: self.get_clock().now().nanoseconds / 1e9,
            )
            self._state_sub = self.create_subscription(
                VescStateStamped, self.state_topic, self._state_callback, 50)
            # Independent of message traffic -- catches state_topic never
            # publishing at all (e.g. hardware disconnected), which
            # on_speed_sample()/confirmed would otherwise never notice.
            self._stationary_watchdog = self.create_timer(
                0.5, self._check_stationary_watchdog)

            self.get_logger().info(
                f'Confirming stationary on "{self.state_topic}" (|speed| <= '
                f'{self.stationary_erpm_threshold:.0f} ERPM for '
                f'{self.stationary_confirm_sec:.1f}s continuously, timeout '
                f'{self.stationary_timeout_sec:.1f}s) before sampling covariance on '
                f'"{self.imu_topic}"/"{self.odom_topic}" for '
                f'{self.sample_duration_sec:.1f}s.')
        else:
            # ---- light_motion mode -- unchanged, no stationary check --------
            # gyro/accel are deliberately NOT touched in this mode (see module
            # docstring) -- no _imu_sub at all, only vx via the same
            # _odom_callback/accumulator the stationary path already uses.
            self.light_motion_target_speed = float(
                self.declare_parameter('light_motion_target_speed', 0.3).value)
            self.light_motion_settle_sec = float(
                self.declare_parameter('light_motion_settle_sec', 1.0).value)
            self.light_motion_duration_sec = float(
                self.declare_parameter('light_motion_duration_sec', 5.0).value)
            # Vehicle constants needed to convert target_speed (m/s) into an
            # erpm command -- plain node-level defaults matching vesc.yaml's
            # current values, same pattern vesc_tuning's speed_tuning_node.py/
            # steering_tuning_node.py already use for the same purpose, not
            # new stack-wide params (these aren't calibration workflow knobs,
            # just known vehicle constants).
            self.speed_to_erpm_gain = float(
                self.declare_parameter('speed_to_erpm_gain', 5499.271647286143).value)
            self.speed_to_erpm_offset = float(
                self.declare_parameter('speed_to_erpm_offset', 0.0).value)
            self.steering_center = float(
                self.declare_parameter('steering_center', 0.5304).value)
            self.command_topic = str(
                self.declare_parameter('command_topic', '/commands/motor/speed').value)
            self.servo_topic = str(
                self.declare_parameter('servo_topic', '/commands/servo/position').value)

            self._odom_sub = self.create_subscription(
                Odometry, self.odom_topic, self._odom_callback, 50)

            # Publishers created immediately and an explicit stop published
            # right away -- same convention as vesc_tuning's
            # steering_calibration_node.py (see its __init__): establishes a
            # known-safe baseline as soon as the node is alive, overriding any
            # stale in-flight command a previous process might have left
            # behind. This is NOT the human-confirmation gate -- that's
            # confirm_light_motion_start() below, which is what actually
            # allows a non-zero (driving) command to be published. Nothing
            # here commands real motion.
            self._speed_pub = self.create_publisher(Float64, self.command_topic, 10)
            self._servo_pub = self.create_publisher(Float64, self.servo_topic, 10)
            self._light_motion_rate_hz = 20.0
            self._light_motion_elapsed = 0.0
            self._light_motion_sampling_active = False
            self._light_motion_timer = None

            time.sleep(0.3)  # allow publisher discovery, same as steering_calibration_node.py
            self.publish_stop()

            self.get_logger().info(
                f'light_motion mode configured: target_speed='
                f'{self.light_motion_target_speed:.2f} m/s, settle='
                f'{self.light_motion_settle_sec:.1f}s, sample='
                f'{self.light_motion_duration_sec:.1f}s. Stop command published -- call '
                'confirm_light_motion_start() to begin driving.')

    # ==========================================================================
    # stationary mode -- gates
    # ==========================================================================
    def _state_callback(self, msg: VescStateStamped):
        if self._timer is not None or self._imu_sub is not None:
            return  # already armed/sampling
        self._gate.on_speed_sample(msg.state.speed)
        if self._gate.confirmed:
            self._arm_sampling()

    def _check_stationary_watchdog(self):
        if self._timer is not None or self._imu_sub is not None or self._gate.confirmed:
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
            f'-- either the car is moving, or "{self.state_topic}" never published. '
            'Aborting without sampling or writing vesc.yaml.')
        self.exit_code = EXIT_NOT_STATIONARY
        self.done = True

    def _arm_sampling(self):
        """Stationary confirmed -- subscribe to the sampled topics and wait
        for at least one message on each before starting the fixed-duration
        sample timer (blocker #7 fix: otherwise driver-startup latency
        silently eats into the sampling window instead of counting as real
        sample time)."""
        self._stationary_watchdog.cancel()
        self._state_sub.destroy()
        self.get_logger().info(
            f'Confirmed stationary for {self.stationary_confirm_sec:.1f}s -- waiting for '
            f'first message on "{self.imu_topic}"/"{self.odom_topic}" before starting the '
            f'{self.sample_duration_sec:.1f}s sampling window.')
        self._imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._imu_callback, 50)
        self._odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, 50)
        self._first_message_watchdog = self.create_timer(
            0.5, self._check_first_message_watchdog)
        self._first_message_deadline = (
            self.get_clock().now().nanoseconds / 1e9 + self.first_message_timeout_sec)

    def _maybe_start_timer(self):
        if self._timer is not None:
            return
        if self._first_imu_received and self._first_odom_received:
            self._first_message_watchdog.cancel()
            self.get_logger().info(
                'First message received on both sampled topics -- starting the '
                f'{self.sample_duration_sec:.1f}s sampling window now.')
            self._timer = self.create_timer(self.sample_duration_sec, self._finish)

    def _check_first_message_watchdog(self):
        if self._timer is not None:
            return
        if self._first_imu_received and self._first_odom_received:
            return
        if self.get_clock().now().nanoseconds / 1e9 >= self._first_message_deadline:
            self._on_first_message_timeout()

    def _on_first_message_timeout(self):
        self._first_message_watchdog.cancel()
        missing = [t for t, got in (
            (self.imu_topic, self._first_imu_received),
            (self.odom_topic, self._first_odom_received)) if not got]
        self.get_logger().error(
            f'No message received on {missing} within '
            f'{self.first_message_timeout_sec:.1f}s of confirming stationary -- aborting '
            'without sampling or writing vesc.yaml.')
        self._imu_sub.destroy()
        self._odom_sub.destroy()
        self.exit_code = EXIT_INSUFFICIENT_SAMPLES
        self.done = True

    def _imu_callback(self, msg: Imu):
        self._accumulators['gyro_variance_x'].update(msg.angular_velocity.x)
        self._accumulators['gyro_variance_y'].update(msg.angular_velocity.y)
        self._accumulators['gyro_variance_z'].update(msg.angular_velocity.z)
        self._accumulators['accel_variance_x'].update(msg.linear_acceleration.x)
        self._accumulators['accel_variance_y'].update(msg.linear_acceleration.y)
        self._accumulators['accel_variance_z'].update(msg.linear_acceleration.z)
        if not self._first_imu_received:
            self._first_imu_received = True
            self._maybe_start_timer()

    def _odom_callback(self, msg: Odometry):
        if self.calibration_mode == 'stationary':
            self._accumulators['vx_variance'].update(msg.twist.twist.linear.x)
            if not self._first_odom_received:
                self._first_odom_received = True
                self._maybe_start_timer()
        elif self._light_motion_sampling_active:
            self._accumulators['vx_variance'].update(msg.twist.twist.linear.x)

    def _finish(self):
        self._timer.cancel()
        self._imu_sub.destroy()
        self._odom_sub.destroy()

        insufficient = [k for k, acc in self._accumulators.items() if acc.n < 2]
        if insufficient:
            self.get_logger().error(
                f'Not enough samples on: {insufficient} (need >=2 each) -- is '
                f'"{self.imu_topic}" / "{self.odom_topic}" publishing? Aborting without '
                'writing vesc.yaml.')
            self.exit_code = EXIT_INSUFFICIENT_SAMPLES
            self.done = True
            return

        self.get_logger().info('Calibration complete -- computed variance per channel:')
        results = {}
        for key, acc in self._accumulators.items():
            variance = acc.variance()
            results[key] = variance
            self.get_logger().info(f'  {key} = {variance:.8f}  (n={acc.n})')

        if YAML is None:
            self.get_logger().error(
                'ruamel.yaml is not installed (pip install ruamel.yaml) -- cannot write '
                f'{self.vesc_yaml_path}. Apply the values above manually.')
            self.exit_code = EXIT_MISSING_DEPENDENCY
            self.done = True
            return

        write_vesc_yaml(self.vesc_yaml_path, results, self.get_logger())
        self.exit_code = EXIT_SUCCESS
        self.done = True

    # ==========================================================================
    # light_motion mode
    # ==========================================================================
    def confirm_light_motion_start(self) -> bool:
        """Blocking human confirmation gate before ANY non-zero (driving)
        command is published -- mirrors vesc_tuning's
        steering_calibration_node.py pattern (see that file's own __init__
        and its per-mode confirmation prompts): the command publishers
        already exist and have already published a stop (see __init__ above,
        same as that file), so what this method actually gates is only the
        driving timer that would command real motion. Called once, from
        main(), before the spin loop begins.

        Genuinely stdin-based (a blocking builtin input() call) -- there is
        no service/topic alternative. `ros2 launch` does not reliably
        forward stdin to a launched node's process (same limitation
        documented on vesc_tuning's steering_calibration_node.py /
        speed_sweep_diagnostic_node.py, which have no launch file at all for
        exactly this reason); calibration.launch.py no longer exposes
        calibration_mode/light_motion_* as of this fix -- light_motion mode
        must be started with `ros2 run` directly (see the isatty() guard
        below and f1tenth_diagnostics/README.md).

        Returns True if confirmed (driving has now started), False if
        aborted -- either by the user, or by the isatty() guard below when
        no real stdin is available. Either way, no non-zero command was ever
        published and it's safe to exit immediately.
        """
        if not sys.stdin.isatty():
            self.get_logger().error(
                'light_motion mode needs a real interactive stdin to confirm before '
                'driving, but stdin here is not a TTY -- this almost always means the '
                'node was started via `ros2 launch`, which does not reliably forward '
                'stdin to the node process (this would otherwise hang forever on the '
                'input() prompt below with no visible cause). Re-run directly instead: '
                '`ros2 run f1tenth_diagnostics sensor_covariance_calibration_node '
                '--ros-args -p calibration_mode:=light_motion -p vesc_yaml_path:=<path> '
                '[-p light_motion_target_speed:=0.3 -p light_motion_settle_sec:=1.0 '
                '-p light_motion_duration_sec:=5.0]` -- see f1tenth_diagnostics/README.md. '
                'Aborting now -- no motion was commanded.')
            return False

        total = self.light_motion_settle_sec + self.light_motion_duration_sec
        print('=' * 70)
        print('LIGHT-MOTION vx_variance CALIBRATION')
        print(f'  Will drive FORWARD in a straight line at '
              f'{self.light_motion_target_speed:.2f} m/s for {total:.1f}s total '
              f'({self.light_motion_settle_sec:.1f}s settle, not sampled, then '
              f'{self.light_motion_duration_sec:.1f}s sampled).')
        print(f'  Estimated distance: ~{self.light_motion_target_speed * total:.1f} m.')
        print('  Ensure the car has that much clear, flat space ahead.')
        print('=' * 70)
        raw = input('Press ENTER to begin, or "q" to abort: ').strip().lower()
        if raw == 'q':
            self.get_logger().warning(
                'light_motion calibration aborted by user before any motion was commanded.')
            return False

        self._light_motion_timer = self.create_timer(
            1.0 / self._light_motion_rate_hz, self._light_motion_tick)
        self.get_logger().info('light_motion calibration started -- driving.')
        return True

    def publish_drive(self, erpm: float, servo: float):
        self._speed_pub.publish(Float64(data=float(erpm)))
        self._servo_pub.publish(Float64(data=float(servo)))

    def publish_stop(self):
        self.publish_drive(0.0, self.steering_center)

    def _light_motion_tick(self):
        erpm = self.speed_to_erpm_gain * self.light_motion_target_speed + self.speed_to_erpm_offset
        self.publish_drive(erpm, self.steering_center)

        self._light_motion_elapsed += 1.0 / self._light_motion_rate_hz
        self._light_motion_sampling_active = (
            self._light_motion_elapsed >= self.light_motion_settle_sec)

        total = self.light_motion_settle_sec + self.light_motion_duration_sec
        if self._light_motion_elapsed >= total:
            self._light_motion_sampling_active = False
            self._light_motion_timer.cancel()
            self._odom_sub.destroy()
            self.publish_stop()
            self._finish_light_motion()

    def _finish_light_motion(self):
        acc = self._accumulators['vx_variance']
        if acc.n < 2:
            self.get_logger().error(
                f'Not enough vx samples collected during the sampled window '
                f'(n={acc.n}, need >=2) -- is "{self.odom_topic}" publishing? '
                'Aborting without writing vesc.yaml.')
            self.exit_code = EXIT_INSUFFICIENT_SAMPLES
            self.done = True
            return

        variance = acc.variance()
        self.get_logger().info(
            f'light_motion calibration complete: vx_variance={variance:.8f} '
            f'(mean vx={acc.mean:+.4f} m/s, target was '
            f'{self.light_motion_target_speed:.2f} m/s, n={acc.n} samples)')

        if YAML is None:
            self.get_logger().error(
                'ruamel.yaml is not installed (pip install ruamel.yaml) -- cannot write '
                f'{self.vesc_yaml_path}. Apply vx_variance={variance:.8f} manually.')
            self.exit_code = EXIT_MISSING_DEPENDENCY
            self.done = True
            return

        write_vesc_yaml(self.vesc_yaml_path, {'vx_variance': variance}, self.get_logger())
        self.exit_code = EXIT_SUCCESS
        self.done = True


def main():
    rclpy.init()
    node = SensorCovarianceCalibrationNode()

    if node.calibration_mode == 'light_motion':
        # Blocking human confirmation -- __init__ already created the command
        # publishers and published one stop (safe baseline), but no non-zero
        # (driving) command is published before this returns True.
        if not node.confirm_light_motion_start():
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            sys.exit(EXIT_INSUFFICIENT_SAMPLES)

    try:
        # NOT rclpy.spin(node): calling rclpy.shutdown() from inside a callback
        # running under the executor deadlocks (executor.shutdown() waits for that
        # same callback to finish -- confirmed live). Callbacks only set node.done;
        # shutdown happens here, in the main thread, once the loop notices it.
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if node.calibration_mode == 'light_motion':
            # Always-leave-stopped safety net (Ctrl-C, exception, or normal
            # completion all land here) -- same pattern as vesc_tuning's
            # steering_calibration_node.py.
            node.publish_stop()
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
