"""Measures the VESC IMU's gyro (angular_velocity) static bias, then writes
the result into vesc.yaml and exits.

Run with the car completely stationary and level -- as of the automatic-
calibration pass, this is enforced, not just requested: before sampling
starts, the node confirms the car is actually stationary via raw ERPM
telemetry (VescStateStamped.state.speed on state_topic, default
/sensors/core) held continuously below stationary_erpm_threshold for
stationary_confirm_sec, or aborts (distinct exit code, see
calibration_common.EXIT_NOT_STATIONARY) after stationary_timeout_sec without
ever sampling. This means the node now needs a live VescStateStamped
publisher (normally vesc_driver_node) even to begin, not just the IMU it
actually samples -- a real behavior change from before this pass, when it
only needed imu_topic.

Once stationary is confirmed, subscribes to the raw IMU topic (default
/sensors/imu/raw, matching f1tenth_bringup/config/ekf.yaml's imu0 source),
averages angular_velocity.x/y/z over a fixed sampling window
(sample_duration_sec), logs the mean and standard deviation per axis, then
writes a CORRECTED gyro_bias_z into vesc.yaml (same backup-then-patch
mechanism sensor_covariance_calibration_node already uses, now shared via
calibration_common.write_vesc_yaml) and exits cleanly with a real exit code.

"Corrected", not "the measured z mean directly" -- see _finish()'s own
comment for a real bug found and fixed live: /sensors/imu/raw already has
whatever gyro_bias_z the driver was started with subtracted out
(vesc_driver.cpp), so the measured mean is a RESIDUAL relative to that, not
the raw hardware bias -- this now reads the old value back out of
vesc_yaml_path (calibration_common.read_vesc_yaml_value) and ADDS the
residual to it before writing, rather than overwriting the old value with
just the residual (which doesn't converge -- it compounds across repeated
runs, confirmed live).

Before this pass this node never exited on its own (rclpy.spin(node),
"Done -- Ctrl+C to exit.") and never wrote anything (manual-reference-only,
requiring a human to hand-edit vesc.yaml's gyro_bias_z after reading the
logged value) -- both fixed here, mirroring sensor_covariance_calibration_
node's sample-then-exit-and-write pattern so this node can be sequenced the
same way (see f1tenth_hardware/launch/vesc.launch.py's calibration:=true
path, which now runs both nodes).

See f1tenth_diagnostics/README.md for expected output and where the value
gets applied.

calibration-safety-gates pass (Phase A -- see this pass's own report for the
investigation this closes: a 2026-08-19 11:06:58 write landed gyro_bias_z
0.0523 rad/s off from truth, with none of the below in place to catch it).
Three independent gates added, all refusing to write rather than degrading
silently:
  - min_samples is now a hard floor (EXIT_INSUFFICIENT_SAMPLES) -- previously
    only a warning; a below-threshold run still computed a mean and wrote it.
  - The pre-sampling stationary check (above) now stays live through the
    WHOLE sampling window, not just before it starts -- any ERPM motion
    during sampling aborts immediately (EXIT_MOTION_DURING_SAMPLING), as does
    an accelerometer-deviation check on the same IMU stream being sampled
    (catches disturbances ERPM alone can't see -- see calibration_common.
    exceeds_vibration_threshold's own docstring, including why its gravity
    reference is 1.0, not 9.81, on this hardware).
  - The corrected value is sanity-checked against both an absolute-magnitude
    bound and a delta-from-old-value bound before ever being written
    (EXIT_SANITY_VIOLATION if either trips) -- see calibration_common.
    check_sanity_bound's own docstring; the actual bad write above would have
    tripped both at their current defaults.
None of this changes the sampling/statistics method itself (still a plain
mean over the window) -- that's Phase B, landing separately on top of this
once Phase A is verified live.
"""

import os
import statistics
import sys

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from vesc_msgs.msg import VescStateStamped

from f1tenth_diagnostics.calibration_common import (
    EXIT_INSUFFICIENT_SAMPLES,
    EXIT_MISSING_DEPENDENCY,
    EXIT_MOTION_DURING_SAMPLING,
    EXIT_SANITY_VIOLATION,
    EXIT_SUCCESS,
    StationaryGate,
    YAML,
    check_sanity_bound,
    exceeds_motion_threshold,
    exceeds_vibration_threshold,
    read_vesc_yaml_value,
    write_vesc_yaml,
)
from f1tenth_diagnostics.calibration_common import EXIT_NOT_STATIONARY as _EXIT_NOT_STATIONARY


class GyroBiasCalibrationNode(Node):

    def __init__(self, **kwargs):
        # **kwargs (calibration-safety-gates pass, added purely for
        # testability): lets a test construct this node directly with
        # parameter_overrides=[...], same pattern
        # slam_pose_covariance_calibration_node.py's own __init__ already
        # uses -- see test_gyro_bias_calibration_node.py's own
        # _construct_with_params() helper. No behavior change for the real
        # launch-file callers (Node(...parameters=[{...}]) never passes
        # __init__ kwargs at all).
        super().__init__('gyro_bias_calibration_node', **kwargs)

        self.imu_topic = str(self.declare_parameter('imu_topic', '/sensors/imu/raw').value)
        self.sample_duration_sec = float(
            self.declare_parameter('sample_duration_sec', 30.0).value)
        self.min_samples = int(self.declare_parameter('min_samples', 300).value)

        # ---- stationary-check knobs (new) -- see module docstring ------------
        self.state_topic = str(self.declare_parameter('state_topic', '/sensors/core').value)
        # Matches vesc.yaml's erpm_deadband (500.0): vesc_driver_node already
        # zeroes any |speed| below that before publishing, so this reuses the
        # same already-established "notionally at rest" threshold for this
        # hardware rather than inventing a second one.
        self.stationary_erpm_threshold = float(
            self.declare_parameter('stationary_erpm_threshold', 500.0).value)
        # Matches this codebase's existing short-settle-window precedent
        # (vesc.launch.py's _TEARDOWN_BUFFER_SEC, battery_voltage_check_node's
        # sample_window_sec -- both 2.0s).
        self.stationary_confirm_sec = float(
            self.declare_parameter('stationary_confirm_sec', 2.0).value)
        # Generous margin above the real boot-time driver-handshake latency
        # observed in practice (~2-4s) without hanging startup indefinitely if
        # the car genuinely never settles (or state_topic never publishes).
        self.stationary_timeout_sec = float(
            self.declare_parameter('stationary_timeout_sec', 10.0).value)

        # ---- calibration-safety-gates knobs (Phase A) -------------------------
        # See calibration_common.check_sanity_bound()'s own docstring for the
        # full reasoning and the actual bad write (2026-08-19 11:06:58) that
        # motivated both bounds -- it would have tripped both at these defaults.
        self.gyro_bias_absolute_bound = float(
            self.declare_parameter('gyro_bias_absolute_bound', 0.05).value)
        self.gyro_bias_delta_bound = float(
            self.declare_parameter('gyro_bias_delta_bound', 0.02).value)
        # See calibration_common.exceeds_vibration_threshold()'s own docstring
        # for the gravity=1.0 (not 9.81) reasoning. 0.3 default: backed by a
        # live 20s/965-sample stationary probe on this exact hardware (this
        # pass's own verification) -- |ax|+|ay|+|az-1.0| measured mean=0.084,
        # stdev=0.010, max=0.153 at rest, so 0.3 is ~2x the observed resting
        # max / ~20 stdev above the resting mean: comfortably clear of normal
        # sensor noise while still well below what a real bump/vibration event
        # would plausibly add.
        self.gyro_bias_vibration_threshold = float(
            self.declare_parameter('gyro_bias_vibration_threshold', 0.3).value)

        # Own fallback default (unchanged pattern from sensor_covariance_
        # calibration_node): only resolves correctly under --symlink-install.
        # calibration.launch.py overrides this explicitly via
        # calibration_common.resolve_source_vesc_yaml_path(); vesc.launch.py's
        # calibration:=true path does the same.
        default_yaml_path = os.path.realpath(os.path.join(
            get_package_share_directory('f1tenth_bringup'), 'config', 'vesc.yaml'))
        self.vesc_yaml_path = str(
            self.declare_parameter('vesc_yaml_path', default_yaml_path).value)

        self.exit_code = EXIT_INSUFFICIENT_SAMPLES
        self.done = False

        self._samples_x = []
        self._samples_y = []
        self._samples_z = []
        self._sub = None
        self._timer = None
        # Set once sampling actually starts (_start_sampling) -- used both to
        # log how far into the window a Phase A motion/vibration abort landed,
        # and (implicitly) as the "are we sampling yet" flag _state_callback
        # already used self._sub is not None for; kept separate rather than
        # reusing self._sub for readability at the call sites below.
        self._sampling_start_time = None

        # ---- Phase 1: confirm stationary before sampling ----------------------
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
        self._stationary_watchdog = self.create_timer(0.5, self._check_stationary_watchdog)

        self.get_logger().info(
            f'Confirming stationary on "{self.state_topic}" (|speed| <= '
            f'{self.stationary_erpm_threshold:.0f} ERPM for '
            f'{self.stationary_confirm_sec:.1f}s continuously, timeout '
            f'{self.stationary_timeout_sec:.1f}s) before sampling gyro bias.')

    def _state_callback(self, msg: VescStateStamped):
        if self._sub is not None:
            # calibration-safety-gates pass (Phase A): previously just
            # `return`ed here once sampling started -- self._state_sub was
            # destroyed in _start_sampling() the moment the pre-sampling gate
            # confirmed, so nothing watched the car for the rest of the ~30s
            # sampling window at all. Now kept alive through the whole window
            # (see _start_sampling()'s own comment) specifically so this
            # continuous check can run -- any single reading over threshold
            # aborts immediately, no averaging/continuous-confirmation window
            # the way the pre-sampling StationaryGate uses (a moving car
            # mid-sample is disqualifying the instant it's detected, not after
            # it's been moving for a while).
            if self.done:
                return
            if exceeds_motion_threshold(msg.state.speed, self.stationary_erpm_threshold):
                self._abort_motion_during_sampling(
                    f'ERPM speed={msg.state.speed:.1f} exceeded threshold '
                    f'{self.stationary_erpm_threshold:.0f}')
            return
        self._gate.on_speed_sample(msg.state.speed)
        if self._gate.confirmed:
            self._start_sampling()

    def _check_stationary_watchdog(self):
        if self._sub is not None or self._gate.confirmed:
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
            'Aborting without sampling.')
        self.exit_code = _EXIT_NOT_STATIONARY
        self.done = True

    def _start_sampling(self):
        self._stationary_watchdog.cancel()
        # calibration-safety-gates pass (Phase A): self._state_sub is
        # deliberately NOT destroyed here anymore -- it used to be, which is
        # exactly what let the car (or anything else) move/get bumped for the
        # rest of the sampling window completely unnoticed (see this pass's
        # own report, and _state_callback's own updated comment for the
        # during-sampling check this now enables). Torn down at the end of
        # the run instead, in _finish()/_abort_motion_during_sampling().
        self._sampling_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f'Confirmed stationary for {self.stationary_confirm_sec:.1f}s -- sampling gyro '
            f'bias on "{self.imu_topic}" for {self.sample_duration_sec:.1f}s.')
        self._sub = self.create_subscription(Imu, self.imu_topic, self._imu_callback, 50)
        self._timer = self.create_timer(self.sample_duration_sec, self._finish)

    def _sampling_elapsed(self):
        return self.get_clock().now().nanoseconds / 1e9 - self._sampling_start_time

    def _abort_motion_during_sampling(self, reason):
        """calibration-safety-gates pass (Phase A): shared abort path for both
        the ERPM-based check (_state_callback) and the accelerometer-based
        check (_imu_callback) -- either one detecting real motion/vibration
        partway through the sampling window aborts immediately, tearing down
        everything and refusing to write, rather than letting a disturbed
        window silently average into the final result (see this pass's own
        report for the write this closes off)."""
        if self.done:
            return
        self.get_logger().error(
            f'Motion/vibration detected {self._sampling_elapsed():.2f}s into the '
            f'{self.sample_duration_sec:.1f}s gyro-bias sampling window -- {reason}. '
            'Aborting without writing vesc.yaml (a moving/disturbed car cannot '
            'measure a static bias).')
        if self._timer is not None:
            self._timer.cancel()
        if self._sub is not None:
            self._sub.destroy()
            self._sub = None
        if self._state_sub is not None:
            self._state_sub.destroy()
            self._state_sub = None
        self.exit_code = EXIT_MOTION_DURING_SAMPLING
        self.done = True

    def _imu_callback(self, msg: Imu):
        if self.done:
            return
        accel = msg.linear_acceleration
        if exceeds_vibration_threshold(
                accel.x, accel.y, accel.z, self.gyro_bias_vibration_threshold):
            deviation = abs(accel.x) + abs(accel.y) + abs(accel.z - 1.0)
            self._abort_motion_during_sampling(
                f'accelerometer deviation |ax|+|ay|+|az-1.0|={deviation:.4f} '
                f'exceeded threshold {self.gyro_bias_vibration_threshold:.4f} '
                f'(ax={accel.x:+.4f} ay={accel.y:+.4f} az={accel.z:+.4f}, in g)')
            return
        self._samples_x.append(msg.angular_velocity.x)
        self._samples_y.append(msg.angular_velocity.y)
        self._samples_z.append(msg.angular_velocity.z)

    def _finish(self):
        # calibration-safety-gates pass (Phase A): a motion/vibration abort
        # (_abort_motion_during_sampling) already cancels this timer, but
        # guard anyway in case both fire in the same executor pass -- see
        # that method's own docstring.
        if self.done:
            return
        self._timer.cancel()
        self._sub.destroy()
        self._sub = None
        if self._state_sub is not None:
            self._state_sub.destroy()
            self._state_sub = None
        count = len(self._samples_z)

        # calibration-safety-gates pass (Phase A): min_samples is now a HARD
        # gate, not a warning -- previously `count < self.min_samples` only
        # logged a warning and fell through to compute the mean and write
        # anyway (the ONLY hard floor was count < 2). That's a real bug, not
        # just missing defense-in-depth: min_samples existed specifically to
        # say "don't trust a result built from too little data," but nothing
        # actually enforced it. See this pass's own report -- a below-
        # threshold-but-still->=2 run succeeding silently is one of the
        # concrete, ranked hypotheses for the 2026-08-19 11:06:58 bad write.
        if count < self.min_samples:
            if count < 2:
                self.get_logger().error(
                    f'Only {count} sample(s) received on "{self.imu_topic}" -- is the '
                    'IMU publishing? No result to report -- aborting without writing '
                    'vesc.yaml.')
            else:
                self.get_logger().error(
                    f'Only {count} samples received on "{self.imu_topic}" '
                    f'(min_samples={self.min_samples}) -- insufficient to trust the '
                    'result. Aborting without writing vesc.yaml.')
            self.exit_code = EXIT_INSUFFICIENT_SAMPLES
            self.done = True
            return

        def report(axis, samples):
            mean = statistics.mean(samples)
            stdev = statistics.stdev(samples)
            self.get_logger().info(
                f'  angular_velocity.{axis}: mean={mean:+.6f} rad/s  stdev={stdev:.6f} rad/s')
            return mean

        self.get_logger().info(f'Gyro bias over {count} samples:')
        report('x', self._samples_x)
        report('y', self._samples_y)
        residual_z = report('z', self._samples_z)

        if YAML is None:
            self.get_logger().error(
                'ruamel.yaml is not installed (pip install ruamel.yaml or apt install '
                f'python3-ruamel.yaml) -- cannot write {self.vesc_yaml_path}. Apply the '
                'measured residual above to gyro_bias_z manually (ADD it to the value '
                'already in the file -- see this method\'s own comment on why) -- see '
                'f1tenth_diagnostics/README.md.')
            self.exit_code = EXIT_MISSING_DEPENDENCY
            self.done = True
            return

        # BUG FIX, found live (real, reproducible -- not a one-off): vesc_driver_node
        # subtracts the CURRENTLY CONFIGURED gyro_bias_z from the raw hardware gyro
        # reading before ever publishing self.imu_topic (vesc_driver.cpp:
        # `angular_velocity.z = imuData->gyr_z() - gyro_bias_z_`) -- so the mean this
        # node just measured is a RESIDUAL relative to whatever gyro_bias_z the
        # driver it sampled from was started with, not the raw hardware bias
        # directly. The previous version of this method wrote that residual straight
        # into gyro_bias_z, REPLACING the old value instead of correcting it --
        # mathematically: measured_residual = raw_hardware_bias - old_gyro_bias_z, so
        # the correct total to write is old_gyro_bias_z + measured_residual, not
        # measured_residual alone. Getting this wrong doesn't self-correct with
        # repeated runs either -- it compounds, since each run's "old" value is the
        # previous run's wrong write: confirmed live, two consecutive automatic
        # calibration passes (this whole session's own "recalibrate the gyro" pass)
        # wrote 2.017238 then 1.254592 in a row, both wildly implausible for a
        # resting car (should be within roughly a couple hundredths of a rad/s),
        # while x/y (never bias-corrected upstream, so never subject to this bug)
        # stayed small and sane both times -- that specific z-only pattern is what
        # gave this away, not a hunch. old_gyro_bias_z is read directly from
        # vesc_yaml_path itself (read_vesc_yaml_value) rather than trusted from a
        # separately-passed parameter -- see that function's own docstring for why
        # that's the more robust source of truth for "what was actually subtracted
        # from the signal this node just sampled."
        old_gyro_bias_z = read_vesc_yaml_value(self.vesc_yaml_path, 'gyro_bias_z')
        corrected_gyro_bias_z = old_gyro_bias_z + residual_z
        self.get_logger().info(
            f'measured residual={residual_z:+.6f} rad/s (relative to the '
            f'gyro_bias_z={old_gyro_bias_z:+.6f} already subtracted upstream by '
            f'vesc_driver_node) -- corrected total gyro_bias_z='
            f'{corrected_gyro_bias_z:+.6f} rad/s. This is the value that matters '
            'for the planar EKF fusion (imu0_config keeps vyaw only).')

        # calibration-safety-gates pass (Phase A): the last line of defense --
        # see calibration_common.check_sanity_bound()'s own docstring for the
        # full reasoning and the exact bad write this closes. Must run AFTER
        # the residual-vs-absolute correction above (it's corrected_gyro_
        # bias_z, the value that would actually be written, that gets
        # checked -- not the raw residual_z).
        if not check_sanity_bound(
                'gyro_bias_z', corrected_gyro_bias_z, old_gyro_bias_z,
                self.gyro_bias_absolute_bound, self.gyro_bias_delta_bound,
                self.get_logger()):
            self.exit_code = EXIT_SANITY_VIOLATION
            self.done = True
            return

        write_vesc_yaml(
            self.vesc_yaml_path, {'gyro_bias_z': corrected_gyro_bias_z}, self.get_logger())
        self.exit_code = EXIT_SUCCESS
        self.done = True


def main():
    rclpy.init()
    node = GyroBiasCalibrationNode()
    try:
        # NOT rclpy.spin(node): same reasoning as sensor_covariance_calibration_
        # node -- calling rclpy.shutdown() from inside a callback running under
        # the executor deadlocks (executor.shutdown() waits for that same
        # callback to finish). Callbacks only set node.done; shutdown happens
        # here, in the main thread, once the loop notices it. This also fixes
        # this node's previous behavior of never exiting on its own (plain
        # rclpy.spin(node) blocked forever after _finish() logged its result,
        # requiring a manual Ctrl+C every time).
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
