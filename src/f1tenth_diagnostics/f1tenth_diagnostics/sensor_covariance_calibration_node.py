"""Measures message-level (Level 1) covariance for the VESC IMU driver and
vesc_to_odom_node_backup, and writes the result directly into the shared
source-tree config both nodes read at startup (f1tenth_bringup/config/vesc.yaml).

Two calibration_mode values:

- 'stationary' (default): run with the car completely stationary and level, for
  sample_duration_sec (default 60s). Subscribes concurrently to the
  pre-covariance raw topics -- imu_topic (default /sensors/imu/raw) and
  odom_topic (default /odom) -- and computes the variance of each channel with
  Welford's online algorithm (O(1) memory per channel, no sample buffer):
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
  not overwritten.

  MUST be started with `ros2 run` directly, never `ros2 launch` --
  confirm_light_motion_start()'s input() confirmation gate needs a real
  stdin, which `ros2 launch` does not reliably forward to a node process
  (same reason vesc_tuning's steering_calibration_node.py /
  speed_sweep_diagnostic_node.py have no launch file at all). Because of
  this, calibration.launch.py no longer exposes calibration_mode or any
  light_motion_* argument -- it only ever runs 'stationary' (its own node
  default). See confirm_light_motion_start()'s docstring and
  f1tenth_diagnostics/README.md for the exact `ros2 run` invocation.

Unlike gyro_bias_calibration_node (read-only, reports only), this node WRITES
its result in both modes: it backs up vesc.yaml (timestamped copy, same
directory) and then patches only the relevant variance keys in place via
ruamel.yaml's round-trip mode, preserving all existing comments/formatting/
ordering (see _write_yaml -- shared by both modes, stationary passes all 7
keys, light_motion passes only vx_variance). Requires `ruamel.yaml` (pip
install ruamel.yaml or apt install python3-ruamel.yaml).

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
  2. calibration.launch.py now passes an EXPLICIT vesc_yaml_path parameter
     (overriding #1) computed by resolve_source_vesc_yaml_path() below.
     Initially this was written assuming ament_python always editable-installs
     a package's own .py modules (egg-link back to source) regardless of
     --symlink-install -- live-verified on this actual workspace to be FALSE:
     without --symlink-install (confirmed to be this workspace's real state),
     THIS FILE ITSELF is installed as a plain copy under
     install/f1tenth_diagnostics/lib/python3.10/site-packages/..., not an
     egg-link, so its own __file__ does not reach source either. What IS
     reliable regardless of --symlink-install is colcon's own workspace
     layout convention: <ws_root>/install/ and <ws_root>/src/ are always
     siblings. resolve_source_vesc_yaml_path() walks up this file's resolved
     path looking for an 'install' directory and, if found, derives the
     workspace root (that directory's parent) and from there the real
     src/f1tenth_bringup/config/vesc.yaml -- working whether this file itself
     landed in install-space (the common case, verified) or -- if some future
     build DOES use --symlink-install -- directly in src/ already (handled as
     a fallback by anchoring off this file's own location instead, see that
     function's docstring). This is why the fix lives here (an importable
     function on the node module, easy to keep in sync with how this package
     actually gets built) rather than as a __file__ trick inside the launch
     file itself, which would NOT reliably resolve to source either (a launch
     file is a plain copy either way, with no equivalent 'install' ancestor
     trick available to it since it doesn't sit under site-packages/).
"""

import datetime
import os
import shutil
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

try:
    from ruamel.yaml import YAML
except ImportError:
    YAML = None


def resolve_source_vesc_yaml_path() -> str:
    """Reliably resolve the real SOURCE-TREE vesc.yaml, regardless of whether
    this workspace was built with --symlink-install -- see the module
    docstring's "Path resolution" section for the full reasoning. Imported and
    used by calibration.launch.py to compute an explicit vesc_yaml_path
    override; NOT used to change this node's own __init__-time default (left
    intact as a fallback, per that same section).

    IMPORTANT (corrected after live verification on this actual workspace):
    ament_python's own .py modules are only editable-installed (egg-link back
    to src/) when the workspace was built WITH --symlink-install. Without it
    (confirmed to be this workspace's actual state -- see module docstring),
    this file's own __file__ resolves to a PLAIN COPY under
    install/f1tenth_diagnostics/lib/python3.10/site-packages/..., same as any
    other install-space file -- so anchoring off __file__ alone is not
    reliable either. What IS reliable regardless of --symlink-install is
    colcon's own workspace layout convention: <ws_root>/install/<pkg>/... and
    <ws_root>/src/<pkg>/... are always siblings. So: walk up this file's
    resolved path looking for an 'install' directory; if found, its parent IS
    the workspace root, and 'src' sits right next to it. If no 'install'
    ancestor is found at all, this file must BE the source file already
    (--symlink-install case), so fall back to anchoring directly off it.
    """
    this_file = Path(__file__).resolve()
    for parent in this_file.parents:
        if parent.name == 'install':
            ws_root = parent.parent
            return str(ws_root / 'src' / 'f1tenth_bringup' / 'config' / 'vesc.yaml')

    # No 'install' ancestor -- this file's own resolved location IS inside
    # src/ already (editable install via --symlink-install).
    # parents[0] = f1tenth_diagnostics/f1tenth_diagnostics (this file's dir)
    # parents[1] = f1tenth_diagnostics (the package, containing setup.py)
    # parents[2] = src (the actual workspace src/ root)
    src_dir = this_file.parents[2]
    return str(src_dir / 'f1tenth_bringup' / 'config' / 'vesc.yaml')


class _Welford:
    """Online (flat-memory) mean/variance accumulator -- no sample buffer."""

    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0

    def update(self, x):
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        delta2 = x - self.mean
        self.m2 += delta * delta2

    def variance(self):
        # sample variance (n-1 denominator); undefined for n < 2
        return self.m2 / (self.n - 1)


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
        # resolution" section for why calibration.launch.py now overrides
        # this with an explicit, reliably-source-anchored value instead of
        # relying on this symlink-following resolution actually finding a
        # symlink.
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

        self.exit_code = 1
        self.done = False

        self._accumulators = {
            'gyro_variance_x': _Welford(),
            'gyro_variance_y': _Welford(),
            'gyro_variance_z': _Welford(),
            'accel_variance_x': _Welford(),
            'accel_variance_y': _Welford(),
            'accel_variance_z': _Welford(),
            'vx_variance': _Welford(),
        }

        if self.calibration_mode == 'stationary':
            # ---- existing behavior, unchanged ------------------------------
            self._imu_sub = self.create_subscription(
                Imu, self.imu_topic, self._imu_callback, 50)
            self._odom_sub = self.create_subscription(
                Odometry, self.odom_topic, self._odom_callback, 50)
            self._timer = self.create_timer(self.sample_duration_sec, self._finish)

            self.get_logger().info(
                f'Sampling covariance on "{self.imu_topic}" and "{self.odom_topic}" for '
                f'{self.sample_duration_sec:.1f}s -- keep the car completely stationary.')
        else:
            # ---- new light_motion mode --------------------------------------
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

    def _imu_callback(self, msg: Imu):
        self._accumulators['gyro_variance_x'].update(msg.angular_velocity.x)
        self._accumulators['gyro_variance_y'].update(msg.angular_velocity.y)
        self._accumulators['gyro_variance_z'].update(msg.angular_velocity.z)
        self._accumulators['accel_variance_x'].update(msg.linear_acceleration.x)
        self._accumulators['accel_variance_y'].update(msg.linear_acceleration.y)
        self._accumulators['accel_variance_z'].update(msg.linear_acceleration.z)

    def _odom_callback(self, msg: Odometry):
        if self.calibration_mode == 'stationary':
            self._accumulators['vx_variance'].update(msg.twist.twist.linear.x)
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
            self.exit_code = 1
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
            self.exit_code = 1
            self.done = True
            return

        self._write_yaml(results)
        self.exit_code = 0
        self.done = True

    def _write_yaml(self, results):
        """Back up vesc_yaml_path (timestamped copy) then patch, in place via
        ruamel.yaml's round-trip mode, exactly the keys present in `results`
        (preserves comments/formatting/ordering). Shared by both calibration
        modes -- stationary's _finish() passes all 7 keys; light_motion's
        _finish_light_motion() passes only vx_variance -- each key is routed
        to the YAML section it actually lives in (shared /**: block for the
        6 gyro/accel keys, vesc_to_odom_node's own block for vx_variance)."""
        timestamp = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
        backup_path = f'{self.vesc_yaml_path}.bak.{timestamp}'
        shutil.copy2(self.vesc_yaml_path, backup_path)
        self.get_logger().info(f'Backed up {self.vesc_yaml_path} -> {backup_path}')

        yaml = YAML()
        yaml.preserve_quotes = True
        with open(self.vesc_yaml_path, 'r') as f:
            data = yaml.load(f)

        shared_params = data['/**']['ros__parameters']
        odom_params = data['vesc_to_odom_node']['ros__parameters']
        for key, value in results.items():
            if key == 'vx_variance':
                odom_params[key] = value
            else:
                shared_params[key] = value

        with open(self.vesc_yaml_path, 'w') as f:
            yaml.dump(data, f)

        self.get_logger().info(
            f'calibration complete, updated {list(results.keys())} in: '
            f'[{self.vesc_yaml_path}]')

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
            self.exit_code = 1
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
            self.exit_code = 1
            self.done = True
            return

        self._write_yaml({'vx_variance': variance})
        self.exit_code = 0
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
            sys.exit(1)

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
