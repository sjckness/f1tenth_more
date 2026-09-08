"""Open-loop steering-offset / effective-wheelbase calibration.

Drives a fixed S-curve with the MPC out of the loop, fits the bicycle model

    d(psi)/d(s) = tan(delta + delta0) / L_eff

to the per-segment (commanded steering, heading change, arc length) triples,
and -- only if three independent gates pass -- writes delta0 and L_eff back
into the config files that actually own them.

STANDALONE, MANUAL TOOL. Not wired into supervisor_bringup: this is the one
node in f1tenth_diagnostics that deliberately drives the car, so it must be
started by a human who is watching it, with a hand on the joystick. Launch it
from a second terminal (see steering_offset_calibration.launch.py).


WHERE THE POSE COMES FROM, AND WHY IT IS NOT NEGOTIABLE
psi and s come from /slam/pose ONLY. Not /odometry/filtered, and not
/ekf_global/odometry/filtered: both of those fuse the wheel odometry and the
gyro, which are exactly the things a steering calibration is trying to check.
Fitting a steering offset against a signal that already contains the steering
model's own output is circular, and the failure mode is not a wild number --
it is a clean-looking fit with a small residual and a confident CI, which is
far worse than an obvious one.

/slam/pose and "map -> base_link" are NOT interchangeable here, despite both
being nominally map-frame. map -> base_link is composed at runtime as
(map -> odom) * (odom -> base_link), and odom -> base_link is published by the
odom EKF at ~50 Hz -- gyro-fused. Only the ~2 Hz map -> odom correction
carries independent SLAM information. So reading map -> base_link from TF
would give a comfortable-looking 50 Hz pose stream whose high-rate content is
precisely the signal we must not use. This node therefore subscribes to
/slam/pose directly and does not offer a TF fallback.

That costs us rate. Measured on this stack, /slam/pose publishes at ~2 Hz. At
0.25 m/s a 0.6 m segment lasts ~2.4 s, so it yields ~5 fixes, and ~4 after the
settling discard. That is thin, and it is why pose support is a gate rather
than a warning -- see steering_offset_fit.check_pose_support.


WHY PREFLIGHT DOES NOT WAIT FOR A POSE
It used to, and the check was unsatisfiable by construction. /slam/pose is
silent on a stationary car BY DESIGN: slam_toolbox's shouldProcessScan is
distance-gated (minimum_travel_distance 0.03 in
f1tenth_online_async.yaml), so a parked car produces no pose no matter how
long preflight listens. A tool whose preflight requires motion, run on a car
that has not moved yet, refuses every time -- and it refuses with a message
blaming SLAM, which sends the operator to look at the wrong thing. This
project has already root-caused that once; downstream consumers must not gate
on /slam/pose freshness.

What replaces it is two staged checks, in this order:

  1. READINESS, before anything moves: /slam/map has been received. That
     proves slam_toolbox is up and holds a map, and it is satisfiable at
     standstill because the map is latched (TRANSIENT_LOCAL), not
     motion-gated.

     NOT map -> odom as a proxy. ekf_global publishes that transform at 50 Hz
     whether or not slam_toolbox ever produced anything, so its presence
     proves nothing at all about SLAM. See the note above on map -> base_link
     for the same trap in its other form.

  2. NUDGE, then confirm: a bounded creep forward (nudge_distance_m, default
     0.18 m, at nudge_speed_mps, default 0.1 m/s) past the 0.03 m gate,
     followed by a hard requirement for a /slam/pose within
     nudge_pose_timeout_sec. No pose, no drive.

The nudge is strictly stronger than any topic-existence check: it exercises
the whole chain end to end -- our mux lane, the VESC, the wheels, wheel
odometry, the scan match, the published pose -- before the car commits to
~3.6 m of open-loop S-curve. It is also the only part of preflight that
moves the car, so it is logged loudly, bounded in both distance and time, and
covered by the same SIGINT/SIGTERM zero-command handler as the drive itself
(main() installs that handler before preflight is entered, and the nudge loop
checks self.done on every iteration).


WHAT GETS WRITTEN, AND WHERE (NOT vesc.yaml, for delta0)
The obvious guess -- "write delta0 into vesc.yaml" -- is wrong on this stack,
and wrong in a way that fails silently. vesc.yaml's own comments say so: the
five steering-calibration keys (servo_min, servo_max,
steering_angle_to_servo_offset, steering_angle_to_servo_gain_left/_right)
were moved out to f1tenth_hardware/config/steering_calibration.yaml, which
vesc.launch.py loads AFTER vesc.yaml. A value written into vesc.yaml for any
of those keys is silently overridden at load time -- it would look written,
read back fine from the file, and do nothing.

So:
  - delta0  -> steering_calibration.yaml, as steering_angle_to_servo_offset.
  - L_eff   -> vesc.yaml, vesc_to_odom_node.wheelbase.

delta0 is not stored directly; it is a steering-ANGLE offset and the file
holds a SERVO offset. ackermann_to_vesc_node computes

    servo = gain * delta_cmd + offset

and the calibration says the wheels actually take (delta_cmd + delta0) when we
command delta_cmd. To make the commanded angle the achieved angle we need the
servo value that previously produced (delta_cmd - delta0):

    servo_new = gain * (delta_cmd - delta0) + offset
              = gain * delta_cmd + (offset - gain * delta0)

so  offset_new = offset - gain * delta0.  With gain negative on this car
(-1.2135), a positive delta0 RAISES the stored offset. _servo_offset_for()
does this conversion and the result is bounds-checked against servo_min /
servo_max before anything is written.

L_eff goes only to vesc_to_odom_node.wheelbase -- the dead-reckoning model,
where an effective wheelbase absorbing tyre slip is the right thing. The other
two copies of `wheelbase` in this workspace (f1tenth_behavior/config/
twist_to_ackermann.yaml, and vesc_tuning's steering_calibration_node.py
default) are deliberately NOT touched: those generate commands from a desired
curvature, where the geometric 0.305 m is the correct value and substituting a
slip-inflated one would bias every command. The final report prints both
locations so the choice stays visible rather than buried here.


DRIVE GEOMETRY -- CHECK YOUR SPACE FIRST
Per repetition: straight 0.6 m, +a 0.6 m, -a 1.2 m, +a 0.6 m, straight 0.6 m
= 3.6 m of PATH. At the default a = 10 deg and L = 0.305 m the curvature is
tan(0.1745)/0.305 = 0.578 rad/m, so the +a segments each turn ~19.9 deg and
the -a segment turns ~-39.7 deg; net heading change per repetition is zero by
construction. The footprint is roughly 3.5 m longitudinally by 0.9 m
laterally.

"Fits a 3 m corridor" is only true of the WIDTH. It does not fit a 3 m-long
run, and repetitions are driven sequentially, so three of them need ~10.8 m of
clear straight space unless the car is repositioned. The node therefore stops,
zeroes the command and waits inter_rep_pause_sec between repetitions, logging
a reposition prompt. Each repetition is fit independently, so repositioning
between them is harmless.

Steering limits are checked against the real ones for this car
(min_steering_angle -0.264 rad, max +0.314 rad -- note the asymmetry) before
the first command, not assumed.


MUX LANE
Commands go out on /calibration_drive, registered in mux.yaml at priority 50.
The work order asked for "above nav (10) and below safety (200)", but that
range is not empty on this stack: joystick sits at 100. Taking any priority
above 100 would put a calibration drive above the human's manual override,
which is exactly backwards for a tool whose safety story is "a human is
watching with a hand on the joystick". 50 keeps the intended ordering
(safety 200 > joystick 100 > calibration 50 > navigation 10).

The mux drops a lane whose input goes silent for 0.2 s, so this node publishes
at publish_rate_hz (default 20) throughout, and on any exit path publishes
explicit zeros as well rather than relying on that timeout alone.
"""

import math
import os
import signal
import socket
import sys
import tempfile
import time
from datetime import datetime

from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from f1tenth_messages.msg import MissionStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

from f1tenth_diagnostics.calibration_common import (
    EXIT_INSUFFICIENT_SAMPLES,
    EXIT_MISSING_DEPENDENCY,
    EXIT_SANITY_VIOLATION,
    EXIT_SUCCESS,
    YAML,
    resolve_source_config_path,
    write_yaml_config_with_provenance,
)
from f1tenth_diagnostics import steering_offset_fit as fitlib

# Exit codes above calibration_common's own block (0-5), so a caller can tell
# this node's specific refusals apart from the stationary-calibration ones.
EXIT_PREFLIGHT_REFUSED = 10
EXIT_ABORTED_ON_SAFETY = 11
EXIT_GATES_REFUSED = 12

TOOL_NAME = 'steering_offset_calibration_node'

# (length_key, sign_of_amplitude) -- the S-curve, in order. See the module
# docstring's DRIVE GEOMETRY section.
# Two segment plans, both satisfying the same three requirements the fit has:
# at least one near-zero segment (which pins delta0 directly -- see
# steering_offset_fit._residuals_and_jacobian on why the straight segments do
# real work), and both steering signs once the per-repetition alternation in
# _segment_spec is applied across >= 2 repetitions.
#
# 'short' is the default. 'full' bookends the S-curve with a second straight
# and a third steered segment: strictly more information per repetition, at
# 3.6 m instead of 2.4 m of clear space, and 15 segments instead of 9 for the
# default 3 repetitions. Prefer 'short' unless a run has actually been refused
# for thin data -- the extra segments buy less than a fourth repetition would,
# and a long profile is the thing most likely to run out of room or patience
# mid-run (a run aborted part-way is worth nothing to the fit).
SHORT_SEGMENT_PLAN = (
    ('straight_length_m', 0.0),
    ('step_length_m', +1.0),
    ('reverse_length_m', -1.0),
)
FULL_SEGMENT_PLAN = (
    ('straight_length_m', 0.0),
    ('step_length_m', +1.0),
    ('reverse_length_m', -1.0),
    ('step_length_m', +1.0),
    ('straight_length_m', 0.0),
)
SEGMENT_PLANS = {'short': SHORT_SEGMENT_PLAN, 'full': FULL_SEGMENT_PLAN}


def yaw_from_quaternion(q):
    """Planar yaw from a geometry_msgs Quaternion. Inlined rather than pulling
    in tf_transformations: this is the only transform this node needs, and
    tf_transformations is not currently a dependency of this package."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class SteeringOffsetCalibrationNode(Node):

    def __init__(self, **kwargs):
        # **kwargs: lets a test construct this node with parameter_overrides,
        # same pattern gyro_bias_calibration_node.__init__ already uses.
        super().__init__(TOOL_NAME, **kwargs)

        # ---- topics -------------------------------------------------------
        self.pose_topic = str(self.declare_parameter('pose_topic', '/slam/pose').value)
        # Readiness proof that slam_toolbox is up, checkable at standstill.
        # See the module docstring on why /slam/pose freshness is not usable
        # for this and why map -> odom is not an acceptable substitute.
        self.map_topic = str(self.declare_parameter('map_topic', '/slam/map').value)
        self.drive_topic = str(
            self.declare_parameter('drive_topic', '/calibration_drive').value)
        self.clearance_topic = str(
            self.declare_parameter('clearance_topic', '/costmap/front_clearance').value)
        self.mission_status_topic = str(
            self.declare_parameter('mission_status_topic', '/mission/status').value)
        self.safety_stop_topic = str(
            self.declare_parameter('safety_stop_topic', '/safety_stop').value)

        # ---- drive profile ------------------------------------------------
        self.speed_mps = float(self.declare_parameter('speed_mps', 0.25).value)
        self.amplitude_rad = float(
            self.declare_parameter('amplitude_rad', math.radians(10.0)).value)
        self.straight_length_m = float(
            self.declare_parameter('straight_length_m', 0.6).value)
        self.step_length_m = float(self.declare_parameter('step_length_m', 0.6).value)
        self.reverse_length_m = float(self.declare_parameter('reverse_length_m', 1.2).value)
        self.repetitions = int(self.declare_parameter('repetitions', 3).value)
        self.settle_sec = float(self.declare_parameter('settle_sec', 0.2).value)
        self.inter_rep_pause_sec = float(
            self.declare_parameter('inter_rep_pause_sec', 6.0).value)
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 20.0).value)
        # 'short' (3 segments/rep, 2.4 m) or 'full' (5 segments/rep, 3.6 m).
        # Validated in preflight, not here: an unknown name must refuse with a
        # readable message rather than raise out of the constructor.
        self.segment_plan_name = str(
            self.declare_parameter('segment_plan', 'short').value)

        # ---- mode ----------------------------------------------------------
        # 'static' is mode A (the ground truth: measured wheel angles, no
        # driving); 'drive' is mode B (the S-curve, now validation of A plus
        # whatever slip A cannot see).
        self.calibration_mode = str(
            self.declare_parameter('calibration_mode', 'drive').value)

        # ---- static sweep (mode A) -----------------------------------------
        self.sweep_amplitude_rad = float(
            self.declare_parameter('sweep_amplitude_rad', math.radians(15.0)).value)
        self.sweep_steps = int(self.declare_parameter('sweep_steps', 7).value)
        self.backlash_tolerance_rad = float(
            self.declare_parameter('backlash_tolerance_rad',
                                   fitlib.DEFAULT_BACKLASH_TOLERANCE_RAD).value)
        self.min_sweep_span_rad = float(
            self.declare_parameter('min_sweep_span_rad',
                                   fitlib.DEFAULT_MIN_SWEEP_SPAN_RAD).value)

        # ---- hardware limits (checked, not assumed) ------------------------
        self.min_steering_angle = float(
            self.declare_parameter('min_steering_angle', -0.264).value)
        self.max_steering_angle = float(
            self.declare_parameter('max_steering_angle', 0.314).value)
        # PINNED, never fitted. See steering_offset_fit's module docstring for
        # why fitting L_eff was wrong, and PINNED_WHEELBASE_M for the fact
        # that 0.3302 is assumed from the F1TENTH spec, not measured here.
        self.pinned_wheelbase_m = float(
            self.declare_parameter('pinned_wheelbase_m',
                                   fitlib.PINNED_WHEELBASE_M).value)
        # Reported against the pinned value, never written. See
        # _report_wheelbase_discrepancy.
        self.vesc_yaml_wheelbase_m = float(
            self.declare_parameter('vesc_yaml_wheelbase_m',
                                   fitlib.VESC_YAML_WHEELBASE_M).value)

        # ---- safety --------------------------------------------------------
        self.min_front_clearance_m = float(
            self.declare_parameter('min_front_clearance_m', 0.4).value)
        self.max_lateral_excursion_m = float(
            self.declare_parameter('max_lateral_excursion_m', 0.8).value)
        self.hard_timeout_sec = float(
            self.declare_parameter('hard_timeout_sec', 300.0).value)
        self.clearance_stale_sec = float(
            self.declare_parameter('clearance_stale_sec', 1.0).value)
        self.require_estop_publisher = bool(
            self.declare_parameter('require_estop_publisher', True).value)
        self.preflight_timeout_sec = float(
            self.declare_parameter('preflight_timeout_sec', 20.0).value)

        # ---- preflight nudge (mode B only) ---------------------------------
        # The one part of preflight that moves the car. Bounded in distance
        # AND in time: the distance bound is what the operator was told to
        # clear, and the time bound is what actually holds if the wheels slip
        # or the VESC ignores us -- neither alone is enough. Default 0.18 m is
        # comfortably past slam_toolbox's 0.03 m minimum_travel_distance gate
        # without being a manoeuvre.
        self.nudge_distance_m = float(
            self.declare_parameter('nudge_distance_m', 0.18).value)
        self.nudge_speed_mps = float(
            self.declare_parameter('nudge_speed_mps', 0.1).value)
        # How long to keep waiting for the pose AFTER the creep has finished
        # and the car is stopped again. /slam/pose runs at ~2 Hz and the scan
        # match lags the motion, so this is not zero.
        self.nudge_pose_timeout_sec = float(
            self.declare_parameter('nudge_pose_timeout_sec', 4.0).value)

        # ---- gates ---------------------------------------------------------
        # Half a /slam/pose interval at the ~1.9 Hz this stack actually
        # publishes -- the resolution with which a turning point located
        # between consecutive fixes can be placed. See fitlib.TurningPoint.
        self.turning_point_sigma_t = float(
            self.declare_parameter('turning_point_sigma_t', 0.26).value)
        self.min_pose_samples_per_segment = int(
            self.declare_parameter('min_pose_samples_per_segment', 4).value)
        self.max_param_correlation = float(
            self.declare_parameter('max_param_correlation',
                                   fitlib.DEFAULT_MAX_PARAM_CORRELATION).value)
        self.delta0_absolute_bound_rad = float(
            self.declare_parameter('delta0_absolute_bound_rad', math.radians(8.0)).value)
        self.gain_min = float(self.declare_parameter('gain_min', 0.5).value)
        self.gain_max = float(self.declare_parameter('gain_max', 2.0).value)

        # ---- write-back ----------------------------------------------------
        self.write_enabled = bool(self.declare_parameter('write_enabled', True).value)
        self.steering_calibration_yaml_path = str(self.declare_parameter(
            'steering_calibration_yaml_path',
            resolve_source_config_path('f1tenth_hardware', 'f1tenth_hardware', 'config',
                                       'steering_calibration.yaml')).value)
        self.vesc_yaml_path = str(self.declare_parameter(
            'vesc_yaml_path',
            resolve_source_config_path('f1tenth_bringup', 'config', 'vesc.yaml')).value)
        self.raw_dump_dir = str(self.declare_parameter(
            'raw_dump_dir', os.path.expanduser('~/.ros/steering_offset_calibration')).value)

        # ---- runtime state --------------------------------------------------
        self.run_id = datetime.now().strftime('steer-offset-%Y-%m-%dT%H-%M-%S')
        self.exit_code = EXIT_SUCCESS
        self.done = False
        self.samples = []
        self.sweep_samples = []
        self.report_lines = []
        self._stopping = False

        self._pose = None            # (t, x, y, yaw) most recent
        self._pose_count = 0
        self._map_seen = False
        self._map_info = None
        self._clearance = None
        self._clearance_stamp = None
        self._mission_state = None
        self._mission_estop = False
        self._mission_seen = False

        self.segment_plan = SEGMENT_PLANS.get(self.segment_plan_name, SHORT_SEGMENT_PLAN)

        self._phase = 'preflight'
        self._rep = 0
        self._seg = 0
        self._seg_started = None
        self._seg_poses = []
        # Every pose kept per segment, and every command issued, so the
        # turning-point cross-check can be run after the drive.
        self._all_seg_poses = []
        self._command_history = []
        self._rep_origin = None
        self._pause_until = None
        self._started_monotonic = time.monotonic()

        # ---- ROS interfaces --------------------------------------------------
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        sensor_qos = QoSProfile(
            depth=10, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self._on_pose, sensor_qos)
        self.create_subscription(Float32, self.clearance_topic, self._on_clearance, 10)
        # TRANSIENT_LOCAL to match slam_toolbox's latched map: a map published
        # before this node started is exactly the case we need to see, and a
        # VOLATILE subscription would miss it and make the readiness check as
        # unsatisfiable as the pose check it replaces.
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._on_map,
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # /mission/status is published transient-local ("latched") by
        # MissionLoader -- matching that durability here is what makes the
        # mission-active refusal work at all: with a VOLATILE subscription we
        # would only ever see a mission state that CHANGES after we start, and
        # an already-running mission (exactly the case we must refuse) would
        # be invisible.
        self.create_subscription(
            MissionStatus, self.mission_status_topic, self._on_mission,
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.timer = self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self._tick)
        self._log_plan()

    # ------------------------------------------------------------------ logs
    def _log_plan(self):
        curvature = math.tan(self.amplitude_rad) / self.pinned_wheelbase_m
        turn_step = math.degrees(curvature * self.step_length_m)
        # Derived from the plan actually in use, never restated by hand: the
        # old hard-coded '2 * straight + 2 * step + reverse' silently described
        # the wrong profile the moment a second plan existed.
        path_len = sum(getattr(self, key) for key, _ in self.segment_plan)
        legs = ', '.join(
            f'{getattr(self, key)} straight' if sign == 0.0
            else f'{"+" if sign > 0 else "-"}a {getattr(self, key)}'
            for key, sign in self.segment_plan)
        self.get_logger().info(
            f'=== {TOOL_NAME} / run {self.run_id} ===')
        self.get_logger().info(
            f'plan: {self.repetitions} repetitions x {len(self.segment_plan)} segments '
            f'({self.segment_plan_name}) [{legs}] at {self.speed_mps:.2f} m/s, '
            f'a = {math.degrees(self.amplitude_rad):.1f} deg')
        self.get_logger().info(
            f'geometry: {path_len:.2f} m of path per repetition, each +a segment turns '
            f'~{turn_step:.1f} deg; footprint roughly {path_len - 0.1:.1f} m long x '
            f'{path_len * 0.25:.1f} m wide. Repetitions run SEQUENTIALLY -- either '
            f'clear ~{path_len * self.repetitions:.1f} m or reposition the car during '
            f'the {self.inter_rep_pause_sec:.0f} s pause between them.')

    def _note(self, line):
        self.report_lines.append(line)

    # ------------------------------------------------------------- callbacks
    def _on_pose(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = (stamp, msg.pose.pose.position.x, msg.pose.pose.position.y,
                yaw_from_quaternion(msg.pose.pose.orientation))
        self._pose = pose
        self._pose_count += 1
        if self._phase == 'driving' and self._seg_started is not None:
            if time.monotonic() - self._seg_started >= self.settle_sec:
                self._seg_poses.append(pose)

    def _on_map(self, msg):
        self._map_seen = True
        self._map_info = (msg.info.width, msg.info.height, msg.info.resolution)

    def _on_clearance(self, msg):
        self._clearance = float(msg.data)
        self._clearance_stamp = time.monotonic()

    def _on_mission(self, msg):
        self._mission_state = msg.state
        self._mission_estop = bool(msg.emergency_stop_active)
        self._mission_seen = True

    # -------------------------------------------------------------- commands
    def _publish_drive(self, steering_angle, speed):
        if self._phase == 'driving':
            self._command_history.append(
                (self.get_clock().now().nanoseconds * 1e-9, float(steering_angle)))
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = float(steering_angle)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def publish_stop(self, repeat=5):
        """Publish explicit zeros. Called on every exit path -- normal
        completion, gate refusal, safety abort, exception, and the signal
        handler. The mux would drop our lane after its own 0.2 s timeout
        anyway, but a latched command is exactly the failure this tool must
        never produce, so we say zero out loud rather than going quiet and
        trusting a timeout."""
        for _ in range(repeat):
            self._publish_drive(0.0, 0.0)

    # ------------------------------------------------------------- preflight
    def _discovery_guard(self):
        """Loud, non-hanging check that this process is on the same DDS
        discovery as the rest of the stack.

        Individual component launch files here have no auto-start guard, and
        under a Discovery Server a missing/unreachable server does not error:
        every participant simply sees an empty graph while every node still
        looks alive. `ros2 node list` comes back empty and means nothing.

        This node DETECTS and REPORTS rather than starting a server itself.
        Starting one would mean reaching into f1tenth_bringup (which owns
        ensure_discovery_server.py), and f1tenth_bringup already depends on
        f1tenth_diagnostics -- depending back would be the build-order cycle
        f1tenth_params exists to avoid. The exact command to fix it is printed
        instead.
        """
        endpoint = os.environ.get('ROS_DISCOVERY_SERVER', '').strip()
        if not endpoint:
            self.get_logger().warning(
                'ROS_DISCOVERY_SERVER is NOT set in this shell. The stack normally '
                'runs behind a Fast-DDS Discovery Server (see stack_bringup.launch.py); '
                'if it is up, this process is on plain multicast discovery and will be '
                'INVISIBLE to it -- no pose in, no drive command out, and nothing will '
                'report an error. Export it to match the stack, e.g. '
                'ROS_DISCOVERY_SERVER=127.0.0.1:11811')
            return True

        address, _, port_text = endpoint.partition(':')
        address = address or '127.0.0.1'
        try:
            port = int(port_text or '11811')
        except ValueError:
            self.get_logger().error(
                f'ROS_DISCOVERY_SERVER={endpoint!r} is not <address>:<port>.')
            return False

        # UDP has no connection state to query, so the only real test is trying
        # to bind the port ourselves -- same technique as
        # f1tenth_bringup/scripts/ensure_discovery_server.py's port_is_taken().
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind((address, port))
            bound = True
        except OSError:
            bound = False
        finally:
            probe.close()

        if bound:
            self.get_logger().error(
                f'ROS_DISCOVERY_SERVER points at {address}:{port} but NOTHING is '
                'listening there -- this process would see an empty ROS graph and hang '
                'silently rather than fail. Bring the stack up (it starts the server), '
                f'or start one directly:\n    {self._discovery_server_command(address, port)}'
                '\nRefusing to run.')
            return False
        self.get_logger().info(
            f'discovery server reachable at {address}:{port}.')
        return True

    @staticmethod
    def _discovery_server_command(address, port):
        """The command that actually starts a server on this stack.

        NOT `ros2 run f1tenth_bringup ensure_discovery_server.py` -- that fails
        with "No executable found" (verified live). f1tenth_bringup installs
        that script as a package SHARE resource, not a console_script entry
        point, precisely so launch files can resolve it by path; there is no
        `ros2 run` name for it. stack_bringup.launch.py invokes it as
        `python3 <share>/scripts/ensure_discovery_server.py <addr> <port>`, so
        that is what this prints.
        """
        try:
            share = get_package_share_directory('f1tenth_bringup')
            script = os.path.join(share, 'scripts', 'ensure_discovery_server.py')
        except (PackageNotFoundError, LookupError):
            script = ('$(ros2 pkg prefix f1tenth_bringup)/share/f1tenth_bringup/'
                      'scripts/ensure_discovery_server.py')
        return f'python3 {script} {address} {port}'

    def _participants_visible(self):
        """A discovery server can be up and this process still be alone on it
        (wrong server id, wrong super-client config, stack not started). Count
        other nodes rather than trusting the port probe alone."""
        others = [n for n in self.get_node_names() if n != TOOL_NAME]
        if not others:
            self.get_logger().error(
                'NO PARTICIPANTS VISIBLE: this process can see no other ROS nodes at '
                'all. Either the stack is not running or this shell is on a different '
                'DDS discovery configuration than it is. An empty `ros2 node list` '
                'under a Discovery Server proves nothing -- check the stack is up and '
                'that ROS_DISCOVERY_SERVER matches. Refusing to run.')
            return False
        self.get_logger().info(f'{len(others)} other node(s) visible on the graph.')
        return True

    def _check_steering_limits(self):
        if self.amplitude_rad <= 0.0:
            self.get_logger().error('amplitude_rad must be positive.')
            return False
        if self.amplitude_rad > self.max_steering_angle:
            self.get_logger().error(
                f'amplitude {math.degrees(self.amplitude_rad):.2f} deg exceeds this '
                f'car\'s max_steering_angle {math.degrees(self.max_steering_angle):.2f} '
                'deg. Refusing to run.')
            return False
        if -self.amplitude_rad < self.min_steering_angle:
            self.get_logger().error(
                f'amplitude -{math.degrees(self.amplitude_rad):.2f} deg exceeds this '
                f'car\'s min_steering_angle {math.degrees(self.min_steering_angle):.2f} '
                'deg (the limits are asymmetric on this car). Refusing to run.')
            return False
        self.get_logger().info(
            f'steering amplitude +/-{math.degrees(self.amplitude_rad):.2f} deg is within '
            f'[{math.degrees(self.min_steering_angle):.2f}, '
            f'{math.degrees(self.max_steering_angle):.2f}] deg.')
        return True

    def _check_mission_idle(self):
        if not self._mission_seen:
            self.get_logger().warning(
                f'no MissionStatus seen on {self.mission_status_topic} within the '
                'preflight window. The behaviour tree is probably not running, so no '
                'mission can be active -- continuing, but note that /safety_stop is '
                'published by that same tree (see the e-stop check).')
            return True
        if self._mission_estop:
            self.get_logger().error(
                'mission emergency_stop_active is latched true. Refusing to run.')
            return False
        if self._mission_state in ('RUNNING', 'HOLDING'):
            self.get_logger().error(
                f'a mission is active (state={self._mission_state!r}). This tool drives '
                'the car open-loop and must never contend with a running mission. '
                'Refusing to run.')
            return False
        self.get_logger().info(f'mission state is {self._mission_state!r} -- not active.')
        return True

    def _check_estop_path(self):
        """Verify the e-stop path is live BEFORE the first command.

        Two independent things have to be true, and neither implies the other:
          - something SUBSCRIBES to our drive topic, i.e. the mux is up and our
            lane is registered. Without this the car simply never moves, which
            is safe but wastes a run.
          - something PUBLISHES /safety_stop, i.e. the behaviour tree's safety
            lane exists and can pre-empt us at mux priority 200. Without it
            nothing but this node's own clearance check can stop the car.
        """
        subs = self.count_subscribers(self.drive_topic)
        if subs < 1:
            self.get_logger().error(
                f'nothing is subscribed to {self.drive_topic} -- ackermann_mux is not '
                f'running, or {self.drive_topic} is not registered as a lane in '
                'mux.yaml. The car would not move. Refusing to run.')
            return False
        self.get_logger().info(
            f'{subs} subscriber(s) on {self.drive_topic} (mux lane is live).')

        estop_pubs = self.count_publishers(self.safety_stop_topic)
        if estop_pubs < 1:
            message = (
                f'NO publisher on {self.safety_stop_topic} -- the behaviour tree safety '
                'lane is not running, so nothing can pre-empt this node at mux priority '
                '200. The only remaining automatic stop is this node\'s own front-'
                'clearance check.')
            if self.require_estop_publisher:
                self.get_logger().error(
                    message + ' Refusing to run. Start the behaviour tree, or re-run '
                    'with require_estop_publisher:=false if you accept driving with '
                    'only the joystick and this node\'s own checks.')
                return False
            self.get_logger().warning(message + ' Continuing because '
                                                'require_estop_publisher is false.')
        else:
            self.get_logger().info(
                f'{estop_pubs} publisher(s) on {self.safety_stop_topic} '
                '(safety lane is live).')

        if self._clearance is None:
            self.get_logger().warning(
                f'no reading yet on {self.clearance_topic} -- the front-clearance abort '
                'cannot arm until one arrives. It is enforced as a staleness check once '
                'driving starts.')
        return True

    def _check_segment_plan(self):
        """Refuse an unknown segment_plan rather than silently driving another
        one. Falling back to the default here would be the worst outcome: the
        operator asked for a specific profile, the log would show the geometry
        of a different one, and the only symptom would be a footprint that did
        not match what they cleared space for."""
        if self.segment_plan_name not in SEGMENT_PLANS:
            self.get_logger().error(
                f'unknown segment_plan {self.segment_plan_name!r} -- expected one of '
                f'{sorted(SEGMENT_PLANS)}. Refusing to run.')
            return False
        per_rep = sum(getattr(self, key) for key, _ in self.segment_plan)
        self.get_logger().info(
            f'segment plan {self.segment_plan_name!r}: {len(self.segment_plan)} segments '
            f'x {self.repetitions} repetitions = '
            f'{len(self.segment_plan) * self.repetitions} fitted segments, '
            f'{per_rep:.2f} m per repetition.')
        return True

    def _check_slam_map_ready(self):
        """READINESS, checked at standstill: slam_toolbox is up and holds a map.

        This replaces the old "wait for a /slam/pose" check, which could never
        pass on a parked car -- slam_toolbox gates scan processing on
        minimum_travel_distance (0.03 m), so a stationary car publishes no
        pose however long we listen. See the module docstring.

        The map is the right standstill proxy and map -> odom is not: that
        transform comes from ekf_global at 50 Hz whether or not slam_toolbox
        ever produced anything, so its presence says nothing about SLAM. A
        latched OccupancyGrid can only have come from slam_toolbox itself.

        This proves SLAM is ALIVE, not that it is producing poses for us --
        that is what the nudge below is for, and the two together are what the
        single pose check was pretending to be.
        """
        if not self._map_seen:
            self.get_logger().error(
                f'no map received on {self.map_topic} during preflight. slam_toolbox '
                'is not running, has not built a map yet, or is not on this discovery '
                'graph. This node fits against /slam/pose only (see the module '
                'docstring on why the EKF topics are not an acceptable substitute), '
                'so without SLAM there is nothing to fit against. Refusing to run.')
            return False
        width, height, resolution = self._map_info
        self.get_logger().info(
            f'{self.map_topic} received: {width}x{height} cells at {resolution:.3f} m '
            '-- slam_toolbox is up and holds a map.')
        return True

    def _nudge_and_confirm_pose(self):
        """NUDGE: creep forward past slam_toolbox's distance gate, then require
        a /slam/pose. Refuse if none arrives.

        Why this and not a topic check. A publisher count on /slam/pose proves
        a node opened a publisher; it does not prove the car can move, that
        our mux lane wins, that the VESC is armed, that the wheels turn, that
        wheel odometry advances, or that the scan match converges. All six
        have to hold before the S-curve means anything, and every one of them
        has failed on this stack at least once. The nudge exercises the whole
        chain end to end for ~0.18 m instead of discovering it 3.6 m into an
        open-loop drive.

        Bounded three ways, because the distance bound alone is a wish -- it
        assumes the car moves at the speed we asked for:
          - distance: nudge_distance_m, via the commanded speed;
          - time: that distance's nominal duration plus a 50% margin, which is
            what actually holds if the wheels slip or the VESC ignores us;
          - front clearance, if a reading exists (it is the same abort the
            drive phase uses; the nudge moves, so it applies here too).

        Ctrl-C during the nudge is handled by main()'s SIGINT/SIGTERM handler,
        which is installed BEFORE preflight is entered: it publishes zeros and
        sets self.done, and the loop below checks self.done every iteration
        and stops.
        """
        if self.nudge_distance_m <= 0.0 or self.nudge_speed_mps <= 0.0:
            self.get_logger().error(
                f'nudge_distance_m ({self.nudge_distance_m}) and nudge_speed_mps '
                f'({self.nudge_speed_mps}) must both be positive -- the nudge is how '
                'this tool proves SLAM will produce poses for it. Refusing to run.')
            return False

        nominal = self.nudge_distance_m / self.nudge_speed_mps
        deadline = time.monotonic() + 1.5 * nominal
        baseline = self._pose_count
        period = 1.0 / max(self.publish_rate_hz, 1.0)

        self.get_logger().warning(
            f'PREFLIGHT NUDGE -- the car is about to MOVE {self.nudge_distance_m:.2f} m '
            f'forward at {self.nudge_speed_mps:.2f} m/s (~{nominal:.1f} s, hard-stopped '
            f'at {1.5 * nominal:.1f} s), steering centred. This is the only way to get a '
            '/slam/pose out of a parked car: slam_toolbox will not process a scan until '
            'the car has travelled its 0.03 m minimum_travel_distance. Keep a hand on '
            'the joystick.')

        self._phase = 'nudge'
        moved = False
        try:
            while rclpy.ok() and not self.done and time.monotonic() < deadline:
                if not self._nudge_clearance_ok():
                    return False
                self._publish_drive(0.0, self.nudge_speed_mps)
                moved = True
                rclpy.spin_once(self, timeout_sec=period)
                if self._pose_count > baseline:
                    break
        finally:
            # Unconditional: a return, a raised exception and a normal finish
            # all leave the car commanded to zero before anything else runs.
            self.publish_stop()
            self._phase = 'preflight'

        if self.done or not rclpy.ok():
            self.get_logger().error('nudge interrupted -- car stopped, refusing to run.')
            return False
        if not moved:
            self.get_logger().error('nudge issued no command at all. Refusing to run.')
            return False

        # Keep waiting with the car STOPPED: the scan match lags the motion and
        # /slam/pose only runs at ~2 Hz, so the pose that the nudge earned
        # usually lands after the creep has already finished.
        wait_until = time.monotonic() + self.nudge_pose_timeout_sec
        while (rclpy.ok() and not self.done and self._pose_count <= baseline
               and time.monotonic() < wait_until):
            self.publish_stop(repeat=1)
            rclpy.spin_once(self, timeout_sec=0.1)

        if self._pose_count <= baseline:
            self.get_logger().error(
                f'nudged {self.nudge_distance_m:.2f} m and waited a further '
                f'{self.nudge_pose_timeout_sec:.1f} s, but no pose arrived on '
                f'{self.pose_topic}. Something between the mux lane and the scan match '
                'is broken: check that the car actually moved (if it did not, the mux, '
                'the VESC or the motor is the problem), then that slam_toolbox is '
                'matching scans. Refusing to drive the profile blind.')
            self.publish_stop()
            return False

        self.get_logger().info(
            f'nudge confirmed: {self._pose_count - baseline} pose(s) on '
            f'{self.pose_topic} after moving. The full chain (mux lane -> VESC -> '
            'wheels -> odometry -> scan match -> pose) is live.')
        return True

    def _nudge_clearance_ok(self):
        """Front-clearance abort during the nudge. Same threshold the drive
        phase uses -- the nudge moves the car, so the reason for the check is
        the same. Deliberately NOT the full _safety_ok(): the lateral-excursion
        arm of that needs a repetition origin that does not exist yet, and the
        hard timeout is about the drive profile, not preflight."""
        if self._clearance is None:
            return True
        if self._clearance < self.min_front_clearance_m:
            self.get_logger().error(
                f'front clearance {self._clearance:.2f} m < '
                f'{self.min_front_clearance_m:.2f} m during the preflight nudge. '
                'Stopping and refusing to run.')
            return False
        return True

    def run_static_preflight(self):
        """Mode A preflight: only the checks that actually apply to a
        stationary sweep. The servo still moves, so a live mission is still
        refused, and the discovery guard still runs so the node cannot sit
        invisible. There is no drive lane, no e-stop path and no SLAM pose to
        require."""
        if not self._discovery_guard():
            return False
        deadline = time.monotonic() + self.preflight_timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._mission_seen:
                break
        checks = (
            self._participants_visible(),
            self._check_sweep_limits(),
            self._check_mission_idle(),
        )
        return all(checks)

    def _check_sweep_limits(self):
        """The sweep amplitude must fit this car's asymmetric steering range,
        same check as the drive amplitude but against the sweep's own knob."""
        if self.sweep_amplitude_rad <= 0.0:
            self.get_logger().error('sweep_amplitude_rad must be positive.')
            return False
        if (self.sweep_amplitude_rad > self.max_steering_angle or
                -self.sweep_amplitude_rad < self.min_steering_angle):
            self.get_logger().error(
                f'sweep amplitude +/-{math.degrees(self.sweep_amplitude_rad):.2f} deg '
                f'does not fit this car\'s range '
                f'[{math.degrees(self.min_steering_angle):.2f}, '
                f'{math.degrees(self.max_steering_angle):.2f}] deg (asymmetric). '
                'Refusing to run.')
            return False
        self.get_logger().info(
            f'sweep amplitude +/-{math.degrees(self.sweep_amplitude_rad):.2f} deg is '
            'within this car\'s steering range.')
        return True

    def run_preflight(self):
        """Mode B preflight, in two stages.

        Stage 1 is every refusal that can be decided while the car is
        stationary, and none of them may depend on motion having happened --
        that was the bug this ordering fixes. All of stage 1 is evaluated
        before anything is commanded.

        Stage 2 is the nudge, and it runs ONLY after stage 1 has passed
        completely: it moves the car, so it must never be the thing that
        discovers a live mission, a missing mux lane or an over-range
        amplitude. Note the checks tuple is built eagerly (not short-circuited
        with `and`) so the operator sees every stage-1 problem in one pass
        rather than fixing them one restart at a time -- but the nudge is
        strictly gated behind all of them.
        """
        if not self._discovery_guard():
            return False
        deadline = time.monotonic() + self.preflight_timeout_sec
        self.get_logger().info(
            f'preflight: listening for {self.preflight_timeout_sec:.0f} s for map, '
            'mission status and clearance... (NOT for a pose -- see the module '
            'docstring: a parked car cannot produce one)')
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._map_seen and self._mission_seen and self._clearance is not None:
                break
        checks = (
            self._participants_visible(),
            self._check_steering_limits(),
            self._check_segment_plan(),
            self._check_mission_idle(),
            self._check_estop_path(),
            self._check_slam_map_ready(),
        )
        if not all(checks):
            return False
        return self._nudge_and_confirm_pose()

    # ----------------------------------------------------------- drive logic
    def _segment_spec(self, rep, seg):
        length_key, sign = self.segment_plan[seg]
        length = getattr(self, length_key)
        # Alternate the starting sign each repetition, so the fit sees both
        # signs even if a repetition is later discarded -- this is what keeps
        # the conditioning gate satisfiable.
        rep_sign = 1.0 if rep % 2 == 0 else -1.0
        return length, sign * rep_sign * self.amplitude_rad

    def _segment_duration(self, length):
        return length / max(self.speed_mps, 1e-3)

    def _begin_segment(self):
        self._seg_started = time.monotonic()
        self._seg_poses = []
        length, delta = self._segment_spec(self._rep, self._seg)
        self.get_logger().info(
            f'rep {self._rep} seg {self._seg}: delta = {math.degrees(delta):+.2f} deg '
            f'for {length:.2f} m ({self._segment_duration(length):.2f} s)')

    def _finish_segment(self):
        length, delta = self._segment_spec(self._rep, self._seg)
        poses = self._seg_poses
        self._all_seg_poses.append(list(poses))
        if len(poses) >= 2:
            dpsi = wrap_angle(poses[-1][3] - poses[0][3])
            # Path length as the sum of consecutive SLAM fixes, not the chord:
            # the segments are arcs, and a chord underestimates a 20 deg arc by
            # ~0.5%. Summing does inflate slightly with pose noise (~1 cm of
            # noise on 0.125 m steps), so the chord is recorded alongside it in
            # the raw dump for an offline sensitivity check.
            ds = sum(math.hypot(poses[i + 1][1] - poses[i][1], poses[i + 1][2] - poses[i][2])
                     for i in range(len(poses) - 1))
            chord = math.hypot(poses[-1][1] - poses[0][1], poses[-1][2] - poses[0][2])
            sample = fitlib.Sample(
                repetition=self._rep, segment=self._seg, delta_cmd=delta, dpsi=dpsi,
                ds=ds, n_pose=len(poses), t_start=poses[0][0], t_end=poses[-1][0],
                ds_chord=chord)
            self.samples.append(sample)
            self.get_logger().info(
                f'  -> dpsi = {math.degrees(dpsi):+.3f} deg over ds = {ds:.3f} m '
                f'from {len(poses)} pose fix(es)')
        else:
            self.get_logger().warning(
                f'  -> rep {self._rep} seg {self._seg} produced only {len(poses)} pose '
                'fix(es) after the settling discard -- no usable sample. This segment '
                'is dropped and will show up in the pose-support gate.')
            self.samples.append(fitlib.Sample(
                repetition=self._rep, segment=self._seg, delta_cmd=delta, dpsi=0.0,
                ds=0.0, n_pose=len(poses), ds_chord=0.0))

    def _safety_ok(self):
        elapsed = time.monotonic() - self._started_monotonic
        if elapsed > self.hard_timeout_sec:
            self.get_logger().error(
                f'hard timeout: {elapsed:.0f} s > {self.hard_timeout_sec:.0f} s. '
                'Stopping.')
            return False
        if self._clearance is not None:
            stale = time.monotonic() - (self._clearance_stamp or 0.0)
            if stale > self.clearance_stale_sec:
                self.get_logger().error(
                    f'{self.clearance_topic} has not published for {stale:.1f} s '
                    '-- the front-clearance abort is blind. Stopping.')
                return False
            if self._clearance < self.min_front_clearance_m:
                self.get_logger().error(
                    f'front clearance {self._clearance:.2f} m < '
                    f'{self.min_front_clearance_m:.2f} m. Stopping.')
                return False
        if self._rep_origin is not None and self._pose is not None:
            ox, oy, oyaw = self._rep_origin[1], self._rep_origin[2], self._rep_origin[3]
            dx, dy = self._pose[1] - ox, self._pose[2] - oy
            # Perpendicular distance from the line this repetition started on.
            lateral = abs(-math.sin(oyaw) * dx + math.cos(oyaw) * dy)
            if lateral > self.max_lateral_excursion_m:
                self.get_logger().error(
                    f'lateral excursion {lateral:.2f} m > '
                    f'{self.max_lateral_excursion_m:.2f} m from this repetition\'s '
                    'start line. Stopping.')
                return False
        return True

    def _tick(self):
        """Timer body: one drive command per tick, plus phase advancement."""
        # 'nudge' is included deliberately: during the preflight nudge the
        # command is published by _nudge_and_confirm_pose's own loop, and this
        # timer must not also drive or advance the segment plan.
        if self.done or self._phase in ('preflight', 'nudge', 'finished'):
            return
        if not self._safety_ok():
            self.publish_stop()
            self.exit_code = EXIT_ABORTED_ON_SAFETY
            self._phase = 'finished'
            self.done = True
            return

        if self._phase == 'pausing':
            self._publish_drive(0.0, 0.0)
            if time.monotonic() >= self._pause_until:
                self._phase = 'driving'
                self._rep_origin = self._pose
                self._begin_segment()
            return

        if self._phase != 'driving':
            return

        length, delta = self._segment_spec(self._rep, self._seg)
        self._publish_drive(delta, self.speed_mps)
        if time.monotonic() - self._seg_started < self._segment_duration(length):
            return

        self._finish_segment()
        self._seg += 1
        if self._seg < len(self.segment_plan):
            self._begin_segment()
            return

        self._seg = 0
        self._rep += 1
        if self._rep >= self.repetitions:
            self.publish_stop()
            self._phase = 'finished'
            self.get_logger().info('drive complete -- fitting.')
            self.finish()
            return

        self.publish_stop()
        self._phase = 'pausing'
        self._pause_until = time.monotonic() + self.inter_rep_pause_sec
        self.get_logger().info(
            f'--- repetition {self._rep - 1} done. Pausing {self.inter_rep_pause_sec:.0f} s. '
            'Reposition the car to a clear start if you do not have room to continue '
            'straight ahead. ---')

    def start_driving(self):
        self._phase = 'driving'
        self._rep_origin = self._pose
        self._started_monotonic = time.monotonic()
        self._begin_segment()

    # ------------------------------------------------------------------ fit
    def _dump_raw_samples(self):
        """Always, on every run including refused ones -- so the fit can be
        redone offline without re-driving."""
        for directory in (self.raw_dump_dir, tempfile.gettempdir()):
            try:
                os.makedirs(directory, exist_ok=True)
                path = os.path.join(directory, f'{self.run_id}.csv')
                fitlib.write_samples_csv(path, self.samples)
                self.get_logger().info(f'raw samples written to {path}')
                self.get_logger().info(
                    f're-fit offline with: python3 -m '
                    f'f1tenth_diagnostics.steering_offset_fit {path}')
                return path
            except OSError as exc:
                # Falling back to a temp dir rather than giving up: by the time
                # this runs the car has already driven the whole profile, and
                # an unwritable raw_dump_dir (a typo, a read-only path) would
                # otherwise throw away the only record of a run that takes
                # ~50 s of supervised driving to reproduce. Hit for real during
                # smoke testing, with raw_dump_dir accidentally set to '/dump'.
                self.get_logger().error(
                    f'could not write raw samples to {directory}: {exc}')
        return None

    def _new_gain_and_offset(self, fitted_gain, delta0, gain_current, offset_current):
        """Compose the fitted (gain, offset) error into new stack constants.

        VERIFIED AGAINST THE REAL SOURCE, not assumed. ackermann_to_vesc.cpp
        line 142 is literally

            servo = steering_to_servo_gain * cmd.steering_angle + offset

        and vesc_to_odom_backup.cpp line 145 inverts exactly that. So the
        stack maps servo = offset + gain*theta, as the derivation below
        assumes.

        The fit gives the composed relation delta_actual = g*theta + delta0.
        The servo-to-wheel relation is fixed by hardware, so with the CURRENT
        constants a servo value corresponds to theta = (servo - off_c)/gain_c
        and therefore to a real wheel angle

            delta_actual = g*(servo - off_c)/gain_c + delta0.

        We want new constants for which commanding theta yields
        delta_actual = theta. Substituting servo_new = gain_n*theta + off_n:

            g*gain_n/gain_c * theta + g*(off_n - off_c)/gain_c + delta0 = theta

        Matching the theta coefficient and the constant term separately:

            gain_new   = gain_current / g
            offset_new = offset_current - delta0 * gain_new

        which is what the work order specifies. Sanity check at g = 1:
        gain_new = gain_current and offset_new = offset_current - gain*delta0
        -- exactly the formula this node used before gain was fitted, so the
        change is a strict generalisation. That reduction is asserted in the
        tests rather than left as a comment.
        """
        if abs(fitted_gain) < 1e-6:
            raise ValueError(f'fitted gain {fitted_gain} is ~0; cannot invert it')
        gain_new = gain_current / fitted_gain
        offset_new = offset_current - delta0 * gain_new
        return gain_new, offset_new

    def _servo_extremes(self, gain_new, offset_new):
        """servo at both steering extremes under the NEW constants.

        Changing the gain rescales the whole steering range, so checking the
        offset alone is not enough any more: a wrong gain can push the servo
        past its mechanical stop at one end while the centre still looks fine.
        Both ends of this car's ASYMMETRIC range (-0.264 / +0.314 rad) are
        evaluated.
        """
        at_min = gain_new * self.min_steering_angle + offset_new
        at_max = gain_new * self.max_steering_angle + offset_new
        return at_min, at_max

    def _report_wheelbase_discrepancy(self):
        """Report -- never change -- the wheelbase currently in vesc.yaml.

        With use_servo_cmd_to_calc_angular_velocity: true, that value is
        already computing /odom yaw (vesc_to_odom_backup.cpp line 147:
        angular_velocity = speed * tan(steering) / wheelbase), so a
        discrepancy against the pinned value is a free datapoint on the
        long-standing odom-vs-SLAM yaw scale error rather than a bookkeeping
        detail.
        """
        pinned = self.pinned_wheelbase_m
        current = self.vesc_yaml_wheelbase_m
        ratio = pinned / current if current else float('inf')
        line = (f'wheelbase: pinned {pinned:.4f} m (F1TENTH spec, ASSUMED NOT MEASURED) '
                f'vs vesc.yaml vesc_to_odom_node.wheelbase {current:.4f} m -- '
                f'discrepancy {100.0 * (pinned - current) / current:+.1f}% '
                f'(ratio {ratio:.4f}). NOT changed by this node. With '
                'use_servo_cmd_to_calc_angular_velocity: true that value already '
                'scales /odom yaw directly, so this is a free datapoint on the '
                'odom-vs-SLAM yaw scale error.')
        self.get_logger().warning(line)
        return line

    def _read_yaml_value(self, path, section, key):
        yaml = YAML()
        yaml.preserve_quotes = True
        with open(path, 'r') as handle:
            data = yaml.load(handle)
        return float(data[section]['ros__parameters'][key])

    def _publish_diagnostics(self, level, message, values):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = f'{TOOL_NAME}: steering offset calibration'
        status.hardware_id = self.run_id
        status.message = message
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values.items()]
        array.status.append(status)
        self.diag_pub.publish(array)

    def finish(self):
        """Mode B: fit, gate, cross-check, and (only if every gate passes) write."""
        dump_path = self._dump_raw_samples()
        usable = [s for s in self.samples if s.ds > 1e-6 and s.n_pose >= 2]

        if len(usable) < 3:
            self.get_logger().error(
                f'only {len(usable)} usable segment(s) of {len(self.samples)} driven -- '
                'cannot fit two parameters. Nothing written.')
            self._publish_diagnostics(
                DiagnosticStatus.ERROR, 'insufficient segments -- nothing written',
                {'run_id': self.run_id, 'usable_segments': len(usable),
                 'raw_samples': dump_path or 'not written'})
            self.exit_code = EXIT_INSUFFICIENT_SAMPLES
            self.done = True
            return

        fit = fitlib.fit_gain_offset(usable, wheelbase=self.pinned_wheelbase_m)
        self.get_logger().info(f'FIT (mode B, drive): {fit.summary()}')
        wheelbase_note = self._report_wheelbase_discrepancy()

        # Independent, L-free cross-check on the offset -- reported, never used
        # to refuse. See fitlib.TurningPoint for why each carries an interval.
        turning = fitlib.turning_points_agree(self._turning_points(fit.gain))
        self.get_logger().info(f'[diagnostic] turning_points: {turning.detail}')

        gates = [
            fitlib.check_conditioning(usable, fit,
                                      max_correlation=self.max_param_correlation),
            fitlib.check_repetition_agreement(usable, fit.gain, fit.wheelbase,
                                              min_repetitions=min(self.repetitions, 3)),
            fitlib.check_pose_support(usable, self.min_pose_samples_per_segment),
            self._plausibility_gate(fit),
        ]
        for gate in gates:
            # Two explicit call sites, not `level = ...info if passed else ...error`.
            # rclpy caches a logger per CALLER LOCATION and raises "Logger
            # severity cannot be changed between calls" if one line logs at two
            # severities -- which crashes the node the first time gates come
            # back mixed, i.e. the ordinary case this block exists to report.
            if gate.passed:
                self.get_logger().info(f'[PASS] {gate.name}: {gate.detail}')
            else:
                self.get_logger().error(f'[REFUSE] {gate.name}: {gate.detail}')

        values = {
            'run_id': self.run_id,
            'mode': 'B (drive)',
            'gain': f'{fit.gain:.5f}',
            'gain_ci95': f'{fit.ci_gain:.5f}',
            'delta0_deg': f'{math.degrees(fit.delta0):+.4f}',
            'delta0_ci95_deg': f'{math.degrees(fit.ci_delta0):.4f}',
            'wheelbase_pinned_m': f'{fit.wheelbase:.4f}',
            'wheelbase_note': wheelbase_note,
            'residual_rms_rad': f'{fit.residual_rms:.6f}',
            'param_correlation': f'{fit.correlation:+.4f}',
            'segments': fit.n_samples,
            'repetitions': self.repetitions,
            'amplitude_deg': f'{math.degrees(self.amplitude_rad):.2f}',
            'turning_points': turning.detail,
            'raw_samples': dump_path or 'not written',
        }
        for gate in gates:
            values[f'gate_{gate.name}'] = 'PASS' if gate.passed else 'REFUSE'

        self._conclude(fit, gates, values, dump_path)

    def _turning_points(self, gain):
        """Turning-point offsets from the poses collected during the drive."""
        poses = [p for seg in self._all_seg_poses for p in seg]
        if len(poses) < 3:
            return []
        times = [p[0] for p in poses]
        yaws = [p[3] for p in poses]
        cmds = sorted(self._command_history)
        if not cmds:
            return []
        ct = [c[0] for c in cmds]
        cv = [c[1] for c in cmds]

        def command_at(t):
            return float(np.interp(t, ct, cv))

        return fitlib.turning_point_estimates(
            times, yaws, command_at, sigma_t=self.turning_point_sigma_t, gain=gain)

    def _conclude(self, fit, gates, values, dump_path):
        """Shared tail for both modes: refuse loudly, or write."""
        failed = [g for g in gates if not g.passed]
        if failed:
            names = ', '.join(g.name for g in failed)
            self.get_logger().error(
                f'REFUSING TO WRITE -- failed gate(s): {names}. The numbers above are '
                'still real and the raw samples are on disk; nothing in the config was '
                'changed.')
            self._publish_diagnostics(
                DiagnosticStatus.WARN, f'calibration refused: {names}', values)
            self.exit_code = EXIT_GATES_REFUSED
            self.done = True
            return

        if not self.write_enabled:
            self.get_logger().warning(
                'all gates passed, but write_enabled is false -- nothing written. '
                'Re-run with write_enabled:=true to apply.')
            self._publish_diagnostics(
                DiagnosticStatus.OK, 'calibration passed (dry run, nothing written)',
                values)
            self.done = True
            return

        self._write_back(fit, values)
        self.done = True

    def _plausibility_gate(self, fit):
        """Absolute bounds on both fitted values. Cheap, and it catches the case
        where every statistical gate passes because the data is internally
        consistent but physically absurd (a mounting error, a sign flip, a
        mis-set servo range)."""
        problems = []
        if abs(fit.delta0) > self.delta0_absolute_bound_rad:
            problems.append(
                f'|delta0| = {math.degrees(fit.delta0):.3f} deg exceeds '
                f'{math.degrees(self.delta0_absolute_bound_rad):.3f} deg -- that is a '
                'mechanical fault, not a trim value')
        if not (self.gain_min <= fit.gain <= self.gain_max):
            problems.append(
                f'gain = {fit.gain:.4f} outside [{self.gain_min:.2f}, '
                f'{self.gain_max:.2f}] -- the commanded-to-actual scale cannot '
                'plausibly be this far off')
        detail = (f'delta0 = {math.degrees(fit.delta0):+.3f} deg '
                  f'(bound +/-{math.degrees(self.delta0_absolute_bound_rad):.2f}); '
                  f'gain = {fit.gain:.4f} '
                  f'(bounds [{self.gain_min:.2f}, {self.gain_max:.2f}])')
        if problems:
            detail += ' || ' + ' ; '.join(problems)
        return fitlib.GateResult('plausibility', not problems, detail)

    def _write_back(self, fit, values):
        if YAML is None:
            self.get_logger().error(
                'ruamel.yaml is not installed -- the fit above is good but cannot be '
                'written. apt install python3-ruamel.yaml, then apply by hand or re-run.')
            self._publish_diagnostics(
                DiagnosticStatus.ERROR, 'ruamel.yaml missing -- nothing written', values)
            self.exit_code = EXIT_MISSING_DEPENDENCY
            return

        try:
            gain_left = self._read_yaml_value(
                self.steering_calibration_yaml_path, '/**',
                'steering_angle_to_servo_gain_left')
            gain_right = self._read_yaml_value(
                self.steering_calibration_yaml_path, '/**',
                'steering_angle_to_servo_gain_right')
            old_offset = self._read_yaml_value(
                self.steering_calibration_yaml_path, '/**',
                'steering_angle_to_servo_offset')
            servo_min = self._read_yaml_value(
                self.steering_calibration_yaml_path, '/**', 'servo_min')
            servo_max = self._read_yaml_value(
                self.steering_calibration_yaml_path, '/**', 'servo_max')
        except (OSError, KeyError, ValueError) as exc:
            self.get_logger().error(
                f'could not read the current config values back: {exc}. Nothing written.')
            self._publish_diagnostics(
                DiagnosticStatus.ERROR, 'config read failed -- nothing written', values)
            self.exit_code = EXIT_SANITY_VIOLATION
            return

        # ackermann_to_vesc picks the gain by the SIGN OF THE COMMAND, so
        # left/right are genuinely independent constants. This fit produces one
        # composed gain error, which applies to both; they are rescaled
        # separately so an existing left/right asymmetry is preserved rather
        # than flattened.
        try:
            new_gain_left, new_offset = self._new_gain_and_offset(
                fit.gain, fit.delta0, gain_left, old_offset)
            new_gain_right, new_offset_r = self._new_gain_and_offset(
                fit.gain, fit.delta0, gain_right, old_offset)
        except ValueError as exc:
            self.get_logger().error(f'{exc}. Nothing written.')
            self._publish_diagnostics(
                DiagnosticStatus.ERROR, 'degenerate gain -- nothing written', values)
            self.exit_code = EXIT_SANITY_VIOLATION
            return

        if abs(new_offset - new_offset_r) > 1e-9:
            # Only possible if left/right gains already differ; a single shared
            # offset key cannot express two different corrections.
            self.get_logger().warning(
                f'left and right gains differ ({gain_left} vs {gain_right}), so the '
                f'offset correction differs per side ({new_offset:.6f} vs '
                f'{new_offset_r:.6f}). steering_angle_to_servo_offset is a single '
                f'shared key, so the left-side value is used. Tune the sides '
                'separately with vesc_tuning\'s steering_calibration_node if this '
                'matters.')

        # Bounds are safety, not bookkeeping: a changed gain rescales the whole
        # range, so BOTH extremes are re-checked under the new constants.
        at_min_l, at_max_l = self._servo_extremes(new_gain_left, new_offset)
        at_min_r, at_max_r = self._servo_extremes(new_gain_right, new_offset)
        violations = []
        for label, value in (('min_steering_angle (left gain)', at_min_l),
                             ('max_steering_angle (left gain)', at_max_l),
                             ('min_steering_angle (right gain)', at_min_r),
                             ('max_steering_angle (right gain)', at_max_r)):
            if value < servo_min:
                violations.append(
                    f'{label}: servo {value:.6f} is {servo_min - value:.6f} BELOW '
                    f'servo_min {servo_min}')
            elif value > servo_max:
                violations.append(
                    f'{label}: servo {value:.6f} is {value - servo_max:.6f} ABOVE '
                    f'servo_max {servo_max}')
        if violations:
            self.get_logger().error(
                'the new constants would command the servo past its mechanical stop: '
                + ' ; '.join(violations) +
                '. Nothing written -- applying this could physically damage the '
                'steering linkage.')
            self._publish_diagnostics(
                DiagnosticStatus.ERROR, 'servo range violated -- nothing written', values)
            self.exit_code = EXIT_SANITY_VIOLATION
            return
        self.get_logger().info(
            f'servo range check: [{min(at_min_l, at_max_l):.4f}, '
            f'{max(at_min_l, at_max_l):.4f}] within [{servo_min}, {servo_max}].')

        stamp = datetime.now().strftime('%Y-%m-%dT%H:%M:%S')
        provenance = [
            f'run id     : {self.run_id}   written {stamp}',
            f'fitted by  : {TOOL_NAME}, mode {values.get("mode")}',
            f'gain       : {fit.gain:.5f} +/- {fit.ci_gain:.5f} (95% CI)',
            f'delta0     : {math.degrees(fit.delta0):+.4f} deg '
            f'+/- {math.degrees(fit.ci_delta0):.4f} deg (95% CI)',
            f'residual   : {fit.residual_rms:.6f} rad RMS over {fit.n_samples} points',
            '',
            f'WHEELBASE PINNED AT {self.pinned_wheelbase_m:.4f} m -- ASSUMED FROM THE',
            'F1TENTH SPEC (lf 0.15875 + lr 0.17145 = 13 inches), NOT MEASURED ON THIS',
            'CAR. Published F1TENTH figures disagree: another parameter set gives',
            'lf 0.128 / lr 0.137 = 0.265 m citing Traxxas Slash 4x4 dimensions. 0.3302',
            'is a deliberate choice, not a lookup. L enters the fit as a divisor, so an',
            'error in it becomes a PROPORTIONAL error in the gain above -- the same',
            'order as the effect being measured. If this gain later looks strange,',
            'this assumption is the first thing to re-examine.',
            '',
            f'{values.get("wheelbase_note", "")}',
            '',
            'DOWNSTREAM: vesc_to_odom runs with use_servo_cmd_to_calc_angular_velocity',
            'true, so /odom yaw is computed from the servo command through these same',
            'constants -- changing the gain changes /odom yaw directly. That is likely',
            'the fix for the long-standing ~30% odom-vs-SLAM yaw scale error, but it',
            'also means the ekf_global Q retune (0.0243/0.0273/0.0123) was fitted under',
            'the OLD constants and may need re-deriving after this change.',
            '',
            'converted  : gain_new = gain_current / g ; '
            'offset_new = offset_current - delta0 * gain_new',
            f'             gain_left  {gain_left:.6f} -> {new_gain_left:.6f}',
            f'             gain_right {gain_right:.6f} -> {new_gain_right:.6f}',
            f'             offset     {old_offset:.6f} -> {new_offset:.6f}',
            f'raw samples: {values.get("raw_samples")}',
        ]
        if values.get('backlash_deg'):
            provenance.insert(5, f'backlash   : {values["backlash_deg"]} deg (measured '
                                 'directly from the two sweep branches)')

        try:
            backup, diff = write_yaml_config_with_provenance(
                self.steering_calibration_yaml_path,
                {'/**': {
                    'steering_angle_to_servo_gain_left': float(round(new_gain_left, 6)),
                    'steering_angle_to_servo_gain_right': float(round(new_gain_right, 6)),
                    'steering_angle_to_servo_offset': float(round(new_offset, 6)),
                }},
                provenance, self.get_logger(), tool=TOOL_NAME)
        except (OSError, KeyError, RuntimeError) as exc:
            self.get_logger().error(f'write failed: {exc}. Nothing else was written.')
            self._publish_diagnostics(
                DiagnosticStatus.ERROR, f'write failed: {exc}', values)
            self.exit_code = EXIT_SANITY_VIOLATION
            return

        path = self.steering_calibration_yaml_path
        self.get_logger().info(f'--- diff: {path} (backup: {backup}) ---')
        for line in diff.splitlines():
            self.get_logger().info(f'  {line}')

        values['steering_angle_to_servo_gain_left'] = f'{gain_left:.6f} -> {new_gain_left:.6f}'
        values['steering_angle_to_servo_gain_right'] = (
            f'{gain_right:.6f} -> {new_gain_right:.6f}')
        values['steering_angle_to_servo_offset'] = f'{old_offset:.6f} -> {new_offset:.6f}'
        self._publish_diagnostics(
            DiagnosticStatus.OK,
            f'calibration written: gain {fit.gain:.4f}, '
            f'delta0 {math.degrees(fit.delta0):+.3f} deg', values)

        self.get_logger().warning(
            f'WRITTEN TO {path} -- REQUIRES A RELAUNCH OF THE VESC NODES TO TAKE '
            'EFFECT. ackermann_to_vesc_node and vesc_driver_node read their parameters '
            'once at startup and register no set-parameters callback, so `ros2 param '
            'set` is a no-op here. This workspace is also NOT built with '
            '--symlink-install, so `colcon build` must copy this file into install/ '
            'before the relaunch will see it.')
        self.get_logger().warning(
            'DOWNSTREAM: /odom yaw is computed from the servo command through these '
            'constants (use_servo_cmd_to_calc_angular_velocity: true), so this change '
            'alters /odom yaw directly. It is likely the fix for the ~30% '
            'odom-vs-SLAM yaw scale error -- but the ekf_global Q retune '
            '(0.0243/0.0273/0.0123) was fitted under the OLD constants and may need '
            're-deriving. NOT done here.')
        self.get_logger().warning(
            'This file will now appear in `git status` alongside the entries already '
            'dirty on this branch. Suggested commit:')
        self.get_logger().warning(
            f'  git add {path} && git commit -m '
            f'"steering calibration: gain {fit.gain:.4f}, delta0 '
            f'{math.degrees(fit.delta0):+.3f} deg (run {self.run_id})"')
        self.exit_code = EXIT_SUCCESS

    # ------------------------------------------------------- mode A (static)
    def run_static_sweep(self, prompt=input):
        """Mode A: command a steering sequence with the car stationary and ask
        the operator for the measured wheel angle at each point.

        Every angle is approached from BOTH directions -- a rising sweep then a
        falling one -- because the gap between the branches IS the backlash,
        and a one-directional sweep cannot tell backlash from measurement
        noise. That matters here specifically: the two turning-point estimates
        from run 2026-09-08T12-27-37 (+4.36 deg at t=5.26, -0.31 deg at
        t=8.74) are opposite-direction transitions, which is exactly the
        signature hysteresis produces.

        `ros2 run` ONLY, never `ros2 launch`. This blocks on a real input(),
        and `ros2 launch` does not reliably forward stdin to a launched node --
        the same trap that made sensor_covariance_calibration_node's
        light_motion mode hang forever with no output and no way to respond
        (see calibration.launch.py's own note). The launch file deliberately
        does not expose this mode.
        """
        amplitude = self.sweep_amplitude_rad
        steps = max(self.sweep_steps, 3)
        rising = [(-amplitude + 2.0 * amplitude * i / (steps - 1)) for i in range(steps)]
        sequence = [(c, +1) for c in rising] + [(c, -1) for c in reversed(rising)]

        self.get_logger().warning(
            'MODE A -- STATIC SWEEP. The car must be STATIONARY and SAFELY SUPPORTED '
            '(wheels clear of the ground). No driving happens, but the steering servo '
            'WILL move. Measure the wheel angle with a protractor or phone '
            'inclinometer at each step, positive = left.')
        self.get_logger().warning(
            f'{len(sequence)} points: a rising sweep then a falling sweep over '
            f'+/-{math.degrees(amplitude):.1f} deg. Enter each measurement in DEGREES, '
            'or "s" to skip a point, or "q" to abort.')

        for index, (command, direction) in enumerate(sequence):
            self._publish_drive(command, 0.0)
            # Let the servo settle and, critically, let it arrive from the
            # direction this branch is sweeping -- the measurement is only
            # meaningful if the approach direction matches the label.
            end = time.monotonic() + self.settle_sec + 0.6
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            label = 'rising' if direction > 0 else 'falling'
            try:
                raw = prompt(
                    f'[{index + 1}/{len(sequence)}] {label}: commanded '
                    f'{math.degrees(command):+6.2f} deg -- measured angle (deg)? ')
            except EOFError:
                self.get_logger().error(
                    'stdin closed -- mode A needs an interactive terminal. Run it with '
                    '`ros2 run`, not `ros2 launch`.')
                return False
            raw = (raw or '').strip().lower()
            if raw in ('q', 'quit', 'abort'):
                self.get_logger().warning('aborted by operator.')
                return False
            if raw in ('s', 'skip', ''):
                self.get_logger().info('  skipped.')
                continue
            try:
                measured = math.radians(float(raw))
            except ValueError:
                self.get_logger().warning(f'  could not parse {raw!r} -- skipping.')
                continue
            self.sweep_samples.append(
                fitlib.SweepSample(index, command, measured, direction))

        self.publish_stop()
        return True

    def finish_static(self):
        """Fit, gate and write from a completed static sweep."""
        dump_path = self._dump_sweep_samples()
        if len(self.sweep_samples) < 5:
            self.get_logger().error(
                f'only {len(self.sweep_samples)} usable sweep point(s) -- cannot fit '
                'four parameters. Nothing written.')
            self._publish_diagnostics(
                DiagnosticStatus.ERROR, 'insufficient sweep points -- nothing written',
                {'run_id': self.run_id, 'points': len(self.sweep_samples),
                 'raw_samples': dump_path or 'not written'})
            self.exit_code = EXIT_INSUFFICIENT_SAMPLES
            self.done = True
            return

        static = fitlib.fit_static_sweep(self.sweep_samples)
        self.get_logger().info(f'FIT (mode A, static sweep): {static.summary()}')
        wheelbase_note = self._report_wheelbase_discrepancy()

        if not static.is_linear:
            self.get_logger().error(
                f'the command->angle relation is NOT LINEAR: fitted quadratic term '
                f'{static.curvature:+.4f} +/- {static.ci_curvature:.4f} rad^-1 is '
                'distinguishable from zero. A single gain is a straight line through a '
                'curve here -- reporting it, not applying it.')

        gates = [
            fitlib.check_sweep_span(static, self.min_sweep_span_rad),
            fitlib.check_sweep_branches(static, self.backlash_tolerance_rad),
            self._linearity_gate(static),
        ]
        for gate in gates:
            if gate.passed:
                self.get_logger().info(f'[PASS] {gate.name}: {gate.detail}')
            else:
                self.get_logger().error(f'[REFUSE] {gate.name}: {gate.detail}')

        # Mode A measures delta_actual = g*theta + offset directly, which is
        # the same composed relation mode B fits -- so the same write-back
        # math applies unchanged.
        fit = fitlib.FitResult(
            gain=static.gain, delta0=static.offset,
            ci_gain=static.ci_gain, ci_delta0=static.ci_offset,
            wheelbase=self.pinned_wheelbase_m, residual_rms=static.residual_rms,
            correlation=0.0, n_samples=static.n_samples, converged=True, iterations=1)
        gates.append(self._plausibility_gate(fit))

        values = {
            'run_id': self.run_id,
            'mode': 'A (static sweep)',
            'gain': f'{static.gain:.5f}',
            'gain_ci95': f'{static.ci_gain:.5f}',
            'delta0_deg': f'{math.degrees(static.offset):+.4f}',
            'delta0_ci95_deg': f'{math.degrees(static.ci_offset):.4f}',
            'backlash_deg': f'{math.degrees(static.backlash):.4f}',
            'backlash_ci95_deg': f'{math.degrees(static.ci_backlash):.4f}',
            'linear': str(static.is_linear),
            'curvature_rad_inv': f'{static.curvature:+.5f}',
            'residual_rms_deg': f'{math.degrees(static.residual_rms):.4f}',
            'sweep_points': static.n_samples,
            'sweep_span_deg': f'{math.degrees(static.span):.2f}',
            'wheelbase_pinned_m': f'{self.pinned_wheelbase_m:.4f}',
            'wheelbase_note': wheelbase_note,
            'raw_samples': dump_path or 'not written',
        }
        for gate in gates:
            values[f'gate_{gate.name}'] = 'PASS' if gate.passed else 'REFUSE'

        self._conclude(fit, gates, values, dump_path)

    def _linearity_gate(self, static):
        """Refuse to WRITE a single gain when the relation is not a line.

        Reporting nonlinearity but writing the straight-line gain anyway would
        be the worst of both: the log says the model is wrong and the config
        gets the wrong model regardless."""
        passed = static.is_linear
        detail = (f'quadratic term {static.curvature:+.5f} +/- {static.ci_curvature:.5f} '
                  f'rad^-1; residual RMS {math.degrees(static.residual_rms):.3f} deg')
        if not passed:
            detail += (' -- command->angle is NOT linear, so a single gain does not '
                       'describe this linkage; refusing to write one')
        return fitlib.GateResult('linearity', passed, detail)

    def _dump_sweep_samples(self):
        for directory in (self.raw_dump_dir, tempfile.gettempdir()):
            try:
                os.makedirs(directory, exist_ok=True)
                path = os.path.join(directory, f'{self.run_id}.sweep.csv')
                fitlib.write_sweep_csv(path, self.sweep_samples)
                self.get_logger().info(f'raw sweep written to {path}')
                self.get_logger().info(
                    f're-fit offline with: python3 -m '
                    f'f1tenth_diagnostics.steering_offset_fit {path}')
                return path
            except OSError as exc:
                self.get_logger().error(
                    f'could not write raw sweep to {directory}: {exc}')
        return None


def main():
    rclpy.init()
    node = SteeringOffsetCalibrationNode()

    def _emergency_stop(signum, _frame):
        # Zero the command on Ctrl-C / SIGTERM. This is a manually-run tool
        # that drives the car; leaving a command latched because the operator
        # hit Ctrl-C is the single worst thing it could do.
        node.get_logger().warning(f'signal {signum} -- publishing zero velocity.')
        try:
            node.publish_stop()
        finally:
            node.done = True
            node.exit_code = EXIT_ABORTED_ON_SAFETY

    signal.signal(signal.SIGINT, _emergency_stop)
    signal.signal(signal.SIGTERM, _emergency_stop)

    try:
        if node.calibration_mode == 'static':
            # Mode A drives nothing, so it skips the drive-oriented preflight
            # (mux lane, e-stop, the SLAM readiness check and the nudge) --
            # none of which apply to a stationary sweep read off a protractor.
            # It reads wheel angles off a protractor and touches no SLAM topic
            # at all, and the front-clearance check is equally irrelevant with
            # the car parked and supported. The discovery guard and the
            # mission-active refusal still run: the servo does move, and
            # contending with a live mission is exactly as wrong here.
            if not node.run_static_preflight():
                node.exit_code = EXIT_PREFLIGHT_REFUSED
            elif node.run_static_sweep():
                node.finish_static()
            else:
                node.exit_code = EXIT_ABORTED_ON_SAFETY
        elif not node.run_preflight():
            node.get_logger().error('preflight refused -- nothing was commanded.')
            node.exit_code = EXIT_PREFLIGHT_REFUSED
        else:
            node.get_logger().warning(
                'PREFLIGHT PASSED -- the car will start moving now. Keep a hand on the '
                'joystick (mux priority 100 overrides this node at 50).')
            node.start_driving()
            # Not rclpy.spin(): shutdown from inside a callback deadlocks the
            # executor. Same pattern as the other calibration nodes here.
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.exit_code = EXIT_ABORTED_ON_SAFETY
    except Exception as exc:  # noqa: BLE001 -- see publish_stop below
        # Any unhandled exception must still leave the car stopped. Catching
        # broadly is deliberate here and is the whole point: an exception that
        # escaped to the top of a node holding a live drive lane would
        # otherwise leave the last command latched until the mux timed it out.
        node.get_logger().error(f'unhandled exception: {exc!r}')
        node.exit_code = EXIT_ABORTED_ON_SAFETY
        raise
    finally:
        try:
            node.publish_stop()
            # Give the zeros a moment to actually leave the process before the
            # context is torn down -- destroy_node() drops anything still queued.
            end = time.monotonic() + 0.3
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:  # noqa: BLE001
            pass
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
