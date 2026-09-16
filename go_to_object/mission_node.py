"""rclpy wrapper for the go_to_object mission mode.

Thin by construction: this module owns message plumbing, TF, parameters and
the clock, and nothing else. Estimation lives in :mod:`object_tracker`,
geometry in :mod:`pursuit_geometry`, and the state machine in
:mod:`mission_state` -- all three import without a ROS environment, and all
three are tested without one.

Out of scope on purpose: segmentation, speed control, obstacle avoidance.
The node publishes a curvature and a drive-enable gate; converting curvature
to a steering angle needs the wheelbase and belongs to the consumer.

Ambiguities resolved here, stated rather than picked silently:

* Two detection inputs are offered because both are common upstream shapes:
  a ``geometry_msgs/PointStamped`` centroid, and a
  ``sensor_msgs/PointCloud2`` of the *already segmented* object, of which
  this node takes the arithmetic mean of the points and nothing more. Wire
  up whichever your pipeline produces; leaving both connected is harmless
  but means two detections per frame.
* ``LOST`` is a live state, not a terminal one: it returns to ``ACQUIRE`` as
  soon as the tracker yields an estimate again.
* ``ARRIVED`` latches until the ``~/reset`` service is called.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

import tf2_ros
from tf2_geometry_msgs import do_transform_point  # noqa: F401  (registers types)

from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .config_checks import VehicleLimits, describe, raise_on_errors, validate
from .diagnostics_format import NONE, format_float, format_optional_float
from .mission_state import GoToObjectMission, MissionParams, MissionState
from .object_tracker import ObjectTracker, TrackerParams
from .pursuit_geometry import (
    CurvatureLimiter,
    PursuitParams,
    curvature_from_steering,
)

_WARN_THROTTLE_S = 2.0
_NIS_EXPECTATION = 2.0
_REJECTION_ALARM = 0.5


@dataclass
class _Topics:
    odom: str
    detection_cloud: str
    detection_point: str
    curvature: str
    drive_enable: str
    state: str
    nis: str
    diagnostics: str
    steering: str


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class GoToObjectNode(Node):
    """Plumbing around :class:`~go_to_object.mission_state.GoToObjectMission`."""

    def __init__(self) -> None:
        super().__init__('go_to_object')

        # -- parameters: every constant is declared, none inlined below ----
        # Shadow mode is the default. The node tracks, plans and publishes
        # full diagnostics while publishing ZERO on the command topic, so the
        # first drive can be a manual one past the object with the whole
        # pipeline recorded and nothing actuated. Turn it on deliberately.
        self.declare_parameter('enabled', False)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('reposition_timeout', 2.0)

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('detection_cloud_topic', '~/detection_cloud')
        self.declare_parameter('detection_point_topic', '~/detection_point')
        self.declare_parameter('curvature_topic', '~/curvature')
        self.declare_parameter('drive_enable_topic', '~/drive_enable')
        self.declare_parameter('state_topic', '~/state')
        self.declare_parameter('nis_topic', '~/nis_mean')
        self.declare_parameter('diagnostics_topic', '~/diagnostics')
        self.declare_parameter('steering_topic', '')

        self.declare_parameter('r_min', 1.5)
        self.declare_parameter('d_lookahead_min', 1.0)
        self.declare_parameter('d_stop', 0.5)
        self.declare_parameter('straight_eps', 1e-3)
        self.declare_parameter('max_kappa_rate', 0.5)

        self.declare_parameter('sigma_range_base', 0.05)
        self.declare_parameter('sigma_range_slope', 0.002)
        self.declare_parameter('sigma_bearing', 0.015)
        self.declare_parameter('q_static', 0.01)
        self.declare_parameter('p0', 4.0)
        self.declare_parameter('gate_chi2', 9.21)
        self.declare_parameter('max_rejections', 12)
        self.declare_parameter('max_age', 0.8)
        self.declare_parameter('n_converged', 4)
        self.declare_parameter('odom_horizon', 2.0)
        self.declare_parameter('max_odom_jump', 0.5)
        self.declare_parameter('max_odom_jump_psi', 0.3)
        self.declare_parameter('odom_jump_var_inflation', 0.25)
        self.declare_parameter('nis_window', 50)
        self.declare_parameter('nis_alarm_ratio', 2.0)
        self.declare_parameter('state_timeout', 1.0)
        self.declare_parameter('odom_timeout', 0.2)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('steering_slew_rate', 6.98)
        self.declare_parameter('vehicle_min_turn_radius', 1.0)
        self.declare_parameter('cruise_speed', 2.0)

        gp = self.get_parameter
        self._base_frame = gp('base_frame').value
        self._enabled = bool(gp('enabled').value)
        self._wheelbase = float(gp('wheelbase').value)
        self._nis_alarm = (float(gp('nis_alarm_ratio').value) * _NIS_EXPECTATION)

        self._tracker = ObjectTracker(TrackerParams(
            sigma_range_base=float(gp('sigma_range_base').value),
            sigma_range_slope=float(gp('sigma_range_slope').value),
            sigma_bearing=float(gp('sigma_bearing').value),
            q_static=float(gp('q_static').value),
            p0=float(gp('p0').value),
            gate_chi2=float(gp('gate_chi2').value),
            max_rejections=int(gp('max_rejections').value),
            max_age=float(gp('max_age').value),
            n_converged=int(gp('n_converged').value),
            odom_horizon=float(gp('odom_horizon').value),
            max_odom_jump=float(gp('max_odom_jump').value),
            max_odom_jump_psi=float(gp('max_odom_jump_psi').value),
            odom_jump_var_inflation=float(gp('odom_jump_var_inflation').value),
            nis_window=int(gp('nis_window').value),
            nis_alarm_ratio=float(gp('nis_alarm_ratio').value),
        ))
        self._mission = GoToObjectMission(
            pursuit=PursuitParams(
                r_min=float(gp('r_min').value),
                d_lookahead_min=float(gp('d_lookahead_min').value),
                d_stop=float(gp('d_stop').value),
                straight_eps=float(gp('straight_eps').value),
            ),
            mission=MissionParams(
                confidence_threshold=float(gp('confidence_threshold').value),
                reposition_timeout=float(gp('reposition_timeout').value),
                odom_timeout=float(gp('odom_timeout').value),
                state_timeout=float(gp('state_timeout').value),
            ),
            limiter=CurvatureLimiter(float(gp('max_kappa_rate').value)),
        )

        topics = _Topics(
            odom=gp('odom_topic').value,
            detection_cloud=gp('detection_cloud_topic').value,
            detection_point=gp('detection_point_topic').value,
            curvature=gp('curvature_topic').value,
            drive_enable=gp('drive_enable_topic').value,
            state=gp('state_topic').value,
            nis=gp('nis_topic').value,
            diagnostics=gp('diagnostics_topic').value,
            steering=gp('steering_topic').value,
        )

        # Refuse an unsafe configuration rather than clamp it silently, and
        # log every loaded value: on the first live drive you want to know
        # exactly what was running, not what the defaults say.
        limits = VehicleLimits(
            wheelbase=self._wheelbase,
            steering_slew_rate=float(gp('steering_slew_rate').value),
            min_turn_radius=float(gp('vehicle_min_turn_radius').value),
            cruise_speed=float(gp('cruise_speed').value))
        findings = validate(self._mission.pursuit, self._tracker.params,
                            self._mission.mission,
                            float(gp('max_kappa_rate').value), limits)
        for finding in findings:
            self.get_logger().warn(str(finding))
        raise_on_errors(findings)
        for line in describe(self._mission.pursuit, self._tracker.params,
                             self._mission.mission,
                             float(gp('max_kappa_rate').value), limits):
            self.get_logger().info(f'param {line}')

        self._pose_xy = (0.0, 0.0)
        self._pose_psi = 0.0
        self._have_odom = False
        self._last_odom_stamp: float | None = None
        self._measured_kappa: float | None = None
        self._detection_accepted = False
        self._odom_jumps_seen = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Odometry, topics.odom, self._on_odom, sensor_qos)
        self.create_subscription(
            PointCloud2, topics.detection_cloud, self._on_cloud, sensor_qos)
        self.create_subscription(
            PointStamped, topics.detection_point, self._on_point, sensor_qos)

        self._pub_curvature = self.create_publisher(Float64, topics.curvature, 10)
        self._pub_enable = self.create_publisher(Bool, topics.drive_enable, 10)
        self._pub_state = self.create_publisher(String, topics.state, 10)
        self._pub_nis = self.create_publisher(Float64, topics.nis, 10)
        self._pub_diagnostics = self.create_publisher(
            DiagnosticArray, topics.diagnostics, 10)
        if topics.steering:
            self.create_subscription(AckermannDriveStamped, topics.steering,
                                     self._on_steering, sensor_qos)
        self.create_service(Trigger, '~/reset', self._on_reset)

        # Published from a timer, never from the detection callback, so that
        # command timing does not inherit camera jitter or dropped frames.
        rate = float(gp('publish_rate_hz').value)
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            'limiter resync source: '
            + (f'measured steering on "{topics.steering}" '
               f'(wheelbase {self._wheelbase:.3f} m)' if topics.steering
               else 'last commanded curvature (no steering topic configured)'))
        if not self._enabled:
            self.get_logger().warn(
                'SHADOW MODE: computing everything, publishing ZERO curvature. '
                'Set enabled:=true to actuate.')
        self.get_logger().info(
            f'go_to_object up: odom "{topics.odom}", detections '
            f'"{topics.detection_cloud}" / "{topics.detection_point}" -> '
            f'curvature "{topics.curvature}" at {rate:.1f} Hz '
            f'(r_min={gp("r_min").value} m, d_stop={gp("d_stop").value} m, '
            f'max_kappa_rate={gp("max_kappa_rate").value} (1/m)/s)')

    # -- ingest -----------------------------------------------------------

    def _on_steering(self, msg: AckermannDriveStamped) -> None:
        """Where the wheels *are*, for resynchronising the limiter."""
        self._measured_kappa = curvature_from_steering(
            msg.drive.steering_angle, self._wheelbase)

    def _on_odom(self, msg: Odometry) -> None:
        stamp = _stamp_to_sec(msg.header.stamp)
        self._last_odom_stamp = stamp
        p = msg.pose.pose.position
        psi = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._pose_xy = (p.x, p.y)
        self._pose_psi = psi
        self._have_odom = True
        jumps = self._tracker.odom_jumps
        self._tracker.push_odom(stamp, p.x, p.y, psi)
        if self._tracker.odom_jumps != jumps:
            self.get_logger().warn(
                'odom discontinuity (relocalization?): object estimate carried '
                'rigidly across it, pose buffer purged',
                throttle_duration_sec=_WARN_THROTTLE_S)

    def _on_cloud(self, msg: PointCloud2) -> None:
        """Segmented object cloud -> centroid. Segmentation is upstream."""
        try:
            pts = point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            arr = np.array([[float(p[0]), float(p[1]), float(p[2])]
                            for p in pts], dtype=float)
        except Exception as exc:  # malformed cloud; skip the frame
            self.get_logger().warn(
                f'unreadable PointCloud2: {exc}',
                throttle_duration_sec=_WARN_THROTTLE_S)
            return
        if arr.size == 0:
            return
        centroid = arr.mean(axis=0)
        self._ingest_detection(msg.header, float(centroid[0]),
                               float(centroid[1]), float(centroid[2]))

    def _on_point(self, msg: PointStamped) -> None:
        self._ingest_detection(msg.header, msg.point.x, msg.point.y, msg.point.z)

    def _ingest_detection(self, header, x: float, y: float, z: float) -> None:
        # The message's own header stamp is the capture time. If the
        # perception pipeline restamps on publish, that is a bug upstream and
        # must be fixed there -- compensating for it here would bake a guess
        # about someone else's latency into this node.
        capture_stamp = _stamp_to_sec(header.stamp)

        point = PointStamped()
        point.header = header
        point.point.x, point.point.y, point.point.z = x, y, z

        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, header.frame_id, header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as exc:
            # Skip the frame. Never fall back to the untransformed point: it
            # would be interpreted as body-frame metres and steer the car.
            self.get_logger().warn(
                f'TF {header.frame_id!r} -> {self._base_frame!r} failed: {exc}; '
                'detection skipped',
                throttle_duration_sec=_WARN_THROTTLE_S)
            return

        body = do_transform_point(point, tf)
        self._detection_accepted = self._tracker.push_detection(
            capture_stamp, (body.point.x, body.point.y))

    def _on_reset(self, _request, response):
        self._tracker.reset()
        self._mission.reset()
        response.success = True
        response.message = 'go_to_object reset to SEARCH'
        return response

    # -- control ----------------------------------------------------------

    def _on_timer(self) -> None:
        """One control tick. Everything is computed even in shadow mode."""
        now = self.get_clock().now().nanoseconds * 1e-9
        track = self._tracker.state(now) if self._have_odom else None
        before = self._mission.state

        if self._have_odom:
            command = self._mission.update(
                now, self._pose_xy, self._pose_psi, track,
                measured_kappa=self._measured_kappa,
                last_odom_stamp=self._last_odom_stamp)
        else:
            command = self._mission.update(now, self._pose_xy, self._pose_psi,
                                           None)

        if command.state is not before:
            self.get_logger().info(f'{before.value} -> {command.state.value}')
        if command.watchdog:
            self.get_logger().error(
                f'WATCHDOG: {getattr(self._mission, "watchdog_reason", "stale")}'
                ' -- zero curvature, drive gate down',
                throttle_duration_sec=_WARN_THROTTLE_S)

        self._warn_on_inconsistency(track)

        jumped = self._tracker.odom_jumps != self._odom_jumps_seen
        self._odom_jumps_seen = self._tracker.odom_jumps

        # Shadow mode publishes zero on the command topic and nothing else
        # changes, so a manual drive past the object records exactly what the
        # system would have done without any of it reaching an actuator.
        published = command.curvature if self._enabled else 0.0
        self._publish(published, command.drive_enable and self._enabled,
                      command.state,
                      track.nis_mean if track is not None else 0.0)
        self._publish_diagnostics(now, command, track, jumped)
        self._detection_accepted = False

    def _warn_on_inconsistency(self, track) -> None:
        if track is None:
            return
        window = self._tracker.params.nis_window
        if track.nis_samples >= window and track.nis_mean > self._nis_alarm:
            self.get_logger().warn(
                f'filter/sensor mismatch: NIS mean {track.nis_mean:.2f} vs '
                f'expected {_NIS_EXPECTATION:.1f} -- the modelled measurement '
                'noise does not match this camera; check sigma_range_base, '
                'sigma_range_slope and sigma_bearing',
                throttle_duration_sec=_WARN_THROTTLE_S)
        if track.rejection_rate > _REJECTION_ALARM:
            # The complement to NIS: a too-tight R hides from NIS, because a
            # rejected update contributes no sample. It shows up here.
            self.get_logger().warn(
                f'gate rejecting {track.rejection_rate * 100:.0f}% of '
                'detections -- if NIS looks healthy, the measurement model is '
                'too tight rather than too loose',
                throttle_duration_sec=_WARN_THROTTLE_S)

    def _publish(self, curvature: float, drive_enable: bool,
                 state: MissionState, nis_mean: float) -> None:
        self._pub_curvature.publish(Float64(data=float(curvature)))
        self._pub_enable.publish(Bool(data=bool(drive_enable)))
        self._pub_state.publish(String(data=state.value))
        self._pub_nis.publish(Float64(data=float(nis_mean)))

    def _publish_diagnostics(self, now: float, command, track,
                             odom_jumped: bool) -> None:
        """One message, one timestamp, everything needed after the fact.

        ``stamp``, ``measured_kappa`` and ``last_odom_stamp`` are here so that
        :mod:`go_to_object.replay` can reproduce the tick schedule exactly;
        without them a replay could only approximate it. ``kappa_raw`` against
        ``kappa_limited`` plus ``limiter_saturated`` is what tells you whether
        the rate limiter is doing anything on a given drive.
        """
        solution = command.solution
        # Every float goes through format_float: exact, and testable without
        # a ROS runtime. Do not substitute an f-string format spec here for
        # readability -- that is precisely what breaks bit-identical replay.
        values = {
            'stamp': format_float(now),
            'enabled': str(self._enabled),
            'state': command.state.value,
            'kappa_raw': format_float(command.curvature_raw),
            'kappa_limited': format_float(command.curvature),
            'limiter_saturated': str(command.limiter_saturated),
            'drive_enable': str(command.drive_enable),
            'watchdog': str(command.watchdog),
            'measured_kappa': format_optional_float(self._measured_kappa),
            'last_odom_stamp': format_optional_float(self._last_odom_stamp),
            'distance': format_float(solution.distance) if solution else NONE,
            'alpha': format_float(solution.alpha) if solution else NONE,
            'reachable': str(solution.reachable) if solution else NONE,
            'behind': str(solution.behind) if solution else NONE,
            'straight': str(solution.straight) if solution else NONE,
            'arrived': str(solution.arrived) if solution else NONE,
            'confidence': format_float(track.confidence) if track else NONE,
            'converged': str(track.converged) if track else NONE,
            'age': format_float(track.age) if track else NONE,
            'nis_mean': format_float(track.nis_mean) if track else NONE,
            'rejection_rate': (format_float(track.rejection_rate)
                               if track else NONE),
            'odom_jump_detected': str(odom_jumped),
            'detection_accepted': str(self._detection_accepted),
        }

        status = DiagnosticStatus()
        status.name = 'go_to_object'
        status.hardware_id = 'go_to_object'
        status.level = (DiagnosticStatus.ERROR if command.watchdog
                        else DiagnosticStatus.OK)
        status.message = command.state.value
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._pub_diagnostics.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoToObjectNode()
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
