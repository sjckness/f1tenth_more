#!/usr/bin/env python3
"""wall_distance_node: the tracked wall's signed distance, its phase, and the
corridor correction.

PUBLISHES
  <output_topic>                  WallDistance  geometry + phase, at publish_rate_hz
                                                (default /perception/d_wall)
  <output_topic>/segment          WallSegment   the tracked wall in base_link
  <output_topic>/psi_correction   Float32       [rad] heading correction for the
                                                corridor, left positive. 0.0
                                                whenever the phase is not
                                                CORRIDOR or the track is invalid.
  <output_topic>/gate_margin      Float32       [m] EXTRA margin a gate should
                                                require, growing with staleness.
                                                The opposite direction from the
                                                correction above -- see
                                                wall_distance.py.
  <output_topic>/swept_arc        Float32       [m] rear-axle ARC LENGTH before the
                                                body would contact the tracked
                                                wall. DEBUG ONLY, see UNITS below.

SUBSCRIBES
  <scan_topic>          LaserScan   best-effort sensor QoS
  <odom_topic>          Odometry    pose, and the travel the coast caps measure
  <wall_track_topic>    WallTrack   MPC_corr's per-tick wall_turn feed
  /tf_static            TFMessage   base_link <- laser, into this node's own Buffer

EVERY NUMBER PUBLISHED HERE IS DECIDED IN wall_distance.py, and the sign
convention, the control law, the staleness asymmetry and the coast caps are all
documented in that module's docstring. Read it before changing anything here.
This file is glue: parameters, QoS, transforms, message packing.

THE NODE IS ITS OWN SUPERVISOR COMPONENT ON PURPOSE -- see
launch/wall_distance.launch.py for why, and do not move it.


UNITS, BECAUSE THREE NEARBY SIGNALS ARE ALL CALLED SOMETHING LIKE "CLEARANCE"
============================================================================
  <output_topic>            d_wall: SIGNED PERPENDICULAR DISTANCE [m] to the
                            tracked wall, left positive. A lateral offset.
  <output_topic>/swept_arc  ARC LENGTH [m] the REAR AXLE travels before the body
                            RECTANGLE first touches the wall. Metres of TRAVEL,
                            not a perpendicular gap, and straight ahead it is the
                            gap to the front bumper (0.443 m ahead of the axle),
                            not to the axle. A wall abeam at 0.6 m reports the
                            5 m horizon, not 0.6 m. ANYTHING READING IT AS A
                            DISTANCE-TO-WALL IS WRONG.
  /perception/swept_clearance          the same quantity from live LiDAR+ZED
                                       points (swept_clearance_node)
  /perception/front_distance           camera, straight ahead, wall only
  /perception/lidar_front_wall         UNSIGNED perpendicular, forward sector
  /mpc/wall_track .d_wall              UNSIGNED magnitude, MPC-internal

swept_arc exists so the tracked wall's contribution can be WATCHED before
anything consumes it. swept_clearance_node is point-based and only ever sees
where beams currently land; the tracked wall is a fitted line that deliberately
extends past that, which is the whole reason it is tracked. Sampling it into
points at glass_point_spacing and running the SAME swept_corridor.clearance()
the registered node uses is how that extension reaches the same geometry -- not
a parallel implementation of it.


WHY THIS NODE OWNS THE GLASS DETECTOR IN-PROCESS
================================================
glass_detect.py has no node of its own: glass_detector_node.py does not exist,
there is no entry point for one and no launch file, despite that module's
docstring naming it. So there is no segment topic to subscribe to, and this node
calls detect() and GlassTracker.update() directly. It is glass_detect's first
and only runtime consumer, and it owns that detector's CPU cost as well as its
own -- which is why publish_rate_hz (10 Hz) is decoupled from the 40 Hz scan
rate by a timer.


NOTHING HERE HAS EVER RUN ON THE CAR
====================================
The LiDAR was powered down for the whole of this work. The node builds and
passes fixtures, which makes it NOT YET FALSIFIED, not "working". See
f1tenth_perception/README.md's UNVALIDATED section for the list, and
docs/bringup_checklist.md for the order the powered session must go in --
Stage 2's physical left/right sign check before anything downstream runs.
"""

import math

import numpy as np
import rclpy
from f1tenth_messages.msg import WallDistance, WallSegment, WallTrack
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException

from f1tenth_perception.cpu_affinity import apply_nice, declare_nice_param
from f1tenth_perception.glass_detect import DetectorConfig, GlassTracker, detect, sample_segment
from f1tenth_perception.swept_corridor import clearance
from f1tenth_perception.wall_distance import (
    PHASE_CORRIDOR,
    PHASE_NAMES,
    PROVENANCE_NAMES,
    CorrectionConfig,
    Observation,
    PhaseMachine,
    WallDistanceTracker,
    gate_margin,
    geometric_candidates,
    psi_correction,
)

# Same profile tf2_ros.TransformListener uses for /tf_static: the static
# transforms are published once, latched, so a late subscriber needs
# transient_local to ever see them.
_TF_STATIC_QOS = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            history=HistoryPolicy.KEEP_LAST)

# The gates DetectorConfig takes, by their bare (unprefixed) parameter name.
# The launch file forwards them from stack_params.yaml's glass_* block.
_DETECTOR_GATES = (
    ('cos_theta_floor', float), ('normal_window', int),
    ('intensity_range_exponent', float), ('spike_factor', float),
    ('spike_window', int), ('max_spike_width', int),
    ('min_gradient_ratio', float), ('min_void_beams', int),
    ('void_range_jump', float), ('support_window_beams', int),
    ('see_through_max_gap_m', float), ('min_line_inliers', int),
    ('line_inlier_dist', float), ('min_segment_length', float),
    ('max_segment_length', float), ('min_range_m', float),
    ('normal_tolerance_deg', float), ('on_line_min_beams', int),
    ('geom_min_length_m', float), ('geom_min_voids', int),
    ('max_candidates', int), ('point_spacing', float),
)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WallDistanceNode(Node):

    def __init__(self, **kwargs):
        super().__init__('wall_distance_node', **kwargs)

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.scan_topic = str(param('scan_topic', '/scan'))
        self.odom_topic = str(param('odom_topic', '/odometry/filtered'))
        self.wall_track_topic = str(param('wall_track_topic', '/mpc/wall_track'))
        self.output_topic = str(param('output_topic', '/perception/d_wall'))
        self.base_frame = str(param('base_frame', 'base_link'))
        publish_rate_hz = float(param('publish_rate_hz', 10.0))
        self.odom_max_age = float(param('odom_max_age_sec', 0.2))

        # Glass detector gates, declared under their bare names -- the launch
        # file strips the glass_ prefix, matching the stack's convention.
        gates = {}
        defaults = DetectorConfig()
        for name, cast in _DETECTOR_GATES:
            gates[name] = cast(param(name, cast(getattr(defaults, name))))
        self.detector_cfg = DetectorConfig(**gates)
        self.point_spacing = float(gates['point_spacing'])

        # THE SECOND CANDIDATE SOURCE, and it is what makes this node do
        # anything at all on an ordinary wall. glass_detect.detect() returns a
        # surface only when it carries transparency evidence -- on 40 real
        # archive scans of an opaque office corridor it accepted 4 candidates
        # against a confirmation bar of 6 in 8, so nothing was ever tracked.
        # See wall_distance.geometric_candidates' docstring for the measurement
        # and stack_params.yaml's wall_distance_geom_* block.
        # Declared ONCE and shared with the GlassTracker below: the range at
        # which a surface stops being expected to return is the same range at
        # which it stops being worth fitting, and declare_parameter raises on a
        # second declaration anyway.
        self.max_range_m = float(param('max_range_m', 5.0))
        self.geom_enable = bool(param('geom_enable', True))
        self.geom_kwargs = dict(
            min_range_m=float(gates['min_range_m']),
            max_range_m=self.max_range_m,
            inlier_distance_m=float(param('geom_inlier_distance_m', 0.03)),
            min_inliers=int(param('geom_min_inliers', 40)),
            min_span_m=float(param('geom_min_span_m', 1.0)),
            min_distance_m=float(param('geom_min_distance_m', 0.5)),
            max_candidates=int(param('geom_max_candidates', 4)),
        )

        glass_tracker = GlassTracker(
            persistence_window=int(param('persistence_window', 8)),
            persistence_hits=int(param('persistence_hits', 3)),
            match_endpoint_tol=float(param('match_endpoint_tol', 0.20)),
            match_angle_tol_deg=float(param('match_angle_tol_deg', 10.0)),
            expect_return_tol_deg=float(param('expect_return_tol_deg', 60.0)),
            fov_half_angle_rad=float(param('fov_half_angle_rad', math.pi / 2)),
            max_range_m=self.max_range_m,
            geometry_only_multiplier=int(param('geometry_only_multiplier', 2)),
        )
        self.tracker = WallDistanceTracker(
            glass_tracker=glass_tracker,
            max_coast_distance=float(param('max_coast_distance', 0.5)),
            max_coast_yaw=float(param('max_coast_yaw', 0.35)),
            track_timeout_sec=float(param('track_timeout_sec', 0.0)),
        )

        self.correction = CorrectionConfig(
            d_ref=float(param('d_ref', 0.60)),
            convergence_length_m=float(param('convergence_length_m', 3.0)),
            max_psi_correction=float(param('max_psi_correction', 0.20)),
            max_psi_rate=float(param('max_psi_rate', 0.5)),
            deadband_floor=float(param('deadband_floor', 0.02)),
            deadband_k=float(param('deadband_k', 2.0)),
            fade_start_age=float(param('fade_start_age', 0.3)),
            fade_zero_age=float(param('fade_zero_age', 1.0)),
            stale_inflate_per_s=float(param('stale_inflate_per_s', 0.15)),
        )
        # A fade window that is empty or inverted makes soft_fade a step
        # function, which defeats the whole reason it is a ramp. Caught here
        # rather than producing a correction that snaps rather than fades.
        if self.correction.fade_zero_age <= self.correction.fade_start_age:
            raise ValueError(
                f'fade_zero_age ({self.correction.fade_zero_age}) must be greater than '
                f'fade_start_age ({self.correction.fade_start_age}): otherwise the soft '
                'correction snaps to zero instead of fading, and the rate limit is the '
                'only thing left smoothing it')

        self.phase_machine = PhaseMachine(
            silence_ticks=int(param('wall_track_silence_ticks', 3)))

        # Footprint for the swept-arc debug value. Same numbers as
        # swept_clearance_node's, read from the same stack_params keys, so the
        # two can never describe different cars.
        self.geometry = {
            'wheelbase': float(param('swept_wheelbase_m', 0.305)),
            'front_x': float(param('swept_body_front_x_m', 0.443)),
            'rear_x': float(param('swept_body_rear_x_m', -0.082)),
            'half_width': float(param('swept_body_half_width_m', 0.136)),
            'margin': float(param('swept_margin_m', 0.10)),
            'max_range': float(param('swept_max_range_m', 5.0)),
            'absolute_min_clearance': float(param('swept_absolute_min_clearance_m', 0.15)),
        }
        self.rear_axle_x = float(param('swept_rear_axle_x_m', 0.0))

        declare_nice_param(self)
        apply_nice(self)

        self._tf_buffer = Buffer()
        self._laser_pose = None            # (x, y, yaw) in base_frame
        self._pose = None                  # (x, y, yaw) in odom
        self._pose_stamp = None
        # psi_commit of the most recent WallTrack since the last publish tick,
        # or None. Consumed and cleared by the timer -- that is what makes
        # wall_track_silence_ticks count TICKS rather than wall-clock time.
        self._pending_psi_commit = None
        # Newest unprocessed scan. Fitted on the publish tick, not in the
        # callback -- see _scan_cb for the measurement that forced it.
        self._scan = None
        self._last_obs = None
        self._psi_correction = 0.0
        self._last_tick_sec = None

        self.d_wall_pub = self.create_publisher(WallDistance, self.output_topic, 10)
        self.segment_pub = self.create_publisher(
            WallSegment, self.output_topic + '/segment', 10)
        # Reliable, like /mpc/wall_track and /mpc/solver_status: a correction
        # the corridor acts on is not a sensor stream, and a dropped one is a
        # tick of stale geometry in mpc_corr.
        self.psi_pub = self.create_publisher(
            Float32, self.output_topic + '/psi_correction', 10)
        self.gate_pub = self.create_publisher(
            Float32, self.output_topic + '/gate_margin', 10)
        self.swept_pub = self.create_publisher(
            Float32, self.output_topic + '/swept_arc', 10)

        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self._tf_static_cb, _TF_STATIC_QOS)
        # Best-effort sensor QoS: compatible with urg_node's reliable
        # publisher, sends it no acknowledgements, and so cannot back-pressure
        # it or the e-stop (f1tenth_behavior's IsProximityTooClose) that reads
        # the same topic. Same reasoning MPC_corr.py:1322-1334 records.
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_cb, qos_profile_sensor_data)
        # Matched to MPC_corr's publisher: depth 10, default reliable/volatile.
        # Volatile is why UNKNOWN is the startup phase -- a subscriber that
        # starts between turns sees nothing at all until the next one.
        self.wall_track_sub = self.create_subscription(
            WallTrack, self.wall_track_topic, self._wall_track_cb, 10)

        self.tick_period = 1.0 / publish_rate_hz
        self.timer = self.create_timer(self.tick_period, self._publish)

        self.get_logger().info(
            f'wall_distance_node up: scan {self.scan_topic}, odom {self.odom_topic}, '
            f'phase from {self.wall_track_topic} '
            f'({self.phase_machine.silence_ticks} silent ticks -> exit) -> '
            f'{self.output_topic} at {publish_rate_hz:.1f} Hz. '
            f'd_ref={self.correction.d_ref:.2f} m, k={self.correction.k:.3f} rad/m '
            f'(convergence {self.correction.convergence_length_m:.1f} m), '
            f'|dpsi|<={self.correction.max_psi_correction:.3f} rad. '
            f'coast caps {self.tracker.max_coast_distance:.2f} m / '
            f'{self.tracker.max_coast_yaw:.2f} rad. PHASE=UNKNOWN until '
            f'{self.wall_track_topic} speaks.')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs -----------------------------------------------------------

    def _tf_static_cb(self, msg):
        for transform in msg.transforms:
            self._tf_buffer.set_transform_static(transform, 'wall_distance_node')

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        self._pose = (float(pose.position.x), float(pose.position.y),
                      yaw_of(pose.orientation))
        self._pose_stamp = self._now()
        # Sampled on EVERY pose, not only when a scan lands: the coast caps
        # measure odometry PATH LENGTH, and a path sampled at the scan rate
        # would under-read an arc.
        self.tracker.odometer.update(self._pose)

    def _wall_track_cb(self, msg):
        self._pending_psi_commit = float(msg.psi_commit)

    def _scan_cb(self, msg):
        """Store the newest scan. THE FIT DOES NOT HAPPEN HERE.

        MEASURED, ON REAL SCANS, BEFORE THIS WAS A TIMER. The full per-scan
        path (glass detect + geometric fit + track + observe + swept_arc) costs
        26.5 ms/scan on 300 real 1081-beam scans on this Jetson, single core.
        /scan arrives at 40 Hz, a 25.0 ms budget -- so fitting in this callback
        is 106% OF ONE CORE and the node cannot keep up. It would fall behind
        urg_node forever, on the core the e-stop's neighbours share.

        Refitting once per PUBLISH tick (10 Hz) costs 26.5% of a core instead,
        and 10 Hz is not a compromise: it is MPC_corr's own control tick, the
        rate at which anything can consume the output, and the rate
        wall_tracker.py's own archive measurement endorses ("the refit is per
        tick and not per rebuild: replayed on the archive with 1 s between
        refits, d_wall ran a 0.15 m sawtooth"). 40 Hz bought nothing any
        consumer could see.

        Keeping the NEWEST scan and dropping the rest is right for the same
        reason MPC_corr's own /scan handler does it: a stale scan is not worth
        fitting when a fresher one is in hand.
        """
        self._scan = msg

    def _fit_latest_scan(self):
        """Refit the track from the stored scan. Called once per publish tick."""
        msg, self._scan = self._scan, None
        if msg is None:
            return
        laser = self._laser_pose_for(msg.header.frame_id)
        if laser is None:
            return
        pose = self._fresh_pose()
        if pose is None:
            self.get_logger().warn(
                f'no pose younger than {self.odom_max_age:.2f} s on {self.odom_topic}; '
                'a scan cannot be placed in odom, so the track cannot be refitted',
                throttle_duration_sec=5.0)
            return
        intensities = msg.intensities if len(msg.intensities) else None
        scan_args = (msg.ranges, msg.angle_min, msg.angle_increment,
                     msg.range_min, msg.range_max, laser, pose)
        # BOTH SOURCES, EVERY FIT, into one candidate list. glass_detect's
        # carry transparency evidence and become GLASS_CONFIRMED tracks;
        # geometric_candidates' carry none and become GEOMETRIC ones -- the
        # distinction WallDistance.msg's provenance field exists for. Glass
        # first, so that where both describe the same surface the glass one is
        # the candidate GlassTracker matches and the track inherits its
        # evidence.
        #
        # BOTH ARE NEEDED. On 40 real archive scans of an opaque office
        # corridor glass_detect accepted 4 candidates against a confirmation
        # bar of 6 in 8, so glass alone tracked nothing for the whole run --
        # see wall_distance.geometric_candidates' docstring.
        candidates, use_intensity = detect(
            *scan_args, self.detector_cfg, intensities=intensities)
        if self.geom_enable:
            candidates = list(candidates) + geometric_candidates(
                *scan_args, **self.geom_kwargs)
        self.tracker.update(candidates, pose, self._now(), use_intensity)

    def _laser_pose_for(self, frame_id):
        if self._laser_pose is not None:
            return self._laser_pose
        try:
            stamped = self._tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except TransformException:
            self.get_logger().warn(
                f'no {self.base_frame} <- {frame_id!r} transform on /tf_static yet; '
                'skipping scans until it arrives', throttle_duration_sec=5.0)
            return None
        t = stamped.transform
        self._laser_pose = (float(t.translation.x), float(t.translation.y),
                            yaw_of(t.rotation))
        self.get_logger().info(
            f'{self.base_frame} <- {frame_id}: '
            f'({self._laser_pose[0]:.3f}, {self._laser_pose[1]:.3f}) m, '
            f'yaw {self._laser_pose[2]:+.3f} rad. NOTE description.launch.py calls these '
            'ruler-measured placeholders, not a calibration.')
        return self._laser_pose

    def _fresh_pose(self):
        if self._pose is None or self._pose_stamp is None:
            return None
        if self._now() - self._pose_stamp > self.odom_max_age:
            return None
        return self._pose

    # ---- the tick ---------------------------------------------------------

    def _publish(self):
        now = self._now()
        # Measured, not assumed: the rate limit is per second and a timer that
        # slipped must not be charged as if it had not. The nominal period is
        # the fallback for the first tick only.
        dt = (now - self._last_tick_sec if self._last_tick_sec is not None
              else self.tick_period)
        self._last_tick_sec = now

        # Refit first, so this tick's observation reads a fit from this tick
        # rather than the previous one.
        self._fit_latest_scan()

        pose = self._fresh_pose()
        if pose is None:
            # No fresh pose means the stored line cannot even be evaluated
            # against the car, let alone predicted forward. That is NONE, not a
            # stale value carried on.
            obs = Observation.none()
        else:
            obs = self.tracker.observation(pose, now)

        psi_commit, self._pending_psi_commit = self._pending_psi_commit, None
        transition = self.phase_machine.tick(psi_commit, track_valid=obs.valid)
        if transition is not None:
            # The first thing anyone debugs. Not throttled: a phase change
            # happens a handful of times a run, and a throttled one that got
            # swallowed is worse than useless.
            self.get_logger().info(f'PHASE | {transition}')

        if obs.coast_cap_first_hit:
            self.get_logger().warn(
                f'COAST/cap | track {obs.track_id} coasted '
                f'{obs.coast_distance:.2f} m / {obs.coast_yaw:.2f} rad past the cap '
                f'({self.tracker.max_coast_distance:.2f} m / '
                f'{self.tracker.max_coast_yaw:.2f} rad): d_wall is no longer trusted. '
                'The caps are set by the measured odometry bias (distance 17-20% short, '
                'gyro yaw gain 0.93-1.09), not by caution -- raise them only after that '
                'calibration lands.')

        if (self._last_obs is not None and obs.valid and self._last_obs.valid
                and obs.track_id != self._last_obs.track_id):
            self.get_logger().info(
                f'TRACK | re-associated {self._last_obs.track_id} -> {obs.track_id}: '
                f'd_wall steps {self._last_obs.d_wall:+.3f} -> {obs.d_wall:+.3f} m. '
                'Published, not hidden, so the consumer can reject the step.')

        # The correction only ever runs in CORRIDOR. UNKNOWN produces no
        # correction and no gating, and a turn in progress is not this node's
        # business -- see wall_distance.py on why the turn's own dpsi_this has
        # no authority at exit anyway.
        applicable = obs.valid and self.phase_machine.phase == PHASE_CORRIDOR
        self._psi_correction = psi_correction(
            obs.d_wall, cfg=self.correction, fit_rms=obs.fit_rms, age=obs.age,
            prev_psi=self._psi_correction, dt=dt, valid=applicable)

        self.d_wall_pub.publish(self._wall_distance_msg(obs))
        self.segment_pub.publish(self._segment_msg(obs))
        self.psi_pub.publish(Float32(data=float(self._psi_correction)))
        self.gate_pub.publish(Float32(data=float(
            gate_margin(obs.age, self.correction.stale_inflate_per_s) if obs.valid else 0.0)))
        self.swept_pub.publish(Float32(data=float(self._swept_arc(obs))))

        self.get_logger().info(
            f'DWALL | phase={PHASE_NAMES[self.phase_machine.phase]} '
            f'track={obs.track_id} valid={obs.valid} '
            f'd_wall={obs.d_wall:+.3f} heading_rel={obs.heading_rel:+.3f} '
            f'rms={obs.fit_rms:.4f} n={obs.inlier_count} age={obs.age:.2f} '
            f'prov={PROVENANCE_NAMES[obs.provenance]} '
            f'coast={obs.coast_distance:.2f}m/{obs.coast_yaw:.2f}rad '
            f'dpsi={self._psi_correction:+.4f}',
            throttle_duration_sec=1.0)
        self._last_obs = obs

    def _swept_arc(self, obs):
        """Rear-axle arc length before the body contacts the TRACKED wall.

        METRES OF TRAVEL, NOT A PERPENDICULAR GAP -- see this module's UNITS
        section. Debug only; nothing consumes it.

        The tracked wall is a fitted LINE and swept_corridor.clearance() is
        point-based, so the segment is sampled at point_spacing first. That
        route was chosen over extending clearance() to take a segment:
        sample_segment() already exists and is unit-tested
        (test_glass_detect.py), while a segment overload would mean new
        geometry inside the one module the stack's swept-clearance
        correctness rests on. A 5 m segment at 0.025 m is 201 points, cheap
        next to the 1081-beam scan the same function already takes.

        Straight corridor only (delta = 0): this node has no steering
        subscription, and giving it one would duplicate
        swept_clearance_node's SteeringHistory. The straight arc is the
        conservative reading for a car that is, in phase CORRIDOR, meant to
        be going straight.
        """
        if not obs.valid or obs.p0 is None:
            return float(self.geometry['max_range'])
        points = sample_segment(obs.p0, obs.p1, self.point_spacing)
        points = points - np.array([self.rear_axle_x, 0.0])
        return clearance(points, 0.0, **self.geometry)

    # ---- message packing --------------------------------------------------

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _wall_distance_msg(self, obs):
        msg = WallDistance()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.base_frame
        msg.track_id = int(obs.track_id)
        msg.d_wall = float(obs.d_wall)
        msg.heading_rel = float(obs.heading_rel)
        msg.fit_rms = float(obs.fit_rms)
        msg.inlier_count = int(obs.inlier_count)
        msg.age = float(obs.age)
        msg.provenance = int(obs.provenance)
        msg.phase = int(self.phase_machine.phase)
        msg.valid = bool(obs.valid)
        return msg

    def _segment_msg(self, obs):
        msg = WallSegment()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.base_frame
        msg.track_id = int(obs.track_id)
        msg.valid = bool(obs.valid and obs.p0 is not None)
        p0 = obs.p0 if msg.valid else (math.nan, math.nan)
        p1 = obs.p1 if msg.valid else (math.nan, math.nan)
        msg.p0.x, msg.p0.y, msg.p0.z = float(p0[0]), float(p0[1]), 0.0
        msg.p1.x, msg.p1.y, msg.p1.z = float(p1[0]), float(p1[1]), 0.0
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = WallDistanceNode()
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
