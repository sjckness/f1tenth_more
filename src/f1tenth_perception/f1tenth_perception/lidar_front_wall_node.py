#!/usr/bin/env python3
"""LiDAR front wall: a robust line fit to the forward /scan sector, and the
usable-for-control wall distance built from it.

PUBLISHES one message on each topic for EVERY /scan -- valid or not, because
a topic that goes silent cannot be told apart from a dead node:

  /perception/lidar_front_wall          f1tenth_messages/WallLineFit
      The MEASURED wall: this scan's fit, whether it passed, and if not the
      first check it failed.
  /perception/lidar_front_wall_virtual  f1tenth_messages/WallEstimate
      The usable-for-control estimate: measured when available, dead-reckoned
      from the last accepted fit when not, seeded from /costmap/front_clearance
      when there is nothing to dead-reckon.

Nothing consumes either topic yet, deliberately, and neither is wired into
missions. The pure logic (fit, checks, state) is lidar_front_wall.py; this
file is the ROS glue and the documentation of record for both.

CONTRACT FOR /perception/lidar_front_wall_virtual
-------------------------------------------------
It always publishes. It does NOT always have a usable number: consumers must
check `valid`. Each scan the provenance is the first of these that applies:

  MEASURED       This scan's fit passed every check below. The estimate SNAPS
                 to it, never blends. When the previous scan was not MEASURED
                 the jump (measured - previous estimate) is logged: it is the
                 diagnostic for how far dead reckoning or the seed had drifted,
                 and blending would hide exactly that.
  DEAD_RECKONED  No accepted fit, the last accepted fit is at most
                 stale_age_sec old, and odometry is fresh.
  SEEDED         Nothing to dead-reckon (never measured, aged out, odometry
                 stale) and a fresh, in-range /costmap/front_clearance value.
                 This re-seeds after an age-out too: the node has no
                 move-start signal, so seeding only before the first ever
                 measurement would mean never seeding again once any wall had
                 been seen. A live seed beats nothing.
  NONE           None of the above. valid=false, distance NaN. This happens
                 at startup before anything arrives, and whenever a
                 dead-reckoned value ages out with no fresh seed to fall back
                 on.

WHAT "MEASURED" REQUIRES -- the checks, in the order a rejection reports them
-----------------------------------------------------------------------------
  no_transform         base_link <- scan frame not known yet (see LASER POSE).
  too_few_returns      Fewer valid returns in the sector than the inlier floor.
  low_inlier_count     The best line holds fewer inliers than the floor.
  low_inlier_fraction  The best line is not the dominant surface.
  oblique              Its normal is too far off the heading: the car is going
                       along this wall, not toward it.
  wrong_surface        It disagrees with the dead-reckoned wall (below).

THRESHOLDS -- measured on the run archive (~58k forward scans across the
archived missions), not guessed. stack_params.yaml carries the same values.

  sector_half_angle_deg   10.0  Normal-angle noise, 1 sigma / p99: 0.73/3.4 deg
                                at 0.4-0.8 m, 0.43/1.6 at 1.2-1.6 m, 0.16/0.7 at
                                2-3 m -- about half of +-5 deg's. +-5 deg holds on
                                those numbers; +-10 is margin for what they
                                cannot see. The base_link->laser TF is a
                                ruler-measured placeholder, and a mount-yaw error
                                is a constant bias invisible to every noise
                                figure. No archived wall_turn ever completed (max
                                rotation 50 deg), so the half-turn timing below is
                                geometric, not measured. Back to +-5 is this one
                                parameter.
  inlier_distance_m       0.03  Clean-wall acceptance is flat across 2/3/5 cm
                                (96.6/96.7/96.9%). 3 cm is ~2.7x the p99 inlier
                                residual (11.3 mm; median 2.8 mm).
  min_inlier_fraction     0.60  Of VALID returns. Majority rule with margin: the
                                accepted line must be the dominant surface, and a
                                50/50 wall/object split is rejected rather than
                                coin-flipped.
  min_inlier_count_ratio  0.50  Floor = ceil(0.5 x sector beams): 41 at +-10 deg,
                                21 at +-5 deg. Catches what the fraction cannot:
                                most beams returning nothing (glass, black
                                surfaces) while the few that do are 100% on-line.
  oblique_max_deg         45.0  Past 45 deg the car moves along the wall more
                                than toward it. Fit quality holds to 50 deg
                                (median fraction 1.00, 2.9 mm residual) and
                                collapses at 50-60 deg (0.56, 8.6 mm).
  max_hypothesis_pairs    800   81 beams make 3240 pairs: 5.0 ms per scan, 20% of
                                a core at 40 Hz. 800 is 1.3 ms, 5%.
  wrong_surface_gate_m    0.30  See WHY THE WRONG-SURFACE GATE EXISTS.
  stale_age_sec           3.0   The dead-reckoned half of a wall_turn: at the
                                0.278 rad steering limit R = 1.07 m, and 45->90
                                deg is 0.84 m of arc, 2.1 s at 0.4 m/s and 2.8 s
                                at 0.3 m/s. Also covers 97% of recorded fit
                                losses in object missions.
  odom_max_age_sec        0.2   /odometry/filtered runs at 50 Hz; its longest
                                recorded gap is 136 ms (p99.9 120 ms).
  seed_max_age_sec        0.5   /costmap/front_clearance runs at 20 Hz; its
                                longest recorded gap is 0.19 s.
  seed_max_valid_m        5.0   costmap_boundary_node's max_range_m. It publishes
                                max_range_m + 1.0 for "nothing within range",
                                which is not a distance.

WHY THE WRONG-SURFACE GATE EXISTS -- DO NOT REMOVE IT AS OVER-ENGINEERING
-------------------------------------------------------------------------
The single-scan checks cannot tell a wall from any other flat surface: when an
object stands in the sector, the fit lands on the object. On the archived
runs, 28% of fit losses lasting 0.3-3 s came back on a surface more than 2 m
from where dead reckoning put the wall. Snapping to those reports the object
as the wall. Without this gate, snap-on-reacquisition is actively unsafe.

A fit further than wrong_surface_gate_m from the dead-reckoned prediction is
rejected (wrong_surface) and dead reckoning carries on. 0.30 m sits in the
valley of the recorded innovation histogram; same-wall reacquisitions fell at
-16/+6 cm (p10/p90). The gate is tested only against a prediction from a real
measurement, never against a seed (a different quantity), and it lapses when
that prediction ages out. After that the next passing fit is accepted as a
fresh acquisition.

DEAD RECKONING
--------------
d = d0 - (p - p0) . n, where n is the wall normal captured in the odometry
frame at the last accepted fit (its normal_angle plus the odometry yaw). That
is the closing rate ds * cos(theta) integrated in closed form. It is NOT
d0 - distance_travelled. Path length is only right when driving square at the
wall; during a wall_turn it over-counts the closing and fires the turn early.
Over the 45->90 deg half-turn at R = 1.07 m that error is 0.53 m. It would
look like a tuning problem and be a geometry error.

Odometry is /odometry/filtered, the local EKF, chosen against the LiDAR on the
archived runs:

  /odom               Yaw integrated from the COMMANDED servo, no IMU. Its yaw
                      gain against the fitted wall was 0.62-0.92.
  /ekf_global         Right distance scale, but only because SLAM steps it back
                      ~1.9 times a second, and dead reckoning integrates those
                      steps as motion: 157 steps over 20 cm per hour, a ~12%
                      chance of one inside a 3 s window, worst seen 3.9 m and
                      44 deg. Disqualified.
  /slam/pose          Median 1.5 Hz with gaps to 9 s, slower than the gaps it
                      would have to bridge.
  /odometry/filtered  Smooth, gyro-fused yaw (gain 0.93-1.09). Chosen.

KNOWN BIAS -- present in /odometry/filtered, adopted anyway, NOT compensated
here. Its distance is short: the LiDAR measured 1.20x the closing the odometry
reported (IQR 1.17-1.25, 49 straight segments). Dead reckoning therefore
under-counts closing. The wall reads FARTHER than it is, and a turn keyed on
this estimate fires LATE -- about 25 cm over 3 s at 0.5 m/s. Expect logged
reacquisition jumps to lean negative. The cause is consistent with vesc.yaml's
speed_to_erpm_gain moving 4614 -> 5499.27 in commit 1099733 (ratio 1.19). That
gain also feeds distance_reached, the MPC and every other odometry distance
consumer, so it gets its own change with its own verification. Correcting for
it here would hide it, and double-correct the day it is fixed.

Odometry older than odom_max_age_sec turns off both dead reckoning and the
wrong-surface gate (there is nothing to predict with). The transition is
logged.

THE SEED
--------
/costmap/front_clearance (f1tenth_costmap's costmap_boundary_node) is the
nearest occupied SLAM-map cell in a +-35 deg forward cone. That is a DIFFERENT
quantity from a fitted wall line, and they disagree: on the archive seed - fit
was -18 cm median with an IQR of -270 to +9 cm, because the cone catches side
clutter. Expect a jump on the first real acquisition; provenance is there to
say so.

Its ABSENCE is the case to handle, not just a bad number.
costmap_boundary_node withholds the topic until it has received a map and a
pose once. After that it publishes every tick from cached inputs however old
they are (it has no recency gate), so a seed computed from a stale map looks
identical on the wire. It is a bare Float32 with no header, so its age here is
receipt time.

BLIND SPOTS -- known gaps, not oversights
-----------------------------------------
  * Glass. The LiDAR does not see it.
  * A flat object face square to the heading that appears within
    wrong_surface_gate_m of the dead-reckoned wall. It passes every check and
    is reported as the wall.
  * A flat object face that appears while there is no prediction to test it
    against: at startup, after a dead-reckoned value ages out, while seeded,
    or while odometry is stale. The first passing fit is accepted.

The intended later mitigation for the last two is a veto from YOLO detections
that fall inside the sector. It is not built.

NO COUPLING TO THE E-STOP
-------------------------
f1tenth_behavior's IsProximityTooClose reads raw /scan in its own process, and
must keep working however this node behaves. This node shares no code, no
parameter and no timing with it. It subscribes with best-effort sensor QoS,
which is compatible with urg_node's reliable publisher, sends it no
acknowledgements, and cannot back-pressure it. It runs as its own supervisor
component, never inside lidar.launch.py or the `perception` component; see
launch/lidar_front_wall.launch.py for why that placement is load-bearing.

SCENE LABELS (enable_scene_labels, stack_params.yaml; default true)
--------------------------------------------------------------------
With the switch on, three things change here, all in scene_labels.py:

  * Every YOLO detection on /camera/detections_3d (younger than
    detection_max_age_sec) cuts its bearing sector, widened by
    detection_margin_deg, out of the scan BEFORE any fit. A person in front of
    the wall no longer makes the wall a minority of the sector (M02 P002-R005:
    low_inlier_fraction for 6.5 s while the camera stopped the car on the
    person).
  * The fit is not the +-sector_half_angle_deg one. Sequential wall lines are
    extracted from the cut wide sector (wall_planes_half_angle_deg) at
    wall_planes_rate_hz and published on /perception/wall_planes for the stereo
    labelling in front_clearance_node; the nearest line the heading crosses
    is tracked and refitted on every scan. It must SPAN wall_min_span_m and lie
    under the heading beams (scene_labels.wall_ahead), else short_span: an
    object face is short, a doorway's two side faces are not a wall ahead. It
    must hold wall_planes_min_inliers and
    face the car within oblique_max_deg. The WallEstimator, its odometry dead
    reckoning and its wrong-surface gate are unchanged.
  * /perception/virtual_wall (Float32, from the FRONT BUMPER, -1.0 = no
    reading) and /perception/virtual_wall/debug (JSON: source, flags): the
    estimate above fused with the stereo WALL distance on
    /perception/stereo_wall (scene_labels.VirtualWall). It is the single wall
    truth: with the switch on every wall consumer reads it
    (f1tenth_params.param_defaults.get_wall_distance_topic()).

With the switch off none of this is subscribed or published, and the fit is
the one above, unchanged.

MULTI-WALL TRACKER (use_wall_tracker, stack_params.yaml; default true)
-----------------------------------------------------------------------
multi_wall_tracker.py runs on every scan. It needs enable_scene_labels: set
without it, the node logs an ERROR at startup and the tracker stays off.

  * Object cut: detections are carried from their own stamp to the scan stamp
    on /odometry/filtered instead of the receipt-age window, and are dropped
    after wall_tracker_detection_max_age_sec. The cut applies to the
    sequential lines too.
  * Up to 4 wall tracks in odom go out on /perception/walls
    (f1tenth_messages/MultiWallTrackArray), with line and text markers on
    /perception/walls/markers.
  * The heading wall: the front track (multi_wall_tracker.front_track: the
    published track labelled front, matched on this scan or coasting for at
    most wall_tracker_front_max_coast_sec, 0.3 s) is the wall that
    refit_wall, WallEstimator (wrong-surface gate included) and VirtualWall
    consume. With no front track, it is the sequential_lines wall, as before.
  * Coast limit: when the front track is dropped because it has coasted
    longer than that (or was deleted while coasting), /perception/virtual_wall
    publishes -1.0 (no wall) on every scan until the tracker has a front track
    again; neither the sequential_lines wall nor the dead-reckoned or stereo
    value stands in for it meanwhile. A front track that stops being the
    front one for another reason (relabelled as the car turns, the heading
    ray misses it) falls back to sequential_lines as before.
  * /perception/virtual_wall/debug says which: wall_source 'tracker',
    'sequential_lines' or 'front_coast_expired'. It also carries tracker_ms.
  * The scan pose is /odometry/filtered interpolated at the scan stamp, the
    pose the EKF also publishes as odom -> base_link.

LASER POSE
----------
base_link <- scan frame comes from /tf_static, fed into a tf2 Buffer this node
fills itself. It does not subscribe to /tf: a TransformListener would, and
would deserialize every dynamic transform in Python to read one static edge.
It is also not a hardcoded offset, because the current TF is a ruler-measured
placeholder and a recalibrated mount yaw must reach the fit. Until the
transform arrives every scan publishes no_transform.
"""

import json
import math
import time

from f1tenth_description.vehicle_frames import base_front_x, MIN_FRONT_DISTANCE_M
from f1tenth_messages.msg import MultiWallTrack, MultiWallTrackArray, WallEstimate, WallLineFit
from f1tenth_params.param_defaults import get_value
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

from f1tenth_perception.cpu_affinity import apply_nice, declare_nice_param
from f1tenth_perception.lidar_front_wall import (
    PROVENANCE_NAMES,
    REASON_LOW_INLIER_COUNT,
    REASON_NO_TRANSFORM,
    REASON_OBLIQUE,
    REASON_OK,
    REASON_SHORT_SPAN,
    REASON_TOO_FEW_RETURNS,
    SectorFit,
    WallEstimator,
    classify_fit,
    fit_forward_line,
)
from f1tenth_perception import multi_wall_tracker as mwt
from f1tenth_perception import scene_labels
from f1tenth_perception.ground_plane import quat_to_matrix

# Same profile tf2_ros.TransformListener uses for /tf_static: the static
# broadcasters latch, so a late subscriber still receives every edge.
_TF_STATIC_QOS = QoSProfile(
    depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LidarFrontWallNode(Node):

    def __init__(self, **kwargs):
        super().__init__('lidar_front_wall_node', **kwargs)

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.scan_topic = str(param('scan_topic', '/scan'))
        self.odom_topic = str(param('odom_topic', '/odometry/filtered'))
        self.seed_topic = str(param('seed_topic', '/costmap/front_clearance'))
        self.fit_topic = str(param('fit_topic', '/perception/lidar_front_wall'))
        self.estimate_topic = str(param('estimate_topic', '/perception/lidar_front_wall_virtual'))
        self.base_frame = str(param('base_frame', 'base_link'))
        # The seed topic is from the front bumper (the URDF's base_front); see
        # _seed_cb.
        self.front_x = base_front_x()

        self.sector_half_angle_rad = math.radians(float(param('sector_half_angle_deg', 10.0)))
        self.inlier_distance_m = float(param('inlier_distance_m', 0.03))
        self.min_inlier_fraction = float(param('min_inlier_fraction', 0.6))
        self.min_inlier_count_ratio = float(param('min_inlier_count_ratio', 0.5))
        self.oblique_max_rad = math.radians(float(param('oblique_max_deg', 45.0)))
        self.max_hypothesis_pairs = int(param('max_hypothesis_pairs', 800))
        self.estimator = WallEstimator(
            stale_age_sec=float(param('stale_age_sec', 3.0)),
            wrong_surface_gate_m=float(param('wrong_surface_gate_m', 0.3)),
            odom_max_age_sec=float(param('odom_max_age_sec', 0.2)),
            seed_max_age_sec=float(param('seed_max_age_sec', 0.5)),
            seed_max_valid_m=float(param('seed_max_valid_m', 5.0)),
        )

        # ---- scene labels (module docstring) --------------------------------
        self.scene_enabled = bool(param('enable_scene_labels', get_value('enable_scene_labels')))
        self.detection_margin_rad = math.radians(float(param('detection_margin_deg', 3.0)))
        self.detection_max_age = float(param('detection_max_age_sec', 0.5))
        self.wall_min_span_m = float(param('wall_min_span_m', 0.8))
        self.planes_half_angle_rad = math.radians(
            float(param('wall_planes_half_angle_deg', 60.0)))
        planes_rate = float(param('wall_planes_rate_hz', 10.0))
        self.planes_period = 1.0 / planes_rate if planes_rate > 0.0 else math.inf
        self.planes_max_lines = int(param('wall_planes_max_lines', 3))
        self.planes_min_inliers = int(param('wall_planes_min_inliers', 15))
        self.virtual = scene_labels.VirtualWall(
            self.front_x,
            disagree_m=float(param('virtual_disagree_m', 0.15)),
            stereo_max_age_s=float(param('stereo_max_age_sec', 0.5)))
        self._detections = None      # (receipt time, [(x, y, half_width)] base_link)
        self._next_planes = 0.0
        self._tracked_wall = None    # scene_labels.WallLine, base_link

        # ---- multi-wall tracker (module docstring) --------------------------
        self.use_wall_tracker = bool(param('use_wall_tracker', get_value('use_wall_tracker')))
        self.tracker_det_max_age = float(param('wall_tracker_detection_max_age_sec', 1.0))
        tracker_params = mwt.TrackerParams(
            process_noise_rho=float(param('wall_tracker_process_noise_rho', 1e-4)),
            process_noise_theta=float(param('wall_tracker_process_noise_theta', 1e-4)),
            k_yaw=float(param('wall_tracker_k_yaw', 0.1)),
            min_overlap_m=float(param('wall_tracker_min_overlap_m', 0.2)),
            delete_after_s=float(param('wall_tracker_delete_after_sec', 3.0)),
            max_sigma_rho_m=float(param('wall_tracker_max_sigma_rho_m', 0.3)),
            max_published=int(param('wall_tracker_max_tracks', 4)),
            front_max_coast_s=float(param('wall_tracker_front_max_coast_sec', 0.3)))
        if self.use_wall_tracker and not self.scene_enabled:
            self.get_logger().error(
                'use_wall_tracker needs enable_scene_labels (the virtual wall it feeds '
                'exists only with scene labels): the multi-wall tracker stays OFF')
            self.use_wall_tracker = False
        self.wall_tracker = mwt.MultiWallTracker(tracker_params) if self.use_wall_tracker else None
        self._poses = mwt.PoseHistory()
        self._odom_frame = 'odom'
        self._wall_source = 'sequential_lines'
        self._front_id = None       # id of the tracker's current front track
        self._front_coast_expired = False   # module docstring, coast limit
        self._seq_wall = None       # the latest sequential_lines heading wall
        self._tracker_ms = math.nan
        self._tracker_cost = []     # per-scan tracker ms, logged every 30 s
        self._next_cost_log = None

        declare_nice_param(self)
        apply_nice(self)

        self._tf_buffer = Buffer()
        self._laser_pose = None  # (scan frame_id, (x, y, yaw) in base_frame)
        self._odometry_was_fresh = None

        self.fit_pub = self.create_publisher(WallLineFit, self.fit_topic, 10)
        self.estimate_pub = self.create_publisher(WallEstimate, self.estimate_topic, 10)
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self._tf_static_cb, _TF_STATIC_QOS)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_cb, qos_profile_sensor_data)
        self.seed_sub = self.create_subscription(
            Float32, self.seed_topic, self._seed_cb, 10)
        if self.scene_enabled:
            self.planes_pub = self.create_publisher(String, '/perception/wall_planes', 10)
            self.virtual_pub = self.create_publisher(Float32, '/perception/virtual_wall', 10)
            self.virtual_debug_pub = self.create_publisher(
                String, '/perception/virtual_wall/debug', 10)
            self.create_subscription(
                Detection3DArray, str(param('detections_3d_topic', '/camera/detections_3d')),
                self._detections_cb, 10)
            self.create_subscription(
                String, '/perception/stereo_wall', self._stereo_wall_cb, 10)
        if self.wall_tracker is not None:
            self.walls_pub = self.create_publisher(MultiWallTrackArray, '/perception/walls', 10)
            self.walls_markers_pub = self.create_publisher(
                MarkerArray, '/perception/walls/markers', 10)
        # Best-effort on purpose -- see the module docstring's NO COUPLING TO
        # THE E-STOP section.
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'lidar_front_wall_node up: {self.scan_topic} -> {self.fit_topic}, '
            f'{self.estimate_topic}; sector +-{math.degrees(self.sector_half_angle_rad):.1f} deg, '
            f'odometry {self.odom_topic}, seed {self.seed_topic}'
            + ('; SCENE LABELS ON: detection sectors cut, /perception/wall_planes, '
               '/perception/virtual_wall' if self.scene_enabled else ''))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tf_static_cb(self, msg):
        for transform in msg.transforms:
            self._tf_buffer.set_transform_static(transform, 'lidar_front_wall_node')

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        yaw = _yaw_from_quaternion(pose.orientation)
        self.estimator.update_odometry(self._now(), pose.position.x, pose.position.y, yaw)
        self.virtual.update_pose(pose.position.x, pose.position.y, yaw)
        if self.wall_tracker is not None:
            stamp = msg.header.stamp
            self._poses.add(stamp.sec + stamp.nanosec * 1e-9,
                            pose.position.x, pose.position.y, yaw)
            if msg.header.frame_id:
                self._odom_frame = msg.header.frame_id

    def _detections_cb(self, msg):
        """Each detection's (x, y, half-width) in base_link, from /tf_static."""
        try:
            t = self._tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, Time()).transform
        except TransformException:
            return
        rot = quat_to_matrix(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        objs = []
        for det in msg.detections:
            p = det.bbox.center.position
            x, y, _ = rot @ (p.x, p.y, p.z) + (t.translation.x, t.translation.y,
                                               t.translation.z)
            objs.append((float(x), float(y), 0.5 * float(det.bbox.size.x)))
        stamp = msg.header.stamp
        self._detections = (self._now(), objs, stamp.sec + stamp.nanosec * 1e-9)

    def _stereo_wall_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if d.get('distance_m') is not None:
            self.virtual.update_stereo(
                self._now(), float(d['distance_m']), bool(d.get('fallback')))

    def _sectors(self, now, laser_pose, scan_stamp=None):
        if self._detections is None:
            return []
        objs = self._detections[1]
        if self.wall_tracker is not None and scan_stamp is not None:
            # Carried to the scan stamp on odometry (module docstring).
            det_stamp = self._detections[2]
            if scan_stamp - det_stamp > self.tracker_det_max_age:
                return []
            then, now_pose = self._poses.at(det_stamp), self._poses.at(scan_stamp)
            if then is None or now_pose is None:
                return []
            objs = mwt.propagate_objects(objs, then, now_pose)
        elif now - self._detections[0] > self.detection_max_age:
            return []
        return scene_labels.detection_sectors(objs, laser_pose[:2], self.detection_margin_rad)

    def _seed_cb(self, msg):
        # /costmap/front_clearance is from the front bumper since 2026-10-12;
        # this node's distances are from the base_link ORIGIN, so the bumper's
        # x goes back on (exact for a square-on wall). A non-positive value
        # stays non-positive: "no reading" to the estimator either way.
        value = float(msg.data)
        if math.isfinite(value) and value > 0.0:
            value += self.front_x
        self.estimator.update_seed(self._now(), value)

    def _laser_pose_for(self, frame_id):
        if self._laser_pose is not None and self._laser_pose[0] == frame_id:
            return self._laser_pose[1]
        try:
            stamped = self._tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except TransformException:
            return None
        t = stamped.transform
        pose = (t.translation.x, t.translation.y, _yaw_from_quaternion(t.rotation))
        self._laser_pose = (frame_id, pose)
        self.get_logger().info(
            f'{self.base_frame} <- {frame_id}: x={pose[0]:.3f} m y={pose[1]:.3f} m '
            f'yaw={math.degrees(pose[2]):.2f} deg')
        return pose

    def _scan_cb(self, msg):
        now = self._now()
        laser_pose = self._laser_pose_for(msg.header.frame_id)
        if laser_pose is None:
            fit = SectorFit(0, 0)
            reason = REASON_NO_TRANSFORM
            self.get_logger().warn(
                f'no {self.base_frame} <- {msg.header.frame_id} transform on /tf_static yet; '
                'publishing no_transform', throttle_duration_sec=5.0)
        elif self.scene_enabled:
            fit, reason = self._scene_fit(msg, laser_pose, now)
        else:
            fit = fit_forward_line(
                msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
                laser_pose, self.sector_half_angle_rad, self.inlier_distance_m,
                self.max_hypothesis_pairs)
            reason = classify_fit(
                fit, self.min_inlier_fraction, self.min_inlier_count_ratio, self.oblique_max_rad)
        step = self.estimator.step(now, fit, reason)
        self._publish(msg.header.stamp, fit, step)
        if self.scene_enabled:
            self._publish_virtual(msg.header.stamp, now, step)
        self._log_step(step)

    def _scene_fit(self, msg, laser_pose, now):
        """The scene-labels wall fit (module docstring, SCENE LABELS).

        Detection sectors are cut out of the wide forward sector first. At
        wall_planes_rate_hz the sequential wall lines are extracted from it
        (every other beam), published on /perception/wall_planes, and the
        nearest one the heading crosses becomes the tracked wall; every scan
        refits that wall on the full-resolution returns. The "sector" of the
        published WallLineFit is the wide sector in this mode. A fit over just
        the +-sector_half_angle_deg beams is not used: a person and a laptop
        in front of the wall cut all but ~3 of its 81 beams in M02 P002-R005.
        """
        stamp = msg.header.stamp
        scan_stamp = stamp.sec + stamp.nanosec * 1e-9
        sectors = self._sectors(now, laser_pose, scan_stamp)
        pts, usable, _, bearings = scene_labels.scan_points(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            laser_pose, self.planes_half_angle_rad, sectors, with_bearings=True)
        if now >= self._next_planes:
            self._next_planes = now + self.planes_period
            lines = scene_labels.sequential_lines(
                pts[::2], self.inlier_distance_m, self.wall_min_span_m,
                self.planes_min_inliers, self.planes_max_lines)
            self._seq_wall = scene_labels.heading_wall(
                lines, self.oblique_max_rad, pts, bearings)
            self._tracked_wall = self._seq_wall
            self.planes_pub.publish(String(data=json.dumps({
                'stamp': stamp.sec + stamp.nanosec * 1e-9, 'frame_id': self.base_frame,
                'sectors': [[round(a, 4), round(b, 4)] for a, b in sectors],
                'walls': [w.to_dict() for w in lines]})))
        if self.wall_tracker is not None:
            front = self._track_walls(msg, laser_pose, sectors, scan_stamp)
            if front is not None:
                self._tracked_wall, self._wall_source = front, 'tracker'
            elif self._wall_source == 'tracker':
                self._tracked_wall, self._wall_source = self._seq_wall, 'sequential_lines'
        wall = scene_labels.refit_wall(
            pts, self._tracked_wall, self.inlier_distance_m, 3.0 * self.inlier_distance_m)
        self._tracked_wall = wall
        if wall is None:
            return SectorFit(usable, len(pts)), REASON_TOO_FEW_RETURNS
        fit = SectorFit(usable, len(pts), wall.d, wall.normal_angle, wall.inliers, wall.rms)
        if wall.inliers < self.planes_min_inliers:
            return fit, REASON_LOW_INLIER_COUNT
        if (wall.span < self.wall_min_span_m
                or not scene_labels.wall_ahead(wall, pts, bearings)):
            return fit, REASON_SHORT_SPAN
        if abs(wall.normal_angle) > self.oblique_max_rad:
            return fit, REASON_OBLIQUE
        return fit, REASON_OK

    def _publish_virtual(self, stamp, now, step):
        v = self.virtual.step(now, step.estimate)
        expired = self.wall_tracker is not None and self._front_coast_expired
        valid = v.valid and not expired
        value = max(v.distance, MIN_FRONT_DISTANCE_M) if valid else -1.0
        self.virtual_pub.publish(Float32(data=float(value)))

        def num(x):
            return None if not math.isfinite(x) else round(x, 4)
        self.virtual_debug_pub.publish(String(data=json.dumps({
            'stamp': stamp.sec + stamp.nanosec * 1e-9, 'valid': valid,
            'distance_m': num(v.distance), 'origin': 'front', 'source': v.source,
            'disagree': v.disagree, 'stereo_fallback': v.stereo_fallback,
            'lidar_m': num(v.lidar_m), 'stereo_m': num(v.stereo_m),
            'lidar_reason': int(step.reason),
            'lidar_provenance': PROVENANCE_NAMES[step.estimate.provenance],
            'wall_source': (None if self.wall_tracker is None
                            else 'front_coast_expired' if expired else self._wall_source),
            'tracker_ms': num(self._tracker_ms)})))
        if v.disagree:
            self.get_logger().warn(
                f'virtual wall: LiDAR {v.lidar_m:.2f} m vs stereo {v.stereo_m:.2f} m from the '
                f'front differ by more than {self.virtual.disagree_m:.2f} m; trusting LiDAR',
                throttle_duration_sec=2.0)

    def _track_walls(self, msg, laser_pose, sectors, scan_stamp):
        """One tracker step. Returns the front track as a base_link WallLine, or None."""
        car = self._poses.at(scan_stamp)
        if car is None:
            self.get_logger().warn(
                f'wall tracker: no {self.odom_topic} pose at the scan stamp; skipped',
                throttle_duration_sec=5.0)
            return None
        t0 = time.perf_counter()
        lines = mwt.extract_lines(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            laser_pose, sectors)
        tracks = self.wall_tracker.step(
            [ln.transform(car, self._odom_frame) for ln in lines], scan_stamp, car)
        front = self.wall_tracker.front_track(car, self.front_x)
        if front is not None:
            self._front_id, self._front_coast_expired = front.id, False
        elif self._front_id is not None:
            # The front track went away: past the coast limit (or deleted)
            # means no wall until a new front track; anything else falls back.
            lost = self.wall_tracker.track(self._front_id)
            if lost is None or self.wall_tracker.coast_expired(lost):
                self._front_coast_expired = True
                self.get_logger().warn(
                    f'wall tracker: front track {self._front_id} unseen for more than '
                    f'{self.wall_tracker.p.front_max_coast_s:.2f} s: virtual wall -1 until '
                    'a new front track')
            self._front_id = None
        self._tracker_ms = (time.perf_counter() - t0) * 1e3
        self._log_tracker_cost(scan_stamp)
        self._publish_walls(msg.header.stamp, tracks, car)
        return None if front is None else mwt.to_wall_line(front, car)

    def _log_tracker_cost(self, stamp):
        self._tracker_cost.append(self._tracker_ms)
        if self._next_cost_log is None:
            self._next_cost_log = stamp + 30.0
        elif stamp >= self._next_cost_log:
            cost = sorted(self._tracker_cost)
            self.get_logger().info(
                f'wall tracker: {len(cost)} scans, per-scan ms p50 '
                f'{cost[len(cost) // 2]:.2f} p95 {cost[int(len(cost) * 0.95)]:.2f} '
                f'max {cost[-1]:.2f}')
            self._tracker_cost, self._next_cost_log = [], stamp + 30.0

    def _publish_walls(self, stamp, tracks, car):
        out = MultiWallTrackArray()
        out.header.stamp = stamp
        out.header.frame_id = self._odom_frame
        markers = MarkerArray()
        clear = Marker()
        clear.header = out.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for trk in tracks:
            bl = mwt.track_in_base_link(trk, car)
            label = self.wall_tracker.label(trk, car)
            d_perp, d_ray = mwt.front_distances(trk, car, self.front_x)
            (x0, y0), (x1, y1) = trk.endpoints()
            m = MultiWallTrack()
            m.id = trk.id
            m.label = label
            m.state = trk.state
            m.rho, m.theta = trk.rho, trk.theta
            m.rho_bl, m.theta_bl = float(bl.rho), float(bl.alpha)
            m.p0, m.p1 = Point(x=float(x0), y=float(y0)), Point(x=float(x1), y=float(y1))
            m.p0_bl = Point(x=float(bl.p_start[0]), y=float(bl.p_start[1]))
            m.p1_bl = Point(x=float(bl.p_end[0]), y=float(bl.p_end[1]))
            m.cov = [float(v) for v in trk.P.ravel()]
            m.hits = trk.hits
            m.age = float(self.wall_tracker._last_stamp - trk.born)
            m.d_perp_front, m.d_ray_front = float(d_perp), float(d_ray)
            out.tracks.append(m)
            line = Marker()
            line.header = out.header
            line.ns, line.id, line.type = 'walls', 2 * trk.id, Marker.LINE_LIST
            line.scale.x = 0.04
            line.color.a, line.color.g = 1.0, 0.6
            line.color.r = 1.0 if label == 'front' else 0.2
            line.pose.orientation.w = 1.0
            line.points = [m.p0, m.p1]
            text = Marker()
            text.header = out.header
            text.ns, text.id, text.type = 'walls', 2 * trk.id + 1, Marker.TEXT_VIEW_FACING
            text.pose.position = Point(x=float(0.5 * (x0 + x1)), y=float(0.5 * (y0 + y1)), z=0.3)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color.a = text.color.r = text.color.g = text.color.b = 1.0
            text.text = f'{trk.id}:{label} d={d_perp:.2f}'
            markers.markers += [line, text]
        self.walls_pub.publish(out)
        self.walls_markers_pub.publish(markers)

    def _publish(self, stamp, fit, step):
        fit_msg = WallLineFit()
        fit_msg.header.stamp = stamp
        fit_msg.header.frame_id = self.base_frame
        fit_msg.valid = step.reason == REASON_OK
        fit_msg.reason = step.reason
        fit_msg.distance = float(fit.distance)
        fit_msg.normal_angle = float(fit.normal_angle)
        fit_msg.inlier_count = fit.inlier_count
        fit_msg.valid_returns = fit.valid_returns
        fit_msg.sector_beams = fit.sector_beams
        fit_msg.inlier_fraction = float(fit.inlier_fraction) if fit.has_line else math.nan
        fit_msg.rms_residual = float(fit.rms_residual)
        fit_msg.predicted_distance = float(step.predicted_distance)
        fit_msg.innovation = float(step.innovation)
        self.fit_pub.publish(fit_msg)

        estimate = step.estimate
        estimate_msg = WallEstimate()
        estimate_msg.header.stamp = stamp
        estimate_msg.header.frame_id = self.base_frame
        estimate_msg.valid = estimate.valid
        estimate_msg.provenance = estimate.provenance
        estimate_msg.distance = float(estimate.distance)
        estimate_msg.normal_angle = float(estimate.normal_angle)
        estimate_msg.source_age = float(estimate.source_age)
        self.estimate_pub.publish(estimate_msg)

    def _log_step(self, step):
        acquired = step.reacquisition
        if acquired is not None:
            self.get_logger().info(
                f'wall acquired at {step.estimate.distance:.3f} m after '
                f'{PROVENANCE_NAMES[acquired.previous_provenance]} '
                f'({acquired.previous_distance:.3f} m): jump {acquired.jump:+.3f} m, '
                f'{acquired.since_last_measurement:.2f} s since the last accepted fit')
        if step.odometry_fresh != self._odometry_was_fresh:
            if step.odometry_fresh:
                self.get_logger().info(f'{self.odom_topic} fresh: dead reckoning available')
            else:
                self.get_logger().warn(
                    f'{self.odom_topic} missing or stale: dead reckoning and the '
                    'wrong-surface gate are OFF until it returns')
            self._odometry_was_fresh = step.odometry_fresh


def main(args=None):
    rclpy.init(args=args)
    node = LidarFrontWallNode()
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
