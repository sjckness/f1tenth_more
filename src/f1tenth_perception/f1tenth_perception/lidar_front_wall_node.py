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
  * /perception/virtual_wall (Float32, from the FRONT BUMPER) and
    /perception/virtual_wall/debug (JSON: state, source, flags). It is the
    single wall truth: with the switch on every wall consumer reads it
    (f1tenth_params.param_defaults.get_wall_distance_topic()).

    THE CONTRACT (since 2026-10-15; wall_fusion.front_wall_output, and
    WallEstimate.state carries the same verdict on the same tick):

      WALL     a FRONT track crosses the car-width strip ahead of the
               bumper. The topic carries its distance from the bumper.
      CLEAR    the sensors work and NOTHING is in the way. The topic carries
               wall_clear_distance_m (50 m) -- a large finite number, NOT a
               wall at that distance. An empty LiDAR strip is CLEAR.
      NO_DATA  the sensors are not telling us: no transform, no scan, no
               odometry to place it with. The topic carries -1.0 and the
               consumer holds (check_stop_condition.py). NOT "clear".

    THERE IS NO LONGER A STEREO FALLBACK. The stereo used to stand in when
    the LiDAR strip was empty, which on an open corridor made the SIDE wall
    a front wall at 1.1 m (corridor_phantom, 2026-10-15). Stereo supports a
    wall only through /perception/wall_observations and the tracker, where a
    wall must cross the strip to be FRONT at all. Objects never appear here:
    a wall distance is about walls, and the generic front_clearance safety
    check is separate and always on.

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
  * Minimum width for a wall. An observation shorter than
    wall_min_span_update_m is not a wall and never reaches the tracker;
    one shorter than wall_min_span_birth_m may only UPDATE an already
    confirmed track, never start one or feed a tentative one. The
    extraction's collinear merge (wall_merge_max_gap_m) reunifies a wall
    broken by an occluder first, so the span is the wall's, not the
    visible piece's. multi_wall_tracker's "WHY THE SPAN GATE IS
    HYSTERETIC" has the reasoning; the count of dropped observations is
    `short_obs` in /perception/virtual_wall/debug.
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
    publishes -1.0 (NO_DATA) on every scan until the tracker has a front track
    again; neither the sequential_lines wall nor the dead-reckoned value
    stands in for it meanwhile. A front track that stops being the
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

import numpy as np

from f1tenth_description.vehicle_frames import (
    base_front_x, front_side_limit_m, MIN_FRONT_DISTANCE_M)
from f1tenth_messages.msg import (
    MultiWallTrack, MultiWallTrackArray, StereoColumns, WallEstimate, WallLineFit,
    WallObservation, WallObservationArray)
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
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
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
from f1tenth_perception import scene_labels, scene_mask, wall_fusion
from f1tenth_perception.ground_plane import quat_to_matrix

# Same profile tf2_ros.TransformListener uses for /tf_static: the static
# broadcasters latch, so a late subscriber still receives every edge.
_TF_STATIC_QOS = QoSProfile(
    depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


#: Marker colours by role (r, g, b).
_ROLE_COLOUR = {'front': (1.0, 0.25, 0.1), 'left': (0.2, 0.9, 0.3),
                'right': (0.2, 0.5, 1.0), 'back': (0.6, 0.6, 0.6),
                # 'other' is real and tracked and is a reference for nothing,
                # so it is drawn -- dimly, and in nobody else's colour.
                'other': (0.35, 0.35, 0.4)}


def _stamp_sec(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _observation_array(stamp, frame_id, observations, n_rejected, n_disagree_columns):
    """Pack [wall_fusion.Observation] into a WallObservationArray."""
    msg = WallObservationArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.n_rejected = int(n_rejected)
    msg.n_disagree_columns = int(n_disagree_columns)
    for obs in observations:
        line = obs.line
        (ax, ay), (bx, by) = line.endpoints()
        item = WallObservation()
        item.header = msg.header
        item.nx, item.ny, item.d = float(line.nx), float(line.ny), float(line.d)
        item.p0 = Point(x=float(ax), y=float(ay), z=0.0)
        item.p1 = Point(x=float(bx), y=float(by), z=0.0)
        item.z_min, item.z_max = float(obs.z_min), float(obs.z_max)
        item.height_measured = bool(obs.height_measured)
        item.source = int(obs.source)
        item.confidence = float(obs.confidence)
        item.inliers = int(max(0, line.inliers))
        item.rms = float(line.rms)
        item.col_begin, item.col_end = int(obs.col_begin), int(obs.col_end)
        item.n_agree = int(obs.counts.get('agree', 0))
        item.n_lidar_only = int(obs.counts.get('lidar_only', 0))
        item.n_stereo_only = int(obs.counts.get('stereo_only', 0))
        item.n_disagree = int(obs.counts.get('disagree', 0))
        msg.observations.append(item)
    return msg


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
            delete_after_s=float(param('wall_tracker_delete_after_sec', 2.0)),
            coast_max_odom_m=float(param('wall_coast_max_odom_m', 1.0)),
            max_sigma_rho_m=float(param('wall_tracker_max_sigma_rho_m', 0.3)),
            max_published=int(param('wall_tracker_max_tracks', 4)),
            front_max_coast_s=float(param('wall_tracker_front_max_coast_sec', 0.3)),
            # Step 4: the observation noise model and what may be called FRONT.
            lidar_sigma_base_m=float(param('lidar_sigma_base_m', 0.02)),
            lidar_sigma_per_m=float(param('lidar_sigma_per_m', 0.005)),
            stereo_sigma_base_m=float(param('stereo_sigma_base_m', 0.03)),
            stereo_sigma_quad_per_m2=float(param('stereo_sigma_quad_per_m2', 0.012)),
            min_span_for_angle_m=float(param('min_span_for_angle_m', 0.3)),
            stereo_confirm_hits=int(param('stereo_confirm_hits', 5)),
            stereo_confirm_window=int(param('stereo_confirm_window', 7)),
            role_hysteresis_rad=float(param('role_hysteresis_rad', math.radians(5.0))),
            stereo_front_max_normal_rad=float(
                param('stereo_front_max_normal_rad', math.radians(30.0))),
            strip_half_width_m=float(
                param('front_strip_half_width_m', front_side_limit_m())),
            side_edge_half_width_m=float(get_value('swept_clearance_body_half_width_m')),
            # Minimum width for a wall: see multi_wall_tracker's own
            # "WHY THE SPAN GATE IS HYSTERETIC".
            min_span_birth_m=float(param('wall_min_span_birth_m', 1.0)),
            min_span_update_m=float(param('wall_min_span_update_m', 0.5)),
            merge_max_gap_m=float(param('wall_merge_max_gap_m', 1.5)),
            # Step 6 (E3): the geometric roles, the first-obstacle rule and
            # the negative evidence.
            back_x_m=float(param('wall_role_back_x_m', 0.0)),
            side_x_min_m=float(param('wall_role_side_x_min_m', -1.0)),
            side_x_max_m=float(param('wall_role_side_x_max_m', 3.0)),
            side_max_normal_rad=math.radians(
                float(param('wall_role_side_max_normal_deg', 45.0))),
            role_hysteresis_m=float(param('wall_role_hysteresis_m', 0.05)),
            occlusion_margin_m=float(param('wall_occlusion_margin_m', 0.10)),
            negative_tol_m=float(param('wall_negative_tol_m', 0.15)),
            negative_min_beams=int(param('wall_negative_min_beams', 5)),
            negative_pass_frac=float(param('wall_negative_pass_frac', 0.7)),
            negative_miss_to_kill=int(param('wall_negative_miss_to_kill', 5)))
        if self.use_wall_tracker and not self.scene_enabled:
            self.get_logger().error(
                'use_wall_tracker needs enable_scene_labels (the virtual wall it feeds '
                'exists only with scene labels): the multi-wall tracker stays OFF')
            self.use_wall_tracker = False
        # Step 5 (E2): the rectangle verification. The thresholds that DECIDE
        # are parameters; the sampling geometry (41 rows from the floor to
        # 2 m, the 0.4 m height weight) stays in wall_fusion.RectParams next
        # to the reasoning that sets it, because it is not a tuning knob --
        # changing it changes what "the wall's rectangle" means.
        self.rect_verify = bool(param('wall_rect_verify', True))
        rect_params = wall_fusion.RectParams(
            coverage_min=float(param('wall_rect_coverage_min', 0.25)),
            support_verify=float(param('wall_rect_support_verify', 0.5)),
            support_reject=float(param('wall_rect_support_reject', 0.2)),
            glass_beyond_share=float(param('wall_rect_glass_beyond_share', 0.7)),
            tol_base_m=float(param('wall_rect_tol_base_m', 0.15)),
            tol_frac=float(param('wall_rect_tol_frac', 0.05)))
        #: The z rows and their weights, built once: nothing about them
        #: depends on the scan or the wall (performance rules).
        self._rect_grid = wall_fusion.rect_grid(rect_params)
        self.wall_tracker = (mwt.MultiWallTracker(tracker_params, rect_params)
                             if self.use_wall_tracker else None)
        #: This cycle's rectangle verdicts by track id, for the debug payload.
        self._rect_verdicts = {}
        #: Objects whose centre is behind a confirmed wall this cycle. The
        #: FIRST-OBSTACLE rule is for WALLS ONLY: an object behind a wall is
        #: still an object and is still cut out of the scan, it is only
        #: FLAGGED. Nothing here publishes objects, so the flag is a count on
        #: the debug payload rather than a field on a message this node owns.
        self._objects_behind_wall = 0
        #: The car pose this cycle's tracks were placed with, for the payload.
        self._car_pose = None
        self._mask_classes = None     # the class mask Image, latest
        self._mask_depth = None       # the mono16 mm depth Image of the same tick
        self._rect_age = math.nan
        self._rect_weight = 0.0
        self._rect_ms = math.nan
        # The extraction's collinear merge has to reunify a wall broken by an
        # occluder BEFORE its span is measured, so its gap is set from the same
        # parameter block as the span thresholds.
        self.extraction_params = mwt.extraction_params(
            tracker_params.merge_max_gap_m,
            quality_gates=bool(param('wall_extract_quality_gates', True)))
        #: What each extraction gate dropped on the last scan, for the debug payload.
        self._extract_stats = {}
        self._poses = mwt.PoseHistory()
        self._odom_frame = 'odom'
        self._wall_source = 'sequential_lines'
        self._front_id = None       # id of the tracker's current front track
        self._front_d_ray = math.nan   # its distance from the bumper, this scan
        self._tracker_has_data = False  # could this cycle see at all (step 5)
        self._front_coast_expired = False   # module docstring, coast limit
        self._seq_wall = None       # the latest sequential_lines heading wall
        self._lines = []            # the latest sequential_lines extraction
        self._tracker_ms = math.nan
        self._tracker_cost = []     # per-scan tracker ms, logged every 30 s
        self._next_cost_log = None

        # ---- step 3: per-cycle fusion ---------------------------------------
        # The profile is carried to this cycle's moment on the odometry and
        # then WEIGHTED by its age, rather than being accepted or discarded on
        # a hard edge. See _fresh_profile and wall_fusion.age_weight.
        self.profile_max_age = float(param('stereo_profile_max_age_s', 0.6))
        self.profile_min_weight = float(param('stereo_profile_min_weight', 0.3))
        self.obs_tol_base_m = float(param('wall_obs_tol_base_m', 0.05))
        self.obs_tol_quad = float(param('wall_obs_tol_quad_per_m2', 0.012))
        self.obs_disagree_reject_frac = float(param('wall_obs_disagree_reject_frac', 0.5))
        self.obs_stereo_min_columns = int(param('wall_obs_stereo_min_columns', 8))
        self.wall_default_height = float(param('wall_default_height_m', 2.5))
        self.wall_clear_distance_m = float(get_value('wall_clear_distance_m'))
        self.wall_marker_thickness_m = float(param('wall_marker_thickness_m', 0.02))
        self.side_edge_half_width_m = float(get_value('swept_clearance_body_half_width_m'))

        declare_nice_param(self)
        apply_nice(self)

        self._tf_buffer = Buffer()
        self._laser_pose = None  # (scan frame_id, (x, y, yaw) in base_frame)
        # Step 3 (per-cycle fusion) inputs, both cached and aged, never waited on.
        self._stereo_columns = None   # (receipt time, StereoColumns)
        self._wall_cloud = None       # (receipt time, PointCloud2)
        self._camera_tf = None        # (frame_id, (R, t))
        self._laser_z_cached = None
        self._odometry_was_fresh = None
        self._stereo_age = math.nan     # this cycle's profile age [s]
        self._stereo_weight = 0.0       # and what it was worth (age_weight)

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
        if self.scene_enabled:
            # The per-cycle fusion (step 3): the stereo's column profile and
            # its WALL cloud, cross-checked against this node's own scan and
            # line fits. The cloud is only used for stereo-ONLY line fits; the
            # profile is what every column decision is made on.
            self.observations_pub = self.create_publisher(
                WallObservationArray, '/perception/wall_observations', 10)
            self.create_subscription(
                StereoColumns, '/perception/stereo_columns', self._stereo_columns_cb, 10)
            self.create_subscription(
                PointCloud2, '/perception/wall_points', self._wall_points_cb,
                qos_profile_sensor_data)
        if self.scene_enabled and self.rect_verify:
            # The class mask and the depth it was computed from, as a PAIR on
            # one stamp (front_clearance_node._publish_scene_mask_classes).
            # Subscribing is what makes that node publish them at all, so a
            # stack with the verification off pays nothing for it.
            self.create_subscription(
                Image, '/perception/scene_labels/classes', self._mask_classes_cb,
                qos_profile_sensor_data)
            self.create_subscription(
                Image, '/perception/scene_labels/depth_mm', self._mask_depth_cb,
                qos_profile_sensor_data)
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
        if self.wall_tracker is not None or self.scene_enabled:
            # Also with the tracker off: the per-cycle fusion needs these two
            # poses to carry the scan to the camera's moment (_compensate).
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

    def _stereo_columns_cb(self, msg):
        self._stereo_columns = (self._now(), msg)

    def _wall_points_cb(self, msg):
        self._wall_cloud = (self._now(), msg)

    def _mask_classes_cb(self, msg):
        self._mask_classes = msg

    def _mask_depth_cb(self, msg):
        self._mask_depth = msg

    def _mask_pair(self, scan_stamp):
        """(labels, depth_mm, age, weight) of the latest mask/depth pair, or None.

        The two images are published together on one tick and carry the depth
        frame's own stamp, so a pair whose stamps differ is a half-arrived
        tick and is not used: a mask paired with another tick's depth would
        put every WALL pixel at the wrong distance, which is the one error
        this verification cannot detect from the inside.

        Age is on the IMAGE stamp, never arrival, and is weighted by the same
        wall_fusion.age_weight the column profile uses -- it is the same
        camera at the same moment, and there is no second staleness rule.

        Neither image is copied: np.frombuffer views the message's own bytes,
        and the depth stays mono16 millimetres all the way into
        verify_rectangles, which scales only the samples it gathers.
        """
        cls_msg, dep_msg = self._mask_classes, self._mask_depth
        if cls_msg is None or dep_msg is None:
            return None
        stamp = _stamp_sec(cls_msg.header)
        if abs(stamp - _stamp_sec(dep_msg.header)) > 1e-6:
            self.get_logger().warn(
                'class mask and depth carry different stamps: the rectangle '
                'verification is skipped this cycle', throttle_duration_sec=5.0)
            return None
        self._rect_age = scan_stamp - stamp
        self._rect_weight = wall_fusion.age_weight(
            self._rect_age, self.profile_max_age, self.profile_min_weight)
        if self._rect_weight <= 0.0:
            return None
        labels = np.frombuffer(cls_msg.data, dtype=np.uint8).reshape(
            cls_msg.height, cls_msg.width)
        depth = np.frombuffer(dep_msg.data, dtype=np.uint16).reshape(
            dep_msg.height, dep_msg.width)
        return labels, depth, self._rect_age, self._rect_weight

    def _verify_tracks(self, car, scan_stamp, profile):
        """Fold this cycle's rectangle verdicts into the tracks. {id: verdict} out.

        The intrinsics come from the StereoColumns profile of the same tick,
        rescaled by the width ratio if the mask is a different resolution;
        the camera frame comes from the mask's own header, so the TF is
        looked up for the frame the pixels are actually in.
        """
        self._rect_ms = math.nan
        self._rect_age, self._rect_weight = math.nan, 0.0
        if not self.rect_verify or self.wall_tracker is None:
            return {}
        tracks = list(self.wall_tracker.tracks)
        pair = self._mask_pair(scan_stamp) if tracks else None
        if pair is None or profile is None or int(profile.width) < 1:
            self.wall_tracker.apply_rect_verdicts({})
            return {}
        labels, depth, _age, _weight = pair
        cam = self._camera_transform(self._mask_classes.header.frame_id)
        if cam is None:
            self.wall_tracker.apply_rect_verdicts({})
            return {}
        rot, trans = cam
        scale = labels.shape[1] / float(profile.width)
        t0 = time.perf_counter()
        segments = [mwt.segment_in_base_link(trk, car) for trk in tracks]
        verdicts = wall_fusion.verify_rectangles(
            segments, labels, depth,
            float(profile.fx) * scale, float(profile.fy) * scale,
            float(profile.cx) * scale, float(profile.cy) * scale,
            rot, trans, laser_z=self._laser_z(), p=self.wall_tracker.rect,
            grid=self._rect_grid, depth_scale=0.001)
        by_id = {trk.id: verdict for trk, verdict in zip(tracks, verdicts)}
        self.wall_tracker.apply_rect_verdicts(by_id)
        self._rect_ms = (time.perf_counter() - t0) * 1e3
        return by_id

    def _camera_transform(self, frame_id):
        """(R, t) taking that camera frame's points into base_frame, or None."""
        if not frame_id:
            return None
        cached = self._camera_tf
        if cached is not None and cached[0] == frame_id:
            return cached[1]
        try:
            t = self._tf_buffer.lookup_transform(self.base_frame, frame_id, Time()).transform
        except TransformException:
            return None
        pose = (quat_to_matrix(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                np.array([t.translation.x, t.translation.y, t.translation.z]))
        self._camera_tf = (frame_id, pose)
        return pose

    def _publish_observations(self, stamp, scan_stamp, lines, pts):
        """Step 3: the stateless per-cycle fusion -> /perception/wall_observations.

        Published EVERY cycle, including empty: "both sensors looked and there
        is no wall" is the answer an open corridor has to be able to give, and
        a consumer must be able to tell it from "the node stopped".
        """
        profile, self._stereo_age, self._stereo_weight = self._fresh_profile(scan_stamp)
        cam = self._camera_transform(profile.camera_frame) if profile is not None else None
        if profile is None or cam is None or profile.width < 1:
            self._stereo_weight = 0.0
            # No fresh stereo at all: every LiDAR line stands on its own, which
            # is exactly what SOURCE_LIDAR means. Not a reason to publish
            # nothing, and not a reason to invent a stereo wall.
            observations = [wall_fusion.Observation(
                line, 0.0, self.wall_default_height, False, wall_fusion.SOURCE_LIDAR,
                wall_fusion.confidence(wall_fusion.SOURCE_LIDAR, line.inliers,
                                       line.rms, 0, 0))
                for line in lines]
            self.observations_pub.publish(_observation_array(
                stamp, self.base_frame, observations, 0, 0))
            return observations

        rot, trans = cam
        k = (float(profile.fx), float(profile.fy), float(profile.cx), float(profile.cy))
        width = int(profile.width)

        image_stamp = _stamp_sec(profile.header)

        def project(points):
            """Columns of base_link points taken at the IMAGE's own moment."""
            return scene_mask.project_to_image(points, *k, rot, trans)

        def project_from_scan(points):
            """Columns of base_link points taken at the SCAN's moment.

            Everything on the LiDAR side of this cycle -- the returns and the
            fitted lines alike -- was measured at the scan stamp, while the
            profile describes the image stamp. With the profile now accepted up
            to stereo_profile_max_age_s (0.6 s) rather than 0.15, that interval
            is long enough to matter: this car's minimum radius is about 0.6 m,
            so 0.6 s can be 28 degrees of yaw, and at fx 262 px that is a
            128-COLUMN shift -- wider than most walls in the frame, and enough
            to turn a whole wall's agreement into DISAGREE.
            """
            return project(self._compensate(points, scan_stamp, image_stamp))

        # The scan is a plane at the laser's own height; the cross-check only
        # uses x and y, but the projection needs the real height to land on
        # the right columns through a pitched mount.
        scan_xyz = np.column_stack([pts, np.full(len(pts), self._laser_z())]) \
            if len(pts) else np.empty((0, 3))
        scan_xyz = self._compensate(scan_xyz, scan_stamp, image_stamp)
        scan_uv = project(scan_xyz)
        scan_depth = ((scan_xyz - trans) @ rot)[:, 2] if len(scan_xyz) else np.empty(0)

        z_lo = np.asarray(profile.z_min, dtype=float)
        z_hi = np.asarray(profile.z_max, dtype=float)
        check = wall_fusion.cross_check_profile(
            scan_uv[:, 0], scan_depth, np.asarray(profile.distance_m, dtype=float), width,
            tol_base_m=self.obs_tol_base_m, tol_quad_per_m2=self.obs_tol_quad)
        observations, rejected = wall_fusion.observations_from_lines(
            lines, check, project_from_scan, width, z_lo, z_hi,
            default_height=self.wall_default_height,
            disagree_reject_frac=self.obs_disagree_reject_frac,
            stereo_weight=self._stereo_weight)
        observations += wall_fusion.stereo_only_observations(
            check, self._cloud_columns(project), width,
            min_span_columns=self.obs_stereo_min_columns,
            inlier_m=self.inlier_distance_m, min_span_m=self.wall_min_span_m,
            default_height=self.wall_default_height, z_min=z_lo, z_max=z_hi)
        n_disagree = int(np.count_nonzero(check['status'] == wall_fusion.STATUS_DISAGREE))
        if rejected:
            self.get_logger().warn(
                f'{rejected} wall observation(s) rejected: the LiDAR and the stereo '
                f'contradict each other over {n_disagree} column(s). A run of this '
                'means the extrinsic or the depth is wrong, not that the wall moved.',
                throttle_duration_sec=5.0)
        self.observations_pub.publish(_observation_array(
            stamp, self.base_frame, observations, rejected, n_disagree))
        return observations

    def _fresh_profile(self, scan_stamp):
        """(profile, age, weight) of the latest StereoColumns, or (None, age, 0).

        THE GATE USED TO BE A HARD EDGE and it cost most of the camera's
        evidence. A profile within the old 0.15 s pairing gate counted
        in full and one a millisecond older counted not at all, so on
        corridor10m_20261015T171753 only 44 of 238 live cycles used the camera
        at all -- not because it had nothing to say, but because it had said
        it slightly too early. Worse, the edge made a 0.14 s profile and a
        0.01 s one equally trusted, which is the same mistake in the other
        direction.

        Now the profile is accepted up to stereo_profile_max_age_s and
        WEIGHTED by its age (wall_fusion.age_weight): the geometry is carried
        to the right moment on the odometry by the caller, and the weight
        covers what that cannot -- the scene moving, the odometry's own error
        over the interval, and the stereo's own staleness. Past the limit the
        cycle is LiDAR only, as before.

        The age is on the camera IMAGE stamp the message carries, never its
        arrival: on this recording the writer ran 0.4-4.5 s behind the sensors.
        """
        held = self._stereo_columns
        if held is None:
            return None, math.nan, 0.0
        age = scan_stamp - _stamp_sec(held[1].header)
        weight = wall_fusion.age_weight(age, self.profile_max_age,
                                        self.profile_min_weight)
        if weight > 0.0:
            return held[1], age, weight
        self.get_logger().warn(
            f'stereo columns {age:+.3f} s from the scan (limit '
            f'{self.profile_max_age:.3f} s): this cycle is LiDAR only',
            throttle_duration_sec=5.0)
        return None, age, 0.0

    def _compensate(self, points, scan_stamp, image_stamp):
        """Carry base_link points from the scan's moment to the image's moment.

        The two sensors are up to stereo_profile_max_age_s apart and the car
        turns between them. At 0.5 m/s the translation is a harmless 30 cm, but
        the YAW is not: this car's minimum radius is about 0.6 m, so 0.6 s can
        be 28 degrees, which at fx 262 px is a 128-COLUMN shift -- wider than
        the width of most walls in the frame, and it would turn agreement into
        DISAGREE for a whole wall.

        Both poses come from /odometry/filtered (PoseHistory), so this is
        exact to the odometry: p_image = pose_image^-1 (pose_scan p). With no
        pose at either stamp the points are used uncompensated, which the age
        gate already bounds.
        """
        if abs(image_stamp - scan_stamp) < 1e-4 or len(points) == 0:
            return points
        pose_scan = self._poses.at(scan_stamp)
        pose_image = self._poses.at(image_stamp)
        if pose_scan is None or pose_image is None:
            self.get_logger().warn(
                'no odometry pose to carry the scan to the image stamp; the '
                'cross-check runs uncompensated', throttle_duration_sec=10.0)
            return points
        xy = pose_image.inverse().transform_points(
            pose_scan.transform_points(np.asarray(points)[:, :2]))
        out = np.array(points, dtype=float, copy=True)
        out[:, :2] = xy
        return out

    def _camera_transform(self, frame_id):
        """(R, t) taking that camera frame's points into base_frame, or None."""
        if not frame_id:
            return None
        cached = self._camera_tf
        if cached is not None and cached[0] == frame_id:
            return cached[1]
        try:
            t = self._tf_buffer.lookup_transform(self.base_frame, frame_id, Time()).transform
        except TransformException:
            return None
        pose = (quat_to_matrix(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                np.array([t.translation.x, t.translation.y, t.translation.z]))
        self._camera_tf = (frame_id, pose)
        return pose

    def _laser_z(self):
        """The scan plane's height [m] in base_frame: the laser's own mount z."""
        if self._laser_z_cached is None:
            self._laser_z_cached = 0.0
            if self._laser_pose is not None:
                try:
                    t = self._tf_buffer.lookup_transform(
                        self.base_frame, self._laser_pose[0], Time()).transform
                    self._laser_z_cached = float(t.translation.z)
                except TransformException:
                    pass
        return self._laser_z_cached

    def _cloud_columns(self, project):
        """column_points(c0, c1) over the latest WALL cloud, for stereo-only fits.

        The cloud is decimated and is used ONLY to fit a line through a span
        the stereo alone reports; every column decision is made on the full
        resolution profile.
        """
        held = self._wall_cloud
        if held is None:
            return lambda a, b: None
        pts = read_points_numpy(held[1], field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        if pts.shape[0] == 0:
            return lambda a, b: None
        us = project(pts)[:, 0]
        finite = np.isfinite(us)

        def column_points(a, b):
            sel = finite & (us >= a) & (us < b)
            return pts[sel] if np.any(sel) else None
        return column_points

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
        # The virtual wall first: its WALL / CLEAR / NO_DATA verdict is what
        # the WallEstimate carries, so the two can never disagree.
        state = (self._publish_virtual(msg.header.stamp, now, step)
                 if self.scene_enabled else None)
        self._publish(msg.header.stamp, fit, step, state)
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
            self._lines = lines
        if self.wall_tracker is not None:
            # With the tracker on, the observations are built from ITS
            # per-scan extraction (adaptive breakpoints, IEPF, a covariance
            # per segment) rather than the 5 Hz sequential_lines above, and
            # the tracker is then fed exactly what went out on the topic.
            front = self._track_walls(msg, laser_pose, sectors, scan_stamp, stamp, pts)
            if front is not None:
                self._tracked_wall, self._wall_source = front, 'tracker'
            elif self._wall_source == 'tracker':
                self._tracked_wall, self._wall_source = self._seq_wall, 'sequential_lines'
        else:
            self._publish_observations(stamp, scan_stamp, self._lines, pts)
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
        """The one front-wall distance, under the WALL / CLEAR / NO_DATA contract.

        See wall_fusion.front_wall_output for the contract itself. With the
        tracker on, the ONLY thing that can produce a WALL is a FRONT track:
        a confirmed wall whose segment crosses the car's strip. There is no
        longer any path from "the LiDAR strip is empty" to "the stereo plane
        is the wall" -- stereo supports a wall only by going through the
        observations and the tracker, and an empty strip is CLEAR.
        """
        if self.wall_tracker is not None:
            has_data = self._tracker_has_data and not self._front_coast_expired
            state, value = wall_fusion.front_wall_output(
                has_data, self._front_d_ray, self.wall_clear_distance_m,
                MIN_FRONT_DISTANCE_M)
            v = self.virtual.step(now, step.estimate)
        else:
            # Without the tracker there are no observations and so no FRONT
            # track; the estimator's own fit is all there is. The contract is
            # the same, and an unusable fit is NO_DATA rather than a guess.
            v = self.virtual.step(now, step.estimate)
            state, value = wall_fusion.front_wall_output(
                v.valid, v.distance if v.valid else None,
                self.wall_clear_distance_m, MIN_FRONT_DISTANCE_M)
        self.virtual_pub.publish(Float32(data=float(value)))

        def num(x):
            return None if x is None or not math.isfinite(x) else round(x, 4)
        self.virtual_debug_pub.publish(String(data=json.dumps({
            'stamp': stamp.sec + stamp.nanosec * 1e-9,
            'state': wall_fusion.STATE_NAMES[state],
            'valid': state != wall_fusion.STATE_NO_DATA,
            'distance_m': num(value), 'origin': 'front', 'source': v.source,
            'front_d_ray_m': num(self._front_d_ray),
            'clear_distance_m': self.wall_clear_distance_m,
            'lidar_m': num(v.lidar_m), 'stereo_m': num(v.stereo_m),
            'lidar_reason': int(step.reason),
            'lidar_provenance': PROVENANCE_NAMES[step.estimate.provenance],
            'wall_source': (None if self.wall_tracker is None
                            else 'front_coast_expired' if self._front_coast_expired
                            else self._wall_source),
            # Observations the span gate dropped as too short to be a wall. A
            # run of these says the room is full of short faces, or that
            # wall_merge_max_gap_m is too tight to reunify one wall.
            'short_obs': (None if self.wall_tracker is None
                          else int(self.wall_tracker.n_short_dropped)),
            # What each EXTRACTION quality gate dropped this scan (seed size,
            # residual rms, fill ratio, support). The span gate above only
            # ever sees what survived these.
            'extract_dropped': (dict(self._extract_stats)
                                if self._extract_stats else None),
            # The profile's age and what it was worth, every cycle: "no fresh
            # stereo profile" was never the whole truth, and the difference
            # between 0.16 s and 2.5 s is the difference between a tuning
            # question and a dead camera.
            'stereo_age_s': num(self._stereo_age),
            'stereo_weight': num(self._stereo_weight),
            'stereo_max_age_s': self.profile_max_age,
            'tracker_ms': num(self._tracker_ms),
            # The rectangle verification: its own age, weight and cost, and
            # how many tracks it had anything to say about this cycle.
            'rect_ms': num(self._rect_ms),
            'rect_age_s': num(self._rect_age),
            'rect_weight': num(self._rect_weight),
            'rect_informative': sum(
                1 for v in self._rect_verdicts.values()
                if v['coverage'] >= self.wall_tracker.rect.coverage_min
            ) if self.wall_tracker is not None else None,
            # E3: every live track's role and verdict, what the negative
            # evidence killed, and how many objects are behind a confirmed
            # wall. This list is where the rectangle verdict, the occlusion
            # flag and the miss count live (see _publish_walls).
            'tracks': self._track_payload(),
            'negative_killed': (None if self.wall_tracker is None
                                else int(self.wall_tracker.n_negative_killed)),
            'objects_behind_wall': self._objects_behind_wall})))
        return state

    def _track_walls(self, msg, laser_pose, sectors, scan_stamp, stamp, pts):
        """One tracker step. Returns the front track as a base_link WallLine, or None.

        Since step 4 the tracker consumes ONLY wall observations: this
        extracts the scan's segments, puts them through the per-cycle fusion
        (which publishes them), and hands the tracker the result. There is no
        other way into it, so the stereo cannot enter as a fallback distance
        behind its back.
        """
        car = self._poses.at(scan_stamp)
        if car is None:
            # No pose to place the walls with: NO_DATA, not "clear".
            self._front_d_ray = math.nan
            self._tracker_has_data = False
            self.get_logger().warn(
                f'wall tracker: no {self.odom_topic} pose at the scan stamp; skipped',
                throttle_duration_sec=5.0)
            return None
        self._tracker_has_data = True
        self._car_pose = car
        t0 = time.perf_counter()
        self._extract_stats = {}
        lines = mwt.extract_lines(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            laser_pose, sectors, params=self.extraction_params,
            stats=self._extract_stats)
        observations = self._publish_observations(
            stamp, scan_stamp, [mwt.line_to_wall_line(ln) for ln in lines], pts)
        # The occlusion rays start at the LASER, not at base_link: 30 cm of
        # lever arm decides whether a wall is behind another one at 15 m.
        self.wall_tracker.laser_xy = laser_pose[:2]
        self._objects_behind_wall = self._count_objects_behind_walls(
            car, laser_pose, scan_stamp)
        # The negative evidence sees exactly the beams the extraction saw,
        # object sectors cut and all (mwt.scan_in_base_link): a beam cut
        # because an object stands there is evidence about the object.
        neg_ranges, neg_bearings, _angles = mwt.scan_in_base_link(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
            msg.range_max, laser_pose, sectors)
        # The roles are DEFERRED: the rectangle runs on the tracks the step
        # just updated and its verdicts are folded in before anything is asked
        # of the roles -- a REJECTED wall is not a reference wall on the same
        # cycle it was rejected, not on the next one -- and deciding them
        # before and after would pay for the role and occlusion pass twice.
        self.wall_tracker.step(observations, scan_stamp, car,
                               scan=(neg_ranges, neg_bearings, laser_pose[:2]),
                               defer_roles=True)
        self._rect_verdicts = self._verify_tracks(
            car, scan_stamp, self._stereo_columns[1] if self._stereo_columns else None)
        self.wall_tracker.update_roles(car)
        tracks = self.wall_tracker.published(car)
        front = self.wall_tracker.front_track(car, self.front_x)
        self._front_d_ray = math.nan
        if front is not None:
            _perp, self._front_d_ray = mwt.front_distances(
                front, car, self.front_x, self.wall_tracker.p.ray_margin_m)
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

    def _count_objects_behind_walls(self, car, laser_pose, scan_stamp):
        """How many propagated object centres lie behind a CONFIRMED wall.

        The same ray test the walls use, and deliberately only a flag: an
        object behind a wall is still an object, its sector is still cut out
        of the scan, and nothing about how it is handled changes. What it is
        NOT is a reason to doubt the object, which is why this counts rather
        than filters.
        """
        if self._detections is None or self.wall_tracker is None:
            return 0
        objs = self._detections[1]
        then = self._poses.at(self._detections[2])
        if not objs or then is None:
            return 0
        objs = mwt.propagate_objects(objs, then, car)
        confirmed = [mwt.segment_in_base_link(t, car)
                     for t in self.wall_tracker.tracks if t.ever_confirmed]
        if not confirmed:
            return 0
        pts = np.array([[x, y] for x, y, _r in objs], dtype=float)
        blocked = mwt.ray_blocked_by(pts, laser_pose[:2], confirmed,
                                     self.wall_tracker.p.occlusion_margin_m)
        return int(np.count_nonzero(blocked))

    def _track_payload(self):
        """Every LIVE track as a dict, for /perception/virtual_wall/debug.

        Not only the published four: the whole point of 'other' and of the
        rectangle's REJECTED is that a track can be real, tracked and a
        reference for nothing, and a payload that showed only the published
        ones would hide exactly those.
        """
        car = self._car_pose
        if self.wall_tracker is None or car is None:
            return None
        def num(x):
            return None if x is None or not math.isfinite(x) else round(float(x), 4)
        out = []
        for trk in self.wall_tracker.tracks:
            bl = mwt.track_in_base_link(trk, car)
            _perp, d_ray = mwt.front_distances(trk, car, self.front_x)
            out.append({
                'id': int(trk.id), 'role': trk.role,
                'state': mwt.STATE_NAMES.get(trk.state), 'hits': int(trk.hits),
                'age_s': num((self.wall_tracker._last_stamp or trk.born) - trk.born),
                'length_m': num(trk.length), 'rho_bl': num(bl.rho),
                'd_ray_front': num(d_ray),
                # The endpoints, so a debug view can draw a track the publish
                # cap kept off /perception/walls -- which is most of the ones
                # worth looking at.
                'p0_bl': [num(bl.p_start[0]), num(bl.p_start[1])],
                'p1_bl': [num(bl.p_end[0]), num(bl.p_end[1])],
                'verify': wall_fusion.VERIFY_NAMES[trk.verify],
                'glass': bool(trk.glass), 'occluded': bool(trk.occluded),
                'miss': int(trk.miss), 'source': wall_fusion.SOURCE_NAMES[trk.source],
                'rect_coverage': num(trk.rect_coverage),
                'rect_support': num(trk.rect_support),
                'rect_beyond_share': num(trk.rect_beyond_share),
                'is_front': bool(self.wall_tracker.is_front(trk, car, self.front_x)),
            })
        return out

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
            # Step 4 additions: what the evidence was, how tall, which side,
            # and whether this is the wall actually in the way.
            m.role = label
            m.source = int(trk.source)
            m.confidence = float(trk.confidence)
            m.z_min, m.z_max = float(trk.z_min), float(trk.z_max)
            m.height_measured = bool(trk.height_measured)
            m.d_side = (float(mwt.side_distance(trk, car, self.side_edge_half_width_m))
                        if label in ('left', 'right') else math.nan)
            m.is_front = bool(self.wall_tracker.is_front(trk, car, self.front_x))
            # The rectangle verdict, the occlusion flag and the negative-
            # evidence count are NOT fields on this message, deliberately:
            # appending to a .msg makes every bag already recorded with the
            # old definition undeserialisable (CDR is positional), and these
            # recordings are the evidence base. They go on
            # /perception/virtual_wall/debug's `tracks` list instead, which is
            # free-form JSON and costs no past run its readability.
            out.tracks.append(m)
            # A wall is a VERTICAL PLANE, so it is drawn as one: a thin slab
            # from z_min to z_max, not a line on the floor. The height is the
            # observed one when the stereo measured it and the default
            # otherwise, which is visible at a glance because an assumed wall
            # is always exactly wall_default_height_m tall.
            (ax, ay), (bx, by) = (x0, y0), (x1, y1)
            length = math.hypot(bx - ax, by - ay)
            yaw = math.atan2(by - ay, bx - ax)
            slab = Marker()
            slab.header = out.header
            slab.ns, slab.id, slab.type = 'walls', 2 * trk.id, Marker.CUBE
            slab.action = Marker.ADD
            slab.pose.position = Point(x=float(0.5 * (ax + bx)), y=float(0.5 * (ay + by)),
                                       z=float(0.5 * (trk.z_min + trk.z_max)))
            slab.pose.orientation.z = math.sin(yaw / 2.0)
            slab.pose.orientation.w = math.cos(yaw / 2.0)
            slab.scale.x = max(float(length), 0.01)
            slab.scale.y = self.wall_marker_thickness_m
            slab.scale.z = max(float(trk.z_max - trk.z_min), 0.01)
            r, g, b = _ROLE_COLOUR.get(label, _ROLE_COLOUR['other'])
            slab.color.r, slab.color.g, slab.color.b = r, g, b
            # A wall only assumed to be this tall is drawn fainter than one
            # whose height was actually seen.
            slab.color.a = 0.65 if trk.height_measured else 0.35
            text = Marker()
            text.header = out.header
            text.ns, text.id, text.type = 'walls', 2 * trk.id + 1, Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=float(0.5 * (ax + bx)), y=float(0.5 * (ay + by)),
                                       z=float(trk.z_max + 0.15))
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color.a = text.color.r = text.color.g = text.color.b = 1.0
            text.text = (f'{trk.id} {label} {d_perp:.2f}m '
                         f'{wall_fusion.SOURCE_NAMES[trk.source]} '
                         f'{mwt.STATE_NAMES[trk.state]}')
            markers.markers += [slab, text]
        self.walls_pub.publish(out)
        self.walls_markers_pub.publish(markers)

    def _publish(self, stamp, fit, step, state=None):
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
        # The wall contract's verdict (step 5), from the same tick's
        # _publish_virtual so the two can never disagree. Without scene
        # labels there is no contract to report and it stays NO_DATA.
        estimate_msg.state = int(
            wall_fusion.STATE_NO_DATA if state is None else state)
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
