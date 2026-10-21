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
  odom_max_age_sec        0.3   /odometry/filtered runs at 20 Hz since cpu-
                                cleanup R2 (50 before, longest recorded gap
                                136 ms, p99.9 120 ms); 0.3 s is 6 periods.
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

      WALL     a FRONT track's MEASURED segment meets the travel strip ahead
               of the bumper. The topic carries the distance from the bumper
               to where the strip meets it.
      CLEAR    the sensors work and NO FRONT track is in the way. The topic
               carries wall_clear_distance_m (50 m) -- a large finite number,
               NOT a wall at that distance. An empty LiDAR strip is CLEAR;
               so is a strip with returns nearer than front_clear_range_m
               and no FRONT track (debug front_reason 'unexplained_returns':
               objects are front_clearance's and IsProximityTooClose's, and
               a wall is never invented from raw returns).
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
    extraction's collinear merge reunifies a wall broken by an occluder
    first, so the span is the wall's, not the visible piece's -- across a
    gap SHORTER than door_width_min_m only (multi_wall_tracker's THE
    COLLINEAR GAP RULE: one number for the extraction's merge, the
    association, the track merge and the extent). multi_wall_tracker's "WHY THE SPAN GATE IS
    HYSTERETIC" has the reasoning; the count of dropped observations is
    `short_obs` in /perception/virtual_wall/debug.
  * A track is CONFIRMED at multi_wall_tracker's THE CONFIRMATION LEVEL,
    the wall_confirm_* parameters (2026-10-21): wall_confirm_min_consecutive_hits
    cycles in a row with a LiDAR-backed hit, a measured segment of at least
    wall_confirm_min_length_m, and the distance and direction of those
    observations steady within wall_confirm_max_dist_jitter_m and
    wall_confirm_max_angle_jitter_deg. Sticky once reached. With
    wall_debug_tracks on, each track's `confirm_reason` in the debug payload
    says which test a tentative one is still failing.
  * Up to 4 wall tracks in odom go out on /perception/walls
    (f1tenth_messages/MultiWallTrackArray), with line and text markers on
    /perception/walls/markers. Roles on the wire are front | back | left |
    right | unknown ('other' until 2026-10-19; readers accept both), and a
    TENTATIVE track is 'unknown' whatever its geometry (mwt.reported_role).
  * The heading wall: the front track (multi_wall_tracker.front_track and
    its WHAT FRONT REQUIRES: confirmed, not contradicted by the travel
    strip's own beams, its MEASURED segment meeting the strip, a LiDAR fit in
    the last front_lidar_recent_cycles cycles or rect VERIFIED, and a
    coast of at most wall_tracker_front_max_coast_sec / _m over a strip that
    does not see through it) is the wall that refit_wall, WallEstimator
    (wrong-surface gate included) and VirtualWall consume. With no front
    track, it is the sequential_lines wall, as before -- for the estimator
    only: the contract's value comes from the tracker alone.
  * No front track is CLEAR, however it was lost (block 3, 2026-10-16). The
    old coast latch published -1.0 (NO_DATA) until a new front track
    appeared, which HOLDS the car, and in an open hall that was
    indefinitely. NO_DATA now means only what the contract says: no pose at
    the scan stamp, no laser transform. The FRONT distance is where the
    travel strip meets the measured segment, from the bumper.
  * /perception/virtual_wall/debug says why: wall_source 'tracker' or
    'sequential_lines', front_reason (multi_wall_tracker.front_reason's
    codes, 'front', 'no_candidate', 'unexplained_returns' or 'no_pose'),
    strip_range_m (the nearest LiDAR return in the strip, from the bumper),
    the gap / chassis rejection counters, and tracker_ms.
  * The scan pose is /odometry/filtered interpolated at the scan stamp, the
    pose the EKF also publishes as odom -> base_link.

DOOR DETECTION (door_detection_enabled, stack_params.yaml; default false)
--------------------------------------------------------------------------
Off by default since 2026-10-20: stack_params.yaml
lidar_front_wall_door_detection_enabled and the node's own fallback agree
(TestDoorParamsCoupling).
PERCEPTION ONLY, and only with the tracker on: nothing in the control path,
the costmap, the BT or the missions reads /perception/doors or /viz/doors.
door_detection.py is the rule; this is how the node runs it, on every
tracker cycle AFTER the walls are published, on the tracker's own tracks and
the SAME scan it was stepped with (object sectors cut, with the cut mask):

  * HOSTS: only CONFIRMED wall tracks with role front, left or right carry a
    door. An unknown segment (a tentative track, role unknown or back, a scan
    fragment) never hosts one and is never evidence for or against one. The
    node's door_detection.ManhattanPrior (door_manhattan_prior) is updated
    from the tracks first: a host off the room's dominant direction hosts no
    door, the others are snapped onto it for the gap tests.
  * door_candidates judges EVERY gap between collinear hosts and returns a
    record per gap with a CODE: '' (a door), STRONG (on_line, behind,
    not_boundary, stereo_wall, furniture, short_line, short_side,
    off_manhattan, not_a_leaf), WEAK (oblique_view, few_beams, jamb_occluded,
    unknown_in_gap: undecidable this cycle) or GEOMETRY (not_collinear,
    untracked_jamb, width: not a candidate). The stereo (the rectangle
    verification's class mask + depth pair) is built LAZILY, at most once a
    cycle and only when a gap reaches the stereo stage, and carried from the
    mask stamp to the scan stamp on the odometry (extrinsic_at_scan); with no
    pose at the mask stamp it is used as is only within 0.1 s of the scan,
    else it has no opinion that cycle.
  * DoorTracker: a positive record spawns or refreshes a CANDIDATE, which is
    CONFIRMED once seen positive from door_min_viewpoints car positions
    door_viewpoint_sep_m apart with a steady width (a car standing still
    never confirms a door). A STRONG record on a VISIBLE gap strikes a door
    (back to CANDIDATE; a second strike deletes it); WEAK and GEOMETRY records
    change nothing. A candidate unseen for door_candidate_max_age_sec and a
    confirmed door unseen for door_confirmed_max_age_sec are dropped, unless
    a detected PERSON inside a confirmed door holds it (occupied). Doors are
    associated by centre, normal and width in odom, never by the parent wall
    ids, which re-spawn.
  * PUBLISHED every door cycle, empty included, subscribed or not:
    /perception/doors (f1tenth_messages/DoorArray) with the CONFIRMED doors
    only -- the candidates too with door_publish_candidates -- and /viz/doors
    (door_markers.py) with every live door, confirmed solid, candidate faded.
    /perception/virtual_wall/debug carries door_candidates (every gap of the
    cycle with its code, at most 32), door_manhattan, the lifecycle's
    counters, door_ms, door_errors and door_mask_age_s.
  * THE OVERLAY RULE: doors never modify, split or remove a wall or any LiDAR
    data, and nothing may treat a doorway as free space because a door is
    published there.
  * A door cycle that raises never takes the walls with it (they are already
    out): it is logged (throttled) and counted (door_errors), and after 50
    consecutive failures door detection is off for the node's life.

Every wall marker (/perception/walls/markers and /viz/walls) is drawn
wall_viz_height_m tall from the floor, whatever height the stereo measured:
visualisation only, the published wall data is unchanged.

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
import traceback

import numpy as np

from f1tenth_description.vehicle_frames import base_front_x, MIN_FRONT_DISTANCE_M
from f1tenth_messages.msg import (
    Door, DoorArray, MultiWallTrack, MultiWallTrackArray, StereoColumns, WallEstimate,
    WallLineFit, WallObservation, WallObservationArray)
from f1tenth_params.param_defaults import get_value
from geometry_msgs.msg import Point, Vector3
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
    virtual_wall_due,
)
from corridor_perception import extraction
from f1tenth_perception import door_detection, door_markers
from f1tenth_perception import multi_wall_tracker as mwt
from f1tenth_perception import scene_labels, scene_mask, wall_fusion, wall_markers
from f1tenth_perception.ground_plane import quat_to_matrix
from f1tenth_perception.latest_frame_qos import LATEST_FRAME_QOS

# Same profile tf2_ros.TransformListener uses for /tf_static: the static
# broadcasters latch, so a late subscriber still receives every edge.
_TF_STATIC_QOS = QoSProfile(
    depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


#: Marker colours by role (r, g, b).
_ROLE_COLOUR = {'front': (1.0, 0.25, 0.1), 'left': (0.2, 0.9, 0.3),
                'right': (0.2, 0.5, 1.0), 'back': (0.6, 0.6, 0.6),
                # 'unknown' is real and tracked and is a reference for
                # nothing (and every tentative track is reported as it), so
                # it is drawn -- dimly, in a neutral grey nobody else uses.
                'unknown': (0.35, 0.35, 0.35),
                # Its name before 2026-10-19, same colour.
                'other': (0.35, 0.35, 0.35)}


def _set_lifetime(marker, seconds):
    """Marker.lifetime from a float, split correctly.

    builtin_interfaces/Duration splits into sec + nanosec and the message
    class asserts nanosec is a uint32 in [0, 4294967295]; packing a whole
    lifetime of more than ~4.295 s into nanosec raises, which is exactly the
    assertion that once crash-looped mpc_corr on every first corridor publish
    (MPC_corr.py:5355-5368). One helper, used by both wall marker
    publishers, so that lesson is encoded once.
    """
    whole = int(seconds)
    marker.lifetime.sec = whole
    marker.lifetime.nanosec = int(round((seconds - whole) * 1e9))
    return marker


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


# ---- door detection helpers (module docstring, DOOR DETECTION) ---------------

#: A perception-only overlay must never cost the walls, and a door cycle that
#: fails every time only floods the log: after this many failures IN A ROW the
#: node turns door detection off for its life.
DOOR_MAX_CONSECUTIVE_FAILURES = 50
#: At most this many door_candidates records a cycle on the debug topic,
#: doors and STRONG records first (DOOR_DEBUG_ORDER).
DOOR_DEBUG_MAX_CANDIDATES = 32
DOOR_DEBUG_ORDER = {'ok': 0, 'strong': 1, 'weak': 2, 'geometry': 3}
#: With no odometry pose at the mask stamp the door stereo is used as it is
#: only this close [s] to the scan; further, it has no opinion that cycle.
DOOR_STEREO_UNCOMPENSATED_MAX_S = 0.1
#: The door keys of /perception/virtual_wall/debug, every one of them None
#: when door detection is off.
DOOR_DEBUG_KEYS = (
    'door_ms', 'doors', 'doors_confirmed', 'door_candidates', 'door_candidates_total',
    'door_manhattan', 'door_n_strikes', 'door_n_deleted_by_strikes', 'door_n_confirmed',
    'door_errors', 'door_disabled', 'door_mask_age_s', 'door_stereo')


def extrinsic_at_scan(rotation, translation, pose_scan, pose_mask):
    """
    Return (rotation, translation) that project base_link points taken at the SCAN stamp.

    `rotation` (the camera's axes in base_link) and `translation` (its
    position in base_link) take base_link points into the optical frame the
    way scene_mask.project_to_image does, p_opt = R^T (p - t) -- base_link
    AT THE IMAGE'S MOMENT. The door candidates are base_link at the scan's.
    With the planar odometry poses (odom from base_link) at both stamps,
    pose_scan and pose_mask, a point moves between the two base_links as
    p_mask = R_ms p_scan + t_ms (a yaw and an xy shift; z unchanged), so

        p_opt = R^T (R_ms p_scan + t_ms - t)
              = (R_ms^T R)^T (p_scan - R_ms^T (t - t_ms)):

    the car's motion folds into the extrinsic, and every projection in
    door_detection stays exactly as it is.
    """
    rel = pose_mask.inverse().compose(pose_scan)    # base_link(mask) <- base_link(scan)
    c, s = math.cos(rel.theta), math.sin(rel.theta)
    r_ms = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    t_ms = np.array([rel.x, rel.y, 0.0])
    rot = np.asarray(rotation, dtype=float).reshape(3, 3)
    trans = np.asarray(translation, dtype=float).reshape(3)
    return r_ms.T @ rot, r_ms.T @ (trans - t_ms)


def _json_num(x, digits):
    """Return x rounded, or None for None, NaN or inf (strict JSON has no NaN)."""
    if x is None:
        return None
    x = float(x)
    return round(x, digits) if math.isfinite(x) else None


def door_record(cand):
    """Return one door_candidates record (base_link) as JSON-safe debug data."""
    centre = cand.centre
    return {
        'code': str(cand.code), 'cls': str(cand.cls), 'visible': bool(cand.visible),
        'detail': str(cand.detail), 'width': _json_num(cand.width, 3),
        'n_beams': int(cand.n_beams), 'n_required': int(cand.n_required),
        'view_deg': _json_num(cand.view_deg, 1),
        'behind_frac': _json_num(cand.behind_frac, 3),
        'boundary_hits': int(cand.boundary_hits),
        'leaf_side': door_detection.LEAF_NAMES.get(int(cand.leaf_side), '?'),
        'glass_suspect': None if cand.glass_suspect is None else bool(cand.glass_suspect),
        'jamb_height_unknown': bool(cand.jamb_height_unknown),
        'jamb_tops': [_json_num(top, 2) for top in cand.jamb_tops],
        'host_offset_deg': _json_num(cand.host_offset_deg, 2),
        'parents': [int(i) for i in cand.parents],
        'centre_bl': [_json_num(centre[0], 3), _json_num(centre[1], 3)]}


def _uint(value, bits):
    """Clamp to an unsigned field of `bits` bits, as a Python int."""
    return min(max(int(value), 0), (1 << bits) - 1)


def door_message(door, to_bl, scan_stamp):
    """
    Return the f1tenth_messages/Door of one DoorTrack, every field its exact wire type.

    Humble type-checks every message field setter (and Jazzy with
    ROS_PYTHON_CHECK_FIELDS=1): numpy scalars and ints in a float field are
    rejected, so every value is cast here. `to_bl` takes odom to base_link
    at the scan stamp.
    """
    centre = door.centre
    bl = to_bl.transform_points(np.vstack([centre, door.jamb0, door.jamb1]))
    m = Door()
    m.id = _uint(door.id, 32)
    m.state = (Door.STATE_CONFIRMED if door.state == door_detection.STATE_CONFIRMED
               else Door.STATE_CANDIDATE)
    m.center = Point(x=float(centre[0]), y=float(centre[1]), z=0.0)
    m.normal = Vector3(x=float(door.normal[0]), y=float(door.normal[1]), z=0.0)
    m.width = float(door.width)
    m.jamb0 = Point(x=float(door.jamb0[0]), y=float(door.jamb0[1]), z=0.0)
    m.jamb1 = Point(x=float(door.jamb1[0]), y=float(door.jamb1[1]), z=0.0)
    m.center_bl, m.jamb0_bl, m.jamb1_bl = (
        Point(x=float(q[0]), y=float(q[1]), z=0.0) for q in bl)
    m.parent_wall_id0, m.parent_wall_id1 = (_uint(i, 32) for i in door.parents)
    m.occupied = bool(door.occupied)
    m.confidence = float(min(max(float(door.confidence), 0.0), 1.0))
    m.stereo_support = bool(door.stereo_support)
    m.hits = _uint(door.hits, 32)
    m.age = float(scan_stamp - door.born)
    m.since_seen = float(scan_stamp - door.last_positive)
    m.n_beams = _uint(door.n_beams, 16)
    m.leaf_side = _uint(door.leaf_side, 8)
    m.glass_suspect = bool(door.glass_suspect)
    m.jamb_height_unknown = bool(door.jamb_height_unknown)
    return m


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
            odom_max_age_sec=float(param('odom_max_age_sec', 0.3)),
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
        # THE COLLINEAR GAP RULE (multi_wall_tracker): collinear LiDAR wall
        # pieces are one wall only across a gap shorter than the narrowest
        # door. ONE parameter, read once, for the extraction's merge, the
        # tracker and the door detector.
        self.door_width_min_m = float(param('door_width_min_m', mwt.DOOR_WIDTH_MIN_M))
        tracker_params = mwt.TrackerParams(
            process_noise_rho=float(param('wall_tracker_process_noise_rho', 1e-4)),
            process_noise_theta=float(param('wall_tracker_process_noise_theta', 1e-4)),
            k_yaw=float(param('wall_tracker_k_yaw', 0.1)),
            # THE COLLINEAR GAP RULE (association, track merge, extent) and
            # the extent decay -- multi_wall_tracker's THE MEASURED EXTENT.
            collinear_gap_m=self.door_width_min_m,
            extent_decay_s=float(param('wall_extent_decay_sec', 1.0)),
            delete_after_s=float(param('wall_tracker_delete_after_sec', 2.0)),
            # The camera gates and the camera side zone (Q2 / P3).
            camera_seg_min_ray_angle_rad=math.radians(
                float(param('camera_seg_min_ray_angle_deg', 25.0))),
            camera_seg_min_bearing_span_rad=math.radians(
                float(param('camera_seg_min_bearing_span_deg', 3.0))),
            stereo_max_rms_m=float(param('stereo_max_rms_m', 0.05)),
            stereo_min_vertical_extent_m=float(param('stereo_min_vertical_extent_m', 0.4)),
            stereo_min_length_m=float(param('stereo_min_length_m', 0.8)),
            camera_side_x_max_m=float(param('camera_side_x_max_m', 4.0)),
            camera_side_max_normal_rad=math.radians(
                float(param('camera_side_max_normal_deg', 30.0))),
            camera_side_offset_min_m=float(param('camera_side_offset_min_m', 0.3)),
            camera_side_offset_max_m=float(param('camera_side_offset_max_m', 3.0)),
            coast_max_odom_m=float(param('wall_coast_max_odom_m', 1.0)),
            max_sigma_rho_m=float(param('wall_tracker_max_sigma_rho_m', 0.3)),
            max_published=int(param('wall_tracker_max_tracks', 4)),
            front_max_coast_s=float(param('wall_tracker_front_max_coast_sec', 0.5)),
            front_max_coast_m=float(param('wall_tracker_front_max_coast_m', 0.3)),
            # Step 4: the observation noise model and what may be called FRONT.
            lidar_sigma_base_m=float(param('lidar_sigma_base_m', 0.02)),
            lidar_sigma_per_m=float(param('lidar_sigma_per_m', 0.005)),
            stereo_sigma_base_m=float(param('stereo_sigma_base_m', 0.03)),
            stereo_sigma_quad_per_m2=float(param('stereo_sigma_quad_per_m2', 0.012)),
            min_span_for_angle_m=float(param('min_span_for_angle_m', 0.3)),
            # THE CONFIRMATION LEVEL (multi_wall_tracker): the ONE promotion
            # rule every reader of "confirmed" relies on, the wall_confirm_*
            # group of stack_params.yaml under its own names.
            confirm_min_consecutive_hits=int(param('wall_confirm_min_consecutive_hits', 5)),
            confirm_min_length_m=float(param('wall_confirm_min_length_m', 1.0)),
            confirm_max_dist_jitter_m=float(param('wall_confirm_max_dist_jitter_m', 0.05)),
            confirm_max_angle_jitter_rad=math.radians(
                float(param('wall_confirm_max_angle_jitter_deg', 3.0))),
            confirm_require_lidar=bool(param('wall_confirm_require_lidar', True)),
            role_hysteresis_rad=float(param('role_hysteresis_rad', math.radians(5.0))),
            stereo_front_max_normal_rad=float(
                param('stereo_front_max_normal_rad', math.radians(30.0))),
            # The TRAVEL STRIP is the bodywork (swept_clearance_body_half_width_m)
            # plus front_strip_margin_m, from the bumper: multi_wall_tracker's
            # WHAT FRONT REQUIRES.
            front_x_m=float(self.front_x),
            side_edge_half_width_m=float(get_value('swept_clearance_body_half_width_m')),
            front_strip_margin_m=float(param('front_strip_margin_m', 0.15)),
            front_min_confidence=float(param('front_min_confidence', 0.5)),
            front_lidar_recent_cycles=int(param('front_lidar_recent_cycles', 3)),
            front_contradict_margin_m=float(param('front_contradict_margin_m', 0.5)),
            front_contradict_cycles=int(param('front_contradict_cycles', 2)),
            stereo_min_rho_m=float(param('wall_obs_stereo_min_rho_m', 0.4)),
            # Minimum width for a wall: see multi_wall_tracker's own
            # "WHY THE SPAN GATE IS HYSTERETIC".
            min_span_birth_m=float(param('wall_min_span_birth_m', 1.0)),
            min_span_update_m=float(param('wall_min_span_update_m', 0.5)),
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
        #: The per-track debug payload (and the objects-behind-a-wall count
        #: that goes with it). OFF by default: it is a transform and two
        #: distances per LIVE track per scan on a box whose tracker already
        #: spends 155-178 ms a scan against a 25 ms scan period, and nothing
        #: in the control path reads it. The replay turns it on.
        self.debug_tracks = bool(param('wall_debug_tracks', False))
        rect_params = wall_fusion.RectParams(
            coverage_min=float(param('wall_rect_coverage_min', 0.25)),
            support_verify=float(param('wall_rect_support_verify', 0.5)),
            support_reject=float(param('wall_rect_support_reject', 0.2)),
            reject_cycles=int(param('wall_rect_reject_cycles', 3)),
            glass_beyond_share=float(param('wall_rect_glass_beyond_share', 0.7)),
            max_range_m=float(param('wall_rect_max_range_m', 8.0)))
        # The tolerance is NOT a parameter of its own: the rectangle uses the
        # SAME model and the SAME two numbers as the column cross-check, which
        # are declared in step 3 below and copied onto this object once they
        # are (the tracker holds this very object). A rectangle sample is the
        # same measurement a column is, one pixel at a time, and two
        # tolerances for one sensor error would only ever disagree.
        self._rect_params = rect_params
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
        # The extraction's collinear merge reunifies a wall broken by an
        # occluder BEFORE its span is measured -- across a gap shorter than
        # door_width_min_m only (THE COLLINEAR GAP RULE).
        self.extraction_params = mwt.extraction_params(
            tracker_params.collinear_gap_m,
            quality_gates=bool(param('wall_extract_quality_gates', True)),
            backend=str(param('wall_extract_backend', 'c')))
        # One warning, at startup, if the C backend was asked for and is not
        # there: the Python is 22x slower a scan and the node must not be the
        # only thing that knows it fell back.
        warning = extraction.c_backend_warning() \
            if self.extraction_params.backend == 'c' else None
        if warning:
            self.get_logger().warn(warning)
        #: What each extraction gate dropped on the last scan, for the debug payload.
        self._extract_stats = {}
        # The tracker runs on one scan in N (see WALL TRACKER DECIMATION).
        # Deterministic: a counter, not a clock, so a replay of the same bag
        # processes the same scans in the same order every time.
        self.tracker_decimation = max(1, int(param('wall_tracker_scan_decimation', 4)))
        self._scan_count = -1
        #: Scans the decimation skipped since the last processed one, for the
        #: debug payload and for the one-line startup message.
        self._tracker_skipped = 0
        self._poses = mwt.PoseHistory()
        # How far a stamp may lie past the newest pose (stack_params
        # lidar_front_wall_pose_max_extrapolation_sec): 0.15 s = 3 periods of
        # the 20 Hz EKF (cpu-cleanup R2; was the hardcoded 0.1 s = 2 at 50 Hz).
        self.pose_max_extrapolation_s = float(param('pose_max_extrapolation_sec', 0.15))
        # /perception/virtual_wall only on a NEW estimate, at most ~this rate
        # (stack_params lidar_front_wall_virtual_wall_rate_hz; see
        # _virtual_due). Was every scan, 3 in 4 repeating the tracker verdict.
        # The confidence model's range scaling (wall_fusion.confidence, RANGE):
        # what the far front wall needs so front_min_confidence stops refusing
        # it. Nothing changes inside conf_range_ref_m.
        self.conf_range_ref_m = float(param('wall_conf_range_ref_m',
                                            wall_fusion.CONF_RANGE_REF_M))
        self.conf_min_inlier_ref = float(param('wall_conf_min_inlier_ref',
                                               wall_fusion.CONF_MIN_INLIER_REF))
        self.conf_max_rms_scale = float(param('wall_conf_max_rms_scale',
                                              wall_fusion.CONF_MAX_RMS_SCALE))
        self.virtual_wall_rate_hz = float(param('virtual_wall_rate_hz', 10.0))
        self._last_virtual_pub = None
        self._tracker_ran = False
        # A no-transform scan produces a NEW verdict (NO_DATA) although the
        # tracker did not run, so it must pass _virtual_due's new-estimate
        # gate -- see _scan_cb. Still subject to virtual_wall_rate_hz.
        self._no_transform_verdict = False
        self._odom_frame = 'odom'
        self._wall_source = 'sequential_lines'
        self._front_id = None       # id of the tracker's current front track
        self._front_d_ray = math.nan   # its distance from the bumper, this scan
        self._tracker_has_data = False  # could this cycle see at all (step 5)
        self._front_reason = 'no_candidate'  # why there is (not) a FRONT, for debug
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
        self._rect_params.tol_base_m = self.obs_tol_base_m
        self._rect_params.tol_quad_per_m2 = self.obs_tol_quad
        self.obs_disagree_reject_frac = float(param('wall_obs_disagree_reject_frac', 0.5))
        self.obs_stereo_min_columns = int(param('wall_obs_stereo_min_columns', 8))
        self.wall_default_height = float(param('wall_default_height_m', 2.5))
        self.wall_clear_distance_m = float(get_value('wall_clear_distance_m'))
        # r_clear: a strip return nearer than this with no FRONT track is
        # flagged 'unexplained_returns' on the debug topic. The contract still
        # says CLEAR (multi_wall_tracker.contract_reason says why).
        self.front_clear_range_m = float(param('front_clear_range_m', 2.0))
        self.wall_marker_thickness_m = float(param('wall_marker_thickness_m', 0.02))
        # HOW LONG A WALL MARKER OUTLIVES THE MESSAGE THAT DREW IT. The
        # markers used to carry no lifetime at all (lifetime 0 == forever) and
        # the array led with a DELETEALL every cycle, which is the pair of
        # choices that makes walls flicker or vanish outright in Foxglove: the
        # torn-down state is the one a client sees whenever the rebuild that
        # was meant to follow it is dropped, late, or simply not subscribed to
        # yet. A lifetime does the same job the DELETEALL was reaching for --
        # a stale wall must not linger and imply liveness it does not have --
        # without the gap, and it also makes a stalled tracker visible as the
        # walls fading out instead of as a corridor that is silently no longer
        # true (the same reasoning as MPC_corr's corridor markers,
        # MPC_corr.py:5345-5356). 0.6 s is >= 3x the tracker's own ~0.1 s
        # period (40 Hz scans / wall_tracker_scan_decimation 4) and exactly 3x
        # the 0.2 s period of the /viz/walls cap, so one number covers both
        # publishers.
        self.wall_marker_lifetime_s = float(param('wall_marker_lifetime_s', 0.6))
        # /viz/walls: the whole tracker state, for a human, on a topic nothing
        # consumes -- see wall_markers.py's own module docstring for why it is
        # a second topic rather than a wider /perception/walls/markers. Off
        # costs one get_subscription_count() per tracker cycle.
        self.viz_walls_enabled = bool(param('viz_walls_enabled', True))
        self.viz_walls_max_hz = float(param('viz_walls_max_hz', 5.0))
        # Every wall marker -- here and on /viz/walls -- is drawn this tall from
        # the floor, whatever the stereo measured: visualisation only.
        self.wall_viz_height_m = float(param('wall_viz_height_m', 2.5))
        self._viz_walls_builder = wall_markers.WallMarkerBuilder(
            self.wall_marker_lifetime_s, self.wall_viz_height_m)
        self._viz_walls_next = None
        # Which track ids the last MarkerArray drew, so a track that dies --
        # or merely drops out of published()'s four slots -- can be removed
        # with an explicit per-id DELETE instead of a blanket DELETEALL.
        self._drawn_track_ids = set()
        self.side_edge_half_width_m = float(get_value('swept_clearance_body_half_width_m'))

        # ---- door detection (module docstring, DOOR DETECTION) --------------
        # Every fallback below is DoorParams' own default (in degrees here
        # where its field is *_rad). DoorParams, these fallbacks and
        # stack_params.yaml's lidar_front_wall_door_* must agree
        # (test_lidar_front_wall_node.py, TestDoorParamsCoupling).
        door_requested = bool(param('door_detection_enabled', False))
        self.door_enabled = door_requested and self.wall_tracker is not None
        self.door_params = door_detection.DoorParams(
            # The gap's geometry and its beams.
            collinear_angle_rad=math.radians(float(param('door_collinear_angle_deg', 5.0))),
            collinear_offset_m=float(param('door_collinear_offset_m', 0.10)),
            width_min_m=self.door_width_min_m,
            width_max_m=float(param('door_width_max_m', 1.00)),
            # ON the wall line is what the negative evidence calls a hit.
            wall_tol_m=float(tracker_params.negative_tol_m),
            min_beams=int(param('door_min_beams', 8)),
            min_beam_fraction=float(param('door_min_beam_fraction', 0.8)),
            # HOSTS and the MANHATTAN PRIOR.
            manhattan_prior=bool(param('door_manhattan_prior', True)),
            manhattan_tol_deg=float(param('door_manhattan_tol_deg', 10.0)),
            manhattan_min_wall_len_m=float(param('door_manhattan_min_wall_len_m', 1.5)),
            manhattan_tau_s=float(param('door_manhattan_tau_sec', 2.0)),
            min_line_extent_m=float(param('door_min_line_extent_m', 2.0)),
            host_front_facing=bool(param('door_host_front_facing', False)),
            min_side_len_m=float(param('door_min_side_len_m', 0.5)),
            # The gap tests, every depth PERPENDICULAR to the (snapped) line.
            max_view_angle_deg=float(param('door_max_view_angle_deg', 45.0)),
            behind_margin_m=float(param('door_behind_margin_m', 1.0)),
            behind_frac=float(param('door_behind_frac', 0.7)),
            boundary_behind_min_m=float(param('door_boundary_behind_min_m', 0.30)),
            boundary_max_hits=int(param('door_boundary_max_hits', 2)),
            boundary_end_margin_m=float(param('door_boundary_end_margin_m', 0.10)),
            on_line_min_beams=int(param('door_on_line_min_beams', 2)),
            # THE LEAF and THE REVEAL: an open leaf hung on a jamb is a door.
            core_fraction=float(param('door_core_fraction', 0.6)),
            leaf_max_len_m=float(param('door_leaf_max_len_m', 1.0)),
            leaf_hinge_tol_m=float(param('door_leaf_hinge_tol_m', 0.10)),
            reveal_max_depth_m=float(param('door_reveal_max_depth_m', 0.20)),
            # The stereo: the glass veto (OFF by default) and the jamb height.
            stereo_glass_veto=bool(param('door_stereo_glass_veto', False)),
            stereo_plane_tol_m=float(param('door_stereo_plane_tol_m', 0.30)),
            stereo_wall_veto_frac=float(param('door_stereo_wall_veto_frac', 0.5)),
            jamb_height_check=bool(param('door_jamb_height_check', True)),
            jamb_min_height_m=float(param('door_jamb_min_height_m', 1.8)),
            furniture_max_height_m=float(param('door_furniture_max_height_m', 1.3)),
            # The LIFECYCLE: two viewpoints and a steady width to confirm.
            min_viewpoints=int(param('door_min_viewpoints', 2)),
            viewpoint_sep_m=float(param('door_viewpoint_sep_m', 0.3)),
            width_std_max_m=float(param('door_width_std_max_m', 0.05)),
            confirm_window=int(param('door_confirm_window', 20)),
            candidate_max_age_s=float(param('door_candidate_max_age_sec', 10.0)),
            confirmed_max_age_s=float(param('door_confirmed_max_age_sec', 60.0)),
            # SEEN SQUARE-ON: an optional extra confirmation requirement, off.
            square_looks=int(param('door_square_looks', 0)),
            square_incidence_rad=math.radians(float(param('door_square_incidence_deg', 3.0))),
            square_min_beams=int(param('door_square_min_beams', 8)))
        # /perception/doors carries the CONFIRMED doors only unless this is
        # set; /viz/doors always draws both states.
        self.door_publish_candidates = bool(param('door_publish_candidates', False))
        self.door_jamb_viz_diameter_m = float(param('door_jamb_viz_diameter_m', 0.10))
        self.door_tracker = (door_detection.DoorTracker(self.door_params)
                             if self.door_enabled else None)
        # The room's dominant wall direction, updated from the tracker's
        # tracks on every door cycle before door_candidates reads it.
        self._door_manhattan = door_detection.ManhattanPrior(self.door_params)
        self._viz_doors_builder = door_markers.DoorMarkerBuilder(
            self.wall_marker_lifetime_s, self.wall_viz_height_m,
            self.door_jamb_viz_diameter_m)
        self._door_ms = math.nan
        #: This cycle's door_candidates records as debug data (door_record),
        #: their uncapped count, and the prior as the cycle left it.
        self._door_records = []
        self._door_candidates_total = 0
        self._door_manhattan_debug = self._door_manhattan.to_debug()
        #: The door stereo this cycle: its mask/depth pair's age (scan stamp
        #: minus mask stamp) and what became of it (_door_stereo_frame).
        self._door_mask_age = math.nan
        self._door_stereo = 'unused'
        #: Door cycles that raised, ever and in a row, and whether that turned
        #: door detection off for the node's life (DOOR_MAX_CONSECUTIVE_FAILURES).
        self._door_errors = 0
        self._door_failures_in_a_row = 0
        self._door_disabled = False
        #: The class_id of each detection in self._detections[1], same order:
        #: a person inside a confirmed door marks it occupied.
        self._detection_classes = []

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
            # Created whatever viz_walls_enabled says, so the topic exists to
            # be subscribed to and the flag is a publish-time decision, not a
            # restart-time one.
            self.viz_walls_pub = self.create_publisher(MarkerArray, '/viz/walls', 10)
        if self.door_tracker is not None:
            self.doors_pub = self.create_publisher(DoorArray, '/perception/doors', 10)
            self.viz_doors_pub = self.create_publisher(MarkerArray, '/viz/doors', 10)
        # Best-effort on purpose -- see the module docstring's NO COUPLING TO
        # THE E-STOP section -- and DEPTH 1 since the decimation: this node is
        # the slowest /scan consumer in the stack, and a queue deeper than one
        # hands the executor the OLDEST scan it holds, so every scan waits for
        # the ones queued ahead of it (latest_frame_qos.py's own measurement,
        # 373 ms of queueing on yolo_detector_node). With depth 1 DDS drops
        # the older scans instead of this node working through them late.
        # Still BEST_EFFORT, so it still cannot back-pressure urg_node.
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_cb, LATEST_FRAME_QOS)

        self.get_logger().info(
            f'lidar_front_wall_node up: {self.scan_topic} -> {self.fit_topic}, '
            f'{self.estimate_topic}; sector +-{math.degrees(self.sector_half_angle_rad):.1f} deg, '
            f'odometry {self.odom_topic}, seed {self.seed_topic}'
            + ('; SCENE LABELS ON: detection sectors cut, /perception/wall_planes, '
               '/perception/virtual_wall' if self.scene_enabled else '')
            + self._door_startup_summary(door_requested))

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
        objs, classes = [], []
        for det in msg.detections:
            p = det.bbox.center.position
            x, y, _ = rot @ (p.x, p.y, p.z) + (t.translation.x, t.translation.y,
                                               t.translation.z)
            objs.append((float(x), float(y), 0.5 * float(det.bbox.size.x)))
            classes.append(str(det.results[0].hypothesis.class_id) if det.results else '')
        stamp = msg.header.stamp
        self._detections = (self._now(), objs, stamp.sec + stamp.nanosec * 1e-9)
        self._detection_classes = classes

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
                                       line.rms, 0, 0,
                                       range_m=abs(float(line.d)),
                                       range_ref_m=self.conf_range_ref_m,
                                       min_inlier_ref=self.conf_min_inlier_ref,
                                       max_rms_scale=self.conf_max_rms_scale))
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
            stereo_weight=self._stereo_weight,
            conf_range_ref_m=self.conf_range_ref_m,
            conf_min_inlier_ref=self.conf_min_inlier_ref,
            conf_max_rms_scale=self.conf_max_rms_scale)
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
        pose_scan = self._poses.at(scan_stamp, self.pose_max_extrapolation_s)
        pose_image = self._poses.at(image_stamp, self.pose_max_extrapolation_s)
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
            then = self._poses.at(det_stamp, self.pose_max_extrapolation_s)
            now_pose = self._poses.at(scan_stamp, self.pose_max_extrapolation_s)
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
        self._tracker_ran = False
        self._no_transform_verdict = False
        laser_pose = self._laser_pose_for(msg.header.frame_id)
        if laser_pose is None:
            fit = SectorFit(0, 0)
            reason = REASON_NO_TRANSFORM
            # No transform: the scan cannot be placed, so the contract is
            # NO_DATA -- not whatever the last placed scan said.
            self._tracker_has_data = False
            self._front_d_ray = math.nan
            # ...and that NO_DATA has to reach the wire. _virtual_due only
            # lets a scan through when the tracker stepped, which it cannot
            # here: without this the contract silently keeps the last placed
            # scan's verdict for as long as /tf_static is missing.
            self._no_transform_verdict = True
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
        # the WallEstimate carries, so the two can never disagree. Computed
        # every scan; PUBLISHED only on a new estimate (_virtual_due).
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
        self._tracker_ran = False
        if self.wall_tracker is not None and self._tracker_due():
            self._tracker_ran = True
            # With the tracker on, the observations are built from ITS
            # per-scan extraction (adaptive breakpoints, IEPF, a covariance
            # per segment) rather than the 5 Hz sequential_lines above, and
            # the tracker is then fed exactly what went out on the topic.
            front = self._track_walls(msg, laser_pose, sectors, scan_stamp, stamp, pts)
            if front is not None:
                self._tracked_wall, self._wall_source = front, 'tracker'
            elif self._wall_source == 'tracker':
                self._tracked_wall, self._wall_source = self._seq_wall, 'sequential_lines'
        elif self.wall_tracker is not None:
            # A SKIPPED scan: the tracker's last answer stands. Not NO_DATA --
            # the tracker's verdict is a 10 Hz quantity published at the scan
            # rate, and flapping the contract three scans in four would be a
            # worse lie than repeating a 75 ms old one. _front_d_ray,
            # _tracker_has_data and _tracked_wall are all left exactly as the
            # last processed scan left them.
            self._tracker_skipped += 1
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
            # NO_DATA only when the LiDAR could not be used at all (no pose
            # at the scan stamp, no transform). A FRONT that was dropped --
            # coasted out, contradicted by the strip's beams, never there --
            # is CLEAR: the sensors are working and no wall is in the way.
            has_data = self._tracker_has_data
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
        if not self._virtual_due(now):
            return state
        self.virtual_pub.publish(Float32(data=float(value)))

        def num(x):
            return None if x is None or not math.isfinite(x) else round(x, 4)
        # NOT ONLY DEBUG: `state`, `distance_m` and `front_id` are the mission
        # wall stop's input since 2026-10-21 (f1tenth_behavior's
        # check_stop_condition.front_from_debug, FRONT-WALL LOCK). The three
        # come from this one publish, so the value is known to belong to
        # that track. Keep the keys and their meaning.
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
            'wall_source': None if self.wall_tracker is None else self._wall_source,
            # Block 3: why there is or is not a FRONT (multi_wall_tracker.
            # front_reason codes, 'front', 'no_candidate' or
            # 'unexplained_returns'), the FRONT track's id, and the nearest
            # LiDAR return in the travel strip, from the bumper (None: the
            # strip returned nothing -- glass, range, or cut for an object).
            'front_reason': None if self.wall_tracker is None else self._front_reason,
            'front_id': None if self.wall_tracker is None else self._front_id,
            'strip_range_m': (None if self.wall_tracker is None
                              else num(self.wall_tracker.strip.range_m)),
            # Observations refused by the collinear gap (a different wall on
            # the same line), track merges refused by it, and stereo-only
            # lines dropped as the car's own chassis; `rejections` has a
            # reason code per event: [code, track id, metres].
            'gap_rejected': (None if self.wall_tracker is None
                             else int(self.wall_tracker.n_gap_rejected)),
            'merge_gap_refused': (None if self.wall_tracker is None
                                  else int(self.wall_tracker.n_merge_gap_refused)),
            'stereo_near_rejected': (None if self.wall_tracker is None
                                     else int(self.wall_tracker.n_stereo_near_rejected)),
            'rejections': (None if self.wall_tracker is None
                           else [list(r) for r in self.wall_tracker.rejections]),
            # Observations the span gate dropped as too short to be a wall. A
            # run of these says the room is full of short faces, or that
            # the collinear gap (door_width_min_m) is too tight to reunify one wall.
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
            # The decimation: how many scans were skipped since the tracker
            # last ran, so a consumer can tell a held verdict from a fresh one.
            'tracker_decimation': int(self.tracker_decimation),
            'tracker_skipped': int(self._tracker_skipped),
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
            'objects_behind_wall': self._objects_behind_wall,
            # Door detection (DOOR DETECTION), all None with it off: its cost
            # this cycle; the live doors and how many are CONFIRMED; EVERY gap
            # door_candidates judged this cycle with its code (door_record, at
            # most DOOR_DEBUG_MAX_CANDIDATES, doors and STRONG ones first) and
            # how many there were; the Manhattan prior; the lifecycle's
            # cumulative strikes, strike deletions and confirmations; failed
            # cycles and whether they turned it off; the stereo pair's age and
            # what became of it (_door_stereo_frame).
            **self._door_payload()})))
        return state

    def _virtual_due(self, now):
        """lidar_front_wall.virtual_wall_due for this node's state; records the publish."""
        due = virtual_wall_due(self.wall_tracker is not None,
                               self._tracker_ran or self._no_transform_verdict, now,
                               self._last_virtual_pub, self.virtual_wall_rate_hz)
        if due:
            self._last_virtual_pub = now
        return due

    def _tracker_due(self):
        """Is this scan one of the one-in-N the tracker processes?

        A counter, not a clock: the same bag decimates the same way every
        replay, which a time-based gate would not. The count advances on every
        scan that reaches the tracker branch, so the phase is stable.
        """
        self._scan_count += 1
        if self.tracker_decimation <= 1:
            return True
        due = (self._scan_count % self.tracker_decimation) == 0
        if due:
            self._tracker_skipped = 0
        return due

    def _track_walls(self, msg, laser_pose, sectors, scan_stamp, stamp, pts):
        """One tracker step. Returns the front track as a base_link WallLine, or None.

        Since step 4 the tracker consumes ONLY wall observations: this
        extracts the scan's segments, puts them through the per-cycle fusion
        (which publishes them), and hands the tracker the result. There is no
        other way into it, so the stereo cannot enter as a fallback distance
        behind its back.
        """
        car = self._poses.at(scan_stamp, self.pose_max_extrapolation_s)
        if car is None:
            # No pose to place the walls with: NO_DATA, not "clear".
            self._front_d_ray = math.nan
            self._tracker_has_data = False
            self._front_reason = 'no_pose'
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
        # Where the camera looks from, for the radial gate, and whether its
        # data was fresh this cycle, for camera-only confirmation (Q2/P3).
        cam = self._camera_transform(self._stereo_columns[1].camera_frame) \
            if self._stereo_columns else None
        if cam is not None:
            self.wall_tracker.camera_xy = (float(cam[1][0]), float(cam[1][1]))
        self.wall_tracker.camera_fresh = bool(
            math.isfinite(self._stereo_age)
            and self._stereo_age <= self.profile_max_age)
        self._objects_behind_wall = (
            self._count_objects_behind_walls(car, laser_pose, scan_stamp)
            if self.debug_tracks else 0)
        # The negative evidence sees exactly the beams the extraction saw,
        # object sectors cut and all (mwt.scan_in_base_link): a beam cut
        # because an object stands there is evidence about the object. The
        # cut MASK goes in too, so a cut beam is never read as free space
        # (the strip range, the collinear gap, the negative evidence).
        neg_ranges, neg_bearings, _angles, cut = mwt.scan_in_base_link(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
            msg.range_max, laser_pose, sectors, with_cut=True)
        # The roles are DEFERRED: the rectangle runs on the tracks the step
        # just updated and its verdicts are folded in before anything is asked
        # of the roles -- a REJECTED wall is not a reference wall on the same
        # cycle it was rejected, not on the next one -- and deciding them
        # before and after would pay for the role and occlusion pass twice.
        self.wall_tracker.step(observations, scan_stamp, car,
                               scan=(neg_ranges, neg_bearings, laser_pose[:2], cut),
                               defer_roles=True)
        self._rect_verdicts = self._verify_tracks(
            car, scan_stamp, self._stereo_columns[1] if self._stereo_columns else None)
        self.wall_tracker.update_roles(car)
        tracks = self.wall_tracker.published(car)
        front = self.wall_tracker.front_track(car, self.front_x)
        status = self.wall_tracker.front_status
        # The FRONT distance is where the travel strip meets the MEASURED
        # segment, from the bumper (mwt.strip_distance), NaN without a FRONT:
        # CLEAR. There is no NO_DATA latch any more -- a FRONT that went away
        # was dropped for a reason the debug output now names, and the strip
        # it was judged against is right there on the same topic.
        self._front_d_ray = status['distance_m'] if front is not None else math.nan
        previous, self._front_id = self._front_id, status['id']
        self._front_reason = mwt.contract_reason(
            status['reason'], self.wall_tracker.strip.range_m, self.front_clear_range_m)
        if previous is not None and front is None:
            self.get_logger().info(
                f'wall tracker: front track {previous} dropped '
                f'({status["rejects"].get(previous, "gone")}); virtual wall CLEAR '
                f'(strip {self.wall_tracker.strip.range_m})')
        if self.wall_tracker.n_stereo_near_rejected:
            self.get_logger().warn(
                f'{self.wall_tracker.n_stereo_near_rejected} stereo-only wall '
                f'observation(s) nearer than {self.wall_tracker.p.stereo_min_rho_m:.2f} m '
                'dropped as chassis: a run of these means the self polygon '
                '(front_clearance_scene_mask_self_polygons) misses part of the car',
                throttle_duration_sec=10.0)
        self._tracker_ms = (time.perf_counter() - t0) * 1e3
        self._log_tracker_cost(scan_stamp)
        self._publish_walls(msg.header.stamp, tracks, car)
        if self.door_tracker is not None:
            self._detect_doors(car, (neg_ranges, neg_bearings, laser_pose[:2], cut),
                               scan_stamp, msg.header.stamp)
        return None if front is None else mwt.to_wall_line(front, car)

    # ---- door detection (module docstring, DOOR DETECTION) ------------------

    def _detect_doors(self, car, scan, scan_stamp, stamp):
        """
        Run one door cycle on this tracker step; nothing it raises leaves it.

        Called AFTER the walls are published. A failure is logged (throttled)
        and counted (door_errors), and DOOR_MAX_CONSECUTIVE_FAILURES in a row
        turn door detection off for the node's life: a perception-only
        overlay never costs the walls or the virtual wall that follows.
        """
        self._door_records, self._door_candidates_total = [], 0
        self._door_mask_age, self._door_stereo = math.nan, 'unused'
        if self._door_disabled:
            self._door_ms = math.nan
            return
        t0 = time.perf_counter()
        try:
            self._door_cycle(car, scan, scan_stamp, stamp)
        except Exception as exc:
            self._door_failed(exc)
        else:
            self._door_failures_in_a_row = 0
        self._door_ms = (time.perf_counter() - t0) * 1e3

    def _door_cycle(self, car, scan, scan_stamp, stamp):
        """Run the Manhattan prior, the candidates (stereo lazily), the tracker, the topics."""
        tracks = self.wall_tracker.tracks
        self._door_manhattan.update(tracks, scan_stamp)
        self._door_manhattan_debug = self._door_manhattan.to_debug()
        candidates = door_detection.door_candidates(
            tracks, car, scan, self.door_params,
            stereo=lambda: self._door_stereo_frame(car, scan_stamp),
            manhattan=self._door_manhattan)
        # The debug records are built here, inside the cycle's guard, so the
        # debug publish only ever reads plain data.
        ordered = sorted(candidates, key=lambda c: DOOR_DEBUG_ORDER.get(c.cls, 4))
        self._door_records = [door_record(c) for c in ordered[:DOOR_DEBUG_MAX_CANDIDATES]]
        self._door_candidates_total = len(candidates)
        doors = self.door_tracker.step(candidates, car, scan_stamp,
                                       self._persons_at(car, scan_stamp), scan=scan)
        self._publish_doors(stamp, doors, car, scan_stamp)

    def _door_failed(self, exc):
        """Log and count a failed door cycle; turn door detection off after too many."""
        self._door_errors += 1
        self._door_failures_in_a_row += 1
        frames = traceback.extract_tb(exc.__traceback__)
        where = (f' (at {frames[-1].filename.rsplit("/", 1)[-1]}:{frames[-1].lineno} '
                 f'in {frames[-1].name})' if frames else '')
        self.get_logger().error(f'door detection failed: {exc!r}{where}',
                                throttle_duration_sec=5.0)
        if self._door_failures_in_a_row >= DOOR_MAX_CONSECUTIVE_FAILURES:
            self._door_disabled = True
            self.get_logger().error(
                f'door detection DISABLED for the life of this node: '
                f'{self._door_failures_in_a_row} consecutive door cycles failed (last: '
                f'{exc!r}{where}). /perception/doors and /viz/doors stop; the walls '
                'and the virtual wall are unaffected.')

    def _door_startup_summary(self, requested):
        """Return the startup log's door clause: on or off, and with which key numbers."""
        if not self.door_enabled:
            return '; door detection OFF' + (
                ' (it needs use_wall_tracker)' if requested else '')
        p = self.door_params
        published = ('confirmed + candidates' if self.door_publish_candidates
                     else 'confirmed only')
        return (f'; DOOR DETECTION ON: /perception/doors {published}, /viz/doors all; '
                f'width {p.width_min_m:.2f}-{p.width_max_m:.2f} m, behind '
                f'{p.behind_margin_m:.2f} m x {p.behind_frac:.2f}, confirm on '
                f'{p.min_viewpoints} viewpoints {p.viewpoint_sep_m:.2f} m apart, '
                f'manhattan {"on" if p.manhattan_prior else "off"} '
                f'({p.manhattan_tol_deg:.0f} deg), jamb height '
                f'{"on" if p.jamb_height_check else "off"}, glass veto '
                f'{"on" if p.stereo_glass_veto else "off"}')

    def _door_payload(self):
        """Return the door keys of /perception/virtual_wall/debug, all None when off."""
        tr = self.door_tracker
        if tr is None:
            return dict.fromkeys(DOOR_DEBUG_KEYS)
        return {
            'door_ms': _json_num(self._door_ms, 4),
            'doors': len(tr.doors),
            'doors_confirmed': sum(1 for d in tr.doors
                                   if d.state == door_detection.STATE_CONFIRMED),
            'door_candidates': list(self._door_records),
            'door_candidates_total': int(self._door_candidates_total),
            'door_manhattan': dict(self._door_manhattan_debug),
            'door_n_strikes': int(tr.n_strikes),
            'door_n_deleted_by_strikes': int(tr.n_deleted_by_strikes),
            'door_n_confirmed': int(tr.n_confirmed),
            'door_errors': int(self._door_errors),
            'door_disabled': bool(self._door_disabled),
            'door_mask_age_s': _json_num(self._door_mask_age, 3),
            'door_stereo': self._door_stereo}

    def _persons_at(self, car, scan_stamp):
        """
        Return this cycle's PERSON detections, (x, y, half width) in base_link.

        Carried from the detection stamp on odometry exactly like the object
        sectors (_sectors), and dropped past wall_tracker_detection_max_age_sec.
        """
        if self._detections is None:
            return []
        people = [obj for obj, cls in zip(self._detections[1], self._detection_classes)
                  if cls in self.door_params.person_classes]
        det_stamp = self._detections[2]
        if not people or scan_stamp - det_stamp > self.tracker_det_max_age:
            return []
        then = self._poses.at(det_stamp, self.pose_max_extrapolation_s)
        if then is None:
            return []
        return mwt.propagate_objects(people, then, car)

    def _door_mask_geometry(self, scan_stamp):
        """
        Return (labels, depth_mm, fx, fy, cx, cy, R, t, mask stamp) for the door stereo, or None.

        The same mask/depth pair, intrinsics and camera transform the
        rectangle verification uses (_verify_tracks), so the door's stereo
        evidence is the same measurement the walls' is. R, t are the camera's
        mount: base_link at the MASK's moment (_door_stereo_frame carries
        them to the scan's).
        """
        if not self.rect_verify or self._stereo_columns is None:
            return None
        profile = self._stereo_columns[1]
        pair = self._mask_pair(scan_stamp)
        if pair is None or int(profile.width) < 1:
            return None
        labels, depth = pair[0], pair[1]
        cam = self._camera_transform(self._mask_classes.header.frame_id)
        if cam is None:
            return None
        scale = labels.shape[1] / float(profile.width)
        return (labels, depth, float(profile.fx) * scale, float(profile.fy) * scale,
                float(profile.cx) * scale, float(profile.cy) * scale, cam[0], cam[1],
                _stamp_sec(self._mask_classes.header))

    def _door_stereo_frame(self, car, scan_stamp):
        """
        Return this cycle's door StereoFrame, valid for base_link at the scan stamp, or None.

        door_candidates calls this at most once a cycle and only when a gap
        reaches its STEREO stage. The candidates are base_link at the scan
        stamp (`car` is the odometry pose there), the mask/depth pair is from
        its own, older stamp: the camera extrinsic is carried across on the
        odometry (extrinsic_at_scan). With no pose at the mask stamp the pair
        is used as it is only within DOOR_STEREO_UNCOMPENSATED_MAX_S of the
        scan; further, there is no stereo opinion this cycle. The mask's age
        and what became of it go on the debug topic (door_mask_age_s,
        door_stereo).
        """
        geometry = self._door_mask_geometry(scan_stamp)
        if geometry is None:
            self._door_stereo = 'no_pair'
            return None
        labels, depth, fx, fy, cx, cy, rot, trans, mask_stamp = geometry
        age = scan_stamp - mask_stamp
        self._door_mask_age = age
        pose_mask = self._poses.at(mask_stamp, self.pose_max_extrapolation_s)
        if car is not None and pose_mask is not None:
            rot, trans = extrinsic_at_scan(rot, trans, car, pose_mask)
            self._door_stereo = 'compensated'
        elif abs(age) > DOOR_STEREO_UNCOMPENSATED_MAX_S:
            self._door_stereo = 'no_pose'
            return None
        else:
            self._door_stereo = 'uncompensated'
        return door_detection.StereoFrame(labels, depth, fx, fy, cx, cy, rot, trans,
                                          depth_scale=0.001)

    def _publish_doors(self, stamp, doors, car, scan_stamp):
        """
        Publish /perception/doors and /viz/doors, every door cycle, empty included.

        /perception/doors carries the CONFIRMED doors only -- the CANDIDATES
        too with door_publish_candidates. /viz/doors draws every live door,
        confirmed solid and candidate faded, whatever that flag says.
        """
        out = DoorArray()
        out.header.stamp = stamp
        out.header.frame_id = self._odom_frame
        to_bl = car.inverse()
        for door in doors:
            if self.door_publish_candidates or door.state == door_detection.STATE_CONFIRMED:
                out.doors.append(door_message(door, to_bl, scan_stamp))
        self.doors_pub.publish(out)
        # /viz/doors goes out on EVERY cycle door detection runs, subscribed
        # or not -- unlike /viz/walls -- so a mission bag records it and the
        # doors show in Foxglove on a recorded run too. A handful of markers
        # at the tracker's ~10 Hz.
        self.viz_doors_pub.publish(
            self._viz_doors_builder.build(doors, stamp, self._odom_frame))

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
        then = self._poses.at(self._detections[2], self.pose_max_extrapolation_s)
        if not objs or then is None:
            return 0
        objs = mwt.propagate_objects(objs, then, car)
        confirmed = [mwt.segment_in_base_link(t, car)
                     for t in self.wall_tracker.tracks
                     if t.ever_confirmed and not t.contradicted]
        if not confirmed:
            return 0
        pts = np.array([[x, y] for x, y, _r in objs], dtype=float)
        blocked = mwt.ray_blocked_by(pts, laser_pose[:2], confirmed,
                                     self.wall_tracker.p.occlusion_margin_m)
        return int(np.count_nonzero(blocked))

    def _track_payload(self):
        """Every LIVE track as a dict, for /perception/virtual_wall/debug.

        Not only the published four: the whole point of 'unknown' and of the
        rectangle's REJECTED is that a track can be real, tracked and a
        reference for nothing, and a payload that showed only the published
        ones would hide exactly those. `role` is the REPORTED one
        (mwt.reported_role): 'unknown' for a tentative track, as on
        /perception/walls and /viz/walls.
        """
        car = self._car_pose
        if not self.debug_tracks or self.wall_tracker is None or car is None:
            return None
        def num(x):
            return None if x is None or not math.isfinite(x) else round(float(x), 4)
        out = []
        for trk in self.wall_tracker.tracks:
            bl = mwt.track_in_base_link(trk, car)
            _perp, d_ray = mwt.front_distances(trk, car, self.front_x)
            out.append({
                'id': int(trk.id), 'role': mwt.reported_role(trk),
                'state': mwt.STATE_NAMES.get(trk.state), 'hits': int(trk.hits),
                # THE CONFIRMATION LEVEL: '' once confirmed (sticky), else the
                # test a tentative track is still failing (confirm_reason).
                'confirm_reason': ('' if trk.ever_confirmed
                                   else self.wall_tracker.confirm_reason(trk)),
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
                'rect_reject_streak': int(trk.rect_reject_streak),
                'is_front': bool(self.wall_tracker.is_front(trk, car, self.front_x)),
                # Block 3: the strip's verdict on this track and why it is
                # (not) FRONT; '' front_reason means it may be.
                'front_reason': self.wall_tracker.front_reason(trk, car, self.front_x),
                'contradicted': bool(trk.contradicted),
                'strip_verdict': trk.strip_verdict, 'strip_miss': int(trk.strip_miss),
                'strip_pred_m': num(trk.strip_pred_m),
                'confidence': num(self.wall_tracker.front_confidence(trk)),
                'lidar_recent': bool(any(trk.lidar_hist)),
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
        for trk in tracks:
            bl = mwt.track_in_base_link(trk, car)
            # The REPORTED role (mwt.reported_role): a TENTATIVE track is
            # 'unknown' on the wire and in its marker whatever its geometry
            # says. The tracker's own Track.role, which FRONT and published()
            # read, is not touched.
            label = mwt.reported_role(trk, self.wall_tracker.label(trk, car))
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
            # The FRONT gate's confidence (front_confidence), not the last
            # observation's: the one number something actually decides on.
            m.confidence = float(self.wall_tracker.front_confidence(trk))
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
            # standing on the floor, not a line on it. EVERY slab is
            # wall_viz_height_m tall whatever the stereo measured (z_min /
            # z_max above are untouched): one corridor's walls used to be
            # drawn at as many heights as the stereo reported. Whether the
            # height was measured is the alpha below.
            z_top = self.wall_viz_height_m
            (ax, ay), (bx, by) = (x0, y0), (x1, y1)
            length = math.hypot(bx - ax, by - ay)
            yaw = math.atan2(by - ay, bx - ax)
            slab = Marker()
            slab.header = out.header
            slab.ns, slab.id, slab.type = 'walls', 2 * trk.id, Marker.CUBE
            slab.action = Marker.ADD
            slab.pose.position = Point(x=float(0.5 * (ax + bx)), y=float(0.5 * (ay + by)),
                                       z=float(0.5 * z_top))
            slab.pose.orientation.z = math.sin(yaw / 2.0)
            slab.pose.orientation.w = math.cos(yaw / 2.0)
            slab.scale.x = max(float(length), 0.01)
            slab.scale.y = self.wall_marker_thickness_m
            slab.scale.z = max(float(z_top), 0.01)
            r, g, b = _ROLE_COLOUR.get(label, _ROLE_COLOUR[mwt.ROLE_UNKNOWN])
            slab.color.r, slab.color.g, slab.color.b = r, g, b
            # A wall only assumed to be this tall is drawn fainter than one
            # whose height was actually seen.
            slab.color.a = 0.65 if trk.height_measured else 0.35
            _set_lifetime(slab, self.wall_marker_lifetime_s)
            text = Marker()
            text.header = out.header
            text.ns, text.id, text.type = 'walls', 2 * trk.id + 1, Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=float(0.5 * (ax + bx)), y=float(0.5 * (ay + by)),
                                       z=float(z_top + 0.15))
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color.a = text.color.r = text.color.g = text.color.b = 1.0
            text.text = (f'{trk.id} {label} {d_perp:.2f}m '
                         f'{wall_fusion.SOURCE_NAMES[trk.source]} '
                         f'{mwt.STATE_NAMES[trk.state]}')
            _set_lifetime(text, self.wall_marker_lifetime_s)
            markers.markers += [slab, text]
        # The ids drawn last cycle and not this one: gone, or pushed out of
        # published()'s cap by a nearer wall of the same role. Either way the
        # marker has to go, and it goes by id -- a DELETE for a marker that is
        # already expired is a no-op, so this is safe to send unconditionally.
        drawn = {trk.id for trk in tracks}
        for dead_id in sorted(self._drawn_track_ids - drawn):
            for slot in (2 * dead_id, 2 * dead_id + 1):
                gone = Marker()
                gone.header = out.header
                gone.ns, gone.id = 'walls', slot
                gone.action = Marker.DELETE
                markers.markers.append(gone)
        self._drawn_track_ids = drawn
        self.walls_pub.publish(out)
        self.walls_markers_pub.publish(markers)
        self._publish_viz_walls(stamp, car)

    def _publish_viz_walls(self, stamp, car):
        """Every track the tracker holds, on /viz/walls, for a human.

        Three gates, cheapest first: the flag, then the subscription count,
        then the rate. NOTHING is built when no one is watching -- that is the
        whole bargain that lets this topic carry every track at a readable
        rate while the stack is CPU-bound, and it is the same lazy behaviour
        the topic_tools relays behind every other /viz topic have (their
        lazy:=true), reproduced here because no relay can invent the tracks
        /perception/walls/markers never carried.
        """
        if not self.viz_walls_enabled:
            return
        if self.viz_walls_pub.get_subscription_count() == 0:
            # The markers expire on their own lifetime, so the next array
            # that is actually sent must not open with DELETEs for ids no
            # live client ever saw.
            self._viz_walls_builder.forget()
            self._viz_walls_next = None
            return
        now = stamp.sec + stamp.nanosec * 1e-9
        if self._viz_walls_next is not None and now < self._viz_walls_next:
            return
        period = 1.0 / self.viz_walls_max_hz if self.viz_walls_max_hz > 0.0 else 0.0
        self._viz_walls_next = now + period
        tracks = list(self.wall_tracker.tracks)
        distances = {}
        for trk in tracks:
            d_perp, _d_ray = mwt.front_distances(trk, car, self.front_x)
            distances[trk.id] = d_perp
        self.viz_walls_pub.publish(
            self._viz_walls_builder.build(tracks, stamp, self._odom_frame, distances))

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
