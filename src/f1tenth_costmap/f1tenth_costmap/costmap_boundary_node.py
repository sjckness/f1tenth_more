#!/usr/bin/env python3
"""
costmap_boundary_node.py

Retires wall_detector_node.py (f1tenth_perception, ZED point-cloud RANSAC
plane fit) and lidar_boundary_node.py (f1tenth_perception, raw /scan total-
least-squares line fit): both re-derived "where is the nearest wall/
boundary" from raw sensor data independently, every tick, with no memory of
what the stack has already integrated. Once a real SLAM occupancy grid
exists (/slam/map), that map IS the accumulated record of every lidar/ZED
return the stack has ever fused -- this node reads boundaries FROM it
instead, via costmap_boundary.py's own pure nearest-occupied-cell-per-
direction extraction (see that module's own docstring for the extraction
algorithm and the sign-convention/frame derivation).

Publishes:
  - /costmap/boundaries (f1tenth_messages/BoundaryConstraintArray, 0-3
    entries: front/left/right, each independently present or absent --
    same variable-length "no data" contract wall_detector_node's own
    /perception/front_wall_boundary and lidar_boundary_node's own
    /perception/lidar_boundaries already established, just unified into
    ONE topic/array now that one node produces all three directions).
  - /costmap/front_clearance (std_msgs/Float32), replacing wall_detector_
    node's own /perception/front_clearance -- see costmap_boundary.py's
    own front_clearance_from_extraction() docstring for why this reports
    max_range_m + 1.0 (a finite, honest "clear at least this far") rather
    than wall_detector_node's own +inf convention when the front cone has
    no occupied cell in range.

INPUTS -- and the /slam/pose situation this whole pass is built around:
  - /slam/map (nav_msgs/OccupancyGrid), slam_toolbox's own occupancy grid,
    transient-local QoS (same latched-durability match costmap_renderer_
    node.py's own module docstring already verified against slam_toolbox's
    own map-publishing convention, reused here rather than re-derived).
  - The GLOBAL EKF's own fused map-frame pose (f1tenth_localization/launch/
    ekf_global.launch.py's ekf_global_filter_node, /ekf_global/odometry/
    filtered by default -- see that launch file's own "REAL BUG FOUND..."
    docstring paragraph for why this is a REMAPPED, non-default topic name)
    -- used DIRECTLY per this pass's own Part 4 instruction, NOT /slam/pose
    (slam_toolbox's raw pose), since the global EKF exists in this same
    pass and is the intended final map-frame pose source. QoS matched to
    MPC_corr.py's own odom subscription (BEST_EFFORT/VOLATILE) -- the
    global EKF is the same vendored robot_localization ekf_node binary as
    the local EKF MPC_corr.py already subscribes to, so this reuses that
    QoS precedent rather than semantic_layer_node.py's own /slam/pose QoS
    match (a different message TYPE and a different, slam_toolbox-owned
    publisher, not the right precedent to match here).

Neither input is live-verifiable as of this pass: /slam/pose is confirmed
(a prior, separate investigation this session) to never publish at all,
which means the global EKF's own pose0 correction never fires either --
its output degrades to pure odom0 dead-reckoning of the LOCAL EKF's own
estimate, composed on top of whatever map -> odom the global EKF last held
(likely never initialized at all if pose0 has never once been available to
seed it). This node is built to fail SAFE regardless: see "STALENESS"
below. VESC and lidar are ALSO currently physically disconnected (this
pass's own hardware-constraint addendum) -- so /slam/map itself has no real
lidar data to accumulate either. This node's own correctness is therefore
verified via code/config inspection, direct-construction unit tests, and
synthetic-grid tests only this pass -- NOT a live run. See this pass's own
final report for exactly what needs confirming once VESC/lidar are
reconnected and /slam/pose is fixed.

STALENESS -- reuses MPC_corr.py's own now_sec - last_time < timeout
freshness-gating PATTERN (not its exact value for both inputs -- see
below), applied independently to each input, BEFORE calling into costmap_
boundary.py's own extraction at all:
  - map_stale_timeout_sec (default 15.0s): /slam/map is legitimately
    event-driven, not periodic -- slam_toolbox_params.yaml's own
    map_update_interval is 5.0s, and a real update can take even longer if
    the robot isn't moving enough to trigger one (minimum_travel_distance/
    heading gates). 15.0s (3x map_update_interval) is a REASONED STARTING
    POINT allowing a couple of legitimately-missed update cycles before
    concluding the map source itself has actually gone stale/died, not
    tuned against real data (blocked on the same disconnected-hardware
    constraint as everything else this pass touches).
  - pose_stale_timeout_sec (default 0.5s): reuses odom_stale_timeout_sec's
    own DEFAULT VALUE directly (not the param itself -- this node has its
    own declared param, deliberately, since it's a different node with no
    access to mpc_corr's own parameter) -- appropriate because the global
    EKF publishes at the same continuous 50Hz frequency= the local EKF
    does, so "stale" here means the SAME kind of thing it means for
    MPC_corr.py's own hw/sim odom staleness check: the publisher process
    itself has stopped, not "no new absolute correction has arrived
    recently" (pose0/slam corrections are sparse by design -- see ekf_
    global.yaml's own docstring -- but the FILTER's own continuous output
    should never itself go quiet for more than a fraction of a second
    while the node is alive).

PERIODIC-PUBLISH pass (supersedes the STALENESS design above -- kept as
history, not current behavior, see the paragraph right after this one for
why): the freshness-gated design above turned out to be the actual bug, not
just an unverified corner case. Live evidence: front_clearance published
EXACTLY ONCE, coincident with /slam/pose's own single publish -- i.e. once
pose_age exceeded pose_stale_timeout_sec (0.5s) after that first message,
EVERY subsequent tick fell into the stale/withhold path forever, even
though the timer itself kept ticking at extraction_rate_hz the whole time.
/slam/pose (and by extension anything downstream of it) going quiet at rest
is NORMAL, not a fault -- see the separate hypothesis-3 investigation this
session: slam_toolbox's own outer scan-accept gate (shouldProcessScan,
Pose2::SquaredDistance) is position-only, so a car that's rotating in place
with zero x/y translation never re-triggers a scan no matter how much yaw
has drifted, regardless of anything this node does. Gating THIS node's
publish on upstream recency was therefore never going to work at rest by
design, not just an untuned timeout.

Fixed by removing the recency check entirely, keeping only "has a message
of each kind EVER been received" (map_stale_timeout_sec/pose_stale_timeout_sec
and the age bookkeeping that fed them are gone -- _map_last_time/
_pose_last_time are still recorded on receipt for potential future
diagnostics, just no longer read by _extraction_tick). Every tick now
either publishes real, freshly-recomputed boundaries + front_clearance from
whatever grid/pose are currently cached (no matter how old), or -- only if
one of them has literally never arrived even once -- the same fail-safe
empty-array-plus-withheld-clearance pairing STALENESS established, logged
at WARN (throttled ~5s) so silence stays visible. Freshness is now this
node's own publish cadence (extraction_rate_hz, see below) -- consumers
judge "is this current" by whether /costmap/front_clearance itself keeps
arriving at that rate, not by how recently the upstream map or pose
updated; that's a deliberate shift, not an oversight (map updates are
inherently sparse -- slam_toolbox_params.yaml's own map_update_interval is
5.0s -- so gating THIS node's output on map recency was always going to
starve consumers of a normal, expected sparse-update cadence, not just at
rest). One real tradeoff worth naming, not glossed over: if the pose
publisher itself dies after publishing at least once, this node has no way
to distinguish that from "just no new correction yet" anymore and will
keep computing boundaries against an increasingly-stale cached pose
indefinitely -- there's no longer a hard cutoff. That's the explicit
tradeoff this pass's own instructions asked for (remove the staleness gate
entirely, no dedup/skip-if-unchanged), not a gap introduced by accident.

CONVEX SAFE CORRIDOR (use_convex_polytope, DEFAULT FALSE) -- built by this
pass, deliberately not enabled. With the flag off this node behaves exactly
as described above and none of the code below runs; MPC_corr.py's own
use_hard_boundary_constraints is also still False, so even a published
polytope reaches no solve. There is a straight-line drive test pending that
this must not ride along into.

When the flag is ON, /costmap/boundaries carries the faces of ONE convex
polytope (up to polytope_max_faces, default 8) inflated around the MPC's own
reference centreline instead of the three nearest-cell half-planes. The
method and its justification live in safe_corridor.py's module docstring;
what belongs here is the plumbing:

  - THE SEED COMES FROM /mpc/corridor_markers, the same MarkerArray
    MPC_corr already publishes for Foxglove. Its centreline is in the ODOM
    frame (MPC_corr._publish_corridor_markers is emphatic about this and
    explains why), while the occupancy grid is MAP frame, so the centreline
    is transformed odom -> map through the tf2 map -> odom edge before it
    can seed anything. No new publisher on the MPC side was needed.

  - THE OUTPUT IS STILL base_link, unchanged from the nearest-cell path,
    which is what lets this drop into MPC_corr's existing consumer with no
    change there at all. The polytope is built in map, then each face is
    rewritten into the car frame with the robot's own map-frame pose
    (safe_corridor.faces_to_car_frame). MPC_corr then transforms base_link
    -> its own odom-frame world with its own odom pose, so map and odom
    never have to agree numerically -- the map -> odom offset (1.88 m and
    32 deg on the run that motivated the corridor anchor fix) cancels
    through the robot instead of being applied twice or not at all.

  - THE CONSTRAINTS CARRY THE CORRIDOR'S STAMP, not this tick's clock, so a
    consumer can detect a generation mismatch. Consistency with the corridor
    matters more than freshness here: two agreeing things that are both a
    second old are safe, a fresh constraint set derived from a corridor the
    MPC has already replaced is not.

  - A DEGENERATE POLYTOPE PUBLISHES AN EMPTY ARRAY and says why on
    /costmap/safe_corridor_report (std_msgs/String, JSON). Unknown and
    out-of-bounds cells BLOCK in the polytope path (safe_corridor.py inverts
    this node's usual permissive convention on purpose), so a sparsely
    explored area legitimately produces a tiny polytope -- reported, never
    silently shrunk and shipped.

  - front_clearance IS NOT AFFECTED BY THIS FLAG. It is recomputed from the
    nearest-cell extraction every tick in both modes, because it feeds the
    behaviour tree's own front_clearance stop condition
    (f1tenth_behavior/mission/condition_eval.py) -- a live mission-critical
    path that must not move behind an experimental flag.

WHAT THE OUT-OF-BOUNDS INVESTIGATION FOUND, since this node owns that code
path and the standing URGENT bug (car stationary, costmap position drifting,
missions stopping instantly) named it as the open hypothesis. The hypothesis
was that an out-of-bounds lookup returns a false OCCUPIED. It does not, and
the truth is the opposite. Measured against the real extraction on a
synthetic grid covering [0, 2] m (full table in
safe_corridor.classify_local_cells' own docstring): a robot at (50, 50),
far outside its own map, gets front=None and a reported front_clearance of
6.00 m. _crop_indices clamps the search window to the grid extent, the
clamped window comes out empty, and front_clearance_from_extraction turns
"no occupied cell found" into max_range_m + 1.0. Out-of-bounds therefore
fails PERMISSIVE -- it can make a mission fail to stop, never stop
instantly, so it cannot be the instant-stop mechanism. An instant stop needs
a SMALL front_clearance, and the mechanism that produces one is the drift
already named in the symptom: the map-frame pose this node indexes the grid
with comes from the global EKF, whose pose0 correction never fires
(/slam/pose does not publish), so it dead-reckons; as it walks into mapped
wall cells the nearest occupied cell gets arbitrarily close, and
test_03_bounce_walls.json trips its front_clearance stop at 2.5 m. The
permissive out-of-bounds handling is still a real bug worth fixing on its
own -- it is simply a different one, and the polytope path does not share it.

EXTRACTION RATE -- a periodic timer (extraction_rate_hz, default 20.0 as of
the periodic-publish pass -- was 5.0), not per-incoming-message on either
input -- same "decouple this node's own compute cost from an irregular/
high-frequency upstream rate" precedent costmap_renderer_node.py's own
module docstring already establishes for its analogous periodic-timer
choice. 20Hz is this pass's own explicit, given target (not independently
derived here from check_stop_condition.py's own actual BT tick rate --
that wasn't checked this pass; worth confirming separately if the exact
number matters) -- now that this node's own publish cadence, not upstream
recency, is what consumers judge freshness by, this rate directly bounds
how current a consumer's view can ever be.
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32, String
from visualization_msgs.msg import MarkerArray

from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from f1tenth_messages.msg import BoundaryConstraint, BoundaryConstraintArray

from f1tenth_costmap.costmap_boundary import (
    extract_boundary_constraints, front_clearance_from_extraction, yaw_from_quaternion)
from f1tenth_costmap.safe_corridor import (
    MAX_FACES_DEFAULT, assert_reference_contained, build_safe_corridor, faces_to_car_frame)

# Order constraints are appended to the published array in -- arbitrary but
# fixed, so a consumer that happens to care about position (none currently
# do; mpc_corr's own pad_boundary_constraints treats the list as an
# unordered set of up to 3 sources, see that function's own docstring)
# still sees a deterministic order across ticks.
_DIRECTIONS = ('front', 'left', 'right')


def centreline_from_marker_array(msg, namespace: str):
    """(points, stamp) for the centreline marker in `msg`, or None.

    `points` is an (K, 2) float array of the marker's own (x, y) -- the
    corridor centreline in whatever frame the marker declares (ODOM, for
    MPC_corr's own publisher). `stamp` is that marker's header stamp,
    passed through untouched so it can be copied onto the constraints
    derived from it.

    Module-level and duck-typed (anything with .markers, each with .ns and
    .points) for the same reason costmap_boundary.py's own extraction
    functions are: the node's constructor creates real publishers and a
    real timer, so anything that wants direct unit coverage lives outside
    it.

    Returns None -- not an empty array -- when the namespace is absent or
    its marker carries fewer than two points, so the caller can tell "no
    corridor yet" from "a corridor with no room in it".
    """
    for marker in getattr(msg, 'markers', []):
        if getattr(marker, 'ns', None) != namespace:
            continue
        pts = [(float(p.x), float(p.y)) for p in marker.points]
        if len(pts) < 2:
            return None
        return np.asarray(pts, dtype=float), marker.header.stamp
    return None


def transform_points_odom_to_map(points, tf_x: float, tf_y: float, tf_yaw: float):
    """Rotate+translate odom-frame (x, y) points into the map frame.

    (tf_x, tf_y, tf_yaw) is the map -> odom edge exactly as tf2 returns it
    for lookup_transform('map', 'odom', ...) -- the pose of odom's origin
    expressed in map -- so applying it to an odom-frame point yields the
    map-frame one. Same direction and same formula as MPC_corr.py's own
    _pose_odom_to_map, reimplemented rather than imported for the same
    reason yaw_from_quaternion already is in this package: two lines of
    planar trigonometry are not worth a cross-package dependency on a
    control node.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    cos_t, sin_t = math.cos(tf_yaw), math.sin(tf_yaw)
    out = np.empty_like(pts)
    out[:, 0] = tf_x + cos_t * pts[:, 0] - sin_t * pts[:, 1]
    out[:, 1] = tf_y + sin_t * pts[:, 0] + cos_t * pts[:, 1]
    return out


def trim_centreline_ahead(points, robot_x: float, robot_y: float, r_local: float):
    """The stretch of centreline the polytope should be seeded on.

    Starts at the sample nearest (robot_x, robot_y) and walks FORWARD along
    the polyline, accumulating arclength, until r_local is reached or the
    centreline ends. Returns an (C, 2) array in the same order.

    Two reasons this trim exists rather than handing the whole centreline
    to build_safe_corridor. First, that function's contract: the blocked-
    cell window is a disc of radius r_local centred on the arc's FIRST
    point, so an arc longer than r_local would extend past the region whose
    obstacles were even collected -- faces would be missing for the far end,
    silently. Second, seeding from the car's own position forward is what
    makes coverage mean something: a covered prefix is the stretch the car
    is about to drive, whereas including corridor behind the car would
    spend the face budget separating obstacles it has already passed.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if pts.shape[0] == 0:
        return pts
    d2 = (pts[:, 0] - robot_x) ** 2 + (pts[:, 1] - robot_y) ** 2
    start = int(np.argmin(d2))
    ahead = pts[start:]
    if ahead.shape[0] < 2:
        return ahead
    seg = np.hypot(np.diff(ahead[:, 0]), np.diff(ahead[:, 1]))
    s_cum = np.concatenate(([0.0], np.cumsum(seg)))
    keep = int(np.searchsorted(s_cum, r_local, side='right'))
    return ahead[:max(keep, 1)]


class CostmapBoundaryNode(Node):
    def __init__(self, **kwargs):
        super().__init__('costmap_boundary_node', **kwargs)

        self.declare_parameter('map_topic', '/slam/map')
        self.declare_parameter('pose_topic', '/ekf_global/odometry/filtered')
        self.declare_parameter('output_topic', '/costmap/boundaries')
        self.declare_parameter('front_clearance_topic', '/costmap/front_clearance')
        self.declare_parameter('robot_frame', 'base_link')
        # deg, car frame -- see costmap_boundary.py's own module docstring
        # for why these reuse wall_detector_node's/lidar_boundary_node's
        # own established angular-window conventions rather than new ones.
        self.declare_parameter('front_facing_max_deg', 35.0)
        self.declare_parameter('side_window_min_deg', 45.0)
        self.declare_parameter('side_window_max_deg', 135.0)
        # OccupancyGrid.data value (0-100) at/above which a cell counts as
        # occupied -- REASONED STARTING POINT (biased toward "confidently
        # occupied", not the raw >0 boundary), not tuned against real SLAM
        # output yet -- same untuned-starting-point discipline this
        # codebase's other detection thresholds already follow.
        self.declare_parameter('occupied_threshold', 65)
        # m -- how far to search for the nearest occupied cell per
        # direction. REASONED STARTING POINT, roughly matching wall_
        # detector_node's own roi_x_max (6.0m) ballpark.
        self.declare_parameter('max_range_m', 5.0)
        # periodic-publish pass: bumped 5.0 -> 20.0, and this is now the ONLY
        # thing that governs how often /costmap/boundaries and /costmap/
        # front_clearance are published -- see module docstring's "PERIODIC-
        # PUBLISH pass" paragraph. map_stale_timeout_sec/pose_stale_timeout_sec
        # (previously declared here) are gone: staleness no longer gates the
        # publish at all, only "has a message of each kind ever arrived" does
        # (_extraction_tick below).
        self.declare_parameter('extraction_rate_hz', 20.0)

        # ---- CONVEX SAFE CORRIDOR (safe_corridor.py) -------------------
        # DEFAULT FALSE. Everything below is built but NOT enabled: with
        # this off, _extraction_tick runs exactly the nearest-occupied-cell
        # path it ran before, byte for byte, and none of the polytope code
        # executes. Deliberate -- there is a straight-line drive test
        # pending that must not have this ride along into it, and
        # MPC_corr.py's own use_hard_boundary_constraints is False too, so
        # even a published polytope would reach no solve.
        #
        # What flipping it to True changes: /costmap/boundaries carries the
        # faces of ONE convex polytope inflated around the MPC's own
        # reference centreline (up to polytope_max_faces of them) instead of
        # three nearest-cell half-planes. See safe_corridor.py's module
        # docstring for the method and for what is wrong with the three
        # half-planes.
        #
        # What it does NOT change, on purpose: /costmap/front_clearance is
        # still computed from the nearest-cell extraction on every tick,
        # whichever mode is selected. That scalar feeds the behaviour tree's
        # own front_clearance stop condition (condition_eval.py), which is a
        # LIVE mission-critical path; a polytope has no natural "distance to
        # the nearest thing straight ahead" to offer in its place, and
        # inventing one would put a mission-stopping signal behind a flag
        # whose whole point is that it is not yet trusted.
        self.declare_parameter('use_convex_polytope', False)
        # Topic carrying the MPC's own reference corridor. ODOM frame (see
        # MPC_corr.py's _publish_corridor_markers docstring, which is
        # emphatic about this), so it has to be transformed into the grid's
        # map frame before it can seed anything -- _corridor_in_map below.
        self.declare_parameter('corridor_markers_topic', '/mpc/corridor_markers')
        # Marker namespace holding the centreline polyline, as set by
        # MPC_corr._publish_corridor_markers. The wall polylines
        # (corridor_left/corridor_right) are in the same MarkerArray and are
        # deliberately ignored: seeding on the reference means the
        # CENTRELINE, and the walls are the corridor's own soft bound, not
        # an observation of anything.
        self.declare_parameter('corridor_centerline_ns', 'corridor_centerline')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('polytope_report_topic', '/costmap/safe_corridor_report')
        # m -- radius of the local window the polytope is carved out of, and
        # the arc length of centreline used to seed it (build_safe_corridor
        # expects an arc already trimmed to this). Matches max_range_m's own
        # ballpark deliberately: a polytope that reaches further than the
        # nearest-cell search did would be claiming knowledge from the same
        # map that the old path declined to claim.
        self.declare_parameter('polytope_r_local_m', 3.0)
        self.declare_parameter('polytope_max_faces', MAX_FACES_DEFAULT)
        # Degeneracy thresholds -- see safe_corridor.polytope_report. These
        # are REASONED STARTING POINTS, not tuned against a real map (the
        # same untuned-starting-point discipline occupied_threshold and
        # max_range_m above already carry). min_seed_clearance is set just
        # under the car's own half-width plus margin (car_radius 0.20 +
        # obstacle_safety_margin_m 0.12 = 0.32, see stack_params.yaml): a
        # polytope with less room than that around the reference point
        # cannot admit the car at all once the consumer subtracts its
        # margin, so applying it could only ever produce an infeasible QP.
        self.declare_parameter('polytope_min_seed_clearance_m', 0.32)
        self.declare_parameter('polytope_min_area_m2', 0.25)
        self.declare_parameter('polytope_min_covered_fraction', 0.25)

        p = self.get_parameter
        self.robot_frame = str(p('robot_frame').value)
        self.front_facing_max_rad = math.radians(float(p('front_facing_max_deg').value))
        self.side_window_min_rad = math.radians(float(p('side_window_min_deg').value))
        self.side_window_max_rad = math.radians(float(p('side_window_max_deg').value))
        self.occupied_threshold = float(p('occupied_threshold').value)
        self.max_range_m = float(p('max_range_m').value)
        extraction_rate_hz = float(p('extraction_rate_hz').value)

        self.use_convex_polytope = bool(p('use_convex_polytope').value)
        self.corridor_centerline_ns = str(p('corridor_centerline_ns').value)
        self.map_frame = str(p('map_frame').value)
        self.odom_frame = str(p('odom_frame').value)
        self.polytope_r_local_m = float(p('polytope_r_local_m').value)
        self.polytope_max_faces = int(p('polytope_max_faces').value)
        self.polytope_min_seed_clearance_m = float(p('polytope_min_seed_clearance_m').value)
        self.polytope_min_area_m2 = float(p('polytope_min_area_m2').value)
        self.polytope_min_covered_fraction = float(p('polytope_min_covered_fraction').value)

        # Same transient-local QoS match costmap_renderer_node.py's own
        # module docstring already verified against slam_toolbox's own
        # (latched) map-publishing convention -- reused here, not re-
        # derived independently.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Same BEST_EFFORT/VOLATILE match MPC_corr.py's own odom
        # subscription already establishes for this exact vendored
        # robot_localization ekf_node publisher pattern.
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._latest_grid: OccupancyGrid = None
        self._map_last_time = None
        self._latest_pose_xy_yaw = None  # (x, y, yaw) or None
        self._pose_last_time = None
        # (points_odom, stamp) from the most recent /mpc/corridor_markers,
        # or None. `stamp` is the corridor's OWN builtin_interfaces/Time as
        # MPC_corr stamped it (self.last_corridor_stamp, the instant the
        # corridor was actually computed -- not when the marker was
        # published), and it is copied verbatim onto every constraint array
        # derived from it. See _publish_polytope for why that matters more
        # than freshness does.
        self._latest_corridor = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(
            OccupancyGrid, str(p('map_topic').value), self._map_cb, map_qos)
        self.pose_sub = self.create_subscription(
            Odometry, str(p('pose_topic').value), self._pose_cb, pose_qos)
        self.boundary_pub = self.create_publisher(
            BoundaryConstraintArray, str(p('output_topic').value), 10)
        self.clearance_pub = self.create_publisher(
            Float32, str(p('front_clearance_topic').value), 10)
        # Diagnostics for the polytope path: one JSON object per tick
        # carrying the safe_corridor.polytope_report dict (cell counts,
        # unknown_fraction, n_faces, cap_hit, covered_fraction,
        # seed_clearance, area, degenerate + reasons). A String rather than
        # a custom message because it is a DIAGNOSTIC, not a control
        # input -- nothing in the stack parses it, and adding a message type
        # for a field set still being shaped would freeze it prematurely.
        # This is how a degenerate polytope gets REPORTED rather than
        # silently shrunk: see _publish_polytope.
        self.report_pub = self.create_publisher(
            String, str(p('polytope_report_topic').value), 10)
        self.corridor_sub = self.create_subscription(
            MarkerArray, str(p('corridor_markers_topic').value),
            self._corridor_cb, 10)

        self.timer = self.create_timer(1.0 / extraction_rate_hz, self._extraction_tick)

        self.get_logger().info(
            'costmap_boundary_node started -- map_topic='
            f'{p("map_topic").value} pose_topic={p("pose_topic").value} '
            f'extraction_rate_hz={extraction_rate_hz:.1f} (periodic-publish '
            'pass: publishes every tick from whatever is currently cached, '
            'regardless of how recently either input last updated -- fail-'
            'safe empty/withheld only if a message of that kind has never '
            'arrived at all yet. See module docstring for why gating on '
            'upstream recency was removed.)'
        )

    # ------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        self._latest_grid = msg
        self._map_last_time = self._now_sec()

    def _pose_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._latest_pose_xy_yaw = (pos.x, pos.y, yaw)
        self._pose_last_time = self._now_sec()

    def _corridor_cb(self, msg: MarkerArray):
        """Cache the MPC's own reference centreline from /mpc/corridor_markers.

        Stores (points_odom, stamp) -- ODOM-frame (x, y) pairs and the
        marker's own header stamp, which MPC_corr sets to the instant the
        corridor was computed rather than the instant it was published.
        """
        found = centreline_from_marker_array(msg, self.corridor_centerline_ns)
        if found is not None:
            self._latest_corridor = found

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    def _map_from_odom(self):
        """(tf_x, tf_y, tf_yaw) for the map -> odom edge, or None.

        rclpy.time.Time() means "latest available", matching MPC_corr.py's
        own _refresh_goal_anchor lookup and semantic_layer_node.py's own
        camera lookup rather than introducing a third convention. Looking it
        up at the CORRIDOR's stamp instead was considered and rejected: tf2
        would then extrapolate or fail for a corridor older than the buffer,
        and the failure mode of a missing transform here is that the whole
        polytope is withheld -- strictly worse than using an edge that moved
        a few centimetres since. The corridor stamp is still carried onto
        the output (see _publish_polytope); it is the CONSTRAINTS that must
        agree with the corridor generation, not the frame edge.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'tf2 lookup "{self.map_frame}" -> "{self.odom_frame}" failed: '
                f'{exc} -- withholding the polytope this tick',
                throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        return float(t.x), float(t.y), yaw_from_quaternion(tf.transform.rotation)

    def _polytope_tick(self, grid, pose) -> bool:
        """The convex-safe-corridor path. Returns True if it published.

        False means "could not, fall back" -- and every False path logs why.
        The caller then runs the nearest-cell extraction instead, so the
        node never simply goes quiet because the polytope path was
        unavailable.
        """
        if self._latest_corridor is None:
            self.get_logger().warn(
                'use_convex_polytope is set but no /mpc/corridor_markers '
                'centreline has arrived yet -- falling back to nearest-cell',
                throttle_duration_sec=5.0)
            return False

        points_odom, corridor_stamp = self._latest_corridor
        edge = self._map_from_odom()
        if edge is None:
            return False

        robot_x, robot_y, robot_yaw = pose
        centreline_map = transform_points_odom_to_map(points_odom, *edge)
        seed_arc = trim_centreline_ahead(
            centreline_map, robot_x, robot_y, self.polytope_r_local_m)
        if seed_arc.shape[0] == 0:
            self.get_logger().warn(
                'corridor centreline is empty after trimming to '
                f'{self.polytope_r_local_m:.2f} m -- falling back to nearest-cell',
                throttle_duration_sec=5.0)
            return False

        built = build_safe_corridor(
            grid.data, grid.info.width, grid.info.height, grid.info.resolution,
            grid.info.origin.position.x, grid.info.origin.position.y,
            seed_arc, self.polytope_r_local_m, self.occupied_threshold,
            max_faces=self.polytope_max_faces,
            min_seed_clearance=self.polytope_min_seed_clearance_m,
            min_area=self.polytope_min_area_m2,
            min_covered_fraction=self.polytope_min_covered_fraction)
        if built is None:
            return False

        self._publish_polytope(built, corridor_stamp, robot_x, robot_y, robot_yaw)
        return True

    def _publish_polytope(self, built, corridor_stamp, robot_x, robot_y, robot_yaw):
        """Publish one polytope's faces, plus its report.

        THE CONSTRAINTS CARRY THE CORRIDOR'S OWN STAMP, not this tick's
        clock. That is deliberate and it is the opposite of what a
        freshness-conscious publisher would do: a consumer comparing this
        stamp against the corridor generation it is currently tracking can
        tell whether the two agree, and two agreeing things that are both a
        second old are safe, while a fresh constraint set derived from a
        corridor the MPC has already replaced is not. Consistency beats
        freshness here because the whole point of seeding on the reference
        is that the reference is inside the result -- a claim that says
        nothing at all if the reference has since moved.

        A DEGENERATE POLYTOPE PUBLISHES AN EMPTY ARRAY plus the report
        naming every failing test. Empty is the same fail-safe the
        never-received path already uses, and it means "no constraints from
        me this tick", which the consumer treats as unconstrained rather
        than as a tiny box it must squeeze into. The alternative -- shipping
        the degenerate faces anyway -- is exactly the silent shrink this is
        built to avoid: constraints carved out of unexplored cells look
        identical to constraints carved out of observed walls once they are
        on the wire.
        """
        polytope, report = built['polytope'], built['report']

        arr = BoundaryConstraintArray()
        arr.header.stamp = corridor_stamp
        arr.header.frame_id = self.robot_frame

        if report['degenerate']:
            self.get_logger().warn(
                'safe corridor DEGENERATE, publishing no constraints: '
                + '; '.join(report['degenerate_reasons']),
                throttle_duration_sec=5.0)
        else:
            # Re-assert on exactly what is about to go on the wire, not just
            # on what inflate_polytope returned. Cheap, and it is the one
            # check that makes "the reference is inside its own constraints"
            # a property of the PUBLISHED message rather than of an
            # intermediate the node could still get wrong between the two.
            assert_reference_contained(
                polytope['A'], polytope['b'], polytope['covered_points'])
            for nx, ny, offset in faces_to_car_frame(
                    polytope['A'], polytope['b'], robot_x, robot_y, robot_yaw):
                arr.constraints.append(
                    BoundaryConstraint(normal=[nx, ny], offset=offset))

        self.boundary_pub.publish(arr)
        self.report_pub.publish(String(data=json.dumps(report, sort_keys=True)))

        if report['cap_hit']:
            self.get_logger().warn(
                f'polytope hit the {self.polytope_max_faces}-face cap with '
                f'{report["n_unseparated"]} blocked cells still un-separated -- '
                'the polytope does NOT fully exclude the local obstacle set',
                throttle_duration_sec=5.0)
        self.get_logger().info(
            f'polytope n_faces={report["n_faces"]} '
            f'covered={report["covered_fraction"]:.2f} '
            f'clearance={report["seed_clearance"]:.2f}m '
            f'area={report["area"]:.2f}m2 unknown={report["unknown_fraction"]:.2f} '
            f'oob={report["oob_fraction"]:.2f}',
            throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    def _extraction_tick(self):
        """periodic-publish pass: runs every timer tick unconditionally --
        see module docstring's own 'PERIODIC-PUBLISH pass' paragraph. Gated
        only on "has a message of each kind EVER arrived" (self._latest_grid/
        self._latest_pose_xy_yaw not None), never on how recently -- a
        currently-cached grid/pose of any age gets re-extracted and
        (re-)published fresh this tick, same content or not (no dedup)."""
        grid = self._latest_grid
        pose = self._latest_pose_xy_yaw

        if grid is None or pose is None:
            self._publish_empty(have_map=grid is not None, have_pose=pose is not None)
            return

        robot_x, robot_y, robot_yaw = pose

        # Convex-safe-corridor path, OFF BY DEFAULT (use_convex_polytope).
        # When it is on and succeeds it owns /costmap/boundaries for this
        # tick and returns; front_clearance below still runs either way, so
        # the behaviour tree's own stop condition is never affected by this
        # flag. When it is on and cannot run (no corridor yet, no TF, empty
        # arc) it logs why and falls through to the nearest-cell path rather
        # than leaving consumers with nothing.
        polytope_published = False
        if self.use_convex_polytope:
            polytope_published = self._polytope_tick(grid, pose)

        extraction = extract_boundary_constraints(
            grid.data, grid.info.width, grid.info.height, grid.info.resolution,
            grid.info.origin.position.x, grid.info.origin.position.y,
            robot_x, robot_y, robot_yaw,
            self.front_facing_max_rad, self.side_window_min_rad, self.side_window_max_rad,
            self.occupied_threshold, self.max_range_m)

        if not polytope_published:
            arr = BoundaryConstraintArray()
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.header.frame_id = self.robot_frame
            for direction in _DIRECTIONS:
                result = extraction[direction]
                if result is None:
                    continue
                nx, ny, offset = result
                arr.constraints.append(BoundaryConstraint(normal=[nx, ny], offset=offset))
            self.boundary_pub.publish(arr)
            n_published = len(arr.constraints)
        else:
            n_published = -1  # owned by _publish_polytope this tick.

        # front_clearance is computed from the nearest-cell extraction on
        # EVERY tick regardless of use_convex_polytope -- see that
        # parameter's own declaration comment. It feeds a live
        # mission-stopping behaviour-tree condition and is deliberately not
        # behind this flag.
        clearance = front_clearance_from_extraction(extraction, self.max_range_m)
        self.clearance_pub.publish(Float32(data=float(clearance)))

        self.get_logger().info(
            f'n_constraints={n_published} front_clearance={clearance:.2f}m',
            throttle_duration_sec=2.0)

    def _publish_empty(self, have_map: bool, have_pose: bool):
        """periodic-publish pass: fail-safe path, now reached ONLY when a
        message of that kind has literally never arrived yet -- never for
        staleness/recency anymore (see module docstring). Preserves the
        exact fail-safe pairing the previous STALENESS design already
        established: publish an EMPTY BoundaryConstraintArray (never a
        stale/held-over one) and skip the front_clearance publish entirely
        this tick -- front_clearance stays None to consumers (check_stop_
        condition.py's own EvalContext.front_clearance contract) rather
        than a fabricated/sentinel value."""
        arr = BoundaryConstraintArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = self.robot_frame
        self.boundary_pub.publish(arr)

        reasons = []
        if not have_map:
            reasons.append('map_topic: no message ever received')
        if not have_pose:
            reasons.append('pose_topic: no message ever received')
        self.get_logger().warn(
            'publishing EMPTY boundary constraints, front_clearance withheld -- '
            + '; '.join(reasons),
            throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapBoundaryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
