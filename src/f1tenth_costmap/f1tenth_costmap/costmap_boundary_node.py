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

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32

from f1tenth_messages.msg import BoundaryConstraint, BoundaryConstraintArray

from f1tenth_costmap.costmap_boundary import (
    extract_boundary_constraints, front_clearance_from_extraction, yaw_from_quaternion)

# Order constraints are appended to the published array in -- arbitrary but
# fixed, so a consumer that happens to care about position (none currently
# do; mpc_corr's own pad_boundary_constraints treats the list as an
# unordered set of up to 3 sources, see that function's own docstring)
# still sees a deterministic order across ticks.
_DIRECTIONS = ('front', 'left', 'right')


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

        p = self.get_parameter
        self.robot_frame = str(p('robot_frame').value)
        self.front_facing_max_rad = math.radians(float(p('front_facing_max_deg').value))
        self.side_window_min_rad = math.radians(float(p('side_window_min_deg').value))
        self.side_window_max_rad = math.radians(float(p('side_window_max_deg').value))
        self.occupied_threshold = float(p('occupied_threshold').value)
        self.max_range_m = float(p('max_range_m').value)
        extraction_rate_hz = float(p('extraction_rate_hz').value)

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

        self.map_sub = self.create_subscription(
            OccupancyGrid, str(p('map_topic').value), self._map_cb, map_qos)
        self.pose_sub = self.create_subscription(
            Odometry, str(p('pose_topic').value), self._pose_cb, pose_qos)
        self.boundary_pub = self.create_publisher(
            BoundaryConstraintArray, str(p('output_topic').value), 10)
        self.clearance_pub = self.create_publisher(
            Float32, str(p('front_clearance_topic').value), 10)

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

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

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

        extraction = extract_boundary_constraints(
            grid.data, grid.info.width, grid.info.height, grid.info.resolution,
            grid.info.origin.position.x, grid.info.origin.position.y,
            robot_x, robot_y, robot_yaw,
            self.front_facing_max_rad, self.side_window_min_rad, self.side_window_max_rad,
            self.occupied_threshold, self.max_range_m)

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

        clearance = front_clearance_from_extraction(extraction, self.max_range_m)
        self.clearance_pub.publish(Float32(data=float(clearance)))

        self.get_logger().info(
            f'n_constraints={len(arr.constraints)} front_clearance={clearance:.2f}m',
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
