"""py_trees Condition: is anything too close, per raw hardware sensors alone?

Last-resort emergency stop, deliberately independent of the YOLO/
classification/obstacle-array pipeline (Detection2DArray/Detection3DArray/
Obstacle2DArray) -- built directly from two raw sensor sources so it keeps
working even if perception/classification is degraded or lagging:

  - Front: f1tenth_perception's front_depth_monitor_node, /perception/
    front_distance (std_msgs/Float32) -- raw ZED depth, no YOLO involved (see
    that node's own docstring). Trips if front_distance < front_distance_
    threshold (default 0.40 m).
  - Sides + rear: raw /scan (sensor_msgs/LaserScan) directly, no frame/TF
    conversion. Each sample's angle is computed in the RAW scan frame as
    msg.angle_min + i * msg.angle_increment -- angle 0 there is the lidar's
    OWN forward axis, which is physically FRONT-facing on this car as of the
    lidar remount (rear-facing -> front-facing) done alongside the first
    slam_toolbox integration pass -- confirmed against f1tenth_bringup/
    config/sensors.yaml: angle_min/max = +-3.14, a full-circle scan, and the
    base_link->laser static transform, now mounted at yaw=0.0 (was pi before
    the remount). The window |angle| >= lidar_blind_cone_half_angle_deg
    (default 45 deg -- HALF of the SAME 90 deg total blind-cone size the
    pre-remount 135 deg "include" threshold implied, since 180-135=45; the
    parameter's own MEANING flipped from "how much to include" to "how much
    to exclude" along with the mount, not just its value, hence the rename
    from lidar_window_half_angle_deg) therefore covers a 270 deg arc (left/
    right/rear) and deliberately excludes the ~90 deg forward cone (now
    centered on raw angle 0, since 0 deg is now the front) that the ZED
    front check above already covers -- do NOT reuse this angle logic
    anywhere that assumes angle 0 is the car's rear (that assumption is now
    stale post-remount). inf/nan samples and anything below msg.range_min
    are skipped before the window filter, never treated as a valid
    near-zero hit. Trips if the minimum in-window range < lidar_distance_
    threshold (default 0.20 m).

Either condition tripping is enough -- wired into the emergency lane's
Selector alongside IsBatteryLow/IsSystemOverheated/IsEmergencyStopTriggered
(see behavior_executor_node.create_root()), reusing the existing Stop
behaviour onto the safety_stop mux lane (priority 200): no new actuation path,
the mux's priority ordering is what actually stops the car. Unconditional, not
gated behind any enable_* toggle -- same reasoning as IsBatteryLow: core
hardware proximity safety should not default to toggleable-off. (Flagged per
the task: if an override is ever wanted, that's a deliberate follow-up
decision, not something added here by default.)

Fails (does not trip) until at least one message has been received on the
relevant topic for that half of the check -- same "no data yet must not mean
tripped" reasoning as IsBatteryLow's has_data guard, applied independently per
sensor: a stale/never-arrived /scan does not suppress the front check, and a
stale/never-arrived front_distance does not suppress the lidar check. This
also correctly covers "a /scan message arrived but nothing was within range
in the window" (all inf/filtered) -- that's a genuinely clear reading, not
missing data, and both cases resolve to "not tripped" either way, so no extra
state is needed to tell them apart.

front_distance_threshold/lidar_distance_threshold: normally passed in by
behavior_executor_node.create_root(), derived from stack_params.yaml's shared
car_radius/obstacle_safety_margin_m/proximity_front_extra_margin_m
(safety-margin unification pass -- see car_radius's own comment for the full
4-mechanism picture). lidar_distance_threshold <- car_radius alone (trip at
roughly the car's own physical radius, no extra margin -- pure
contact-avoidance last resort, zero other coverage on the sides/rear).
front_distance_threshold <- car_radius + obstacle_safety_margin_m +
proximity_front_extra_margin_m (front already has YOLO-lane coverage via
handle_obstacle, so keeps its own small extra backstop conservatism on top).
The 0.20/0.40 defaults here are that same derivation's result, unchanged
numerically from before this pass -- kept in sync for standalone
construction/tests, not re-derived independently.
"""

import math

import py_trees
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


# ==============================================================================
# Windowing -- pure function, no rclpy dependency, independently unit-
# testable (same "pure logic separate from ROS glue" convention
# f1tenth_perception/lidar_boundary_node.py's own _classify_side/
# _scan_angle_to_car_frame already use). Extracted here (was previously
# inline in _scan_callback) specifically so the front-facing-remount window
# flip below is covered by a real test, not just eyeballed.
# ==============================================================================

def _in_lidar_window(raw_angle: float, blind_cone_half_angle_rad: float) -> bool:
    """True if `raw_angle` (RAW scan frame -- 0 = lidar's own forward axis,
    physically the car's FRONT post-remount, see module docstring) falls
    OUTSIDE the front blind cone, i.e. should count toward this behaviour's
    side/rear proximity check. The blind cone is +-blind_cone_half_angle_rad
    around raw angle 0 -- excluded here because the ZED-based front check
    above already covers that direction."""
    return abs(raw_angle) >= blind_cone_half_angle_rad


class IsProximityTooClose(py_trees.behaviour.Behaviour):

    def __init__(
        self,
        name='IsProximityTooClose',
        front_distance_topic='/perception/front_distance',
        scan_topic='/scan',
        front_distance_threshold=0.40,
        lidar_distance_threshold=0.20,
        lidar_blind_cone_half_angle_deg=45.0,
    ):
        super().__init__(name=name)
        self.front_distance_topic = front_distance_topic
        self.scan_topic = scan_topic
        self.front_distance_threshold = front_distance_threshold
        self.lidar_distance_threshold = lidar_distance_threshold
        self.lidar_blind_cone_half_angle_rad = math.radians(lidar_blind_cone_half_angle_deg)

        self.node = None
        self.front_distance_sub = None
        self.scan_sub = None

        self.latest_front_distance = None
        self.latest_lidar_min_in_window = None

        # Diagnostic-only (live investigation, see chat) -- logged once on the
        # first /scan message received, not every callback.
        self._logged_scan_layout_once = False

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError(
                "IsProximityTooClose.setup() didn't find 'node' in kwargs") from e
        self.front_distance_sub = self.node.create_subscription(
            Float32, self.front_distance_topic, self._front_distance_callback, 10)
        self.scan_sub = self.node.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, 10)

    def _front_distance_callback(self, msg: Float32):
        self.latest_front_distance = float(msg.data)

    def _scan_callback(self, msg: LaserScan):
        # ---- DIAGNOSTIC (live investigation, see chat) -- one-time raw
        # message layout, so real angle_min/max/increment/range_min can be
        # compared against what the window math below assumes. ----
        if not self._logged_scan_layout_once:
            self._logged_scan_layout_once = True
            self.node.get_logger().info(
                f'[IsProximityTooClose] DIAG /scan layout (first message): '
                f'angle_min={msg.angle_min:.4f} rad '
                f'({math.degrees(msg.angle_min):+.1f} deg), '
                f'angle_max={msg.angle_max:.4f} rad '
                f'({math.degrees(msg.angle_max):+.1f} deg), '
                f'angle_increment={msg.angle_increment:.6f} rad '
                f'({math.degrees(msg.angle_increment):.4f} deg), '
                f'range_min={msg.range_min:.3f} m, len(ranges)={len(msg.ranges)}'
            )

        total = len(msg.ranges)
        in_window_count = 0
        excluded_by_filter = 0
        min_in_window = None

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if not _in_lidar_window(angle, self.lidar_blind_cone_half_angle_rad):
                continue
            in_window_count += 1
            if not math.isfinite(r) or r < msg.range_min:
                excluded_by_filter += 1
                continue
            if min_in_window is None or r < min_in_window:
                min_in_window = r

        self.latest_lidar_min_in_window = min_in_window

        # ---- DIAGNOSTIC (live investigation, see chat) -- throttled, every
        # callback, so "not firing at all" vs. "firing but computing wrong"
        # can be told apart from the logs alone. ----
        self.node.get_logger().info(
            f'[IsProximityTooClose] DIAG scan: total={total} '
            f'in_window={in_window_count} excluded_by_filter={excluded_by_filter} '
            f'min_in_window='
            + (f'{min_in_window:.3f}' if min_in_window is not None else 'none'),
            throttle_duration_sec=2.0,
        )

    def update(self):
        front_tripped = (
            self.latest_front_distance is not None
            and self.latest_front_distance < self.front_distance_threshold
        )
        lidar_tripped = (
            self.latest_lidar_min_in_window is not None
            and self.latest_lidar_min_in_window < self.lidar_distance_threshold
        )
        return (
            py_trees.common.Status.SUCCESS
            if (front_tripped or lidar_tripped)
            else py_trees.common.Status.FAILURE
        )
