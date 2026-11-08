"""py_trees Action: run the current enter_door move -- doors in, a pose out.

  in   /perception/doors_semantic  f1tenth_messages/DoorArray, MAP frame, the
                                   CONFIRMED doors whichever detector
                                   door_source names is holding
       GLOBAL_XY_KEY / GLOBAL_YAW_KEY
                                   the vehicle in the map frame
                                   (CheckStopCondition's global EKF
                                   subscription -- the same pose the doors
                                   were built in, which is the whole reason
                                   this move can compare the two at all)
       /slam/map                   nav_msgs/OccupancyGrid, the grid the door
                                   detector reads; the Dubins path is checked
                                   against its occupied cells
  out  /mpc/goal_pose              PoseStamped, map: lookahead goals along the
                                   Dubins path, then the door centre, then
                                   one exit goal beyond it
       /mpc/hold                   on a failure outcome, which aborts the mission

THREE PHASES. A doorway is 0.9 m wide and the car is 0.27 m wide with a
turning radius R_min of ~0.96 m: arriving at the opening from an angle means
arriving wedged.

APPROACH follows a forward Dubins path (dubins.py, radius rho =
door_dubins_radius_factor * R_min) from the car to the ENTRY POSE: on the
door's own axis, door_entry_dist_m behind the opening, carrying the axis yaw.
The MPC is never given the path: it gets a goal door_lookahead_m further
along it (the path's own tangent as its yaw), re-sent only when that goal
moved by door_goal_republish_dist_m or turned by door_goal_republish_yaw_deg
(every /mpc/goal_pose resets mpc_corr's goal tracking). The last stretch
aims at the entry pose itself. The path is re-planned (at most
door_replan_max_hz) when the car is more than door_replan_dev_m off it or the
live door moved by more than 0.1 m / 5 deg.

"Infeasible" is: no Dubins word, or the shortest one's footprint hits an
occupied cell, or it is longer than 3 x the straight distance to the entry
pose. It is logged (INFEASIBLE); the path, if there is one, is still
followed -- forward-only driving has nothing better to offer.

APPROACH -> ENTER when (a) the car is within 0.3 m of the entry pose, or (b)
it is at or past the entry pose's along (along >= -door_entry_dist_m), within
1.5 x lat_tol of the axis AND within yaw_tol of the axis heading. Nothing
else: there is no "passed the pre-point" rule any more -- a car that went by
off the axis is off the path, re-plans, and if no forward path fits, backs up.

ENTER freezes the door pose (the track keeps refining; the car commits to
what it lined up on), drives at the door centre with goal_yaw_for's yaw, and
once the car is within 0.3 m of the centre or past it (along >= 0) sends ONE
exit goal door_exit_dist_m beyond the centre on the axis. No goal is sent
after that, so the car is never asked to turn back to a point behind it.
`entered` -- what the door_entered stop condition reads -- is true once the
car is through: along >= door_exit_dist_m - 0.2, or within 0.2 m of the exit
goal (mpc_corr's arrival latch is 0.15 m).

ONLY A STRONG DOOR MAY BE SELECTED. `Door.strong` is the detector's own
verdict on whether a door has earned being driven at -- enough sightings, from
far enough apart, no flags (Door.msg, semantic_doors.DoorTracker.is_strong).
A matching door that is merely CONFIRMED fails the move with
`no_strong_door`, which is a different answer from `door_not_found` on
purpose: one says "there is a door there but it has not earned it", the other
says "there is no such door". Strength gates SELECTION only -- once resolved,
a door is followed through the confirmed list, so one that dips below strong
mid-approach does not read as `door_lost` and the car finishes the approach it
committed to. The wall detectors publish every confirmed door strong, so for
them none of this changes anything.

THE DOOR IS RESOLVED ONCE. "nearest" picks the confirmed door nearest the car
at the moment the move starts and then holds that door's id for the rest of
the move; "door_<id>" names one outright; "left"/"right" (select_door's
side_of()) pick the nearest STRONG door on that side of the car's heading AT
THAT SAME MOMENT -- for the case nearest alone cannot resolve, two doors
facing each other across a corridor at roughly the same distance, where
"nearest" would be deciding on noise.

ARRIVAL IS MEASURED HERE, not by mpc_corr: along, lateral and yaw against the
door frame every tick, written into the move's DoorApproachRecord.

FAILURE IS CLEAN AND IT IS FINAL. No confirmed door to resolve within the
spec's acquire_timeout_sec -> `door_not_found`; a matching door that is
confirmed but not strong, for that long -> `no_strong_door`; the resolved door
gone from the topic for LOST_GRACE_SEC before ENTER -> `door_lost`. Each one
ends the move, holds the car and aborts the
mission with the reason on MissionRuntimeState.last_stop_reason. It never
falls back to another door.

Placed in mission_progress between PublishMoveGoal and CheckStopCondition,
beside GoToObject. For any other move type it clears any door record it was
holding and returns SUCCESS.
"""

import math
import time

import py_trees
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from f1tenth_messages.msg import Door, DoorArray

from f1tenth_behavior.behaviours import dubins
from f1tenth_behavior.mission.mission_config import DOOR_LEFT, DOOR_NEAREST, DOOR_RIGHT
from f1tenth_behavior.mission.move_scoring import record_move_outcome, write_mission_summary
from f1tenth_behavior.mission.runtime import (
    DoorApproachRecord,
    GLOBAL_TURN_ACCUM_KEY,
    GLOBAL_XY_KEY,
    GLOBAL_YAW_KEY,
    MISSION_KEY,
    MissionRuntimeState,
)

#: The car is "in the doorway already" at or past this x in the door frame
#: [m] when the move starts: APPROACH is skipped.
APPROACH_SKIP_X_M = -0.2

#: mpc_corr latches a /mpc/goal_pose arrival at this radius (its
#: pose_goal_tolerance, 0.15 m) and then HOLDS THE CAR. Any phase decision here
#: tighter than that can never fire for a car the MPC has already stopped.
MPC_POSE_TOL_M = 0.15

#: door_lat_tol_m default: clamp(LAT_TOL_SLACK_FRAC * (door width - car width),
#: LAT_TOL_MIN_M, LAT_TOL_MAX_M) [m]: a share of the clearance each side of the
#: car, bounded.
LAT_TOL_SLACK_FRAC = 0.3
LAT_TOL_MIN_M = 0.10
LAT_TOL_MAX_M = 0.25
#: Fallback car geometry [m]: 2 x swept_clearance_body_half_width_m (URDF
#: chassis), base_front_x, car_rear_offset_m.
DEFAULT_CAR_WIDTH_M = 0.272
DEFAULT_FRONT_M = 0.405
DEFAULT_REAR_M = 0.082

#: door_entry_yaw_tol_deg default. A crooked car that is ON the door's line and
#: pointing in counts: the heading only has to be good enough not to clip a jamb.
DEFAULT_ENTRY_YAW_TOL_DEG = 20.0

#: Closer than this [m] to the goal point the bearing is not meaningful.
NEAR_GOAL_M = 0.2

# ---- the Dubins approach: ROS params of the behaviour executor (defaults)
DEFAULT_DUBINS_RADIUS_FACTOR = 1.1     # rho = factor * R_min
DEFAULT_ENTRY_DIST_M = 0.5             # entry pose: along = -this
DEFAULT_REPLAN_DEV_M = 0.3
DEFAULT_REPLAN_MAX_HZ = 1.0
DEFAULT_GOAL_REPUBLISH_DIST_M = 0.2
DEFAULT_GOAL_REPUBLISH_YAW_DEG = 10.0
DEFAULT_EXIT_DIST_M = 0.8
#: door_lookahead_m <= 0 means auto: max(LOOKAHEAD_MIN_M, LOOKAHEAD_R_MIN_FACTOR * R_min).
LOOKAHEAD_MIN_M = 1.0
LOOKAHEAD_R_MIN_FACTOR = 1.5
#: R_min [m] when the steering limits cannot be read.
FALLBACK_R_MIN_M = 1.0

#: APPROACH -> ENTER rule (a): within this of the entry pose [m].
ENTRY_SWITCH_M = 0.3
#: rule (b): |lateral| within this multiple of lat_tol.
LAT_SWITCH_FACTOR = 1.5
#: ENTER -> EXIT: within this of the door centre [m] (or along >= 0).
EXIT_SWITCH_M = 0.3
#: `entered`: along >= exit_dist - this, or within EXIT_ARRIVE_M of the exit goal.
EXIT_DONE_SLACK_M = 0.2
EXIT_ARRIVE_M = 0.2
#: Re-plan when the live door moved this much from the one the plan used.
DOOR_REPLAN_M = 0.1
DOOR_REPLAN_DEG = 5.0
#: Footprint margin [m] for the forward path.
PATH_MARGIN_M = 0.05
#: Infeasible: the shortest path is longer than this x the straight distance.
MAX_LENGTH_RATIO = 3.0
#: Footprint sampling step along a path [m].
PATH_STEP_M = 0.05

#: Phase names, as they appear in the move's record and the log.
PHASE_APPROACH, PHASE_ENTER, PHASE_EXIT = 'APPROACH', 'ENTER', 'EXIT'

#: How long [s] the resolved door may be missing from the topic before the
#: move fails with door_lost.
LOST_GRACE_SEC = 2.0



def confirmed_doors(msg) -> dict:
    """{id: (centre_xy, yaw)} of the CONFIRMED doors in a DoorArray.

    The yaw is Door.normal's bearing: the crossing direction the detector
    fixed at the door's first confirmation, which is also the x axis of the
    map -> door_<id> transform it broadcasts.
    """
    return _doors(msg, strong_only=False)


def strong_doors(msg) -> dict:
    """{id: (centre_xy, yaw)} of the STRONG doors in a DoorArray.

    What a mission is allowed to drive at. Used for SELECTION ONLY: once a
    door is resolved it is followed through `confirmed_doors`.
    """
    return _doors(msg, strong_only=True)


def _doors(msg, strong_only: bool) -> dict:
    out = {}
    for door in msg.doors:
        if door.state != Door.STATE_CONFIRMED:
            continue
        # getattr: a bag recorded before Door.strong existed deserialises
        # without it, and treating those doors as strong is the right default.
        if strong_only and not bool(getattr(door, 'strong', True)):
            continue
        out[int(door.id)] = ((float(door.center.x), float(door.center.y)),
                             math.atan2(float(door.normal.y), float(door.normal.x)))
    return out


def door_widths(msg) -> dict:
    """{id: width [m]} of the CONFIRMED doors in a DoorArray."""
    return {int(d.id): float(d.width) for d in msg.doors
            if d.state == Door.STATE_CONFIRMED}


def lateral_tol(width_m, override_m=0.0, car_width_m=DEFAULT_CAR_WIDTH_M) -> float:
    """door_lat_tol_m: `override_m` if > 0, else
    clamp(0.3 * (door width - car width), 0.10, 0.25)."""
    if override_m and override_m > 0.0:
        return float(override_m)
    if width_m is None or width_m <= 0.0:
        return MPC_POSE_TOL_M
    raw = LAT_TOL_SLACK_FRAC * (float(width_m) - float(car_width_m))
    return max(LAT_TOL_MIN_M, min(LAT_TOL_MAX_M, raw))


def has_entered(along, lateral, heading_err_deg, along_tol, lat_tol, yaw_tol_deg):
    """
    Tell whether the car is in the doorway, on the line, pointing in.

    No upper bound on along.

    No longer what `entered` means (see the module docstring: through the
    door); kept for callers that ask the narrower question.
    """
    return (along >= -along_tol and abs(lateral) <= lat_tol
            and heading_err_deg <= yaw_tol_deg)


def angle_gap_deg(a, b) -> float:
    """|a - b| in degrees, wrapped, for two yaws in radians."""
    return abs(wrap_deg(a - b))


def goal_yaw_for(car_xy, goal_xy, axis_yaw, tol_deg) -> float:
    """The yaw to put in the goal: the bearing from the car to the goal point,
    clamped to axis_yaw +- tol_deg (angle-wrapped).

    The goal POSITION stays on the door axis; only its yaw is relaxed, so the
    MPC is never asked to line up tighter than the entry tolerance, and a car
    inside the tolerance is simply asked to point at the goal. Within
    NEAR_GOAL_M the bearing is noise, so the axis yaw is used.
    """
    if car_xy is None:
        return axis_yaw
    dx, dy = goal_xy[0] - car_xy[0], goal_xy[1] - car_xy[1]
    if math.hypot(dx, dy) < NEAR_GOAL_M:
        return axis_yaw
    tol = math.radians(tol_deg)
    off = math.atan2(math.sin(math.atan2(dy, dx) - axis_yaw),
                     math.cos(math.atan2(dy, dx) - axis_yaw))
    off = max(-tol, min(tol, off))
    return math.atan2(math.sin(axis_yaw + off), math.cos(axis_yaw + off))


def wrap_deg(radians_delta: float) -> float:
    """A yaw difference in radians as degrees in (-180, 180]."""
    return math.degrees(math.atan2(math.sin(radians_delta), math.cos(radians_delta)))


def side_of(point_xy, car_xy, car_yaw):
    """DOOR_LEFT or DOOR_RIGHT: which side of the car's heading `point_xy` is on.

    Positive relative bearing is LEFT (REP-103), zero or negative is RIGHT.
    """
    bearing = math.atan2(point_xy[1] - car_xy[1], point_xy[0] - car_xy[0]) - car_yaw
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))
    return DOOR_LEFT if bearing > 0.0 else DOOR_RIGHT


def select_door(doors: dict, want: str, car_xy, car_yaw=None):
    """The door `want` asks for, or None. See the module docstring."""
    if not doors:
        return None
    if want in (DOOR_LEFT, DOOR_RIGHT):
        if car_xy is None or car_yaw is None:
            return None
        side = [i for i in doors if side_of(doors[i][0], car_xy, car_yaw) == want]
        if not side:
            return None
        return min(side, key=lambda i: math.hypot(doors[i][0][0] - car_xy[0],
                                                   doors[i][0][1] - car_xy[1]))
    if want != DOOR_NEAREST:
        try:
            door_id = int(want.split('_', 1)[1])
        except (IndexError, ValueError):
            return None
        return door_id if door_id in doors else None
    if car_xy is None:
        return None
    return min(doors, key=lambda i: math.hypot(doors[i][0][0] - car_xy[0],
                                               doors[i][0][1] - car_xy[1]))


def door_frame_xy(point_xy, centre_xy, yaw):
    """`point_xy` expressed in the door frame: x along the crossing direction.

    x is "how far through the opening", negative on the side the car came
    from; y is the sideways offset from the opening's centre line.
    """
    dx, dy = point_xy[0] - centre_xy[0], point_xy[1] - centre_xy[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * dx + s * dy, -s * dx + c * dy)


def pre_point(centre_xy, yaw, approach_dist_m):
    """A point approach_dist_m back along the door's own x axis."""
    return (centre_xy[0] - math.cos(yaw) * approach_dist_m,
            centre_xy[1] - math.sin(yaw) * approach_dist_m)


def entry_pose(centre_xy, yaw, entry_dist_m):
    """Return the APPROACH target: (x, y, yaw) entry_dist_m back on the axis."""
    x, y = pre_point(centre_xy, yaw, entry_dist_m)
    return (x, y, yaw)


def exit_pose(centre_xy, yaw, exit_dist_m):
    """Return the one goal sent after the centre: exit_dist_m beyond it on the axis."""
    x, y = pre_point(centre_xy, yaw, -exit_dist_m)
    return (x, y, yaw)


def switch_to_enter(car_xy, car_yaw, entry, centre_xy, axis_yaw, entry_dist_m,
                    lat_tol, yaw_tol_deg):
    """
    Decide APPROACH -> ENTER: return '' (stay) or which rule fired, 'a' or 'b'.

    (a) within ENTRY_SWITCH_M of the entry pose; (b) at or past its along,
    within LAT_SWITCH_FACTOR * lat_tol of the axis and within yaw_tol of it.
    """
    if math.hypot(car_xy[0] - entry[0], car_xy[1] - entry[1]) <= ENTRY_SWITCH_M:
        return 'a'
    along, lateral = door_frame_xy(car_xy, centre_xy, axis_yaw)
    if (along >= -entry_dist_m and abs(lateral) <= LAT_SWITCH_FACTOR * lat_tol
            and angle_gap_deg(axis_yaw, car_yaw) <= yaw_tol_deg):
        return 'b'
    return ''


class Footprint:
    """The car body in base_link (rear axle): front/rear bumper x, half-width [m]."""

    def __init__(self, front_m=DEFAULT_FRONT_M, rear_m=DEFAULT_REAR_M,
                 half_width_m=DEFAULT_CAR_WIDTH_M / 2.0):
        self.front_m = float(front_m)
        self.rear_m = float(rear_m)
        self.half_width_m = float(half_width_m)


class ForwardPlan:
    """A forward Dubins plan to the entry pose and its verdict."""

    def __init__(self, path, check, straight_m, reason):
        self.path = path
        self.check = check              # dubins.FreeCheck or None
        self.straight_m = straight_m
        self.reason = reason            # '' = feasible

    @property
    def feasible(self):
        return not self.reason


def plan_forward(q0, entry, rho, grid, fp: Footprint):
    """Shortest forward Dubins path q0 -> entry, checked; see the module docstring."""
    straight = math.hypot(entry[0] - q0[0], entry[1] - q0[1])
    path = dubins.shortest_path(q0, entry, rho)
    if path is None:
        return ForwardPlan(None, None, straight, 'no Dubins word')
    check = dubins.path_is_free(path, grid, fp.half_width_m, PATH_MARGIN_M,
                                step=PATH_STEP_M, front_m=fp.front_m, rear_m=fp.rear_m)
    if not check.free:
        return ForwardPlan(path, check, straight,
                           f'{path.word} blocked at s={check.first_blocked_s:.2f} m '
                           f'of {path.length:.2f} m')
    if path.length > MAX_LENGTH_RATIO * max(straight, 1e-6):
        return ForwardPlan(path, check, straight,
                           f'{path.word} {path.length:.2f} m > {MAX_LENGTH_RATIO:.0f} x '
                           f'straight {straight:.2f} m')
    return ForwardPlan(path, check, straight, '')


class EnterDoor(py_trees.behaviour.Behaviour):
    """Run the current enter_door move; see the module docstring."""

    def __init__(self, name='EnterDoor',
                 doors_topic='/perception/doors_semantic',
                 goal_pose_topic='/mpc/goal_pose',
                 hold_topic='/mpc/hold',
                 grid_topic='/slam/map',
                 map_frame='map',
                 clock=time.monotonic):
        super().__init__(name=name)
        self._doors_topic = doors_topic
        self._goal_pose_topic = goal_pose_topic
        self._hold_topic = hold_topic
        self._grid_topic = grid_topic
        self.map_frame = map_frame
        self._clock = clock
        self.node = None
        self.goal_pose_pub = None
        self.hold_pub = None
        self.doors = {}
        self.widths = {}
        #: door_lat_tol_m (0 = auto, see lateral_tol) and door_entry_yaw_tol_deg.
        self.lat_tol_override_m = 0.0
        self.entry_yaw_tol_deg = DEFAULT_ENTRY_YAW_TOL_DEG
        #: Minimum turning radius [m]; None = unknown (FALLBACK_R_MIN_M is used).
        self.r_min_m = None
        self.car_width_m = DEFAULT_CAR_WIDTH_M
        self.footprint = Footprint()
        # The Dubins approach (ROS params, read in setup()).
        self.radius_factor = DEFAULT_DUBINS_RADIUS_FACTOR
        self.entry_dist_m = DEFAULT_ENTRY_DIST_M
        self.lookahead_m = max(LOOKAHEAD_MIN_M, LOOKAHEAD_R_MIN_FACTOR * FALLBACK_R_MIN_M)
        self.replan_dev_m = DEFAULT_REPLAN_DEV_M
        self.replan_max_hz = DEFAULT_REPLAN_MAX_HZ
        self.republish_dist_m = DEFAULT_GOAL_REPUBLISH_DIST_M
        self.republish_yaw_deg = DEFAULT_GOAL_REPUBLISH_YAW_DEG
        self.exit_dist_m = DEFAULT_EXIT_DIST_M
        self.occupied_threshold = 65
        # World inputs.
        self.grid = None
        self.strong = {}
        self.doors_received_sec = None
        self._grid_warned = False
        self._active_move_key = None
        self._reset_move_state()
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=MISSION_KEY, access=py_trees.common.Access.WRITE)
        for key in (GLOBAL_XY_KEY, GLOBAL_YAW_KEY, GLOBAL_TURN_ACCUM_KEY):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    # ------------------------------------------------------------ setup

    @property
    def rho(self):
        return self.radius_factor * (self.r_min_m if self.r_min_m else FALLBACK_R_MIN_M)

    def setup(self, **kwargs):
        """Create the subscriptions and the publishers."""
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("EnterDoor.setup() didn't find 'node' in kwargs") from e
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        self._declare_params()
        self.node.create_subscription(DoorArray, self._doors_topic, self._doors_cb, 10)
        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.node.create_subscription(OccupancyGrid, self._grid_topic, self._grid_cb, map_qos)
        self.goal_pose_pub = self.node.create_publisher(
            PoseStamped, self._goal_pose_topic, 10)
        self.hold_pub = self.node.create_publisher(Bool, self._hold_topic, 10)

    def _declare_params(self):
        """Tolerances, the Dubins approach params, R_min and the car geometry."""
        def declare(name, default):
            if not self.node.has_parameter(name):
                self.node.declare_parameter(name, default)
            return self.node.get_parameter(name).value
        self.lat_tol_override_m = float(declare('door_lat_tol_m', 0.0))
        self.entry_yaw_tol_deg = float(declare(
            'door_entry_yaw_tol_deg', DEFAULT_ENTRY_YAW_TOL_DEG))
        try:
            from f1tenth_params.param_defaults import get_value
        except Exception:  # noqa: BLE001 - no params package: keep the fallbacks
            get_value = None

        def pv(key, default):
            if get_value is None:
                return default
            try:
                return float(get_value(key))
            except Exception:  # noqa: BLE001 - a missing key keeps the fallback
                return default
        try:
            bound = max(abs(pv('mpc_steering_angle_min_rad', math.nan)),
                        abs(pv('mpc_steering_angle_max_rad', math.nan)))
            r_min = pv('mpc_wheelbase_m', math.nan) / math.tan(bound)
            self.r_min_m = r_min if math.isfinite(r_min) and r_min > 0.0 else None
        except (ValueError, ZeroDivisionError):
            self.r_min_m = None
        half = pv('swept_clearance_body_half_width_m', DEFAULT_CAR_WIDTH_M / 2.0)
        self.car_width_m = 2.0 * half
        try:
            from f1tenth_description.vehicle_frames import base_front_x
            front = float(base_front_x())
        except Exception:  # noqa: BLE001 - keep the URDF fallback
            front = DEFAULT_FRONT_M
        self.footprint = Footprint(front, pv('car_rear_offset_m', DEFAULT_REAR_M), half)
        self.occupied_threshold = int(pv('freespace_doors_occupied_threshold', 65))
        r_min = self.r_min_m if self.r_min_m else FALLBACK_R_MIN_M
        self.radius_factor = float(declare('door_dubins_radius_factor',
                                           DEFAULT_DUBINS_RADIUS_FACTOR))
        self.entry_dist_m = float(declare('door_entry_dist_m', DEFAULT_ENTRY_DIST_M))
        lookahead = float(declare('door_lookahead_m', -1.0))
        self.lookahead_m = (lookahead if lookahead > 0.0
                            else max(LOOKAHEAD_MIN_M, LOOKAHEAD_R_MIN_FACTOR * r_min))
        self.replan_dev_m = float(declare('door_replan_dev_m', DEFAULT_REPLAN_DEV_M))
        self.replan_max_hz = float(declare('door_replan_max_hz', DEFAULT_REPLAN_MAX_HZ))
        self.republish_dist_m = float(declare('door_goal_republish_dist_m',
                                              DEFAULT_GOAL_REPUBLISH_DIST_M))
        self.republish_yaw_deg = float(declare('door_goal_republish_yaw_deg',
                                               DEFAULT_GOAL_REPUBLISH_YAW_DEG))
        self.exit_dist_m = float(declare('door_exit_dist_m', DEFAULT_EXIT_DIST_M))
        self.node.get_logger().info(self.settings_line())

    def settings_line(self):
        """Return the one setup log line: every value the move runs on."""
        fp = self.footprint
        return (
            '[enter_door] R_min '
            + (f'{self.r_min_m:.3f} m' if self.r_min_m else
               f'unknown (using {FALLBACK_R_MIN_M:.2f})')
            + f', rho {self.rho:.3f} m (x{self.radius_factor:.2f}), entry '
            f'{self.entry_dist_m:.2f} m, lookahead {self.lookahead_m:.2f} m, replan dev '
            f'{self.replan_dev_m:.2f} m / max {self.replan_max_hz:.1f} Hz, republish '
            f'{self.republish_dist_m:.2f} m / {self.republish_yaw_deg:.0f} deg, exit '
            f'{self.exit_dist_m:.2f} m, footprint +{fp.front_m:.3f}/-{fp.rear_m:.3f} '
            f'x +-{fp.half_width_m:.3f} m (margin {PATH_MARGIN_M:.2f}), yaw tol '
            f'{self.entry_yaw_tol_deg:.1f} deg, grid '
            f'{self._grid_topic} occ>={self.occupied_threshold}')

    # ------------------------------------------------------------ inputs

    def _doors_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.node.get_logger().warn(
                f'[enter_door] doors in frame {msg.header.frame_id!r}, expected '
                f'{self.map_frame!r}; ignored', throttle_duration_sec=2.0)
            return
        self.doors = confirmed_doors(msg)
        self.strong = strong_doors(msg)
        self.widths = door_widths(msg)
        self.doors_received_sec = self._clock()

    def _grid_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            return
        try:
            self.grid = dubins.OccGrid.from_msg(msg, self.occupied_threshold)
        except Exception as exc:  # noqa: BLE001 - a bad grid is no grid
            self.node.get_logger().warn(f'[enter_door] grid unusable: {exc}',
                                        throttle_duration_sec=5.0)

    def _bb(self, key):
        """Return a blackboard value, or None while nothing has written it yet."""
        try:
            return getattr(self.blackboard, key)
        except (KeyError, AttributeError):
            return None

    # ------------------------------------------------------------ state

    def _reset_move_state(self):
        self._move_start_sec = None
        self._last_seen_sec = None
        #: (centre_xy, axis_yaw) captured when ENTER starts; None before.
        self._frozen = None
        self._enter_goal_sent = False
        self._exit_goal = None
        # The plan.
        self._plan = None                 # ForwardPlan (feasible) being followed
        self._plan_sec = None
        self._plan_door = None            # (centre_xy, yaw) the plan was made for
        self._plan_s = 0.0
        self._last_goal = None            # (x, y, yaw) last published
        self._last_infeasible = ''

    def _clear(self):
        self._active_move_key = None
        self._reset_move_state()

    # ------------------------------------------------------------ the tick

    def update(self):
        """Resolve the door, steer at it, and measure the arrival."""
        state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)
        move = state.current_move
        spec = getattr(move, 'enter_door', None) if move is not None else None
        if spec is None:
            if self._active_move_key is not None:
                self._clear()
                state.door_record = None
            return py_trees.common.Status.SUCCESS

        now = self._clock()
        key = (state.config.mission_id, state.run_generation, move.id)
        if key != self._active_move_key:
            self._clear()
            self._active_move_key = key
            self._move_start_sec = now
            state.door_record = DoorApproachRecord(door=spec.door)
            self.node.get_logger().info(
                f"[enter_door] move '{move.id}': looking for {spec.door!r}, "
                f'along_tol {spec.along_tol:.2f} m, lateral_tol '
                f'{spec.lateral_tol:.2f} m, yaw_tol {spec.yaw_tol_deg:.1f} deg')

        record = state.door_record
        car_xy = self._bb(GLOBAL_XY_KEY)
        car_yaw = self._bb(GLOBAL_YAW_KEY)

        if record.door_id is None:
            door_id = select_door(self.strong, spec.door, car_xy, car_yaw)
            if door_id is None:
                if now - self._move_start_sec >= spec.acquire_timeout_sec:
                    matched = select_door(self.doors, spec.door, car_xy, car_yaw)
                    outcome = 'no_strong_door' if matched is not None else 'door_not_found'
                    return self._fail(state, move, record, outcome, now)
                self.node.get_logger().warn(
                    f"[enter_door] move '{move.id}': no STRONG {spec.door!r} door "
                    f'yet ({len(self.doors)} confirmed, {len(self.strong)} strong); '
                    'not moving', throttle_duration_sec=2.0)
                return py_trees.common.Status.SUCCESS
            record.door_id = door_id
            record.frame_id = f'door_{door_id}'
            self._last_seen_sec = now
            self.node.get_logger().info(
                f"[enter_door] move '{move.id}': {spec.door!r} resolved to "
                f'{record.frame_id!r}')

        target = self.doors.get(record.door_id)
        if self._frozen is not None:
            if target is None:
                self.node.get_logger().warn(
                    f'[enter_door] {record.frame_id} lost from {self._doors_topic} '
                    f'during {record.phase}; holding the frozen door pose',
                    throttle_duration_sec=2.0)
            door_xy, tyaw = self._frozen
        else:
            if target is None:
                if (self._last_seen_sec is not None
                        and now - self._last_seen_sec > LOST_GRACE_SEC):
                    return self._fail(state, move, record, 'door_lost', now)
                return py_trees.common.Status.SUCCESS
            self._last_seen_sec = now
            (dx_, dy_), tyaw = target
            door_xy = (dx_, dy_)
        record.target_yaw = tyaw
        # A mission's own approach_dist_m (> 0) still sets the entry distance.
        entry_dist = getattr(spec, 'approach_dist_m', None) or self.entry_dist_m
        entry = entry_pose(door_xy, tyaw, entry_dist)
        record.pre_point_xy = (entry[0], entry[1])

        lat_tol = lateral_tol(self.widths.get(record.door_id), self.lat_tol_override_m,
                              self.car_width_m)
        yaw_tol = self.entry_yaw_tol_deg
        record.lat_tol_m, record.yaw_tol_deg = lat_tol, yaw_tol
        if car_xy is None or car_yaw is None:
            record.entered = False
            return py_trees.common.Status.SUCCESS
        car = (float(car_xy[0]), float(car_xy[1]), float(car_yaw))
        along, lateral = door_frame_xy(car_xy, door_xy, tyaw)
        heading_err = angle_gap_deg(tyaw, car_yaw)
        record.along_error_m, record.lateral_error_m = along, lateral
        record.yaw_error_deg = heading_err
        ctx = {'move': move, 'record': record, 'state': state, 'now': now, 'car': car,
               'door_xy': door_xy, 'tyaw': tyaw, 'entry': entry, 'entry_dist': entry_dist,
               'lat_tol': lat_tol, 'yaw_tol': yaw_tol, 'along': along, 'lateral': lateral,
               'heading_err': heading_err}

        if record.phase is None:
            rule = switch_to_enter(car_xy, car_yaw, entry, door_xy, tyaw, entry_dist,
                                   lat_tol, yaw_tol)
            if along > APPROACH_SKIP_X_M or rule:
                record.phase = PHASE_ENTER
                self.node.get_logger().info(
                    f"[enter_door] move '{move.id}': {record.frame_id} starts in "
                    f'{PHASE_ENTER} (along {along:+.2f} m, lateral {lateral:+.2f} m, '
                    f'heading err {heading_err:.1f} deg'
                    + (f', rule ({rule})' if rule else ', already in the doorway') + ')')
            else:
                record.phase = PHASE_APPROACH
                self.node.get_logger().info(
                    f"[enter_door] move '{move.id}': {record.frame_id} phase "
                    f'{PHASE_APPROACH} (entry pose {entry_dist:.2f} m back on the axis)')

        status = None
        if record.phase == PHASE_APPROACH:
            rule = switch_to_enter(car_xy, car_yaw, entry, door_xy, tyaw, entry_dist,
                                   lat_tol, yaw_tol)
            if rule:
                record.phase = PHASE_ENTER
                self.node.get_logger().info(
                    f"[enter_door] SWITCH move '{move.id}': {record.frame_id} "
                    f'{PHASE_APPROACH} -> {PHASE_ENTER} by rule ({rule}): along '
                    f'{along:+.2f} m, lateral {lateral:+.2f} m, heading err '
                    f'{heading_err:.1f} deg, '
                    f'{math.hypot(car[0] - entry[0], car[1] - entry[1]):.2f} m from the '
                    'entry pose')
            else:
                status = self._tick_approach(ctx)
        if status is not None:
            return status

        if record.phase == PHASE_ENTER:
            if self._frozen is None:
                self._frozen = (door_xy, tyaw)
                self._plan = None
                self.node.get_logger().info(
                    f'[enter_door] {record.frame_id} pose frozen for ENTER: centre '
                    f'({door_xy[0]:.2f}, {door_xy[1]:.2f}), axis '
                    f'{math.degrees(tyaw):.1f} deg')
            near = math.hypot(car[0] - door_xy[0], car[1] - door_xy[1]) <= EXIT_SWITCH_M
            if near or along >= 0.0:
                self._send_exit(move, record, door_xy, tyaw, along, lateral)
            else:
                record.target_xy = door_xy
                goal_yaw = goal_yaw_for(car_xy, door_xy, tyaw, yaw_tol)
                record.goal_yaw = goal_yaw
                if not self._enter_goal_sent:
                    self._publish_goal(move, door_xy[0], door_xy[1], goal_yaw)
                    self._enter_goal_sent = True
        if record.phase == PHASE_EXIT:
            ex = self._exit_goal
            record.target_xy = (ex[0], ex[1])
            record.entered = (along >= self.exit_dist_m - EXIT_DONE_SLACK_M
                              or math.hypot(car[0] - ex[0], car[1] - ex[1]) <= EXIT_ARRIVE_M)
        else:
            record.entered = False
        return py_trees.common.Status.SUCCESS

    # ------------------------------------------------------------ APPROACH

    def _tick_approach(self, c):
        """Follow (or make) the Dubins plan; None to fall through, else a status."""
        move, record, now, car = c['move'], c['record'], c['now'], c['car']
        door_xy, tyaw, entry = c['door_xy'], c['tyaw'], c['entry']
        why = ''
        force = self._plan is None
        if force:
            why = 'initial'
        else:
            (pdx, pdy), pyaw = self._plan_door
            moved = math.hypot(door_xy[0] - pdx, door_xy[1] - pdy)
            turned = angle_gap_deg(tyaw, pyaw)
            if moved > DOOR_REPLAN_M or turned > DOOR_REPLAN_DEG:
                why = f'door moved {moved:.2f} m / {turned:.1f} deg'
            else:
                s, e = self._plan.path.project(car[0], car[1], s_min=max(
                    0.0, self._plan_s - 0.3), s_max=self._plan_s + 2.0)
                self._plan_s = max(self._plan_s, s)
                if abs(e) > self.replan_dev_m:
                    why = f'deviation {e:+.2f} m > {self.replan_dev_m:.2f} m'
        if why and (force or self.replan_max_hz <= 0.0
                    or now - self._plan_sec >= 1.0 / self.replan_max_hz):
            if why != 'initial':
                self.node.get_logger().info(
                    f"[enter_door] REPLAN move '{move.id}': {why}")
            plan = plan_forward(car, entry, self.rho, self._grid_or_warn(), self.footprint)
            self._log_plan(move, plan, car)
            if not plan.feasible:
                self._last_infeasible = plan.reason
                self.node.get_logger().warn(
                    f"[enter_door] INFEASIBLE move '{move.id}': forward entry -- "
                    f'{plan.reason} (followed anyway: no reverse)',
                    throttle_duration_sec=2.0)
                if plan.path is None:
                    return py_trees.common.Status.SUCCESS
            self._adopt(plan, now, door_xy, tyaw)
        plan = self._plan
        if plan is None:            # rate-limited with nothing to follow yet
            return None
        path = plan.path
        if self._plan_s + self.lookahead_m >= path.length:
            goal = entry
        else:
            goal = path.pose_at(self._plan_s + self.lookahead_m)
        record.target_xy = (goal[0], goal[1])
        record.goal_yaw = goal[2]
        last = self._last_goal
        if (last is None
                or math.hypot(goal[0] - last[0], goal[1] - last[1]) > self.republish_dist_m
                or angle_gap_deg(goal[2], last[2]) > self.republish_yaw_deg):
            self._publish_goal(move, goal[0], goal[1], goal[2])
            self._last_goal = goal
        record.entered = False
        return py_trees.common.Status.SUCCESS

    def _adopt(self, plan, now, door_xy, tyaw):
        self._plan = plan
        self._plan_sec = now
        self._plan_door = (door_xy, tyaw)
        self._plan_s = 0.0

    def _grid_or_warn(self):
        if self.grid is None and not self._grid_warned:
            self._grid_warned = True
            self.node.get_logger().warn(
                f'[enter_door] no grid on {self._grid_topic} yet: paths are checked '
                'against nothing (every cell free)')
        return self.grid

    def _log_plan(self, move, plan, car):
        p, chk = plan.path, plan.check
        self.node.get_logger().info(
            f"[enter_door] PLAN move '{move.id}': from ({car[0]:.2f}, {car[1]:.2f}, "
            f'{math.degrees(car[2]):.0f} deg) '
            + (f'{p.word} t/p/q {p.t:.2f}/{p.p:.2f}/{p.q:.2f} L {p.length:.2f} m '
               f'(straight {plan.straight_m:.2f} m), free '
               f'{chk.free if chk else "-"}, unknown cells '
               f'{chk.unknown_cells if chk else "-"}' if p is not None
               else 'no Dubins word')
            + (f' -> INFEASIBLE: {plan.reason}' if plan.reason else ''))

    # ------------------------------------------------------------ ENTER / EXIT

    def _send_exit(self, move, record, door_xy, tyaw, along, lateral):
        ex = exit_pose(door_xy, tyaw, self.exit_dist_m)
        self._exit_goal = ex
        record.phase = PHASE_EXIT
        record.goal_yaw = ex[2]
        self._publish_goal(move, ex[0], ex[1], ex[2])
        self.node.get_logger().info(
            f"[enter_door] EXIT move '{move.id}': {record.frame_id} at along "
            f'{along:+.2f} m, lateral {lateral:+.2f} m: one exit goal '
            f'{self.exit_dist_m:.2f} m beyond the centre ({ex[0]:.2f}, {ex[1]:.2f}); '
            'no further goals')

    # ------------------------------------------------------------ output

    def _publish_goal(self, move, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pose_pub.publish(msg)
        self.node.get_logger().info(
            f"[enter_door] move '{move.id}': published goal_pose=(x={x:.2f}, "
            f'y={y:.2f}, yaw={math.degrees(yaw):.1f} deg) to /mpc/goal_pose.')

    def _fail(self, state, move, record, outcome, now):
        """End the move with `outcome`: hold the car and abort the mission."""
        record.outcome = outcome
        why = {'door_not_found': (
                   f'no confirmed door matching {record.door!r} within '
                   f'{move.enter_door.acquire_timeout_sec:.1f} s on '
                   f'{self._doors_topic} '
                   + ('(no message has arrived at all -- is a door detector '
                      'running? door_source picks which one, and each has its '
                      'own enable flag)' if self.doors_received_sec is None
                      else f'({len(self.doors)} confirmed door(s) seen, none matched)')),
               'no_strong_door': (
                   f'a confirmed door matching {record.door!r} is on '
                   f'{self._doors_topic} but none is STRONG within '
                   f'{move.enter_door.acquire_timeout_sec:.1f} s '
                   f'({len(self.doors)} confirmed, {len(self.strong)} strong) '
                   '-- a door has to be seen enough times from far enough '
                   'apart, with no flags, before a mission may drive at it'),
               'door_lost': (
                   f'{record.frame_id} disappeared from {self._doors_topic} for more '
                   f'than {LOST_GRACE_SEC:.1f} s')}[outcome]
        self.hold_pub.publish(Bool(data=True))
        state.last_stop_reason = f'enter_door:{outcome}'
        self.node.get_logger().error(
            f"[mission] Move '{move.id}' failed: {outcome} -- {why}; aborting mission.")
        record_move_outcome(
            state, self.node.get_logger(), move, state.last_stop_reason, now,
            end_global_xy=self._bb(GLOBAL_XY_KEY),
            end_global_yaw=self._bb(GLOBAL_YAW_KEY),
            global_turn_accum_deg=self._bb(GLOBAL_TURN_ACCUM_KEY))
        write_mission_summary(
            state.config.mission_id, state.move_outcomes, 'ABORTED',
            state.mission_start_wall_time, self.node.get_logger())
        state.abort()
        self._clear()
        return py_trees.common.Status.FAILURE
