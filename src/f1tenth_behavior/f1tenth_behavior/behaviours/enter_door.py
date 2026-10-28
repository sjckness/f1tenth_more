"""py_trees Action: run the current enter_door move -- doors in, a pose out.

  in   /perception/doors_semantic  f1tenth_messages/DoorArray, MAP frame, the
                                   CONFIRMED doors semantic_layer_node holds
                                   (f1tenth_costmap/semantic_doors.py)
       GLOBAL_XY_KEY / GLOBAL_YAW_KEY
                                   the vehicle in the map frame
                                   (CheckStopCondition's global EKF
                                   subscription -- the same pose the doors
                                   were built in, which is the whole reason
                                   this move can compare the two at all)
  out  /mpc/goal_pose              PoseStamped, map, ONCE per resolved door
       /mpc/hold                   on a failure outcome, which aborts the mission

THE DOOR IS RESOLVED ONCE. "nearest" picks the confirmed door nearest the car
at the moment the move starts and then holds that door's id for the rest of
the move; "door_<id>" names one outright. Re-picking every tick would let the
car swap targets halfway across a room the instant another door came closer,
and a target that changes under a running approach is the failure
go_to_object's own follow gate exists to prevent.

ARRIVAL IS MEASURED HERE, not by mpc_corr. /mpc/goal_pose is position-only on
arrival -- mpc_corr stores the yaw and does not consume it (see
publish_move_goal.py) -- and a doorway is the one goal whose heading is the
point. So this behaviour measures both errors every tick against the door
frame's origin and x axis, writes them into the move's DoorApproachRecord,
and sets `entered` when both are inside the mission's tolerances;
condition_eval's door_entered reads exactly that flag.

THE GOAL IS PUBLISHED ONCE per resolved door, not every tick, for the reason
PublishMoveGoal's docstring gives: mpc_corr resets its goal progress tracking
on every /mpc/goal_pose message it receives, so a goal republished each tick
makes the move never appear to progress. A door does not move, so once is
right; if the detector's smoothing shifts a confirmed door by more than
REPUBLISH_EPS_M the goal goes out again, which is the only case where "once"
would otherwise leave the car driving at a stale point.

FAILURE IS CLEAN AND IT IS FINAL. No confirmed door to resolve within the
spec's acquire_timeout_sec -> `door_not_found`; the resolved door gone from
the topic for LOST_GRACE_SEC -> `door_lost`. Either one ends the move, holds
the car and aborts the mission with the reason on
MissionRuntimeState.last_stop_reason -- the same shape GoToObject's
target_not_found has. It never falls back to another door: a mission that
said door_7 and got door_9 has not been carried out.

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

from f1tenth_behavior.mission.mission_config import DOOR_NEAREST
from f1tenth_behavior.mission.move_scoring import record_move_outcome, write_mission_summary
from f1tenth_behavior.mission.runtime import (
    DoorApproachRecord,
    GLOBAL_TURN_ACCUM_KEY,
    GLOBAL_XY_KEY,
    GLOBAL_YAW_KEY,
    MISSION_KEY,
    MissionRuntimeState,
)

#: A confirmed door whose smoothed centre moves further than this [m] gets a
#: fresh /mpc/goal_pose. Well above the detector's own jitter (the EMA on a
#: confirmed door settles inside a centimetre) and well under anything that
#: would change which doorway the car is driving at.
REPUBLISH_EPS_M = 0.05

#: How long [s] the resolved door may be missing from the topic before the
#: move fails with door_lost. A confirmed door is kept in the map by the
#: detector and does not normally disappear at all; this covers the detector
#: node restarting (which restarts its ids) mid-move.
LOST_GRACE_SEC = 2.0


def confirmed_doors(msg) -> dict:
    """{id: (centre_xy, yaw)} of the CONFIRMED doors in a DoorArray.

    The yaw is Door.normal's bearing: the crossing direction the detector
    fixed at the door's first confirmation, which is also the x axis of the
    map -> door_<id> transform it broadcasts. Taking it from the message
    rather than from TF keeps this behaviour on ONE input -- a TF lookup
    would be a second clock and a second failure mode for the same number.
    """
    out = {}
    for door in msg.doors:
        if door.state != Door.STATE_CONFIRMED:
            continue
        out[int(door.id)] = ((float(door.center.x), float(door.center.y)),
                             math.atan2(float(door.normal.y), float(door.normal.x)))
    return out


def wrap_deg(radians_delta: float) -> float:
    """A yaw difference in radians as degrees in (-180, 180]."""
    return math.degrees(math.atan2(math.sin(radians_delta), math.cos(radians_delta)))


def select_door(doors: dict, want: str, car_xy):
    """The door `want` asks for, or None. See the module docstring."""
    if not doors:
        return None
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


class EnterDoor(py_trees.behaviour.Behaviour):
    """Run the current enter_door move; see the module docstring."""

    def __init__(self, name='EnterDoor',
                 doors_topic='/perception/doors_semantic',
                 goal_pose_topic='/mpc/goal_pose',
                 hold_topic='/mpc/hold',
                 map_frame='map',
                 clock=time.monotonic):
        super().__init__(name=name)
        self._doors_topic = doors_topic
        self._goal_pose_topic = goal_pose_topic
        self._hold_topic = hold_topic
        self.map_frame = map_frame
        self._clock = clock
        self.node = None
        self.goal_pose_pub = None
        self.hold_pub = None
        self.doors = {}
        self.doors_received_sec = None
        self._active_move_key = None
        self._move_start_sec = None
        self._published_xy = None
        self._last_seen_sec = None
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=MISSION_KEY, access=py_trees.common.Access.WRITE)
        for key in (GLOBAL_XY_KEY, GLOBAL_YAW_KEY, GLOBAL_TURN_ACCUM_KEY):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """Create the doors subscription and the two publishers."""
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("EnterDoor.setup() didn't find 'node' in kwargs") from e
        self.node.create_subscription(DoorArray, self._doors_topic, self._doors_cb, 10)
        self.goal_pose_pub = self.node.create_publisher(
            PoseStamped, self._goal_pose_topic, 10)
        self.hold_pub = self.node.create_publisher(Bool, self._hold_topic, 10)

    def _doors_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.node.get_logger().warn(
                f'[enter_door] doors in frame {msg.header.frame_id!r}, expected '
                f'{self.map_frame!r}; ignored', throttle_duration_sec=2.0)
            return
        self.doors = confirmed_doors(msg)
        self.doors_received_sec = self._clock()

    def _bb(self, key):
        """Return a blackboard value, or None while nothing has written it yet."""
        try:
            return getattr(self.blackboard, key)
        except (KeyError, AttributeError):
            return None

    def _clear(self):
        self._active_move_key = None
        self._move_start_sec = None
        self._published_xy = None
        self._last_seen_sec = None

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
                f'xy_tol {spec.xy_tol:.2f} m, yaw_tol {spec.yaw_tol_deg:.1f} deg')

        record = state.door_record
        car_xy = self._bb(GLOBAL_XY_KEY)
        car_yaw = self._bb(GLOBAL_YAW_KEY)

        if record.door_id is None:
            door_id = select_door(self.doors, spec.door, car_xy)
            if door_id is None:
                if now - self._move_start_sec >= spec.acquire_timeout_sec:
                    return self._fail(state, move, record, 'door_not_found', now)
                return py_trees.common.Status.SUCCESS
            record.door_id = door_id
            record.frame_id = f'door_{door_id}'
            self._last_seen_sec = now
            self.node.get_logger().info(
                f"[enter_door] move '{move.id}': {spec.door!r} resolved to "
                f'{record.frame_id!r}')

        target = self.doors.get(record.door_id)
        if target is None:
            if self._last_seen_sec is not None and now - self._last_seen_sec > LOST_GRACE_SEC:
                return self._fail(state, move, record, 'door_lost', now)
            return py_trees.common.Status.SUCCESS
        self._last_seen_sec = now

        (tx, ty), tyaw = target
        record.target_xy = (tx, ty)
        record.target_yaw = tyaw
        if (self._published_xy is None
                or math.hypot(tx - self._published_xy[0],
                              ty - self._published_xy[1]) > REPUBLISH_EPS_M):
            self._publish_goal(move, tx, ty, tyaw)
            self._published_xy = (tx, ty)

        if car_xy is None or car_yaw is None:
            record.entered = False
            return py_trees.common.Status.SUCCESS
        record.xy_error_m = math.hypot(tx - car_xy[0], ty - car_xy[1])
        record.yaw_error_deg = abs(wrap_deg(tyaw - car_yaw))
        record.entered = (record.xy_error_m <= spec.xy_tol
                          and record.yaw_error_deg <= spec.yaw_tol_deg)
        return py_trees.common.Status.SUCCESS

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
                   + ('(no message has arrived at all -- is semantic_doors_enabled '
                      'on, and door_source "semantic"?)' if self.doors_received_sec is None
                      else f'({len(self.doors)} confirmed door(s) seen, none matched)')),
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
