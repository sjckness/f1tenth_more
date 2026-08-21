"""py_trees Condition: has the current move's stop_condition (or its
timeout_sec) fired yet?

Owns the mission subtree's *only* /odom subscription -- per the task's own
"or better, subscribe to a shared pose topic to avoid duplicating odom-fusion
logic" -- there is no ready-made shared-pose topic in the stack today (the
map->odom TF chain isn't a single topic; /odom is the same thing mpc_corr
itself reads directly), so the simplest thing that satisfies "don't duplicate
odom-fusion logic" is: exactly one subscription, here, with the parsed (x, y)
shared onto the blackboard (CURRENT_XY_KEY) for HandleObjectAction's
resume_condition checks to reuse rather than subscribing again. Deliberately
simpler than mpc_corr's own hw/sim dual-source selection -- this only needs a
position for distance-traveled bookkeeping, not a full odom-source state
machine, so it takes /odom directly.

Also owns /mpc/min_obstacle_distance (published by mpc_corr itself), for
obstacle_distance_below -- shared the same way, via MIN_OBSTACLE_DISTANCE_KEY.

Also owns /costmap/front_clearance (published by f1tenth_costmap's
costmap_boundary_node -- nearest-occupied-cell extraction from slam_
toolbox's own /slam/map, added alongside this stop_condition type;
front_clearance did not exist as any kind of stop_condition, stub or real,
before that integration -- see mission_config.py's own comment on
STUB_STOP_CONDITION_TYPES), for the front_clearance stop_condition --
shared the same way as min_obstacle_distance, via FRONT_CLEARANCE_KEY.
Same "last received value, no freshness/timeout check" caveat applies as
it already does for min_obstacle_distance: if costmap_boundary_node stops
publishing entirely (crashed, not just "nothing occupied within range" --
that case is a finite, honest "clear at least this far" value, not
silence -- see that node's own module docstring), this blackboard value
goes stale silently rather than resetting. Not fixed here -- pre-existing
pattern (originally documented against wall_detector_node, the source this
retired -- see the dual-EKF + costmap-derived-MPC-boundaries pass), not
introduced by this addition or by that source swap.

Also owns /mpc/goal_reached (std_msgs/Bool, published by mpc_corr for BOTH its
distance-mode and pose-mode arrival -- same topic, mode-agnostic on the wire),
for the goal_reached stop_condition type. mpc_corr only ever publishes `True`
on arrival -- it never republishes `False` when a new goal supersedes the old
one -- so a bare "last received value" would stay latched True from a
previous move forever. _goal_reached_flag is reset to False any time
state.current_move's id changes (a new move started, whether via normal
advance or skip_to_move) and only set True by a message arriving *after* that
reset, so it means "mpc_corr has confirmed THIS move's own goal, not some
earlier one." Threaded into EvalContext.goal_reached for condition_eval.py to
consume -- see that module for why HandleObjectAction's resume_condition
usage does NOT get a live value here (stays stub-like, flagged there).

move_start_xy lazy-capture: MissionRuntimeState.load()/goto_move() both leave
move_start_xy as None on purpose (odom may not be available yet at the moment a
move starts). The first tick here where a position is known and
move_start_xy is still None for the current move, it gets captured then --
see the top of update().

Yaw tracking (schema_version 2.0's "turn" step / orientation_delta), added
alongside the above: /odom already carries orientation, not just position --
_odom_cb now also extracts yaw (the same atan2-based planar-yaw-from-
quaternion formula MPC_corr.py's own quaternion_to_yaw() uses, reimplemented
here rather than imported to avoid a cross-package dependency between
f1tenth_behavior and mpc_controller for two lines of math) and shares it via
CURRENT_YAW_KEY, the same "own dedicated blackboard key" pattern
MIN_OBSTACLE_DISTANCE_KEY/FRONT_CLEARANCE_KEY already use (not folded into
CURRENT_XY_KEY's existing 2-tuple, which HandleObjectAction also reads and
whose shape changing would be a more invasive edit than a new key).
move_start_yaw is lazily captured exactly like move_start_xy -- see
MissionRuntimeState.move_start_yaw's own comment.

Turn accumulation (orientation_delta's own real fix, added when live testing
found 180deg turns spinning instead of stopping): _odom_cb accumulates
_turn_accum_deg across EVERY odom message (not just once per BT tick, so a
turn between two ticks can't be missed) by unwrap-and-adding each small
per-message step, rather than re-deriving a single current-minus-start delta
the way move_start_yaw-based tracking did. That wrapped-delta approach is
mathematically bounded to (-180, 180] deg -- see condition_eval.py's own
turn_accum_deg comment for exactly why that made a 180deg target nearly
unreachable and anything above 180 unreachable at all. _turn_accum_deg/
_turn_accum_prev_yaw reset (to 0.0 / the current yaw) in update() whenever
move.id changes, same trigger as _goal_reached_flag's own reset just below --
harmless if an odom message sneaks in between the move actually changing and
this reset running (it would accumulate onto the stale value using the stale
baseline, which then simply gets overwritten).

on_timeout='stop' (new alongside 'abort'/'skip'): unlike those two, this
does not change mission.state or advance current_index at all -- it holds
the car (/mpc/hold True, hence the new hold_pub here) and leaves the mission
sitting on the same (now-timed-out) move indefinitely, logged once via
log_stub_once rather than every tick.
"""

import math
import time

import py_trees
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32

from f1tenth_behavior.mission.condition_eval import EvalContext, evaluate
from f1tenth_behavior.mission.mission_config import STUB_STOP_CONDITION_TYPES
from f1tenth_behavior.mission.detected_classes_bridge import DETECTED_CLASSES_KEY
from f1tenth_behavior.mission.runtime import MISSION_KEY, MissionRuntimeState

CURRENT_XY_KEY = 'mission_current_xy'
CURRENT_YAW_KEY = 'mission_current_yaw'
MIN_OBSTACLE_DISTANCE_KEY = 'mission_min_obstacle_distance'
FRONT_CLEARANCE_KEY = 'mission_front_clearance'


def _quaternion_to_yaw(q) -> float:
    """Planar yaw (radians) from a geometry_msgs/Quaternion -- standard
    atan2 formula, valid for the roll=pitch=0 planar case this stack always
    operates in. Same formula as MPC_corr.py's own quaternion_to_yaw()."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class CheckStopCondition(py_trees.behaviour.Behaviour):

    def __init__(self, name='CheckStopCondition', odom_topic='/odom',
                 min_obstacle_distance_topic='/mpc/min_obstacle_distance',
                 goal_reached_topic='/mpc/goal_reached',
                 front_clearance_topic='/costmap/front_clearance',
                 hold_topic='/mpc/hold'):
        super().__init__(name=name)
        self._odom_topic = odom_topic
        self._min_obstacle_distance_topic = min_obstacle_distance_topic
        self._goal_reached_topic = goal_reached_topic
        self._front_clearance_topic = front_clearance_topic
        self._hold_topic = hold_topic
        self.node = None
        self.hold_pub = None
        self.x = None
        self.y = None
        self.yaw = None
        self._goal_reached_flag = False
        self._goal_reached_tracked_move_id = None
        # orientation_delta's own accumulator -- see module docstring's "Turn
        # accumulation" paragraph. _turn_accum_prev_yaw is None until the
        # first odom message for the CURRENT move has been folded in (reset
        # alongside _turn_accum_deg on move change, in update() below).
        self._turn_accum_deg = 0.0
        self._turn_accum_prev_yaw = None

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=MISSION_KEY, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            key=DETECTED_CLASSES_KEY, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=CURRENT_XY_KEY, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=CURRENT_YAW_KEY, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            key=MIN_OBSTACLE_DISTANCE_KEY, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            key=FRONT_CLEARANCE_KEY, access=py_trees.common.Access.WRITE)
        setattr(self.blackboard, CURRENT_XY_KEY, None)
        setattr(self.blackboard, CURRENT_YAW_KEY, None)
        setattr(self.blackboard, MIN_OBSTACLE_DISTANCE_KEY, None)
        setattr(self.blackboard, FRONT_CLEARANCE_KEY, None)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("CheckStopCondition.setup() didn't find 'node' in kwargs") from e
        self.node.create_subscription(Odometry, self._odom_topic, self._odom_cb, 10)
        self.node.create_subscription(
            Float32, self._min_obstacle_distance_topic, self._min_obstacle_cb, 10)
        self.node.create_subscription(
            Bool, self._goal_reached_topic, self._goal_reached_cb, 10)
        self.node.create_subscription(
            Float32, self._front_clearance_topic, self._front_clearance_cb, 10)
        self.hold_pub = self.node.create_publisher(Bool, self._hold_topic, 10)

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = _quaternion_to_yaw(msg.pose.pose.orientation)
        # Unwrap-and-accumulate THIS message's own small step, not a re-derived
        # current-minus-start delta -- see module docstring's "Turn
        # accumulation" paragraph for why. Every odom message folds in here
        # (not just once per BT tick), so a fast turn between two ticks can't
        # be missed/collapsed.
        if self._turn_accum_prev_yaw is not None:
            step_rad = math.atan2(
                math.sin(self.yaw - self._turn_accum_prev_yaw),
                math.cos(self.yaw - self._turn_accum_prev_yaw),
            )
            self._turn_accum_deg += math.degrees(step_rad)
        self._turn_accum_prev_yaw = self.yaw
        setattr(self.blackboard, CURRENT_XY_KEY, (self.x, self.y))
        setattr(self.blackboard, CURRENT_YAW_KEY, self.yaw)

    def _min_obstacle_cb(self, msg: Float32):
        setattr(self.blackboard, MIN_OBSTACLE_DISTANCE_KEY, float(msg.data))

    def _front_clearance_cb(self, msg: Float32):
        setattr(self.blackboard, FRONT_CLEARANCE_KEY, float(msg.data))

    def _goal_reached_cb(self, msg: Bool):
        # Only ever latches True -- see module docstring for why a bare
        # "last received value" would misfire on a stale True from a
        # previous move. Reset-on-move-change happens in update().
        if msg.data:
            self._goal_reached_flag = True

    def update(self):
        state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)
        move = state.current_move
        if move is None:
            # Defensive only -- MissionActive already gates on RUNNING/HOLDING,
            # which shouldn't be reachable without a valid current move.
            return py_trees.common.Status.FAILURE

        if move.id != self._goal_reached_tracked_move_id:
            self._goal_reached_tracked_move_id = move.id
            self._goal_reached_flag = False
            self._turn_accum_deg = 0.0
            self._turn_accum_prev_yaw = self.yaw

        current_xy = (self.x, self.y) if self.x is not None else None
        if state.move_start_xy is None and current_xy is not None:
            state.move_start_xy = current_xy
        if state.move_start_yaw is None and self.yaw is not None:
            state.move_start_yaw = self.yaw

        now = time.monotonic()

        if move.vdes is not None:
            state.log_stub_once(
                self.node.get_logger(), f'{move.id}:vdes',
                f"[mission] Move '{move.id}' sets vdes={move.vdes} but mpc_corr has no "
                'external vdes-override mechanism yet -- ignored (TODO).'
            )

        # timeout_sec is optional as of schema_version 2.0 -- None means no
        # cap is enforced for this move at all, so the whole block is skipped
        # rather than crashing on a None comparison.
        if move.timeout_sec is not None and (now - state.move_start_time) >= move.timeout_sec:
            if move.on_timeout == 'skip':
                self.node.get_logger().warn(
                    f"[mission] Move '{move.id}' timed out after {move.timeout_sec}s -- "
                    "on_timeout=skip, advancing."
                )
                return py_trees.common.Status.SUCCESS
            if move.on_timeout == 'stop':
                # Neither abort (mission.state -> ABORTED) nor skip (advance)
                # -- just hold here. Logged once (not every tick, unlike the
                # hold publish itself, which is cheap/idempotent to repeat)
                # -- see module docstring's on_timeout='stop' paragraph.
                state.log_stub_once(
                    self.node.get_logger(), f'{move.id}:on_timeout_stop',
                    f"[mission] Move '{move.id}' timed out after {move.timeout_sec}s -- "
                    'on_timeout=stop: holding here, mission left on this move '
                    '(not aborted, not advanced) until manually intervened.'
                )
                self.hold_pub.publish(Bool(data=True))
                return py_trees.common.Status.RUNNING
            self.node.get_logger().error(
                f"[mission] Move '{move.id}' timed out after {move.timeout_sec}s -- "
                'on_timeout=abort, aborting mission.'
            )
            state.abort()
            return py_trees.common.Status.FAILURE

        min_obstacle_distance = getattr(self.blackboard, MIN_OBSTACLE_DISTANCE_KEY)
        front_clearance = getattr(self.blackboard, FRONT_CLEARANCE_KEY)
        ctx = EvalContext(
            now=now,
            move_start_time=state.move_start_time,
            move_start_xy=state.move_start_xy,
            current_xy=current_xy,
            detected_classes=getattr(self.blackboard, DETECTED_CLASSES_KEY),
            min_obstacle_distance=min_obstacle_distance,
            front_clearance=front_clearance,
            default_distance=move.goal_distance,
            goal_reached=self._goal_reached_flag,
            current_yaw=self.yaw,
            turn_start_yaw=state.move_start_yaw,
            turn_accum_deg=(
                self._turn_accum_deg if self._turn_accum_prev_yaw is not None else None
            ),
        )
        result = evaluate(move.stop_condition, ctx)

        if result is None:
            assert move.stop_condition.type in STUB_STOP_CONDITION_TYPES
            state.log_stub_once(
                self.node.get_logger(), f'{move.id}:stop_condition',
                f"[mission] Move '{move.id}' uses stop_condition.type="
                f"{move.stop_condition.type!r}, which has no real implementation yet "
                '(TODO) -- will only ever advance via timeout_sec.'
            )
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.SUCCESS if result else py_trees.common.Status.RUNNING
