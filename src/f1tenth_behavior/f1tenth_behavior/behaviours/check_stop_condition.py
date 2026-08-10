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
"""

import time

import py_trees
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32

from f1tenth_behavior.mission.condition_eval import EvalContext, evaluate
from f1tenth_behavior.mission.mission_config import STUB_STOP_CONDITION_TYPES
from f1tenth_behavior.mission.detected_classes_bridge import DETECTED_CLASSES_KEY
from f1tenth_behavior.mission.runtime import MISSION_KEY, MissionRuntimeState

CURRENT_XY_KEY = 'mission_current_xy'
MIN_OBSTACLE_DISTANCE_KEY = 'mission_min_obstacle_distance'


class CheckStopCondition(py_trees.behaviour.Behaviour):

    def __init__(self, name='CheckStopCondition', odom_topic='/odom',
                 min_obstacle_distance_topic='/mpc/min_obstacle_distance',
                 goal_reached_topic='/mpc/goal_reached'):
        super().__init__(name=name)
        self._odom_topic = odom_topic
        self._min_obstacle_distance_topic = min_obstacle_distance_topic
        self._goal_reached_topic = goal_reached_topic
        self.node = None
        self.x = None
        self.y = None
        self._goal_reached_flag = False
        self._goal_reached_tracked_move_id = None

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=MISSION_KEY, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            key=DETECTED_CLASSES_KEY, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=CURRENT_XY_KEY, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            key=MIN_OBSTACLE_DISTANCE_KEY, access=py_trees.common.Access.WRITE)
        setattr(self.blackboard, CURRENT_XY_KEY, None)
        setattr(self.blackboard, MIN_OBSTACLE_DISTANCE_KEY, None)

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

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        setattr(self.blackboard, CURRENT_XY_KEY, (self.x, self.y))

    def _min_obstacle_cb(self, msg: Float32):
        setattr(self.blackboard, MIN_OBSTACLE_DISTANCE_KEY, float(msg.data))

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

        current_xy = (self.x, self.y) if self.x is not None else None
        if state.move_start_xy is None and current_xy is not None:
            state.move_start_xy = current_xy

        now = time.monotonic()

        if move.vdes is not None:
            state.log_stub_once(
                self.node.get_logger(), f'{move.id}:vdes',
                f"[mission] Move '{move.id}' sets vdes={move.vdes} but mpc_corr has no "
                'external vdes-override mechanism yet -- ignored (TODO).'
            )

        if (now - state.move_start_time) >= move.timeout_sec:
            if move.on_timeout == 'skip':
                self.node.get_logger().warn(
                    f"[mission] Move '{move.id}' timed out after {move.timeout_sec}s -- "
                    "on_timeout=skip, advancing."
                )
                return py_trees.common.Status.SUCCESS
            self.node.get_logger().error(
                f"[mission] Move '{move.id}' timed out after {move.timeout_sec}s -- "
                'on_timeout=abort, aborting mission.'
            )
            state.abort()
            return py_trees.common.Status.FAILURE

        min_obstacle_distance = getattr(self.blackboard, MIN_OBSTACLE_DISTANCE_KEY)
        ctx = EvalContext(
            now=now,
            move_start_time=state.move_start_time,
            move_start_xy=state.move_start_xy,
            current_xy=current_xy,
            detected_classes=getattr(self.blackboard, DETECTED_CLASSES_KEY),
            min_obstacle_distance=min_obstacle_distance,
            default_distance=move.goal_distance,
            goal_reached=self._goal_reached_flag,
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
