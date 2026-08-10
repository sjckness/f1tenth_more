"""py_trees Action: dispatch on_object matches per the action table, and manage
a stop_and_hold's resume once already holding.

Two distinct modes, both handled here (see module-level control flow in
update()):

1. Freshly triggered (mission.state == RUNNING, sibling ObjectSeen just matched
   an on_object entry and wrote it to the blackboard): dispatch per entry.action.

2. Already holding (mission.state == HOLDING, reached because MissionActive/
   ObjectSeen both let HOLDING through -- see their own docstrings for why):
   check whether the hold's resume_condition (or the implicit object_cleared
   default) is now satisfied, and if so, resume.

Hold mechanism: publishes /mpc/hold (std_msgs/Bool) rather than touching
/mpc/goal_distance -- see mpc_corr's own hold_callback/control_loop for the
matching receive side. This was flagged back to the user before implementing
(republishing goal_distance resets mpc_corr's goal_start_xy, clearing progress)
and confirmed: add /mpc/hold to mpc_corr.py, which is done.

Return-status convention per action (deliberate, not from the literal spec,
which didn't say): stop_and_hold/abort_mission/skip_to_move return SUCCESS
(they actively change mission flow and should win the outer Selector's tick
over the progress Sequence -- see behavior_executor_node's tree comment).
log_only and the reduce_speed*/vdes-override stubs return FAILURE instead: they
have no control effect, so letting them "win" the Selector would silently
block normal mission progression (CheckStopCondition/AdvanceMove/PublishMoveGoal)
every tick an already-inert on_object class happens to be in view, which is
exactly the kind of confusing failure the task asked to avoid for the stubs.
"""

import time

import py_trees
from std_msgs.msg import Bool

from f1tenth_behavior.behaviours.check_stop_condition import (
    CURRENT_XY_KEY,
    MIN_OBSTACLE_DISTANCE_KEY,
)
from f1tenth_behavior.behaviours.object_seen import MATCHED_OBJECT_KEY
from f1tenth_behavior.mission.condition_eval import EvalContext, evaluate
from f1tenth_behavior.mission.detected_classes_bridge import DETECTED_CLASSES_KEY
from f1tenth_behavior.mission.mission_config import StopCondition
from f1tenth_behavior.mission.runtime import (
    MISSION_KEY,
    HoldContext,
    MissionRuntimeState,
    MissionState,
)


class HandleObjectAction(py_trees.behaviour.Behaviour):

    def __init__(self, name='HandleObjectAction', hold_topic='/mpc/hold'):
        super().__init__(name=name)
        self._hold_topic = hold_topic
        self.node = None
        self.hold_pub = None

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=MISSION_KEY, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            key=MATCHED_OBJECT_KEY, access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key=DETECTED_CLASSES_KEY, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=CURRENT_XY_KEY, access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key=MIN_OBSTACLE_DISTANCE_KEY, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("HandleObjectAction.setup() didn't find 'node' in kwargs") from e
        self.hold_pub = self.node.create_publisher(Bool, self._hold_topic, 10)

    def update(self):
        state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)

        if state.state == MissionState.HOLDING:
            return self._check_resume(state)

        move = state.current_move
        entry = getattr(self.blackboard, MATCHED_OBJECT_KEY)
        if move is None or entry is None or entry not in move.on_object:
            # Stale match (e.g. the move changed between ObjectSeen's tick and
            # this one) -- shouldn't happen since they're ticked back-to-back in
            # the same Sequence pass, but don't act on the wrong move's entry.
            return py_trees.common.Status.FAILURE

        return self._dispatch(state, move, entry)

    # -- freshly-triggered dispatch -------------------------------------------

    def _dispatch(self, state, move, entry):
        action = entry.action

        if action == 'stop_and_hold':
            resume_condition = entry.params.get('resume_condition')  # StopCondition or None
            state.hold_context = HoldContext(
                triggering_class=entry.cls,
                resume_condition=resume_condition,
                hold_start_time=time.monotonic(),
                hold_start_xy=getattr(self.blackboard, CURRENT_XY_KEY),
            )
            state.state = MissionState.HOLDING
            self.hold_pub.publish(Bool(data=True))
            self.node.get_logger().warn(
                f"[mission] Move '{move.id}': '{entry.cls}' seen -> stop_and_hold "
                f"(resume={'object cleared' if resume_condition is None else resume_condition.type})."
            )
            return py_trees.common.Status.SUCCESS

        if action == 'abort_mission':
            reason = entry.params['reason']
            state.abort()
            self.hold_pub.publish(Bool(data=True))
            self.node.get_logger().error(
                f"[mission] Move '{move.id}': '{entry.cls}' seen -> abort_mission: {reason}"
            )
            return py_trees.common.Status.SUCCESS

        if action == 'skip_to_move':
            target_id = entry.params['move_id']
            target_index = state.config.index_of(target_id)
            if target_index is None:
                # mission_config.py validates skip_to_move targets against known
                # move ids at load time -- reaching this means a bug upstream,
                # not a user config error. Fail closed, don't crash the tick.
                self.node.get_logger().error(
                    f"[mission] Move '{move.id}': skip_to_move target {target_id!r} not "
                    'found at runtime (should have been caught at load time) -- ignoring.'
                )
                return py_trees.common.Status.FAILURE
            self.node.get_logger().warn(
                f"[mission] Move '{move.id}': '{entry.cls}' seen -> skip_to_move '{target_id}'."
            )
            state.goto_move(target_index, now=time.monotonic())
            return py_trees.common.Status.SUCCESS

        if action == 'log_only':
            self.node.get_logger().info(
                f"[mission] Move '{move.id}': '{entry.cls}' seen -> {entry.params['message']}"
            )
            return py_trees.common.Status.FAILURE  # pass through -- see module docstring

        if action in ('reduce_speed', 'reduce_speed_for'):
            state.log_stub_once(
                self.node.get_logger(), f'{move.id}:{entry.cls}:{action}',
                f"[mission] Move '{move.id}': '{entry.cls}' seen -> {action}, but mpc_corr "
                'has no external vdes-override mechanism yet -- ignored (TODO).'
            )
            return py_trees.common.Status.FAILURE  # pass through, same reasoning

        # Unreachable if mission_config.py validated `action` against the same
        # set -- fail safe rather than crash the tick if it somehow wasn't.
        return py_trees.common.Status.FAILURE

    # -- already-holding: check for resume ------------------------------------

    def _check_resume(self, state):
        hold_ctx = state.hold_context
        if hold_ctx is None:
            self.node.get_logger().error(
                '[mission] state=HOLDING but hold_context is None -- forcing resume '
                'rather than holding forever with no way out.'
            )
            self._resume(state)
            return py_trees.common.Status.SUCCESS

        resume_condition = hold_ctx.resume_condition
        if resume_condition is None:
            resume_condition = StopCondition(
                type='object_cleared',
                params={'class': hold_ctx.triggering_class, 'debounce_sec': 1.0},
            )

        ctx = EvalContext(
            now=time.monotonic(),
            move_start_time=hold_ctx.hold_start_time,
            move_start_xy=hold_ctx.hold_start_xy,
            current_xy=getattr(self.blackboard, CURRENT_XY_KEY),
            detected_classes=getattr(self.blackboard, DETECTED_CLASSES_KEY),
            min_obstacle_distance=getattr(self.blackboard, MIN_OBSTACLE_DISTANCE_KEY),
            default_distance=None,
        )
        result = evaluate(resume_condition, ctx)

        if result is None:
            move = state.current_move
            state.log_stub_once(
                self.node.get_logger(),
                f'{move.id if move else "?"}:resume:{resume_condition.type}',
                f"[mission] Hold resume_condition.type={resume_condition.type!r} has no "
                'real implementation yet (TODO) -- this hold will not auto-resume.'
            )
            return py_trees.common.Status.RUNNING

        if not result:
            return py_trees.common.Status.RUNNING

        self._resume(state)
        return py_trees.common.Status.SUCCESS

    def _resume(self, state):
        state.state = MissionState.RUNNING
        state.hold_context = None
        self.hold_pub.publish(Bool(data=False))
        self.node.get_logger().warn('[mission] Resumed -- hold released.')
