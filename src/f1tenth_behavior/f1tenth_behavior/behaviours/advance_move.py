"""py_trees Action: advance to the next move once CheckStopCondition succeeds.

Only reached when CheckStopCondition just returned SUCCESS (Sequence
semantics -- see progress_sequence in behavior_executor_node.create_root()),
so this behaviour's own job is unconditional: move on. If that was the last
move, transition mission.state to COMPLETE and publish /mpc/hold(True) so
mpc_corr actually stops instead of being left mid-move with a stale unreachable
goal (safety requirement from the task) -- mission.goal_dirty is also cleared
by MissionRuntimeState.complete() so PublishMoveGoal (next in the Sequence)
does not fire again with nothing left to publish.

Closed-loop verification/precision scoring (3.2/3.5): this is the ONE place
that sees every NORMAL move transition (advance-to-next AND mission-complete
both land here -- see the two branches in update()), so it is also where
mission/move_scoring.py.record_move_outcome() gets called for the move that
just finished, and where write_mission_summary() gets called on mission
completion. The other ways a move/mission can end -- a move's own
timeout_sec aborting the mission, or an on_object abort_mission/skip_to_move
-- never reach this behaviour at all (Sequence/Selector semantics -- see
behavior_executor_node's tree comment), so CheckStopCondition's own timeout-
abort branch and HandleObjectAction's abort_mission/skip_to_move branches
each call record_move_outcome() themselves instead. See move_scoring.py's
own module docstring for the one path that currently does NOT (the external
/mission/abort_mission service call) -- flagged there, not silently missed.
"""

import time

import py_trees
from std_msgs.msg import Bool

from f1tenth_behavior.mission.move_scoring import record_move_outcome, write_mission_summary
from f1tenth_behavior.mission.runtime import (
    GLOBAL_TURN_ACCUM_KEY,
    GLOBAL_XY_KEY,
    GLOBAL_YAW_KEY,
    MISSION_KEY,
    MissionRuntimeState,
)


class AdvanceMove(py_trees.behaviour.Behaviour):

    def __init__(self, name='AdvanceMove', hold_topic='/mpc/hold'):
        super().__init__(name=name)
        self._hold_topic = hold_topic
        self.node = None
        self.hold_pub = None
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=MISSION_KEY, access=py_trees.common.Access.WRITE)
        # READ-only -- CheckStopCondition owns writing these (see runtime.py's
        # own comment on why the key constants live there). Read here only to
        # get the finished move's END global pose/turn-accum for scoring --
        # see mission/move_scoring.py's own docstring for why global EKF,
        # not local odom.
        self.blackboard.register_key(key=GLOBAL_XY_KEY, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=GLOBAL_YAW_KEY, access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key=GLOBAL_TURN_ACCUM_KEY, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("AdvanceMove.setup() didn't find 'node' in kwargs") from e
        self.hold_pub = self.node.create_publisher(Bool, self._hold_topic, 10)

    def _record_finished_move(self, state, finished_move):
        record_move_outcome(
            state, self.node.get_logger(), finished_move,
            stop_reason=state.last_stop_reason or 'unknown',
            now=time.monotonic(),
            end_global_xy=getattr(self.blackboard, GLOBAL_XY_KEY),
            end_global_yaw=getattr(self.blackboard, GLOBAL_YAW_KEY),
            global_turn_accum_deg=getattr(self.blackboard, GLOBAL_TURN_ACCUM_KEY),
        )

    def update(self):
        state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)
        finished_move = state.current_move
        next_index = state.current_index + 1

        if finished_move is not None:
            self._record_finished_move(state, finished_move)

        if state.config is None or next_index >= len(state.config.moves):
            state.complete()
            self.hold_pub.publish(Bool(data=True))
            self.node.get_logger().info(
                f"[mission] '{state.config.mission_id if state.config else '?'}' COMPLETE "
                f"after move '{finished_move.id if finished_move else '?'}'."
            )
            write_mission_summary(
                state.config.mission_id if state.config else 'unknown',
                state.move_outcomes, 'COMPLETE', state.mission_start_wall_time,
                self.node.get_logger(),
            )
            return py_trees.common.Status.SUCCESS

        state.goto_move(next_index, now=time.monotonic())
        next_move = state.current_move
        self.node.get_logger().info(
            f"[mission] Move '{finished_move.id if finished_move else '?'}' complete -- "
            f"advancing to '{next_move.id}' ({next_index}/{len(state.config.moves) - 1})."
        )
        return py_trees.common.Status.SUCCESS
