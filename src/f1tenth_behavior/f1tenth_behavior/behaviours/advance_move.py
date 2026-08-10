"""py_trees Action: advance to the next move once CheckStopCondition succeeds.

Only reached when CheckStopCondition just returned SUCCESS (Sequence
semantics -- see progress_sequence in behavior_executor_node.create_root()),
so this behaviour's own job is unconditional: move on. If that was the last
move, transition mission.state to COMPLETE and publish /mpc/hold(True) so
mpc_corr actually stops instead of being left mid-move with a stale unreachable
goal (safety requirement from the task) -- mission.goal_dirty is also cleared
by MissionRuntimeState.complete() so PublishMoveGoal (next in the Sequence)
does not fire again with nothing left to publish.
"""

import time

import py_trees
from std_msgs.msg import Bool

from f1tenth_behavior.mission.runtime import MISSION_KEY, MissionRuntimeState


class AdvanceMove(py_trees.behaviour.Behaviour):

    def __init__(self, name='AdvanceMove', hold_topic='/mpc/hold'):
        super().__init__(name=name)
        self._hold_topic = hold_topic
        self.node = None
        self.hold_pub = None
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=MISSION_KEY, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("AdvanceMove.setup() didn't find 'node' in kwargs") from e
        self.hold_pub = self.node.create_publisher(Bool, self._hold_topic, 10)

    def update(self):
        state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)
        finished_move = state.current_move
        next_index = state.current_index + 1

        if state.config is None or next_index >= len(state.config.moves):
            state.complete()
            self.hold_pub.publish(Bool(data=True))
            self.node.get_logger().info(
                f"[mission] '{state.config.mission_id if state.config else '?'}' COMPLETE "
                f"after move '{finished_move.id if finished_move else '?'}'."
            )
            return py_trees.common.Status.SUCCESS

        state.goto_move(next_index, now=time.monotonic())
        next_move = state.current_move
        self.node.get_logger().info(
            f"[mission] Move '{finished_move.id if finished_move else '?'}' complete -- "
            f"advancing to '{next_move.id}' ({next_index}/{len(state.config.moves) - 1})."
        )
        return py_trees.common.Status.SUCCESS
