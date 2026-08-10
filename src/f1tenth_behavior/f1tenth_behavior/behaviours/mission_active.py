"""py_trees Condition gating the whole mission subtree.

Deviates from the literal one-line spec ("SUCCESS if mission.state == RUNNING")
in one deliberate way: SUCCESS for RUNNING *or* HOLDING, not RUNNING alone.

Reasoning (flagged explicitly, not a silent change): the mission subtree is
mission := Sequence[MissionActive, Selector[object_response, progress]]. If
MissionActive gated on RUNNING only, then the tick HandleObjectAction sets
mission.state = HOLDING, every subsequent tick would fail MissionActive
*before* the Selector -- and therefore before ObjectSeen/HandleObjectAction --
ever ticks again. Nothing would ever be able to notice the resume_condition
being satisfied, and a hold would be permanent with no way out. Including
HOLDING here is what lets HandleObjectAction keep being reached tick after
tick while holding, to actually check for resume (see its own docstring).
IDLE/COMPLETE/ABORTED are correctly excluded either way -- those are genuine
"stop ticking this subtree" states.
"""

import py_trees

from f1tenth_behavior.mission.runtime import MISSION_KEY, MissionRuntimeState, MissionState


class MissionActive(py_trees.behaviour.Behaviour):

    def __init__(self, name='MissionActive'):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=MISSION_KEY, access=py_trees.common.Access.READ)

    def update(self):
        state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)
        if state.state in (MissionState.RUNNING, MissionState.HOLDING):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
