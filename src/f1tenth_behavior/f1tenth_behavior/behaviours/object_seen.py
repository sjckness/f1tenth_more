"""py_trees Condition: is there an on_object trigger for the current move?

Deviates from the literal "ObjectSeen(class_name)" signature in the task
description: a Move's on_object is a *list* of {class, action} entries (a move
can watch several classes at once, each with its own action), not a single
fixed class. Taking a constructor-time class_name would only support one -- so
instead this checks the *current move's* on_object list dynamically every tick
(mirroring how HandleObjectAction dispatch also has to be move-dependent), and
records which entry matched onto the blackboard (as `mission.matched_object`)
for the sibling HandleObjectAction to read and act on. This also matches the
task's own tree diagram literally -- one fixed ObjectSeen -> HandleObjectAction
pair, no per-move tree rebuilding -- while still supporting the array.

Also SUCCESS whenever mission.state == HOLDING, regardless of current
detections: this is what lets HandleObjectAction keep being reached every tick
while holding, even after the triggering class clears, so it can actually
detect the resume condition being satisfied and end the hold. See
MissionActive's docstring for the matching half of this same fix -- both were
needed together, one alone still dead-ends.

Debounce window: object_seen (the BT condition, this file) has no per-entry
config field for it in the on_object schema (only the *stop_condition* type
object_cleared has a configurable debounce_sec) -- DEFAULT_SEEN_FRESHNESS_SEC
from condition_eval.py is reused here as the assumed default, flagged back to
the user rather than invented silently.
"""

import time

import py_trees

from f1tenth_behavior.mission.condition_eval import DEFAULT_SEEN_FRESHNESS_SEC
from f1tenth_behavior.mission.detected_classes_bridge import DETECTED_CLASSES_KEY
from f1tenth_behavior.mission.runtime import MISSION_KEY, MissionRuntimeState, MissionState

MATCHED_OBJECT_KEY = 'mission_matched_object'


class ObjectSeen(py_trees.behaviour.Behaviour):

    def __init__(self, name='ObjectSeen', freshness_sec=DEFAULT_SEEN_FRESHNESS_SEC):
        super().__init__(name=name)
        self.freshness_sec = freshness_sec
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=MISSION_KEY, access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key=DETECTED_CLASSES_KEY, access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key=MATCHED_OBJECT_KEY, access=py_trees.common.Access.WRITE)

    def update(self):
        state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)

        if state.state == MissionState.HOLDING:
            # Keep the object_response branch reachable so HandleObjectAction can
            # check for resume even once the triggering class is gone -- see
            # module docstring.
            return py_trees.common.Status.SUCCESS

        move = state.current_move
        if move is None or not move.on_object:
            return py_trees.common.Status.FAILURE

        detected = getattr(self.blackboard, DETECTED_CLASSES_KEY)
        now = time.monotonic()
        for entry in move.on_object:
            info = detected.get(entry.cls)
            if info is not None and (now - info.last_seen) <= self.freshness_sec:
                setattr(self.blackboard, MATCHED_OBJECT_KEY, entry)
                return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE
