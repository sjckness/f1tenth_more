"""Dash-cam mode of test_campaign_logger: /mission/status opens and closes tests.

With ``auto_mode`` true the logger needs no plan_result and no
open_test.sh: every mission START opens a test and its end closes it,
decided from f1tenth_behavior's latched /mission/status (the same source and
QoS as mission_logger_node). Pure logic, no ROS, so it is tested offline.

    status edge                              action
    -> RUNNING/HOLDING from anything else    open (close an open test first:
                                             INTERRUPTED)
    RUNNING <-> HOLDING                      nothing (one mission)
    -> COMPLETE                              close completed, COMPLETE
    -> ABORTED (abort, Foxglove stop)        close aborted,   ABORTED
    emergency_stop_active                    close aborted,   EMERGENCY_STOP
    -> LOADED while a test is open           close aborted,   INTERRUPTED
    -> IDLE while a test is open             close aborted,   IDLE
    LOADED with nothing open                 nothing (load without start)
    the same message again                   nothing (idempotent)

The first message after the logger starts is compared with "nothing": a
latched RUNNING opens a test for a mission that was already driving.

``completed``/``aborted`` is TestLogger's own outcome vocabulary (meta.json
``auto_outcome``, which the analysis reads); the status label goes into the
close reason and meta.json ``mission_outcome``.
"""

from __future__ import annotations

import os

STARTED_STATES = frozenset({"RUNNING", "HOLDING"})

#: label -> TestLogger outcome
LABEL_OUTCOME = {
    "COMPLETE": "completed",
    "ABORTED": "aborted",
    "EMERGENCY_STOP": "aborted",
    "INTERRUPTED": "aborted",
    "IDLE": "aborted",
}


def mission_id(json_path):
    """``/x/missions/straight_2m_then_left_door.json`` -> ``straight_2m_then_left_door``."""
    base = os.path.basename(str(json_path or ""))
    return base[:-5] if base.endswith(".json") else base


class AutoTrigger:
    """Turns /mission/status messages into ('open', mission_id) / ('close', label)."""

    def __init__(self):
        self._last = None   # (state, json_path, estop) of the last message

    def decide(self, state, json_path, estop, test_open):
        """Actions for one status message. ``test_open``: a test is recording
        and not already closing."""
        state = str(state or "").upper()
        cur = (state, str(json_path or ""), bool(estop))
        prev, self._last = self._last, cur
        if cur == prev:
            return []
        prev_state = prev[0] if prev else None
        if cur[2]:
            # latched e-stop: the car cannot drive, so nothing opens either
            return [("close", "EMERGENCY_STOP")] if test_open else []
        if state in STARTED_STATES:
            if prev_state in STARTED_STATES:
                return []
            actions = [("close", "INTERRUPTED")] if test_open else []
            return actions + [("open", mission_id(json_path))]
        if not test_open:
            return []
        if state == "COMPLETE":
            return [("close", "COMPLETE")]
        if state == "ABORTED":
            return [("close", "ABORTED")]
        if state == "LOADED":
            return [("close", "INTERRUPTED")]
        if state == "IDLE":
            return [("close", "IDLE")]
        return []
