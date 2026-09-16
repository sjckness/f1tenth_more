"""Every mission abort engages /mpc/hold.

The four ways a running mission aborts, and where each is covered:
  CheckStopCondition move timeout (on_timeout abort)  here, for every move type
  HandleObjectAction on_object abort_mission          here
  /mission/abort_mission service                       test_mission_loader_hold_release.py
  GoToObject failure outcomes                          test_go_to_object_behaviour.py

The timeout path used to publish nothing (hold only for object moves since
22ff031): the mission subtree stopped ticking while mpc_corr kept driving the
move's last goal. The mpc_corr side -- hold to zero /drive within one control
tick -- is test_abort_holds_the_car.py in mpc_controller.

Run standalone: python3 -m pytest test/test_every_abort_holds.py -v
"""

import time
from types import SimpleNamespace

import pytest

py_trees = pytest.importorskip('py_trees')

import f1tenth_behavior.behaviours.check_stop_condition as check_module  # noqa: E402
import f1tenth_behavior.behaviours.handle_object_action as action_module  # noqa: E402
from f1tenth_behavior.behaviours.check_stop_condition import (  # noqa: E402
    CheckStopCondition,
)
from f1tenth_behavior.behaviours.handle_object_action import HandleObjectAction  # noqa: E402
from f1tenth_behavior.mission.detected_classes_bridge import (  # noqa: E402
    DETECTED_CLASSES_KEY,
)
from f1tenth_behavior.mission.mission_config import parse_mission  # noqa: E402
from f1tenth_behavior.mission.runtime import (  # noqa: E402
    CURRENT_XY_KEY,
    FRONT_CLEARANCE_KEY,
    GLOBAL_TURN_ACCUM_KEY,
    GLOBAL_XY_KEY,
    GLOBAL_YAW_KEY,
    MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
    MIN_OBSTACLE_DISTANCE_KEY,
    MISSION_KEY,
    MissionRuntimeState,
    MissionState,
)

S = py_trees.common.Status


@pytest.fixture(autouse=True)
def _no_report_files(monkeypatch):
    """Both abort paths write a mission summary into the source tree; not here."""
    monkeypatch.setattr(check_module, 'write_mission_summary', lambda *a, **k: None)
    monkeypatch.setattr(action_module, 'write_mission_summary', lambda *a, **k: None)


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _logger():
    return SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None,
                           error=lambda *a, **k: None)


MOVES = {
    'goal_distance': {'goal_distance': 3.0, 'stop_condition': {'type': 'distance_reached'}},
    'goal_pose': {'goal_pose': {'x': 3.0, 'y': 0.0, 'yaw': 0.0},
                  'stop_condition': {'type': 'goal_reached'}},
    'turn': {'turn': {'heading_delta_deg': 90.0, 'speed': 0.3, 'steering': 'full_lock'},
             'stop_condition': {'type': 'orientation_delta', 'value': 90.0}},
    'drive': {'drive': {'mode': 'straight', 'speed': 0.4},
              'stop_condition': {'type': 'front_clearance', 'distance': 1.0}},
    'go_to_object': {'go_to_object': {'target_class': 'person', 'speed': 0.4,
                                      'acquire_timeout_sec': 5.0},
                     'stop_condition': {'type': 'object_reached'}},
}


def _state(move_type):
    move = dict(MOVES[move_type], id='m', timeout_sec=10, on_timeout='abort', terminal=True)
    config = parse_mission({'mission_id': 't', 'schema_version': '4.0', 'moves': [move]})
    state = MissionRuntimeState()
    state.load(config, time.monotonic())
    state.begin(time.monotonic())
    return state


def _check(state):
    behaviour = CheckStopCondition()
    behaviour.node = SimpleNamespace(get_logger=_logger)
    behaviour.blackboard = SimpleNamespace()
    setattr(behaviour.blackboard, MISSION_KEY, state)
    for key in (MIN_OBSTACLE_DISTANCE_KEY, MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
                FRONT_CLEARANCE_KEY):
        setattr(behaviour.blackboard, key, None)
    setattr(behaviour.blackboard, DETECTED_CLASSES_KEY, {})
    behaviour.hold_pub, behaviour.object_end_pub = _Pub(), _Pub()
    return behaviour


@pytest.mark.parametrize('move_type', sorted(MOVES))
def test_a_timeout_abort_holds_on_the_same_tick(move_type):
    state = _state(move_type)
    behaviour = _check(state)
    state.move_start_time -= 11.0
    assert behaviour.update() == S.FAILURE
    assert state.state == MissionState.ABORTED
    assert [m.data for m in behaviour.hold_pub.msgs] == [True]


def test_timeout_skip_and_a_running_move_do_not_hold():
    state = _state('goal_distance')
    behaviour = _check(state)
    assert behaviour.update() == S.RUNNING
    assert behaviour.hold_pub.msgs == []


def test_on_object_abort_mission_holds():
    config = parse_mission({'mission_id': 't', 'moves': [{
        'id': 'm', 'goal_distance': 3.0, 'stop_condition': {'type': 'distance_reached'},
        'on_object': [{'class': 'person', 'action': 'abort_mission', 'reason': 'test'}]}]})
    state = MissionRuntimeState()
    state.load(config, time.monotonic())
    state.begin(time.monotonic())
    behaviour = HandleObjectAction()
    behaviour.node = SimpleNamespace(get_logger=_logger)
    behaviour.blackboard = SimpleNamespace()
    setattr(behaviour.blackboard, MISSION_KEY, state)
    for key in (CURRENT_XY_KEY, GLOBAL_XY_KEY, GLOBAL_YAW_KEY, GLOBAL_TURN_ACCUM_KEY):
        setattr(behaviour.blackboard, key, None)
    behaviour.hold_pub = _Pub()
    entry = config.moves[0].on_object[0]
    behaviour._dispatch(state, config.moves[0], entry)
    assert state.state == MissionState.ABORTED
    assert [m.data for m in behaviour.hold_pub.msgs] == [True]
