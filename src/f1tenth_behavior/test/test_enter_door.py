"""The enter_door move: its schema, its door selection, and its arrival test.

The behaviour's ROS glue (the subscription, the two publishers) is not
exercised here; what is exercised is every decision it makes -- which door it
resolves to, when it publishes a goal, when `entered` goes true, and the two
ways it fails -- by driving EnterDoor.update() against a fake node and a fake
blackboard. The same shape test_go_to_object_behaviour.py uses.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from f1tenth_behavior.mission.mission_config import (  # noqa: E402
    MissionConfigError, parse_mission)


# ---------------------------------------------------------------------------
# The schema
# ---------------------------------------------------------------------------
def door_mission(**door):
    spec = {'type': 'enter_door', 'door': 'nearest', 'xy_tol': 0.15,
            'yaw_tol_deg': 10}
    spec.update(door)
    return {
        'mission_id': 'doors', 'schema_version': '6.0',
        'moves': [{'id': 'm0', 'enter_door': spec,
                   'stop_condition': {'type': 'door_entered'},
                   'timeout_sec': 60, 'on_timeout': 'abort', 'terminal': True}],
    }


def test_the_example_mission_loads():
    path = os.path.join(os.path.dirname(__file__), '..', 'missions',
                        'enter_nearest_door.json')
    from f1tenth_behavior.mission.mission_config import load_mission_file
    config = load_mission_file(path)
    spec = config.moves[0].enter_door
    assert config.schema_version == '6.0'
    assert (spec.door, spec.xy_tol, spec.yaw_tol_deg) == ('nearest', 0.15, 10.0)
    assert config.moves[0].stop_condition.type == 'door_entered'


def test_the_goal_as_specified_parses():
    spec = parse_mission(door_mission()).moves[0].enter_door
    assert spec.door == 'nearest'
    assert spec.acquire_timeout_sec == 5.0      # the default


def test_a_named_door_parses_and_a_malformed_name_does_not():
    assert parse_mission(door_mission(door='door_7')).moves[0].enter_door.door == 'door_7'
    for bad in ('door', 'door_', 'door_x', 'nearest_door', ''):
        with pytest.raises(MissionConfigError):
            parse_mission(door_mission(door=bad))


def test_the_tolerances_are_validated():
    for bad in (0.0, -0.1):
        with pytest.raises(MissionConfigError):
            parse_mission(door_mission(xy_tol=bad))
        with pytest.raises(MissionConfigError):
            parse_mission(door_mission(yaw_tol_deg=bad))
    with pytest.raises(MissionConfigError):
        parse_mission(door_mission(yaw_tol_deg=181.0))


def test_enter_door_and_door_entered_require_each_other():
    raw = door_mission()
    raw['moves'][0]['stop_condition'] = {'type': 'goal_reached'}
    with pytest.raises(MissionConfigError):
        parse_mission(raw)
    raw = door_mission()
    del raw['moves'][0]['enter_door']
    raw['moves'][0]['goal_distance'] = 1.0
    with pytest.raises(MissionConfigError):
        parse_mission(raw)


def test_enter_door_is_exclusive_with_the_other_goal_shapes():
    raw = door_mission()
    raw['moves'][0]['goal_pose'] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    with pytest.raises(MissionConfigError):
        parse_mission(raw)


def test_an_enter_door_step_requires_a_timeout_and_refuses_on_object():
    raw = door_mission()
    del raw['moves'][0]['timeout_sec']
    with pytest.raises(MissionConfigError):
        parse_mission(raw)
    raw = door_mission()
    raw['moves'][0]['on_object'] = [{'class': 'person', 'action': 'stop_and_hold'}]
    with pytest.raises(MissionConfigError):
        parse_mission(raw)


# ---------------------------------------------------------------------------
# The behaviour
# ---------------------------------------------------------------------------
# f1tenth_messages comes from the BUILT workspace, so the two tests that reach
# it skip the import inside their own body rather than at module level: a
# module-level importorskip takes the schema tests above down with it on a
# machine with no install space, and those need nothing but this package.


def test_door_entered_only_fires_on_an_enter_door_step():
    """condition_eval's own gate, independent of the behaviour."""
    from f1tenth_behavior.mission.condition_eval import EvalContext, evaluate
    from f1tenth_behavior.mission.mission_config import StopCondition
    cond = StopCondition(type='door_entered')
    base = dict(now=0.0, move_start_time=0.0, move_start_xy=None, current_xy=None,
                detected_classes={}, min_obstacle_distance=None, front_clearance=None,
                default_distance=None)
    assert evaluate(cond, EvalContext(**base)) is False            # not tracked
    assert evaluate(cond, EvalContext(**base, door_entered=False)) is False
    assert evaluate(cond, EvalContext(**base, door_entered=True)) is True


def test_select_door_and_wrap_deg():
    pytest.importorskip('f1tenth_messages.msg', reason='needs the built workspace')
    from f1tenth_behavior.behaviours.enter_door import select_door, wrap_deg
    doors = {3: ((1.0, 0.0), 0.0), 9: ((5.0, 0.0), math.pi)}
    assert select_door(doors, 'nearest', (0.0, 0.0)) == 3
    assert select_door(doors, 'nearest', (6.0, 0.0)) == 9
    assert select_door(doors, 'door_9', (0.0, 0.0)) == 9
    assert select_door(doors, 'door_4', (0.0, 0.0)) is None
    assert select_door({}, 'nearest', (0.0, 0.0)) is None
    assert select_door(doors, 'nearest', None) is None
    assert wrap_deg(math.radians(-170.0) - math.radians(170.0)) == pytest.approx(20.0)


def test_confirmed_doors_skips_candidates():
    pytest.importorskip('f1tenth_messages.msg', reason='needs the built workspace')
    from f1tenth_messages.msg import Door, DoorArray
    from f1tenth_behavior.behaviours.enter_door import confirmed_doors
    msg = DoorArray()
    good = Door(id=2, state=Door.STATE_CONFIRMED, width=0.9)
    good.center.x, good.center.y = 1.0, 2.0
    good.normal.x, good.normal.y = 0.0, 1.0
    bad = Door(id=3, state=Door.STATE_CANDIDATE, width=0.9)
    msg.doors = [good, bad]
    out = confirmed_doors(msg)
    assert set(out) == {2}
    assert out[2][0] == (1.0, 2.0)
    assert out[2][1] == pytest.approx(math.pi / 2.0)
