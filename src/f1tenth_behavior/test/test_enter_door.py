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
from types import SimpleNamespace
from unittest.mock import MagicMock

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


# ---------------------------------------------------------------------------
# The approach pre-point
# ---------------------------------------------------------------------------
def _door_scene():
    """A door at (0, 3) in map, crossing direction +y: x_door = y_map - 3."""
    pytest.importorskip('f1tenth_messages.msg', reason='needs the built workspace')
    from f1tenth_behavior.behaviours import enter_door as ed
    return ed, (0.0, 3.0), math.pi / 2.0


def test_the_pre_point_sits_back_along_the_door_axis():
    ed, centre, yaw = _door_scene()
    assert ed.pre_point(centre, yaw, 0.8) == pytest.approx((0.0, 2.2), abs=1e-9)
    assert ed.pre_point(centre, yaw, 1.5) == pytest.approx((0.0, 1.5), abs=1e-9)
    # A door facing -x: the pre-point is on the +x side of it.
    assert ed.pre_point((4.0, 0.0), math.pi, 0.8) == pytest.approx((4.8, 0.0), abs=1e-9)


def test_door_frame_x_is_how_far_through_the_opening():
    ed, centre, yaw = _door_scene()
    assert ed.door_frame_xy((0.0, 1.0), centre, yaw)[0] == pytest.approx(-2.0)
    assert ed.door_frame_xy((0.0, 3.0), centre, yaw)[0] == pytest.approx(0.0)
    assert ed.door_frame_xy((0.0, 4.0), centre, yaw)[0] == pytest.approx(1.0)
    # y is the sideways offset from the opening's centre line.
    assert ed.door_frame_xy((0.5, 3.0), centre, yaw)[1] == pytest.approx(-0.5)


@pytest.mark.parametrize('car_y, expect_phase', [
    (0.0, 'APPROACH'),     # 3 m short: line up first
    (2.0, 'APPROACH'),     # x_door = -1.0, still short of the skip threshold
    (2.85, 'ENTER'),       # x_door = -0.15, past APPROACH_SKIP_X_M: in the doorway
    (3.5, 'ENTER'),        # already through it
])
def test_the_pre_point_is_skipped_when_the_car_is_already_in_the_doorway(
        car_y, expect_phase):
    ed, centre, yaw = _door_scene()
    x_door = ed.door_frame_xy((0.0, car_y), centre, yaw)[0]
    phase = ed.PHASE_ENTER if x_door > ed.APPROACH_SKIP_X_M else ed.PHASE_APPROACH
    assert phase == expect_phase


def test_the_phase_walks_approach_then_enter_and_entered_waits_for_it():
    """The whole move, driven tick by tick against a fake node."""
    ed, centre, yaw = _door_scene()
    from f1tenth_messages.msg import Door, DoorArray
    from f1tenth_behavior.mission.mission_config import parse_mission
    from f1tenth_behavior.mission.runtime import (
        GLOBAL_XY_KEY, GLOBAL_YAW_KEY, MISSION_KEY, MissionRuntimeState)

    config = parse_mission(door_mission())
    state = MissionRuntimeState()
    state.config = config
    state.current_index = 0

    behaviour = ed.EnterDoor(clock=lambda: behaviour._fake_now)
    behaviour._fake_now = 0.0
    behaviour.node = MagicMock()
    behaviour.goal_pose_pub = MagicMock()
    behaviour.hold_pub = MagicMock()
    bb = {MISSION_KEY: state, GLOBAL_XY_KEY: (0.0, 0.0), GLOBAL_YAW_KEY: yaw}
    behaviour.blackboard = SimpleNamespace(**bb)
    behaviour._bb = lambda key: getattr(behaviour.blackboard, key, None)

    msg = DoorArray()
    door = Door(id=4, state=Door.STATE_CONFIRMED, width=0.9)
    door.center.x, door.center.y = centre
    door.normal.x, door.normal.y = math.cos(yaw), math.sin(yaw)
    msg.doors = [door]
    behaviour.doors = ed.confirmed_doors(msg)
    behaviour.doors_received_sec = 0.0

    def tick(car_xy, car_yaw=yaw):
        behaviour.blackboard.mission_global_xy = car_xy
        behaviour.blackboard.mission_global_yaw = car_yaw
        behaviour._fake_now += 0.1
        return behaviour.update()

    # Far short of the door: APPROACH, aiming at the pre-point, not entered.
    tick((0.0, 0.0))
    record = state.door_record
    assert record.frame_id == 'door_4'
    assert record.phase == 'APPROACH'
    assert record.pre_point_xy == pytest.approx((0.0, 2.2), abs=1e-9)
    assert record.target_xy == pytest.approx((0.0, 2.2), abs=1e-9)
    assert record.entered is False
    # ...and the errors it reports are against the DOOR, not the pre-point.
    assert record.xy_error_m == pytest.approx(3.0)

    # At the pre-point: hand over to ENTER, now aiming at the origin.
    tick((0.0, 2.2))
    assert record.phase == 'ENTER'
    assert record.target_xy == pytest.approx(centre, abs=1e-9)
    assert record.entered is False          # 0.8 m from the door still

    # In the doorway, square to it: entered.
    tick((0.0, 2.95))
    assert record.entered is True
    assert record.xy_error_m == pytest.approx(0.05, abs=1e-9)
    assert record.yaw_error_deg == pytest.approx(0.0, abs=1e-9)

    # Turned 30 deg off the crossing direction: inside xy_tol, not entered.
    tick((0.0, 2.95), car_yaw=yaw + math.radians(30.0))
    assert record.yaw_error_deg == pytest.approx(30.0, abs=1e-6)
    assert record.entered is False


def test_a_car_that_starts_in_the_doorway_never_leaves_to_line_up():
    ed, centre, yaw = _door_scene()
    from f1tenth_messages.msg import Door, DoorArray
    from f1tenth_behavior.mission.mission_config import parse_mission
    from f1tenth_behavior.mission.runtime import (
        GLOBAL_XY_KEY, GLOBAL_YAW_KEY, MISSION_KEY, MissionRuntimeState)

    state = MissionRuntimeState()
    state.config = parse_mission(door_mission())
    behaviour = ed.EnterDoor(clock=lambda: 1.0)
    behaviour.node = MagicMock()
    behaviour.goal_pose_pub = MagicMock()
    behaviour.hold_pub = MagicMock()
    behaviour.blackboard = SimpleNamespace(**{
        MISSION_KEY: state, GLOBAL_XY_KEY: (0.0, 2.95), GLOBAL_YAW_KEY: yaw})
    behaviour._bb = lambda key: getattr(behaviour.blackboard, key, None)

    msg = DoorArray()
    door = Door(id=1, state=Door.STATE_CONFIRMED, width=0.9)
    door.center.x, door.center.y = centre
    door.normal.x, door.normal.y = math.cos(yaw), math.sin(yaw)
    msg.doors = [door]
    behaviour.doors = ed.confirmed_doors(msg)
    behaviour.doors_received_sec = 0.0

    behaviour.update()
    record = state.door_record
    assert record.phase == 'ENTER'
    # It never aimed at the pre-point, which is 0.75 m BEHIND where it stands.
    assert record.target_xy == pytest.approx(centre, abs=1e-9)
    assert record.entered is True


# -------------------------------------------------------------------- preflight
def test_the_preflight_requires_the_node_door_source_names():
    """Three detectors publish /perception/doors_semantic; one of them runs.

    The topic and the message are identical whichever one it is -- which is
    why EnterDoor itself has no door_source branch at all -- but the node that
    has to be ALIVE for that topic to exist is different for each, so the
    preflight's node_name has to follow door_source rather than naming
    semantic_layer_node unconditionally (which it did until the grid detector
    landed, and which would have let an enter_door mission start with
    door_source 'freespace' and freespace_door_node absent).
    """
    from unittest.mock import patch

    from f1tenth_behavior.mission.preflight import (
        DOOR_SOURCE_NODES, required_dependencies)
    from f1tenth_params.param_defaults import get_value as real_get_value

    plain = {'id': 'd', 'goal_distance': 1.0,
             'stop_condition': {'type': 'distance_reached'}, 'terminal': True}
    for source, node_name in DOOR_SOURCE_NODES.items():
        with patch('f1tenth_behavior.mission.preflight.get_value',
                   side_effect=lambda key, s=source: (
                       s if key == 'door_source' else real_get_value(key))):
            reqs = required_dependencies(parse_mission(door_mission()))
            assert any(r.node_name == node_name for r in reqs), source
            assert any(source in r.reason for r in reqs), source
            # And a mission with no enter_door step asks for no detector.
            other = required_dependencies(parse_mission(_mission_of(plain)))
            assert not any(r.node_name == node_name for r in other), source


def _mission_of(move):
    raw = door_mission()
    raw['moves'] = [move]
    return raw
