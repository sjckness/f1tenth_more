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
    spec = {'type': 'enter_door', 'door': 'nearest', 'along_tol': 0.15,
            'lateral_tol': 0.05, 'yaw_tol_deg': 10}
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
    assert (spec.door, spec.along_tol, spec.lateral_tol, spec.yaw_tol_deg) == (
        'nearest', 0.15, 0.05, 10.0)
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


def test_left_and_right_parse_as_door_selectors():
    assert parse_mission(door_mission(door='left')).moves[0].enter_door.door == 'left'
    assert parse_mission(door_mission(door='right')).moves[0].enter_door.door == 'right'


def test_the_tolerances_are_validated():
    for bad in (0.0, -0.1):
        with pytest.raises(MissionConfigError):
            parse_mission(door_mission(along_tol=bad))
        with pytest.raises(MissionConfigError):
            parse_mission(door_mission(lateral_tol=bad))
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


def test_left_and_right_resolve_two_symmetric_doors_nearest_cannot_tell_apart():
    """Two doors facing each other across a corridor, equally near the car.

    The real case this selector exists for: 'nearest' alone is a coin flip
    between them (same distance, to within float noise), but left/right is
    not -- the car's own heading splits them cleanly.
    """
    pytest.importorskip('f1tenth_messages.msg', reason='needs the built workspace')
    from f1tenth_behavior.behaviours.enter_door import select_door, side_of
    left_door, right_door = 1, 2
    doors = {left_door: ((5.0, 2.0), 0.0), right_door: ((5.0, -2.0), math.pi)}
    car_xy, car_yaw = (0.0, 0.0), 0.0
    assert side_of(doors[left_door][0], car_xy, car_yaw) == 'left'
    assert side_of(doors[right_door][0], car_xy, car_yaw) == 'right'
    assert select_door(doors, 'left', car_xy, car_yaw) == left_door
    assert select_door(doors, 'right', car_xy, car_yaw) == right_door
    # Turned all the way around: what was ahead-left is now behind-right,
    # and vice versa -- unlike a partial turn, 180 deg swaps both cleanly
    # because the two doors sit symmetric about the car's original heading.
    assert select_door(doors, 'right', car_xy, math.pi) == left_door
    assert select_door(doors, 'left', car_xy, math.pi) == right_door
    # No car pose, or nothing on that side at all: None, not a wrong guess.
    assert select_door(doors, 'left', None, car_yaw) is None
    assert select_door(doors, 'left', car_xy, None) is None
    assert select_door({left_door: doors[left_door]}, 'right', car_xy, car_yaw) is None


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


def test_a_single_radius_would_have_let_a_jamb_hit_read_as_entered():
    """The along/lateral split's whole reason to exist.

    A car sitting 0.12 m sideways of the axis (well past lateral_tol's
    0.05 m -- flush against a jamb, not through the middle) and only 0.09 m
    short along it: hypot(0.12, 0.09) == 0.15, exactly the OLD single xy_tol
    radius -- so the pre-split check would have called this "entered". The
    split correctly refuses it on lateral alone.
    """
    ed, centre, yaw = _door_scene()
    along_tol, lateral_tol = 0.15, 0.05
    car_xy = (centre[0] - 0.12, centre[1] - 0.09)  # lateral=0.12, along=-0.09
    along, lateral = ed.door_frame_xy(car_xy, centre, yaw)
    assert along == pytest.approx(-0.09, abs=1e-9)
    assert lateral == pytest.approx(0.12, abs=1e-9)

    old_single_radius_would_enter = math.hypot(along, lateral) <= 0.15
    assert old_single_radius_would_enter is True   # the bug this replaces

    new_split_entered = abs(along) <= along_tol and abs(lateral) <= lateral_tol
    assert new_split_entered is False              # the fix


def test_within_lateral_tol_but_short_along_is_also_not_entered():
    """The split cuts both ways: good lateral alignment alone is not enough
    either -- the car still has to have actually reached the opening."""
    ed, centre, yaw = _door_scene()
    along_tol, lateral_tol = 0.15, 0.05
    car_xy = (centre[0], centre[1] - 0.5)   # dead on-axis, 0.5 m short
    along, lateral = ed.door_frame_xy(car_xy, centre, yaw)
    assert lateral == pytest.approx(0.0, abs=1e-9)
    assert abs(along) > along_tol              # still 0.5 m short
    assert not (abs(along) <= along_tol and abs(lateral) <= lateral_tol)


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


def test_the_phase_walks_approach_enter_exit_and_entered_waits_for_the_far_side():
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
    # STRONG, because selection resolves over strong doors only -- these
    # tests are about the approach phases, not about earning selection.
    door.strong = True
    msg.doors = [door]
    behaviour.doors = ed.confirmed_doors(msg)
    behaviour.strong = ed.strong_doors(msg)
    behaviour.doors_received_sec = 0.0

    def tick(car_xy, car_yaw=yaw):
        behaviour.blackboard.mission_global_xy = car_xy
        behaviour.blackboard.mission_global_yaw = car_yaw
        behaviour._fake_now += 0.1
        return behaviour.update()

    # Far short of the door: APPROACH along a (straight) Dubins path to the
    # entry pose 0.5 m back on the axis, aiming one lookahead ahead.
    tick((0.0, 0.0))
    record = state.door_record
    assert record.frame_id == 'door_4'
    assert record.phase == 'APPROACH'
    assert record.pre_point_xy == pytest.approx((0.0, 2.5), abs=1e-9)
    assert record.target_xy == pytest.approx((0.0, behaviour.lookahead_m), abs=1e-6)
    assert record.entered is False
    # ...and the errors it reports are against the DOOR, not the entry pose.
    assert record.along_error_m == pytest.approx(-3.0)
    assert record.lateral_error_m == pytest.approx(0.0, abs=1e-9)

    # Within 0.3 m of the entry pose: rule (a) hands over to ENTER (the centre).
    tick((0.0, 2.3))
    assert record.phase == 'ENTER'
    assert record.target_xy == pytest.approx(centre, abs=1e-9)
    assert record.entered is False

    # Within 0.3 m of the centre: ONE exit goal 0.8 m beyond it.
    tick((0.0, 2.8))
    assert record.phase == 'EXIT'
    assert record.target_xy == pytest.approx((0.0, 3.8), abs=1e-9)
    assert record.entered is False          # still in the doorway
    sent = behaviour.goal_pose_pub.publish.call_count

    # Through: along >= 0.8 - 0.2. The heading is not part of it any more.
    tick((0.0, 3.65), car_yaw=yaw + math.radians(30.0))
    assert record.entered is True
    assert record.yaw_error_deg == pytest.approx(30.0, abs=1e-6)
    assert behaviour.goal_pose_pub.publish.call_count == sent    # nothing after the exit


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
    door.strong = True
    msg.doors = [door]
    behaviour.doors = ed.confirmed_doors(msg)
    behaviour.strong = ed.strong_doors(msg)
    behaviour.doors_received_sec = 0.0

    behaviour.update()
    record = state.door_record
    # In the doorway already, 5 cm from the centre: straight to the exit goal.
    assert record.phase == 'EXIT'
    sent = [_goal_of(c) for c in behaviour.goal_pose_pub.publish.call_args_list]
    assert sent == [pytest.approx((0.0, 3.8, yaw), abs=1e-9)]
    assert record.entered is False


def _goal_of(call):
    m = call.args[0]
    q = m.pose.orientation
    return (m.pose.position.x, m.pose.position.y, 2.0 * math.atan2(q.z, q.w))


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


# ---------------------------------------------------------------------------
# Only a STRONG door may be selected
# ---------------------------------------------------------------------------
def _door_msg(door_id, xy, strong, state=None):
    from f1tenth_messages.msg import Door
    d = Door(id=door_id, state=Door.STATE_CONFIRMED if state is None else state,
             width=0.9)
    d.center.x, d.center.y = float(xy[0]), float(xy[1])
    d.normal.x, d.normal.y = 0.0, 1.0
    d.strong = bool(strong)
    return d


def test_strong_doors_filters_and_confirmed_doors_does_not():
    """Two lists on purpose: strength gates SELECTION, not following.

    Once a door is resolved the move follows it through the confirmed list,
    so a door that dips below strong mid-approach does not read as
    `door_lost` and the car finishes the approach it committed to.
    """
    pytest.importorskip('f1tenth_messages.msg', reason='needs the built workspace')
    from f1tenth_messages.msg import DoorArray
    from f1tenth_behavior.behaviours.enter_door import confirmed_doors, strong_doors
    msg = DoorArray()
    msg.doors = [_door_msg(2, (1.0, 2.0), True), _door_msg(3, (0.5, 2.0), False)]
    assert set(confirmed_doors(msg)) == {2, 3}
    assert set(strong_doors(msg)) == {2}


def test_nearest_skips_a_closer_door_that_is_not_strong():
    """The case that makes this more than a filter: door 3 is NEARER.

    Resolving "nearest" over the confirmed list would pick a door the
    detector has not vouched for, and the car would drive at it.
    """
    pytest.importorskip('f1tenth_messages.msg', reason='needs the built workspace')
    from f1tenth_messages.msg import DoorArray
    from f1tenth_behavior.behaviours import enter_door as ed
    msg = DoorArray()
    msg.doors = [_door_msg(2, (3.0, 0.0), True), _door_msg(3, (1.0, 0.0), False)]
    car = (0.0, 0.0)
    assert ed.select_door(ed.confirmed_doors(msg), 'nearest', car) == 3
    assert ed.select_door(ed.strong_doors(msg), 'nearest', car) == 2
    # And a door named outright is subject to the same rule.
    assert ed.select_door(ed.strong_doors(msg), 'door_3', car) is None
    assert ed.select_door(ed.strong_doors(msg), 'door_2', car) == 2


def test_a_confirmed_but_not_strong_door_fails_with_no_strong_door():
    """And an EMPTY topic still fails with door_not_found.

    The two reasons answer different questions -- "there is a door there but
    it has not earned it" versus "there is no such door" -- and they lead to
    different things to do about it, so they must not be collapsed.
    """
    pytest.importorskip('f1tenth_messages.msg', reason='needs the built workspace')
    from f1tenth_messages.msg import DoorArray
    from f1tenth_behavior.behaviours import enter_door as ed
    from f1tenth_behavior.mission.runtime import (
        GLOBAL_XY_KEY, GLOBAL_YAW_KEY, MISSION_KEY, MissionRuntimeState)

    def run(doors):
        state = MissionRuntimeState()
        state.config = parse_mission(door_mission())
        clock = [0.0]
        b = ed.EnterDoor(clock=lambda: clock[0])
        b.node = MagicMock()
        b.goal_pose_pub = MagicMock()
        b.hold_pub = MagicMock()
        b.blackboard = SimpleNamespace(**{
            MISSION_KEY: state, GLOBAL_XY_KEY: (0.0, 0.0), GLOBAL_YAW_KEY: 0.0})
        b._bb = lambda key: getattr(b.blackboard, key, None)
        msg = DoorArray()
        msg.doors = doors
        b._doors_cb(msg)
        b.update()                                    # resolves, or starts the clock
        clock[0] = state.config.moves[0].enter_door.acquire_timeout_sec + 1.0
        b.update()                                    # past acquire_timeout_sec
        return state

    not_strong = run([_door_msg(3, (0.0, 3.0), False)])
    assert not_strong.door_record.outcome == 'no_strong_door'
    assert not_strong.last_stop_reason == 'enter_door:no_strong_door'

    nothing = run([])
    assert nothing.door_record.outcome == 'door_not_found'
    assert nothing.last_stop_reason == 'enter_door:door_not_found'

    strong = run([_door_msg(3, (0.0, 3.0), True)])
    assert strong.door_record.outcome is None
    assert strong.door_record.door_id == 3


# ---------------------------------------------------------------------------
# Tolerances: one source, never tighter than mpc_corr's 0.15 m arrival latch
# ---------------------------------------------------------------------------
def test_phase_and_entered_tolerances_match_what_the_mpc_can_deliver():
    from f1tenth_behavior.behaviours import enter_door as ed
    # Default lateral budget: clamp(0.3 * (width - 0.272 car), 0.10, 0.25).
    assert ed.lateral_tol(0.9) == pytest.approx(0.3 * (0.9 - 0.272))   # 0.188
    assert ed.lateral_tol(1.5) == pytest.approx(0.25)                  # upper clamp
    assert ed.lateral_tol(0.5) == pytest.approx(0.10)                  # lower clamp
    assert ed.lateral_tol(0.9, override_m=0.1) == pytest.approx(0.1)
    assert ed.lateral_tol(1.0, car_width_m=0.4) == pytest.approx(0.18)  # car width is an input
    lat, yaw_tol, along_tol, dist = ed.lateral_tol(0.9), 20.0, 0.15, 0.8

    # Entered: the MPC latches within 0.15 m of the centre, so 0.12 m short counts.
    assert ed.has_entered(-0.12, 0.10, 18.0, along_tol, lat, yaw_tol)
    assert ed.has_entered(0.6, 0.0, 0.0, along_tol, lat, yaw_tol)       # well through
    assert not ed.has_entered(-0.4, 0.0, 0.0, along_tol, lat, yaw_tol)  # not there yet
    assert not ed.has_entered(0.0, 0.22, 0.0, along_tol, lat, yaw_tol)
    assert not ed.has_entered(0.0, 0.0, 30.0, along_tol, lat, yaw_tol)


def test_goal_yaw_is_the_bearing_clamped_to_the_axis_plus_minus_tolerance():
    from f1tenth_behavior.behaviours import enter_door as ed
    axis = math.radians(90.0)           # the door's crossing direction is +y
    goal = (0.0, 5.0)
    tol = 20.0
    # Car 45 deg off the line (bearing to the goal is 45 deg from the axis):
    # clamped to axis + tol, on the side of the bearing.
    car = (-5.0, 0.0)                   # bearing to goal = atan2(5, 5) = 45 deg
    assert math.degrees(ed.goal_yaw_for(car, goal, axis, tol)) == pytest.approx(90.0 - tol)
    car = (5.0, 0.0)                    # bearing 135 deg: other side
    assert math.degrees(ed.goal_yaw_for(car, goal, axis, tol)) == pytest.approx(90.0 + tol)
    # Car inside the tolerance (bearing 10 deg off the axis): the bearing itself.
    car = (-0.88, 0.0)
    bearing = math.atan2(goal[1] - car[1], goal[0] - car[0])
    assert abs(math.degrees(bearing) - 90.0) < tol
    assert ed.goal_yaw_for(car, goal, axis, tol) == pytest.approx(bearing)
    # Wrapped: an axis at 175 deg and a bearing at -170 deg are 15 deg apart.
    axis = math.radians(175.0)
    car, goal = (0.0, 0.0), (math.cos(math.radians(-170.0)), math.sin(math.radians(-170.0)))
    assert ed.goal_yaw_for(car, goal, axis, tol) == pytest.approx(math.radians(-170.0))
    # Right on top of the goal the bearing is noise: the axis.
    assert ed.goal_yaw_for((0.0, 4.95), (0.0, 5.0), math.radians(90.0), tol) == pytest.approx(
        math.radians(90.0))


def test_the_score_report_quotes_the_tolerances_actually_applied():
    from f1tenth_behavior.mission import move_scoring
    from f1tenth_behavior.mission.runtime import DoorApproachRecord
    move = SimpleNamespace(id='m1', enter_door=SimpleNamespace(
        along_tol=0.15, lateral_tol=0.05, yaw_tol_deg=10.0, door='left'))
    rec = DoorApproachRecord(door='left', frame_id='door_3', along_error_m=-0.1,
                             lateral_error_m=0.09, yaw_error_deg=12.0,
                             lat_tol_m=0.15, yaw_tol_deg=20.0)
    out = move_scoring._door_outcome(move, 'door_entered', 0.0, 1.0, None, None,
                                     None, None, rec)
    assert '(tol 0.150), yaw error=12.0 deg (tol 20.0)' in out.note
    assert '0.050' not in out.note
    assert out.commanded == pytest.approx(0.15)


def _frozen_rig(centre_shift=0.0):
    ed, centre, yaw = _door_scene()
    from f1tenth_messages.msg import Door, DoorArray
    from f1tenth_behavior.mission.mission_config import parse_mission
    from f1tenth_behavior.mission.runtime import (
        GLOBAL_XY_KEY, GLOBAL_YAW_KEY, MISSION_KEY, MissionRuntimeState)
    state = MissionRuntimeState()
    state.config = parse_mission(door_mission())
    state.current_index = 0
    b = ed.EnterDoor(clock=lambda: b._fake_now)
    b._fake_now = 0.0
    b.node, b.goal_pose_pub, b.hold_pub = MagicMock(), MagicMock(), MagicMock()
    b.blackboard = SimpleNamespace(**{MISSION_KEY: state, GLOBAL_XY_KEY: (0.0, 0.0),
                                      GLOBAL_YAW_KEY: yaw})
    b._bb = lambda key: getattr(b.blackboard, key, None)

    def set_door(cx, cy, axis=yaw, present=True):
        msg = DoorArray()
        if present:
            d = Door(id=4, state=Door.STATE_CONFIRMED, width=0.9)
            d.center.x, d.center.y = cx, cy
            d.normal.x, d.normal.y = math.cos(axis), math.sin(axis)
            d.strong = True
            msg.doors = [d]
        b.doors, b.strong = ed.confirmed_doors(msg), ed.strong_doors(msg)
        b.widths = ed.door_widths(msg)
        b.doors_received_sec = 0.0

    def tick(car_xy, car_yaw=yaw):
        b.blackboard.mission_global_xy = car_xy
        b.blackboard.mission_global_yaw = car_yaw
        b._fake_now += 0.1
        return b.update()
    return ed, b, state, centre, set_door, tick


def test_enter_freezes_the_door_pose_and_sends_its_goal_once():
    ed, b, state, centre, set_door, tick = _frozen_rig()
    set_door(*centre)
    tick((0.0, 0.0))                      # APPROACH
    tick((0.0, 2.3))                      # 0.2 m from the entry pose -> ENTER
    rec = state.door_record
    assert rec.phase == 'ENTER'
    sent = b.goal_pose_pub.publish.call_count
    assert rec.target_xy == pytest.approx(centre, abs=1e-9)
    # the detector refines the centre by 0.3 m and the axis by 20 deg: ignored
    set_door(centre[0] + 0.3, centre[1] + 0.2, axis=math.radians(110.0))
    tick((0.0, 2.5))
    assert rec.target_xy == pytest.approx(centre, abs=1e-9)
    assert rec.target_yaw == pytest.approx(math.radians(90.0), abs=1e-9)
    assert b.goal_pose_pub.publish.call_count == sent        # no republish
    # track lost: hold the frozen pose, warn, do not fail the move
    set_door(0, 0, present=False)
    b._fake_now += 10.0                                      # past LOST_GRACE_SEC
    status = tick((0.0, 2.6))
    assert status.name == 'SUCCESS' and rec.outcome is None
    assert rec.target_xy == pytest.approx(centre, abs=1e-9)
    assert b.node.get_logger().warn.called
    assert b.goal_pose_pub.publish.call_count == sent


def test_enter_door_ignores_a_closer_door_that_is_not_strong():
    ed, b, state, centre, set_door, tick = _frozen_rig()
    from f1tenth_messages.msg import Door, DoorArray
    msg = DoorArray()
    near = Door(id=9, state=Door.STATE_CONFIRMED, width=0.9)      # 1 m ahead
    near.center.x, near.center.y = 0.0, 1.0
    near.normal.x, near.normal.y = 0.0, 1.0
    near.strong = False
    far = Door(id=4, state=Door.STATE_CONFIRMED, width=0.9)       # 3 m ahead
    far.center.x, far.center.y = centre
    far.normal.x, far.normal.y = 0.0, 1.0
    far.strong = True
    msg.doors = [near, far]
    b.doors, b.strong = ed.confirmed_doors(msg), ed.strong_doors(msg)
    b.widths = ed.door_widths(msg)
    b.doors_received_sec = 0.0
    tick((0.0, 0.0))
    assert state.door_record.door_id == 4
    # with no strong door at all: no goal, a throttled warning, no failure yet
    ed2, b2, state2, _, set_door2, tick2 = _frozen_rig()
    msg2 = DoorArray()
    msg2.doors = [near]
    b2.doors, b2.strong = ed2.confirmed_doors(msg2), ed2.strong_doors(msg2)
    b2.widths = ed2.door_widths(msg2)
    b2.doors_received_sec = 0.0
    tick2((0.0, 0.0))
    assert state2.door_record.door_id is None
    assert not b2.goal_pose_pub.publish.called
    assert b2.node.get_logger().warn.called


# ---------------------------------------------------------------------------
# The Dubins approach, the exit goal and the reverse
# ---------------------------------------------------------------------------
R_MIN = 0.305 / math.tan(0.319)        # wheelbase / max(abs(steering bounds)): 0.923 m


def _free_grid(occupied=(), size=12.0, res=0.05, origin=(-6.0, -6.0)):
    import numpy as np
    from f1tenth_behavior.behaviours import dubins as db
    n = int(round(size / res))
    data = np.zeros((n, n), dtype=np.int16)
    for x0, y0, x1, y1 in occupied:
        i0, i1 = int((x0 - origin[0]) / res), int(math.ceil((x1 - origin[0]) / res))
        j0, j1 = int((y0 - origin[1]) / res), int(math.ceil((y1 - origin[1]) / res))
        data[max(j0, 0):max(j1, 0), max(i0, 0):max(i1, 0)] = 100
    return db.OccGrid(data, res, origin[0], origin[1])


def _dubins_rig(door_xy, axis, car, grid=None):
    """Build an EnterDoor on the shipped geometry (R_min 0.923, rho 1.016, lookahead 1.385)."""
    pytest.importorskip('f1tenth_messages.msg', reason='needs the built workspace')
    from f1tenth_messages.msg import Door, DoorArray
    from f1tenth_behavior.behaviours import enter_door as ed
    from f1tenth_behavior.mission.runtime import (
        GLOBAL_XY_KEY, GLOBAL_YAW_KEY, MISSION_KEY, MissionRuntimeState)
    state = MissionRuntimeState()
    state.config = parse_mission(door_mission())
    state.current_index = 0
    b = ed.EnterDoor(clock=lambda: b._fake_now)
    b._fake_now = 0.0
    b.node, b.goal_pose_pub, b.hold_pub = MagicMock(), MagicMock(), MagicMock()
    b.r_min_m = R_MIN
    b.lookahead_m = max(1.0, 1.5 * R_MIN)
    b.grid = grid if grid is not None else _free_grid()
    b.blackboard = SimpleNamespace(**{MISSION_KEY: state, GLOBAL_XY_KEY: car[:2],
                                      GLOBAL_YAW_KEY: car[2]})
    b._bb = lambda key: getattr(b.blackboard, key, None)
    msg = DoorArray()
    d = Door(id=7, state=Door.STATE_CONFIRMED, width=0.9)
    d.center.x, d.center.y = door_xy
    d.normal.x, d.normal.y = math.cos(axis), math.sin(axis)
    d.strong = True
    msg.doors = [d]
    b.doors, b.strong, b.widths = ed.confirmed_doors(msg), ed.strong_doors(msg), \
        ed.door_widths(msg)
    b.doors_received_sec = 0.0

    def tick(pose=None, dt=0.1):
        if pose is not None:
            b.blackboard.mission_global_xy = (pose[0], pose[1])
            b.blackboard.mission_global_yaw = pose[2]
        b._fake_now += dt
        return b.update()
    return ed, b, state, tick


def _logged(b, level='info'):
    return [str(c.args[0]) for c in getattr(b.node.get_logger(), level).call_args_list]


def test_run_15_42_00_geometry_gets_a_free_path_that_ends_on_the_axis():
    """2026-11-06T15-42-00: car (1.93, 0.25, 12 deg), door (4.13, 1.83), axis ~80 deg."""
    door, axis = (4.13, 1.83), math.radians(80.0)
    car = (1.93, 0.25, math.radians(12.0))
    ed, b, state, tick = _dubins_rig(door, axis, car)
    tick()
    rec = state.door_record
    assert rec.phase == 'APPROACH'
    plan = b._plan
    assert plan is not None and plan.feasible
    path = plan.path
    assert path.word == 'RSL'
    assert path.length == pytest.approx(2.557, abs=0.005)
    assert b.rho == pytest.approx(1.1 * R_MIN)
    end = path.end()
    along, lateral = ed.door_frame_xy(end[:2], door, axis)
    assert along == pytest.approx(-0.5, abs=0.02)
    assert abs(lateral) < 0.02
    assert abs(ed.wrap_deg(end[2] - axis)) < 1.0
    assert any('PLAN' in line and 'RSL' in line for line in _logged(b))

    # Drive the car along the plan: goals advance, ENTER fires by (a) or (b),
    # never before along -0.8.
    goals_s = []
    switched_at = None
    s = 0.0
    while s < path.length + 0.5 and rec.phase == 'APPROACH':
        s += 0.1
        pose = path.pose_at(s)
        n_before = b.goal_pose_pub.publish.call_count
        tick(pose)
        if b.goal_pose_pub.publish.call_count > n_before and rec.phase == 'APPROACH':
            g = _goal_of(b.goal_pose_pub.publish.call_args)
            goals_s.append(path.project(g[0], g[1])[0])
        if rec.phase == 'ENTER':
            switched_at = ed.door_frame_xy(pose[:2], door, axis)
    assert len(goals_s) >= 3
    assert all(b2 > a2 for a2, b2 in zip(goals_s, goals_s[1:]))
    assert switched_at is not None and switched_at[0] >= -0.8
    assert any('SWITCH' in line and ('rule (a)' in line or 'rule (b)' in line)
               for line in _logged(b))
    assert not b.hold_pub.publish.called


def test_the_exit_goal_goes_out_once_and_nothing_is_ever_sent_behind_the_car():
    door, axis = (4.13, 1.83), math.radians(80.0)
    ed, b, state, tick = _dubins_rig(door, axis, (1.93, 0.25, math.radians(12.0)))
    tick()
    path = b._plan.path
    s = 0.0
    while state.door_record.phase == 'APPROACH':
        s += 0.1
        tick(path.pose_at(s))
    # ENTER: drive down the axis through the door.
    for along in [x / 20.0 for x in range(-10, 21)]:
        pose = (door[0] + along * math.cos(axis), door[1] + along * math.sin(axis), axis)
        n = b.goal_pose_pub.publish.call_count
        tick(pose)
        for c in b.goal_pose_pub.publish.call_args_list[n:]:
            g = _goal_of(c)
            # every goal is ahead of the car along the door axis
            assert ed.door_frame_xy(g[:2], door, axis)[0] > along
    exits = [c for c in b.goal_pose_pub.publish.call_args_list
             if ed.door_frame_xy(_goal_of(c)[:2], door, axis)[0] > 0.5]
    assert len(exits) == 1
    gx = _goal_of(exits[0])
    assert ed.door_frame_xy(gx[:2], door, axis)[0] == pytest.approx(0.8, abs=1e-9)
    assert state.door_record.phase == 'EXIT'
    assert state.door_record.entered is True
    assert sum('EXIT' in line for line in _logged(b)) == 1


def test_the_passed_the_pre_point_rule_is_gone():
    """
    Do not switch to ENTER just for being past the old pre-point.

    1.0 m short (past the old 1.24 m pre-point), 0.5 m off the axis, square:
    the old rule sent this car to ENTER; now it is no approach at all.
    """
    door, axis = (0.0, 0.0), 0.0
    ed, b, state, tick = _dubins_rig(door, axis, (-1.0, 0.5, 0.0))
    assert not hasattr(ed, 'at_pre_point')
    tick()
    assert state.door_record.phase == 'APPROACH'
    assert state.door_record.phase != 'ENTER'
    assert ed.switch_to_enter((-1.0, 0.5), 0.0, ed.entry_pose(door, axis, 0.5), door, axis,
                              0.5, 0.19, 20.0) == ''
    # rule (b) needs all three: past the entry along, on the axis, square
    entry = ed.entry_pose(door, axis, 0.5)
    assert ed.switch_to_enter((-0.1, 0.25), 0.1, entry, door, axis, 0.5, 0.19, 20.0) == 'b'
    assert ed.switch_to_enter((-0.1, 0.30), 0.1, entry, door, axis, 0.5, 0.19, 20.0) == ''
    assert ed.switch_to_enter((-0.1, 0.25), 0.5, entry, door, axis, 0.5, 0.19, 20.0) == ''
    # rule (a): within 0.3 m of the entry pose, whatever the heading
    assert ed.switch_to_enter((-0.7, 0.2), 1.5, entry, door, axis, 0.5, 0.19, 20.0) == 'a'
