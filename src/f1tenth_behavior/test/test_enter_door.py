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
    # STRONG, because selection resolves over strong doors only -- these
    # tests are about the approach phases, not about earning selection.
    door.strong = True
    msg.doors = [door]
    behaviour.doors = ed.confirmed_doors(msg)
    behaviour.strong = ed.strong_doors(msg)
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
# The side selectors (schema_version 6.1)
# ---------------------------------------------------------------------------
def test_a_side_parses_and_a_bogus_side_does_not():
    """Only the two DOOR_SIDES words, exactly as spelled."""
    from f1tenth_behavior.mission.mission_config import DOOR_SIDES
    for side in DOOR_SIDES:
        assert parse_mission(door_mission(door=side)).moves[0].enter_door.door == side
    for bad in ('LEFT', 'links', 'sinistra', 'left ', 'front', 'ahead', 'both'):
        with pytest.raises(MissionConfigError):
            parse_mission(door_mission(door=bad))


def test_a_side_is_not_gated_on_schema_version():
    """Deliberate, and asserted so it is not "fixed" into a forward gate.

    schema_version is informational in this loader (parse_mission's own
    docstring) and does not decide which fields are legal; the one version
    gate, the terminal rule, exists for BACKWARD compatibility. "6.1" is what
    a mission DECLARES when it uses a side -- the number a reader looks up --
    and a loader built before sides existed rejects one loudly through
    DOOR_NAME_RE, which is the failure that actually protects anyone.
    """
    raw = door_mission(door='left')
    raw['schema_version'] = '6.0'
    assert parse_mission(raw).moves[0].enter_door.door == 'left'


def test_the_side_example_missions_load():
    """The three committed side missions, through the real loader."""
    from f1tenth_behavior.mission.mission_config import load_mission_file
    here = os.path.join(os.path.dirname(__file__), '..', 'missions')
    for name, door in (('enter_door_left', 'left'), ('enter_door_right', 'right')):
        config = load_mission_file(os.path.join(here, f'{name}.json'))
        assert config.schema_version == '6.1'
        assert config.moves[0].enter_door.door == door
        assert config.moves[0].terminal

    # The two-move shape: the door move is NOT terminal, the drive after it is.
    # "entered" means standing in the opening, so going through needs a second
    # move -- that is the whole point of the file.
    through = load_mission_file(os.path.join(here, 'enter_door_right_then_straight.json'))
    assert [m.id for m in through.moves] == ['move_0_enter_door_right',
                                             'move_1_straight_through']
    assert through.moves[0].enter_door.door == 'right'
    assert not through.moves[0].terminal
    assert through.moves[1].drive.mode == 'straight'
    assert through.moves[1].stop_condition.type == 'distance_reached'
    assert through.moves[1].terminal


def _side_scene():
    """Five strong doors around a car at the origin looking along +x.

    Bearings off the nose, which is what a side is measured in
    (DOOR_SIDE_MIN_DEG 15 .. DOOR_SIDE_MAX_DEG 135), and the distance, which
    is what breaks a tie between two doors on the same side:

        1  (3.0,  2.0)   +33.7 deg   3.61 m   left
        2  (4.0, -2.0)   -26.6 deg   4.47 m   right
        3  (1.0, -1.2)   -50.2 deg   1.56 m   right, and NEARER than 2
        4  (4.0,  0.3)    +4.3 deg   4.01 m   neither: the car points at it
        5  (-2.0, 1.0)  +153.4 deg   2.24 m   neither: behind the car

    No two doors are equidistant and none sits on a sector bound, here or at
    the other heading these tests use -- a scene that leans on either would
    be testing float luck rather than the rule.
    """
    pytest.importorskip('f1tenth_messages.msg', reason='needs the built workspace')
    from f1tenth_messages.msg import DoorArray
    from f1tenth_behavior.behaviours import enter_door as ed
    msg = DoorArray()
    msg.doors = [_door_msg(1, (3.0, 2.0), True), _door_msg(2, (4.0, -2.0), True),
                 _door_msg(3, (1.0, -1.2), True), _door_msg(4, (4.0, 0.3), True),
                 _door_msg(5, (-2.0, 1.0), True)]
    return ed, ed.strong_doors(msg)


def test_a_side_picks_the_nearest_door_on_that_side():
    """A side narrows the field; distance still decides inside it."""
    ed, doors = _side_scene()
    car, yaw = (0.0, 0.0), 0.0
    assert ed.select_door(doors, 'left', car, yaw) == 1
    assert ed.select_door(doors, 'right', car, yaw) == 3     # nearer than 2
    # ... and 'nearest' is unchanged by any of this: door 3 is nearest overall.
    assert ed.select_door(doors, 'nearest', car, yaw) == 3


def test_a_door_the_car_points_at_and_one_behind_it_are_on_neither_side():
    """Doors 4 and 5 are the two sector bounds, one each.

    4 is 4.3 deg off the nose: "ahead", which is what 'nearest' is for. 5 is
    153 deg round: the car would have to turn back to it, which is not what
    "the door on the left" asks for. Neither may be reachable by a side.
    """
    ed, doors = _side_scene()
    only_those_two = {k: v for k, v in doors.items() if k in (4, 5)}
    assert ed.select_door(only_those_two, 'left', (0.0, 0.0), 0.0) is None
    assert ed.select_door(only_those_two, 'right', (0.0, 0.0), 0.0) is None
    # Both ARE still visible to the other two selectors: 5 is the nearer of
    # the two, which is exactly why "nearest" and a side are different
    # questions -- the nearest door here is the one behind the car.
    assert ed.select_door(only_those_two, 'nearest', (0.0, 0.0), 0.0) == 5
    assert ed.select_door(only_those_two, 'door_4', (0.0, 0.0), 0.0) == 4


def test_a_side_is_measured_off_the_car_heading_not_the_map():
    """The same five doors, the same positions, the car turned 90 deg to port.

    Every answer moves. Door 1 was 33.7 deg to PORT and is now 56.3 deg to
    STARBOARD, so it is the nearest door on the RIGHT; door 5, behind the car
    before, is now 63.4 deg to port and is the only one on the left; door 3,
    which WAS the nearest on the right, is now 140 deg round and on neither
    side. A side that was a property of the door could not do any of this --
    and 'nearest', which asks a question about distance only, does not move.
    """
    ed, doors = _side_scene()
    car, yaw = (0.0, 0.0), math.pi / 2.0
    assert ed.select_door(doors, 'right', car, yaw) == 1
    assert ed.select_door(doors, 'left', car, yaw) == 5
    assert ed.select_door(doors, 'nearest', car, yaw) == 3


def test_a_side_needs_a_heading_and_a_position():
    """No pose yet is "not resolved", never a guess.

    The caller turns that into door_not_found once acquire_timeout_sec is up;
    picking a side off a missing heading would commit the whole move to the
    wrong doorway.
    """
    ed, doors = _side_scene()
    assert ed.select_door(doors, 'right', (0.0, 0.0), None) is None
    assert ed.select_door(doors, 'right', None, 0.0) is None
    # A door named outright needs neither, and still does not.
    assert ed.select_door(doors, 'door_2', None, None) == 2


def test_the_sector_is_bounded_by_the_two_constants():
    """Just inside each bound is on the side, just outside is not.

    A tenth of a degree either way rather than the bound itself: a bearing
    built with cos/sin and read back with atan2 lands a few 1e-14 off the
    angle it was built from, so an exactly-on-the-bound assertion would be
    testing float arithmetic, not the sector. Which way the bound itself goes
    is not a question any caller can ask.
    """
    ed, _ = _side_scene()
    for side, sign in (('left', 1.0), ('right', -1.0)):
        for deg, want in ((ed.DOOR_SIDE_MIN_DEG + 0.1, True),
                          (ed.DOOR_SIDE_MIN_DEG - 0.1, False),
                          (ed.DOOR_SIDE_MAX_DEG - 0.1, True),
                          (ed.DOOR_SIDE_MAX_DEG + 0.1, False)):
            bearing = math.radians(sign * deg)
            door_xy = (2.0 * math.cos(bearing), 2.0 * math.sin(bearing))
            assert ed.on_side(door_xy, (0.0, 0.0), 0.0, side) is want, (side, deg)


def test_a_side_is_subject_to_the_strong_rule():
    """Door 3 is the nearest on the right and is NOT strong.

    The side selector runs over whichever list it is handed, so the strength
    rule holds for it exactly as it does for 'nearest': selection sees strong
    doors only, and the answer is the next one along, not the nearest.
    """
    pytest.importorskip('f1tenth_messages.msg', reason='needs the built workspace')
    from f1tenth_messages.msg import DoorArray
    from f1tenth_behavior.behaviours import enter_door as ed
    msg = DoorArray()
    msg.doors = [_door_msg(2, (3.0, -2.0), True), _door_msg(3, (1.0, -1.0), False)]
    car, yaw = (0.0, 0.0), 0.0
    assert ed.select_door(ed.confirmed_doors(msg), 'right', car, yaw) == 3
    assert ed.select_door(ed.strong_doors(msg), 'right', car, yaw) == 2


def _run_acquire(selector, doors, car_xy=(0.0, 0.0), car_yaw=0.0):
    """Drive EnterDoor's acquisition past acquire_timeout_sec; return the state."""
    pytest.importorskip('f1tenth_messages.msg', reason='needs the built workspace')
    from f1tenth_messages.msg import DoorArray
    from f1tenth_behavior.behaviours import enter_door as ed
    from f1tenth_behavior.mission.runtime import (
        GLOBAL_XY_KEY, GLOBAL_YAW_KEY, MISSION_KEY, MissionRuntimeState)

    state = MissionRuntimeState()
    state.config = parse_mission(door_mission(door=selector))
    clock = [0.0]
    b = ed.EnterDoor(clock=lambda: clock[0])
    b.node = MagicMock()
    b.goal_pose_pub = MagicMock()
    b.hold_pub = MagicMock()
    b.blackboard = SimpleNamespace(**{MISSION_KEY: state, GLOBAL_XY_KEY: car_xy,
                                      GLOBAL_YAW_KEY: car_yaw})
    b._bb = lambda key: getattr(b.blackboard, key, None)
    msg = DoorArray()
    msg.doors = doors
    b._doors_cb(msg)
    b.update()
    clock[0] = state.config.moves[0].enter_door.acquire_timeout_sec + 1.0
    b.update()
    return state, b


def test_the_move_resolves_a_side_and_fails_cleanly_when_that_side_is_empty():
    """Both failures keep their meaning for a side selector.

    `door_not_found` is "there is no door on the right" -- which includes a
    door that is there but ahead, or behind -- and `no_strong_door` is "there
    is one on the right and it has not earned being driven at". Collapsing
    them would lose the difference between looking elsewhere and driving past
    the same doorway again.
    """
    ok, _ = _run_acquire('right', [_door_msg(2, (3.0, -2.0), True)])
    assert ok.door_record.door_id == 2
    assert ok.door_record.outcome is None

    # The only door is on the OTHER side: nothing matched, not "not strong".
    wrong_side, _ = _run_acquire('right', [_door_msg(1, (3.0, 2.0), True)])
    assert wrong_side.door_record.outcome == 'door_not_found'
    assert wrong_side.last_stop_reason == 'enter_door:door_not_found'

    # On the right, but not strong.
    weak, _ = _run_acquire('right', [_door_msg(2, (3.0, -2.0), False)])
    assert weak.door_record.outcome == 'no_strong_door'

    # On the right, strong, but the car has no heading yet: still unresolved,
    # and the acquire timeout is what ends it.
    blind, _ = _run_acquire('right', [_door_msg(2, (3.0, -2.0), True)], car_yaw=None)
    assert blind.door_record.outcome == 'door_not_found'


def test_a_resolved_side_is_not_re_picked_when_the_car_turns():
    """The reason the side is resolved ONCE.

    The car resolves "right" to door 2, then turns to port to line up with
    it -- and door 2 is now dead ahead, on neither side. Re-reading the
    selector would drop the door the move is already approaching.
    """
    state, b = _run_acquire('right', [_door_msg(2, (3.0, -2.0), True)])
    assert state.door_record.door_id == 2
    b.blackboard.mission_global_yaw = math.radians(-33.7)   # nose on the door
    b.update()
    assert state.door_record.door_id == 2
    assert state.door_record.outcome is None
