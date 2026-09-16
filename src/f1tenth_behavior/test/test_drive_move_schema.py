"""schema_version 3.0: the "drive" step and the `terminal` flag.

Covers the four things that can silently go wrong here, in the order they
would bite:

  1. A drive move parses, and the wrong ones are REJECTED BY NAME rather than
     accepted and left to misbehave live.
  2. `terminal` is positioned correctly -- last move only, required on the
     last move of a 3.0 mission, and NOT required of anything older.
  3. Backward compatibility: every mission already under missions/ still
     parses, and the pre-3.0 ones still parse without a terminal flag.
  4. orientation_delta, which used to be turn-exclusive, now also works on a
     drive step -- the prerequisite that makes the LLM's "turned" guard
     expressible at all.
"""

import json
from pathlib import Path

import pytest

from f1tenth_behavior.mission.mission_config import (
    TERMINAL_REQUIRED_VERSIONS, MissionConfigError, load_mission_file, parse_mission)

MISSIONS_DIR = Path(__file__).resolve().parent.parent / 'missions'


def _mission(*moves, schema_version='3.0'):
    raw = {'mission_id': 'unit_test', 'moves': list(moves)}
    if schema_version is not None:
        raw['schema_version'] = schema_version
    return raw


def _drive_move(move_id='m0', terminal=True, drive=None, stop_condition=None):
    return {
        'id': move_id,
        'drive': drive if drive is not None else {'mode': 'straight'},
        'stop_condition': (stop_condition if stop_condition is not None
                           else {'type': 'front_clearance', 'distance': 3.0}),
        'terminal': terminal,
    }


class TestDriveMoveParsing:

    def test_minimal_straight_drive_move_parses(self):
        cfg = parse_mission(_mission(_drive_move()))
        move = cfg.moves[0]
        assert move.drive is not None
        assert move.drive.mode == 'straight'
        # Every optional field falls back to its "use the controller default"
        # sentinel rather than to a number this layer made up.
        assert move.drive.speed == 0.0
        assert move.drive.turn_mag_deg == 0.0
        assert move.drive.approach_d_safe is None
        # A drive move is a drive move and nothing else.
        assert move.goal_distance is None
        assert move.goal_pose is None
        assert move.turn is None

    def test_wall_turn_carries_sign_magnitude_and_speed(self):
        cfg = parse_mission(_mission(_drive_move(
            drive={'mode': 'wall_turn', 'turn_sign': -1.0,
                   'turn_mag_deg': 90.0, 'speed': 0.3},
            stop_condition={'type': 'orientation_delta', 'value': 90.0},
        )))
        d = cfg.moves[0].drive
        assert (d.mode, d.turn_sign, d.turn_mag_deg, d.speed) == (
            'wall_turn', -1.0, 90.0, 0.3)

    def test_approach_d_safe_zero_is_a_real_value_not_unset(self):
        # 0.0 means "no standoff at all" and must survive as such -- it is
        # exactly why "unset" is None here and a NEGATIVE sentinel on the wire.
        cfg = parse_mission(_mission(_drive_move(
            drive={'mode': 'straight', 'approach_d_safe': 0.0})))
        assert cfg.moves[0].drive.approach_d_safe == 0.0

    def test_drive_is_mutually_exclusive_with_the_other_three_shapes(self):
        for other in ({'goal_distance': 2.0},
                      {'goal_pose': {'x': 1.0, 'y': 1.0, 'yaw': 0.0}},
                      {'turn': {'heading_delta_deg': 90, 'speed': 0.3,
                                'steering': 'full_lock'}}):
            move = _drive_move()
            move.update(other)
            with pytest.raises(MissionConfigError, match='exactly one of'):
                parse_mission(_mission(move))

    def test_a_move_with_no_goal_shape_at_all_is_rejected(self):
        with pytest.raises(MissionConfigError, match='exactly one of'):
            parse_mission(_mission({
                'id': 'm0',
                'stop_condition': {'type': 'front_clearance', 'distance': 3.0},
                'terminal': True,
            }))


class TestDriveMoveRejections:

    def test_unknown_mode_rejected_by_name(self):
        with pytest.raises(MissionConfigError, match='drive.mode'):
            parse_mission(_mission(_drive_move(drive={'mode': 'reverse'})))

    def test_wall_turn_without_turn_sign_rejected(self):
        with pytest.raises(MissionConfigError, match='turn_sign'):
            parse_mission(_mission(_drive_move(
                drive={'mode': 'wall_turn', 'turn_mag_deg': 90.0})))

    def test_wall_turn_with_a_non_unit_turn_sign_rejected(self):
        with pytest.raises(MissionConfigError, match='turn_sign'):
            parse_mission(_mission(_drive_move(
                drive={'mode': 'wall_turn', 'turn_sign': -0.5,
                       'turn_mag_deg': 90.0})))

    def test_negative_turn_mag_rejected_as_a_sign_mistake(self):
        # The direction belongs in turn_sign; a negative magnitude paired with
        # a negative sign would otherwise silently turn the wrong way.
        with pytest.raises(MissionConfigError, match='MAGNITUDE'):
            parse_mission(_mission(_drive_move(
                drive={'mode': 'wall_turn', 'turn_sign': 1.0,
                       'turn_mag_deg': -90.0})))

    def test_string_speed_rejected_not_coerced(self):
        with pytest.raises(MissionConfigError, match='drive.speed must be numeric'):
            parse_mission(_mission(_drive_move(
                drive={'mode': 'straight', 'speed': '0.5'})))

    def test_bool_is_not_accepted_as_a_number(self):
        # bool IS an int in Python; a bare isinstance check would let this by.
        with pytest.raises(MissionConfigError, match='drive.turn_mag_deg must be numeric'):
            parse_mission(_mission(_drive_move(
                drive={'mode': 'straight', 'turn_mag_deg': True})))

    def test_negative_approach_d_safe_rejected(self):
        # The negative sentinel is a WIRE-level encoding, created in
        # publish_move_goal; it is not a legal value in the JSON.
        with pytest.raises(MissionConfigError, match='approach_d_safe'):
            parse_mission(_mission(_drive_move(
                drive={'mode': 'straight', 'approach_d_safe': -1.0})))

    def test_drive_must_be_an_object(self):
        with pytest.raises(MissionConfigError, match='drive must be an object'):
            parse_mission(_mission(_drive_move(drive='straight')))


class TestTerminalFlagPosition:

    def _three(self, terminal_on):
        moves = []
        for i in range(3):
            moves.append(_drive_move(
                move_id=f'm{i}',
                terminal=(i == terminal_on),
                stop_condition={'type': 'distance_reached', 'distance': 1.0},
            ))
        return _mission(*moves)

    def test_terminal_on_the_last_move_is_accepted(self):
        cfg = parse_mission(self._three(terminal_on=2))
        assert [m.terminal for m in cfg.moves] == [False, False, True]

    def test_terminal_on_the_first_move_is_rejected(self):
        with pytest.raises(MissionConfigError, match='only legal on the LAST move'):
            parse_mission(self._three(terminal_on=0))

    def test_terminal_on_a_middle_move_is_rejected(self):
        with pytest.raises(MissionConfigError, match='only legal on the LAST move'):
            parse_mission(self._three(terminal_on=1))

    def test_a_3_0_mission_whose_last_move_is_not_terminal_is_rejected(self):
        with pytest.raises(MissionConfigError, match='requires the last move'):
            parse_mission(self._three(terminal_on=None))

    def test_terminal_must_be_a_real_boolean(self):
        with pytest.raises(MissionConfigError, match='terminal must be a JSON boolean'):
            parse_mission(_mission(_drive_move(terminal=1)))


class TestPre30MissionsAreUntouched:
    """COMPATIBILITY IS NON-NEGOTIABLE (mission_config.parse_mission's own
    docstring). These are the assertions that hold the line."""

    def test_a_2_0_mission_needs_no_terminal_flag(self):
        cfg = parse_mission(_mission({
            'id': 'm0',
            'goal_distance': 2.0,
            'stop_condition': {'type': 'distance_reached', 'distance': 2.0},
        }, schema_version='2.0'))
        assert cfg.moves[-1].terminal is False

    def test_a_versionless_mission_needs_no_terminal_flag(self):
        cfg = parse_mission(_mission({
            'id': 'm0',
            'goal_distance': 2.0,
            'stop_condition': {'type': 'distance_reached', 'distance': 2.0},
        }, schema_version=None))
        assert cfg.schema_version == '1.0'
        assert cfg.moves[-1].terminal is False

    def test_terminal_position_is_still_enforced_below_3_0(self):
        # The "last move only" rule is universal -- it can never break an
        # older mission, because no older mission carries the field at all.
        with pytest.raises(MissionConfigError, match='only legal on the LAST move'):
            parse_mission(_mission(
                {'id': 'a', 'goal_distance': 1.0, 'terminal': True,
                 'stop_condition': {'type': 'distance_reached', 'distance': 1.0}},
                {'id': 'b', 'goal_distance': 1.0,
                 'stop_condition': {'type': 'distance_reached', 'distance': 1.0}},
                schema_version='2.0'))

    def test_every_shipped_mission_still_parses(self):
        found = sorted(MISSIONS_DIR.glob('*.json'))
        assert found, 'no missions found -- wrong path?'
        for path in found:
            load_mission_file(str(path))  # raises on any violation

    def test_no_pre_3_0_shipped_mission_was_migrated(self):
        # The task forbade migrating them. This asserts none acquired a
        # terminal flag or a drive step as a side effect.
        for path in sorted(MISSIONS_DIR.glob('*.json')):
            raw = json.loads(path.read_text(encoding='utf-8'))
            # 3.0 and every later version that requires terminal (4.0 added
            # go_to_object) -- only the pre-3.0 missions must stay untouched.
            if raw.get('schema_version') in TERMINAL_REQUIRED_VERSIONS:
                continue
            for move in raw['moves']:
                assert 'terminal' not in move, f'{path.name}: {move["id"]} gained terminal'
                assert 'drive' not in move, f'{path.name}: {move["id"]} gained a drive step'


class TestOrientationDeltaOnADriveStep:
    """Used to be turn-exclusive. Relaxing it is what makes the LLM's
    "turned" guard expressible -- see mission_config._parse_move."""

    def test_accepted_on_a_wall_turn_drive_move(self):
        cfg = parse_mission(_mission(_drive_move(
            drive={'mode': 'wall_turn', 'turn_sign': 1.0, 'turn_mag_deg': 90.0},
            stop_condition={'type': 'orientation_delta', 'value': 90.0})))
        assert cfg.moves[0].stop_condition.params['value'] == 90.0

    @pytest.mark.parametrize('degrees', [180.0, 270.0])
    def test_turns_at_and_beyond_180_are_accepted(self, degrees):
        # Only reachable at all because CheckStopCondition accumulates
        # rotation unwrapped -- a wrapped delta is bounded to (-180, 180].
        cfg = parse_mission(_mission(_drive_move(
            drive={'mode': 'wall_turn', 'turn_sign': -1.0, 'turn_mag_deg': degrees},
            stop_condition={'type': 'orientation_delta', 'value': degrees})))
        assert cfg.moves[0].drive.turn_mag_deg == degrees

    def test_value_must_match_the_commanded_magnitude(self):
        with pytest.raises(MissionConfigError, match='must equal'):
            parse_mission(_mission(_drive_move(
                drive={'mode': 'wall_turn', 'turn_sign': 1.0, 'turn_mag_deg': 90.0},
                stop_condition={'type': 'orientation_delta', 'value': 45.0})))

    def test_rejected_on_a_straight_drive_move(self):
        with pytest.raises(MissionConfigError, match='requires drive.mode "wall_turn"'):
            parse_mission(_mission(_drive_move(
                drive={'mode': 'straight'},
                stop_condition={'type': 'orientation_delta', 'value': 90.0})))

    def test_rejected_when_the_magnitude_is_the_default_sentinel(self):
        # turn_mag_deg 0 means "use the controller default", which this
        # equality check has no way to compare against.
        with pytest.raises(MissionConfigError, match='non-zero drive.turn_mag_deg'):
            parse_mission(_mission(_drive_move(
                drive={'mode': 'wall_turn', 'turn_sign': 1.0},
                stop_condition={'type': 'orientation_delta', 'value': 90.0})))

    def test_still_rejected_on_a_move_that_is_neither_turn_nor_drive(self):
        with pytest.raises(MissionConfigError, match='only valid on a "turn" or "drive" step'):
            parse_mission(_mission({
                'id': 'm0',
                'goal_distance': 2.0,
                'stop_condition': {'type': 'orientation_delta', 'value': 90.0},
                'terminal': True,
            }))


class TestStopConditionOptionalFields:

    def test_debounce_ticks_accepted_on_front_clearance(self):
        cfg = parse_mission(_mission(_drive_move(
            stop_condition={'type': 'front_clearance', 'distance': 3.0,
                            'debounce_ticks': 3})))
        assert cfg.moves[0].stop_condition.params['debounce_ticks'] == 3

    def test_debounce_ticks_rejected_on_another_type(self):
        with pytest.raises(MissionConfigError, match='only meaningful on a front_clearance'):
            parse_mission(_mission(_drive_move(
                stop_condition={'type': 'distance_reached', 'distance': 3.0,
                                'debounce_ticks': 3})))

    def test_debounce_ticks_must_be_an_int_above_zero(self):
        for bad in (0, -1, 1.5, '3', True):
            with pytest.raises(MissionConfigError, match='debounce_ticks'):
                parse_mission(_mission(_drive_move(
                    stop_condition={'type': 'front_clearance', 'distance': 3.0,
                                    'debounce_ticks': bad})))

    def test_forward_only_accepted_on_obstacle_distance_below(self):
        cfg = parse_mission(_mission(_drive_move(
            stop_condition={'type': 'obstacle_distance_below', 'distance': 1.0,
                            'forward_only': True})))
        assert cfg.moves[0].stop_condition.params['forward_only'] is True

    def test_forward_only_rejected_on_another_type(self):
        with pytest.raises(MissionConfigError, match='only meaningful on an obstacle'):
            parse_mission(_mission(_drive_move(
                stop_condition={'type': 'front_clearance', 'distance': 3.0,
                                'forward_only': True})))

    def test_forward_only_must_be_a_real_boolean(self):
        with pytest.raises(MissionConfigError, match='forward_only must be a JSON boolean'):
            parse_mission(_mission(_drive_move(
                stop_condition={'type': 'obstacle_distance_below', 'distance': 1.0,
                                'forward_only': 'yes'})))
