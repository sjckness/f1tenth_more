"""Unit tests for llm.plan_translate.phases_to_mission() -- pure Python, no
rclpy/ROS runtime needed (see plan_translate.py's own module docstring).

REWRITTEN for schema_version 3.0. The old suite asserted the shape of the
LOSSY mapping this module used to perform -- a 50 m goal_distance cap for
clearance-guarded phases, a made-up turn speed, front_clearance for guard
"front_object". All three are gone by design, so tests pinning them are gone
with them; what replaces them asserts that nothing is invented any more.

THE LOAD-BEARING TEST IN THIS FILE is TestRoundTripThroughTheRealParser: it
runs the output back through f1tenth_behavior's actual
mission_config.parse_mission(), not a local copy of the rules. A translator
that emits a mission the real loader rejects is worse than useless -- it
fails at the moment a person has already spoken a command to the robot. If
f1tenth_behavior is not importable the round-trip is skipped rather than
silently passing on a hand-rolled stand-in.

Run standalone: python3 -m pytest test/test_plan_translate.py -v
"""

import math

import pytest

from llm.plan_translate import (
    APPROACH_D_SAFE_BACKOFF_M,
    MISSION_SCHEMA_VERSION,
    WALL_DEBOUNCE_TICKS,
    PlanTranslationError,
    phases_to_mission,
)

try:
    from f1tenth_behavior.mission.mission_config import parse_mission
except ImportError:  # pragma: no cover - depends on the workspace being sourced
    parse_mission = None

requires_behavior = pytest.mark.skipif(
    parse_mission is None,
    reason='f1tenth_behavior not importable -- source the workspace to run the round-trip',
)

# SYSTEM_PROMPT's own example 1 (llm_planner_node.py):
#   "vai dritto, al muro gira a destra e avanza 2 metri"
# Phase 3 is mode "wall_turn" with guard "distance", NOT "turned" -- real,
# expected LLM output (REGOLA 1: "stay in wall_turn after a turn to keep
# going"), and the single most important case in this file. See
# plan_translate.py's module docstring for why it must NOT become a
# drive.mode of "wall_turn".
EXAMPLE_1_PLAN = [
    {'mode': 'straight', 'guard': 'wall', 'thresh': 3.0},
    {'mode': 'wall_turn', 'turn_sign': -1.0, 'guard': 'turned', 'thresh': 1.3},
    {'mode': 'wall_turn', 'turn_sign': -1.0, 'guard': 'distance', 'thresh': 2.0,
     'stop_at_distance': 2.0},
]

# SYSTEM_PROMPT's own example 2:
#   "vai dritto, supera l'ostacolo e avanza 3 metri"
EXAMPLE_2_PLAN = [
    {'mode': 'straight', 'guard': 'distance', 'thresh': 3.0, 'stop_at_distance': 3.0},
]


class TestExample1ThreePhasePlan:

    def setup_method(self):
        self.mission = phases_to_mission(EXAMPLE_1_PLAN, mission_id='ex1')
        self.moves = self.mission['moves']

    def test_schema_version_is_3_0(self):
        assert self.mission['schema_version'] == MISSION_SCHEMA_VERSION == '3.0'

    def test_every_move_is_a_drive_move_and_nothing_else(self):
        for m in self.moves:
            assert 'drive' in m
            assert 'goal_distance' not in m
            assert 'goal_pose' not in m
            assert 'turn' not in m

    def test_no_fabricated_distance_cap_anywhere(self):
        # The whole point of 3.0. A clearance-guarded phase used to become
        # goal_distance: 50.0 and the MPC really did drive toward it.
        assert '50' not in str(self.mission)

    def test_phase_1_wall_guard_becomes_a_debounced_front_clearance(self):
        m = self.moves[0]
        assert m['drive'] == {'mode': 'straight'}
        assert m['stop_condition'] == {
            'type': 'front_clearance', 'distance': 3.0,
            'debounce_ticks': WALL_DEBOUNCE_TICKS}

    def test_phase_2_turned_guard_becomes_a_wall_turn_drive(self):
        m = self.moves[1]
        expected_deg = math.degrees(1.3)
        assert m['drive']['mode'] == 'wall_turn'
        assert m['drive']['turn_sign'] == -1.0
        assert m['drive']['turn_mag_deg'] == pytest.approx(expected_deg)
        assert m['stop_condition']['type'] == 'orientation_delta'
        assert m['stop_condition']['value'] == pytest.approx(expected_deg)

    def test_turn_magnitude_and_orientation_value_are_bit_identical(self):
        # mission_config compares them with abs_tol=1e-6; deriving them twice
        # from thresh would risk a last-bit mismatch it rejects at load time.
        m = self.moves[1]
        assert m['drive']['turn_mag_deg'] == m['stop_condition']['value']

    def test_phase_3_wall_turn_continuation_becomes_straight_not_wall_turn(self):
        # THE regression this file exists to prevent. mpc_corr re-anchors the
        # heading per move, so copying "wall_turn" across would command a
        # SECOND 90 degree turn and send the car back the way it came.
        m = self.moves[2]
        assert m['drive']['mode'] == 'straight'
        assert 'turn_sign' not in m['drive']
        assert 'turn_mag_deg' not in m['drive']

    def test_only_the_last_move_is_terminal(self):
        assert [m.get('terminal', False) for m in self.moves] == [False, False, True]

    def test_stop_at_distance_overrides_the_last_moves_stop_condition(self):
        assert self.moves[2]['stop_condition'] == {
            'type': 'distance_reached', 'distance': 2.0}

    def test_stop_at_distance_adds_no_standoff_relaxation(self):
        # A distance is not an object; there is nothing to close on.
        assert 'approach_d_safe' not in self.moves[2]['drive']

    def test_move_ids_are_unique_and_ordered(self):
        assert [m['id'] for m in self.moves] == ['move_0', 'move_1', 'move_2']


class TestExample2SinglePhasePlan:

    def setup_method(self):
        self.mission = phases_to_mission(EXAMPLE_2_PLAN, mission_id='ex2')

    def test_single_terminal_drive_move(self):
        moves = self.mission['moves']
        assert len(moves) == 1
        assert moves[0]['drive'] == {'mode': 'straight'}
        assert moves[0]['terminal'] is True
        assert moves[0]['stop_condition'] == {
            'type': 'distance_reached', 'distance': 3.0}


class TestFrontObjectGuard:
    """The claim-1 correction: guard "front_object" used to map to
    front_clearance, which reads slam_toolbox's occupancy grid. A chair is
    not on the map, so that value never dropped and the move never ended."""

    def test_maps_to_a_forward_filtered_obstacle_distance(self):
        mission = phases_to_mission(
            [{'mode': 'straight', 'guard': 'front_object', 'thresh': 1.0,
              'stop_at': 1.0}])
        assert mission['moves'][0]['stop_condition'] == {
            'type': 'obstacle_distance_below', 'distance': 1.0, 'forward_only': True}

    def test_non_terminal_front_object_still_uses_the_same_sensor(self):
        mission = phases_to_mission([
            {'mode': 'straight', 'guard': 'front_object', 'thresh': 2.0},
            {'mode': 'straight', 'guard': 'distance', 'thresh': 1.0,
             'stop_at_distance': 1.0},
        ])
        cond = mission['moves'][0]['stop_condition']
        assert cond['type'] == 'obstacle_distance_below'
        assert cond['forward_only'] is True


class TestTerminalStopAt:

    def setup_method(self):
        self.mission = phases_to_mission(
            [{'mode': 'straight', 'guard': 'front_object', 'thresh': 1.0,
              'stop_at': 1.0}])
        self.move = self.mission['moves'][0]

    def test_stop_at_becomes_a_forward_obstacle_stop(self):
        assert self.move['stop_condition'] == {
            'type': 'obstacle_distance_below', 'distance': 1.0, 'forward_only': True}

    def test_stop_at_relaxes_the_approach_standoff(self):
        assert self.move['drive']['approach_d_safe'] == pytest.approx(
            1.0 - APPROACH_D_SAFE_BACKOFF_M)

    def test_the_relaxation_is_below_the_stop_threshold(self):
        # The entire point: avoidance must not push the car around the object
        # before the stop condition can fire.
        assert self.move['drive']['approach_d_safe'] < self.move['stop_condition']['distance']

    def test_a_tiny_stop_at_never_produces_a_negative_standoff(self):
        mission = phases_to_mission(
            [{'mode': 'straight', 'guard': 'front_object', 'thresh': 0.05,
              'stop_at': 0.05}])
        assert mission['moves'][0]['drive']['approach_d_safe'] == 0.0

    def test_stop_at_overrides_even_on_a_turn_phase(self):
        mission = phases_to_mission(
            [{'mode': 'wall_turn', 'turn_sign': 1.0, 'guard': 'turned',
              'thresh': 1.57, 'stop_at': 1.0}])
        move = mission['moves'][0]
        # Overridden, so the turn's own orientation_delta is gone -- exactly
        # the precedence f110_autonomy's if/elif-before-the-guard had.
        assert move['stop_condition']['type'] == 'obstacle_distance_below'
        assert move['drive']['mode'] == 'wall_turn'


class TestTurnSignConvention:

    def _turn(self, sign):
        mission = phases_to_mission(
            [{'mode': 'wall_turn', 'turn_sign': sign, 'guard': 'turned',
              'thresh': 1.57, 'stop_at_distance': 1.0}])
        return mission['moves'][0]['drive']

    def test_negative_is_right(self):
        assert self._turn(-1.0)['turn_sign'] == -1.0

    def test_positive_is_left(self):
        assert self._turn(1.0)['turn_sign'] == 1.0

    def test_magnitude_is_unsigned_in_both_directions(self):
        assert self._turn(-1.0)['turn_mag_deg'] == self._turn(1.0)['turn_mag_deg'] > 0


class TestLargeTurnsAreExpressible:
    """180 and 270 degree turns, now that orientation_delta accumulates
    unwrapped. A wrapped delta is bounded to (-180, 180] and could never
    satisfy either."""

    @pytest.mark.parametrize('radians,degrees', [(math.pi, 180.0), (1.5 * math.pi, 270.0)])
    def test_translated_without_wrapping(self, radians, degrees):
        mission = phases_to_mission(
            [{'mode': 'wall_turn', 'turn_sign': 1.0, 'guard': 'turned',
              'thresh': radians, 'stop_at_distance': 1.0}])
        # stop_at_distance overrides the stop_condition, so check the drive.
        assert mission['moves'][0]['drive']['turn_mag_deg'] == pytest.approx(degrees)

    @pytest.mark.parametrize('radians,degrees', [(math.pi, 180.0), (1.5 * math.pi, 270.0)])
    def test_orientation_delta_survives_when_not_overridden(self, radians, degrees):
        mission = phases_to_mission([
            {'mode': 'wall_turn', 'turn_sign': 1.0, 'guard': 'turned', 'thresh': radians},
            {'mode': 'straight', 'guard': 'distance', 'thresh': 1.0,
             'stop_at_distance': 1.0},
        ])
        cond = mission['moves'][0]['stop_condition']
        assert cond['type'] == 'orientation_delta'
        assert cond['value'] == pytest.approx(degrees)


class TestRejections:

    def test_rejects_non_list(self):
        with pytest.raises(PlanTranslationError):
            phases_to_mission({'mode': 'straight'})

    def test_rejects_empty_list(self):
        with pytest.raises(PlanTranslationError):
            phases_to_mission([])

    def test_rejects_phase_missing_required_keys(self):
        with pytest.raises(PlanTranslationError):
            phases_to_mission([{'mode': 'straight', 'guard': 'wall'}])

    def test_rejects_non_numeric_thresh(self):
        with pytest.raises(PlanTranslationError):
            phases_to_mission([{'mode': 'straight', 'guard': 'wall', 'thresh': '3.0'}])

    def test_rejects_wall_turn_without_turn_sign(self):
        with pytest.raises(PlanTranslationError):
            phases_to_mission([{'mode': 'wall_turn', 'guard': 'turned', 'thresh': 1.3}])

    def test_rejects_unsupported_guard(self):
        with pytest.raises(PlanTranslationError, match='unsupported guard'):
            phases_to_mission([{'mode': 'straight', 'guard': 'vibes', 'thresh': 1.0}])

    def test_rejects_turned_guard_on_a_straight_phase(self):
        # No direction to derive the rotation from, and mission_config would
        # reject orientation_delta on a straight drive move anyway.
        with pytest.raises(PlanTranslationError, match="guard 'turned' requires"):
            phases_to_mission([{'mode': 'straight', 'guard': 'turned', 'thresh': 1.3}])


@requires_behavior
class TestRoundTripThroughTheRealParser:
    """Every plan this module can produce must LOAD. Run against
    f1tenth_behavior's actual parser, not a restatement of its rules."""

    def test_example_1_round_trips(self):
        cfg = parse_mission(phases_to_mission(EXAMPLE_1_PLAN, mission_id='ex1'))
        assert cfg.schema_version == '3.0'
        assert [m.drive.mode for m in cfg.moves] == ['straight', 'wall_turn', 'straight']
        assert cfg.moves[-1].terminal is True

    def test_example_2_round_trips(self):
        cfg = parse_mission(phases_to_mission(EXAMPLE_2_PLAN, mission_id='ex2'))
        assert len(cfg.moves) == 1
        assert cfg.moves[0].drive.mode == 'straight'
        assert cfg.moves[0].terminal is True

    @pytest.mark.parametrize('radians', [math.pi / 2, math.pi, 1.5 * math.pi])
    def test_turns_up_to_270_degrees_round_trip(self, radians):
        # A 90, a 180 and a 270. The orientation_delta / turn_mag_deg equality
        # check inside parse_mission is what this really exercises.
        cfg = parse_mission(phases_to_mission([
            {'mode': 'wall_turn', 'turn_sign': -1.0, 'guard': 'turned', 'thresh': radians},
            {'mode': 'straight', 'guard': 'distance', 'thresh': 1.0,
             'stop_at_distance': 1.0},
        ]))
        assert cfg.moves[0].drive.turn_mag_deg == pytest.approx(math.degrees(radians))
        assert cfg.moves[0].stop_condition.type == 'orientation_delta'

    def test_a_terminal_object_stop_round_trips_with_its_relaxation(self):
        cfg = parse_mission(phases_to_mission([
            {'mode': 'straight', 'guard': 'wall', 'thresh': 3.0},
            {'mode': 'straight', 'guard': 'front_object', 'thresh': 1.0, 'stop_at': 1.0},
        ]))
        last = cfg.moves[-1]
        assert last.terminal is True
        assert last.stop_condition.type == 'obstacle_distance_below'
        assert last.stop_condition.params['forward_only'] is True
        assert last.drive.approach_d_safe == pytest.approx(1.0 - APPROACH_D_SAFE_BACKOFF_M)

    def test_every_guard_round_trips(self):
        for guard, thresh in (('wall', 3.0), ('front_object', 1.5), ('distance', 2.0)):
            cfg = parse_mission(phases_to_mission([
                {'mode': 'straight', 'guard': guard, 'thresh': thresh},
                {'mode': 'straight', 'guard': 'distance', 'thresh': 1.0,
                 'stop_at_distance': 1.0},
            ]))
            assert cfg.moves[0].drive is not None
