"""Unit tests for llm.plan_translate.phases_to_mission() -- pure Python, no
rclpy/ROS runtime needed (see plan_translate.py's own module docstring).

Run standalone: python3 -m pytest test/test_plan_translate.py -v
"""

import math

import pytest

from llm.plan_translate import (
    DEFAULT_TURN_SPEED_MPS,
    MISSION_SCHEMA_VERSION,
    STRAIGHT_GOAL_DISTANCE_CAP_M,
    PlanTranslationError,
    phases_to_mission,
)

# SYSTEM_PROMPT's own example 1 (llm_planner_node.py):
#   "vai dritto, al muro gira a destra e avanza 2 metri"
# Note phase 2 is mode "wall_turn" with guard "distance", NOT "turned" --
# see plan_translate.py's module docstring for why that's real, expected
# LLM output (REGOLA 1's "stay in wall_turn mode after a turn to keep
# going") and not a malformed phase.
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

    def test_three_moves_correct_types_and_ids(self):
        mission = phases_to_mission(EXAMPLE_1_PLAN)
        assert mission['schema_version'] == MISSION_SCHEMA_VERSION
        moves = mission['moves']
        assert [m['id'] for m in moves] == ['move_0', 'move_1', 'move_2']

        # move_0: straight/wall -> goal_distance (capped, NOT thresh -- the
        # move stops on clearance, thresh is the front_clearance distance)
        # + front_clearance
        assert moves[0]['goal_distance'] == pytest.approx(STRAIGHT_GOAL_DISTANCE_CAP_M)
        assert moves[0]['stop_condition'] == {'type': 'front_clearance', 'distance': 3.0}
        assert 'turn' not in moves[0]

        # move_1: wall_turn/turned -> turn + orientation_delta
        assert 'turn' in moves[1]
        assert moves[1]['turn']['steering'] == 'full_lock'
        assert moves[1]['turn']['speed'] == DEFAULT_TURN_SPEED_MPS
        expected_heading = math.degrees(1.3) * -1.0
        assert moves[1]['turn']['heading_delta_deg'] == pytest.approx(expected_heading)
        assert moves[1]['stop_condition']['type'] == 'orientation_delta'
        assert moves[1]['stop_condition']['value'] == pytest.approx(abs(expected_heading))
        assert 'goal_distance' not in moves[1]

        # move_2: wall_turn/distance (post-turn continuation, NOT a second
        # turn) -> goal_distance + distance_reached, then overridden by its
        # own stop_at_distance (same value here, so unchanged either way).
        assert moves[2]['goal_distance'] == pytest.approx(2.0)
        assert moves[2]['stop_condition'] == {'type': 'distance_reached', 'distance': 2.0}
        assert 'turn' not in moves[2]

    def test_orientation_delta_value_matches_turn_heading_delta_deg_exactly(self):
        """mission_config.py rejects the mission at load time unless these two
        are math.isclose (abs_tol=1e-6) -- assert the exact invariant, not
        just approximately-plausible numbers."""
        mission = phases_to_mission(EXAMPLE_1_PLAN)
        turn_move = mission['moves'][1]
        assert math.isclose(
            abs(turn_move['turn']['heading_delta_deg']),
            turn_move['stop_condition']['value'],
            abs_tol=1e-6,
        )


class TestExample2SinglePhasePlan:

    def test_single_move_distance_reached(self):
        mission = phases_to_mission(EXAMPLE_2_PLAN)
        assert len(mission['moves']) == 1
        move = mission['moves'][0]
        assert move['goal_distance'] == pytest.approx(3.0)
        assert move['stop_condition'] == {'type': 'distance_reached', 'distance': 3.0}


class TestFinalPhaseStopOverride:

    def _plan_with_final(self, final_phase):
        return [
            {'mode': 'straight', 'guard': 'wall', 'thresh': 3.0},
            final_phase,
        ]

    def test_stop_at_produces_front_clearance(self):
        plan = self._plan_with_final(
            {'mode': 'straight', 'guard': 'distance', 'thresh': 1.0, 'stop_at': 0.8})
        mission = phases_to_mission(plan)
        assert mission['moves'][-1]['stop_condition'] == {
            'type': 'front_clearance', 'distance': 0.8}

    def test_stop_at_distance_produces_distance_reached(self):
        plan = self._plan_with_final(
            {'mode': 'straight', 'guard': 'distance', 'thresh': 4.0, 'stop_at_distance': 4.0})
        mission = phases_to_mission(plan)
        assert mission['moves'][-1]['stop_condition'] == {
            'type': 'distance_reached', 'distance': 4.0}

    def test_stop_at_overrides_even_on_a_turn_move(self):
        """A turn move has no goal_distance to fall back on -- the override
        must use an explicit `distance` param regardless."""
        plan = self._plan_with_final(
            {'mode': 'wall_turn', 'turn_sign': 1.0, 'guard': 'turned', 'thresh': 1.0,
             'stop_at_distance': 2.0})
        mission = phases_to_mission(plan)
        last = mission['moves'][-1]
        assert 'turn' in last  # still a turn move
        assert last['stop_condition'] == {'type': 'distance_reached', 'distance': 2.0}


class TestTurnSignSign:

    def _turn_plan(self, turn_sign):
        return [{'mode': 'wall_turn', 'turn_sign': turn_sign, 'guard': 'turned', 'thresh': 1.0,
                 'stop_at': 1.0}]

    def test_negative_turn_sign_is_negative_heading_delta(self):
        mission = phases_to_mission(self._turn_plan(-1.0))
        assert mission['moves'][0]['turn']['heading_delta_deg'] < 0.0

    def test_positive_turn_sign_is_positive_heading_delta(self):
        mission = phases_to_mission(self._turn_plan(1.0))
        assert mission['moves'][0]['turn']['heading_delta_deg'] > 0.0


class TestRejectsUnvalidatedInput:
    """phases_to_mission() must never be the only line of defense -- see
    plan_translate._sanity_check's own docstring."""

    def test_rejects_non_list(self):
        with pytest.raises(PlanTranslationError):
            phases_to_mission({'not': 'a list'})

    def test_rejects_empty_list(self):
        with pytest.raises(PlanTranslationError):
            phases_to_mission([])

    def test_rejects_phase_missing_required_keys(self):
        with pytest.raises(PlanTranslationError):
            phases_to_mission([{'mode': 'straight'}])  # no guard/thresh

    def test_rejects_non_numeric_thresh(self):
        with pytest.raises(PlanTranslationError):
            phases_to_mission([{'mode': 'straight', 'guard': 'wall', 'thresh': '3.0 metri'}])

    def test_rejects_wall_turn_without_turn_sign(self):
        with pytest.raises(PlanTranslationError):
            phases_to_mission([{'mode': 'wall_turn', 'guard': 'turned', 'thresh': 1.0}])

    def test_rejects_unsupported_guard(self):
        with pytest.raises(PlanTranslationError):
            phases_to_mission([{'mode': 'straight', 'guard': 'not_a_real_guard', 'thresh': 1.0}])


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
