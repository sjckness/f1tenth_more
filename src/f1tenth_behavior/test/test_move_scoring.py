"""mission/move_scoring.py tests -- score_turn/score_straight/is_mismatch/
build_move_outcome/aggregate_score are plain functions, zero rclpy
dependency, same convention test_condition_eval.py/test_preflight.py already
establish. record_move_outcome()/write_mission_summary() (the two functions
that touch a real logger/filesystem) are exercised too, with a fake logger
and tmp_path -- still no rclpy/py_trees required.

Run standalone: python3 -m pytest test/test_move_scoring.py -v
"""

import json
import math

import pytest

from f1tenth_behavior.mission.mission_config import GoalPose, Move, StopCondition, TurnSpec
from f1tenth_behavior.mission.move_scoring import (
    aggregate_score,
    build_move_outcome,
    is_mismatch,
    record_move_outcome,
    score_straight,
    score_turn,
    write_mission_summary,
)


class _FakeLogger:
    def __init__(self):
        self.infos = []
        self.warns = []
        self.errors = []

    def info(self, msg):
        self.infos.append(msg)

    def warn(self, msg):
        self.warns.append(msg)

    def error(self, msg):
        self.errors.append(msg)


class _FakeState:
    def __init__(self, move_start_time=0.0, move_start_global_xy=(0.0, 0.0), move_start_global_yaw=0.0):
        self.move_start_time = move_start_time
        self.move_start_global_xy = move_start_global_xy
        self.move_start_global_yaw = move_start_global_yaw
        self.move_outcomes = []


def _turn_move(move_id='t1', heading_delta_deg=90.0):
    return Move(
        id=move_id,
        stop_condition=StopCondition(type='orientation_delta', params={'value': abs(heading_delta_deg)}),
        turn=TurnSpec(heading_delta_deg=heading_delta_deg, speed=0.5, steering='full_lock'),
    )


def _distance_move(move_id='d1', goal_distance=1.0, stop_type='distance_reached', stop_params=None):
    return Move(
        id=move_id,
        stop_condition=StopCondition(type=stop_type, params=stop_params or {}),
        goal_distance=goal_distance,
    )


def _pose_move(move_id='p1'):
    return Move(
        id=move_id,
        stop_condition=StopCondition(type='goal_reached', params={}),
        goal_pose=GoalPose(x=1.0, y=0.0, yaw=0.0),
    )


class TestScoreTurn:

    def test_exact_match_is_100_percent(self):
        assert score_turn(90.0, 90.0) == pytest.approx(100.0)

    def test_undershoot_below_100(self):
        assert score_turn(90.0, 87.0) == pytest.approx(96.666, abs=1e-2)

    def test_overshoot_above_100(self):
        assert score_turn(90.0, 93.0) == pytest.approx(103.333, abs=1e-2)

    def test_unsigned_direction_does_not_matter(self):
        # a -90 (right) turn measured as -90 actual scores the same as +90/+90
        assert score_turn(-90.0, -90.0) == pytest.approx(100.0)

    def test_zero_commanded_degenerate_case(self):
        assert score_turn(0.0, 0.0) == 100.0
        assert score_turn(0.0, 5.0) == math.inf


class TestScoreStraight:

    def test_exact_forward_match_is_100_percent(self):
        actual, score = score_straight(1.0, (0.0, 0.0), (1.0, 0.0), intended_heading_rad=0.0)
        assert actual == pytest.approx(1.0)
        assert score == pytest.approx(100.0)

    def test_undershoot_below_100(self):
        actual, score = score_straight(1.0, (0.0, 0.0), (0.94, 0.0), intended_heading_rad=0.0)
        assert actual == pytest.approx(0.94)
        assert score == pytest.approx(94.0)

    def test_overshoot_above_100(self):
        _, score = score_straight(1.0, (0.0, 0.0), (1.1, 0.0), intended_heading_rad=0.0)
        assert score == pytest.approx(110.0)

    def test_sideways_drift_projects_onto_intended_heading(self):
        # Moved diagonally, but only the component along the intended
        # (world +x) heading counts -- pure sideways (+y) drift contributes 0.
        actual, score = score_straight(1.0, (0.0, 0.0), (0.0, 5.0), intended_heading_rad=0.0)
        assert actual == pytest.approx(0.0)
        assert score == pytest.approx(0.0)

    def test_backward_motion_relative_to_intent_is_negative(self):
        actual, score = score_straight(1.0, (0.0, 0.0), (-0.2, 0.0), intended_heading_rad=0.0)
        assert actual == pytest.approx(-0.2)
        assert score < 0

    def test_non_axis_aligned_intended_heading(self):
        heading = math.radians(90.0)  # facing +y
        actual, score = score_straight(2.0, (0.0, 0.0), (0.0, 1.8), intended_heading_rad=heading)
        assert actual == pytest.approx(1.8)
        assert score == pytest.approx(90.0)


class TestIsMismatch:

    def test_within_tolerance_not_flagged(self):
        # 90 commanded, 87 actual -- 3.3% error, well under 10% (and above
        # the 3deg floor) -- not flagged.
        assert is_mismatch(90.0, 87.0, floor=3.0) is False

    def test_beyond_tolerance_flagged(self):
        # 90 commanded, 70 actual -- 22% error, beyond 10% -- flagged.
        assert is_mismatch(90.0, 70.0, floor=3.0) is True

    def test_small_commanded_value_uses_the_floor_not_10_percent(self):
        # 0.1m commanded: 10% would be 0.01m, an unreasonably tight bound --
        # the 0.05m floor applies instead.
        assert is_mismatch(0.1, 0.14, floor=0.05) is False  # 0.04m error, under the 0.05 floor
        assert is_mismatch(0.1, 0.20, floor=0.05) is True   # 0.10m error, beyond the floor


class TestBuildMoveOutcome:

    def test_turn_move_scored_from_global_turn_accum(self):
        move = _turn_move(heading_delta_deg=90.0)
        outcome = build_move_outcome(
            move=move, stop_reason='stop_condition:orientation_delta',
            start_time=0.0, end_time=1.5,
            start_global_xy=(0.0, 0.0), end_global_xy=(0.0, 0.0),
            start_global_yaw=0.0, end_global_yaw=math.radians(90.0),
            global_turn_accum_deg=90.0,
        )
        assert outcome.move_type == 'turn'
        assert outcome.commanded == pytest.approx(90.0)
        assert outcome.actual == pytest.approx(90.0)
        assert outcome.score_percent == pytest.approx(100.0)
        assert outcome.mismatch_flagged is False

    def test_turn_move_unscored_without_global_turn_accum(self):
        move = _turn_move()
        outcome = build_move_outcome(
            move=move, stop_reason='stop_condition:orientation_delta',
            start_time=0.0, end_time=1.5,
            start_global_xy=(0.0, 0.0), end_global_xy=(0.0, 0.0),
            start_global_yaw=0.0, end_global_yaw=None,
            global_turn_accum_deg=None,
        )
        assert outcome.score_percent is None
        assert outcome.note is not None

    def test_distance_move_scored_from_global_pose_delta(self):
        move = _distance_move(goal_distance=1.0)
        outcome = build_move_outcome(
            move=move, stop_reason='stop_condition:distance_reached',
            start_time=0.0, end_time=2.0,
            start_global_xy=(0.0, 0.0), end_global_xy=(0.94, 0.0),
            start_global_yaw=0.0, end_global_yaw=0.0,
            global_turn_accum_deg=None,
        )
        assert outcome.move_type == 'goal_distance'
        assert outcome.actual == pytest.approx(0.94)
        assert outcome.score_percent == pytest.approx(94.0)
        # 0.06m error vs. tolerance=max(10% of 1.0m, 0.05m floor)=0.10m -- under
        # tolerance, not flagged.
        assert outcome.mismatch_flagged is False

    def test_distance_reached_override_is_commanded_not_goal_distance(self):
        # goal_distance=10.0 is a safety ceiling; stop_condition's own
        # `distance: 0.5` override is the ACTUAL commanded target -- found
        # live generating this pass's own sample summary (see move_scoring.
        # py's own comment on this exact case).
        move = _distance_move(
            goal_distance=10.0, stop_type='distance_reached', stop_params={'distance': 0.5})
        outcome = build_move_outcome(
            move=move, stop_reason='stop_condition:distance_reached',
            start_time=0.0, end_time=1.0,
            start_global_xy=(0.0, 0.0), end_global_xy=(0.47, 0.0),
            start_global_yaw=0.0, end_global_yaw=0.0,
            global_turn_accum_deg=None,
        )
        assert outcome.commanded == pytest.approx(0.5)
        assert outcome.actual == pytest.approx(0.47)
        assert outcome.score_percent == pytest.approx(94.0)

    def test_goal_reached_type_uses_goal_distance_as_commanded(self):
        move = _distance_move(goal_distance=1.0, stop_type='goal_reached')
        outcome = build_move_outcome(
            move=move, stop_reason='stop_condition:goal_reached',
            start_time=0.0, end_time=1.0,
            start_global_xy=(0.0, 0.0), end_global_xy=(1.0, 0.0),
            start_global_yaw=0.0, end_global_yaw=0.0,
            global_turn_accum_deg=None,
        )
        assert outcome.commanded == pytest.approx(1.0)
        assert outcome.score_percent == pytest.approx(100.0)

    def test_sensor_driven_stop_condition_is_not_scored(self):
        # goal_distance=10.0 is only ever a safety ceiling for a front_
        # clearance-gated approach -- there is no predetermined "commanded"
        # distance to score against, so this must stay unscored, not
        # silently graded against the ceiling.
        move = _distance_move(
            goal_distance=10.0, stop_type='front_clearance', stop_params={'distance': 1.5})
        outcome = build_move_outcome(
            move=move, stop_reason='stop_condition:front_clearance',
            start_time=0.0, end_time=3.0,
            start_global_xy=(0.0, 0.0), end_global_xy=(3.2, 0.0),
            start_global_yaw=0.0, end_global_yaw=0.0,
            global_turn_accum_deg=None,
        )
        assert outcome.commanded is None
        assert outcome.score_percent is None
        assert 'sensor/time signal' in outcome.note

    def test_goal_pose_move_is_never_scored(self):
        move = _pose_move()
        outcome = build_move_outcome(
            move=move, stop_reason='stop_condition:goal_reached',
            start_time=0.0, end_time=2.0,
            start_global_xy=(0.0, 0.0), end_global_xy=(1.0, 0.0),
            start_global_yaw=0.0, end_global_yaw=0.0,
            global_turn_accum_deg=None,
        )
        assert outcome.move_type == 'goal_pose'
        assert outcome.score_percent is None
        assert outcome.commanded is None


class TestRecordMoveOutcomeAndSummary:

    def test_record_appends_to_state_and_logs(self):
        state = _FakeState()
        logger = _FakeLogger()
        move = _turn_move(heading_delta_deg=90.0)
        outcome = record_move_outcome(
            state, logger, move, stop_reason='stop_condition:orientation_delta', now=1.5,
            end_global_xy=(0.0, 0.0), end_global_yaw=math.radians(90.0),
            global_turn_accum_deg=90.0,
        )
        assert state.move_outcomes == [outcome]
        assert any('score=100.0%' in m for m in logger.infos)
        assert logger.warns == []

    def test_record_warns_on_mismatch(self):
        state = _FakeState()
        logger = _FakeLogger()
        move = _turn_move(heading_delta_deg=90.0)
        record_move_outcome(
            state, logger, move, stop_reason='stop_condition:orientation_delta', now=1.5,
            end_global_xy=(0.0, 0.0), end_global_yaw=math.radians(60.0),
            global_turn_accum_deg=60.0,  # well beyond the 10%/3deg tolerance
        )
        assert len(logger.warns) == 1
        assert 'mismatch' in logger.warns[0]

    def test_write_mission_summary_produces_valid_json(self, tmp_path, monkeypatch):
        import f1tenth_behavior.mission.move_scoring as move_scoring_mod

        monkeypatch.setattr(
            move_scoring_mod, '_resolve_mission_reports_dir', lambda: tmp_path)
        logger = _FakeLogger()
        state = _FakeState()
        move = _turn_move(heading_delta_deg=90.0)
        outcome = record_move_outcome(
            state, logger, move, stop_reason='stop_condition:orientation_delta', now=1.5,
            end_global_xy=(0.0, 0.0), end_global_yaw=math.radians(90.0),
            global_turn_accum_deg=90.0,
        )
        path = write_mission_summary(
            mission_id='unit_test_mission', outcomes=state.move_outcomes,
            final_state='COMPLETE', mission_start_wall_time=0.0, logger=logger,
        )
        assert path is not None
        assert path.exists()
        payload = json.loads(path.read_text())
        assert payload['mission_id'] == 'unit_test_mission'
        assert payload['aggregate_score_percent'] == pytest.approx(100.0)
        assert len(payload['moves']) == 1

    def test_aggregate_score_excludes_unscored_moves(self):
        outcomes = [
            record_move_outcome(
                _FakeState(), _FakeLogger(), _turn_move('t1', 90.0),
                'stop_condition:orientation_delta', 1.0, (0.0, 0.0), math.radians(90.0), 90.0,
            ),
            record_move_outcome(
                _FakeState(), _FakeLogger(), _pose_move('p1'),
                'stop_condition:goal_reached', 1.0, (1.0, 0.0), 0.0, None,
            ),
        ]
        assert aggregate_score(outcomes) == pytest.approx(100.0)  # only t1 counts

    def test_aggregate_score_none_when_nothing_scorable(self):
        outcomes = [
            record_move_outcome(
                _FakeState(), _FakeLogger(), _pose_move('p1'),
                'stop_condition:goal_reached', 1.0, (1.0, 0.0), 0.0, None,
            ),
        ]
        assert aggregate_score(outcomes) is None


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
