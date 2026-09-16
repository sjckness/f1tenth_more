"""go_to_object (schema 4.0): parsing, exclusivity, object_reached, scoring, preflight.

Pure: no rclpy. The class-list drift check reads llm's intent_v1.json from the
SOURCE tree only -- a test-time read, not a runtime dependency (llm
exec-depends on this package, so the reverse would be a cycle).

Run standalone: python3 -m pytest test/test_go_to_object_schema.py -v
"""

import json
import math
from pathlib import Path

import pytest

from f1tenth_behavior.mission.condition_eval import EvalContext, evaluate
from f1tenth_behavior.mission.mission_config import (
    MissionConfigError,
    StopCondition,
    load_mission_file,
    parse_mission,
)
from f1tenth_behavior.mission.move_scoring import build_move_outcome
from f1tenth_behavior.mission.object_classes import OBJECT_CLASSES, SOURCE_MODEL
from f1tenth_behavior.mission.preflight import required_dependencies
from f1tenth_behavior.mission.runtime import (
    GLOBAL_XY_KEY,
    ObjectApproachRecord,
    ObjectStatusSample,
)

PKG = Path(__file__).resolve().parent.parent
MISSIONS = PKG / 'missions'
INTENT_SCHEMA = (PKG.parent / 'f1tenth_intelligence' / 'llm' / 'schemas' / 'intent_v1.json')


def _object_move(**overrides):
    move = {
        'id': 'go',
        'go_to_object': {'target_class': 'person', 'speed': 0.4, 'acquire_timeout_sec': 5.0},
        'stop_condition': {'type': 'object_reached'},
        'timeout_sec': 60,
        'terminal': True,
    }
    move.update(overrides)
    return move


def _mission(*moves, schema_version='4.0'):
    return {'mission_id': 't', 'schema_version': schema_version, 'moves': list(moves)}


# ------------------------------------------------------------------- parsing

class TestParse:

    def test_a_minimal_move_gets_the_documented_defaults(self):
        spec = parse_mission(_mission(_object_move())).moves[0].go_to_object
        assert spec.target_class == 'person'
        assert spec.standoff_m == 1.0
        assert spec.lost_grace_sec == 1.5
        assert spec.speed == 0.4
        assert spec.acquire_timeout_sec == 5.0

    def test_every_field_can_be_set(self):
        spec = parse_mission(_mission(_object_move(go_to_object={
            'target_class': 'chair', 'standoff_m': 1.2, 'speed': 0.3,
            'acquire_timeout_sec': 8.0, 'lost_grace_sec': 0.0}))).moves[0].go_to_object
        assert (spec.target_class, spec.standoff_m, spec.speed,
                spec.acquire_timeout_sec, spec.lost_grace_sec) == ('chair', 1.2, 0.3, 8.0, 0.0)

    @pytest.mark.parametrize('field, value', [
        ('target_class', 'unicorn'),
        ('target_class', 7),
        ('standoff_m', 0.0),
        ('standoff_m', -1.0),
        ('speed', 0.0),
        ('speed', True),
        ('acquire_timeout_sec', 0.0),
        ('lost_grace_sec', -0.1),
    ])
    def test_invalid_values_are_rejected(self, field, value):
        spec = {'target_class': 'person', 'speed': 0.4, 'acquire_timeout_sec': 5.0}
        spec[field] = value
        with pytest.raises(MissionConfigError, match=field):
            parse_mission(_mission(_object_move(go_to_object=spec)))

    @pytest.mark.parametrize('missing', ['target_class', 'speed', 'acquire_timeout_sec'])
    def test_required_fields(self, missing):
        spec = {'target_class': 'person', 'speed': 0.4, 'acquire_timeout_sec': 5.0}
        del spec[missing]
        with pytest.raises(MissionConfigError, match=missing):
            parse_mission(_mission(_object_move(go_to_object=spec)))

    def test_an_unknown_field_is_rejected_by_name(self):
        with pytest.raises(MissionConfigError, match='standoff'):
            parse_mission(_mission(_object_move(go_to_object={
                'target_class': 'person', 'speed': 0.4, 'acquire_timeout_sec': 5.0,
                'standoff': 1.0})))


class TestExclusivity:

    @pytest.mark.parametrize('other', [
        {'goal_distance': 1.0},
        {'goal_pose': {'x': 1.0, 'y': 0.0, 'yaw': 0.0}},
        {'turn': {'heading_delta_deg': 90.0, 'speed': 0.3, 'steering': 'full_lock'}},
        {'drive': {'mode': 'straight'}},
    ])
    def test_exactly_one_goal_shape(self, other):
        with pytest.raises(MissionConfigError, match='go_to_object'):
            parse_mission(_mission(_object_move(**other)))

    def test_go_to_object_requires_object_reached(self):
        with pytest.raises(MissionConfigError, match='object_reached'):
            parse_mission(_mission(_object_move(
                stop_condition={'type': 'time_elapsed', 'duration_sec': 3.0})))

    def test_object_reached_requires_go_to_object(self):
        with pytest.raises(MissionConfigError, match='only valid on a go_to_object'):
            parse_mission(_mission({
                'id': 'd', 'goal_distance': 1.0,
                'stop_condition': {'type': 'object_reached'}, 'terminal': True}))

    def test_object_reached_takes_no_fields(self):
        with pytest.raises(MissionConfigError, match='no fields'):
            parse_mission(_mission(_object_move(
                stop_condition={'type': 'object_reached', 'tolerance': 0.1})))

    def test_timeout_is_required(self):
        move = _object_move()
        del move['timeout_sec']
        with pytest.raises(MissionConfigError, match='timeout_sec'):
            parse_mission(_mission(move))

    def test_on_object_is_rejected(self):
        with pytest.raises(MissionConfigError, match='on_object'):
            parse_mission(_mission(_object_move(on_object=[
                {'class': 'chair', 'action': 'log_only', 'message': 'x'}])))

    def test_object_reached_is_not_a_resume_condition(self):
        with pytest.raises(MissionConfigError, match='resume_condition'):
            parse_mission(_mission({
                'id': 'd', 'goal_distance': 1.0,
                'stop_condition': {'type': 'distance_reached'},
                'on_object': [{'class': 'person', 'action': 'stop_and_hold',
                               'resume_condition': {'type': 'object_reached'}}],
                'terminal': True}))

    def test_schema_4_0_requires_a_terminal_last_move(self):
        with pytest.raises(MissionConfigError, match='terminal'):
            parse_mission(_mission(_object_move(terminal=False)))


class TestShippedMissions:

    @pytest.mark.parametrize('name, cls', [('go_to_person', 'person'), ('go_to_chair', 'chair')])
    def test_they_load_as_single_object_moves(self, name, cls):
        cfg = load_mission_file(str(MISSIONS / f'{name}.json'))
        assert cfg.schema_version == '4.0'
        (move,) = cfg.moves
        assert move.go_to_object.target_class == cls
        assert move.goal_distance is None, 'no goal_distance placeholder'
        assert move.stop_condition.type == 'object_reached'
        assert move.terminal is True


class TestClassList:

    def test_it_matches_the_llm_intent_schema_enum(self):
        schema = json.loads(INTENT_SCHEMA.read_text())
        branch = next(b for b in schema['properties']['plan']['items']['oneOf']
                      if b['properties']['mode'].get('const') == 'go_to')
        assert set(branch['properties']['target']['enum']) == set(OBJECT_CLASSES)

    def test_it_is_a_snapshot_of_the_configured_model(self):
        params = (PKG.parent / 'f1tenth_params' / 'config' / 'stack_params.yaml').read_text()
        lines = params.splitlines()
        i = lines.index('yolo_model:')
        assert SOURCE_MODEL in lines[i + 1]


# ------------------------------------------------------------ object_reached

def _ctx(status, move_id='t#2/go', now=100.0, **limits):
    return EvalContext(
        now=now, move_start_time=0.0, move_start_xy=None, current_xy=None,
        detected_classes={}, min_obstacle_distance=None, front_clearance=None,
        default_distance=None, object_status=status, object_move_id=move_id, **limits)


def _status(r=0.05, move_id='t#2/go', age=0.3, watchdog=False, received=99.9):
    return ObjectStatusSample(move_id=move_id, r=r, alpha=0.1, target_age_s=age,
                              target_behind_terminal=False, goal_watchdog=watchdog,
                              received_sec=received)


REACHED = StopCondition(type='object_reached', params={})


class TestObjectReached:

    def test_r_within_tolerance_is_reached(self):
        assert evaluate(REACHED, _ctx(_status(r=0.10))) is True
        assert evaluate(REACHED, _ctx(_status(r=-0.2))) is True

    def test_r_outside_tolerance_is_not(self):
        assert evaluate(REACHED, _ctx(_status(r=0.11))) is False

    def test_another_moves_status_never_satisfies_it(self):
        """Including the same move of the previous run."""
        assert evaluate(REACHED, _ctx(_status(move_id='t#1/go'))) is False

    def test_a_stale_status_does_not(self):
        assert evaluate(REACHED, _ctx(_status(received=99.4))) is False

    def test_an_old_target_estimate_does_not(self):
        assert evaluate(REACHED, _ctx(_status(age=1.01))) is False
        assert evaluate(REACHED, _ctx(_status(age=1.01),
                                      object_reach_max_target_age_sec=2.0)) is True

    def test_a_tripped_refresh_watchdog_does_not(self):
        assert evaluate(REACHED, _ctx(_status(watchdog=True))) is False

    def test_no_status_or_no_move_does_not(self):
        assert evaluate(REACHED, _ctx(None)) is False
        assert evaluate(REACHED, _ctx(_status(), move_id=None)) is False

    def test_the_tolerance_is_the_context_value(self):
        assert evaluate(REACHED, _ctx(_status(r=0.15), object_reach_tol_m=0.2)) is True


# --------------------------------------------------------------------- scoring

def _move(standoff=1.2):
    return parse_mission(_mission(_object_move(go_to_object={
        'target_class': 'person', 'speed': 0.4, 'acquire_timeout_sec': 5.0,
        'standoff_m': standoff}))).moves[0]


def _outcome(record, stop_reason='stop_condition:object_reached', end_xy=(1.0, 0.0)):
    return build_move_outcome(
        _move(), stop_reason, 0.0, 10.0, (0.0, 0.0), end_xy, 0.0, 0.0, None,
        object_record=record)


class TestScoring:

    def _record(self, r=0.04, alpha=math.radians(37.0), outcome='reached',
                target=(2.3, 0.0)):
        rec = ObjectApproachRecord(wire_move_id='t#2/go', target_xy=target, track_id='12',
                                   outcome=outcome)
        rec.last_status = _status(r=r)._replace(alpha=alpha)
        return rec

    def test_commanded_is_the_standoff_and_actual_the_final_range(self):
        out = _outcome(self._record(r=0.04))
        assert out.move_type == 'go_to_object'
        assert out.commanded == pytest.approx(1.2)
        assert out.actual == pytest.approx(1.24)
        assert out.score_percent == pytest.approx(1.24 / 1.2 * 100.0)
        assert out.outcome == 'reached'

    def test_bearing_error_is_recorded_and_not_part_of_mismatch(self):
        out = _outcome(self._record(r=0.0, alpha=math.radians(37.0)))
        assert out.arrival_bearing_error_deg == pytest.approx(37.0)
        assert out.mismatch_flagged is False

    def test_a_range_error_beyond_tolerance_is_a_mismatch(self):
        assert _outcome(self._record(r=0.3)).mismatch_flagged is True

    def test_the_track_based_range_is_recorded_beside_it(self):
        out = _outcome(self._record(target=(2.3, 0.0)), end_xy=(1.0, 0.0))
        assert out.track_range_m == pytest.approx(1.3)
        assert out.target_track_id == '12'

    def test_without_a_status_it_is_unscored_but_keeps_the_outcome(self):
        rec = ObjectApproachRecord(wire_move_id='t#2/go', outcome='target_not_found')
        out = _outcome(rec, stop_reason='go_to_object:target_not_found')
        assert out.score_percent is None
        assert out.outcome == 'target_not_found'
        assert 'unscored' in out.note

    def test_a_timeout_with_no_recorded_outcome_says_timeout(self):
        out = _outcome(None, stop_reason='timeout:abort')
        assert out.outcome == 'timeout'


# -------------------------------------------------------------------- preflight

def test_preflight_requires_semantic_tracks_and_the_global_pose():
    cfg = parse_mission(_mission(_object_move()))
    reqs = required_dependencies(cfg)
    assert any(r.node_name == 'semantic_layer_node' for r in reqs)
    assert any(r.blackboard_key == GLOBAL_XY_KEY for r in reqs)


def test_preflight_does_not_require_them_otherwise():
    cfg = parse_mission(_mission({'id': 'd', 'goal_distance': 1.0,
                                  'stop_condition': {'type': 'distance_reached'},
                                  'terminal': True}))
    reqs = required_dependencies(cfg)
    assert not any(r.node_name == 'semantic_layer_node' for r in reqs)
