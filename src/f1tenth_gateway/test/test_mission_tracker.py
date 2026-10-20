"""The mission as the phone sees it, and the abort when every phone is gone."""

from f1tenth_gateway.mission import MissionTracker
import pytest

PATH = '/state/missions/m1.mission.json'
MOVES = {'mission_id': 'llm_1', 'moves': ['straight_1', 'turn_2', 'to_person']}


def describe(ros_state, json_path, wall_now):
    names = {'IDLE': 'idle', 'LOADED': 'loaded', 'RUNNING': 'running', 'HOLDING': 'running',
             'COMPLETE': 'finished', 'ABORTED': 'aborted'}
    state = names.get(ros_state, 'unknown')
    if state == 'loaded' and describe.started and wall_now - describe.started <= 20:
        state = 'starting'
    return {'state': state, 'id': 'm1' if json_path == PATH else None}


describe.started = None


@pytest.fixture
def tr():
    describe.started = None
    return MissionTracker(describe, auto_abort_s=2.0, reader=lambda p: MOVES)


def test_the_active_move_comes_from_the_finished_ones(tr):
    tr.on_status('LOADED', PATH, False, 0.0)
    tr.on_status('RUNNING', PATH, False, 1.0)
    v = tr.view(1.5, 101.5)
    assert v['state'] == 'running' and v['move_index'] == 1 and v['n_moves'] == 3
    assert v['move_id'] == 'straight_1' and v['elapsed_s'] == 0.5
    tr.on_move_outcome('llm_1', 'straight_1')
    tr.on_move_outcome('other_mission', 'x')          # not this mission's: ignored
    assert tr.view(2.0, 102.0)['move_index'] == 2
    tr.on_move_outcome('llm_1', 'turn_2')
    tr.on_move_outcome('llm_1', 'to_person')
    tr.on_status('COMPLETE', PATH, False, 9.0)
    v = tr.view(10.0, 110.0)
    assert v['state'] == 'finished' and v['move_index'] == 3 and v['elapsed_s'] == 8.0


def test_the_countdown_left_while_loaded(tr):
    tr.on_status('LOADED', PATH, False, 0.0)
    describe.started = 100.0
    tr.on_app_start(PATH, 15.0, 100.0)
    v = tr.view(1.0, 104.0)
    assert v['state'] == 'starting' and v['countdown_left_s'] == 11.0 and v['started_by_app']


def test_auto_abort_once_for_an_app_mission_and_never_for_others(tr):
    tr.on_status('LOADED', PATH, False, 0.0)
    describe.started = 100.0
    tr.on_app_start(PATH, 15.0, 100.0)
    tr.on_status('RUNNING', PATH, False, 15.0)
    assert not tr.auto_abort_due(16.0, 116.0, last_client_t=15.0)      # 1 s of silence
    assert tr.auto_abort_due(17.0, 117.0, last_client_t=15.0)          # 2 s
    tr.mark_auto_abort({'ok': True}, 117.0)
    assert not tr.auto_abort_due(18.0, 118.0, last_client_t=15.0)      # once
    other = MissionTracker(describe, 2.0, reader=lambda p: MOVES)
    other.on_status('RUNNING', '/elsewhere.json', False, 0.0)          # Telegram, terminal
    assert not other.auto_abort_due(60.0, 160.0, last_client_t=0.0)


def test_auto_abort_covers_the_countdown_too(tr):
    tr.on_status('LOADED', PATH, False, 0.0)
    describe.started = 100.0
    tr.on_app_start(PATH, 15.0, 100.0)
    assert tr.view(3.0, 103.0)['state'] == 'starting'
    assert tr.auto_abort_due(3.0, 103.0, last_client_t=0.5)


def test_no_auto_abort_once_the_mission_ended(tr):
    describe.started = 100.0
    tr.on_app_start(PATH, 0.0, 100.0)
    tr.on_status('COMPLETE', PATH, False, 0.0)
    assert not tr.auto_abort_due(60.0, 160.0, last_client_t=0.0)


def test_the_latch_is_carried(tr):
    tr.on_status('IDLE', '', True, 0.0)
    assert tr.view(0.0, 100.0)['estop_latched'] is True
