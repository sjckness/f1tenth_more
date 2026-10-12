"""mpc_corr's BLOCKED verdict ends the mission at once (M02 campaign fix A3).

R011 sat 33 s of an 80 s move timeout with the car not moving. mpc_corr now
decides BLOCKED (no progress, steering lock without progress, no feasible
plan at >= 0.4 m/s), stops, and publishes /mpc/blocked; CheckStopCondition
must abort the mission on it -- hold, outcome mpc_blocked:<reason>, summary
BLOCKED -- and the reason must reach /test/mission_event.

Run standalone: python3 -m pytest test/test_mpc_blocked_ends_mission.py -v
"""

import json
import time
from types import SimpleNamespace

import pytest

py_trees = pytest.importorskip('py_trees')

import f1tenth_behavior.behaviours.check_stop_condition as check_module  # noqa: E402
from f1tenth_behavior.behaviours.check_stop_condition import (  # noqa: E402
    CheckStopCondition,
)
from f1tenth_behavior.mission.loader import MissionLoader  # noqa: E402
from f1tenth_behavior.mission.object_handler import object_move_wire_id  # noqa: E402
from f1tenth_behavior.mission.runtime import MissionState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from test_every_abort_holds import MOVES, _check, _state  # noqa: E402

S = py_trees.common.Status


@pytest.fixture
def summaries(monkeypatch):
    calls = []
    monkeypatch.setattr(check_module, 'write_mission_summary',
                        lambda *a, **k: calls.append(a))
    return calls


def _record(reason='no_feasible_plan', move_id=None):
    return {'reason': reason, 'detail': {'for_s': 1.0, 'no_side_fits': []},
            'move_kind': 'drive:straight', 'move_id': move_id, 't_ros': 1e12}


def _deliver(behaviour, record, received):
    behaviour._blocked_cb(String(data=json.dumps(record)))
    behaviour._blocked_received = received


@pytest.mark.parametrize('move_type', sorted(MOVES))
def test_a_blocked_verdict_aborts_the_mission_with_its_reason(move_type, summaries):
    state = _state(move_type)
    check = _check(state)
    assert check.update() == S.RUNNING          # the move starts here
    move_id = None
    if move_type == 'go_to_object':
        move_id = object_move_wire_id('t', state.run_generation, 'm')
    _deliver(check, _record(move_id=move_id), state.move_start_time + 1.0)
    assert check.update() == S.FAILURE
    assert state.state == MissionState.BLOCKED      # its own outcome (review item 4)
    assert state.abort_result == 'BLOCKED'
    assert 'no_feasible_plan' in state.abort_reason
    assert check.hold_pub.msgs and check.hold_pub.msgs[-1].data is True
    assert state.move_outcomes[-1].stop_reason == 'mpc_blocked:no_feasible_plan'
    assert summaries and summaries[-1][2] == 'BLOCKED'


def test_a_record_received_inside_the_grace_is_not_this_moves(summaries):
    """A verdict for the previous move can still be in flight at a move change."""
    state = _state('drive')
    check = _check(state)
    check.update()
    _deliver(check, _record(), state.move_start_time + 0.1)
    assert check.update() == S.RUNNING
    assert state.state == MissionState.RUNNING


def test_a_record_stamped_before_the_move_started_is_ignored(summaries):
    state = _state('drive')
    check = _check(state)
    check.update()
    check._move_ros_start = 100.0
    rec = _record()
    rec['t_ros'] = 99.0
    _deliver(check, rec, state.move_start_time + 1.0)
    assert check.update() == S.RUNNING


def test_an_object_move_ignores_another_moves_record(summaries):
    state = _state('go_to_object')
    check = _check(state)
    check.update()
    _deliver(check, _record(move_id='some/other/move'), state.move_start_time + 1.0)
    assert check.update() == S.RUNNING


def test_garbage_on_the_topic_is_ignored():
    check = CheckStopCondition()
    check._blocked_cb(String(data='not json'))
    check._blocked_cb(String(data=json.dumps({'detail': {}})))
    assert check._blocked_record is None


def test_the_reason_reaches_the_test_event(summaries):
    """A BT-internal abort used to publish reason '' on /test/mission_event."""
    state = _state('drive')
    check = _check(state)
    check.update()
    _deliver(check, _record(reason='no_progress'), state.move_start_time + 1.0)
    check.update()
    loader = SimpleNamespace(_pending_event_reason='', _last_event_key=None,
                             test_event_pub=SimpleNamespace(msgs=[]))
    loader.test_event_pub.publish = loader.test_event_pub.msgs.append
    MissionLoader._publish_test_event(loader, state)
    payload = json.loads(loader.test_event_pub.msgs[-1].data)
    assert payload['event'] == 'mission_blocked'
    assert payload['reason'].startswith('BLOCKED: no_progress')


def test_a_timeout_abort_now_carries_its_reason_too(summaries):
    """And stays an ABORT, not a BLOCKED."""
    state = _state('drive')
    check = _check(state)
    check.update()
    state.move_start_time = time.monotonic() - 11.0
    assert check.update() == S.FAILURE
    assert state.abort_result == 'ABORTED'
    assert state.state == MissionState.ABORTED
    assert state.abort_reason.startswith("timeout: move 'm'")


def test_a_new_load_clears_the_reason():
    state = _state('drive')
    state.abort(reason='x', result='BLOCKED')
    state.load(state.config, time.monotonic())
    assert state.abort_reason == '' and state.abort_result == 'ABORTED'
