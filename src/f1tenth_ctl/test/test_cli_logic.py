"""Pure decisions inside the CLI."""

from f1tenth_ctl import cli
from f1tenth_ctl.procscan import Proc
from f1tenth_ctl.rosprobe import choose_reset_path


def test_a_stray_process_is_evidence():
    seen = cli.unmanaged_evidence(['/n'], [], [Proc(7, 1, 0, 'x', ['x'])], {}, 100.0)
    assert seen == {'pids': [7]}


def test_a_node_in_the_graph_is_evidence():
    assert cli.unmanaged_evidence(['/n'], ['/a', '/n'], [], {}, 100.0) == {'nodes': ['/n']}


def test_a_node_left_over_from_a_recent_stop_is_not():
    desired = {'desired': 'stopped', 'time': 100.0}
    assert cli.unmanaged_evidence(['/n'], ['/n'], [], desired, 110.0) == {}
    after = 100.0 + cli.GRAPH_GRACE_S + 1
    assert cli.unmanaged_evidence(['/n'], ['/n'], [], desired, after) == {'nodes': ['/n']}


def test_nothing_seen():
    assert cli.unmanaged_evidence(['/n'], None, [], {}, 0.0) == {}


def test_reset_path_prefers_a_reset_service():
    assert choose_reset_path('/mission/reset_estop', True, True) == 'reset_service'


def test_reset_path_restarts_behavior_when_there_is_no_service():
    # Today: no reset service, the supervisor's /restart_component is there.
    assert choose_reset_path('', False, True) == 'restart_behavior'
    # A configured service that is not up does not count.
    assert choose_reset_path('/mission/reset_estop', False, True) == 'restart_behavior'


def test_reset_path_falls_back_to_a_stack_restart():
    assert choose_reset_path('', False, False) == 'stack_restart'


def _step(status, msg='m', ms=10):
    return {'status': status, 'ok': status == 'ok', 'message': msg, 'duration_ms': ms}


def test_estop_held_by_zero_drive_and_the_latch():
    res = cli.estop_result({'zero_drive': _step('ok'), 'emergency_stop': _step('ok'),
                            'abort_mission': _step('failed', 'service not available')})
    assert res['car_held'] is True and res['ok'] is True
    assert list(res['steps']) == ['zero_drive', 'emergency_stop', 'abort_mission']
    for st in res['steps'].values():
        assert set(st) >= {'status', 'ok', 'message', 'duration_ms'}


def test_estop_held_by_zero_drive_and_the_abort():
    res = cli.estop_result({'zero_drive': _step('ok'),
                            'emergency_stop': _step('failed', 'service not available'),
                            'abort_mission': _step('ok', 'no mission to abort (state=IDLE)')})
    assert res['car_held'] is True


def test_estop_not_held_without_zero_drive():
    res = cli.estop_result({'zero_drive': _step('failed', 'no subscriber matched'),
                            'emergency_stop': _step('ok'), 'abort_mission': _step('ok')})
    assert res['car_held'] is False and res['ok'] is False


def test_estop_not_held_when_neither_service_answered():
    res = cli.estop_result({'zero_drive': _step('ok'), 'emergency_stop': _step('failed'),
                            'abort_mission': _step('failed')})
    assert res['car_held'] is False


def test_estop_that_never_ran_is_all_skipped():
    res = cli.estop_result({}, 'discovery server down: nothing reachable')
    assert res['car_held'] is False and res['error'].startswith('discovery server down')
    assert {st['status'] for st in res['steps'].values()} == {'skipped'}


def test_no_mission_to_abort_counts_as_ok():
    from f1tenth_ctl.rosprobe import abort_ok, step_record
    assert abort_ok('no mission to abort (state=IDLE)')
    assert not abort_ok('aborted mission x')    # a real abort answers success=True anyway
    rec = step_record('ok', 'x', 0.0123, call_ms=5)
    assert rec == {'status': 'ok', 'ok': True, 'message': 'x', 'duration_ms': 12, 'call_ms': 5}
