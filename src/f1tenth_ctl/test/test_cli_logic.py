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
