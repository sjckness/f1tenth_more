"""Pure decisions inside the CLI."""

from f1tenth_ctl import cli
from f1tenth_ctl.procscan import Proc


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
