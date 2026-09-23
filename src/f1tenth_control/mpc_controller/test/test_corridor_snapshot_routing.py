"""corridor_debug.jsonl: where a snapshot goes, and that nothing is truncated.

The bug this pins: the snapshot file used to be a single fixed path opened
'w' at every node start, so each launch destroyed the previous run's
snapshots, and it was never copied anywhere. It was found at 0 bytes.

Same duck-typed stand-in shape as test_campaign_status.py: the methods under
test are MPCController's own, bound to an object carrying only what they read.

Run standalone: python3 -m pytest test/test_corridor_snapshot_routing.py -v
"""

import json

import numpy as np
import pytest

from mpc_controller.MPC_corr import MPCController


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, text):
        self.messages.append(('info', text))

    def warn(self, text, **kwargs):
        self.messages.append(('warn', text))


class FakeClock:
    class _Now:
        nanoseconds = 1_700_000_000_000_000_000

    def now(self):
        return self._Now()


class FakeNode:
    """Only what _on_campaign_status and save_corridor_snapshot read."""

    def __init__(self, fallback_path):
        self.save_corridor_debug = True
        self.corridor_log_path = fallback_path
        fallback_path.parent.mkdir(parents=True, exist_ok=True)
        self.corridor_log_file = open(fallback_path, 'a', encoding='utf-8')
        self._corridor_test_file = None
        self._corridor_test_dir = None
        self.x, self.y, self.yaw, self.v = 1.0, 2.0, 0.3, 0.5
        self.cached_pref_nom = np.array([3.0, 4.0])
        self._logger = FakeLogger()
        self._clock = FakeClock()

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    # the methods under test, bound to this stand-in
    on_status = MPCController._on_campaign_status
    snapshot = MPCController.save_corridor_snapshot

    def close(self):
        self.corridor_log_file.close()
        if self._corridor_test_file is not None:
            self._corridor_test_file.close()


class FakeMsg:
    def __init__(self, payload):
        self.data = payload if isinstance(payload, str) else json.dumps(payload)


def corridor():
    n = 4
    return {
        'xc': np.linspace(0, 3, n), 'yc': np.zeros(n),
        'xL': np.linspace(0, 3, n), 'yL': np.full(n, 0.5),
        'xR': np.linspace(0, 3, n), 'yR': np.full(n, -0.5),
        'Pend': np.array([3.0, 0.0]),
    }


def status(campaign_dir, state='open', mission='M01',
           test_id='P001-R001-20260101T000000'):
    return FakeMsg({'state': state, 'campaign_dir': str(campaign_dir),
                    'mission': mission, 'test_id': test_id})


def lines(path):
    if not path.exists():
        return []
    with open(path, encoding='utf-8') as fh:
        return [json.loads(line) for line in fh if line.strip()]


@pytest.fixture
def node(tmp_path):
    n = FakeNode(tmp_path / 'corridors_jsons' / 'corridor_debug_20260101T000000.jsonl')
    yield n
    n.close()


# ---------------------------------------------------------------------------
# routing
# ---------------------------------------------------------------------------

def test_with_no_test_open_a_snapshot_goes_to_the_per_run_file(node, tmp_path):
    node.snapshot(corridor(), [])
    assert len(lines(node.corridor_log_path)) == 1


def test_an_open_test_takes_the_snapshots(node, tmp_path):
    node.on_status(status(tmp_path))
    node.snapshot(corridor(), [])
    test_file = (tmp_path / 'M01' / 'P001-R001-20260101T000000'
                 / 'corridor_debug.jsonl')
    assert len(lines(test_file)) == 1
    # and NOT the fallback: a snapshot belongs to exactly one run
    assert lines(node.corridor_log_path) == []


def test_closing_the_test_returns_to_the_per_run_file(node, tmp_path):
    node.on_status(status(tmp_path))
    node.snapshot(corridor(), [])
    node.on_status(status(tmp_path, state='closed'))
    node.snapshot(corridor(), [])

    test_file = (tmp_path / 'M01' / 'P001-R001-20260101T000000'
                 / 'corridor_debug.jsonl')
    assert len(lines(test_file)) == 1
    assert len(lines(node.corridor_log_path)) == 1


def test_a_second_test_gets_its_own_file(node, tmp_path):
    node.on_status(status(tmp_path, test_id='P001-R001-20260101T000000'))
    node.snapshot(corridor(), [])
    node.on_status(status(tmp_path, test_id='P001-R002-20260101T000100'))
    node.snapshot(corridor(), [])
    node.snapshot(corridor(), [])

    first = tmp_path / 'M01' / 'P001-R001-20260101T000000' / 'corridor_debug.jsonl'
    second = tmp_path / 'M01' / 'P001-R002-20260101T000100' / 'corridor_debug.jsonl'
    assert len(lines(first)) == 1
    assert len(lines(second)) == 2


def test_repeated_status_for_the_same_test_does_not_reopen(node, tmp_path):
    """The status is published every tick while a test is open. Reopening on
    each one would be wasteful; reopening in 'w' would erase the test."""
    node.on_status(status(tmp_path))
    node.snapshot(corridor(), [])
    handle = node._corridor_test_file
    for _ in range(5):
        node.on_status(status(tmp_path))
    assert node._corridor_test_file is handle
    node.snapshot(corridor(), [])
    test_file = (tmp_path / 'M01' / 'P001-R001-20260101T000000'
                 / 'corridor_debug.jsonl')
    assert len(lines(test_file)) == 2


# ---------------------------------------------------------------------------
# nothing is ever truncated
# ---------------------------------------------------------------------------

def test_reopening_a_test_appends_rather_than_truncating(node, tmp_path):
    """THE BUG. A second pass over the same test folder -- a relaunched node,
    a resumed test -- must not erase what is already there."""
    test_file = (tmp_path / 'M01' / 'P001-R001-20260101T000000'
                 / 'corridor_debug.jsonl')
    node.on_status(status(tmp_path))
    node.snapshot(corridor(), [])
    node.on_status(status(tmp_path, state='closed'))
    node.on_status(status(tmp_path))          # same test again
    node.snapshot(corridor(), [])
    assert len(lines(test_file)) == 2


def test_a_second_node_run_does_not_erase_the_first(tmp_path):
    """The per-run file carries the node's start time and is opened 'a', so
    two runs cannot collide -- and could not truncate each other if they did."""
    path = tmp_path / 'corridors_jsons' / 'corridor_debug_20260101T000000.jsonl'
    first = FakeNode(path)
    first.snapshot(corridor(), [])
    first.close()

    second = FakeNode(path)               # same name on purpose: the hard case
    second.snapshot(corridor(), [])
    second.close()

    assert len(lines(path)) == 2


# ---------------------------------------------------------------------------
# the record, and robustness
# ---------------------------------------------------------------------------

def test_a_snapshot_carries_a_timestamp(node):
    node.snapshot(corridor(), [(1.0, 2.0, 0.3)])
    record = lines(node.corridor_log_path)[0]
    assert record['t'] == pytest.approx(1.7e9, rel=1e-6)
    assert record['robot']['x'] == 1.0
    assert record['obstacles_world'] == [{'x': 1.0, 'y': 2.0, 'r': 0.3}]
    assert len(record['corridor']['xc']) == 4


@pytest.mark.parametrize('payload', [
    'not json at all',
    {'state': 'open'},                                  # no folder
    {'state': 'open', 'campaign_dir': '/tmp', 'mission': 'M01'},   # no test_id
    {},
])
def test_a_malformed_status_never_raises_and_keeps_logging(node, payload):
    """This runs on the control node: a bad status costs a destination, not
    the node."""
    node.on_status(FakeMsg(payload))
    node.snapshot(corridor(), [])
    assert len(lines(node.corridor_log_path)) == 1


def test_a_status_for_an_unwritable_folder_is_survived(node):
    node.on_status(FakeMsg({'state': 'open', 'campaign_dir': '/proc/nope',
                            'mission': 'M01', 'test_id': 'P001-R001-20260101T000000'}))
    node.snapshot(corridor(), [])
    assert len(lines(node.corridor_log_path)) == 1
    assert any(kind == 'warn' for kind, _ in node.get_logger().messages)
