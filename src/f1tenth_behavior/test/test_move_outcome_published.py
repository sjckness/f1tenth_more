"""MissionLoader publishes each finished move's outcome once, on /mission/move_outcome.

Same fake-node convention as test_mission_loader_hold_release.py. The outcome
is appended the way move_scoring.record_move_outcome does; the state watcher
tick is what publishes it.

Run standalone: python3 -m pytest test/test_move_outcome_published.py -v
"""

import math
from pathlib import Path

import pytest

from f1tenth_behavior.mission.loader import MissionLoader
from f1tenth_behavior.mission.move_scoring import MoveOutcome

MISSIONS = Path(__file__).resolve().parent.parent / 'missions'


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _Node:
    def __init__(self):
        self.publishers = {}

    def create_publisher(self, _type, topic, _qos):
        self.publishers[topic] = _Pub()
        return self.publishers[topic]

    def create_subscription(self, *a, **k):
        return None

    def create_service(self, *a, **k):
        return None

    def create_timer(self, *a, **k):
        return None

    def declare_parameter(self, _name, default):
        return type('P', (), {'value': default})()

    def get_logger(self):
        return type('L', (), {'info': lambda *a, **k: None, 'warn': lambda *a, **k: None,
                              'error': lambda *a, **k: None})()

    def get_clock(self):
        from builtin_interfaces.msg import Time
        return type('C', (), {'now': lambda self: type(
            'N', (), {'to_msg': lambda self: Time()})()})()


def _outcome(**kw):
    base = dict(move_id='move_0_go_to_person', move_type='go_to_object',
                stop_reason='stop_condition:object_reached', start_time=10.0, end_time=16.5,
                start_global_xy=None, end_global_xy=None, start_global_yaw=None,
                end_global_yaw=None, commanded=1.2, actual=1.28, score_percent=None,
                outcome='reached', arrival_bearing_error_deg=2.9, track_gap_m=None,
                wire_move_id='go_to_person#2/move_0_go_to_person')
    base.update(kw)
    return MoveOutcome(**base)


@pytest.fixture
def loader():
    node = _Node()
    loader = MissionLoader(node)
    assert loader._load(str(MISSIONS / 'go_to_person.json'))[0]
    return loader, node.publishers['/mission/move_outcome']


def test_a_new_outcome_is_published_once(loader):
    loader, pub = loader
    state = loader.blackboard.mission
    state.move_outcomes.append(_outcome())
    loader._on_state_watch_tick()
    loader._on_state_watch_tick()
    (msg,) = pub.msgs
    assert (msg.mission_id, msg.move_type, msg.outcome) == ('go_to_person', 'go_to_object',
                                                            'reached')
    assert msg.wire_move_id == 'go_to_person#2/move_0_go_to_person'
    assert msg.duration_s == pytest.approx(6.5)
    assert msg.actual == pytest.approx(1.28)
    assert math.isnan(msg.score_percent)
    assert math.isnan(msg.track_gap_m)


def test_a_reload_starts_counting_again(loader):
    loader, pub = loader
    loader.blackboard.mission.move_outcomes.append(_outcome())
    loader._on_state_watch_tick()
    assert loader._load(str(MISSIONS / 'go_to_person.json'))[0]
    loader.blackboard.mission.move_outcomes.append(_outcome(outcome='target_lost'))
    loader._on_state_watch_tick()
    assert [m.outcome for m in pub.msgs] == ['reached', 'target_lost']
