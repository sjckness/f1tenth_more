"""/safety/event: one 'estop' per emergency stop, on the tick it begins.

The handler is behavior_executor_node's post-tick hook (safety_event.py). The
conditions under it are levels, re-evaluated every tick, so what is pinned
here is the edge: one event when the emergency lane becomes the active lane,
none while it stays active, a new one when it clears and trips again -- and
that the handler cannot raise into the tree.

The tree is a stand-in carrying only what the handler reads (names, statuses,
children), the same shape py_trees gives it: root Selector -> lanes ->
emergency Sequence -> emergency_condition Selector -> condition leaves.

Run standalone: python3 -m pytest test/test_safety_event_published.py -v
"""

import json

import pytest

py_trees = pytest.importorskip('py_trees')

from f1tenth_behavior.safety_event import (  # noqa: E402
    active_lane, emergency_trip, make_safety_event_publisher)

S = py_trees.common.Status


class _B:
    def __init__(self, name, status=S.INVALID, children=(), **attrs):
        self.name = name
        self.status = status
        self.children = list(children)
        self.__dict__.update(attrs)


class _Tree:
    def __init__(self):
        self.proximity = _B('IsProximityTooClose')
        self.battery = _B('IsBatteryLow')
        self.overheat = _B('IsSystemOverheated')
        self.condition = _B('emergency_condition',
                            children=[self.battery, self.proximity, self.overheat])
        self.emergency = _B('emergency', children=[self.condition, _B('Stop')])
        self.mission = _B('mission')
        self.root = _B('root', children=[self.emergency, self.mission, _B('navigation')])

    def tick(self, tripped=None, reason=''):
        """One tick: `tripped` (a leaf) wins the emergency lane, or the mission runs."""
        for leaf in self.condition.children:
            leaf.status = S.FAILURE
            leaf.tripped_reason = ''
        if tripped is not None:
            tripped.status = S.SUCCESS
            tripped.tripped_reason = reason
            self.emergency.status = S.SUCCESS
            self.mission.status = S.INVALID
        else:
            self.emergency.status = S.FAILURE
            self.mission.status = S.SUCCESS


class _Publisher:
    def __init__(self, fail=False):
        self.sent = []
        self.fail = fail

    def publish(self, msg):
        if self.fail:
            raise RuntimeError('publisher context is invalid')
        self.sent.append(json.loads(msg.data))


class _Node:
    def __init__(self):
        self.warnings = []

    def get_logger(self):
        return self

    def warn(self, text, **_):
        self.warnings.append(text)


def _run(ticks, fail=False):
    tree, pub, node = _Tree(), _Publisher(fail), _Node()
    handler = make_safety_event_publisher(node, pub)
    for tick in ticks:
        tree.tick(*tick(tree))
        handler(tree)
    return pub.sent, node.warnings


def test_one_event_when_the_lane_trips_none_while_it_holds():
    sent, _ = _run([lambda t: (None,)] * 3 + [lambda t: (t.proximity,)] * 5)
    assert sent == [{'event': 'estop', 'cause': 'IsProximityTooClose',
                     'source': 'behavior_tree', 'lane': 'emergency'}]


def test_a_second_trip_after_the_lane_clears_is_a_second_event():
    sent, _ = _run([lambda t: (t.proximity,)] * 2 + [lambda t: (None,)]
                   + [lambda t: (t.battery,)] * 2)
    assert [m['cause'] for m in sent] == ['IsProximityTooClose', 'IsBatteryLow']


def test_the_cause_carries_the_tripped_reason_like_tree_status():
    sent, _ = _run([lambda t: (t.overheat, 'cpu 92.0 C > 90.0 C')])
    assert sent[0]['cause'] == 'IsSystemOverheated: cpu 92.0 C > 90.0 C'


def test_a_new_cause_while_the_lane_is_already_active_is_not_a_new_event():
    # the documented blind spot: the edge is on the lane, not on each condition
    sent, _ = _run([lambda t: (t.proximity,), lambda t: (t.battery,)])
    assert len(sent) == 1


def test_no_stop_no_event():
    sent, warnings = _run([lambda t: (None,)] * 10)
    assert sent == [] and warnings == []


def test_a_failing_publisher_never_raises_into_the_tree():
    sent, warnings = _run([lambda t: (t.proximity,)], fail=True)
    assert sent == [] and len(warnings) == 1


def test_the_helpers_read_the_tree_the_way_tree_status_always_did():
    tree = _Tree()
    tree.tick(None)
    assert active_lane(tree.root) == 'mission'
    assert emergency_trip(tree.root) == ''
    tree.tick(tree.battery)
    assert active_lane(tree.root) == 'emergency'
    assert emergency_trip(tree.root) == 'IsBatteryLow'
