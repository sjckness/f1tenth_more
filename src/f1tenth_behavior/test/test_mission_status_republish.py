"""Regression tests for the MISSION-END EVENT GAP fix in mission/loader.py.

The bug (found while building f1tenth_diagnostics' mission_logger_node, which
depends on this event existing): /mission/status was only ever published from
this file's own four service/load paths, but THREE of the four ways a mission
actually reaches a terminal state happen inside a BT tick, against the shared
MissionRuntimeState object -- which holds no node and no publisher, so
complete()/abort() there simply assign self.state and nothing is ever emitted:

    advance_move.py          state.complete()   NORMAL SUCCESS
    handle_object_action.py  state.abort()      on_object abort
    check_stop_condition.py  state.abort()      stop-condition abort

Only an operator calling /mission/abort_mission republished. So /mission/status
stayed latched on RUNNING through a mission that had already finished or
aborted itself, contradicting MissionStatus.msg's own documented contract
("Republished whenever mission state changes"), and any lifecycle consumer
would start on RUNNING and never stop -- losing exactly the failure-run data
it exists to capture.

Fixed with a state-change WATCHER (a timer diffing live state against the last
published value) rather than three new publish calls in the behaviours,
because enumerating call sites is what let them drift apart in the first
place. These tests therefore assert on the watcher's observable behaviour, not
on any particular behaviour file: any code path that changes the state must
result in a republish, and an unchanged state must NOT.

Same never-spun fake-node convention as test_mission_loader_hold_release.py --
see that file's own docstring. state.complete()/state.abort() are called
directly on the blackboard's MissionRuntimeState, which is exactly what the
three BT behaviours above do.

Run standalone: python3 -m pytest test/test_mission_status_republish.py -v
"""

import py_trees
import pytest

from f1tenth_behavior.mission.loader import MissionLoader
from f1tenth_behavior.mission.runtime import (
    CURRENT_XY_KEY, MISSION_KEY, MissionRuntimeState, MissionState)


class _FakeLogger:
    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


class _FakePublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _FakeClock:
    class _Time:
        def to_msg(self):
            from builtin_interfaces.msg import Time
            return Time()

    def now(self):
        return self._Time()


class _FakeParam:
    def __init__(self, value):
        self.value = value


class _FakeNode:
    def __init__(self):
        self.publishers = {}
        self.timers = []

    def create_publisher(self, msg_type, topic, qos):
        pub = _FakePublisher()
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, *a, **k):
        return None

    def create_service(self, *a, **k):
        return None

    def create_timer(self, period_sec, callback):
        self.timers.append((period_sec, callback))
        return None

    def declare_parameter(self, name, default):
        return _FakeParam(default)

    def get_logger(self):
        return _FakeLogger()

    def get_clock(self):
        return _FakeClock()

    def get_node_names(self):
        return []


@pytest.fixture
def loader_and_node():
    node = _FakeNode()
    loader = MissionLoader(node)
    bb = py_trees.blackboard.Client(name='FakeCheckStopCondition')
    bb.register_key(key=CURRENT_XY_KEY, access=py_trees.common.Access.WRITE)
    setattr(bb, CURRENT_XY_KEY, (0.0, 0.0))
    return loader, node


def _status_pub(node):
    return node.publishers['/mission/status']


def _runtime(loader) -> MissionRuntimeState:
    return getattr(loader.blackboard, MISSION_KEY)


class TestStateChangeWatcher:

    def test_watcher_timer_is_registered(self, loader_and_node):
        """The fix is a timer on the node -- if it is not registered, nothing
        below can ever fire in production no matter what the logic does."""
        _loader, node = loader_and_node
        assert node.timers, 'MissionLoader registered no state-watch timer'
        period, callback = node.timers[0]
        assert period > 0.0
        assert callable(callback)

    def test_bt_internal_complete_is_republished(self, loader_and_node):
        """advance_move.py's state.complete() -- the NORMAL SUCCESS path, and
        the one whose absence silently lost every completed run."""
        loader, node = loader_and_node
        pub = _status_pub(node)
        before = len(pub.published)

        _runtime(loader).complete()          # exactly what AdvanceMove does
        loader._on_state_watch_tick()

        assert len(pub.published) == before + 1, 'no republish on BT-internal complete()'
        assert pub.published[-1].state == MissionState.COMPLETE.value

    def test_bt_internal_abort_is_republished(self, loader_and_node):
        """handle_object_action.py / check_stop_condition.py's state.abort() --
        the autonomous abort paths, i.e. the failure runs that matter most."""
        loader, node = loader_and_node
        pub = _status_pub(node)
        before = len(pub.published)

        _runtime(loader).abort()
        loader._on_state_watch_tick()

        assert len(pub.published) == before + 1, 'no republish on BT-internal abort()'
        assert pub.published[-1].state == MissionState.ABORTED.value

    def test_unchanged_state_does_not_republish(self, loader_and_node):
        """MissionStatus.msg's other documented promise: NOT published on every
        BT tick. A steady state must emit nothing, however often the watcher
        runs -- otherwise this fix would spam a transient-local topic at timer
        rate."""
        loader, node = loader_and_node
        pub = _status_pub(node)
        before = len(pub.published)

        for _ in range(25):
            loader._on_state_watch_tick()

        assert len(pub.published) == before

    def test_only_one_republish_per_transition(self, loader_and_node):
        """Having emitted the transition once, subsequent ticks stay quiet
        until the state changes again."""
        loader, node = loader_and_node
        pub = _status_pub(node)

        _runtime(loader).complete()
        loader._on_state_watch_tick()
        after_first = len(pub.published)
        for _ in range(10):
            loader._on_state_watch_tick()

        assert len(pub.published) == after_first

    def test_service_path_publish_also_updates_the_watcher_baseline(self, loader_and_node):
        """The service paths still publish directly (unchanged behaviour); the
        watcher must record that, or it would immediately re-emit the same
        state a second time on its next tick."""
        loader, node = loader_and_node
        pub = _status_pub(node)

        loader._publish_status()
        after_direct = len(pub.published)
        loader._on_state_watch_tick()

        assert len(pub.published) == after_direct
