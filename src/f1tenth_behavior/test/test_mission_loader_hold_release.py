"""Regression test for a real bug found via live testing: after
/mission/abort_mission, a subsequent /mission/load_mission +
/mission/start_mission on a different mission reported success and the
mission genuinely reached RUNNING, but the car never moved -- mpc_corr's
own self.hold flag (MPC_corr.py) was still latched True from the abort and
nothing on the load/start path ever released it (see
_on_start_mission_service's own extended comment in mission/loader.py for
the full trace).

MissionLoader needs a real rclpy `node` for construction (create_publisher/
create_service/create_subscription/declare_parameter/get_logger/get_clock/
get_node_names) -- this test builds a minimal fake rather than standing up
a full rclpy context, same "keep it testable without the ROS runtime"
spirit as this package's other pure-logic tests, extended just far enough
to cover this one class's actual bug. _load()/_on_start_mission_service()/
_on_abort_mission_service() are called directly (bypassing ROS's own
service dispatch) -- legitimate here since those bound methods ARE the
thing under test, not the service wiring around them.

Run standalone: python3 -m pytest test/test_mission_loader_hold_release.py -v
"""

import types

import py_trees
import pytest
from std_msgs.msg import Bool

from f1tenth_behavior.mission.loader import MissionLoader
from f1tenth_behavior.mission.runtime import CURRENT_XY_KEY, MISSION_KEY

MISSION_A = 'missions/dock_approach_01.json'
MISSION_B = 'missions/turn_90_left.json'


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
    """Just enough of rclpy.node.Node for MissionLoader.__init__ and the
    three service handlers under test -- see module docstring."""

    def __init__(self, live_node_names):
        self._live_node_names = live_node_names
        self.publishers = {}

    def create_publisher(self, msg_type, topic, qos):
        pub = _FakePublisher()
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, *a, **k):
        return None

    def create_service(self, *a, **k):
        return None

    def create_timer(self, period_sec, callback):
        # MissionLoader's own /mission/status state-change watcher (see that
        # file's MISSION-END EVENT GAP FIX comment). Stored, never fired here
        # -- this test covers the hold-release path; the watcher itself is
        # covered by test_mission_status_republish.py.
        self.timers = getattr(self, 'timers', [])
        self.timers.append((period_sec, callback))
        return None

    def declare_parameter(self, name, default):
        return _FakeParam(default)  # mission_file_name unset -- no auto-load

    def get_logger(self):
        return _FakeLogger()

    def get_clock(self):
        return _FakeClock()

    def get_node_names(self):
        return self._live_node_names


def _make_loader():
    node = _FakeNode(live_node_names=['mpc_corr', 'ackermann_to_vesc_node'])
    loader = MissionLoader(node)
    # Preflight's localization check reads CURRENT_XY_KEY off the blackboard
    # (normally written by CheckStopCondition) -- set it directly via a
    # WRITE-registered client, same as that behaviour would in production,
    # so this test exercises the hold-release bug specifically, not preflight
    # (already covered by test_preflight.py).
    bb = py_trees.blackboard.Client(name='FakeCheckStopCondition')
    bb.register_key(key=CURRENT_XY_KEY, access=py_trees.common.Access.WRITE)
    setattr(bb, CURRENT_XY_KEY, (0.0, 0.0))
    return loader, node


def _trigger_request():
    return types.SimpleNamespace()


def _trigger_response():
    return types.SimpleNamespace(success=None, message=None)


class TestHoldReleasedOnStart:

    def test_start_mission_releases_hold(self):
        """The core fix: a plain load+start (no prior abort) still
        publishes hold(False) -- idempotent/harmless, and confirms the new
        publish happens on the success path at all."""
        loader, node = _make_loader()
        loader._load(MISSION_A)
        resp = loader._on_start_mission_service(_trigger_request(), _trigger_response())
        assert resp.success is True
        hold_pub = node.publishers['/mpc/hold']
        assert any(m.data is False for m in hold_pub.published)

    def test_reproduces_the_bug_then_confirms_the_fix(self):
        """Mission A -> abort (hold latched True, matching mpc_corr's own
        control_loop() gate) -> load mission B -> start mission B. Before
        this fix, nothing published hold(False) anywhere on this path --
        mpc_corr would have stayed held despite RUNNING. After the fix,
        start_mission's own publish releases it."""
        loader, node = _make_loader()
        hold_pub = node.publishers['/mpc/hold']

        loader._load(MISSION_A)
        start_resp = loader._on_start_mission_service(_trigger_request(), _trigger_response())
        assert start_resp.success is True

        abort_resp = loader._on_abort_mission_service(_trigger_request(), _trigger_response())
        assert abort_resp.success is True
        # abort_mission's own immediate hold(True) -- must still happen,
        # unchanged (see this fix's own "don't weaken abort's safety intent"
        # reasoning).
        assert hold_pub.published[-1].data is True

        # Reproduce: load + start a DIFFERENT mission. Both must succeed
        # (state.state correctly resets via load()/begin() -- never the
        # actual blocker, see the trace in loader.py's own comment) --
        # this is the mismatch: success without motion, before the fix.
        load_ok, _ = loader._load(MISSION_B)
        assert load_ok is True
        start_resp_2 = loader._on_start_mission_service(_trigger_request(), _trigger_response())
        assert start_resp_2.success is True

        # THE fix: the hold latched by the abort above must have been
        # released by this second, successful start_mission -- the last
        # message on /mpc/hold must be False, not still the True from abort.
        assert hold_pub.published[-1].data is False

    def test_hold_not_released_when_start_rejected_state_not_loaded(self):
        """No mission loaded at all -- start_mission must reject (state is
        IDLE, not LOADED) and must NOT touch /mpc/hold at all: releasing it
        on a rejected start would be a safety regression in its own right."""
        loader, node = _make_loader()
        resp = loader._on_start_mission_service(_trigger_request(), _trigger_response())
        assert resp.success is False
        hold_pub = node.publishers['/mpc/hold']
        assert hold_pub.published == []

    def test_hold_not_released_when_preflight_fails(self):
        """A dependency (ackermann_to_vesc_node) missing from the ROS graph
        -- start_mission must reject via the preflight check (3.1) and must
        NOT release the hold: a rejected start is not "now actively
        driving", so nothing should un-hold the car."""
        node = _FakeNode(live_node_names=['mpc_corr'])  # ackermann_to_vesc_node missing
        loader = MissionLoader(node)
        bb = py_trees.blackboard.Client(name='FakeCheckStopCondition2')
        bb.register_key(key=CURRENT_XY_KEY, access=py_trees.common.Access.WRITE)
        setattr(bb, CURRENT_XY_KEY, (0.0, 0.0))

        loader._load(MISSION_A)
        resp = loader._on_start_mission_service(_trigger_request(), _trigger_response())
        assert resp.success is False
        assert 'ackermann_to_vesc_node' in resp.message
        hold_pub = node.publishers['/mpc/hold']
        assert hold_pub.published == []


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
