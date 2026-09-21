"""The start countdown in mission/loader.py, and above all its stop paths.

/mission/start_mission no longer starts the mission: it arms a
``mission_countdown_sec`` timer (default 3 s) so every logged test begins from
a measured standstill, and the mission begins in ``_on_countdown_elapsed``.
That creates a window in which the operator has asked for a start and the car
has not moved yet -- and the only thing that matters about that window is that
a stop arriving inside it always wins.

The three tests this file exists for:

* :func:`test_emergency_stop_during_countdown_never_starts`
* :func:`test_abort_during_countdown_never_starts`
* :func:`test_timer_firing_after_a_stop_does_not_begin`

The last one is the nasty case: rclpy can already have the timer callback
queued in the executor when it is cancelled, so cancelling the timer is not by
itself enough. ``_start_cancelled`` is what closes that window, and this test
fires the callback by hand *after* a stop to prove it.

MissionLoader is driven through its own bound service handlers with a fake
node, the same approach (and for the same reason) as
src/f1tenth_behavior/test/test_mission_loader_hold_release.py: the handlers
are the thing under test, not ROS's service dispatch.
"""

from __future__ import annotations

import json
import types
from pathlib import Path

import pytest

py_trees = pytest.importorskip("py_trees")
loader_module = pytest.importorskip("f1tenth_behavior.mission.loader")

from f1tenth_behavior.mission.loader import MissionLoader  # noqa: E402
from f1tenth_behavior.mission.runtime import (  # noqa: E402
    CURRENT_XY_KEY,
    MISSION_KEY,
    MissionState,
)
MISSION_JSON = str(
    Path(__file__).resolve().parents[1] / "missions" / "dock_approach_01.json"
)
COUNTDOWN_S = 3.0


# --------------------------------------------------------------------------
# the fake node
# --------------------------------------------------------------------------

class _Logger:
    def __init__(self):
        self.lines = []

    def _record(self, *args, **kwargs):
        self.lines.append(" ".join(str(a) for a in args))

    info = warn = error = _record


class _Publisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _Clock:
    class _Time:
        def to_msg(self):
            from builtin_interfaces.msg import Time
            return Time()

    def now(self):
        return self._Time()


class _Param:
    def __init__(self, value):
        self.value = value


class _Timer:
    def __init__(self, period_sec, callback):
        self.period_sec = period_sec
        self.callback = callback
        self.cancelled = False
        self.destroyed = False

    def cancel(self):
        self.cancelled = True


class _Node:
    """Enough of rclpy.node.Node for MissionLoader and its service handlers."""

    def __init__(self, live_node_names, countdown_s=COUNTDOWN_S):
        self._live_node_names = live_node_names
        self._countdown_s = countdown_s
        self.publishers = {}
        self.timers = []
        self.logger = _Logger()

    def create_publisher(self, msg_type, topic, qos):
        pub = _Publisher()
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, *a, **k):
        return None

    def create_service(self, *a, **k):
        return None

    def create_timer(self, period_sec, callback):
        timer = _Timer(period_sec, callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        timer.destroyed = True
        self.timers = [t for t in self.timers if t is not timer]

    def declare_parameter(self, name, default):
        if name == "mission_countdown_sec":
            return _Param(self._countdown_s)
        return _Param(default)

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return _Clock()

    def get_node_names(self):
        return self._live_node_names


def _request():
    return types.SimpleNamespace()


def _response():
    return types.SimpleNamespace(success=None, message=None)


def _make_loader(countdown_s=COUNTDOWN_S):
    node = _Node(live_node_names=["mpc_corr", "ackermann_to_vesc_node"],
                 countdown_s=countdown_s)
    loader = MissionLoader(node)
    # Preflight reads this off the blackboard (CheckStopCondition writes it in
    # production); without it every start would fail preflight instead of
    # reaching the countdown under test.
    bb = py_trees.blackboard.Client(name="FakeCheckStopCondition")
    bb.register_key(key=CURRENT_XY_KEY, access=py_trees.common.Access.WRITE)
    setattr(bb, CURRENT_XY_KEY, (0.0, 0.0))
    ok, message = loader._load(MISSION_JSON)
    assert ok, message
    return loader, node


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def _state(loader):
    return getattr(loader.blackboard, MISSION_KEY).state


def _countdown_timers(loader, node):
    return [t for t in node.timers
            if t.callback == loader._on_countdown_elapsed and not t.cancelled]


def _fire_countdown(loader, node, force=False):
    """Run the countdown callback. ``force`` fires it even after a cancel.

    Forcing is not cheating: rclpy hands a timer callback to the executor
    before the cancel is processed, so this is exactly the real race.
    """
    if force:
        loader._on_countdown_elapsed()
        return
    pending = _countdown_timers(loader, node)
    assert pending, "no start countdown was armed"
    for timer in pending:
        timer.callback()


def _events(node):
    return [json.loads(m.data) for m in node.publishers["/test/mission_event"].published]


def _event_names(node):
    return [e["event"] for e in _events(node)]


def _holds(node):
    return [m.data for m in node.publishers["/mpc/hold"].published]


def _count_begins(loader):
    """Wrap MissionRuntimeState.begin so a call can be proven, not inferred."""
    state = getattr(loader.blackboard, MISSION_KEY)
    calls = []
    original = state.begin

    def counting_begin(*args, **kwargs):
        calls.append((args, kwargs))
        return original(*args, **kwargs)

    state.begin = counting_begin
    return calls


# --------------------------------------------------------------------------
# the three mandatory tests
# --------------------------------------------------------------------------

def test_emergency_stop_during_countdown_never_starts():
    """E-stop mid-countdown: no start, ever, and the test is an abort."""
    loader, node = _make_loader()
    begins = _count_begins(loader)

    start = loader._on_start_mission_service(_request(), _response())
    assert start.success is True
    assert _state(loader) is MissionState.LOADED, "the car must not be RUNNING yet"

    stop = loader._on_emergency_stop_service(_request(), _response())
    assert stop.success is True

    # Whatever the executor does with the already-armed timer, this must hold.
    _fire_countdown(loader, node, force=True)

    assert begins == [], "begin() was called after an emergency stop"
    assert _state(loader) is MissionState.ABORTED
    assert "mission_started" not in _event_names(node)
    assert False not in _holds(node), "/mpc/hold was released after an e-stop"
    aborted = [e for e in _events(node) if e["event"] == "mission_aborted"]
    assert len(aborted) == 1
    assert aborted[0]["reason"] == "cancelled before start"


def test_abort_during_countdown_never_starts():
    """/mission/abort_mission mid-countdown: same outcome, same reason."""
    loader, node = _make_loader()
    begins = _count_begins(loader)

    start = loader._on_start_mission_service(_request(), _response())
    assert start.success is True

    abort = loader._on_abort_mission_service(_request(), _response())
    assert abort.success is True

    _fire_countdown(loader, node, force=True)

    assert begins == [], "begin() was called after an abort"
    assert _state(loader) is MissionState.ABORTED
    assert "mission_started" not in _event_names(node)
    assert False not in _holds(node)
    aborted = [e for e in _events(node) if e["event"] == "mission_aborted"]
    assert len(aborted) == 1
    assert aborted[0]["reason"] == "cancelled before start"


def test_timer_firing_after_a_stop_does_not_begin():
    """The race itself: the callback was already queued when the stop landed.

    The timer is cancelled and destroyed by the stop path, so this fires the
    callback directly -- which is what rclpy would do with a callback it had
    already dequeued. ``_start_cancelled`` is the only thing standing between
    that and a car that drives off after the operator pressed stop.
    """
    loader, node = _make_loader()
    begins = _count_begins(loader)

    loader._on_start_mission_service(_request(), _response())
    armed = _countdown_timers(loader, node)
    assert len(armed) == 1

    loader._on_abort_mission_service(_request(), _response())
    assert armed[0].cancelled is True
    assert armed[0].destroyed is True
    assert loader._start_cancelled is True
    assert loader._countdown_timer is None

    loader._on_countdown_elapsed()          # the queued callback, fired late

    assert begins == [], "begin() was called by a timer that had been stopped"
    assert _state(loader) is MissionState.ABORTED
    assert "mission_started" not in _event_names(node)
    assert False not in _holds(node)


# --------------------------------------------------------------------------
# the rest of the countdown contract
# --------------------------------------------------------------------------

def test_start_arms_a_countdown_and_says_so():
    loader, node = _make_loader()
    resp = loader._on_start_mission_service(_request(), _response())
    assert resp.success is True
    assert resp.message == f"starting in {COUNTDOWN_S:.1f} s"
    assert len(_countdown_timers(loader, node)) == 1
    assert _countdown_timers(loader, node)[0].period_sec == COUNTDOWN_S
    # Nothing that could move the car has happened yet.
    assert _state(loader) is MissionState.LOADED
    assert _holds(node) == []


def test_countdown_elapsing_starts_the_mission():
    loader, node = _make_loader()
    begins = _count_begins(loader)
    loader._on_start_mission_service(_request(), _response())
    _fire_countdown(loader, node)

    assert len(begins) == 1
    assert _state(loader) is MissionState.RUNNING
    assert _holds(node)[-1] is False, "/mpc/hold must be released on the real start"
    assert _event_names(node) == ["mission_loaded", "mission_started"]


def test_a_second_start_during_the_countdown_is_refused():
    loader, node = _make_loader()
    loader._on_start_mission_service(_request(), _response())
    second = loader._on_start_mission_service(_request(), _response())
    assert second.success is False
    assert "already starting" in second.message
    assert len(_countdown_timers(loader, node)) == 1


def test_start_is_refused_while_the_emergency_stop_is_latched():
    loader, node = _make_loader()
    loader._on_emergency_stop_service(_request(), _response())
    resp = loader._on_start_mission_service(_request(), _response())
    assert resp.success is False
    assert "emergency stop" in resp.message
    assert _countdown_timers(loader, node) == []


def test_emergency_stop_answers_while_counting_down():
    """(e) The e-stop must never be waiting behind a sleeping callback."""
    loader, node = _make_loader()
    loader._on_start_mission_service(_request(), _response())
    resp = loader._on_emergency_stop_service(_request(), _response())
    assert resp.success is True
    assert resp.message == "emergency stop engaged"


def test_loading_another_mission_cancels_a_pending_start():
    loader, node = _make_loader()
    begins = _count_begins(loader)
    loader._on_start_mission_service(_request(), _response())
    ok, message = loader._load(MISSION_JSON)
    assert ok, message
    assert loader._countdown_timer is None
    loader._on_countdown_elapsed()
    assert begins == [], "the superseded countdown started the new mission"


def test_the_countdown_is_read_live_so_it_can_be_retuned():
    """`ros2 param set` between tests must take effect without a restart."""
    loader, node = _make_loader()
    node.get_parameter = lambda name: _Param(7.5)
    resp = loader._on_start_mission_service(_request(), _response())
    assert resp.message == "starting in 7.5 s"
    assert _countdown_timers(loader, node)[0].period_sec == 7.5
    assert _events(node)[0]["countdown_s"] == COUNTDOWN_S, (
        "the mission_loaded event was published before the change")


def test_a_node_without_get_parameter_still_works():
    """The unit-test stubs in src/f1tenth_behavior/test are exactly this."""
    loader, node = _make_loader()
    assert not hasattr(node, "get_parameter")
    assert loader._countdown_seconds() == COUNTDOWN_S


def test_zero_countdown_starts_immediately():
    """mission_countdown_sec=0 is the behaviour from before there was one."""
    loader, node = _make_loader(countdown_s=0.0)
    begins = _count_begins(loader)
    resp = loader._on_start_mission_service(_request(), _response())
    assert resp.success is True
    assert resp.message.startswith("started")
    assert len(begins) == 1
    assert _state(loader) is MissionState.RUNNING
    assert _holds(node)[-1] is False
    assert _countdown_timers(loader, node) == []


# --------------------------------------------------------------------------
# the events the campaign logger consumes
# --------------------------------------------------------------------------

def test_events_carry_the_plan_id_and_the_countdown():
    loader, node = _make_loader()
    state = getattr(loader.blackboard, MISSION_KEY)
    mission_id = state.config.mission_id

    loaded = _events(node)[0]
    assert loaded["event"] == "mission_loaded"
    assert loaded["plan_id"] == mission_id, "plan_id must be the mission_id"
    assert loaded["countdown_s"] == COUNTDOWN_S

    loader._on_start_mission_service(_request(), _response())
    _fire_countdown(loader, node)
    started = _events(node)[-1]
    assert started["event"] == "mission_started"
    assert started["plan_id"] == mission_id
    # repeated on the start, for a logger that missed mission_loaded
    assert started["countdown_s"] == COUNTDOWN_S


def test_one_event_per_edge_not_per_publish():
    """_publish_status runs on every service call; events are edges only."""
    loader, node = _make_loader()
    loader._publish_status()
    loader._publish_status()
    assert _event_names(node) == ["mission_loaded"]


def test_the_loader_publishes_exactly_the_events_the_logger_consumes():
    """The contract between the two files, asserted rather than assumed.

    Renaming an event on one side without the other would leave the campaign
    logger silently waiting for a mission that already finished.
    """
    logger_node = pytest.importorskip("f1tenth_logger.test_campaign.logger_node")
    assert set(loader_module._TEST_EVENTS.values()) == set(logger_node.MISSION_EVENTS)


def test_a_finished_mission_is_announced_by_the_state_watcher():
    """The BT completes a mission without touching any service path."""
    loader, node = _make_loader()
    loader._on_start_mission_service(_request(), _response())
    _fire_countdown(loader, node)
    state = getattr(loader.blackboard, MISSION_KEY)
    state.complete()                      # what AdvanceMove does, in the tree
    loader._on_state_watch_tick()
    assert _event_names(node)[-1] == "mission_finished"
