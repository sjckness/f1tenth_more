"""The front_clearance wall stop holds the car on a stale or absent wall distance.

check_stop_condition.py, WALL DISTANCE FRESHNESS: while the current move's
stop_condition is front_clearance, a wall distance that was never received,
is older than wall_distance_max_age_s, or is not positive (-1.0, "no wall")
engages /mpc/hold and the stop is not evaluated; the first fresh positive
value releases the hold and the stop is evaluated on that same tick.

Run standalone: python3 -m pytest test/test_wall_distance_freshness.py -v
"""

import time
from types import SimpleNamespace

import pytest

py_trees = pytest.importorskip('py_trees')

import f1tenth_behavior.behaviours.check_stop_condition as check_module  # noqa: E402
from f1tenth_behavior.behaviours.check_stop_condition import (  # noqa: E402
    CheckStopCondition,
)
from f1tenth_behavior.mission.detected_classes_bridge import (  # noqa: E402
    DETECTED_CLASSES_KEY,
)
from f1tenth_behavior.mission.mission_config import parse_mission  # noqa: E402
from f1tenth_behavior.mission.runtime import (  # noqa: E402
    FRONT_CLEARANCE_KEY,
    MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
    MIN_OBSTACLE_DISTANCE_KEY,
    MISSION_KEY,
    MissionRuntimeState,
    MissionState,
)
from std_msgs.msg import Float32  # noqa: E402

S = py_trees.common.Status
MAX_AGE = 0.5
STOP_AT = 1.0


@pytest.fixture(autouse=True)
def _no_report_files(monkeypatch):
    monkeypatch.setattr(check_module, 'write_mission_summary', lambda *a, **k: None)


class _Clock:
    """time.monotonic() for check_stop_condition, set by the test."""

    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock(monkeypatch):
    c = _Clock()
    monkeypatch.setattr(check_module.time, 'monotonic', c)
    return c


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg.data)


class _Logger:
    def __init__(self):
        self.warns = []

    def info(self, *a, **k):
        pass

    def warn(self, msg, **k):
        self.warns.append((msg, k))

    def error(self, *a, **k):
        pass


def _mission(moves):
    moves = [dict(m) for m in moves]
    moves[-1]['terminal'] = True
    config = parse_mission({'mission_id': 't', 'schema_version': '4.0', 'moves': moves})
    state = MissionRuntimeState()
    state.load(config, check_module.time.monotonic())
    state.begin(check_module.time.monotonic())
    return state


def _wall_move(move_id='m', **extra):
    return dict({'id': move_id, 'drive': {'mode': 'straight', 'speed': 0.4},
                 'stop_condition': {'type': 'front_clearance', 'distance': STOP_AT},
                 'timeout_sec': 60, 'on_timeout': 'abort'}, **extra)


def _check(state):
    behaviour = CheckStopCondition(wall_distance_max_age_s=MAX_AGE)
    logger = _Logger()
    behaviour.node = SimpleNamespace(get_logger=lambda: logger)
    behaviour.logger_stub = logger
    behaviour.blackboard = SimpleNamespace()
    setattr(behaviour.blackboard, MISSION_KEY, state)
    for key in (MIN_OBSTACLE_DISTANCE_KEY, MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
                FRONT_CLEARANCE_KEY):
        setattr(behaviour.blackboard, key, None)
    setattr(behaviour.blackboard, DETECTED_CLASSES_KEY, {})
    behaviour.hold_pub, behaviour.object_end_pub = _Pub(), _Pub()
    return behaviour


def _wall(behaviour, value):
    """A wall-topic message arriving now, through the real callback."""
    behaviour._front_clearance_cb(Float32(data=float(value)))


def test_a_fresh_distance_above_the_threshold_keeps_driving(clock):
    behaviour = _check(_mission([_wall_move()]))
    _wall(behaviour, 2.5)
    clock.t += 0.1
    assert behaviour.update() == S.RUNNING
    assert behaviour.hold_pub.msgs == []


def test_a_fresh_distance_below_the_threshold_stops(clock):
    state = _mission([_wall_move()])
    behaviour = _check(state)
    _wall(behaviour, 0.8)
    clock.t += 0.1
    assert behaviour.update() == S.SUCCESS
    assert state.last_stop_reason == 'stop_condition:front_clearance'
    assert behaviour.hold_pub.msgs == []


def test_a_stale_distance_holds_and_does_not_stop_on_the_old_value(clock):
    behaviour = _check(_mission([_wall_move()]))
    _wall(behaviour, 0.8)                   # below the threshold, but...
    clock.t += MAX_AGE + 0.01               # ...no newer message since
    assert behaviour.update() == S.RUNNING
    assert behaviour.hold_pub.msgs == [True]
    msg, kwargs = behaviour.logger_stub.warns[-1]
    assert 'stale' in msg and kwargs.get('throttle_duration_sec') == 2.0


def test_minus_one_holds_and_is_never_read_as_a_distance(clock):
    behaviour = _check(_mission([_wall_move()]))
    _wall(behaviour, -1.0)                  # -1.0 < 1.0, which must not be a stop
    clock.t += 0.1
    assert behaviour.update() == S.RUNNING
    assert behaviour.hold_pub.msgs == [True]
    assert 'no wall' in behaviour.logger_stub.warns[-1][0]


def test_no_message_at_all_holds(clock):
    behaviour = _check(_mission([_wall_move()]))
    assert behaviour.update() == S.RUNNING
    assert behaviour.hold_pub.msgs == [True]


def test_the_hold_is_repeated_every_tick_while_stale(clock):
    behaviour = _check(_mission([_wall_move()]))
    _wall(behaviour, 2.0)
    clock.t += 1.0
    for _ in range(3):
        assert behaviour.update() == S.RUNNING
    assert behaviour.hold_pub.msgs == [True, True, True]


def test_recovery_after_stale_releases_once_and_drives_on(clock):
    behaviour = _check(_mission([_wall_move()]))
    _wall(behaviour, 2.0)
    clock.t += 1.0
    assert behaviour.update() == S.RUNNING          # stale: held
    _wall(behaviour, 1.8)                           # fresh again, above threshold
    clock.t += 0.05
    assert behaviour.update() == S.RUNNING
    assert behaviour.update() == S.RUNNING
    assert behaviour.hold_pub.msgs == [True, False]


def test_recovery_straight_into_the_threshold_releases_and_stops(clock):
    state = _mission([_wall_move()])
    behaviour = _check(state)
    _wall(behaviour, -1.0)
    clock.t += 0.05
    assert behaviour.update() == S.RUNNING          # no wall: held
    _wall(behaviour, 0.9)
    clock.t += 0.05
    assert behaviour.update() == S.SUCCESS
    assert behaviour.hold_pub.msgs == [True, False]


def test_the_move_timeout_still_runs_while_held(clock):
    state = _mission([_wall_move(timeout_sec=5)])
    behaviour = _check(state)
    assert behaviour.update() == S.RUNNING
    clock.t += 6.0
    assert behaviour.update() == S.FAILURE
    assert state.state == MissionState.ABORTED
    assert behaviour.hold_pub.msgs == [True, True]


def test_a_move_without_a_wall_stop_is_not_held(clock):
    behaviour = _check(_mission([{
        'id': 'm', 'goal_distance': 3.0, 'stop_condition': {'type': 'distance_reached'},
        'timeout_sec': 60}]))
    assert behaviour.update() == S.RUNNING
    assert behaviour.hold_pub.msgs == []


def test_leaving_a_held_wall_move_releases_for_the_next_move(clock):
    state = _mission([_wall_move('a', timeout_sec=5, on_timeout='skip'),
                      {'id': 'b', 'goal_distance': 3.0,
                       'stop_condition': {'type': 'distance_reached'}, 'timeout_sec': 60}])
    behaviour = _check(state)
    assert behaviour.update() == S.RUNNING           # held, no wall yet
    clock.t += 6.0
    assert behaviour.update() == S.SUCCESS           # timeout skip
    state.goto_move(1, clock.t)
    assert behaviour.update() == S.RUNNING           # move b: no wall stop
    assert behaviour.hold_pub.msgs == [True, False]


def test_a_holding_mission_is_left_to_its_own_hold(clock):
    state = _mission([_wall_move()])
    behaviour = _check(state)
    state.state = MissionState.HOLDING
    assert behaviour.update() == S.RUNNING
    assert behaviour.hold_pub.msgs == []
