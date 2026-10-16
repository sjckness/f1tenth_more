"""cpu-cleanup (2026-10-16): the BT tick timer, its timing report, the
event-driven lidar e-stop and the vectorised /scan windows.

Apart from TestDropQosEventHandlers (rclpy.init, real node) none of these
tests starts rclpy: the ticker and the behaviour are driven through a fake
node, and the clock is a mock, so the tick rate and the e-stop latency are
asserted from the code path itself. They import ROS message packages, so they
run on the Orin (Humble), not on a machine with another distro.
"""

import ast
import math
import pathlib
import random
import threading
import time
from unittest.mock import MagicMock

from diagnostic_msgs.msg import DiagnosticStatus
import py_trees
import pytest
from rclpy.clock import Clock
from sensor_msgs.msg import LaserScan

from f1tenth_behavior.behavior_executor_node import (
    _BehaviourSnapshot, _status_dot_graph, create_root, make_tree_visualizer)
from f1tenth_behavior.behaviours.is_proximity_too_close import (
    IsProximityTooClose, _in_front_cone, _in_lidar_window)
from f1tenth_behavior.tick_timing import BtTicker, TickTimingStats, percentile

PKG = pathlib.Path(__file__).resolve().parents[1] / 'f1tenth_behavior'

# Hokuyo UST-10LX as configured (sensors.yaml): 40 Hz, 1081 beams over 270 deg.
SCAN_PERIOD_S = 1.0 / 40.0
N_BEAMS = 1081


class _Recorder:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _FakeTimer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _FakeNode:
    """Enough of rclpy.node.Node for BtTicker and IsProximityTooClose."""

    def __init__(self):
        self.timers = []
        self.publishers = {}
        self.logger = MagicMock()
        self._clock = Clock()

    def create_timer(self, period, callback):
        timer = _FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def create_publisher(self, msg_type, topic, qos):
        return self.publishers.setdefault(topic, _Recorder())

    def create_subscription(self, *args, **kwargs):
        return MagicMock()

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return self._clock


class _FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class _FakeTree:
    """tree.tick() that takes `cost_s` of fake-clock time."""

    def __init__(self, clock, cost_s):
        self.clock = clock
        self.cost_s = cost_s
        self.ticks = 0

    def tick(self):
        self.ticks += 1
        self.clock.now += self.cost_s


# ---------------------------------------------------------------- tick timer

class TestTickRate:

    @pytest.mark.parametrize('hz', [5.0, 10.0, 20.0])
    def test_the_tick_timer_period_is_one_over_bt_tick_hz(self, hz):
        node = _FakeNode()
        ticker = BtTicker(node, _FakeTree(_FakeClock(), 0.0), hz)
        assert ticker.timer.period == pytest.approx(1.0 / hz)
        assert ticker.timer.callback == ticker.tick
        # The report timer runs at 1 Hz whatever the tick rate.
        assert ticker.report_timer.period == pytest.approx(1.0)

    def test_measured_rate_matches_bt_tick_hz_under_a_mock_clock(self):
        node, clock = _FakeNode(), _FakeClock()
        tree = _FakeTree(clock, cost_s=0.002)
        ticker = BtTicker(node, tree, 10.0, clock=clock)
        start = clock.now
        for i in range(11):           # the executor firing the timer on time
            clock.now = start + i * ticker.period_s
            ticker.tick()
        ticker.report()
        status = node.publishers['/diagnostics'].msgs[-1].status[0]
        values = {kv.key: kv.value for kv in status.values}
        assert tree.ticks == 11
        assert float(values['rate_hz_measured']) == pytest.approx(10.0, rel=1e-3)
        assert float(values['tick_ms_p50']) == pytest.approx(2.0, abs=1e-3)
        assert status.level == DiagnosticStatus.OK

    def test_rejects_a_non_positive_rate(self):
        with pytest.raises(ValueError):
            BtTicker(_FakeNode(), _FakeTree(_FakeClock(), 0.0), 0.0)

    def test_a_tick_over_half_the_period_warns_and_is_reported(self):
        node, clock = _FakeNode(), _FakeClock()
        ticker = BtTicker(node, _FakeTree(clock, cost_s=0.060), 10.0, clock=clock)
        ticker.tick()
        assert node.logger.warn.called
        ticker.report()
        status = node.publishers['/diagnostics'].msgs[-1].status[0]
        assert status.level == DiagnosticStatus.WARN
        assert {kv.key: kv.value for kv in status.values}['overruns'] == '1'

    def test_no_report_for_a_window_without_ticks(self):
        node = _FakeNode()
        BtTicker(node, _FakeTree(_FakeClock(), 0.0), 10.0).report()
        assert '/diagnostics' not in node.publishers or not node.publishers['/diagnostics'].msgs


class TestTickTimingStats:

    def test_nearest_rank_percentiles(self):
        values = sorted(float(v) for v in range(1, 101))
        assert percentile(values, 50) == 50.0
        assert percentile(values, 95) == 95.0
        assert percentile([7.0], 95) == 7.0

    def test_report_resets_the_window(self):
        stats = TickTimingStats(0.1)
        stats.record(0.0, 0.01)
        stats.record(0.1, 0.07)       # over the 50 ms budget
        report = stats.report_and_reset()
        assert report['ticks'] == 2 and report['overruns'] == 1
        assert report['max_ms'] == pytest.approx(70.0)
        assert stats.report_and_reset() is None


# ------------------------------------------------- no blocking in the tick

# Calls that block the single-threaded executor (or build ROS entities) and
# must never run inside a tick: update()/initialise()/terminate() of every
# behaviour, and the post-tick handlers in behavior_executor_node.
_BLOCKING = {
    'spin_until_future_complete', 'spin_once', 'spin', 'sleep',
    'wait_for_service', 'wait_for_server', 'create_subscription',
    'create_publisher', 'create_client', 'create_timer', 'create_png', 'call',
}
_TICK_METHODS = {'update', 'initialise', 'terminate'}


def _called_names(func):
    for node in ast.walk(func):
        if isinstance(node, ast.Call):
            f = node.func
            if isinstance(f, ast.Attribute):
                yield f.attr
            elif isinstance(f, ast.Name):
                yield f.id


class TestNoBlockingCallsInTheTick:

    @pytest.mark.parametrize(
        'path', sorted((PKG / 'behaviours').glob('*.py')), ids=lambda p: p.name)
    def test_behaviour_tick_methods_make_no_blocking_calls(self, path):
        tree = ast.parse(path.read_text())
        for cls in (n for n in tree.body if isinstance(n, ast.ClassDef)):
            for func in cls.body:
                if isinstance(func, ast.FunctionDef) and func.name in _TICK_METHODS:
                    bad = set(_called_names(func)) & _BLOCKING
                    assert not bad, f'{path.name}:{cls.name}.{func.name} calls {bad}'

    def test_post_tick_handlers_make_no_blocking_calls(self):
        tree = ast.parse((PKG / 'behavior_executor_node.py').read_text())
        handlers = {'_log_snapshot', '_visualize', '_publish'}
        found = set()
        for func in ast.walk(tree):
            if isinstance(func, ast.FunctionDef) and func.name in handlers:
                found.add(func.name)
                bad = set(_called_names(func)) & _BLOCKING
                assert not bad, f'{func.name} calls {bad}'
        assert found == handlers

    def test_the_visualizer_returns_while_a_render_is_still_running(self):
        release = threading.Event()
        rendered = threading.Event()

        def slow_render(snapshot):
            release.wait(5.0)
            rendered.set()
            return b'png'

        from f1tenth_behavior.behavior_executor_node import _PngRenderWorker
        node, pub = _FakeNode(), _Recorder()
        worker = _PngRenderWorker(node, pub, render=slow_render)
        handler = make_tree_visualizer(node, pub, min_interval_sec=0.0, worker=worker)
        tree = MagicMock()
        tree.root = py_trees.composites.Selector(name='root', memory=False)
        t0 = time.monotonic()
        handler(tree)
        handler(tree)                 # a second tick while the first renders
        elapsed = time.monotonic() - t0
        assert elapsed < 0.05, f'post-tick handler blocked for {elapsed:.3f} s'
        release.set()
        assert rendered.wait(5.0)
        deadline = time.monotonic() + 5.0
        while not pub.msgs and time.monotonic() < deadline:
            time.sleep(0.01)
        worker.stop()
        assert pub.msgs and pub.msgs[0].format == 'png'

    def test_the_snapshot_graph_matches_the_live_tree_graph(self):
        root = py_trees.composites.Selector(name='root', memory=False)
        seq = py_trees.composites.Sequence(name='seq', memory=False)
        seq.add_children([py_trees.behaviours.Success(name='ok')])
        root.add_children([seq, py_trees.behaviours.Failure(name='no')])
        root.tick_once()
        assert (_status_dot_graph(_BehaviourSnapshot(root)).to_string()
                == _status_dot_graph(root).to_string())


# ------------------------------------------------------ event-driven e-stop

def _scan(ranges, angle_min=-math.radians(135.0), range_min=0.02):
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.angle_min = angle_min
    msg.angle_increment = math.radians(270.0) / (len(ranges) - 1)
    msg.angle_max = angle_min + msg.angle_increment * (len(ranges) - 1)
    msg.range_min = range_min
    msg.range_max = 30.0
    msg.ranges = ranges
    return msg


def _proximity(event_stop=True, laser_x=None):
    behaviour = IsProximityTooClose(
        front_distance_threshold=0.15, lidar_distance_threshold=0.15,
        base_front_x_m=0.405,
        immediate_stop_topic='safety_stop' if event_stop else None)
    node = _FakeNode()
    behaviour.setup(node=node)
    return behaviour, node


def _front_return_at(distance_from_laser):
    ranges = [5.0] * N_BEAMS
    ranges[N_BEAMS // 2] = distance_from_laser   # angle 0: straight ahead
    return _scan(ranges)


class TestEventDrivenEstop:

    def test_a_tripping_scan_publishes_the_stop_inside_the_callback(self):
        behaviour, node = _proximity()
        # No /tf_static: the laser is taken at base_link, so a return 0.50 m
        # ahead is 0.095 m past the 0.405 m bumper -- inside 0.15 m.
        behaviour._scan_callback(_front_return_at(0.50))
        sent = node.publishers['safety_stop'].msgs
        assert len(sent) == 1
        assert sent[0].header.frame_id == 'base_link/emergency'
        assert sent[0].drive.speed == 0.0 and sent[0].drive.steering_angle == 0.0
        # The tree agrees on its next tick, as before.
        assert behaviour.update() == py_trees.common.Status.SUCCESS

    def test_a_clear_scan_publishes_nothing(self):
        behaviour, node = _proximity()
        behaviour._scan_callback(_front_return_at(2.0))
        assert not node.publishers['safety_stop'].msgs
        assert behaviour.update() == py_trees.common.Status.FAILURE

    def test_disabled_event_stop_creates_no_publisher(self):
        behaviour, node = _proximity(event_stop=False)
        behaviour._scan_callback(_front_return_at(0.50))
        assert 'safety_stop' not in node.publishers
        assert behaviour.update() == py_trees.common.Status.SUCCESS

    def test_estop_latency_is_one_scan_period_plus_one_callback(self):
        """The stated worst case (stack_params.yaml enable_proximity_event_stop):
        scan period + this callback. Asserted against a 5 ms callback budget on
        the full 1081-beam layout, which leaves the ~30 ms total with margin."""
        behaviour, node = _proximity()
        clear, close = _front_return_at(3.0), _front_return_at(0.50)
        for _ in range(20):
            behaviour._scan_callback(clear)
        worst = 0.0
        for _ in range(50):
            t0 = time.perf_counter()
            behaviour._scan_callback(close)
            worst = max(worst, time.perf_counter() - t0)
        assert len(node.publishers['safety_stop'].msgs) == 50
        assert worst < 0.005, f'scan callback took {1e3 * worst:.2f} ms'
        latency_s = SCAN_PERIOD_S + worst
        assert latency_s < 0.030
        # 0.4 and 1.0 m/s, in cm: what the car covers before the stop is sent.
        assert 0.4 * latency_s * 100 < 1.2 and 1.0 * latency_s * 100 < 3.0

    def test_create_root_wires_the_event_stop_onto_the_emergency_topic(self):
        root = create_root(enable_proximity_event_stop=True)
        prox = [b for b in root.iterate() if isinstance(b, IsProximityTooClose)]
        assert len(prox) == 1
        assert prox[0].immediate_stop_topic == 'safety_stop'
        assert prox[0].immediate_stop_frame_id == 'base_link/emergency'
        stops = [b for b in root.iterate() if b.name == 'Stop'
                 and getattr(b, 'frame_id', '') == 'base_link/emergency']
        assert stops and stops[0].output_topic == 'safety_stop'

    def test_create_root_default_keeps_the_tick_only_path(self):
        root = create_root()
        prox = [b for b in root.iterate() if isinstance(b, IsProximityTooClose)]
        assert prox[0].immediate_stop_topic is None


class TestEventStopScope:
    """Block 1b: the callback stop and the tree decide through one predicate,
    under the same enable conditions, and re-arm together."""

    def _prox(self, clock, active=True):
        behaviour = IsProximityTooClose(
            front_distance_threshold=0.15, lidar_distance_threshold=0.15,
            base_front_x_m=0.405, immediate_stop_topic='safety_stop', clock=clock)
        if not active:
            behaviour._active = lambda: False
        node = _FakeNode()
        behaviour.setup(node=node)
        return behaviour, node

    def test_active_case_both_paths_stop(self):
        clock = _FakeClock()
        behaviour, node = self._prox(clock)
        behaviour._scan_callback(_front_return_at(0.50))
        assert len(node.publishers['safety_stop'].msgs) == 1
        assert behaviour.update() == py_trees.common.Status.SUCCESS

    def test_inactive_case_neither_path_stops(self):
        # A future gate in _active() (the only place one may go) silences
        # the callback AND the tick on the very same tripping scan.
        clock = _FakeClock()
        behaviour, node = self._prox(clock, active=False)
        behaviour._scan_callback(_front_return_at(0.50))
        assert not node.publishers['safety_stop'].msgs
        assert behaviour.update() == py_trees.common.Status.FAILURE

    def test_disabled_lidar_safety_stop_builds_neither_path(self):
        root = create_root(enable_lidar_safety_stop=False, enable_proximity_event_stop=True)
        assert not [b for b in root.iterate() if isinstance(b, IsProximityTooClose)]

    def test_a_sub_tick_trip_is_held_by_the_tree_as_long_as_by_the_mux(self):
        clock = _FakeClock()
        behaviour, node = self._prox(clock)
        behaviour._scan_callback(_front_return_at(0.50))     # trips, publishes
        clock.now += 0.025
        behaviour._scan_callback(_front_return_at(3.0))      # clear scan
        clock.now += 0.050                                   # the next tick
        assert behaviour.update() == py_trees.common.Status.SUCCESS
        clock.now += 0.200                                   # mux lane expired
        assert behaviour.update() == py_trees.common.Status.FAILURE
        assert len(node.publishers['safety_stop'].msgs) == 1

    def test_the_tree_cannot_bypass_the_emergency_lane(self):
        """No decorator, guard or earlier lane can stop the tree ticking it
        while the callback keeps publishing: emergency is root child 0 of a
        memory=False Selector, IsProximityTooClose sits directly in it, and it
        reads no blackboard key."""
        root = create_root(enable_proximity_event_stop=True)
        assert isinstance(root, py_trees.composites.Selector) and not root.memory
        emergency = root.children[0]
        assert emergency.name == 'emergency'
        condition = emergency.children[0]
        assert isinstance(condition, py_trees.composites.Selector) and not condition.memory
        prox = [c for c in condition.children if isinstance(c, IsProximityTooClose)]
        assert len(prox) == 1 and prox[0].blackboards == []
        for node in (emergency, condition, prox[0]):
            assert not isinstance(node, py_trees.decorators.Decorator)


# ------------------------------------- vectorised windows == the old loop

def _reference(behaviour, msg, laser):
    """The pre-cpu-cleanup per-beam loop, verbatim in its arithmetic."""
    lx, ly, lyaw = laser
    min_in_window = min_front = None
    in_window = excluded = 0
    for i, r in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        valid = math.isfinite(r) and r >= msg.range_min
        if valid:
            dx = lx + r * math.cos(lyaw + angle) - behaviour.front_x
            dy = ly + r * math.sin(lyaw + angle)
            if dx > 0.0 and _in_front_cone(math.atan2(dy, dx),
                                           behaviour.front_cone_half_angle_rad):
                d = math.hypot(dx, dy)
                if min_front is None or d < min_front:
                    min_front = d
        if _in_lidar_window(angle, behaviour.lidar_blind_cone_half_angle_rad):
            in_window += 1
            if not valid:
                excluded += 1
            elif min_in_window is None or r < min_in_window:
                min_in_window = r
    return min_front, min_in_window


class TestVectorisedMatchesTheLoop:

    @pytest.mark.parametrize('seed', range(12))
    def test_random_scans(self, seed):
        rng = random.Random(seed)
        n = rng.choice([5, 64, N_BEAMS])
        ranges = []
        for _ in range(n):
            u = rng.random()
            ranges.append(float('nan') if u < 0.05 else float('inf') if u < 0.10
                          else rng.uniform(0.0, 0.03) if u < 0.15
                          else rng.uniform(0.05, 6.0))
        msg = _scan(ranges, angle_min=rng.uniform(-math.pi, -1.0))
        laser = rng.choice([(0.0, 0.0, 0.0), (0.305, 0.0, 0.0),
                            (-0.12, 0.02, math.pi), (0.1, -0.03, 0.2)])
        behaviour = IsProximityTooClose(
            base_front_x_m=0.405,
            front_cone_half_angle_deg=rng.choice([45.0, 60.0]),
            lidar_blind_cone_half_angle_deg=rng.choice([45.0, 30.0]))
        behaviour.node = MagicMock()
        behaviour._frames.pose = lambda frame_id: laser
        behaviour._scan_callback(msg)
        ref_front, ref_side = _reference(behaviour, msg, laser)
        for got, want in ((behaviour.latest_lidar_front_min, ref_front),
                          (behaviour.latest_lidar_min_in_window, ref_side)):
            if want is None:
                assert got is None
            else:
                assert got == pytest.approx(want, abs=1e-12)

    def test_layout_change_rebuilds_the_tables(self):
        behaviour = IsProximityTooClose(base_front_x_m=0.405)
        behaviour.node = MagicMock()
        behaviour._scan_callback(_scan([5.0] * 101))
        first = behaviour._layout_key
        behaviour._scan_callback(_scan([5.0] * 201))
        assert behaviour._layout_key != first
        assert behaviour.latest_lidar_front_min == pytest.approx(5.0 - 0.405)


# --------------------------------------------------- QoS event handlers

class TestDropQosEventHandlers:

    def test_drops_every_handler_and_the_node_still_destroys_cleanly(self):
        import rclpy
        from std_msgs.msg import Float32
        from f1tenth_behavior.qos_events import EventHandler, drop_qos_event_handlers
        context = rclpy.context.Context()
        rclpy.init(context=context)
        try:
            node = rclpy.create_node('qos_events_test', context=context)
            pubs = [node.create_publisher(Float32, f'/t{i}', 10) for i in range(3)]
            sub = node.create_subscription(Float32, '/t0', lambda m: None, 10)
            before = [w for w in node.waitables if isinstance(w, EventHandler)]
            assert before, 'rclpy no longer creates default handlers?'
            assert drop_qos_event_handlers(node) == len(before)
            assert not [w for w in node.waitables if isinstance(w, EventHandler)]
            assert all(not p.event_handlers for p in pubs) and not sub.event_handlers
            # Publishing and a runtime destroy still work.
            pubs[0].publish(Float32(data=1.0))
            assert node.destroy_publisher(pubs[1])
            node.destroy_node()
        finally:
            rclpy.shutdown(context=context)
