"""Part D: the BT's side of mpc_corr's reverse manoeuvre (reverse_integration.py).

Mocked: IsProximityTooClose with a fake node and clock, the mode monitor fed
ReverseStatus-shaped messages by hand, ReverseIntegration's blackboard and
publishers replaced. The front threshold is TEST-CONFIGURED at 0.40 m: the
shipped 0.15 m is under the 0.20 m floor, so with it every front trip is a
full stop and the direction-aware gate never shows.

The sixth case of the work order, a stale /bt/allow_reverse -> no reverse, is
mpc_corr's side: test_reverse_manoeuvre.py (bt_allow_stale).

Run standalone: python3 -m pytest test/test_bt_reverse_integration.py -v
"""

import math
from pathlib import Path
from types import SimpleNamespace

from f1tenth_behavior import reverse_integration
from f1tenth_behavior.behaviours.is_battery_low import IsBatteryLow
from f1tenth_behavior.behaviours.is_proximity_too_close import IsProximityTooClose
from f1tenth_behavior.mission.mission_config import load_mission_file
from f1tenth_behavior.mission.runtime import (
    GLOBAL_TURN_ACCUM_KEY, GLOBAL_XY_KEY, GLOBAL_YAW_KEY, MISSION_KEY,
    MissionRuntimeState, MissionState)
from f1tenth_behavior.reverse_integration import (
    REVERSE_MODE_KEY, ReverseIntegration, ReverseModeMonitor)
import py_trees

S = py_trees.common.Status
MISSIONS = Path(__file__).resolve().parent.parent / 'missions'
BUMPER = 0.405


class _Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, *_a, **_k):
        self.lines.append(str(msg))

    warn = warning = error = debug = info


class _Node:
    def __init__(self):
        self.logger = _Logger()
        self.publishers = {}

    def get_logger(self):
        return self.logger

    def create_publisher(self, _type, topic, _qos):
        return self.publishers.setdefault(topic, _Pub())

    def create_subscription(self, *_a, **_k):
        return None

    def get_clock(self):
        from builtin_interfaces.msg import Time
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))


def _status(mode, reverse_active=False, attempts=0):
    return SimpleNamespace(mode=mode, reverse_active=reverse_active, attempts=attempts,
                           last_termination_reason='')


def _scan(front_from_bumper=None, side=None):
    """Build a full-circle scan with the laser at base_link.

    One front return straight ahead at the given distance from the bumper,
    one side return at 90 deg.
    """
    n = 360
    ranges = [float('inf')] * n
    inc = 2.0 * math.pi / n
    if front_from_bumper is not None:
        ranges[n // 2] = BUMPER + front_from_bumper           # angle 0
    if side is not None:
        ranges[n // 2 + n // 4] = side                         # angle +90 deg
    return SimpleNamespace(header=SimpleNamespace(frame_id='laser'), ranges=ranges,
                           angle_min=-math.pi, angle_max=math.pi - inc,
                           angle_increment=inc, range_min=0.02)


def _rig(mission='drive_test_straight_2m'):
    clock = _Clock()
    monitor = ReverseModeMonitor(max_age_s=0.5, clock=clock)
    prox = IsProximityTooClose(
        front_distance_threshold=0.40, lidar_distance_threshold=0.15,
        base_front_x_m=BUMPER, immediate_stop_topic='safety_stop', clock=clock,
        reverse_mode=monitor, front_hard_floor_m=0.20)
    node = _Node()
    prox.setup(node=node)
    battery = IsBatteryLow()
    root = py_trees.composites.Selector(name='emergency_condition', memory=False)
    root.add_children([battery, prox])

    integ = ReverseIntegration(monitor, root, clock=clock)
    integ.node = node
    integ.allow_pub, integ.hold_pub, integ.end_pub = _Pub(), _Pub(), _Pub()
    state = MissionRuntimeState()
    bb = SimpleNamespace()
    setattr(bb, MISSION_KEY, state)
    for key in (GLOBAL_XY_KEY, GLOBAL_YAW_KEY, GLOBAL_TURN_ACCUM_KEY):
        setattr(bb, key, None)
    integ.blackboard = bb
    state.load(load_mission_file(str(MISSIONS / f'{mission}.json')), clock.now)
    state.begin(clock.now)
    return SimpleNamespace(clock=clock, monitor=monitor, prox=prox, node=node,
                           root=root, integ=integ, state=state,
                           safety=node.publishers['safety_stop'])


def _mpc(rig, mode, reverse_active=False):
    rig.monitor.on_status(_status(mode, reverse_active))


def _tick(rig, mode=None, reverse_active=False):
    """One BT tick: the MPC's status (if any), the tree, the post-tick handler."""
    rig.clock.now += 0.1
    if mode is not None:
        _mpc(rig, mode, reverse_active)
    rig.root.tick_once()
    rig.integ.post_tick()
    return rig.integ.allow_pub.msgs[-1].data


def test_stuck_at_the_front_stop_reverses_without_a_safety_stop_then_it_resumes():
    rig = _rig()
    rig.prox._scan_callback(_scan(front_from_bumper=0.35))
    # Forward: the front stop holds the car, and the reverse is allowed.
    assert _tick(rig, 'forward') is True
    assert rig.prox.status == S.SUCCESS
    stops_before = len(rig.safety.msgs)
    assert stops_before == 1

    # Stopping for the reverse, then reversing: no zero on the safety lane,
    # from the scan callback or the tick, while the wall stays at 0.35 m.
    for mode in ('stopping', 'reverse', 'reverse'):
        _mpc(rig, mode, reverse_active=True)
        rig.prox._scan_callback(_scan(front_from_bumper=0.35))
        assert _tick(rig) is True
        assert rig.prox.status == S.FAILURE
        assert rig.root.status == S.FAILURE
    assert len(rig.safety.msgs) == stops_before

    # Back to forward (stopping toward forward is not reverse_active): the
    # front stop is in force again.
    _mpc(rig, 'stopping', reverse_active=False)
    rig.prox._scan_callback(_scan(front_from_bumper=0.35))
    _tick(rig)
    assert rig.prox.status == S.SUCCESS
    assert len(rig.safety.msgs) == stops_before + 1


def test_front_under_the_floor_stops_in_every_direction_and_forbids_reversing():
    rig = _rig()
    _mpc(rig, 'reverse', reverse_active=True)
    rig.prox._scan_callback(_scan(front_from_bumper=0.15))
    assert _tick(rig) is False
    assert rig.prox.status == S.SUCCESS
    assert len(rig.safety.msgs) == 1


def test_an_emergency_condition_withdraws_the_permission():
    rig = _rig()
    battery = rig.root.children[0]
    battery.update = lambda: S.SUCCESS
    assert _tick(rig, 'forward') is False
    assert rig.integ.allow_reason == battery.name


def test_a_side_obstacle_during_the_reverse_stops_and_withdraws():
    rig = _rig()
    _mpc(rig, 'reverse', reverse_active=True)
    rig.prox._scan_callback(_scan(side=0.10))
    assert _tick(rig) is False
    assert rig.prox.status == S.SUCCESS
    assert len(rig.safety.msgs) == 1


def test_no_permission_without_a_running_mission():
    rig = _rig()
    rig.state.abort()
    assert _tick(rig, 'forward') is False


def test_a_stale_status_reads_as_forward():
    rig = _rig()
    _tick(rig, 'reverse', reverse_active=True)
    rig.clock.now += 1.0
    rig.prox._scan_callback(_scan(front_from_bumper=0.35))
    assert rig.monitor.mode == reverse_integration.MODE_UNKNOWN
    assert rig.prox.should_stop()


def test_three_failed_attempts_fail_the_move_stuck_after_reverse(monkeypatch):
    summaries = []
    monkeypatch.setattr(reverse_integration, 'write_mission_summary',
                        lambda *a, **_k: summaries.append(a[2]))
    rig = _rig()
    move_id = rig.state.current_move.id
    _tick(rig, 'forward')
    assert _tick(rig, 'stuck') is False

    assert rig.state.state is MissionState.ABORTED
    assert rig.state.last_stop_reason == 'failed:stuck_after_reverse'
    assert rig.state.move_outcomes[-1].move_id == move_id
    assert rig.state.move_outcomes[-1].stop_reason == 'failed:stuck_after_reverse'
    assert [m.data for m in rig.integ.hold_pub.msgs] == [True]
    assert summaries == ['ABORTED']
    assert getattr(rig.integ.blackboard, REVERSE_MODE_KEY) == 'stuck'
    assert any('forward -> stuck' in line for line in rig.node.logger.lines)


def test_a_stuck_left_over_from_before_the_move_does_not_fail_it(monkeypatch):
    monkeypatch.setattr(reverse_integration, 'write_mission_summary', lambda *a, **k: None)
    rig = _rig()
    rig.monitor.on_status(_status('stuck'))     # before this run's move started
    rig.clock.now += 0.05
    rig.state.begin(rig.clock.now)
    _tick(rig, 'stuck')
    assert rig.state.state is MissionState.RUNNING
    assert rig.integ.hold_pub.msgs == []
