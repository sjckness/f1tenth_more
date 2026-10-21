"""The deceleration ramp aims at the mission's locked wall (ramp_wall_source.py).

The four cases the ramp source must get right, through MPC_corr's own
methods (_ramp_wall_distance, _wall_stop_d_rem, _wall_stop_tick,
_apply_stop_ramp) on a duck-typed node:

  wall_lock_enabled false   the raw wall topic, exactly as before;
  not locked yet            the raw wall topic;
  locked                    the locked wall's value from the debug payload
                            while it is FRONT, no wall ramp while another
                            wall is (the P002-R029 swerve);
  locked wall lost          the raw wall topic, until re-acquired.

Then the same four cases end to end with f1tenth_behavior's real lock
(mission/wall_lock.py): its phases, mapped to /mpc/stop_wall_id by
ramp_wall_id, read back here.

The mission stops 1.0 m from the target wall, track 7; the side wall is
track 9, 0.6 m away in the strip during the swerve; track 12 is the target
wall re-born on the same line.

Run standalone: python3 -m pytest test/test_ramp_wall_source.py -v
"""

import json
import math
import os
from types import SimpleNamespace

from f1tenth_params.param_defaults import get_value
from mpc_controller.MPC_corr import MPCController
from mpc_controller.ramp_wall_source import (
    front_from_debug, FrontReading, NO_LOCK, RampWallSource, select_ramp_distance,
    SOURCE_LOCKED, SOURCE_NOT_FRONT, SOURCE_RAW, SOURCE_RAW_NO_DEBUG, SOURCE_RAW_NO_LOCK)
from mpc_controller.stop_profile import RAMP_UNKNOWN, StopRampLatch, StopRampProfile
import pytest

TARGET_M = 1.0
TARGET, SIDE, TWIN = 7, 9, 12
SIDE_M = 0.6        # under the target: the raw topic would stop the car here
CLEAR_M = float(get_value('wall_clear_distance_m'))
MAX_AGE = float(get_value('wall_distance_max_age_s'))


class _Clock:
    def __init__(self, node):
        self._node = node

    def now(self):
        return SimpleNamespace(nanoseconds=int(round(self._node.t * 1e9)))


class _Logger:
    def __init__(self):
        self.infos, self.warns = [], []

    def info(self, msg, **k):
        self.infos.append(msg)

    def warn(self, msg, **k):
        self.warns.append(msg)


class _Node:
    """The ramp's slice of MPCController: its real methods, on plain state."""

    _fresh_front_distance = MPCController._fresh_front_distance
    _ramp_wall_distance = MPCController._ramp_wall_distance
    _wall_stop_d_rem = MPCController._wall_stop_d_rem
    _wall_stop_tick = MPCController._wall_stop_tick
    _apply_stop_ramp = MPCController._apply_stop_ramp
    front_distance_callback = MPCController.front_distance_callback
    stop_target_callback = MPCController.stop_target_callback
    stop_wall_id_callback = MPCController.stop_wall_id_callback
    virtual_wall_debug_callback = MPCController.virtual_wall_debug_callback

    def __init__(self, lock_enabled=True):
        self.t = 100.0
        self.front_distance = 10.0
        self.front_distance_stamp_sec = None
        self.corr_wall_turn_front_distance_max_age_sec = 0.5
        self.wall_clear_distance_m = CLEAR_M
        self.stop_ramp = StopRampProfile(
            a_dec=float(get_value('stop_ramp_a_dec_mps2')),
            a_stop=float(get_value('stop_ramp_a_stop_mps2')),
            tau_latency_s=float(get_value('stop_ramp_tau_latency_s')),
            v_min=float(get_value('min_moving_speed_mps')))
        self.stop_ramp_latch = StopRampLatch(self.stop_ramp)
        self.stop_target_m = None
        self.stop_ramp_mode = self.stop_ramp_d_rem = None
        self.v = self.vdes = 0.5
        self._last_published_steer = 0.1
        self.drive = []
        self.log = _Logger()
        self.ramp_wall_pick = None
        self.ramp_wall_source = (RampWallSource(MAX_AGE, CLEAR_M)
                                 if lock_enabled else None)
        self.stop_target_callback(SimpleNamespace(data=TARGET_M))

    def get_clock(self):
        return _Clock(self)

    def get_logger(self):
        return self.log

    def _publish_drive(self, speed, steer):
        self.drive.append((speed, steer))

    # -- what arrives on the wire --------------------------------------------

    def scan(self, distance, front_id, state='wall'):
        """One lidar_front_wall_node publish: the Float32 and its debug payload."""
        self.front_distance_callback(SimpleNamespace(data=float(distance)))
        if self.ramp_wall_source is not None:      # subscribed only with the lock
            self.virtual_wall_debug_callback(SimpleNamespace(data=json.dumps(
                {'state': state, 'distance_m': distance, 'front_id': front_id})))

    def lock(self, track_id):
        """One BT tick's /mpc/stop_wall_id."""
        self.stop_wall_id_callback(SimpleNamespace(data=int(track_id)))


def _d_rem(node, d):
    return node.stop_ramp.predict(d, node.v) - TARGET_M


# -- wall_lock_enabled false: exactly the old behaviour --------------------------

@pytest.mark.parametrize('value, age', [
    (2.5, 0.1), (0.6, 0.1), (2.5, 0.9), (-1.0, 0.1), (0.0, 0.1),
    (float('nan'), 0.1), (CLEAR_M, 0.1), (CLEAR_M - 0.1, 0.1)])
def test_without_the_lock_the_ramp_distance_is_the_fresh_front_distance(value, age):
    node = _Node(lock_enabled=False)
    node.front_distance, node.front_distance_stamp_sec = value, node.t - age
    raw = node._fresh_front_distance()
    assert node._ramp_wall_distance() == raw
    expected = None if raw is None else _d_rem(node, raw)
    assert node._wall_stop_d_rem() == expected
    assert node.ramp_wall_pick is None
    assert not any(m.startswith('RAMP |') for m in node.log.infos)


def test_without_the_lock_a_side_wall_in_the_strip_stops_the_car():
    """The old behaviour, kept: the ramp latches on whatever wall is FRONT."""
    node = _Node(lock_enabled=False)
    node.scan(SIDE_M, SIDE)
    assert node._wall_stop_tick() is True
    assert node.drive == [(0.0, 0.1)]


# -- not locked yet ---------------------------------------------------------------

def test_before_any_lock_id_the_ramp_reads_the_raw_wall_topic():
    node = _Node()
    node.scan(2.4, TARGET)
    assert node._ramp_wall_distance() == 2.4
    assert node.ramp_wall_pick.source == SOURCE_RAW_NO_LOCK


def test_not_locked_yet_the_ramp_reads_the_raw_wall_topic():
    node = _Node()
    node.lock(NO_LOCK)
    node.scan(2.4, SIDE)                # e.g. a stereo-only FRONT the BT did not lock
    assert node._ramp_wall_distance() == 2.4
    assert node.ramp_wall_pick.source == SOURCE_RAW


# -- locked -----------------------------------------------------------------------

def test_locked_the_ramp_reads_the_locked_walls_value_from_the_debug_payload():
    node = _Node()
    node.lock(TARGET)
    node.scan(1.8, TARGET)
    node.front_distance = 1.9           # not the Float32: the payload's own value
    assert node._ramp_wall_distance() == 1.8
    assert node._wall_stop_d_rem() == pytest.approx(_d_rem(node, 1.8))
    assert node.ramp_wall_pick.source == SOURCE_LOCKED
    assert any('locked wall 7 at 1.80 m' in m for m in node.log.infos)


def test_locked_with_a_side_wall_in_the_strip_there_is_no_wall_ramp():
    """THE R029 CASE: the swerve puts the side wall in the strip under the target.

    The raw topic would latch the stop; the locked ramp does not brake for
    it, and does not WARN about it either.
    """
    node = _Node()
    node.lock(TARGET)
    node.scan(SIDE_M, SIDE)
    assert node._ramp_wall_distance() is None
    assert node.ramp_wall_pick.source == SOURCE_NOT_FRONT
    assert node._wall_stop_tick() is False
    assert node.drive == [] and not node.stop_ramp_latch.latched
    assert node._apply_stop_ramp(0.5) == 0.5
    assert node.stop_ramp_mode == RAMP_UNKNOWN
    assert node.log.warns == []
    assert any('locked wall 7 is not in the way (track 9 is the front wall)' in m
               for m in node.log.infos)


def test_locked_and_back_in_front_the_ramp_reads_it_again():
    node = _Node()
    node.lock(TARGET)
    node.scan(SIDE_M, SIDE)
    assert node._wall_stop_tick() is False
    node.scan(1.05, TARGET)
    assert node._ramp_wall_distance() == 1.05
    assert node._wall_stop_tick() is True       # inside d_final of the target


@pytest.mark.parametrize('state, front_id', [('clear', None), ('no_data', None)])
def test_locked_with_no_wall_in_front_there_is_no_wall_ramp(state, front_id):
    node = _Node()
    node.lock(TARGET)
    node.scan(CLEAR_M if state == 'clear' else -1.0, front_id, state=state)
    assert node._ramp_wall_distance() is None
    assert node.ramp_wall_pick.source == SOURCE_NOT_FRONT


# -- locked wall lost -------------------------------------------------------------

def test_a_lost_lock_ramps_on_the_raw_wall_topic_and_can_only_brake_more():
    node = _Node()
    node.lock(TARGET)
    node.scan(SIDE_M, SIDE)
    assert node._wall_stop_tick() is False
    node.lock(NO_LOCK)                  # the BT: locked wall 7 lost, 9 refused
    assert node._ramp_wall_distance() == SIDE_M
    assert node.ramp_wall_pick.source == SOURCE_RAW
    assert node._wall_stop_tick() is True       # brakes for the side wall


def test_a_reacquired_lock_reads_the_new_track():
    node = _Node()
    node.lock(NO_LOCK)
    node.lock(TWIN)
    node.scan(1.9, TWIN)
    assert node._ramp_wall_distance() == 1.9
    assert node.ramp_wall_pick.source == SOURCE_LOCKED


# -- degraded inputs: the raw wall topic, as if unlocked --------------------------

def test_a_stale_lock_id_reads_the_raw_wall_topic():
    """A BT that stopped running the lock leaves the ramp on the raw value.

    Holding the car, dead, between moves: its last id must not keep the ramp
    off the side wall.
    """
    node = _Node()
    node.lock(TARGET)
    node.t += MAX_AGE + 0.05
    node.scan(SIDE_M, SIDE)
    assert node._ramp_wall_distance() == SIDE_M
    assert node.ramp_wall_pick.source == SOURCE_RAW_NO_LOCK


def test_a_stale_debug_payload_reads_the_raw_wall_topic():
    node = _Node()
    node.scan(1.8, TARGET)
    node.t += MAX_AGE + 0.05
    node.lock(TARGET)
    node.front_distance_callback(SimpleNamespace(data=1.7))   # the Float32 is fresh
    assert node._ramp_wall_distance() == 1.7
    assert node.ramp_wall_pick.source == SOURCE_RAW_NO_DEBUG


def test_an_unreadable_debug_payload_reads_the_raw_wall_topic():
    node = _Node()
    node.lock(TARGET)
    node.front_distance_callback(SimpleNamespace(data=1.7))
    node.virtual_wall_debug_callback(SimpleNamespace(data='{not json'))
    assert node._ramp_wall_distance() == 1.7
    assert node.ramp_wall_pick.source == SOURCE_RAW_NO_DEBUG


@pytest.mark.parametrize('bad', [-1.0, 0.0, CLEAR_M, float('nan'), None])
def test_a_locked_front_that_is_not_a_distance_never_comes_out(bad):
    pick = select_ramp_distance(1.7, TARGET, FrontReading('wall', bad, TARGET), CLEAR_M)
    assert (pick.distance, pick.source) == (1.7, SOURCE_RAW_NO_DEBUG)


def test_a_new_stop_target_forgets_the_last_moves_lock():
    node = _Node()
    node.lock(TARGET)
    node.stop_target_callback(SimpleNamespace(data=2.0))
    node.scan(SIDE_M, SIDE)
    assert node._ramp_wall_distance() == SIDE_M
    assert node.ramp_wall_pick.source == SOURCE_RAW_NO_LOCK


# -- the payload parser: the lock's contract ---------------------------------------

@pytest.mark.parametrize('text, expected', [
    ('{"state": "wall", "distance_m": 1.5, "front_id": 3}', ('wall', 1.5, 3)),
    ('{"state": "clear", "distance_m": 50.0, "front_id": null}', ('clear', 50.0, None)),
    ('{"state": "no_data", "distance_m": -1.0}', ('no_data', -1.0, None)),
])
def test_front_from_debug_reads_state_distance_and_front_id(text, expected):
    assert tuple(front_from_debug(text)) == expected


@pytest.mark.parametrize('text', [
    '', '{not json', '[1, 2]', '"wall"', '{"distance_m": 1.0}',
    '{"state": "wall", "distance_m": "far", "front_id": 3}'])
def test_front_from_debug_rejects_what_it_cannot_read(text):
    assert front_from_debug(text) is None


# -- end to end with f1tenth_behavior's lock ---------------------------------------

@pytest.fixture
def wall_lock():
    # Skipped in the test, not at module level: a module-level importorskip
    # would skip the whole directory under the launch_testing plugin.
    return pytest.importorskip('f1tenth_behavior.mission.wall_lock')


def test_no_lock_is_the_value_the_behaviour_publishes(wall_lock):
    assert NO_LOCK == wall_lock.NO_LOCK_ID


@pytest.mark.parametrize('lock_enabled', [True, False])
def test_the_four_cases_end_to_end_with_the_real_lock(wall_lock, lock_enabled):
    """The BT's FrontWallLock decides, ramp_wall_id sends it, the ramp reads it.

    Car on +x, bumper at x + 0.4; target wall x = 4.
    """
    wl = wall_lock

    def wall(track_id, rho, theta=0.0, role='front'):
        return wl.WallTrack(track_id=track_id, state=wl.STATE_CONFIRMED, role=role,
                            rho=rho, theta=theta)

    target, side, twin = wall(TARGET, 4.0), wall(SIDE, 1.2, math.pi / 2, 'left'), \
        wall(TWIN, 4.05)
    lock = wl.FrontWallLock()
    node = _Node(lock_enabled=lock_enabled)
    ticks = [
        # (front state, front id, distance, tracks, bumper x) -> the BT phase,
        # then what the ramp aims at with the lock and without it.
        ('clear', None, CLEAR_M, [side], 0.4, wl.UNLOCKED, None, None),
        ('wall', TARGET, 3.6, [target, side], 0.4, wl.LOCKED, 3.6, 3.6),
        ('wall', SIDE, SIDE_M, [target, side], 1.6, wl.NOT_FRONT, None, SIDE_M),
        ('wall', SIDE, SIDE_M, [side], 1.7, wl.LOST, SIDE_M, SIDE_M),
        ('wall', TWIN, 1.9, [twin, side], 2.1, wl.LOCKED, 1.9, 1.9),
    ]
    for state, front_id, d, tracks, x, phase, with_lock, without in ticks:
        node.t += 0.1
        node.scan(d, front_id, state=state)
        step = lock.update(node.t, d, wl.FrontReading(state, d, front_id),
                           {t.track_id: t for t in tracks}, (x, 0.0), x)
        assert step.phase == phase
        if lock_enabled:
            node.lock(wl.ramp_wall_id(step))
        assert node._ramp_wall_distance() == (with_lock if lock_enabled else without)


def test_select_ramp_distance_is_the_raw_value_whenever_no_lock_is_in_force():
    for locked_id in (None, NO_LOCK):
        for front in (None, FrontReading('wall', 0.5, SIDE), FrontReading('clear', 50.0, None)):
            pick = select_ramp_distance(0.8, locked_id, front, CLEAR_M)
            assert pick.distance == 0.8
            assert pick.source in (SOURCE_RAW, SOURCE_RAW_NO_LOCK)


# -- the real node: the parameter, the subscriptions, the callbacks ----------------

@pytest.fixture
def real_node(monkeypatch):
    """Build the real MPCController (stack_params defaults), torn down after the test."""
    rclpy = pytest.importorskip('rclpy')
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '147'))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    made = []

    def _make(lock_enabled):
        rclpy.init(args=['--ros-args', '--log-level', 'fatal',
                         '-p', f'wall_lock_enabled:={str(lock_enabled).lower()}'])
        node = MPCController()
        made.append(node)
        return node

    yield _make
    for node in made:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


@pytest.mark.parametrize('enabled', [True, False])
def test_the_node_reads_the_lock_only_with_wall_lock_enabled(real_node, enabled):
    from std_msgs.msg import Float32, Int32, String
    node = real_node(enabled)
    topics = {sub.topic_name for sub in node.subscriptions}
    assert ('/mpc/stop_wall_id' in topics) == enabled
    assert ('/perception/virtual_wall/debug' in topics) == enabled
    node.stop_target_callback(Float32(data=TARGET_M))
    node.front_distance_callback(Float32(data=SIDE_M))
    if enabled:
        node.stop_wall_id_callback(Int32(data=TARGET))
        node.virtual_wall_debug_callback(String(data=json.dumps(
            {'state': 'wall', 'distance_m': SIDE_M, 'front_id': SIDE})))
    # The swerve: no wall ramp with the lock, the side wall without it.
    assert node._ramp_wall_distance() == (None if enabled else pytest.approx(SIDE_M))
