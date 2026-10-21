"""CheckStopCondition fires a front_clearance stop only on the locked wall.

check_stop_condition.py, FRONT-WALL LOCK (rules in mission/wall_lock.py). The
messages go through the real callbacks: /perception/virtual_wall (Float32),
its /debug payload (the FRONT track id), /perception/walls and odometry.

The car drives +x from the origin, bumper 0.4 m ahead of base_link. The
target wall is x = 4 (track 7); the side wall is y = 1.2 (track 9).

Run standalone: python3 -m pytest test/test_wall_lock_behaviour.py -v
"""

import json
import math
from types import SimpleNamespace

import pytest

py_trees = pytest.importorskip('py_trees')

import f1tenth_behavior.behaviours.check_stop_condition as check_module  # noqa: E402
from f1tenth_behavior.behaviours.check_stop_condition import (  # noqa: E402
    CheckStopCondition, front_from_debug, walls_from_message,
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
)
from f1tenth_messages.msg import MultiWallTrack, MultiWallTrackArray  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import Float32, Int32, String  # noqa: E402

S = py_trees.common.Status
STOP_AT = 1.0
FRONT_X = 0.4
TARGET, SIDE, TWIN = 7, 9, 12


@pytest.fixture(autouse=True)
def _no_report_files(monkeypatch):
    monkeypatch.setattr(check_module, 'write_mission_summary', lambda *a, **k: None)


class _Clock:
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
        self.infos, self.warns = [], []

    def info(self, msg, **k):
        self.infos.append(msg)

    def warn(self, msg, **k):
        self.warns.append((msg, k))

    def error(self, *a, **k):
        pass


def _wall_move(move_id='m'):
    return {'id': move_id, 'drive': {'mode': 'straight', 'speed': 0.4},
            'stop_condition': {'type': 'front_clearance', 'distance': STOP_AT},
            'timeout_sec': 60, 'on_timeout': 'abort'}


def _mission(moves):
    moves = [dict(m) for m in moves]
    moves[-1]['terminal'] = True
    config = parse_mission({'mission_id': 't', 'schema_version': '4.0', 'moves': moves})
    state = MissionRuntimeState()
    state.load(config, check_module.time.monotonic())
    state.begin(check_module.time.monotonic())
    return state


def _check(state, **kwargs):
    behaviour = CheckStopCondition(base_front_x_m=FRONT_X, **kwargs)
    logger = _Logger()
    behaviour.node = SimpleNamespace(get_logger=lambda: logger)
    behaviour.log = logger
    behaviour.blackboard = SimpleNamespace()
    setattr(behaviour.blackboard, MISSION_KEY, state)
    for key in (MIN_OBSTACLE_DISTANCE_KEY, MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
                FRONT_CLEARANCE_KEY):
        setattr(behaviour.blackboard, key, None)
    setattr(behaviour.blackboard, DETECTED_CLASSES_KEY, {})
    behaviour.hold_pub, behaviour.object_end_pub = _Pub(), _Pub()
    return behaviour


def _track(track_id, rho, theta, state=MultiWallTrack.STATE_CONFIRMED, role='front',
           source=MultiWallTrack.SOURCE_LIDAR):
    t = MultiWallTrack()
    t.id, t.state, t.role, t.label = track_id, state, role, role
    t.rho, t.theta, t.source = float(rho), float(theta), source
    return t


def _target(track_id=TARGET, rho=4.0, **kw):
    return _track(track_id, rho, 0.0, **kw)


def _side(**kw):
    return _track(SIDE, 1.2, math.pi / 2, role='left', **kw)


def _odom(b, x, y=0.0):
    msg = Odometry()
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    msg.pose.pose.orientation.w = 1.0
    b._odom_cb(msg)


def _scan(b, distance, front_id, tracks, state='wall'):
    """One tracker step as lidar_front_wall_node publishes it: walls, then the
    virtual wall and its debug payload from the same publish."""
    msg = MultiWallTrackArray()
    msg.tracks = list(tracks)
    b._walls_cb(msg)
    b._front_clearance_cb(Float32(data=float(distance)))
    b._wall_debug_cb(String(data=json.dumps(
        {'state': state, 'distance_m': distance, 'front_id': front_id})))


def _approach(b, clock, x=0.0):
    """Lock the target wall from 3.6 m out."""
    _odom(b, x)
    _scan(b, 3.6 - x, TARGET, [_target(), _side()])
    clock.t += 0.1
    assert b.update() == S.RUNNING


def test_a_normal_approach_stops_on_the_locked_wall(clock):
    state = _mission([_wall_move()])
    b = _check(state)
    _approach(b, clock)
    assert any('locked on track 7' in m for m in b.log.infos)
    _odom(b, 2.7)
    _scan(b, 0.9, TARGET, [_target(), _side()])
    clock.t += 0.1
    assert b.update() == S.SUCCESS
    assert state.last_stop_reason == 'stop_condition:front_clearance'
    assert b.hold_pub.msgs == []


def test_a_side_wall_in_the_strip_does_not_stop_the_move(clock):
    """THE R029 CASE. The swerve puts the side wall in the travel strip at
    0.8 m, under the 1.0 m stop; the raw wall topic carries 0.8."""
    b = _check(_mission([_wall_move()]))
    _approach(b, clock)
    _odom(b, 1.2, 0.3)
    _scan(b, 0.8, SIDE, [_target(), _side()])
    clock.t += 0.1
    assert b.update() == S.RUNNING
    assert b.hold_pub.msgs == []        # not a hold: the car is not stopped by this
    assert any('locked wall 7 is not in the way' in m for m in b.log.infos)


def test_without_the_lock_the_side_wall_stops_it(clock):
    """The same ticks with wall_lock_enabled false: the old behaviour."""
    b = _check(_mission([_wall_move()]), wall_lock_enabled=False)
    _approach(b, clock)
    _odom(b, 1.2, 0.3)
    _scan(b, 0.8, SIDE, [_target(), _side()])
    clock.t += 0.1
    assert b.update() == S.SUCCESS


def test_back_on_the_locked_wall_it_stops(clock):
    b = _check(_mission([_wall_move()]))
    _approach(b, clock)
    _scan(b, 0.8, SIDE, [_target(), _side()])
    clock.t += 0.1
    assert b.update() == S.RUNNING
    _odom(b, 2.7)
    _scan(b, 0.9, TARGET, [_target(), _side()])
    clock.t += 0.1
    assert b.update() == S.SUCCESS
    assert any('front wall again' in m for m in b.log.infos)


def test_the_arrival_arm_does_not_fire_on_another_wall(clock):
    """At rest within the ramp's tolerance of the target -- but next to the
    side wall. The rest arm reads the same locked value, so it waits too."""
    b = _check(_mission([_wall_move()]))
    _approach(b, clock)
    _scan(b, STOP_AT + 0.05, SIDE, [_target(), _side()])
    b.speed = 0.0
    clock.t += 0.1
    assert b.update() == S.RUNNING


def test_a_stereo_only_front_does_not_capture_the_lock(clock):
    """bf7938 (2026-10-21 12:25): a short-lived stereo-only FRONT must not
    lock out the LiDAR wall that comes later."""
    state = _mission([_wall_move()])
    b = _check(state)
    _odom(b, 0.0)
    phantom = _track(SIDE, 5.35, math.radians(-23.4), source=MultiWallTrack.SOURCE_STEREO)
    _scan(b, 4.6, SIDE, [phantom])
    clock.t += 0.1
    assert b.update() == S.RUNNING
    _scan(b, 50.0, None, [], state='clear')
    clock.t += 0.1
    assert b.update() == S.RUNNING
    _odom(b, 3.1)
    _scan(b, 0.9, TARGET, [_target(source=MultiWallTrack.SOURCE_BOTH)])
    clock.t += 0.1
    assert b.update() == S.SUCCESS
    assert any('locked on track 7' in m for m in b.log.infos)


def test_a_reborn_wall_on_the_same_line_is_reacquired(clock):
    b = _check(_mission([_wall_move()]))
    _approach(b, clock)
    _odom(b, 2.7)
    _scan(b, 0.9, TWIN, [_target(TWIN, rho=4.05), _side()])   # track 7 gone
    clock.t += 0.1
    assert b.update() == S.SUCCESS
    assert any('re-acquired: track 7 -> 12' in m for m in b.log.infos)


def test_a_nearer_surface_is_refused_with_a_throttled_warn(clock):
    b = _check(_mission([_wall_move()]))
    _approach(b, clock)
    _odom(b, 2.0)
    _scan(b, 0.9, TWIN, [_target(TWIN, rho=3.3), _side()])    # 0.7 m nearer
    clock.t += 0.1
    assert b.update() == S.RUNNING
    msg, kwargs = b.log.warns[-1]
    assert 'not adopted' in msg and kwargs.get('throttle_duration_sec') == 2.0


def test_a_new_move_locks_its_own_wall(clock):
    state = _mission([_wall_move('a'), _wall_move('b')])
    b = _check(state)
    _approach(b, clock)
    state.goto_move(1, clock.t)
    _scan(b, 0.8, SIDE, [_target(), _side()])
    clock.t += 0.1
    assert b.update() == S.SUCCESS      # move b's first FRONT is the side wall


def test_an_unreadable_debug_payload_reads_not_yet(clock):
    b = _check(_mission([_wall_move()]))
    _approach(b, clock)
    b._front_clearance_cb(Float32(data=0.5))
    b._wall_debug_cb(String(data='{not json'))
    clock.t += 0.1
    assert b.update() == S.RUNNING


def test_without_debug_or_walls_topics_it_reads_the_wall_topic_as_before(clock):
    """A perception stack that does not publish them: never locks, so the
    stop is the pre-lock one."""
    b = _check(_mission([_wall_move()]))
    b._front_clearance_cb(Float32(data=0.9))
    clock.t += 0.1
    assert b.update() == S.SUCCESS


def test_a_stale_debug_payload_is_not_used(clock):
    b = _check(_mission([_wall_move()]))
    _approach(b, clock)
    _scan(b, 0.9, TARGET, [_target(), _side()])
    clock.t += 0.6                      # > wall_distance_max_age_s for all three...
    b._front_clearance_cb(Float32(data=0.9))   # ...but the Float32 is fresh again
    assert b.update() == S.RUNNING


# -- /mpc/stop_wall_id: the wall mpc_corr's deceleration ramp aims at ---------------

def test_the_locked_wall_goes_out_for_the_ramp_on_every_tick(clock):
    """Locked, the swerve (still the locked wall), then lost: -1.

    On -1 mpc_corr ramps on the raw wall topic again.
    """
    b = _check(_mission([_wall_move()]))
    b.stop_wall_id_pub = _Pub()
    _approach(b, clock)
    _odom(b, 1.2, 0.3)
    _scan(b, 0.8, SIDE, [_target(), _side()])
    clock.t += 0.1
    assert b.update() == S.RUNNING
    _scan(b, 0.8, SIDE, [_side()])      # track 7 gone; the side wall is refused
    clock.t += 0.1
    assert b.update() == S.RUNNING
    assert b.stop_wall_id_pub.msgs == [TARGET, TARGET, -1]


def test_a_tick_that_holds_the_car_sends_no_lock_id(clock):
    """mpc_corr reads a stale id as no lock: the raw wall topic."""
    b = _check(_mission([_wall_move()]))
    b.stop_wall_id_pub = _Pub()
    _approach(b, clock)
    clock.t += 0.6                      # every wall input stale: WALL DISTANCE FRESHNESS
    assert b.update() == S.RUNNING
    assert b.hold_pub.msgs == [True]
    assert b.stop_wall_id_pub.msgs == [TARGET]


class _SetupNode:
    def __init__(self):
        self.publishers = []

    def create_subscription(self, *args, **kwargs):
        return None

    def create_publisher(self, msg_type, topic, qos):
        self.publishers.append((msg_type, topic))
        return _Pub()


@pytest.mark.parametrize('enabled', [True, False])
def test_the_ramp_topic_exists_only_with_the_lock(enabled):
    b = CheckStopCondition(base_front_x_m=FRONT_X, wall_lock_enabled=enabled)
    node = _SetupNode()
    b.setup(node=node)
    assert ((Int32, '/mpc/stop_wall_id') in node.publishers) == enabled
    assert (b.stop_wall_id_pub is not None) == enabled


def test_the_odometer_is_path_length():
    b = _check(_mission([_wall_move()]))
    for x, y in ((0.0, 0.0), (0.3, 0.0), (0.3, 0.4), (0.0, 0.4)):
        _odom(b, x, y)
    assert b._odometer == pytest.approx(1.0)


def test_walls_from_message_reads_ids_states_roles_and_lines():
    msg = MultiWallTrackArray()
    msg.tracks = [_target(), _side(state=MultiWallTrack.STATE_COASTING,
                                   source=MultiWallTrack.SOURCE_STEREO)]
    walls = walls_from_message(msg)
    assert set(walls) == {TARGET, SIDE}
    assert (walls[SIDE].state, walls[SIDE].role, walls[SIDE].source) == (
        MultiWallTrack.STATE_COASTING, 'left', MultiWallTrack.SOURCE_STEREO)
    assert walls[TARGET].rho == pytest.approx(4.0) and walls[SIDE].theta == pytest.approx(
        math.pi / 2)


@pytest.mark.parametrize('text, expected', [
    ('{"state": "wall", "distance_m": 1.5, "front_id": 3}', ('wall', 1.5, 3)),
    ('{"state": "clear", "distance_m": 50.0, "front_id": null}', ('clear', 50.0, None)),
    ('{"state": "no_data", "distance_m": -1.0}', ('no_data', -1.0, None)),
])
def test_front_from_debug_reads_state_distance_and_front_id(text, expected):
    reading = front_from_debug(text)
    assert (reading.state, reading.distance, reading.front_id) == expected


@pytest.mark.parametrize('text', ['', '{not json', '[1, 2]', '{"distance_m": 1.0}'])
def test_front_from_debug_rejects_what_it_cannot_read(text):
    assert front_from_debug(text) is None
