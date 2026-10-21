"""THE CONFIRMATION LEVEL (multi_wall_tracker, wall_confirm_* in stack_params.yaml).

The ONE rule that makes a wall track CONFIRMED, which every reader of
"confirmed" relies on without a count of its own:

  * promotion on exactly the wall_confirm_min_consecutive_hits-th cycle in a
    row with a LiDAR-backed hit, and not one cycle before;
  * never on stereo alone (wall_confirm_require_lidar);
  * never while the distance or the direction of those observations jitters
    beyond its bound, measured in odom, so the car's own motion is not jitter,
    and the distance at the wall, so where the car is in odom does not matter;
  * never on a segment shorter than wall_confirm_min_length_m;
  * no demotion: once confirmed, a missed or jittery cycle only makes the
    track COASTING;
  * the node builds the rule from the stack_params.yaml group.

Run standalone: python3 -m pytest test/test_wall_confirm_level.py -v
"""

import math
import os

import numpy as np
import pytest
import yaml

from corridor_perception.geometry import Line, Pose2D
from f1tenth_perception import multi_wall_tracker as mwt
from f1tenth_perception import wall_fusion
from f1tenth_perception.scene_labels import WallLine

ORIGIN = Pose2D(0.0, 0.0, 0.0)
FRONT_X = 0.405
N = mwt.TrackerParams().confirm_min_consecutive_hits

_SRC_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))


def wall(d, angle_deg=0.0, t0=-1.0, t1=1.0, inliers=120):
    """A base_link wall line at distance d, its normal angle_deg off the heading."""
    a = math.radians(angle_deg)
    return WallLine(math.cos(a), math.sin(a), d, t0, t1, inliers, 0.01, 'lidar')


def obs(line, source=wall_fusion.SOURCE_LIDAR, confidence=0.8):
    return wall_fusion.Observation(line, 0.0, 2.5, False, source, confidence)


def feed(tracker, observations, n, car=ORIGIN, t0=1.0, dt=0.1):
    for i in range(n):
        tracker.step(list(observations), t0 + i * dt, car)
    return t0 + n * dt


def only(tracker):
    assert len(tracker.tracks) == 1, tracker.tracks
    return tracker.tracks[0]


# --- promotion after exactly N hits ----------------------------------------

@pytest.mark.parametrize('n', [1, 3, N])
def test_a_lidar_wall_is_confirmed_on_exactly_its_nth_consecutive_hit(n):
    trk = mwt.MultiWallTracker(mwt.TrackerParams(confirm_min_consecutive_hits=n))
    t = feed(trk, [obs(wall(2.0))], n - 1)
    if n > 1:
        track = only(trk)
        assert not track.ever_confirmed and track.state == mwt.STATE_TENTATIVE
        assert trk.confirm_reason(track) == 'hits'
        assert trk.published(ORIGIN) == [] and trk.front_track(ORIGIN, FRONT_X) is None
    feed(trk, [obs(wall(2.0))], 1, t0=t)
    track = only(trk)
    assert track.ever_confirmed and track.state == mwt.STATE_CONFIRMED
    assert trk.published(ORIGIN) == [track]
    assert trk.front_track(ORIGIN, FRONT_X) is track


def test_the_default_run_is_five_cycles():
    assert N == 5


def test_one_missed_cycle_ends_a_tentative_track_and_its_run():
    """A tentative track that misses a cycle is no nearer confirmation than one
    its next observation would start, so it goes, and the count starts over."""
    trk = mwt.MultiWallTracker()
    t = feed(trk, [obs(wall(2.0))], N - 1)
    first = only(trk)
    t = feed(trk, [], 1, t0=t)
    assert trk.tracks == []
    t = feed(trk, [obs(wall(2.0))], N - 1, t0=t)
    second = only(trk)
    assert second.id != first.id and not second.ever_confirmed
    feed(trk, [obs(wall(2.0))], 1, t0=t)
    assert second.ever_confirmed


def test_both_sensors_count_like_the_lidar_alone():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0), wall_fusion.SOURCE_BOTH)], N)
    assert only(trk).ever_confirmed


# --- no promotion on stereo alone ------------------------------------------

def test_a_stereo_only_wall_never_confirms():
    trk = mwt.MultiWallTracker()
    assert trk.camera_fresh, 'every camera cycle is fresh: the strictest case'
    feed(trk, [obs(wall(2.0), wall_fusion.SOURCE_STEREO, confidence=0.35)], 4 * N)
    track = only(trk)
    assert not track.ever_confirmed and track.state == mwt.STATE_TENTATIVE
    assert trk.confirm_reason(track) == 'no_lidar'
    assert mwt.reported_role(track) == mwt.ROLE_UNKNOWN
    assert trk.published(ORIGIN) == [] and trk.front_track(ORIGIN, FRONT_X) is None


def test_the_stereo_hits_of_a_track_do_not_count_towards_its_lidar_run():
    """Stereo keeps the track alive and raises its standing; only the LiDAR
    hits make the run."""
    trk = mwt.MultiWallTracker()
    t = feed(trk, [obs(wall(2.0), wall_fusion.SOURCE_STEREO, confidence=0.35)], 3)
    t = feed(trk, [obs(wall(2.0))], N - 1, t0=t)
    track = only(trk)
    assert not track.ever_confirmed
    feed(trk, [obs(wall(2.0))], 1, t0=t)
    assert track.ever_confirmed and track.source == wall_fusion.SOURCE_LIDAR


def test_a_stereo_cycle_inside_the_run_breaks_it():
    trk = mwt.MultiWallTracker()
    t = feed(trk, [obs(wall(2.0))], N - 1)
    t = feed(trk, [obs(wall(2.0), wall_fusion.SOURCE_STEREO, confidence=0.35)], 1, t0=t)
    track = only(trk)
    assert not track.ever_confirmed and trk.confirm_reason(track) == 'hits'
    t = feed(trk, [obs(wall(2.0))], N - 1, t0=t)
    assert not track.ever_confirmed
    feed(trk, [obs(wall(2.0))], 1, t0=t)
    assert track.ever_confirmed


# --- no promotion above the jitter bounds ----------------------------------

@pytest.mark.parametrize('step_m, confirmed', [(0.04, True), (0.07, False)])
def test_a_wall_whose_distance_jumps_more_than_the_bound_is_not_confirmed(step_m, confirmed):
    """A line that alternates between two distances step_m apart: inside the
    5 cm bound it confirms on schedule, outside it never does -- although every
    observation is associated with the track."""
    trk = mwt.MultiWallTracker()
    t = 1.0
    for i in range(3 * N):
        trk.step([obs(wall(2.0 + step_m * (i % 2)))], t, ORIGIN)
        t += 0.1
    track = only(trk)
    assert track.hits == 3 * N, 'every observation was associated'
    assert track.ever_confirmed is confirmed
    if not confirmed:
        assert trk.confirm_reason(track) == 'dist_jitter'


@pytest.mark.parametrize('step_deg, confirmed', [(2.5, True), (3.5, False)])
def test_a_wall_whose_direction_swings_more_than_the_bound_is_not_confirmed(
        step_deg, confirmed):
    trk = mwt.MultiWallTracker()
    t = 1.0
    for i in range(3 * N):
        trk.step([obs(wall(2.0, angle_deg=step_deg * (i % 2)))], t, ORIGIN)
        t += 0.1
    track = only(trk)
    assert track.hits == 3 * N, 'every observation was associated'
    assert track.ever_confirmed is confirmed
    if not confirmed:
        assert trk.confirm_reason(track) == 'angle_jitter'


def fixed_wall_from(car, rho_odom=3.0, t0=-1.5, t1=1.5):
    """The observation a wall FIXED in odom at x = rho_odom produces from `car`."""
    p0, p1 = np.array([rho_odom, t0]), np.array([rho_odom, t1])
    odom = Line(rho_odom, 0.0, np.zeros((2, 2)), p0, p1, 120, 'odom')
    return obs(mwt.line_to_wall_line(odom.transform(car.inverse(), 'base_link')))


def test_driving_towards_a_wall_is_not_jitter():
    """The car closes 10 cm a cycle, so the wall's base_link distance changes
    by 40 cm over the run. In odom, where the bounds are applied, it does not
    move at all, and the wall confirms on schedule."""
    trk = mwt.MultiWallTracker()
    for i in range(N):
        car = Pose2D(0.1 * i, 0.0, 0.0)
        trk.step([fixed_wall_from(car)], 1.0 + 0.1 * i, car)
    track = only(trk)
    assert track.ever_confirmed
    assert track.rho == pytest.approx(3.0, abs=0.01)


def test_line_jitter_reads_a_line_flipped_through_the_origin_as_the_same_line():
    dist, angle = mwt.line_jitter([(0.01, 0.0), (0.01, math.pi), (0.02, 0.0)], (0.0, 0.0))
    # (0.01, pi) is (-0.01, 0): 3 cm of spread, not 0, and no angle at all.
    assert dist == pytest.approx(0.03)
    assert angle == pytest.approx(0.0, abs=1e-12)


def test_line_jitter_measures_the_distance_at_the_reference_point_not_at_the_odom_origin():
    """Five lines through ONE wall point 10 m along the wall from the origin.

    Their direction swings 0.5 deg: the wall does not move at that point,
    while their rho -- the distance from the odom origin -- swings 8.7 cm.
    """
    wall_point = (2.0, 10.0)
    samples = []
    for i in range(N):
        theta = math.radians(0.25 if i % 2 else -0.25)
        samples.append((math.cos(theta) * wall_point[0] + math.sin(theta) * wall_point[1], theta))
    dist, angle = mwt.line_jitter(samples, wall_point)
    assert dist == pytest.approx(0.0, abs=1e-9)
    assert math.degrees(angle) == pytest.approx(0.5)
    assert mwt.line_jitter(samples, (0.0, 0.0))[0] == pytest.approx(0.087, abs=0.001)


def test_the_reason_codes_come_in_the_order_they_are_checked():
    p = mwt.TrackerParams()
    at_wall = (2.0, 0.0)
    steady = [(2.0, 0.0)] * N
    assert mwt.confirmation_reason(steady, 2.0, p, at_wall) == ''
    assert mwt.confirmation_reason(steady[1:], 2.0, p, at_wall) == 'hits'
    assert mwt.confirmation_reason(steady[:-1] + [None], 2.0, p, at_wall) == 'hits'
    assert mwt.confirmation_reason(steady, 0.9, p, at_wall) == 'length'
    jumpy = steady[:-1] + [(2.06, math.radians(4.0))]
    assert mwt.confirmation_reason(jumpy, 2.0, p, at_wall) == 'dist_jitter'
    swung = steady[:-1] + [(2.0, math.radians(4.0))]
    assert mwt.confirmation_reason(swung, 2.0, p, at_wall) == 'angle_jitter'
    # Only the last N cycles are judged: older history neither helps nor hurts.
    assert mwt.confirmation_reason([None, (9.0, 1.0)] + steady, 2.0, p, at_wall) == ''


@pytest.mark.parametrize('car', [Pose2D(0.0, 10.0, 0.0), Pose2D(15.0, -7.0, 1.0),
                                 Pose2D(-20.0, 4.0, -2.5)])
def test_the_same_observations_confirm_on_the_same_cycle_wherever_the_car_is_in_odom(car):
    """Identical base_link observations confirm on cycle N at any odom pose.

    A wall 2 m ahead, its normal swinging 0.5 deg from scan to scan (well
    inside the 3 deg bound). Where the car happens to be in odom -- 10 or
    20 m from where the EKF started, turned or not -- is not a property of
    the wall: the verdict, cycle for cycle, is the one at the origin.
    Measured as odom rho, the swing times the 10 m to the origin's foot point
    read as 8.7 cm and the wall never confirmed.
    """
    def first_confirmed_cycle(pose):
        trk = mwt.MultiWallTracker()
        for i in range(3 * N):
            swing = 0.25 if i % 2 else -0.25
            trk.step([obs(wall(2.0, angle_deg=swing))], 1.0 + 0.1 * i, pose)
            if trk.tracks and only(trk).ever_confirmed:
                assert trk.front_track(pose, FRONT_X) is only(trk)
                return i + 1
        return None

    assert first_confirmed_cycle(ORIGIN) == N
    assert first_confirmed_cycle(car) == N


# --- no promotion on a short segment ---------------------------------------

@pytest.mark.parametrize('length_m, confirmed', [(0.8, False), (1.2, True)])
def test_a_segment_shorter_than_the_minimum_length_is_not_confirmed(length_m, confirmed):
    # Birth lowered below the level's own minimum, so the short wall is
    # tracked at all and the LENGTH test is what decides.
    trk = mwt.MultiWallTracker(mwt.TrackerParams(min_span_birth_m=0.6))
    feed(trk, [obs(wall(2.0, t0=-length_m / 2.0, t1=length_m / 2.0))], 3 * N)
    track = only(trk)
    assert track.ever_confirmed is confirmed
    if not confirmed:
        assert trk.confirm_reason(track) == 'length'


# --- hysteresis: no demotion on one bad cycle ------------------------------

def test_a_confirmed_wall_is_not_demoted_by_one_missed_cycle():
    trk = mwt.MultiWallTracker()
    t = feed(trk, [obs(wall(2.0))], N)
    track = only(trk)
    assert track.ever_confirmed
    t = feed(trk, [], 1, t0=t)
    assert only(trk) is track
    assert track.ever_confirmed and track.state == mwt.STATE_COASTING
    feed(trk, [obs(wall(2.0))], 1, t0=t)
    assert track.ever_confirmed and track.state == mwt.STATE_CONFIRMED
    assert trk.published(ORIGIN) == [track]


def test_a_confirmed_wall_is_not_demoted_by_one_jittery_cycle():
    """An 8 cm jump is beyond the 5 cm bound (and still inside the association
    gate): it would keep a tentative track tentative, and it does nothing to a
    confirmed one."""
    trk = mwt.MultiWallTracker()
    t = feed(trk, [obs(wall(2.0))], N)
    track = only(trk)
    t = feed(trk, [obs(wall(2.08))], 1, t0=t)
    assert only(trk) is track and track.hits == N + 1, 'the jump was associated'
    assert mwt.confirmation_reason(track.confirm_obs, track.length, trk.p,
                                   mwt.track_midpoint(track)) == 'dist_jitter'
    assert track.ever_confirmed and track.state == mwt.STATE_CONFIRMED
    assert trk.front_track(ORIGIN, FRONT_X) is track


def test_a_merge_never_leaves_a_confirmed_track_tentative():
    """_merge folds the newer track into the older one. When only the newer was
    confirmed, the survivor is confirmed on that same cycle -- in its STATE
    too, which is what every reader of MultiWallTrack.state sees."""
    trk = mwt.MultiWallTracker()
    t = feed(trk, [obs(wall(2.0, t0=-1.5, t1=-0.3))], 2)
    older = only(trk)
    newer_line = mwt.observation_line(obs(wall(2.0, t0=-0.2, t1=1.0)), ORIGIN, trk.p)
    trk._spawn(newer_line, t - 0.1)
    newer = trk.tracks[1]
    newer.ever_confirmed, newer.state = True, mwt.STATE_CONFIRMED
    trk._merge()
    assert only(trk) is older
    assert older.ever_confirmed and older.state == mwt.STATE_CONFIRMED
    assert not mwt.is_tentative(older)


# --- the parameters: stack_params.yaml -> launch -> node -> tracker ---------

def _stack_params():
    path = os.path.join(_SRC_DIR, 'f1tenth_params', 'config', 'stack_params.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


def test_the_stack_params_group_is_the_agreed_level_and_the_code_default():
    """A coupling, on purpose: change the level in stack_params.yaml and
    TrackerParams in the same commit, or this fails."""
    doc = _stack_params()
    p = mwt.TrackerParams()
    expected = {
        'wall_confirm_min_consecutive_hits': (5, p.confirm_min_consecutive_hits),
        'wall_confirm_min_length_m': (1.0, p.confirm_min_length_m),
        'wall_confirm_max_dist_jitter_m': (0.05, p.confirm_max_dist_jitter_m),
        'wall_confirm_max_angle_jitter_deg': (3.0, math.degrees(p.confirm_max_angle_jitter_rad)),
        'wall_confirm_require_lidar': (True, p.confirm_require_lidar),
    }
    assert {k for k in doc if k.startswith('wall_confirm_')} == set(expected)
    for key, (value, code_default) in expected.items():
        assert doc[key]['default'] == pytest.approx(value), key
        assert code_default == pytest.approx(value), key


def test_the_rules_it_replaced_are_gone():
    doc = _stack_params()
    for key in ('lidar_front_wall_stereo_confirm_hits', 'lidar_front_wall_stereo_confirm_window',
                'lidar_front_wall_camera_confirm_fresh_only'):
        assert key not in doc, key
    p = mwt.TrackerParams()
    for name in ('confirm_hits', 'confirm_window', 'stereo_confirm_hits',
                 'stereo_confirm_window', 'camera_confirm_fresh_only'):
        assert not hasattr(p, name), name


@pytest.mark.parametrize('bad', [
    {'confirm_min_consecutive_hits': 0}, {'confirm_min_length_m': -0.1},
    {'confirm_max_dist_jitter_m': math.nan}, {'confirm_max_angle_jitter_rad': -1.0}])
def test_a_level_that_cannot_mean_anything_is_refused_at_construction(bad):
    with pytest.raises(ValueError):
        mwt.TrackerParams(**bad)


@pytest.fixture
def rclpy_context():
    rclpy = pytest.importorskip('rclpy')
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_the_node_builds_the_level_from_its_wall_confirm_parameters(rclpy_context):
    from rclpy.parameter import Parameter
    from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode
    n = LidarFrontWallNode(parameter_overrides=[
        Parameter('enable_scene_labels', value=True),
        Parameter('use_wall_tracker', value=True),
        Parameter('wall_confirm_min_consecutive_hits', value=7),
        Parameter('wall_confirm_min_length_m', value=1.5),
        Parameter('wall_confirm_max_dist_jitter_m', value=0.03),
        Parameter('wall_confirm_max_angle_jitter_deg', value=2.0),
        Parameter('wall_confirm_require_lidar', value=False)])
    try:
        p = n.wall_tracker.p
        assert p.confirm_min_consecutive_hits == 7
        assert p.confirm_min_length_m == pytest.approx(1.5)
        assert p.confirm_max_dist_jitter_m == pytest.approx(0.03)
        assert p.confirm_max_angle_jitter_rad == pytest.approx(math.radians(2.0))
        assert p.confirm_require_lidar is False
    finally:
        n.destroy_node()


def test_the_node_defaults_are_the_stack_params_group(rclpy_context):
    from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode
    from f1tenth_params.param_defaults import get_value
    n = LidarFrontWallNode()
    try:
        for name in ('wall_confirm_min_consecutive_hits', 'wall_confirm_min_length_m',
                     'wall_confirm_max_dist_jitter_m', 'wall_confirm_max_angle_jitter_deg',
                     'wall_confirm_require_lidar'):
            assert n.get_parameter(name).value == get_value(name), name
    finally:
        n.destroy_node()
