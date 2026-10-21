"""The front-wall lock's rules, with no ROS (mission/wall_lock.py).

The car drives +x in odom from the origin; the wall it is sent to is the line
x = 4 (rho 4, theta 0). A side wall is y = 1.2 (rho 1.2, theta pi/2). The
bumper is passed in directly.

Run standalone: python3 -m pytest test/test_wall_lock.py -v
"""

import math

import pytest

from f1tenth_behavior.mission.wall_lock import (
    EVENT_ACQUIRED, EVENT_REACQUIRED, EVENT_REFUSED, LOCKED, LOST, NO_READING,
    NOT_FRONT, SOURCE_BOTH, SOURCE_LIDAR, SOURCE_STEREO, STATE_COASTING,
    NO_LOCK_ID, STATE_CONFIRMED, STATE_TENTATIVE, UNLOCKED,
    FrontReading, FrontWallLock, LockParams, LockStep, WallTrack, ramp_wall_id, same_wall)

TARGET = 7
SIDE = 9
TWIN = 12        # the target wall, re-born under a new id

GATE_M = 0.30
GATE_RAD = math.radians(15.0)


def _wall(track_id=TARGET, rho=4.0, theta=0.0, state=STATE_CONFIRMED, role='front',
          source=SOURCE_LIDAR):
    return WallTrack(track_id=track_id, state=state, role=role, rho=rho, theta=theta,
                     source=source)


def _side(track_id=SIDE, state=STATE_CONFIRMED):
    return _wall(track_id, rho=1.2, theta=math.pi / 2, state=state, role='left')


def _tracks(*walls):
    return {w.track_id: w for w in walls}


def _front(front_id, distance):
    return FrontReading(state='wall', distance=distance, front_id=front_id)


CLEAR = FrontReading(state='clear', distance=50.0, front_id=None)


def _locked(now=0.0, bumper=(0.4, 0.0)):
    lock = FrontWallLock(LockParams(gate_m=GATE_M, gate_rad=GATE_RAD))
    step = lock.update(now, 3.6, _front(TARGET, 3.6), _tracks(_wall()), bumper, 0.0)
    assert step.event == EVENT_ACQUIRED
    return lock


def test_states_and_sources_mirror_the_message_constants():
    msg = pytest.importorskip('f1tenth_messages.msg')
    assert STATE_TENTATIVE == msg.MultiWallTrack.STATE_TENTATIVE
    assert STATE_CONFIRMED == msg.MultiWallTrack.STATE_CONFIRMED
    assert STATE_COASTING == msg.MultiWallTrack.STATE_COASTING
    assert SOURCE_LIDAR == msg.MultiWallTrack.SOURCE_LIDAR
    assert SOURCE_STEREO == msg.MultiWallTrack.SOURCE_STEREO
    assert SOURCE_BOTH == msg.MultiWallTrack.SOURCE_BOTH


# -- unlocked -------------------------------------------------------------------

def test_unlocked_reads_the_raw_distance_while_there_is_no_front_wall():
    lock = FrontWallLock()
    step = lock.update(0.0, 50.0, CLEAR, _tracks(_side()), (0.4, 0.0), 0.0)
    assert (step.phase, step.distance, step.track_id) == (UNLOCKED, 50.0, None)


def test_unlocked_without_a_debug_payload_is_the_old_behaviour():
    step = FrontWallLock().update(0.0, 1.7, None, None, None, None)
    assert (step.phase, step.distance) == (UNLOCKED, 1.7)


def test_the_first_front_wall_is_locked_and_its_distance_read():
    lock = FrontWallLock()
    step = lock.update(0.0, 3.6, _front(TARGET, 3.6), _tracks(_wall(), _side()),
                       (0.4, 0.0), 0.0)
    assert (step.phase, step.distance, step.track_id, step.event) == (
        LOCKED, 3.6, TARGET, EVENT_ACQUIRED)
    assert lock.anchor == (4.0, 0.0)


def test_a_front_not_on_a_fresh_walls_message_is_not_locked():
    lock = FrontWallLock()
    step = lock.update(0.0, 3.6, _front(TARGET, 3.6), None, (0.4, 0.0), 0.0)
    assert (step.phase, step.distance, step.track_id) == (UNLOCKED, 3.6, None)
    step = lock.update(0.1, 3.5, _front(TARGET, 3.5), _tracks(_side()), (0.5, 0.0), 0.1)
    assert (step.phase, step.distance, step.track_id) == (UNLOCKED, 3.5, None)


def test_a_stereo_only_front_is_not_locked_and_the_next_lidar_one_is():
    """The bf7938 mission (2026-10-21 12:25): a stereo-only track, 23 deg
    oblique, was FRONT for two steps at 4.6 m; the real wall (LiDAR + stereo,
    0.88 m / 25.7 deg off it) came twelve seconds later. Locking the first
    would have refused the second."""
    lock = FrontWallLock()
    phantom = _wall(SIDE, rho=5.35, theta=math.radians(-23.4), source=SOURCE_STEREO)
    step = lock.update(0.0, 4.6, _front(SIDE, 4.6), _tracks(phantom), (0.4, 0.0), 0.0)
    assert (step.phase, step.distance, step.track_id) == (UNLOCKED, 4.6, None)
    assert 'stereo-only' in step.detail
    step = lock.update(12.0, 2.3, _front(TARGET, 2.3),
                       _tracks(_wall(rho=4.0, source=SOURCE_BOTH)), (1.7, 0.0), 1.3)
    assert (step.phase, step.distance, step.track_id, step.event) == (
        LOCKED, 2.3, TARGET, EVENT_ACQUIRED)


# -- locked ---------------------------------------------------------------------

def test_locked_reads_the_contract_distance_while_its_wall_is_front():
    lock = _locked()
    step = lock.update(0.1, 2.0, _front(TARGET, 2.0), _tracks(_wall()), (2.0, 0.0), 1.6)
    assert (step.phase, step.distance, step.event) == (LOCKED, 2.0, None)


def test_a_side_wall_that_becomes_front_is_never_read():
    """THE R029 CASE: a swerve puts the side wall in the travel strip, nearer
    than the target. The raw wall topic says 0.9 m; the stop must not see it."""
    lock = _locked()
    step = lock.update(0.2, 0.9, _front(SIDE, 0.9), _tracks(_wall(), _side()),
                       (1.0, 0.3), 0.7)
    assert (step.phase, step.distance, step.track_id) == (NOT_FRONT, None, TARGET)
    assert f'track {SIDE} is the front wall' in step.detail


def test_clear_while_the_locked_wall_is_tracked_reads_not_yet():
    lock = _locked()
    step = lock.update(0.2, 50.0, CLEAR, _tracks(_wall(role='left')), (1.0, 0.3), 0.7)
    assert (step.phase, step.distance) == (NOT_FRONT, None)


def test_the_locked_wall_back_in_front_is_read_again():
    lock = _locked()
    lock.update(0.2, 0.9, _front(SIDE, 0.9), _tracks(_wall(), _side()), (1.0, 0.3), 0.7)
    step = lock.update(0.4, 2.4, _front(TARGET, 2.4), _tracks(_wall(), _side()),
                       (1.6, 0.0), 1.3)
    assert (step.phase, step.distance, step.event) == (LOCKED, 2.4, None)


def test_no_debug_payload_while_locked_reads_not_yet():
    lock = _locked()
    step = lock.update(0.1, 2.0, None, _tracks(_wall()), (2.0, 0.0), 1.6)
    assert (step.phase, step.distance) == (NO_READING, None)


def test_the_anchor_follows_the_locked_tracks_confirmed_estimate():
    lock = _locked()
    lock.update(0.1, 3.4, _front(TARGET, 3.4), _tracks(_wall(rho=3.8)), (0.4, 0.0), 0.2)
    assert lock.anchor == (3.8, 0.0)
    # ...but not its coasting prediction.
    lock.update(0.2, 3.3, _front(TARGET, 3.3),
                _tracks(_wall(rho=3.6, state=STATE_COASTING)), (0.4, 0.0), 0.3)
    assert lock.anchor == (3.8, 0.0)


# -- coasting and loss ------------------------------------------------------------

def test_a_coasting_locked_wall_within_the_caps_is_not_replaced():
    lock = _locked()
    coasting = _wall(state=STATE_COASTING)
    step = lock.update(0.3, 3.5, _front(TWIN, 3.5), _tracks(coasting, _wall(TWIN)),
                       (0.5, 0.0), 0.1)
    assert (step.phase, step.track_id) == (NOT_FRONT, TARGET)


@pytest.mark.parametrize('now, odometer', [(0.6, 0.1), (0.3, 0.35)])
def test_a_locked_wall_coasting_past_a_cap_is_lost_and_its_twin_takes_over(now, odometer):
    """0.5 s / 0.3 m: the tracker's own FRONT coast caps."""
    lock = _locked()
    coasting = _wall(state=STATE_COASTING)
    step = lock.update(now, 3.5, _front(TWIN, 3.5), _tracks(coasting, _wall(TWIN)),
                       (0.5, 0.0), odometer)
    assert (step.phase, step.distance, step.track_id, step.event) == (
        LOCKED, 3.5, TWIN, EVENT_REACQUIRED)


def test_a_vanished_locked_wall_is_reacquired_on_the_same_line():
    lock = _locked()
    twin = _wall(TWIN, rho=4.1)                  # 0.10 m off: same wall
    step = lock.update(1.0, 2.0, _front(TWIN, 2.0), _tracks(twin), (2.0, 0.0), 1.6)
    assert (step.phase, step.distance, step.track_id, step.event) == (
        LOCKED, 2.0, TWIN, EVENT_REACQUIRED)
    assert lock.anchor == (4.1, 0.0)
    assert f'track {TARGET} -> {TWIN}' in step.detail


def test_a_surface_nearer_than_the_gate_is_refused():
    """An object face or a cabinet 0.5 m in front of the lost wall."""
    lock = _locked()
    nearer = _wall(TWIN, rho=3.5)
    step = lock.update(1.0, 1.5, _front(TWIN, 1.5), _tracks(nearer), (2.0, 0.0), 1.6)
    assert (step.phase, step.distance, step.track_id, step.event) == (
        LOST, None, TARGET, EVENT_REFUSED)
    assert 'not adopted' in step.detail


def test_a_side_wall_at_the_same_distance_is_refused_by_the_normal():
    """What one distance gate could not do: the side wall's line meets the
    bumper at the locked wall's predicted offset, but it faces 90 deg away."""
    lock = _locked()
    # x = 4 seen from bumper (2, 0) is 2.0 m ahead; y = 2 is 2.0 m to the left.
    side = _wall(SIDE, rho=2.0, theta=math.pi / 2, role='front')
    step = lock.update(1.0, 2.0, _front(SIDE, 2.0), _tracks(side), (2.0, 0.0), 1.6)
    assert (step.phase, step.event, step.track_id) == (LOST, EVENT_REFUSED, TARGET)


def test_lost_with_nothing_in_front_reads_not_yet():
    lock = _locked()
    step = lock.update(1.0, 50.0, CLEAR, _tracks(), (2.0, 0.0), 1.6)
    assert (step.phase, step.distance, step.event) == (LOST, None, None)


def test_a_tentative_front_never_reacquires():
    lock = _locked()
    step = lock.update(1.0, 2.0, _front(TWIN, 2.0),
                       _tracks(_wall(TWIN, state=STATE_TENTATIVE)), (2.0, 0.0), 1.6)
    assert (step.phase, step.distance) == (LOST, None)


def test_a_locked_wall_turned_unknown_is_lost_and_does_not_move_the_anchor():
    lock = _locked()
    step = lock.update(0.1, 2.0, _front(TWIN, 2.0),
                       _tracks(_wall(rho=3.0, role='unknown'), _wall(TWIN)), (2.0, 0.0), 1.6)
    assert (step.track_id, step.event) == (TWIN, EVENT_REACQUIRED)
    # Gated against x = 4 (the last meaningful estimate), not x = 3.
    assert 'track 7 -> 12, 0.00 m' in step.detail


def test_no_pose_refuses_reacquisition():
    lock = _locked()
    step = lock.update(1.0, 2.0, _front(TWIN, 2.0), _tracks(_wall(TWIN)), None, None)
    assert (step.phase, step.event) == (LOST, EVENT_REFUSED)


def test_a_stereo_only_twin_on_the_locked_line_may_reacquire():
    """Lockability is about the FIRST commitment. A replacement is judged by
    the gate against a wall the LiDAR already vouched for."""
    lock = _locked()
    step = lock.update(1.0, 2.0, _front(TWIN, 2.0),
                       _tracks(_wall(TWIN, rho=4.05, source=SOURCE_STEREO)), (2.0, 0.0), 1.6)
    assert (step.track_id, step.event) == (TWIN, EVENT_REACQUIRED)


def test_reset_forgets_the_wall():
    lock = _locked()
    lock.reset()
    step = lock.update(0.0, 0.9, _front(SIDE, 0.9), _tracks(_side()), (0.4, 0.0), 0.0)
    assert (step.track_id, step.event) == (SIDE, EVENT_ACQUIRED)


# -- the gate -------------------------------------------------------------------

def test_same_wall_measures_the_offset_where_the_car_is():
    ok, offset, angle = same_wall((4.0, 0.0), (4.2, 0.0), (2.0, 0.0), GATE_M, GATE_RAD)
    assert ok and offset == pytest.approx(0.2) and angle == pytest.approx(0.0)


def test_same_wall_is_the_dead_reckoned_prediction():
    """The anchor line seen from where the car is now is _predict's d0 - (p - p0) . n."""
    d0, p0, p = 3.6, (0.4, 0.0), (1.9, 0.2)
    anchor = (d0 + p0[0], 0.0)          # x = 4 in odom
    predicted = d0 - (p[0] - p0[0])
    candidate = (p[0] + predicted + 0.25, 0.0)
    ok, offset, _ = same_wall(anchor, candidate, p, GATE_M, GATE_RAD)
    assert ok and offset == pytest.approx(0.25)


def test_same_wall_flips_a_line_normalised_the_other_way():
    # The same line, x = -0.1: (0.1, pi) and (-0.1, 0) are one line.
    ok, offset, angle = same_wall((0.1, math.pi), (-0.1, 0.0), (-1.0, 0.0), GATE_M, GATE_RAD)
    assert ok and offset == pytest.approx(0.0, abs=1e-9) and angle == pytest.approx(0.0)


def test_same_wall_refuses_a_rotated_line():
    ok, _, angle = same_wall((4.0, 0.0), (4.0, math.radians(20.0)), (4.0, 0.0),
                             GATE_M, GATE_RAD)
    assert not ok and math.degrees(angle) == pytest.approx(20.0)


# -- what goes out for the deceleration ramp ---------------------------------------

@pytest.mark.parametrize('phase, track_id, expected', [
    (LOCKED, TARGET, TARGET),
    (NOT_FRONT, TARGET, TARGET),        # the swerve: still the wall the move approaches
    (NO_READING, TARGET, TARGET),
    (UNLOCKED, None, NO_LOCK_ID),
    (LOST, TARGET, NO_LOCK_ID),         # lost: the ramp falls back to the raw wall
])
def test_ramp_wall_id_is_the_locked_track_only_while_the_lock_holds(phase, track_id, expected):
    assert ramp_wall_id(LockStep(phase, None, track_id)) == expected


def test_ramp_wall_id_follows_a_lock_through_swerve_loss_and_reacquisition():
    lock = _locked()
    ids = [ramp_wall_id(lock.update(0.1, 3.5, _front(TARGET, 3.5), _tracks(_wall(), _side()),
                                    (0.5, 0.0), 0.1))]
    ids.append(ramp_wall_id(lock.update(0.2, 0.8, _front(SIDE, 0.8),
                                        _tracks(_wall(), _side()), (1.2, 0.3), 0.9)))
    ids.append(ramp_wall_id(lock.update(0.3, 0.8, _front(SIDE, 0.8), _tracks(_side()),
                                        (1.3, 0.3), 1.0)))
    ids.append(ramp_wall_id(lock.update(0.4, 2.0, _front(TWIN, 2.0),
                                        _tracks(_wall(TWIN, rho=4.05), _side()),
                                        (2.0, 0.0), 1.7)))
    assert ids == [TARGET, TARGET, NO_LOCK_ID, TWIN]
