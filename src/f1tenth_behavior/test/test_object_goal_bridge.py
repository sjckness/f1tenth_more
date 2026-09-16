"""The standoff geometry: the part of the bridge that can hurt someone."""

import math

import pytest

from f1tenth_behavior.object_goal_bridge import standoff_goal


def test_the_goal_sits_short_of_the_target_on_the_line_to_it():
    (gx, gy), yaw, distance = standoff_goal((0.0, 0.0), (5.0, 0.0), 1.0)

    assert (gx, gy) == pytest.approx((4.0, 0.0))
    assert distance == pytest.approx(5.0)
    assert yaw == pytest.approx(0.0)


@pytest.mark.parametrize('target', [(5.0, 5.0), (-3.0, 4.0), (0.0, -7.0),
                                    (2.5, -1.25)])
@pytest.mark.parametrize('standoff', [0.5, 1.0, 2.0])
def test_the_goal_never_lands_past_the_target(target, standoff):
    """Past the target means driving into it."""
    vehicle = (1.0, -0.5)
    goal, _, distance = standoff_goal(vehicle, target, standoff)

    to_target = math.dist(vehicle, target)
    to_goal = math.dist(vehicle, goal)

    assert to_goal <= to_target, 'goal is beyond the target'
    if distance > standoff:
        assert math.dist(goal, target) == pytest.approx(standoff, abs=1e-9)
        assert to_goal == pytest.approx(distance - standoff, abs=1e-9)
    else:
        # Already inside the standoff -- holding position is the whole point.
        assert goal == pytest.approx(vehicle)


@pytest.mark.parametrize('distance', [0.0, 0.2, 0.9, 0.999])
def test_inside_the_standoff_the_goal_collapses_to_the_vehicle(distance):
    """Never a goal behind the vehicle: the MPC would reverse toward a person."""
    vehicle = (2.0, 3.0)
    target = (2.0 + distance, 3.0)

    goal, _, reported = standoff_goal(vehicle, target, 1.0)

    assert goal == pytest.approx(vehicle)
    assert reported == pytest.approx(distance)


def test_a_coincident_target_does_not_divide_by_zero():
    goal, yaw, distance = standoff_goal((1.0, 1.0), (1.0, 1.0), 1.0)

    assert goal == pytest.approx((1.0, 1.0))
    assert distance == 0.0
    assert yaw == 0.0


@pytest.mark.parametrize('target,expected_deg', [
    ((1.0, 0.0), 0.0), ((0.0, 1.0), 90.0), ((-1.0, 0.0), 180.0),
    ((0.0, -1.0), -90.0),
])
def test_the_yaw_points_at_the_target(target, expected_deg):
    _, yaw, _ = standoff_goal((0.0, 0.0), target, 0.1)

    assert math.degrees(yaw) == pytest.approx(expected_deg)


# -- target selection among already-associated tracks ---------------------

from f1tenth_behavior.object_goal_bridge import nearest_track  # noqa: E402

TRACKS = [
    ('1', 'person', 5.0, 0.0, 0.9),
    ('2', 'chair', 1.0, 0.0, 0.9),
    ('3', 'person', 2.0, 0.0, 0.8),
    ('4', 'person', 3.0, 0.0, 0.2),
]


def test_the_nearest_track_of_the_wanted_class_wins():
    distance, track_id, xy = nearest_track(TRACKS, (0.0, 0.0), 'person', 0.5)

    assert track_id == '3', 'the chair at 1 m must not be chosen'
    assert distance == pytest.approx(2.0)
    assert xy == (2.0, 0.0)


def test_a_nearer_object_of_another_class_is_ignored():
    """The whole point: class-blind selection is what stops at the chair."""
    _, track_id, _ = nearest_track(TRACKS, (0.0, 0.0), 'chair', 0.5)

    assert track_id == '2'


def test_low_confidence_tracks_are_skipped():
    _, track_id, _ = nearest_track(TRACKS, (2.9, 0.0), 'person', 0.5)

    assert track_id == '3', 'track 4 is nearer but below the confidence floor'


def test_no_track_of_the_class_returns_nothing():
    assert nearest_track(TRACKS, (0.0, 0.0), 'bottle', 0.5) is None
    assert nearest_track([], (0.0, 0.0), 'person', 0.5) is None


def test_selection_follows_the_vehicle():
    """Nearest is relative to where the car is, not to the map origin."""
    _, near_origin, _ = nearest_track(TRACKS, (0.0, 0.0), 'person', 0.5)
    _, near_far_end, _ = nearest_track(TRACKS, (10.0, 0.0), 'person', 0.5)

    assert near_origin == '3'
    assert near_far_end == '1'


def test_tracks_missing_a_hypothesis_do_not_crash_selection():
    assert nearest_track([('9', '', 1.0, 1.0, 0.0)], (0.0, 0.0), 'person', 0.5) is None
