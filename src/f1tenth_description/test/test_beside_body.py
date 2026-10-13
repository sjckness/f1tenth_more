"""
beside_body_behind_bumper: a footprint beside the car is not a front reading.

2026-10-13: behind the bumper face, a point outside the car's width
(|y| > half-width + margin) is BESIDE the car and is excluded from the front
distances; inside the width it is touching and still counts (clamped to
MIN_FRONT_DISTANCE_M by the callers).
"""

import math

from f1tenth_description.vehicle_frames import beside_body_behind_bumper, front_side_limit_m
from f1tenth_params.param_defaults import get_value
import pytest

FRONT = 0.425
SIDE = 0.186


def _beside(x, y, r=0.0, corridor=0.35):
    return beside_body_behind_bumper(
        x, y, r, front_x=FRONT, side_limit_m=SIDE, corridor_half_width_m=corridor)


def test_the_side_limit_is_the_body_half_width_plus_the_margin():
    assert front_side_limit_m() == pytest.approx(
        get_value('swept_clearance_body_half_width_m') + get_value('front_beside_body_margin_m'))
    assert front_side_limit_m() == pytest.approx(0.186)


@pytest.mark.parametrize('x, y', [(0.30, 0.25), (0.30, -0.25), (0.425, 0.20), (0.10, 0.34)])
def test_a_point_behind_the_bumper_face_outside_the_width_is_beside(x, y):
    assert _beside(x, y)


@pytest.mark.parametrize('x, y', [(0.30, 0.0), (0.30, 0.186), (0.425, -0.10)])
def test_a_point_behind_the_bumper_face_inside_the_width_is_touching(x, y):
    assert not _beside(x, y)


@pytest.mark.parametrize('x, y', [(0.43, 0.30), (1.0, 0.0), (0.50, 0.25)])
def test_a_point_ahead_of_the_bumper_face_is_never_beside(x, y):
    assert not _beside(x, y)


def test_a_person_next_to_the_nose_is_beside():
    # Centre 0.45 m to the side, 0.25 m radius, 0.275 m behind the bumper
    # face: reaches |y| 0.20 > 0.186 and never gets ahead of the bumper.
    assert _beside(0.15, 0.45, 0.25)


def test_a_person_beside_the_nose_who_sticks_out_ahead_is_in_the_way():
    # Same person 0.15 m further forward: the disc reaches (0.425..0.55,
    # |y| 0.20..0.35) -- the corridor just ahead of the bumper.
    assert not _beside(0.30, 0.45, 0.25)


def test_a_disc_reaching_into_the_car_width_is_touching():
    assert not _beside(0.30, 0.30, 0.15)   # reaches |y| 0.15 <= 0.186


def test_a_wide_disc_that_also_sticks_out_ahead_is_in_the_way():
    # Near face 0.40 (behind the bumper), centre beside, but it covers
    # (0.60..1.0, |y| < 0.35): in the corridor ahead of the car.
    assert not _beside(1.0, 0.8, 0.6)
    # The same disc with no corridor to reach is beside.
    assert _beside(1.0, 0.8, 0.6, corridor=0.0)


def test_the_point_case_is_exactly_the_vectorised_rule():
    # costmap_boundary.nearest_occupied_in_window applies the r = 0 case as
    # (x <= front) & (|y| > side) on whole grids; the two must agree.
    for x in [i * 0.05 for i in range(0, 15)]:
        for y in [j * 0.05 - 0.5 for j in range(0, 21)]:
            vec = (x <= FRONT) and (abs(y) > SIDE)
            assert _beside(x, y, 0.0, corridor=math.inf) == vec, (x, y)
