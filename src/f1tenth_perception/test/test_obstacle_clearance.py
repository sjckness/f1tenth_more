"""obstacle_clearance.py against closed-form answers -- no ROS.

The footprint is the rectangle x in [rear_x, rear_x + length], |y| <= width/2.
Every expected value below is worked out by hand from that definition, not by
calling the function under test a second way.

Run standalone: python3 -m pytest test/test_obstacle_clearance.py -v
"""

import math

import numpy as np
import pytest

from f1tenth_perception.obstacle_clearance import (
    footprint_clearance, footprint_signed_distances)

# a round-number car: 0.5 m long from x = -0.1 to x = 0.4, 0.3 m wide
L, W, REAR = 0.5, 0.3, -0.1
FRONT = REAR + L
HALF_W = W / 2.0


def _wall(x=None, y=None, span=(-5.0, 5.0), n=2001):
    """Points of an infinite-ish straight wall at x = const or y = const."""
    s = np.linspace(*span, n)
    if x is not None:
        return np.stack([np.full(n, x), s], 1)
    return np.stack([s, np.full(n, y)], 1)


@pytest.mark.parametrize('gap', [0.05, 0.5, 2.0, 7.3])
def test_a_wall_ahead_reads_its_gap_to_the_front_face(gap):
    assert footprint_clearance(_wall(x=FRONT + gap), L, W, REAR) == pytest.approx(gap, abs=1e-9)


@pytest.mark.parametrize('gap', [0.05, 1.0])
def test_a_wall_behind_reads_its_gap_to_the_tail(gap):
    assert footprint_clearance(_wall(x=REAR - gap), L, W, REAR) == pytest.approx(gap, abs=1e-9)


@pytest.mark.parametrize('side', [1.0, -1.0])
def test_a_wall_beside_reads_its_gap_to_the_flank(side):
    y = side * (HALF_W + 0.4)
    assert footprint_clearance(_wall(y=y), L, W, REAR) == pytest.approx(0.4, abs=1e-9)


def test_a_point_off_a_corner_is_measured_to_the_corner_not_a_face():
    # 0.3 m ahead of the front face and 0.4 m out from the flank: a 3-4-5 triangle
    point = [[FRONT + 0.3, HALF_W + 0.4]]
    assert footprint_clearance(point, L, W, REAR) == pytest.approx(0.5, abs=1e-12)


def test_the_nearest_of_several_obstacles_wins():
    points = np.vstack([_wall(x=FRONT + 2.0), _wall(y=-(HALF_W + 0.25)), [[REAR - 3.0, 0.0]]])
    assert footprint_clearance(points, L, W, REAR) == pytest.approx(0.25, abs=1e-9)


def test_a_point_touching_the_edge_is_zero():
    assert footprint_clearance([[FRONT, 0.0]], L, W, REAR) == pytest.approx(0.0, abs=1e-12)
    assert footprint_clearance([[0.1, HALF_W]], L, W, REAR) == pytest.approx(0.0, abs=1e-12)


def test_a_point_inside_is_minus_its_depth_to_the_nearest_face():
    # 0.02 m behind the front face, on the centreline: nearest face is the front
    assert footprint_clearance([[FRONT - 0.02, 0.0]], L, W, REAR) == pytest.approx(-0.02)
    # 0.03 m in from the left flank, mid-length
    assert footprint_clearance([[0.15, HALF_W - 0.03]], L, W, REAR) == pytest.approx(-0.03)
    # dead centre: half the width is the shallowest way out
    assert footprint_clearance([[REAR + L / 2, 0.0]], L, W, REAR) == pytest.approx(-HALF_W)


def test_no_points_is_infinite_clearance():
    assert footprint_clearance(np.empty((0, 2)), L, W, REAR) == math.inf


def test_the_signed_distance_is_continuous_across_the_edge():
    xs = np.linspace(FRONT - 0.01, FRONT + 0.01, 201)
    d = footprint_signed_distances(np.stack([xs, np.zeros_like(xs)], 1), L, W, REAR)
    assert np.allclose(d, xs - FRONT, atol=1e-12)


def test_rear_x_moves_the_footprint_along_x():
    wall = _wall(x=1.0)
    assert footprint_clearance(wall, L, W, rear_x=0.0) == pytest.approx(0.5)
    assert footprint_clearance(wall, L, W, rear_x=0.2) == pytest.approx(0.3)


@pytest.mark.parametrize('length, width', [(0.0, 0.3), (0.5, -0.1)])
def test_a_degenerate_footprint_is_refused(length, width):
    with pytest.raises(ValueError):
        footprint_clearance([[1.0, 0.0]], length, width)
