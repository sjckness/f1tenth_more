"""The side rule (mpc_controller/side_rule.py): which way round an obstacle on the car's line."""

import math

import pytest

from f1tenth_params.param_defaults import get_value
from mpc_controller.side_rule import break_ties, side_sign

LEFT, RIGHT = side_sign('left'), side_sign('right')


def test_the_sides_and_a_bad_name():
    assert LEFT == 1.0 and RIGHT == -1.0
    with pytest.raises(ValueError):
        side_sign('on')


def test_the_shipped_parameters_are_valid():
    assert float(get_value('mpc_obstacle_tie_band_m')) > 0.0
    side_sign(get_value('mpc_obstacle_tie_side'))


def test_a_centred_obstacle_moves_to_the_other_side_and_grows():
    out, n = break_ties([(1.5, 0.0, 0.5)], (0.0, 0.0), 0.0, 0.05, LEFT, 3.0)
    assert n == 1
    x, y, r = out[0]
    assert (x, y, r) == pytest.approx((1.5, -0.05, 0.55))


def test_the_displaced_disc_contains_the_real_one():
    for lat in (-0.049, -0.02, 0.0, 0.03, 0.049):
        for side in (LEFT, RIGHT):
            (x, y, r), = break_ties([(2.0, lat, 0.25)], (0.0, 0.0), 0.0, 0.05, side, 3.0)[0]
            assert math.hypot(x - 2.0, y - lat) + 0.25 <= r + 1e-12
            # and on the preferred side the two discs share the edge: no clearance lost there
            assert y + side * r == pytest.approx(lat + side * 0.25)


def test_outside_the_band_behind_or_beyond_reach_nothing_changes():
    obstacles = [(1.5, 0.2, 0.5), (-1.0, 0.0, 0.5), (4.0, 0.0, 0.5)]
    out, n = break_ties(obstacles, (0.0, 0.0), 0.0, 0.05, LEFT, 3.0)
    assert n == 0 and out == obstacles


def test_it_follows_the_heading_and_the_car_position():
    # car at (1, 1) heading +y; obstacle 1.5 m ahead on its line -> displaced to +x (its right)
    (x, y, r), = break_ties([(1.0, 2.5, 0.3)], (1.0, 1.0), math.pi / 2, 0.05, LEFT, 3.0)[0]
    assert (x, y, r) == pytest.approx((1.05, 2.5, 0.35))


def test_band_zero_turns_it_off():
    obstacles = [(1.5, 0.0, 0.5)]
    assert break_ties(obstacles, (0.0, 0.0), 0.0, 0.0, LEFT, 3.0) == (obstacles, 0)
