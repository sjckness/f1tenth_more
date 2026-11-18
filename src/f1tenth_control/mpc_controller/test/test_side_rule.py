"""The side rule (mpc_controller/side_rule.py): which way round an obstacle on the car's line."""

import math

import pytest

from f1tenth_params.param_defaults import get_value
from mpc_controller import side_rule
from mpc_controller.obstacle_motion import Motion
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


# ---- the informed rule (mpc-dodge) ------------------------------------------------------------


def _informed(obstacles, motions, latch, free=None):
    return side_rule.break_ties_informed(obstacles, motions, (0.0, 0.0), 0.0, 0.05, 0.15, 1.0,
                                         3.0, latch, v_min=0.1, free_space=free)


def test_informed_passes_behind_a_moving_obstacle_and_latches_the_side():
    latch = side_rule.SideLatch()
    out, decided = _informed([(1.5, 0.01, 0.25)], [Motion(7, 0.0, 0.4, True)], latch)
    assert decided == [(0, -1.0, 'velocity')]           # moving left: pass on its right
    assert out[0][1] > 0.01 and out[0][2] > 0.25         # displaced left, grown
    # next tick it has moved out of the band but inside the release: the side holds
    _out, decided = _informed([(1.4, 0.10, 0.25)], [Motion(7, 0.0, -0.4, True)], latch)
    assert decided == [(0, -1.0, 'latched')]
    # past the release: free again
    _informed([(1.3, 0.30, 0.25)], [Motion(7, 0.0, 0.0, False)], latch)
    assert 7 not in latch.sides


def test_informed_stopped_obstacle_goes_to_the_side_it_came_from_then_space_then_fixed():
    latch = side_rule.SideLatch()
    _informed([(2.0, -0.6, 0.25)], [Motion(3, 0.0, 0.4, True)], latch)     # seen on the right
    _out, decided = _informed([(1.8, 0.0, 0.25)], [Motion(3, 0.0, 0.0, False)], latch)
    assert decided == [(0, -1.0, 'came_from')]
    _out, decided = _informed([(1.8, 0.0, 0.25)], [None], side_rule.SideLatch(),
                              free=lambda a, lat: (0.5, 2.0))
    assert decided == [(0, -1.0, 'space')]
    _out, decided = _informed([(1.8, 0.0, 0.25)], [None], side_rule.SideLatch())
    assert decided == [(0, 1.0, 'fixed')]
