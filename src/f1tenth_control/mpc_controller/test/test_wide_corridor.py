"""The campaign's wide fixed corridor (M02 campaign fix A2).

corr_wmin / corr_wmax / corr_L_base were bare literals in MPC_corr.py
(0.4333 / 0.7667 / 3.0, and 1.5 live on the Orin from an uncommitted edit).
They are ROS parameters read from stack_params.yaml now, and the shipped
values give a corridor a centred 0.5 m person can be passed in.
"""

import numpy as np
import pytest

from f1tenth_params.param_defaults import get_value
from mpc_controller.MPC_corr import MPCController

import test_object_approach_closed_loop as rig

PERSON_HALF_WIDTH_M = 0.25


def _goal_distance_corridor(wmin, wmax, L):
    fake = rig._ObjectMPC(standoff=1.0, speed=0.4)
    fake.corr_wmin = wmin
    fake.corr_wmax = wmax
    fake.corr_L_base = L
    fake.goal_distance = 5.0
    fake.goal_start_xy = (0.0, 0.0)
    fake.goal_anchor_odom = (0.0, 0.0, 0.0)
    fake.psi_init_corridor = 0.0
    return MPCController.build_straight_corridor(fake, np.zeros(4))


class TestTheShippedGeometry:

    def test_the_values(self):
        assert get_value('corr_wmin') == pytest.approx(0.60)
        assert get_value('corr_wmax') == pytest.approx(0.60)
        assert get_value('corr_L_base') == pytest.approx(3.0)

    def test_a_centred_person_is_passable_by_the_rear_axle(self):
        """The rows bound the rear axle, so the axle needs r + car_radius + margin."""
        needed = (PERSON_HALF_WIDTH_M + get_value('car_radius')
                  + get_value('obstacle_safety_margin_m'))
        assert needed == pytest.approx(0.57)
        assert get_value('corr_wmin') >= needed
        assert get_value('corr_wmax') >= needed

    def test_the_corridor_is_at_least_the_horizon_reach_long(self):
        reach = (get_value('mpc_horizon_steps') * get_value('mpc_prediction_dt_sec')
                 * 0.5)
        assert get_value('corr_L_base') >= reach
        assert (get_value('corr_lookahead_frac') * get_value('corr_L_base')
                >= 1.25 * reach)


def test_the_built_corridor_is_fixed_width_with_the_shipped_values():
    c = _goal_distance_corridor(get_value('corr_wmin'), get_value('corr_wmax'),
                                get_value('corr_L_base'))
    hw = np.asarray(c['halfWidth'])
    assert hw.min() == pytest.approx(0.60, abs=1e-3)
    assert hw.max() == pytest.approx(0.60, abs=1e-3)
    assert c['L'] == pytest.approx(3.0)
    assert c['defn']['w0'] == pytest.approx(0.60)
    assert c['defn']['w1'] == pytest.approx(0.60)


def test_the_old_literals_still_build_the_old_funnel():
    """The parameters reach the geometry: the pre-campaign values give the old shape."""
    c = _goal_distance_corridor(0.4333, 0.7667, 3.0)
    hw = np.asarray(c['halfWidth'])
    assert hw[0] == pytest.approx(0.4333, abs=1e-3)
    assert hw[-1] == pytest.approx(0.7667, abs=1e-3)
