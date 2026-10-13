"""The corridor funnel's shipped geometry (2026-10-12).

corr_wmin / corr_wmax / corr_L_base were bare literals in MPC_corr.py
(0.4333 / 0.7667 / 3.0, and 1.5 live on the Orin from an uncommitted edit).
The M02 campaign fix A2 made them ROS parameters and shipped 0.60 / 0.60, a
rectangle. The funnel is back: the far end is corr_wmin x corr_width_ratio, a
FIXED ratio (2.3 / 1.3, the original funnel's shape), and corr_wmin 0.65 is
sized so a centred person stays passable with the perception tolerance on top.
stack_params.yaml carries the sizing; these tests recompute it from the same
keys, so retuning any of them past the design case fails here.
"""

import math

import campaign_rig as cr
from f1tenth_description.vehicle_frames import base_front_x
from f1tenth_params.corridor_geometry import funnel_half_widths
from f1tenth_params.object_geometry import NOMINAL_FOOTPRINT_WIDTH_M
from f1tenth_params.param_defaults import get_value
from mpc_controller.MPC_corr import MPCController
import numpy as np
import pytest
import test_object_approach_closed_loop as rig

ORIGINAL_FUNNEL_RATIO = 2.3 / 1.3
PERSON_R = NOMINAL_FOOTPRINT_WIDTH_M['person'] / 2.0
# Design case: the person is this far ahead of the car's FRONT when the
# corridor is built.
PERSON_AHEAD_OF_FRONT_M = 1.0


def _shipped_half_widths():
    return funnel_half_widths(get_value('corr_wmin'), get_value('corr_width_ratio'))


def _goal_distance_corridor(wmin, wmax, L, w0_override=None):
    fake = rig._ObjectMPC(standoff=1.0, speed=0.4)
    fake.corr_wmin = wmin
    fake.corr_wmax = wmax
    fake.corr_L_base = L
    fake.goal_distance = 5.0
    fake.goal_start_xy = (0.0, 0.0)
    fake.goal_anchor_odom = (0.0, 0.0, 0.0)
    fake.psi_init_corridor = 0.0
    if w0_override is not None:
        fake._corridor_w0 = w0_override
    return MPCController.build_straight_corridor(fake, np.zeros(4))


def _axle_offset_to_pass_a_centred_person():
    """r_person + tolerance + car_radius + margin: where the rear axle passes."""
    return (PERSON_R + get_value('corr_perception_lateral_tolerance_m')
            + get_value('car_radius') + get_value('obstacle_safety_margin_m'))


class TestTheShippedGeometry:

    def test_the_values(self):
        assert get_value('corr_wmin') == pytest.approx(0.65)
        assert get_value('corr_width_ratio') == pytest.approx(ORIGINAL_FUNNEL_RATIO)
        assert get_value('corr_L_base') == pytest.approx(3.0)

    def test_the_far_end_is_the_start_times_the_fixed_ratio(self):
        w0, w1 = _shipped_half_widths()
        assert w1 / w0 == pytest.approx(ORIGINAL_FUNNEL_RATIO)
        assert w1 == pytest.approx(1.15)

    def test_a_centred_person_with_the_tolerance_fits_at_the_corridor_start(self):
        """
        Size the start for the axle.

        The rows bound the rear axle, and the car meets an obstacle in the
        first metre of a corridor (rebuilt from its pose every period).
        """
        need = _axle_offset_to_pass_a_centred_person()
        assert need == pytest.approx(0.65)
        assert get_value('corr_wmin') >= need - 1e-9

    def test_the_corridor_is_at_least_the_horizon_reach_long(self):
        reach = (get_value('mpc_horizon_steps') * get_value('mpc_prediction_dt_sec')
                 * 0.5)
        assert get_value('corr_L_base') >= reach
        assert (get_value('corr_lookahead_frac') * get_value('corr_L_base')
                >= 1.25 * reach)

    def test_the_adaptive_corridor_is_off_by_default(self):
        assert get_value('corridor_adaptive_enabled') is False


def test_the_built_default_corridor_keeps_the_fixed_end_to_start_ratio():
    w0, w1 = _shipped_half_widths()
    c = _goal_distance_corridor(w0, w1, get_value('corr_L_base'))
    hw = np.asarray(c['halfWidth'])
    assert hw[0] == pytest.approx(w0, abs=1e-3)
    assert hw[-1] == pytest.approx(w1, abs=1e-3)
    assert hw[-1] / hw[0] == pytest.approx(ORIGINAL_FUNNEL_RATIO, rel=1e-3)
    assert c['defn']['w1'] / c['defn']['w0'] == pytest.approx(ORIGINAL_FUNNEL_RATIO)
    assert c['defn']['w0_base'] == pytest.approx(w0)
    assert c['L'] == pytest.approx(3.0)


def test_the_built_default_corridor_is_a_funnel_not_a_rectangle():
    w0, w1 = _shipped_half_widths()
    hw = np.asarray(_goal_distance_corridor(w0, w1, 3.0)['halfWidth'])
    assert np.all(np.diff(hw) >= -1e-9), 'the funnel never narrows ahead'
    assert hw[-1] - hw[0] == pytest.approx(0.5, abs=1e-3)


def test_the_whole_car_stays_inside_the_funnel_at_the_design_persons_station():
    """
    Hold the whole car at the design person's station.

    At the person's station (1 m ahead of the front bumper when the corridor
    is built) the funnel holds the car disk -- axle offset plus car_radius --
    not just the axle.
    """
    w0, w1 = _shipped_half_widths()
    c = _goal_distance_corridor(w0, w1, 3.0)
    station = base_front_x() + PERSON_AHEAD_OF_FRONT_M
    i = int(np.argmin(np.abs(np.asarray(c['xc']) - station)))
    car_disk_edge = _axle_offset_to_pass_a_centred_person() + get_value('car_radius')
    assert float(c['halfWidth'][i]) >= car_disk_edge
    # halfWidth at the nearest centreline index, as the solver's rows read it
    assert float(c['halfWidth'][i]) == pytest.approx(0.884, abs=0.005)


def test_a_wider_adaptive_start_scales_both_ends_and_keeps_the_ratio():
    w0, w1 = _shipped_half_widths()
    c = _goal_distance_corridor(w0, w1, 3.0, w0_override=0.80)
    hw = np.asarray(c['halfWidth'])
    assert hw[0] == pytest.approx(0.80, abs=1e-3)
    assert hw[-1] / hw[0] == pytest.approx(ORIGINAL_FUNNEL_RATIO, rel=1e-3)
    assert c['defn']['w0'] == pytest.approx(0.80)
    assert c['defn']['w0_base'] == pytest.approx(w0)


def test_the_old_literals_still_build_the_old_funnel():
    """The parameters reach the geometry: the pre-campaign values give the old shape."""
    c = _goal_distance_corridor(0.4333, 0.7667, 3.0)
    hw = np.asarray(c['halfWidth'])
    assert hw[0] == pytest.approx(0.4333, abs=1e-3)
    assert hw[-1] == pytest.approx(0.7667, abs=1e-3)


# ---- closed loop on the nominal model (campaign_rig) -----------------------
# Nominal model, perfect obstacles: necessary, not sufficient.

def _run(r, offset, appear, **cfg):
    w0, w1 = _shipped_half_widths()
    return cr.run(cr.Config(a3=True, half_width_start=w0, half_width_end=w1, **cfg),
                  [cr.Obstacle(3.0, offset, r, appear)])


@pytest.mark.parametrize('appear', [math.inf, 2.0])
def test_a_centred_person_is_passed_in_the_default_funnel(appear):
    res = _run(PERSON_R, 0.0, appear)
    assert res.reached_s is not None and res.blocked_s is None
    assert res.min_body_gap > 0.0
    assert res.protrusion == pytest.approx(0.0, abs=1e-6)
    assert all(w == pytest.approx(get_value('corr_wmin')) for w in res.start_half_widths)


@pytest.mark.parametrize('label, r, appear', [
    ('R011 plant centred, seen at 1.4 m', 0.15, 1.4),
    ('1.0 m object centred, seen from the start', 0.50, math.inf),
])
def test_the_funnel_passes_what_the_rectangle_ended_blocked(label, r, appear):
    """
    Pass the two cases the 1.2 m rectangle ended BLOCKED.

    Both end BLOCKED no_feasible_plan in the 1.2 m rectangle
    (test_blocked_closed_loop.py); the funnel's wider far end lets the car
    start moving over early, and every rebuild re-centres the corridor on it.
    """
    res = _run(r, 0.0, appear)
    assert res.reached_s is not None, label
    assert res.min_body_gap > 0.0


def test_the_adaptive_corridor_widens_for_a_wide_centred_object_and_still_passes():
    cap = get_value('corridor_adaptive_max_half_width_m')
    res = _run(0.50, 0.0, math.inf, adaptive=True, adaptive_max_w0=cap,
               lateral_tolerance=get_value('corr_perception_lateral_tolerance_m'))
    assert res.reached_s is not None
    assert max(res.start_half_widths) == pytest.approx(cap)
    assert min(res.start_half_widths) == pytest.approx(get_value('corr_wmin'))
