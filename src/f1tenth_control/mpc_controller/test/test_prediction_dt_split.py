"""Control period vs prediction step (M02 campaign fix A1).

The two were one literal (self.ts = 0.1). They are now
mpc_control_period_sec (0.1, the loop) and mpc_prediction_dt_sec (0.2, the
model step), so the warm start must shift by a FRACTION of a stage and the
first rate row must span one control period, not one stage.
"""

import math

import numpy as np
import pytest

from f1tenth_params.param_defaults import get_value
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta

import test_object_approach_closed_loop as rig


def _old_shift(zopt, horizon):
    """The pre-split shift_warm_start body, verbatim, as the reference."""
    z = np.asarray(zopt, dtype=float).ravel()
    shifted = np.empty_like(z)
    shifted[: 2 * (horizon - 1)] = z[2:]
    shifted[2 * (horizon - 1):] = z[2 * (horizon - 1):]
    return shifted


def _plan(horizon):
    k = np.arange(horizon, dtype=float)
    return np.column_stack([0.01 * k, 0.1 * (k + 1)]).ravel()


class TestTheShippedHorizon:

    def test_the_values(self):
        assert get_value('mpc_control_period_sec') == pytest.approx(0.1)
        assert get_value('mpc_prediction_dt_sec') == pytest.approx(0.2)
        assert get_value('mpc_horizon_steps') == 20

    def test_the_horizon_reaches_past_1_5_m_at_the_operating_floor(self):
        horizon_s = get_value('mpc_horizon_steps') * get_value('mpc_prediction_dt_sec')
        reach = horizon_s * get_value('min_moving_speed_mps')
        assert horizon_s == pytest.approx(4.0)
        assert reach == pytest.approx(1.6)
        assert reach >= 1.5

    def test_the_lookahead_sits_outside_the_reach_floor(self):
        """corr_lookahead_frac * L must clear 1.25 x the reach, or the floor
        takes over and logs a geometry warning every 5 s. L is MPC_corr's
        corr_L_base (3.0)."""
        reach = (get_value('mpc_horizon_steps') * get_value('mpc_prediction_dt_sec')
                 * 0.5)
        assert get_value('corr_lookahead_frac') * 3.0 >= 1.25 * reach


class TestShiftWarmStart:

    @pytest.mark.parametrize('horizon', [2, 7, 20])
    def test_an_integer_shift_is_the_old_shift_exactly(self, horizon):
        z = _plan(horizon)
        assert np.array_equal(shift_warm_start(z, horizon), _old_shift(z, horizon))
        assert np.array_equal(shift_warm_start(z, horizon, shift=1.0),
                              _old_shift(z, horizon))

    def test_a_half_stage_shift_interpolates_between_neighbours(self):
        horizon = 5
        z = _plan(horizon)
        u = z.reshape(horizon, 2)
        out = shift_warm_start(z, horizon, shift=0.5).reshape(horizon, 2)
        for k in range(horizon - 1):
            assert out[k] == pytest.approx(0.5 * (u[k] + u[k + 1]))
        # Past the end the last control is held, as the integer shift does.
        assert out[-1] == pytest.approx(u[-1])

    def test_the_shipped_ratio_is_half_a_stage(self):
        shift = get_value('mpc_control_period_sec') / get_value('mpc_prediction_dt_sec')
        assert shift == pytest.approx(0.5)

    def test_a_zero_shift_returns_the_plan_unchanged(self):
        z = _plan(6)
        assert np.array_equal(shift_warm_start(z, 6, shift=0.0), z)

    def test_a_shift_beyond_one_stage(self):
        horizon = 6
        z = _plan(horizon)
        u = z.reshape(horizon, 2)
        out = shift_warm_start(z, horizon, shift=1.25).reshape(horizon, 2)
        assert out[0] == pytest.approx(0.75 * u[1] + 0.25 * u[2])
        assert out[-1] == pytest.approx(u[-1])

    @pytest.mark.parametrize('bad', [-0.1, math.nan, math.inf])
    def test_a_meaningless_shift_falls_back_to_the_tile(self, bad):
        assert shift_warm_start(_plan(4), 4, shift=bad) is None

    def test_the_input_is_not_modified(self):
        z = _plan(4)
        before = z.copy()
        shift_warm_start(z, 4, shift=0.5)
        assert np.array_equal(z, before)


def _straight_corridor(length=3.0, half_width=0.6, n=120):
    s = np.linspace(0.0, length, n)
    zeros = np.zeros(n)
    return {
        'xc': s, 'yc': zeros.copy(),
        'xL': s, 'yL': zeros + half_width, 'xR': s, 'yR': zeros - half_width,
        'tx': np.ones(n), 'ty': zeros.copy(), 'nx': zeros.copy(), 'ny': np.ones(n),
        'halfWidth': zeros + half_width,
        'psiRef': 0.0, 'psiRefTurn': None, 'L': length,
        'Pend': np.array([length, 0.0]),
        'obstacles_world': [], 'car_radius': rig.CAR_RADIUS,
        'avoidance_margin': rig.AVOIDANCE_MARGIN,
    }


def _solve(last_u, pref_nom, ts, control_period, x0=(0.0, 0.0, 0.0, 0.5)):
    # A cheap steering rate, so the solve wants to steer as fast as the rate
    # rows let it and the rows are what the test sees. The tests start the
    # car pointing 0.4 rad right of the corridor, so it wants hard left.
    weights = dict(rig.WEIGHTS, w_du_delta=0.01, w_psi=0.0, w_psi_stage=0.0)
    return solve_mpc_step(
        x0=np.asarray(x0, dtype=float), last_u=np.asarray(last_u, dtype=float),
        pref_nom=np.asarray(pref_nom, dtype=float), corridor=_straight_corridor(),
        horizon=rig.HORIZON, ts=ts, params=rig.PARAMS, limits=rig.LIMITS,
        weights=weights, obstacles=[], dmin=rig.DMIN, vdes=0.5,
        solver='rti', control_period=control_period)


class TestTheFirstRateRowSpansOneControlPeriod:
    """u_0 against the command already on the car has one PERIOD to move."""

    def test_the_first_steering_step_is_limited_by_the_period(self):
        u0, info = _solve([0.0, 0.0], [2.0, 0.0], ts=0.2, control_period=0.1,
                          x0=(0.0, 0.0, -0.4, 0.5))
        assert info['success']
        assert abs(u0[0]) <= rig.LIMITS['dDeltaMax'] * 0.1 + 1e-3  # OSQP eps
        # The solve really is pressing on that row ...
        assert abs(u0[0]) == pytest.approx(rig.LIMITS['dDeltaMax'] * 0.1, abs=1e-3)
        z = np.asarray(info['zopt']).reshape(rig.HORIZON, 2)
        # ... and stage to stage the rows span a full prediction step.
        assert np.max(np.abs(np.diff(z[:, 0]))) <= rig.LIMITS['dDeltaMax'] * 0.2 + 1e-3

    def test_a_full_stage_period_allows_the_full_stage_step(self):
        u0, info = _solve([0.0, 0.0], [2.0, 0.0], ts=0.2, control_period=0.2,
                          x0=(0.0, 0.0, -0.4, 0.5))
        assert info['success']
        assert abs(u0[0]) > rig.LIMITS['dDeltaMax'] * 0.1 + 1e-3

    def test_none_keeps_the_single_step_behaviour(self):
        u_a, _ = _solve([0.0, 0.0], [1.2, 0.55], ts=0.1, control_period=None)
        u_b, _ = _solve([0.0, 0.0], [1.2, 0.55], ts=0.1, control_period=0.1)
        assert u_a == pytest.approx(u_b, abs=1e-9)


def _closed_loop_end_state(period, dt, lookahead):
    x = np.array([0.0, 0.15, 0.0, 0.0])
    last_u = np.zeros(2)
    warm = None
    for _ in range(80):
        corridor = _straight_corridor()
        for key in ('xc', 'xL', 'xR'):
            corridor[key] = corridor[key] + x[0]
        u0, info = solve_mpc_step(
            x0=x, last_u=last_u, pref_nom=np.array([x[0] + lookahead, 0.0]),
            corridor=corridor, horizon=rig.HORIZON, ts=dt, params=rig.PARAMS,
            limits=rig.LIMITS, weights=dict(rig.WEIGHTS), obstacles=[],
            dmin=rig.DMIN, vdes=0.4, solver='rti', warm_start_z=warm,
            control_period=period)
        assert info['success']
        warm = shift_warm_start(info['zopt'], rig.HORIZON, shift=period / dt)
        x = np.array(f1tenth_state_fcn_dt_beta(x, u0, period, rig.WHEELBASE, rig.LR))
        last_u = np.asarray(u0, dtype=float)
    return x


def test_a_closed_loop_straight_run_at_0_2_over_0_1_matches_the_old_loop():
    """The split, end to end: 0.1 s plant steps, 0.2 s stages, half shift.

    Same steady state as the old 0.1/0.1 loop, lookahead scaled with the
    reach (1.5 m -> 2.55 m). Both settle ABOVE vdes (~0.6 m/s at vdes 0.4):
    the terminal position cost on a lookahead past the reach pulls speed up,
    and on the car the /drive clamp (max_forward_speed_mps 0.5) cuts it.
    That is pre-existing behaviour, not something the split introduced.
    """
    old = _closed_loop_end_state(0.1, 0.1, 1.5)
    new = _closed_loop_end_state(0.1, 0.2, 2.55)
    assert abs(new[1]) < 0.05
    assert new[3] == pytest.approx(old[3], abs=0.05)
