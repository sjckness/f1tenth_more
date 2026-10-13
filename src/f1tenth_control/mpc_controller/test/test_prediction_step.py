"""
The prediction step is not the control period (mpc_prediction_dt_s).

mpc_corr runs its loop at 10 Hz (MPC_corr.ts = 0.1 s) and, since the 4 s
horizon change, predicts N = 20 steps of mpc_prediction_dt_s = 0.2 s. Every
place the two were the same number had to pick one:

  * the model rollout and the later rate rows    -> prediction step
  * the FIRST rate row (u_0 - last_u)            -> control period: u_0 is
    the next command, published one control period after last_u
  * the warm-start shift                         -> a fraction of a step
  * the lookahead floor (horizon's reach)        -> prediction step

Equal steps reproduce the single-step solver exactly; that is the rollback.

These are synthetic-input tests against the nominal model, not vehicle
behaviour.

Run standalone: python3 -m pytest test/test_prediction_step.py -v
"""

import math
import pathlib
import re

from f1tenth_params.param_defaults import get_value
from mpc_controller.MPC_corr import lookahead_clamp_length, MPCController
from mpc_controller.mpc_solver import (
    OSQP_AVAILABLE,
    planner_cost_corridor,
    planner_nonlcon_corridor,
    rate_weight_factor,
    scale_stage_weights,
    shift_warm_start,
    solve_mpc_step,
)
import numpy as np
import pytest
from test_warm_start import _corridor, LIMITS, PARAMS, WEIGHTS

CONTROL_PERIOD = 0.1
PRED_DT = 0.2
HORIZON = 20
U_START = float(get_value('corr_turn_u_start'))
U_END = float(get_value('corr_turn_u_end'))

needs_osqp = pytest.mark.skipif(not OSQP_AVAILABLE, reason='osqp not installed')


def _solve(x0, last_u, corridor, *, ts, control_period, warm=None, vdes=0.5,
           pref=None):
    return solve_mpc_step(
        x0=np.asarray(x0, dtype=float), last_u=np.asarray(last_u, dtype=float),
        pref_nom=corridor['Pend'] if pref is None else pref, corridor=corridor,
        horizon=HORIZON, ts=ts, params=PARAMS, limits=LIMITS,
        weights=dict(WEIGHTS), obstacles=[], dmin=0.32, vdes=vdes, solver='rti',
        warm_start_z=warm, control_period=control_period)


# ---------------------------------------------------------------------------
# warm start
# ---------------------------------------------------------------------------

class TestFractionalWarmStartShift:

    def _plan(self):
        return np.arange(2 * HORIZON, dtype=float) * 0.01

    def test_a_whole_step_is_the_original_shift(self):
        z = self._plan()
        expected = np.concatenate([z[2:], z[-2:]])
        assert np.array_equal(shift_warm_start(z, HORIZON, step_fraction=1.0), expected)

    def test_the_default_is_a_whole_step(self):
        z = self._plan()
        assert np.array_equal(shift_warm_start(z, HORIZON),
                              shift_warm_start(z, HORIZON, step_fraction=1.0))

    def test_half_a_step_is_the_zoh_average_of_the_two_stages_it_straddles(self):
        z = self._plan()
        out = shift_warm_start(z, HORIZON, step_fraction=0.5).reshape(HORIZON, 2)
        pairs = z.reshape(HORIZON, 2)
        for k in range(HORIZON - 1):
            assert out[k] == pytest.approx(0.5 * (pairs[k] + pairs[k + 1]))
        assert out[-1] == pytest.approx(pairs[-1])

    @pytest.mark.parametrize('fraction', [0.0, -0.5, float('nan'), float('inf')])
    def test_a_fraction_that_is_not_a_positive_time_gives_no_warm_start(self, fraction):
        assert shift_warm_start(self._plan(), HORIZON, step_fraction=fraction) is None


# ---------------------------------------------------------------------------
# rate rows and rate cost
# ---------------------------------------------------------------------------

class TestRateWeightFactor:

    def test_equal_steps_leave_every_stage_untouched(self):
        assert all(rate_weight_factor(k, 0.1, 0.1) == 1.0 for k in range(HORIZON))

    def test_the_first_stage_is_charged_over_the_control_period(self):
        assert rate_weight_factor(0, PRED_DT, CONTROL_PERIOD) == 1.0

    def test_later_stages_are_charged_as_a_rate(self):
        assert rate_weight_factor(1, PRED_DT, CONTROL_PERIOD) == pytest.approx(0.25)


@needs_osqp
class TestFirstRateRowUsesTheControlPeriod:
    """Steering hard over from straight: the rate bound is what binds."""

    def _hard_left(self):
        corridor = _corridor(0.0, math.radians(80.0))
        pref = np.array([0.5, 2.5])
        return corridor, pref

    def test_the_first_step_moves_no_more_than_one_control_period_allows(self):
        corridor, pref = self._hard_left()
        u0, info = _solve([0.0, 0.0, 0.0, 0.5], [0.0, 0.0], corridor,
                          ts=PRED_DT, control_period=CONTROL_PERIOD, pref=pref)
        assert info['success']
        assert abs(u0[0]) <= LIMITS['dDeltaMax'] * CONTROL_PERIOD + 1e-3

    def test_later_steps_may_move_one_prediction_step_worth(self):
        corridor, pref = self._hard_left()
        _u0, info = _solve([0.0, 0.0, 0.0, 0.5], [0.0, 0.0], corridor,
                           ts=PRED_DT, control_period=CONTROL_PERIOD, pref=pref)
        steer = info['zopt'][0::2]
        steps = np.abs(np.diff(steer))
        assert steps.max() <= LIMITS['dDeltaMax'] * PRED_DT + 1e-3
        assert steps.max() > LIMITS['dDeltaMax'] * CONTROL_PERIOD + 1e-3

    def test_no_control_period_is_the_single_step_solver(self):
        corridor = _corridor(0.0, math.radians(30.0))
        x0, last_u = [0.0, 0.05, 0.1, 0.4], [0.02, 0.0]
        _u, a = _solve(x0, last_u, corridor, ts=0.1, control_period=None)
        _u, b = _solve(x0, last_u, corridor, ts=0.1, control_period=0.1)
        assert np.array_equal(a['zopt'], b['zopt'])


class TestSlsqpRowsMatch:

    def test_the_first_rate_constraint_is_over_the_control_period(self):
        z = np.zeros(2 * HORIZON)
        z[0::2] = 0.08          # 0.08 rad in one control period = 0.8 rad/s
        corridor = _corridor(0.0, 0.0)
        kwargs = {'z': z, 'x0': np.array([0.0, 0.0, 0.0, 0.5]), 'last_u': np.zeros(2),
                  'corridor': corridor, 'horizon': HORIZON, 'ts': PRED_DT,
                  'params': PARAMS, 'limits': LIMITS, 'obstacles': [], 'dmin': 0.32}
        c_old = planner_nonlcon_corridor(**kwargs)
        c_new = planner_nonlcon_corridor(control_period=CONTROL_PERIOD, **kwargs)
        assert c_old[0] < 0.0      # 0.4 rad/s over 0.2 s: inside the bound
        assert c_new[0] > 0.0      # 0.8 rad/s over 0.1 s: violates it


@needs_osqp
class TestReportedCostIsTheQpsCost:

    def test_true_cost_uses_the_same_rate_weights_as_the_qp(self):
        corridor = _corridor(0.0, math.radians(30.0))
        x0, last_u = np.array([0.0, 0.0, 0.0, 0.5]), np.array([0.0, 0.0])
        _u0, info = _solve(x0, last_u, corridor, ts=PRED_DT,
                           control_period=CONTROL_PERIOD)
        recomputed = planner_cost_corridor(
            z=info['zopt'], x0=x0, last_u=last_u, pref_nom=corridor['Pend'],
            corridor=corridor, horizon=HORIZON, ts=PRED_DT, params=PARAMS,
            weights=scale_stage_weights(dict(WEIGHTS), HORIZON), vdes=0.5,
            control_period=CONTROL_PERIOD)
        assert info['cost'] == pytest.approx(recomputed)


# ---------------------------------------------------------------------------
# horizon length and closed loop
# ---------------------------------------------------------------------------

@needs_osqp
def test_twenty_steps_of_0p2_s_predict_four_seconds_ahead():
    corridor = _corridor(0.0, 0.0, length=3.0)
    _u0, info = _solve([0.0, 0.0, 0.0, 0.5], [0.0, 0.0], corridor,
                       ts=PRED_DT, control_period=CONTROL_PERIOD)
    assert len(info['x_pred']) == HORIZON
    # 4 s at ~0.5 m/s: the horizon ends ~2 m out, not the 1 m of a 0.1 s step.
    assert info['x_pred'][-1][0] > 1.6


@needs_osqp
def test_closed_loop_60_degree_turn_settles_inside_the_servo_rate():
    """
    Nominal plant, 10 Hz, 6 s of a 60 degree heading change.

    The shipped corridor ramp (corr_turn_u_start/u_end, read from the yaml):
    the 4 s horizon is measurably slower to return the heading on the
    reference 0.10/0.70 ramp, so this pins the shape that ships.
    """
    from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta
    x = np.array([0.0, 0.0, 0.0, 0.5])
    last_u = np.zeros(2)
    warm = None
    worst = 0.0
    corridor = None
    for tick in range(60):
        if tick % 10 == 0:
            corridor = _corridor(float(x[2]), math.radians(60.0),
                                 x0=float(x[0]), y0=float(x[1]),
                                 u_start=U_START, u_end=U_END)
        u0, info = _solve(x, last_u, corridor, ts=PRED_DT,
                          control_period=CONTROL_PERIOD, warm=warm)
        warm = (shift_warm_start(info['zopt'], HORIZON,
                                 step_fraction=CONTROL_PERIOD / PRED_DT)
                if info['success'] else None)
        worst = max(worst, abs(float(u0[0]) - float(last_u[0])) / CONTROL_PERIOD)
        last_u = np.asarray(u0, dtype=float)
        x = np.asarray(f1tenth_state_fcn_dt_beta(
            x, last_u, CONTROL_PERIOD, PARAMS['L'], PARAMS['lr']), dtype=float)
    assert worst <= LIMITS['dDeltaMax'] + 0.02
    assert abs(math.atan2(math.sin(x[2] - math.radians(60.0)),
                          math.cos(x[2] - math.radians(60.0)))) < 0.05


# ---------------------------------------------------------------------------
# the node's side: the floor follows the prediction step
# ---------------------------------------------------------------------------

class _Floor:
    def __init__(self, ts_pred=None):
        self.N, self.ts, self.vdes = HORIZON, CONTROL_PERIOD, 0.5
        self.corr_lookahead_reach_margin = 1.25
        if ts_pred is not None:
            self.ts_pred = ts_pred

    clamp_length = MPCController._lookahead_clamp_length


def test_the_lookahead_floor_is_beyond_the_four_second_reach():
    # 1.25 * 20 * 0.2 * 0.5 = 2.5 m, past the 2.0 m the horizon covers.
    assert _Floor(ts_pred=PRED_DT).clamp_length() == pytest.approx(2.5)


def test_a_stand_in_without_a_prediction_step_keeps_the_control_period_floor():
    assert _Floor().clamp_length() == pytest.approx(1.25)


# ---------------------------------------------------------------------------
# the shipped geometry: the corridor is longer than the horizon reaches
# ---------------------------------------------------------------------------
#
# N, the control period, corr_L_base and the reach margin are literals in
# MPCController.__init__, so they are read from the source as text (the
# test_corridor_sizing.py pattern); the prediction step, the speed cap and the
# rebuild period are read from stack_params.yaml. THESE ENCODE THE SHIPPED
# DEFAULTS: changing mpc_prediction_dt_s, N or corr_L_base must come past here.

_MPC_CORR = (pathlib.Path(__file__).resolve().parents[1]
             / 'mpc_controller' / 'MPC_corr.py')


def _literal(name):
    src = _MPC_CORR.read_text()
    return float(re.search(r'self\.' + name + r'\s*=\s*([0-9.]+)\n', src).group(1))


def _shipped():
    n = int(_literal('N'))
    dt = float(get_value('mpc_prediction_dt_s'))
    v = float(get_value('max_forward_speed_mps'))
    return n, dt, v, _literal('corr_L_base')


def test_the_shipped_horizon_is_twenty_steps_of_0p2_s_on_a_10_hz_loop():
    n, dt, _v, _l = _shipped()
    assert _literal('ts') == pytest.approx(CONTROL_PERIOD)
    assert n == HORIZON
    assert dt == pytest.approx(PRED_DT)
    assert n * dt == pytest.approx(4.0)


def test_the_corridor_is_at_least_as_long_as_the_horizon_distance():
    n, dt, v, corr_l = _shipped()
    horizon_distance = n * dt * v          # 2.0 m at 20 x 0.2 s x 0.5 m/s
    assert horizon_distance <= corr_l, (
        f'the horizon reaches {horizon_distance:.2f} m but a full-length '
        f'corridor is only corr_L_base = {corr_l:.2f} m')


def test_the_corridor_still_covers_the_horizon_just_before_the_next_rebuild():
    """The car drives v * corridor_update_period into the corridor between rebuilds."""
    n, dt, v, corr_l = _shipped()
    travelled = v * float(get_value('corridor_update_period'))
    assert n * dt * v + travelled <= corr_l + 1e-9


def test_the_lookahead_floor_stays_inside_a_full_length_corridor():
    """A floor at or past corr_L_base would pin the target to the corridor end."""
    n, dt, v, corr_l = _shipped()
    floor = lookahead_clamp_length(n, dt, v, _literal('corr_lookahead_reach_margin'))
    assert floor == pytest.approx(2.5)
    assert floor < corr_l
