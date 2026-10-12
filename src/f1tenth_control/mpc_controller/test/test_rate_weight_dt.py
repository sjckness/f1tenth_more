"""Per-step rate weights follow the step they span (A1 review, 2026-10-12).

w_du_delta penalises (delta_k - delta_{k-1})^2 PER STEP. The set was tuned at
a 0.1 s step (sigma = dDeltaMax * ts); at the 0.2 s prediction step the same
steering RATE is twice the change and four times the penalty. The solver
scales the weight by (0.1 / step)^2: stage-to-stage terms x0.25 (3.0 -> 0.75
effective), the first term (u_0 vs the command on the car, one 0.1 s control
period) x1.
"""

import numpy as np
import pytest

from f1tenth_params.param_defaults import get_value
from mpc_controller.mpc_solver import (
    RATE_WEIGHT_REF_DT, STAGE_WEIGHT_REF_HORIZON, planner_cost_corridor,
    rate_weight_factor, solve_mpc_step)

import test_object_approach_closed_loop as rig
from test_prediction_dt_split import _straight_corridor


def test_the_factor():
    assert RATE_WEIGHT_REF_DT == pytest.approx(0.1)
    assert rate_weight_factor(0.1) == pytest.approx(1.0)
    assert rate_weight_factor(0.2) == pytest.approx(0.25)
    assert rate_weight_factor(0.0) == 1.0


def test_the_shipped_effective_values():
    literal = get_value('mpc_w_du_delta')
    assert literal == pytest.approx(8.5714)
    per_stage = literal * STAGE_WEIGHT_REF_HORIZON / get_value('mpc_horizon_steps')
    assert per_stage == pytest.approx(3.0, abs=1e-3)
    dt = get_value('mpc_prediction_dt_sec')
    period = get_value('mpc_control_period_sec')
    assert per_stage * rate_weight_factor(dt) == pytest.approx(0.75, abs=1e-3)
    assert per_stage * rate_weight_factor(period) == pytest.approx(3.0, abs=1e-3)


def test_the_same_steering_rate_costs_the_same_per_second_squared():
    """A constant 0.25 rad/s ramp: at dt 0.2 the raw per-step cost is 4x the
    dt 0.1 one; with the factor the per-step costs agree."""
    horizon = 10
    weights = {k: 0.0 for k in ('w_v', 'w_obs', 'w_corr', 'w_psi_stage', 'w_delta0',
                                'w_u_a', 'w_du_a', 'w_term', 'w_psi')}
    weights['w_du_delta'] = 1.0
    costs = {}
    for dt in (0.1, 0.2):
        steer = 0.25 * dt * np.arange(1, horizon + 1)
        z = np.column_stack([steer, np.zeros(horizon)]).ravel()
        costs[dt] = planner_cost_corridor(
            z=z, x0=np.zeros(4), last_u=np.zeros(2), pref_nom=np.zeros(2),
            corridor=_straight_corridor(), horizon=horizon, ts=dt,
            params=rig.PARAMS, weights=weights, vdes=0.0, control_period=dt)
    assert costs[0.2] == pytest.approx(costs[0.1], rel=1e-6)


def _ramp(control_period, w_du):
    """First-tick steering of a solve that wants hard left (heading -0.4 rad)."""
    weights = dict(rig.WEIGHTS, w_du_delta=w_du, w_psi=4.5)
    corridor = _straight_corridor()
    _, info = solve_mpc_step(
        x0=np.array([0.0, 0.0, -0.4, 0.4]), last_u=np.zeros(2),
        pref_nom=np.array([2.55, 0.0]), corridor=corridor, horizon=rig.HORIZON,
        ts=0.2, params=rig.PARAMS, limits=rig.LIMITS, weights=weights,
        obstacles=[], dmin=rig.DMIN, vdes=0.4, solver='rti',
        control_period=control_period)
    return np.asarray(info['zopt']).reshape(rig.HORIZON, 2)[:, 0]


def test_the_planned_steering_ramp_is_no_longer_held_back_by_the_step_change():
    """At the shipped 8.5714 the plan's stage-to-stage steering steps get
    bigger once the weight is corrected -- the hard 0.5 rad/s x 0.2 s row,
    not the 4x-stiff soft cost, limits the ramp."""
    plan = _ramp(0.1, 8.5714)
    steps = np.diff(plan)
    assert np.max(np.abs(steps)) <= rig.LIMITS['dDeltaMax'] * 0.2 + 1e-3
    assert np.max(np.abs(steps)) > rig.LIMITS['dDeltaMax'] * 0.1
