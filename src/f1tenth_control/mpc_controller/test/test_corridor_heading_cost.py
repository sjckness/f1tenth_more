"""
Coverage for the corridor heading-error cost (weights["w_psi"]).

Audited rather than assumed while fixing the straight-move drift-following
bug: the term already exists and is already wired in BOTH backends -- as a
terminal-yaw cost against corridor["psiRef"] in _solve_rti's QP
(mpc_solver.py) and in planner_cost_corridor's objective -- so nothing was
added here. What was missing was any test of it: test_solver_rti.py mirrors
MPC_corr.py's live weights, which is the right call for what that file
tests, but it means every scenario there runs with w_psi carried in at 0.0
and the term switched off. A regression that silently unwired it (its own
history: it spent a while as a commented-out line plus a `0 * 5.0` weight,
inert twice over) would not have failed a single test.

These are synthetic-input tests, not vehicle behaviour: non-zero heading
error must produce non-zero cost and a correcting steering response, zero
error must produce neither, and the term must key off psiRef -- which is
what makes the frozen straight reference (see
test_corridor_direction_recovery.py) actually pull on the solver.

Run standalone: python3 -m pytest test/test_corridor_heading_cost.py -v
"""

import math

import numpy as np

import pytest

from mpc_controller.mpc_solver import (
    OSQP_AVAILABLE,
    f1tenth_state_fcn_dt_beta,
    planner_cost_corridor,
    solve_mpc_step,
)

WHEELBASE = 0.305
LR = 0.17
TS = 0.1
HORIZON = 7
PARAMS = {"L": WHEELBASE, "lr": LR}

LIMITS = {
    "delta_min": -1.05,
    "delta_max": 1.05,
    "a_min": -2.0,
    "a_max": 3.0,
    "dDeltaMin": -0.5,
    "dDeltaMax": 0.5,
    "dAMin": -2.0,
    "dAMax": 2.0,
    "vMin": -1.0,
    "vMax": 3.0,
}

# MPC_corr.py's live weights, same choice test_solver_rti.py makes -- w_psi
# is overridden per-test below, which is the whole point of this file.
BASE_WEIGHTS = {
    "w_term": 3.0,
    "w_v": 8.0,
    "w_psi": 0.0,
    "w_u_a": 0.0,
    "w_du_delta": 15.0,
    "w_du_a": 0.0,
    "w_delta0": 0.0,
    "w_obs": 8.0,
    "w_corr": 0.0,
}


def _weights(**overrides):
    w = dict(BASE_WEIGHTS)
    w.update(overrides)
    return w


def _frozen_corridor(psi_ref, length=3.0, wmin=1.3, wmax=2.3, n=200):
    """
    Build a straight corridor along psi=0 through the origin, with a free psiRef.

    Letting psiRef differ from the centerline's own direction is deliberate:
    it decouples the heading term from every other term, so a difference
    between two solves can only have come from w_psi. The shape matches what
    mpc_solver actually reads (xc/yc/nx/ny/halfWidth/psiRef), the same
    minimal stand-in test_solver_rti.py's own _straight_corridor uses.
    """
    u = np.linspace(0.0, 1.0, n)
    s = length * u
    xc = s
    yc = np.zeros(n)
    nx = np.zeros(n)
    ny = np.ones(n)
    return {
        "xc": xc, "yc": yc,
        "xL": xc, "yL": yc + wmin,
        "xR": xc, "yR": yc - wmin,
        "tx": np.ones(n), "ty": np.zeros(n),
        "nx": nx, "ny": ny,
        "halfWidth": wmin + (wmax - wmin) * u,
        "psiRef": float(psi_ref), "t": 0.0,
        "Pend": np.array([xc[-1], yc[-1]]),
        "dFront": 10.0, "dpsi": 0.0,
        "obstacles_world": [],
        "car_radius": 0.20,
        "avoidance_margin": 0.12,
        "d_safe": 0.9,
    }


def _terminal_yaw(x0, z, horizon=HORIZON):
    """
    Roll the true model forward exactly as planner_cost_corridor does.

    Lets a test name the exact heading error the terminal term is being
    charged for, rather than inferring it.
    """
    x = np.asarray(x0, dtype=float).copy()
    for k in range(horizon):
        x = np.asarray(
            f1tenth_state_fcn_dt_beta(x, z[2 * k:2 * k + 2], TS, WHEELBASE, LR),
            dtype=float)
    return float(x[2])


def _cost(psi_ref, w_psi, x0, z):
    return planner_cost_corridor(
        z=z, x0=np.asarray(x0, dtype=float), last_u=np.zeros(2),
        pref_nom=np.array([3.0, 0.0]), corridor=_frozen_corridor(psi_ref),
        horizon=HORIZON, ts=TS, params=PARAMS, weights=_weights(w_psi=w_psi),
        vdes=0.5)


class TestPlannerCostHeadingTerm:
    """The objective form (SLSQP backend, and the true_cost RTI reports)."""

    def test_zero_heading_error_costs_nothing(self):
        x0 = [0.0, 0.0, 0.0, 0.5]
        z = np.zeros(2 * HORIZON)
        # Straight-ahead controls from psi=0: terminal yaw stays 0, and
        # psiRef is 0 -- so the term must contribute exactly nothing.
        assert math.isclose(_terminal_yaw(x0, z), 0.0, abs_tol=1e-12)
        assert math.isclose(_cost(0.0, 5.0, x0, z), _cost(0.0, 0.0, x0, z),
                            rel_tol=0.0, abs_tol=1e-12)

    def test_nonzero_heading_error_costs_w_psi_times_error_squared(self):
        x0 = [0.0, 0.0, 0.3, 0.5]
        z = np.zeros(2 * HORIZON)
        psi_ref = -0.2
        err = _terminal_yaw(x0, z) - psi_ref
        assert abs(err) > 1e-6
        added = _cost(psi_ref, 5.0, x0, z) - _cost(psi_ref, 0.0, x0, z)
        assert added > 0.0
        assert math.isclose(added, 5.0 * err ** 2, rel_tol=1e-9)

    def test_cost_scales_linearly_with_the_weight(self):
        x0 = [0.0, 0.0, 0.3, 0.5]
        z = np.zeros(2 * HORIZON)
        base = _cost(-0.2, 0.0, x0, z)
        one = _cost(-0.2, 1.0, x0, z) - base
        four = _cost(-0.2, 4.0, x0, z) - base
        assert math.isclose(four, 4.0 * one, rel_tol=1e-9)

    def test_error_is_wrapped_not_taken_raw(self):
        """
        Confirm the error is wrapped, not taken raw.

        psiRef just below +pi against a car just above -pi is a small error,
        not a ~2pi one -- the car is already pointing the right way.
        """
        x0 = [0.0, 0.0, -math.pi + 0.05, 0.5]
        z = np.zeros(2 * HORIZON)
        added = _cost(math.pi - 0.05, 5.0, x0, z) - _cost(math.pi - 0.05, 0.0, x0, z)
        assert added < 5.0 * (0.2 ** 2)


@pytest.mark.skipif(not OSQP_AVAILABLE, reason="osqp not importable")
class TestRtiQpHeadingTerm:
    """The QP the deployed backend actually solves."""

    @staticmethod
    def _steer(psi_ref, w_psi):
        u0, _info = solve_mpc_step(
            x0=np.array([0.0, 0.0, 0.0, 0.5]),
            last_u=np.zeros(2),
            pref_nom=np.array([3.0, 0.0]),
            corridor=_frozen_corridor(psi_ref),
            horizon=HORIZON, ts=TS, params=PARAMS, limits=LIMITS,
            weights=_weights(w_psi=w_psi), obstacles=[], dmin=0.9, vdes=0.5,
            solver='rti')
        return float(u0[0])

    def test_zero_heading_error_steers_straight(self):
        assert abs(self._steer(0.0, 5.0)) < 1e-6

    def test_heading_error_steers_toward_the_reference(self):
        """
        Confirm a reference rotated left of the car produces a left steer.

        And mirrored for the right. That gradient is what the frozen straight
        corridor relies on to pull a drifted car's yaw back to the direction
        its move started in.
        """
        left = self._steer(+0.3, 5.0)
        right = self._steer(-0.3, 5.0)
        assert left > 1e-4
        assert right < -1e-4
        assert math.isclose(left, -right, rel_tol=1e-6)

    def test_term_is_inert_when_its_weight_is_zero(self):
        """
        Guard the wiring in both directions.

        With w_psi off, psiRef must make no difference at all to the QP's
        answer.
        """
        off = self._steer(0.0, 0.0)
        assert math.isclose(self._steer(+0.3, 0.0), off, abs_tol=1e-9)
        assert math.isclose(self._steer(-0.3, 0.0), off, abs_tol=1e-9)


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
