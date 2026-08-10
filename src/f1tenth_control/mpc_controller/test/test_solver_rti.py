"""First real logic tests in mpc_controller (previously only the stock ament
linter boilerplate). Added by the MPC optimization pass (Phase B: SLSQP ->
OSQP real-time-iteration solver) -- no prior synthetic-scenario test
infrastructure existed anywhere in this repo/history to extend, so these
scenarios are constructed fresh here, matching the plan.

Two things are checked:
  1. vehicle_model.py's analytic dynamics Jacobians (f1tenth_state_fcn_dt_
     beta_jacobian) against scipy.optimize.approx_fprime finite differences,
     at a handful of (x, u) points spanning the steering/speed range.
  2. mpc_solver.py's two solve paths (_solve_rti / _solve_slsqp, both behind
     solve_mpc_step) agree "same ballpark" -- not bit-exact, since RTI is a
     single-linearization approximation of the same nonlinear problem SLSQP
     solves from scratch every tick -- across a handful of synthetic
     corridor+obstacle scenarios.

Run standalone: python3 -m pytest test/test_solver_rti.py -v
"""

import numpy as np
import pytest
from scipy.optimize import approx_fprime

from mpc_controller.mpc_solver import solve_mpc_step
from mpc_controller.vehicle_model import (
    f1tenth_state_fcn_dt_beta,
    f1tenth_state_fcn_dt_beta_jacobian,
)

# Same values MPC_corr.py uses (self.params / self.ts / self.N).
WHEELBASE = 0.305
LR = 0.17
TS = 0.1
HORIZON = 7

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

# Same weights MPC_corr.py uses today (several deliberately zeroed -- kept
# identical here rather than picking "nicer" test weights, so this exercises
# the actual live cost shape, not a friendlier stand-in).
WEIGHTS = {
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

PARAMS = {"L": WHEELBASE, "lr": LR}


# ============================================================
# 1. Jacobian correctness
# ============================================================

def _numeric_state_jacobians(xk, uk, ts=TS, wheelbase=WHEELBASE, lr=LR, eps=1e-6):
    """Finite-difference (A, B) via scipy.optimize.approx_fprime, one output
    row (of the 4-dim next-state vector) at a time -- independent of, and a
    cross-check on, vehicle_model.py's closed-form Jacobian.
    """
    xk = np.asarray(xk, dtype=float)
    uk = np.asarray(uk, dtype=float)

    def f_x(x, i):
        return f1tenth_state_fcn_dt_beta(x, uk, ts, wheelbase, lr)[i]

    def f_u(u, i):
        return f1tenth_state_fcn_dt_beta(xk, u, ts, wheelbase, lr)[i]

    A_num = np.array([approx_fprime(xk, f_x, eps, i) for i in range(4)])
    B_num = np.array([approx_fprime(uk, f_u, eps, i) for i in range(4)])
    return A_num, B_num


# (xk, uk) points spanning the steering/speed range: standstill, moderate
# speed + moderate left steer, high speed straight, negative position/heading
# + hard right steer + strong braking.
JACOBIAN_TEST_POINTS = [
    ([0.0, 0.0, 0.0, 0.0], [0.0, 0.0]),
    ([1.0, 0.5, 0.3, 1.0], [0.2, 0.5]),
    ([0.0, 0.0, 0.0, 2.5], [-0.8, -1.0]),
    ([2.0, -1.0, -0.5, 0.5], [0.9, 2.0]),
]


@pytest.mark.parametrize("xk, uk", JACOBIAN_TEST_POINTS)
def test_jacobian_matches_finite_difference(xk, uk):
    A, B = f1tenth_state_fcn_dt_beta_jacobian(xk, uk, TS, WHEELBASE, LR)
    A_num, B_num = _numeric_state_jacobians(xk, uk)

    np.testing.assert_allclose(A, A_num, atol=1e-4, err_msg=f"A mismatch at xk={xk} uk={uk}")
    np.testing.assert_allclose(B, B_num, atol=1e-4, err_msg=f"B mismatch at xk={xk} uk={uk}")


# ============================================================
# 2. RTI vs. SLSQP, same synthetic scenarios
# ============================================================

def _straight_corridor(x0, length=3.0, wmin=1.3, wmax=2.3, n=200):
    """Minimal standalone stand-in for MPC_corr.py's build_straight_corridor:
    a straight corridor along x0's heading, centerline + linearly-widening
    halfWidth. Not the full Bezier-edge construction that file does (xL/yL/
    xR/yR here are a simple offset, not curved) -- mpc_solver.py's actual
    solve paths only read xc/yc/nx/ny/halfWidth, so that's all this needs to
    reproduce faithfully.
    """
    X0, Y0, psi0 = float(x0[0]), float(x0[1]), float(x0[2])
    u = np.linspace(0.0, 1.0, n)
    s = length * u
    xc = X0 + s * np.cos(psi0)
    yc = Y0 + s * np.sin(psi0)
    half_width = wmin + (wmax - wmin) * u
    nx = np.full(n, -np.sin(psi0))
    ny = np.full(n, np.cos(psi0))
    return {
        "xc": xc, "yc": yc,
        "xL": xc + wmin * nx, "yL": yc + wmin * ny,
        "xR": xc - wmin * nx, "yR": yc - wmin * ny,
        "tx": np.full(n, np.cos(psi0)), "ty": np.full(n, np.sin(psi0)),
        "nx": nx, "ny": ny,
        "halfWidth": half_width,
        "psiRef": psi0, "t": 0.0,
        "Pend": np.array([xc[-1], yc[-1]]),
        "dFront": 10.0, "dpsi": 0.0,
    }


def _with_obstacles(corridor, obstacles, car_radius=0.20, avoidance_margin=0.12, d_safe=0.9):
    corridor = dict(corridor)
    corridor["obstacles_world"] = obstacles
    corridor["car_radius"] = car_radius
    corridor["avoidance_margin"] = avoidance_margin
    corridor["d_safe"] = d_safe
    return corridor


def _scenario_straight_no_obstacle():
    x0 = np.array([0.0, 0.0, 0.0, 0.3])
    corridor = _with_obstacles(_straight_corridor(x0), [])
    return x0, corridor, corridor["Pend"], 0.5


def _scenario_obstacle_dead_ahead():
    x0 = np.array([0.0, 0.0, 0.0, 0.3])
    corridor = _with_obstacles(_straight_corridor(x0), [(1.5, 0.0, 0.2)])
    return x0, corridor, corridor["Pend"], 0.5


def _scenario_obstacle_near_edge():
    x0 = np.array([0.0, 0.0, 0.0, 0.3])
    # halfWidth at s~1.5m (u~0.5) is wmin + 0.5*(wmax-wmin) = 1.3 + 0.5 = 1.8m
    # -- an obstacle at y=1.5 sits close to that edge, not dead-center.
    corridor = _with_obstacles(_straight_corridor(x0), [(1.5, 1.5, 0.2)])
    return x0, corridor, corridor["Pend"], 0.5


def _scenario_tight_corridor_near_limit_speed():
    x0 = np.array([0.0, 0.0, 0.0, 2.8])  # vMax is 3.0 -- near the limit
    corridor = _with_obstacles(_straight_corridor(x0, wmin=0.4, wmax=0.5), [])
    return x0, corridor, corridor["Pend"], 2.8


SCENARIOS = {
    "straight_no_obstacle": _scenario_straight_no_obstacle,
    "obstacle_dead_ahead": _scenario_obstacle_dead_ahead,
    "obstacle_near_edge": _scenario_obstacle_near_edge,
    "tight_corridor_near_limit_speed": _scenario_tight_corridor_near_limit_speed,
}


@pytest.mark.parametrize("scenario_name", list(SCENARIOS.keys()))
def test_rti_matches_slsqp_same_ballpark(scenario_name):
    x0, corridor, pref_nom, vdes = SCENARIOS[scenario_name]()
    last_u = np.array([0.0, 0.0])
    common_kwargs = dict(
        x0=x0,
        last_u=last_u,
        pref_nom=pref_nom,
        corridor=corridor,
        horizon=HORIZON,
        ts=TS,
        params=PARAMS,
        limits=LIMITS,
        weights=WEIGHTS,
        obstacles=corridor["obstacles_world"],
        dmin=corridor["d_safe"],
        vdes=vdes,
    )

    u0_slsqp, info_slsqp = solve_mpc_step(solver='slsqp', **common_kwargs)
    u0_rti, info_rti = solve_mpc_step(solver='rti', **common_kwargs)

    assert info_slsqp["success"], f"{scenario_name}: SLSQP baseline itself failed to converge"
    # Per the approved MPC-optimization-pass decision, RTI's result is used
    # regardless of solve status -- only require a finite, usable control here.
    assert np.all(np.isfinite(u0_rti)), f"{scenario_name}: RTI produced a non-finite control"

    # "Same ballpark", not bit-exact -- generous absolute tolerances (see
    # module docstring): RTI is a single linearize-once-per-tick
    # approximation of the same nonlinear problem SLSQP re-solves from
    # scratch. delta lives in [-1.05, 1.05] rad, a in [-2, 3] m/s^2.
    assert u0_rti[0] == pytest.approx(u0_slsqp[0], abs=0.35), (
        f"{scenario_name}: steering diverged too far -- "
        f"slsqp={u0_slsqp} rti={u0_rti}")
    assert u0_rti[1] == pytest.approx(u0_slsqp[1], abs=1.0), (
        f"{scenario_name}: acceleration diverged too far -- "
        f"slsqp={u0_slsqp} rti={u0_rti}")

    xN_slsqp = info_slsqp["x_pred"][-1][:2]
    xN_rti = info_rti["x_pred"][-1][:2]
    terminal_gap = float(np.hypot(*(xN_rti - xN_slsqp)))
    assert terminal_gap < 1.0, (
        f"{scenario_name}: terminal predicted position diverged too far "
        f"({terminal_gap:.2f} m) -- slsqp={xN_slsqp} rti={xN_rti}")
