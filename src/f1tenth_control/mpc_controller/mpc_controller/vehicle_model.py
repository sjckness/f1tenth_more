#!/usr/bin/env python3

import math
from typing import List, Sequence, Tuple

import numpy as np


def f1tenth_state_fcn_dt_beta(
    xk: Sequence[float],
    uk: Sequence[float],
    ts: float,
    wheelbase: float,
    lr: float
) -> List[float]:
    """
    Stato:    [X, Y, psi, v]
    Controllo [delta, a]
    """
    x = float(xk[0])
    y = float(xk[1])
    psi = float(xk[2])
    v = float(xk[3])

    delta = float(uk[0])
    a = float(uk[1])

    beta = math.atan((lr / wheelbase) * math.tan(delta))

    xdot = v * math.cos(psi + beta)
    #xdot = v
    ydot = v * math.sin(psi + beta)
    #ydot = v * (psi + beta)
    psidot = (v / wheelbase) * math.cos(beta) * math.tan(delta)
    #psidot = (v / wheelbase) * (delta)
    vdot = a

    x_next = x + ts * xdot
    y_next = y + ts * ydot
    psi_next = psi + ts * psidot
    v_next = v + ts * vdot

    return [x_next, y_next, psi_next, v_next]


def f1tenth_state_fcn_dt_beta_jacobian(
    xk: Sequence[float],
    uk: Sequence[float],
    ts: float,
    wheelbase: float,
    lr: float
) -> Tuple[np.ndarray, np.ndarray]:
    """Analytic Jacobians of f1tenth_state_fcn_dt_beta at (xk, uk).

    Added for mpc_solver.py's OSQP real-time-iteration (RTI) path: RTI
    linearizes the (otherwise nonlinear) dynamics once per control tick
    around a reference trajectory point, instead of SLSQP's per-iteration
    finite-difference gradient -- that's the structural cost RTI removes, so
    this is deliberately a closed-form (not numerical) derivative: cheap,
    exact, and independent of step-size choice.

    Returns (A, B) where, to first order:
        x_next ~= f(x_ref, u_ref) + A @ (x - x_ref) + B @ (u - u_ref)
    A is the 4x4 d(x_next)/d(x) Jacobian, B is the 4x2 d(x_next)/d(u)
    Jacobian, both evaluated at (xk, uk) -- the caller is responsible for
    passing the reference point, not the actual current state, when building
    an RTI-style affine model (see mpc_solver.py's _linearize_dynamics_rti).

    Derivation: same explicit-Euler discretization as
    f1tenth_state_fcn_dt_beta, differentiated by hand (bicycle model with
    slip angle beta = atan((lr/L) tan(delta)), 4 states / 2 controls -- small
    and simple enough to be worth doing in closed form rather than trusting a
    numerical Jacobian at every solve). Verified against
    scipy.optimize.approx_fprime finite differences in
    test/test_solver_rti.py.
    """
    psi = float(xk[2])
    v = float(xk[3])
    delta = float(uk[0])

    k = lr / wheelbase
    t = math.tan(delta)
    sec2_delta = 1.0 + t * t              # d(tan(delta))/d(delta)
    denom = 1.0 + (k * t) ** 2            # 1 + k^2 tan^2(delta)
    beta = math.atan(k * t)
    dbeta_ddelta = (k * sec2_delta) / denom

    cos_pb = math.cos(psi + beta)
    sin_pb = math.sin(psi + beta)
    cos_b = math.cos(beta)
    sin_b = math.sin(beta)

    # d/ddelta [ cos(beta) * tan(delta) ]
    dgd_ddelta = -sin_b * dbeta_ddelta * t + cos_b * sec2_delta

    A = np.array([
        [1.0, 0.0, -ts * v * sin_pb,               ts * cos_pb],
        [0.0, 1.0,  ts * v * cos_pb,                ts * sin_pb],
        [0.0, 0.0,  1.0,                            (ts / wheelbase) * cos_b * t],
        [0.0, 0.0,  0.0,                            1.0],
    ], dtype=float)

    B = np.array([
        [-ts * v * sin_pb * dbeta_ddelta, 0.0],
        [ ts * v * cos_pb * dbeta_ddelta, 0.0],
        [ ts * (v / wheelbase) * dgd_ddelta, 0.0],
        [0.0, ts],
    ], dtype=float)

    return A, B
