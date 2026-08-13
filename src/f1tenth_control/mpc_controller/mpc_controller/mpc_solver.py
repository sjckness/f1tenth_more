#!/usr/bin/env python3
"""mpc_corr's solve step: two interchangeable solvers behind one
solve_mpc_step() entry point.

  * _solve_slsqp -- the original from-scratch nonlinear SLSQP solve (SciPy
    minimize, one full nonlinear re-solve every 100ms tick). Kept verbatim
    (same cost/constraint functions, same options) as the rollback target.
  * _solve_rti -- OSQP-based real-time-iteration (RTI): linearize the
    dynamics + corridor + obstacle-cost terms ONCE per tick around a
    reference trajectory, then solve the resulting convex QP with OSQP
    (warm-started from the previous tick's solution). Added by the MPC
    optimization pass following the frequency/bottleneck audit -- SLSQP's
    solve_dt measured 93ms of a 112.6ms/100ms-budget loop (82.6%), and this
    is the standard fix for "full nonlinear re-solve every tick, short
    horizon, fast sample rate."

solve_mpc_step()'s signature and returned info dict shape
(success/status/cost/zopt/x_pred) are identical either way -- MPC_corr.py
calls it as a black box and does not need to know which path ran.

Per the approved MPC-optimization-pass decision: _solve_rti accepts
whatever OSQP returns as long as it produced a numeric AND FEASIBLE
solution (matches _solve_slsqp's own pre-existing behavior of falling back
to the warm start z0 only when the solver produced literally nothing
usable) -- there is no per-tick fallback to _solve_slsqp on a merely
low-quality/inaccurate RTI solve. Hard constraints (steering/accel/rate/
corridor bounds) stay hard QP constraints in _solve_rti exactly as they're
hard SLSQP constraints today, so a low-quality solve means worse
tracking/avoidance, not an unsafe command -- PROVIDED the "numeric AND
FEASIBLE" check actually holds: a plain isfinite() check on results.x
alone is NOT sufficient (see _OSQP_INFEASIBLE_STATUSES' own comment, added
after hard boundary constraints below made genuine infeasibility easily
reachable and demonstrated the gap live) -- an infeasibility certificate
can be finite while being an arbitrarily large, physically meaningless
number nowhere near the actual box constraints.

Hard boundary constraints (_solve_rti ONLY -- see below): complements the
existing soft w_obs obstacle-avoidance cost with up to 3 HARD linear
constraints from f1tenth_perception (the ZED front wall,
wall_detector_node.py's /perception/front_wall_boundary, plus the lidar
left/right line fits, lidar_boundary_node.py's /perception/lidar_boundaries)
-- see MPC_corr.py's own module-level note for how it gathers/stales/
transforms these into the `boundaries` list this module consumes. Each
active constraint is `normal . (x, y) <= offset` (see
f1tenth_messages/BoundaryConstraint.msg's own field comments for the sign
convention), added as a hard row at EVERY stage k in the horizon -- this
is what makes it "per-stage" with no raycasting: the bound itself is fixed
for the whole solve, but checking it against every predicted stage
position naturally constrains the whole predicted trajectory, regardless
of predicted heading.

Fixed-3-slot padding (pad_boundary_constraints): the RTI/OSQP path is
warm-started every tick (see _solve_rti's own docstring) and a resized
constraint matrix is the textbook way to fight that -- even though
CURRENTLY every _solve_rti call does a fresh osqp.OSQP().setup() rather
than incrementally osqp.update()-ing a persistent solver object (so a
varying row count wouldn't literally break TODAY's warm_start(x=z_guess)
call), keeping the QP's structure IDENTICAL tick-to-tick is still the
right discipline here: it's what a future move to incremental .update()
solving (a natural next optimization on top of this one) would need
anyway, and there's no real cost to it now. So: ALWAYS exactly
max_sources=3 boundary rows per stage, regardless of how many sources are
actually live this tick -- an absent/stale/disabled slot is a sentinel
((0.0, 0.0, math.inf), see pad_boundary_constraints) whose resulting QP
row is all-zero (0*z <= anything, unconditionally satisfied), not a
missing row.

Purely additive: does not remove or weaken w_obs/compute_local_target's
existing soft deflection -- boundaries is an EXTRA set of hard rows on
top of everything _solve_rti already builds, empty by default (`[]`),
matching solve_mpc_step's/_solve_slsqp's existing "boundaries not
implemented there" scope (SLSQP is the legacy/rollback path, not what
this feature targets -- see solve_mpc_step's own note).
"""

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import minimize

from mpc_controller.vehicle_model import (
    f1tenth_state_fcn_dt_beta,
    f1tenth_state_fcn_dt_beta_jacobian,
)

# osqp is a pip-only dependency (see docker/Dockerfile.* + README), not a
# rosdep key -- same undeclared-in-package.xml convention this workspace
# already uses for numpy/scipy in this same package. Guarded import so this
# module (and the SLSQP path, and everything that only needs
# planner_cost_corridor/nearest_centerline_index etc.) keeps working even on
# a machine that hasn't installed osqp yet; _solve_rti raises a clear error
# only if it's actually called without it.
try:
    import osqp
    OSQP_AVAILABLE = True
except ImportError:
    osqp = None
    OSQP_AVAILABLE = False

# Statuses where results.x is an INFEASIBILITY CERTIFICATE, not a usable
# primal solution -- can be finite (even huge-but-finite) while being
# physically meaningless, so the plain isfinite() check in _solve_rti
# alone doesn't catch it. Found live while testing hard boundary
# constraints (see that feature's own module-docstring section): a tight
# boundary combined with a reference-trajectory linearization that
# couldn't satisfy it produced OSQP_PRIMAL_INFEASIBLE_INACCURATE with a
# "solution" of ~2e9 in a control channel bounded to [-1.05, 1.05] -- the
# OLD finite-ness-only check let that straight through as success=True.
if OSQP_AVAILABLE:
    _OSQP_INFEASIBLE_STATUSES = {
        osqp.SolverStatus.OSQP_PRIMAL_INFEASIBLE,
        osqp.SolverStatus.OSQP_PRIMAL_INFEASIBLE_INACCURATE,
        osqp.SolverStatus.OSQP_DUAL_INFEASIBLE,
        osqp.SolverStatus.OSQP_DUAL_INFEASIBLE_INACCURATE,
    }
else:
    _OSQP_INFEASIBLE_STATUSES = set()


# ==============================================================================
# Hard boundary constraints -- pure functions, no OSQP/numpy-solver
# dependency, independently unit-testable. See module docstring's "Hard
# boundary constraints" section for the full design.
# ==============================================================================

# Sentinel for an absent/inactive boundary slot (see pad_boundary_constraints):
# a zero normal makes the resulting QP row all-zero, unconditionally
# satisfied regardless of the offset/margin math applied to it -- matches
# wall_detector_node.py's own _wall_boundary_from_track pathological-case
# return value exactly (same sentinel shape, same reasoning).
BOUNDARY_DISABLED = (0.0, 0.0, math.inf)


def pad_boundary_constraints(
        boundaries: Sequence[Tuple[float, float, float]],
        max_sources: int = 3) -> List[Tuple[float, float, float]]:
    """Pad/truncate `boundaries` (each (normal_x, normal_y, offset), 0..
    max_sources actually-live hard-boundary constraints this tick) to
    EXACTLY max_sources entries -- extra slots are BOUNDARY_DISABLED. See
    module docstring's "Fixed-3-slot padding" section for why this must
    happen even when 0-2 sources are actually live."""
    boundaries = list(boundaries)[:max_sources]
    while len(boundaries) < max_sources:
        boundaries.append(BOUNDARY_DISABLED)
    return boundaries


def boundary_constraint_bounds(
        normal_x: float, normal_y: float, offset: float,
        car_radius: float, margin: float) -> Tuple[float, float]:
    """(lo, hi) OSQP bounds for ONE boundary constraint row at ONE stage:
    normal . (x, y) <= offset - car_radius - margin, one-sided (lo=-inf).
    A BOUNDARY_DISABLED slot (offset=inf) naturally produces hi=inf too --
    combined with that slot's all-zero normal (applied by the caller, not
    here), the resulting row is unconditionally satisfied regardless."""
    return -math.inf, offset - car_radius - margin


def solve_mpc_step(
    x0: Sequence[float],
    last_u: Sequence[float],
    pref_nom: Sequence[float],
    corridor: Dict,
    horizon: int,
    ts: float,
    params: Dict,
    limits: Dict,
    weights: Dict,
    obstacles: List[Tuple[float, float, float]],
    dmin: float,
    vdes: float,
    solver: str = 'rti',
    boundaries: Optional[List[Tuple[float, float, float]]] = None,
) -> Tuple[np.ndarray, Dict]:
    """
    Replica MATLAB:
    - costo SOLO sul punto finale della direttrice
    - corridoio hard
    - rate hard
    - speed hard
    - obstacle hard solo davanti al muso

    solver: 'rti' (default, OSQP real-time-iteration) or 'slsqp' (original
    from-scratch nonlinear solve). See module docstring.

    boundaries: up to 3 (normal_x, normal_y, offset) hard boundary
    constraints (see module docstring's "Hard boundary constraints"
    section) -- RTI-ONLY (padded/added as extra hard QP rows in
    _solve_rti); silently ignored by the legacy/rollback SLSQP path, which
    this feature doesn't target. None (default) means "no boundaries",
    identical to passing an empty list.
    """
    if solver == 'slsqp':
        return _solve_slsqp(
            x0, last_u, pref_nom, corridor, horizon, ts, params, limits,
            weights, obstacles, dmin, vdes)
    elif solver == 'rti':
        return _solve_rti(
            x0, last_u, pref_nom, corridor, horizon, ts, params, limits,
            weights, obstacles, dmin, vdes, boundaries=boundaries)
    else:
        raise ValueError(f"Unknown solver={solver!r}, expected 'rti' or 'slsqp'")


def _rollout_true_dynamics(
    x0: np.ndarray,
    z: np.ndarray,
    horizon: int,
    ts: float,
    params: Dict,
) -> np.ndarray:
    """Roll a flat control sequence z=[delta_0,a_0,delta_1,a_1,...] forward
    through the TRUE nonlinear model (never the RTI path's linearized one),
    so info["x_pred"] always reflects the same "what will actually happen"
    prediction regardless of which solver chose z -- MPC_corr.py's
    compute_predicted_clearance() etc. depend on that being physically
    meaningful, not a linearization artifact.
    """
    x_pred_list = []
    x_roll = np.asarray(x0, dtype=float).copy()
    for k in range(horizon):
        uk = z[2 * k: 2 * k + 2]
        x_roll = np.asarray(
            f1tenth_state_fcn_dt_beta(
                xk=x_roll, uk=uk, ts=ts,
                wheelbase=params["L"], lr=params["lr"]),
            dtype=float)
        x_pred_list.append(x_roll.copy())
    return np.array(x_pred_list, dtype=float)


def _solve_slsqp(
    x0: Sequence[float],
    last_u: Sequence[float],
    pref_nom: Sequence[float],
    corridor: Dict,
    horizon: int,
    ts: float,
    params: Dict,
    limits: Dict,
    weights: Dict,
    obstacles: List[Tuple[float, float, float]],
    dmin: float,
    vdes: float
) -> Tuple[np.ndarray, Dict]:
    """Original solver, unchanged. See module docstring."""
    x0 = np.asarray(x0, dtype=float)
    last_u = np.asarray(last_u, dtype=float)
    pref_nom = np.asarray(pref_nom, dtype=float)

    z0 = np.tile(last_u, horizon)

    lb = np.tile([limits["delta_min"], limits["a_min"]], horizon)
    ub = np.tile([limits["delta_max"], limits["a_max"]], horizon)
    bounds = list(zip(lb, ub))

    constraints = [
        {
            "type": "ineq",
            "fun": lambda z: -planner_nonlcon_corridor(
                z=z,
                x0=x0,
                last_u=last_u,
                corridor=corridor,
                horizon=horizon,
                ts=ts,
                params=params,
                limits=limits,
                obstacles=obstacles,
                dmin=dmin
            )
        }
    ]

    result = minimize(
        fun=lambda z: planner_cost_corridor(
            z=z,
            x0=x0,
            last_u=last_u,
            pref_nom=pref_nom,
            corridor=corridor,
            horizon=horizon,
            ts=ts,
            params=params,
            weights=weights,
            vdes=vdes
        ),
        x0=z0,
        method="SLSQP",
        bounds=bounds,
        constraints=constraints,
        options={
            "maxiter": 120,
            "ftol": 1e-3,
            "disp": False
        }
    )

    if result.success and result.x is not None:
        zopt = np.asarray(result.x, dtype=float)
    else:
        zopt = np.asarray(z0, dtype=float)

    u0 = zopt[:2].copy()

    info = {
        "success": bool(result.success),
        "status": int(result.status),
        "message": str(result.message),
        "cost": float(result.fun) if result.fun is not None else 0.0,
        "zopt": zopt.copy()
    }

    info["x_pred"] = _rollout_true_dynamics(x0, zopt, horizon, ts, params)

    return u0, info


# ============================================================
# RTI / OSQP path
# ============================================================

_SOFTPLUS_BETA = 10.0  # matches planner_cost_corridor's own softplus(beta=10.0)


def _softplus_sq_derivs(s: float, beta: float = _SOFTPLUS_BETA) -> Tuple[float, float, float]:
    """g(s) = softplus(s, beta)^2 and its first/second derivatives at s.

    softplus(s) = log1p(exp(beta*s))/beta is convex, and g=softplus^2 is
    convex too (square of a nonnegative convex nondecreasing function) --
    so g''(s) >= 0 always, which keeps the QP's obstacle-cost contribution
    to P positive-semidefinite (required for OSQP's convex-QP assumption).
    """
    # numerically stable sigmoid(beta*s)
    bs = beta * s
    if bs >= 0:
        sig = 1.0 / (1.0 + np.exp(-bs))
    else:
        e = np.exp(bs)
        sig = e / (1.0 + e)
    sp = np.log1p(np.exp(-abs(bs))) / beta + max(s, 0.0)  # stable softplus(s)
    sp_prime = sig
    sp_second = beta * sig * (1.0 - sig)

    g = sp * sp
    g_prime = 2.0 * sp * sp_prime
    g_second = 2.0 * sp_prime * sp_prime + 2.0 * sp * sp_second
    return float(g), float(g_prime), float(g_second)


def _d_front_at(X: float, Y: float, psi: float, L: float,
                 obstacles_world: List[Tuple[float, float, float]]) -> float:
    """Same front-clearance computation as planner_cost_corridor's obstacle
    block (3 nose-offset points, min distance over all obstacles), exposed
    standalone as a function of (X,Y,psi) so _linearize_obstacle_cost_rti
    can numerically differentiate it at a reference point. Not evaluated
    per-SLSQP-iteration like the original -- just once (plus a few central-
    difference perturbations) per horizon step per tick, which is what makes
    this affordable inside RTI's single-linearization-per-tick budget.
    """
    e = np.array([np.cos(psi), np.sin(psi)], dtype=float)
    p_nose = np.array([X, Y], dtype=float) + (L / 2.0) * e
    front_points = [p_nose + 0.10 * e, p_nose + 0.25 * e, p_nose + 0.40 * e]

    d_front = 1e6
    for pF in front_points:
        for ox, oy, r in obstacles_world:
            d = float(np.hypot(pF[0] - ox, pF[1] - oy) - r)
            d_front = min(d_front, d)
    return d_front


def _linearize_obstacle_cost_rti(
    x_ref: np.ndarray,
    L: float,
    car_radius: float,
    avoidance_margin: float,
    obstacles_world: List[Tuple[float, float, float]],
) -> Tuple[np.ndarray, float]:
    """Local quadratic model of w_obs's per-step cost term
    softplus((car_radius+avoidance_margin) - d_front(X,Y,psi))^2 around
    x_ref, returned as (M, b) such that the term's QP contribution for
    state vector x_k is x_k^T M x_k + b @ x_k (constant dropped -- doesn't
    affect argmin). M is guaranteed PSD (see _softplus_sq_derivs).

    d_front's gradient w.r.t. (X,Y,psi) is taken numerically (central
    differences) rather than by hand-differentiating the nested
    min-over-obstacles-and-frontpoints expression -- deliberate choice, see
    mpc_solver.py module docstring / the MPC-optimization-pass plan: cheap
    (6 extra evaluations of a trivial function per horizon step) and avoids
    an error-prone by-hand chain rule through a discrete argmin, at a cost
    nowhere near what SLSQP's own per-iteration finite differencing was
    paying.
    """
    n_x = 4
    if not obstacles_world:
        return np.zeros((n_x, n_x)), np.zeros(n_x)

    X, Y, psi = float(x_ref[0]), float(x_ref[1]), float(x_ref[2])
    eps = 1e-4

    d0 = _d_front_at(X, Y, psi, L, obstacles_world)
    dX = (_d_front_at(X + eps, Y, psi, L, obstacles_world)
          - _d_front_at(X - eps, Y, psi, L, obstacles_world)) / (2 * eps)
    dY = (_d_front_at(X, Y + eps, psi, L, obstacles_world)
          - _d_front_at(X, Y - eps, psi, L, obstacles_world)) / (2 * eps)
    dPsi = (_d_front_at(X, Y, psi + eps, L, obstacles_world)
            - _d_front_at(X, Y, psi - eps, L, obstacles_world)) / (2 * eps)

    s_ref = (car_radius + avoidance_margin) - d0
    grad_s = np.array([-dX, -dY, -dPsi, 0.0], dtype=float)  # d(s)/d(X,Y,psi,v)

    g_ref, g1, g2 = _softplus_sq_derivs(s_ref)

    m = grad_s
    c = x_ref  # reference point itself (m . x_ref = m . c)
    m_dot_c = float(m @ c)

    # g(s) ~= g_ref + g1*(m.(x-c)) + 0.5*g2*(m.(x-c))^2, expanded in x (see
    # mpc_solver.py's derivation in the MPC-optimization-pass plan):
    M = 0.5 * g2 * np.outer(m, m)          # quadratic block (x^T M x)
    b = g1 * m - g2 * m_dot_c * m           # linear block (b . x)
    return M, b


def _nearest_corridor_frame(p: np.ndarray, corridor: Dict) -> Tuple[np.ndarray, np.ndarray, float]:
    """Frozen (pc, n, half_w) at the nearest corridor index to p -- reuses
    the same nearest_centerline_index lookup planner_nonlcon_corridor uses,
    so the RTI corridor constraint's geometry matches the SLSQP path's
    exactly at the reference point (only the trailing linearization of
    "stay within half_w of THIS index" is new, and that part is exact, not
    approximate -- see module docstring).
    """
    _, idx = nearest_centerline_index(p, corridor)
    pc = np.array([corridor["xc"][idx], corridor["yc"][idx]], dtype=float)
    n = np.array([corridor["nx"][idx], corridor["ny"][idx]], dtype=float)
    half_w = float(corridor["halfWidth"][idx])
    return pc, n, half_w


def _solve_rti(
    x0: Sequence[float],
    last_u: Sequence[float],
    pref_nom: Sequence[float],
    corridor: Dict,
    horizon: int,
    ts: float,
    params: Dict,
    limits: Dict,
    weights: Dict,
    obstacles: List[Tuple[float, float, float]],
    dmin: float,
    vdes: float,
    warm_start_z: Optional[np.ndarray] = None,
    boundaries: Optional[List[Tuple[float, float, float]]] = None,
) -> Tuple[np.ndarray, Dict]:
    """One linearization + one warm-started OSQP QP solve. See module
    docstring for the overall design and the MPC-optimization-pass plan for
    the full per-term derivation (dynamics: exact analytic Jacobian;
    control/rate/speed bounds: exact, already linear; corridor bound: exact
    given the frozen nearest-index; obstacle soft cost: local quadratic
    model). warm_start_z defaults to last_u tiled (matches _solve_slsqp's
    own z0 warm start) when the caller has no previous solution yet.

    boundaries: up to 3 (normal_x, normal_y, offset) hard boundary
    constraints, padded to exactly 3 (pad_boundary_constraints) and added
    as extra hard rows at EVERY stage -- see module docstring's "Hard
    boundary constraints" section. None/[] (default) means all 3 slots are
    disabled (BOUNDARY_DISABLED) -- a fully inert no-op, identical to this
    feature not existing.
    """
    if not OSQP_AVAILABLE:
        raise RuntimeError(
            "solver='rti' requires the 'osqp' package (pip install osqp) -- "
            "not importable. Pass solver='slsqp' to fall back to the "
            "original solver, or install osqp (see docker/Dockerfile.*).")

    x0 = np.asarray(x0, dtype=float)
    last_u = np.asarray(last_u, dtype=float)
    pref_nom = np.asarray(pref_nom, dtype=float)
    N = horizon
    n_x, n_u = 4, 2

    if warm_start_z is None:
        warm_start_z = np.tile(last_u, N)
    warm_start_z = np.asarray(warm_start_z, dtype=float)

    # ---- 1. Reference trajectory: forward-simulate the TRUE nonlinear
    # model using the warm-start control sequence (standard RTI practice --
    # linearize around a dynamically-consistent guess, not x0 held frozen).
    x_ref = [x0.copy()]
    u_ref = []
    xk = x0.copy()
    for k in range(N):
        uk = warm_start_z[2 * k: 2 * k + 2]
        u_ref.append(uk)
        xk = np.asarray(
            f1tenth_state_fcn_dt_beta(xk, uk, ts, params["L"], params["lr"]),
            dtype=float)
        x_ref.append(xk)

    # ---- 2. Linearize dynamics at each step.
    A_list, B_list, c_list = [], [], []
    for k in range(N):
        A_k, B_k = f1tenth_state_fcn_dt_beta_jacobian(
            x_ref[k], u_ref[k], ts, params["L"], params["lr"])
        f_ref = np.asarray(
            f1tenth_state_fcn_dt_beta(x_ref[k], u_ref[k], ts, params["L"], params["lr"]),
            dtype=float)
        c_k = f_ref - A_k @ x_ref[k] - B_k @ u_ref[k]
        A_list.append(A_k)
        B_list.append(B_k)
        c_list.append(c_k)

    # ---- 3. Decision vector layout: z = [u_0..u_{N-1}, x_1..x_N].
    n_z = N * (n_u + n_x)

    def u_idx(k):
        return slice(k * n_u, k * n_u + n_u)

    def x_idx(k):  # k = 1..N (state AFTER step k-1, i.e. x_ref[k])
        base = N * n_u
        return slice(base + (k - 1) * n_x, base + (k - 1) * n_x + n_x)

    P = np.zeros((n_z, n_z))
    q = np.zeros(n_z)
    rows: List[np.ndarray] = []
    lows: List[float] = []
    ups: List[float] = []

    def add_row(row: np.ndarray, lo: float, hi: float):
        rows.append(row)
        lows.append(lo)
        ups.append(hi)

    # ---- Dynamics equality constraints: x_{k+1} - A_k x_k - B_k u_k = c_k
    # (x_0 is x0, a known constant, not a decision variable -- folded into
    # the RHS of the k=0 block).
    for k in range(N):
        block = np.zeros((n_x, n_z))
        block[:, u_idx(k)] = -B_list[k]
        if k == 0:
            rhs = c_list[k] + A_list[k] @ x0
        else:
            block[:, x_idx(k)] = -A_list[k]
            rhs = c_list[k]
        block[:, x_idx(k + 1)] = np.eye(n_x)
        for r in range(n_x):
            add_row(block[r].copy(), float(rhs[r]), float(rhs[r]))

    # ---- Control box + rate constraints (exact, same bounds as SLSQP path).
    u_prev = last_u
    for k in range(N):
        row_d = np.zeros(n_z)
        row_d[u_idx(k)] = [1.0, 0.0]
        add_row(row_d, limits["delta_min"], limits["delta_max"])

        row_a = np.zeros(n_z)
        row_a[u_idx(k)] = [0.0, 1.0]
        add_row(row_a, limits["a_min"], limits["a_max"])

        rate_row_d = np.zeros(n_z)
        rate_row_d[u_idx(k)] += [1.0, 0.0]
        rate_lo_d = limits["dDeltaMin"] * ts
        rate_hi_d = limits["dDeltaMax"] * ts
        if k > 0:
            rate_row_d[u_idx(k - 1)] += [-1.0, 0.0]
        else:
            rate_lo_d += u_prev[0]
            rate_hi_d += u_prev[0]
        add_row(rate_row_d, rate_lo_d, rate_hi_d)

        rate_row_a = np.zeros(n_z)
        rate_row_a[u_idx(k)] += [0.0, 1.0]
        rate_lo_a = limits["dAMin"] * ts
        rate_hi_a = limits["dAMax"] * ts
        if k > 0:
            rate_row_a[u_idx(k - 1)] += [0.0, -1.0]
        else:
            rate_lo_a += u_prev[1]
            rate_hi_a += u_prev[1]
        add_row(rate_row_a, rate_lo_a, rate_hi_a)

    # ---- Per-step cost/constraint terms evaluated at x_ref[k+1] (the state
    # AFTER step k), matching planner_cost_corridor's own convention (it
    # steps x forward first, then costs the result against uk).
    obstacles_world = corridor.get("obstacles_world", obstacles)
    car_radius = corridor.get("car_radius", 0.0)
    avoidance_margin = corridor.get("avoidance_margin", 0.12)

    # Hard boundary constraints (see module docstring's "Hard boundary
    # constraints" section) -- padded to a FIXED 3 slots once, outside the
    # stage loop (the same 3 (normal, offset) tuples apply at every stage;
    # only the ROW itself differs per stage, via xk1_idx below).
    padded_boundaries = pad_boundary_constraints(boundaries or [])

    for k in range(N):
        xk1_idx = x_idx(k + 1)
        x_ref_k1 = x_ref[k + 1]

        # velocity tracking: w_v*(v-vdes)^2, v = x_{k+1}[3] -- exact quadratic.
        # c*(e.x - r)^2 => P += 2c*outer(e,e), q += -2*c*r*e (const dropped).
        e_v = np.zeros(n_x)
        e_v[3] = 1.0
        P[xk1_idx, xk1_idx] += 2.0 * weights["w_v"] * np.outer(e_v, e_v)
        q[xk1_idx] += -2.0 * weights["w_v"] * vdes * e_v

        # control effort: w_delta0*delta_k^2 + w_u_a*a_k^2 -- exact quadratic,
        # r=0 so no q contribution.
        uk_idx = u_idx(k)
        P[uk_idx, uk_idx] += 2.0 * weights["w_delta0"] * np.array([[1.0, 0.0], [0.0, 0.0]])
        P[uk_idx, uk_idx] += 2.0 * weights["w_u_a"] * np.array([[0.0, 0.0], [0.0, 1.0]])

        # rate smoothness: (u_k - u_prev)' Wdu (u_k - u_prev) -- exact
        # quadratic. For k==0, u_prev=last_u is a CONSTANT, so it contributes
        # both a P diagonal block and a q linear term. For k>0, u_prev=u_{k-1}
        # is itself a decision variable, so the cross-dependency is captured
        # entirely by the off-diagonal P blocks below -- no separate q term
        # (adding one too, unconditionally, would double-count it).
        Wdu = np.diag([weights["w_du_delta"], weights["w_du_a"]])
        P[uk_idx, uk_idx] += 2.0 * Wdu
        if k == 0:
            q[uk_idx] += -2.0 * Wdu @ last_u
        else:
            uk_1_idx = u_idx(k - 1)
            P[uk_idx, uk_1_idx] += -2.0 * Wdu
            P[uk_1_idx, uk_idx] += -2.0 * Wdu
            P[uk_1_idx, uk_1_idx] += 2.0 * Wdu

        # obstacle soft cost: local quadratic model (see
        # _linearize_obstacle_cost_rti).
        M_obs, b_obs = _linearize_obstacle_cost_rti(
            x_ref_k1, params["L"], car_radius, avoidance_margin, obstacles_world)
        P[xk1_idx, xk1_idx] += 2.0 * weights["w_obs"] * M_obs
        q[xk1_idx] += weights["w_obs"] * b_obs

        # corridor lateral bound: exact linear inequality at the frozen
        # nearest index.
        pc, n_vec, half_w = _nearest_corridor_frame(x_ref_k1[:2], corridor)
        row_corr = np.zeros(n_z)
        row_corr[xk1_idx][0] = n_vec[0]
        row_corr[xk1_idx][1] = n_vec[1]
        n_dot_pc = float(n_vec @ pc)
        add_row(row_corr, n_dot_pc - half_w, n_dot_pc + half_w)

        # speed bound: box on x_{k+1}[3].
        row_v = np.zeros(n_z)
        row_v[xk1_idx][3] = 1.0
        add_row(row_v, limits["vMin"], limits["vMax"])

        # hard boundary constraints: ALWAYS exactly 3 rows (padded above),
        # one per source, at this stage's predicted (x, y) -- see module
        # docstring. A disabled slot's all-zero normal row is
        # unconditionally satisfied regardless of its (also-disabled) bound.
        for bnx, bny, boffset in padded_boundaries:
            row_b = np.zeros(n_z)
            row_b[xk1_idx][0] = bnx
            row_b[xk1_idx][1] = bny
            lo, hi = boundary_constraint_bounds(
                bnx, bny, boffset, car_radius, avoidance_margin)
            add_row(row_b, lo, hi)

    # ---- terminal position cost: w_term * ||x_N[:2] - pref_nom||^2, exact
    # quadratic, only at the LAST state block.
    xN_idx = x_idx(N)
    E = np.zeros((2, n_x))
    E[0, 0] = 1.0
    E[1, 1] = 1.0
    P[xN_idx, xN_idx] += 2.0 * weights["w_term"] * (E.T @ E)
    q[xN_idx] += weights["w_term"] * (-2.0 * (E.T @ pref_nom))

    # ---- assemble + solve.
    A_mat = np.array(rows, dtype=float)
    l_vec = np.array(lows, dtype=float)
    u_vec = np.array(ups, dtype=float)

    # Symmetrize P defensively (small asymmetries can creep in from the
    # +=/cross-term bookkeeping above; OSQP expects/assumes a symmetric P
    # and only reads its upper triangle, but an asymmetric P would silently
    # encode the wrong cost).
    P = 0.5 * (P + P.T)

    import scipy.sparse as sp
    P_csc = sp.csc_matrix(P)
    A_csc = sp.csc_matrix(A_mat)

    z_guess = np.concatenate([warm_start_z, np.concatenate(x_ref[1:])])

    prob = osqp.OSQP()
    # warm_starting/polishing: current osqp (>=1.x) setting names -- the
    # older warm_start=/polish= aliases still work but log a
    # DeprecationWarning every solve (found running this file's own tests),
    # and Dockerfile.* pin no osqp version, so whatever's newest at image
    # build time is what actually ships -- worth staying on the current names.
    prob.setup(P_csc, q, A_csc, l_vec, u_vec,
               warm_starting=True, verbose=False, polishing=True, max_iter=200)
    prob.warm_start(x=z_guess)
    # raise_error=False: this function's whole point (see module docstring/
    # the approved MPC-optimization-pass decision) is to accept whatever
    # OSQP returns, including a non-converged/inaccurate solve, via the
    # results.x finite-ness check below -- osqp's own upcoming default is
    # to raise instead of returning such a result, which would turn a merely
    # low-quality solve into an uncaught exception instead of the
    # already-handled "hold the warm start" fallback path.
    results = prob.solve(raise_error=False)

    status_val = results.info.status_val if results.info is not None else None
    is_infeasible_certificate = status_val in _OSQP_INFEASIBLE_STATUSES

    if (results.x is not None and np.all(np.isfinite(results.x))
            and not is_infeasible_certificate):
        z_full = np.asarray(results.x, dtype=float)
        success = True
    else:
        # Mirrors _solve_slsqp's own pre-existing behavior on a solver that
        # produced nothing usable: hold the warm start rather than inventing
        # a value. Not a fallback to _solve_slsqp -- see module docstring.
        # Also catches infeasibility certificates (see
        # _OSQP_INFEASIBLE_STATUSES' own comment) -- finite but not a real
        # solution.
        z_full = z_guess.copy()
        success = False

    zopt = z_full[: N * n_u].copy()  # control sequence only, same shape/
    # meaning as _solve_slsqp's zopt (both are the flat [delta_0,a_0,...]
    # control sequence) -- x_1..x_N are internal to this solver.
    u0 = zopt[:2].copy()

    x_pred = _rollout_true_dynamics(x0, zopt, N, ts, params)

    # True (non-quadratic-approximated) cost at this solution, for
    # apples-to-apples comparison/logging against _solve_slsqp -- see module
    # docstring.
    true_cost = planner_cost_corridor(
        z=zopt, x0=x0, last_u=last_u, pref_nom=pref_nom, corridor=corridor,
        horizon=N, ts=ts, params=params, weights=weights, vdes=vdes)

    info = {
        "success": success,
        "status": int(results.info.status_val) if results.info is not None else -1,
        "status_message": str(results.info.status) if results.info is not None else "no solve",
        "message": str(results.info.status) if results.info is not None else "osqp returned no solution",
        "cost": float(true_cost),
        "zopt": zopt.copy(),
        "x_pred": x_pred,
    }

    return u0, info


def planner_cost_corridor(
    z: Sequence[float],
    x0: np.ndarray,
    last_u: np.ndarray,
    pref_nom: np.ndarray,
    corridor: Dict,
    horizon: int,
    ts: float,
    params: Dict,
    weights: Dict,
    vdes: float
) -> float:
    z = np.asarray(z, dtype=float)
    x = np.asarray(x0, dtype=float).copy()
    u_prev = np.asarray(last_u, dtype=float).copy()

    J = 0.0

    for k in range(horizon):
        uk = z[2 * k: 2 * k + 2]
        x = np.asarray(
            f1tenth_state_fcn_dt_beta(
                xk=x,
                uk=uk,
                ts=ts,
                wheelbase=params["L"],
                lr=params["lr"]
            ),
            dtype=float
        )

        v = x[3]
        delta = uk[0]
        a = uk[1]

        # ================= ostacolo soft =================

        def softplus(s: float, beta: float = 10.0) -> float:
            return float(np.log1p(np.exp(beta * s)) / beta)

        # ================= ostacolo soft GRADUALE =================
        psi = float(x[2])
        e = np.array([np.cos(psi), np.sin(psi)], dtype=float)

        p_nose = np.array([x[0], x[1]], dtype=float) + (params["L"] / 2.0) * e
        front_points = [
            p_nose + 0.10 * e,
            p_nose + 0.25 * e,
            p_nose + 0.40 * e
        ]

        d_front = 1e6
        for pF in front_points:
            for ox, oy, r in corridor["obstacles_world"]:
                d = np.hypot(pF[0] - ox, pF[1] - oy) - r
                d_front = min(d_front, d)

        # UPGRADE: trigger distance now sized off car_radius + avoidance_margin
        # (set by MPC_corr.py's control_loop into the corridor dict) instead of
        # d_safe (self.dmin, the disabled hard constraint's own value) + a
        # hardcoded 0.3m margin -- neither previously accounted for the car's
        # actual footprint. .get() with fallbacks so this doesn't hard-crash if
        # called from somewhere that hasn't been updated to inject these keys.
        car_radius = corridor.get("car_radius", 0.0)
        avoidance_margin = corridor.get("avoidance_margin", 0.12)

        s = (car_radius + avoidance_margin) - d_front
        pen = softplus(s, beta=10.0)

        J += weights["w_obs"] * pen ** 2

        # ================= velocità =================
        J += weights["w_v"] * (v - vdes) ** 2

        # ================= sforzo controllo =================
        J += weights["w_delta0"] * delta ** 2
        J += weights["w_u_a"] * a ** 2

        # ================= centratura corridoio =================
        #p = np.array([x[0], x[1]], dtype=float)
        #d_lat, half_w = corridor_lateral_coordinates(p, corridor)
        #J += weights["w_corr"] * (d_lat ** 2)

        # ================= regolarità =================
        du = uk - u_prev
        J += weights["w_du_delta"] * du[0] ** 2
        J += weights["w_du_a"] * du[1] ** 2

        u_prev = uk

    # ================= terminale posizione =================
    pN = np.array([x[0], x[1]], dtype=float)
    eN = pN - pref_nom
    J += weights["w_term"] * float(eN @ eN)

    # ================= terminale yaw =================
    #psi_err = x[2] - corridor["psiRef"]
    psi_err = x[2] - 0
   # psi_err = np.arctan2(np.sin(psi_err), np.cos(psi_err))
   # J += 2000* psi_err ** 2

    return float(J)


def planner_nonlcon_corridor(
    z: Sequence[float],
    x0: np.ndarray,
    last_u: np.ndarray,
    corridor: Dict,
    horizon: int,
    ts: float,
    params: Dict,
    limits: Dict,
    obstacles: List[Tuple[float, float, float]],
    dmin: float
) -> np.ndarray:
    z = np.asarray(z, dtype=float)
    x = np.asarray(x0, dtype=float).copy()

    c = []
    u_prev = np.asarray(last_u, dtype=float).copy()

    # rate constraints
    for k in range(horizon):
        uk = z[2 * k: 2 * k + 2]
        du = (uk - u_prev) / ts

        c.append(du[0] - limits["dDeltaMax"])
        c.append(-du[0] + limits["dDeltaMin"])
        c.append(du[1] - limits["dAMax"])
        c.append(-du[1] + limits["dAMin"])

        u_prev = uk

    # rollout constraints
    x = np.asarray(x0, dtype=float).copy()
    for k in range(horizon):
        uk = z[2 * k: 2 * k + 2]
        x = np.asarray(
            f1tenth_state_fcn_dt_beta(
                xk=x,
                uk=uk,
                ts=ts,
                wheelbase=params["L"],
                lr=params["lr"]
            ),
            dtype=float
        )

        # speed bounds
        v = float(x[3])
        c.append(v - limits["vMax"])
        c.append(-v + limits["vMin"])

        # ================= CORRIDOR HARD CONSTRAINT =================


        p = np.array([x[0], x[1]], dtype=float)
        d_lat, half_w = corridor_lateral_coordinates(p, corridor)

        # dentro il corridoio se abs(d_lat) <= half_w
        c.append(abs(d_lat) - half_w)

        # ================= OBSTACLE HARD CONSTRAINT =================
        #px = float(x[0])
        #py = float(x[1])
        #psi = float(x[2])

        #e = np.array([np.cos(psi), np.sin(psi)], dtype=float)

        #p_nose = np.array([px, py]) + (params["L"] / 2.0) * e

        #front_points = [
         #   p_nose + 0.25 * e,
        #]

        #d_robot_obs = 1e6
        #for pF in front_points:
        #    for ox, oy, r in obstacles:
        #        d = np.hypot(pF[0] - ox, pF[1] - oy) - r
        #        d_robot_obs = min(d_robot_obs, d)

        #c.append(dmin - d_robot_obs)

    return np.asarray(c, dtype=float)


def min_distance_to_obstacles(
    p: np.ndarray,
    obstacles: List[Tuple[float, float, float]]
) -> float:
    """
    Distanza firmata approssimata da ostacoli circolari:
    d = distanza_centro - raggio
    """
    if len(obstacles) == 0:
        return 1e6

    px = float(p[0])
    py = float(p[1])

    dmin = 1e6
    for ox, oy, r in obstacles:
        d = np.hypot(px - ox, py - oy) - r
        if d < dmin:
            dmin = d

    return float(dmin)


def corridor_lateral_coordinates(
    p: np.ndarray,
    corridor: Dict
) -> Tuple[float, float]:
    _, idx = nearest_centerline_index(p, corridor)

    pc = np.array([corridor["xc"][idx], corridor["yc"][idx]], dtype=float)
    n = np.array([corridor["nx"][idx], corridor["ny"][idx]], dtype=float)

    d_lat = float(n @ (p - pc))
    half_w = float(corridor["halfWidth"][idx])

    return d_lat, half_w


def nearest_centerline_index(
    p: np.ndarray,
    corridor: Dict
) -> Tuple[float, int]:
    dx = corridor["xc"] - p[0]
    dy = corridor["yc"] - p[1]
    d2 = dx ** 2 + dy ** 2
    idx = int(np.argmin(d2))
    return float(d2[idx]), idx
