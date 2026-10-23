//! The solver's cost pieces, operation for operation with `mpc_solver.py`:
//!
//! - [`linearize_obstacle`]: `_linearize_obstacle_cost_rti` with its
//!   `_d_front_batch` (seven poses, central differences) and
//!   `_softplus_sq_derivs`: the local quadratic model of the `w_obs` term;
//! - [`unwrapped_heading_target`], [`terminal_heading_target`],
//!   [`rate_weight_factor`]: the scalar helpers both the QP and the true cost
//!   use;
//! - [`planner_cost`]: `planner_cost_corridor` for `stage_spec None`, the
//!   true cost `_solve_rti` reports as `info['cost']`.
//!
//! Which operations are numpy and which CPython matters here (see
//! [`crate::pyfloat`]): `np.cos`/`np.exp`/`np.hypot` never raise, `math.sin`
//! of an infinity does; `pen ** 2` on a Python float is CPython's
//! `float_pow`, `(v - vdes) ** 2` on a numpy float64 is numpy's scalar power.
//! Both are libm `pow`, never `x * x`.

use crate::model::State;
use crate::npcompat::{blas_dot, dot2, min_reduce, np_pow, py_atan2, py_max, py_min};
use crate::problem::{CorridorView, RtiInput, SolveError};
use crate::pyfloat::{self, PyError};
use crate::query::{self, Frame, QueryError};

/// `_SOFTPLUS_BETA` / planner_cost_corridor's `softplus(beta=10.0)`.
const SOFTPLUS_BETA: f64 = 10.0;
/// `_linearize_obstacle_cost_rti`'s central-difference step.
const FD_EPS: f64 = 1e-4;

/// `_nearest_corridor_frame(p, corridor)` (plus the lateral offset and the
/// heading its two other callers take from the same index).
pub fn frame(c: &CorridorView, p: [f64; 2]) -> Result<Frame, SolveError> {
    query::corridor_frame(c.xc, c.yc, c.nx, c.ny, c.half_width, p).map_err(|e| match e {
        QueryError::Empty => SolveError::EmptyCorridor,
        QueryError::LengthMismatch => SolveError::Length,
    })
}

/// `_d_front_batch(X, Y, psi, L, obstacles_world)` for the seven poses of
/// one linearisation: per pose, the minimum over every probe point (nose
/// `base + L/2 e`, then `nose + offset e`) and every obstacle of
/// `hypot(point - centre) - r`, as `np.minimum.reduce` (NaN propagates).
fn d_front_batch(
    xs: &[f64; 7],
    ys: &[f64; 7],
    psis: &[f64; 7],
    wheelbase: f64,
    obstacles: &[[f64; 3]],
    offsets: &[f64],
) -> [f64; 7] {
    let half = wheelbase / 2.0;
    let mut out = [0.0; 7];
    let mut vals = Vec::with_capacity(offsets.len() * obstacles.len());
    for p in 0..7 {
        // e = np.stack([np.cos(psi), np.sin(psi)]): numpy, no exception
        let e = [psis[p].cos(), psis[p].sin()];
        let nose = [xs[p] + half * e[0], ys[p] + half * e[1]];
        vals.clear();
        for &off in offsets {
            let fx = nose[0] + off * e[0];
            let fy = nose[1] + off * e[1];
            for o in obstacles {
                vals.push((fx - o[0]).hypot(fy - o[1]) - o[2]);
            }
        }
        // numpy raises on an empty reduction; there is always >= 1 obstacle
        // here and SOLVER_FRONT_POINT_OFFSETS_M has three entries.
        out[p] = min_reduce(&vals).unwrap_or(f64::NAN);
    }
    out
}

/// `_softplus_sq_derivs(s)`: `g = softplus(s)^2` and its first two
/// derivatives, with the numerically stable sigmoid and softplus.
pub fn softplus_sq_derivs(s: f64) -> (f64, f64, f64) {
    let beta = SOFTPLUS_BETA;
    let bs = beta * s;
    let sig = if bs >= 0.0 {
        1.0 / (1.0 + (-bs).exp())
    } else {
        let e = bs.exp();
        e / (1.0 + e)
    };
    // np.log1p(np.exp(-abs(bs))) / beta + max(s, 0.0)
    let sp = (-(bs.abs())).exp().ln_1p() / beta + py_max(s, 0.0);
    let sp_prime = sig;
    let sp_second = (beta * sig) * (1.0 - sig);
    let g = sp * sp;
    let g_prime = (2.0 * sp) * sp_prime;
    let g_second = (2.0 * sp_prime) * sp_prime + (2.0 * sp) * sp_second;
    (g, g_prime, g_second)
}

/// `_linearize_obstacle_cost_rti(x_ref[:4], L, car_radius, margin,
/// obstacles_world)`: `(M, b)` with the term ~ `x^T M x + b . x`. Zeros
/// without obstacles.
pub fn linearize_obstacle(
    x_ref: &State,
    wheelbase: f64,
    car_radius: f64,
    avoidance_margin: f64,
    obstacles: &[[f64; 3]],
    offsets: &[f64],
) -> ([[f64; 4]; 4], [f64; 4]) {
    if obstacles.is_empty() {
        return ([[0.0; 4]; 4], [0.0; 4]);
    }
    let (x, y, psi) = (x_ref[0], x_ref[1], x_ref[2]);
    let eps = FD_EPS;
    let d = d_front_batch(
        &[x, x + eps, x - eps, x, x, x, x],
        &[y, y, y, y + eps, y - eps, y, y],
        &[psi, psi, psi, psi, psi, psi + eps, psi - eps],
        wheelbase,
        obstacles,
        offsets,
    );
    let two_eps = 2.0 * eps;
    let d_x = (d[1] - d[2]) / two_eps;
    let d_y = (d[3] - d[4]) / two_eps;
    let d_psi = (d[5] - d[6]) / two_eps;

    let s_ref = (car_radius + avoidance_margin) - d[0];
    let m = [-d_x, -d_y, -d_psi, 0.0];
    let (_g_ref, g1, g2) = softplus_sq_derivs(s_ref);
    // m_dot_c = float(m @ c), c = x_ref: cblas_ddot, n = 4
    let m_dot_c = blas_dot(&m, &x_ref[..4]);

    let mut mm = [[0.0; 4]; 4];
    let half_g2 = 0.5 * g2;
    for (i, row) in mm.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            // 0.5 * g2 * np.outer(m, m)
            *v = half_g2 * (m[i] * m[j]);
        }
    }
    let g2c = g2 * m_dot_c;
    // b = g1 * m - g2 * m_dot_c * m
    let b = [
        g1 * m[0] - g2c * m[0],
        g1 * m[1] - g2c * m[1],
        g1 * m[2] - g2c * m[2],
        g1 * m[3] - g2c * m[3],
    ];
    (mm, b)
}

/// `unwrapped_heading_target(psi_target, psi_linearization)`.
pub fn unwrapped_heading_target(psi_target: f64, psi_lin: f64) -> Result<f64, PyError> {
    let delta = py_atan2(
        pyfloat::sin(psi_target - psi_lin)?,
        pyfloat::cos(psi_target - psi_lin)?,
    );
    Ok(psi_lin + delta)
}

/// `terminal_heading_target(corridor, psi_ref, psi_ref_N)`: the shortest
/// branch without `psiRefTurn`, the signed rotation still owed with it.
pub fn terminal_heading_target(
    c: &CorridorView,
    psi_ref: f64,
    psi_ref_n: f64,
) -> Result<f64, PyError> {
    let Some(turn) = c.psi_ref_turn else {
        return unwrapped_heading_target(psi_ref, psi_ref_n);
    };
    let psi_start = c.psi_start.unwrap_or(psi_ref_n);
    let progress = py_atan2(
        pyfloat::sin(psi_ref_n - psi_start)?,
        pyfloat::cos(psi_ref_n - psi_start)?,
    );
    Ok(psi_ref_n + (turn - progress))
}

/// `rate_weight_factor(k, ts, control_period)`: 1 at stage 0, else
/// `(control_period / ts) ** 2` on Python floats.
pub fn rate_weight_factor(k: usize, ts: f64, control_period: f64) -> Result<f64, PyError> {
    if k == 0 {
        return Ok(1.0);
    }
    pyfloat::pow2(pyfloat::div(control_period, ts)?)
}

/// `planner_cost_corridor(z, x0, last_u, pref_nom, corridor, N, ts, params,
/// weights, vdes, control_period=t_first, stage_spec=None, speed_model)`.
///
/// `xs` is the rollout of `z` from `x0` (x_1..x_N): `planner_cost_corridor`
/// rolls the same model forward with the same controls as
/// `_rollout_true_dynamics`, so the states are the same numbers.
pub fn planner_cost(inp: &RtiInput, z: &[f64], xs: &[State]) -> Result<f64, SolveError> {
    let w = &inp.weights;
    let c = &inp.corridor;
    let half = inp.wheelbase / 2.0;
    let t_first = inp.t_first();
    let mut u_prev = inp.last_u;
    let mut j_cost = 0.0;
    let n = inp.horizon;
    if xs.len() < n || z.len() < 2 * n {
        return Err(SolveError::Length);
    }

    for k in 0..n {
        let uk = [z[2 * k], z[2 * k + 1]];
        let x = &xs[k];
        let v = x[3];
        let delta = uk[0];
        let a = uk[1];

        // obstacle: the front probe points, min over points and obstacles,
        // starting from 1e6 with Python's min()
        let psi = x[2];
        let e = [psi.cos(), psi.sin()];
        let nose = [x[0] + half * e[0], x[1] + half * e[1]];
        let mut d_front = 1e6;
        for &off in inp.front_offsets {
            let pf = [nose[0] + off * e[0], nose[1] + off * e[1]];
            for o in c.obstacles {
                let d = (pf[0] - o[0]).hypot(pf[1] - o[1]) - o[2];
                d_front = py_min(d_front, d);
            }
        }
        let s = (c.car_radius + c.avoidance_margin) - d_front;
        // softplus(s, beta=10.0) = float(np.log1p(np.exp(beta * s)) / beta)
        let pen = (SOFTPLUS_BETA * s).exp().ln_1p() / SOFTPLUS_BETA;
        j_cost += w.w_obs * pyfloat::pow2(pen)?;

        // speed, effort (numpy float64 powers)
        j_cost += w.w_v * np_pow(v - inp.vdes, 2.0);
        j_cost += w.w_delta0 * np_pow(delta, 2.0);
        j_cost += w.w_u_a * np_pow(a, 2.0);

        if w.w_corr != 0.0 {
            // corridor_lateral_coordinates: d_lat = float(n @ (p - pc))
            let d_lat = frame(c, [x[0], x[1]])?.d_lat;
            j_cost += w.w_corr * pyfloat::pow2(d_lat)?;
        }

        if let Some(line) = c.target_line {
            if w.w_line != 0.0 {
                // d_line = -sin(psi_line) * (x[0] - p0) + cos(psi_line) * (x[1] - p1)
                let d_line = (-pyfloat::sin(line.psi)?) * (x[0] - line.p[0])
                    + pyfloat::cos(line.psi)? * (x[1] - line.p[1]);
                j_cost += w.w_line * np_pow(d_line, 2.0);
            }
        }

        if w.w_psi_stage != 0.0 {
            let psi_stage_ref = frame(c, [x[0], x[1]])?.heading;
            let err = py_atan2(
                pyfloat::sin(x[2] - psi_stage_ref)?,
                pyfloat::cos(x[2] - psi_stage_ref)?,
            );
            j_cost += w.w_psi_stage * pyfloat::pow2(err)?;
        }

        // rate
        let du = [uk[0] - u_prev[0], uk[1] - u_prev[1]];
        let rate_w = rate_weight_factor(k, inp.ts, t_first)?;
        j_cost += (rate_w * w.w_du_delta) * np_pow(du[0], 2.0);
        j_cost += (rate_w * w.w_du_a) * np_pow(du[1], 2.0);
        u_prev = uk;
    }

    // terminal position: float(eN @ eN)
    let x = &xs[n - 1];
    let e_n = [x[0] - inp.pref_nom[0], x[1] - inp.pref_nom[1]];
    j_cost += w.w_term * dot2(e_n, e_n);

    // terminal yaw
    if w.w_psi != 0.0 {
        let psi_ref = c.psi_ref.ok_or(SolveError::MissingPsiRef)?;
        let psi_err = x[2] - terminal_heading_target(c, psi_ref, x[2])?;
        j_cost += w.w_psi * pyfloat::pow2(psi_err)?;
    }
    Ok(j_cost)
}

#[cfg(test)]
#[allow(clippy::needless_range_loop)]
mod tests {
    use super::*;

    #[test]
    fn the_softplus_derivatives_match_finite_differences() {
        for &s in &[-0.5, -0.05, 0.0, 0.03, 0.4] {
            let (g, g1, g2) = softplus_sq_derivs(s);
            let h = 1e-6;
            let (gp, g1p, _) = softplus_sq_derivs(s + h);
            let (gm, g1m, _) = softplus_sq_derivs(s - h);
            assert!(((gp - gm) / (2.0 * h) - g1).abs() < 1e-6, "{s}");
            assert!(((g1p - g1m) / (2.0 * h) - g2).abs() < 1e-4, "{s}");
            assert!(g >= 0.0 && g2 >= 0.0);
        }
    }

    #[test]
    fn no_obstacle_no_term_and_the_model_is_psd() {
        let x = [0.0, 0.0, 0.0, 0.5, 0.0];
        let (m, b) = linearize_obstacle(&x, 0.33, 0.2, 0.12, &[], &[0.1, 0.25, 0.4]);
        assert_eq!(m, [[0.0; 4]; 4]);
        assert_eq!(b, [0.0; 4]);
        let (m, _b) =
            linearize_obstacle(&x, 0.33, 0.2, 0.12, &[[1.0, 0.1, 0.18]], &[0.1, 0.25, 0.4]);
        // outer product: symmetric, non-negative diagonal, zero speed row
        for i in 0..4 {
            assert!(m[i][i] >= 0.0);
            assert_eq!(m[3][i], 0.0);
            for j in 0..4 {
                assert_eq!(m[i][j], m[j][i]);
            }
        }
    }

    #[test]
    fn the_rate_factor_is_one_at_stage_zero() {
        assert_eq!(rate_weight_factor(0, 0.2, 0.1), Ok(1.0));
        assert_eq!(rate_weight_factor(3, 0.2, 0.1), pyfloat::pow2(0.1 / 0.2));
        assert_eq!(rate_weight_factor(1, 0.0, 0.1), Err(PyError::ZeroDivision));
    }
}
