//! The RTI QP of `mpc_solver._solve_rti` (stage_spec None), built exactly as
//! osqp-python hands it to OSQP C.
//!
//! WHY BIT-IDENTICAL AND NOT 1e-14. About half the closed-loop solves stop
//! at OSQP's iteration limit or "solved inaccurate" (rti_profile.md): a
//! non-converged ADMM iterate follows every rounding difference in the data,
//! so the QP has to be the same bits, and the same sparsity pattern (a
//! different pattern is a different factorisation). That fixes how it is
//! built:
//!
//! - `P` is accumulated DENSE, term by term, in the Python's order: the
//!   same `+=` on the same entries in the same sequence gives the same
//!   rounding. Each term adds its whole numpy block (zeros included), so a
//!   NaN or inf weight spreads the way `inf * 0.0` does in numpy;
//! - `0.5 * (P + P.T)`, then `scipy.sparse.csc_matrix` (drops exact zeros of
//!   either sign, keeps NaN, sorted row indices), then osqp-python's upper
//!   triangle (`scipy.sparse.triu`);
//! - `A` row by row (each row a fresh `np.zeros(n_z)` with entries assigned
//!   once), converted the same way; `l`/`u` as built, then clamped with
//!   `np.maximum(l, -OSQP_INFTY)` / `np.minimum(u, OSQP_INFTY)`;
//! - every small matrix product is numpy's BLAS call in reference-BLAS order
//!   ([`crate::npcompat::blas_gemv`]).
//!
//! Layout `z = [u_0..u_{N-1}, x_1..x_N, s_0..s_{M-1}, (speed-cap slack)]`.

// Index loops on purpose: each `for i .. for j` spells out the numpy block
// operation it reproduces, entry for entry and in numpy's order.
#![allow(clippy::needless_range_loop)]

use crate::cost::{self, linearize_obstacle, rate_weight_factor, terminal_heading_target};
use crate::model::{self, Jacobian, LagParams, State, MAX_NX};
use crate::npcompat::{blas_gemv, dot2, np_maximum, np_minimum, py_max};
use crate::problem::{RtiInput, SolveError};
use crate::pyfloat;

/// `OSQP_INFTY` (OSQP C, double precision).
pub const OSQP_INFTY: f64 = 1e30;

/// `BOUNDARY_DISABLED`: an absent slot, an all-zero row.
pub const BOUNDARY_DISABLED: [f64; 3] = [0.0, 0.0, f64::INFINITY];

/// Largest QP the core builds (variables). `P` is accumulated dense, so the
/// memory is `8 n_z^2` bytes (288 MB here); a failed Rust allocation would
/// abort the process where numpy raises MemoryError. The MPC builds 120-145
/// (N = 20); 6000 allows a horizon of ~850 stages.
pub const MAX_VARIABLES: usize = 6000;

/// Number of controls per stage.
const NU: usize = 2;

/// Where each block of `z` starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Layout {
    /// The horizon `N`.
    pub horizon: usize,
    /// The state dimension (4, or 5 with a speed model).
    pub nx: usize,
    /// Boundary slacks: `boundary_max_sources` when soft rows are built.
    pub n_slack: usize,
    /// 1 when the speed model has a soft real-speed cap.
    pub n_vs: usize,
    /// Number of variables.
    pub n_z: usize,
}

impl Layout {
    /// `u_idx(k)`: the first entry of `u_k`.
    pub fn u(&self, k: usize) -> usize {
        k * NU
    }

    /// `x_idx(k)`, k = 1..N: the first entry of `x_k`.
    pub fn x(&self, k: usize) -> usize {
        self.horizon * NU + (k - 1) * self.nx
    }

    /// `s_idx(j)`.
    pub fn s(&self, j: usize) -> usize {
        self.horizon * (NU + self.nx) + j
    }

    /// `vs_idx`, the speed-cap slack.
    pub fn vs(&self) -> usize {
        self.horizon * (NU + self.nx) + self.n_slack
    }
}

/// A CSC matrix as osqp-python passes it: column pointers, sorted row
/// indices, values (`int64` here; OSQP C takes `OSQPInt`).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Csc {
    pub nrows: usize,
    pub ncols: usize,
    pub colptr: Vec<i64>,
    pub rowind: Vec<i64>,
    pub values: Vec<f64>,
}

impl Csc {
    /// Number of stored entries.
    pub fn nnz(&self) -> usize {
        self.values.len()
    }
}

/// One tick's QP, as OSQP C receives it, with what the solve needs after.
#[derive(Clone, Debug)]
pub struct Qp {
    pub layout: Layout,
    /// Upper triangle of `0.5 (P + P^T)`, zeros dropped.
    pub p: Csc,
    pub q: Vec<f64>,
    pub a: Csc,
    /// `l` and `u`, clamped to `+-OSQP_INFTY`.
    pub l: Vec<f64>,
    pub u: Vec<f64>,
    /// The warm start handed to `osqp_warm_start` (x only).
    pub z_guess: Vec<f64>,
    /// The reference trajectory `x_ref[0..=N]`.
    pub x_ref: Vec<State>,
    /// The warm-start controls actually used (`warm_start_z` or the tile).
    pub warm: Vec<f64>,
}

/// A dense row-major square matrix accumulated with `+=`.
struct Dense {
    n: usize,
    v: Vec<f64>,
}

impl Dense {
    fn new(n: usize) -> Self {
        Dense {
            n,
            v: vec![0.0; n * n],
        }
    }

    #[inline]
    fn add(&mut self, i: usize, j: usize, val: f64) {
        self.v[i * self.n + j] += val;
    }

    #[inline]
    fn get(&self, i: usize, j: usize) -> f64 {
        self.v[i * self.n + j]
    }
}

/// The constraint rows: entries as assigned into each fresh zero row.
#[derive(Default)]
struct Rows {
    /// (row, column, value) of every entry that is not an exact zero.
    entries: Vec<(usize, usize, f64)>,
    lo: Vec<f64>,
    hi: Vec<f64>,
}

impl Rows {
    /// `add_row(row, lo, hi)`: `cols` are the row's assigned entries (any
    /// order, no duplicates); exact zeros are what `csc_matrix` drops.
    fn add(&mut self, cols: &[(usize, f64)], lo: f64, hi: f64) {
        let r = self.lo.len();
        for &(c, v) in cols {
            if v != 0.0 {
                self.entries.push((r, c, v));
            }
        }
        self.lo.push(lo);
        self.hi.push(hi);
    }

    /// `scipy.sparse.csc_matrix(np.array(rows))`: column-major, rows sorted.
    fn to_csc(&self, ncols: usize) -> Csc {
        let mut count = vec![0_usize; ncols + 1];
        for &(_r, c, _v) in &self.entries {
            count[c + 1] += 1;
        }
        for c in 0..ncols {
            count[c + 1] += count[c];
        }
        let colptr: Vec<i64> = count.iter().map(|&v| v as i64).collect();
        let mut next = count;
        let nnz = self.entries.len();
        let mut rowind = vec![0_i64; nnz];
        let mut values = vec![0.0; nnz];
        // entries are in row order, so a stable fill keeps rows sorted
        for &(r, c, v) in &self.entries {
            let at = next[c];
            rowind[at] = r as i64;
            values[at] = v;
            next[c] += 1;
        }
        Csc {
            nrows: self.lo.len(),
            ncols,
            colptr,
            rowind,
            values,
        }
    }
}

/// `scipy.sparse.triu(csc_matrix(0.5 * (P + P.T)))`: the upper triangle of
/// the symmetrised dense `P`, zeros dropped, rows sorted.
fn sym_upper_csc(p: &Dense) -> Csc {
    let n = p.n;
    let mut colptr = Vec::with_capacity(n + 1);
    let mut rowind = Vec::new();
    let mut values = Vec::new();
    colptr.push(0_i64);
    for j in 0..n {
        for i in 0..=j {
            // (P + P.T)[i, j] = P[i, j] + P[j, i], then * 0.5
            let v = 0.5 * (p.get(i, j) + p.get(j, i));
            if v != 0.0 {
                rowind.push(i as i64);
                values.push(v);
            }
        }
        colptr.push(rowind.len() as i64);
    }
    Csc {
        nrows: n,
        ncols: n,
        colptr,
        rowind,
        values,
    }
}

/// The stage-k `nx x nx` diagonal block `P[x, x] += c * B` for an outer
/// product `B = v v^T` (all entries, numpy's `c * np.outer(v, v)`).
fn add_outer(p: &mut Dense, base: usize, nx: usize, c: f64, v: &[f64; MAX_NX]) {
    for i in 0..nx {
        for j in 0..nx {
            p.add(base + i, base + j, c * (v[i] * v[j]));
        }
    }
}

/// Builds the tick's QP: `_solve_rti` up to `prob.setup(...)`.
pub fn build(inp: &RtiInput) -> Result<Qp, SolveError> {
    let n = inp.horizon;
    let nx = inp.nx();
    if inp.x0.len() != nx {
        return Err(SolveError::X0Size {
            got: inp.x0.len(),
            want: nx,
        });
    }
    if n < 1 {
        return Err(SolveError::Horizon);
    }
    let lag: Option<LagParams> = inp.speed_model.map(|sm| sm.lag);
    if let Some(sm) = &inp.speed_model {
        if sm.c_lo.len() < n || sm.c_hi.len() < n || sm.v_hi.is_some_and(|v| v.len() < n) {
            return Err(SolveError::Length);
        }
    }
    let t_first = inp.t_first();
    let (ts, wb, lr) = (inp.ts, inp.wheelbase, inp.lr);

    // warm start: wrong length or non-finite -> last_u tiled
    let warm: Vec<f64> = match inp.warm_start {
        Some(w) if w.len() == n * NU && w.iter().all(|v| v.is_finite()) => w.to_vec(),
        _ => (0..n).flat_map(|_| inp.last_u).collect(),
    };

    // ---- 1. reference trajectory through the true model
    let mut x0: State = [0.0; MAX_NX];
    x0[..nx].copy_from_slice(inp.x0);
    let mut x_ref: Vec<State> = Vec::with_capacity(n + 1);
    x_ref.push(x0);
    for k in 0..n {
        let uk = [warm[2 * k], warm[2 * k + 1]];
        let next = model::step(&x_ref[k], uk, ts, wb, lr, lag.as_ref())?;
        x_ref.push(next);
    }

    // ---- 2. linearised dynamics: c_k = f_ref - A_k x_ref_k - B_k u_ref_k
    // (f_ref is _model_step again at the same point: x_ref[k + 1])
    let mut jac: Vec<Jacobian> = Vec::with_capacity(n);
    let mut c_aff: Vec<State> = Vec::with_capacity(n);
    for k in 0..n {
        let uk = [warm[2 * k], warm[2 * k + 1]];
        let j = model::jacobian(&x_ref[k], uk, ts, wb, lr, lag.as_ref())?;
        let (ax, bu) = (mat_vec(&j.a, nx, &x_ref[k]), mat_vec_b(&j.b, nx, uk));
        let mut c = [0.0; MAX_NX];
        for r in 0..nx {
            c[r] = (x_ref[k + 1][r] - ax[r]) - bu[r];
        }
        jac.push(j);
        c_aff.push(c);
    }

    // ---- 3. layout
    let bnd = &inp.boundaries;
    let n_slack = if bnd.use_slack { bnd.max_sources } else { 0 };
    let n_vs = usize::from(inp.speed_model.is_some_and(|sm| sm.v_hi.is_some()));
    let n_z = n * (NU + nx) + n_slack + n_vs;
    if n_z > MAX_VARIABLES {
        return Err(SolveError::TooLarge);
    }
    let lay = Layout {
        horizon: n,
        nx,
        n_slack,
        n_vs,
        n_z,
    };
    let mut p = Dense::new(n_z);
    let mut q = vec![0.0; n_z];
    let mut rows = Rows::default();
    let w = &inp.weights;
    let lim = &inp.limits;
    let cor = &inp.corridor;

    // ---- dynamics: x_{k+1} - A_k x_k - B_k u_k = c_k (x_0 folded into the rhs)
    for k in 0..n {
        let rhs: State = if k == 0 {
            let ax0 = mat_vec(&jac[0].a, nx, &x_ref[0]);
            let mut r = [0.0; MAX_NX];
            for i in 0..nx {
                r[i] = c_aff[0][i] + ax0[i];
            }
            r
        } else {
            c_aff[k]
        };
        let u = lay.u(k);
        let x_next = lay.x(k + 1);
        let mut cols: Vec<(usize, f64)> = Vec::with_capacity(2 + 2 * nx);
        for r in 0..nx {
            cols.clear();
            cols.push((u, -jac[k].b[r][0]));
            cols.push((u + 1, -jac[k].b[r][1]));
            if k > 0 {
                let x_prev = lay.x(k);
                for c in 0..nx {
                    cols.push((x_prev + c, -jac[k].a[r][c]));
                }
            }
            for c in 0..nx {
                cols.push((x_next + c, if c == r { 1.0 } else { 0.0 }));
            }
            rows.add(&cols, rhs[r], rhs[r]);
        }
    }

    // ---- control box + rate rows
    for k in 0..n {
        let u = lay.u(k);
        rows.add(&[(u, 1.0), (u + 1, 0.0)], lim.delta_min, lim.delta_max);
        rows.add(&[(u, 0.0), (u + 1, 1.0)], lim.a_min, lim.a_max);

        let t_step = if k == 0 { t_first } else { ts };
        let mut lo_d = lim.d_delta_min * t_step;
        let mut hi_d = lim.d_delta_max * t_step;
        let mut lo_a = lim.d_a_min * t_step;
        let mut hi_a = lim.d_a_max * t_step;
        if k > 0 {
            let up = lay.u(k - 1);
            rows.add(
                &[(u, 1.0), (u + 1, 0.0), (up, -1.0), (up + 1, 0.0)],
                lo_d,
                hi_d,
            );
            rows.add(
                &[(u, 0.0), (u + 1, 1.0), (up, 0.0), (up + 1, -1.0)],
                lo_a,
                hi_a,
            );
        } else {
            lo_d += inp.last_u[0];
            hi_d += inp.last_u[0];
            rows.add(&[(u, 1.0), (u + 1, 0.0)], lo_d, hi_d);
            lo_a += inp.last_u[1];
            hi_a += inp.last_u[1];
            rows.add(&[(u, 0.0), (u + 1, 1.0)], lo_a, hi_a);
        }
    }

    // ---- per-stage terms at x_ref[k + 1]
    let mut e_v = [0.0; MAX_NX];
    e_v[3] = 1.0;
    for k in 0..n {
        let xk1 = lay.x(k + 1);
        let xr = &x_ref[k + 1];
        let u = lay.u(k);

        // velocity tracking: w_v (v - vdes)^2
        add_outer(&mut p, xk1, nx, 2.0 * w.w_v, &e_v);
        let cv = (-2.0 * w.w_v) * inp.vdes;
        for i in 0..nx {
            q[xk1 + i] += cv * e_v[i];
        }

        // control effort: 2 w_delta0 [[1,0],[0,0]], then 2 w_u_a [[0,0],[0,1]]
        for (c, diag) in [(2.0 * w.w_delta0, [1.0, 0.0]), (2.0 * w.w_u_a, [0.0, 1.0])] {
            for i in 0..2 {
                for j in 0..2 {
                    let b = if i == j { diag[i] } else { 0.0 };
                    p.add(u + i, u + j, c * b);
                }
            }
        }

        // rate smoothness: Wdu = f * diag(w_du_delta, w_du_a)
        let f = rate_weight_factor(k, ts, t_first)?;
        let wdu = [[f * w.w_du_delta, f * 0.0], [f * 0.0, f * w.w_du_a]];
        for i in 0..2 {
            for j in 0..2 {
                p.add(u + i, u + j, 2.0 * wdu[i][j]);
            }
        }
        if k == 0 {
            // q[u] += (-2.0 * Wdu) @ last_u
            let m2 = [
                -2.0 * wdu[0][0],
                -2.0 * wdu[0][1],
                -2.0 * wdu[1][0],
                -2.0 * wdu[1][1],
            ];
            let g = blas_gemv(&m2, 2, 2, &inp.last_u);
            q[u] += g[0];
            q[u + 1] += g[1];
        } else {
            let up = lay.u(k - 1);
            for i in 0..2 {
                for j in 0..2 {
                    p.add(u + i, up + j, -2.0 * wdu[i][j]);
                }
            }
            for i in 0..2 {
                for j in 0..2 {
                    p.add(up + i, u + j, -2.0 * wdu[i][j]);
                }
            }
            for i in 0..2 {
                for j in 0..2 {
                    p.add(up + i, up + j, 2.0 * wdu[i][j]);
                }
            }
        }

        // obstacle soft cost: local quadratic model, padded to nx
        let (m_obs, b_obs) = linearize_obstacle(
            xr,
            wb,
            cor.car_radius,
            cor.avoidance_margin,
            cor.obstacles,
            inp.front_offsets,
        );
        let co = 2.0 * w.w_obs;
        for i in 0..nx {
            for j in 0..nx {
                let mij = if i < 4 && j < 4 { m_obs[i][j] } else { 0.0 };
                p.add(xk1 + i, xk1 + j, co * mij);
            }
        }
        for i in 0..nx {
            let bi = if i < 4 { b_obs[i] } else { 0.0 };
            q[xk1 + i] += w.w_obs * bi;
        }

        // corridor bound at the frozen nearest index
        let fr = cost::frame(cor, [xr[0], xr[1]])?;
        let n_dot_pc = dot2(fr.n, fr.pc);
        rows.add(
            &[(xk1, fr.n[0]), (xk1 + 1, fr.n[1])],
            n_dot_pc - fr.half_w,
            n_dot_pc + fr.half_w,
        );

        // corridor centring: w_corr (n . (p - pc))^2
        if w.w_corr != 0.0 {
            let mut n_ext = [0.0; MAX_NX];
            n_ext[0] = fr.n[0];
            n_ext[1] = fr.n[1];
            add_outer(&mut p, xk1, nx, 2.0 * w.w_corr, &n_ext);
            let cq = -2.0 * n_dot_pc;
            for i in 0..nx {
                q[xk1 + i] += w.w_corr * (cq * n_ext[i]);
            }
        }

        // target line: w_line (n_line . (p - p_line))^2
        if let Some(line) = cor.target_line {
            if w.w_line != 0.0 {
                let n_line = [-pyfloat::sin(line.psi)?, pyfloat::cos(line.psi)?];
                let n_dot_pl = dot2(n_line, line.p);
                let mut nl_ext = [0.0; MAX_NX];
                nl_ext[0] = n_line[0];
                nl_ext[1] = n_line[1];
                add_outer(&mut p, xk1, nx, 2.0 * w.w_line, &nl_ext);
                let cq = -2.0 * n_dot_pl;
                for i in 0..nx {
                    q[xk1 + i] += w.w_line * (cq * nl_ext[i]);
                }
            }
        }

        // stage heading: w_psi_stage (psi - psi_corridor)^2
        if w.w_psi_stage != 0.0 {
            let target = cost::unwrapped_heading_target(fr.heading, xr[2])?;
            p.add(xk1 + 2, xk1 + 2, 2.0 * w.w_psi_stage);
            q[xk1 + 2] += w.w_psi_stage * (-2.0 * target);
        }

        // speed: legacy box on v_{k+1}; offset-free hard box on c_{k+1} and
        // the soft real-speed cap v_{k+1} - s <= v_hi_k
        if let Some(sm) = &inp.speed_model {
            rows.add(&[(xk1 + 4, 1.0)], sm.c_lo[k], sm.c_hi[k]);
            if let Some(v_hi) = sm.v_hi {
                rows.add(
                    &[(xk1 + 3, 1.0), (lay.vs(), -1.0)],
                    f64::NEG_INFINITY,
                    v_hi[k],
                );
            }
        } else {
            rows.add(&[(xk1 + 3, 1.0)], lim.v_min, lim.v_max);
        }

        // boundary rows: always max_sources, padded with BOUNDARY_DISABLED
        for j in 0..bnd.max_sources {
            let [bnx, bny, boff] = bnd.rows.get(j).copied().unwrap_or(BOUNDARY_DISABLED);
            let hi = (boff - cor.car_radius) - cor.avoidance_margin;
            if bnd.use_slack {
                rows.add(
                    &[(xk1, bnx), (xk1 + 1, bny), (lay.s(j), -1.0)],
                    f64::NEG_INFINITY,
                    hi,
                );
            } else {
                rows.add(&[(xk1, bnx), (xk1 + 1, bny)], f64::NEG_INFINITY, hi);
            }
        }
    }

    // ---- boundary slack: 0 <= s_j <= slack_max, w (s^2 + s)
    if bnd.use_slack {
        for j in 0..n_slack {
            let s = lay.s(j);
            rows.add(&[(s, 1.0)], 0.0, bnd.slack_max);
            p.add(s, s, 2.0 * bnd.slack_weight);
            q[s] += bnd.slack_weight;
        }
    }

    // ---- speed-cap slack: s >= 0, w (s^2 + s)
    if n_vs == 1 {
        if let Some(sm) = &inp.speed_model {
            let vs = lay.vs();
            rows.add(&[(vs, 1.0)], 0.0, f64::INFINITY);
            p.add(vs, vs, 2.0 * sm.slack_weight);
            q[vs] += sm.slack_weight;
        }
    }

    // ---- terminal position: w_term |x_N[:2] - pref_nom|^2
    let x_n = lay.x(n);
    let ct = 2.0 * w.w_term;
    for i in 0..nx {
        for j in 0..nx {
            // E.T @ E: exactly 1.0 on (0, 0) and (1, 1), +0.0 elsewhere
            let ete = if i == j && i < 2 { 1.0 } else { 0.0 };
            p.add(x_n + i, x_n + j, ct * ete);
        }
    }
    // E.T @ pref_nom: dgemv('N'), y_i = (0 + p0 E[0][i]) + p1 E[1][i]
    let mut e_t = [0.0; 2 * MAX_NX];
    e_t[0] = 1.0; // E.T[0][0]
    e_t[3] = 1.0; // E.T[1][1]
    let etp = blas_gemv(&e_t[..2 * nx], nx, 2, &inp.pref_nom);
    for i in 0..nx {
        q[x_n + i] += w.w_term * (-2.0 * etp[i]);
    }

    // ---- terminal yaw: w_psi (psi_N - psi_target)^2
    if w.w_psi != 0.0 {
        let psi_ref_n = x_ref[n][2];
        let psi_ref = cor.psi_ref.unwrap_or(psi_ref_n);
        let target = terminal_heading_target(cor, psi_ref, psi_ref_n)?;
        p.add(x_n + 2, x_n + 2, 2.0 * w.w_psi);
        q[x_n + 2] += w.w_psi * (-2.0 * target);
    }

    // ---- assemble as osqp-python does
    let p_csc = sym_upper_csc(&p);
    let a_csc = rows.to_csc(n_z);
    let l: Vec<f64> = rows
        .lo
        .iter()
        .map(|&v| np_maximum(v, -OSQP_INFTY))
        .collect();
    let u: Vec<f64> = rows.hi.iter().map(|&v| np_minimum(v, OSQP_INFTY)).collect();

    // z_guess = [warm, x_ref[1:], zeros(n_slack), (speed-cap slack guess)]
    let mut z_guess = Vec::with_capacity(n_z);
    z_guess.extend_from_slice(&warm);
    for xr in &x_ref[1..] {
        z_guess.extend_from_slice(&xr[..nx]);
    }
    z_guess.extend(std::iter::repeat_n(0.0, n_slack));
    if n_vs == 1 {
        if let Some(v_hi) = inp.speed_model.and_then(|sm| sm.v_hi) {
            // max(0.0, max(float(x_ref[k + 1][3]) - v_hi[k] for k in range(N)))
            let mut worst = x_ref[1][3] - v_hi[0];
            for k in 1..n {
                worst = py_max(worst, x_ref[k + 1][3] - v_hi[k]);
            }
            z_guess.push(py_max(0.0, worst));
        }
    }

    Ok(Qp {
        layout: lay,
        p: p_csc,
        q,
        a: a_csc,
        l,
        u,
        z_guess,
        x_ref,
        warm,
    })
}

/// `A @ x` for the padded state Jacobian (`nx x nx`), numpy/BLAS order.
fn mat_vec(a: &[[f64; MAX_NX]; MAX_NX], nx: usize, x: &State) -> State {
    let mut flat = [0.0; MAX_NX * MAX_NX];
    for r in 0..nx {
        flat[r * nx..r * nx + nx].copy_from_slice(&a[r][..nx]);
    }
    let y = blas_gemv(&flat[..nx * nx], nx, nx, &x[..nx]);
    let mut out = [0.0; MAX_NX];
    out[..nx].copy_from_slice(&y);
    out
}

/// `B @ u` for the padded control Jacobian (`nx x 2`), numpy/BLAS order.
fn mat_vec_b(b: &[[f64; 2]; MAX_NX], nx: usize, u: [f64; 2]) -> State {
    let mut flat = [0.0; 2 * MAX_NX];
    for r in 0..nx {
        flat[2 * r] = b[r][0];
        flat[2 * r + 1] = b[r][1];
    }
    let y = blas_gemv(&flat[..2 * nx], nx, 2, &u);
    let mut out = [0.0; MAX_NX];
    out[..nx].copy_from_slice(&y);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::problem::{Boundaries, CorridorView, Limits, Weights};

    /// xc, yc, nx, ny, halfWidth of a straight corridor along +x.
    type CorridorArrays = (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>);

    fn straight_corridor(n: usize) -> CorridorArrays {
        let xc: Vec<f64> = (0..n).map(|i| i as f64 * 0.05).collect();
        (xc, vec![0.0; n], vec![-0.0; n], vec![1.0; n], vec![0.5; n])
    }

    #[test]
    fn the_layout_and_sizes_follow_the_python() {
        let (xc, yc, nxs, nys, hw) = straight_corridor(60);
        let obstacles = [[1.5, 0.1, 0.18]];
        let rows = [[0.0, 1.0, 0.9]];
        let inp = RtiInput {
            x0: &[0.0, 0.0, 0.0, 0.3],
            last_u: [0.0, 0.0],
            pref_nom: [2.0, 0.0],
            horizon: 5,
            ts: 0.1,
            control_period: None,
            wheelbase: 0.33,
            lr: 0.165,
            limits: Limits {
                delta_min: -0.4,
                delta_max: 0.4,
                a_min: -2.0,
                a_max: 2.0,
                d_delta_min: -3.0,
                d_delta_max: 3.0,
                d_a_min: -5.0,
                d_a_max: 5.0,
                v_min: 0.0,
                v_max: 1.0,
            },
            weights: Weights {
                w_v: 1.0,
                w_delta0: 0.1,
                w_u_a: 0.1,
                w_du_delta: 1.0,
                w_du_a: 0.5,
                w_obs: 5.0,
                w_corr: 0.5,
                w_line: 0.0,
                w_psi_stage: 0.2,
                w_term: 3.0,
                w_psi: 1.0,
            },
            vdes: 0.5,
            front_offsets: &[0.1, 0.25, 0.4],
            corridor: CorridorView {
                xc: &xc,
                yc: &yc,
                nx: &nxs,
                ny: &nys,
                half_width: &hw,
                psi_ref: Some(0.0),
                psi_ref_turn: None,
                psi_start: None,
                target_line: None,
                car_radius: 0.2,
                avoidance_margin: 0.12,
                obstacles: &obstacles,
            },
            boundaries: Boundaries {
                rows: &rows,
                max_sources: 3,
                use_slack: true,
                slack_weight: 1e4,
                slack_max: 0.5,
            },
            warm_start: None,
            speed_model: None,
        };
        let qp = build(&inp).unwrap();
        // 5 stages x (2 + 4) + 3 slacks
        assert_eq!(qp.layout.n_z, 33);
        // rows: 4 dynamics + 4 control + (corridor, speed, 3 boundary) per stage, 3 slack
        assert_eq!(qp.l.len(), 5 * 4 + 5 * 4 + 5 * 5 + 3);
        assert_eq!(qp.z_guess.len(), 33);
        // upper triangle only, rows sorted within each column
        for j in 0..qp.p.ncols {
            let (a, b) = (qp.p.colptr[j] as usize, qp.p.colptr[j + 1] as usize);
            for w in qp.p.rowind[a..b].windows(2) {
                assert!(w[0] < w[1]);
            }
            assert!(qp.p.rowind[a..b].iter().all(|&i| i as usize <= j));
        }
        // the disabled slots clamp to OSQP_INFTY
        assert!(qp.u.contains(&OSQP_INFTY));
        assert!(qp.l.contains(&-OSQP_INFTY));
        assert!(qp.a.values.iter().all(|&v| v != 0.0));
        // a QP beyond MAX_VARIABLES is refused, not allocated
        let huge = RtiInput {
            horizon: 1001,
            ..inp
        };
        assert_eq!(build(&huge).err(), Some(SolveError::TooLarge));
        let slots = RtiInput {
            boundaries: Boundaries {
                max_sources: MAX_VARIABLES,
                ..inp.boundaries
            },
            ..inp
        };
        assert_eq!(build(&slots).err(), Some(SolveError::TooLarge));
    }
}
