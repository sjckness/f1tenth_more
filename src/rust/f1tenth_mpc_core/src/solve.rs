//! One RTI tick: `mpc_solver._solve_rti` (stage_spec None) end to end.
//!
//! [`solve_rti`] builds the QP ([`crate::qp`]), solves it with OSQP C
//! v1.0.0 (osqp-sys 1.0.1, [`crate::ffi::osqp`]) in a FRESH workspace, as the
//! Python does every tick (output/rust/mpc/rti_profile.md: a persistent
//! workspace changes the numerics), rolls the chosen controls through the
//! true model and evaluates the true cost ([`crate::cost::planner_cost`]).
//!
//! OSQP SETTINGS. [`PYTHON_SETTINGS`] sets every `OSQPSettings` field
//! explicitly, to the values the Python path runs with: `osqp.OSQP().setup(
//! P, q, A, l, u, warm_starting=True, verbose=False, polishing=True,
//! max_iter=200)` on top of `osqp_set_default_settings` (the dump of
//! `prob.settings` in rti_profile.md; the fixture header carries it too, and
//! the parity harness compares the two field by field). `allocate_solution`
//! and `profiler_level` are not exposed by osqp-python; they are C's
//! defaults (1, 0). The warm start is x only (`warm_start(x=z_guess)`),
//! so y starts at 0.
//!
//! RESULT. As `_solve_rti`: an infeasibility certificate (status 3-6) or a
//! non-finite x means `success` False and the warm start `z_guess` held.

use crate::cost::planner_cost;
use crate::ffi::osqp as c_osqp;
use crate::model::{self, LagParams, State};
use crate::problem::{RtiInput, SolveError};
use crate::qp::{self, Csc, Qp};

/// Every `OSQPSettings` field, as plain integers / floats.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OsqpSettings {
    pub device: i64,
    pub linsys_solver: i64,
    pub allocate_solution: i64,
    pub verbose: i64,
    pub profiler_level: i64,
    pub warm_starting: i64,
    pub scaling: i64,
    pub polishing: i64,
    pub rho: f64,
    pub rho_is_vec: i64,
    pub sigma: f64,
    pub alpha: f64,
    pub cg_max_iter: i64,
    pub cg_tol_reduction: i64,
    pub cg_tol_fraction: f64,
    pub cg_precond: i64,
    pub adaptive_rho: i64,
    pub adaptive_rho_interval: i64,
    pub adaptive_rho_fraction: f64,
    pub adaptive_rho_tolerance: f64,
    pub max_iter: i64,
    pub eps_abs: f64,
    pub eps_rel: f64,
    pub eps_prim_inf: f64,
    pub eps_dual_inf: f64,
    pub scaled_termination: i64,
    pub check_termination: i64,
    pub check_dualgap: i64,
    pub time_limit: f64,
    pub delta: f64,
    pub polish_refine_iter: i64,
}

/// The Python path's effective OSQP settings (rti_profile.md, "OSQP
/// settings the Python path runs with"), every field.
pub const PYTHON_SETTINGS: OsqpSettings = OsqpSettings {
    device: 0,
    linsys_solver: 1, // OSQP_DIRECT_SOLVER (QDLDL)
    allocate_solution: 1,
    verbose: 0,
    profiler_level: 0,
    warm_starting: 1,
    scaling: 10,
    polishing: 1,
    rho: 0.1,
    rho_is_vec: 1,
    sigma: 1e-06,
    alpha: 1.6,
    cg_max_iter: 20,
    cg_tol_reduction: 10,
    cg_tol_fraction: 0.15,
    cg_precond: 1,   // OSQP_DIAGONAL_PRECONDITIONER
    adaptive_rho: 1, // OSQP_ADAPTIVE_RHO_UPDATE_ITERATIONS
    adaptive_rho_interval: 50,
    adaptive_rho_fraction: 0.4,
    adaptive_rho_tolerance: 5.0,
    max_iter: 200,
    eps_abs: 0.001,
    eps_rel: 0.001,
    eps_prim_inf: 0.0001,
    eps_dual_inf: 0.0001,
    scaled_termination: 0,
    check_termination: 25,
    check_dualgap: 1,
    time_limit: 10_000_000_000.0,
    delta: 1e-06,
    polish_refine_iter: 3,
};

/// `_OSQP_INFEASIBLE_STATUSES`: primal/dual infeasible (+ inaccurate).
pub const INFEASIBLE_STATUSES: [i64; 4] = [3, 4, 5, 6];

/// What OSQP reported for one solve (osqp-python's `results.info` + x).
#[derive(Clone, Debug, PartialEq)]
pub struct OsqpRun {
    pub status_val: i64,
    /// `info->status`, e.g. "solved", "maximum iterations reached".
    pub status: String,
    pub status_polish: i64,
    pub iter: i64,
    pub rho_updates: i64,
    pub rho_estimate: f64,
    pub obj_val: f64,
    pub prim_res: f64,
    pub dual_res: f64,
    /// `solution->x` (n entries), if OSQP allocated one.
    pub x: Option<Vec<f64>>,
    /// The settings the solver ran with, read back after `osqp_setup`.
    pub effective: OsqpSettings,
}

/// Why OSQP did not run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OsqpError {
    /// The arrays do not form a valid problem (refused before OSQP C sees
    /// them: a bad CSC would make it read out of bounds).
    Invalid,
    /// `osqp_setup` returned this error code.
    Setup(i64),
}

/// Solves `min 1/2 x'Px + q'x s.t. l <= Ax <= u` once, fresh workspace,
/// optionally warm-started in x. `p` must be the upper triangle.
pub fn osqp_solve(
    p: &Csc,
    q: &[f64],
    a: &Csc,
    l: &[f64],
    u: &[f64],
    settings: &OsqpSettings,
    warm_x: Option<&[f64]>,
) -> Result<OsqpRun, OsqpError> {
    c_osqp::run(p, q, a, l, u, settings, warm_x)
}

/// The result `_solve_rti` returns, field for field.
#[derive(Clone, Debug, PartialEq)]
pub struct RtiOutput {
    /// The control sequence `[delta_0, a_0, ...]` (2N).
    pub zopt: Vec<f64>,
    /// `x_1..x_N` of the true-model rollout, 4 columns.
    pub x_pred: Vec<[f64; 4]>,
    /// The command column with a speed model.
    pub v_cmd_pred: Option<Vec<f64>>,
    pub success: bool,
    pub status: i64,
    pub status_message: String,
    pub iterations: i64,
    pub cost: f64,
    /// Per-slot boundary slack (empty when the rows are hard or inert).
    pub boundary_slack: Vec<f64>,
    /// With a speed model: the speed-cap slack (0.0 without a cap).
    pub speed_slack: Option<f64>,
    pub osqp: OsqpRun,
    pub qp_n: usize,
    pub qp_m: usize,
    pub nnz_p: usize,
    pub nnz_a: usize,
}

/// `_solve_rti(...)` for `stage_spec None`.
pub fn solve_rti(inp: &RtiInput) -> Result<RtiOutput, SolveError> {
    let qp = qp::build(inp)?;
    let run = osqp_solve(
        &qp.p,
        &qp.q,
        &qp.a,
        &qp.l,
        &qp.u,
        &PYTHON_SETTINGS,
        Some(&qp.z_guess),
    )
    .map_err(|e| match e {
        OsqpError::Setup(code) => SolveError::OsqpSetup(code),
        OsqpError::Invalid => SolveError::Length,
    })?;
    finish(inp, &qp, run)
}

/// Everything after `prob.solve()`: the accept/hold decision, the rollout,
/// the true cost, the slacks.
fn finish(inp: &RtiInput, qp: &Qp, run: OsqpRun) -> Result<RtiOutput, SolveError> {
    let lay = qp.layout;
    let n = lay.horizon;
    let infeasible = INFEASIBLE_STATUSES.contains(&run.status_val);
    let (z_full, success) = match &run.x {
        Some(x) if !infeasible && x.iter().all(|v| v.is_finite()) => (x.clone(), true),
        _ => (qp.z_guess.clone(), false),
    };
    let zopt: Vec<f64> = z_full[..2 * n].to_vec();

    // _rollout_true_dynamics(x0, zopt, N, ts, params, sm)
    let lag: Option<LagParams> = inp.speed_model.map(|sm| sm.lag);
    let mut xs: Vec<State> = Vec::with_capacity(n);
    let mut x = qp.x_ref[0];
    for k in 0..n {
        x = model::step(
            &x,
            [zopt[2 * k], zopt[2 * k + 1]],
            inp.ts,
            inp.wheelbase,
            inp.lr,
            lag.as_ref(),
        )?;
        xs.push(x);
    }
    let x_pred: Vec<[f64; 4]> = xs.iter().map(|s| [s[0], s[1], s[2], s[3]]).collect();
    let v_cmd_pred = inp
        .speed_model
        .map(|_| xs.iter().map(|s| s[4]).collect::<Vec<f64>>());

    let cost = planner_cost(inp, &zopt, &xs)?;

    let base = n * (2 + lay.nx);
    let boundary_slack = if inp.boundaries.use_slack {
        z_full[base..base + lay.n_slack].to_vec()
    } else {
        Vec::new()
    };
    let speed_slack = inp
        .speed_model
        .map(|_| if lay.n_vs == 1 { z_full[lay.vs()] } else { 0.0 });

    Ok(RtiOutput {
        zopt,
        x_pred,
        v_cmd_pred,
        success,
        status: run.status_val,
        status_message: run.status.clone(),
        iterations: run.iter,
        cost,
        boundary_slack,
        speed_slack,
        qp_n: lay.n_z,
        qp_m: qp.l.len(),
        nnz_p: qp.p.nnz(),
        nnz_a: qp.a.nnz(),
        osqp: run,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_settings_are_the_c_defaults_plus_the_four_python_overrides() {
        let mut d = c_osqp::default_settings();
        // _solve_rti's setup(warm_starting=True, verbose=False,
        // polishing=True, max_iter=200)
        d.warm_starting = 1;
        d.verbose = 0;
        d.polishing = 1;
        d.max_iter = 200;
        assert_eq!(d, PYTHON_SETTINGS);
    }

    #[test]
    fn a_tiny_qp_solves_like_the_osqp_demo() {
        // OSQP's demo: P = [[4, 1], [1, 2]], q = [1, 1], A = [[1, 1], [1, 0], [0, 1]]
        let p = Csc {
            nrows: 2,
            ncols: 2,
            colptr: vec![0, 1, 3],
            rowind: vec![0, 0, 1],
            values: vec![4.0, 1.0, 2.0],
        };
        let a = Csc {
            nrows: 3,
            ncols: 2,
            colptr: vec![0, 2, 4],
            rowind: vec![0, 1, 0, 2],
            values: vec![1.0, 1.0, 1.0, 1.0],
        };
        let run = osqp_solve(
            &p,
            &[1.0, 1.0],
            &a,
            &[1.0, 0.0, 0.0],
            &[1.0, 0.7, 0.7],
            &PYTHON_SETTINGS,
            None,
        )
        .unwrap();
        assert_eq!(run.status_val, 1);
        assert_eq!(run.status, "solved");
        let x = run.x.unwrap();
        assert!((x[0] - 0.3).abs() < 1e-3 && (x[1] - 0.7).abs() < 1e-3);
        assert_eq!(run.effective, PYTHON_SETTINGS);
    }

    #[test]
    fn a_bad_problem_is_refused_before_osqp_c() {
        let p = Csc {
            nrows: 2,
            ncols: 2,
            colptr: vec![0, 1, 5],
            rowind: vec![0, 0, 1],
            values: vec![4.0, 1.0, 2.0],
        };
        let a = Csc {
            nrows: 0,
            ncols: 2,
            colptr: vec![0, 0, 0],
            rowind: vec![],
            values: vec![],
        };
        assert_eq!(
            osqp_solve(&p, &[1.0, 1.0], &a, &[], &[], &PYTHON_SETTINGS, None),
            Err(OsqpError::Invalid)
        );
    }

    #[test]
    fn l_above_u_is_an_osqp_setup_error() {
        let p = Csc {
            nrows: 1,
            ncols: 1,
            colptr: vec![0, 1],
            rowind: vec![0],
            values: vec![1.0],
        };
        let a = Csc {
            nrows: 1,
            ncols: 1,
            colptr: vec![0, 1],
            rowind: vec![0],
            values: vec![1.0],
        };
        assert_eq!(
            osqp_solve(&p, &[0.0], &a, &[1.0], &[0.0], &PYTHON_SETTINGS, None),
            Err(OsqpError::Setup(1))
        );
    }
}
