//! The solver core's C ABI for `mpc_controller/mpc_native.py` (R2).
//!
//! - [`mpc_solve_rti`]: one `_solve_rti` tick (stage_spec None), stateless:
//!   [`MpcRtiArgs`] in, [`MpcRtiResult`] and the caller's
//!   [`MpcRtiBuffers`] out;
//! - debug exports for the parity layers: [`mpc_build_qp`] (the QP exactly
//!   as OSQP C receives it, layer L1), [`mpc_osqp_solve_csc`] (OSQP with the
//!   solver's settings on a caller's QP, layer L2), [`mpc_osqp_settings`]
//!   and [`mpc_osqp_version`].
//!
//! Same contract as the corridor entry points (see the parent module):
//! `#[repr(C)]` structs whose sizes the loader checks, optional inputs as a
//! value plus an `i32` `has_*` flag (never NaN as a sentinel), caller-owned
//! output buffers with their capacities, an `i32` status, panics caught.
#![allow(unsafe_code)]

use std::ffi::c_char;

use super::{
    guarded, MPC_ERR_EMPTY, MPC_ERR_HORIZON, MPC_ERR_LENGTH, MPC_ERR_MATH_DOMAIN,
    MPC_ERR_MISSING_KEY, MPC_ERR_NULL, MPC_ERR_OSQP_SETUP, MPC_ERR_OVERFLOW, MPC_ERR_TOO_LARGE,
    MPC_ERR_X0_SIZE, MPC_ERR_ZERO_DIVISION, MPC_OK,
};
use crate::model::LagParams;
use crate::problem::{
    Boundaries, CorridorView, Limits, RtiInput, SolveError, SpeedModel, TargetLine, Weights,
};
use crate::pyfloat::PyError;
use crate::qp::{self, Csc};
use crate::solve::{self, OsqpError, OsqpRun, OsqpSettings, PYTHON_SETTINGS};

/// One tick's inputs (`solve_mpc_step`'s arguments, weights already scaled).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MpcRtiArgs {
    pub horizon: i64,
    /// Entries of `x0` (4, or 5 with a speed model).
    pub n_x0: i64,
    /// Entries of each corridor array.
    pub n_corridor: i64,
    /// Rows of `obstacles` (`(x, y, r)`).
    pub n_obstacles: i64,
    /// Rows of `boundaries`, already `list(boundaries)[:max_sources]`.
    pub n_boundaries: i64,
    pub boundary_max_sources: i64,
    /// Entries of `warm_start` (raveled); ignored without `has_warm_start`.
    pub n_warm_start: i64,
    pub n_front_offsets: i64,
    /// Entries of `c_lo`, `c_hi` (and `v_hi`) with a speed model.
    pub n_speed: i64,
    pub x0: *const f64,
    pub warm_start: *const f64,
    pub xc: *const f64,
    pub yc: *const f64,
    pub nx: *const f64,
    pub ny: *const f64,
    pub half_width: *const f64,
    /// `n_obstacles x 3`, row-major.
    pub obstacles: *const f64,
    /// `n_boundaries x 3`, row-major.
    pub boundaries: *const f64,
    pub front_offsets: *const f64,
    pub c_lo: *const f64,
    pub c_hi: *const f64,
    pub v_hi: *const f64,
    pub last_u: [f64; 2],
    pub pref_nom: [f64; 2],
    pub line_p: [f64; 2],
    pub ts: f64,
    pub control_period: f64,
    pub wheelbase: f64,
    pub lr: f64,
    pub vdes: f64,
    pub delta_min: f64,
    pub delta_max: f64,
    pub a_min: f64,
    pub a_max: f64,
    pub d_delta_min: f64,
    pub d_delta_max: f64,
    pub d_a_min: f64,
    pub d_a_max: f64,
    pub v_min: f64,
    pub v_max: f64,
    pub w_v: f64,
    pub w_delta0: f64,
    pub w_u_a: f64,
    pub w_du_delta: f64,
    pub w_du_a: f64,
    pub w_obs: f64,
    pub w_corr: f64,
    pub w_line: f64,
    pub w_psi_stage: f64,
    pub w_term: f64,
    pub w_psi: f64,
    pub car_radius: f64,
    pub avoidance_margin: f64,
    pub psi_ref: f64,
    pub psi_ref_turn: f64,
    pub psi_start: f64,
    pub line_psi: f64,
    pub boundary_slack_weight: f64,
    pub boundary_slack_max: f64,
    pub sm_alpha: f64,
    pub sm_d: f64,
    pub sm_slack_weight: f64,
    pub has_warm_start: i32,
    pub has_control_period: i32,
    pub has_psi_ref: i32,
    pub has_psi_ref_turn: i32,
    pub has_psi_start: i32,
    pub has_target_line: i32,
    /// `bool(boundaries) and not boundary_hard`.
    pub boundary_use_slack: i32,
    pub has_speed_model: i32,
    /// `int(speed_model.get('delay', 0)) != 0`.
    pub sm_delay: i32,
    pub has_v_hi: i32,
}

/// The scalar part of `_solve_rti`'s info dict, plus OSQP's own numbers.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MpcRtiResult {
    /// OSQP `status_val` (info['status']).
    pub status: i64,
    pub iterations: i64,
    pub status_polish: i64,
    pub rho_updates: i64,
    /// QP size: variables, rows, nonzeros of P (upper) and A.
    pub n: i64,
    pub m: i64,
    pub nnz_p: i64,
    pub nnz_a: i64,
    /// Entries written to `boundary_slack` (0 when the rows are hard).
    pub n_boundary_slack: i64,
    /// The OSQP error code with MPC_ERR_OSQP_SETUP.
    pub osqp_error: i64,
    /// The x0 size the speed model needs, with MPC_ERR_X0_SIZE.
    pub x0_expected: i64,
    pub cost: f64,
    pub speed_slack: f64,
    pub obj_val: f64,
    pub prim_res: f64,
    pub dual_res: f64,
    pub rho_estimate: f64,
    pub success: i32,
    pub has_speed_slack: i32,
    /// OSQP's `info->status` ("solved", ...), NUL-terminated.
    pub status_message: [c_char; 32],
}

/// Caller-owned output arrays with their capacities.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MpcRtiBuffers {
    /// `2 N`.
    pub n_zopt: i64,
    pub zopt: *mut f64,
    /// `N * 4`, row-major (x_1..x_N).
    pub n_x_pred: i64,
    pub x_pred: *mut f64,
    /// `N` with a speed model, else 0 (pointer may be null).
    pub n_v_cmd: i64,
    pub v_cmd_pred: *mut f64,
    /// At least `boundary_max_sources` (pointer may be null when 0).
    pub n_slack: i64,
    pub boundary_slack: *mut f64,
}

/// QP sizes ([`mpc_build_qp`]).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MpcQpSizes {
    pub n: i64,
    pub m: i64,
    pub nnz_p: i64,
    pub nnz_a: i64,
    pub n_slack: i64,
    pub n_vs: i64,
}

/// Caller-owned QP arrays: `colptr` n + 1, `rowind`/`val` nnz, `q` and
/// `z_guess` n, `l` and `u` m. The capacities must equal the sizes.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MpcQpBuffers {
    pub sizes: MpcQpSizes,
    pub p_colptr: *mut i64,
    pub p_rowind: *mut i64,
    pub p_val: *mut f64,
    pub q: *mut f64,
    pub a_colptr: *mut i64,
    pub a_rowind: *mut i64,
    pub a_val: *mut f64,
    pub l: *mut f64,
    pub u: *mut f64,
    pub z_guess: *mut f64,
}

/// A QP for [`mpc_osqp_solve_csc`] (P upper triangular, CSC).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MpcOsqpArgs {
    pub n: i64,
    pub m: i64,
    pub nnz_p: i64,
    pub nnz_a: i64,
    pub p_colptr: *const i64,
    pub p_rowind: *const i64,
    pub p_val: *const f64,
    pub q: *const f64,
    pub a_colptr: *const i64,
    pub a_rowind: *const i64,
    pub a_val: *const f64,
    pub l: *const f64,
    pub u: *const f64,
    /// n entries, or null: no warm start.
    pub warm_x: *const f64,
}

/// Every OSQPSettings field, in OSQP's order.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MpcOsqpSettings {
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

/// OSQP's report for [`mpc_osqp_solve_csc`].
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MpcOsqpResult {
    pub status_val: i64,
    pub status_polish: i64,
    pub iter: i64,
    pub rho_updates: i64,
    /// The OSQP error code with MPC_ERR_OSQP_SETUP.
    pub setup_error: i64,
    /// 1 when x was written.
    pub has_x: i64,
    pub obj_val: f64,
    pub prim_res: f64,
    pub dual_res: f64,
    pub rho_estimate: f64,
    /// The settings the solver ran with, read back after setup.
    pub settings: MpcOsqpSettings,
    pub status: [c_char; 32],
}

/// `(field name, byte offset)` of every field of `$t`, from `offset_of!`.
macro_rules! fields {
    ($t:ty; $($f:ident),* $(,)?) => {
        &[$((stringify!($f), std::mem::offset_of!($t, $f))),*]
    };
}

/// Every field of each solver struct with its offset: the loader compares
/// them with its ctypes mirrors, so two fields of one type in a different
/// order (which the struct size cannot show) are refused too.
pub(super) fn field_table(kind: i32) -> Option<&'static [(&'static str, usize)]> {
    const RTI_ARGS: &[(&str, usize)] = fields!(MpcRtiArgs;
        horizon, n_x0, n_corridor, n_obstacles, n_boundaries, boundary_max_sources,
        n_warm_start, n_front_offsets, n_speed, x0, warm_start, xc, yc, nx, ny, half_width,
        obstacles, boundaries, front_offsets, c_lo, c_hi, v_hi, last_u, pref_nom, line_p, ts,
        control_period, wheelbase, lr, vdes, delta_min, delta_max, a_min, a_max, d_delta_min,
        d_delta_max, d_a_min, d_a_max, v_min, v_max, w_v, w_delta0, w_u_a, w_du_delta, w_du_a,
        w_obs, w_corr, w_line, w_psi_stage, w_term, w_psi, car_radius, avoidance_margin,
        psi_ref, psi_ref_turn, psi_start, line_psi, boundary_slack_weight, boundary_slack_max,
        sm_alpha, sm_d, sm_slack_weight, has_warm_start, has_control_period, has_psi_ref,
        has_psi_ref_turn, has_psi_start, has_target_line, boundary_use_slack, has_speed_model,
        sm_delay, has_v_hi);
    const RTI_RESULT: &[(&str, usize)] = fields!(MpcRtiResult;
        status, iterations, status_polish, rho_updates, n, m, nnz_p, nnz_a, n_boundary_slack,
        osqp_error, x0_expected, cost, speed_slack, obj_val, prim_res, dual_res, rho_estimate,
        success, has_speed_slack, status_message);
    const RTI_BUFFERS: &[(&str, usize)] = fields!(MpcRtiBuffers;
        n_zopt, zopt, n_x_pred, x_pred, n_v_cmd, v_cmd_pred, n_slack, boundary_slack);
    const QP_SIZES: &[(&str, usize)] = fields!(MpcQpSizes; n, m, nnz_p, nnz_a, n_slack, n_vs);
    const QP_BUFFERS: &[(&str, usize)] = fields!(MpcQpBuffers;
        sizes, p_colptr, p_rowind, p_val, q, a_colptr, a_rowind, a_val, l, u, z_guess);
    const OSQP_ARGS: &[(&str, usize)] = fields!(MpcOsqpArgs;
        n, m, nnz_p, nnz_a, p_colptr, p_rowind, p_val, q, a_colptr, a_rowind, a_val, l, u,
        warm_x);
    const OSQP_RESULT: &[(&str, usize)] = fields!(MpcOsqpResult;
        status_val, status_polish, iter, rho_updates, setup_error, has_x, obj_val, prim_res,
        dual_res, rho_estimate, settings, status);
    const OSQP_SETTINGS: &[(&str, usize)] = fields!(MpcOsqpSettings;
        device, linsys_solver, allocate_solution, verbose, profiler_level, warm_starting,
        scaling, polishing, rho, rho_is_vec, sigma, alpha, cg_max_iter, cg_tol_reduction,
        cg_tol_fraction, cg_precond, adaptive_rho, adaptive_rho_interval,
        adaptive_rho_fraction, adaptive_rho_tolerance, max_iter, eps_abs, eps_rel,
        eps_prim_inf, eps_dual_inf, scaled_termination, check_termination, check_dualgap,
        time_limit, delta, polish_refine_iter);
    match kind {
        super::STRUCT_RTI_ARGS => Some(RTI_ARGS),
        super::STRUCT_RTI_RESULT => Some(RTI_RESULT),
        super::STRUCT_RTI_BUFFERS => Some(RTI_BUFFERS),
        super::STRUCT_QP_SIZES => Some(QP_SIZES),
        super::STRUCT_QP_BUFFERS => Some(QP_BUFFERS),
        super::STRUCT_OSQP_ARGS => Some(OSQP_ARGS),
        super::STRUCT_OSQP_RESULT => Some(OSQP_RESULT),
        super::STRUCT_OSQP_SETTINGS => Some(OSQP_SETTINGS),
        _ => None,
    }
}

fn settings_out(s: &OsqpSettings) -> MpcOsqpSettings {
    MpcOsqpSettings {
        device: s.device,
        linsys_solver: s.linsys_solver,
        allocate_solution: s.allocate_solution,
        verbose: s.verbose,
        profiler_level: s.profiler_level,
        warm_starting: s.warm_starting,
        scaling: s.scaling,
        polishing: s.polishing,
        rho: s.rho,
        rho_is_vec: s.rho_is_vec,
        sigma: s.sigma,
        alpha: s.alpha,
        cg_max_iter: s.cg_max_iter,
        cg_tol_reduction: s.cg_tol_reduction,
        cg_tol_fraction: s.cg_tol_fraction,
        cg_precond: s.cg_precond,
        adaptive_rho: s.adaptive_rho,
        adaptive_rho_interval: s.adaptive_rho_interval,
        adaptive_rho_fraction: s.adaptive_rho_fraction,
        adaptive_rho_tolerance: s.adaptive_rho_tolerance,
        max_iter: s.max_iter,
        eps_abs: s.eps_abs,
        eps_rel: s.eps_rel,
        eps_prim_inf: s.eps_prim_inf,
        eps_dual_inf: s.eps_dual_inf,
        scaled_termination: s.scaled_termination,
        check_termination: s.check_termination,
        check_dualgap: s.check_dualgap,
        time_limit: s.time_limit,
        delta: s.delta,
        polish_refine_iter: s.polish_refine_iter,
    }
}

fn status_text(s: &str) -> [c_char; 32] {
    let mut out = [0 as c_char; 32];
    for (dst, &b) in out.iter_mut().zip(s.as_bytes().iter().take(31)) {
        *dst = b as c_char;
    }
    out
}

fn len(v: i64) -> Option<usize> {
    usize::try_from(v).ok()
}

/// # Safety
/// `ptr` must be null or point to `n` readable values.
unsafe fn slice<'a, T>(ptr: *const T, n: usize) -> Option<&'a [T]> {
    if n == 0 {
        return Some(&[]);
    }
    if ptr.is_null() {
        return None;
    }
    // SAFETY: non-null and `n` readable values by the caller's contract.
    Some(unsafe { std::slice::from_raw_parts(ptr, n) })
}

/// # Safety
/// `ptr` must be null or point to `n` writable values.
unsafe fn slice_mut<'a, T>(ptr: *mut T, n: usize) -> Option<&'a mut [T]> {
    if n == 0 {
        return Some(&mut []);
    }
    if ptr.is_null() {
        return None;
    }
    // SAFETY: non-null and `n` writable values by the caller's contract.
    Some(unsafe { std::slice::from_raw_parts_mut(ptr, n) })
}

/// `n` rows of three f64 as `[f64; 3]` (same size and alignment as three
/// consecutive f64).
///
/// # Safety
/// `ptr` must be null or point to `3 n` readable f64s.
unsafe fn rows3<'a>(ptr: *const f64, n: usize) -> Option<&'a [[f64; 3]]> {
    // SAFETY: [f64; 3] is three contiguous f64, alignment 8.
    unsafe { slice(ptr.cast::<[f64; 3]>(), n) }
}

/// The args as an [`RtiInput`]; `Err(status)` for a null or a bad size.
///
/// # Safety
/// Every pointer in `a` must point to the number of values its `n_*` field
/// declares (see [`MpcRtiArgs`]).
unsafe fn input(a: &MpcRtiArgs) -> Result<RtiInput<'_>, i32> {
    let bad = MPC_ERR_LENGTH;
    let (Some(n_x0), Some(n_cor), Some(n_obs), Some(n_bnd), Some(max_src), Some(n_off)) = (
        len(a.n_x0),
        len(a.n_corridor),
        len(a.n_obstacles),
        len(a.n_boundaries),
        len(a.boundary_max_sources),
        len(a.n_front_offsets),
    ) else {
        return Err(bad);
    };
    let horizon = len(a.horizon).ok_or(MPC_ERR_HORIZON)?;
    if n_bnd > max_src {
        return Err(bad);
    }
    let null = MPC_ERR_NULL;
    // SAFETY (all slices): the caller's contract on the declared sizes.
    let x0 = unsafe { slice(a.x0, n_x0) }.ok_or(null)?;
    let cor = unsafe {
        (
            slice(a.xc, n_cor),
            slice(a.yc, n_cor),
            slice(a.nx, n_cor),
            slice(a.ny, n_cor),
            slice(a.half_width, n_cor),
        )
    };
    let (Some(xc), Some(yc), Some(nx), Some(ny), Some(half_width)) = cor else {
        return Err(null);
    };
    let obstacles = unsafe { rows3(a.obstacles, n_obs) }.ok_or(null)?;
    let boundary_rows = unsafe { rows3(a.boundaries, n_bnd) }.ok_or(null)?;
    let front_offsets = unsafe { slice(a.front_offsets, n_off) }.ok_or(null)?;
    let warm_start = if a.has_warm_start != 0 {
        let n_w = len(a.n_warm_start).ok_or(bad)?;
        Some(unsafe { slice(a.warm_start, n_w) }.ok_or(null)?)
    } else {
        None
    };
    let speed_model = if a.has_speed_model != 0 {
        let n_s = len(a.n_speed).ok_or(bad)?;
        let c_lo = unsafe { slice(a.c_lo, n_s) }.ok_or(null)?;
        let c_hi = unsafe { slice(a.c_hi, n_s) }.ok_or(null)?;
        let v_hi = if a.has_v_hi != 0 {
            Some(unsafe { slice(a.v_hi, n_s) }.ok_or(null)?)
        } else {
            None
        };
        Some(SpeedModel {
            lag: LagParams {
                alpha: a.sm_alpha,
                d: a.sm_d,
                delay: a.sm_delay != 0,
            },
            c_lo,
            c_hi,
            v_hi,
            slack_weight: a.sm_slack_weight,
        })
    } else {
        None
    };
    let opt = |has: i32, v: f64| (has != 0).then_some(v);
    Ok(RtiInput {
        x0,
        last_u: a.last_u,
        pref_nom: a.pref_nom,
        horizon,
        ts: a.ts,
        control_period: opt(a.has_control_period, a.control_period),
        wheelbase: a.wheelbase,
        lr: a.lr,
        limits: Limits {
            delta_min: a.delta_min,
            delta_max: a.delta_max,
            a_min: a.a_min,
            a_max: a.a_max,
            d_delta_min: a.d_delta_min,
            d_delta_max: a.d_delta_max,
            d_a_min: a.d_a_min,
            d_a_max: a.d_a_max,
            v_min: a.v_min,
            v_max: a.v_max,
        },
        weights: Weights {
            w_v: a.w_v,
            w_delta0: a.w_delta0,
            w_u_a: a.w_u_a,
            w_du_delta: a.w_du_delta,
            w_du_a: a.w_du_a,
            w_obs: a.w_obs,
            w_corr: a.w_corr,
            w_line: a.w_line,
            w_psi_stage: a.w_psi_stage,
            w_term: a.w_term,
            w_psi: a.w_psi,
        },
        vdes: a.vdes,
        front_offsets,
        corridor: CorridorView {
            xc,
            yc,
            nx,
            ny,
            half_width,
            psi_ref: opt(a.has_psi_ref, a.psi_ref),
            psi_ref_turn: opt(a.has_psi_ref_turn, a.psi_ref_turn),
            psi_start: opt(a.has_psi_start, a.psi_start),
            target_line: (a.has_target_line != 0).then_some(TargetLine {
                p: a.line_p,
                psi: a.line_psi,
            }),
            car_radius: a.car_radius,
            avoidance_margin: a.avoidance_margin,
            obstacles,
        },
        boundaries: Boundaries {
            rows: boundary_rows,
            max_sources: max_src,
            use_slack: a.boundary_use_slack != 0,
            slack_weight: a.boundary_slack_weight,
            slack_max: a.boundary_slack_max,
        },
        warm_start,
        speed_model,
    })
}

/// The status code of a [`SolveError`]; details go into the result.
fn error_status(e: SolveError, result: &mut MpcRtiResult) -> i32 {
    match e {
        SolveError::Py(PyError::MathDomain) => MPC_ERR_MATH_DOMAIN,
        SolveError::Py(PyError::ZeroDivision) => MPC_ERR_ZERO_DIVISION,
        SolveError::Py(PyError::Overflow) => MPC_ERR_OVERFLOW,
        SolveError::X0Size { want, .. } => {
            result.x0_expected = i64::try_from(want).unwrap_or(-1);
            MPC_ERR_X0_SIZE
        }
        SolveError::Horizon => MPC_ERR_HORIZON,
        SolveError::EmptyCorridor => MPC_ERR_EMPTY,
        SolveError::Length => MPC_ERR_LENGTH,
        SolveError::MissingPsiRef => MPC_ERR_MISSING_KEY,
        SolveError::TooLarge => MPC_ERR_TOO_LARGE,
        SolveError::OsqpSetup(code) => {
            result.osqp_error = code;
            MPC_ERR_OSQP_SETUP
        }
    }
}

fn empty_result() -> MpcRtiResult {
    MpcRtiResult {
        status: -1,
        iterations: -1,
        status_polish: 0,
        rho_updates: 0,
        n: 0,
        m: 0,
        nnz_p: 0,
        nnz_a: 0,
        n_boundary_slack: 0,
        osqp_error: 0,
        x0_expected: 0,
        cost: f64::NAN,
        speed_slack: 0.0,
        obj_val: f64::NAN,
        prim_res: f64::NAN,
        dual_res: f64::NAN,
        rho_estimate: f64::NAN,
        success: 0,
        has_speed_slack: 0,
        status_message: [0; 32],
    }
}

fn to_i64(v: usize) -> i64 {
    i64::try_from(v).unwrap_or(i64::MAX)
}

/// One RTI tick: `_solve_rti(...)` with `stage_spec None`.
///
/// # Safety
/// `args`, `result` and `buf` must be valid pointers to their structs; every
/// pointer inside them must point to the number of values its size field
/// declares.
#[no_mangle]
pub unsafe extern "C" fn mpc_solve_rti(
    args: *const MpcRtiArgs,
    result: *mut MpcRtiResult,
    buf: *const MpcRtiBuffers,
) -> i32 {
    if args.is_null() || result.is_null() || buf.is_null() {
        return MPC_ERR_NULL;
    }
    // SAFETY: non-null, valid by the caller's contract.
    let (a, result, buf) = unsafe { (&*args, &mut *result, *buf) };
    guarded(|| {
        *result = empty_result();
        // SAFETY: the caller's contract on the argument arrays.
        let inp = match unsafe { input(a) } {
            Ok(i) => i,
            Err(s) => return s,
        };
        let out = match solve::solve_rti(&inp) {
            Ok(o) => o,
            Err(e) => return error_status(e, result),
        };
        let n = inp.horizon;
        let (Some(n_zopt), Some(n_xp), Some(n_vc), Some(n_sl)) = (
            len(buf.n_zopt),
            len(buf.n_x_pred),
            len(buf.n_v_cmd),
            len(buf.n_slack),
        ) else {
            return MPC_ERR_LENGTH;
        };
        if n_zopt != 2 * n || n_xp != 4 * n || n_sl < out.boundary_slack.len() {
            return MPC_ERR_LENGTH;
        }
        let want_vc = out.v_cmd_pred.as_ref().map_or(0, Vec::len);
        if n_vc != want_vc {
            return MPC_ERR_LENGTH;
        }
        // SAFETY: the caller's contract on the buffer capacities.
        let bufs = unsafe {
            (
                slice_mut(buf.zopt, n_zopt),
                slice_mut(buf.x_pred, n_xp),
                slice_mut(buf.v_cmd_pred, n_vc),
                slice_mut(buf.boundary_slack, out.boundary_slack.len()),
            )
        };
        let (Some(zopt), Some(x_pred), Some(v_cmd), Some(slack)) = bufs else {
            return MPC_ERR_NULL;
        };
        zopt.copy_from_slice(&out.zopt);
        for (k, row) in out.x_pred.iter().enumerate() {
            x_pred[4 * k..4 * k + 4].copy_from_slice(row);
        }
        if let Some(vc) = &out.v_cmd_pred {
            v_cmd.copy_from_slice(vc);
        }
        slack.copy_from_slice(&out.boundary_slack);
        let run: &OsqpRun = &out.osqp;
        *result = MpcRtiResult {
            status: out.status,
            iterations: out.iterations,
            status_polish: run.status_polish,
            rho_updates: run.rho_updates,
            n: to_i64(out.qp_n),
            m: to_i64(out.qp_m),
            nnz_p: to_i64(out.nnz_p),
            nnz_a: to_i64(out.nnz_a),
            n_boundary_slack: to_i64(out.boundary_slack.len()),
            osqp_error: 0,
            x0_expected: 0,
            cost: out.cost,
            speed_slack: out.speed_slack.unwrap_or(0.0),
            obj_val: run.obj_val,
            prim_res: run.prim_res,
            dual_res: run.dual_res,
            rho_estimate: run.rho_estimate,
            success: i32::from(out.success),
            has_speed_slack: i32::from(out.speed_slack.is_some()),
            status_message: status_text(&out.status_message),
        };
        MPC_OK
    })
}

/// The QP of one tick, as OSQP C receives it (debug, parity layer L1).
/// Always writes `sizes`; with a non-null `buf` whose capacities equal them,
/// also the arrays.
///
/// # Safety
/// As [`mpc_solve_rti`]; `buf`, when non-null, must point to arrays of the
/// capacities in `buf.sizes`.
#[no_mangle]
pub unsafe extern "C" fn mpc_build_qp(
    args: *const MpcRtiArgs,
    sizes: *mut MpcQpSizes,
    buf: *const MpcQpBuffers,
) -> i32 {
    if args.is_null() || sizes.is_null() {
        return MPC_ERR_NULL;
    }
    // SAFETY: non-null, valid by the caller's contract.
    let (a, sizes) = unsafe { (&*args, &mut *sizes) };
    // SAFETY: null checked; valid by the caller's contract otherwise.
    let buf = if buf.is_null() {
        None
    } else {
        Some(unsafe { *buf })
    };
    guarded(|| {
        // SAFETY: the caller's contract on the argument arrays.
        let inp = match unsafe { input(a) } {
            Ok(i) => i,
            Err(s) => return s,
        };
        let qp = match qp::build(&inp) {
            Ok(q) => q,
            Err(e) => return error_status(e, &mut empty_result()),
        };
        let got = MpcQpSizes {
            n: to_i64(qp.layout.n_z),
            m: to_i64(qp.l.len()),
            nnz_p: to_i64(qp.p.nnz()),
            nnz_a: to_i64(qp.a.nnz()),
            n_slack: to_i64(qp.layout.n_slack),
            n_vs: to_i64(qp.layout.n_vs),
        };
        *sizes = got;
        let Some(b) = buf else {
            return MPC_OK;
        };
        let c = b.sizes;
        if (c.n, c.m, c.nnz_p, c.nnz_a) != (got.n, got.m, got.nnz_p, got.nnz_a) {
            return MPC_ERR_LENGTH;
        }
        let (n, m, np, na) = (qp.layout.n_z, qp.l.len(), qp.p.nnz(), qp.a.nnz());
        // SAFETY: the capacities equal the sizes, by the check above and the
        // caller's contract.
        let out = unsafe {
            (
                slice_mut(b.p_colptr, n + 1),
                slice_mut(b.p_rowind, np),
                slice_mut(b.p_val, np),
                slice_mut(b.q, n),
                slice_mut(b.a_colptr, n + 1),
                slice_mut(b.a_rowind, na),
                slice_mut(b.a_val, na),
                slice_mut(b.l, m),
                slice_mut(b.u, m),
                slice_mut(b.z_guess, n),
            )
        };
        let (
            Some(pc),
            Some(pr),
            Some(pv),
            Some(q),
            Some(ac),
            Some(ar),
            Some(av),
            Some(l),
            Some(u),
            Some(zg),
        ) = out
        else {
            return MPC_ERR_NULL;
        };
        pc.copy_from_slice(&qp.p.colptr);
        pr.copy_from_slice(&qp.p.rowind);
        pv.copy_from_slice(&qp.p.values);
        q.copy_from_slice(&qp.q);
        ac.copy_from_slice(&qp.a.colptr);
        ar.copy_from_slice(&qp.a.rowind);
        av.copy_from_slice(&qp.a.values);
        l.copy_from_slice(&qp.l);
        u.copy_from_slice(&qp.u);
        zg.copy_from_slice(&qp.z_guess);
        MPC_OK
    })
}

/// OSQP with the solver's settings ([`PYTHON_SETTINGS`]) on the caller's QP,
/// fresh workspace, x warm start if given (debug, parity layer L2).
///
/// # Safety
/// `args` and `result` must be valid; the arrays in `args` must have the
/// sizes it declares; `x_out` must point to `n` writable f64s.
#[no_mangle]
pub unsafe extern "C" fn mpc_osqp_solve_csc(
    args: *const MpcOsqpArgs,
    result: *mut MpcOsqpResult,
    x_out: *mut f64,
) -> i32 {
    if args.is_null() || result.is_null() || x_out.is_null() {
        return MPC_ERR_NULL;
    }
    // SAFETY: non-null, valid by the caller's contract.
    let (a, result) = unsafe { (&*args, &mut *result) };
    guarded(|| {
        let (Some(n), Some(m), Some(np), Some(na)) =
            (len(a.n), len(a.m), len(a.nnz_p), len(a.nnz_a))
        else {
            return MPC_ERR_LENGTH;
        };
        // SAFETY: the caller's contract on the declared sizes.
        let arrays = unsafe {
            (
                slice(a.p_colptr, n + 1),
                slice(a.p_rowind, np),
                slice(a.p_val, np),
                slice(a.q, n),
                slice(a.a_colptr, n + 1),
                slice(a.a_rowind, na),
                slice(a.a_val, na),
                slice(a.l, m),
                slice(a.u, m),
            )
        };
        let (Some(pc), Some(pr), Some(pv), Some(q), Some(ac), Some(ar), Some(av), Some(l), Some(u)) =
            arrays
        else {
            return MPC_ERR_NULL;
        };
        let warm = if a.warm_x.is_null() {
            None
        } else {
            // SAFETY: non-null, n readable f64s by the caller's contract.
            Some(unsafe { std::slice::from_raw_parts(a.warm_x, n) })
        };
        let p = Csc {
            nrows: n,
            ncols: n,
            colptr: pc.to_vec(),
            rowind: pr.to_vec(),
            values: pv.to_vec(),
        };
        let am = Csc {
            nrows: m,
            ncols: n,
            colptr: ac.to_vec(),
            rowind: ar.to_vec(),
            values: av.to_vec(),
        };
        let run = match solve::osqp_solve(&p, q, &am, l, u, &PYTHON_SETTINGS, warm) {
            Ok(r) => r,
            Err(OsqpError::Invalid) => return MPC_ERR_LENGTH,
            Err(OsqpError::Setup(code)) => {
                result.setup_error = code;
                return MPC_ERR_OSQP_SETUP;
            }
        };
        let has_x = run.x.as_ref().is_some_and(|x| x.len() == n);
        if let Some(x) = run.x.as_ref().filter(|x| x.len() == n) {
            // SAFETY: non-null, n writable f64s by the caller's contract.
            let dst = unsafe { std::slice::from_raw_parts_mut(x_out, n) };
            dst.copy_from_slice(x);
        }
        *result = MpcOsqpResult {
            status_val: run.status_val,
            status_polish: run.status_polish,
            iter: run.iter,
            rho_updates: run.rho_updates,
            setup_error: 0,
            has_x: i64::from(has_x),
            obj_val: run.obj_val,
            prim_res: run.prim_res,
            dual_res: run.dual_res,
            rho_estimate: run.rho_estimate,
            settings: settings_out(&run.effective),
            status: status_text(&run.status),
        };
        MPC_OK
    })
}

/// `which` 0: the settings the solver sets ([`PYTHON_SETTINGS`]); 1: OSQP
/// C's `osqp_set_default_settings`.
///
/// # Safety
/// `out` must be valid for writing.
#[no_mangle]
pub unsafe extern "C" fn mpc_osqp_settings(which: i32, out: *mut MpcOsqpSettings) -> i32 {
    if out.is_null() {
        return MPC_ERR_NULL;
    }
    let s = match which {
        0 => PYTHON_SETTINGS,
        1 => super::osqp::default_settings(),
        _ => return MPC_ERR_LENGTH,
    };
    // SAFETY: non-null, valid for writing by the caller's contract.
    unsafe { *out = settings_out(&s) };
    MPC_OK
}

/// The vendored OSQP C version into `buf` (NUL-terminated, truncated to
/// `cap - 1`). Returns its full length, or MPC_ERR_NULL.
///
/// # Safety
/// `buf` must point to `cap` writable bytes.
#[no_mangle]
pub unsafe extern "C" fn mpc_osqp_version(buf: *mut c_char, cap: i64) -> i64 {
    let v = super::osqp::version();
    let Some(cap) = len(cap).filter(|&c| c > 0) else {
        return i64::from(MPC_ERR_NULL);
    };
    if buf.is_null() {
        return i64::from(MPC_ERR_NULL);
    }
    let bytes = v.as_bytes();
    let n = bytes.len().min(cap - 1);
    // SAFETY: `cap` writable bytes by the caller's contract; n + 1 <= cap.
    let out = unsafe { std::slice::from_raw_parts_mut(buf.cast::<u8>(), cap) };
    out[..n].copy_from_slice(&bytes[..n]);
    out[n] = 0;
    to_i64(bytes.len())
}
