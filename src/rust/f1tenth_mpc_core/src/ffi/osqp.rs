//! The OSQP C library (OSQP v1.0.0 through osqp-sys 1.0.1): the crate's
//! second C ABI, the one it CALLS. Part of the `ffi` module, the only place
//! `unsafe` is allowed.
//!
//! [`run`] does what osqp-python's `OSQP().setup(...)`, `warm_start(x=...)`
//! and `solve()` do for `_solve_rti`: a fresh solver per call, every setting
//! written explicitly, `osqp_warm_start(x, NULL)`, `osqp_solve`, then the
//! info fields and `solution->x` copied out and the solver freed. Like
//! osqp-python it ignores the return codes of warm_start and solve, and
//! like OSQP C itself it does NOT check `l <= u` or finiteness (OSQP C
//! validates `l <= u`; NaN bounds pass, as they do from Python).
//!
//! Safety: the CSC arrays are validated first (column pointers monotone,
//! row indices in range, lengths consistent), so OSQP C never reads out of
//! bounds; OSQP copies the data in `osqp_setup` and never writes through
//! the pointers handed to it, but they come from owned copies anyway.
#![allow(unsafe_code)]

use std::ffi::CStr;
use std::ptr;

use osqp_sys as sys;

use crate::qp::Csc;
use crate::solve::{OsqpError, OsqpRun, OsqpSettings};

type OInt = sys::osqp_int;

/// Frees the solver on every exit path.
struct SolverGuard(*mut sys::OSQPSolver);

impl Drop for SolverGuard {
    fn drop(&mut self) {
        if !self.0.is_null() {
            // SAFETY: a solver osqp_setup allocated, freed exactly once.
            unsafe {
                sys::osqp_cleanup(self.0);
            }
        }
    }
}

// OSQPInt is i64 on 64-bit targets (osqp-sys builds with OSQP_USE_LONG) and
// i32 on 32-bit ones: the conversions are identities here, checked there.
#[allow(clippy::useless_conversion)]
fn to_oint(v: i64) -> Result<OInt, OsqpError> {
    OInt::try_from(v).map_err(|_| OsqpError::Invalid)
}

#[allow(clippy::useless_conversion)]
fn oi(v: OInt) -> i64 {
    i64::from(v)
}

fn usize_to_oint(v: usize) -> Result<OInt, OsqpError> {
    OInt::try_from(v).map_err(|_| OsqpError::Invalid)
}

/// A CSC matrix OSQP C can read without going out of bounds.
fn valid_csc(m: &Csc) -> bool {
    if m.colptr.len() != m.ncols + 1 || m.rowind.len() != m.values.len() {
        return false;
    }
    if m.colptr.first() != Some(&0) {
        return false;
    }
    if m.colptr.windows(2).any(|w| w[1] < w[0]) {
        return false;
    }
    if usize::try_from(m.colptr[m.ncols]).ok() != Some(m.values.len()) {
        return false;
    }
    m.rowind
        .iter()
        .all(|&r| r >= 0 && usize::try_from(r).is_ok_and(|r| r < m.nrows))
}

/// Owned OSQP-typed copies of a CSC matrix, and the struct pointing at them.
struct CscOwned {
    p: Vec<OInt>,
    i: Vec<OInt>,
    x: Vec<f64>,
    nrows: OInt,
    ncols: OInt,
}

impl CscOwned {
    fn new(m: &Csc) -> Result<Self, OsqpError> {
        Ok(CscOwned {
            p: m.colptr
                .iter()
                .map(|&v| to_oint(v))
                .collect::<Result<_, _>>()?,
            i: m.rowind
                .iter()
                .map(|&v| to_oint(v))
                .collect::<Result<_, _>>()?,
            x: m.values.clone(),
            nrows: usize_to_oint(m.nrows)?,
            ncols: usize_to_oint(m.ncols)?,
        })
    }

    /// `csc_set_data(M, m, n, nzmax, x, i, p)`: what OSQPCscMatrix_new sets.
    fn raw(&mut self) -> Result<sys::OSQPCscMatrix, OsqpError> {
        Ok(sys::OSQPCscMatrix {
            m: self.nrows,
            n: self.ncols,
            p: self.p.as_mut_ptr(),
            i: self.i.as_mut_ptr(),
            x: self.x.as_mut_ptr(),
            nzmax: usize_to_oint(self.x.len())?,
            nz: -1,
            owned: 0,
        })
    }
}

fn write_settings(st: &mut sys::OSQPSettings, s: &OsqpSettings) -> Result<(), OsqpError> {
    let u = |v: i64| u32::try_from(v).map_err(|_| OsqpError::Invalid);
    st.device = to_oint(s.device)?;
    st.linsys_solver = u(s.linsys_solver)?;
    st.allocate_solution = to_oint(s.allocate_solution)?;
    st.verbose = to_oint(s.verbose)?;
    st.profiler_level = to_oint(s.profiler_level)?;
    st.warm_starting = to_oint(s.warm_starting)?;
    st.scaling = to_oint(s.scaling)?;
    st.polishing = to_oint(s.polishing)?;
    st.rho = s.rho;
    st.rho_is_vec = to_oint(s.rho_is_vec)?;
    st.sigma = s.sigma;
    st.alpha = s.alpha;
    st.cg_max_iter = to_oint(s.cg_max_iter)?;
    st.cg_tol_reduction = to_oint(s.cg_tol_reduction)?;
    st.cg_tol_fraction = s.cg_tol_fraction;
    st.cg_precond = u(s.cg_precond)?;
    st.adaptive_rho = to_oint(s.adaptive_rho)?;
    st.adaptive_rho_interval = to_oint(s.adaptive_rho_interval)?;
    st.adaptive_rho_fraction = s.adaptive_rho_fraction;
    st.adaptive_rho_tolerance = s.adaptive_rho_tolerance;
    st.max_iter = to_oint(s.max_iter)?;
    st.eps_abs = s.eps_abs;
    st.eps_rel = s.eps_rel;
    st.eps_prim_inf = s.eps_prim_inf;
    st.eps_dual_inf = s.eps_dual_inf;
    st.scaled_termination = to_oint(s.scaled_termination)?;
    st.check_termination = to_oint(s.check_termination)?;
    st.check_dualgap = to_oint(s.check_dualgap)?;
    st.time_limit = s.time_limit;
    st.delta = s.delta;
    st.polish_refine_iter = to_oint(s.polish_refine_iter)?;
    Ok(())
}

fn read_settings(st: &sys::OSQPSettings) -> OsqpSettings {
    OsqpSettings {
        device: oi(st.device),
        linsys_solver: i64::from(st.linsys_solver),
        allocate_solution: oi(st.allocate_solution),
        verbose: oi(st.verbose),
        profiler_level: oi(st.profiler_level),
        warm_starting: oi(st.warm_starting),
        scaling: oi(st.scaling),
        polishing: oi(st.polishing),
        rho: st.rho,
        rho_is_vec: oi(st.rho_is_vec),
        sigma: st.sigma,
        alpha: st.alpha,
        cg_max_iter: oi(st.cg_max_iter),
        cg_tol_reduction: oi(st.cg_tol_reduction),
        cg_tol_fraction: st.cg_tol_fraction,
        cg_precond: i64::from(st.cg_precond),
        adaptive_rho: oi(st.adaptive_rho),
        adaptive_rho_interval: oi(st.adaptive_rho_interval),
        adaptive_rho_fraction: st.adaptive_rho_fraction,
        adaptive_rho_tolerance: st.adaptive_rho_tolerance,
        max_iter: oi(st.max_iter),
        eps_abs: st.eps_abs,
        eps_rel: st.eps_rel,
        eps_prim_inf: st.eps_prim_inf,
        eps_dual_inf: st.eps_dual_inf,
        scaled_termination: oi(st.scaled_termination),
        check_termination: oi(st.check_termination),
        check_dualgap: oi(st.check_dualgap),
        time_limit: st.time_limit,
        delta: st.delta,
        polish_refine_iter: oi(st.polish_refine_iter),
    }
}

/// `osqp_set_default_settings`, read back (the C library's own defaults).
pub fn default_settings() -> OsqpSettings {
    // SAFETY: OSQPSettings is plain data; osqp_set_default_settings writes
    // every field of the struct it is handed.
    unsafe {
        let mut st: sys::OSQPSettings = std::mem::zeroed();
        sys::osqp_set_default_settings(&mut st);
        read_settings(&st)
    }
}

/// The OSQP C version string ("1.0.0").
pub fn version() -> String {
    // SAFETY: osqp_version returns a static NUL-terminated string.
    unsafe {
        let p = sys::osqp_version();
        if p.is_null() {
            return String::new();
        }
        CStr::from_ptr(p).to_string_lossy().into_owned()
    }
}

/// One fresh-workspace solve (see the module docs).
pub fn run(
    p: &Csc,
    q: &[f64],
    a: &Csc,
    l: &[f64],
    u: &[f64],
    settings: &OsqpSettings,
    warm_x: Option<&[f64]>,
) -> Result<OsqpRun, OsqpError> {
    let n = p.ncols;
    let m = a.nrows;
    if p.nrows != n
        || a.ncols != n
        || q.len() != n
        || l.len() != m
        || u.len() != m
        || warm_x.is_some_and(|x| x.len() != n)
        || !valid_csc(p)
        || !valid_csc(a)
    {
        return Err(OsqpError::Invalid);
    }
    let mut p_own = CscOwned::new(p)?;
    let mut a_own = CscOwned::new(a)?;
    let p_raw = p_own.raw()?;
    let a_raw = a_own.raw()?;
    let (m_o, n_o) = (usize_to_oint(m)?, usize_to_oint(n)?);

    // SAFETY: plain data; every field is written by the two calls below.
    let mut st: sys::OSQPSettings = unsafe { std::mem::zeroed() };
    // SAFETY: a valid, writable settings struct.
    unsafe { sys::osqp_set_default_settings(&mut st) };
    write_settings(&mut st, settings)?;

    let mut solver: *mut sys::OSQPSolver = ptr::null_mut();
    // SAFETY: the matrices were validated against their dimensions, the
    // vectors have n / m entries, the settings struct is fully written.
    let code = unsafe {
        sys::osqp_setup(
            &mut solver,
            &p_raw,
            q.as_ptr(),
            &a_raw,
            l.as_ptr(),
            u.as_ptr(),
            m_o,
            n_o,
            &st,
        )
    };
    let guard = SolverGuard(solver);
    if code != 0 {
        return Err(OsqpError::Setup(oi(code)));
    }
    if guard.0.is_null() {
        return Err(OsqpError::Setup(-1));
    }
    // SAFETY: a solver osqp_setup returned without error: settings, info
    // and solution are allocated (allocate_solution 1) for n / m entries.
    unsafe {
        let s = &*guard.0;
        let effective = if s.settings.is_null() {
            *settings
        } else {
            read_settings(&*s.settings)
        };
        if let Some(x) = warm_x {
            // osqp-python's warm_start(x=z_guess): y stays as setup left it
            let _ = sys::osqp_warm_start(guard.0, x.as_ptr(), ptr::null());
        }
        let _ = sys::osqp_solve(guard.0);
        if s.info.is_null() {
            return Err(OsqpError::Setup(-1));
        }
        let info = &*s.info;
        let bytes: Vec<u8> = info.status.iter().map(|&c| c as u8).collect();
        let status = match CStr::from_bytes_until_nul(&bytes) {
            Ok(c) => c.to_string_lossy().into_owned(),
            Err(_) => String::from_utf8_lossy(&bytes).into_owned(),
        };
        let x = if s.solution.is_null() || (*s.solution).x.is_null() {
            None
        } else {
            Some(std::slice::from_raw_parts((*s.solution).x, n).to_vec())
        };
        Ok(OsqpRun {
            status_val: oi(info.status_val),
            status,
            status_polish: oi(info.status_polish),
            iter: oi(info.iter),
            rho_updates: oi(info.rho_updates),
            rho_estimate: info.rho_estimate,
            obj_val: info.obj_val,
            prim_res: info.prim_res,
            dual_res: info.dual_res,
            x,
            effective,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_vendored_osqp_is_c_1_0_0() {
        assert_eq!(version(), "1.0.0");
    }
}
