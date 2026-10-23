//! The solver's vehicle model: `mpc_controller/vehicle_model.py`, operation
//! for operation.
//!
//! - [`step_beta`] / [`jacobian_beta`]: `f1tenth_state_fcn_dt_beta` and its
//!   analytic Jacobian (kinematic bicycle with slip angle, explicit Euler,
//!   state `[X, Y, psi, v]`, control `[delta, a]`);
//! - [`step`] / [`jacobian`]: `mpc_solver._model_step` / `_model_jacobian`,
//!   i.e. the above or, with a [`SpeedModel`], the offset-free variant
//!   `f1tenth_state_fcn_dt_beta_lag` (state `[X, Y, psi, v, c]`, `c` the
//!   command in force, `a` its rate).
//!
//! Every value is computed on Python floats in the reference (`float(xk[i])`,
//! `math.*`), so the failures are CPython's: `math.tan/cos/sin` of an
//! infinity is a ValueError, a division by a zero wheelbase a
//! ZeroDivisionError, `(k * t) ** 2` an OverflowError when it overflows
//! ([`crate::pyfloat`]). A NaN passes through, as in the Python.

use crate::pyfloat::{self, PyError};

/// Largest state dimension (the offset-free model).
pub const MAX_NX: usize = 5;

/// A state, padded to [`MAX_NX`]; the model in use reads the first 4 or 5.
pub type State = [f64; MAX_NX];

/// The offset-free speed model's parameters (`speed_channel.SpeedChannel.plan`
/// builds the dict): the per-stage arrays live in the QP input.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LagParams {
    /// `alpha = 1 - exp(-ts / tau)`, the ZOH lag over one prediction step.
    pub alpha: f64,
    /// `d`, the disturbance in command m/s, constant over the horizon.
    pub d: f64,
    /// `int(speed_model.get('delay', 0)) != 0`: the step acts with the
    /// command already in force (`c`) instead of the new one (`c + ts a`).
    pub delay: bool,
}

/// `(A, B)` of one model step, padded: rows/columns beyond `nx` are zero.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Jacobian {
    pub a: [[f64; MAX_NX]; MAX_NX],
    pub b: [[f64; 2]; MAX_NX],
}

/// `f1tenth_state_fcn_dt_beta(xk, uk, ts, wheelbase, lr)`.
pub fn step_beta(
    xk: &[f64],
    uk: [f64; 2],
    ts: f64,
    wheelbase: f64,
    lr: f64,
) -> Result<[f64; 4], PyError> {
    let (x, y, psi, v) = (xk[0], xk[1], xk[2], xk[3]);
    let (delta, a) = (uk[0], uk[1]);
    // beta = math.atan((lr / wheelbase) * math.tan(delta))
    let beta = pyfloat::atan(pyfloat::div(lr, wheelbase)? * pyfloat::tan(delta)?);
    let xdot = v * pyfloat::cos(psi + beta)?;
    let ydot = v * pyfloat::sin(psi + beta)?;
    // (v / wheelbase) * math.cos(beta) * math.tan(delta)
    let psidot = (pyfloat::div(v, wheelbase)? * pyfloat::cos(beta)?) * pyfloat::tan(delta)?;
    let vdot = a;
    Ok([
        x + ts * xdot,
        y + ts * ydot,
        psi + ts * psidot,
        v + ts * vdot,
    ])
}

/// `f1tenth_state_fcn_dt_beta_jacobian(xk, uk, ts, wheelbase, lr)`: the 4x4
/// `A` and 4x2 `B`, written into the top-left of a padded [`Jacobian`].
pub fn jacobian_beta(
    xk: &[f64],
    uk: [f64; 2],
    ts: f64,
    wheelbase: f64,
    lr: f64,
) -> Result<Jacobian, PyError> {
    let psi = xk[2];
    let v = xk[3];
    let delta = uk[0];

    let k = pyfloat::div(lr, wheelbase)?;
    let t = pyfloat::tan(delta)?;
    let sec2_delta = 1.0 + t * t;
    // denom = 1.0 + (k * t) ** 2 -- a Python float power, i.e. libm pow
    let denom = 1.0 + pyfloat::pow2(k * t)?;
    let beta = pyfloat::atan(k * t);
    let dbeta_ddelta = pyfloat::div(k * sec2_delta, denom)?;

    let cos_pb = pyfloat::cos(psi + beta)?;
    let sin_pb = pyfloat::sin(psi + beta)?;
    let cos_b = pyfloat::cos(beta)?;
    let sin_b = pyfloat::sin(beta)?;

    // -sin_b * dbeta_ddelta * t + cos_b * sec2_delta
    let dgd_ddelta = ((-sin_b) * dbeta_ddelta) * t + cos_b * sec2_delta;

    let mut j = Jacobian {
        a: [[0.0; MAX_NX]; MAX_NX],
        b: [[0.0; 2]; MAX_NX],
    };
    // A = [[1, 0, -ts * v * sin_pb, ts * cos_pb],
    //      [0, 1,  ts * v * cos_pb, ts * sin_pb],
    //      [0, 0,  1,               (ts / wheelbase) * cos_b * t],
    //      [0, 0,  0,               1]]
    j.a[0] = [1.0, 0.0, ((-ts) * v) * sin_pb, ts * cos_pb, 0.0];
    j.a[1] = [0.0, 1.0, (ts * v) * cos_pb, ts * sin_pb, 0.0];
    j.a[2] = [
        0.0,
        0.0,
        1.0,
        (pyfloat::div(ts, wheelbase)? * cos_b) * t,
        0.0,
    ];
    j.a[3] = [0.0, 0.0, 0.0, 1.0, 0.0];
    // B = [[-ts * v * sin_pb * dbeta_ddelta, 0],
    //      [ ts * v * cos_pb * dbeta_ddelta, 0],
    //      [ ts * (v / wheelbase) * dgd_ddelta, 0],
    //      [0, ts]]
    j.b[0] = [(((-ts) * v) * sin_pb) * dbeta_ddelta, 0.0];
    j.b[1] = [((ts * v) * cos_pb) * dbeta_ddelta, 0.0];
    j.b[2] = [(ts * pyfloat::div(v, wheelbase)?) * dgd_ddelta, 0.0];
    j.b[3] = [0.0, ts];
    Ok(j)
}

/// `mpc_solver._model_step(xk, uk, ts, params, speed_model)`: the next state
/// (4 entries without a speed model, 5 with one; the rest of the array 0).
pub fn step(
    xk: &State,
    uk: [f64; 2],
    ts: f64,
    wheelbase: f64,
    lr: f64,
    lag: Option<&LagParams>,
) -> Result<State, PyError> {
    let x4 = step_beta(xk, uk, ts, wheelbase, lr)?;
    let mut out = [x4[0], x4[1], x4[2], x4[3], 0.0];
    if let Some(sm) = lag {
        // f1tenth_state_fcn_dt_beta_lag
        let v = xk[3];
        let c = xk[4];
        let a = uk[1];
        let c_next = c + ts * a;
        let c_eff = if sm.delay { c } else { c_next };
        // v + alpha * (c_eff + d - v)
        out[3] = v + sm.alpha * ((c_eff + sm.d) - v);
        out[4] = c_next;
    }
    Ok(out)
}

/// `mpc_solver._model_jacobian(xk, uk, ts, params, speed_model)`.
pub fn jacobian(
    xk: &State,
    uk: [f64; 2],
    ts: f64,
    wheelbase: f64,
    lr: f64,
    lag: Option<&LagParams>,
) -> Result<Jacobian, PyError> {
    let j4 = jacobian_beta(xk, uk, ts, wheelbase, lr)?;
    let Some(sm) = lag else {
        return Ok(j4);
    };
    // f1tenth_state_fcn_dt_beta_lag_jacobian
    let alpha = sm.alpha;
    let fresh = if sm.delay { 0.0 } else { 1.0 };
    let mut j = Jacobian {
        a: [[0.0; MAX_NX]; MAX_NX],
        b: [[0.0; 2]; MAX_NX],
    };
    for r in 0..3 {
        j.a[r][..4].copy_from_slice(&j4.a[r][..4]);
        j.b[r] = j4.b[r];
    }
    j.a[3][3] = 1.0 - alpha;
    j.a[3][4] = alpha;
    j.a[4][4] = 1.0;
    j.b[3][1] = (alpha * fresh) * ts;
    j.b[4][1] = ts;
    Ok(j)
}

#[cfg(test)]
mod tests {
    use super::*;

    const L: f64 = 0.33;
    const LR: f64 = 0.165;

    #[test]
    fn a_straight_step_integrates_speed_and_acceleration() {
        let x = step_beta(&[1.0, 2.0, 0.0, 0.5], [0.0, 1.0], 0.1, L, LR).unwrap();
        assert_eq!(x, [1.0 + 0.1 * 0.5, 2.0, 0.0, 0.5 + 0.1]);
    }

    #[test]
    fn the_jacobian_matches_finite_differences() {
        let x0 = [0.2, -0.1, 0.4, 0.8, 0.0];
        let u0 = [0.15, 0.3];
        let j = jacobian_beta(&x0, u0, 0.1, L, LR).unwrap();
        let h = 1e-7;
        for c in 0..4 {
            let mut xp = x0;
            let mut xm = x0;
            xp[c] += h;
            xm[c] -= h;
            let fp = step_beta(&xp, u0, 0.1, L, LR).unwrap();
            let fm = step_beta(&xm, u0, 0.1, L, LR).unwrap();
            for r in 0..4 {
                let fd = (fp[r] - fm[r]) / (2.0 * h);
                assert!((fd - j.a[r][c]).abs() < 1e-6, "A[{r}][{c}]");
            }
        }
        for c in 0..2 {
            let mut up = u0;
            let mut um = u0;
            up[c] += h;
            um[c] -= h;
            let fp = step_beta(&x0, up, 0.1, L, LR).unwrap();
            let fm = step_beta(&x0, um, 0.1, L, LR).unwrap();
            for r in 0..4 {
                let fd = (fp[r] - fm[r]) / (2.0 * h);
                assert!((fd - j.b[r][c]).abs() < 1e-6, "B[{r}][{c}]");
            }
        }
    }

    #[test]
    fn the_lag_model_carries_the_command() {
        let lag = LagParams {
            alpha: 0.4,
            d: 0.05,
            delay: false,
        };
        let x = step(
            &[0.0, 0.0, 0.0, 0.5, 0.6],
            [0.0, 1.0],
            0.1,
            L,
            LR,
            Some(&lag),
        )
        .unwrap();
        assert_eq!(x[4], 0.6 + 0.1 * 1.0);
        assert_eq!(x[3], 0.5 + 0.4 * ((x[4] + 0.05) - 0.5));
        let j = jacobian(
            &[0.0, 0.0, 0.0, 0.5, 0.6],
            [0.0, 1.0],
            0.1,
            L,
            LR,
            Some(&lag),
        )
        .unwrap();
        assert_eq!(j.a[3][3], 1.0 - 0.4);
        assert_eq!(j.a[3][4], 0.4);
        assert_eq!(j.b[3][1], 0.4 * 1.0 * 0.1);
        assert_eq!(j.b[4][1], 0.1);
        let delayed = LagParams { delay: true, ..lag };
        let j = jacobian(&[0.0; 5], [0.0, 1.0], 0.1, L, LR, Some(&delayed)).unwrap();
        assert_eq!(j.b[3][1], 0.0);
    }

    #[test]
    fn failures_are_the_pythons() {
        assert_eq!(
            step_beta(&[0.0, 0.0, f64::INFINITY, 1.0], [0.0, 0.0], 0.1, L, LR),
            Err(PyError::MathDomain)
        );
        assert_eq!(
            step_beta(&[0.0; 4], [0.0, 0.0], 0.1, 0.0, LR),
            Err(PyError::ZeroDivision)
        );
        assert!(
            step_beta(&[f64::NAN, 0.0, 0.0, 1.0], [0.0, 0.0], 0.1, L, LR)
                .is_ok_and(|x| x[0].is_nan())
        );
    }
}
