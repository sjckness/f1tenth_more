//! CPython `float` semantics the solver port needs, exception for exception.
//!
//! `vehicle_model.py`, `_solve_rti` and `planner_cost_corridor` do part of
//! their arithmetic on Python floats (`math.tan`, `x ** 2`, `a / b`), part on
//! numpy scalars (`np.float64 ** 2`, `np.exp`), and the two differ where an
//! operation fails: CPython raises (ValueError, ZeroDivisionError,
//! OverflowError), numpy returns inf/NaN and warns. Each helper here is the
//! CPython side; [`crate::npcompat`] has the numpy side.
//!
//! The values are libm's in both: `math.sin`/`cos`/`tan`/`atan` call the
//! platform libm, as `f64::sin` & co. do here.

use std::hint::black_box;

use crate::npcompat::py_trig_domain_ok;

/// A Python exception the reference would raise.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PyError {
    /// `ValueError('math domain error')`: `math.sin/cos/tan` of an infinity.
    MathDomain,
    /// `ZeroDivisionError('float division by zero')`.
    ZeroDivision,
    /// `OverflowError((34, 'Numerical result out of range'))`: `x ** 2`
    /// on a Python float whose square overflows.
    Overflow,
}

/// `math.sin(x)`.
#[inline]
pub fn sin(x: f64) -> Result<f64, PyError> {
    if py_trig_domain_ok(x) {
        Ok(x.sin())
    } else {
        Err(PyError::MathDomain)
    }
}

/// `math.cos(x)`.
#[inline]
pub fn cos(x: f64) -> Result<f64, PyError> {
    if py_trig_domain_ok(x) {
        Ok(x.cos())
    } else {
        Err(PyError::MathDomain)
    }
}

/// `math.tan(x)`: libm never returns an infinity for a finite double, so the
/// only failure is an infinite argument.
#[inline]
pub fn tan(x: f64) -> Result<f64, PyError> {
    if py_trig_domain_ok(x) {
        Ok(x.tan())
    } else {
        Err(PyError::MathDomain)
    }
}

/// `math.atan(x)`: defined everywhere.
#[inline]
pub fn atan(x: f64) -> f64 {
    x.atan()
}

/// `a / b` on Python floats: ZeroDivisionError for `b == 0.0` (either sign),
/// whatever `a` is.
#[inline]
pub fn div(a: f64, b: f64) -> Result<f64, PyError> {
    if b == 0.0 {
        Err(PyError::ZeroDivision)
    } else {
        Ok(a / b)
    }
}

/// `x ** 2` on a Python float (CPython 3.12 `float_pow` with `iw == 2.0`):
///
/// - `nan ** 2` is nan, `(+-inf) ** 2` is `+inf`, `(+-0.0) ** 2` is `+0.0`
///   and `(+-1.0) ** 2` is `1.0`, all without calling libm;
/// - otherwise libm `pow(|x|, 2.0)`; a result of `+-inf` (the square
///   overflowed) raises OverflowError. Underflow to zero or to a subnormal
///   does not raise (checked on linus).
///
/// NOT `x * x`: glibc's `pow` differs from the product in the last bit for
/// about 0.1 % of arguments.
pub fn pow2(x: f64) -> Result<f64, PyError> {
    if x.is_nan() {
        return Ok(x);
    }
    if x.is_infinite() {
        return Ok(f64::INFINITY);
    }
    if x == 0.0 {
        return Ok(0.0);
    }
    let iv = x.abs();
    if iv == 1.0 {
        return Ok(1.0);
    }
    let r = iv.powf(black_box(2.0));
    if r.is_infinite() {
        return Err(PyError::Overflow);
    }
    Ok(r)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trig_refuses_infinities_and_passes_nan() {
        assert_eq!(sin(f64::INFINITY), Err(PyError::MathDomain));
        assert_eq!(cos(f64::NEG_INFINITY), Err(PyError::MathDomain));
        assert_eq!(tan(f64::INFINITY), Err(PyError::MathDomain));
        assert!(sin(f64::NAN).is_ok_and(f64::is_nan));
        assert_eq!(cos(0.3), Ok(0.3_f64.cos()));
        assert_eq!(atan(f64::INFINITY), std::f64::consts::FRAC_PI_2);
    }

    #[test]
    fn division_by_either_zero_raises() {
        assert_eq!(div(1.0, 0.0), Err(PyError::ZeroDivision));
        assert_eq!(div(f64::NAN, -0.0), Err(PyError::ZeroDivision));
        assert_eq!(div(1.0, 4.0), Ok(0.25));
    }

    #[test]
    fn pow2_follows_float_pow() {
        assert_eq!(pow2(0.0).map(f64::to_bits), Ok(0));
        assert_eq!(pow2(-0.0).map(f64::to_bits), Ok(0));
        assert_eq!(pow2(-1.0), Ok(1.0));
        assert_eq!(pow2(f64::NEG_INFINITY), Ok(f64::INFINITY));
        assert!(pow2(f64::NAN).is_ok_and(f64::is_nan));
        assert_eq!(pow2(1e155), Err(PyError::Overflow));
        assert_eq!(pow2(-1e155), Err(PyError::Overflow));
        assert_eq!(pow2(1e-160), Ok(1e-160_f64.powf(2.0)));
        assert_eq!(pow2(1e-170), Ok(0.0));
        assert_eq!(pow2(-3.5), Ok(12.25));
    }
}
