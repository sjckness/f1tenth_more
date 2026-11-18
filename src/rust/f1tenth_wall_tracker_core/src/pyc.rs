//! CPython 3.12 and numpy 1.26 semantics, operation for operation.
//!
//! The tracker port is bit-identical to `multi_wall_tracker.py` only if every
//! operation rounds the way the Python's does, in the same order. Each
//! function here reproduces one Python builtin or numpy operation as the
//! installed CPython 3.12.3 / numpy 1.26.4 (apt, netlib reference BLAS, no
//! AVX-512 loops) compute it on linus. Measured there (scratch numcheck, 201k
//! values each):
//!
//! - `np.cos`, `np.sin`, `np.arctan2`, `np.hypot`, `math.cos`, `math.atan2`
//!   are libm's: 0 differences against glibc. So are `f64::cos` etc. here;
//! - `math.hypot` is NOT libm's (1205 differences): CPython has its own
//!   correctly-rounding `vector_norm`, ported in [`py_hypot`];
//! - Python's `x ** 2` on a float is libm `pow(x, 2.0)`, NOT `x * x` (176
//!   differences): [`py_pow2`]. numpy's `x ** 2` on an array is
//!   `np.square`, i.e. `x * x`;
//! - numpy's float32 `np.exp` is numpy's own (79k differences from expf):
//!   nothing here computes it; the rectangle grid comes from Python.
//!
//! Rust never contracts `a * b + c` into an FMA or reassociates, at any
//! opt-level, so `+ - * /` and `sqrt` are the same IEEE-754 operations.

use std::f64::consts::PI;
use std::hint::black_box;

/// `2.0 * math.pi` (corridor_perception.geometry.TWO_PI).
pub const TWO_PI: f64 = 2.0 * PI;

/// Python's builtin `max(a, b)`: `a` unless `b > a` (the first argument wins
/// a tie and every false comparison, so `max(nan, 1.0)` is nan and
/// `max(1.0, nan)` is 1.0).
#[inline]
pub fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python's builtin `min(a, b)`: `a` unless `b < a`.
#[inline]
pub fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// Python's `max(seq)` over a non-empty sequence: left fold of [`py_max`].
pub fn py_max_seq(v: &[f64]) -> f64 {
    let mut it = v.iter();
    let mut best = it.next().copied().unwrap_or(f64::NAN);
    for &x in it {
        best = py_max(best, x);
    }
    best
}

/// Python's `min(seq)` over a non-empty sequence: left fold of [`py_min`].
pub fn py_min_seq(v: &[f64]) -> f64 {
    let mut it = v.iter();
    let mut best = it.next().copied().unwrap_or(f64::NAN);
    for &x in it {
        best = py_min(best, x);
    }
    best
}

/// `np.maximum(a, b)` for float64: `(a >= b || isnan(a)) ? a : b`.
#[inline]
pub fn np_maximum(a: f64, b: f64) -> f64 {
    if a >= b || a.is_nan() {
        a
    } else {
        b
    }
}

/// `np.minimum(a, b)` for float64: `(a <= b || isnan(a)) ? a : b`.
#[inline]
pub fn np_minimum(a: f64, b: f64) -> f64 {
    if a <= b || a.is_nan() {
        a
    } else {
        b
    }
}

/// `x.min()` / `x.max()` of a non-empty float64 array (numpy reductions:
/// NaN if any element is NaN). `None` when empty (numpy raises).
pub fn np_min_reduce(v: &[f64]) -> Option<f64> {
    let mut it = v.iter();
    let mut best = *it.next()?;
    for &x in it {
        if best.is_nan() {
            break;
        }
        best = np_minimum(best, x);
    }
    Some(best)
}

/// See [`np_min_reduce`].
pub fn np_max_reduce(v: &[f64]) -> Option<f64> {
    let mut it = v.iter();
    let mut best = *it.next()?;
    for &x in it {
        if best.is_nan() {
            break;
        }
        best = np_maximum(best, x);
    }
    Some(best)
}

/// Python's float `x % y` (CPython `float_rem`), which numpy's float64
/// `np.remainder` reproduces (`npy_divmod`): fmod, then moved into the sign
/// of `y`; a zero remainder takes the sign of `y`.
#[inline]
pub fn py_mod(x: f64, y: f64) -> f64 {
    let m = x % y;
    if m != 0.0 || m.is_nan() {
        if m.is_nan() {
            return m;
        }
        if (y < 0.0) != (m < 0.0) {
            m + y
        } else {
            m
        }
    } else {
        0.0_f64.copysign(y)
    }
}

/// corridor_perception.geometry.wrap_pi: `(a + pi) % (2 pi) - pi`, the same
/// three operations for a scalar (Python floats) and an array (numpy).
#[inline]
pub fn wrap_pi(a: f64) -> f64 {
    py_mod(a + PI, TWO_PI) - PI
}

/// Python's `x ** 2` for a float: CPython `float_pow` hands it to libm
/// `pow(|x|, 2.0)` (its special cases agree with pow's for an even integer
/// exponent). `black_box` keeps LLVM from rewriting `pow(x, 2.0)` as `x * x`,
/// which rounds differently in ~0.1 % of cases on glibc.
#[inline]
pub fn py_pow2(x: f64) -> f64 {
    if x.is_nan() {
        return x;
    }
    if x == 0.0 {
        return 0.0;
    }
    x.abs().powf(black_box(2.0))
}

#[derive(Clone, Copy)]
struct DoubleLength {
    hi: f64,
    lo: f64,
}

#[inline]
fn dl_fast_sum(a: f64, b: f64) -> DoubleLength {
    let x = a + b;
    let y = (a - x) + b;
    DoubleLength { hi: x, lo: y }
}

#[inline]
fn dl_mul(x: f64, y: f64) -> DoubleLength {
    let z = x * y;
    let zz = x.mul_add(y, -z);
    DoubleLength { hi: z, lo: zz }
}

/// CPython 3.12 `math.hypot(x, y)` (Modules/mathmodule.c `math_hypot` +
/// `vector_norm`): exact scaling by a power of two, lossless squares
/// (`dl_mul`, an fma), compensated sum, one Newton correction. Not libm's.
pub fn py_hypot(x: f64, y: f64) -> f64 {
    let mut vec = [x.abs(), y.abs()];
    let mut max = 0.0_f64;
    let mut found_nan = false;
    for &v in &vec {
        found_nan |= v.is_nan();
        if v > max {
            max = v;
        }
    }
    vector_norm(&mut vec, max, found_nan)
}

fn frexp_exp(x: f64) -> i32 {
    // frexp's exponent for a finite non-zero x: x = m * 2^e, 0.5 <= |m| < 1.
    let bits = x.to_bits();
    let exp = ((bits >> 52) & 0x7ff) as i32;
    if exp == 0 {
        // subnormal: normalise
        let m = x * f64::from_bits(0x4350_0000_0000_0000); // 2^54
        let e2 = ((m.to_bits() >> 52) & 0x7ff) as i32;
        e2 - 1022 - 54
    } else {
        exp - 1022
    }
}

fn ldexp_one(e: i32) -> f64 {
    // 2^e for the range vector_norm uses (|e| <= 1074).
    let mut r = 1.0_f64;
    let mut k = e;
    while k > 1000 {
        r *= f64::from_bits(0x7e70_0000_0000_0000); // 2^1000
        k -= 1000;
    }
    while k < -1000 {
        r *= f64::from_bits(0x0170_0000_0000_0000); // 2^-1000
        k += 1000;
    }
    r * f64::from_bits(((k + 1023) as u64) << 52)
}

fn vector_norm(vec: &mut [f64], max: f64, found_nan: bool) -> f64 {
    if max.is_infinite() {
        return max;
    }
    if found_nan {
        return f64::NAN;
    }
    if max == 0.0 || vec.len() <= 1 {
        return max;
    }
    let max_e = frexp_exp(max);
    if max_e < -1023 {
        // When max_e < -1023, ldexp(1.0, -max_e) would overflow.
        for v in vec.iter_mut() {
            *v /= f64::MIN_POSITIVE;
        }
        return f64::MIN_POSITIVE * vector_norm(vec, max / f64::MIN_POSITIVE, found_nan);
    }
    let scale = ldexp_one(-max_e);
    let mut csum = 1.0_f64;
    let mut frac1 = 0.0_f64;
    let mut frac2 = 0.0_f64;
    for &v in vec.iter() {
        let x = v * scale;
        let pr = dl_mul(x, x);
        let sm = dl_fast_sum(csum, pr.hi);
        csum = sm.hi;
        frac1 += pr.lo;
        frac2 += sm.lo;
    }
    let mut h = (csum - 1.0 + (frac1 + frac2)).sqrt();
    let pr = dl_mul(-h, h);
    let sm = dl_fast_sum(csum, pr.hi);
    csum = sm.hi;
    frac1 += pr.lo;
    frac2 += sm.lo;
    let x = csum - 1.0 + (frac1 + frac2);
    h += x / (2.0 * h);
    h / scale
}

/// CPython's `math.atan2` (`m_atan2`): its own NaN / infinity / zero cases,
/// then libm `atan2`.
pub fn py_atan2(y: f64, x: f64) -> f64 {
    if x.is_nan() || y.is_nan() {
        return f64::NAN;
    }
    if y.is_infinite() {
        if x.is_infinite() {
            if 1.0_f64.copysign(x) == 1.0 {
                return (0.25 * PI).copysign(y);
            }
            return (0.75 * PI).copysign(y);
        }
        return (0.5 * PI).copysign(y);
    }
    if x.is_infinite() || y == 0.0 {
        if 1.0_f64.copysign(x) == 1.0 {
            return 0.0_f64.copysign(y);
        }
        return PI.copysign(y);
    }
    y.atan2(x)
}

/// The two-term dot product numpy computes for every 2x2 `@` here (netlib
/// `ddot` / `dgemv` / `dgemm`, all of which start from 0.0 and add in index
/// order). With two terms the order cannot matter; the leading `0.0 +` only
/// turns a `-0.0` into `+0.0`, as BLAS does.
#[inline]
pub fn dot2(a0: f64, a1: f64, b0: f64, b1: f64) -> f64 {
    (0.0 + a0 * b0) + a1 * b1
}

/// CPython 3.12 `sum(floats)` with the default start 0: the int 0 plus the
/// first float, then Arnold Neumaier's compensated summation, the
/// compensation added at the end when it is non-zero and finite. `None` for
/// an empty sequence (Python returns the int 0, which no caller here reaches).
pub fn py_sum(v: &[f64]) -> Option<f64> {
    let mut it = v.iter();
    let first = *it.next()?;
    let mut f_result = 0.0 + first;
    let mut c = 0.0_f64;
    for &x in it {
        let t = f_result + x;
        if f_result.abs() >= x.abs() {
            c += (f_result - t) + x;
        } else {
            c += (x - t) + f_result;
        }
        f_result = t;
    }
    if c != 0.0 && c.is_finite() {
        f_result += c;
    }
    Some(f_result)
}

/// numpy's `np.arange(start, stop, step)` for Python floats
/// (`PyArray_ArangeObj`): `len = ceil((stop - start) / step)`; element 0 is
/// `start`, element 1 is `start + step`, and every later element is
/// `start + i * delta` with `delta = (start + step) - start` (DOUBLE_fill).
pub fn np_arange(start: f64, stop: f64, step: f64) -> Vec<f64> {
    let len_f = ((stop - start) / step).ceil();
    if len_f.is_nan() || len_f <= 0.0 {
        return Vec::new();
    }
    // A length numpy could not allocate is refused there; cap it here.
    let len = if len_f > 1.0e7 {
        10_000_000
    } else {
        len_f as usize
    };
    let mut out = Vec::with_capacity(len);
    out.push(start);
    if len == 1 {
        return out;
    }
    let second = start + step;
    out.push(second);
    let delta = second - start;
    for i in 2..len {
        out.push(start + (i as f64) * delta);
    }
    out
}

/// `np.linspace(start, stop, num)` (endpoint=True), numpy 1.26.
pub fn np_linspace(start: f64, stop: f64, num: usize) -> Vec<f64> {
    let mut y: Vec<f64> = (0..num).map(|i| i as f64).collect();
    if num == 0 {
        return y;
    }
    let delta = stop - start;
    let div = num - 1;
    if div > 0 {
        let div_f = div as f64;
        let step = delta / div_f;
        if step == 0.0 {
            for v in &mut y {
                *v = *v / div_f * delta;
            }
        } else {
            for v in &mut y {
                *v *= step;
            }
        }
    } else {
        for v in &mut y {
            *v *= delta;
        }
    }
    for v in &mut y {
        *v += start;
    }
    if num > 1 {
        y[num - 1] = stop;
    }
    y
}

/// numpy's float `less` for sorting and searching: NaNs sort last.
#[inline]
fn np_less(a: f64, b: f64) -> bool {
    a < b || (b.is_nan() && !a.is_nan())
}

/// `np.searchsorted(arr, keys)` (side='left') for float64, numpy 1.26
/// `binsearch<side::left>` verbatim, including the bounds it carries from
/// one key to the next (for a sorted `arr` that cannot change an answer; on
/// an unsorted one the path taken IS the answer).
pub fn np_searchsorted_left(arr: &[f64], keys: &[f64]) -> Vec<usize> {
    let arr_len = arr.len();
    let mut out = Vec::with_capacity(keys.len());
    if keys.is_empty() {
        return out;
    }
    let mut min_idx: usize = 0;
    let mut max_idx: usize = arr_len;
    let mut last_key_val = keys[0];
    for &key_val in keys {
        if np_less(last_key_val, key_val) {
            max_idx = arr_len;
        } else {
            min_idx = 0;
            max_idx = if max_idx < arr_len {
                max_idx + 1
            } else {
                arr_len
            };
        }
        last_key_val = key_val;
        while min_idx < max_idx {
            let mid_idx = min_idx + ((max_idx - min_idx) >> 1);
            if np_less(arr[mid_idx], key_val) {
                min_idx = mid_idx + 1;
            } else {
                max_idx = mid_idx;
            }
        }
        out.push(min_idx);
    }
    out
}

/// `np.argmin(a)` for float64: the first index of the minimum, the first NaN
/// if there is one. `None` for an empty slice (numpy raises).
pub fn np_argmin(a: &[f64]) -> Option<usize> {
    let first = *a.first()?;
    let mut best = first;
    let mut idx = 0;
    if best.is_nan() {
        return Some(0);
    }
    for (i, &v) in a.iter().enumerate().skip(1) {
        #[allow(clippy::neg_cmp_op_on_partial_ord)]
        let replace = !(v >= best);
        if replace {
            best = v;
            idx = i;
            if best.is_nan() {
                break;
            }
        }
    }
    Some(idx)
}

/// Python's `sorted()` of float pairs (lexicographic, stable). Exactly
/// Python's order whenever no element is NaN (a NaN makes Python's
/// comparisons inconsistent and its order depends on timsort's path).
pub fn py_sort_pairs(v: &mut [(f64, f64)]) {
    v.sort_by(|a, b| {
        if a.0 < b.0 {
            std::cmp::Ordering::Less
        } else if b.0 < a.0 {
            std::cmp::Ordering::Greater
        } else if a.1 < b.1 {
            std::cmp::Ordering::Less
        } else if b.1 < a.1 {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });
}

/// A stable sort by a float key, Python `list.sort(key=...)` for NaN-free keys.
pub fn py_sort_by_key<T>(v: &mut [T], key: impl Fn(&T) -> f64) {
    v.sort_by(|a, b| {
        let (ka, kb) = (key(a), key(b));
        if ka < kb {
            std::cmp::Ordering::Less
        } else if kb < ka {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn python_max_min_keep_the_first_argument() {
        assert_eq!(py_max(1.0, 2.0), 2.0);
        assert!(py_max(f64::NAN, 1.0).is_nan());
        assert_eq!(py_max(1.0, f64::NAN), 1.0);
        assert_eq!(py_min(2.0, 1.0), 1.0);
        assert!(py_min(f64::NAN, 1.0).is_nan());
        assert_eq!(py_max_seq(&[1.0, 3.0, 2.0]), 3.0);
        assert_eq!(py_min_seq(&[1.0, 3.0, -2.0]), -2.0);
    }

    #[test]
    fn py_mod_follows_the_sign_of_the_divisor() {
        assert_eq!(py_mod(-1.0, TWO_PI), -1.0 + TWO_PI);
        assert_eq!(py_mod(1.0, TWO_PI), 1.0);
        assert_eq!(py_mod(0.0, TWO_PI).to_bits(), 0.0_f64.to_bits());
        assert_eq!(py_mod(-0.0, TWO_PI).to_bits(), 0.0_f64.to_bits());
        assert!(py_mod(f64::NAN, TWO_PI).is_nan());
        assert!(py_mod(f64::INFINITY, TWO_PI).is_nan());
    }

    #[test]
    fn wrap_pi_lands_in_minus_pi_to_pi() {
        for k in -20..20 {
            let a = f64::from(k) * 0.7;
            let w = wrap_pi(a);
            assert!((-PI..PI).contains(&w), "{a} -> {w}");
        }
        assert_eq!(wrap_pi(PI), -PI);
    }

    #[test]
    fn hypot_matches_known_values() {
        assert_eq!(py_hypot(3.0, 4.0), 5.0);
        assert_eq!(py_hypot(0.0, 0.0), 0.0);
        assert_eq!(py_hypot(-2.0, 0.0), 2.0);
        assert!(py_hypot(f64::NAN, 1.0).is_nan());
        assert_eq!(py_hypot(f64::INFINITY, f64::NAN), f64::INFINITY);
        assert_eq!(py_hypot(1e-310, 1e-310), 1e-310 * std::f64::consts::SQRT_2);
    }

    #[test]
    fn sum_is_neumaier() {
        // 1e100 + 1 - 1e100: naive 0.0, Neumaier 1.0 (Python 3.12).
        assert_eq!(py_sum(&[1e100, 1.0, -1e100]), Some(1.0));
        assert_eq!(py_sum(&[0.1, 0.2, 0.3]), Some(0.6));
        assert_eq!(py_sum(&[]), None);
        assert_eq!(py_sum(&[-0.0]).map(f64::to_bits), Some(0.0_f64.to_bits()));
    }

    #[test]
    fn arange_uses_the_second_element_as_delta() {
        let v = np_arange(0.025, 1.0, 0.05);
        assert_eq!(v.len(), 20);
        let delta = (0.025 + 0.05) - 0.025;
        assert_eq!(v[7], 0.025 + 7.0 * delta);
        assert!(np_arange(1.0, 0.5, 0.05).is_empty());
        assert_eq!(np_arange(0.0, 0.01, 0.05), vec![0.0]);
    }

    #[test]
    fn searchsorted_left_on_sorted_data() {
        let a = [0.0, 1.0, 2.0, 3.0];
        assert_eq!(
            np_searchsorted_left(&a, &[-1.0, 0.0, 0.5, 3.0, 9.0, f64::NAN]),
            vec![0, 0, 1, 3, 4, 4]
        );
        assert_eq!(np_searchsorted_left(&a, &[2.5, 1.0]), vec![3, 1]);
    }

    #[test]
    fn argmin_takes_the_first_minimum_and_nan_first() {
        assert_eq!(np_argmin(&[3.0, 1.0, 1.0]), Some(1));
        assert_eq!(np_argmin(&[3.0, f64::NAN, 0.0]), Some(1));
        assert_eq!(np_argmin(&[]), None);
    }

    #[test]
    fn pow2_is_libm_pow() {
        let x = 1.234_567_891_234_5_f64;
        assert_eq!(py_pow2(x), x.powf(2.0));
        assert_eq!(py_pow2(-x), py_pow2(x));
        assert!(py_pow2(f64::NAN).is_nan());
    }
}
