//! numpy 1.26 and CPython semantics, operation for operation.
//!
//! The corridor port is bit-identical to `corridor_geometry.py` only if every
//! array operation rounds the same way numpy's does, in the same order. Each
//! function here reproduces one numpy (or Python builtin) operation as the
//! installed numpy 1.26.4 computes it on linus (apt `python3-numpy`, netlib
//! reference BLAS, no AVX-512, so no SVML loops):
//!
//! - elementwise `+ - * /` and `sqrt`: IEEE-754, the same in both. Rust never
//!   contracts `a * b + c` into an FMA or reassociates, at any opt-level;
//! - `x ** 2` is `np.square` (numpy's scalar-power fast path), i.e. `x * x`;
//!   `x ** 3` is `np.power`, i.e. libm `pow(x, 3.0)` ([`pow3`]): NOT `x*x*x`;
//! - `np.sin`, `np.cos`, `np.hypot` are libm's (checked on linus over 34k
//!   values: 0 differences), and so are `f64::sin`/`cos`/`hypot` here;
//! - reductions and searches (`cumsum`, `argmin`, `max`, `interp`, `dot`) are
//!   sequential with numpy's own tie and NaN rules, ported below.
//!
//! The parity harness (tools/rust_parity/corridor) compares the whole port with
//! the Python bit for bit; the unit tests here pin the semantics (NaN
//! propagation, tie-breaking, edge cases) that a random sweep rarely reaches.

use std::hint::black_box;

/// Python's builtin `max(a, b)` for two floats: `a` unless `b > a`.
///
/// Not `f64::max`: Python keeps the FIRST argument on a tie and whenever the
/// comparison is false, so `max(nan, 1e-6)` is nan and `max(1e-6, nan)` is
/// 1e-6.
#[inline]
pub fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python's builtin `min(a, b)` for two floats: `a` unless `b < a`.
#[inline]
pub fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `np.maximum(a, b)` for float64 (numpy 1.26 `scalar_max`):
/// `(a >= b || isnan(a)) ? a : b`, so a NaN in either argument propagates.
#[inline]
pub fn np_maximum(a: f64, b: f64) -> f64 {
    if a >= b || a.is_nan() {
        a
    } else {
        b
    }
}

/// `np.clip(x, lo, hi)` for float64 (numpy 1.26 `_NPY_CLIP`):
/// `_NPY_MIN(_NPY_MAX(x, lo), hi)` with `_NPY_MAX(a, b) = isnan(a) ? a :
/// (a > b ? a : b)` and `_NPY_MIN(a, b) = isnan(a) ? a : (a < b ? a : b)`.
/// A NaN `x` stays NaN; `clip(-0.0, 0.0, 1.0)` is `+0.0`.
#[inline]
pub fn np_clip(x: f64, lo: f64, hi: f64) -> f64 {
    let t = if x.is_nan() || x > lo { x } else { lo };
    if t.is_nan() || t < hi {
        t
    } else {
        hi
    }
}

/// `np.power(x, 3)` for float64: libm `pow(x, 3.0)`.
///
/// `black_box` keeps LLVM from seeing the constant exponent. It does not turn
/// `pow(x, 3.0)` into `x * x * x` without fast-math today, but nothing
/// guarantees that, and the two differ in the last bit.
#[inline]
pub fn pow3(x: f64) -> f64 {
    x.powf(black_box(3.0))
}

/// `np.linspace(start, stop, num)` (endpoint=True), numpy 1.26
/// `function_base.linspace`:
///
/// ```text
/// div = num - 1; delta = stop - start; y = arange(num)
/// if div > 0:
///     step = delta / div
///     if step == 0: y = y / div * delta
///     else:         y = y * step
/// else:             y = y * delta
/// y += start;  y[-1] = stop  (num > 1)
/// ```
pub fn linspace(start: f64, stop: f64, num: usize) -> Vec<f64> {
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

/// `np.cumsum(a)` for float64: `add.accumulate`, strictly sequential
/// (`out[i] = out[i - 1] + a[i]`); numpy's pairwise summation applies to
/// `sum`, never to `cumsum`.
pub fn cumsum(a: &[f64]) -> Vec<f64> {
    let mut out = Vec::with_capacity(a.len());
    let mut acc = 0.0;
    for (i, &v) in a.iter().enumerate() {
        acc = if i == 0 { v } else { acc + v };
        out.push(acc);
    }
    out
}

/// `np.diff(a)`: `a[i + 1] - a[i]`, one shorter than `a`.
pub fn diff(a: &[f64]) -> Vec<f64> {
    a.windows(2).map(|w| w[1] - w[0]).collect()
}

/// `np.gradient(f)` of a 1-D array: unit spacing, `edge_order=1` (the
/// defaults). Interior `(f[i+1] - f[i-1]) / 2.0`, then the two one-sided
/// ends `(f[1] - f[0]) / 1.0` and `(f[-1] - f[-2]) / 1.0`, written in that
/// order (numpy 1.26 `function_base.gradient`). Needs `len >= 2`, which numpy
/// enforces with a ValueError; here a shorter input returns a copy of itself
/// unchanged (no caller can pass one: corridor builders refuse n < 2).
pub fn gradient(f: &[f64]) -> Vec<f64> {
    let n = f.len();
    let mut out = vec![0.0; n];
    if n < 2 {
        out.copy_from_slice(f);
        return out;
    }
    let ax_dx = 1.0_f64;
    let two_dx = 2.0 * ax_dx;
    for i in 1..n - 1 {
        out[i] = (f[i + 1] - f[i - 1]) / two_dx;
    }
    out[0] = (f[1] - f[0]) / ax_dx;
    out[n - 1] = (f[n - 1] - f[n - 2]) / ax_dx;
    out
}

/// `np.argmin(a)` for float64: the FIRST index of the minimum; a NaN counts as
/// the minimum, so the first NaN wins (numpy's `DOUBLE_argmin`: replace while
/// `!(a[i] >= current)`, stop at a NaN). `None` for an empty slice (numpy
/// raises "attempt to get argmin of an empty sequence").
pub fn argmin(a: &[f64]) -> Option<usize> {
    let first = *a.first()?;
    let mut best = first;
    let mut idx = 0;
    if best.is_nan() {
        return Some(0);
    }
    for (i, &v) in a.iter().enumerate().skip(1) {
        // Negated comparison, as numpy writes it, for the NaN handling.
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

/// `a.max()` for float64 (`np.maximum.reduce`): NaN if any element is NaN,
/// otherwise the largest. `None` for an empty slice (numpy raises).
///
/// Which of two equal-valued signed zeros is returned is not specified by
/// numpy (its SIMD and scalar paths differ); the corridor only reduces
/// `abs(...) / speed**3 >= 0`, where it cannot matter.
pub fn max_reduce(a: &[f64]) -> Option<f64> {
    let mut it = a.iter();
    let mut best = *it.next()?;
    for &v in it {
        if best.is_nan() {
            break;
        }
        best = np_maximum(best, v);
    }
    Some(best)
}

/// numpy's `float64 @ float64` for two 2-vectors: `DOUBLE_dot` through the
/// netlib reference BLAS on linus. `cblas_ddot` accumulates from `0.0`
/// sequentially (`n < 5`: `dtemp = dtemp + dx(i) * dy(i)`), and DOUBLE_dot
/// adds that to a `sum = 0.` of its own. The leading zeros matter only for
/// signed zeros: `(-1, -1) @ (0, 0)` is `+0.0`, not `-0.0`.
#[inline]
pub fn dot2(a: [f64; 2], b: [f64; 2]) -> f64 {
    let mut dtemp = 0.0;
    dtemp += a[0] * b[0];
    dtemp += a[1] * b[1];
    let mut sum = 0.0;
    sum += dtemp;
    sum
}

/// numpy's `LIKELY_IN_CACHE_SIZE` (compiled_base.c).
const LIKELY_IN_CACHE_SIZE: isize = 8;

/// numpy 1.26 `binary_search_with_guess` (compiled_base.c), verbatim:
/// the index `j` with `arr[j] <= key < arr[j + 1]`, `-1` below the range and
/// `len` above it. The guess and its cache-window shortcuts are kept as
/// written: for a sorted `arr` they cannot change the answer, but `np.interp`
/// does not check that `xp` is sorted, and on an unsorted or NaN-carrying `xp`
/// the path taken IS the answer.
pub fn binary_search_with_guess(key: f64, arr: &[f64], guess: isize) -> isize {
    let len = arr.len() as isize;
    let at = |i: isize| arr[i as usize];
    let mut imin: isize = 0;
    let mut imax: isize = len;
    let mut guess = guess;

    if key > at(len - 1) {
        return len;
    } else if key < at(0) {
        return -1;
    }

    // len <= 4: linear search (key >= arr[0] is known here).
    if len <= 4 {
        let mut i: isize = 1;
        while i < len && key >= at(i) {
            i += 1;
        }
        return i - 1;
    }

    if guess > len - 3 {
        guess = len - 3;
    }
    if guess < 1 {
        guess = 1;
    }

    // check most likely values: guess - 1, guess, guess + 1
    if key < at(guess) {
        if key < at(guess - 1) {
            imax = guess - 1;
            // last attempt to restrict search to items in cache
            if guess > LIKELY_IN_CACHE_SIZE && key >= at(guess - LIKELY_IN_CACHE_SIZE) {
                imin = guess - LIKELY_IN_CACHE_SIZE;
            }
        } else {
            // key >= arr[guess - 1]
            return guess - 1;
        }
    } else {
        // key >= arr[guess]
        if key < at(guess + 1) {
            return guess;
        }
        // key >= arr[guess + 1]
        if key < at(guess + 2) {
            return guess + 1;
        }
        // key >= arr[guess + 2]
        imin = guess + 2;
        // last attempt to restrict search to items in cache
        if guess < len - LIKELY_IN_CACHE_SIZE - 1 && key < at(guess + LIKELY_IN_CACHE_SIZE) {
            imax = guess + LIKELY_IN_CACHE_SIZE;
        }
    }

    // finally, find index by bisection
    while imin < imax {
        let imid = imin + ((imax - imin) >> 1);
        if key >= at(imid) {
            imin = imid + 1;
        } else {
            imax = imid;
        }
    }
    imin - 1
}

/// `np.interp(x, xp, fp)` with the default `left`/`right` (numpy 1.26
/// `arr_interp`, compiled_base.c), verbatim: NaN keys pass through; outside
/// the range the end values; on a sample point `fp[j]` exactly; otherwise
/// `slope * (x - xp[j]) + fp[j]`, retried from the right end of the interval
/// if that is NaN. The slope is computed per key (numpy precomputes the
/// slopes only when `len(xp) <= len(x)`, with the same arithmetic).
///
/// `None` if `xp` is empty or `xp` and `fp` differ in length (numpy raises
/// ValueError).
pub fn interp(x: &[f64], xp: &[f64], fp: &[f64]) -> Option<Vec<f64>> {
    let lenxp = xp.len();
    if lenxp == 0 || fp.len() != lenxp {
        return None;
    }
    let lval = fp[0];
    let rval = fp[lenxp - 1];
    let mut out = Vec::with_capacity(x.len());
    if lenxp == 1 {
        let xp_val = xp[0];
        let fp_val = fp[0];
        for &x_val in x {
            out.push(if x_val < xp_val {
                lval
            } else if x_val > xp_val {
                rval
            } else {
                fp_val
            });
        }
        return Some(out);
    }
    let len = lenxp as isize;
    let mut j: isize = 0;
    for &x_val in x {
        if x_val.is_nan() {
            out.push(x_val);
            continue;
        }
        j = binary_search_with_guess(x_val, xp, j);
        let v = if j == -1 {
            lval
        } else if j == len {
            rval
        } else if j == len - 1 {
            fp[j as usize]
        } else {
            let ju = j as usize;
            if xp[ju] == x_val {
                // Avoid potential non-finite interpolation
                fp[ju]
            } else {
                let slope = (fp[ju + 1] - fp[ju]) / (xp[ju + 1] - xp[ju]);
                // If we get nan in one direction, try the other
                let mut r = slope * (x_val - xp[ju]) + fp[ju];
                if r.is_nan() {
                    r = slope * (x_val - xp[ju + 1]) + fp[ju + 1];
                    if r.is_nan() && fp[ju] == fp[ju + 1] {
                        r = fp[ju];
                    }
                }
                r
            }
        };
        out.push(v);
    }
    Some(out)
}

/// CPython's `math.atan2` (`m_atan2`, Modules/mathmodule.c): its own handling
/// of NaN, infinities and zeros, then libm `atan2`. The special cases agree
/// with C99's Annex F; they are spelled out so the result does not depend on
/// the libm's choice of NaN.
pub fn py_atan2(y: f64, x: f64) -> f64 {
    use std::f64::consts::PI;
    if x.is_nan() || y.is_nan() {
        return f64::NAN;
    }
    if y.is_infinite() {
        if x.is_infinite() {
            if 1.0_f64.copysign(x) == 1.0 {
                // atan2(+-inf, +inf) == +-pi/4
                return (0.25 * PI).copysign(y);
            }
            // atan2(+-inf, -inf) == +-pi*3/4
            return (0.75 * PI).copysign(y);
        }
        // atan2(+-inf, x) == +-pi/2 for finite x
        return (0.5 * PI).copysign(y);
    }
    if x.is_infinite() || y == 0.0 {
        if 1.0_f64.copysign(x) == 1.0 {
            // atan2(+-y, +inf) = atan2(+-0, +x) = +-0.
            return 0.0_f64.copysign(y);
        }
        // atan2(+-y, -inf) = atan2(+-0., -x) = +-pi.
        return PI.copysign(y);
    }
    y.atan2(x)
}

/// CPython's `math.sin` / `math.cos` refuse an infinite argument ("math
/// domain error", ValueError): libm returns NaN for it, and CPython raises
/// whenever the result is NaN but the argument is not. For a finite or NaN
/// argument they return libm's value.
#[inline]
pub fn py_trig_domain_ok(x: f64) -> bool {
    !x.is_infinite()
}

// ---------------------------------------------------------------------------
// R2 (the solver core): the numpy scalar and BLAS operations `_solve_rti`,
// `_linearize_obstacle_cost_rti` and `planner_cost_corridor` use.
// ---------------------------------------------------------------------------

/// `np.minimum(a, b)` for float64 (numpy 1.26 `scalar_min`):
/// `(a <= b || isnan(a)) ? a : b`, so a NaN in either argument propagates.
#[inline]
pub fn np_minimum(a: f64, b: f64) -> f64 {
    if a <= b || a.is_nan() {
        a
    } else {
        b
    }
}

/// `np.minimum.reduce(a)` (`a.min()`): NaN if any element is NaN, otherwise
/// the smallest. `None` for an empty slice (numpy raises). Which of two
/// equal-valued signed zeros comes back is not specified by numpy; the
/// solver only reduces `hypot(...) - r`, which is never `-0.0`.
pub fn min_reduce(a: &[f64]) -> Option<f64> {
    let mut it = a.iter();
    let mut best = *it.next()?;
    for &v in it {
        if best.is_nan() {
            break;
        }
        best = np_minimum(best, v);
    }
    Some(best)
}

/// `x ** y` on a numpy float64 SCALAR (numpy 1.26 scalarmath
/// `double_power`): libm `pow(x, y)`, with no Python-level checks (an
/// overflow is `inf` and a warning, not an exception).
///
/// NOT `x * x` for `y == 2`: glibc's `pow` is not correctly rounded and
/// differs from the product in about 0.1 % of arguments (measured on linus).
/// Arrays are different: `array ** 2` is `np.square` (numpy's scalar-power
/// fast path), i.e. `x * x`. `black_box` keeps LLVM from rewriting
/// `pow(x, 2.0)` into `x * x`, which it does without fast-math.
#[inline]
pub fn np_pow(x: f64, y: f64) -> f64 {
    x.powf(black_box(y))
}

/// numpy's `float64 @ float64` of two 1-D vectors: `DOUBLE_dot` through
/// `cblas_ddot` (netlib reference BLAS on linus). ddot sums the first
/// `n % 5` products sequentially from `0.0`, then five at a time,
/// `dtemp + p1 + p2 + p3 + p4 + p5` left to right (gfortran without
/// -ffast-math keeps that order). DOUBLE_dot adds the result to a `sum = 0.`
/// of its own. For n < 5 this is a plain sequential sum from `+0.0`.
pub fn blas_dot(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len());
    let mut dtemp = 0.0;
    let m = n % 5;
    for i in 0..m {
        dtemp += a[i] * b[i];
    }
    let mut i = m;
    while i + 5 <= n {
        dtemp = dtemp
            + a[i] * b[i]
            + a[i + 1] * b[i + 1]
            + a[i + 2] * b[i + 2]
            + a[i + 3] * b[i + 3]
            + a[i + 4] * b[i + 4];
        i += 5;
    }
    let mut sum = 0.0;
    sum += dtemp;
    sum
}

/// numpy's `A @ x` for a C-contiguous float64 matrix `A` (`rows x cols`,
/// row-major) and a contiguous vector: `DOUBLE_gemv` -> `cblas_dgemv`
/// (ColMajor, Trans) -> reference `dgemv('T')`. Per output element
/// `temp = 0; temp += A[i][j] * x[j]` over j in order, then
/// `y[i] = 0.0 + 1.0 * temp` (beta 0 zeroes y first, alpha 1).
pub fn blas_gemv(a: &[f64], rows: usize, cols: usize, x: &[f64]) -> Vec<f64> {
    (0..rows)
        .map(|i| {
            let mut temp = 0.0;
            for j in 0..cols {
                temp += a[i * cols + j] * x[j];
            }
            let mut y = 0.0;
            y += 1.0 * temp;
            y
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bits(v: &[f64]) -> Vec<u64> {
        v.iter().map(|x| x.to_bits()).collect()
    }

    #[test]
    fn python_max_min_keep_the_first_argument() {
        assert_eq!(py_max(1.0, 2.0), 2.0);
        assert_eq!(py_max(2.0, 1.0), 2.0);
        assert!(py_max(f64::NAN, 1e-6).is_nan());
        assert_eq!(py_max(1e-6, f64::NAN), 1e-6);
        assert_eq!(py_max(-0.0, 0.0).to_bits(), (-0.0_f64).to_bits());
        assert_eq!(py_min(1.0, 2.0), 1.0);
        assert!(py_min(f64::NAN, 1.0).is_nan());
        assert_eq!(py_min(1.0, f64::NAN), 1.0);
    }

    #[test]
    fn np_maximum_propagates_nan_from_either_side() {
        assert!(np_maximum(f64::NAN, 1.0).is_nan());
        assert!(np_maximum(1.0, f64::NAN).is_nan());
        assert_eq!(np_maximum(3.0, 1e-9), 3.0);
        assert_eq!(np_maximum(0.0, 1e-9), 1e-9);
    }

    #[test]
    fn np_clip_semantics() {
        assert_eq!(np_clip(-0.5, 0.0, 1.0), 0.0);
        assert_eq!(np_clip(1.5, 0.0, 1.0), 1.0);
        assert_eq!(np_clip(0.25, 0.0, 1.0), 0.25);
        assert!(np_clip(f64::NAN, 0.0, 1.0).is_nan());
        assert_eq!(np_clip(-0.0, 0.0, 1.0).to_bits(), 0.0_f64.to_bits());
        assert_eq!(np_clip(f64::INFINITY, 0.0, 1.0), 1.0);
        assert_eq!(np_clip(f64::NEG_INFINITY, 0.0, 1.0), 0.0);
    }

    #[test]
    fn pow3_is_libm_pow_not_two_multiplications() {
        // A value where pow(x, 3) and x*x*x round differently. (Found by a
        // search; numpy's x**3 equals libm pow on linus, see the module doc.)
        let mut differs = 0;
        let mut x = 1.000_000_1_f64;
        for _ in 0..10_000 {
            if pow3(x).to_bits() != (x * x * x).to_bits() {
                differs += 1;
            }
            x *= 1.000_37;
        }
        assert!(differs > 0, "pow3 must not be x*x*x");
        assert_eq!(pow3(2.0), 8.0);
        assert_eq!(pow3(-0.5), -0.125);
    }

    #[test]
    fn linspace_endpoints_and_steps() {
        assert_eq!(linspace(0.0, 1.0, 5), vec![0.0, 0.25, 0.5, 0.75, 1.0]);
        assert_eq!(linspace(0.0, 1.0, 2), vec![0.0, 1.0]);
        assert_eq!(linspace(2.0, 2.0, 3), vec![2.0, 2.0, 2.0]);
        assert_eq!(linspace(0.0, 3.0, 1), vec![0.0]);
        assert!(linspace(0.0, 1.0, 0).is_empty());
        // i * step + start, then the exact stop: 0.1 * 3 != 0.3
        let y = linspace(0.0, 0.3, 4);
        assert_eq!(y[3], 0.3);
        assert_eq!(y[2].to_bits(), (2.0_f64 * (0.3 / 3.0)).to_bits());
        // a zero step: (i / div) * delta, so a subnormal delta still spreads
        let tiny = 5e-324;
        let z = linspace(0.0, tiny, 3);
        assert_eq!(bits(&z), bits(&[0.0, (1.0_f64 / 2.0) * tiny, tiny]));
        // NaN length: every sample but the endpoint is NaN, the endpoint is
        // the NaN stop itself
        let n = linspace(0.0, f64::NAN, 3);
        assert!(n.iter().all(|v| v.is_nan()));
    }

    #[test]
    fn cumsum_is_sequential() {
        let a = [1e16, 1.0, -1e16, 1.0];
        // sequential: 1e16 + 1 rounds back to 1e16, so the -1e16 cancels it
        // exactly and the last 1.0 survives (a pairwise sum would differ)
        assert_eq!(cumsum(&a), vec![1e16, 1e16, 0.0, 1.0]);
        assert!(cumsum(&[]).is_empty());
        assert_eq!(
            cumsum(&[-0.0]).first().map(|v| v.to_bits()),
            Some((-0.0_f64).to_bits())
        );
    }

    #[test]
    fn diff_and_gradient() {
        assert_eq!(diff(&[1.0, 4.0, 9.0]), vec![3.0, 5.0]);
        assert_eq!(gradient(&[1.0, 4.0, 9.0, 16.0]), vec![3.0, 4.0, 6.0, 7.0]);
        assert_eq!(gradient(&[1.0, 3.0]), vec![2.0, 2.0]);
    }

    #[test]
    fn argmin_first_index_and_nan() {
        assert_eq!(argmin(&[3.0, 1.0, 1.0, 2.0]), Some(1));
        assert_eq!(argmin(&[3.0, f64::NAN, 0.0, f64::NAN]), Some(1));
        assert_eq!(argmin(&[f64::NAN, 0.0]), Some(0));
        assert_eq!(argmin(&[0.0, -0.0]), Some(0));
        assert_eq!(argmin(&[]), None);
    }

    #[test]
    fn max_reduce_propagates_nan() {
        assert_eq!(max_reduce(&[1.0, 3.0, 2.0]), Some(3.0));
        assert!(max_reduce(&[1.0, f64::NAN, 2.0]).is_some_and(f64::is_nan));
        assert_eq!(max_reduce(&[]), None);
    }

    #[test]
    fn dot2_starts_from_positive_zero() {
        assert_eq!(dot2([-1.0, -1.0], [0.0, 0.0]).to_bits(), 0.0_f64.to_bits());
        assert_eq!(dot2([1.0, 2.0], [3.0, 4.0]), 11.0);
    }

    #[test]
    fn binary_search_with_guess_matches_a_plain_search_on_sorted_input() {
        let arr: Vec<f64> = (0..50).map(|i| (i / 3) as f64).collect(); // duplicates
        for guess in [-5, 0, 1, 7, 20, 46, 47, 60] {
            for k in -2..=40 {
                let key = k as f64 * 0.5;
                let expect = if key > arr[arr.len() - 1] {
                    arr.len() as isize
                } else if key < arr[0] {
                    -1
                } else {
                    arr.iter()
                        .rposition(|&a| a <= key)
                        .map_or(-1, |p| p as isize)
                };
                assert_eq!(
                    binary_search_with_guess(key, &arr, guess),
                    expect,
                    "key {key} guess {guess}"
                );
            }
        }
        // the linear path for len <= 4
        let short = [0.0, 1.0, 1.0, 2.0];
        assert_eq!(binary_search_with_guess(1.0, &short, 0), 2);
        assert_eq!(binary_search_with_guess(2.0, &short, 0), 3);
        assert_eq!(binary_search_with_guess(2.5, &short, 0), 4);
        assert_eq!(binary_search_with_guess(-0.5, &short, 0), -1);
    }

    #[test]
    fn interp_edges_nan_and_exact_points() {
        let xp = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
        let fp = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0];
        let got = interp(&[-1.0, 0.0, 0.5, 2.0, 4.75, 5.0, 6.0, f64::NAN], &xp, &fp);
        let got = got.unwrap_or_default();
        assert_eq!(&got[..7], &[0.0, 0.0, 5.0, 20.0, 47.5, 50.0, 50.0]);
        assert!(got[7].is_nan());
        // single sample point
        assert_eq!(
            interp(&[-1.0, 0.0, 1.0], &[0.0], &[7.0]),
            Some(vec![7.0, 7.0, 7.0])
        );
        // mismatched / empty
        assert_eq!(interp(&[0.0], &[], &[]), None);
        assert_eq!(interp(&[0.0], &[0.0, 1.0], &[0.0]), None);
        // inf - inf gives a NaN slope: the retry from the right end is NaN as
        // well, and equal end values then give fp[j] itself
        let xp2 = [0.0, 1.0, 2.0, 3.0, 4.0];
        let fp2 = [0.0, f64::INFINITY, f64::INFINITY, 0.0, 0.0];
        let r = interp(&[1.5], &xp2, &fp2).unwrap_or_default();
        assert_eq!(r, vec![f64::INFINITY]);
        // unequal ends: the NaN stays
        let fp3 = [0.0, f64::INFINITY, f64::NEG_INFINITY, 0.0, 0.0];
        let r = interp(&[1.5], &xp2, &fp3).unwrap_or_default();
        assert!(r[0].is_nan());
    }

    #[test]
    fn py_atan2_special_cases() {
        use std::f64::consts::PI;
        assert!(py_atan2(f64::NAN, 1.0).is_nan());
        assert_eq!(py_atan2(0.0, -0.0), PI);
        assert_eq!(py_atan2(-0.0, -0.0), -PI);
        assert_eq!(py_atan2(-0.0, 0.0).to_bits(), (-0.0_f64).to_bits());
        assert_eq!(py_atan2(f64::INFINITY, f64::NEG_INFINITY), 0.75 * PI);
        assert_eq!(py_atan2(1.0, 1.0), 1.0_f64.atan2(1.0));
        assert!(!py_trig_domain_ok(f64::INFINITY));
        assert!(py_trig_domain_ok(f64::NAN));
    }

    #[test]
    fn np_minimum_and_min_reduce_propagate_nan() {
        assert!(np_minimum(f64::NAN, 1.0).is_nan());
        assert!(np_minimum(1.0, f64::NAN).is_nan());
        assert_eq!(np_minimum(3.0, 1e-9), 1e-9);
        assert_eq!(min_reduce(&[3.0, -1.0, 2.0]), Some(-1.0));
        assert!(min_reduce(&[3.0, f64::NAN, -9.0]).is_some_and(f64::is_nan));
        assert_eq!(min_reduce(&[]), None);
    }

    #[test]
    fn blas_order_sums_from_positive_zero() {
        // a signed zero product sums to +0.0, as ddot / dgemv's leading 0.0
        assert_eq!(blas_dot(&[-1.0, -1.0], &[0.0, 0.0]).to_bits(), 0);
        assert_eq!(blas_dot(&[1.0, 2.0, 3.0], &[4.0, 5.0, 6.0]), 32.0);
        // n = 7: two sequential, then one group of five, left to right
        let a = [1e16, 1.0, -1e16, 1.0, 1.0, 1.0, 1.0];
        let b = [1.0; 7];
        let seq2 = (0.0 + 1e16) + 1.0;
        let expect = seq2 + -1e16 + 1.0 + 1.0 + 1.0 + 1.0;
        assert_eq!(blas_dot(&a, &b), expect);
        let y = blas_gemv(&[1.0, 2.0, 3.0, 4.0], 2, 2, &[-0.0, 0.0]);
        assert_eq!(y.iter().map(|v| v.to_bits()).collect::<Vec<_>>(), [0, 0]);
        assert_eq!(
            blas_gemv(&[1.0, 2.0, 3.0, 4.0], 2, 2, &[1.0, 1.0]),
            [3.0, 7.0]
        );
    }

    #[test]
    fn np_pow_is_libm_pow_not_the_product() {
        // pow(x, 2) and x * x differ for some x: np_pow must be pow.
        let mut differ = 0;
        let mut x = 0.123_456_789_f64;
        for _ in 0..20_000 {
            x = (x * 9.876_543_21 + 0.314_159).fract() * 20.0 - 10.0;
            assert_eq!(np_pow(x, 2.0).to_bits(), x.powf(black_box(2.0)).to_bits());
            if np_pow(x, 2.0) != x * x {
                differ += 1;
            }
        }
        assert!(
            differ > 0,
            "glibc pow(x, 2) is expected to differ from x*x somewhere"
        );
    }
}
