//! `np.linalg.svd(X, full_matrices=False)` for an M x 2 matrix, as numpy 1.26
//! computes it on linus: numpy's `_umath_linalg` links the SYSTEM LAPACK
//! (`liblapack.so.3` -> netlib reference LAPACK 3.12.0, gfortran, no FMA;
//! numpy's show_config "lapack: internal" is not what runs) and BLAS 3.12.
//!
//! numpy calls `dgesdd` with JOBZ = 'S'. For M >= MNTHR = int(2 * 11 / 6) = 3
//! that is path 3: `dgeqrf` (unblocked `dgeqr2`: `dlarfg`, whose norm is
//! BLAS 3.12's `dnrm2` -- Blue's three-accumulator algorithm, i.e.
//! `sqrt(sum x^2)` in the mid range -- and `dlarf`), `dgebrd` of the 2 x 2 R
//! (every reflector of a 2 x 2 upper triangle is the identity: tau = 0),
//! `dbdsdc` -> `dlasdq` -> `dbdsqr`, which for a 2 x 2 upper bidiagonal block
//! is one `dlasv2` and a `drot` of VT (or a split when the off-diagonal is
//! below its threshold), then the sign fix and the sorts (for n = 2 the three
//! sorts together leave VT as dbdsqr made it). `dormbr` with tau = 0 leaves VT
//! alone. Only VT is computed: `multi_wall_tracker`'s robust line reads
//! `vt[-1]` alone.
//!
//! The gfortran details that decide the bits are kept: `SIGN(a, b)` follows
//! the sign BIT of b (copysign); `**2` is `x * x`; `dlamch('E')` is
//! DBL_EPSILON / 2 and `dlamch('S')` DBL_MIN; `EPS ** (-1/8)` is libm pow.
//! Verified bit for bit against numpy (tools/rust_parity/wall_tracker/
//! numerics.py, `svd_vt`).

use std::hint::black_box;

/// `dlamch('E')`: lapack_lite's relative machine precision (rounding).
const EPS: f64 = f64::EPSILON * 0.5;
/// `dlamch('S')`: the safe minimum (1 / DBL_MAX is smaller than DBL_MIN).
const SFMIN: f64 = f64::MIN_POSITIVE;

/// gfortran's `SIGN(a, b)`: |a| with the sign bit of b.
#[inline]
fn d_sign(a: f64, b: f64) -> f64 {
    a.abs().copysign(b)
}

/// f2c `max` / `min` macros.
#[inline]
fn fmax(a: f64, b: f64) -> f64 {
    if a >= b {
        a
    } else {
        b
    }
}

#[inline]
fn fmin(a: f64, b: f64) -> f64 {
    if a <= b {
        a
    } else {
        b
    }
}

/// BLAS 3.12 `dnrm2` (dnrm2.f90, Blue's scaling): the squares summed in three
/// accumulators (scaled down above TBIG, up below TSML, as they are between).
fn dnrm2(x: &[f64]) -> f64 {
    const TSML: f64 = f64::from_bits(512_u64 << 52); // 2^-511
    const TBIG: f64 = f64::from_bits(1509_u64 << 52); // 2^486
    const SSML: f64 = f64::from_bits(1560_u64 << 52); // 2^537
    const SBIG: f64 = f64::from_bits(485_u64 << 52); // 2^-538
    if x.is_empty() {
        return 0.0;
    }
    let mut notbig = true;
    let (mut asml, mut amed, mut abig) = (0.0_f64, 0.0_f64, 0.0_f64);
    for &v in x {
        let ax = v.abs();
        if ax > TBIG {
            let t = ax * SBIG;
            abig += t * t;
            notbig = false;
        } else if ax < TSML {
            if notbig {
                let t = ax * SSML;
                asml += t * t;
            }
        } else {
            amed += ax * ax;
        }
    }
    let (scl, sumsq);
    if abig > 0.0 {
        if amed > 0.0 || amed > f64::MAX || amed.is_nan() {
            abig += (amed * SBIG) * SBIG;
        }
        scl = 1.0 / SBIG;
        sumsq = abig;
    } else if asml > 0.0 {
        if amed > 0.0 || amed > f64::MAX || amed.is_nan() {
            let amed_r = amed.sqrt();
            let asml_r = asml.sqrt() / SSML;
            let (ymin, ymax) = if asml_r > amed_r {
                (amed_r, asml_r)
            } else {
                (asml_r, amed_r)
            };
            scl = 1.0;
            let q = ymin / ymax;
            sumsq = ymax * ymax * (1.0 + q * q);
        } else {
            scl = 1.0 / SSML;
            sumsq = asml;
        }
    } else {
        scl = 1.0;
        sumsq = amed;
    }
    scl * sumsq.sqrt()
}

/// `dlapy2(x, y)` (3.12): sqrt(x^2 + y^2) without destructive overflow.
fn dlapy2(x: f64, y: f64) -> f64 {
    let xabs = x.abs();
    let yabs = y.abs();
    let w = fmax(xabs, yabs);
    let z = fmin(xabs, yabs);
    if z == 0.0 || w > f64::MAX {
        w
    } else {
        let d = z / w;
        w * (1.0 + d * d).sqrt()
    }
}

/// `dlarfg(n, alpha, x, 1, tau)` with n = 1 + x.len(); returns tau.
fn dlarfg(alpha: &mut f64, x: &mut [f64]) -> f64 {
    if x.is_empty() {
        return 0.0;
    }
    let mut xnorm = dnrm2(x);
    if xnorm == 0.0 {
        return 0.0;
    }
    let mut beta = -d_sign(dlapy2(*alpha, xnorm), *alpha);
    let safmin = SFMIN / EPS;
    let mut knt = 0;
    if beta.abs() < safmin {
        let rsafmn = 1.0 / safmin;
        loop {
            knt += 1;
            for v in x.iter_mut() {
                *v *= rsafmn;
            }
            beta *= rsafmn;
            *alpha *= rsafmn;
            if beta.abs() >= safmin {
                break;
            }
        }
        xnorm = dnrm2(x);
        beta = -d_sign(dlapy2(*alpha, xnorm), *alpha);
    }
    let tau = (beta - *alpha) / beta;
    let scal = 1.0 / (*alpha - beta);
    for v in x.iter_mut() {
        *v *= scal;
    }
    for _ in 0..knt {
        beta *= safmin;
    }
    *alpha = beta;
    tau
}

/// `dlarf('Left', m, 1, v, 1, tau, c, ldc, work)`: H = I - tau v v' applied
/// to one column `c` (v[0] == 1.0 by the caller).
fn dlarf_left_1(v: &[f64], tau: f64, c: &mut [f64]) {
    if tau == 0.0 {
        return;
    }
    let mut lastv = v.len();
    while lastv > 0 && v[lastv - 1] == 0.0 {
        lastv -= 1;
    }
    // iladlc(lastv, 1, C): 1 when some C(1:lastv) is non-zero, else 0.
    let lastc = usize::from(lastv > 0 && c[..lastv].iter().any(|&x| x != 0.0));
    if lastv == 0 || lastc == 0 {
        return;
    }
    // dgemv('T'): w = 0 + 1 * sum c(i) v(i), in order.
    let mut temp = 0.0_f64;
    for i in 0..lastv {
        temp += c[i] * v[i];
    }
    let w = 0.0 + 1.0 * temp;
    // dger: skips a zero y.
    if w != 0.0 {
        let t = -tau * w;
        for i in 0..lastv {
            c[i] += v[i] * t;
        }
    }
}

/// `dlasv2(f, g, h)`: (ssmin, ssmax, snr, csr, snl, csl) of [[f, g], [0, h]].
#[allow(clippy::many_single_char_names)]
pub fn dlasv2(f: f64, g: f64, h: f64) -> (f64, f64, f64, f64, f64, f64) {
    let mut ft = f;
    let mut fa = ft.abs();
    let mut ht = h;
    let mut ha = h.abs();
    let mut pmax = 1;
    let swap = ha > fa;
    if swap {
        pmax = 3;
        std::mem::swap(&mut ft, &mut ht);
        std::mem::swap(&mut fa, &mut ha);
    }
    let gt = g;
    let ga = gt.abs();
    let (ssmin, ssmax, clt, crt, slt, srt);
    if ga == 0.0 {
        ssmin = ha;
        ssmax = fa;
        clt = 1.0;
        crt = 1.0;
        slt = 0.0;
        srt = 0.0;
    } else {
        let mut gasmal = true;
        let mut out = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        if ga > fa {
            pmax = 2;
            if (fa / ga) < EPS {
                gasmal = false;
                let smax = ga;
                let smin = if ha > 1.0 {
                    fa / (ga / ha)
                } else {
                    (fa / ga) * ha
                };
                out = (smin, smax, 1.0, ft / gt, ht / gt, 1.0);
            }
        }
        if gasmal {
            let d = fa - ha;
            let mut l = if d == fa { 1.0 } else { d / fa };
            let m = gt / ft;
            let mut t = 2.0 - l;
            let mm = m * m;
            let tt = t * t;
            let s = (tt + mm).sqrt();
            let r = if l == 0.0 {
                m.abs()
            } else {
                (l * l + mm).sqrt()
            };
            let a = 0.5 * (s + r);
            let smin = ha / a;
            let smax = fa * a;
            if mm == 0.0 {
                if l == 0.0 {
                    t = d_sign(2.0, ft) * d_sign(1.0, gt);
                } else {
                    t = gt / d_sign(d, ft) + m / t;
                }
            } else {
                t = (m / (s + t) + m / (r + l)) * (1.0 + a);
            }
            l = (t * t + 4.0).sqrt();
            let crt_ = 2.0 / l;
            let srt_ = t / l;
            let clt_ = (crt_ + srt_ * m) / a;
            let slt_ = (ht / ft) * srt_ / a;
            out = (smin, smax, clt_, crt_, slt_, srt_);
        }
        ssmin = out.0;
        ssmax = out.1;
        clt = out.2;
        crt = out.3;
        slt = out.4;
        srt = out.5;
    }
    let (csl, snl, csr, snr) = if swap {
        (srt, crt, slt, clt)
    } else {
        (clt, slt, crt, srt)
    };
    let tsign = match pmax {
        1 => d_sign(1.0, csr) * d_sign(1.0, csl) * d_sign(1.0, f),
        2 => d_sign(1.0, snr) * d_sign(1.0, csl) * d_sign(1.0, g),
        _ => d_sign(1.0, snr) * d_sign(1.0, snl) * d_sign(1.0, h),
    };
    let ssmax = d_sign(ssmax, tsign);
    let ssmin = d_sign(ssmin, tsign * d_sign(1.0, f) * d_sign(1.0, h));
    (ssmin, ssmax, snr, csr, snl, csl)
}

/// `drot` of two 2-vectors (rows of VT): x' = c x + s y, y' = c y - s x.
fn drot(x: &mut [f64; 2], y: &mut [f64; 2], c: f64, s: f64) {
    for k in 0..2 {
        let dtemp = c * x[k] + s * y[k];
        y[k] = c * y[k] - s * x[k];
        x[k] = dtemp;
    }
}

/// dbdsqr for the 2 x 2 upper bidiagonal [[d1, e1], [0, d2]] with VT = I:
/// the VT rows (singular values sorted decreasing).
fn dbdsqr2(d1: f64, e1: f64, d2: f64) -> [[f64; 2]; 2] {
    let n = 2.0_f64;
    let tolmul = fmax(10.0, fmin(100.0, EPS.powf(black_box(-0.125))));
    let tol = tolmul * EPS;
    // Relative accuracy (tol >= 0).
    let mut sminoa = d1.abs();
    if sminoa != 0.0 {
        let mu = d2.abs() * (sminoa / (sminoa + e1.abs()));
        sminoa = fmin(sminoa, mu);
    }
    sminoa /= n.sqrt();
    let maxitr = 6.0;
    let thresh = fmax(tol * sminoa, maxitr * n * n * SFMIN);
    let mut d = [d1, d2];
    let mut vt = [[1.0, 0.0], [0.0, 1.0]];
    if e1.abs() > thresh {
        let (sigmn, sigmx, sinr, cosr, _sinl, _cosl) = dlasv2(d1, e1, d2);
        d = [sigmx, sigmn];
        let (mut r0, mut r1) = (vt[0], vt[1]);
        drot(&mut r0, &mut r1, cosr, sinr);
        vt = [r0, r1];
    }
    // Make the singular values positive.
    for i in 0..2 {
        if d[i] < 0.0 {
            d[i] = -d[i];
            // dscal(-1): the same as negation for every non-NaN value.
            vt[i] = [-vt[i][0], -vt[i][1]];
        }
    }
    // Sort decreasing: one pass for n = 2.
    let mut isub = 0;
    let smin = d[0];
    if d[1] <= smin {
        isub = 1;
    }
    if isub != 1 {
        d.swap(0, 1);
        vt.swap(0, 1);
    }
    vt
}

/// `np.linalg.svd(x, full_matrices=False)[2]` for an M x 2 matrix with M >= 3
/// (`x[i] = [x_i0, x_i1]`). None for M < 3 (a different dgesdd path, which no
/// caller here takes).
pub fn svd_vt_mx2(x: &[[f64; 2]]) -> Option<[[f64; 2]; 2]> {
    let m = x.len();
    if m < 3 {
        return None;
    }
    // Column-major copy of A.
    let mut c0: Vec<f64> = x.iter().map(|r| r[0]).collect();
    let mut c1: Vec<f64> = x.iter().map(|r| r[1]).collect();
    // dgeqr2, i = 1.
    let (head, tail) = c0.split_at_mut(1);
    let tau1 = dlarfg(&mut head[0], tail);
    let beta1 = c0[0];
    c0[0] = 1.0;
    dlarf_left_1(&c0, tau1, &mut c1);
    c0[0] = beta1;
    // i = 2.
    let (_, rest) = c1.split_at_mut(1);
    let (head2, tail2) = rest.split_at_mut(1);
    let _tau2 = dlarfg(&mut head2[0], tail2);
    let r11 = c0[0];
    let r12 = c1[0];
    let r22 = c1[1];
    // dgebrd of the 2 x 2 upper triangle: identity reflectors, B = R.
    Some(dbdsqr2(r11, r12, r22))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_smallest_singular_vector_of_a_line_is_its_normal() {
        // Points along y = 2x (centred): the normal is +-(2, -1)/sqrt(5).
        let pts: Vec<[f64; 2]> = (-5..=5)
            .map(|k| [f64::from(k) * 0.1, f64::from(k) * 0.2])
            .collect();
        let vt = svd_vt_mx2(&pts).unwrap();
        let n = vt[1];
        let dot = (n[0] * 2.0 - n[1]) / 5.0_f64.sqrt();
        assert!((dot.abs() - 1.0).abs() < 1e-12, "{n:?}");
        assert!((n[0] * n[0] + n[1] * n[1] - 1.0).abs() < 1e-14);
    }

    #[test]
    fn dnrm2_and_dlapy2() {
        assert_eq!(dnrm2(&[3.0, 4.0]), 5.0);
        assert_eq!(dnrm2(&[0.0, 0.0]), 0.0);
        assert_eq!(dlapy2(3.0, 4.0), 5.0);
        assert_eq!(dlapy2(0.0, -2.0), 2.0);
    }

    #[test]
    fn too_few_rows() {
        assert!(svd_vt_mx2(&[[1.0, 0.0], [0.0, 1.0]]).is_none());
    }
}
