//! 2x2 linear algebra exactly as numpy 1.26 computes it on linus.
//!
//! numpy's `_umath_linalg` links the system LAPACK (netlib reference 3.12.0)
//! and BLAS. `solve(S, v)` and `inv(S)` are both `dgesv` (`inv` solves
//! against the identity): `dgetrf` -> `dgetrf2`, which for a 2 x 2 does the
//! arithmetic of the classic unblocked `dgetf2` -- partial pivoting by
//! `idamax` (the first maximum wins), the column scaled by the RECIPROCAL of
//! the pivot (a division below DBL_MIN), the trailing update -- then `dgetrs`
//! (`dlaswp`, unit-lower then upper `dtrsm`, both skipping a zero right-hand
//! side entry). Every branch is reproduced; checked bit for bit against numpy
//! on 200k matrices (tools/rust_parity/wall_tracker/numerics.py).
//!
//! The matrix products are netlib `dgemm` / `dgemv` / `ddot`: two-term sums
//! from 0.0 ([`crate::pyc::dot2`]).

use crate::pyc::dot2;

/// A row-major 2x2 matrix, `m[row][col]`.
pub type Mat2 = [[f64; 2]; 2];

/// `np.linalg.LinAlgError("Singular matrix")`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Singular;

/// `A @ B` (netlib dgemm).
#[inline]
pub fn mat_mul(a: &Mat2, b: &Mat2) -> Mat2 {
    let mut c = [[0.0; 2]; 2];
    for (i, row) in c.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = dot2(a[i][0], a[i][1], b[0][j], b[1][j]);
        }
    }
    c
}

/// `A @ x` (netlib dgemv).
#[inline]
pub fn mat_vec(a: &Mat2, x: [f64; 2]) -> [f64; 2] {
    [
        dot2(a[0][0], a[0][1], x[0], x[1]),
        dot2(a[1][0], a[1][1], x[0], x[1]),
    ]
}

/// `A.T`.
#[inline]
pub fn transpose(a: &Mat2) -> Mat2 {
    [[a[0][0], a[1][0]], [a[0][1], a[1][1]]]
}

/// `A + B` elementwise.
#[inline]
pub fn add(a: &Mat2, b: &Mat2) -> Mat2 {
    [
        [a[0][0] + b[0][0], a[0][1] + b[0][1]],
        [a[1][0] + b[1][0], a[1][1] + b[1][1]],
    ]
}

/// `np.diag([a, b])`.
#[inline]
pub fn diag(a: f64, b: f64) -> Mat2 {
    [[a, 0.0], [0.0, b]]
}

/// The LU factors dgetf2 leaves for a 2x2: row swap, U, and the multiplier.
struct Lu {
    swap: bool,
    u00: f64,
    u01: f64,
    u11: f64,
    l10: f64,
}

/// `DBL_MIN`, `dlamch('S')` (1/DBL_MAX is smaller).
const SFMIN: f64 = f64::MIN_POSITIVE;

fn dgetf2(a: &Mat2) -> Result<Lu, Singular> {
    let (mut r0, mut r1) = (a[0], a[1]);
    // j = 1: idamax over the first column, the first maximum winning (a NaN
    // never compares greater, so it never pivots).
    let swap = r1[0].abs() > r0[0].abs();
    if swap {
        std::mem::swap(&mut r0, &mut r1);
    }
    // A zero pivot sets info = 1 and dgesv then refuses the matrix, whatever
    // the second step finds.
    if r0[0] == 0.0 {
        return Err(Singular);
    }
    let l10 = if r0[0].abs() >= SFMIN {
        r1[0] * (1.0 / r0[0])
    } else {
        r1[0] / r0[0]
    };
    // dger(m-1, n-1, -1, A(2,1), 1, A(1,2), lda, A(2,2), lda): skips y == 0.
    let mut a11 = r1[1];
    if r0[1] != 0.0 {
        let temp = -r0[1];
        a11 += l10 * temp;
    }
    // j = 2: a single element; zero is a singular matrix (info = 2).
    if a11 == 0.0 {
        return Err(Singular);
    }
    Ok(Lu {
        swap,
        u00: r0[0],
        u01: r0[1],
        u11: a11,
        l10,
    })
}

fn dgetrs(lu: &Lu, b: [f64; 2]) -> [f64; 2] {
    let (mut b0, mut b1) = (b[0], b[1]);
    if lu.swap {
        std::mem::swap(&mut b0, &mut b1);
    }
    // dtrsm Left, Lower, NoTrans, Unit.
    if b0 != 0.0 {
        b1 -= b0 * lu.l10;
    }
    // dtrsm Left, Upper, NoTrans, NonUnit: k = 2 then k = 1.
    if b1 != 0.0 {
        b1 /= lu.u11;
        b0 -= b1 * lu.u01;
    }
    if b0 != 0.0 {
        b0 /= lu.u00;
    }
    [b0, b1]
}

/// `np.linalg.solve(A, b)` for a 2x2 `A` and a 2-vector `b`.
pub fn solve(a: &Mat2, b: [f64; 2]) -> Result<[f64; 2], Singular> {
    let lu = dgetf2(a)?;
    Ok(dgetrs(&lu, b))
}

/// `np.linalg.inv(A)` for a 2x2 `A`: dgesv against the identity.
pub fn inv(a: &Mat2) -> Result<Mat2, Singular> {
    let lu = dgetf2(a)?;
    let c0 = dgetrs(&lu, [1.0, 0.0]);
    let c1 = dgetrs(&lu, [0.0, 1.0]);
    Ok([[c0[0], c1[0]], [c0[1], c1[1]]])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solve_and_inverse_of_a_well_conditioned_matrix() {
        let a = [[4.0, 1.0], [2.0, 3.0]];
        let x = solve(&a, [1.0, 2.0]).unwrap();
        assert!((x[0] - 0.1).abs() < 1e-15 && (x[1] - 0.6).abs() < 1e-15);
        let ai = inv(&a).unwrap();
        let id = mat_mul(&a, &ai);
        assert!((id[0][0] - 1.0).abs() < 1e-15 && id[0][1].abs() < 1e-15);
        assert!(id[1][0].abs() < 1e-15 && (id[1][1] - 1.0).abs() < 1e-15);
    }

    #[test]
    fn pivoting_swaps_the_rows() {
        let a = [[1e-3, 1.0], [1.0, 1.0]];
        let x = solve(&a, [1.0, 2.0]).unwrap();
        assert!((1e-3 * x[0] + x[1] - 1.0).abs() < 1e-12);
        assert!((x[0] + x[1] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn singular_matrices_are_refused() {
        assert_eq!(solve(&[[1.0, 2.0], [2.0, 4.0]], [1.0, 1.0]), Err(Singular));
        assert_eq!(inv(&[[0.0, 0.0], [0.0, 1.0]]), Err(Singular));
    }
}
