//! corridor_perception.geometry (Pose2D, Line) and scene_labels.WallLine,
//! operation for operation.

use crate::linalg::{diag, mat_mul, transpose, Mat2};
use crate::pyc::{dot2, wrap_pi};
use std::f64::consts::PI;

/// `Pose2D(x, y, theta)`: frame A from frame B, `p_A = R(theta) p_B + t`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Pose2D {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }

    /// `self.R`: `[[c, -s], [s, c]]` with `math.cos` / `math.sin`.
    #[inline]
    pub fn r(&self) -> Mat2 {
        let (c, s) = (self.theta.cos(), self.theta.sin());
        [[c, -s], [s, c]]
    }

    /// `compose(other)`: `t = self.t + self.R @ other.t`.
    pub fn compose(&self, other: &Pose2D) -> Pose2D {
        let r = self.r();
        let rt = [
            dot2(r[0][0], r[0][1], other.x, other.y),
            dot2(r[1][0], r[1][1], other.x, other.y),
        ];
        Pose2D::new(
            self.x + rt[0],
            self.y + rt[1],
            wrap_pi(self.theta + other.theta),
        )
    }

    /// `inverse()`: `t = -(self.R.T @ self.t)`.
    pub fn inverse(&self) -> Pose2D {
        let rt = transpose(&self.r());
        let v = [
            dot2(rt[0][0], rt[0][1], self.x, self.y),
            dot2(rt[1][0], rt[1][1], self.x, self.y),
        ];
        Pose2D::new(-v[0], -v[1], wrap_pi(-self.theta))
    }

    /// `transform_points([p])`: `pts @ self.R.T + self.t`, one point.
    #[inline]
    pub fn transform_point(&self, p: [f64; 2]) -> [f64; 2] {
        let rt = transpose(&self.r());
        [
            dot2(p[0], p[1], rt[0][0], rt[1][0]) + self.x,
            dot2(p[0], p[1], rt[0][1], rt[1][1]) + self.y,
        ]
    }

    /// `transform_points(pts)` for many points (R computed once, as numpy does).
    pub fn transform_points(&self, pts: &[[f64; 2]]) -> Vec<[f64; 2]> {
        let rt = transpose(&self.r());
        pts.iter()
            .map(|p| {
                [
                    dot2(p[0], p[1], rt[0][0], rt[1][0]) + self.x,
                    dot2(p[0], p[1], rt[0][1], rt[1][1]) + self.y,
                ]
            })
            .collect()
    }

    #[inline]
    pub fn t(&self) -> [f64; 2] {
        [self.x, self.y]
    }
}

/// `normalise_line(rho, alpha)`: rho >= 0 by flipping alpha by pi.
#[inline]
pub fn normalise_line(rho: f64, alpha: f64) -> (f64, f64) {
    if rho < 0.0 {
        (-rho, wrap_pi(alpha + PI))
    } else {
        (rho, wrap_pi(alpha))
    }
}

/// corridor_perception's `Line`: Hesse form, covariance, support.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line {
    pub rho: f64,
    pub alpha: f64,
    pub cov: Mat2,
    pub p_start: [f64; 2],
    pub p_end: [f64; 2],
    pub n_points: i64,
}

impl Line {
    /// `Line.transform(pose)`: the line (frame B) in frame A.
    pub fn transform(&self, pose: &Pose2D) -> Line {
        let alpha_a = wrap_pi(self.alpha + pose.theta);
        let n_a = [alpha_a.cos(), alpha_a.sin()];
        let (tx, ty) = (pose.x, pose.y);
        let rho_a = self.rho + tx * n_a[0] + ty * n_a[1];
        let j: Mat2 = [[1.0, -tx * n_a[1] + ty * n_a[0]], [0.0, 1.0]];
        let mut cov_a = mat_mul(&mat_mul(&j, &self.cov), &transpose(&j));
        let ends = pose.transform_points(&[self.p_start, self.p_end]);
        let (rho_n, alpha_n) = normalise_line(rho_a, alpha_a);
        if rho_n != rho_a {
            let f = diag(-1.0, 1.0);
            cov_a = mat_mul(&mat_mul(&f, &cov_a), &transpose(&f));
        }
        Line {
            rho: rho_n,
            alpha: alpha_n,
            cov: cov_a,
            p_start: ends[0],
            p_end: ends[1],
            n_points: self.n_points,
        }
    }
}

/// multi_wall_tracker's `_along(theta, pts)`: `-sin(theta) x + cos(theta) y`.
#[inline]
pub fn along(theta: f64, p: [f64; 2]) -> f64 {
    let (s, c) = (theta.sin(), theta.cos());
    -s * p[0] + c * p[1]
}

/// `_along` over several points, sin/cos computed once.
pub fn along_many(theta: f64, pts: &[[f64; 2]]) -> Vec<f64> {
    let (s, c) = (theta.sin(), theta.cos());
    pts.iter().map(|p| -s * p[0] + c * p[1]).collect()
}

/// scene_labels.WallLine: `n . (x, y) = d` in base_link with its extent.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WallLine {
    pub nx: f64,
    pub ny: f64,
    pub d: f64,
    pub t0: f64,
    pub t1: f64,
    pub inliers: i64,
    pub rms: f64,
}

impl WallLine {
    #[inline]
    pub fn span(&self) -> f64 {
        self.t1 - self.t0
    }

    /// `endpoints()`: `((px + tx t0, py + ty t0), (px + tx t1, py + ty t1))`.
    #[inline]
    pub fn endpoints(&self) -> ([f64; 2], [f64; 2]) {
        let (px, py) = (self.nx * self.d, self.ny * self.d);
        let (tx, ty) = (-self.ny, self.nx);
        (
            [px + tx * self.t0, py + ty * self.t0],
            [px + tx * self.t1, py + ty * self.t1],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inverse_composes_to_identity() {
        let p = Pose2D::new(1.0, -2.0, 0.7);
        let q = p.compose(&p.inverse());
        assert!(q.x.abs() < 1e-15 && q.y.abs() < 1e-15 && q.theta.abs() < 1e-15);
    }

    #[test]
    fn a_line_moves_with_the_frame() {
        let l = Line {
            rho: 2.0,
            alpha: 0.0,
            cov: [[1e-4, 0.0], [0.0, 1e-4]],
            p_start: [2.0, -1.0],
            p_end: [2.0, 1.0],
            n_points: 10,
        };
        let m = l.transform(&Pose2D::new(1.0, 0.0, 0.0));
        assert_eq!(m.rho, 3.0);
        assert_eq!(m.p_start, [3.0, -1.0]);
        let flipped = l.transform(&Pose2D::new(-5.0, 0.0, 0.0));
        assert_eq!(flipped.rho, 3.0);
        assert!((flipped.alpha.abs() - PI).abs() < 1e-15);
    }

    #[test]
    fn wall_line_endpoints() {
        let w = WallLine {
            nx: 1.0,
            ny: 0.0,
            d: 2.0,
            t0: -1.0,
            t1: 1.0,
            inliers: 3,
            rms: 0.0,
        };
        assert_eq!(w.endpoints(), ([2.0, -1.0], [2.0, 1.0]));
        assert_eq!(w.span(), 2.0);
    }
}
