//! `wall_fusion`'s per-cycle functions and the projection they run on,
//! ported operation for operation.
//!
//! The Python (src/f1tenth_perception/f1tenth_perception/wall_fusion.py,
//! scene_mask.project_to_image, lidar_front_wall._robust_line and
//! lidar_front_wall_node._publish_observations' glue) is the reference and
//! its docstrings are the documentation. What decides the bits:
//!
//! - every `@` here is netlib BLAS: two- and three-term sums from 0.0 in
//!   index order ([`dot3`]);
//! - `sum(axis=0)` over the (points, hypotheses) residual matrix is
//!   sequential per column, EXCEPT for a single column, which numpy sums
//!   pairwise (it is contiguous then); a 1-D `mean` is pairwise
//!   ([`pairwise_sum`], numpy's PW_BLOCKSIZE 128 / 8 accumulators); a
//!   (K, 2) `mean(axis=0)` is sequential (all measured on linus);
//! - the rectangle verification is float32 the way numpy 1.26 does it: a
//!   Python scalar meeting a float32 array is cast to float32 first (values
//!   AND comparisons), a float32 array meeting a float64 array is widened;
//! - `np.linalg.svd` is [`crate::lapack::svd_vt_mx2`].

use crate::geometry::{Pose2D, WallLine};
use crate::lapack::svd_vt_mx2;
use crate::pyc::{np_argmin, np_linspace, np_minimum, py_hypot, py_max, py_min, py_sum};
use crate::tracker::{SOURCE_BOTH, SOURCE_LIDAR, SOURCE_STEREO};

pub const STATUS_EMPTY: u8 = 0;
pub const STATUS_AGREE: u8 = 1;
pub const STATUS_LIDAR_ONLY: u8 = 2;
pub const STATUS_STEREO_ONLY: u8 = 3;
pub const STATUS_DISAGREE: u8 = 4;

/// `lidar_front_wall._MIN_HYPOTHESIS_BASELINE_M`.
pub const MIN_HYPOTHESIS_BASELINE_M: f64 = 0.02;

/// Python's `math.floor` / `math.ceil` of an infinity (OverflowError).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FusionError {
    Overflow,
}

/// `((0 + a0 b0) + a1 b1) + a2 b2`: netlib dgemm / dgemv over three terms.
#[inline]
fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((0.0 + a[0] * b[0]) + a[1] * b[1]) + a[2] * b[2]
}

/// numpy 1.26 `pairwise_sum` (umath/loops_utils.h.src), float64.
pub fn pairwise_sum(a: &[f64]) -> f64 {
    let n = a.len();
    if n < 8 {
        let mut res = 0.0_f64;
        for &v in a {
            res += v;
        }
        res
    } else if n <= 128 {
        let mut r = [0.0_f64; 8];
        r.copy_from_slice(&a[..8]);
        let mut i = 8;
        while i < n - (n % 8) {
            for k in 0..8 {
                r[k] += a[i + k];
            }
            i += 8;
        }
        let mut res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
        while i < n {
            res += a[i];
            i += 1;
        }
        res
    } else {
        let mut n2 = n / 2;
        n2 -= n2 % 8;
        pairwise_sum(&a[..n2]) + pairwise_sum(&a[n2..])
    }
}

/// The camera: intrinsics and `R`, `t` (base -> optical: `p_opt = R^T (p - t)`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// Row-major 3 x 3.
    pub rot: [[f64; 3]; 3],
    pub trans: [f64; 3],
}

impl Camera {
    /// `(pts - t) @ R`: the optical-frame point.
    #[inline]
    fn optical(&self, p: [f64; 3]) -> [f64; 3] {
        let d = [
            p[0] - self.trans[0],
            p[1] - self.trans[1],
            p[2] - self.trans[2],
        ];
        let r = &self.rot;
        [
            dot3(d, [r[0][0], r[1][0], r[2][0]]),
            dot3(d, [r[0][1], r[1][1], r[2][1]]),
            dot3(d, [r[0][2], r[1][2], r[2][2]]),
        ]
    }

    /// scene_mask.project_to_image for one point: (u, v), NaN behind.
    #[inline]
    pub fn project(&self, p: [f64; 3]) -> (f64, f64) {
        let o = self.optical(p);
        // `behind = ~(z > 1e-6)`: a NaN depth is behind too.
        let in_front = o[2] > 1e-6;
        if !in_front {
            return (f64::NAN, f64::NAN);
        }
        (
            self.fx * o[0] / o[2] + self.cx,
            self.fy * o[1] / o[2] + self.cy,
        )
    }

    /// `((p - t) @ R)[2]`: the optical depth (the node's scan_depth).
    #[inline]
    pub fn depth(&self, p: [f64; 3]) -> f64 {
        self.optical(p)[2]
    }
}

/// `_compensate`: base_link points at the scan moment -> the image moment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Compensation {
    pub pose_scan: Pose2D,
    pub pose_image: Pose2D,
}

impl Compensation {
    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let q = self.pose_scan.transform_point([p[0], p[1]]);
        let r = self.pose_image.inverse().transform_point(q);
        [r[0], r[1], p[2]]
    }

    /// Many points: the inverse pose computed once, as numpy does.
    pub fn apply_many(&self, pts: &[[f64; 3]]) -> Vec<[f64; 3]> {
        let inv = self.pose_image.inverse();
        pts.iter()
            .map(|p| {
                let q = self.pose_scan.transform_point([p[0], p[1]]);
                let r = inv.transform_point(q);
                [r[0], r[1], p[2]]
            })
            .collect()
    }
}

/// `np.rint` (round half to even).
#[inline]
fn rint(x: f64) -> f64 {
    x.round_ties_even()
}

/// `column_nearest(cols, depths, width)`.
pub fn column_nearest(cols: &[f64], depths: &[f64], width: usize) -> Vec<f64> {
    let mut out = vec![f64::INFINITY; width];
    let wl = width as f64 - 0.5;
    for (&c, &d) in cols.iter().zip(depths) {
        if c.is_finite() && d.is_finite() && c >= -0.5 && c < wl {
            let k = rint(c) as i64;
            if k >= 0 && (k as usize) < width {
                let slot = &mut out[k as usize];
                *slot = np_minimum(*slot, d);
            }
        }
    }
    out.iter()
        .map(|&v| if v.is_finite() { v } else { f64::NAN })
        .collect()
}

/// `tolerance(r, base, quad)` for one value.
#[inline]
fn tolerance(r: f64, base: f64, quad: f64) -> f64 {
    let v = if r.is_finite() { r } else { 0.0 };
    base + quad * (v * v)
}

/// `cross_check_profile`'s arrays.
#[derive(Debug, Clone, PartialEq)]
pub struct Check {
    pub status: Vec<u8>,
    pub lidar_m: Vec<f64>,
    pub stereo_m: Vec<f64>,
    pub tol_m: Vec<f64>,
}

/// `cross_check_profile(scan_cols, scan_depths, stereo_m, width, tol_base, tol_quad)`.
pub fn cross_check_profile(
    scan_cols: &[f64],
    scan_depths: &[f64],
    stereo_in: &[f64],
    width: usize,
    tol_base: f64,
    tol_quad: f64,
) -> Check {
    let lidar_m = column_nearest(scan_cols, scan_depths, width);
    let mut stereo_m = vec![f64::NAN; width];
    let n = width.min(stereo_in.len());
    stereo_m[..n].copy_from_slice(&stereo_in[..n]);
    let mut status = vec![STATUS_EMPTY; width];
    let mut tol_m = vec![0.0; width];
    for k in 0..width {
        let has_l = lidar_m[k].is_finite();
        let has_s = stereo_m[k].is_finite();
        let r = if has_l {
            lidar_m[k]
        } else if has_s {
            stereo_m[k]
        } else {
            0.0
        };
        let tol = tolerance(r, tol_base, tol_quad);
        tol_m[k] = tol;
        status[k] = match (has_l, has_s) {
            (true, false) => STATUS_LIDAR_ONLY,
            (false, true) => STATUS_STEREO_ONLY,
            (true, true) => {
                if (stereo_m[k] - lidar_m[k]).abs() <= tol {
                    STATUS_AGREE
                } else {
                    STATUS_DISAGREE
                }
            }
            (false, false) => STATUS_EMPTY,
        };
    }
    Check {
        status,
        lidar_m,
        stereo_m,
        tol_m,
    }
}

/// The confidence model's numbers (`confidence`'s keyword arguments).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConfParams {
    pub inlier_ref: f64,
    pub rms_ref: f64,
    pub range_ref_m: f64,
    pub min_inlier_ref: f64,
    pub max_rms_scale: f64,
}

/// `confidence(source, inliers, rms, n_agree, n_span, ...)`; `range_m` and
/// `rect_support` None as in the Python's defaults.
#[allow(clippy::too_many_arguments)]
pub fn confidence(
    source: i32,
    inliers: f64,
    rms: f64,
    n_agree: i64,
    n_span: i64,
    stereo_weight: f64,
    rect_support: Option<f64>,
    range_m: Option<f64>,
    c: &ConfParams,
) -> f64 {
    let (base, corro);
    if source == SOURCE_STEREO {
        base = 0.35;
        corro = 1.0;
    } else {
        let mut inlier_ref = c.inlier_ref;
        let mut rms_ref = c.rms_ref;
        if let Some(r) = range_m {
            if r.is_finite() && c.range_ref_m > 0.0 && inliers >= c.min_inlier_ref {
                let k = py_max(1.0, r / c.range_ref_m);
                inlier_ref = py_max(c.min_inlier_ref, inlier_ref / k);
                rms_ref *= py_min(c.max_rms_scale, k);
            }
        }
        let evidence = py_min(1.0, inliers / py_max(1.0, inlier_ref));
        let tightness = 1.0 / (1.0 + py_max(0.0, rms) / py_max(1e-6, rms_ref));
        base = 0.5 * evidence + 0.5 * tightness;
        let w = py_min(1.0, py_max(0.0, stereo_weight));
        let mut terms = Vec::with_capacity(2);
        if n_span > 0 {
            terms.push(0.5 + w * ((n_agree as f64) / (n_span as f64) - 0.5));
        }
        if let Some(s) = rect_support {
            terms.push(0.5 + w * (s - 0.5));
        }
        corro = match py_sum(&terms) {
            Some(s) => s / (terms.len() as f64),
            None => 0.5,
        };
    }
    let mut value = base * (0.6 + 0.4 * corro);
    if source == SOURCE_BOTH {
        value = py_min(1.0, value * 1.15);
    }
    py_min(1.0, py_max(0.0, value))
}

/// `np.nanmin` / `np.nanmax` of a slice with at least one non-NaN value.
fn nanmin(v: &[f64]) -> f64 {
    let mut best = f64::NAN;
    for &x in v {
        if !x.is_nan() && (best.is_nan() || x < best) {
            best = x;
        }
    }
    best
}

fn nanmax(v: &[f64]) -> f64 {
    let mut best = f64::NAN;
    for &x in v {
        if !x.is_nan() && (best.is_nan() || x > best) {
            best = x;
        }
    }
    best
}

/// Python's `seq[a:b]` for non-negative ints.
fn sub<T>(v: &[T], a: usize, b: usize) -> &[T] {
    let b = b.min(v.len());
    let a = a.min(b);
    &v[a..b]
}

/// `_height_from_columns(z_min, z_max, col_begin, col_end, default_height)`.
pub fn height_from_columns(
    z_min: &[f64],
    z_max: &[f64],
    c0: usize,
    c1: usize,
    default_height: f64,
) -> (f64, f64, bool) {
    if c1 > c0 {
        let lo = sub(z_min, c0, c1);
        let hi = sub(z_max, c0, c1);
        if lo.iter().any(|v| v.is_finite()) && hi.iter().any(|v| v.is_finite()) {
            return (nanmin(lo), nanmax(hi), true);
        }
    }
    (0.0, default_height, false)
}

/// `line_columns(line, project, width)`: `project` is base -> (u, v) for a
/// base point at z = 0.
pub fn line_columns(
    line: &WallLine,
    project: &dyn Fn([f64; 3]) -> (f64, f64),
    width: usize,
) -> Result<(usize, usize), FusionError> {
    let (a, b) = line.endpoints();
    let us = [project([a[0], a[1], 0.0]).0, project([b[0], b[1], 0.0]).0];
    if !us.iter().any(|u| u.is_finite()) {
        return Ok((0, 0));
    }
    let mut lo = nanmin(&us);
    let mut hi = nanmax(&us);
    if lo.is_finite() != hi.is_finite() {
        if hi.is_finite() {
            lo = 0.0;
        } else {
            hi = width as f64;
        }
    }
    if !lo.is_finite() || !hi.is_finite() {
        return Err(FusionError::Overflow);
    }
    let lo_i = (lo.floor() as i64).max(0);
    let hi_i = (width as i64).min(hi.ceil() as i64 + 1);
    if hi_i > lo_i {
        Ok((lo_i as usize, hi_i as usize))
    } else {
        Ok((0, 0))
    }
}

/// One observation: `wall_fusion.Observation`'s fields; `line_index` is the
/// input line it came from (None: a stereo-only line, `line` its fit).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FusedObservation {
    pub line_index: Option<usize>,
    pub line: WallLine,
    pub z_min: f64,
    pub z_max: f64,
    pub height_measured: bool,
    pub source: i32,
    pub confidence: f64,
    pub col_begin: i64,
    pub col_end: i64,
    pub n_agree: i64,
    pub n_lidar_only: i64,
    pub n_stereo_only: i64,
    pub n_disagree: i64,
    pub stereo_weight: f64,
}

/// `observations_from_lines(lines, check, project, width, z_min, z_max, ...)`:
/// the observations and the number rejected.
#[allow(clippy::too_many_arguments)]
pub fn observations_from_lines(
    lines: &[WallLine],
    status: &[u8],
    project: &dyn Fn([f64; 3]) -> (f64, f64),
    width: usize,
    z_min: &[f64],
    z_max: &[f64],
    default_height: f64,
    disagree_reject_frac: f64,
    stereo_weight: f64,
    conf: &ConfParams,
) -> Result<(Vec<FusedObservation>, i64), FusionError> {
    let mut out = Vec::new();
    let mut rejected = 0_i64;
    for (k, line) in lines.iter().enumerate() {
        let (c0, c1) = line_columns(line, project, width)?;
        let span = if c1 > c0 {
            sub(status, c0, c1)
        } else {
            &status[..0]
        };
        let count = |s: u8| span.iter().filter(|&&x| x == s).count() as i64;
        let n_agree = count(STATUS_AGREE);
        let n_dis = count(STATUS_DISAGREE);
        let n_lidar = count(STATUS_LIDAR_ONLY);
        let n_stereo = count(STATUS_STEREO_ONLY);
        let n_opinion = n_agree + n_dis;
        let w = py_min(1.0, py_max(0.0, stereo_weight));
        if n_opinion > 0 && (n_dis as f64) * w >= disagree_reject_frac * (n_opinion as f64) {
            rejected += 1;
            continue;
        }
        let source = if n_agree > 0 {
            SOURCE_BOTH
        } else {
            SOURCE_LIDAR
        };
        let (hc0, hc1) = if n_agree > 0 { (c0, c1) } else { (0, 0) };
        let (lo, hi, measured) = height_from_columns(z_min, z_max, hc0, hc1, default_height);
        let c = confidence(
            source,
            line.inliers as f64,
            line.rms,
            n_agree,
            n_opinion,
            w,
            None,
            Some(line.d.abs()),
            conf,
        );
        out.push(FusedObservation {
            line_index: Some(k),
            line: *line,
            z_min: lo,
            z_max: hi,
            height_measured: measured,
            source,
            confidence: c,
            col_begin: c0 as i64,
            col_end: c1 as i64,
            n_agree,
            n_lidar_only: n_lidar,
            n_stereo_only: n_stereo,
            n_disagree: n_dis,
            stereo_weight: w,
        });
    }
    Ok((out, rejected))
}

/// `contiguous_spans(mask, min_width)`: half-open runs of true.
pub fn contiguous_spans(mask: &[bool], min_width: i64) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut start: Option<usize> = None;
    for (i, &m) in mask.iter().chain(std::iter::once(&false)).enumerate() {
        match (m, start) {
            (true, None) => start = Some(i),
            (false, Some(a)) => {
                if (i - a) as i64 >= min_width {
                    out.push((a, i));
                }
                start = None;
            }
            _ => {}
        }
    }
    out
}

/// `_hypothesis_pairs(n, max_pairs)`: triu_indices(n, 1), every k-th.
///
/// The every-k-th selection is computed row by row (pair `(i, j)` has the
/// row-major index `start_i + j - i - 1`), not by building all n(n-1)/2 pairs
/// first; [`tests`] pins it against the plain enumeration.
pub fn hypothesis_pairs(n: usize, max_pairs: usize) -> Vec<(usize, usize)> {
    let total = n * n.saturating_sub(1) / 2;
    let step = if total > max_pairs && max_pairs > 0 {
        total.div_ceil(max_pairs)
    } else {
        1
    };
    let mut pairs = Vec::with_capacity(total.div_ceil(step));
    let mut start = 0_usize;
    for i in 0..n {
        let row = n - 1 - i;
        // The first selected index in [start, start + row).
        let mut idx = start.div_ceil(step) * step;
        while idx < start + row {
            pairs.push((i, i + 1 + (idx - start)));
            idx += step;
        }
        start += row;
    }
    pairs
}

/// `_robust_line(points, inlier_distance_m, max_pairs)`:
/// (nx, ny, offset, inlier count, rms).
pub fn robust_line(
    points: &[[f64; 2]],
    inlier_m: f64,
    max_pairs: usize,
) -> Option<(f64, f64, f64, i64, f64)> {
    let pairs = hypothesis_pairs(points.len(), max_pairs);
    // The kept hypotheses as columns: normal (nx, ny) and offset n . p_i.
    let (mut hx, mut hy, mut off) = (Vec::new(), Vec::new(), Vec::new());
    for &(i, j) in &pairs {
        let dx = points[j][0] - points[i][0];
        let dy = points[j][1] - points[i][1];
        let length = dx.hypot(dy);
        if length > MIN_HYPOTHESIS_BASELINE_M {
            let (nx, ny) = (-dy / length, dx / length);
            hx.push(nx);
            hy.push(ny);
            off.push((0.0 + nx * points[i][0]) + ny * points[i][1]);
        }
    }
    if hx.is_empty() {
        return None;
    }
    let m = hx.len();
    // |points @ normals.T - offsets|, element (k, h): netlib dgemm's two terms.
    #[inline]
    fn resid(p: [f64; 2], nx: f64, ny: f64, off: f64) -> f64 {
        ((0.0 + p[0] * nx) + p[1] * ny - off).abs()
    }
    // np.minimum(residuals, inlier_m).sum(axis=0): one column is contiguous
    // and numpy sums it pairwise; several are summed row by row, in order.
    let sums: Vec<f64> = if m == 1 {
        let col: Vec<f64> = points
            .iter()
            .map(|p| np_minimum(resid(*p, hx[0], hy[0], off[0]), inlier_m))
            .collect();
        vec![pairwise_sum(&col)]
    } else {
        let mut s: Vec<f64> = (0..m)
            .map(|h| np_minimum(resid(points[0], hx[h], hy[h], off[h]), inlier_m))
            .collect();
        for p in &points[1..] {
            for (((slot, &nx), &ny), &o) in s.iter_mut().zip(&hx).zip(&hy).zip(&off) {
                *slot += np_minimum(resid(*p, nx, ny, o), inlier_m);
            }
        }
        s
    };
    let best = np_argmin(&sums)?;
    let mut inliers: Vec<bool> = points
        .iter()
        .map(|p| resid(*p, hx[best], hy[best], off[best]) <= inlier_m)
        .collect();
    let mut normal = [0.0_f64; 2];
    let mut offset = 0.0_f64;
    let mut distances = vec![0.0_f64; points.len()];
    for _ in 0..2 {
        let ins: Vec<[f64; 2]> = points
            .iter()
            .zip(&inliers)
            .filter(|(_, &b)| b)
            .map(|(p, _)| *p)
            .collect();
        if ins.len() < 3 {
            return None;
        }
        let kf = ins.len() as f64;
        let mut c = ins[0];
        for p in &ins[1..] {
            c = [c[0] + p[0], c[1] + p[1]];
        }
        let centroid = [c[0] / kf, c[1] / kf];
        let centred: Vec<[f64; 2]> = ins
            .iter()
            .map(|p| [p[0] - centroid[0], p[1] - centroid[1]])
            .collect();
        let vt = svd_vt_mx2(&centred)?;
        normal = vt[1];
        offset = (0.0 + normal[0] * centroid[0]) + normal[1] * centroid[1];
        if offset < 0.0 {
            normal = [-normal[0], -normal[1]];
            offset = -offset;
        }
        for (k, p) in points.iter().enumerate() {
            distances[k] = ((0.0 + p[0] * normal[0]) + p[1] * normal[1] - offset).abs();
        }
        inliers = distances.iter().map(|&d| d <= inlier_m).collect();
    }
    let sq: Vec<f64> = distances
        .iter()
        .zip(&inliers)
        .filter(|(_, &b)| b)
        .map(|(&d, _)| d * d)
        .collect();
    let count = sq.len() as i64;
    if count < 3 {
        return None;
    }
    let rms = (pairwise_sum(&sq) / (count as f64)).sqrt();
    Some((normal[0], normal[1], offset, count, rms))
}

/// stereo_only_observations' numbers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StereoParams {
    pub min_span_columns: i64,
    pub inlier_m: f64,
    pub min_points: usize,
    pub min_span_m: f64,
    pub default_height: f64,
    pub max_pairs: usize,
}

/// `stereo_only_observations(check, column_points, width, ...)` with the
/// node's `column_points`: the cloud points whose projected column u is
/// finite and in [c0, c1), in cloud order. `z` None: no profile heights.
pub fn stereo_only_observations(
    status: &[u8],
    cloud: &[[f64; 3]],
    cloud_u: &[f64],
    sp: &StereoParams,
    z: Option<(&[f64], &[f64])>,
) -> Vec<FusedObservation> {
    let mut out = Vec::new();
    let mask: Vec<bool> = status.iter().map(|&s| s == STATUS_STEREO_ONLY).collect();
    for (c0, c1) in contiguous_spans(&mask, sp.min_span_columns) {
        let (a, b) = (c0 as f64, c1 as f64);
        let mut pts: Vec<[f64; 2]> = cloud
            .iter()
            .zip(cloud_u)
            .filter(|(_, &u)| u.is_finite() && u >= a && u < b)
            .map(|(p, _)| [p[0], p[1]])
            .collect();
        if pts.is_empty() || pts.len() < sp.min_points {
            continue;
        }
        if pts.len() > 400 {
            let idx = np_linspace(0.0, (pts.len() - 1) as f64, 400);
            pts = idx.iter().map(|&f| pts[f as usize]).collect();
        }
        let Some((nx, ny, d, count, rms)) = robust_line(&pts, sp.inlier_m, sp.max_pairs) else {
            continue;
        };
        let on: Vec<[f64; 2]> = pts
            .iter()
            .filter(|p| ((0.0 + p[0] * nx) + p[1] * ny - d).abs() <= sp.inlier_m)
            .copied()
            .collect();
        if on.len() < 3 {
            continue;
        }
        let t: Vec<f64> = on.iter().map(|p| -ny * p[0] + nx * p[1]).collect();
        let tmin = crate::pyc::np_min_reduce(&t).unwrap_or(f64::NAN);
        let tmax = crate::pyc::np_max_reduce(&t).unwrap_or(f64::NAN);
        if tmax - tmin < sp.min_span_m {
            continue;
        }
        let line = WallLine {
            nx,
            ny,
            d,
            t0: tmin,
            t1: tmax,
            inliers: count,
            rms,
        };
        let (lo, hi, measured) = match z {
            Some((zlo, zhi)) => height_from_columns(zlo, zhi, c0, c1, sp.default_height),
            None => (0.0, sp.default_height, false),
        };
        let span = (c1 - c0) as i64;
        let conf = confidence(
            SOURCE_STEREO,
            count as f64,
            rms,
            span,
            span,
            1.0,
            None,
            None,
            &ConfParams {
                inlier_ref: 40.0,
                rms_ref: 0.03,
                range_ref_m: 5.0,
                min_inlier_ref: 8.0,
                max_rms_scale: 6.0,
            },
        );
        out.push(FusedObservation {
            line_index: None,
            line,
            z_min: lo,
            z_max: hi,
            height_measured: measured,
            source: SOURCE_STEREO,
            confidence: conf,
            col_begin: c0 as i64,
            col_end: c1 as i64,
            n_agree: 0,
            n_lidar_only: 0,
            n_stereo_only: span,
            n_disagree: 0,
            stereo_weight: 1.0,
        });
    }
    out
}

/// Everything `_publish_observations` computes on a cycle with a fresh
/// profile and a camera transform.
pub struct FusionInput<'a> {
    pub lines: &'a [WallLine],
    /// The scan points, base_link at the scan stamp (`pts` of _scene_fit).
    pub scan_pts: &'a [[f64; 2]],
    pub laser_z: f64,
    pub camera: Camera,
    pub width: usize,
    pub distance_m: &'a [f64],
    pub z_min: &'a [f64],
    pub z_max: &'a [f64],
    /// None: the scan is not carried to the image moment.
    pub compensation: Option<Compensation>,
    /// The WALL cloud (read_points_numpy, float64), None without one.
    pub cloud: Option<&'a [[f64; 3]]>,
    pub tol_base: f64,
    pub tol_quad: f64,
    pub disagree_reject_frac: f64,
    pub stereo_weight: f64,
    pub conf: ConfParams,
    pub stereo: StereoParams,
}

/// The cycle's fusion: the check, the observations (from the lines, then the
/// stereo-only ones), the number rejected.
pub fn fusion_cycle(
    inp: &FusionInput<'_>,
) -> Result<(Check, Vec<FusedObservation>, i64), FusionError> {
    let cam = inp.camera;
    // The scan, carried to the image moment, projected.
    let xyz: Vec<[f64; 3]> = inp
        .scan_pts
        .iter()
        .map(|p| [p[0], p[1], inp.laser_z])
        .collect();
    let xyz = match &inp.compensation {
        Some(c) if !xyz.is_empty() => c.apply_many(&xyz),
        _ => xyz,
    };
    let scan_u: Vec<f64> = xyz.iter().map(|p| cam.project(*p).0).collect();
    let scan_depth: Vec<f64> = xyz.iter().map(|p| cam.depth(*p)).collect();
    let check = cross_check_profile(
        &scan_u,
        &scan_depth,
        inp.distance_m,
        inp.width,
        inp.tol_base,
        inp.tol_quad,
    );
    let comp = inp.compensation;
    let project_from_scan = move |p: [f64; 3]| -> (f64, f64) {
        let q = match &comp {
            Some(c) => c.apply(p),
            None => p,
        };
        cam.project(q)
    };
    let (mut obs, rejected) = observations_from_lines(
        inp.lines,
        &check.status,
        &project_from_scan,
        inp.width,
        inp.z_min,
        inp.z_max,
        inp.stereo.default_height,
        inp.disagree_reject_frac,
        inp.stereo_weight,
        &inp.conf,
    )?;
    if let Some(cloud) = inp.cloud {
        if !cloud.is_empty() {
            let us: Vec<f64> = cloud.iter().map(|p| cam.project(*p).0).collect();
            obs.extend(stereo_only_observations(
                &check.status,
                cloud,
                &us,
                &inp.stereo,
                Some((inp.z_min, inp.z_max)),
            ));
        }
    }
    Ok((check, obs, rejected))
}

// ============================================================================
// The rectangle (float32, numpy 1.26 casting)
// ============================================================================

/// `RectParams`' sampling and decision numbers verify_rectangles reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RectSampleParams {
    pub t_step_min_m: f64,
    pub t_steps_max: i64,
    pub tol_base_m: f64,
    pub tol_quad_per_m2: f64,
    pub min_depth_m: f64,
    pub max_range_m: f64,
}

/// One segment's verdict (`verify_rectangles` entry).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RectOut {
    pub coverage: f64,
    pub support: f64,
    pub beyond_share: f64,
    pub n_samples: i64,
    pub n_in_image: i64,
    pub at: f64,
    pub beyond: f64,
    pub against: f64,
}

/// The class mask and depth (row-major H x W), with the classes that matter.
pub struct RectImage<'a> {
    pub labels: &'a [u8],
    pub depth: &'a [u16],
    pub height: usize,
    pub width: usize,
    pub depth_scale: f32,
    pub wall_class: u8,
    pub object_class: u8,
}

/// `verify_rectangles(segments, labels, depth, ...)` with an integer depth
/// image, no object mask and no detail (the node's call). `grid` is
/// `rect_grid(p)` (float32 z rows and weights, computed by the Python:
/// numpy's float32 exp is its own).
#[allow(clippy::too_many_arguments)]
pub fn verify_rectangles(
    segments: &[[[f64; 2]; 2]],
    img: &RectImage<'_>,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    rot: &[[f64; 3]; 3],
    trans: &[f64; 3],
    laser_z: f64,
    p: &RectSampleParams,
    grid_z: &[f32],
    grid_w: &[f32],
) -> Vec<RectOut> {
    let n_seg = segments.len();
    let mut out = vec![RectOut::default(); n_seg];
    if n_seg == 0 {
        return out;
    }
    // rect_samples
    let nz = grid_z.len();
    let lz = laser_z as f32;
    let zb: Vec<f32> = grid_z.iter().map(|&z| z + lz).collect();
    let mut xs: Vec<f32> = Vec::new();
    let mut ys: Vec<f32> = Vec::new();
    let mut ks: Vec<usize> = Vec::new();
    for (k, s) in segments.iter().enumerate() {
        let (ax, ay, bx, by) = (s[0][0], s[0][1], s[1][0], s[1][1]);
        let length = py_hypot(bx - ax, by - ay);
        let step = py_max(p.t_step_min_m, length / (p.t_steps_max.max(1) as f64));
        let nt = ((length / step).floor() as i64 + 1).max(2) as usize;
        let f = np_linspace(0.0, 1.0, nt);
        let (dxs, dys) = ((bx - ax) as f32, (by - ay) as f32);
        let (axs, ays) = (ax as f32, ay as f32);
        for fi in f {
            let f32v = fi as f32;
            xs.push(axs + f32v * dxs);
            ys.push(ays + f32v * dys);
            ks.push(k);
        }
    }
    if xs.is_empty() {
        return out;
    }
    let r32: [[f32; 3]; 3] = [
        [rot[0][0] as f32, rot[0][1] as f32, rot[0][2] as f32],
        [rot[1][0] as f32, rot[1][1] as f32, rot[1][2] as f32],
        [rot[2][0] as f32, rot[2][1] as f32, rot[2][2] as f32],
    ];
    let t32 = [trans[0] as f32, trans[1] as f32, trans[2] as f32];
    let (fx32, fy32, cx32, cy32) = (fx as f32, fy as f32, cx as f32, cy as f32);
    let min_depth32 = p.min_depth_m as f32;
    let max_range32 = p.max_range_m as f32;
    let (w32, h32) = (img.width as f32, img.height as f32);
    let mut w_tot = vec![0.0_f64; n_seg];
    let mut w_inf = vec![0.0_f64; n_seg];
    let mut w_sup = vec![0.0_f64; n_seg];
    let mut w_bey = vec![0.0_f64; n_seg];
    let mut w_at = vec![0.0_f64; n_seg];
    let mut w_ag = vec![0.0_f64; n_seg];
    let mut n_all = vec![0_i64; n_seg];
    let mut n_in = vec![0_i64; n_seg];
    let mut any_inside = false;
    // Pass 1, in sample order: the totals (bincount over every sample).
    for &k in &ks {
        for &w in grid_w.iter().take(nz) {
            w_tot[k] += f64::from(w);
            n_all[k] += 1;
        }
    }
    // Pass 2: the in-image samples, in sample order.
    for (t, &k) in ks.iter().enumerate() {
        let (px, py) = (xs[t], ys[t]);
        for zi in 0..nz {
            let pz = zb[zi];
            let dx = px - t32[0];
            let dy = py - t32[1];
            let dz = pz - t32[2];
            let ox = dx * r32[0][0] + dy * r32[1][0] + dz * r32[2][0];
            let oy = dx * r32[0][1] + dy * r32[1][1] + dz * r32[2][1];
            let d_exp = dx * r32[0][2] + dy * r32[1][2] + dz * r32[2][2];
            let u = (fx32 * ox / d_exp + cx32).round_ties_even();
            let v = (fy32 * oy / d_exp + cy32).round_ties_even();
            let inside = d_exp > min_depth32 && u >= 0.0 && u < w32 && v >= 0.0 && v < h32;
            if !inside {
                continue;
            }
            any_inside = true;
            let (ui, vi) = (u as i32 as usize, v as i32 as usize);
            let cls = img.labels[vi * img.width + ui];
            let raw = img.depth[vi * img.width + ui];
            let dep = if raw == 0 {
                f32::NAN
            } else {
                f32::from(raw) * img.depth_scale
            };
            let exp = d_exp;
            let wi = grid_w[zi];
            let tol = tolerance(f64::from(exp), p.tol_base_m, p.tol_quad_per_m2);
            let is_wall = cls == img.wall_class;
            let obj = cls == img.object_class;
            let diff = dep - exp;
            let diff64 = f64::from(diff);
            let against = obj || (is_wall && diff64 < -tol);
            let mut at = is_wall && !against && diff64.abs() <= tol;
            let mut beyond = is_wall && !against && diff64 > tol;
            let in_range = exp <= max_range32;
            at &= in_range;
            beyond &= in_range;
            let against = against && in_range;
            let informative = at || beyond || against;
            let supporting = at || beyond;
            let wf = |b: bool| f64::from(wi * if b { 1.0_f32 } else { 0.0_f32 });
            w_inf[k] += wf(informative);
            w_sup[k] += wf(supporting);
            w_bey[k] += wf(beyond);
            w_at[k] += wf(at);
            w_ag[k] += wf(against);
            n_in[k] += 1;
        }
    }
    if !any_inside {
        for k in 0..n_seg {
            out[k].n_samples = if w_tot[k] > 0.0 { n_all[k] } else { 0 };
        }
        return out;
    }
    for k in 0..n_seg {
        out[k] = RectOut {
            coverage: if w_tot[k] > 0.0 {
                w_inf[k] / w_tot[k]
            } else {
                0.0
            },
            support: if w_inf[k] > 0.0 {
                w_sup[k] / w_inf[k]
            } else {
                0.0
            },
            beyond_share: if w_sup[k] > 0.0 {
                w_bey[k] / w_sup[k]
            } else {
                0.0
            },
            n_samples: n_all[k],
            n_in_image: n_in[k],
            at: w_at[k],
            beyond: w_bey[k],
            against: w_ag[k],
        };
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pairwise_sum_matches_sequential_for_short_inputs() {
        let v: Vec<f64> = (0..7).map(|k| f64::from(k) * 0.1).collect();
        let mut s = 0.0;
        for &x in &v {
            s += x;
        }
        assert_eq!(pairwise_sum(&v), s);
        let long: Vec<f64> = (0..1000).map(|k| f64::from(k % 7) * 0.013).collect();
        let exact: f64 = long.iter().sum();
        assert!((pairwise_sum(&long) - exact).abs() < 1e-10);
    }

    #[test]
    fn strided_pairs_equal_the_plain_enumeration() {
        for n in 0..60 {
            for max_pairs in [0, 1, 2, 5, 17, 300, 800, 5000] {
                let mut all = Vec::new();
                for i in 0..n {
                    for j in (i + 1)..n {
                        all.push((i, j));
                    }
                }
                let want: Vec<(usize, usize)> = if all.len() > max_pairs && max_pairs > 0 {
                    let step = all.len().div_ceil(max_pairs);
                    all.into_iter().step_by(step).collect()
                } else {
                    all
                };
                assert_eq!(
                    hypothesis_pairs(n, max_pairs),
                    want,
                    "n {n} max {max_pairs}"
                );
            }
        }
        assert_eq!(hypothesis_pairs(400, 300).len(), 300);
    }

    #[test]
    fn spans_and_pairs() {
        let m = [false, true, true, false, true, true, true];
        assert_eq!(contiguous_spans(&m, 2), vec![(1, 3), (4, 7)]);
        assert_eq!(contiguous_spans(&m, 3), vec![(4, 7)]);
        assert_eq!(hypothesis_pairs(4, 100).len(), 6);
        let p = hypothesis_pairs(30, 100);
        assert_eq!(p[0], (0, 1));
        assert!(p.len() <= 100);
    }

    #[test]
    fn a_line_of_points_fits_its_normal() {
        let pts: Vec<[f64; 2]> = (0..50).map(|k| [2.0, -1.0 + f64::from(k) * 0.04]).collect();
        let (nx, ny, d, count, rms) = robust_line(&pts, 0.03, 300).unwrap();
        assert!((nx.abs() - 1.0).abs() < 1e-12 && ny.abs() < 1e-12);
        assert!((d - 2.0).abs() < 1e-12);
        assert_eq!(count, 50);
        assert!(rms < 1e-12);
    }

    #[test]
    fn column_nearest_takes_the_nearest_per_column() {
        let out = column_nearest(
            &[0.4, 0.6, 1.4, f64::NAN, -0.6],
            &[3.0, 2.0, 5.0, 1.0, 1.0],
            3,
        );
        assert_eq!(out[0], 3.0);
        assert_eq!(out[1], 2.0);
        assert!(out[2].is_nan());
    }

    #[test]
    fn confidence_of_a_stereo_wall_is_capped() {
        let c = ConfParams {
            inlier_ref: 40.0,
            rms_ref: 0.03,
            range_ref_m: 5.0,
            min_inlier_ref: 8.0,
            max_rms_scale: 6.0,
        };
        let v = confidence(SOURCE_STEREO, 50.0, 0.01, 10, 10, 1.0, None, None, &c);
        assert!((v - 0.35).abs() < 1e-12);
    }
}
