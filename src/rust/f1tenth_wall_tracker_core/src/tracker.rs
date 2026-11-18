//! `multi_wall_tracker.MultiWallTracker`, ported function for function.
//!
//! The Python (src/f1tenth_perception/f1tenth_perception/multi_wall_tracker.py)
//! is the reference and its docstrings are the documentation: every function
//! here carries the Python name it ports, and does the same operations in the
//! same order (`crate::pyc` for the Python/numpy semantics, `crate::linalg`
//! for numpy's LAPACK, `crate::lsa` for scipy's assignment). The tracker state
//! is the Python's field for field (`Track`), so the ctypes binding can mirror
//! it into real `mwt.Track` objects for every Python reader.

use std::collections::{HashMap, HashSet, VecDeque};
use std::f64::consts::PI;

use crate::geometry::{along_many, normalise_line, Line, Pose2D, WallLine};
use crate::linalg::{self, add, diag, mat_mul, mat_vec, Mat2, Singular};
use crate::lsa::{linear_sum_assignment, LsaError};
use crate::pyc::{
    dot2, np_arange, np_argmin, np_linspace, np_max_reduce, np_maximum, np_min_reduce,
    np_searchsorted_left, py_atan2, py_hypot, py_max, py_max_seq, py_min, py_pow2, py_sort_by_key,
    py_sort_pairs, py_sum, wrap_pi,
};

pub const STATE_TENTATIVE: i32 = 0;
pub const STATE_CONFIRMED: i32 = 1;
pub const STATE_COASTING: i32 = 2;

pub const SOURCE_LIDAR: i32 = 0;
pub const SOURCE_STEREO: i32 = 1;
pub const SOURCE_BOTH: i32 = 2;

pub const VERIFY_UNVERIFIED: i32 = 0;
pub const VERIFY_VERIFIED: i32 = 1;
pub const VERIFY_REJECTED: i32 = 2;

/// `_GAP_STEP_M`.
const GAP_STEP_M: f64 = 0.05;
/// `_MAX_REJECTIONS`.
const MAX_REJECTIONS: usize = 16;

/// Everything a step can refuse with: the Python raises at the same places.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackerError {
    /// `np.linalg.LinAlgError("Singular matrix")`.
    Singular,
    /// scipy's ValueError ("invalid numeric entries" / "infeasible").
    Lsa(LsaError),
}

impl From<Singular> for TrackerError {
    fn from(_: Singular) -> Self {
        TrackerError::Singular
    }
}

impl From<LsaError> for TrackerError {
    fn from(e: LsaError) -> Self {
        TrackerError::Lsa(e)
    }
}

/// A role: the Python strings as codes. `Unset` is `''`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(i32)]
pub enum Role {
    #[default]
    Unset = 0,
    Front = 1,
    Back = 2,
    Left = 3,
    Right = 4,
    Unknown = 5,
    /// `LEGACY_ROLE_OTHER`: never assigned, accepted from a reader's state.
    Other = 6,
}

impl Role {
    pub fn from_code(c: i32) -> Option<Role> {
        Some(match c {
            0 => Role::Unset,
            1 => Role::Front,
            2 => Role::Back,
            3 => Role::Left,
            4 => Role::Right,
            5 => Role::Unknown,
            6 => Role::Other,
            _ => return None,
        })
    }

    /// Python truthiness of the role string.
    #[inline]
    pub fn is_set(self) -> bool {
        self != Role::Unset
    }

    /// `is_unknown_role`.
    #[inline]
    pub fn is_unknown(self) -> bool {
        matches!(self, Role::Unset | Role::Unknown | Role::Other)
    }
}

/// `STRIP_*` (and `''`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(i32)]
pub enum StripVerdict {
    #[default]
    Unset = 0,
    Supported = 1,
    Contradicted = 2,
    Blocked = 3,
    NoReturns = 4,
}

impl StripVerdict {
    pub fn from_code(c: i32) -> Option<StripVerdict> {
        Some(match c {
            0 => StripVerdict::Unset,
            1 => StripVerdict::Supported,
            2 => StripVerdict::Contradicted,
            3 => StripVerdict::Blocked,
            4 => StripVerdict::NoReturns,
            _ => return None,
        })
    }
}

/// `front_reason`'s codes (and the two front_status-only reasons).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum FrontCode {
    Ok = 0,
    Unconfirmed = 1,
    Contradicted = 2,
    RoleOther = 3,
    OffStrip = 4,
    CoastExpired = 5,
    CoastContradicted = 6,
    LowConfidence = 7,
    Unverified = 8,
    StereoOblique = 9,
    /// front_status 'front'.
    Front = 10,
    /// front_status 'no_candidate'.
    NoCandidate = 11,
}

/// The refusal codes of `_reject`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum RejectCode {
    GapAssoc = 1,
    GapMerge = 2,
    StereoNear = 3,
    StereoShort = 4,
    StereoRms = 5,
    StereoFlat = 6,
    CameraRadial = 7,
}

/// One `self.rejections` entry: `(code, track id or None, value)`; the value
/// is RAW here and rounded (`round(value, 3)`) by the Python binding.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rejection {
    pub code: RejectCode,
    pub track_id: Option<i64>,
    pub value: f64,
}

/// A Python `collections.deque` with its `maxlen`.
#[derive(Debug, Clone, PartialEq)]
pub struct PyDeque<T> {
    pub items: VecDeque<T>,
    pub maxlen: Option<usize>,
}

impl<T: Clone> PyDeque<T> {
    pub fn new(maxlen: Option<usize>) -> Self {
        Self {
            items: VecDeque::new(),
            maxlen,
        }
    }

    /// `deque(iterable, maxlen)`: the last `maxlen` items.
    pub fn from_vec(v: Vec<T>, maxlen: Option<usize>) -> Self {
        let mut d = Self::new(maxlen);
        for x in v {
            d.append(x);
        }
        d
    }

    /// `append`: a full deque drops from the left first.
    pub fn append(&mut self, v: T) {
        match self.maxlen {
            Some(0) => {}
            Some(m) => {
                while self.items.len() >= m {
                    self.items.pop_front();
                }
                self.items.push_back(v);
            }
            None => self.items.push_back(v),
        }
    }

    pub fn len(&self) -> usize {
        self.items.len()
    }

    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
}

/// One `obs_ext` entry: `(stamp, (2, 2) odom endpoints)`, plus a serial the
/// binding uses to reuse the Python tuple it already made for it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExtEntry {
    pub stamp: f64,
    pub pts: [[f64; 2]; 2],
    pub serial: u64,
}

/// `TrackerParams`, field for field (`strip_half_width_m` is the property).
#[derive(Debug, Clone, PartialEq)]
pub struct Params {
    pub process_noise_rho: f64,
    pub process_noise_theta: f64,
    pub k_yaw: f64,
    pub meas_sigma_rho_min: f64,
    pub meas_sigma_theta_min: f64,
    pub gate_chi2: f64,
    pub collinear_gap_m: f64,
    pub extent_decay_s: f64,
    pub confirm_min_consecutive_hits: i64,
    pub confirm_min_length_m: f64,
    pub confirm_max_dist_jitter_m: f64,
    pub confirm_max_angle_jitter_rad: f64,
    pub confirm_require_lidar: bool,
    pub delete_after_s: f64,
    pub max_sigma_rho_m: f64,
    pub max_published: i64,
    pub label_half_sector_rad: f64,
    pub role_hysteresis_rad: f64,
    pub ray_margin_m: f64,
    pub front_max_coast_s: f64,
    pub front_max_coast_m: f64,
    pub lidar_sigma_base_m: f64,
    pub lidar_sigma_per_m: f64,
    pub stereo_sigma_base_m: f64,
    pub stereo_sigma_quad_per_m2: f64,
    pub min_span_for_angle_m: f64,
    pub min_span_birth_m: f64,
    pub min_span_update_m: f64,
    pub front_x_m: f64,
    pub side_edge_half_width_m: f64,
    pub front_strip_margin_m: f64,
    pub stereo_front_max_normal_rad: f64,
    pub front_min_confidence: f64,
    pub front_lidar_recent_cycles: i64,
    pub front_contradict_margin_m: f64,
    pub front_contradict_cycles: i64,
    pub stereo_min_rho_m: f64,
    pub camera_seg_min_ray_angle_rad: f64,
    pub camera_seg_min_bearing_span_rad: f64,
    pub stereo_max_rms_m: f64,
    pub stereo_min_vertical_extent_m: f64,
    pub stereo_min_length_m: f64,
    pub back_x_m: f64,
    pub side_x_min_m: f64,
    pub side_x_max_m: f64,
    pub side_max_normal_rad: f64,
    pub camera_side_x_max_m: f64,
    pub camera_side_max_normal_rad: f64,
    pub camera_side_offset_min_m: f64,
    pub camera_side_offset_max_m: f64,
    pub role_hysteresis_m: f64,
    pub occlusion_points: i64,
    pub occlusion_min_blocked: i64,
    pub occlusion_margin_m: f64,
    pub negative_shrink_beams: i64,
    pub negative_tol_m: f64,
    pub negative_min_beams: i64,
    pub negative_pass_frac: f64,
    pub negative_miss_to_kill: i64,
    pub coast_max_odom_m: f64,
}

impl Default for Params {
    /// TrackerParams' own defaults.
    fn default() -> Self {
        Self {
            process_noise_rho: 1e-4,
            process_noise_theta: 1e-4,
            k_yaw: 0.1,
            meas_sigma_rho_min: 0.02,
            meas_sigma_theta_min: 0.5_f64.to_radians_py(),
            gate_chi2: 9.21,
            collinear_gap_m: 0.70,
            extent_decay_s: 1.0,
            confirm_min_consecutive_hits: 5,
            confirm_min_length_m: 1.0,
            confirm_max_dist_jitter_m: 0.05,
            confirm_max_angle_jitter_rad: 3.0_f64.to_radians_py(),
            confirm_require_lidar: true,
            delete_after_s: 2.0,
            max_sigma_rho_m: 0.3,
            max_published: 4,
            label_half_sector_rad: 45.0_f64.to_radians_py(),
            role_hysteresis_rad: 5.0_f64.to_radians_py(),
            ray_margin_m: 0.0,
            front_max_coast_s: 0.5,
            front_max_coast_m: 0.3,
            lidar_sigma_base_m: 0.02,
            lidar_sigma_per_m: 0.005,
            stereo_sigma_base_m: 0.03,
            stereo_sigma_quad_per_m2: 0.012,
            min_span_for_angle_m: 0.3,
            min_span_birth_m: 1.0,
            min_span_update_m: 0.5,
            front_x_m: 0.405,
            side_edge_half_width_m: 0.136,
            front_strip_margin_m: 0.15,
            stereo_front_max_normal_rad: 30.0_f64.to_radians_py(),
            front_min_confidence: 0.5,
            front_lidar_recent_cycles: 3,
            front_contradict_margin_m: 0.5,
            front_contradict_cycles: 2,
            stereo_min_rho_m: 0.4,
            camera_seg_min_ray_angle_rad: 25.0_f64.to_radians_py(),
            camera_seg_min_bearing_span_rad: 3.0_f64.to_radians_py(),
            stereo_max_rms_m: 0.05,
            stereo_min_vertical_extent_m: 0.4,
            stereo_min_length_m: 0.8,
            back_x_m: 0.0,
            side_x_min_m: -1.0,
            side_x_max_m: 3.0,
            side_max_normal_rad: 45.0_f64.to_radians_py(),
            camera_side_x_max_m: 4.0,
            camera_side_max_normal_rad: 30.0_f64.to_radians_py(),
            camera_side_offset_min_m: 0.3,
            camera_side_offset_max_m: 3.0,
            role_hysteresis_m: 0.05,
            occlusion_points: 5,
            occlusion_min_blocked: 3,
            occlusion_margin_m: 0.10,
            negative_shrink_beams: 2,
            negative_tol_m: 0.15,
            negative_min_beams: 5,
            negative_pass_frac: 0.7,
            negative_miss_to_kill: 5,
            coast_max_odom_m: 1.0,
        }
    }
}

/// CPython's `math.radians` (`x * (pi / 180)`), for the defaults only.
trait RadiansPy {
    fn to_radians_py(self) -> f64;
}

impl RadiansPy for f64 {
    fn to_radians_py(self) -> f64 {
        self * (PI / 180.0)
    }
}

impl Params {
    /// `strip_half_width_m`: the bodywork plus front_strip_margin_m.
    #[inline]
    pub fn strip_half_width_m(&self) -> f64 {
        self.side_edge_half_width_m + self.front_strip_margin_m
    }
}

/// The part of `wall_fusion.RectParams` the tracker's verify state reads.
#[derive(Debug, Clone, PartialEq)]
pub struct RectVerifyParams {
    pub coverage_min: f64,
    pub support_verify: f64,
    pub support_reject: f64,
    pub reject_cycles: i64,
    pub glass_beyond_share: f64,
    pub glass_at_share: f64,
}

impl Default for RectVerifyParams {
    fn default() -> Self {
        Self {
            coverage_min: 0.25,
            support_verify: 0.5,
            support_reject: 0.2,
            reject_cycles: 3,
            glass_beyond_share: 0.7,
            glass_at_share: 0.2,
        }
    }
}

/// A `wall_fusion.Observation`, the fields the tracker reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observation {
    pub line: WallLine,
    pub z_min: f64,
    pub z_max: f64,
    pub height_measured: bool,
    pub source: i32,
    pub confidence: f64,
    pub stereo_weight: f64,
}

/// `(ranges, base_link bearings, laser xy[, cut])`.
#[derive(Debug, Clone, PartialEq)]
pub struct Scan {
    pub ranges: Vec<f64>,
    pub bearings: Vec<f64>,
    pub laser_xy: [f64; 2],
    /// None: a 3-tuple scan (no cut mask).
    pub cut: Option<Vec<bool>>,
}

impl Scan {
    #[inline]
    fn cut_at(&self, i: usize) -> bool {
        self.cut.as_ref().is_some_and(|c| c[i])
    }
}

/// `StripReading`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct StripReading {
    pub range_m: Option<f64>,
    pub n_returns: i64,
    pub n_cut: i64,
    pub n_no_return: i64,
}

/// `front_status`.
#[derive(Debug, Clone, PartialEq)]
pub struct FrontStatus {
    pub id: Option<i64>,
    pub reason: FrontCode,
    pub distance_m: f64,
    /// `{id: code}` in insertion order.
    pub rejects: Vec<(i64, FrontCode)>,
}

impl Default for FrontStatus {
    fn default() -> Self {
        Self {
            id: None,
            reason: FrontCode::NoCandidate,
            distance_m: f64::NAN,
            rejects: Vec::new(),
        }
    }
}

/// A rectangle verdict (`verify_rectangles` entry) as the tracker reads it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RectVerdict {
    pub coverage: f64,
    pub support: f64,
    pub beyond_share: f64,
}

/// `Track`, field for field.
#[derive(Debug, Clone, PartialEq)]
pub struct Track {
    pub id: i64,
    pub x: [f64; 2],
    pub p: Mat2,
    pub t0: f64,
    pub t1: f64,
    pub born: f64,
    pub last_seen: f64,
    pub hits: i64,
    pub confirm_obs: PyDeque<Option<(f64, f64)>>,
    pub ever_confirmed: bool,
    pub state: i32,
    pub source: i32,
    pub has_lidar: bool,
    pub has_both: bool,
    pub z_min: f64,
    pub z_max: f64,
    pub height_measured: bool,
    pub confidence: f64,
    pub role: Role,
    pub verify: i32,
    pub glass: bool,
    pub rect_coverage: f64,
    pub rect_support: f64,
    pub rect_beyond_share: f64,
    pub rect_informative: bool,
    pub rect_reject_streak: i64,
    pub miss: i64,
    pub odo_at_last_seen: f64,
    pub occluded: bool,
    pub obs_ext: PyDeque<ExtEntry>,
    pub lidar_hist: PyDeque<bool>,
    pub conf_hist: PyDeque<f64>,
    pub strip_verdict: StripVerdict,
    pub strip_pred_m: f64,
    pub strip_miss: i64,
    pub contradicted: bool,
}

impl Track {
    /// `Track(id, x, P, t0, t1, born, last_seen)` with every default.
    pub fn new(id: i64, x: [f64; 2], p: Mat2, t0: f64, t1: f64, born: f64, last_seen: f64) -> Self {
        Self {
            id,
            x,
            p,
            t0,
            t1,
            born,
            last_seen,
            hits: 1,
            confirm_obs: PyDeque::new(Some(5)),
            ever_confirmed: false,
            state: STATE_TENTATIVE,
            source: SOURCE_LIDAR,
            has_lidar: false,
            has_both: false,
            z_min: 0.0,
            z_max: 2.5,
            height_measured: false,
            confidence: 0.0,
            role: Role::Unset,
            verify: VERIFY_UNVERIFIED,
            glass: false,
            rect_coverage: 0.0,
            rect_support: 0.0,
            rect_beyond_share: 0.0,
            rect_informative: false,
            rect_reject_streak: 0,
            miss: 0,
            odo_at_last_seen: 0.0,
            occluded: false,
            obs_ext: PyDeque::new(Some(64)),
            lidar_hist: PyDeque::new(Some(3)),
            conf_hist: PyDeque::new(Some(5)),
            strip_verdict: StripVerdict::Unset,
            strip_pred_m: f64::NAN,
            strip_miss: 0,
            contradicted: false,
        }
    }

    #[inline]
    pub fn rho(&self) -> f64 {
        self.x[0]
    }

    #[inline]
    pub fn theta(&self) -> f64 {
        self.x[1]
    }

    #[inline]
    pub fn length(&self) -> f64 {
        self.t1 - self.t0
    }

    /// `endpoints()`: the extent's two ends in odom.
    pub fn endpoints(&self) -> ([f64; 2], [f64; 2]) {
        let (c, s) = (self.theta().cos(), self.theta().sin());
        let rho = self.rho();
        let base = [c * rho, s * rho];
        let tan = [-s, c];
        (
            [base[0] + tan[0] * self.t0, base[1] + tan[1] * self.t0],
            [base[0] + tan[0] * self.t1, base[1] + tan[1] * self.t1],
        )
    }
}

/// A base_link segment `((x0, y0), (x1, y1))`.
pub type Seg = [[f64; 2]; 2];

/// `segment_in_base_link(track, car_pose)`.
pub fn segment_in_base_link(track: &Track, car: &Pose2D) -> Seg {
    let (p0, p1) = track.endpoints();
    let pts = car.inverse().transform_points(&[p0, p1]);
    [pts[0], pts[1]]
}

/// `_innovation(track_x, rho, alpha)`: whichever sign of the line is closer.
pub fn innovation(track_x: [f64; 2], rho: f64, alpha: f64) -> [f64; 2] {
    let v1 = [rho - track_x[0], wrap_pi(alpha - track_x[1])];
    let v2 = [-rho - track_x[0], wrap_pi((alpha + PI) - track_x[1])];
    if v2[1].abs() < v1[1].abs() {
        v2
    } else {
        v1
    }
}

/// `observation_covariance(source, distance_m, span_m, p, stereo_weight)`.
pub fn observation_covariance(
    source: i32,
    distance_m: f64,
    span_m: f64,
    p: &Params,
    stereo_weight: f64,
) -> Mat2 {
    let d = py_max(distance_m, 0.0);
    let w = py_min(1.0, py_max(0.0, stereo_weight));
    let lidar = p.lidar_sigma_base_m + p.lidar_sigma_per_m * d;
    let mut stereo = p.stereo_sigma_base_m + p.stereo_sigma_quad_per_m2 * d * d;
    stereo = if w > 0.0 { stereo / w } else { f64::INFINITY };
    let sigma_rho = if source == SOURCE_STEREO {
        stereo
    } else if source == SOURCE_BOTH && stereo.is_finite() {
        1.0 / (1.0 / py_pow2(lidar) + 1.0 / py_pow2(stereo)).sqrt()
    } else {
        lidar
    };
    let span = py_max(span_m, p.min_span_for_angle_m);
    let sigma_theta = py_max(sigma_rho / span, p.meas_sigma_theta_min);
    diag(
        py_pow2(py_max(sigma_rho, p.meas_sigma_rho_min)),
        py_pow2(sigma_theta),
    )
}

/// `observation_line(obs, car_pose, p)`: the observation as an odom Line.
pub fn observation_line(obs: &Observation, car: &Pose2D, p: &Params) -> Line {
    let wl = &obs.line;
    let (rho, alpha) = normalise_line(wl.d, py_atan2(wl.ny, wl.nx));
    let (a, b) = wl.endpoints();
    let line = Line {
        rho,
        alpha,
        cov: observation_covariance(obs.source, wl.d, wl.span(), p, obs.stereo_weight),
        p_start: a,
        p_end: b,
        n_points: wl.inliers.max(1),
    };
    line.transform(car)
}

/// `_clip_param(a, b, lo, hi)`: Liang-Barsky on one coordinate.
pub fn clip_param(a: f64, b: f64, lo: f64, hi: f64) -> Option<(f64, f64)> {
    let d = b - a;
    let (mut t0, mut t1) = (0.0_f64, 1.0_f64);
    for (sign, limit) in [(1.0_f64, hi), (-1.0_f64, -lo)] {
        let num = limit - sign * a;
        let den = sign * d;
        if den.abs() < 1e-12 {
            if num < 0.0 {
                return None;
            }
        } else if den > 0.0 {
            t1 = py_min(t1, num / den);
        } else {
            t0 = py_max(t0, num / den);
        }
    }
    if t0 <= t1 {
        Some((t0, t1))
    } else {
        None
    }
}

/// `strip_distance(seg, front_x, half_width_m)`.
pub fn strip_distance(seg: &Seg, front_x: f64, half_width_m: f64) -> f64 {
    let ([x0, y0], [x1, y1]) = (seg[0], seg[1]);
    let half = half_width_m.abs();
    let Some(span) = clip_param(y0, y1, -half, half) else {
        return f64::NAN;
    };
    let xs = (x0 + span.0 * (x1 - x0), x0 + span.1 * (x1 - x0));
    if py_max(xs.0, xs.1) <= front_x {
        return f64::NAN;
    }
    py_max(py_min(xs.0, xs.1) - front_x, 0.0)
}

/// `side_distance(track, car_pose, half_width_m)`.
pub fn side_distance(track: &Track, car: &Pose2D, half_width_m: f64) -> f64 {
    let n = [track.theta().cos(), track.theta().sin()];
    let d_centre = (track.rho() - dot2(n[0], n[1], car.x, car.y)).abs();
    py_max(0.0, d_centre - half_width_m.abs())
}

/// `strip_role(...)`: Front, Back or Unset.
pub fn strip_role(seg: &Seg, front_x: f64, back_x: f64, half_width_m: f64) -> Role {
    let ([x0, y0], [x1, y1]) = (seg[0], seg[1]);
    let half = half_width_m.abs();
    let Some(span) = clip_param(y0, y1, -half, half) else {
        return Role::Unset;
    };
    let xs = (x0 + span.0 * (x1 - x0), x0 + span.1 * (x1 - x0));
    if py_max(xs.0, xs.1) > front_x {
        return Role::Front;
    }
    if py_min(xs.0, xs.1) < back_x {
        Role::Back
    } else {
        Role::Unset
    }
}

/// `side_role(...)`: Left, Right or Unset.
pub fn side_role(
    track: &Track,
    car: &Pose2D,
    x_min: f64,
    x_max: f64,
    max_normal_rad: f64,
    seg: &Seg,
) -> Role {
    let ([x0, y0], [x1, y1]) = (seg[0], seg[1]);
    let Some(span) = clip_param(x0, x1, x_min, x_max) else {
        return Role::Unset;
    };
    let ny = (track.theta() - car.theta).sin();
    if ny.abs() < max_normal_rad.cos() {
        return Role::Unset;
    }
    let ys = (y0 + span.0 * (y1 - y0), y0 + span.1 * (y1 - y0));
    if 0.5 * (ys.0 + ys.1) > 0.0 {
        Role::Left
    } else {
        Role::Right
    }
}

/// `camera_side_role(track, car_pose, p, seg, widen_m, widen_rad)`.
pub fn camera_side_role(
    track: &Track,
    car: &Pose2D,
    p: &Params,
    seg: &Seg,
    widen_m: f64,
    widen_rad: f64,
) -> Role {
    let role = side_role(
        track,
        car,
        p.side_x_min_m - widen_m,
        p.camera_side_x_max_m + widen_m,
        p.camera_side_max_normal_rad + widen_rad,
        seg,
    );
    if !role.is_set() {
        return Role::Unset;
    }
    let offset = (0.5 * (seg[0][1] + seg[1][1])).abs();
    if !(p.camera_side_offset_min_m - widen_m <= offset
        && offset <= p.camera_side_offset_max_m + widen_m)
    {
        return Role::Unset;
    }
    role
}

/// `geometric_role(track, car_pose, p, widen_m, widen_rad, seg)`.
pub fn geometric_role(
    track: &Track,
    car: &Pose2D,
    p: &Params,
    widen_m: f64,
    widen_rad: f64,
    seg: &Seg,
) -> Role {
    let strip = strip_role(
        seg,
        p.front_x_m - widen_m,
        p.back_x_m + widen_m,
        p.strip_half_width_m() + widen_m,
    );
    if strip.is_set() {
        return strip;
    }
    let role = if track.source == SOURCE_STEREO {
        camera_side_role(track, car, p, seg, widen_m, widen_rad)
    } else {
        side_role(
            track,
            car,
            p.side_x_min_m - widen_m,
            p.side_x_max_m + widen_m,
            p.side_max_normal_rad + widen_rad,
            seg,
        )
    };
    if role.is_set() {
        role
    } else {
        Role::Unknown
    }
}

/// `role_with_hysteresis(track, car_pose, current, p, seg)`.
pub fn role_with_hysteresis(
    track: &Track,
    car: &Pose2D,
    current: Role,
    p: &Params,
    seg: &Seg,
) -> Role {
    let fresh = geometric_role(track, car, p, 0.0, 0.0, seg);
    if !current.is_set() || current == fresh {
        return fresh;
    }
    let widened = geometric_role(
        track,
        car,
        p,
        p.role_hysteresis_m,
        p.role_hysteresis_rad,
        seg,
    );
    if widened == current {
        current
    } else {
        fresh
    }
}

/// `line_jitter(samples, ref_xy)`: (distance spread, direction spread).
pub fn line_jitter(samples: &[(f64, f64)], ref_xy: [f64; 2]) -> (f64, f64) {
    let (rho0, theta0) = samples[0];
    let (rx, ry) = (ref_xy[0], ref_xy[1]);
    let mut dists = Vec::with_capacity(samples.len());
    let mut thetas = Vec::with_capacity(samples.len());
    for &(rho, theta) in samples {
        let v = innovation([rho0, theta0], rho, theta);
        let th = theta0 + v[1];
        dists.push(rho0 + v[0] - (th.cos() * rx + th.sin() * ry));
        thetas.push(v[1]);
    }
    (
        py_max_seq(&dists) - crate::pyc::py_min_seq(&dists),
        py_max_seq(&thetas) - crate::pyc::py_min_seq(&thetas),
    )
}

/// `confirmation_reason`'s codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum ConfirmCode {
    Ok = 0,
    Hits = 1,
    Length = 2,
    DistJitter = 3,
    AngleJitter = 4,
    NoLidar = 5,
}

/// `confirmation_reason(samples, length_m, p, ref_xy)`.
pub fn confirmation_reason(
    samples: &PyDeque<Option<(f64, f64)>>,
    length_m: f64,
    p: &Params,
    ref_xy: [f64; 2],
) -> ConfirmCode {
    let n = p.confirm_min_consecutive_hits.max(1) as usize;
    let all: Vec<Option<(f64, f64)>> = samples.items.iter().copied().collect();
    let recent: &[Option<(f64, f64)>] = if all.len() > n {
        &all[all.len() - n..]
    } else {
        &all
    };
    if recent.len() < n || recent.iter().any(Option::is_none) {
        return ConfirmCode::Hits;
    }
    if length_m < p.confirm_min_length_m {
        return ConfirmCode::Length;
    }
    let pts: Vec<(f64, f64)> = recent.iter().flatten().copied().collect();
    let (dist, angle) = line_jitter(&pts, ref_xy);
    if dist > p.confirm_max_dist_jitter_m {
        return ConfirmCode::DistJitter;
    }
    if angle > p.confirm_max_angle_jitter_rad {
        return ConfirmCode::AngleJitter;
    }
    ConfirmCode::Ok
}

/// `track_midpoint(track)`.
pub fn track_midpoint(track: &Track) -> [f64; 2] {
    let (p0, p1) = track.endpoints();
    [0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1])]
}

/// `radial_verdict(seg, origin_xy, min_ray_rad, min_span_rad)`.
pub fn radial_verdict(
    seg: &Seg,
    origin: [f64; 2],
    min_ray_rad: f64,
    min_span_rad: f64,
) -> (bool, f64, f64) {
    let ([x0, y0], [x1, y1]) = (seg[0], seg[1]);
    let (ox, oy) = (origin[0], origin[1]);
    let (mx, my) = (0.5 * (x0 + x1), 0.5 * (y0 + y1));
    let (rx, ry) = (mx - ox, my - oy);
    let (dx, dy) = (x1 - x0, y1 - y0);
    let (nr, nd) = (py_hypot(rx, ry), py_hypot(dx, dy));
    if nr < 1e-6 || nd < 1e-6 {
        return (false, 0.0, 0.0);
    }
    let cosang = py_min(1.0, (rx * dx + ry * dy).abs() / (nr * nd));
    let mut ray = cosang.acos();
    ray = py_min(ray, PI - ray);
    let b0 = py_atan2(y0 - oy, x0 - ox);
    let b1 = py_atan2(y1 - oy, x1 - ox);
    let span = wrap_pi(b1 - b0).abs();
    (ray >= min_ray_rad && span >= min_span_rad, ray, span)
}

/// `_bearing_sector(seg, laser_xy)`: (centre, half width).
pub fn bearing_sector(seg: &Seg, laser_xy: [f64; 2]) -> (f64, f64) {
    let (a, b) = (seg[0], seg[1]);
    let b0 = py_atan2(a[1] - laser_xy[1], a[0] - laser_xy[0]);
    let b1 = py_atan2(b[1] - laser_xy[1], b[0] - laser_xy[0]);
    let d = wrap_pi(b1 - b0);
    (wrap_pi(b0 + 0.5 * d), 0.5 * d.abs())
}

/// `_sectors_overlap(a, b)`.
#[inline]
pub fn sectors_overlap(a: (f64, f64), b: (f64, f64)) -> bool {
    wrap_pi(a.0 - b.0).abs() <= a.1 + b.1
}

/// `ray_blocked_by(points, laser_xy, blockers, margin_m)`.
pub fn ray_blocked_by(
    points: &[[f64; 2]],
    laser_xy: [f64; 2],
    blockers: &[Seg],
    margin_m: f64,
) -> Vec<bool> {
    let mut out = vec![false; points.len()];
    if points.is_empty() {
        return out;
    }
    let d: Vec<[f64; 2]> = points
        .iter()
        .map(|p| [p[0] - laser_xy[0], p[1] - laser_xy[1]])
        .collect();
    let norm: Vec<f64> = d.iter().map(|v| v[0].hypot(v[1])).collect();
    let ok: Vec<bool> = norm.iter().map(|&n| n > 1e-6).collect();
    if !ok.iter().any(|&o| o) {
        return out;
    }
    for q in blockers {
        let ([qx0, qy0], [qx1, qy1]) = (q[0], q[1]);
        let e = [qx1 - qx0, qy1 - qy0];
        let q0 = [qx0 - laser_xy[0], qy0 - laser_xy[1]];
        for k in 0..points.len() {
            let den = d[k][0] * e[1] - d[k][1] * e[0];
            let s = (q0[0] * e[1] - q0[1] * e[0]) / den;
            let u = (d[k][0] * q0[1] - d[k][1] * q0[0]) / -den;
            let near = s < 1.0 - margin_m / np_maximum(norm[k], 1e-6);
            let hit = den.abs() > 1e-12 && ok[k] && s > 1e-6 && near && (0.0..=1.0).contains(&u);
            out[k] |= hit;
        }
    }
    out
}

/// `strip_returns(scan, front_x, half_width_m)`.
pub fn strip_returns(scan: &Scan, front_x: f64, half_width_m: f64) -> StripReading {
    let (lx, ly) = (scan.laser_xy[0], scan.laser_xy[1]);
    let half = half_width_m.abs();
    let fx = front_x;
    let num = fx - lx;
    let mut best: Option<f64> = None;
    let (mut n_inside, mut n_cut, mut n_no) = (0_i64, 0_i64, 0_i64);
    for (i, (&r, &b)) in scan.ranges.iter().zip(scan.bearings.iter()).enumerate() {
        let (c, s) = (b.cos(), b.sin());
        let cut = scan.cut_at(i);
        let t_in = if c > 1e-6 { num / c } else { f64::INFINITY };
        let through = c > 1e-6 && (ly + t_in * s).abs() <= half;
        let finite = r.is_finite() && !cut;
        let rr = if finite { r } else { 0.0 };
        let px = lx + rr * c;
        let py = ly + rr * s;
        let inside = finite && px > fx && py.abs() <= half;
        if through && cut {
            n_cut += 1;
        }
        if through && !cut && !r.is_finite() {
            n_no += 1;
        }
        if inside {
            n_inside += 1;
            best = Some(match best {
                None => px,
                Some(m) => crate::pyc::np_minimum(m, px),
            });
        }
    }
    match best {
        None => StripReading {
            range_m: None,
            n_returns: 0,
            n_cut,
            n_no_return: n_no,
        },
        Some(m) => StripReading {
            range_m: Some(m - fx),
            n_returns: n_inside,
            n_cut,
            n_no_return: n_no,
        },
    }
}

/// `strip_verdict(pred_m, strip_range_m, margin_m)`.
pub fn strip_verdict(pred_m: f64, strip_range_m: Option<f64>, margin_m: f64) -> StripVerdict {
    if !pred_m.is_finite() {
        return StripVerdict::Unset;
    }
    let Some(r) = strip_range_m else {
        return StripVerdict::NoReturns;
    };
    if r > pred_m + margin_m {
        return StripVerdict::Contradicted;
    }
    if r < pred_m - margin_m {
        return StripVerdict::Blocked;
    }
    StripVerdict::Supported
}

/// `wall_fusion.verify_step(state, glass, verdict, p, reject_streak)`.
pub fn verify_step(
    state: i32,
    glass: bool,
    verdict: &RectVerdict,
    p: &RectVerifyParams,
    reject_streak: i64,
) -> (i32, bool, i64) {
    let mut state = state;
    let mut streak = reject_streak;
    if verdict.coverage < p.coverage_min {
        return (state, glass, streak);
    }
    let support = verdict.support;
    if support >= p.support_verify {
        state = VERIFY_VERIFIED;
        streak = 0;
    } else if support <= p.support_reject {
        streak += 1;
        if streak >= p.reject_cycles.max(1) {
            state = VERIFY_REJECTED;
        }
    } else {
        streak = 0;
    }
    if state != VERIFY_VERIFIED {
        return (state, false, streak);
    }
    let beyond = verdict.beyond_share;
    (
        state,
        beyond > p.glass_beyond_share && (1.0 - beyond) < p.glass_at_share,
        streak,
    )
}

/// Python slicing `seq[start:stop]` bounds for a length (ints, may be negative).
fn py_slice(len: usize, start: i64, stop: i64) -> (usize, usize) {
    let n = len as i64;
    let fix = |v: i64| -> usize {
        let v = if v < 0 { v + n } else { v };
        v.clamp(0, n) as usize
    };
    let (a, b) = (fix(start), fix(stop));
    if b < a {
        (a, a)
    } else {
        (a, b)
    }
}

/// The scan the step in progress was given, and its car pose (`_scan_ctx`).
#[derive(Debug, Clone)]
pub struct ScanCtx {
    pub car: Pose2D,
    pub scan: Scan,
}

/// `MultiWallTracker`.
#[derive(Debug, Clone)]
pub struct Tracker {
    pub p: Params,
    pub rect: RectVerifyParams,
    pub tracks: Vec<Track>,
    pub laser_xy: Option<[f64; 2]>,
    pub camera_xy: Option<[f64; 2]>,
    pub camera_fresh: bool,
    pub next_id: i64,
    pub last_stamp: Option<f64>,
    pub last_yaw: Option<f64>,
    pub n_short_dropped: i64,
    pub odometer: f64,
    pub last_car_t: Option<[f64; 2]>,
    pub n_negative_killed: i64,
    pub n_gap_rejected: i64,
    pub n_merge_gap_refused: i64,
    pub n_stereo_near_rejected: i64,
    pub n_camera_rejected: i64,
    pub rejections: Vec<Rejection>,
    pub strip: StripReading,
    pub front_status: FrontStatus,
    pub scan_ctx: Option<ScanCtx>,
    /// The next obs_ext serial (binding bookkeeping, not Python state).
    pub next_serial: u64,
}

/// `_open_length(theta, rho, t_lo, t_hi, unseen_open)`.
fn open_length(
    p: &Params,
    ctx: Option<&ScanCtx>,
    theta: f64,
    rho: f64,
    t_lo: f64,
    t_hi: f64,
    unseen_open: bool,
) -> f64 {
    let length = t_hi - t_lo;
    let Some(ctx) = ctx else {
        return length;
    };
    if length <= 0.0 {
        return length;
    }
    let ts = np_arange(t_lo + 0.5 * GAP_STEP_M, t_hi, GAP_STEP_M);
    if ts.is_empty() {
        return length;
    }
    let scan = &ctx.scan;
    let b = &scan.bearings;
    if b.len() < 2 {
        return length;
    }
    let (c, s) = (theta.cos(), theta.sin());
    let n = [c, s];
    let tan = [-n[1], n[0]];
    let odom: Vec<[f64; 2]> = ts
        .iter()
        .map(|&t| [n[0] * rho + t * tan[0], n[1] * rho + t * tan[1]])
        .collect();
    let pts = ctx.car.inverse().transform_points(&odom);
    let lxy = scan.laser_xy;
    let d: Vec<[f64; 2]> = pts.iter().map(|q| [q[0] - lxy[0], q[1] - lxy[1]]).collect();
    let dist: Vec<f64> = d.iter().map(|v| v[0].hypot(v[1])).collect();
    let bear: Vec<f64> = d.iter().map(|v| v[1].atan2(v[0])).collect();
    let r = &scan.ranges;
    let inc = (b[1] - b[0]).abs();
    let last = b.len() - 1;
    let idx: Vec<usize> = if b[last] > b[0] {
        let raw = np_searchsorted_left(b, &bear);
        raw.iter()
            .zip(bear.iter())
            .map(|(&i, &be)| {
                let i = i.clamp(1, last);
                if (b[i - 1] - be).abs() < (b[i] - be).abs() {
                    i - 1
                } else {
                    i
                }
            })
            .collect()
    } else {
        bear.iter()
            .map(|&be| {
                let row: Vec<f64> = b.iter().map(|&bb| wrap_pi(bb - be).abs()).collect();
                np_argmin(&row).unwrap_or(0)
            })
            .collect()
    };
    let tol = p.negative_tol_m;
    let mut count = 0_usize;
    for (k, &i) in idx.iter().enumerate() {
        let in_fov = wrap_pi(b[i] - bear[k]).abs() <= 1.5 * inc;
        let rv = r[i];
        let mut open = in_fov && !scan.cut_at(i) && (!rv.is_finite() || rv > dist[k] + tol);
        if unseen_open {
            open = open || !in_fov;
        }
        if open {
            count += 1;
        }
    }
    py_min(length, count as f64 * GAP_STEP_M)
}

/// `_gap_between(theta, rho, a_lo, a_hi, b_lo, b_hi, unseen_open)`.
#[allow(clippy::too_many_arguments)]
fn gap_between(
    p: &Params,
    ctx: Option<&ScanCtx>,
    theta: f64,
    rho: f64,
    a_lo: f64,
    a_hi: f64,
    b_lo: f64,
    b_hi: f64,
    unseen_open: bool,
) -> f64 {
    let raw = py_max(b_lo - a_hi, a_lo - b_hi);
    if raw < p.collinear_gap_m {
        return raw;
    }
    let (lo, hi) = if b_lo - a_hi >= a_lo - b_hi {
        (a_hi, b_lo)
    } else {
        (b_hi, a_lo)
    };
    open_length(p, ctx, theta, rho, lo, hi, unseen_open)
}

/// `_extent_gap(trk, p_a, p_b)`.
fn extent_gap(p: &Params, ctx: Option<&ScanCtx>, trk: &Track, pa: [f64; 2], pb: [f64; 2]) -> f64 {
    let m = along_many(trk.theta(), &[pa, pb]);
    let lo = np_min_reduce(&m).unwrap_or(f64::NAN);
    let hi = np_max_reduce(&m).unwrap_or(f64::NAN);
    gap_between(
        p,
        ctx,
        trk.theta(),
        trk.rho(),
        trk.t0,
        trk.t1,
        lo,
        hi,
        false,
    )
}

/// `_refresh_extent(trk, now)`.
fn refresh_extent(p: &Params, ctx: Option<&ScanCtx>, trk: &mut Track, now: Option<f64>) {
    if trk.obs_ext.is_empty() {
        return;
    }
    let newest_stamp = trk.obs_ext.items.back().map_or(f64::NAN, |e| e.stamp);
    let cutoff = now.unwrap_or(newest_stamp) - p.extent_decay_s;
    while trk.obs_ext.len() > 1 && trk.obs_ext.items.front().is_some_and(|e| e.stamp < cutoff) {
        trk.obs_ext.items.pop_front();
    }
    let theta = trk.theta();
    let newest = trk
        .obs_ext
        .items
        .back()
        .map_or([[f64::NAN; 2]; 2], |e| e.pts);
    let m = along_many(theta, &newest);
    let mut lo = np_min_reduce(&m).unwrap_or(f64::NAN);
    let mut hi = np_max_reduce(&m).unwrap_or(f64::NAN);
    let mut spans: Vec<(f64, f64)> = trk
        .obs_ext
        .items
        .iter()
        .map(|e| {
            let v = along_many(theta, &e.pts);
            (
                np_min_reduce(&v).unwrap_or(f64::NAN),
                np_max_reduce(&v).unwrap_or(f64::NAN),
            )
        })
        .collect();
    py_sort_pairs(&mut spans);
    let gap = p.collinear_gap_m;
    let rho = trk.rho();
    let mut grew = true;
    while grew {
        grew = false;
        for &(a, b) in &spans {
            if (a < lo || b > hi) && gap_between(p, ctx, theta, rho, lo, hi, a, b, false) < gap {
                lo = py_min(lo, a);
                hi = py_max(hi, b);
                grew = true;
            }
        }
    }
    trk.t0 = lo;
    trk.t1 = hi;
}

/// `_meas_cov(line)`: the line's covariance plus the sigma floor.
fn meas_cov(p: &Params, line: &Line) -> Mat2 {
    let floor = diag(
        py_pow2(p.meas_sigma_rho_min),
        py_pow2(p.meas_sigma_theta_min),
    );
    add(&line.cov, &floor)
}

/// Why `_gate` refused a pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GateWhy {
    None,
    Birth,
    Chi2,
    Gap,
}

impl Tracker {
    /// `MultiWallTracker(params, rect_params)`.
    pub fn new(p: Params, rect: RectVerifyParams) -> Self {
        Self {
            p,
            rect,
            tracks: Vec::new(),
            laser_xy: None,
            camera_xy: None,
            camera_fresh: true,
            next_id: 1,
            last_stamp: None,
            last_yaw: None,
            n_short_dropped: 0,
            odometer: 0.0,
            last_car_t: None,
            n_negative_killed: 0,
            n_gap_rejected: 0,
            n_merge_gap_refused: 0,
            n_stereo_near_rejected: 0,
            n_camera_rejected: 0,
            rejections: Vec::new(),
            strip: StripReading::default(),
            front_status: FrontStatus::default(),
            scan_ctx: None,
            next_serial: 1,
        }
    }

    fn reject(&mut self, code: RejectCode, track_id: Option<i64>, value: f64) {
        if self.rejections.len() < MAX_REJECTIONS {
            self.rejections.push(Rejection {
                code,
                track_id,
                value,
            });
        }
    }

    fn new_serial(&mut self) -> u64 {
        let s = self.next_serial;
        self.next_serial += 1;
        s
    }

    // ---- filter --------------------------------------------------------

    /// `_predict(dt, dyaw)`.
    fn predict(&mut self, dt: f64, dyaw: f64) {
        let dtp = py_max(dt, 0.0);
        let q: Mat2 = [
            [self.p.process_noise_rho * dtp, 0.0 * dtp],
            [0.0 * dtp, self.p.process_noise_theta * dtp],
        ];
        let turn = self.p.k_yaw * dyaw.abs();
        for trk in &mut self.tracks {
            let extra = diag(py_pow2(turn * trk.rho()), py_pow2(turn));
            trk.p = add(&add(&trk.p, &q), &extra);
        }
    }

    /// `_wall_spans(observations, lines, eligible)`.
    fn wall_spans(
        &self,
        observations: &[Observation],
        lines: &[Line],
        eligible: &[bool],
    ) -> Vec<f64> {
        let mut spans: Vec<f64> = observations.iter().map(|o| o.line.span()).collect();
        let idx: Vec<usize> = (0..observations.len())
            .filter(|&k| eligible[k] && observations[k].source != SOURCE_STEREO)
            .collect();
        if idx.len() < 2 || idx.iter().all(|&k| spans[k] >= self.p.min_span_birth_m) {
            return spans;
        }
        let mut parent: Vec<usize> = (0..idx.len()).collect();
        fn find(parent: &mut [usize], mut i: usize) -> usize {
            while parent[i] != i {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            i
        }
        let sub: Vec<Line> = idx.iter().map(|&k| lines[k]).collect();
        let chi2 = self.chi2_between(&sub);
        let ctx = self.scan_ctx.as_ref();
        for i in 0..idx.len() {
            for j in (i + 1)..idx.len() {
                // np.triu(chi2 <= gate, 1): a NaN is not the same line.
                let same_line = chi2[i * idx.len() + j] <= self.p.gate_chi2;
                if !same_line {
                    continue;
                }
                let (la, lb) = (&lines[idx[i]], &lines[idx[j]]);
                let ta = along_many(la.alpha, &[la.p_start, la.p_end]);
                let tb = along_many(la.alpha, &[lb.p_start, lb.p_end]);
                let gap = gap_between(
                    &self.p,
                    ctx,
                    la.alpha,
                    la.rho,
                    np_min_reduce(&ta).unwrap_or(f64::NAN),
                    np_max_reduce(&ta).unwrap_or(f64::NAN),
                    np_min_reduce(&tb).unwrap_or(f64::NAN),
                    np_max_reduce(&tb).unwrap_or(f64::NAN),
                    true,
                );
                if gap < self.p.collinear_gap_m {
                    // parent[find(j)] = find(i): Python evaluates the right side first.
                    let ri = find(&mut parent, i);
                    let rj = find(&mut parent, j);
                    parent[rj] = ri;
                }
            }
        }
        // groups in first-appearance order of their root, members in i order.
        let mut order: Vec<usize> = Vec::new();
        let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();
        for (i, &k) in idx.iter().enumerate() {
            let root = find(&mut parent, i);
            if !groups.contains_key(&root) {
                order.push(root);
            }
            groups.entry(root).or_default().push(k);
        }
        for root in order {
            let members = &groups[&root];
            if members.len() < 2 {
                continue;
            }
            // max(members, key=spans): the first maximal.
            let mut best = members[0];
            for &k in &members[1..] {
                if spans[k] > spans[best] {
                    best = k;
                }
            }
            let rf = &lines[best];
            let mut pts = Vec::with_capacity(2 * members.len());
            for &k in members {
                pts.push(lines[k].p_start);
                pts.push(lines[k].p_end);
            }
            let t = along_many(rf.alpha, &pts);
            let group_span =
                np_max_reduce(&t).unwrap_or(f64::NAN) - np_min_reduce(&t).unwrap_or(f64::NAN);
            for &k in members {
                spans[k] = py_max(spans[k], group_span);
            }
        }
        spans
    }

    /// `_chi2_between(lines)`: (n, n) row-major.
    fn chi2_between(&self, lines: &[Line]) -> Vec<f64> {
        let n = lines.len();
        let r: Vec<Mat2> = lines.iter().map(|l| meas_cov(&self.p, l)).collect();
        let mut out = vec![0.0; n * n];
        for i in 0..n {
            for j in 0..n {
                let (ai, aj) = (lines[i].alpha, lines[j].alpha);
                let (rhoi, rhoj) = (lines[i].rho, lines[j].rho);
                let d_same = wrap_pi(aj - ai);
                let d_flip = wrap_pi((aj + PI) - ai);
                let flip = d_flip.abs() < d_same.abs();
                let v0 = (if flip { -rhoj } else { rhoj }) - rhoi;
                let v1 = if flip { d_flip } else { d_same };
                let s = add(&r[i], &r[j]);
                let (a, b, c, d) = (s[0][0], s[0][1], s[1][0], s[1][1]);
                out[i * n + j] = (d * v0 * v0 - (b + c) * v0 * v1 + a * v1 * v1) / (a * d - b * c);
            }
        }
        out
    }

    /// `_refusal(obs)`: the first non-span gate it fails.
    fn refusal(&self, obs: &Observation) -> Option<(RejectCode, f64)> {
        let span = obs.line.span();
        if obs.source == SOURCE_STEREO && obs.line.d.abs() < self.p.stereo_min_rho_m {
            return Some((RejectCode::StereoNear, obs.line.d.abs()));
        }
        if obs.source == SOURCE_STEREO {
            if span < self.p.stereo_min_length_m {
                return Some((RejectCode::StereoShort, span));
            }
            if obs.line.rms > self.p.stereo_max_rms_m {
                return Some((RejectCode::StereoRms, obs.line.rms));
            }
            let height = obs.z_max - obs.z_min;
            if obs.height_measured && height < self.p.stereo_min_vertical_extent_m {
                return Some((RejectCode::StereoFlat, height));
            }
        }
        if obs.source != SOURCE_LIDAR {
            let origin = self.camera_xy.or(self.laser_xy).unwrap_or([0.0, 0.0]);
            let (a, b) = obs.line.endpoints();
            let (keep, ray, _span) = radial_verdict(
                &[a, b],
                origin,
                self.p.camera_seg_min_ray_angle_rad,
                self.p.camera_seg_min_bearing_span_rad,
            );
            if !keep {
                return Some((RejectCode::CameraRadial, ray * (180.0 / PI)));
            }
        }
        None
    }

    /// `_gate(trk, line, may_birth)`.
    fn gate(
        &self,
        trk: &Track,
        line: &Line,
        may_birth: bool,
    ) -> Result<(f64, GateWhy), TrackerError> {
        if !may_birth && !trk.ever_confirmed {
            return Ok((f64::INFINITY, GateWhy::Birth));
        }
        let v = innovation(trk.x, line.rho, line.alpha);
        let s = add(&trk.p, &meas_cov(&self.p, line));
        let sol = linalg::solve(&s, v)?;
        let d2 = dot2(v[0], v[1], sol[0], sol[1]);
        if d2 > self.p.gate_chi2 {
            return Ok((f64::INFINITY, GateWhy::Chi2));
        }
        if extent_gap(
            &self.p,
            self.scan_ctx.as_ref(),
            trk,
            line.p_start,
            line.p_end,
        ) >= self.p.collinear_gap_m
        {
            return Ok((f64::INFINITY, GateWhy::Gap));
        }
        Ok((d2, GateWhy::None))
    }

    /// `_absorb(trk, obs)`.
    fn absorb(trk: &mut Track, obs: &Observation) {
        trk.confidence = obs.confidence;
        trk.conf_hist.append(obs.confidence);
        if obs.source == SOURCE_BOTH {
            trk.has_lidar = true;
            trk.has_both = trk.has_both || trk.rect_informative;
        } else if obs.source == SOURCE_LIDAR {
            trk.has_lidar = true;
        }
        trk.source = if trk.has_both {
            SOURCE_BOTH
        } else if trk.has_lidar {
            SOURCE_LIDAR
        } else {
            SOURCE_STEREO
        };
        if obs.height_measured {
            trk.z_min = obs.z_min;
            trk.z_max = obs.z_max;
            trk.height_measured = true;
        }
    }

    /// `_update(trk, line, stamp)`.
    fn update(&mut self, ti: usize, line: &Line, stamp: f64) -> Result<(), TrackerError> {
        let serial = self.new_serial();
        let p = &self.p;
        let ctx = self.scan_ctx.as_ref();
        let odometer = self.odometer;
        let trk = &mut self.tracks[ti];
        let v = innovation(trk.x, line.rho, line.alpha);
        let r = meas_cov(p, line);
        let s = add(&trk.p, &r);
        let k = mat_mul(&trk.p, &linalg::inv(&s)?);
        let kv = mat_vec(&k, v);
        trk.x = [trk.x[0] + kv[0], trk.x[1] + kv[1]];
        let i_k = [
            [1.0 - k[0][0], 0.0 - k[0][1]],
            [0.0 - k[1][0], 1.0 - k[1][1]],
        ];
        trk.p = mat_mul(&i_k, &trk.p);
        let (rho, theta) = normalise_line(trk.x[0], trk.x[1]);
        trk.x = [rho, theta];
        trk.obs_ext.append(ExtEntry {
            stamp,
            pts: [line.p_start, line.p_end],
            serial,
        });
        refresh_extent(p, ctx, trk, Some(stamp));
        trk.hits += 1;
        trk.last_seen = stamp;
        trk.odo_at_last_seen = odometer;
        Ok(())
    }

    /// `_spawn(line, stamp, obs)`.
    pub fn spawn(&mut self, line: &Line, stamp: f64, obs: Option<&Observation>) {
        let m = along_many(line.alpha, &[line.p_start, line.p_end]);
        let mut trk = Track::new(
            self.next_id,
            [line.rho, line.alpha],
            meas_cov(&self.p, line),
            np_min_reduce(&m).unwrap_or(f64::NAN),
            np_max_reduce(&m).unwrap_or(f64::NAN),
            stamp,
            stamp,
        );
        trk.odo_at_last_seen = self.odometer;
        let source = obs.map_or(SOURCE_LIDAR, |o| o.source);
        let n_confirm = self.p.confirm_min_consecutive_hits.max(1) as usize;
        trk.confirm_obs =
            PyDeque::from_vec(vec![self.confirm_sample(line, source)], Some(n_confirm));
        let serial = self.new_serial();
        trk.obs_ext.append(ExtEntry {
            stamp,
            pts: [line.p_start, line.p_end],
            serial,
        });
        trk.lidar_hist = PyDeque::from_vec(
            vec![obs.is_none_or(|o| o.source != SOURCE_STEREO)],
            Some(self.p.front_lidar_recent_cycles.max(1) as usize),
        );
        trk.conf_hist = PyDeque::new(Some(n_confirm));
        if let Some(o) = obs {
            Self::absorb(&mut trk, o);
        }
        self.promote(&mut trk, true);
        self.next_id += 1;
        self.tracks.push(trk);
    }

    // ---- THE CONFIRMATION LEVEL ------------------------------------------

    /// `_confirm_sample(line, source)`.
    fn confirm_sample(&self, line: &Line, source: i32) -> Option<(f64, f64)> {
        if source == SOURCE_STEREO && (self.p.confirm_require_lidar || !self.camera_fresh) {
            return None;
        }
        Some((line.rho, line.alpha))
    }

    /// `confirm_reason(trk)`.
    pub fn confirm_reason(&self, trk: &Track) -> ConfirmCode {
        if self.p.confirm_require_lidar && trk.source == SOURCE_STEREO {
            return ConfirmCode::NoLidar;
        }
        confirmation_reason(&trk.confirm_obs, trk.length(), &self.p, track_midpoint(trk))
    }

    /// `_promote(trk, hit)`.
    fn promote(&self, trk: &mut Track, hit: bool) {
        if !trk.ever_confirmed && self.confirm_reason(trk) == ConfirmCode::Ok {
            trk.ever_confirmed = true;
        }
        trk.state = if trk.ever_confirmed {
            if hit {
                STATE_CONFIRMED
            } else {
                STATE_COASTING
            }
        } else {
            STATE_TENTATIVE
        };
    }

    /// `coasting(trk)`.
    pub fn coasting(&self, trk: &Track) -> bool {
        self.last_stamp.is_some_and(|s| trk.last_seen < s)
    }

    /// `coast_expired(trk)`.
    pub fn coast_expired(&self, trk: &Track) -> bool {
        if !self.coasting(trk) {
            return false;
        }
        let last = self.last_stamp.unwrap_or(f64::NAN);
        last - trk.last_seen > self.p.front_max_coast_s
            || self.odometer - trk.odo_at_last_seen > self.p.front_max_coast_m
    }

    /// `_merge()`.
    pub fn merge(&mut self) -> Result<(), TrackerError> {
        let mut refused: HashSet<(i64, i64)> = HashSet::new();
        let mut merged = true;
        while merged {
            merged = false;
            // sorted(self.tracks, key=id): stable, so equal ids keep list order.
            let mut order: Vec<usize> = (0..self.tracks.len()).collect();
            order.sort_by_key(|&k| self.tracks[k].id);
            'pairs: for ai in 0..order.len() {
                for bi in (ai + 1)..order.len() {
                    let (ia, ib) = (order[ai], order[bi]);
                    let (a, b) = (&self.tracks[ia], &self.tracks[ib]);
                    let v = innovation(a.x, b.rho(), b.theta());
                    let s = add(&a.p, &b.p);
                    let sol = linalg::solve(&s, v)?;
                    if dot2(v[0], v[1], sol[0], sol[1]) > self.p.gate_chi2 {
                        continue;
                    }
                    let (be0, be1) = b.endpoints();
                    let gap = extent_gap(&self.p, self.scan_ctx.as_ref(), a, be0, be1);
                    if gap >= self.p.collinear_gap_m {
                        let key = (a.id, b.id);
                        if !refused.contains(&key) {
                            refused.insert(key);
                            self.n_merge_gap_refused += 1;
                            let aid = a.id;
                            self.reject(RejectCode::GapMerge, Some(aid), gap);
                        }
                        continue;
                    }
                    self.fold(ia, ib, v)?;
                    merged = true;
                    break 'pairs;
                }
            }
        }
        Ok(())
    }

    /// The body of `_merge` for one pair: fold b into a, remove b.
    fn fold(&mut self, ia: usize, ib: usize, v: [f64; 2]) -> Result<(), TrackerError> {
        let b = self.tracks[ib].clone();
        let last_stamp = self.last_stamp;
        let coasting_after = {
            let a = &self.tracks[ia];
            let info_a = linalg::inv(&a.p)?;
            let info_b = linalg::inv(&b.p)?;
            let pm = linalg::inv(&add(&info_a, &info_b))?;
            let xa = a.x;
            let ia_x = mat_vec(&info_a, xa);
            let ib_x = mat_vec(&info_b, [xa[0] + v[0], xa[1] + v[1]]);
            let x = mat_vec(&pm, [ia_x[0] + ib_x[0], ia_x[1] + ib_x[1]]);
            let (rho, theta) = normalise_line(x[0], x[1]);
            let p = &self.p;
            let ctx = self.scan_ctx.as_ref();
            let a = &mut self.tracks[ia];
            a.x = [rho, theta];
            a.p = pm;
            let mut ext: Vec<ExtEntry> = a
                .obs_ext
                .items
                .iter()
                .chain(b.obs_ext.items.iter())
                .copied()
                .collect();
            ext.sort_by(|e, f| {
                if e.stamp < f.stamp {
                    std::cmp::Ordering::Less
                } else if f.stamp < e.stamp {
                    std::cmp::Ordering::Greater
                } else {
                    std::cmp::Ordering::Equal
                }
            });
            a.obs_ext = PyDeque::from_vec(ext, a.obs_ext.maxlen);
            refresh_extent(p, ctx, a, last_stamp);
            a.hits += b.hits;
            a.last_seen = py_max(a.last_seen, b.last_seen);
            a.odo_at_last_seen = py_max(a.odo_at_last_seen, b.odo_at_last_seen);
            a.born = py_min(a.born, b.born);
            a.miss = a.miss.max(b.miss);
            a.strip_miss = a.strip_miss.max(b.strip_miss);
            a.contradicted = a.contradicted || b.contradicted;
            a.rect_reject_streak = a.rect_reject_streak.max(b.rect_reject_streak);
            a.verify = if a.verify == VERIFY_REJECTED || b.verify == VERIFY_REJECTED {
                VERIFY_REJECTED
            } else {
                a.verify.max(b.verify)
            };
            a.glass = a.glass || b.glass;
            a.ever_confirmed = a.ever_confirmed || b.ever_confirmed;
            a.has_lidar = a.has_lidar || b.has_lidar;
            a.has_both = a.has_both || b.has_both;
            a.source = max_by_source_rank(a.source, b.source);
            a.ever_confirmed
        };
        if coasting_after {
            let coast = self.coasting(&self.tracks[ia]);
            self.tracks[ia].state = if coast {
                STATE_COASTING
            } else {
                STATE_CONFIRMED
            };
        }
        let a = &mut self.tracks[ia];
        // Newest-aligned combination of the histories.
        let ca: Vec<Option<(f64, f64)>> = a.confirm_obs.items.iter().rev().copied().collect();
        let cb: Vec<Option<(f64, f64)>> = b.confirm_obs.items.iter().rev().copied().collect();
        let mut comb: Vec<Option<(f64, f64)>> = (0..ca.len().max(cb.len()))
            .map(|k| {
                let x = ca.get(k).copied().flatten();
                let y = cb.get(k).copied().flatten();
                if x.is_some() {
                    x
                } else {
                    y
                }
            })
            .collect();
        comb.reverse();
        a.confirm_obs = PyDeque::from_vec(comb, a.confirm_obs.maxlen);
        let la: Vec<bool> = a.lidar_hist.items.iter().rev().copied().collect();
        let lb: Vec<bool> = b.lidar_hist.items.iter().rev().copied().collect();
        let mut lcomb: Vec<bool> = (0..la.len().max(lb.len()))
            .map(|k| la.get(k).copied().unwrap_or(false) || lb.get(k).copied().unwrap_or(false))
            .collect();
        lcomb.reverse();
        a.lidar_hist = PyDeque::from_vec(lcomb, a.lidar_hist.maxlen);
        for &c in &b.conf_hist.items {
            a.conf_hist.append(c);
        }
        self.tracks.remove(ib);
        Ok(())
    }

    // ---- one scan --------------------------------------------------------

    /// `step(observations, stamp, car_pose, scan, defer_roles)`: the indices
    /// of the published tracks (empty when `defer_roles`).
    pub fn step(
        &mut self,
        observations: &[Observation],
        stamp: f64,
        car: &Pose2D,
        scan: Option<Scan>,
        defer_roles: bool,
    ) -> Result<Vec<usize>, TrackerError> {
        self.scan_ctx = scan.map(|s| ScanCtx { car: *car, scan: s });
        if let Some(t) = self.last_car_t {
            self.odometer += (car.x - t[0]).hypot(car.y - t[1]);
        }
        self.last_car_t = Some(car.t());
        self.n_short_dropped = 0;
        self.n_stereo_near_rejected = 0;
        self.n_camera_rejected = 0;
        self.n_gap_rejected = 0;
        self.n_merge_gap_refused = 0;
        self.rejections.clear();
        let lines: Vec<Line> = observations
            .iter()
            .map(|o| observation_line(o, car, &self.p))
            .collect();
        let refusals: Vec<Option<(RejectCode, f64)>> =
            observations.iter().map(|o| self.refusal(o)).collect();
        let eligible: Vec<bool> = refusals.iter().map(Option::is_none).collect();
        let spans = self.wall_spans(observations, &lines, &eligible);
        let mut kept: Vec<Observation> = Vec::new();
        let mut lines_odom: Vec<Line> = Vec::new();
        let mut may_birth: Vec<bool> = Vec::new();
        for (k, obs) in observations.iter().enumerate() {
            let span = spans[k];
            if span < self.p.min_span_update_m {
                self.n_short_dropped += 1;
                continue;
            }
            if let Some((code, value)) = refusals[k] {
                if code == RejectCode::StereoNear {
                    self.n_stereo_near_rejected += 1;
                } else {
                    self.n_camera_rejected += 1;
                }
                self.reject(code, None, value);
                continue;
            }
            kept.push(*obs);
            lines_odom.push(lines[k]);
            may_birth.push(span >= self.p.min_span_birth_m);
        }
        let observations = kept;
        let dt = self.last_stamp.map_or(0.0, |s| stamp - s);
        self.last_stamp = Some(stamp);
        let last_yaw = self.last_yaw;
        self.last_yaw = Some(car.theta);
        self.predict(dt, last_yaw.map_or(0.0, |y| wrap_pi(car.theta - y)));

        let mut used_lines: HashSet<usize> = HashSet::new();
        let mut hit_ids: HashSet<i64> = HashSet::new();
        let mut lidar_ids: HashSet<i64> = HashSet::new();
        let mut samples: HashMap<i64, Option<(f64, f64)>> = HashMap::new();
        // gap_refused: observation j -> (track id, gap), insertion order.
        let mut gap_refused: Vec<(usize, i64, f64)> = Vec::new();
        if !self.tracks.is_empty() && !lines_odom.is_empty() {
            let (nt, nl) = (self.tracks.len(), lines_odom.len());
            let mut cost = vec![f64::INFINITY; nt * nl];
            for i in 0..nt {
                for j in 0..nl {
                    let (c, why) = self.gate(&self.tracks[i], &lines_odom[j], may_birth[j])?;
                    cost[i * nl + j] = c;
                    if why == GateWhy::Gap && !gap_refused.iter().any(|g| g.0 == j) {
                        let g = extent_gap(
                            &self.p,
                            self.scan_ctx.as_ref(),
                            &self.tracks[i],
                            lines_odom[j].p_start,
                            lines_odom[j].p_end,
                        );
                        gap_refused.push((j, self.tracks[i].id, g));
                    }
                }
            }
            let finite: Vec<bool> = cost.iter().map(|c| c.is_finite()).collect();
            if finite.iter().any(|&f| f) {
                let fin: Vec<f64> = cost
                    .iter()
                    .zip(&finite)
                    .filter(|(_, &f)| f)
                    .map(|(&c, _)| c)
                    .collect();
                let big = np_max_reduce(&fin).unwrap_or(f64::NAN) * 10.0 + 1e6;
                let filled: Vec<f64> = cost
                    .iter()
                    .zip(&finite)
                    .map(|(&c, &f)| if f { c } else { big })
                    .collect();
                let (rows, cols) = linear_sum_assignment(nt, nl, &filled)?;
                for (&i, &j) in rows.iter().zip(cols.iter()) {
                    if !finite[i * nl + j] {
                        continue;
                    }
                    if self.tracks[i].glass {
                        lines_odom[j].cov = observation_covariance(
                            SOURCE_LIDAR,
                            observations[j].line.d,
                            observations[j].line.span(),
                            &self.p,
                            1.0,
                        );
                    }
                    let line = lines_odom[j];
                    self.update(i, &line, stamp)?;
                    Self::absorb(&mut self.tracks[i], &observations[j]);
                    used_lines.insert(j);
                    let id = self.tracks[i].id;
                    hit_ids.insert(id);
                    if observations[j].source != SOURCE_STEREO {
                        lidar_ids.insert(id);
                    }
                    let sample = self.confirm_sample(&line, observations[j].source);
                    samples.insert(id, sample);
                }
            }
        }
        for &(j, tid, gap) in &gap_refused {
            if !used_lines.contains(&j) {
                self.n_gap_rejected += 1;
                self.reject(RejectCode::GapAssoc, Some(tid), gap);
            }
        }
        for ti in 0..self.tracks.len() {
            let id = self.tracks[ti].id;
            let hit = hit_ids.contains(&id);
            let lid = lidar_ids.contains(&id);
            let sample = samples.get(&id).copied().flatten();
            let mut trk = std::mem::replace(
                &mut self.tracks[ti],
                Track::new(0, [0.0; 2], [[0.0; 2]; 2], 0.0, 0.0, 0.0, 0.0),
            );
            trk.lidar_hist.append(lid);
            trk.confirm_obs.append(sample);
            self.promote(&mut trk, hit);
            if !hit {
                refresh_extent(&self.p, self.scan_ctx.as_ref(), &mut trk, Some(stamp));
            }
            self.tracks[ti] = trk;
        }
        for (j, line) in lines_odom.iter().enumerate() {
            if !used_lines.contains(&j) && may_birth[j] {
                self.spawn(line, stamp, Some(&observations[j]));
            }
        }

        self.n_negative_killed = 0;
        let segments: HashMap<i64, Seg> = self
            .tracks
            .iter()
            .map(|t| (t.id, segment_in_base_link(t, car)))
            .collect();
        if self.scan_ctx.is_some() {
            self.apply_negative_evidence(car, &segments);
        }
        let keep: Vec<bool> = self
            .tracks
            .iter()
            .map(|t| !self.expired(t, stamp))
            .collect();
        let mut k = 0;
        self.tracks.retain(|_| {
            let r = keep[k];
            k += 1;
            r
        });
        self.merge()?;
        self.apply_strip_evidence(car);
        if defer_roles {
            return Ok(Vec::new());
        }
        self.update_roles(car);
        Ok(self.published(car))
    }

    // ---- the travel strip's own beams ------------------------------------

    /// `_apply_strip_evidence(car_pose, scan)` (the scan is `scan_ctx`'s).
    fn apply_strip_evidence(&mut self, car: &Pose2D) {
        let Some(ctx) = self.scan_ctx.as_ref() else {
            self.strip = StripReading::default();
            for trk in &mut self.tracks {
                trk.strip_verdict = StripVerdict::Unset;
                trk.strip_pred_m = f64::NAN;
            }
            return;
        };
        let half = self.p.strip_half_width_m();
        self.strip = strip_returns(&ctx.scan, self.p.front_x_m, half);
        let strip_range = self.strip.range_m;
        for trk in &mut self.tracks {
            let pred = strip_distance(&segment_in_base_link(trk, car), self.p.front_x_m, half);
            trk.strip_pred_m = pred;
            let verdict = strip_verdict(pred, strip_range, self.p.front_contradict_margin_m);
            trk.strip_verdict = if trk.ever_confirmed {
                verdict
            } else {
                StripVerdict::Unset
            };
            if !trk.ever_confirmed {
                continue;
            }
            if verdict == StripVerdict::Contradicted {
                trk.strip_miss += 1;
                if trk.strip_miss >= self.p.front_contradict_cycles {
                    trk.contradicted = true;
                }
            } else {
                trk.strip_miss = 0;
                if verdict == StripVerdict::Supported {
                    trk.contradicted = false;
                }
            }
        }
    }

    // ---- negative evidence ---------------------------------------------

    /// `negative_evidence(trk, car_pose, scan, seg)`: (contradicted, n_pass, n_hit).
    pub fn negative_evidence(
        &self,
        trk: &Track,
        car: &Pose2D,
        scan: &Scan,
        seg: Option<&Seg>,
    ) -> (bool, i64, i64) {
        let own;
        let seg = match seg {
            Some(s) => s,
            None => {
                own = segment_in_base_link(trk, car);
                &own
            }
        };
        let (p0, p1) = (seg[0], seg[1]);
        let r = &scan.ranges;
        let b = &scan.bearings;
        let lxy = scan.laser_xy;
        let b0 = py_atan2(p0[1] - lxy[1], p0[0] - lxy[0]);
        let b1 = py_atan2(p1[1] - lxy[1], p1[0] - lxy[0]);
        let half = 0.5 * wrap_pi(b1 - b0).abs();
        let centre = wrap_pi(b0 + 0.5 * wrap_pi(b1 - b0));
        let inside: Vec<usize> = if b.len() > 1 && b[b.len() - 1] > b[0] {
            let lohi = np_searchsorted_left(b, &[centre - half, centre + half]);
            (lohi[0]..lohi[1].max(lohi[0])).collect()
        } else {
            (0..b.len())
                .filter(|&i| wrap_pi(b[i] - centre).abs() <= half)
                .collect()
        };
        let k = self.p.negative_shrink_beams;
        if (inside.len() as i64) <= 2 * k {
            return (false, 0, 0);
        }
        let (s0, s1) = py_slice(inside.len(), k, inside.len() as i64 - k);
        let inside = &inside[s0..s1];
        let theta_bl = trk.theta() - car.theta;
        let n = [theta_bl.cos(), theta_bl.sin()];
        let d = dot2(n[0], n[1], p0[0], p0[1]);
        let dl = d - dot2(n[0], n[1], lxy[0], lxy[1]);
        let tol = self.p.negative_tol_m;
        let mut any_usable = false;
        let (mut n_pass, mut n_hit) = (0_i64, 0_i64);
        for &i in inside {
            let bv = b[i];
            let inc = n[0] * bv.cos() + n[1] * bv.sin();
            let usable = inc.abs() > 0.05;
            if !usable {
                continue;
            }
            any_usable = true;
            let s = dl / inc;
            let rv = r[i];
            let mut ahead = s > 0.05;
            if let Some(cut) = &scan.cut {
                ahead = ahead && !cut[i];
            }
            let hit = ahead && (rv - s).abs() <= tol;
            let passed = ahead && !hit && (!rv.is_finite() || rv > s + tol);
            if passed {
                n_pass += 1;
            }
            if hit {
                n_hit += 1;
            }
        }
        if !any_usable {
            return (false, 0, 0);
        }
        let total = n_pass + n_hit;
        if total < self.p.negative_min_beams {
            return (false, n_pass, n_hit);
        }
        (
            (n_pass as f64) / (total as f64) > self.p.negative_pass_frac,
            n_pass,
            n_hit,
        )
    }

    /// `_apply_negative_evidence(car_pose, scan, segments)`.
    fn apply_negative_evidence(&mut self, car: &Pose2D, segments: &HashMap<i64, Seg>) {
        let Some(ctx) = self.scan_ctx.as_ref() else {
            return;
        };
        let mut misses = Vec::with_capacity(self.tracks.len());
        for trk in &self.tracks {
            if !trk.ever_confirmed || trk.glass {
                misses.push(0);
                continue;
            }
            let (contradicted, _np, _nh) =
                self.negative_evidence(trk, car, &ctx.scan, segments.get(&trk.id));
            misses.push(if contradicted { trk.miss + 1 } else { 0 });
        }
        for (trk, m) in self.tracks.iter_mut().zip(misses) {
            trk.miss = m;
        }
        let kill = self.p.negative_miss_to_kill;
        let n_killed = self.tracks.iter().filter(|t| t.miss >= kill).count();
        if n_killed > 0 {
            self.n_negative_killed = n_killed as i64;
            self.tracks.retain(|t| t.miss < kill);
        }
    }

    // ---- first obstacle --------------------------------------------------

    /// `update_occlusion(car_pose, segments)`.
    pub fn update_occlusion(&mut self, segments: &HashMap<i64, Seg>) {
        let confirmed: Vec<i64> = self
            .tracks
            .iter()
            .filter(|t| t.ever_confirmed && !t.contradicted)
            .map(|t| t.id)
            .collect();
        let laser_xy = self.laser_xy.unwrap_or([0.0, 0.0]);
        let n_pts = self.p.occlusion_points.max(2) as usize;
        let sectors: HashMap<i64, (f64, f64)> = segments
            .iter()
            .map(|(&id, seg)| (id, bearing_sector(seg, laser_xy)))
            .collect();
        let f = np_linspace(0.0, 1.0, n_pts);
        let mut occluded = Vec::with_capacity(self.tracks.len());
        for trk in &self.tracks {
            let mine = sectors[&trk.id];
            let blockers: Vec<Seg> = confirmed
                .iter()
                .filter(|&&id| id != trk.id && sectors_overlap(mine, sectors[&id]))
                .map(|id| segments[id])
                .collect();
            if blockers.is_empty() {
                occluded.push(false);
                continue;
            }
            let [a, b] = segments[&trk.id];
            let pts: Vec<[f64; 2]> = f
                .iter()
                .map(|&fi| [a[0] + fi * (b[0] - a[0]), a[1] + fi * (b[1] - a[1])])
                .collect();
            let blocked = ray_blocked_by(&pts, laser_xy, &blockers, self.p.occlusion_margin_m);
            let n_blocked = blocked.iter().filter(|&&x| x).count() as i64;
            occluded.push(n_blocked >= self.p.occlusion_min_blocked);
        }
        for (trk, o) in self.tracks.iter_mut().zip(occluded) {
            trk.occluded = o;
        }
    }

    /// `_expired(trk, stamp)`.
    fn expired(&self, trk: &Track, stamp: f64) -> bool {
        if stamp - trk.last_seen > self.p.delete_after_s {
            return true;
        }
        if self.odometer - trk.odo_at_last_seen > self.p.coast_max_odom_m {
            return true;
        }
        if py_max(trk.p[0][0], 0.0).sqrt() > self.p.max_sigma_rho_m {
            return true;
        }
        !trk.ever_confirmed && trk.last_seen < stamp
    }

    /// `published(car_pose)`: indices into `tracks`.
    pub fn published(&self, car: &Pose2D) -> Vec<usize> {
        let mut front = Vec::new();
        let mut left = Vec::new();
        let mut right = Vec::new();
        let mut back = Vec::new();
        for (k, trk) in self.tracks.iter().enumerate() {
            if !trk.ever_confirmed {
                continue;
            }
            let role = if trk.role.is_set() {
                trk.role
            } else {
                geometric_role(trk, car, &self.p, 0.0, 0.0, &segment_in_base_link(trk, car))
            };
            match role {
                Role::Front => front.push(k),
                Role::Left => left.push(k),
                Role::Right => right.push(k),
                Role::Back => back.push(k),
                _ => {}
            }
        }
        let perpendicular = |trk: &Track| {
            let n = [trk.theta().cos(), trk.theta().sin()];
            (trk.rho() - dot2(n[0], n[1], car.x, car.y)).abs()
        };
        let along_heading = |k: &usize| {
            let trk = &self.tracks[*k];
            let d = strip_distance(
                &segment_in_base_link(trk, car),
                self.p.front_x_m,
                self.p.strip_half_width_m(),
            );
            if d.is_finite() {
                d
            } else {
                perpendicular(trk)
            }
        };
        let from_the_side =
            |k: &usize| side_distance(&self.tracks[*k], car, self.p.side_edge_half_width_m);
        py_sort_by_key(&mut front, along_heading);
        py_sort_by_key(&mut left, from_the_side);
        py_sort_by_key(&mut right, from_the_side);
        py_sort_by_key(&mut back, |k: &usize| perpendicular(&self.tracks[*k]));
        let mut order: Vec<usize> = front;
        order.extend(left.iter().take(1));
        order.extend(right.iter().take(1));
        order.extend(back);
        order.extend(left.iter().skip(1));
        order.extend(right.iter().skip(1));
        let (a, b) = py_slice(order.len(), 0, self.p.max_published);
        order[a..b].to_vec()
    }

    /// `apply_rect_verdicts(verdicts)`: `verdicts` by track id.
    pub fn apply_rect_verdicts(&mut self, verdicts: &HashMap<i64, RectVerdict>) {
        let rect = self.rect.clone();
        for trk in &mut self.tracks {
            let Some(v) = verdicts.get(&trk.id) else {
                trk.rect_informative = false;
                continue;
            };
            trk.rect_coverage = v.coverage;
            trk.rect_support = v.support;
            trk.rect_beyond_share = v.beyond_share;
            trk.rect_informative = trk.rect_coverage >= rect.coverage_min;
            let (verify, glass, streak) =
                verify_step(trk.verify, trk.glass, v, &rect, trk.rect_reject_streak);
            trk.verify = verify;
            trk.glass = glass;
            trk.rect_reject_streak = streak;
        }
    }

    /// `update_roles(car_pose)`.
    pub fn update_roles(&mut self, car: &Pose2D) {
        let segments: HashMap<i64, Seg> = self
            .tracks
            .iter()
            .map(|t| (t.id, segment_in_base_link(t, car)))
            .collect();
        self.update_occlusion(&segments);
        let p = self.p.clone();
        for trk in &mut self.tracks {
            if trk.verify == VERIFY_REJECTED || trk.occluded || trk.contradicted {
                trk.role = Role::Unknown;
                continue;
            }
            trk.role = role_with_hysteresis(trk, car, trk.role, &p, &segments[&trk.id]);
        }
    }

    /// `front_confidence(trk)`.
    pub fn front_confidence(&self, trk: &Track) -> f64 {
        let items: Vec<f64> = trk.conf_hist.items.iter().copied().collect();
        let mut conf = match py_sum(&items) {
            Some(s) => s / (items.len() as f64),
            None => trk.confidence,
        };
        if trk.verify == VERIFY_VERIFIED {
            conf = py_max(conf, trk.rect_support);
        }
        conf
    }

    /// `front_reason(trk, car_pose, front_x)`.
    pub fn front_reason(&self, trk: &Track, car: &Pose2D, front_x: f64) -> FrontCode {
        if !trk.ever_confirmed {
            return FrontCode::Unconfirmed;
        }
        if trk.contradicted {
            return FrontCode::Contradicted;
        }
        if trk.role.is_set() && trk.role.is_unknown() {
            return FrontCode::RoleOther;
        }
        let d = strip_distance(
            &segment_in_base_link(trk, car),
            front_x,
            self.p.strip_half_width_m(),
        );
        if !d.is_finite() {
            return FrontCode::OffStrip;
        }
        if self.coasting(trk) {
            if self.coast_expired(trk) {
                return FrontCode::CoastExpired;
            }
            if trk.strip_verdict == StripVerdict::Contradicted {
                return FrontCode::CoastContradicted;
            }
        }
        if self.front_confidence(trk) < self.p.front_min_confidence {
            return FrontCode::LowConfidence;
        }
        if !trk.lidar_hist.items.iter().any(|&x| x) && trk.verify != VERIFY_VERIFIED {
            return FrontCode::Unverified;
        }
        if trk.source == SOURCE_STEREO {
            let n = [trk.theta().cos(), trk.theta().sin()];
            let to_wall = if dot2(n[0], n[1], car.x, car.y) <= trk.rho() {
                trk.theta()
            } else {
                trk.theta() + PI
            };
            if wrap_pi(to_wall - car.theta).abs() > self.p.stereo_front_max_normal_rad {
                return FrontCode::StereoOblique;
            }
        }
        FrontCode::Ok
    }

    /// `front_track(car_pose, front_x)`: the index of the FRONT track; fills
    /// `front_status`.
    pub fn front_track(&mut self, car: &Pose2D, front_x: f64) -> Option<usize> {
        let half = self.p.strip_half_width_m();
        let mut rejects: Vec<(i64, FrontCode)> = Vec::new();
        let mut nearest = (f64::INFINITY, FrontCode::NoCandidate);
        for trk in &self.tracks {
            if !trk.ever_confirmed {
                continue;
            }
            let d = strip_distance(&segment_in_base_link(trk, car), front_x, half);
            if !d.is_finite() {
                continue;
            }
            let code = self.front_reason(trk, car, front_x);
            if code != FrontCode::Ok {
                if let Some(e) = rejects.iter_mut().find(|e| e.0 == trk.id) {
                    e.1 = code;
                } else {
                    rejects.push((trk.id, code));
                }
                if d < nearest.0 {
                    nearest = (d, code);
                }
            }
        }
        let mut best: Option<usize> = None;
        let mut best_d = f64::INFINITY;
        for k in self.published(car) {
            let trk = &self.tracks[k];
            if rejects.iter().any(|e| e.0 == trk.id)
                || self.front_reason(trk, car, front_x) != FrontCode::Ok
            {
                continue;
            }
            let d = strip_distance(&segment_in_base_link(trk, car), front_x, half);
            if d < best_d {
                best = Some(k);
                best_d = d;
            }
        }
        self.front_status = FrontStatus {
            id: best.map(|k| self.tracks[k].id),
            reason: if best.is_some() {
                FrontCode::Front
            } else {
                nearest.1
            },
            distance_m: if best.is_some() { best_d } else { f64::NAN },
            rejects,
        };
        best
    }
}

/// `max(a.source, b.source, key=_SOURCE_RANK.get)`: the first maximal.
fn max_by_source_rank(a: i32, b: i32) -> i32 {
    let rank = |s: i32| match s {
        SOURCE_STEREO => 0,
        SOURCE_LIDAR => 1,
        SOURCE_BOTH => 2,
        _ => -1,
    };
    if rank(b) > rank(a) {
        b
    } else {
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn obs(d: f64, t0: f64, t1: f64) -> Observation {
        Observation {
            line: WallLine {
                nx: 1.0,
                ny: 0.0,
                d,
                t0,
                t1,
                inliers: 40,
                rms: 0.005,
            },
            z_min: 0.0,
            z_max: 2.5,
            height_measured: false,
            source: SOURCE_LIDAR,
            confidence: 0.8,
            stereo_weight: 1.0,
        }
    }

    #[test]
    fn a_steady_wall_confirms_on_the_fifth_cycle_and_is_front() {
        let mut trk = Tracker::new(Params::default(), RectVerifyParams::default());
        let car = Pose2D::new(0.0, 0.0, 0.0);
        for i in 0..5 {
            let out = trk
                .step(
                    &[obs(2.0, -1.0, 1.0)],
                    f64::from(i) * 0.1,
                    &car,
                    None,
                    false,
                )
                .unwrap();
            assert_eq!(trk.tracks.len(), 1);
            assert_eq!(trk.tracks[0].ever_confirmed, i == 4, "cycle {i}");
            assert_eq!(out.len(), usize::from(i == 4));
        }
        assert_eq!(trk.tracks[0].role, Role::Front);
        assert_eq!(trk.front_track(&car, 0.405), Some(0));
        assert_eq!(trk.front_status.reason, FrontCode::Front);
        assert!((trk.front_status.distance_m - (2.0 - 0.405)).abs() < 1e-9);
    }

    #[test]
    fn a_tentative_track_that_misses_a_cycle_is_deleted() {
        let mut trk = Tracker::new(Params::default(), RectVerifyParams::default());
        let car = Pose2D::new(0.0, 0.0, 0.0);
        trk.step(&[obs(2.0, -1.0, 1.0)], 0.0, &car, None, false)
            .unwrap();
        assert_eq!(trk.tracks.len(), 1);
        trk.step(&[], 0.1, &car, None, false).unwrap();
        assert!(trk.tracks.is_empty());
        assert_eq!(trk.next_id, 2);
    }

    #[test]
    fn a_short_observation_is_dropped() {
        let mut trk = Tracker::new(Params::default(), RectVerifyParams::default());
        let car = Pose2D::new(0.0, 0.0, 0.0);
        trk.step(&[obs(2.0, -0.1, 0.1)], 0.0, &car, None, false)
            .unwrap();
        assert!(trk.tracks.is_empty());
        assert_eq!(trk.n_short_dropped, 1);
    }

    #[test]
    fn deque_keeps_the_last_maxlen() {
        let mut d = PyDeque::new(Some(3));
        for k in 0..5 {
            d.append(k);
        }
        assert_eq!(d.items.iter().copied().collect::<Vec<_>>(), vec![2, 3, 4]);
        let e = PyDeque::from_vec(vec![1, 2, 3, 4], Some(2));
        assert_eq!(e.items.iter().copied().collect::<Vec<_>>(), vec![3, 4]);
    }

    #[test]
    fn python_slices() {
        assert_eq!(py_slice(5, 0, 4), (0, 4));
        assert_eq!(py_slice(3, 0, 4), (0, 3));
        assert_eq!(py_slice(5, 0, -1), (0, 4));
        assert_eq!(py_slice(5, 2, 1), (2, 2));
        assert_eq!(py_slice(5, -1, 6), (4, 5));
    }

    #[test]
    fn verify_step_is_sticky() {
        let p = RectVerifyParams::default();
        let v = RectVerdict {
            coverage: 0.9,
            support: 0.9,
            beyond_share: 0.9,
        };
        assert_eq!(
            verify_step(VERIFY_UNVERIFIED, false, &v, &p, 0),
            (VERIFY_VERIFIED, true, 0)
        );
        let low = RectVerdict {
            coverage: 0.1,
            support: 0.0,
            beyond_share: 0.0,
        };
        assert_eq!(
            verify_step(VERIFY_VERIFIED, true, &low, &p, 2),
            (VERIFY_VERIFIED, true, 2)
        );
        let rej = RectVerdict {
            coverage: 0.9,
            support: 0.1,
            beyond_share: 0.0,
        };
        assert_eq!(
            verify_step(VERIFY_UNVERIFIED, false, &rej, &p, 2),
            (VERIFY_REJECTED, false, 3)
        );
    }
}
