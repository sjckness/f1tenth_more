//! The C ABI that `f1tenth_perception/wall_tracker_native.py` loads with ctypes.
//!
//! The only module of the crate allowed to use `unsafe`: raw pointers come in
//! from Python, are checked for null and turned into slices of the length the
//! caller declared, and nothing else is unsafe.
//!
//! CONTRACT (kept in step with wall_tracker_native.py; [`ABI_VERSION`]
//! changes whenever any of it does, and the loader refuses a mismatch):
//! - every struct is `#[repr(C)]` with 8-byte fields only (f64 or i64), so
//!   numpy can mirror it without padding rules; `*_fields(kind)` gives each
//!   struct's field list ("name:f,name:i,..."), from which the binding builds
//!   its numpy dtypes, and `*_struct_size(kind)` its size, which the binding
//!   compares;
//! - the tracker parameters travel as an f64 and an i64 array in the order of
//!   `wt_param_names(kind)`;
//! - a tracker is a handle (`wt_tracker_new` / `wt_tracker_free`) holding the
//!   `MultiWallTracker` state behind a mutex: calls on one handle serialize;
//! - every entry point returns an `i32` status and never unwinds into C: a
//!   panic is caught and reported as [`WT_ERR_PANIC`].
#![allow(unsafe_code)]

use std::collections::HashMap;
use std::ffi::c_char;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::Mutex;

use crate::geometry::{Line, Pose2D, WallLine};
use crate::linalg;
use crate::lsa::{self, LsaError};
use crate::pyc;
use crate::tracker::{
    ExtEntry, Observation, Params, PyDeque, RectVerdict, RectVerifyParams, Role, Scan,
    StripVerdict, Track, Tracker, TrackerError,
};

/// Bumped on any change to a struct layout, a signature or a status code.
pub const ABI_VERSION: u32 = 1;

pub const WT_OK: i32 = 0;
pub const WT_ERR_NULL: i32 = -1;
/// numpy LinAlgError("Singular matrix").
pub const WT_ERR_SINGULAR: i32 = -2;
/// scipy ValueError: the cost matrix has NaN or -inf.
pub const WT_ERR_LSA_INVALID: i32 = -3;
/// scipy ValueError: the cost matrix is infeasible.
pub const WT_ERR_LSA_INFEASIBLE: i32 = -4;
/// A Rust panic, caught at the boundary (a bug: report it).
pub const WT_ERR_PANIC: i32 = -5;
/// An output buffer is too small; the sizes say how big they must be.
pub const WT_ERR_CAPACITY: i32 = -6;
/// The parameter arrays do not have the agreed lengths.
pub const WT_ERR_PARAMS: i32 = -7;
/// An imported state carries a code no Python value maps to.
pub const WT_ERR_STATE: i32 = -8;
/// A length or an offset is out of range.
pub const WT_ERR_LENGTH: i32 = -9;
/// Python's OverflowError (math.floor of an infinity in line_columns).
pub const WT_ERR_OVERFLOW: i32 = -10;

pub const STRUCT_OBSERVATION: i32 = 1;
pub const STRUCT_LINE: i32 = 2;
pub const STRUCT_TRACK: i32 = 3;
pub const STRUCT_SCALARS: i32 = 4;
pub const STRUCT_REJECTION: i32 = 5;
pub const STRUCT_SIZES: i32 = 6;

/// One observation (`wall_fusion.Observation` + its `WallLine`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct WtObservation {
    pub nx: f64,
    pub ny: f64,
    pub d: f64,
    pub t0: f64,
    pub t1: f64,
    pub inliers: i64,
    pub rms: f64,
    pub z_min: f64,
    pub z_max: f64,
    pub height_measured: i64,
    pub source: i64,
    pub confidence: f64,
    pub stereo_weight: f64,
}

const OBSERVATION_FIELDS: &str = "nx:f,ny:f,d:f,t0:f,t1:f,inliers:i,rms:f,z_min:f,z_max:f,\
height_measured:i,source:i,confidence:f,stereo_weight:f\0";

/// A corridor_perception `Line`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct WtLine {
    pub rho: f64,
    pub alpha: f64,
    pub cov00: f64,
    pub cov01: f64,
    pub cov10: f64,
    pub cov11: f64,
    pub sx: f64,
    pub sy: f64,
    pub ex: f64,
    pub ey: f64,
    pub n_points: i64,
}

const LINE_FIELDS: &str =
    "rho:f,alpha:f,cov00:f,cov01:f,cov10:f,cov11:f,sx:f,sy:f,ex:f,ey:f,n_points:i\0";

/// One `Track`: its scalar fields, and where its deques sit in the shared
/// buffers (`*_off`, `*_len`; `*_maxlen` -1 for a deque without maxlen).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct WtTrack {
    pub id: i64,
    pub hits: i64,
    pub state: i64,
    pub source: i64,
    pub verify: i64,
    pub role: i64,
    pub strip_verdict: i64,
    pub rect_reject_streak: i64,
    pub miss: i64,
    pub strip_miss: i64,
    pub ever_confirmed: i64,
    pub has_lidar: i64,
    pub has_both: i64,
    pub height_measured: i64,
    pub glass: i64,
    pub rect_informative: i64,
    pub occluded: i64,
    pub contradicted: i64,
    pub x0: f64,
    pub x1: f64,
    pub p00: f64,
    pub p01: f64,
    pub p10: f64,
    pub p11: f64,
    pub t0: f64,
    pub t1: f64,
    pub born: f64,
    pub last_seen: f64,
    pub z_min: f64,
    pub z_max: f64,
    pub confidence: f64,
    pub rect_coverage: f64,
    pub rect_support: f64,
    pub rect_beyond_share: f64,
    pub odo_at_last_seen: f64,
    pub strip_pred_m: f64,
    pub confirm_maxlen: i64,
    pub confirm_off: i64,
    pub confirm_len: i64,
    pub lidar_maxlen: i64,
    pub lidar_off: i64,
    pub lidar_len: i64,
    pub conf_maxlen: i64,
    pub conf_off: i64,
    pub conf_len: i64,
    pub ext_maxlen: i64,
    pub ext_off: i64,
    pub ext_len: i64,
    /// A 64-bit fingerprint of everything above and the deque contents
    /// (export only): equal hash, unchanged track.
    pub hash: i64,
}

const TRACK_FIELDS: &str = "id:i,hits:i,state:i,source:i,verify:i,role:i,strip_verdict:i,\
rect_reject_streak:i,miss:i,strip_miss:i,ever_confirmed:i,has_lidar:i,has_both:i,\
height_measured:i,glass:i,rect_informative:i,occluded:i,contradicted:i,x0:f,x1:f,p00:f,\
p01:f,p10:f,p11:f,t0:f,t1:f,born:f,last_seen:f,z_min:f,z_max:f,confidence:f,rect_coverage:f,\
rect_support:f,rect_beyond_share:f,odo_at_last_seen:f,strip_pred_m:f,confirm_maxlen:i,\
confirm_off:i,confirm_len:i,lidar_maxlen:i,lidar_off:i,lidar_len:i,conf_maxlen:i,conf_off:i,\
conf_len:i,ext_maxlen:i,ext_off:i,ext_len:i,hash:i\0";

/// The tracker-level state (`MultiWallTracker` attributes).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct WtScalars {
    pub laser_x: f64,
    pub laser_y: f64,
    pub has_laser: i64,
    pub camera_x: f64,
    pub camera_y: f64,
    pub has_camera: i64,
    pub camera_fresh: i64,
    pub next_id: i64,
    pub last_stamp: f64,
    pub has_last_stamp: i64,
    pub last_yaw: f64,
    pub has_last_yaw: i64,
    pub odometer: f64,
    pub last_car_x: f64,
    pub last_car_y: f64,
    pub has_last_car_t: i64,
    pub n_short_dropped: i64,
    pub n_negative_killed: i64,
    pub n_gap_rejected: i64,
    pub n_merge_gap_refused: i64,
    pub n_stereo_near_rejected: i64,
    pub n_camera_rejected: i64,
    pub strip_range_m: f64,
    pub strip_has_range: i64,
    pub strip_n_returns: i64,
    pub strip_n_cut: i64,
    pub strip_n_no_return: i64,
    pub front_id: i64,
    pub front_has_id: i64,
    pub front_reason: i64,
    pub front_distance_m: f64,
    pub n_rejections: i64,
    pub n_front_rejects: i64,
    pub n_tracks: i64,
}

const SCALARS_FIELDS: &str = "laser_x:f,laser_y:f,has_laser:i,camera_x:f,camera_y:f,\
has_camera:i,camera_fresh:i,next_id:i,last_stamp:f,has_last_stamp:i,last_yaw:f,has_last_yaw:i,\
odometer:f,last_car_x:f,last_car_y:f,has_last_car_t:i,n_short_dropped:i,n_negative_killed:i,\
n_gap_rejected:i,n_merge_gap_refused:i,n_stereo_near_rejected:i,n_camera_rejected:i,\
strip_range_m:f,strip_has_range:i,strip_n_returns:i,strip_n_cut:i,strip_n_no_return:i,\
front_id:i,front_has_id:i,front_reason:i,front_distance_m:f,n_rejections:i,n_front_rejects:i,\
n_tracks:i\0";

/// One `rejections` entry; `value` raw (the binding rounds it).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct WtRejection {
    pub code: i64,
    pub has_id: i64,
    pub id: i64,
    pub value: f64,
}

const REJECTION_FIELDS: &str = "code:i,has_id:i,id:i,value:f\0";

/// Export sizes: what the buffers must hold.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct WtSizes {
    pub n_tracks: i64,
    pub n_confirm: i64,
    pub n_lidar: i64,
    pub n_conf: i64,
    pub n_ext: i64,
}

const SIZES_FIELDS: &str = "n_tracks:i,n_confirm:i,n_lidar:i,n_conf:i,n_ext:i\0";

/// The f64 tracker parameters, in array order.
const PARAM_F64_NAMES: &str = "process_noise_rho,process_noise_theta,k_yaw,meas_sigma_rho_min,\
meas_sigma_theta_min,gate_chi2,collinear_gap_m,extent_decay_s,confirm_min_length_m,\
confirm_max_dist_jitter_m,confirm_max_angle_jitter_rad,delete_after_s,max_sigma_rho_m,\
label_half_sector_rad,role_hysteresis_rad,ray_margin_m,front_max_coast_s,front_max_coast_m,\
lidar_sigma_base_m,lidar_sigma_per_m,stereo_sigma_base_m,stereo_sigma_quad_per_m2,\
min_span_for_angle_m,min_span_birth_m,min_span_update_m,front_x_m,side_edge_half_width_m,\
front_strip_margin_m,stereo_front_max_normal_rad,front_min_confidence,front_contradict_margin_m,\
stereo_min_rho_m,camera_seg_min_ray_angle_rad,camera_seg_min_bearing_span_rad,stereo_max_rms_m,\
stereo_min_vertical_extent_m,stereo_min_length_m,back_x_m,side_x_min_m,side_x_max_m,\
side_max_normal_rad,camera_side_x_max_m,camera_side_max_normal_rad,camera_side_offset_min_m,\
camera_side_offset_max_m,role_hysteresis_m,occlusion_margin_m,negative_tol_m,\
negative_pass_frac,coast_max_odom_m\0";
const N_PARAM_F64: usize = 50;

/// The integer (and bool) tracker parameters, in array order.
const PARAM_I64_NAMES: &str = "confirm_min_consecutive_hits,confirm_require_lidar,max_published,\
front_lidar_recent_cycles,front_contradict_cycles,occlusion_points,occlusion_min_blocked,\
negative_shrink_beams,negative_min_beams,negative_miss_to_kill\0";
const N_PARAM_I64: usize = 10;

/// The rectangle verify parameters (wall_fusion.RectParams' decision part).
const RECT_F64_NAMES: &str =
    "coverage_min,support_verify,support_reject,glass_beyond_share,glass_at_share\0";
const N_RECT_F64: usize = 5;
const RECT_I64_NAMES: &str = "reject_cycles\0";
const N_RECT_I64: usize = 1;

static VERSION: &[u8] = concat!(env!("CARGO_PKG_VERSION"), "\0").as_bytes();

/// A tracker behind a mutex.
pub struct WtHandle {
    inner: Mutex<Tracker>,
}

fn guarded(f: impl FnOnce() -> i32) -> i32 {
    catch_unwind(AssertUnwindSafe(f)).unwrap_or(WT_ERR_PANIC)
}

fn status_of(e: TrackerError) -> i32 {
    match e {
        TrackerError::Singular => WT_ERR_SINGULAR,
        TrackerError::Lsa(LsaError::Invalid) => WT_ERR_LSA_INVALID,
        TrackerError::Lsa(LsaError::Infeasible) => WT_ERR_LSA_INFEASIBLE,
    }
}

/// `&[T]` from a pointer and a length; None for null with n > 0.
unsafe fn slice<'a, T>(p: *const T, n: i64) -> Option<&'a [T]> {
    let n = usize::try_from(n).ok()?;
    if n == 0 {
        return Some(&[]);
    }
    if p.is_null() {
        return None;
    }
    Some(std::slice::from_raw_parts(p, n))
}

unsafe fn slice_mut<'a, T>(p: *mut T, n: i64) -> Option<&'a mut [T]> {
    let n = usize::try_from(n).ok()?;
    if n == 0 {
        return Some(&mut []);
    }
    if p.is_null() {
        return None;
    }
    Some(std::slice::from_raw_parts_mut(p, n))
}

unsafe fn handle<'a>(h: *mut WtHandle) -> Option<&'a WtHandle> {
    h.as_ref()
}

fn lock(h: &WtHandle) -> std::sync::MutexGuard<'_, Tracker> {
    match h.inner.lock() {
        Ok(g) => g,
        Err(poisoned) => poisoned.into_inner(),
    }
}

/// [`ABI_VERSION`].
#[no_mangle]
pub extern "C" fn f1tenth_wall_tracker_core_abi_version() -> u32 {
    ABI_VERSION
}

/// `size_of` the struct `kind` (`STRUCT_*`), -1 for an unknown kind.
#[no_mangle]
pub extern "C" fn f1tenth_wall_tracker_core_struct_size(kind: i32) -> i64 {
    let size = match kind {
        STRUCT_OBSERVATION => std::mem::size_of::<WtObservation>(),
        STRUCT_LINE => std::mem::size_of::<WtLine>(),
        STRUCT_TRACK => std::mem::size_of::<WtTrack>(),
        STRUCT_SCALARS => std::mem::size_of::<WtScalars>(),
        STRUCT_REJECTION => std::mem::size_of::<WtRejection>(),
        STRUCT_SIZES => std::mem::size_of::<WtSizes>(),
        STRUCT_FUSION_IN => std::mem::size_of::<WtFusionIn>(),
        STRUCT_FUSION_OBS => std::mem::size_of::<WtFusionObs>(),
        STRUCT_RECT_OUT => std::mem::size_of::<WtRectOut>(),
        _ => return -1,
    };
    i64::try_from(size).unwrap_or(-1)
}

/// The field list of struct `kind`, "name:f,name:i,...", NUL-terminated
/// static; null for an unknown kind.
#[no_mangle]
pub extern "C" fn f1tenth_wall_tracker_core_fields(kind: i32) -> *const c_char {
    let s = match kind {
        STRUCT_OBSERVATION => OBSERVATION_FIELDS,
        STRUCT_LINE => LINE_FIELDS,
        STRUCT_TRACK => TRACK_FIELDS,
        STRUCT_SCALARS => SCALARS_FIELDS,
        STRUCT_REJECTION => REJECTION_FIELDS,
        STRUCT_SIZES => SIZES_FIELDS,
        STRUCT_FUSION_IN => FUSION_IN_FIELDS,
        STRUCT_FUSION_OBS => FUSION_OBS_FIELDS,
        STRUCT_RECT_OUT => RECT_OUT_FIELDS,
        _ => return std::ptr::null(),
    };
    s.as_ptr().cast()
}

/// The parameter names of array `kind` (0 tracker f64, 1 tracker i64, 2 rect
/// f64, 3 rect i64), comma-separated, NUL-terminated static.
#[no_mangle]
pub extern "C" fn wt_param_names(kind: i32) -> *const c_char {
    let s = match kind {
        0 => PARAM_F64_NAMES,
        1 => PARAM_I64_NAMES,
        2 => RECT_F64_NAMES,
        3 => RECT_I64_NAMES,
        _ => return std::ptr::null(),
    };
    s.as_ptr().cast()
}

/// The crate version, a static NUL-terminated string.
#[no_mangle]
pub extern "C" fn f1tenth_wall_tracker_core_version() -> *const c_char {
    VERSION.as_ptr().cast()
}

/// Writes the build summary into `buf` (NUL-terminated, truncated to
/// `cap - 1`). Returns the full length without the NUL, or WT_ERR_NULL.
///
/// # Safety
/// `buf` must point to `cap` writable bytes.
#[no_mangle]
pub unsafe extern "C" fn f1tenth_wall_tracker_core_build_info(buf: *mut c_char, cap: i64) -> i64 {
    let s = crate::build_info::summary();
    let bytes = s.as_bytes();
    let Ok(cap) = usize::try_from(cap) else {
        return i64::from(WT_ERR_NULL);
    };
    if buf.is_null() || cap == 0 {
        return i64::from(WT_ERR_NULL);
    }
    let n = bytes.len().min(cap - 1);
    std::ptr::copy_nonoverlapping(bytes.as_ptr().cast::<c_char>(), buf, n);
    *buf.add(n) = 0;
    i64::try_from(bytes.len()).unwrap_or(i64::MAX)
}

fn params_from(pf: &[f64], pi: &[i64]) -> Option<Params> {
    if pf.len() != N_PARAM_F64 || pi.len() != N_PARAM_I64 {
        return None;
    }
    Some(Params {
        process_noise_rho: pf[0],
        process_noise_theta: pf[1],
        k_yaw: pf[2],
        meas_sigma_rho_min: pf[3],
        meas_sigma_theta_min: pf[4],
        gate_chi2: pf[5],
        collinear_gap_m: pf[6],
        extent_decay_s: pf[7],
        confirm_min_length_m: pf[8],
        confirm_max_dist_jitter_m: pf[9],
        confirm_max_angle_jitter_rad: pf[10],
        delete_after_s: pf[11],
        max_sigma_rho_m: pf[12],
        label_half_sector_rad: pf[13],
        role_hysteresis_rad: pf[14],
        ray_margin_m: pf[15],
        front_max_coast_s: pf[16],
        front_max_coast_m: pf[17],
        lidar_sigma_base_m: pf[18],
        lidar_sigma_per_m: pf[19],
        stereo_sigma_base_m: pf[20],
        stereo_sigma_quad_per_m2: pf[21],
        min_span_for_angle_m: pf[22],
        min_span_birth_m: pf[23],
        min_span_update_m: pf[24],
        front_x_m: pf[25],
        side_edge_half_width_m: pf[26],
        front_strip_margin_m: pf[27],
        stereo_front_max_normal_rad: pf[28],
        front_min_confidence: pf[29],
        front_contradict_margin_m: pf[30],
        stereo_min_rho_m: pf[31],
        camera_seg_min_ray_angle_rad: pf[32],
        camera_seg_min_bearing_span_rad: pf[33],
        stereo_max_rms_m: pf[34],
        stereo_min_vertical_extent_m: pf[35],
        stereo_min_length_m: pf[36],
        back_x_m: pf[37],
        side_x_min_m: pf[38],
        side_x_max_m: pf[39],
        side_max_normal_rad: pf[40],
        camera_side_x_max_m: pf[41],
        camera_side_max_normal_rad: pf[42],
        camera_side_offset_min_m: pf[43],
        camera_side_offset_max_m: pf[44],
        role_hysteresis_m: pf[45],
        occlusion_margin_m: pf[46],
        negative_tol_m: pf[47],
        negative_pass_frac: pf[48],
        coast_max_odom_m: pf[49],
        confirm_min_consecutive_hits: pi[0],
        confirm_require_lidar: pi[1] != 0,
        max_published: pi[2],
        front_lidar_recent_cycles: pi[3],
        front_contradict_cycles: pi[4],
        occlusion_points: pi[5],
        occlusion_min_blocked: pi[6],
        negative_shrink_beams: pi[7],
        negative_min_beams: pi[8],
        negative_miss_to_kill: pi[9],
    })
}

fn rect_from(rf: &[f64], ri: &[i64]) -> Option<RectVerifyParams> {
    if rf.len() != N_RECT_F64 || ri.len() != N_RECT_I64 {
        return None;
    }
    Some(RectVerifyParams {
        coverage_min: rf[0],
        support_verify: rf[1],
        support_reject: rf[2],
        glass_beyond_share: rf[3],
        glass_at_share: rf[4],
        reject_cycles: ri[0],
    })
}

/// A new tracker; null on a parameter-length mismatch (`*status` says why).
///
/// # Safety
/// Each pointer must point to its declared number of elements; `status` to
/// one writable i32 (or be null).
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn wt_tracker_new(
    pf: *const f64,
    npf: i64,
    pi: *const i64,
    npi: i64,
    rf: *const f64,
    nrf: i64,
    ri: *const i64,
    nri: i64,
    status: *mut i32,
) -> *mut WtHandle {
    let mut out: *mut WtHandle = std::ptr::null_mut();
    let st = guarded(|| {
        let (Some(pf), Some(pi), Some(rf), Some(ri)) = (
            slice(pf, npf),
            slice(pi, npi),
            slice(rf, nrf),
            slice(ri, nri),
        ) else {
            return WT_ERR_NULL;
        };
        let (Some(p), Some(r)) = (params_from(pf, pi), rect_from(rf, ri)) else {
            return WT_ERR_PARAMS;
        };
        out = Box::into_raw(Box::new(WtHandle {
            inner: Mutex::new(Tracker::new(p, r)),
        }));
        WT_OK
    });
    if !status.is_null() {
        *status = st;
    }
    out
}

/// Frees a tracker (null is a no-op).
///
/// # Safety
/// `h` must come from `wt_tracker_new` and not be used afterwards.
#[no_mangle]
pub unsafe extern "C" fn wt_tracker_free(h: *mut WtHandle) {
    if !h.is_null() {
        drop(Box::from_raw(h));
    }
}

/// Replaces the parameters.
///
/// # Safety
/// As for `wt_tracker_new`; `h` a live handle.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn wt_tracker_set_params(
    h: *mut WtHandle,
    pf: *const f64,
    npf: i64,
    pi: *const i64,
    npi: i64,
    rf: *const f64,
    nrf: i64,
    ri: *const i64,
    nri: i64,
) -> i32 {
    guarded(|| {
        let Some(h) = handle(h) else {
            return WT_ERR_NULL;
        };
        let (Some(pf), Some(pi), Some(rf), Some(ri)) = (
            slice(pf, npf),
            slice(pi, npi),
            slice(rf, nrf),
            slice(ri, nri),
        ) else {
            return WT_ERR_NULL;
        };
        let (Some(p), Some(r)) = (params_from(pf, pi), rect_from(rf, ri)) else {
            return WT_ERR_PARAMS;
        };
        let mut t = lock(h);
        t.p = p;
        t.rect = r;
        WT_OK
    })
}

fn opt(has: i64, v: f64) -> Option<f64> {
    (has != 0).then_some(v)
}

fn opt2(has: i64, x: f64, y: f64) -> Option<[f64; 2]> {
    (has != 0).then_some([x, y])
}

fn scalars_of(t: &Tracker) -> WtScalars {
    let (lx, ly) = t.laser_xy.map_or((0.0, 0.0), |v| (v[0], v[1]));
    let (cx, cy) = t.camera_xy.map_or((0.0, 0.0), |v| (v[0], v[1]));
    let (vx, vy) = t.last_car_t.map_or((0.0, 0.0), |v| (v[0], v[1]));
    WtScalars {
        laser_x: lx,
        laser_y: ly,
        has_laser: i64::from(t.laser_xy.is_some()),
        camera_x: cx,
        camera_y: cy,
        has_camera: i64::from(t.camera_xy.is_some()),
        camera_fresh: i64::from(t.camera_fresh),
        next_id: t.next_id,
        last_stamp: t.last_stamp.unwrap_or(0.0),
        has_last_stamp: i64::from(t.last_stamp.is_some()),
        last_yaw: t.last_yaw.unwrap_or(0.0),
        has_last_yaw: i64::from(t.last_yaw.is_some()),
        odometer: t.odometer,
        last_car_x: vx,
        last_car_y: vy,
        has_last_car_t: i64::from(t.last_car_t.is_some()),
        n_short_dropped: t.n_short_dropped,
        n_negative_killed: t.n_negative_killed,
        n_gap_rejected: t.n_gap_rejected,
        n_merge_gap_refused: t.n_merge_gap_refused,
        n_stereo_near_rejected: t.n_stereo_near_rejected,
        n_camera_rejected: t.n_camera_rejected,
        strip_range_m: t.strip.range_m.unwrap_or(0.0),
        strip_has_range: i64::from(t.strip.range_m.is_some()),
        strip_n_returns: t.strip.n_returns,
        strip_n_cut: t.strip.n_cut,
        strip_n_no_return: t.strip.n_no_return,
        front_id: t.front_status.id.unwrap_or(0),
        front_has_id: i64::from(t.front_status.id.is_some()),
        front_reason: t.front_status.reason as i64,
        front_distance_m: t.front_status.distance_m,
        n_rejections: t.rejections.len() as i64,
        n_front_rejects: t.front_status.rejects.len() as i64,
        n_tracks: t.tracks.len() as i64,
    }
}

/// The tracker-level state.
///
/// # Safety
/// `h` a live handle, `out` one writable WtScalars.
#[no_mangle]
pub unsafe extern "C" fn wt_tracker_get_scalars(h: *mut WtHandle, out: *mut WtScalars) -> i32 {
    guarded(|| {
        let (Some(h), Some(out)) = (handle(h), out.as_mut()) else {
            return WT_ERR_NULL;
        };
        *out = scalars_of(&lock(h));
        WT_OK
    })
}

/// Sets the tracker-level INPUT state (laser / camera position, camera_fresh,
/// the id counter, the last stamp / yaw / car position, the odometer) and the
/// counters. The strip, front_status and rejections are step outputs and are
/// not set.
///
/// # Safety
/// `h` a live handle, `s` one readable WtScalars.
#[no_mangle]
pub unsafe extern "C" fn wt_tracker_set_scalars(h: *mut WtHandle, s: *const WtScalars) -> i32 {
    guarded(|| {
        let (Some(h), Some(s)) = (handle(h), s.as_ref()) else {
            return WT_ERR_NULL;
        };
        let mut t = lock(h);
        t.laser_xy = opt2(s.has_laser, s.laser_x, s.laser_y);
        t.camera_xy = opt2(s.has_camera, s.camera_x, s.camera_y);
        t.camera_fresh = s.camera_fresh != 0;
        t.next_id = s.next_id;
        t.last_stamp = opt(s.has_last_stamp, s.last_stamp);
        t.last_yaw = opt(s.has_last_yaw, s.last_yaw);
        t.odometer = s.odometer;
        t.last_car_t = opt2(s.has_last_car_t, s.last_car_x, s.last_car_y);
        t.n_short_dropped = s.n_short_dropped;
        t.n_negative_killed = s.n_negative_killed;
        t.n_gap_rejected = s.n_gap_rejected;
        t.n_merge_gap_refused = s.n_merge_gap_refused;
        t.n_stereo_near_rejected = s.n_stereo_near_rejected;
        t.n_camera_rejected = s.n_camera_rejected;
        WT_OK
    })
}

/// FNV-1a over 64-bit words.
struct Fnv(u64);

impl Fnv {
    fn new() -> Self {
        Fnv(0xcbf2_9ce4_8422_2325)
    }
    fn word(&mut self, w: u64) {
        for b in w.to_le_bytes() {
            self.0 ^= u64::from(b);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    fn f(&mut self, v: f64) {
        self.word(v.to_bits());
    }
    fn i(&mut self, v: i64) {
        self.word(v as u64);
    }
}

fn maxlen_code(m: Option<usize>) -> i64 {
    m.map_or(-1, |v| v as i64)
}

fn track_scalars(t: &Track) -> WtTrack {
    WtTrack {
        id: t.id,
        hits: t.hits,
        state: i64::from(t.state),
        source: i64::from(t.source),
        verify: i64::from(t.verify),
        role: t.role as i64,
        strip_verdict: t.strip_verdict as i64,
        rect_reject_streak: t.rect_reject_streak,
        miss: t.miss,
        strip_miss: t.strip_miss,
        ever_confirmed: i64::from(t.ever_confirmed),
        has_lidar: i64::from(t.has_lidar),
        has_both: i64::from(t.has_both),
        height_measured: i64::from(t.height_measured),
        glass: i64::from(t.glass),
        rect_informative: i64::from(t.rect_informative),
        occluded: i64::from(t.occluded),
        contradicted: i64::from(t.contradicted),
        x0: t.x[0],
        x1: t.x[1],
        p00: t.p[0][0],
        p01: t.p[0][1],
        p10: t.p[1][0],
        p11: t.p[1][1],
        t0: t.t0,
        t1: t.t1,
        born: t.born,
        last_seen: t.last_seen,
        z_min: t.z_min,
        z_max: t.z_max,
        confidence: t.confidence,
        rect_coverage: t.rect_coverage,
        rect_support: t.rect_support,
        rect_beyond_share: t.rect_beyond_share,
        odo_at_last_seen: t.odo_at_last_seen,
        strip_pred_m: t.strip_pred_m,
        confirm_maxlen: maxlen_code(t.confirm_obs.maxlen),
        confirm_off: 0,
        confirm_len: t.confirm_obs.len() as i64,
        lidar_maxlen: maxlen_code(t.lidar_hist.maxlen),
        lidar_off: 0,
        lidar_len: t.lidar_hist.len() as i64,
        conf_maxlen: maxlen_code(t.conf_hist.maxlen),
        conf_off: 0,
        conf_len: t.conf_hist.len() as i64,
        ext_maxlen: maxlen_code(t.obs_ext.maxlen),
        ext_off: 0,
        ext_len: t.obs_ext.len() as i64,
        hash: 0,
    }
}

fn hash_track(w: &WtTrack, t: &Track) -> i64 {
    let mut h = Fnv::new();
    for v in [
        w.id,
        w.hits,
        w.state,
        w.source,
        w.verify,
        w.role,
        w.strip_verdict,
        w.rect_reject_streak,
        w.miss,
        w.strip_miss,
        w.ever_confirmed,
        w.has_lidar,
        w.has_both,
        w.height_measured,
        w.glass,
        w.rect_informative,
        w.occluded,
        w.contradicted,
        w.confirm_maxlen,
        w.lidar_maxlen,
        w.conf_maxlen,
        w.ext_maxlen,
    ] {
        h.i(v);
    }
    for v in [
        w.x0,
        w.x1,
        w.p00,
        w.p01,
        w.p10,
        w.p11,
        w.t0,
        w.t1,
        w.born,
        w.last_seen,
        w.z_min,
        w.z_max,
        w.confidence,
        w.rect_coverage,
        w.rect_support,
        w.rect_beyond_share,
        w.odo_at_last_seen,
        w.strip_pred_m,
    ] {
        h.f(v);
    }
    h.i(t.confirm_obs.len() as i64);
    for c in &t.confirm_obs.items {
        match c {
            Some((r, a)) => {
                h.i(1);
                h.f(*r);
                h.f(*a);
            }
            None => h.i(0),
        }
    }
    h.i(t.lidar_hist.len() as i64);
    for &b in &t.lidar_hist.items {
        h.i(i64::from(b));
    }
    h.i(t.conf_hist.len() as i64);
    for &c in &t.conf_hist.items {
        h.f(c);
    }
    h.i(t.obs_ext.len() as i64);
    for e in &t.obs_ext.items {
        h.word(e.serial);
        h.f(e.stamp);
        for p in e.pts {
            h.f(p[0]);
            h.f(p[1]);
        }
    }
    h.0 as i64
}

/// Exports every track: scalars into `tracks`, the deques into the shared
/// buffers (confirm rows `[valid, rho, theta]`, lidar 0/1, conf, ext rows
/// `[stamp, x0, y0, x1, y1]` plus `ext_serial`). `sizes` always receives the
/// sizes needed; WT_ERR_CAPACITY (nothing written) when a buffer is short.
///
/// # Safety
/// Each buffer must hold its declared capacity (rows of 3 / 5 f64 for confirm
/// / ext); `sizes` one writable WtSizes.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn wt_tracker_export_tracks(
    h: *mut WtHandle,
    tracks: *mut WtTrack,
    cap_tracks: i64,
    confirm: *mut f64,
    cap_confirm: i64,
    lidar: *mut i64,
    cap_lidar: i64,
    conf: *mut f64,
    cap_conf: i64,
    ext: *mut f64,
    ext_serial: *mut i64,
    cap_ext: i64,
    sizes: *mut WtSizes,
) -> i32 {
    guarded(|| {
        let (Some(h), Some(sizes)) = (handle(h), sizes.as_mut()) else {
            return WT_ERR_NULL;
        };
        let t = lock(h);
        let need = WtSizes {
            n_tracks: t.tracks.len() as i64,
            n_confirm: t.tracks.iter().map(|k| k.confirm_obs.len() as i64).sum(),
            n_lidar: t.tracks.iter().map(|k| k.lidar_hist.len() as i64).sum(),
            n_conf: t.tracks.iter().map(|k| k.conf_hist.len() as i64).sum(),
            n_ext: t.tracks.iter().map(|k| k.obs_ext.len() as i64).sum(),
        };
        *sizes = need;
        if need.n_tracks > cap_tracks
            || need.n_confirm > cap_confirm
            || need.n_lidar > cap_lidar
            || need.n_conf > cap_conf
            || need.n_ext > cap_ext
        {
            return WT_ERR_CAPACITY;
        }
        let (Some(ts), Some(cs), Some(ls), Some(fs), Some(es), Some(ss)) = (
            slice_mut(tracks, need.n_tracks),
            slice_mut(confirm, need.n_confirm * 3),
            slice_mut(lidar, need.n_lidar),
            slice_mut(conf, need.n_conf),
            slice_mut(ext, need.n_ext * 5),
            slice_mut(ext_serial, need.n_ext),
        ) else {
            return WT_ERR_NULL;
        };
        let (mut oc, mut ol, mut of, mut oe) = (0_usize, 0_usize, 0_usize, 0_usize);
        for (k, trk) in t.tracks.iter().enumerate() {
            let mut w = track_scalars(trk);
            w.confirm_off = oc as i64;
            for c in &trk.confirm_obs.items {
                let (v, r, a) = match c {
                    Some((r, a)) => (1.0, *r, *a),
                    None => (0.0, 0.0, 0.0),
                };
                cs[3 * oc] = v;
                cs[3 * oc + 1] = r;
                cs[3 * oc + 2] = a;
                oc += 1;
            }
            w.lidar_off = ol as i64;
            for &b in &trk.lidar_hist.items {
                ls[ol] = i64::from(b);
                ol += 1;
            }
            w.conf_off = of as i64;
            for &c in &trk.conf_hist.items {
                fs[of] = c;
                of += 1;
            }
            w.ext_off = oe as i64;
            for e in &trk.obs_ext.items {
                es[5 * oe] = e.stamp;
                es[5 * oe + 1] = e.pts[0][0];
                es[5 * oe + 2] = e.pts[0][1];
                es[5 * oe + 3] = e.pts[1][0];
                es[5 * oe + 4] = e.pts[1][1];
                ss[oe] = e.serial as i64;
                oe += 1;
            }
            w.hash = hash_track(&w, trk);
            ts[k] = w;
        }
        WT_OK
    })
}

fn maxlen_of(code: i64) -> Option<usize> {
    usize::try_from(code).ok()
}

fn range(off: i64, len: i64, total: usize) -> Option<std::ops::Range<usize>> {
    let off = usize::try_from(off).ok()?;
    let len = usize::try_from(len).ok()?;
    let end = off.checked_add(len)?;
    (end <= total).then_some(off..end)
}

/// Replaces every track with the given state (the binding's push of a state
/// Python changed). An ext serial <= 0 gets a fresh one.
///
/// # Safety
/// Each buffer must hold its declared number of rows.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn wt_tracker_import_tracks(
    h: *mut WtHandle,
    n_tracks: i64,
    tracks: *const WtTrack,
    confirm: *const f64,
    n_confirm: i64,
    lidar: *const i64,
    n_lidar: i64,
    conf: *const f64,
    n_conf: i64,
    ext: *const f64,
    ext_serial: *const i64,
    n_ext: i64,
) -> i32 {
    guarded(|| {
        let Some(h) = handle(h) else {
            return WT_ERR_NULL;
        };
        let (Some(ts), Some(cs), Some(ls), Some(fs), Some(es), Some(ss)) = (
            slice(tracks, n_tracks),
            slice(confirm, n_confirm.saturating_mul(3)),
            slice(lidar, n_lidar),
            slice(conf, n_conf),
            slice(ext, n_ext.saturating_mul(5)),
            slice(ext_serial, n_ext),
        ) else {
            return WT_ERR_NULL;
        };
        let mut t = lock(h);
        let mut out = Vec::with_capacity(ts.len());
        for w in ts {
            let (Some(role), Some(sv)) = (
                Role::from_code(w.role as i32),
                StripVerdict::from_code(w.strip_verdict as i32),
            ) else {
                return WT_ERR_STATE;
            };
            let (Some(rc), Some(rl), Some(rf), Some(re)) = (
                range(w.confirm_off, w.confirm_len, cs.len() / 3),
                range(w.lidar_off, w.lidar_len, ls.len()),
                range(w.conf_off, w.conf_len, fs.len()),
                range(w.ext_off, w.ext_len, ss.len()),
            ) else {
                return WT_ERR_LENGTH;
            };
            let mut trk = Track::new(
                w.id,
                [w.x0, w.x1],
                [[w.p00, w.p01], [w.p10, w.p11]],
                w.t0,
                w.t1,
                w.born,
                w.last_seen,
            );
            trk.hits = w.hits;
            trk.state = w.state as i32;
            trk.source = w.source as i32;
            trk.verify = w.verify as i32;
            trk.role = role;
            trk.strip_verdict = sv;
            trk.rect_reject_streak = w.rect_reject_streak;
            trk.miss = w.miss;
            trk.strip_miss = w.strip_miss;
            trk.ever_confirmed = w.ever_confirmed != 0;
            trk.has_lidar = w.has_lidar != 0;
            trk.has_both = w.has_both != 0;
            trk.height_measured = w.height_measured != 0;
            trk.glass = w.glass != 0;
            trk.rect_informative = w.rect_informative != 0;
            trk.occluded = w.occluded != 0;
            trk.contradicted = w.contradicted != 0;
            trk.z_min = w.z_min;
            trk.z_max = w.z_max;
            trk.confidence = w.confidence;
            trk.rect_coverage = w.rect_coverage;
            trk.rect_support = w.rect_support;
            trk.rect_beyond_share = w.rect_beyond_share;
            trk.odo_at_last_seen = w.odo_at_last_seen;
            trk.strip_pred_m = w.strip_pred_m;
            // Python's deque(iterable, maxlen) keeps the last maxlen items; a
            // mirror never holds more than its maxlen, but a hand-made one may.
            trk.confirm_obs = PyDeque::from_vec(
                rc.map(|i| (cs[3 * i] != 0.0).then_some((cs[3 * i + 1], cs[3 * i + 2])))
                    .collect(),
                maxlen_of(w.confirm_maxlen),
            );
            trk.lidar_hist =
                PyDeque::from_vec(rl.map(|i| ls[i] != 0).collect(), maxlen_of(w.lidar_maxlen));
            trk.conf_hist =
                PyDeque::from_vec(rf.map(|i| fs[i]).collect(), maxlen_of(w.conf_maxlen));
            let mut entries = Vec::with_capacity(re.len());
            for i in re {
                let serial = if ss[i] > 0 {
                    ss[i] as u64
                } else {
                    let s = t.next_serial;
                    t.next_serial += 1;
                    s
                };
                t.next_serial = t.next_serial.max(serial + 1);
                entries.push(ExtEntry {
                    stamp: es[5 * i],
                    pts: [
                        [es[5 * i + 1], es[5 * i + 2]],
                        [es[5 * i + 3], es[5 * i + 4]],
                    ],
                    serial,
                });
            }
            trk.obs_ext = PyDeque::from_vec(entries, maxlen_of(w.ext_maxlen));
            out.push(trk);
        }
        t.tracks = out;
        WT_OK
    })
}

fn observation_of(o: &WtObservation) -> Observation {
    Observation {
        line: WallLine {
            nx: o.nx,
            ny: o.ny,
            d: o.d,
            t0: o.t0,
            t1: o.t1,
            inliers: o.inliers,
            rms: o.rms,
        },
        z_min: o.z_min,
        z_max: o.z_max,
        height_measured: o.height_measured != 0,
        source: o.source as i32,
        confidence: o.confidence,
        stereo_weight: o.stereo_weight,
    }
}

fn write_ids(t: &Tracker, idx: &[usize], out: *mut i64, cap: i64, n_out: *mut i64) -> i32 {
    // SAFETY: the caller's contract (cap writable i64s at out, one at n_out).
    unsafe {
        if !n_out.is_null() {
            *n_out = idx.len() as i64;
        }
        if (idx.len() as i64) > cap {
            return WT_ERR_CAPACITY;
        }
        let Some(o) = slice_mut(out, idx.len() as i64) else {
            return WT_ERR_NULL;
        };
        for (slot, &k) in o.iter_mut().zip(idx) {
            *slot = t.tracks[k].id;
        }
    }
    WT_OK
}

/// One tracker step. `has_scan` 0: no scan (the arrays are ignored); `cut`
/// may be null (a 3-tuple scan). The published track ids go to `out_ids`
/// (none with `defer_roles`).
///
/// # Safety
/// `obs` must hold `n_obs` structs; `ranges` / `bearings` (and `cut` if not
/// null) `n_beams` elements; `out_ids` `cap_ids` i64s; `n_ids` one i64.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn wt_tracker_step(
    h: *mut WtHandle,
    obs: *const WtObservation,
    n_obs: i64,
    stamp: f64,
    car_x: f64,
    car_y: f64,
    car_theta: f64,
    has_scan: i32,
    ranges: *const f64,
    bearings: *const f64,
    n_beams: i64,
    laser_x: f64,
    laser_y: f64,
    cut: *const u8,
    defer_roles: i32,
    out_ids: *mut i64,
    cap_ids: i64,
    n_ids: *mut i64,
) -> i32 {
    guarded(|| {
        let (Some(h), Some(obs)) = (handle(h), slice(obs, n_obs)) else {
            return WT_ERR_NULL;
        };
        let scan = if has_scan != 0 {
            let (Some(r), Some(b)) = (slice(ranges, n_beams), slice(bearings, n_beams)) else {
                return WT_ERR_NULL;
            };
            let c = if cut.is_null() {
                None
            } else {
                let Some(c) = slice(cut, n_beams) else {
                    return WT_ERR_NULL;
                };
                Some(c.iter().map(|&v| v != 0).collect())
            };
            Some(Scan {
                ranges: r.to_vec(),
                bearings: b.to_vec(),
                laser_xy: [laser_x, laser_y],
                cut: c,
            })
        } else {
            None
        };
        let observations: Vec<Observation> = obs.iter().map(observation_of).collect();
        let car = Pose2D::new(car_x, car_y, car_theta);
        let mut t = lock(h);
        match t.step(&observations, stamp, &car, scan, defer_roles != 0) {
            Ok(idx) => write_ids(&t, &idx, out_ids, cap_ids, n_ids),
            Err(e) => status_of(e),
        }
    })
}

/// `update_roles(car_pose)`.
///
/// # Safety
/// `h` a live handle.
#[no_mangle]
pub unsafe extern "C" fn wt_tracker_update_roles(
    h: *mut WtHandle,
    x: f64,
    y: f64,
    theta: f64,
) -> i32 {
    guarded(|| {
        let Some(h) = handle(h) else {
            return WT_ERR_NULL;
        };
        lock(h).update_roles(&Pose2D::new(x, y, theta));
        WT_OK
    })
}

/// `published(car_pose)`: the ids, in order.
///
/// # Safety
/// `out_ids` `cap_ids` writable i64s, `n_ids` one.
#[no_mangle]
pub unsafe extern "C" fn wt_tracker_published(
    h: *mut WtHandle,
    x: f64,
    y: f64,
    theta: f64,
    out_ids: *mut i64,
    cap_ids: i64,
    n_ids: *mut i64,
) -> i32 {
    guarded(|| {
        let Some(h) = handle(h) else {
            return WT_ERR_NULL;
        };
        let t = lock(h);
        let idx = t.published(&Pose2D::new(x, y, theta));
        write_ids(&t, &idx, out_ids, cap_ids, n_ids)
    })
}

/// `front_track(car_pose, front_x)`: `*has_front`/`*front_id`, and
/// front_status' rejects as `(id, code)` pairs (the scalars carry the rest).
///
/// # Safety
/// `front_id`, `has_front`, `n_rejects` one writable i64 each; `rejects`
/// `2 * cap_rejects` writable i64s.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn wt_tracker_front_track(
    h: *mut WtHandle,
    x: f64,
    y: f64,
    theta: f64,
    front_x: f64,
    front_id: *mut i64,
    has_front: *mut i64,
    rejects: *mut i64,
    cap_rejects: i64,
    n_rejects: *mut i64,
) -> i32 {
    guarded(|| {
        let (Some(h), Some(fid), Some(has), Some(nr)) = (
            handle(h),
            front_id.as_mut(),
            has_front.as_mut(),
            n_rejects.as_mut(),
        ) else {
            return WT_ERR_NULL;
        };
        let mut t = lock(h);
        let best = t.front_track(&Pose2D::new(x, y, theta), front_x);
        *has = i64::from(best.is_some());
        *fid = best.map_or(0, |k| t.tracks[k].id);
        let rj = &t.front_status.rejects;
        *nr = rj.len() as i64;
        if (rj.len() as i64) > cap_rejects {
            return WT_ERR_CAPACITY;
        }
        let Some(out) = slice_mut(rejects, 2 * rj.len() as i64) else {
            return WT_ERR_NULL;
        };
        for (k, (id, code)) in rj.iter().enumerate() {
            out[2 * k] = *id;
            out[2 * k + 1] = *code as i64;
        }
        WT_OK
    })
}

/// `apply_rect_verdicts({id: verdict})`: `values` rows
/// `[coverage, support, beyond_share]`.
///
/// # Safety
/// `ids` `n` i64s, `values` `3 n` f64s.
#[no_mangle]
pub unsafe extern "C" fn wt_tracker_apply_rect_verdicts(
    h: *mut WtHandle,
    n: i64,
    ids: *const i64,
    values: *const f64,
) -> i32 {
    guarded(|| {
        let (Some(h), Some(ids), Some(v)) =
            (handle(h), slice(ids, n), slice(values, n.saturating_mul(3)))
        else {
            return WT_ERR_NULL;
        };
        let mut map = HashMap::with_capacity(ids.len());
        for (k, &id) in ids.iter().enumerate() {
            map.insert(
                id,
                RectVerdict {
                    coverage: v[3 * k],
                    support: v[3 * k + 1],
                    beyond_share: v[3 * k + 2],
                },
            );
        }
        lock(h).apply_rect_verdicts(&map);
        WT_OK
    })
}

fn line_of(l: &WtLine) -> Line {
    Line {
        rho: l.rho,
        alpha: l.alpha,
        cov: [[l.cov00, l.cov01], [l.cov10, l.cov11]],
        p_start: [l.sx, l.sy],
        p_end: [l.ex, l.ey],
        n_points: l.n_points,
    }
}

/// `_spawn(line, stamp, obs)`; `obs` may be null.
///
/// # Safety
/// `line` one readable WtLine, `obs` null or one readable WtObservation.
#[no_mangle]
pub unsafe extern "C" fn wt_tracker_spawn(
    h: *mut WtHandle,
    line: *const WtLine,
    stamp: f64,
    obs: *const WtObservation,
) -> i32 {
    guarded(|| {
        let (Some(h), Some(line)) = (handle(h), line.as_ref()) else {
            return WT_ERR_NULL;
        };
        let o = obs.as_ref().map(observation_of);
        lock(h).spawn(&line_of(line), stamp, o.as_ref());
        WT_OK
    })
}

/// `_merge()`.
///
/// # Safety
/// `h` a live handle.
#[no_mangle]
pub unsafe extern "C" fn wt_tracker_merge(h: *mut WtHandle) -> i32 {
    guarded(|| {
        let Some(h) = handle(h) else {
            return WT_ERR_NULL;
        };
        match lock(h).merge() {
            Ok(()) => WT_OK,
            Err(e) => status_of(e),
        }
    })
}

/// `rejections` (raw values).
///
/// # Safety
/// `out` `cap` writable WtRejection, `n` one i64.
#[no_mangle]
pub unsafe extern "C" fn wt_tracker_rejections(
    h: *mut WtHandle,
    out: *mut WtRejection,
    cap: i64,
    n: *mut i64,
) -> i32 {
    guarded(|| {
        let (Some(h), Some(n)) = (handle(h), n.as_mut()) else {
            return WT_ERR_NULL;
        };
        let t = lock(h);
        *n = t.rejections.len() as i64;
        if (t.rejections.len() as i64) > cap {
            return WT_ERR_CAPACITY;
        }
        let Some(o) = slice_mut(out, t.rejections.len() as i64) else {
            return WT_ERR_NULL;
        };
        for (slot, r) in o.iter_mut().zip(&t.rejections) {
            *slot = WtRejection {
                code: r.code as i64,
                has_id: i64::from(r.track_id.is_some()),
                id: r.track_id.unwrap_or(0),
                value: r.value,
            };
        }
        WT_OK
    })
}

// ---- the per-cycle fusion (wall_fusion + the node's glue) -----------------

/// `_publish_observations`' inputs on a cycle with a fresh stereo profile.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct WtFusionIn {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub r00: f64,
    pub r01: f64,
    pub r02: f64,
    pub r10: f64,
    pub r11: f64,
    pub r12: f64,
    pub r20: f64,
    pub r21: f64,
    pub r22: f64,
    pub t0: f64,
    pub t1: f64,
    pub t2: f64,
    pub laser_z: f64,
    pub width: i64,
    pub has_comp: i64,
    pub scan_x: f64,
    pub scan_y: f64,
    pub scan_theta: f64,
    pub image_x: f64,
    pub image_y: f64,
    pub image_theta: f64,
    pub tol_base: f64,
    pub tol_quad: f64,
    pub disagree_reject_frac: f64,
    pub stereo_weight: f64,
    pub inlier_ref: f64,
    pub rms_ref: f64,
    pub range_ref_m: f64,
    pub min_inlier_ref: f64,
    pub max_rms_scale: f64,
    pub min_span_columns: i64,
    pub inlier_m: f64,
    pub min_points: i64,
    pub min_span_m: f64,
    pub default_height: f64,
    pub max_pairs: i64,
}

const FUSION_IN_FIELDS: &str = "fx:f,fy:f,cx:f,cy:f,r00:f,r01:f,r02:f,r10:f,r11:f,r12:f,r20:f,\
r21:f,r22:f,t0:f,t1:f,t2:f,laser_z:f,width:i,has_comp:i,scan_x:f,scan_y:f,scan_theta:f,\
image_x:f,image_y:f,image_theta:f,tol_base:f,tol_quad:f,disagree_reject_frac:f,stereo_weight:f,\
inlier_ref:f,rms_ref:f,range_ref_m:f,min_inlier_ref:f,max_rms_scale:f,min_span_columns:i,\
inlier_m:f,min_points:i,min_span_m:f,default_height:f,max_pairs:i\0";

/// One fused observation; `line_index` -1 for a stereo-only line.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct WtFusionObs {
    pub line_index: i64,
    pub nx: f64,
    pub ny: f64,
    pub d: f64,
    pub t0: f64,
    pub t1: f64,
    pub inliers: i64,
    pub rms: f64,
    pub z_min: f64,
    pub z_max: f64,
    pub height_measured: i64,
    pub source: i64,
    pub confidence: f64,
    pub col_begin: i64,
    pub col_end: i64,
    pub n_agree: i64,
    pub n_lidar_only: i64,
    pub n_stereo_only: i64,
    pub n_disagree: i64,
    pub stereo_weight: f64,
}

const FUSION_OBS_FIELDS: &str = "line_index:i,nx:f,ny:f,d:f,t0:f,t1:f,inliers:i,rms:f,z_min:f,\
z_max:f,height_measured:i,source:i,confidence:f,col_begin:i,col_end:i,n_agree:i,n_lidar_only:i,\
n_stereo_only:i,n_disagree:i,stereo_weight:f\0";

/// One `verify_rectangles` entry.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct WtRectOut {
    pub coverage: f64,
    pub support: f64,
    pub beyond_share: f64,
    pub n_samples: i64,
    pub n_in_image: i64,
    pub at: f64,
    pub beyond: f64,
    pub against: f64,
}

const RECT_OUT_FIELDS: &str =
    "coverage:f,support:f,beyond_share:f,n_samples:i,n_in_image:i,at:f,beyond:f,against:f\0";

pub const STRUCT_FUSION_IN: i32 = 7;
pub const STRUCT_FUSION_OBS: i32 = 8;
pub const STRUCT_RECT_OUT: i32 = 9;

/// The cycle's fusion: the check (`status`, `lidar_m`, `stereo_m`, `tol_m`,
/// `width` each) and the observations (from the lines in order, then the
/// stereo-only ones), `n_rejected` lines dropped by the disagreement veto.
///
/// # Safety
/// `inp` one readable WtFusionIn; `lines` `7 n_lines` f64s (nx, ny, d, t0,
/// t1, inliers, rms); `pts` `2 n_pts`; `distance`, `z_min`, `z_max` their
/// lengths; `cloud` null or `3 n_cloud`; the four check outputs `width`
/// elements each; `out_obs` `cap_obs`; `n_obs`, `n_rejected` one i64 each.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn wt_fusion_cycle(
    inp: *const WtFusionIn,
    lines: *const f64,
    n_lines: i64,
    pts: *const f64,
    n_pts: i64,
    distance: *const f64,
    n_distance: i64,
    z_min: *const f64,
    n_z_min: i64,
    z_max: *const f64,
    n_z_max: i64,
    cloud: *const f64,
    n_cloud: i64,
    out_status: *mut u8,
    out_lidar: *mut f64,
    out_stereo: *mut f64,
    out_tol: *mut f64,
    out_obs: *mut WtFusionObs,
    cap_obs: i64,
    n_obs: *mut i64,
    n_rejected: *mut i64,
) -> i32 {
    use crate::fusion::{self, Camera, Compensation, ConfParams, FusionInput, StereoParams};
    guarded(|| {
        let Some(i) = inp.as_ref() else {
            return WT_ERR_NULL;
        };
        let Ok(width) = usize::try_from(i.width) else {
            return WT_ERR_LENGTH;
        };
        let (Some(ls), Some(ps), Some(ds), Some(zl), Some(zh), Some(no), Some(nr)) = (
            slice(lines, n_lines.saturating_mul(7)),
            slice(pts, n_pts.saturating_mul(2)),
            slice(distance, n_distance),
            slice(z_min, n_z_min),
            slice(z_max, n_z_max),
            n_obs.as_mut(),
            n_rejected.as_mut(),
        ) else {
            return WT_ERR_NULL;
        };
        let cloud_rows: Option<Vec<[f64; 3]>> = if cloud.is_null() {
            None
        } else {
            let Some(c) = slice(cloud, n_cloud.saturating_mul(3)) else {
                return WT_ERR_NULL;
            };
            Some(c.chunks_exact(3).map(|r| [r[0], r[1], r[2]]).collect())
        };
        let (Some(st), Some(lm), Some(sm), Some(tm)) = (
            slice_mut(out_status, i.width),
            slice_mut(out_lidar, i.width),
            slice_mut(out_stereo, i.width),
            slice_mut(out_tol, i.width),
        ) else {
            return WT_ERR_NULL;
        };
        let wl: Vec<WallLine> = ls
            .chunks_exact(7)
            .map(|r| WallLine {
                nx: r[0],
                ny: r[1],
                d: r[2],
                t0: r[3],
                t1: r[4],
                inliers: r[5] as i64,
                rms: r[6],
            })
            .collect();
        let sp: Vec<[f64; 2]> = ps.chunks_exact(2).map(|r| [r[0], r[1]]).collect();
        let camera = Camera {
            fx: i.fx,
            fy: i.fy,
            cx: i.cx,
            cy: i.cy,
            rot: [
                [i.r00, i.r01, i.r02],
                [i.r10, i.r11, i.r12],
                [i.r20, i.r21, i.r22],
            ],
            trans: [i.t0, i.t1, i.t2],
        };
        let compensation = (i.has_comp != 0).then_some(Compensation {
            pose_scan: Pose2D::new(i.scan_x, i.scan_y, i.scan_theta),
            pose_image: Pose2D::new(i.image_x, i.image_y, i.image_theta),
        });
        let input = FusionInput {
            lines: &wl,
            scan_pts: &sp,
            laser_z: i.laser_z,
            camera,
            width,
            distance_m: ds,
            z_min: zl,
            z_max: zh,
            compensation,
            cloud: cloud_rows.as_deref(),
            tol_base: i.tol_base,
            tol_quad: i.tol_quad,
            disagree_reject_frac: i.disagree_reject_frac,
            stereo_weight: i.stereo_weight,
            conf: ConfParams {
                inlier_ref: i.inlier_ref,
                rms_ref: i.rms_ref,
                range_ref_m: i.range_ref_m,
                min_inlier_ref: i.min_inlier_ref,
                max_rms_scale: i.max_rms_scale,
            },
            stereo: StereoParams {
                min_span_columns: i.min_span_columns,
                inlier_m: i.inlier_m,
                min_points: usize::try_from(i.min_points).unwrap_or(0),
                min_span_m: i.min_span_m,
                default_height: i.default_height,
                max_pairs: usize::try_from(i.max_pairs).unwrap_or(0),
            },
        };
        let (check, obs, rejected) = match fusion::fusion_cycle(&input) {
            Ok(v) => v,
            Err(fusion::FusionError::Overflow) => return WT_ERR_OVERFLOW,
        };
        st.copy_from_slice(&check.status);
        lm.copy_from_slice(&check.lidar_m);
        sm.copy_from_slice(&check.stereo_m);
        tm.copy_from_slice(&check.tol_m);
        *no = obs.len() as i64;
        *nr = rejected;
        if (obs.len() as i64) > cap_obs {
            return WT_ERR_CAPACITY;
        }
        let Some(out) = slice_mut(out_obs, obs.len() as i64) else {
            return WT_ERR_NULL;
        };
        for (slot, o) in out.iter_mut().zip(&obs) {
            *slot = WtFusionObs {
                line_index: o.line_index.map_or(-1, |k| k as i64),
                nx: o.line.nx,
                ny: o.line.ny,
                d: o.line.d,
                t0: o.line.t0,
                t1: o.line.t1,
                inliers: o.line.inliers,
                rms: o.line.rms,
                z_min: o.z_min,
                z_max: o.z_max,
                height_measured: i64::from(o.height_measured),
                source: i64::from(o.source),
                confidence: o.confidence,
                col_begin: o.col_begin,
                col_end: o.col_end,
                n_agree: o.n_agree,
                n_lidar_only: o.n_lidar_only,
                n_stereo_only: o.n_stereo_only,
                n_disagree: o.n_disagree,
                stereo_weight: o.stereo_weight,
            };
        }
        WT_OK
    })
}

/// `verify_rectangles` with the node's arguments (integer depth image, no
/// object mask, no detail). `cam` is fx, fy, cx, cy, R (9, row-major), t (3),
/// laser_z; `p` is t_step_min_m, t_steps_max, tol_base_m, tol_quad_per_m2,
/// min_depth_m, max_range_m.
///
/// # Safety
/// `segs` `4 n_seg` f64s; `labels` / `depth` `height * width` elements;
/// `cam` 17 and `p` 6 f64s; `grid_z` / `grid_w` `nz` f32s; `out` `n_seg`.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn wt_verify_rectangles(
    segs: *const f64,
    n_seg: i64,
    labels: *const u8,
    depth: *const u16,
    height: i64,
    width: i64,
    depth_scale: f32,
    wall_class: i64,
    object_class: i64,
    cam: *const f64,
    p: *const f64,
    grid_z: *const f32,
    grid_w: *const f32,
    nz: i64,
    out: *mut WtRectOut,
) -> i32 {
    use crate::fusion::{self, RectImage, RectSampleParams};
    guarded(|| {
        let (Ok(h), Ok(w)) = (usize::try_from(height), usize::try_from(width)) else {
            return WT_ERR_LENGTH;
        };
        let npx = (h * w) as i64;
        let (Some(sg), Some(lb), Some(dp), Some(cm), Some(pp), Some(gz), Some(gw), Some(o)) = (
            slice(segs, n_seg.saturating_mul(4)),
            slice(labels, npx),
            slice(depth, npx),
            slice(cam, 17),
            slice(p, 6),
            slice(grid_z, nz),
            slice(grid_w, nz),
            slice_mut(out, n_seg),
        ) else {
            return WT_ERR_NULL;
        };
        let (Ok(wc), Ok(oc)) = (u8::try_from(wall_class), u8::try_from(object_class)) else {
            return WT_ERR_LENGTH;
        };
        let segments: Vec<[[f64; 2]; 2]> = sg
            .chunks_exact(4)
            .map(|r| [[r[0], r[1]], [r[2], r[3]]])
            .collect();
        let img = RectImage {
            labels: lb,
            depth: dp,
            height: h,
            width: w,
            depth_scale,
            wall_class: wc,
            object_class: oc,
        };
        let rot = [
            [cm[4], cm[5], cm[6]],
            [cm[7], cm[8], cm[9]],
            [cm[10], cm[11], cm[12]],
        ];
        let trans = [cm[13], cm[14], cm[15]];
        let params = RectSampleParams {
            t_step_min_m: pp[0],
            t_steps_max: pp[1] as i64,
            tol_base_m: pp[2],
            tol_quad_per_m2: pp[3],
            min_depth_m: pp[4],
            max_range_m: pp[5],
        };
        let res = fusion::verify_rectangles(
            &segments, &img, cm[0], cm[1], cm[2], cm[3], &rot, &trans, cm[16], &params, gz, gw,
        );
        for (slot, r) in o.iter_mut().zip(res) {
            *slot = WtRectOut {
                coverage: r.coverage,
                support: r.support,
                beyond_share: r.beyond_share,
                n_samples: r.n_samples,
                n_in_image: r.n_in_image,
                at: r.at,
                beyond: r.beyond,
                against: r.against,
            };
        }
        WT_OK
    })
}

// ---- debug exports (the parity harness checks the numerics directly) ----

/// `np.linalg.solve(a, b)` for a row-major 2x2.
///
/// # Safety
/// `a` 4, `b` 2, `out` 2 f64s.
#[no_mangle]
pub unsafe extern "C" fn wt_debug_solve2(a: *const f64, b: *const f64, out: *mut f64) -> i32 {
    guarded(|| {
        let (Some(a), Some(b), Some(o)) = (slice(a, 4), slice(b, 2), slice_mut(out, 2)) else {
            return WT_ERR_NULL;
        };
        match linalg::solve(&[[a[0], a[1]], [a[2], a[3]]], [b[0], b[1]]) {
            Ok(x) => {
                o[0] = x[0];
                o[1] = x[1];
                WT_OK
            }
            Err(_) => WT_ERR_SINGULAR,
        }
    })
}

/// `np.linalg.inv(a)` for a row-major 2x2.
///
/// # Safety
/// `a` 4, `out` 4 f64s.
#[no_mangle]
pub unsafe extern "C" fn wt_debug_inv2(a: *const f64, out: *mut f64) -> i32 {
    guarded(|| {
        let (Some(a), Some(o)) = (slice(a, 4), slice_mut(out, 4)) else {
            return WT_ERR_NULL;
        };
        match linalg::inv(&[[a[0], a[1]], [a[2], a[3]]]) {
            Ok(x) => {
                o[0] = x[0][0];
                o[1] = x[0][1];
                o[2] = x[1][0];
                o[3] = x[1][1];
                WT_OK
            }
            Err(_) => WT_ERR_SINGULAR,
        }
    })
}

/// `linear_sum_assignment(cost)`: `min(nr, nc)` pairs into rows / cols.
///
/// # Safety
/// `cost` `nr * nc` f64s; `rows`, `cols` `min(nr, nc)` writable i64s.
#[no_mangle]
pub unsafe extern "C" fn wt_debug_lsa(
    nr: i64,
    nc: i64,
    cost: *const f64,
    rows: *mut i64,
    cols: *mut i64,
) -> i32 {
    guarded(|| {
        let (Ok(r), Ok(c)) = (usize::try_from(nr), usize::try_from(nc)) else {
            return WT_ERR_LENGTH;
        };
        let k = r.min(c) as i64;
        let (Some(cost), Some(ro), Some(co)) =
            (slice(cost, nr * nc), slice_mut(rows, k), slice_mut(cols, k))
        else {
            return WT_ERR_NULL;
        };
        match lsa::linear_sum_assignment(r, c, cost) {
            Ok((a, b)) => {
                for (slot, v) in ro.iter_mut().zip(a) {
                    *slot = v as i64;
                }
                for (slot, v) in co.iter_mut().zip(b) {
                    *slot = v as i64;
                }
                WT_OK
            }
            Err(LsaError::Invalid) => WT_ERR_LSA_INVALID,
            Err(LsaError::Infeasible) => WT_ERR_LSA_INFEASIBLE,
        }
    })
}

/// `np.linalg.svd(x, full_matrices=False)[2]` of an M x 2 row-major `x`
/// (M >= 3) into `out` (4, row-major). WT_ERR_LENGTH for M < 3.
///
/// # Safety
/// `x` `2 m` f64s, `out` 4.
#[no_mangle]
pub unsafe extern "C" fn wt_debug_svd_vt(x: *const f64, m: i64, out: *mut f64) -> i32 {
    guarded(|| {
        let (Some(x), Some(o)) = (slice(x, m.saturating_mul(2)), slice_mut(out, 4)) else {
            return WT_ERR_NULL;
        };
        let rows: Vec<[f64; 2]> = x.chunks_exact(2).map(|r| [r[0], r[1]]).collect();
        match crate::lapack::svd_vt_mx2(&rows) {
            Some(vt) => {
                o[0] = vt[0][0];
                o[1] = vt[0][1];
                o[2] = vt[1][0];
                o[3] = vt[1][1];
                WT_OK
            }
            None => WT_ERR_LENGTH,
        }
    })
}

/// `lidar_front_wall._robust_line(points, inlier_m, max_pairs)`: `out` =
/// [nx, ny, offset, count, rms]; returns 1 with a line, 0 for None, < 0 on
/// an error.
///
/// # Safety
/// `pts` `2 n` f64s, `out` 5.
#[no_mangle]
pub unsafe extern "C" fn wt_debug_robust_line(
    pts: *const f64,
    n: i64,
    inlier_m: f64,
    max_pairs: i64,
    out: *mut f64,
) -> i32 {
    guarded(|| {
        let (Some(p), Some(o), Ok(mp)) = (
            slice(pts, n.saturating_mul(2)),
            slice_mut(out, 5),
            usize::try_from(max_pairs),
        ) else {
            return WT_ERR_NULL;
        };
        let rows: Vec<[f64; 2]> = p.chunks_exact(2).map(|r| [r[0], r[1]]).collect();
        match crate::fusion::robust_line(&rows, inlier_m, mp) {
            Some((nx, ny, d, count, rms)) => {
                o.copy_from_slice(&[nx, ny, d, count as f64, rms]);
                1
            }
            None => 0,
        }
    })
}

/// CPython `math.hypot(x, y)`.
#[no_mangle]
pub extern "C" fn wt_debug_hypot(x: f64, y: f64) -> f64 {
    pyc::py_hypot(x, y)
}

/// Python `x ** 2`.
#[no_mangle]
pub extern "C" fn wt_debug_pow2(x: f64) -> f64 {
    pyc::py_pow2(x)
}

/// corridor_perception `wrap_pi(x)`.
#[no_mangle]
pub extern "C" fn wt_debug_wrap_pi(x: f64) -> f64 {
    pyc::wrap_pi(x)
}

/// CPython 3.12 `sum(v)` (NaN for an empty `v`).
///
/// # Safety
/// `v` `n` f64s.
#[no_mangle]
pub unsafe extern "C" fn wt_debug_sum(v: *const f64, n: i64) -> f64 {
    slice(v, n).and_then(pyc::py_sum).unwrap_or(f64::NAN)
}

/// `np.arange(start, stop, step)`: the length; the values into `out` when
/// it holds them.
///
/// # Safety
/// `out` `cap` writable f64s.
#[no_mangle]
pub unsafe extern "C" fn wt_debug_arange(
    start: f64,
    stop: f64,
    step: f64,
    out: *mut f64,
    cap: i64,
) -> i64 {
    let v = pyc::np_arange(start, stop, step);
    if (v.len() as i64) <= cap {
        if let Some(o) = slice_mut(out, v.len() as i64) {
            o.copy_from_slice(&v);
        }
    }
    v.len() as i64
}

/// `np.searchsorted(arr, keys)` (side left).
///
/// # Safety
/// `arr` `n` f64s, `keys` and `out` `nk` each.
#[no_mangle]
pub unsafe extern "C" fn wt_debug_searchsorted(
    arr: *const f64,
    n: i64,
    keys: *const f64,
    nk: i64,
    out: *mut i64,
) -> i32 {
    guarded(|| {
        let (Some(a), Some(k), Some(o)) = (slice(arr, n), slice(keys, nk), slice_mut(out, nk))
        else {
            return WT_ERR_NULL;
        };
        for (slot, v) in o.iter_mut().zip(pyc::np_searchsorted_left(a, k)) {
            *slot = v as i64;
        }
        WT_OK
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn count(s: &str) -> usize {
        s.trim_end_matches('\0').split(',').count()
    }

    #[test]
    fn field_lists_match_the_struct_sizes() {
        assert_eq!(
            count(OBSERVATION_FIELDS) * 8,
            std::mem::size_of::<WtObservation>()
        );
        assert_eq!(count(LINE_FIELDS) * 8, std::mem::size_of::<WtLine>());
        assert_eq!(count(TRACK_FIELDS) * 8, std::mem::size_of::<WtTrack>());
        assert_eq!(count(SCALARS_FIELDS) * 8, std::mem::size_of::<WtScalars>());
        assert_eq!(
            count(REJECTION_FIELDS) * 8,
            std::mem::size_of::<WtRejection>()
        );
        assert_eq!(count(SIZES_FIELDS) * 8, std::mem::size_of::<WtSizes>());
        assert_eq!(
            count(FUSION_IN_FIELDS) * 8,
            std::mem::size_of::<WtFusionIn>()
        );
        assert_eq!(
            count(FUSION_OBS_FIELDS) * 8,
            std::mem::size_of::<WtFusionObs>()
        );
        assert_eq!(count(RECT_OUT_FIELDS) * 8, std::mem::size_of::<WtRectOut>());
        assert_eq!(count(PARAM_F64_NAMES), N_PARAM_F64);
        assert_eq!(count(PARAM_I64_NAMES), N_PARAM_I64);
        assert_eq!(count(RECT_F64_NAMES), N_RECT_F64);
        assert_eq!(count(RECT_I64_NAMES), N_RECT_I64);
    }

    #[test]
    fn a_handle_round_trips_its_tracks() {
        let p = Params::default();
        let pf = [0.0; N_PARAM_F64];
        let pi = [0_i64; N_PARAM_I64];
        let rf = [0.0; N_RECT_F64];
        let ri = [0_i64; N_RECT_I64];
        let mut st = 1;
        let h = unsafe {
            wt_tracker_new(
                pf.as_ptr(),
                pf.len() as i64,
                pi.as_ptr(),
                pi.len() as i64,
                rf.as_ptr(),
                rf.len() as i64,
                ri.as_ptr(),
                ri.len() as i64,
                &mut st,
            )
        };
        assert_eq!(st, WT_OK);
        assert!(!h.is_null());
        {
            let hh = unsafe { handle(h) }.unwrap();
            let mut t = lock(hh);
            t.p = p;
            let o = Observation {
                line: WallLine {
                    nx: 1.0,
                    ny: 0.0,
                    d: 2.0,
                    t0: -1.0,
                    t1: 1.0,
                    inliers: 40,
                    rms: 0.005,
                },
                z_min: 0.0,
                z_max: 2.5,
                height_measured: false,
                source: 0,
                confidence: 0.8,
                stereo_weight: 1.0,
            };
            t.step(&[o], 0.0, &Pose2D::new(0.0, 0.0, 0.0), None, false)
                .unwrap();
        }
        let mut sizes = WtSizes::default();
        let st = unsafe {
            wt_tracker_export_tracks(
                h,
                std::ptr::null_mut(),
                0,
                std::ptr::null_mut(),
                0,
                std::ptr::null_mut(),
                0,
                std::ptr::null_mut(),
                0,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                0,
                &mut sizes,
            )
        };
        assert_eq!(st, WT_ERR_CAPACITY);
        assert_eq!(sizes.n_tracks, 1);
        let mut tr = vec![WtTrack::default(); 1];
        let mut cf = vec![0.0; 3 * sizes.n_confirm as usize];
        let mut li = vec![0_i64; sizes.n_lidar as usize];
        let mut co = vec![0.0; sizes.n_conf as usize];
        let mut ex = vec![0.0; 5 * sizes.n_ext as usize];
        let mut es = vec![0_i64; sizes.n_ext as usize];
        let st = unsafe {
            wt_tracker_export_tracks(
                h,
                tr.as_mut_ptr(),
                1,
                cf.as_mut_ptr(),
                sizes.n_confirm,
                li.as_mut_ptr(),
                sizes.n_lidar,
                co.as_mut_ptr(),
                sizes.n_conf,
                ex.as_mut_ptr(),
                es.as_mut_ptr(),
                sizes.n_ext,
                &mut sizes,
            )
        };
        assert_eq!(st, WT_OK);
        let before = tr[0].hash;
        let st = unsafe {
            wt_tracker_import_tracks(
                h,
                1,
                tr.as_ptr(),
                cf.as_ptr(),
                sizes.n_confirm,
                li.as_ptr(),
                sizes.n_lidar,
                co.as_ptr(),
                sizes.n_conf,
                ex.as_ptr(),
                es.as_ptr(),
                sizes.n_ext,
            )
        };
        assert_eq!(st, WT_OK);
        let st = unsafe {
            wt_tracker_export_tracks(
                h,
                tr.as_mut_ptr(),
                1,
                cf.as_mut_ptr(),
                sizes.n_confirm,
                li.as_mut_ptr(),
                sizes.n_lidar,
                co.as_mut_ptr(),
                sizes.n_conf,
                ex.as_mut_ptr(),
                es.as_mut_ptr(),
                sizes.n_ext,
                &mut sizes,
            )
        };
        assert_eq!(st, WT_OK);
        assert_eq!(
            tr[0].hash, before,
            "an import of an export is the same state"
        );
        unsafe { wt_tracker_free(h) };
    }
}
