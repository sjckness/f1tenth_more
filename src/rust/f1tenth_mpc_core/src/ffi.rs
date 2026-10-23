//! The C ABI that `mpc_controller/mpc_native.py` loads with ctypes.
//!
//! The only module of the crate allowed to use `unsafe`: raw pointers come in
//! from Python, are checked for null and turned into slices of the length the
//! caller declared, and nothing else is unsafe.
//!
//! CONTRACT (kept in step with mpc_native.py; [`ABI_VERSION`] changes whenever
//! any of it does, and the loader refuses a mismatch):
//! - every struct is `#[repr(C)]`; [`f1tenth_mpc_core_struct_size`] reports
//!   each one's size so the loader can compare it with its ctypes mirror;
//! - optional inputs are an `f64` plus an `i32` `has_*` flag (not NaN as a
//!   sentinel: NaN is a legal input that the Python passes through);
//! - outputs are written into caller-owned float64 buffers of exactly `n`
//!   elements ([`MpcCorridorBuffers`]); the dict's `ny` is `tx`, so there is
//!   no `ny` buffer;
//! - every entry point returns an `i32` status ([`MPC_OK`] or a negative
//!   `MPC_ERR_*`) and never unwinds into C: a panic is caught and reported as
//!   [`MPC_ERR_PANIC`].
//!
//! NOT a shared-state API: every call is pure, so concurrent calls are safe.
#![allow(unsafe_code)]

use std::ffi::c_char;
use std::panic::{catch_unwind, AssertUnwindSafe};

use crate::corridor::{self, Corridor, CorridorError, CurvesArgs, PoseArgs, Walls};
use crate::query::{self, QueryError};

/// Bumped on any change to a struct layout, a signature or a status code.
pub const ABI_VERSION: u32 = 1;

pub const MPC_OK: i32 = 0;
/// A required pointer was null.
pub const MPC_ERR_NULL: i32 = -1;
/// `n < 2` (Python: ValueError "a corridor needs at least 2 samples").
pub const MPC_ERR_TOO_FEW_SAMPLES: i32 = -2;
/// `walls` is not 0 (bezier) or 1 (trapezoid).
pub const MPC_ERR_WALLS: i32 = -3;
/// A buffer length differs from `n` / the arrays differ in length.
pub const MPC_ERR_LENGTH: i32 = -4;
/// `math.sin`/`math.cos` of an infinite angle (Python: ValueError).
pub const MPC_ERR_MATH_DOMAIN: i32 = -5;
/// `1.0 / r_min` with `r_min == 0` (Python: ZeroDivisionError).
pub const MPC_ERR_ZERO_DIVISION: i32 = -6;
/// A Rust panic, caught at the boundary (a bug: report it).
pub const MPC_ERR_PANIC: i32 = -7;
/// `n` above corridor::MAX_SAMPLES.
pub const MPC_ERR_TOO_MANY_SAMPLES: i32 = -8;
/// An empty centreline (Python: np.argmin ValueError).
pub const MPC_ERR_EMPTY: i32 = -9;

/// `corridor_curves(...)` inputs.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MpcCorridorCurvesArgs {
    pub x0: f64,
    pub y0: f64,
    pub psi_start: f64,
    pub psi_end: f64,
    pub length: f64,
    pub dpsi: f64,
    pub u_start: f64,
    pub u_end: f64,
    pub w0: f64,
    pub w1: f64,
    pub handle_frac: f64,
    pub r_min: f64,
    pub length_ref: f64,
    pub n: i64,
    pub has_dpsi: i32,
    pub has_r_min: i32,
    pub has_length_ref: i32,
    /// 0 bezier, 1 trapezoid.
    pub walls: i32,
}

/// `corridor_curves_to_pose(...)` inputs.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MpcCorridorPoseArgs {
    pub x0: f64,
    pub y0: f64,
    pub psi_start: f64,
    pub x1: f64,
    pub y1: f64,
    pub psi_end: f64,
    pub handle_a: f64,
    pub handle_b: f64,
    pub w0: f64,
    pub w1: f64,
    pub handle_frac: f64,
    pub r_min: f64,
    pub length_ref: f64,
    pub n: i64,
    pub has_r_min: i32,
    pub has_length_ref: i32,
    /// 0 bezier, 1 trapezoid.
    pub walls: i32,
    pub reserved: i32,
}

/// The scalar part of the corridor dict.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MpcCorridorInfo {
    pub pend: [f64; 2],
    pub dpsi: f64,
    /// Ramp: the input length. Pose: the arclength (the dict's `length`).
    pub length: f64,
    pub u_end_eff: f64,
    pub w1_eff: f64,
    pub kappa_max: f64,
    /// `[x, y]` of the four control points, in order.
    pub ctrl_left: [f64; 8],
    pub ctrl_right: [f64; 8],
    pub ctrl_centre: [f64; 8],
    pub ramp_clamped: i32,
    pub feasible: i32,
    /// 1: ctrl_left/ctrl_right are set (bezier walls); 0: None (trapezoid).
    pub has_ctrl_walls: i32,
    /// 1: ctrl_centre is set (pose corridor).
    pub has_ctrl_centre: i32,
}

/// Caller-owned output arrays, `n` float64 each.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MpcCorridorBuffers {
    pub n: i64,
    pub xc: *mut f64,
    pub yc: *mut f64,
    pub x_l: *mut f64,
    pub y_l: *mut f64,
    pub x_r: *mut f64,
    pub y_r: *mut f64,
    pub tx: *mut f64,
    pub ty: *mut f64,
    pub nx: *mut f64,
    pub half_width: *mut f64,
}

/// `_nearest_corridor_frame` plus `d_lat` and the heading.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MpcCorridorFrame {
    pub idx: i64,
    pub d2: f64,
    pub pc: [f64; 2],
    pub n: [f64; 2],
    pub half_w: f64,
    pub d_lat: f64,
    pub heading: f64,
}

pub const STRUCT_CURVES_ARGS: i32 = 1;
pub const STRUCT_POSE_ARGS: i32 = 2;
pub const STRUCT_INFO: i32 = 3;
pub const STRUCT_BUFFERS: i32 = 4;
pub const STRUCT_FRAME: i32 = 5;

static VERSION: &[u8] = concat!(env!("CARGO_PKG_VERSION"), "\0").as_bytes();

/// [`ABI_VERSION`].
#[no_mangle]
pub extern "C" fn f1tenth_mpc_core_abi_version() -> u32 {
    ABI_VERSION
}

/// `size_of` the struct `kind` (`STRUCT_*`), -1 for an unknown kind.
#[no_mangle]
pub extern "C" fn f1tenth_mpc_core_struct_size(kind: i32) -> i64 {
    let size = match kind {
        STRUCT_CURVES_ARGS => std::mem::size_of::<MpcCorridorCurvesArgs>(),
        STRUCT_POSE_ARGS => std::mem::size_of::<MpcCorridorPoseArgs>(),
        STRUCT_INFO => std::mem::size_of::<MpcCorridorInfo>(),
        STRUCT_BUFFERS => std::mem::size_of::<MpcCorridorBuffers>(),
        STRUCT_FRAME => std::mem::size_of::<MpcCorridorFrame>(),
        _ => return -1,
    };
    i64::try_from(size).unwrap_or(-1)
}

/// The crate version, a static NUL-terminated string.
#[no_mangle]
pub extern "C" fn f1tenth_mpc_core_version() -> *const c_char {
    VERSION.as_ptr().cast()
}

/// Writes the build summary (`build_info::summary()`) into `buf` as a
/// NUL-terminated string, truncated to `cap - 1` bytes. Returns the full
/// length without the NUL, or MPC_ERR_NULL.
///
/// # Safety
/// `buf` must point to `cap` writable bytes.
#[no_mangle]
pub unsafe extern "C" fn f1tenth_mpc_core_build_info(buf: *mut c_char, cap: i64) -> i64 {
    let s = crate::build_info::summary();
    let bytes = s.as_bytes();
    let Ok(cap) = usize::try_from(cap) else {
        return i64::from(MPC_ERR_NULL);
    };
    if buf.is_null() || cap == 0 {
        return i64::from(MPC_ERR_NULL);
    }
    let len = bytes.len().min(cap - 1);
    // SAFETY: the caller guarantees `cap` writable bytes; len + 1 <= cap.
    let out = unsafe { std::slice::from_raw_parts_mut(buf.cast::<u8>(), cap) };
    out[..len].copy_from_slice(&bytes[..len]);
    out[len] = 0;
    i64::try_from(bytes.len()).unwrap_or(i64::MAX)
}

fn walls_from(code: i32) -> Option<Walls> {
    match code {
        0 => Some(Walls::Bezier),
        1 => Some(Walls::Trapezoid),
        _ => None,
    }
}

fn opt(has: i32, v: f64) -> Option<f64> {
    (has != 0).then_some(v)
}

fn status_of(e: CorridorError) -> i32 {
    match e {
        CorridorError::TooFewSamples(_) => MPC_ERR_TOO_FEW_SAMPLES,
        CorridorError::TooManySamples(_) => MPC_ERR_TOO_MANY_SAMPLES,
        CorridorError::MathDomain => MPC_ERR_MATH_DOMAIN,
        CorridorError::ZeroDivision => MPC_ERR_ZERO_DIVISION,
    }
}

fn flatten(points: Option<[corridor::Point; 4]>) -> [f64; 8] {
    let mut out = [0.0; 8];
    if let Some(p) = points {
        for (i, pt) in p.iter().enumerate() {
            out[2 * i] = pt[0];
            out[2 * i + 1] = pt[1];
        }
    }
    out
}

/// Copies `c` into the caller's buffers and info struct.
///
/// # Safety
/// `buf`'s ten pointers must each point to `buf.n` writable f64s.
unsafe fn write_out(c: &Corridor, info: &mut MpcCorridorInfo, buf: &MpcCorridorBuffers) -> i32 {
    let n = c.xc.len();
    if usize::try_from(buf.n).ok() != Some(n) {
        return MPC_ERR_LENGTH;
    }
    let targets = [
        (buf.xc, &c.xc),
        (buf.yc, &c.yc),
        (buf.x_l, &c.x_l),
        (buf.y_l, &c.y_l),
        (buf.x_r, &c.x_r),
        (buf.y_r, &c.y_r),
        (buf.tx, &c.tx),
        (buf.ty, &c.ty),
        (buf.nx, &c.nx),
        (buf.half_width, &c.half_width),
    ];
    if targets.iter().any(|(p, _)| p.is_null()) {
        return MPC_ERR_NULL;
    }
    for (ptr, src) in targets {
        if src.len() != n {
            return MPC_ERR_LENGTH;
        }
        // SAFETY: non-null and `n` writable f64s by the caller's contract.
        let dst = unsafe { std::slice::from_raw_parts_mut(ptr, n) };
        dst.copy_from_slice(src);
    }
    info.pend = c.pend;
    info.dpsi = c.dpsi;
    info.length = c.length;
    info.u_end_eff = c.u_end_eff;
    info.w1_eff = c.w1_eff;
    info.kappa_max = c.kappa_max;
    info.ctrl_left = flatten(c.ctrl_left);
    info.ctrl_right = flatten(c.ctrl_right);
    info.ctrl_centre = flatten(c.ctrl_centre);
    info.ramp_clamped = i32::from(c.ramp_clamped);
    info.feasible = i32::from(c.feasible);
    info.has_ctrl_walls = i32::from(c.ctrl_left.is_some());
    info.has_ctrl_centre = i32::from(c.ctrl_centre.is_some());
    MPC_OK
}

fn guarded(f: impl FnOnce() -> i32) -> i32 {
    catch_unwind(AssertUnwindSafe(f)).unwrap_or(MPC_ERR_PANIC)
}

/// `corridor_curves(...)` into the caller's buffers.
///
/// # Safety
/// `args`, `info` and `buf` must be valid pointers to their structs, and
/// every pointer in `buf` must point to `buf.n` writable f64s.
#[no_mangle]
pub unsafe extern "C" fn mpc_corridor_curves(
    args: *const MpcCorridorCurvesArgs,
    info: *mut MpcCorridorInfo,
    buf: *const MpcCorridorBuffers,
) -> i32 {
    if args.is_null() || info.is_null() || buf.is_null() {
        return MPC_ERR_NULL;
    }
    // SAFETY: non-null, valid by the caller's contract.
    let (a, info, buf) = unsafe { (*args, &mut *info, &*buf) };
    guarded(|| {
        let Some(walls) = walls_from(a.walls) else {
            return MPC_ERR_WALLS;
        };
        let ca = CurvesArgs {
            x0: a.x0,
            y0: a.y0,
            psi_start: a.psi_start,
            psi_end: a.psi_end,
            length: a.length,
            n: a.n,
            dpsi: opt(a.has_dpsi, a.dpsi),
            u_start: a.u_start,
            u_end: a.u_end,
            w0: a.w0,
            w1: a.w1,
            handle_frac: a.handle_frac,
            r_min: opt(a.has_r_min, a.r_min),
            length_ref: opt(a.has_length_ref, a.length_ref),
            walls,
        };
        match corridor::corridor_curves(&ca) {
            // SAFETY: the buffer contract is the caller's (see above).
            Ok(c) => unsafe { write_out(&c, info, buf) },
            Err(e) => status_of(e),
        }
    })
}

/// `corridor_curves_to_pose(...)` into the caller's buffers.
///
/// # Safety
/// As [`mpc_corridor_curves`].
#[no_mangle]
pub unsafe extern "C" fn mpc_corridor_curves_to_pose(
    args: *const MpcCorridorPoseArgs,
    info: *mut MpcCorridorInfo,
    buf: *const MpcCorridorBuffers,
) -> i32 {
    if args.is_null() || info.is_null() || buf.is_null() {
        return MPC_ERR_NULL;
    }
    // SAFETY: non-null, valid by the caller's contract.
    let (a, info, buf) = unsafe { (*args, &mut *info, &*buf) };
    guarded(|| {
        let Some(walls) = walls_from(a.walls) else {
            return MPC_ERR_WALLS;
        };
        let pa = PoseArgs {
            x0: a.x0,
            y0: a.y0,
            psi_start: a.psi_start,
            x1: a.x1,
            y1: a.y1,
            psi_end: a.psi_end,
            n: a.n,
            handle_a: a.handle_a,
            handle_b: a.handle_b,
            w0: a.w0,
            w1: a.w1,
            handle_frac: a.handle_frac,
            r_min: opt(a.has_r_min, a.r_min),
            length_ref: opt(a.has_length_ref, a.length_ref),
            walls,
        };
        match corridor::corridor_curves_to_pose(&pa) {
            // SAFETY: the buffer contract is the caller's (see above).
            Ok(c) => unsafe { write_out(&c, info, buf) },
            Err(e) => status_of(e),
        }
    })
}

/// # Safety
/// `ptr` must be null or point to `n` readable f64s.
unsafe fn slice_in<'a>(ptr: *const f64, n: usize) -> Option<&'a [f64]> {
    if ptr.is_null() {
        return None;
    }
    // SAFETY: non-null and `n` readable f64s by the caller's contract.
    Some(unsafe { std::slice::from_raw_parts(ptr, n) })
}

fn query_status(e: QueryError) -> i32 {
    match e {
        QueryError::Empty => MPC_ERR_EMPTY,
        QueryError::LengthMismatch => MPC_ERR_LENGTH,
    }
}

/// `nearest_centerline_index(p, corridor)`: writes `d2[idx]` and `idx`.
///
/// # Safety
/// `xc` and `yc` must point to `n` readable f64s; `d2` and `idx` must be
/// valid for writing.
#[no_mangle]
pub unsafe extern "C" fn mpc_nearest_centerline_index(
    xc: *const f64,
    yc: *const f64,
    n: i64,
    px: f64,
    py: f64,
    d2: *mut f64,
    idx: *mut i64,
) -> i32 {
    let Ok(nu) = usize::try_from(n) else {
        return MPC_ERR_LENGTH;
    };
    if nu == 0 {
        return MPC_ERR_EMPTY;
    }
    // SAFETY: the caller's contract.
    let (Some(xs), Some(ys)) = (unsafe { slice_in(xc, nu) }, unsafe { slice_in(yc, nu) }) else {
        return MPC_ERR_NULL;
    };
    if d2.is_null() || idx.is_null() {
        return MPC_ERR_NULL;
    }
    guarded(|| match query::nearest_centerline_index(xs, ys, [px, py]) {
        Ok((v, i)) => {
            // SAFETY: non-null, valid for writing by the caller's contract.
            unsafe {
                *d2 = v;
                *idx = i64::try_from(i).unwrap_or(-1);
            }
            MPC_OK
        }
        Err(e) => query_status(e),
    })
}

/// `_nearest_corridor_frame(p, corridor)` with `d_lat` and the heading.
///
/// # Safety
/// The five arrays must point to `n` readable f64s each; `out` must be valid
/// for writing.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn mpc_corridor_frame(
    xc: *const f64,
    yc: *const f64,
    nx: *const f64,
    ny: *const f64,
    half_width: *const f64,
    n: i64,
    px: f64,
    py: f64,
    out: *mut MpcCorridorFrame,
) -> i32 {
    let Ok(nu) = usize::try_from(n) else {
        return MPC_ERR_LENGTH;
    };
    if nu == 0 {
        return MPC_ERR_EMPTY;
    }
    if out.is_null() {
        return MPC_ERR_NULL;
    }
    // SAFETY: the caller's contract.
    let arrays = unsafe {
        (
            slice_in(xc, nu),
            slice_in(yc, nu),
            slice_in(nx, nu),
            slice_in(ny, nu),
            slice_in(half_width, nu),
        )
    };
    let (Some(xs), Some(ys), Some(nxs), Some(nys), Some(hws)) = arrays else {
        return MPC_ERR_NULL;
    };
    guarded(
        || match query::corridor_frame(xs, ys, nxs, nys, hws, [px, py]) {
            Ok(f) => {
                let frame = MpcCorridorFrame {
                    idx: i64::try_from(f.idx).unwrap_or(-1),
                    d2: f.d2,
                    pc: f.pc,
                    n: f.n,
                    half_w: f.half_w,
                    d_lat: f.d_lat,
                    heading: f.heading,
                };
                // SAFETY: non-null, valid for writing by the caller's contract.
                unsafe { *out = frame };
                MPC_OK
            }
            Err(e) => query_status(e),
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn buffers(store: &mut [Vec<f64>; 10], n: usize) -> MpcCorridorBuffers {
        for v in store.iter_mut() {
            v.resize(n, f64::NAN);
        }
        let [a, b, c, d, e, f, g, h, i, j] = store;
        MpcCorridorBuffers {
            n: n as i64,
            xc: a.as_mut_ptr(),
            yc: b.as_mut_ptr(),
            x_l: c.as_mut_ptr(),
            y_l: d.as_mut_ptr(),
            x_r: e.as_mut_ptr(),
            y_r: f.as_mut_ptr(),
            tx: g.as_mut_ptr(),
            ty: h.as_mut_ptr(),
            nx: i.as_mut_ptr(),
            half_width: j.as_mut_ptr(),
        }
    }

    fn zero_info() -> MpcCorridorInfo {
        MpcCorridorInfo {
            pend: [0.0; 2],
            dpsi: 0.0,
            length: 0.0,
            u_end_eff: 0.0,
            w1_eff: 0.0,
            kappa_max: 0.0,
            ctrl_left: [0.0; 8],
            ctrl_right: [0.0; 8],
            ctrl_centre: [0.0; 8],
            ramp_clamped: 0,
            feasible: 0,
            has_ctrl_walls: 0,
            has_ctrl_centre: 0,
        }
    }

    fn curves_args(n: i64) -> MpcCorridorCurvesArgs {
        MpcCorridorCurvesArgs {
            x0: 0.5,
            y0: -1.0,
            psi_start: 0.1,
            psi_end: 0.7,
            length: 3.0,
            dpsi: 0.0,
            u_start: 0.0,
            u_end: 0.4,
            w0: 0.4333,
            w1: 0.7667,
            handle_frac: 0.55,
            r_min: 0.923,
            length_ref: 0.0,
            n,
            has_dpsi: 0,
            has_r_min: 1,
            has_length_ref: 0,
            walls: 0,
        }
    }

    #[test]
    fn the_abi_reports_itself() {
        assert_eq!(f1tenth_mpc_core_abi_version(), ABI_VERSION);
        for kind in 1..=5 {
            assert!(f1tenth_mpc_core_struct_size(kind) > 0);
        }
        assert_eq!(f1tenth_mpc_core_struct_size(99), -1);
        let mut buf = [0 as c_char; 128];
        let len = unsafe { f1tenth_mpc_core_build_info(buf.as_mut_ptr(), 128) };
        assert!(len > 10);
    }

    #[test]
    fn the_ffi_writes_what_the_library_computes() {
        let mut store: [Vec<f64>; 10] = Default::default();
        let buf = buffers(&mut store, 120);
        let mut info = zero_info();
        let args = curves_args(120);
        let status = unsafe { mpc_corridor_curves(&args, &mut info, &buf) };
        assert_eq!(status, MPC_OK);
        let mut ca = CurvesArgs::new(0.5, -1.0, 0.1, 0.7, 3.0, 120);
        ca.r_min = Some(0.923);
        let c = corridor::corridor_curves(&ca).unwrap();
        assert_eq!(store[0], c.xc);
        assert_eq!(store[9], c.half_width);
        assert_eq!(info.feasible, i32::from(c.feasible));
        assert_eq!(info.kappa_max.to_bits(), c.kappa_max.to_bits());
        assert_eq!(info.has_ctrl_walls, 1);
        assert_eq!(info.has_ctrl_centre, 0);
    }

    #[test]
    fn the_ffi_reports_errors() {
        let mut store: [Vec<f64>; 10] = Default::default();
        let buf = buffers(&mut store, 120);
        let mut info = zero_info();
        let mut args = curves_args(1);
        assert_eq!(
            unsafe { mpc_corridor_curves(&args, &mut info, &buf) },
            MPC_ERR_TOO_FEW_SAMPLES
        );
        args.n = 121;
        assert_eq!(
            unsafe { mpc_corridor_curves(&args, &mut info, &buf) },
            MPC_ERR_LENGTH
        );
        args.n = 120;
        args.walls = 7;
        assert_eq!(
            unsafe { mpc_corridor_curves(&args, &mut info, &buf) },
            MPC_ERR_WALLS
        );
        args.walls = 0;
        args.psi_end = f64::NEG_INFINITY;
        assert_eq!(
            unsafe { mpc_corridor_curves(&args, &mut info, &buf) },
            MPC_ERR_MATH_DOMAIN
        );
        assert_eq!(
            unsafe { mpc_corridor_curves(std::ptr::null(), &mut info, &buf) },
            MPC_ERR_NULL
        );
        let mut d2 = 0.0;
        let mut idx = 0_i64;
        assert_eq!(
            unsafe {
                mpc_nearest_centerline_index(
                    std::ptr::null(),
                    std::ptr::null(),
                    0,
                    0.0,
                    0.0,
                    &mut d2,
                    &mut idx,
                )
            },
            MPC_ERR_EMPTY
        );
    }

    #[test]
    fn the_frame_query_round_trips() {
        let xc = [0.0, 1.0, 2.0];
        let yc = [0.0, 0.0, 0.0];
        let nx = [0.0, 0.0, 0.0];
        let ny = [1.0, 1.0, 1.0];
        let hw = [0.4, 0.5, 0.6];
        let mut f = MpcCorridorFrame::default();
        let s = unsafe {
            mpc_corridor_frame(
                xc.as_ptr(),
                yc.as_ptr(),
                nx.as_ptr(),
                ny.as_ptr(),
                hw.as_ptr(),
                3,
                1.9,
                -0.2,
                &mut f,
            )
        };
        assert_eq!(s, MPC_OK);
        assert_eq!(f.idx, 2);
        assert_eq!(f.half_w, 0.6);
    }
}
