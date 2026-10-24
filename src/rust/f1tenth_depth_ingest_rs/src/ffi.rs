//! C ABI for f1tenth_perception/depth_ingest_native.py (ctypes): the same
//! [`crate::ingest`] state machines, synchronizer and median the node runs,
//! for the parity harness (tools/rust_parity/depth_ingest) and the
//! in-process test hook (F1TENTH_DEPTH_INGEST_INPROCESS).
//!
//! Images travel as seven scalars (data, length, height, width, step,
//! is_bigendian, encoding); results as flat int64 / float64 / float32 arrays
//! whose index order is exported by [`di_layout`] and checked by the loader.
//! Every function returns 0 on success and a negative code on bad arguments.
//! Handles are opaque boxes; a null handle is refused, a freed one is the
//! caller's bug.
#![allow(unsafe_code)]

use std::ffi::{c_char, CStr};

use crate::build_info;
use crate::detection::{BoxPx, SampleParams};
use crate::image::ImageParts;
use crate::ingest::{DetectionIngest, DetectionParams, RoiIngest, RoiOutcome};
use crate::median::median_f32;
use crate::roi::RoiParams;
use crate::sync::ApproxSync;

/// Bumped on any change of a signature or a layout below.
pub const ABI_VERSION: u32 = 1;

const ERR_ARGS: i32 = -1;
const ERR_CAPACITY: i32 = -2;

/// The flat result layouts, by kind (`di_layout`).
pub const LAYOUT_FRAME_I64: i32 = 1;
pub const LAYOUT_FRAME_F64: i32 = 2;
pub const LAYOUT_DET_I64: i32 = 3;
pub const LAYOUT_ROI_I64: i32 = 4;

const FRAME_I64: &CStr = c"depth_status,mask_status,depth_width,depth_height,mask_width,mask_height,has_scene";
const FRAME_F64: &CStr = c"fx,fy,cx,cy,scene_stamp";
const DET_I64: &CStr = c"source,valid,box_px,mask_px,other_px";
const ROI_I64: &CStr = c"status,width,height,y0,y1,x0,x1,roi_len,ground_stride,ground_rows,ground_cols,scene_mask_tick,scene_labels_tick";

fn count(names: &CStr) -> usize {
    names.to_bytes().split(|&b| b == b',').count()
}

/// # Safety
/// `ptr` must be valid for `len` reads (or null with len 0).
unsafe fn slice<'a, T>(ptr: *const T, len: i64) -> Option<&'a [T]> {
    if len < 0 {
        return None;
    }
    if len == 0 {
        return Some(&[]);
    }
    if ptr.is_null() {
        return None;
    }
    Some(std::slice::from_raw_parts(ptr, len as usize))
}

/// # Safety
/// `ptr` must be valid for `len` writes (or null with len 0).
unsafe fn slice_mut<'a, T>(ptr: *mut T, len: i64) -> Option<&'a mut [T]> {
    if len < 0 {
        return None;
    }
    if len == 0 {
        return Some(&mut []);
    }
    if ptr.is_null() {
        return None;
    }
    Some(std::slice::from_raw_parts_mut(ptr, len as usize))
}

/// # Safety
/// `data` valid for `len` reads, `encoding` a NUL-terminated string.
#[allow(clippy::too_many_arguments)]
unsafe fn image<'a>(
    data: *const u8,
    len: i64,
    height: i64,
    width: i64,
    step: i64,
    is_bigendian: i64,
    encoding: *const c_char,
) -> Option<ImageParts<'a>> {
    if encoding.is_null() {
        return None;
    }
    let to_u32 = |v: i64| u32::try_from(v).ok();
    Some(ImageParts {
        height: to_u32(height)?,
        width: to_u32(width)?,
        encoding: CStr::from_ptr(encoding).to_str().ok()?,
        is_bigendian: u8::from(is_bigendian != 0),
        step: to_u32(step)?,
        data: slice(data, len)?,
    })
}

/// # Safety
/// `buf` valid for `cap` writes.
unsafe fn write_text(text: &str, buf: *mut c_char, cap: i64) {
    let Some(out) = slice_mut(buf.cast::<u8>(), cap) else {
        return;
    };
    if out.is_empty() {
        return;
    }
    let n = text.len().min(out.len() - 1);
    out[..n].copy_from_slice(&text.as_bytes()[..n]);
    out[n] = 0;
}

#[no_mangle]
pub extern "C" fn di_abi_version() -> u32 {
    ABI_VERSION
}

static VERSION: &CStr = c"0.1.0";

#[no_mangle]
pub extern "C" fn di_version() -> *const c_char {
    VERSION.as_ptr()
}

/// The build summary (build_info.rs) into `buf`; returns its length.
///
/// # Safety
/// `buf` valid for `cap` writes.
#[no_mangle]
pub unsafe extern "C" fn di_build_info(buf: *mut c_char, cap: i64) -> i64 {
    let s = build_info::summary();
    write_text(&s, buf, cap);
    s.len() as i64
}

/// The index names of a flat result array (comma-separated), or null.
#[no_mangle]
pub extern "C" fn di_layout(kind: i32) -> *const c_char {
    match kind {
        LAYOUT_FRAME_I64 => FRAME_I64.as_ptr(),
        LAYOUT_FRAME_F64 => FRAME_F64.as_ptr(),
        LAYOUT_DET_I64 => DET_I64.as_ptr(),
        LAYOUT_ROI_I64 => ROI_I64.as_ptr(),
        _ => std::ptr::null(),
    }
}

/// `float(np.median(v))` of `n` float32 values; 1 for n == 0.
///
/// # Safety
/// `v` valid for `n` reads, `out` for one write.
#[no_mangle]
pub unsafe extern "C" fn di_median_f32(v: *const f32, n: i64, out: *mut f32) -> i32 {
    let (Some(v), false) = (slice(v, n), out.is_null()) else {
        return ERR_ARGS;
    };
    let mut copy = v.to_vec();
    match median_f32(&mut copy) {
        Some(m) => {
            *out = m;
            0
        }
        None => 1,
    }
}

/// `cv2.resize(src, (dw, dh), interpolation=INTER_NEAREST)` of a uint8 image.
///
/// # Safety
/// `src` valid for `sh * sstep` reads, `dst` for `dh * dw` writes.
#[no_mangle]
pub unsafe extern "C" fn di_resize_nearest_u8(
    src: *const u8,
    sh: i64,
    sw: i64,
    sstep: i64,
    dst: *mut u8,
    dh: i64,
    dw: i64,
) -> i32 {
    if sh < 1 || sw < 1 || dh < 1 || dw < 1 || sstep < sw {
        return ERR_ARGS;
    }
    let (Some(s), Some(d)) = (slice(src, sh * sstep), slice_mut(dst, dh * dw)) else {
        return ERR_ARGS;
    };
    let img = match crate::image::MonoImage::decode(&ImageParts {
        height: sh as u32,
        width: sw as u32,
        encoding: "mono8",
        is_bigendian: 0,
        step: sstep as u32,
        data: s,
    }) {
        Ok(i) => i,
        Err(_) => return ERR_ARGS,
    };
    let r = img.resize_nearest(dh as usize, dw as usize);
    for y in 0..dh as usize {
        d[y * dw as usize..(y + 1) * dw as usize].copy_from_slice(r.row(y));
    }
    0
}

// ---------------------------------------------------------------- detection

pub struct DetHandle {
    ingest: DetectionIngest,
}

/// A detection-side ingest with detection_3d_node's settings; null (and the
/// reason in `err`) when they are refused.
///
/// # Safety
/// `err` valid for `err_cap` writes.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn di_det_new(
    center_fraction: f64,
    use_mask_depth: i32,
    scene_enabled: i32,
    scene_labels_max_age_s: f64,
    scene_min_other_px: i64,
    sync_slop: f64,
    sync_queue_size: i64,
    err: *mut c_char,
    err_cap: i64,
) -> *mut DetHandle {
    let params = DetectionParams {
        sample: SampleParams {
            center_fraction,
            use_mask_depth: use_mask_depth != 0,
            scene_enabled: scene_enabled != 0,
            scene_min_other_px,
        },
        scene_labels_max_age_s,
        sync_slop,
        sync_queue_size,
    };
    if let Err(e) = params.validate() {
        write_text(&e, err, err_cap);
        return std::ptr::null_mut();
    }
    Box::into_raw(Box::new(DetHandle {
        ingest: DetectionIngest::new(params),
    }))
}

/// # Safety
/// `h` from di_det_new, not freed before.
#[no_mangle]
pub unsafe extern "C" fn di_det_free(h: *mut DetHandle) {
    if !h.is_null() {
        drop(Box::from_raw(h));
    }
}

/// # Safety
/// `h` a live handle, `k` valid for 9 reads.
#[no_mangle]
pub unsafe extern "C" fn di_det_camera_info(h: *mut DetHandle, k: *const f64) -> i32 {
    let (Some(h), Some(k)) = (h.as_mut(), slice(k, 9)) else {
        return ERR_ARGS;
    };
    let mut kk = [0.0; 9];
    kk.copy_from_slice(k);
    h.ingest.camera_info(&kk);
    0
}

/// # Safety
/// `h` a live handle; the image arguments as for [`image`].
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn di_det_scene_labels(
    h: *mut DetHandle,
    stamp: f64,
    data: *const u8,
    len: i64,
    height: i64,
    width: i64,
    step: i64,
    is_bigendian: i64,
    encoding: *const c_char,
) -> i32 {
    let (Some(h), Some(img)) = (
        h.as_mut(),
        image(data, len, height, width, step, is_bigendian, encoding),
    ) else {
        return ERR_ARGS;
    };
    h.ingest.scene_labels(stamp, &img);
    0
}

/// One synchronized set (see [`DetectionIngest::frame`]). `boxes`: n rows
/// of (cx, cy, w, h). Outputs: `frame_i64` / `frame_f64` (LAYOUT_FRAME_*),
/// `det_i64` n rows of LAYOUT_DET_I64, `det_f32` n depths (NaN when not
/// valid), the decode error text in `err`.
///
/// # Safety
/// `h` a live handle; every pointer valid for its stated length.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn di_det_frame(
    h: *mut DetHandle,
    d_data: *const u8,
    d_len: i64,
    d_height: i64,
    d_width: i64,
    d_step: i64,
    d_big: i64,
    d_enc: *const c_char,
    depth_stamp: f64,
    has_mask: i32,
    m_data: *const u8,
    m_len: i64,
    m_height: i64,
    m_width: i64,
    m_step: i64,
    m_big: i64,
    m_enc: *const c_char,
    boxes: *const f64,
    n: i64,
    frame_i64: *mut i64,
    frame_f64: *mut f64,
    det_i64: *mut i64,
    det_f32: *mut f32,
    err: *mut c_char,
    err_cap: i64,
) -> i32 {
    let Some(h) = h.as_mut() else {
        return ERR_ARGS;
    };
    let Some(depth) = image(d_data, d_len, d_height, d_width, d_step, d_big, d_enc) else {
        return ERR_ARGS;
    };
    let mask = if has_mask != 0 {
        match image(m_data, m_len, m_height, m_width, m_step, m_big, m_enc) {
            Some(m) => Some(m),
            None => return ERR_ARGS,
        }
    } else {
        None
    };
    let nd = count(DET_I64) as i64;
    let (Some(bx), Some(fi), Some(ff), Some(di), Some(df)) = (
        slice(boxes, n.saturating_mul(4)),
        slice_mut(frame_i64, count(FRAME_I64) as i64),
        slice_mut(frame_f64, count(FRAME_F64) as i64),
        slice_mut(det_i64, n.saturating_mul(nd)),
        slice_mut(det_f32, n),
    ) else {
        return ERR_ARGS;
    };
    let boxes: Vec<BoxPx> = bx
        .chunks_exact(4)
        .map(|b| BoxPx {
            cx: b[0],
            cy: b[1],
            w: b[2],
            h: b[3],
        })
        .collect();
    let f = h.ingest.frame(&depth, depth_stamp, mask.as_ref(), &boxes);
    fi.copy_from_slice(&[
        f.depth_status as i64,
        f.mask_status as i64,
        i64::from(f.depth_width),
        i64::from(f.depth_height),
        i64::from(f.mask_width),
        i64::from(f.mask_height),
        i64::from(f.scene_stamp.is_some()),
    ]);
    ff.copy_from_slice(&[
        f.intrinsics[0],
        f.intrinsics[1],
        f.intrinsics[2],
        f.intrinsics[3],
        f.scene_stamp.unwrap_or(f64::NAN),
    ]);
    for k in 0..n as usize {
        let row = &mut di[k * nd as usize..(k + 1) * nd as usize];
        match f.depths.get(k) {
            Some(d) => {
                row.copy_from_slice(&[
                    d.source as i64,
                    i64::from(d.depth.is_some()),
                    i64::from(d.box_px),
                    i64::from(d.mask_px),
                    i64::from(d.other_px),
                ]);
                df[k] = d.depth.unwrap_or(f32::NAN);
            }
            None => {
                row.fill(-1);
                df[k] = f32::NAN;
            }
        }
    }
    write_text(&f.error, err, err_cap);
    0
}

// ---------------------------------------------------------------- sync

pub struct SyncHandle {
    sync: ApproxSync<i64>,
}

/// An ApproximateTimeSynchronizer over `inputs` inputs carrying caller ids.
#[no_mangle]
pub extern "C" fn di_sync_new(inputs: i64, queue_size: i64, slop: f64) -> *mut SyncHandle {
    if !(1..=16).contains(&inputs) || queue_size < 0 || !slop.is_finite() {
        return std::ptr::null_mut();
    }
    Box::into_raw(Box::new(SyncHandle {
        sync: ApproxSync::new(inputs as usize, queue_size as usize, slop),
    }))
}

/// # Safety
/// `h` from di_sync_new, not freed before.
#[no_mangle]
pub unsafe extern "C" fn di_sync_free(h: *mut SyncHandle) {
    if !h.is_null() {
        drop(Box::from_raw(h));
    }
}

/// `add` of message `id` with stamp `stamp_ns` on `input`: 1 and the set's
/// ids (one per input, input order) in `out_ids` when it fires, else 0.
///
/// # Safety
/// `h` a live handle, `out_ids` valid for `inputs` writes.
#[no_mangle]
pub unsafe extern "C" fn di_sync_add(
    h: *mut SyncHandle,
    input: i64,
    stamp_ns: i64,
    id: i64,
    out_ids: *mut i64,
) -> i32 {
    let Some(h) = h.as_mut() else {
        return ERR_ARGS;
    };
    let n = h.sync.inputs() as i64;
    let (true, Some(out)) = ((0..n).contains(&input), slice_mut(out_ids, n)) else {
        return ERR_ARGS;
    };
    match h.sync.add(input as usize, stamp_ns, id) {
        Some(set) => {
            out.copy_from_slice(&set);
            1
        }
        None => 0,
    }
}

// ---------------------------------------------------------------- ROI

pub struct RoiHandle {
    ingest: RoiIngest,
}

/// A front_clearance-side ingest. `ints`: roi_half_width_px,
/// roi_half_height_px, roi_shift_up_px, ground_removal_enabled,
/// ground_fit_stride_px, scene_mask_enabled, enable_scene_labels;
/// `rates`: scene_mask_rate_hz, scene_rate_hz.
///
/// # Safety
/// `ints` valid for 7 reads, `rates` for 2.
#[no_mangle]
pub unsafe extern "C" fn di_roi_new(ints: *const i64, rates: *const f64) -> *mut RoiHandle {
    let (Some(i), Some(r)) = (slice(ints, 7), slice(rates, 2)) else {
        return std::ptr::null_mut();
    };
    Box::into_raw(Box::new(RoiHandle {
        ingest: RoiIngest::new(RoiParams {
            roi_half_width_px: i[0],
            roi_half_height_px: i[1],
            roi_shift_up_px: i[2],
            ground_removal_enabled: i[3] != 0,
            ground_stride: i[4].max(1),
            scene_mask_enabled: i[5] != 0,
            scene_mask_rate_hz: r[0],
            scene_labels_enabled: i[6] != 0,
            scene_rate_hz: r[1],
        }),
    }))
}

/// # Safety
/// `h` from di_roi_new, not freed before.
#[no_mangle]
pub unsafe extern "C" fn di_roi_free(h: *mut RoiHandle) {
    if !h.is_null() {
        drop(Box::from_raw(h));
    }
}

/// One depth frame at node-clock time `now` (see [`RoiIngest::frame`]):
/// `out` (LAYOUT_ROI_I64), the ROI and the floor sample into the two
/// buffers (row-major float32, bit for bit), the decode error in `err`.
/// ERR_CAPACITY when a buffer is too small (nothing written but `out`).
///
/// # Safety
/// `h` a live handle; every pointer valid for its stated length.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn di_roi_frame(
    h: *mut RoiHandle,
    data: *const u8,
    len: i64,
    height: i64,
    width: i64,
    step: i64,
    is_bigendian: i64,
    encoding: *const c_char,
    now: f64,
    out: *mut i64,
    roi_buf: *mut f32,
    roi_cap: i64,
    ground_buf: *mut f32,
    ground_cap: i64,
    err: *mut c_char,
    err_cap: i64,
) -> i32 {
    let Some(h) = h.as_mut() else {
        return ERR_ARGS;
    };
    let (Some(img), Some(o), Some(rb), Some(gb)) = (
        image(data, len, height, width, step, is_bigendian, encoding),
        slice_mut(out, count(ROI_I64) as i64),
        slice_mut(roi_buf, roi_cap),
        slice_mut(ground_buf, ground_cap),
    ) else {
        return ERR_ARGS;
    };
    match h.ingest.frame(&img, now) {
        RoiOutcome::DecodeFailed(e) => {
            o.fill(0);
            o[0] = 1;
            write_text(&e, err, err_cap);
            0
        }
        RoiOutcome::Frame(f) => {
            let (y0, y1, x0, x1) = f.bounds;
            o.copy_from_slice(&[
                0,
                i64::from(f.width),
                i64::from(f.height),
                y0,
                y1,
                x0,
                x1,
                f.roi.len() as i64,
                i64::from(f.ground_stride),
                f.ground_rows as i64,
                f.ground_cols as i64,
                i64::from(f.scene_mask_tick),
                i64::from(f.scene_labels_tick),
            ]);
            if f.roi.len() > rb.len() || f.ground.len() > gb.len() {
                return ERR_CAPACITY;
            }
            rb[..f.roi.len()].copy_from_slice(&f.roi);
            gb[..f.ground.len()].copy_from_slice(&f.ground);
            write_text("", err, err_cap);
            0
        }
    }
}
