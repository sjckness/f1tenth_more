//! front_clearance_node's per-frame depth reads (output/rust/depth_ingest_design.md 1.1):
//! the ROI `depth[y0:y1, x0:x1]`, the floor-fit sample
//! `depth[np.mgrid[y0:h:stride, x0:x1:stride]]` and the two 5 Hz tick gates
//! (`now >= next; next = now + period`, period `1 / rate` or inf for a rate
//! <= 0). The statistics stay in the node; this delivers their pixels.

use crate::image::DepthView;

/// The node parameters these depend on (front_clearance_node's names).
#[derive(Clone, Debug, PartialEq)]
pub struct RoiParams {
    pub roi_half_width_px: i64,
    pub roi_half_height_px: i64,
    pub roi_shift_up_px: i64,
    pub ground_removal_enabled: bool,
    /// `max(1, ground_fit_stride_px)`, as the node stores it.
    pub ground_stride: i64,
    pub scene_mask_enabled: bool,
    pub scene_mask_rate_hz: f64,
    pub scene_labels_enabled: bool,
    pub scene_rate_hz: f64,
}

/// `(y0, y1, x0, x1)` as depth_callback computes them (Python ints).
pub fn bounds(h: i64, w: i64, p: &RoiParams) -> (i64, i64, i64, i64) {
    let cy = h.div_euclid(2) - p.roi_shift_up_px;
    let cx = w.div_euclid(2);
    let y0 = (cy - p.roi_half_height_px).max(0);
    let y1 = (cy + p.roi_half_height_px).min(h);
    let x0 = (cx - p.roi_half_width_px).max(0);
    let x1 = (cx + p.roi_half_width_px).min(w);
    (y0, y1, x0, x1)
}

/// The index range a Python slice `a[start:stop]` selects in a length-`len`
/// axis (negative indices count from the end; empty when stop <= start).
pub fn py_slice(start: i64, stop: i64, len: i64) -> (usize, usize) {
    let norm = |i: i64| -> i64 {
        let j = if i < 0 { i + len } else { i };
        j.clamp(0, len)
    };
    let a = norm(start);
    let b = norm(stop).max(a);
    (a as usize, b as usize)
}

/// `np.arange(start, stop, step)` of non-negative ints (start >= 0, step >= 1).
pub fn arange(start: i64, stop: i64, step: i64) -> Vec<usize> {
    if start >= stop || step < 1 {
        return Vec::new();
    }
    (start..stop).step_by(step as usize).map(|i| i as usize).collect()
}

fn period(rate_hz: f64) -> f64 {
    if rate_hz > 0.0 {
        1.0 / rate_hz
    } else {
        f64::INFINITY
    }
}

/// The two schedules, as the node keeps `_next_scene_mask` / `_next_scene`.
#[derive(Clone, Debug)]
pub struct Ticks {
    next_mask: f64,
    next_labels: f64,
    mask_period: f64,
    labels_period: f64,
}

impl Ticks {
    pub fn new(p: &RoiParams) -> Self {
        Ticks {
            next_mask: 0.0,
            next_labels: 0.0,
            mask_period: period(p.scene_mask_rate_hz),
            labels_period: period(p.scene_rate_hz),
        }
    }

    /// (mask tick, labels tick) for a frame decoded at `now`.
    pub fn due(&mut self, p: &RoiParams, now: f64) -> (bool, bool) {
        let mask = p.scene_mask_enabled && now >= self.next_mask;
        if mask {
            self.next_mask = now + self.mask_period;
        }
        let labels = p.scene_labels_enabled && now >= self.next_labels;
        if labels {
            self.next_labels = now + self.labels_period;
        }
        (mask, labels)
    }
}

/// One frame's delivery (DepthRoi without the header and the frame copy).
#[derive(Clone, Debug, Default)]
pub struct RoiFrame {
    pub width: u32,
    pub height: u32,
    pub bounds: (i64, i64, i64, i64),
    pub roi: Vec<f32>,
    /// 0 when ground removal is off (then `ground` is empty).
    pub ground_stride: u32,
    pub ground: Vec<f32>,
    pub ground_rows: usize,
    pub ground_cols: usize,
    pub scene_mask_tick: bool,
    pub scene_labels_tick: bool,
}

/// The pixels of one decoded depth frame (the tick flags are filled by the caller).
pub fn extract(depth: &DepthView<'_>, p: &RoiParams) -> RoiFrame {
    let h = depth.height as i64;
    let w = depth.width as i64;
    let (y0, y1, x0, x1) = bounds(h, w, p);
    let (ra, rb) = py_slice(y0, y1, h);
    let (ca, cb) = py_slice(x0, x1, w);
    let mut roi = Vec::with_capacity((rb - ra) * (cb - ca));
    for r in ra..rb {
        depth.extend_row(r, ca, cb, &mut roi);
    }
    let mut out = RoiFrame {
        width: depth.width as u32,
        height: depth.height as u32,
        bounds: (y0, y1, x0, x1),
        roi,
        ..RoiFrame::default()
    };
    if p.ground_removal_enabled {
        let stride = p.ground_stride.max(1);
        let rows = arange(y0, h, stride);
        let cols = arange(x0, x1, stride);
        let mut g = Vec::with_capacity(rows.len() * cols.len());
        for &r in &rows {
            g.extend(cols.iter().map(|&c| depth.get(r, c)));
        }
        out.ground_stride = stride as u32;
        out.ground_rows = rows.len();
        out.ground_cols = cols.len();
        out.ground = g;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image::ImageParts;

    fn params() -> RoiParams {
        RoiParams {
            roi_half_width_px: 180,
            roi_half_height_px: 70,
            roi_shift_up_px: 35,
            ground_removal_enabled: true,
            ground_stride: 4,
            scene_mask_enabled: true,
            scene_mask_rate_hz: 5.0,
            scene_labels_enabled: true,
            scene_rate_hz: 5.0,
        }
    }

    #[test]
    fn shipped_bounds_on_the_zed_frame() {
        assert_eq!(bounds(360, 640, &params()), (75, 215, 140, 500));
    }

    #[test]
    fn python_slices() {
        assert_eq!(py_slice(2, 5, 10), (2, 5));
        assert_eq!(py_slice(0, -3, 10), (0, 7));
        assert_eq!(py_slice(8, 4, 10), (8, 8));
        assert_eq!(py_slice(12, 20, 10), (10, 10));
        assert_eq!(arange(75, 360, 4).len(), 72);
        assert_eq!(arange(140, 500, 4).len(), 90);
        assert!(arange(5, 5, 4).is_empty());
    }

    #[test]
    fn ticks_follow_the_node_gate() {
        let mut p = params();
        p.scene_mask_rate_hz = 4.0;
        p.scene_rate_hz = 8.0;
        let mut t = Ticks::new(&p);
        assert_eq!(t.due(&p, 100.0), (true, true));
        assert_eq!(t.due(&p, 100.125), (false, true));
        assert_eq!(t.due(&p, 100.1875), (false, false));
        assert_eq!(t.due(&p, 100.25), (true, true));
        let mut off = params();
        off.scene_mask_rate_hz = 0.0;
        let mut t = Ticks::new(&off);
        assert_eq!(t.due(&off, 1.0), (true, true));
        assert_eq!(t.due(&off, 1e9), (false, true));
    }

    #[test]
    fn extract_is_bit_for_bit() {
        let (h, w) = (12_u32, 16_u32);
        let mut data = Vec::new();
        for i in 0..(h * w) {
            data.extend_from_slice(&f32::from_bits(0x3f80_0000 + i).to_le_bytes());
        }
        let parts = ImageParts {
            height: h,
            width: w,
            encoding: "32FC1",
            is_bigendian: 0,
            step: 4 * w,
            data: &data,
        };
        let d = DepthView::decode(&parts).unwrap();
        let mut p = params();
        p.roi_half_width_px = 3;
        p.roi_half_height_px = 2;
        p.roi_shift_up_px = 1;
        p.ground_stride = 3;
        let f = extract(&d, &p);
        assert_eq!(f.bounds, (3, 7, 5, 11));
        assert_eq!(f.roi.len(), 4 * 6);
        assert_eq!(f.roi[0].to_bits(), 0x3f80_0000 + 3 * w + 5);
        assert_eq!((f.ground_rows, f.ground_cols), (3, 2));
        assert_eq!(f.ground[2].to_bits(), 0x3f80_0000 + 6 * w + 5);
    }
}
