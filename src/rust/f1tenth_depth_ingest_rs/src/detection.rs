//! detection_3d_node's depth sampling per detection (`_median_depth`,
//! `_mask_median_depth`, `_scene_other`), operation for operation; the
//! choice of path is reported as a source code so the node books its
//! counters and warnings exactly as before (DetectionDepth.msg).

use crate::image::{valid_depth, DepthView, MonoImage};
use crate::median::median_f32;
use crate::scene::LABEL_OTHER;

/// DetectionDepth.SOURCE_* (same numbers).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Source {
    Box = 0,
    Mask = 1,
    SceneOther = 2,
    SceneFallback = 3,
    MaskEmpty = 4,
    LabelOverflow = 5,
    InvalidBox = 6,
}

/// One detection's result (DetectionDepth without the message).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DetDepth {
    pub source: Source,
    /// `None`: the Python's None (no region or no valid pixel).
    pub depth: Option<f32>,
    pub box_px: u32,
    pub mask_px: u32,
    pub other_px: u32,
}

/// A Detection2D's bbox in pixels: centre and size, as the message carries them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoxPx {
    pub cx: f64,
    pub cy: f64,
    pub w: f64,
    pub h: f64,
}

/// `np.clip(v, lo, hi)` for a float and int bounds (NaN propagates).
#[inline]
fn np_clip(v: f64, lo: f64, hi: f64) -> f64 {
    if v.is_nan() {
        return v;
    }
    let v = if v < lo { lo } else { v };
    if v > hi {
        hi
    } else {
        v
    }
}

/// The box region `_median_depth` reads, `(x1, x2, y1, y2)`; `Ok(None)` for a
/// degenerate one, `Err` for a NaN coordinate (Python's int() raises).
pub fn box_region(
    b: &BoxPx,
    center_fraction: f64,
    img_w: usize,
    img_h: usize,
) -> Result<Option<(usize, usize, usize, usize)>, ()> {
    let half_w = (b.w * center_fraction) / 2.0;
    let half_h = (b.h * center_fraction) / 2.0;
    let wmax = img_w as f64 - 1.0;
    let hmax = img_h as f64 - 1.0;
    let coords = [
        np_clip(b.cx - half_w, 0.0, wmax),
        np_clip(b.cx + half_w, 0.0, wmax),
        np_clip(b.cy - half_h, 0.0, hmax),
        np_clip(b.cy + half_h, 0.0, hmax),
    ];
    if coords.iter().any(|v| v.is_nan()) {
        return Err(());
    }
    // int() truncates; every value is in [0, size - 1] here.
    let [x1, x2, y1, y2] = coords.map(|v| v as usize);
    if x2 <= x1 || y2 <= y1 {
        return Ok(None);
    }
    Ok(Some((x1, x2, y1, y2)))
}

/// `_median_depth`: (median, valid pixel count), `Err` for a NaN box.
pub fn box_median(
    depth: &DepthView<'_>,
    b: &BoxPx,
    center_fraction: f64,
    scratch: &mut Vec<f32>,
) -> Result<(Option<f32>, u32), ()> {
    let Some((x1, x2, y1, y2)) = box_region(b, center_fraction, depth.width, depth.height)? else {
        return Ok((None, 0));
    };
    scratch.clear();
    for r in y1..y2 {
        for c in x1..x2 {
            let d = depth.get(r, c);
            if valid_depth(d) {
                scratch.push(d);
            }
        }
    }
    let n = scratch.len() as u32;
    Ok((median_f32(scratch), n))
}

/// The settings `_mask_median_depth` and `_scene_other` read.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SampleParams {
    pub center_fraction: f64,
    pub use_mask_depth: bool,
    pub scene_enabled: bool,
    pub scene_min_other_px: i64,
}

/// Per frame: the valid in-mask pixels of every label, found in one pass
/// over the frame (the Python compares the whole frame per detection; the
/// pixel sets are the same).
pub struct MaskIndex {
    /// For each label 1..=255: (row, col, depth) of its valid pixels, row-major.
    by_label: Vec<Vec<(u32, u32, f32)>>,
}

impl MaskIndex {
    /// `mask` must have the depth's shape (resized by the caller).
    pub fn build(depth: &DepthView<'_>, mask: &MonoImage<'_>, labels_needed: usize) -> Self {
        let n = labels_needed.min(255);
        let mut by_label: Vec<Vec<(u32, u32, f32)>> = vec![Vec::new(); n + 1];
        if n > 0 {
            for r in 0..depth.height {
                let row = mask.row(r);
                for (c, &m) in row.iter().enumerate() {
                    let m = usize::from(m);
                    if m == 0 || m > n {
                        continue;
                    }
                    let d = depth.get(r, c);
                    if valid_depth(d) {
                        by_label[m].push((r as u32, c as u32, d));
                    }
                }
            }
        }
        MaskIndex { by_label }
    }

    fn pixels(&self, label: usize) -> &[(u32, u32, f32)] {
        self.by_label.get(label).map(Vec::as_slice).unwrap_or(&[])
    }
}

/// The scene labels paired with this frame (None: off, missing or stale).
pub type SceneSel<'a> = Option<&'a MonoImage<'static>>;

/// One detection, index `i` of the Detection2DArray (mask label i + 1).
pub fn sample(
    depth: &DepthView<'_>,
    mask: Option<&MaskIndex>,
    scene: SceneSel<'_>,
    i: usize,
    b: &BoxPx,
    p: &SampleParams,
    scratch: &mut Vec<f32>,
) -> DetDepth {
    let boxed = |source: Source, scratch: &mut Vec<f32>, mask_px: u32| -> DetDepth {
        match box_median(depth, b, p.center_fraction, scratch) {
            Ok((z, n)) => DetDepth {
                source,
                depth: z,
                box_px: n,
                mask_px,
                other_px: 0,
            },
            Err(()) => DetDepth {
                source: Source::InvalidBox,
                depth: None,
                box_px: 0,
                mask_px,
                other_px: 0,
            },
        }
    };
    let Some(index) = mask.filter(|_| p.use_mask_depth) else {
        return boxed(Source::Box, scratch, 0);
    };
    let label = i + 1;
    if label > 255 {
        return boxed(Source::LabelOverflow, scratch, 0);
    }
    let px = index.pixels(label);
    if px.is_empty() {
        return boxed(Source::MaskEmpty, scratch, 0);
    }
    let mask_px = px.len() as u32;
    let mask_median = |scratch: &mut Vec<f32>| {
        scratch.clear();
        scratch.extend(px.iter().map(|e| e.2));
        median_f32(scratch)
    };
    if p.scene_enabled {
        if let Some(labels) = scene {
            let (gh, gw) = (labels.height, labels.width);
            let (h, w) = (depth.height, depth.width);
            scratch.clear();
            for &(r, c, d) in px {
                let lr = r as usize * gh / h;
                let lc = c as usize * gw / w;
                if labels.get(lr, lc) == LABEL_OTHER {
                    scratch.push(d);
                }
            }
            let other_px = scratch.len() as u32;
            if i64::from(other_px) >= p.scene_min_other_px {
                // np.median of an empty array is NaN (scene_min_other_px <= 0).
                let z = median_f32(scratch).or(Some(f32::NAN));
                return DetDepth {
                    source: Source::SceneOther,
                    depth: z,
                    box_px: 0,
                    mask_px,
                    other_px,
                };
            }
            return DetDepth {
                source: Source::SceneFallback,
                depth: mask_median(scratch),
                box_px: 0,
                mask_px,
                other_px,
            };
        }
        return DetDepth {
            source: Source::SceneFallback,
            depth: mask_median(scratch),
            box_px: 0,
            mask_px,
            other_px: 0,
        };
    }
    DetDepth {
        source: Source::Mask,
        depth: mask_median(scratch),
        box_px: 0,
        mask_px,
        other_px: 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image::ImageParts;

    fn depth_bytes(v: &[f32]) -> Vec<u8> {
        v.iter().flat_map(|x| x.to_le_bytes()).collect()
    }

    fn view<'a>(h: u32, w: u32, data: &'a [u8]) -> DepthView<'a> {
        DepthView::decode(&ImageParts {
            height: h,
            width: w,
            encoding: "32FC1",
            is_bigendian: 0,
            step: 4 * w,
            data,
        })
        .unwrap()
    }

    fn params(mask: bool, scene: bool) -> SampleParams {
        SampleParams {
            center_fraction: 0.5,
            use_mask_depth: mask,
            scene_enabled: scene,
            scene_min_other_px: 2,
        }
    }

    #[test]
    fn the_box_is_the_clipped_centre_half() {
        // 20 x 20 of 4.0 with a NaN, a 0 and a -inf inside the centre.
        let mut v = vec![4.0_f32; 400];
        v[9 * 20 + 9] = f32::NAN;
        v[9 * 20 + 10] = 0.0;
        v[10 * 20 + 9] = f32::NEG_INFINITY;
        v[10 * 20 + 10] = 2.0;
        // Outside the region (x2, y2 are exclusive).
        v[11 * 20 + 11] = 0.5;
        let data = depth_bytes(&v);
        let d = view(20, 20, &data);
        let b = BoxPx {
            cx: 10.0,
            cy: 10.0,
            w: 6.0,
            h: 6.0,
        };
        // x1 = int(10 - 1.5) = 8, x2 = int(11.5) = 11: 3 x 3 = 9 px, 3 invalid.
        assert_eq!(box_region(&b, 0.5, 20, 20), Ok(Some((8, 11, 8, 11))));
        let mut s = Vec::new();
        let r = sample(&d, None, None, 0, &b, &params(false, false), &mut s);
        assert_eq!(r.source, Source::Box);
        assert_eq!(r.box_px, 6);
        // {4, 4, 4, 4, 4, 2}: median (4 + 4) / 2.
        assert_eq!(r.depth, Some(4.0));
        let nan = BoxPx { cx: f64::NAN, ..b };
        assert_eq!(
            sample(&d, None, None, 0, &nan, &params(false, false), &mut s).source,
            Source::InvalidBox
        );
        // int(10.4) == int(10.6): x2 <= x1.
        let tiny = BoxPx { cx: 10.5, w: 0.4, ..b };
        assert_eq!(
            sample(&d, None, None, 0, &tiny, &params(false, false), &mut s).depth,
            None
        );
    }

    #[test]
    fn mask_paths() {
        let mut v = vec![9.0_f32; 100];
        let mut m = vec![0_u8; 100];
        for r in 2..4 {
            for c in 2..5 {
                v[r * 10 + c] = 1.0;
                m[r * 10 + c] = 1;
            }
        }
        v[3 * 10 + 4] = 9.0;
        // Label 2 has only invalid depth.
        m[7 * 10 + 7] = 2;
        v[7 * 10 + 7] = f32::NAN;
        let data = depth_bytes(&v);
        let d = view(10, 10, &data);
        let mask = MonoImage::owned(10, 10, m);
        let idx = MaskIndex::build(&d, &mask, 3);
        let b = BoxPx {
            cx: 5.0,
            cy: 5.0,
            w: 8.0,
            h: 8.0,
        };
        let mut s = Vec::new();
        let r = sample(&d, Some(&idx), None, 0, &b, &params(true, false), &mut s);
        assert_eq!((r.source, r.mask_px, r.depth), (Source::Mask, 6, Some(1.0)));
        let r = sample(&d, Some(&idx), None, 1, &b, &params(true, false), &mut s);
        assert_eq!(r.source, Source::MaskEmpty);
        assert_eq!(r.depth, Some(9.0));
        let r = sample(&d, Some(&idx), None, 255, &b, &params(true, false), &mut s);
        assert_eq!(r.source, Source::LabelOverflow);
        // use_mask_depth false ignores the mask.
        let r = sample(&d, Some(&idx), None, 0, &b, &params(false, false), &mut s);
        assert_eq!(r.source, Source::Box);

        // Scene labels 5 x 5: OTHER only in the top-left quarter.
        let mut lab = vec![1_u8; 25];
        for r in 0..2 {
            for c in 0..2 {
                lab[r * 5 + c] = LABEL_OTHER;
            }
        }
        let labels = MonoImage::owned(5, 5, lab);
        let r = sample(&d, Some(&idx), Some(&labels), 0, &b, &params(true, true), &mut s);
        // In-mask px (2..4, 2..5) map to label rows 1, cols 1..2: OTHER at
        // cols < 2 -> (2,2), (2,3), (3,2), (3,3) = 4 px, all 1.0.
        assert_eq!(
            (r.source, r.other_px, r.depth),
            (Source::SceneOther, 4, Some(1.0))
        );
        let r = sample(&d, Some(&idx), None, 0, &b, &params(true, true), &mut s);
        assert_eq!((r.source, r.depth), (Source::SceneFallback, Some(1.0)));
    }
}
