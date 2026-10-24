//! The ingest node's state without ROS: what it does with each message.
//! main.rs (the rclrs node) and ffi.rs (the in-process path for
//! f1tenth_perception's tests and parity harness) both drive these, so the
//! two cannot differ.
//!
//! [`DetectionIngest`] is detection_3d_node's depth side: the camera_info
//! cache, the scene-label buffer, and per synchronized set the order of the
//! Python's checks (camera_info, depth decode, mask decode) and the sampling
//! of every detection. [`RoiIngest`] is front_clearance_node's: the frame
//! decode, the ROI and floor sample, the two tick gates.

use crate::detection::{sample, BoxPx, DetDepth, MaskIndex, SampleParams};
use crate::image::{DecodeError, DepthView, ImageParts, MonoImage};
use crate::roi::{extract, RoiFrame, RoiParams, Ticks};
use crate::scene::SceneLabels;

/// DetectionDepthArray.DEPTH_*.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum DepthStatus {
    Ok = 0,
    NoCameraInfo = 1,
    DecodeFailed = 2,
}

/// DetectionDepthArray.MASK_*.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MaskStatus {
    None = 0,
    Empty = 1,
    Ok = 2,
    Resized = 3,
    DecodeFailed = 4,
}

/// detection_3d_node's parameters the ingest side reads (its names).
#[derive(Clone, Debug, PartialEq)]
pub struct DetectionParams {
    pub sample: SampleParams,
    pub scene_labels_max_age_s: f64,
    pub sync_slop: f64,
    pub sync_queue_size: i64,
}

impl DetectionParams {
    /// Startup refusals: what makes the Python synchronizer raise on its
    /// first message (a negative queue size empties the queue and then
    /// calls min() on it; a NaN slop fails int()).
    pub fn validate(&self) -> Result<(), String> {
        let mut errs = Vec::new();
        if self.sync_queue_size < 0 {
            errs.push(format!(
                "sync_queue_size {} < 0 (message_filters would raise on the first message)",
                self.sync_queue_size
            ));
        }
        if !self.sync_slop.is_finite() {
            errs.push(format!("sync_slop {} is not finite", self.sync_slop));
        }
        if errs.is_empty() {
            Ok(())
        } else {
            Err(errs.join("; "))
        }
    }
}

/// One synchronized set's result (DetectionDepthArray without the headers).
#[derive(Clone, Debug)]
pub struct DetectionFrame {
    pub depth_status: DepthStatus,
    pub mask_status: MaskStatus,
    pub error: String,
    pub depth_width: u32,
    pub depth_height: u32,
    /// fx, fy, cx, cy (zeros with NoCameraInfo).
    pub intrinsics: [f64; 4],
    pub mask_width: u32,
    pub mask_height: u32,
    pub scene_stamp: Option<f64>,
    pub depths: Vec<DetDepth>,
}

pub struct DetectionIngest {
    pub params: DetectionParams,
    camera: Option<[f64; 4]>,
    scene: SceneLabels,
    scratch: Vec<f32>,
}

impl DetectionIngest {
    pub fn new(params: DetectionParams) -> Self {
        DetectionIngest {
            params,
            camera: None,
            scene: SceneLabels::new(),
            scratch: Vec::new(),
        }
    }

    /// `_camera_info_callback`: k[0], k[4], k[2], k[5], cached as received.
    pub fn camera_info(&mut self, k: &[f64; 9]) {
        self.camera = Some([k[0], k[4], k[2], k[5]]);
    }

    /// `_scene_labels_callback` (only subscribed with scene labels on).
    pub fn scene_labels(&mut self, stamp: f64, img: &ImageParts<'_>) {
        self.scene.push(stamp, img);
    }

    pub fn scene_buffered(&self) -> usize {
        self.scene.len()
    }

    /// One synchronized set: `_synced_callback`'s checks in its order, then
    /// every detection sampled. `mask`: None for a 2-way set.
    pub fn frame(
        &mut self,
        depth: &ImageParts<'_>,
        depth_stamp: f64,
        mask: Option<&ImageParts<'_>>,
        boxes: &[BoxPx],
    ) -> DetectionFrame {
        let mut out = DetectionFrame {
            depth_status: DepthStatus::Ok,
            mask_status: MaskStatus::None,
            error: String::new(),
            depth_width: 0,
            depth_height: 0,
            intrinsics: [0.0; 4],
            mask_width: mask.map_or(0, |m| m.width),
            mask_height: mask.map_or(0, |m| m.height),
            scene_stamp: None,
            depths: Vec::new(),
        };
        let Some(k) = self.camera else {
            out.depth_status = DepthStatus::NoCameraInfo;
            return out;
        };
        out.intrinsics = k;
        let view = match DepthView::decode(depth) {
            Ok(v) => v,
            Err(e) => {
                out.depth_status = DepthStatus::DecodeFailed;
                out.error = e.text();
                return out;
            }
        };
        out.depth_width = view.width as u32;
        out.depth_height = view.height as u32;

        // The mask, as the Python decodes it: zero-size is the "no
        // instances" marker; anything undecodable leaves every detection on
        // the box region.
        let mut mask_img: Option<MonoImage<'_>> = None;
        if let Some(m) = mask {
            if m.height == 0 || m.width == 0 {
                out.mask_status = MaskStatus::Empty;
            } else {
                match MonoImage::decode(m) {
                    Ok(img) => {
                        if img.height != view.height || img.width != view.width {
                            out.mask_status = MaskStatus::Resized;
                            mask_img = Some(img.resize_nearest(view.height, view.width));
                        } else {
                            out.mask_status = MaskStatus::Ok;
                            mask_img = Some(img);
                        }
                    }
                    Err(e) => {
                        out.mask_status = MaskStatus::DecodeFailed;
                        out.error = match e {
                            DecodeError::Empty => "zero-size mask".to_owned(),
                            DecodeError::Bad(s) => s,
                        };
                    }
                }
            }
        }

        let p = self.params.sample;
        let index = match (&mask_img, p.use_mask_depth) {
            (Some(img), true) => Some(MaskIndex::build(&view, img, boxes.len())),
            _ => None,
        };
        let scene = if p.scene_enabled && index.is_some() {
            self.scene
                .nearest(depth_stamp, self.params.scene_labels_max_age_s)
        } else {
            None
        };
        out.scene_stamp = scene.map(|s| s.0);
        let scene_img = scene.map(|s| s.1);
        let scratch = &mut self.scratch;
        out.depths = boxes
            .iter()
            .enumerate()
            .map(|(i, b)| sample(&view, index.as_ref(), scene_img, i, b, &p, scratch))
            .collect();
        out
    }
}

/// One depth frame's DepthRoi content, or why it was not decoded.
pub enum RoiOutcome {
    Frame(RoiFrame),
    DecodeFailed(String),
}

pub struct RoiIngest {
    pub params: RoiParams,
    ticks: Ticks,
}

impl RoiIngest {
    pub fn new(params: RoiParams) -> Self {
        let ticks = Ticks::new(&params);
        RoiIngest { params, ticks }
    }

    /// `depth_callback` up to the statistics: decode (a failure returns
    /// before the tick gates, as the Python returns before reading its
    /// clock), the pixels, the gates at `now` (the node clock, seconds).
    pub fn frame(&mut self, depth: &ImageParts<'_>, now: f64) -> RoiOutcome {
        let view = match DepthView::decode(depth) {
            Ok(v) => v,
            Err(e) => return RoiOutcome::DecodeFailed(e.text()),
        };
        let mut f = extract(&view, &self.params);
        let (mask, labels) = self.ticks.due(&self.params, now);
        f.scene_mask_tick = mask;
        f.scene_labels_tick = labels;
        RoiOutcome::Frame(f)
    }
}
