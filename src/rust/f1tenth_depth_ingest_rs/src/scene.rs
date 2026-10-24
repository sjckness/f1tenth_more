//! detection_3d_node's scene-label buffer: `_scene_labels_callback` (append,
//! then drop from the front while the NEW stamp is more than
//! SCENE_LABELS_BUFFER_S past the oldest) and the pairing in `_scene_other`
//! (the entry with the smallest |depth_stamp - stamp|, the first on ties as
//! Python's `min` keeps it, refused beyond scene_labels_max_age_s).
//!
//! Stamps are `sec + nanosec * 1e-9` in float64, as the Python computes them.

use std::collections::VecDeque;

use crate::image::{DecodeError, ImageParts, MonoImage};

/// detection_3d_node.SCENE_LABELS_BUFFER_S.
pub const SCENE_LABELS_BUFFER_S: f64 = 0.6;

/// scene_labels.LABEL_OTHER.
pub const LABEL_OTHER: u8 = 5;

/// `msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9`.
#[inline]
pub fn stamp_s(sec: i32, nanosec: u32) -> f64 {
    f64::from(sec) + f64::from(nanosec) * 1e-9
}

/// One buffered label image. `None`: the message did not decode to an image
/// (cv_bridge returns None for a zero-size one, which the Python stores and
/// would raise on if it were ever the nearest; here it pairs as "no labels").
type Entry = (f64, Option<MonoImage<'static>>);

#[derive(Default)]
pub struct SceneLabels {
    entries: VecDeque<Entry>,
}

impl SceneLabels {
    pub fn new() -> Self {
        SceneLabels::default()
    }

    /// `_scene_labels_callback`. A label image of an encoding cv_bridge
    /// would refuse is dropped (the Python returns on CvBridgeError); see
    /// image.rs for the encodings accepted.
    pub fn push(&mut self, stamp: f64, img: &ImageParts<'_>) {
        let decoded = match MonoImage::decode(img) {
            Ok(m) => Some(MonoImage::owned(m.height, m.width, copy_rows(&m))),
            Err(DecodeError::Empty) => None,
            Err(DecodeError::Bad(_)) => return,
        };
        self.entries.push_back((stamp, decoded));
        while let Some(front) = self.entries.front() {
            if stamp - front.0 > SCENE_LABELS_BUFFER_S {
                self.entries.pop_front();
            } else {
                break;
            }
        }
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The labels `_scene_other` pairs with a depth frame, with their stamp;
    /// None without fresh, usable labels.
    pub fn nearest(&self, depth_stamp: f64, max_age: f64) -> Option<(f64, &MonoImage<'static>)> {
        let mut best: Option<&Entry> = None;
        let mut best_key = 0.0_f64;
        for e in &self.entries {
            let key = (depth_stamp - e.0).abs();
            // Python's min(key=...): the first entry, replaced only by a
            // strictly smaller key (a NaN key is never replaced, never replaces).
            if best.is_none() || key < best_key {
                best = Some(e);
                best_key = key;
            }
        }
        let e = best?;
        if (depth_stamp - e.0).abs() > max_age {
            return None;
        }
        e.1.as_ref().map(|img| (e.0, img))
    }
}

fn copy_rows(m: &MonoImage<'_>) -> Vec<u8> {
    let mut out = Vec::with_capacity(m.height * m.width);
    for r in 0..m.height {
        out.extend_from_slice(m.row(r));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn img(value: u8) -> Vec<u8> {
        vec![value; 9 * 16]
    }

    fn parts(data: &[u8]) -> ImageParts<'_> {
        ImageParts {
            height: 9,
            width: 16,
            encoding: "mono8",
            is_bigendian: 0,
            step: 16,
            data,
        }
    }

    #[test]
    fn nearest_by_stamp_not_newest() {
        let mut s = SceneLabels::new();
        let other = img(LABEL_OTHER);
        let ground = img(1);
        s.push(100.0, &parts(&other));
        s.push(100.2, &parts(&ground));
        assert_eq!(s.nearest(100.02, 0.3).map(|x| x.1.get(0, 0)), Some(LABEL_OTHER));
        assert_eq!(s.nearest(100.19, 0.3).map(|x| x.1.get(0, 0)), Some(1));
        assert!(s.nearest(101.0, 0.3).is_none());
    }

    #[test]
    fn the_new_stamp_trims_the_front() {
        let mut s = SceneLabels::new();
        let a = img(1);
        for k in 0..10 {
            s.push(10.0 + 0.25 * f64::from(k), &parts(&a));
        }
        // Newest 12.25: 11.75, 12.0 and 12.25 are within 0.6 of it.
        assert_eq!(s.len(), 3);
        // An older stamp arriving late trims nothing.
        s.push(11.0, &parts(&a));
        assert_eq!(s.len(), 4);
    }

    #[test]
    fn ties_keep_the_first() {
        let mut s = SceneLabels::new();
        let a = img(1);
        let b = img(2);
        s.push(1.0, &parts(&a));
        s.push(1.5, &parts(&b));
        assert_eq!(s.nearest(1.25, 0.3).map(|x| x.1.get(0, 0)), Some(1));
    }
}
