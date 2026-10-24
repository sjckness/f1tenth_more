//! sensor_msgs/Image decoding as cv_bridge's `imgmsg_to_cv2` does it for the
//! two encodings the depth consumers read: `32FC1` depth (desired `32FC1`)
//! and `mono8` / `8UC1` label images (desired `mono8`).
//!
//! cv_bridge views the data as `(height, step / itemsize)` rows of the
//! message's byte order, keeps the first `width` columns and byte-swaps to
//! the host order; `32FC1 -> 32FC1` and `mono8 -> mono8` are then identities.
//! The views here read the same bytes in place (bit for bit, NaN payloads
//! included), without the copy.
//!
//! What cv_bridge accepts and these refuse (documented fail-safe
//! differences, output/rust/parity_depth_ingest.md, none reachable with the
//! ZED or the YOLO node): other encodings it would convert (a colour mask to
//! grey, `16UC1` depth to float), and malformed images it turns into a crash
//! of the consumer's callback (a buffer shorter than `height * step`:
//! numpy's TypeError) or into a narrower image (`step < 4 * width`). A
//! zero-size depth image is refused too (cv_bridge returns None there and the
//! raw consumer then raises).

/// The fields of a sensor_msgs/Image the decoders read.
#[derive(Clone, Copy, Debug)]
pub struct ImageParts<'a> {
    pub height: u32,
    pub width: u32,
    pub encoding: &'a str,
    pub is_bigendian: u8,
    pub step: u32,
    pub data: &'a [u8],
}

/// Why an image is not decoded.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// Zero rows or columns.
    Empty,
    /// Anything else, with the reason.
    Bad(String),
}

impl DecodeError {
    pub fn text(&self) -> String {
        match self {
            DecodeError::Empty => "zero-size image".to_owned(),
            DecodeError::Bad(s) => s.clone(),
        }
    }
}

fn check_layout(p: &ImageParts<'_>, itemsize: usize) -> Result<(usize, usize, usize), DecodeError> {
    let h = p.height as usize;
    let w = p.width as usize;
    let step = p.step as usize;
    if h == 0 || w == 0 {
        return Err(DecodeError::Empty);
    }
    if step % itemsize != 0 || step < w * itemsize {
        return Err(DecodeError::Bad(format!(
            "{} image {}x{}: step {} does not hold {} columns of {} bytes",
            p.encoding, w, h, step, w, itemsize
        )));
    }
    let needed = h.checked_mul(step).unwrap_or(usize::MAX);
    if p.data.len() < needed {
        return Err(DecodeError::Bad(format!(
            "{} image {}x{}, step {}: {} data bytes, {} needed",
            p.encoding,
            w,
            h,
            step,
            p.data.len(),
            needed
        )));
    }
    Ok((h, w, step))
}

/// A decoded `32FC1` depth image (metres), read in place.
#[derive(Clone, Copy, Debug)]
pub struct DepthView<'a> {
    pub height: usize,
    pub width: usize,
    step: usize,
    big_endian: bool,
    data: &'a [u8],
}

impl<'a> DepthView<'a> {
    pub fn decode(p: &ImageParts<'a>) -> Result<Self, DecodeError> {
        if p.encoding != "32FC1" {
            return Err(DecodeError::Bad(format!(
                "depth encoding {:?}: only 32FC1 is decoded",
                p.encoding
            )));
        }
        let (height, width, step) = check_layout(p, 4)?;
        Ok(DepthView {
            height,
            width,
            step,
            big_endian: p.is_bigendian != 0,
            data: p.data,
        })
    }

    /// The pixel's float32 bits (host order).
    #[inline]
    pub fn bits(&self, r: usize, c: usize) -> u32 {
        let i = r * self.step + 4 * c;
        let b = [
            self.data[i],
            self.data[i + 1],
            self.data[i + 2],
            self.data[i + 3],
        ];
        if self.big_endian {
            u32::from_be_bytes(b)
        } else {
            u32::from_le_bytes(b)
        }
    }

    #[inline]
    pub fn get(&self, r: usize, c: usize) -> f32 {
        f32::from_bits(self.bits(r, c))
    }

    /// Row `r`, columns `c0..c1`, appended to `out` bit for bit.
    pub fn extend_row(&self, r: usize, c0: usize, c1: usize, out: &mut Vec<f32>) {
        let start = r * self.step + 4 * c0;
        let end = r * self.step + 4 * c1;
        let row = &self.data[start..end];
        if self.big_endian {
            out.extend(
                row.chunks_exact(4)
                    .map(|b| f32::from_bits(u32::from_be_bytes([b[0], b[1], b[2], b[3]]))),
            );
        } else {
            out.extend(
                row.chunks_exact(4)
                    .map(|b| f32::from_bits(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))),
            );
        }
    }
}

/// `np.isfinite(d) & (d > 0.0)`: the validity both consumers apply.
#[inline]
pub fn valid_depth(d: f32) -> bool {
    d.is_finite() && d > 0.0
}

/// A decoded 8-bit single-channel image (`mono8` / `8UC1`), read in place
/// or owned (a resized mask).
#[derive(Clone, Debug)]
pub struct MonoImage<'a> {
    pub height: usize,
    pub width: usize,
    step: usize,
    data: std::borrow::Cow<'a, [u8]>,
}

impl<'a> MonoImage<'a> {
    pub fn decode(p: &ImageParts<'a>) -> Result<Self, DecodeError> {
        if p.encoding != "mono8" && p.encoding != "8UC1" {
            return Err(DecodeError::Bad(format!(
                "label image encoding {:?}: only mono8 / 8UC1 are decoded",
                p.encoding
            )));
        }
        let (height, width, step) = check_layout(p, 1)?;
        Ok(MonoImage {
            height,
            width,
            step,
            data: std::borrow::Cow::Borrowed(p.data),
        })
    }

    /// An owned image (row-major, step = width).
    pub fn owned(height: usize, width: usize, data: Vec<u8>) -> MonoImage<'static> {
        MonoImage {
            height,
            width,
            step: width,
            data: std::borrow::Cow::Owned(data),
        }
    }

    #[inline]
    pub fn get(&self, r: usize, c: usize) -> u8 {
        self.data[r * self.step + c]
    }

    /// Row `r` (its `width` pixels).
    #[inline]
    pub fn row(&self, r: usize) -> &[u8] {
        &self.data[r * self.step..r * self.step + self.width]
    }

    /// `cv2.resize(img, (width, height), interpolation=cv2.INTER_NEAREST)`
    /// (OpenCV 4.6 resizeNN: inv_scale = dst / src as double, source index
    /// floor(dst_index * (1 / inv_scale)), clamped to the last source index).
    pub fn resize_nearest(&self, height: usize, width: usize) -> MonoImage<'static> {
        let cols = nearest_map(self.width, width);
        let rows = nearest_map(self.height, height);
        let mut out = Vec::with_capacity(height * width);
        for &sy in &rows {
            let row = self.row(sy);
            out.extend(cols.iter().map(|&sx| row[sx]));
        }
        MonoImage::owned(height, width, out)
    }
}

/// resizeNN's source index for every destination index.
pub fn nearest_map(src: usize, dst: usize) -> Vec<usize> {
    let scale = dst as f64 / src as f64;
    let inv = 1.0 / scale;
    (0..dst)
        .map(|x| {
            let s = (x as f64 * inv).floor();
            let s = if s < 0.0 { 0 } else { s as usize };
            s.min(src.saturating_sub(1))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parts<'a>(enc: &'a str, h: u32, w: u32, step: u32, data: &'a [u8], big: u8) -> ImageParts<'a> {
        ImageParts {
            height: h,
            width: w,
            encoding: enc,
            is_bigendian: big,
            step,
            data,
        }
    }

    #[test]
    fn depth_reads_both_byte_orders_and_honours_the_step() {
        let v = [1.5_f32, -2.0, f32::NAN, 7.25];
        let mut le = Vec::new();
        let mut be = Vec::new();
        for (i, x) in v.iter().enumerate() {
            le.extend_from_slice(&x.to_le_bytes());
            be.extend_from_slice(&x.to_be_bytes());
            if i % 2 == 1 {
                // 4 padding bytes per row (step 12 for 2 columns).
                le.extend_from_slice(&[9, 9, 9, 9]);
                be.extend_from_slice(&[9, 9, 9, 9]);
            }
        }
        for (data, big) in [(&le, 0), (&be, 1)] {
            let d = DepthView::decode(&parts("32FC1", 2, 2, 12, data, big)).unwrap();
            for r in 0..2 {
                for c in 0..2 {
                    assert_eq!(d.bits(r, c), v[r * 2 + c].to_bits());
                }
            }
            let mut row = Vec::new();
            d.extend_row(1, 0, 2, &mut row);
            assert_eq!(row[1].to_bits(), 7.25_f32.to_bits());
        }
    }

    #[test]
    fn refusals() {
        let data = [0_u8; 16];
        assert_eq!(
            DepthView::decode(&parts("32FC1", 0, 2, 8, &data, 0)).unwrap_err(),
            DecodeError::Empty
        );
        assert!(DepthView::decode(&parts("16UC1", 2, 2, 8, &data, 0)).is_err());
        assert!(DepthView::decode(&parts("32FC1", 2, 2, 6, &data, 0)).is_err());
        assert!(DepthView::decode(&parts("32FC1", 3, 2, 8, &data, 0)).is_err());
        assert!(MonoImage::decode(&parts("bgr8", 2, 2, 2, &data, 0)).is_err());
        assert!(MonoImage::decode(&parts("mono8", 2, 2, 2, &data, 0)).is_ok());
        assert!(MonoImage::decode(&parts("8UC1", 2, 2, 2, &data, 0)).is_ok());
    }

    #[test]
    fn nearest_resize_maps_like_opencv() {
        assert_eq!(nearest_map(4, 2), vec![0, 2]);
        assert_eq!(nearest_map(2, 4), vec![0, 0, 1, 1]);
        assert_eq!(nearest_map(3, 3), vec![0, 1, 2]);
        let img = MonoImage::owned(2, 3, vec![1, 2, 3, 4, 5, 6]);
        let r = img.resize_nearest(4, 6);
        assert_eq!(r.row(0), &[1, 1, 2, 2, 3, 3]);
        assert_eq!(r.row(3), &[4, 4, 5, 5, 6, 6]);
    }
}
