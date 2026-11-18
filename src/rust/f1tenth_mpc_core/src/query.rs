//! Corridor queries of `mpc_controller/mpc_solver.py`: the nearest centreline
//! sample and the frame frozen there.
//!
//! - [`nearest_centerline_index`]: `nearest_centerline_index(p, corridor)`;
//! - [`corridor_frame`]: `_nearest_corridor_frame(p, corridor)` (pc, n, half_w
//!   at that index), plus what its two callers derive from it:
//!   `corridor_lateral_coordinates` (`d_lat = n @ (p - pc)`) and
//!   `corridor_heading_at` (`atan2(-n_x, n_y)`).
//!
//! R1 ports and parity-checks these; the Python solver keeps calling its own
//! (the R2 solver core is their first Rust consumer).

use crate::corridor::Point;
use crate::npcompat::{dot2, py_atan2};

/// Why a query has no answer: each is an exception the Python raises.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QueryError {
    /// An empty centreline: `np.argmin` of an empty sequence (ValueError).
    Empty,
    /// Arrays of different lengths (numpy broadcasting or IndexError).
    LengthMismatch,
}

/// The frame at the nearest centreline sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    /// `np.argmin(d2)`: the first minimum, or the first NaN.
    pub idx: usize,
    /// `d2[idx]`, the squared distance.
    pub d2: f64,
    /// `(xc[idx], yc[idx])`.
    pub pc: Point,
    /// `(nx[idx], ny[idx])`.
    pub n: Point,
    /// `halfWidth[idx]`.
    pub half_w: f64,
    /// `corridor_lateral_coordinates`' `d_lat = n @ (p - pc)`.
    pub d_lat: f64,
    /// `corridor_heading_at`: `math.atan2(-n[0], n[1])`.
    pub heading: f64,
}

/// `nearest_centerline_index(p, corridor)`: `(d2[idx], idx)` with
/// `d2 = (xc - p0)**2 + (yc - p1)**2` and `idx = np.argmin(d2)`.
pub fn nearest_centerline_index(
    xc: &[f64],
    yc: &[f64],
    p: Point,
) -> Result<(f64, usize), QueryError> {
    if xc.len() != yc.len() {
        return Err(QueryError::LengthMismatch);
    }
    if xc.is_empty() {
        return Err(QueryError::Empty);
    }
    // np.argmin over d2, streamed: replace while !(d2 >= best), stop at NaN.
    let d2_at = |i: usize| {
        let dx = xc[i] - p[0];
        let dy = yc[i] - p[1];
        dx * dx + dy * dy
    };
    let mut best = d2_at(0);
    let mut idx = 0;
    if !best.is_nan() {
        for i in 1..xc.len() {
            let v = d2_at(i);
            #[allow(clippy::neg_cmp_op_on_partial_ord)]
            let replace = !(v >= best);
            if replace {
                best = v;
                idx = i;
                if best.is_nan() {
                    break;
                }
            }
        }
    }
    Ok((best, idx))
}

/// `_nearest_corridor_frame(p, corridor)` and the two values its callers
/// compute from it.
pub fn corridor_frame(
    xc: &[f64],
    yc: &[f64],
    nx: &[f64],
    ny: &[f64],
    half_width: &[f64],
    p: Point,
) -> Result<Frame, QueryError> {
    let (d2, idx) = nearest_centerline_index(xc, yc, p)?;
    if nx.len() != xc.len() || ny.len() != xc.len() || half_width.len() != xc.len() {
        return Err(QueryError::LengthMismatch);
    }
    let pc = [xc[idx], yc[idx]];
    let n = [nx[idx], ny[idx]];
    let d_lat = dot2(n, [p[0] - pc[0], p[1] - pc[1]]);
    Ok(Frame {
        idx,
        d2,
        pc,
        n,
        half_w: half_width[idx],
        d_lat,
        heading: py_atan2(-n[0], n[1]),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_nearest_sample_is_the_first_minimum() {
        let xc = [0.0, 1.0, 2.0, 1.0];
        let yc = [0.0, 0.0, 0.0, 0.0];
        assert_eq!(
            nearest_centerline_index(&xc, &yc, [1.0, 0.5]),
            Ok((0.25, 1))
        );
        assert_eq!(
            nearest_centerline_index(&[], &[], [0.0, 0.0]),
            Err(QueryError::Empty)
        );
        assert_eq!(
            nearest_centerline_index(&[0.0], &[0.0, 1.0], [0.0, 0.0]),
            Err(QueryError::LengthMismatch)
        );
        let nan = [0.0, f64::NAN, 5.0];
        assert_eq!(
            nearest_centerline_index(&nan, &[0.0; 3], [5.0, 0.0]).map(|r| r.1),
            Ok(1)
        );
    }

    #[test]
    fn the_frame_gives_lateral_offset_and_heading() {
        let xc = [0.0, 1.0, 2.0];
        let yc = [0.0, 0.0, 0.0];
        let nx = [-0.0, -0.0, -0.0];
        let ny = [1.0, 1.0, 1.0];
        let hw = [0.4, 0.5, 0.6];
        let f = corridor_frame(&xc, &yc, &nx, &ny, &hw, [1.2, 0.3]).unwrap();
        assert_eq!(f.idx, 1);
        assert_eq!(f.half_w, 0.5);
        assert!((f.d_lat - 0.3).abs() < 1e-15);
        assert_eq!(f.heading, 0.0);
    }
}
