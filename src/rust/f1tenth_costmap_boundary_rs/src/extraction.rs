//! Nearest-occupied-cell boundaries and front clearance, without ROS.
//!
//! A port of f1tenth_costmap/costmap_boundary.py as costmap_boundary_node.py's
//! `_extraction_tick` uses it with use_convex_polytope false:
//!
//! - `extract_boundary_constraints`: per direction (front cone +-35 deg,
//!   left [45, 135] deg, right [-135, -45] deg, car frame) the SINGLE nearest
//!   cell with value >= occupied_threshold within max_range_m, as a radial
//!   half-plane (normal toward the cell, offset = its distance);
//! - `nearest_occupied_in_strip` + `front_clearance_from_extraction`: the
//!   front clearance from the bumper, over the car-width strip ahead.
//!
//! Same arithmetic, same order of operations, in f64: the crop box
//! (`_crop_indices`), cell centres `origin + (i + 0.5) * resolution`, the map
//! -> car rotation R(-yaw), `hypot`, `atan2`, and numpy's argmin tie-break
//! (the first minimum in row-major order of the cropped block, which the
//! row-then-column loop with a strict `<` reproduces).
//!
//! The Python runs a separate crop/meshgrid per window; the per-cell values
//! are the same in all of them, so one pass computes all four searches.

/// One OccupancyGrid's data and geometry.
#[derive(Clone, Copy, Debug)]
pub struct Grid<'a> {
    /// Row-major, `width * height` values, -1 unknown, 0..=100.
    pub data: &'a [i8],
    pub width: usize,
    pub height: usize,
    /// info.resolution (an f32 on the wire), as f64.
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
}

/// Robot pose in the grid's (map) frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub front_facing_max_rad: f64,
    pub side_window_min_rad: f64,
    pub side_window_max_rad: f64,
    pub occupied_threshold: f64,
    pub max_range_m: f64,
    /// vehicle_frames.base_front_x() (0.405).
    pub front_x: f64,
    /// vehicle_frames.front_side_limit_m() (0.136 + 0.05).
    pub front_side_limit_m: f64,
    /// vehicle_frames.MIN_FRONT_DISTANCE_M (0.01).
    pub min_front_distance_m: f64,
}

/// (normal_x, normal_y, offset) in base_link.
pub type Constraint = (f64, f64, f64);

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Extraction {
    pub front: Option<Constraint>,
    pub left: Option<Constraint>,
    pub right: Option<Constraint>,
    /// /costmap/front_clearance before the f32 cast.
    pub front_clearance: f64,
}

impl Extraction {
    /// In costmap_boundary_node's publish order: front, left, right.
    pub fn constraints(&self) -> Vec<Constraint> {
        [self.front, self.left, self.right]
            .into_iter()
            .flatten()
            .collect()
    }
}

/// Inputs the Python cannot process: it raises inside the timer callback
/// (numpy reshape, ZeroDivisionError, int(NaN)), which ends the node.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InputError {
    /// data.len() != width * height.
    DataSize,
    /// resolution not finite or <= 0.
    Resolution,
    /// origin not finite.
    Origin,
    /// pose x/y/yaw not finite.
    Pose,
}

impl InputError {
    pub fn as_str(self) -> &'static str {
        match self {
            InputError::DataSize => "grid data length != width * height",
            InputError::Resolution => "grid resolution not a positive finite number",
            InputError::Origin => "grid origin not finite",
            InputError::Pose => "pose not finite",
        }
    }
}

/// `_crop_indices`: (col_min, col_max, row_min, row_max), clipped to the grid.
fn crop_indices(g: &Grid<'_>, robot_x: f64, robot_y: f64, max_range_m: f64) -> [usize; 4] {
    let clip = |v: f64, hi: usize| -> usize {
        // int(...) then max(0, min(v, hi)); the f64 -> i64 cast saturates,
        // which clips the same way Python's unbounded int would.
        let v = v as i64;
        v.clamp(0, hi as i64) as usize
    };
    let r = g.resolution;
    let col_min = ((robot_x - max_range_m - g.origin_x) / r).floor();
    let col_max = ((robot_x + max_range_m - g.origin_x) / r).ceil() + 1.0;
    let row_min = ((robot_y - max_range_m - g.origin_y) / r).floor();
    let row_max = ((robot_y + max_range_m - g.origin_y) / r).ceil() + 1.0;
    [
        clip(col_min, g.width),
        clip(col_max, g.width),
        clip(row_min, g.height),
        clip(row_max, g.height),
    ]
}

/// `boundary_from_nearest_point`.
pub fn boundary_from_nearest_point(car_dx: f64, car_dy: f64) -> Constraint {
    let k = car_dx.hypot(car_dy);
    if k < 1e-9 {
        return (0.0, 0.0, f64::INFINITY);
    }
    (car_dx / k, car_dy / k, k)
}

/// `front_clearance_from_extraction` for the node's call (front_x, min
/// distance and side limit always given).
pub fn front_clearance(front: Option<Constraint>, p: &Params) -> f64 {
    let Some((nx, ny, offset)) = front else {
        return p.max_range_m + 1.0;
    };
    if p.front_x == 0.0 || !offset.is_finite() {
        return offset;
    }
    let (px, py) = (nx * offset, ny * offset);
    if px <= p.front_x {
        if py.abs() > p.front_side_limit_m {
            return p.max_range_m + 1.0;
        }
        return p.min_front_distance_m;
    }
    (px - p.front_x).hypot(py).max(p.min_front_distance_m)
}

#[derive(Clone, Copy)]
struct Best {
    dist: f64,
    dx: f64,
    dy: f64,
}

fn consider(best: &mut Option<Best>, dist: f64, dx: f64, dy: f64) {
    // np.argmin over np.where(mask, dist, inf): the FIRST minimum wins.
    match best {
        Some(b) if b.dist <= dist => {}
        _ => *best = Some(Best { dist, dx, dy }),
    }
}

fn in_window(angle: f64, lo: f64, hi: f64) -> bool {
    if lo <= hi {
        angle >= lo && angle <= hi
    } else {
        angle >= lo || angle <= hi
    }
}

/// The whole `_extraction_tick` computation on one grid and pose.
pub fn extract(g: &Grid<'_>, pose: Pose, p: &Params) -> Result<Extraction, InputError> {
    if g.width.checked_mul(g.height) != Some(g.data.len()) {
        return Err(InputError::DataSize);
    }
    if !(g.resolution.is_finite() && g.resolution > 0.0) {
        return Err(InputError::Resolution);
    }
    if !(g.origin_x.is_finite() && g.origin_y.is_finite()) {
        return Err(InputError::Origin);
    }
    if !(pose.x.is_finite() && pose.y.is_finite() && pose.yaw.is_finite()) {
        return Err(InputError::Pose);
    }

    let [col_min, col_max, row_min, row_max] = crop_indices(g, pose.x, pose.y, p.max_range_m);
    let (mut front, mut left, mut right, mut strip) = (None, None, None, None);
    if col_min < col_max && row_min < row_max {
        let (sin_yaw, cos_yaw) = (pose.yaw.sin(), pose.yaw.cos());
        let (front_lo, front_hi) = (-p.front_facing_max_rad, p.front_facing_max_rad);
        let (left_lo, left_hi) = (p.side_window_min_rad, p.side_window_max_rad);
        let (right_lo, right_hi) = (-p.side_window_max_rad, -p.side_window_min_rad);
        for row in row_min..row_max {
            let cell_y = g.origin_y + (row as f64 + 0.5) * g.resolution;
            let dy = cell_y - pose.y;
            let base = row * g.width;
            for col in col_min..col_max {
                let value = f64::from(g.data[base + col]);
                if value < p.occupied_threshold {
                    // occupied_mask false: in none of the four searches. Unknown
                    // (-1) is below every positive threshold, as in the Python.
                    continue;
                }
                let cell_x = g.origin_x + (col as f64 + 0.5) * g.resolution;
                let dx = cell_x - pose.x;
                let car_dx = cos_yaw * dx + sin_yaw * dy;
                let car_dy = -sin_yaw * dx + cos_yaw * dy;
                let dist = car_dx.hypot(car_dy);
                if dist > p.max_range_m {
                    continue;
                }
                let angle = car_dy.atan2(car_dx);
                if in_window(angle, front_lo, front_hi) {
                    consider(&mut front, dist, car_dx, car_dy);
                }
                if in_window(angle, left_lo, left_hi) {
                    consider(&mut left, dist, car_dx, car_dy);
                }
                if in_window(angle, right_lo, right_hi) {
                    consider(&mut right, dist, car_dx, car_dy);
                }
                if car_dx > p.front_x && car_dy.abs() <= p.front_side_limit_m {
                    consider(&mut strip, dist, car_dx, car_dy);
                }
            }
        }
    }
    let to_constraint = |b: Option<Best>| b.map(|b| boundary_from_nearest_point(b.dx, b.dy));
    Ok(Extraction {
        front: to_constraint(front),
        left: to_constraint(left),
        right: to_constraint(right),
        front_clearance: front_clearance(to_constraint(strip), p),
    })
}

/// costmap_boundary.py's `yaw_from_quaternion`.
pub fn yaw_from_quaternion(x: f64, y: f64, z: f64, w: f64) -> f64 {
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> Params {
        Params {
            front_facing_max_rad: 35f64.to_radians(),
            side_window_min_rad: 45f64.to_radians(),
            side_window_max_rad: 135f64.to_radians(),
            occupied_threshold: 65.0,
            max_range_m: 5.0,
            front_x: 0.405,
            front_side_limit_m: 0.186,
            min_front_distance_m: 0.01,
        }
    }

    /// 10 m x 10 m at 0.05 m, origin (-5, -5), all free.
    fn free(n: usize) -> Vec<i8> {
        vec![0; n * n]
    }

    fn grid(data: &[i8], n: usize) -> Grid<'_> {
        Grid {
            data,
            width: n,
            height: n,
            resolution: 0.05,
            origin_x: -5.0,
            origin_y: -5.0,
        }
    }

    fn set(data: &mut [i8], n: usize, x: f64, y: f64, v: i8) {
        let c = ((x + 5.0) / 0.05).floor() as usize;
        let r = ((y + 5.0) / 0.05).floor() as usize;
        data[r * n + c] = v;
    }

    const ORIGIN: Pose = Pose {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
    };

    #[test]
    fn empty_map_has_no_constraints_and_reads_clear() {
        let d = free(200);
        let e = extract(&grid(&d, 200), ORIGIN, &params()).unwrap();
        assert!(e.constraints().is_empty());
        assert_eq!(e.front_clearance, 6.0);
    }

    #[test]
    fn wall_ahead_and_sides() {
        let n = 200;
        let mut d = free(n);
        for i in 0..n {
            let y = -5.0 + (i as f64 + 0.5) * 0.05;
            set(&mut d, n, 2.0, y, 100); // wall at x = 2
            let x = -5.0 + (i as f64 + 0.5) * 0.05;
            // Side walls far enough out that the +-35 deg front cone meets
            // the x = 2 wall first (a wall at y = 1 enters the cone at 1.74 m:
            // the cone's corridor phantom, which the strip search avoids).
            set(&mut d, n, x, 1.5, 100); // wall at y = 1.5
            set(&mut d, n, x, -2.0, 100); // wall at y = -2
        }
        let e = extract(&grid(&d, n), ORIGIN, &params()).unwrap();
        let (nx, ny, off) = e.front.unwrap();
        assert!((nx - 1.0).abs() < 1e-3 && ny.abs() < 0.02 && (off - 2.025).abs() < 0.03);
        let (_, ly, loff) = e.left.unwrap();
        assert!(ly > 0.99 && (loff - 1.525).abs() < 0.03);
        let (_, ry, roff) = e.right.unwrap();
        assert!(ry < -0.99 && (roff - 1.975).abs() < 0.03);
        // Front clearance from the bumper, inside the strip.
        assert!((e.front_clearance - (2.025 - 0.405)).abs() < 0.03);
        assert_eq!(e.constraints().len(), 3);
    }

    #[test]
    fn unknown_is_never_occupied() {
        let n = 200;
        let d = vec![-1i8; n * n];
        let e = extract(&grid(&d, n), ORIGIN, &params()).unwrap();
        assert!(e.constraints().is_empty());
        assert_eq!(e.front_clearance, 6.0);
    }

    #[test]
    fn far_outside_the_map_is_permissive() {
        // The out-of-bounds finding in costmap_boundary_node.py: no false
        // occupied, nothing found, clearance max_range + 1.
        let n = 200;
        let d = vec![100i8; n * n];
        let far = Pose {
            x: 50.0,
            y: 50.0,
            yaw: 0.0,
        };
        let e = extract(&grid(&d, n), far, &params()).unwrap();
        assert!(e.constraints().is_empty());
        assert_eq!(e.front_clearance, 6.0);
    }

    #[test]
    fn beside_the_car_is_not_front_clearance() {
        let n = 200;
        let mut d = free(n);
        set(&mut d, n, 0.3, 0.5, 100); // beside the body, ahead of base_link
        let e = extract(&grid(&d, n), ORIGIN, &params()).unwrap();
        assert_eq!(e.front_clearance, 6.0);
        assert!(e.left.is_some());
    }

    #[test]
    fn bad_inputs_are_errors() {
        let n = 10;
        let d = free(n);
        let mut g = grid(&d, n);
        g.width = 11;
        assert_eq!(extract(&g, ORIGIN, &params()), Err(InputError::DataSize));
        let mut g = grid(&d, n);
        g.resolution = 0.0;
        assert_eq!(extract(&g, ORIGIN, &params()), Err(InputError::Resolution));
        let nan = Pose {
            x: f64::NAN,
            y: 0.0,
            yaw: 0.0,
        };
        assert_eq!(extract(&grid(&d, n), nan, &params()), Err(InputError::Pose));
    }

    #[test]
    fn clearance_contact_band_reads_min_distance() {
        let p = params();
        // A cell at the bumper face inside the width: touching.
        assert_eq!(front_clearance(Some((1.0, 0.0, 0.40)), &p), 0.01);
        // Behind the face, beside the body: not a front reading.
        let k = (0.3f64).hypot(0.3);
        assert_eq!(front_clearance(Some((0.3 / k, 0.3 / k, k)), &p), 6.0);
        assert_eq!(front_clearance(None, &p), 6.0);
        assert_eq!(
            front_clearance(Some((0.0, 0.0, f64::INFINITY)), &p),
            f64::INFINITY
        );
    }

    #[test]
    fn yaw_formula() {
        let q = |yaw: f64| (0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos());
        for yaw in [-3.0, -1.0, 0.0, 0.5, 3.0] {
            let (x, y, z, w) = q(yaw);
            assert!((yaw_from_quaternion(x, y, z, w) - yaw).abs() < 1e-12);
        }
    }
}
