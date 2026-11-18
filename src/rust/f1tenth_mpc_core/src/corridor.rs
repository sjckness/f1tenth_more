//! Port of `f1tenth_params/f1tenth_params/corridor_geometry.py`.
//!
//! [`corridor_curves`] (the integrated heading ramp of a given length) and
//! [`corridor_curves_to_pose`] (the cubic Bezier between two poses), with the
//! two clamps (ramp span against `r_min`, width rate against `length_ref`)
//! and both wall constructions. Read the Python module's docstring for what
//! the curves are; this file only repeats what matters for the port: every
//! numpy operation is the [`crate::npcompat`] one, in the order the Python
//! writes it, so the arrays come out bit for bit equal on linus.
//!
//! One deliberate representation difference: the Python dict's `ny` IS the
//! `tx` array (`'ny': tx`), so [`Corridor`] has no `ny` field; the ctypes
//! binding hands the same array object out under both keys.

use crate::npcompat::{
    cumsum, gradient, interp, linspace, max_reduce, np_clip, np_maximum, pow3, py_atan2, py_max,
    py_min, py_trig_domain_ok,
};

/// `CORRIDOR_HANDLE_FRAC`: Bezier handle length as a fraction of the length.
pub const CORRIDOR_HANDLE_FRAC: f64 = 0.55;
/// `CORRIDOR_WIDTH_RATIO`: end-to-start half-width ratio, `2.3 / 1.3`.
pub const CORRIDOR_WIDTH_RATIO: f64 = 2.3 / 1.3;
/// `POSE_HANDLE_FRAC`: pose-to-pose handle as a fraction of the chord.
pub const POSE_HANDLE_FRAC: f64 = 0.40;
/// `SMOOTHSTEP_PEAK_SLOPE`: peak of the smoothstep's derivative.
pub const SMOOTHSTEP_PEAK_SLOPE: f64 = 1.5;

/// The largest `n` the port accepts. The pose corridor evaluates
/// `max(8 n, 512)` dense samples; above this a Rust allocation failure would
/// abort the host process instead of raising, so the call is refused
/// ([`CorridorError::TooManySamples`]). The planner uses n = 120.
pub const MAX_SAMPLES: usize = 1_000_000;

/// `CORRIDOR_WALL_SHAPES`: `('bezier', 'trapezoid')`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Walls {
    /// Two cubic Beziers (the default, every logged record before trapezoid).
    Bezier,
    /// The centreline offset along its own normal by a linear half-width.
    Trapezoid,
}

/// Why a corridor was not built: each is an exception the Python raises.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CorridorError {
    /// `n < 2`: `ValueError('a corridor needs at least 2 samples, got n')`.
    TooFewSamples(i64),
    /// `n > MAX_SAMPLES` (no Python counterpart: it would try the allocation).
    TooManySamples(i64),
    /// `math.sin` / `math.cos` of an infinite angle:
    /// `ValueError('math domain error')`.
    MathDomain,
    /// `1.0 / r_min` with `r_min == 0` (pose corridor):
    /// `ZeroDivisionError('float division by zero')`.
    ZeroDivision,
}

/// A point, `[x, y]`.
pub type Point = [f64; 2];

/// Inputs of [`corridor_curves`], named and defaulted as in the Python.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurvesArgs {
    pub x0: f64,
    pub y0: f64,
    pub psi_start: f64,
    pub psi_end: f64,
    pub length: f64,
    pub n: i64,
    /// `None`: the shortest-branch wrap of `psi_end - psi_start`.
    pub dpsi: Option<f64>,
    pub u_start: f64,
    pub u_end: f64,
    pub w0: f64,
    pub w1: f64,
    pub handle_frac: f64,
    /// `Some`: the ramp-span clamp.
    pub r_min: Option<f64>,
    /// `Some`: the width rate.
    pub length_ref: Option<f64>,
    pub walls: Walls,
}

impl CurvesArgs {
    /// The required arguments, every keyword at its Python default.
    pub fn new(x0: f64, y0: f64, psi_start: f64, psi_end: f64, length: f64, n: i64) -> Self {
        Self {
            x0,
            y0,
            psi_start,
            psi_end,
            length,
            n,
            dpsi: None,
            u_start: 0.0,
            u_end: 0.40,
            w0: 0.4333,
            w1: 0.7667,
            handle_frac: CORRIDOR_HANDLE_FRAC,
            r_min: None,
            length_ref: None,
            walls: Walls::Bezier,
        }
    }
}

/// Inputs of [`corridor_curves_to_pose`], named and defaulted as in the Python.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PoseArgs {
    pub x0: f64,
    pub y0: f64,
    pub psi_start: f64,
    pub x1: f64,
    pub y1: f64,
    pub psi_end: f64,
    pub n: i64,
    pub handle_a: f64,
    pub handle_b: f64,
    pub w0: f64,
    pub w1: f64,
    pub handle_frac: f64,
    pub r_min: Option<f64>,
    pub length_ref: Option<f64>,
    pub walls: Walls,
}

impl PoseArgs {
    /// The required arguments, every keyword at its Python default.
    #[allow(clippy::too_many_arguments)]
    pub fn new(x0: f64, y0: f64, psi_start: f64, x1: f64, y1: f64, psi_end: f64, n: i64) -> Self {
        Self {
            x0,
            y0,
            psi_start,
            x1,
            y1,
            psi_end,
            n,
            handle_a: POSE_HANDLE_FRAC,
            handle_b: POSE_HANDLE_FRAC,
            w0: 0.4333,
            w1: 0.7667,
            handle_frac: CORRIDOR_HANDLE_FRAC,
            r_min: None,
            length_ref: None,
            walls: Walls::Bezier,
        }
    }
}

/// The corridor dict's contents. `ny` is `tx` (see the module doc).
#[derive(Clone, Debug, PartialEq)]
pub struct Corridor {
    pub xc: Vec<f64>,
    pub yc: Vec<f64>,
    pub x_l: Vec<f64>,
    pub y_l: Vec<f64>,
    pub x_r: Vec<f64>,
    pub y_r: Vec<f64>,
    pub tx: Vec<f64>,
    pub ty: Vec<f64>,
    /// `-ty`.
    pub nx: Vec<f64>,
    pub half_width: Vec<f64>,
    pub pend: Point,
    pub dpsi: f64,
    /// Ramp: the `length` asked for. Pose: the centreline's arclength (an
    /// output, the dict's `length` key).
    pub length: f64,
    pub u_end_eff: f64,
    pub w1_eff: f64,
    pub ramp_clamped: bool,
    pub feasible: bool,
    pub kappa_max: f64,
    /// `None` for trapezoid walls.
    pub ctrl_left: Option<[Point; 4]>,
    pub ctrl_right: Option<[Point; 4]>,
    /// Pose corridor only: the centreline's four control points.
    pub ctrl_centre: Option<[Point; 4]>,
}

impl Corridor {
    /// The dict's `ny`: the same array as `tx`.
    pub fn ny(&self) -> &[f64] {
        &self.tx
    }
}

/// `wrap_pi(angle)`: `math.atan2(math.sin(angle), math.cos(angle))`.
pub fn wrap_pi(angle: f64) -> Result<f64, CorridorError> {
    if !py_trig_domain_ok(angle) {
        return Err(CorridorError::MathDomain);
    }
    Ok(py_atan2(angle.sin(), angle.cos()))
}

/// `ramp_span_for(dpsi, r_min)`: `1.5 * abs(dpsi) * r_min`.
pub fn ramp_span_for(dpsi: f64, r_min: f64) -> f64 {
    SMOOTHSTEP_PEAK_SLOPE * dpsi.abs() * r_min
}

/// `bezier(p0, c0, c1, p1, u)` as its two coordinate columns:
/// `((1-u)^3 p0 + 3 (1-u)^2 u c0) + 3 (1-u) u^2 c1) + u^3 p1`, left to right,
/// each product in the order numpy evaluates it.
pub fn bezier(p0: Point, c0: Point, c1: Point, p1: Point, u: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let mut xs = Vec::with_capacity(u.len());
    let mut ys = Vec::with_capacity(u.len());
    for &uu in u {
        let om = 1.0 - uu;
        let a = pow3(om);
        let b = 3.0 * (om * om) * uu;
        let c = 3.0 * om * (uu * uu);
        let d = pow3(uu);
        xs.push(a * p0[0] + b * c0[0] + c * c1[0] + d * p1[0]);
        ys.push(a * p0[1] + b * c0[1] + c * c1[1] + d * p1[1]);
    }
    (xs, ys)
}

fn check_n(n: i64) -> Result<usize, CorridorError> {
    if n < 2 {
        return Err(CorridorError::TooFewSamples(n));
    }
    let nu = usize::try_from(n).map_err(|_| CorridorError::TooManySamples(n))?;
    if nu > MAX_SAMPLES {
        return Err(CorridorError::TooManySamples(n));
    }
    Ok(nu)
}

/// `w1_eff`, the width rate: `w0 + (w1 - w0) * min(length / length_ref, 1)`
/// when `length_ref` is given and positive, else `w1`.
fn width_rate(w0: f64, w1: f64, length: f64, length_ref: Option<f64>) -> f64 {
    match length_ref {
        Some(lr) if lr > 0.0 => w0 + (w1 - w0) * py_min(length / lr, 1.0),
        _ => w1,
    }
}

#[inline]
fn add(a: Point, b: Point) -> Point {
    [a[0] + b[0], a[1] + b[1]]
}

#[inline]
fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}

#[inline]
fn scale(k: f64, a: Point) -> Point {
    [k * a[0], k * a[1]]
}

/// The two walls, the tangents and the half-width: the part both corridor
/// families share once the centreline (`xc`, `yc`) exists.
struct Walled {
    x_l: Vec<f64>,
    y_l: Vec<f64>,
    x_r: Vec<f64>,
    y_r: Vec<f64>,
    tx: Vec<f64>,
    ty: Vec<f64>,
    nx: Vec<f64>,
    half_width: Vec<f64>,
    ctrl_left: Option<[Point; 4]>,
    ctrl_right: Option<[Point; 4]>,
}

/// Everything below the centreline in both Python functions, in their order:
/// Bezier control points from the end poses, the Bezier walls at `u`, the
/// tangents from `np.gradient`, the trapezoid offset when asked for, the
/// half-width.
#[allow(clippy::too_many_arguments)]
fn walls_and_tangents(
    xc: &[f64],
    yc: &[f64],
    u: &[f64],
    n0: Point,
    n1: Point,
    e0: Point,
    e1: Point,
    w0: f64,
    w1_eff: f64,
    handle_frac: f64,
    length: f64,
    walls: Walls,
) -> Walled {
    let n = xc.len();
    let c0_pt = [xc[0], yc[0]];
    let c1_pt = [xc[n - 1], yc[n - 1]];
    let p_l0 = add(c0_pt, scale(w0, n0));
    let p_r0 = sub(c0_pt, scale(w0, n0));
    let p_l1 = add(c1_pt, scale(w1_eff, n1));
    let p_r1 = sub(c1_pt, scale(w1_eff, n1));
    let k = handle_frac * length;
    let cl0 = add(p_l0, scale(k, e0));
    let cl1 = sub(p_l1, scale(k, e1));
    let cr0 = add(p_r0, scale(k, e0));
    let cr1 = sub(p_r1, scale(k, e1));

    let dx = gradient(xc);
    let dy = gradient(yc);
    let mut tx = Vec::with_capacity(n);
    let mut ty = Vec::with_capacity(n);
    for i in 0..n {
        let dn = np_maximum((dx[i] * dx[i] + dy[i] * dy[i]).sqrt(), 1e-9);
        tx.push(dx[i] / dn);
        ty.push(dy[i] / dn);
    }

    let (x_l, y_l, x_r, y_r, ctrl_left, ctrl_right) = match walls {
        Walls::Bezier => {
            let (x_l, y_l) = bezier(p_l0, cl0, cl1, p_l1, u);
            let (x_r, y_r) = bezier(p_r0, cr0, cr1, p_r1, u);
            (
                x_l,
                y_l,
                x_r,
                y_r,
                Some([p_l0, cl0, cl1, p_l1]),
                Some([p_r0, cr0, cr1, p_r1]),
            )
        }
        Walls::Trapezoid => {
            // hw(u) = w0 + (w1_eff - w0) u along the centreline's own normal
            // (-ty, tx). The Python evaluates the Bezier walls first and
            // discards them; that has no observable effect, so it is skipped.
            let mut x_l = Vec::with_capacity(n);
            let mut y_l = Vec::with_capacity(n);
            let mut x_r = Vec::with_capacity(n);
            let mut y_r = Vec::with_capacity(n);
            let dw = w1_eff - w0;
            for i in 0..n {
                let hw = w0 + dw * u[i];
                x_l.push(xc[i] - hw * ty[i]);
                y_l.push(yc[i] + hw * tx[i]);
                x_r.push(xc[i] + hw * ty[i]);
                y_r.push(yc[i] - hw * tx[i]);
            }
            (x_l, y_l, x_r, y_r, None, None)
        }
    };

    let nx: Vec<f64> = ty.iter().map(|v| -v).collect();
    let half_width: Vec<f64> = (0..n)
        .map(|i| {
            let ddx = x_l[i] - x_r[i];
            let ddy = y_l[i] - y_r[i];
            0.5 * (ddx * ddx + ddy * ddy).sqrt()
        })
        .collect();

    Walled {
        x_l,
        y_l,
        x_r,
        y_r,
        tx,
        ty,
        nx,
        half_width,
        ctrl_left,
        ctrl_right,
    }
}

/// `corridor_curves(...)`: the integrated heading ramp of length `length`.
pub fn corridor_curves(a: &CurvesArgs) -> Result<Corridor, CorridorError> {
    let n = check_n(a.n)?;
    // math.sin/cos of psi_start and psi_end (n0, n1, e0, e1) and, without an
    // explicit dpsi, of psi_end - psi_start (wrap_pi): all ValueError.
    if !py_trig_domain_ok(a.psi_start) || !py_trig_domain_ok(a.psi_end) {
        return Err(CorridorError::MathDomain);
    }
    let length = a.length;

    let u = linspace(0.0, 1.0, n);
    let s: Vec<f64> = u.iter().map(|&ui| length * ui).collect();

    let dpsi = match a.dpsi {
        Some(d) => d,
        None => wrap_pi(a.psi_end - a.psi_start)?,
    };

    // ---- width rate ----
    let w1_eff = width_rate(a.w0, a.w1, length, a.length_ref);

    // ---- ramp-span clamp ----
    let mut u_end_eff = a.u_end;
    let mut ramp_clamped = false;
    let mut feasible = true;
    if let Some(r_min) = a.r_min {
        if length > 0.0 {
            let need = ramp_span_for(dpsi, r_min);
            let u_need = a.u_start + need / length;
            if u_need > u_end_eff {
                u_end_eff = py_min(1.0, u_need);
                ramp_clamped = true;
            }
            feasible = (u_end_eff - a.u_start) * length + 1e-12 >= need;
        }
    }
    let u_end = u_end_eff;

    let denom = py_max(u_end - a.u_start, 1e-6);
    let mut xc = Vec::with_capacity(n);
    let mut yc = Vec::with_capacity(n);
    // cumsum(cos(theta) * ds) with ds[0] = 0 and ds[i] = s[i] - s[i-1],
    // then x0 + (the running sum), element by element as numpy does.
    let mut acc_x = 0.0;
    let mut acc_y = 0.0;
    for i in 0..n {
        let tau = np_clip((u[i] - a.u_start) / denom, 0.0, 1.0);
        let shape = 3.0 * (tau * tau) - 2.0 * pow3(tau);
        let theta = a.psi_start + dpsi * shape;
        let ds = if i == 0 { 0.0 } else { s[i] - s[i - 1] };
        let cx = theta.cos() * ds;
        let cy = theta.sin() * ds;
        if i == 0 {
            acc_x = cx;
            acc_y = cy;
        } else {
            acc_x += cx;
            acc_y += cy;
        }
        xc.push(a.x0 + acc_x);
        yc.push(a.y0 + acc_y);
    }

    let n0 = [-a.psi_start.sin(), a.psi_start.cos()];
    let n1 = [-a.psi_end.sin(), a.psi_end.cos()];
    let e0 = [a.psi_start.cos(), a.psi_start.sin()];
    let e1 = [a.psi_end.cos(), a.psi_end.sin()];

    let w = walls_and_tangents(
        &xc,
        &yc,
        &u,
        n0,
        n1,
        e0,
        e1,
        a.w0,
        w1_eff,
        a.handle_frac,
        length,
        a.walls,
    );
    let pend = [xc[n - 1], yc[n - 1]];
    let kappa_max =
        SMOOTHSTEP_PEAK_SLOPE * dpsi.abs() / py_max((u_end_eff - a.u_start) * length, 1e-9);

    Ok(Corridor {
        xc,
        yc,
        x_l: w.x_l,
        y_l: w.y_l,
        x_r: w.x_r,
        y_r: w.y_r,
        tx: w.tx,
        ty: w.ty,
        nx: w.nx,
        half_width: w.half_width,
        pend,
        dpsi,
        length,
        u_end_eff,
        w1_eff,
        ramp_clamped,
        feasible,
        kappa_max,
        ctrl_left: w.ctrl_left,
        ctrl_right: w.ctrl_right,
        ctrl_centre: None,
    })
}

/// `corridor_curves_to_pose(...)`: the cubic Bezier centreline that starts on
/// `(x0, y0, psi_start)` and ends on `(x1, y1, psi_end)`, sampled by
/// arclength.
pub fn corridor_curves_to_pose(a: &PoseArgs) -> Result<Corridor, CorridorError> {
    let n = check_n(a.n)?;

    let p0 = [a.x0, a.y0];
    let p3 = [a.x1, a.y1];
    let chord = (p3[0] - p0[0]).hypot(p3[1] - p0[1]);
    // e_start / e_end: math.cos / math.sin of the two headings.
    if !py_trig_domain_ok(a.psi_start) || !py_trig_domain_ok(a.psi_end) {
        return Err(CorridorError::MathDomain);
    }
    let e_start = [a.psi_start.cos(), a.psi_start.sin()];
    let e_end = [a.psi_end.cos(), a.psi_end.sin()];
    let ha = py_max(a.handle_a * chord, 1e-6);
    let hb = py_max(a.handle_b * chord, 1e-6);
    let p1 = add(p0, scale(ha, e_start));
    let p2 = sub(p3, scale(hb, e_end));

    // Dense in the Bezier parameter: the points, then curvature from the
    // first two derivatives, every product in numpy's order.
    let dense = (8 * n).max(512);
    let t = linspace(0.0, 1.0, dense);
    let d10 = sub(p1, p0);
    let d21 = sub(p2, p1);
    let d32 = sub(p3, p2);
    // (p2 - 2 p1 + p0) and (p3 - 2 p2 + p1), left to right
    let q0 = add(sub(p2, scale(2.0, p1)), p0);
    let q1 = add(sub(p3, scale(2.0, p2)), p1);
    let mut px = Vec::with_capacity(dense);
    let mut py = Vec::with_capacity(dense);
    let mut kappa = Vec::with_capacity(dense);
    for &tt in &t {
        let om = 1.0 - tt;
        let a3 = pow3(om);
        let b3 = 3.0 * (om * om) * tt;
        let c3 = 3.0 * om * (tt * tt);
        let d3 = pow3(tt);
        px.push(a3 * p0[0] + b3 * p1[0] + c3 * p2[0] + d3 * p3[0]);
        py.push(a3 * p0[1] + b3 * p1[1] + c3 * p2[1] + d3 * p3[1]);

        let f1 = 3.0 * (om * om);
        let f2 = 6.0 * om * tt;
        let f3 = 3.0 * (tt * tt);
        let d1x = f1 * d10[0] + f2 * d21[0] + f3 * d32[0];
        let d1y = f1 * d10[1] + f2 * d21[1] + f3 * d32[1];
        let g1 = 6.0 * om;
        let g2 = 6.0 * tt;
        let d2x = g1 * q0[0] + g2 * q1[0];
        let d2y = g1 * q0[1] + g2 * q1[1];
        let speed = np_maximum(d1x.hypot(d1y), 1e-12);
        kappa.push((d1x * d2y - d1y * d2x).abs() / pow3(speed));
    }

    let seg: Vec<f64> = (0..dense - 1)
        .map(|i| (px[i + 1] - px[i]).hypot(py[i + 1] - py[i]))
        .collect();
    let mut s_dense = Vec::with_capacity(dense);
    s_dense.push(0.0);
    s_dense.extend(cumsum(&seg));
    let length = s_dense[dense - 1];

    let s_out = linspace(0.0, length, n);
    let (Some(mut xc), Some(mut yc)) =
        (interp(&s_out, &s_dense, &px), interp(&s_out, &s_dense, &py))
    else {
        // unreachable: xp and fp both have `dense` >= 512 samples
        return Err(CorridorError::TooFewSamples(a.n));
    };
    // The ends are the poses asked for, exactly.
    xc[0] = p0[0];
    yc[0] = p0[1];
    xc[n - 1] = p3[0];
    yc[n - 1] = p3[1];

    let kappa_max = max_reduce(&kappa).unwrap_or(f64::NAN);
    let feasible = match a.r_min {
        None => true,
        Some(r) => {
            if r == 0.0 {
                return Err(CorridorError::ZeroDivision);
            }
            kappa_max <= 1.0 / r
        }
    };

    let w1_eff = width_rate(a.w0, a.w1, length, a.length_ref);
    let n0 = [-a.psi_start.sin(), a.psi_start.cos()];
    let n1 = [-a.psi_end.sin(), a.psi_end.cos()];
    let u = linspace(0.0, 1.0, n);
    let w = walls_and_tangents(
        &xc,
        &yc,
        &u,
        n0,
        n1,
        e_start,
        e_end,
        a.w0,
        w1_eff,
        a.handle_frac,
        length,
        a.walls,
    );
    let pend = [xc[n - 1], yc[n - 1]];
    let dpsi = wrap_pi(a.psi_end - a.psi_start)?;

    Ok(Corridor {
        xc,
        yc,
        x_l: w.x_l,
        y_l: w.y_l,
        x_r: w.x_r,
        y_r: w.y_r,
        tx: w.tx,
        ty: w.ty,
        nx: w.nx,
        half_width: w.half_width,
        pend,
        dpsi,
        length,
        u_end_eff: 1.0,
        w1_eff,
        ramp_clamped: false,
        feasible,
        kappa_max,
        ctrl_left: w.ctrl_left,
        ctrl_right: w.ctrl_right,
        ctrl_centre: Some([p0, p1, p2, p3]),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn constants_are_the_python_ones() {
        assert_eq!(CORRIDOR_WIDTH_RATIO, 2.3 / 1.3);
        assert_eq!(CORRIDOR_HANDLE_FRAC, 0.55);
        assert_eq!(POSE_HANDLE_FRAC, 0.40);
    }

    #[test]
    fn a_straight_corridor_is_straight_and_ends_at_length() {
        let c = corridor_curves(&CurvesArgs::new(1.0, -2.0, 0.0, 0.0, 3.0, 120)).unwrap();
        assert_eq!(c.xc.len(), 120);
        assert!(c.yc.iter().all(|&y| y == -2.0));
        assert!(close(c.xc[119], 4.0, 1e-12));
        assert!(c.tx.iter().all(|&t| close(t, 1.0, 1e-15)));
        assert!(c.ty.iter().all(|&t| t == 0.0));
        assert!(close(c.half_width[0], 0.4333, 1e-12));
        assert!(close(c.half_width[119], 0.7667, 1e-12));
        assert_eq!(c.pend, [c.xc[119], c.yc[119]]);
        assert!(c.feasible);
        assert!(!c.ramp_clamped);
        assert!(c.ctrl_left.is_some() && c.ctrl_centre.is_none());
        assert_eq!(c.ny(), &c.tx[..]);
    }

    #[test]
    fn too_few_samples_and_domain_errors_are_reported() {
        let mut a = CurvesArgs::new(0.0, 0.0, 0.0, 0.0, 3.0, 1);
        assert_eq!(corridor_curves(&a), Err(CorridorError::TooFewSamples(1)));
        a.n = 3;
        a.psi_start = f64::INFINITY;
        assert_eq!(corridor_curves(&a), Err(CorridorError::MathDomain));
        // an overflowing psi_end - psi_start, only without an explicit dpsi
        a.psi_start = -1e308;
        a.psi_end = 1e308;
        assert_eq!(corridor_curves(&a), Err(CorridorError::MathDomain));
        a.dpsi = Some(0.1);
        assert!(corridor_curves(&a).is_ok());
        let mut p = PoseArgs::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 10);
        p.r_min = Some(0.0);
        assert_eq!(
            corridor_curves_to_pose(&p),
            Err(CorridorError::ZeroDivision)
        );
        p.r_min = None;
        p.n = MAX_SAMPLES as i64 + 1;
        assert!(matches!(
            corridor_curves_to_pose(&p),
            Err(CorridorError::TooManySamples(_))
        ));
    }

    #[test]
    fn the_ramp_clamp_widens_and_reports() {
        let dpsi = 120.0_f64.to_radians();
        let mut a = CurvesArgs::new(0.0, 0.0, 0.0, dpsi, 0.3, 120);
        a.dpsi = Some(dpsi);
        a.r_min = Some(0.923);
        let c = corridor_curves(&a).unwrap();
        assert!(!c.feasible);
        assert_eq!(c.u_end_eff, 1.0);
        assert!(c.ramp_clamped);
        assert_eq!(c.dpsi, dpsi);
    }

    #[test]
    fn the_pose_corridor_ends_on_both_poses() {
        let c = corridor_curves_to_pose(&PoseArgs::new(0.0, 0.0, 0.2, 2.0, 1.0, 0.5, 120)).unwrap();
        assert_eq!([c.xc[0], c.yc[0]], [0.0, 0.0]);
        assert_eq!([c.xc[119], c.yc[119]], [2.0, 1.0]);
        assert!(c.length > 2.0_f64.hypot(1.0));
        assert!(c.ctrl_centre.is_some());
        assert_eq!(c.u_end_eff, 1.0);
        assert!(!c.ramp_clamped);
    }

    #[test]
    fn trapezoid_walls_are_two_half_widths_apart() {
        let mut a = CurvesArgs::new(0.0, 0.0, 0.0, 0.6, 3.0, 50);
        a.walls = Walls::Trapezoid;
        let c = corridor_curves(&a).unwrap();
        assert!(c.ctrl_left.is_none() && c.ctrl_right.is_none());
        for i in 0..50 {
            let hw = 0.4333 + (0.7667 - 0.4333) * (i as f64 / 49.0);
            assert!(close(c.half_width[i], hw, 1e-12), "{i}");
        }
    }

    #[test]
    fn n_two_and_zero_length_do_not_fail() {
        let c = corridor_curves(&CurvesArgs::new(0.0, 0.0, 0.0, 1.0, 0.0, 2)).unwrap();
        assert_eq!(c.xc, vec![0.0, 0.0]);
        let p = corridor_curves_to_pose(&PoseArgs::new(1.0, 1.0, 0.0, 1.0, 1.0, 3.0, 2)).unwrap();
        assert_eq!(p.xc, vec![1.0, 1.0]);
    }
}
