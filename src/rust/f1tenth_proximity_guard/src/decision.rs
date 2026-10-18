//! The front-proximity decision, without ROS.
//!
//! A port of the FRONT half of `IsProximityTooClose` (f1tenth_behavior/
//! behaviours/is_proximity_too_close.py, campaign-final 0c1cd41): the
//! vectorised `_scan_callback` front block and `_tripped()`'s front test.
//! The side/rear window is not ported.
//!
//! Same arithmetic, same order of operations, in f64:
//!
//! ```text
//! angle_i = angle_min + i * angle_increment
//! valid_i = isfinite(r_i) and r_i >= range_min
//! dx_i    = (lx + r_i * cos(lyaw + angle_i)) - front_x
//! dy_i    =  ly + r_i * sin(lyaw + angle_i)
//! front_i = valid_i and dx_i > 0 and |atan2(dy_i, dx_i)| < cone_half
//! min     = min over front_i of hypot(dx_i, dy_i)
//! tripped = min exists and min < threshold
//! ```
//!
//! `tripped` is the parity output: it must equal the Python front decision on
//! every input. `stop` adds the fail-safe rules the Python does not have
//! ([`FailSafe`]); each is reported as its own [`Reason`] so a difference
//! from the Python is always attributable.

/// Where the LiDAR sits in base_link: (x, y, yaw), as `StaticFramePoses` reads it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LaserPose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl LaserPose {
    /// The Python's stand-in until /tf_static has the laser: base_link's
    /// origin, yaw 0 ("trips early, never late" for a laser behind the bumper).
    pub const FALLBACK: LaserPose = LaserPose {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
    };
}

/// The parts of a sensor_msgs/LaserScan the decision reads.
#[derive(Clone, Copy, Debug)]
pub struct ScanView<'a> {
    pub angle_min: f32,
    pub angle_increment: f32,
    pub range_min: f32,
    pub ranges: &'a [f32],
}

/// Geometry and fail-safe limits.
#[derive(Clone, Copy, Debug)]
pub struct GuardConfig {
    /// base_front x in base_link [m] (the URDF's front bumper, 0.405).
    pub front_x: f64,
    /// Trip distance from the bumper [m] (stack_params proximity_front_threshold_m).
    pub threshold_m: f64,
    /// Half-angle of the front cone, measured from the bumper [rad].
    pub cone_half_angle_rad: f64,
    /// Fail-safe: stop when more than this fraction of the beams pointing into
    /// the front sector are NaN or below range_min.
    pub max_invalid_fraction: f64,
}

/// What one scan says about the front, before any decision.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrontEval {
    /// Nearest valid return in the cone, from the bumper [m] (Python's
    /// `latest_lidar_front_min`).
    pub min_front: Option<f64>,
    /// Beams whose bearing from the laser (in base_link) is inside the cone.
    pub sector_beams: usize,
    /// Of those: NaN, or finite and below range_min (skipped by the Python).
    pub invalid_beams: usize,
    /// Of those: -inf, REP-117 "too close to measure" (skipped by the Python).
    pub too_close_beams: usize,
    /// The scan header's layout was unusable (non-finite angle fields).
    pub bad_layout: bool,
}

/// Why the guard decided what it decided.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    /// Nothing in the cone nearer than the threshold.
    Clear,
    /// The Python condition trips (parity case).
    Tripped,
    /// Fail-safe: no scan received since the node started.
    NoScanYet,
    /// Fail-safe: the last scan is older than the timeout.
    Stale,
    /// Fail-safe: a scan with no ranges.
    EmptyScan,
    /// Fail-safe: angle_min/angle_increment/range_min not finite.
    BadLayout,
    /// Fail-safe: no beam of this scan points into the front sector.
    NoFrontBeams,
    /// Fail-safe: too many front-sector beams are NaN / below range_min.
    InvalidBeams,
    /// Fail-safe: a front-sector beam reads -inf (too close to measure).
    TooCloseUnquantified,
}

impl Reason {
    pub fn as_str(self) -> &'static str {
        match self {
            Reason::Clear => "clear",
            Reason::Tripped => "tripped",
            Reason::NoScanYet => "no_scan_yet",
            Reason::Stale => "stale",
            Reason::EmptyScan => "empty_scan",
            Reason::BadLayout => "bad_layout",
            Reason::NoFrontBeams => "no_front_beams",
            Reason::InvalidBeams => "invalid_beams",
            Reason::TooCloseUnquantified => "too_close_unquantified",
        }
    }

    /// Every reason but `Clear` stops the car in active mode.
    pub fn stops(self) -> bool {
        self != Reason::Clear
    }

    /// True for the reasons the Python has no equivalent for.
    pub fn is_fail_safe(self) -> bool {
        !matches!(self, Reason::Clear | Reason::Tripped)
    }
}

/// One decision.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Decision {
    /// The Python's front decision on the same scan and laser pose.
    pub tripped: bool,
    /// What the guard acts on: `tripped` or a fail-safe reason.
    pub stop: bool,
    pub reason: Reason,
    pub eval: FrontEval,
}

/// Wrap an angle into [-pi, pi].
fn wrap_pi(a: f64) -> f64 {
    let two_pi = 2.0 * std::f64::consts::PI;
    let w = (a + std::f64::consts::PI).rem_euclid(two_pi) - std::f64::consts::PI;
    if w < -std::f64::consts::PI {
        w + two_pi
    } else {
        w
    }
}

/// The front block of `IsProximityTooClose._scan_callback`, plus the counts
/// the fail-safe rules need.
pub fn evaluate_front(scan: &ScanView<'_>, laser: LaserPose, cfg: &GuardConfig) -> FrontEval {
    let angle_min = f64::from(scan.angle_min);
    let increment = f64::from(scan.angle_increment);
    let range_min = f64::from(scan.range_min);
    let bad_layout = !(angle_min.is_finite() && increment.is_finite() && range_min.is_finite());

    let mut min_front: Option<f64> = None;
    let mut sector_beams = 0usize;
    let mut invalid_beams = 0usize;
    let mut too_close_beams = 0usize;

    for (i, &r32) in scan.ranges.iter().enumerate() {
        // np.arange(total, dtype=float64) * angle_increment, then + angle_min.
        let angle = angle_min + (i as f64) * increment;
        let theta = laser.yaw + angle;
        let (sin_t, cos_t) = (theta.sin(), theta.cos());
        let r = f64::from(r32);
        // `r >= range_min` is false for NaN, as numpy's comparison is.
        let valid = r.is_finite() && r >= range_min;

        if valid {
            let dx = laser.x + r * cos_t - cfg.front_x;
            let dy = laser.y + r * sin_t;
            if dx > 0.0 && dy.atan2(dx).abs() < cfg.cone_half_angle_rad {
                let d = dx.hypot(dy);
                min_front = Some(match min_front {
                    Some(m) if m <= d => m,
                    _ => d,
                });
            }
        }

        // Fail-safe bookkeeping only: which beams LOOK into the front sector.
        // An invalid beam has no range, so the bumper bearing cannot be known;
        // the laser's own bearing in base_link stands in for it.
        if wrap_pi(theta).abs() < cfg.cone_half_angle_rad {
            sector_beams += 1;
            if r == f64::NEG_INFINITY {
                too_close_beams += 1;
            } else if r.is_nan() || (r.is_finite() && r < range_min) {
                invalid_beams += 1;
            }
        }
    }

    FrontEval {
        min_front,
        sector_beams,
        invalid_beams,
        too_close_beams,
        bad_layout,
    }
}

/// The Python's `_tripped()` front test: strict `<`.
pub fn tripped(eval: &FrontEval, cfg: &GuardConfig) -> bool {
    matches!(eval.min_front, Some(m) if m < cfg.threshold_m)
}

/// Decide on one scan. Order: the Python trip first (so `reason == Tripped`
/// whenever the Python would stop), then the fail-safe rules.
pub fn decide(scan: &ScanView<'_>, laser: LaserPose, cfg: &GuardConfig) -> Decision {
    let eval = evaluate_front(scan, laser, cfg);
    let is_tripped = tripped(&eval, cfg);
    let reason = if is_tripped {
        Reason::Tripped
    } else if scan.ranges.is_empty() {
        Reason::EmptyScan
    } else if eval.bad_layout {
        Reason::BadLayout
    } else if eval.too_close_beams > 0 {
        Reason::TooCloseUnquantified
    } else if eval.sector_beams == 0 {
        Reason::NoFrontBeams
    } else if (eval.invalid_beams as f64) > cfg.max_invalid_fraction * (eval.sector_beams as f64) {
        Reason::InvalidBeams
    } else {
        Reason::Clear
    };
    Decision {
        tripped: is_tripped,
        stop: reason.stops(),
        reason,
        eval,
    }
}

/// The decision when there is no current scan to decide on.
pub fn no_data(reason: Reason) -> Decision {
    Decision {
        tripped: false,
        stop: true,
        reason,
        eval: FrontEval {
            min_front: None,
            sector_beams: 0,
            invalid_beams: 0,
            too_close_beams: 0,
            bad_layout: false,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CFG: GuardConfig = GuardConfig {
        front_x: 0.405,
        threshold_m: 0.15,
        cone_half_angle_rad: std::f64::consts::FRAC_PI_4,
        max_invalid_fraction: 0.5,
    };
    const LASER: LaserPose = LaserPose {
        x: 0.305,
        y: 0.0,
        yaw: 0.0,
    };
    // The car's UST-10LX layout (bag 2026-10-15): 1081 beams over +-135 deg.
    const ANGLE_MIN: f32 = -2.356_194_5;
    const INC: f32 = 0.004_363_323;

    fn scan(ranges: &[f32]) -> ScanView<'_> {
        ScanView {
            angle_min: ANGLE_MIN,
            angle_increment: INC,
            range_min: 0.02,
            ranges,
        }
    }

    fn open() -> Vec<f32> {
        vec![30.0; 1081]
    }

    #[test]
    fn open_scan_is_clear() {
        let r = open();
        let d = decide(&scan(&r), LASER, &CFG);
        assert_eq!(d.reason, Reason::Clear);
        assert!(!d.stop && !d.tripped);
        // +-45 deg at 0.25 deg spacing; the two edge beams fall either side of `<`.
        assert!((359..=361).contains(&d.eval.sector_beams));
    }

    #[test]
    fn straight_ahead_trips_by_bumper_distance() {
        // Beam 540 is angle ~0. Laser 0.10 m behind the bumper: a return at
        // 0.24 m is 0.14 m from the bumper (< 0.15), at 0.26 m it is 0.16.
        let mut r = open();
        r[540] = 0.24;
        assert!(decide(&scan(&r), LASER, &CFG).tripped);
        r[540] = 0.26;
        assert!(!decide(&scan(&r), LASER, &CFG).tripped);
    }

    #[test]
    fn return_behind_bumper_is_not_front() {
        let mut r = open();
        r[540] = 0.05; // 0.355 in base_link: behind the bumper face.
        let d = decide(&scan(&r), LASER, &CFG);
        assert!(!d.tripped);
        assert_eq!(d.reason, Reason::Clear);
    }

    #[test]
    fn nan_and_inf_are_skipped_like_the_python() {
        let mut r = open();
        r[540] = f32::NAN;
        r[541] = f32::INFINITY;
        let d = decide(&scan(&r), LASER, &CFG);
        assert!(!d.tripped);
        assert_eq!(d.eval.invalid_beams, 1);
        assert_eq!(d.reason, Reason::Clear);
    }

    #[test]
    fn negative_infinity_in_cone_stops() {
        let mut r = open();
        r[540] = f32::NEG_INFINITY;
        let d = decide(&scan(&r), LASER, &CFG);
        assert!(!d.tripped);
        assert_eq!(d.reason, Reason::TooCloseUnquantified);
        assert!(d.stop);
    }

    #[test]
    fn mostly_nan_front_stops() {
        let r = vec![f32::NAN; 1081];
        let d = decide(&scan(&r), LASER, &CFG);
        assert!(!d.tripped);
        assert_eq!(d.reason, Reason::InvalidBeams);
    }

    #[test]
    fn empty_scan_stops() {
        let d = decide(&scan(&[]), LASER, &CFG);
        assert_eq!(d.reason, Reason::EmptyScan);
        assert!(d.stop);
    }

    #[test]
    fn rear_only_scan_has_no_front_beams() {
        let r = vec![1.0f32; 10];
        let s = ScanView {
            angle_min: 3.0,
            angle_increment: 0.01,
            range_min: 0.02,
            ranges: &r,
        };
        assert_eq!(decide(&s, LASER, &CFG).reason, Reason::NoFrontBeams);
    }

    #[test]
    fn bad_layout_stops() {
        let r = open();
        let s = ScanView {
            angle_min: f32::NAN,
            angle_increment: INC,
            range_min: 0.02,
            ranges: &r,
        };
        assert_eq!(decide(&s, LASER, &CFG).reason, Reason::BadLayout);
    }

    #[test]
    fn missing_tf_fallback_trips_earlier() {
        // 0.30 m straight ahead: from the real mount it is 0.20 m from the
        // bumper (clear); with the laser assumed at base_link it reads as
        // behind the bumper. 0.50 m: 0.40 real, 0.095 with the fallback.
        let mut r = open();
        r[540] = 0.50;
        assert!(!decide(&scan(&r), LASER, &CFG).tripped);
        assert!(decide(&scan(&r), LaserPose::FALLBACK, &CFG).tripped);
    }

    #[test]
    fn wrap_pi_stays_in_range() {
        for a in [-10.0, -3.2, -1.0, 0.0, 1.0, 3.2, 10.0] {
            let w = wrap_pi(a);
            assert!((-std::f64::consts::PI..=std::f64::consts::PI).contains(&w));
            let k = (w - a) / (2.0 * std::f64::consts::PI);
            assert!((k - k.round()).abs() < 1e-12);
        }
    }
}
