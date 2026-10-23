//! The last-resort LiDAR stop's decision, as pure functions: a port of
//! `f1tenth_control/f1tenth_control/hard_stop.py`, no ROS.
//!
//! See that file's docstring for WHAT the stop is for (the backstop for
//! everything perception does not recognise; it stops, it never avoids) and
//! why the self-hit mask exists (the car's own body is inside the LiDAR's
//! field of view: a naive "anything under 0.20 m" latches on the chassis
//! forever). This port changes none of it.
//!
//! Every function keeps the Python's arithmetic, operand order and
//! comparisons, so the same inputs give bit-identical results (Rust never
//! contracts or reassociates float operations; `cos`/`sin` are glibc's libm,
//! as Python's `math` module; tools/rust_parity/hard_stop checks it):
//! - the bearing is `angle_min + i * angle_increment` in f64 (Python floats),
//!   the base_link point `laser_x + r * cos(b)`, `laser_y + r * sin(b)`;
//! - a return is valid when finite and inside `[range_min, range_max]`;
//! - triggering returns are sorted nearest first with a stable sort
//!   (Python's `list.sort` is stable; ranges are finite, so the order is total
//!   and the result unique; -0.0 and 0.0 compare equal in both).

use std::cmp::Ordering;

/// Self-hit margin [m] grown around the footprint polygon
/// (hard_stop.py SELF_MARGIN_M).
pub const SELF_MARGIN_M: f64 = 0.02;

/// The mux lanes' common timeout (hard_stop.py LANE_TIMEOUT_S).
pub const LANE_TIMEOUT_S: f64 = 0.2;

/// The car's footprint in base_link, grown by the margin:
/// (x_min, x_max, y_min, y_max) of the Python tuple.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Polygon {
    pub x_min: f64,
    pub x_max: f64,
    pub y_min: f64,
    pub y_max: f64,
}

impl Polygon {
    /// The Python tuple's order.
    pub fn as_array(&self) -> [f64; 4] {
        [self.x_min, self.x_max, self.y_min, self.y_max]
    }

    pub fn from_array([x_min, x_max, y_min, y_max]: [f64; 4]) -> Polygon {
        Polygon {
            x_min,
            x_max,
            y_min,
            y_max,
        }
    }
}

/// `footprint_polygon(front_x, rear_x, half_width, margin)`.
pub fn footprint_polygon(front_x: f64, rear_x: f64, half_width: f64, margin: f64) -> Polygon {
    Polygon {
        x_min: rear_x - margin,
        x_max: front_x + margin,
        y_min: -(half_width + margin),
        y_max: half_width + margin,
    }
}

/// `is_self_hit(x, y, polygon)`: inside the grown footprint, edges included.
pub fn is_self_hit(x: f64, y: f64, polygon: &Polygon) -> bool {
    polygon.x_min <= x && x <= polygon.x_max && polygon.y_min <= y && y <= polygon.y_max
}

/// One valid return: `range` and `bearing` in the laser's own frame (what
/// the trigger radius is measured in), `x`/`y` in base_link.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    pub range: f64,
    pub bearing: f64,
    pub x: f64,
    pub y: f64,
}

/// The LaserScan fields the decision reads. `T` is `f32` for a message
/// (`sensor_msgs/LaserScan.ranges`) and `f64` for the Python tests' lists;
/// every value is widened to f64 first, as Python's `float(r)` does.
#[derive(Clone, Copy, Debug)]
pub struct ScanView<'a, T> {
    pub ranges: &'a [T],
    pub angle_min: f64,
    pub angle_increment: f64,
    pub range_min: f64,
    pub range_max: f64,
}

/// `scan_points_base_link(...)`: every VALID return, in scan order.
///
/// NaN and inf are no-returns, and a value below range_min is the sensor
/// saying "too close to measure", not "something at 0 m".
pub fn scan_points_base_link<'a, T: Copy + Into<f64>>(
    scan: &ScanView<'a, T>,
    laser_x: f64,
    laser_y: f64,
) -> impl Iterator<Item = Hit> + 'a {
    let ScanView {
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
    } = *scan;
    ranges.iter().enumerate().filter_map(move |(i, &r)| {
        let r: f64 = r.into();
        if !r.is_finite() || r < range_min || r > range_max {
            return None;
        }
        // Python: `angle_min + i * angle_increment`, i an int -> float(i).
        let bearing = angle_min + (i as f64) * angle_increment;
        Some(Hit {
            range: r,
            bearing,
            x: laser_x + r * bearing.cos(),
            y: laser_y + r * bearing.sin(),
        })
    })
}

/// `trigger_range_hits(...)`: returns at or under `trigger_range_m` from the
/// laser origin that are not self-hits, nearest first.
pub fn trigger_range_hits<T: Copy + Into<f64>>(
    scan: &ScanView<'_, T>,
    laser_x: f64,
    polygon: &Polygon,
    trigger_range_m: f64,
) -> Vec<Hit> {
    // The Python calls scan_points_base_link(..., laser_x): laser_y = 0.0.
    let mut hits: Vec<Hit> = scan_points_base_link(scan, laser_x, 0.0)
        .filter(|h| h.range <= trigger_range_m && !is_self_hit(h.x, h.y, polygon))
        .collect();
    // Stable, like list.sort(key=...). Ranges are finite (filtered above), so
    // partial_cmp is never None; Equal keeps the scan order as Python does.
    hits.sort_by(|a, b| a.range.partial_cmp(&b.range).unwrap_or(Ordering::Equal));
    hits
}

/// The mux lanes under /safety_stop (f1tenth_bringup/config/mux.yaml).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Lane {
    Teleop,
    CalibrationDrive,
    Drive,
}

impl Lane {
    /// hard_stop.py LOWER_LANES: (lane, priority), in that order.
    pub const LOWER_LANES: [(Lane, i32); 3] = [
        (Lane::Teleop, 100),
        (Lane::CalibrationDrive, 50),
        (Lane::Drive, 10),
    ];

    pub fn name(self) -> &'static str {
        match self {
            Lane::Teleop => "teleop",
            Lane::CalibrationDrive => "calibration_drive",
            Lane::Drive => "drive",
        }
    }

    pub fn parse(name: &str) -> Option<Lane> {
        Lane::LOWER_LANES
            .iter()
            .map(|&(lane, _)| lane)
            .find(|lane| lane.name() == name)
    }

    /// The topic the node subscribes: `'/' + lane`.
    pub fn topic(self) -> String {
        format!("/{}", self.name())
    }

    fn index(self) -> usize {
        match self {
            Lane::Teleop => 0,
            Lane::CalibrationDrive => 1,
            Lane::Drive => 2,
        }
    }
}

/// `{lane: (receipt time, speed)}` of the node: the last command per lane.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LaneLatest([Option<(f64, f64)>; 3]);

impl LaneLatest {
    pub fn get(&self, lane: Lane) -> Option<(f64, f64)> {
        self.0[lane.index()]
    }

    pub fn set(&mut self, lane: Lane, received: f64, speed: f64) {
        self.0[lane.index()] = Some((received, speed));
    }
}

/// `winning_speed(latest, now, timeout_s=...)`: the speed the mux would pass
/// from the lanes under the safety lane, or None. The highest-priority lane
/// whose last message is not older than the timeout.
pub fn winning_speed(latest: &LaneLatest, now: f64, timeout_s: f64) -> Option<f64> {
    // The Python iterates `sorted(lanes, key=-priority)`; LOWER_LANES is
    // already in that order (unit test below).
    Lane::LOWER_LANES.iter().find_map(|&(lane, _prio)| {
        latest
            .get(lane)
            .filter(|&(received, _)| now - received <= timeout_s)
            .map(|(_, speed)| speed)
    })
}

/// `all_ahead(hits, polygon)`: every triggering return lies ahead of the
/// bumper (the mask's front edge), and there is at least one.
pub fn all_ahead(hits: &[Hit], polygon: &Polygon) -> bool {
    !hits.is_empty() && hits.iter().all(|h| h.x > polygon.x_max)
}

/// `stop_now(engaged, allow_reverse, ahead_only, speed)`: whether to claim
/// the safety lane with a zero right now. With every triggering return ahead
/// of the bumper and hard_stop_allow_reverse, a negative command passes;
/// anything else engaged is stopped.
pub fn stop_now(engaged: bool, allow_reverse: bool, ahead_only: bool, speed: Option<f64>) -> bool {
    if !engaged {
        return false;
    }
    if !allow_reverse || !ahead_only {
        return true;
    }
    !matches!(speed, Some(s) if s < 0.0)
}

/// `HardStopState`: the debounce and the latch, as a state machine over scans.
/// Time comes in from the caller (the node clock), so it owns no clock.
#[derive(Clone, Debug, PartialEq)]
pub struct HardStopState {
    /// `max(int(debounce_scans), 1)`.
    pub debounce_scans: i64,
    pub clear_s: f64,
    pub engaged: bool,
    /// Consecutive scans WITH a hit, reset by any clear scan.
    pub consecutive_hits: i64,
    /// Time of the most recent scan that had a hit.
    pub last_hit_time: Option<f64>,
    /// Engage events since construction.
    pub trigger_count: i64,
}

/// `HardStopState.as_dict(n_hits, nearest)`, before JSON encoding
/// (core::diag_json): the rounded values are Python's `round(x, 4)` and
/// `round(math.degrees(b), 2)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StateRecord {
    pub engaged: bool,
    pub trigger_count: i64,
    pub consecutive_hits: i64,
    pub debounce_scans: i64,
    pub clear_s: f64,
    pub n_hits_this_scan: i64,
    pub nearest_range_m: Option<f64>,
    pub nearest_bearing_deg: Option<f64>,
}

impl HardStopState {
    pub fn new(debounce_scans: i64, clear_s: f64) -> HardStopState {
        HardStopState {
            debounce_scans: debounce_scans.max(1),
            clear_s,
            engaged: false,
            consecutive_hits: 0,
            last_hit_time: None,
            trigger_count: 0,
        }
    }

    /// `update(has_hit, now)`: fold one scan's verdict in; true when engaged.
    pub fn update(&mut self, has_hit: bool, now: f64) -> bool {
        if has_hit {
            self.consecutive_hits = self.consecutive_hits.saturating_add(1);
            self.last_hit_time = Some(now);
            if !self.engaged && self.consecutive_hits >= self.debounce_scans {
                self.engaged = true;
                self.trigger_count = self.trigger_count.saturating_add(1);
            }
            return self.engaged;
        }
        self.consecutive_hits = 0;
        if let (true, Some(last)) = (self.engaged, self.last_hit_time) {
            if now - last >= self.clear_s {
                self.engaged = false;
            }
        }
        self.engaged
    }

    /// `tick(now)`: re-evaluate the clear hold without a scan.
    pub fn tick(&mut self, now: f64) -> bool {
        let Some(last) = self.last_hit_time else {
            return self.engaged;
        };
        if !self.engaged {
            return self.engaged;
        }
        if now - last >= self.clear_s {
            self.engaged = false;
        }
        self.engaged
    }

    /// `as_dict(n_hits, nearest)`: nearest is (range, bearing) or None.
    pub fn record(&self, n_hits: i64, nearest: Option<(f64, f64)>) -> StateRecord {
        StateRecord {
            engaged: self.engaged,
            trigger_count: self.trigger_count,
            consecutive_hits: self.consecutive_hits,
            debounce_scans: self.debounce_scans,
            clear_s: self.clear_s,
            n_hits_this_scan: n_hits,
            nearest_range_m: nearest.map(|(r, _)| crate::pyfmt::py_round(r, 4)),
            nearest_bearing_deg: nearest
                .map(|(_, b)| crate::pyfmt::py_round(crate::pyfmt::degrees(b), 2)),
        }
    }
}

#[cfg(test)]
mod tests {
    //! The pure-function assertions of f1tenth_control/test/test_hard_stop.py,
    //! ported one for one (same names where a Python test maps onto one
    //! function). The node-level tests are in core.rs, and every one of the
    //! 42 also runs as a JSON case against the Python's recorded outputs
    //! (tests/test_cases.rs).
    use super::*;

    // This rig, from the bag's own /tf_static and the URDF.
    const LASER_X: f64 = 0.305;
    const FRONT_X: f64 = 0.405;
    const REAR_X: f64 = -0.10;
    const HALF_WIDTH: f64 = 0.136;
    const TRIGGER_M: f64 = 0.20;
    const N_BEAMS: usize = 1081;
    const RANGE_MIN: f64 = 0.02;
    const RANGE_MAX: f64 = 30.0;

    fn angle_min() -> f64 {
        (-135.0f64).to_radians()
    }

    fn angle_inc() -> f64 {
        0.25f64.to_radians()
    }

    fn poly() -> Polygon {
        footprint_polygon(FRONT_X, REAR_X, HALF_WIDTH, SELF_MARGIN_M)
    }

    fn beam_index(bearing_deg: f64) -> usize {
        ((bearing_deg.to_radians() - angle_min()) / angle_inc()).round() as usize
    }

    fn ranges_with(points: &[(f64, f64)]) -> Vec<f64> {
        let mut r = vec![5.0; N_BEAMS];
        for &(b, rng) in points {
            r[beam_index(b)] = rng;
        }
        r
    }

    fn hits(ranges: &[f64]) -> Vec<Hit> {
        let scan = ScanView {
            ranges,
            angle_min: angle_min(),
            angle_increment: angle_inc(),
            range_min: RANGE_MIN,
            range_max: RANGE_MAX,
        };
        trigger_range_hits(&scan, LASER_X, &poly(), TRIGGER_M)
    }

    #[test]
    fn a_return_at_0p18_m_ahead_is_a_trigger() {
        let found = hits(&ranges_with(&[(0.0, 0.18)]));
        assert_eq!(found.len(), 1);
        assert!((found[0].range - 0.18).abs() < 1e-12);
        assert!((found[0].x - (LASER_X + 0.18)).abs() < 1e-12);
    }

    #[test]
    fn it_engages_within_two_scans() {
        let mut s = HardStopState::new(2, 0.5);
        assert!(!s.update(true, 0.000), "one scan is not evidence");
        assert!(s.update(true, 0.025), "the second scan engages it");
        assert_eq!(s.trigger_count, 1);
    }

    #[test]
    fn a_return_just_outside_the_radius_does_not_trigger() {
        assert!(hits(&ranges_with(&[(0.0, 0.21)])).is_empty());
    }

    #[test]
    fn the_radius_is_inclusive() {
        assert_eq!(hits(&ranges_with(&[(0.0, TRIGGER_M)])).len(), 1);
    }

    #[test]
    fn it_triggers_all_round_not_only_ahead() {
        for b in [0.0, 20.0, -20.0, 40.0, -40.0, 75.0, -75.0, 90.0, -90.0] {
            assert_eq!(hits(&ranges_with(&[(b, 0.18)])).len(), 1, "{b} deg");
        }
    }

    #[test]
    fn the_blind_band_is_exactly_the_car_s_own_body() {
        let (x_max, y_max) = (FRONT_X + SELF_MARGIN_M, HALF_WIDTH + SELF_MARGIN_M);
        for deg in 0..136 {
            let b = f64::from(deg).to_radians();
            let x = LASER_X + 0.18 * b.cos();
            let y = 0.18 * b.sin();
            let inside = x <= x_max && y.abs() <= y_max;
            let found = hits(&ranges_with(&[(f64::from(deg), 0.18)]));
            assert_eq!(found.is_empty(), inside, "{deg} deg at ({x:+.3},{y:+.3})");
        }
    }

    #[test]
    fn a_return_at_the_bumper_face_is_inside_the_mask() {
        assert!(hits(&ranges_with(&[(0.0, 0.12)])).is_empty());
        assert_eq!(hits(&ranges_with(&[(0.0, 0.13)])).len(), 1);
    }

    #[test]
    fn no_returns_at_all_is_not_a_trigger() {
        assert!(hits(&vec![f64::NAN; N_BEAMS]).is_empty());
        assert!(hits(&vec![f64::INFINITY; N_BEAMS]).is_empty());
    }

    #[test]
    fn a_value_under_range_min_is_not_a_trigger() {
        assert!(hits(&ranges_with(&[(0.0, 0.01)])).is_empty());
    }

    #[test]
    fn the_nearest_hit_is_reported_first() {
        let found = hits(&ranges_with(&[(0.0, 0.19), (20.0, 0.14), (-20.0, 0.17)]));
        let r: Vec<f64> = found
            .iter()
            .map(|h| (h.range * 100.0).round() / 100.0)
            .collect();
        assert_eq!(r, vec![0.14, 0.17, 0.19]);
    }

    #[test]
    fn equal_ranges_keep_the_scan_order() {
        let found = hits(&ranges_with(&[(10.0, 0.15), (-10.0, 0.15), (0.0, 0.15)]));
        let b: Vec<f64> = found
            .iter()
            .map(|h| h.bearing.to_degrees().round())
            .collect();
        assert_eq!(b, vec![-10.0, 0.0, 10.0]);
    }

    #[test]
    fn the_polygon_covers_the_body_plus_the_margin() {
        let p = poly();
        assert_eq!(p.x_max, FRONT_X + SELF_MARGIN_M);
        assert_eq!(p.x_min, REAR_X - SELF_MARGIN_M);
        assert_eq!(p.y_max, HALF_WIDTH + SELF_MARGIN_M);
        assert_eq!(p.y_min, -(HALF_WIDTH + SELF_MARGIN_M));
    }

    #[test]
    fn a_return_on_the_rear_deck_is_a_self_hit() {
        assert!(is_self_hit(LASER_X - 0.18, 0.0, &poly()));
        assert!(hits(&ranges_with(&[(135.0, 0.18)])).is_empty());
        assert!(hits(&ranges_with(&[(-135.0, 0.18)])).is_empty());
    }

    #[test]
    fn returns_across_the_chassis_never_trigger() {
        for b in [100.0, 120.0, 134.0, -100.0, -120.0, -134.0] {
            let found = hits(&ranges_with(&[(b, 0.18)]));
            let rad = f64::to_radians(b);
            if is_self_hit(LASER_X + 0.18 * rad.cos(), 0.18 * rad.sin(), &poly()) {
                assert!(found.is_empty(), "{b} deg is the car itself");
            }
        }
    }

    #[test]
    fn the_margin_is_what_decides_a_point_at_the_shell() {
        assert!(is_self_hit(0.0, HALF_WIDTH + 0.01, &poly()));
        assert!(!is_self_hit(
            0.0,
            HALF_WIDTH + SELF_MARGIN_M + 0.01,
            &poly()
        ));
    }

    #[test]
    fn it_stays_engaged_while_the_zone_is_still_occupied() {
        let mut s = engaged();
        for k in 0..40 {
            assert!(s.update(true, 0.05 + 0.025 * f64::from(k)));
        }
    }

    fn engaged() -> HardStopState {
        let mut s = HardStopState::new(2, 0.5);
        s.update(true, 0.000);
        s.update(true, 0.025);
        assert!(s.engaged);
        s
    }

    #[test]
    fn one_clear_scan_does_not_release_it() {
        assert!(engaged().update(false, 0.050));
    }

    #[test]
    fn it_releases_after_the_full_clear_hold() {
        let mut s = engaged();
        assert!(s.update(false, 0.400), "0.375 s of clear is not 0.5");
        assert!(!s.update(false, 0.525), "past 0.5 s it releases");
    }

    #[test]
    fn a_hit_during_the_hold_restarts_the_clock() {
        let mut s = engaged();
        s.update(false, 0.300);
        s.update(true, 0.400);
        assert!(s.update(false, 0.800), "only 0.4 s since the last hit");
        assert!(!s.update(false, 0.950));
    }

    #[test]
    fn re_engaging_counts_as_a_second_trigger() {
        let mut s = engaged();
        assert_eq!(s.trigger_count, 1);
        s.update(false, 0.6);
        assert!(!s.engaged);
        s.update(true, 0.7);
        s.update(true, 0.8);
        assert!(s.engaged && s.trigger_count == 2);
    }

    #[test]
    fn the_hold_is_re_evaluated_without_any_scan() {
        let mut s = engaged();
        assert!(s.tick(0.300));
        assert!(!s.tick(0.600));
    }

    #[test]
    fn debounce_below_one_is_one() {
        assert_eq!(HardStopState::new(0, 0.5).debounce_scans, 1);
        assert_eq!(HardStopState::new(-3, 0.5).debounce_scans, 1);
        let mut s = HardStopState::new(0, 0.5);
        assert!(s.update(true, 0.0));
    }

    #[test]
    fn the_rule() {
        assert!(!stop_now(false, true, true, Some(-0.4)));
        assert!(!stop_now(true, true, true, Some(-0.4)));
        assert!(stop_now(true, true, true, Some(0.0)));
        assert!(stop_now(true, true, true, Some(0.4)));
        assert!(stop_now(true, true, true, None));
        assert!(stop_now(true, true, false, Some(-0.4)));
        assert!(stop_now(true, false, true, Some(-0.4)));
    }

    #[test]
    fn the_mux_s_own_lane_choice() {
        let mut latest = LaneLatest::default();
        latest.set(Lane::Drive, 0.0, -0.4);
        latest.set(Lane::Teleop, 0.0, 0.0);
        assert_eq!(winning_speed(&latest, 0.1, LANE_TIMEOUT_S), Some(0.0));
        assert_eq!(winning_speed(&latest, 0.25, LANE_TIMEOUT_S), None);
        let mut drive = LaneLatest::default();
        drive.set(Lane::Drive, 0.0, -0.4);
        assert_eq!(winning_speed(&drive, 0.1, LANE_TIMEOUT_S), Some(-0.4));
    }

    #[test]
    fn a_stale_higher_lane_gives_way_to_a_fresh_lower_one() {
        let mut latest = LaneLatest::default();
        latest.set(Lane::Teleop, 0.0, 0.3);
        latest.set(Lane::Drive, 1.0, -0.4);
        assert_eq!(winning_speed(&latest, 1.1, LANE_TIMEOUT_S), Some(-0.4));
    }

    #[test]
    fn the_lanes_are_in_priority_order() {
        let prios: Vec<i32> = Lane::LOWER_LANES.iter().map(|&(_, p)| p).collect();
        let mut sorted = prios.clone();
        sorted.sort_by(|a, b| b.cmp(a));
        assert_eq!(prios, sorted);
        for (lane, _) in Lane::LOWER_LANES {
            assert_eq!(Lane::parse(lane.name()), Some(lane));
        }
        assert_eq!(Lane::parse("safety_stop"), None);
        assert_eq!(Lane::Drive.topic(), "/drive");
    }

    #[test]
    fn only_returns_past_the_bumper_are_ahead() {
        assert!(all_ahead(&hits(&ranges_with(&[(0.0, 0.18)])), &poly()));
        let beside = hits(&ranges_with(&[(75.0, 0.18)]));
        assert!(!beside.is_empty() && !all_ahead(&beside, &poly()));
        assert!(!all_ahead(&[], &poly()));
    }

    #[test]
    fn a_message_s_f32_ranges_widen_like_python_floats() {
        let r32: Vec<f32> = ranges_with(&[(0.0, 0.18)])
            .iter()
            .map(|&v| v as f32)
            .collect();
        let scan = ScanView {
            ranges: &r32,
            angle_min: angle_min(),
            angle_increment: angle_inc(),
            range_min: RANGE_MIN,
            range_max: RANGE_MAX,
        };
        let found = trigger_range_hits(&scan, LASER_X, &poly(), TRIGGER_M);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].range, f64::from(0.18f32));
    }
}
