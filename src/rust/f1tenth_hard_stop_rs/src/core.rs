//! HardStopNode's logic without ROS: `_scan_cb`, `_republish`, `_lane_cb`
//! and `_claim` of f1tenth_control/hard_stop_node.py, statement for statement.
//!
//! main.rs is the ROS glue around it (subscriptions, publishers, timer,
//! logger) and the replay binary drives it from recorded or synthetic event
//! sequences, so the parity harness compares exactly the code the node runs.
//!
//! Time: every place the Python calls `self._now()` (the node clock, ROS time:
//! the system clock on the car, /clock under use_sim_time) calls `clock` here,
//! in the same order and the same number of times. `_claim` evaluates
//! `self._speed()` as an argument, so it reads the clock on every call,
//! engaged or not; `_lane_cb` claims only when enabled (`and` short-circuits);
//! `passing_reverse` claims only when engaged.

use crate::config::NodeParams;
use crate::decision::{
    all_ahead, stop_now, trigger_range_hits, winning_speed, HardStopState, Hit, Lane, LaneLatest,
    Polygon, ScanView, StateRecord,
};
use crate::pyfmt::{degrees, json_float, py_fixed};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    Info,
    Warn,
}

impl Level {
    pub fn as_str(self) -> &'static str {
        match self {
            Level::Info => "info",
            Level::Warn => "warn",
        }
    }
}

/// One log call of the Python node: `get_logger().warn(text)` or `.info`,
/// with `throttle_duration_sec` when it passes one.
#[derive(Clone, Debug, PartialEq)]
pub struct LogLine {
    pub level: Level,
    pub text: String,
    pub throttle_s: Option<f64>,
}

/// What one scan did. Only produced when the node is enabled (the Python
/// returns before anything else otherwise: no stop, no diagnostic).
#[derive(Clone, Debug, PartialEq)]
pub struct ScanOutcome {
    /// Publish one zero-speed stop on the safety lane.
    pub stop: bool,
    /// The /safety/hard_stop payload (std_msgs/String data).
    pub diag: String,
    pub logs: Vec<LogLine>,
    /// The triggering returns, nearest first.
    pub hits: Vec<Hit>,
}

/// What one republish tick did (enabled only).
#[derive(Clone, Debug, PartialEq)]
pub struct TickOutcome {
    pub stop: bool,
    pub log: Option<LogLine>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct HardStopCore {
    pub enabled: bool,
    pub allow_reverse: bool,
    pub lane_timeout_s: f64,
    /// Every triggering return of the last scan WITH hits lay ahead of the bumper.
    pub ahead_only: bool,
    pub lane_latest: LaneLatest,
    pub trigger_range_m: f64,
    pub laser_x: f64,
    pub polygon: Polygon,
    pub state: HardStopState,
}

impl HardStopCore {
    pub fn new(p: &NodeParams) -> HardStopCore {
        HardStopCore {
            enabled: p.enabled,
            allow_reverse: p.allow_reverse,
            lane_timeout_s: p.lane_timeout_s,
            ahead_only: false,
            lane_latest: LaneLatest::default(),
            trigger_range_m: p.range_m,
            laser_x: p.laser_x_m,
            polygon: p.polygon(),
            state: HardStopState::new(p.debounce_scans, p.clear_s),
        }
    }

    /// `_claim(engaged)`: `stop_now(engaged, allow_reverse, ahead_only, _speed())`.
    fn claim(&self, engaged: bool, clock: &mut dyn FnMut() -> f64) -> bool {
        let speed = winning_speed(&self.lane_latest, clock(), self.lane_timeout_s);
        stop_now(engaged, self.allow_reverse, self.ahead_only, speed)
    }

    /// `_scan_cb(msg)`.
    pub fn on_scan<T: Copy + Into<f64>>(
        &mut self,
        scan: &ScanView<'_, T>,
        clock: &mut dyn FnMut() -> f64,
    ) -> Option<ScanOutcome> {
        if !self.enabled {
            return None;
        }
        let hits = trigger_range_hits(scan, self.laser_x, &self.polygon, self.trigger_range_m);
        let was_engaged = self.state.engaged;
        let engaged = self.state.update(!hits.is_empty(), clock());
        if !hits.is_empty() {
            self.ahead_only = all_ahead(&hits, &self.polygon);
        }
        let stop = self.claim(engaged, clock);

        let mut logs = Vec::new();
        if let Some(nearest) = hits.first() {
            let message = format!(
                "HARD STOP {} ({}/{}): {} return(s) within {} m, nearest {} m at {} deg \
                 (base_link {},{})",
                if engaged { "ENGAGED" } else { "pending" },
                self.state.consecutive_hits,
                self.state.debounce_scans,
                hits.len(),
                py_fixed(self.trigger_range_m, 2, false),
                py_fixed(nearest.range, 3, false),
                py_fixed(degrees(nearest.bearing), 1, true),
                py_fixed(nearest.x, 3, true),
                py_fixed(nearest.y, 3, true),
            );
            if engaged && !was_engaged {
                logs.push(LogLine {
                    level: Level::Warn,
                    text: format!("{message} [trigger #{}]", self.state.trigger_count),
                    throttle_s: None,
                });
            } else {
                logs.push(LogLine {
                    level: Level::Warn,
                    text: message,
                    throttle_s: Some(1.0),
                });
            }
        } else if was_engaged && !engaged {
            logs.push(LogLine {
                level: Level::Info,
                text: format!(
                    "hard stop RELEASED after {} s clear (trigger #{})",
                    py_fixed(self.state.clear_s, 2, false),
                    self.state.trigger_count
                ),
                throttle_s: None,
            });
        }

        let record = self.state.record(
            hits.len() as i64,
            hits.first().map(|h| (h.range, h.bearing)),
        );
        let passing_reverse = engaged && !self.claim(engaged, clock);
        let diag = diag_json(
            &record,
            self.allow_reverse,
            self.ahead_only,
            passing_reverse,
        );
        Some(ScanOutcome {
            stop,
            diag,
            logs,
            hits,
        })
    }

    /// `_republish()`: keep the lane claimed while engaged, age the clear hold.
    pub fn on_republish(&mut self, clock: &mut dyn FnMut() -> f64) -> Option<TickOutcome> {
        if !self.enabled {
            return None;
        }
        let was_engaged = self.state.engaged;
        let engaged = self.state.tick(clock());
        if self.claim(engaged, clock) {
            return Some(TickOutcome {
                stop: true,
                log: None,
            });
        }
        let log = (was_engaged && !engaged).then(|| LogLine {
            level: Level::Info,
            text: format!(
                "hard stop RELEASED on the clear hold with no further scans (trigger #{})",
                self.state.trigger_count
            ),
            throttle_s: None,
        });
        Some(TickOutcome { stop: false, log })
    }

    /// `_lane_cb(lane, msg)`: record the command; re-claim at once if it is
    /// not a reverse. `speed` is `float(msg.drive.speed)`. Returns whether to
    /// publish a stop.
    pub fn on_lane(&mut self, lane: Lane, speed: f64, clock: &mut dyn FnMut() -> f64) -> bool {
        self.lane_latest.set(lane, clock(), speed);
        self.enabled && self.claim(self.state.engaged, clock)
    }
}

fn json_opt(v: Option<f64>) -> String {
    v.map_or_else(|| "null".to_owned(), json_float)
}

/// `json.dumps(as_dict(...) | {allow_reverse, ahead_only, passing_reverse},
/// sort_keys=True)`: default separators, keys in sorted order.
pub fn diag_json(
    r: &StateRecord,
    allow_reverse: bool,
    ahead_only: bool,
    passing_reverse: bool,
) -> String {
    format!(
        "{{\"ahead_only\": {ahead_only}, \"allow_reverse\": {allow_reverse}, \"clear_s\": {}, \
         \"consecutive_hits\": {}, \"debounce_scans\": {}, \"engaged\": {}, \
         \"n_hits_this_scan\": {}, \"nearest_bearing_deg\": {}, \"nearest_range_m\": {}, \
         \"passing_reverse\": {passing_reverse}, \"trigger_count\": {}}}",
        json_float(r.clear_s),
        r.consecutive_hits,
        r.debounce_scans,
        r.engaged,
        r.n_hits_this_scan,
        json_opt(r.nearest_bearing_deg),
        json_opt(r.nearest_range_m),
        r.trigger_count,
    )
}

#[cfg(test)]
mod tests {
    //! The node-level tests of f1tenth_control/test/test_hard_stop.py on the
    //! core (the Python drives a real HardStopNode with mocked publishers).
    use super::*;

    const N: usize = 1081;

    fn params() -> NodeParams {
        NodeParams {
            front_x_m: 0.405,
            half_width_m: 0.136,
            ..NodeParams::default()
        }
    }

    struct Harness {
        core: HardStopCore,
        t: f64,
        stops: usize,
        last_diag: Option<String>,
    }

    impl Harness {
        fn new(p: NodeParams) -> Harness {
            Harness {
                core: HardStopCore::new(&p),
                t: 1000.0,
                stops: 0,
                last_diag: None,
            }
        }

        fn scan(&mut self, points: &[(f64, f64)]) {
            let mut r = vec![5.0f32; N];
            let (amin, inc) = ((-135.0f64).to_radians() as f32, 0.25f64.to_radians() as f32);
            for &(b, rng) in points {
                let i = ((b.to_radians() - f64::from(amin)) / f64::from(inc)).round() as usize;
                r[i] = rng as f32;
            }
            let scan = ScanView {
                ranges: &r,
                angle_min: f64::from(amin),
                angle_increment: f64::from(inc),
                range_min: f64::from(0.02f32),
                range_max: 30.0,
            };
            self.t += 0.025;
            let t = self.t;
            if let Some(o) = self.core.on_scan(&scan, &mut || t) {
                self.stops += usize::from(o.stop);
                self.last_diag = Some(o.diag);
            }
        }

        fn republish(&mut self) {
            let t = self.t;
            if let Some(o) = self.core.on_republish(&mut || t) {
                self.stops += usize::from(o.stop);
            }
        }

        fn drive(&mut self, speed: f64) {
            let t = self.t;
            self.stops += usize::from(self.core.on_lane(Lane::Drive, speed, &mut || t));
        }

        fn diag(&self) -> &str {
            self.last_diag.as_deref().unwrap_or("")
        }
    }

    fn self_arc(extra: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut v: Vec<(f64, f64)> = (95..136)
            .flat_map(|b| [(f64::from(b), 0.10), (-f64::from(b), 0.10)])
            .collect();
        v.extend_from_slice(extra);
        v
    }

    #[test]
    fn the_node_publishes_a_zero_speed_stop_on_the_safety_lane() {
        let mut h = Harness::new(params());
        h.scan(&[(0.0, 0.18)]);
        assert_eq!(h.stops, 0, "debounce: not on scan 1");
        h.scan(&[(0.0, 0.18)]);
        assert_eq!(h.stops, 1);
    }

    #[test]
    fn a_scan_that_is_nothing_but_self_hits_never_engages() {
        let mut h = Harness::new(params());
        for _ in 0..20 {
            h.scan(&self_arc(&[]));
        }
        assert_eq!(h.stops, 0);
        assert_eq!(h.core.state.trigger_count, 0);
    }

    #[test]
    fn a_real_obstacle_still_triggers_through_the_self_hits() {
        let mut h = Harness::new(params());
        h.scan(&self_arc(&[(0.0, 0.18)]));
        h.scan(&self_arc(&[(0.0, 0.18)]));
        assert!(h.core.state.engaged);
        assert_eq!(h.stops, 1);
    }

    #[test]
    fn the_node_keeps_the_mux_lane_claimed_while_engaged() {
        let mut h = Harness::new(params());
        h.scan(&[(0.0, 0.18)]);
        h.scan(&[(0.0, 0.18)]);
        let before = h.stops;
        h.republish();
        h.republish();
        assert_eq!(h.stops, before + 2);
    }

    #[test]
    fn the_republish_stops_once_released() {
        let mut h = Harness::new(params());
        h.scan(&[(0.0, 0.18)]);
        h.scan(&[(0.0, 0.18)]);
        h.core.state.last_hit_time = Some(h.t - 10.0);
        h.republish();
        let count = h.stops;
        h.republish();
        assert_eq!(h.stops, count);
    }

    #[test]
    fn it_publishes_state_and_count_every_scan() {
        let mut h = Harness::new(params());
        h.scan(&[]);
        assert_eq!(
            h.diag(),
            "{\"ahead_only\": false, \"allow_reverse\": true, \"clear_s\": 0.5, \
             \"consecutive_hits\": 0, \"debounce_scans\": 2, \"engaged\": false, \
             \"n_hits_this_scan\": 0, \"nearest_bearing_deg\": null, \"nearest_range_m\": null, \
             \"passing_reverse\": false, \"trigger_count\": 0}"
        );
    }

    #[test]
    fn a_trigger_is_reported_with_its_range_and_bearing() {
        let mut h = Harness::new(params());
        h.scan(&[(-30.0, 0.17)]);
        h.scan(&[(-30.0, 0.17)]);
        let d = h.diag();
        assert!(d.contains("\"engaged\": true"), "{d}");
        assert!(d.contains("\"trigger_count\": 1"), "{d}");
        assert!(d.contains("\"nearest_range_m\": 0.17,"), "{d}");
        assert!(d.contains("\"nearest_bearing_deg\": -30.0,"), "{d}");
    }

    #[test]
    fn disabled_never_claims_the_lane() {
        let mut h = Harness::new(NodeParams {
            enabled: false,
            ..params()
        });
        for _ in 0..10 {
            h.scan(&[(0.0, 0.10)]);
        }
        h.republish();
        assert_eq!(h.stops, 0);
        assert_eq!(h.core.state.trigger_count, 0);
        assert!(
            h.last_diag.is_none(),
            "disabled publishes no diagnostic either"
        );
    }

    #[test]
    fn a_reverse_command_passes_an_obstacle_ahead() {
        let mut h = Harness::new(params());
        h.scan(&[(0.0, 0.18)]);
        h.scan(&[(0.0, 0.18)]);
        assert!(h.core.state.engaged && h.core.ahead_only);
        h.drive(-0.4);
        let count = h.stops;
        h.republish();
        h.scan(&[(0.0, 0.18)]);
        assert_eq!(h.stops, count, "quiet while reversing");
        assert!(h.core.state.engaged, "quiet is not a release");
        assert!(h.diag().contains("\"passing_reverse\": true"));
    }

    #[test]
    fn the_first_non_negative_command_reclaims_the_lane_at_once() {
        let mut h = Harness::new(params());
        h.scan(&[(0.0, 0.18)]);
        h.scan(&[(0.0, 0.18)]);
        h.drive(-0.4);
        let count = h.stops;
        h.drive(0.0);
        assert_eq!(h.stops, count + 1);
        h.drive(0.5);
        assert_eq!(h.stops, count + 2);
    }

    #[test]
    fn an_obstacle_beside_the_nose_blocks_reverse_too() {
        let mut h = Harness::new(params());
        h.scan(&[(75.0, 0.18)]);
        h.scan(&[(75.0, 0.18)]);
        h.drive(-0.4);
        let count = h.stops;
        h.republish();
        assert_eq!(h.stops, count + 1);
    }

    #[test]
    fn off_it_blocks_every_direction() {
        let mut h = Harness::new(NodeParams {
            allow_reverse: false,
            ..params()
        });
        h.scan(&[(0.0, 0.18)]);
        h.scan(&[(0.0, 0.18)]);
        h.drive(-0.4);
        let count = h.stops;
        h.republish();
        assert_eq!(h.stops, count + 1);
    }

    #[test]
    fn the_clock_is_read_where_the_python_reads_it() {
        let mut core = HardStopCore::new(&params());
        let mut reads = 0;
        let r = [5.0f32; 10];
        let scan = ScanView {
            ranges: &r,
            angle_min: 0.0,
            angle_increment: 0.01,
            range_min: 0.02,
            range_max: 30.0,
        };
        // Clear scan, not engaged: update + _claim; passing_reverse short-circuits.
        core.on_scan(&scan, &mut || {
            reads += 1;
            0.0
        });
        assert_eq!(reads, 2);
        reads = 0;
        core.on_republish(&mut || {
            reads += 1;
            0.0
        });
        assert_eq!(reads, 2);
        reads = 0;
        core.on_lane(Lane::Drive, 0.1, &mut || {
            reads += 1;
            0.0
        });
        assert_eq!(reads, 2);
    }

    #[test]
    fn released_logs_once() {
        let mut h = Harness::new(params());
        h.scan(&[(0.0, 0.18)]);
        h.scan(&[(0.0, 0.18)]);
        h.t += 1.0;
        let t = h.t;
        let o = h.core.on_republish(&mut || t);
        assert_eq!(
            o.and_then(|o| o.log).map(|l| l.text),
            Some(
                "hard stop RELEASED on the clear hold with no further scans (trigger #1)"
                    .to_owned()
            )
        );
    }
}
