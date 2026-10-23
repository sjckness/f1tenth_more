//! hard_stop_node's parameters: the Python node's names, types and defaults
//! (f1tenth_control/hard_stop_node.py HardStopNode.__init__), and what the
//! Rust node refuses at startup.
//!
//! Two Python defaults are computed at import time there and have no literal:
//! `footprint_front_x_m` (vehicle_frames.base_front_x(), the URDF bumper) and
//! `footprint_half_width_m` (stack_params swept_clearance_body_half_width_m).
//! ackermann_mux.launch.py passes both to the Rust node (the guard and the
//! costmap node's pattern); declared here with NaN, a missing value refuses
//! the start instead of becoming a silent number.

use crate::decision::{footprint_polygon, Lane, Polygon, LANE_TIMEOUT_S, SELF_MARGIN_M};
use crate::pyfmt::py_fixed;

/// frame_id every stop is stamped with (hard_stop_node.py HARD_STOP_FRAME_ID):
/// the only thing that attributes an actuated command to this node in a bag.
pub const HARD_STOP_FRAME_ID: &str = "base_link/hard_stop";

#[derive(Clone, Debug, PartialEq)]
pub struct NodeParams {
    /// hard_stop_enabled
    pub enabled: bool,
    /// hard_stop_range_m
    pub range_m: f64,
    /// hard_stop_debounce_scans
    pub debounce_scans: i64,
    /// hard_stop_clear_s
    pub clear_s: f64,
    /// hard_stop_republish_hz
    pub republish_hz: f64,
    pub scan_topic: String,
    pub stop_topic: String,
    pub diag_topic: String,
    /// footprint_front_x_m (from the launch file)
    pub front_x_m: f64,
    /// footprint_rear_x_m
    pub rear_x_m: f64,
    /// footprint_half_width_m (from the launch file)
    pub half_width_m: f64,
    /// footprint_self_margin_m
    pub self_margin_m: f64,
    /// laser_x_m
    pub laser_x_m: f64,
    /// hard_stop_allow_reverse
    pub allow_reverse: bool,
    /// lane_timeout_s
    pub lane_timeout_s: f64,
}

impl Default for NodeParams {
    /// The Python's declared defaults; the two launch-supplied values NaN.
    fn default() -> NodeParams {
        NodeParams {
            enabled: true,
            range_m: 0.20,
            debounce_scans: 2,
            clear_s: 0.5,
            republish_hz: 20.0,
            scan_topic: "/scan".to_owned(),
            stop_topic: "/safety_stop".to_owned(),
            diag_topic: "/safety/hard_stop".to_owned(),
            front_x_m: f64::NAN,
            rear_x_m: -0.10,
            half_width_m: f64::NAN,
            self_margin_m: SELF_MARGIN_M,
            laser_x_m: 0.305,
            allow_reverse: true,
            lane_timeout_s: LANE_TIMEOUT_S,
        }
    }
}

impl NodeParams {
    /// The self-hit mask, as the Python builds it.
    pub fn polygon(&self) -> Polygon {
        footprint_polygon(
            self.front_x_m,
            self.rear_x_m,
            self.half_width_m,
            self.self_margin_m,
        )
    }

    /// The republish timer's period: rclpy's `create_timer(1.0 / hz)` takes
    /// `int(float(period) * 1e9)` ns (truncated). None when that is not a
    /// positive whole number of ns.
    pub fn republish_period_ns(&self) -> Option<u64> {
        let ns = (1.0 / self.republish_hz) * 1e9;
        if ns.is_finite() && ns >= 1.0 && ns < u64::MAX as f64 {
            Some(ns as u64)
        } else {
            None
        }
    }

    /// Refuse what the Python cannot run either, plus a missing launch value.
    ///
    /// - footprint_front_x_m / footprint_half_width_m not finite: not passed
    ///   by the launch file (the Python would have used the URDF's numbers).
    /// - hard_stop_republish_hz not giving a positive ns period: the Python
    ///   dies at 0 (ZeroDivisionError), negative or NaN (timer creation);
    ///   inf (a 0 ns period) would make its timer spin, here it is refused.
    ///
    /// Every other value is accepted as the Python accepts it. All errors,
    /// joined with "; ".
    pub fn validate(&self) -> Result<(), String> {
        let mut errors = Vec::new();
        for (name, value) in [
            ("footprint_front_x_m", self.front_x_m),
            ("footprint_half_width_m", self.half_width_m),
        ] {
            if !value.is_finite() {
                errors.push(format!(
                    "{name} is required (ackermann_mux.launch.py passes the value the Python \
                     node takes from vehicle_frames / stack_params), got {value}"
                ));
            }
        }
        if self.republish_period_ns().is_none() {
            errors.push(format!(
                "hard_stop_republish_hz must give a positive timer period, got {}",
                self.republish_hz
            ));
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors.join("; "))
        }
    }

    /// The topics the node subscribes, as rclpy lists them: the scan, plus
    /// the three lanes under the safety lane with hard_stop_allow_reverse.
    pub fn subscriptions(&self) -> Vec<String> {
        let mut topics = vec![self.scan_topic.clone()];
        if self.allow_reverse {
            topics.extend(Lane::LOWER_LANES.iter().map(|(lane, _)| lane.topic()));
        }
        topics
    }

    /// The Python node's startup log line, byte for byte.
    pub fn startup_line(&self) -> String {
        let p = self.polygon();
        let debounce = self.debounce_scans.max(1);
        format!(
            "HARD STOP {}: range <= {} m from the laser at x={} (bumper {}), debounce {} scans, \
             clear hold {} s, self-mask x[{},{}] y[{},{}], stop -> {} as {}, reverse {} past an \
             obstacle ahead",
            if self.enabled { "enabled" } else { "DISABLED" },
            py_fixed(self.range_m, 3, false),
            py_fixed(self.laser_x_m, 3, false),
            py_fixed(self.front_x_m, 3, false),
            debounce,
            py_fixed(self.clear_s, 2, false),
            py_fixed(p.x_min, 3, true),
            py_fixed(p.x_max, 3, true),
            py_fixed(p.y_min, 3, true),
            py_fixed(p.y_max, 3, true),
            self.stop_topic,
            HARD_STOP_FRAME_ID,
            if self.allow_reverse {
                "allowed"
            } else {
                "blocked"
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn launched() -> NodeParams {
        NodeParams {
            front_x_m: 0.405,
            half_width_m: 0.136,
            ..NodeParams::default()
        }
    }

    #[test]
    fn the_launch_values_are_required() {
        let e = NodeParams::default().validate().unwrap_err();
        assert!(e.contains("footprint_front_x_m is required"), "{e}");
        assert!(e.contains("footprint_half_width_m is required"), "{e}");
        assert_eq!(e.matches("; ").count(), 1, "{e}");
        assert!(launched().validate().is_ok());
    }

    #[test]
    fn the_republish_period_is_rclpy_s() {
        assert_eq!(launched().republish_period_ns(), Some(50_000_000));
        let p = |hz| NodeParams {
            republish_hz: hz,
            ..launched()
        };
        assert_eq!(p(3.0).republish_period_ns(), Some(333_333_333));
        for hz in [0.0, -20.0, f64::NAN, f64::INFINITY, 1e10] {
            assert!(p(hz).validate().is_err(), "{hz}");
        }
    }

    #[test]
    fn subscriptions_are_the_scan_and_the_lanes() {
        let mut s = launched().subscriptions();
        s.sort();
        assert_eq!(s, ["/calibration_drive", "/drive", "/scan", "/teleop"]);
        let off = NodeParams {
            allow_reverse: false,
            ..launched()
        };
        assert_eq!(off.subscriptions(), ["/scan"]);
    }

    #[test]
    fn the_startup_line_is_the_python_s() {
        assert_eq!(
            launched().startup_line(),
            "HARD STOP enabled: range <= 0.200 m from the laser at x=0.305 (bumper 0.405), \
             debounce 2 scans, clear hold 0.50 s, self-mask x[-0.120,+0.425] y[-0.156,+0.156], \
             stop -> /safety_stop as base_link/hard_stop, reverse allowed past an obstacle ahead"
        );
    }
}
