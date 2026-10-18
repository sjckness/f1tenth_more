//! The parity harness's case format (tools/rust_parity/proximity).
//!
//! One JSON object per line. Ranges are numbers, or the strings "nan",
//! "inf", "-inf" (JSON has no non-finite numbers). Every float is the f64
//! value of the message's f32, so the f32 round trip is exact.

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::decision::{decide, GuardConfig, LaserPose, ScanView};

#[derive(Debug, Deserialize)]
pub struct Case {
    pub id: String,
    pub angle_min: f64,
    pub angle_increment: f64,
    pub range_min: f64,
    pub ranges: Vec<Value>,
    /// [x, y, yaw] of the laser in base_link, or null: no /tf_static yet.
    pub laser: Option<[f64; 3]>,
    pub front_x: f64,
    pub threshold_m: f64,
    pub cone_half_angle_deg: f64,
    #[serde(default = "default_invalid_fraction")]
    pub max_invalid_fraction: f64,
}

fn default_invalid_fraction() -> f64 {
    0.5
}

#[derive(Debug, Serialize, PartialEq)]
pub struct Outcome {
    pub id: String,
    pub tripped: bool,
    pub stop: bool,
    pub reason: &'static str,
    pub min_front: Option<f64>,
    pub tf_fallback: bool,
}

fn range_value(v: &Value) -> Result<f32, String> {
    match v {
        Value::Number(n) => n
            .as_f64()
            .map(|x| x as f32)
            .ok_or_else(|| format!("range {n} is not a float")),
        Value::String(s) => match s.as_str() {
            "nan" => Ok(f32::NAN),
            "inf" => Ok(f32::INFINITY),
            "-inf" => Ok(f32::NEG_INFINITY),
            other => Err(format!("unknown range string {other:?}")),
        },
        other => Err(format!("range {other} is neither a number nor a string")),
    }
}

/// Evaluate one case line.
pub fn run_line(line: &str) -> Result<Outcome, String> {
    let case: Case = serde_json::from_str(line).map_err(|e| e.to_string())?;
    let ranges = case
        .ranges
        .iter()
        .map(range_value)
        .collect::<Result<Vec<f32>, String>>()?;
    let scan = ScanView {
        angle_min: case.angle_min as f32,
        angle_increment: case.angle_increment as f32,
        range_min: case.range_min as f32,
        ranges: &ranges,
    };
    let cfg = GuardConfig {
        front_x: case.front_x,
        threshold_m: case.threshold_m,
        cone_half_angle_rad: case.cone_half_angle_deg.to_radians(),
        max_invalid_fraction: case.max_invalid_fraction,
    };
    let laser = case.laser.map(|[x, y, yaw]| LaserPose { x, y, yaw });
    let d = decide(&scan, laser.unwrap_or(LaserPose::FALLBACK), &cfg);
    Ok(Outcome {
        id: case.id,
        tripped: d.tripped,
        stop: d.stop,
        reason: d.reason.as_str(),
        min_front: d.eval.min_front,
        tf_fallback: laser.is_none(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_non_finite_ranges() {
        let line = r#"{"id":"x","angle_min":-0.1,"angle_increment":0.1,"range_min":0.02,
            "ranges":[0.3,"nan","inf"],"laser":[0.305,0.0,0.0],"front_x":0.405,
            "threshold_m":0.15,"cone_half_angle_deg":45.0}"#;
        let o = run_line(&line.replace('\n', " ")).unwrap();
        assert!(!o.tripped);
        assert_eq!(o.reason, "clear");
        assert!(!o.tf_fallback);
    }

    #[test]
    fn rejects_garbage() {
        assert!(run_line("{}").is_err());
        let line = r#"{"id":"x","angle_min":0,"angle_increment":0.1,"range_min":0.02,
            "ranges":["foo"],"laser":null,"front_x":0.405,"threshold_m":0.15,
            "cone_half_angle_deg":45.0}"#;
        assert!(run_line(&line.replace('\n', " ")).is_err());
    }
}
