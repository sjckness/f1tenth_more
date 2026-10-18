//! The parity harness's case format (tools/rust_parity/costmap_boundary).
//!
//! One JSON object per line: one grid and the poses to evaluate on it.
//! Output: one JSON object per pose. Non-finite floats travel as the strings
//! "inf", "-inf", "nan" (JSON has no such numbers).

use serde::Deserialize;
use serde_json::{json, Value};

use crate::extraction::{extract, Grid, Params, Pose};

#[derive(Debug, Deserialize)]
pub struct Case {
    pub id: String,
    pub width: usize,
    pub height: usize,
    /// info.resolution as the f64 value of its f32.
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub data: Vec<i8>,
    pub poses: Vec<[Value; 3]>,
    pub front_facing_max_deg: f64,
    pub side_window_min_deg: f64,
    pub side_window_max_deg: f64,
    pub occupied_threshold: f64,
    pub max_range_m: f64,
    pub front_x: f64,
    pub front_side_limit_m: f64,
    pub min_front_distance_m: f64,
}

fn num(v: &Value) -> Result<f64, String> {
    match v {
        Value::Number(n) => n.as_f64().ok_or_else(|| format!("{n} is not a float")),
        Value::String(s) => match s.as_str() {
            "nan" => Ok(f64::NAN),
            "inf" => Ok(f64::INFINITY),
            "-inf" => Ok(f64::NEG_INFINITY),
            other => Err(format!("unknown float string {other:?}")),
        },
        other => Err(format!("{other} is not a number")),
    }
}

fn out(x: f64) -> Value {
    if x.is_nan() {
        json!("nan")
    } else if x.is_infinite() {
        json!(if x > 0.0 { "inf" } else { "-inf" })
    } else {
        json!(x)
    }
}

/// Evaluate one case line: one output object per pose.
pub fn run_line(line: &str) -> Result<Vec<Value>, String> {
    let c: Case = serde_json::from_str(line).map_err(|e| e.to_string())?;
    let grid = Grid {
        data: &c.data,
        width: c.width,
        height: c.height,
        resolution: c.resolution,
        origin_x: c.origin_x,
        origin_y: c.origin_y,
    };
    let params = Params {
        front_facing_max_rad: c.front_facing_max_deg.to_radians(),
        side_window_min_rad: c.side_window_min_deg.to_radians(),
        side_window_max_rad: c.side_window_max_deg.to_radians(),
        occupied_threshold: c.occupied_threshold,
        max_range_m: c.max_range_m,
        front_x: c.front_x,
        front_side_limit_m: c.front_side_limit_m,
        min_front_distance_m: c.min_front_distance_m,
    };
    let mut results = Vec::with_capacity(c.poses.len());
    for (i, [x, y, yaw]) in c.poses.iter().enumerate() {
        let pose = Pose {
            x: num(x)?,
            y: num(y)?,
            yaw: num(yaw)?,
        };
        let r = match extract(&grid, pose, &params) {
            Ok(e) => json!({
                "id": c.id, "pose_index": i, "error": Value::Null,
                "constraints": e.constraints().iter()
                    .map(|&(nx, ny, off)| json!([out(nx), out(ny), out(off)]))
                    .collect::<Vec<_>>(),
                // Published as std_msgs/Float32.
                "front_clearance": out(f64::from(e.front_clearance as f32)),
            }),
            Err(err) => json!({
                "id": c.id, "pose_index": i, "error": err.as_str(),
                "constraints": [], "front_clearance": Value::Null,
            }),
        };
        results.push(r);
    }
    Ok(results)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_wall_cell() {
        let mut data = vec![0i8; 100];
        data[5 * 10 + 8] = 100; // row 5, col 8: x = 0.85, y = 0.55 at 0.1 m
        let line = json!({
            "id": "t", "width": 10, "height": 10, "resolution": 0.1,
            "origin_x": 0.0, "origin_y": 0.0, "data": data,
            "poses": [[0.05, 0.55, 0.0], ["nan", 0.0, 0.0]],
            "front_facing_max_deg": 35.0, "side_window_min_deg": 45.0,
            "side_window_max_deg": 135.0, "occupied_threshold": 65.0,
            "max_range_m": 5.0, "front_x": 0.405, "front_side_limit_m": 0.186,
            "min_front_distance_m": 0.01
        })
        .to_string();
        let r = run_line(&line).unwrap();
        assert_eq!(r.len(), 2);
        assert_eq!(r[0]["constraints"].as_array().map(Vec::len), Some(1));
        assert!(r[0]["error"].is_null());
        assert_eq!(r[1]["error"], "pose not finite");
        assert!(r[1]["front_clearance"].is_null());
    }
}
