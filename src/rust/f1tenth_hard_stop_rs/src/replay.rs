//! The parity harness's protocol (tools/rust_parity/hard_stop): JSON lines in,
//! one JSON line out per op. The same ops drive the Python reference
//! (hard_stop.py's functions, and a real HardStopNode with its clock, logger
//! and publishers replaced), so every output can be compared exactly.
//!
//! Floats: JSON numbers, or the strings "nan", "inf", "-inf" (in and out);
//! outputs are written with the shortest round-trip digits, so the harness
//! compares bit patterns. Ops (`"op"`):
//! - `fn`: one pure function, `"fn"` + `"args"` (see [`run_fn`]);
//! - `node`: start a node session with `"params"` (ROS parameter names);
//!   later ops act on it, as the node's callbacks do;
//! - `scan`: `_scan_cb` with `ranges` (sent as f32, as in a LaserScan),
//!   `angle_min`, `angle_increment`, `range_min`, `range_max`;
//! - `republish`: `_republish`; `lane`: `_lane_cb` with `lane`, `speed`;
//! - `set_state`: overwrite `last_hit_time` (a test that ages the latch).
//!
//! Each callback op carries `"clock"`: the values successive clock reads
//! return (the last one repeats), and reports `clock_reads`.

use serde_json::{json, Map, Value};

use crate::config::{NodeParams, HARD_STOP_FRAME_ID};
use crate::core::{HardStopCore, LogLine};
use crate::decision::{
    all_ahead, footprint_polygon, is_self_hit, scan_points_base_link, stop_now, trigger_range_hits,
    winning_speed, HardStopState, Hit, Lane, LaneLatest, Polygon, ScanView,
};
use crate::pyfmt::{degrees, json_float, py_fixed, py_repr, py_round};

/// A float for the output: a number, or "nan" / "inf" / "-inf".
pub fn jf(x: f64) -> Value {
    serde_json::Number::from_f64(x).map_or_else(
        || {
            Value::String(
                if x.is_nan() {
                    "nan"
                } else if x > 0.0 {
                    "inf"
                } else {
                    "-inf"
                }
                .to_owned(),
            )
        },
        Value::Number,
    )
}

fn jopt(x: Option<f64>) -> Value {
    x.map_or(Value::Null, jf)
}

/// A float input: a number or "nan" / "inf" / "-inf".
pub fn float_of(v: &Value) -> Result<f64, String> {
    match v {
        Value::Number(n) => n.as_f64().ok_or_else(|| format!("{n} is not a float")),
        Value::String(s) => match s.as_str() {
            "nan" => Ok(f64::NAN),
            "inf" => Ok(f64::INFINITY),
            "-inf" => Ok(f64::NEG_INFINITY),
            other => Err(format!("unknown float string {other:?}")),
        },
        other => Err(format!("{other} is not a float")),
    }
}

fn field<'a>(obj: &'a Value, key: &str) -> Result<&'a Value, String> {
    obj.get(key).ok_or_else(|| format!("missing {key:?}"))
}

fn f(obj: &Value, key: &str) -> Result<f64, String> {
    float_of(field(obj, key)?).map_err(|e| format!("{key}: {e}"))
}

fn b(obj: &Value, key: &str) -> Result<bool, String> {
    field(obj, key)?
        .as_bool()
        .ok_or_else(|| format!("{key} is not a bool"))
}

fn i(obj: &Value, key: &str) -> Result<i64, String> {
    field(obj, key)?
        .as_i64()
        .ok_or_else(|| format!("{key} is not an integer"))
}

fn floats(v: &Value, key: &str) -> Result<Vec<f64>, String> {
    field(v, key)?
        .as_array()
        .ok_or_else(|| format!("{key} is not an array"))?
        .iter()
        .map(float_of)
        .collect()
}

fn polygon_of(v: &Value) -> Result<Polygon, String> {
    let p = floats(v, "polygon")?;
    let arr: [f64; 4] = p
        .try_into()
        .map_err(|_| "polygon needs 4 values".to_owned())?;
    Ok(Polygon::from_array(arr))
}

fn hit_json(h: &Hit) -> Value {
    json!([jf(h.range), jf(h.bearing), jf(h.x), jf(h.y)])
}

fn hits_of(v: &Value) -> Result<Vec<Hit>, String> {
    field(v, "hits")?
        .as_array()
        .ok_or_else(|| "hits is not an array".to_owned())?
        .iter()
        .map(|h| {
            let a = h
                .as_array()
                .ok_or_else(|| "hit is not an array".to_owned())?;
            let g = |k: usize| {
                a.get(k)
                    .ok_or_else(|| "short hit".to_owned())
                    .and_then(float_of)
            };
            Ok(Hit {
                range: g(0)?,
                bearing: g(1)?,
                x: g(2)?,
                y: g(3)?,
            })
        })
        .collect()
}

fn latest_of(v: &Value) -> Result<LaneLatest, String> {
    let mut latest = LaneLatest::default();
    let obj = field(v, "latest")?
        .as_object()
        .ok_or_else(|| "latest is not an object".to_owned())?;
    for (name, tv) in obj {
        let lane = Lane::parse(name).ok_or_else(|| format!("unknown lane {name:?}"))?;
        let a = tv
            .as_array()
            .ok_or_else(|| "lane entry is not [t, speed]".to_owned())?;
        let g = |k: usize| {
            a.get(k)
                .ok_or_else(|| "short lane entry".to_owned())
                .and_then(float_of)
        };
        latest.set(lane, g(0)?, g(1)?);
    }
    Ok(latest)
}

fn state_json(s: &HardStopState) -> Value {
    json!({
        "engaged": s.engaged,
        "consecutive_hits": s.consecutive_hits,
        "last_hit_time": jopt(s.last_hit_time),
        "trigger_count": s.trigger_count,
        "debounce_scans": s.debounce_scans,
        "clear_s": jf(s.clear_s),
    })
}

fn core_json(c: &HardStopCore) -> Value {
    let mut st = state_json(&c.state);
    let lanes: Map<String, Value> = Lane::LOWER_LANES
        .iter()
        .map(|&(lane, _)| {
            let v = c
                .lane_latest
                .get(lane)
                .map_or(Value::Null, |(t, s)| json!([jf(t), jf(s)]));
            (lane.name().to_owned(), v)
        })
        .collect();
    if let Value::Object(m) = &mut st {
        m.insert("ahead_only".to_owned(), Value::Bool(c.ahead_only));
        m.insert("lane_latest".to_owned(), Value::Object(lanes));
    }
    st
}

fn log_json(l: &LogLine) -> Value {
    json!([l.level.as_str(), l.text, jopt(l.throttle_s)])
}

/// The pure functions of hard_stop.py, plus the CPython text forms (pyfmt).
pub fn run_fn(name: &str, a: &Value) -> Result<Value, String> {
    Ok(match name {
        "footprint_polygon" => {
            let p = footprint_polygon(
                f(a, "front_x")?,
                f(a, "rear_x")?,
                f(a, "half_width")?,
                f(a, "margin")?,
            );
            Value::Array(p.as_array().iter().map(|&v| jf(v)).collect())
        }
        "is_self_hit" => Value::Bool(is_self_hit(f(a, "x")?, f(a, "y")?, &polygon_of(a)?)),
        "scan_points_base_link" | "trigger_range_hits" => {
            let ranges = floats(a, "ranges")?;
            let scan = ScanView {
                ranges: &ranges,
                angle_min: f(a, "angle_min")?,
                angle_increment: f(a, "angle_increment")?,
                range_min: f(a, "range_min")?,
                range_max: f(a, "range_max")?,
            };
            let hits: Vec<Hit> = if name == "trigger_range_hits" {
                trigger_range_hits(
                    &scan,
                    f(a, "laser_x")?,
                    &polygon_of(a)?,
                    f(a, "trigger_range_m")?,
                )
            } else {
                scan_points_base_link(&scan, f(a, "laser_x")?, f(a, "laser_y")?).collect()
            };
            Value::Array(hits.iter().map(hit_json).collect())
        }
        "bench_scan" => bench_scan(a)?,
        "all_ahead" => Value::Bool(all_ahead(&hits_of(a)?, &polygon_of(a)?)),
        "stop_now" => {
            let speed = match field(a, "speed")? {
                Value::Null => None,
                v => Some(float_of(v)?),
            };
            Value::Bool(stop_now(
                b(a, "engaged")?,
                b(a, "allow_reverse")?,
                b(a, "ahead_only")?,
                speed,
            ))
        }
        "winning_speed" => jopt(winning_speed(
            &latest_of(a)?,
            f(a, "now")?,
            f(a, "timeout_s")?,
        )),
        "state" => run_state(a)?,
        "py_round" => jf(py_round(f(a, "x")?, ndigits(a, "ndigits")?)),
        "py_repr" => Value::String(py_repr(f(a, "x")?)),
        "json_float" => Value::String(json_float(f(a, "x")?)),
        "py_fixed" => Value::String(py_fixed(f(a, "x")?, ndigits(a, "prec")?, b(a, "plus")?)),
        "degrees" => jf(degrees(f(a, "x")?)),
        other => return Err(format!("unknown fn {other:?}")),
    })
}

/// Timing only (not a parity op): the node's per-scan work (`on_scan` on an
/// f32 scan: hits, latch, claim, log text, diagnostic JSON) `repeat` times on
/// a node with `params`; the median of 5 batches, ns per scan.
fn bench_scan(a: &Value) -> Result<Value, String> {
    let p = params_of(field(a, "params")?)?;
    p.validate()?;
    let ranges: Vec<f32> = floats(a, "ranges")?.iter().map(|&r| r as f32).collect();
    let scan = ScanView {
        ranges: &ranges,
        angle_min: f(a, "angle_min")?,
        angle_increment: f(a, "angle_increment")?,
        range_min: f(a, "range_min")?,
        range_max: f(a, "range_max")?,
    };
    let repeat = usize::try_from(i(a, "repeat")?).map_err(|_| "repeat must be >= 0")?;
    let mut batches = Vec::with_capacity(5);
    let mut sink = 0usize;
    for _ in 0..5 {
        let mut core = HardStopCore::new(&p);
        let mut t = 1000.0;
        let t0 = std::time::Instant::now();
        for _ in 0..repeat {
            t += 0.025;
            if let Some(o) = core.on_scan(&scan, &mut || t) {
                sink += o.diag.len();
            }
        }
        batches.push(t0.elapsed().as_secs_f64() * 1e9 / repeat.max(1) as f64);
    }
    batches.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
    let median = batches.get(2).copied().unwrap_or(f64::NAN);
    Ok(json!({"ns_per_scan": jf(median), "sink": sink}))
}

fn ndigits(a: &Value, key: &str) -> Result<usize, String> {
    usize::try_from(i(a, key)?).map_err(|_| format!("{key} must be >= 0"))
}

/// HardStopState driven by `ops`: ["update", has_hit, now], ["tick", now],
/// ["as_dict", n_hits, [range, bearing] | null]. Returns each op's result and
/// the final state.
fn run_state(a: &Value) -> Result<Value, String> {
    let mut s = HardStopState::new(i(a, "debounce_scans")?, f(a, "clear_s")?);
    let ops = field(a, "ops")?
        .as_array()
        .ok_or_else(|| "ops is not an array".to_owned())?;
    let mut out = Vec::with_capacity(ops.len());
    for op in ops {
        let v = op
            .as_array()
            .ok_or_else(|| "op is not an array".to_owned())?;
        let arg = |k: usize| v.get(k).ok_or_else(|| "short op".to_owned());
        match v.first().and_then(Value::as_str) {
            Some("update") => {
                let has_hit = arg(1)?.as_bool().ok_or("has_hit is not a bool")?;
                out.push(Value::Bool(s.update(has_hit, float_of(arg(2)?)?)));
            }
            Some("tick") => out.push(Value::Bool(s.tick(float_of(arg(1)?)?))),
            Some("as_dict") => {
                let n = arg(1)?.as_i64().ok_or("n_hits is not an integer")?;
                let nearest = match arg(2)? {
                    Value::Null => None,
                    Value::Array(p) if p.len() == 2 => Some((float_of(&p[0])?, float_of(&p[1])?)),
                    other => return Err(format!("nearest {other} is not [range, bearing]")),
                };
                let r = s.record(n, nearest);
                out.push(json!({
                    "engaged": r.engaged,
                    "trigger_count": r.trigger_count,
                    "consecutive_hits": r.consecutive_hits,
                    "debounce_scans": r.debounce_scans,
                    "clear_s": jf(r.clear_s),
                    "n_hits_this_scan": r.n_hits_this_scan,
                    "nearest_range_m": jopt(r.nearest_range_m),
                    "nearest_bearing_deg": jopt(r.nearest_bearing_deg),
                }));
            }
            other => return Err(format!("unknown state op {other:?}")),
        }
    }
    Ok(json!({"results": out, "final": state_json(&s)}))
}

/// NodeParams from `params` (ROS parameter names; missing keys keep the
/// Python defaults, the two launch-supplied values NaN).
pub fn params_of(p: &Value) -> Result<NodeParams, String> {
    let mut n = NodeParams::default();
    let obj = p
        .as_object()
        .ok_or_else(|| "params is not an object".to_owned())?;
    for (key, v) in obj {
        let fl = || float_of(v).map_err(|e| format!("{key}: {e}"));
        let bo = || v.as_bool().ok_or_else(|| format!("{key} is not a bool"));
        let st = || {
            v.as_str()
                .map(str::to_owned)
                .ok_or_else(|| format!("{key} is not a string"))
        };
        match key.as_str() {
            "hard_stop_enabled" => n.enabled = bo()?,
            "hard_stop_range_m" => n.range_m = fl()?,
            "hard_stop_debounce_scans" => {
                n.debounce_scans = v
                    .as_i64()
                    .ok_or_else(|| format!("{key} is not an integer"))?
            }
            "hard_stop_clear_s" => n.clear_s = fl()?,
            "hard_stop_republish_hz" => n.republish_hz = fl()?,
            "scan_topic" => n.scan_topic = st()?,
            "stop_topic" => n.stop_topic = st()?,
            "diag_topic" => n.diag_topic = st()?,
            "footprint_front_x_m" => n.front_x_m = fl()?,
            "footprint_rear_x_m" => n.rear_x_m = fl()?,
            "footprint_half_width_m" => n.half_width_m = fl()?,
            "footprint_self_margin_m" => n.self_margin_m = fl()?,
            "laser_x_m" => n.laser_x_m = fl()?,
            "hard_stop_allow_reverse" => n.allow_reverse = bo()?,
            "lane_timeout_s" => n.lane_timeout_s = fl()?,
            other => return Err(format!("unknown parameter {other:?}")),
        }
    }
    Ok(n)
}

/// The clock of one callback: successive reads return `values` (the last
/// one repeats); counts the reads.
struct ReplayClock {
    values: Vec<f64>,
    reads: usize,
}

impl ReplayClock {
    fn of(op: &Value) -> Result<ReplayClock, String> {
        Ok(ReplayClock {
            values: floats(op, "clock")?,
            reads: 0,
        })
    }

    fn read(&mut self) -> f64 {
        let v = self
            .values
            .get(self.reads)
            .or(self.values.last())
            .copied()
            .unwrap_or(f64::NAN);
        self.reads += 1;
        v
    }
}

/// One replay session: the pure functions, or one node at a time.
#[derive(Default)]
pub struct Session {
    core: Option<HardStopCore>,
}

impl Session {
    fn core(&mut self) -> Result<&mut HardStopCore, String> {
        self.core
            .as_mut()
            .ok_or_else(|| "no node: send a node op first".to_owned())
    }

    /// Run one op; the result carries the op's `id`.
    pub fn run(&mut self, op: &Value) -> Result<Value, String> {
        let id = op.get("id").cloned().unwrap_or(Value::Null);
        let kind = field(op, "op")?
            .as_str()
            .ok_or_else(|| "op is not a string".to_owned())?;
        let mut out = match kind {
            "fn" => {
                let name = field(op, "fn")?
                    .as_str()
                    .ok_or_else(|| "fn is not a string".to_owned())?;
                json!({"out": run_fn(name, field(op, "args")?)?})
            }
            "node" => {
                let p = params_of(field(op, "params")?)?;
                match p.validate() {
                    Ok(()) => {
                        let core = HardStopCore::new(&p);
                        let state = core_json(&core);
                        self.core = Some(core);
                        let mut subs = p.subscriptions();
                        subs.sort();
                        json!({
                            "started": true,
                            "subscriptions": subs,
                            "republish_period_ns": p.republish_period_ns(),
                            "startup": p.startup_line(),
                            "stop_frame_id": HARD_STOP_FRAME_ID,
                            "state": state,
                        })
                    }
                    Err(e) => {
                        self.core = None;
                        json!({"started": false, "error": e})
                    }
                }
            }
            "scan" => {
                let ranges: Vec<f32> = floats(op, "ranges")?.iter().map(|&r| r as f32).collect();
                let scan = ScanView {
                    ranges: &ranges,
                    angle_min: f(op, "angle_min")?,
                    angle_increment: f(op, "angle_increment")?,
                    range_min: f(op, "range_min")?,
                    range_max: f(op, "range_max")?,
                };
                let mut clock = ReplayClock::of(op)?;
                let core = self.core()?;
                let o = core.on_scan(&scan, &mut || clock.read());
                json!({
                    "enabled": o.is_some(),
                    "stop": o.as_ref().is_some_and(|o| o.stop),
                    "diag": o.as_ref().map(|o| o.diag.clone()),
                    "logs": o.as_ref().map_or_else(Vec::new, |o| o.logs.iter().map(log_json).collect()),
                    "hits": o.as_ref().map(|o| o.hits.iter().map(hit_json).collect::<Vec<_>>()),
                    "clock_reads": clock.reads,
                    "state": core_json(core),
                })
            }
            "republish" => {
                let mut clock = ReplayClock::of(op)?;
                let core = self.core()?;
                let o = core.on_republish(&mut || clock.read());
                json!({
                    "enabled": o.is_some(),
                    "stop": o.as_ref().is_some_and(|o| o.stop),
                    "logs": o.as_ref().and_then(|o| o.log.as_ref()).map_or_else(Vec::new, |l| vec![log_json(l)]),
                    "clock_reads": clock.reads,
                    "state": core_json(core),
                })
            }
            "lane" => {
                let name = field(op, "lane")?.as_str().ok_or("lane is not a string")?;
                let lane = Lane::parse(name).ok_or_else(|| format!("unknown lane {name:?}"))?;
                let speed = f(op, "speed")?;
                let mut clock = ReplayClock::of(op)?;
                let core = self.core()?;
                let stop = core.on_lane(lane, speed, &mut || clock.read());
                json!({
                    "stop": stop,
                    "clock_reads": clock.reads,
                    "state": core_json(core),
                })
            }
            "set_state" => {
                let t = match field(op, "last_hit_time")? {
                    Value::Null => None,
                    v => Some(float_of(v)?),
                };
                let core = self.core()?;
                core.state.last_hit_time = t;
                json!({"state": core_json(core)})
            }
            "deps" => json!({"out": crate_dependencies()}),
            other => return Err(format!("unknown op {other:?}")),
        };
        if let Value::Object(m) = &mut out {
            m.insert("id".to_owned(), id);
        }
        Ok(out)
    }
}

/// Every package this crate depends on: Cargo.toml [dependencies] and the
/// package.xml depend tags (test_the_module_imports_nothing_from_the_
/// behaviour_stack, Rust side).
pub fn crate_dependencies() -> Vec<String> {
    let cargo = include_str!("../Cargo.toml");
    let mut deps = Vec::new();
    let mut in_deps = false;
    for line in cargo.lines() {
        let line = line.trim();
        if line.starts_with('[') {
            in_deps = line == "[dependencies]";
            continue;
        }
        if in_deps && !line.starts_with('#') {
            if let Some((name, _)) = line.split_once('=') {
                deps.push(name.trim().to_owned());
            }
        }
    }
    let xml = include_str!("../package.xml");
    for tag in ["depend", "build_depend", "exec_depend"] {
        let open = format!("<{tag}>");
        let close = format!("</{tag}>");
        for part in xml.split(&open).skip(1) {
            if let Some((name, _)) = part.split_once(&close) {
                deps.push(name.trim().to_owned());
            }
        }
    }
    deps
}

/// Bit-exact equality of two outputs: floats by bit pattern (so -0.0 is not
/// 0.0), NaN strings equal, everything else structurally.
pub fn same(a: &Value, b: &Value) -> bool {
    match (a, b) {
        (Value::Number(x), Value::Number(y)) => {
            if x.is_f64() || y.is_f64() {
                match (x.as_f64(), y.as_f64()) {
                    (Some(p), Some(q)) => x.is_f64() == y.is_f64() && p.to_bits() == q.to_bits(),
                    _ => false,
                }
            } else {
                x == y
            }
        }
        (Value::Array(x), Value::Array(y)) => {
            x.len() == y.len() && x.iter().zip(y).all(|(p, q)| same(p, q))
        }
        (Value::Object(x), Value::Object(y)) => {
            x.len() == y.len() && x.iter().all(|(k, v)| y.get(k).is_some_and(|w| same(v, w)))
        }
        _ => a == b,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_node_session_runs() {
        let mut s = Session::default();
        let started = s
            .run(&json!({"op": "node", "id": 1, "params": {
                "footprint_front_x_m": 0.405, "footprint_half_width_m": 0.136}}))
            .unwrap();
        assert_eq!(started["started"], json!(true));
        assert_eq!(started["republish_period_ns"], json!(50_000_000));
        let mut ranges = vec![json!(5.0); 1081];
        ranges[540] = json!(0.18);
        let scan = json!({"op": "scan", "id": 2, "clock": [1000.0], "ranges": ranges,
            "angle_min": -2.356194490192345, "angle_increment": 0.004363323129985824,
            "range_min": 0.02, "range_max": 30.0});
        let first = s.run(&scan).unwrap();
        assert_eq!(first["stop"], json!(false));
        assert_eq!(first["clock_reads"], json!(2));
        let second = s.run(&scan).unwrap();
        assert_eq!(second["stop"], json!(true));
        assert_eq!(second["clock_reads"], json!(3));
        assert_eq!(second["state"]["engaged"], json!(true));
    }

    #[test]
    fn a_refused_node_says_why() {
        let mut s = Session::default();
        let out = s.run(&json!({"op": "node", "params": {}})).unwrap();
        assert_eq!(out["started"], json!(false));
        assert!(s.run(&json!({"op": "republish", "clock": [0.0]})).is_err());
    }

    #[test]
    fn garbage_is_an_error() {
        let mut s = Session::default();
        assert!(s.run(&json!({})).is_err());
        assert!(s
            .run(&json!({"op": "fn", "fn": "nope", "args": {}}))
            .is_err());
        assert!(s
            .run(&json!({"op": "node", "params": {"hard_stop_debounce_scans": 2.5}}))
            .is_err());
        assert!(float_of(&json!("foo")).is_err());
    }

    #[test]
    fn same_compares_bits() {
        assert!(same(&json!(0.0), &json!(0.0)));
        assert!(!same(&json!(0.0), &json!(-0.0)));
        assert!(!same(&json!(1), &json!(1.0)));
        assert!(same(&json!({"a": [1, "nan"]}), &json!({"a": [1, "nan"]})));
    }

    #[test]
    fn nothing_from_the_behaviour_stack() {
        let deps = crate_dependencies();
        assert!(deps.contains(&"rclrs".to_owned()), "{deps:?}");
        assert!(deps.contains(&"sensor_msgs".to_owned()), "{deps:?}");
        for d in &deps {
            for forbidden in ["py_trees", "f1tenth_behavior", "behaviours"] {
                assert!(!d.contains(forbidden), "{d}");
            }
        }
    }
}
