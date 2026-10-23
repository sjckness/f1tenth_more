//! proximity_guard_node: the front check of IsProximityTooClose as its own node.
//!
//! Subscribes /scan and /tf_static (the laser's mount in base_link). On every
//! scan it decides (crate::decision) and publishes the decision on
//! `decision_topic` (diagnostic_msgs/DiagnosticStatus). A watchdog decides
//! `stale` / `no_scan_yet` when scans stop arriving: on the steady clock on
//! the car, on ROS time under use_sim_time (watchdog.rs).
//!
//! mode:=shadow  decisions only, no command of any kind.
//! mode:=active  also a zero-speed AckermannDriveStamped on `stop_topic`
//!               (the ackermann_mux safety_stop lane, priority 200, 0.2 s
//!               timeout) for every stopping decision, frame_id
//!               `stop_frame_id` so a bag attributes it to this guard.
//!
//! guard:=off is "not launched" (behavior_bringup.launch.py); this node has
//! no off mode.

use std::process::ExitCode;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use ackermann_msgs::msg::AckermannDriveStamped;
use diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use rclrs::{
    log_error, log_info, log_warn, ClockType, Context, CreateBasicExecutor, IntoPrimitiveOptions,
    Node, Publisher, QoSProfile, RclrsErrorFilter, SpinOptions, ToLogParams,
};
use sensor_msgs::msg::LaserScan;
use tf2_msgs::msg::TFMessage;

use f1tenth_proximity_guard::build_info;
use f1tenth_proximity_guard::decision::{
    decide, no_data, Decision, GuardConfig, LaserPose, Reason, ScanView,
};
use f1tenth_proximity_guard::static_tf::{StaticTfTree, Transform};
use f1tenth_proximity_guard::watchdog::{ros_age, verdict, ScanAge, StaleClock, Verdict};

const NODE_NAME: &str = "proximity_guard_node";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    Shadow,
    Active,
}

impl Mode {
    fn parse(s: &str) -> Option<Mode> {
        match s {
            "shadow" => Some(Mode::Shadow),
            "active" => Some(Mode::Active),
            _ => None,
        }
    }
    fn as_str(self) -> &'static str {
        match self {
            Mode::Shadow => "shadow",
            Mode::Active => "active",
        }
    }
}

struct Settings {
    mode: Mode,
    cfg: GuardConfig,
    robot_frame: String,
    scan_timeout: Duration,
    /// Steady on the car, the node clock (/clock) under use_sim_time (watchdog.rs).
    stale_clock: StaleClock,
    stop_frame_id: String,
}

struct Topics {
    scan: String,
    tf_static: String,
    decision: String,
    stop: String,
    watchdog: Duration,
}

struct State {
    tf: StaticTfTree,
    started: Instant,
    /// Steady-clock and node-clock time of the last scan's receipt.
    last_scan_rx: Option<(Instant, i64)>,
    /// Sim only: wall-clock gaps over the timeout that were not stale in ROS time.
    sim_pauses: u64,
    in_sim_pause: bool,
}

fn lock(state: &Mutex<State>) -> MutexGuard<'_, State> {
    // Nothing on these paths can panic with the lock held; recover the data
    // anyway rather than stop deciding.
    state
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Node clock now as builtin_interfaces/Time (the message crate's type).
fn stamp_now(node: &Node) -> builtin_interfaces::msg::Time {
    let ns = node.get_clock().now().nsec;
    builtin_interfaces::msg::Time {
        sec: ns.div_euclid(1_000_000_000) as i32,
        nanosec: ns.rem_euclid(1_000_000_000) as u32,
    }
}

fn stamp_ns(t: &builtin_interfaces::msg::Time) -> i64 {
    i64::from(t.sec) * 1_000_000_000 + i64::from(t.nanosec)
}

fn kv(key: &str, value: String) -> KeyValue {
    KeyValue {
        key: key.to_owned(),
        value,
    }
}

fn opt(v: Option<f64>, prec: usize) -> String {
    v.map_or_else(|| "none".to_owned(), |x| format!("{x:.prec$}"))
}

struct DecisionInfo {
    scan_stamp_ns: Option<i64>,
    latency_ms: Option<f64>,
    compute_us: Option<f64>,
    tf_fallback: bool,
    scan_age_ms: f64,
    sim_pauses: u64,
}

fn decision_msg(d: &Decision, s: &Settings, info: &DecisionInfo) -> DiagnosticStatus {
    let level = match d.reason {
        Reason::Clear => DiagnosticStatus::OK,
        Reason::NoScanYet | Reason::Stale => DiagnosticStatus::STALE,
        _ => DiagnosticStatus::ERROR,
    };
    DiagnosticStatus {
        level,
        name: "proximity_guard: front".to_owned(),
        message: d.reason.as_str().to_owned(),
        hardware_id: "lidar".to_owned(),
        values: vec![
            kv("stop", d.stop.to_string()),
            kv("tripped", d.tripped.to_string()),
            kv("reason", d.reason.as_str().to_owned()),
            kv("fail_safe", d.reason.is_fail_safe().to_string()),
            kv("mode", s.mode.as_str().to_owned()),
            kv("min_front_m", opt(d.eval.min_front, 4)),
            kv("threshold_m", format!("{:.4}", s.cfg.threshold_m)),
            kv(
                "scan_stamp_ns",
                info.scan_stamp_ns
                    .map_or_else(|| "none".to_owned(), |v| v.to_string()),
            ),
            kv("latency_ms", opt(info.latency_ms, 3)),
            kv("compute_us", opt(info.compute_us, 1)),
            kv("scan_age_ms", format!("{:.1}", info.scan_age_ms)),
            kv("tf_fallback", info.tf_fallback.to_string()),
            kv("sector_beams", d.eval.sector_beams.to_string()),
            kv("invalid_beams", d.eval.invalid_beams.to_string()),
            kv("too_close_beams", d.eval.too_close_beams.to_string()),
            kv("stale_clock", s.stale_clock.as_str().to_owned()),
            kv("sim_pauses", info.sim_pauses.to_string()),
        ],
    }
}

fn publish_stop(node: &Node, publisher: &Publisher<AckermannDriveStamped>, frame_id: &str) {
    let mut msg = AckermannDriveStamped::default();
    msg.header.stamp = stamp_now(node);
    msg.header.frame_id = frame_id.to_owned();
    msg.drive.speed = 0.0;
    msg.drive.steering_angle = 0.0;
    if let Err(e) = publisher.publish(msg) {
        log_error!(
            node.logger().throttle(Duration::from_secs(1)),
            "safety stop publish failed: {e}"
        );
    }
}

fn publish_decision(
    node: &Node,
    publisher: &Publisher<DiagnosticStatus>,
    d: &Decision,
    s: &Settings,
    info: &DecisionInfo,
) {
    if let Err(e) = publisher.publish(decision_msg(d, s, info)) {
        log_error!(
            node.logger().throttle(Duration::from_secs(1)),
            "decision publish failed: {e}"
        );
    }
}

fn read_settings(node: &Node) -> Result<(Settings, Topics), String> {
    let p_str = |name: &str, default: &str| -> Result<String, String> {
        node.declare_parameter::<Arc<str>>(name)
            .default(Arc::from(default))
            .read_only()
            .map(|p| p.get().to_string())
            .map_err(|e| format!("parameter {name}: {e:?}"))
    };
    let p_f64 = |name: &str, default: f64| -> Result<f64, String> {
        node.declare_parameter::<f64>(name)
            .default(default)
            .read_only()
            .map(|p| p.get())
            .map_err(|e| format!("parameter {name}: {e:?}"))
    };

    let mode_s = p_str("mode", "shadow")?;
    let mode = Mode::parse(&mode_s)
        .ok_or_else(|| format!("mode must be 'shadow' or 'active', got {mode_s:?}"))?;
    // No usable default: the launch file passes vehicle_frames.base_front_x()
    // (the URDF). A missing value must not become a silent 0.
    let front_x = p_f64("base_front_x_m", f64::NAN)?;
    let threshold = p_f64("front_distance_threshold_m", 0.15)?;
    let cone_deg = p_f64("front_cone_half_angle_deg", 45.0)?;
    let invalid = p_f64("max_invalid_fraction", 0.5)?;
    let timeout_ms = p_f64("scan_timeout_ms", 200.0)?;
    let watchdog_ms = p_f64("watchdog_period_ms", 50.0)?;
    if !front_x.is_finite() {
        return Err("base_front_x_m is required (vehicle_frames.base_front_x())".to_owned());
    }
    for (name, v) in [
        ("front_distance_threshold_m", threshold),
        ("front_cone_half_angle_deg", cone_deg),
        ("scan_timeout_ms", timeout_ms),
        ("watchdog_period_ms", watchdog_ms),
    ] {
        if !(v.is_finite() && v > 0.0) {
            return Err(format!("{name} must be a positive number, got {v}"));
        }
    }
    if !(0.0..=1.0).contains(&invalid) {
        return Err(format!(
            "max_invalid_fraction must be in [0, 1], got {invalid}"
        ));
    }
    let settings = Settings {
        mode,
        cfg: GuardConfig {
            front_x,
            threshold_m: threshold,
            cone_half_angle_rad: cone_deg.to_radians(),
            max_invalid_fraction: invalid,
        },
        robot_frame: p_str("robot_frame", "base_link")?,
        scan_timeout: Duration::from_secs_f64(timeout_ms / 1000.0),
        // use_sim_time switches the node clock to ROS time (rclrs TimeSource).
        stale_clock: match node.get_clock().clock_type() {
            ClockType::RosTime => StaleClock::Ros,
            _ => StaleClock::Steady,
        },
        stop_frame_id: p_str("stop_frame_id", "base_link/proximity_guard")?,
    };
    let topics = Topics {
        scan: p_str("scan_topic", "/scan")?,
        tf_static: p_str("tf_static_topic", "/tf_static")?,
        decision: p_str("decision_topic", "/proximity_guard/decision")?,
        stop: p_str("stop_topic", "safety_stop")?,
        watchdog: Duration::from_secs_f64(watchdog_ms / 1000.0),
    };
    Ok((settings, topics))
}

fn run() -> Result<(), String> {
    let context = Context::default_from_env().map_err(|e| e.to_string())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(NODE_NAME).map_err(|e| e.to_string())?;
    // First, so a refused configuration still shows what was built.
    log_info!(node.logger(), "{NODE_NAME} {}", build_info::summary());

    let (settings, topics) = read_settings(&node).inspect_err(|e| {
        log_error!(node.logger(), "configuration error: {e}");
    })?;
    let settings = Arc::new(settings);
    let state = Arc::new(Mutex::new(State {
        tf: StaticTfTree::default(),
        started: Instant::now(),
        last_scan_rx: None,
        sim_pauses: 0,
        in_sim_pause: false,
    }));
    let err = |e: rclrs::RclrsError| e.to_string();

    let decision_pub = node
        .create_publisher::<DiagnosticStatus>(topics.decision.as_str())
        .map_err(err)?;
    let stop_pub = match settings.mode {
        Mode::Active => Some(
            node.create_publisher::<AckermannDriveStamped>(topics.stop.as_str())
                .map_err(err)?,
        ),
        Mode::Shadow => None,
    };

    // /tf_static: reliable, transient local, depth 100 (static_frames.py's TF_STATIC_QOS).
    let tf_state = Arc::clone(&state);
    let _tf_sub = node
        .create_subscription::<TFMessage, _>(
            topics
                .tf_static
                .as_str()
                .keep_last(100)
                .reliable()
                .transient_local(),
            move |msg: TFMessage| {
                let mut st = lock(&tf_state);
                for t in &msg.transforms {
                    let tr = &t.transform.translation;
                    let q = &t.transform.rotation;
                    st.tf.insert(
                        &t.header.frame_id,
                        &t.child_frame_id,
                        Transform {
                            t: [tr.x, tr.y, tr.z],
                            q: [q.x, q.y, q.z, q.w],
                        },
                    );
                }
            },
        )
        .map_err(err)?;

    // /scan: best effort (sensor data), which matches a reliable and a
    // best-effort publisher alike. The Python subscribes reliable, depth 10.
    let scan_node = Arc::clone(&node);
    let scan_state = Arc::clone(&state);
    let scan_settings = Arc::clone(&settings);
    let scan_decision_pub = Arc::clone(&decision_pub);
    let scan_stop_pub = stop_pub.clone();
    let _scan_sub = node
        .create_subscription::<LaserScan, _>(
            topics
                .scan
                .as_str()
                .qos(QoSProfile::sensor_data_default())
                .keep_last(5),
            move |msg: LaserScan| {
                let t0 = Instant::now();
                let s = &*scan_settings;
                let ros_now = scan_node.get_clock().now().nsec;
                let (laser, sim_pauses) = {
                    let mut st = lock(&scan_state);
                    st.last_scan_rx = Some((t0, ros_now));
                    st.in_sim_pause = false;
                    (
                        st.tf.pose(&s.robot_frame, &msg.header.frame_id),
                        st.sim_pauses,
                    )
                };
                let tf_fallback = laser.is_none();
                let scan = ScanView {
                    angle_min: msg.angle_min,
                    angle_increment: msg.angle_increment,
                    range_min: msg.range_min,
                    ranges: &msg.ranges,
                };
                let d = decide(&scan, laser.unwrap_or(LaserPose::FALLBACK), &s.cfg);
                let compute_us = t0.elapsed().as_secs_f64() * 1e6;
                if d.stop {
                    if let Some(publisher) = scan_stop_pub.as_ref() {
                        publish_stop(&scan_node, publisher, &s.stop_frame_id);
                    }
                }
                let scan_stamp = stamp_ns(&msg.header.stamp);
                let now_ns = scan_node.get_clock().now().nsec;
                let info = DecisionInfo {
                    scan_stamp_ns: Some(scan_stamp),
                    latency_ms: Some((now_ns - scan_stamp) as f64 / 1e6),
                    compute_us: Some(compute_us),
                    tf_fallback,
                    scan_age_ms: 0.0,
                    sim_pauses,
                };
                publish_decision(&scan_node, &scan_decision_pub, &d, s, &info);
                if tf_fallback {
                    log_warn!(
                        scan_node.logger().throttle(Duration::from_secs(5)),
                        "no {} <- {:?} on /tf_static yet: front measured as if the LiDAR \
                         sat at {} (trips early, never late), as IsProximityTooClose does",
                        s.robot_frame,
                        msg.header.frame_id,
                        s.robot_frame
                    );
                }
                if d.reason.is_fail_safe() {
                    log_warn!(
                        scan_node.logger().throttle(Duration::from_secs(2)),
                        "front proximity fail-safe: {} (sector {} beams, {} invalid, {} -inf){}",
                        d.reason.as_str(),
                        d.eval.sector_beams,
                        d.eval.invalid_beams,
                        d.eval.too_close_beams,
                        if s.mode == Mode::Active {
                            ": STOPPING"
                        } else {
                            ""
                        }
                    );
                }
            },
        )
        .map_err(err)?;

    // Watchdog: no scan yet, or the last one too old on the judging clock -> stop
    // (watchdog.rs: steady on the car, ROS time under use_sim_time).
    let wd_node = Arc::clone(&node);
    let wd_state = Arc::clone(&state);
    let wd_settings = Arc::clone(&settings);
    let wd_decision_pub = Arc::clone(&decision_pub);
    let wd_stop_pub = stop_pub.clone();
    let _watchdog = node
        .create_timer_repeating(topics.watchdog, move || {
            let s = &*wd_settings;
            let ros_now = wd_node.get_clock().now().nsec;
            let (reason, age, sim_pauses) = {
                let mut st = lock(&wd_state);
                let last = st.last_scan_rx.map(|(t, ros)| ScanAge {
                    steady: t.elapsed(),
                    ros: ros_age(ros_now, ros),
                });
                let v = verdict(s.stale_clock, last, s.scan_timeout);
                let steady_age = last.map_or_else(|| st.started.elapsed(), |a| a.steady);
                let reason = match v {
                    Verdict::NoScanYet => Some(Reason::NoScanYet),
                    Verdict::Stale => Some(Reason::Stale),
                    Verdict::SimPause => {
                        if !st.in_sim_pause {
                            st.in_sim_pause = true;
                            st.sim_pauses += 1;
                            log_info!(
                                wd_node.logger(),
                                "simulator pause #{}: no /scan for {:.0} ms of wall time but \
                                 {:.0} ms of ROS time (timeout {:.0} ms): not stale",
                                st.sim_pauses,
                                steady_age.as_secs_f64() * 1e3,
                                last.and_then(|a| a.ros).unwrap_or_default().as_secs_f64() * 1e3,
                                s.scan_timeout.as_secs_f64() * 1e3
                            );
                        }
                        None
                    }
                    Verdict::Fresh => None,
                };
                (reason, steady_age, st.sim_pauses)
            };
            let Some(reason) = reason else { return };
            let d = no_data(reason);
            if let Some(publisher) = wd_stop_pub.as_ref() {
                publish_stop(&wd_node, publisher, &s.stop_frame_id);
            }
            let info = DecisionInfo {
                scan_stamp_ns: None,
                latency_ms: None,
                compute_us: None,
                tf_fallback: false,
                scan_age_ms: age.as_secs_f64() * 1e3,
                sim_pauses,
            };
            publish_decision(&wd_node, &wd_decision_pub, &d, s, &info);
            log_warn!(
                wd_node.logger().throttle(Duration::from_secs(2)),
                "front proximity {}: no /scan for {:.0} ms (timeout {:.0} ms){}",
                reason.as_str(),
                age.as_secs_f64() * 1e3,
                s.scan_timeout.as_secs_f64() * 1e3,
                if s.mode == Mode::Active {
                    ": STOPPING"
                } else {
                    ""
                }
            );
        })
        .map_err(err)?;

    log_info!(
        node.logger(),
        "{NODE_NAME} started: mode={} threshold={:.3} m from base_front (x={:.3} m), cone +-{:.1} deg, \
         scan timeout {:.0} ms ({} clock), max invalid fraction {:.2}, decisions on {}{}",
        settings.mode.as_str(),
        settings.cfg.threshold_m,
        settings.cfg.front_x,
        settings.cfg.cone_half_angle_rad.to_degrees(),
        settings.scan_timeout.as_secs_f64() * 1e3,
        settings.stale_clock.as_str(),
        settings.cfg.max_invalid_fraction,
        topics.decision,
        match settings.mode {
            Mode::Active => format!(
                ", stops on {} (frame_id {})",
                topics.stop, settings.stop_frame_id
            ),
            Mode::Shadow => " (shadow: no commands)".to_owned(),
        }
    );

    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(err)
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("[{NODE_NAME}] fatal: {e}");
            ExitCode::FAILURE
        }
    }
}
