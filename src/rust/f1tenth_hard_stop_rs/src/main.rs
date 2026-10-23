//! hard_stop_node (Rust): the last-resort LiDAR stop, hard_stop_impl:=rust.
//!
//! The ROS glue around crate::core, which is HardStopNode's logic
//! (f1tenth_control/hard_stop_node.py) statement for statement. Same node
//! name, parameters, topics, QoS and timer as the Python node:
//! - /scan (`scan_topic`): sensor-data QoS (best effort, keep last 5), every
//!   scan, never decimated;
//! - /safety_stop (`stop_topic`): AckermannDriveStamped speed 0, steering 0,
//!   frame_id base_link/hard_stop, on the mux's priority-200 safety lane;
//! - /safety/hard_stop (`diag_topic`): std_msgs/String, the JSON diagnostic
//!   (byte-identical to the Python's, crate::pyfmt);
//! - /teleop, /calibration_drive, /drive (hard_stop_allow_reverse): the lanes
//!   under the safety lane, for the reverse gate;
//! - the republish timer at hard_stop_republish_hz (default 20, 4x the mux's
//!   0.2 s lane timeout), so a dead LiDAR holds the car instead of freeing it.
//!
//! Time is the node clock, as rclpy's: the system clock on the car, /clock
//! under use_sim_time (rclrs TimeSource). The republish timer runs on the
//! node clock too (rclpy timers do; rclrs's default would be the steady
//! clock), so under use_sim_time it follows simulated time like the Python's.

use std::any::Any;
use std::cell::RefCell;
use std::process::ExitCode;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use ackermann_msgs::msg::AckermannDriveStamped;
use rclrs::{
    log_error, log_info, log_warn, Clock, ClockType, Context, CreateBasicExecutor,
    IntoPrimitiveOptions, IntoTimerOptions, Node, Publisher, QoSProfile, RclrsErrorFilter,
    ReadOnlyParameter, SpinOptions, TimerOptions, ToLogParams,
};
use sensor_msgs::msg::LaserScan;

use f1tenth_hard_stop_rs::build_info;
use f1tenth_hard_stop_rs::config::{NodeParams, HARD_STOP_FRAME_ID};
use f1tenth_hard_stop_rs::core::{HardStopCore, Level, LogLine};
use f1tenth_hard_stop_rs::decision::{Lane, ScanView};

const NODE_NAME: &str = "hard_stop_node";

fn lock(core: &Mutex<HardStopCore>) -> MutexGuard<'_, HardStopCore> {
    // Nothing on these paths can panic with the lock held; recover the data
    // anyway rather than stop deciding.
    core.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// `self.get_clock().now().nanoseconds * 1e-9`.
fn now_s(clock: &Clock) -> f64 {
    clock.now().nsec as f64 * 1e-9
}

fn stamp(clock: &Clock) -> builtin_interfaces::msg::Time {
    let ns = clock.now().nsec;
    builtin_interfaces::msg::Time {
        sec: ns.div_euclid(1_000_000_000) as i32,
        nanosec: ns.rem_euclid(1_000_000_000) as u32,
    }
}

struct Outputs {
    node: Node,
    clock: Clock,
    stop: Publisher<AckermannDriveStamped>,
    diag: Publisher<std_msgs::msg::String>,
}

impl Outputs {
    /// `_publish_stop()`.
    fn publish_stop(&self) {
        let mut msg = AckermannDriveStamped::default();
        msg.header.stamp = stamp(&self.clock);
        msg.header.frame_id = HARD_STOP_FRAME_ID.to_owned();
        msg.drive.speed = 0.0;
        msg.drive.steering_angle = 0.0;
        if let Err(e) = self.stop.publish(msg) {
            log_error!(
                self.node.logger().throttle(Duration::from_secs(1)),
                "hard stop publish failed: {e}"
            );
        }
    }

    fn publish_diag(&self, data: String) {
        if let Err(e) = self.diag.publish(std_msgs::msg::String { data }) {
            log_error!(
                self.node.logger().throttle(Duration::from_secs(1)),
                "hard stop diagnostic publish failed: {e}"
            );
        }
    }

    /// The Python's three kinds of log call, one call site each (rclrs
    /// throttles per call site, as rclpy does).
    fn log(&self, line: &LogLine) {
        let logger = self.node.logger();
        match (line.level, line.throttle_s) {
            (Level::Warn, Some(s)) => {
                log_warn!(logger.throttle(Duration::from_secs_f64(s)), "{}", line.text)
            }
            (Level::Warn, None) => log_warn!(logger, "{}", line.text),
            (Level::Info, _) => log_info!(logger, "{}", line.text),
        }
    }
}

/// The declared parameters' handles. rclrs undeclares a parameter when its
/// handle is dropped (rclrs 0.7 parameter.rs, `Drop for ReadOnlyParameter`):
/// kept for the node's lifetime so `ros2 param list/get` show all of them, as
/// they show the Python node's.
type Declared = RefCell<Vec<Box<dyn Any>>>;

fn read_params(node: &Node, keep: &Declared) -> Result<NodeParams, String> {
    fn held<T: rclrs::ParameterVariant + 'static, V>(
        keep: &Declared,
        p: ReadOnlyParameter<T>,
        v: V,
    ) -> V {
        keep.borrow_mut().push(Box::new(p));
        v
    }
    let s = |name: &str, default: &str| -> Result<String, String> {
        node.declare_parameter::<Arc<str>>(name)
            .default(Arc::from(default))
            .read_only()
            .map(|p| {
                let v = p.get().to_string();
                held(keep, p, v)
            })
            .map_err(|e| format!("parameter {name}: {e:?}"))
    };
    let f = |name: &str, default: f64| -> Result<f64, String> {
        node.declare_parameter::<f64>(name)
            .default(default)
            .read_only()
            .map(|p| {
                let v = p.get();
                held(keep, p, v)
            })
            .map_err(|e| format!("parameter {name}: {e:?}"))
    };
    let i = |name: &str, default: i64| -> Result<i64, String> {
        node.declare_parameter::<i64>(name)
            .default(default)
            .read_only()
            .map(|p| {
                let v = p.get();
                held(keep, p, v)
            })
            .map_err(|e| format!("parameter {name}: {e:?}"))
    };
    let b = |name: &str, default: bool| -> Result<bool, String> {
        node.declare_parameter::<bool>(name)
            .default(default)
            .read_only()
            .map(|p| {
                let v = p.get();
                held(keep, p, v)
            })
            .map_err(|e| format!("parameter {name}: {e:?}"))
    };

    // The Python's parameters, names, types and defaults (hard_stop_node.py),
    // one `x("name", default)?` call each: test_hard_stop_impl_launch.py
    // reads these calls to check the key parity with the Python node.
    let p = NodeParams {
        enabled: b("hard_stop_enabled", true)?,
        range_m: f("hard_stop_range_m", 0.20)?,
        debounce_scans: i("hard_stop_debounce_scans", 2)?,
        clear_s: f("hard_stop_clear_s", 0.5)?,
        republish_hz: f("hard_stop_republish_hz", 20.0)?,
        scan_topic: s("scan_topic", "/scan")?,
        stop_topic: s("stop_topic", "/safety_stop")?,
        diag_topic: s("diag_topic", "/safety/hard_stop")?,
        // From the launch file (the Python's vehicle_frames / stack_params
        // defaults). No usable default: refused when missing (config.rs).
        front_x_m: f("footprint_front_x_m", f64::NAN)?,
        rear_x_m: f("footprint_rear_x_m", -0.10)?,
        half_width_m: f("footprint_half_width_m", f64::NAN)?,
        self_margin_m: f("footprint_self_margin_m", 0.02)?,
        laser_x_m: f("laser_x_m", 0.305)?,
        allow_reverse: b("hard_stop_allow_reverse", true)?,
        lane_timeout_s: f("lane_timeout_s", 0.2)?,
    };
    p.validate()?;
    Ok(p)
}

fn run() -> Result<(), String> {
    let context = Context::default_from_env().map_err(|e| e.to_string())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(NODE_NAME).map_err(|e| e.to_string())?;
    // First, so a refused configuration still shows what was built.
    log_info!(
        node.logger(),
        "{NODE_NAME} (rust) {}",
        build_info::summary()
    );

    let declared = Declared::default();
    let params = read_params(&node, &declared).inspect_err(|e| {
        log_error!(node.logger(), "configuration error: {e}");
    })?;
    let err = |e: rclrs::RclrsError| e.to_string();
    // use_sim_time switches the node clock to ROS time (rclrs TimeSource,
    // attached when the node was created).
    let clock = node.get_clock();
    let clock_kind = match clock.clock_type() {
        ClockType::RosTime => "ROS time (/clock)",
        ClockType::SystemTime => "system time",
        ClockType::SteadyTime => "steady time",
    };

    let out = Arc::new(Outputs {
        node: Arc::clone(&node),
        clock: clock.clone(),
        stop: node
            .create_publisher::<AckermannDriveStamped>(
                params.stop_topic.as_str().keep_last(10).reliable(),
            )
            .map_err(err)?,
        diag: node
            .create_publisher::<std_msgs::msg::String>(
                params.diag_topic.as_str().keep_last(10).reliable(),
            )
            .map_err(err)?,
    });
    let core = Arc::new(Mutex::new(HardStopCore::new(&params)));

    // /scan: qos_profile_sensor_data, as the Python subscribes.
    let scan_core = Arc::clone(&core);
    let scan_out = Arc::clone(&out);
    let _scan_sub = node
        .create_subscription::<LaserScan, _>(
            params
                .scan_topic
                .as_str()
                .qos(QoSProfile::sensor_data_default())
                .keep_last(5),
            move |msg: LaserScan| {
                let scan = ScanView {
                    ranges: &msg.ranges,
                    angle_min: f64::from(msg.angle_min),
                    angle_increment: f64::from(msg.angle_increment),
                    range_min: f64::from(msg.range_min),
                    range_max: f64::from(msg.range_max),
                };
                let mut core = lock(&scan_core);
                let Some(o) = core.on_scan(&scan, &mut || now_s(&scan_out.clock)) else {
                    return;
                };
                if o.stop {
                    scan_out.publish_stop();
                }
                for line in &o.logs {
                    scan_out.log(line);
                }
                scan_out.publish_diag(o.diag);
            },
        )
        .map_err(err)?;

    // The lanes under the safety lane (mux.yaml), for the reverse gate.
    let mut lane_subs = Vec::new();
    if params.allow_reverse {
        for (lane, _prio) in Lane::LOWER_LANES {
            let topic = lane.topic();
            let lane_core = Arc::clone(&core);
            let lane_out = Arc::clone(&out);
            lane_subs.push(
                node.create_subscription::<AckermannDriveStamped, _>(
                    topic.as_str().keep_last(10).reliable(),
                    move |msg: AckermannDriveStamped| {
                        let speed = f64::from(msg.drive.speed);
                        let stop =
                            lock(&lane_core).on_lane(lane, speed, &mut || now_s(&lane_out.clock));
                        if stop {
                            lane_out.publish_stop();
                        }
                    },
                )
                .map_err(err)?,
            );
        }
    }

    // The republish timer, on the node clock (see the module docs).
    let period = Duration::from_nanos(params.republish_period_ns().unwrap_or(50_000_000));
    let tick_core = Arc::clone(&core);
    let tick_out = Arc::clone(&out);
    let _republish = node
        .create_timer_repeating(TimerOptions::new(period).node_time(), move || {
            let Some(o) = lock(&tick_core).on_republish(&mut || now_s(&tick_out.clock)) else {
                return;
            };
            if o.stop {
                tick_out.publish_stop();
            }
            if let Some(line) = &o.log {
                tick_out.log(line);
            }
        })
        .map_err(err)?;

    log_info!(node.logger(), "{}", params.startup_line());
    log_info!(
        node.logger(),
        "{NODE_NAME} (rust): clock {clock_kind}, republish every {} ns on it, subscribed {:?}",
        period.as_nanos(),
        params.subscriptions()
    );

    let spun = executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(err);
    drop(declared);
    spun
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
