//! costmap_boundary_node (Rust): interface-identical to
//! f1tenth_costmap/costmap_boundary_node.py with use_convex_polytope false.
//!
//! Same node name, parameters, topics, types and QoS:
//!   in   map_topic   (/slam/map, nav_msgs/OccupancyGrid; reliable, transient local, depth 1)
//!        pose_topic  (/ekf_global/odometry/filtered, nav_msgs/Odometry; best effort, depth 10)
//!   out  output_topic          (/costmap/boundaries, f1tenth_messages/BoundaryConstraintArray)
//!        front_clearance_topic (/costmap/front_clearance, std_msgs/Float32)
//! On a timer at extraction_rate_hz on the node clock (sim time under
//! use_sim_time), from whatever map and pose are cached, however old: the
//! constraints (front, left, right, each present or absent) stamped with the
//! node clock now in robot_frame, and the front clearance from the bumper.
//! Until a map AND a pose have arrived: an empty array and no clearance.
//!
//! Differences, all on inputs the Python cannot process (it raises in the
//! timer callback and the node exits): a grid whose data length is not
//! width * height, a non-positive resolution, a non-finite origin or pose.
//! Here those publish the same empty-array / no-clearance pair as "never
//! received" and say why (crate::extraction::InputError).
//!
//! use_convex_polytope true is refused at startup: the polytope path
//! (safe_corridor.py) is not ported. The Python subscribes /mpc/corridor_markers
//! for that path even when it is off; this node does not. The costmap-mode
//! cleanup (component_filter, exclude_tracked_object_cells, both default
//! false) is not ported either: either switch true is refused the same way
//! (crate::config::validate). Their parameters are declared with the
//! Python's defaults, so both nodes take the same keys (the launch test
//! test_costmap_boundary_impl_launch.py checks it); with both switches false
//! the Python does not subscribe obstacles_topic and neither does this node.
//!
//! Extra parameters, set by costmap.launch.py from f1tenth_description's
//! vehicle_frames (the Python imports them): base_front_x_m,
//! front_side_limit_m, min_front_distance_m. timing_report_period_s > 0 logs
//! the tick duration p50/p95/max over each period.

use std::process::ExitCode;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use f1tenth_messages::msg::{BoundaryConstraint, BoundaryConstraintArray};
use nav_msgs::msg::{OccupancyGrid, Odometry};
use rclrs::{
    log_error, log_info, log_warn, Context, CreateBasicExecutor, IntoPrimitiveOptions,
    IntoTimerOptions, Node, RclrsErrorFilter, SpinOptions, ToLogParams,
};
use std_msgs::msg::{Float32, String as StringMsg};

use f1tenth_costmap_boundary_rs::build_info;
use f1tenth_costmap_boundary_rs::config::{validate, StartupParams};
use f1tenth_costmap_boundary_rs::extraction::{extract, yaw_from_quaternion, Grid, Params, Pose};

const NODE_NAME: &str = "costmap_boundary_node";

struct State {
    grid: Option<Arc<OccupancyGrid>>,
    pose: Option<Pose>,
    tick_us: Vec<f64>,
    last_report: Instant,
}

fn lock(state: &Mutex<State>) -> MutexGuard<'_, State> {
    state
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn stamp_now(node: &Node) -> builtin_interfaces::msg::Time {
    let ns = node.get_clock().now().nsec;
    builtin_interfaces::msg::Time {
        sec: ns.div_euclid(1_000_000_000) as i32,
        nanosec: ns.rem_euclid(1_000_000_000) as u32,
    }
}

struct Config {
    params: Params,
    robot_frame: String,
    rate_hz: f64,
    timing_report: Option<Duration>,
    map_topic: String,
    pose_topic: String,
    output_topic: String,
    clearance_topic: String,
    report_topic: String,
}

fn read_config(node: &Node) -> Result<Config, String> {
    let s = |name: &str, default: &str| -> Result<String, String> {
        node.declare_parameter::<Arc<str>>(name)
            .default(Arc::from(default))
            .read_only()
            .map(|p| p.get().to_string())
            .map_err(|e| format!("parameter {name}: {e:?}"))
    };
    let f = |name: &str, default: f64| -> Result<f64, String> {
        node.declare_parameter::<f64>(name)
            .default(default)
            .read_only()
            .map(|p| p.get())
            .map_err(|e| format!("parameter {name}: {e:?}"))
    };
    let i = |name: &str, default: i64| -> Result<i64, String> {
        node.declare_parameter::<i64>(name)
            .default(default)
            .read_only()
            .map(|p| p.get())
            .map_err(|e| format!("parameter {name}: {e:?}"))
    };
    let b = |name: &str, default: bool| -> Result<bool, String> {
        node.declare_parameter::<bool>(name)
            .default(default)
            .read_only()
            .map(|p| p.get())
            .map_err(|e| format!("parameter {name}: {e:?}"))
    };

    // The Python's parameters, names and defaults (costmap_boundary_node.py),
    // one `x("name", default)?` call each: test_costmap_boundary_impl_launch.py
    // reads these calls to check the key parity with the Python node.
    let map_topic = s("map_topic", "/slam/map")?;
    let pose_topic = s("pose_topic", "/ekf_global/odometry/filtered")?;
    let output_topic = s("output_topic", "/costmap/boundaries")?;
    let clearance_topic = s("front_clearance_topic", "/costmap/front_clearance")?;
    let robot_frame = s("robot_frame", "base_link")?;
    let front_facing_max_deg = f("front_facing_max_deg", 35.0)?;
    let side_window_min_deg = f("side_window_min_deg", 45.0)?;
    let side_window_max_deg = f("side_window_max_deg", 135.0)?;
    let occupied_threshold = i("occupied_threshold", 65)?;
    let max_range_m = f("max_range_m", 5.0)?;
    let rate_hz = f("extraction_rate_hz", 10.0)?;
    let use_convex_polytope = b("use_convex_polytope", false)?;
    // Declared for an identical parameter interface; read only by the
    // polytope path, which this node refuses (below).
    s("corridor_markers_topic", "/mpc/corridor_markers")?;
    s("corridor_centerline_ns", "corridor_centerline")?;
    s("map_frame", "map")?;
    s("odom_frame", "odom")?;
    let report_topic = s("polytope_report_topic", "/costmap/safe_corridor_report")?;
    f("polytope_r_local_m", 3.0)?;
    i("polytope_max_faces", 8)?;
    f("polytope_min_seed_clearance_m", 0.32)?;
    f("polytope_min_area_m2", 0.25)?;
    f("polytope_min_covered_fraction", 0.25)?;
    // Costmap-mode cleanup (main-jazzy): the two switches are refused when
    // true (validate below); the rest is read only by the cleanup, as in the
    // Python, which subscribes obstacles_topic only with
    // exclude_tracked_object_cells true.
    let component_filter = b("component_filter", false)?;
    f("min_component_length_m", 0.5)?;
    i("min_component_scans", 2)?;
    let exclude_tracked_object_cells = b("exclude_tracked_object_cells", false)?;
    s("obstacles_topic", "/perception/obstacles_2d")?;
    f("object_footprint_margin_m", 0.12)?;
    // vehicle_frames values, from the launch file. No usable defaults.
    let front_x = f("base_front_x_m", f64::NAN)?;
    let front_side_limit_m = f("front_side_limit_m", f64::NAN)?;
    let min_front_distance_m = f("min_front_distance_m", f64::NAN)?;
    let timing_report_period_s = f("timing_report_period_s", 0.0)?;

    validate(&StartupParams {
        use_convex_polytope,
        component_filter,
        exclude_tracked_object_cells,
        base_front_x_m: front_x,
        front_side_limit_m,
        min_front_distance_m,
        extraction_rate_hz: rate_hz,
    })?;
    Ok(Config {
        params: Params {
            front_facing_max_rad: front_facing_max_deg.to_radians(),
            side_window_min_rad: side_window_min_deg.to_radians(),
            side_window_max_rad: side_window_max_deg.to_radians(),
            occupied_threshold: occupied_threshold as f64,
            max_range_m,
            front_x,
            front_side_limit_m,
            min_front_distance_m,
        },
        robot_frame,
        rate_hz,
        timing_report: (timing_report_period_s > 0.0)
            .then(|| Duration::from_secs_f64(timing_report_period_s)),
        map_topic,
        pose_topic,
        output_topic,
        clearance_topic,
        report_topic,
    })
}

fn percentile(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    // numpy's default (linear interpolation).
    let pos = q * (sorted.len() - 1) as f64;
    let lo = pos.floor() as usize;
    let hi = pos.ceil() as usize;
    let w = pos - lo as f64;
    sorted[lo] * (1.0 - w) + sorted[hi] * w
}

fn run() -> Result<(), String> {
    let err = |e: rclrs::RclrsError| e.to_string();
    let context = Context::default_from_env().map_err(err)?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(NODE_NAME).map_err(err)?;
    // First, so a refused configuration still shows what was built.
    log_info!(
        node.logger(),
        "{NODE_NAME} (Rust) {}",
        build_info::summary()
    );
    let cfg = Arc::new(read_config(&node).inspect_err(|e| {
        log_error!(node.logger(), "configuration error: {e}");
    })?);

    let state = Arc::new(Mutex::new(State {
        grid: None,
        pose: None,
        tick_us: Vec::new(),
        last_report: Instant::now(),
    }));

    let boundary_pub = node
        .create_publisher::<BoundaryConstraintArray>(cfg.output_topic.as_str())
        .map_err(err)?;
    let clearance_pub = node
        .create_publisher::<Float32>(cfg.clearance_topic.as_str())
        .map_err(err)?;
    // Created as in the Python (the polytope report); never published here.
    let _report_pub = node
        .create_publisher::<StringMsg>(cfg.report_topic.as_str())
        .map_err(err)?;

    let map_state = Arc::clone(&state);
    let _map_sub = node
        .create_subscription::<OccupancyGrid, _>(
            cfg.map_topic
                .as_str()
                .reliable()
                .transient_local()
                .keep_last(1),
            move |msg: OccupancyGrid| {
                lock(&map_state).grid = Some(Arc::new(msg));
            },
        )
        .map_err(err)?;

    let pose_state = Arc::clone(&state);
    let _pose_sub = node
        .create_subscription::<Odometry, _>(
            cfg.pose_topic
                .as_str()
                .best_effort()
                .volatile()
                .keep_last(10),
            move |msg: Odometry| {
                let p = &msg.pose.pose;
                let q = &p.orientation;
                lock(&pose_state).pose = Some(Pose {
                    x: p.position.x,
                    y: p.position.y,
                    yaw: yaw_from_quaternion(q.x, q.y, q.z, q.w),
                });
            },
        )
        .map_err(err)?;

    let tick_node = Arc::clone(&node);
    let tick_state = Arc::clone(&state);
    let tick_cfg = Arc::clone(&cfg);
    let period = Duration::from_secs_f64(1.0 / cfg.rate_hz);
    let _timer = node
        .create_timer_repeating(period.node_time(), move || {
            let t0 = Instant::now();
            let c = &*tick_cfg;
            let (grid, pose) = {
                let st = lock(&tick_state);
                (st.grid.clone(), st.pose)
            };
            let mut arr = BoundaryConstraintArray::default();
            arr.header.stamp = stamp_now(&tick_node);
            arr.header.frame_id = c.robot_frame.clone();

            let mut clearance = None;
            match (grid.as_deref(), pose) {
                (Some(g), Some(pose)) => {
                    let view = Grid {
                        data: &g.data,
                        width: g.info.width as usize,
                        height: g.info.height as usize,
                        resolution: f64::from(g.info.resolution),
                        origin_x: g.info.origin.position.x,
                        origin_y: g.info.origin.position.y,
                    };
                    match extract(&view, pose, &c.params) {
                        Ok(e) => {
                            arr.constraints = e
                                .constraints()
                                .into_iter()
                                .map(|(nx, ny, offset)| BoundaryConstraint {
                                    normal: [nx, ny],
                                    offset,
                                })
                                .collect();
                            clearance = Some(e.front_clearance as f32);
                        }
                        Err(why) => log_warn!(
                            tick_node.logger().throttle(Duration::from_secs(5)),
                            "publishing EMPTY boundary constraints, front_clearance withheld -- \
                             unusable input: {} (the Python node raises here)",
                            why.as_str()
                        ),
                    }
                }
                (g, p) => {
                    let mut reasons = Vec::new();
                    if g.is_none() {
                        reasons.push("map_topic: no message ever received");
                    }
                    if p.is_none() {
                        reasons.push("pose_topic: no message ever received");
                    }
                    log_warn!(
                        tick_node.logger().throttle(Duration::from_secs(5)),
                        "publishing EMPTY boundary constraints, front_clearance withheld -- {}",
                        reasons.join("; ")
                    );
                }
            }
            let n = arr.constraints.len();
            if let Err(e) = boundary_pub.publish(arr) {
                log_error!(
                    tick_node.logger().throttle(Duration::from_secs(1)),
                    "boundaries publish failed: {e}"
                );
            }
            if let Some(v) = clearance {
                if let Err(e) = clearance_pub.publish(Float32 { data: v }) {
                    log_error!(
                        tick_node.logger().throttle(Duration::from_secs(1)),
                        "front_clearance publish failed: {e}"
                    );
                }
                log_info!(
                    tick_node.logger().throttle(Duration::from_secs(2)),
                    "n_constraints={n} front_clearance={v:.2}m"
                );
            }

            let mut st = lock(&tick_state);
            st.tick_us.push(t0.elapsed().as_secs_f64() * 1e6);
            if let Some(every) = c.timing_report {
                if st.last_report.elapsed() >= every {
                    let mut v = std::mem::take(&mut st.tick_us);
                    v.sort_by(f64::total_cmp);
                    log_info!(
                        tick_node.logger(),
                        "tick timing over {} ticks: p50 {:.1} us, p95 {:.1} us, max {:.1} us",
                        v.len(),
                        percentile(&v, 0.50),
                        percentile(&v, 0.95),
                        v.last().copied().unwrap_or(f64::NAN)
                    );
                    st.last_report = Instant::now();
                }
            } else if st.tick_us.len() > 4096 {
                st.tick_us.clear();
            }
        })
        .map_err(err)?;

    log_info!(
        node.logger(),
        "{NODE_NAME} (Rust) started -- map_topic={} pose_topic={} extraction_rate_hz={:.1} \
         (publishes every tick from whatever is cached; empty/withheld only until a map \
         and a pose have both arrived)",
        cfg.map_topic,
        cfg.pose_topic,
        cfg.rate_hz
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
