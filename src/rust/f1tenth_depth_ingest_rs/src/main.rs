//! depth_ingest_node (Rust): the one reader of the ZED depth frame,
//! depth_source:=ingest (detection.launch.py).
//!
//! Subscribes, once:
//! - `depth_topic` (sensor_msgs/Image 32FC1): keep last 10, reliable, every
//!   frame (detection_3d_node's synchronizer input QoS: the depth history
//!   must cover the detector's latency);
//! - `depth_info_topic` (CameraInfo, keep last 10), `detections_topic`
//!   (Detection2DArray, keep last 10), `masks_topic` (mono8, keep last 10;
//!   only with use_mask_depth, as the Python's 3-way sync), and
//!   `/perception/scene_labels` (newest frame only, only with
//!   enable_scene_labels), all as detection_3d_node subscribes them;
//!
//! and publishes, on the node clock (/clock under use_sim_time):
//! - `detection_depths_topic` (f1tenth_messages/DetectionDepthArray), once
//!   per set the ApproximateTimeSynchronizer port pairs: detection_3d_node's
//!   per-detection medians (crate::ingest::DetectionIngest);
//! - `front_roi_topic` (f1tenth_messages/DepthRoi), once per depth frame:
//!   front_clearance_node's ROI, floor sample and, on its 5 Hz ticks, the
//!   whole frame (crate::ingest::RoiIngest).
//!
//! Parameters carry the consumers' own names and defaults (detection_3d_node
//! and front_clearance_node); the launch file passes both nodes the same
//! values. test_depth_ingest_launch.py checks the key parity.

use std::any::Any;
use std::cell::RefCell;
use std::process::ExitCode;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use f1tenth_messages::msg::rmw::DepthRoi as RmwDepthRoi;
use f1tenth_messages::msg::{DetectionDepth, DetectionDepthArray};
use rclrs::{
    log_error, log_info, log_warn, Clock, ClockType, Context, CreateBasicExecutor,
    IntoPrimitiveOptions, Node, Publisher, QoSProfile, RclrsErrorFilter, ReadOnlyParameter,
    SpinOptions, ToLogParams,
};
use rosidl_runtime_rs::Message;
use sensor_msgs::msg::rmw::Image as RmwImage;
use sensor_msgs::msg::CameraInfo;
use vision_msgs::msg::Detection2DArray;

use f1tenth_depth_ingest_rs::build_info;
use f1tenth_depth_ingest_rs::detection::{BoxPx, SampleParams};
use f1tenth_depth_ingest_rs::image::ImageParts;
use f1tenth_depth_ingest_rs::ingest::{
    DetectionIngest, DetectionParams, DepthStatus, RoiIngest, RoiOutcome,
};
use f1tenth_depth_ingest_rs::roi::RoiParams;
use f1tenth_depth_ingest_rs::scene::stamp_s;
use f1tenth_depth_ingest_rs::sync::{stamp_ns, ApproxSync};

const NODE_NAME: &str = "depth_ingest_node";

/// One synchronizer input. Order = the Python's [det_sub, depth_sub, mask_sub].
#[derive(Clone)]
enum Input {
    Det(Arc<Detection2DArray>),
    Depth(Arc<RmwImage>),
    Mask(Arc<RmwImage>),
}

struct Params {
    depth_topic: String,
    depth_info_topic: String,
    detections_topic: String,
    masks_topic: String,
    scene_labels_topic: String,
    detection_depths_topic: String,
    front_roi_topic: String,
    detection_depths_enabled: bool,
    front_roi_enabled: bool,
    det: DetectionParams,
    roi: RoiParams,
}

#[derive(Default)]
struct Counters {
    det2d_in: u64,
    depth_in: u64,
    mask_in: u64,
    synced: u64,
    roi_out: u64,
    det_out: u64,
}

struct State {
    det: DetectionIngest,
    roi: RoiIngest,
    sync: ApproxSync<Input>,
    counts: Counters,
}

fn lock(s: &Mutex<State>) -> MutexGuard<'_, State> {
    s.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// `self.get_clock().now().nanoseconds * 1e-9`.
fn now_s(clock: &Clock) -> f64 {
    clock.now().nsec as f64 * 1e-9
}

/// The decoders' view of a received image (the encoding copied out).
fn with_parts<R>(img: &RmwImage, f: impl FnOnce(&ImageParts<'_>) -> R) -> R {
    let encoding = img.encoding.to_string();
    let parts = ImageParts {
        height: img.height,
        width: img.width,
        encoding: &encoding,
        is_bigendian: img.is_bigendian,
        step: img.step,
        data: img.data.as_slice(),
    };
    f(&parts)
}

type Declared = RefCell<Vec<Box<dyn Any>>>;

/// The declared parameters, handles kept for the node's lifetime (rclrs 0.7
/// undeclares a parameter when its handle drops; R4 found it).
fn read_params(node: &Node, keep: &Declared) -> Result<Params, String> {
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

    // One `x("name", default)?` call per parameter: test_depth_ingest_launch.py
    // reads these and checks each against the consumer that owns the name.
    let p = Params {
        // detection_3d_node's names and defaults.
        detections_topic: s("detections_topic", "/camera/detections")?,
        depth_topic: s("depth_topic", "/zed2/zed_node/depth/depth_registered")?,
        depth_info_topic: s("depth_info_topic", "/zed2/zed_node/depth/camera_info")?,
        masks_topic: s("masks_topic", "/camera/detection_masks")?,
        det: DetectionParams {
            sync_slop: f("sync_slop", 0.1)?,
            sync_queue_size: i("sync_queue_size", 15)?,
            sample: SampleParams {
                center_fraction: f("center_fraction", 0.5)?,
                use_mask_depth: b("use_mask_depth", false)?,
                // stack_params enable_scene_labels (true); shared by both consumers.
                scene_enabled: b("enable_scene_labels", true)?,
                scene_min_other_px: i("scene_min_other_px", 20)?,
            },
            scene_labels_max_age_s: f("scene_labels_max_age_s", 0.3)?,
        },
        // front_clearance_node's names and defaults.
        roi: {
            let mut r = RoiParams {
                roi_half_width_px: i("roi_half_width_px", 90)?,
                roi_half_height_px: i("roi_half_height_px", 35)?,
                roi_shift_up_px: i("roi_shift_up_px", 0)?,
                ground_removal_enabled: b("ground_removal_enabled", false)?,
                ground_stride: i("ground_fit_stride_px", 4)?,
                scene_mask_enabled: b("scene_mask_enabled", true)?,
                scene_mask_rate_hz: f("scene_mask_rate_hz", 5.0)?,
                scene_labels_enabled: false,
                scene_rate_hz: f("scene_rate_hz", 5.0)?,
            };
            r.ground_stride = r.ground_stride.max(1);
            r
        },
        // This node's own.
        scene_labels_topic: s("scene_labels_topic", "/perception/scene_labels")?,
        detection_depths_topic: s(
            "detection_depths_topic",
            "/perception/depth_ingest/detection_depths",
        )?,
        front_roi_topic: s("front_roi_topic", "/perception/depth_ingest/front_roi")?,
        detection_depths_enabled: b("detection_depths_enabled", true)?,
        front_roi_enabled: b("front_roi_enabled", true)?,
    };
    p.det.validate()?;
    let mut p = p;
    // front_clearance's scene labels are the same stack switch.
    p.roi.scene_labels_enabled = p.det.sample.scene_enabled;
    Ok(p)
}

struct Outputs {
    node: Node,
    det_pub: Option<Publisher<DetectionDepthArray>>,
    roi_pub: Option<Publisher<RmwDepthRoi>>,
}

impl Outputs {
    fn publish_roi(&self, msg: &RmwDepthRoi) {
        if let Some(p) = &self.roi_pub {
            if let Err(e) = p.publish(msg) {
                log_error!(
                    self.node.logger().throttle(Duration::from_secs(5)),
                    "front_roi publish failed: {e}"
                );
            }
        }
    }

    fn publish_det(&self, msg: DetectionDepthArray) {
        if let Some(p) = &self.det_pub {
            if let Err(e) = p.publish(msg) {
                log_error!(
                    self.node.logger().throttle(Duration::from_secs(5)),
                    "detection_depths publish failed: {e}"
                );
            }
        }
    }
}

/// DepthRoi for one depth frame.
fn roi_message(state: &mut State, img: &RmwImage, now: f64) -> RmwDepthRoi {
    let outcome = with_parts(img, |parts| state.roi.frame(parts, now));
    let p = &state.roi.params;
    let mut msg = RmwDepthRoi {
        header: img.header.clone(),
        scene_mask_enabled: p.scene_mask_enabled,
        scene_mask_rate_hz: p.scene_mask_rate_hz,
        scene_labels_enabled: p.scene_labels_enabled,
        scene_rate_hz: p.scene_rate_hz,
        depth_in: state.counts.depth_in,
        ..RmwDepthRoi::default()
    };
    match outcome {
        RoiOutcome::DecodeFailed(e) => {
            msg.status = 1;
            msg.error = rosidl_runtime_rs::String::from(e.as_str());
        }
        RoiOutcome::Frame(f) => {
            let (y0, y1, x0, x1) = f.bounds;
            let to_i32 = |v: i64| i32::try_from(v).unwrap_or(i32::MAX);
            msg.width = f.width;
            msg.height = f.height;
            msg.roi_y0 = to_i32(y0);
            msg.roi_y1 = to_i32(y1);
            msg.roi_x0 = to_i32(x0);
            msg.roi_x1 = to_i32(x1);
            msg.roi = rosidl_runtime_rs::Sequence::from(f.roi.as_slice());
            msg.ground_stride = f.ground_stride;
            msg.ground = rosidl_runtime_rs::Sequence::from(f.ground.as_slice());
            msg.scene_mask_tick = f.scene_mask_tick;
            msg.scene_labels_tick = f.scene_labels_tick;
            if f.scene_mask_tick || f.scene_labels_tick {
                msg.frame = img.clone();
            }
        }
    }
    msg
}

/// DetectionDepthArray for one synchronized set.
fn detection_message(state: &mut State, set: &[Input]) -> Option<DetectionDepthArray> {
    let (Some(Input::Det(det)), Some(Input::Depth(depth))) = (set.first(), set.get(1)) else {
        return None;
    };
    let mask = match set.get(2) {
        Some(Input::Mask(m)) => Some(Arc::clone(m)),
        _ => None,
    };
    let boxes: Vec<BoxPx> = det
        .detections
        .iter()
        .map(|d| BoxPx {
            cx: d.bbox.center.position.x,
            cy: d.bbox.center.position.y,
            w: d.bbox.size_x,
            h: d.bbox.size_y,
        })
        .collect();
    let depth_stamp = stamp_s(depth.header.stamp.sec, depth.header.stamp.nanosec);
    let mask_encoding = mask.as_ref().map(|m| m.encoding.to_string());
    let frame = with_parts(depth, |dp| match &mask {
        Some(m) => with_parts(m, |mp| state.det.frame(dp, depth_stamp, Some(mp), &boxes)),
        None => state.det.frame(dp, depth_stamp, None, &boxes),
    });
    let p = &state.det.params;
    let mut msg = DetectionDepthArray {
        header: std_msgs::msg::Header::from_rmw_message(depth.header.clone()),
        detections: (**det).clone(),
        depth_status: frame.depth_status as u8,
        mask_status: frame.mask_status as u8,
        error: frame.error.clone(),
        depth_width: frame.depth_width,
        depth_height: frame.depth_height,
        fx: frame.intrinsics[0],
        fy: frame.intrinsics[1],
        cx: frame.intrinsics[2],
        cy: frame.intrinsics[3],
        mask_width: frame.mask_width,
        mask_height: frame.mask_height,
        mask_encoding: mask_encoding.unwrap_or_default(),
        use_mask_depth: p.sample.use_mask_depth,
        center_fraction: p.sample.center_fraction,
        enable_scene_labels: p.sample.scene_enabled,
        scene_labels_max_age_s: p.scene_labels_max_age_s,
        scene_min_other_px: i32::try_from(p.sample.scene_min_other_px).unwrap_or(i32::MAX),
        sync_slop: p.sync_slop,
        sync_queue_size: i32::try_from(p.sync_queue_size).unwrap_or(i32::MAX),
        det2d_in: state.counts.det2d_in,
        depth_in: state.counts.depth_in,
        mask_in: state.counts.mask_in,
        synced: state.counts.synced,
        ..DetectionDepthArray::default()
    };
    if let Some(m) = &mask {
        msg.mask_stamp = builtin_interfaces::msg::Time {
            sec: m.header.stamp.sec,
            nanosec: m.header.stamp.nanosec,
        };
    }
    if frame.depth_status == DepthStatus::Ok {
        msg.depths = frame
            .depths
            .iter()
            .map(|d| DetectionDepth {
                source: d.source as u8,
                valid: d.depth.is_some(),
                depth_m: d.depth.unwrap_or(f32::NAN),
                box_px: d.box_px,
                mask_px: d.mask_px,
                other_px: d.other_px,
            })
            .collect();
    }
    Some(msg)
}

fn on_sync_input(state: &Arc<Mutex<State>>, out: &Outputs, input: usize, stamp: i64, item: Input) {
    let msg = {
        let mut s = lock(state);
        let Some(set) = s.sync.add(input, stamp, item) else {
            return;
        };
        s.counts.synced += 1;
        let m = detection_message(&mut s, &set);
        if m.is_some() {
            s.counts.det_out += 1;
        }
        m
    };
    if let Some(m) = msg {
        out.publish_det(m);
    }
}

fn run() -> Result<(), String> {
    let context = Context::default_from_env().map_err(|e| e.to_string())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(NODE_NAME).map_err(|e| e.to_string())?;
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
    let clock = node.get_clock();
    let clock_kind = match clock.clock_type() {
        ClockType::RosTime => "ROS time (/clock)",
        ClockType::SystemTime => "system time",
        ClockType::SteadyTime => "steady time",
    };

    let inputs = if params.det.sample.use_mask_depth { 3 } else { 2 };
    let queue_size = usize::try_from(params.det.sync_queue_size).unwrap_or(0);
    let state = Arc::new(Mutex::new(State {
        det: DetectionIngest::new(params.det.clone()),
        roi: RoiIngest::new(params.roi.clone()),
        sync: ApproxSync::new(inputs, queue_size, params.det.sync_slop),
        counts: Counters::default(),
    }));
    let out = Arc::new(Outputs {
        node: Arc::clone(&node),
        det_pub: if params.detection_depths_enabled {
            Some(
                node.create_publisher::<DetectionDepthArray>(
                    params.detection_depths_topic.as_str().keep_last(10).reliable(),
                )
                .map_err(err)?,
            )
        } else {
            None
        },
        roi_pub: if params.front_roi_enabled {
            Some(
                node.create_publisher::<RmwDepthRoi>(
                    params.front_roi_topic.as_str().keep_last(10).reliable(),
                )
                .map_err(err)?,
            )
        } else {
            None
        },
    });

    let mut subs: Vec<Box<dyn Any>> = Vec::new();

    // camera_info: detection_3d_node's create_subscription(..., 10).
    {
        let st = Arc::clone(&state);
        subs.push(Box::new(
            node.create_subscription::<CameraInfo, _>(
                params.depth_info_topic.as_str().keep_last(10).reliable(),
                move |msg: CameraInfo| lock(&st).det.camera_info(&msg.k),
            )
            .map_err(err)?,
        ));
    }
    // Scene labels: LATEST_FRAME_QOS, only with scene labels on.
    if params.det.sample.scene_enabled {
        let st = Arc::clone(&state);
        subs.push(Box::new(
            node.create_subscription::<RmwImage, _>(
                params
                    .scene_labels_topic
                    .as_str()
                    .qos(QoSProfile::sensor_data_default())
                    .keep_last(1),
                move |msg: RmwImage| {
                    let stamp = stamp_s(msg.header.stamp.sec, msg.header.stamp.nanosec);
                    with_parts(&msg, |p| lock(&st).det.scene_labels(stamp, p));
                },
            )
            .map_err(err)?,
        ));
    }
    // Detections: message_filters' default QoS (keep last 10, reliable).
    {
        let st = Arc::clone(&state);
        let o = Arc::clone(&out);
        let enabled = params.detection_depths_enabled;
        subs.push(Box::new(
            node.create_subscription::<Detection2DArray, _>(
                params.detections_topic.as_str().keep_last(10).reliable(),
                move |msg: Detection2DArray| {
                    lock(&st).counts.det2d_in += 1;
                    if enabled {
                        let stamp = stamp_ns(msg.header.stamp.sec, msg.header.stamp.nanosec);
                        on_sync_input(&st, &o, 0, stamp, Input::Det(Arc::new(msg)));
                    }
                },
            )
            .map_err(err)?,
        ));
    }
    // Masks (3-way sync only).
    if params.det.sample.use_mask_depth && params.detection_depths_enabled {
        let st = Arc::clone(&state);
        let o = Arc::clone(&out);
        subs.push(Box::new(
            node.create_subscription::<RmwImage, _>(
                params.masks_topic.as_str().keep_last(10).reliable(),
                move |msg: RmwImage| {
                    lock(&st).counts.mask_in += 1;
                    let stamp = stamp_ns(msg.header.stamp.sec, msg.header.stamp.nanosec);
                    on_sync_input(&st, &o, 2, stamp, Input::Mask(Arc::new(msg)));
                },
            )
            .map_err(err)?,
        ));
    }
    // Depth, last (as the Python nodes subscribe it after everything else).
    {
        let st = Arc::clone(&state);
        let o = Arc::clone(&out);
        let clk = clock.clone();
        let (roi_enabled, det_enabled) = (params.front_roi_enabled, params.detection_depths_enabled);
        subs.push(Box::new(
            node.create_subscription::<RmwImage, _>(
                params.depth_topic.as_str().keep_last(10).reliable(),
                move |msg: RmwImage| {
                    let img = Arc::new(msg);
                    if roi_enabled {
                        let now = now_s(&clk);
                        let roi = {
                            let mut s = lock(&st);
                            s.counts.depth_in += 1;
                            let m = roi_message(&mut s, &img, now);
                            s.counts.roi_out += 1;
                            m
                        };
                        if roi.status != 0 {
                            log_error!(
                                o.node.logger().throttle(Duration::from_secs(5)),
                                "depth frame not decoded: {}",
                                roi.error
                            );
                        }
                        o.publish_roi(&roi);
                    } else {
                        lock(&st).counts.depth_in += 1;
                    }
                    if det_enabled {
                        let stamp = stamp_ns(img.header.stamp.sec, img.header.stamp.nanosec);
                        on_sync_input(&st, &o, 1, stamp, Input::Depth(img));
                    }
                },
            )
            .map_err(err)?,
        ));
    }

    // A one-line summary every 10 s (steady clock).
    let summary_state = Arc::clone(&state);
    let summary_node = Arc::clone(&node);
    let _summary = node
        .create_timer_repeating(Duration::from_secs(10), move || {
            let s = lock(&summary_state);
            let c = &s.counts;
            log_info!(
                summary_node.logger(),
                "INGEST | depth_in={} det2d_in={} mask_in={} synced={} detection_depths={} front_roi={} scene_labels_buffered={}",
                c.depth_in,
                c.det2d_in,
                c.mask_in,
                c.synced,
                c.det_out,
                c.roi_out,
                s.det.scene_buffered()
            );
        })
        .map_err(err)?;

    log_info!(
        node.logger(),
        "{NODE_NAME} (rust) up: depth \"{}\", camera_info \"{}\", detections \"{}\"{} -> \
         detection_depths \"{}\" ({}), front_roi \"{}\" ({}); sync {}-way queue {} slop {} s; \
         ROI half {}x{} px shift {} px, ground removal {} stride {}; scene mask {} at {} Hz, \
         scene labels {} at {} Hz; clock {clock_kind}",
        params.depth_topic,
        params.depth_info_topic,
        params.detections_topic,
        if params.det.sample.use_mask_depth {
            format!(", masks \"{}\"", params.masks_topic)
        } else {
            String::new()
        },
        params.detection_depths_topic,
        if params.detection_depths_enabled { "on" } else { "off" },
        params.front_roi_topic,
        if params.front_roi_enabled { "on" } else { "off" },
        inputs,
        params.det.sync_queue_size,
        params.det.sync_slop,
        params.roi.roi_half_width_px,
        params.roi.roi_half_height_px,
        params.roi.roi_shift_up_px,
        params.roi.ground_removal_enabled,
        params.roi.ground_stride,
        params.roi.scene_mask_enabled,
        params.roi.scene_mask_rate_hz,
        params.roi.scene_labels_enabled,
        params.roi.scene_rate_hz,
    );
    if !params.detection_depths_enabled && !params.front_roi_enabled {
        log_warn!(node.logger(), "both outputs are off: this node only counts depth frames");
    }

    let spun = executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(err);
    drop(subs);
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
