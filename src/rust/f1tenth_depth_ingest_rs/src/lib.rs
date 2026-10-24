//! The depth ingest (R5): one reader of the ZED depth frame for the two
//! Python consumers that used to take it whole (output/rust/depth_ingest_design.md).
//!
//! [`ingest`] is what the node does with each message, without ROS;
//! [`detection`] detection_3d_node's per-detection sampling,
//! [`median`] np.median of float32, bit for bit,
//! [`sync`] the Python message_filters ApproximateTimeSynchronizer,
//! [`scene`] detection_3d_node's scene-label buffer,
//! [`roi`] front_clearance_node's ROI, floor sample and tick gates,
//! [`image`] cv_bridge's decode of the two encodings used,
//! [`ffi`] the C ABI f1tenth_perception loads (ctypes),
//! [`build_info`] the build the node logs.
//!
//! None of it uses rclrs or a message crate: the node binary (src/main.rs)
//! is the only ROS code, so libf1tenth_depth_ingest_rs.so links no ROS library.

pub mod build_info;
pub mod detection;
pub mod ffi;
pub mod image;
pub mod ingest;
pub mod median;
pub mod roi;
pub mod scene;
pub mod sync;
