//! The last-resort LiDAR stop in Rust: a port of f1tenth_control's
//! hard_stop.py (decision) and hard_stop_node.py (node logic), run as
//! `hard_stop_node` with hard_stop_impl:=rust (ackermann_mux.launch.py).
//!
//! [`decision`] is hard_stop.py (pure functions and the latch),
//! [`core`] HardStopNode's callbacks without ROS,
//! [`config`] its parameters and startup refusals,
//! [`pyfmt`] the CPython float forms its JSON and logs go through,
//! [`replay`] the parity harness's protocol (tools/rust_parity/hard_stop),
//! [`build_info`] the build the binaries log.

pub mod build_info;
pub mod config;
pub mod core;
pub mod decision;
pub mod pyfmt;
pub mod replay;
