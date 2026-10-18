//! Front-proximity guard: a Rust port of the front check of
//! f1tenth_behavior's `IsProximityTooClose`, run as its own node.
//!
//! [`decision`] is the ROS-free decision (parity with the Python),
//! [`static_tf`] the /tf_static lookup of the laser's mount,
//! [`replay`] the parity harness's case format.

pub mod decision;
pub mod replay;
pub mod static_tf;
