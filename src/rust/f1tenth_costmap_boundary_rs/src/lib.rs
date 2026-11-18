//! costmap_boundary_node in Rust: the nearest-occupied-cell path of
//! f1tenth_costmap/costmap_boundary_node.py (campaign-final 0c1cd41).
//!
//! [`extraction`] is the ROS-free port of costmap_boundary.py (parity with
//! the Python), [`replay`] the parity harness's case format, [`config`] the
//! startup checks of the parameters, [`build_info`] the build the binaries
//! log.

pub mod build_info;
pub mod config;
pub mod extraction;
pub mod replay;
