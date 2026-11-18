//! f1tenth_wall_tracker_core: lidar_front_wall_node's multi-wall tracker and
//! its per-cycle wall fusion in Rust, as a library.
//!
//! No ROS: `f1tenth_perception` (Python) loads the cdylib with ctypes
//! (`f1tenth_perception/wall_tracker_native.py`) when its `wall_tracker_impl`
//! parameter is `rust`; the default `py` never loads it.
//!
//! - [`tracker`]: port of `multi_wall_tracker.MultiWallTracker` and its pure
//!   helpers (the state machine, association, extent, roles, FRONT);
//! - [`fusion`]: port of `wall_fusion`'s per-cycle functions (the column
//!   cross-check, observations from LiDAR lines, the stereo-only walls and
//!   their robust line fit, the rectangle verification);
//! - [`lsa`]: scipy's `linear_sum_assignment` (rectangular_lsap.cpp),
//!   tie-breaking included;
//! - [`lapack`]: numpy's `svd` of an M x 2 matrix (lapack_lite dgesdd path 3);
//! - [`linalg`]: numpy's 2x2 `solve` / `inv` (lapack_lite's dgesv) and the
//!   BLAS products;
//! - [`pyc`]: CPython 3.12 / numpy 1.26 scalar semantics (Python max/min,
//!   float `%`, `x ** 2`, `math.hypot`, `sum`, `np.arange`, searchsorted);
//! - [`geometry`]: corridor_perception's Pose2D / Line, scene_labels' WallLine;
//! - [`ffi`]: the C ABI, the only module with `unsafe`;
//! - [`build_info`]: the build the library reports.
//!
//! Parity: `tools/rust_parity/wall_tracker/` -> `output/rust/parity_wall_tracker.md`.

pub mod build_info;
pub mod ffi;
pub mod fusion;
pub mod geometry;
pub mod lapack;
pub mod linalg;
pub mod lsa;
pub mod pyc;
pub mod tracker;
