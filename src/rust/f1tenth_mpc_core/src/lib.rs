//! f1tenth_mpc_core: MPC_corr's corridor geometry in Rust, as a library.
//!
//! No ROS: `mpc_controller` (Python) loads the cdylib with ctypes
//! (`mpc_controller/mpc_native.py`) when its `corridor_impl` parameter is
//! `rust`; the default `py` never loads it.
//!
//! - [`corridor`]: port of `f1tenth_params/corridor_geometry.py`
//!   (`corridor_curves`, `corridor_curves_to_pose`, the Bezier walls, the
//!   ramp-span clamp, the width rate, `wrap_pi`);
//! - [`npcompat`]: numpy 1.26 / CPython semantics the port is built from
//!   (linspace, cumsum, gradient, interp, argmin, clip, maximum, Python's
//!   max/min, math.atan2), so the arrays are bit-identical on linus;
//! - [`query`]: the corridor queries of `mpc_solver.py`
//!   (`nearest_centerline_index`, `_nearest_corridor_frame`, the lateral
//!   coordinate and the heading);
//! - [`ffi`]: the C ABI, the only module with `unsafe`;
//! - [`build_info`]: the build the library reports (`f1tenth_mpc_core_build_info`).
//!
//! Parity: `tools/rust_parity/corridor/` -> `output/rust/parity_corridor.md`.

pub mod build_info;
pub mod corridor;
pub mod ffi;
pub mod npcompat;
pub mod query;
