//! f1tenth_mpc_core: MPC_corr's corridor geometry (R1) and RTI solver core
//! (R2) in Rust, as a library.
//!
//! No ROS: `mpc_controller` (Python) loads the cdylib with ctypes
//! (`mpc_controller/mpc_native.py`) when its `corridor_impl` or
//! `mpc_solver_impl` parameter is `rust`; the default `py` never loads it.
//!
//! Corridor (R1, `output/rust/parity_corridor.md`):
//! - [`corridor`]: port of `f1tenth_params/corridor_geometry.py`
//!   (`corridor_curves`, `corridor_curves_to_pose`, the Bezier walls, the
//!   ramp-span clamp, the width rate, `wrap_pi`);
//! - [`npcompat`]: numpy 1.26 / CPython semantics the ports are built from
//!   (linspace, cumsum, gradient, interp, argmin, clip, maximum/minimum,
//!   Python's max/min, math.atan2, reference-BLAS dot and gemv, numpy's
//!   scalar power), so the arrays are bit-identical on linus;
//! - [`query`]: the corridor queries of `mpc_solver.py`
//!   (`nearest_centerline_index`, `_nearest_corridor_frame`, the lateral
//!   coordinate and the heading).
//!
//! Solver core (R2, `output/rust/parity_mpc_solver.md`): `_solve_rti` for
//! `stage_spec None`, both speed models.
//! - [`pyfloat`]: CPython float semantics (math domain errors, `x ** 2`,
//!   division by zero);
//! - [`model`]: `vehicle_model.py` (the bicycle model, its lag variant and
//!   the analytic Jacobians);
//! - [`problem`]: one tick's inputs and failures;
//! - [`cost`]: the obstacle linearisation and `planner_cost_corridor`;
//! - [`qp`]: the QP, bit for bit as osqp-python hands it to OSQP C;
//! - [`solve`]: OSQP (fresh workspace, the Python's settings), rollout, the
//!   result.
//!
//! - [`ffi`]: the C ABIs (to Python, and to OSQP C), the only module with
//!   `unsafe`;
//! - [`build_info`]: the build the library reports (`f1tenth_mpc_core_build_info`).

pub mod build_info;
pub mod corridor;
pub mod cost;
pub mod ffi;
pub mod model;
pub mod npcompat;
pub mod problem;
pub mod pyfloat;
pub mod qp;
pub mod query;
pub mod solve;
