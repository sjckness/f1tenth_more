# main-jazzy-rust: design notes for the implementers (from the planning session, 2026-10-23)

Plan (approved): `~/.claude/plans/fuzzy-enchanting-teapot.md`. Worktree `~/dev_ws/f1tenth_more_rust`,
branch `main-jazzy-rust` = main-jazzy cb5e3847 + the jazzy-rust commits cherry-picked (00d5869a..96f67bd4).
Environment: `source output/rust_main/env.sh` (bash). Decisions: `output/rust_main/decisions.md`.

## Ground rules (every phase)
- Behaviour-identical ports (parity); a deliberate fail-safe difference is documented in the parity
  report. Every switch defaults to py/off. No default flips.
- No git state changes (no add/commit/stash/checkout/reset/branch/merge/cherry-pick). The coordinator commits.
- No sudo, no apt, no ssh to the Thor/Orin, no docker builds, no deletion of user data. Never `rm -rf`
  a path built from a variable (CLAUDE.md). Never launch ROS against a live graph: isolated domain,
  localhost (env.sh). Never two colcon builds at once in the same workspace.
- Python stays 3.12-compatible (Jazzy); Rust: `cargo fmt --check`, `cargo clippy -D warnings`, `cargo test`
  in `src/rust`; `unsafe_code` forbidden except in an `ffi.rs`.
- Known baseline: `output/jazzy_main/test_failing_ids.txt` (45 failures + 1 error, all pre-existing).
  13 mpc_controller tests fail because they pin old defaults: record, do not change them.
- When something is unclear: write the question and your chosen answer in decisions.md, take the most
  reasonable choice, go on. Never wait.

## Key facts
- Every installed Rust binary was a debug build (colcon-ros-cargo calls cargo ament-build without a
  profile). Fix: `[profile.dev] opt-level = 3` in `src/rust/Cargo.toml`; log the profile at startup.
- linus: apt `ros-jazzy-rosidl-generator-rs 0.4.11` -> rclrs `=0.7.0`. Thor 0.5.0 -> rclrs 0.8 later
  (needs sudo on linus to match; NOT in this run). The Orin image has no Rust yet.
- `cargo-ament-build` 0.1.11 installs a cdylib `lib<pkg>.so` into `install/<pkg>/lib/<pkg>/`.
- Python osqp 1.1.3 builds OSQP C v1.0.0; Rust crate `osqp = "=1.0.1"` wraps the same C v1.0.0.
  numpy on linus uses reference BLAS; Rust can reproduce its summation order (bit parity possible on
  x86, not promised on aarch64).
- Live MPC: `speed_command_mode` default `offset_free` -> 5 states, 145 QP variables with 4 wall slacks
  and the speed-cap slack. `stage_spec` paths (reverse candidates, `reverse_manoeuvre.py`) stay Python.
- MPC_corr needs map->odom (dynamic): not /tf_static. `f1tenth_comms.tf_poll.PolledTf` exists.
- obstacle_clearance feeds the test-campaign logger's `contact` events: stays always-on. swept_clearance
  has no consumer; `swept_clearance_body_half_width_m` is read by 6 modules and must stay.
- front_clearance reads the whole depth frame (scene mask), not only an ROI.

## R0 (foundation) remaining items
1. Rust costmap node (`src/rust/f1tenth_costmap_boundary_rs/src/main.rs`): declare the 5 new params with
   the Python defaults (component_filter false, min_component_length_m 0.5, min_component_scans 2,
   exclude_tracked_object_cells false, object_footprint_margin_m 0.12); refuse to start (Err, same
   message pattern as use_convex_polytope) when component_filter or exclude_tracked_object_cells is
   true; pure `validate()` + unit test; extend `test_costmap_boundary_impl_launch.py` so both nodes get
   the same parameter keys.
2. `[profile.dev] opt-level = 3`; each binary logs `cfg!(debug_assertions)`; re-measure guard and costmap
   CPU with the existing live tests (`tools/rust_parity/*/live_*_test.py`) if they run offline.
3. Re-run `tools/rust_parity/{proximity,costmap_boundary}` against main-jazzy's Python: still bit-exact.
4. Prepare (do not build) a `rust` stage in `docker/orin/Dockerfile`: pinned rustup-init + toolchain,
   cargo-ament-build 0.1.11 --locked, colcon-cargo/colcon-ros-cargo 0.2.0, `ros-jazzy-rosidl-generator-rs`
   in `ros_packages.txt` (Thor snapshot 0.5.0 -> needs rclrs 0.8: note it), with a fallback that
   ignores the ament_cargo packages. Comment clearly "not built yet".
Gate: cargo fmt/clippy/test; colcon build (skip list) OK; colcon test no new failing IDs; parity bit-exact.

## R1 corridor library `src/rust/f1tenth_mpc_core`
- ament_cargo, `crate-type = ["cdylib","rlib"]`, no ROS deps, `#[allow(unsafe_code)]` only in ffi.rs.
- corridor.rs: port `f1tenth_params/f1tenth_params/corridor_geometry.py` (`corridor_curves`,
  `corridor_curves_to_pose`, bezier, ramp span, wrap_pi, r_min clamp, width rate). npcompat.rs: exact
  numpy linspace/cumsum/gradient(edge_order 1)/interp/first-index argmin. query.rs:
  nearest_centerline_index, corridor_lateral_coordinates, corridor_heading_at (`_nearest_corridor_frame`
  in mpc_solver.py:748-760).
- wall_turn.py / object_approach.py stay Python.
- Python: `mpc_controller/mpc_native.py` (ctypes load from
  `get_package_prefix('f1tenth_mpc_core')/lib/f1tenth_mpc_core/libf1tenth_mpc_core.so`, ABI version and
  struct-size check, `available()`/`load_error()`), `mpc_controller/corridor_backend.py` (dispatcher,
  same dict as today); MPC_corr's geometry call sites (~6687, ~6753) via
  `getattr(self, 'corridor_impl', corridor_backend.DEFAULT_IMPL)` so the 21 fake-object test files
  keep working; rust requested but not loadable -> refuse to start. package.xml exec/test_depend.
- Switch `corridor_impl:=py|rust` (stack_params, mpc_corr.launch.py, navigation.launch.py forward list,
  `_RUST_SWITCHES` generalized to `{arg: (launch_file, choices)}` in component_supervisor_node.py,
  supervisor/stack bringup args, test_rust_switches.py).
- Parity `tools/rust_parity/corridor/` -> `output/rust/parity_corridor.md`: the 24 rebuilds in
  `f1tenth_params/test/data/object_rebuilds.json` x shapes/walls/clamps; in-process capture of every
  call from the tests and closed-loop rigs (conftest recorder) replayed through Rust; 10k edge-case sweep
  (n=2, L~0, dpsi +-pi/+-2pi, tiny chords, u_end=u_start); the corridor tests with DEFAULT_IMPL='rust'.
  Gate: arrays within 1e-12, flags/argmin identical (target bit-identical on linus).

## R2 solver core (same crate)
0. Measure first: `tools/rust_parity/mpc/profile_rti.py` -> `output/rust/mpc/rti_profile.md` (row
   assembly, csc_matrix, OSQP.setup, solve+polish, `_linearize_obstacle_cost_rti`, model+Jacobians,
   `_rollout_true_dynamics`, `planner_cost_corridor`; legacy and offset_free; 0/1/8 obstacles; 0/4 rows;
   BLAS, OSQP version, CPU).
1. model.rs (f1tenth_state_fcn_dt_beta + lag variant, Jacobians), qp.rs (same rows/order/additions,
   0.5(P+P^T), scipy-identical CSC: zeros dropped, sorted indices, upper triangle as osqp-python takes),
   solve.rs (`osqp =1.0.1`, fresh Problem every tick, warm start x only, bounds clamped to +-OSQP_INFTY,
   every setting set explicitly from a dump of Python's effective values: max_iter 200, polish on,
   check_termination 25, adaptive_rho_interval 50, ...; same infeasible-status set), cost.rs.
2. Interface per tick in: x0 (4/5), last_u, pref_nom, N, ts, control_period, L, lr, the 10 limits,
   weights already scaled by scale_stage_weights, obstacles_world (Mx3), car_radius, avoidance_margin,
   SOLVER_FRONT_POINT_OFFSETS_M, padded boundary rows + hard/slack settings, warm start (2N or null),
   corridor xc yc nx ny halfWidth psiRef psiRefTurn(NaN=None) psiStart targetLine, speed model
   (alpha, d, delay, c_lo/c_hi/v_hi over N, inf allowed, slack_weight). Out: u0, zopt (2N),
   x_pred ((N+1)x4), v_cmd_pred, success, status code+string, iterations, cost, boundary_slack,
   speed_slack. Stateless. Debug exports `mpc_build_qp`, `mpc_osqp_solve_csc`.
3. Dispatch in `solve_mpc_step(..., impl='py')`: rust only for solver rti and stage_spec None. Switch
   `mpc_solver_impl:=py|rust`, plumbed like R1.
4. Parity layers: L1 QP assembly vs Python arrays captured by patching osqp.OSQP.setup (sparsity
   identical, values 1e-14); L2 Rust OSQP on Python's QP (bit-identical x, iterations, status); L3 per
   tick on ~800 captured ticks + live captures via env `F1TENTH_MPC_CAPTURE` (>=99 % within 1e-9 on u,
   zopt, x_pred, identical status; outliers explained, <=1e-3); L4 closed loop
   `test/closed_loop_parity.py` (same outcome and end tick, <=1 cm, <=0.01 rad; the SHA golden stays the
   py reference). Timing `tools/rust_parity/mpc/bench_solve.py --impl py|rust` (gate p95 <= Python).
   Persistent OSQP workspace only as extra value `rust_persistent` if timing calls for it.

## R3 cheap Python fixes (separate worktree `~/dev_ws/f1tenth_more_pyfix`, branch `jazzy-pyfixes`)
1. detection_3d_node (TransformListener ~:282) and obstacle_projector_node (~:197): /tf_static-only
   buffer (pattern `f1tenth_behavior/behaviours/static_frames.py`); refuse a dynamic output frame.
2. MPC_corr (~:1812): `f1tenth_comms.tf_poll.PolledTf` for map->odom (`map_odom_max_age_sec`) + a
   static buffer for base_link<-laser; exec_depend f1tenth_comms; verify identical lookups.
3. swept_clearance out of `_ALWAYS_AUTO_START` (component_supervisor_node.py ~:326), component kept.
   obstacle_clearance unchanged.
4. Thread limits for semantic_layer (costmap.launch.py ~:157) and front_clearance (detection.launch.py
   ~:548): `*_cpu_threads` param + OMP/MKL/OPENBLAS/OPENCV env via launch, the yolo pattern
   (detection.launch.py OpaqueFunction). 0 = unset.
Gate: unit tests; replays identical where applicable.

## R4 hard_stop in Rust `src/rust/f1tenth_hard_stop_rs`
Port `src/f1tenth_control/f1tenth_control/hard_stop.py` (footprint, self-hit, scan points, debounce,
clear, direction from /teleop, /calibration_drive, /drive) into decision.rs; main.rs: /scan sensor QoS,
AckermannDriveStamped lanes, /safety_stop, /safety/hard_stop JSON, 20 Hz republish, lane_timeout_s,
same params (front_x/half_width from launch like the costmap node), sim-time clock as in 96f67bd4.
Switch `hard_stop_impl:=py|rust` in `f1tenth_control/launch/ackermann_mux.launch.py`. Parity: the 42
tests as JSON cases for a Rust replay binary; recorded scans incl. self-hits; timing like
`tools/rust_parity/proximity/timeout_parity.py`.

## R5 depth ingest (after R3; rclrs 0.7 on linus)
Inventory first -> `output/rust/depth_ingest_design.md`; then an rclrs node reading depth, camera_info,
masks, detections once, publishing ROI stats + new `f1tenth_messages/DetectionDepthArray`
(per-detection medians, synchronized like detection_3d); consumers get `depth_source:=raw|ingest`
(default raw). Parity: bit-identical medians (numpy even-count/NaN), identical consumer outputs; CPU with
`tools/rust_parity/heavy`.

## R6 wall tracker
Stage timing with `tools/corridor_wall_replay.py` on car bags; `f1tenth_wall_tracker_core` (stateful
handle, same FFI pattern) for multi_wall_tracker + wall_fusion, exact linear_sum_assignment tie-breaking,
reuse C extract_lines. Switch `wall_tracker_impl:=py|rust`. Parity: 2292-scan replay + car bags (ids,
roles, states identical; floats 1e-9); the ~300 tests against both backends.
