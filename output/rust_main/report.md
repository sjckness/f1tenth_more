# main-jazzy-rust: first Rust ports (run of 2026-10-23/24)

Branch `main-jazzy-rust` (worktree `~/dev_ws/f1tenth_more_rust`), from main-jazzy. Not pushed.
Plan: `~/.claude/plans/fuzzy-enchanting-teapot.md`. For each phase an implementer agent wrote
the code, then a manager agent reviewed it against the plan and re-ran every gate itself. This
session made the commits, after each gate passed.
**Every switch defaults to py/off. With no switch set, the stack behaves exactly like main-jazzy.**

## Result per phase
| phase | what | status | key numbers (linus, x86) | commit |
|---|---|---|---|---|
| R0 | foundation: jazzy-rust's proximity guard and costmap_boundary nodes cherry-picked; the costmap node refuses to start with parameters it does not implement; opt-level 3; Docker rust stage prepared | done | costmap tick p50 4.95 ms py -> **92 us** rust; guard 0.61 -> 0.25 ms CPU per scan; parity bit-exact | e019d08a |
| R3 | cheap Python fixes: /tf_static-only TF in detection_3d / obstacle_projector, polled map->odom in MPC_corr, swept_clearance started only on demand, thread limits | done (also on main-jazzy) | MPC_corr TF lookups identical on 2531 car ticks | ceb7c884 (merge 34288544) |
| R1 | Rust corridor library `f1tenth_mpc_core`, `corridor_impl:=py\|rust` | done | bit-identical for every finite input; every test gives the same result with py and rust | e59fc52a |
| R2.0 | MPC solve profile + 723-tick fixture | done | Python tick ~8 ms, the OSQP C solve ~1 ms of it | b6909dc1 |
| R2 | Rust MPC solver core (OSQP C 1.0, as in Python), `mpc_solver_impl:=py\|rust` | done | bit-identical on every layer, closed loop 32/32; **p95 10.4 -> 2.0 ms** (live config 11.1 -> 2.2 ms) | a751e630 |
| R4 | hard_stop_node in Rust (rclrs), `hard_stop_impl:=py\|rust` | done | 1.13 M replay ops, 0 differences; live py vs rust: same trigger scan, same release | 27f263dc (merge c7a5571a) |
| R6 | wall tracker core in Rust (multi_wall_tracker + wall_fusion), `wall_tracker_impl:=py\|rust` | done | every bag, cycle and field bit-identical; **tracker_ms p95 12.8 -> 4.5 ms** (worst bag), 2.2-4.3x | 40e8282a (merge d2fca783) |
| R5 | depth ingest node `f1tenth_depth_ingest_rs` | **partial, not committed** | see below | — |

Merged tree (d58c0f8e): cargo fmt/clippy/test OK, colcon build of 38 packages, colcon test of
the 14 touched packages: 4247 tests, only the failures already in the baseline
(`output/jazzy_main/test_failing_ids.txt`) after d58c0f8e removed a duplicate assertion that
the merges had left behind.

## R5: stopped by the session limit
The workflow's implementer and manager both died on the account's session limit
(2026-10-24 ~01:07). What exists, **uncommitted in the worktree**:
- `output/rust/depth_ingest_design.md` (the inventory, complete): front_clearance's statistics
  depend on its own object-confidence EMA, label grid and floor RANSAC, so the ingest node can
  only hand it **pixels** (ROI, stride-4 floor sample, whole frame on the 5 Hz ticks), not
  statistics. The clear win is detection_3d: per-detection medians computed next to the depth,
  with the same message_filters sync policy.
- messages `DepthRoi`, `DetectionDepth`, `DetectionDepthArray` (additive) in f1tenth_messages;
- `src/rust/f1tenth_depth_ingest_rs` (~3000 lines: sync, median, roi, detection, ffi, node);
- `f1tenth_perception/depth_ingest_{backend,native}.py`.
Not done: the consumer switches in front_clearance_node / detection_3d_node, the parity
harness, the gates. It has not been built or tested. The next run resumes from these files.

## How to try a switch (linus, Isaac or bag replay)
`ros2 launch f1tenth_bringup supervisor_bringup.launch.py corridor_impl:=rust mpc_solver_impl:=rust`
(also `hard_stop_impl:=rust`, `wall_tracker_impl:=rust`, `costmap_boundary_impl:=rust`,
`proximity_guard:=shadow|active`). A Rust switch whose library or binary is missing refuses to start.

## Pending (needs a machine, disk or you)
- R5: finish (consumer switches, parity, gates).
- Isaac e2e per switch (`tools/e2e_test.sh`): `sim_isaac.sh` needs >= 25 GB free on `/`; linus
  has 16 GB.
- Thor: rustup + colcon cargo plugins, rclrs 0.8 (the Thor's generator is 0.5.0), aarch64 parity
  (bit identity not promised there, tolerance gates apply), benches.
- Orin image: the `docker/orin` RUST=build stage is prepared, not built; it needs rclrs 0.8 and
  vendoring of osqp-sys's QDLDL (fetched from GitHub at build time).
- R0b: apt upgrade of linus to the Thor snapshot (needs your sudo), so all machines share one
  generator / rclrs.
- Known small item: the rclrs nodes end on SIGINT by the signal (exit -2, launch logs "process
  has died"); harmless, but it reads like a crash in the logs (`r4.md`).
- Default flips to rust: separate commits, after evidence from the car.
