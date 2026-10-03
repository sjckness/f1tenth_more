# Backlog: post-migration items

This is the single list of work the Jazzy migration reports deferred. Each
item comes from a report. Items those reports already closed are not listed;
they are under "Done" at the bottom.

## Priorities

- **HIGH:** must be settled before any test drive on Jazzy.
- **MEDIUM:** settle before the stack is relied on (field runs, data
  collection).
- **LOW:** debt, cleanup, nice to have.

Sources: P0 = `thor_phase0_report.md`, P1–P5 = `phase<N>_*_report.md`,
FB1/FB2 = `fix_batch_<N>_report.md`, Plan = `jazzy_migration_plan.md`.

---

## HIGH: before any test drive

| # | Item | Source |
|---|---|---|
| H1 | **Supervisor topic-liveness check.** A node can be alive but silently isolated from discovery: in cycle 5, `swept_clearance_node` ran but never saw `/scan` or `/tf_static` (1 in ~320 node starts, no error logged). The supervisor only checks process exit. Needs a per-component "expected output topic is publishing" check that restarts or flags the component. Also run the same 10 cycles on the Orin (Humble) to learn whether the isolation is pre-existing. | P5 Finding 3, Decision 3 |
| H2 | **ABI drift on the car's Jetson / in the Docker image.** Phase 1 found `robot_localization` crashing on a `diagnostic_updater` ABI mismatch between apt packages. Phase 5 found the workspace's own `ackermann_mux` and ZED binaries broken by the same upgrade. Check the target before the first live run. Build the image against the apt snapshot it runs, then run the `ldd -r` scan (CLAUDE.md, "Rebuild policy"). | P1 Decision 1, P5 |
| H3 | **MPC deadline overrun on the Orin.** The recorded Humble run had a mean period of 112 ms against a 100 ms deadline: 53 periods over 150 ms, worst 309 ms, all cores 78–100%. This is pre-existing, and the main timing risk for Jazzy-in-Docker on the Orin. Re-measure there. | P3 Decision 3 |
| H4 | **Local EKF + IMU parity, live.** No archived bag has `/sensors/imu/raw`, so the local EKF's IMU fusion has never been compared. Phase 6 on the stand. | P1 Decision 3 |
| H5 | **Review the `slam.launch.py` lifecycle fix (`4edcbbf`) before the first live SLAM test.** Before it, slam_toolbox never left `unconfigured`, silently. Also confirm `transform_publish_period: 0` (no `map→odom`) under real scan-matching load. | P0 Decision 2 |
| H6 | **Graph introspection now that stack nodes are plain Discovery Server clients.** On the Orin, `~/.bashrc` sets `ROS_SUPER_CLIENT` (per `foxglove_bridge.launch.py`'s comment), so every Humble node was presumably a super client; on Jazzy only foxglove and the mission logger are (FB2). A plain client sees node names but not other participants' topics, so code that reads topics from the graph may silently get nothing. Concrete call sites to check live: `ekf_cost_observer_node._subscribe()` resolves its input/output topic types with `get_topic_names_and_types()` and subscribes only if found (it may measure nothing); `steering_offset_calibration_node` refuses to run unless `count_publishers(/safety_stop)` ≥ 1 (a plain client that does not itself use `/safety_stop` may count 0). Also `scripts/stackctl.py`. `get_node_names()` users (BT preflight, `diagnostics_server`) are fine: node names reach plain clients. Fix per node with the same launch-scoped `ROS_SUPER_CLIENT` as FB2 where needed. | FB2 |

## MEDIUM

| # | Item | Source |
|---|---|---|
| M1 | **Driver QoS on the car** for the newly recorded `/imu`, `/joint_states`, `/sensors/imu/raw`. Any best-effort publisher must also go into `_DEFAULT_BEST_EFFORT_TOPICS`. | FB1 |
| M2 | **`/imu` has no publisher in the workspace.** MPC_corr subscribes to it; `ax_imu` only feeds the control-log CSV's `a_imu` column (NaN without it). Check on the car whether anything publishes it. | FB1 Decision B |
| M3 | **`system_observer` in the Orin container.** It needs jtop and `/run/jtop.sock`, or it uses the sysfs fallback (now crash-safe, FB1). Without jtop, the GPU temperature is 0.0, so `IsSystemOverheated` cannot see GPU heat. Decide whether the container gets jtop. | P4 Decision 3, FB1 |
| M4 | **CPU pinning for the target machine.** The Orin defaults ran unchanged on Thor. Cores 12–13 are unused, and four nodes share core 5. Use the P3 Step 6 proposal as input to a measurement on the target. Not before the target is decided. | P0 Decision 3, P3 Step 6, P5 |
| M5 | **Profile MPC_corr on the Orin** (`py-spy top`). Only about 7% of its CPU is the solve; find the rest before tuning pinning. | P3 Decision 4 |
| M6 | **Pin osqp** in the Jazzy image to the Orin's version once known. 1.0.5, 1.1.0 and 1.1.3 are bit-identical on our data; a future 2.x may change defaults. | P3 Decision 5 |
| M7 | **pose0 rejection rate unresolved.** The 3.1% tuning baseline could not be reproduced from one bag. It needs the original 37-run archive, or a live capture with `/diagnostics` recorded. | P1 Decision 2, follow-up |
| M8 | **`costmap_boundary_node` outlier rate.** About 2–5% of ticks are off by more than 10 cm / 5°. Pre-existing. Filter before treating `/costmap/boundaries` or `/costmap/front_clearance` as continuous signals (e.g. in the MPC). | P2 Decision 2 |
| M9 | **torch / torchvision / ultralytics for JetPack 7.** Needed for YOLO on Thor or the sim host. Match the CUDA minor version. Never pip `nvidia-*` / PyPI torch (CUDA safety rule). | P0 Decision 4, P5 |
| M10 | **Two-machine sim checklist (TPad → Thor):**<br>- Discovery Server on Thor's LAN IP; same `ROS_DOMAIN_ID`; firewall open for UDP 11811 + RTPS ports.<br>- The TPad must not publish robot_state_publisher, EKF, `/joint_states`, `base_link→laser/imu`.<br>- It must publish `/camera/image_raw`.<br>- Push the TPad sim-port report so its network section can be cross-checked. | P5 Step 6, Decision 6 |
| M11 | **`humble-final` tag on the Jetson** before any Jazzy commit reaches it (clean diff and cherry-pick base). Not confirmed done. | Plan Decision 4 |
| M12 | **`mpc_controller`: 10 tests encode stale config defaults.** Pre-existing ("default moved, test not updated", see CLAUDE.md). Update the tests or the defaults deliberately. | P0 A2 |

## LOW

| # | Item | Source |
|---|---|---|
| L1 | **Shutdown force-kill path.** Seen in 2 of 15 Phase 5 runs: launch's SIGTERM arrived before `_stop_all_components` returned, although every component had exited within 0.7 s. Unexplained; one instrumented run on the Orin. | P5 Decision 5 |
| L2 | **Double-SIGINT shutdown.** The supervisor's `killpg` plus launch's forwarded SIGINT interrupts Python nodes' `finally:` cleanup: tracebacks, and a stale `/tmp/mission_logger.lock` (reclaimed at the next start). Distro-independent (probe). | P5 Finding 4 |
| L3 | **Record `/test_campaign/logger_status`?** Held back because `8e0d48c` pins that the mission logger records nothing the test-campaign side publishes. Needs a decision on that invariant. | FB1 Decision A |
| L4 | **Record the mission-start window.** The Phase 3 bag began about 13.5 s into the move, so the reference heading was unrecoverable. Start recording before the move starts. | P3 Decision 2, P4 |
| L5 | **Live llama-server check on Thor:**<br>- Build llama.cpp (CUDA, sm_110) and copy the Orin's .gguf.<br>- Judge by mission equality after translation.<br>- Use the same llama.cpp commit on the Orin. | P4 Decision 1 |
| L6 | **`raw_odom_map_tf_node` has no parity coverage.** Needs a bag recorded in `raw_odom` mode plus one case in `replay_localization.sh`. | P1 Decision 4 |
| L7 | **`inflate_polytope` / `use_convex_polytope` is off and never parity-tested.** Re-verify that path (it relies on P0's numpy fix `fd83d6e`) before enabling it. | P2 Decision 3 |
| L8 | **rosbag2 `Recorder()` and two-argument `record()` are deprecated in Jazzy.** They work today; migrate before they are removed. | P4 Step 2 |
| L9 | **Style/lint debt.** flake8, pep257 and copyright fail in several packages, e.g. 3,517 flake8 findings in `f1tenth_logger`. Untouched under the no-style-fixes rule. | P0, P4 Decision 6 |
| L10 | **`llm`: 12 `_note_llm_sent` test-stub failures** (stubs never got the method added in `dc158d2`). Pre-existing. | P0 A4, P4 |
| L11 | **`f1tenth_behavior` flaky test.** One failure under a whole-workspace parallel `colcon test`; not reproducible in 20+ runs, even under 2× CPU oversubscription. | P0 A3, P4 |
| L12 | **`stack_bringup.launch.py`** (the single-process fallback) has no sim mode and was not exercised in Phase 5. Keep it working, or retire it. | P5 |
| L13 | **QoS on bag rewriting.** Any future tool that filters or rewrites bags must carry `offered_qos_profiles` through, or `/tf_static` breaks silently. A practice note, not a code item. | P2 Decision 1 |

---

## Migration runs still pending on the Orin (tracked in the phase reports)

These are not post-migration work, but they block closing their phases.

| Item | Source |
|---|---|
| Phase 2 `costmap_boundary` frozen-input run (`output/phase2/ORIN_BOUNDARY_INSTRUCTIONS.md`) | P2 Addendum |
| Phase 3 MPC frozen/replay run and the Orin's osqp version (`output/phase3/ORIN_MPC_INSTRUCTIONS.md`) | P3 Decision 1 |
| Phase 4 BT replays, extract and py_trees versions (`output/phase4/ORIN_BT_INSTRUCTIONS.md`) | P4 Decision 2 |
| Phase 5 10-cycle comparison and the Orin's stack environment | P5 Decisions 2–3 |

---

## Done (closed by a fix batch)

| Item | Source | Closed by |
|---|---|---|
| `system_observer_node` crashes on an EAGAIN sysfs read | P4 Decision 3 | FB1 `931a0eb` |
| Preflight required `costmap_boundary_node` for `front_clearance` | P4 Decision 4 | FB1 `0397d6b` |
| Mission logger default topic list (7 of 8 topics) | P3 Decision 2, P4 | FB1 `7155f79` |
| Environment script / `~/.bashrc` | P5 Decision 1 | FB2 `d9a7f00` |
| Mission logger empty under the Discovery Server | P5 Decision 2 | FB2 `84a12f1` |
| Rebuild policy | P5 Decision 4 | FB2 `b0740ea` |
