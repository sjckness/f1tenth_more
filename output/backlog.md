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
FB1–FB5 = `fix_batch_<N>_report.md`, Plan = `jazzy_migration_plan.md`,
SP = `sim_port_report.md`.

---

## HIGH: before any test drive

| # | Item | Source |
|---|---|---|
| H2 | **ABI drift on the car's Jetson / in the Docker image.** Phase 1 found `robot_localization` crashing on a `diagnostic_updater` ABI mismatch between apt packages. Phase 5 found the workspace's own `ackermann_mux` and ZED binaries broken by the same upgrade. Check the target before the first live run. Build the image against the apt snapshot it runs, then run the `ldd -r` scan (CLAUDE.md, "Rebuild policy"). | P1 Decision 1, P5 |
| H3 | **MPC deadline overrun on the Orin.** The recorded Humble run had a mean period of 112 ms against a 100 ms deadline: 53 periods over 150 ms, worst 309 ms, all cores 78–100%. This is pre-existing, and the main timing risk for Jazzy-in-Docker on the Orin. Re-measure there. | P3 Decision 3 |
| H4 | **Local EKF + IMU parity, live.** No archived bag has `/sensors/imu/raw`, so the local EKF's IMU fusion has never been compared. Phase 6 on the stand. | P1 Decision 3 |
| H5 | **Review the `slam.launch.py` lifecycle fix (`4edcbbf`) before the first live SLAM test.** Before it, slam_toolbox never left `unconfigured`, silently. Also confirm `transform_publish_period: 0` (no `map→odom`) under real scan-matching load. | P0 Decision 2 |
| H7 | **The BT must stop the car on `/supervisor/health` ERROR for a safety-relevant component** (localization, slam, control). Decided 2026-10-05 (FB4 decision 2c, FB5 open decision 1): required before Phase 8 (ground driving). The status exists since FB5; nothing consumes it yet. Was M18. | FB4 B7, FB5 |

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
| M10 | **Two-machine sim checklist (TPad → Thor):**<br>- Discovery Server on Thor's LAN IP; same `ROS_DOMAIN_ID`; firewall open for UDP 11811 + RTPS ports.<br>- Sim host (linus / TPad): `export F1TENTH_DISCOVERY_SERVER=<Thor LAN IP>:11811`, then `source <repo>/scripts/env/jazzy.sh`, then start the simulator and bridge from that shell: every sim-side participant must carry the 5 s lease profile (FB5, "Phase S").<br>- The TPad must not publish robot_state_publisher, EKF, `/joint_states`, `base_link→laser/imu`.<br>- It must publish `/camera/image_raw`.<br>- Push the TPad sim-port report so its network section can be cross-checked.<br>**Status 2026-10-04 (SP §5):** TPad side done: no `/tf`/`/tf_static`, no `/robot_description`, no `/joint_states` from the sim (its own on `/sim/*`), no EKF; report pushed and merged. `scripts/env/jazzy.sh` now takes `F1TENTH_DISCOVERY_SERVER` and works from zsh. Still open: the DS/firewall/LAN run itself (Phase S, sim host linus), and **`/camera/image_raw` is not published**: the sim's ZED mock is off (SP §2.4), so detection has no images in sim mode. | P5 Step 6, Decision 6, SP, FB5 |
| M11 | **`humble-final` tag on the Jetson** before any Jazzy commit reaches it (clean diff and cherry-pick base). Not confirmed done. | Plan Decision 4 |
| M12 | **`mpc_controller`: 10 tests encode stale config defaults.** Pre-existing ("default moved, test not updated", see CLAUDE.md). Update the tests or the defaults deliberately. | P0 A2 |
| M13 | **Double-SIGINT shutdown.** The supervisor's `killpg` plus launch's forwarded SIGINT interrupts Python nodes' `finally:` cleanup: tracebacks, and a stale `/tmp/mission_logger.lock` (reclaimed at the next start). Distro-independent (probe). Raised from LOW by fix batch 4: participants destroyed this way never dispose and stay in the Discovery Server until their lease runs out, which fed H1 (closed by FB5: the window is now 5 s instead of 20 s, and possibly the edge case M16). | P5 Finding 4 |
| M14 | **MPC measured steering is dead:** `MPC_corr.py:1255` looks for `car_1_left/right_steering_hinge_joint`, which nothing publishes (car or sim); the object-exit ramp falls back to the last command (`MPC_corr.py:2379`). | SP §4 (2026-10-03 code check) |
| M15 | **Measure the real wheelbase.** URDF, `controllers.yaml` and the sim's `drive_bridge` use 0.325 m; `vesc_to_odom_node.wheelbase` (which drives the car's `/odom` yaw rate) is 0.305 m, a 6 % gap in ω = v·tanδ/L. The sim keeps 0.325 until measured. | SP P2 / D5 |
| M16 | **Participant never in the graph: 1 bringup in 60 with the lease fix.** FB5 proof run 29: `semantic_layer_node` was running and logged `started`, but was never in the graph from bringup to shutdown, while every other node was. Not FB4's signature (single endpoint pairs of participants that were in the graph); the previous run's processes died about 4 s before it started, so the 5 s lease window may just touch. Cause open. The watchdog cannot see it: `semantic_layer_node` only publishes with detections. Data: `output/fix_batch_5/isolation/proof/run_29`. | FB5 A4 |
| M17 | **Components the liveness watchdog judges only on "it runs".** wall_distance (`/perception/d_wall/segment`, a 10 Hz timer), behavior (`/behavior/tree_status`, every BT tick) and system_observer have no output that depends on their input; `semantic_layer_node` and `costmap_renderer` have no watchable output at all. A heartbeat like `mpc_corr`'s `/mpc/input_status` (FB5, decision 2b) would make them input-proving. **Decided 2026-10-05: do it, wall_distance first** (its input is `/scan`, the e-stop's sensor). | FB5 B2 |
| M20 | **`f1tenth_behavior`: `test_go_to_object_behaviour.py::TestFailureOutcomes::test_grace_then_target_lost` fails every run** (5/5): after the GRACE window it returns SUCCESS where the test expects FAILURE (`target_lost`). Pre-existing: it fails on the merge base `8c0891d`, on origin/jazzy and on local jazzy alike, so neither the sim work nor FB5 caused it. Not the flaky L11. Found in the post-merge test run of 2026-10-05. | FB5 merge check |

## LOW

| # | Item | Source |
|---|---|---|
| L1 | **Shutdown force-kill path.** Seen in 2 of 15 Phase 5 runs: launch's SIGTERM arrived before `_stop_all_components` returned, although every component had exited within 0.7 s. Unexplained; one instrumented run on the Orin. | P5 Decision 5 |
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
| L14 | **Give `/sensors/imu/raw` a real frame (`imu`) on car and sim.** Both publish `frame_id ""` today, which robot_localization reads as `base_link` (source-checked, correct numerically). If changed, `base_link→imu` must be on TF before any publisher stamps it, or the EKF drops the IMU. | SP B1 |
| L15 | **slam_toolbox's map interval grows with the session.** Before every publish it rebuilds the whole grid from every scan under the mapper lock (`updateMap()`, 2.8.5). In FB5's 10-minute looped-bag run: median gap 5 s at first, 12.5 s in the last third, one gap of **415 s**. costmap_boundary then works from a map that can be minutes old. The liveness watchdog judges `/slam/map` `once` per start for this reason. Check with a real drive; consider `map_update_interval` / map size limits. | FB5 B5 |

---

## Migration runs still pending on the Orin (tracked in the phase reports)

These are not post-migration work, but they block closing their phases.

| Item | Source |
|---|---|
| Phase 2 `costmap_boundary` frozen-input run (`output/phase2/ORIN_BOUNDARY_INSTRUCTIONS.md`) | P2 Addendum |
| Phase 3 MPC frozen/replay run and the Orin's osqp version (`output/phase3/ORIN_MPC_INSTRUCTIONS.md`) | P3 Decision 1 |
| Phase 4 BT replays, extract and py_trees versions (`output/phase4/ORIN_BT_INSTRUCTIONS.md`) | P4 Decision 2 |
| Phase 5 10-cycle comparison and the Orin's stack environment | P5 Decisions 2–3 |
| The 5 s lease under the Orin's real load (78–100% CPU): `scripts/fix_batch_5/run_batches.sh` with `BATCHES="proof stress"`; and the full-stack Humble isolation rate (FB4 decision 3) | FB4, FB5 |
| The topic-liveness watchdog's CPU under the Orin's real load: the supervisor's thread took 0.44 cores on Thor while fed (FB5 A5). Re-measure with `run_batches.sh` `BATCHES="dl_after dl_wd"` and decide whether to thin the subscriptions. | FB5 |

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
| Plain-client audit: `ekf_cost_observer_node` measured nothing | FB2 (old H6) | FB3 `a936c75` |
| Plain-client audit: `stackctl.py status` reported supervisor services missing | FB2 (old H6) | FB3 `d29b3e0` |
| Plain-client audit: steering calibration never saw `/safety_stop` | FB2 (old H6) | FB3 `5b320a5` + FB4 `b31b76c` (graph wait; 10/10 pass with the BT, 3/3 refuse without) |
| H1 Discovery isolation (stale participants in the reused Discovery Server; slam never ACTIVE, subscriptions never matched) | P5, FB3, FB4 | FB5 `7b301b2` (5 s lease profile for every participant: isolated bringups 8/30 → 1/60, p = 0.0005; slam hang 7/30 → 0/60; 0/20 under CPU saturation) + `dac4bef` (jazzy.sh) + `96a5b12`/`725f256` (supervisor topic-liveness watchdog; 0 false triggers in 80 bringups and 10 min, SIGSTOPped node restarted and OK in 34 s) + `0f151aa` (`/mpc/input_status`). Remaining 1/60: M16. |
| M19 Watchdog `once` check fired when its input appeared after the grace (restarted a healthy slam, 1 of 5 enforce runs) | FB5 B6 | `f9e0fea` (`settle_sec` for `once` checks, 15 s: 0 watchdog events in 10 late-input bringups, 5 live and 5 sim at RTF 0.74, plus 5 normal; first map 0.1–4.8 s after the first scan over 22 runs) |
