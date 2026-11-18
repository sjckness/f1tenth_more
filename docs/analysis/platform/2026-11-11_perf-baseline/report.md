---
id: 2026-11-11_perf-baseline
title: "Baseline prestazioni fase 1 (sola misura) dello stack Jazzy replayato su linus"
area: platform
date: 2026-11-11
tags: [perf-baseline, cpu, mpc-corr, osqp, replay, profiling]
modules: {mpc_controller: "0.0.0", f1tenth_perception: "0.0.1", f1tenth_costmap: "0.0.1", f1tenth_behavior: "0.0.1", f1tenth_bringup: "0.0.1"}
branch: "perf/baseline"
commit: "08079f57bb00"
status: done
hypothesis: "Misurare dove lo stack replayato consuma CPU, senza modificare file di produzione"
result: "Stack 3.75 core (56 processi); mpc_corr 69.3 % di un core (solver 11.3 ms: 82 % Python che costruisce il QP, 15 % creazione OSQP, 4 % solve); front_clearance_node 40.2 %; yolo 32.6 %"
decision: "Prima ottimizzare overhead per-tick del nodo MPC e assemblaggio QP, poi front_clearance_node e consumer sempre attivi di immagini full-rate; fase 2 attende via libera"
raw_data: "git exp/2026-11-11_perf-baseline:output/perf/20261111_0001/ (94 file .npz/.log/.txt/.json). File >1 MB non copiati: output/perf/20261111_0001/replay/probe_run1.json. Esterni: ~/f1tenth_archive/mission_row/2026-10-23T14-01-54_mission-llm_38d747d91edf (bag dell'auto, 120 s)"
migrated_from: "perf/baseline:output/perf/20261111_0001/REPORT.md"
migrated_on: 2026-11-18
---
# Performance baseline, phase 1 (measure only)

**Base branch: `hil-hydra-integration` @ `f686d815`, not `hil-isaac`.** It is the newest
Jazzy line (2026-11-10) and contains `hil-isaac` (`d175e96b`) plus 12 commits (Hydra
alongside the HIL sim). Work branch `perf/baseline`, worktree `~/dev_ws/f1tenth_more-perf`.
No production file was changed: the branch adds only `scripts/perf/` and this directory.

In one paragraph: on linus the replayed stack uses **3.75 cores**. The numeric functions
that level A could isolate add up to about **0.5 core**; the rest is node glue: callbacks
at sensor rate, logging, JSON and file I/O, message copies. The largest single consumer is
**mpc_corr at 69 % of a core, of which its solver is 11**. Inside the solver, **82 % of
the 11.3 ms is Python building the QP**, 15 % is creating and setting up a new OSQP object
every tick, and 4 % is OSQP solving. So the first things to change are the MPC node's
per-tick overhead and the QP assembly, then the per-frame work of `front_clearance_node`
and the always-on consumers of full-rate images.

## 1. What was run, on what, and what was not

| | |
|---|---|
| Machine | linus: i7-9700K (8 cores, no SMT), 15.9 GB RAM + 8 GB swap, RTX 5050 8 GB (driver 580.178.04), Ubuntu 24.04, kernel 7.0.0-42 |
| Governor | `powersave` (intel_pstate, EPP `performance`); not changed (sudo) |
| Software | ROS 2 Jazzy, rmw_fastrtps_cpp, Python 3.12.3, numpy 1.26.4, scipy 1.11.4, osqp 1.1.3, torch 2.8.0+cu128, ultralytics 8.4.90 (`~/dev_ws/jc_env.sh`) |
| Build | in the worktree, `--symlink-install`, 20 packages up to `f1tenth_bringup`; submodules `ackermann_mux` and `vesc` initialised (only `vesc_msgs` built); Hydra submodules untouched |
| **Machine state** | **shared during every measurement**: two Isaac Sim containers of other sessions (`recursing_lewin`, `more_spot_isaac`) held about 2.7 cores, 4.5 GB of GPU memory and most of the RAM; swap went from 5.3 to 7.4 GB used during level B |

Levels: **A** done (41 cases, golden files verified); **B** done with the limits below;
**C** done (`static_audit.md`).

Limits of level B, all of which lower or blur the numbers:
- **No bag on linus holds the raw camera.** The best one
  (`~/f1tenth_archive/mission_row/2026-10-23T14-01-54_mission-llm_38d747d91edf`, 120 s,
  recorded on the car) has `/scan` 40 Hz, `/odom` 50 Hz, the depth at 4.5 Hz and a clean
  480x270 JPEG of the left image at 4.9 Hz. `scripts/perf/replay_feeder.py` republishes
  the latest image (upscaled to 640x360 bgra8) and depth at 30 Hz with fresh stamps, so
  sizes and encodings are the car's but the content changes five times a second.
- **Only ~11 of the ~28 frames/s published reached the consumers** (0.92 MB messages,
  machine at 88 % CPU). Camera-driven nodes therefore did about a third of the work
  they would do at 30 Hz.
- **No IMU** in the bag (the EKFs fused wheel odometry only), no VESC, no mission. One
  `/mpc/goal_distance` of 8 m was sent 8 s into each run so that the MPC solves.
- **Sim time adds a cost the car does not have**: every Python node pays for `/clock`
  (50 Hz here). Nodes that do nothing else sat at 3.2 % of a core; read every Python
  node's figure with that in mind.
- **The warm-up was a 25 s run**, not 60 s: it was the harness test, and a third full
  start of the stack would have pushed the other sessions further into swap. The two
  measured runs are 60 s each (first 8 s dropped).
- **Solve time cannot be read from `/mpc/solver_status` in sim time**: `solve_dt_sec` is
  taken from the ROS clock, which advances in 20 ms steps, and comes out as 0.0.
- No profile of a live node: `ptrace_scope` is 1 (`SUDO_TODO.md` #1).

Not covered by level A: `hard_stop.py`, the behaviour tree's behaviours, the glue of
`MPC_corr.control_loop` and of `semantic_layer_node`, `wall_boundaries.build_wall_rows`,
the GPU mask post-processing, and the parts that are off by default (`glass_detect`,
unknown objects, the reverse cost-to-go lattice). There is no "adaptive corridor" on
this branch: the corridor is `build_straight_corridor` plus `compute_local_target`.

## 2. Per node (level B)

`replay/nodes.csv`, `replay/topics.csv`. CPU is % of one linus core, mean of the two
runs; "diff" is the difference between them. Rates are received by the probe.

| node | CPU % | RSS MB | diff % | configured -> real rate | p95 period |
|---|---|---|---|---|---|
| mpc_corr | **69.3** | 134 | 2.5 | timer 10 Hz -> `/drive` 9.97 Hz, solves 9.5 / 8.9 Hz | 110 / 104 ms |
| front_clearance_node | **40.2** | 195 | 2.3 | per depth frame -> 10.6-11.1 Hz; scene mask 5 Hz -> **3.7 Hz** | mask 439 / 400 ms |
| yolo_detector_node | 32.6 | 1697 | 3.0 | gate 15 Hz -> detections 8.3-8.6 Hz | dropped (probe lost 7 / 20 msgs) |
| lidar_front_wall_node | 27.8 | 133 | 2.4 | per scan 40 Hz -> 39.5 Hz; tracker 10 Hz -> 9.88 Hz | 30 ms; 110 ms |
| behavior_executor_node | 23.0 | 130 | 0.3 | tick 10 Hz -> 10.0 Hz | 102 ms |
| freespace_door_node | 22.4 | 112 | 1.3 | 10 Hz -> **8.6 Hz** | 140 ms |
| semantic_layer_node | 16.0 | 127 | 0.2 | 10 Hz -> 10.0 Hz | 102 ms |
| viz_relay_container (C++) | 15.9 | 85 | **14.9** | lazy relays, no client | - |
| detection_3d_node | 14.6 | 176 | 0.3 | <= 15 Hz -> 7.8 Hz | 320 / 313 ms |
| hard_stop_node | 14.3 | 88 | 1.0 | per scan 40 Hz (not probed) | - |
| reset_manager | 11.4 | 89 | 0.8 | event-driven, idle | - |
| costmap_boundary_node | 10.0 | 97 | 2.1 | 10 Hz -> 10.0 Hz | 102 ms |
| obstacle_clearance_node | 7.4 | 89 | 0.6 | per scan 40 Hz (not probed) | - |
| foxglove_bridge (C++) | 7.0 | 93 | **6.1** | no client | - |
| 16 `ros2 launch` wrappers | 6.4 | **905** | 3.6 | - | - |
| image_raw_viz_throttle (C++) | 5.7 | 52 | 1.7 | 5 Hz output nobody reads | - |
| costmap_renderer_node | 4.2 | 88 | 0.9 | 2 Hz | - |
| obstacle_projector_node | 4.1 | 94 | 0.5 | per detections_3d -> 7.8 Hz | 320 ms |
| ekf_filter_node (C++) | 4.0 | 53 | 1.0 | **20 Hz -> 43.4 / 40.6 Hz** | 40 ms |
| diagnostics_server_node | 4.0 | 89 | 0.5 | 2 Hz | - |
| joint_state_publisher, viz_jpeg_node | 3.8 each | 86, 114 | < 3 | 10 Hz -> 10.0 Hz; lazy | 102 ms |
| ackermann_mux, system_observer, twist_to_ackermann, slam_pose_relay | 3.2-3.4 each | 42-87 | < 2 | idle | - |
| ekf_global_filter_node (C++) | 3.0 | 52 | 0.6 | 20 Hz -> 19.9 / 19.8 Hz | 80 ms |
| slam_toolbox (C++) | 2.9 | 81 | 0.7 | - | - |
| image_annotated_viz_throttle (C++) | 1.6 | 50 | **7.2** | - | - |
| **stack total (56 processes)** | **375** | **5 710** | 0.8 | | |
| probe (harness) | 37.5 | 98 | 0.2 | | |
| feeder + bag player (harness) | 21.7 | 250 | | | |
| everything else on the machine | 272 | | 0.5 | | |

- **Repeatability is good**: every node above 2 % of a core differs by less than 5 %
  between the two runs, except `viz_relay_container` (14.9 %), `foxglove_bridge` (6.1 %)
  and `image_annotated_viz_throttle` (7.2 %). On those three a small gain will not be
  measurable this way.
- **The probe is heavy**: 37.5 % of a core for about 330 messages/s on a saturated
  machine. It lost messages on `/camera/detections` (7 and 20) and `/scan` (1 and 2);
  their period statistics are left out, their counts are kept.
- **`/tf`**: 73 / 70 messages/s. `odom->base_link` 43.4 / 40.6 Hz (p95 40 ms),
  `map->odom` 19.9 Hz (p95 80 ms), the six wheel joints 10.0 Hz each.
- **The local EKF published at 40-43 Hz, twice its configured 20 Hz.** The launch
  override is in place and the car's bag shows 19.9 Hz, so this belongs to sim time; the
  cause was not established. It doubles the callbacks of every `/odometry/filtered`
  consumer in the Isaac HIL setup as well, if it happens there too.
- **Short-lived children are not attributed** in these two runs: the behaviour tree's
  `dot` (a PNG every 0.4 s) is inside "everything else". It showed 13 % of a core in the
  warm-up when the sampler caught it. `probe.py` now records such children.
- **MPC**: 493 and 460 solves, all `success`, solver `rti`, 1 boundary row, 0 obstacles.
  Iterations are not published; level A measured p50 25, p95 50, max 75.
- **Wall tracker cycle, timed by the node itself**: p50 13.1 / 11.6 ms, p95 26.5 / 25.5 ms.
- **YOLO**: `yolo26s-seg.pt` on `cuda`, `gpu_pipeline` on, backend `pt`, imgsz 640
  letterboxed, fp16 off (node log). The node logs no latency with the default
  parameters (`perception_stage_timing` false). Alone, `predict()` on a 640x360 frame
  takes 11.6 ms p50, 14.6 ms p95 on this GPU while it is shared
  (`yolo_standalone.json`): the 8.5 Hz above is the frame delivery, not the GPU.

What the car recorded about itself in the same bag (`orin_reference.json`; Orin, the
code of 2026-10-23, so not this branch):

| stage | car p50 | car p95 | linus |
|---|---|---|---|
| MPC solve (`solve_dt_sec`) | 25.3 ms | 35.9 ms (max 69.8) | 11.3 ms (level A, this branch's QP) |
| wall tracker cycle | 26.1 ms | 40.1 ms | 11.6-13.1 ms (level B, same scans) |
| scene mask tick | 41.3 ms | 63.2 ms | 18.9 ms (sum of its level A parts) |
| scene labels tick | 8.2 ms | 14.9 ms | 3.1 ms (level A) |

The same Python stage is **2.0 to 2.7 times slower on the Orin** by these four pairs
(different code versions and inputs: an order of magnitude, not a calibration).

## 3. Top 10 hot spots (level A)

`micro.csv`, measured pinned to one core, 20 warm-up + 200 calls. "per cycle" is the cost
in one 100 ms control period: p50 times the calls in that period. Paths are under `src/`.

| # | function | file:line | p50 / p95 / max ms | calls per cycle | ms per cycle |
|---|---|---|---|---|---|
| 1 | `solve_mpc_step`, forward RTI, N = 20, offset-free, 2 wall rows, 2 obstacles | `f1tenth_control/mpc_controller/mpc_controller/mpc_solver.py:327` | 11.31 / 12.21 / 12.77 | 1 | **11.3** |
| 2 | freespace doors: `scan_profile` + `occupancy_window` + candidates | `f1tenth_costmap/f1tenth_costmap/freespace_doors.py:412` | 9.83 / 10.49 / 12.11 | 1 | **9.8** |
| 3 | stereo columns: `wall_points` + `column_ranges` + `column_extent` | `f1tenth_perception/f1tenth_perception/scene_mask.py:173`, `wall_fusion.py:112` | 14.88 / 16.45 / 18.11 | 0.5 | **7.4** |
| 4 | `MultiWallTracker.step` + roles + published | `f1tenth_perception/f1tenth_perception/multi_wall_tracker.py:1787` | 4.31 / 7.53 / 8.11 | 1 | 4.3 |
| 5 | `extract_boundary_constraints` (+ `nearest_occupied_in_strip` 0.60) | `f1tenth_costmap/f1tenth_costmap/costmap_boundary.py:293`, `:157` | 3.37 / 4.05 / 4.39 | 1 | 4.0 |
| 6 | `sequential_lines` + `heading_wall` | `f1tenth_perception/f1tenth_perception/scene_labels.py:255` | 1.76 / 2.47 / 2.96 | 1 | 1.8 |
| 7 | scene labels tick (back-project, ground fit, labels, clusters) | `f1tenth_perception/f1tenth_perception/scene_labels.py:391` | 3.08 / 3.76 / 4.07 | 0.5 | 1.5 |
| 8 | `_linearize_obstacle_cost_rti` (inside 1) | `.../mpc_solver.py:687` | 0.059 / 0.066 / 0.21 | 20 | 1.2 |
| 9 | `planner_cost_corridor` (inside 1; reporting only) | `.../mpc_solver.py:1510` | 1.13 / 1.43 / 1.49 | 1 | 1.1 |
| 10 | `rasterize_walls` 240x240 | `f1tenth_costmap/f1tenth_costmap/wall_raster.py:63` | 2.09 / 2.78 / 3.10 | 0.5 | 1.0 |

Next: `verify_rectangles` 0.95, `class_stats` 0.92, `_mask_median_depth` 0.86,
`cross_check_profile` 0.81, `cv_bridge.imgmsg_to_cv2` of a depth frame 0.70 (0.12 ms, six
times per cycle at 30 Hz), `scene_mask.classify` 0.58. All default-stack cases together:
**about 48 ms per 100 ms cycle**.

Where the 11.3 ms of the forward solve go (timed around each OSQP call, same inputs):

| part | p50 ms | share |
|---|---|---|
| Python: rollout, linearisation, dense P / q / A, dense -> CSC, cost | 9.33 | 82 % |
| `osqp.OSQP()`: a new solver object each tick | 0.84 | 7 % |
| `setup()`: factorisation from scratch | 0.82 | 7 % |
| `solve()` with polish | 0.41 | 4 % |

Other profiles (`profiles/`): freespace doors spend 76 % in `scipy.ndimage` binary
erosion on a window whose map changes every 5 s; the stereo columns spend their time in
`back_project` (34 %), a full `numpy.sort` (29 %) and `_usable` (16 %), with a 12.7 MB
allocation peak; the tracker is spread over many small Python functions. The profile
file of the solve is not usable around the OSQP calls (pybind11 methods are invisible to
cProfile); use the table above.

Costly but not in the default stack: `build_safe_corridor` 12.2 ms p50, 30.7 ms p95
(`use_convex_polytope`); `scene_mask.overlay` 6.7 ms (only with a subscriber); the Python
extraction backend 2.7 ms against 0.43 ms for the C one; the lidar door cycle 1.8 ms; the
rear QP 3.9 ms (only while stuck); `inferred_floor.build_lut` 29 ms (once).

**Noise**: two identical runs of the whole benchmark differed per case by 1.7 % (median),
4.9 % (p90), 8 % (max) in the first pair and by 3.9 %, 9.7 %, 12 % in the pair kept here
(`micro.csv`, `micro_rerun.csv`). On this machine in this state a gain under about 10 %
on one function is not measurable.

**Golden files**: `golden/*.npz`, 41 cases, 4.9 MB. `bench_micro.py --check-golden`
reproduced all 41 twice at rtol 1e-9 (`golden_check.csv`), so every case is deterministic.

## 4. Optimisation candidates

Ordered by expected gain over risk. Gains are for linus: "points" are % of one core in
the level B table, "ms" are per 100 ms cycle in level A. "Orin" says whether the gain
should carry over to the car.

| # | what | evidence | expected gain | risk | Orin | golden |
|---|---|---|---|---|---|---|
| 1 | **mpc_corr outside the solver**: about 136 log lines/s, a JSON corridor snapshot and three flushed files per tick, `/scan` and `/perception/lidar_front_wall_virtual` callbacks at 40 Hz each | node 69 points, solver 11; 38 points while not solving (warm-up run) | 20-35 points; **profile the live node first** (`SUDO_TODO.md` #1) to split it | low for logging and I/O, medium for the subscriptions | yes; the car has no `/clock`, so re-measure there | none needed for logging; `mpc.solve_mpc_step...npz` guards the solver |
| 2 | **QP assembly in `_solve_rti`**: fixed sparsity filled in place instead of ~320 dense rows and a dense P of about 145x145; drop the duplicate model step; vectorise `planner_cost_corridor` and the obstacle linearisation | 9.3 of 11.3 ms | 5-7 ms per tick (12-17 ms on the car by the ratio above) | medium: results must stay within tolerance, and the bit-exact digests of `test/closed_loop_golden.py` will change | yes, re-verify on ARM | `mpc.solve_mpc_step...npz` (replays 40 closed-loop solves), `mpc._linearize_obstacle_cost_rti`, `mpc.planner_cost_corridor`, `mpc._rollout_true_dynamics` |
| 3 | **Always-on readers of full-rate images that nobody consumes**: `image_raw_viz_throttle`, `image_annotated_viz_throttle` (which also makes yolo draw and publish the 691 KB overlay on every frame), `depth_throttle_5hz` (mission logger, on by default) | 5.7 + 1.6 points measured; the logger was off in the replay; frames delivered at 11 of 28 Hz | 7 points directly, an unmeasured part of yolo's 33, and more frames delivered | low | yes | none (no numeric output changes) |
| 4 | **front_clearance_node**: gate the per-depth-frame path to 10-15 Hz before decoding, as yolo does; stop decoding `/camera/image_raw` and `/scan` just to store them; stereo columns without the full sort | 40 points at 11 frames/s; columns 14.9 ms per mask tick; mask at 3.7 Hz instead of 5 | 15-25 points, more at the car's frame rate; 3-7 ms per mask tick | low to medium: check who reads `/perception/front_distance` at frame rate | yes | `camera.scene_mask.*.npz`, `camera.scene_labels...npz`, `camera.inferred_floor...npz` |
| 5 | **freespace_door_node**: cache the window's morphology and distance transform until the map or the window moves; replace the `TransformListener` | 9.8 ms per cycle, 76 % morphology; node 22 points, 8.6 Hz instead of 10 | 6-7 ms per cycle plus 2-3 points | low to medium (the window follows the robot) | yes | `doors.freespace_doors...npz` |
| 6 | **hard_stop_node**: vectorise the per-beam loop, publish the JSON status on change | 14 points for a 40 Hz scan callback | about 10 points | medium: safety path, the stop decision must be identical | yes | **none yet: add a case before touching it** |
| 7 | **Behaviour tree PNG render** every 0.4 s with no subscriber check | node 23 points; `dot` 13 points when sampled | 5-10 points plus `dot` | low (visualisation) | yes | none |
| 8 | **lidar_front_wall_node**: publish `WallEstimate` at 20 Hz instead of per scan (rule: MPC inputs at 10-20 Hz), cache the per-beam ray table, tracker internals | node 28 points; tracker cycle 12-13 ms; 40 Hz into mpc_corr | 5-8 points here, part of #1 in mpc_corr | medium (virtual wall timing) | yes | `walls.*.npz` |
| 9 | **Persistent OSQP object** with `update()` | `OSQP()` 0.84 + `setup()` 0.82 ms per tick | about 1.4 ms per tick | medium: fixed sparsity needed, iterates change within tolerance | yes | `mpc.solve_mpc_step...npz` with a looser tolerance, stated |
| 10 | **costmap_boundary_node**: skip the tick when map and pose are unchanged, one int16 view instead of four copies | 4.0 ms per tick, node 10 points | about 3 points | low | yes | `costmap.costmap_boundary.*.npz` |
| 11 | **detection_3d_node**: one depth-validity mask per frame; do not deserialise depth frames it will not use | node 15 points at 7.8 Hz; 0.57 ms per frame of two detections | 3-5 points | low | yes | `camera.Detection3DNode._mask_median_depth.npz` |
| 12 | **semantic_layer_node**: no wall layer without a consumer; `/scan` at its 10 Hz cycle | raster 2.1 ms at 5 Hz; node 16 points | 3-5 points | low | yes | `costmap.wall_raster...npz` |
| 13 | **Sim and HIL only**: `reset_manager` 11 points idle (4-thread executor waking on `/clock`), `viz_relay_container` 16 points, local EKF at twice its rate | level B | up to 30 points in the HIL setup, none on the car | low | no | none |
| 14 | **0.92 MB images over DDS** with default transport settings | 28 Hz published, 11 Hz delivered | sets the perception rate rather than the CPU | medium (transport profile on the car) | re-verify: the car's profile differs | none |

Two things to fix before measuring phase 2, both small: take `solve_dt_sec` from a steady
clock so it means something in sim and HIL, and turn on `perception_stage_timing` in the
sim overlay so yolo logs its latency.

## 5. Regression checks (level C)

Full table with file:line in `static_audit.md`, section 5.

| check | result |
|---|---|
| ZED `publish_imu_tf` false at launch | **present and effective**: `src/f1tenth_perception/launch/camera.launch.py:146` passes `'publish_imu_tf': 'false'`; the pinned wrapper (86171bc) declares the argument and maps it to `sensors.publish_imu_tf` after the YAML. The car's 2026-10-23 bag has no `zed2_imu_link` edge on `/tf` |
| both EKFs at 20 Hz | **present by launch override** (`ekf_frequency_hz` 20.0, `stack_params.yaml:2619`; `ekf.launch.py:99-103`, `ekf_global.launch.py:167-170`); the YAMLs still read 50. Car bag: 19.9 Hz both. **Replay in sim time: local EKF 40-43 Hz** (section 2) |
| `ekf_cost_observer` removed | **absent** (only a test asserting its removal mentions it) |
| `wall_distance_node` removed | **still present, off by default** (`enable_wall_distance: false`, `stack_params.yaml:1096`; entry point `src/f1tenth_perception/setup.py:51`) |
| BT tick rate and idle load | 10 Hz (`bt_tick_hz`, `stack_params.yaml:2637`), measured 10.0 Hz; 23 points with no mission, plus the PNG render |
| `costmap_boundary` <= 10 Hz | 10 Hz (launch default in `costmap.launch.py:151`, not in stack_params); measured 10.0 Hz |
| `semantic_layer` <= 10 Hz | cycle 10, memory 10, wall layer 5 Hz; measured 10.0 Hz |
| `reset_manager` <= 10 Hz | event-driven, no timer outside a reset; still 11 points idle in sim time |
| RMW and discovery | `rmw_fastrtps_cpp`; Fast DDS Discovery Server on 127.0.0.1:11811, 5 s lease, SUPER_CLIENT only for the foxglove tree, the viz relays, the logger tree, the gateway and the ctl probes |

Two things the audit found that are not on the list: **`enable_mission_logger` defaults
to true** (`stack_params.yaml:2727`) while its own description and
`mission_logger.launch.py:12-13` say it is off; and
**`/perception/lidar_front_wall_virtual` reaches mpc_corr at 40 Hz**, against the
10-20 Hz rule.

## Files

`micro.csv`, `micro_rerun.csv`, `golden/`, `golden_check.csv`, `profiles/` (level A);
`replay/` (level B: `nodes.csv`, `topics.csv`, raw samples, logs of both runs);
`static_audit.md` (level C); `orin_reference.json`, `yolo_standalone.json`;
`SUDO_TODO.md`. Tools and how to rerun them: `scripts/perf/README.md`.
