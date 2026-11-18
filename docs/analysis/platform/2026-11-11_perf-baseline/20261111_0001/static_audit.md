# Static audit (level C) — `perf/baseline` = `hil-hydra-integration` @ f686d815

Read-only. Nothing in this file is a measurement: every cost is an inference from
the code. Three read-only passes (control/navigation/BT, perception/costmap,
diagnostics/logger/DDS) were consolidated here; the items marked **[checked]** were
re-read by hand afterwards, the others are as reported by the pass that covered them.

Abbreviations: `SP` = `src/f1tenth_params/config/stack_params.yaml`,
`SUP` = `src/f1tenth_bringup/f1tenth_bringup/component_supervisor_node.py`,
`MPC` = `src/f1tenth_control/mpc_controller/mpc_controller/MPC_corr.py`,
`SOLV` = `.../mpc_controller/mpc_solver.py`,
`BEN` = `src/f1tenth_behavior/f1tenth_behavior/behavior_executor_node.py`,
`P/` = `src/f1tenth_perception/f1tenth_perception/`, `PL/` = `src/f1tenth_perception/launch/`,
`C/` = `src/f1tenth_costmap/f1tenth_costmap/`, `CL` = `src/f1tenth_costmap/launch/costmap.launch.py`,
`BL` = `src/f1tenth_bringup/launch`.

Project rule used for the "rule" column: **everything that feeds the MPC (10 Hz) runs
at 10–20 Hz.**

## 1. What starts in a default car bringup

Auto-start sets: `_ALWAYS_AUTO_START` `SUP:326-329`, `_NEVER_AUTO_START` `SUP:334`,
`_CONDITIONAL_AUTO_START` `SUP:343-360`. Each launch entry of a component is its own
`ros2 launch` Python process (`SUP:572-583`): 18 launch processes by default, on top of
the nodes.

| component | nodes | default | flag (default, line) |
|---|---|---|---|
| hardware | battery_voltage_check_node (one-shot), vesc_driver_node, ackermann_to_vesc_node, vesc_to_odom_node, static base_link→imu | yes | `calibration` false (`SP:141`) |
| localization | ekf_filter_node, ekf_global_filter_node, slam_pose_relay_node, robot_state_publisher, joint_state_publisher (Python), static base_link→laser | yes | `localization_source` ekf (`SP:48`) |
| perception | ZED wrapper + static TF; urg_node; yolo_detector_node, detection_3d_node, obstacle_projector_node, front_clearance_node | yes | `camera_source` zed (`SP:43`), `use_lidar` true (`SP:415`) |
| lidar_front_wall | lidar_front_wall_node (wall tracker, door detector in-process) | yes | `use_lidar` |
| obstacle_clearance | obstacle_clearance_node | yes | `use_lidar` |
| control | hard_stop_node, ackermann_mux (C++) | yes | – |
| navigation | mpc_corr, map_server + lifecycle_manager_map | yes | `enable_nav2` false (`SP:112`) |
| slam | slam_toolbox; semantic_layer_node, freespace_door_node, costmap_renderer_node, costmap_boundary_node | yes | `enable_slam` true (`SP:2700`) |
| behavior | behavior_executor_node, twist_to_ackermann_node | yes | `use_behavior_tree` true (`SP:104`) |
| diagnostics | system_observer_node, diagnostics_server_node, mission_logger_node + `depth_throttle_5hz` | yes | `enable_sys_obs` true (`SP:2800`), `enable_mission_logger` **true** (`SP:2726-2727`) **[checked]** |
| dev_tools | foxglove_bridge, image_raw_viz_throttle, image_annotated_viz_throttle, viz_relay_container (17 ThrottleNodes), viz_jpeg_node | yes | `enable_foxglove` true (`SP:3139`), `enable_viz_relays` true (`SP:3223`) |
| reset_manager | reset_manager | yes | – |
| intelligence | llama-server, llm_warmup | no | `enable_intelligence` false (`SP:100`) |
| wall_distance | wall_distance_node | no | `enable_wall_distance` false (`SP:1096`) |
| swept_clearance | swept_clearance_node | no | `enable_swept_clearance` false (`SP:1108`) |
| comms | wifi_coverage_node, link_manager | no | `enable_comms` false (`SP:4339`) |
| calibrate_hardware, startup_sequence | – | never | `SUP:334` |

Outside the supervisor: Fast DDS Discovery Server and the supervisor node
(`BL/supervisor_bringup.launch.py:134-141, 267-273`); LLM2 only through
`stack_bringup.launch.py` (`enable_llm2` false); Hydra standalone; gateway, f1t-monitor
and the planner are systemd units / the container main.

## 2. Per-node table

All Python nodes are separate processes on rclpy's single-threaded executor unless
noted. "Pinning" is the `taskset -c` prefix in the launch file (Orin core ids).

### 2.1 Control, navigation, localization, behaviour, bringup

| node | file | lang | pinning | default | timers and publishers | rule | heavy topics, copies | TF / log / alloc / cache |
|---|---|---|---|---|---|---|---|---|
| mpc_corr | `MPC` | Py | `10,11` (`mpc_corr.launch.py:306,350`) | yes | Timer 10 Hz, `self.ts = 0.1` hard-coded (`MPC:859, 2309`). Per tick: `/drive`, `/mpc/solver_status`, `/mpc/status` (JSON), two obstacle distances, `/mpc/predicted_clearance`, `/mpc/reverse_status`, `/mpc/speed_channel`, `/mpc/wall_constraints_diag` (JSON). Corridor rebuild 1 Hz (`corridor_update_period` 1.0, `SP:1293`; `MPC:5589-5608`) | loop OK. Inputs: `/odometry/filtered` 20 Hz OK; **`/scan` 40 Hz** (`MPC:2117`) read only in wall_turn; **`/perception/lidar_front_wall_virtual` arrives at 40 Hz** (`MPC:2041`) | LaserScan deserialised per scan; rear-costmap OccupancyGrid `np.asarray(msg.data, int16)` per message (`MPC:4700-4707`) | see 3.1 |
| andre_mpc_node | `mpc_controller/andre_mpc_node.py` | Py | – | no (in no component) | Timer 10 Hz, SLSQP | – | – | – |
| model_log | `mpc_controller/model_log.py` | Py | library | no | – | – | – | `setup.py` entry point targets a `main` that does not exist; the writer flushes every row |
| hard_stop_node | `f1tenth_control/hard_stop_node.py` | Py | none | yes | Timer 20 Hz (`:104,157`, not in SP); `/safety/hard_stop` JSON on every scan (`:209`) | 40 Hz event-driven; safety path, not an MPC input | `/scan` sensor QoS | **per-beam Python loop with `math.cos/sin` on every scan** (`hard_stop.py:95-100, 116-120`) plus `json.dumps` per scan |
| ackermann_mux | external C++ | C++ | none | yes | per input | not read | – | – |
| ekf_filter_node | robot_localization | C++ | `0,1` (`ekf.launch.py:66,105`) | yes | 20 Hz (see 5) | OK | – | `print_diagnostics: true` |
| ekf_global_filter_node | robot_localization | C++ | `0,1` (`ekf_global.launch.py:132,173`) | yes | 20 Hz | OK | – | same |
| slam_pose_relay_node | `f1tenth_localization` | Py | none | yes | event relay, ≤ 2 Hz | OK | – | trivial |
| robot_state_publisher / joint_state_publisher / static TF | `f1tenth_description/launch/description.launch.py:118,181,203` | C++ / **Py** / C++ | none | yes | joint_state_publisher at the package default 10 Hz (not read) | OK | – | feeds `/joint_states` into mpc_corr (`MPC:1539`) |
| slam_toolbox (async) | `slam.launch.py:123-130` | C++ | `2` | yes | `map_update_interval` 5 s, `minimum_time_interval` 0.5, `throttle_scans` 1 | consumes `/scan` 40 Hz | `/slam/map` every 5 s | – |
| map_server + lifecycle_manager | `map_only.launch.py` | C++ | none | yes | latched `/map` | – | static grid | – |
| Nav2 stack | `nav2.launch.py` | C++ | – | no | controller 30 Hz, BT 10 Hz | would exceed the rule | costmaps | – |
| behavior_executor_node | `BEN` | Py | `4` (`behavior_bringup.launch.py:231,250`) | yes | see 3.2 | tick OK; `/scan` 40 Hz e-stop path | `/scan` reliable depth 10; PNG | see 3.2 |
| twist_to_ackermann_node | `f1tenth_behavior` | Py | none | yes | event on cmd_vel | idle without Nav2 | – | – |
| component_supervisor_node | `SUP` | Py | none | yes | watchdog 0.5 Hz (`SP:3119`; `SUP:836`), `Popen.poll()` only | – | – | negligible |
| reset_manager | `f1tenth_bringup/reset_manager_node.py` | Py | none; MultiThreadedExecutor(4) | yes | event-driven (see 5) | – | – | idle |
| foxglove_bridge | `BL/foxglove_bridge.launch.py:157-183` | C++ | `3` | yes | whitelist `^/viz/.*`, tf, maps (`SP:3172-3193`) | – | `/slam/map`, `/map`, `/tf` unthrottled | its own docstring records ~52 % of a core at rest (`:59-66`) |
| image_raw_viz_throttle, image_annotated_viz_throttle | `BL/foxglove_bridge.launch.py:190-214` | C++ topic_tools | **none** | yes | 5 Hz (`SP:3150`) | – | **always-on subscribers of the two full-rate images; their outputs are not on the bridge whitelist and no consumer was found** | – |
| viz_relay_container | `BL/viz_relays.launch.py:69-85` | C++, 17 composable ThrottleNodes | `3` | yes | 5.5 / 3.3 Hz, `lazy: True` | – | lazy | idle without a client |
| viz_jpeg_node | `f1tenth_bringup/viz_jpeg_node.py` | Py | `3` | yes | 1 Hz subscription check | – | lazy; `np.frombuffer` + `imencode`, no cv_bridge | – |
| vesc_driver / vesc_to_odom / ackermann_to_vesc | `f1tenth_hardware/launch/vesc.launch.py` | C++ | none | yes | not read (submodule) | `/odom` ~50 Hz feeds the EKF | – | – |

### 2.2 Perception, corridor extraction, costmap

| node | file | pinning | default | timers and publishers | rule | heavy topics, copies | TF / log / alloc / cache |
|---|---|---|---|---|---|---|---|
| yolo_detector_node | `P/yolo_detector_node.py` | `8,9` (`PL/detection.launch.py:377,432`), 2 threads (`SP:376`) | yes | per processed frame, ≤ 15 Hz (`yolo_max_rate_hz` 15.0, `SP:395`; gate `:679-685`); debug JPEG 5 Hz | OK. Every 30 Hz frame is still deserialised before the gate drops it | `/camera/image_raw` BEST_EFFORT depth 1; GPU path: `np.frombuffer` view, one upload; masks → `.cpu().numpy()` → `cv2_to_imgmsg`. With an annotated-image subscriber: mask download, `cvtColor`, two 691 KB copies, overlay, `cv2_to_imgmsg` | no TF listener; logs throttled |
| detection_3d_node | `P/detection_3d_node.py` | `6,7` | yes | per synced det+depth+mask, ≤ 15 Hz | OK | depth and masks through `message_filters` RELIABLE depth 10: **every 30 Hz depth frame deserialised, ≤ 15 Hz used**; `imgmsg_to_cv2` ×2; per detection a full-frame `(mask==label) & isfinite(depth) & (depth>0)` (`:747`) | `lookup_transform` of a fixed edge every frame (`:611`) |
| obstacle_projector_node | `P/obstacle_projector_node.py` | `6,7` | yes | per detections_3d message | OK | – | static lookup per message (`:232`) |
| front_clearance_node | `P/front_clearance_node.py` | **none** | yes | **per depth frame (30 Hz configured)**: 4 topics + JSON debug (`:1826-1832`, `:2052`). Scene labels 5 Hz, scene mask 5 Hz, watchdog 2 Hz | **30 Hz > 20**; not an MPC input any more. Its 5 Hz stereo products feed the wall tracker | depth `imgmsg_to_cv2` every frame (`:1722`); mask `imgmsg_to_cv2` + `astype(float32)` every depth frame (`:1544-1550`); full-frame float32 EMA (`:1635`). **`/camera/image_raw` at 30 Hz only to cache a message for an overlay nobody subscribes to** (`:858-859`). `/scan` 40 Hz store-only | `/tf_static` only; ROI and housing masks rebuilt every frame (`:2061-2082`) |
| front_depth_monitor_node | `P/front_depth_monitor_node.py` | – | no (no launch file) | – | – | – | – |
| lidar_front_wall_node | `P/lidar_front_wall_node.py` + `multi_wall_tracker.py`, `wall_fusion.py`, `scene_labels.py`, `door_detection.py` | `5` | yes | **per scan, 40 Hz**: `/perception/lidar_front_wall`, `/perception/lidar_front_wall_virtual` (`:2270-2299`). Virtual wall + debug JSON 10 Hz, wall planes 10 Hz, tracker 1 scan in 4 = 10 Hz (`SP:4287`) | **`lidar_front_wall_virtual` at 40 Hz is read by mpc_corr**; the rest 10 Hz OK | `/scan` BEST_EFFORT depth 1; classes + depth_mm as `np.frombuffer` views | `scan_points` recomputes bearings, sin, cos, `arctan2` for all beams on every scan (`P/scene_labels.py:157-171`); large `json.dumps` at 10 Hz (`:1636`) |
| obstacle_clearance_node | `P/obstacle_clearance_node.py` | `5` (**same core as lidar_front_wall**) | yes | per scan, 40 Hz | only the test-campaign logger reads it | `/scan` | small numpy |
| swept_clearance_node | `P/swept_clearance_node.py` | `5` | no | timer 20 Hz | – | two full-frame depth copies | – |
| wall_distance_node | `P/wall_distance_node.py` | `5` | no | timer 10 Hz | – | `/scan` | – |
| costmap_boundary_node | `C/costmap_boundary_node.py` | none | yes | timer 10 Hz (`:582`) | OK | **four full-map `np.asarray(grid.data, int16)` copies per tick** (`C/costmap_boundary.py:130` once, `:227` ×3), no change detection | info log every tick (`:906`) |
| semantic_layer_node | `C/semantic_layer_node.py` | none | yes | cycle 10 Hz (`SP:3586`), memory 10 Hz (`SP:3679`), wall layer 5 Hz (`CL:164`) | OK | 240×240 OccupancyGrid + 57 600-int array at 5 Hz with no live consumer (`:1062-1067`); `/scan` 40 Hz store-only | `get_parameter` in a callback (`:1052`) |
| costmap_renderer_node | `C/costmap_renderer_node.py` | none | yes | timer 2 Hz | viz only | full map → rgb8 every tick, no subscriber gate (`:103-116`) | – |
| freespace_door_node | `C/freespace_door_node.py` | none | yes | scan-driven, throttled to 10 Hz (`SP:4024`) | OK | `/slam/map`, `/scan` 40 Hz; per cycle a full-map int16 copy, morphology and `distance_transform_edt` on an 8 m window (`C/freespace_doors.py:441-462`) | **a full `TransformListener` (`:236`) for one static edge** |
| corridor_perception(_c) | library | – | in-process (lidar_front_wall, 10 Hz) | – | – | `np.ascontiguousarray(ranges, float32)` | C backend `-O3`, default backend `c` |

### 2.3 Diagnostics, logger, comms, gateway, LLM

| node | file | pinning | default | timers and publishers | heavy topics, copies | notes |
|---|---|---|---|---|---|---|
| system_observer_node | `f1tenth_diagnostics/system_observer_node.py` | none | yes | 1 Hz | – | `psutil.cpu_percent` ×2 + `virtual_memory` per tick, no process scan |
| diagnostics_server_node | `…/diagnostics_server_node.py` | none | yes | 2 Hz battery, 2 Hz calibration poll | – | one callback per `/sensors/core` message for a 2 Hz mean; `get_node_names()` at 2 Hz |
| mission_logger_node | `f1tenth_logger/mission_logger_node.py` | none; SUPER_CLIENT | yes | idle: 1 Hz check | recording: ~100 topics serialised incl. `/scan`, `/tf`, `/slam/map`, masks, depth 5 Hz; sqlite3 | graph polled every 100 ms for the whole run; finalize thread hashes and extracts the bag while the stack runs |
| depth_throttle_5hz | `f1tenth_logger/launch/mission_logger.launch.py:104-115` | none; SUPER_CLIENT | yes | 5 Hz | **permanent full-rate subscriber of the 0.92 MB depth image**, not lazy | runs whenever the logger is enabled, not only during a mission |
| foxglove tree | see 2.1 | | | | | |
| wifi_coverage_node, link_manager | `f1tenth_comms` | none | no | 1 Hz, 2 Hz | – | polled TF on an unspun side node (`tf_poll.py`) |
| pwa_gateway | `f1tenth_gateway` | none; SUPER_CLIENT | outside the supervisor | 50 Hz teleop timer always; 1 Hz status loop even with no phone (`app.py:528-539`) | with a phone: `/slam/map` → PNG | the status loop scans `/proc` and forks `systemctl` every second |
| f1tenth_ctl_monitor | `f1tenth_ctl/rosprobe.py:215-256` | none; SUPER_CLIENT | whenever `f1tenth_ctl status` sees a stack | JSON snapshot 2 Hz | raw subs on `/scan`, two odometries | **the one unconditional `TransformListener`** (`:89-92`); `spin_once` loop at 50 Hz |
| llama-server, llm nodes | `llm` | none | no | – | – | – |

## 3. The two loops in detail

### 3.1 mpc_corr control cycle (RTI, `allow_reverse` false, `speed_command_mode` offset_free)

1. Odom select, TF drain (a `PolledTf` side node, not a listener), speed observer (`MPC:5127-5145`).
2. Obstacle distances, hold / wall-stop / goal branches (`MPC:5147-5480`).
3. `_planning_obstacles` (`MPC:5514`) and `_reverse_env` (`MPC:5704`) each look up map→odom: 2–4 `lookup_transform` per tick.
4. `build_straight_corridor` once per second (`corridor_update_period`), `compute_local_target` every tick (`MPC:5664`).
5. `save_corridor_snapshot` every tick (`MPC:5674`): six `.tolist()`, `json.dumps`, write + flush (`MPC:7774-7813`).
6. `_wall_boundaries` rebuilds the rows and publishes a JSON diagnostic every tick (`MPC:5683, 3966-4040`).
7. One forward QP per tick through `solve_mpc_step` (`MPC:5734`); the rear solver only while stuck (`SP:1672`).
8. Status publishes, CSV write + flush (`MPC:5972-5975`), `/drive`, model-log row + flush (`MPC:6012`).

Inside `_solve_rti` (N = 20, ~145 variables, ~320 rows) **[checked against the source]**:
- `scale_stage_weights` once (`SOLV:432`);
- `_model_step` 4N = 80 times (`SOLV:974, 981, 500, 1547`), the call at 981 recomputes `x_ref[k+1]`;
- `_model_jacobian` N times, `_linearize_obstacle_cost_rti` N times (`SOLV:1178`);
- P and q are dense `np.zeros((n_z, n_z))` (`SOLV:1074-1075`), every constraint row is a fresh `np.zeros(n_z)`;
- dense → CSC conversion every tick (`SOLV:1404-1405`);
- **a new `osqp.OSQP()` + `setup()` every tick**, `polishing=True`, `max_iter=200`, then `warm_start` (`SOLV:1419-1427`); `update()` is never used;
- `planner_cost_corridor` re-rolls the horizon in Python only to report a cost (`SOLV:1470`).

Logging per solve tick: ~16 unthrottled `get_logger().info` plus ~6 throttled ones;
`ODOM/hw` is a throttled log call per odometry message.

Solve time is on `/mpc/solver_status` (`MpcSolverStatus.solve_dt_sec`, `MPC:2240, 5829`).

### 3.2 Behaviour tree

- Tick: `bt_tick_hz` 10.0 (`SP:2637-2638` **[checked]**), one timer; a 1 Hz report timer.
- At rest: the tree still ticks at 10 Hz, publishes `/behavior/tree_status` each tick, logs an ascii snapshot at 1 Hz.
- **PNG visualisation every 0.4 s through a `dot` subprocess on a worker thread** (`BEN:277, 291, 337, 970`), on by default (`SP:3257`), not gated on subscribers.
- Always-on subscriptions: `/scan`, both EKF odometries (20 Hz), `/mpc/*`, `/perception/walls`, `/perception/virtual_wall/debug`, detections, system status.
- Event-driven e-stop: `IsProximityTooClose._scan_callback` per scan (40 Hz), vectorised numpy with cached sin/cos (`behaviours/is_proximity_too_close.py:311-391`).
- No TransformListener; QoS event handlers dropped (`BEN:982-985`).

## 4. Heavy topics

| topic | type, size | publisher, rate | subscribers | QoS | conversions |
|---|---|---|---|---|---|
| `/camera/image_raw` | bgra8 640×360, 0.92 MB | ZED, 30 Hz configured | yolo_detector_node; front_clearance_node; `image_raw_viz_throttle` (always on) | BEST_EFFORT depth 1 (Python nodes) | yolo: view + GPU upload; front_clearance: stores the message |
| `/zed2/zed_node/depth/depth_registered` | 32FC1 640×360, 0.92 MB | ZED, 30 Hz | detection_3d_node; front_clearance_node; `depth_throttle_5hz` (always on) | detection_3d RELIABLE depth 10; front_clearance BEST_EFFORT 1 | `imgmsg_to_cv2` in both |
| `/camera/detection_masks` | mono8, 230 KB | yolo, ≤ 15 Hz | detection_3d; front_clearance; mission bag | RELIABLE 10 / BEST_EFFORT 1 | `imgmsg_to_cv2`, `astype(float32)` |
| `/camera/image_annotated` | bgr8, 691 KB | yolo, ≤ 15 Hz while subscribed | `image_annotated_viz_throttle` (always on → the overlay is always drawn); viz relay (lazy) | depth 10 | mask download + overlay in yolo |
| `/perception/scene_labels` | mono8 160×90 | front_clearance, 5 Hz | detection_3d; yolo (debug) | BEST_EFFORT 1 | `imgmsg_to_cv2` |
| `/perception/scene_labels/classes`, `/depth_mm` | mono8 230 KB, mono16 460 KB | front_clearance, 5 Hz | lidar_front_wall_node (always, `wall_rect_verify` true) | sensor QoS | views |
| `/perception/wall_points` | PointCloud2, every 4th point | front_clearance, 5 Hz | lidar_front_wall_node; mission bag | sensor QoS | `create_cloud_xyz32` |
| `/slam/map` | OccupancyGrid (173×295 in the 2026-10-23 bag) | slam_toolbox, every 5 s | costmap_boundary, costmap_renderer, freespace_door, mpc_corr (rear costmap) | RELIABLE, TRANSIENT_LOCAL, depth 1 | full int8 → int16 copies per tick |
| `/costmap/wall_cells`, `/wall_cell_ids` | OccupancyGrid 240×240; Int32MultiArray 230 KB | semantic_layer, 5 Hz | logger / none | depth 1 | `astype` + `tobytes` |

ZED configuration (`src/f1tenth_perception/config/zed2_perception.yaml`): `grab_frame_rate`
30 (`:18`), `pub_resolution` CUSTOM with `pub_downscale_factor` 2.0 (`:23-24`),
`pub_frame_rate` 30 (`:25`), depth `PERFORMANCE`. `grab_resolution` is not overridden; the
pinned wrapper (86171bc, read in another checkout of the same commit) defaults zed2i to
HD720 (`zed_wrapper/config/zed2i.yaml:10`), so the published size is **640×360**, which is
what the 2026-10-23 bag carries **[checked]**.

## 5. Regression checks

| check | result | where |
|---|---|---|
| ZED `publish_imu_tf` false at launch | **present, effective** | YAML `zed2_perception.yaml:93`; launch argument `PL/camera.launch.py:146` `'publish_imu_tf': 'false'`. In the pinned wrapper (86171bc) `zed_camera.launch.py:563` declares the argument, `:443` maps it to `sensors.publish_imu_tf` after the YAML, and `zed_camera_component_main.cpp:4800` gates the broadcast on it. The 2026-10-23 car bag has no `zed2_imu_link` edge on `/tf` (Humble wrapper on the car; the Jazzy one was not run here) **[checked]** |
| both EKFs at 20 Hz | **present, by launch override** | the YAMLs still say `frequency: 50.0` (`ekf.yaml:30`, `ekf_global.yaml:86`, both commented "OVERRIDDEN"); `ekf_frequency_hz` 20.0 (`SP:2619-2620`) is applied in `ekf.launch.py:99-103` and `ekf_global.launch.py:167-170`. The bag shows 19.9 Hz on both filtered odometries **[checked]** |
| `ekf_cost_observer` removed | **absent** | only `src/f1tenth_localization/test/test_ekf_global_config.py` mentions it (asserting removal) **[checked]** |
| `wall_distance_node` removed | **present, off by default** | entry point `src/f1tenth_perception/setup.py:51`, `PL/wall_distance.launch.py`, component `components.yaml`, `enable_wall_distance: false` (`SP:1096-1097`) **[checked]** |
| BT tick rate and idle load | 10 Hz; idle load not zero | see 3.2: tick + status publish at 10 Hz, PNG render every 0.4 s, 40 Hz scan callback |
| `costmap_boundary` ≤ 10 Hz | **10 Hz** | `costmap_boundary_extraction_rate_hz` 10.0, a launch default in `CL:151`, not in stack_params |
| `semantic_layer` ≤ 10 Hz | **10 / 10 / 5 Hz** | cycle `SP:3586`, memory `SP:3679`, wall layer `CL:164` |
| `reset_manager` ≤ 10 Hz | **event-driven** | the 20 Hz hold timer and the watch subscriptions exist only during a `/reset_all` (`reset_manager_node.py:309-344`) |
| RMW | `rmw_fastrtps_cpp` | exported in `scripts/env/jazzy.sh:47` and `scripts/sim/sim_isaac.sh:202`; launch files inherit it |
| discovery | Fast DDS Discovery Server, 127.0.0.1:11811 | `src/f1tenth_bringup/scripts/ensure_discovery_server.py:119`, `SP:3086-3090`; 5 s lease (`config/fastdds_profile.xml:45-46`). SUPER_CLIENT only for the foxglove tree, the viz relays, the mission logger tree, the gateway and the ctl probes. No SHM / socket-buffer tuning on the car profile |

## 6. Static findings, most likely CPU cost first

1. **OSQP rebuilt from dense matrices on every MPC tick** (`SOLV:1074-1075, 1404-1405, 1419-1427`). Measured in level A.
2. **front_clearance_node works per depth frame (30 Hz), unpinned**, and decodes two more full-rate streams it only stores (`P/front_clearance_node.py:858-860, 1720-1850`).
3. **Always-on consumers of full-rate images with no reader**: `image_raw_viz_throttle`, `image_annotated_viz_throttle` (`BL/foxglove_bridge.launch.py:190-214`) and `depth_throttle_5hz` (`mission_logger.launch.py:104-115`). The annotated one also forces yolo to draw and publish the overlay on every frame.
4. **The mission logger is on by default** (`SP:2727`) while its own description, `mission_logger.launch.py:12-13` and `SUP:1178` say off.
5. **hard_stop_node: per-beam Python loop at 40 Hz** plus a JSON publish per scan (`hard_stop.py:95-100`).
6. **lidar_front_wall_node publishes two messages at 40 Hz** that cost mpc_corr 40 callbacks/s, and rebuilds the per-beam trig table on every scan (`P/scene_labels.py:157-171`). Measured in level A.
7. **mpc_corr per-tick logging and file I/O**: ~22 logger calls, three flushed files, a JSON corridor snapshot (`MPC:5674, 5972-5975, 6012, 7812`). Not covered by level A (node glue).
8. **detection_3d_node deserialises all 30 Hz depth through a RELIABLE depth-10 synchroniser** and does a full-frame boolean op per detection (`P/detection_3d_node.py:304-318, 747`).
9. **costmap_boundary_node: four full-map copies per tick, no change detection** (`C/costmap_boundary.py:130, 227`). Measured in level A.
10. **BT PNG render every 0.4 s with no subscriber gate** (`BEN:291, 337, 970`).
11. **Two `TransformListener`s for static edges**: freespace_door_node (`C/freespace_door_node.py:236`) and f1t-monitor (`rosprobe.py:89-92`); ~0.35 ms per `/tf` message each.
12. **foxglove_bridge idles at about half of core 3** by its own docstring, shared with the relay container and viz_jpeg_node.
13. **Core 5 is shared** by lidar_front_wall_node and obstacle_clearance_node; the four costmap nodes, front_clearance_node, hard_stop_node and everything in diagnostics/logger are unpinned.
14. **semantic_layer_node builds a 230 KB wall layer at 5 Hz for no live consumer**; costmap_renderer renders at 2 Hz with no subscriber gate.
15. mpc_corr deserialises `/scan` at 40 Hz for a wall_turn-only use (`MPC:2117, 4236`).

Not read: vesc driver, ackermann_mux, slam_toolbox, robot_localization, py_trees_ros and
topic_tools internals (whether `topic_tools throttle` is lazy by default decides
finding 3 and should be confirmed with `ros2 topic info -v` on a running stack).
