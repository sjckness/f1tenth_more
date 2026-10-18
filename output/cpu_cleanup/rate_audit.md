# Rate audit (Blocks 2a–2d): branch `cpu-cleanup` (HEAD f66cfc5, base 9c46add)

How this was done: static reading only. Nothing was built, launched or edited.
- Repo paths are relative to `/home/andreas/dev_ws/f1tenth_more_cpu_cleanup`.
- `H:` paths are relative to `/tmp/claude-1000/-home-andreas-dev-ws/0f360719-b5a9-43e4-a45a-243442f30708/scratchpad/humble_src`.
- **UNVERIFIED** marks any claim the repo or the Humble copies cannot settle. The robot_localization, vesc_driver, vesc_ackermann, urg_node, slam_toolbox, ackermann_mux and ZED wrapper sources are not in the repo (`src/f1tenth_hardware/vesc` is an empty directory), and they are not in `H:` either.

Policy under test:
1. sensors run at their native rate;
2. the lidar e-stop runs at `/scan` rate;
3. anything that feeds the MPC (10 Hz, `self.ts = 0.1` hardcoded at `src/f1tenth_control/mpc_controller/mpc_controller/MPC_corr.py:740`, timer at `:1918`) or the BT (10 Hz, `bt_tick_hz` at `src/f1tenth_params/config/stack_params.yaml:2084`) runs at ≤ 20 Hz.

---

## 0. Headline findings

1. **Two EKF outputs at 50 Hz are the largest > 20 Hz streams that feed only 10 Hz consumers.**
   - Both `frequency: 50.0`: `src/f1tenth_bringup/config/ekf.yaml:30` and `ekf_global.yaml:86`.
   - Each also drives `/tf` at 50 Hz (`publish_tf: true`, `ekf.yaml:33`, `ekf_global.yaml:99`).
   - None of their consumers needs more than 20 Hz.
   - One gate is right at 2× a 20 Hz period: `PoseHistory.at(max_extrapolation_s=0.1)` in `src/f1tenth_perception/f1tenth_perception/multi_wall_tracker.py:170`, used by the lidar_front_wall tracker at `lidar_front_wall_node.py:1254`. It needs raising together with the EKF change, otherwise the virtual wall can go to NO_DATA (-1). Details in §2.
   - `transform_time_offset` must go 0.05 → 0.08 together with the EKF change. That decision is already logged and pending (`docs/history/LOG.md:22-62`).
2. **`/perception/virtual_wall` is published on every processed scan (≤ 40 Hz), but its value only changes on tracker steps (1 scan in 4, nominally 10 Hz).**
   - The BT and MPC_corr each receive about 3 duplicate messages per real estimate (§3).
   - A ≤ 20 Hz publish rule ("on every tracker step, plus a heartbeat at most every 50 ms") loses nothing and keeps every gate.
3. **MPC_corr runs a full Python `TransformListener`**, `MPC_corr.py:1613-1614.
   - It deserialises every `/tf` message (2 EKFs × 50 Hz + robot_state_publisher) to read one edge, map→odom, at goal-anchor time.
   - detection_3d_node (`:282`) and obstacle_projector_node (`:188`) do the same. They are ZED-pipeline nodes, but they are launched whenever `camera_source == zed`, including when the ZED is off. They only ever look up static edges.
4. **MPC_corr calls throttled loggers on every message**, which the BT audit measured at 0.24 ms per call:
   - `hw_odom_callback` at 50 Hz (`MPC_corr.py:2006-2010`);
   - `front_distance_callback` at virtual_wall rate (`:3296-3300`).
5. **Three > 20 Hz publishers have no consumer in the live stack** but are auto-started (`component_supervisor_node.py:325-328`):
   - swept_clearance_node: `/perception/swept_clearance*`, 20 Hz plus 40 Hz plus depth rate;
   - obstacle_clearance_node: `/obstacle_clearance` at 40 Hz, read only by the manually started test-campaign logger;
   - front_clearance_node's `/perception/front_clearance`, `/front_blocked` and `/front_wall`, at depth rate (ZED-on).
6. **Safety-path note, not a rate issue.** `IsProximityTooClose` has no `/scan` staleness gate (`is_proximity_too_close.py:371-387`). A dead urg_node leaves the last, untripped value standing, so the e-stop fails open. Nothing in this change list touches it, but it was asked for under "every gate".

---

## 1. (2a) Every topic published at > 20 Hz, or per message of a > 20 Hz input

Rates are the configured or nominal ones, with the source cited. In the Z column, **off** means present with the ZED off, and **ZED** means it only carries data with the ZED on. The ZED nodes are launched whenever `camera_source == zed` (`detection.launch.py:374, 438-510`), so their subscriptions exist even with the camera off.

### 1.1 Sensors and drivers

| topic | type | publisher, rate source | Z | subscribers (need) and gates |
|---|---|---|---|---|
| `/scan` | LaserScan | urg_node (`lidar.launch.py:53`, `sensors.yaml`), native **40 Hz** (UST-10LX; `stack_params.yaml:2925`, `sensors.yaml:26`) | off | see the `/scan` rows below |
| ↳ | | | | **BT IsProximityTooClose**: safety. RELIABLE depth 10, `is_proximity_too_close.py:270-271`. Event stop published inside the callback, `:348-351`. **No staleness gate**, `:371-387` |
| ↳ | | | | **lidar_front_wall_node**: perception for the BT/MPC wall. LATEST_FRAME_QOS = BEST_EFFORT depth 1 (`latest_frame_qos.py:21-22`, `lidar_front_wall_node.py:656-657`). Tracker runs 1 scan in 4 (`:554`, `stack_params.yaml:2924`) |
| ↳ | | | | **slam_toolbox**: sensor input. `slam_toolbox_params.yaml:61`; `throttle_scans 1` (`:66`); `minimum_time_interval 0.5` (`:72`); `transform_timeout 0.2` (`:73`); `tf_buffer_duration 30` (`:74`); `scan_queue_size 100` (`:143`) |
| ↳ | | | | **swept_clearance_node**: no consumer of its output. sensor QoS, `swept_clearance_node.py:165-168`; `lidar_timeout_sec 0.25` (`:131`) |
| ↳ | | | | **obstacle_clearance_node**: offline/test logger only. `obstacle_clearance_node.py:129`; docstring "nothing in the stack subscribes", `:17-20` |
| ↳ | | | | **front_clearance_node** (ZED): scene-mask inferred floor, latest-only (`front_clearance_node.py:860-861`); `inferred_floor_max_scan_age_s 0.15` (`:643`, `:1349`) |
| ↳ | | | | **MPC_corr**: only if its private wall tracker is on. Off by default with `enable_scene_labels` (`MPC_corr.py:1085-1089, 1772-1775`) |
| ↳ | | | | **wall_distance_node**: off by default (`enable_wall_distance: false`, `stack_params.yaml:1013`) |
| ↳ | | | | **viz relay** throttle to 5.5 Hz: viz (`viz_relays.yaml` /scan entry; `viz_rate_scan`, `stack_params.yaml:2538`) |
| ↳ | | | | **mission logger** rosbag: logging, off by default (`mission_logger_node.py:304`; `enable_mission_logger: false`, `stack_params.yaml:2170`) |
| `/sensors/core` | VescStateStamped | vesc_driver_node, ~**50 Hz** poll (`battery_voltage_check_node.py:5`; rate UNVERIFIED, driver source absent) | off | vesc_to_odom (sensor input, C++, UNVERIFIED). diagnostics_server_node, Python, for a 2 Hz BatteryStatus (`diagnostics_server_node.py:67, 87, 98`). battery_voltage_check: window `sample_window_sec 2.0` in seconds (`:120-121`). gyro_bias / sensor_cov calibration at startup (`gyro_bias_calibration_node.py:115`). viz relay 5.5 Hz |
| `/sensors/imu/raw` | Imu | vesc_driver_node, **50 Hz** (`src/f1tenth_logger/TEST_CAMPAIGN.md:38`) | off | ekf_filter_node `imu0` (`ekf.yaml:90`, queue 10 at `:96`). gyro_bias_calibration at startup (`:109`). test-campaign logger, manual (`test_campaign_logger.yaml:30`). ekf_cost_observer, off (`localization.launch.py:84`) |
| `/odom` | Odometry | vesc_to_odom_node_backup (`vesc.launch.py:211-213`), per `/sensors/core` message, ≈ 50 Hz (UNVERIFIED; `corridor_perception/replay.py:134` says "~10 Hz in these bags") | off | ekf_filter_node `odom0` (`ekf.yaml:56`, queue 10 at `:62`). reset_manager during a reset only (`reset_manager.yaml:43`; subscriptions opened per reset, `reset_manager_node.py:307-313`). MPC/BT only when `localization_source == raw_odom` (`param_defaults.py:84`). mission logger, off |
| `/zed2/zed_node/depth/depth_registered` | Image | ZED wrapper, `pub_frame_rate: 30.0` (`zed2_perception.yaml:17`); ~17 Hz measured (`stack_params.yaml:1341`) | ZED | detection_3d (message_filters, default depth, `detection_3d_node.py:293-303`). front_clearance (latest-only, `front_clearance_node.py:869-870`). swept_clearance (sensor QoS, `swept_clearance_node.py:174`; `camera_timeout_sec 0.5`) |
| `/camera/image_raw` | Image | ZED rgb, remapped (`camera.launch.py:119`), 30 Hz | ZED | yolo_detector (latest-only, `yolo_detector_node.py:452-453`). front_clearance scene mask (latest-only, `front_clearance_node.py:858-859`; internal 5 Hz, `:591`) |

### 1.2 Localisation and TF

| topic | type | publisher, rate source | Z | subscribers (need) and gates |
|---|---|---|---|---|
| `/odometry/filtered` | Odometry | ekf_filter_node, `frequency: 50.0` (`ekf.yaml:30`) | off | see the rows below |
| ↳ | | | | ekf_global `odom0`, differential (`ekf_global.yaml:174, 263`) |
| ↳ | | | | **MPC_corr** (10 Hz control): BEST_EFFORT depth 10 (`MPC_corr.py:1619-1637`); `odom_stale_timeout_sec 0.5` (`:330-331`, `stack_params.yaml:1151`) |
| ↳ | | | | **BT CheckStopCondition** (10 Hz BT): RELIABLE depth 10 (`check_stop_condition.py:289`); topic from `get_odom_topic()` (`behavior_executor_node.py:611`) |
| ↳ | | | | **lidar_front_wall_node** (dead reckoning and tracker pose): sensor QoS (`:607-608`); `odom_max_age_sec 0.2` (`:415`); `PoseHistory.at` 0.1 s extrapolation (`multi_wall_tracker.py:170`) |
| ↳ | | | | reset_manager during a reset only (`reset_manager.yaml:29, 51`) |
| ↳ | | | | mission logger (off). test-campaign logger (manual, `test_campaign_logger.yaml:29`). ekf_cost_observer (off). wall_distance (off). Nav2 (off; `nav2_params.yaml:115`; `enable_nav2: false`, `stack_params.yaml:104`) |
| `/ekf_global/odometry/filtered` | Odometry | ekf_global_filter_node, `frequency: 50.0` (`ekf_global.yaml:86`); namespace (`ekf_global.launch.py:157`) | off | see the rows below |
| ↳ | | | | **BT CheckStopCondition**: global pose and turn accumulation for scoring (`check_stop_condition.py:299-300, 350-372`) |
| ↳ | | | | **costmap_boundary_node** (10 Hz timer): BEST_EFFORT depth 10, latest only (`costmap_boundary_node.py:344, 512-513, 551-555`); no staleness gate (`:367-378`) |
| ↳ | | | | **semantic_layer_node**: pose history of 200 samples (`semantic_layer_node.py:133, 183, 294-295, 423-428`) |
| ↳ | | | | viz relay at 5.5 Hz (`viz_relays.yaml`). reset_manager during a reset (`reset_manager.yaml:64`). mission logger (off) |
| `/tf` | TFMessage | odom→base_link from ekf_filter_node at 50 Hz (`ekf.yaml:33`, post-dated 0.05 at `:35`). map→odom from ekf_global at 50 Hz (`ekf_global.yaml:99, 101`). Wheel/steer joints from robot_state_publisher at the `/joint_states` rate (joint_state_publisher default 10 Hz, UNVERIFIED; `description.launch.py:116-131`). ZED TF off (`zed2_perception.yaml` pos_tracking `publish_tf: false`). slam_toolbox publishes no TF (`transform_publish_period: 0.0`, `slam_toolbox_params.yaml:67`). Total ≈ 110 msg/s | off | see the rows below |
| ↳ | | | | **MPC_corr**, full Python listener (`MPC_corr.py:1613-1614`). Uses map→odom only, with `Time()` = latest (`:2152-2153`) and the age guard `map_odom_max_age_sec 0.2` (`:397-398`) |
| ↳ | | | | **detection_3d_node** (`:282`) and **obstacle_projector_node** (`:188`), full Python listeners, static edges only with `Time()` (`detection_3d_node.py:595-602`, `obstacle_projector_node.py:211-212`). Launched with ZED off too |
| ↳ | | | | slam_toolbox (C++, MessageFilter on the scan stamp). foxglove_bridge (whitelist `^/tf$`, `stack_params.yaml:2488`). mission logger (off). costmap_boundary only with the polytope (off, `costmap_boundary_node.py:505-508`). semantic_layer only with `tf_static_only:=false` (`semantic_layer_node.py:269-279`) |

### 1.3 Perception: lidar_front_wall_node and its siblings (`enable_scene_labels` true, `use_wall_tracker` true: `stack_params.yaml:747, 752`)

| topic | type | rate | Z | subscribers (need) and gates |
|---|---|---|---|---|
| `/perception/virtual_wall` | Float32 | **every processed scan, ≤ 40 Hz** (`lidar_front_wall_node.py:1056-1080`, `:1172`). Value refreshed only on tracker steps (§3). ~6 Hz measured live on 2026-10-14 when CPU-bound (`stack_params.yaml:758`) | off | see the rows below |
| ↳ | | | | **BT CheckStopCondition**: `wall_distance_max_age_s 0.5`, receipt time (`check_stop_condition.py:379-418`; `behavior_executor_node.py:758-759`; `stack_params.yaml:756`); `debounce_ticks` 3, in BT ticks (`check_stop_condition.py:598`) |
| ↳ | | | | **MPC_corr**: wall_turn distance cap; `corr_wall_turn_front_distance_max_age_sec 0.5` (`MPC_corr.py:1010-1013, 1732-1737, 3005-3040, 4423`; `stack_params.yaml:1339`) |
| ↳ | | | | preflight: presence plus a first message (`preflight.py:141-163`). mission logger (off, `mission_logger_node.py:276`). Offline tools (`tools/replay_raw.launch.py:68`, `tools/front_phantom_replay.py:111`, `tools/corridor_wall_replay.py:143`) |
| `/perception/virtual_wall/debug` | String (JSON) | every processed scan (`:1176`) | off | yolo_detector debug image, latest at 5 Hz (`yolo_detector_node.py:1004-1008`). mission logger (off, `:277`). Offline tools |
| `/perception/lidar_front_wall` | WallLineFit | every scan (`:603, 1495`) | off | front_clearance_node, debug JSON only (`front_clearance_node.py:799-804`; `publish_debug_wall` default true, `stack_params.yaml:536`). `tools/corridor_wall_replay.py:144` |
| `/perception/lidar_front_wall_virtual` | WallEstimate | every scan (`:604, 1511`) | off | viz relay at 5.5 Hz only (`viz_relays.yaml`) |
| `/perception/wall_observations` | WallObservationArray | tracker steps, ~10 Hz (`:1271-1272` via `_publish_observations`) | off | mission logger (off, `:303`); offline tools. Under 20 Hz |
| `/perception/walls`, `/walls/markers` | MultiWallTrackArray, MarkerArray | tracker steps, ~10 Hz (`:1477-1478`) | off | semantic_layer, latest-only, 5 Hz raster (`semantic_layer_node.py:309, 334-336`). yolo debug (`yolo_detector_node.py:1013-1014`). mission logger (off). Under 20 Hz |
| `/perception/wall_planes` | String | `wall_planes_rate_hz 10` (`:427`, `stack_params.yaml:774`) | off | front_clearance (`front_clearance_node.py:827-828`). Under 20 Hz |

### 1.4 Other > 20 Hz perception outputs

| topic | type | rate | Z | subscribers (need) and gates |
|---|---|---|---|---|
| `/perception/swept_clearance`, `/steering` | Float32 | timer `publish_rate_hz 20` (`swept_clearance_node.py:133, 177`; `stack_params.yaml:852`) | off | **none** ("NOTHING SUBSCRIBES TO THIS YET", `swept_clearance_node.py:18-20`; grep confirms) |
| `/perception/swept_clearance/lidar` | Float32 | per scan, 40 Hz (`:154`) | off | **none** |
| `/perception/swept_clearance/camera` | Float32 | per depth frame (`:155`) | ZED | **none** |
| `/obstacle_clearance` | Float32 | per scan, 40 Hz (`obstacle_clearance_node.py:3-9, 123`) | off | test-campaign logger only, started by hand (`logger_node.py:203-204`; `test_campaign_logger.yaml:37`) |
| `/perception/front_distance` | Float32 | per depth frame, ~17 Hz (`front_clearance_node.py:778-779`) | ZED | viz relay only (`viz_relays.yaml`). Retired as a wall source (`param_defaults.py:95-102`). Not read by the BT or MPC any more |
| `/perception/front_clearance`, `/front_blocked`, `/front_wall` | Float32/Bool | per depth frame (`:780-785`) | ZED | viz relay for `front_clearance` only. **None** for `front_blocked` and `front_wall` |
| `/perception/debug/front_wall` | String | per depth frame | ZED | yolo debug (`yolo_detector_node.py:1036-1037`); mission logger (off) |
| `/camera/detections`, `/detections_3d`, `/perception/obstacles_2d` | Detection2D/3D, Obstacle2DArray | per YOLO frame, ~15 Hz callback (`yolo_detector_node.py:450`); 3D ~5-8 Hz (`stack_params.yaml:765`) | ZED | BT DetectedClassesBridge (`behavior_executor_node.py:815`, `detected_classes_bridge.py:33`). MPC obstacles, **no staleness gate** (`MPC_corr.py:3171-3184`). lidar_front_wall cut (`detection_max_age_sec 0.5`, `:423`; tracker 1.0, `:441`). Under 20 Hz |

### 1.5 Control path (all ≤ 20 Hz)

| topic | type | rate | Z | subscribers (need) and gates |
|---|---|---|---|---|
| `/mpc/*`, `/drive` | various | 10 Hz control loop (`MPC_corr.py:3544, 3552, 3559, 3505, 4061`) | off | BT; ackermann_mux (`mux.yaml:12-13`, `timeout 0.2`). Already ≤ 20 Hz |
| `/safety_stop` | AckermannDriveStamped | BT Stop per tick (10 Hz, `behavior_executor_node.py:538`). **Plus the event stop on every scan while tripped (≤ 40 Hz, `is_proximity_too_close.py:348-351`)**. reset_manager 20 Hz while holding (`reset_manager.yaml:27`) | off | ackermann_mux priority 200, `timeout 0.2` (`mux.yaml:8-10`). Safety: the 40 Hz is intentional |
| `/ackermann_drive` | AckermannDriveStamped | ackermann_mux output, per accepted input (UNVERIFIED, mux source absent) | off | ackermann_to_vesc (`vesc.launch.py:339`). swept_clearance steering history (`swept_clearance_node.py:110, 160`). mission logger (off) |

### 1.6 Diagnostics

| topic | rate | notes |
|---|---|---|
| `/diagnostics*` | ≤ 2 Hz | BT tick timing at 1 Hz (report.md §1). battery status at 2 Hz (`battery_check_rate_hz`, `stack_params.yaml:2284`). system_status at 1 Hz (`:2280`). Nothing here is > 20 Hz |

**Not present at all:**
- `/imu`: subscribed by MPC_corr (`MPC_corr.py:1777-1782`). No publisher in the repo (ZED IMU off, `zed2_perception.yaml` sensors), so this is UNVERIFIED for the external vesc driver.
- `/model/virtual_robot/odometry`: sim only (`MPC_corr.py:1639-1644`).

---

## 2. (2b) The EKFs

### 2.1 Configuration

| param | local (`ekf.yaml`) | global (`ekf_global.yaml`) | rate implication |
|---|---|---|---|
| `frequency` | 50.0 (`:30`) | 50.0 (`:86`) | Output Odometry and TF rate. Inputs keep their own rates: local odom0 and imu0 at ~50 Hz each, so a 20 Hz filter still fuses ~100 measurements/s; only predict and publish drop (UNVERIFIED, robot_localization source absent) |
| `sensor_timeout` | 0.1 (`:31`) | 0.1 (`:92-97`) | Local: inputs stay 50 Hz, no change. **Global: odom0 is `/odometry/filtered`. At 20 Hz this timeout equals 2 input periods**, so a late local message produces a predict-only cycle. That is harmless by design (`:93-97`), but it is the one EKF parameter at exactly 2× the 20 Hz period |
| `two_d_mode` | true (`:32`) | true (`:98`) | rate-independent |
| `transform_time_offset` | 0.05 (`:35-39`) | 0.05 (`:101-105`) | **Must change with the rate.** At 50 Hz, scans already lead the newest TF for 0.3-4.9 % (odom→base_link) and 3.2-50 % (map→odom) of scans. Decision recorded: 0.08 on both edges, to land with the 20 Hz change (`docs/history/LOG.md:22-62`). It is still 0.05 at HEAD |
| `history_length`, `smooth_lagged_data`, `predict_to_current_time` | not set | not set | Defaults apply (UNVERIFIED: history 0, no smoothing). **No parameter is in units of filter updates** |
| `*_queue_size` | odom0 10 (`:62`), imu0 10 (`:96`) | odom0 10 (`:191`), pose0 10 (`:276`) | Counted in **messages**. Global odom0 at 20 Hz input is a 0.5 s buffer, still ample |
| `process_noise_covariance` | — | — | A **per-second** rate (`ekf_global.yaml:322-323`: `P = JPJ' + delta_sec*Q`), so it is frequency-independent. The "Kalman gain 0.375 inside a single 50 Hz tick" text (`:331-337`) describes P grown over the pose0 interval (~0.5 s), not over one tick. The gain is unchanged at 20 Hz (UNVERIFIED by replay); the comment wording becomes stale |
| `pose0_rejection_threshold` | — | 5.0 (`:312`) | rate-independent |
| "Failed to meet update rate" | | | Judged against 1/frequency (`localization.launch.py:55-60`), so it gets laxer at 20 Hz. `scripts/check_ekf_update_rate.py:44-47, 91-92, 104` hardcodes `_TARGET_PERIOD_MS = 20.0` / 50 Hz and **must be updated** |

### 2.2 Consumers: is 20 Hz fine?

| consumer | what it does with odometry / TF | gates (≤ 0.1 s flagged) | verdict |
|---|---|---|---|
| **MPC_corr** | Latest pose per 10 Hz tick (`_update_active_odom`, `MPC_corr.py:3133-3170`). Turn progress unwrapped **per control tick**, not per message (`:2952-2982`, called `:3594`). map→odom via TF, `Time()` latest (`:2152-2153`) | `odom_stale_timeout_sec 0.5` (`:330-331`; also boundaries `:3248` and scan `:3388`). `map_odom_max_age_sec 0.2`, one-sided, so post-dating cannot trip it (`:397-398`, `:2161-2185`) | **OK.** The state handed to the solver is on average ~15 ms older (worst +30 ms): ≤ 1.5 cm at 0.5 m/s. Per-message cost halves, and the throttled logger call per message (`:2006-2010`) drops 50 → 20/s |
| **ekf_global** | odom0 differential: velocity from deltas between consecutive messages (`ekf_global.yaml:207-221`) | `sensor_timeout 0.1` | **OK**, see 2.1 |
| **reset_manager** | Standstill: every fresh sample ≤ 0.05 m/s for `standstill_hold_s 0.5` **seconds** (`reset_manager_node.py:355-377`; `reset_manager.yaml:30-32`). Per-sample wait 0.2 s (`:360`); fresh = `now - seen > 0.5` (`:374`). verify ±tolerance within `verify_timeout_s 5.0` (`reset_manager.yaml:21`; `:173`) | none < 0.1 s; all time-based | **OK** (10 samples per hold window at 20 Hz) |
| **steering_offset_calibration** | Uses **/slam/pose only**, by design (`steering_offset_calibration_node.py:18-35`). `/costmap/front_clearance` gate `clearance_stale_sec 1.0` (`:327-328, 1010-1011`). Sample counts are per /slam/pose segment (`min_pose_samples_per_segment 4`, `:357-358`). Manual tool, not in the supervisor (`:11-14`) | — | **Unaffected** |
| **BT CheckStopCondition** | Turn accumulation **per message** for local and global (`check_stop_condition.py:331-348, 350-372`). Unwrap is valid while the step is < π: at 20 Hz and ~1.7 rad/s (R ≈ 0.6 m at 1 m/s) the step is ~0.085 rad. Scoring uses end global xy/yaw (`_record_and_summarize`, `:427`). `orientation_delta` is evaluated per tick (`condition_eval.py:408-420`) | `wall_distance_max_age_s` applies to the wall topic only | **OK.** Stop latency on a turn or distance grows by ≤ 30 ms (≤ 3° at 1.7 rad/s, ≤ 1.5 cm at 0.5 m/s), small next to the 100 ms tick |
| **semantic_layer** | `_POSE_HISTORY_MAXLEN = 200` **samples** (`semantic_layer_node.py:133`), nearest-stamp lookup with no max gap (`semantic_layer.py:444-455`). 200 samples is 4 s at 50 Hz and 10 s at 20 Hz | none | **OK.** Nearest-pose error ≤ 25 ms × v (≤ 1.3 cm at 0.5 m/s). Note: the window is sized in samples, so at 20 Hz it covers 10 s, which is fine |
| **costmap_boundary** | Latest pose only; 10 Hz timer (`costmap_boundary_node.py:533, 551-555`) | none (gates removed, `:367-378`) | **OK**; halves its wasted deserialisation |
| **lidar_front_wall_node** | (a) WallEstimator dead reckoning from the latest pose, receipt-aged (`lidar_front_wall.py:299-310`). (b) `PoseHistory` (2 s by time, `multi_wall_tracker.py:158-168`) interpolated at the scan stamp for every tracker step (`lidar_front_wall_node.py:1254`), at detection stamps (`:1023`, `:1331`) and at image stamps (`:941-942`) | (a) `odom_max_age_sec 0.2` (`:415`; `stack_params.yaml:725`): 4 periods at 20 Hz. The longest recorded gap was 136 ms even at 50 Hz (`:727`), so margin shrinks. **(b) `max_extrapolation_s=0.1` (`multi_wall_tracker.py:170, 184`): exactly 2 × 50 ms.** | **AT RISK.** The scan stamp leads the newest odometry stamp by ~13-14 ms at 50 Hz (`LOG.md:37-39`). At 20 Hz that grows by up to +30 ms, plus EKF jitter (global p99 period 36-55 ms against a nominal 20 ms, `LOG.md:57-59`). When `at()` returns None the tracker step sets `_tracker_has_data=False` (`:1254-1262`), and `/perception/virtual_wall` publishes **-1 (NO_DATA)**. The BT then holds the car (`check_stop_condition.py:399-406`) and the MPC drops the wall_turn cap (`MPC_corr.py:3036-3037`). **Raise to ≥ 0.15 s (param) with the EKF change**, and `odom_max_age_sec` to 0.3 |
| **wall_tracker in MPC** | Off with scene labels (`MPC_corr.py:1085-1089`) | `odom_stale_timeout_sec` on scan (`:3388`) | n/a |
| **mission logger** | rosbag, off by default (`stack_params.yaml:2170`) | — | **OK** (records fewer messages) |
| **test-campaign logger** | Per-message pose log (`logger_node.py:198, 255, 779-794`), manual | — | **OK**: a 20 Hz pose trace |
| **slam_toolbox** | odom→laser at the scan stamp via MessageFilter | `transform_timeout 0.2` (`slam_toolbox_params.yaml:73`); `tf_buffer_duration 30` (`:74`) | **OK, with the offset going to 0.08.** A scan that leads TF waits one publish period (`LOG.md:40-44`); at 20 Hz that is ≤ 50 ms of extra latency on a 0.5 s-gated SLAM |
| **Python tf2 users** | All lookups use `Time()` = latest, never a stamped lookup. Buffer default cache, UNVERIFIED 10 s (`H:tf2_ros/buffer.py:66-80`) | — | **OK**, no extrapolation exposure |
| **Nav2** | off (`stack_params.yaml:104`); `odom_topic /odometry/filtered` (`nav2_params.yaml:115`) | — | n/a |
| **ekf_cost_observer, wall_distance** | off by default (`localization.launch.py:83-88`; `stack_params.yaml:1013`). wall_distance `odom_max_age_sec 0.2` (`stack_params.yaml:1112`) | — | n/a while off |

**Debounces counted in samples on odometry:** none found. Every odometry consumer gates by time:
- BT `debounce_ticks` counts BT ticks, not odom samples (`condition_eval.py:180-220`);
- semantic_layer's history is sized in samples but is a lookup window, not a debounce.

---

## 3. (2c) `/perception/virtual_wall`

### What is published, and when
- **Subscription:** the node subscribes `/scan` best-effort with depth 1 (`lidar_front_wall_node.py:648-657`). Under load it drops scans rather than queueing them.
- **On every processed scan**, `_scan_cb` (`:1056-1080`) runs:
  - the scene fit (`:1082-1146`);
  - `estimator.step`;
  - `_publish_virtual` (`:1148-1176`): Float32 plus the JSON debug;
  - `_publish` (`:1480-1511`): WallLineFit plus WallEstimate.
- **Tracker decimation:** `_tracker_due()` counts scans and runs the tracker on scan_count % N == 0, with N = `wall_tracker_scan_decimation` = 4 (`:554`, `:1230-1243`; `stack_params.yaml:2924-2926`).
  - On skipped scans the last verdict "STANDS" (`:1121-1130`).
- **What a "new estimate" means.** With the tracker on, the published value is computed only from `_front_d_ray`, `_tracker_has_data` and `_front_coast_expired` (`:1158-1163`). These three change **only inside `_track_walls`** (`:1254-1263, 1298-1309`).
  - The per-scan refit (`:1132-1146`) feeds only the WallLineFit/WallEstimate pair and the debug JSON, not the Float32.
  - So the Float32 carries a new value once per tracker step: nominally 10 Hz at 40 Hz scans. It is neither dead-reckoned nor refreshed in between.
  - About 3 of every 4 messages repeat the previous value. The receipt time still advances, which is what keeps the consumers' age gates happy.
- **Live rate:** "~6 Hz with up to 0.8 s per scan" on 2026-10-14 (`stack_params.yaml:758`); ~4 observations/s when CPU-bound (`:2925`).
  - Coast limit: `wall_tracker_front_max_coast_sec 0.3` (`:451`; `stack_params.yaml:759`), which is about 3 tracker steps at 10 Hz.

### Consumers and gates

| consumer | gate | at 20 Hz publish / 10 Hz new value |
|---|---|---|
| BT CheckStopCondition | `wall_distance_max_age_s 0.5`, receipt (`check_stop_condition.py:394-397`; value read `:392`; hold `:399-406`; release `:408-417`) | 0.5 s = 10 publish periods, 5 tracker steps. OK |
| BT front_clearance stop | `debounce_ticks` 3 in `drive_stop_2m_from_wall.json:8`, `drive_straight_to_wall_3m.json:8`, `wall_turn_then_straight.json:8, 22`, and the LLM's `WALL_DEBOUNCE_TICKS` (`plan_translate.py:208`); counted in 10 Hz BT ticks (`check_stop_condition.py:598`; `stack_params.yaml:2081-2086`) | Unchanged by the publish rate. Note that today 3 ticks span ~3 tracker steps, so it is effectively ~3 independent estimates at nominal rate and fewer when CPU-bound |
| BT condition_eval | `front_clearance` evaluated per tick from the blackboard (`condition_eval.py:326-336`) | unchanged |
| MPC_corr wall_turn | `corr_wall_turn_front_distance_max_age_sec 0.5`, receipt (`MPC_corr.py:3005-3040`; used `:4423`); ≥ `wall_clear_distance_m` → None (`:3038-3039`) | OK |
| preflight | presence plus a first message (`preflight.py:141-163`) | OK |
| mission logger | records (`mission_logger_node.py:276`), off by default | fewer duplicates |
| yolo debug | `/virtual_wall/debug`, latest, 5 Hz (`yolo_detector_node.py:1004-1008`) | OK |

**Verdict.** Publishing at most at 20 Hz loses no information, because new values arrive at ≤ 10 Hz. The rule: always on a tracker step, otherwise a heartbeat on a skipped scan only if ≥ 50 ms have passed since the last publish.
- **Gap risk.** Publishing on tracker steps alone would stretch the worst receipt gap to (N scans + tracker time), e.g. 100 + 155 ms (`stack_params.yaml:2914` measured 155-178 ms a step). The heartbeat keeps the gap near today's.

---

## 4. (2d) Other > 20 Hz inputs consumed only by 10 Hz control or the BT

### 4.1 What behavior_executor_node deserialises today, and after the EKF change

| topic | rate now | after EKF 20 Hz | needed by the BT |
|---|---|---|---|
| `/scan` | 40 | 40 | **yes** (e-stop, safety) |
| `/odometry/filtered` | 50 | 20 | ≤ 20 OK |
| `/ekf_global/odometry/filtered` | 50 | 20 | ≤ 20 OK |
| `/perception/virtual_wall` | ≤ 40 | ≤ 40 (**unchanged unless §3 is applied**) | 10 Hz of information |
| `/mpc/*` (5 topics) | 10 | 10 | — |
| `/costmap/semantic_tracks`, `/camera/detections` | ZED, ≤ 15 | same | — |

So after EKF → 20 Hz the BT no longer receives any > 20 Hz odometry. Its only > 20 Hz inputs would be `/scan`, which is correct, and `/perception/virtual_wall`.

### 4.2 Humble APIs for consumer-side decimation (if a publisher cannot be changed)
- `Node.create_subscription(..., raw: bool = False)` exists in Humble rclpy 3.3.21 (`H:rclpy/node.py:1322-1333`, docs `:1348-1349`).
  - It is passed into `Subscription` (`H:rclpy/subscription.py:39, 57, 68`) and honoured at take time: `sub.handle.take_message(sub.msg_type, sub.raw)` (`H:rclpy/executors.py:398-402`).
- `rclpy.serialization.deserialize_message(serialized_message, message_type)` exists (`H:rclpy/serialization.py:32-42`).
- **Caveat:** raw only skips Python message construction. Every message still wakes the executor and passes through `_take_subscription`. The cleanup report found per-wake-up waitable overhead to be the dominant BT cost. Lowering the publisher rate (EKF frequency, virtual_wall throttle) is strictly better than raw-plus-decimate in the consumer.
- Humble `TransformListener` has no static-only option. It always subscribes `/tf` (`H:tf2_ros/transform_listener.py:50-88`, `:85-88`). Static-only means the hand-rolled `/tf_static` subscription pattern already used in `semantic_layer_node.py:269-279` and `lidar_front_wall_node.py:605-606`.

### 4.3 Candidates

| # | candidate | consumers (need) | change | gain / risk |
|---|---|---|---|---|
| C1 | `/odometry/filtered` 50 → 20 | §2.2 | `ekf.yaml:30` → 20.0, behind the existing yaml | Halves the work of 3 Python subscribers (MPC, BT, lidar_front_wall) and of `/tf`. Risk: §2.2 PoseHistory |
| C2 | `/ekf_global/odometry/filtered` 50 → 20 | BT, costmap_boundary, semantic_layer, viz | `ekf_global.yaml:86` → 20.0 | Same. The global p99 jitter is already 36-55 ms (`LOG.md:57-59`) |
| C3 | `/perception/virtual_wall` (+ `/debug`) ≤ 20 Hz | BT, MPC (10 Hz) | new param, e.g. `lidar_front_wall_virtual_wall_max_rate_hz: 20.0` (0 = every scan): publish on a tracker step, else at most every 1/rate | BT and MPC wake-ups drop 40 → ≤ 20/s; JSON dumps halve. No semantic change (§3) |
| C4 | MPC_corr `/tf` listener | map→odom only | A `/tf` subscription that keeps only `map→odom`, still deserialising; or `use_map_frame_goal_anchor`-gated creation like costmap_boundary (`costmap_boundary_node.py:505-508`) | Removes ~110 → ~50 msg/s of Python TFMessage handling (after C1/C2) when the anchor is off. With the anchor on (default under ekf, `MPC_corr.py:388-390`) only C1/C2 help |
| C5 | detection_3d / obstacle_projector `/tf` listeners | static camera chain only | `tf_static_only` param, default true, as in `semantic_layer_node.py:269-279` | Removes `/tf` deserialisation from 2 Python processes, **ZED-off included** |
| C6 | MPC_corr per-message throttled logs | — | Pre-throttle as in the BT fix: `hw_odom_callback` (`MPC_corr.py:2006-2010`), `front_distance_callback` (`:3296-3300`), `joint_states_callback` (`:3267-3271`) | ~0.24 ms × (50 + 40 + 10)/s on linus, ≈ 2.4 % of a linus core, ~8 % Orin (est.) |
| C7 | swept_clearance_node | **no consumer** | add `enable_swept_clearance: false` (supervisor `_CONDITIONAL_AUTO_START`, as `wall_distance` was) | Removes a 40 Hz `/scan` plus depth (ZED) Python subscriber |
| C8 | obstacle_clearance_node | test-campaign logger only (manual) | `enable_obstacle_clearance: false`; turn it on for campaign runs | Removes another 40 Hz `/scan` Python subscriber. Note the supervisor comment explains why it was made always-on (`component_supervisor_node.py:313-316`), so this is a trade-off for the user |
| C9 | diagnostics_server `/sensors/core` 50 Hz | 2 Hz battery status | keep the sensor native; optional raw subscription with decimation (§4.2) | Small |
| C10 | MPC_corr QoS event handlers | — | apply `qos_events.drop_qos_event_handlers` (BT `f1tenth_behavior/qos_events.py`) | Same mechanism as the BT's −25 %; not measured for MPC (UNVERIFIED) |

Not candidates:
- `/scan` to the BT: safety.
- `/scan`, `/sensors/*`, ZED raw: sensors stay native.
- `/safety_stop` event stop at scan rate: safety.
- `/perception/walls`, `/wall_observations`, `/wall_planes`, `/mpc/*`: already ≤ 10 Hz.

---

## 5. Concrete change list (proposed, not applied)

**EKF change.** Land as one commit. Steps 1-3 are coupled.
1. `src/f1tenth_bringup/config/ekf.yaml:30` `frequency: 50.0` → `20.0`.
   - Also `src/f1tenth_bringup/config/ekf_global.yaml:86` `frequency: 50.0` → `20.0`.
   - Update the comment at `ekf_global.yaml:86-97` (sensor_timeout = 2 input periods) and at `:331-337` (the "50 Hz tick" wording).
2. `transform_time_offset` 0.05 → 0.08 in both files (`ekf.yaml:35`, `ekf_global.yaml:101`), per the pending decision at `docs/history/LOG.md:22-62`.
3. `multi_wall_tracker.PoseHistory.at(max_extrapolation_s=0.1)` (`multi_wall_tracker.py:170`) → a param, `lidar_front_wall_pose_max_extrapolation_sec`, default 0.15-0.2.
   - Pass it from `lidar_front_wall_node.py:559` and its call sites `:1254, 1023, 1331, 941-942`.
   - Also `lidar_front_wall_odom_max_age_sec` 0.2 → 0.3 (`stack_params.yaml:725-727`; node default `lidar_front_wall_node.py:415`; docstring `:90-91`).
4. `scripts/check_ekf_update_rate.py:44-47, 91-92, 104`: derive the target period from ekf.yaml instead of the hardcoded 20 ms.
5. Text that states 50 Hz:
   - `stack_params.yaml:727` (odom 50 Hz);
   - `src/f1tenth_logger/TEST_CAMPAIGN.md:37`;
   - `MPC_corr.py:394-396` (map_odom "~32 Hz", 6 cycles → ~4 at 20 Hz; the 0.2 s value stays);
   - `semantic_layer_node.py:126-133` (200 samples = 10 s at 20 Hz);
   - `steering_offset_calibration_node.py:29, 59, 737`;
   - `ekf.launch.py:40`.
6. Verify on the Orin:
   - `ros2 topic hz /odometry/filtered /ekf_global/odometry/filtered` ≈ 20;
   - no "wall tracker: no /odometry/filtered pose at the scan stamp" WARN (`lidar_front_wall_node.py:1259-1262`);
   - `/perception/virtual_wall/debug` `state` never NO_DATA while a wall is in view;
   - slam_toolbox shows no new "earlier than all the data" drops (`LOG.md:47-50`).

**virtual_wall.**

7. `lidar_front_wall_node.py:1148-1176`: publish the Float32 and the debug JSON on a tracker step, else at most every 1/`virtual_wall_max_rate_hz`.
   - New param `lidar_front_wall_virtual_wall_max_rate_hz: 20.0` in `stack_params.yaml`, wired through `lidar_front_wall.launch.py` next to `wall_tracker_scan_decimation` (`:102`).
   - Keep WallLineFit and WallEstimate per scan: their per-scan contract is in the docstring at `:4-6`. Or throttle them too and amend the docstring; their only live consumers are debug and viz.
   - Update the `wall_distance_max_age_s` text at `stack_params.yaml:758`.

**TF and logging CPU.**

8. MPC_corr (`MPC_corr.py:1613-1614`): create the TransformListener only when `use_map_frame_goal_anchor` is true. Otherwise subscribe `/tf_static` only, for the laser pose at `:3366`.
9. detection_3d_node (`:282`) and obstacle_projector_node (`:188`): add `tf_static_only` (default true), copying `semantic_layer_node.py:265-279`.
10. MPC_corr: pre-throttle the logger calls at `:2006-2010`, `:3296-3300` and `:3267-3271` (keep the throttle duration, skip the call when it would be throttled).

**Unconsumed publishers.**

11. Gate swept_clearance and obstacle_clearance behind `enable_*` keys in the supervisor's `_CONDITIONAL_AUTO_START` (`component_supervisor_node.py:325-348`), defaults per user decision. obstacle_clearance is campaign tooling.

**Out of scope, flagged.**

12. Add a `/scan` staleness gate to `IsProximityTooClose._tripped` (`is_proximity_too_close.py:371-380`), e.g. trip if there has been no scan for 0.2 s. Today the e-stop fails open if urg_node dies.
13. Add a staleness gate on MPC `obstacles_global_live` (`MPC_corr.py:3171-3184`). ZED-on only.

---

## 6. UNVERIFIED items
- robot_localization internals (Humble 3.5.4 per `ekf_global.yaml:284-288`): source not available. Specifically:
  - all queued measurements are fused each cycle, so input rate ≠ output rate;
  - the Odometry stamp = last measurement time;
  - default `history_length` and `predict_to_current_time`.
- vesc_driver and vesc_to_odom rates. 50 Hz comes from repo comments only; one replay note says `/odom` ran at ~10 Hz in some bags (`corridor_perception/replay.py:134`).
- joint_state_publisher default rate of 10 Hz.
- The ackermann_mux output rate.
- Whether anything publishes `/imu`.
- The tf2 `BufferCore` default cache of 10 s. It is not visible in `H:tf2_ros/buffer.py`; the MPC comment at `MPC_corr.py:2163` asserts it.
- The size of the scan-lead increase at 20 Hz (+0 to +30 ms plus jitter) is reasoning from `LOG.md:37-39, 57-59`, not a measurement.
- The CPU gains in §4.3 are estimates; none was measured in this audit.

---

## Skeptic check

An independent re-check of the claims above against the code at HEAD f66cfc5 and the Humble copies (`H:`). It was static only. Most claims hold: the EKF numbers, every gate value I checked, virtual_wall published per scan but changing only on tracker steps, the PoseHistory 0.1 s risk, the lack of a staleness gate in IsProximityTooClose, no consumer for swept_clearance, and the Humble `raw=` and `deserialize_message` APIs. The corrections and missing items follow.

### A. Wrong or imprecise claims (claim → correct fact)

1. **yolo_detector as a virtual_wall/debug consumer ("latest, 5 Hz", §1.3 and §3).** The claim is wrong.
   - The subscription is RELIABLE depth 10, and it runs `json.loads` on **every** message (`src/f1tenth_perception/f1tenth_perception/yolo_detector_node.py:1004-1008`, `:1108-1112`). Only the drawing runs at 5 Hz.
   - It is on by default: `debug_image_rate_hz` defaults to 5.0 (`stack_params.yaml:344-346`), the gate is `_debug_enabled = rate > 0` (`yolo_detector_node.py:445-447`), and `enable_scene_labels` must be true (`:1000-1001`).
   - `yolo_detector_node` is added to the launch unconditionally, before the `if is_zed:` block (`src/f1tenth_perception/launch/detection.launch.py:430-438`). So this cost exists with the ZED off too.
   - So the JSON is serialised at ≤ 40 Hz in lidar_front_wall and then parsed at ≤ 40 Hz in yolo. This makes C3, which throttles `/debug` too, more valuable than §4.3 says.
2. **reset_manager "fresh = `now - seen > 0.5` (`:374`)".** `:374` only picks the text of the timeout error. Freshness is `latest.wait(..., 0.2, after=seen)`, which needs a sample newer than the last one seen (`src/f1tenth_bringup/f1tenth_bringup/reset_manager_node.py:358-362`). The verdict is unchanged.
3. **`vesc.launch.py:211-213` (the vesc_to_odom publisher).** Those lines are docstring. The Node is at `src/f1tenth_hardware/f1tenth_hardware/launch/vesc.launch.py:219-229`, and `publish_tf: false` is at `src/f1tenth_bringup/config/vesc.yaml:204`.
4. **`description.launch.py:116-131` (joint_state_publisher).** The file is `src/f1tenth_description/launch/description.launch.py`.
   - Lines 119-128 are robot_state_publisher; joint_state_publisher is at `:181-188`.
   - It publishes static 0.0 for all 6 joints (`:7-13`). So MPC_corr's `joint_states_callback` (`MPC_corr.py:3250-3280`) reads constant zeros, and its rate is still UNVERIFIED.
5. **`check_ekf_update_rate.py:44-47`.** The constant is `_TARGET_PERIOD_MS = 20.0` at `scripts/check_ekf_update_rate.py:48`. The `--max-delay-ms` default of 50.0, described as "2.5x the target", is at `:91-93`. It needs re-deriving too, because at 20 Hz 50 ms is 1× the period.
6. **The `logger_node.py:...` citations.** The path is `src/f1tenth_logger/f1tenth_logger/test_campaign/logger_node.py`. Its subscriptions are at `:250-259` and the topic params at `:196-210`.
7. **Line drift, with the content correct:**
   - `wall_distance_max_age_s` is declared at `behavior_executor_node.py:765-766`, not 758-759;
   - `DetectedClassesBridge(node)` is at `:822`, not 815;
   - `_hold_for_wall` starts at `check_stop_condition.py:383`;
   - `front_distance_callback` is at `MPC_corr.py:3291-3298`;
   - `WALL_DEBOUNCE_TICKS = 3` is defined at `src/f1tenth_intelligence/llm/llm/plan_translate.py:159` and used at `:208`.
8. **§4.1 "BT subscribes `/mpc/*` (5 topics), semantic_tracks, detections".** The list is incomplete, but the conclusion holds because none of the missing topics is > 20 Hz. The BT process also subscribes:
   - BatteryStatus at 2 Hz (`is_battery_low.py:37`);
   - MissionStatus (`is_emergency_stop_triggered.py:53`);
   - SystemStatus, only with sys_obs (`is_system_overheated.py:104`);
   - `/camera/detections_3d`, only with `enable_camera_obstacle_stop`, default false (`behavior_executor_node.py:550-553`; `stack_params.yaml:1969-1970`);
   - `/tf_static` (`static_frames.py:40`);
   - the mission load-path topic (`mission/loader.py:249`).

### B. Missing items

1. **The EKF retarget is gated on a baseline that the branch just switched off.** `LOG.md:56-62` makes the 20 Hz change wait for a 50 Hz cost baseline from `ekf_cost_observer_node`, and says the offset should be revisited against its `period_ms_p90`/`max`.
   - Commit f2b4393 made that observer off by default (`src/f1tenth_localization/launch/localization.launch.py:72-86`).
   - `scripts/ekf_cost_report.py:4-13` is the before/after tool built for exactly this retarget.
   - **Add to §5 step 0:** run one baseline with `enable_ekf_cost_observer:=true` at 50 Hz and one at 20 Hz, and compare them with `ekf_cost_report.py`.
2. **The PoseHistory risk is probably larger than §2.2 says. Raise to 0.2 s, not 0.15.**
   - (a) `LOG.md:33-36`: D = scan − newest **TF** stamp is > 0 for 0.3-4.9 % of scans on odom→base_link, at 50 Hz. TF is post-dated by 0.05 (`ekf.yaml:35`), and PoseHistory uses the **Odometry header** stamp (`lidar_front_wall_node.py:680-682`). So, if Odometry stamp = TF stamp − offset (UNVERIFIED, robot_localization source absent), those scans already lead the newest pose by > 50 ms today. The worst recorded D of +64.8 ms (`LOG.md:42`) would then be ≈ 115 ms, past 0.1 s, if it was on this edge (the LOG does not say which edge).
   - (b) Humble rclpy takes **one** message per ready subscription per wait cycle, in creation order (`H:rclpy/executors.py:718-723`, `:398-402`). The odometry subscription is `qos_profile_sensor_data`, depth 5 (`lidar_front_wall_node.py:607-608`). After a 155-178 ms tracker step (`stack_params.yaml`, `lidar_front_wall_wall_debug_tracks` description), the next scan can be handled while queued odometry has not yet reached PoseHistory. At 20 Hz each missed sample is 50 ms instead of 20 ms. PLAUSIBLE; not measured.
   - Suggest adding `pose_lead_s = scan_stamp − newest pose stamp` to the `/perception/virtual_wall/debug` JSON (`:1176-1227`) and measuring it before choosing the value.
3. **The C8 trade-off is concrete, not only a comment.** With obstacle_clearance off, the test-campaign logger raises an ERROR and a `no_obstacle_clearance` event on every test (`test_campaign/logger_node.py:55-57`, `:512-530`; `clearance_grace_s 2.0`, `test_campaign_logger.yaml:20`). If C8 lands, the campaign launch must turn it back on.
4. **The safety path's QoS is not latest-only.** IsProximityTooClose subscribes to `/scan` RELIABLE KEEP_LAST 10 (`is_proximity_too_close.py:270-271`). Given the one-take-per-cycle executor (B2b), a BT process that falls behind handles the **oldest** queued scan first, so up to 10 × 25 ms = 250 ms stale. Every other slow `/scan` consumer uses depth 1 (`latest_frame_qos.py:21-22`). This was not raised in §1/§5. PLAUSIBLE; not measured. Out of scope like item 12, but it belongs to "e-stop at /scan rate".
5. **Viz relays are lazy.** `'lazy': True` (`src/f1tenth_bringup/launch/viz_relays.launch.py:78`; `stack_params.yaml:2528-2531`) means the throttles subscribe to `/scan`, `/sensors/core` and `/ekf_global/...` only while a Foxglove client is connected. They cost nothing with no client.
6. **There is field evidence that consumers already live below 50 Hz.** A bag under load got `/odometry/filtered` at 38 Hz and `/scan` at 30 Hz (`stack_params.yaml:2172`). It supports the 20 Hz verdicts.
7. **The VESC command topics are missing from §1.** ackermann_to_vesc publishes `/commands/motor/*` and `/commands/servo/position` per `/ackermann_drive` message (`vesc.launch.py:332-339`). That is up to the event-stop rate plus 10 Hz while tripped. They are consumed in C++ by vesc_driver and vesc_to_odom. Topic names and rates are UNVERIFIED, because the vesc sources are absent.
8. **More 50 Hz text to update in §5 step 5:**
   - `src/f1tenth_localization/launch/ekf.launch.py:27`;
   - `src/f1tenth_localization/launch/ekf_global.launch.py:30`;
   - `localization.launch.py:75`;
   - `costmap_boundary_node.py:81, 500`;
   - `semantic_layer_node.py:264`;
   - `src/f1tenth_diagnostics/launch/steering_offset_calibration.launch.py:73`;
   - `stack_params.yaml:1111` (wall_distance odom);
   - `MPC_corr.py:1270`;
   - `lidar_front_wall_node.py:90-91`;
   - `ekf_global.yaml:92`.
9. **C4 adds nothing by default.** `localization_source` defaults to `ekf` (`stack_params.yaml:48-49`), so `use_map_frame_goal_anchor` is true (`MPC_corr.py:388-390`), and the map→odom lookups at `:2098, 2245, 2490` are live.
   - A real alternative: derive map→odom from `/ekf_global/odometry/filtered` (20 Hz) composed with the `/odometry/filtered` already subscribed, instead of a listener on all of `/tf`. The stamp-matching semantics are UNVERIFIED.
   - Note also that Humble's listener subscribes `/tf` RELIABLE depth 100 in a Reentrant group (`H:tf2_ros/transform_listener.py:68-88`), so after a long solve MPC_corr drains a burst of up to 100 TFMessages.
10. **The C3 design is safe.** `VirtualWall.step` is stateless apart from `update_pose`/`update_stereo` (`src/f1tenth_perception/f1tenth_perception/scene_labels.py:597-627`). Skipping it on throttled scans loses nothing, but `_publish_virtual`'s returned `state` still feeds the per-scan WallEstimate (`lidar_front_wall_node.py:1076-1079`), so compute the state every scan and throttle only the two publishes. With `use_wall_tracker:=false` every scan is a new value, and a 20 Hz cap then decimates real data (acceptable at 10 Hz consumers).
11. **semantic_layer's wall layer also uses the 200-sample pose history.** `_walls_cb` looks up the nearest pose with no gap limit (`semantic_layer_node.py:384-387`), now driven by the 5 Hz timer from commit 752f160. At 20 Hz the nearest-pose error is ≤ 25 ms × v; that is OK.
12. **Preflight also requires GLOBAL_XY**, which means `/ekf_global` has been received at least once (`mission/preflight.py:222`). It is a one-shot presence check with no rate gate (`:22-45`), so it is rate-independent.
13. **A Humble API note on §4.2.** The Jazzy Python `TransformListener` also has no `static_only` option (checked `/opt/ros/jazzy/.../tf2_ros/transform_listener.py`). So the hand-rolled `/tf_static` pattern is right on both distros, and this is not a Humble difference.

### C. Verdict on the change list
- Steps 1-2: agree, plus the new step 0 (B1).
- Step 3: agree, but use a default of 0.2 s and measure `pose_lead_s` first (B2).
- Step 4: also fix `--max-delay-ms` (A5).
- Step 5: add the files in B8.
- Step 7 (C3): agree; it is stronger because of A1 and needs the B10 detail.
- Step 8 (C4): no gain under the default config (B9).
- Step 11 (C8): needs the campaign-logger coupling resolved (B3).
- Item 12: also add the QoS issue in B4.
