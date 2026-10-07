# Phase 1 — Code analysis: ideal rates

Static analysis only. Nothing in the stack was launched, modified or measured
to produce this document; every claim here is traceable to a file and line.
Phase 2 measures the live stack and Phase 3 reconciles the two — several
entries below are explicitly marked as needing that.

---

## 0. Corrections to the work order's premises

These are stated first because three of them change what Phases 2–4 can
usefully do.

| # | Premise as given | What the code says |
|---|---|---|
| P1 | `supervisor.launch.py` | The file is **`supervisor_bringup.launch.py`** (`src/f1tenth_bringup/launch/supervisor_bringup.launch.py`). There is no `supervisor.launch.py`. |
| P2 | "ZED 2i" | **You are right about the hardware; the config says `zed2`, and the SDK overrides it at runtime.** `lsusb` reports `2b03:f880 STEREOLABS ZED 2i`. `camera.launch.py:97` passes `camera_model: 'zed2'`, so the wrapper loads `zed2.yaml` rather than the `zed2i.yaml` beside it, and every topic is namespaced `/zed2/zed_node/...`. **Resolved in the 2026-09-24 run**: the wrapper opens the camera successfully (S/N 36866199) and logs `Camera model does not match user parameter. Please modify the value of the parameter 'general.camera_model' to 'zed2i'`, then reports `* Camera Model -> ZED 2i`. The SDK detects the real model and uses it; the parameter is **cosmetic and warning-level only**. `zed2.yaml` and `zed2i.yaml` differ *only* in `camera_model`/`camera_name`, so no tuning value is wrong either. |
| P3 | "disable topics/features nobody uses (especially the ZED 2i)" | **A ZED optimization pass has already been done**, and it already disabled object detection, body tracking, mapping, positional tracking, IMU publishing, the stereo pair and the status topics, halved the published resolution and capped the frame rate. See §5. The remaining ZED headroom is small and is mostly *depth*, which is genuinely consumed. |
| P4 | Phase 2's `ros2 topic list` / `hz` / `info -v` method | The stack runs under a **Fast DDS Discovery Server** (`supervisor_bringup.launch.py` sets `ROS_DISCOVERY_SERVER` before any node starts). A shell without that variable exported sees an **empty** `ros2 node list` / `ros2 topic list` — not an empty graph. This is a known, previously-bitten failure mode in this workspace. Phase 2 must export `ROS_DISCOVERY_SERVER` in the measuring shell (and/or probe with rclpy) or every measurement will read zero. |
| P5 | "the motors aren't driven in this mode" | **Confirmed, with a caveat.** See §1.3. Commanded speed is zero at idle and nothing auto-loads a mission, but the VESC driver is live and the steering servo *is* commanded. The car should be on blocks. |

---

## 1. Environment and launch tree

### 1.1 Platform

| Item | Value | Source |
|---|---|---|
| ROS distro | Humble | `/opt/ros/humble`, `ROS_DISTRO=humble` |
| Platform | Jetson (Linux 5.15.185-tegra, Orin) | `uname`; `tegrastats` is the right baseline tool for Phase 2 §7 |
| Middleware | Fast DDS with **Discovery Server** | `supervisor_bringup.launch.py:70-73` (`SetEnvironmentVariable('ROS_DISCOVERY_SERVER', ...)`) |
| Workspace install | **symlink-install** | `src/` edits reach installed nodes; only a relaunch is needed |

Because `ROS_LOCALHOST_ONLY` / `ROS_AUTOMATIC_DISCOVERY_RANGE` interacts with
the Discovery Server, the Wi-Fi note the work order asks for in Phase 4 has to
be written against the Discovery Server setup, not against plain SIMPLE
discovery. Humble has **no** `ROS_AUTOMATIC_DISCOVERY_RANGE` (that is Iron and
later) — on Humble the levers are `ROS_LOCALHOST_ONLY`, `ROS_DOMAIN_ID`, and
the Discovery Server address itself.

### 1.2 What actually starts

`supervisor_bringup.launch.py` starts exactly two things: the Fast DDS
discovery server (via `scripts/ensure_discovery_server.py`) and
`component_supervisor_node`. That node then reads
`src/f1tenth_bringup/config/components.yaml` and spawns each component as its
own `ros2 launch` subprocess.

Auto-start sets are in `component_supervisor_node.py:324-344`:

- `_ALWAYS_AUTO_START` — hardware, localization, navigation, perception,
  lidar_front_wall, wall_distance, swept_clearance, obstacle_clearance,
  control, diagnostics, dev_tools, slam, reset_manager
- `_CONDITIONAL_AUTO_START` — behavior (`use_behavior_tree`), intelligence
  (`enable_intelligence`)
- `_NEVER_AUTO_START` — calibrate_hardware, startup_sequence

Branching defaults from `src/f1tenth_params/config/stack_params.yaml`:

| Key | Default | Consequence |
|---|---|---|
| `calibration` | `false` | no calibration nodes, no calibration drive |
| `enable_nav2` | `false` | `navigation.launch.py` brings up **mpc_corr**, not Nav2 |
| `use_behavior_tree` | `true` | `behavior` starts |
| `enable_intelligence` | `true` | `llm` + llama-server start |
| `camera_source` | `zed` | ZED 2 wrapper starts (not the v4l2 webcam path) |
| `localization_source` | `ekf` | MPC reads `/odometry/filtered` |
| `enable_slam` | `true` | `async_slam_toolbox_node` starts |
| `enable_foxglove` | `true` | foxglove_bridge + 2 image throttles start |
| `enable_viz_relays` | `true` | the `/viz/*` throttle container starts |
| `mission_file_name` | `''` | **no mission loaded at boot** |

So a default `ros2 launch f1tenth_bringup supervisor_bringup.launch.py` brings
up roughly 35 nodes across 15 components.

### 1.3 Motor-safety confirmation (Phase 2 gate)

The work order requires confirming from the code that the motors are not driven.
Confirmed, by four independent facts:

1. **No mission is loaded at boot.** `stack_params.yaml:1755` sets
   `mission_file_name: ''`, documented as "start with no mission loaded
   (mission.state stays IDLE until `/mission/load_path` or
   `/mission/load_mission`)".
2. **The MPC publishes a stop when it has no goal.** `MPC_corr.py:3666`
   (`elif self.goal_distance is None or self.goal_start_xy is None:`) calls
   `self._publish_stop()` and returns. With no odom at all
   (`MPC_corr.py:3484`) it calls `_publish_drive(0.0, 0.0)`. Both paths command
   zero speed.
3. **Calibration is off.** `calibration: false`, so `vesc.launch.py` never
   enters its calibration sequencing. (Note the calibration that *is* wired
   here is the stationary covariance/gyro-bias pair, not a drive; the 1 m
   speed-tuning drive lives in `vesc_tuning/speed_tuning.launch.py`, which is
   in no component.)
4. **`startup_sequence` — the steering sweep — is in `_NEVER_AUTO_START`.**

**Caveat, stated plainly.** This is "commanded to zero", not "disarmed". The
VESC driver (`vesc_driver_node`) is running with a live serial link to the
motor controller, `ackermann_mux` is arbitrating, and `ackermann_to_vesc_node`
is translating. The steering servo **is** commanded — `_publish_stop()` still
carries a steering angle — so the front wheels will move during the run. A
single stray publish on `/drive`, `/teleop`, `/safety_stop` or
`/calibration_drive` would move the car. Phase 2's "never publish to actuation
topics" rule is what keeps this safe; the car should be on blocks regardless.

### 1.4 Launch tree (auto-started only)

```
supervisor_bringup.launch.py
├── fastdds discovery server            (ensure_discovery_server.py)
└── component_supervisor_node
    ├── hardware      vesc.launch.py
    │                   ├── vesc_driver_node               (vesc_driver, C++)
    │                   ├── vesc_to_odom_node_backup       (vesc_ackermann, C++)
    │                   ├── ackermann_to_vesc_node         (vesc_ackermann, C++)
    │                   ├── static_transform_publisher     (base_link->imu)
    │                   └── battery_voltage_check_node
    ├── localization  localization.launch.py
    │                   ├── ekf.launch.py        -> ekf_node          (robot_localization)
    │                   ├── ekf_global.launch.py -> ekf_node (global) + slam_pose_relay_node
    │                   ├── ekf_cost_observer_node
    │                   └── description.launch.py -> robot_state_publisher, joint_state_publisher, static TF
    ├── navigation    navigation.launch.py  (enable_nav2=false branch)
    │                   ├── mpc_corr.launch.py -> mpc_corr            (mpc_controller)
    │                   └── map_only.launch.py -> map_server + lifecycle_manager
    ├── perception    camera.launch.py    -> zed_camera.launch.py (zed_wrapper) + static TF
    │                  lidar.launch.py     -> urg_node_driver
    │                  detection.launch.py -> yolo_detector_node, detection_3d_node,
    │                                         obstacle_projector_node, front_clearance_node
    ├── lidar_front_wall  -> lidar_front_wall_node
    ├── wall_distance     -> wall_distance_node
    ├── swept_clearance   -> swept_clearance_node
    ├── obstacle_clearance-> obstacle_clearance_node
    ├── control       ackermann_mux.launch.py -> ackermann_mux
    ├── slam          slam.launch.py    -> async_slam_toolbox_node
    │                  costmap.launch.py -> semantic_layer_node, costmap_renderer_node,
    │                                       costmap_boundary_node
    ├── diagnostics   system_observer_node, diagnostics_server_node, mission_logger_node
    ├── dev_tools     foxglove_bridge + 2x topic_tools throttle + viz relay container
    ├── reset_manager -> reset_manager
    ├── behavior      behavior_executor_node, twist_to_ackermann_node
    └── intelligence  llm.launch.py -> llm_planner_node + llama-server
```

---

## 2. The actuation chain

This is the one chain where a rate change is a safety change, so it is called
out separately.

```
mpc_corr  --/drive-->  ackermann_mux  --/ackermann_drive-->  ackermann_to_vesc_node
                                                                    |
                                            /commands/motor/speed   |
                                            /commands/servo/position|
                                                                    v
                                                             vesc_driver_node --> VESC
```

Mux lanes (`src/f1tenth_bringup/config/mux.yaml`), all with `timeout: 0.2`:

| Lane | Topic | Priority | Publisher |
|---|---|---|---|
| safety_stop | `safety_stop` | 200 | behavior tree Stop action |
| joystick | `teleop` | 100 | joy teleop (not auto-started) |
| calibration | `calibration_drive` | 50 | steering_offset_calibration_node (manual only) |
| navigation | `drive` | 10 | **mpc_corr** |

**The 0.2 s mux timeout is a hard floor on every command-lane rate.** Any lane
publishing slower than 5 Hz is treated as dead and drops out of arbitration.
This bounds the MPC below at 5 Hz no matter what else the analysis says.

---

## 3. Producer inventory

Rates are the *configured* value with its source. "callback" means the node has
no timer and publishes once per input message, so its rate is its input's rate.

### 3.1 Hardware / state estimation

| Node | Topic | Type | Configured rate | Source |
|---|---|---|---|---|
| `vesc_driver_node` | `/sensors/core` | VescStateStamped | **50 Hz** | `vesc_driver.cpp:147` `create_wall_timer(20ms)` — **hardcoded, vendor source** |
| `vesc_driver_node` | `/sensors/imu` | Imu | 50 Hz | same timer |
| `vesc_driver_node` | `/sensors/imu/raw` | Imu | 50 Hz | same timer |
| `vesc_driver_node` | `/sensors/servo_position_command` | Float64 | callback | on servo command |
| `vesc_to_odom_node_backup` | `/odom` | Odometry | callback on `/sensors/core` → 50 Hz | `vesc_ackermann` |
| `ackermann_to_vesc_node` | `/commands/motor/speed` | Float64 | callback on `/ackermann_drive` | `vesc_ackermann` |
| `ackermann_to_vesc_node` | `/commands/servo/position` | Float64 | callback | `vesc_ackermann` |
| `ekf_node` (local) | `/odometry/filtered` | Odometry | **50 Hz** | `config/ekf.yaml:30` `frequency: 50.0` |
| `ekf_node` (local) | `odom->base_link` TF | TFMessage | 50 Hz | `ekf.yaml:33` `publish_tf: true` |
| `ekf_node` (global) | `/ekf_global/odometry/filtered` | Odometry | 50 Hz (see `ekf_global.yaml`) | `ekf_global.launch.py` |
| `ackermann_mux` | `/ackermann_drive` | AckermannDriveStamped | callback on winning lane → 10 Hz | `ackermann_mux` |

### 3.2 Sensors

| Node | Topic | Type | Configured rate | Source |
|---|---|---|---|---|
| `urg_node_driver` | `/scan` | LaserScan | **40 Hz native** (UST-10LX, 1081 beams, intensity on) | `config/sensors.yaml`; `cluster: 1`, `skip: 0` |
| ZED wrapper | `/camera/image_raw` (remap of `/zed2/zed_node/rgb/image_rect_color`) | Image | **30 Hz**, half resolution | `zed2_perception.yaml` `pub_frame_rate: 30.0`, `pub_downscale_factor: 2.0` |
| ZED wrapper | `/camera/camera_info` | CameraInfo | 30 Hz | same |
| ZED wrapper | `/zed2/zed_node/depth/depth_registered` | Image | 30 Hz configured (**~25 Hz observed** per `detection_3d_node.py:139`) | same |
| ZED wrapper | `/zed2/zed_node/depth/camera_info` | CameraInfo | 30 Hz | same |
| ZED wrapper | point cloud | PointCloud2 | **subscriber-gated, 0 Hz with no subscriber** | `zed2_perception.yaml` depth section |

### 3.3 Perception

| Node | Topic | Type | Configured rate | Source |
|---|---|---|---|---|
| `yolo_detector_node` | `/camera/detections` | Detection2DArray | callback on `/camera/image_raw` → ≤30 Hz, GPU-bound | `detection.launch.py:311-313` |
| `yolo_detector_node` | `/camera/image_annotated` | Image | callback, same | `detection.launch.py:313` |
| `yolo_detector_node` | `/camera/detection_masks` | Image | callback, same | `detection.launch.py:325` |
| `detection_3d_node` | `/camera/detections_3d` | Detection3DArray | callback via 3-way `ApproximateTimeSynchronizer` | `detection_3d_node.py:254-267` |
| `detection_3d_node` | markers | MarkerArray | same | `detection_3d_node.py:248` |
| `obstacle_projector_node` | `/perception/obstacles_2d` | Obstacle2DArray | callback on detections_3d | `obstacle_projector_node.py:190` |
| `front_clearance_node` | `/perception/front_distance` | Float32 | callback | `front_clearance_node.py:508` |
| `front_clearance_node` | `/perception/front_wall`, `/front_clearance`, `/front_blocked` | Bool/Float32 | callback | `front_clearance_node.py:510-514` |
| `front_clearance_node` | `/perception/debug/front_distance_raw`, `/debug/bg_pixel_count` | Float32 | callback | `front_clearance_node.py:525-527` |
| `lidar_front_wall_node` | `/perception/wall_fit`, `/wall_estimate` | WallLineFit/WallEstimate | callback on `/scan` → 40 Hz | `lidar_front_wall_node.py:273-274` |
| `wall_distance_node` | `/perception/wall_distance` (+ `/segment`, `/psi_correction`, `/gate_margin`, `/swept_arc`) | WallDistance etc. | **timer, `tick_period`** | `wall_distance_node.py:291` |
| `swept_clearance_node` | `/perception/swept_clearance` (+ `/lidar`, `/camera`, `/steering`) | Float32 | **20 Hz** | `swept_clearance_node.py:177` `publish_rate_hz = 20.0` |
| `obstacle_clearance_node` | `/obstacle_clearance`, `/safety/event` | Float32/String | callback on `/scan` → 40 Hz | `obstacle_clearance_node.py:123-124` |

### 3.4 Control

| Node | Topic | Type | Configured rate | Source |
|---|---|---|---|---|
| `mpc_corr` | `/drive` | AckermannDriveStamped | **10 Hz** | `MPC_corr.py:727` `self.ts = 0.1`; `MPC_corr.py:1859` `create_timer(self.ts, self.control_loop)` — **timer, not callback** |
| `mpc_corr` | `/mpc/drive_clamp` | DriveClamp | 10 Hz | `MPC_corr.py:1745` |
| `mpc_corr` | `/mpc/min_obstacle_distance` (+ `_forward`) | Float32 | 10 Hz, **every tick regardless of goal state** | `MPC_corr.py:1747,1774` + `control_loop:3473-3481` |
| `mpc_corr` | `/mpc/predicted_clearance` | Float32 | 10 Hz | `MPC_corr.py:1782` |
| `mpc_corr` | `/mpc/solver_status` | — | 10 Hz | `MPC_corr.py:1795` |
| `mpc_corr` | `/mpc/status` | String | 10 Hz | `MPC_corr.py:1805` |
| `mpc_corr` | `/corridor` | String | on corridor rebuild, `corridor_update_period = 1.0 s` | `MPC_corr.py:1806,1128` |
| `mpc_corr` | `/mpc/goal_reached` | — | event | `MPC_corr.py:1809` |
| `mpc_corr` | `/mpc/corridor_markers` | MarkerArray | on rebuild | `MPC_corr.py:1839` |
| `mpc_corr` | `/mpc/wall_track` | WallTrack | wall_turn only | `MPC_corr.py:1852` |

**The MPC runs on a timer at 10 Hz.** The work order's example ("40–50 Hz for a
20 Hz MPC") assumes 20 Hz; the real loop is 10 Hz. This matters for the
odom-rate rule below.

### 3.5 Mapping / costmap / viz / diagnostics

| Node | Topic | Configured rate | Source |
|---|---|---|---|
| `async_slam_toolbox_node` | `/slam/map`, `/slam/pose` | map on update; `transform_publish_period: 0.0` (no map->odom TF, deliberate) | `slam.launch.py` |
| `semantic_layer_node` | costmap semantic layer / markers | — | `costmap.launch.py:147` |
| `costmap_renderer_node` | costmap image | **2 Hz** | `costmap_renderer_node.py:54` `render_rate_hz = 2.0` |
| `costmap_boundary_node` | `/costmap/boundaries` | **20 Hz** | `costmap_boundary_node.py:363` `extraction_rate_hz = 20.0` |
| `system_observer_node` | system stats | **1 Hz** | `system_observer_node.py:41` |
| `ekf_cost_observer_node` | EKF cost | **1 Hz** | `ekf_cost_observer_node.py:292` |
| `map_server` | `/map` | latched (transient_local), once | `map.launch.py` |
| `topic_tools throttle` x2 | `/camera/image_raw/viz`, `/camera/image_annotated/viz` | **5 Hz** | `foxglove_image_throttle_hz: 5.0` |
| viz relay container | `/viz/*` | 5.5 Hz (3.3 Hz for annotated image) | `viz_relays.yaml` + `stack_params.yaml` |

---

## 3.6 The camera chain, consumer by consumer

Added 2026-09-24, with the camera actually running. This is the section the
rate proposal turns on, so every consumer is named with its subscription style
— which matters, because the three depth consumers do **not** subscribe the
same way.

### Who consumes what

| Producer topic | Consumer | Subscription style | What it needs |
|---|---|---|---|
| `/camera/image_raw` (ZED RGB, remapped) | `yolo_detector_node` | plain, `image_topic` param | one frame per inference; anything faster is dropped |
| | `image_raw_viz_throttle` | `topic_tools` throttle, **lazy** | 5 Hz for Foxglove only |
| `/zed2/zed_node/depth/depth_registered` | `detection_3d_node` | **`message_filters.ApproximateTimeSynchronizer`**, 3-way (detections + depth + masks), `detection_3d_node.py:254-267` | a depth frame time-matched to each detection |
| | `front_clearance_node` | **plain `create_subscription`** ×4, deliberately *not* synchronized (`front_clearance_node.py:13` says so explicitly), lines 532-538 | latest depth at detection time |
| | `swept_clearance_node` | **plain `create_subscription`**, `qos_profile_sensor_data`, gated on `use_camera` | latest depth at its own 20 Hz timer |
| `/zed2/zed_node/depth/camera_info` | `detection_3d_node`, `swept_clearance_node` | plain | intrinsics; effectively static |
| `/camera/detections` | `detection_3d_node` | message_filters | — |
| | `front_clearance_node` | plain | — |
| | `mission_logger_node` (`mission_logger_node.py:204`) | plain | logging |
| | **`detected_classes_bridge`** (inside `behavior_executor_node`, `mission/detected_classes_bridge.py:33`) | plain | **the behaviour tree sees detections through this** |
| `/camera/detection_masks` | `detection_3d_node`, `front_clearance_node` | message_filters / plain | per-detection mask |
| `/camera/detections_3d` | `obstacle_projector_node`, `semantic_layer_node` (`semantic_layer_node.py:157`) | plain | — |
| `/camera/image_annotated` | `image_annotated_viz_throttle` **only** | lazy throttle | 3.3 Hz debug |

**Correction to an earlier claim in this document**: only `detection_3d_node`
uses `message_filters`. `front_clearance_node` explicitly avoids a synchronizer,
and `swept_clearance_node` uses a plain sensor-QoS subscription. A
`create_subscription` grep therefore misses exactly one node, not three.

### `swept_clearance_node`'s camera gate

`swept_clearance.launch.py:93` sets `'use_camera': get_value('camera_source') == 'zed'`,
and `camera_source` defaults to `zed`. So with the default stack **this node
does hold a live subscription on the ZED depth stream**, which is what makes it
relevant to the camera proposal and not merely to CPU.

### Rate the chain can actually sustain

`yolo_model` defaults to **`yolo26s-seg.pt`** with `yolo_model_task: segment`
on `yolo_device: cuda` (`stack_params.yaml`). A segmentation model is
substantially heavier than the box model this stack used earlier. The ideal
rate for `/camera/image_raw` is therefore *whatever YOLO can sustain* — any
faster and the wrapper is producing frames that are decoded, copied and
dropped. Phase 2 measures it.

## 3.7 `/costmap/boundaries` — frame and how the MPC uses it

Both asked for explicitly.

**frame_id**: `costmap_boundary_node.py:643,718,754` set
`arr.header.frame_id = self.robot_frame` — the message is published in the
**robot body frame** (`base_link`), not in a world frame.

**How the MPC uses it**: `MPC_corr.py:3120` `costmap_boundaries_callback`

1. Drops the frame entirely if the MPC has no pose yet.
2. Converts each constraint to **world frame at receipt time**, using the
   MPC's *current* `(self.x, self.y, self.yaw)`, via `_boundary_to_world`.
3. Stores the result plus `costmap_boundaries_last_time`.
4. At solve time `_get_live_boundaries()` applies a staleness gate
   (`_select_live_boundaries`, timeout = `odom_stale_timeout_sec` = **0.5 s**)
   and passes the survivors straight into `solve_mpc_step(boundaries=...)`.

**Constraint hardness**: gated by `use_hard_boundary_constraints` (default
**true**), but `boundary_hard` defaults **false** and `boundary_slack_weight`
is **10000.0** — so they enter the solver as *soft* constraints with a very
large slack penalty, not as true hard constraints. At most
`boundary_max_sources: 3` of them (front/left/right).

**Why this matters for the rate.** Because the transform is applied at
*receipt* and then held until the next message, the pose baked into the
constraint is up to one publish period old when the solver uses it. Halving
the rate doubles that staleness. The 0.5 s gate is not the binding limit
(10 Hz still leaves 5× margin); the binding quantity is **position error in
the constraint**, which scales with speed. See Phase 4 P4 for the numbers.

## 4. Consumer → producer dependency map

Read as: **topic → who consumes it → what the fastest consumer actually needs.**

### 4.1 State-estimator inputs — DO NOT REDUCE (work-order rule 1)

| Topic | Rate | Consumers | Verdict |
|---|---|---|---|
| `/sensors/core` | 50 Hz | `vesc_to_odom_node_backup`, mission_logger, viz relay | **locked** — EKF chain input |
| `/sensors/imu/raw` | 50 Hz | local `ekf_node` (`imu0`), `mpc_corr` (`/imu` sub) | **locked** |
| `/odom` | 50 Hz | local `ekf_node` (`odom0`), viz | **locked** |
| `/scan` | 40 Hz | `async_slam_toolbox_node`, `lidar_front_wall_node`, `wall_distance_node`, `swept_clearance_node`, `obstacle_clearance_node`, behavior tree `IsProximityTooClose` (e-stop), viz relay | **locked at native rate** — work-order rule 4, and it is the e-stop's only sensor |

### 4.2 Estimator outputs — reducible in principle

| Topic | Rate | Consumers | Ideal | Reasoning |
|---|---|---|---|---|
| `/odometry/filtered` | 50 Hz | `mpc_corr` (10 Hz loop), `wall_distance_node`, `lidar_front_wall_node`, costmap, mission_logger | **20–25 Hz** by the 2× rule | 50 Hz is **5× the 10 Hz MPC**, not 2×. By the work order's own rule the ideal is ~20 Hz. **But see the warning below.** |
| `odom->base_link` TF | 50 Hz | SLAM, costmap, every TF lookup in perception | **50 Hz — locked** | Produced by the *same* `frequency: 50.0` knob as `/odometry/filtered`. They cannot be separated in `robot_localization`. |
| `/ekf_global/odometry/filtered` | 50 Hz | `mpc_corr` map-anchor refresh, logger | 20–25 Hz | Same knob, same coupling |

> **Warning carried forward to Phase 4.** `/odometry/filtered` and the
> `odom->base_link` transform come from one `frequency:` parameter in
> `ekf.yaml`. Dropping it to 20 Hz to satisfy the MPC rule would *also* drop
> the TF that SLAM and the whole perception TF chain depend on, from 50 Hz to
> 20 Hz. This is a coupling the work order's rule does not anticipate. The
> honest options are (a) leave the EKF at 50 Hz and accept 5× oversampling,
> which costs one `Odometry` message at ~50 Hz — a few tens of kB/s, or
> (b) throttle only the MPC's own subscription. (a) is almost certainly right;
> the saving from (b) is not worth the risk to a state input.

### 4.3 Camera chain

| Topic | Rate | Consumers | Ideal | Reasoning |
|---|---|---|---|---|
| `/camera/image_raw` | 30 Hz | `yolo_detector_node`, `image_raw_viz_throttle` | **= YOLO's real throughput** | If YOLO cannot sustain 30 Hz on this Orin (very likely — Phase 2 must measure), the wrapper is publishing frames that are dropped. Match `pub_frame_rate` to measured YOLO rate. |
| `/camera/camera_info` | 30 Hz | (paired with image) | match image | — |
| `/zed2/.../depth/depth_registered` | ~25 Hz | `detection_3d_node`, `front_clearance_node`, `swept_clearance_node` — **all three via `message_filters`** | match the detection rate | Genuinely consumed by three nodes. **Not** a disable candidate. |
| `/zed2/.../depth/camera_info` | 30 Hz | `detection_3d_node`, `swept_clearance_node` | match depth | — |
| `/camera/detections` | ≤30 Hz | `detection_3d_node`, `front_clearance_node` | = YOLO rate | callback-driven |
| `/camera/detection_masks` | ≤30 Hz | `detection_3d_node`, `front_clearance_node` | = YOLO rate | **Image-typed, full resolution** — a real bandwidth item |
| `/camera/image_annotated` | ≤30 Hz | `image_annotated_viz_throttle` **only** | **debug — candidate for gating** | Nothing in the race path consumes it. It is a full Image at YOLO rate whose only consumer is a 3.3 Hz viz throttle. |

### 4.4 Control chain

| Topic | Rate | Consumers | Ideal | Reasoning |
|---|---|---|---|---|
| `/drive` | 10 Hz | `ackermann_mux` (navigation lane) | **10 Hz** | Work-order rule 3: match the controller. Mux timeout 0.2 s is a 5 Hz floor. |
| `/ackermann_drive` | 10 Hz | `ackermann_to_vesc_node`, `swept_clearance_node` | 10 Hz | callback |
| `/commands/motor/speed`, `/commands/servo/position` | 10 Hz | `vesc_driver_node` | 10 Hz | callback |
| `/safety_stop` | event | `ackermann_mux` | event | e-stop lane, never throttle |

### 4.5 Classification summary

**Race-critical** (never reduce below ideal): `/scan`, `/sensors/imu/raw`,
`/sensors/core`, `/odom`, `/odometry/filtered`, `odom->base_link` TF, `/drive`,
`/ackermann_drive`, `/commands/*`, `/safety_stop`, `/camera/image_raw`,
depth + depth camera_info, `/camera/detections`, `/camera/detection_masks`,
`/perception/obstacles_2d`, `/perception/front_*`, `/obstacle_clearance`.

**Debug / visualization**: `/camera/image_annotated`, `/viz/*` (already
throttled to 5.5 / 3.3 Hz), `/camera/image_raw/viz`,
`/camera/image_annotated/viz`, `/mpc/corridor_markers`, `/mpc/status`,
`/corridor`, `/perception/debug/*`, `/mpc/min_obstacle_distance*`
(the code itself calls these "diagnostic, no consumer yet"),
costmap renderer output, `system_observer` and `ekf_cost_observer` at 1 Hz.

**Unused (no in-workspace subscriber found)**: `/perception/swept_clearance`
and its three sub-topics — `components.yaml` states outright that "Nothing
subscribes to `/perception/swept_clearance` yet", and the node has never run
on the car. It costs a 20 Hz timer plus a `message_filters` subscription to the
**ZED depth image** and `/scan`. This is the single largest genuinely-unused
consumer in the stack. `/mpc/min_obstacle_distance` and
`/mpc/min_obstacle_distance_forward` are also marked "no consumer yet" in
`MPC_corr.py` but cost only a Float32 at 10 Hz.

---

## 5. ZED 2 feature-by-feature status

The work order asks for a per-feature consumed/unconsumed verdict. The honest
answer is that most of this was already done in a prior "perception-optimization
pass"; `zed2_perception.yaml` is the override file and it is already lean.

| Feature | Setting today | Consumed by a race-stack node? | Ideal |
|---|---|---|---|
| RGB rect colour | `publish_rgb: true`, 30 Hz, half res | **Yes** — `yolo_detector_node` | keep, rate = measured YOLO throughput |
| Stereo pair | `publish_stereo: false` | No | **already disabled** |
| Depth | `depth_mode: 'PERFORMANCE'` | **Yes** — `detection_3d_node`, `front_clearance_node`, `swept_clearance_node` | keep |
| Point cloud | no publish flag; subscriber-gated | **No subscriber anywhere** | already 0 Hz in practice; cannot be hidden from the topic list in this wrapper version |
| Positional tracking | `pos_tracking_enabled: false` **and** `depth_stabilization: 0` | No | **already disabled** (and the `depth_stabilization` fix is the one that actually stopped it) |
| Mapping | `mapping_enabled: false` | No | **already disabled** |
| Object detection | `od_enabled: false` | No | **already disabled** |
| Body tracking | upstream default `false` | No | already disabled |
| IMU | `publish_imu: false`, `publish_imu_raw: false` | No — the EKF fuses only the **VESC** IMU (`ekf.yaml` `imu0: sensors/imu/raw`) | **already disabled** |
| IMU TF | `publish_imu_tf: false` **and** launch-arg override | No | **already disabled** — and this one was a real 65%-of-`/tf` bug, fixed |
| TF / map TF | `publish_tf: false`, `publish_map_tf: false` | No — stack owns map/odom | already disabled |
| Status topics | `publish_status: false` | No | already disabled |

**Remaining ZED headroom** is therefore only: (a) resolution/frame rate, if
Phase 2 shows YOLO cannot keep up with 30 Hz half-res, and (b) whether
`swept_clearance` — an unused consumer — should stop pulling the depth stream
at all. Both are Phase 4 items, both depend on Phase 2 numbers.

---

## 6. Summary table

Rates marked "?" are callback-driven and cannot be known without Phase 2.

| Topic | Producer | Consumers | Configured | Ideal | Reasoning |
|---|---|---|---|---|---|
| `/scan` | urg_node | slam, lidar_front_wall, wall_distance, swept_clearance, obstacle_clearance, e-stop, viz | 40 Hz | **40 Hz** | native rate; e-stop sensor |
| `/sensors/imu/raw` | vesc_driver | ekf local, mpc_corr | 50 Hz | **50 Hz** | estimator input |
| `/sensors/core` | vesc_driver | vesc_to_odom, logger | 50 Hz | **50 Hz** | estimator input |
| `/odom` | vesc_to_odom | ekf local | 50 Hz | **50 Hz** | estimator input |
| `/odometry/filtered` | ekf local | mpc_corr, wall_distance, lidar_front_wall, costmap | 50 Hz | 20–25 Hz by rule, **50 Hz in practice** | TF coupling — §4.2 |
| `odom->base_link` TF | ekf local | slam, costmap, perception TF | 50 Hz | **50 Hz** | same knob as above |
| `/camera/image_raw` | ZED | yolo, viz throttle | 30 Hz | = YOLO throughput | measure first |
| ZED depth | ZED | detection_3d, front_clearance, swept_clearance | ~25 Hz | = detection rate | consumed |
| ZED point cloud | ZED | **none** | 0 Hz (gated) | disabled | no flag available |
| `/camera/detections` | yolo | detection_3d, front_clearance | ? | = YOLO | callback |
| `/camera/detection_masks` | yolo | detection_3d, front_clearance | ? | = YOLO | Image type, real bandwidth |
| `/camera/image_annotated` | yolo | **viz throttle only** | ? | debug — gate it | no race consumer |
| `/perception/obstacles_2d` | obstacle_projector | mpc_corr, costmap | ? | = detection | callback |
| `/perception/front_distance` | front_clearance | mpc_corr, behavior | ? | = detection | callback |
| `/perception/swept_clearance` | swept_clearance | **none** | 20 Hz | **disabled** | never consumed, never run on the car |
| `/perception/wall_distance` | wall_distance | mpc_corr | timer | = MPC 10 Hz | — |
| `/obstacle_clearance` | obstacle_clearance | logger | 40 Hz | 10 Hz | logging only |
| `/drive` | mpc_corr | ackermann_mux | 10 Hz | **10 Hz** | controller rate; 5 Hz mux floor |
| `/ackermann_drive` | ackermann_mux | ackermann_to_vesc, swept_clearance | 10 Hz | 10 Hz | callback |
| `/commands/motor/speed` | ackermann_to_vesc | vesc_driver | 10 Hz | 10 Hz | callback |
| `/mpc/min_obstacle_distance(_forward)` | mpc_corr | **none** | 10 Hz | debug | "no consumer yet" per source |
| `/viz/*` | throttle container | foxglove_bridge | 5.5 / 3.3 Hz | unchanged | already throttled |
| costmap render | costmap_renderer | viz | 2 Hz | unchanged | already low |
| `/costmap/boundaries` | costmap_boundary | mpc_corr | 20 Hz | 10 Hz | MPC reads at 10 Hz |
| system/ekf observers | diagnostics | viz | 1 Hz | unchanged | already minimal |

---

## 7. Open questions carried into Phase 2

1. **What is YOLO's real throughput** on this Orin with the current model? This
   sets the ideal rate for the whole camera chain. `stack_params.yaml`'s
   `yolo_model` default moved to a `-seg.pt` segmentation model, which is
   slower than the box model.
2. ~~**What is `wall_distance_node`'s `tick_period`?**~~ **RESOLVED** by the
   2026-09-24 measurement: 10 Hz, matching the MPC.
3. **Does the ZED point cloud stay at 0 Hz** once `ros2 topic hz` subscribes to
   it? Measuring a lazy publisher *creates* the load being measured — this must
   be noted in Phase 2, not silently recorded as a real rate.
4. ~~**Is the global EKF actually at 50 Hz?**~~ **RESOLVED**:
   `ekf_global.yaml:86 frequency: 50.0` with `publish_tf: true` (it owns
   `map→odom`). Measured at 50.00 Hz.
5. **Do `swept_clearance` and `obstacle_clearance` really have no subscribers?**
   Phase 2's `ros2 topic info -v` (taken *before* the measuring subscription) is
   the authority here.
6. **Does `/camera/image_annotated` get published when nothing subscribes?**
   If `yolo_detector_node` publishes unconditionally, the encode cost is paid
   every frame for a debug topic.
7. ~~**ZED 2i hardware running under a `camera_model: 'zed2'` config.**~~
   **RESOLVED** in the 2026-09-24 run: the SDK detects the real model from the
   device, logs `Camera model does not match user parameter ... modify ... to
   'zed2i'`, and proceeds as `ZED 2i`. Since `zed2.yaml` and `zed2i.yaml`
   differ only in `camera_model`/`camera_name`, nothing functional is wrong.
   Worth a one-line fix to silence the warning, but it is **not** a rate issue
   and must not be bundled into this pass.
