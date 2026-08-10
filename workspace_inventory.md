# F1TENTH Workspace Inventory

> **⚠ SUPERSEDED (2026-08-10).** This is a point-in-time snapshot from 2026-07-18 --
> kept for historical value, not maintained. A later full-stack dead-code/
> duplicate-variable/topic audit and its implementation pass (retiring
> `safety_stop_controller`, unifying the 4 obstacle-avoidance safety margins into
> `car_radius`/`obstacle_safety_margin_m`/`proximity_front_extra_margin_m`, deleting
> `tf_publisher.py`/`throttle_interpolator.py`/`auto_forward_node.py`, and more) have
> since made several of this document's findings stale, including (but not limited
> to) every `safety_stop_controller`/`enable_safety_stop` reference below. Do not
> treat anything here as current fact -- cross-check against the live code
> (`stack_params.yaml`, `README.md`) before acting on any specific claim.

**Generated:** 2026-07-18. Read-only audit pass — nothing in this document reflects any code changes; nothing in the workspace was modified while producing it.

**Scope:** every ROS 2 package under `src/`, organized as: (1) core `f1tenth_*` packages, (2) `f1tenth_external/` + `f1tenth_hardware/vesc` git submodules (with git archaeology vs. upstream), (3) a cross-workspace topic-wiring table, (4) a consolidated "Flagged for review" section.

**Root-level check requested up front:** in `f1tenth_bringup/launch/stack_bringup.launch.py`, `mpc_bringup = include('f1tenth_control', 'mpc_launch.py')` is built (line 115) but **is not added to the final `return LaunchDescription([...])` list** (line 169+) — confirmed by direct read, matches the file's own comment ("intentionally never added to the returned actions below"). It is also absent from `f1tenth_bringup/config/components.yaml` / `component_supervisor_node.py`. **Current state, unchanged: `mpc_launch.py` never runs in either automatic bring-up path.** The BT/Nav2 path (`f1tenth_behavior`) is what actually drives the mux by default (`use_behavior_tree: true`).

---

## Table of contents

1. [Core stack packages](#1-core-stack-packages)
   - [1.1 f1tenth_hardware](#11-f1tenth_hardware-wrapper)
   - [1.2 f1tenth_control](#12-f1tenth_control-wrapper--mpc_controller--safety_stop_controller)
   - [1.3 f1tenth_perception](#13-f1tenth_perception)
   - [1.4 f1tenth_behavior](#14-f1tenth_behavior)
   - [1.5 f1tenth_navigation](#15-f1tenth_navigation)
   - [1.6 f1tenth_bringup](#16-f1tenth_bringup)
   - [1.7 f1tenth_diagnostics](#17-f1tenth_diagnostics)
   - [1.8 f1tenth_localization](#18-f1tenth_localization)
   - [1.9 f1tenth_description](#19-f1tenth_description)
   - [1.10 f1tenth_intelligence/llm](#110-f1tenth_intelligencellm)
   - [1.11 f1tenth_params](#111-f1tenth_params)
   - [1.12 f1tenth_messages](#112-f1tenth_messages)
   - [1.13 f1tenth_sim](#113-f1tenth_sim)
   - [1.14 f1tenth_more](#114-f1tenth_more-top-level-metapackage)
2. [External submodules](#2-external-submodules)
   - [2.1 vesc](#21-vesc-srcf1tenth_hardwarevesc)
   - [2.2 ackermann_mux](#22-ackermann_mux)
   - [2.3 teleop_tools](#23-teleop_tools)
   - [2.4 zed_ros2_wrapper](#24-zed_ros2_wrapper)
   - [2.5 zed-ros2-interfaces](#25-zed-ros2-interfaces-zed_msgs)
   - [2.6 transport_drivers](#26-transport_drivers)
3. [Cross-workspace topic wiring](#3-cross-workspace-topic-wiring)
4. [Flagged for review](#4-flagged-for-review)

---

## 1. Core stack packages

### 1.1 `f1tenth_hardware` (wrapper)

**Purpose:** `ament_python` metapackage (no own code) owning `launch/vesc_launch.py`, which orchestrates the vendored VESC stack (§2.1) plus the battery pre-flight gate and calibration sequencing.

**Nodes:** none of its own — everything it launches lives in the `vesc` submodule (§2.1) or `f1tenth_diagnostics`.

**Launch files:**

| File | Contents | Referenced by |
|---|---|---|
| `f1tenth_hardware/f1tenth_hardware/launch/vesc_launch.py` | Battery pre-flight gate (`f1tenth_diagnostics/battery_voltage_check_node`) → calibration:=true/false branch (driver-group v1/v2 via `sensor_covariance_calibration_node`) → `ackermann_to_vesc_node`, `vesc_to_odom_node_backup` (as name `vesc_to_odom_node`), `vesc_driver_node`, 2× static TF → conditionally includes `f1tenth_localization/ekf_launch.py` + `f1tenth_navigation/nav2_bringup.launch.py` | `stack_bringup.launch.py` (unconditional, first), `components.yaml`'s `hardware` + `calibrate_hardware` |

**Params:** `vesc_config` (path→`f1tenth_bringup/config/vesc.yaml`), `calibration` (false), `calibration_duration_sec` (60.0), `release_downstream` (true), `min_battery_voltage` (10.8) — all sourced from `stack_params.yaml`.

**Flag:** `f1tenth_hardware/package.xml` declares `<exec_depend>vesc_tuning</exec_depend>` — **no `vesc_tuning` package exists anywhere in `src/`** (only a stray `0001-vesc_tuning-update-speed-calibration-gains-from-late.patch` file at the workspace root, presumably belonging to the `vesc-tuner-test` branch found dangling in the `vesc` submodule's git history — see §2.1). Dead/broken dependency declaration.

---

### 1.2 `f1tenth_control` (wrapper) + `mpc_controller` + `safety_stop_controller`

**Purpose:** `f1tenth_control` (ament_python metapackage, no own code) owns `ackermann_mux_launch.py`, `joy_launch.py`, `mpc_launch.py`. `mpc_controller`: 7 alternative MPC/PID-style Ackermann controllers (`package.xml` description is still a TODO placeholder). `safety_stop_controller`: reactive supervisor — watches `Detection3DArray`, on obstacle-in-corridor calls `SetParameters` on the live MPC node to zero `v_ref` (doesn't publish `/drive` itself).

**Nodes:**

| File | Executable | Wired into a launch file? |
|---|---|---|
| `mpc_controller/mpc_controller/mpc_node.py` | `mpc_node` | **No — orphaned.** |
| `mpc_controller/mpc_controller/trajectory_mpc_node.py` | `trajectory_mpc_node` | **No — orphaned.** |
| `mpc_controller/mpc_controller/kinematic_mpc_node.py` | `kinematic_mpc_node` | **No — orphaned.** |
| `mpc_controller/mpc_controller/frenet_mpc_node.py` | `frenet_mpc_node` | **No — orphaned.** |
| `mpc_controller/mpc_controller/andre_mpc_node.py` (node name `andre_mpc_controller`) | `andre_mpc_node` | Referenced by `mpc_launch.py` only — and that file itself is built-but-unreturned (see root check above). Not reachable from automatic bring-up. |
| `mpc_controller/mpc_controller/andre_mpc_node_linear.py` (same node name `andre_mpc_controller`) | `andre_mpc_node_linear` | **No — orphaned.** |
| `mpc_controller/mpc_controller/andre_mpc_opt_node.py` (same node name `andre_mpc_controller`) | `andre_mpc_opt_node` | **No — orphaned.** |
| `mpc_controller/mpc_controller/track_mpc_opt_node.py` (node name `track_mpc_controller`) | `track_mpc_controller` | **No — orphaned.** |
| `safety_stop_controller/safety_stop_controller/simple_stop_controller_node.py` | `simple_stop_controller_node` | Yes — `safety_stop.launch.py`, conditionally included (`enable_safety_stop`, default `false`) by both `stack_bringup.launch.py` and `component_supervisor_node.py`'s `control` component. |

**6 of 7 `mpc_controller` executables are orphaned**; the 7th (`andre_mpc_node`) is referenced by a launch file that is itself dead code.

**Topics** (see §3 for the full cross-workspace table); notable:
- `ackermann_mux` (external, launched here): sub `safety_stop`(200)/`drive`(10)/`teleop`(100) per `mux.yaml`; pub `ackermann_cmd`→remapped→`ackermann_drive`.
- `simple_stop_controller_node`: sub `/camera/detections_3d` (hardcoded in `safety_stop.launch.py`'s `parameters=[{...}]`, not a `DeclareLaunchArgument` despite matching the code default).

**Services:** `simple_stop_controller_node` is a `SetParameters` **client** targeting `{mpc_node_name}/set_parameters`, default `mpc_node_name` = `/andre_mpc_opt_controller` (from `stack_params.yaml`). **This does not match any node that actually registers itself** — `andre_mpc_node.py` (the one launchable-in-principle MPC node) registers as `andre_mpc_controller`, not `andre_mpc_opt_controller` (that name belongs only to the orphaned `andre_mpc_opt_node.py`). Even if `mpc_launch.py` were wired back in, `safety_stop_controller`'s default target would silently fail to find it.

**Launch files:**

| File | Contents | Referenced elsewhere? |
|---|---|---|
| `ackermann_mux_launch.py` | `Node(ackermann_mux, ackermann_mux, remappings=[('ackermann_cmd','ackermann_drive')])` | `stack_bringup.launch.py`, `components.yaml`'s `control` |
| `joy_launch.py` | `joy_node` + `joy_teleop` | Intentionally standalone-only (own docstring) — not a bug. |
| `mpc_launch.py` | `andre_mpc_node` (named `andre_mpc_controller`) + crash-`Shutdown()` handler | Built-but-unreturned in `stack_bringup.launch.py`; absent from `components.yaml`. |
| `safety_stop_controller/launch/safety_stop.launch.py` | `simple_stop_controller_node` | `stack_bringup.launch.py` + `components.yaml`, both gated on `enable_safety_stop` |

**Params:** `mux_config`, `joy_config`, `mpc_node_name` (`/andre_mpc_opt_controller` — see mismatch above), `safety_forward_v_ref` (0.5), `safety_stop_distance` (1.0), `safety_corridor_half_width` (0.4), `safety_corridor_half_height` (0.4), `safety_clear_frames_required` (3) — all from `stack_params.yaml`. `mpc_controller` nodes' own tunable gains (`qn`,`qv`,`qalpha`,`qddelta`,`alat_max`,`a_min/max`,`v_min/max`,`v_ref`,`sine_amp`,`sine_period`, etc.) are declared with code defaults in every one of the 7 node files, but **`mpc_launch.py` passes no `parameters=[...]` at all** — even if it were wired back in, none of these gains would be externally configurable, only the hardcoded in-code defaults would ever be live.

---

### 1.3 `f1tenth_perception`

**Purpose (package.xml):** "Centralized perception… publishes `vision_msgs/Detection2DArray` on `/yolo/2D_detections`." **This description is stale** — actual code and launch-time config both use `/camera/detections`; `/yolo/2D_detections` appears nowhere in the codebase.

**Nodes:**

| File | Executable | Wired in? |
|---|---|---|
| `f1tenth_perception/yolo_detector_node.py` | `yolo_detector_node` | Yes — `detection.launch.py` |
| `f1tenth_perception/detection_3d_node.py` | `detection_3d_node` | Yes — `detection.launch.py`, conditionally (`camera_source=='zed'`, the default) |

No orphaned nodes of its own; external nodes it launches (`zed_wrapper`, `v4l2_camera_node`, `urg_node`) are covered under their owning packages/submodules.

**Launch files:**

| File | Contents | Referenced by |
|---|---|---|
| `camera.launch.py` | Branches on `camera_source` (plain value from `stack_params.yaml`): ZED → static TF `base_link→zed2_camera_link` + `GroupAction(SetRemap×2, Include(zed_wrapper/zed_camera.launch.py))`; webcam → `v4l2_camera_node` | `stack_bringup.launch.py`, `components.yaml`'s `perception`, `perception.launch.py` |
| `detection.launch.py` | `yolo_detector_node` + conditional `detection_3d_node` | same 3 places |
| `lidar.launch.py` | `urg_node`, gated by `use_lidar` (default **false**) | `f1tenth_navigation/nav2_bringup.launch.py` (**no `use_lidar` override passed — see §4 flag**) |
| `perception.launch.py` | Standalone: `Include(camera.launch.py)` + a second, hand-duplicated inline `urg_node` block (hardcoded IP, own docstring self-flags this as "a pre-existing, separate duplication… not reconciled here") + `Include(detection.launch.py)` | **Not referenced anywhere else** — a standalone test entry point, separate from both main bring-up paths |

**Params:** `confidence_threshold` (0.3 — **only reaches `detection_3d_node`, not `yolo_detector_node`**, despite reading as a shared/global default), `yolo_device` (`cuda`), `yolo_model` (`yolo26s.engine`), `use_lidar` (`false`), `sensors_config`. `detection_3d_node`'s own params `sync_slop`, `sync_queue_size`, `default_depth_extent`, `marker_lifetime`, `center_fraction`, `output_frame`, `label_scale`, `label_z_offset` are declared with code defaults and **never overridden by any launch file** — not unread (the node itself reads them), but never externally configured either.

---

### 1.4 `f1tenth_behavior`

**Purpose (package.xml):** py_trees-based reactive safety-stop (obstacle-corridor check onto the mux) + Nav2 goal-pose navigation (`NavigateThroughPoses`, single `/goal_pose`-driven, no static waypoint list). Mutually exclusive with `mpc_controller`'s direct drive path, guarded by `use_behavior_tree`. Consistent with `git status` showing `config/waypoints.yaml` staged deleted and 3 new BT-behaviour files staged — package is mid-refactor away from static waypoints.

**Nodes:**

| File | Executable | Wired in? |
|---|---|---|
| `behavior_executor_node.py` | `behavior_executor_node` | Yes — `behavior_bringup.launch.py` |
| `twist_to_ackermann_node.py` | `twist_to_ackermann_node` | Yes — same |
| `wait_for_trigger_service_node.py` (instance named `wait_for_nav2_ready`) | `wait_for_trigger_service_node` | Yes — same |

No orphaned nodes. (`behaviours/` submodule — `has_goal_pose.py`, `is_obstacle_detected.py`, `navigate_through_poses_client.py`, `stop.py` — are py_trees `Behaviour` classes instantiated inside `behavior_executor_node.py`, not separate executables.)

**Services/actions:** `NavigateThroughPosesClient` behaviour is an **action client** for `navigate_through_poses` (`nav2_msgs/action/NavigateThroughPoses`). `wait_for_trigger_service_node` is a **service client** for `std_srvs/srv/Trigger`, target set at launch to `/lifecycle_manager_navigation/is_active`.

**Launch files:**

| File | Contents | Referenced by |
|---|---|---|
| `behavior_bringup.launch.py` | Always starts `wait_for_nav2_ready`; only on its successful exit (`OnProcessExit`, returncode 0) does a `RegisterEventHandler` add `behavior_executor_node` + `twist_to_ackermann_node` to the tree | `stack_bringup.launch.py` (gated on `use_behavior_tree`, default `true`), `components.yaml`'s `behavior` |

**Params:** `nav2_readiness_service` (`/lifecycle_manager_navigation/is_active`), `nav2_readiness_timeout_sec` (90.0), `bt_setup_timeout_sec` (90.0, yaml; code fallback 60.0). `wait_for_trigger_service_node`'s own `poll_interval_sec` (code default 0.5) is **never overridden** by the launch file. `twist_to_ackermann_node`'s `wheelbase`/`min_steering_angle`/`max_steering_angle`/`frame_id` are loaded from `config/twist_to_ackermann.yaml` (values identical to the code defaults — a real read-path, functionally a no-op override). `behavior_executor_node`'s `bt_loop_duration_ms` (code default 100) is declared and read in the same file, never exposed via any launch arg/yaml.

---

### 1.5 `f1tenth_navigation`

**Purpose:** Static map server + full Nav2 stack (planner/controller/behavior servers, `bt_navigator`, one shared lifecycle manager). (`setup.py`'s own `description=` is stale — doesn't mention Nav2 at all, unlike `package.xml`'s.)

**Nodes:**

| File | Executable | Wired in? |
|---|---|---|
| `odom_to_tf_node.py` | `odom_tf_broadcaster` | **No — orphaned.** Only reachable from the dead `map_server_launch_old.py` (below). |

Everything else launched by this package is external (`nav2_map_server`, `nav2_controller`, `nav2_planner`, `nav2_behaviors`, `nav2_bt_navigator`, `nav2_lifecycle_manager`, `tf2_ros/static_transform_publisher`).

**Launch files:**

| File | Contents | Referenced by |
|---|---|---|
| `nav2_bringup.launch.py` | `map_server`, static `odom→base_link` identity TF, `Include(f1tenth_perception/lidar.launch.py)` **with no `launch_arguments` passed** (see §4 flag), `controller_server` (remaps `cmd_vel`→`cmd_vel_nav`), `planner_server`, `behavior_server`, `bt_navigator`, `lifecycle_manager_navigation` | `stack_bringup.launch.py` §6 (gated `enable_nav2`), `components.yaml`'s `navigation` |
| `map_server_launch_old.py` | Own `map_server` + own `lifecycle_manager_map` + hardcoded `map_to_odom_tf` + conditional `odom_tf_broadcaster_node` | **Dead.** Own docstring: "DEPRECATED… No longer included from anywhere; kept here for reference only." Confirmed unreferenced except in `nav2_bringup.launch.py`'s own docstring prose. |

**Params:** `map` (`maps/square_100m.yaml`, resolved against `f1tenth_navigation` share), `use_sim_time` (`false`, deliberately local/not in `stack_params.yaml`), `autostart` (`true`). `map_server_launch_old.py` declares its own dead copies (hardcoded `track_bw.yaml` default) — unreachable.

**Also stale:** `f1tenth_navigation/maps/README.md` documents an outdated TF chain (static `map→odom` + `vesc_to_odom_node`-owned `odom→base_link`) that no longer matches current reality (EKF dynamically owns `map→odom`; `odom→base_link` is now the fixed identity transform).

---

### 1.6 `f1tenth_bringup`

**Purpose:** Owns the current main real-hardware entry point (`stack_bringup.launch.py`) **and** a parallel, per-component-restartable supervisor path (`supervisor_bringup.launch.py` + `component_supervisor_node.py` + `config/components.yaml`). Also owns a few standalone/legacy nodes and most of the shared hardware config (`ekf.yaml`, `vesc.yaml`, `mux.yaml`, `joy_teleop.yaml`, `sensors.yaml`).

**Nodes:**

| File | Executable | Wired in? |
|---|---|---|
| `component_supervisor_node.py` | `component_supervisor_node` | Yes — `supervisor_bringup.launch.py` |
| `stack_startup_sequence.py` | `stack_startup_sequence` | Yes, but only from its own standalone `startup_sequence_launch.py`, which nothing else references — opt-in utility, not part of either bring-up tree. |
| `tf_publisher.py` | `tf_publisher` | **No — orphaned.** Publishes placeholder-looking static-ish TFs (`base_link→laser`, `base_link→odom`) that would conflict with the real `sensor_tf_launch.py`/EKF TF ownership if it were ever launched. Looks like early-stack scaffolding. |
| `throttle_interpolator.py` | `throttle_interpolator` | **No — orphaned.** Has a matching (also-orphaned) `throttle_interpolator:` config block in `vesc.yaml`, but no `Node(...)` anywhere references it, and it's not in `components.yaml`. |

**`component_supervisor_node.py` services:**

| Service | Type | Direction |
|---|---|---|
| `restart_component` | `f1tenth_messages/srv/RestartComponent` | server |
| `~/control_component` → `/component_supervisor_node/control_component` | `f1tenth_messages/srv/ComponentControl` | server |

No in-repo client calls either service — both are meant for manual `ros2 service call` use.

**`components.yaml` registry** (confirmed current contents):

| Component | Launches | Auto-start rule |
|---|---|---|
| `hardware` | `f1tenth_hardware/vesc_launch.py` (`calibration:='true'` — **note:** this now runs calibration on every hardware start, a recent edit) | always |
| `calibrate_hardware` | `f1tenth_hardware/vesc_launch.py` (`calibration:='true' release_downstream:='false'`) | never — on-demand only |
| `localization` | `f1tenth_localization/localization_launch.py` | always |
| `perception` | `f1tenth_perception/camera.launch.py` + `detection.launch.py` | always |
| `control` | `f1tenth_control/ackermann_mux_launch.py` (+ `safety_stop.launch.py` iff `enable_safety_stop`) | always |
| `navigation` | `f1tenth_navigation/nav2_bringup.launch.py` | iff `enable_nav2` |
| `behavior` | `f1tenth_behavior/behavior_bringup.launch.py` | iff `use_behavior_tree` |
| `diagnostics` | `f1tenth_diagnostics/system_observer.launch.py` | always |
| `intelligence` | `llm/llm.launch.py` | iff `enable_llm` |
| `dev_tools` | `f1tenth_bringup/foxglove_bridge_launch.py` | always |

**`components.yaml` itself is currently untracked in git** — the whole supervisor registry has not been committed.

**Launch files:**

| File | Role | Referenced by |
|---|---|---|
| `stack_bringup.launch.py` | **Current main entry point.** 8-section thin orchestrator (Stack-Wide args → Hardware → Localization/TF → Perception → Command/Control → Autonomy → Diagnostics/Intelligence → Dev Tools). | top-level only |
| `supervisor_bringup.launch.py` | Parallel entry point — starts exactly `component_supervisor_node` | top-level only, coexists with `stack_bringup.launch.py` by design ("untouched as a working fallback") |
| `startup_sequence_launch.py` | Starts `stack_startup_sequence` | nothing else references it |
| `foxglove_bridge_launch.py` | Single `foxglove_bridge` Node, extracted from `stack_bringup.launch.py`'s own inline copy | `components.yaml`'s `dev_tools` only — `stack_bringup.launch.py` keeps a separate, deliberately-duplicate inline `Node()` (by design, per its own docstring) |

**Stale-name note:** many docstrings across the repo (including inside `component_supervisor_node.py` itself) still call this file `stack_bringup_launch.py` (old underscore name) even though the actual current file is `stack_bringup.launch.py` — cosmetic, but worth a cleanup pass.

**Config artifacts:** 7 untracked `config/vesc.yaml.bak.<timestamp>` files — real runtime output from `sensor_covariance_calibration_node` during actual calibration runs, not source-controlled config; repo clutter.

---

### 1.7 `f1tenth_diagnostics`

**Purpose:** Self-contained calibration/diagnostic tooling — safe to run standalone, no modifying live params / publishing control topics except where a node is explicitly a calibration writer.

**Nodes:**

| File | Executable | Wired in? |
|---|---|---|
| `gyro_bias_calibration_node.py` | `gyro_bias_calibration_node` | Yes — own `gyro_bias_calibration.launch.py` (standalone/manual) |
| `sensor_covariance_calibration_node.py` | `sensor_covariance_calibration_node` | Yes — own `sensor_covariance_calibration.launch.py` (standalone) **and** `f1tenth_hardware/vesc_launch.py`'s `calibration:=true` path |
| `battery_voltage_check_node.py` | `battery_voltage_check_node` | Yes — but **only** from `vesc_launch.py`; no standalone launch file of its own |
| `system_observer_node.py` | `system_observer_node` | Yes — `system_observer.launch.py`, included by `stack_bringup.launch.py` §7 (unconditional) and `components.yaml`'s `diagnostics` |

**Topics:** `system_observer_node` publishes `/diagnostics/system_status` (`f1tenth_messages/msg/SystemStatus`) — **no subscriber found anywhere in the workspace.**

**Params:** all sourced from `stack_params.yaml` (`imu_topic`, `gyro_sample_duration_sec`, `min_samples`, `odom_topic`, `covariance_sample_duration_sec`, `publish_rate_hz`). Two flagged mismatches:
- `gyro_bias_calibration_node.py`'s in-code default for `imu_topic` is `/zed/zed_node/imu/data`; the launch file overrides it to `/sensors/imu/raw` — the two defaults disagree, masked in practice by the launch override.
- `min_samples`: code default 300, launch overrides to 150.
- `stack_params.yaml` has two separately-editable keys that both ultimately feed the *same* node parameter (`sensor_covariance_calibration_node`'s `sample_duration_sec`) depending on launch path: `calibration_duration_sec` (60.0, via `vesc_launch.py`) and `covariance_sample_duration_sec` (60.0, via the standalone launch file) — currently in agreement, but two sources of truth for one value.

---

### 1.8 `f1tenth_localization`

**Purpose:** EKF / state estimation, owns `robot_localization` EKF bring-up.

**Nodes:**

| File | Executable | Wired in? |
|---|---|---|
| `raw_odom_map_tf_node.py` | `raw_odom_map_tf_node` | Yes — `localization_launch.py` and directly in `stack_bringup.launch.py` §3, both gated on `localization_source=='raw_odom'` (**the default**). This is the node actually running by default, not the EKF. |

**Topics:** `raw_odom_map_tf_node` subscribes `/odom`, broadcasts TF `map→odom` mirroring `/odom`'s pose verbatim (unfiltered). No declared parameters at all. `ekf_launch.py`'s external `ekf_filter_node`: fuses `odom0: /odom` (x/y/yaw/vx) + `imu0: /sensors/imu/raw` (vyaw only), publishes **`/odometry/filtered`** + dynamic `map→odom` TF (`world_frame: map`) — confirmed via `ekf.yaml`'s own comment. **`/odometry/filtered` has no subscriber found anywhere in the workspace.**

**Launch files:**

| File | Contents | Referenced by |
|---|---|---|
| `ekf_launch.py` | `ekf_filter_node` | `localization_launch.py` + `stack_bringup.launch.py` §3, both gated `localization_source=='ekf'` |
| `localization_launch.py` | Conditionally includes `ekf_launch.py` OR inlines `raw_odom_map_tf_node` per `localization_source`; always includes `f1tenth_description/sensor_tf_launch.py` | `components.yaml`'s `localization` |

**Params:** `ekf_config` (path → `f1tenth_bringup/config/ekf.yaml`, resolved against `f1tenth_bringup`'s share dir even though this launch file lives in `f1tenth_localization`).

---

### 1.9 `f1tenth_description`

**Purpose:** URDF/xacro + meshes; owns a `robot_state_publisher`-only launch reused by real bring-up and sim.

**Nodes:** none of its own (`setup.py`'s `console_scripts` is empty) — only launches external `robot_state_publisher` / `tf2_ros static_transform_publisher`.

**Launch files:**

| File | Contents | Referenced by |
|---|---|---|
| `description_launch.py` | `robot_state_publisher` with `roboracer.urdf.xacro` | **Only `f1tenth_sim/sim_bringup_launch.py`.** **Not included anywhere in the real-hardware `stack_bringup.launch.py` tree** — confirmed by direct read: `stack_bringup.launch.py` only includes `sensor_tf_launch.py`, never `description_launch.py`. **On the real car, via the default bring-up path, `robot_state_publisher`/the URDF never runs — no `/robot_description`, no full robot TF tree.** |
| `sensor_tf_launch.py` | Single static `base_link→laser` transform | `localization_launch.py` (always) + directly in `stack_bringup.launch.py` §3 (always) |

**Params:** `use_sim_time` (`false`, deliberately local), `use_sim` (from `stack_params.yaml`, false), `enable_sensors` (true), `control_config` (`''`).

---

### 1.10 `f1tenth_intelligence/llm`

**Purpose:** LLM-based MPC tuner + the `llama-server` bring-up hosting it. ROS package name is `llm` (not `f1tenth_intelligence`).

**Nodes:**

| File | Executable | Wired in? |
|---|---|---|
| `llm_mpc_tuner_node.py` | `llm_mpc_tuner_node` | Indirectly — `config/interrogations.yaml`'s `mpc_tuner.executable` key is read by `llm.launch.py`'s `OpaqueFunction` to build the `Node(...)` at launch time (no static string literal anywhere). Reachable via `stack_bringup.launch.py` §7 (iff `enable_llm`) and `components.yaml`'s `intelligence`. |

**Topics:** subscribes `odom_topic` (default `/odom`) for speed feedback; publishes `stop_topic` (default `/teleop`) only on a manual `'p'` STOP keypress. Otherwise talks HTTP to `llama-server` and shells out to `ros2 param dump`/`ros2 param set <target_node>` via `subprocess` (regex-parses the YAML text output) rather than using `SetParameters`/`GetParameters` service clients directly — fragile but functional.

**Naming discrepancy flagged:** `stack_params.yaml`'s `mpc_node_name` (`/andre_mpc_opt_controller`, documented as used by "safety-stop, startup sequence") is **not** used by this node — `llm_mpc_tuner_node`'s own `target_node` param defaults to `/andre_mpc_controller` (no `_opt`), hardcoded independently (also duplicated in `config/mpc_tuner_params.yaml`), not threaded through `f1tenth_params` at all. Two different "the MPC node" name conventions coexist.

**Launch files:** `llm.launch.py` — `ExecuteProcess` for the `llama-server` binary (hardcoded machine-specific absolute paths: `/scratch/fabiocar/llama.cpp/build/bin/llama-server`, cwd `/scratch/fabiocar/llama.cpp` — not portable to another machine) gated on `start_server`, plus the dynamically-resolved interrogation node. Referenced by `stack_bringup.launch.py` §7 (iff `enable_llm`) and `components.yaml`'s `intelligence`.

**Params:** `start_server` (true), `model` (`qwen_mpc_pruned`), `interrogation` (`mpc_tuner`) — all from `stack_params.yaml`.

---

### 1.11 `f1tenth_params`

**Purpose:** Dependency-free shared source of launch-parameter defaults for the whole workspace — `config/stack_params.yaml` + `param_defaults.py` (`get_default`/`get_value`/`get_path_default`). Deliberately has **no** `exec_depend` on any other `f1tenth_*` package, to avoid a build-order cycle (`f1tenth_bringup`, which nearly everything is included from, itself depends on this).

**Consumed by:** `f1tenth_bringup`, `f1tenth_localization`, `f1tenth_navigation`, `f1tenth_description`, `f1tenth_diagnostics`, `f1tenth_intelligence/llm`, `f1tenth_perception`, `f1tenth_behavior`, `f1tenth_control`, `safety_stop_controller`, `f1tenth_hardware`. **Not consumed by `f1tenth_sim`** — confirmed no dependency and `sim_bringup_launch.py` hardcodes all its own defaults independently.

**The 6 stack-wide branching args** (confirmed: NOT `DeclareLaunchArgument` anywhere — read only via `get_value()`, so CLI overrides are silently no-ops):

| Name | Default | Meaning |
|---|---|---|
| `camera_source` | `zed` | `'zed'` \| `'webcam'` |
| `localization_source` | `raw_odom` | `'ekf'` \| `'raw_odom'` |
| `enable_safety_stop` | `false` | opt-in corridor-stop layer |
| `enable_llm` | `true` *(changed from `false`)* | opt-in LLM stack |
| `use_behavior_tree` | `true` | BT+Nav2 vs. `mpc_launch.py` |
| `enable_nav2` | `true` | Nav2 stack on/off |

All other (non-branching) keys — `vesc_config`, `calibration`, `calibration_duration_sec`, `min_battery_voltage`, `release_downstream`, `ekf_config`, `confidence_threshold`, `yolo_device`, `yolo_model`, `use_lidar` *(now `false`)*, `sensors_config`, `mux_config`, `joy_config`, `mpc_node_name` *(now `/andre_mpc_opt_controller`)*, `safety_forward_v_ref`, `safety_stop_distance`, `safety_corridor_half_width`/`half_height` *(now 0.4/0.4)*, `safety_clear_frames_required`, `map`, `autostart`, `nav2_readiness_service`, `nav2_readiness_timeout_sec` *(now 90.0)*, `bt_setup_timeout_sec` *(now 90.0)*, `imu_topic`, `gyro_sample_duration_sec`, `min_samples`, `odom_topic`, `covariance_sample_duration_sec`, `publish_rate_hz`, `start_server`, `model`, `interrogation`, `use_sim`, `enable_sensors`, `control_config`, `components_config`, `restart_timeout_sec`, `log_dir`, `watchdog_period_sec`, `max_auto_restarts`, `restart_budget_window_sec`, `startup_delay`, `right_duration`, `left_duration`, `center_duration`, `max_steering_angle`, `startup_command_topic` — every one confirmed read somewhere (per-key grep, no unread keys found). `use_sim_time` is deliberately **not** here (real-vs-sim need different defaults, documented in the yaml's own header).

**Minor drift note:** the "Simulation" section header groups `use_sim`/`enable_sensors`/`control_config` as if for `f1tenth_sim`, but they're actually consumed by `f1tenth_description`, not `f1tenth_sim` itself.

---

### 1.12 `f1tenth_messages`

**Purpose:** `ament_cmake` interfaces-only package holding all custom `.msg`/`.srv` for the workspace — confirmed clean merge of two former packages, `f1tenth_bringup_msgs` + `f1tenth_diagnostics_msgs` (neither exists anywhere anymore).

**Definitions and consumers (whole-tree grep):**

| File | Consumers |
|---|---|
| `msg/SystemStatus.msg` | `f1tenth_diagnostics/system_observer_node.py` only (publisher). **No subscriber anywhere.** |
| `srv/RestartComponent.srv` | `f1tenth_bringup/component_supervisor_node.py` only (server). No client anywhere. |
| `srv/ComponentControl.srv` | Same file (server). No client anywhere. |

Only 2 consumer packages workspace-wide (`f1tenth_diagnostics`, `f1tenth_bringup`), matching their `package.xml` `exec_depend` entries exactly.

---

### 1.13 `f1tenth_sim`

**Purpose:** Gazebo Fortress (ignition) simulation — spawns the `f1tenth_description` model, bridges sim sensors to real-stack topic names, drives via `ros2_control` + Ackermann controller. No VESC/ZED SDK/urg_node.

**Nodes:**

| File | Executable | Wired in? |
|---|---|---|
| `drive_bridge.py` (node name `f1tenth_sim_drive_bridge`) | `drive_bridge` | Yes — `sim_bringup_launch.py` only |

**Topics:** subscribes `drive_topic` (`/drive`) → publishes `reference_topic` (`/ackermann_steering_controller/reference`, `TwistStamped`). Subscribes `controller_odom_topic` (`/ackermann_steering_controller/odometry`) → republishes verbatim on `odom_topic` (`/odom`), so the real stack's `ekf.yaml` (`odom0: odom`) can be reused unmodified in sim.

**Launch files:** `sim_bringup_launch.py` — Gazebo Fortress, `description_launch.py` include, `ros_ign_bridge`, spawn (`TimerAction` @4s), 2× controller spawners (`TimerAction` @8s), `drive_bridge`, `foxglove_bridge`, `slam_toolbox` (reusing `f1tenth_bringup/config/f1tenth_online_async.yaml`), `ekf_node` (reusing `f1tenth_bringup/config/ekf.yaml` verbatim + `use_sim_time` override). Top-level entry point, entirely separate from both real-hardware paths.

**Params:** only `use_sim_time` is a real `DeclareLaunchArgument` (hardcoded `'true'`, local, matching `stack_params.yaml`'s documented rationale for excluding it). Everything else is a plain Node-literal.

---

### 1.14 `f1tenth_more` (top-level metapackage)

**Purpose:** Aggregates first-party packages. No code (`setup.py`'s `packages=[]`, empty `console_scripts`).

**Dependency list** (`package.xml`, 9 entries): `f1tenth_bringup`, `f1tenth_description`, `f1tenth_hardware`, `f1tenth_perception`, `f1tenth_localization`, `f1tenth_navigation`, `f1tenth_control`, `llm`, `f1tenth_params`.

**Flag:** notably absent — `f1tenth_diagnostics`, `f1tenth_behavior`, `f1tenth_messages`, `f1tenth_sim`, `safety_stop_controller`, and everything under `f1tenth_external/`. Most are transitively pulled in via `f1tenth_bringup`'s own dependency list, so this doesn't break a build, but the metapackage's manifest doesn't fully live up to its "aggregates all first-party packages" description.

---

## 2. External submodules

Per `.gitmodules`, 6 submodules total. **Structural note:** `vesc` is declared with path `src/f1tenth_hardware/vesc` — it physically lives under `f1tenth_hardware/`, not under `f1tenth_external/` where the other 5 live, despite being the most heavily modified of all six.

### 2.1 `vesc` (`src/f1tenth_hardware/vesc`)

**Remote:** `origin` only → `https://github.com/sjckness/vesc.git` (personal fork). **Pinned SHA:** `d08fcfd5` — HEAD is **detached on a commit that exists on no branch, local or remote** (reachable only via the outer superproject's gitlink).

**⚠️ Working tree is dirty beyond the pin** — uncommitted local modifications on top of `d08fcfd` in 4 files: `vesc_ackermann/include/vesc_ackermann/vesc_to_odom_backup.hpp`, `vesc_ackermann/src/vesc_to_odom_backup.cpp`, `vesc_driver/include/vesc_driver/vesc_driver.hpp`, `vesc_driver/src/vesc_driver.cpp`. **The code on disk is not what the outer superproject's index records.**

#### Package: `vesc` (metapackage) — no code.

#### Package: `vesc_driver`
- **Nodes:** `vesc_driver_node` (component, referenced 3× in `vesc_launch.py`); `vesc_device_namer` (udev-helper CLI utility, plain executable — **not referenced by any launch file anywhere**).
- **Topics:** pub `sensors/core` (`VescStateStamped`), `sensors/imu` (`VescImuStamped`), `sensors/imu/raw` (`sensor_msgs/Imu`), `sensors/servo_position_command` (`Float64`, echo). Sub `commands/motor/{duty_cycle,current,brake,speed,position}`, `commands/servo/position` (all `Float64`). No outer remapping.
- **Services/actions:** none.
- **Launch files:** own `launch/vesc_driver_node.launch.py` — **not included anywhere in the outer workspace** (superseded by `vesc_launch.py`'s inline `Node()` construction).
- **Params overridden by outer `vesc.yaml`:** all limit params (duty/current/brake/speed/position/servo min/max), plus `speed_to_erpm_gain/offset`, `steering_angle_to_servo_gain/offset` (none of which exist in the submodule's own default file), and `gyro_bias_z`, `gyro_variance_x/y/z`, `accel_variance_x/y/z` (explicitly flagged in the yaml as placeholders).

#### Package: `vesc_ackermann`
- **Nodes:**
  - `ackermann_to_vesc_node` (referenced in outer `vesc_launch.py` + submodule's own `.launch.xml`).
  - `vesc_to_odom_node` — the **newer IMU+VESC Kalman-filter fusion node** (5-state KF: x,y,θ,v,ω). Referenced only in the submodule's own `.launch.xml`. **Not used anywhere in the outer workspace.**
  - `vesc_to_odom_node_backup` — "Unmodified backup… kept as a fallback" per its own source comment. **This is the one actually deployed**: `vesc_launch.py` launches executable `vesc_to_odom_node_backup` under **node name `vesc_to_odom_node`** (so `vesc.yaml`'s name-keyed param block still applies). **The naming is now inverted from what the source comments suggest** — the "backup"/"fallback" code is production; the non-backup code is the orphan.
- **Topics:** `ackermann_to_vesc_node`: sub `ackermann_cmd` (hardcoded, not a param) — outer-remapped to `ackermann_drive`. `vesc_to_odom_node_backup` (deployed): sub `sensors/core`, sub `commands/servo/position` (**per the uncommitted fix** — the committed/pinned version still subscribes the old `sensors/servo_position_command`) → pub `odom`.
- **Launch files:** `ackermann_to_vesc_node.launch.xml`, `vesc_to_odom_node.launch.xml` — neither included by the outer workspace; the latter has the backup variant present only as a **commented-out** block explicitly labeled "===== FALLBACK: original bicycle-model-only odometry node =====" — i.e. the vendored launch file's own comments assume the *opposite* of what's actually deployed.
- **Params overridden by outer `vesc.yaml`:** `wheelbase` (0.2→0.25, real car value), `publish_tf` (true→false, reassigned to the EKF), `vx_variance` (added, flagged in-yaml as an uncalibrated placeholder actively implicated in EKF divergence). `ackermann_to_vesc_node`'s gain/offset values are numerically identical between submodule default and outer override — pure centralization, no functional change there.

#### Package: `vesc_msgs`
- **Definitions:** `VescState.msg`, `VescStateStamped.msg` (consumed within the submodule + cross-package by `f1tenth_diagnostics/battery_voltage_check_node.py`), `VescImu.msg`, `VescImuStamped.msg` (published on `sensors/imu` — **no subscriber found anywhere**, in-tree or cross-package).

#### Git archaeology (vesc_driver + vesc_ackermann share the relevant commits)
- Real upstream fork chain (Triton-AI/f1tenth-derived, contributors Mailamaca/anscipione) up to `153998d` (merge PR #33, ros2_humble) — genuine, multi-hop fork lineage, not invented locally.
- `7169442` "first attempt of IMU+eRPM+servo with a kalman filter" — introduces the KF fusion odometry, splits off `vesc_to_odom_backup` as the preserved original.
- `d08fcfd` "nav2 not working, odometry not bad" (author `andreas`, `Co-Authored-By: Claude Sonnet 5`, 2026-07-13, **local, unpushed, on no branch**) — adds `erpm_deadband_` (rejects phantom hall-sensor standstill noise, observed 346-384 ERPM at true rest) and `gyro_bias_z_` (subtracted from `gyr_z()` before publishing).
- **Uncommitted, on top of `d08fcfd`:** adds per-axis IMU covariance calibration params; adds `vx_variance_`; **fixes a sign-convention bug** (`current_speed = (-raw_erpm - offset)/gain` → `(raw_erpm - offset)/gain`, comment: "positive erpm (forward) must yield positive speed"); switches the servo-command subscription from `sensors/servo_position_command` to `commands/servo/position` (rationale in the diff: the old topic "only fires once the driver is in MODE_OPERATING and has actually received a command"); removes an early-return gate that previously blocked `/odom` publication entirely until the first servo command arrived.
- **These four uncommitted hunks are exactly what `f1tenth_bringup/config/vesc.yaml`'s comments describe as newly-added** — the outer config was written against this uncommitted code, not the pinned commit. Also: an identically-worded, identically-dated, identically-co-authored commit exists in `ackermann_mux` (§2.2) — evidently one coordinated debugging session across both submodules.
- Other branches present but not checked out: `odom-test` (local, ahead of `origin/odom-test`, a divergent merge-PR chain), `vesc-tuner-test` (local, 1 ahead of `origin/vesc-tuner-test`: "vesc_tuning: update speed calibration gains from latest tuning run" — likely the origin of the stray `.patch` file at the workspace root, §1.1's flag).
- Confirmed real-code use: `vesc_driver.cpp` links `transport_drivers`' `serial_driver`/`io_context` as a C++ library for actual serial I/O (§2.6).

---

### 2.2 `ackermann_mux`

**Remote:** `origin` only → `https://github.com/sjckness/ackermann_mux.git` (personal fork). **Pinned SHA:** `42b7cd70` — same pattern as `vesc`, HEAD detached on a commit reachable on **no branch**. Working tree **clean** (this local commit is fully committed, just unpushed).

**Purpose:** Ackermann-command multiplexer with priority "locks" — fork of `twist_mux`, retargeted from `Twist` to `AckermannDriveStamped`.

**Nodes:** `ackermann_mux` (referenced by `f1tenth_control/ackermann_mux_launch.py`). `scripts/joystick_relay.py` present but its `install()` line in `CMakeLists.txt` is **commented out** ("Pending to test ROS2 migration") — never installed, dead.

**Topics:** pub `ackermann_cmd` (hardcoded) — outer-remapped to `ackermann_drive`, meeting `vesc_ackermann`'s own remap in the middle. Sub: one topic per configured `topics:` entry + one `Bool` per `locks:` entry (config-driven, not hardcoded).

**Launch files:** own `launch/ackermann_mux_launch.py` (3 separate config files: locks/topics/joy) — **not used** by the outer workspace, which has its own hand-written, differently-shaped launch file of the same name in a different package (explicitly documented as a deliberate non-reuse in that file's own docstring).

**Params:** outer `f1tenth_bringup/config/mux.yaml` **replaces** (not deltas) the submodule's own default `config/ackermann_mux_topics.yaml` — different naming scheme entirely (`safety_stop`/`drive`/`teleop` vs. submodule's `safety`/`navigation`/`joystick`/`keyboard`/`tablet`), and the outer file has no `locks:` section at all (submodule's lock/priority-disable feature unused in this deployment).

**Git archaeology:**
- Real, already-**pushed** F1TENTH fork lineage on `origin/foxy-devel`: `c6d4926` (maintainer handoff to F1TENTH's Hongrui Zheng) → `050b9ba` ("ackermann mux": renames `twist_mux`→`ackermann_mux` throughout) → `2c30a97` (swaps `Twist` for `AckermannDriveStamped`) → `6fcf96f` → `b3c0b08` ("Change out going topic name": sets output to `ackermann_cmd`). Genuine, published fork history.
- **Local, unpushed `42b7cd7`** "nav2 not working, odometry not bad" (same author, same co-author, same date as the `vesc` commit — one combined session): single-file diff to `config/ackermann_mux_topics.yaml` — adds a new `safety:` entry (priority 200, the new highest) and retunes existing priorities (`navigation` 10→120, `joystick` 100→150, `tablet` 100→80). **Note:** this retuned file is the submodule's own default, which the outer workspace's launch file never actually loads (§ above) — the equivalent fix (`safety_stop`/`navigation`/`joystick` priorities) was applied **redundantly** in the outer `mux.yaml` too.

---

### 2.3 `teleop_tools`

**Remote:** `origin` → `https://github.com/f1tenth/teleop_tools.git` (F1TENTH org's own vendoring of PAL Robotics' `teleop_tools` — no further personal fork on top). **Pinned SHA:** exact tip of `origin/foxy-devel` (`1.2.1-15-g4337558` — 15 upstream lint/feature commits past the `1.2.1` tag). Working tree **clean, no local fork history** — confirms clean vendoring.

**Packages:** `teleop_tools` (metapackage, no code); `joy_teleop` (nodes: `joy_teleop` — used by outer `joy_launch.py`; `incrementer_server` — demo-only action server, unused); `key_teleop` (node `key_teleop` — **not referenced anywhere in the outer workspace**); `mouse_teleop` (node `mouse_teleop` — **not referenced anywhere**); `teleop_tools_msgs` (`action/Increment.action` — zero consumers anywhere in the outer `src/` tree, only used by the unused `incrementer_server`).

**Outer config:** `f1tenth_bringup/config/joy_teleop.yaml` configures 3 `type: topic` mappings, all publishing `AckermannDriveStamped` on `teleop` — matching `mux.yaml`'s joystick lane. The package's generic `type: service`/`type: action` dispatch support exists in code but is entirely unexercised by this deployment.

**Only `joy_teleop` (of 3 sibling nodes) is actually wired in; `key_teleop` and `mouse_teleop` are fully dormant.**

---

### 2.4 `zed_ros2_wrapper`

**Remote:** `origin` → `https://github.com/stereolabs/zed-ros2-wrapper.git` (real upstream). **Pinned SHA:** `b5844a81`, tag `humble-v4.2.5` — confirmed genuine upstream commit (`git merge-base --is-ancestor` = true), 574 commits behind `origin/master`'s current tip, **zero local modification**. Clean vendoring.

**Packages:** `zed_ros2` (metapackage, no code); `zed_wrapper` (owns `launch/zed_camera.launch.py`, included by outer `f1tenth_perception/camera.launch.py`); `zed_components` (the actual implementation, `ZedCamera`/`ZedCameraOne` composable nodes — `ZedCamera` is what's instantiated, `camera_model='zed2'`).

**Params overridden by outer `camera.launch.py`** (vs. submodule defaults): `camera_model` (required→`'zed2'`), `camera_name` (`''`→`'zed2'`), `ros_params_override_path` (`''`→`f1tenth_perception/config/zed2_perception.yaml`), `publish_tf` (`true`→`false`), `publish_map_tf` (`true`→`false`). Left at submodule defaults: `publish_urdf` (stays `true` — needed so `robot_state_publisher` broadcasts the camera's internal TF subtree, though note §1.9's flag that `robot_state_publisher` itself never runs on the real-hardware path), `publish_imu_tf`, and everything sim/streaming/GNSS-related.

**`zed2_perception.yaml` overlay:** disables `publish_status`, `publish_stereo`, `pos_tracking_enabled`, `mapping_enabled`, `od_enabled`, `publish_imu_raw`, `publish_imu_tf`; enables `publish_rgb`; sets `depth_mode: PERFORMANCE`, `pub_resolution: CUSTOM` @ `pub_downscale_factor: 2.0`, `pub_frame_rate: 30.0`.

**Active topics (given the above config):** `rgb/image_rect_color`+`camera_info` (remapped to `/camera/image_raw`/`/camera/camera_info`), `depth/depth_registered` + `depth/depth_info` (`zed_msgs/DepthInfoStamped`), `imu/data`. Position-tracking/mapping/object-detection topics and services exist in code but are structurally disabled by config.

**Services defined but never called from the outer workspace:** `reset_odometry`, `set_pose`, `enable_obj_det`/`enable_body_trk`/`enable_mapping`/`enable_streaming`, `start_svo_rec`, `stop_svo_rec`/`pause_svo`, `set_roi`, `reset_roi`, `to_ll`/`from_ll` (GNSS, disabled). `set_svo_frame` service call in code actually uses a `cob_srvs/srv/SetInt` type, not the `zed_msgs/srv/SetSvoFrame` message that exists in the pinned `zed-ros2-interfaces` — appears unused/superseded in this wrapper version.

---

### 2.5 `zed-ros2-interfaces` (`zed_msgs`)

**Remote:** `origin` → `https://github.com/stereolabs/zed-ros2-interfaces.git`. **Pinned SHA:** `e97008721`, tag `5.3.0` — **exactly matches** local `master`/`origin/master`/`origin/HEAD`. Zero divergence — the cleanest of all 6 submodules.

**Definitions:** 17 `.msg` files (`BoundingBox2Df/Di`, `BoundingBox3D`, `DepthInfoStamped`, `GnssFusionStatus`, `HealthStatusStamped`, `Heartbeat`, `Keypoint2Df/Di`, `Keypoint3D`, `MagHeadingStatus`, `Object`, `ObjectsStamped`, `PlaneStamped`, `PosTrackStatus`, `Skeleton2D/3D`, `SvoStatus`) + 5 `.srv` (`SaveAreaMemory`, `SetPose`, `SetROI`, `SetSvoFrame`, `StartSvoRec`).

**Consumers:** inside `zed_components` only (`DepthInfoStamped` published, `SetPose`/`SetROI`/`StartSvoRec` served). **Zero consumers anywhere in the outer `src/` tree** — verified precisely. **False-lead note:** `f1tenth_perception`'s `detection_3d_node.py`/`yolo_detector_node.py` use a class also named `BoundingBox3D`, but it's `from vision_msgs.msg import BoundingBox3D` — the standard ROS `vision_msgs` package, unrelated to `zed_msgs`'s own `BoundingBox3D.msg`. A naive grep for "BoundingBox3D" would misleadingly suggest a dependency that doesn't exist.

---

### 2.6 `transport_drivers`

**Remote:** `origin` → `https://github.com/ros-drivers/transport_drivers.git` (real upstream). **Pinned SHA:** `d3f510ce`, tag `1.2.0` — exactly `origin/humble`. Local `main` sits ahead at `0af5988` (the moving branch) — pin correctly tracks the humble-distro release, not the moving tip. No local commits, no uncommitted changes.

**Packages:** `asio_cmake_module` (CMake module only, no code); `io_context` (library, no standalone node — consumed by `serial_driver`, `udp_driver`, **and directly by `vesc_driver`**: `vesc_interface.cpp` instantiates `IoContext(2)`); `serial_driver` (node `serial_bridge` — **not referenced by any launch file anywhere**; the package is consumed as a **C++ library**, not a node — `vesc_interface.cpp` directly includes `serial_driver.hpp`/instantiates `SerialDriver`/`SerialPortConfig` for real `/dev/ttyACM0` I/O — this is the actual load-bearing use of the whole submodule); `udp_driver` (3 executables — `udp_receiver_node_exe`, `udp_sender_node_exe`, `udp_bridge_node_exe` — **none referenced anywhere**, and no outer package even depends on `udp_driver` at all; fully dormant).

**Only the `serial_driver`→`io_context`→`asio_cmake_module` chain, consumed as a compiled library by `vesc_driver`, is actually load-bearing.** Every standalone node in this submodule is dormant.

---

## 3. Cross-workspace topic wiring

Topics with confirmed publisher(s) and subscriber(s) across every package/submodule above. **"Live (default config)"** means the connection is active under the workspace's current defaults (`camera_source=zed`, `localization_source=raw_odom`, `enable_safety_stop=false`, `enable_llm=true`, `use_behavior_tree=true`, `enable_nav2=true`, `use_lidar=false`).

| Topic | Type | Publisher(s) | Subscriber(s) | Status |
|---|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | `vesc_to_odom_node_backup` (as `vesc_to_odom_node`) | `raw_odom_map_tf_node` (**live, default**), `ekf_filter_node` (only if `localization_source=ekf`), `llm_mpc_tuner_node` (only if `enable_llm`, **live, default**), 6 orphaned `mpc_controller` nodes, orphaned `odom_tf_broadcaster` | ✅ live |
| `commands/servo/position` | `std_msgs/Float64` | `ackermann_to_vesc_node` | `vesc_driver_node` (own command input), `vesc_to_odom_node_backup` (**per the uncommitted vesc fix**) | ✅ live |
| `sensors/servo_position_command` | `std_msgs/Float64` | `vesc_driver_node` | *(none, with the uncommitted fix applied — the pinned/committed code's `vesc_to_odom_node` would have subscribed here, but that node isn't deployed)* | ⚠️ orphaned publisher |
| `sensors/imu` | `vesc_msgs/VescImuStamped` | `vesc_driver_node` | *(none, anywhere)* | ⚠️ dead topic |
| `sensors/imu/raw` | `sensor_msgs/Imu` | `vesc_driver_node` | `ekf_filter_node` (only if `localization_source=ekf` — **not the default**), `f1tenth_diagnostics` calibration nodes (manual/standalone only) | ⚠️ published continuously, no subscriber in the *default* auto-launched graph |
| `/scan` | `sensor_msgs/LaserScan` | `urg_node` (gated `use_lidar`, **default `false`**, and `nav2_bringup.launch.py` passes no override) | Nav2's costmap obstacle layer (`nav2_bringup.launch.py`'s `controller_server`/`local_costmap`/`global_costmap`) | 🔴 **no publisher under default config** — Nav2's costmap gets zero LiDAR data unless `use_lidar:=true` is explicitly passed at the top-level `ros2 launch` invocation |
| `/camera/detections_3d` | `vision_msgs/Detection3DArray` | `detection_3d_node` (default `camera_source=zed`) | `IsObstacleDetected` BT behaviour (**live, default**), `simple_stop_controller_node` (only if `enable_safety_stop`, not default) | ✅ live (BT path) |
| `/diagnostics/system_status` | `f1tenth_messages/SystemStatus` | `system_observer_node` (always running) | *(none, anywhere)* | ⚠️ dead topic (external-consumer-only by design, e.g. Foxglove) |
| `/odometry/filtered` | `nav_msgs/Odometry` | `ekf_filter_node` (only if `localization_source=ekf`, not default) | *(none found anywhere)* | ⚠️ dead topic even when EKF is active |
| `cmd_vel` → `cmd_vel_nav` | `geometry_msgs/Twist` | `controller_server` (Nav2, remapped) | `twist_to_ackermann_node` | ✅ live, confirmed-good connection |
| `drive` (mux "navigation" lane) | `ackermann_msgs/AckermannDriveStamped` | `twist_to_ackermann_node` (**live, default BT path**); also 7 `mpc_controller` nodes (orphaned/unreachable) | `ackermann_mux` | ✅ live (BT path); MPC path would double-publish here if ever re-enabled — by design, mutually exclusive |
| `safety_stop` (mux lane, prio 200) | `AckermannDriveStamped` | `Stop` BT behaviour (fires only when triggered) | `ackermann_mux` | ✅ live |
| `teleop` (mux lane, prio 100) | `AckermannDriveStamped` | `joy_teleop` (3 mappings), `llm_mpc_tuner_node` (STOP keypress only), `stack_startup_sequence` (only if separately launched) | `ackermann_mux` | ✅ live, multiple legitimate publishers by design |
| `ackermann_cmd` → `ackermann_drive` | `AckermannDriveStamped` | `ackermann_mux` (remapped) | `ackermann_to_vesc_node` (remapped) | ✅ live, confirmed-good connection |
| `/goal_pose` | `geometry_msgs/PoseStamped` | *(none inside this workspace — intended to come from RViz2's "2D Goal Pose" tool or Foxglove's publish panel)* | `HasGoalPose` BT behaviour | ✅ intended human-in-the-loop pattern, not a bug |

---

## 4. Flagged for review

### Orphaned executables (registered, never referenced by any launch file or `components.yaml`)
- `f1tenth_navigation`: `odom_tf_broadcaster` (reachable only from the dead `map_server_launch_old.py`)
- `f1tenth_bringup`: `tf_publisher`, `throttle_interpolator`
- `mpc_controller`: `mpc_node`, `trajectory_mpc_node`, `kinematic_mpc_node`, `frenet_mpc_node`, `andre_mpc_node_linear`, `andre_mpc_opt_node`, `track_mpc_controller` (6 of 7 — only `andre_mpc_node` is referenced anywhere, and even that reference is dead code)
- `vesc_driver` (submodule): `vesc_device_namer` (likely udev-invoked, not launch-invoked — lower-severity flag)
- `vesc_ackermann` (submodule): `vesc_to_odom_node` (the newer IMU-fused KF odometry node — superseded in deployment by `vesc_to_odom_node_backup`, but its own source comments suggest the opposite was intended)
- `ackermann_mux` (submodule): `scripts/joystick_relay.py` (never installed — commented out in `CMakeLists.txt`)
- `teleop_tools` (submodule): `key_teleop`, `mouse_teleop`, `incrementer_server`
- `transport_drivers` (submodule): `serial_bridge`, `udp_receiver_node_exe`, `udp_sender_node_exe`, `udp_bridge_node_exe` (this whole submodule is consumed as a C++ library by `vesc_driver`, not via any of its own nodes)

### Dead / orphaned topics
- `sensors/imu` (`VescImuStamped`) — published, never subscribed, anywhere.
- `sensors/servo_position_command` — published, no subscriber once the uncommitted `vesc` fix is accounted for (the topic that *would* consume it belongs to the undeployed `vesc_to_odom_node`).
- `/diagnostics/system_status` — published continuously, no in-repo subscriber (external-tool-only by design, but worth confirming that's intentional).
- `/odometry/filtered` — the EKF's actual output topic, no subscriber anywhere even when the EKF path is selected.
- `/sensors/imu/raw` — has a real subscriber only when `localization_source=ekf`; under the *default* `raw_odom` config it's published continuously to nobody.

### 🔴 High-priority: `/scan` has no publisher under default config
`f1tenth_navigation/nav2_bringup.launch.py` includes `f1tenth_perception/lidar.launch.py` with **no `launch_arguments` override**, so `use_lidar` stays at that file's own default of `false`. Since `enable_nav2` defaults `true`, Nav2's costmap obstacle layer runs by default with **zero LiDAR data** unless an operator explicitly passes `use_lidar:=true` at the top-level `ros2 launch` invocation (which *would* propagate down correctly, since ROS 2 launch args are globally overridable from any include depth — this isn't broken, just silently off by default). Given LiDAR is the primary local-obstacle sensor for Nav2's costmap, this seems worth a deliberate decision either way, not a silent default.

### 🔴 High-priority: `vesc` submodule has uncommitted changes the outer workspace already depends on
The pinned commit (`d08fcfd`) does **not** include the erpm sign-fix, the servo-topic fix, or the per-axis IMU/vx covariance params that `f1tenth_bringup/config/vesc.yaml` already assumes exist. A `git submodule update` from this exact pin (e.g. on a fresh clone, or CI) would silently revert to code with the pre-fix erpm sign bug and missing covariance-calibration parameters, likely breaking VESC odometry and/or failing to declare params the yaml expects. **Recommend committing (and ideally pushing) the working-tree changes in the `vesc` submodule before any reorganization work touches it.**

### Naming / identity mismatches
- `mpc_node_name` default (`/andre_mpc_opt_controller`, used by `safety_stop_controller`) doesn't match the node name the one launchable-in-principle MPC node (`andre_mpc_node.py`) actually registers (`andre_mpc_controller`).
- `llm_mpc_tuner_node`'s own `target_node` default (`/andre_mpc_controller`) is a second, independent "the MPC node" name, not sourced from `stack_params.yaml` at all.
- `vesc_to_odom_node` vs. `vesc_to_odom_node_backup`: the "backup"/"fallback"-labeled code is what's actually deployed in production; the non-backup code is the orphan. Source comments in both the outer workspace and the vendored `.launch.xml` files read as if the relationship is the other way around.

### Duplicate / superseded / legacy launch files
- `f1tenth_navigation/launch/map_server_launch_old.py` — self-documented deprecated, confirmed unreferenced.
- `f1tenth_perception/launch/perception.launch.py` — standalone test entry point with a self-acknowledged, unreconciled duplicate inline `urg_node` block (vs. `lidar.launch.py`).
- `vesc_driver/launch/vesc_driver_node.launch.py`, `vesc_ackermann/launch/*.launch.xml` — vendored/upstream standalone launch files, superseded by the outer workspace's own inline `Node()` construction in `vesc_launch.py`; the `.launch.xml` files are additionally self-contradictory about which odometry variant is "the fallback."
- `f1tenth_bringup/launch/startup_sequence_launch.py` — not dead, but not part of either bring-up path; opt-in utility only.

### Declared-but-effectively-unused params
- `f1tenth_behavior`: `wait_for_trigger_service_node`'s `poll_interval_sec`, `behavior_executor_node`'s `bt_loop_duration_ms` (both declared+read in-node, never exposed via any launch arg/yaml).
- `f1tenth_perception`: `detection_3d_node`'s `sync_slop`, `sync_queue_size`, `default_depth_extent`, `marker_lifetime`, `center_fraction`, `output_frame`, `label_scale`, `label_z_offset` (all declared, never externally overridden).
- `f1tenth_bringup`: `startup_sequence_launch.py` forwards a `mpc_node_name` arg to `stack_startup_sequence`, which never declares/reads that parameter — silently dropped. The same file also declares `startup_command_topic` but the node's own `publish_rate_hz` param isn't in the forwarded-args list at all, so it's stuck at its hardcoded default (20.0) regardless of what's passed at launch.
- `mpc_controller`: all 7 nodes' tuning gains (`qn`, `qalpha`, `qddelta`, `v_ref`, etc.) — `mpc_launch.py` passes no `parameters=[...]` at all, so even if re-enabled, only in-code defaults would ever be live.

### Config/description drift
- `f1tenth_perception/package.xml` description references `/yolo/2D_detections` — real topic is `/camera/detections`.
- `f1tenth_navigation/setup.py`'s description doesn't mention Nav2 (package.xml's does).
- `f1tenth_navigation/maps/README.md` documents an outdated TF-ownership chain.
- Several docstrings workspace-wide still call the main launch file `stack_bringup_launch.py` (old name) instead of the current `stack_bringup.launch.py`.
- `f1tenth_hardware/package.xml` depends on a nonexistent `vesc_tuning` package.
- `f1tenth_more`'s metapackage dependency list omits several real, active packages (transitively resolved via `f1tenth_bringup`, so not build-breaking, but incomplete as documentation).

### Uncommitted / untracked repo state
- `f1tenth_bringup/config/components.yaml` — the entire supervisor registry — is untracked in git.
- 7 timestamped `f1tenth_bringup/config/vesc.yaml.bak.*` files — real calibration-run output, untracked, repo clutter.
- `f1tenth_behavior/config/waypoints.yaml` — staged as deleted (consistent with the package's documented move to single-`/goal_pose` input).
- `vesc` submodule — see the high-priority flag above.
- `ackermann_mux` submodule — 1 local commit, fully committed but unpushed to `origin/foxy-devel`.

### Confirmed, not a bug
- `stack_bringup.launch.py`'s `mpc_bringup` variable remains built-but-unreturned (per the explicit task ask) — unchanged, matches the file's own comment describing this as intentional/pre-existing.
- `joy_launch.py` is intentionally never auto-included (by design, per its own docstring).
- `/goal_pose` having no in-workspace publisher is the intended human-in-the-loop design (RViz2/Foxglove), not an orphaned topic.
