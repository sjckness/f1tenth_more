# f1tenth_more

ROS 2 Humble workspace for a physical F1TENTH autonomous racing car (VESC drive stack,
ZED2 stereo camera or a webcam fallback, Hokuyo LiDAR, Jetson Orin/Thor), plus a Gazebo
Fortress simulation. Real hardware and simulation are separate, non-overlapping entry
points that share the same robot description, EKF config, and most node code.

This README reflects the workspace after a full package-by-package reorganization
(the "Phase 0-10" reorg referenced in commit history / `workspace_inventory.md`, the
original read-only audit that scoped the work).

## Quick start — the two bringup entry points

| | `stack_bringup.launch.py` | `supervisor_bringup.launch.py` |
|---|---|---|
| **Run** | `ros2 launch f1tenth_bringup stack_bringup.launch.py` | `ros2 launch f1tenth_bringup supervisor_bringup.launch.py` |
| **Process model** | One `ros2 launch` process, one process tree. | Spawns each functional group ("component") as its own `ros2 launch` subprocess, independently restartable. |
| **Restart a subsystem without restarting everything?** | No — killing/restarting the launch tears down the whole stack. | Yes — `ros2 service call /restart_component ...` (see below). |
| **Crash recovery** | None built in. | A watchdog auto-respawns crashed components (bounded by a restart budget), except ones you've explicitly stopped. |
| **When to use** | Simplest option; default choice for normal driving sessions. | Development/debugging where you want to bounce one subsystem (e.g. perception after a camera hiccup) without losing localization/control. |

Both read every default from `f1tenth_params/config/stack_params.yaml` and both are
kept in sync by hand — `stack_bringup.launch.py` is the working fallback and is
intentionally left untouched by the supervisor's existence.

Simulation is a third, separate entry point, unrelated to the two above:
```
ros2 launch f1tenth_sim sim_bringup.launch.py
```

## Packages

### Core stack (real hardware)

| Package | Purpose |
|---|---|
| `f1tenth_params` | Single source of truth for every launch-parameter default (`config/stack_params.yaml`) + the `param_defaults` helper every other package's launch files import. Dependency-free leaf package (avoids a build-order cycle). |
| `f1tenth_hardware` | VESC drive chain: battery pre-flight gate, optional live covariance calibration, `ackermann_to_vesc_node`, `vesc_to_odom_node` (backup/KF-free variant), `vesc_driver_node`, static `base_link→imu` TF. |
| `f1tenth_description` | URDF/xacro + meshes + `robot_state_publisher`. Single entry point for the robot model *and* the real-hardware static sensor TFs (`base_link→laser`). Shared by real hardware and sim. |
| `f1tenth_localization` | `map→odom`: either `robot_localization` EKF (fuses `/odom` + VESC IMU) or a raw unfiltered mirror of `/odom` — selected by `localization_source`. |
| `f1tenth_perception` | Camera bringup (ZED2 wrapper or `v4l2` webcam, selected by `camera_source`) + YOLO 2D/3D detection fusion. Owns the ZED2's static TF too. |
| `f1tenth_control` | `ackermann_mux` (arbitrates drive-command sources by priority) + the MPC controller (`mpc_controller`; `MPC_corr.py`/`mpc_corr` is the drive-command source `f1tenth_navigation/navigation.launch.py` brings up when `enable_nav2:=false` — see that package's row below. `andre_mpc_node.py`'s own `mpc.launch.py` still exists but isn't included by either bringup path). `safety_stop_controller` (a reactive corridor-stop layer that used to live here) was retired — superseded by `f1tenth_behavior`'s own unconditional `handle_obstacle` lane, see below. |
| `f1tenth_navigation` | Static map server + the full Nav2 stack (planner/controller/behavior servers, `bt_navigator`, one shared lifecycle manager), split into one launch file per node plus an orchestrator (`nav2.launch.py`). `navigation.launch.py` wraps that orchestrator one level up: Nav2 when `enable_nav2:=true`, `mpc_controller`'s `mpc_corr` (see `f1tenth_control`'s row above) plus a standalone `map_server` (`map_only.launch.py`, its own single-node lifecycle manager) when `false` — `/map` stays published either way — the single site both bringup paths include instead of branching this themselves. |
| `f1tenth_behavior` | py_trees behavior tree: emergency-stop / obstacle-stop / Nav2-goal-navigation priority lanes (see below), plus the Nav2-readiness gate and the Twist→Ackermann bridge. |
| `f1tenth_diagnostics` | Calibration tooling (gyro bias, IMU/odom covariance) + `system_observer_node` (CPU/GPU/RAM, gated by `enable_sys_obs`) + `diagnostics_server_node` (continuous battery monitoring + on-demand diagnostics service, never gated). |
| `f1tenth_bringup` | Owns both top-level entry points, `component_supervisor_node`, the Foxglove bridge, the boot-time steering-sweep self-check, and most shared hardware config (`vesc.yaml`, `ekf.yaml`, `mux.yaml`, ...). |
| `f1tenth_intelligence/llm` (ROS package name: `llm`) | `llama-server` bringup + an LLM-driven MPC parameter tuner (manual-trigger, talks to the running MPC node over `ros2 param set`). Opt-in (`enable_llm`). |
| `f1tenth_messages` | Shared custom interfaces: `SystemStatus.msg`, `BatteryStatus.msg`, `RestartComponent.srv`, `ComponentControl.srv`, `RunDiagnostics.srv`. `ament_cmake` (the only interfaces-only package — everything else here is `ament_python`). |
| `f1tenth_more` | Top-level metapackage. Aggregates every first-party package (no code of its own). |

### Simulation

| Package | Purpose |
|---|---|
| `f1tenth_sim` | Gazebo Fortress bringup: spawns the shared `f1tenth_description` model, bridges sim sensors onto the same topic names the real stack uses, drives via `ros2_control` + an Ackermann controller. Reuses `ekf.yaml` and the SLAM config verbatim. No VESC/ZED SDK/urg_node. |

### Vendored (git submodules, under `f1tenth_external/` and `f1tenth_hardware/vesc/`)

`vesc` (driver + odometry + msgs, personal fork with local commits), `ackermann_mux`
(personal fork), `teleop_tools` (joystick teleop), `zed_ros2_wrapper` +
`zed-ros2-interfaces` (ZED2 SDK wrapper, clean upstream), `transport_drivers` (serial
I/O library `vesc_driver` links against). Not first-party code — not covered by the
reorg or this README's param reference.

## The 5 stack-wide branching args

These are **not** `DeclareLaunchArgument`s anywhere in the workspace — every file that
needs one reads it directly from `stack_params.yaml` as a plain Python value at parse
time. The only way to change one is editing that file; passing it on the CLI
(`camera_source:=webcam`) is silently ignored, since no launch argument by that name
exists to receive it.

| Arg | Default | Meaning |
|---|---|---|
| `camera_source` | `zed` | `'zed'` \| `'webcam'` — camera hardware selection. |
| `localization_source` | `raw_odom` | `'ekf'` \| `'raw_odom'` — `map→odom` source. **Default is raw, not EKF.** |
| `enable_llm` | `false` | Opt-in LLM stack — loads a GGUF model into GPU memory, ~20-30s warm-up. |
| `use_behavior_tree` | `true` | `true`: the py_trees BT drives the `ackermann_mux` "navigation" lane (via Nav2, when `enable_nav2` is also true). `false`: the BT does not run — see `enable_nav2` for what drives the mux instead. |
| `enable_nav2` | `true` | `true`: bring up the Nav2 stack (idles with no goal, puts nothing on the mux by itself). `false`: `navigation.launch.py` brings up `mpc_corr` instead, which does drive the mux directly. Independent of `use_behavior_tree`. |

## Full parameter reference (`f1tenth_params/config/stack_params.yaml`)

Every other param below is a normal `DeclareLaunchArgument` — its owning launch file
still declares it, so CLI overrides and `ros2 launch <file> --show-args` work exactly
as you'd expect; only the *default value* and *description* are sourced centrally.

### Hardware
| Param | Default | What it does |
|---|---|---|
| `vesc_config` | `config/vesc.yaml` | VESC calibration (speed/steering gains, limits), shared by the whole VESC chain. |
| `calibration` | `false` | Run a live message-level covariance calibration before releasing EKF/Nav2, then relaunch with fresh values. |
| `calibration_duration_sec` | `60.0` | Stationary sampling window for the auto-calibration path above. |
| `min_battery_voltage` | `10.8` | Startup veto threshold (volts) — also the continuous-monitoring threshold feeding the BT emergency lane. |
| `release_downstream` | `true` | Whether `vesc.launch.py` releases EKF/Nav2 itself after calibration (`false` for the component-supervisor path, which tracks them independently). |

### Localization/TF
| Param | Default | What it does |
|---|---|---|
| `ekf_config` | `config/ekf.yaml` | `robot_localization` EKF config: fuses `/odom` + VESC IMU, publishes `map→odom`. |

### Perception
| Param | Default | What it does |
|---|---|---|
| `confidence_threshold` | `0.3` | Minimum YOLO detection score kept before publishing to `Detection3DArray`. |
| `yolo_device` | `cuda` | Torch inference device: `'cuda'` or `'cpu'`. |
| `yolo_model` | `yolo26s.engine` | YOLO weights filename (TensorRT `.engine` or portable `.pt`). |
| `use_lidar` | `false` | Start the Hokuyo LiDAR (`urg_node`) — also what Nav2's costmap gets, since `nav2.launch.py` doesn't override it. |
| `sensors_config` | `config/sensors.yaml` | Hokuyo LiDAR IP/port/frame config. |

### Command/Control
| Param | Default | What it does |
|---|---|---|
| `mux_config` | `config/mux.yaml` | `ackermann_mux` priority/lock/topic config. |
| `joy_config` | `config/joy_teleop.yaml` | Joystick button/axis mapping for manual control. |
| `mpc_node_name` | `/mpc_corr` | Fully-qualified MPC node name, used by the LLM MPC tuner for `SetParameters` calls. |
| `qn`, `qalpha`, `qddelta` | `50.0`, `30.0`, `2.0` | MPC rollout cost weights: radial tracking error, heading error, steering-rate smoothness. |
| `qv` | `50.0` | Declared MPC cost weight — **currently unused** by `andre_mpc_node.py`'s own cost function (pre-existing gap in the node). |
| `alat_max` | `10.0` | Lateral acceleration soft-constraint limit [m/s²]. |
| `a_min` | `-3.0` | Declared MPC deceleration limit — **currently unused** by the node's own control loop (pre-existing gap). |
| `a_max` | `3.0` | Acceleration rate-limit toward `v_ref` [m/s²]. |
| `v_min`, `v_max` | `-1.5`, `1.5` | MPC speed floor/ceiling [m/s]. |
| `v_ref` | `1.0` | Center of the MPC's open-loop sine speed profile. |
| `sine_amp`, `sine_period` | `0.0`, `4.0` | Amplitude/period of that sine profile (`0.0` amplitude = constant speed). |
| `car_radius` | `0.20` | Car's physical footprint radius [m] — single source of truth shared by `MPC_corr.py` (`car_radius`/soft-avoidance trigger radius, `dmin`) and the BT's `IsObstacleDetected`/`IsProximityTooClose` (corridor + proximity thresholds, see below). Replaces `safety_stop_controller`'s now-deleted, drifted-out-of-agreement corridor params. |
| `obstacle_safety_margin_m` | `0.12` | Shared base safety-margin buffer [m] beyond `car_radius`, combined per-mechanism (see `car_radius` above and `stack_params.yaml`'s own comment on this key for the full derivation). |
| `proximity_front_extra_margin_m` | `0.08` | Small extra conservatism [m] added only to `IsProximityTooClose`'s front standoff (raw-sensor last resort behind the front direction's already-covered YOLO lane). |

### Autonomy (Nav2 + BT)
| Param | Default | What it does |
|---|---|---|
| `map` | `maps/square_100m.yaml` | Map file `map_server` serves (resolved against `f1tenth_navigation`). |
| `autostart` | `true` | Auto-activate the Nav2 lifecycle-managed nodes on startup. |
| `nav2_params_config` | `config/nav2_params.yaml` | Shared Nav2 params file (controller/planner/behavior/bt_navigator + costmaps). |
| `enable_nav2_map`, `enable_nav2_controller`, `enable_nav2_planner`, `enable_nav2_behavior_server`, `enable_nav2_bt_navigator` | all `true` | Per-node opt-outs from `nav2.launch.py`'s bring-up. |
| `nav2_readiness_service` | `/lifecycle_manager_navigation/is_active` | Service polled before starting the BT nodes. |
| `nav2_readiness_timeout_sec` | `90.0` | How long to wait for Nav2 readiness before giving up (BT will not launch). |
| `bt_setup_timeout_sec` | `90.0` | The BT's own `tree.setup()` timeout — an independent safety margin. |
| `sys_obs_max_temp_c` | `85.0` | Shared CPU/GPU temp threshold for the BT emergency lane's overheat check. |
| `sys_obs_max_load_percent` | `95.0` | Shared CPU/GPU load threshold, same check. |

### Diagnostics & Intelligence
| Param | Default | What it does |
|---|---|---|
| `enable_sys_obs` | `true` | Run `system_observer_node` and include the overheat check in the BT. `false` fully skips both, not just idles them. |
| `imu_topic` | `/sensors/imu/raw` | Topic sampled by the gyro-bias and covariance calibration nodes. |
| `gyro_sample_duration_sec` | `30.0` | Gyro-bias calibration sampling window. |
| `min_samples` | `150` | Warn if gyro-bias calibration collected fewer samples than this. |
| `odom_topic` | `/odom` | Topic sampled by the covariance calibration node. |
| `covariance_sample_duration_sec` | `60.0` | Covariance calibration sampling window (standalone entry point). |
| `publish_rate_hz` | `1.0` | `system_observer_node`'s `SystemStatus` publish rate. |
| `battery_check_rate_hz` | `2.0` | `diagnostics_server_node`'s `BatteryStatus` publish rate — faster than the line above since it feeds a continuously-ticking BT condition. |
| `start_server` | `true` | Whether `llm.launch.py` starts `llama-server` itself, vs. assuming one's already running. |
| `model` | `qwen_mpc_pruned` | Model name key into `llm/config/models.yaml`. |
| `interrogation` | `mpc_tuner` | Interrogation name key into `llm/config/interrogations.yaml`. |
| `llama_server_path` | `/scratch/fabiocar/llama.cpp/build/bin/llama-server` | Absolute path to the `llama-server` binary. |
| `llama_server_cwd` | `/scratch/fabiocar/llama.cpp` | Working directory to launch it from (the llama.cpp project root — not derivable from the binary path, two directory levels up). |

### Description/Simulation
| Param | Default | What it does |
|---|---|---|
| `use_sim` | `false` | Emit the `ign_ros2_control` Gazebo plugin + sim sensors in the URDF. Real-hardware-safe default; `f1tenth_sim` overrides to `true`. |
| `enable_sensors` | `false` | Include LiDAR/camera/IMU sensor *links* in the URDF. **Must stay `false` on real hardware** — those frames come from `static_transform_publisher` nodes instead; the URDF and the static publishers would otherwise fight over the same transforms. `f1tenth_sim` overrides to `true`. |
| `control_config` | `''` | Path to the `ros2_control` controllers.yaml (sim only). |

### Component Supervisor
| Param | Default | What it does |
|---|---|---|
| `components_config` | `config/components.yaml` | Registry mapping component name → the `ros2 launch` command(s) that bring it up. |
| `restart_timeout_sec` | `10.0` | How long the supervisor waits for SIGINT before escalating to SIGKILL. |
| `log_dir` | `~/.ros/log/component_supervisor` | Per-component stdout/stderr log directory. |
| `watchdog_period_sec` | `2.0` | How often the watchdog polls for crashed components. |
| `max_auto_restarts` | `3` | Auto-respawns allowed per process within the trailing budget window. |
| `restart_budget_window_sec` | `60.0` | That trailing window, in seconds — a sliding-window crash-loop budget. |

### Dev Tools
| Param | Default | What it does |
|---|---|---|
| `enable_foxglove` | `true` | Launch `foxglove_bridge` (port 8765). `false` fully skips it. |
| `startup_delay` | `5.0` | Seconds to wait before the boot-time steer-sweep self-check starts. |
| `right_duration`, `left_duration`, `center_duration` | `2.0`, `2.0`, `1.0` | How long each phase of the sweep holds. |
| `max_steering_angle` | `0.18` | Max steering angle [rad] during the sweep. |
| `startup_command_topic` | `/teleop` | High-priority mux input topic used for the sweep. |

`use_sim_time` is deliberately **not** in `stack_params.yaml` — it's declared per-file
with a locally-appropriate default, since real hardware and simulation genuinely need
different values and a shared default would be wrong for one of them.

## Component supervisor: restart/control service

`supervisor_bringup.launch.py` starts exactly one node, `component_supervisor_node`,
which spawns every registered component (see `components.yaml`) as its own `ros2
launch` subprocess in its own process group, and exposes two services:

```bash
# Kill-and-relaunch one component
ros2 service call /restart_component f1tenth_messages/srv/RestartComponent \
    "{component_name: 'navigation'}"

# Fine-grained control: SHUTDOWN=0 (kill, stays down, no auto-respawn),
# START=1 (spawn if not running), RESTART=2 (kill-if-running then respawn)
ros2 service call /component_supervisor_node/control_component \
    f1tenth_messages/srv/ComponentControl "{component_name: 'navigation', action: 0}"
```

Registered components: `hardware`, `localization`, `perception`, `control`,
`navigation`, `behavior`, `diagnostics`, `intelligence`, `dev_tools`,
`startup_sequence` (all auto-start except `behavior`/`intelligence`, which are
gated on `use_behavior_tree`/`enable_llm` respectively — `navigation` always
auto-starts, its own `navigation.launch.py` branches Nav2 vs. `mpc_corr` on
`enable_nav2` itself), plus `calibrate_hardware` (registered but never
auto-started — on-demand recalibration only, via `restart_component`).

A watchdog polls every tracked process; anything that exits on its own (a crash, not
a requested stop) gets auto-respawned, up to `max_auto_restarts` within a trailing
`restart_budget_window_sec` — past that, the watchdog leaves it down and logs clearly
until you `START`/`RESTART` it manually.

## BT priority-lane architecture

`f1tenth_behavior`'s `behavior_executor_node` runs a py_trees tree with a root
Selector of 3 priority lanes, ticked continuously (`bt_loop_duration_ms`, default
100ms) — first child that succeeds wins:

1. **`emergency`** — `(IsBatteryLow OR IsSystemOverheated) → Stop`. `IsBatteryLow` is
   unconditional (battery safety is never gateable). `IsSystemOverheated` only exists
   in the tree at all if `enable_sys_obs` is true — when disabled, it's structurally
   absent, not just failing, so "sys_obs off" can never be mistaken for "system
   healthy."
2. **`handle_obstacle`** — `IsObstacleDetected → Stop`. Corridor check against
   `/camera/detections_3d`.
3. **`navigation`** — `HasGoalPose → NavigateThroughPosesClient`. Does nothing until a
   `/goal_pose` message has actually been received (e.g. RViz2's "2D Goal Pose" tool).

Both `emergency` and `handle_obstacle` publish onto the same `safety_stop` topic via
the same generic `Stop` behaviour — the mux doesn't care which BT branch published,
only that something did.

`ackermann_mux` (`f1tenth_bringup/config/mux.yaml`) arbitrates the final drive command
by priority:

| Lane | Topic | Priority |
|---|---|---|
| `safety_stop` | `safety_stop` | **200** (highest — beats everything) |
| `joystick` | `teleop` | 100 |
| `navigation` | `drive` | 10 (lowest) |

So a BT emergency-stop or obstacle-stop always wins over manual joystick input, which
in turn always wins over autonomous driving.
