# 01 — Directory Map

Strict map: **domain → path**. Resolve a question by domain here, then open only the
files under that path. `build/ install/ log/ output/` are generated — never look there
for source. `src/f1tenth_external/**` and `src/f1tenth_hardware/vesc/` are pinned git
submodules (read-only; not first-party code).

## Entry points / orientation
| What | Where |
|---|---|
| Workspace overview, param reference, bringup choice table | `README.md` |
| Hard-won gotchas (DDS lease, test discovery, rebuild policy, Thor) | `CLAUDE.md` |
| Assistant memory (this bank) | `memory-bank/` |
| Narrative docs: setup, procedures, analyses, history | `docs/{setup,procedures,analysis,history,topic_optimization}/` |
| Model-validation / CSV diagnostics doc | `docs/DIAGNOSTICS.md`, `docs/wall_turn_investigation.md` |

## First-party ROS packages (`src/`)
| Domain | Path |
|---|---|
| **Launch-param defaults — single source of truth** | `src/f1tenth_params/config/stack_params.yaml`; helper: `src/f1tenth_params/f1tenth_params/` |
| **Top-level bringup, component supervisor, Foxglove, startup steer sweep** | `src/f1tenth_bringup/launch/` (`stack_bringup`, `supervisor_bringup`, `startup_sequence`, `foxglove_bridge`, `reset_manager`, `sim_component`); contracts → `src/f1tenth_bringup/memory.md` |
| **Shared hardware/runtime config**: VESC cal, EKF, mux priority, joy teleop, LiDAR, DDS profile, component registry | `src/f1tenth_bringup/config/` (`vesc.yaml`, `ekf.yaml`, `ekf_global.yaml`, `mux.yaml`, `joy_teleop.yaml`, `sensors.yaml`, `fastdds_profile.xml`, `components.yaml`); contracts → `src/f1tenth_bringup/memory.md` |
| **VESC drive chain** (battery pre-flight gate, calibration, `ackermann_to_vesc`, `vesc_to_odom`, `vesc_driver`, `base_link→imu` TF) | `src/f1tenth_hardware/f1tenth_hardware/` |
| **Vendored VESC driver + msgs** | `src/f1tenth_hardware/vesc/` (submodule) |
| **Robot model**: URDF/xacro, meshes, `robot_state_publisher`, hardware static sensor TFs | `src/f1tenth_description/{urdf,meshes,config,launch}/` |
| **Localization `map→odom`**: EKF vs raw-odom mirror | `src/f1tenth_localization/launch/` (`localization`, `ekf`, `ekf_global`, `raw_odom`); config `f1tenth_bringup/config/ekf*.yaml` |
| **Camera** (ZED2 wrapper / v4l2 webcam) + **YOLO detection fusion** + LiDAR/wall tools | `src/f1tenth_perception/{launch,config,models,scripts}/`; contracts → `src/f1tenth_perception/memory.md` |
| **Navigation**: Nav2 stack, orchestrators, maps | `src/f1tenth_navigation/launch/` (`navigation`, `nav2`, `map`, `map_only`, `planner`, `controller`, `behavior_server`, `bt_navigator`, `slam`); params `config/{nav2_params,slam_toolbox_params}.yaml`; maps `maps/` |
| **MPC controller** (drive source when `enable_nav2:=false`) | `src/f1tenth_control/mpc_controller/` |
| **Control metapackage** | `src/f1tenth_control/f1tenth_control/` |
| **Drive-command arbitration (`ackermann_mux`)** | vendored `src/f1tenth_external/ackermann_mux/`; config `f1tenth_bringup/config/mux.yaml` |
| **Behaviour tree** (emergency / obstacle / navigation lanes, Nav2 gate, Twist→Ackermann) | `src/f1tenth_behavior/f1tenth_behavior/`, `launch/`, `config/twist_to_ackermann.yaml` |
| **Mission definitions (JSON, loaded via `/mission/*`)** | `src/f1tenth_behavior/missions/` |
| **Costmap** (occupancy from slam_toolbox + semantic from detections) | `src/f1tenth_costmap/` |
| **Diagnostics**: system observer, battery/diagnostics server, calibration | `src/f1tenth_diagnostics/{launch,config}/` |
| **Custom interfaces** (msg/srv) | `src/f1tenth_messages/{msg,srv}/` |
| **LLM intelligence**: `llama-server` bringup, NL mission planner, prompts, schema | `src/f1tenth_intelligence/llm/{llm,launch,config,schemas,prompts,test}/` |
| **Run logging / per-mission rosbag2, campaign logger** | `src/f1tenth_logger/{launch,config}/` |
| **Simulation** (Gazebo Harmonic bringup, sim sensors/bridges, controllers) | `src/f1tenth_sim/{launch,config,worlds,models}/` |
| **Gazebo worlds/models (vendored)** | `src/husarion_gz_worlds/{worlds,models,launch,maps}/` |
| **Camera pan servo** | `src/f1tenth_camera_pan/` |
| **Corridor perception** (`package.xml` description empty) | `src/corridor_perception/` |
| **Minimal demo package** (own URDF + worlds; description empty) | `src/f1tenth_minimal/{urdf,worlds,launch,config}/` |
| **Top-level metapackage** | `src/f1tenth_more/` |
| **Other vendored submodules**: joystick teleop, ZED wrapper + interfaces, serial transport | `src/f1tenth_external/{teleop_tools,zed_ros2_wrapper,zed-ros2-interfaces,transport_drivers}/` |

## Non-ROS / support
| Domain | Path |
|---|---|
| Host probes & one-off scripts (CPU pinning, EKF rate, clock, `kill_ros2`, `stackctl`, `drive_3m`, `watch_objects`) | `scripts/` |
| **DDS shell env** — sets `ROS_DISCOVERY_SERVER` + `FASTRTPS_DEFAULT_PROFILES_FILE`; source it for anything started by hand | `scripts/env/jazzy.sh` |
| Isolation / DDS-lease / watchdog experiment artefacts | `scripts/fix_batch_{1,4,5}/`, `scripts/jazzy_parity/`, `scripts/sim/` |
| Container launch helpers | `scripts/docker_{build,run}.sh`, `scripts/{run,resume}_container.sh` |
| Offline analysis (MPC model check, stop distance, bag replay, campaign pruning, prompt regression) | `tools/`; MATLAB `tools/matlab/+f1db/` |
| Live campaign driver + scenarios | `test_campaign/` (`prompts.json`, `run_dry_campaign.py`, `runs/`), `tools/run_scenarios.sh` |
| Historical campaign results/analysis (images, CSVs) | `first_test_campaing/` (`M0*` run dirs, `analysis/`) |
| Generated markdown reports (phase reports, backlogs, reviews) | `output/` |
| Images: CPU / GPU / Jetson-Thor, plus Orin build inputs and gate scripts | `docker/` (`Dockerfile.*`, `orin/`, `orin/spike/gates/`) |
| Dev containers (compose, host net, `/dev` bind) | `.devcontainer/` (`docker-compose.yml`, `cpu/`, `gpu/`, `jetson-thor/`) |
| CI issue templates | `.github/ISSUE_TEMPLATE/` |
| Editor config | `.vscode/` |
| Generated by colcon — ignore | `build/`, `install/`, `log/` |

## Fast lookups
| Question | Answer |
|---|---|
| Change a launch default | `src/f1tenth_params/config/stack_params.yaml` |
| Add/remove a supervised component | `src/f1tenth_bringup/config/components.yaml` **+** a `_*_AUTO_START` set in `component_supervisor_node.py` (same commit) — see `src/f1tenth_bringup/memory.md` |
| Supervisor/component-set/bringup/Foxglove/viz/teleop contracts | `src/f1tenth_bringup/memory.md` |
| Who wins between joystick and autonomy | `src/f1tenth_bringup/config/mux.yaml` (200 / 100 / 10) |
| Node never sees peers / 5 s lease | `src/f1tenth_bringup/config/fastdds_profile.xml` + `scripts/env/jazzy.sh` |
| Where Nav2 vs MPC is chosen | `src/f1tenth_navigation/launch/navigation.launch.py` |
| Which TF owns `base_link→laser` on hardware | `src/f1tenth_description/` (static publishers, **not** URDF sensors) |

