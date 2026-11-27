# 02 — Architecture

**Identity.** `f1tenth_ws` is a **ROS 2 Jazzy** colcon workspace: a physical 1:10
F1TENTH car (VESC drive, Hokuyo LiDAR, ZED2/webcam, Jetson Orin/Thor) *plus* a
Gazebo Harmonic sim. `README.md:3` still says "Humble" — stale; the distro, `docs/jazzy_*`
and `install/` are Jazzy.

## Core technologies
| Layer | Tech |
|---|---|
| Runtime | ROS 2 Jazzy, DDS = **Fast DDS** (Discovery Server) |
| Build | colcon, `--symlink-install`; mostly `ament_python`, `ament_cmake` only in `f1tenth_messages` (interfaces) and vendored `ackermann_mux` / `husarion_gz_worlds` |
| Language | Python 3.12 first-party; C++ only in submodules |
| Nav | Nav2 (planner/controller/behavior/bt_navigator + one lifecycle manager), slam_toolbox, `robot_localization` EKF, custom MPC (`mpc_corr`) |
| Perception | Ultralytics YOLO (`.engine`/`.pt`), v4l2 or ZED SDK wrapper, `urg_node` |
| Behaviour | `py_trees` 3-lane priority Selector |
| Sim | Gazebo Harmonic + `ros_gz` + `ros2_control` Ackermann controller |
| AI | llama.cpp `llama-server` + `llm_planner_node` (natural language → mission) |
| Deploy | Docker (`cpu`/`gpu`/`jetson-thor`), devcontainer compose, **host networking only** |

## Config / parameter model
Single source of truth: `f1tenth_params/config/stack_params.yaml`; every launch file
imports the `param_defaults` helper from that package. Two distinct parameter classes:

1. **Normal params** — `DeclareLaunchArgument` still declared locally, default pulled
   from that YAML; CLI override and `--show-args` work as expected.
2. **4 stack-wide branching args** — `camera_source` (`zed|webcam`, default **zed**),
   `localization_source` (`ekf|raw_odom`, default **ekf**), `use_behavior_tree`
   (default **true**), `enable_nav2` (default **false** — `mpc_corr` drives, Nav2
   idles). Not declared anywhere: read as plain Python at parse time, so a CLI
   override is **silently ignored**. Change them only by editing the YAML. README's
   table is test-pinned to the YAML by `f1tenth_params/test/test_readme_branching_args.py`.

`use_sim_time` is deliberately *not* centralised (real vs sim need different values).

## Process topology (three entry points)
- `f1tenth_bringup/launch/stack_bringup.launch.py` — one launch, one process tree, no
  crash recovery.
- `f1tenth_bringup/launch/supervisor_bringup.launch.py` — `component_supervisor_node`
  spawns each component as its own `ros2 launch` subprocess in its own process group;
  a watchdog auto-respawns crashes under a sliding-window restart budget;
  `RestartComponent` / `ComponentControl` services. **The component set (and the
  auto-start rules, which live in code, not config) is listed in
  `src/f1tenth_bringup/memory.md` — not duplicated here.**
- `f1tenth_sim/launch/sim_bringup.launch.py` — independent of both.

## Data / control flow
Sensors → VESC odom + IMU → EKF or raw mirror (`map→odom`) → Nav2 **or** `mpc_corr` →
`ackermann_mux` (priority lanes: `safety_stop` **200** > `teleop` **100** > `drive`
**10**) → VESC. BT lanes: `emergency` (battery, unconditional) > `handle_obstacle` >
`navigation`; both safety lanes publish the same `safety_stop`.

## State management
No database and no mutable runtime global state. State lives in: ROS params (loaded at
launch, immutable → restart to change), topics/TF, per-mission rosbag2 +
`f1tenth_logger`, JSON campaigns (`first_test_campaing/`, `test_campaign/`), markdown
reports (`output/`, `docs/`). Transport state is DDS-side: a **5 s participant lease**
in `f1tenth_bringup/config/fastdds_profile.xml`, which every participant must load
itself (`FASTRTPS_DEFAULT_PROFILES_FILE`) — a server cannot impose it on others.

## Test / quality
pytest per package; `colcon test` runs **nothing** unless `setup.py` declares
`extras_require={'test': ['pytest']}`. Marker-gated live tests (e.g. `-m llm`) need the
real hardware/model and will fail against stand-ins.

## Current environment note
Build with **`scripts/colcon_build.sh`**, not bare `colcon build`: setuptools 84.2.1 in
`~/.local` (pip `--user`) shadows the distro's 68.1.2 and dropped `setup.py develop
--uninstall`, which colcon calls for every python package. The wrapper isolates
`setup.py` behind `PYTHONNOUSERSITE=1` in a `--system-site-packages` venv
(`.venv-build/`, gitignored, created on first use) and appends `--symlink-install`.
Restored 2026-11-04: all 22 packages build clean, 19 egg-links, source edits live.
Root cause still present — details in `src/f1tenth_intelligence/llm/memory.md` drift 9.

