# f1tenth_more

ROS 2 software for a 1/10-scale F1TENTH car that drives scripted missions
indoors: sensor bringup, localization, camera and LiDAR perception, a corridor
MPC, a behavior tree that runs JSON missions, layered safety stops, and the
tools around them (process manager, phone app, Telegram bot, mission logger,
replay and analysis).

> **This is `main-jazzy`: `main` ported to ROS 2 Jazzy (Ubuntu 24.04), plus the
> Isaac Sim mode.** It replaces `main` once it has passed on the Thor and in the
> car's Jazzy container. Until then the car runs Humble (`main`).

## Branches

| Branch | ROS 2 | What it holds |
|---|---|---|
| `main` | Humble, Ubuntu 22.04 | The consolidated stack: `campaign-final` plus the LLM2 parameter node, the YOLO GPU pipeline, the comms link and the fixes made on the car (car-sync). Humble fixes land there. |
| **`main-jazzy`** (this one) | Jazzy, Ubuntu 24.04 | `main` ported to Jazzy, plus the Isaac Sim mode. |
| `main-jazzy-rust` | Jazzy, Ubuntu 24.04 | `main-jazzy` plus Rust ports of the hot paths (every switch defaults to Python) and the Docker image for the car. |
| `campaign-final` | Humble | The code the first test campaign ran on. **Frozen**: never commit to it, merge into it or rebase it. |

Each branch contains the one above it: `main` is merged into `main-jazzy`, and
`main-jazzy` into `main-jazzy-rust`. A Humble fix goes on `main` and is merged
forward; Jazzy-only work goes here. This branch is never merged into `main`:
at the cutover it replaces it.

## Status (2026-10-25)

| Step | State |
|---|---|
| Build and tests on linus | Done. Every failing test is in the recorded baseline ([output/jazzy_main/report.md](output/jazzy_main/report.md)). |
| Isaac Sim end-to-end on one machine | `tools/e2e_test.sh`, passed 9/9 on the port ([output/jazzy_port/report.md](output/jazzy_port/report.md)) |
| The car's Jazzy container | Image built on the Orin and checked without actuation; built from `main-jazzy-rust` ([report](https://github.com/sjckness/f1tenth_more/blob/main-jazzy-rust/output/orin_container/report.md) on that branch) |
| Open | The run on the Thor; container parity with sensors; wheels-up and floor tests on the car; the cutover |

## What differs from `main`

| Area | `main` (Humble) | `main-jazzy` |
|---|---|---|
| ZED wrapper | humble-v4.2.5 | v5.4.1, topic remaps for v5 |
| VESC serial library | `transport_drivers` submodule | `serial_driver` from apt |
| DDS | Discovery Server | Discovery Server, and every participant loads `f1tenth_bringup/config/fastdds_profile.xml` (5 s lease). Hand-started processes get it from `scripts/env/jazzy.sh` |
| Simulation | `f1tenth_sim` (Gazebo Fortress), separate bringup | `supervisor_bringup.launch.py sim:=isaac` against Isaac Sim. `f1tenth_sim` has a `COLCON_IGNORE` (Gazebo Harmonic port deferred) |
| Defaults | — | `detection_3d` and `obstacle_projector` read TF from `/tf_static` only; `swept_clearance` starts only on demand (`enable_swept_clearance: false`) |
| Static checks | Python 3.10 (the car) | Python 3.12 |
| Car tooling | — | `f1tenth_ctl` profiles for the container and the Isaac sim; `docker/orin/` for the car's image |

## The car and the machines

| Part | What |
|---|---|
| Drive | VESC motor controller: speed, steering servo, wheel odometry and IMU |
| LiDAR | Hokuyo 2D LiDAR on Ethernet (`urg_node`, `/scan` at 40 Hz) |
| Camera | Stereolabs ZED 2, RGB and depth. A USB webcam is the fallback (`camera_source`) |
| Computer | NVIDIA Jetson AGX Orin (JetPack 6). Native Humble today; Jazzy in a container |

| Machine | Role |
|---|---|
| Orin (`jetson-orin`) | The car. The Jazzy image is built and tested on it. |
| Thor (`jetson-thor`) | Jetson AGX Thor, Ubuntu 24.04 and Jazzy: the stack host for the two-machine Isaac sim. |
| linus | x86_64 workstation, Ubuntu 24.04 and Jazzy: Isaac Sim, builds and test runs, the Telegram bot. |

The Jazzy machines and the car's image install `ros-jazzy-*` from the same ROS
apt snapshot (2026-09-11; `tools/linus/ros_snapshot_move.sh`).

## How the stack fits together

```
sensors      Hokuyo LiDAR /scan        ZED 2 RGB + depth          VESC odometry + IMU
                  │                          │                          │
perception   slam_toolbox (map)         YOLO26 segmentation,        robot_localization EKF
             LiDAR wall tracker         2D -> 3D with depth,        (odom -> base_link)
             corridor extraction        front clearance
                  └────────────┬─────────────┘
                     f1tenth_costmap: occupancy + semantic objects + boundaries
                               │
decision     behavior_executor: py_trees BT, JSON missions  <── f1tenth_ctl, phone app, LLM planner
                               │ /mpc/goal_*, /mpc/hold
control      mpc_corr: corridor MPC (OSQP, RTI), reverse manoeuvre
                               │ /drive (priority 10)
arbitration  ackermann_mux  <── /teleop (100): joystick, phone app
                            <── /safety_stop (200): BT emergency stops, hard_stop_node
                               │
actuation    ackermann_to_vesc -> VESC
```

The BT does not steer. It picks what the car does next (a mission move, a
stop, a reverse) and hands the goal to `mpc_corr`, which plans around walls
and objects and publishes the drive command. Everything that stops the car
publishes on the highest-priority mux lane.

## Packages

**Core stack**

| Package | What it does |
|---|---|
| `f1tenth_params` | Every launch default and its description, in `config/stack_params.yaml`. A dependency-free leaf package that every launch file reads from. |
| `f1tenth_messages` | Custom messages and services (the only `ament_cmake` interface package). |
| `f1tenth_bringup` | The two top-level launch files, `component_supervisor_node`, the Fast DDS Discovery Server and profile, Foxglove bridge, the boot-time steering sweep, the shared hardware config (`vesc.yaml`, `ekf.yaml`, `mux.yaml`, `components.yaml`) and the sim overlays (`config/sim/`). |
| `f1tenth_hardware` | The VESC chain: battery pre-flight gate, `ackermann_to_vesc`, VESC odometry, the driver (submodule `vesc`), covariance calibration. |
| `f1tenth_description` | URDF/xacro, meshes, `robot_state_publisher` and the static sensor TFs. |
| `f1tenth_localization` | `robot_localization` EKF over wheel odometry and the VESC IMU, or raw odometry (`localization_source`). |
| `f1tenth_perception` | ZED 2 or webcam, the LiDAR, YOLO detection and segmentation, depth fusion to 3D, front clearance, the LiDAR front-wall tracker and scene labels. |
| `corridor_perception` | Corridor structure from the 2D LiDAR: line extraction, corridor axis, surface classes. The core is numpy only and runs without ROS. `corridor_perception_c` is a C port of the extractor, loaded through ctypes. |
| `f1tenth_costmap` | Occupancy from slam_toolbox plus a semantic layer of tracked objects, the boundary constraints for the MPC, a PNG render for Foxglove. |
| `f1tenth_navigation` | `map_server`, slam_toolbox (mapping mode) and Nav2. Nav2 is off by default (`enable_nav2`). |
| `f1tenth_control` + `mpc_controller` | The mux wiring, `hard_stop_node` (the last-resort LiDAR stop) and `mpc_corr`, the corridor MPC that drives the car. |
| `f1tenth_behavior` | The behavior tree, the mission loader and the example missions in `missions/`. |
| `f1tenth_diagnostics` | Calibration tools (gyro bias, covariance, steering), `system_observer_node` (CPU, GPU, temperatures), `diagnostics_server_node` (battery). |
| `f1tenth_logger` | Per-mission rosbag2 recording (sqlite3), a SQLite index of the runs, replay and report tools. |
| `llm` (`f1tenth_intelligence/llm`) | `llama-server` bringup, the natural-language mission planner and the LLM2 MPC-parameter node. Both off by default. |
| `f1tenth_more` | Metapackage over every first-party package. |

**Operating the car** (these run beside the stack, not inside it)

| Package | What it does |
|---|---|
| `f1tenth_ctl` | CLI on the car: starts and stops the stack, planner and logger (systemd user units natively, runner processes in the container), reports READY, triggers the e-stop, kills every ROS process. JSON output. |
| `f1tenth_gateway` | Backend of the phone app: REST and a WebSocket, teleop with a 250 ms watchdog, e-stop, missions, the run archive. Runs on the car behind `tailscale serve`. |
| `apps/pwa` | **F1Tenth Lab Control**, the phone app (Vite, TypeScript). |
| `f1tenth_telegram` | Telegram control panel. Runs on linus and reaches the car only over ssh and `f1tenth_ctl`. |
| `f1tenth_comms` | Car-to-base link monitor (heartbeat, hotspot reconnect) and Wi-Fi coverage maps. Off by default (`enable_comms`). |

**Submodules** (not first-party): `vesc` (fork, branch `jazzy`), `ackermann_mux`
(fork), `teleop_tools`, `zed_ros2_wrapper` (v5.4.1), `zed-ros2-interfaces`.

## Repository layout

```
src/                  ROS 2 packages; submodules in src/f1tenth_external/ and src/f1tenth_hardware/vesc
apps/pwa/             the phone app
docker/orin/          the car's Jazzy image (built from main-jazzy-rust)
docker/, .devcontainer/  the older Humble dev images: cpu and gpu (x86_64), jetson-thor (arm64)
scripts/              operational scripts: env/jazzy.sh, sim/ (Isaac), jazzy_parity/ (Humble vs Jazzy
                      parity harness), stackctl.py, kill_ros2.py, YOLO engine export, base/, orin/
tools/                offline replay and analysis, tools/sim/ (e2e checks), tools/e2e_test.sh,
                      tools/car_sync/ (capture what the car runs)
docs/                 setup, procedures, analyses, the project log (docs/history/LOG.md), the PWA spec
first_test_campaign/  missions M01-M05 and the results of the first test campaign
output/               reports and evidence from work sessions: jazzy_port/, jazzy_main/, consolidation/, ...
CLAUDE.md             build, test, DDS and Docker gotchas that have already cost debugging time
```

## Build

```bash
git clone --recurse-submodules -b main-jazzy git@github.com:sjckness/f1tenth_more.git
cd f1tenth_more            # in an existing clone: git submodule update --init --recursive

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

- rosdep does not cover every Python dependency: `osqp` (the MPC), `ultralytics`
  and `torch` (YOLO), `pyarrow`. Install them with pip (a venv on `PYTHONPATH`
  works).
- On a machine without the ZED SDK (linus), leave out the ZED wrapper and the
  VESC driver:
  `colcon build --symlink-install --packages-ignore $(cat output/jazzy_port/linus_skip.txt)`.
  Pass the same `--packages-ignore` to every later `--packages-select` build.
- **After any `ros-jazzy-*` apt upgrade, clean-rebuild every CMake package**: an
  ABI change inside the distro builds fine and fails at startup
  (`undefined symbol`). The commands and the check are in CLAUDE.md,
  "Rebuild policy".
- In every interactive shell, `source scripts/env/jazzy.sh` instead of the bare
  ROS setup file (bash or zsh). It sources ROS and the workspace, loads the
  Fast DDS profile and points the shell at the Discovery Server; without it a
  hand-started `ros2` command sees none of the stack.

## Test

```bash
# Keep test nodes off the stack's graph and away from other runs
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=93 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

colcon test --packages-ignore $(cat output/jazzy_port/linus_skip.txt)
colcon test-result --verbose

# New failures only: compare against the recorded baseline
python3 tools/r3_failing_ids.py build output/jazzy_main/test_failing_ids.txt

# Static checks, no build needed (Python 3.12 syntax, launch, YAML/JSON/XML, stack_params keys, ...)
python3 -I output/consolidation/static_checks.py
```

The baseline (`output/jazzy_main/test_failing_ids.txt`) holds the failures that
predate the port: ament lint in a few packages, the LLM planner stub tests, MPC
tests that pin older defaults, upstream lint in the submodules.

`colcon test` only runs pytest for a package whose `setup.py` declares
`extras_require={'test': ['pytest']}`. Without it, the package reports
`Ran 0 tests ... OK`. Every new `ament_python` package needs that line
(CLAUDE.md, "Testing").

## Run

```bash
ros2 launch f1tenth_bringup supervisor_bringup.launch.py
ros2 launch f1tenth_bringup supervisor_bringup.launch.py --show-args   # every argument
```

`supervisor_bringup.launch.py` starts the Discovery Server and one node,
`component_supervisor_node`. It launches each component from
`f1tenth_bringup/config/components.yaml` as its own `ros2 launch` process:
`hardware`, `localization`, `perception`, `lidar_front_wall`, `control`,
`navigation`, `slam`, `behavior`, `diagnostics`, `dev_tools`,
`startup_sequence`, and the optional ones (`intelligence`, `comms`,
`calibrate_hardware`, ...). A watchdog restarts a crashed component, at most
`max_auto_restarts` times within `restart_budget_window_sec`. This is what
`f1tenth_ctl start stack` runs.

```bash
# Restart one component
ros2 service call /restart_component f1tenth_messages/srv/RestartComponent \
    "{component_name: 'perception'}"

# SHUTDOWN=0 (stays down), START=1, RESTART=2
ros2 service call /component_supervisor_node/control_component \
    f1tenth_messages/srv/ComponentControl "{component_name: 'perception', action: 0}"
```

`stack_bringup.launch.py` is the single-process alternative: one launch tree,
no per-component restart. A new top-level launch file must set
`ROS_DISCOVERY_SERVER` and `FASTRTPS_DEFAULT_PROFILES_FILE` the same way these
two do.

Foxglove connects to the bridge on port 8765.

### Missions

A mission is a JSON list of moves (a distance, a pose, a turn or an open-ended
drive), each ending on a stop condition, with optional reactions to detected
objects. The schema is in
[src/f1tenth_behavior/README.md](src/f1tenth_behavior/README.md); examples are
in `src/f1tenth_behavior/missions/`. Loading a mission does not move the car;
starting it does.

```bash
M=$(ros2 pkg prefix f1tenth_behavior)/share/f1tenth_behavior/missions/drive_test_straight_2m.json
ros2 service call /mission/load_mission f1tenth_messages/srv/LoadMission "{path: '$M'}"
ros2 service call /mission/start_mission std_srvs/srv/Trigger
ros2 service call /mission/abort_mission std_srvs/srv/Trigger
ros2 topic echo /mission/status
```

### On the car

Use `f1tenth_ctl` rather than starting launch files by hand
([src/f1tenth_ctl/README.md](src/f1tenth_ctl/README.md)):

```bash
f1tenth_ctl start stack        # also: planner, logger
f1tenth_ctl status             # processes and READY
f1tenth_ctl estop              # e-stop service + zero drive; never kills a process
f1tenth_ctl estop-reset
f1tenth_ctl stop stack
f1tenth_ctl killall            # every ROS 2 process of this user
```

- **Phone:** the F1Tenth Lab Control app talks to `f1tenth_gateway` over
  Tailscale ([src/f1tenth_gateway/README.md](src/f1tenth_gateway/README.md)).
  **The app's emergency button is not the primary e-stop**: it goes through
  Wi-Fi, Tailscale and the gateway. Whenever the car can move, hold the RC
  transmitter or the physical switch.
- **Telegram:** [src/f1tenth_telegram/README.md](src/f1tenth_telegram/README.md).
- **LLM planner:** `enable_intelligence:=true` starts `llama-server`; then
  `f1tenth_ctl start planner` and `f1tenth_ctl send planner "go ahead 2 meters"`
  ([src/f1tenth_intelligence/llm/README.md](src/f1tenth_intelligence/llm/README.md)).

### Simulation (Isaac Sim)

`sim:=isaac` runs the same bringup as the car: the VESC, LiDAR and ZED drivers
are replaced by Isaac Sim, every node runs on `/clock`, and
`f1tenth_bringup/config/sim/isaac.yaml` is loaded after the normal parameters.
The Isaac side is the [isaac_ws](https://github.com/sjckness/isaac_ws) repo,
branch `f1tenth`, checked out at `~/isaac_ws`.

```bash
# Everything on one machine (linus); Ctrl+C stops both
scripts/sim/sim_isaac.sh all --scene base_room

# Two machines on the direct Ethernet link, Discovery Server on the Thor
scripts/sim/sim_isaac.sh isaac --net eth --scene base_room    # linus
scripts/sim/sim_stack.sh                                      # Thor
scripts/sim/llm.sh "go to the person"                         # Thor: one planner command

scripts/sim/sim_isaac.sh shell        # a shell that sees the whole graph
scripts/sim/sim_isaac.sh reset        # /sim/reset

# End-to-end test: Isaac + stack up, graph, rates, TF, perception, a 2 m mission
tools/e2e_test.sh
```

`sim_isaac.sh` refuses to start Isaac with less than 25 GB free on `/`. Scenes
and layouts: `sim_isaac.sh --help`.

## Configuration

`src/f1tenth_params/config/stack_params.yaml` holds the default and the
description of every launch parameter (about 650 keys). Override one on the
command line (`key:=value`), or change the default in that file.

Four stack-wide values are read from the file when the launch files are parsed
and have **no launch argument**: `camera_source`, `localization_source`,
`use_behavior_tree` and `enable_nav2`. Passing them on the command line is
silently ignored; edit the file.

| Key | Default | Meaning |
|---|---|---|
| `camera_source` | `zed` | `zed` or `webcam` |
| `localization_source` | `ekf` | `ekf` or `raw_odom` |
| `use_behavior_tree` | `true` | Run the behavior tree |
| `enable_nav2` | `false` | `false`: `mpc_corr` drives. `true`: bring up Nav2 |
| `enable_slam` | `true` | slam_toolbox mapping, the occupancy layer of the costmap |
| `enable_mission_logger` | `true` | Record a rosbag2 per mission |
| `yolo_model` | `yolo26s-seg.pt` | YOLO weights; `yolo_backend: engine` uses a TensorRT engine (`scripts/export_yolo_engine.sh`) |
| `yolo_gpu_pipeline` | `true` | Pre- and post-processing on the GPU, detection capped at 15 Hz |
| `enable_intelligence` | `false` | `llama-server` and the LLM mission planner |
| `enable_llm2` | `false` | Second LLM that proposes MPC parameters |
| `enable_comms` | `false` | Car-to-base link monitor |
| `enable_swept_clearance` | `false` | Start `swept_clearance` with the stack instead of on demand |
| `lidar_front_wall_door_detection_enabled` | `false` | Door detection in the LiDAR wall tracker |
| `enable_foxglove` | `true` | `foxglove_bridge` on port 8765 |

A test that asserts a default is coupled to this file: change both in the same
commit. Launch writes parameters through YAML, so a string parameter never
takes `on`, `off`, `yes` or `no` as a value (YAML makes them booleans).

## Safety layers

`ackermann_mux` (`f1tenth_bringup/config/mux.yaml`) forwards the
highest-priority lane that is publishing:

| Lane | Topic | Priority | Who publishes |
|---|---|---|---|
| `safety_stop` | `/safety_stop` | 200 | BT emergency lane, BT obstacle lane, `hard_stop_node` |
| `joystick` | `/teleop` | 100 | Joystick, phone app |
| `calibration` | `/calibration_drive` | 50 | Steering calibration |
| `navigation` | `/drive` | 10 | `mpc_corr` (or Nav2) |

- **BT emergency lane:** low battery, the operator e-stop
  (`/mission/emergency_stop`, latched), overheating (with `enable_sys_obs`)
  and a raw-sensor proximity check on the ZED depth and `/scan`, independent of
  YOLO.
- **BT obstacle lane:** a stop on camera detections, off by default
  (`enable_camera_obstacle_stop`).
- **`hard_stop_node`:** a stop on the raw `/scan`, every scan, outside the BT.
  It keeps working if the BT is down.

Each stop sets its own `header.frame_id` (`base_link/emergency`,
`base_link/obstacle`, `base_link/hard_stop`). The mux forwards the message
unchanged, so a bag shows which layer stopped the car.

## More documentation

| Topic | Where |
|---|---|
| The Jazzy port: decisions, per-package results, Isaac sim mode | [output/jazzy_port/report.md](output/jazzy_port/report.md) |
| `main-jazzy` build and test results | [output/jazzy_main/report.md](output/jazzy_main/report.md) |
| Package by package (nodes, topics, parameters) | [src/f1tenth_more/README.md](src/f1tenth_more/README.md) |
| Mission JSON schema | [src/f1tenth_behavior/README.md](src/f1tenth_behavior/README.md) |
| Perception (YOLO, depth, walls) | [src/f1tenth_perception/README.md](src/f1tenth_perception/README.md) |
| Calibration tools | [src/f1tenth_diagnostics/README.md](src/f1tenth_diagnostics/README.md) |
| MPC model-validation logging | [docs/DIAGNOSTICS.md](docs/DIAGNOSTICS.md) |
| Phone app spec | [docs/pwa/SPEC.md](docs/pwa/SPEC.md) |
| Comms link setup | [src/f1tenth_comms/README.md](src/f1tenth_comms/README.md) |
| What `main` is made of, the bug backlog | [output/consolidation/report.md](output/consolidation/report.md), [known_bugs.md](output/consolidation/known_bugs.md) |
| Project log | [docs/history/LOG.md](docs/history/LOG.md) |
| Workspace gotchas (DDS, rebuilds, Docker on the Orin) | [CLAUDE.md](CLAUDE.md) |

## License

MIT, see [LICENSE](LICENSE).
