# Phase 3: Isaac sim mode for the stack

Branch `jazzy-campaign` in the worktree `~/dev_ws/f1tenth_more_jc`. The sim stack is the car's stack. Only parameters differ: the device drivers are off and every node runs on `/clock`. Decisions D9-D20 are in `decisions.md`.

**Back-port note.** This change must later go to `campaign-final` (D9). It uses only launch and launch_ros features that Humble also has: `SetParameter`, `SetParametersFromFile`, `OpaqueFunction`, and `IncludeLaunchDescription` with `PathJoinSubstitution`.

## 1. How to run

```bash
# one machine (linus): Isaac in the background, the stack in the foreground. Ctrl+C stops both.
scripts/sim/sim_isaac.sh all --scene base_room

# two machines (direct cable; F1TENTH_SIM_DISCOVERY_IP defaults to 10.42.0.2, the Thor):
scripts/sim/sim_isaac.sh isaac --net eth --scene base_room    # linus
scripts/sim/sim_isaac.sh stack --net eth                      # Thor
scripts/sim/sim_isaac.sh reset                                # /sim/reset, either machine

# by hand, without the script (same as the car plus two arguments):
ros2 launch f1tenth_bringup supervisor_bringup.launch.py sim:=isaac [use_sim_time:=true]
```

`scripts/sim/sim_isaac.sh --help` lists every option.

The script's pre-checks, before anything starts:
- docker reachable, and the `nvidia` runtime present;
- image `isaac_ws_devcontainer-isaac-sim:latest` present (never pulled);
- at least 25 GB free on /;
- GPU name and VRAM used/total printed;
- no `f1t_isaac` container already running.

`stack` starts the Discovery Server first, then waits up to 30 s for `/clock` (300 s in `all`). If `llama_server_path` does not exist it adds `enable_intelligence:=false` (D19). On hosts with fewer than 12 CPUs it adds the host overlay (D14).

Script conventions follow jazzy's `scripts/sim/`: bash re-exec, `set -eo pipefail`, `--clean`, `--dry-run`, `== ... ==` / `cmd:` echo lines, and help taken from the header. It is shellcheck 0.11.0 clean.

## 2. What `sim:=isaac` changes

| component (components.yaml) | car | `sim:=isaac` |
|---|---|---|
| hardware | `vesc.launch.py` (VESC driver, battery check, ackermann_to_vesc, vesc_to_odom, base_link->imu) | `sim_hardware_tf.launch.py`: only `static_baselink_to_imu`, with the same arguments (test checks this via the AST). Isaac stamps the IMU `imu`; on the car the frame_id is empty, which robot_localization reads as base_link |
| perception | `lidar.launch.py` (urg_node), `camera.launch.py` (ZED wrapper + base_link->zed2_camera_link), detection, ... | lidar dropped; camera replaced by `sim_camera_tf.launch.py` (the deployed mount `static_baselink_to_zed2` + `zed2/zed2_state_publisher` for the ZED frame tree, v4 frame names as on the car); the rest unchanged |
| every component | `ros2 launch <pkg> <file> k:=v` | `ros2 launch f1tenth_bringup sim_component.launch.py component_package:=<pkg> component_launch_file:=<file> sim_params_file:=<isaac.yaml> k:=v` |
| supervisor | wall time | wall time (D10) |
| calibration | `~/run_calibration` | refused (no hardware) |

The Isaac sim itself publishes:
- `/scan`, `/odom` (no TF), `/sensors/imu/raw`;
- `/camera/image_raw` + `/camera/camera_info`, where the car has the `camera.launch.py` relay of the ZED's rectified image;
- the ZED depth topics, `/clock`, and ground truth.

It consumes `/ackermann_drive`, the ackermann_mux output, which ackermann_to_vesc consumes on the car.

The overlay `config/sim/isaac.yaml` has one comment per entry:

| key | value | why |
|---|---|---|
| diagnostics `mission_logger_runs_dir` | `~/f1tenth_sim_archive` | keep sim runs out of the car's archive |
| `behavior_executor_node.preflight_actuation_node` | `isaac_f1tenth_sim` | preflight needs the node that turns `/ackermann_drive` into motion (D13) |
| `reset_manager.wheel_odom` | `{method: service, service: /sim/reset}` | the car resets wheel odom by restarting vesc_to_odom, which does not run in sim |

## 3. Default launch description: identical (evidence)

`tools/sim/launch_snapshot.py` walks the launch descriptions without starting anything:
- the supervisor's own registry, through the real `ComponentSupervisorNode` constructor with the process start stubbed;
- then every component's command and every Node it resolves to: command line, parameter files inlined, prefix, node name, effective `use_sim_time`.

Before = the worktree at d529557 (Phase 2), built. After = the Phase 3 tree. Paths and temp names are normalised.

| arguments | before vs after |
|---|---|
| (none) | **IDENTICAL** (byte-identical JSON, 0 diff lines); 15 auto-started components, 38 resolved processes |
| `sim:=none use_sim_time:=false` | IDENTICAL |
| `calibration:=true` | IDENTICAL |
| `enable_intelligence:=false` | IDENTICAL |

The files are in `launch_snapshots/`. `isaac_linus.json` is the sim-mode snapshot:
- all 54 nodes, including composables, resolve `use_sim_time: True`;
- `vesc.launch.py`, `lidar.launch.py` and `camera.launch.py` are gone;
- reset_manager gets `isaac.yaml` appended last;
- mpc_corr has prefix `taskset -c 6,7`, yolo `4,5` (linus overlay).

## 4. Tests

- `src/f1tenth_bringup/test/test_sim_mode.py`, 14 tests:
  - drivers removed, and replacements in place;
  - unknown components are rejected;
  - overlay precedence;
  - the supervisor's command is wrapped only in sim;
  - the sim IMU TF equals vesc.launch.py's;
  - the ZED frames use the v4 names.
- `src/f1tenth_behavior/test/test_preflight.py::TestActuationNode`, 3 tests. The default actuation requirement is the very object the car uses.

All 28 tests in both files pass. The bringup package's lint tests (flake8, pep257, copyright) failed before Phase 3 too. Phase 3 adds no flake8 finding in the files it touches (`ament_flake8` per-file counts are equal before and after, apart from a `sys.path` import-order note in the new test).

## 5. TF owners (live, `sim_isaac.sh all`, base_room)

Tool: `tools/sim/tf_owners.py`. Evidence: `phase3_evidence/tf_run1.md`.

| edge | owner | evidence |
|---|---|---|
| map -> odom | `ekf_global/ekf_global_filter_node` | ekf_global.yaml `publish_tf: true`, the only parent of `odom` |
| odom -> base_link | `ekf_filter_node` (exactly one) | ekf.yaml `publish_tf: true`; Isaac `publish_odom_tf: false`; vesc_to_odom does not run; the only parent of `base_link` |
| base_link -> laser | `static_baselink_to_laser` (description.launch.py) | 1 /tf_static message |
| base_link -> imu | `static_baselink_to_imu` (sim_hardware_tf) | 1 /tf_static message |
| base_link -> zed2_camera_link + ZED tree | `static_baselink_to_zed2`, `zed2/zed2_state_publisher` (sim_camera_tf) | 1 message per edge; values = deployed (0.2399, 0, 0.205), q (0, -0.018174, 0, 0.999835) |
| base_link -> chassis, wheels, hinges | `robot_state_publisher` + `joint_state_publisher` (description.launch.py) | as on the car |

What the check found:
- Every child frame has exactly one parent.
- No static edge is carried by two `/tf_static` messages.
- Isaac publishes nothing on `/tf` or `/tf_static`, because of `publish_static_tf=false`.
- The laser is at z 0.20 on this branch; the 2026-10-15 car TF says 0.19. That is campaign-final 7b17e2f's value, not changed by the port. It does not matter for a 2D scan.

## 6. Topic coverage (live)

Tool: `tools/sim/topic_coverage.py`, which reads the graph through the rclpy graph API. It keeps no topic list of its own. Results: `phase3_evidence/topic_coverage_run1.md`. The run found 127 topics on 61 nodes.

**Simulator → stack**, all consumed:
- `/scan`
- `/odom`
- `/sensors/imu/raw`
- `/camera/image_raw`
- `/zed2/zed_node/depth/depth_registered` + `camera_info`
- `/clock`

**Published by the sim, not consumed:**
- `/camera/camera_info` (as on the car, nothing consumes it)
- the left/right rectified images and infos
- ground truth (used by the e2e test)

**Subscribed, no publisher (12).** None of these is a sim gap:

| topic | subscriber | on the car |
|---|---|---|
| `/sensors/core` | diagnostics_server | VESC driver (device). Without samples, `has_data` stays false, so the battery lane never trips (is_battery_low.py) |
| `/imu` | mpc_corr | no publisher in the repo either (the VESC IMU is `/sensors/imu/raw`) |
| `/model/virtual_robot/odometry` | mpc_corr | Gazebo-only legacy source |
| `/test_campaign/logger_status` | mpc_corr | only while a test campaign logger runs |
| `/teleop`, `/calibration_drive` | ackermann_mux | joystick / calibration only |
| `/cmd_vel_nav` | twist_to_ackermann | nav2 lane, nav2 disabled |
| `/set_pose`, `/ekf_global/set_pose` | the EKFs | manual relocalisation |
| `/mission/load_path` | behavior_executor | manual |
| `/slam_toolbox/feedback` | slam_toolbox | RViz interactive markers |
| `/zed2/joint_states` | zed2_state_publisher | the ZED URDF has fixed joints only |

## 7. IMU units (D17)

The car's VESC IMU (`vesc_driver.cpp`, `vesc.yaml`):
- angular_velocity.z: rad/s (`gyro_scale_z` = pi/180, bias removed);
- angular_velocity.x/y: deg/s;
- linear_acceleration: g;
- frame_id: empty.

Isaac publishes SI on all axes, with frame `imu`.

The stack does no unit conversion of its own. `ekf.yaml` `imu0_config` fuses only vyaw, and vyaw is rad/s on both. So Isaac keeps SI, and the stack is unchanged.

Measured live (20 s, parked):
- gyro z noise std 0.0029 rad/s, with Isaac's default noise;
- the published covariance now uses the car's gyro z variance (D16).

## 8. Known sim limitation (D20, isaac_ws D25)

The parked Isaac car creeps backwards: ground truth -6.5 mm/s, yaw rate -0.005 rad/s. Its wheel joints turn although their velocity targets are 0.

Three fixes were tried and reverted: wheel kd, steering kd, and a position brake. The EKF and SLAM see this creep as real motion, which it is, in the sim. The e2e test compares perception against ground truth at the same moment.

## 9. Files

New:
- `src/f1tenth_bringup/launch/sim_component.launch.py`
- `src/f1tenth_bringup/launch/sim_hardware_tf.launch.py`
- `src/f1tenth_bringup/launch/sim_camera_tf.launch.py`
- `src/f1tenth_bringup/config/sim/isaac.yaml`
- `src/f1tenth_bringup/config/sim/hosts/linus.yaml`
- `src/f1tenth_bringup/config/sim/dds/{local.xml, isaac_local.xml, eth.xml.in}`
- `scripts/sim/sim_isaac.sh`
- `tools/sim/{launch_snapshot.py, topic_coverage.py, tf_owners.py}`
- `src/f1tenth_bringup/test/test_sim_mode.py`

Changed:
- `component_supervisor_node.py`: sim mode, wrapper, calibration refusal
- `supervisor_bringup.launch.py`: arguments and docstring
- `reset_manager.launch.py`: `params_overlay`
- `setup.py`: data files
- `mission/preflight.py` and `mission/loader.py`: actuation parameter
- `test_preflight.py`
- `scripts/env/jazzy.sh`: comment only

isaac_ws `f1tenth` (commits on top):
- ba9c29b: `--ros-distro {humble,jazzy}` (default jazzy), `--dds-profile`, discovery env pass-through
- f07f6c3: drive-gain log line and the creep finding
