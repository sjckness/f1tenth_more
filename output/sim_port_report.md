# f1tenth_sim port: Ignition Fortress → Gazebo Harmonic (ROS 2 Jazzy)

Host: simulation PC (Ubuntu 24.04). Branch: `jazzy-sim`, created from `origin/jazzy` @ `fc1ae6f`.
Status (2026-10-02, after a session resume): **Steps 0–3 done and committed. Step 5 written (§5). Step 4 (run and
verify) is blocked on the ROS install, which needs an interactive sudo password: run `scripts/sim_host_setup.sh`.**
Disk is now 28 GB free (user freed space; precondition ≥ 25 GB met). `f1tenth_sim/COLCON_IGNORE` is still in place
on purpose until Step 4 passes (§4).

Tags used below: **[code]** = read from source in this repo, **[doc]** = official upstream docs,
**UNVERIFIED** = an assumption not yet checked against a running system.

---

## ⚠ Priority real-car item (outside this port, found during it)

**P2. The real wheelbase must be measured.** The URDF and `drive_bridge` use 0.325 m (`robot.urdf.xacro`,
`core.xacro` via `wheels`/hinge origin x = 0.325, `controllers.yaml`). The odometry that actually drives the car uses
`vesc_to_odom_node.wheelbase: .305` (`f1tenth_bringup/config/vesc.yaml`). That is a 6 % disagreement in the yaw
rate computed by `vesc_to_odom` (ω = v·tanδ/L) and in MPC's bicycle model if it uses either value. The sim keeps
0.325 for now (D5).

(The earlier P1, the empty IMU frame_id, was downgraded on 2026-10-02 because the source check shows the IMU is fused
correctly. It is now backlog item **B1** at the end of this report.)

---

## 0. Clean clone and host state

### 0.1 Old clone
`~/dev_ws/f1tenth_more` **did not exist** on this PC. A search (`find / -xdev -maxdepth 6 -name 'f1tenth_more*'`,
plus every git repo under `~/dev_ws`) found no clone anywhere. The repos in `~/dev_ws` are `kumi`, `kumi_stack`,
`msauber`, `msauber_stack`; none point at `sjckness/f1tenth_more`.
So there was nothing to inspect, nothing that existed only in the old clone, and nothing to move aside. Step 0.2 (`mv … _old_<date>`) was skipped
because there was no source directory.

### 0.3 Fresh clone
```
git clone --recurse-submodules -b jazzy git@github.com:sjckness/f1tenth_more.git ~/dev_ws/f1tenth_more
HEAD          fc1ae6fa323d452855e1b4cbf8a63da9525e791f
origin/jazzy  fc1ae6fa323d452855e1b4cbf8a63da9525e791f   → equal
```
`git submodule status` (no `-`/`+` prefixes):
```
 b3c0b08 src/f1tenth_external/ackermann_mux        (heads/foxy-devel)
 4337558 src/f1tenth_external/teleop_tools         (1.2.1-15-g4337558)
 e970087 src/f1tenth_external/zed-ros2-interfaces  (5.3.0)
 86171bc src/f1tenth_external/zed_ros2_wrapper     (v5.4.1)
 2848a9f src/f1tenth_hardware/vesc                 (remotes/origin/jazzy)
```
There is no `transport_drivers` submodule (checked against `.gitmodules`). Small oddity: `.gitmodules` pins
`ackermann_mux` to `branch = jazzy`, but the checked-out commit describes as `heads/foxy-devel`. The gitlink
commit is what counts, so this is cosmetic.

### 0.4 Work branch
`git checkout -b jazzy-sim`, done. Nothing has been edited yet apart from this report.

### 0.5 Host

| Item | Value |
|---|---|
| OS | Ubuntu 24.04.4 LTS, kernel 7.0.0-31 |
| ROS 2 | **Not installed.** `/opt/ros` does not exist and `ros2` is not on PATH. Only `ros-dev-tools` and `ros-build-essential` are installed. The ROS apt source (`packages.ros.org/ros2/ubuntu noble`) is configured and rosdep is initialised. Other projects on this PC run ROS in Docker (`msauber_ros2_jazzy`, `nuc_forzaeth_racestack_ros2:jazzy`). |
| Gazebo | Standalone **Gazebo Harmonic from packages.osrfoundation.org** is installed: `gz-harmonic 1.0.0`, `gz-sim8 8.11.0` (`gz sim --versions` → 8.11.0). |
| ros_gz | `ros-jazzy-ros-gz`: Installed **(none)**, Candidate 1.0.24-1noble.20260905 |
| gz_ros2_control | `ros-jazzy-gz-ros2-control`: Installed **(none)**, Candidate 1.2.20-1noble.20260905 |
| CPU / RAM | i7-1165G7 (4C/8T), 15 GiB RAM |
| GPU | Hybrid graphics. Intel Iris Xe (TGL GT2) is the active OpenGL renderer (Mesa 25.2.8). NVIDIA GeForce MX450 (2 GB) runs driver 580.173.02 in `prime-select on-demand` mode. |
| Disk | **`/` is 97 % full: 15 GB free of 468 GB** |
| Network | `enp0s31f6` (wired) is **DOWN**, so no cable is connected yet. `wlp0s20f3` 192.168.1.90/24 and `tailscale0` are up. No `ROS_*`/`RMW_*`/`GZ_*` variables are set in the shell or rc files. |

### 0.6 Is this the right machine? Evidence says probably not the earlier migration PC

| Evidence | This PC | Earlier x86 migration PC (`output/jazzy_migration_analysis.md`, `trial_build.log`) |
|---|---|---|
| Hostname / model | `TPad`, Lenovo ThinkPad T15 Gen 2i | not recorded |
| ROS Jazzy | **Purged 2026-08-31 14:49** (`/var/log/apt/history.log`: `apt purge ros-jazzy-*`; zsh history has the same three purge commands). There has been no ROS install since; only unattended-upgrades are in apt history after that date. | "Host: Ubuntu 24.04.4 / x86_64 / Python 3.12.3 / ROS 2 Jazzy", generated **2026-09-30** |
| f1tenth_more clone | Never present. `~/.zsh_history` has no `f1tenth` entry at all. | Build paths `/home/andreas/dev_ws/f1tenth_more/...` |
| git identity | global `user.name = andreas` | all jazzy commits are authored by `andreas-linus` |

The migration analysis ran on 2026-09-30 on an x86_64 host with Jazzy and a clone at the same path. That is a month
after Jazzy was purged here, and this PC never had the clone. So **the earlier migration PC is a different machine**.
That is an inference, not proof: the hostname of that machine is not recorded anywhere I can see.

### 0.7 Disk on this PC (read-only survey; nothing deleted)

`/` = 468 GB, 430 GB used, **15 GB free**. Target ≥ 40 GB free means freeing **≥ 25 GB**.
`du -x /` sees only 241 GB. The ≈189 GB gap is root-only `/var/lib/docker` (no sudo here), which matches
`docker system df` (≈187 GB).

| Where | Size | Notes (nothing removed) |
|---|---|---|
| **Docker images** | 109.2 GB (7 images) | `msauber_ros2_jazzy` 30.5 GB (25.6 unique, used by 2 exited containers), `vsc-msauber_stack-…` 29 GB (24.1 unique, **no container**), `nuc_forzaeth_racestack_ros2:jazzy` 9.6 GB, `kumi:dev` 7.9 GB and `vsc-kumi_stack-…` 7.9 GB (6.0 shared), **dangling `<none>` 74b59706947f 6.4 GB** (2.1 unique), `hello-world`. `docker system df` says 28.1 GB reclaimable. |
| **Docker build cache** | 71.8 GB (103 entries) | `docker buildx du`: Reclaimable 71.84 GB, but **Shared 71.77 GB**, i.e. the layers are shared with existing images. A `docker builder prune` would likely free much less than 71 GB while those images exist. UNVERIFIED how much. |
| Docker containers | 5.2 GB (4, all exited 4–7 months ago) | `kumi_stack_devcontainer-kumi-1` 2.6 GB, `reverent_goldstine` 2.6 GB, `msauber`, `nuc_forzaeth_racestack_ros2_jazzy` |
| `~/.steam` | 37 GB | Skate 8.2 GB, Fallout New Vegas 6.8 GB, The Forest 5.6 GB, Proton 1.4 GB, … |
| `~/model_training` | 20 GB | – |
| `~/Documents` | 17 GB | – |
| `~/Politecnico Di Torino Studenti Dropbox` | 16 GB | synced folder |
| **`~/.local/share/Trash`** | **15 GB** (16 items) | already in the trash |
| `~/.cache` | 15 GB | **pip 8.6 GB**, vscode-cpptools 4.2 GB, Raspberry Pi 1.3 GB, tracker3 0.5 GB |
| `~/Downloads` | 9.5 GB | `ubuntu-20.04.6-desktop-amd64.iso` 4.1 GB, Notebooks.zip 0.9 GB, … |
| `~/my_py_venv` | 8.8 GB | – |
| `/var/lib/snapd` | 7.8 GB | 18 **disabled** (old) snap revisions are kept (blender, firefox, core18/20/22/24, telegram, …) |
| `~/.arduino15` | 7.6 GB | – |
| `~/webDev`, `~/image_processing` | 6.0 GB, 5.4 GB | – |
| `~/.npm` | 4.1 GB | cache |
| `/var/cache/apt`, `/var/log/journal` | 1.1 GB, 0.6 GB | – |

Low-risk candidates that together comfortably exceed 25 GB (your call, nothing done):
- Trash: 15 GB.
- Dangling Docker image plus the unused `vsc-msauber_stack` image: about 26 GB unique.
- Exited containers: 5 GB.
- pip cache: 8.6 GB.
- The Ubuntu 20.04 ISO: 4 GB.
- Disabled snap revisions: size UNVERIFIED.

### 0.5 Install list (D0: native apt install approved, gated on §0.6 and §0.7)

The rosdep keys of `f1tenth_description`, `f1tenth_messages` and `f1tenth_params` (`rosdep install --simulate
--rosdistro jazzy --skip-keys ament_python`; `ament_python` as a `<buildtool_depend>` is not a rosdep key, a
pre-existing quirk) resolve to:
```
ros-jazzy-ament-copyright ros-jazzy-ament-flake8 ros-jazzy-ament-pep257 ros-jazzy-ament-index-python
ros-jazzy-launch ros-jazzy-launch-ros ros-jazzy-xacro ros-jazzy-robot-state-publisher
ros-jazzy-joint-state-publisher ros-jazzy-tf2-ros ros-jazzy-std-msgs ros-jazzy-geometry-msgs
ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common ros-jazzy-ament-cmake-auto
ros-jazzy-rosidl-default-runtime ros-jazzy-rosidl-default-generators
```
f1tenth_sim after the port (rosdep keys resolved one by one, since the current package.xml still names `ros_ign_*`):
```
ros-jazzy-ros-gz (meta: ros-gz-sim, ros-gz-bridge, ros-gz-image, ros-gz-interfaces + Harmonic *-vendor pkgs)
ros-jazzy-gz-ros2-control ros-jazzy-controller-manager ros-jazzy-joint-state-broadcaster
ros-jazzy-ackermann-steering-controller ros-jazzy-ackermann-msgs ros-jazzy-nav-msgs ros-jazzy-sensor-msgs
ros-jazzy-rclpy
```
Base plus tools for the tests: `ros-jazzy-ros-base ros-jazzy-rmw-fastrtps-cpp ros-jazzy-tf2-tools ros-jazzy-rviz2`
(RViz is optional). `ros-jazzy-foxglove-bridge` is optional and only needed if the PC hosts Foxglove.

Proposed command (one line):
```
sudo apt-get install -y ros-jazzy-ros-base ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager ros-jazzy-joint-state-broadcaster ros-jazzy-ackermann-steering-controller \
  ros-jazzy-ackermann-msgs ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  ros-jazzy-tf2-ros ros-jazzy-tf2-tools ros-jazzy-rviz2 ros-jazzy-foxglove-bridge ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-ament-copyright ros-jazzy-ament-flake8 ros-jazzy-ament-pep257 ros-jazzy-ament-lint-auto \
  ros-jazzy-ament-lint-common ros-jazzy-ament-cmake-auto ros-jazzy-rosidl-default-generators
```
`apt-get -s` reports **536 new packages, 0 removals, 0 upgrades**. The summed `Installed-Size` is about 0.9 GB; budget
1–2 GB including the download cache, against 15 GB free.

**Note on two Gazebo installs:** `ros-jazzy-ros-gz` pulls in the ROS-built Harmonic *vendor* packages
(`ros-jazzy-gz-sim-vendor`, `-gz-physics-vendor`, `-gz-rendering-vendor`, …) under `/opt/ros/jazzy/opt/`. The
OSRF `gz-harmonic` already installed stays as it is. The Gazebo docs name Jazzy+Harmonic via `ros-jazzy-ros-gz` as
the recommended pairing [doc: gazebosim.org/docs/harmonic/ros_installation]. The mixing warning there applies only
to *non-default* pairings. After sourcing `/opt/ros/jazzy/setup.zsh`, `gz sim` resolves to the vendor build.
UNVERIFIED that the two coexist without plugin-path confusion; Step 4 will check `which gz` and `gz sim --versions`
under the sourced environment.

---

## 1. Inventory

### 1.1 f1tenth_sim as it stands (ament_python, `COLCON_IGNORE` since `0607db8`)

| File | Content |
|---|---|
| `launch/sim_bringup.launch.py` | Starts, in order: (1) `ros_ign_gazebo/ign_gazebo.launch.py` with `ign_args='-r -v 4 empty_room.sdf'`; (2) `f1tenth_description/description.launch.py` with `use_sim:=true enable_sensors:=true control_config:=controllers.yaml`, i.e. **robot_state_publisher on /tf, /tf_static**; (3) `ros_ign_bridge parameter_bridge` with a YAML `config_file`; (4) at t=4 s, `ros_ign_gazebo create -name roboracer -string <xacro> -z 0.1`; (5) at t=8 s, spawners for `joint_state_broadcaster` and `ackermann_steering_controller`; (6) `drive_bridge`; (7) `foxglove_bridge` :8765; (8) `slam_toolbox` async (f1tenth_bringup config); (9) `robot_localization ekf_node` (f1tenth_bringup `ekf.yaml`). It sets both `IGN_GAZEBO_RESOURCE_PATH` and `GZ_SIM_RESOURCE_PATH`. It uses `f1tenth_bringup`'s share directory **without declaring it in package.xml**. |
| `config/ros_gz_bridge.yaml` | GZ→ROS only: `/clock` (`ignition.msgs.Clock`), `/scan` (`ignition.msgs.LaserScan`), `/sensors/imu/raw` (`ignition.msgs.IMU`), and three ZED topics: rgb image, `/zed2/zed_node/rgb/camera_info` (gz `…/image_rect_color/camera_info`), and depth (`ignition.msgs.Image`/`CameraInfo`). |
| `config/controllers.yaml` | `controller_manager` at `update_rate: 100`, `use_sim_time`. `joint_state_broadcaster`. `ackermann_steering_controller` with `traction_joints_names` [right/left_rear_wheel_joint], `steering_joints_names` [right/left_steering_hinge_joint], `wheelbase 0.325`, `traction_track_width`/`steering_track_width 0.2`, `traction_wheels_radius 0.05`, `reference_timeout 1.0`, `use_stamped_vel true`, `enable_odom_tf false`, `odom_frame_id odom`, `base_frame_id base_link`. Its own comment says these keys were never confirmed on the Humble image. |
| `f1tenth_sim/drive_bridge.py` | Converts `/drive` (AckermannDriveStamped) to `/ackermann_steering_controller/reference` (TwistStamped: `linear.x = v`, `angular.z = v·tan(δ)/L`, L=0.325) and relays `/ackermann_steering_controller/odometry` to `/odom`. |
| `worlds/empty_room.sdf` | SDF 1.6, 1 ms step, RTF 1. Plugins: `ignition-gazebo-{physics,user-commands,scene-broadcaster,contact,imu,sensors}-system` (sensors use `ogre2`). A sun, a 100×100 ground plane, and a 10×10 m room of 1 m × 0.1 m walls. |
| models | None of its own. The robot comes from f1tenth_description and meshes are referenced by absolute `pkg_share` path. |
| package.xml | exec_depends `ros_ign_gazebo`, `ros_ign_bridge`, `ign_ros2_control`, `controller_manager`, `joint_state_broadcaster`, `ackermann_steering_controller`, `robot_localization`, `slam_toolbox`, `foxglove_bridge`, `robot_state_publisher`, `xacro`, `rclpy`, `ackermann_msgs`, `geometry_msgs`, `nav_msgs`, `f1tenth_description`. |
| tests | None (`test_depend` entries exist, but there is no `test/` directory). |

### 1.2 What it shares with f1tenth_description [code]
- `description.launch.py` uses `roboracer.urdf.xacro`, which includes `macros.xacro`, `core.xacro`, `sensors.xacro` and `ros2_control.xacro`.
  The launch file also runs `joint_state_publisher` and a static `base_link→laser` at (0.12, 0, 0.20, yaw 0).
  Both are gated `UnlessCondition(use_sim)`.
- `robot.urdf.xacro` (with `base.xacro`, `wheels.xacro`, `inertial_macros.xacro`) is a second, newer top-level model.
  **Nothing launches it.** It includes the same `sensors.xacro` and `ros2_control.xacro`.
- `ros2_control.xacro`: the `<ros2_control>` block is **unconditional** and uses hardware `ign_ros2_control/IgnitionSystem`.
  Rear wheels take a velocity command, steering hinges a position command, and front wheels are passive.
  Inside `xacro:if use_sim` sits the `<gazebo><plugin filename="ign_ros2_control-system" name="ign_ros2_control::IgnitionROS2ControlPlugin">`.
  Legacy `<gazebo reference>` friction tags (`mu1/mu2/kp/kd/fdir1`) follow.
- `sensors.xacro` (only when `enable_sensors`):
  - `laser`: `gpu_lidar` on topic `scan` with `<ignition_frame_id>laser`, 40 Hz, 1081 samples over ±2.356 rad, range 0.1–10 m. Mounted at **(−0.12, 0, 0.15) yaw π, rear-facing**.
  - `imu` on topic `sensors/imu/raw`, frame `imu`, 100 Hz, at base_link (identity).
  - `zed2_camera_link` at (0.12, 0, 0.15), with left lens and optical frames. RGB camera 1280×720 at 30 Hz and a depth camera 1280×720 R_FLOAT32 at 30 Hz.
- Geometry: wheelbase 0.325, track 0.2, wheel radius 0.05. base_link sits at the rear axle, at ground level (chassis +0.05, wheel axles on the chassis origin).
  Chassis mass 4.0 kg. Steering hinge limits are ±0.4 rad.

### 1.3 How the stack used the sim on Humble [code]
- `MPC_corr.py:1587` subscribes to `/model/virtual_robot/odometry` as a **fallback** odometry source.
  It is used only while the hardware odom topic (`get_odom_topic()`: `/odometry/filtered`, or `/odom` in raw_odom mode) is stale.
  The threshold is `odom_stale_timeout_sec` in `stack_params.yaml`. **Nothing in the current f1tenth_sim publishes that topic.**
  It belongs to an older sim that used the gz `OdometryPublisher` on a model called `virtual_robot`.
- `MPC_corr.py:1733` comment: MPC used to publish `/rear_wheels_controller/commands` and `/steering_controller/commands` (forward-command controllers of an older sim).
  It now publishes only `/drive`.
- MPC also subscribes to `/joint_states` (to read steering hinge positions), `/imu` (**no publisher anywhere in this repo**, a pre-existing gap and not a sim matter), and `/scan`.
- Bringup has **no sim flag**. `stack_bringup.launch.py` always includes `vesc.launch.py`, `localization`, `description` (use_sim default false), `camera`, `detection`, `ackermann_mux`, navigation, slam, costmap, diagnostics, foxglove, and `startup_sequence`.
  urg_node comes from `f1tenth_perception/lidar.launch.py` (`use_lidar` arg) through the supervisor's `components.yaml`.
- `f1tenth_params/stack_params.yaml`: `use_sim` (default false; the description says it emits the ign_ros2_control plugin), `enable_sensors` (false), `control_config` ('').
  `discovery_server_address` defaults to 127.0.0.1:11811.

### 1.4 Real hardware interface the sim must reproduce [code]

| Topic | Type | Publisher | frame_id / child | Rate | Units / content |
|---|---|---|---|---|---|
| `/odom` | nav_msgs/Odometry | `vesc_to_odom_node` (executable `vesc_to_odom_node_backup`) | `odom` / `base_link` | one message per `sensors/core`. The driver polls every 20 ms, so **≈50 Hz** (UNVERIFIED, no live hardware) | `twist.linear.x` = (ERPM−offset)/5499.27 m/s, zeroed below \|ERPM\|<500 and below \|v\|<0.05. `angular.z = v·tan(δ_cmd)/0.305`, where δ_cmd is recovered from the **commanded** servo (offset 0.4874, gain −1.0926). Pose is integrated by Euler. Covariance: pose[0]=0.2, [7]=0.2, [35]=0.03; twist[0]=0.0 (vx_variance). `publish_tf: false`. |
| `/sensors/imu/raw` | sensor_msgs/Imu | `vesc_driver_node` | **`""` (empty). The driver never sets header.frame_id** (`vesc_driver.cpp:248-305`) | one per ImuData reply, polled at 20 ms, so **≈50 Hz** (UNVERIFIED) | `angular_velocity.z = (gyr_z − (−0.01928)) · 0.0174533`. The raw value is in deg/s and gyro_scale_z = π/180 makes it **rad/s**. x/y are scaled ×1.0 (still deg/s, unused by the EKF). linear_acceleration is in VESC units, presumably m/s² with gravity (UNVERIFIED). Orientation comes from the VESC AHRS. Diagonal covariances come from `vesc.yaml` (gyro_z 1.75e-6). |
| `/sensors/imu` | vesc_msgs/VescImuStamped | `vesc_driver_node` | – | ≈50 Hz | raw, unscaled |
| `/sensors/core` | vesc_msgs/VescStateStamped | `vesc_driver_node` | – | ≈50 Hz | ERPM, voltage, … (battery check, calibration and diagnostics read it) |
| `/sensors/servo_position_command` | std_msgs/Float64 | `vesc_driver_node` | – | per command | echoes the commanded servo |
| TF `base_link→imu` | static | `vesc.launch.py` | identity | – | – |
| `/scan` | sensor_msgs/LaserScan | `urg_node` (`sensors.yaml`, Hokuyo at 192.168.0.10:10940) | `laser` | **40 Hz** (`jazzy_migration_plan.md` go/no-go, `phase3_control_report.md:456`) | `publish_intensity: true`. Requested angles ±3.14 are clipped by the device; for a UST-10LX expect ±2.356 rad, 1081 beams, 0.25° steps. Range limits are UNVERIFIED because no real `/scan` header is available on this PC. |
| TF `base_link→laser` | static | `description.launch.py` (real-only) | (0.12, 0, 0.20), yaw 0, front-facing | – | – |
| `/joint_states` | sensor_msgs/JointState | `joint_state_publisher` (real-only) | – | default 10 Hz | **static 0.0** for all 6 joints |
| Drive in | AckermannDriveStamped | MPC and others publish `/drive`. `ackermann_mux` (`mux.yaml`: safety_stop 200, teleop 100, calibration_drive 50, drive 10; timeout 0.2 s) outputs `/ackermann_drive`. | – | – | `speed` m/s, `steering_angle` rad, + = left (REP-103). `ackermann_to_vesc` applies the velocity-correction LUT, then ERPM = 5499.27·v, servo = gain·δ + 0.4874 (gain −1.2135). The servo is clamped to [0.15, 0.8318], so **δ ∈ [−0.2838, +0.2780] rad**. |

**Gaps: old sim vs. real hardware**

| # | Gap | Severity |
|---|---|---|
| G1 | The sim ran **robot_state_publisher on /tf and /tf_static** (base_link→chassis→wheels/hinges/laser/imu/zed). That collides with the Thor's RSP and breaks the TF rule. | high |
| G2 | The sim ran its own **EKF** (publishing `odom→base_link`) and **slam_toolbox**. Both are the stack's job. | high |
| G3 | Sim laser at (−0.12, 0, 0.15) yaw π (rear-facing, the pre-remount layout). Real laser at (0.12, 0, 0.20) yaw 0. Scans would disagree with the Thor's static TF. | high |
| G4 | The sim drive input was `/drive`, which **bypasses ackermann_mux** (safety_stop/teleop priority). The real actuator consumes `/ackermann_drive`. | high |
| G5 | Sim steering was limited only by the URDF ±0.4 rad. The real servo saturates at about [−0.284, +0.278] rad. | medium |
| G6 | `/odom` rate: the sim's controller update rate was 100 Hz, real is ≈50 Hz. Covariances: the controller's defaults vs. the real values 0.2/0.2/0.03/vx 0.0. | low |
| G7 | IMU 100 Hz vs. real ≈50 Hz. Sim frame `imu` vs. real `""`. The sim gyro is true rad/s with no bias, matching the post-correction real value. | low (frame: see D3) |
| G8 | The URDF wheelbase is 0.325; the real `vesc_to_odom` uses `wheelbase: .305`. Same-geometry rule applies: **keep 0.325 in sim**, flagged. | info |
| G9 | `/joint_states`: real publishes static zeros at 10 Hz; sim publishes true joint states. The sim value is better, and MPC reads hinge angles from it. | info |
| G10 | The sim does not publish `/sensors/core`, `/sensors/imu`, `/sensors/servo_position_command`. They are only needed by the battery, calibration and diagnostics nodes, which a sim mode should skip anyway. | info |
| G11 | `/model/virtual_robot/odometry` (MPC fallback) is not published by the old sim, and will **deliberately not** be published by the new one (see Plan §2.4). | info |
| G12 | The sim had the ZED mock: 2×1280×720 at 30 Hz rendered and bridged. Out of scope, and expensive on this GPU. | – |
| G13 | Doc/URDF inconsistency: `description.launch.py` says base_link is "centered between the axles, 0.07 m above ground", but the URDF puts base_link at the rear axle on the ground. The laser/camera TF numbers use the former convention. Not fixed here. | info |

---

## 2. Port plan (written before any edit)

### 2.1 Mechanical Fortress → Harmonic renames

| Class | Before | After | Source |
|---|---|---|---|
| ROS packages | `ros_ign_gazebo`, `ros_ign_bridge`, `ign_ros2_control` | `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control` | [doc] gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages; [doc] gazebosim.org/docs/harmonic/ros_installation |
| Sim launch | `ign_gazebo.launch.py`, `ign_args` | `gz_sim.launch.py`, `gz_args` | [doc] same migration page |
| Spawn | `ros_ign_gazebo create` | `ros_gz_sim create` (`-name`, `-string`) | [doc] same |
| Bridge type strings | `ignition.msgs.Clock/LaserScan/IMU` | `gz.msgs.Clock/LaserScan/IMU`, plus `gz.msgs.Odometry` for ground truth | [doc] github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md (fetch returned 503 today; schema keys `ros_topic_name/gz_topic_name/ros_type_name/gz_type_name/direction` will be re-verified against the installed package) |
| World system plugins | `ignition-gazebo-X-system` / `ignition::gazebo::systems::X` | `gz-sim-X-system` / `gz::sim::systems::X` | [doc] migration page ("gz-sim-physics-system", "gz-sim-sensors-system", "gz-sim-imu-system") and gz-sim `Migration.md` |
| SDF sensor frame tag | `<ignition_frame_id>` | `<gz_frame_id>` | [doc] migration page ("Add `<gz_frame_id>`") |
| ros2_control hardware | `ign_ros2_control/IgnitionSystem` | `gz_ros2_control/GazeboSimSystem` | [doc] control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html |
| ros2_control gz plugin | `ign_ros2_control-system` / `ign_ros2_control::IgnitionROS2ControlPlugin` | `gz_ros2_control-system` / `gz_ros2_control::GazeboSimROS2ControlPlugin` | [doc] same (also the working `msauber_description/urdf/gazebo_plugins_acker.xacro` on this PC) |
| Env var | `IGN_GAZEBO_RESOURCE_PATH` | `GZ_SIM_RESOURCE_PATH` only | [doc] gz-sim Migration.md |
| Controller params | `use_stamped_vel` | removed if Jazzy rejects it. In Jazzy the reference is always `~/reference` TwistStamped. `traction_*`/`steering_*` names are current, and `front/rear_wheel*` are deprecated. | [doc] control.ros.org/jazzy/…/steering_controllers_library/doc/userdoc.html, …/ackermann_steering_controller/doc/userdoc.html. **Verify with `ros2 param dump` after install.** |

### 2.2 Interface decision: sim = drop-in for the drivers

The sim publishes exactly what vesc_driver, vesc_to_odom and urg_node publish. It consumes exactly what
`ackermann_to_vesc` consumes. In sim mode the Thor skips `vesc.launch.py`, `urg_node` and the ZED, and keeps
everything else, **including ackermann_mux**.

| Real | Sim source (PC) | Notes |
|---|---|---|
| `/ackermann_drive` in | `drive_bridge` subscribes to **`/ackermann_drive`** (was `/drive`) | The mux stays in the loop (fixes G4). In standalone mode, publish to `/ackermann_drive` directly. |
| `/odom` | ackermann_steering_controller `~/odometry`, relayed to `/odom` by `drive_bridge` | `odom_frame_id: odom`, `base_frame_id: base_link`, `enable_odom_tf: false`. `pose_covariance_diagonal [0.2,0.2,0,0,0,0.03]`, `twist_covariance_diagonal [0,…]` to match vesc.yaml. `controller_manager.update_rate: 50` to match the real ≈50 Hz. |
| `/sensors/imu/raw` | gz IMU sensor, bridged to `/sim/imu_raw`, then `drive_bridge` republishes it | 50 Hz, rad/s, **frame_id `""`, same as the real driver** (D3 option c). Covariance diagonals are the vesc.yaml values. |
| `/scan` | gz `gpu_lidar`, bridged | frame `laser`, 40 Hz, 1081 beams over ±2.356 rad. Mount moved to the real pose (0.12, 0, 0.20, yaw 0) to fix G3. Range 0.1–10 m kept (UNVERIFIED vs. real). |
| `/joint_states` | `joint_state_broadcaster` | 6 joints, true values. The Thor must not run `joint_state_publisher` in sim mode. |
| `/clock` | bridged | All PC nodes use `use_sim_time`. |
| `/sim/ground_truth` | gz `OdometryPublisher` system on the model, bridged as `nav_msgs/Odometry` | Frames `sim_world` / `sim_base_link`, so it can never be mistaken for a real frame. **Never on /tf.** |

**Ackermann mapping.** Keep the existing (minimal) path: `AckermannDriveStamped(v, δ)` becomes TwistStamped
`(v, v·tan(δ)/L)`, L = 0.325. The controller inverts it as `δ = atan(ω·L/v)`, so the round trip is exact for v ≠ 0.
Two additions in `drive_bridge`:
1. Clamp δ to the real servo envelope `[−0.2838, +0.2780]` rad. These are ROS params derived from
   `steering_calibration.yaml` (fixes G5).
2. Keep the controller's `reference_timeout` and add the mux's 0.2 s semantics. If `/ackermann_drive` goes silent,
   the controller zeroes the reference after `reference_timeout` (set to 0.2 s to match the mux timeout).

Known limitation: the Twist interface cannot express **steering at v = 0** (ω = 0 there), so the sim can't
pre-steer while stopped. The real servo can. An alternative (forward-command controllers plus our own odom
integration) is noted under decisions, but it is not proposed because it is a redesign.

Sign convention: + steering_angle = left, + speed = forward.
- Steering hinge: rotation (0, π/2, 0), axis −x, so + hinge angle is +yaw (left).
- Wheel joints: rotation (π/2, 0, 0), axis −z, which is +y in the chassis frame, so + velocity rolls toward +x.

This will be verified empirically in Step 4.

### 2.3 TF rule
The sim publishes **nothing on `/tf` or `/tf_static`**.
- The controller's `enable_odom_tf: false`.
- No EKF and no slam_toolbox in the sim launch. They move to the Thor (G2). `foxglove_bridge` becomes optional (`foxglove:=false` by default).
- gz_ros2_control needs the sim URDF (with the `<gazebo>` plugin and sensors). *(Superseded by §3: in 1.2.20 the
  controller_manager reads it from the `robot_description` topic, remapped to `/sim/robot_description`; the
  `<robot_param_node>` plan below no longer applies.)* Original assumption: in Jazzy it fetches `robot_description`
  from the node `robot_state_publisher` by default. The plan is to run a **private** RSP named
  `sim_robot_state_publisher` with `/tf → /sim/tf`, `/tf_static → /sim/tf_static` and
  `/robot_description → /sim/robot_description` remapped, then point the plugin at it with
  `<robot_param_node>`. If Jazzy's plugin cannot be pointed at a renamed node, the fallback is the same remaps on
  an RSP that keeps its default name. That is still TF-silent, but it collides by name with the Thor's RSP,
  so the renamed node is preferred.
- Ground truth goes only to `/sim/ground_truth` (an Odometry topic, not TF).
- Step 4 check: `ros2 topic info /tf -v` and `/tf_static -v` show **zero publishers** from the PC.

### 2.4 Things deliberately not done
- `/model/virtual_robot/odometry` is not published. If it were, MPC would silently fall back to perfect ground truth whenever `/odometry/filtered` is stale, which hides localization faults in sim. **D4** asks whether the MPC fallback should be removed.
- No VESC telemetry emulation (`/sensors/core` etc.).
- ZED/camera: out of scope. The gz `<sensor>` blocks for zed2 are disabled by a new xacro arg `enable_camera_mock` (default false), and the camera bridges are dropped. What camera simulation would take: re-enable the arg; bridge `gz.msgs.Image`/`CameraInfo` (or use `ros_gz_image` image_bridge for compression); remap to the ZED wrapper's real topic names and frames (`zed2_left_camera_optical_frame`); add a depth-registered encoding of 32FC1. Budget is ≈110 MB/s raw at 2×1280×720 at 30 Hz, too much for the LAN without compression, so resolution would need to drop. The detector would also need sim-appropriate objects in the world.
- Same world, same masses, inertias, friction and geometry (wheelbase 0.325).

### 2.5 File-by-file change list (one commit each)
1. `f1tenth_description/urdf/ros2_control.xacro`: hardware plugin and gz plugin renamed to gz_ros2_control. Add `<robot_param_node>`, and a `<ros>` remapping only if needed. Header comment updated.
2. `f1tenth_description/urdf/sensors.xacro`: `ignition_frame_id` → `gz_frame_id`. Laser mount moved to the real pose (G3). IMU 100 → 50 Hz. Camera sensors gated by `enable_camera_mock`. Header updated.
3. `f1tenth_description/urdf/roboracer.urdf.xacro`: declare `enable_camera_mock` (default false). Fix comments.
4. `f1tenth_params/config/stack_params.yaml`: only the `use_sim` description text ("ign_ros2_control" → "gz_ros2_control"). No value changes.
5. `f1tenth_sim/worlds/empty_room.sdf`: plugin renames.
6. `f1tenth_sim/config/ros_gz_bridge.yaml`: `gz.msgs.*`, drop ZED entries, add `/sim/ground_truth`.
7. `f1tenth_sim/config/controllers.yaml`: update_rate 50, covariances, reference_timeout 0.2, Jazzy key check.
8. `f1tenth_sim/f1tenth_sim/drive_bridge.py`: input `/ackermann_drive`, steering clamp params.
9. `f1tenth_sim/launch/sim_bringup.launch.py`: ros_gz_sim/ros_gz_bridge, private RSP, no EKF/SLAM, optional foxglove and gui args, OdometryPublisher for ground truth (added to the URDF `<gazebo>` block or the world; in the URDF so it follows the model).
10. `f1tenth_sim/package.xml` / `setup.py`: deps updated (drop robot_localization and slam_toolbox, add ros_gz_*/gz_ros2_control, sensor_msgs).
11. Last: remove `f1tenth_sim/COLCON_IGNORE`.

Changes outside f1tenth_sim, and why:
- **f1tenth_description**: it owns the URDF that carries the Gazebo plugin and sensor tags, so this is unavoidable. Real-hardware impact: none. `<gazebo>` blocks are ignored by RSP. The `<ros2_control>` hardware string changes, but nothing on the car loads ros2_control. The laser change affects only `enable_sensors:=true`, which the real hardware never sets.
- **f1tenth_params**: one description string only.
- **Nothing else.** The Thor's `sim:=true` bringup mode is documented in §5, not implemented (the Thor side comes later).

---

## 3. Port (Step 3, done 2026-10-02)

Branch `jazzy-sim`, 10 commits on top of `origin/jazzy` (`fc1ae6f`):

| Commit | What |
|---|---|
| `63f46d4` | `scripts/sim_host_setup.sh`: the D0 install as a script (`--check` is read-only). |
| `4f0a3d3` | `ros2_control.xacro`: gz_ros2_control, CM reads `/sim/robot_description`, ground-truth OdometryPublisher. |
| `cc3addd` | `sensors.xacro` + `roboracer.urdf.xacro`: `gz_frame_id`, laser at the real pose (G3), IMU 50 Hz, `enable_camera_mock` (default false). |
| `90cbfb1` | `stack_params.yaml`: description/comment text only. |
| `ace37af` | `empty_room.sdf`: `gz-sim-*` plugin names. |
| `66d9ff3` | `ros_gz_bridge.yaml`: `gz.msgs.*`, IMU → `/sim/imu_raw`, `/sim/ground_truth`, ZED dropped. |
| `bd49bfc` | `controllers.yaml`: update_rate 50, vesc.yaml covariances, reference_timeout 0.2, `use_stamped_vel` dropped. |
| `6739899` | `drive_bridge.py` (+ ROS-free `kinematics.py`, tests): `/ackermann_drive` (D1), steering clamp (D2), IMU relay with `frame_id ""` (D3). |
| `8b8a83a` | `sim_bringup.launch.py`: ros_gz, private TF-silent RSP, no EKF/SLAM, chained spawners; `description.launch.py` comments. |
| `0ef3500` | `package.xml` / `setup.py` deps. |

Not yet done: removing `f1tenth_sim/COLCON_IGNORE` (plan item 11). It waits for Step 4, because without it the
package is built (and its tests run) on the Thor too.

**Checked against the real Jazzy binaries before writing**, without installing them (`apt-get download` + `dpkg-deb -x`
into the scratchpad, plus the gz_ros2_control `1.2.20` source tag):

- **Correction to §2.3.** gz_ros2_control 1.2.20 has **no** `robot_param` / `robot_param_node`: it does not read the URDF
  from robot_state_publisher's parameter any more. It constructs the controller_manager and then waits ("Wait for CM to
  receive robot description from the topic", `gz_ros2_control_plugin.cpp`). The controller_manager (4.48.0) subscribes
  to the relative topic `robot_description`. The plugin passes `<ros><remapping>` entries to the CM node as `-r`
  arguments. So the design is: `<remapping>robot_description:=/sim/robot_description</remapping>` in the URDF plus a
  private `sim_robot_state_publisher` that publishes there, with TF remapped to `/sim/tf*`. Without the remap the sim's
  CM would read the Thor's `/robot_description` (a URDF without the gz plugin or sensors). The spawn also reads
  `/sim/robot_description` (`create -topic`), so xacro runs once.
- ros2_controllers 4.42.1 (`steering_controllers_library_parameters.hpp`): `traction_joints_names`,
  `steering_joints_names`, `traction_wheels_radius`, `traction_track_width`, `steering_track_width`,
  `reference_timeout`, `pose/twist_covariance_diagonal`, `enable_odom_tf` exist; **`use_stamped_vel` does not**.
- ros_gz_bridge 1.0.24: YAML keys `ros_topic_name/gz_topic_name/ros_type_name/gz_type_name/direction` (plus `lazy`,
  queues) and the types `gz.msgs.Clock/LaserScan/IMU/Odometry` are in `libros_gz_bridge.so`.
- ros_gz_sim 1.0.24: `gz_sim.launch.py` takes `gz_args`, `on_exit_shutdown`; `create` has `-topic`.

**Tests added** (`src/f1tenth_sim/test/`, 36 pass in a scratch venv; `colcon test` after install):
`test_kinematics.py` (round trip through the controller's inverse, clamp to +0.2780/−0.2838, REP-103 signs, the
standstill limitation) and `test_sim_matches_car_config.py`. The second pins every value the sim copies from the car
to its source file: servo envelope ↔ `steering_calibration.yaml`, IMU covariances and odom covariances ↔ `vesc.yaml`,
reference_timeout ↔ `mux.yaml`, wheelbase, `/ackermann_drive`, `frame_id ""`, `enable_odom_tf: false`, and no
bridge entry on `/tf*`. Per CLAUDE.md, those are couplings: change the car value and this test fails until the sim
follows.

Design details worth knowing:
- IMU orientation: the gz IMU fills orientation (world-relative). The car's comes from the VESC AHRS. The EKF fuses
  only vyaw (`imu0_config`), so neither matters today. The sim's x/y gyro is rad/s, while the car's is deg/s (gyro_scale
  1.0), and sim accel is m/s² against the car's g. Not emulated; also unused by the EKF.
- The real ERPM limit (`speed_max` 23250 → 4.23 m/s) and the low-speed velocity-correction LUT are **not** emulated.
  The sim follows the commanded speed. Follow-up if MPC runs above ~4 m/s in sim.

## 4. Verification (Step 4): pending the install

Run in order on the sim PC. Items marked ★ settle an UNVERIFIED point above.

1. `scripts/sim_host_setup.sh` (sudo password). ★ It prints `which gz` and `gz sim --versions` under the sourced
   env (OSRF gz-harmonic vs. the `*-vendor` build, §0.5 note).
2. `rm src/f1tenth_sim/COLCON_IGNORE` (working tree only), then
   `colcon build --packages-up-to f1tenth_sim && colcon test --packages-select f1tenth_sim && colcon test-result --verbose`.
   Expect 36 + flake8.
3. URDF → SDF: `xacro …/roboracer.urdf.xacro use_sim:=true enable_sensors:=true pkg_share:=… control_config:=… > /tmp/r.urdf && gz sdf -p /tmp/r.urdf`.
   ★ Both `<plugin>`s present. hokuyo + imu_sensor survive fixed-joint lumping onto base_link with the right pose.
   No zed2 sensors.
4. `ros2 launch f1tenth_sim sim_bringup.launch.py` (then `gui:=true` once). Expect: CM logs the robot description
   received, both controllers `active` (`ros2 control list_controllers`).
5. ★ TF rule: `ros2 topic info -v /tf` and `/tf_static` show **0 publishers**. `/sim/tf_static` has one.
6. Topics and rates: `/clock`; `/scan` 40 Hz, frame `laser`, 1081 beams; `/sensors/imu/raw` ~50 Hz, `frame_id: ''`,
   covariance[8] = 1.746756e-06; `/odom` ~50 Hz, `odom`/`base_link`, covariance[0,7,35] = 0.2/0.2/0.03;
   `/joint_states` 6 joints. ★ gz sensor topic names (`gz topic -l`: `/scan`, `/sensors/imu/raw`).
7. ★ `ros2 param dump /ackermann_steering_controller`: every key from controllers.yaml applied, none rejected.
8. ★ Laser pose: at spawn (0,0,0) facing the +X wall at 5 m (inner face 4.95), `ranges[540]` (angle 0) ≈ 4.95 − 0.12 =
   **4.83 m**. A rear-facing laser would read ≈ 5.07.
9. ★ Signs and clamp: `ros2 topic pub -r 20 /ackermann_drive ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.5, steering_angle: 0.4}}"`.
   `/sim/ground_truth` yaw increases (left turn). The hinge positions in `/joint_states` correspond to a bicycle angle
   of 0.2780, not 0.4. `/odom` twist ≈ (0.5, ω = 0.5·tan(0.278)/0.325 ≈ 0.44). Stop publishing: the car stops within
   ~0.2 s sim time (reference_timeout).
10. `/odom` vs `/sim/ground_truth` over a 20 s circle: drift recorded, not a pass/fail.
11. Load: RTF from `gz stats` with gui:=false and gui:=true on the Iris Xe. CPU and GPU in `top` / `intel_gpu_top`.
12. If all pass: commit the `COLCON_IGNORE` removal.

## 5. Thor side: running the stack against the sim (Step 5, documented, not implemented)

The sim PC provides `/clock /scan /odom /sensors/imu/raw /joint_states` and consumes `/ackermann_drive`. On the Thor,
a `sim:=true` mode of `stack_bringup.launch.py` needs:

| Component | Sim mode | Why |
|---|---|---|
| `vesc.launch.py` (vesc_driver, ackermann_to_vesc, vesc_to_odom, static `base_link→imu`) | **skip** | The sim publishes `/odom` and `/sensors/imu/raw`. `ackermann_to_vesc` would fail without a VESC. The `base_link→imu` static TF is not needed: frame `""` is resolved to base_link by the EKF (B1-a). |
| urg_node (`lidar.launch.py` via supervisor `components.yaml`, `use_lidar`) | **skip** | Sim `/scan`. |
| ZED camera + detection | **skip** | No camera mock (§2.4). |
| Battery / calibration / VESC diagnostics (read `/sensors/core`) | **skip or tolerate** | Not emulated (G10). The startup_sequence "VESC is responding" check must not block. |
| `description.launch.py` | **RSP and static `base_link→laser`: keep. `joint_state_publisher`: skip.** | The sim publishes the real `/joint_states`. **`use_sim:=true` is the wrong switch**: it gates `joint_state_publisher` *and* `static_baselink_to_laser` together, and the laser TF must stay. It needs a separate condition (e.g. a `sim` arg gating only `joint_state_publisher`), and `enable_sensors` stays false. |
| ackermann_mux, EKF, slam_toolbox, costmap, navigation, MPC, behavior | **keep** | That is what is under test. MPC must not get `/model/virtual_robot/odometry` (D4); the sim does not publish it. |
| `use_sim_time` | **true for every node** | Today it is plumbed through only 4 launch files. Cheapest whole-tree switch: `launch_ros.actions.SetParameter(name='use_sim_time', value=True)` at the top of `stack_bringup.launch.py` under the `sim` condition. **But** components started by `component_supervisor_node` are separate processes and do not inherit it. They need the parameter passed by the supervisor. UNVERIFIED which nodes that covers. |
| DDS discovery | **must change** | The Thor's Fast-DDS discovery server listens on `discovery_server_address: 127.0.0.1` (`stack_params.yaml`), which the PC cannot reach. Sim mode needs it on the Thor's LAN address, and the PC exports `ROS_DISCOVERY_SERVER=<thor-ip>:11811` (plus `ROS_SUPER_CLIENT=TRUE` for `ros2` CLI introspection). Same `RMW_IMPLEMENTATION` (fastrtps) on both. The PC's wired NIC is currently down (§0.5); a cable is needed for 40 Hz scans + clock without Wi-Fi jitter. |

Order of work for the Thor phase: discovery over the LAN first (a `ros2 topic hz /clock` from the Thor), then the
`sim` arg with the table above, then a closed loop with MPC in empty_room.

---

## D. Decisions (recorded 2026-10-02)

- **Machine**: stay on this PC (TPad) for now. linus (the earlier x86 Jazzy PC) becomes a second sim host later;
  `scripts/sim_host_setup.sh` is the reproducible host setup record. Disk precondition: ≥ 25 GB free (the user is freeing
  space).
- **D0 — Install**: native apt (not Docker), once disk is OK.
- **D1 — Drive input**: the sim consumes `/ackermann_drive` (behind ackermann_mux).
- **D2 — Steering clamp**: clamp to the real servo envelope. Derivation from the real config, `src/f1tenth_hardware/f1tenth_hardware/config/steering_calibration.yaml`: `servo_min: 0.15`, `servo_max: 0.8318`, `steering_angle_to_servo_offset: 0.4874`, `steering_angle_to_servo_gain_left/_right: -1.2135`. `vesc_ackermann/src/ackermann_to_vesc.cpp:140-142` computes servo = gain·δ + offset, and `vesc_driver/src/vesc_driver.cpp:63,389` clamps it with `servo_limit_("servo", …)`, which reads `servo_min`/`servo_max`. So δ_max(left) = (0.15 − 0.4874)/−1.2135 = **+0.2780 rad** and δ_min(right) = (0.8318 − 0.4874)/−1.2135 = **−0.2838 rad**. `drive_bridge` takes these as parameters with exactly these defaults, and the comment cites the file.
- **D3 — IMU frame: option (c), decided 2026-10-02.** The sim publishes `/sensors/imu/raw` with **the same
  `frame_id` as the real VESC driver, the empty string**. The Thor's EKF then takes the identical code path as on the
  car (empty frame → treated as `base_link`, identity; see B1-a), and no TF change is needed on either side.
  Implementation: Gazebo cannot be relied on to emit an empty frame (gz-sensors substitutes a default frame when none
  is given), so the gz IMU is bridged to a sim-internal topic `/sim/imu_raw`. `drive_bridge` republishes it on
  `/sensors/imu/raw` with `frame_id = ""` and the real covariance diagonals from `vesc.yaml` (parameters, defaults
  copied from that file). Background on why `imu` would have needed a TF change: the URDF `imu` link
  (`sensors.xacro:77-89`) exists only with `enable_sensors:=true`, which the stack's RSP never sets; on the car
  base_link→imu comes from `vesc.launch.py`'s static publisher, which a sim mode skips. Setting a real IMU frame on
  car and sim is backlog item **B1**, with the former options (a)/(b) as its starting point.
- **D4 — `/model/virtual_robot/odometry`**: not published by the sim.
- **D5 — Wheelbase**: sim keeps 0.325. P2 says the real value must be measured.


---

## B. Post-migration backlog

**B1. Set a proper IMU frame on both the car and the sim (low priority).** Today both publish `/sensors/imu/raw` with
`frame_id: ""` (D3 option c). Starting options:
- **(a)** Make the URDF `imu` link and joint unconditional in `sensors.xacro`, so the stack's RSP owns base_link→imu.
  Delete `static_baselink_to_imu` from `vesc.launch.py` in the same commit. Set `frame_id = "imu"` in `vesc_driver.cpp`
  (a new `imu_frame` parameter) and in the sim's IMU relay (`drive_bridge` parameter `imu_frame_id`).
- **(b)** Keep the static publisher on the car, and give the Thor's sim mode its own static base_link→imu. Then switch
  both publishers to `imu`.
- Whichever option is chosen, the frame must exist on TF **before** any publisher stamps it, or the EKF drops the IMU
  (see B1-a's last point).

**B1-a. Finding: empty `/sensors/imu/raw` frame_id on the car is fused correctly (source check, low priority).**
- *Code fact:* `vesc_driver.cpp` (submodule `vesc` @ `2848a9f`, ImuData branch, lines 247–304) never assigns
  `std_imu_msg.header.frame_id`, so the published frame_id is `""`.
- *Read-only source check:* robot_localization does **not** drop such a message; it treats it as being in
  `base_link_frame`. The behaviour is identical in Humble and Jazzy:
  - `imuCallback` → `twistCallback(twist_ptr, …, base_link_frame_id_)`.
    Humble `humble-devel@8696ee5` (3.5.4) `src/ros_filter.cpp:578`; Jazzy `jazzy-devel@3efa714` (3.8.3, the version
    of the current `ros-jazzy-robot-localization` binary) `src/ros_filter.cpp:640`.
  - `prepareTwist`: `std::string msg_frame = (msg->header.frame_id == "" ? target_frame : msg->header.frame_id);`
    Humble `ros_filter.cpp:3184`, Jazzy `ros_filter.cpp:3367`. The empty frame becomes `target_frame`, which is `base_link`.
  - `lookupTransformSafe(base_link, base_link)`: tf2 normally resolves this. If it throws (no TF yet), the fallback
    `if (target_frame == source_frame) { target_frame_trans.setIdentity(); retVal = true; }` applies.
    Humble `src/ros_filter_utilities.cpp:174–181`, Jazzy `:172–179`.
  - The IMU subscription is a plain `create_subscription<sensor_msgs::msg::Imu>` (Jazzy `ros_filter.cpp:1745`),
    with no tf2 MessageFilter that could discard frame-less messages.
  - The real `angular_velocity_covariance[0]` is `gyro_variance_x` = 0.0305, not −1, so the "ignore angular
    velocity" branch (`ros_filter.cpp:619`) does not fire either.
- *Conclusion (from code, UNVERIFIED live):* the gyro yaw rate **is** fused, assumed to be in `base_link`, with an
  identity rotation. Because the real IMU TF is identity anyway, the result is numerically the same as if the frame
  were `imu`. **The hypothesis "the EKF dropped the IMU" is not supported by the source.**
- *Still to do on the car:* `ros2 topic echo --once /sensors/imu/raw --field header` to confirm `frame_id: ''`, and
  check the EKF's diagnostics (`print_diagnostics: true`) for `imu0` warnings.
- *The inverse risk matters more:* if the driver is fixed to stamp `frame_id: imu`, the EKF **will** need
  `base_link→imu` on TF. A frame that isn't `""` and isn't `base_link` takes the lookup path, and if the lookup
  fails there is no identity fallback, so the measurement is dropped. The sim stamps `""` like the car (D3 option c),
  so this applies to both only once B1 is done.
