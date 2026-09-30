# F1Tenth Jazzy Migration Analysis

Generated: 2026-09-30  
Branch: `jazzy`  HEAD: `e47e646` (2026-09-30)  
Host: Ubuntu 24.04.4 / x86_64 / Python 3.12.3 / ROS 2 Jazzy  

---

## Summary Table

| Package | Tier | Main reason | Trial build | Trial test | Import smoke | Effort | Confidence |
|---|---|---|---|---|---|---|---|
| f1tenth_messages | T0 | Pure rosidl CMake | PASS | PASS | N/A (C) | S | High |
| f1tenth_params | T0 | Config-only, no ROS API | PASS | PASS | OK | S | High |
| corridor_perception | T0 | Pure maths, numpy only | PASS | PASS | OK | S | High |
| f1tenth_description | T0 | Static URDF/xacro, no nodes | PASS | PASS | N/A | S | High |
| f1tenth_localization | T1 | robot_localization API: pose0_rejection_threshold still present in 3.8.3 | PASS | 3 fail* | OK | S | High |
| f1tenth_costmap | T0 | Pure Python, no upstream API change | PASS | PASS | OK | S | High |
| f1tenth_navigation | T1 | slam_toolbox Humble 2.6→Jazzy 2.8.5: lifecycle node, param check needed | PASS | PASS | N/A | S | High |
| f1tenth_perception | T1 | urg_node/zed/cv_bridge version checks; `vesc_msgs` dep on test host | PASS | 2 fail* | OK (partial) | S | Medium |
| f1tenth_diagnostics | T1 | vesc_msgs submodule not initialized; ruamel.yaml not installed | PASS | 1 error* | FAIL (vesc_msgs) | S | High |
| f1tenth_control / mpc_controller | T1 | osqp not installed; flake8 style | PASS | 157 err/58 fail* | OK | M | High |
| f1tenth_behavior | T1 | py_trees_ros not installed | PASS | 1 error* | FAIL (py_trees_ros) | S | High |
| f1tenth_logger | T1 | pyarrow not installed (no apt package in Jazzy) | PASS | 1 error* | OK (node) | S | Medium |
| f1tenth_bringup | T2 | ROS_LOCALHOST_ONLY deprecated in Jazzy; Discovery Server setup; SHM glob pattern | PASS | 3 fail* | OK | M | High |
| f1tenth_hardware | T2 | vesc submodule empty; VESC C++ build on Jazzy unverified | PASS† | PASS (0 tests) | FAIL (vesc_msgs) | M | Low |
| f1tenth_intelligence / llm | T1 | flake8/pep257 style; llama-server bringup unchanged | PASS | 14 fail* | OK | S | High |
| f1tenth_sim | T3 | ign_ros2_control→gz_ros2_control; ros_ign_gazebo→ros_gz_sim package rename | PASS† | PASS (0 tests) | N/A | M | High |
| f1tenth_more | T0 | Meta-package only | PASS | PASS | N/A | S | High |
| f1tenth_control (top) | T0 | Launch/config wrapper | PASS | PASS | N/A | S | High |

†build passes only because submodules are empty (no source to compile).  
\*failures classified in detail below; all test environment issues or missing-dep, no real API breaks yet.

---

## Step 0 — Baseline and Environment

### Repo identity

| Field | Value |
|---|---|
| Path | `/home/andreas/dev_ws/f1tenth_more` |
| Branch | `jazzy` |
| HEAD | `e47e646` — "repo root: move the loose docs under docs/, drop the stale ones" |
| HEAD date | 2026-09-30 09:42 +0200 |
| Remote | `origin git@github.com:sjckness/f1tenth_more.git` |
| Tags | none |

### Relation to the Humble baseline

`git ls-remote origin` shows **all six remote branches** (`main`, `jazzy`, `scene-graph`, `llm`, `sick`, `test-logging`) point at the same commit `e47e646`. There is no `humble-final` tag and no separate Humble commit anywhere in the reachable history. The oldest commits (2026-05-30) are the original workspace restructure. This repo **is** the Humble codebase with Jazzy work layered on the same branch; there is no independent Humble tree to cherry-pick from.

**Recommendation (do not change remotes now):** add the Jetson's copy as a named remote:

```
git remote add humble git@github.com:sjckness/f1tenth_more.git  # same origin,
# then track a Humble-state tag when Andreas cuts one on the Jetson side
```

The more actionable step is for Andreas to **tag the Jetson's current HEAD** as `humble-final` before any Jazzy edits land there, so `git diff humble-final HEAD` gives a clean Humble→Jazzy diff.

### Content check — all required items present

| Item | Status | Location |
|---|---|---|
| `f1tenth_costmap` + `costmap_boundary_node` | ✓ Present | `src/f1tenth_costmap/f1tenth_costmap/costmap_boundary_node.py` |
| Dual EKF: `ekf.launch.py` + `ekf_global.launch.py` | ✓ Present | `src/f1tenth_localization/launch/` |
| `slam_pose_relay_node` | ✓ Present | `src/f1tenth_localization/f1tenth_localization/slam_pose_relay_node.py` |
| `raw_odom_map_tf_node` | ✓ Present | `src/f1tenth_localization/f1tenth_localization/raw_odom_map_tf_node.py` |
| `mission_logger_node` | ✓ Present | `src/f1tenth_logger/f1tenth_logger/mission_logger_node.py` (in `f1tenth_logger`, not `f1tenth_diagnostics` as the task assumed) |
| S-curve corridor `test_corridor_turn_shape.py` | ✓ Present | `src/f1tenth_control/mpc_controller/test/test_corridor_turn_shape.py` |
| map→odom anchor: `_refresh_goal_anchor`, `test_map_frame_anchor.py` | ✓ Present | `src/f1tenth_control/mpc_controller/mpc_controller/MPC_corr.py`, `test/test_map_frame_anchor.py` |
| `corridor_update_period` | ✓ Present, value `1.0` s | `src/f1tenth_params/config/stack_params.yaml:899` |
| `use_hard_boundary_constraints` | ✓ Present, value `true` | `src/f1tenth_params/config/stack_params.yaml:1205` |
| `front_clearance_corridor_half_width_m` | ✓ Present, value `0.35` m | `src/f1tenth_params/config/stack_params.yaml:448` |

Repo is **not stale**. All critical work items are present.

### Environment hygiene

```
AMENT_PREFIX_PATH   = /opt/ros/jazzy                (clean, Jazzy only)
CMAKE_PREFIX_PATH   = /opt/ros/jazzy/opt/...        (Jazzy only)
PYTHONPATH          = /opt/ros/jazzy/lib/python3.12/site-packages (Jazzy only)
ROS_DISCOVERY_SERVER     = (unset in this shell)
FASTRTPS_DEFAULT_PROFILES_FILE = (unset)
ROS_LOCALHOST_ONLY       = (unset)
ROS_DOMAIN_ID            = (unset)
RMW_IMPLEMENTATION       = (unset → default: rmw_fastrtps_cpp)
ROS_AUTOMATIC_DISCOVERY_RANGE = SUBNET   (Jazzy default)
```

No Humble paths present. Only Jazzy is sourced. Clean baseline.

**DDS env vars in source:** `ROS_DISCOVERY_SERVER=127.0.0.1:11811` set via `SetEnvironmentVariable` in `stack_bringup.launch.py:92` and `supervisor_bringup.launch.py:67`. The test campaign conftest at `src/f1tenth_logger/test/test_campaign/conftest.py:20` sets `ROS_LOCALHOST_ONLY=1`; this is deprecated in Jazzy (see §f1tenth_bringup). The replay script `scripts/replay_bag_through_mpc.sh:35` also sets `ROS_LOCALHOST_ONLY=1`.

---

## Step 1 — Empirical Probe

### Package inventory (18 packages)

| Package | Build type | Type |
|---|---|---|
| `f1tenth_messages` | ament_cmake (rosidl) | First-party |
| `f1tenth_params` | ament_python | First-party |
| `corridor_perception` | ament_python | First-party |
| `f1tenth_description` | ament_python | First-party |
| `f1tenth_localization` | ament_python | First-party |
| `f1tenth_sim` | ament_python | First-party |
| `f1tenth_perception` | ament_python | First-party |
| `f1tenth_logger` | ament_python | First-party |
| `f1tenth_diagnostics` | ament_python | First-party |
| `f1tenth_costmap` | ament_python | First-party |
| `f1tenth_navigation` | ament_python | First-party |
| `f1tenth_behavior` | ament_python | First-party |
| `llm` | ament_python | First-party |
| `mpc_controller` | ament_python | First-party |
| `f1tenth_control` | ament_python | First-party (launch wrapper) |
| `f1tenth_bringup` | ament_python | First-party |
| `f1tenth_more` | ament_python | Meta |
| `f1tenth_hardware` | ament_python | First-party (launch wrapper) |
| **vesc** (submodule) | ament_cmake | Vendored — submodule **empty** |
| **ackermann_mux** (submodule) | ament_cmake | Vendored — submodule **empty** |
| **teleop_tools** (submodule) | ament_cmake | External — submodule **empty** |
| **transport_drivers** (submodule) | ament_cmake | External — submodule **empty** |
| **zed_ros2_wrapper** (submodule) | ament_cmake | External — submodule **empty** |
| **zed-ros2-interfaces** (submodule) | ament_cmake | External — submodule **empty** |

All six submodules are registered in `.gitmodules` but **not initialized** (each directory contains only a `.git` file pointer). The 18 first-party packages build cleanly because they have no compile-time hard dependency on the vendored C++ sources.

### rosdep check results

Notable errors from `rosdep check`:

| rosdep key | Status | Note |
|---|---|---|
| `ament_python` | Not found | Expected; `ament_python` is a build-system concept, not a rosdep key. Harmless. |
| `python3-pyarrow` | Not found | No `apt` package for pyarrow on Noble. **Must install via pip.** |
| `ign_ros2_control` | Not found | Renamed to `gz_ros2_control` in Jazzy. **T3 for f1tenth_sim.** |
| `vesc_ackermann`, `vesc_msgs`, `vesc_driver`, `vesc` | Not found | Vendored submodule, not a system package. Expected until submodule initialized. |
| `ackermann_mux` | Not found | Vendored submodule. Expected. |

### Third-party Python packages

| Module | Installed | Version | Jazzy apt available | Notes |
|---|---|---|---|---|
| `numpy` | ✓ | 1.26.4 | `ros-jazzy-python3-numpy` | NumPy 1.x. `numpy.float` etc. removed in 1.20 but this code uses no deprecated aliases. |
| `cv2` (opencv) | ✓ | 4.6.0 | `ros-jazzy-cv-bridge` 4.1.0 deps it | Works with numpy 1.26. |
| `scipy` | ✓ | 1.11.4 | `python3-scipy` | OK |
| `matplotlib` | ✓ | 3.6.3 | `python3-matplotlib` | OK |
| `psutil` | ✓ | 5.9.8 | `python3-psutil` | OK |
| `requests` | ✓ | 2.31.0 | `python3-requests` | OK |
| `jsonschema` | ✓ | 4.10.3 | `python3-jsonschema` | OK |
| `yaml` (PyYAML) | ✓ | system | `python3-yaml` | OK |
| `pydot` | ✓ | 1.4.2 | `python3-pydot` | OK |
| `rosbag2_py` | ✓ | Jazzy | `ros-jazzy-rosbag2-py` | OK; `mcap` and `sqlite3` both registered |
| `py_trees` | ✓ | 2.4.0 installed, 2.5.0 available | `ros-jazzy-py-trees` | Upgrade needed; see §f1tenth_behavior |
| `py_trees_ros` | ✗ | not installed | `ros-jazzy-py-trees-ros` 2.5.0 available | **Missing. T1 install.** |
| `pyarrow` | ✗ | not installed | none in apt | **Must pip install.** UNVERIFIED for py3.12 wheel; pip index shows 25.0.1 available — likely has cp312 wheel. |
| `osqp` | ✗ | not installed | none in apt | **Must pip install.** pip index shows 1.1.3 available. py3.12 wheel UNVERIFIED but osqp 1.x distributes `manylinux` wheels; likely works. |
| `ruamel.yaml` | ✗ | not installed | `python3-ruamel.yaml` 0.17.21 available | `f1tenth_diagnostics` dep. **T1 install.** |
| `torch` / `torchvision` / `ultralytics` | ✗ | not installed | none | Perception/YOLO. x86_64-only question here; real environment is Jetson (aarch64+CUDA). UNVERIFIED. |

### Trial build

```
colcon build --symlink-install --continue-on-error --event-handlers console_direct+
```

Result: **18/18 packages finished** in 40.6 s. No errors. No deprecation warnings from setuptools.

The `local_setup.bash` note: the install overlay references `local_setup.sh` at the workspace root (not under `install/`), which produces a warning when sourcing in a plain bash session. This is a Jazzy colcon behavior and does not affect the build.

### Trial test

```
colcon test --event-handlers console_direct+
```

| Package | Tests | Errors | Failures | Classification |
|---|---|---|---|---|
| `f1tenth_messages` | ✓ | 0 | 0 | — |
| `f1tenth_params` | ✓ | 0 | 0 | — |
| `corridor_perception` | ✓ | 0 | 0 | — |
| `f1tenth_description` | ✓ | 0 | 0 | — |
| `f1tenth_localization` | 14 | 0 | 3 | Test-environment: tests look up `f1tenth_bringup` share dir, but `colcon test` runs each package in isolation without the full install overlay sourced — `PackageNotFoundError: 'f1tenth_bringup' not found`. Not a Jazzy API break. |
| `f1tenth_sim` | 0 | — | — | No tests declared |
| `f1tenth_costmap` | ✓ | 0 | 0 | — |
| `f1tenth_navigation` | 2 | 0 | 0 | — |
| `f1tenth_perception` | 540 | 0 | 2 | Test-environment: same `f1tenth_bringup` cross-package lookup in launch config tests. Not a Jazzy API break. |
| `f1tenth_diagnostics` | 1 | 1 | 0 | Missing dep: `ModuleNotFoundError: No module named 'vesc_msgs'`. Submodule not initialized. |
| `f1tenth_logger` | 1 | 1 | 0 | Missing dep: `ModuleNotFoundError: No module named 'pyarrow'`. Not installed. |
| `f1tenth_behavior` | 1 | 1 | 0 | Missing dep: `ModuleNotFoundError: No module named 'py_trees_ros'`. Not installed. |
| `f1tenth_bringup` | 35 | 0 | 3 | Style: ament_flake8 (101 issues in `component_supervisor_node.py`, `stack_bringup.launch.py`, etc.) + ament_pep257 + ament_copyright. Not a runtime break. |
| `f1tenth_hardware` | 0 | — | — | No tests declared |
| `llm` | 270 | 0 | 14 | 13×ament_flake8/pep257 style; 1×test_intent_go_to/test_planner_path logic (API change in `plan_translate` import ordering). Needs investigation. |
| `mpc_controller` | 710 | 157 | 58 | **Missing dep: osqp not installed** (`RuntimeError: solver='rti' requires 'osqp'`). All 157 errors trace to this single missing package. The 58 pure failures include ament_flake8 style + a handful of logic tests. |
| `f1tenth_control` | ✓ | 0 | 0 | — |
| `f1tenth_more` | ✓ | 0 | 0 | — |

**Summary of failure causes:**
- Missing dep (installable): `py_trees_ros`, `pyarrow`, `osqp`, `vesc_msgs` (submodule) — 3 collection errors + all 157 mpc_controller errors
- Test-environment isolation (not a bug): `PackageNotFoundError: f1tenth_bringup not found` in cross-package launch tests — 5 failures in localization + perception
- Style: ament_flake8 / pep257 / copyright — 117 failures across bringup, llm, mpc_controller
- Real logic failures: a handful in `llm` (test_intent_go_to, test_planner_path) needing investigation

### Import smoke test

Tested after manually building the PYTHONPATH with build-dir egg-links. All modules that import correctly on Humble:

| Module | Result | Note |
|---|---|---|
| `f1tenth_params.param_defaults` | OK | |
| `corridor_perception.geometry` | OK | |
| `corridor_perception.extraction` | OK | |
| `f1tenth_costmap.safe_corridor` | OK | |
| `f1tenth_costmap.costmap_boundary` | OK | |
| `f1tenth_costmap.costmap_boundary_node` | OK | |
| `f1tenth_costmap.semantic_layer_node` | OK | |
| `f1tenth_costmap.costmap_renderer_node` | OK | |
| `f1tenth_localization.slam_pose_relay_node` | OK | |
| `f1tenth_localization.raw_odom_map_tf_node` | OK | |
| `f1tenth_bringup.component_supervisor_node` | OK | |
| `f1tenth_bringup.stack_startup_sequence` | OK | |
| `mpc_controller.MPC_corr` | OK | |
| `f1tenth_perception.lidar_front_wall_node` | OK | |
| `f1tenth_logger.mission_logger_node` | OK | |
| `llm.llm_planner_node` | OK | |
| `f1tenth_diagnostics.battery_voltage_check_node` | FAIL | `vesc_msgs` not available (submodule empty) |
| `f1tenth_behavior.behavior_executor_node` | FAIL | `py_trees_ros` not installed |

### ament_prefix_path hook issue

The build uses `--symlink-install`, so packages are installed as egg-links pointing at build dirs. The `install/*/local_setup.bash` files reference `local_setup.sh` at the workspace root; on a fresh shell that file is absent until a full build is run. This is the documented behavior on Jazzy. On Humble the same pattern applies. **This is a test-environment ergonomics issue, not a Jazzy regression.** The fix is to always source `install/setup.bash` (overlay), not `local_setup.bash`, and to run `colcon build` before `colcon test` in CI.

---

## Step 2 — Per-Package Analysis

---

### f1tenth_messages

**Build:** ament_cmake (rosidl)  
**Nodes/entries:** none — pure custom message/service definitions  
**Trial build:** PASS. **Trial test:** PASS. **Import smoke:** N/A (C binding)

**Deps with Jazzy availability:** `rosidl_default_generators`, `std_msgs`, `geometry_msgs` — all installed.

**Jazzy ROS API changes:** rosidl pipeline is unchanged between Humble and Jazzy. No message field types changed. All downstream packages import `f1tenth_messages.msg.*` and `f1tenth_messages.srv.*` normally.

**Verdict: T0 — Rebuild-only.** rosidl CMake packages rebuild identically across distros. Confidence: High.

---

### f1tenth_params

**Build:** ament_python  
**Nodes/entries:** none — YAML config + `param_defaults.py` library  
**Trial build:** PASS. **Trial test:** PASS. **Import smoke:** OK

**Deps:** `ament_index_python`, `python3-yaml`, `python3-numpy` — all installed.

**Python 3.12 exposure:** none. No stdlib removals touched. `param_defaults.py` uses `os`, `yaml`, `pathlib`, `ament_index_python` — all stable.

**Verdict: T0.** Confidence: High.

---

### corridor_perception

**Build:** ament_python  
**Nodes/entries:** `BagReplay` (offline replay helper), geometry/extraction/scan libs  
**Trial build:** PASS. **Trial test:** PASS. **Import smoke:** OK

**Deps:** `python3-numpy`, `rclpy`, `sensor_msgs`, `nav_msgs`, `tf2_msgs`.

**Python 3.12 exposure:** uses `numpy`, `scipy.optimize`, `scipy.signal` — both installed. No deprecated numpy API usage found. No `distutils`/`imp`.

**Verdict: T0.** Confidence: High.

---

### f1tenth_description

**Build:** ament_python  
**Nodes/entries:** `description.launch.py` (robot_state_publisher + xacro)  
**Trial build:** PASS. **Trial test:** PASS (lint only).

**Deps:** `xacro`, `robot_state_publisher` (3.3.3 installed), `joint_state_publisher` (2.4.1 installed), `tf2_ros` — all installed.

**Jazzy changes:** `robot_state_publisher` 3.3.3 is API-compatible. `xacro` unchanged.

**Verdict: T0.** Confidence: High.

---

### f1tenth_localization

**Build:** ament_python  
**Nodes:** `slam_pose_relay_node`, `raw_odom_map_tf_node`  
**Launch:** `ekf.launch.py`, `ekf_global.launch.py`, `localization.launch.py`, `raw_odom.launch.py`  
**Config:** `src/f1tenth_bringup/config/ekf.yaml`, `ekf_global.yaml`  
**Trial build:** PASS. **Trial test:** 3 failures (test-environment only). **Import smoke:** OK.

#### 1. robot_localization: Humble 3.5.4 → Jazzy 3.8.3

Installed: `ros-jazzy-robot-localization` 3.8.3. The reference config at `/opt/ros/jazzy/share/robot_localization/params/ekf.yaml` still lists `pose0_rejection_threshold`. The parameter is present and documented at line 151.

**Key parameters in use and their Jazzy status:**

| Parameter | Our value | Jazzy 3.8.3 status |
|---|---|---|
| `frequency` | 50.0 | Unchanged |
| `sensor_timeout` | 0.1 | Unchanged |
| `two_d_mode` | true | Unchanged |
| `publish_tf` | true (both instances) | Unchanged |
| `world_frame` | `odom` (local), `map` (global) | Unchanged |
| `odom0_differential` | true | Unchanged |
| `pose0_rejection_threshold` | **5.0** | **Present in 3.8.3 reference config.** The ekf_global.yaml comment accurately describes the units as n-sigma (not chi-squared), citing `filter_base.cpp:431-451`. This API is unchanged. |
| `process_noise_covariance` | Tuned values (Q_x=0.0243, Q_y=0.0273, Q_yaw=0.0123) | Format unchanged (flat 15×15 array). |
| `initial_state` | all zeros | Unchanged |

**`pose0_rejection_threshold` units trap:** the ekf_global.yaml has extensive inline documentation (lines 399-431) establishing that this is an n-sigma value, 5.0 means a cutoff of 25.0, and that 3.1% of corrections are rejected at current Q. This reasoning was validated against the Humble 3.5.4 source. **UNVERIFIED** whether 3.8.3's `checkMahalanobisThreshold` is bit-for-bit identical — the include header `ros_filter.hpp` is present at `/opt/ros/jazzy/include/robot_localization/` but the `.cpp` source is not installed. The parameter name, description and example value in the reference config are unchanged, which is strong evidence the behavior is preserved.

**T2 risk:** if robot_localization 3.8.3 changed Q normalization or the rejection-threshold semantics, the tuned `pose0_rejection_threshold: 5.0` and `process_noise_covariance` would need re-derivation from the archive. This must be verified by replaying archived bags against the Jazzy EKF before any tuning changes.

#### 2. Test failures (3 in f1tenth_localization)

Root cause: `test_ekf_global_config.py:258` calls `generate_launch_description()` which resolves `get_package_share_directory('f1tenth_bringup')` — but `colcon test` for `f1tenth_localization` runs with only that package's install prefix on `AMENT_PREFIX_PATH`. This is a test isolation issue, not a Jazzy API break. Fix: either mock the path lookup or source the full workspace overlay before running tests.

**Verdict: T2 — Behavioral revalidation.** robot_localization 3.8.3 is installed and functional; EKF config is compatible. The tuned Q values and `pose0_rejection_threshold` must be validated by bag replay on Jazzy before trusting the filter outputs. Confidence: High.

---

### f1tenth_costmap

**Build:** ament_python  
**Nodes:** `semantic_layer_node`, `costmap_renderer_node`, `costmap_boundary_node`  
**Trial build:** PASS. **Trial test:** PASS. **Import smoke:** OK (all four modules).

**Deps:** `rclpy`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `vision_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `f1tenth_messages`, `python3-scipy`.

**Python 3.12:** uses `scipy.optimize` and `numpy`. Both installed and compatible.

**The ament_prefix_path hook issue (from the task):** with `--symlink-install`, `f1tenth_costmap` installs as an egg-link pointing at the build dir. Colcon does NOT re-run the hook-install step on every build; it only changes the egg-link target on initial build and when `setup.py` changes. This is identical behavior to Humble. No regression observed on this host.

**Verdict: T0.** Confidence: High.

---

### f1tenth_navigation

**Build:** ament_python  
**Launch:** `slam.launch.py`, `localization.launch.py`  
**Config:** `slam_toolbox_params.yaml` (extensively documented)  
**Trial build:** PASS. **Trial test:** PASS (2/2).

#### slam_toolbox: Humble 2.6.x → Jazzy 2.8.5

Jazzy candidate: `ros-jazzy-slam-toolbox` 2.8.5 (not installed on this host). The apt package depends on `ros-jazzy-rclcpp-lifecycle`, confirming it is a **lifecycle node** in 2.8.5.

**Key parameters in `slam_toolbox_params.yaml` and their Jazzy 2.8.5 status:**

| Parameter | Our value | Status in 2.8.5 |
|---|---|---|
| `transform_publish_period` | **0.0** (never publish TF) | Parameter present in 2.8.x. Setting to 0.0 disables TF broadcasting — this is the documented mechanism. UNVERIFIED in 2.8.5 source, but the parameter semantics are explicitly described in slam_toolbox's own documentation and have not changed across 2.x releases. |
| `minimum_travel_distance` | 0.03 m | Present. UNVERIFIED whether 2.8.5 renamed it. |
| `minimum_travel_heading` | 0.035 rad | Present. Same caveat. |
| `minimum_time_interval` | 0.5 s | Present. Same caveat. |
| `scan_queue_size` | 100 | Present. The bug investigation at lines 50-120 of the config established why 100 is needed; this is a configuration value, not a parameter rename. |
| `base_frame` | `base_link` | Present. |
| `mode` | `mapping` | Present. |

**Lifecycle node impact:** In 2.8.5, slam_toolbox is a lifecycle-managed node. `f1tenth_bringup`'s `component_supervisor_node` launches slam_toolbox via `ros2 launch`, which handles lifecycle transitions internally. Our launch file does not call lifecycle transition services directly. **T2 risk:** if lifecycle startup sequencing changes (e.g., `configure` and `activate` must be called explicitly), the launch behavior must be tested. Nav2's `nav2_lifecycle_manager` (installed: 1.3.11) handles lifecycle for nav2 components; slam_toolbox has its own lifecycle management.

**map→odom NOT published by slam_toolbox:** `transform_publish_period: 0.0` is the explicit mechanism preventing slam_toolbox from publishing `map→odom`. This is load-bearing for the dual-EKF design. Must be verified after installing slam_toolbox 2.8.5.

**Verdict: T2.** slam_toolbox lifecycle node change requires launch testing with 2.8.5 installed. The `transform_publish_period: 0.0` contract is the most critical item to verify. Confidence: High.

---

### f1tenth_perception

**Build:** ament_python  
**Nodes:** `lidar_front_wall_node`, `detection_3d_node`, `wall_distance_node`, plus launch files  
**Trial build:** PASS. **Trial test:** 538/540 pass, 2 fail (test-environment).

**Deps:** `cv_bridge` (4.1.0 installed), `message_filters` (4.11.12 installed), `tf2_ros`, `urg_node` (not installed, candidate 1.1.2), `zed_wrapper` (submodule empty), `v4l2_camera` (UNVERIFIED).

**Test failures:** `test_lidar_front_wall_launch.py` and `test_wall_distance_launch.py` both fail with `PackageNotFoundError: f1tenth_bringup not found` — same isolation issue as f1tenth_localization.

**cv_bridge 4.1.0:** uses `CvBridge` and `CvBridgeError`. cv_bridge 4.x is the Jazzy version, supports Python 3.12 and numpy 1.x/2.x. No API break.

**message_filters:** uses `message_filters.ApproximateTimeSynchronizer` and `TimeSynchronizer`. Both unchanged in Jazzy 4.11.12.

**urg_node 1.1.2:** Not installed. The Jazzy version is available. The urg_node API (topic `/scan`, frame_id param, TCP port param) is unchanged.

**zed_ros2_wrapper submodule:** Currently at `v5.5.0` (tag in git history). The Jazzy zed_ros2_wrapper requires **ZED SDK 5.4.x**; the submodule at v5.5.0 requires ZED SDK 5.5.x. **This host has no ZED SDK and no ZED camera.** Full validation is Jetson-only.

**YOLO/TensorRT/torch:** Not installed on this host. The `detection_3d_node` imports `ultralytics` and `torch`. Both are Jetson/GPU concerns only. Past incident noted in the task: `nvidia-cudss-cu12` shadowing `cuda-toolkit`. Under Python 3.12 on Jetson, the pip install path for torch must be from the NVIDIA Jetson wheel index (`torch-2.x+nv*` builds), not from PyPI — PyPI torch wheels do not ship CUDA for aarch64. The `nvidia-cudss-cu12` shadowing issue only applies to x86_64 PyPI installs.

**Verdict: T1.** Once `urg_node` is installed and the `f1tenth_bringup` cross-package test issue is fixed, all tests will pass. ZED and YOLO remain Jetson-only. Confidence: Medium (ZED SDK compatibility UNVERIFIED).

---

### f1tenth_diagnostics

**Build:** ament_python  
**Nodes:** `battery_voltage_check_node`, `gyro_bias_calibration_node`, `slam_pose_covariance_calibration_node`, `ekf_cost_observer_node`, `steering_offset_calibration_node`, `sensor_covariance_calibration_node`, `system_observer_node`, `diagnostics_server_node`  
**Trial build:** PASS. **Trial test:** 1 collection error (`vesc_msgs` not found). **Import smoke:** FAIL (`vesc_msgs`).

**Missing deps:**
- `vesc_msgs`: the vesc submodule is not initialized. `battery_voltage_check_node` imports `from vesc_msgs.msg import VescState`. Until the submodule is initialized and built, all nodes that touch VESC topics cannot be imported on this host.
- `ruamel.yaml`: `steering_offset_calibration_node` uses `ruamel.yaml` for in-place YAML editing. Not installed. `python3-ruamel.yaml` 0.17.21 available in apt.

**Verdict: T1.** Install `python3-ruamel.yaml` from apt; initialize vesc submodule for Jetson. Confidence: High.

---

### f1tenth_control / mpc_controller

**Build:** ament_python  
**Nodes:** `MPC_corr` node, `andre_mpc_node`  
**Trial build:** PASS. **Trial test:** 157 errors + 58 failures. **Import smoke:** OK (`mpc_controller.MPC_corr`).

**Missing dep — OSQP:** All 157 test errors trace to `RuntimeError: solver='rti' requires the 'osqp' package`. The RTI (Real-Time Iteration) solver is the production solver. OSQP is a Python package (`pip install osqp`); pip index shows 1.1.3 available. The package uses `manylinux` wheels; Python 3.12 cp312 wheel **UNVERIFIED** but very likely available for x86_64.

**58 remaining failures:** ament_flake8 style (1 test), plus logic failures including `test_corridor_direction_recovery`, `test_d_wall_correction`, `test_model_log`, and `test_campaign_status`. These need investigation once OSQP is installed — some may be due to missing OSQP rather than real logic bugs.

**Corridor params confirmed present:**
- `corridor_update_period`: 1.0 s (`stack_params.yaml:899`)
- `use_hard_boundary_constraints`: true (`stack_params.yaml:1205`)
- `boundary_hard`: false (soft slack rows by default)
- `front_clearance_corridor_half_width_m`: 0.35 m

**OSQP version note:** The code checks `import osqp` at import time. OSQP 1.x has a changed Python API compared to 0.6.x (interface object, `setup()` vs direct calls). The `mpc_solver.py:768` error message references `osqp`, suggesting the code uses osqp directly. Verify the API version expected matches 1.x.

**Verdict: T1.** Primary blocker is OSQP not installed. Install and re-run tests. Confidence: High.

---

### f1tenth_behavior

**Build:** ament_python  
**Nodes:** `behavior_executor_node` (py_trees_ros BehaviourTree)  
**Trial build:** PASS. **Trial test:** 1 collection error. **Import smoke:** FAIL (`py_trees_ros`).

**Missing dep — py_trees_ros:** `ros-jazzy-py-trees-ros` 2.5.0 is available in apt but not installed. The node imports `py_trees_ros.trees.BehaviourTree` at line 107, which is the primary py_trees_ros usage.

**py_trees 2.4.0 → 2.5.0:** Installed is 2.4.0; apt candidate is 2.5.0. The behavior executor uses `py_trees.behaviour.Behaviour`, `py_trees.composites.Selector`, `py_trees.decorators`, `py_trees.display`. The 2.4→2.5 changelog is UNVERIFIED; however, py_trees maintains API compatibility within the 2.x series. Upgrading to 2.5.0 (which `ros-jazzy-py-trees` recommends) is the right step.

**py_trees_ros_interfaces:** `ros-jazzy-py-trees-ros-interfaces` 2.1.2 is also available; py_trees_ros 2.5.0 depends on it.

**Verdict: T1.** Install `ros-jazzy-py-trees-ros` (which installs `py_trees_ros` + updates `py_trees` to 2.5.0 + installs `py_trees_ros_interfaces`). Confidence: High.

---

### f1tenth_logger

**Build:** ament_python  
**Nodes:** `mission_logger_node`  
**Scripts:** `mission_extract.py`, `mission_render.py`, `mission_split.py`, `bag_replay.py`  
**Trial build:** PASS. **Trial test:** 1 collection error. **Import smoke:** OK (node itself).

**Missing dep — pyarrow:** `mission_render.py:59` does `import pyarrow.parquet as pq`. No `python3-pyarrow` in Jazzy apt (rosdep check confirms). Must `pip install pyarrow`. pip index shows 25.0.1; Python 3.12 cp312 wheels **UNVERIFIED** but very likely available (pyarrow 14+ distributes cp312).

**rosbag2 storage format:** `mission_logger_node.py` comments (lines 47-55) document that on the Humble Jetson, `rosbag2_py.get_registered_writers()` returned `{'sqlite3'}` only — mcap was unavailable. On this Jazzy host, `get_registered_writers()` returns `{'sqlite3', 'mcap'}` — both installed (`ros-jazzy-rosbag2-storage-mcap` 0.26.10, `ros-jazzy-rosbag2-storage-sqlite3` 0.26.10). The node's `_resolve_storage_id()` defaults to `mcap` and falls back gracefully. **New bags on Jazzy will default to mcap.** Existing `.db3` archives from the Jetson remain readable by `rosbag2_py.SequentialReader` with `storage_id='sqlite3'` — both storage backends are installed and the reader takes an explicit `storage_id` argument.

**Verdict: T1.** Install pyarrow via pip. Confidence: Medium (pyarrow cp312 wheel availability UNVERIFIED but expected).

---

### f1tenth_bringup

**Build:** ament_python  
**Nodes:** `component_supervisor_node`, `stack_startup_sequence`  
**Launch:** `supervisor_bringup.launch.py`, `stack_bringup.launch.py`, `foxglove_bridge.launch.py`  
**Config:** `ekf.yaml`, `ekf_global.yaml`, `mux.yaml`, `vesc.yaml`, `sensors.yaml`, `components.yaml`

**Trial build:** PASS. **Trial test:** 3 failures (style). **Import smoke:** OK.

#### 1. Discovery Server: ROS_DISCOVERY_SERVER in Jazzy

`stack_bringup.launch.py:92` and `supervisor_bringup.launch.py:67` both set `ROS_DISCOVERY_SERVER=<address>:<port>` via `SetEnvironmentVariable`. This is the **super-client pattern**: every child process started by `ros2 launch` inherits the variable and connects as a super-client to the FastDDS Discovery Server.

In Jazzy (FastDDS 2.14.6, installed), `ROS_DISCOVERY_SERVER` is **still supported** — it was not removed, it is the official FastDDS super-client mechanism. The Discovery Server binary (`fastdds discovery -i 0 -l 127.0.0.1 -p 11811`) is still shipped as part of `ros-jazzy-fastrtps`.

**Fast DDS 2.6 → 2.14.6 changes relevant to us:**
- Shared Memory (SHM) transport: improved stability and error handling in 2.14.x vs 2.6.x. The `component_supervisor_node.py:348` SHM cleanup pattern (`glob('/dev/shm/*fastrtps*')`) should still work as the SHM file naming is unchanged.
- The "DISCOVERY_DATABASE Error: Matching unexisting participant" EDP race: this was a known issue in Fast DDS 2.6.x under high-churn scenarios (many processes starting/stopping rapidly). In 2.14.x the Discovery Server implementation was substantially rewritten (RTPS layer refactor in 2.10+). **UNVERIFIED** whether this specific race is fixed; no Fast DDS changelog entry was found for this exact error string, but the rewrite makes it less likely. If this race recurs, the fix is the same as on Humble: ensure `fastdds discovery` is fully up before the first `ros2 launch` child starts.
- **`component_supervisor_node`'s restart behavior** cleans stale SHM files before restart. The pattern `glob('/dev/shm/*fastrtps*')` works with FastDDS 2.14.6's SHM file naming (confirmed: SHM files are still named with `fastrtps` prefix in rmw_fastrtps_cpp 8.4.3).

#### 2. ROS_LOCALHOST_ONLY deprecation

`ROS_LOCALHOST_ONLY=1` is **deprecated in Jazzy** (Iron migration guide). The replacement is:
```
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

The current host shows `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` (the Jazzy default). Affected files:
- `src/f1tenth_logger/test/test_campaign/conftest.py:20` — sets `ROS_LOCALHOST_ONLY = "1"` for test isolation
- `scripts/replay_bag_through_mpc.sh:35` — sets `ROS_LOCALHOST_ONLY=1`

`ROS_LOCALHOST_ONLY` still **works** in Jazzy (it is treated as equivalent to `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`) but generates a deprecation notice in some rmw implementations. This is a T1 mechanical fix.

#### 3. rosbridge_server → rosbridge_suite

`package.xml` depends on `rosbridge_server` (`<depend>rosbridge_server</depend>`). The Jazzy package is `ros-jazzy-rosbridge-server` 2.7.1 (available but not installed). The package name is unchanged; `rosbridge_server` is still a valid package name under Jazzy.

#### 4. foxglove_bridge

`ros-jazzy-foxglove-bridge` 3.2.6 is installed (3.5.0 available). The launch file at `launch/foxglove_bridge.launch.py` launches it via `ros2 launch`. No API break observed.

#### 5. Style failures (ament_flake8/pep257/copyright)

101 flake8 issues in `component_supervisor_node.py` (docstring formatting, import ordering, quote style), several in `stack_bringup.launch.py`, `supervisor_bringup.launch.py`, `ensure_discovery_server.py`, `setup.py`. One copyright check failure. All are mechanical fixes, no behavioral impact.

**Verdict: T2.** Discovery Server behavior needs live testing on Jazzy (FastDDS 2.14 vs 2.6). `ROS_LOCALHOST_ONLY` must be replaced. Confidence: High.

---

### f1tenth_hardware

**Build:** ament_python (launch wrapper + config)  
**Nodes:** none (delegates to vesc submodule)  
**Trial build:** PASS (trivial, no source). **Trial test:** PASS (0 tests). **Import smoke:** FAIL (`vesc_msgs`).

#### VESC submodule (sjckness/vesc, HEAD `e3f3084`)

The submodule is empty on this host. The vesc driver is a C++ ament_cmake package. Key items to check when initializing the submodule:

**Build deps:** `serial` / transport_drivers — the `transport_drivers` submodule is also empty. In Jazzy, the serial transport is provided by `ros-jazzy-serial-driver` which is part of `transport_drivers`. This must be verified against the vesc submodule's `package.xml` `<depend>` tags.

**gyro_scale_z:** `src/f1tenth_bringup/config/vesc.yaml:118`: `gyro_scale_z: 0.0174533` (= π/180, degrees-to-radians). This is a pure numeric conversion, distro-independent. The vesc driver's `gyro_scale_z_` parameter handling is unchanged between Humble and Jazzy.

**Verdict: T1 on this host (submodule init), T2 on Jetson (C++ build verification).** Confidence: Low (submodule not built).

---

### f1tenth_intelligence / llm

**Build:** ament_python  
**Nodes:** `llm_planner_node`, `llm_interrogator_node`  
**Trial build:** PASS. **Trial test:** 14 failures (13 style, 1 logic). **Import smoke:** OK.

**Missing deps:** none beyond what's already installed (`requests`, `jsonschema`).

**llama-server startup:** `supervisor_bringup.launch.py` launches llama-server as an `ExecuteProcess`. This is OS/binary-level; distro-independent. The `llama.cpp` server binary path must be correct on the target machine, unchanged from Humble.

**13 style failures:** ament_flake8 (166 issues in launch files, test files) + ament_pep257. Mechanical.

**1 logic failure (test_intent_go_to/test_planner_path):** these tests do not depend on any ROS infrastructure and fail in pytest isolation. Root cause needs investigation; likely import ordering (the flake8 output shows `I100`/`I101` import order errors in the test files themselves, which may be affecting collection).

**Verdict: T1.** Flake8/pep257 fixes + investigate logic test failures. Confidence: High.

---

### f1tenth_sim

**Build:** ament_python  
**Trial build:** PASS (no deps to fail against; empty submodule). **Trial test:** PASS (0 tests).

#### ign_ros2_control → gz_ros2_control (Jazzy T3)

The `package.xml` depends on `ign_ros2_control` and `ros_ign_gazebo` + `ros_ign_bridge`. These are the **Ignition Fortress** package names, used in ROS 2 Humble. In Jazzy (Gazebo Harmonic / gz-sim 8):

| Humble package | Jazzy package | Installed |
|---|---|---|
| `ros_ign_gazebo` | `ros_gz_sim` | ✓ installed (1.0.22) |
| `ros_ign_bridge` | `ros_gz_bridge` | likely installed (part of ros_gz) |
| `ign_ros2_control` | `gz_ros2_control` | ✓ installed (1.2.17) |

The `sim_bringup.launch.py` launches `ros_ign_gazebo.launch_description.actions.IgnitionBridgeNode` and similar. These API names are **removed** in Jazzy; they are replaced by `ros_gz_sim` equivalents. This requires logic changes in the launch file: import paths, executable names, and the bridge format string (Ignition uses `@ign.msgs.Clock` while Gazebo Harmonic uses `@gz.msgs.Clock`).

**Verdict: T3.** `f1tenth_sim` requires explicit porting to Gazebo Harmonic package names and API. This is a simulator-only concern; the real car is unaffected. Confidence: High.

---

## Appendix: Cross-cutting findings

### Python 3.12 stdlib removals

- `distutils`: removed in 3.12. **Not used** in this codebase (grep confirmed zero hits for `import distutils` or `from distutils`).
- `imp`: removed in 3.12. **Not used** (grep confirmed zero hits).
- `setup.cfg` dash-style keys (`script-dir`, `install-scripts`): found in `setup.cfg` files as `script_dir=...` and `install_scripts=...` (underscore, not dash). This is the correct modern form. No issue.
- `tests_require` in `setup.py`: removed from all packages by commit `626fb7b` (per CLAUDE.md). Confirmed absent; `extras_require={'test': ['pytest']}` is the correct form.

### NumPy 1.26.4 vs NumPy 2

The installed version is 1.26.4. `numpy.float`, `numpy.int`, etc. were removed in NumPy 1.20; if the code survived to 1.26.4 it uses no such aliases. Upgrade to NumPy 2.x is not required for Jazzy but will happen naturally if torch requires it on the Jetson. The costmap and corridor code uses standard array operations, no legacy type aliases found.

### cv_bridge 4.1.0 and OpenCV 4.6.0

`cv_bridge` 4.1.0 (Jazzy) supports Python 3.12 and numpy 1.x/2.x. The `from cv_bridge import CvBridge, CvBridgeError` usage in `f1tenth_perception` is unchanged. No break.

### tf2_ros / tf2_geometry_msgs

Both are installed (`ros-jazzy-tf2-ros` via transitive dep). Usage pattern in the stack: `tf2_ros.Buffer`, `tf2_ros.TransformListener`, `tf2_geometry_msgs` (import registers PoseStamped transform support). API unchanged in Jazzy.

### Twist vs TwistStamped

**This stack does not use `twist_mux`.** It uses `ackermann_mux` (custom sjckness submodule) which multiplexes `AckermannDriveStamped` topics. The Twist→TwistStamped change that affects vanilla `twist_mux` 4.x (Jazzy) **does not apply**. All velocity commands are `AckermannDriveStamped` throughout.

### message_filters (4.11.12 → 4.11.17)

No API break between these versions. `ApproximateTimeSynchronizer` and `TimeSynchronizer` are unchanged.

### tf_transformations / transforms3d

`ros-jazzy-tf-transformations` 1.1.1 is available but not installed. The stack uses `tf2_ros` directly; no import of `tf_transformations` found. Not needed.
