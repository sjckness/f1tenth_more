# F1Tenth Jazzy Migration Plan

Generated: 2026-09-30  
Based on: `output/jazzy_migration_analysis.md`

---

## T2/T3 Items and Non-Trivial T1 Items

---

### Item 1 — robot_localization 3.8.3: validate `pose0_rejection_threshold` and Q matrix (T2)

**Problem:** The global EKF's `pose0_rejection_threshold: 5.0` and `process_noise_covariance` (Q_x=0.0243, Q_y=0.0273, Q_yaw=0.0123) were derived empirically from 689 archived intervals on the Humble stack (robot_localization 3.5.4). The Jazzy version is 3.8.3. The parameter name is present in the 3.8.3 reference config with the same description, but the `checkMahalanobisThreshold` implementation cannot be verified from headers alone (`.cpp` source not installed).

**Options:**

| Option | Pros | Cons | Effort |
|---|---|---|---|
| a) Trust the parameter is unchanged, validate by bag replay | Low risk if behavior matches; fast to check | Does not rule out subtle numeric changes in 3.8.3's filter propagation | S |
| b) Re-derive Q and threshold from scratch on Jazzy | Definitive | Requires multiple real runs; weeks of work | L |
| c) Adopt Jazzy defaults then retune | Correct if API changed | Loses calibrated values | M |

**Recommendation:** Option (a). The risk is low: `pose0_rejection_threshold` is a long-standing parameter with no changelog entry indicating behavioral change between 3.5.4 and 3.8.3. Run the existing archived bags through the Jazzy EKF node (`ros2 run robot_localization ekf_node` with the same YAML) and compare `/ekf_global/odometry/filtered` and `/tf` (map→odom) against the Humble replay baseline. If the outputs match within noise, proceed. If they differ materially, investigate the 3.x changelog.

**Re-validation:** replay archived runs (specifically the 11-run set used to calibrate Q_yaw) through the Jazzy EKF node. Compare:
- map→odom yaw step size per SLAM correction (acceptance: largest step ≤ 5.54 deg, matching the current archive best)
- rejection rate at `pose0_rejection_threshold: 5.0` (acceptance: 3±2% of corrections rejected)
- `/odometry/filtered` output frequency and phase noise

**Acceptance criterion before moving on:** same rejection rate (±5%) and same maximum yaw step (±30%) across the 11-run replay set.

---

### Item 2 — slam_toolbox 2.8.5 lifecycle node: verify `transform_publish_period: 0.0` and launch behavior (T2)

**Problem:** slam_toolbox 2.8.5 (Jazzy) is a lifecycle-managed node (`rclcpp_lifecycle::LifecycleNode`). Our bringup launches it via `ros2 launch slam_toolbox online_async_launch.py`. If the Jazzy launch file requires explicit `configure`/`activate` lifecycle transitions that Humble's non-lifecycle version did not, slam_toolbox will never start scanning. The critical invariant is `transform_publish_period: 0.0` — slam_toolbox must NOT publish map→odom.

**Options:**

| Option | Pros | Cons | Effort |
|---|---|---|---|
| a) Install slam_toolbox 2.8.5, use its own launch file unchanged | Minimal our-code change; its own launch file handles lifecycle transitions | Must verify TF silence (transform_publish_period: 0.0) is honored in 2.8.5 | S |
| b) Use nav2_lifecycle_manager to manage slam_toolbox alongside nav2 | Standard Nav2 pattern | Adds coupling; our launch already separates slam from nav2 | M |
| c) Call lifecycle transition services explicitly in our launch | Full control | Fragile; transition timing races | M |

**Recommendation:** Option (a). Install slam_toolbox (`sudo apt install ros-jazzy-slam-toolbox`), use its own `online_async_launch.py` (which handles configure/activate internally), pass `slam_params_file:=<our yaml>`. The `transform_publish_period: 0.0` parameter is evaluated inside slam_toolbox; as long as our YAML is loaded, the invariant holds. Verify in a running stack by checking `/tf` does NOT contain a map→odom edge from slam_toolbox (only from the global EKF).

**Re-validation:**
- Start slam_toolbox with `transform_publish_period: 0.0` and confirm no map→odom TF edge from slam_toolbox: `ros2 run tf2_tools view_frames` should show map→odom owned by `/ekf_global/ekf_global_filter_node` only.
- Confirm `/slam/pose` publishes after `minimum_travel_distance: 0.03` is crossed.
- Replay a known mapping bag and compare `minimum_travel_distance`/`minimum_travel_heading` gate behavior (drop rate in the scan queue; compare to the 0% drop after scan_queue_size: 100 was set).

**Acceptance criterion:** no map→odom from slam_toolbox in `/tf`; `/slam/pose` publishes ≥ 1 message after 0.03 m of travel.

---

### Item 3 — ROS_LOCALHOST_ONLY → ROS_AUTOMATIC_DISCOVERY_RANGE (T1, multiple files)

**Problem:** `ROS_LOCALHOST_ONLY=1` is deprecated in Jazzy. While it still works (treated as `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`), it generates deprecation warnings and should not be relied upon. Affected files:
- `src/f1tenth_logger/test/test_campaign/conftest.py:20`
- `scripts/replay_bag_through_mpc.sh:35`

**Options:**

| Option | Pros | Cons | Effort |
|---|---|---|---|
| a) Replace with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` | Correct Jazzy idiom | Simple; two files | S |
| b) Leave as-is | No immediate break | Deprecation noise; may break in a future Jazzy patch | S |

**Recommendation:** Option (a). One-line change per file. Note: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` combined with `ROS_DISCOVERY_SERVER` unset is the Jazzy way to say "local only, no discovery server". The conftest.py also does `os.environ.pop("ROS_DISCOVERY_SERVER", None)` which remains correct.

---

### Item 4 — pyarrow: no apt package in Jazzy (T1 — install decision)

**Problem:** `f1tenth_logger` requires `pyarrow` for `mission_render.py`. No `python3-pyarrow` exists in Jazzy apt (rosdep check confirmed). Must pip-install. Ubuntu 24.04 enforces PEP 668 (externally-managed environment); `pip install pyarrow` without `--break-system-packages` will fail.

**Options:**

| Option | Pros | Cons | Effort |
|---|---|---|---|
| a) `pip install pyarrow --break-system-packages` (on this dev host) | Immediate; pyarrow is pure Python after native build | Breaks PEP 668; not for production | S |
| b) Use a virtual environment (`python3 -m venv .venv`) | PEP 668 compliant | Requires sourcing `.venv` as well as ROS overlay; complicates colcon test | S |
| c) Add `pyarrow` to a `requirements.txt` and install in a Docker dev container | Clean; reproducible | Requires container setup | M |
| d) Remove pyarrow from the render path; use parquet via `pandas` (which has apt package) | Eliminates the missing dep | Non-trivial code change; pandas parquet support requires `pyarrow` anyway | M |

**Recommendation:** Option (a) for this dev host. For the Jetson production environment and any CI, option (c) (Docker with requirements.txt). `pip index` shows pyarrow 25.0.1 available; cp312 wheels exist for x86_64. The `package.xml` currently declares `python3-pyarrow` as an exec_depend — update to a comment noting pip-install only.

---

### Item 5 — osqp: pip-only package (T1 — install decision)

**Problem:** `mpc_controller` requires `osqp` for the RTI solver. Not in apt. Same PEP 668 constraint.

**Options:** same as pyarrow above. Recommendation: `pip install osqp --break-system-packages` on this dev host. Verify the installed version is 1.x compatible with the code's usage pattern (`mpc_solver.py` — check whether it uses the 0.6.x or 1.x API).

**Re-validation:** after installing osqp, the 157 test errors in `mpc_controller` should collapse to zero or near-zero. Run `colcon test --packages-select mpc_controller` and compare result counts.

---

### Item 6 — py_trees_ros: not installed (T1)

**Problem:** `f1tenth_behavior` requires `ros-jazzy-py-trees-ros` which is not installed.

**Options:**

| Option | Pros | Cons | Effort |
|---|---|---|---|
| a) `sudo apt install ros-jazzy-py-trees-ros` | Clean apt install | py_trees upgrades 2.4.0→2.5.0 | S |
| b) Pin to current py_trees 2.4.0, build py_trees_ros from source | No surprise upgrade | Non-standard; harder to maintain | M |

**Recommendation:** Option (a). py_trees 2.x maintains API compatibility within the series. The behavior executor uses `py_trees_ros.trees.BehaviourTree` — verify it is present and unchanged in 2.5.0 (likely yes; it is the primary py_trees_ros class). After installing, run the behavior test suite.

**py_trees 2.5.0 API risk:** UNVERIFIED. Run import smoke after install: `python3 -c "import py_trees_ros; print(py_trees_ros.__version__)"`. If imports succeed, proceed.

---

### Item 7 — f1tenth_sim: ign→gz package name migration (T3)

**Problem:** `f1tenth_sim` uses Ignition Fortress package names (`ros_ign_gazebo`, `ign_ros2_control`) that do not exist in Jazzy. The Jazzy packages are `ros_gz_sim` and `gz_ros2_control`. The bridge message format strings also change (`@ignition.msgs.*` → `@gz.msgs.*`).

**Options:**

| Option | Pros | Cons | Effort |
|---|---|---|---|
| a) Port to Gazebo Harmonic (ros_gz_sim + gz_ros2_control) | Correct Jazzy simulator | Non-trivial launch file edits; bridge format strings; xacro plugins | M |
| b) Defer sim entirely (sim is not the Jetson path) | Zero effort now | Sim stays broken on Jazzy dev host | S |

**Recommendation:** Option (b) for now — the real vehicle work takes priority. Mark `f1tenth_sim` as "Jazzy sim port deferred" and add a `COLCON_IGNORE` file to prevent it from being built when you want a clean first-party-only build. Port it as a separate, later work item.

**Specific changes needed when sim is ported:**
- `package.xml`: replace `ros_ign_gazebo`→`ros_gz_sim`, `ros_ign_bridge`→`ros_gz_bridge`, `ign_ros2_control`→`gz_ros2_control`
- `sim_bringup.launch.py`: update import paths and executable names; update bridge format strings (`@ignition.msgs.Clock` → `@gz.msgs.Clock` etc.)

---

### Item 8 — FastDDS 2.14 Discovery Server: validate super-client behavior and SHM cleanup (T2)

**Problem:** The stack relies on FastDDS Discovery Server at `127.0.0.1:11811`. Fast DDS went from 2.6 (Humble) to 2.14.6 (Jazzy). The super-client pattern (`ROS_DISCOVERY_SERVER=127.0.0.1:11811`) is unchanged, but internal Discovery Server behavior changed significantly in the 2.6→2.14 range (RTPS layer rewrite in 2.10, Discovery Database in 2.7+).

**Specific risk:** `component_supervisor_node.py:348` cleans up stale SHM files matching `*fastrtps*` in `/dev/shm` before restarting. This pattern is still correct (FastDDS 2.14.6 still uses `fastrtps`-prefixed SHM files, confirmed by the installed dpkg name `ros-jazzy-fastrtps`).

**Discovery Database EDP race:** The "DISCOVERY_DATABASE Error: Matching unexisting participant" race seen under Humble with rapid process churn was a FastDDS 2.6.x issue. FastDDS 2.14.6 has a substantially rewritten Discovery Server. **UNVERIFIED** in this specific codebase, but the rewrite makes a recurrence less likely. Monitor for this log message in the first Jazzy live runs.

**Re-validation:**
- Start `fastdds discovery -i 0 -l 127.0.0.1 -p 11811` on the Jetson
- Start `supervisor_bringup.launch.py` with no car hardware
- Verify all expected topics are visible in `ros2 topic list` from a separate terminal also using `ROS_DISCOVERY_SERVER=127.0.0.1:11811`
- Restart one component via the supervisor and confirm it rejoins the discovery graph

---

### Item 9 — ruamel.yaml: apt-installable (T1)

**Problem:** `f1tenth_diagnostics` depends on `ruamel.yaml` (`python3-ruamel.yaml` 0.17.21 available in apt but not installed).

**Recommendation:** `sudo apt install python3-ruamel.yaml`. No API break in 0.17.21 vs the version used on Humble (the round-trip YAML API is stable).

---

### Item 10 — ament_flake8 / pep257 style failures (T1, multiple packages)

**Problem:** ament_flake8 and pep257 checks fail across `f1tenth_bringup`, `llm`, `mpc_controller` with 200+ combined issues. These are docstring formatting (`D205`, `D400`, `D401`), import ordering (`I100`, `I101`), and quote style (`Q000`, `Q003`). None affect runtime behavior.

**Recommendation:** Fix as part of each package's commit. Run `ament_flake8 .` and `ament_pep257 .` locally, fix, commit. This is straightforward but tedious. Group the fixes for each package into a single "style" commit.

**Note on `f1tenth_bringup` copyright failure:** one file is missing a copyright/license header. `ament_copyright` expects SPDX-style or similar headers. Add the appropriate header to the offending file.

---

## Environment and Coexistence

### Current state

All environment setup is via `~/.bashrc` sourcing `/opt/ros/jazzy/setup.bash`. No Humble paths were found in `.bashrc`, confirming this host is Jazzy-only. No ROS environment variables are set in `.bashrc` (no `ROS_DOMAIN_ID`, no `ROS_DISCOVERY_SERVER`, no `RMW_IMPLEMENTATION`).

### Proposed per-distro env scripts

Create two scripts (not bash profiles, just sourced files):

**`~/ros_jazzy.sh`** (for this dev host):
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Discovery Server will be set by launch files; do NOT set here
# ROS_LOCALHOST_ONLY is deprecated; use:
# export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # only for dev without a server
```

**`~/ros_humble.sh`** (for the Jetson — to be created there):
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# ROS_LOCALHOST_ONLY=1 (Humble idiom; still valid there)
```

On the Jetson, remove the Humble source line from `~/.bashrc` and instead source the appropriate script per-session. This prevents accidental cross-contamination if a Jazzy container is added later.

### Discovery Server startup under Jazzy

On the Jetson (Jazzy, when migrated):
```bash
fastdds discovery -i 0 -l 127.0.0.1 -p 11811 &
```
The `ensure_discovery_server.py` script in `f1tenth_bringup/scripts/` already does this check and start; it remains correct under Jazzy.

---

## Component-by-Component Bring-up Order

### Phase 0: Foundation (this host, offline)

**Goal:** build succeeds, tests pass, imports smoke cleanly.

**Work:**
1. Initialize submodules: `git submodule update --init` (or skip hardware/external for offline work: `git submodule update --init src/f1tenth_external/teleop_tools`)
2. Install missing apt packages: `ros-jazzy-py-trees-ros`, `ros-jazzy-slam-toolbox`, `ros-jazzy-urg-node`, `ros-jazzy-twist-mux`, `ros-jazzy-tf-transformations`, `ros-jazzy-rosbridge-server`, `python3-ruamel.yaml`
3. Install missing pip packages: `pip install osqp pyarrow --break-system-packages`
4. Fix `ROS_LOCALHOST_ONLY` → `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in conftest.py and replay script
5. Fix ament_flake8/pep257/copyright style failures
6. Add `COLCON_IGNORE` to `src/f1tenth_sim` until Gazebo Harmonic port is scheduled

**Go/no-go:** `colcon build && colcon test` shows ≤ 5 failures, all in known categories (cross-package lookup, hardware-absent nodes).

---

### Phase 1: Messages and params (this host, offline)

**What:** `f1tenth_messages`, `f1tenth_params`, `corridor_perception`  
**Validated on this host:** build + tests pass. Import smoke OK.  
**Needs real car:** nothing.  
**Go/no-go:** 0 test failures.

**Commit grouping:** "T0: messages, params, corridor_perception — no changes needed"

---

### Phase 2: Description and localization (this host, offline)

**What:** `f1tenth_description`, `f1tenth_localization`  
**Validated on this host:** build + tests. Fix cross-package lookup issue in `test_ekf_global_config.py` (source full overlay before running tests, or mock `get_package_share_directory`).  
**Needs real car:** EKF bag replay validation (Item 1 above).  
**Go/no-go on this host:** all localization tests pass. Go/no-go for real car: EKF replay matches baseline.

**Commit grouping:** "T1: f1tenth_localization — fix cross-package test lookup; T2 note: Q/threshold validation pending bag replay"

---

### Phase 3: Costmap and navigation (this host, offline)

**What:** `f1tenth_costmap`, `f1tenth_navigation`  
**Validated on this host:** build + tests pass (currently passing). After installing slam_toolbox: verify slam_toolbox launch doesn't break `f1tenth_navigation` tests.  
**Needs real car:** slam_toolbox live test (Item 2).  
**Go/no-go on this host:** all tests pass. Go/no-go for real car: no map→odom from slam_toolbox in live TF tree.

---

### Phase 4: Control — MPC (this host, offline)

**What:** `mpc_controller`, `f1tenth_control`  
**Validated on this host:** install osqp; run test suite. Expect 157 errors to clear. Remaining failures: style + a handful of logic tests. Fix each.  
**Needs real car:** corridor/boundary behavior. Replay bags through `replay_bag_through_mpc.sh` (update `ROS_LOCALHOST_ONLY` → `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` first).  
**Go/no-go on this host:** all non-style test failures explained and fixed.  
**Go/no-go for real car:** same bag replays produce same `AckermannDriveStamped` output (within 5% on steering angle, within 10% on speed) vs Humble baseline.

---

### Phase 5: Behavior (this host, offline)

**What:** `f1tenth_behavior`  
**Validated on this host:** install py_trees_ros; run full test suite.  
**Needs real car:** BT tick behavior; mission execution.  
**Go/no-go on this host:** all py_trees_ros-dependent tests pass.

---

### Phase 6: Logger and diagnostics (this host, offline)

**What:** `f1tenth_logger`, `f1tenth_diagnostics` (non-VESC parts), `f1tenth_intelligence/llm`  
**Validated on this host:** install pyarrow, ruamel.yaml; run test suites; verify rosbag2 mcap recording works with `mission_logger_node`.  
**Needs real car:** nothing (logger works offline with bag replay).  
**Go/no-go:** logger test suite passes; mcap bags are written and readable.

---

### Phase 7: Bringup (this host, offline then real car)

**What:** `f1tenth_bringup`  
**Validated on this host:** all style failures fixed; import smoke clean; `ensure_discovery_server.py` launches correctly; `component_supervisor_node` imports and initializes without VESC (using `--simulate-hardware` or equivalent).  
**Needs real car:** Discovery Server live test (Item 8); full supervisor bring-up cycle.  
**Go/no-go on this host:** 0 test failures. Go/no-go for real car: supervisor starts, all software components report healthy, restart cycle works.

---

### Phase 8: Hardware drivers (Jetson only)

**What:** `f1tenth_hardware` (VESC submodule), `ackermann_mux` submodule, `teleop_tools` submodule  
**Validated on this host:** none possible (C++ build requires VESC hardware knowledge).  
**On Jetson:**
1. Initialize submodules: `git submodule update --init`
2. Check `vesc` submodule `package.xml` for its deps (serial/transport_drivers)
3. `colcon build --packages-select vesc vesc_msgs vesc_driver vesc_ackermann ackermann_mux teleop_tools`
4. Test VESC connection: `ros2 topic echo /sensors/imu/raw`
**Go/no-go:** `/odom` and `/sensors/imu/raw` publish at expected rates; EKF produces `/odometry/filtered`.

---

### Phase 9: Perception — lidar (Jetson, physical lidar required)

**What:** `f1tenth_perception` (lidar front wall, wall distance), `urg_node`  
**Validated on this host:** fix cross-package test failures.  
**On Jetson:** lidar TCP connection + `/scan` topic.  
**Go/no-go:** `/scan` at ≥ 40 Hz; lidar_front_wall_node produces `/perception/front_distance`.

---

### Phase 10: Perception — ZED and YOLO (Jetson, ZED SDK required)

**What:** ZED wrapper (submodule v5.5.0 → requires ZED SDK 5.5.x), detection_3d_node (torch, ultralytics, TensorRT)  
**Note:** ZED SDK 5.5.x on the Jetson must be verified for compatibility with JetPack. ZED wrapper is at `v5.5.0` which requires ZED SDK 5.5.x (not 5.4.x as originally expected).  
**YOLO/TensorRT:** Python 3.12 on Jetson (if running Jazzy natively on Thor/AGX Orin) requires JetPack 7 wheels, not PyPI wheels. The past CUDA toolkit shadowing incident only applies to PyPI x86_64 installs; Jetson torch wheels from NVIDIA's index are self-contained.

---

## Change List Per Package (In Bring-up Order)

| # | Package | Changes | Commit label |
|---|---|---|---|
| 1 | f1tenth_messages | None | — |
| 2 | f1tenth_params | None | — |
| 3 | corridor_perception | None | — |
| 4 | f1tenth_description | None | — |
| 5 | f1tenth_localization | Fix cross-package test isolation (mock or source overlay); add T2 validation note | style: fix test isolation |
| 6 | f1tenth_costmap | None | — |
| 7 | f1tenth_navigation | Verify slam_toolbox 2.8.5 launch; confirm transform_publish_period: 0.0 honored | chore: verify slam_toolbox 2.8.5 |
| 8 | f1tenth_perception | Fix 2 cross-package test failures; install urg_node | fix: cross-package test lookup |
| 9 | mpc_controller | Fix flake8 style; investigate remaining logic test failures after osqp install | fix: style; install osqp |
| 10 | f1tenth_control | None | — |
| 11 | f1tenth_behavior | Install py_trees_ros; verify 2.5.0 API compat | dep: install py_trees_ros |
| 12 | f1tenth_logger | Install pyarrow (pip); update rosbag2 error message (mentions `ros-humble-rosbag2-storage-mcap` → should say `ros-jazzy-...`) | fix: update pip dep note |
| 13 | f1tenth_diagnostics | Install ruamel.yaml (apt); vesc_msgs available once submodule initialized | dep: install ruamel.yaml |
| 14 | f1tenth_intelligence/llm | Fix flake8/pep257 style; investigate test_intent_go_to logic failures | fix: style + investigate logic failures |
| 15 | f1tenth_bringup | Replace ROS_LOCALHOST_ONLY in conftest.py + replay script; fix flake8/pep257/copyright style | fix: ROS_LOCALHOST_ONLY + style |
| 16 | f1tenth_sim | Add COLCON_IGNORE (defer Gazebo Harmonic port) | chore: defer sim port to Gazzy Harmonic |
| 17 | f1tenth_hardware | (Jetson) initialize submodule; build vesc + ackermann_mux | jetson: initialize vesc submodule |
| 18 | f1tenth_more | None | — |

---

## Jetson / Docker Notes (Later Phase)

### Thor (JetPack 7, Ubuntu 24.04) — native Jazzy

If the car runs on AGX Thor with JetPack 7 (Ubuntu 24.04), ROS 2 Jazzy can be installed natively via apt. This is the ideal path: no container, no SHM/DDS workarounds. USB/serial passthrough and CPU pinning work as on any Linux system.

### Orin (JetPack 6, Ubuntu 22.04) — container required

JetPack 6 ships Ubuntu 22.04, which is the ROS 2 Humble base. To run Jazzy on Orin requires either:
- A Docker container with Ubuntu 24.04 + ROS 2 Jazzy (the existing `docker/` scaffolding in this repo is a starting point)
- Or compile ROS 2 Jazzy from source on 22.04 (not recommended)

In a container on Orin:
- **SHM transport:** FastDDS SHM requires that the container share the host's `/dev/shm`. Pass `--shm-size=1g` and mount `/dev/shm`. The `component_supervisor_node.py` SHM cleanup pattern works.
- **USB/serial (VESC, lidar):** pass through with `--device /dev/ttyACM0` or equivalent.
- **GPU/ZED:** ZED SDK must be installed inside the container; pass `--privileged` or specific device permissions.
- **CPU pinning (`taskset`):** `taskset` works inside a container as long as the container is not CPU-quota-limited (avoid `--cpus` Docker flag if CPU affinity matters).
- **DDS across container boundary:** if the supervisor runs outside and some nodes inside, use `--network host` or configure `ROS_DISCOVERY_SERVER` to point at the host IP.
- **Discovery Server port:** map port 11811 or use `--network host`.

---

## Decisions for Andreas

1. **Submodule init strategy.** The six submodules (vesc, ackermann_mux, teleop_tools, transport_drivers, zed_ros2_wrapper, zed-ros2-interfaces) are all empty on this host. For offline/software development you can work without them. For hardware testing you need at minimum vesc and ackermann_mux. **Decision: initialize all now, or only hardware-critical ones when needed?**  
   *Recommendation: defer hardware submodules (vesc, ackermann_mux) until Phase 8 (Jetson). Initialize teleop_tools if you want joystick testing on this host.*

2. **pyarrow and osqp: pip install with --break-system-packages or use a venv?**  
   *Recommendation: `--break-system-packages` on this dev host only; use a container with requirements.txt for the Jetson and CI.*

3. **f1tenth_sim: defer or port now?** The sim requires T3 effort (Gazebo Harmonic package rename + launch file changes). The real car path doesn't need it.  
   *Recommendation: add `COLCON_IGNORE` to `src/f1tenth_sim/` and defer.*

4. **Humble tag on the Jetson.** The analysis found no `humble-final` tag; all branches point at the same HEAD. To enable clean diff and cherry-pick between Humble (Jetson) and Jazzy (this host), Andreas should tag the Jetson's current HEAD as `humble-final` before any Jazzy-only commits land there.  
   *Recommendation: tag now, before Phase 7 changes reach the Jetson.*

5. **EKF bag replay validation (Item 1).** The `pose0_rejection_threshold: 5.0` and Q values must be re-validated against the Jazzy robot_localization. This requires the archived bags from the Jetson. Are the bags available on this host, or does replay need to happen on the Jetson?  
   *Recommendation: copy one representative bag to this host; replay through the Jazzy EKF node in isolation (no real hardware needed). The `replay_bag_through_mpc.sh` script provides the pattern.*

6. **slam_toolbox lifecycle node.** When slam_toolbox 2.8.5 is installed, does the existing `f1tenth_navigation/launch/slam.launch.py` call its own `online_async_launch.py` (which handles lifecycle), or does it launch the node directly? If direct node launch, lifecycle configure/activate must be added.  
   *Recommendation: check slam.launch.py after installing 2.8.5 — if it delegates to slam_toolbox's own launch file, no changes needed.*

7. **Jetson platform: Thor (JetPack 7) or Orin (JetPack 6)?** The Docker notes differ substantially between the two. Thor (Ubuntu 24.04) allows native Jazzy; Orin (Ubuntu 22.04) requires a container.  
   *Please confirm which Jetson model is the target.*

8. **osqp version: 0.6.x or 1.x?** The `mpc_solver.py` code was written for a specific osqp API. Check whether it uses the `osqp.OSQP()` class directly (0.6.x) or `osqp.solve()` / `osqp.Problem()` (1.x). The pip-available version is 1.1.3. If the code was written against 0.6.x, a small adaptation of the solver interface may be needed.  
   *Recommendation: install 1.1.3 and run tests; if they fail with AttributeError, check the 0.6→1.0 migration guide.*
