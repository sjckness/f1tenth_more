# Phase 5: Bringup, Discovery Server, supervisor (Thor, no hardware)

Gate:
- Previous phase: `output/phase4_support_report.md`, verdict **GO-WITH-NOTES**.
  Its pending Orin runs are known and non-blocking.
- Fix batch: `output/fix_batch_1_report.md`, verdict **GO-WITH-NOTES**. No
  blocker for this phase.

Data: `output/phase5/`. Bags and input bags are not committed.

## Verdict: **GO-WITH-NOTES**

On Thor, the production `supervisor_bringup.launch.py` brings up every
enabled component on Jazzy and keeps it running:
- 32 nodes, every node up within 7.5 s.
- The TF tree is complete, with exactly one publisher per edge.
- Supervisor restarts work and the restarted nodes rejoin discovery.
- 10 of 10 bringup/shutdown cycles are clean.
- 0 Discovery Server errors in 15 runs.
- The sim mode is in place and verified.

Four findings need attention before Phase 6:

1. **`ackermann_mux` was broken on Thor.** It died at startup with an
   undefined symbol from `diagnostic_updater`, so the `control` component
   crash-looped.
   - Cause: Phase 1's apt upgrade (4.2.6 → 4.2.7) changed that library's
     ABI after the workspace's C++ packages had been built.
   - Fixed by a clean rebuild; no source change.
   - The same breakage was in `zed_components` and `zed_debug`.
2. **Under the Discovery Server, the mission logger records nothing
   unless it runs as a super client.**
   - Started with only the environment the launch file sets, the bag held
     0 messages. With `ROS_SUPER_CLIENT=TRUE` it held 35 topics.
   - Humble's Fast DDS 2.6.11 behaves identically in a probe, so this is
     not a distro change. The question is what the Orin's environment
     provides (Decision 2).
3. **One silent discovery isolation in about 320 node starts.**
   - In cycle 5, `swept_clearance_node` ran but never appeared in the graph
     and never received `/scan` or `/tf_static`. No error was logged, and
     the supervisor cannot see it.
   - Whether Humble does the same is unknown (Decision 3).
4. **Shutdown has distro-independent rough edges.**
   - Python nodes print `KeyboardInterrupt` tracebacks because they receive
     two SIGINTs. A Humble container probe gives the same result.
   - The logger's lock file is left behind; it is reclaimed at the next
     start.
   - The Discovery Server outlives the launch, by design.

The environment script is **proposed, not applied** (Step 2). CPU pinning is
**not applied**: the target machine is undecided.

---

## Step 1: Inventory (from code)

### Supervisor

**`supervisor_bringup.launch.py`:**
1. Sets `ROS_DISCOVERY_SERVER=<address>:<port>`. The defaults come from
   `stack_params.yaml`: `127.0.0.1`, `11811`.
2. Starts `ensure_discovery_server.py` as an `ExecuteProcess` with
   `respawn=True`.
3. Starts `component_supervisor_node`.

Launch arguments: `calibration` (false), `enable_intelligence` (true),
`components_config`, `restart_timeout_sec` (10), `log_dir`,
`watchdog_period_sec` (2), `max_auto_restarts` (3),
`restart_budget_window_sec` (60), and, new in Step 6, `sim` (false).

**`component_supervisor_node`** runs each component's launch files as
`ros2 launch <pkg> <file>` in its own session (process group).

| Component | Launch files | Auto-start | Hardware |
|---|---|---|---|
| hardware | `vesc.launch.py`: VESC driver, `vesc_to_odom`, `ackermann_to_vesc`, battery precheck, static `base_link→imu` | always | **VESC serial** |
| calibrate_hardware | `vesc.launch.py calibration:=true` | never (on demand, `~/run_calibration`) | **VESC** |
| localization | `localization.launch.py`: both EKFs, `ekf_cost_observer`, `slam_pose_relay`, `description.launch.py` (robot_state_publisher, joint_state_publisher, static `base_link→laser`) | always | – |
| perception | `camera.launch.py` (ZED + static `base_link→zed2_camera_link`), `lidar.launch.py` (urg_node, gated on `use_lidar`), `detection.launch.py` (YOLO, needs torch, plus detection_3d, obstacle_projector, front_clearance) | always | **ZED, urg, torch** |
| lidar_front_wall, wall_distance, swept_clearance, obstacle_clearance | one launch file each | always | – |
| control | `ackermann_mux.launch.py` | always | – |
| navigation | `navigation.launch.py` (mpc_corr; map_server and its lifecycle manager) | always | – |
| slam | `slam.launch.py`, `costmap.launch.py` (both gated on `enable_slam`, default true) | always | – |
| behavior | `behavior_bringup.launch.py` | if `use_behavior_tree` (true) | – |
| diagnostics | `system_observer`, `diagnostics_server`, `mission_logger` | always | – |
| intelligence | `llm.launch.py` (llama-server) | if `enable_intelligence` | host binary |
| dev_tools | `foxglove_bridge.launch.py` (sets `ROS_SUPER_CLIENT=TRUE` for itself) | always | – |
| startup_sequence | steering sweep | never | via mux |

- **joy** (`joy.launch.py`) is not in any component and not in
  `stack_bringup.launch.py`. It is only ever started by hand.
- **Start order:** a single unordered loop over the components; there are no
  dependencies. The one exception is the `localization` deferral, which
  applies only when `calibration` is true (default false).
- **Health checks:** none beyond process exit. The 2 s watchdog respawns a
  launch process that exits, up to 3 times per 60 s. A node that is alive but
  broken goes unnoticed (Finding 3).
- **SHM cleanup:** `/dev/shm/*fastrtps*` files that no live process holds
  are removed at supervisor startup and shutdown. Nothing is removed on a
  component restart.
- **Single-instance locks:** `/tmp/component_supervisor.lock` and
  `/tmp/mission_logger.lock`. A lock naming a dead PID is reclaimed.

**CPU pinning in effect** (`taskset -c` prefixes, read live with
`taskset -pc`, Step 4):
- EKFs `0,1`; slam_toolbox `2`; foxglove `3`; behavior_executor `4`;
- wall_distance, lidar_front_wall, obstacle_clearance and swept_clearance
  `5`;
- `ekf_cost_observer` `2-11`; mpc_corr `10,11`;
- everything else unpinned (`0-13`).

These are the Orin defaults, unchanged; see "CPU pinning" below.

### Environment variables: where they are set

- **Repo:** `ROS_DISCOVERY_SERVER` is set by `supervisor_bringup.launch.py`
  and `stack_bringup.launch.py` (SetEnvironmentVariable).
  `ROS_SUPER_CLIENT=TRUE` is set by `foxglove_bridge.launch.py` only.
  `steering_offset_calibration_node` refuses to run without
  `ROS_DISCOVERY_SERVER`.
- **Test harnesses** unset `ROS_DISCOVERY_SERVER` and use isolated
  `ROS_DOMAIN_ID`s with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
- **Not set anywhere:** `FASTRTPS_DEFAULT_PROFILES_FILE` and
  `FASTDDS_DEFAULT_PROFILES_FILE` (no XML profile anywhere in the repo),
  `RMW_IMPLEMENTATION` (outside a `zed` alias), and `ROS_DOMAIN_ID` (so 0).
- **Thor `~/.bashrc`:**
  - `source /opt/ros/jazzy/setup.bash`, which also gives Jazzy's default
    `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`;
  - a `zed` alias (`ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
    a separate `~/zed_host_ws`).
  - It sets nothing about the Discovery Server.
- `supervisor_bringup.launch.py`'s docstring says `~/.bashrc` sets
  `ROS_DISCOVERY_SERVER`. That is the Orin, not checked here (Decision 2).

---

## Step 2: Environment (proposed, NOT applied)

- **Proposed `scripts/env/jazzy.sh`:** `output/phase5/env_proposal/jazzy.sh`.
  It does the following:
  - sources Jazzy and the workspace overlay;
  - sets `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`;
  - sets `ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}`;
  - sets `ROS_DISCOVERY_SERVER=127.0.0.1:11811` and `ROS_SUPER_CLIENT=TRUE`;
  - leaves `ROS_AUTOMATIC_DISCOVERY_RANGE` at its default;
  - provides an `f1tenth_simple_discovery <domain>` helper for isolated
    replays.
- **`~/.bashrc` diff:** `output/phase5/env_proposal/bashrc.diff`, one line:

```diff
-source /opt/ros/jazzy/setup.bash
+source ~/dev_ws/f1tenth_more/scripts/env/jazzy.sh
```

- Neither the script nor `~/.bashrc` has been created or changed.
- Every Phase 5 probe ran with the same variables the script sets (domain 85
  instead of 0).
- `ROS_SUPER_CLIENT=TRUE` in that shell also decides Finding 2 for a stack
  started from it.

---

## Step 3: Discovery Server under Fast DDS 2.14

### Versions and configuration

- **Versions:** `ros-jazzy-fastrtps` 2.14.6, `rmw-fastrtps-cpp` 8.4.4, the
  `fastdds` CLI tool, RMW in use `rmw_fastrtps_cpp`.
- **Server:** `ensure_discovery_server.py 127.0.0.1 11811` execs
  `sh fastdds discovery --server-id 0 --udp-address 127.0.0.1 --udp-port 11811`.
  - It reports "Participant Type: SERVER, Server ID 0, GUID prefix
    44.53.00.5f.45.50.52.4f.53.49.4d.41, UDPv4 127.0.0.1:11811".
  - **The socket is bound to `0.0.0.0:11811`** (`ss -lunp`), although the
    advertised locator is loopback. This matters for the two-machine setup.
- **Clients:** `ROS_DISCOVERY_SERVER=127.0.0.1:11811` environment only. No
  XML.

### Probe results (`scripts/jazzy_parity/ds_probe.sh`, `output/phase5/discovery/probe/`)

- A talker and listener talk through the server (4 messages in 6 s at
  1 Hz, after discovery).
- **A plain client sees nodes but not foreign topics.** `ros2 topic list`
  as a plain client shows only `/parameter_events` and `/rosout`. **As a
  super client it sees `/chatter`.** So the ros2 CLI needs
  `ROS_SUPER_CLIENT=TRUE` to see the full graph.
- In the full stack, a super client saw 34 nodes and 89 topics
  (`step4/graph.json`).
- A participant without `ROS_DISCOVERY_SERVER` sees nothing of the stack.
- `ROS_AUTOMATIC_DISCOVERY_RANGE` SUBNET vs LOCALHOST: identical graphs.
- **Reuse branch:** a second `ensure_discovery_server.py` finds the port
  taken and idles (no respawn loop). On SIGINT it prints a
  `KeyboardInterrupt` traceback (rc 130), which is cosmetic.
- **Cross-distro (`output/phase5/probes/ds_plain_client/`):** the same
  plain-vs-super-client probe in the Humble container (Fast DDS 2.6.11)
  gives the identical result. This behaviour did not change with the distro.
- **Lifetime:** SIGINT to the launched `sh` wrapper does not reach
  `fast-discovery-server` (sh does not forward it). At launch shutdown,
  launch escalates to SIGTERM on the wrapper and the server keeps running,
  orphaned. This is the documented design ("must outlive whatever
  component churns"). The next bringup reuses it (cycles 2–10).

### Discovery errors, Steps 4–6

**"Matching unexisting participant": 0**, and any other DISCOVERY_DATABASE,
RTPS or PARTICIPANT error: **0**. That covers 15 runs (step4 ×2, the logger
run, 10 cycles, sim_false, sim_true), counted over every launch, component
and node log, including the server's own output
(`*/discovery_errors.txt`).

---

## Step 4: Full bringup without hardware, fed by a bag

### Setup

- **Launch:** `scripts/jazzy_parity/phase5_bringup.sh step4`, which runs the
  production
  `supervisor_bringup.launch.py components_config:=<generated> enable_intelligence:=false log_dir:=<run>`
  on `ROS_DOMAIN_ID=85`.
- **What runs without hardware:**
  - The generated `components.yaml` is production with `hardware: []` and
    `perception: []`. Both keys stay registered, because the supervisor
    indexes every auto-start name; removing a key would crash it at startup.
  - The mission logger's runs directory is redirected into the run
    directory.
  - No VESC, ZED, urg, joy, torch or llama-server process runs.
  - No serial device exists on Thor.
- **Input:** `/scan`, `/odom` and `/tf_static` from `humble_obstacle_run`.
  - `/sensors/imu/raw` is not in the bag. The local EKF runs on `/odom`
    alone.
  - `/tf_static` carries only the edges whose publishers are hardware
    launch files that do not run here: `base_link→imu`,
    `base_link→zed2_camera_link` and the ZED tree, merged into one latched
    message.
  - The stack's own static edges come from the stack, so no edge gets two
    publishers.
- **Player:** the stack runs on wall time, as on the car, so the bag is
  replayed by `restamp_play.py` with `header.stamp = now`.
  - With `ros2 bag play` (the original stamps are about 11 days old), the
    local EKF put `odom→base_link` on `/tf` only 28 times in 12 s.
  - Re-stamped: 512 times in 12 s.
  - First run kept as `output/phase5/step4_recorded_stamps/`.

### Health: all 11 components that have launch files here (14 launch files), 32 nodes

Node list: `step4/component_nodes_settled.json`. robot_state_publisher has no
`__node` remap, so it is not attributed there; it runs under `localization`.

| Topic | Hz | Topic | Hz |
|---|---|---|---|
| /odom (input) | 50.2 | /scan (input) | 40.2 |
| /odometry/filtered | 48.1 | /ekf_global/odometry/filtered | 48.0 |
| /tf | 105.9 | /joint_states | 10.0 |
| /slam/pose, /slam/pose_calibrated | 1.95 | /slam/map | 0.2 |
| /costmap/boundaries, /front_clearance | 20.1 | /behavior/tree_status | 9.65 |
| /drive, /ackermann_drive | 10.0 | /perception/d_wall, /segment | 10.0 |
| /perception/swept_clearance | 20.0 | /obstacle_clearance, /perception/lidar_front_wall | ~39.7 |
| /diagnostics/system_status | 1.0 | /diagnostics/battery_status, /calibration/in_progress | 2.0 |

- `/mpc/solver_status` and `/mpc/status` are 0, as expected: no goal, so no
  solve.
- No component crashed, apart from the 3 deliberate kills below. Every
  component log has 0 tracebacks before shutdown.
- **`system_observer` (fix batch item 1):** up for the whole run in every
  Phase 5 run, `/diagnostics/system_status` at 1.0 Hz, no traceback before
  shutdown.

### TF tree

One tree: `map → odom → base_link → {chassis → wheels/hinges, laser, imu, zed2_camera_link → …}`.
- `view_frames`: `step4/frames_*.pdf`/`.gv`.
- Writers per edge are counted from DDS publication sequence numbers
  (`step4/tf.json`).

| Edge | Topic | Writers seen | Publisher (attribution) |
|---|---|---|---|
| map → odom | /tf | 1 (511 msgs / 12 s) | ekf_global_filter_node (world_frame map) |
| odom → base_link | /tf | 1 (512 msgs) | ekf_filter_node (world_frame odom) |
| chassis → {left,right}_rear_wheel, {left,right}_steering_hinge; hinge → front wheel ×2 | /tf | 1 each (111 msgs) | robot_state_publisher (joint_state_publisher's /joint_states) |
| base_link → chassis, chassis → chassis_inertia | /tf_static | 1 | robot_state_publisher (URDF fixed joints) |
| base_link → laser | /tf_static | 1 | static_baselink_to_laser (description.launch.py) |
| base_link → imu | /tf_static | 1 | bag stand-in for vesc.launch.py's static_baselink_to_imu |
| base_link → zed2_camera_link | /tf_static | 1 | bag stand-in for camera.launch.py's static_baselink_to_zed2 |
| zed2_camera_link → … (9 ZED frames) | /tf_static | 1 | bag stand-in for the ZED wrapper's robot_state_publisher |

- Publishers on `/tf`: both EKFs, robot_state_publisher, and slam_toolbox
  (two writers that send nothing; `transform_publish_period: 0`).
- Publishers on `/tf_static`: robot_state_publisher,
  static_baselink_to_laser, and the player.

### Mission logger inside the full stack

A real mission went through the BT: a 12 s hold, loaded and started via
`/mission/*`. The start preflight needs a node named `ackermann_to_vesc_node`
(the VESC converter, part of the hardware component), so a name-only stub was
used; it publishes nothing.
- Result: IDLE → LOADED → RUNNING → COMPLETE.
- The logger created `complete/<run>/` (MCAP, manifest COMPLETE, extract
  written).
- **The bag held 0 messages** (`step4/runs/`).

| Stack environment | Bag |
|---|---|
| As set by `supervisor_bringup.launch.py` (Discovery Server, plain clients) | mcap, **0 messages, 0 topics** |
| Same plus `ROS_SUPER_CLIENT=TRUE` (`logger_superclient/`) | mcap, **3,970 messages, 35 topics**, extract written, outcome COMPLETE |

- **Why:** the recorder discovers topics from the graph, and a plain
  Discovery Server client is never told about foreign topics (Step 3).
- This is not Jazzy-specific: Fast DDS 2.6.11 behaves identically in the
  probe.
- The archived Humble bags were recorded with content after the Discovery
  Server was introduced (`76804f5`, 2026-08-21). So the Orin's stack
  presumably runs with `ROS_SUPER_CLIENT` set in its environment. Not
  verified (Decision 2).
- The Phase 4 logger fix works inside the full stack once the logger can
  discover topics.

### Restart test: SIGKILL of a component's whole process group (a crash)

| Component | Detected and respawned after kill | Nodes back | After |
|---|---|---|---|
| localization (6 nodes, both EKFs) | 0.26 s | all 6 (new pgid) | /odometry/filtered 47.3 Hz, /ekf_global 47.4 Hz, /tf 104.5 Hz |
| navigation (mpc_corr, map_server, lifecycle manager) | 0.65 s | all 3 | stack rates unchanged |
| diagnostics (system_observer, diagnostics_server, mission_logger) | 1.72 s | all 3 | system_status 1.0 Hz; the logger reclaimed the dead PID's lock and came up |

The detection latency is bounded by the 2 s watchdog.

**SHM** (`restart_*/shm_*.txt`): fastrtps files 244 → 238 after the kill →
244 after the respawn. The killed participants' segments did not
accumulate. The supervisor sweeps only at its own start and shutdown:
- start: 34 found, 18 orphans removed;
- shutdown: 140 found, 119 orphans removed.

`/dev/shm` held 19 fastrtps files before the run and 18 after; those
belonged to the still-running Discovery Server.

### Shutdown

- **Launch SIGINT:** launch exit 0, about 5.6 s.
- **Stack processes left behind:** none, apart from `fast-discovery-server`,
  by design.
- **Mission-logger lock left behind:** `/tmp/mission_logger.lock`, with a
  dead PID. It is reclaimed at the next start; every cycle that followed
  started its logger normally.
- **Cause:** each node gets two SIGINTs, one from the supervisor's
  `killpg` and one forwarded by its `ros2 launch`. The second interrupts
  the `finally:` cleanup.
- **Distro independence (`probes/double_sigint/`):** the same launch
  structure gives 1 clean exit in 20 on Jazzy and 0 in 20 on Humble 3.3.21.
- **Force-kill path:** the Step 4 and logger runs ended with the
  supervisor's force-kill path. Launch's SIGTERM arrived before
  `_stop_all_components` returned. All component launches had logged their
  nodes' exit within 0.7 s of their SIGINT, so this only cut short
  processes that were already finishing.
- **Unexplained:** why the supervisor's stop loop outlasted 5 s in those
  two runs. All 10 cycles shut down gracefully.

---

## Step 5: 10 bringup/shutdown cycles (`output/phase5/cycles/`)

Each cycle has the same setup as Step 4:
1. Bringup.
2. Settle (node set unchanged for 15 s).
3. 18 s of re-stamped bag.
4. Rates check.
5. SIGINT shutdown.

The Discovery Server was started fresh in cycle 1 and reused in cycles 2–10.

| cycle | nodes | all up [s] | crashes | shutdown | shutdown [s] | fastrtps SHM before → after | Matching unexisting | DISCOVERY_DATABASE | tracebacks (all at shutdown) | other leftover procs | lock left |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 32 | 7.47 | 0 | graceful | 5.33 | 35 → 23 | 0 | 0 | 14 | 0 | logger |
| 2 | 32 | 7.33 | 0 | graceful | 3.56 | 23 → 19 | 0 | 0 | 13 | 2* | – |
| 3 | 31† | 6.99 | 0 | graceful | 3.06 | 19 → 10 | 0 | 0 | 15 | 0 | logger |
| 4 | 32 | 6.73 | 0 | graceful | 2.80 | 10 → 20 | 0 | 0 | 14 | 0 | – |
| 5 | 31‡ | 6.77 | 0 | graceful | 2.80 | 20 → 16 | 0 | 0 | 14 | 0 | logger |
| 6 | 32 | 6.72 | 0 | graceful | 5.08 | 16 → 15 | 0 | 0 | 13 | 0 | – |
| 7 | 32 | 6.98 | 0 | graceful | 1.53 | 15 → 24 | 0 | 0 | 15 | 0 | logger |
| 8 | 32 | 7.23 | 0 | graceful | 3.06 | 24 → 20 | 0 | 0 | 13 | 0 | logger |
| 9 | 32 | 6.75 | 0 | graceful | 1.79 | 20 → 20 | 0 | 0 | 16 | 0 | – |
| 10 | 32 | 6.59 | 0 | graceful | 3.81 | 20 → 14 | 0 | 0 | 13 | 0 | – |

- `fast-discovery-server` was left running after every cycle, by design. It
  is not counted under "other leftover procs".
- \* Cycle 2's two "leftovers" were my concurrent Humble-container probe
  (domain 90), matched by the harness's process pattern. Not stack
  processes.
- † One of the three anonymous `transform_listener_impl_*` nodes was not
  seen. All named nodes were present.
- ‡ **`swept_clearance_node` was isolated from discovery (Finding 3):**
  - Its process ran for the whole cycle and logged "every clearance sensor
    is stale (lidar never received …)".
  - It never appeared in the super client's graph and never got
    `/tf_static`.
  - The other `/scan` consumers in the same cycle received scans.
  - No Fast DDS error was logged. The supervisor counts it as healthy.
  - Once in about 320 node starts (10 cycles × 32).
- SHM never grew across cycles (10–35 fastrtps files, swept at each
  start and shutdown).
- Rates in every cycle: /odometry/filtered and /ekf_global 41–48 Hz,
  /tf 92–106 Hz, /behavior/tree_status 9.6–10.1 Hz, system_status 1.0 Hz.

**Startup time per component** (seconds from `ros2 launch` until its last
node appeared, 10 cycles):

| control / dev_tools | diagnostics | localization | navigation | slam | behavior | swept_clearance | obstacle_clearance | wall_distance | lidar_front_wall | **all** |
|---|---|---|---|---|---|---|---|---|---|---|
| 2.98 / 2.96 (max 3.46) | 3.71 (4.01) | 4.54 (5.49) | 4.64 (6.25) | 4.64 (5.73) | 5.39 (6.49) | 6.33 (6.99) | 6.61 (7.23) | 6.62 (7.47) | 6.72 (6.98) | **6.88 (6.59–7.47)** |

Values are median (max).

Summaries: `output/phase5/cycles_summary.{json,md}`.

---

## Step 6: Sim mode (`jazzy/sim`, `af41013`)

### What it does

`supervisor_bringup.launch.py sim:=true` (default **false**):

- **Not started:** `vesc.launch.py`, `camera.launch.py` (ZED, plus the
  hardware-only `base_link→zed2_camera_link` static TF) and
  `lidar.launch.py` (urg_node).
  - `calibrate_hardware` is left with no launch files, and
    `~/run_calibration` refuses.
  - joy is never started by the supervisor in any mode.
- **Kept exactly as on the car:**
  - robot_state_publisher, joint_state_publisher, both EKFs, ekf.yaml's
    IMU handling (the empty `frame_id` on `/sensors/imu/raw` is read as
    `base_link`), and every software component.
  - The static `base_link→imu`, through `sim_hardware_tf.launch.py`: same
    node name and arguments as `vesc.launch.py`. A unit test fails if they
    drift apart.
- **Sim time:** every component launch file runs through
  `sim_component.launch.py`.
  - `SetParameter(use_sim_time=True)` covers all nodes.
  - The launch configuration `use_sim_time:=true` covers launch files that
    pass it explicitly (`description.launch.py`, `map.launch.py`).
  - `/clock` comes from the network.
  - The supervisor itself stays on wall time, so its watchdog keeps
    running while the sim is paused.

### Verification on Thor (`output/phase5/sim/`)

**sim:=false vs Step 4** (`compare_step4_vs_sim_false.json`):
- **Process lists:** the same 45 processes, identical command lines. The
  only difference is the literal `sim:=false` on the top-level command.
- **Parameters:** identical for all nodes, except:
  - run-directory paths;
  - the supervisor's new `sim: false`;
  - slam_toolbox's `max_laser_range`. slam_toolbox declares it when the
    first scan arrives. The snapshot ran unfed; the sim_true run, which was
    fed, has it again.
- **Behaviour is unchanged for the car.**

**sim:=true** (`compare_sim_false_vs_sim_true.json`):
- **Setup:** the same no-torch components file, but `hardware` and
  `perception` (camera + lidar) left registered, so that sim mode itself
  has to skip them. The harness checks the effective registry before
  launching.
- **Stand-in simulator:** `ros2 bag play --clock 100` of the input bag.
- **Results:**
  - **No driver process ran:** no vesc_driver, urg, zed, joy,
    ackermann_to_vesc, vesc_to_odom or battery check.
  - `static_baselink_to_imu` ran with the car's arguments.
  - **Every component node has `use_sim_time: true`**, with three
    exceptions, all by design: the supervisor (wall time), launch's own
    `launch_ros_*` node, and the bag player.
  - The two EKFs report no parameters through their parameter services, in
    either mode (a probe limitation, same in Step 4). Their sim time is
    shown by their command line (`-p use_sim_time:=True`, nothing in
    ekf.yaml overrides it) and by their log, "Waiting for clock to
    start...", which appears only in sim mode.
  - The stack ran on `/clock`: /clock 101.6 Hz, /odometry/filtered and
    /ekf_global 47.1 Hz, /tf 104 Hz, /slam/pose 2 Hz, /behavior/tree_status
    9.6 Hz.
  - Shutdown clean (exit 0).

`stack_bringup.launch.py` (the single-process fallback) is not changed. It
has no sim mode.

### Two-machine setup (TPad sim → Thor stack). Documented, not configured.

There is no `jazzy-sim` branch on origin (fetched 2026-10-03). The TPad
report's network section could not be referenced.

From the code and Step 3:
- **Discovery Server address:** on Thor,
  `supervisor_bringup.launch.py sim:=true discovery_server_address:=<Thor LAN IP>`.
  The server socket binds 0.0.0.0 regardless; the address chosen is the one
  advertised.
- **Client settings:** both machines need
  `ROS_DISCOVERY_SERVER=<Thor LAN IP>:11811`, the same `ROS_DOMAIN_ID`, and
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`. CLI shells also need
  `ROS_SUPER_CLIENT=TRUE`.
- **Firewall:** UDP 11811 plus the RTPS unicast ports for the domain
  (7400 + 250·domain + …) must be open between the two machines.
- **Transport:** shared memory only works within one machine, so the
  cross-machine traffic will be UDP.
- **What the simulator must not publish,** because the stack publishes it
  in sim mode:
  - `/robot_description`, `/joint_states`;
  - `base_link→laser`, `base_link→imu`, `odom→base_link`, `map→odom`.
  The old Humble `f1tenth_sim` launched its own robot_state_publisher,
  EKF, slam and foxglove; those must not run on the TPad.
- **Camera topics:** the simulator bridges
  `/zed2/zed_node/rgb/image_rect_color`. On the car, `camera.launch.py`
  remaps that to `/camera/image_raw`. In sim mode `camera.launch.py` is
  skipped, so the simulator must publish `/camera/image_raw` itself (or
  remap).
- **YOLO** needs torch, which is not on Thor.

---

## Other findings

### `ackermann_mux` (and ZED) ABI break, fixed by a rebuild

**Symptom:** in the first smoke run, `control` crash-looped (`exit 127`,
budget exhausted):

```
ackermann_mux: symbol lookup error: … undefined symbol: _ZN18diagnostic_updater7UpdaterC1E…NodeTopicsInterfaceEEd
```

**Cause:**
- `diagnostic_updater` 4.2.7 added a parameter to `Updater`'s constructor.
  This is an ABI break inside Jazzy.
- Phase 1 upgraded it, plus 248 other `ros-jazzy-*` packages, on
  2026-10-01.
- `ackermann_mux` and `zed_components` were built on 2026-09-30.
- An `ldd -r` scan of every ELF in `install/` found exactly those three
  binaries (with `zed_debug`) and no others.

**Fix:**

```
colcon build --symlink-install --cmake-clean-first --packages-select <all 13 CMake packages>
```

- `--cmake-clean-first` is required. dpkg keeps headers' original mtimes
  (2026-05-26), so make considers the old objects up to date and the
  relink fails.
- After the rebuild, the scan is clean and `ackermann_mux` runs (Steps 4–5,
  /ackermann_drive 10 Hz).
- There is no source change, so no commit. Log:
  `output/phase5/rebuild_cmake_packages.log`.

**Also restored:** the fix batch had rebuilt four Python packages without
`--symlink-install`. They are back to the workspace's `--symlink-install`
convention.

### Other notes

- `restamp_play.py` and the other probes match the stack's process pattern
  only when run concurrently (cycle 2). Cosmetic.

---

## CPU pinning: NOT applied

- The launch files' Orin defaults (listed in Step 1) ran unchanged on Thor's
  14 cores.
- Cores 12–13 are idle by pinning. Phase 0 noted this.
- No retuning until the target machine is decided.

## What isn't covered

- **Humble comparison of Findings 3 and 4:** the discovery isolation rate,
  and the force-kill shutdown, need the same cycles on the Orin.
- **The Orin's actual shell and service environment** (Decision 2).
- **Hardware components:** VESC, ZED, urg, YOLO/torch, and
  intelligence/llama-server (not on Thor).
- **The `localization` calibration deferral** (only with `calibration:=true`).
- **`calibrate_hardware` / `~/run_calibration`.**
- **`startup_sequence`.**
- **`stack_bringup.launch.py`.**
- **The two-machine network.**
- **The env script applied as a file:** its variables were used, but the
  file was not sourced.

## Commits

```
42b416d jazzy/p5: full-stack bringup harness — Phase 5 needs the production supervisor bringup run, fed and measured, without hardware
af41013 jazzy/sim: supervisor_bringup sim:=true — the Gazebo phase needs the stack without hardware drivers, on simulated time
3978b81 jazzy/p5: Phase 5 results, env proposal and report — GO-WITH-NOTES, logger needs a super client under the Discovery Server
```

Also on this branch from this session, none pushed:
- the fix batch: `931a0eb`, `0397d6b`, `7155f79`, `e4d6110`, `7d3303c`.

## Decisions for Andreas

1. **Apply the environment script?**
   - `output/phase5/env_proposal/jazzy.sh` would become
     `scripts/env/jazzy.sh`, plus the one-line `~/.bashrc` change above.
   - Includes `ROS_SUPER_CLIENT=TRUE` for every shell.
2. **Mission logger under the Discovery Server.**
   - Please record the Orin's environment for the running stack:
     `tr '\0' '\n' < /proc/$(pgrep -f component_supervisor_node)/environ | grep ROS_`,
     plus how the stack is started (shell, systemd).
   - If the Orin has `ROS_SUPER_CLIENT=TRUE`, the env script reproduces it.
   - Independently of the shell, a one-line launch change, like
     `foxglove_bridge.launch.py` already has, would make the logger always a
     super client: `SetEnvironmentVariable('ROS_SUPER_CLIENT', 'TRUE')` in
     `mission_logger.launch.py`. That is a behaviour change; not applied.
3. **The silent discovery isolation (1 in about 320 node starts).**
   - Run the same 10 cycles on the Orin, Humble, to see whether it is
     pre-existing.
   - The supervisor has no liveness check that would catch it.
4. **Rebuild policy.** After any `ros-jazzy-*` upgrade, clean-rebuild the
   workspace's C++ packages (`--cmake-clean-first`). The Docker image for
   the Orin must build the workspace against the same apt snapshot it runs.
5. **Shutdown.**
   - The double-SIGINT tracebacks and the stale logger lock are
     pre-existing and distro-independent (shown with a probe). Left as is.
   - The force-kill path appeared in 2 of 15 runs, unexplained; worth one
     instrumented look on the Orin.
6. **Two-machine sim checklist** above, for the TPad side: no
   robot_state_publisher, EKF or `/joint_states` on the TPad, and publish
   `/camera/image_raw`. Please push the TPad sim-port report so the network
   section can be cross-checked.
