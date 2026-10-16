# jazzy-rust, Phase 1: Zenoh recon report

**Base commit:** `3dbe37fe58138764fbd1255237cb9469c3c38f89` (`3dbe37f`, tip of `jazzy`, identical
to `origin/jazzy` on 2026-10-16; last commit 2026-10-13 09:29).
**Branch / worktree:** `jazzy-rust`, created from that commit; worktree
`~/dev_ws/f1tenth_more_rust`, submodules initialised at their pinned commits.
**Date:** 2026-10-16. **Scope:** read-only recon. Nothing installed, no config or launch changes, no
Rust. The Humble/Orin side is out of scope.

The main checkout (`~/dev_ws/f1tenth_more`, on `jazzy-campaign`) was not touched. Its state before
this session started:

```
$ git log --oneline -5 jazzy
3dbe37f LOG: jazzy test parity run (corridor funnel, base_front, pan off, grey boxes)
5f19359 output: jazzy test parity report (overnight run 2026-10-12/13)
9e21beb yolo_detector: sub-threshold detections drawn grey on the annotated image
740ccae camera_pan: camera_pan_enabled (default false) as a real parameter
30fa1c7 base_front: one URDF front-bumper frame; every front distance measured from it
$ git status --short | head        # main checkout
 m src/f1tenth_external/zed-ros2-interfaces
 m src/f1tenth_external/zed_ros2_wrapper
?? docs/branches.md
?? src/husarion_gz_worlds/
```

How to read this report:
- Sections 1-5 below are the answers.
- **Appendix A** is the full topic inventory (about 175 topics, file:line for every publisher).
- **Appendix B** is the rmw_zenoh / Isaac research, with every citation.
- **MEASURED** means a number measured on our machines, either in this session or in an earlier
  report that is named. **ESTIMATED** means derived from params and code. No new traffic was
  measured in this session (see 2.4).

---

## Summary

| Question | Answer |
|---|---|
| What runs today | Fast DDS (`rmw_fastrtps_cpp`) with a Fast DDS **Discovery Server**, unicast, plus a 5 s lease profile. It is set in the bringup launch files for the stack, and in `scripts/env/jazzy.sh` for everything started by hand. |
| Heaviest cross-machine topics | (c) Isaac HIL: raw `/zed2/zed_node/depth/depth_registered` and `/camera/image_raw`, ~0.92 MB each at 15 Hz, **~28 MB/s payload, ~71 MB/s on the wire today** (one copy per remote subscriber). (b) Gazebo: compressed camera (~0.5 MB/s each), `/scan` fan-out, and `/clock` at ~10 000 msg/s on the wire. |
| Isaac Sim 5.1 + rmw_zenoh | **Not possible as installed.** The bridge's internal Jazzy libs ship Fast DDS and Cyclone only, and Isaac 5.1 is Python 3.11, so the apt rmw_zenoh (Python 3.12) cannot be sourced. NVIDIA documents Zenoh only from **Isaac Sim 6.x**. `zenoh-bridge-ros2dds` **cannot** talk to rmw_zenoh (rmw_zenoh README). |
| Top 3 risks | 1. HIL is blocked on Isaac (above). 2. `/tf_static` late-joiner bug rmw_zenoh #976: a subscriber with depth 100 (tf2's default static listener) may never get the static frames. 3. Silent loss: large messages are dropped silently by default (1 ms `wait_before_drop`), and with a missing router or a mixed RMW, nodes see nothing, again silently. |
| Next session | Install `ros-jazzy-rmw-zenoh-cpp` 0.2.11 on linus and the Thor. Run the three risk tests first (#976 tf_static, ZED lazy publishing #1046, rosbag of /tf_static). Then the `rmw_probe` matrix. Then the switch, behind `rmw:=fastdds` (default unchanged). |

---

## 1. Current RMW setup

### 1.1 Where it is set (repo at `3dbe37f`)

| What | Where (file:line) | Value / behaviour |
|---|---|---|
| `RMW_IMPLEMENTATION` | `scripts/env/jazzy.sh:45` | `rmw_fastrtps_cpp`, for hand-started tools (comment: so a changed default cannot silently make `ROS_DISCOVERY_SERVER` a no-op) |
| | `.devcontainer/docker-compose.yml:37,61,97` | `rmw_fastrtps_cpp` |
| | Thor `~/.bashrc:100` (alias `zed`, captured in `docker/orin/thor_capture/bashrc_ros_lines.txt:1`) | `rmw_fastrtps_cpp` for a hand-started ZED wrapper |
| | launch files | **not set.** The stack inherits the shell's value, or Jazzy's default (Fast DDS) |
| `ROS_DOMAIN_ID` | `scripts/env/jazzy.sh:49` | `${ROS_DOMAIN_ID:-0}`: production domain 0 |
| | `scripts/sim/sim_net.sh:15` | `42` for the two-machine Gazebo sim |
| | `.devcontainer/docker-compose.yml:36,60,96` | `0` |
| | `tools/run_scenarios.sh:7`, `src/f1tenth_logger/test/test_campaign/conftest.py:19`, `scripts/jazzy_parity/replay_localization.sh:143` | private test domains (87, 91, 77) |
| `ROS_DISCOVERY_SERVER` | `supervisor_bringup.launch.py:72-73`, `stack_bringup.launch.py:96-97` | `SetEnvironmentVariable` = `discovery_server_address:discovery_server_port`, for every process the launch starts |
| | `src/f1tenth_params/config/stack_params.yaml:2228-2233` | `127.0.0.1` / `11811` |
| | `scripts/env/jazzy.sh:57` | `${F1TENTH_DISCOVERY_SERVER:-127.0.0.1:11811}` |
| | `scripts/sim/sim_net.sh:13-16` | Thor `10.42.0.2:11811` for the two-machine sim (direct Ethernet) |
| Discovery Server process | `supervisor_bringup.launch.py:117`, `stack_bringup.launch.py:125` -> `src/f1tenth_bringup/scripts/ensure_discovery_server.py` | Idempotent: reuses a live server on the port, else execs `fastdds discovery`. `respawn=True`. It outlives the stack by design. |
| `ROS_SUPER_CLIENT` | `foxglove_bridge.launch.py:111`, `mission_logger.launch.py:90`, `viz_relays.launch.py:112`, `steering_offset_calibration.launch.py:166` | `TRUE` for those nodes only (they need the whole graph). `jazzy.sh:59-93` deliberately does not export it; the `ros2cli` helper sets it per call. |
| Fast DDS XML profile | `src/f1tenth_bringup/config/fastdds_profile.xml` | One participant profile: 5 s lease, 1 s announcement. Transports at defaults (UDP + SHM). |
| | `supervisor_bringup.launch.py:84-85`, `stack_bringup.launch.py:106-107`, `stack_params.yaml:2240-2243` | `FASTRTPS_DEFAULT_PROFILES_FILE` set for the whole launch tree |
| | `scripts/env/jazzy.sh:39` | same file for hand-started tools and the sim host |
| `ROS_LOCALHOST_ONLY` | nowhere (replaced in the Jazzy port) | – |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `jazzy.sh:96-98` leaves Jazzy's default (SUBNET); `LOCALHOST` in `jazzy.sh:109` (`f1tenth_simple_discovery`), `tools/run_scenarios.sh:8`, `conftest.py:20`, `replay_localization.sh:144` | test/replay isolation only |
| `ROS_STATIC_PEERS` | nowhere | – |
| Cyclone / Zenoh config | nowhere in our code (the ZED wrapper's own Docker files mention Cyclone) | – |
| `/dev/shm` cleanup | `scripts/kill_ros2.py:26` | removes stale Fast DDS SHM segments |
| Docker | `docker/orin/entrypoint.sh:8` and `/etc/bash.bashrc` source `scripts/env/jazzy.sh` | same env as the Thor shell. `Dockerfile.jetson-thor`, `.gpu`, `.cpu` set no RMW variables. |
| Gazebo sim (`jazzy`) | `scripts/sim/sim.sh` (linus) and `sim_stack.sh` (Thor) both source `sim_net.sh` | Discovery Server on the Thor at `10.42.0.2:11811`, domain 42. Big images are compressed on linus before crossing (`output/sim_camera_compression.md`). |
| Isaac Sim (`~/isaac_ws`, branch `f1tenth`, `4d2de1e`) | `launch.sh:55` | **hard-coded** `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |
| | `launch.sh:57` | profile `config/fastdds_udp.xml` (UDP only, no SHM) |
| | `config/ros_interface.yaml:3-4` | domain 42; the `rmw:` key there is read by nothing |

`sim_isaac.sh` exists only on `jazzy-campaign`, not on `jazzy`. That is why it is not in this table;
it follows the same Discovery Server pattern, with rendered `eth.xml.in` / `local.xml` profiles.

### 1.2 What is installed (read-only, 2026-10-16)

| | linus (amd64) | Thor (arm64) |
|---|---|---|
| `ros2 doctor --report`, RMW section | `middleware name: rmw_fastrtps_cpp` | (not run; same env script) |
| `ros-jazzy-rmw-fastrtps-cpp` | 8.4.3 | **8.4.4** |
| `ros-jazzy-fastrtps` (Fast DDS) | 2.14.6 | 2.14.6 |
| `ros-jazzy-rmw-cyclonedds-cpp` | not installed | not installed |
| `ros-jazzy-rmw-zenoh-cpp` | **not installed** (apt candidate `0.2.11-1noble.20260915`) | not installed |
| foxglove-bridge / rosbag2 / mcap / ros_gz_bridge | 3.2.6 / 0.26.10 / 0.26.10 / 1.0.22 | – |
| Shell env | `~/.zshrc:126` sources `/opt/ros/jazzy/setup.zsh` only. `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, no RMW or Discovery Server variables. | `~/.bashrc:120` sources `scripts/env/jazzy.sh` |
| Tailscale | installed (`100.89.193.76`) | installed |
| IPv6 | – | enabled (`disable_ipv6=0`, `::1` on lo), so the stock `rmw_zenohd` `tcp/[::]:7447` is fine on the Thor |

The two machines are on different apt snapshots (`rmw_fastrtps_cpp` 8.4.3 vs 8.4.4). That is
harmless for DDS. For rmw_zenoh it would not be harmless: version skews have broken the wire format
(Appendix B, section 0), so **pin one rmw_zenoh version on every host**.

### 1.3 Why the setup looks like this (history that matters for Zenoh)

These are MEASURED in earlier reports on `jazzy`:
- **Discovery Server + 5 s lease:** with the default 20 s lease, 8 of 30 bringups had endpoints that
  never matched; with 5 s, 0 of 30 (`output/fix_batch_4_report.md` B3, `fastdds_profile.xml`
  comment).
- **`/clock` over the link:** Gazebo published `/clock` at 1000/s. The Thor received it in waves of
  0-1300/s with gaps up to 16 s, which made the supervisor's watchdog restart healthy components
  (`output/sim_clock_fanout.md`).
- **Raw images over the link:** a subscriber on the Thor got **3 frames in 60 s** of raw 640x360
  RGB (~675 KB) over the Discovery Server link. The fix was to compress on linus: 21 KB JPEG at
  ~26 Hz and 20 KB PNG depth at ~25 Hz (`output/sim_camera_compression.md`, `a4488e5`).
- On `jazzy-campaign` (Isaac, `output/sim_next/decisions.md` S19): Isaac's raw images to the remote
  stack, about **55 MB/s**, drop its real-time factor from 1.00 to ~0.72.

Fast DDS's per-subscriber unicast fan-out over the Discovery Server is the common cause, and it is
exactly what a Zenoh router-to-router link changes (2.3).

---

## 2. Topic inventory

The full table is in **Appendix A**: about 175 topic names, with type, publishers and subscribers
(file:line), rate, size and QoS. All numbers are ESTIMATED from params and code, except where they
are quoted from the reports above. This section is the digest.

### 2.1 Heavy topics (ESTIMATED; payload = size × rate, delivered = payload × subscriber processes)

| topic | where | size | Hz | payload MB/s | delivered MB/s |
|---|---|---|---|---|---|
| `/zed2/zed_node/depth/depth_registered` | car (ZED) | 0.92 MB | 30 (17 seen) | 27.6 | 83 (3 readers) |
| `/camera/image_raw` | car (ZED) | 0.92 MB | 30 | 27.6 | 55 (2) |
| `/camera/image_raw` | Isaac -> Thor | 0.92 MB | 15 | 13.8 | 27.6 |
| `depth_registered` | Isaac -> Thor | 0.92 MB | 15 | 13.8 | 41.5 |
| `/camera/image_raw`, `depth_registered` (decompressed) | Gazebo, local on the Thor | 0.69 / 0.92 MB | ~26 / ~25 | 18 / 23 | 36 / 69 |
| `/camera/image_annotated` | Thor | 0.69 MB | 13.7 | 9.5 | 9.5-19 |
| `/camera/detection_masks` | Thor | 0.23 MB | 13.7 | 3.2 | 6.3 |
| `/camera/image_raw/compressed` | linus -> Thor (Gazebo) | 21.4 KB (MEASURED) | ~26 (MEASURED) | 0.56 | 0.56 |
| `depth_registered/compressedDepth` | linus -> Thor (Gazebo) | 20.1 KB (MEASURED) | ~25 (MEASURED) | 0.50 | 0.50 |
| `/scan` | all scenarios | 8.7 KB (Isaac 4.4) | 40 | 0.35 | 3.1 (~9 readers) |
| `/slam/map` | Thor | 94 KB to 2.25 MB | 0.05-0.2 | ~0.02 | ~0.1 |
| `/map` (map_server, transient_local) | Thor | 4.0 MB | once | – | 4 MB per late joiner |
| `/clock` | sim -> Thor | 12 B | 250 (Gazebo) / 60 (Isaac) | ~0 | **~10 000 / 2 400 msg/s** on the wire (~40 readers) |
| `/tf` | Thor | 0.1-0.6 KB | ~110 msg/s | 0.02 | 0.12 |
| ZED point cloud | car | 3.7 MB | 10 | 37 | 0 (lazy, nobody subscribes) |

### 2.2 What crosses machines

Only about 10 topics carry data between machines:
- `/clock`, `/scan`, `/odom`, `/sensors/imu/raw`, the depth `camera_info`;
- `/ackermann_drive` in the other direction;
- the two camera streams: compressed in (b), raw in (c);
- `/parameter_events` in bursts at startup.

No service or action crosses machines in (b) or (c), except script calls such as `/sim/reset`.

| scenario | direction | payload | today on the wire (Fast DDS, one unicast copy per remote subscriber) |
|---|---|---|---|
| (a) one machine | – | nothing crosses | – (local: delivered copies via SHM/loopback, up to ~250 MB/s on the car with ZED depth + RGB) |
| (b) stack on Thor, Gazebo on linus | linus -> Thor | ~1.5 MB/s (~12 Mbit/s) | ~5.3 MB/s (~43 Mbit/s), ~13 000 packets/s, mostly `/clock` and `/scan` fan-out |
| | Thor -> linus | ~1 KB/s (`/ackermann_drive`) | same |
| (c) Isaac HIL | Isaac -> Thor | **~28 MB/s (~223 Mbit/s)** | **~71 MB/s (~570 Mbit/s)**: past what a shared 1 GbE office LAN reliably gives |
| | Thor -> Isaac | ~1 KB/s | same |

Notes:
- **Visualisation in (b):** Foxglove reaches the Thor's `foxglove_bridge` over its own websocket
  (8765), not over the ROS link. The RMW switch does not affect it, which makes it the right default
  over Tailscale. RViz on linus would subscribe over the RMW: `/map` (4 MB once) plus `/tf`.
- **Tailscale vs cable:** the repo's (b) scripts assume the direct cable (`10.42.0.x`). The flows
  are the same over Tailscale. Over Tailscale only the compressed camera can be considered; raw
  images are out of the question with any RMW.
- **Fan-out under Zenoh (ESTIMATED, to be measured):** rmw_zenoh nodes listen on
  `tcp/localhost:0` only, so traffic between hosts goes through the routers. A message should then
  cross the link **once per host**, not once per remote subscriber. That would bring (b) down from
  ~5.3 to ~1.5 MB/s and ~13 000 to ~300 packets/s, and (c) from ~71 to ~28 MB/s. This is the main
  expected gain; benchmark item B3 verifies it.

### 2.3 QoS facts that matter for Zenoh (details in Appendix A, sections 5-6)

- **Transient-local topics:** `/tf_static`, `/robot_description`, `/slam/map`, `/map`,
  `/mission/status`, `/calibration/in_progress`, `/supervisor/health`, `/mpc/config`,
  `/perception/front_clearance/config`, `/rosout`. All of them are subject to rmw_zenoh #976 (late
  joiner, depth-dependent).
- **TransformListener users** in our code: 8 nodes (`static_frames.py`, `semantic_layer_node.py`,
  `costmap_boundary_node.py`, `detection_3d_node.py`, `lidar_front_wall_node.py`,
  `obstacle_projector_node.py`, `wall_distance_node.py`, `MPC_corr.py`). Third-party ones:
  slam_toolbox, both robot_localization EKFs, and foxglove/rviz.
- **Deadline / liveliness QoS:** not used anywhere in our code (grep of `src/` excluding
  third-party). The supervisor's watchdog is topic-freshness based. rmw_zenoh not implementing
  those events (Appendix B, A4) therefore does not hit us.
- **Reliability:** no mismatch blocks matching in the default run. Zenoh does not enforce
  reliability compatibility anyway: a best-effort publisher reaches a reliable subscriber.
- **Composition:** no stack launch uses intra-process comms. The only containers are the viz relays
  and the ZED wrapper (IPC on). The ZED wrapper publishes lazily, so rmw_zenoh #1046 (zero matched
  count for same-process subscribers) needs a test.
- **Becomes moot under Zenoh:** the Discovery Server, `fastdds_profile.xml`, `ROS_SUPER_CLIENT` on
  four nodes, the `ros2cli` super-client helper, and the `/dev/shm` Fast DDS cleanup.

### 2.4 Measurement of rates and sizes: skipped

The Gazebo sim exists only on `jazzy`, and linus has no built `jazzy` workspace with `f1tenth_sim`
(the main checkout is on `jazzy-campaign`, which removed Gazebo). Building one is well over the
5-minute budget, so `ros2 topic hz/bw` was **not** run. The only MEASURED numbers are the ones quoted
from earlier reports.

### 2.5 Side findings (outside the Zenoh scope, recorded so they are not lost)

1. **The car's ZED colour topic is possibly not remapped.**
   - ZED wrapper 5.x (the submodule is v5.4.1) renamed `~/rgb/image_rect_color` to
     `~/rgb/color/rect/image` (wrapper `CHANGELOG.rst:112`).
   - `src/f1tenth_perception/launch/camera.launch.py:88` still remaps the old name to
     `/camera/image_raw`.
   - If the car runs this wrapper version, `/camera/image_raw` would never be published there.
     The Thor's `zed` alias runs a separate `~/zed_host_ws` build. **Not verified**; check with
     `ros2 topic list` on the car.
2. `mpc_corr` subscribes to `/imu` and `/model/virtual_robot/odometry`, which nothing publishes.
3. Isaac's `publish_static_tf` defaults to true; `sim_isaac.sh` sets it false.
4. `/map` (4 MB) exceeds `foxglove_send_buffer_limit` (2.5 MB).
5. `isaac_ws` `sim/ros_bridge.py:1` still says "Humble libs".

---

## 3. Isaac Sim

### 3.1 What the bridge uses today (`~/isaac_ws`, branch `f1tenth`, read-only)

- Image `nvcr.io/nvidia/isaac-sim:5.1.0`, extension `isaacsim.ros2.bridge` 4.12.4, Python 3.11.13.
  The container has **no `/opt/ros`**.
- `launch.sh:54-67`:
  - `ROS_DISTRO=jazzy`;
  - **`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` hard-coded** (`:55`);
  - Fast DDS profile `config/fastdds_udp.xml` (`:57`);
  - passes the `ROS_DISCOVERY_SERVER` / `_SUPER_CLIENT` / `_STATIC_PEERS` /
    `_AUTOMATIC_DISCOVERY_RANGE` variables through;
  - puts `/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib` on `LD_LIBRARY_PATH`.
- rclpy comes from the internal libs (`sim/app.py:100-108`), version 7.1.5. Our ROS interface is
  plain rclpy (`sim/ros_bridge.py`), not OmniGraph.
- The internal libs ship `librmw_fastrtps_cpp` (Fast DDS 2.14.5) and `librmw_cyclonedds_cpp`
  (Cyclone 0.10.5). There is **no `librmw_zenoh_cpp`, no `libzenohc`** and no `share/` ament index.
  Verified by listing the image read-only (`docker run --rm --network none … ls`).

### 3.2 What NVIDIA documents

- **Isaac Sim 5.1.0, "ROS 2 Installation"**
  (https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html):
  - no mention of Zenoh;
  - "Isaac Sim is compatible with Python 3.11 only";
  - the internal libs include Fast DDS and Cyclone;
  - a system ROS may be sourced only "if it is built with Python 3.11".
- **Isaac Sim 6.0.1 / 6.2.0**, same page, section **"On Linux using Zenoh"**:
  - "Isaac Sim supports RMW Zenoh middleware for Linux and ROS 2 Jazzy";
  - "Isaac Sim Zenoh interfaces do not come packaged with internal libraries. Please install RMW
    Zenoh on your system and source it";
  - 6.x is Python 3.12, so the apt Jazzy can be sourced.

### 3.3 Verdict

**Isaac Sim 5.1 cannot run rmw_zenoh_cpp in our setup.** The options, best first:

| option | cost | risk |
|---|---|---|
| 1. Upgrade to **Isaac Sim 6.x** and source apt Jazzy + rmw_zenoh (the documented path) | new image; check `sim/` API changes; `launch.sh:55,62`, `app.py:100-108` change | 6.x API drift in our `sim/` code |
| 2. Stay on 5.1 and build a **Python 3.11 Jazzy workspace with rmw_zenoh 0.2.11** (NVIDIA's `IsaacSim-ros_workspaces` 5.1.0 Dockerfile plus rmw_zenoh/zenoh_cpp_vendor; needs Rust ≥1.75) | an image rebuild, undocumented by NVIDIA | moderate; version must match the Thor exactly |
| 3. Copy apt `librmw_zenoh_cpp.so` / `libzenohc.so` next to the internal libs | small | **ABI mix, not recommended** |
| 4. **Keep HIL on Fast DDS** and use Zenoh for (a) and (b) only | none | the Thor runs a different RMW in HIL than on the car |

**`zenoh-bridge-ros2dds` is not a way out.**
- The rmw_zenoh README says: "`rmw_zenoh` utilizes Zenoh differently, particularly in terms of key
  expressions. Consequently, `rmw_zenoh` cannot interoperate with `zenoh-plugin-ros2dds`."
- The bridge is also built on Cyclone; Fast DDS is not supported on its DDS side (ros2dds #492).
- It is only usable as **DDS ↔ Zenoh ↔ DDS**: one bridge next to Isaac and one on the Thor, with
  the Thor stack still on DDS. That replaces the Fast DDS unicast link with one filtered Zenoh TCP
  link, but the stack would not be on Zenoh.
- If you want that as an interim HIL transport, Appendix B (B3) has an untested config sketch for
  the workstation bridge: allow-list of `/clock`, `/scan`, `/odom`, IMU, ZED, `/tf*`,
  `/sim/ground_truth/*`, `/ackermann_drive`, `/sim/reset`, rate caps, priorities, a 30 s query
  timeout for `/sim/reset`. Isaac would then have to use Cyclone or simple discovery locally,
  because Cyclone cannot use a Fast DDS Discovery Server.

---

## 4. rmw_zenoh on Jazzy: risks and gaps

**Versions:**
- Main apt has **`ros-jazzy-rmw-zenoh-cpp` 0.2.11** (amd64 + arm64), bundling **zenoh-c 1.8.0**.
- **0.2.12** (Zenoh 1.10.1) is in ros2-testing only.
- All statements below are about 0.2.11 unless marked. Citations are in Appendix B.

| # | Topic | What rmw_zenoh does | Hits in our stack | Severity |
|---|---|---|---|---|
| 1 | **Router** | One `rmw_zenohd` per host. Multicast scouting is **off**. Without a router a node warns after `ZENOH_ROUTER_CHECK_ATTEMPTS` (default 1 check) and **carries on isolated**. The router listens on `tcp/[::]:7447`, which crashes on IPv4-only hosts. | Needs a router per host, started before the stack: a supervised process, like the Discovery Server today. Thor IPv6 is fine; the Orin (out of scope) has no IPv6. | High (silent) |
| 2 | **Peer vs client** | Nodes are peers by default, listen on `tcp/localhost:0` and get discovery via the router's gossip. Routers do not forward between peers by default. A remote laptop needs client mode or its own router (`connect/endpoints`). A client exits on its first connect failure (`exit_on_failure.client: true`). | (b): a router on linus linked to the Thor's router. (c): the same with the workstation. | Medium |
| 3 | **Shared memory** | Built in, **off** by default. **Open #978:** with SHM on, service requests and TRANSIENT_LOCAL history are **silently dropped** under CPU load. Fixed in Zenoh 1.10 (rmw_zenoh 0.2.12). Workaround on 0.2.11: `transport_optimization/enabled=false`. | (a) on the car is where SHM would help (ZED depth/RGB ~250 MB/s delivered locally). Launch is a CPU burst (~45 processes), the exact #978 case. | High if enabled |
| 4 | **QoS** | Reliability is not enforced (best effort reaches reliable). TRANSIENT_LOCAL is emulated with a publisher cache of `depth` samples. **Deadline, lifespan and liveliness events never fire**, though they are accepted (`rmw_event.cpp:198-215`). | We use none of them (grep). | Low |
| 5 | **Services / actions** | Built on queryables. The default query timeout is **10 min**, so a lost request hangs that long. Open: #783 (Nav2 lifecycle timeout), #1029 (client sometimes never discovers a reachable service; Jazzy, aarch64; maybe fixed by Zenoh 1.10). | The supervisor's `/restart_component`, `/reset_all` (reset_manager already uses timeouts), mission services, `/sim/reset`, robot_localization `set_pose`. Check every caller has its own timeout. | Medium |
| 6 | **`/tf_static` late joiners** | **Open #976:** a late RELIABLE+TRANSIENT_LOCAL subscriber with depth ≥~80 gets nothing; with **depth 100, never, not even later live messages**. tf2_ros `StaticListenerQoS` defaults to depth 100. Measured on Kilted with Zenoh 1.8.0 (the same Zenoh as Jazzy 0.2.11); not confirmed by maintainers and not reproduced on Jazzy yet. | Every TransformListener: 8 of our nodes plus slam_toolbox, EKFs, rviz/foxglove. The supervisor restarts components, so **every restart is a late join**. Ours can use `StaticListenerQoS(10)`; third-party nodes cannot easily be patched. | **Blocker if it reproduces** |
| 7 | **Nav2** | No compatibility statement. Nav2 nightly CI runs rmw_zenoh; failures were the exit-time Tokio panic, fixed. #783 is open. | Nav2 is off by default (`enable_nav2 false`); only an idle NavigateThroughPoses client. | Low now |
| 8 | **rosbag2** | Recording silently stopping (rosbag2 #2463) is fixed by rmw_zenoh #1040, which is in 0.2.11. A late-started recorder is subject to #976 for `/tf_static`. | `mission_logger` records bags and is restarted per run, so it is a late joiner. | Medium (test) |
| 9 | **Foxglove bridge** | No reports, either way. | `foxglove_bridge` on the Thor; it today needs `ROS_SUPER_CLIENT` (moot under Zenoh). | Low (test) |
| 10 | **robot_localization, slam_toolbox** | No reports. Generic risks: #976 via their TransformListeners; `set_pose` and SLAM services (#5). | `ekf_filter_node`, `ekf_global_filter_node`, `slam_toolbox` | Medium via #976 |
| 11 | **Large messages** | KEEP_LAST publishers use DROP congestion control: a message that cannot be queued within **1 ms** (50 ms for fragmented ones) is **dropped silently**. Multi-MB images over a link: #696, #672. Fix: `wait_before_drop=1000000` on nodes **and routers**, or RELIABLE+KEEP_ALL. Batch size is capped at 64 KB, so images fragment. | ZED depth/RGB on the car (local, (a)); raw Isaac images in (c). | High (silent) |
| 12 | **ZED lazy publishing** | Open #1046: in a composable container, a same-process subscriber counts as **0 matched**, so lazy publishers never publish. | The ZED wrapper runs as a component with lazy topics. | Medium (test) |
| 13 | **Env vars** | `ROS_DOMAIN_ID` works (first key-expression segment). **`ROS_LOCALHOST_ONLY`, `ROS_AUTOMATIC_DISCOVERY_RANGE` and `ROS_STATIC_PEERS` are ignored.** `ROS_DISCOVERY_SERVER` and Fast DDS profiles are irrelevant. | Our test isolation (`run_scenarios.sh`, `conftest.py`, `replay_localization.sh`, `f1tenth_simple_discovery`) relies on `LOCALHOST` + private domains. Under Zenoh the domain still isolates, but "localhost only" comes from the router config. | Medium |
| 14 | **Version skew** | Different rmw_zenoh versions have broken the wire format (#637, #733). | Thor and linus already sit on different apt snapshots. | High (pin) |
| 15 | **Humble ↔ Jazzy** | rmw_zenoh is incompatible between Humble and newer distros (README Known Issues). | The Orin/Humble side cannot join a Zenoh Jazzy graph. Out of scope, but it closes that door. | Note |

### 4.1 Mixed RMW: what happens, and a startup check (design only)

What happens if one machine forgets the switch:
- DDS and Zenoh do not interoperate at all, and there is **no error and no warning**. The other
  side's topics simply do not exist.
- In (b)/(c) that looks exactly like "the sim is not running": no `/clock`. `sim_isaac.sh` (on
  `jazzy-campaign`) and `sim_stack.sh` would wait for `/clock` and time out.
- `ros2 daemon` keeps the RMW it was started with, so a CLI can show a stale or empty graph after a
  switch (ROS docs "Working with multiple RMW implementations"; rmw_zenoh #853).
- Within one host, a node started by hand from a shell without the switch lands in the wrong world
  silently. Typical cases: a `ros2 bag record`, or the Thor's `zed` alias, which hard-codes
  `rmw_fastrtps_cpp`.

Proposed check, `rmw_check`, which fails loudly. It runs at bringup, in each sim script, and is
callable by hand:
1. **Local consistency** (before anything starts):
   - `RMW_IMPLEMENTATION` equals the requested `rmw` launch arg. If not, abort and name both values.
   - In zenoh mode, abort if `ROS_DISCOVERY_SERVER` / `FASTRTPS_DEFAULT_PROFILES_FILE` are set, and
     say they are ignored. This catches a shell that sourced the old env.
   - The ros2 daemon's RMW (`ros2 doctor --report` from a fresh daemon) must match; otherwise
     `ros2 daemon stop`.
2. **Router present** (zenoh mode):
   - A TCP connect to the configured router endpoint (`127.0.0.1:7447`, or the configured one) must
     succeed within N s, else abort: "no rmw_zenohd on <endpoint>: start it (…)".
   - Do not rely on `ZENOH_ROUTER_CHECK_ATTEMPTS`: it only warns.
3. **Peer host agrees** (two-machine modes; the step that catches the forgotten switch):
   - Out of band, so it works whatever the peer's RMW: probe the peer host for **both** a Zenoh
     router (TCP 7447) and a Fast DDS Discovery Server (UDP 11811 answering, or the `sim_*.sh`
     marker below).
   - If the peer shows the other RMW, abort with: "peer <host> is on fastdds, this host is on
     zenoh".
   - The sim scripts also write the chosen RMW into the per-run log, and both sides print it in
     their banner.
4. **Version match** (zenoh mode): every host publishes a latched `/system/rmw_info`
   (host, `rmw_zenoh_cpp` version, zenoh version, domain). The supervisor compares it with its own
   and fails on any mismatch, because skew can break the wire silently (#637, #733).
5. **Liveness of the link:** the existing `/clock` wait (30 s) becomes a check with a diagnosis
   message that lists the results of steps 2-4, instead of a plain timeout.

---

## 5. Proposal for the next session (design only)

### 5.1 Where the switch lives (default unchanged: `fastdds`)

| Layer | Switch | Notes |
|---|---|---|
| Params | `stack_params.yaml`: `rmw: fastdds` (`fastdds` / `zenoh`) plus `zenoh_router_endpoint`, `zenoh_connect_endpoints`, `zenoh_config_dir` | One source of truth, read through `get_default` like `discovery_server_address` |
| Launch | `supervisor_bringup.launch.py` / `stack_bringup.launch.py` arg `rmw:=fastdds\|zenoh` | `SetEnvironmentVariable('RMW_IMPLEMENTATION', …)` **first**, as `ROS_DISCOVERY_SERVER` is today. In fastdds mode: today's Discovery Server + profile, unchanged. In zenoh mode: no Discovery Server; an `ensure_zenoh_router.py` (same idempotent pattern as `ensure_discovery_server.py`: reuse a live router on the port, else exec `rmw_zenohd` with the scenario config); `ZENOH_SESSION_CONFIG_URI` and `ZENOH_CONFIG_OVERRIDE` from params; `ZENOH_ROUTER_CHECK_ATTEMPTS=0` for stack nodes (wait for the router); then `rmw_check`. |
| Env for hand-started tools | `scripts/env/jazzy.sh` reads `F1TENTH_RMW=fastdds\|zenoh` (default fastdds) | Zenoh branch: export `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and the Zenoh variables; unset the Fast DDS ones; stop the ros2 daemon if its RMW differs. `ros2cli` becomes plain `ros2` under Zenoh, since there is no super client. |
| Sim scripts | `scripts/sim/sim_net.sh` (Gazebo, `jazzy`) and `sim_isaac.sh` (`jazzy-campaign`): `--rmw fastdds\|zenoh` / `F1TENTH_RMW`, same value on both machines | Render the scenario's Zenoh config with this host's endpoint, as `eth.xml.in` is rendered today; the router per host is started by the script. Isaac 5.1: zenoh refused with the verdict in 3.3. |
| Docker | `docker/orin/entrypoint.sh` already sources `jazzy.sh`, so `F1TENTH_RMW` comes for free; compose `environment: F1TENTH_RMW` | Host networking already (CLAUDE.md). Add `ipc: host` only if SHM is enabled. The router runs as its own service or container per host. |
| Hard-coded spots to change | Thor `~/.bashrc:100` `zed` alias (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`); `isaac_ws` `launch.sh:55` | These would silently stay on Fast DDS. |

### 5.2 Zenoh config files: `src/f1tenth_bringup/config/zenoh/`

All are copies of the 0.2.11 defaults with minimal edits; Zenoh replaces the whole file, so no
partial configs. Endpoints are **parameters**, not literals: rendered at launch from params or
script options, or set with `ZENOH_CONFIG_OVERRIDE`.

| File | Scenario | Contents |
|---|---|---|
| `router_local.json5` | (a) one machine | `listen: ["tcp/127.0.0.1:7447"]` (nothing leaves the host, the Zenoh equivalent of LOCALHOST), `connect: []`, multicast off |
| `session_local.json5` | (a) | peer mode, `connect: ["tcp/127.0.0.1:7447"]`, `wait_before_drop=1000000`, `max_wait_before_drop_fragments=5000000`. SHM variant `session_local_shm.json5`: `shared_memory.enabled=true`, `transport_optimization.enabled=false` on 0.2.11 (#978), pool sized for ZED frames. For the benchmark first, not the default. |
| `router_tailscale.json5` | (b) Thor ↔ linus over Tailscale | Thor router: `listen: ["tcp/0.0.0.0:7447"]`, or bound to the Thor's tailscale0 address via param `zenoh_listen`. Linus router: `connect: ["tcp/@THOR_ENDPOINT@:7447"]`, where `@THOR_ENDPOINT@` = MagicDNS name (param, e.g. `thor.<tailnet>.ts.net`) or Tailscale IP. Raised drop timeouts on both routers. Optional `#iface=tailscale0`. |
| `router_lan.json5` | (b) on the direct cable / (c) HIL on the Poli LAN or own switch | Same structure, endpoints `@PEER_IP@` (the Thor `10.42.0.2` on the cable; workstation and Thor LAN IPs as params for (c)). Explicit `listen` on the LAN interface only. Raised drop timeouts. TCP buffers (`so_sndbuf`/`so_rcvbuf`) for raw images in (c). |
| `session_remote.json5` | nodes on the remote host (linus in (b), the workstation in (c)) | peer, connect to the local router (same as `session_local`) |
| `session_client.json5` | an ad-hoc laptop without its own router | `mode: "client"`, `connect: ["tcp/@ROUTER@:7447"]`, `connect.timeout_ms` raised so it waits instead of exiting |
| `README.md` | – | which file for which scenario, the version pin, the #976/#978 caveats |

### 5.3 Benchmark plan (next session)

**B0, risk tests first. Each is a go/no-go; a failure stops the migration until it is solved.**

| test | how | pass |
|---|---|---|
| #976 tf_static | robot_state_publisher first, then a node with a default `TransformListener` (depth 100), then `tf2_echo base_link zed2_left_camera_frame`. Repeat with a supervisor component restart, and across hosts. | static frames resolve within 2 s, 10 of 10 tries |
| #1046 ZED lazy | ZED wrapper in its container with a subscriber in the same container, and one outside | both receive |
| rosbag late joiner | start `ros2 bag record /tf_static /scan` after the stack | `/tf_static` is in the bag; counts keep growing over 10 min |
| services | 1000 × `std_srvs/Trigger` calls under the launch CPU burst | 0 lost, max latency |

**B1, `rmw_probe` matrix** (a small pub/sub tool written next session):
- Payloads 1 KB / 100 KB / 1 MB / 5 MB × rates 10 / 20 / 40 Hz.
- RMW: Fast DDS + Discovery Server (today) / zenoh / zenoh + SHM (local only).
- Topology: same host (Thor) / linus ↔ Thor on the cable / over Tailscale.
- RELIABLE KEEP_LAST 10 (like the ZED) and BEST_EFFORT (like `qos_profile_sensor_data`).
- 60 s per cell, 3 repetitions, 1 and 5 subscribers (to see fan-out).
- Metrics:
  - delivered rate and loss: a sequence number in the payload;
  - latency p50/p99/max: one-way on one host; across hosts, round-trip through an echo node,
    halved;
  - CPU and RSS of publisher, subscribers and `rmw_zenohd`;
  - link bytes and packets (`/proc/net/dev` or `ip -s link`): this is the per-host vs
    per-subscriber copy question from 2.2.
- The full cross product is 4 × 3 × 3 × 3 × 2 × 2 = 432 cells, about 7 h per repetition at 60 s
  each. That is too much for one night with 3 repetitions. Run each cell once (SHM only on the
  same-host topology, which drops a third of the zenoh+SHM cells), then 3 repetitions only of the
  cells that matter: 1 MB and 5 MB, cable and Tailscale. Script it, write a CSV and a summary table.

**B2, full stack in Gazebo** (needs a built `jazzy` workspace on linus with `f1tenth_sim`):
- (b) topology: linus Gazebo + router, Thor stack + router, once on fastdds and once on zenoh.
- Measure on the Thor: `/clock` regularity (`scripts/clock_probe.py`, max gap), `/scan` and
  compressed camera rates, supervisor watchdog restarts (must be 0), mission M01 outcome and
  timing, bringup time to all-components-up, idle CPU.
- Then (a): everything on the Thor (car-like, ZED-sized images from the sim) with and without SHM.
- Same mission set as the jazzy parity run, so the results compare with `output/jazzy_parity_*`.

**B3, HIL (c):** only after the Isaac decision (3.3). Measure raw-image delivery at 15 Hz and the
Isaac RTF against the current 0.72 with Fast DDS.

### 5.4 Open questions for you

1. **Isaac:** go to Isaac Sim 6.x, build a Python 3.11 Jazzy+rmw_zenoh workspace for 5.1, or keep
   HIL on Fast DDS for now (3.3)?
2. **rmw_zenoh version:** start on 0.2.11 (main apt: has #978 and possibly #1029), or wait for 0.2.12
   (Zenoh 1.10.1) to reach main? Both machines must use the same one.
3. **(b) transport:** is (b) really over Tailscale, or over the direct cable as the scripts assume
   today? If Tailscale: the MagicDNS names of the Thor and linus.
4. **(c) network:** at PoliTO, is TCP 7447 between the workstation and the Thor allowed on the Poli
   LAN, or will you use your own switch? Gigabit everywhere? (raw (c) is ~28 MB/s payload)
5. **Gazebo for benchmarks:** OK to build the `jazzy` workspace on linus (Gazebo is on `jazzy`, not
   on `jazzy-campaign`), or should the benchmarks run on `jazzy-campaign` with Isaac (Fast DDS on
   the Isaac side, so (c) cannot be compared)?
6. **"Rust" in jazzy-rust:** should `rmw_probe` be the first Rust piece (rclrs, or native `zenoh`
   crate talking the rmw_zenoh key format), or should it be C++/Python and Rust come later?
7. **SHM on the car:** is cutting the local copy cost of ZED images (a) a goal of this plan, or is
   the plan about the multi-machine links only?
8. Side finding 2.5-1 (ZED colour topic name on the car): want it checked separately?

---

# Appendix A: topic inventory

Written in this session from the code at 3dbe37f. Headings are shifted down two levels.

### ROS topic inventory for the Fast DDS -> Zenoh (rmw_zenoh_cpp) report

Repo: `/home/andreas/dev_ws/f1tenth_more_rust`, branch `jazzy-rust` = `3dbe37f` (branch `jazzy`).
Isaac side read from `/home/andreas/isaac_ws` (branch `f1tenth`, `4d2de1e`).
Date: 2026-10-16. Nothing here is MEASURED by this task. Every rate and size is ESTIMATED from
params/code unless it says "prior meas." (a number quoted from a report or comment already in
the repo, with its source).

#### 1. Method and the run this describes

- Start point: `f1tenth_bringup/launch/supervisor_bringup.launch.py` -> `component_supervisor_node`
  -> `f1tenth_bringup/config/components.yaml`. Auto-started components (`_ALWAYS_AUTO_START`,
  `component_supervisor_node.py:410`): hardware, localization, navigation, perception,
  lidar_front_wall, wall_distance, swept_clearance, obstacle_clearance, control, diagnostics,
  dev_tools, slam, reset_manager; plus behavior (use_behavior_tree) and intelligence
  (llama-server only, not a ROS node). Never auto-started: calibrate_hardware, startup_sequence.
- Defaults from `f1tenth_params/config/stack_params.yaml`: camera_source zed, localization_source
  ekf, enable_nav2 **false** (so mpc_corr + map_server, no Nav2), use_behavior_tree true,
  enable_slam true, calibration false, camera_pan_enabled **false**, use_lidar true,
  yolo_model yolo26s-seg.pt (segment -> masks published), use_mask_depth true,
  front_clearance_publish_debug_raw **true**, wall_track_enable true, enable_foxglove true,
  enable_viz_relays true, enable_mission_logger true, enable_bt_visualization true.
- Sim mode (`sim:=true`, `component_supervisor_node.py:279-311,438-489`): skips vesc.launch.py,
  camera.launch.py (ZED), lidar.launch.py (urg_node); runs `sim_hardware_tf.launch.py` (static
  base_link->imu) and `f1tenth_perception/launch/sim_camera_tf.launch.py` (decompressors, ZED
  frame RSP, camera pan nodes); every component via `sim_component.launch.py` => **every Thor
  node is on use_sim_time and subscribes `/clock`**.
- Each `create_publisher` / `create_subscription` / `message_filters.Subscriber` /
  TF broadcaster in `src/` (excluding tests) was read (227 call sites) and resolved against the
  launch-file parameters. Third-party nodes read from source where available: VESC submodule,
  ackermann_mux and ZED wrapper submodules, slam_toolbox / topic_tools / py_trees_ros in
  `~/dev_ws/jc_deps_ws/src`, robot_localization `ros_filter.cpp` (a copy in another session's
  scratchpad). urg_node, nav2_map_server, ros_gz_bridge, image_transport and ros2_controllers were
  **not** on disk; their QoS comes from upstream knowledge (marked "upstream").
- QoS shorthand: `R` reliable, `BE` best effort, `V` volatile, `TL` transient_local, `dN` depth N.
  An integer depth in Python/C++ = R/V/dN. `sensor_data` = BE/V/d5 (rclcpp SensorDataQoS(KeepLast(n))
  = BE/V/dn).
- Size conventions: Image = W*H*bytes/px (+~60 B header). LaserScan 1081 beams with intensities
  = 1081*4*2 + ~70 B = **8.7 KB**; without intensities 4.4 KB. Odometry ~0.72 KB (85 float64 +
  header + frame ids). Imu ~0.32 KB. CameraInfo ~0.4 KB. TFMessage ~0.1 KB per transform.
  AckermannDriveStamped ~60 B. Clock 12 B (CDR). Float32/Bool ~8-30 B. OccupancyGrid = W*H B.
- Crossing columns: (a) one machine; (b) stack on the Thor, Gazebo + viz on linus;
  (c) stack on the Thor, Isaac on a workstation. Codes: `loc` = stays on one host; `L>T` linus to
  Thor; `T>L` Thor to linus; `I>T` Isaac host to Thor; `T>I` Thor to Isaac host; `-` = topic does
  not exist in that scenario; `(0)` = published on one host, no subscriber on the other, so no
  traffic (Fast DDS sends only to matched readers; Zenoh only to matching subscriptions).

#### 2. Topic table

Default-run topics. Subscriber lists name the node, file:line where cheap, and the QoS if it
differs from the publisher's. "logger" = `mission_logger_node` (rosbag2 recorder, records only
while a mission is LOADED/RUNNING/HOLDING; list at `mission_logger_node.py:195-305`).
"health" = `component_supervisor_node` liveness watchdog subscriptions (BE/V/d1,
`component_supervisor_node.py:1528,1555`). "fox" = foxglove_bridge on the Thor (whitelist,
`stack_params.yaml` foxglove_topic_whitelist).

##### 2.1 Drivers, simulator inputs, command path

| # | topic | msg type | publisher(s) | subscriber(s) | rate Hz | size | size basis | QoS (pub) | heavy? | a / b / c |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | /scan | sensor_msgs/LaserScan | car: urg_node (urg_node pkg, `sensors.yaml`, publish_intensity true); gz: sim_ros_gz_bridge (`f1tenth_sim/config/ros_gz_bridge.yaml`); Isaac: `isaac_ws/sim/ros_bridge.py:79` | behavior_executor IsProximityTooClose `is_proximity_too_close.py:233` (R d10); lidar_front_wall `:287`, wall_distance `:280`, swept_clearance `:167`, obstacle_clearance `:129`, front_clearance `:639`, mpc_corr `MPC_corr.py:1837` (all BE); slam_toolbox (BE, `slam_toolbox_common.cpp:470`); health; viz throttle (lazy); logger. ~9 processes steady | 40 | 8.7 KB (Isaac 4.4 KB) | 1081 beams (sensors.xacro:103, isaac sensors.yaml) x2 arrays x4 B; Isaac has no intensities | urg: R d20 (upstream); gz bridge R d10 (upstream default); Isaac R d5 | medium (0.35 MB/s, x9 readers) | loc / L>T / I>T |
| 2 | /odom | nav_msgs/Odometry | car: vesc_to_odom_node `vesc_ackermann/src/vesc_to_odom_backup.cpp:101` (publish_tf false); gz: drive_bridge `f1tenth_sim/drive_bridge.py:90`; Isaac `ros_bridge.py:80` | ekf_filter_node (BE, `ros_filter.cpp:1283`); ekf_cost_observer (R d500); health; reset_manager (BE, `reset_manager_node.py:212`); logger | 50 | 0.72 KB | 85 float64 + header | R d10 | no | loc / L>T / I>T |
| 3 | /sensors/imu/raw | sensor_msgs/Imu | car: vesc_driver `vesc_driver.cpp:123`; gz: drive_bridge `drive_bridge.py:96`; Isaac `ros_bridge.py:83` | ekf_filter_node (BE); ekf_cost_observer; logger | 50 | 0.32 KB | 37 float64 + header | R d10 (Isaac R d5) | no | loc / L>T / I>T |
| 4 | /sensors/core | vesc_msgs/VescStateStamped | vesc_driver `vesc_driver.cpp:121` | vesc_to_odom; diagnostics_server `diagnostics_server_node.py:87` (d50); health; viz relay | 50 (20 ms timer, `vesc_driver.cpp:147`) | ~0.15 KB | ~15 float64 | R d10 | no | loc / - / - |
| 5 | /sensors/imu | vesc_msgs/VescImuStamped | vesc_driver `:122` | none | 50 | ~0.2 KB | | R d10 | no | loc / - / - |
| 6 | /sensors/servo_position_command | std_msgs/Float64 | vesc_driver `:127` | vesc_to_odom | ~10 | 8 B | | R d10 | no | loc / - / - |
| 7 | /commands/motor/speed | std_msgs/Float64 | ackermann_to_vesc `ackermann_to_vesc.cpp:116` | vesc_driver `:139` | ~10 | 8 B | | R d10 | no | loc / - / - |
| 8 | /commands/servo/position | std_msgs/Float64 | ackermann_to_vesc `:117` | vesc_driver `:143` | ~10 | 8 B | | R d10 | no | loc / - / - |
| 9 | /ackermann_drive | ackermann_msgs/AckermannDriveStamped | ackermann_mux `ackermann_mux/src/ackermann_mux.cpp:90` (remap ackermann_cmd) | car: ackermann_to_vesc; gz: drive_bridge `drive_bridge.py:87`; Isaac (command, R d1); swept_clearance steering `swept_clearance_node.py:160`; health; logger | 10 (follows the active lane; /drive is 10 Hz) | 60 B | | R d1 | no | loc / T>L / T>I |
| 10 | /drive | AckermannDriveStamped | mpc_corr `MPC_corr.py:1858` (every 0.1 s tick); twist_to_ackermann `twist_to_ackermann_node.py:33` (idle without Nav2) | ackermann_mux (SystemDefaults = R d10, `topic_handle.hpp:173`); health; viz relay; logger | 10 | 60 B | | R d10 | no | loc / loc / loc |
| 11 | /safety_stop | AckermannDriveStamped | BT Stop `stop.py:51` (relative name `safety_stop`); reset_manager `reset_manager_node.py:179` (20 Hz during a reset only) | ackermann_mux (prio 200); logger | event (10-20 while active) | 60 B | | R d10 | no | loc / loc / loc |
| 12 | /teleop | AckermannDriveStamped | startup_sequence `stack_startup_sequence.py:37` (on demand, 20 Hz) | ackermann_mux | on demand | 60 B | | R d10 | no | loc |
| 13 | /calibration_drive | AckermannDriveStamped | none in default run | ackermann_mux | - | | | | no | loc |
| 14 | /cmd_vel_nav | geometry_msgs/Twist | none (Nav2 off) | twist_to_ackermann `:31` | - | | | | no | loc |
| 15 | /clock | rosgraph_msgs/Clock | gz: sim_clock_throttle `f1tenth_sim/clock_throttle.py:121` (250 Hz); Isaac `ros_bridge.py:78` (60 Hz, every physics step, `sim/main.py:24,181`) | every use_sim_time node on the Thor (~40 processes, rcl TimeSource BE d1) + health (BE d1, sim only); linus-local sim nodes | 250 (gz) / 60 (Isaac) | 12 B | Clock CDR | gz R d1; Isaac R d10 | **rate x fan-out** (10k msg/s on the wire with DDS unicast) | - / L>T / I>T |
| 16 | /sim/clock_raw | Clock | sim_ros_gz_bridge (gz /clock, one per 4 ms physics step) | clock_throttle (BE d1, raw) | 250 | 12 B | | R d10 (upstream) | no | - / loc(L) / - |
| 17 | /sim/imu_raw | Imu | sim_ros_gz_bridge | drive_bridge (BE) | 50 | 0.32 KB | sensors.xacro:143 | R d10 | no | - / loc(L) / - |
| 18 | /sim/ground_truth | Odometry | sim_ros_gz_bridge (OdometryPublisher 50 Hz, ros2_control.xacro:110) | ground_truth_relay `ground_truth_relay.py:29` | 50 | 0.72 KB | | R d10 | no | - / loc(L) / - |
| 19 | /sim/ground_truth/car | geometry_msgs/PoseStamped | ground_truth_relay `:28` | none | 50 | ~0.1 KB | | R d10 | no | - / loc(L) / - |
| 20 | /sim/ground_truth/person, /sim/ground_truth/obstacle | PoseStamped | sim_ground_truth_bridge (PosePublisher, models/*/model.sdf update_frequency 10) | none | 10 | ~0.1 KB | | R d10 | no | - / loc(L) / - |
| 21 | /sim/robot_description | std_msgs/String | sim_robot_state_publisher (`sim_bringup.launch.py`, remap) | ros_gz_sim create, gz_ros2_control | once | ~10-30 KB | URDF text (not measured) | R TL d1 | no | - / loc(L) / - |
| 22 | /sim/tf, /sim/tf_static | tf2_msgs/TFMessage | sim_robot_state_publisher (remapped, never /tf) | none | 50 / once | ~1 KB | | R d100 / R TL | no | - / loc(L) / - |
| 23 | /sim/joint_states, /sim/dynamic_joint_states | JointState / DynamicJointState | joint_state_broadcaster (in gz controller_manager, 50 Hz `controllers.yaml`) | sim_robot_state_publisher, camera_pan_bridge (BE) | 50 | ~0.5 KB | | upstream default | no | - / loc(L) / - |
| 24 | /ackermann_steering_controller/reference | geometry_msgs/TwistStamped | drive_bridge `:85` | ackermann_steering_controller (in gz) | 10 | ~70 B | | R d10 | no | - / loc(L) / - |
| 25 | /ackermann_steering_controller/odometry (+ controller_state; advertises /tf but enable_odom_tf false) | Odometry | ackermann_steering_controller | drive_bridge `:92` (BE) | 50 | 0.72 KB | | upstream | no | - / loc(L) / - |
| 26 | /camera_pan/command | std_msgs/Float64 | camera_pan_controller `camera_pan_controller_node.py:143` **only if camera_pan_enabled** (default false: not created) | camera_pan_bridge `camera_pan_bridge.py:46` | - (off) | 8 B | | R d10 | no | - / (0) / - |
| 27 | /camera_pan_position_controller/commands | std_msgs/Float64MultiArray | camera_pan_bridge `:44` | position controller (in gz) | - (off) | | | R d10 | no | - / loc(L) / - |
| 28 | /camera_pan/joint_state | JointState | camera_pan_bridge `:50` (BE) | camera_pan_tf_node `camera_pan_tf_node.py:97` only if pan enabled (default: no subscriber) | 50 | ~0.1 KB | | BE d5 | no | - / (0) / - |

##### 2.2 Camera / perception

| # | topic | msg type | publisher(s) | subscriber(s) | rate Hz | size | size basis | QoS (pub) | heavy? | a / b / c |
|---|---|---|---|---|---|---|---|---|---|---|
| 29 | /camera/image_raw | sensor_msgs/Image | car: ZED wrapper via SetRemap in `camera.launch.py:88` (**see uncertainty U1: remap source name is stale for wrapper 5.4.1**); gz: sim_rgb_decompressor on the Thor (`sim_camera_tf.launch.py`); Isaac `ros_bridge.py:88` | yolo_detector `yolo_detector_node.py:490` (R d10); image_raw_viz_throttle (topic_tools throttle, non-lazy, `foxglove_bridge.launch.py`) | car 30; gz ~26 (prior meas.); Isaac 15 | car/Isaac 0.92 MB; gz 0.69 MB | 640x360 (pub_downscale_factor 2 of HD720) x4 B bgra8; gz rgb8 x3 B | ZED R d10 (`sl_types.hpp:102`); Isaac R d5; decompressor upstream default | **YES** 27.6 MB/s car, 18 MB/s gz, 13.8 MB/s Isaac, x2 readers | loc / loc(T) / **I>T** |
| 30 | /camera/camera_info | sensor_msgs/CameraInfo | ZED (same remap issue); gz camera bridge; Isaac `:89` | **none in the stack** | 30 / 30 / 15 | 0.4 KB | | R | no | loc / (0) / (0) |
| 31 | /zed2/zed_node/depth/depth_registered | Image 32FC1 | car: ZED `zed_camera_component_video_depth.cpp:81,280`; gz: sim_depth_decompressor (Thor); Isaac `ros_bridge.py:86` | detection_3d message_filters `detection_3d_node.py:256` (R d10); front_clearance `front_clearance_node.py:647` (R d10); swept_clearance `:174` (BE) | car 30 (params; ~17 measured per stack_params comments); gz ~25 (prior meas.); Isaac 15 | 0.92 MB | 640x360x4 B | ZED R d10; Isaac R d5 | **YES** 27.6 / 23 / 13.8 MB/s, x3 readers | loc / loc(T) / **I>T** |
| 32 | /zed2/zed_node/depth/camera_info | CameraInfo | ZED; gz camera bridge (`ros_gz_bridge_camera.yaml`); Isaac `:87` | detection_3d `:251` (R d10); swept_clearance `:172` (BE) | 30 / 30 / 15 | 0.4 KB | | R | no | loc / L>T / I>T |
| 33 | /sim/camera/image_raw | Image rgb8 | sim_ros_gz_camera_bridge | sim_rgb_compressor | ~30 (gz measured ~36, prior meas.) | 0.69 MB | 640x360x3 (sensors.xacro:254-256) | R d10 | **YES** 20.7 MB/s, linus-local | - / loc(L) / - |
| 34 | /sim/camera/depth_raw | Image 32FC1 | sim_ros_gz_camera_bridge | sim_depth_compressor | ~30 | 0.92 MB | 640x360x4 | R d10 | **YES** 27.6 MB/s, linus-local | - / loc(L) / - |
| 35 | /camera/image_raw/compressed | sensor_msgs/CompressedImage (JPEG) | sim_rgb_compressor (linus, `sim_bringup.launch.py`) | sim_rgb_decompressor (Thor) | ~26 (prior meas.) | 21.4 KB (prior meas., `output/sim_camera_compression.md`) | ~31x JPEG | upstream default | medium 0.56 MB/s | - / **L>T** / - |
| 36 | /zed2/zed_node/depth/depth_registered/compressedDepth | CompressedImage (PNG) | sim_depth_compressor (linus) | sim_depth_decompressor (Thor) | ~25 (prior meas.) | 20.1 KB (prior meas.) | ~45x | upstream default | medium 0.50 MB/s | - / **L>T** / - |
| 37 | ZED lazy topics not subscribed: /zed2/zed_node/rgb/color/rect/image (+ /compressed, /compressedDepth, /theora, /zstd transports and camera_info), /zed2/zed_node/point_cloud/cloud_registered, depth transports | Image / PointCloud2 | ZED wrapper (car) | none (0 traffic) | (30; cloud 10) | rgb 0.92 MB; cloud 3.7 MB | cloud 640x360x16 B XYZRGB | R d10 | would be YES if subscribed (cloud 37 MB/s) | loc / - / - |
| 38 | Isaac extras not subscribed: /zed2/zed_node/left/image_rect_color, /right/image_rect_color (+ camera_info), /sim/ground_truth/{odom, person_pose, front_range, objects (TL), start_pose (TL)}, confidence/semantic (off) | Image etc. | `ros_bridge.py:81-115` | none on the Thor | 15 / 50 / 20 / 1 | images 0.92 MB | | R d5 | YES if someone subscribes (each image 13.8 MB/s) | - / - / (0) |
| 39 | /camera/detections | vision_msgs/Detection2DArray | yolo_detector `:480` | detection_3d (msg filter + counter `:298`); front_clearance `:632`; BT detected_classes_bridge `detected_classes_bridge.py:40`; logger | ~13.7 (prior meas., stack_params viz_rate_image_annotated) | 0.5-3 KB | ~300 B per detection | R d10 | no | loc / loc / loc |
| 40 | /camera/image_annotated | Image bgr8 | yolo_detector `:482` | image_annotated_viz_throttle (non-lazy); viz relay throttle (lazy); logger | ~13.7 | 0.69 MB | 640x360x3 | R d10 | **YES** 9.5 MB/s | loc / loc / loc |
| 41 | /camera/detection_masks | Image mono8 | yolo_detector `:488` (segment task) | detection_3d (use_mask_depth true) `:265`; front_clearance `:634`; logger | ~13.7 | 0.23 MB (0 B when no detection) | 640x360x1 | R d10 | **YES** 3.2 MB/s | loc / loc / loc |
| 42 | /camera/detections_3d | vision_msgs/Detection3DArray | detection_3d `:246` | obstacle_projector `:193`; semantic_layer `:272`; BT IsObstacleDetected `is_obstacle_detected.py:68`; logger | ~13.7 | 0.5-3 KB | | R d10 | no | loc / loc / loc |
| 43 | /camera/detection_markers | visualization_msgs/MarkerArray | detection_3d `:248` | logger | ~13.7 | 1-5 KB | cube + text per detection | R d10 | no | loc / loc / loc |
| 44 | /perception/obstacles_2d | f1tenth_messages/Obstacle2DArray | obstacle_projector `obstacle_projector_node.py:190` | mpc_corr `MPC_corr.py:1776`; front_clearance `:636`; viz relay; logger | ~13.7 | <1 KB | | R d10 | no | loc / loc / loc |
| 45 | /perception/front_distance | std_msgs/Float32 | front_clearance `:596` (per depth frame) | mpc_corr `:1795`; viz relay; logger | 17-30 | 8 B | | R d10 | no | loc |
| 46 | /perception/front_wall | std_msgs/Bool | front_clearance `:598` | none | 17-30 | 8 B | | R d10 | no | loc |
| 47 | /perception/front_clearance | Float32 | front_clearance `:600` | viz relay; logger | 17-30 | 8 B | | R d10 | no | loc |
| 48 | /perception/front_blocked | Bool | front_clearance `:602` | none | 17-30 | 8 B | | R d10 | no | loc |
| 49 | /perception/front_wall_confirmed | Float32 | front_clearance `:604` | BT CheckStopCondition (wall_stop_require_lidar true, `behavior_executor_node.py:534`); logger | 17-30 | 8 B | | R d10 | no | loc |
| 50 | /perception/debug/front_distance_raw | Float32 | front_clearance `:615` (publish_debug_raw true) | logger | 17-30 | 8 B | | R d10 | no | loc |
| 51 | /perception/debug/bg_pixel_count | Float32 | front_clearance `:617` | logger | 17-30 | 8 B | | R d10 | no | loc |
| 52 | /perception/debug/front_clearance | std_msgs/String | front_clearance `:619` | logger; test-campaign logger | 17-30 | ~0.5 KB | JSON | R d10 | no | loc |
| 53 | /perception/debug/front_roi_depth | Image 32FC1 | front_clearance `:625` | logger (missions only) | 17-30 | 50 KB | 180x70x4 (roi half 90x35 px); code comment says ~63 KB, ~1 MB/s at 17 Hz | R d10 | medium 0.9-1.5 MB/s (0 when not recorded) | loc |
| 54 | /perception/debug/front_roi_class | Image mono8 | front_clearance `:627` | logger | 17-30 | 12.6 KB | 180x70x1 | R d10 | no | loc |
| 55 | /perception/front_clearance/config | String | front_clearance `:654` | logger | once | ~2 KB | JSON | R TL d1 | no | loc |
| 56 | /perception/lidar_front_wall | f1tenth_messages/WallLineFit | lidar_front_wall `lidar_front_wall_node.py:277` | health; logger | 40 (per scan) | <0.2 KB | | R d10 | no | loc |
| 57 | /perception/lidar_front_wall_virtual | WallEstimate | lidar_front_wall `:278` | viz relay | 40 | <0.2 KB | | R d10 | no | loc |
| 58 | /perception/d_wall | WallDistance | wall_distance `wall_distance_node.py:261` | viz relay | 10 | <0.2 KB | | R d10 | no | loc |
| 59 | /perception/d_wall/segment | WallSegment | wall_distance `:262` | health | 10 | <0.2 KB | | R d10 | no | loc |
| 60 | /perception/d_wall/psi_correction | Float32 | wall_distance `:267` | mpc_corr `:1813`; logger | 10 | 8 B | | R d10 | no | loc |
| 61 | /perception/d_wall/gate_margin | Float32 | wall_distance `:269` | none | 10 | 8 B | | R d10 | no | loc |
| 62 | /perception/d_wall/swept_arc | Float32 | wall_distance `:271` | none | 10 | 8 B | | R d10 | no | loc |
| 63 | /perception/swept_clearance | Float32 | swept_clearance `:153` | none | 20 | 8 B | | R d10 | no | loc |
| 64 | /perception/swept_clearance/lidar | Float32 | swept_clearance `:154` | health | 20 | 8 B | | R d10 | no | loc |
| 65 | /perception/swept_clearance/camera | Float32 | swept_clearance `:155` | none | 20 | 8 B | | R d10 | no | loc |
| 66 | /perception/swept_clearance/steering | Float32 | swept_clearance `:156` | none | 20 | 8 B | | R d10 | no | loc |
| 67 | /obstacle_clearance | Float32 | obstacle_clearance `obstacle_clearance_node.py:123` | health; test-campaign logger | 40 | 8 B | | R d10 | no | loc |
| 68 | /safety/event | String | obstacle_clearance `:124`; behavior_executor `behavior_executor_node.py:748` | test-campaign logger; fox | event | <0.5 KB | | R d10 | no | loc |

##### 2.3 Localization, TF, description

| # | topic | msg type | publisher(s) | subscriber(s) | rate Hz | size | size basis | QoS (pub) | heavy? | a / b / c |
|---|---|---|---|---|---|---|---|---|---|---|
| 69 | /odometry/filtered | Odometry | ekf_filter_node (robot_localization, `ros_filter.cpp:2112`; `ekf.yaml` frequency 50) | ekf_global (BE); mpc_corr (BE d10, `MPC_corr.py:1702`); lidar_front_wall (BE); wall_distance (BE); BT CheckStopCondition (R d10); reset_manager (BE); ekf_cost_observer (R d500); health; logger (BE override) | 50 | 0.72 KB | | R d10 | no | loc |
| 70 | /ekf_global/odometry/filtered | Odometry | ekf_global_filter_node (ns ekf_global) | costmap_boundary (BE `costmap_boundary_node.py:485`); semantic_layer (BE `:270`); BT CheckStopCondition; reset_manager; ekf_cost_observer; health; viz relay; logger | 50 | 0.72 KB | | R d10 | no | loc |
| 71 | /slam/pose | geometry_msgs/PoseWithCovarianceStamped | slam_toolbox (`slam_toolbox_common.cpp:441`, remap /pose) | slam_pose_relay `slam_pose_relay_node.py:60`; reset_manager; viz relay; logger | <=2 (scan-match only, minimum_time_interval 0.5) | 0.34 KB | | R d10 | no | loc |
| 72 | /slam/pose_calibrated | PoseWithCovarianceStamped | slam_pose_relay `:62` | ekf_global pose0 (BE); ekf_cost_observer; viz relay; logger | <=2 | 0.34 KB | | R d10 | no | loc |
| 73 | /set_pose, /ekf_global/set_pose | PoseWithCovarianceStamped | none (reset_manager uses the service of the same name) | robot_localization (`ros_filter.cpp:1150`, R d1) | - | | | | no | loc |
| 74 | /tf | tf2_msgs/TFMessage | ekf_filter_node (odom->base_link, 50 Hz); ekf_global (map->odom, 50 Hz); robot_state_publisher (6 movable joints at /joint_states rate, 10 Hz); camera_pan_tf_node only if pan enabled | TransformListeners: mpc_corr `MPC_corr.py:1684`, detection_3d `:243`, obstacle_projector `:188`, costmap_boundary `:481`, semantic_layer `:255`, slam_toolbox; fox; logger (deep queue) | ~110 msgs/s | 0.1 KB (ekf), ~0.6 KB (rsp) | 1 or 6 transforms | R d100 (tf2_ros) | rate only (~17 KB/s, 110 msg/s, x7 readers) | loc / loc(T) / loc(T) |
| 75 | /tf_static | TFMessage | robot_state_publisher; static_baselink_to_imu (`vesc.launch.py` / `sim_hardware_tf.launch.py`); static_baselink_to_laser (`description.launch.py`); car: static_baselink_to_zed2 (`camera.launch.py`), ZED wrapper RSP; sim: zed2_state_publisher + camera_pan_tf_node (`camera_pan_tf_node.py:78,88`) | TF listeners; StaticFrames subscribers (TL d100): behavior `static_frames.py:40`, front_clearance `:643`, lidar_front_wall `:279`, wall_distance `:274`, swept_clearance `:158`, obstacle_clearance `:125`; fox; logger | latched | ~2-5 KB total | | R TL d1 (static broadcaster) | no | loc / loc(T) / loc(T)* |
| 76 | /robot_description | String | robot_state_publisher (`description.launch.py`) | joint_state_publisher; fox | latched | ~10-30 KB | URDF (not measured) | R TL d1 | no | loc |
| 77 | /zed2/zed2_description | String | ZED RSP (car) / zed2_state_publisher (sim, `sim_camera_tf.launch.py`) | none | latched | ~10 KB | | R TL d1 | no | loc |
| 78 | /joint_states | sensor_msgs/JointState | joint_state_publisher (10 Hz default, kept in sim) | robot_state_publisher; mpc_corr `MPC_corr.py:1377`; health; fox; logger | 10 | 0.35 KB | 6 joints | R d10 | no | loc |

(*) Isaac publishes /tf_static itself unless `publish_static_tf` is set false (`isaac_ws/config/ros_interface.yaml`, default **true**, comment says it must be false with the stack). If left true it is a second /tf_static publisher on the Thor's graph (I>T, latched).

##### 2.4 Control (mpc_corr), behavior, mission

| # | topic | msg type | publisher(s) | subscriber(s) | rate Hz | size | QoS | heavy? | a / b / c |
|---|---|---|---|---|---|---|---|---|---|
| 79 | /mpc/pass_sides | String | mpc_corr `MPC_corr.py:631` | logger | event | <0.5 KB | R d10 | no | loc |
| 80 | /mpc/blocked | String | mpc_corr `:640` | BT CheckStopCondition; logger; test logger | event | <0.5 KB | R d10 | no | loc |
| 81 | /mpc/object_status | ObjectApproachStatus | mpc_corr `:1739` | BT; logger | 10 (object moves) | <0.2 KB | R d10 | no | loc |
| 82 | /mpc/drive_clamp | DriveClamp | mpc_corr `:1867` | logger | 10 | <0.1 KB | R d10 | no | loc |
| 83 | /mpc/min_obstacle_distance | Float32 | mpc_corr `:1869` | BT CheckStopCondition | 10 | 8 B | R d10 | no | loc |
| 84 | /mpc/min_obstacle_distance_forward | Float32 | mpc_corr `:1896` | BT CheckStopCondition | 10 | 8 B | R d10 | no | loc |
| 85 | /mpc/predicted_min_clearance | Float32 | mpc_corr `:1904` | none | 10 | 8 B | R d10 | no | loc |
| 86 | /mpc/solver_status | MpcSolverStatus | mpc_corr `:1917` | logger | 10 | ~0.5-1 KB (4 float32[N] arrays) | R d10 | no | loc |
| 87 | /mpc/input_status | diagnostic_msgs/DiagnosticStatus | mpc_corr `:1923` | health | 2 | <0.5 KB | R d10 | no | loc |
| 88 | /mpc/status | String | mpc_corr `:1936` | logger; test logger | 10 | <0.5 KB | R d10 | no | loc |
| 89 | /corridor | String | mpc_corr `:1937` | logger; test logger | 10 | ~1 KB JSON | R d10 | no | loc |
| 90 | /mpc/goal_reached | Bool | mpc_corr `:1940` | BT CheckStopCondition | event | 8 B | R d10 | no | loc |
| 91 | /mpc/corridor_markers | MarkerArray | mpc_corr `:1970` | costmap_boundary `:502`; viz relay; logger | 10 | ~3-5 KB | R d10 | no (0.04 MB/s) | loc |
| 92 | /mpc/wall_track | WallTrack | mpc_corr `:1983` | wall_distance `:287` | 10 | <0.1 KB | R d10 | no | loc |
| 93 | /mpc/config | String | mpc_corr `:3601` | logger | latched | ~3 KB | R TL d1 | no | loc |
| 94 | /mpc/goal_distance | Float32 | BT PublishMoveGoal `publish_move_goal.py:102` | mpc_corr `:1716`; BT HasMpcGoal `has_mpc_goal.py:46`; logger | event | 8 B | R d10 | no | loc |
| 95 | /mpc/goal_pose | PoseStamped | BT `publish_move_goal.py:104` | mpc_corr `:1723`; logger | event | 0.1 KB | R d10 | no | loc |
| 96 | /mpc/goal_turn | TurnGoal | BT `publish_move_goal.py:106` | mpc_corr `:1753`; logger | event | <0.1 KB | R d10 | no | loc |
| 97 | /mpc/goal_drive | DriveCommand | BT `publish_move_goal.py:108` | mpc_corr `:1762`; logger | event | <0.1 KB | R d10 | no | loc |
| 98 | /mpc/goal_object | ObjectGoal | BT GoToObject `go_to_object.py:126` | mpc_corr `:1733`; logger | event / tracks rate | <0.2 KB | R d10 | no | loc |
| 99 | /mpc/goal_object_end | String | BT `go_to_object.py:127`, `check_stop_condition.py:299` | mpc_corr `:1743`; logger | event | <0.1 KB | R d10 | no | loc |
| 100 | /mpc/hold | Bool | BT `advance_move.py:66`, `handle_object_action.py:90`, `check_stop_condition.py:296`, `go_to_object.py:128`; mission loader `loader.py:213` | mpc_corr `:1769`; logger | event | 8 B | R d10 | no | loc |
| 101 | /imu | Imu | **no publisher anywhere** (U3) | mpc_corr `:1840`; logger | - | | | no | loc |
| 102 | /model/virtual_robot/odometry | Odometry | **no publisher** (legacy sim name) | mpc_corr `:1709` (BE) | - | | | no | loc |
| 103 | /test_campaign/logger_status | String | test-campaign logger (on demand) | mpc_corr `:1667` | 0.1 | small | R d10 | no | loc |
| 104 | /behavior/tree_status | BehaviorTreeStatus | behavior_executor `behavior_executor_node.py:741` | health; logger | 10 (BT tick 100 ms) | ~0.3 KB | R d10 | no | loc |
| 105 | /bt/tree_visualization | CompressedImage (PNG) | behavior_executor `:764` (`make_tree_visualizer` min_interval 0.4 s) | viz relay (lazy) | 2.5 | ~50 KB (guess, graphviz PNG) | R d10 | medium 0.125 MB/s | loc |
| 106 | /mission/status | MissionStatus | mission loader `loader.py:214` | BT IsEmergencyStopTriggered `is_emergency_stop_triggered.py:53` (TL); mission_logger `mission_logger_node.py:691` (TL); fox | event | <0.5 KB | R TL d1 | no | loc |
| 107 | /mission/move_outcome | MoveOutcome | loader `:218` | logger; fox | event | <0.5 KB | R d10 | no | loc |
| 108 | /test/mission_event | String | loader `:224` | test-campaign logger | event | small | R d10 | no | loc |
| 109 | /mission/load_path | String | none (manual) | loader `:245` | - | | R d10 | no | loc |
| 110 | /goal_pose | PoseStamped | Foxglove client publish (client whitelist) | BT HasGoalPose `has_goal_pose.py:41` | manual | 0.1 KB | R d10 | no | loc (or T<-viz client) |
| 111 | /test/plan_result | String | llm_planner_node `llm_planner_node.py:883` (not auto-launched) | test-campaign logger | on demand | ~2 KB | R d10 | no | loc |

##### 2.5 SLAM, map, costmap

| # | topic | msg type | publisher(s) | subscriber(s) | rate Hz | size | size basis | QoS | heavy? | a / b / c |
|---|---|---|---|---|---|---|---|---|---|---|
| 112 | /slam/map | nav_msgs/OccupancyGrid | slam_toolbox `slam_toolbox_common.cpp:443` (remap /map) | costmap_boundary `:483` (R TL d1); costmap_renderer `:72` (R TL d1); health (BE d1, raw); fox; logger | every map_update_interval 5 s, stretches to ~18 s as the session grows (components.yaml comment) | 94 KB on the car (prior meas. 2026-09-22, stack_params) ; 1500x1500 = 2.25 MB worst case | W*H B at 0.05 m | R TL d1 | YES by size (bursty, latched re-send to every late joiner) | loc |
| 113 | /slam/map_metadata | MapMetaData | slam_toolbox `:445` | fox | with map | <0.1 KB | | R TL d1 | no | loc |
| 114 | /slam_toolbox/graph_visualization, /slam_toolbox/scan_visualization, interactive-marker topics (enable_interactive_mode true) | MarkerArray / LaserScan | slam_toolbox `loop_closure_assistant.cpp:62,76` | none | with map update | grows with pose graph | | R d1 / R d10 | no | loc |
| 115 | /map | OccupancyGrid | map_server (nav2_map_server, `map.launch.py`, map `maps/square_100m.yaml`) | **none in the stack**; fox whitelist; RViz default.rviz on linus | once (latched) | **4.0 MB** | square_100m.pgm 2000x2000 at 0.05 m | R TL d1 (upstream) | YES one-shot 4 MB to every late joiner | loc / (T>L if RViz on linus) / loc |
| 116 | /costmap/boundaries | BoundaryConstraintArray | costmap_boundary `costmap_boundary_node.py:487` | mpc_corr `:1783`; logger (BE override) | 20 | ~0.3-1 KB | 24 B per constraint | R d10 | no | loc |
| 117 | /costmap/front_clearance | Float32 | costmap_boundary `:489` | lidar_front_wall seed `:283`; health; viz relay; logger | 20 | 8 B | | R d10 | no | loc |
| 118 | /costmap/safe_corridor_report | String | costmap_boundary `:500` | none | <=20 | ~1 KB | | R d10 | no | loc |
| 119 | /costmap/visualization | Image rgb8 | costmap_renderer `costmap_renderer_node.py:76` | viz relay (lazy) | 2 | ~0.28 MB | 3 B x map cells (94 KB map) | R d10 | medium 0.56 MB/s | loc |
| 120 | /costmap/semantic_markers | MarkerArray | semantic_layer `semantic_layer_node.py:274` | costmap_renderer `:74`; fox; logger | ~13.7 (per detections_3d) | ~1-3 KB | disk + label per track | R d10 | no | loc |
| 121 | /costmap/semantic_tracks | Detection3DArray | semantic_layer `:275` | BT GoToObject `go_to_object.py:124`; logger; test logger | ~13.7 | ~1 KB | | R d10 | no | loc |

##### 2.6 Diagnostics, supervisor, viz, per-node

| # | topic | msg type | publisher(s) | subscriber(s) | rate Hz | size | QoS | heavy? | a / b / c |
|---|---|---|---|---|---|---|---|---|---|
| 122 | /diagnostics/system_status | SystemStatus | system_observer `system_observer_node.py:43` | diagnostics_server `:92`; BT IsSystemOverheated `is_system_overheated.py:104`; health; viz relay; logger | 1 | ~0.2 KB | R d10 | no | loc |
| 123 | /diagnostics/battery_status | BatteryStatus | diagnostics_server `diagnostics_server_node.py:85` | BT IsBatteryLow `is_battery_low.py:37`; health | 2 | <0.1 KB | R d10 | no | loc |
| 124 | /calibration/in_progress | Bool | diagnostics_server `:109` | supervisor `component_supervisor_node.py:1242` (R TL d1); fox | on change | 8 B | R TL d1 | no | loc |
| 125 | /diagnostics | DiagnosticArray | supervisor `:1572`; ekf_cost_observer `ekf_cost_observer_node.py:313`; both robot_localization EKFs; ackermann_mux; car: urg_node, ZED | ekf_cost_observer `:317` (R d100); logger | ~1 per publisher (~6-8 msg/s) | 0.5-2 KB | R d10 | no | loc |
| 126 | /supervisor/health | DiagnosticArray | supervisor `:1568` | scripts/stackctl.py | 1 + on change | ~2 KB | R TL d1 | no | loc |
| 127 | /viz/scan, /viz/sensors/core, /viz/bt/tree_visualization, /viz/ekf_global/odometry/filtered, /viz/slam/pose, /viz/slam/pose_calibrated, /viz/perception/d_wall, /viz/perception/front_clearance, /viz/perception/front_distance, /viz/perception/lidar_front_wall_virtual, /viz/perception/obstacles_2d, /viz/costmap/front_clearance, /viz/drive, /viz/mpc/corridor_markers, /viz/diagnostics/system_status (15 topics) | same as source | topic_tools ThrottleNode x15 in `viz_relay_container` (`viz_relays.launch.py`, `viz_relays.yaml`, lazy) | foxglove_bridge only | 5.5 cap (scan 5.5) | as source | copies source QoS | no | loc |
| 128 | /viz_internal/camera/image_annotated, /viz_internal/costmap/visualization (2 topics) | Image (raw) | ThrottleNode (lazy) | viz_jpeg_node | 3.3 / 2 | 0.69 / 0.28 MB | copies source | medium when a client watches | loc |
| 129 | /viz/camera/image_annotated/compressed, /viz/costmap/visualization/compressed (2 topics) | CompressedImage JPEG q75 | viz_jpeg_node `viz_jpeg_node.py:82` | foxglove_bridge | 3.3 / 2 | ~30-50 KB (guess) | R V d1 | no | loc |
| 130 | /camera/image_raw/viz, /camera/image_annotated/viz (2 topics) | Image (raw) | topic_tools throttle x2 (`foxglove_bridge.launch.py`, non-lazy, foxglove_image_throttle_hz 5) | none (not whitelisted) | 5 | 0.92 / 0.69 MB | copies source | publish cost only, no readers | loc |
| 131 | /rosout | rcl_interfaces/Log | every node (~45 processes) | foxglove_bridge | bursty | ~0.2 KB | R TL d1000 (Jazzy rosout QoS, lifespan 10 s) | no | loc (+ every remote node's own) |
| 132 | /parameter_events | rcl_interfaces/ParameterEvent | every node (one event per declare_parameter at startup) | every rclcpp node (TimeSource), fox | bursty at startup | 0.1-2 KB | R V d1000 | no, but all-to-all at startup | crosses both ways in b/c |

Not running in the default run (listed so the report can say they were checked): Nav2 stack
(`enable_nav2` false: /plan, /cmd_vel_nav, costmaps, NavigateThroughPoses action server), joy
(`joy.launch.py` in no component: /joy, /teleop), calibration nodes (calibration false:
sensor_covariance / gyro_bias / steering_offset publish /commands/motor/speed,
/commands/servo/position, /calibration_drive, /diagnostics), front_depth_monitor_node,
raw_odom_map_tf_node (localization_source ekf), andre_mpc_node, campaign_scenario,
test_campaign logger + trigger, llm_planner_node, startup_sequence (/teleop), the optional
sim-host foxglove_bridge (`sim_bringup.launch.py foxglove:=true`).

**Topic count: 132 rows; counting each name inside the grouped rows (20, 22, 23, 25, 37, 38,
73, 114, 127-130) separately, about 175 distinct topic names. Only 10 carry data across machines
in (b) or (c): /clock, /scan, /odom, /sensors/imu/raw, /zed2/zed_node/depth/camera_info,
/ackermann_drive, plus the two compressed camera topics in (b) or the two raw image topics in
(c); /parameter_events crosses in startup bursts. Everything else stays on one host.**

#### 3. Heavy topics

Payload = one copy (size x rate). Readers = subscriber processes (each gets its own copy over
DDS unicast across hosts, and over Fast DDS SHM / Zenoh loopback locally).

| topic | where | size | rate Hz | payload MB/s | readers | delivered MB/s | basis |
|---|---|---|---|---|---|---|---|
| /zed2/zed_node/depth/depth_registered | car ZED | 0.92 MB | 30 (17 seen) | 27.6 (15.7) | 3 | 83 (47) | 640x360x4 |
| /camera/image_raw | car ZED (U1) | 0.92 MB | 30 | 27.6 | 2 | 55 | bgra8 |
| /camera/image_raw | Isaac **crosses** | 0.92 MB | 15 | 13.8 | 2 | 27.6 | bgra8 |
| /zed2/.../depth_registered | Isaac **crosses** | 0.92 MB | 15 | 13.8 | 3 | 41.5 | 32FC1 |
| /camera/image_raw | gz, decompressed on Thor | 0.69 MB | ~26 | 18.0 | 2 | 36 | rgb8 |
| /zed2/.../depth_registered | gz, decompressed on Thor | 0.92 MB | ~25 | 23.0 | 3 | 69 | 32FC1 |
| /sim/camera/depth_raw | linus local | 0.92 MB | ~30 | 27.6 | 1 | 27.6 | 32FC1 |
| /sim/camera/image_raw | linus local | 0.69 MB | ~30 (36) | 20.7 | 1 | 20.7 | rgb8 |
| /camera/image_annotated | Thor/car | 0.69 MB | 13.7 | 9.5 | 1 (+logger, +lazy relay) | 9.5-19 | bgr8 |
| /camera/image_raw/viz | Thor/car | 0.92 MB | 5 | 4.6 | 0 | 0 | publish cost only |
| /camera/image_annotated/viz | Thor/car | 0.69 MB | 5 | 3.5 | 0 | 0 | publish cost only |
| /camera/detection_masks | Thor/car | 0.23 MB | 13.7 | 3.2 | 2 (+logger) | 6.3 | mono8 |
| /perception/debug/front_roi_depth | Thor/car | 50 KB | 17-30 | 0.9-1.5 | logger only | 0-1.5 | 180x70x4 |
| /costmap/visualization | Thor/car | 0.28 MB | 2 | 0.56 | lazy relay | 0-0.56 | rgb8 of the 94 KB map |
| /camera/image_raw/compressed | **L>T** (gz) | 21.4 KB | ~26 | 0.56 | 1 | 0.56 | prior meas. |
| /.../depth_registered/compressedDepth | **L>T** (gz) | 20.1 KB | ~25 | 0.50 | 1 | 0.50 | prior meas. |
| /scan | car local / **L>T** / **I>T** | 8.7 KB (Isaac 4.4) | 40 | 0.35 (0.18) | ~9 (+logger, +relay) | 3.1 (1.6) | 1081 x 2 x 4 B |
| /bt/tree_visualization | Thor/car | ~50 KB | 2.5 | 0.13 | lazy relay | 0-0.13 | PNG size guessed |
| /slam/map | Thor/car | 94 KB (to 2.25 MB) | 0.05-0.2 | ~0.02 | 4 (+logger, fox) | ~0.1 | latched; re-sent whole each update |
| /map | Thor/car | 4.0 MB | once | one-shot | 0 in stack | 4 MB per late joiner | latched |
| /tf | Thor/car | 0.1-0.6 KB | ~110 msg/s | 0.017 | ~7 (+fox, logger) | 0.12 | high message rate, not bytes |
| /clock | **L>T** (gz) / **I>T** (Isaac) | 12 B | 250 / 60 | ~0.003 | ~40 | ~10,000 / 2,400 msgs/s on the wire | rate x fan-out (see output/sim_clock_fanout.md) |
| /zed2/zed_node/point_cloud/cloud_registered | car ZED | 3.7 MB | 10 | 37 | 0 (lazy) | 0 | only if someone subscribes |

#### 4. Cross-machine flows and totals

##### (a) One machine (car Thor, or everything on linus in sim)
Nothing crosses. Relevant for Zenoh anyway: the large local streams above (ZED rgb + depth
~140 MB/s delivered on the car, decompressed sim images ~105 MB/s on the Thor) move today through
Fast DDS shared memory. Under rmw_zenoh they move between processes over the local session
transport (loopback TCP by default; Zenoh shared memory is an opt-in config in the installed
rmw_zenoh version, check before relying on it). These are the topics to measure first on the
car after a switch.

##### (b) Stack on the Thor, Gazebo + viz on linus
Note: the repo's own scripts (`scripts/sim/sim_net.sh`, `sim.sh`, `sim_stack.sh`) assume a direct
Ethernet link (Discovery Server on the Thor at 10.42.0.2:11811, ROS_DOMAIN_ID 42), not Tailscale.
The topic flows are the same over Tailscale; only capacity and latency differ.

linus -> Thor (subscribers are on the Thor):

| topic | payload MB/s | Thor readers | Fast DDS unicast wire MB/s |
|---|---|---|---|
| /clock (250 Hz) | 0.003 | ~40 | ~0.9 (10,000 pkts/s x ~90 B incl. RTPS/UDP/IP) |
| /scan | 0.348 | ~9 | 3.13 |
| /odom | 0.036 | 4-5 | 0.16 |
| /sensors/imu/raw | 0.016 | 2-3 | 0.04 |
| /zed2/zed_node/depth/camera_info | 0.012 | 2 | 0.02 |
| /camera/image_raw/compressed | 0.56 | 1 | 0.56 |
| /zed2/.../depth_registered/compressedDepth | 0.50 | 1 | 0.50 |
| **total linus -> Thor** | **~1.5 MB/s (~12 Mbit/s)** | | **~5.3 MB/s (~43 Mbit/s), ~13k pkts/s** |

Not crossing although published on linus: /camera/camera_info (no subscriber), /camera_pan/joint_state
(pan off), /sim/* (sim-local), the raw /sim/camera/* staging images (local by design).
Nothing from the sim is on /tf or /tf_static (sim RSP is remapped to /sim/tf).

Thor -> linus:

| topic | payload MB/s | linus readers |
|---|---|---|
| /ackermann_drive | ~0.001 (10 Hz x 60 B) | drive_bridge |
| /camera_pan/command | 0 (pan disabled) | camera_pan_bridge |
| /parameter_events (startup bursts), discovery | negligible | every rclcpp sim node |
| **total Thor -> linus (RMW)** | **~0.001 MB/s** | |

Visualisation on linus:
- Foxglove (the stack's tooling): the Foxglove app on linus talks **websocket** to
  foxglove_bridge on the Thor (port 8765). The bridge subscribes locally on the Thor, so viz
  topics never touch the RMW link and are unaffected by the Zenoh switch. Estimated websocket
  load with the default whitelist: /viz/scan 0.04 + annotated JPEG 3.3 Hz x ~40 KB 0.13 +
  costmap JPEG 2 Hz x ~30 KB 0.06 + BT PNG 2.5 x ~50 KB 0.13 + /tf 0.02 + /slam/map ~0.02 +
  rest < 0.02 = **~0.4 MB/s (~3 Mbit/s) per client** (ESTIMATED; JPEG/PNG sizes guessed).
  /map (4.0 MB) exceeds foxglove_send_buffer_limit (2.5 MB) and is never delivered.
- RViz on linus would be an RMW participant: `~/.rviz2/default.rviz` subscribes /map (4.0 MB
  latched, one-shot), /map_updates, /plan (no publisher), /tf (~0.02 MB/s), /tf_static. Every
  display added (/scan 0.35 MB/s, /slam/map, an image 9.5 MB/s raw) adds its full payload T>L.

##### (c) HIL: Isaac on a workstation, stack on the Thor
Assumes the stack runs `sim:=true` (no VESC/ZED/urg drivers, all nodes on /clock) and Isaac's
`isaac_ws/config/ros_interface.yaml` topics: Isaac publishes raw images directly, there is no
compress/decompress pair.

Isaac -> Thor:

| topic | payload MB/s | Thor readers | Fast DDS unicast wire MB/s |
|---|---|---|---|
| /camera/image_raw (bgra8 15 Hz) | 13.82 | 2 (yolo, image_raw_viz_throttle) | 27.65 |
| /zed2/zed_node/depth/depth_registered (32FC1 15 Hz) | 13.82 | 3 (detection_3d, front_clearance, swept_clearance) | 41.47 |
| /scan (no intensities) | 0.175 | ~9 | 1.58 |
| /odom | 0.036 | 4-5 | 0.16 |
| /sensors/imu/raw | 0.016 | 2-3 | 0.04 |
| /zed2/zed_node/depth/camera_info | 0.006 | 2 | 0.01 |
| /clock (60 Hz) | 0.001 | ~40 | ~0.2 (2,400 pkts/s) |
| **total Isaac -> Thor** | **~27.9 MB/s (~223 Mbit/s)** | | **~71 MB/s (~570 Mbit/s)** |

During a mission, mission_logger adds one more reader of /scan, /odom, /sensors/imu/raw
(+0.4 MB/s wire). Published by Isaac but not crossing (no Thor subscriber): left/right
image_rect_color and camera_info, /camera/camera_info, /sim/ground_truth/*. If any Thor tool
subscribes to one of the extra images (Foxglove raw debug, rqt), add 13.8 MB/s each.

Thor -> Isaac: /ackermann_drive ~0.001 MB/s (Isaac sub R d1).

##### What the fan-out column means for Zenoh
The "wire" columns are what Fast DDS does today: no multicast across the link, so every
remote reader process gets its own unicast copy. Under rmw_zenoh the cross-host count depends on
the topology chosen: with the default peer mode, a Thor process can open a direct session to the
remote publisher and the copy count stays per-process; with a zenohd router on each host and
processes attached only to their local router, each sample crosses once per host. For (c) that
is the difference between ~71 MB/s and ~28 MB/s; for (b) between ~5.3 MB/s / 13k pkts/s and
~1.5 MB/s with /clock crossing once per tick.

#### 5. Durability, latching, composition, discovery settings

Transient-local publishers: /tf_static (every static broadcaster and RSP), /robot_description,
/zed2/zed2_description, /sim/robot_description, /slam/map, /slam/map_metadata, /map,
/mission/status (`loader.py:138` R TL d1), /calibration/in_progress (R TL d1),
/supervisor/health (R TL d1), /mpc/config (R TL d1), /perception/front_clearance/config
(R TL d1), /rosout (Jazzy default), Isaac /sim/ground_truth/objects and /start_pose.
TL subscribers that need the latch: StaticFrames /tf_static subscribers (d100) in 7 nodes,
costmap_boundary and costmap_renderer on /slam/map, BT and mission_logger on /mission/status,
supervisor on /calibration/in_progress.

Composition / intra-process: no stack launch file sets `use_intra_process_comms`. Composable
containers: `viz_relay_container` (rclcpp_components component_container, 15 ThrottleNodes, no
IPC) and the ZED wrapper's `zed_container` (`enable_ipc` default true in
`zed_camera.launch.py:589`, but no stack consumer lives in that container, so IPC changes nothing).
slam_toolbox is a LifecycleNode driven by launch events; map_server by lifecycle_manager_map.

ROS_LOCALHOST_ONLY: not used anywhere. ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST: only in test /
replay scripts and the `f1tenth_simple_discovery` helper (`scripts/env/jazzy.sh`).
Fast DDS-specific settings that become moot under Zenoh: ROS_DISCOVERY_SERVER and the
`fastdds discovery` process (`supervisor_bringup.launch.py`, `ensure_discovery_server.py`),
FASTRTPS_DEFAULT_PROFILES_FILE (`fastdds_profile.xml`, 5 s lease), ROS_SUPER_CLIENT set for
foxglove_bridge, viz_relays, mission_logger and ekf_cost_observer (they need the full graph:
topic_tools and rosbag2 resolve types from discovery), the `ros2cli` super-client helper, the
supervisor's `/dev/shm/*fastrtps*` cleanup, RMW_IMPLEMENTATION=rmw_fastrtps_cpp in
`scripts/env/jazzy.sh` and Isaac's `ros_interface.yaml` + `fastdds_udp.xml`.

QoS mismatches seen: none that block matching in the default run. Best-effort subscribers on
reliable publishers everywhere (EKF inputs, /scan consumers, /clock readers) are compatible.
Notes: Isaac chose reliable for its sensors because the stack's image subscribers are reliable
(its config says so); `/clock` from clock_throttle is R d1 while readers are BE.
mission_logger's comment says costmap_boundary / semantic_layer publish best effort; the code
publishes R d10 (`costmap_boundary_node.py:487`, `semantic_layer_node.py:274`), so the override
is unnecessary but harmless. rmw_zenoh does not enforce DDS QoS compatibility, so any latent
mismatch would start matching rather than fail.

#### 6. Services and actions

Services (Zenoh queryables under rmw_zenoh):
- component_supervisor_node: /restart_component (f1tenth_messages/RestartComponent),
  /component_supervisor_node/control_component (ComponentControl),
  /component_supervisor_node/run_calibration (std_srvs/Trigger).
- reset_manager: /reset_all (Trigger); clients of /restart_component, /mission/abort_mission,
  /set_pose and /ekf_global/set_pose (robot_localization SetPose); Nav2 cancel/clear only with Nav2.
- mission loader (behavior_executor): /mission/load_mission (LoadMission), /mission/start_mission,
  /mission/abort_mission, /mission/emergency_stop (Trigger). llm_planner_node (on demand) is a client.
- diagnostics_server: /diagnostics_server_node/run_diagnostics (RunDiagnostics).
- robot_localization x2: set_pose, enable, reset, toggle (`ros_filter.cpp:1156-1178`), in / and /ekf_global/.
- slam_toolbox: /slam_toolbox/{dynamic_map, pause_new_measurements, serialize_map,
  deserialize_map, reset, clear_changes, manual_loop_closure, toggle_interactive_mode,
  get_interactive_markers, save_map} + lifecycle (change_state, get_state, ...), called by the
  launch process at startup.
- map_server: /map_server/map (GetMap), /map_server/load_map + lifecycle; lifecycle_manager_map:
  manage_nodes, is_active.
- mission_logger: rosbag2 recorder pause/resume, and list/get_parameters clients to every node at
  mission start (parameter snapshot).
- ZED wrapper (car): ~20 /zed2/zed_node/* services (reset/set pose, SVO, toggles).
- gz side (linus-local): controller_manager services used by the spawners; Isaac: /sim/reset.
- Every node: 6 parameter services + ~/get_type_description (Jazzy), roughly 45 nodes x 7 = ~300
  services; foxglove_service_whitelist hides the parameter ones.

Actions: only `navigate_through_poses` (NavigateThroughPoses client in the BT,
`navigate_through_poses_client.py:77`), idle with Nav2 off; Nav2's own actions only with
enable_nav2. No action crosses machines in (b) or (c).

#### 7. Uncertainties

- U1. ZED wrapper 5.4.1 renamed `~/rgb/image_rect_color` to `~/rgb/color/rect/image` (its
  CHANGELOG line 112; topic built at `zed_camera_component_video_depth.cpp:66`).
  `camera.launch.py:88` still remaps the old name, so on the car `/camera/image_raw` and
  `/camera/camera_info` are probably not produced by the ZED (yolo would get no input). Sim is
  unaffected (it publishes /camera/image_raw itself). Check with `ros2 topic list` on the car.
  Isaac's `ros_interface.yaml` also uses the v4 left/right names.
- U2. urg_node QoS (R d20) comes from upstream knowledge; urg_node source is not on this machine.
  If it published best effort, IsProximityTooClose's reliable /scan subscription would not match.
  The Isaac config states the real urg_node offers RELIABLE.
- U3. mpc_corr subscribes /imu and /model/virtual_robot/odometry; nothing publishes either.
- U4. Rates: image paths use the configured 30 Hz; comments in stack_params record ~17 Hz ZED
  depth and ~13.7 Hz YOLO on the car. In sim every rate is in sim time and scales with RTF.
- U5. Sizes guessed rather than derived: BT PNG (~50 KB), JPEG q75 viz images (~30-50 KB), URDF
  strings, marker arrays. The compressed sim camera sizes are prior measurements.
- U6. /slam/map size depends on the explored area (94 KB measured once on the car); the costmap
  render scales with it (x3).
- U7. Reader counts are processes counted from the code in the default config; mission_logger
  and the lazy viz relays add readers only while recording / while a Foxglove client watches.
- U8. Isaac `publish_static_tf` defaults to true although its comment says it must be false with
  the stack; Isaac /clock is 60 Hz (physics step), not 250 Hz like the gz throttle.
- U9. robot_localization, slam_toolbox, topic_tools and py_trees_ros QoS come from source copies
  that may differ in patch version from the Thor's apt packages (slam_toolbox on the Thor is
  2.6.10 per `reset_manager.yaml`); the QoS calls checked have been stable across those versions.

---

# Appendix B: rmw_zenoh and Isaac Sim research, with citations

Written in this session, read-only, from the sources listed at its end. Headings are shifted down two levels.

### rmw_zenoh on Jazzy for the F1TENTH stack, and the Isaac Sim bridge

Research date 2026-10-16. Read-only: nothing installed, built or changed. The only "execution" was
`docker run --rm --network none` with `ls`/`cat`/`grep` against the existing Isaac image, to list the
bundled bridge libraries.

#### 0. Version baseline (what the statements below apply to)

| Item | Value | Source |
|---|---|---|
| `ros-jazzy-rmw-zenoh-cpp` in **main** apt (amd64 and arm64) | **0.2.11** (amd64 `0.2.11-1noble.20260915.171026`, arm64 `...20260915.184907`) | live `packages.ros.org/ros2/ubuntu/dists/noble/main/binary-{amd64,arm64}/Packages.gz`, InRelease Last-Modified 2026-10-06 |
| Zenoh bundled by 0.2.11 | **zenoh-c 1.8.0 + fixes** (commit `05bd370…`; a Rust 1.75 fallback commit `b31348f…`), cargo features `shared-memory zenoh/transport_serial` | https://raw.githubusercontent.com/ros2/rmw_zenoh/0.2.11/zenoh_cpp_vendor/CMakeLists.txt |
| Next Jazzy release | **0.2.12** (2026-10-06) in **ros2-testing only** (amd64 `0.2.12-1noble.20261011`, arm64 `...20261007`): Zenoh bumped to **1.10.1 + fixes** (PR #1071, backport of #1068) | rmw_zenoh_cpp CHANGELOG.rst (jazzy branch); ros2-testing Packages.gz; https://github.com/ros2/rmw_zenoh/pull/1071 |
| 0.2.11 changes | #1057, #1051, #1040 (lost-wakeup fix, backport of #1036), #1026, #1023 ("When SHM enabled, enable transport_optimization") | https://raw.githubusercontent.com/ros2/rmw_zenoh/0.2.11/rmw_zenoh_cpp/CHANGELOG.rst |
| 0.2.10 (2026-07-22) | Zenoh 1.7.1 then 1.8.0; #1014 lock-order/deadlock fix (backport of #1005/#1010); #943 deadline/liveliness events (see 4) | same |
| Other Jazzy apt versions today | foxglove-bridge 3.6.0, navigation2 1.3.13, rosbag2 0.26.11, slam-toolbox 2.8.5, robot-localization 3.8.3, tf2-ros 0.36.23 | main Packages.gz |

The Thor (arm64) and an amd64 laptop or workstation get the same 0.2.11 from main today. Earlier
there was an amd64/arm64 skew that broke the wire format. With 0.2.3 on one side and 0.2.4 on the
other, rviz panicked with `zenohc::serialization: Failed to deserialize the payload`
(https://github.com/ros2/rmw_zenoh/issues/637, closed 2025-05-23). With 0.2.3 against 0.2.5 every
node crashed (https://github.com/ros2/rmw_zenoh/issues/733). **Rule: run the exact same rmw_zenoh
version on every host and container.** In particular, do not mix 0.2.11 (main) with 0.2.12
(testing). Wire compatibility between 0.2.11 and 0.2.12 (Zenoh 1.8 to 1.10) is **not verified**.

"README" below means https://github.com/ros2/rmw_zenoh/blob/jazzy/README.md (jazzy branch head,
which equals 0.2.12). The config files quoted are the **0.2.11 tag** copies:
`rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_{SESSION,ROUTER}_CONFIG.json5`. The design doc is
https://github.com/ros2/rmw_zenoh/blob/jazzy/docs/design.md.

---

### Part A: rmw_zenoh on Jazzy, risks and gaps

#### A1. Router (zenohd)

**Is the router mandatory?** Yes, in practice. The ROS docs page "Working with Zenoh" (Jazzy,
source `ros2_documentation/jazzy/.../Working-with-Zenoh.rst`) says:

> "Without the Zenoh router, nodes will not be able to discover each other since multicast
> discovery is disabled by default in the node's session config. Instead, nodes will receive
> discovery information about other peers via the Zenoh router's gossip functionality."

The design doc says "As of 2024-02-09, the user is expected to launch the router by hand." The
command is `ros2 run rmw_zenoh_cpp rmw_zenohd`.

**What happens without a router** (0.2.11 `rmw_context_impl_s.cpp:401-435`, README "Checking for a
Zenoh router"):
- `ZENOH_ROUTER_CHECK_ATTEMPTS` unset means 1 check. `0` waits forever, `<0` skips the check, and
  `N>0` makes N checks 1 s apart.
- After the attempts the node logs `Unable to connect to a Zenoh router after %zu attempt(s)…
  Proceeding with initialization but other peers will not discover or receive data from peers in
  this session until a router is started.` and **continues**. Nodes do not block or fail. They are
  silently isolated until a router appears.
- Sessions keep retrying the router: session `connect.timeout_ms: { router: -1, peer: -1, client: 0 }`
  and `exit_on_failure: { router: false, peer: false, client: true }`. A peer therefore reconnects
  when the router (re)starts. A **client-mode** session with the default `exit_on_failure.client:
  true` gives up on the first connect failure. That matters for the laptop in client mode.
- **Inference, not verified:** data between same-host peers flows over direct peer-to-peer TCP
  links (design doc: "Data is sent via direct peer-to-peer connections"). If the router dies,
  existing local links should keep working, but new discovery and all inter-host traffic stop.

**One router per host.** README "Connecting multiple hosts": "By default, all discovery &
communication is restricted within a host, where a host is a machine running a Zenoh router along
with various ROS 2 nodes". To join hosts, copy the router config and set `connect.endpoints` to the
other router (for example `["tcp/192.168.1.1:7447"]`), then start `rmw_zenohd` with
`ZENOH_ROUTER_CONFIG_URI` pointing at it. A connect in one direction is enough. Router-to-router is
the documented multi-host topology.

**Config env vars** (0.2.11 `zenoh_config.cpp:40-52`):
- `ZENOH_ROUTER_CONFIG_URI`, `ZENOH_SESSION_CONFIG_URI`: a full replacement file. Start from a copy
  of the default, because a custom file replaces the defaults entirely (Stereolabs doc, below).
- `ZENOH_CONFIG_OVERRIDE='key/path=value;key2=value2'` is applied after the file, to both router
  and nodes (added in 0.2.3). Examples from the README:
  `connect/endpoints=["tcp/192.168.0.3:7447"]`, `scouting/multicast/enabled=true` (together with
  `ZENOH_ROUTER_CHECK_ATTEMPTS=-1`), and `mode="client";connect/endpoints=[…]`. Each `export`
  replaces the previous value, so put everything in one `;`-separated string.
- `ZENOH_ROUTER_CHECK_ATTEMPTS` (above). `ZENOH_SHM_ALLOC_SIZE` and
  `ZENOH_SHM_MESSAGE_SIZE_THRESHOLD` are legacy and "will be deprecated". `RMW_ZENOH_BUFFER_POOL_MAX_SIZE_BYTES`
  sets the serialization buffer pool (default 8 MiB). `RUST_LOG` sets logging (for example
  `zenoh=info`, `zenoh_transport=debug`).

**Ports and endpoints (0.2.11 defaults):**
- Router: `listen.endpoints: ["tcp/[::]:7447"]` (router cfg line 92), `connect.endpoints: []`.
- Node session: `mode: "peer"`, `connect.endpoints: ["tcp/localhost:7447"]` (session cfg line 45),
  `listen.endpoints: ["tcp/localhost:0"]` (line 97, "accept incoming connections only from
  localhost").
- **Multicast scouting is off** in both files: `scouting.multicast.enabled: false` with the comment
  "ROS setting: disable multicast discovery by default". Gossip is on: `multihop: false`,
  `autoconnect: { router: [], peer: ["router","peer"] }`.

**IPv4-only hosts: the router crashes** (README "Known Issues > Router crashes on IPv4-only
systems"). The default `tcp/[::]:7447` makes the router crash where IPv6 is disabled or broken. The
fix is to **replace** (not add to) the entry with `"tcp/0.0.0.0:7447"`. The workspace CLAUDE.md
records that the Orin has **no IPv6** (`ipv6.ko` fails to load), so the stock router config would
crash there. Check the Thor's kernel too, or use `tcp/0.0.0.0:7447` everywhere.

**Tailscale (scenario b).** There is no multicast and none is needed, because Zenoh uses unicast
TCP. Two documented options:
1. Laptop nodes in client mode connect straight to the Thor router:
   `ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<thor-tailscale-ip-or-magicdns>:7447"]'`
   (README "Connecting to the Zenoh router on another host"). The README explains why client mode
   is needed: "By default, Zenoh router doesn't forward messages between peers… we need to switch
   the remote node into the client mode". The router config has
   `routing.router.peers_failover_brokering: false` (line 239). A laptop *peer* would learn the
   Thor nodes' `tcp/localhost:0` listen endpoints through gossip and could not reach them.
2. A router on the laptop with `connect/endpoints=["tcp/<thor>:7447"]`, and laptop nodes left at
   the defaults. Community example: `tcp/va0003:7447#iface=tailscale0`, where hostnames work
   (https://discourse.openrobotics.org/t/rmw-zenoh-router-configuration/43885).

The Thor router listening on `[::]` or `0.0.0.0` exposes port 7447 on every interface, including
tailscale0. To restrict it, list explicit endpoints, or use the ACLs in `zenoh_security_tools`
(README "Security").

#### A2. Peer vs client mode

- Default node session: **peer** (session cfg `mode: "peer"`). Default router: `mode: "router"`.
  Peers on one host connect to the local router for gossip and then directly to each other
  (design doc, "Data is sent via direct peer-to-peer connections"; the router is "not used as an
  intra-host broker").
- **Client:** all traffic goes through the router it connects to. This is the recommended mode for
  a remote rviz or laptop (README). It costs one extra hop through the router.
- Remember that `exit_on_failure.client: true` and `connect.timeout_ms.client: 0`: a client exits
  if the router is unreachable at start. Override `connect/timeout_ms` if the laptop should wait.

#### A3. Shared memory (SHM)

- **Built in** (cargo `shared-memory`) but **off by default** in 0.2.11:
  `transport.shared_memory.enabled: false`, with the comment "ROS setting: disabled by default until
  fully tested".
- Enable it on **every** session that exchanges data, and optionally on the router:
  `ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'`. When `enabled` is true,
  rmw_zenoh forces `transport_optimization.enabled=true` and logs INFO (#1023, in 0.2.11).
- Sizes: `pool_size: 50331648` (48 MiB per process, sized for a 4K frame), `message_size_threshold:
  512` B (must be a power of two), `mode: "lazy"` (or `"init"`). If allocation fails, publishing
  silently falls back to the network transport. `/dev/shm` must hold one pool per process.
  In Docker use `--ipc=host`, a shareable IPC namespace, or a tmpfs volume at `/dev/shm` (README
  "Zenoh Shared Memory").
- SHM nodes interoperate transparently with non-SHM nodes and remote nodes (README
  "Interoperability").
- **Caveat that matters for the stack: open bug #978.** "Query requests are silently dropped when
  using shared memory when receiver's RX thread stalls" (https://github.com/ros2/rmw_zenoh/issues/978,
  opened 2026-06-01, still open). With SHM plus `transport_optimization`, a briefly descheduled
  receiver (high CPU load at launch, with many nodes) **silently loses service requests** (the
  caller hangs until `queries_default_timeout`, 10 min) **and TRANSIENT_LOCAL history queries**.
  The README carries the same warning (jazzy README lines 252-254). The root cause is
  eclipse-zenoh/zenoh#2628, fixed in **Zenoh 1.10.0** (#2646). The 0.2.11 apt release (Zenoh 1.8.0)
  is **affected**. The 0.2.12 release (Zenoh 1.10.1, testing) should contain the fix (inference
  from the changelog and the issue; not tested). Workaround on 0.2.11:
  `transport/shared_memory/enabled=true;transport/shared_memory/transport_optimization/enabled=false`.
  The reporter says pub/sub still uses SHM with this setting.
- Measured benefit: one report found SHM better than loopback but not better than
  rmw_cyclonedds for camera images (https://github.com/ros2/rmw_zenoh/issues/764, open, 0.2.6).
- **Recommendation:** start the Thor without SHM. If the ZED images make SHM necessary on the Thor,
  use the workaround above on 0.2.11, or move to 0.2.12 everywhere once it reaches main.

#### A4. QoS support (0.2.11; design doc "Quality of Service", `qos.cpp`, `rmw_publisher_data.cpp`, `rmw_subscription_data.cpp`)

| Policy | Behaviour |
|---|---|
| Reliability | RELIABLE gives a reliable Zenoh publisher. BEST_EFFORT "may be dropped". The default transport is TCP only. **Unlike DDS, a BEST_EFFORT publisher and a RELIABLE subscriber still communicate**: rosbag2 #1936 states "rmw_zenoh_cpp is not a DDS-backboned RMW", so a reliability mismatch does not block delivery. |
| History / depth | KEEP_LAST depth is the subscriber queue. `depth=0` becomes 42 (`qos.cpp:27`). |
| **Congestion control** | `pub_opts.congestion_control = DROP` unless the publisher is **RELIABLE + KEEP_ALL**, which gets BLOCK (`rmw_publisher_data.cpp:135-145`). With DROP, a message that cannot get a TX batch within `wait_before_drop: 1000` µs (1 ms), or within `max_wait_before_drop_fragments: 50000` µs (50 ms) for fragmented messages, is **silently dropped**. See A11. |
| Durability TRANSIENT_LOCAL | Emulated. The publisher is a Zenoh *AdvancedPublisher* with a cache of `max_samples = depth` (`rmw_publisher_data.cpp:117-131`). A RELIABLE+TL publisher also gets sample-miss detection with sporadic heartbeats. The subscriber is an *AdvancedSubscriber* with `history.max_samples = depth`, `detect_late_publishers = true` and `query_timeout_ms = max` (`rmw_subscription_data.cpp:192-206`). History is fetched by **queries**, which is why #978 affects latched topics. The default durability is VOLATILE. |
| Deadline, Lifespan | **Not implemented** (design doc). The QoS values are accepted. |
| Liveliness | AUTOMATIC only; MANUAL_BY_TOPIC is unsupported (design doc). |
| QoS events | `rmw_event_type_is_supported()` returns **false** for LIVELINESS_CHANGED, LIVELINESS_LOST, REQUESTED_DEADLINE_MISSED and OFFERED_DEADLINE_MISSED. The code comment says init succeeds "but Zenoh does not yet implement the underlying tracking — the callbacks will never fire" (0.2.11 and jazzy `rmw_event.cpp:198-215`). The 0.2.10 changelog entry #943 "add deadline/liveliness QoS events" only made these events **accepted**. They never fire. Matched and incompatible-QoS events are supported. |
| Unknown QoS values | Entity creation fails if the QoS holds UNKNOWN values (0.2.3 changelog). |

**Stack impact:** any node that relies on deadline-missed or liveliness-lost callbacks, for example
a watchdog built on `deadline` QoS, gets **no callback and no error**. Grep the stack for
`deadline(` and `liveliness(` before migrating. The component supervisor must detect dead
components by other means, such as process state or heartbeat topics with timers.

#### A5. Services and actions

- Services are Zenoh queryables. The client is a querier with `target ALL_COMPLETE`,
  `consolidation NONE`, and the timeout from `queries_default_timeout`. That is **600000 ms
  (10 min)** since 0.2.8 (#825), and **unlimited for `**/_action/get_result/**`**
  (`rmw_client_data.cpp:132-148`, 0.2.6 "get_result timeout to max"). Actions are built from
  services and topics; there is no RMW-level action.
- **Lost requests hang.** If a request or reply is dropped (#978 with SHM, or a broken link),
  `rclcpp`/`rclpy` futures do not complete until the 10 min query timeout. Use timeouts on the
  client side (`call_async` plus a wait with timeout), never an infinite wait. This matters for the
  std_srvs/Trigger calls of the supervisor and scripts.
- **Nav2 lifecycle timeout:** https://github.com/ros2/rmw_zenoh/issues/783 (open) reports `Service
  call reports z_reply_is_ok returned False Reason: Timeout` with nav2 bringup, because
  lifecycle_manager uses an infinite `change_state` timeout. JEnoch: PR #820 is a workaround
  pending ros2/rmw#404, an RMW API that would pass request timeouts down. The 10 min default
  (0.2.8) is part of that mitigation.
- **Discovery race:** https://github.com/ros2/rmw_zenoh/issues/1029 (open, Jazzy, aarch64 RPi):
  "Client session sometimes never discovers an existing, reachable queryable". The maintainer
  asked the reporter to retest with jazzy HEAD, meaning Zenoh 1.10.1 (#1071, part of 0.2.12). The
  fix is unconfirmed. `wait_for_service` relies on the graph cache from liveliness tokens.
- Earlier deadlocks in `rmw_wait` and the client teardown (#921, #998, #993, #1003, #1032) are
  fixed and backported: #1014 is in 0.2.10 and #1040 is in 0.2.11.
- Service server latency spikes on a PREEMPT_RT kernel: https://github.com/ros2/rmw_zenoh/issues/862
  (open, not reproduced by the maintainer).

#### A6. tf_static and TRANSIENT_LOCAL late joiners

- **The most relevant open issue: https://github.com/ros2/rmw_zenoh/issues/976** "Late joining
  subscriber doesn't get data" (open, `bug`). The original report is from Kilted (rmw_zenoh 0.6.6, Zenoh 1.7.2) on Ubuntu 24.04 x86_64 (corrected after re-reading the issue on 2026-10-16).
  A 2026-09-03 comment (Kilted rmw_zenoh 0.6.7, **Zenoh 1.8.0, the same Zenoh as Jazzy 0.2.11**,
  multi-host, peers connected to a router on another host) measured `/tf_static` delivery to a
  late RELIABLE+TL subscriber by **subscriber depth**:
  - depths 1 to 50: history arrives at +0.0 to 0.1 s;
  - depth 60: +2.3 s; depth 80: nothing in 12 s; depth 99: +8.5 s;
  - **depth 100: never, and later live messages do not arrive either.**

  **tf2_ros's `StaticListenerQoS` default depth is 100** (geometry2 jazzy
  `tf2_ros/include/tf2_ros/qos.hpp:56`, used by every `TransformListener`). The reporter's
  consequence: "every stock tf2_ros::TransformListener … never learns the static frames when it
  starts after robot_state_publisher". The reported workaround is `tf2_ros::StaticListenerQoS(10)`,
  or `static_qos=QoSProfile(depth=10, durability=TRANSIENT_LOCAL)` in rclpy. `/robot_description`
  behaved the same way. No maintainer has confirmed this yet, it is not yet reproduced on Jazzy,
  and the data comes from a multi-host setup. **This must be tested on the Thor before
  migrating.** The test: start robot_state_publisher first, then start a node with a default
  TransformListener and check `tf2_echo` for a static frame. If it fails, the stack's own
  TransformListeners need `StaticListenerQoS(10)`. Third-party nodes such as nav2, slam_toolbox,
  robot_localization and rviz cannot easily be patched, which would be a **blocker**.
- #978 (A3) also drops TL history under CPU load when SHM is enabled.
- Large TL history over a slow link (#710, duplicate of #696, closed): a 1 MB × depth 25 replay
  to a late joiner over 8 Mbit/s closed the transport ("Unable to push non droppable network
  message … Closing transport!"). Keep TL depths small for large messages. Stereolabs repeats
  this advice.
- Memory: a TL publisher cache holds `depth` samples per publisher (`max_samples = depth`), the
  same order as DDS. There are no reports of leaks specific to TL. An image-publishing memory leak
  in Jazzy (#827) was fixed in 0.2.8 ("Fix memory leak when publishing with the default
  allocator").

#### A7. Nav2

- There is no compatibility statement on docs.nav2.org (search found nothing).
- Nav2 **nightly CI runs fastrtps, cyclonedds and rmw_zenoh**
  (https://github.com/ros-navigation/navigation2/issues/6163). Of 21 Zenoh failures, 20 were the
  Tokio "Thread Local Storage … destroyed" panic at process exit, caused by `rclcpp::shutdown()` in
  global destructors. That is test-harness hygiene, fixed by nav2 PR #6241 (merged 2026-07-05). The
  README lists this panic as a known issue: call `rclcpp::shutdown()` explicitly, and **composable
  nodes should never call it**.
- Functional issue: #783 (lifecycle `change_state` with an infinite timeout, see A5).
- The Nav2 lifecycle bringup is a burst of services at launch, the exact case #978 describes when
  SHM is on.

#### A8. rosbag2 (Jazzy 0.26.11)

- Recording works in general. **Serious bug, now fixed:** https://github.com/ros2/rosbag2/issues/2463
  "`rosbag2_transport::Recorder` quietly stops recording all messages after an unspecified amount
  of time" (Jazzy, rmw_zenoh, Jetson Orin AGX and Intel; still open in rosbag2). Users traced it to
  rmw_zenoh wait-set bugs fixed by #1005 and #1036. The Jazzy backports are #1014 (in 0.2.10) and
  #1040 (in 0.2.11, merged 2026-08-28). **Use at least 0.2.11.** On a first run, check that bag
  size and message counts keep growing.
- QoS: rosbag2 0.26.7 contains #1936/#1938, "QoS incompatibilities are not expected with
  rmw_zenoh_cpp". It changes test expectations only. Reliability mismatches do not block delivery
  on Zenoh.
- Recording TL topics such as `/tf_static`: the recorder is a late joiner whenever it starts after
  the publishers, so #976's depth dependence applies. **Not verified.** Test that `/tf_static`
  lands in a bag started after the stack.
- Large topics: the recorder receives through the same DROP congestion path (A11) when the
  publisher's tx queue is congested. Measure `ros2 bag info` counts against `topic hz`.

#### A9. Foxglove bridge (ros-jazzy-foxglove-bridge 3.6.0)

- **No reports found.** A GitHub search of foxglove/ros-foxglove-bridge and foxglove/foxglove-sdk
  for "zenoh" returned nothing, and so did rmw_zenoh issues for "foxglove". It is plain
  rclcpp with generic subscriptions and graph queries, which rmw_zenoh supports. Treat it as
  **untested, not known-broken**.
- Recommendation for scenario (b): run foxglove_bridge **on the Thor**, so only the WebSocket
  (8765) crosses Tailscale and Zenoh never carries images over Tailscale. Foxglove's per-client
  topic selection and throttling then also limit the WAN load.

#### A10. robot_localization, slam_toolbox

- **No reported issues.** GitHub searches of cra-ros-pkg/robot_localization and
  SteveMacenski/slam_toolbox for "zenoh" returned nothing. Their risks are the generic ones:
  TransformListener static depth 100 (A6), `/map` TRANSIENT_LOCAL late joiners (A6, depth 1 is in
  the "works" range), and slam_toolbox's services (A5).
- Related intra-process issue: https://github.com/ros2/rmw_zenoh/issues/1046 (open, Kilted
  0.6.7, Zenoh 1.8.0). In `component_container_mt` with intra-process comms, a publisher whose only
  subscriber is in the same process reports a **matched count of 0**, so lazy publishers never
  publish. This matters for the **ZED wrapper**, which publishes lazily based on subscriber counts
  and usually runs as a composable node. Test ZED topics with consumers in the same container.

#### A11. Large messages (ZED images and depth at 15-30 Hz)

- Size: 720p bgra8 is about 3.7 MB, and 720p 32FC1 depth is about 3.7 MB. Two to three such
  streams at 15-30 Hz is 110-330 MB/s. **That exceeds 1 GbE (about 118 MB/s) whatever the RMW**,
  and Tailscale far more so.
- **The silent-drop trap:** a KEEP_LAST publisher (the ZED default is RELIABLE KEEP_LAST 10) uses
  DROP congestion control with `wait_before_drop` = 1 ms and `max_wait_before_drop_fragments` =
  50 ms. Messages above `batch_size` (65535 B, the 16-bit maximum, which cannot be raised) are
  fragmented. Multi-MB messages over a real link are dropped **silently**:
  - https://github.com/ros2/rmw_zenoh/issues/696 (open): loss grows gradually with payload size
    even on loopback. JEnoch explains the DROP mechanism and gives two fixes: RELIABLE + KEEP_ALL,
    which blocks the publisher, or
    `ZENOH_CONFIG_OVERRIDE='transport/link/tx/queue/congestion_control/drop/wait_before_drop=1000000'`
    (µs).
  - https://github.com/ros2/rmw_zenoh/issues/672 (open): PointCloud2 or Image is visible in
    `topic list` but **never** received over the network. JEnoch's advice is to set
    `wait_before_drop=400000` **on `rmw_zenohd`**, because the router forwards inter-host traffic.
    A 2026-04 report (peci1) works with
    `wait_before_drop=1000000;max_wait_before_drop_fragments=5000000;transport/link/rx/buffer_size=16777216;transport/link/rx/max_message_size=1073741824`.
  - Stereolabs ZED docs (https://docs.stereolabs.com/docs/integrations/ros-2/using-zenoh-as-middleware)
    recommend the same `wait_before_drop=1000000`, or RELIABLE+KEEP_ALL, an optional
    `RMW_ZENOH_BUFFER_POOL_MAX_SIZE_BYTES=33554432`, SHM only for same-host, compressed topics for
    remote, and no DDS sysctl tuning (TCP).
- Other knobs:
  - `transport/link/tx/queue/size/data` (default 2 batches);
  - TCP buffers per endpoint, for example `tcp/0.0.0.0:7447#so_sndbuf=…;so_rcvbuf=…` (config
    comment, line 24);
  - `transport/unicast/lowlatency` (default false; it cannot fragment, so **do not** use it for
    images);
  - `queries_default_timeout` (600000) only affects services and TL queries, not pub/sub.
- **Recommended starting override for the Thor (nodes and router) and the HIL link:**
  `transport/link/tx/queue/congestion_control/drop/wait_before_drop=1000000;transport/link/tx/queue/congestion_control/drop/max_wait_before_drop_fragments=5000000`.
  Verify with `ros2 topic hz` on the *remote* side. Stay aware of the latency cost: a congested
  link now buffers up to 1 s instead of dropping.
- A 2026-02 report from a Jetson with PREEMPT_RT on Humble saw intermittent latency above 1 s that
  FastRTPS did not show (https://github.com/ros2/rmw_zenoh/issues/912, open). The maintainer
  suspects SHM was not actually used between containers.

#### A12. Mixed RMW (rmw_zenoh_cpp with rmw_fastrtps_cpp)

- **No interop.** Zenoh is not RTPS, and nodes on different RMWs simply never see each other: no
  error, no warning, missing topics. ROS docs (jazzy, "About different middleware vendors > Cross-Vendor
  Communication"): "While different RMW implementations may be compatible in limited
  circumstances, this is not guaranteed… ensure that all parts of the distributed system are using
  the same ROS version and the same RMW implementation." The rmw_zenoh README also states it
  "cannot interoperate with `zenoh-plugin-ros2dds`" (B3) and is incompatible between Humble and
  Iron+ (type hashes are in the key expressions; participants are discovered but messages are
  "silently dropped", #569).
- **`ros2 daemon`:** the daemon is spawned with the RMW of the first CLI call and keeps it. ROS
  docs ("Working with multiple RMW implementations"): "Even if you run the command line tool again
  with the correct RMW implementation, the daemon's RMW implementation will not change and the ROS
  2 command line tools will fail. … `ros2 daemon stop`." The README's first test step is
  `pkill -9 -f ros && ros2 daemon stop`. Under Zenoh the daemon is also a Zenoh session using the
  same `ZENOH_*` env vars. It does not follow a changed router address until stopped
  (https://github.com/ros2/rmw_zenoh/issues/853, closed 2025-11). Use `ros2 daemon stop` or
  `--no-daemon` when switching. Open design issue: daemon vs router lifetime
  (https://github.com/ros2/rmw_zenoh/issues/242). **Inference:** with a mismatched daemon,
  `ros2 topic list` shows only the daemon's RMW world, for example an empty or stale list.
- Practical consequence: the switch has to be atomic across the Thor stack, the supervisor, the
  scripts (`scripts/sim/*.sh`), rviz/foxglove, the Isaac sim and the Docker images.

#### A13. Graph introspection, ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY / ROS_AUTOMATIC_DISCOVERY_RANGE

- Graph: every **context** (one per process with rclcpp/rclpy init) keeps a cache of all
  entities. It bootstraps with `liveliness_get` and then follows liveliness tokens
  `@ros2_lv/<domain>/<zid>/<nid>/<eid>/<kind>/<enclave>/<ns>/<node>/<topic>/<type>/<hash>/<qos>`
  (design doc). The cost is O(all entities) per process, plus token churn on every node restart,
  which the supervisor causes. Config values were tuned for more than 200 nodes (0.2.3). There is
  one unresolved report of an rmw_zenohd CPU spike (all cores near 100 % for about 1 s, loopback
  traffic ×10) when a node restarts on another host
  (https://github.com/ros2/rmw_zenoh/issues/763, closed 2026-01 without a code fix; it went away
  after re-basing on the current default config).
- ROS_DOMAIN_ID: rcl passes it to the RMW, with 0 as the default (`rmw_init.cpp:69-70`). It
  becomes the first key-expression segment `<domain_id>/<topic>/<type>/<hash>`. Different domains
  never communicate, even through linked routers (design doc). Routers are domain-agnostic: one
  router serves all domains. Stereolabs: "ROS_DOMAIN_ID still isolates graphs, as with DDS". Note
  that the Isaac sim uses domain 42 (`config/ros_interface.yaml:3`) and the car uses 0
  (`isaac_ws/output/todo_for_andreas.md:55`). All processes must agree.
- **ROS_LOCALHOST_ONLY, ROS_AUTOMATIC_DISCOVERY_RANGE and ROS_STATIC_PEERS are ignored.** None
  appears in 0.2.11 `rmw_init.cpp`, `rmw_context_impl_s.cpp` or `zenoh_config.cpp`
  (`discovery_options` is not read), and neither the README nor the design doc mentions them.
  Locality comes from the Zenoh config: sessions listen on `tcp/localhost:0`, and the router's
  `connect`/`listen` decide what leaves the host. `ROS_DISCOVERY_SERVER` and
  `FASTRTPS_DEFAULT_PROFILES_FILE` are also irrelevant under Zenoh. **Inference:** leftover
  settings are harmless, but they would mislead anyone reading the env.

#### A14. Jetson / aarch64, router CPU

- Arm64 binaries ship with every sync; the versions match amd64 today (0.2.11).
- Historic arm64/amd64 sync skew broke the wire format (#637, #733; see section 0). Pin versions.
- #912: latency spikes on Jetson with PREEMPT_RT on Humble (open). #976 (x86_64) and #1046 were reported on
  Kilted (A6, A10). #1029 was reported on aarch64 (A5).
- Router CPU: #763 (A13). No Thor-specific issue was found.
- **The Orin has no IPv6, so it hits the IPv4-only router crash** (A1).
- Benign log noise: `zenoh_shm::watchdog … error setting scheduling priority` (Stereolabs).
- **Docker on the Orin: host networking only** (CLAUDE.md). That is fine for Zenoh, which uses TCP
  7447 plus the peers' ephemeral loopback ports. For SHM across containers, also use `ipc: host`.

#### A15. What the switch needs (checklist)

1. Run one `rmw_zenohd` per host as a supervised service, started before the stack. Set
   `ZENOH_ROUTER_CHECK_ATTEMPTS=0` on stack nodes so they wait for it, or `-1` for tools.
2. Use the same `ros-jazzy-rmw-zenoh-cpp` version on Thor, laptop, workstation and every image.
3. In the router config use `listen: ["tcp/0.0.0.0:7447"]` (IPv4-safe) and set `connect:` for
   router-to-router links (Thor to workstation for HIL, Thor to laptop or laptop client mode for
   Tailscale). Apply the `wait_before_drop` overrides on routers and on image publishers.
4. Test #976 (tf_static depth 100) and #1046 (ZED lazy publishing) on the Thor first.
5. Leave SHM off at first. If you enable it on 0.2.11, set `transport_optimization/enabled=false`
   (#978).
6. Remove the expectations on deadline/liveliness callbacks, and put timeouts on every service
   call.
7. Add `ros2 daemon stop` to scripts that switch RMW. Drop the Fast DDS env vars
   (`ROS_DISCOVERY_SERVER`, profiles) from the Zenoh configuration.

---

### Part B: Isaac Sim ROS 2 bridge

#### B1. How /home/andreas/isaac_ws (branch `f1tenth`) runs the bridge (read-only inspection)

- **Image:** `isaac_ws_devcontainer-isaac-sim:latest`, built FROM `nvcr.io/nvidia/isaac-sim:5.1.0`
  (`.devcontainer/Dockerfile:1`). Its `/isaac-sim/VERSION` is `5.1.0-rc.19+release.26219…` and
  the bridge extension is `isaacsim.ros2.bridge` **4.12.4** (extension.toml). Compose uses
  `pull_policy: never`, `network_mode: host`, `ipc: host` and `runtime: nvidia`
  (`.devcontainer/docker-compose.yaml:6-10`).
- **No system ROS in the container.** `ls /opt/ros` fails ("No such file or directory").
  `Dockerfile:7` appends `source /opt/ros/jazzy/setup.bash` to `/etc/bash.bashrc`, which only prints
  a harmless error in interactive shells (`output/decisions.md:6`, D2). The older `/opt/ros/jazzy`
  mount from `output/isaac_recon.md:90` is gone from the current compose.
- **The internal bridge libraries are used:**
  - compose env `ROS_DISTRO=${ISAAC_ROS_DISTRO:-jazzy}`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
    `LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/${ISAAC_ROS_DISTRO:-jazzy}/lib`,
    `ROS_DOMAIN_ID`, `FASTRTPS_DEFAULT_PROFILES_FILE=/dds/${ISAAC_DDS_FILE:-fastdds_udp.xml}`,
    discovery pass-through, `PYTHONPATH=` (`docker-compose.yaml:20-29`);
  - `launch.sh:54` `export ROS_DISTRO="$ROS_DISTRO_ARG"` (default jazzy, `launch.sh:23`);
  - **`launch.sh:55` `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, hard-coded.** It overrides
    whatever the environment says;
  - `launch.sh:57` sets `FASTRTPS_DEFAULT_PROFILES_FILE`, defaulting to
    `/home/isaac_ws/config/fastdds_udp.xml`. `--dds-profile` mounts the profile's directory at
    `/dds` (`launch.sh:41-45`, compose `:45`);
  - `launch.sh:59-61` passes `ROS_DISCOVERY_SERVER`, `ROS_AUTOMATIC_DISCOVERY_RANGE`,
    `ROS_STATIC_PEERS` and `ROS_SUPER_CLIENT` through, dropping empty ones;
  - `launch.sh:62-65` puts `/isaac-sim/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib` on
    `LD_LIBRARY_PATH` after removing the other distro's lib path;
  - `launch.sh:67` strips `/opt/ros` from `PYTHONPATH` ("Isaac uses Python 3.11").
- **rclpy comes from the internal libs:** `sim/app.py:100-108` `_ensure_rclpy_path()` appends
  `/isaac-sim/exts/isaacsim.ros2.bridge/{ROS_DISTRO}/rclpy` to `sys.path` and removes `/opt/ros`
  entries. It is called from `create_app()` at `app.py:40`, after
  `enable_extension("isaacsim.ros2.bridge")` at `app.py:13-14,36-37`. The internal Jazzy rclpy is
  **7.1.5** (`rclpy-7.1.5-py3.11.egg-info`); apt Jazzy is 7.1.11 installed or 7.1.12 candidate.
  Isaac's Python is **3.11.13**.
- **The ROS interface is pure rclpy, not OmniGraph:** `sim/ros_bridge.py:54-118` creates a
  node, about 15 publishers, the `/ackermann_drive` subscriber and the `/sim/reset` Trigger service.
  `spin_once` runs between physics steps (`ros_bridge.py:135-136`). QoS comes from
  `config/ros_interface.yaml:50-60` (sensors RELIABLE depth 5; tf_static and objects
  TRANSIENT_LOCAL). The module docstring at `ros_bridge.py:1` still says "internal Humble libs",
  which is stale because Jazzy is the default.
- `config/ros_interface.yaml:4` `rmw: rmw_fastrtps_cpp` is **informational only**: `sim/config.py:84`
  reads only `ros.domain_id`/`ros.node_name`, and nothing reads `ros.rmw`. Domain 42 is at
  `ros_interface.yaml:3`.
- The Fast DDS profile `config/fastdds_udp.xml` is UDPv4 only, with no SHM and 8 MB socket
  buffers.
- **RMWs shipped in the image (verified by `ls`):**
  - `jazzy/lib`: `librmw_fastrtps_cpp.so` (Fast DDS **2.14.5**), `librmw_fastrtps_dynamic_cpp.so`,
    `librmw_cyclonedds_cpp.so` (Cyclone `libddsc.so.0.10.5`);
  - `humble/lib`: the same, with Fast DDS 2.6.10;
  - **no `librmw_zenoh_cpp`, no `libzenohc`**, no `libtracetools.so` (only `libtracetools_status`),
    and no `share/` ament index (the jazzy dir has only `lib/` and `rclpy/`).

  The bridge's own help text offers only FastDDS or CycloneDDS
  (`isaacsim/ros2/bridge/impl/ros2_common.py:83-94`). Bridge CHANGELOG 4.9.3 (2025-07-29):
  "CycloneDDS is now supported for Jazzy".

#### B2. What NVIDIA documents

- **Isaac Sim 5.1.0 "ROS 2 Installation"**
  (https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html; the banner says
  "Isaac Sim 5.1.0 is no longer supported"):
  - The page contains **zero occurrences of "zenoh"** (the full page text was checked).
  - "Isaac Sim is compatible with Python 3.11 only."
  - "The ROS_DISTRO env variable is used to determine if ROS 2 is sourced and which distro to use.
    If this variable is not set, an internal ROS 2 distro build is used."
  - "only source the Isaac Sim internal libraries or source your ROS 2 installation if it is built
    with Python 3.11".
  - "The internal libraries are in Python 3.11, so attempting to source the default installation
    of ROS results in errors." System Jazzy on 24.04 is Python 3.12.
  - "Isaac Sim ROS 2 Humble and Jazzy internal libraries include Cyclone DDS compiled with Python
    3.11", used via `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
  - Custom packages require building a Python 3.11 ROS workspace (IsaacSim-ros_workspaces tag
    `IsaacSim-5.1.0`, `dockerfiles/ubuntu_24_jazzy_python_311_minimal.dockerfile`). That
    Dockerfile's `rosinstall_generator` list (line 167) has no rmw_zenoh.
- **Isaac Sim 6.0.1 and 6.2.0** (same page path under `/6.0.1/` and `/6.2.0/`), section
  **"Enabling the ROS 2 Bridge > On Linux using Zenoh"**:
  - "Isaac Sim supports RMW Zenoh middleware for Linux and ROS 2 Jazzy."
  - "Note: Currently, Isaac Sim Zenoh interfaces do not come packaged with internal libraries.
    Please install RMW Zenoh on your system and source it before running Isaac Sim."
    The procedure: `sudo apt install ros-jazzy-rmw-zenoh-cpp`, `ros2 run rmw_zenoh_cpp rmw_zenohd`,
    `export RMW_IMPLEMENTATION=rmw_zenoh_cpp`, `source /opt/ros/jazzy/setup.bash`,
    `./isaac-sim.sh`.
  - 6.2.0 adds that "Linux bundled libraries do not include Zenoh; install or build it" and "Zenoh
    with Humble on Ubuntu 22.04 is untested".
  - This works because Isaac 6.x moved to **Python 3.12** ("compiled with Python 3.12"), so the
    system Jazzy rclpy can be sourced.

#### B3. Verdict and alternatives

**Can Isaac 5.1's bridge in this setup run rmw_zenoh_cpp? Not supported, and not without a custom
build.** The internal libs ship no Zenoh, the 5.1 docs do not mention Zenoh, and the system Jazzy
(Python 3.12, apt rmw_zenoh) cannot be sourced into Isaac 5.1's Python 3.11.

Options, from most to least practical:

1. **Upgrade the sim to Isaac Sim 6.x.** Zenoh with system Jazzy is the documented path there. It
   costs a 6.x image and a check of API changes in `sim/`. The bridge extension layout also
   changed: 6.2 docs reference `exts/isaacsim.ros2.core/<distro>/lib`. `launch.sh:55,62` and
   `app.py:100-108` would change to "source /opt/ros/jazzy, RMW_IMPLEMENTATION=rmw_zenoh_cpp".
   rmw_zenoh would then need to be added to the image (apt), along with a router.
2. **Stay on 5.1 and build a Python 3.11 Jazzy workspace that includes `rmw_zenoh_cpp` and
   `zenoh_cpp_vendor`**, using the 5.1.0 ros_workspaces Dockerfile with rmw_zenoh added to the
   source set. zenoh-c needs Rust: cargo ≥1.88 for the default commit, or the vendor's ≥1.75
   fallback commit. Source it instead of the internal libs, the route the 5.1 docs allow ("source
   your ROS 2 installation if it is built with Python 3.11"). Pin the **same rmw_zenoh tag (0.2.11)
   as the Thor**. This is feasible but undocumented by NVIDIA, and it means rebuilding a ROS
   workspace in the image.
3. A hack: drop the apt `librmw_zenoh_cpp.so`, `libzenohc.so` and `libtracetools.so` next to the
   internal libs, and point `AMENT_PREFIX_PATH` at a prefix containing `share/rmw_zenoh_cpp/config`.
   rmw_zenoh needs that for its default configs (`zenoh_config.cpp:121-124`,
   `ament_index_cpp::get_package_share_directory("rmw_zenoh_cpp")`). This mixes apt-Jazzy binaries
   (built against rmw 7.3.4 and rcutils from 2026-09) with NVIDIA's older internal Jazzy build
   (rclpy 7.1.5). **ABI risk; not recommended.** CLAUDE.md "Rebuild policy" shows a real
   in-distro ABI break.
4. **Keep HIL on Fast DDS** (the status quo) and use rmw_zenoh only for scenarios (a) and (b). This
   is the zero-risk option if HIL runs on a dedicated LAN. It means the Thor stack runs on a
   different RMW in HIL mode than on the car.

**zenoh-bridge-ros2dds (eclipse-zenoh/zenoh-plugin-ros2dds) is NOT a solution for "Isaac on DDS ↔
Thor on rmw_zenoh".**
- What it does: it discovers DDS ROS 2 entities on its host (it is built on **CycloneDDS**, tested
  with `rmw_cyclonedds_cpp`; other vendors "are interoperable over UDP multicast and unicast" but
  "non-standard features… (e.g. shared memory) might cause some issues") and routes them over
  Zenoh. Services and actions map to queryables. It runs as a router by default since v0.11.0,
  listening on `tcp/0.0.0.0:7447`. The latest release is **1.10.1** (2026-09-07). Sources:
  https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds README and `DEFAULT_CONFIG.json5`.
- **Incompatible with rmw_zenoh, by the rmw_zenoh maintainers' statement** (README section "On
  interoperability with eclipse-zenoh/zenoh-plugin-ros2dds and zenoh-bridge-ros2dds"):
  "`rmw_zenoh` utilizes Zenoh differently, particularly in terms of key expressions. Consequently,
  `rmw_zenoh` cannot interoperate with `zenoh-plugin-ros2dds`. This use case is beyond the scope of
  `rmw_zenoh`."
  - rmw_zenoh keys are `<domain>/<topic>/<type>/<RIHS01_hash>`, with liveliness
    `@ros2_lv/<domain>/…` and its own attachment and serialization design.
  - ros2dds keys are the ROS name (with an optional `namespace` prefix), and its liveliness and
    admin space live under `@/<zid>/ros2/**`.
  - Issue https://github.com/ros2/rmw_zenoh/issues/522 (open): JEnoch says the router "is unaware
    of how rmw_zenoh structures its key expressions".
  - The discourse thread "RMW-RMW bridge - is it possible" (2025-08 to 2026-03,
    https://discourse.openrobotics.org/t/rmw-rmw-bridge-is-it-possible-has-anyone-done-it/49564)
    mentions a private, unreleased ros2dds fork that rewrites keys to the rmw_zenoh format. There
    is no released DDS↔rmw_zenoh bridge. Fast DDS on the ros2dds side is also unsupported
    ("This only works with cyclone dds", ros2dds issue #492).
- The only valid use of ros2dds here is **DDS ↔ Zenoh ↔ DDS**: a bridge next to Isaac on the
  workstation **and** a bridge on the Thor, with the Thor stack on DDS. It would replace Fast DDS
  discovery-server/UDP across the LAN with a filtered Zenoh TCP link. For that case only (not
  rmw_zenoh), here is a config sketch for the **workstation** bridge.

  It assumes:
  - Isaac on domain 42 with the UDP-only Fast DDS profile. That suits DDS-vendor interop, because
    Fast DDS SHM is already off.
  - `ROS_DISCOVERY_SERVER` **unset** for Isaac. Cyclone cannot talk to a Fast DDS Discovery Server,
    so simple discovery on localhost must be used.
  - A port that differs from any rmw_zenohd on the same host.

```json5
// zenoh-bridge-ros2dds -c ws_bridge.json5   (zenoh-bridge-ros2dds 1.10.1)
{
  mode: "router",
  listen:  { endpoints: ["tcp/0.0.0.0:7448"] },          // 7447 may be taken by a router
  connect: { endpoints: ["tcp/<thor-lan-ip>:7448"] },     // the Thor-side bridge
  plugins: {
    ros2dds: {
      nodename: "zenoh_bridge_isaac",
      domain: 42,                                         // Isaac's ROS_DOMAIN_ID
      ros_automatic_discovery_range: "LOCALHOST",         // only Isaac on this host; avoids DDS loops
      // allow: full-name regexes; an unlisted kind = none of that kind allowed.
      // The roles are those of the DDS entities on THIS host (Isaac):
      allow: {
        publishers: [
          "/clock", "/scan", "/odom", "/sensors/imu/raw", "/tf", "/tf_static",
          "/zed2/zed_node/left/(image_rect_color|camera_info)",
          "/zed2/zed_node/right/(image_rect_color|camera_info)",
          "/zed2/zed_node/depth/(depth_registered|camera_info)",
          "/camera/(image_raw|camera_info)",
          "/sim/ground_truth/.*",
        ],
        subscribers:     ["/ackermann_drive"],             // Isaac subscribes, the Thor publishes
        service_servers: ["/sim/reset"],                  // Isaac serves, Thor scripts call
      },
      pub_max_frequencies: [".*/image_rect_color=15", "/camera/image_raw=15", ".*/depth_registered=15"],
      pub_priorities: ["/ackermann_drive=2:express", "/clock=3", "/tf=3"],
      reliable_routes_blocking: true,                     // RELIABLE writers -> CongestionControl::Block
      transient_local_cache_multiplier: 10,               // /tf_static and the latched ground-truth topics
      queries_timeout: { default: 5.0, services: ["/sim/reset=30.0"] },  // the 5 s default may be short for a reset
    },
  },
}
```

  The Thor-side bridge mirrors this. Its `allow` lists the Thor's roles: a `publishers:
  ["/ackermann_drive"]` entry, `subscribers` for the sensor topics, and `service_clients:
  ["/sim/reset"]`. It sets `domain` to the Thor's domain, and `connect`/`listen` are swapped.
  Option names are verified against `DEFAULT_CONFIG.json5` (allow/deny regex semantics, "cannot set
  both", `pub_max_frequencies "<regex>=<float>"`, `pub_priorities "<regex>=<1-7>[:express]"`,
  `queries_timeout` default 5.0 s, `transient_local_cache_multiplier`). The sketch itself is
  **untested**.

**Bottom line for (c):**
- With Isaac 5.1, rmw_zenoh on the Thor in HIL means either option 2 (a custom Python 3.11 Jazzy
  build with rmw_zenoh) or option 1 (Isaac 6.x).
- ros2dds cannot bridge to rmw_zenoh.
- A homegrown relay (Fast DDS process ↔ IPC ↔ Zenoh process; one process cannot load two RMWs
  through rclcpp/rclpy) is possible but is new code to maintain.

---

#### Source index (primary)
- rmw_zenoh README (jazzy): https://github.com/ros2/rmw_zenoh/blob/jazzy/README.md
- Design doc: https://github.com/ros2/rmw_zenoh/blob/jazzy/docs/design.md
- 0.2.11 tag files: `zenoh_cpp_vendor/CMakeLists.txt`, `rmw_zenoh_cpp/CHANGELOG.rst`, `rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_{SESSION,ROUTER}_CONFIG.json5`, `rmw_zenoh_cpp/src/{rmw_event.cpp,detail/qos.cpp,detail/rmw_publisher_data.cpp,detail/rmw_subscription_data.cpp,detail/rmw_client_data.cpp,detail/zenoh_config.cpp,detail/rmw_context_impl_s.cpp,rmw_init.cpp}` at https://github.com/ros2/rmw_zenoh/tree/0.2.11
- Issues: rmw_zenoh #978, #976, #696, #672, #710, #783, #1029, #1046, #912, #764, #763, #637, #733, #853, #242, #522, #758, #1003; PRs #1005, #1036/#1040, #1071. rosbag2 #2463, #1936. navigation2 #6163, #6241.
- ROS docs (jazzy sources): `Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.rst`, `How-To-Guides/Working-with-multiple-RMW-implementations.rst`, `Concepts/Intermediate/About-Different-Middleware-Vendors.rst` (github.com/ros2/ros2_documentation, branch jazzy; docs.ros.org returns an Anubis bot wall to fetchers).
- tf2_ros QoS: https://github.com/ros2/geometry2/blob/jazzy/tf2_ros/include/tf2_ros/qos.hpp
- Stereolabs: https://docs.stereolabs.com/docs/integrations/ros-2/using-zenoh-as-middleware
- Isaac Sim: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html ; https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_ros.html (section "On Linux using Zenoh") ; https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/install_ros.html ; https://github.com/isaac-sim/IsaacSim-ros_workspaces (tag IsaacSim-5.1.0)
- zenoh-plugin-ros2dds: https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds (README, DEFAULT_CONFIG.json5, releases 1.10.1)
- Discourse: https://discourse.openrobotics.org/t/rmw-rmw-bridge-is-it-possible-has-anyone-done-it/49564 ; https://discourse.openrobotics.org/t/rmw-zenoh-router-configuration/43885
