# Phase 2 — Live measurement: real rates

**Run 2 (2026-09-24, 17:08–17:55), camera working.** This document replaces the
first run entirely; that one measured a stack whose ZED had failed to open.

Stack: `ros2 launch f1tenth_bringup supervisor_bringup.launch.py`, all
defaults, car stationary, **no mission loaded**, nothing published to any
actuation topic at any point.

---

## 0. Pre-flight

### 0.1 ZED USB link — verified at both ends of the run

| When | `lsusb -t` | `/sys/.../speed` |
|---|---|---|
| Before launch | Bus 02, `Class=Video, Driver=uvcvideo, 5000M` | `2-2 speed=5000` |
| After 46 min | unchanged | `2-2 speed=5000` |

**No fallback to USB 2.0.** The ZED container ran for **2794 s** with **zero**
`CAMERA NOT DETECTED` / `process has died` entries in its component log. The
separate `ZED-2i HID INTERFACE` sits on USB 2 at 12 Mb/s, which is normal — it
is a HID endpoint, not the video path.

The camera opened cleanly:

```
[Init]  Camera successfully opened.
[Init]  Serial Number: S/N 36866199
[WARN]  Camera model does not match user parameter.
        Please modify the value of the parameter 'general.camera_model' to 'zed2i'
        * Camera Model  -> ZED 2i
```

This settles Phase 1's open question 7: the SDK reads the real model off the
device and proceeds correctly. `camera_model: 'zed2'` is a **warning-level
cosmetic mismatch**, not a functional one.

### 0.2 The `ros2` CLI — fixed, and here is what was actually wrong

Run 1 concluded the CLI was unusable here. **That conclusion was wrong, and so
was the reason given for it.** Fast DDS **2.6.11 does support**
`ROS_SUPER_CLIENT` — `strings libfastrtps.so.2.6.11` contains both
`ROS_SUPER_CLIENT` and `eprosima::fastdds::rtps::ros_super_client_env()`, and
the repo already documents this as the confirmed fix for the "topic list shows
only 2 topics" symptom (`foxglove_bridge.launch.py:86-111`, `~/.bashrc:132-147`).

Two real causes, found this run:

1. **A stale `ros2` daemon** (pid 13323) left over from the previous session was
   serving a cached, empty graph to every CLI call.
2. **The daemon never becomes a super client even when it inherits the
   variable.** Verified directly: `/proc/<daemon>/environ` contains
   `ROS_SUPER_CLIENT=TRUE` and `ROS_DISCOVERY_SERVER=127.0.0.1:11811`, and the
   daemon still reports **2 topics, 0 nodes**. A `SUPER_CLIENT` XML profile via
   `FASTRTPS_DEFAULT_PROFILES_FILE` does not fix the daemon either — also
   tested, also 2 topics.

**Working recipe, used for every measurement in this document:**

```bash
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_SUPER_CLIENT=TRUE
ros2 daemon stop                                   # must stay stopped
ros2 node list  --no-daemon --spin-time 12         # 66 nodes
ros2 topic list --no-daemon --spin-time 12         # 238 topics
ros2 topic info <t> --verbose --no-daemon --spin-time 8
ros2 topic hz <t> --spin-time 6                    # no --no-daemon flag exists;
ros2 topic bw <t> --spin-time 6                    # works because daemon is stopped
```

`hz` and `bw` have **no `--no-daemon` flag**. They work only because the daemon
is stopped, which makes `ros2cli` fall back to a direct node. If a daemon is
running, both hang until timeout.

### 0.3 Measurement caveat — the CLI under-reports on a saturated box

This car idles at **85 % CPU** with the camera running (§2). Under that load the
CLI is not a trustworthy rate source:

- `ros2 topic hz /zed2/zed_node/depth/depth_registered` → **23.36 Hz**, while
  `detection_3d_node`'s own internal counter over the same period says
  **28.6 Hz**.
- `ros2 topic info` intermittently returned `Publisher count: 0` for topics
  publishing at 50 Hz (`/sensors/core`), and returned nothing at all for
  `/sensors/imu`.

So this document uses three sources and says which is which:

| Source | Used for |
|---|---|
| `ros2 topic hz`/`bw` | headline CLI numbers, treated as a **lower bound** |
| **node-internal counters** (`detection_3d_node` 10 s YIELD report) | authoritative rates for the camera chain — costs no extra subscriber |
| **rclpy graph snapshot** | authoritative publisher/subscriber counts |

Exact message sizes were read once off a single message each, so
`size × counter-rate` gives the best bandwidth estimate.

---

## 1. Graph scale

| Metric | Run 1 (camera dead) | **Run 2 (camera up)** |
|---|---|---|
| Nodes | 66 | **68** |
| Topics | 125 | **238** |
| Publishers | — | **376** |
| Subscribers | — | **207** |
| Endpoints | — | **583** |

**The ZED advertises 111 topics by itself** — 47 % of the whole graph. Of those,
**62 are `image_transport` plugin variants** (`compressed` 17,
`compressedDepth` 15, `ffmpeg` 15, `theora` 15). Nothing in this stack uses any
of them. They are lazy (no data until subscribed) but they are **advertised**,
so every one is a discovery endpoint.

All components started clean. No node crashed or restarted this run.

---

## 2. Load baseline — 70 s, idle, **no subscribers of mine**

`tegrastats`, 52 samples, nothing of mine attached to any topic:

| Metric | Run 1 (camera dead) | **Run 2 (camera up)** |
|---|---|---|
| CPU, mean of 12 cores | 44.2 % | **85.2 %** |
| CPU, max | 47.0 % | **92.8 %** |
| GPU `GR3D_FREQ`, mean | 0 % | **59.0 %** |
| GPU, max | 0 % | **99 %** |
| RAM | 9 930 MB | **11 604 MB** |

**The car is near CPU saturation standing still.** That single fact reframes the
whole exercise: the first run's baseline was not a quiet stack, it was a stack
missing its two most expensive consumers.

During my measurements the same figures read CPU **89.4 %**, GPU **54.7 %**,
RAM 11 633 MB — so the measurement overhead is about **4 points of CPU**, and
the idle baseline above is clean.

### Per-process CPU, idle, no subscribers

| Process | mean %CPU | max %CPU |
|---|---|---|
| `mpc_corr` | **68.2** | 68.7 |
| `yolo_detector_node` | **61.5** | 62.3 |
| `costmap_boundary_node` | **53.3** | 53.6 |
| `component_container` (ZED + viz relays) | 51.7 | 67.9 |
| `foxglove_bridge` | 51.3 | 51.5 |
| `detection_3d_node` | 47.8 | 48.2 |
| `ekf_node` (×2) | 43.2 | 44.1 |
| `behavior_executor_node` | 42.3 | 42.6 |
| **`fast-discovery-server`** | **41.1** | 44.3 |
| `semantic_layer_node` | 40.4 | 40.7 |
| `wall_distance_node` | 40.2 | 40.4 |
| `reset_manager` | 34.8 | 34.9 |
| `obstacle_projector_node` | 33.9 | 34.1 |
| `ekf_cost_observer_node` | 30.1 | 30.3 |
| `lidar_front_wall_node` | 25.3 | 25.4 |
| `robot_state_publisher` | 24.8 | 25.9 |
| `front_clearance_node` | 22.6 | 22.7 |

---

## 3. The camera chain — authoritative rates

From `detection_3d_node`'s own 10-second YIELD report, over 14 consecutive
windows. This costs no extra subscriber and is the best number available.

```
YIELD | det2d_in=90 depth_in=287 mask_in=90 -> synced=90 (100%) published=90 (100%)
      | frame drops: sync=0 no_camera_info=0 cv_bridge=0 tf2=0
      | detection drops: low_conf=0 bad_depth=0 mask_fallback=0
```

| Stream | per 10 s window | **rate** |
|---|---|---|
| `/camera/detections` (`det2d_in`) | 73 – 100, mean ≈ 89 | **≈ 8.9 Hz** |
| `/camera/detection_masks` (`mask_in`) | tracks det2d | **≈ 8.9 Hz** |
| ZED depth (`depth_in`) | 277 – 294, mean ≈ 286 | **≈ 28.6 Hz** |
| sync yield | 100 % every window | no drops |

**YOLO is the bottleneck, and the ZED is not.** Depth arrives at ~28.6 Hz
against a configured `pub_frame_rate` of 30.0 — the camera is keeping up.
Detections come out at ~8.9 Hz. The model is `yolo26s-seg.pt`,
`yolo_model_task: segment`, on `yolo_device: cuda` — a **segmentation** model,
which is why `yolo_detector_node` sits at 61.5 % CPU *and* drives the GPU to
59 % mean.

So the wrapper produces **about 3.2 frames for every one YOLO consumes**, and
`detection_3d_node` reports zero sync drops, meaning the surplus is discarded
silently at the subscriber.

### Exact message sizes (read once off a live message)

| Topic | Dimensions | Encoding | Bytes/msg |
|---|---|---|---|
| `/camera/image_raw` | 640 × 360 | **`bgra8`** | **921 600** |
| `/zed2/…/depth/depth_registered` | 640 × 360 | `32FC1` | 921 600 |
| `/camera/detection_masks` | 640 × 360 | `mono8` | 230 400 |
| `/camera/image_annotated` | 640 × 360 | `bgr8` | 691 200 |

`/camera/image_raw` is **BGRA8** — four bytes per pixel with an alpha channel
nothing uses. That is 230 kB per frame, 25 % of the stream, carried for nothing.

### Best-estimate camera bandwidth

`size × authoritative rate`, not the CLI's under-read:

| Topic | Rate | **MB/s** | Real subs |
|---|---|---|---|
| `/camera/image_raw` | ~28.6 Hz | **≈ 25.7** | 2 (yolo + lazy viz throttle) |
| ZED depth | ~28.6 Hz | **≈ 25.7** | **3** |
| `/camera/detection_masks` | ~8.9 Hz | ≈ 2.0 | 2 |
| `/camera/image_annotated` | ~4.9 Hz | ≈ 3.4 | 2 (both viz) |

**≈ 57 MB/s of camera traffic**, against ~0.35 MB/s for the lidar.

---

## 4. Measured rates — full CLI table

`ros2 topic info -v` taken **before** each subscription; `hz` then `bw`, one
topic at a time, never concurrently. Subscriber counts below are the
**authoritative rclpy snapshot** taken after all measuring processes exited.

### 4.1 Race-critical

| Topic | CLI Hz | CLI kB/s | pub | **sub** |
|---|---|---|---|---|
| `/tf` | 102.70 | 15.4 | 7 | 10 |
| `/odometry/filtered` | 50.01 | 35.4 | 1 | 8 |
| `/odom` | 50.00 | 36.2 | 1 | 8 |
| `/sensors/imu/raw` | 50.00 | 15.9 | 1 | 2 |
| `/sensors/core` | 50.00 | 9.9 | 1 | 3 |
| `/ekf_global/odometry/filtered` | 38.76 | 30.7 | 1 | 6 |
| `/scan` | 40.00 | **347.5** | 1 | **8** |
| `/drive` | 10.00 | 0.4 | 2 | 2 |
| `/ackermann_drive` | 9.99 | 0.4 | 1 | 2 |
| `/commands/motor/speed` | 10.00 | 0.1 | 1 | 1 |

`/ekf_global/odometry/filtered` reading 38.76 Hz against a configured 50.0 is a
**CPU-starvation artifact of the measuring process**, not a real drop — see §0.3.

### 4.2 Camera chain (CLI figures; §3 has the authoritative ones)

| Topic | CLI Hz | CLI kB/s | pub | **sub** |
|---|---|---|---|---|
| `/camera/image_raw` | 23.51 | 18 452 | 1 | 2 |
| `/camera/camera_info` | 28.23 | 12.0 | 1 | **0** |
| `/camera/detections` | 9.52 | 5.5 | 1 | **4** |
| `/camera/detection_masks` | 10.07 | 2 243 | 1 | 2 |
| `/camera/image_annotated` | 4.90 | 3 523 | 1 | 2 |
| `/camera/detections_3d` | 10.09 | 5.2 | 1 | 2 |
| `/camera/detection_markers` | 10.93 | 5.7 | 1 | **0** |
| `/zed2/…/depth/depth_registered` | 23.36 | 19 272 | 1 | **3** |
| `/zed2/…/depth/camera_info` | 29.08 | 12.2 | 1 | 2 |

### 4.3 ZED topics with **zero** subscribers — read §4.4 before believing these

| Topic | CLI Hz | CLI kB/s | pub | **sub** |
|---|---|---|---|---|
| `/zed2/…/confidence/confidence_map` | 20.17 | **22 907** | 1 | **0** |
| `/zed2/…/left/image_rect_color` | 25.39 | **25 487** | 1 | **0** |
| `/zed2/…/disparity/disparity_image` | 22.70 | **21 248** | 1 | **0** |
| `/zed2/…/point_cloud/cloud_registered` | 9.36 | **17 705** | 1 | **0** |
| `/zed2/…/imu/data` | **200.56** | 62.9 | 1 | **0** |
| `/zed2/…/imu/data_raw` | **201.85** | 67.9 | 1 | **0** |
| `/zed2/…/depth/depth_info` | — | — | 1 | **0** |

### 4.4 These are observer effect — and that matters

**Every one of those rates was caused by my own measurement.** The ZED wrapper
gates publication on subscriber count. Verified in its source:

```cpp
// zed_camera_component.cpp:6436
imu_SubCount     = count_subscribers(mPubImu->get_topic_name());
imu_RawSubCount  = count_subscribers(mPubImuRaw->get_topic_name());
...
if (imu_SubCount > 0)  { ... }      // line 6553
if (imu_RawSubCount > 0) { ... }    // line 6626
```

With nobody subscribed these topics carry **no data**. The numbers above are
therefore **"what this costs the moment anything subscribes"** — which is not
hypothetical: opening one of them in Foxglove is a single click, and the point
cloud alone is **17.7 MB/s**.

The IMU is the interesting case, and it exposes a real bug — see §5.

---

## 5. A disabled feature that was never disabled

`zed2_perception.yaml` contains:

```yaml
    sensors:
      publish_imu: false
      publish_imu_raw: false
      publish_imu_tf: false
```

with a comment explaining that the ROS IMU publish "up to sensors_pub_rate=200Hz
was pure overhead with zero consumers".

**Two of those three parameters do not exist.** Queried live on the running
node:

```
sensors.publish_imu       ->  Parameter not set
sensors.publish_imu_raw   ->  Parameter not set
```

`ros2 param list /zed2/zed_node` confirms the wrapper's `sensors:` block
declares exactly three keys, and `common_stereo.yaml:47-50` shows the same:

```yaml
        sensors:
            publish_imu_tf: true
            sensors_image_sync: false
            sensors_pub_rate: 200.     # MAX 400, MIN grab rate
```

There is **no `publish_imu` / `publish_imu_raw` in this wrapper version.** The
two lines are silently ignored — YAML the wrapper never reads. The measured
**200.56 Hz / 201.85 Hz** is exactly `sensors_pub_rate: 200.` doing what it is
configured to do.

The bandwidth happens to be avoided anyway, by the subscriber gate in §4.4. What
is **not** avoided is the sensor thread running at 200 Hz — retrieving SDK
sensor data and calling `count_subscribers()` on four topics, 200 times a
second, forever, for nobody.

By contrast, these overrides **do** apply (verified live):

| Parameter | Live value |
|---|---|
| `general.pub_frame_rate` | 30.0 ✓ |
| `general.pub_downscale_factor` | 2.0 ✓ |
| `depth.depth_stabilization` | 0 ✓ |
| `pos_tracking.pos_tracking_enabled` | False ✓ |
| `object_detection.od_enabled` | False ✓ |

---

## 6. Perception, control, viz

| Topic | Hz | kB/s | pub | **sub** |
|---|---|---|---|---|
| `/perception/lidar_front_wall` | 39.97 | 2.4 | 1 | **0** |
| `/obstacle_clearance` | 39.77 | 0.3 | 1 | **0** |
| `/perception/swept_clearance` | 20.00 | 0.2 | 1 | **0** |
| `/perception/swept_clearance/camera` | 16.45 | 0.1 | 1 | **0** |
| `/costmap/boundaries` | 20.02 | 2.2 | 1 | 1 |
| `/perception/obstacles_2d` | 9.83 | 0.3 | 1 | 3 |
| `/perception/front_distance` | 26.30 | 0.2 | 1 | 2 |
| `/perception/front_clearance` | — | — | 1 | **1** |
| `/perception/d_wall` | 8.62 | 0.5 | 1 | 1 |
| `/safety/event` | — | — | 2 | **0** |
| `/costmap/visualization` | 2.00 | **208.4** | 1 | **0** |
| `/bt/tree_visualization` | 2.24 | **230.9** | 1 | **0** |
| `/behavior/tree_status` | 7.84 | 1.0 | 1 | **0** |
| `/slam/map` | — | 13.2 | 1 | 4 |
| `/diagnostics` | 11.32 | 7.8 | 6 | 1 |

`/bt/tree_visualization` and `/costmap/visualization` are **not** lazy — they
publish unconditionally from their own nodes, and together account for
**439 kB/s with zero subscribers**, confirmed again this run.

---

## 7. `fast-discovery-server` — the CPU has a specific shape

Measured over the clean 70 s idle window, no subscribers of mine: **41.1 % mean,
44.3 % max.**

`top -H` on the process:

| Thread | %CPU | State | CPU time |
|---|---|---|---|
| **29764** | **72.3** | **R** | 4:39.79 |
| 29763 | 0.7 | S | 0:25.81 |
| 29760 | 0.3 | S | 0:03.40 |
| other 7 threads | 0.0 | S | ≤ 0:02 |

**One thread out of ten is spinning**; the rest are idle. This is not spread
network I/O — it is a single hot loop.

Context that plausibly drives it:

- **583 endpoints** across **68 participants**, 238 topics.
- **111 of those topics are the ZED's**, and **62 are unused image_transport
  variants** that exist purely as discovery endpoints.
- **No reconnect or respawn loop**: `ensure_discovery_server.py` logged
  `127.0.0.1:11811 is free -- starting a fresh fastdds discovery server` exactly
  once, and the process ran 46 minutes without restarting.
- Two super clients (`foxglove_bridge`, the viz relay container) receive the
  entire graph's endpoint data relayed to them.

I have **not** proven which loop it is; identifying it needs a profiler or a
Fast DDS build with discovery tracing. What is established is that it is one
thread, it is CPU-bound rather than I/O-blocked, and the endpoint count it is
servicing is inflated by ~62 topics nothing uses.

---

## 8. Shutdown

`SIGINT` to the top-level `ros2 launch` did **not** propagate — its children
survived. `SIGINT` to `component_supervisor_node` shut every component down
correctly. Afterwards: **0 stack processes**, discovery server and `ros2`
daemon stopped, port 11811 free. Machine returned to its pre-run state.

---

## 9. Still needs verification while driving

1. Every `/mpc/goal_*`, `/corridor`, `/safety_stop` topic — idle without a mission.
2. `/slam/map` rate — a stationary car gives slam_toolbox nothing to map.
3. **Whether BT-side subscriber counts rise once a mission runs.** BT behaviours
   create their subscriptions at tree-build time, so `sub=0` on a mission-less
   car can understate the real consumer set. `/perception/front_clearance`
   already showed this: 0 subs in run 1, 1 sub now.
4. Whether a ZED TF edge re-enters `/tf` under load (7 publishers now, vs 6).
5. YOLO's rate under real scene complexity — 8.9 Hz was measured on a static
   indoor scene.
