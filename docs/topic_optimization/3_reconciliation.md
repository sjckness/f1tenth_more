# Phase 3 — Reconciling configured vs. ideal vs. measured

Rewritten against **run 2** (camera working, 2026-09-24). Where run 1 or Phase 1
was wrong, the correction is stated rather than edited away.

---

## 1. The three corrections that matter most

### 1.1 A ZED override that was never in effect

`zed2_perception.yaml` sets `sensors.publish_imu: false` and
`sensors.publish_imu_raw: false`, with a comment saying this removes "up to
sensors_pub_rate=200Hz" of "pure overhead with zero consumers".

**Those parameter names do not exist in this wrapper version.** Live query
returns `Parameter not set` for both, and `common_stereo.yaml:47-50` declares
only `publish_imu_tf`, `sensors_image_sync` and `sensors_pub_rate`. The two
lines are inert YAML.

The *bandwidth* is avoided anyway, but by a different mechanism than the comment
claims — the wrapper's own `if (imu_SubCount > 0)` gate
(`zed_camera_component.cpp:6553`). What survives is the **sensor thread running
at `sensors_pub_rate: 200.` Hz**, polling the SDK and calling
`count_subscribers()` on four topics 200 times a second for nobody.

This is the clearest instance of a pattern worth naming: **this config file has
been trusted rather than verified.** Every other override in it *was* verified
live this run and does apply.

### 1.2 The camera is not the bottleneck; YOLO is

| | Configured | Measured (authoritative) |
|---|---|---|
| ZED publish rate | `pub_frame_rate: 30.0` | **28.6 Hz** — keeping up |
| YOLO output | — | **8.9 Hz** |

Run 1 could not measure this at all. The oversupply ratio is **3.2:1**, and
`detection_3d_node` reports **zero sync drops**, so the surplus is being
produced, transported, and silently discarded at the subscriber.

### 1.3 "Zero subscribers" was measured on a mission-less car

BT behaviours build their subscriptions when the tree is built, not at node
start. `/perception/front_clearance` read **0 subscribers in run 1** and **1
now**, without any code change. So a `sub=0` reading here is evidence, not
proof, and every "unused" verdict below was re-checked with a workspace-wide
source grep rather than resting on the count.

---

## 2. Reconciliation table

| Topic | Configured | Ideal | Measured | Explanation | Gap |
|---|---|---|---|---|---|
| `/camera/image_raw` | 30 Hz | **≈ 9 Hz** (= YOLO) | 28.6 Hz, **25.7 MB/s** | `pub_frame_rate: 30.0` applies and the camera achieves it. Its only functional consumer runs at 8.9 Hz. 3.2× oversupply. Also **BGRA8** — a 4th byte per pixel nothing reads. | **Yes — largest** |
| ZED depth | 30 Hz | **≈ 9 Hz**, or 20 Hz while `swept_clearance` exists | 28.6 Hz, **25.7 MB/s** | Same `pub_frame_rate`. Three subscribers: `detection_3d_node` (syncs to the 8.9 Hz detection stream), `front_clearance_node` (latest-at-detection), `swept_clearance_node` (20 Hz timer — **whose own output has no consumer**). Drop `swept_clearance` and the fastest genuine need becomes ~9 Hz. | **Yes** |
| ZED `imu/data`, `imu/data_raw` | intended off; **override inert** | off | 200.6 / 201.9 Hz **when subscribed**, 0 otherwise | §1.1. `sensors_pub_rate: 200.` is the live value. | **Yes (config bug)** |
| ZED point cloud / confidence / disparity / `left/image_rect_color` | subscriber-gated | disabled | 9–25 Hz, **17–25 MB/s each**, only because I subscribed | Lazy publishers. Idle in normal operation, but one Foxglove click away from 17–25 MB/s. | Hazard, not a live gap |
| 62 ZED `image_transport` variants | advertised | not advertised | lazy, 0 Hz | `compressed`/`compressedDepth`/`ffmpeg`/`theora`. No data, but 62 discovery endpoints out of 583. | **Yes (discovery cost)** |
| `/camera/detections` | callback | = YOLO | 9.5 Hz, **4 subs** | `detection_3d_node`, `front_clearance_node`, `mission_logger_node`, and **`detected_classes_bridge` inside the BT**. All four confirmed in source. | No |
| `/camera/detection_masks` | callback | = YOLO | 10.1 Hz, 2.0 MB/s, 2 subs | Full-frame `mono8`. Genuinely consumed. | No |
| `/camera/image_annotated` | callback | debug only | 4.9 Hz, **3.4 MB/s**, 2 subs | Both subscribers are the viz path (throttle → `viz_jpeg_node`). No race consumer. Confirms Phase 1. | **Yes** |
| `/camera/camera_info`, `/camera/detection_markers` | callback | debug | 28.2 / 10.9 Hz, **0 subs** | Tiny. Not worth acting on. | No |
| `/scan` | 40 Hz native | 40 Hz | 40.00 Hz, 347 kB/s, **8 subs** | Matches. `publish_intensity: true` is most of the bandwidth. E-stop sensor. | **No** |
| `/odometry/filtered` | 50 Hz | 20–25 Hz by the 2× rule | 50.01 Hz, **8 subs** | Same `frequency:` knob also drives `odom→base_link` TF. Unchanged verdict: **do not reduce.** | **No — deliberate** |
| `/ekf_global/odometry/filtered` | 50 Hz | 50 Hz | 38.76 Hz | **Measurement artifact**, not a real drop — the CLI process was starved at 89 % CPU. Configured and actual are 50. | No |
| `/tf` | — | — | 102.70 Hz, **7 pubs** | Two EKFs at 50 Hz + `robot_state_publisher` ~10 Hz. 7th publisher is the ZED's own `robot_state_publisher` (6 in run 1, camera dead). No dynamic ZED IMU TF — `publish_imu_tf: false` holds. | No |
| `/drive`, `/ackermann_drive`, `/commands/*` | 10 Hz | 10 Hz | 10.00 Hz | Matches MPC `ts=0.1`. Mux timeout 0.2 s = 5 Hz floor. | No |
| `/costmap/boundaries` | 20 Hz | **10 Hz, speed-dependent** | 20.02 Hz, 1 sub | See §3. | **Yes, with a caveat** |
| `/perception/swept_clearance` ×4 | 20 Hz timer | disabled | 16–20 Hz, **0 subs** | Workspace grep: referenced only in its own node, launch file and prose. **No code subscriber anywhere.** Holds a live ZED-depth subscription. | **Yes** |
| `/perception/lidar_front_wall(+_virtual)` | callback on `/scan` | disabled | 39.97 Hz, **0 subs** | Same grep result: no consumer in any package. 25.3 % CPU. | **Yes** |
| `/obstacle_clearance`, `/safety/event` | callback | campaign-only | 39.8 Hz, **0 subs** | Consumed **only** by `test_campaign/robot_logger.py` and `test_campaign_logger.yaml`. Real consumers — but only during a test campaign, never in a race. | **Yes, conditionally** |
| `/perception/d_wall/*` | 10 Hz timer | 10 Hz | 8.6–10 Hz, 1 sub on the main topic | Resolves Phase 1 OQ-2. `stack_params.yaml` claims "no live run has ever produced a `/perception/d_wall` message" — **that is now stale**; it publishes. | No |
| `/perception/front_clearance` | callback | = detection | **1 sub** | `costmap_boundary_node`, plus `check_stop_condition` once a mission builds the tree. **Run 1's 0-sub reading was wrong in effect.** | No |
| `/bt/tree_visualization` | ≤2.5 Hz | debug | 2.24 Hz, **230.9 kB/s**, **0 subs** | `min_interval_sec=0.4`; a graphviz layout + PNG render per publish, in `behavior_executor_node`'s post-tick handler. Not lazy. | **Yes** |
| `/costmap/visualization` | 2 Hz | debug | 2.00 Hz, **208.4 kB/s**, **0 subs** | `render_rate_hz = 2.0`. Rate is already low; **size** is the cost. Not lazy. | **Yes** |
| `/sensors/imu` | 50 Hz | — | 50.02 Hz, 7.5 kB/s | Cooked IMU beside `/sensors/imu/raw`. Run 1 measured 0 subscribers; run 2's `info` returned nothing for it, so the count is unconfirmed this run. Rate hardcoded in vendor C++. | No change proposed |
| `/behavior/tree_status` | per tick | — | 7.84 Hz, **0 subs** | "Unthrottled, deliberately." 1.0 kB/s. | No |

---

## 3. `/costmap/boundaries` — what the frame_id and the MPC actually imply

Phase 1 §3.7 established the mechanism; here is what it means for the rate.

- **frame_id is `base_link`** (`costmap_boundary_node.py:643,718,754`), i.e. the
  constraint is published in the **body frame**.
- `MPC_corr.py:3120` converts it to world **at receipt**, using the pose at that
  instant, and holds the result until the next message.
- The staleness gate uses `odom_stale_timeout_sec = 0.5 s`. At 10 Hz that is a
  **5× margin** — the gate is *not* the binding constraint.
- The constraint is **soft**: `use_hard_boundary_constraints: true` but
  `boundary_hard: false` with `boundary_slack_weight: 10000.0`. It is a heavy
  penalty, not an infeasibility.

**The binding quantity is position error baked into the constraint**, which is
`speed × (age of the transform)`. Halving the rate doubles the worst-case age
from 50 ms to 100 ms:

| Speed | extra error at 10 Hz vs 20 Hz |
|---|---|
| 0.47 m/s (this run's max) | **2.4 cm** |
| 2 m/s | 10 cm |
| 5 m/s | **25 cm** |

So 10 Hz is safe at the speeds this car has actually been driven, and
progressively less safe as speed rises. This is a speed-dependent decision, not
a flat yes — see Phase 4 P4.

---

## 4. Corrections to Phase 1 and to run 1

1. **Only `detection_3d_node` uses `message_filters`.** Run 1's Phase 3 said all
   three depth consumers did. `front_clearance_node` explicitly avoids a
   synchronizer (`front_clearance_node.py:13`) and `swept_clearance_node` uses a
   plain sensor-QoS subscription.
2. **`swept_clearance_node`'s `use_camera` defaults true**
   (`swept_clearance.launch.py:93`: `get_value('camera_source') == 'zed'`), so
   it does hold a live ZED-depth subscription.
3. **`/perception/front_clearance` has a consumer** (`costmap_boundary_node`,
   and `check_stop_condition` at mission time). Run 1 recorded 0.
4. **`/camera/detections` has four consumers**, one of which is the behaviour
   tree via `detected_classes_bridge`. Phase 1 listed two.
5. **`/tf` has 7 publishers with the camera up**, 6 without.
6. **The ZED's zero-subscriber heavy topics are lazy.** Run 1 implied the point
   cloud might be a live cost; it is not, until something subscribes.
7. **`camera_model: 'zed2'` is cosmetic** — the SDK detects ZED 2i and says so.
8. **The CPU baseline in run 1 (44 %) was meaningless** for proposal sizing. The
   real idle figure is **85 %**.

---

## 5. Still unknown

- Which loop inside `fast-discovery-server` spins (one thread, 72 %, state R).
- YOLO's rate on a visually complex scene; 8.9 Hz was a static indoor view.
- All mission-time topic rates, and whether more BT subscriptions appear.
- Whether `sensors_pub_rate` can be lowered to the grab rate without upsetting
  the SDK's internal sensor pipeline (documented MIN is the grab rate).
