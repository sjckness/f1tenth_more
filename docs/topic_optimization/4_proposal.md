# Phase 4 — Proposal

**Nothing in this document has been applied.** Every diff is a proposal.
Rewritten against **run 2** (camera working). Run 1's savings estimates were
computed on a stack with no camera and are all superseded.

Design constraint, unchanged: everything lands as a **new launch profile plus
config overrides**. `supervisor_bringup.launch.py`, `components.yaml`,
`stack_params.yaml`, `zed2_perception.yaml` and all vendor source stay
untouched, so the two profiles A/B on the same car.

```
NEW  src/f1tenth_bringup/launch/race_optimized.launch.py
NEW  src/f1tenth_bringup/config/components_race.yaml
NEW  src/f1tenth_perception/config/zed2_race.yaml
```

The existing component tests (`test_lidar_front_wall_component.py`,
`test_wall_distance_component.py`, `test_obstacle_clearance_component.py`)
assert membership in **`components.yaml`**, which this proposal does not touch —
so they keep passing.

---

## The headline number

| | Run 1 (camera dead) | **Run 2 (camera up)** |
|---|---|---|
| Idle CPU | 44 % | **85 %** |
| Idle GPU | 0 % | **59 %** |
| Camera bus traffic | 0 | **≈ 57 MB/s** |

The car is at **85 % CPU standing still**. Everything below is sized against
that, not against run 1.

---

## P7 — ZED publish rate matched to YOLO  ← now the biggest item

**Current** `pub_frame_rate: 30.0`, achieved 28.6 Hz.
**Proposed** `pub_frame_rate: 12.0`.

The measurement that decides it, from `detection_3d_node`'s own counters:

| Stream | measured |
|---|---|
| ZED publishes | **28.6 Hz** |
| YOLO produces detections | **8.9 Hz** |
| sync drops | **0** |

The camera produces **3.2 frames for every one anything consumes**. The surplus
is encoded, copied over the bus, and dropped at the subscriber.

**Why 12.0 and not 9.0.** YOLO measured 8.9–9.5 Hz. Setting the source to
exactly that makes YOLO frame-limited instead of compute-limited, so any jitter
in the camera pipeline directly delays a detection. 12 Hz keeps ~25 % headroom
so YOLO stays the limiter and detection latency is unchanged.

**Every consumer of the two affected streams, and what each needs:**

| Stream | Consumer | Needs |
|---|---|---|
| `/camera/image_raw` | `yolo_detector_node` | one frame per inference → **8.9 Hz** |
| | `image_raw_viz_throttle` (lazy) | 5 Hz, Foxglove only |
| ZED depth | `detection_3d_node` | time-matched to detections → **8.9 Hz** |
| | `front_clearance_node` | latest at detection time → **8.9 Hz** |
| | `swept_clearance_node` | 20 Hz timer — **but its own output has no consumer** (P3) |

With P3 applied, the fastest genuine need on either stream is ~9 Hz. Without
P3, `swept_clearance_node` samples depth at 20 Hz to produce a topic nobody
reads, so it should not set the camera rate either way.

```diff
# src/f1tenth_perception/config/zed2_race.yaml  (new file, layered after
# zed2_perception.yaml)
+/**:
+  ros__parameters:
+    general:
+      # P7: YOLO measured 8.9 Hz; 12 Hz keeps headroom so YOLO stays the limiter.
+      pub_frame_rate: 12.0
+    sensors:
+      # P11: the real parameter. publish_imu/publish_imu_raw do not exist.
+      sensors_pub_rate: 15.0
```

- **Saving**: `/camera/image_raw` **25.7 → 10.8 MB/s**, ZED depth **25.7 →
  10.8 MB/s** — **≈ 30 MB/s**, over half the total bus traffic. Plus the ZED
  container's own encode/publish work (it measured 51.7 % CPU shared with the
  viz relays).
- **Risk**: **medium.** Detection *rate* is unchanged (YOLO is the limiter), but
  worst-case frame age rises from 35 ms to 83 ms, which matters at speed. Verify
  detection latency on the car.
- **Revert**: drop `zed2_race.yaml` from the profile.

### Resolution — not proposed, and why

Currently HD720 grab, `pub_downscale_factor: 2.0` → **640 × 360**. Lowering it
further trades directly against detection range, and I have no measurement of
the smallest object this stack must detect at what distance. **Do not change
resolution without that measurement.**

One free win is available at the same size: `/camera/image_raw` is **BGRA8**,
921 600 B/frame, and the alpha channel is unused — 25 % of the stream. Whether
this wrapper can publish BGR8 instead is an open question (see below).

---

## P11 — ZED IMU: fix a setting that has never been in effect  *(new)*

`zed2_perception.yaml` sets `sensors.publish_imu: false` and
`publish_imu_raw: false`. **Neither parameter exists.** Live query returns
`Parameter not set`; `common_stereo.yaml:47-50` declares only `publish_imu_tf`,
`sensors_image_sync` and `sensors_pub_rate: 200.`

The bandwidth is avoided anyway by the wrapper's `if (imu_SubCount > 0)` gate,
so this is **not** a bandwidth proposal. What it costs today is a sensor thread
polling the SDK and calling `count_subscribers()` on four topics **200 times a
second**, permanently, for zero consumers.

Fix is the `sensors_pub_rate: 15.0` line in the P7 diff above (documented MIN is
the grab rate).

- **Saving**: small but free; removes 185 polls/s of pure overhead.
- **Risk**: **low.** Nothing subscribes to any ZED sensor topic — the EKF fuses
  the **VESC** IMU (`ekf.yaml`: `imu0: sensors/imu/raw`).
- **Also**: delete the two dead lines from `zed2_perception.yaml` in a separate
  commit so the file stops claiming something untrue. That file edit is outside
  the "new files only" rule, so it is called out separately rather than bundled.

---

## P12 — Trim the ZED's advertised topic surface  *(new; addresses the discovery-server CPU)*

`fast-discovery-server` measured **41.1 % mean CPU idle**, and `top -H` shows
**one thread at 72.3 % in state R** while the other nine idle — a single hot
loop, CPU-bound, not blocked on I/O.

What it is servicing: **583 endpoints** across **68 participants** and 238
topics. **The ZED advertises 111 of those topics**, of which **62 are
`image_transport` plugin variants** — `compressed` (17), `compressedDepth` (15),
`ffmpeg` (15), `theora` (15). Nothing in this workspace subscribes to any of
them. They carry no data, but each is a discovery endpoint the server tracks and
relays to both super clients.

Ruled out as causes: no respawn loop (`ensure_discovery_server.py` logged one
clean start; the process ran 46 min), and no reconnect churn in the logs.

```diff
# src/f1tenth_perception/config/zed2_race.yaml
+    # P12: publish only the raw transport. Removes ~62 advertised topics.
+    # (Exact key is image_transport's own `disable_pub_plugins`, which the ZED
+    #  wrapper passes through; confirm the parameter path before applying.)
```

- **Saving**: ~62 of 583 endpoints (**11 %**). Whether that translates
  proportionally to discovery-server CPU is **unverified** — I did not prove
  which loop spins, and I am not going to claim a number I have not measured.
- **Risk**: **low** for the stack (nothing uses these transports), **medium** for
  Foxglove workflows that rely on `compressed` image topics.
- **If it does not help**: the next step is a profiler on that thread, or
  testing whether SIMPLE discovery over localhost is viable at all — the
  Discovery Server exists here to fix an EKF-pair stall, so that is a real
  trade, not a free swap.

---

## P1 — Disable the behaviour-tree PNG visualization  *(pre-approved)*

**Topic** `/bt/tree_visualization` · **measured** 2.24 Hz, **230.9 kB/s**,
**0 subscribers**.

`behavior_executor_node.py:250-270` runs
`_status_dot_graph(tree.root).create_png()` — a **graphviz layout + PNG render,
in-process**, up to every 0.4 s, unconditionally.

Your instruction is explicit and correct: **the gate must skip the render, not
just the publish.** The handler must return *before* `create_png()`, otherwise
the expensive part still runs.

```diff
# src/f1tenth_bringup/config/components_race.yaml
   behavior:
     - package: f1tenth_behavior
       launch_file: behavior_bringup.launch.py
+      args:
+        # P1: skips make_tree_visualizer entirely, so no graphviz render.
+        enable_bt_visualization: 'false'
```

`behavior_executor_node.py:755-756` only constructs the publisher and adds the
post-tick handler when the flag is set, so gating at that point already skips
the render — **verify this is a declared parameter reaching the node, not a
parse-time `get_value()`**, before relying on the arg above.

- **Saving**: 230.9 kB/s and a graphviz render 2.2×/s.
  `behavior_executor_node` measured **42.3 % CPU** idle with no mission.
- **Risk**: **low.** `/behavior/tree_status` and the ASCII snapshot log remain.

---

## P2 — Drop the costmap renderer  *(pre-approved)*

**Topic** `/costmap/visualization` · **measured** 2.00 Hz, **208.4 kB/s**,
**0 subscribers**. Not lazy.

Rate is already 2 Hz (`costmap_renderer_node.py:54`); the cost is message
**size**, so the lever is not running it.

- **Saving**: 208.4 kB/s plus that node's CPU.
- **Risk**: **low.** `semantic_layer_node` and `costmap_boundary_node` — the one
  the MPC actually consumes — are untouched.

---

## P10 — `dev_tools` out of the race profile  *(pre-approved)*

Removes `foxglove_bridge` (**51.3 % CPU idle**), the two image throttles, the
viz relay container and `viz_jpeg_node`.

- **Saving**: all Wi-Fi traffic, and **51.3 % CPU** — far more than run 1
  suggested, because the bridge now has a camera to serve.
- **Bonus**: removes one of the two super clients, cutting what the discovery
  server must relay.
- **Risk**: **low**, but you lose live remote visibility during the run.
- **Note on Wi-Fi**: Humble has **no `ROS_AUTOMATIC_DISCOVERY_RANGE`** (Iron+).
  Levers here are `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, and the fact that the
  Discovery Server is already bound to `127.0.0.1`. **Do not** set
  `ROS_LOCALHOST_ONLY` piecemeal — the three-places rollout of
  `ROS_DISCOVERY_SERVER` exists precisely because a partial rollout leaves nodes
  unable to find each other.

---

## P7a — Gate `/camera/image_annotated`  *(pre-approved)*

**Measured** 4.9 Hz, **3.4 MB/s**, 2 subscribers — **both** the viz path
(throttle → `viz_jpeg_node`). No race consumer.

With P10 applied its subscribers disappear, but `yolo_detector_node` still
draws and publishes the annotated frame every inference. Gate the draw on the
same `enable_foxglove` flag that gates its only consumers.

- **Saving**: 3.4 MB/s plus the annotation draw inside the 61.5 % CPU
  `yolo_detector_node`.
- **Risk**: **low.**

---

## P3 — Drop `swept_clearance`

**Topics and their consumers — the full list you asked for:**

| Topic | Rate | Subs | Consumer found in source? |
|---|---|---|---|
| `/perception/swept_clearance` | 20.00 Hz | **0** | **none** |
| `/perception/swept_clearance/lidar` | 39.94 Hz | **0** | **none** |
| `/perception/swept_clearance/camera` | 16.45 Hz | **0** | **none** |
| `/perception/swept_clearance/steering` | 20.00 Hz | **0** | **none** |

A workspace-wide grep finds these names only in the node itself, its launch
file, and prose in `stack_params.yaml`. **No package subscribes.**
`components.yaml` says so outright, and its README lists the node as never
validated on the car.

**What breaks if it never publishes: nothing.** No BT behaviour reads it — the
tree's inputs are `/scan` (`IsProximityTooClose`), `/camera/detections` (via
`detected_classes_bridge`), `/perception/front_clearance`, `/mpc/*`, battery and
system status. The supervisor's watchdog only polls for process exit, so a
component absent from the profile is simply never started and never missed.

- **Saving**: the node's CPU, and — importantly — **it removes one of the three
  ZED-depth subscribers**, which is what lets P7 drop the depth rate to ~9 Hz
  instead of 20.
- **Risk**: **low.** Registered in `components.yaml` for the default profile, so
  nothing about the existing stack changes.

---

## P5 — Drop `lidar_front_wall`; **hold** `obstacle_clearance`

### P5a — `lidar_front_wall` (recommended)

| Topic | Rate | Subs | Consumer in source? |
|---|---|---|---|
| `/perception/lidar_front_wall` | 39.97 Hz | **0** | **none** |
| `/perception/lidar_front_wall_virtual` | 39.94 Hz | **0** | **none** |

Same grep result: referenced only in its own node and prose. **Nothing breaks**
— no BT lane, no controller, no logger reads it.

- **Saving**: **25.3 % CPU** and ~4 kB/s.
- **Risk**: **low.**

### P5b — `obstacle_clearance` (**recommended: do not apply**)

| Topic | Rate | Subs | Consumer in source? |
|---|---|---|---|
| `/obstacle_clearance` | 39.77 Hz | 0 | **yes — `test_campaign/robot_logger.py`** |
| `/safety/event` | event | 0 | **yes — `test_campaign_logger.yaml`** |

This resolves run 1's open question. Both topics **are** consumed — by the
**test-campaign logger**, which only subscribes while a campaign is running.
In a race there is no consumer; during your campaigns there is.

`robot_logger.py` defaults `obstacle_clearance=None`, so a missing topic
degrades to an empty column rather than a crash — but it degrades *silently*,
and you have been running campaigns all week.

**Recommendation: leave `obstacle_clearance` in the race profile.** It costs
0.3 kB/s and a node that did not make the top-17 CPU list. The downside of
silently losing campaign data outweighs it.

---

## P4 — `/costmap/boundaries` 20 → 10 Hz: safe at current speeds

You asked whether the frame_id and MPC findings make 10 Hz safe. **Yes at the
speeds this car has been driven, and it is a speed-dependent call.**

What the code does (Phase 1 §3.7, Phase 3 §3):

- Published in **`base_link`** — body frame.
- `MPC_corr.py:3120` transforms it to world **at receipt**, using the pose at
  that instant, and holds it until the next message.
- Staleness gate is `odom_stale_timeout_sec = 0.5 s` → at 10 Hz a **5× margin**.
  **The gate is not the binding limit.**
- The constraint is **soft**: `boundary_hard: false`,
  `boundary_slack_weight: 10000.0`. A heavy penalty, not an infeasibility.

The binding quantity is position error baked into the held transform,
`speed × age`:

| Speed | extra error, 10 Hz vs 20 Hz |
|---|---|
| 0.47 m/s (measured max this run) | **2.4 cm** |
| 2 m/s | 10 cm |
| 5 m/s | **25 cm** |

At 2.4 cm against a soft constraint, 10 Hz is comfortably safe today. At true
race speed it is not obviously safe, and the right answer there is to make
`costmap_boundary_node` publish at 20 Hz again rather than to accept 25 cm.

- **Saving**: roughly half the work of a **53.3 % CPU** node.
- **Risk**: **medium**, and **speed-dependent — re-evaluate before racing above
  ~2 m/s.**
- **Revert**: set `extraction_rate_hz` back to `20.0`.

---

## P6 / P8 — unchanged, still not proposed

- **P6 `/sensors/imu`** (50 Hz, 7.5 kB/s, no known consumer): rate is hardcoded
  in `vesc_driver.cpp:147` vendor C++. Not worth a fork.
- **P8 sensor-data QoS**: still not proposed. QoS saves no bandwidth and no CPU;
  the measured problems are message size and unconsumed publishers. The only
  candidate, `/scan`, is the e-stop's only sensor.

---

## P9 — MPC odom-triggered: separate branch

Unchanged from run 1, and now better motivated: `mpc_corr` measured **68.2 %
CPU**, the hottest process on the car. It runs a free-running 10 Hz timer
against a 50 Hz `/odometry/filtered`, so it acts on state 0–20 ms stale, varying
run to run.

- **Risk**: **high** — changes the controller's timing model. If odom stalls the
  loop stops, whereas the timer keeps publishing a stop. Needs a watchdog
  reproducing that.
- **Recommendation**: own branch, own on-car validation, after the rate changes
  are A/B'd.

---

## Recommended order

| # | Change | Saving | Risk |
|---|---|---|---|
| **P7** | ZED `pub_frame_rate` 30 → 12 | **≈ 30 MB/s** | medium |
| **P10** | `dev_tools` out *(pre-approved)* | **51.3 % CPU** + all Wi-Fi | low |
| **P1** | BT PNG viz off *(pre-approved)* | 231 kB/s + graphviz, of 42.3 % CPU | low |
| **P5a** | `lidar_front_wall` out | **25.3 % CPU** | low |
| **P2** | costmap renderer off *(pre-approved)* | 208 kB/s | low |
| **P7a** | gate `/camera/image_annotated` *(pre-approved)* | 3.4 MB/s | low |
| **P3** | `swept_clearance` out | CPU + unblocks P7's depth cut | low |
| **P11** | ZED `sensors_pub_rate` 200 → 15 | 185 polls/s | low |
| **P12** | trim 62 ZED transport topics | 11 % of endpoints, CPU effect unproven | low |
| **P4** | boundaries 20 → 10 Hz | ~half of 53.3 % CPU | medium, speed-dependent |
| P5b | `obstacle_clearance` out | 0.3 kB/s | **not recommended** |
| P9 | MPC odom-triggered | timing jitter | high, separate branch |
| P6, P8 | — | — | no change proposed |

---

## Open questions / uncertain

1. **Is `enable_bt_visualization` a declared parameter on
   `behavior_executor_node`, or a parse-time `get_value()`?** Determines whether
   P1's launch arg reaches the node. Unverified.
2. **Can the ZED wrapper publish BGR8 instead of BGRA8?** A free 25 % cut on the
   largest stream if so. Parameter not located.
3. **Exact parameter path for disabling `image_transport` plugins** through this
   wrapper (P12). The mechanism is standard, the key name is not confirmed.
4. **Which loop in `fast-discovery-server` spins?** One thread, 72 %, state R.
   Not identified — needs a profiler. P12 is a plausible mitigation, **not a
   proven fix**.
5. **Does `sensors_pub_rate: 15.0` upset the SDK's sensor pipeline?** Documented
   MIN is the grab rate; 15 > 12, so it should be legal, but untested.
6. **Does `costmap.launch.py` expose per-node toggles?** P2 and P4 assume a
   small edit to our own launch file is acceptable.
7. **All mission-time rates remain unmeasured** — `/mpc/goal_*`, `/corridor`,
   `/safety_stop`, and whether more BT subscriptions appear once a tree is built.
   `/perception/front_clearance` already went 0 → 1 subscriber between runs.
8. **YOLO at 8.9 Hz was measured on a static indoor scene.** A cluttered scene
   with many detections will be slower, which makes P7's 12 Hz *more* than
   enough, not less — but it should be confirmed.
