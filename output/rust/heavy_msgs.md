# Heavy messages: who reads images, depth, camera_info, PointCloud2 and scans, and what it costs

Measurement only, no port. linus (i7-9700K, 8 cores), 2026-10-18, Isaac Sim + the full stack on
one host (`--net local`, Fast DDS with shared memory, Discovery Server), branch `jazzy-rust`,
run `isaac_smoke/py_shadow` (e2e session, `impl:=py guard:=shadow`). Sources:
`isaac_smoke/py_shadow/heavy_census.json` (rates, sizes and subscribers over 20 s, per-process CPU
over the same 20 s), `isaac_smoke/py_shadow/ingest_probes.txt` (rclpy vs rclrs ingest),
`bench_ingest.json` (offline deserialization and conversion). Tools: `tools/rust_parity/heavy/`.

**Short answer.** The Python perception nodes take ~29 MB/s of images, depth and point clouds
between them. Ingest (DDS take, deserialization and conversion into numpy) costs about **4.6 ms per MB
in rclpy and 1.9 ms per MB in rclrs**, measured live on the same topics. That puts **~34 % of a core
(about a quarter of the five nodes' CPU) into ingest**; the other three quarters are their own
compute (and, for yolo, the GPU). A Rust ingest node gains the most where a Python consumer reads
much more than it uses: front_clearance (a depth ROI), swept_clearance (every 4th pixel) and
detection_3d (depth inside the detections). Expected gain of that one node: ~17 % of a core on
linus, net of its own cost (~23 % if front_clearance also drops its debug-only RGB input).

## 1. The heavy topics (Isaac, 20 s census; RAW subscriptions, nothing deserialized)

| Topic | Type | Hz | KB/msg | MB/s | Publisher | Subscribers (stack) |
|---|---|---:|---:|---:|---|---|
| `/camera/camera_info` | CameraInfo | 15.1 | 0.4 | 0.01 | isaac_f1tenth_sim | yolo_detector_node |
| `/camera/detection_masks` | Image | 13.5 | 225.1 | 2.97 | yolo_detector_node | detection_3d_node, front_clearance_node |
| `/camera/image_annotated` | Image | 4.7 | 675.1 | 3.06 | yolo_detector_node | image_annotated_viz_throttle |
| `/camera/image_annotated/viz` | Image | 3.2 | 675.1 | 2.11 | image_annotated_viz_throttle | (no subscriber) |
| `/camera/image_raw` | Image | 14.4 | 900.1 | 12.70 | isaac_f1tenth_sim | front_clearance_node, image_raw_viz_throttle, yolo_detector_node |
| `/camera/image_raw/viz` | Image | 4.0 | 900.1 | 3.47 | image_raw_viz_throttle | (no subscriber) |
| `/perception/scene_labels` | Image | 4.9 | 14.1 | 0.07 | front_clearance_node | detection_3d_node, yolo_detector_node |
| `/perception/scene_labels/classes` | Image | 4.8 | 225.1 | 1.07 | front_clearance_node | lidar_front_wall_node |
| `/perception/scene_labels/depth_mm` | Image | 4.6 | 450.1 | 2.02 | front_clearance_node | lidar_front_wall_node |
| `/perception/scene_labels/image` | Image | 4.0 | 675.1 | 2.64 | front_clearance_node | (no subscriber) |
| `/perception/stereo_columns` | StereoColumns | 4.9 | 10.1 | 0.05 | front_clearance_node | lidar_front_wall_node |
| `/perception/wall_points` | PointCloud2 | 4.9 | 249.7 | 1.20 | front_clearance_node | lidar_front_wall_node |
| `/scan` | LaserScan | 40.2 | 4.3 | 0.17 | isaac_f1tenth_sim | behavior_executor_node, front_clearance_node, guard_vs_bt, lidar_front_wall_node, obstacle_clearance_node, proximity_guard_node, slam_toolbox, swept_clearance_node |
| `/slam/map` | OccupancyGrid | 0.2 | 14.5 | 0.00 | slam_toolbox | costmap_boundary_node, costmap_renderer_node, slam_toolbox |
| `/viz_internal/camera/image_annotated` | Image | 2.1 | 675.1 | 1.42 | viz_throttle_camera_image_annotated | (no subscriber) |
| `/zed2/zed_node/depth/camera_info` | CameraInfo | 15.1 | 0.4 | 0.01 | isaac_f1tenth_sim | detection_3d_node, front_clearance_node, swept_clearance_node |
| `/zed2/zed_node/depth/depth_registered` | Image | 14.2 | 900.1 | 12.44 | isaac_f1tenth_sim | depth_throttle_5hz, detection_3d_node, front_clearance_node, swept_clearance_node |
| `/zed2/zed_node/depth/depth_registered_5hz` | Image | 4.2 | 900.1 | 3.65 | depth_throttle_5hz | (no subscriber) |
| `/zed2/zed_node/left/image_rect_color` | Image | 14.6 | 900.1 | 12.83 | isaac_f1tenth_sim | (no subscriber) |
| `/zed2/zed_node/right/image_rect_color` | Image | 13.9 | 900.1 | 12.21 | isaac_f1tenth_sim | (no subscriber) |

Notes:
- Rates are sim rates times RTF (camera 15 Hz in Isaac; the car's ZED publishes 30, its
  camera_info measured 13.7 Hz in the 2026-10-15 bag). The car's /scan carries intensities
  (8.7 KB/msg in the bag, 4.3 KB in Isaac).
- `guard_vs_bt` is the comparison recorder of this test, not part of the stack.
- Isaac renders and publishes the ZED left and right rectified images (25 MB/s together) and nobody
  in the stack subscribes them. With DDS, a topic without subscribers costs no transport, but
  Isaac still renders the right camera. Sim-only saving: `sensors.yaml stereo.right_camera: false`
  (it says "set false to save VRAM"). Not changed: VRAM is not short (section 4).

## 2. Who needs which resolution

| Consumer | Lang | CPU % (one core) | Heavy input MB/s | Reads | Needs full resolution? |
|---|---|---:|---:|---|---|
| yolo_detector_node | Python (GPU inference) | 39.7 | 12.8 | `/camera/image_raw` 640x360 bgra8 -> `imgmsg_to_cv2('bgr8')` -> model | **yes**: the model input |
| front_clearance_node | Python | 40.1 | 28.3 | depth `imgmsg_to_cv2('32FC1')`, then an ROI (half-width 180 px, half-height 70 px around a centre 35 px up: ~22 % of the frame); masks; the RGB frame only for the 5 Hz scene-mask overlay (debug) | **no**: ROI of depth; RGB only for debug |
| detection_3d_node | Python | 17.1 | 15.5 | depth `32FC1` + masks `mono8`: median depth inside each detection's mask (box fallback) | full resolution, but only inside detections |
| swept_clearance_node | Python | 12.6 | 12.6 | depth: `np.frombuffer(bytes(msg.data))` (an extra copy) then **every 4th pixel** (`camera_stride_px` 4) unprojected | **no**: 1/16 of the pixels |
| lidar_front_wall_node | Python | 28.2 | 4.5 | /scan, scene_labels classes + depth_mm (5 Hz, np.frombuffer views), wall_points (`read_points_numpy`), stereo_columns | yes, at 5 Hz (already reduced upstream); mostly compute |
| behavior_executor_node, obstacle_clearance_node | Python | 13.9, 5.0 | 0.2 | /scan, `np.asarray(ranges)` | yes (1081 beams, cheap) |
| proximity_guard_node | **Rust** | 2.6 | 0.2 | /scan | yes |
| slam_toolbox, depth_throttle_5hz, *_viz_throttle, foxglove_bridge | native | 2.3, 3.2, 3.0/2.1, 9.4 | | | |

The per-process CPU includes everything each node does (timers, other subscriptions).

## 3. Ingest vs compute

**Live ingest cost per language** (same 8 topics, same QoS: sensor data, depth 2; 30 s each;
CPU from `/usr/bin/time`): the probes subscribe and count, `--touch` additionally reads every byte
(Python: `np.frombuffer(...).sum()`, the nodes' conversion).

| Probe | CPU % | MB received | ms CPU per MB | messages |
|---|---:|---:|---:|---:|
| rclrs, no-op | 5.6 | 894 | **1.89** | 3,424 |
| rclpy, no-op | 11.8 | 768 | **4.62** | 3,029 |
| rclrs, touch | 6.5 | 882 | 2.20 | 3,375 |
| rclpy, touch | 14.6 | 865 | 5.07 | 3,393 |

The rclpy no-op probe received 11 % fewer messages than rclrs over the same 30 s: the
best-effort, depth-2 queues overflow while the Python executor is busy. Cross-check: the native
`depth_throttle_5hz` (topic_tools, C++) uses 3.2 % for 12.4 MB/s = 2.6 ms/MB, the same order as
the rclrs probe.

**Offline, the CDR deserialization itself is small** (`bench_ingest.json`, recorded car messages and
a synthetic ZED frame): `deserialize_message` 0.10-0.13 ms for a 0.9 MB image or depth frame,
0.054 ms for the 225 KB wall_points cloud, 0.016 ms for a scan. Conversions:
`imgmsg_to_cv2('32FC1')` 0.08 ms, `('bgr8')` from bgra8 0.06 ms, `np.frombuffer(bytes(data))`
0.03 ms, `read_points_numpy` 0.19 ms, `np.asarray(ranges)` 0.002 ms. Live, the same 0.9 MB costs
~4.2 ms in rclpy. **The bulk of the Python ingest cost is the take path (rmw/Fast DDS receive of a
fragmented 0.9 MB sample, copies into Python objects, executor wake-ups), not the numpy
conversion.** That is why a PyO3 kernel, which runs after rclpy has taken the message, cannot
remove it.

**Estimated split per node** (heavy MB/s x 4.62 ms/MB, against the node's measured CPU):

| Node | CPU % | ingest est. (rclpy) | compute (rest) | same ingest in rclrs |
|---|---:|---:|---:|---:|
| front_clearance_node | 40.1 | 13.1 | 27.0 | 5.3 |
| yolo_detector_node | 39.7 | 5.9 | 33.8 (+ GPU) | 2.4 |
| lidar_front_wall_node | 28.2 | 2.1 | 26.1 | 0.8 |
| detection_3d_node | 17.1 | 7.2 | 9.9 | 2.9 |
| swept_clearance_node | 12.6 | 5.8 | 6.8 | 2.4 |
| **five nodes** | **137.7** | **34.1** | **103.6** | 13.8 |

(Estimate: it assumes each node pays the probe's per-MB cost; scans and camera_info are counted in
MB/s but are negligible.)

## 4. Recommendation for a later ingest node

**A full rclrs ingest node for depth, not a PyO3 kernel.**

- **What it would do:** subscribe once to `/zed2/zed_node/depth/depth_registered` (+ camera_info,
  + `/camera/detection_masks`), and publish the reductions the Python nodes compute from the full
  frame today: the front_clearance depth-ROI statistics, swept_clearance's stride-4 unprojected
  points (a small PointCloud2), detection_3d's per-detection median depths. The Python nodes then
  subscribe to the small products instead of the 0.9 MB frames.
- **Expected gain on linus:** the depth + mask ingest of front_clearance (15.4 MB/s: 7.1 %),
  swept_clearance (5.8 %) and detection_3d (7.2 %) is ~20 % of a core in rclpy. The Rust node
  ingests that stream once for ~2.9 % (15.4 MB/s x 1.89 ms/MB), so the net is **~17 % of a core**.
  Another ~6 % if front_clearance stops taking the full RGB frame it only uses for its 5 Hz debug
  overlay. Plus a share of the reduction compute (ROI / median / unprojection), not measured. Second effect: less executor load in those nodes, so fewer dropped frames (the probe
  above lost 11 %).
- **Why not PyO3:** a kernel speeds up compute inside a Python callback but rclpy still takes the
  full 0.9 MB message (the 4.6 ms/MB above). The compute-bound nodes are better targets for
  kernels: lidar_front_wall_node (26 % compute, 2 % ingest; extract_lines is already a C kernel
  since campaign-final H1) and front_clearance's scene labelling.
- **Leave yolo_detector_node alone:** it needs the full RGB frame on the GPU; its ingest is ~6 % of
  39.7 % and moving it would need a zero-copy path into torch.
- **Order of magnitude on the car:** the ZED publishes 30 Hz there (twice the sim rate) and Orin
  cores are slower, so the share is likely larger; not measured here. Orin/Humble is out of scope.
- **Prerequisites learned here:** the rclrs toolchain and the parity method (setup.md, the two parity
  reports) carry over unchanged; the ingest node's outputs need their own parity harness
  (the same pattern: the Python reduction on recorded depth frames vs the Rust replay).

## 5. Resources on linus with Isaac + the full stack (both e2e runs)

| | impl:=py guard:=shadow | impl:=rust guard:=shadow |
|---|---|---|
| VRAM used (max, all processes incl. desktop ~1.5 GB) | 3,571 MiB of 8,151 | 3,586 MiB of 8,151 |
| of which Isaac / YOLO (nvidia-smi compute apps) | 1,681 / 342 MiB | |
| GPU utilisation mean / max | 17.5 / 37 % | 19.8 / 43 % |
| host RAM used mean / max (32 GB) | 18.9 / 20.2 GB | 18.9 / 19.9 GB |
| host CPU mean / p95 / max (8 cores = 100 %) | 49 / 67 / 77 % | 54 / 68 / 94 % |
| Isaac CPU mean (100 % = one core) | 112 % | 124 % |
| costmap_boundary_node CPU / RSS | 9.0 % / 89 MB (Python) | **1.7 % / 38 MB (Rust)** |
| proximity_guard_node CPU / RSS | 2.6 % / 38 MB | 2.8 % / 38 MB |

VRAM headroom 4.5 GB: YOLO's size and rate need no sim-only reduction. RAM headroom ~12 GB.
