# Depth ingest (R5): what the two depth consumers read, and what a Rust ingest node gives them

main-jazzy-rust, phase R5.0 (inventory) and the design R5 implements. Sources: the code at
d58c0f8e (`f1tenth_perception/front_clearance_node.py`, `detection_3d_node.py` and the
modules they call), stack_params.yaml (the shipped values), `output/rust/heavy_msgs.md` (the
Isaac heavy-topic census and the rclpy/rclrs ingest cost), the car bag
`~/bags/yolo_gpu/corridor10m_20261015T171753` (ZED 640x360, 32FC1, step 2560, little endian;
masks mono8 640x360, zero-size when a frame has no detection). swept_clearance is retired
from the auto-started set since R3, so its stride-4 point product is not built.

Shipped values used below: depth 640x360 (230,400 px, 921,600 bytes), ROI half-width 180,
half-height 70, shift up 35; ground removal on, fit stride 4; scene labels on (grid 160x90,
5 Hz); scene mask on (5 Hz); use_mask_depth true (yolo26s-seg); detection_3d sync slop 0.1 s,
queue 15; scene_labels_max_age_s 0.3; scene_min_other_px 20; center_fraction 0.5.

## 1. Per depth frame: exactly what each consumer reads

### 1.1 front_clearance_node (every depth frame, `depth_callback`)

Input: `/zed2/zed_node/depth/depth_registered`, LATEST_FRAME_QOS (keep last 1, best effort),
`imgmsg_to_cv2(msg, '32FC1')` of the whole frame. The car's ZED publishes 30 Hz (Isaac 15).

| Step | Pixels of the depth frame | Fields / tests | Rate |
|---|---|---|---|
| 2. ROI | `depth[y0:y1, x0:x1]`, y0 = max(0, h//2 - shift - half_h) = 75, y1 = 215, x0 = 140, x1 = 500: **140 x 360 = 50,400 px (21.9 %)** | the values | every frame |
| 3. car's LiDAR housing | none (geometry: lidar_exclusion_* fractions of h, w; 340 ROI px at (480..500, 198..215)) | | every frame |
| 4. too close | the ROI | `isneginf` count outside the housing vs `too_close_min_pixel_fraction` | every frame |
| 5.-6. validity | the ROI | `isfinite & > 0`, `1 - conf >= background_weight_threshold` (conf = the object-confidence EMA, at depth resolution, from masks/boxes, no depth), `~housing` | every frame |
| 6b. scene validity | none (the label grid of the last scene-label tick, max age 0.3 s, nearest-row/col map into the ROI) | WALL or OTHER | every frame |
| 6b. floor fallback (no fresh labels, ground_removal_enabled) | `depth[y0:h:4, x0:x1:4]` (**72 x 90 = 6,480 px**, rows below the ROI included) for the RANSAC plane; the ROI's finite px back-projected | values | frames without fresh labels |
| 7.-10. statistics | `roi[valid]`: percentiles 5 / 50, band mean, count; EMAs, latches | | every frame |
| debug payload (/perception/debug/front_wall) | the ROI again (counts) | | every frame |
| 3a. scene MASK tick | **the whole frame**: `scene_mask.classify` (pixel_z, valid_depth of every px), `wall_points` (WALL px back-projected), stereo column ranges/extent, class stats (`isneginf` of every px), and when subscribed `/perception/scene_labels/classes` + `/depth_mm` (every px as mono16 mm) | values | scene_mask_rate_hz 5, gated on the node clock at the callback (`now >= next`, next = now + 0.2) |
| 3b. scene LABELS tick | the 160 x 90 grid `depth[4i+2, 4j+2]` (14,400 px), back-projected; floor RANSAC, ceiling, wall planes, labels, clusters | values | scene_rate_hz 5, the same kind of gate, its own `next` |
| inferred floor (in the mask tick) | none (labels, /scan) | | 5 Hz |

So at the depth rate front_clearance needs the ROI (and on frames without fresh labels a
stride-4 floor sample); only at the two 5 Hz ticks does it need the whole frame.

The **statistics themselves cannot be computed upstream without moving front_clearance's
logic**: the valid set depends on the object-confidence EMA (masks/boxes with a wall-clock
dt-normalised alpha, attack instant / release EMA), on the label grid produced by the node's own
5 Hz label tick, and on the floor RANSAC, which shares one `np.random.default_rng(0)` stream
between the fallback fit and the label tick. A Rust ingest node computing "ROI statistics"
would have to port all three (most of the node). The reduction that does not move logic is
the **pixels** the node reads: the ROI, the floor sample, and the whole frame at the ticks.

### 1.2 detection_3d_node (every synchronized set, `_synced_callback`)

Inputs: `message_filters.ApproximateTimeSynchronizer([detections, depth, masks], queue_size 15,
slop 0.1)`, each a `message_filters.Subscriber` (keep last 10, reliable; NOT newest-frame-only:
the depth and mask queues must hold ~0.5 s while the detections arrive ~120-450 ms after
capture). camera_info (keep last 10) is cached; `/perception/scene_labels` (LATEST_FRAME_QOS)
is buffered 0.6 s by stamp. Rates: depth 30 Hz on the car (every frame deserialized by rclpy,
also the ~80 % never paired), detections and masks at the detector rate (~13 Hz in Isaac),
sync at the detector rate.

The synchronizer (Python message_filters, not the C++ policy): per input a dict
stamp_ns -> msg (a re-sent stamp replaces the value in place), trimmed to 15 by evicting the
smallest stamp; on each arrival, for every OTHER input the stamps within slop of the new one,
sorted by |delta| (stable); the first combination of `itertools.product` (first input slowest)
with max - min < slop (strict) whose stamps are all still queued fires and is removed.

Per synced set (`depth = imgmsg_to_cv2(depth, '32FC1')`, `mask = imgmsg_to_cv2(mask, 'mono8')`,
zero-size mask = "no instances" marker), for each detection i whose score passes
confidence_threshold:

| Path (first that applies) | Pixels read | Result |
|---|---|---|
| no usable mask (2-way sync, zero-size, decode failure) | box region: `depth[y1:y2, x1:x2]` with x1 = int(clip(cx - w*cf/2, 0, W-1)) ..., cf = center_fraction 0.5 (25 % of the box area) | median of `isfinite & > 0` (None if empty or degenerate) |
| mask (shape != depth: cv2 INTER_NEAREST resize first) | **the whole frame**, `(mask == i+1) & isfinite(depth) & (depth > 0)` (vectorised over 230,400 px per detection) | |
| label i+1 > 255 | box region | box median (fallback, counted) |
| no valid depth in the mask | box region | box median (fallback, counted + warned) |
| scene labels on, a buffered label image within 0.3 s of the depth stamp (nearest stamp, first on ties) | the in-mask px whose label `labels[r*gh//H, c*gw//W]` is OTHER | median over them if >= scene_min_other_px (20), else the mask median (both counted) |
| otherwise | the in-mask valid px | median |

Medians are `float(np.median(float32 values))`: an order statistic of the partition, for an even
count the float32 sum of the two middle values halved (float32(float64(a+b)/2)), NaN if any
NaN. The selection `isfinite & > 0` means the medians only ever see finite positive values.
Then (Python, unchanged): z < min_valid_depth drop, back-projection with fx/fy/cx/cy, the
/tf_static lookup, covariance, Detection3DArray + markers, yield counters.

### 1.3 Who else reads depth

`depth_throttle_5hz` (topic_tools, native: the recorder's 5 Hz copy). swept_clearance (stride
4) is no longer auto-started (R3). front_depth_monitor_node is not launched. So after the
ingest node the full-rate depth frame has one Python-free reader per process that needs it.

## 2. What a Rust ingest node can provide without moving their logic

| Consumer | Reduced output | Moves | Stays in Python |
|---|---|---|---|
| detection_3d | `f1tenth_messages/DetectionDepthArray` per synchronized set: the synced Detection2DArray unchanged, the depth header, camera_info's fx fy cx cy, the depth/mask decode status, and per detection the chosen median (float32, exact), its **source** (box / mask / scene OTHER / scene fallback / mask-empty fallback / label-overflow fallback / invalid box) and the pixel counts (box, mask, OTHER) | the depth sampling functions (`_median_depth`, `_mask_median_depth`, `_scene_other`), the scene-label buffer, the synchronizer | confidence filter, min_valid_depth, back-projection, TF, covariance, markers, yield/stat counters and every log line (driven by the source codes) |
| front_clearance | `f1tenth_messages/DepthRoi` per depth frame: the ROI pixels (bit for bit), the stride-4 floor sample, and on a scene tick the whole depth Image unchanged; the two 5 Hz tick schedules run in the ingest node (same rule, its node clock) | the frame decode, the ROI crop and the tick gate | everything else: the node rebuilds a depth array (NaN outside the delivered pixels, which nothing reads on a non-tick frame) and runs its code unchanged |

The node subscribes depth (keep last 10, reliable: the synchronizer needs the history),
camera_info, detections, masks (keep last 10, reliable) and scene labels (latest frame) once.
front_clearance keeps its own masks / detections / obstacles / camera_info / tf_static / scan /
RGB subscriptions (none of them is depth; the object-confidence EMA and the overlay are its
logic). Bytes into Python per second at 30 Hz depth: front_clearance 27.6 MB/s raw ->
6.8 MB/s (ROI 201,600 B + floor sample 25,920 B per frame) + 4.6 MB/s (whole frame at 5 Hz);
detection_3d 27.6 MB/s depth + ~3 MB/s masks -> ~1 KB per synced set.

## 3. Switches and plumbing

- `depth_source:=raw|ingest` (stack_params `depth_source`, default raw): detection.launch.py
  argument, passed to both consumers as the read-only parameter `depth_source`; with ingest the
  launch file also starts `f1tenth_depth_ingest_rs depth_ingest_node` with the consumers' own
  parameter values (the same LaunchConfigurations). Supervisor `_RUST_SWITCHES['depth_source'] =
  ('detection.launch.py', ('raw', 'ingest'))`, bringups `depth_source:=` (stack_bringup forwards
  it to its detection.launch.py include).
- Topics: `/perception/depth_ingest/detection_depths` (DetectionDepthArray, reliable keep last
  10, as detection_3d's synchronizer inputs were), `/perception/depth_ingest/front_roi`
  (DepthRoi; front_clearance subscribes it newest-frame-only, as it subscribed depth).
- No package.xml dependency on the Rust crate (R1 rule); the launch file references the
  package only when depth_source is ingest.
- Test hook `F1TENTH_DEPTH_INGEST_INPROCESS=1`: a raw-mode node routes every raw frame through
  the same Rust reduction in-process (libf1tenth_depth_ingest_rs.so, ctypes) and its ingest
  consumer path, so the existing suites exercise the ingest path with their own inputs.

## 4. Not in R5

- front_clearance's `/camera/image_raw` subscription (the RGB frame for the debug overlay, ~6 %
  of a core in Isaac) and its masks: not depth; moving the object-confidence EMA is moving logic.
- The scene mask and scene labelling compute (most of front_clearance's 27 % compute): a kernel
  question, not ingest.
- rclrs 0.8 (R0b), Thor/Orin builds (R7).
