# f1tenth_perception

Camera bringup (ZED2 or webcam, mutually exclusive), Hokuyo LiDAR bringup,
and the YOLO 2D-detection → 3D-fusion → ground-plane-obstacle pipeline that
feeds both `mpc_controller`'s soft-avoidance and `f1tenth_behavior`'s
`IsObstacleDetected` condition. (`IsProximityTooClose`'s front check used to
also be fed from here, via `front_depth_monitor_node`/`front_clearance_node` below — REPLACED by a
LiDAR front-cone check reading `f1tenth_behavior`'s own `/scan` subscription
directly; see that behaviour's own docstring.)

## Nodes

| Node | Subscribes | Publishes | Notes |
|---|---|---|---|
| `yolo_detector_node` | `/camera/image_raw` | `/camera/detections` (`vision_msgs/Detection2DArray`), `/camera/image_annotated` | Ultralytics YOLO (TensorRT `.engine` or `.pt`), `device`/`confidence_threshold`/`model_path` params. Header is copied verbatim from the input image (capture-time propagation, not re-stamped at publish). |
| `detection_3d_node` | `/camera/detections`, `/zed2/zed_node/depth/depth_registered`, `.../depth/camera_info` | `/camera/detections_3d` (`vision_msgs/Detection3DArray`), `/camera/detection_markers` (`MarkerArray`) | `message_filters.ApproximateTimeSynchronizer` (`sync_slop=0.1s`, `sync_queue_size=60`). Back-projects 2D box + depth → 3D pose via pinhole model, transforms into `output_frame` (default `zed2_left_camera_frame`) via tf2. Only built when `camera_source == 'zed'`. |
| `obstacle_projector_node` | `/camera/detections_3d` | `/perception/obstacles_2d` (`f1tenth_messages/Obstacle2DArray`) | tf2 into `output_frame` (default `base_link`), height-band filter (`obstacle_z_min`/`obstacle_z_max`), radius filter (`min_obstacle_radius`/`max_obstacle_radius`), merges close detections (`obstacle_merge_distance`). ZED-only, depends on `detection_3d_node`'s output. |
| `front_clearance_node` | `/zed2/zed_node/depth/depth_registered`, `/camera/detections`, `/camera/detection_masks` (optional), `/perception/obstacles_2d` (optional) | `/perception/front_distance` (`std_msgs/Float32`), `/perception/front_wall` (`Bool`), `/perception/front_clearance` (`Float32`), `/perception/front_blocked` (`Bool`) | Jitter-hardened. **Every output is filtered** — EMA on the distances, dead-band+dwell latches on the booleans, and an EMA'd per-pixel object-exclusion mask (not a fresh/stale switch). `front_distance` is **background/wall only** (detected objects excluded by construction); `front_clearance` is the stop/slow-down signal, `min(background, nearest in-corridor obstacle)`. Publishes one message per depth frame regardless of what else arrives — never gated on a synchronizer. Cuts the car's-own-LiDAR rectangle out of the ROI using the same `lidar_exclusion_*` fractions `yolo_detector_node` uses. `publish_debug_raw` adds `/perception/debug/front_distance_raw` + `/perception/debug/bg_pixel_count` for bag-based tuning. ZED-only. **Every tuning default is unfitted.** |
| `front_depth_monitor_node` | `/zed2/zed_node/depth/depth_registered` | `/perception/front_distance` (`std_msgs/Float32`) | **No longer launched** — replaced by `front_clearance_node` above, which publishes the same topic (two publishers on one topic is why it was removed rather than left running). Module and `console_scripts` entry point both still present and still work if run by hand. Raw ZED depth, center-crop percentile (`center_fraction`, `front_distance_percentile`), no smoothing and no object exclusion. |
| ~~`wall_detector_node`~~ | — | ~~`/perception/front_clearance`~~ | **DELETED** by the dual-EKF + costmap-derived-boundaries pass (Open3D RANSAC plane segmentation on the ZED point cloud). Listed only because it used to own the name `/perception/front_clearance`, which `front_clearance_node` above now publishes with a **different meaning and a different source**. `f1tenth_behavior`'s `front_clearance` stop_condition no longer reads any `/perception/*` topic — it reads **`/costmap/front_clearance`** from `f1tenth_costmap`'s `costmap_boundary_node`. Anything under `/perception/front_clearance` in an old bag or Foxglove layout is this deleted node's output. |

## Launch files

| File | Purpose |
|---|---|
| `camera.launch.py` | Exactly one of `{zed_wrapper's zed_camera.launch.py, v4l2_camera_node}`, selected by `camera_source`. Both remapped onto the canonical `/camera/image_raw` + `/camera/camera_info` so `detection.launch.py` is camera-agnostic. Also publishes the static `base_link → zed2_camera_link` TF when `camera_source=zed` (0.12m ahead, 0.15m above `base_link`, forward-facing). |
| `lidar.launch.py` | `urg_node`, gated by `use_lidar` (default `false`), config from `f1tenth_bringup/config/sensors.yaml`. **Not included by `stack_bringup.launch.py` directly** — only reachable via `f1tenth_navigation/nav2.launch.py` (i.e. only when `enable_nav2=true`) or this package's own standalone `perception.launch.py`. |
| `detection.launch.py` | `yolo_detector_node` (always) + `detection_3d_node`/`obstacle_projector_node`/`front_clearance_node` (only when `camera_source == 'zed'`, since all three need ZED depth). `front_clearance_node` declares 17 `front_clearance_*` launch args sourced from `stack_params.yaml`. Declares per-node `cpu_affinity`/`nice` launch args (machine-specific, not in `stack_params.yaml`), added by the perception-optimization pass after the latency audit found `/camera/detections` lagging `/camera/image_raw` by ~278ms, root-caused to CPU/executor contention rather than transport/QoS. |
| `perception.launch.py` | Standalone entry point bundling `camera.launch.py` + `lidar.launch.py` + `detection.launch.py` for exercising perception in isolation — `stack_bringup.launch.py` brings up the same three pieces itself rather than including this file. |

## Config

`config/zed2_perception.yaml` — ZED wrapper param overlay: disables
`publish_status`/`publish_stereo`/`pos_tracking_enabled`/`mapping_enabled`/
`od_enabled`/`publish_imu_raw`/`publish_imu_tf`; enables `publish_rgb`; sets
`depth_mode: PERFORMANCE`, `pub_resolution: CUSTOM` @ `pub_downscale_factor: 2.0`,
`pub_frame_rate: 30.0`.

## Consumed `stack_params.yaml` keys

`camera_source`, `confidence_threshold`, `yolo_device`, `yolo_model`,
`obstacle_z_min`, `obstacle_z_max`, `use_lidar`, `sensors_config`,
`enable_wall_detector` — see each key's own `# Consumed by:` comment in
`stack_params.yaml`.

`wall_detector_node`'s own `wall_*` keys (all in `stack_params.yaml`, all
wired through `detection.launch.py` the same way as every other param
above):

| Key | Default | Purpose |
|---|---|---|
| `wall_input_topic` | `/zed2/zed_node/point_cloud/cloud_registered` | Source point cloud topic. |
| `wall_roi_x_max` | `6.0` m | Forward ROI cutoff. |
| `wall_roi_y_half_width` | `3.0` m | Lateral ROI half-width. |
| `wall_verticality_max_deg` | `20.0` deg | Max tilt off vertical for a candidate plane's normal to count as a "wall" (vs. floor/ceiling). |
| `wall_front_facing_max_deg` | `35.0` deg | Max `\|bearing\|` for a wall to count toward `/perception/front_clearance`. |
| `wall_corner_perp_tolerance_deg` | `25.0` deg | How close to exactly perpendicular two simultaneously-detected planes must be to both get `is_corner=true`. |
| `wall_merge_normal_cos_thresh` | `0.97` | `dot(normal_a, normal_b)` above this (both unit vectors) counts as "same direction" when deduping RANSAC's occasional single-wall-split-in-two. Deliberately tight so a genuine ~90° corner pair (dot ≈ 0) is never at risk. |
| `wall_merge_distance_thresh_m` | `0.08` m | Perpendicular offset gap below which (AND'd with the cosine threshold above) two candidate planes are treated as duplicates and merged (pooled inliers, refit) rather than published separately. |
| `wall_track_assoc_distance_thresh_m` | `0.4` m | Max frame-to-frame distance jump the tracker still treats as the same physical wall when associating this frame's (post-merge) detections to existing tracks. |
| `wall_track_assoc_bearing_thresh_deg` | `15.0` deg | Same association gate, bearing axis (both thresholds must pass). |
| `wall_track_hold_frames` | `5` | Consecutive unmatched frames a track's last smoothed value is kept alive (still published) before the track is dropped — covers brief occlusion without a visible pop in/out. |
| `wall_ema_alpha` | `0.3` | EMA smoothing factor applied to each tracked wall's distance/bearing/normal/centroid every matched frame (`smoothed = alpha*new + (1-alpha)*smoothed_prev`). **A reasoned starting point, not yet tuned against real sensor noise** — expect to retune once this runs against live ZED data for a while; see `wall_detector_node.py`'s own module docstring. |

### Plane merge + tracking (duplicate-detection + jitter fix)

Two independent post-processing stages, always run in this order, on every
frame, before anything is published:

1. **`merge_duplicate_walls()`** — pairwise-compares each frame's raw RANSAC
   plane candidates (normal similarity AND inter-plane offset gap, both
   thresholds above) and collapses near-coplanar duplicates (iterative
   re-segmentation occasionally splits one noisy real wall into two or more
   overlapping candidates) into a single plane, refit through the pooled
   inlier points. Runs **before** corner tagging. A genuinely perpendicular
   corner pair's normals dot to ~0, nowhere near `wall_merge_normal_cos_thresh`,
   so corner detection is structurally not at risk from this step.
2. **`WallTracker`** — associates each frame's (post-merge) walls to
   existing tracks by (distance, bearing) proximity (never by array index,
   which is not stable frame to frame), then EMA-smooths matched tracks'
   distance/bearing/normal/centroid (`wall_ema_alpha`). Bearing is blended
   through `atan2`-based wraparound handling so a track near the ±180°
   bearing seam doesn't spike; the blended normal is re-normalized to unit
   length after every update (a linear blend of two unit vectors isn't one).
   A brand-new track is seeded directly, unsmoothed, on its first frame.
   Unmatched tracks hold their last smoothed value for up to
   `wall_track_hold_frames` frames before being dropped. Each wall in a
   corner pair gets fully independent tracked state/EMA — there is no shared
   "corner angle" quantity, which would double-lag relative to smoothing the
   two wall normals directly.

Every consumer of this node's output — `WallDetection` entries,
`/perception/front_clearance`, and the `/perception/wall_markers` Foxglove
markers (which use stable per-track marker ids, `Marker.DELETE`'d
individually when a track is dropped, rather than relying on a shrinking
`MarkerArray` to implicitly clear stale ids) — reflects the merged+smoothed
values, never a raw single-frame detection.

**Statically/synthetically verified** (`f1tenth_perception/test/test_wall_detector.py`,
17 cases): merge correctly collapses coplanar pairs and 3-way splits while
leaving a ~90° corner pair separate; EMA seeds unsmoothed and smooths
correctly on match; bearing smoothing across the ±180° wrap doesn't spike;
an unmatched track holds for exactly `wall_track_hold_frames` frames then
drops; the tracked normal stays unit-length after every update; a corner
pair's two tracks stay independent (no EMA cross-contamination). **Pending
live hardware verification**: `wall_ema_alpha=0.3` is a starting point, not
tuned against real ZED sensor noise; whether the merge thresholds
(`0.97`/`0.08m`) hold up against real (not synthetic) RANSAC output; overall
jitter reduction "looks right" in Foxglove against a real wall.

## Known limitations

- **Detection-to-depth sync latency comment is stale/unverified.**
  `detection_3d_node.py`'s own comment justifying `sync_queue_size=60`
  claims YOLO inference finishes "~1s later on this hardware" — this is a
  hand-written guess, not a measurement, and it doesn't match an earlier
  latency audit's own measured **~0.41–0.43s** total pipeline figure
  (capture → MPC receipt). A dedicated instrumentation pass to break this
  down stage-by-stage was planned but this doc can't confirm whether it was
  ever run to completion — the comment is unchanged from before that plan
  existed. Treat `sync_queue_size`'s justification as directionally correct
  (comfortably oversized vs. whatever the real latency is) but the specific
  "~1s" figure as unverified.
- **`camera.launch.py`/`detection.launch.py`/`perception.launch.py` all
  still say "6 stack-wide branching args"** in their module docstrings —
  stale by one; the actual count is 5 as of the `safety_stop_controller`
  retirement (`enable_safety_stop` was the 6th). Doesn't affect behavior
  (these files read `camera_source` correctly either way), just a leftover
  comment the code-analysis/fixes pass didn't happen to touch.
- `yolo_detector_node`'s `cpu_affinity`/`nice` (and the other two ZED-path
  nodes') are machine-specific launch args with no `stack_params.yaml`
  default — a fresh deployment needs its own core-id choice, not a
  copy-paste of this repo's defaults (`8,9`/`6,7`), which were tuned against
  one specific Jetson's contention profile.
- `zed_msgs`' own service surface (`reset_odometry`, `enable_obj_det`, SVO
  recording, etc.) is vendored but entirely unused here — position
  tracking/mapping/object-detection are structurally disabled by
  `zed2_perception.yaml`, not just left at defaults.
