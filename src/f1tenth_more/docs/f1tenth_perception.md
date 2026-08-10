# f1tenth_perception

Camera bringup (ZED2 or webcam, mutually exclusive), Hokuyo LiDAR bringup,
and the YOLO 2D-detection → 3D-fusion → ground-plane-obstacle pipeline that
feeds both `mpc_controller`'s soft-avoidance and `f1tenth_behavior`'s
`IsObstacleDetected`/`IsProximityTooClose` conditions.

## Nodes

| Node | Subscribes | Publishes | Notes |
|---|---|---|---|
| `yolo_detector_node` | `/camera/image_raw` | `/camera/detections` (`vision_msgs/Detection2DArray`), `/camera/image_annotated` | Ultralytics YOLO (TensorRT `.engine` or `.pt`), `device`/`confidence_threshold`/`model_path` params. Header is copied verbatim from the input image (capture-time propagation, not re-stamped at publish). |
| `detection_3d_node` | `/camera/detections`, `/zed2/zed_node/depth/depth_registered`, `.../depth/camera_info` | `/camera/detections_3d` (`vision_msgs/Detection3DArray`), `/camera/detection_markers` (`MarkerArray`) | `message_filters.ApproximateTimeSynchronizer` (`sync_slop=0.1s`, `sync_queue_size=60`). Back-projects 2D box + depth → 3D pose via pinhole model, transforms into `output_frame` (default `zed2_left_camera_frame`) via tf2. Only built when `camera_source == 'zed'`. |
| `obstacle_projector_node` | `/camera/detections_3d` | `/perception/obstacles_2d` (`f1tenth_messages/Obstacle2DArray`) | tf2 into `output_frame` (default `base_link`), height-band filter (`obstacle_z_min`/`obstacle_z_max`), radius filter (`min_obstacle_radius`/`max_obstacle_radius`), merges close detections (`obstacle_merge_distance`). ZED-only, depends on `detection_3d_node`'s output. |
| `front_depth_monitor_node` | `/zed2/zed_node/depth/depth_registered` | `/perception/front_distance` (`std_msgs/Float32`) | Raw ZED depth, center-crop percentile (`center_fraction`, `front_distance_percentile`) — **deliberately independent** of the YOLO/detection pipeline above, by design: it's the front half of `f1tenth_behavior`'s `IsProximityTooClose` last-resort check, which must keep working even if YOLO is degraded/lagging. ZED-only. |

## Launch files

| File | Purpose |
|---|---|
| `camera.launch.py` | Exactly one of `{zed_wrapper's zed_camera.launch.py, v4l2_camera_node}`, selected by `camera_source`. Both remapped onto the canonical `/camera/image_raw` + `/camera/camera_info` so `detection.launch.py` is camera-agnostic. Also publishes the static `base_link → zed2_camera_link` TF when `camera_source=zed` (0.12m ahead, 0.15m above `base_link`, forward-facing). |
| `lidar.launch.py` | `urg_node`, gated by `use_lidar` (default `false`), config from `f1tenth_bringup/config/sensors.yaml`. **Not included by `stack_bringup.launch.py` directly** — only reachable via `f1tenth_navigation/nav2.launch.py` (i.e. only when `enable_nav2=true`) or this package's own standalone `perception.launch.py`. |
| `detection.launch.py` | `yolo_detector_node` (always) + `detection_3d_node`/`obstacle_projector_node`/`front_depth_monitor_node` (only when `camera_source == 'zed'`, since all three need ZED depth). Declares per-node `cpu_affinity`/`nice` launch args (machine-specific, not in `stack_params.yaml`), added by the perception-optimization pass after the latency audit found `/camera/detections` lagging `/camera/image_raw` by ~278ms, root-caused to CPU/executor contention rather than transport/QoS. |
| `perception.launch.py` | Standalone entry point bundling `camera.launch.py` + `lidar.launch.py` + `detection.launch.py` for exercising perception in isolation — `stack_bringup.launch.py` brings up the same three pieces itself rather than including this file. |

## Config

`config/zed2_perception.yaml` — ZED wrapper param overlay: disables
`publish_status`/`publish_stereo`/`pos_tracking_enabled`/`mapping_enabled`/
`od_enabled`/`publish_imu_raw`/`publish_imu_tf`; enables `publish_rgb`; sets
`depth_mode: PERFORMANCE`, `pub_resolution: CUSTOM` @ `pub_downscale_factor: 2.0`,
`pub_frame_rate: 30.0`.

## Consumed `stack_params.yaml` keys

`camera_source`, `confidence_threshold`, `yolo_device`, `yolo_model`,
`obstacle_z_min`, `obstacle_z_max`, `use_lidar`, `sensors_config` — see each
key's own `# Consumed by:` comment in `stack_params.yaml`.

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
