# f1tenth_perception — local memory

Contracts/invariants only; narrative + UNVALIDATED register: `README.md`.

## Two ladders
Camera→detection (ZED only): `camera.launch.py` → `/camera/image_raw` → `yolo_detector_node` →
`/camera/detections` [+`/camera/detection_masks` if task `segment`] → `detection_3d_node`
(+depth) → `/camera/detections_3d` → `obstacle_projector_node` → `/perception/obstacles_2d`.
`front_clearance_node` taps depth, masks, obstacles in parallel.
LiDAR→clearance: `urg_node` `/scan` → `lidar_front_wall_node`, `wall_distance_node`,
`swept_clearance_node`, `obstacle_clearance_node`; independent; best-effort QoS so nothing
back-pressures the driver.

## Placement law
Each `/scan` consumer is its own supervisor component; none may sit under `perception` or inside
`lidar.launch.py` — stopping a component SIGINTs every launch file in it, `urg_node` included,
blinding e-stop `IsProximityTooClose`. Asserted by
`f1tenth_bringup/test/test_{lidar_front_wall,wall_distance,obstacle_clearance}_component.py`.
Three gate on `IfCondition(use_lidar)`; `swept_clearance.launch.py` self-gates on the `use_lidar`
param.
⚠️ `nav2.launch.py:142` also includes `lidar.launch.py`: with `enable_nav2:=true` a second
`urg_node` starts on the same hardcoded IP. Dormant only while `enable_nav2` is false.

## Six "clearance" signals
Definitions, sign conventions and the "an arc length is not a gap" warning live in `README.md`
(§ *The signals that are all called something like "clearance"*). Anchor there; never restate or
compare a swept value to a wall distance.

## Wires
Pub `/camera/*`: `detections`, `image_annotated`, `detection_masks`, `detections_3d`,
`detection_markers`. Pub `/perception/*`: `obstacles_2d`, `front_distance`, `front_wall`,
`front_clearance` (+`/config`), `front_blocked`, `front_wall_confirmed`, `lidar_front_wall`,
`lidar_front_wall_virtual`, `d_wall` (+`/segment`, `/psi_correction`, `/gate_margin`,
`/swept_arc`), `swept_clearance` (+`/lidar`, `/camera`, `/steering`); plus `/obstacle_clearance`,
`/safety/event`.
Sub: `/scan`, `/camera/image_raw`, `/camera/{detections,detection_masks,detections_3d}`,
`/zed2/zed_node/depth/{depth_registered,camera_info}`, `/odometry/filtered`,
`/ackermann_drive`, `/tf_static`, plus cross-package inputs `/mpc/wall_track`
(`mpc_controller`) and `/costmap/front_clearance` (`f1tenth_costmap` `seed_topic`).
`MPC_corr` derives its correction topic from `wall_distance_output_topic`; renaming that key
silently unhooks the corridor correction.

## Params
Every key is `stack_params.yaml`-owned and prefixed (`wall_distance_*`, `glass_*`,
`front_clearance_*`, `swept_clearance_*`, `lidar_front_wall_*`, `yolo_*`); launch args keep the
prefix, nodes take bare names. Pass every value as `ParameterValue(..., value_type=…)`; else
`'False'` infers truthy, `'10'` an int. `camera_source` is read in plain Python,
not declared: CLI override ignored, only the YAML works. Footprint front edge is
`f1tenth_description.vehicle_frames.base_front_x()` (URDF), not a param; rear/half-width are
`swept_clearance_body_*`. Affinity is a `taskset -c` prefix, never self-pinning; **core 4 is
reserved** for the behavior executor and e-stop.

## Not retunable from launch
`detection_3d_node`: `output_frame zed2_left_camera_frame`, sync 0.1/queue 60, sigmas, label/marker
sizes. `obstacle_projector_node`: merge 0.2 m, radius 0.03–1.5 m. `front_clearance_node`:
`wall_confirm_scan_max_age_s` 0.3, all 10 publisher names. `lidar_front_wall_node`: `seed_topic`,
`base_frame`. `wall_distance_node`: `base_frame`. `obstacle_clearance_node`: `safety_event_topic`
never passed.

## Trust level
`swept_clearance_node` has never run on the car; `wall_distance_*`/`glass_*` are
powered-down-era code; `geom_*`, every `# SYNTHETIC` value and the YOLO exclusion rectangle are
reasoned or screenshot-derived, not measured.
