# Phase 0: the real campaign-final

Checked 2026-10-16 02:20–02:45 on linus after `git fetch --all --prune` (superproject and every submodule).

## Source commit

| Item | Value |
|---|---|
| Commit in the newest car bags | `e3793e47f595c501e0c9796e93791faf0cc9edef` (2026-10-15, branch campaign-final, **dirty tree**), and `736d46fe14` (2026-10-14 bags) |
| On any remote branch? | **No.** `git ls-remote origin` lists only main, campaign-final 7b17e2f, jazzy 3dbe37f (+ archive/evidence/jazzy-p* tags). `git fetch origin e3793e47…` → `upload-pack: not our ref`. Not on linus or /HDD2T either |
| Used as source of truth | **origin/campaign-final 7b17e2f** (2026-10-14 13:50, "docs: repo reorganisation plan") |
| How far behind the car | Unknown in commits: the Orin commits after `3ffe245` were never pushed, so a `git log` comparison is impossible. origin has 3 commits the Orin lacks (`cdfd736`, `a8a61db`, `7b17e2f`: campaign data + docs), so the two have **diverged** |
| Evidence of what is missing | The car's params snapshot (`car_params_2026-10-15.yaml`, copied from the 2026-10-15T15-18-27 run) differs from 7b17e2f's `stack_params.yaml` in 160 lines (`car_params_vs_7b17e2f.diff`). New keys on the car: perception_stage_timing front_clearance_scene_mask_enabled front_clearance_scene_mask_rate_hz front_clearance_scene_mask_floor_tol_m front_clearance_scene_mask_ceiling_min_z_m front_clearance_scene_mask_min_depth_m front_clearance_scene_mask_max_depth_m front_clearance_scene_mask_strip_half_width_m front_clearance_scene_mask_self_polygons front_clearance_scene_mask_occluded_polygons front_clearance_scene_mask_image_topic use_wall_tracker wall_distance_max_age_s lidar_front_wall_wall_tracker_front_max_coast_sec mission_logger_record_depth mission_logger_record_annotated_image front_clearance_scene_mask_cloud_stride front_clearance_scene_mask_column_percentile front_clearance_scene_mask_column_min_px lidar_front_wall_stereo_columns_max_age_s lidar_front_wall_wall_obs_tol_base_m lidar_front_wall_wall_obs_tol_quad_per_m2 lidar_front_wall_wall_obs_disagree_reject_frac lidar_front_wall_wall_obs_stereo_min_columns lidar_front_wall_wall_default_height_m lidar_front_wall_lidar_sigma_base_m lidar_front_wall_lidar_sigma_per_m lidar_front_wall_stereo_sigma_base_m lidar_front_wall_stereo_sigma_quad_per_m2 lidar_front_wall_min_span_for_angle_m lidar_front_wall_stereo_confirm_hits lidar_front_wall_stereo_confirm_window lidar_front_wall_role_hysteresis_rad lidar_front_wall_stereo_front_max_normal_rad lidar_front_wall_front_strip_half_width_m wall_clear_distance_m lidar_front_wall_wall_marker_thickness_m . Changed default: `enable_scene_labels` false → **true**. Plus TF: laser z 0.20 → 0.19, ZED z 0.15 → 0.205 with pitch -2.083° (bag TF). Not in 7b17e2f at all: multi-wall tracker (`/perception/walls`), wall fusion, scene mask, inferred floor, stage timing, depth recording |

Bag commits seen in October (manifests in ~/f1tenth_archive/mission_row):

| Bag date | Branch | Commit | On origin? |
|---|---|---|---|
| 10-01 | test-logging | 573f313 | yes |
| 10-08 | main / test-campaign | e47e646 / 1d00853 | yes |
| 10-13 09:17 | campaign-final | e9e5ade | yes |
| 10-13 11:10 | campaign-final | 1bb5ea2 | yes |
| 10-13 15:05 | campaign-final | 3ffe245 | yes |
| 10-14 13:11 | campaign-final | **736d46f** | **no** |
| 10-15 15:18 | campaign-final | **e3793e4** (dirty) | **no** |

## Submodules pinned by campaign-final 7b17e2f

| Submodule | Pin | Reachable on | jazzy branch pin | Fork has a `jazzy` branch? |
|---|---|---|---|---|
| vesc (sjckness/vesc) | 2848a9f | origin/jazzy, origin/scene-graph | 2848a9f (same) | yes, = the pin |
| ackermann_mux (sjckness/ackermann_mux) | 42b7cd7 "nav2 not working, odometry not bad" (config yaml only) | origin/humble-jetson | b3c0b08 (= origin/jazzy = foxy-devel) | yes, but 1 commit **behind** the pin (42b7cd7 is b3c0b08 + 1 yaml change) |
| teleop_tools (f1tenth/teleop_tools, upstream) | 4337558 | origin/foxy-devel, origin/humble-devel | 4337558 (same) | no (upstream repo; local-only `jazzy` branch in the main checkout = 4337558) |
| transport_drivers (ros-drivers, upstream) | d3f510c (tag 1.2.0) | origin/humble, origin/main | **removed** on jazzy (3d545db: apt serial_driver/io_context instead) | n/a |
| zed_ros2_wrapper (stereolabs, upstream) | b5844a8 (tag humble-v4.2.5) | origin/master | 86171bc (tag v5.4.1) | n/a (upstream) |
| zed-ros2-interfaces (stereolabs, upstream) | e970087 (tag 5.3.0) | origin/master | e970087 (same) | n/a |
| husarion_gz_worlds | – (not on campaign-final) | – | 87271ef (jazzy only, Gazebo sim) | n/a |

All campaign-final pins are fetchable. The Orin's own submodule state on 2026-10-15 is unknown (the manifest records only the superproject commit).

## Consequence for every report
The port (`jazzy-campaign`) is built from **7b17e2f**, older than the car. When the Orin's campaign-final is pushed (`todo_for_andreas.md` §1), the port has to be brought forward to it.
