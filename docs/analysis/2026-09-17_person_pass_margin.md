# Passing a standing person: field-of-view dropout and the person margin (2026-09-17)

Work order Part 1. Result: **no candidate person margin (0.2 / 0.3 / 0.4 m)
keeps a 0.25 m body gap at every lateral offset once the person drops out of
the camera's view, so `obstacle_class_margin_m` stays `{}` (no m\*).**

Rig numbers are nominal-model only and reproduced by
`docs/analysis/person_pass_margin.py`; the scenario is pinned in
`src/f1tenth_control/mpc_controller/test/test_person_pass_closed_loop.py`.

## 1. How a person leaves the MPC's obstacle list (code facts)

| Stage | Behaviour | Where |
|---|---|---|
| YOLO | publishes a `Detection2DArray` every frame, empty or not | `yolo_detector_node.py` `self.det_pub.publish(out)` after the per-box loop |
| detection_3d_node | publishes a `Detection3DArray` per synced frame; skips the frame only on missing camera_info or a TF failure | `_synced_callback` |
| obstacle_projector_node | empty detections -> publishes `[]`; TF failure -> skips the frame; no persistence, within-frame merge only | `_detections_callback`, module docstring |
| MPC_corr | `obstacles_2d_callback` REPLACES `obstacles_global_live` with each message, converted to odom at receipt; **no age check**, so if messages stop the last list is kept indefinitely | `MPC_corr.py` `obstacles_2d_callback` |
| compute_local_target | after the deflecting obstacle disappears, the lookahead deflection decays linearly over `deflection_decay_ticks` = 5 control ticks (0.5 s) | `MPC_corr.py` `deflection_decay_ticks` |
| solver `w_obs` | no memory: uses the current list only | `mpc_solver.py` |

So a person leaves the obstacle list on the first detection frame after they
leave the image, with only the 0.5 s target-deflection coast behind it.

**Other sources.** Semantic tracks do not feed the MPC's obstacles (mpc_corr
subscribes to `/perception/obstacles_2d` and `/costmap/boundaries` only).
LiDAR reaches the MPC only through `/costmap/boundaries`, built by
costmap_boundary_node from slam_toolbox's `/slam/map` (map_update_interval
5.0 s), not from raw scans; mpc_corr's own `/scan` subscription feeds the wall
tracker only.

**LiDAR safety stop at the addendum-C distances.** IsProximityTooClose trips
when any raw range in the ±45° front cone, or on the sides/rear, is below
0.15 m (the LiDAR sits 0.12 m ahead of base_link on the current mount). The
closest camera-measured passes in the archive were 0.56–1.58 m centre
distance: roughly 0.2–1.3 m from the LiDAR to the body surface. **Inference:** the stop would
not have tripped on any of them; it is a contact backstop, not avoidance. The
one map-frame estimate at 0.12 m (2026-09-04T15-23-48, a walking person) is
unreliable, as noted in addendum C.

## 2. Camera half field of view

No bag records `camera_info`. Intrinsics recovered from 32 single-detection
`/camera/detections` + `/camera/detections_3d` pairs in
`2026-09-03T12-45-06_mission-bottle_then_person` (read-only): fx 262.4 px from
both box width and position, cx 318.7 px, image 640 px (downscaled 2× per
`zed2_perception.yaml`). Half-FOV: left atan(318.7/262.4) = 50.54°, right
atan(321.3/262.4) = 50.76°; 50.54° used.

## 3. Rig: goal_distance 4 m, person 0.50 × 1.75 m standing 2 m ahead

Detection frames at 8.2 Hz (measured, 2026-09-01 analysis), bearing from the
camera (0.12 m ahead of base_link), /drive clamp +1.0 / 0.0 m/s. Body gap =
centre distance − 0.25 − car_radius 0.20.

### With dropout at the real half-FOV (50.53 deg)

| obstacle | offset m | min centre m | min body gap m | dropout tick | peak cross-track m | peak speed m/s | goal reached |
|---|---|---|---|---|---|---|---|
| legacy | 0.0 | 1.673 | +1.223 | - | 0.019 | 0.28 | no |
| legacy | 0.3 | 0.686 | +0.236 | 24 | 0.394 | 0.82 | 7.3 s |
| legacy | 0.6 | 0.801 | +0.351 | 20 | 0.216 | 0.75 | 6.9 s |
| footprint+0.0 | 0.0 | 0.245 | -0.205 | 33 | 0.249 | 0.76 | 7.1 s |
| footprint+0.0 | 0.3 | 0.418 | -0.032 | 29 | 0.119 | 0.68 | 6.9 s |
| footprint+0.0 | 0.6 | 0.613 | +0.163 | 27 | 0.013 | 0.70 | 7.0 s |
| footprint+0.2 | 0.0 | 0.340 | -0.110 | 32 | 0.354 | 0.90 | 7.3 s |
| footprint+0.2 | 0.3 | 0.503 | +0.053 | 26 | 0.208 | 0.74 | 6.9 s |
| footprint+0.2 | 0.6 | 0.663 | +0.213 | 25 | 0.068 | 0.69 | 6.9 s |
| footprint+0.3 | 0.0 | 0.402 | -0.048 | 32 | 0.417 | 0.95 | 7.5 s |
| footprint+0.3 | 0.3 | 0.542 | +0.092 | 25 | 0.254 | 0.78 | 6.9 s |
| footprint+0.3 | 0.6 | 0.701 | +0.251 | 24 | 0.109 | 0.69 | 6.9 s |
| footprint+0.4 | 0.0 | 0.426 | -0.024 | 37 | 0.434 | 0.97 | 8.0 s |
| footprint+0.4 | 0.3 | 0.567 | +0.117 | 24 | 0.292 | 0.81 | 7.0 s |
| footprint+0.4 | 0.6 | 0.732 | +0.282 | 22 | 0.143 | 0.72 | 6.9 s |

### Attribution only: the person never leaves view

| obstacle | offset m | min centre m | min body gap m | dropout tick | peak cross-track m | peak speed m/s | goal reached |
|---|---|---|---|---|---|---|---|
| legacy | 0.0 | 1.673 | +1.223 | - | 0.019 | 0.28 | no |
| legacy | 0.3 | 0.845 | +0.395 | - | 0.564 | 0.88 | 7.0 s |
| legacy | 0.6 | 0.959 | +0.509 | - | 0.371 | 0.78 | 6.8 s |
| footprint+0.0 | 0.0 | 0.262 | -0.188 | - | 0.262 | 0.76 | 7.1 s |
| footprint+0.0 | 0.3 | 0.435 | -0.015 | - | 0.135 | 0.68 | 6.9 s |
| footprint+0.0 | 0.6 | 0.618 | +0.168 | - | 0.018 | 0.70 | 7.0 s |
| footprint+0.2 | 0.0 | 0.385 | -0.065 | - | 0.384 | 0.90 | 7.1 s |
| footprint+0.2 | 0.3 | 0.555 | +0.105 | - | 0.257 | 0.74 | 6.8 s |
| footprint+0.2 | 0.6 | 0.696 | +0.246 | - | 0.096 | 0.69 | 6.9 s |
| footprint+0.3 | 0.0 | 0.443 | -0.007 | - | 0.450 | 0.95 | 7.3 s |
| footprint+0.3 | 0.3 | 0.620 | +0.170 | - | 0.324 | 0.78 | 6.8 s |
| footprint+0.3 | 0.6 | 0.751 | +0.301 | - | 0.153 | 0.69 | 6.9 s |
| footprint+0.4 | 0.0 | 0.513 | +0.063 | - | 0.529 | 0.97 | 7.8 s |
| footprint+0.4 | 0.3 | 0.685 | +0.235 | - | 0.392 | 0.81 | 6.8 s |
| footprint+0.4 | 0.6 | 0.812 | +0.362 | - | 0.215 | 0.72 | 6.8 s |

## 4. Reading it

- **Facts.** With dropout, the best margin (0.4) still reaches a body gap of
  −0.024 m with the person on the line and +0.117 m at 0.3 m offset. No
  margin in {0.2, 0.3, 0.4} passes the ≥ 0.25 m criterion at every offset, so
  there is no m\*. Dropout happens on every completed pass, 2.0–3.7 s in.
- **Facts.** Without dropout the gaps are larger (0.4: +0.063 / +0.235 /
  +0.362) but still fail the criterion on the line. Dropout costs roughly
  0.05–0.12 m.
- **Facts.** Footprint mode with no margin — the shipped default since
  4e0391d — gives −0.205 m on the line and −0.032 m at 0.3 m: disk-model
  contact. Legacy radii never pass a person on the line (w_obs holds the car
  1.67 m away and the move does not complete) and keep +0.236 m at 0.3 m.
- **Inference.** Two effects stack: avoidance is a soft cost that passes close
  even while the person is visible, and once they leave the image the car
  steers back toward its line with nothing in the list. A margin alone cannot
  fix the second; an obstacle memory (holding a person's last odom position
  for a short time after they leave view) or a wider camera coverage would.
  Neither was changed here.
