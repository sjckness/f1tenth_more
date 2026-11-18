# Phase 4: end-to-end test on linus (Isaac base_room + the jazzy-campaign stack)

```bash
tools/e2e_test.sh            # about 4 min; results in output/jazzy_port/e2e/ (summary.md, results.jsonl, raw JSON)
```

One script runs the whole test. It starts `scripts/sim/sim_isaac.sh all --scene base_room` in its own session and runs the checks in `tools/sim/` against the live graph. It samples CPU, RAM and VRAM for the whole run. At the end it stops everything and checks that no Isaac container and no ROS process is left.

Ground truth always comes from the sim (`/sim/ground_truth/*`), never from the stack.

The final run is from 2026-10-16 11:14 (`e2e/`). **All 9 steps PASS**, and the stop was clean: no container and no ROS process left.

| step | result | numbers |
|---|---|---|
| 1 up | PASS | `/clock` 26 s after start (shader cache warm); stack ready (EKF output and first detection) 61 s after start |
| 2 graph | PASS | 127 topics, 61 nodes. The sim feeds `/scan`, `/odom`, `/sensors/imu/raw`, `/camera/image_raw`, depth and `/clock`. The 12 subscribed-but-unpublished topics all lack a publisher on the car too, or are optional (phase3.md §6) |
| 3 hz | PASS | see the table below |
| 4 tf | PASS | 19 edges, one parent per frame, one /tf_static message per static edge. `odom->base_link` matches the EKF output 50 ms earlier in 351/351 TF messages (`transform_time_offset`). Laser (0.305, 0, 0.20), ZED link (0.2399, 0, 0.205, pitch -2.083°) and imu (identity) as expected |
| 5 perception | PASS | person: 210 2D detections in 15 s, score 0.93. 3D position error vs ground truth: median 0.10 m, max 0.12 m (tolerance 0.30). LiDAR front wall: 5.311 m measured vs ground truth (wall x 6.0 - car x): error 2.4 mm mean, 2.5 mm max (tolerance 0.10 m) |
| 6 mission | PASS | `drive_test_straight_2m` through `/mission/load_mission` + `/mission/start_mission`: IDLE → LOADED → RUNNING → COMPLETE (the 15 s countdown, then 5.5 s driving). Ground-truth travel 2.019 m (+1.9 cm). 9 `/slam/pose` updates while driving. Mission logger wrote and closed `~/f1tenth_sim_archive/complete/2026-10-16T09-16-08_mission-drive_test_straight_2m` (mcap bag, extract, manifest, snapshots) |
| 7 reset | PASS | `/reset_all` (reset_manager → `/sim/reset`) in 5.3 s (SLAM 4.3 s of it). Car back 1.6 cm from the start pose, which is creep (isaac_ws D25). First EKF outputs after the service: local (-0.033, 0.000, -1.1°), global (-0.002, 0.000, -0.06°) |
| 8 person | PASS (with a finding) | `go_to_person`: COMPLETE, MPC driving, the car turns 30° toward the person. **It stops 1.49 m from the person's body (ground truth), not at the commanded 0.6 m gap.** The sim person is T-posed, and its 1.4 m box width counts as the object width (D23, isaac_ws D26) |
| 9 perf | PASS | RTF 0.993 mean, 0.96 min (18 × 10 s windows) |

## Topic rates (step 3, 15 s window, reader QoS matched to each publisher)

| topic | Hz | car / configured |
|---|---:|---|
| /clock | 59.5 | sim physics 60 Hz |
| /scan | 39.6 | car 39.9 Hz (bag) |
| /odom | 49.5 | car 50 Hz |
| /sensors/imu/raw | 49.5 | car 50 Hz |
| /camera/image_raw | 14.9 | sim 15 Hz (car publishes 30, bag camera_info 13.7) |
| /zed2/zed_node/depth/depth_registered | 14.9 | sim 15 Hz |
| /odometry/filtered | 45.9 | |
| /ekf_global/odometry/filtered | 39.3 | |
| /camera/detections, /camera/detections_3d | 14.9 | every camera frame |
| /perception/lidar_front_wall | 39.6 | every scan |
| /costmap/front_clearance | 19.9 | |
| /ackermann_drive | 9.9 | mux at the MPC rate |
| /diagnostics/system_status | 1.0 | |
| /slam/pose | 0.2 parked | motion-gated (D21); 9 updates in 5.5 s while driving |

## Performance (whole run, 5 s samples, linus: i7-9700K 8 cores, 32 GB, RTX 5050 8 GB)

| | mean | p95 | max |
|---|---:|---:|---:|
| stack CPU (100 % = one core) | 287 % | 361 % | 387 % |
| stack RSS | 4.8 GB | 5.9 GB | 5.9 GB |
| Isaac CPU | 129 % | 134 % | 233 % |
| Isaac RSS | 4.4 GB | 4.7 GB | 4.7 GB |
| host CPU (all 8 cores) | 73 % | 86 % | 94 % |
| host RAM used | 16.7 GB | 17.4 GB | 17.5 GB |
| VRAM (Isaac + YOLO on CUDA) | 3.8 GB | 4.0 GB | 4.0 GB of 8.1 |
| GPU utilisation | 27 % | 40 % | 40 % |
| RTF | 0.993 | | min 0.96 |

**MPC solve time.** `MpcSolverStatus.solve_dt_sec` reads 0.0 in every sim tick (51/51 and 15/15). MPC_corr measures it with the node clock, which in sim is `/clock`, and a single-threaded executor cannot advance that clock inside the callback (proposal P21).

As a replacement, the mpc_corr process CPU while RUNNING:
- straight mission: 3.02 s CPU in 5.5 s (55 % of a core), 59 ms per published tick;
- go_to_person: 49 %, 76 ms per tick.

These are upper bounds: they include the node's scan, odometry and perception callbacks. The control period is 0.1 s, the MPC ran at 10 Hz (51 ticks in 5.5 s at RUNNING), and no solve failed.

## Findings of Phase 4

| finding | where | status |
|---|---|---|
| parked Isaac car creeps backwards (-6.5 mm/s, -0.005 rad/s yaw, wheels turn against zero targets) | isaac_ws D25 | 3 fixes tried, none worked; logged (D20) |
| Isaac person is T-posed, so `go_to_person` stops 0.9 m further away than commanded | isaac_ws D26, D23 | logged; sim-side fix in todo §8 |
| MPC solve time measured on the node clock | P21 | proposal (behaviour-neutral on the car) |
| Nav2 servers keep `use_sim_time: false` if `enable_nav2: true` | P22 | proposal (Nav2 is off by default) |
| mission bag recorder on wall clock in sim | P23 | proposal |
| sim_isaac.sh: DS process tree leaked; SIGINT ignored by the background launch; `--net eth` took the Wi-Fi route; Ctrl+C ignored during the /clock wait | D24 + log | fixed |
| supervisor: `use_sim_time:=true` alone made every component fail | D24 (C1) | fixed, with a unit test |

Earlier runs, kept only in the log: run 1 (hz/TF/reset thresholds), run 2 (reset threshold), run 3 (best-effort hz reader). Each failure was in the test, not in the stack, and is explained in `log.md` and D21/D22/D24.
