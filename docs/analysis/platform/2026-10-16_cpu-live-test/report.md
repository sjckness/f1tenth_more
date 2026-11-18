---
id: 2026-10-16_cpu-live-test
title: "Test CPU live sull'Orin (fasi a e b, auto ferma)"
area: platform
date: 2026-10-16
tags: [cpu, orin, pinning, logger, discovery-super-client, lidar]
modules: {f1tenth_perception: "0.0.1", mpc_controller: "0.0.0", f1tenth_logger: "0.0.1", f1tenth_costmap: "0.0.1", f1tenth_behavior: "0.0.1"}
branch: "orin/orin-wip-2026-10-22"
commit: "02cd0f2084da"
status: open
hypothesis: "Misurare CPU per nodo e per core con logger spento (a) e acceso (b); fasi c/d con moto non eseguite."
result: "CPU scheda mediana 719.5 % (a) e 718.0 % (b) su 1200 % con camera morta; core 5 saturo al 100 % (wall_distance + lidar_front_wall), p95 per scan 81 ms, max 144.2 ms; logger -1.5 %, +406 MB RAM."
decision: "3 azioni: sbloccare lidar_front_wall dal core 5, enable_foxglove:=false quando non serve, fissare i 4 nodi non pinnati (~157 %)."
raw_data: "git exp/2026-10-16_cpu-live-test:output/cpu_test/20261016T102307/ (11 file .log/.txt); exp/2026-10-16_cpu-live-test:output/cpu_test/20261016T102543/ (17 file .log/.txt); exp/2026-10-16_cpu-live-test:output/cpu_test/20261016T102801/ (17 file .log/.txt); exp/2026-10-16_cpu-live-test:output/cpu_test/20261016T103321/ (10 file .log/.txt); exp/2026-10-16_cpu-live-test:output/cpu_test/20261016T103455/ (17 file .log/.txt); exp/2026-10-16_cpu-live-test:output/cpu_test/live/ (11 file .log/.txt); +2 gruppi (vedi _inventory.md). Esterni: Orin: output/cpu_test/live/, /home/fabiocar/raw_data/corridor10m_20261015T171753; output/live/stack_*.log"
migrated_from: "orin/orin-wip-2026-10-22:output/live/report.md"
migrated_on: 2026-11-18
---
# Live CPU test 2026-10-16 — phases a, b (stationary). c/d NOT run: no motion allowed.

## 1. Summary
| smoke item | result |
|---|---|
| /scan ~40 Hz | **PASS** a 39.90 / b 39.95 |
| walls = 1/4 of virtual_wall/debug | **PASS** 9.98/39.88 = 0.250 |
| no "fallback"/"python backend" in stack logs | **PASS** empty in both (C backend) |
| scene_labels/health age < 1 s | **FAIL** topic never published (camera dead) |
| front_offset_m = 0.1756 | **n/a** topic exists, silent while stationary |
| tracker_ms, rect_ms, scene_labels ms, solve_time_ms | **n/a** never emitted (camera dead, no mission) |
| lidar_front_wall per-scan ms (only timing that ran) | p50 4.58-4.76, p95 9.0-81.5, max 144.2 |

Board CPU median **a 719.5 %**, **b 718.0 %** of 1200 %, car standing still and camera dead.
GPU **0.0 %** med and max both phases: no YOLO, no TensorRT, no depth. Throttle lines **0**.
tj max **53.6 C**. RAM max 10629 -> 11035 MB. CPU 3657 mW / SYS 5782 mW in both.

## 2. Top 10 by CPU, phase a -> b (%med / %p95, threads, affinity)
| node | a | b | thr | affinity |
|---|---|---|---|---|
| mpc_corr | 52.0 / 55.6 | 56.2 / 59.0 | 4 | 10,11 |
| behavior_executor_node | 50.6 / 55.2 | 51.6 / 56.2 | 5-6 | 4 |
| costmap_boundary_node | 48.2 / 51.4 | 47.2 / 50.2 | 4 | all 12 |
| wall_distance_node | 42.5 / 45.0 | 41.6 / 44.1 | 4-5 | 5 |
| semantic_layer_node | 40.6 / 43.4 | 41.4 / 45.1 | 5 | all 12 |
| foxglove_bridge | 39.7 / 44.4 | 31.2 / 35.8 | 17-18 | 3 |
| reset_manager | 34.7 / 37.0 | 36.5 / 40.2 | 9 | all 12 |
| lidar_front_wall_node | 34.0 / 35.9 | 34.9 / 36.7 | 5 | 5 |
| cpu_test_probe.py (the measurement itself) | 31.0 / 33.5 | 32.2 / 35.7 | 5-6 | all 12 |
| ekf_cost_observer_node | 30.6 / 33.0 | 32.1 / 35.0 | 4-5 | 2..11 |

## 3. Per core vs pinning (median %)
a `0:48.5 1:46.5 2:44 3:70 4:86 5:100 6:31 7:34 8:55 9:59 10:68 11:65`
b `0:51 1:48 2:50 3:48 4:86 5:100 6:38 7:39 8:57 9:54 10:65 11:65.5`
**Core 5 saturated at 100 % in both**: wall_distance (42) + lidar_front_wall (35) are pinned to it
alone = 77 % user + softirq; that is the 81 ms p95 / 144 ms max per-scan spike. Core 4 at 86 % is
behavior_executor alone. 10-11 = mpc_corr. YOLO's 8-9 (55-57 %) and detection's 6-7 (31-39 %) are
busy with *floaters*, not their own work: 4 unpinned nodes (costmap_boundary 48 + semantic_layer 41
+ reset_manager 36 + ekf_cost_observer 32 = ~157 %) roam all 12 cores and fill every gap.

## 4. a -> b: logger cost
Board **-1.5 %** (noise), RAM **+406 MB**, rates unchanged (/scan 39.90->39.95, walls 9.98->10.0,
odometry/filtered 48.43->50.02, /diagnostics 10.05->10.01). **Caveat: no bag was written** - the
logger records from mission LOADED, no mission ran, ~/f1tenth_archive/active/ got no new run. So
this is its *idle* cost; recording 56 topics is still unmeasured. Zero-rate in both phases: zed
depth/camera_info, stereo_columns, scene_labels(+/stats), /mpc/status; /mpc/diagnostics not in graph.

## 5. Verdict + 3 actions
Yesterday's 12 cores are **not** explained here: the heaviest consumer never ran. The ZED container
died (exit 1, "CAMERA NOT DETECTED", 14:02:04), GPU stayed at 0 %, YOLO/detection_3d/scene_labels
never ticked. 719 % with the camera half dead and the car still is a floor, not the answer.
1. **Unpin lidar_front_wall_node from core 5** (move to 6 or 7, at 31-39 %): frees ~35 % on the only
   saturated core, p95 per-scan should fall from 81 ms toward the 9 ms seen when core 5 is quiet.
2. **enable_foxglove:=false when nobody watches**: -31..-40 % and 18 threads off core 3.
3. **Pin or fix the 4 floaters (~157 %)**; reset_manager at 36 % / 9 threads while idle is the
   standout. Pair with OPENBLAS_NUM_THREADS=1 for the numpy nodes.
Subtract **~31 %** from every total: cpu_test_probe.py is itself the 9th heaviest process.

Paths: output/cpu_test/live/{summary.md,run.txt,phase_a/,phase_b/} · output/live/log.md ·
output/live/stack_nolog.log (phase a) · output/live/stack_log.log (phase b)
