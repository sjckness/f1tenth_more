# cpu_test summary

```
argv: tools/cpu_test.sh 
date: 2026-10-16T14:08:23+02:00
host: Linux jetson-orin 5.15.185-tegra #2 SMP PREEMPT Fri May 29 12:19:10 CEST 2026 aarch64 aarch64 aarch64 GNU/Linux
git:  9c46add 21 modified file(s)
dry_run: 0  no_motion: 0  duration: 60
bag: /home/fabiocar/raw_data/corridor10m_20261015T171753
```

## Phase a — stationary, logger OFF

CPU source: `/proc via cpu_test_probe.py (pidstat not installed)`

### Board (tegrastats, 1 s)

| samples | CPU total % (median) | GPU % med | GPU % max | tj C med | tj C max | RAM MB max | CPU mW | SYS mW | throttle lines |
|---|---|---|---|---|---|---|---|---|---|
| 66 | 718.5 | 0.0 | 0.0 | 52.9 | 53.3 | 10629 | 3657.0 | 5782.0 | 0 |

Per core, median %: `0:49.0, 1:46.5, 2:44.0, 3:70.0, 4:86.0, 5:100.0, 6:30.5, 7:34.0, 8:54.5, 9:58.0, 10:65.5, 11:65.5`

### CPU per node, top 10 (sum over its threads)

| node | CPU % med | CPU % p95 | threads | affinity | read KB/s | write KB/s |
|---|---|---|---|---|---|---|
| mpc_corr | 52.0 | 55.6 | 4 | 10,11 | 0.0 | 0.0 |
| behavior_executor_node | 50.6 | 55.2 | 6 | 4 | 0.0 | 0.0 |
| costmap_boundary_node | 48.2 | 51.4 | 4 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| wall_distance_node | 42.5 | 45.0 | 5 | 5 | 0.0 | 0.0 |
| semantic_layer_node | 40.6 | 43.4 | 5 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| foxglove_bridge | 39.7 | 44.4 | 17 | 3 | 0.0 | 0.0 |
| reset_manager | 34.7 | 37.0 | 9 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| lidar_front_wall_node | 34.0 | 35.9 | 5 | 5 | 0.0 | 0.0 |
| cpu_test_probe.py | 31.0 | 33.5 | 5 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| ekf_cost_observer_node | 30.6 | 33.0 | 4 | 2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |

CPU seen per core (sum of the thread samples that last ran there): `0:1894.0, 1:2124.6, 2:1864.6, 3:2412.4, 4:2999.2, 5:5530.1, 6:2028.2, 7:1983.3, 8:2243.4, 9:2218.0, 10:2750.8, 11:2533.9`

### Rates, measured outside any bag

| topic | messages | Hz |
|---|---|---|
| /scan | 2185 | 39.9 |
| /odometry/filtered | 2656 | 48.43 |
| /zed2/zed_node/depth/camera_info | 0 | -- |
| /perception/stereo_columns | 0 | -- |
| /perception/scene_labels | 0 | -- |
| /perception/walls | 547 | 9.98 |
| /mpc/diagnostics | 0 | -- |
| /diagnostics | 550 | 10.05 |
| /perception/virtual_wall/debug | 2184 | 39.88 |
| /perception/scene_labels/stats | 0 | -- |
| /mpc/status | 0 | -- |

Absent from the graph: `/mpc/diagnostics`

## Phase b — stationary, logger ON (recording)

CPU source: `/proc via cpu_test_probe.py (pidstat not installed)`

### Board (tegrastats, 1 s)

| samples | CPU total % (median) | GPU % med | GPU % max | tj C med | tj C max | RAM MB max | CPU mW | SYS mW | throttle lines |
|---|---|---|---|---|---|---|---|---|---|
| 62 | 718.0 | 0.0 | 0.0 | 53.0 | 53.6 | 11035 | 3657.0 | 5782.0 | 0 |

Per core, median %: `0:51.0, 1:48.0, 2:50.0, 3:48.0, 4:86.0, 5:100.0, 6:38.0, 7:39.0, 8:57.0, 9:54.0, 10:65.0, 11:65.5`

### CPU per node, top 10 (sum over its threads)

| node | CPU % med | CPU % p95 | threads | affinity | read KB/s | write KB/s |
|---|---|---|---|---|---|---|
| mpc_corr | 56.2 | 59.0 | 4 | 10,11 | 0.0 | 0.0 |
| behavior_executor_node | 51.6 | 56.2 | 5 | 4 | 0.0 | 0.0 |
| costmap_boundary_node | 47.2 | 50.2 | 4 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| wall_distance_node | 41.6 | 44.1 | 4 | 5 | 0.0 | 0.0 |
| semantic_layer_node | 41.4 | 45.1 | 5 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| reset_manager | 36.5 | 40.2 | 9 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| lidar_front_wall_node | 34.9 | 36.7 | 5 | 5 | 0.0 | 0.0 |
| cpu_test_probe.py | 32.2 | 35.7 | 6 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| ekf_cost_observer_node | 32.1 | 35.0 | 5 | 2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| foxglove_bridge | 31.2 | 35.8 | 18 | 3 | 0.0 | 0.0 |

CPU seen per core (sum of the thread samples that last ran there): `0:1661.4, 1:1494.4, 2:2061.8, 3:2412.9, 4:2903.6, 5:5431.0, 6:1761.9, 7:1815.4, 8:1922.6, 9:2559.6, 10:2962.3, 11:3006.2`

### Rates, measured outside any bag

| topic | messages | Hz |
|---|---|---|
| /scan | 2189 | 39.95 |
| /odometry/filtered | 2784 | 50.02 |
| /zed2/zed_node/depth/camera_info | 0 | -- |
| /perception/stereo_columns | 0 | -- |
| /perception/scene_labels | 0 | -- |
| /perception/walls | 547 | 10.0 |
| /mpc/diagnostics | 0 | -- |
| /diagnostics | 556 | 10.01 |
| /perception/virtual_wall/debug | 2187 | 39.93 |
| /perception/scene_labels/stats | 0 | -- |
| /mpc/status | 0 | -- |

Absent from the graph: `/mpc/diagnostics`

## Diff a -> b (what recording costs at rest)

| quantity | a | b | delta |
|---|---|---|---|
| CPU total % (median) | 718.5 | 718.0 | -0.5 |
| GPU % median | 0.0 | 0.0 | 0.0 |
| tj C median | 52.9 | 53.0 | 0.1 |
| RAM MB max | 10629 | 11035 | 406 |
| CPU % behavior_executor_node | 50.6 | 51.6 | 1.0 |
| CPU % costmap_boundary_node | 48.2 | 47.2 | -1.0 |
| CPU % cpu_test_probe.py | 31.0 | 32.2 | 1.2 |
| CPU % ekf_cost_observer_node | 30.6 | 32.1 | 1.5 |
| CPU % foxglove_bridge | 39.7 | 31.2 | -8.5 |
| CPU % mpc_corr | 52.0 | 56.2 | 4.2 |
| CPU % reset_manager | 34.7 | 36.5 | 1.8 |

