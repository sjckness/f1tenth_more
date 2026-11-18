# cpu_test summary

```
argv: tools/cpu_test.sh 
date: 2026-10-16T10:28:01+02:00
host: Linux jetson-orin 5.15.185-tegra #2 SMP PREEMPT Fri May 29 12:19:10 CEST 2026 aarch64 aarch64 aarch64 GNU/Linux
git:  6cd1609 26 modified file(s)
dry_run: 1  no_motion: 1  duration: 30
bag: /home/fabiocar/raw_data/corridor10m_20261015T171753
```

## Phase a — stationary, logger OFF

CPU source: `/proc via cpu_test_probe.py (pidstat not installed)`

### Board (tegrastats, 1 s)

| samples | CPU total % (median) | GPU % med | GPU % max | tj C med | tj C max | RAM MB max | CPU mW | SYS mW | throttle lines |
|---|---|---|---|---|---|---|---|---|---|
| 35 | 186.0 | 0.0 | 0.0 | 47.6 | 48.2 | 7169 | 1201.0 | 4760.0 | 0 |

Per core, median %: `0:11.0, 1:5.0, 2:4.0, 3:5.0, 4:6.0, 5:15.0, 6:7.0, 7:8.0, 8:5.0, 9:6.0, 10:5.0, 11:4.0`

### CPU per node, top 10 (sum over its threads)

| node | CPU % med | CPU % p95 | threads | affinity | read KB/s | write KB/s |
|---|---|---|---|---|---|---|
| lidar_front_wall_node | 24.9 | 32.3 | 5 | 5 | 0.0 | 0.0 |
| cpu_test_probe.py | 4.8 | 25.9 | 3 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| bag | 1.9 | 2.9 | 4 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| depth_throttle_5hz | 1.0 | 1.0 | 2 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| mission_logger_node | 1.0 | 1.7 | 3 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| launch | 1.0 | 1.0 | 1 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |

CPU seen per core (sum of the thread samples that last ran there): `0:9.6, 1:2.9, 2:22.1, 3:23.1, 4:47.0, 5:309.0, 6:102.6, 7:139.6, 8:31.7, 9:29.7, 11:1.0`

### Rates, measured outside any bag

| topic | messages | Hz |
|---|---|---|
| /scan | 422 | 35.81 |
| /odometry/filtered | 396 | 32.61 |
| /zed2/zed_node/depth/camera_info | 69 | 7.7 |
| /perception/stereo_columns | 23 | 2.6 |
| /perception/scene_labels | 28 | 3.04 |
| /perception/walls | 43 | 3.69 |
| /mpc/diagnostics | 0 | -- |
| /diagnostics | 0 | -- |
| /perception/virtual_wall/debug | 193 | 16.34 |
| /perception/scene_labels/stats | 25 | 2.87 |
| /mpc/status | 0 | -- |

Absent from the graph: `/mpc/diagnostics`

## Phase b — stationary, logger ON (recording)

CPU source: `/proc via cpu_test_probe.py (pidstat not installed)`

### Board (tegrastats, 1 s)

| samples | CPU total % (median) | GPU % med | GPU % max | tj C med | tj C max | RAM MB max | CPU mW | SYS mW | throttle lines |
|---|---|---|---|---|---|---|---|---|---|
| 49 | 181.0 | 0.0 | 0.0 | 47.7 | 48.2 | 7327 | 1200.0 | 4760.0 | 0 |

Per core, median %: `0:13.0, 1:7.0, 2:10.0, 3:5.0, 4:6.0, 5:27.0, 6:19.0, 7:6.0, 8:10.0, 9:7.0, 10:10.0, 11:5.0`

### CPU per node, top 10 (sum over its threads)

| node | CPU % med | CPU % p95 | threads | affinity | read KB/s | write KB/s |
|---|---|---|---|---|---|---|
| lidar_front_wall_node | 28.4 | 41.5 | 9 | 5 | 0.0 | 0.0 |
| cpu_test_probe.py | 5.7 | 24.0 | 3 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| bag | 1.9 | 2.9 | 4 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| depth_throttle_5hz | 1.0 | 2.0 | 6 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| mission_logger_node | 1.0 | 1.8 | 2 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| bash | 1.0 | 1.0 | 1 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| launch | 1.0 | 1.0 | 1 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |

CPU seen per core (sum of the thread samples that last ran there): `0:85.2, 1:67.3, 2:20.2, 3:44.5, 4:52.1, 5:361.0, 6:22.0, 7:52.2, 8:23.2, 9:17.4, 10:1.0, 11:34.6`

### Rates, measured outside any bag

| topic | messages | Hz |
|---|---|---|
| /scan | 416 | 36.14 |
| /odometry/filtered | 390 | 32.75 |
| /zed2/zed_node/depth/camera_info | 69 | 7.7 |
| /perception/stereo_columns | 23 | 2.6 |
| /perception/scene_labels | 28 | 3.04 |
| /perception/walls | 28 | 2.6 |
| /mpc/diagnostics | 0 | -- |
| /diagnostics | 0 | -- |
| /perception/virtual_wall/debug | 372 | 32.3 |
| /perception/scene_labels/stats | 25 | 2.87 |
| /mpc/status | 0 | -- |

Absent from the graph: `/mpc/diagnostics`

## Diff a -> b (what recording costs at rest)

| quantity | a | b | delta |
|---|---|---|---|
| CPU total % (median) | 186.0 | 181.0 | -5.0 |
| GPU % median | 0.0 | 0.0 | 0.0 |
| tj C median | 47.6 | 47.7 | 0.1 |
| RAM MB max | 7169 | 7327 | 158 |
| CPU % bash | 0.0 | 1.0 | 1.0 |
| CPU % lidar_front_wall_node | 24.9 | 28.4 | 3.5 |

