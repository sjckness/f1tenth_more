# cpu_test summary

```
argv: tools/cpu_test.sh 
date: 2026-10-16T10:34:55+02:00
host: Linux jetson-orin 5.15.185-tegra #2 SMP PREEMPT Fri May 29 12:19:10 CEST 2026 aarch64 aarch64 aarch64 GNU/Linux
git:  1d0334e 21 modified file(s)
dry_run: 1  no_motion: 1  duration: 15
bag: /home/fabiocar/raw_data/corridor10m_20261015T171753
```

## Phase a — stationary, logger OFF

CPU source: `/proc via cpu_test_probe.py (pidstat not installed)`

### Board (tegrastats, 1 s)

| samples | CPU total % (median) | GPU % med | GPU % max | tj C med | tj C max | RAM MB max | CPU mW | SYS mW | throttle lines |
|---|---|---|---|---|---|---|---|---|---|
| 20 | 205.5 | 0.0 | 0.0 | 47.4 | 47.9 | 7360 | 1200.0 | 4760.0 | 0 |

Per core, median %: `0:13.0, 1:4.0, 2:12.0, 3:4.0, 4:8.0, 5:5.0, 6:5.0, 7:7.5, 8:21.5, 9:7.5, 10:18.0, 11:5.5`

### CPU per node, top 10 (sum over its threads)

| node | CPU % med | CPU % p95 | threads | affinity | read KB/s | write KB/s |
|---|---|---|---|---|---|---|
| cpu_test_probe.py | 4.8 | 25.6 | 3 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| jtop | 1.0 | 1.0 | 1 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| lidar_front_wall_node | 1.0 | 3.2 | 7 | 5 | 0.0 | 0.0 |
| depth_throttle_5hz | 1.0 | 2.7 | 4 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| bag | 1.0 | 2.9 | 4 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| mission_logger_node | 1.0 | 1.0 | 3 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| bash | 1.0 | 1.0 | 1 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |

CPU seen per core (sum of the thread samples that last ran there): `0:1.0, 1:3.8, 2:72.8, 3:12.6, 4:54.1, 5:10.8, 6:2.9, 7:4.9, 8:1.0, 9:5.8, 10:2.9, 11:4.0`

### Rates, measured outside any bag

| topic | messages | Hz |
|---|---|---|
| /scan | 0 | -- |
| /odometry/filtered | 0 | -- |
| /zed2/zed_node/depth/camera_info | 0 | -- |
| /perception/stereo_columns | 0 | -- |
| /perception/scene_labels | 0 | -- |
| /perception/walls | 0 | -- |
| /mpc/diagnostics | 0 | -- |
| /diagnostics | 0 | -- |
| /perception/virtual_wall/debug | 0 | -- |
| /perception/scene_labels/stats | 0 | -- |
| /mpc/status | 0 | -- |

Absent from the graph: `/mpc/diagnostics`

## Phase b — stationary, logger ON (recording)

CPU source: `/proc via cpu_test_probe.py (pidstat not installed)`

### Board (tegrastats, 1 s)

| samples | CPU total % (median) | GPU % med | GPU % max | tj C med | tj C max | RAM MB max | CPU mW | SYS mW | throttle lines |
|---|---|---|---|---|---|---|---|---|---|
| 35 | 194.0 | 0.0 | 0.0 | 47.5 | 48.0 | 7411 | 1200.0 | 4760.0 | 0 |

Per core, median %: `0:9.0, 1:8.0, 2:5.0, 3:7.0, 4:4.0, 5:12.0, 6:5.0, 7:6.0, 8:6.0, 9:8.0, 10:6.0, 11:6.0`

### CPU per node, top 10 (sum over its threads)

| node | CPU % med | CPU % p95 | threads | affinity | read KB/s | write KB/s |
|---|---|---|---|---|---|---|
| cpu_test_probe.py | 4.8 | 25.7 | 4 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| bag | 1.4 | 2.4 | 2 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| mission_logger_node | 1.0 | 1.0 | 3 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| lidar_front_wall_node | 1.0 | 3.5 | 10 | 5 | 0.0 | 0.0 |
| depth_throttle_5hz | 1.0 | 2.1 | 7 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| bash | 1.0 | 1.0 | 1 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |

CPU seen per core (sum of the thread samples that last ran there): `0:4.8, 1:6.8, 2:6.8, 3:5.8, 4:1.9, 5:14.6, 7:1.0, 8:76.6, 9:3.8, 10:26.9, 11:32.8`

### Rates, measured outside any bag

| topic | messages | Hz |
|---|---|---|
| /scan | 0 | -- |
| /odometry/filtered | 0 | -- |
| /zed2/zed_node/depth/camera_info | 0 | -- |
| /perception/stereo_columns | 0 | -- |
| /perception/scene_labels | 0 | -- |
| /perception/walls | 0 | -- |
| /mpc/diagnostics | 0 | -- |
| /diagnostics | 0 | -- |
| /perception/virtual_wall/debug | 0 | -- |
| /perception/scene_labels/stats | 0 | -- |
| /mpc/status | 0 | -- |

Absent from the graph: `/mpc/diagnostics`

## Diff a -> b (what recording costs at rest)

| quantity | a | b | delta |
|---|---|---|---|
| CPU total % (median) | 205.5 | 194.0 | -11.5 |
| GPU % median | 0.0 | 0.0 | 0.0 |
| tj C median | 47.4 | 47.5 | 0.1 |
| RAM MB max | 7360 | 7411 | 51 |
| CPU % jtop | 1.0 | 0.0 | -1.0 |

