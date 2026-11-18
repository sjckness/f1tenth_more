# cpu_test summary

```
argv: tools/cpu_test.sh 
date: 2026-10-16T10:25:43+02:00
host: Linux jetson-orin 5.15.185-tegra #2 SMP PREEMPT Fri May 29 12:19:10 CEST 2026 aarch64 aarch64 aarch64 GNU/Linux
git:  6cd1609 25 modified file(s)
dry_run: 1  no_motion: 1  duration: 25
bag: /home/fabiocar/raw_data/corridor10m_20261015T171753
```

## Phase a — stationary, logger OFF

CPU source: `/proc via cpu_test_probe.py (pidstat not installed)`

### Board (tegrastats, 1 s)

| samples | CPU total % (median) | GPU % med | GPU % max | tj C med | tj C max | RAM MB max | CPU mW | SYS mW | throttle lines |
|---|---|---|---|---|---|---|---|---|---|
| 30 | 191.0 | 0.0 | 0.0 | 47.4 | 47.9 | 7010 | 1400.0 | 4760.0 | 0 |

Per core, median %: `0:9.5, 1:5.5, 2:5.0, 3:6.0, 4:4.5, 5:5.5, 6:3.5, 7:4.0, 8:6.5, 9:7.0, 10:9.0, 11:3.5`

### CPU per node, top 10 (sum over its threads)

| node | CPU % med | CPU % p95 | threads | affinity | read KB/s | write KB/s |
|---|---|---|---|---|---|---|
| cpu_test_probe.py | 3.9 | 11.1 | 3 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| bag | 1.9 | 2.9 | 4 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| lidar_front_wall_node | 1.0 | 1.0 | 1 | 5 | 0.0 | 0.0 |
| bash | 1.0 | 1.0 | 2 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |

CPU seen per core (sum of the thread samples that last ran there): `0:3.8, 1:1.0, 2:1.0, 3:1.0, 4:4.8, 5:6.8, 6:8.7, 7:7.8, 8:74.7, 9:41.7, 10:50.2, 11:12.5`

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
| 44 | 192.0 | 0.0 | 0.0 | 47.6 | 48.1 | 7102 | 1200.0 | 4709.0 | 0 |

Per core, median %: `0:6.0, 1:8.5, 2:5.0, 3:4.5, 4:4.0, 5:4.0, 6:10.5, 7:3.0, 8:5.5, 9:6.0, 10:3.5, 11:5.5`

### CPU per node, top 10 (sum over its threads)

| node | CPU % med | CPU % p95 | threads | affinity | read KB/s | write KB/s |
|---|---|---|---|---|---|---|
| cpu_test_probe.py | 4.8 | 5.8 | 3 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| bag | 1.9 | 2.9 | 4 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| depth_throttle_5hz | 1.0 | 1.0 | 2 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| lidar_front_wall_node | 1.0 | 1.0 | 2 | 5 | 0.0 | 0.0 |
| mission_logger_node | 1.0 | 1.0 | 2 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| bash | 1.0 | 1.0 | 1 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |

CPU seen per core (sum of the thread samples that last ran there): `0:86.3, 1:4.8, 2:12.6, 3:3.9, 4:15.4, 5:5.8, 6:1.9, 7:4.8, 8:56.6, 9:1.9, 10:2.9, 11:31.1`

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
| CPU total % (median) | 191.0 | 192.0 | 1.0 |
| GPU % median | 0.0 | 0.0 | 0.0 |
| tj C median | 47.4 | 47.6 | 0.2 |
| RAM MB max | 7010 | 7102 | 92 |
| CPU % depth_throttle_5hz | 0.0 | 1.0 | 1.0 |
| CPU % mission_logger_node | 0.0 | 1.0 | 1.0 |

