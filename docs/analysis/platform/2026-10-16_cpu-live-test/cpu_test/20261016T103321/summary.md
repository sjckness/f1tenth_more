# cpu_test summary

```
argv: tools/cpu_test.sh 
date: 2026-10-16T10:33:21+02:00
host: Linux jetson-orin 5.15.185-tegra #2 SMP PREEMPT Fri May 29 12:19:10 CEST 2026 aarch64 aarch64 aarch64 GNU/Linux
git:  e2518ad 21 modified file(s)
dry_run: 1  no_motion: 1  duration: 20
bag: /home/fabiocar/raw_data/corridor10m_20261015T171753
```

## Phase b — stationary, logger ON (recording)

CPU source: `/proc via cpu_test_probe.py (pidstat not installed)`

### Board (tegrastats, 1 s)

| samples | CPU total % (median) | GPU % med | GPU % max | tj C med | tj C max | RAM MB max | CPU mW | SYS mW | throttle lines |
|---|---|---|---|---|---|---|---|---|---|
| 40 | 198.5 | 0.0 | 0.0 | 47.4 | 48.0 | 7295 | 1400.0 | 4760.0 | 0 |

Per core, median %: `0:7.5, 1:5.0, 2:9.5, 3:7.0, 4:4.0, 5:28.5, 6:4.5, 7:5.0, 8:6.0, 9:10.5, 10:3.0, 11:6.5`

### CPU per node, top 10 (sum over its threads)

| node | CPU % med | CPU % p95 | threads | affinity | read KB/s | write KB/s |
|---|---|---|---|---|---|---|
| cpu_test_probe.py | 4.8 | 18.2 | 2 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| lidar_front_wall_node | 2.0 | 47.5 | 12 | 5 | 0.0 | 0.0 |
| bag | 1.0 | 2.9 | 4 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| mission_logger_node | 1.0 | 1.8 | 2 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| depth_throttle_5hz | 1.0 | 1.7 | 2 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |
| launch | 1.0 | 1.0 | 1 | 0,1,2,3,4,5,6,7,8,9,10,11 | 0.0 | 0.0 |

CPU seen per core (sum of the thread samples that last ran there): `0:15.5, 1:4.8, 2:55.0, 3:2.9, 4:5.8, 5:103.9, 6:1.0, 7:3.8, 8:9.7, 9:27.9, 10:1.9, 11:78.5`

### Rates, measured outside any bag

| topic | messages | Hz |
|---|---|---|
| /scan | 56 | 35.73 |
| /odometry/filtered | 63 | 31.59 |
| /zed2/zed_node/depth/camera_info | 0 | -- |
| /perception/stereo_columns | 1 | -- |
| /perception/scene_labels | 5 | 3.33 |
| /perception/walls | 4 | 2.91 |
| /mpc/diagnostics | 0 | -- |
| /diagnostics | 0 | -- |
| /perception/virtual_wall/debug | 76 | 48.48 |
| /perception/scene_labels/stats | 1 | -- |
| /mpc/status | 0 | -- |

Absent from the graph: `/mpc/diagnostics`

