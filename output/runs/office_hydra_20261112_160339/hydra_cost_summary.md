# hydra_cost summary (2026-11-12T16:03:41)

166 samples over 166 s on DESKTOP-Q3F81S6; GPU source nvidia-smi; CSV `samples.csv`.

System: CPU mean 10 % / max 12 % of all cores; RAM max 12.4 GB.

| process | CPU mean % | CPU max % | RSS max MB |
|---|---|---|---|
| hydra_ros_node[273845] | 633 | 867 | 8473 |
| rviz2[273847] | 158 | 468 | 370 |
| hydra_cost.py[273848] | 11 | 119 | 107 |
| odom_to_tf[273844] | 6 | 7 | 73 |
| hydra_visualizer_node[273846] | 2 | 5 | 202 |

Top threads (mean CPU %): hydra_ros_node[273845]/hydra_ros_node:274098 99, rviz2[273847]/rviz2:273847 19, hydra_ros_node[273845]/hydra_ros_node:274099 13, rviz2[273847]/llvmpipe-31:273937 7, hydra_ros_node[273845]/hydra_ros_node:274100 7, hydra_cost.py[273848]/python3:274078 6, rviz2[273847]/llvmpipe-0:273906 6, rviz2[273847]/llvmpipe-1:273907 6, rviz2[273847]/llvmpipe-2:273908 6, rviz2[273847]/llvmpipe-3:273909 6

GPU gpu0: util mean 38 % / max 41 %, VRAM max 4530 MB

| topic | Hz mean | kB/s mean |
|---|---|---|
| `/camera/image_raw` | 9.9 | 8945 |
| `/hydra/backend/dsg` | 1.4 | 736 |
| `/hydra/backend/dsg_mesh` | 0.5 | 4087 |
| `/hydra_visualizer/graph` | 1.4 | 325 |
| `/sim/ground_truth/left/semantic` | 9.9 | 2237 |
| `/sim/ground_truth/odom` | 49.7 | 35 |
| `/tf` | 52.9 | 6 |
| `/zed2/zed_node/depth/depth_registered` | 9.9 | 8945 |

| layer | nodes (last) | nodes (max) | edges (last) |
|---|---|---|---|
| AGENTS | 70 | 70 | 69 |
| BUILDINGS | 1 | 1 | 0 |
| MESH_PLACES | 28 | 28 | 38 |
| OBJECTS | 0 | 0 | 0 |
| PLACES | 939 | 949 | 3187 |
| ROOMS | 5 | 7 | 3 |

Graph message: 904 kB last, 910 kB max.

Hydra status NOMINAL in 100 % of samples.

Hydra timing: no `timing_stats.csv` in `/home/andreas/f1tenth_sim_archive/office_hydra_20261112_160339/hydra` yet (Hydra writes it when it shuts down).

