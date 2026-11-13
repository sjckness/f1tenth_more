# Planner-controller comparison: "go straight 5m then enter the door on the left"

Session `door_sem_20261113_151220`, host `DESKTOP-Q3F81S6`, scene `office` layout `door_left`, branch `planner-modes-hil` @ `e326bb4`. One run per mode, the same command sent through the LLM planner each time.

The LLM produced the **same mission** for every mode (moves sha `fa9e6540f521`):

```json
[
 {
  "id": "move_0_straight",
  "drive": {
   "mode": "straight",
   "speed": 0.5
  },
  "stop_condition": {
   "type": "distance_reached",
   "distance": 5.0
  },
  "timeout_sec": 35,
  "on_timeout": "abort"
 },
 {
  "id": "move_1_enter_door",
  "enter_door": {
   "type": "enter_door",
   "door": "left",
   "xy_tol": 0.15,
   "yaw_tol_deg": 10.0,
   "acquire_timeout_sec": 5.0,
   "approach_dist_m": 0.8
  },
  "stop_condition": {
   "type": "door_entered"
  },
  "timeout_sec": 65,
  "on_timeout": "abort",
  "terminal": true
 }
]
```

## Results

| metric | unit | corridor_mpc | corridor_rrt | p2p_geometric |
|---|---|---|---|---|
| mission result |  | ABORTED | ABORTED | ABORTED |
| mission duration | s | 16.08 | 15.65 | 15.75 |
| distance at mission COMPLETE | m | 7.612 | 7.602 | 7.651 |
| distance where the car stopped | m | 7.673 | 7.663 | 7.713 |
| stopped minus commanded | m | 2.673 | 2.663 | 2.713 |
| max lateral offset | m | 0.117 | 0.261 | 0.263 |
| RMS lateral offset (mission) | m | 0.026 | 0.106 | 0.108 |
| final heading error | deg | 8.94 | 4.54 | 4.23 |
| driven length | m | 7.706 | 7.696 | 7.746 |
| mean speed (mission) | m/s | 0.473 | 0.486 | 0.486 |
| max speed | m/s | 0.496 | 0.498 | 0.498 |
| mean |steering cmd| | deg | 0.95 | 0.62 | 0.61 |
| max |steering cmd| | deg | 5.76 | 1.01 | 1.01 |
| steering total variation | deg | 39.2 | 11.2 | 11.4 |
| reference tracked |  | corridor centreline | planned path | planned path |
| references published |  | 16 | 16 | 16 |
| max lateral of the reference | m | 0.227 | 0.095 | 0.093 |
| distance to reference, mean | m | 0.091 | 0.116 | 0.117 |
| distance to reference, max | m | 0.316 | 0.348 | 0.348 |
| EKF vs ground truth at the end | m | 0.307 | 0.365 | 0.367 |
| controller CPU, mission | % core | 39.6 | 10.0 | 9.8 |
| controller CPU, peak 1 s | % core | 45.9 | 12.0 | 12.0 |
| controller CPU, idle before | % core | 26.4 | 12.1 | 12.1 |
| controller CPU per sim second | ms | 402.9 | 97.9 | 100.3 |
| controller CPU per /drive msg | ms | 40.38 | 9.68 | 9.87 |
| /drive rate | Hz | 9.9 | 10.0 | 10.0 |
| replans (new modes) |  | - | 16 | 16 |
| plan time mean (wall) | ms | - | 1.07 | 0.35 |
| plan time p95 | ms | - | 1.37 | 0.37 |
| plan time max | ms | - | 1.39 | 0.38 |
| MPC solves |  | 157 | - | - |
| MPC solve success | % | 100.0 | - | - |
| MPC solve_dt mean (ROS clock) | ms | 0.0 | - | - |
| controller RSS | MB | 129.0 | 96.7 | 96.8 |
| sim real-time factor |  | 0.982 | 1.02 | 0.978 |
| host CPU busy (all cores) | % | 3.4 | 3.2 | 3.5 |
| LLM translation time | s | 19.642 | 20.213 | 19.717 |

Frame: the car's pose when the mission went RUNNING; x forward, y left. "stopped minus commanded" is ground truth against 5 m. CPU is the controller process from /proc (utime+stime, all threads) over the mission's wall-clock window; "per sim second" removes the sim's real-time factor.

![trajectories.png](trajectories.png)

![paths.png](paths.png)

![timeseries.png](timeseries.png)

![cpu.png](cpu.png)

## Notes

- **corridor_mpc**: mpc_corr solve_dt_sec is 0 throughout (ROS clock in sim time): use the process CPU columns for its cost
- **corridor_mpc**: mission ended ABORTED
- **corridor_rrt**: mission ended ABORTED
- **p2p_geometric**: mission ended ABORTED
