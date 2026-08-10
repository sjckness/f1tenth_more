# f1tenth_messages

Custom message/service interface definitions shared across the stack. The
only `ament_cmake` package in the workspace — every consumer is pure
`ament_python`, which can't run `rosidl_generate_interfaces()` in its own
build, so the interfaces live in this separate, dedicated package instead
(merged from two earlier packages, `f1tenth_diagnostics_msgs` and
`f1tenth_bringup_msgs`).

No nodes, no launch files, no config — just `.msg`/`.srv` definitions.

## Messages

| Message | Published by | Topic | Purpose |
|---|---|---|---|
| `SystemStatus` | `f1tenth_diagnostics`' `system_observer_node` | `/diagnostics/system_status` | CPU/RAM (`psutil`) + Jetson GPU/EMC/temps (`jtop`). Only published when `enable_sys_obs` is true. |
| `BatteryStatus` | `f1tenth_diagnostics`' `diagnostics_server_node` | `/diagnostics/battery_status` | Continuous battery voltage monitoring, never gated behind `enable_sys_obs`. `has_data` guards against a cold-start misread as "critically low." |
| `Obstacle2D` | (element type only) | — | Ground-plane disk approximation (`x, y, r`) used by `MPC_corr.py`. |
| `Obstacle2DArray` | `f1tenth_perception`'s `obstacle_projector_node` | `/perception/obstacles_2d` | Per-frame obstacle list — one detections message in, one list out, no cross-frame persistence/smoothing. |
| `MissionStatus` | `f1tenth_behavior`'s `MissionLoader` (`mission/loader.py`) | `/mission/status` (transient-local/latched QoS) | `state` (`IDLE/LOADED/RUNNING/HOLDING/COMPLETE/ABORTED`), `json_path`, `emergency_stop_active`. Republished on load/start/abort/emergency_stop, not every tick. |

## Services

| Service | Served by | Purpose |
|---|---|---|
| `RestartComponent` | `f1tenth_bringup`'s `component_supervisor_node` | `component_name` → restart that component's whole launch group. |
| `ComponentControl` | `f1tenth_bringup`'s `component_supervisor_node` | `component_name` + `action` (`SHUTDOWN=0`/`START=1`/`RESTART=2`) — finer-grained than `RestartComponent`. |
| `RunDiagnostics` | `f1tenth_diagnostics`' `diagnostics_server_node` (`~/run_diagnostics`) | Empty request; returns current `BatteryStatus` + last-received `SystemStatus` + `sys_obs_enabled` (tells the caller whether `system_status` is meaningful, since nothing publishes it when system observability is disabled). |
| `LoadMission` | `f1tenth_behavior`'s `MissionLoader` (`/mission/load_mission`) | `path` → load a mission JSON synchronously; `success`/`message` report the result. |

`start_mission`/`abort_mission`/`emergency_stop` (also served by
`MissionLoader`) reuse `std_srvs/Trigger` directly — no custom `.srv` needed
for those three. An earlier `AbortMission.srv` (which added a
`mission_id` guard) was removed once that guard was dropped as
over-engineering for a system with only ever one mission slot at a time; see
`f1tenth_behavior`'s doc.

## Known limitations

- No versioning story if a message's fields ever need to change shape —
  every consumer would need updating in lockstep, same as any ROS interface
  package.
