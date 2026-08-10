# f1tenth_control

`f1tenth_control` (a thin launch-file metapackage) + `mpc_controller` (the
actual MPC nodes). Owns the drive-command arbitration layer
(`ackermann_mux`), the MPC controller, and manual joystick control.
`safety_stop_controller` — a third package that used to live under this
directory — was **retired** during the code-analysis-and-fixes pass (see
"Known limitations" below).

## Nodes

### `ackermann_mux` (vendored, `f1tenth_external`'s doc)
Priority-based arbitration between drive-command sources, publishing the one
`/ackermann_drive` topic `vesc_ackermann`'s `ackermann_to_vesc_node`
consumes. Lanes (`f1tenth_bringup/config/mux.yaml`):

| Lane | Topic | Priority | Publisher |
|---|---|---|---|
| `safety_stop` | `safety_stop` | 200 (highest) | `f1tenth_behavior`'s `Stop` BT action — emergency lane or `handle_obstacle` lane |
| `navigation` | `drive` | 10 | `mpc_corr`/`andre_mpc_node` (direct-MPC path) or `twist_to_ackermann_node` (BT/Nav2 path) |
| `joystick` | `teleop` | 100 | `joy_teleop` (manual control) |

### `mpc_corr` (`mpc_controller/MPC_corr.py`) — **the currently-deployed MPC node**
Launched by `f1tenth_navigation/navigation.launch.py` when `enable_nav2:=false`.

**Subscribes:**

| Topic | Type | Purpose |
|---|---|---|
| `get_odom_topic()` result (`/odometry/filtered` or `/odom`, follows `localization_source`) | `nav_msgs/Odometry` | Hardware odometry, `BEST_EFFORT` QoS |
| `/model/virtual_robot/odometry` | `nav_msgs/Odometry` | Sim odometry fallback |
| `/mpc/goal_distance` | `std_msgs/Float32` | Commands a straight-line drive: captures current `(x, y)` on receipt, stops once that Euclidean distance is traveled. A new message overrides any goal in progress. |
| `/mpc/goal_pose` | `geometry_msgs/PoseStamped` | Position-only goal-pose driving (used by `f1tenth_behavior`'s mission subtree's `goal_pose` moves) |
| `/mpc/hold` | `std_msgs/Bool` | Short-circuits `control_loop` to publish zero speed/steering without touching goal-progress state — used by the mission subtree's `stop_and_hold`/`abort_mission` actions |
| `/perception/obstacles_2d` | `f1tenth_messages/Obstacle2DArray` | Live obstacle list for soft-avoidance deflection |
| `/perception/front_distance` | `std_msgs/Float32` | Raw ZED depth front-distance reading |
| `/imu` | `sensor_msgs/Imu` | — |

**Publishes:** `/drive` (`AckermannDriveStamped`, the mux's `navigation` lane),
`/mpc/min_obstacle_distance`, `/mpc/predicted_min_clearance`,
`/mpc/goal_reached` (`Bool`).

**ROS params** (declared in-code, wired via `mpc_corr.launch.py`):
`odom_stale_timeout_sec` (0.5s — max age before falling back to sim odom, then
to "no odom"), `use_rti_solver` (true — OSQP real-time-iteration solve vs.
the original from-scratch SLSQP path, kept as an explicit rollback opt-out),
`car_radius` (0.20m), `avoidance_margin` (0.12m — together the soft-avoidance
deflection trigger radius `R_safe = car_radius + avoidance_margin`, and the
basis for `dmin`, a disabled/warning-only clearance-log threshold),
`pose_goal_tolerance` (0.15m), `target_smoothing_alpha`,
`deflection_decay_ticks`, `cpu_affinity`/`nice` (machine-specific process
pinning, not sourced from `stack_params.yaml` — see that launch file's own
docstring).

### `andre_mpc_node` (`mpc_controller/andre_mpc_node.py`) — **not in either automatic bring-up path**
An earlier, simpler circle-tracking MPC (open-loop sine speed profile around
a reference circle). Its own launch file, `mpc.launch.py`, wires up all 12 of
its cost/limit gains (`qn`, `qv`, `qalpha`, `qddelta`, `alat_max`, `a_min`,
`a_max`, `v_min`, `v_max`, `v_ref`, `sine_amp`, `sine_period`) from
`stack_params.yaml`, but **`mpc.launch.py` itself is not included by
`stack_bringup.launch.py`, `components.yaml`, or anywhere else** — confirmed
by grep, no reference outside its own file and `stack_params.yaml`'s
comments. Subscribes `get_odom_topic()`, publishes `/drive` — same wiring
shape as `mpc_corr`, so it could stand in for it if ever re-enabled, but
today it's dead weight kept around, not a live alternative.

## Launch files

| File | Purpose |
|---|---|
| `ackermann_mux.launch.py` | Starts `ackermann_mux` with this stack's own combined `mux.yaml` — **not** the vendored package's own launch file (which loads 3 separate locks/topics/joystick configs with a different remap target; deliberate non-reuse). |
| `joy.launch.py` | `joy_node` + `joy_teleop`, config from `f1tenth_bringup/config/joy_teleop.yaml`. Not included by `stack_bringup.launch.py` — run standalone when manual control is needed. |
| `mpc.launch.py` | Wires up `andre_mpc_node` (see above). Shuts down the whole launch tree if the node dies (`OnProcessExit` → `Shutdown()`). **Orphaned** — not included anywhere. |
| `mpc_corr.launch.py` | Wires up `mpc_corr` (see above). Included by `f1tenth_navigation/navigation.launch.py` when `enable_nav2:=false`. |

## Config

`f1tenth_bringup/config/mux.yaml` (mux priority lanes, owned by
`f1tenth_bringup`, not this package) and `f1tenth_bringup/config/joy_teleop.yaml`
(joystick button/axis mapping).

## Consumed `stack_params.yaml` keys

`mux_config`, `joy_config`, `odom_stale_timeout_sec`, `use_rti_solver`,
`car_radius`, `obstacle_safety_margin_m` (→ `mpc_corr`'s `avoidance_margin`),
`mpc_node_name` (consumed by `llm`, not this package, but describing this
package's deployed node), `qn`/`qv`/`qalpha`/`qddelta`/`alat_max`/`a_min`/
`a_max`/`v_min`/`v_max`/`v_ref`/`sine_amp`/`sine_period` (all
`andre_mpc_node`/`mpc.launch.py` only) — see each key's own `# Consumed by:`
comment in `stack_params.yaml`.

## Known limitations

- **`andre_mpc_node`/`mpc.launch.py` is fully wired but unreachable** — no
  automatic bring-up path includes it. Anyone wanting it running needs
  `ros2 launch f1tenth_control mpc.launch.py` by hand.
- **RTI solve time is self-monitored, not hard-bounded.** `MPC_corr.py` logs
  a throttled warning (`solve_dt > self.ts`, i.e. > the 100ms/10Hz tick
  budget) whenever an individual OSQP solve overruns — this is a live,
  active check, not a resolved/removed one, meaning occasional worst-case
  ticks are still possible in principle (warm-started QP solves aren't
  constant-time) and the only mitigation today is visibility via that log
  line, not a fallback path.
- **`mpc_controller`'s own `package.xml`/`setup.py` still carry placeholder
  metadata** (`TODO: Package description`, `TODO: License declaration`,
  maintainer `fabiocar@todo.todo`) — cosmetic, but genuinely still present.
- **`corridors_jsons/`** (`src/f1tenth_control/corridors_jsons/`) holds
  `MPC_corr.py`'s per-run debug snapshot output (`self.save_corridor_debug`,
  a hardcoded `True` in-code flag, not a launch-configurable param) —
  gitignored, not source.
