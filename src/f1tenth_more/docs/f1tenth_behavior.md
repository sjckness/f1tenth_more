# f1tenth_behavior

> **Mission subtree config format (JSON schema, `stop_condition`/`on_object`
> tables, worked examples) lives in [`src/f1tenth_behavior/README.md`](../../f1tenth_behavior/README.md),
> not duplicated here** — it's a deep, actively-maintained reference doc tied
> tightly to `mission/mission_config.py`, the single source of truth it
> documents. This page covers the package as a whole (all nodes/behaviours,
> topics, launch files, params); go there for "how do I write a mission
> JSON."

py_trees-based behavior tree supervisor: emergency-stop, obstacle-corridor
stop, a scripted mission subtree, and Nav2/direct-MPC goal navigation,
priority-ordered in one root `Selector`. Also owns the Nav2-readiness launch
sequencing and the Twist→Ackermann bridge Nav2's controller needs.

## Priority structure

Root `Selector` (first child that succeeds wins, re-ticked every cycle at
`bt_loop_duration_ms`, default 100ms):

1. **`emergency`**: `Selector[IsBatteryLow, IsEmergencyStopTriggered, IsProximityTooClose, IsSystemOverheated?]` → `Stop`. `IsSystemOverheated` only exists in the tree at all when `enable_sys_obs` is true (constructed conditionally, not just gated inside `update()` — so "sys_obs off" and "no data yet" stay distinguishable). Everything else here is unconditional — battery/emergency-stop/raw-proximity safety is never toggle-gated.
2. **`handle_obstacle`**: `IsObstacleDetected` → `Stop`. **Gated on `enable_camera_obstacle_stop`, which defaults to `false`** as of the 2026-09-01 obstacle-avoidance pass — the lane is structurally absent (not ticked-and-failing) when disabled, so a BT snapshot distinguishes "disabled" from "not tripping". Camera-seen obstacles instead reach the controller as **soft avoidance**, via `/camera/detections_3d` → `obstacle_projector_node` → `/perception/obstacles_2d` → `MPC_corr`'s `w_obs` cost term — a path unaffected by this flag. Set the flag true to restore stop-on-sight. (`safety_stop_controller`, a second, since-retired mechanism, used to also exist — see "Known limitations".)
3. **`mission`**: `MissionActive` → `Selector[mission_object_response, mission_progress]` — see the in-package README for the full schema.
4. **`navigation`**: `enable_nav2=true` → `HasGoalPose` → `NavigateThroughPosesClient`. `enable_nav2=false` → `HasMpcGoal` (mpc_corr drives itself out-of-band once `/mpc/goal_distance` arrives).

Both `mission` and `navigation` ultimately drive `mpc_corr` through
`/mpc/goal_distance`/`/mpc/goal_pose`; `mission` sits *above* `navigation` so
an active mission always wins outright and `HasMpcGoal` (a permanent latch)
never double-drives on the same tick.

## Behaviours (`f1tenth_behavior/behaviours/`)

| Behaviour | Subscribes | Role |
|---|---|---|
| `IsBatteryLow` | `/diagnostics/battery_status` (`f1tenth_messages/BatteryStatus`) | Reads `diagnostics_server_node`'s own ok/has_data verdict — doesn't duplicate the voltage threshold logic. `FAILURE` (not trip) until first message, so a cold-start race can't misfire. |
| `IsEmergencyStopTriggered` | `/mission/status` (transient-local) | Reads `MissionLoader`'s latched `emergency_stop_active` flag. |
| `IsProximityTooClose` | `/scan` | Raw-sensor last resort, YOLO-independent. Front cone + sides/rear, two angular windows over the same LaserScan (front used to read `/perception/front_distance`/ZED depth, replaced by a LiDAR front-cone check — see its own docstring for the rationale/trade-off). Thresholds are now direct params (`proximity_front_threshold_m`/`proximity_side_threshold_m`, both 0.15 m) rather than derived from `car_radius` + margins — tightened to a last-resort contact range so MPC avoidance owns the band above it. Gated on `enable_lidar_safety_stop` (default **true**): this is the floor that catches avoidance being wrong, and the sides/rear have no other coverage at all. |
| `IsSystemOverheated` | `/diagnostics/system_status` | CPU/GPU **temperature** (`sys_obs_max_temp_c`) trips on a single sample, undebounced. CPU/GPU **utilization** (`sys_obs_max_load_percent`) is a separate trigger, disabled by default (`enable_sys_obs_load_trip`) and debounced by `sys_obs_load_trip_consecutive_samples` when enabled — it caused 3 of 9 stop episodes on 2026-09-01, all false positives on bursty YOLO inference load while temps sat 45 °C below their limit. |
| `IsObstacleDetected` | `/camera/detections_3d` | Forward corridor box check. Corridor half-width/height derived from `car_radius + obstacle_safety_margin_m`. |
| `Stop` | — (publisher only) | Zero-speed `AckermannDriveStamped` onto the mux's `safety_stop` lane every tick; one instance per lane, stamping **distinct `frame_id`s** (`base_link/emergency` vs `base_link/obstacle`) so a bag can attribute a stop to its lane — the mux republishes the winning message verbatim, and both used to stamp a bare `base_link`. |
| `HasGoalPose` | `/goal_pose` | Nav2-mode goal input (RViz "2D Goal Pose"); writes a one-element `poses` list onto the blackboard. |
| `NavigateThroughPosesClient` | — (action client) | Custom action-client behaviour (not `py_trees_ros`'s stock one — see its own docstring for why); only sends a new goal when the poses array actually changed. |
| `HasMpcGoal` | `/mpc/goal_distance` | Direct-MPC-mode presence check, permanent latch once any goal has arrived. |
| `MissionActive`, `PublishMoveGoal`, `CheckStopCondition`, `AdvanceMove`, `ObjectSeen`, `HandleObjectAction` | various (`/mission/status`, `/mpc/goal_reached`, `get_odom_topic()`, `detected_classes`) | Mission subtree — see the in-package README. |

## Nodes

| Node | Role |
|---|---|
| `behavior_executor_node` | Owns the tree: `create_root()` builds it, `main()` bootstrap-reads params needed before `tree.node` exists (`bt_setup_timeout_sec`, `sys_obs_max_temp_c`/`_max_load_percent`, `car_radius`/`obstacle_safety_margin_m`/`proximity_front_extra_margin_m`), then `tree.tick_tock()`s forever. Also publishes a live colored dot-graph of tree status as PNG on `/bt/tree_visualization` (gated by `enable_bt_visualization`) and a throttled ascii-tree snapshot log. |
| `twist_to_ackermann_node` | Subscribes `cmd_vel_nav` (`geometry_msgs/Twist`, from `nav2_regulated_pure_pursuit_controller` — Nav2 has no Ackermann-native controller plugin in this ROS distro), publishes `drive` (`AckermannDriveStamped`, the mux's `navigation` lane) — an alternative source for that lane, mutually exclusive with `mpc_corr` via `use_behavior_tree`. |
| `wait_for_trigger_service_node` | Generic: polls a named `std_srvs/Trigger` service until success, exits 0/1. Used by `behavior_bringup.launch.py` to wait on `lifecycle_manager_navigation`'s `is_active` before starting `behavior_executor_node`/`twist_to_ackermann_node`, only when `enable_nav2=true`. |

## Launch files

**`behavior_bringup.launch.py`** — the only one. `enable_nav2=true`: gates
startup behind `wait_for_trigger_service_node` polling Nav2 readiness
(`nav2_readiness_timeout_sec`), via a `RegisterEventHandler(OnProcessExit)`
event chain, not a fixed-duration timer. `enable_nav2=false`: launches
`behavior_executor_node`/`twist_to_ackermann_node` immediately — no
`lifecycle_manager_navigation` to ever wait for.

## Config

`f1tenth_behavior/config/twist_to_ackermann.yaml` — wheelbase (0.305) and
min/max steering angle for the Twist→Ackermann conversion.
`f1tenth_behavior/missions/*.json` — worked mission examples (see the
in-package README).

## Consumed `stack_params.yaml` keys

`nav2_readiness_service`, `nav2_readiness_timeout_sec`,
`bt_setup_timeout_sec`, `sys_obs_max_temp_c`, `sys_obs_max_load_percent`,
`car_radius`, `obstacle_safety_margin_m`, `proximity_front_extra_margin_m`,
`mission_file_name`, `enable_bt_visualization`, `enable_sys_obs`,
`enable_nav2`, `localization_source` (via `get_odom_topic()`) — see each
key's own `# Consumed by:` comment in `stack_params.yaml`.

## Known limitations

- `safety_stop_controller` — a second, independent obstacle-stop mechanism
  that used to live in `f1tenth_control` — was retired during the
  code-analysis/fixes pass. `handle_obstacle` (`IsObstacleDetected → Stop`)
  is confirmed the sole obstacle-stop mechanism now, unconditional, no
  config change needed to keep it that way.
- The mission subtree has several deliberate stub fields (`vdes` override,
  `goal_reached`'s `tolerance`, `reduce_speed`/`reduce_speed_for`, `manual`
  stop-condition type) that log once and otherwise no-op — see the
  in-package README's own tables for exactly which.
- `MissionLoader`'s `threading.Lock` documents an invariant (single-threaded
  executor, so its own three entry points can't actually race) rather than
  fixing an active bug — see that README's last "Assumptions" bullet.
