# f1tenth_bringup

Owns both top-level entry points (single-process `stack_bringup.launch.py`
and per-component-restartable `component_supervisor_node`), the Foxglove
bridge, the boot-time steering-sweep self-check, and most shared hardware
config (`vesc.yaml`, `ekf.yaml`, `mux.yaml`, `sensors.yaml`, `components.yaml`).

## Nodes

| Node | Role |
|---|---|
| `component_supervisor_node` | Spawns each named "component" (`hardware`, `localization`, `perception`, `control`, `navigation`, `behavior`, `diagnostics`, `intelligence`, `dev_tools`, `startup_sequence`, plus the on-demand-only `calibrate_hardware`) as an independent `ros2 launch` subprocess in its own process group. Exposes `/restart_component` (`f1tenth_messages/RestartComponent`), `~/control_component` (`f1tenth_messages/ComponentControl` — `SHUTDOWN`/`START`/`RESTART`), and `~/run_calibration` (`std_srvs/Trigger` — a named, fire-and-forget wrapper: stops `hardware` first (shares the VESC serial port with `calibrate_hardware`, not opened exclusively by the vendored driver, so running both at once risks real UART contention, not just a clean failure), then starts `calibrate_hardware`; does not wait for calibration to finish or auto-restart `hardware` afterward — watch `/calibration/in_progress`, see `f1tenth_diagnostics`, then `START`/`RESTART` `hardware` yourself). A watchdog timer (`watchdog_period_sec`) auto-respawns any component that exited on its own (not a `SHUTDOWN`-marked one), with a sliding-window crash-loop budget (`max_auto_restarts` per `restart_budget_window_sec`) before giving up and leaving it down until a manual `START`/`RESTART` — one real launch file within a multi-launch component (`lidar.launch.py`, gated on `use_lidar`) is filtered out of the registry entirely when its own flag is off instead, since a launch file that legitimately exits clean (code 0) immediately would otherwise burn its restart budget in seconds and get stuck; see the node's own module docstring for the exact mechanism and why the fix isn't a blanket "ignore exit 0". |
| `stack_startup_sequence` | Boot-time steer-sweep self-check (right → left → center), publishing onto the mux's high-priority `joystick` lane (`/teleop`, priority 100) so it visibly preempts whatever the drive-command source is doing. |

## Which components auto-start, and how they're modified

`component_supervisor_node` decides this itself, once at startup, reading
the same 4 stack-wide branching values `stack_bringup.launch.py` also uses,
plus its own `enable_intelligence` declared parameter for `intelligence`
specifically (a real, CLI-overridable arg, calibration-style — not one of
the 4 stack-wide values; see that node's own module docstring):

| Component | Auto-starts? | Modified by |
|---|---|---|
| `behavior` | only if `use_behavior_tree` | — |
| `intelligence` | only if `enable_intelligence` | — |
| `control` | always | Just `ackermann_mux.launch.py` — `safety_stop_controller` (which used to be conditionally appended here) was retired. |
| `navigation` | always | `navigation.launch.py` itself branches Nav2 vs. `mpc_corr` on `enable_nav2` — the supervisor doesn't gate this component. |
| `dev_tools` | always | `foxglove_bridge.launch.py`'s own `enable_foxglove` param decides whether the `Node` inside it actually launches — the supervisor doesn't gate this either. |
| `hardware`, `localization`, `perception`, `diagnostics`, `startup_sequence` | always | `hardware` runs `vesc.launch.py` with `calibration:='true'` (default, as of the automatic-startup-calibration pass) `release_downstream:='false'` — the latter is load-bearing: without it, `vesc.launch.py`'s own calibration:=true path would, once calibration finished, also nest-include `ekf.launch.py`/`navigation.launch.py` itself, duplicating what this supervisor already separately auto-starts as the independent `localization`/`navigation` components. |
| `calibrate_hardware` | **never** — only via an explicit `RestartComponent`/`ComponentControl` call, or the friendlier `~/run_calibration` (see above) | Runs `vesc.launch.py` with `calibration:=true release_downstream:=false`. Does **not** auto-sequence a full recalibration workflow — after it finishes, `localization`/`navigation` must be restarted separately by the caller to pick up the fresh values (`RestartComponent`'s request is just a bare `component_name`, no "and then also restart these" field); `~/run_calibration` doesn't change this, it only automates the `hardware`-stop step beforehand. |

## Launch files

| File | Purpose |
|---|---|
| `stack_bringup.launch.py` | The original, single-process entry point — thin orchestrator, every node lives in its owning package's own launch file, this one just `include()`s them all in order (hardware → localization/TF → perception → command/control → autonomy → diagnostics/intelligence → dev tools → startup self-check). Its own `enable_llm`-gated inclusion of `llm.launch.py` was removed outright — `llama-server` is only reachable through `supervisor_bringup.launch.py`'s `intelligence` component now. |
| `supervisor_bringup.launch.py` | Starts `component_supervisor_node` alone — the parallel, per-component-restartable path. Does not replicate `stack_bringup.launch.py`'s grouping logic; that stays untouched as a working single-process fallback. |
| `foxglove_bridge.launch.py` | `foxglove_bridge` (port 8765, gated by `enable_foxglove`) plus two `topic_tools throttle` copies (`/camera/image_raw`/`/camera/image_annotated` → `*_viz` at `foxglove_image_throttle_hz`, default 5Hz) — the real 30Hz topics feeding the perception pipeline are untouched, only the visualization copies are throttled. Added after finding `foxglove_bridge` 3.3.0 has no built-in per-topic/per-client rate control at all. Uses `arguments=[...]` (positional argv), not `parameters=[{...}]`, for `throttle` — that node's CLI is `messages|bytes, in_topic, rate, [out_topic]`, not ROS params, despite param-like strings appearing in its binary. |
| `startup_sequence.launch.py` | Wires up `stack_startup_sequence`. |

## Config

| File | Holds |
|---|---|
| `config/components.yaml` | Structural registry for `component_supervisor_node` — WHAT each component runs (one or more `ros2 launch <package> <file> [args]` entries). WHICH components auto-start is decided in code, not here (see table above). |
| `config/mux.yaml` | `ackermann_mux` priority lanes — see `f1tenth_control`'s doc. |
| `config/vesc.yaml` | VESC hardware calibration/config — see `f1tenth_hardware`'s doc. |
| `config/ekf.yaml` | `robot_localization` EKF tuning — see `f1tenth_localization`'s doc. |
| `config/sensors.yaml` | Hokuyo `urg_node` params (IP `192.168.0.10:10940`, frame `laser`, full-circle `angle_min/max = ±3.14`). |
| `config/joy_teleop.yaml` | Joystick button/axis mapping — see `f1tenth_control`'s doc. |
| `config/f1tenth_online_async.yaml` | SLAM Toolbox (async mapping) config — reused verbatim by `f1tenth_sim/launch/sim_bringup.launch.py`, not by anything in this package's own launch files. Lives here rather than in `f1tenth_sim` so the real-hardware and sim paths share one config file instead of two copies. |

## Consumed `stack_params.yaml` keys

`vesc_config` (path resolution default package), `mux_config`, `sensors_config`,
`joy_config`, `ekf_config`, `components_config`, `restart_timeout_sec`,
`log_dir`, `watchdog_period_sec`, `max_auto_restarts`,
`restart_budget_window_sec`, `enable_foxglove`, `foxglove_image_throttle_hz`
— see each key's own `# Consumed by:` comment in `stack_params.yaml`.

## Known limitations

- The two bringup paths (`stack_bringup.launch.py` and
  `component_supervisor_node`) are independently maintained — a change to
  one's component list/branching logic doesn't automatically propagate to
  the other; both were confirmed in sync as of this writing (4 stack-wide
  branching args, `safety_stop_controller` retirement reflected in both),
  but that's a manual-consistency invariant, not an enforced one. (The two
  paths diverge on `intelligence`/`llama-server` specifically now: only
  `supervisor_bringup.launch.py` can start it, gated on `enable_intelligence`
  — `stack_bringup.launch.py`'s own `enable_llm`-gated path was removed
  outright rather than kept in sync, see the `stack_bringup.launch.py` row
  above.)
