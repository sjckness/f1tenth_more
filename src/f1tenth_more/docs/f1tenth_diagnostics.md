# f1tenth_diagnostics

> **How to run each calibration tool, expected output, and where results get
> applied lives in [`src/f1tenth_diagnostics/README.md`](../../f1tenth_diagnostics/README.md),
> not duplicated here** — it's a detailed, per-node operational guide. This
> page is the shorter what/why overview.

Calibration and diagnostic tooling: one-off measurement/validation nodes
that shouldn't live inside the production control/perception/hardware
packages. Convention for every node here: self-contained, single-purpose,
safe to run standalone with no side effects on the rest of the stack (unless
a node's whole point *is* to affect something, e.g. writing calibration
results back to `vesc.yaml`).

## Nodes

| Node | Type | Publishes/Serves | Role |
|---|---|---|---|
| `gyro_bias_calibration_node` | One-shot measurement + writer | (logs + patches `vesc.yaml`) | Static gyro DC-offset bias over a stationary sampling window, gated on a pre-sampling stationary check (raw ERPM telemetry). **Writes its result** (changed — used to be logs-only, manual-reference-only): patches `gyro_bias_z` into `vesc.yaml` via the same shared `calibration_common.write_vesc_yaml` mechanism `sensor_covariance_calibration_node` uses. Also now exits cleanly with a real exit code instead of `rclpy.spin()`-ing forever (previously required a manual Ctrl+C even after logging its result) — both changes made as part of the automatic-startup-calibration pass, specifically to make this node sequenceable the same way the covariance node already was. |
| `sensor_covariance_calibration_node` | One-shot measurement + writer | (logs + patches `vesc.yaml`) | Message-level covariance (Welford's online variance) for VESC IMU + `vesc_to_odom_node_backup`, `stationary` mode by default. **Writes its result**: backs up `vesc.yaml` (timestamped copy, pruned to the newest 5) then patches only the relevant variance keys in place via `ruamel.yaml` round-trip mode (preserves comments/formatting) — shared with `gyro_bias_calibration_node` via `calibration_common.write_vesc_yaml`. `stationary` mode gates sampling on a pre-sampling stationary check plus a wait-for-first-message gate (see `calibration_common.StationaryGate`). `light_motion` mode (real non-zero `vx_variance` via a human-confirmed drive) is `ros2 run`-only, never `ros2 launch`, and has no stationary check (it inherently drives on purpose) — its confirmation gate blocks on `input()`, which `ros2 launch` doesn't reliably forward to a subprocess's stdin. |
| `battery_voltage_check_node` | One-shot startup gate | (logs, exit code) | Samples `VescStateStamped.state.voltage_input` on `/sensors/core` for `sample_window_sec`, compares the mean against `min_battery_voltage`. Exit code (0/1) gates whether `vesc.launch.py` lets the rest of the drive stack launch at all. Not continuous — checks once at boot, never again. |
| `system_observer_node` | Continuous | `/diagnostics/system_status` (`f1tenth_messages/SystemStatus`) | CPU/RAM (`psutil`) + Jetson GPU/EMC/temps (`jtop`, graceful fallback if not connected) at `publish_rate_hz`. Gated behind `enable_sys_obs` — `system_observer.launch.py` returns an empty `LaunchDescription` when disabled (executable never starts at all, not just idles). |
| `diagnostics_server_node` | Continuous + on-demand | `/diagnostics/battery_status` (`f1tenth_messages/BatteryStatus`); `/calibration/in_progress` (`std_msgs/Bool`); `~/run_diagnostics` (`f1tenth_messages/RunDiagnostics`) | Three jobs, deliberately **not** gated behind `enable_sys_obs` (battery safety must keep working regardless): continuous battery monitoring (averages `/sensors/core` samples since the last publish tick, `has_data` guards a cold start), an on-demand snapshot service (battery + last `SystemStatus`, with `sys_obs_enabled` telling the caller whether the latter is meaningful), and `/calibration/in_progress` — `true` while either calibration node is alive in the ROS graph (plain `get_node_names()` polling, TRANSIENT_LOCAL QoS so a late subscriber gets the current value immediately), `false` otherwise. Bringup-mode-agnostic by construction: correct regardless of which of the three ways calibration got triggered (`stack_bringup.launch.py`'s `calibration:=true`, `component_supervisor_node`'s `hardware`/`calibrate_hardware`/`~/run_calibration`, or a standalone `calibration.launch.py`), since it never talks to any of those paths directly. |

## Launch files

| File | Purpose |
|---|---|
| `calibration.launch.py` | `gyro_bias_calibration_node` + `sensor_covariance_calibration_node` together, consolidated from two previously-separate files since both are stationary samplers meant to be run as one pass. |
| `diagnostics_server.launch.py` | `diagnostics_server_node` alone. |
| `system_observer.launch.py` | `system_observer_node` alone, self-gated behind `enable_sys_obs`. |

`battery_voltage_check_node` has no launch file of its own — it's only
launched inline by `f1tenth_hardware/vesc.launch.py` (needs a live
`vesc_driver_node` to sample from, so it can't stand alone the way the
others can).

## Consumed `stack_params.yaml` keys

`imu_topic`, `odom_topic`, `gyro_sample_duration_sec`, `min_samples`,
`calibration_duration_sec` (shared by both `calibration.launch.py` and
`f1tenth_hardware/vesc.launch.py`'s `calibration:=true` path — reconciled
from two previously-separate keys during the code-analysis/fixes pass;
`gyro_sample_duration_sec` is now shared the same way, as of the
automatic-startup-calibration pass),
`min_battery_voltage`, `enable_sys_obs` — see each key's own `# Consumed by:`
comment in `stack_params.yaml`.

## calibration_common.py

Shared, non-node module (no `main()`, not a console_script entry point)
introduced by the automatic-startup-calibration pass: the exit-code scheme
both calibration nodes' `main()` ends in (`EXIT_SUCCESS`/
`EXIT_INSUFFICIENT_SAMPLES`/`EXIT_NOT_STATIONARY`/`EXIT_MISSING_DEPENDENCY`),
`resolve_source_vesc_yaml_path()` (moved here from
`sensor_covariance_calibration_node.py`, still re-exported there for
backward compatibility), `write_vesc_yaml()` (the backup-then-patch-then-
prune writer both nodes now share), and `StationaryGate` (the pre-sampling
"confirm the car is actually stationary" gate both nodes now use in their
respective stationary-sampling modes).

## Known limitations

- `gyro_bias_calibration_node`'s x/y-axis measurements stay
  **manual-reference only** — `robot_localization`'s EKF config has no
  explicit per-axis bias field for `imu0` (only `vyaw`/z matters for the
  planar fusion). The z-axis (`vyaw`) bias, however, **is** now auto-applied
  (see the node table above) — this bullet used to describe the whole node
  before that changed.
- `sensor_covariance_calibration_node`/`gyro_bias_calibration_node` require
  `ruamel.yaml` to actually write `vesc.yaml` — now a real `rosdep`-tracked
  `exec_depend` (`python3-ruamel.yaml` resolves via `rosdep`, verified;
  previously undeclared on the mistaken assumption it had no rosdep key at
  all). `system_observer_node` still requires `jetson-stats` (`jtop`), which
  genuinely has no `rosdep` key — still a manual `pip`/`apt` install,
  documented as a `package.xml` comment; degrades gracefully if missing
  (unlike a missing `ruamel.yaml`, which fails the write outright —
  `EXIT_MISSING_DEPENDENCY` — though computed values are still logged
  either way).
- Both stationary calibration nodes now need a live `VescStateStamped`
  publisher (normally `vesc_driver_node`, on `state_topic`, default
  `/sensors/core`) even to begin, for the pre-sampling stationary check —
  a real behavior change for `gyro_bias_calibration_node`, which previously
  only needed `imu_topic` and could be bench-tested off the vehicle.
- `light_motion` calibration mode is deliberately `ros2 run`-only, and has
  no stationary check of its own (it inherently drives the car on purpose)
  — see the
  in-package README for the exact reason (stdin forwarding) and invocation.
