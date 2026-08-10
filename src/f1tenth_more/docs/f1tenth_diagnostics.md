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
| `gyro_bias_calibration_node` | One-shot measurement | (logs only) | Static gyro DC-offset bias over a stationary sampling window. Never publishes, never touches other nodes' params — reference measurement only, no automatic correction applied anywhere. |
| `sensor_covariance_calibration_node` | One-shot measurement + writer | (logs + patches `vesc.yaml`) | Message-level covariance (Welford's online variance) for VESC IMU + `vesc_to_odom_node_backup`, `stationary` mode by default. **Writes its result**: backs up `vesc.yaml` (timestamped copy) then patches only the 7 variance keys in place via `ruamel.yaml` round-trip mode (preserves comments/formatting). `light_motion` mode (real non-zero `vx_variance` via a human-confirmed drive) is `ros2 run`-only, never `ros2 launch` — its confirmation gate blocks on `input()`, which `ros2 launch` doesn't reliably forward to a subprocess's stdin. |
| `battery_voltage_check_node` | One-shot startup gate | (logs, exit code) | Samples `VescStateStamped.state.voltage_input` on `/sensors/core` for `sample_window_sec`, compares the mean against `min_battery_voltage`. Exit code (0/1) gates whether `vesc.launch.py` lets the rest of the drive stack launch at all. Not continuous — checks once at boot, never again. |
| `system_observer_node` | Continuous | `/diagnostics/system_status` (`f1tenth_messages/SystemStatus`) | CPU/RAM (`psutil`) + Jetson GPU/EMC/temps (`jtop`, graceful fallback if not connected) at `publish_rate_hz`. Gated behind `enable_sys_obs` — `system_observer.launch.py` returns an empty `LaunchDescription` when disabled (executable never starts at all, not just idles). |
| `diagnostics_server_node` | Continuous + on-demand | `/diagnostics/battery_status` (`f1tenth_messages/BatteryStatus`); `~/run_diagnostics` (`f1tenth_messages/RunDiagnostics`) | Two jobs, deliberately **not** gated behind `enable_sys_obs` (battery safety must keep working regardless): continuous battery monitoring (averages `/sensors/core` samples since the last publish tick, `has_data` guards a cold start) and an on-demand snapshot service (battery + last `SystemStatus`, with `sys_obs_enabled` telling the caller whether the latter is meaningful). |

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
from two previously-separate keys during the code-analysis/fixes pass),
`min_battery_voltage`, `enable_sys_obs` — see each key's own `# Consumed by:`
comment in `stack_params.yaml`.

## Known limitations

- `gyro_bias_calibration_node`'s measurement is **manual-reference only** —
  `robot_localization`'s EKF config has no explicit per-axis bias field for
  `imu0`, so there's no automatic place to feed the measured bias back into.
- `sensor_covariance_calibration_node` requires `ruamel.yaml` and
  `system_observer_node` requires `jetson-stats` (`jtop`) — neither has a
  `rosdep` key, so neither is a hard build dependency; both are documented
  in `package.xml`'s own comments as manual `pip`/`apt` installs instead
  (the latter degrades gracefully if missing, the former will fail outright
  if actually invoked without it).
- `light_motion` calibration mode is deliberately `ros2 run`-only — see the
  in-package README for the exact reason (stdin forwarding) and invocation.
