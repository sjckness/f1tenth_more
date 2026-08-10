# f1tenth_hardware

Owns the VESC drive chain: battery pre-flight gate, optional live sensor
covariance calibration, `ackermann_to_vesc_node`, `vesc_to_odom_node`, the
VESC driver itself, and the static `base_link → imu` TF. A thin metapackage
(no code of its own beyond one launch file) wrapping the vendored `vesc`
submodule's real nodes.

## Nodes (all from the vendored `vesc` submodule — see `f1tenth_external`'s doc)

| Node | Package | Role |
|---|---|---|
| `vesc_driver_node` | `vesc_driver` | Talks to the physical VESC over serial (`/dev/ttyACM0`, via `transport_drivers`' `serial_driver` linked as a C++ library). Publishes `sensors/core` (`VescStateStamped`), `sensors/imu` (`VescImuStamped`), `sensors/imu/raw` (`sensor_msgs/Imu`). Subscribes `commands/motor/{duty_cycle,current,brake,speed,position}`, `commands/servo/position`. |
| `ackermann_to_vesc_node` | `vesc_ackermann` | Converts `AckermannDriveStamped` → VESC motor/servo commands. Subscribes `ackermann_cmd` (hardcoded in source, remapped here to `ackermann_drive` to receive `ackermann_mux`'s arbitrated output). |
| `vesc_to_odom_node_backup` (launched under node name `vesc_to_odom_node`) | `vesc_ackermann` | Raw wheel/steering dead-reckoning odometry (no IMU fusion) — publishes `/odom`. **This is the deployed variant**, launched under a node name that matches `vesc.yaml`'s name-keyed param block despite the executable itself being named `..._backup`; the submodule's other, newer `vesc_to_odom_node` (a 5-state IMU+VESC Kalman filter) exists in the same package but is not referenced by any launch file in this workspace. |
| `static_transform_publisher` (as `static_baselink_to_imu`) | `tf2_ros` | Fixed `base_link → imu` TF — the VESC IMU is treated as rigidly co-located with `base_link` (zero offset; adjust if the IMU is physically off-center). |

## Launch files

### `vesc.launch.py`

The one launch file in this package, included by `stack_bringup.launch.py`
and by `components.yaml`'s `hardware`/`calibrate_hardware` component entries.
Three layered behaviors:

1. **Battery pre-flight gate** (unconditional, runs first regardless of
   `calibration`): a standalone precheck `vesc_driver_node` instance gives
   `battery_voltage_check_node` (`f1tenth_diagnostics`) something to sample
   `/sensors/core` from. Pass → precheck driver shut down, full stack
   launches ~2s later (OS process-teardown buffer for the serial port).
   Fail → full stack never launches; the precheck driver is deliberately
   left running (harmless, telemetry-only) so voltage recovery can be
   watched without a relaunch.
2. **`calibration:=false`** (default): driver group (`vesc_driver_node`,
   `vesc_to_odom_node`, static IMU TF) launches once, with a crash handler
   (`OnProcessExit` → whole-launch `Shutdown()` if `vesc_driver_node` dies —
   drive-by-wire has no meaning without it).
3. **`calibration:=true`**: driver group v1 launches *without* a crash
   handler (its exit is expected, not a failure) so `sensor_covariance_calibration_node`
   (`f1tenth_diagnostics`) has live sensor data to measure against. On
   calibration exit, all 3 v1 processes are shut down individually
   (`ShutdownProcess` targeted, not a whole-launch `Shutdown()`); once a
   closure-captured counter confirms all 3 have actually exited (order
   isn't guaranteed), a fixed 2.0s teardown buffer runs, then a fresh driver
   group v2 launches — now reading the config file calibration just
   overwrote. `release_downstream` (default `true`) then decides whether
   this file also includes `f1tenth_localization/ekf.launch.py` and
   `f1tenth_navigation/navigation.launch.py` itself (`stack_bringup.launch.py`'s
   behavior) or leaves that to something else (`component_supervisor_node`'s
   `calibrate_hardware` component passes `release_downstream:=false` so
   localization/navigation stay independently-restartable components).

**Config file layering**: `vesc_config` (default `f1tenth_bringup/config/vesc.yaml`)
and `steering_calibration_config` (default `f1tenth_hardware/config/steering_calibration.yaml`)
are both passed to `vesc_driver_node`/`ackermann_to_vesc_node`'s `parameters=[...]`
list, `steering_calibration_config` **second** — ROS 2 merges multiple params
files in argument order, later files winning on a key collision, so the 5
steering-calibration keys (`servo_min`, `servo_max`,
`steering_angle_to_servo_offset`, `steering_angle_to_servo_gain_left/_right`)
always come from `steering_calibration.yaml`, never from `vesc.yaml` (which
no longer defines them at all, to prevent the two files disagreeing).
Neither node hot-reloads either file — both are read once at startup, no
`set-parameters` callback registered in either — so a config change (hand
edit or a live write from `vesc_tuning`'s `steering_calibration_node.py`)
only takes effect on that node's next restart.

## Config

| File | Holds |
|---|---|
| `f1tenth_hardware/config/steering_calibration.yaml` | The 5 steering-calibration keys (see above) — single source of truth, also read/live-overwritten directly (not via ROS params) by `vesc_tuning`'s `steering_calibration_node.py` during manual calibration runs. |
| `f1tenth_bringup/config/vesc.yaml` | Everything else VESC-related (ERPM gain/offset, hardware limits, IMU covariance placeholders, `vesc_to_odom_node`'s block) — not owned by this package, but loaded first (and overridden second-if-colliding by `steering_calibration.yaml`) by this package's launch file. |

## Consumed `stack_params.yaml` keys

`vesc_config`, `steering_calibration_config`, `calibration`,
`calibration_duration_sec`, `release_downstream`, `min_battery_voltage` — see
each key's own `# Consumed by:` comment in `f1tenth_params/config/stack_params.yaml`
for the authoritative per-key consumer list.

## `vesc_tuning` (inside the vendored `vesc` submodule, not a first-party package)

A human-in-the-loop calibration package living at
`src/f1tenth_hardware/vesc/vesc_tuning/` (not covered by `f1tenth_external`'s
doc, since it's specific to this submodule and referenced by several
first-party comments): `steering_calibration_node.py` (interactively retunes
`steering_angle_to_servo_gain_left/_right`, writes back to
`steering_calibration.yaml`) and `speed_tuning_node.py`/
`speed_sweep_diagnostic_node.py` (empirical `speed_to_erpm_gain` calibration,
writes back to `vesc.yaml`). Run via `ros2 run vesc_tuning ...` directly, not
launched by anything in this workspace — see that package's own `README.md`
for the exact invocation.

## Known limitations

- The `vesc` submodule's working tree currently has **uncommitted local
  modifications** on top of its pinned commit (`vesc_ackermann`/`vesc_driver`
  header/source files, plus a partially-staged `vesc_tuning` package) — the
  code actually running may not match what the outer superproject's gitlink
  records. This is an ongoing pattern for this submodule (it was true at an
  earlier point in this project's history too, with a different set of
  files, before that round was committed and the pin bumped) rather than a
  one-time incident — worth checking `git -C src/f1tenth_hardware/vesc status`
  before assuming the pinned SHA tells the whole story.
- `vesc_driver`'s `vesc_device_namer` (udev-helper CLI) and the submodule's
  newer `vesc_to_odom_node` (IMU+VESC Kalman filter) are real, buildable code
  that nothing in this workspace launches.
- No dynamic reconfiguration story for any VESC param — every calibration
  workflow here ends in "restart the driver," never a live `ros2 param set`.
