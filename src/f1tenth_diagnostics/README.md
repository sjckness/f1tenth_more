# f1tenth_diagnostics

Calibration and diagnostic tooling for the F1TENTH stack: one-off measurement/
validation nodes that shouldn't live inside the production control/perception/
hardware packages.

Convention for every node added here:
- Self-contained and single-purpose (one node per measurement).
- Safe to run standalone, at any time, without side effects on the rest of the
  stack: no modifying live parameters on other nodes, no publishing onto
  control topics -- unless a node is explicitly a diagnostic tool for that
  (e.g. an odom noise-floor logger, which only *reads*).
- Documented below: what it measures, how to run it, what output to expect,
  and where the resulting values should be applied.

## calibration.launch.py

`gyro_bias_calibration_node` and `sensor_covariance_calibration_node` (both
documented individually below) are launched together via one consolidated
launch file -- both are stationary samplers meant to be run as a single
calibration pass:

```bash
ros2 launch f1tenth_diagnostics calibration.launch.py
```

Optional launch arguments (shared `imu_topic`, plus each node's own sample
window so they're independently tunable from one command):
- `imu_topic` (default `/sensors/imu/raw`, matching
  `f1tenth_bringup/config/ekf.yaml`'s `imu0` source)
- `odom_topic` (default `/odom`)
- `gyro_sample_duration_sec` (default `30.0`)
- `min_samples` (default `150` -- logs a warning if `gyro_bias_calibration_node`
  collected fewer samples than this in the window)
- `calibration_duration_sec` (default `60.0`)

## gyro_bias_calibration_node

**What it measures:** the static bias (DC offset) of the VESC IMU's gyro
(`sensor_msgs/Imu.angular_velocity`, all three axes), by averaging samples
over a fixed window while the car is stationary and level.

**How to run:** see `calibration.launch.py` above (`gyro_sample_duration_sec` /
`min_samples` launch arguments).

Keep the car completely still for the full sampling window. The node never
publishes anything and never touches any other node's parameters -- it only
subscribes to the IMU topic.

**Expected output:** on completion, the node logs the mean and standard
deviation of `angular_velocity.x/y/z` over the sampling window, e.g.:

```
[gyro_bias_calibration_node]: Gyro bias over 487 samples:
[gyro_bias_calibration_node]:   angular_velocity.x: mean=+0.001234 rad/s  stdev=0.000456 rad/s
[gyro_bias_calibration_node]:   angular_velocity.y: mean=-0.000789 rad/s  stdev=0.000321 rad/s
[gyro_bias_calibration_node]:   angular_velocity.z: mean=+0.002101 rad/s  stdev=0.000512 rad/s
[gyro_bias_calibration_node]: vyaw (z) bias = +0.002101 rad/s -- ...
```

A low stdev relative to the mean indicates a stable bias measurement; a large
stdev suggests vibration, an unlevel surface, or the car wasn't fully
stationary.

**Where to apply it:** `f1tenth_bringup/config/ekf.yaml`'s `imu0` fusion only
keeps `vyaw` (`angular_velocity.z`) -- see its `imu0_config` comment. As of
this writing, `robot_localization`'s EKF config has no explicit per-axis bias
field for `imu0`, so this is currently a **manual reference measurement**,
not an automatically-applied correction: note the measured `vyaw` bias down
and account for it wherever the raw gyro reading is consumed (e.g. before
deciding the EKF's fused yaw rate is drifting for some other reason, or if a
future bias-removal step is added upstream of the EKF). Do not edit
`ekf.yaml` from this tool -- that stays a manual, reviewed change.

## sensor_covariance_calibration_node

**What it measures:** message-level (Level 1) covariance for the VESC IMU
driver and `vesc_to_odom_node_backup`, by running Welford's online variance
(flat O(1) memory, no sample buffer) over `sample_duration_sec` (default
`60.0`) while the car is stationary and level, on:
- `imu_topic` (default `/sensors/imu/raw`) -> `gyro_variance_x/y/z`,
  `accel_variance_x/y/z`
- `odom_topic` (default `/odom`) -> `vx_variance`

`twist.twist.linear.y` is **not** calibrated: `vesc_to_odom_node_backup`
hardcodes it to the constant `0.0` (Ackermann kinematics assumption), never
derived from a sensor, so its variance is always exactly `0.0` by
construction -- not a meaningful measurement.

Unlike `gyro_bias_calibration_node`, **this node writes its result**: it
backs up `vesc.yaml` (timestamped copy alongside the original) and then
patches only the 7 variance keys in place via `ruamel.yaml`'s round-trip
mode, preserving all existing comments/formatting/ordering. Requires
`ruamel.yaml` (`pip install ruamel.yaml` or
`apt install python3-ruamel.yaml`) -- not a standard ROS dependency, so it's
not in `package.xml`.

**How to run standalone (`stationary` mode, the default):** see
`calibration.launch.py` above (`calibration_duration_sec` launch
argument).

Or automatically as part of VESC bringup -- see
`f1tenth_hardware/launch/vesc.launch.py`'s `calibration:=true` argument,
which runs this node first and only starts the VESC driver/odometry nodes
once it exits, so they pick up the freshly-calibrated values on their first
(and only) startup of that run. (That path also never touches
`calibration_mode` -- it only runs `stationary`.)

**`light_motion` mode — `ros2 run` only, never `ros2 launch`.** This mode
additionally drives a short, human-confirmed constant-velocity straight line
to get a real non-zero `vx_variance` (a stationary sample is structurally
stuck at exactly `0.0` -- see the node's own module docstring). Like
`vesc_tuning`'s `steering_calibration_node`/`speed_sweep_diagnostic_node`,
its confirmation gate (`confirm_light_motion_start()`) blocks on a real
`input()` call, and `ros2 launch` does not reliably forward stdin to a
launched node's process -- `calibration.launch.py` deliberately does **not**
expose `calibration_mode`/`light_motion_*` for exactly this reason (it
previously did, via `emulate_tty=True`, which does *not* fix stdin
forwarding -- that combination hung forever right after "Stop command
published -- call confirm_light_motion_start() to begin driving" with no
way to respond). The node itself now also fails fast with a clear log
message instead of hanging if it's ever started without a real stdin
(checks `sys.stdin.isatty()`).

Run it directly:
```bash
ros2 run f1tenth_diagnostics sensor_covariance_calibration_node --ros-args \
  -p calibration_mode:=light_motion \
  -p vesc_yaml_path:=/path/to/f1tenth_bringup/config/vesc.yaml \
  -p light_motion_target_speed:=0.3 \
  -p light_motion_settle_sec:=1.0 \
  -p light_motion_duration_sec:=5.0
```
All five are plain `declare_parameter` calls the node accepts via
`--ros-args -p` regardless of invocation method (the same parameters
`calibration.launch.py` used to pass as launch arguments) -- the values
above are the current `stack_params.yaml` defaults, so only
`vesc_yaml_path` actually needs to be passed explicitly (the node's own
default only resolves correctly under `--symlink-install`, which this
workspace does not use -- see `resolve_source_vesc_yaml_path()`'s
docstring). `odom_topic`, `command_topic`, `servo_topic`,
`speed_to_erpm_gain`, `speed_to_erpm_offset`, and `steering_center` are also
overridable the same way if the defaults don't match your setup. Ensure the
car has ≥ 2 m of clear, flat space ahead before pressing ENTER at the
prompt.

**Where the values are applied:** `f1tenth_bringup/config/vesc.yaml`, in
place -- the same source-tree file both `vesc_driver_node` and
`vesc_to_odom_node` already read their parameters from at startup. The
default `vesc_yaml_path` is resolved by following the *installed*
`vesc.yaml`'s symlink back to its source file, which only works if the
workspace was built with `colcon build --symlink-install`; the node logs a
warning at startup if the resolved path doesn't look like a source tree.

## battery_voltage_check_node

**What it checks:** a ONE-TIME startup gate, not continuous monitoring.
Samples `VescStateStamped.state.voltage_input` on `state_topic` (default
`/sensors/core`, published by `vesc_driver_node`) for `sample_window_sec`
(default `2.0`s -- at `vesc_driver_node`'s 50Hz telemetry poll rate that's
~100 samples), averages it, and compares against `min_battery_voltage`
(default `10.8`). A short averaging window is used rather than a single
reading because no historical noise data for `voltage_input` exists in this
repo to justify trusting one sample near a hard safety cutoff.

**How it's used:** exclusively via `f1tenth_hardware/launch/vesc.launch.py`
-- it's not meant to be run standalone in normal use, since it needs a live
`vesc_driver_node` publishing `/sensors/core` to sample from. `vesc.launch.py`
launches a standalone precheck `vesc_driver_node` instance, runs this node
against it, and inspects its process exit code (0 = pass, 1 = fail) via a
launch event handler to decide whether the rest of the drive stack
(`ackermann_to_vesc_node`, the rest of the sensor driver group, `ekf_node`,
Nav2) is allowed to launch at all. On failure, the precheck `vesc_driver_node`
is left running (harmless telemetry-only) so you can watch the voltage
recover on a charger without needing to relaunch anything.

**Expected output:**

```
[battery_voltage_check_node]: Sampling battery voltage on "/sensors/core" for 2.0s (min_battery_voltage=10.8V).
[battery_voltage_check_node]: Battery check passed: 12.3V >= 10.8V minimum (n=98 samples).
```

or, on failure:

```
[battery_voltage_check_node]: STARTUP ABORTED: battery voltage 10.2V is below minimum 10.8V -- charge or replace battery before starting (n=101 samples).
```

This node still checks once, at boot, and never again for the life of the
process -- it is unchanged and still the only thing gating whether the drive
stack launches at all. Continuous voltage monitoring during operation now
exists separately, see `diagnostics_server_node` below, which was the
previously out-of-scope "BT condition node" integration.

## system_observer_node

**What it publishes:** `f1tenth_messages/SystemStatus` (CPU/RAM via `psutil`,
Jetson GPU/EMC/temps via `jtop`) on `/diagnostics/system_status` at
`publish_rate_hz` (default `1.0` Hz). See the node's own module docstring for
jtop connection/fallback behavior.

**Gated behind `enable_sys_obs`** (default `true`, in
`f1tenth_params/config/stack_params.yaml`): when `false`,
`system_observer.launch.py` returns a `LaunchDescription` with no `Node` at
all -- the executable never starts, wherever this launch file is included
from (`stack_bringup.launch.py`, `components.yaml`'s `diagnostics` component,
or standalone). `f1tenth_behavior`'s BT emergency lane reads the same
`enable_sys_obs` value to decide whether its `IsSystemOverheated` condition
is even constructed, so disabling this is a true "fully skipped" state on
both sides, not just "publishes nothing."

## diagnostics_server_node

**What it does:** a persistent coordinator node with two independent jobs,
deliberately *not* gated behind `enable_sys_obs` (battery safety monitoring
must keep working even with system observability disabled):

1. **Continuous battery monitoring.** Subscribes to `state_topic` (default
   `/sensors/core`, same source `battery_voltage_check_node` samples) and
   publishes `f1tenth_messages/BatteryStatus` on
   `/diagnostics/battery_status` at `battery_check_rate_hz` (default `2.0`
   Hz), averaging samples received since the last publish tick. `has_data`
   stays `false` (and `ok` stays `false`) until the first sample has actually
   arrived, so a cold start can't be misread as "battery critically low."
   This is what `f1tenth_behavior`'s `IsBatteryLow` BT condition subscribes
   to (see that behaviour's own docstring) -- the emergency lane, not the
   startup gate above.
2. **On-demand diagnostics.** Serves `~/run_diagnostics`
   (`f1tenth_messages/srv/RunDiagnostics`, resolves to
   `/diagnostics_server_node/run_diagnostics`): returns the current battery
   status plus the last-received `SystemStatus` (with `sys_obs_enabled` in
   the response telling you whether `system_status` is meaningful, since
   nothing publishes it at all when `system_observer_node` is disabled).

```bash
ros2 launch f1tenth_diagnostics diagnostics_server.launch.py
ros2 service call /diagnostics_server_node/run_diagnostics f1tenth_messages/srv/RunDiagnostics
```

**How it's used:** included unconditionally (never gated) by
`stack_bringup.launch.py` / `components.yaml`'s `diagnostics` component,
alongside `system_observer.launch.py`.
