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

## gyro_bias_calibration_node

**What it measures:** the static bias (DC offset) of the VESC IMU's gyro
(`sensor_msgs/Imu.angular_velocity`, all three axes), by averaging samples
over a fixed window while the car is stationary and level.

**How to run:**

```bash
ros2 launch f1tenth_diagnostics gyro_bias_calibration.launch.py
```

Optional launch arguments:
- `imu_topic` (default `/sensors/imu/raw`, matching
  `f1tenth_bringup/config/ekf.yaml`'s `imu0` source)
- `sample_duration_sec` (default `10.0`)
- `min_samples` (default `50` -- logs a warning if fewer samples were
  collected in the window)

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
