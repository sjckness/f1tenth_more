# sensors_stack_bringup

Bringup (launch + config + udev) for the F1TENTH / RoboRacer sensor suite on the
Jetson Orin AGX. Currently integrates the **Stereolabs ZED 2** stereo camera via
the official [`zed-ros2-wrapper`](https://github.com/stereolabs/zed-ros2-wrapper)
(the ZED SDK bridge — analogous in role to `vesc_driver` in the VESC stack).

> **TODO: LiDAR** — a LiDAR will be added to this metapackage later. The launch
> file, udev rules, URDF and this README already contain placeholders for it.

## Prerequisites (Jetson)

The `zed_ros2_wrapper` submodule is **C++ and CUDA-based** and builds independently
of the Python packages here. Before it will build/run you need, on the Jetson:

1. **ZED SDK** installed system-wide (`/usr/local/zed/`) via the Stereolabs `.run`
   installer for your JetPack version.
2. On **JetPack 6**, a known CMake/CUDA error at first build is fixed with:
   ```bash
   sudo apt install nvidia-jetpack nvidia-jetpack-dev
   ```
3. A **full workspace build** (`colcon build`) with the ZED SDK present. The three
   Python packages of `sensors_stack` build without the SDK; the wrapper does not.

## Install the udev rule

```bash
sudo cp sensors_stack_bringup/udev/99-zed2.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

This creates a stable `/dev/zed2` symlink (ZED 2 USB VID:PID `2b03:f582`).

## Launch

```bash
# Full sensor bringup (camera on)
ros2 launch sensors_stack_bringup sensors_stack.launch.py

# Run without the camera (e.g. on a bench with no hardware) — does NOT crash
ros2 launch sensors_stack_bringup sensors_stack.launch.py use_zed:=false

# use_lidar defaults to false and is a placeholder (no effect yet)
ros2 launch sensors_stack_bringup sensors_stack.launch.py use_lidar:=false
```

Launch arguments:

| Arg | Default | Description |
|---|---|---|
| `use_zed` | `true` | Launch the ZED 2 camera + its static TF (`base_link → zed2_camera_link`). |
| `use_lidar` | `false` | **TODO: LiDAR** placeholder; no effect yet. |
| `zed_params_file` | `config/zed2_params.yaml` | ZED 2 parameter overrides passed via `params_file`. |

RViz is intentionally **not** started (too heavy for runtime on the Jetson).

## Published topics

With the ZED 2 enabled, topics appear under `/zed2/zed_node/` (the `~` below).
All are enabled by default with the provided `config/zed2_params.yaml`.

| Topic | Type | Default | Notes |
|---|---|---|---|
| `~/rgb/color/rect/image` | `sensor_msgs/Image` | ✅ | Left rectified RGB |
| `~/right/color/rect/image` | `sensor_msgs/Image` | ✅ | Right rectified RGB |
| `~/rgb/color/rect/camera_info` | `sensor_msgs/CameraInfo` | ✅ | Intrinsics for left |
| `~/depth/depth_registered` | `sensor_msgs/Image` | ✅ | Depth map aligned to RGB |
| `~/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | ✅ | Colored 3D point cloud |
| `~/imu/data` | `sensor_msgs/Imu` | ✅ | Gyro + accelerometer (fused) |
| `~/imu/data_raw` | `sensor_msgs/Imu` | ✅ | Raw IMU |
| `~/imu/mag` | `sensor_msgs/MagneticField` | ✅ | Magnetometer (ZED 2 only) |
| `~/atm_press` | `sensor_msgs/FluidPressure` | ✅ | Barometer (ZED 2 only) |
| `~/temperature/imu` | `sensor_msgs/Temperature` | ✅ | IMU die temperature |
| `~/temperature/left` | `sensor_msgs/Temperature` | ✅ | Left CMOS temp (ZED 2 only) |
| `~/temperature/right` | `sensor_msgs/Temperature` | ✅ | Right CMOS temp (ZED 2 only) |
| `~/odom` | `nav_msgs/Odometry` | ✅ | Visual-inertial odometry (ZED SDK) |
| `~/pose` | `geometry_msgs/PoseStamped` | ✅ | 6DOF pose from ZED SDK tracking |

> Note: exact topic names can vary slightly by `zed-ros2-wrapper` version. Verify
> with `ros2 topic list` after launch.

## TF

`base_link → zed2_camera_link` is published by a `static_transform_publisher` in
the launch file (only when `use_zed:=true`). The placeholder mount pose
(x=0.15, y=0.0, z=0.10) **must be measured on the physical car** and kept in sync
with `sensors_stack_description/urdf/sensors.urdf.xacro`. ZED-internal TF publishing
is disabled in the config (`pos_tracking_module.publish_tf: false`) so the odom
stack owns the tree.

## Tuning

See [`TUNING.md`](TUNING.md) for the `depth_mode` (PERFORMANCE / QUALITY / ULTRA)
CPU/GPU tradeoffs on the Jetson Orin AGX.
