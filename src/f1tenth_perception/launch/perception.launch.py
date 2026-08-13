"""F1TENTH perception bringup -- standalone entry point for testing
perception in isolation from the rest of the stack (stack_bringup.launch.py
brings these same pieces up itself; this file exists so perception can be
exercised on its own).

Starts, in one launch file:
  1. The ZED2 stereo camera (RGB + depth), via camera.launch.py (shared with
     stack_bringup.launch.py -- one definition of the camera bringup).
  2. The Hokuyo LiDAR (urg_node) -> /scan, frame `laser`, via lidar.launch.py
     (shared with f1tenth_navigation/nav2.launch.py -- one definition
     of the LiDAR bringup, gated by `use_lidar`, config sourced from
     f1tenth_bringup/config/sensors.yaml).
  3. YOLO 2D detection + 2D-to-3D fusion, via detection.launch.py (shared with
     stack_bringup.launch.py -- avoids independent definitions of the same
     nodes drifting out of sync).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    perception_share = get_package_share_directory('f1tenth_perception')

    # ---- 1) ZED2 camera (RGB + depth) -------------------------------------
    # camera_source is one of the 6 stack-wide branching args (see f1tenth_bringup/
    # config/stack_params.yaml) -- camera.launch.py reads it directly from that yaml
    # itself now, so it's not passed as a launch_arguments override here anymore
    # (previously hardcoded 'zed' by hand at this call site -- a second, silently
    # driftable copy of the same value; camera.launch.py's own yaml read is now the
    # only place this is decided).
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'camera.launch.py')
        ),
    )

    # ---- 2) Hokuyo LiDAR ----------------------------------------------------
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'lidar.launch.py')
        ),
    )

    # ---- 3) YOLO 2D detector + 2D-to-3D fusion -----------------------------
    # Same camera_source note as the camera include above -- detection.launch.py
    # reads it directly from stack_params.yaml itself now.
    detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'detection.launch.py')
        ),
    )

    return LaunchDescription([
        camera,
        lidar,
        detection,
    ])
