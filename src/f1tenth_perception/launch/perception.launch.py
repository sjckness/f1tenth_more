"""F1TENTH perception bringup -- standalone entry point for testing
perception in isolation from the rest of the stack (stack_bringup_launch.py
brings these same pieces up itself; this file exists so perception can be
exercised on its own).

Starts, in one launch file:
  1. The ZED2 stereo camera (RGB + depth), via camera.launch.py (shared with
     stack_bringup_launch.py -- one definition of the camera bringup).
  2. The Hokuyo LiDAR (urg_node) -> /scan, frame `laser`  [gated by `use_lidar`].
  3. YOLO 2D detection + 2D-to-3D fusion, via detection.launch.py (shared with
     stack_bringup_launch.py -- avoids independent definitions of the same
     nodes drifting out of sync).

The LiDAR config (IP/port/frame) is replicated from
f1tenth_bringup/config/sensors.yaml. If you enable the LiDAR here, REMOVE the
duplicate urg_node from f1tenth_bringup bringup launches (see README/notes) to
avoid two nodes driving the same device.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_share = get_package_share_directory('f1tenth_perception')

    # ---- launch arguments --------------------------------------------------
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar', default_value='True',
        description='Start the Hokuyo LiDAR. Set false to test without it.')

    # ---- 1) ZED2 camera (RGB + depth) -------------------------------------
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'camera.launch.py')
        ),
        launch_arguments={'camera_source': 'zed'}.items(),
    )

    # ---- 2) Hokuyo LiDAR (replicated from f1tenth_bringup/config/sensors.yaml) -
    urg_node = Node(
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[{
            'ip_address': '192.168.0.10',
            'ip_port': 10940,
            'serial_baud': 115200,
            'laser_frame_id': 'laser',
            'angle_max': 3.14,
            'angle_min': -3.14,
            'calibrate_time': False,
            'default_user_latency': 0.0,
            'cluster': 1,
            'skip': 0,
        }],
    )

    # ---- 3) YOLO 2D detector + 2D-to-3D fusion -----------------------------
    detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'detection.launch.py')
        ),
        launch_arguments={'camera_source': 'zed'}.items(),
    )

    return LaunchDescription([
        use_lidar_arg,
        camera,
        urg_node,
        detection,
    ])
