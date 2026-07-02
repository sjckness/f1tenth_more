"""F1TENTH perception bringup.

Starts, in one launch file:
  1. The ZED2 stereo camera (RGB + depth) via the upstream zed_wrapper launch,
     configured by config/zed2_perception.yaml (override-only).
  2. The Hokuyo LiDAR (urg_node) -> /scan, frame `laser`  [gated by `use_lidar`].
  3. The YOLO 2D detector node -> /camera/detections (+ /camera/image_annotated).

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
    zed_wrapper_share = get_package_share_directory('zed_wrapper')

    zed_override_config = os.path.join(
        perception_share, 'config', 'zed2_perception.yaml')
    zed_launch_file = os.path.join(
        zed_wrapper_share, 'launch', 'zed_camera.launch.py')

    # ---- launch arguments --------------------------------------------------
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar', default_value='false',
        description='Start the Hokuyo LiDAR. Set false to test without it.')
    # The ZED wrapper publishes fixed topic names it cannot rename; we point the
    # detector at the canonical RGB topic for camera_name:=zed2.
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic', default_value='/zed2/zed_node/rgb/image_rect_color',
        description='RGB image topic the YOLO detector subscribes to.')
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='../models/yolo26m.pt',
        description='Path to the YOLO model file (empty = passthrough mode).')

    # ---- 1) ZED2 camera (RGB + depth) -------------------------------------
    # camera_name:=zed2 -> topics under /zed2/zed_node/... and frames zed2_*.
    # ros_params_override_path applies our override ON TOP of the wrapper config.
    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file),
        launch_arguments={
            'camera_model': 'zed2',
            'camera_name': 'zed2',
            'ros_params_override_path': zed_override_config,
        }.items(),
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

    # ---- 3) YOLO 2D detector ----------------------------------------------
    yolo_detector = Node(
        package='f1tenth_perception',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('rgb_topic'),
            'detections_topic': '/camera/detections',
            'annotated_topic': '/camera/image_annotated',
            'model_path': LaunchConfiguration('model_path'),
        }],
    )

    return LaunchDescription([
        use_lidar_arg,
        rgb_topic_arg,
        model_path_arg,
        zed_camera,
        urg_node,
        yolo_detector,
    ])
