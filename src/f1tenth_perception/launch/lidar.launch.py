"""Hokuyo LiDAR (urg_node) -> /scan, frame `laser`.

Extracted out of perception.launch.py's inline urg_node block so
f1tenth_behavior/behavior_bringup.launch.py (which needs /scan for Nav2's costmap
obstacle layer) can reuse it without a third hand-copied params dict -- this file loads
f1tenth_bringup/config/sensors.yaml directly, the same shared-config convention
vesc_launch.py/ackermann_mux_launch.py already use.

perception.launch.py's own inline urg_node Node() is left as-is for now (a pre-existing,
separate duplication its own docstring already flags) -- not reconciled here.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_bringup'), 'config', 'sensors.yaml')

    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar', default_value='True',
        description='Start the Hokuyo LiDAR. Set false to test without it.')
    sensors_config_arg = DeclareLaunchArgument(
        'sensors_config', default_value=sensors_config)

    urg_node = Node(
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[LaunchConfiguration('sensors_config')],
    )

    return LaunchDescription([use_lidar_arg, sensors_config_arg, urg_node])
