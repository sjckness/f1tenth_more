"""
wifi_coverage_node (f1tenth_comms): the car's side of the link to the base.

Registered as the `comms` component in f1tenth_bringup/config/components.yaml.
The supervisor starts it only with stack_params.yaml enable_comms: true
(default false); stack_bringup.launch.py includes it on the same key. With
the stack already up, on demand:

    ./scripts/stackctl.py start comms

or alone:

    ros2 launch f1tenth_comms comms.launch.py

Parameters come from config/comms.yaml; comms_params_file:=<file> replaces
it. The node publishes /comms/* only: nothing in it drives, stops or aborts
the car.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('f1tenth_comms'), 'config', 'comms.yaml')
    params_file = LaunchConfiguration('comms_params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'comms_params_file', default_value=default_params,
            description='Parameter file for wifi_coverage_node.'),
        DeclareLaunchArgument(
            'enable_wifi_coverage', default_value='true',
            description='Run wifi_coverage_node (/comms/coverage_*).'),
        Node(
            package='f1tenth_comms',
            executable='wifi_coverage_node',
            name='wifi_coverage_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration('enable_wifi_coverage')),
        ),
    ])
