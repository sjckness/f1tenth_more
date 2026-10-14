"""Sensors-only bringup for hand-pushed raw recording (tools/record_raw.sh).

    ros2 launch tools/sensors_only.launch.py

Starts the Fast-DDS discovery server (same as supervisor_bringup), the robot
description and LiDAR TF, the ZED (camera.launch.py: wrapper + its TF), the
Hokuyo (lidar.launch.py: urg_node), the VESC driver and its odometry
(vesc_driver_node, vesc_to_odom_node, base_link -> imu TF) and the local EKF
(ekf.launch.py: /odometry/filtered).

NO COMMAND PATH. This file deliberately does NOT use vesc.launch.py: that file
always starts ackermann_to_vesc_node (the only node that turns
/ackermann_drive into VESC motor and servo commands), runs the battery-check
and calibration sequence, and includes navigation. Nothing here publishes on,
or subscribes to and converts, any drive topic. There is no ackermann_mux,
MPC, BT or ackermann_to_vesc_node, so vesc_driver_node receives no
commands/motor/* or commands/servo/* message from anything in this launch. The
car rolls freely and is pushed by hand.
"""

import os

from ament_index_python.packages import get_package_share_directory
from f1tenth_params.param_defaults import get_default, get_path_default
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _include(package, launch_file):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package), 'launch', launch_file)))


def generate_launch_description():
    address = str(get_default('discovery_server_address')[0])
    port = str(get_default('discovery_server_port')[0])
    vesc_config = get_path_default('vesc_config')[0]
    steering_config = get_path_default('steering_calibration_config',
                                       package='f1tenth_hardware')[0]
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DISCOVERY_SERVER', f'{address}:{port}'),
        ExecuteProcess(
            cmd=['python3', os.path.join(get_package_share_directory('f1tenth_bringup'),
                                         'scripts', 'ensure_discovery_server.py'),
                 address, port],
            name='fastdds_discovery_server', output='screen',
            respawn=True, respawn_delay=2.0),
        _include('f1tenth_description', 'description.launch.py'),
        _include('f1tenth_perception', 'camera.launch.py'),
        _include('f1tenth_perception', 'lidar.launch.py'),
        Node(package='vesc_driver', executable='vesc_driver_node', name='vesc_driver_node',
             parameters=[vesc_config, steering_config]),
        Node(package='vesc_ackermann', executable='vesc_to_odom_node_backup',
             name='vesc_to_odom_node', parameters=[vesc_config, steering_config]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_baselink_to_imu',
             arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu']),
        _include('f1tenth_localization', 'ekf.launch.py'),
    ])
