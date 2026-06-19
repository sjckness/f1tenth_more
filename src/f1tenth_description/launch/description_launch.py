"""F1TENTH robot description launch.

Launches ONLY robot_state_publisher with the roboracer xacro. No Gazebo, no
hardware drivers. Reused by:
  * f1tenth_sim  (sim_bringup_launch.py includes this, then adds the simulator)
  * f1tenth_bringup (real hardware) - to publish the robot TF tree.

Args:
  use_sim_time   - publish TF with the sim clock (sim only). Default false.
  use_sim        - emit the ign_ros2_control gazebo plugin + sim sensors. False.
  enable_sensors - include LiDAR / camera / IMU links + sensors. Default true.
  control_config - path to ros2_control controllers.yaml (sim). Default ''.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim = LaunchConfiguration('use_sim')
    enable_sensors = LaunchConfiguration('enable_sensors')
    control_config = LaunchConfiguration('control_config')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use the simulation clock if true.')
    declare_use_sim = DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='Emit the ign_ros2_control gazebo plugin + sim sensors.')
    declare_enable_sensors = DeclareLaunchArgument(
        'enable_sensors', default_value='true',
        description='Include LiDAR / camera / IMU sensor links.')
    declare_control_config = DeclareLaunchArgument(
        'control_config', default_value='',
        description='Path to the ros2_control controllers.yaml (sim only).')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('f1tenth_description'), 'urdf', 'roboracer.urdf.xacro'])

    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_sim:=', use_sim,
        ' enable_sensors:=', enable_sensors,
        ' pkg_share:=', pkg_share,
        ' control_config:=', control_config,
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(robot_description_content,
                                                 value_type=str)},
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_sim,
        declare_enable_sensors,
        declare_control_config,
        robot_state_publisher,
    ])
