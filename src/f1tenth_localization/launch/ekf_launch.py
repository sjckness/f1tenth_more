"""robot_localization EKF: fuses /odom (x, y, yaw, from vesc_to_odom_node)
with the VESC IMU (/sensors/imu/raw: yaw rate + linear acceleration) and
broadcasts the odom -> base_link transform. Replaces the old vesc_to_odom
in-node Kalman filter (vesc_to_odom_node's own publish_tf is left false).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('f1tenth_bringup'), 'config', 'ekf.yaml')
    ekf_la = DeclareLaunchArgument('ekf_config', default_value=ekf_config)

    # Not consumed by this file -- declared here (rather than in f1tenth_navigation or
    # stack_bringup_launch.py) so the localization package owns the single on/off switch
    # for whichever localization-dependent stack (Nav2) is layered on top of this EKF.
    # ROS 2 launch arguments declared anywhere in the included-launch-file tree are
    # visible/overridable from the top-level `ros2 launch` invocation, so
    # stack_bringup_launch.py reads this same LaunchConfiguration to gate its include of
    # f1tenth_navigation's nav2_bringup.launch.py.
    enable_nav2_la = DeclareLaunchArgument(
        'enable_nav2', default_value='true',
        description='Bring up the Nav2 stack (f1tenth_navigation/nav2_bringup.launch.py). '
                    'Independent of use_behavior_tree: Nav2 idles with no active goal and '
                    'puts nothing on the ackermann_mux by itself.')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config')],
    )

    return LaunchDescription([ekf_la, enable_nav2_la, ekf_node])
