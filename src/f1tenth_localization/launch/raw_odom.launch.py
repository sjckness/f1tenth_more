"""Raw, unfiltered map -> odom fallback: broadcasts map -> odom as a direct
mirror of /odom via raw_odom_map_tf_node, in place of the EKF
(f1tenth_localization/launch/ekf.launch.py) while EKF tuning is in progress.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    raw_odom_map_tf_node = Node(
        package='f1tenth_localization',
        executable='raw_odom_map_tf_node',
        name='raw_odom_map_tf_node',
        output='screen',
    )

    return LaunchDescription([raw_odom_map_tf_node])
