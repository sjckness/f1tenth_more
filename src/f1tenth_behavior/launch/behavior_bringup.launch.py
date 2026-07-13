"""py_trees safety-stop supervisor + Twist->Ackermann bridge -- the part that actually
drives.

Included from stack_bringup_launch.py only when use_behavior_tree:=true (mutually
exclusive with mpc_launch.py -- see stack_bringup_launch.py). Depends on
f1tenth_navigation's nav2_bringup.launch.py already running (stack_bringup includes it,
gated by enable_nav2, default true); this file does NOT re-launch the Nav2 stack itself,
to avoid duplicate controller_server/planner_server/bt_navigator nodes if both were
included together.

behavior_executor_node owns the outer BT (obstacle-stop + NavigateThroughPoses).
twist_to_ackermann_node bridges nav2_regulated_pure_pursuit_controller's Twist output
(from nav2_bringup.launch.py's controller_server) onto the ackermann_mux's "navigation"
lane -- this is the only piece that actually puts a Nav2-derived drive command on the
mux, and it's gated behind this same use_behavior_tree arg.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    behavior_share = get_package_share_directory('f1tenth_behavior')

    waypoints_yaml = os.path.join(behavior_share, 'config', 'waypoints.yaml')
    twist_to_ackermann_yaml = os.path.join(
        behavior_share, 'config', 'twist_to_ackermann.yaml')

    behavior_executor_node = Node(
        package='f1tenth_behavior',
        executable='behavior_executor_node',
        name='behavior_executor_node',
        output='screen',
        parameters=[waypoints_yaml],
    )
    twist_to_ackermann_node = Node(
        package='f1tenth_behavior',
        executable='twist_to_ackermann_node',
        name='twist_to_ackermann_node',
        output='screen',
        parameters=[twist_to_ackermann_yaml],
    )

    return LaunchDescription([
        behavior_executor_node,
        twist_to_ackermann_node,
    ])
