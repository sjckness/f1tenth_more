"""Nav2 behavior_server (spin/backup/wait recovery behaviors, invoked by
bt_navigator's internal recovery tree).

One of nav2.launch.py's selectable per-component pieces (see that file for the
enable_nav2_behavior_server switch and node_names list construction) -- this
file only declares the arg behavior_server itself needs (the shared
nav2_params file). behavior_server is a Nav2 lifecycle node: it stays
`unconfigured` until an external lifecycle_manager activates it.
nav2.launch.py's shared lifecycle_manager_navigation does this whenever this
file is included through the orchestrator; launched standalone, it will sit
unconfigured with no manager to activate it.
"""

from f1tenth_params.param_defaults import get_path_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_params_default, nav2_params_desc = get_path_default(
        'nav2_params_config', package='f1tenth_navigation')
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params', default_value=nav2_params_default, description=nav2_params_desc,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[LaunchConfiguration('nav2_params')],
    )

    return LaunchDescription([nav2_params_arg, behavior_server])
