"""Nav2 controller_server (FollowPath / RegulatedPurePursuitController --
tolerances tuned for an Ackermann-steered car, see nav2_params.yaml).

One of nav2.launch.py's selectable per-component pieces (see that file for the
enable_nav2_controller switch and node_names list construction) -- this file
only declares the arg controller_server itself needs (the shared nav2_params
file). controller_server is a Nav2 lifecycle node: it stays `unconfigured`
until an external lifecycle_manager activates it. nav2.launch.py's shared
lifecycle_manager_navigation does this whenever this file is included through
the orchestrator; launched standalone, it will sit unconfigured with no
manager to activate it.
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

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('nav2_params')],
        # nav2_regulated_pure_pursuit_controller's raw Twist output is remapped here so
        # it lands on the ackermann_mux's "navigation" lane instead of the bare cmd_vel
        # topic -- twist_to_ackermann_node (f1tenth_behavior) picks it up from there.
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    return LaunchDescription([nav2_params_arg, controller_server])
