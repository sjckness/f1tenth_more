"""Standalone Nav2 map_server, lifecycle-managed on its own -- for when Nav2 itself
isn't running at all (enable_nav2:=false, mpc_corr driving instead -- see
navigation.launch.py, which is what actually includes this file) but /map should
still be published (RViz/Foxglove map display, and anything else that expects it).
Not one of nav2.launch.py's own enable_nav2_<component> pieces -- that orchestrator
and its shared lifecycle_manager_navigation only run at all when enable_nav2:=true.

map_server is a Nav2 lifecycle node (see map.launch.py's own docstring): it sits
`unconfigured` forever with no external lifecycle_manager to activate it. This file
pairs map.launch.py with its own single-node lifecycle_manager, named
lifecycle_manager_map -- deliberately NOT lifecycle_manager_navigation, so nothing
that specifically waits on that name's readiness (e.g. f1tenth_behavior's
wait_for_trigger_service_node, gating the BT's Nav2-driven lane on the full 5-node
Nav2 stack being active) can mistake this smaller one-node manager for that one.
"""

import os

from ament_index_python.packages import get_package_share_directory

from f1tenth_params.param_defaults import get_default, get_path_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_map, map_desc = get_path_default('map', package='f1tenth_navigation')
    map_yaml_arg = DeclareLaunchArgument(
        'map', default_value=default_map, description=map_desc,
    )
    # use_sim_time deliberately NOT sourced from stack_params.yaml -- same reasoning
    # as map.launch.py/nav2.launch.py's own copies of this arg (see stack_params.yaml's
    # header comment).
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true.',
    )
    autostart_default, autostart_desc = get_default('autostart')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value=str(autostart_default), description=autostart_desc,
    )

    map_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('f1tenth_navigation'), 'launch', 'map.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': ['map_server'],
        }],
    )

    return LaunchDescription([
        map_yaml_arg, use_sim_time_arg, autostart_arg, map_include, lifecycle_manager,
    ])
