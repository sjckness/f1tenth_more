"""Nav2 map_server: serves the static map on /map (nav_msgs/OccupancyGrid,
latched/transient-local QoS).

One of nav2.launch.py's selectable per-component pieces (see that file for the
enable_nav2_map switch and node_names list construction) -- this file only
declares the args map_server itself needs. map_server is a Nav2 lifecycle
node: it stays `unconfigured` until an external lifecycle_manager activates
it. nav2.launch.py's shared lifecycle_manager_navigation does this whenever
this file is included through the orchestrator (see task 3's decision there);
launched standalone, map_server will sit unconfigured with no manager to
activate it.
"""

from f1tenth_params.param_defaults import get_path_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_map, map_desc = get_path_default('map', package='f1tenth_navigation')
    map_yaml_arg = DeclareLaunchArgument(
        'map', default_value=default_map, description=map_desc,
    )
    # use_sim_time is deliberately NOT sourced from stack_params.yaml -- sim and real
    # hardware need different defaults (see stack_params.yaml's header comment), so it
    # stays a local, per-file literal.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true.',
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'topic_name': 'map',
            'frame_id': 'map',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([map_yaml_arg, use_sim_time_arg, map_server_node])
