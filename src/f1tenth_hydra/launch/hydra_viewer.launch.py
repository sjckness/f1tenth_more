"""View a Hydra scene graph that runs elsewhere (jetson_min on the Orin) or here.

    ros2 launch f1tenth_hydra hydra_viewer.launch.py [viewer:=rviz|custom|both] [use_sim_time:=true]

- rviz: Hydra's own hydra_visualizer (streaming, /hydra/backend/dsg -> /hydra_visualizer/graph)
  plus RViz with rviz/hydra_hil.rviz: graph layers, agent poses, the live mesh when the profile
  publishes one (workstation_full), camera and semantics images.
- custom: f1tenth_hydra's own Python viewer (hydra_dsg_viewer), graph only (nodes, edges,
  labels), Open3D or matplotlib.
Needs ~/hydra_ws sourced (hydra_visualizer, hydra_msgs): scripts/hydra/hydra_env.sh.
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetRemap


def _setup(context):
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    viewer = LaunchConfiguration('viewer').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    topic = LaunchConfiguration('dsg_topic').perform(context)
    if viewer not in ('rviz', 'custom', 'both'):
        return [LogInfo(msg=f'hydra_viewer: viewer:= must be rviz, custom or both, got {viewer}')]
    own = get_package_share_directory('f1tenth_hydra')
    actions = [SetParameter(name='use_sim_time', value=use_sim_time)]
    if viewer in ('rviz', 'both'):
        try:
            viz = get_package_share_directory('hydra_visualizer')
            hros = get_package_share_directory('hydra_ros')
        except PackageNotFoundError:
            return [LogInfo(msg='hydra_viewer: hydra_visualizer not found -- source '
                                'scripts/hydra/hydra_env.sh (or use viewer:=custom)')]
        actions.append(GroupAction([
            SetRemap(src='/hydra_visualizer/dsg', dst=topic),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(viz, 'launch', 'streaming_visualizer.launch.yaml')),
                launch_arguments={
                    'visualizer_frame': 'hydra_map',
                    'external_plugins_path': os.path.join(
                        hros, 'config', 'hydra_ros_visualizer_plugins.yaml'),
                    'rviz_path': os.path.join(own, 'rviz', 'hydra_hil.rviz'),
                }.items()),
        ]))
    if viewer in ('custom', 'both'):
        actions.append(Node(package='f1tenth_hydra', executable='hydra_dsg_viewer',
                            name='hydra_dsg_viewer', arguments=['--topic', topic],
                            output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='rviz', description='rviz | custom | both'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('dsg_topic', default_value='/hydra/backend/dsg'),
        OpaqueFunction(function=_setup),
    ])
