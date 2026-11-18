r"""Run one component launch file on simulated time (component_supervisor_node, sim mode).

    ros2 launch f1tenth_bringup sim_component.launch.py \\
        component_package:=<pkg> component_launch_file:=<file> [sim_params_file:=<yaml>] [k:=v ...]

Used by component_supervisor_node instead of `ros2 launch <pkg> <file> k:=v` when
supervisor_bringup.launch.py runs with sim:=isaac (or use_sim_time:=true); the car never
goes through this file. It includes the component's launch file unchanged, after:

- SetLaunchConfiguration use_sim_time 'true': for the launch files that pass use_sim_time
  explicitly (description.launch.py's robot_state_publisher and joint_state_publisher,
  map_only.launch.py's map_server). A node's own parameter beats a global one, so the
  global SetParameter alone would lose there.
- SetParameter use_sim_time True: for every other node, none of which sets it. This also
  reaches composable nodes.
- the sim parameter overlay (config/sim/isaac.yaml), if given: as global parameters
  (SetParametersFromFile, for keys the nodes do not set themselves) and as the launch
  configuration params_overlay, which launch files that need it (reset_manager.launch.py)
  append AFTER their own parameter files.

Every other k:=v reaches the included file as a launch configuration, as with plain
`ros2 launch`. Proof that use_sim_time reaches every node: output/jazzy_port/phase3.md.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter, SetParametersFromFile
from launch_ros.substitutions import FindPackageShare


def _overlay(context):
    path = LaunchConfiguration('sim_params_file').perform(context)
    if not path:
        return []
    return [SetParametersFromFile(path), SetLaunchConfiguration('params_overlay', path)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('component_package',
                              description='package of the component launch file'),
        DeclareLaunchArgument('component_launch_file', description='component launch file name'),
        DeclareLaunchArgument('sim_params_file', default_value='',
                              description='sim parameter overlay (config/sim/isaac.yaml)'),
        SetLaunchConfiguration('use_sim_time', 'true'),
        SetParameter(name='use_sim_time', value=True),
        OpaqueFunction(function=_overlay),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(LaunchConfiguration('component_package')), 'launch',
            LaunchConfiguration('component_launch_file')]))),
    ])
