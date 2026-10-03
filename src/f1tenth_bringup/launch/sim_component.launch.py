r"""
Run one component launch file with every node on simulated time.

Sim mode (supervisor_bringup.launch.py sim:=true).

component_supervisor_node spawns, in sim mode,
  ros2 launch f1tenth_bringup sim_component.launch.py \\
      component_package:=<pkg> component_launch_file:=<file> [<component args>...]
instead of `ros2 launch <pkg> <file> [<component args>...]`. The component's
own args arrive as launch configurations from the command line, exactly as
they would at the top of `ros2 launch <pkg> <file>`, and the included file
reads them unchanged.

use_sim_time reaches every node two ways, because the stack's launch files do
it two ways:
  - SetParameter(use_sim_time=True): a global parameter for every Node
    launched below, including the many that never mention use_sim_time;
  - the launch configuration use_sim_time:=true: launch files that pass
    {'use_sim_time': LaunchConfiguration('use_sim_time')} explicitly
    (description.launch.py, map.launch.py) would otherwise override the
    global value with their own default, false. DeclareLaunchArgument never
    replaces a configuration that is already set.

No /clock publisher here: the simulator provides /clock on the network.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from ros2launch.api import get_share_file_path_from_package


def _include_component(context):
    path = get_share_file_path_from_package(
        package_name=LaunchConfiguration('component_package').perform(context),
        file_name=LaunchConfiguration('component_launch_file').perform(context))
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('component_package',
                              description='Package of the component launch file.'),
        DeclareLaunchArgument('component_launch_file',
                              description='Component launch file, as ros2 launch finds it.'),
        SetLaunchConfiguration('use_sim_time', 'true'),
        SetParameter(name='use_sim_time', value=True),
        OpaqueFunction(function=_include_component),
    ])
