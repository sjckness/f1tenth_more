"""
Replay wrapper: one PRODUCTION launch file, unmodified, on sim time.

use_sim_time is forced on for every node it starts.

Runs on: the Orin host (native Humble), the harness container (Jazzy), linus.
Started by replay_localization.sh for its launch-file layers (ekf_global,
slam, costmap, mpc), so each side replays its own production configuration
(every parameter its launch file passes from its own stack_params.yaml)
rather than a hand-kept copy that could drift.

  ros2 launch layer_replay.launch.py package:=PKG launch_file:=FILE [ARG:=V ...]

The production launch files have no use_sim_time argument; launch_ros's
SetParameter applies it to every Node the included file starts (lifecycle
nodes included). Every other launch argument is visible to the included file
unchanged (e.g. cpu_affinity:=6,7 for mpc_corr.launch.py on an 8-core host;
the production default 10,11 is the Orin's).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def _include(context):
    package = LaunchConfiguration('package').perform(context)
    launch_file = LaunchConfiguration('launch_file').perform(context)
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('package', description='package of the production launch file'),
        DeclareLaunchArgument('launch_file', description='the production launch file'),
        SetParameter(name='use_sim_time', value=True),
        OpaqueFunction(function=_include),
    ])
