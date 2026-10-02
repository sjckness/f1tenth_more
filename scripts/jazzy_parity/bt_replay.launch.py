"""Phase 4 replay wrapper: the production behavior_bringup.launch.py, unmodified,
with use_sim_time forced on for every node it starts (same approach as
mpc_replay.launch.py: SetParameter over an IncludeLaunchDescription, so the
replay runs exactly the deployed parameters and taskset pinning).

With enable_nav2 false (stack_params.yaml) that file launches
behavior_executor_node and twist_to_ackermann_node directly. The latter only
converts cmd_vel to `drive`; nothing publishes cmd_vel in the replay, and the
isolated domain has no mux or VESC driver.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter


def generate_launch_description():
    bt_launch = os.path.join(
        get_package_share_directory('f1tenth_behavior'), 'launch', 'behavior_bringup.launch.py')
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bt_launch)),
    ])
