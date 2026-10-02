"""Phase 3 replay wrapper: the production mpc_corr.launch.py, unmodified, with
use_sim_time forced on for every node it starts.

mpc_corr.launch.py passes ~30 parameters from stack_params.yaml and pins the
node with a `taskset -c <cpu_affinity>` prefix, but has no use_sim_time
argument. Rather than re-listing its parameters by hand (and risking a drift
between the replay and the deployed configuration), this includes the file
as-is under launch_ros's SetParameter, which applies use_sim_time to the
included Node. Works on Humble and Jazzy.

Launch arguments are forwarded unchanged (e.g. cpu_affinity:=10,11, the
production default).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter


def generate_launch_description():
    mpc_launch = os.path.join(
        get_package_share_directory('f1tenth_control'), 'launch', 'mpc_corr.launch.py')
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(mpc_launch)),
    ])
