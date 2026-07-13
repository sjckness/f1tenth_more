"""MPC controller. If it dies, the whole launch tree is shut down (the car
has no autonomous drive-command source without it).
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    mpc_node = Node(
        package='mpc_controller',
        executable='andre_mpc_node',
        name='andre_mpc_controller',
        output='screen',
    )

    return LaunchDescription([
        mpc_node,
        RegisterEventHandler(
            OnProcessExit(target_action=mpc_node, on_exit=[Shutdown()])
        ),
    ])
