"""Steer-sweep startup sequence.

Runs stack_startup_sequence, which drives a brief steering sweep (right ->
left -> center) shortly after boot as a visual "the stack is alive and the
VESC is responding" check, then leaves the car under normal control.

Not included by stack_bringup_launch.py by default (kept as a standalone,
opt-in launch file; see stack_bringup_launch.py for how to wire it back in).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    startup_delay_la = DeclareLaunchArgument(
        'startup_delay', default_value='5.0',
        description='Seconds to wait before steer sweep starts.')
    right_duration_la = DeclareLaunchArgument(
        'right_duration', default_value='2.0',
        description='Seconds to hold max right steering.')
    left_duration_la = DeclareLaunchArgument(
        'left_duration', default_value='2.0',
        description='Seconds to hold max left steering.')
    center_duration_la = DeclareLaunchArgument(
        'center_duration', default_value='1.0',
        description='Seconds to hold center steering.')
    max_steering_angle_la = DeclareLaunchArgument(
        'max_steering_angle', default_value='0.18',
        description='Max steering angle in radians.')
    startup_command_topic_la = DeclareLaunchArgument(
        'startup_command_topic', default_value='/teleop',
        description='High-priority mux input topic for the steer sweep.')
    mpc_node_name_la = DeclareLaunchArgument(
        'mpc_node_name', default_value='/andre_mpc_controller',
        description='Fully-qualified MPC node name for SetParameters call.')

    startup_sequence = Node(
        package='f1tenth_bringup',
        executable='stack_startup_sequence',
        name='stack_startup_sequence',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'startup_delay':      LaunchConfiguration('startup_delay'),
            'right_duration':     LaunchConfiguration('right_duration'),
            'left_duration':      LaunchConfiguration('left_duration'),
            'center_duration':    LaunchConfiguration('center_duration'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'command_topic':      LaunchConfiguration('startup_command_topic'),
            'mpc_node_name':      LaunchConfiguration('mpc_node_name'),
        }],
    )

    return LaunchDescription([
        startup_delay_la, right_duration_la, left_duration_la,
        center_duration_la, max_steering_angle_la,
        startup_command_topic_la, mpc_node_name_la,
        startup_sequence,
    ])
