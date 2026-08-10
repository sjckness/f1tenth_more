"""Steer-sweep startup sequence.

Runs stack_startup_sequence, which drives a brief steering sweep (right ->
left -> center) shortly after boot as a visual "the stack is alive and the
VESC is responding" check, then leaves the car under normal control.

Included unconditionally by stack_bringup.launch.py's own bringup and registered as
an always-auto-starting component_supervisor_node component (see components.yaml's
startup_sequence entry) -- both bringup paths, not a separate standalone-only file
anymore.

Phase 10: mpc_node_name used to be declared/forwarded to stack_startup_sequence here,
but that node never declared or read a parameter by that name -- it's a pure
steering-sweep node with no MPC awareness (see
f1tenth_bringup/stack_startup_sequence.py: it only publishes AckermannDriveStamped to
a mux topic, it doesn't call SetParameters on anything). Removed rather than wired up,
since there was nothing for the node to do with the value.
"""

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = {}
    for name in ('startup_delay', 'right_duration', 'left_duration', 'center_duration',
                 'max_steering_angle', 'startup_command_topic'):
        default, description = get_default(name)
        args[name] = DeclareLaunchArgument(
            name, default_value=str(default), description=description)

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
        }],
    )

    return LaunchDescription([
        *args.values(),
        startup_sequence,
    ])
