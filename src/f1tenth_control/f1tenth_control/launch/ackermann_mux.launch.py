"""Ackermann command mux plus the last-resort LiDAR stop that feeds its safety lane.

The mux arbitrates between drive-command sources (MPC, teleop, startup
sequence) by priority, publishing the single /ackermann_drive consumed by
vesc_ackermann's ackermann_to_vesc_node.

Not the upstream ackermann_mux package's own launch file: that one loads
three separate locks/topics/joystick config files and a different remap
target than this stack uses, so it's not a drop-in here -- mux_config below
is our single combined config (f1tenth_bringup/config/mux.yaml).

HARD_STOP_NODE LAUNCHES HERE, with the mux, and that placement is the point.
It is the last-resort stop for everything perception does not recognise (see
f1tenth_control/hard_stop.py for the whole rationale), and it has to survive the
behaviour tree -- so it must not share the `behavior` supervisor component,
whose whole job is to be restartable. It shares `control` with the mux instead,
which is the honest coupling: the mux is the one process whose death already
means nothing can command a stop at all.
"""

from f1tenth_params.param_defaults import get_path_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mux_config, mux_config_desc = get_path_default('mux_config')
    mux_la = DeclareLaunchArgument(
        'mux_config', default_value=mux_config, description=mux_config_desc)

    # ---- last-resort LiDAR stop (f1tenth_control/hard_stop_node.py) --------
    # Enabled by default: it is a backstop, and a backstop that ships off is
    # not a backstop. The thresholds are the ones the avoidance pass settled
    # on; hard_stop_range_m 0.20 is measured FROM THE LASER (base_link x 0.305),
    # so it fires 0.10 m ahead of the 0.405 m bumper. The behaviour tree's own
    # 0.40 m proximity e-stop is a separate, untouched mechanism.
    hard_stop_enabled_la = DeclareLaunchArgument(
        'hard_stop_enabled', default_value='true',
        description="false disables the last-resort LiDAR stop entirely "
                    "(hard_stop_node stays up and keeps publishing its "
                    "diagnostic, but never claims the safety lane). Default "
                    "true: it is the only thing standing behind the MPC for an "
                    "obstacle perception does not recognise.")
    hard_stop_range_la = DeclareLaunchArgument(
        'hard_stop_range_m', default_value='0.20',
        description="Trigger radius (m) measured FROM THE LIDAR ORIGIN "
                    "(base_link x 0.305), i.e. 0.10 m ahead of the 0.405 m "
                    "bumper. Returns inside the car's own footprint polygon "
                    "plus footprint_self_margin_m are excluded as self-hits "
                    "and never trigger.")
    hard_stop_debounce_la = DeclareLaunchArgument(
        'hard_stop_debounce_scans', default_value='2',
        description="Consecutive scans with a triggering return before the "
                    "stop engages. 2 at the measured 40 Hz is 50 ms, far "
                    "inside the drive train's own response, so waiting costs "
                    "nothing and not waiting stops the car on single bad "
                    "returns.")
    hard_stop_clear_la = DeclareLaunchArgument(
        'hard_stop_clear_s', default_value='0.5',
        description="How long (s) the zone must be clear before the latched "
                    "stop releases. Deliberately asymmetric with the debounce: "
                    "engaging needs evidence, releasing needs confidence, and "
                    "an object at 0.18 m is exactly the one the LiDAR is about "
                    "to lose inside range_min or behind the body.")

    hard_stop_allow_reverse_la = DeclareLaunchArgument(
        'hard_stop_allow_reverse', default_value='true',
        description="true: with every triggering return ahead of the bumper the "
                    "stop blocks only forward motion -- it stays quiet while the "
                    "command the mux would pass is negative, so the reverse "
                    "manoeuvre can back the car out. false: it stops every "
                    "direction, as before.")

    hard_stop_node = Node(
        package='f1tenth_control',
        executable='hard_stop_node',
        name='hard_stop_node',
        output='screen',
        parameters=[{
            'hard_stop_enabled': ParameterValue(
                LaunchConfiguration('hard_stop_enabled'), value_type=bool),
            'hard_stop_range_m': ParameterValue(
                LaunchConfiguration('hard_stop_range_m'), value_type=float),
            'hard_stop_debounce_scans': ParameterValue(
                LaunchConfiguration('hard_stop_debounce_scans'), value_type=int),
            'hard_stop_clear_s': ParameterValue(
                LaunchConfiguration('hard_stop_clear_s'), value_type=float),
            'hard_stop_allow_reverse': ParameterValue(
                LaunchConfiguration('hard_stop_allow_reverse'), value_type=bool),
        }],
    )

    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd', 'ackermann_drive')],
    )

    return LaunchDescription([
        mux_la, hard_stop_enabled_la, hard_stop_range_la,
        hard_stop_debounce_la, hard_stop_clear_la, hard_stop_allow_reverse_la,
        ackermann_mux_node, hard_stop_node,
    ])
