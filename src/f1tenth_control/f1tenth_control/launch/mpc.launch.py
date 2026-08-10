"""MPC controller. If it dies, the whole launch tree is shut down (the car
has no autonomous drive-command source without it).

Tuning gains (qn, qv, qalpha, qddelta, alat_max, a_min/max, v_min/max, v_ref,
sine_amp, sine_period) are andre_mpc_node.py's own already-declared ROS params,
sourced here from f1tenth_params/config/stack_params.yaml -- previously this file
passed no parameters=[...] at all, so they were only ever the in-code defaults;
values here match those defaults exactly, so wiring them up changes nothing
behaviorally by itself.
"""

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    qn_default, qn_desc = get_default('qn')
    qn_la = DeclareLaunchArgument(
        'qn', default_value=str(qn_default), description=qn_desc)
    qv_default, qv_desc = get_default('qv')
    qv_la = DeclareLaunchArgument(
        'qv', default_value=str(qv_default), description=qv_desc)
    qalpha_default, qalpha_desc = get_default('qalpha')
    qalpha_la = DeclareLaunchArgument(
        'qalpha', default_value=str(qalpha_default), description=qalpha_desc)
    qddelta_default, qddelta_desc = get_default('qddelta')
    qddelta_la = DeclareLaunchArgument(
        'qddelta', default_value=str(qddelta_default), description=qddelta_desc)
    alat_max_default, alat_max_desc = get_default('alat_max')
    alat_max_la = DeclareLaunchArgument(
        'alat_max', default_value=str(alat_max_default), description=alat_max_desc)
    a_min_default, a_min_desc = get_default('a_min')
    a_min_la = DeclareLaunchArgument(
        'a_min', default_value=str(a_min_default), description=a_min_desc)
    a_max_default, a_max_desc = get_default('a_max')
    a_max_la = DeclareLaunchArgument(
        'a_max', default_value=str(a_max_default), description=a_max_desc)
    v_min_default, v_min_desc = get_default('v_min')
    v_min_la = DeclareLaunchArgument(
        'v_min', default_value=str(v_min_default), description=v_min_desc)
    v_max_default, v_max_desc = get_default('v_max')
    v_max_la = DeclareLaunchArgument(
        'v_max', default_value=str(v_max_default), description=v_max_desc)
    v_ref_default, v_ref_desc = get_default('v_ref')
    v_ref_la = DeclareLaunchArgument(
        'v_ref', default_value=str(v_ref_default), description=v_ref_desc)
    sine_amp_default, sine_amp_desc = get_default('sine_amp')
    sine_amp_la = DeclareLaunchArgument(
        'sine_amp', default_value=str(sine_amp_default), description=sine_amp_desc)
    sine_period_default, sine_period_desc = get_default('sine_period')
    sine_period_la = DeclareLaunchArgument(
        'sine_period', default_value=str(sine_period_default), description=sine_period_desc)

    mpc_node = Node(
        package='mpc_controller',
        executable='andre_mpc_node',
        name='andre_mpc_controller',
        output='screen',
        parameters=[{
            'qn': LaunchConfiguration('qn'),
            'qv': LaunchConfiguration('qv'),
            'qalpha': LaunchConfiguration('qalpha'),
            'qddelta': LaunchConfiguration('qddelta'),
            'alat_max': LaunchConfiguration('alat_max'),
            'a_min': LaunchConfiguration('a_min'),
            'a_max': LaunchConfiguration('a_max'),
            'v_min': LaunchConfiguration('v_min'),
            'v_max': LaunchConfiguration('v_max'),
            'v_ref': LaunchConfiguration('v_ref'),
            'sine_amp': LaunchConfiguration('sine_amp'),
            'sine_period': LaunchConfiguration('sine_period'),
        }],
    )

    return LaunchDescription([
        qn_la, qv_la, qalpha_la, qddelta_la, alat_max_la, a_min_la, a_max_la,
        v_min_la, v_max_la, v_ref_la, sine_amp_la, sine_period_la,
        mpc_node,
        RegisterEventHandler(
            OnProcessExit(target_action=mpc_node, on_exit=[Shutdown()])
        ),
    ])
