"""reset_manager -> /reset_all. The `reset_manager` component in components.yaml.

    ros2 service call /reset_all std_srvs/srv/Trigger

Parameters come from config/reset_manager.yaml; nav2.enabled comes from
stack_params' enable_nav2, so the reset clears Nav2 costmaps exactly when
Nav2 is running, and hooks.skip_components from enable_wall_distance, so a
disabled wall_distance is not started by the hooks step. Override the file
with config:=<your.yaml>.

Only meaningful under the component supervisor: the wheel_odom, slam and
hooks steps restart components through its /restart_component, which the
non-supervisor stack_bringup.launch.py path does not have.
"""

import os

from ament_index_python.packages import get_package_share_directory
from f1tenth_params.param_defaults import get_value

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context):
    parameters = [LaunchConfiguration('config'),
                  {'nav2.enabled': bool(get_value('enable_nav2')),
                   # cpu-cleanup: restarting a component the supervisor did
                   # not start would START it, so a disabled one is skipped.
                   # [''] is "none": ROS 2 cannot type an empty list.
                   'hooks.skip_components': (
                       [''] if get_value('enable_wall_distance') else ['wall_distance'])}]
    # Sim overlay (config/sim/isaac.yaml, set by sim_component.launch.py in sim:=isaac):
    # appended AFTER the node's own file so it wins (a global parameter file would lose to
    # config). Empty on the car: the parameter list is exactly the two entries above.
    overlay = LaunchConfiguration('params_overlay').perform(context)
    if overlay:
        parameters.append(overlay)
    return [Node(
        package='f1tenth_bringup',
        executable='reset_manager',
        name='reset_manager',
        output='screen',
        parameters=parameters,
    )]


def generate_launch_description():
    config_la = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(
            get_package_share_directory('f1tenth_bringup'), 'config', 'reset_manager.yaml'),
        description='Parameter file for reset_manager.')
    overlay_la = DeclareLaunchArgument(
        'params_overlay', default_value='',
        description="Extra parameter file loaded after 'config' (the sim overlay in sim:=isaac).")
    return LaunchDescription([config_la, overlay_la, OpaqueFunction(function=_node)])
