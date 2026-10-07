"""reset_manager -> /reset_all. The `reset_manager` component in components.yaml.

    ros2 service call /reset_all std_srvs/srv/Trigger

Parameters come from config/reset_manager.yaml; nav2.enabled comes from
stack_params' enable_nav2, so the reset clears Nav2 costmaps exactly when
Nav2 is running. Override the file with config:=<your.yaml>.

Only meaningful under the component supervisor: the wheel_odom, slam and
hooks steps restart components through its /restart_component, which the
non-supervisor stack_bringup.launch.py path does not have.
"""

import os

from ament_index_python.packages import get_package_share_directory
from f1tenth_params.param_defaults import get_value

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_la = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(
            get_package_share_directory('f1tenth_bringup'), 'config', 'reset_manager.yaml'),
        description='Parameter file for reset_manager.')

    node = Node(
        package='f1tenth_bringup',
        executable='reset_manager',
        name='reset_manager',
        output='screen',
        parameters=[LaunchConfiguration('config'),
                    {'nav2.enabled': bool(get_value('enable_nav2'))}],
    )
    return LaunchDescription([config_la, node])
