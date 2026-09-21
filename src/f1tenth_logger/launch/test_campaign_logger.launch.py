"""Launches test_campaign_logger, the test-campaign recorder. BY HAND ONLY.

    ros2 launch f1tenth_logger test_campaign_logger.launch.py
    ros2 launch f1tenth_logger test_campaign_logger.launch.py root:=/path/to/f1tenth_more

NOT part of any bringup, not even behind a flag: no components.yaml entry, no
include from another launch file (test_test_campaign_isolation.py fails if
one appears). It runs for the length of a test session and then stops.
mission_logger.launch.py, which the supervisor starts with the stack, is a
separate logger with its own node name, output folder and topics. Running
both at once is the normal case.

ROOT. The campaign must land in <f1tenth_more>/first_test_campaing however
this package was built. From an install, the recorder's own upward search
starts under install/ or build/, so the root is resolved here once:
F1TENTH_MORE_ROOT if set, otherwise the workspace holding
src/f1tenth_logger (robot_logger.find_root). It then reaches the node twice,
as its `root` parameter and as F1TENTH_MORE_ROOT in its environment, so
anything it runs agrees. An empty root (an install outside the workspace
with no variable set) makes the node refuse to start and say why, instead of
guessing.
"""

import os

from ament_index_python.packages import get_package_share_directory

from f1tenth_logger.test_campaign.robot_logger import find_root

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_root():
    try:
        return str(find_root())
    except (RuntimeError, NotADirectoryError):
        return ''


def generate_launch_description():
    root_la = DeclareLaunchArgument(
        'root', default_value=_default_root(),
        description='The f1tenth_more workspace; the campaign is <root>/<campaign>. '
                    'Default: $F1TENTH_MORE_ROOT, else the workspace holding '
                    'src/f1tenth_logger.')
    config_la = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(
            get_package_share_directory('f1tenth_logger'), 'config',
            'test_campaign_logger.yaml'),
        description='Parameter file for test_campaign_logger.')

    node = Node(
        package='f1tenth_logger',
        executable='test_campaign_logger',
        name='test_campaign_logger',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config'), {'root': LaunchConfiguration('root')}],
        additional_env={'F1TENTH_MORE_ROOT': LaunchConfiguration('root')},
    )

    return LaunchDescription([
        root_la,
        config_la,
        LogInfo(msg=['[test_campaign] root: ', LaunchConfiguration('root')]),
        node,
    ])
