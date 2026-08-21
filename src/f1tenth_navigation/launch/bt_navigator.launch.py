"""Nav2 bt_navigator: hosts navigate_through_poses and runs Nav2's own internal
replanning/recovery tree (separate from f1tenth_behavior's outer
safety-stop-and-navigate BT, which only calls into this one as an action).

One of nav2.launch.py's selectable per-component pieces (see that file for the
enable_nav2_bt_navigator switch and node_names list construction) -- this file
only declares the arg bt_navigator itself needs (the shared nav2_params file).
bt_navigator is a Nav2 lifecycle node: it stays `unconfigured` until an
external lifecycle_manager activates it. nav2.launch.py's shared
lifecycle_manager_navigation does this whenever this file is included through
the orchestrator; launched standalone, it will sit unconfigured with no
manager to activate it.
"""

from f1tenth_params.param_defaults import get_odom_topic, get_path_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_params_default, nav2_params_desc = get_path_default(
        'nav2_params_config', package='f1tenth_navigation')
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params', default_value=nav2_params_default, description=nav2_params_desc,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        # Second params entry wins on key collision (same merge-order idiom as
        # vesc.launch.py's vesc_config + steering_calibration_config) -- overrides
        # nav2_params.yaml's own static odom_topic: /odometry/filtered with whatever
        # localization_source actually resolves to, so bt_navigator doesn't silently
        # keep expecting the EKF's topic while running in the raw_odom fallback.
        parameters=[
            LaunchConfiguration('nav2_params'),
            {'odom_topic': get_odom_topic()},
        ],
    )

    return LaunchDescription([nav2_params_arg, bt_navigator])
