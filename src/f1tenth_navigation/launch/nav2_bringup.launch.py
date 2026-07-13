"""Static map server + full Nav2 navigation stack: map_server, controller_server,
planner_server, behavior_server, bt_navigator (Nav2's own -- hosts navigate_through_poses
and runs its own internal replanning/recovery tree), local/global costmaps, plus the
LiDAR (costmap obstacle-data source; not part of the default stack_bringup tree
otherwise). All lifecycle-managed together by a single lifecycle_manager_navigation, so
there is exactly one authority bringing map_server up before the costmaps' static_layer
(map_subscribe_transient_local) needs it -- no manual TimerAction delay is used; ordering
relies on the lifecycle manager's own configure/activate sequencing plus the static
layer's transient-local subscription (a late subscriber still gets the latched /map).

Moved here from f1tenth_behavior/launch/navigation.launch.py (Nav2 nodes) and
f1tenth_navigation/launch/map_server_launch_old.py (map_server + its own standalone
lifecycle manager, now merged in) so that f1tenth_behavior's own launch file
(behavior_bringup.launch.py) no longer needs to declare any Nav2 nodes itself -- it only
depends on this file already having been included. Enablement is controlled by the
stack-wide `enable_nav2` launch argument (declared in
f1tenth_localization/launch/ekf_launch.py, forwarded by stack_bringup_launch.py), not by
use_behavior_tree: Nav2 idles with no active goal and produces no drive command on its
own, so it's safe to run independently of which drive-command source is active.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    navigation_share = get_package_share_directory('f1tenth_navigation')
    perception_share = get_package_share_directory('f1tenth_perception')

    nav2_params = os.path.join(navigation_share, 'config', 'nav2_params.yaml')

    default_map = PathJoinSubstitution([
        FindPackageShare('f1tenth_navigation'),
        'maps',
        'square_100m.yaml',
    ])

    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Absolute path to the map.yaml file to serve.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true.',
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Auto-activate the lifecycle_manager_navigation-managed nodes.',
    )
    # vesc_to_odom_node already broadcasts odom -> base_link (vesc.yaml
    # publish_tf:=true), so this alternative broadcaster is OFF by default.
    # Only enable it if vesc_to_odom's TF is disabled, otherwise two publishers
    # fight over the same transform.
    publish_odom_tf_arg = DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='false',
        description='Publish odom -> base_link from /odom via odom_tf_broadcaster '
                    '(leave false when vesc_to_odom_node owns this transform).',
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'topic_name': 'map',
            'frame_id': 'map',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Static map -> odom transform encoding the car's starting pose, so that
    # when /odom reads (0,0,0) at startup base_link appears at the real start
    # (x=2.9639, y=2.1302, yaw=1.8132 rad) in the map frame. This is the
    # inverse-free convention: the transform IS the start pose. Quaternion for
    # yaw=1.8132: qz=sin(yaw/2)=0.787412, qw=cos(yaw/2)=0.616427.
    # arg order: x y z qx qy qz qw parent_frame child_frame
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        output='screen',
        arguments=['2.9639', '2.1302', '0.0',
                   '0.0', '0.0', '0.787412', '0.616427',
                   'map', 'odom'],
    )

    # Optional dynamic odom -> base_link broadcaster (default off; see arg).
    odom_tf_broadcaster_node = Node(
        package='f1tenth_navigation',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_odom_tf')),
    )

    lidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'lidar.launch.py')
        ),
    )

    # map_server listed first: lifecycle_manager configures/activates managed nodes in
    # this order, so map_server reaches active before the costmaps' static_layer
    # (map_subscribe_transient_local: True) needs /map -- though transient-local QoS
    # means a late subscriber would still get the latched map either way.
    lifecycle_nodes = [
        'map_server', 'controller_server', 'planner_server', 'behavior_server',
        'bt_navigator',
    ]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        map_yaml_arg,
        use_sim_time_arg,
        autostart_arg,
        publish_odom_tf_arg,
        map_server_node,
        map_to_odom_tf,
        odom_tf_broadcaster_node,
        lidar_bringup,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])
