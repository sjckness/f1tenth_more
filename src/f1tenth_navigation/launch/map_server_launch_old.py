"""DEPRECATED -- superseded by nav2_bringup.launch.py, which merged map_server (and its
lifecycle management) into the single unified lifecycle_manager_navigation alongside the
rest of the Nav2 stack. No longer included from anywhere; kept here for reference only.

Static map server.

Brings up nav2_map_server with the bundled placeholder map and a
nav2_lifecycle_manager configured with `autostart=True` so the map_server
node is transitioned from `unconfigured` → `inactive` → `active`
automatically. After lifecycle activation, the map is published on the
standard `/map` topic as `nav_msgs/OccupancyGrid` (latched / transient
local QoS).

Foxglove visualization
----------------------
Once foxglove_bridge is connected, the `/map` topic can be visualized in:
  * the **Map** panel (uses OccupancyGrid layers natively), or
  * the **3D** panel with an *OccupancyGrid* layer added on `/map`.
The Map panel is the most direct choice for a top-down 2D map view.

TODO: foxglove_bridge is not currently launched anywhere in this stack.
`f1tenth_bringup/package.xml` declares `rosbridge_server` as a dependency,
but neither rosbridge nor foxglove_bridge is started by the existing
bringup. To actually see `/map` in Foxglove Studio, add a foxglove_bridge
launch (apt: `ros-humble-foxglove-bridge`) — e.g.::

    Node(package='foxglove_bridge', executable='foxglove_bridge',
         name='foxglove_bridge', parameters=[{'port': 8765}])

either in this launch file or in stack_bringup_launch.py.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_map = PathJoinSubstitution([
        FindPackageShare('f1tenth_navigation'),
        'maps',
        'track_bw.yaml',
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
        description='Auto-activate the map_server lifecycle node.',
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

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': ['map_server'],
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

    return LaunchDescription([
        map_yaml_arg,
        use_sim_time_arg,
        autostart_arg,
        publish_odom_tf_arg,
        map_server_node,
        lifecycle_manager_node,
        map_to_odom_tf,
        odom_tf_broadcaster_node,
    ])
