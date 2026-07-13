"""Static base_link -> laser TF for the Hokuyo LiDAR mount.

Not derived from the URDF (roboracer.urdf.xacro's sensors.xacro does define a
`laser` link, but that xacro is currently only wired up in f1tenth_sim's
robot_state_publisher path, not the real-hardware stack_bringup path) -- kept
as a standalone static_transform_publisher so it doesn't depend on bringing
up the full robot_state_publisher / mesh pipeline on the real car.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # LiDAR is mounted symmetrically to the ZED2 across the y-z plane
    # (x -> -x), i.e. 0.27 m behind base_link at the same height.
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['-0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser'],
    )

    return LaunchDescription([static_tf_node])
