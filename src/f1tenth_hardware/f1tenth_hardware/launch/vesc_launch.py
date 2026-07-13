"""VESC hardware chain: ackermann->VESC conversion, odometry, and the VESC
driver itself, plus the static base_link->imu TF for the VESC's onboard IMU.

All three nodes share one config file (vesc_config, defaults to
f1tenth_bringup/config/vesc.yaml -- the single source of truth for VESC
calibration across the stack). If vesc_driver_node dies, the whole launch
tree is shut down (drive-by-wire has no meaning without it).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_bringup'), 'config', 'vesc.yaml')
    vesc_la = DeclareLaunchArgument('vesc_config', default_value=vesc_config)

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')],
        # ackermann_mux_launch.py remaps its output ackermann_cmd_out ->
        # ackermann_drive; this node's own subscription topic ("ackermann_cmd") is
        # hardcoded in vesc_ackermann's source, not configurable via param, so it
        # must be remapped here to actually receive the mux's arbitrated output.
        remappings=[('ackermann_cmd', 'ackermann_drive')],
    )
    # Raw odometry source: pure bicycle-model integration of the VESC speed +
    # steering (no IMU fusion), publishing /odom. IMU + odometry fusion is
    # handled by the robot_localization EKF (f1tenth_localization), which
    # replaces the old in-node Kalman filter. publish_tf is false (vesc.yaml)
    # so the EKF owns the odom -> base_link transform.
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node_backup',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')],
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')],
        sigterm_timeout='5',
        sigkill_timeout='2',
    )
    # VESC IMU is rigidly mounted on the chassis; treat it as coincident with
    # base_link. The EKF needs this transform to bring sensors/imu/raw (frame
    # 'imu') into base_link. Adjust the offset if the IMU is calibrated.
    static_imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu'],
    )

    return LaunchDescription([
        vesc_la,
        ackermann_to_vesc_node,
        vesc_to_odom_node,
        vesc_driver_node,
        static_imu_tf_node,
        RegisterEventHandler(
            OnProcessExit(target_action=vesc_driver_node, on_exit=[Shutdown()])
        ),
    ])
