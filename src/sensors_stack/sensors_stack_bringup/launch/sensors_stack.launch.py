# Copyright 2026 Andreas Steffens
#
# Bringup for the F1TENTH / RoboRacer sensor suite on the Jetson Orin AGX.
#
# Currently launches the Stereolabs ZED 2 (via the official zed-ros2-wrapper, the
# SDK bridge) and publishes the static base_link -> zed2_camera_link transform.
# A LiDAR will be added later — see the clearly marked placeholder below.
#
# Design notes:
#   * `use_zed`   (default true)  : launch the ZED 2 camera + its static TF.
#   * `use_lidar` (default false) : placeholder for the future LiDAR.
#   * RViz is intentionally NOT started here — too heavy for runtime on the Jetson.
#   * The zed_wrapper package path is resolved lazily via FindPackageShare and is
#     only evaluated when use_zed is true, so `use_zed:=false` works even on a
#     machine where the (C++) zed_wrapper submodule has not been built.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ----- Launch arguments -------------------------------------------------
    use_zed = LaunchConfiguration('use_zed')
    use_lidar = LaunchConfiguration('use_lidar')
    zed_params_file = LaunchConfiguration('zed_params_file')

    declare_use_zed = DeclareLaunchArgument(
        'use_zed', default_value='true',
        description='Launch the ZED 2 camera and its static TF. Set false to run without hardware.')

    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar', default_value='false',
        description='Placeholder for the future LiDAR. No effect yet.')

    declare_zed_params = DeclareLaunchArgument(
        'zed_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('sensors_stack_bringup'), 'config', 'zed2_params.yaml']),
        description='ZED 2 parameter overrides (merged on top of the wrapper defaults).')

    # ----- ZED 2 camera (SDK bridge) ---------------------------------------
    # Include the official wrapper launch. FindPackageShare('zed_wrapper') is a
    # substitution, so it is only resolved when this group actually runs (i.e.
    # when use_zed is true) — keeping use_zed:=false crash-free.
    zed_group = GroupAction(
        condition=IfCondition(use_zed),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
                launch_arguments={
                    'camera_model': 'zed2',
                    'camera_name': 'zed2',
                    # The pinned wrapper (humble-v4.2.5, for ZED SDK 4.2) names the
                    # override-file argument `ros_params_override_path`. Newer
                    # wrappers (v5.x) renamed it to `params_file` — switch this key
                    # if you ever bump the submodule tag.
                    'ros_params_override_path': zed_params_file,
                }.items(),
            ),
            # Static TF: base_link -> zed2_camera_link.
            # Placeholder translation; MUST be measured on the physical car.
            # Mirrors the origin defined in sensors_stack_description/urdf/sensors.urdf.xacro.
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_zed2_camera_link',
                arguments=[
                    '--x', '0.15', '--y', '0.0', '--z', '0.10',
                    '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'zed2_camera_link',
                ],
            ),
        ],
    )

    # ----- LiDAR (placeholder) ---------------------------------------------
    # TODO: LiDAR — when the LiDAR is added, include its launch here, gated on
    # `use_lidar`. Example skeleton (kept commented until the driver exists):
    #
    # lidar_group = GroupAction(
    #     condition=IfCondition(use_lidar),
    #     actions=[
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 PathJoinSubstitution(
    #                     [FindPackageShare('<lidar_driver_pkg>'),
    #                      'launch', '<lidar>.launch.py'])),
    #         ),
    #         Node(
    #             package='tf2_ros', executable='static_transform_publisher',
    #             name='base_link_to_lidar_link',
    #             arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
    #                        '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
    #         ),
    #     ],
    # )

    return LaunchDescription([
        declare_use_zed,
        declare_use_lidar,
        declare_zed_params,
        zed_group,
        # lidar_group,  # TODO: LiDAR — uncomment when the LiDAR driver is added.
    ])
