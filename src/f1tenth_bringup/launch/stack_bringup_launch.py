import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import Shutdown
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- config files --------------------------------------------------
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_bringup'), 'config', 'joy_teleop.yaml')
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_bringup'), 'config', 'vesc.yaml')
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_bringup'), 'config', 'sensors.yaml')
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_bringup'), 'config', 'mux.yaml')
    ekf_config = os.path.join(
        get_package_share_directory('f1tenth_bringup'), 'config', 'ekf.yaml')

    joy_la     = DeclareLaunchArgument('joy_config',     default_value=joy_teleop_config)
    vesc_la    = DeclareLaunchArgument('vesc_config',    default_value=vesc_config)
    sensors_la = DeclareLaunchArgument('sensors_config', default_value=sensors_config)
    mux_la     = DeclareLaunchArgument('mux_config',     default_value=mux_config)
    use_zed_la = DeclareLaunchArgument(
        'use_zed_perception', default_value='true',
        description='Start the ZED2 camera stream. Set false to run without the camera.')

    # ---- startup sequence parameters -----------------------------------
    startup_delay_la = DeclareLaunchArgument(
        'startup_delay', default_value='5.0',
        description='Seconds to wait before steer sweep starts.')
    right_duration_la = DeclareLaunchArgument(
        'right_duration', default_value='2.0',
        description='Seconds to hold max right steering.')
    left_duration_la = DeclareLaunchArgument(
        'left_duration', default_value='2.0',
        description='Seconds to hold max left steering.')
    center_duration_la = DeclareLaunchArgument(
        'center_duration', default_value='1.0',
        description='Seconds to hold center steering.')
    max_steering_angle_la = DeclareLaunchArgument(
        'max_steering_angle', default_value='0.18',
        description='Max steering angle in radians.')
    startup_command_topic_la = DeclareLaunchArgument(
        'startup_command_topic', default_value='/teleop',
        description='High-priority mux input topic for the steer sweep.')
    mpc_node_name_la = DeclareLaunchArgument(
        'mpc_node_name', default_value='/andre_mpc_controller',
        description='Fully-qualified MPC node name for SetParameters call.')

    ld = LaunchDescription([
        joy_la, vesc_la, sensors_la, mux_la, use_zed_la,
        startup_delay_la, right_duration_la, left_duration_la,
        center_duration_la, max_steering_angle_la,
        startup_command_topic_la, mpc_node_name_la,
    ])

    # ---- nodes ---------------------------------------------------------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')],
    )
    # Raw odometry source: pure bicycle-model integration of the VESC speed +
    # steering (no IMU fusion), publishing /odom. IMU + odometry fusion is now
    # handled by the robot_localization EKF below (which replaces the old
    # in-node Kalman filter). publish_tf is false (vesc.yaml) so the EKF owns
    # the odom -> base_link transform.
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node_backup',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    # robot_localization EKF: fuses /odom (x, y, yaw) with the VESC IMU
    # (/sensors/imu/raw: yaw rate + linear acceleration) and broadcasts the
    # odom -> base_link transform. Replaces the old vesc_to_odom Kalman filter.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')],
        sigterm_timeout='5',
        sigkill_timeout='2',
    )

    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )
    # ZED2 is mounted where the LiDAR used to be: 0.27 m forward, 0.11 m up.
    # zed2_camera_link is the root of the ZED URDF chain (the mounting point),
    # so attaching base_link -> zed2_camera_link places the whole camera tree.
    static_zed2_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_zed2',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'zed2_camera_link']
    )
    # LiDAR is now mounted symmetrically to the ZED2 across the y-z plane
    # (x -> -x), i.e. 0.27 m behind base_link at the same height.
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['-0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    # VESC IMU is rigidly mounted on the chassis; treat it as coincident with
    # base_link. The EKF needs this transform to bring sensors/imu/raw (frame
    # 'imu') into base_link. Adjust the offset if the IMU is calibrated.
    static_imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu']
    )
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
        }]
    )

    # ---- f1tenth_navigation static map server --------------------------
    f1tenth_nav_share = get_package_share_directory('f1tenth_navigation')
    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_nav_share, 'launch', 'map_server_launch.py')
        )
    )

    # ---- ZED2 camera stream --------------------------------------------
    # publish_tf=false prevents the ZED wrapper from publishing the
    # positional-tracking odom->camera_link TF, which would conflict with the
    # EKF's odom->base_link transform. zed2_perception.yaml also sets this, but
    # we repeat it here as an explicit guard at the call site.
    # base_link->zed2_camera_link is already owned by static_zed2_tf_node above;
    # publish_urdf=true (default) is still needed so the ZED's robot_state_publisher
    # can broadcast the internal camera TF subtree (zed2_camera_link->zed2_left_camera_frame, etc.).
    zed_wrapper_share = get_package_share_directory('zed_wrapper')
    perception_share = get_package_share_directory('f1tenth_perception')
    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_share, 'launch', 'zed_camera.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_zed_perception')),
        launch_arguments={
            'camera_model': 'zed2',
            'camera_name': 'zed2',
            'ros_params_override_path': os.path.join(
                perception_share, 'config', 'zed2_perception.yaml'),
            'publish_tf': 'false',
            'publish_map_tf': 'false',
        }.items(),
    )

    # ---- MPC node ------------------------------------------------------
    mpc_node = Node(
        package='mpc_controller',
        executable='andre_mpc_node',
        name='andre_mpc_controller',
        output='screen'
    )

    # ---- startup sequence ----------------------------------------------
    startup_sequence = Node(
        package='f1tenth_bringup',
        executable='stack_startup_sequence',
        name='stack_startup_sequence',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'startup_delay':      LaunchConfiguration('startup_delay'),
            'right_duration':     LaunchConfiguration('right_duration'),
            'left_duration':      LaunchConfiguration('left_duration'),
            'center_duration':    LaunchConfiguration('center_duration'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'command_topic':      LaunchConfiguration('startup_command_topic'),
            'mpc_node_name':      LaunchConfiguration('mpc_node_name'),
        }],
    )

    # ---- finalize ------------------------------------------------------
    # ld.add_action(joy_node)
    # ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(ekf_node)
    ld.add_action(vesc_driver_node)
    # ld.add_action(throttle_interpolator_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_node)
    ld.add_action(static_zed2_tf_node)
    ld.add_action(static_imu_tf_node)
    ld.add_action(map_server)
    ld.add_action(zed_camera)
    ld.add_action(foxglove_bridge_node)
    ld.add_action(startup_sequence)
    # Start MPC only after startup_sequence exits
    
    # shut everything down if vesc_driver or MPC dies
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=vesc_driver_node,
            on_exit=[Shutdown()]
        )
    ))
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=mpc_node,
            on_exit=[Shutdown()]
        )
    ))

    return ld
