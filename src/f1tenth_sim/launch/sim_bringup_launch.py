"""F1TENTH simulation bringup (Gazebo Fortress / ignition).

Sim-only counterpart to f1tenth_bringup. Launches, in order:
  1. Gazebo Fortress with worlds/empty_room.sdf       (ros_ign_gazebo)
  2. robot_state_publisher with the roboracer xacro    (f1tenth_description)
  3. ros_ign_bridge parameter_bridge (config/ros_gz_bridge.yaml)
  4. spawn the robot into Gazebo                        (ros_ign_gazebo create)
  5. ros2_control spawners: joint_state_broadcaster + ackermann_steering_controller
  6. drive_bridge: /drive (AckermannDriveStamped) -> controller; controller odom -> /odom
  7. foxglove_bridge (port 8765)
  8. slam_toolbox, async mapping (reuses f1tenth_bringup/config/f1tenth_online_async.yaml)
  9. robot_localization EKF (reuses f1tenth_bringup/config/ekf.yaml verbatim)

It deliberately does NOT launch urg_node, the VESC driver, the ZED SDK, or any
other real-hardware node.

ros_ign_bridge note: this uses the YAML config_file feature. The equivalent
guaranteed-CLI form (if your bridge build lacks config_file support) is:
  parameter_bridge \
    /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock \
    /scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
    /sensors/imu/raw@sensor_msgs/msg/Imu[ignition.msgs.IMU \
    /zed2/zed_node/rgb/image_rect_color@sensor_msgs/msg/Image[ignition.msgs.Image \
    /zed2/zed_node/rgb/image_rect_color/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo \
    /zed2/zed_node/depth/depth_registered@sensor_msgs/msg/Image[ignition.msgs.Image
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = get_package_share_directory('f1tenth_sim')
    desc_share = get_package_share_directory('f1tenth_description')
    bringup_share = get_package_share_directory('f1tenth_bringup')
    ros_ign_gazebo_share = get_package_share_directory('ros_ign_gazebo')

    world_path = os.path.join(sim_share, 'worlds', 'empty_room.sdf')
    bridge_config = os.path.join(sim_share, 'config', 'ros_gz_bridge.yaml')
    controllers_file = os.path.join(sim_share, 'config', 'controllers.yaml')
    # Reused verbatim from the real stack (only use_sim_time is overridden).
    ekf_config = os.path.join(bringup_share, 'config', 'ekf.yaml')
    slam_config = os.path.join(bringup_share, 'config', 'f1tenth_online_async.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the Gazebo sim clock (should stay true for sim).')

    # Let Gazebo find the world + (description) meshes if referenced by URI.
    gz_resource_paths = [
        os.path.join(sim_share, 'worlds'),
        desc_share,
        os.path.join(desc_share, 'meshes'),
    ]
    gz_env = [
        AppendEnvironmentVariable(name=var, value=p, separator=':')
        for var in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH')
        for p in gz_resource_paths
    ]

    # --- 1) Gazebo Fortress -------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_ign_gazebo_share, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': f'-r -v 4 {world_path}'}.items(),
    )

    # --- 2) robot_state_publisher (RSP-only description launch) -------------
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'description_launch.py')),
        launch_arguments={
            'use_sim': 'true',
            'use_sim_time': use_sim_time,
            'enable_sensors': 'true',
            'control_config': controllers_file,
        }.items(),
    )

    # Same description string used to spawn the entity (deterministic; no
    # dependence on the /robot_description topic being up at spawn time).
    xacro_file = PathJoinSubstitution([
        FindPackageShare('f1tenth_description'), 'urdf', 'roboracer.urdf.xacro'])
    robot_description = ParameterValue(Command([
        'xacro ', xacro_file,
        ' use_sim:=true',
        ' enable_sensors:=true',
        ' pkg_share:=', desc_share,
        ' control_config:=', controllers_file,
    ]), value_type=str)

    # --- 3) ros_ign_bridge --------------------------------------------------
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ros_ign_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
    )

    # --- 4) spawn the robot -------------------------------------------------
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_roboracer',
        output='screen',
        arguments=[
            '-name', 'roboracer',
            '-string', robot_description,
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
    )

    # --- 5) ros2_control spawners (controller_manager lives in the gz plugin) -
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
    )
    ackermann_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['ackermann_steering_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controllers_file],
    )

    # --- 6) drive bridge ----------------------------------------------------
    drive_bridge = Node(
        package='f1tenth_sim',
        executable='drive_bridge',
        name='f1tenth_sim_drive_bridge',
        output='screen',
        parameters=[{'use_sim_time': True, 'wheelbase': 0.325}],
    )

    # --- 7) foxglove --------------------------------------------------------
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765, 'address': '0.0.0.0', 'use_sim_time': True}],
    )

    # --- 8) slam_toolbox (async mapping) -----------------------------------
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': True}],
    )

    # --- 9) robot_localization EKF -----------------------------------------
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        *gz_env,
        gz_sim,
        description,
        bridge,
        drive_bridge,
        foxglove,
        ekf,
        slam,
        # Spawn the robot a few seconds after Gazebo is up...
        TimerAction(period=4.0, actions=[spawn_entity]),
        # ...then bring up the controllers once the gz controller_manager exists.
        TimerAction(period=8.0, actions=[jsb_spawner, ackermann_spawner]),
    ])
