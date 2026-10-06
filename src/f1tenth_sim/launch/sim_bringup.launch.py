"""F1TENTH simulation bringup (ROS 2 Jazzy + Gazebo Harmonic).

The sim is a drop-in for the car's drivers (vesc_driver, vesc_to_odom,
urg_node) and nothing else: it publishes what they publish and consumes what
ackermann_to_vesc consumes. Everything else (robot_state_publisher on /tf,
EKF, slam_toolbox, ackermann_mux, MPC) is the stack's job on the Thor
(output/sim_port_report.md §2.2, §5).

Launches:
  1. Gazebo Harmonic with the 2x husarion office world     (husarion_gz_worlds)
  2. sim_robot_state_publisher: private RSP whose only job is to publish the
     sim URDF on /sim/robot_description. Its TF goes to /sim/tf and
     /sim/tf_static, never /tf (§2.3); it reads /sim/joint_states.
  3. ros_gz_bridge: /sim/clock_raw (Gazebo's 1 kHz clock), /scan, /sim/imu_raw,
     /sim/ground_truth, and with
     camera:=true the ZED 2i streams on the car's topic names:
     /camera/image_raw, /camera/camera_info (what camera.launch.py remaps the
     wrapper's RGB to), /zed2/zed_node/depth/depth_registered and
     /zed2/zed_node/depth/camera_info (what detection.launch.py reads).
  4. spawn the robot from /sim/robot_description          (ros_gz_sim create)
  5. once spawned: joint_state_broadcaster, then ackermann_steering_controller
     (the controller_manager runs inside the gz_ros2_control plugin and reads
     the same /sim/robot_description)
  6. drive_bridge: /ackermann_drive -> controller, controller odom -> /odom,
     /sim/imu_raw -> /sensors/imu/raw
  7. optional foxglove_bridge (foxglove:=true)
  8. clock_throttle: /sim/clock_raw -> /clock at clock_rate (default 200 Hz).
     Physics keeps its 1 ms step; only the published clock is decimated, so
     /clock costs 5x fewer packets per Thor subscriber on the cable
     (output/sim_clock_fanout.md).

Published on the shared graph: /clock /scan /odom /sensors/imu/raw, the four
camera topics above (camera:=true), plus /sim/* (internal and ground truth).
Nothing on /tf or /tf_static, and nothing
on /joint_states: on the car joint_state_publisher (static zeros, 10 Hz, kept
by the Thor in sim:=true) owns that topic, so the sim's true joint states go to
/sim/joint_states, like /sim/ground_truth.

Args: gui (default false: server only), world, foxglove, camera (default true:
render the ZED 2i at the car's 640x360 @ 30 Hz, about 50 MB/s raw over the LAN),
clock_rate (Hz of /clock, default 200; 0 = every physics step, the old
behaviour), x/y/yaw spawn pose.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

SIM_DESCRIPTION_TOPIC = '/sim/robot_description'


def generate_launch_description():
    sim_share = get_package_share_directory('f1tenth_sim')
    desc_share = get_package_share_directory('f1tenth_description')
    husarion_share = get_package_share_directory('husarion_gz_worlds')

    bridge_config = os.path.join(sim_share, 'config', 'ros_gz_bridge.yaml')
    camera_bridge_config = os.path.join(sim_share, 'config', 'ros_gz_bridge_camera.yaml')
    controllers_file = os.path.join(sim_share, 'config', 'controllers.yaml')

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    foxglove = LaunchConfiguration('foxglove')
    camera = LaunchConfiguration('camera')
    clock_rate = LaunchConfiguration('clock_rate')

    args = [
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Start the Gazebo GUI. false = server only (sensors '
                        'still render).'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(sim_share, 'worlds', 'husarion_office_2x.sdf'),
            description='SDF world file. Defaults to the husarion office world '
                        'scaled 2x in x/y (worlds/husarion_office_2x.sdf, made by '
                        'tools/scale_office_world.py; models come from '
                        'husarion_gz_worlds); '
                        'pass world:=/abs/path.sdf (e.g. the old '
                        'f1tenth_sim/worlds/empty_room.sdf) for another.'),
        DeclareLaunchArgument(
            'foxglove', default_value='false',
            description='Also start foxglove_bridge on :8765 on this host.'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Render the ZED 2i RGB + depth and bridge them onto the '
                        "car's topics. false = LiDAR/IMU/odom only."),
        DeclareLaunchArgument(
            'clock_rate', default_value='200.0',
            description='Rate of /clock in Hz of sim time. It is the time '
                        'resolution of every sim-time timer on the Thor (fastest '
                        'are 50 Hz): 200 = 5 ms, 100 is the practical minimum. '
                        '0 = forward every physics step (1000 Hz, floods the '
                        'LAN with ~40 Thor subscribers).'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    # Let Gazebo find the world + description meshes if referenced by URI.
    # (Fortress also needed IGN_GAZEBO_RESOURCE_PATH; Harmonic reads only this.)
    gz_env = [
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=p, separator=':')
        for p in (os.path.join(sim_share, 'worlds'), desc_share,
                  os.path.join(desc_share, 'meshes'))
    ]

    # --- 1) Gazebo Harmonic (via husarion_gz_worlds) ------------------------
    # husarion's gz_sim.launch.py wraps ros_gz_sim and knows where its office
    # models live (its env-hook sets GZ_SIM_RESOURCE_PATH). It takes the world
    # on gz_world and runs server-only unless gz_headless_mode is False, so we
    # map our gui arg onto it: gui:=false -> headless_mode True (-s, sensors
    # still render via --headless-rendering).
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(husarion_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_world': world,
            'gz_headless_mode': PythonExpression(
                ["'False' if '", gui, "' == 'true' else 'True'"]),
            'gz_log_level': '3',
        }.items(),
    )

    # --- 2) private robot_state_publisher -----------------------------------
    xacro_file = PathJoinSubstitution([
        FindPackageShare('f1tenth_description'), 'urdf', 'roboracer.urdf.xacro'])
    robot_description = ParameterValue(Command([
        'xacro ', xacro_file,
        ' use_sim:=true',
        ' enable_sensors:=true',
        ' enable_camera_mock:=', camera,
        ' pkg_share:=', desc_share,
        ' control_config:=', controllers_file,
    ]), value_type=str)

    sim_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='sim_robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        remappings=[
            ('/robot_description', SIM_DESCRIPTION_TOPIC),
            ('/tf', '/sim/tf'),
            ('/tf_static', '/sim/tf_static'),
            ('/joint_states', '/sim/joint_states'),
        ],
    )

    # --- 3) ros_gz_bridge ---------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sim_ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sim_ros_gz_camera_bridge',
        output='screen',
        parameters=[{'config_file': camera_bridge_config, 'use_sim_time': True}],
        condition=IfCondition(camera),
    )

    # --- 3b) /clock throttle -----------------------------------------------
    # Wall time on purpose (the node forces use_sim_time false itself): on sim
    # time it would subscribe to the /clock it publishes.
    clock_throttle = Node(
        package='f1tenth_sim',
        executable='clock_throttle',
        name='sim_clock_throttle',
        output='screen',
        parameters=[{
            'rate_hz': ParameterValue(clock_rate, value_type=float),
            'input_topic': '/sim/clock_raw',
            'output_topic': '/clock',
        }],
    )

    # --- 4) spawn the robot -------------------------------------------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='sim_spawn_roboracer',
        output='screen',
        arguments=[
            '-name', 'roboracer',
            '-topic', SIM_DESCRIPTION_TOPIC,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.05',
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # --- 5) ros2_control spawners (controller_manager lives in the gz plugin) -
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-ros-args', '-r /joint_states:=/sim/joint_states',
                   '--controller-ros-args',
                   '-r /dynamic_joint_states:=/sim/dynamic_joint_states'],
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
        parameters=[{'use_sim_time': True, 'wheelbase': 0.3302}],
    )

    # --- 7) foxglove (optional) ---------------------------------------------
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='sim_foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765, 'address': '0.0.0.0', 'use_sim_time': True}],
        condition=IfCondition(foxglove),
    )

    return LaunchDescription([
        *args,
        *gz_env,
        gz_sim,
        sim_rsp,
        bridge,
        clock_throttle,
        camera_bridge,
        spawn_entity,
        drive_bridge,
        foxglove_node,
        # create exits once the model is in the world, and the plugin's
        # controller_manager comes up with it; the spawner then waits for the
        # CM's services on its own.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_entity, on_exit=[jsb_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb_spawner, on_exit=[ackermann_spawner])),
    ])
