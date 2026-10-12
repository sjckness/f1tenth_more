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
     /sim/ground_truth, the two camera_info streams on the car's names
     (/camera/camera_info, /zed2/zed_node/depth/camera_info), and with
     camera:=true the two heavy Image streams on sim-PC-local staging topics
     (/sim/camera/image_raw RGB, /sim/camera/depth_raw depth) -- NOT the
     canonical names, because raw frames do not survive the two-machine link
     (see 3c and ros_gz_bridge_camera.yaml).
  3c. camera compressors (camera:=true): image_transport republishers compress
     the staging Image streams so only the small compressed topics cross the
     LAN: /camera/image_raw/compressed (JPEG) and
     /zed2/zed_node/depth/depth_registered/compressedDepth (PNG). On the Thor
     sim_camera_tf.launch.py decompresses them back to /camera/image_raw and
     /zed2/zed_node/depth/depth_registered, the names camera.launch.py /
     detection.launch.py read on the car.
  4. spawn the robot from /sim/robot_description          (ros_gz_sim create)
  5. once spawned: joint_state_broadcaster, then ackermann_steering_controller
     (the controller_manager runs inside the gz_ros2_control plugin and reads
     the same /sim/robot_description)
  6. drive_bridge: /ackermann_drive -> controller, controller odom -> /odom,
     /sim/imu_raw -> /sensors/imu/raw
  7. optional foxglove_bridge (foxglove:=true)
  8. clock_throttle: /sim/clock_raw -> /clock at clock_rate (default 250 Hz =
     one per 4 ms physics step). Gazebo publishes a clock per step (~250 Hz);
     the throttle decimates on sim time, so /clock costs fewer packets per Thor
     subscriber on the cable (output/sim_clock_fanout.md).

Published on the shared graph: /clock /scan /odom /sensors/imu/raw, the two
camera_info topics and the two compressed camera topics (camera:=true), plus
/sim/* (internal, ground truth, and the local camera staging topics).
Nothing on /tf or /tf_static, and nothing
on /joint_states: on the car joint_state_publisher (static zeros, 10 Hz, kept
by the Thor in sim:=true) owns that topic, so the sim's true joint states go to
/sim/joint_states, like /sim/ground_truth.

Args: gui (default false: server only), world, foxglove, camera (default true:
render the ZED 2i at the car's 640x360 @ 30 Hz, about 50 MB/s raw over the LAN),
clock_rate (Hz of /clock, default 250 = one per 4 ms step; 0 = every physics
step, the old behaviour), x/y/yaw spawn pose.

TEST ROOM (world:=test_room_6x6, worlds/test_room_6x6.sdf, see README.md):
world also takes a bare NAME, resolved to f1tenth_sim/worlds/<name>.sdf.
  car_x / car_y / car_yaw   spawn pose; '' (default) = the world's own
                            default: (-2.5, 0, 0) in test_room_6x6, else x/y/yaw
  person_enabled            auto (default: true in test_room_6x6 only) | true | false
  person_x / person_y       default (0, 0), the room centre; the person always
                            faces the car's spawn point
  obstacle_enabled          false (default) | true
  obstacle_type             plant_0.3 (default) | box_1.0   (f1tenth_sim/models)
  obstacle_x / obstacle_y   default (0, 0)
Ground truth (measuring only, never read by the stack, any world):
/sim/ground_truth/{car,person,obstacle}, PoseStamped in the world frame. The
person and obstacle models publish their own pose (PosePublisher); the car's
comes from the URDF's /sim/ground_truth Odometry via ground_truth_relay.
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

    test_room = PythonExpression(["'", world, "' in ('test_room_6x6', '",
                                  os.path.join(sim_share, 'worlds', 'test_room_6x6.sdf'),
                                  "')"])
    world_path = PythonExpression([
        "'", world, "' if '", world, "'.endswith('.sdf') else '",
        os.path.join(sim_share, 'worlds'), "/' + '", world, "' + '.sdf'"])

    def _car(axis, room_default, fallback):
        return PythonExpression([
            "'", LaunchConfiguration('car_' + axis), "' if '",
            LaunchConfiguration('car_' + axis), "' != '' else ('", room_default,
            "' if ", test_room, " else '", LaunchConfiguration(fallback), "')"])
    car_x = _car('x', '-2.5', 'x')
    car_y = _car('y', '0.0', 'y')
    car_yaw = _car('yaw', '0.0', 'yaw')

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
            'clock_rate', default_value='250.0',
            description='Rate of /clock in Hz of sim time. It is the time '
                        'resolution of every sim-time timer on the Thor (fastest '
                        'are 50 Hz). Default 250 = one per 4 ms physics step '
                        '(the current step); 200 (5 ms bins) would alias the '
                        '4 ms steps and add 4/8 ms jitter. 0 = forward every '
                        'physics step (floods the LAN with ~40 Thor subscribers).'),
        DeclareLaunchArgument('x', default_value='1.0'),    # 1 m ahead (+x world)
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='-1.5708'),  # -pi/2 = 90 deg right
        # --- test room (see the module docstring and README.md) -------------
        DeclareLaunchArgument('car_x', default_value='',
                              description="'' = world default (-2.5 in test_room_6x6, else x)"),
        DeclareLaunchArgument('car_y', default_value='',
                              description="'' = world default (0 in test_room_6x6, else y)"),
        DeclareLaunchArgument('car_yaw', default_value='',
                              description="'' = world default (0 in test_room_6x6, else yaw)"),
        DeclareLaunchArgument('person_enabled', default_value='auto',
                              description='auto = true in test_room_6x6 only'),
        DeclareLaunchArgument('person_x', default_value='0.0'),
        DeclareLaunchArgument('person_y', default_value='0.0'),
        DeclareLaunchArgument('obstacle_enabled', default_value='false'),
        DeclareLaunchArgument('obstacle_type', default_value='plant_0.3',
                              description='plant_0.3 | box_1.0 (f1tenth_sim/models)'),
        DeclareLaunchArgument('obstacle_x', default_value='0.0'),
        DeclareLaunchArgument('obstacle_y', default_value='0.0'),
    ]

    # Let Gazebo find the world + description meshes if referenced by URI.
    # (Fortress also needed IGN_GAZEBO_RESOURCE_PATH; Harmonic reads only this.)
    gz_env = [
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=p, separator=':')
        for p in (os.path.join(sim_share, 'worlds'), os.path.join(sim_share, 'models'),
                  desc_share, os.path.join(desc_share, 'meshes'))
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
            'gz_world': world_path,
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

    # --- 3c) camera compressors -------------------------------------------
    # Raw ZED frames do not survive the two-machine link (see
    # ros_gz_bridge_camera.yaml / output/sim_camera_compression.md): the bridge
    # puts each Image on a sim-PC-local staging topic and these image_transport
    # republishers compress it, so only the small compressed topics cross the
    # cable. The Thor's sim_camera_tf.launch.py decompresses back to the
    # canonical raw names the stack reads. republish takes the transports as
    # PARAMETERS (not positional) and only the fully-qualified in/out topics
    # remap (the 'in'/'out' base-name remaps are ignored).
    rgb_compressor = Node(
        package='image_transport',
        executable='republish',
        name='sim_rgb_compressor',
        output='screen',
        parameters=[{'in_transport': 'raw', 'out_transport': 'compressed',
                     'use_sim_time': True}],
        remappings=[
            ('/in', '/sim/camera/image_raw'),
            ('/out/compressed', '/camera/image_raw/compressed'),
        ],
        condition=IfCondition(camera),
    )
    depth_compressor = Node(
        package='image_transport',
        executable='republish',
        name='sim_depth_compressor',
        output='screen',
        parameters=[{'in_transport': 'raw', 'out_transport': 'compressedDepth',
                     'use_sim_time': True}],
        remappings=[
            ('/in', '/sim/camera/depth_raw'),
            ('/out/compressedDepth',
             '/zed2/zed_node/depth/depth_registered/compressedDepth'),
        ],
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
            '-x', car_x,
            '-y', car_y,
            '-z', '0.05',
            '-Y', car_yaw,
        ],
    )

    # --- 4b) test-room person / obstacle, ground truth ---------------------
    person_enabled = LaunchConfiguration('person_enabled')
    person_x, person_y = LaunchConfiguration('person_x'), LaunchConfiguration('person_y')
    # Facing the car's spawn point. The mesh's front is model -y, so the
    # model yaw is the bearing to the car + pi/2.
    person_yaw = PythonExpression([
        'str(__import__("math").atan2(float(', car_y, ') - float(', person_y,
        '), float(', car_x, ') - float(', person_x, ')) + __import__("math").pi / 2)'])
    spawn_person = Node(
        package='ros_gz_sim',
        executable='create',
        name='sim_spawn_person',
        output='screen',
        arguments=['-name', 'person',
                   '-file', os.path.join(sim_share, 'models', 'person_standing', 'model.sdf'),
                   '-x', person_x, '-y', person_y, '-z', '0.0', '-Y', person_yaw],
        condition=IfCondition(PythonExpression([
            "'", person_enabled, "' == 'true' or ('", person_enabled, "' == 'auto' and ",
            test_room, ')'])),
    )
    spawn_obstacle = Node(
        package='ros_gz_sim',
        executable='create',
        name='sim_spawn_obstacle',
        output='screen',
        arguments=['-name', 'obstacle',
                   '-file', PathJoinSubstitution([
                       FindPackageShare('f1tenth_sim'), 'models',
                       LaunchConfiguration('obstacle_type'), 'model.sdf']),
                   '-x', LaunchConfiguration('obstacle_x'),
                   '-y', LaunchConfiguration('obstacle_y'), '-z', '0.0', '-Y', '0.0'],
        condition=IfCondition(LaunchConfiguration('obstacle_enabled')),
    )
    # The person and obstacle models carry a PosePublisher (/model/<name>/pose).
    # The car's world pose is the URDF's /sim/ground_truth Odometry, which
    # ground_truth_relay republishes as /sim/ground_truth/car.
    ground_truth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sim_ground_truth_bridge',
        output='screen',
        arguments=[f'/model/{m}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'
                   for m in ('person', 'obstacle')],
        remappings=[(f'/model/{m}/pose', f'/sim/ground_truth/{m}')
                    for m in ('person', 'obstacle')],
        parameters=[{'use_sim_time': True}],
    )
    ground_truth_relay = Node(
        package='f1tenth_sim',
        executable='ground_truth_relay',
        name='sim_ground_truth_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
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

    # --- 6b) camera pan (sim side): only when the camera renders ------------
    # The pan position controller drives camera_pan_joint; the bridge adapts the
    # hardware-agnostic topics (/camera_pan/command, /camera_pan/joint_state) to
    # it, the way the real Pico driver will on the car. The stack's
    # camera_pan_controller + camera_pan_tf_node run on the Thor
    # (sim_camera_tf.launch.py), so nothing here owns the pan TF.
    camera_pan_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['camera_pan_position_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controllers_file],
        condition=IfCondition(camera),
    )
    camera_pan_bridge = Node(
        package='f1tenth_sim',
        executable='camera_pan_bridge',
        name='f1tenth_sim_camera_pan_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(camera),
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
        rgb_compressor,
        depth_compressor,
        spawn_entity,
        spawn_person,
        spawn_obstacle,
        ground_truth_bridge,
        ground_truth_relay,
        drive_bridge,
        camera_pan_bridge,
        foxglove_node,
        # create exits once the model is in the world, and the plugin's
        # controller_manager comes up with it; the spawner then waits for the
        # CM's services on its own.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_entity, on_exit=[jsb_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb_spawner, on_exit=[ackermann_spawner])),
        # camera_pan_position_controller after ackermann (camera:=true only; the
        # spawner Node carries its own IfCondition, so this is a no-op otherwise).
        RegisterEventHandler(OnProcessExit(
            target_action=ackermann_spawner, on_exit=[camera_pan_spawner])),
    ])
