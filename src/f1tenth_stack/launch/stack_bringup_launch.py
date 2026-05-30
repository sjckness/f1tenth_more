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
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- config files --------------------------------------------------
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config', 'joy_teleop.yaml')
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config', 'vesc.yaml')
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config', 'sensors.yaml')
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config', 'mux.yaml')

    joy_la     = DeclareLaunchArgument('joy_config',     default_value=joy_teleop_config)
    vesc_la    = DeclareLaunchArgument('vesc_config',    default_value=vesc_config)
    sensors_la = DeclareLaunchArgument('sensors_config', default_value=sensors_config)
    mux_la     = DeclareLaunchArgument('mux_config',     default_value=mux_config)

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
        joy_la, vesc_la, sensors_la, mux_la,
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
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')],
        sigterm_timeout='5',
        sigkill_timeout='2',
    )
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('sensors_config')]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
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

    # ---- MPC node ------------------------------------------------------
    mpc_node = Node(
        package='mpc_controller',
        executable='andre_mpc_node',
        name='andre_mpc_controller',
        output='screen'
    )

    # ---- startup sequence ----------------------------------------------
    startup_sequence = Node(
        package='f1tenth_stack',
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
    ld.add_action(vesc_driver_node)
    # ld.add_action(throttle_interpolator_node)
    ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_node)
    ld.add_action(map_server)
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
