"""Full stack bringup.

This file replicates `bringup_launch.py` inline (every node, every remap,
every parameter — including the throttle_interpolator that the original
leaves commented out) and adds:

  * an ackermann_cmd remap fix on ackermann_to_vesc_node (see BUG FIX
    comment below)
  * the f1tenth_navigation map server (static /map for Foxglove)
  * the MPC node (mpc_controller / frenet_mpc_node, registered under the
    runtime name `frenet_mpc_controller` so ros2 param targets line up)
  * the stack_startup_sequence node which:
      - publishes a steer-right / steer-left / center sweep on the
        Ackermann command topic (`/teleop`, the high-priority mux input —
        see bringup_launch.py mux config)
      - then prints:
            [stack_bringup] Start sequence complete. Press ENTER to activate MPC.
      - waits for Enter on stdin
      - calls SetParameters on the MPC node to set v_ref = 0.0

Magic numbers (startup delay, hold durations, max steering angle, etc.)
are exposed as LaunchArguments and forwarded to stack_startup_sequence
as ROS parameters.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- config files from f1tenth_stack (same as bringup_launch.py) ----
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'sensors.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Descriptions for sensor configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')

    # ---- LaunchArguments for the startup-sequence magic numbers --------
    startup_delay_la = DeclareLaunchArgument(
        'startup_delay',
        default_value='5.0',
        description='Seconds to wait after launch before the steer sweep '
                    'starts (lets other nodes finish coming up).')
    right_duration_la = DeclareLaunchArgument(
        'right_duration',
        default_value='2.0',
        description='Seconds to hold steering = max right (speed=0).')
    left_duration_la = DeclareLaunchArgument(
        'left_duration',
        default_value='2.0',
        description='Seconds to hold steering = max left (speed=0).')
    center_duration_la = DeclareLaunchArgument(
        'center_duration',
        default_value='1.0',
        description='Seconds to hold steering = 0 (speed=0).')
    max_steering_angle_la = DeclareLaunchArgument(
        'max_steering_angle',
        default_value='0.18',
        description='Max steering angle in radians (matches MPC max_steer).')
    startup_command_topic_la = DeclareLaunchArgument(
        'startup_command_topic',
        default_value='/teleop',
        description='AckermannDriveStamped topic used by the startup '
                    'sweep. /teleop is the high-priority mux input so it '
                    'preempts the MPC on /drive during the sweep.')
    mpc_node_name_la = DeclareLaunchArgument(
        'mpc_node_name',
        default_value='/frenet_mpc_controller',
        description='Fully-qualified name of the MPC node for the '
                    'post-Enter SetParameters call.')

    ld = LaunchDescription([
        joy_la, vesc_la, sensors_la, mux_la,
        startup_delay_la, right_duration_la, left_duration_la,
        center_duration_la, max_steering_angle_la,
        startup_command_topic_la, mpc_node_name_la,
    ])

    # ---- replicated from bringup_launch.py -----------------------------
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
        # BUG FIX: mux outputs on /ackermann_drive but vesc node expects
        # /ackermann_cmd — remap here to bridge the gap.
        remappings=[
            ('ackermann_cmd', '/ackermann_drive'),
        ],
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
        parameters=[LaunchConfiguration('vesc_config')]
    )
    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')]
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

    # ---- f1tenth_navigation static map server --------------------------
    f1tenth_nav_share = get_package_share_directory('f1tenth_navigation')
    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_nav_share, 'launch', 'map_server_launch.py')
        )
    )

    # ---- MPC node (mpc_controller / frenet_mpc_node) -------------------
    # NOTE: "andre_mpc_node" in the task spec doesn't exist in this
    # workspace; confirmed with the user it's a typo for frenet_mpc_node
    # in the mpc_controller package.
    mpc_node = Node(
        package='mpc_controller',
        executable='frenet_mpc_node',
        name='frenet_mpc_controller',
        output='screen',
    )

    # ---- startup sequence (non-blocking; runs in its own node) ---------
    # TODO: the task spec asks the post-Enter activation call to set
    # "v_ref=0.0 AND angular target=0.0". frenet_mpc_node exposes no
    # angular-target parameter (only v_ref controls speed; lateral
    # behaviour is driven by qn/qalpha/qddelta weights against a baked
    # reference path). stack_startup_sequence sets v_ref=0.0 only.
    # Update stack_startup_sequence.py if/when the MPC gets an explicit
    # angular target parameter or command topic.
    startup_sequence = Node(
        package='f1tenth_stack',
        executable='stack_startup_sequence',
        name='stack_startup_sequence',
        output='screen',
        emulate_tty=True,  # needed so sys.stdin.readline() can read Enter
        parameters=[{
            'startup_delay': LaunchConfiguration('startup_delay'),
            'right_duration': LaunchConfiguration('right_duration'),
            'left_duration': LaunchConfiguration('left_duration'),
            'center_duration': LaunchConfiguration('center_duration'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'command_topic': LaunchConfiguration('startup_command_topic'),
            'mpc_node_name': LaunchConfiguration('mpc_node_name'),
        }],
    )

    # ---- finalize ------------------------------------------------------
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    # ld.add_action(throttle_interpolator_node)
    ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_node)
    ld.add_action(map_server)
    ld.add_action(mpc_node)
    ld.add_action(startup_sequence)

    return ld
