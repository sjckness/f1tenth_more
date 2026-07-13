"""Reactive safety-stop supervisor.

Watches Detection3DArray and zeroes the MPC's own v_ref parameter (via
SetParameters on `mpc_node_name`) when an obstacle enters a fixed corridor,
restoring it once clear. Whether this gets launched at all (opt-in per run)
is stack_bringup's decision (`enable_safety_stop`); this file only owns the
corridor/timing parameters once it's been included.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mpc_node_name_la = DeclareLaunchArgument(
        'mpc_node_name', default_value='/andre_mpc_controller',
        description='Fully-qualified MPC node name for SetParameters call.')
    forward_v_ref_la = DeclareLaunchArgument(
        'safety_forward_v_ref', default_value='0.5',
        description='v_ref [m/s] the safety-stop node restores on the MPC '
                    'once the stop corridor is confirmed clear.')
    stop_distance_la = DeclareLaunchArgument(
        'safety_stop_distance', default_value='1.0',
        description='Forward (x) extent [m] of the safety-stop corridor, in '
                    "detection_3d_node's output_frame (zed2_left_camera_frame).")
    corridor_half_width_la = DeclareLaunchArgument(
        'safety_corridor_half_width', default_value='0.25',
        description='Lateral (y) half-width [m] of the safety-stop corridor.')
    corridor_half_height_la = DeclareLaunchArgument(
        'safety_corridor_half_height', default_value='0.25',
        description='Vertical (z) half-height [m] of the safety-stop corridor.')
    clear_frames_required_la = DeclareLaunchArgument(
        'safety_clear_frames_required', default_value='3',
        description='Consecutive clear Detection3DArray frames required '
                    'before the safety-stop node restores v_ref.')

    safety_stop_controller_node = Node(
        package='safety_stop_controller',
        executable='simple_stop_controller_node',
        name='simple_stop_controller_node',
        output='screen',
        parameters=[{
            'detections_topic': '/camera/detections_3d',
            'mpc_node_name': LaunchConfiguration('mpc_node_name'),
            'forward_v_ref': LaunchConfiguration('safety_forward_v_ref'),
            'stop_distance': LaunchConfiguration('safety_stop_distance'),
            'corridor_half_width': LaunchConfiguration('safety_corridor_half_width'),
            'corridor_half_height': LaunchConfiguration('safety_corridor_half_height'),
            'clear_frames_required': LaunchConfiguration('safety_clear_frames_required'),
        }],
    )

    return LaunchDescription([
        mpc_node_name_la, forward_v_ref_la, stop_distance_la,
        corridor_half_width_la, corridor_half_height_la, clear_frames_required_la,
        safety_stop_controller_node,
    ])
