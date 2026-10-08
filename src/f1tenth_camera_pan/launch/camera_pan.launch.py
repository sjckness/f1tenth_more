"""Stack-side camera-pan bring-up: the controller + the measured-angle TF node.

Hardware-agnostic. Included by the sim (sim_camera_tf.launch.py) and, behind
camera_pan_enabled, by the car (camera.launch.py). It does NOT start a servo or
the sim joint -- that is the sim pan bridge's / real driver's job. The pivot xyz
is passed in (sim: sim_sensor_mounts.yaml camera_pan_pivot; car: camera.launch.py
param), so the one chain base_link -> camera_pan_base -> zed2_camera_link has a
single per-machine pivot shared by the aim law and the TF.

Replaces the static base_link -> zed2_camera_link transform the including launch
used to publish: that becomes camera_pan_tf_node's job, driven by the measured
angle, so pan=0 is bit-identical to the old static TF.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    pivot_x = LaunchConfiguration('pivot_x_m')
    pivot_y = LaunchConfiguration('pivot_y_m')
    pivot_z = LaunchConfiguration('pivot_z_m')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('mode', default_value='track_heading',
                              description='track_heading | fixed | scan (reserved)'),
        DeclareLaunchArgument('fixed_angle_rad', default_value='0.0'),
        DeclareLaunchArgument('pivot_x_m', default_value='0.12'),   # car default
        DeclareLaunchArgument('pivot_y_m', default_value='0.0'),
        DeclareLaunchArgument('pivot_z_m', default_value='0.15'),   # car default
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    controller = Node(
        package='f1tenth_camera_pan',
        executable='camera_pan_controller_node',
        name='camera_pan_controller_node',
        output='screen',
        parameters=[{
            'mode': mode,
            'fixed_angle_rad': LaunchConfiguration('fixed_angle_rad'),
            'pivot_x_m': pivot_x,
            'pivot_y_m': pivot_y,
            'use_sim_time': use_sim_time,
        }],
    )
    tf_node = Node(
        package='f1tenth_camera_pan',
        executable='camera_pan_tf_node',
        name='camera_pan_tf_node',
        output='screen',
        parameters=[{
            'pivot_x_m': pivot_x,
            'pivot_y_m': pivot_y,
            'pivot_z_m': pivot_z,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([*args, controller, tf_node])
