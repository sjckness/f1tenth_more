"""Launches gyro_bias_calibration_node standalone. Run with the car stationary and
level; see f1tenth_diagnostics/README.md for how to read and apply the result.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/sensors/imu/raw',
        description='sensor_msgs/Imu topic to sample.')
    sample_duration_arg = DeclareLaunchArgument(
        'sample_duration_sec', default_value='10.0',
        description='How long to collect samples before reporting, in seconds.')
    min_samples_arg = DeclareLaunchArgument(
        'min_samples', default_value='50',
        description='Warn if fewer than this many samples were collected.')

    gyro_bias_calibration_node = Node(
        package='f1tenth_diagnostics',
        executable='gyro_bias_calibration_node',
        name='gyro_bias_calibration_node',
        output='screen',
        parameters=[{
            'imu_topic': LaunchConfiguration('imu_topic'),
            'sample_duration_sec': LaunchConfiguration('sample_duration_sec'),
            'min_samples': LaunchConfiguration('min_samples'),
        }],
    )

    return LaunchDescription([
        imu_topic_arg,
        sample_duration_arg,
        min_samples_arg,
        gyro_bias_calibration_node,
    ])
