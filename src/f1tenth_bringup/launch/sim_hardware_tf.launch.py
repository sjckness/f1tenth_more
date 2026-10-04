"""
Sim mode: what the 'hardware' component keeps when the VESC is simulated.

vesc.launch.py does two things: run the VESC driver group (skipped in sim, the
simulator publishes /odom and /sensors/imu/raw), and publish the static
base_link -> imu transform. That transform stays exactly as on the car, same
node name and same arguments, so the TF tree and the EKFs' IMU handling do
not change between the car and the simulator. (The VESC driver, and the
simulator, publish /sensors/imu/raw with an empty frame_id, which
robot_localization reads as base_link_frame.)

The node and its arguments are a copy of vesc.launch.py's static_imu_tf_node.
test_sim_mode.py fails if the two drift apart.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_imu',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu'],
        ),
    ])
