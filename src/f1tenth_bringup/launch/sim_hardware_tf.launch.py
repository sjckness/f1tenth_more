"""sim:=isaac stand-in for the 'hardware' component (vesc.launch.py).

The simulator replaces the VESC (driver, ackermann_to_vesc, vesc_to_odom: Isaac Sim consumes
/ackermann_drive and publishes /odom and /sensors/imu/raw itself). The one thing of
vesc.launch.py the rest of the stack still needs is its static base_link -> imu transform:
Isaac stamps its IMU with frame 'imu' (the car's driver leaves the frame empty, which
robot_localization reads as base_link), so without it the EKF silently drops the IMU yaw rate.

Same node name and the same arguments as vesc.launch.py's static_baselink_to_imu
(test_sim_mode.py asserts they stay identical). Started by component_supervisor_node only
in sim:=isaac (apply_sim_mode); never on the car.
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
