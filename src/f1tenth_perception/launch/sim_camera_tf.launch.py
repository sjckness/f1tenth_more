"""
Sim mode: the camera TF tree camera.launch.py would have published on the car.

In sim mode the supervisor skips camera.launch.py (there is no ZED; f1tenth_sim
renders the images and publishes them on the same topics). But the images are
stamped zed2_left_camera_frame_optical, and detection_3d / front_clearance /
obstacle_projector look that frame up to base_link, so the tree must exist on
the stack's /tf exactly as on the car. On the car it comes from two places,
reproduced here:

  * the static base_link -> zed2_camera_link, same node NAME as
    camera.launch.py's but with the SIM mount pose, read from
    f1tenth_description/config/sim_sensor_mounts.yaml (the single source of
    truth, same numbers sensors.xacro gives the simulated camera). It is
    deliberately NOT the car's mount (camera.launch.py keeps 0.12 0 0.15);
    test_sim_mode.py checks this file uses the yaml and the car keeps its own;
  * the ZED wrapper's robot_state_publisher for the camera's internal frames
    (zed2_camera_link -> zed2_camera_center -> zed2_left/right_camera_frame ->
    *_frame_optical): same node (namespace zed2, name zed2_state_publisher),
    same xacro (zed_wrapper/urdf/zed_descr.urdf.xacro) with the same
    camera_name / camera_model that camera.launch.py passes to the wrapper, and
    the same robot_description remap (zed_camera.launch.py).

f1tenth_sim's URDF (sensors.xacro) mounts its gz camera on the same chain, so
the pixels and the TF agree. use_sim_time comes from sim_component.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import yaml

# Must equal camera.launch.py's zed_camera.launch.py arguments.
CAMERA_NAME = 'zed2'
CAMERA_MODEL = 'zed2i'


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'), 'urdf', 'zed_descr.urdf.xacro')

    # Single source of truth for the SIM camera mount (sensors.xacro reads the
    # same file; the car's camera.launch.py keeps its own 0.12 0 0.15).
    mounts = yaml.safe_load(open(os.path.join(
        get_package_share_directory('f1tenth_description'),
        'config', 'sim_sensor_mounts.yaml')))
    zed_xyz = [str(v) for v in mounts['zed2_camera_link']['xyz']]

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_zed2',
            arguments=[*zed_xyz, '0.0', '0.0', '0.0',
                       'base_link', 'zed2_camera_link'],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=CAMERA_NAME,
            name=CAMERA_NAME + '_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command([
                    'xacro ', xacro_path,
                    ' camera_name:=', CAMERA_NAME,
                    ' camera_model:=', CAMERA_MODEL,
                ]), value_type=str),
            }],
            remappings=[('robot_description', CAMERA_NAME + '_description')],
        ),
    ])
