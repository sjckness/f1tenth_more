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

This file also runs the camera DECOMPRESSORS. f1tenth_sim renders the ZED on
linus but raw frames do not survive the two-machine Discovery-Server link, so
sim_bringup.launch.py compresses them and only the small compressed topics
cross the LAN (output/sim_camera_compression.md). Here, on the Thor, two
image_transport republishers turn them back into the canonical raw topics the
stack reads on the car, so detection.launch.py / yolo_detector_node are
unchanged:
    /camera/image_raw/compressed (JPEG) -> /camera/image_raw
    .../depth/depth_registered/compressedDepth (PNG)
                                        -> /zed2/zed_node/depth/depth_registered
republish takes the transports as PARAMETERS (not positional), and only the
fully-qualified in/out topics remap (the 'in'/'out' base-name remaps are
ignored). The round-trip preserves each message's header (frame + stamp).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import yaml

# Must equal camera.launch.py's zed_camera.launch.py arguments.
CAMERA_NAME = 'zed2'
CAMERA_MODEL = 'zed2i'


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'), 'urdf', 'zed_descr.urdf.xacro')

    # Single source of truth for the SIM camera mount / pan pivot (sensors.xacro
    # reads the same file; the car's camera.launch.py keeps its own 0.12 0 0.15).
    mounts = yaml.safe_load(open(os.path.join(
        get_package_share_directory('f1tenth_description'),
        'config', 'sim_sensor_mounts.yaml')))
    pivot = [str(v) for v in mounts['camera_pan_pivot']['xyz']]

    camera_pan_share = get_package_share_directory('f1tenth_camera_pan')

    return LaunchDescription([
        # Pass-throughs so a test can toggle the pan behaviour without editing.
        DeclareLaunchArgument('camera_pan_mode', default_value='track_heading'),
        DeclareLaunchArgument('camera_pan_track_when_stopped', default_value='false'),
        # base_link -> camera_pan_base -> zed2_camera_link from the MEASURED pan
        # angle (replaces the old static base_link->zed2_camera_link). The pivot
        # is the sim mount, so pan=0 is the old transform exactly.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(camera_pan_share, 'launch', 'camera_pan.launch.py')),
            launch_arguments={
                'mode': LaunchConfiguration('camera_pan_mode'),
                'track_when_stopped': LaunchConfiguration('camera_pan_track_when_stopped'),
                'pivot_x_m': pivot[0],
                'pivot_y_m': pivot[1],
                'pivot_z_m': pivot[2],
                'use_sim_time': 'true',
            }.items(),
        ),
        # Decompressors: compressed topics off the LAN -> canonical raw topics.
        Node(
            package='image_transport',
            executable='republish',
            name='sim_rgb_decompressor',
            output='screen',
            parameters=[{'in_transport': 'compressed', 'out_transport': 'raw'}],
            remappings=[
                ('/in/compressed', '/camera/image_raw/compressed'),
                ('/out', '/camera/image_raw'),
            ],
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='sim_depth_decompressor',
            output='screen',
            parameters=[{'in_transport': 'compressedDepth', 'out_transport': 'raw'}],
            remappings=[
                ('/in/compressedDepth',
                 '/zed2/zed_node/depth/depth_registered/compressedDepth'),
                ('/out', '/zed2/zed_node/depth/depth_registered'),
            ],
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
