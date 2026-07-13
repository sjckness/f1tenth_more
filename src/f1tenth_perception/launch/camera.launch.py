"""Camera bringup: exactly one of {ZED2 wrapper, v4l2 webcam}, selected by
camera_source. Both publish onto the canonical /camera/image_raw +
/camera/camera_info so downstream nodes (detection.launch.py) are agnostic
to which camera is active.

The ZED2's static base_link->zed2_camera_link TF lives here too (rather than
in f1tenth_description) since it shares the is_zed condition with the camera
group itself -- keeping them together avoids duplicating that condition
across packages.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    camera_source_la = DeclareLaunchArgument(
        'camera_source', default_value='zed',
        description="Camera source: 'zed' (ZED2 wrapper) or 'webcam' (v4l2 UVC "
                    'on /dev/video0). Selects exactly one; the other is not '
                    'launched at all.')

    is_webcam = IfCondition(
        PythonExpression(["'", LaunchConfiguration('camera_source'), "' == 'webcam'"]))
    is_zed = IfCondition(
        PythonExpression(["'", LaunchConfiguration('camera_source'), "' == 'zed'"]))

    # ZED2 is mounted where the LiDAR used to be: 0.27 m forward, 0.11 m up.
    # zed2_camera_link is the root of the ZED URDF chain (the mounting point),
    # so attaching base_link -> zed2_camera_link places the whole camera tree.
    static_zed2_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_zed2',
        condition=is_zed,
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'zed2_camera_link'],
    )

    # publish_tf=false prevents the ZED wrapper from publishing the
    # positional-tracking odom->camera_link TF, which would conflict with the
    # EKF's odom->base_link transform (f1tenth_localization). zed2_perception.yaml
    # also sets this, but we repeat it here as an explicit guard at the call site.
    # base_link->zed2_camera_link is already owned by static_zed2_tf_node above;
    # publish_urdf=true (default) is still needed so the ZED's robot_state_publisher
    # can broadcast the internal camera TF subtree (zed2_camera_link->zed2_left_camera_frame, etc.).
    zed_wrapper_share = get_package_share_directory('zed_wrapper')
    perception_share = get_package_share_directory('f1tenth_perception')
    # Wrapped in a GroupAction with SetRemap so the wrapper's raw RGB image +
    # camera_info land on the canonical /camera/image_raw + /camera/camera_info
    # (downstream nodes stay agnostic). The whole group is gated by is_zed, so
    # in webcam mode NONE of the ZED nodes / TF publishers / diagnostics start.
    zed_camera = GroupAction(
        condition=is_zed,
        actions=[
            SetRemap(src='/zed2/zed_node/rgb/image_rect_color',
                     dst='/camera/image_raw'),
            SetRemap(src='/zed2/zed_node/rgb/camera_info',
                     dst='/camera/camera_info'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(zed_wrapper_share, 'launch', 'zed_camera.launch.py')
                ),
                launch_arguments={
                    'camera_model': 'zed2',
                    'camera_name': 'zed2',
                    'ros_params_override_path': os.path.join(
                        perception_share, 'config', 'zed2_perception.yaml'),
                    'publish_tf': 'false',
                    'publish_map_tf': 'false',
                }.items(),
            ),
        ],
    )

    # Only when camera_source == 'webcam'. Logitech/UVC on /dev/video0, MJPG
    # 1280x720@30, decoded to rgb8. Remapped so it publishes the same canonical
    # topics as the ZED path. This is the ONLY node started in webcam mode.
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        condition=is_webcam,
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [1280, 720],
            'pixel_format': 'YUYV',   # was MJPG
            'camera_frame_id': 'camera_link',
            'output_encoding': 'rgb8',
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
    )

    return LaunchDescription([
        camera_source_la,
        static_zed2_tf_node,
        zed_camera,
        v4l2_camera_node,
    ])
