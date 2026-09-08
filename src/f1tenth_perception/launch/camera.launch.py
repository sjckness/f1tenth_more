"""Camera bringup: exactly one of {ZED2 wrapper, v4l2 webcam}, selected by
camera_source (one of the 6 stack-wide branching args -- see f1tenth_params/config/
stack_params.yaml -- a plain Python value here, not a DeclareLaunchArgument; passing
camera_source:=... on the CLI to this file no longer does anything). Both publish
onto the canonical /camera/image_raw + /camera/camera_info so downstream nodes
(detection.launch.py) are agnostic to which camera is active.

The ZED2's static base_link->zed2_camera_link TF lives here too (rather than in
f1tenth_description) since it shares the is_zed branch with the camera group itself
-- keeping them together avoids duplicating that condition across packages.
"""

import os

from ament_index_python.packages import get_package_share_directory

from f1tenth_params.param_defaults import get_value

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    camera_source = get_value('camera_source')
    is_zed = camera_source == 'zed'
    is_webcam = camera_source == 'webcam'

    actions = []

    if is_zed:
        # Phase 9 layout: base_link is centered between the axles, 0.07m above the
        # ground. ZED2 sits 0.12m ahead of base_link, 0.15m higher, facing forward
        # (yaw=0). Mirrored by f1tenth_description/launch/description.launch.py's
        # base_link->laser TF (0.12m behind instead of ahead, same 0.15m higher,
        # rear-facing -- yaw = this yaw + pi). zed2_camera_link is the root of the
        # ZED URDF chain (the mounting point), so attaching base_link ->
        # zed2_camera_link places the whole camera tree. Positional args are x y z
        # YAW PITCH ROLL (verified empirically -- NOT roll pitch yaw).
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_zed2',
            arguments=['0.12', '0.0', '0.15', '0.0', '0.0', '0.0',
                       'base_link', 'zed2_camera_link'],
        ))

        # publish_tf=false prevents the ZED wrapper from publishing the
        # positional-tracking odom->camera_link TF, which would conflict with the
        # EKF's odom->base_link transform (f1tenth_localization). zed2_perception.yaml
        # also sets this, but we repeat it here as an explicit guard at the call site.
        #
        # publish_imu_tf=false is NOT redundant with zed2_perception.yaml's own
        # sensors.publish_imu_tf: false -- it is the only thing that actually
        # takes effect. zed_camera.launch.py appends a launch-argument dict
        # AFTER ros_params_override_path ("Launch arguments must override the
        # YAML files values"), and that dict contains
        # 'sensors.publish_imu_tf': publish_imu_tf, whose DeclareLaunchArgument
        # default is 'true'. So the yaml key was being silently overwritten on
        # every run. Measured in the archives: zed2_left_camera_frame ->
        # zed2_imu_link was broadcast dynamically at 178.9 Hz, 845 of the 1301
        # /tf messages in run 2026-09-08T11-51-59 -- 65% of all /tf traffic, on
        # a frame that is a LEAF (never a parent in any of 25 archived bags,
        # and not modelled in the ZED URDF at all, so nothing can attach to it).
        # In the two runs of the same session where the ZED was not publishing,
        # both EKF edges sat at exactly 50.00 Hz; in this one they measured
        # 49.12 (odom->base_link) and 37.87 (map->odom).
        #
        # The wrapper's own DeclareLaunchArgument description claims this is
        # "Ignored if publish_tf is False" -- that is wrong, and the archives
        # are what prove it: publish_tf has been false here all along and the
        # IMU TF was published anyway. zed_camera_component.cpp's
        # publishSensorsData() gates the broadcast on mPublishImuTF alone, with
        # no reference to mPublishTf. (It also comments the send as "static TF"
        # while calling the dynamic mTfBroadcaster with a fresh stamp each
        # time, which is why this frame never appeared on /tf_static.)
        # base_link->zed2_camera_link is already owned by the static TF above;
        # publish_urdf=true (default) is still needed so the ZED's robot_state_publisher
        # can broadcast the internal camera TF subtree (zed2_camera_link->zed2_left_camera_frame, etc.).
        zed_wrapper_share = get_package_share_directory('zed_wrapper')
        perception_share = get_package_share_directory('f1tenth_perception')
        # Wrapped in a GroupAction with SetRemap so the wrapper's raw RGB image +
        # camera_info land on the canonical /camera/image_raw + /camera/camera_info
        # (downstream nodes stay agnostic).
        actions.append(GroupAction(
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
                        'publish_imu_tf': 'false',
                    }.items(),
                ),
            ],
        ))

    if is_webcam:
        # Logitech/UVC on /dev/video0, MJPG 1280x720@30, decoded to rgb8. Remapped so
        # it publishes the same canonical topics as the ZED path. This is the ONLY
        # node started in webcam mode.
        actions.append(Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
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
        ))

    return LaunchDescription(actions)
