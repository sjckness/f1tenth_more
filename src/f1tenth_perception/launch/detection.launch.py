"""YOLO 2D detector + 2D-to-3D detection fusion.

Source-agnostic: subscribes to the canonical /camera/image_raw (published by
whichever camera stack_bringup brought up) and publishes vision_msgs
Detection2DArray/Detection3DArray + a MarkerArray for Foxglove/RViz.

detection_3d_node additionally needs depth, which only exists in ZED mode, so
it's gated on `camera_source == 'zed'` -- same condition stack_bringup uses
for the ZED camera group itself.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # src/ dir: src/f1tenth_perception/launch/<this file> -> ../.. -> src/.
    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.realpath(__file__))))

    camera_source_la = DeclareLaunchArgument(
        'camera_source', default_value='zed',
        description="Camera source in use: 'zed' or 'webcam'. Only gates "
                    'detection_3d_node (needs ZED depth); yolo_detector_node '
                    'runs regardless.')
    confidence_threshold_la = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.3',
        description='Minimum YOLO detection score kept by detection_3d_node '
                    'before publishing to Detection3DArray/MarkerArray.')
    yolo_device_la = DeclareLaunchArgument(
        'yolo_device', default_value='cuda',
        description="Torch device yolo_detector_node runs inference on: "
                    "'cuda' (default, requires a JetPack/CUDA-matched torch "
                    "build) or 'cpu'. Node raises at startup rather than "
                    "silently falling back to CPU if 'cuda' is requested but "
                    'unavailable.')
    yolo_model_la = DeclareLaunchArgument(
        'yolo_model', default_value='yolo26s.engine',
        description='Filename (under f1tenth_perception/models/) of the YOLO '
                    'weights to load: a *.engine TensorRT build (default -- '
                    "this exact host's GPU-accelerated path, NOT portable to "
                    'other machines/JetPack versions -- rebuild per host with '
                    '`yolo export model=yolo26s.pt format=onnx device=cpu` + '
                    '`trtexec --onnx=... --saveEngine=... --fp16`) or a *.pt '
                    "checkpoint (portable, runs via torch; pair with "
                    "yolo_device:=cpu on hosts without a working CUDA torch).")

    is_zed = IfCondition(
        PythonExpression(["'", LaunchConfiguration('camera_source'), "' == 'zed'"]))

    yolo_detector_node = Node(
        package='f1tenth_perception',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        parameters=[{
            'image_topic': '/camera/image_raw',
            'detections_topic': '/camera/detections',
            'annotated_topic': '/camera/image_annotated',
            'model_path': PathJoinSubstitution([
                src_dir, 'f1tenth_perception', 'models',
                LaunchConfiguration('yolo_model')]),
            'device': LaunchConfiguration('yolo_device'),
        }],
    )

    detection_3d_node = Node(
        package='f1tenth_perception',
        executable='detection_3d_node',
        name='detection_3d_node',
        output='screen',
        condition=is_zed,
        parameters=[{
            'detections_topic': '/camera/detections',
            'depth_topic': '/zed2/zed_node/depth/depth_registered',
            'depth_info_topic': '/zed2/zed_node/depth/camera_info',
            'detections_3d_topic': '/camera/detections_3d',
            'markers_topic': '/camera/detection_markers',
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
        }],
    )

    return LaunchDescription([
        camera_source_la, confidence_threshold_la, yolo_device_la, yolo_model_la,
        yolo_detector_node, detection_3d_node,
    ])
