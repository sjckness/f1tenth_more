"""What the default foxglove_bridge whitelists hide, and what they must not.

foxglove_bridge full-matches each topic and service name against the regexes
foxglove_bridge.launch.py passes it (std::regex, ECMAScript, case-insensitive).
The defaults live in stack_params.yaml as denylists written with negative
lookaheads, which read the same under Python's re for these patterns. A live
run of the bridge on the car with the same patterns advertised 101 of 213
topics and none under /zed2 (2026-09-17).

Plain YAML and re, no rclpy.
"""
import os
import re

import yaml

_STACK_PARAMS = os.path.join(
    os.path.dirname(__file__), '..', '..', 'f1tenth_params', 'config', 'stack_params.yaml')


def _pattern(name):
    with open(_STACK_PARAMS) as f:
        return re.compile(yaml.safe_load(f)[name]['default'], re.IGNORECASE)


def _advertised(name, names):
    pattern = _pattern(name)
    return {n for n in names if pattern.fullmatch(n)}


def test_topics_hide_the_zed_tree_and_full_rate_images_only():
    hidden = [
        '/zed2/zed_node/rgb/image_rect_color/compressed',
        '/zed2/zed_node/point_cloud/cloud_registered',
        '/zed2/zed_node/depth/camera_info',
        '/camera/image_raw',
        '/camera/image_annotated',
        '/camera/detection_masks',
    ]
    shown = [
        '/camera/image_raw/viz',
        '/camera/image_annotated/viz',
        '/camera/detections',
        '/costmap/semantic_markers',
        '/mission/status',
        '/mpc/goal_object',
        '/scan',
        '/tf',
        '/tf_static',
        '/slam/map',
        '/rosout',
    ]
    assert _advertised('foxglove_topic_whitelist', hidden + shown) == set(shown)


def test_services_hide_parameter_services_but_keep_mission_and_supervisor():
    hidden = [
        '/mpc_corr/get_parameters',
        '/behavior_executor_node/set_parameters_atomically',
        '/zed2/zed_node/list_parameters',
        '/zed2/zed_node/reset_odometry',
    ]
    shown = [
        '/mission/load_mission',
        '/mission/start_mission',
        '/mission/abort_mission',
        '/mission/emergency_stop',
        '/component_supervisor_node/control_component',
        '/restart_component',
    ]
    assert _advertised('foxglove_service_whitelist', hidden + shown) == set(shown)
