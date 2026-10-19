"""What the default foxglove_bridge whitelists hide, and what they must not.

foxglove_bridge full-matches each topic and service name against the regexes
foxglove_bridge.launch.py passes it (std::regex, ECMAScript, case-insensitive);
a name is advertised when ANY pattern of the list matches. The defaults live in
stack_params.yaml: the service whitelist is one denylist pattern written with
negative lookaheads, the topic whitelist a LIST of allowlist patterns (the /viz
relays plus tf, latched and on-change topics). On 2026-10-01 the bridge on the
car advertised exactly that list's topics (foxglove_bridge log of every
campaign run that day).

Plain YAML and re, no rclpy.
"""
import os
import re

import yaml

_STACK_PARAMS = os.path.join(
    os.path.dirname(__file__), '..', '..', 'f1tenth_params', 'config', 'stack_params.yaml')


def _patterns(name):
    """The default as a list of compiled patterns: a string is a list of one."""
    with open(_STACK_PARAMS) as f:
        default = yaml.safe_load(f)[name]['default']
    if isinstance(default, str):
        default = [default]
    return [re.compile(p, re.IGNORECASE) for p in default]


def _advertised(name, names):
    patterns = _patterns(name)
    return {n for n in names if any(p.fullmatch(n) for p in patterns)}


def test_topics_allow_only_the_viz_relays_and_latched_topics():
    hidden = [
        '/zed2/zed_node/rgb/image_rect_color/compressed',
        '/zed2/zed_node/point_cloud/cloud_registered',
        '/zed2/zed_node/depth/camera_info',
        '/camera/image_raw',
        '/camera/image_annotated',
        '/camera/detection_masks',
        '/camera/image_raw/viz',          # the old throttle copies stay off
        '/camera/image_annotated/viz',
        '/camera/detections',             # raw, full rate: only its /viz copy
        '/scan',
        '/mpc/goal_object',
        '/perception/walls',               # the tracks, not the markers
    ]
    # What the bridge advertised on the car, 2026-10-01 (every campaign run),
    # plus the two wall topics added 2026-10-16 when the walls turned out to be
    # published all along and simply not on the allowlist.
    shown = [
        '/viz/scan',
        '/viz/camera/image_annotated/compressed',
        '/viz/costmap/visualization/compressed',
        '/calibration/in_progress',
        '/costmap/semantic_markers',
        '/joint_states',
        '/perception/walls/markers',       # pass-through: per-id DELETEs
        '/viz/walls',                      # covered by '^/viz/.*'
        '/viz/doors',                      # door markers, covered by '^/viz/.*'
        '/map',
        '/mission/move_outcome',
        '/mission/status',
        '/parameter_events',
        '/robot_description',
        '/rosout',
        '/safety/event',
        '/slam/map',
        '/slam/map_metadata',
        '/tf',
        '/tf_static',
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
