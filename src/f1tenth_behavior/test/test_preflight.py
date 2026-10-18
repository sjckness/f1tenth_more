"""mission/preflight.py tests -- required_dependencies()/check_liveness() are
plain functions with zero rclpy dependency, same "pure logic, static/
synthetic only" convention test_condition_eval.py already establishes.

Run standalone: python3 -m pytest test/test_preflight.py -v
"""

from pathlib import Path

import pytest

from f1tenth_behavior.mission.mission_config import load_mission_file, parse_mission
from f1tenth_params.param_defaults import get_value, get_wall_distance_topic
from f1tenth_behavior.mission.preflight import (
    PREFLIGHT_BLACKBOARD_KEYS,
    check_liveness,
    required_dependencies,
)
from f1tenth_behavior.mission.runtime import (
    CURRENT_XY_KEY,
    FRONT_CLEARANCE_KEY,
    GLOBAL_XY_KEY,
    MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
)

MISSIONS = Path(__file__).resolve().parent.parent / 'missions'


def _mission(moves):
    return parse_mission({'mission_id': 'test', 'moves': moves})


def _distance_move(move_id='m1', stop_type='distance_reached', stop_params=None, on_object=None):
    return {
        'id': move_id,
        'goal_distance': 1.0,
        'stop_condition': {'type': stop_type, **(stop_params or {})},
        'on_object': on_object or [],
    }


def _names(reqs):
    return {r.name for r in reqs}


def _nodes(reqs):
    return {r.node_name for r in reqs if r.node_name}


class TestRequiredDependencies:

    def test_always_requires_mpc_corr_ackermann_and_localization(self):
        config = _mission([_distance_move()])
        reqs = required_dependencies(config)
        names = _names(reqs)
        assert 'mpc_corr' in names
        assert 'ackermann_to_vesc_node' in names
        assert 'localization (/odom)' in names

    def test_plain_distance_mission_does_not_require_perception_or_costmap(self):
        config = _mission([_distance_move()])
        nodes = _nodes(required_dependencies(config))
        assert not nodes & {'costmap_boundary_node', 'lidar_front_wall_node',
                            'front_clearance_node', 'yolo_detector_node'}

    def test_front_clearance_requires_the_publisher_of_the_wall_topic(self):
        """The node behind get_wall_distance_topic(), not costmap_boundary_node.

        Since 2026-10-15 that is lidar_front_wall_node WHATEVER the scene
        labels switch says: the camera-only /perception/front_distance is
        retired as a wall source (its ROI reads the floor on an open
        corridor). With the switch off nothing publishes a wall distance, so
        this requirement simply fails the preflight -- a mission that stops
        on a wall must not run with the wall pipeline switched off.
        """
        config = _mission([_distance_move(
            stop_type='front_clearance', stop_params={'distance': 1.0})])
        on = _nodes(required_dependencies(config, scene_labels=True))
        off = _nodes(required_dependencies(config, scene_labels=False))
        assert 'lidar_front_wall_node' in on and 'front_clearance_node' not in on
        assert 'lidar_front_wall_node' in off and 'front_clearance_node' not in off
        assert 'costmap_boundary_node' not in on | off

    def test_the_default_follows_enable_scene_labels(self):
        config = _mission([_distance_move(
            stop_type='front_clearance', stop_params={'distance': 1.0})])
        expected = 'lidar_front_wall_node'      # whatever the switch says
        assert expected in _nodes(required_dependencies(config))
        assert any(get_wall_distance_topic() in name
                   for name in _names(required_dependencies(config)))

    def test_front_clearance_as_resume_condition_also_requires_it(self):
        move = _distance_move(
            stop_type='time_elapsed', stop_params={'duration_sec': 5.0},
            on_object=[{
                'class': 'person', 'action': 'stop_and_hold',
                'resume_condition': {'type': 'front_clearance', 'distance': 2.0},
            }],
        )
        names = _names(required_dependencies(_mission([move])))
        assert 'lidar_front_wall_node' in _nodes(
            required_dependencies(_mission([move]), scene_labels=True))
        # on_object entries always need perception regardless of the resume
        # condition's own type -- ObjectSeen has to match the class first.
        assert 'yolo_detector_node' in names

    def test_object_seen_stop_condition_requires_yolo_detector(self):
        config = _mission([_distance_move(stop_type='object_seen', stop_params={'class': 'bottle'})])
        names = _names(required_dependencies(config))
        assert 'yolo_detector_node' in names

    def test_obstacle_distance_below_requires_mpc_corr_min_obstacle_distance_data(self):
        config = _mission([
            _distance_move(stop_type='obstacle_distance_below', stop_params={'distance': 0.5})
        ])
        names = _names(required_dependencies(config))
        assert 'mpc_corr (/mpc/min_obstacle_distance)' in names

    def test_every_blackboard_key_named_is_one_the_loader_can_read(self):
        """MissionLoader registers READ access to PREFLIGHT_BLACKBOARD_KEYS and
        nothing else, so any other key raises inside start_mission. One mission
        per branch that names a key; go_to_object from a file, since its gap
        is resolved against stack params at load."""
        configs = [
            _mission([_distance_move()]),
            _mission([_distance_move(stop_type='front_clearance', stop_params={'distance': 1.0})]),
            _mission([_distance_move(
                stop_type='obstacle_distance_below', stop_params={'distance': 0.5})]),
            _mission([_distance_move(
                stop_type='obstacle_distance_below',
                stop_params={'distance': 0.5, 'forward_only': True})]),
            load_mission_file(str(MISSIONS / 'go_to_person_floor.json')),
        ]
        keys = {r.blackboard_key for c in configs for r in required_dependencies(c)} - {None}
        assert keys <= set(PREFLIGHT_BLACKBOARD_KEYS)
        # The two a hand-kept list in loader.py had missed.
        assert {GLOBAL_XY_KEY, MIN_OBSTACLE_DISTANCE_FORWARD_KEY} <= keys


class TestCheckLiveness:

    def test_all_satisfied_returns_no_failures(self):
        config = _mission([_distance_move()])
        reqs = required_dependencies(config)
        live = {'mpc_corr', 'ackermann_to_vesc_node'}
        data = {CURRENT_XY_KEY: (0.0, 0.0)}
        failures = check_liveness(reqs, list(live), lambda k: data.get(k))
        assert failures == []

    def test_missing_node_is_reported_by_name_and_reason(self):
        config = _mission([_distance_move()])
        reqs = required_dependencies(config)
        # ackermann_to_vesc_node missing -- the exact historical race.
        live = {'mpc_corr'}
        data = {CURRENT_XY_KEY: (0.0, 0.0)}
        failures = check_liveness(reqs, list(live), lambda k: data.get(k))
        assert len(failures) == 1
        assert 'ackermann_to_vesc_node' in failures[0]
        assert 'not found in the ROS graph' in failures[0]

    def test_node_present_but_no_data_yet_is_reported_distinctly(self):
        config = _mission([
            _distance_move(stop_type='front_clearance', stop_params={'distance': 1.0})
        ])
        reqs = required_dependencies(config, scene_labels=True)
        live = {'mpc_corr', 'ackermann_to_vesc_node', 'lidar_front_wall_node'}
        # lidar_front_wall_node is up, but FRONT_CLEARANCE_KEY has no value yet.
        data = {CURRENT_XY_KEY: (0.0, 0.0), FRONT_CLEARANCE_KEY: None}
        failures = check_liveness(reqs, list(live), lambda k: data.get(k))
        assert len(failures) == 1
        assert 'lidar_front_wall_node' in failures[0]
        assert 'no data received yet' in failures[0]

    def test_multiple_missing_dependencies_all_reported(self):
        config = _mission([
            _distance_move(stop_type='object_seen', stop_params={'class': 'bottle'})
        ])
        reqs = required_dependencies(config)
        live = set()  # nothing alive at all
        data = {}
        failures = check_liveness(reqs, list(live), lambda k: data.get(k))
        # mpc_corr, ackermann_to_vesc_node, localization, yolo_detector_node
        assert len(failures) == 4


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))


class TestActuationNode:
    """preflight_actuation_node (sim overlay): the car's requirement is untouched by default."""

    def test_default_is_the_cars_ackermann_to_vesc_requirement_object(self):
        from f1tenth_behavior.mission import preflight
        config = _mission([_distance_move()])
        reqs = required_dependencies(config)
        assert preflight._ACKERMANN_TO_VESC in reqs
        assert required_dependencies(config, 'ackermann_to_vesc_node') == reqs

    def test_sim_names_the_simulator_instead(self):
        config = _mission([_distance_move()])
        names = _names(required_dependencies(config, 'isaac_f1tenth_sim'))
        assert 'isaac_f1tenth_sim' in names
        assert 'ackermann_to_vesc_node' not in names
        assert {'mpc_corr', 'localization (/odom)'} <= names

    def test_sim_requirement_failure_names_the_simulator_node(self):
        config = _mission([_distance_move()])
        reqs = required_dependencies(config, 'isaac_f1tenth_sim')
        failures = check_liveness(reqs, ['mpc_corr'], lambda key: (0.0, 0.0))
        assert len(failures) == 1 and 'isaac_f1tenth_sim' in failures[0]
