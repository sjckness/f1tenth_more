"""mission/preflight.py tests -- required_dependencies()/check_liveness() are
plain functions with zero rclpy dependency, same "pure logic, static/
synthetic only" convention test_condition_eval.py already establishes.

Run standalone: python3 -m pytest test/test_preflight.py -v
"""

import pytest

from f1tenth_behavior.mission.mission_config import parse_mission
from f1tenth_behavior.mission.preflight import check_liveness, required_dependencies
from f1tenth_behavior.mission.runtime import CURRENT_XY_KEY, FRONT_CLEARANCE_KEY


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
        names = _names(required_dependencies(config))
        assert 'costmap_boundary_node' not in names
        assert 'yolo_detector_node' not in names

    def test_front_clearance_stop_condition_requires_costmap_boundary_node(self):
        config = _mission([_distance_move(stop_type='front_clearance', stop_params={'distance': 1.0})])
        names = _names(required_dependencies(config))
        assert 'costmap_boundary_node' in names

    def test_front_clearance_as_resume_condition_also_requires_it(self):
        move = _distance_move(
            stop_type='time_elapsed', stop_params={'duration_sec': 5.0},
            on_object=[{
                'class': 'person', 'action': 'stop_and_hold',
                'resume_condition': {'type': 'front_clearance', 'distance': 2.0},
            }],
        )
        names = _names(required_dependencies(_mission([move])))
        assert 'costmap_boundary_node' in names
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
        reqs = required_dependencies(config)
        live = {'mpc_corr', 'ackermann_to_vesc_node', 'costmap_boundary_node'}
        # costmap_boundary_node is up, but FRONT_CLEARANCE_KEY has no value yet.
        data = {CURRENT_XY_KEY: (0.0, 0.0), FRONT_CLEARANCE_KEY: None}
        failures = check_liveness(reqs, list(live), lambda k: data.get(k))
        assert len(failures) == 1
        assert 'costmap_boundary_node' in failures[0]
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
