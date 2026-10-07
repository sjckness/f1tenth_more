"""mission/preflight.py tests -- required_dependencies()/check_liveness() are
plain functions with zero rclpy dependency, same "pure logic, static/
synthetic only" convention test_condition_eval.py already establishes.

Run standalone: python3 -m pytest test/test_preflight.py -v
"""

from pathlib import Path

import pytest

from f1tenth_behavior.mission.mission_config import load_mission_file, parse_mission
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


class TestRequiredDependencies:

    def test_always_requires_mpc_corr_ackermann_and_localization(self):
        config = _mission([_distance_move()])
        reqs = required_dependencies(config)
        names = _names(reqs)
        assert 'mpc_corr' in names
        assert 'ackermann_to_vesc_node' in names
        assert 'localization (/odom)' in names

    def test_sim_swaps_ackermann_to_vesc_for_the_sim_drive_bridge(self):
        # In sim there is no VESC; f1tenth_sim_drive_bridge is the actuation.
        # mpc_corr + localization stay required, ackermann_to_vesc_node does not.
        config = _mission([_distance_move()])
        reqs = required_dependencies(config, sim=True)
        names = _names(reqs)
        assert 'mpc_corr' in names
        assert 'localization (/odom)' in names
        assert 'sim drive bridge' in names
        assert 'ackermann_to_vesc_node' not in names
        drive = next(r for r in reqs if r.name == 'sim drive bridge')
        assert drive.node_name == 'f1tenth_sim_drive_bridge'

    def test_car_mode_default_keeps_ackermann_to_vesc_and_not_the_sim_bridge(self):
        names = _names(required_dependencies(_mission([_distance_move()]), sim=False))
        assert 'ackermann_to_vesc_node' in names
        assert 'sim drive bridge' not in names

    def test_plain_distance_mission_does_not_require_perception_or_costmap(self):
        config = _mission([_distance_move()])
        names = _names(required_dependencies(config))
        assert 'front_clearance_node' not in names
        assert 'yolo_detector_node' not in names

    def test_front_clearance_stop_condition_requires_front_clearance_node(self):
        # The condition reads /perception/front_distance (63a6080), published by
        # front_clearance_node -- not costmap_boundary_node, which f960cf2's
        # original check named when the condition still read /costmap/front_clearance.
        config = _mission([_distance_move(stop_type='front_clearance', stop_params={'distance': 1.0})])
        reqs = required_dependencies(config)
        names = _names(reqs)
        assert 'front_clearance_node' in names
        assert 'costmap_boundary_node' not in names
        req = next(r for r in reqs if r.name == 'front_clearance_node')
        assert req.node_name == 'front_clearance_node'
        assert req.blackboard_key == FRONT_CLEARANCE_KEY

    def test_front_clearance_as_resume_condition_also_requires_it(self):
        move = _distance_move(
            stop_type='time_elapsed', stop_params={'duration_sec': 5.0},
            on_object=[{
                'class': 'person', 'action': 'stop_and_hold',
                'resume_condition': {'type': 'front_clearance', 'distance': 2.0},
            }],
        )
        names = _names(required_dependencies(_mission([move])))
        assert 'front_clearance_node' in names
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
        reqs = required_dependencies(config)
        live = {'mpc_corr', 'ackermann_to_vesc_node', 'front_clearance_node'}
        # front_clearance_node is up, but FRONT_CLEARANCE_KEY has no value yet.
        data = {CURRENT_XY_KEY: (0.0, 0.0), FRONT_CLEARANCE_KEY: None}
        failures = check_liveness(reqs, list(live), lambda k: data.get(k))
        assert len(failures) == 1
        assert 'front_clearance_node' in failures[0]
        assert 'no data received yet' in failures[0]

    def test_front_clearance_with_only_costmap_boundary_node_up_is_refused(self):
        # The case the old check let through by name: the old producer is up,
        # the actual producer of /perception/front_distance is not.
        config = _mission([
            _distance_move(stop_type='front_clearance', stop_params={'distance': 1.0})
        ])
        reqs = required_dependencies(config)
        live = {'mpc_corr', 'ackermann_to_vesc_node', 'costmap_boundary_node'}
        data = {CURRENT_XY_KEY: (0.0, 0.0), FRONT_CLEARANCE_KEY: 3.0}
        failures = check_liveness(reqs, list(live), lambda k: data.get(k))
        assert len(failures) == 1
        assert 'front_clearance_node' in failures[0]
        assert 'not found in the ROS graph' in failures[0]

    def test_front_clearance_node_up_and_publishing_passes(self):
        config = _mission([
            _distance_move(stop_type='front_clearance', stop_params={'distance': 1.0})
        ])
        reqs = required_dependencies(config)
        live = {'mpc_corr', 'ackermann_to_vesc_node', 'front_clearance_node'}
        data = {CURRENT_XY_KEY: (0.0, 0.0), FRONT_CLEARANCE_KEY: 3.0}
        assert check_liveness(reqs, list(live), lambda k: data.get(k)) == []

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
