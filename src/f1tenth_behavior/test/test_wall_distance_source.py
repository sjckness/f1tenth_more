"""stop_condition.distance_source on a front_clearance move (campaign fix A4).

The LLM translator records who chose a wall stop's distance -- the model
('llm') or its default ('default' / 'default_before_turn') -- because R009
stopped on a 2.0 m default that nothing on disk named. The loader accepts the
field on front_clearance only, with those values (plus 'mission' for a
hand-written one).
"""

import pytest

from f1tenth_behavior.mission.mission_config import MissionConfigError, parse_mission


def _mission(stop):
    return {'mission_id': 't', 'moves': [{
        'id': 'm', 'drive': {'mode': 'straight', 'speed': 0.4},
        'stop_condition': stop, 'timeout_sec': 30, 'on_timeout': 'abort'}]}


@pytest.mark.parametrize('source', ['llm', 'default', 'default_before_turn', 'mission'])
def test_a_known_source_loads_and_is_kept(source):
    config = parse_mission(_mission({'type': 'front_clearance', 'distance': 0.5,
                                     'distance_source': source}))
    assert config.moves[0].stop_condition.params['distance_source'] == source


def test_an_unknown_source_is_refused():
    with pytest.raises(MissionConfigError):
        parse_mission(_mission({'type': 'front_clearance', 'distance': 0.5,
                                'distance_source': 'guess'}))


def test_the_field_belongs_to_front_clearance_only():
    with pytest.raises(MissionConfigError):
        parse_mission(_mission({'type': 'obstacle_distance_below', 'distance': 0.5,
                                'distance_source': 'llm'}))


def test_it_stays_optional():
    config = parse_mission(_mission({'type': 'front_clearance', 'distance': 0.5}))
    assert 'distance_source' not in config.moves[0].stop_condition.params


def _check_stop_condition(root):
    from f1tenth_behavior.behaviours.check_stop_condition import CheckStopCondition
    found = [b for b in root.iterate() if isinstance(b, CheckStopCondition)]
    assert len(found) == 1
    return found[0]


def test_the_wall_stop_reads_the_lidar_confirmed_distance_by_default():
    """Campaign fix A4: camera AND LiDAR must agree before the wall stop fires."""
    from f1tenth_behavior.behavior_executor_node import create_root
    from f1tenth_params.param_defaults import get_value
    assert get_value('wall_stop_require_lidar') is True
    check = _check_stop_condition(create_root())
    assert check._front_clearance_topic == '/perception/front_wall_confirmed'


def test_the_camera_only_stop_is_one_parameter_away():
    from f1tenth_behavior.behavior_executor_node import create_root
    check = _check_stop_condition(create_root(wall_stop_require_lidar=False))
    assert check._front_clearance_topic == '/perception/front_distance'
