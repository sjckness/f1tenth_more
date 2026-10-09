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
