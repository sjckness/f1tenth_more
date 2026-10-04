"""components.yaml's topic-liveness section (fix batch 5, H1).

Every component states how its liveness is judged, and the rules that are easy
to break by accident fail loudly.

Plain YAML checks plus a message-type import; no rclpy context.
"""
import os
import sys

from rosidl_runtime_py.utilities import get_message
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'f1tenth_bringup'))

from topic_watchdog import parse_health_config  # noqa: E402, I100

_COMPONENTS_YAML = os.path.join(os.path.dirname(__file__), '..', 'config', 'components.yaml')


def _doc():
    with open(_COMPONENTS_YAML) as f:
        return yaml.safe_load(f)


def _parsed():
    doc = _doc()
    return parse_health_config(doc, doc['components'])


def test_every_component_is_watched_or_says_why_not():
    doc = _doc()
    health, _, unwatched = _parsed()
    missing = set(doc['components']) - set(health) - set(unwatched)
    assert not missing, (
        f'{sorted(missing)}: add a health: entry in components.yaml, or an '
        'unwatched: entry saying why it has no liveness check')
    assert all(isinstance(r, str) and r.strip() for r in unwatched.values())


def test_every_health_topic_type_imports():
    _, types, _ = _parsed()
    for topic, type_name in types.items():
        assert get_message(type_name) is not None, topic


def test_hardware_and_perception_are_alert_only():
    """Never restart these: it would restart the VESC driver or urg_node.

    urg_node publishes the e-stop's only /scan.
    """
    health, _, _ = _parsed()
    assert health['hardware'].action == 'alert'
    assert health['perception'].action == 'alert'


def test_navigation_is_not_judged_by_drive():
    """Judge mpc_corr by its input-freshness status, never by /drive.

    mpc_corr publishes /drive every tick with or without odometry: fix batch
    3's isolated mpc_corr did exactly that.
    """
    health, _, _ = _parsed()
    checks = health['navigation'].checks
    assert '/drive' not in {c.topic for c in checks}
    assert any(c.topic == '/mpc/input_status' and c.expect.get('level') == [0]
               for c in checks)


def test_slam_is_not_judged_by_slam_pose():
    """Never judge slam by /slam/pose.

    slam_toolbox publishes it only after the car has moved: a parked car would
    be restarted.
    """
    health, _, _ = _parsed()
    assert '/slam/pose' not in {c.topic for c in health['slam'].checks}


def test_enabled_if_names_real_stack_params():
    params_yaml = os.path.join(os.path.dirname(__file__), '..', '..', 'f1tenth_params',
                               'config', 'stack_params.yaml')
    with open(params_yaml) as f:
        params = yaml.safe_load(f)
    health, _, _ = _parsed()
    for name, cfg in health.items():
        for key in cfg.enabled_if:
            assert key in params, f'{name}: enabled_if {key} is not in stack_params.yaml'


def test_max_ages_are_not_tighter_than_half_a_second():
    health, _, _ = _parsed()
    for name, cfg in health.items():
        for c in cfg.checks:
            assert c.max_age_sec >= 0.5, (name, c.topic)
