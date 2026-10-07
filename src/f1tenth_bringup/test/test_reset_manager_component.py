"""Supervisor side of reset_manager: its own component, never restarting the LLM.

It starts at boot and is never configured to restart the LLM server.

/reset_all restarts `hardware`, `slam` and its hook components through the
supervisor's /restart_component. It must not be able to restart itself
mid-reset (own component), and it must never restart `intelligence`, the
llama-server whose restart costs the next prompt its warm-up.

Plain YAML and set checks, no rclpy.
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'f1tenth_bringup'))

from component_supervisor_node import (  # noqa: E402
    _ALWAYS_AUTO_START, _CONDITIONAL_AUTO_START, _NEVER_AUTO_START)

_CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')


def _components():
    with open(os.path.join(_CONFIG, 'components.yaml')) as f:
        return yaml.safe_load(f)['components']


def _params():
    with open(os.path.join(_CONFIG, 'reset_manager.yaml')) as f:
        return yaml.safe_load(f)['reset_manager']['ros__parameters']


def test_reset_manager_is_a_component_of_its_own():
    entries = _components()['reset_manager']
    assert [(e['package'], e['launch_file']) for e in entries] == [
        ('f1tenth_bringup', 'reset_manager.launch.py')]


def test_it_starts_with_the_stack():
    assert 'reset_manager' in _ALWAYS_AUTO_START
    assert 'reset_manager' not in _NEVER_AUTO_START
    assert 'reset_manager' not in _CONDITIONAL_AUTO_START


def test_it_never_restarts_the_llm_server_or_itself():
    params = _params()
    restarted = {params['slam']['component'],
                 *params['hooks']['restart_components']}
    if params['wheel_odom']['method'] == 'restart_component':
        restarted.add(params['wheel_odom']['component'])
    assert 'intelligence' in params['protected_components']
    assert not restarted & {'intelligence', 'reset_manager'}
    components = _components()
    assert restarted <= set(components), restarted - set(components)
