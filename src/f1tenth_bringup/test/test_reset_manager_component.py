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

from launch import LaunchContext
from launch.actions import OpaqueFunction
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


def _launch_params():
    import importlib.util
    path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'reset_manager.launch.py')
    spec = importlib.util.spec_from_file_location('reset_manager_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    node = description.entities[-1]
    if isinstance(node, OpaqueFunction):
        # The Node is built in an OpaqueFunction (the sim overlay's
        # params_overlay is read there): evaluate it with the declared
        # defaults, as `ros2 launch` would.
        context = LaunchContext()
        for entity in description.entities[:-1]:
            entity.visit(context)
        node = node.execute(context)[0]
    raw = [p for p in node._Node__parameters if isinstance(p, dict)][0]

    # launch_ros normalises keys and values into TextSubstitution tuples.
    def text(subs):
        return ''.join(s.text for s in subs)

    # A list item arrives as its own YAML document ('wall_distance\n...\n').
    return {text(k): ([yaml.safe_load(text(item)) or '' for item in v]
                      if isinstance(v, (list, tuple)) else v)
            for k, v in raw.items()}


def test_a_disabled_wall_distance_is_not_restarted_by_the_hooks():
    """
    Skip a disabled wall_distance in the /reset_all hooks (cpu-cleanup).

    restart_component on a component the supervisor did not start would
    start it. With enable_wall_distance false (the default) the launch file
    tells reset_manager to skip it; with it true, skip nothing.
    """
    from f1tenth_params.param_defaults import get_value
    skip = _launch_params()['hooks.skip_components']
    if get_value('enable_wall_distance'):
        assert skip == ['']
    else:
        assert skip == ['wall_distance']
        assert _CONDITIONAL_AUTO_START['wall_distance'] == 'enable_wall_distance'
