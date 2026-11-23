"""supervisor_bringup.launch.py profile:=<name> (<profiles_dir>/<name>.yaml).

The profile fills the defaults of declared launch arguments before they are
declared; a value given on the command line wins. Its component_args reach
the launch files that declare them (f1tenth_bringup/launch_profile.py, and
apply_profile_component_args in the supervisor). Launch inspection only, no
rclpy node.
"""

import importlib.util
import os
import sys

from f1tenth_params.planner_modes import PLANNER_CONTROLLER_MODES
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import pytest

_PKG = os.path.join(os.path.dirname(__file__), '..')
_PROFILES = os.path.join(_PKG, 'config', 'profiles')
_COMPONENTS = os.path.join(_PKG, 'config', 'components.yaml')
# The source tree's f1tenth_bringup (launch_profile.py), not the install space's.
sys.path.insert(0, os.path.abspath(_PKG))
sys.path.insert(1, os.path.join(os.path.abspath(_PKG), 'f1tenth_bringup'))

from component_supervisor_node import (  # noqa: E402, I100
    _RUST_SWITCHES,
    apply_profile_component_args,
)
from f1tenth_bringup import launch_profile as lp  # noqa: E402


def _module():
    spec = importlib.util.spec_from_file_location(
        'supervisor_bringup', os.path.join(_PKG, 'launch', 'supervisor_bringup.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _run(cli):
    """Visit the launch description up to the declarations; return the context."""
    mod = _module()
    ld = mod.generate_launch_description()
    ctx = LaunchContext()
    ctx.launch_configurations.update(cli)
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)
        elif isinstance(entity, OpaqueFunction) and \
                entity._OpaqueFunction__function is mod._apply_profile:
            # The source tree's profiles, not the install space's.
            mod._apply_profile(ctx, *entity._OpaqueFunction__args, profiles_dir=_PROFILES)
    return ctx


def test_campaign3_rrt_sets_rrt_and_both_loggers():
    ctx = _run({'profile': 'campaign3_rrt'})
    cfg = ctx.launch_configurations
    assert cfg['planner_controller_mode'] == 'corridor_rrt'
    assert cfg['planner_controller_mode'] in PLANNER_CONTROLLER_MODES
    assert cfg['enable_mission_logger'] == 'true'
    assert cfg['enable_test_campaign_logger'] == 'true'
    assert cfg['enable_intelligence'] == 'true'


def test_command_line_wins_over_profile():
    ctx = _run({'profile': 'campaign3_rrt', 'planner_controller_mode': 'corridor_mpc'})
    assert ctx.launch_configurations['planner_controller_mode'] == 'corridor_mpc'
    assert ctx.launch_configurations['enable_test_campaign_logger'] == 'true'


def test_no_profile_keeps_test_campaign_logger_off():
    ctx = _run({})
    assert ctx.launch_configurations['profile'] == 'none'
    assert ctx.launch_configurations['enable_test_campaign_logger'] == 'false'


def _declared():
    ld = _module().generate_launch_description()
    return {e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)}


def _launch_args_or_skip(package, launch_file):
    try:
        return lp.launch_arguments_of(package, launch_file)
    except Exception as e:  # noqa: BLE001 - package not built in this environment
        pytest.skip(f'{package}/{launch_file} not loadable here: {e}')


def test_every_profile_validates():
    """Every profile in the repo: known sections, declared launch args, real components."""
    declared = _declared()
    registry = lp.load_registry(_COMPONENTS)
    names = lp.known_profiles(_PROFILES)
    assert {'campaign3_rrt', 'test_orin_rrt', 'test_orin_mpc'} <= set(names)
    for name in names:
        profile = lp.read_profile(name, _PROFILES)
        assert profile['launch_arguments'] or profile['component_args'], name
        lp.validate_profile(profile, declared, registry, launch_args_of=_launch_args_or_skip)


def test_every_profile_planner_mode_is_known():
    for name in lp.known_profiles(_PROFILES):
        mode = lp.load_profile(name, _PROFILES).get('planner_controller_mode')
        assert mode is None or mode in PLANNER_CONTROLLER_MODES, name


def test_unknown_profile_fails():
    with pytest.raises(RuntimeError, match='no '):
        lp.load_profile('does_not_exist', _PROFILES)


@pytest.mark.parametrize('name', ['../campaign3_rrt', '/etc/passwd', 'x.yaml/..', ''])
def test_profile_name_is_a_bare_name(name):
    if name == '':
        assert lp.read_profile(name, _PROFILES)['path'] is None
        return
    with pytest.raises(lp.ProfileError):
        lp.read_profile(name, _PROFILES)


def test_profiles_dir_argument_is_used(tmp_path):
    (tmp_path / 'mine.yaml').write_text('launch_arguments: {enable_comms: true}\n')
    mod = _module()
    ld = mod.generate_launch_description()
    ctx = LaunchContext()
    ctx.launch_configurations.update({'profile': 'mine', 'profiles_dir': str(tmp_path)})
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)
        elif isinstance(entity, OpaqueFunction) and \
                entity._OpaqueFunction__function is mod._apply_profile:
            mod._apply_profile(ctx, *entity._OpaqueFunction__args)
    assert ctx.launch_configurations['enable_comms'] == 'true'
    assert ctx.launch_configurations['_profile_component_args'] == '{}'


def _profile(tmp_path, text, name='p'):
    (tmp_path / f'{name}.yaml').write_text(text)
    return lp.read_profile(name, str(tmp_path))


def test_read_profile_hash_and_strings(tmp_path):
    p = _profile(tmp_path, 'launch_arguments: {enable_comms: false}\n'
                           'component_args: {perception: {yolo_backend: engine, n: 3}}\n')
    assert p['launch_arguments'] == {'enable_comms': 'false'}
    assert p['component_args'] == {'perception': {'yolo_backend': 'engine', 'n': '3'}}
    assert len(p['sha256']) == 64


def test_unknown_section_fails(tmp_path):
    with pytest.raises(lp.ProfileError, match='unknown sections'):
        _profile(tmp_path, 'launch_args: {enable_comms: true}\n')


_REGISTRY = {
    'perception': [
        {'package': 'f1tenth_perception', 'launch_file': 'camera.launch.py', 'args': {}},
        {'package': 'f1tenth_perception', 'launch_file': 'detection.launch.py',
         'args': {'yolo_backend': 'pt'}},
    ],
    'navigation': [{'package': 'f1tenth_navigation', 'launch_file': 'navigation.launch.py',
                    'args': {}}],
}
_FAKE_ARGS = {
    'camera.launch.py': {'camera_model'},
    'detection.launch.py': {'yolo_backend', 'yolo_model'},
    'navigation.launch.py': {'corridor_impl', 'planner_controller_mode'},
}


def _fake_launch_args(_package, launch_file):
    return _FAKE_ARGS[launch_file]


def test_component_args_go_only_to_the_declaring_launch_file():
    resolved = lp.resolve_component_args(
        {'perception': {'yolo_backend': 'engine'}}, _REGISTRY, _fake_launch_args)
    assert resolved == {'perception': {'detection.launch.py': {'yolo_backend': 'engine'}}}
    import copy
    registry = apply_profile_component_args(copy.deepcopy(_REGISTRY), resolved)
    assert registry['perception'][0]['args'] == {}
    assert registry['perception'][1]['args'] == {'yolo_backend': 'engine'}


@pytest.mark.parametrize('component_args, match', [
    ({'percepton': {'yolo_backend': 'engine'}}, 'not in components.yaml'),
    ({'perception': {'yolo_backed': 'engine'}}, 'declared by none'),
    ({'navigation': {'corridor_impl': 'rust'}}, 'set by the supervisor'),
    ({'navigation': {'planner_controller_mode': 'corridor_rrt'}}, 'set by the supervisor'),
])
def test_component_args_errors(component_args, match):
    with pytest.raises(lp.ProfileError, match=match):
        lp.resolve_component_args(component_args, _REGISTRY, _fake_launch_args)


def test_validate_lists_every_error(tmp_path):
    p = _profile(tmp_path, 'launch_arguments: {nope: 1}\n'
                           'component_args: {ghost: {a: 1}}\n')
    with pytest.raises(lp.ProfileError) as e:
        lp.validate_profile(p, _declared(), _REGISTRY, _fake_launch_args)
    assert 'nope' in str(e.value) and 'ghost' in str(e.value)


def test_supervisor_refuses_unknown_component_or_file():
    import copy
    with pytest.raises(ValueError, match='not in components.yaml'):
        apply_profile_component_args(copy.deepcopy(_REGISTRY), {'ghost': {'x.launch.py': {}}})
    with pytest.raises(ValueError, match='does not launch'):
        apply_profile_component_args(copy.deepcopy(_REGISTRY),
                                     {'perception': {'lidar.launch.py': {'a': '1'}}})


def test_injected_args_cover_the_supervisor_switches():
    """Every argument the supervisor writes itself is refused in component_args."""
    assert set(_RUST_SWITCHES) <= lp.SUPERVISOR_INJECTED_ARGS
    assert {'calibration', 'enable_mission_logger', 'planner_controller_mode'} \
        <= lp.SUPERVISOR_INJECTED_ARGS


def test_cli_check_and_list(capsys):
    assert lp.main(['list', '--profiles-dir', _PROFILES]) == 0
    assert 'campaign3_rrt' in capsys.readouterr().out.split()


def test_component_args_reach_the_supervisor_as_a_parameter(tmp_path, monkeypatch):
    """Profile component_args -> resolved -> profile_component_args (JSON) on the supervisor."""
    import json

    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    (tmp_path / 'p.yaml').write_text('component_args: {perception: {yolo_backend: engine}}\n')
    mod = _module()
    monkeypatch.setattr(mod, 'resolve_component_args',
                        lambda args, registry: lp.resolve_component_args(
                            args, _REGISTRY, _fake_launch_args))
    ld = mod.generate_launch_description()
    ctx = LaunchContext()
    ctx.launch_configurations.update({'profile': 'p', 'profiles_dir': str(tmp_path)})
    supervisor = None
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)
        elif isinstance(entity, OpaqueFunction):
            fn = entity._OpaqueFunction__function
            if fn is mod._apply_profile:
                mod._apply_profile(ctx, *entity._OpaqueFunction__args)
            elif fn.__name__ == '_supervisor_node':
                supervisor = entity
    node = next(a for a in supervisor.execute(ctx) if isinstance(a, Node))
    params = {}
    for p in evaluate_parameters(ctx, normalize_parameters(node._Node__parameters)):
        params.update(p)
    assert json.loads(params['profile_component_args']) == {
        'perception': {'detection.launch.py': {'yolo_backend': 'engine'}}}


def test_no_component_args_no_supervisor_parameter():
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    mod = _module()
    ld = mod.generate_launch_description()
    ctx = _run({'profile': 'campaign3_rrt'})
    op = next(e for e in ld.entities if isinstance(e, OpaqueFunction)
              and e._OpaqueFunction__function.__name__ == '_supervisor_node')
    node = next(a for a in op.execute(ctx) if isinstance(a, Node))
    params = {}
    for p in evaluate_parameters(ctx, normalize_parameters(node._Node__parameters)):
        params.update(p)
    assert 'profile_component_args' not in params
