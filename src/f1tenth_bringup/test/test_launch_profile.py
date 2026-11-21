"""supervisor_bringup.launch.py profile:=<name> (config/profiles/<name>.yaml).

The profile fills the defaults of declared launch arguments before they are
declared; a value given on the command line wins. Launch inspection only, no
rclpy node.
"""

import importlib.util
import os

from f1tenth_params.planner_modes import PLANNER_CONTROLLER_MODES
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import pytest

_PKG = os.path.join(os.path.dirname(__file__), '..')
_PROFILES = os.path.join(_PKG, 'config', 'profiles')


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


def test_every_profile_names_only_declared_arguments():
    mod = _module()
    ld = mod.generate_launch_description()
    declared = {e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    for f in os.listdir(_PROFILES):
        values = mod.load_profile(f[:-5], _PROFILES)
        assert values, f
        assert set(values) <= declared, f


def test_unknown_profile_fails():
    with pytest.raises(RuntimeError, match='no '):
        _module().load_profile('does_not_exist', _PROFILES)
