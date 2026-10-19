"""
The 'diagnostics' group must not "crash" when the mission logger is off.

Orin, campaign-final, 2026-10-16 (output/live/stack_nolog.log): with
enable_mission_logger false the supervisor started
`ros2 launch f1tenth_logger mission_logger.launch.py enable_mission_logger:=false`.
Every Node in that file is gated on the flag, so launch had nothing to run and
exited 0 after ~4 s; the watchdog reads any exit as a crash: three
"crashed (exit 0) -- auto-respawning", then "exceeded its restart budget"
every 2 s. Never with the logger on (stack_log.log: 0 crashes).

The fix drops the disabled launch file from the registry
(ComponentSupervisorNode._apply_mission_logger_override). The first two tests
run without ROS (the method is lifted out of the module source with ast); the
launch-file test needs launch/launch_ros: run on the Orin.
"""
import ast
import copy
import os

import pytest
import yaml

_HERE = os.path.dirname(__file__)
_SUPERVISOR = os.path.join(_HERE, '..', 'f1tenth_bringup', 'component_supervisor_node.py')
_COMPONENTS = os.path.join(_HERE, '..', 'config', 'components.yaml')
_LOGGER_LAUNCH = os.path.join(
    _HERE, '..', '..', 'f1tenth_logger', 'launch', 'mission_logger.launch.py')


def _method(name):
    """Return a ComponentSupervisorNode method as a plain function (no rclpy)."""
    tree = ast.parse(open(_SUPERVISOR).read())
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == 'ComponentSupervisorNode')
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = {}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), _SUPERVISOR, 'exec'), namespace)
    return namespace[name]


class _Supervisor:
    def __init__(self, registry, enable_mission_logger):
        self._registry = registry
        self.enable_mission_logger = enable_mission_logger


def _registry():
    with open(_COMPONENTS) as f:
        return copy.deepcopy(yaml.safe_load(f)['components'])


def _files(entries):
    return [e['launch_file'] for e in entries]


def test_logger_off_drops_only_the_logger_launch_file_from_diagnostics():
    sup = _Supervisor(_registry(), enable_mission_logger=False)
    assert 'mission_logger.launch.py' in _files(sup._registry['diagnostics'])
    _method('_apply_mission_logger_override')(sup)
    files = _files(sup._registry['diagnostics'])
    # Nothing the watchdog could see exit 0 ...
    assert 'mission_logger.launch.py' not in files
    # ... and the rest of the group still starts.
    assert files == ['system_observer.launch.py', 'diagnostics_server.launch.py']


def test_logger_on_still_launches_it_with_the_flag_true():
    sup = _Supervisor(_registry(), enable_mission_logger=True)
    _method('_apply_mission_logger_override')(sup)
    entry = next(e for e in sup._registry['diagnostics']
                 if e['launch_file'] == 'mission_logger.launch.py')
    assert entry['args']['enable_mission_logger'] == 'true'


def test_with_the_flag_false_the_logger_launch_file_runs_nothing():
    """
    Reproduce the crash classification's cause.

    Launched with :=false the file has no action left to run, which is the
    clean exit 0 the watchdog called a crash. Needs launch/launch_ros (Orin).
    """
    launch = pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    import importlib.util
    from launch_ros.actions import Node
    spec = importlib.util.spec_from_file_location('mission_logger_launch', _LOGGER_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ld = module.generate_launch_description()
    context = launch.LaunchContext()
    for arg in ld.entities:
        if isinstance(arg, launch.actions.DeclareLaunchArgument):
            arg.visit(context)
    context.launch_configurations['enable_mission_logger'] = 'false'
    nodes = [e for e in ld.entities if isinstance(e, Node)]
    assert nodes
    assert not [n for n in nodes if n.condition is None or n.condition.evaluate(context)]
