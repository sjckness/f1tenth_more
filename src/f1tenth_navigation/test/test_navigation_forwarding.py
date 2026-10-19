"""navigation.launch.py's forwarding, and the shipped boundary_source default.

Both claims this covers are about what the LAUNCH SYSTEM produces, not about
what a yaml file says, and the difference is the bug that was fixed here: this
file used to include mpc_corr.launch.py with a bare IncludeLaunchDescription and
no launch_arguments, so every mpc_corr argument given to stack_bringup or to the
supervisor's `navigation` entry was accepted at the top level and SILENTLY
DROPPED. A default can be right in stack_params.yaml and still never reach the
node. So these tests evaluate the launch description rather than reading the
yaml.

Nothing is launched and no node is constructed.

Run standalone: python3 -m pytest test/test_navigation_forwarding.py -v
"""

import importlib.util
import os
import sys

import pytest

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions


def _load(package, launch_file, name):
    """Load the INSTALLED launch file -- what ros2 launch would actually run."""
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _text(value, context):
    if value is None:
        return None
    if isinstance(value, str):
        return value
    if not isinstance(value, (list, tuple)):
        value = [value]
    return perform_substitutions(context, list(value))


@pytest.fixture(scope='module')
def nav():
    module = _load('f1tenth_navigation', 'navigation.launch.py', 'nav_launch_under_test')
    return module, module.generate_launch_description()


@pytest.fixture(scope='module')
def declared(nav):
    _module, ld = nav
    context = LaunchContext()
    return {e.name: _text(e.default_value, context)
            for e in ld.entities if isinstance(e, DeclareLaunchArgument)}


@pytest.fixture(scope='module')
def includes(nav):
    _module, ld = nav
    return [e for e in ld.entities if isinstance(e, IncludeLaunchDescription)]


class TestForwarding:

    def test_every_forwarded_key_is_also_declared(self, nav, declared):
        module, _ld = nav
        undeclared = [k for k in module._FORWARDED if k not in declared]
        assert undeclared == [], (
            'a forwarded-but-undeclared argument is worse than not forwarding it: '
            'ros2 launch rejects the command line as an unknown argument. '
            f'undeclared: {undeclared}')

    def test_the_mpc_corr_include_receives_them(self, nav, includes):
        module, _ld = nav
        context = LaunchContext()
        forwarded = set()
        for entity in includes:
            for name, _value in (entity.launch_arguments or []):
                forwarded.add(_text(name, context))
        assert forwarded == set(module._FORWARDED), (
            'the whole point of this file: these reach mpc_corr.launch.py rather '
            f'than being dropped. missing: {set(module._FORWARDED) - forwarded}')

    def test_every_forwarded_key_exists_on_mpc_corr(self, nav):
        # A forwarded name mpc_corr.launch.py does not declare is a launch-time
        # error, and it would only show up when somebody actually overrode it.
        module, _ld = nav
        mpc = _load('f1tenth_control', 'mpc_corr.launch.py', 'mpc_launch_under_test')
        mpc_args = {e.name for e in mpc.generate_launch_description().entities
                    if isinstance(e, DeclareLaunchArgument)}
        missing = [k for k in module._FORWARDED if k not in mpc_args]
        assert missing == [], f'mpc_corr.launch.py declares no such argument: {missing}'

    def test_map_only_gets_no_forwarded_arguments(self, includes):
        # Only the mpc_corr include takes them. map_only.launch.py declares none,
        # so forwarding to it would be a launch-time error.
        counts = sorted(len(e.launch_arguments or []) for e in includes)
        assert len(counts) == 2, f'expected mpc_corr + map_only, got {counts}'
        assert counts[0] == 0, 'map_only must receive nothing'
        assert counts[1] > 0, 'mpc_corr must receive the forwarded set'

    def test_the_forwarded_set_is_narrow_on_purpose(self, nav):
        # Not a blanket re-export of mpc_corr's whole argument list: that would
        # duplicate every default in a second place and make this file a
        # maintenance mirror of mpc_corr.launch.py.
        module, _ld = nav
        mpc = _load('f1tenth_control', 'mpc_corr.launch.py', 'mpc_launch_under_test2')
        mpc_args = {e.name for e in mpc.generate_launch_description().entities
                    if isinstance(e, DeclareLaunchArgument)}
        assert len(module._FORWARDED) < len(mpc_args), (
            'if this ever forwards everything, delete the list and forward '
            'programmatically instead of maintaining two copies')


class TestShippedDefaults:
    """The avoidance pass's own decision, pinned where a reader will find it."""

    def test_boundary_source_defaults_to_walls(self, declared):
        assert declared['boundary_source'] == 'walls', (
            'the MPC avoids only what perception RECOGNISES: one occupied cell is '
            'enough to make a costmap row, so a map artefact or a person already '
            'handled by w_obs becomes a half-plane across the whole horizon')

    def test_the_qp_has_room_for_every_wall_row(self, declared):
        # The startup check in MPCController.__init__ refuses to launch when it
        # does not, so a default pair that violates it would mean the car's own
        # launch path cannot start. This is the test that catches that in CI
        # rather than on the car.
        assert int(declared['boundary_max_sources']) >= int(
            declared['max_wall_constraints']), (
            f'boundary_max_sources {declared["boundary_max_sources"]} < '
            f'max_wall_constraints {declared["max_wall_constraints"]}: mpc_corr '
            'raises at construction rather than truncating a row silently')

    def test_the_slack_cap_is_reachable_and_unchanged(self, declared):
        assert float(declared['boundary_slack_max']) == 0.5, (
            'promoted to a parameter, default deliberately unchanged')

    def test_the_wall_slack_weight_is_the_derived_one(self, declared):
        assert float(declared['wall_row_slack_weight']) == 2.0e6

    def test_costmap_is_still_selectable(self, nav, declared):
        # boundary_source:=costmap must bring back the old behaviour, so the
        # value has to remain a legal one all the way down.
        from mpc_controller import wall_boundaries as wb
        assert wb.check_row_capacity(
            'costmap', int(declared['max_wall_constraints']),
            int(declared['boundary_max_sources'])) == ''
