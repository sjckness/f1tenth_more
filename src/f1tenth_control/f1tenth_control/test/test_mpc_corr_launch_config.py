"""mpc_corr.launch.py config tests -- the launch file's OWN wiring, not the node's.

Same "no live launch, direct Python construction of generate_launch_description()
+ a LaunchContext" convention as f1tenth_perception/test/test_detection_launch_
config.py and f1tenth_localization/test/test_ekf_global_config.py. No rclpy node,
no solver, no hardware.

WHY THIS FILE EXISTS. On 2026-09-23 every supervisor bringup lost mpc_corr --
the MPC itself -- before main() ran:

    rclpy.exceptions.InvalidParameterTypeException: Trying to set parameter
    'object_corridor_mode' to 'False' of type 'BOOL', expecting type 'STRING'

The value was correct everywhere a test was looking. stack_params.yaml holds
`default: "off"`, get_default() returns the string 'off', and mpc_controller's
own test_object_corridor_mode.py asserts exactly that. The damage happened in
between: an UNTYPED LaunchConfiguration is coerced with YAML 1.1 rules when
launch builds the parameter file, and under those rules a bare `off` is the
BOOLEAN false. 'arc' and 'arc_far' survive, so only the default -- i.e. every
plain bringup -- was affected, and the supervisor's watchdog just respawned the
crash until the restart budget ran out.

So the assertions here are deliberately about the TYPE that comes out of the
substitution machinery, at the last point before the node reads it. A test that
checks the launch ARGUMENT (ctx.launch_configurations) cannot see this bug: the
argument is the string 'off' either way. The coercion is downstream of that, in
evaluate_parameters(), which is what this file calls.

Run standalone: python3 -m pytest test/test_mpc_corr_launch_config.py -v
"""

import importlib.util
import os

import pytest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
_LAUNCH_DIR = os.path.join(os.path.dirname(_THIS_DIR), 'launch')

# MPC_corr.__init__ rejects anything else and falls back to 'off' with a warning;
# kept here as a literal so that widening the node's set has to come past this
# file too.
_VALID_MODES = ('off', 'arc', 'arc_far')


def _load_mpc_corr_launch():
    spec = importlib.util.spec_from_file_location(
        'mpc_corr_launch', os.path.join(_LAUNCH_DIR, 'mpc_corr.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _node_parameters(overrides=None):
    """The parameters mpc_corr actually receives, resolved the way launch does.

    Builds the real LaunchDescription, seeds a LaunchContext with `overrides`
    (the `key:=value` a CLI user would pass), visits every DeclareLaunchArgument
    so the defaults land, then runs the node's parameter list through the same
    normalize/evaluate pair launch_ros uses internally to write the params file.
    Returns one flat dict.
    """
    launch_description = _load_mpc_corr_launch().generate_launch_description()
    ctx = LaunchContext()
    for key, value in (overrides or {}).items():
        ctx.launch_configurations[key] = value
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)

    node = next(e for e in launch_description.entities if isinstance(e, Node))
    # Name-mangled because launch_ros exposes no public accessor for a Node's
    # unresolved parameters; this is the same attribute Node._perform_
    # substitutions() reads before it writes the temporary params YAML.
    raw = node._Node__parameters
    merged = {}
    for chunk in evaluate_parameters(ctx, normalize_parameters(raw)):
        if isinstance(chunk, dict):
            merged.update(chunk)
    return merged


class TestObjectCorridorModeSurvivesSubstitution:
    """The 2026-09-23 bringup crash, pinned at the layer that broke."""

    def test_default_mode_reaches_the_node_as_the_string_off(self):
        # The regression. `False` here is the crash; anything non-str is the
        # same class of bug with a different YAML-1.1 keyword.
        value = _node_parameters()['object_corridor_mode']
        assert isinstance(value, str), (
            f'object_corridor_mode reached the node as {value!r} '
            f'({type(value).__name__}); launch coerced the string with YAML 1.1 '
            f'rules. Wrap it in ParameterValue(..., value_type=str).')
        assert value == 'off'

    @pytest.mark.parametrize('mode', _VALID_MODES)
    def test_every_accepted_mode_survives_as_a_string(self, mode):
        # 'arc' and 'arc_far' were never at risk, which is exactly why the bug
        # hid: it only showed on the value nobody passes explicitly.
        value = _node_parameters({'object_corridor_mode': mode})['object_corridor_mode']
        assert isinstance(value, str)
        assert value == mode

    def test_the_default_is_one_of_the_modes_the_node_accepts(self):
        # Couples the launch default to MPC_corr's own validation set: a new
        # mode name in stack_params.yaml that the node would reject (and
        # silently downgrade to 'off') fails here instead.
        assert _node_parameters()['object_corridor_mode'] in _VALID_MODES
