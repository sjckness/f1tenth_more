"""
stack_bringup.launch.py starts LLM2 (llm2.launch.py) only when enable_llm2 is true.

LLM2 (two-llm-arch) is a second llama-server plus llm2_mpc_param_node. With
enable_llm2 at its default (false) the stack must launch exactly as before:
the include's condition is false, so llm2.launch.py is never even loaded.
Without the condition every bringup would start a second 2.3 GB model.

No live launch: the real LaunchDescription is built with the package share
lookup stubbed and each include's file path recorded (the included files are
named, never loaded), and the include's condition is evaluated against the
declared defaults, then with enable_llm2:=true. Nothing is started.

Run standalone: python3 -m pytest test/test_stack_bringup_llm2_gate.py -v
"""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import pytest

_STACK_BRINGUP = os.path.join(
    os.path.dirname(__file__), '..', 'launch', 'stack_bringup.launch.py')


class _RecordedSource(PythonLaunchDescriptionSource):
    """The real source, plus the path it was given (its location is lazy)."""

    def __init__(self, path):
        super().__init__(path)
        self.path = path


def _includes_and_context(**overrides):
    spec = importlib.util.spec_from_file_location('stack_bringup', _STACK_BRINGUP)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.get_package_share_directory = lambda package: os.path.join('/share', package)
    mod.PythonLaunchDescriptionSource = _RecordedSource
    entities = mod.generate_launch_description().entities
    ctx = LaunchContext()
    for key, value in overrides.items():
        ctx.launch_configurations[key] = value
    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)   # sets the default unless overridden above
    includes = [e for e in entities if isinstance(e, IncludeLaunchDescription)]
    return includes, ctx


def _llm2_includes(includes):
    return [i for i in includes if i.launch_description_source.path
            == os.path.join('/share', 'llm', 'launch', 'llm2.launch.py')]


def test_llm2_launch_is_included_once_behind_a_condition():
    includes, _ = _includes_and_context()
    (llm2,) = _llm2_includes(includes)
    assert llm2.condition is not None


@pytest.mark.parametrize('overrides, included', [
    ({}, False),                        # the default: the stack as before
    ({'enable_llm2': 'false'}, False),
    ({'enable_llm2': 'true'}, True),
])
def test_stack_bringup_includes_llm2_only_when_enable_llm2_is_true(overrides, included):
    includes, ctx = _includes_and_context(**overrides)
    (llm2,) = _llm2_includes(includes)
    assert llm2.condition.evaluate(ctx) is included
