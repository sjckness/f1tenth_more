"""
llm2.launch.py wiring: one port for the LLM2 server and its node, never LLM1's.

Same no-live-launch convention as f1tenth_control/test/test_mpc_corr_launch_
config.py: the real LaunchDescription, a LaunchContext seeded with the
arguments, and the OpaqueFunction performed by hand. Nothing is started.

What it pins:
  - LLM2 is off by default (enable_llm2, a config coupling with
    stack_params.yaml);
  - the shipped llm2_port is not LLM1's port (models.yaml's planner model).
    The intelligence component starts LLM1 by default, so a second server on
    that port could not bind and the node would quietly talk to LLM1;
  - the server is started with llm2_model's file and flags on llm2_port, and
    the node's server_url is built from that same port.

Run standalone: python3 -m pytest test/test_llm2_launch.py -v
"""

import importlib.util
import os
import pathlib

from f1tenth_params.param_defaults import get_value
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import yaml

LLM_DIR = pathlib.Path(__file__).resolve().parents[1]


def _yaml(name):
    with open(LLM_DIR / 'config' / name) as f:
        return yaml.safe_load(f)


def _llm1_port():
    planner_model = _yaml('interrogations.yaml')['planner']['default_model']
    return int(_yaml('models.yaml')[planner_model]['port'])


def _launch_actions(**overrides):
    """Return the actions llm2.launch.py would start for `overrides` (key:=value)."""
    spec = importlib.util.spec_from_file_location(
        'llm2_launch', LLM_DIR / 'launch' / 'llm2.launch.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    launch_description = mod.generate_launch_description()
    ctx = LaunchContext()
    for key, value in overrides.items():
        ctx.launch_configurations[key] = value
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)
    opaque = next(e for e in launch_description.entities if isinstance(e, OpaqueFunction))
    return ctx, opaque.execute(ctx)


def test_llm2_is_off_by_default():
    assert get_value('enable_llm2') is False


def test_the_shipped_llm2_port_is_not_llm1s_port():
    assert int(get_value('llm2_port')) != _llm1_port()


def test_the_server_and_the_node_share_llm2_port():
    ctx, actions = _launch_actions(llm2_port='8099')
    servers = [a for a in actions if isinstance(a, ExecuteProcess) and not isinstance(a, Node)]
    nodes = [a for a in actions if isinstance(a, Node)]
    assert len(servers) == 1 and len(nodes) == 1

    cmd = [perform_substitutions(ctx, part) for part in servers[0].cmd]
    model = _yaml('models.yaml')[get_value('llm2_model')]
    assert cmd[1:] == ['-m', os.path.expanduser(model['model_path']), '--port', '8099',
                       *model['extra_args']]

    # Name-mangled for the reason test_mpc_corr_launch_config.py gives: no
    # public accessor for a launch_ros Node's unresolved parameters. The dict
    # comes after the params file, so it is the value the node receives.
    raw = nodes[0]._Node__parameters
    inline = {}
    for chunk in evaluate_parameters(ctx, normalize_parameters(raw)):
        if isinstance(chunk, dict):
            inline.update(chunk)
    assert inline == {'server_url': 'http://127.0.0.1:8099'}
