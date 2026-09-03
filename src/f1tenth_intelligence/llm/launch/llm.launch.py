"""llama-server bringup ONLY (see config/interrogations.yaml for the model this
resolves against). Does NOT launch an interrogation node -- component-auto-start
pass: this file used to also launch the selected interrogation's node (e.g.
llm_planner_node) as a persistent background Node action, but that was always
architecturally wrong for a CLI tool -- llm_planner_node.py takes a command as a
positional argv arg and, absent one, falls back to reading commands from stdin,
which `ros2 launch` does not reliably give it a real terminal for (see
llm_planner_node's own module docstring and llm_planner_params.yaml's comment on
this exact limitation, pre-dating this pass). Making f1tenth_bringup/component_
supervisor_node.py's new 'intelligence' component (see components.yaml, gated on
enable_intelligence) auto-start this file at supervisor bringup would have made
that mismatch a live, automatic problem instead of only a manual-invocation
footgun -- fixed here by removing the node launch entirely, project-wide, not
just for the supervisor path. llm_planner_node stays a separately, manually-
invoked CLI tool: `ros2 run llm llm_planner_node "<command>" [--dry-run|--confirm]`,
never through this launch file or the component-supervisor/registry system.

One consequence, flagged rather than silently accepted: llm_url (the port
llm_planner_node's own get_plan_from_llm() talks to) was previously injected here
at launch time from models.yaml's own resolved port -- one source of truth. With
no Node action left to inject it into, llm_planner_node.py's own hardcoded
module-level LLAMA_URL default (currently http://127.0.0.1:8083/completion,
matching models.yaml's qwen25_3b_instruct entry) is now the ONLY source for a
bare `ros2 run` invocation -- if that entry's port ever changes, LLAMA_URL needs
updating by hand there too, nothing keeps them in sync automatically any more.

llama-server is launched directly as an ExecuteProcess -- no wrapper node -- against the
binary + cwd from stack_params.yaml's llama_server_path / llama_server_cwd (two distinct
keys, not one derived from the other: llama-server's cwd needs to be the llama.cpp
project root, two directory levels above the binary itself -- see stack_params.yaml's
llama_server_cwd comment for why a simple dirname() of the binary path would be wrong;
that tree holds the built server, NOT /home/fabiocar/projects/llm/, which only holds the
.gguf model files). Everything model-specific (-m path, --port, extra CLI flags like
-ngl) comes from config/models.yaml, keyed by the `model` launch argument, itself
resolved against the `interrogation` launch argument's own default_model (see
_launch_setup()'s own model/interrogation pairing comment) -- interrogations.yaml's
`executable`/`params_file` fields are no longer consumed by this file at all (nothing
here launches a Node to pass them to any more); still validated/looked-up for
default_model resolution and left in the schema for whenever/if node-auto-launch is
deliberately reintroduced, not read for anything else today.

An OpaqueFunction is required (rather than pure substitutions) because the model and
interrogation to launch are resolved from YAML file contents at launch time, not just
string-substituted launch arguments.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    llm_share = get_package_share_directory('llm')

    with open(os.path.join(llm_share, 'config', 'models.yaml')) as f:
        models = yaml.safe_load(f) or {}
    with open(os.path.join(llm_share, 'config', 'interrogations.yaml')) as f:
        interrogations = yaml.safe_load(f) or {}

    interrogation_name = LaunchConfiguration('interrogation').perform(context)
    if interrogation_name not in interrogations:
        raise RuntimeError(
            f"Unknown interrogation '{interrogation_name}' -- available: "
            f'{sorted(interrogations)} (see config/interrogations.yaml)')
    interrogation_config = interrogations[interrogation_name]

    # model/interrogation pairing fix: `model` defaults to '' (see model_la's
    # own default_value below) -- an EXPLICIT `model:=<name>` on the CLI
    # always wins regardless of interrogation, but leaving it unset now
    # derives the right model FROM the chosen interrogation
    # (interrogation_config['default_model'], see interrogations.yaml's own
    # comment) instead of silently falling back to whatever the `model`
    # launch argument's own hardcoded default used to be -- previously that
    # was ALWAYS qwen_mpc_pruned regardless of interrogation, so picking a
    # non-mpc_tuner interrogation with no explicit model override served it
    # with the wrong model with no error at all. An interrogation entry with
    # no default_model of its own (not expected today -- the one entry that
    # exists sets one -- but a future addition could omit it) falls back to
    # stack_params.yaml's own `model` key, the sole global default before
    # this fix, with a clear warning rather than a silent RuntimeError deep
    # in the "unknown model" check below.
    model_name = LaunchConfiguration('model').perform(context)
    model_explicit = bool(model_name)
    if not model_explicit:
        model_name = interrogation_config.get('default_model')
        if not model_name:
            model_name, _ = get_default('model')
            print(
                f"[llm.launch] WARNING: interrogation '{interrogation_name}' has no "
                f"default_model in interrogations.yaml -- falling back to stack_params."
                f"yaml's global model default ('{model_name}'). Add a default_model entry "
                'for this interrogation, or pass model:=<name> explicitly.'
            )
    if model_name not in models:
        raise RuntimeError(
            f"Unknown model '{model_name}' -- available: {sorted(models)} "
            '(see config/models.yaml)')
    model_config = models[model_name]
    model_path = os.path.expanduser(model_config['model_path'])
    port = model_config['port']
    extra_args = model_config.get('extra_args', [])

    start_server = LaunchConfiguration('start_server').perform(context).lower() == 'true'
    llama_server_bin = LaunchConfiguration('llama_server_path').perform(context)
    llama_server_cwd = LaunchConfiguration('llama_server_cwd').perform(context)

    server_process = ExecuteProcess(
        cmd=[
            llama_server_bin,
            '-m', model_path,
            '--port', str(port),
            *extra_args,
        ],
        cwd=llama_server_cwd,
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_server')),
    )

    return [
        LogInfo(msg=(
            f"[llm.launch] model='{model_name}' "
            f"({'explicit' if model_explicit else 'auto-selected for this interrogation'}) "
            f"({model_path}, port={port}) "
            f"interrogation='{interrogation_name}' "
            f"({interrogation_config['executable']}, not launched by this file -- "
            f'run it by hand) -- '
            f'server {"starting now" if start_server else "assumed already running"}.'
        )),
        server_process,
    ]


def generate_launch_description():
    start_server_default, start_server_desc = get_default('start_server')
    start_server_la = DeclareLaunchArgument(
        'start_server', default_value=str(start_server_default), description=start_server_desc)
    # Default is '' (NOT stack_params.yaml's own model default), a sentinel
    # meaning "auto-select from the chosen interrogation's own default_model
    # (see config/interrogations.yaml)" -- see _launch_setup()'s own
    # model/interrogation pairing comment for why. Passing model:=<name>
    # explicitly always overrides the auto-selection regardless of
    # interrogation, same as before this fix.
    _, model_desc = get_default('model')
    model_la = DeclareLaunchArgument(
        'model', default_value='',
        description=(
            f'{model_desc} Leave unset (default) to auto-select from the chosen '
            "interrogation's own default_model in config/interrogations.yaml."))
    interrogation_default, interrogation_desc = get_default('interrogation')
    interrogation_la = DeclareLaunchArgument(
        'interrogation', default_value=str(interrogation_default),
        description=interrogation_desc)
    llama_server_path_default, llama_server_path_desc = get_default('llama_server_path')
    llama_server_path_la = DeclareLaunchArgument(
        'llama_server_path', default_value=str(llama_server_path_default),
        description=llama_server_path_desc)
    llama_server_cwd_default, llama_server_cwd_desc = get_default('llama_server_cwd')
    llama_server_cwd_la = DeclareLaunchArgument(
        'llama_server_cwd', default_value=str(llama_server_cwd_default),
        description=llama_server_cwd_desc)

    return LaunchDescription([
        start_server_la,
        model_la,
        interrogation_la,
        llama_server_path_la,
        llama_server_cwd_la,
        OpaqueFunction(function=_launch_setup),
    ])
