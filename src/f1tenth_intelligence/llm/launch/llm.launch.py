"""llama-server bringup + one LLM "interrogation" node (see config/interrogations.yaml).

llama-server is launched directly as an ExecuteProcess -- no wrapper node -- against the
binary + cwd from stack_params.yaml's llama_server_path / llama_server_cwd (two distinct
keys, not one derived from the other: llama-server's cwd needs to be the llama.cpp
project root, two directory levels above the binary itself -- see stack_params.yaml's
llama_server_cwd comment for why a simple dirname() of the binary path would be wrong;
that tree holds the built server, NOT /home/fabiocar/projects/llm/, which only holds the
.gguf model files). Everything model-specific (-m path, --port, extra CLI flags like
-ngl) comes from config/models.yaml, keyed by the `model` launch argument.

The interrogation node (e.g. llm_mpc_tuner_node) is launched unconditionally, regardless
of start_server, in case a server is already running standalone elsewhere on the Jetson.
Its mpc_url parameter is built here from the selected model's resolved port (models.yaml)
and injected as an override on top of its params_file, rather than being duplicated in
config/mpc_tuner_params.yaml -- one source of truth for the port. target_node is injected
the same way, from stack_params.yaml's mpc_node_name -- one source of truth for the MPC
node name (see that key's own comment); harmless no-op if a future non-MPC interrogation
doesn't declare a target_node parameter at all.

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
from launch_ros.actions import Node


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
    params_file = os.path.join(
        llm_share, 'config', interrogation_config['params_file'])

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
    # no default_model of its own (not expected today -- both existing
    # entries set one -- but a future addition could omit it) falls back to
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
    mpc_node_name = LaunchConfiguration('mpc_node_name').perform(context)

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

    interrogation_node = Node(
        package='llm',
        executable=interrogation_config['executable'],
        name=interrogation_config['executable'],
        output='screen',
        # params_file first, override dict second: launch_ros applies later entries'
        # keys over earlier ones, so this always wins over anything (or nothing) the
        # params_file sets for mpc_url/target_node/llm_url. Applied uniformly to
        # EVERY interrogation node regardless of which of these params it actually
        # declares -- ROS silently ignores an undeclared parameter override, so this
        # stays a harmless no-op for whichever ones a given node doesn't read (e.g.
        # llm_mpc_tuner_node doesn't declare llm_url, llm_planner_node doesn't
        # declare mpc_url/target_node), same as before this pass.
        parameters=[params_file, {
            'mpc_url': f'http://127.0.0.1:{port}/completion',
            'target_node': mpc_node_name,
            # llm_planner_node's own equivalent of mpc_url above -- named
            # differently since "mpc_url" is specifically an llm_mpc_tuner_node
            # concept (it targets an MPC controller node), not a generic name
            # this interrogation-agnostic override dict should imply for every
            # future interrogation. Resolved from the SAME `port` this
            # interrogation's own model_name/model_config picked, so a
            # get_plan_from_llm() call always reaches whichever llama-server was
            # actually started for it, not a stale/hardcoded port.
            'llm_url': f'http://127.0.0.1:{port}/completion',
        }],
    )

    return [
        LogInfo(msg=(
            f"[llm.launch] model='{model_name}' "
            f"({'explicit' if model_explicit else 'auto-selected for this interrogation'}) "
            f"({model_path}, port={port}) "
            f"interrogation='{interrogation_name}' "
            f"({interrogation_config['executable']}) -- "
            f'server {"starting now" if start_server else "assumed already running"}.'
        )),
        server_process,
        interrogation_node,
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
    mpc_node_name_default, mpc_node_name_desc = get_default('mpc_node_name')
    mpc_node_name_la = DeclareLaunchArgument(
        'mpc_node_name', default_value=str(mpc_node_name_default),
        description=mpc_node_name_desc)

    return LaunchDescription([
        start_server_la,
        model_la,
        interrogation_la,
        llama_server_path_la,
        llama_server_cwd_la,
        mpc_node_name_la,
        OpaqueFunction(function=_launch_setup),
    ])
