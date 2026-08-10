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

    model_name = LaunchConfiguration('model').perform(context)
    if model_name not in models:
        raise RuntimeError(
            f"Unknown model '{model_name}' -- available: {sorted(models)} "
            '(see config/models.yaml)')
    model_config = models[model_name]
    model_path = os.path.expanduser(model_config['model_path'])
    port = model_config['port']
    extra_args = model_config.get('extra_args', [])

    interrogation_name = LaunchConfiguration('interrogation').perform(context)
    if interrogation_name not in interrogations:
        raise RuntimeError(
            f"Unknown interrogation '{interrogation_name}' -- available: "
            f'{sorted(interrogations)} (see config/interrogations.yaml)')
    interrogation_config = interrogations[interrogation_name]
    params_file = os.path.join(
        llm_share, 'config', interrogation_config['params_file'])

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
        # params_file sets for mpc_url/target_node.
        parameters=[params_file, {
            'mpc_url': f'http://127.0.0.1:{port}/completion',
            'target_node': mpc_node_name,
        }],
    )

    return [
        LogInfo(msg=(
            f"[llm.launch] model='{model_name}' ({model_path}, port={port}) "
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
    model_default, model_desc = get_default('model')
    model_la = DeclareLaunchArgument(
        'model', default_value=str(model_default), description=model_desc)
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
