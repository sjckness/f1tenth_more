"""
LLM2 (two-llm-arch): the second llama-server and llm2_mpc_param_node.

LLM1 is the mission translator (llm_planner_node, a CLI tool) whose server
llm.launch.py starts on its models.yaml port (8083) through the supervisor's
'intelligence' component. LLM2 is the MPC-parameter interrogation
(interrogations.yaml's llm2_mpc_params): a long-running node that asks its own
server for an MPC parameter set and publishes it on /llm2/mpc_params. This
file starts exactly those two processes and nothing else -- it never touches
LLM1's server.

Included by stack_bringup.launch.py only when enable_llm2 is true (default
false), so with the default the stack launches exactly as before. It can also
be started on its own next to supervisor_bringup.launch.py, which has no
enable_llm2:
  ros2 launch llm llm2.launch.py [llm2_port:=8084] [llm2_model:=...]

The server is launched the same way llm.launch.py launches LLM1's
(ExecuteProcess, binary and cwd from stack_params.yaml's llama_server_path/
llama_server_cwd, -m/extra_args from models.yaml) with two differences: the
model comes from llm2_model, and the port from llm2_port rather than from the
models.yaml entry. The same port is handed to the node as server_url, so the
two cannot disagree.

An OpaqueFunction for the same reason as llm.launch.py: the command line is
built from YAML file contents at launch time.
"""

import os

from ament_index_python.packages import get_package_share_directory

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml

#: interrogations.yaml key this file launches.
INTERROGATION = 'llm2_mpc_params'
#: interrogations.yaml key of LLM1, whose server port llm2_port must not reuse.
LLM1_INTERROGATION = 'planner'


def _launch_setup(context, *args, **kwargs):
    llm_share = get_package_share_directory('llm')

    with open(os.path.join(llm_share, 'config', 'models.yaml')) as f:
        models = yaml.safe_load(f) or {}
    with open(os.path.join(llm_share, 'config', 'interrogations.yaml')) as f:
        interrogations = yaml.safe_load(f) or {}
    interrogation_config = interrogations[INTERROGATION]

    model_name = LaunchConfiguration('llm2_model').perform(context)
    if model_name not in models:
        raise RuntimeError(
            f"Unknown llm2_model '{model_name}' -- available: {sorted(models)} "
            '(see config/models.yaml)')
    model_config = models[model_name]
    model_path = os.path.expanduser(model_config['model_path'])
    extra_args = model_config.get('extra_args', [])
    port = int(LaunchConfiguration('llm2_port').perform(context))

    llama_server_bin = LaunchConfiguration('llama_server_path').perform(context)
    llama_server_cwd = LaunchConfiguration('llama_server_cwd').perform(context)

    actions = [LogInfo(msg=(
        f"[llm2.launch] model='{model_name}' ({model_path}, port={port}) "
        f"node={interrogation_config['executable']} "
        f'server_url=http://127.0.0.1:{port}'))]

    # A second server on LLM1's port cannot bind while LLM1 runs, and the
    # node would then quietly talk to LLM1's server. A warning, not an error:
    # with LLM1 off (enable_intelligence:=false) the port is free.
    llm1_model = models.get(interrogations.get(LLM1_INTERROGATION, {}).get('default_model'))
    if llm1_model is not None and int(llm1_model['port']) == port:
        actions.append(LogInfo(msg=(
            f"[llm2.launch] WARNING: llm2_port {port} is LLM1's port "
            f"({LLM1_INTERROGATION}'s server). If LLM1 is running, the LLM2 "
            'server cannot bind and llm2_mpc_param_node will talk to LLM1.')))

    actions.append(ExecuteProcess(
        cmd=[
            llama_server_bin,
            '-m', model_path,
            '--port', str(port),
            *extra_args,
        ],
        cwd=llama_server_cwd,
        name='llama_server_llm2',
        output='screen',
    ))
    actions.append(Node(
        package='llm',
        executable=interrogation_config['executable'],
        name=interrogation_config['executable'],
        output='screen',
        parameters=[
            os.path.join(llm_share, 'config', interrogation_config['params_file']),
            {'server_url': f'http://127.0.0.1:{port}'},
        ],
    ))
    return actions


def generate_launch_description():
    llm2_model_default, llm2_model_desc = get_default('llm2_model')
    llm2_model_la = DeclareLaunchArgument(
        'llm2_model', default_value=str(llm2_model_default), description=llm2_model_desc)
    llm2_port_default, llm2_port_desc = get_default('llm2_port')
    llm2_port_la = DeclareLaunchArgument(
        'llm2_port', default_value=str(llm2_port_default), description=llm2_port_desc)
    llama_server_path_default, llama_server_path_desc = get_default('llama_server_path')
    llama_server_path_la = DeclareLaunchArgument(
        'llama_server_path', default_value=str(llama_server_path_default),
        description=llama_server_path_desc)
    llama_server_cwd_default, llama_server_cwd_desc = get_default('llama_server_cwd')
    llama_server_cwd_la = DeclareLaunchArgument(
        'llama_server_cwd', default_value=str(llama_server_cwd_default),
        description=llama_server_cwd_desc)

    return LaunchDescription([
        llm2_model_la,
        llm2_port_la,
        llama_server_path_la,
        llama_server_cwd_la,
        OpaqueFunction(function=_launch_setup),
    ])
