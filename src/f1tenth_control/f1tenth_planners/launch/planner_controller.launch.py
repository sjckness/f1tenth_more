"""One of the alternative planner-controllers: corridor_rrt or p2p_geometric.

Included by f1tenth_navigation/navigation.launch.py when planner_controller_mode
is not corridor_mpc (see f1tenth_params/planner_modes.py). Runnable alone:

  ros2 launch f1tenth_planners planner_controller.launch.py planner_controller_mode:=p2p_geometric

The node reads its parameters' defaults from stack_params.yaml itself, so
nothing needs passing here; override any of them with a params file or
`ros2 param set` like any other node.

cpu_affinity defaults to mpc_corr's '10,11': the node takes mpc_corr's place
on the Orin, so it takes its cores too. Hosts with fewer cores set it through
the same 'navigation' component argument that moves mpc_corr
(f1tenth_bringup/config/sim/hosts/linus.yaml); launch configurations reach
this included file the way they reach mpc_corr.launch.py.
"""

from f1tenth_params.planner_modes import (
    CONTROLLER_NODE_NAMES, validate_planner_controller_mode)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context):
    mode = validate_planner_controller_mode(
        LaunchConfiguration('planner_controller_mode').perform(context))
    if mode == 'corridor_mpc':
        raise ValueError('planner_controller.launch.py starts corridor_rrt or p2p_geometric; '
                         'corridor_mpc is mpc_corr (f1tenth_control mpc_corr.launch.py)')
    name = CONTROLLER_NODE_NAMES[mode]
    return [Node(
        package='f1tenth_planners',
        executable=name,
        name=name,
        output='screen',
        prefix=['taskset -c ', LaunchConfiguration('cpu_affinity')],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_controller_mode', default_value='corridor_rrt',
            description='corridor_rrt or p2p_geometric (f1tenth_params/planner_modes.py)'),
        DeclareLaunchArgument(
            'cpu_affinity', default_value='10,11',
            description="Core ids for 'taskset -c'; mpc_corr's cores, see the docstring."),
        OpaqueFunction(function=_node),
    ])
