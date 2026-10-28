"""costmap.launch.py: semantic_layer_node's thread env (semantic_layer_cpu_threads).

R3 (2026-10-23), the yolo_cpu_threads pattern of f1tenth_perception's
detection.launch.py: an OpaqueFunction sets OMP_NUM_THREADS / MKL_NUM_THREADS /
OPENBLAS_NUM_THREADS / OPENCV_FOR_THREADS_NUM of semantic_layer_node's process
to N for N > 0. The default, 0, sets none of them: the env the node had before.

No live launch: the LaunchDescription is built and resolved against a
LaunchContext, the same convention as test_detection_launch_config.py.

Run standalone: python3 -m pytest test/test_costmap_launch_threads.py -v
"""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

_LAUNCH = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
                       'launch', 'costmap.launch.py')
_FOUR = ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
         'OPENCV_FOR_THREADS_NUM')


def _spawned(overrides):
    spec = importlib.util.spec_from_file_location('costmap_launch', _LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    ctx.launch_configurations.update(overrides)
    nodes = []
    for entity in mod.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)
        elif isinstance(entity, OpaqueFunction):
            nodes.extend(entity.execute(ctx))
        elif isinstance(entity, Node):
            nodes.append(entity)
    out = {}
    for node in nodes:
        env = {perform_substitutions(ctx, k): perform_substitutions(ctx, v)
               for k, v in (node.additional_env or [])}
        out[node.node_executable] = (node, env, ctx)
    return out


def test_the_default_sets_no_thread_variable():
    spawned = _spawned({})
    assert set(spawned) == {'semantic_layer_node', 'costmap_renderer_node',
                            'costmap_boundary_node', 'freespace_door_node'}
    for _, env, _ctx in spawned.values():
        assert env == {}
    _, _, ctx = spawned['semantic_layer_node']
    assert ctx.launch_configurations['semantic_layer_cpu_threads'] == '0'


def test_n_sets_all_four_for_the_semantic_layer_only():
    spawned = _spawned({'semantic_layer_cpu_threads': '2'})
    assert spawned['semantic_layer_node'][1] == {var: '2' for var in _FOUR}
    assert spawned['costmap_boundary_node'][1] == {}
    assert spawned['costmap_renderer_node'][1] == {}
    assert spawned['freespace_door_node'][1] == {}


def test_the_semantic_layer_keeps_its_enable_slam_gate():
    node, _, ctx = _spawned({'enable_slam': 'false'})['semantic_layer_node']
    assert node.condition is not None
    assert node.condition.evaluate(ctx) is False
    node, _, ctx = _spawned({})['semantic_layer_node']
    assert node.condition.evaluate(ctx) is True


def test_the_grid_door_detector_is_gated_on_enable_slam_too():
    """Same gate as the other three nodes, for the same reason: it reads
    /slam/map, so with SLAM off it would be an idle subscriber, not a
    meaningful independent on/off state."""
    node, _, ctx = _spawned({'enable_slam': 'false'})['freespace_door_node']
    assert node.condition is not None
    assert node.condition.evaluate(ctx) is False
    node, _, ctx = _spawned({})['freespace_door_node']
    assert node.condition.evaluate(ctx) is True


def _param_names(node, ctx):
    """The parameter NAMES a launch_ros Node was given, resolved.

    Reaches into Node's own name-mangled __parameters because launch_ros
    exposes no public accessor before the node is actually launched, and the
    keys are substitution tuples rather than plain strings.
    """
    out = set()
    for entry in (node._Node__parameters or ()):
        if isinstance(entry, dict):
            out.update(perform_substitutions(ctx, list(k)) for k in entry)
    return out


def test_the_grid_door_detector_gets_the_shared_door_source_argument():
    """door_source must reach every door-owning node with the SAME value.

    One launch argument, two nodes here (the third detector,
    lidar_front_wall_node, is in f1tenth_perception's own launch file). If one
    of them stopped receiving it, two detectors could publish doors at once --
    the single thing door_source exists to prevent.
    """
    spawned = _spawned({'door_source': 'profile'})
    for name in ('semantic_layer_node', 'freespace_door_node'):
        node, _env, ctx = spawned[name]
        assert 'door_source' in _param_names(node, ctx), name
        assert ctx.launch_configurations['door_source'] == 'profile'


def test_the_stack_params_default_is_zero():
    from f1tenth_params.param_defaults import get_value
    assert get_value('semantic_layer_cpu_threads') == 0
    assert get_value('front_clearance_cpu_threads') == 0
