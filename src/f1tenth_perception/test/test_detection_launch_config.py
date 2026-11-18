"""detection.launch.py config-parsing tests -- same "no live launch, direct
Python construction of generate_launch_description() + a LaunchContext"
convention f1tenth_localization/test/test_ekf_global_config.py already uses
(see that file's own module docstring). No rclpy Node, no hardware, no GPU,
no model weights -- purely resolves DeclareLaunchArgument default_value
substitutions against a LaunchContext, exactly what `ros2 launch` itself
does before spawning any process.

Scope: the yolo-seg live-deployable pass's "single switch" requirement --
selecting a seg-family yolo_model (e.g. yolo26s-seg.pt) must automatically
flip use_mask_depth on too, rather than requiring Andreas to remember two
independent launch args for one capability (see detection.launch.py's own
comment at the use_mask_depth_la declaration for the full rationale). This
is real launch-time behavior (PythonExpression evaluated by the actual
launch system against actual argument overrides) that test_detection_3d_
node.py/test_yolo_detector_node.py's node-level, no-launch-file-involved
tests cannot cover -- and the one thing the real-GPU
scripts/verify_seg_live_deploy.py dev-tool script (also new this pass, see
its own docstring) does NOT re-check on every run, since it hardcodes a
seg model_path directly rather than going through this launch file at all.
Together the three files cover: launch-arg wiring (here, fast, no hardware)
+ node-callback wiring (existing 21 tests, fast, mocked Ultralytics/torch)
+ real end-to-end GPU inference (verify_seg_live_deploy.py, slow, needs a
GPU and the actual weights file -- a manual dev tool, not part of this
suite, same convention benchmark_yolo_latency.py already established).

Run standalone: python3 -m pytest test/test_detection_launch_config.py -v
"""

import importlib.util
import os

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
_LAUNCH_DIR = os.path.join(os.path.dirname(_THIS_DIR), 'launch')


def _load_detection_launch():
    spec = importlib.util.spec_from_file_location(
        'detection_launch', os.path.join(_LAUNCH_DIR, 'detection.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _resolve(overrides):
    """Builds the real LaunchDescription and resolves every declared
    argument's default against a LaunchContext seeded with `overrides` --
    exactly what `ros2 launch f1tenth_perception detection.launch.py
    key:=value ...` resolves at startup, before any Node actually spawns."""
    mod = _load_detection_launch()
    launch_description = mod.generate_launch_description()
    ctx = LaunchContext()
    for key, value in overrides.items():
        ctx.launch_configurations[key] = value
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)
    return ctx.launch_configurations


class TestUseMaskDepthAutoDerivation:
    """See detection.launch.py's own comment at use_mask_depth_la -- this is
    the "single switch, not two" requirement's actual coverage."""

    def test_default_launch_is_the_seg_model_with_mask_depth_auto_on(self):
        # No overrides at all: the deployed default is the seg checkpoint
        # (stack_params.yaml's yolo_model/yolo_model_task, flipped once the
        # seg path was judged live-ready and used by every recorded run since
        # 2026-09-02), and use_mask_depth therefore auto-derives ON with no
        # second argument.
        #
        # This assertion previously read 'yolo26s.engine' / 'False' and was
        # WRONG FROM THE DAY IT WAS WRITTEN: the test and the seg default
        # landed in the same commit (96c6bbc) contradicting each other, and
        # nothing caught it for two days because colcon never executed this
        # file at all (no extras_require 'test' extra -- fixed in aaae377).
        # Kept explicit here because a stale expected-value is exactly what
        # this test now exists to catch.
        resolved = _resolve({})
        assert resolved['yolo_model'] == 'yolo26s-seg.pt'
        assert resolved['yolo_model_task'] == 'segment'
        assert resolved['use_mask_depth'] == 'True'

    def test_selecting_the_tensorrt_box_engine_auto_derives_mask_depth_off(self):
        # The other direction of the same rule, and the coverage the old
        # default-case assertion was really providing before the default
        # moved out from under it: the deployed TensorRT box engine has no
        # masks to decode, so mask-based fusion must auto-derive OFF for it
        # without needing a second argument either.
        resolved = _resolve({'yolo_model': 'yolo26s.engine'})
        assert resolved['use_mask_depth'] == 'False'

    def test_selecting_the_seg_model_alone_turns_on_mask_depth(self):
        # The one launch arg Andreas needs for the seg live test -- see this
        # pass's own final report.
        resolved = _resolve({'yolo_model': 'yolo26s-seg.pt'})
        assert resolved['use_mask_depth'] == 'True'

    def test_selecting_a_different_seg_family_filename_also_turns_it_on(self):
        # Generalizes on the '-seg' substring, not a hardcoded filename --
        # confirms this isn't special-cased to only yolo26s-seg.pt.
        resolved = _resolve({'yolo_model': 'yoloe-26s-seg.pt'})
        assert resolved['use_mask_depth'] == 'True'

    def test_a_non_seg_pt_model_does_not_turn_it_on(self):
        resolved = _resolve({'yolo_model': 'yolo26s.pt'})
        assert resolved['use_mask_depth'] == 'False'

    def test_explicit_override_wins_over_the_seg_model_auto_derivation(self):
        # Andreas can still decouple mask-based fusion from model selection
        # if he ever wants to -- see the use_mask_depth_la description.
        resolved = _resolve({'yolo_model': 'yolo26s-seg.pt', 'use_mask_depth': 'false'})
        assert resolved['use_mask_depth'] == 'false'

    def test_explicit_override_can_also_force_it_on_for_a_non_seg_model(self):
        resolved = _resolve({'yolo_model': 'yolo26s.engine', 'use_mask_depth': 'true'})
        assert resolved['use_mask_depth'] == 'true'

    def test_yolo_device_defaults_to_cuda_already_no_second_arg_needed(self):
        # Confirms Andreas's live-test command really is just
        # yolo_model:=yolo26s-seg.pt -- yolo_device already defaults to cuda
        # stack-wide (f1tenth_params/config/stack_params.yaml), so it does
        # not need to be passed alongside yolo_model for a GPU seg run.
        resolved = _resolve({'yolo_model': 'yolo26s-seg.pt'})
        assert resolved['yolo_device'] == 'cuda'


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))


class TestObstacleRadiusLaunchArgs:
    """obstacle_radius_source and obstacle_class_margin_m reach the projector
    from stack_params.yaml; the margin map travels as a JSON string."""

    def test_defaults_are_footprint_and_a_person_margin(self):
        config = _resolve({})
        assert config['obstacle_radius_source'] == 'footprint'
        assert config['obstacle_class_margin_m'] == '{"person": 0.5}'

    def test_overrides_pass_through(self):
        config = _resolve({'obstacle_radius_source': 'legacy',
                           'obstacle_class_margin_m': '{"person": 0.3}'})
        assert config['obstacle_radius_source'] == 'legacy'
        assert config['obstacle_class_margin_m'] == '{"person": 0.3}'


class TestDebugImageLaunchArgs:
    """The debug image's four knobs and the wall payload switch come from stack_params.yaml."""

    def test_debug_image_defaults_are_5hz_480px_quality_50_floor_0_1(self):
        config = _resolve({})
        assert config['debug_image_rate_hz'] == '5.0'
        assert config['debug_image_width'] == '480'
        assert config['debug_image_jpeg_quality'] == '50'
        assert config['debug_image_min_confidence'] == '0.1'

    def test_front_wall_debug_payload_is_on_by_default(self):
        assert _resolve({})['front_clearance_publish_debug_wall'] == 'True'


def _spawned(overrides):
    """
    Map executable -> (Node, its resolved additional_env) for every Node launched.

    OpaqueFunctions are executed against the resolved context.
    """
    from launch.actions import OpaqueFunction
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node

    mod = _load_detection_launch()
    launch_description = mod.generate_launch_description()
    ctx = LaunchContext()
    for key, value in overrides.items():
        ctx.launch_configurations[key] = value
    nodes = []
    for entity in launch_description.entities:
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
        out[node.node_executable] = (node, env)
    return out


_FOUR = ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
         'OPENCV_FOR_THREADS_NUM')


class TestThreadEnv:
    """R3 (2026-10-23): front_clearance_cpu_threads, the yolo_cpu_threads pattern.

    The default env of every node is the one it had before: yolo's four
    variables at yolo_cpu_threads (2), front_clearance's single
    OPENBLAS_NUM_THREADS=1 (2026-10-13), nothing for the rest.
    """

    def test_the_default_envs_are_unchanged(self):
        spawned = _spawned({})
        assert spawned['yolo_detector_node'][1] == {var: '2' for var in _FOUR}
        assert spawned['front_clearance_node'][1] == {'OPENBLAS_NUM_THREADS': '1'}
        assert spawned['detection_3d_node'][1] == {}
        assert spawned['obstacle_projector_node'][1] == {}

    def test_front_clearance_cpu_threads_defaults_to_zero(self):
        assert _resolve({})['front_clearance_cpu_threads'] == '0'

    def test_front_clearance_cpu_threads_sets_all_four(self):
        spawned = _spawned({'front_clearance_cpu_threads': '3'})
        assert spawned['front_clearance_node'][1] == {var: '3' for var in _FOUR}
        # Only that node's env moves.
        assert spawned['yolo_detector_node'][1] == {var: '2' for var in _FOUR}

    def test_yolo_cpu_threads_zero_is_still_unset(self):
        spawned = _spawned({'yolo_cpu_threads': '0'})
        assert spawned['yolo_detector_node'][1] == {}
        assert spawned['front_clearance_node'][1] == {'OPENBLAS_NUM_THREADS': '1'}

    def test_cpu_threads_is_not_passed_as_a_node_parameter(self):
        node = _spawned({'front_clearance_cpu_threads': '3'})['front_clearance_node'][0]
        names = set()
        for params in node._Node__parameters:   # normalized: tuples of TextSubstitution
            names.update(''.join(sub.text for sub in key) for key in params)
        assert 'nice' in names and 'depth_topic' in names
        assert not any('cpu_threads' in n for n in names)
