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

    def test_default_launch_stays_on_tensorrt_box_detector_unchanged(self):
        # No overrides at all -- must reproduce exactly what every launch
        # before this pass already did: the deployed TensorRT engine,
        # box-region depth sampling. This is the "default stays off"
        # guardrail's own regression coverage.
        resolved = _resolve({})
        assert resolved['yolo_model'] == 'yolo26s.engine'
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
