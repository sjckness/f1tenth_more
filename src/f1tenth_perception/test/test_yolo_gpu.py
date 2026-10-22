"""Unit tests for f1tenth_perception.yolo_gpu (pure helpers of yolo_detector_node)."""

from f1tenth_perception import yolo_gpu


def _passed(stamps, period):
    next_due, out = None, []
    for stamp in stamps:
        ok, next_due = yolo_gpu.rate_gate(stamp, next_due, period)
        out.append(ok)
    return out


class TestRateGate:

    def test_30hz_input_gated_to_15hz_passes_every_second_frame(self):
        passed = _passed([i / 30.0 for i in range(300)], 1.0 / 15.0)
        assert sum(passed) == 150
        assert passed[:6] == [True, False, True, False, True, False]

    def test_15hz_input_with_jitter_passes_every_frame(self):
        jitter = [0.002, -0.002, 0.0015, -0.0018, 0.0]
        stamps = [i / 15.0 + jitter[i % len(jitter)] for i in range(150)]
        assert all(_passed(stamps, 1.0 / 15.0))

    def test_gap_and_backwards_stamp_reset_the_deadline(self):
        period = 1.0 / 15.0
        ok, due = yolo_gpu.rate_gate(10.0, None, period)
        assert ok
        ok, due = yolo_gpu.rate_gate(20.0, due, period)  # 10 s gap
        assert ok and abs(due - (20.0 + period)) < 1e-9
        ok, due = yolo_gpu.rate_gate(5.0, due, period)  # replayed bag
        assert ok and abs(due - (5.0 + period)) < 1e-9

    def test_non_positive_period_passes_everything(self):
        assert all(_passed([0.0, 0.0, 0.001], 0.0))


class TestEnginePath:

    def test_engine_name_carries_the_tensorrt_version(self):
        assert yolo_gpu.engine_path_for('/m/yolo26s-seg.pt', '10.3.0') == \
            '/m/yolo26s-seg.trt10.3.0.fp16.engine'


class TestCudaDeviceIndex:

    def test_device_strings(self):
        assert yolo_gpu.cuda_device_index('cuda') == 0
        assert yolo_gpu.cuda_device_index('cuda:1') == 1
        assert yolo_gpu.cuda_device_index('0') == 0
        assert yolo_gpu.cuda_device_index('cpu') is None


class TestLetterbox:

    def test_zed_frame_pads_12_px_top_and_bottom(self):
        # .pt rect (auto, stride 32) and a static 384x640 engine (no auto).
        assert yolo_gpu.letterbox_params(360, 640, (640, 640), True, 32) == \
            ((640, 360), 12, 12, 0, 0)
        assert yolo_gpu.letterbox_params(360, 640, (384, 640), False, 32) == \
            ((640, 360), 12, 12, 0, 0)

    def test_matches_ultralytics_letterbox(self):
        import numpy as np
        import pytest
        augment = pytest.importorskip('ultralytics.data.augment')
        for (h, w) in ((360, 640), (720, 1280), (480, 640), (375, 500), (1080, 810)):
            for new_shape, auto in (((640, 640), True), ((384, 640), False), ((640, 640), False)):
                lb = augment.LetterBox(new_shape, auto=auto, stride=32)
                out = lb(image=np.zeros((h, w, 3), dtype=np.uint8))
                (uw, uh), top, bottom, left, right = yolo_gpu.letterbox_params(
                    h, w, new_shape, auto, 32)
                assert out.shape[:2] == (uh + top + bottom, uw + left + right)


class TestMaskHelpers:

    def test_label_image_later_index_wins_and_stats_count_pixels(self):
        import pytest
        torch = pytest.importorskip('torch')
        masks = torch.zeros((2, 6, 6), dtype=torch.uint8)
        masks[0, 0:4, 0:4] = 1
        masks[1, 2:6, 2:6] = 1
        label = yolo_gpu.label_image(masks)
        assert int(label[0, 0]) == 1 and int(label[3, 3]) == 2 and int(label[5, 0]) == 0
        total, inside = yolo_gpu.mask_stats(masks, (3, 3, 6, 6))
        assert total.tolist() == [16.0, 16.0]
        assert inside.tolist() == [1.0, 9.0]
        _, none_inside = yolo_gpu.mask_stats(masks, (4, 4, 4, 6))
        assert none_inside.tolist() == [0.0, 0.0]
