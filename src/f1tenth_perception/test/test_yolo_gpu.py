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
