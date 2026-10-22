"""Pure helpers for yolo_detector_node's CPU-cost work (no rclpy import).

- rate_gate: the stamp-driven frame drop at max_rate_hz.
- set_cuda_blocking_sync / cuda_primary_ctx_flags: CU_CTX_SCHED_BLOCKING_SYNC
  on the device's primary context through the CUDA driver API (ctypes), so a
  thread waiting for the GPU sleeps instead of spinning a core.
- engine_path_for: the TensorRT engine file name the node looks for and
  scripts/export_yolo_engine.sh writes.

torch is imported inside the functions that need it, so this module imports
without torch (the node's tests replace torch with a fake module).
"""

import ctypes
import os

# cuda.h: CUcontext scheduling flags.
CU_CTX_SCHED_SPIN = 0x01
CU_CTX_SCHED_YIELD = 0x02
CU_CTX_SCHED_BLOCKING_SYNC = 0x04
CU_CTX_SCHED_MASK = 0x07

# Fraction of the period a frame may arrive early and still pass the gate:
# camera stamps jitter by a few ms, and without it a 30 Hz stream gated to
# 15 Hz would drop to 10 Hz whenever the second frame lands just short.
RATE_GATE_TOLERANCE = 0.1


def rate_gate(stamp_sec, next_due_sec, period_sec, tolerance=RATE_GATE_TOLERANCE):
    """
    Return (process, next_due) for the frame drop at max_rate_hz.

    perception_debug.tick_due semantics (the deadline advances from the
    previous deadline, resets after a gap or a backwards stamp) plus an early
    tolerance of `tolerance` * period. period_sec <= 0 lets every frame pass.
    """
    if period_sec <= 0.0:
        return True, next_due_sec
    if (next_due_sec is None or stamp_sec - next_due_sec > period_sec
            or stamp_sec < next_due_sec - period_sec):
        return True, stamp_sec + period_sec
    if stamp_sec + tolerance * period_sec >= next_due_sec:
        return True, next_due_sec + period_sec
    return False, next_due_sec


def _libcuda():
    return ctypes.CDLL('libcuda.so.1')


def set_cuda_blocking_sync(device_index=0):
    """
    Set CU_CTX_SCHED_BLOCKING_SYNC on the primary context of device_index.

    Call before torch creates its CUDA context (the first CUDA op). Returns
    (ok, message); never raises.
    """
    try:
        cuda = _libcuda()
        dev = ctypes.c_int()
        rc = cuda.cuInit(0)
        if rc != 0:
            return False, f'cuInit returned {rc}'
        rc = cuda.cuDeviceGet(ctypes.byref(dev), int(device_index))
        if rc != 0:
            return False, f'cuDeviceGet({device_index}) returned {rc}'
        set_flags = getattr(cuda, 'cuDevicePrimaryCtxSetFlags_v2', None) \
            or cuda.cuDevicePrimaryCtxSetFlags
        rc = set_flags(dev, ctypes.c_uint(CU_CTX_SCHED_BLOCKING_SYNC))
        if rc != 0:
            return False, f'cuDevicePrimaryCtxSetFlags returned {rc}'
        return True, 'CU_CTX_SCHED_BLOCKING_SYNC set'
    except Exception as exc:  # noqa: BLE001 - optional optimisation
        return False, str(exc)


def cuda_primary_ctx_flags(device_index=0):
    """Return (flags, active) of the primary context, or (None, None)."""
    try:
        cuda = _libcuda()
        dev, flags, active = ctypes.c_int(), ctypes.c_uint(), ctypes.c_int()
        if cuda.cuDeviceGet(ctypes.byref(dev), int(device_index)) != 0:
            return None, None
        if cuda.cuDevicePrimaryCtxGetState(dev, ctypes.byref(flags), ctypes.byref(active)) != 0:
            return None, None
        return int(flags.value), bool(active.value)
    except Exception:  # noqa: BLE001 - diagnostics only
        return None, None


def cuda_device_index(device):
    """'cuda' / 'cuda:1' / '0' -> device index, None for anything else (cpu)."""
    device = str(device).strip().lower()
    if device == 'cuda':
        return 0
    if device.startswith('cuda:'):
        device = device[len('cuda:'):]
    return int(device) if device.isdigit() else None


def engine_path_for(model_path, trt_version):
    """<dir>/<stem>.trt<trt_version>.fp16.engine next to model_path."""
    stem = os.path.splitext(os.path.basename(model_path))[0]
    return os.path.join(os.path.dirname(model_path), f'{stem}.trt{trt_version}.fp16.engine')
