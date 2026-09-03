#!/usr/bin/env python3
"""Standalone latency benchmark: current TensorRT box detector vs. the new
Ultralytics-runtime segmentation path (see yolo_detector_node.py's own
"Task resolution" docstring paragraph, and this repo's YOLO-seg + mask-based
depth fusion pass this file was written for).

NOT a ROS node, NOT wired into setup.py/launch -- deliberately a manual dev
tool, run directly from source:

    python3 src/f1tenth_perception/scripts/benchmark_yolo_latency.py

CUDA inference via torch was blocked for two prior sessions (both running
directly on this Jetson, not a sandboxed devcontainer) by `torch`+CUDA
failing with `RuntimeError: CUDA error: CUBLAS_STATUS_ALLOC_FAILED when
calling cublasCreate(handle)` on ANY cuBLAS-backed op (even a bare
`torch.randn(4,4,device='cuda') @ ...`), while the TensorRT *.engine path
below loaded and ran fine on GPU in the same environment (it manages its own
CUDA context outside torch/cuBLAS entirely -- see yolo_detector_node.py's
own docstring). ROOT CAUSE, found and fixed (2026-09-01), confirmed via
/proc/<pid>/maps rather than guessed: this torch 2.11.0 install's `pip`
metadata declares `nvidia-cublas-cu12==12.9.2.10` as a dependency (installed
alongside it in ~/.local/lib/python3.10/site-packages/nvidia/cublas/lib/) --
a generic PyPI aarch64 cuBLAS build, NOT the JetPack/L4T-provided Tegra-
native one (this device: JetPack 6.2.2 / L4T R36.5.0, CUDA 12.6.11,
`/usr/local/cuda-12.6/.../libcublas.so.12.6.4.1`). torch's own import-time
preload (ctypes-based, absolute-path, independent of LD_LIBRARY_PATH/RPATH)
maps the bundled 12.9.2.10 build FIRST, confirmed live via
`/proc/<pid>/maps` right after `import torch` -- `libcudart.so` correctly
maps to the system JetPack one, but `libcublas.so.12`/`libcublasLt.so.12`
map to the bundled pip one instead. That version/ABI mismatch against
Tegra's driver model is what cublasCreate() fails on (cuDNN, checked
separately via a conv2d op, is unaffected -- no bundled pip cudnn package
exists here, it correctly resolves to the system
`/usr/lib/aarch64-linux-gnu/libcudnn.so.9.3.0`).

FIX -- applied properly (2026-09-01), NOT just an LD_PRELOAD workaround
anymore: uninstalled the shadowing chain (`nvidia-cudss-cu12` ->
`cuda-toolkit` -> `nvidia-cublas-cu12` -> `nvidia-cuda-nvrtc-cu12` -- pip
reverse-dependency-checked first: nothing else installed, and no code
anywhere in this repo, references cuDSS/depends on that pinned cublas), then
reinstalled ONLY `nvidia-cudss-cu12` with `--no-deps`. Why cudss alone still
needed: `import torch` itself hard-fails with `ImportError: libcudss.so.0:
cannot open shared object file` without it -- torch's compiled `_C`
extension has a direct link-time dependency on it (undeclared in torch's own
pip metadata, only surfaces as this ImportError), and JetPack 6.2.2/CUDA
12.6.11 ships no Tegra-native cuDSS at all (checked `/usr/local/cuda-12.6/
version.json`'s full component list) -- so a pip-distributed libcudss.so.0
is the only source there is, unlike cublas which DOES have a correct
Tegra-native copy the pip one was shadowing. `--no-deps` matters: a plain
`pip install nvidia-cudss-cu12` re-pulls the exact same `cuda-toolkit` ->
`nvidia-cublas-cu12==12.9.2.10` chain that caused this bug in the first
place; `--no-deps` installs only cudss's own bundled `libcudss.so.0` (also
confirmed self-contained: it lives directly under cudss's own package dir,
not something cuda-toolkit/cublas actually had to provide). Verified via
`/proc/<pid>/maps` after this fix: `libcublas.so.12`/`libcublasLt.so.12` now
resolve to the system JetPack copy (matches the earlier ldd/RUNPATH finding
exactly), `libcudss.so.0` resolves to the (now dependency-free) pip package,
`libcudnn.so.9.3.0` unaffected as before. The raw matmul and conv2d checks
both pass with NO LD_PRELOAD needed -- this is now the actual fixed state of
the environment, not a per-invocation routing trick. One residual, harmless
`pip check` warning: `nvidia-cudss-cu12 requires cuda-toolkit, which is not
installed` -- expected and left as-is (that requirement was never actually
needed for the one file torch links against; reinstalling cuda-toolkit to
silence this warning would just reintroduce the bug). Does NOT affect the
deployed default either way: yolo26s.engine (TensorRT) never touched torch's
cuBLAS path at all (see above). IS what makes a `.pt` model_path with
device=cuda in yolo_detector_node.py actually deployable now, not just
benchmarkable from this script.

GPU numbers, now real (2026-09-01, this Jetson AGX Orin, clean environment
per the fix above -- no LD_PRELOAD, --device cuda, 1080x810 bus.jpg,
--warmup 3 --iters 15, three consecutive runs for stability, two under the
LD_PRELOAD workaround before the proper fix and one after -- all three
agree):
    yolo26s.engine (GPU, TensorRT, detect):  mean ~25.4ms (22.3-27.4ms across both runs)
    yolo26s.pt     (GPU, PyTorch, detect):   mean ~40.7ms (39.9-41.2ms across both runs)
    yolo26s-seg.pt (GPU, PyTorch, segment):  mean ~52.3ms (47.9-55.6ms across both runs)
Segment costs ~1.29x the same-runtime PyTorch detect model on GPU (52.3/40.7
-- close to the CPU-measured 1.52x ratio below, so that CPU-based estimate
was directionally right) and ~2.1x the deployed TensorRT engine (52.3/25.4).
This is the real, decision-relevant number this whole investigation was
blocked on -- see this pass's own final report for the read on whether a
TensorRT mask-decode port is worth prioritizing from here.

Earlier CPU-only numbers (kept for reference/history, NOT the
decision-relevant data anymore -- superseded by the real GPU numbers above):
1080x810 bus.jpg, --warmup 2 --iters 10, 12-core host CPU, torch on CPU
(sidesteps the cuBLAS issue entirely, which is CUDA-specific):
    yolo26s.engine (GPU, TensorRT, detect):  mean  24.05ms (min 21.92 / max 25.91)
    yolo26s.pt     (CPU, PyTorch, detect):   mean 1014.99ms (min 1009.94 / max 1018.24)
    yolo26s-seg.pt (CPU, PyTorch, segment):  mean 1542.87ms (min 1530.37 / max 1554.49)
(segment/detect ratio on CPU: 1.52x -- reasonably close to the real 1.29x
GPU ratio measured above, for what that's worth in hindsight.)

Compares, on --device (default cuda, matching the deployed default):
  * yolo26s.engine  (current default, TensorRT, task=detect)      -- if present
  * yolo26s.pt      (task=detect, same architecture as the engine, portable)
  * yolo26s-seg.pt  (task=segment, the new mask-based-fusion candidate)
on the same static test image (bus.jpg, the Ultralytics sample asset those
weights were trained against -- reliably produces real detections, unlike a
random-noise fixture; see f1tenth_perception's own test/ directory for the
synthetic-fixture unit tests this benchmark does NOT replace).

Each config: --warmup untimed passes (excluded -- CUDA context/cuDNN autotune
warmup, not representative of steady-state per-frame cost), then --iters
timed passes; reports mean/median/min/max in milliseconds.
"""

import argparse
import os
import time

import cv2
import numpy as np


def _default_image_path():
    try:
        from ultralytics.utils import ASSETS
        return str(ASSETS / 'bus.jpg')
    except Exception:  # noqa: BLE001 - fall back to a synthetic image below
        return None


def _synthetic_image():
    # Only used if the Ultralytics sample asset can't be located -- real
    # detections are NOT guaranteed on this (see module docstring), but
    # timing is still meaningful since inference cost is dominated by image
    # resolution/model architecture, not scene content.
    img = (np.random.rand(1080, 810, 3) * 255).astype(np.uint8)
    img[200:900, 200:600] = [120, 60, 60]
    return img


def bench(model_path, device, img, task=None, retina_masks=True, warmup=3, iters=15):
    from ultralytics import YOLO

    kwargs = {'task': task} if task is not None else {}
    model = YOLO(model_path, **kwargs)
    if not model_path.endswith('.engine'):
        model.to(device)

    for _ in range(warmup):
        model(img, verbose=False, device=device, conf=0.3, retina_masks=retina_masks)

    times_ms = []
    for _ in range(iters):
        t0 = time.perf_counter()
        model(img, verbose=False, device=device, conf=0.3, retina_masks=retina_masks)
        times_ms.append((time.perf_counter() - t0) * 1000.0)

    return np.array(times_ms), model.task


def _report(label, times_ms):
    print(f'{label}:')
    print(f'  mean={times_ms.mean():7.2f}ms  median={np.median(times_ms):7.2f}ms '
          f'min={times_ms.min():7.2f}ms  max={times_ms.max():7.2f}ms  (n={len(times_ms)})')


def main():
    models_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'models')

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--device', default='cuda',
        help="'cuda' (default, matches the deployed stack) or 'cpu'.")
    parser.add_argument('--models-dir', default=models_dir)
    parser.add_argument('--image', default=_default_image_path())
    parser.add_argument('--warmup', type=int, default=3)
    parser.add_argument('--iters', type=int, default=15)
    args = parser.parse_args()

    img = cv2.imread(args.image) if args.image else None
    if img is None:
        print('No test image found/loaded -- falling back to a synthetic '
              'image (see module docstring: timing still valid, detection '
              'CONTENT is not).')
        img = _synthetic_image()
    print(f'Image shape: {img.shape}, device={args.device}, '
          f'warmup={args.warmup}, iters={args.iters}\n')

    configs = [
        ('yolo26s.engine (current default: TensorRT, task=detect)',
         os.path.join(args.models_dir, 'yolo26s.engine'), 'detect'),
        ('yolo26s.pt (PyTorch, task=detect)',
         os.path.join(args.models_dir, 'yolo26s.pt'), None),
        ('yolo26s-seg.pt (PyTorch, task=segment -- this pass\'s candidate)',
         os.path.join(args.models_dir, 'yolo26s-seg.pt'), None),
    ]

    results = {}
    for label, path, task in configs:
        if not os.path.isfile(path):
            print(f'{label}: SKIPPED -- "{path}" not found on this machine.\n')
            continue
        try:
            times_ms, resolved_task = bench(
                path, args.device, img, task=task,
                warmup=args.warmup, iters=args.iters)
        except Exception as exc:  # noqa: BLE001 - keep benchmarking the rest
            print(f'{label}: FAILED -- {exc}\n')
            continue
        results[label] = times_ms
        _report(f'{label} [resolved task={resolved_task}]', times_ms)
        print()

    if 'yolo26s.engine (current default: TensorRT, task=detect)' in results and any(
            'segment' in k for k in results):
        engine_mean = results['yolo26s.engine (current default: TensorRT, task=detect)'].mean()
        for label, times_ms in results.items():
            if 'segment' in label:
                delta = times_ms.mean() - engine_mean
                pct = (delta / engine_mean) * 100.0
                print(f'Delta vs current deployed default: {delta:+.2f}ms '
                      f'({pct:+.1f}%) -- {label}')


if __name__ == '__main__':
    main()
