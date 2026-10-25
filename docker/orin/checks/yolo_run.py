#!/usr/bin/env python3
"""
Run a YOLO model on stored images as yolo_detector_node does; keep every detection.

The output is yolo_compare.py's input.

Runs on: the Orin host (native Humble: py3.10, torch 2.11) and inside the
container (py3.12, torch 2.8), unchanged; also linus (--device cpu). From the
spike gate (jazzy line), with --task for TensorRT engines.
Plan P9.5: "YOLO container (torch 2.8) vs native (2.11) IoU >= 0.99, |dconf| <=
2e-3, engine vs .pt the same": run the .pt on both sides and the .engine in
the container over the same image directory (extract_frames.py), then
yolo_compare.py pairwise.

The call is the node's: model(bgr, verbose=False, device=..., conf=...,
retina_masks=True), after 3 warm-up calls on the first image.

usage: yolo_run.py MODEL IMAGE_DIR OUT.npz [--conf 0.3] [--device cuda]
                   [--limit N] [--task segment]
"""
import argparse
import glob
import json
import os
import platform
import sys
import time


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('model', help='.pt checkpoint or TensorRT .engine')
    ap.add_argument('images', help='directory of .png / .jpg images')
    ap.add_argument('out', help='output .npz')
    ap.add_argument('--conf', type=float, default=0.3,
                    help='stack_params.yaml confidence_threshold (0.3)')
    ap.add_argument('--device', default='cuda', help='stack_params.yaml yolo_device (cuda)')
    ap.add_argument('--limit', type=int, default=0)
    ap.add_argument('--task', default=None,
                    help='Ultralytics task for a model without metadata (engine: segment)')
    a = ap.parse_args(argv)

    import cv2
    import numpy as np
    import torch
    import ultralytics
    from ultralytics import YOLO

    paths = sorted(glob.glob(os.path.join(a.images, '*.png'))
                   + glob.glob(os.path.join(a.images, '*.jpg')))
    if a.limit:
        paths = paths[:a.limit]
    if not paths:
        raise SystemExit('no images in %s' % a.images)
    model = YOLO(a.model, task=a.task) if a.task else YOLO(a.model)
    first = cv2.imread(paths[0])
    for _ in range(3):  # warm-up, as the node's first frames
        model(first, verbose=False, device=a.device, conf=a.conf, retina_masks=True)

    arrays = {}
    per_image = []
    lat = []
    cuda = a.device.startswith('cuda')
    for i, p in enumerate(paths):
        img = cv2.imread(p)
        if cuda:
            torch.cuda.synchronize()
        t = time.perf_counter()
        r = model(img, verbose=False, device=a.device, conf=a.conf, retina_masks=True)[0]
        if cuda:
            torch.cuda.synchronize()
        lat.append(time.perf_counter() - t)
        b = r.boxes
        arrays['%d_xyxy' % i] = b.xyxy.cpu().numpy().astype(np.float64)
        arrays['%d_conf' % i] = b.conf.cpu().numpy().astype(np.float64)
        arrays['%d_cls' % i] = b.cls.cpu().numpy().astype(np.int64)
        if r.masks is not None:
            m = r.masks.data.cpu().numpy() > 0.5
            arrays['%d_mask_shape' % i] = np.array(m.shape)
            arrays['%d_mask_bits' % i] = np.packbits(m.reshape(m.shape[0], -1), axis=1)
        per_image.append({'image': os.path.basename(p), 'n': int(len(b))})

    meta = {
        'python': platform.python_version(), 'torch': torch.__version__,
        'cuda': torch.version.cuda, 'cudnn': torch.backends.cudnn.version(),
        'ultralytics': ultralytics.__version__, 'cv2': cv2.__version__,
        'numpy': np.__version__, 'model': os.path.basename(a.model), 'conf': a.conf,
        'device': a.device, 'images': len(paths),
        'detections': int(sum(x['n'] for x in per_image)),
        'latency_ms_mean': round(1e3 * float(np.mean(lat)), 2),
        'latency_ms_p95': round(1e3 * float(np.percentile(lat, 95)), 2),
        'per_image': per_image,
    }
    np.savez_compressed(a.out, meta=json.dumps(meta), **arrays)
    print(json.dumps({k: v for k, v in meta.items() if k != 'per_image'}, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
