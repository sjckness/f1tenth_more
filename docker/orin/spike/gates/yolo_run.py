#!/usr/bin/env python3
"""G1 (YOLO): run a YOLO checkpoint on stored images the way yolo_detector_node
does (model(bgr, verbose=False, device=..., conf=..., retina_masks=True)) and
save every detection. Runs unchanged on the host (Humble, py3.10, torch 2.11)
and in the spike image (py3.12, torch 2.8), so the two outputs can be compared
with yolo_compare.py.

usage: yolo_run.py MODEL IMAGE_DIR OUT.npz [--conf 0.3] [--device cuda] [--limit N]
"""
import argparse
import glob
import json
import os
import platform
import time

import cv2
import numpy as np
import torch
import ultralytics
from ultralytics import YOLO

ap = argparse.ArgumentParser()
ap.add_argument('model')
ap.add_argument('images')
ap.add_argument('out')
ap.add_argument('--conf', type=float, default=0.3)   # stack_params.yaml confidence_threshold
ap.add_argument('--device', default='cuda')          # stack_params.yaml yolo_device
ap.add_argument('--limit', type=int, default=0)
args = ap.parse_args()

paths = sorted(glob.glob(os.path.join(args.images, '*.png')) + glob.glob(os.path.join(args.images, '*.jpg')))
if args.limit:
    paths = paths[:args.limit]
model = YOLO(args.model)
first = cv2.imread(paths[0])
for _ in range(3):  # warm-up, as the node's first frames
    model(first, verbose=False, device=args.device, conf=args.conf, retina_masks=True)

arrays = {}
per_image = []
lat = []
for i, p in enumerate(paths):
    img = cv2.imread(p)
    if args.device.startswith('cuda'):
        torch.cuda.synchronize()
    t = time.perf_counter()
    r = model(img, verbose=False, device=args.device, conf=args.conf, retina_masks=True)[0]
    if args.device.startswith('cuda'):
        torch.cuda.synchronize()
    lat.append(time.perf_counter() - t)
    b = r.boxes
    arrays[f'{i}_xyxy'] = b.xyxy.cpu().numpy().astype(np.float64)
    arrays[f'{i}_conf'] = b.conf.cpu().numpy().astype(np.float64)
    arrays[f'{i}_cls'] = b.cls.cpu().numpy().astype(np.int64)
    if r.masks is not None:
        m = r.masks.data.cpu().numpy() > 0.5
        arrays[f'{i}_mask_shape'] = np.array(m.shape)
        arrays[f'{i}_mask_bits'] = np.packbits(m.reshape(m.shape[0], -1), axis=1)
    per_image.append({'image': os.path.basename(p), 'n': int(len(b))})

meta = {
    'python': platform.python_version(), 'torch': torch.__version__, 'cuda': torch.version.cuda,
    'cudnn': torch.backends.cudnn.version(), 'ultralytics': ultralytics.__version__,
    'cv2': cv2.__version__, 'numpy': np.__version__, 'model': os.path.basename(args.model),
    'conf': args.conf, 'device': args.device, 'images': len(paths),
    'detections': int(sum(x['n'] for x in per_image)),
    'latency_ms_mean': round(1e3 * float(np.mean(lat)), 2),
    'latency_ms_p95': round(1e3 * float(np.percentile(lat, 95)), 2),
    'per_image': per_image,
}
np.savez_compressed(args.out, meta=json.dumps(meta), **arrays)
print(json.dumps({k: v for k, v in meta.items() if k != 'per_image'}, indent=1))
