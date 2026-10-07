#!/usr/bin/env python3
"""Compare two yolo_run.py outputs: per image, match detections greedily by
class and box IoU, then report count mismatches, worst box IoU, worst
|conf delta|, worst box corner shift (px) and worst mask IoU.

usage: yolo_compare.py A.npz B.npz
"""
import json
import sys

import numpy as np


def load(p):
    z = np.load(p, allow_pickle=False)
    return json.loads(str(z['meta'])), z


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def masks(z, i):
    if f'{i}_mask_bits' not in z:
        return None
    shape = z[f'{i}_mask_shape']
    bits = np.unpackbits(z[f'{i}_mask_bits'], axis=1)[:, :shape[1] * shape[2]]
    return bits.reshape(shape).astype(bool)


ma, a = load(sys.argv[1])
mb, b = load(sys.argv[2])
assert ma['images'] == mb['images'], 'different image sets'
worst = {'min_box_iou': 1.0, 'max_conf_delta': 0.0, 'max_corner_px': 0.0, 'min_mask_iou': 1.0}
count_mismatch = []
unmatched = 0
matched = 0
for i in range(ma['images']):
    xa, ca, ka = a[f'{i}_xyxy'], a[f'{i}_conf'], a[f'{i}_cls']
    xb, cb, kb = b[f'{i}_xyxy'], b[f'{i}_conf'], b[f'{i}_cls']
    if len(xa) != len(xb):
        count_mismatch.append((ma['per_image'][i]['image'], len(xa), len(xb)))
    mka, mkb = masks(a, i), masks(b, i)
    used = set()
    for j in range(len(xa)):
        best, bk = -1.0, None
        for k in range(len(xb)):
            if k in used or kb[k] != ka[j]:
                continue
            v = iou(xa[j], xb[k])
            if v > best:
                best, bk = v, k
        if bk is None or best < 0.5:
            unmatched += 1
            continue
        used.add(bk)
        matched += 1
        worst['min_box_iou'] = min(worst['min_box_iou'], best)
        worst['max_conf_delta'] = max(worst['max_conf_delta'], abs(float(ca[j] - cb[bk])))
        worst['max_corner_px'] = max(worst['max_corner_px'], float(np.abs(xa[j] - xb[bk]).max()))
        if mka is not None and mkb is not None and mka.shape[1:] == mkb.shape[1:]:
            inter = np.logical_and(mka[j], mkb[bk]).sum()
            union = np.logical_or(mka[j], mkb[bk]).sum()
            if union:
                worst['min_mask_iou'] = min(worst['min_mask_iou'], inter / union)
    unmatched += len(xb) - len(used)

res = {
    'A': {k: ma[k] for k in ('python', 'torch', 'cuda', 'cudnn', 'ultralytics', 'cv2', 'numpy', 'detections', 'latency_ms_mean', 'latency_ms_p95')},
    'B': {k: mb[k] for k in ('python', 'torch', 'cuda', 'cudnn', 'ultralytics', 'cv2', 'numpy', 'detections', 'latency_ms_mean', 'latency_ms_p95')},
    'images': ma['images'], 'matched': matched, 'unmatched': unmatched,
    'images_with_count_mismatch': count_mismatch,
    **{k: round(float(v), 6) for k, v in worst.items()},
}
print(json.dumps(res, indent=1))
