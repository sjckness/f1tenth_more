#!/usr/bin/env python3
"""
YOLO inference on its own (perf baseline): device actually used, ms per frame.

    bench_yolo.py [--model src/f1tenth_perception/models/yolo26s-seg.pt] [--out file.json]

Not the node: Ultralytics' own predict() on one synthetic 640x360 BGR frame
(perf_scene's corridor is not a photograph; the cost of a forward pass does not
depend on the content, the post-processing does, slightly). imgsz 640 with the
rect letterbox, retina_masks on, as yolo_detector_node configures the predictor.
20 warm-up frames, 100 timed with a CUDA synchronise on both sides.
"""

import argparse
import json
import pathlib
import sys
import time

import numpy as np


def main(argv=None):
    here = pathlib.Path(__file__).resolve().parents[2]
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--model', default=str(here / 'src/f1tenth_perception/models/yolo26s-seg.pt'))
    ap.add_argument('--device', default='cuda')
    ap.add_argument('--out')
    args = ap.parse_args(argv)
    import torch
    from ultralytics import YOLO
    if args.device.startswith('cuda') and not torch.cuda.is_available():
        print('CUDA not available: the node would refuse to start', file=sys.stderr)
        return 2
    frame = np.random.default_rng(0).integers(0, 255, (360, 640, 3), dtype=np.uint8)
    model = YOLO(args.model)
    sync = torch.cuda.synchronize if args.device.startswith('cuda') else (lambda: None)
    for _ in range(20):
        model.predict(frame, imgsz=640, device=args.device, retina_masks=True, verbose=False)
    total, stages = [], {'preprocess': [], 'inference': [], 'postprocess': []}
    for _ in range(100):
        sync()
        t0 = time.perf_counter()
        res = model.predict(frame, imgsz=640, device=args.device, retina_masks=True,
                            verbose=False)[0]
        sync()
        total.append((time.perf_counter() - t0) * 1e3)
        for k in stages:
            stages[k].append(res.speed[k])
    def st(v):  # noqa: E306
        return {'p50': round(float(np.percentile(v, 50)), 2),
                'p95': round(float(np.percentile(v, 95)), 2), 'max': round(float(np.max(v)), 2)}
    out = {'model': pathlib.Path(args.model).name, 'device': args.device,
           'gpu': torch.cuda.get_device_name(0) if args.device.startswith('cuda') else None,
           'torch': torch.__version__, 'frame': '640x360', 'predict_total_ms': st(total),
           **{f'{k}_ms': st(v) for k, v in stages.items()}}
    text = json.dumps(out, indent=1)
    if args.out:
        pathlib.Path(args.out).write_text(text + '\n')
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
