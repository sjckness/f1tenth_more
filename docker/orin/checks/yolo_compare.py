#!/usr/bin/env python3
"""
Compare two yolo_run.py outputs over the same images and give the verdict.

Runs on: anywhere with numpy (no torch needed). Plan P9.5, the criterion
decided for D3 on 2026-10-07: objects matched one to one (same class,
greedy by box IoU, >= 0.5 to pair at all), every matched pair with box IoU
>= 0.99, mask IoU >= 0.99 and |conf delta| <= 2e-3. A detection present on
one side only counts as agreement only if its confidence is within 2e-3 of
the detection threshold (0.30): it sat on the threshold and the other side's
score fell just under it. A torch 2.8 (image) vs 2.11 (host) residual of that
size is expected and cannot be removed; anything larger is a finding.

usage: yolo_compare.py A.npz B.npz [--box-iou 0.99] [--mask-iou 0.99]
                       [--conf-tol 2e-3] [--threshold CONF] [--out FILE.json]
Exit 0 on PASS.
"""
import argparse
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
    if '%d_mask_bits' % i not in z:
        return None
    shape = z['%d_mask_shape' % i]
    bits = np.unpackbits(z['%d_mask_bits' % i], axis=1)[:, :shape[1] * shape[2]]
    return bits.reshape(shape).astype(bool)


def match_image(xa, ca, ka, xb, cb, kb, mka=None, mkb=None):
    """
    Greedy one-to-one matching of one image's detections.

    Returns (pairs, unmatched_a, unmatched_b); a pair is
    (j, k, box_iou, |dconf|, max_corner_px, mask_iou or None).
    """
    used, pairs, unmatched_a = set(), [], []
    for j in range(len(xa)):
        best, bk = -1.0, None
        for k in range(len(xb)):
            if k in used or kb[k] != ka[j]:
                continue
            v = iou(xa[j], xb[k])
            if v > best:
                best, bk = v, k
        if bk is None or best < 0.5:
            unmatched_a.append(j)
            continue
        used.add(bk)
        miou = None
        if mka is not None and mkb is not None and mka.shape[1:] == mkb.shape[1:]:
            union = np.logical_or(mka[j], mkb[bk]).sum()
            if union:
                miou = float(np.logical_and(mka[j], mkb[bk]).sum() / union)
        pairs.append((j, bk, best, abs(float(ca[j] - cb[bk])),
                      float(np.abs(np.asarray(xa[j]) - np.asarray(xb[bk])).max()), miou))
    unmatched_b = [k for k in range(len(xb)) if k not in used]
    return pairs, unmatched_a, unmatched_b


def compare(ma, a, mb, b, box_iou=0.99, mask_iou=0.99, conf_tol=2e-3, threshold=None):
    if ma['images'] != mb['images']:
        raise SystemExit('different image sets (%d vs %d)' % (ma['images'], mb['images']))
    thr = ma['conf'] if threshold is None else threshold
    worst = {'min_box_iou': 1.0, 'max_conf_delta': 0.0, 'max_corner_px': 0.0,
             'min_mask_iou': 1.0}
    matched, borderline, problems = 0, [], []
    for i in range(ma['images']):
        name = ma['per_image'][i]['image']
        xa, ca, ka = a['%d_xyxy' % i], a['%d_conf' % i], a['%d_cls' % i]
        xb, cb, kb = b['%d_xyxy' % i], b['%d_conf' % i], b['%d_cls' % i]
        pairs, ua, ub = match_image(xa, ca, ka, xb, cb, kb, masks(a, i), masks(b, i))
        for j, k, biou, dconf, corner, miou in pairs:
            matched += 1
            worst['min_box_iou'] = min(worst['min_box_iou'], biou)
            worst['max_conf_delta'] = max(worst['max_conf_delta'], dconf)
            worst['max_corner_px'] = max(worst['max_corner_px'], corner)
            if miou is not None:
                worst['min_mask_iou'] = min(worst['min_mask_iou'], miou)
            if biou < box_iou or dconf > conf_tol or (miou is not None and miou < mask_iou):
                problems.append('%s: class %d box IoU %.4f mask IoU %s |dconf| %.2e'
                                % (name, int(ka[j]), biou, miou, dconf))
        for side, idx, conf, cls in (('A', ua, ca, ka), ('B', ub, cb, kb)):
            for j in idx:
                entry = '%s: only in %s, class %d conf %.4f' % (name, side, int(cls[j]), conf[j])
                if abs(float(conf[j]) - thr) <= conf_tol:
                    borderline.append(entry)
                else:
                    problems.append(entry)
    keys = ('python', 'torch', 'cuda', 'ultralytics', 'model', 'detections', 'latency_ms_mean',
            'latency_ms_p95')
    return {
        'A': {k: ma.get(k) for k in keys}, 'B': {k: mb.get(k) for k in keys},
        'images': ma['images'], 'matched': matched, 'threshold': thr,
        'gates': {'box_iou': box_iou, 'mask_iou': mask_iou, 'conf_tol': conf_tol},
        'borderline_unmatched': borderline, 'problems': problems,
        **{k: round(float(v), 6) for k, v in worst.items()},
        'result': 'PASS' if not problems else 'FAIL',
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('usage:'):])
    ap.add_argument('a')
    ap.add_argument('b')
    ap.add_argument('--box-iou', type=float, default=0.99)
    ap.add_argument('--mask-iou', type=float, default=0.99)
    ap.add_argument('--conf-tol', type=float, default=2e-3)
    ap.add_argument('--threshold', type=float, default=None,
                    help='detection threshold (default: the conf A was run with)')
    ap.add_argument('--out')
    args = ap.parse_args(argv)
    ma, a = load(args.a)
    mb, b = load(args.b)
    res = compare(ma, a, mb, b, args.box_iou, args.mask_iou, args.conf_tol, args.threshold)
    text = json.dumps(res, indent=1)
    print(text)
    if args.out:
        with open(args.out, 'w') as f:
            f.write(text + '\n')
    return 0 if res['result'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
