#!/usr/bin/env python3
"""Trace the car's own hardware in the ZED's frame and propose a self-mask polygon.

NOT COMMITTED and NOT wired into anything: this is the evidence behind
self_mask_proposal.md and self_mask.patch, kept next to them so every number
in them can be reproduced.

Usage (after `source install/setup.bash`):
    python3 output/self_mask_proposal/trace_self_mask.py [--margin-px 8] BAG

WHY NOT FROM DEPTH. The obvious trace is "pixels with no usable depth, or
depth under ~0.3 m, in nearly every frame" -- something centimetres from the
lens is far inside the ZED's minimum stereo range. On
corridor10m_20261015T171753 that does not work, and the reason is worth
writing down: across all 155 depth frames NO pixel is ever -Inf in even a
tenth of them (377 pixels of 230400 are -Inf in at least one frame, none
persistently) and NOT ONE finite depth is under 0.3 m. The 9.96 % of the
frame that is persistently without depth is persistent NaN -- "no stereo
match" -- and on this building that is the GLASS walls and the textureless
floor, not the car. Depth alone cannot tell the car's own hardware from a
pane of glass, so it is not used to place the mask here. The housing itself
is 0.3-1.5 m from the lens and the stereo matches it perfectly well.

WHAT DOES WORK. The LiDAR housing is an orange cap in the bottom-right corner
of the left image, against a grey-and-white corridor, and it is the only
saturated orange in the frame. So the trace is a colour segmentation on the
RGB, required to be persistent across every frame (the mount is rigid), then
the component's own contour -- NOT its convex hull, which on a cap touching
two frame edges adds nothing but which on a diagonal strut would swallow a
triangle of live scene.

This is a ONE-OFF: the mount is rigid, so the output is a static polygon in
FRACTIONS of width and height, which is the format front_clearance_node
already loads and which survives a resolution change with no rescaling.
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS / 'tools'))

import corridor_wall_replay as cwr  # noqa: E402
import rclpy  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from f1tenth_params.param_defaults import get_value  # noqa: E402
from f1tenth_perception import scene_mask  # noqa: E402

#: Saturated orange/amber, bright: the housing cap and nothing else in this
#: corridor. The fire extinguisher is red (hue under 5) and far away.
ORANGE_LO, ORANGE_HI = (5, 120, 90), (32, 255, 255)
#: Components smaller than this fraction of the frame are speckle.
MIN_COMPONENT_FRAC = 0.002
#: A pixel must be hardware in at least this fraction of the frames.
PERSISTENT = 0.9


def current_polygons():
    """The self mask the node builds today, exactly as front_clearance_node does."""
    override = get_value('front_clearance_scene_mask_self_polygons')
    polys = []
    if override:
        polys += json.loads(override)
    else:
        polys.append(scene_mask.rect_polygon(
            float(get_value('lidar_exclusion_x_min')),
            float(get_value('lidar_exclusion_y_min')),
            float(get_value('lidar_exclusion_x_max')),
            float(get_value('lidar_exclusion_y_max'))))
    occluded = get_value('front_clearance_scene_mask_occluded_polygons')
    if occluded:
        polys += json.loads(occluded)
    return polys


def depth_evidence(depths, near_m=0.3):
    """What the depth route would have said, and why it is not used. See above."""
    h, w = depths[0].shape[:2]
    neg = np.zeros((h, w), np.int32)
    near = np.zeros((h, w), np.int32)
    nan = np.zeros((h, w), np.int32)
    good = np.zeros((h, w), np.int32)
    for d in depths:
        with np.errstate(invalid='ignore'):
            ok = np.isfinite(d) & (d > 0)
            neg += np.isneginf(d).astype(np.int32)
            nan += (~ok & ~np.isneginf(d)).astype(np.int32)
            near += (ok & (d < near_m)).astype(np.int32)
            good += (ok & (d >= near_m)).astype(np.int32)
    n = len(depths)
    return {
        'frames': n, 'near_m': near_m,
        'neg_inf_px_ever': int((neg > 0).sum()),
        'near_px_ever': int((near > 0).sum()),
        'persistent_no_match_frac': round(float((nan / n >= PERSISTENT).mean()), 4),
        'persistent_usable_frac': round(float((good / n >= PERSISTENT).mean()), 4),
    }, (nan / n >= PERSISTENT), (good / n >= 0.5)


def hardware_mask(images):
    """Persistently orange pixels: the housing cap."""
    h, w = images[0].shape[:2]
    acc = np.zeros((h, w), np.int32)
    for im in images:
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        acc += (cv2.inRange(hsv, ORANGE_LO, ORANGE_HI) > 0).astype(np.int32)
    return acc / len(images) >= PERSISTENT


def polygons_from(mask, margin_px, h, w):
    """[(vertices px, area)] of each component, grown by margin_px and simplified."""
    m = mask.astype(np.uint8)
    if margin_px > 0:
        k = 2 * int(margin_px) + 1
        m = cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    n_lab, labels = cv2.connectedComponents(m, connectivity=8)
    out = []
    for lab in range(1, n_lab):
        comp = (labels == lab).astype(np.uint8)
        if comp.sum() < MIN_COMPONENT_FRAC * h * w:
            continue
        cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt = max(cnts, key=cv2.contourArea)
        simple = cv2.approxPolyDP(cnt, 0.006 * cv2.arcLength(cnt, True), True)
        pts = simple.reshape(-1, 2)
        if pts.shape[0] >= 3:
            out.append((pts, int(comp.sum())))
    out.sort(key=lambda p: -p[1])
    return out


def overlay(img, mask, colour, alpha=0.45):
    tint = np.zeros_like(img)
    tint[mask] = colour
    out = cv2.addWeighted(img, 1.0 - alpha, tint, alpha, 0.0)
    cnts, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(out, cnts, -1, colour, 2, cv2.LINE_AA)
    return out


def label(img, text, y, colour=(240, 240, 240)):
    for thick, col in ((3, (0, 0, 0)), (1, colour)):
        cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.46, col, thick,
                    cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', type=Path)
    ap.add_argument('--out', type=Path, default=Path(__file__).resolve().parent)
    ap.add_argument('--margin-px', type=int, default=8,
                    help='safety margin at full resolution (default 8)')
    cfg = ap.parse_args()
    rclpy.init()
    try:
        src = cwr.Src(cfg.bag)
    finally:
        rclpy.shutdown()
    bridge = CvBridge()
    depth_topic = next(t for t in cwr.DEPTH_TOPICS if src.get(t))
    depths = [bridge.imgmsg_to_cv2(m, desired_encoding='32FC1')
              for _t, _r, m in src.get(depth_topic)]
    h, w = depths[0].shape[:2]
    t0 = min(t for t, _r, _m in src.get('/scan'))
    rgb = [(t, bridge.imgmsg_to_cv2(m, desired_encoding='bgr8'))
           for t, _r, m in src.get('/camera/image_raw')]
    images = [cv2.resize(im, (w, h), interpolation=cv2.INTER_AREA)
              if im.shape[:2] != (h, w) else im for _t, im in rgb]

    report = {'bag': str(cfg.bag), 'image_wh': [w, h], 'depth_topic': depth_topic,
              'depth_frames': len(depths), 'rgb_frames': len(images),
              'margin_px': cfg.margin_px}
    report['why_not_depth'], no_match, live = depth_evidence(depths)

    cur_polys = current_polygons()
    cur = scene_mask.polygon_mask((h, w), cur_polys)
    hw = hardware_mask(images)
    polys = polygons_from(hw, cfg.margin_px, h, w)
    frac_polys = [[[round(float(x) / w, 4), round(float(y) / h, 4)] for x, y in pts]
                  for pts, _a in polys]
    prop = scene_mask.polygon_mask((h, w), frac_polys)

    report['current'] = {
        'polygons': cur_polys,
        'frac_of_frame': round(float(cur.mean()), 4), 'px': int(cur.sum()),
        'covers_hardware_frac_of_hardware':
            round(float((cur & hw).sum() / max(hw.sum(), 1)), 4),
        'covers_live_scene_frac_of_mask':
            round(float((cur & live).sum() / max(cur.sum(), 1)), 4),
        'covers_live_scene_frac_of_frame': round(float((cur & live).mean()), 4),
        'covers_no_match_frac_of_mask':
            round(float((cur & no_match).sum() / max(cur.sum(), 1)), 4)}
    report['hardware'] = {
        'frac_of_frame': round(float(hw.mean()), 4), 'px': int(hw.sum()),
        'method': 'persistent saturated orange in the left image',
        'outside_current_frac_of_frame': round(float((hw & ~cur).mean()), 4)}
    report['proposed'] = {
        'n_polygons': len(polys), 'polygons': frac_polys,
        'frac_of_frame': round(float(prop.mean()), 4), 'px': int(prop.sum()),
        'freed_frac_of_frame': round(float((cur & ~prop).mean()), 4),
        'newly_masked_frac_of_frame': round(float((prop & ~cur).mean()), 4),
        'covers_hardware_frac_of_hardware':
            round(float((prop & hw).sum() / max(hw.sum(), 1)), 4),
        'covers_live_scene_frac_of_frame': round(float((prop & live).mean()), 4)}

    # ---- before / after overlays ------------------------------------------
    moving = (25.12, 42.72)
    picks = [(t0 + 1.0, 'standstill'), (t0 + 10.0, 'standstill'),
             (0.5 * (moving[0] + moving[1]) + t0, 'moving')]
    comp = {t: m for t, _r, m in src.get('/camera/debug_image/compressed')}
    saved = []
    for want, kind in picks:
        base, source = None, None
        if rgb:
            t, im = min(rgb, key=lambda f: abs(f[0] - want))
            if abs(t - want) < 1.5:
                base, source = im, '/camera/image_raw'
        if base is None and comp:
            t = min(comp, key=lambda x: abs(x - want))
            base = cv2.imdecode(np.frombuffer(bytes(comp[t].data), np.uint8),
                                cv2.IMREAD_COLOR)
            source = '/camera/debug_image/compressed (detector-annotated)'
        if base is None:
            continue
        if base.shape[:2] != (h, w):
            base = cv2.resize(base, (w, h), interpolation=cv2.INTER_AREA)
        before = overlay(base, cur, (0, 0, 255))
        after = overlay(base, prop, (0, 220, 120))
        label(before, f'BEFORE   current self mask   {100 * cur.mean():.1f} % of frame', 20)
        label(before, f'covers {100 * report["current"]["covers_live_scene_frac_of_mask"]:.0f} %'
                      f' live scene, {100 * report["current"]["covers_hardware_frac_of_hardware"]:.0f} %'
                      ' of the hardware', 38, (160, 160, 160))
        label(after, f'AFTER   proposed polygon   {100 * prop.mean():.1f} % '
                     f'(frees {100 * (cur & ~prop).mean():.1f} % of the frame)', 20)
        label(after, f'margin {cfg.margin_px} px, covers '
                     f'{100 * report["proposed"]["covers_hardware_frac_of_hardware"]:.0f} %'
                     ' of the hardware', 38, (160, 160, 160))
        label(after, f't {t - t0:+.1f} s  {kind}  base {source}', h - 10, (160, 160, 160))
        name = f'self_mask_{kind}_t{t - t0:+.1f}s.png'
        cv2.imwrite(str(cfg.out / name), np.hstack([before, after]))
        saved.append(name)

    # The three maps side by side, for the record.
    panel = np.zeros((h, w, 3), np.uint8)
    panel[no_match] = (60, 60, 60)
    panel[cur] = (0, 0, 200)
    panel[hw] = (0, 220, 120)
    panel[cur & hw] = (0, 255, 255)
    label(panel, 'dark grey: persistent NO STEREO MATCH (glass, textureless floor)', 20)
    label(panel, 'red: the current self mask    green: the hardware    '
                 'yellow: both', 38)
    cv2.imwrite(str(cfg.out / 'self_mask_footprint.png'), panel)
    saved.append('self_mask_footprint.png')
    report['images'] = saved
    (cfg.out / 'self_mask_trace.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
