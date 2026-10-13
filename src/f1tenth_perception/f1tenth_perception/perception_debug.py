"""
Perception debug video: the pure half (no rclpy, no message packages).

Two producers and one reader share this module, so the wall debug payload has
exactly one definition:

  * front_clearance_node builds /perception/debug/front_wall with
    build_wall_debug() and publishes it as std_msgs/String JSON.
  * yolo_detector_node reads it back with parse_wall_debug(), pairs this
    frame's 2D detections with their 3D positions and semantic track ids
    (match_by_class_score()), and draws the throttled debug image with
    draw_debug_frame() / encode_jpeg().

Everything here is plain numpy/cv2 and is unit-tested without ROS
(test/test_perception_debug.py).

WHY std_msgs/String JSON FOR THE WALL PAYLOAD. No existing message carries
these values (WallLineFit/WallEstimate are the LiDAR's own, WallDistance is the
side-wall tracker's) and a new .msg would change f1tenth_messages for a debug
stream that is only logged. A Float32MultiArray would need a positional layout
documented somewhere else and loses float64 stamps, booleans and the list of
objects over the ROI. JSON carries its own field names, survives fields being
added, and costs ~0.02 ms per frame to build here (~0.1 ms on the Orin).

WHY (class_id, score) EQUALITY PAIRS 2D WITH 3D AND WITH TRACKS. Every
consumer copies the YOLO score unchanged: detection_3d_node copies
hypothesis.score into its Detection3D, and semantic_layer.SemanticObject.update
stores the raw score of the detection that matched a track. All three messages
carry the image's capture stamp. On the 2026-10-01 campaign bags (P004-R012 to
R019, 635 detections) the key matched 632 of 635 2D detections to their 3D
detection (the other 3 were dropped for bad depth) and 518 to a confirmed
track, with zero ambiguous matches.

DISTANCES. Every distance the payload carries names the point it is measured
from (ORIGIN_*), and the reader re-expresses it from the front bumper with
distance_from_front(). front_clearance_node already publishes front_distance,
the obstacle term and the clearance from the bumper (ORIGIN_FRONT); only the
raw pre-EMA stereo value stays from the lens (ORIGIN_CAMERA), next to the
lens-to-bumper offset the node applied. The bumper position is the URDF's
base_front frame (f1tenth_description.vehicle_frames.base_front_x()) in the
nodes; nothing here holds a number for it.
"""

import json
import math

import cv2
import numpy as np

# Version of the /perception/debug/front_wall payload. Bump on a breaking
# change; readers must ignore payloads whose version they do not know.
WALL_DEBUG_VERSION = 1

# The point a payload distance is measured from. 'camera' is the depth
# image's own frame (payload 'frame_id'), resolved by the reader from
# /tf_static; 'front' is the front bumper.
ORIGIN_BASE_LINK = 'base_link'
ORIGIN_CAMERA = 'camera'
ORIGIN_FRONT = 'front'
ORIGIN_MIXED = 'mixed'

# BGR colours. Grey for below-threshold detections, by request.
COLOR_LOW = (150, 150, 150)
COLOR_TEXT = (255, 255, 255)
COLOR_ROI = (0, 215, 255)          # amber: ROI with nothing on it
COLOR_ROI_HIT = (0, 0, 255)        # red: a detected object overlaps the ROI
COLOR_EXCLUSION = (90, 90, 90)
STRIP_BG = (32, 32, 32)
STRIP_PAD_PX = 3
LINE_HEIGHT_PX = 13
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.36
MIN_FONT_SCALE = 0.28


def debug_size(src_w, src_h, out_w):
    """Return (w, h) of the debug image: out_w wide, aspect kept, never upscaled."""
    src_w, src_h, out_w = int(src_w), int(src_h), int(out_w)
    if out_w <= 0 or out_w >= src_w:
        return src_w, src_h
    return out_w, max(1, int(round(src_h * out_w / float(src_w))))


def tick_due(stamp_sec, next_due_sec, period_sec):
    """
    Return (due, next_due) for a stamp-driven throttle.

    The next deadline advances by one period from the previous deadline, not
    from the stamp, so a 15 Hz input throttled to 10 Hz alternates one- and
    two-frame gaps and averages 10 Hz instead of collapsing to 7.5 Hz. A
    deadline more than one period in the past (a gap, or the first frame) is
    reset to stamp + period so the throttle never bursts to catch up; so is a
    stamp that went backwards past the last tick (a replayed bag, a clock
    reset), which would otherwise stall the throttle until time caught up.
    """
    if period_sec <= 0.0:
        return False, next_due_sec
    if (next_due_sec is None or stamp_sec - next_due_sec > period_sec
            or stamp_sec < next_due_sec - period_sec):
        return True, stamp_sec + period_sec
    if stamp_sec + 1e-6 >= next_due_sec:
        return True, next_due_sec + period_sec
    return False, next_due_sec


def match_by_class_score(keys, candidates):
    """
    Pair each (class_id, score) in keys with a candidate carrying the same key.

    candidates: iterable of (class_id, score, payload). Equality is exact on
    purpose (see module docstring); each candidate is used at most once, so
    two detections with an identical key fall back to arrival order. Returns
    a list aligned with keys: the payload, or None when nothing matched.
    """
    pool = {}
    for class_id, score, payload in candidates:
        pool.setdefault((str(class_id), float(score)), []).append(payload)
    out = []
    for class_id, score in keys:
        bucket = pool.get((str(class_id), float(score)))
        out.append(bucket.pop(0) if bucket else None)
    return out


def distance_from_front(value_m, origin_x_m, front_x_m):
    """
    Re-express a forward distance measured from origin_x_m as one from the car front.

    value + origin_x - front_x, the same re-referencing vehicle_frames applies
    (exact for a surface square ahead of the car). Not floored: a debug image
    shows a negative value rather than hiding it. Returns None when any input
    is missing or not finite.
    """
    if value_m is None or origin_x_m is None or front_x_m is None:
        return None
    out = float(value_m) + float(origin_x_m) - float(front_x_m)
    return out if math.isfinite(out) else None


def _num(value, digits=4):
    """Return value rounded for JSON, or None for missing/non-finite values."""
    if value is None:
        return None
    value = float(value)
    return round(value, digits) if math.isfinite(value) else None


def stamp_to_sec(stamp):
    """Return a builtin_interfaces/Time-like stamp as float seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def build_wall_debug(stamp_sec, img_hw, roi, counts, stereo, clearance,
                     objects_in_roi, object_source, det_stamp_sec, lidar,
                     too_close, frame_id='', stereo_origin=ORIGIN_FRONT,
                     obstacle_origin=ORIGIN_FRONT, stereo_raw_origin=ORIGIN_CAMERA,
                     front_offset_m=None):
    """
    Build the /perception/debug/front_wall payload (a dict, see to_json()).

    stamp_sec: the depth frame's stamp. img_hw: (h, w) of the depth image.
    roi: (x0, y0, x1, y1) depth pixels, half-open, as front_clearance_node
    sliced it. counts: dict of pixel counts (roi, housing, too_close, invalid,
    object, bg, min_bg). stereo: dict raw_m, front_distance_m, front_wall.
    clearance: dict obstacle_m, corridor_half_width_m, raw_m,
    front_clearance_m, front_blocked. objects_in_roi: list of
    (class_id, score, overlap_px) for detections whose box meets the ROI.
    lidar: dict from lidar_fit_summary(), or None. frame_id: the depth
    image's frame. stereo_origin / obstacle_origin: what front_distance_m
    and the obstacle distance are measured from; the clearance (the min of
    the two) is 'mixed' when they differ. stereo_raw_origin: what the raw
    pre-EMA stereo value is measured from. front_offset_m: how far the bumper
    sits ahead of the depth frame (None while its pose is unknown).
    """
    h, w = img_hw
    x0, y0, x1, y1 = roi
    return {
        'v': WALL_DEBUG_VERSION,
        'stamp': _num(stamp_sec, 6),
        'frame_id': str(frame_id),
        'img_w': int(w), 'img_h': int(h),
        'roi': [int(x0), int(y0), int(x1), int(y1)],
        'px': {k: int(v) for k, v in counts.items()},
        'too_close': bool(too_close),
        'source': 'stereo',
        'fused_m': None,
        'stereo_origin': str(stereo_origin),
        'stereo_raw_origin': str(stereo_raw_origin),
        'stereo_raw_m': _num(stereo.get('raw_m')),
        'front_offset_m': _num(front_offset_m),
        'front_distance_m': _num(stereo.get('front_distance_m')),
        'front_wall': bool(stereo.get('front_wall')),
        'obj_source': object_source,
        'det_stamp': _num(det_stamp_sec, 6),
        'det_age_s': (_num(stamp_sec - det_stamp_sec, 3)
                      if det_stamp_sec is not None else None),
        'objects_in_roi': [[str(c), _num(s, 3), int(n)] for c, s, n in objects_in_roi],
        'obstacle_origin': str(obstacle_origin),
        'obstacle_m': _num(clearance.get('obstacle_m')),
        'corridor_half_width_m': _num(clearance.get('corridor_half_width_m'), 3),
        'clearance_origin': (str(stereo_origin) if stereo_origin == obstacle_origin
                             else ORIGIN_MIXED),
        'clearance_raw_m': _num(clearance.get('raw_m')),
        'front_clearance_m': _num(clearance.get('front_clearance_m')),
        'front_blocked': bool(clearance.get('front_blocked')),
        'lidar': lidar,
    }


def lidar_fit_summary(valid, reason, distance_m, inlier_count, valid_returns,
                      stamp_sec, now_sec):
    """Summarise one WallLineFit for the wall payload (origin: base_link)."""
    return {
        'valid': bool(valid), 'reason': int(reason),
        'distance_m': _num(distance_m),
        'inliers': int(inlier_count), 'returns': int(valid_returns),
        'age_s': _num(now_sec - stamp_sec, 3),
        'origin': ORIGIN_BASE_LINK,
    }


def to_json(payload):
    """Serialise a payload compactly (no spaces); NaN never appears (see _num)."""
    return json.dumps(payload, separators=(',', ':'), allow_nan=False)


def parse_wall_debug(text):
    """Return the payload dict, or None if it is unreadable or a version we do not know."""
    try:
        payload = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(payload, dict) or payload.get('v') != WALL_DEBUG_VERSION:
        return None
    return payload


def box_overlap_px(box, roi):
    """Return the pixel area of box (x0, y0, x1, y1) that falls inside roi."""
    ix0, iy0 = max(box[0], roi[0]), max(box[1], roi[1])
    ix1, iy1 = min(box[2], roi[2]), min(box[3], roi[3])
    if ix1 <= ix0 or iy1 <= iy0:
        return 0
    return int((ix1 - ix0) * (iy1 - iy0))


def mask_contours(mask, size_wh):
    """Return the outer contours of a binary mask, resized to size_wh first."""
    small = cv2.resize(mask.astype(np.uint8), size_wh, interpolation=cv2.INTER_NEAREST)
    contours, _ = cv2.findContours(small, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def _fmt_m(value):
    return '--' if value is None else f'{value:.2f}'


def _text(img, text, org, color=COLOR_TEXT, fit=False):
    """Draw text; with fit, shrink it until it fits the image width."""
    scale = FONT_SCALE
    if fit:
        width = cv2.getTextSize(text, FONT, scale, 1)[0][0]
        room = img.shape[1] - org[0] - 2
        if width > room > 0:
            scale = max(MIN_FONT_SCALE, scale * room / float(width))
    cv2.putText(img, text, org, FONT, scale, color, 1, cv2.LINE_AA)


def strip_lines(frame):
    """Return the text lines (text, colour) of the strip under the image."""
    wall = frame.get('wall')
    wf = frame.get('wall_front') or {}
    at = frame.get('front_label', '@front')
    head = '{} {} lag {}s conf>{:.2f} grey>={:.2f} d{}'.format(
        frame.get('mission', 'MISSION --'), frame.get('stamp_text', ''),
        _fmt_m(frame.get('lag_s')), frame.get('threshold', 0.0),
        frame.get('min_confidence', 0.0), at)
    lines = [(head, COLOR_TEXT)]
    if wall is None:
        lines.append(('WALL -- (no /perception/debug/front_wall)', COLOR_TEXT))
        return lines
    px = wall.get('px') or {}
    too_close = bool(wall.get('too_close'))
    lines.append((
        'WALL[{}] {}{} raw {}@{} bg {}/{}px obj {}px{}'.format(
            wall.get('source'), _fmt_m(wf.get('used_m')), at,
            _fmt_m(wall.get('stereo_raw_m')),
            wall.get('stereo_raw_origin', wall.get('stereo_origin')),
            px.get('bg', 0), px.get('roi', 0), px.get('object', 0),
            ' TOO CLOSE' if too_close else ''),
        COLOR_ROI_HIT if too_close else COLOR_TEXT))
    lidar = wall.get('lidar')
    if lidar:
        verdict = 'ok' if lidar.get('valid') else 'rej{}'.format(lidar.get('reason'))
        lidar_txt = '{}{} {} n={}/{} age {}s'.format(
            _fmt_m(wf.get('lidar_m')), at, verdict, lidar.get('inliers'),
            lidar.get('returns'), _fmt_m(lidar.get('age_s')))
    else:
        lidar_txt = '--'
    lines.append(('LIDAR {} fused {}'.format(lidar_txt, _fmt_m(wall.get('fused_m'))),
                  COLOR_TEXT))
    lines.append((
        'CLEAR {}@{} {} obj {}{} |y|<{} det_age {}s {}'.format(
            _fmt_m(wall.get('front_clearance_m')), wall.get('clearance_origin'),
            'BLOCKED' if wall.get('front_blocked') else 'free',
            _fmt_m(wf.get('obstacle_m')), at,
            _fmt_m(wall.get('corridor_half_width_m')), _fmt_m(wall.get('det_age_s')),
            wall.get('obj_source')),
        COLOR_TEXT))
    objs = wall.get('objects_in_roi') or []
    if objs or px.get('object', 0):
        listed = ' '.join('{} {:.2f} {}px'.format(c, sc, n) for c, sc, n in objs[:3])
        lines.append(('ROI OBJ ' + (listed or '{}px masked'.format(px.get('object', 0))),
                      COLOR_ROI_HIT))
    else:
        lines.append(('ROI clear', COLOR_TEXT))
    return lines


def draw_debug_frame(img, frame):
    """
    Draw the overlays onto img (downscaled BGR) and return it with the text strip below.

    frame keys (all optional except scale):
      scale           debug px per source px
      hi              list of dicts: box (x0, y0, x1, y1) source px, class_id,
                      score, color, track (str or None), d_front_m (or None),
                      in_corridor (True/False/None), contours (debug px) or None
      lo              list of dicts: box, class_id, score (below threshold)
      threshold       the YOLO confidence threshold in force
      min_confidence  the floor the grey detections were collected down to
      wall            parsed /perception/debug/front_wall payload, or None
      wall_front      dict of distances re-expressed from the car front
      exclusion       (x0, y0, x1, y1) source px of the LiDAR-housing cut, or None
      mission         str, e.g. 'RUNNING llm_536cba700065'
      stamp_text      str, the frame's capture time
      lag_s           float, draw time minus capture time
      front_label     '@front', or '@front?' while the bumper is unknown
    """
    s = float(frame['scale'])
    h, w = img.shape[:2]

    def sp(box):
        return (int(round(box[0] * s)), int(round(box[1] * s)),
                int(round(box[2] * s)), int(round(box[3] * s)))

    exclusion = frame.get('exclusion')
    if exclusion is not None:
        x0, y0, x1, y1 = sp(exclusion)
        cv2.rectangle(img, (x0, y0), (x1 - 1, y1 - 1), COLOR_EXCLUSION, 1)

    wall = frame.get('wall')
    if wall is not None:
        roi = wall['roi']
        hit = bool(wall.get('objects_in_roi')) or int((wall.get('px') or {}).get('object', 0)) > 0
        ws = w / float(wall['img_w'])
        cv2.rectangle(img, (int(roi[0] * ws), int(roi[1] * ws)),
                      (int(roi[2] * ws) - 1, int(roi[3] * ws) - 1),
                      COLOR_ROI_HIT if hit else COLOR_ROI, 1)

    for det in frame.get('lo') or []:
        x0, y0, x1, y1 = sp(det['box'])
        cv2.rectangle(img, (x0, y0), (x1, y1), COLOR_LOW, 1)
        _text(img, '{} {:.2f}'.format(det['class_id'], det['score']),
              (x0 + 2, min(max(y1 - 3, 10), h - 3)), COLOR_LOW)

    for det in frame.get('hi') or []:
        x0, y0, x1, y1 = sp(det['box'])
        color = det['color']
        if det.get('contours'):
            cv2.drawContours(img, det['contours'], -1, color, 1, cv2.LINE_AA)
        cv2.rectangle(img, (x0, y0), (x1, y1), color, 1)
        track = det.get('track')
        label = '{} {:.2f} T{} {}m{}'.format(
            det['class_id'], det['score'], '-' if track is None else track,
            _fmt_m(det.get('d_front_m')), ' C' if det.get('in_corridor') else '')
        (tw, th), base = cv2.getTextSize(label, FONT, FONT_SCALE, 1)
        ty = y0 - 3 if y0 - th - base - 2 >= 0 else min(y1 + th + 3, h - base - 1)
        tx = min(max(x0, 0), max(w - tw - 2, 0))
        cv2.rectangle(img, (tx, ty - th - 2), (tx + tw + 2, ty + base), color, -1)
        _text(img, label, (tx + 1, ty))

    lines = strip_lines(frame)
    strip = np.full((STRIP_PAD_PX * 2 + LINE_HEIGHT_PX * len(lines), w, 3),
                    STRIP_BG, dtype=np.uint8)
    for i, (text, color) in enumerate(lines):
        _text(strip, text, (3, STRIP_PAD_PX + LINE_HEIGHT_PX * (i + 1) - 3), color, fit=True)
    return np.vstack((img, strip))


def encode_jpeg(img, quality):
    """Return JPEG bytes of img at the given quality, or None if encoding failed."""
    ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return buf.tobytes() if ok else None
