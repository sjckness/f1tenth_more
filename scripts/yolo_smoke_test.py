#!/usr/bin/env python3
r"""
Smoke test of yolo_detector_node before a GPU-pipeline change goes to the car.

Runs the REAL YoloDetectorNode class in-process on extracted frames, once per
side, each side in a fresh child process (sequentially, never two YOLO
processes at once), and compares side B against side A per frame:

  A ("old"): gpu_pipeline=false     B ("new"): gpu_pipeline=true
  both: model_path=--model, device=cuda, confidence_threshold=0.3,
        max_rate_hz=0.0 (every frame processed), yolo_backend=pt

The node's publishers are replaced by capture functions (no DDS traffic;
the child is also DDS-isolated: domain 147, localhost only, no Discovery
Server) and image_callback() is called directly with sensor_msgs/Image bgra8
messages built from the frames. PASS requires, per frame: no exception, one
detections and one mask message carrying the input header, the same class
multiset (an unmatched detection within 0.05 of the confidence threshold is
ignored), every matched box pair IoU > 0.8, label masks of the image size with
labels 1..N (0x0 on empty frames), the same message types and field sets, and
no ERROR line on side B beyond side A's count. Mask IoU, timing and CPU are
printed as information. Exit 0 on PASS, 1 on FAIL.

Usage on linus (bash; the GPU lock is taken inside each child):
  source /home/andreas/bags/yolo_gpu/work/env_yolo_gpu.sh
  cd /home/andreas/dev_ws/f1tenth_more_yolo_gpu
  $PY scripts/yolo_smoke_test.py                      # A=classic vs B=gpu_pipeline
  $PY scripts/yolo_smoke_test.py --b gpu_pipeline=false   # A vs A: must PASS
  $PY scripts/yolo_smoke_test.py --a cpu_threads=0 --b gpu_pipeline=true,cpu_threads=2

Usage on the Orin (frames copied to ~/yolo_gpu/, the people set is generated
there from the ultralytics assets when missing; results in ~/yolo_gpu/smoke):
  source /opt/ros/humble/setup.bash
  source ~/dev_ws/f1tenth_more/install/setup.bash
  cd ~/dev_ws/f1tenth_more
  python3 scripts/yolo_smoke_test.py \
      --frames ~/yolo_gpu/frames_corridor10m_20261015T171753 \
      --frames ~/yolo_gpu/frames_people

Options: --a / --b k=v,k=v (param overrides; true/false -> bool, 3 -> int,
0.5 -> float, else str; ints given for known float params become floats),
--frames DIR (repeatable; default: the corridor set + the people set),
--model PATH, --out DIR, --make-people (regenerate the people set),
--compare-only (compare the existing <out>/a.json and b.json again).
Outputs in --out: <side>.log (child stdout+stderr), <side>.json (per-frame
detections, mask headers, types, timing), <side>.npz (mono8 masks).
"""

import argparse
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import platform
import re
import resource
import statistics
import subprocess
import sys
import time
import traceback

REPO = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = REPO / 'src/f1tenth_perception/models/yolo26s-seg.pt'
_LINUS_BASE = Path('/home/andreas/bags/yolo_gpu')
BASE = _LINUS_BASE if _LINUS_BASE.is_dir() else Path.home() / 'yolo_gpu'
DEFAULT_OUT = BASE / 'work/smoke' if _LINUS_BASE.is_dir() else BASE / 'smoke'
CORRIDOR = BASE / 'frames_corridor10m_20261015T171753'
PEOPLE = BASE / 'frames_people'
PEOPLE_NAME = 'frames_people'
GPU_LOCK = Path('/home/andreas/bags/yolo_gpu/work/gpu.lock')
FRAME_ID = 'zed2_left_camera_optical_frame'
PEOPLE_STAMP0 = 1800000000  # after the corridor stamps (2026-10), increasing at 30 Hz

BASE_PARAMS = {'device': 'cuda', 'confidence_threshold': 0.3,
               'max_rate_hz': 0.0, 'yolo_backend': 'pt'}
SIDE_DEFAULTS = {'a': {'gpu_pipeline': False}, 'b': {'gpu_pipeline': True}}
FLOAT_PARAMS = {
    'confidence_threshold', 'max_rate_hz', 'debug_image_rate_hz',
    'debug_image_min_confidence', 'lidar_exclusion_x_min', 'lidar_exclusion_x_max',
    'lidar_exclusion_y_min', 'lidar_exclusion_y_max',
    'lidar_exclusion_overlap_threshold', 'stage_timing_period_s',
    'stage_timing_frame_period_s'}
DDS_ENV = {'ROS_DOMAIN_ID': '147', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
BORDERLINE = 0.05
BOX_IOU_MIN = 0.8
SKIP_TOPICS = ('/parameter_events', '/rosout')
LOG_WARN = re.compile(r'\[(WARN|WARNING)\]|Warning:')
LOG_ERROR = re.compile(r'\[(ERROR|FATAL)\]|Traceback')


# ---------------------------------------------------------------- parameters
def parse_value(key, text):
    low = text.lower()
    if low in ('true', 'false'):
        return low == 'true'
    try:
        ival = int(text)
    except ValueError:
        pass
    else:
        return float(ival) if key in FLOAT_PARAMS else ival
    try:
        return float(text)
    except ValueError:
        return text


def parse_overrides(text):
    out = {}
    for item in (text or '').split(','):
        item = item.strip()
        if not item:
            continue
        if '=' not in item:
            raise SystemExit(f'bad override {item!r}: want name=value')
        key, val = item.split('=', 1)
        out[key.strip()] = parse_value(key.strip(), val.strip())
    return out


# ---------------------------------------------------------------- people set
def ultralytics_assets():
    spec = importlib.util.find_spec('ultralytics')  # does not import torch
    if spec is None or not spec.submodule_search_locations:
        raise SystemExit('ultralytics not found: cannot build the people frame set')
    return Path(list(spec.submodule_search_locations)[0]) / 'assets'


def make_people(dst):
    """Write ~20 deterministic 640x360 bgra8 frames from bus.jpg / zidane.jpg."""
    import cv2
    import numpy as np
    assets = ultralytics_assets()
    imgs = {name: cv2.imread(str(assets / f'{name}.jpg')) for name in ('bus', 'zidane')}
    for name, img in imgs.items():
        if img is None:
            raise SystemExit(f'cannot read {assets / name}.jpg')
    out_w, out_h = 640, 360

    def crop(img, fx, fy, fw):
        h, w = img.shape[:2]
        cw = fw * w
        ch = cw * out_h / out_w
        if ch > h:
            ch, cw = h, h * out_w / out_h
        x0 = int(round(fx * (w - cw)))
        y0 = int(round(fy * (h - ch)))
        win = img[y0:y0 + int(round(ch)), x0:x0 + int(round(cw))]
        interp = cv2.INTER_AREA if win.shape[1] > out_w else cv2.INTER_LINEAR
        return cv2.resize(win, (out_w, out_h), interpolation=interp)

    def letterbox(img, scale):
        h, w = img.shape[:2]
        r = min(out_w / w, out_h / h) * scale
        nw, nh = max(1, int(round(w * r))), max(1, int(round(h * r)))
        canvas = np.full((out_h, out_w, 3), 114, np.uint8)
        x0, y0 = (out_w - nw) // 2, (out_h - nh) // 2
        canvas[y0:y0 + nh, x0:x0 + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
        return canvas

    def pair(scale):
        left = letterbox(imgs['bus'], scale)[:, 160:480]
        right = letterbox(imgs['zidane'], scale)[:, 160:480]
        return np.concatenate([left, right], axis=1)

    z, b = imgs['zidane'], imgs['bus']
    recipe = [
        ('zidane letterbox 1.0', lambda: letterbox(z, 1.0), False),
        ('zidane letterbox 1.0', lambda: letterbox(z, 1.0), True),
        ('zidane letterbox 0.7', lambda: letterbox(z, 0.7), False),
        ('zidane letterbox 0.5', lambda: letterbox(z, 0.5), True),
        ('zidane crop 0.75 top-left', lambda: crop(z, 0.0, 0.0, 0.75), False),
        ('zidane crop 0.75 bottom-right', lambda: crop(z, 1.0, 1.0, 0.75), False),
        ('zidane crop 0.6 centre', lambda: crop(z, 0.5, 0.5, 0.6), False),
        ('zidane crop 0.6 centre', lambda: crop(z, 0.5, 0.5, 0.6), True),
        ('zidane crop 0.5 left', lambda: crop(z, 0.1, 0.4, 0.5), False),
        ('bus letterbox 1.0', lambda: letterbox(b, 1.0), False),
        ('bus letterbox 1.0', lambda: letterbox(b, 1.0), True),
        ('bus letterbox 0.8', lambda: letterbox(b, 0.8), False),
        ('bus crop 1.0 y=0.3', lambda: crop(b, 0.0, 0.3, 1.0), False),
        ('bus crop 1.0 y=0.5', lambda: crop(b, 0.0, 0.5, 1.0), False),
        ('bus crop 1.0 y=0.6', lambda: crop(b, 0.0, 0.6, 1.0), True),
        ('bus crop 0.7 bottom', lambda: crop(b, 0.5, 1.0, 0.7), False),
        ('bus crop 0.5 left', lambda: crop(b, 0.0, 0.55, 0.5), False),
        ('bus+zidane side by side 1.0', lambda: pair(1.0), False),
        ('bus+zidane side by side 0.8', lambda: pair(0.8), True),
        ('blank grey 114 (no detections)',
         lambda: np.full((out_h, out_w, 3), 114, np.uint8), False),
    ]
    (dst / 'rgb').mkdir(parents=True, exist_ok=True)
    frames = []
    for i, (desc, build, flip) in enumerate(recipe):
        bgr = build()
        if flip:
            bgr = cv2.flip(bgr, 1)
        bgra = cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA)
        assert bgra.shape == (out_h, out_w, 4) and bgra.dtype == np.uint8
        rel = f'rgb/{i:06d}.npy'
        np.save(dst / rel, bgra)
        frames.append({
            'i': i, 'stamp_sec': PEOPLE_STAMP0 + i // 30,
            'stamp_nanosec': int(round((i % 30) * 1e9 / 30)), 'frame_id': FRAME_ID,
            'encoding': 'bgra8', 'height': out_h, 'width': out_w, 'step': out_w * 4,
            'is_bigendian': 0, 'rgb': rel, 'desc': desc + (' h-flip' if flip else '')})
    index = {'source': f'ultralytics assets {assets} (bus.jpg, zidane.jpg)',
             'generator': 'scripts/yolo_smoke_test.py --make-people', 'frames': frames}
    with open(dst / 'index.json', 'w') as fh:
        json.dump(index, fh, indent=1)
    print(f'people set: {len(frames)} frames written to {dst}')


# ---------------------------------------------------------------- child side
def msg_schema(msg, out):
    """Collect {type: get_fields_and_field_types()} over msg and nested msgs."""
    name = f'{type(msg).__module__}.{type(msg).__name__}'
    if name in out:
        return
    out[name] = dict(msg.get_fields_and_field_types())
    for field in out[name]:
        val = getattr(msg, field)
        if hasattr(val, 'get_fields_and_field_types'):
            msg_schema(val, out)
        elif isinstance(val, (list, tuple)) and val and hasattr(
                val[0], 'get_fields_and_field_types'):
            msg_schema(val[0], out)


def isolate_dds(env):
    """Domain 147, localhost only, no Discovery Server: never touch a live graph.

    ROS_LOCALHOST_ONLY only on distros before Iron (Humble on the Orin): Jazzy
    deprecates it (two WARN lines) and ROS_AUTOMATIC_DISCOVERY_RANGE does the job.
    """
    env.update(DDS_ENV)
    env.pop('ROS_DISCOVERY_SERVER', None)
    if env.get('ROS_DISTRO', 'humble') in ('foxy', 'galactic', 'humble'):
        env['ROS_LOCALHOST_ONLY'] = '1'
    else:
        env.pop('ROS_LOCALHOST_ONLY', None)


def run_child(cfg_path):
    with open(cfg_path) as fh:
        cfg = json.load(fh)
    lock = None
    if GPU_LOCK.parent.is_dir():
        lock = open(GPU_LOCK, 'a')
        t_lock = time.monotonic()
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        print(f'[smoke] gpu.lock taken after {time.monotonic() - t_lock:.1f} s',
              file=sys.stderr, flush=True)
    isolate_dds(os.environ)

    import array
    import numpy as np
    import rclpy
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import Image
    from f1tenth_perception import yolo_detector_node as ydn

    params = cfg['params']
    res = {'side': cfg['side'], 'params': params, 'node_module': ydn.__file__,
           'frames': [], 'schema': {}, 'fatal': None}
    out = Path(cfg['out'])
    rclpy.init()
    try:
        node = ydn.YoloDetectorNode(
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    except Exception:  # noqa: BLE001 - reported to the parent
        res['fatal'] = traceback.format_exc()
        print(res['fatal'], file=sys.stderr, flush=True)
        with open(out / f"{cfg['side']}.json", 'w') as fh:
            json.dump(res, fh)
        rclpy.shutdown()
        return 2
    res['has_parameter'] = {k: bool(node.has_parameter(k)) for k in params}

    captured = []

    def capture(name):
        return lambda msg: captured.append((name, msg))

    patched = set()
    for attr, pub in list(vars(node).items()):
        if attr.endswith('_pub') and pub is not None and hasattr(pub, 'publish'):
            pub.publish = capture(attr)
            patched.add(id(pub))
    for pub in list(node.publishers):
        if id(pub) not in patched and pub.topic_name not in SKIP_TOPICS:
            pub.publish = capture(pub.topic_name)
    for attr in ('annotated_pub', 'debug_clean_pub'):
        if getattr(node, attr, None) is not None:
            getattr(node, attr).get_subscription_count = lambda: 0
    res['patched'] = sorted(a for a, p in vars(node).items() if id(p) in patched)

    by_stamp = {}
    first = True
    for set_name, set_dir in cfg['frame_sets']:
        set_dir = Path(set_dir)
        with open(set_dir / 'index.json') as fh:
            index = json.load(fh)
        for k, meta in enumerate(index['frames']):
            arr = np.load(set_dir / meta['rgb'])
            msg = Image()
            if 'stamp_sec' in meta:
                msg.header.stamp.sec = int(meta['stamp_sec'])
                msg.header.stamp.nanosec = int(meta['stamp_nanosec'])
            else:
                msg.header.stamp.sec = PEOPLE_STAMP0 + k // 30
                msg.header.stamp.nanosec = int(round((k % 30) * 1e9 / 30))
            msg.header.frame_id = FRAME_ID
            msg.height, msg.width = int(arr.shape[0]), int(arr.shape[1])
            msg.encoding = 'bgra8'
            msg.is_bigendian = 0
            msg.step = msg.width * 4
            msg.data = array.array('B', np.ascontiguousarray(arr).tobytes())
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if key in by_stamp:
                raise SystemExit(f'duplicate stamp {key} in {set_name}: cannot pair outputs')
            rec = {'set': set_name, 'i': k, 'stamp': list(key), 'frame_id': FRAME_ID,
                   'h': msg.height, 'w': msg.width, 'exception': None,
                   'det_msgs': [], 'mask_msgs': [], 'other': {}, 'warmup': first}
            first = False
            by_stamp[key] = rec
            ru0 = resource.getrusage(resource.RUSAGE_SELF)
            t0 = time.perf_counter()
            try:
                node.image_callback(msg)
            except Exception:  # noqa: BLE001 - one exception = FAIL, keep going
                rec['exception'] = traceback.format_exc()
                print(rec['exception'], file=sys.stderr, flush=True)
            rec['callback_ms'] = (time.perf_counter() - t0) * 1e3
            rclpy.spin_once(node, timeout_sec=0.0)
            ru1 = resource.getrusage(resource.RUSAGE_SELF)
            rec['wall_ms'] = (time.perf_counter() - t0) * 1e3
            rec['cpu_ms'] = ((ru1.ru_utime - ru0.ru_utime)
                             + (ru1.ru_stime - ru0.ru_stime)) * 1e3
            res['frames'].append(rec)
    t_end = time.monotonic() + 0.3   # drain anything published from a timer
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    masks = {}
    res['orphans'] = []
    for name, msg in captured:
        schema = res['schema'].setdefault(name, {})
        msg_schema(msg, schema)
        header = getattr(msg, 'header', None)
        key = (header.stamp.sec, header.stamp.nanosec) if header is not None else None
        rec = by_stamp.get(key)
        if rec is None:
            res['orphans'].append({'pub': name, 'stamp': list(key) if key else None})
            continue
        if name == 'det_pub':
            dets = []
            for det in msg.detections:
                hyp = det.results[0].hypothesis if det.results else None
                dets.append({
                    'class_id': hyp.class_id if hyp else '', 'score': hyp.score if hyp else 0.0,
                    'cx': det.bbox.center.position.x, 'cy': det.bbox.center.position.y,
                    'w': det.bbox.size_x, 'h': det.bbox.size_y,
                    'theta': det.bbox.center.theta, 'n_results': len(det.results)})
            rec['det_msgs'].append({'frame_id': header.frame_id, 'detections': dets})
        elif name == 'masks_pub':
            mkey = f"{rec['set']}_{rec['i']:06d}_{len(rec['mask_msgs'])}"
            h, w, step = int(msg.height), int(msg.width), int(msg.step)
            raw = np.frombuffer(bytes(msg.data), np.uint8)
            if h * w > 0 and raw.size == h * step:
                masks[mkey] = raw.reshape(h, step)[:, :w].copy()
            rec['mask_msgs'].append({
                'frame_id': header.frame_id, 'height': h, 'width': w, 'step': step,
                'encoding': msg.encoding, 'is_bigendian': int(msg.is_bigendian),
                'len_data': int(raw.size), 'key': mkey if mkey in masks else None})
        else:
            rec['other'][name] = rec['other'].get(name, 0) + 1
            if header.frame_id != FRAME_ID:
                res['orphans'].append({'pub': name, 'frame_id': header.frame_id})
    try:
        node.destroy_node()
    finally:
        rclpy.shutdown()
    np.savez_compressed(out / f"{cfg['side']}.npz", **masks)
    with open(out / f"{cfg['side']}.json", 'w') as fh:
        json.dump(res, fh)
    if lock is not None:
        lock.close()
    return 0


def child_env():
    env = dict(os.environ)
    isolate_dds(env)
    if Path('/etc/nv_tegra_release').exists() or platform.machine() == 'aarch64':
        ver = f'python{sys.version_info.major}.{sys.version_info.minor}'
        cu12 = Path.home() / f'.local/lib/{ver}/site-packages/nvidia/cu12/lib'
        parts = [p for p in env.get('LD_LIBRARY_PATH', '').split(':') if p]
        if cu12.is_dir() and str(cu12) not in parts:
            env['LD_LIBRARY_PATH'] = ':'.join([str(cu12)] + parts)
    return env


def run_side(side, params, frame_sets, out):
    for suffix in ('json', 'npz'):
        stale = out / f'{side}.{suffix}'
        if stale.exists():
            stale.unlink()
    cfg_path = out / f'{side}_config.json'
    with open(cfg_path, 'w') as fh:
        json.dump({'side': side, 'params': params, 'out': str(out),
                   'frame_sets': [[n, str(d)] for n, d in frame_sets]}, fh, indent=1)
    t0 = time.monotonic()
    print(f'side {side}: running ({params}) ...', flush=True)
    with open(out / f'{side}.log', 'w') as log:
        proc = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), '--child', str(cfg_path)],
            env=child_env(), stdout=log, stderr=subprocess.STDOUT, timeout=1800)
    print(f'side {side}: exit {proc.returncode} after {time.monotonic() - t0:.0f} s', flush=True)
    return proc.returncode


# ---------------------------------------------------------------- comparison
def box_iou(a, b):
    ax1, ay1, ax2, ay2 = a['cx'] - a['w'] / 2, a['cy'] - a['h'] / 2, \
        a['cx'] + a['w'] / 2, a['cy'] + a['h'] / 2
    bx1, by1, bx2, by2 = b['cx'] - b['w'] / 2, b['cy'] - b['h'] / 2, \
        b['cx'] + b['w'] / 2, b['cy'] + b['h'] / 2
    iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    union = a['w'] * a['h'] + b['w'] * b['h'] - inter
    return inter / union if union > 0 else 0.0


def match(dets_a, dets_b):
    """Greedy per-class matching by IoU; returns pairs (ia, ib, iou), unmatched a/b."""
    pairs, used_a, used_b = [], set(), set()
    cand = sorted(((box_iou(da, db), ia, ib)
                   for ia, da in enumerate(dets_a) for ib, db in enumerate(dets_b)
                   if da['class_id'] == db['class_id']), reverse=True)
    for iou, ia, ib in cand:
        if iou > 0.0 and ia not in used_a and ib not in used_b:
            pairs.append((ia, ib, iou))
            used_a.add(ia)
            used_b.add(ib)
    return (pairs, [i for i in range(len(dets_a)) if i not in used_a],
            [i for i in range(len(dets_b)) if i not in used_b])


def log_counts(path):
    warn = err = 0
    samples = []
    try:
        with open(path, errors='replace') as fh:
            for line in fh:
                if LOG_ERROR.search(line):
                    err += 1
                elif LOG_WARN.search(line):
                    warn += 1
                else:
                    continue
                text = re.sub(r'\[\d+\.\d+\]', '', line.strip())[:160]
                if text not in samples:
                    samples.append(text)
    except OSError:
        pass
    return warn, err, samples


def check_mask(mm, arr, n_det, h, w):
    """Return an error string or None for one frame's mask message."""
    if mm['encoding'] != 'mono8':
        return f"mask encoding {mm['encoding']}"
    if n_det == 0:
        if mm['height'] * mm['width'] != 0 or mm['len_data'] != 0:
            return f"empty frame mask {mm['height']}x{mm['width']} (want 0x0)"
        return None
    if (mm['height'], mm['width']) != (h, w) or mm['step'] != w or arr is None:
        return f"mask {mm['height']}x{mm['width']} step {mm['step']} (want {h}x{w} step {w})"
    top = int(arr.max())
    if top < 1 or top > n_det:
        return f'mask max label {top} with {n_det} detections'
    return None


def compare(out, perturb=False):
    import numpy as np
    sides = {}
    for side in ('a', 'b'):
        path = out / f'{side}.json'
        if not path.exists():
            return None, [f'{side}: no {path} (child failed, see {out / side}.log)']
        with open(path) as fh:
            sides[side] = json.load(fh)
        if sides[side].get('fatal'):
            return None, [f'{side}: node construction failed: '
                          f"{sides[side]['fatal'].strip().splitlines()[-1]}"]
        npz = out / f'{side}.npz'
        sides[side]['masks'] = np.load(npz) if npz.exists() else {}
    a, b = sides['a'], sides['b']
    if perturb:
        for rec in b['frames']:
            for dm in rec['det_msgs']:
                for det in dm['detections']:
                    det['cx'] += 40.0
    fails = []
    thr = float(a['params'].get('confidence_threshold', 0.3))
    expect_masks = any(r['mask_msgs'] for s in (a, b) for r in s['frames'])
    for side, s in sides.items():
        if s['orphans']:
            fails.append(f"{side}: {len(s['orphans'])} message(s) with a header not equal to "
                         f"any input frame, e.g. {s['orphans'][0]}")
    for name in sorted(set(a['schema']) | set(b['schema'])):
        if a['schema'].get(name) != b['schema'].get(name):
            fails.append(f'{name}: message types/fields differ: '
                         f'A {sorted(a["schema"].get(name, {}))} '
                         f'B {sorted(b["schema"].get(name, {}))}')
    for name, want in (('det_pub', 'Detection2DArray'), ('masks_pub', 'Image')):
        for side, s in sides.items():
            types = list(s['schema'].get(name, {}))
            if (name == 'det_pub' or expect_masks) and (not types or not types[0].endswith(want)):
                fails.append(f'{side}: {name} published {types[:1] or "nothing"}, want {want}')

    b_frames = {(r['set'], r['i']): r for r in b['frames']}
    stats = {}
    for ra in a['frames']:
        rb = b_frames.get((ra['set'], ra['i']))
        st = stats.setdefault(ra['set'], {
            'frames': 0, 'det_frames': [0, 0], 'classes': {}, 'n_det': [0, 0],
            'cnt_mismatch': 0, 'borderline': 0, 'box_iou': [], 'mask_iou': [],
            'exc': [0, 0], 'bad_frames': 0, 'ms': [[], []], 'cpu': [[0.0, 0.0], [0.0, 0.0]]})
        st['frames'] += 1
        tag = f"{ra['set']}#{ra['i']}"
        if rb is None:
            fails.append(f'{tag}: missing on side b')
            st['bad_frames'] += 1
            continue
        bad = []
        dets = []
        for j, (side, r) in enumerate((('a', ra), ('b', rb))):
            if r['exception']:
                st['exc'][j] += 1
                bad.append(f"{side} exception {r['exception'].strip().splitlines()[-1]}")
            if not r['warmup']:
                st['ms'][j].append(r['callback_ms'])
                st['cpu'][j][0] += r['cpu_ms']
                st['cpu'][j][1] += r['wall_ms']
            if len(r['det_msgs']) != 1:
                bad.append(f"{side}: {len(r['det_msgs'])} detection msgs")
            want_masks = 1 if expect_masks else 0
            if len(r['mask_msgs']) != want_masks:
                bad.append(f"{side}: {len(r['mask_msgs'])} mask msgs (want {want_masks})")
            for m in r['det_msgs'] + r['mask_msgs']:
                if m['frame_id'] != r['frame_id']:
                    bad.append(f"{side}: frame_id {m['frame_id']}")
            d = r['det_msgs'][0]['detections'] if r['det_msgs'] else []
            dets.append(d)
            st['n_det'][j] += len(d)
            if d:
                st['det_frames'][j] += 1
            for det in d:
                if det['n_results'] != 1:
                    bad.append(f"{side}: detection with {det['n_results']} results")
                if j == 0:
                    st['classes'][det['class_id']] = st['classes'].get(det['class_id'], 0) + 1
        da, db = dets
        for cls in {x['class_id'] for x in da + db}:
            na = sum(1 for x in da if x['class_id'] == cls)
            nb = sum(1 for x in db if x['class_id'] == cls)
            st['cnt_mismatch'] = max(st['cnt_mismatch'], abs(na - nb))
        pairs, un_a, un_b = match(da, db)
        for side, un, dd in (('a', un_a, da), ('b', un_b, db)):
            for i in un:
                if abs(dd[i]['score'] - thr) <= BORDERLINE:
                    st['borderline'] += 1
                else:
                    bad.append(f"{side}-only {dd[i]['class_id']} score {dd[i]['score']:.2f}")
        for ia, ib, iou in pairs:
            st['box_iou'].append(iou)
            if iou <= BOX_IOU_MIN:
                bad.append(f"{da[ia]['class_id']} box IoU {iou:.3f}")
        arrs = []
        for side, r, d in (('a', ra, da), ('b', rb, db)):
            mm = r['mask_msgs'][0] if len(r['mask_msgs']) == 1 else None
            arr = None
            if mm is not None and mm['key'] is not None:
                arr = sides[side]['masks'][mm['key']]
            arrs.append(arr)
            if mm is not None:
                err = check_mask(mm, arr, len(d), r['h'], r['w'])
                if err:
                    bad.append(f'{side}: {err}')
        if arrs[0] is not None and arrs[1] is not None and arrs[0].shape == arrs[1].shape:
            for ia, ib, _ in pairs:
                ma, mb = arrs[0] == ia + 1, arrs[1] == ib + 1
                union = int(np.logical_or(ma, mb).sum())
                if union:
                    st['mask_iou'].append(int(np.logical_and(ma, mb).sum()) / union)
        if bad:
            st['bad_frames'] += 1
            fails.append(f"{tag}: {'; '.join(bad)}")
    return stats, fails


def fmt(vals, f='{:.3f}'):
    return f.format(vals) if vals is not None else '-'


def print_report(stats, fails, out, sides_params):
    logs = {s: log_counts(out / f'{s}.log') for s in ('a', 'b')}
    info = {}
    for s in ('a', 'b'):
        path = out / f'{s}.json'
        if path.exists():
            with open(path) as fh:
                info[s] = json.load(fh)
    for s in ('a', 'b'):
        missing = [k for k, v in info.get(s, {}).get('has_parameter', {}).items() if not v]
        print(f'side {s}: {info.get(s, {}).get("params", sides_params[s])}')
        print(f'        node {info.get(s, {}).get("node_module", "?")}')
        if missing:
            print(f'        NOT DECLARED by the node (ignored): {", ".join(missing)}')
    if logs['b'][1] > logs['a'][1]:
        fails.append(f"b: {logs['b'][1]} ERROR lines vs {logs['a'][1]} on a (see {out}/b.log)")
    if stats:
        sets = list(stats)
        rows = []

        def row(label, fn):
            rows.append([label] + [fn(stats[n]) for n in sets])

        def med(v):
            return statistics.median(v) if v else None

        def cpu(st, j):
            c, w = st['cpu'][j]
            return f'{100.0 * c / w:.0f}' if w > 0 else '-'

        row('frames', lambda st: str(st['frames']))
        row('frames with detections A/B', lambda st: '{}/{}'.format(*st['det_frames']))
        row('detections A/B', lambda st: '{}/{}'.format(*st['n_det']))
        row('classes seen (A)', lambda st: ' '.join(
            f'{c}:{n}' for c, n in sorted(st['classes'].items())) or '-')
        row('max class-count mismatch', lambda st: str(st['cnt_mismatch']))
        row('borderline unmatched (ignored)', lambda st: str(st['borderline']))
        row('box IoU min / median', lambda st: '{} / {}'.format(
            fmt(min(st['box_iou']) if st['box_iou'] else None), fmt(med(st['box_iou']))))
        row('mask IoU median (info)', lambda st: fmt(med(st['mask_iou'])))
        row('exceptions A/B', lambda st: '{}/{}'.format(*st['exc']))
        row('callback ms mean A/B', lambda st: '{} / {}'.format(
            fmt(statistics.mean(st['ms'][0]) if st['ms'][0] else None, '{:.1f}'),
            fmt(statistics.mean(st['ms'][1]) if st['ms'][1] else None, '{:.1f}')))
        row('process CPU % A/B (100 = 1 core)', lambda st: f'{cpu(st, 0)} / {cpu(st, 1)}')
        row('failing frames', lambda st: str(st['bad_frames']))
        widths = [max(len(r[i]) for r in rows + [[''] + sets]) for i in range(len(sets) + 1)]
        print()
        print('  '.join(h.ljust(w) for h, w in zip([''] + sets, widths)))
        for r in rows:
            print('  '.join(c.ljust(w) for c, w in zip(r, widths)))
        print('(timing excludes the first frame of each run: warm-up)')
    print(f"WARN/ERROR log lines: A {logs['a'][0]}/{logs['a'][1]}  "
          f"B {logs['b'][0]}/{logs['b'][1]}  (logs in {out})")
    for s in ('a', 'b'):
        for line in logs[s][2][:3]:
            print(f'  {s}: {line}')
    if fails:
        print(f'\n{len(fails)} failure(s):')
        for line in fails[:15]:
            print(f'  {line}')
        if len(fails) > 15:
            print(f'  ... {len(fails) - 15} more')
    print('\nRESULT: ' + ('FAIL' if fails else 'PASS'))
    return not fails


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--a', default='', help='side A param overrides k=v,k=v')
    ap.add_argument('--b', default='', help='side B param overrides k=v,k=v')
    ap.add_argument('--frames', action='append', type=Path, help='frame set dir (repeatable)')
    ap.add_argument('--model', type=Path, default=DEFAULT_MODEL)
    ap.add_argument('--out', type=Path, default=DEFAULT_OUT)
    ap.add_argument('--make-people', action='store_true', help='regenerate the people set')
    ap.add_argument('--compare-only', action='store_true',
                    help='compare the existing <out>/a.json and b.json again')
    ap.add_argument('--self-test-perturb', action='store_true', help=argparse.SUPPRESS)
    ap.add_argument('--child', help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.child:
        return run_child(args.child)

    frame_dirs = [p.expanduser().resolve() for p in (args.frames or [CORRIDOR, PEOPLE])]
    for d in frame_dirs:
        if d.name == PEOPLE_NAME and (args.make_people or not (d / 'index.json').exists()):
            make_people(d)
        if not (d / 'index.json').exists():
            raise SystemExit(f'{d}: no index.json')
    names = [d.name for d in frame_dirs]
    if len(set(names)) != len(names):
        raise SystemExit('frame set directory names must be distinct')
    out = args.out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    sides_params = {}
    for side, text in (('a', args.a), ('b', args.b)):
        params = dict(BASE_PARAMS, model_path=str(args.model.expanduser().resolve()))
        params.update(SIDE_DEFAULTS[side])
        params.update(parse_overrides(text))
        sides_params[side] = params
    if not args.compare_only:
        if not Path(sides_params['a']['model_path']).exists():
            raise SystemExit(f"model not found: {sides_params['a']['model_path']}")
        for side in ('a', 'b'):
            run_side(side, sides_params[side], list(zip(names, frame_dirs)), out)
    stats, fails = compare(out, perturb=args.self_test_perturb)
    if args.self_test_perturb:
        print('SELF-TEST: side B boxes shifted by +40 px in x before comparing')
    return 0 if print_report(stats, fails, out, sides_params) else 1


if __name__ == '__main__':
    sys.exit(main())
