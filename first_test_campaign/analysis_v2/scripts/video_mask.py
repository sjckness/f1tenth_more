"""Segmentation-mask videos for the ``analysis_v2`` pipeline (VIDEO_MASK).

Segmentation code used (pinned, see ``.sjck/interfaces/
analysis_v2_controller_perception.md`` section 2, SCOUT_PERCEPTION):

    branch  campaign-final-yolo-gpu
    commit  5792e2bad18d0dad0febc8c3f8cadfd336f4aa27
            (2026-10-22 16:01, "yolo_detector_node: GPU pipeline
            (gpu_pipeline, default on) + smoke test")
    file    src/f1tenth_perception/f1tenth_perception/yolo_detector_node.py
    model   yolo26s-seg.pt (stack_params.yaml ``yolo_model``; the default set
            by commit 96c6bbc, which is an ancestor of 5792e2ba, so the two
            agree)

This module does NOT import that ROS node (it needs rclpy, cv_bridge and a
live graph). It reproduces its classic per-frame path offline, with the same
constants, read with ``git show 5792e2ba:<file>``:

* ``YOLO(model)(frame, device='cuda', conf=0.3, retina_masks=True)``
  (``image_callback``, confidence_threshold 0.3);
* the car's own LiDAR exclusion rectangle: x 0.75-1.0, y 0.55-1.0 of the
  frame, a detection whose mask has more than 50% of its pixels inside the
  rectangle is dropped (``_lidar_exclusion_keep_mask``,
  ``_mask_overlap_fraction``);
* the mask overlay: one color per class, the MD5 of the class name with
  bytes 0/1/2 as R/G/B, blended with alpha 0.5, and a "class (score)" label
  above each mask (``_draw_mask_overlay``, ``_class_color_bgr``,
  ``MASK_OVERLAY_ALPHA``).

What each selected run (``maschera=si`` in ``selection.yaml``) produces:

``<run>.mp4``
    The run has clean RGB frames in its bag (``CLEAN_TOPICS``). They are
    segmented again on the GPU and the masks are drawn on top.
``<run>_recorded.mp4``
    The bag holds only frames that were already annotated while the car was
    driving (``ANNOTATED_TOPICS``, masks drawn into the pixels). Those frames
    are NOT segmented again: drawing new masks over old ones would make the
    result look like a fresh segmentation when it is not. The frame says
    "RECORDED" and the run summary says so too.
no video
    No bag, a bag with no camera frames, or a bag that is a symlink or
    incomplete. The run summary gives the reason. This is an expected case,
    not an error.
skipped
    Clean frames exist but a GPU or model dependency is missing. Nothing is
    installed: the run summary gives the exact command to run.

Every video shows the run name, the time since test_start
(``meta.extra_meta.ros_start_time``; if that is missing, the time since the
first frame, marked as such) and a fixed legend of class colors. The legend
is fixed for the whole video: ``LEGEND_BASE_CLASSES`` followed by any other
class seen anywhere in the run.

Bags are found in this order (``analysis_v2_data.md`` section 4):
``first_test_campaign/<Mxx>/<run>/`` (a ``bag/`` folder or ``*.db3``/
``*.mcap`` next to a ``metadata.yaml``), then
``~/f1tenth_archive/{complete,active,incomplete}/<UTC>_mission-<plan_id>/bag``.
The archive is matched on the bag start in UTC, which is between 0 and 90 s
after the test_id's local time (Europe/Rome). A bag whose ``plan_id`` suffix
matches is preferred, then the smallest offset. Bags are opened read-only.
ROS (``rosbag2_py``) is imported only when frames are actually read; this is
the only place in the pipeline that needs it.

Usage (from ``first_test_campaign/analysis_v2``)::

    python3 -m scripts.video_mask                 # every maschera=si run
    python3 -m scripts.video_mask --runs P001-R008 --dry-run
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np
import yaml

try:  # package import (python3 -m scripts.video_mask / from scripts.video_mask)
    from .run_data import (
        DEFAULT_CAMPAIGN,
        RunResolutionError,
        find_root,
        group_key,
        load_selection,
        parse_test_id,
        resolve_run,
    )
except ImportError:  # direct script run: python3 scripts/video_mask.py
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from run_data import (  # type: ignore[no-redef]
        DEFAULT_CAMPAIGN,
        RunResolutionError,
        find_root,
        group_key,
        load_selection,
        parse_test_id,
        resolve_run,
    )

__all__ = [
    "SEG_BRANCH",
    "SEG_COMMIT",
    "SEG_FILE",
    "MissingDependency",
    "MaskVideoResult",
    "Segmenter",
    "class_color_bgr",
    "find_run_bag",
    "classify_bag",
    "render_mask_video",
    "group_folder",
    "main",
]

# --------------------------------------------------------------------------
# pinned segmentation source (SCOUT_PERCEPTION, interface section 2)
# --------------------------------------------------------------------------

SEG_BRANCH = "campaign-final-yolo-gpu"
SEG_COMMIT = "5792e2bad18d0dad0febc8c3f8cadfd336f4aa27"
SEG_FILE = "src/f1tenth_perception/f1tenth_perception/yolo_detector_node.py"
SEG_MODEL_NAME = "yolo26s-seg.pt"

# constants mirrored from SEG_FILE @ SEG_COMMIT
CONFIDENCE_THRESHOLD = 0.3
MASK_OVERLAY_ALPHA = 0.5
LIDAR_EXCLUSION = (0.75, 1.0, 0.55, 1.0)  # x_min, x_max, y_min, y_max (fractions)
LIDAR_EXCLUSION_OVERLAP_THRESHOLD = 0.5

# --------------------------------------------------------------------------
# bag topics
# --------------------------------------------------------------------------

#: Camera frames with nothing drawn on them, in order of preference.
CLEAN_TOPICS = (
    "/camera/image_raw",
    "/camera/image_raw_5hz",
    "/camera/debug_image/clean/compressed",
)
#: Frames annotated while the car was driving (masks already drawn in).
ANNOTATED_TOPICS = (
    "/camera/image_annotated",
    "/camera/debug_image/compressed",
)
DETECTIONS_TOPIC = "/camera/detections"

#: Classes always in the legend, in this order (the campaign's targets).
LEGEND_BASE_CLASSES = ("person", "chair", "bottle", "potted plant", "tv")

ARCHIVE_SUBDIRS = ("complete", "active", "incomplete")
ARCHIVE_MAX_OFFSET_S = 90.0
LOCAL_TZ = "Europe/Rome"
OUTPUT_FPS = 10.0
VIDEO_SUBDIR = "video_maschera"

# exact commands reported when a dependency is missing (never run here)
_INSTALL_CMDS = {
    "rosbag2_py": "sudo apt install ros-humble-rosbag2-py ros-humble-rosidl-runtime-py "
                  "&& source /opt/ros/humble/setup.bash",
    "ultralytics": "python3 -m pip install --user ultralytics==8.4.90",
    "torch": "sudo apt install nvidia-jetpack  # then the JetPack torch wheel with CUDA "
             "(pip3 install --user torch from the NVIDIA Jetson index)",
    "cuda": "sudo apt install nvidia-jetpack  # torch.cuda.is_available() is False",
    "cv2": "sudo apt install python3-opencv",
    "ffmpeg": "sudo apt install ffmpeg",
}


class MissingDependency(RuntimeError):
    """A GPU, model or library dependency is missing. Carries the fix command."""

    def __init__(self, what, command):
        self.what = what
        self.command = command
        super().__init__(f"missing dependency {what}: {command}")


@dataclass
class MaskVideoResult:
    run: str
    status: str  # resegmented | recorded | no_images | skipped | failed
    video: str | None = None
    reason: str = ""
    bag_dir: str | None = None
    topic: str | None = None
    n_frames: int = 0
    classes: list = field(default_factory=list)
    install_command: str | None = None
    seg_branch: str = SEG_BRANCH
    seg_commit: str = SEG_COMMIT


# --------------------------------------------------------------------------
# drawing (mirrors SEG_FILE)
# --------------------------------------------------------------------------

def class_color_bgr(class_id):
    """Fixed per-class color in BGR, the same as ``_class_color_bgr`` in SEG_FILE."""
    digest = hashlib.md5(str(class_id).encode("utf-8")).digest()
    r, g, b = int(digest[0]), int(digest[1]), int(digest[2])
    return (b, g, r)


def _cv2():
    try:
        import cv2  # noqa: PLC0415
    except ImportError as exc:
        raise MissingDependency("cv2", _INSTALL_CMDS["cv2"]) from exc
    return cv2


def draw_mask_overlay(frame_bgr, instances):
    """Draw the masks and labels the same way as ``_draw_mask_overlay`` in SEG_FILE.

    ``instances`` is a list of ``(mask_bool_hw, class_name, score)``.
    """
    cv2 = _cv2()
    annotated = frame_bgr.copy()
    if not instances:
        return annotated
    overlay = frame_bgr.copy()
    drawn = []
    for mask, class_id, score in instances:
        if not np.any(mask):
            continue
        color = class_color_bgr(class_id)
        overlay[mask] = color
        drawn.append((mask, color, class_id, score))
    cv2.addWeighted(overlay, MASK_OVERLAY_ALPHA, annotated, 1.0 - MASK_OVERLAY_ALPHA,
                    0.0, dst=annotated)
    img_h, img_w = annotated.shape[:2]
    for mask, color, class_id, score in drawn:
        ys, xs = np.where(mask)
        label_x = int(xs.mean())
        label_y = max(int(ys.min()) - 6, 12)
        text = f"{class_id} ({score:.2f})"
        (text_w, text_h), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        box_w, box_h = text_w + 4, text_h + baseline + 2
        bottom_right = (
            min(max(label_x - text_w // 2 - 2, 0) + box_w, img_w - 1),
            min(max(label_y - text_h - 2, 0) + box_h, img_h - 1))
        top_left = (max(bottom_right[0] - box_w, 0), max(bottom_right[1] - box_h, 0))
        cv2.rectangle(annotated, top_left, bottom_right, color, thickness=-1)
        cv2.putText(annotated, text, (top_left[0] + 2, bottom_right[1] - baseline - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return annotated


def draw_hud(frame_bgr, run, t_label, mode_label, legend_classes):
    """Add a top strip (run name, time, mode) and a fixed legend panel on the right.

    The frame is padded, so neither the strip nor the legend covers the image.
    """
    cv2 = _cv2()
    fh, w = frame_bgr.shape[:2]
    panel_w, strip_h = 170, 44
    h = fh + strip_h
    canvas = np.full((h, w + panel_w, 3), (17, 17, 18), dtype=np.uint8)
    canvas[strip_h:, :w] = frame_bgr
    cv2.putText(canvas, f"{run}   {t_label}", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    mode_color = (80, 200, 255) if mode_label.startswith("RECORDED") else (120, 230, 120)
    cv2.putText(canvas, mode_label, (6, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                mode_color, 1, cv2.LINE_AA)
    # legend
    x0 = w + 8
    ly = 20
    cv2.putText(canvas, "legend", (x0, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    ly += 8
    for name in legend_classes:
        if ly + 16 > h:
            break
        color = class_color_bgr(name)
        cv2.rectangle(canvas, (x0, ly), (x0 + 14, ly + 14), color, thickness=-1)
        cv2.putText(canvas, name, (x0 + 20, ly + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
        ly += 20
    return canvas


def legend_for(classes_seen):
    extra = sorted(c for c in set(classes_seen) if c and c not in LEGEND_BASE_CLASSES)
    return list(LEGEND_BASE_CLASSES) + extra


# --------------------------------------------------------------------------
# segmentation (GPU, lazy)
# --------------------------------------------------------------------------

def default_model_path(root=None):
    root = find_root(root)
    return root / "src" / "f1tenth_perception" / "models" / SEG_MODEL_NAME


class Segmenter:
    """Offline copy of SEG_FILE's classic inference path. Loads the model on first use."""

    def __init__(self, model_path=None, device="cuda"):
        self.model_path = Path(model_path) if model_path else default_model_path()
        self.device = device
        self._model = None
        self._names = {}

    def check(self):
        """Raise :class:`MissingDependency` with the fix command if anything is missing."""
        if not self.model_path.is_file():
            raise MissingDependency(
                f"model weights {self.model_path}",
                f"cp ~/dev_ws/f1tenth_more_yolo_gpu/src/f1tenth_perception/models/"
                f"{SEG_MODEL_NAME} {self.model_path}")
        try:
            import torch  # noqa: PLC0415
        except ImportError as exc:
            raise MissingDependency("torch", _INSTALL_CMDS["torch"]) from exc
        if str(self.device).startswith("cuda") and not torch.cuda.is_available():
            raise MissingDependency("CUDA (torch.cuda.is_available() is False)",
                                    _INSTALL_CMDS["cuda"])
        try:
            import ultralytics  # noqa: F401, PLC0415
        except ImportError as exc:
            raise MissingDependency("ultralytics", _INSTALL_CMDS["ultralytics"]) from exc
        _cv2()

    def _load(self):
        if self._model is None:
            self.check()
            from ultralytics import YOLO  # noqa: PLC0415
            self._model = YOLO(str(self.model_path))
            self._names = dict(getattr(self._model, "names", {}) or {})
        return self._model

    def __call__(self, frame_bgr):
        """Return ``[(mask_bool_hw, class_name, score), ...]`` after the LiDAR exclusion."""
        model = self._load()
        result = model(frame_bgr, verbose=False, device=self.device,
                       conf=CONFIDENCE_THRESHOLD, retina_masks=True)[0]
        if result.boxes is None or len(result.boxes) == 0:
            return []
        h, w = frame_bgr.shape[:2]
        ex1, ex2 = LIDAR_EXCLUSION[0] * w, LIDAR_EXCLUSION[1] * w
        ey1, ey2 = LIDAR_EXCLUSION[2] * h, LIDAR_EXCLUSION[3] * h
        masks = result.masks.data.cpu().numpy() if result.masks is not None else None
        names = result.names or self._names
        out = []
        for i, box in enumerate(result.boxes):
            cls_id = int(box.cls[0].item())
            score = float(box.conf[0].item())
            if masks is not None and np.any(masks[i] > 0.5):
                mask = masks[i] > 0.5
                overlap = _mask_overlap_fraction(mask, ex1, ey1, ex2, ey2)
            else:
                mask = np.zeros((h, w), dtype=bool)
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
                overlap = _bbox_overlap_fraction(x1, y1, x2, y2, ex1, ey1, ex2, ey2)
            if overlap > LIDAR_EXCLUSION_OVERLAP_THRESHOLD:
                continue
            if mask.shape != (h, w):  # retina_masks=True should make this impossible
                mask = _cv2().resize(mask.astype(np.uint8), (w, h)) > 0
            out.append((mask, str(names.get(cls_id, cls_id)), score))
        return out


def _bbox_overlap_fraction(x1, y1, x2, y2, ex1, ey1, ex2, ey2):
    ix1, iy1 = max(x1, ex1), max(y1, ey1)
    ix2, iy2 = min(x2, ex2), min(y2, ey2)
    if ix2 <= ix1 or iy2 <= iy1:
        return 0.0
    area = (x2 - x1) * (y2 - y1)
    if area <= 0.0:
        return 0.0
    return ((ix2 - ix1) * (iy2 - iy1)) / area


def _mask_overlap_fraction(mask, ex1, ey1, ex2, ey2):
    total = int(np.count_nonzero(mask))
    if total == 0:
        return 0.0
    h, w = mask.shape[:2]
    cx1, cy1 = max(int(ex1), 0), max(int(ey1), 0)
    cx2, cy2 = min(int(ex2), w), min(int(ey2), h)
    if cx2 <= cx1 or cy2 <= cy1:
        return 0.0
    return int(np.count_nonzero(mask[cy1:cy2, cx1:cx2])) / total


# --------------------------------------------------------------------------
# bag discovery (read-only, no ROS)
# --------------------------------------------------------------------------

def _read_meta(run_dir):
    path = Path(run_dir) / "meta.json"
    try:
        with open(path, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, json.JSONDecodeError):
        return {}


def _test_start_utc(test_id):
    when = parse_test_id(test_id)["datetime"]
    try:
        from zoneinfo import ZoneInfo  # noqa: PLC0415
        local = when.replace(tzinfo=ZoneInfo(LOCAL_TZ))
        return local.astimezone(timezone.utc).replace(tzinfo=None)
    except Exception:  # noqa: BLE001 - no tzdata: CEST until 2026-10-25 03:00, CET after
        dst_end = datetime(2026, 10, 25, 3, 0, 0)
        return when - timedelta(hours=2 if when < dst_end else 1)


def _bag_in_run_dir(run_dir):
    run_dir = Path(run_dir)
    for cand in (run_dir / "bag", run_dir):
        if (cand / "metadata.yaml").exists() or any(cand.glob("*.db3")) or any(cand.glob("*.mcap")):
            return cand
    return None


def find_run_bag(run_dir, test_id=None, archive_root=None):
    """Return ``(bag_dir, how)``, or ``(None, reason)`` if the run has no bag."""
    run_dir = Path(run_dir)
    test_id = test_id or run_dir.name
    local = _bag_in_run_dir(run_dir)
    if local is not None:
        return local, "run folder"

    archive_root = Path(archive_root) if archive_root else Path.home() / "f1tenth_archive"
    try:
        start = _test_start_utc(test_id)
    except ValueError:
        return None, f"test_id {test_id!r} has no timestamp, cannot match the archive"
    plan_id = (_read_meta(run_dir).get("extra_meta") or {}).get("plan_id")

    cands = []
    for sub in ARCHIVE_SUBDIRS:
        base = archive_root / sub
        if not base.is_dir():
            continue
        for d in base.iterdir():
            try:
                t = datetime.strptime(d.name[:19], "%Y-%m-%dT%H-%M-%S")
            except ValueError:
                continue
            off = (t - start).total_seconds()
            if 0.0 <= off <= ARCHIVE_MAX_OFFSET_S:
                pid_ok = bool(plan_id) and d.name.endswith(f"_mission-{plan_id}")
                cands.append((0 if pid_ok else 1, off, d))
    if not cands:
        return None, (f"no bag in the run folder and none in {archive_root}/"
                      f"{{{','.join(ARCHIVE_SUBDIRS)}}} starting 0-{ARCHIVE_MAX_OFFSET_S:.0f} s "
                      f"after {start.isoformat()}Z")
    cands.sort(key=lambda c: (c[0], c[1]))
    _, off, d = cands[0]
    return d / "bag", f"archive {d.parent.name}/{d.name} (+{off:.0f} s)"


def classify_bag(bag_dir):
    """Read ``metadata.yaml`` only (no ROS).

    Returns a dict with ``ok``, ``reason``, ``storage_id``, ``clean`` (best
    clean topic with frames, or None), ``annotated`` (same for annotated
    frames) and ``counts``.
    """
    bag_dir = Path(bag_dir)
    info = {"ok": False, "reason": "", "storage_id": None, "clean": None,
            "annotated": None, "counts": {}}
    if bag_dir.is_symlink():
        info["reason"] = f"bag {bag_dir} is a symlink (not copied yet), skipped"
        return info
    meta_path = bag_dir / "metadata.yaml"
    if not meta_path.is_file() or meta_path.is_symlink():
        info["reason"] = f"bag {bag_dir} has no metadata.yaml (incomplete), skipped"
        return info
    try:
        with open(meta_path, encoding="utf-8") as fh:
            meta = (yaml.safe_load(fh) or {}).get("rosbag2_bagfile_information")
    except (OSError, yaml.YAMLError) as exc:
        info["reason"] = f"bag {bag_dir}: metadata.yaml unreadable ({exc}), skipped"
        return info
    if not meta:
        info["reason"] = f"bag {bag_dir}: metadata.yaml empty (bag not closed), skipped"
        return info
    files = [bag_dir / p for p in meta.get("relative_file_paths") or []]
    bad = [str(p) for p in files if not p.is_file() or p.is_symlink() or p.stat().st_size == 0]
    if not files or bad:
        info["reason"] = f"bag {bag_dir}: data files missing, empty or symlinked {bad or '(none listed)'}, skipped"
        return info
    info["storage_id"] = meta.get("storage_identifier") or "sqlite3"
    counts = {}
    for entry in meta.get("topics_with_message_count") or []:
        tm = entry.get("topic_metadata") or {}
        counts[tm.get("name")] = int(entry.get("message_count") or 0)
    info["counts"] = counts
    info["clean"] = next((t for t in CLEAN_TOPICS if counts.get(t, 0) > 0), None)
    info["annotated"] = next((t for t in ANNOTATED_TOPICS if counts.get(t, 0) > 0), None)
    info["ok"] = True
    return info


# --------------------------------------------------------------------------
# bag reading (ROS, lazy)
# --------------------------------------------------------------------------

def _ros():
    try:
        import rosbag2_py  # noqa: PLC0415
        from rclpy.serialization import deserialize_message  # noqa: PLC0415
        from rosidl_runtime_py.utilities import get_message  # noqa: PLC0415
    except ImportError as exc:
        raise MissingDependency("rosbag2_py/rclpy", _INSTALL_CMDS["rosbag2_py"]) from exc
    return rosbag2_py, deserialize_message, get_message


def _msg_to_bgr(msg):
    cv2 = _cv2()
    if hasattr(msg, "format") and not hasattr(msg, "encoding"):  # CompressedImage
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return img
    enc = msg.encoding.lower()
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, step)
    if enc in ("bgr8", "rgb8"):
        img = raw[:, : w * 3].reshape(h, w, 3)
        return img.copy() if enc == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc in ("bgra8", "rgba8"):
        img = raw[:, : w * 4].reshape(h, w, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR if enc == "bgra8" else cv2.COLOR_RGBA2BGR)
    if enc == "mono8":
        return cv2.cvtColor(raw[:, :w].copy(), cv2.COLOR_GRAY2BGR)
    return None


def read_frames(bag_dir, topic, storage_id="sqlite3", detections_topic=DETECTIONS_TOPIC):
    """Yield ``("frame", stamp_s, bag_t_s, bgr)`` and ``("det", stamp_s, bag_t_s, [class,...])``.

    Read-only. ``stamp_s`` is the header stamp (sensor time).
    """
    rosbag2_py, deserialize_message, get_message = _ros()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in (topic, detections_topic) if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    while reader.has_next():
        name, data, t_ns = reader.read_next()
        msg = deserialize_message(data, classes[name])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if name == detections_topic:
            labels = [d.results[0].hypothesis.class_id for d in msg.detections if d.results]
            yield "det", stamp, t_ns * 1e-9, labels
            continue
        img = _msg_to_bgr(msg)
        if img is not None:
            yield "frame", stamp, t_ns * 1e-9, img


# --------------------------------------------------------------------------
# video writing
# --------------------------------------------------------------------------

def _write_video(path, frames, times, fps=OUTPUT_FPS):
    """Write ``frames`` (BGR, same size) at their real times, holding each one until the next.

    ``frames`` is a list of encoded PNG bytes, decoded one at a time to keep memory low.
    """
    cv2 = _cv2()
    first = cv2.imdecode(np.frombuffer(frames[0], np.uint8), cv2.IMREAD_COLOR)
    h, w = first.shape[:2]
    w2, h2 = w + (w % 2), h + (h % 2)
    t0, t_end = times[0], times[-1]
    n_out = max(1, int(round((t_end - t0) * fps)) + 1)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".part.mp4")
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise MissingDependency("ffmpeg", _INSTALL_CMDS["ffmpeg"])
    cmd = [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
           "-s", f"{w2}x{h2}", "-r", f"{fps:g}", "-i", "-", "-c:v", "libx264",
           "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p",
           "-movflags", "+faststart", str(tmp)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    idx, cur, cur_idx = 0, first, 0
    try:
        for k in range(n_out):
            t = t0 + k / fps
            while idx + 1 < len(times) and times[idx + 1] <= t + 1e-9:
                idx += 1
            if idx != cur_idx:
                cur = cv2.imdecode(np.frombuffer(frames[idx], np.uint8), cv2.IMREAD_COLOR)
                cur_idx = idx
            out = cur
            if (w2, h2) != (w, h):
                out = cv2.copyMakeBorder(cur, 0, h2 - h, 0, w2 - w, cv2.BORDER_CONSTANT)
            proc.stdin.write(np.ascontiguousarray(out).tobytes())
    finally:
        proc.stdin.close()
        rc = proc.wait()
    if rc != 0:
        tmp.unlink(missing_ok=True)
        raise RuntimeError(f"ffmpeg exited with {rc} writing {path}")
    tmp.replace(path)
    return n_out


# --------------------------------------------------------------------------
# main entry point per run
# --------------------------------------------------------------------------

def group_folder(run_dir, ostacolo):
    """Canonical RULES.md §6/§9/§11 group folder name (see ``run_data.group_key``)."""
    mission = Path(run_dir).parent.name
    return group_key(mission, ostacolo)


def render_mask_video(run_dir, out_dir, *, run=None, segmenter=None, bag_dir=None,
                      archive_root=None, dry_run=False, fps=OUTPUT_FPS):
    """Make the mask video of one run in ``out_dir``. Never raises for an expected case.

    Returns a :class:`MaskVideoResult`. ``status`` is one of:

    ``resegmented``
        ``<run>.mp4`` written from clean frames segmented again on the GPU.
    ``recorded``
        ``<run>_recorded.mp4`` written from frames annotated while driving,
        NOT segmented again.
    ``no_images``
        No bag, or no camera frames in the bag. No video.
    ``skipped``
        Bag is a symlink or incomplete, or a dependency is missing
        (``install_command``). No video.
    ``failed``
        Unexpected error while reading or writing (``reason``).

    With ``dry_run=True`` only the decision is made: no ROS, GPU or video work.
    """
    run_dir = Path(run_dir)
    run = run or run_dir.name
    out_dir = Path(out_dir)
    res = MaskVideoResult(run=run, status="no_images")

    if bag_dir is None:
        bag_dir, how = find_run_bag(run_dir, run, archive_root)
        if bag_dir is None:
            res.reason = how
            return res
    else:
        how = "given"
    bag_dir = Path(bag_dir)
    res.bag_dir = str(bag_dir)

    info = classify_bag(bag_dir)
    if not info["ok"]:
        res.status, res.reason = "skipped", info["reason"]
        return res

    if info["clean"]:
        mode, topic = "resegmented", info["clean"]
    elif info["annotated"]:
        mode, topic = "recorded", info["annotated"]
    else:
        present = {t: info["counts"][t] for t in CLEAN_TOPICS + ANNOTATED_TOPICS
                   if t in info["counts"]}
        res.reason = (f"bag found ({how}) but it holds no camera frames: "
                      f"{present or 'no camera topic recorded'}")
        return res
    res.topic = topic

    if mode == "resegmented":
        segmenter = segmenter or Segmenter()
        try:
            segmenter.check()
        except MissingDependency as exc:
            res.status, res.install_command = "skipped", exc.command
            res.reason = f"clean frames on {topic} but {exc.what} is missing"
            return res

    video = out_dir / (f"{run}.mp4" if mode == "resegmented" else f"{run}_recorded.mp4")
    res.status = mode
    res.video = str(video)
    if mode == "recorded":
        res.reason = (f"only frames annotated while driving ({topic}, {how}): masks are the "
                      f"ones drawn at run time, NOT segmented again with {SEG_COMMIT[:8]}")
    else:
        res.reason = f"clean frames ({topic}, {how}) segmented again with {SEG_COMMIT[:8]}"
    if dry_run:
        res.n_frames = info["counts"].get(topic, 0)
        return res

    try:
        cv2 = _cv2()
        ros_start = (_read_meta(run_dir).get("extra_meta") or {}).get("ros_start_time")
        frames, stamps, seen = [], [], []
        for kind, stamp, _bag_t, payload in read_frames(bag_dir, topic, info["storage_id"]):
            if kind == "det":
                if mode == "recorded":
                    seen.extend(payload)
                continue
            img = payload
            if mode == "resegmented":
                instances = segmenter(img)
                seen.extend(name for _, name, _ in instances)
                img = draw_mask_overlay(img, instances)
            ok, buf = cv2.imencode(".png", img)
            if ok:
                frames.append(buf.tobytes())
                stamps.append(stamp)
        if not frames:
            res.status, res.video = "no_images", None
            res.reason = f"{topic} listed in the bag but no frame could be decoded"
            return res

        order = np.argsort(stamps, kind="stable")
        frames = [frames[i] for i in order]
        stamps = [stamps[i] for i in order]
        if ros_start:
            t0, t_tag = float(ros_start), "t"
        else:
            t0, t_tag = stamps[0], "t (from 1st frame, no ros_start_time)"
        legend = legend_for(seen)
        mode_label = ("RECORDED at run time - not re-segmented"
                      if mode == "recorded" else
                      f"re-segmented {SEG_MODEL_NAME} @ {SEG_COMMIT[:8]}")
        hud_frames = []
        for buf, stamp in zip(frames, stamps):
            img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
            canvas = draw_hud(img, run, f"{t_tag} = {stamp - t0:6.2f} s", mode_label, legend)
            ok, enc = cv2.imencode(".png", canvas)
            hud_frames.append(enc.tobytes())
        del frames
        _write_video(video, hud_frames, stamps, fps=fps)
        res.n_frames = len(hud_frames)
        res.classes = sorted(set(seen))
    except MissingDependency as exc:
        res.status, res.video = "skipped", None
        res.install_command = exc.command
        res.reason = f"{exc.what} is missing"
    except Exception as exc:  # noqa: BLE001 - one bad run never stops the others
        res.status, res.video = "failed", None
        res.reason = f"{type(exc).__name__}: {exc}"
    return res


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def _summary_lines(results):
    lines = []
    for r in results:
        line = f"{r.run:28s} {r.status:12s} {r.video or '-'}"
        if r.reason:
            line += f"\n{'':41s}{r.reason}"
        if r.install_command:
            line += f"\n{'':41s}command: {r.install_command}"
        lines.append(line)
    recorded = [r.run for r in results if r.status == "recorded"]
    cmds = sorted({r.install_command for r in results if r.install_command})
    lines.append("")
    lines.append(f"segmentation: {SEG_BRANCH} @ {SEG_COMMIT}")
    for status in ("resegmented", "recorded", "no_images", "skipped", "failed"):
        n = sum(1 for r in results if r.status == status)
        lines.append(f"  {status:12s} {n}")
    if recorded:
        lines.append("RECORDED ONLY (annotated at run time, NOT re-segmented): "
                     + ", ".join(recorded))
    if cmds:
        lines.append("commands needed (not run):")
        lines.extend(f"  {c}" for c in cmds)
    return lines


def main(argv=None):
    here = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--selection", type=Path, default=here / "selection.yaml")
    ap.add_argument("--out-root", type=Path, default=None,
                    help="default: <analysis_v2>/<output_folder from selection.yaml>")
    ap.add_argument("--runs", nargs="*", default=None,
                    help="only these run codes (full test_id or P0nn-R0nn)")
    ap.add_argument("--model", type=Path, default=None)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--archive-root", type=Path, default=None)
    ap.add_argument("--dry-run", action="store_true",
                    help="decide per run, do not read frames or write videos")
    args = ap.parse_args(argv)

    selection = load_selection(args.selection)
    out_root = args.out_root or (args.selection.resolve().parent / selection.output_folder)
    campaign = find_root() / DEFAULT_CAMPAIGN
    segmenter = Segmenter(args.model, args.device)

    results = []
    for row in selection:
        if not row.maschera:
            continue
        if args.runs and not any(row.run == c or row.run.startswith(f"{c}-") for c in args.runs):
            continue
        try:
            run_dir = resolve_run(row.run, [campaign])
        except RunResolutionError as exc:
            results.append(MaskVideoResult(run=row.run, status="failed", reason=str(exc)))
            continue
        out_dir = out_root / group_folder(run_dir, row.ostacolo) / VIDEO_SUBDIR
        res = render_mask_video(run_dir, out_dir, run=row.run, segmenter=segmenter,
                                archive_root=args.archive_root, dry_run=args.dry_run)
        results.append(res)
        print(f"[video_mask] {res.run}: {res.status} {res.video or ''} {res.reason}", flush=True)

    print("\n".join(_summary_lines(results)))
    if not args.dry_run:
        out_root.mkdir(parents=True, exist_ok=True)
        with open(out_root / "video_maschera_summary.json", "w", encoding="utf-8") as fh:
            json.dump({"seg_branch": SEG_BRANCH, "seg_commit": SEG_COMMIT,
                       "runs": [asdict(r) for r in results]}, fh, indent=2)
    return 0 if all(r.status != "failed" for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
