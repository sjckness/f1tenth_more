#!/usr/bin/env python3
"""bag_to_video.py -- what Hydra saw, as a video: the RGB beside the GT semantic mask.

    scripts/hydra/bag_to_video.py RUN_DIR_OR_BAG [--out video.mp4] [--fps 10] [--max-frames N]

The PNGs the recorder drops are one per sim second, enough to glance at and far too sparse to
watch. The bag has the full 10 Hz, so the video comes from there; no sim and no Hydra needed,
only the bag on disk.

Left is `/camera/image_raw` (bgra8), right is `/sim/ground_truth/left/semantic` (mono8 class ids)
through scripts/hydra/semantic_palette.py, with a legend of the ids actually present. Frames are
paired by nearest timestamp, so a dropped frame on either side does not shift the two apart.

ffmpeg is not installed here, so this writes through OpenCV's VideoWriter: H.264 (avc1) when the
build has it, falling back to MPEG-4 Visual (mp4v), which any OpenCV can write. Both go in an mp4
container and play in VLC; H.264 also plays in browsers and on phones.

A note if you ever check the result with OpenCV: `VideoCapture(path)` may pick the GStreamer
backend and fail on a perfectly good file. Ask for FFMPEG explicitly --
`cv2.VideoCapture(path, cv2.CAP_FFMPEG)` -- before concluding the video is broken.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from semantic_palette import NAMES, PALETTE, UNKNOWN, colourise  # noqa: E402

RGB_TOPIC = "/camera/image_raw"
SEM_TOPIC = "/sim/ground_truth/left/semantic"


def read_images(bag: str, topics):
    """{topic: [(stamp_ns, HxWx3 BGR or HxW ids)]}, decoded straight from the bag."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    if missing:
        raise SystemExit(f"{bag}: not recorded: {', '.join(missing)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))

    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, _ts = reader.read_next()
        m = deserialize_message(data, get_message(types[topic]))
        a = np.frombuffer(m.data, dtype=np.uint8)
        stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        if m.encoding == "mono8":
            img = a.reshape(m.height, m.step)[:, :m.width]
        elif m.encoding in ("bgra8", "rgba8"):
            img = a.reshape(m.height, m.step // 4, 4)[:, :m.width, :3]
            if m.encoding == "rgba8":
                img = img[:, :, ::-1]
        elif m.encoding in ("bgr8", "rgb8"):
            img = a.reshape(m.height, m.step // 3, 3)[:, :m.width, :]
            if m.encoding == "rgb8":
                img = img[:, :, ::-1]
        else:
            raise SystemExit(f"{topic}: unhandled encoding {m.encoding}")
        out[topic].append((stamp, np.ascontiguousarray(img)))
    for t in out:
        out[t].sort(key=lambda p: p[0])
    return out


def legend(frame, ids, cv2) -> None:
    """A swatch and a name per id present, bottom-left of the mask half."""
    x, y = 8, frame.shape[0] - 8 - 14 * len(ids)
    for i in ids:
        r, g, b = PALETTE.get(i, UNKNOWN)
        cv2.rectangle(frame, (x, y), (x + 12, y + 10), (int(b), int(g), int(r)), -1)
        cv2.putText(frame, f"{i} {NAMES.get(i, '?')}", (x + 18, y + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
        y += 14


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", help="a run directory (its bag/ is used) or a bag directory")
    ap.add_argument("--out", default="", help="default: <run>/what_hydra_saw.mp4")
    ap.add_argument("--fps", type=float, default=10.0, help="source is 10 Hz, so 10 = real time")
    ap.add_argument("--fourcc", default="", help="default: avc1 if this OpenCV can, else mp4v")
    ap.add_argument("--max-frames", type=int, default=0)
    a = ap.parse_args()

    import cv2

    run = os.path.abspath(os.path.expanduser(a.run))
    bag = os.path.join(run, "bag") if os.path.isdir(os.path.join(run, "bag")) else run
    out = a.out or os.path.join(run if bag != run else os.path.dirname(run),
                                "what_hydra_saw.mp4")

    print(f"reading {bag}")
    data = read_images(bag, (RGB_TOPIC, SEM_TOPIC))
    rgb, sem = data[RGB_TOPIC], data[SEM_TOPIC]
    print(f"  {len(rgb)} RGB, {len(sem)} mask frames")
    if not rgb or not sem:
        raise SystemExit("one of the two streams is empty")

    sem_t = np.array([s for s, _ in sem], dtype=np.int64)
    t0 = rgb[0][0]
    h, w = rgb[0][1].shape[:2]
    writer = None
    for fourcc in ([a.fourcc] if a.fourcc else ["avc1", "mp4v"]):
        cand = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*fourcc), a.fps, (w * 2, h))
        if cand.isOpened():
            writer, used = cand, fourcc
            break
        cand.release()
    if writer is None:
        raise SystemExit(f"OpenCV cannot write {out} with any of avc1/mp4v")
    print(f"  codec {used}")

    n = len(rgb) if not a.max_frames else min(a.max_frames, len(rgb))
    worst_pair_ms = 0.0
    for k in range(n):
        stamp, left = rgb[k]
        j = int(np.argmin(np.abs(sem_t - stamp)))          # nearest mask, not the k-th
        worst_pair_ms = max(worst_pair_ms, abs(int(sem_t[j]) - stamp) / 1e6)
        ids_img = sem[j][1]
        right_rgb, ids = colourise(ids_img)
        right = np.ascontiguousarray(right_rgb[:, :, ::-1])   # to BGR for OpenCV
        frame = np.hstack([left, right])
        cv2.putText(frame, f"t {(stamp - t0) / 1e9:6.1f}s   frame {k + 1}/{n}", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, "camera", (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, "GT semantic mask (what seg_source:=gt feeds Hydra)", (w + 8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        legend(frame[:, w:], ids, cv2)
        writer.write(frame)
        if (k + 1) % 200 == 0:
            print(f"  {k + 1}/{n}")
    writer.release()

    print(f"\n{out}  ({os.path.getsize(out) / 1e6:.1f} MB, {n} frames at {a.fps:g} fps, "
          f"{n / a.fps:.1f} s)")
    print(f"worst RGB/mask pairing error: {worst_pair_ms:.1f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
