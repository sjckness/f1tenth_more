#!/usr/bin/env python3
"""
Write the camera frames of a bag as image files: the YOLO parity input.

Runs on: anywhere with the side's ROS Python (rosbag2_py). Build the image
set ONCE (on either side) and give the same directory to yolo_run.py on both
sides (P9.5).

A sensor_msgs/CompressedImage topic (a car mission bag's
/camera/debug_image/clean/compressed: the detector's tick frame with nothing
drawn on it) is written byte for byte (.jpg / .png as its format says, no
re-encoding); a sensor_msgs/Image topic (bgr8 / rgb8 / mono8) is written as
lossless PNG.

usage: extract_frames.py BAG TOPIC OUT_DIR [--every N] [--max M]
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..',
                                'scripts', 'jazzy_parity'))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('topic')
    ap.add_argument('out')
    ap.add_argument('--every', type=int, default=1, help='keep every N-th frame')
    ap.add_argument('--max', type=int, default=200, help='at most M frames')
    a = ap.parse_args(argv)
    from bag_read import read_topic
    if os.path.exists(a.out) and os.listdir(a.out):
        raise SystemExit('%s is not empty (nothing is overwritten)' % a.out)
    os.makedirs(a.out, exist_ok=True)
    n = kept = 0
    for _, msg in read_topic(a.bag, a.topic):
        n += 1
        if (n - 1) % a.every:
            continue
        if hasattr(msg, 'format'):  # CompressedImage
            ext = 'png' if 'png' in msg.format.lower() else 'jpg'
            with open(os.path.join(a.out, 'frame_%05d.%s' % (kept, ext)), 'wb') as f:
                f.write(bytes(msg.data))
        else:
            import cv2
            import numpy as np
            ch = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(msg.encoding)
            if ch is None:
                raise SystemExit('unsupported encoding %s' % msg.encoding)
            img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                msg.height, msg.step // ch if ch else msg.width, ch)[:, :msg.width]
            if msg.encoding == 'rgb8':
                img = img[:, :, ::-1]
            cv2.imwrite(os.path.join(a.out, 'frame_%05d.png' % kept), img)
        kept += 1
        if kept >= a.max:
            break
    print('%d of %d frames of %s written to %s' % (kept, n, a.topic, a.out))
    return 0 if kept else 1


if __name__ == '__main__':
    sys.exit(main())
