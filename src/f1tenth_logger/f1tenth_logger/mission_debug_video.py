"""
Perception debug video: a run's recorded debug JPEGs -> one small MP4, on the host.

The car never encodes video. yolo_detector_node publishes an already-drawn,
already-JPEG-compressed frame at debug_image_rate_hz on
/camera/debug_image/compressed and the mission logger records it; this tool
reads those JPEG bytes straight out of the run's bag and hands them to ffmpeg.

    python3 -m f1tenth_logger.mission_debug_video <run_dir> [<run_dir> ...]
    ros2 run f1tenth_logger mission_debug_video <run_dir> --out-dir DIR

<run_dir> is a mission_logger run folder (the one holding bag/), a bag
directory, or a .db3 file. Output: <out_dir>/<run_id>.debug.mp4.

THE RUN-REPORT OVERLAY FIRST (--source auto, the default). When the run has a
clean camera frame and a scene mask (mission_overlay's module docstring),
every camera frame is drawn with the mask and the LiDAR on it, here, and those
frames are the video; the app's Run view offers it under the same name. A run
without that data, or a host without OpenCV or f1tenth_perception, gets the
recorded debug JPEGs as before. --source recorded always takes the JPEGs,
--source overlay never does. --alpha / --point-px / --no-legend override
stack_params.yaml's mission_report_overlay_*.

NO ROS NEEDED, like mission_render.py: the bag is a sqlite3 database and a
sensor_msgs/CompressedImage is decoded by hand from its CDR bytes (header,
format, data), the same technique mission_extract._decode_legacy_solver_status
uses. Only sqlite3 bags are read; the car records sqlite3 because the mcap
plugin is not installed there (see mission_logger_node's STORAGE FORMAT note).

WHY NOT THROUGH THE EXTRACT. mission_render.py draws from the per-run
*.extract.parquet, which the car writes at finalize and which is topic-filtered
on purpose to keep images out. Copying ~16 KiB JPEGs at 5 Hz (~82 KB/s) into
it would make it about four times larger (P004-R018: 190 KB for a 7.3 s bag,
~26 KB/s) and add car-side work at finalize, for frames that need no
processing at all.

TIMING. Each frame is shown for the gap between its capture stamp and the next
one's (clamped to [0.01, 2.0] s), through ffmpeg's concat demuxer, and the
output is resampled to a constant --fps, so a 5 Hz stream with dropped frames
still plays back in real time.
"""

import argparse
import os
from pathlib import Path
import sqlite3
import struct
import subprocess
import sys
import tempfile

DEBUG_IMAGE_TOPIC = '/camera/debug_image/compressed'
MIN_FRAME_S = 0.01
MAX_FRAME_S = 2.0


def decode_compressed_image(raw):
    """
    Decode a CDR sensor_msgs/CompressedImage into (stamp_sec, format, data).

    Little-endian CDR with the 4-byte encapsulation header and alignment
    relative to the body start. Raises ValueError on a truncated or
    big-endian message.
    """
    if len(raw) < 4 or raw[1] != 0x01:
        raise ValueError('not a little-endian CDR message')
    body = memoryview(raw)[4:]
    off = 0

    def take(fmt, size):
        nonlocal off
        off += (-off) % size
        if off + size > len(body):
            raise ValueError('truncated')
        value = struct.unpack_from(fmt, body, off)[0]
        off += size
        return value

    def take_bytes():
        nonlocal off
        n = take('<I', 4)
        if off + n > len(body):
            raise ValueError('truncated')
        value = bytes(body[off:off + n])
        off += n
        return value

    sec = take('<i', 4)
    nanosec = take('<I', 4)
    take_bytes()                                  # header.frame_id
    fmt = take_bytes().rstrip(b'\0').decode('utf-8', 'replace')
    data = take_bytes()
    return sec + nanosec * 1e-9, fmt, data


def find_db3(path):
    """Return the sorted .db3 files of a run dir, a bag dir or a .db3 path."""
    path = Path(path).expanduser()
    if path.suffix == '.db3':
        return [path]
    bag = path / 'bag' if (path / 'bag').is_dir() else path
    return sorted(bag.glob('*.db3'))


def run_id_of(path):
    """Return the run id: the run folder's name (the parent of bag/)."""
    path = Path(path).expanduser().resolve()
    if path.suffix == '.db3':
        path = path.parent
    return path.parent.name if path.name == 'bag' else path.name


def read_frames(db3_files, topic=DEBUG_IMAGE_TOPIC):
    """Return [(stamp_sec, jpeg_bytes)] for topic, sorted by capture stamp."""
    frames = []
    for db3 in db3_files:
        con = sqlite3.connect(f'file:{db3}?mode=ro', uri=True)
        try:
            row = con.execute('SELECT id FROM topics WHERE name = ?', (topic,)).fetchone()
            if row is None:
                continue
            for (raw,) in con.execute(
                    'SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp',
                    (row[0],)):
                try:
                    stamp, fmt, data = decode_compressed_image(bytes(raw))
                except ValueError:
                    continue
                if 'jpeg' in fmt.lower() or 'jpg' in fmt.lower():
                    frames.append((stamp, data))
        finally:
            con.close()
    frames.sort(key=lambda f: f[0])
    return frames


def frame_durations(stamps):
    """Return each frame's on-screen duration from the gap to the next stamp."""
    if not stamps:
        return []
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    typical = sorted(gaps)[len(gaps) // 2] if gaps else 0.2
    gaps.append(typical)
    return [min(max(g, MIN_FRAME_S), MAX_FRAME_S) for g in gaps]


def write_mp4(frames, out_path, fps=10, crf=28):
    """
    Encode frames, an iterable of (stamp, jpeg), into out_path with ffmpeg; return out_path.

    The JPEGs go to a temporary directory as they come, so a generator of
    drawn frames is never held in memory whole. ValueError when there are none.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='debug_video_') as tmp:
        stamps = []
        for i, (stamp, jpeg) in enumerate(frames):
            with open(os.path.join(tmp, f'f{i:06d}.jpg'), 'wb') as fh:
                fh.write(jpeg)
            stamps.append(stamp)
        if not stamps:
            raise ValueError('no frames')
        lines = ['ffconcat version 1.0']
        for i, duration in enumerate(frame_durations(stamps)):
            lines += [f"file 'f{i:06d}.jpg'", f'duration {duration:.4f}']
        # The concat demuxer ignores the last entry's duration unless the file
        # is listed once more.
        lines.append(f"file 'f{len(stamps) - 1:06d}.jpg'")
        listing = os.path.join(tmp, 'frames.ffconcat')
        with open(listing, 'w') as fh:
            fh.write('\n'.join(lines) + '\n')
        cmd = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
               '-f', 'concat', '-safe', '0', '-i', listing,
               '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', '-r', str(fps),
               '-c:v', 'libx264', '-preset', 'veryfast', '-crf', str(crf),
               '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(out_path)]
        subprocess.run(cmd, check=True)
    return out_path


def open_overlay(run, style_overrides=None):
    """Return mission_overlay.open_run, or (None, reason) where it cannot run."""
    try:
        from f1tenth_logger import mission_overlay
    except ImportError as exc:
        return None, f'overlay unavailable on this host: {exc}'
    try:
        return mission_overlay.open_run(
            run, mission_overlay.style_from_params(**(style_overrides or {})))
    except Exception as exc:  # noqa: BLE001 - a bad overlay must not cost the video
        return None, f'overlay failed: {exc}'


def render_run(run, out_dir, topic=DEBUG_IMAGE_TOPIC, fps=10, crf=28, source='auto',
               style_overrides=None):
    """
    Write <out_dir>/<run_id>.debug.mp4 for one run; return its path, or None.

    The run-report overlay's frames when `source` allows it and the run has
    them (module docstring), else the recorded JPEGs on topic. None, with the
    reason on stderr, when the run has no sqlite3 bag, no frames, or ffmpeg
    fails.
    """
    db3 = find_db3(run)
    if not db3:
        print(f'{run}: no .db3 found (only sqlite3 bags are supported)', file=sys.stderr)
        return None
    overlay = frames = None
    what = topic
    if source != 'recorded':
        overlay, why = open_overlay(run, style_overrides)
        if overlay is not None:
            frames = overlay.jpegs()
            what = f'overlay on {overlay.camera.name}, {overlay.mask_source} mask'
        elif source == 'overlay':
            print(f'{run}: no overlay ({why})', file=sys.stderr)
            return None
        else:
            print(f'{run}: no overlay ({why}) -- the recorded {topic} frames instead',
                  file=sys.stderr)
    out = Path(out_dir).expanduser() / f'{run_id_of(run)}.debug.mp4'
    stamps = []

    def counted(source_frames):
        for stamp, jpeg in source_frames:
            stamps.append(stamp)
            yield stamp, jpeg

    try:
        if frames is not None:
            try:
                write_mp4(counted(frames), out, fps, crf)
            except Exception as exc:  # noqa: BLE001 - a bad overlay must not cost the video
                print(f'{run}: overlay failed ({exc})', file=sys.stderr)
                if source == 'overlay':
                    return None
                frames, what = None, topic
                stamps.clear()
        if frames is None:
            frames = read_frames(db3, topic)
            if not frames:
                print(f'{run}: no {topic} frames', file=sys.stderr)
                return None
            write_mp4(counted(frames), out, fps, crf)
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f'{run}: ffmpeg failed: {exc}', file=sys.stderr)
        return None
    finally:
        if overlay is not None:
            overlay.close()
    span = stamps[-1] - stamps[0]
    print(f'{out}  ({len(stamps)} frames over {span:.1f} s, {what})')
    return out


def main(argv=None):
    """Turn each run's overlay frames, or its recorded debug JPEGs, into an MP4. No ROS."""
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', type=Path,
                    help='run folders, bag directories or .db3 files')
    ap.add_argument('--out-dir', type=Path, default=None,
                    help='default: the workspace-root mission_videos/ mission_render uses')
    ap.add_argument('--topic', default=DEBUG_IMAGE_TOPIC)
    ap.add_argument('--fps', type=int, default=10)
    ap.add_argument('--crf', type=int, default=28)
    ap.add_argument('--source', choices=('auto', 'overlay', 'recorded'), default='auto',
                    help='auto: the run-report overlay when the run has it, else the '
                         'recorded --topic JPEGs')
    ap.add_argument('--alpha', type=float, default=None,
                    help='overlay mask fill opacity (default: mission_report_overlay_alpha)')
    ap.add_argument('--point-px', type=int, default=None,
                    help='overlay LiDAR dot size (default: mission_report_overlay_point_px)')
    ap.add_argument('--no-legend', dest='legend', action='store_false', default=None)
    cfg = ap.parse_args(argv)
    overrides = {'alpha': cfg.alpha, 'point_px': cfg.point_px, 'legend': cfg.legend}

    out_dir = cfg.out_dir
    if out_dir is None:
        from f1tenth_logger.mission_render import DEFAULT_OUT_DIR
        out_dir = DEFAULT_OUT_DIR
    if out_dir is None:
        ap.error('--out-dir is required: no source workspace above this module.')

    written = [render_run(run, out_dir, cfg.topic, cfg.fps, cfg.crf, cfg.source, overrides)
               for run in cfg.runs]
    return 0 if any(written) else 1


if __name__ == '__main__':
    sys.exit(main())
