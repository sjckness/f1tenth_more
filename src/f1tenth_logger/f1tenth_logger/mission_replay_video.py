"""
Bag to MP4 in one step, on a machine that has ROS.

This file used to hold the whole pipeline. It is now a thin composition of the
two halves it was split into, so the single-command workflow it always had
still works unchanged:

    ros2 run f1tenth_logger mission_replay_video               # 3 newest runs
    ros2 run f1tenth_logger mission_replay_video --count 5
    ros2 run f1tenth_logger mission_replay_video <bag_dir> [<bag_dir> ...]
    ros2 run f1tenth_logger mission_replay_video --speed 0.5   # slow motion
    ros2 run f1tenth_logger mission_replay_video --follow 6.0  # camera follows

WHY THE SPLIT: rendering does not need ROS, but reading a bag does, and the
reports are now produced on a machine that has no ROS installed at all. The
seam was already in the file (one pass deserializing into Stream objects, then
drawing from them), so:

    mission_extract.py   bag -> <run_id>.extract.parquet   (needs ROS)
    mission_render.py    extract -> MP4                    (needs NO ROS)

Nothing about a frame changed. The claim is checked rather than asserted: the
reference run re-rendered through extract+render matches the pre-split MP4's
md5 exactly.

Videos are written to <workspace root>/mission_videos/<run_id>.mp4 (gitignored)
unless --out-dir says otherwise; --out-dir is REQUIRED when this module has no
source workspace above it (a plain non-symlink install), rather than guessing
at the cwd.

Requires a sourced ROS 2 workspace (rosbag2_py + f1tenth_messages), matplotlib,
pyarrow and ffmpeg. For the ROS-free path, see mission_render.py.
"""

import argparse
import sys
import time
from pathlib import Path

from f1tenth_logger.mission_extract import read_bag
from f1tenth_logger.mission_render import (
    DEFAULT_BAG_ROOT, DEFAULT_OUT_DIR, discover_bags, load_manifest_for,
    render_bag)


def render_bag_dir(bag_dir: Path, cfg):
    """Read one bag with ROS and hand it straight to the ROS-free renderer."""
    manifest = load_manifest_for(bag_dir)
    print(f'[{bag_dir.name}] reading...')
    read_start = time.time()
    bag = read_bag(bag_dir, cfg.pose_source)
    print(f'[{bag_dir.name}] bag read in {time.time() - read_start:.1f}s')
    return render_bag(bag, manifest, bag_dir.name, cfg)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='*', type=Path,
                    help='explicit bag directories; default is the newest --count '
                         'missions under --bag-root')
    ap.add_argument('--bag-root', type=Path, default=DEFAULT_BAG_ROOT)
    ap.add_argument('--count', type=int, default=3)
    ap.add_argument('--out-dir', type=Path, default=None,
                    help=(f'default: {DEFAULT_OUT_DIR}' if DEFAULT_OUT_DIR
                          else 'REQUIRED here: no source workspace was found '
                               'above this module, so there is no default'))
    ap.add_argument('--dt', type=float, default=0.1,
                    help="resample period [s]; default 0.1 = the MPC's control period")
    ap.add_argument('--speed', type=float, default=1.0,
                    help='playback speed (1.0 = real time, 0.5 = slow motion)')
    ap.add_argument('--follow', type=float, default=None, metavar='HALF_WIDTH_M',
                    help='follow the car with this half-width instead of framing '
                         'the whole run')
    ap.add_argument('--trail-seconds', type=float, default=3.0)
    ap.add_argument('--corridor-span', type=float, default=5.0, metavar='M',
                    help='how far along each hard boundary to draw')
    ap.add_argument('--pose-source', choices=('global', 'local'), default='global',
                    help='global: map-frame EKF. local: odom-frame EKF.')
    ap.add_argument('--car-radius', type=float, default=0.20)
    ap.add_argument('--avoidance-margin', type=float, default=0.12)
    ap.add_argument('--dpi', type=int, default=100)
    ap.add_argument('--bitrate', type=int, default=4000)
    ap.add_argument('--no-map', action='store_true', help='skip the SLAM grid layer')
    ap.add_argument('--no-progress', dest='progress', action='store_false',
                    default=True)
    cfg = ap.parse_args(argv)

    if cfg.bags:
        bag_dirs = [b.expanduser().resolve() for b in cfg.bags]
    else:
        found = discover_bags(cfg.bag_root.expanduser(), cfg.count)
        if not found:
            print(f'no mission bags found under {cfg.bag_root}', file=sys.stderr)
            return 1
        bag_dirs = [d for _t, d, _m in found]
        print('selected (newest first):')
        for st, d, m in found:
            print(f'  {st}  {m.get("outcome", "?"):9s}  {d.name}')

    out_dir = cfg.out_dir or DEFAULT_OUT_DIR
    if out_dir is None:
        ap.error('--out-dir is required: this module has no src/ ancestor to '
                 'derive the workspace-root mission_videos/ from (a plain '
                 'non-symlink colcon install). Pass --out-dir explicitly.')
    cfg.out_dir = out_dir.expanduser()

    outputs = []
    for bag_dir in bag_dirs:
        if not (bag_dir / 'metadata.yaml').exists():
            print(f'{bag_dir}: not a rosbag2 directory; skipped', file=sys.stderr)
            continue
        try:
            out = render_bag_dir(bag_dir, cfg)
        except Exception as exc:                       # noqa: BLE001 - one bad bag
            print(f'{bag_dir.name}: FAILED: {exc}', file=sys.stderr)  # must not
            continue                                   # abort the others
        if out:
            outputs.append(out)

    if outputs:
        print('\nwrote:')
        for out in outputs:
            print(f'  {out}')
    return 0 if outputs else 1


if __name__ == '__main__':
    sys.exit(main())
