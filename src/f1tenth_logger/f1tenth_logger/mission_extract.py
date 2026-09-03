"""
ROS-dependent half of the mission replay pipeline: bag -> extract.parquet.

Reads a rosbag2 directory and writes everything the renderer draws into one
self-describing Parquet file, so that rendering can happen on a machine with
no ROS at all (see mission_render.py, which imports FROM here's counterpart
direction: this module imports mission_render, never the reverse).

This is the deserialize half of mission_replay_video.py, moved verbatim. The
cut is along a seam the file already had -- one pass over the bag building
Stream objects, then drawing from them -- so nothing about what a frame looks
like changes here. Enforced by test: extract -> render must reproduce the
pre-split MP4's md5 exactly.

EXTRACT FORMAT
--------------
One Parquet table, four columns, one row per record. Row kinds are
distinguished by `kind` rather than by separate files, so a run's extract is a
single artifact that can be copied or synced whole.

    kind     string   'meta' | 'grid' | 'static_tf' | 'sample:<stream>'
    t        double   sample timestamp, seconds, bag receive clock.
                      NULL for meta/grid/static_tf rows.
    payload  string   JSON. Shape depends on kind, see below.
    blob     binary   zlib-compressed raster bytes; only on 'grid' rows,
                      NULL everywhere else.

Samples are stored AT FULL RECORDED RATE, deliberately NOT resampled onto a
fixed grid at extract time. Resampling is the renderer's job (zero-order hold
with a per-stream max age, --dt selects the period), and baking one dt into
the extract would both change the output for any other dt and make the md5
equivalence check meaningless.

Payloads are JSON for CONVENIENCE, not for precision: the structures are
ragged and differently shaped per stream (variable-length horizons, halfspace
lists, corridor polylines keyed by namespace, tuples mixing strings, floats and
null), and one uniform payload column beats a wide mostly-null native schema.
Parquet's own double is IEEE-754 and would round-trip just as exactly. The
tradeoff is a larger, slower file that a non-Python consumer must parse JSON
out of; see EXTRACT_SCHEMA.md for when converting the numeric streams to native
columns would be worth it.

THE OCCUPANCY GRID IS DEDUPLICATED, and it is the one thing here that would
otherwise dominate the file. slam_toolbox republishes the whole map on every
update and the renderer only ever needs "the grid in force at time t", so each
UNIQUE raster is stored once as a 'grid' row (zlib over the int8 bytes) and
each 'sample:map' row carries only {grid_id, extent}. A 14s run typically
holds two or three distinct rasters rather than 140 copies of one.
"""

import argparse
import hashlib
import json
import math
import sys
import zlib
from pathlib import Path

import numpy as np
import yaml

import pyarrow as pa
import pyarrow.parquet as pq

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from f1tenth_logger.mission_render import (
    MAX_AGE, Stream, discover_bags, load_manifest_for, quat_to_matrix,
    yaw_from_quat)

TOPICS = {
    'pose_global': '/ekf_global/odometry/filtered',
    'pose_local': '/odometry/filtered',
    'boundaries': '/costmap/boundaries',
    'clearance': '/costmap/front_clearance',
    'obstacles': '/perception/obstacles_2d',
    'detections': '/camera/detections_3d',
    'markers': '/costmap/semantic_markers',
    'corridor': '/mpc/corridor_markers',
    'tree': '/behavior/tree_status',
    'solver': '/mpc/solver_status',
    'stop': '/safety_stop',
    'drive': '/drive',
    'map': '/slam/map',
    'tf': '/tf',
    'tf_static': '/tf_static',
}


def _decode_legacy_solver_status(raw: bytes):
    """Hand-decode a PRE-horizon MpcSolverStatus (the 9-scalar layout, before
    prediction_frame_id/pred_* were appended) straight from its CDR bytes.

    Appending fields to a .msg is wire-incompatible in the reading direction:
    the installed deserializer expects the longer layout and raises
    "Fast CDR exception" on every older message, which would silently strip the
    solver panel -- the single most useful part of the HUD -- from every bag
    recorded before the change. Rather than lose that, decode the old layout
    directly: plain little-endian CDR with the standard 4-byte-encapsulation
    prefix and natural alignment relative to the body start.

    Returns the same tuple shape read_bag builds for a current message (with an
    empty horizon), or None if the bytes are not that layout either.
    """
    import struct

    if len(raw) < 4:
        return None
    body = raw[4:]                       # skip the encapsulation header
    off = 0

    def align(n):
        nonlocal off
        off += (-off) % n

    def take(fmt, size, alignment):
        nonlocal off
        align(alignment)
        if off + size > len(body):
            raise ValueError('truncated')
        val = struct.unpack_from(fmt, body, off)[0]
        off += size
        return val

    def take_string():
        nonlocal off
        length = take('<I', 4, 4)
        if off + length > len(body):
            raise ValueError('truncated string')
        val = body[off:off + length - 1].decode('utf-8', 'replace') if length else ''
        off += length
        return val

    try:
        take('<i', 4, 4)                 # header.stamp.sec
        take('<I', 4, 4)                 # header.stamp.nanosec
        take_string()                    # header.frame_id
        success = bool(take('<?', 1, 1))
        status = take('<i', 4, 4)
        status_message = take_string()
        solve_dt = take('<f', 4, 4)
        period = take('<f', 4, 4)
        cost = take('<f', 4, 4)
        backend = take_string()
        n_boundary = take('<i', 4, 4)
        n_obstacles = take('<i', 4, 4)
    except (ValueError, struct.error):
        return None
    return (success, status, status_message, solve_dt, period, cost, backend,
            n_boundary, n_obstacles, [], [], [], [], '')


def read_bag(bag_dir: Path, pose_source: str):
    """One pass over the bag, decoding only the topics this video needs.

    Deliberately topic-filtered: these bags carry /camera/image_annotated and
    /camera/detection_masks, and deserializing every image would dominate the
    runtime for pixels this video never draws.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in TOPICS.values() if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))

    pose_topic = TOPICS['pose_global'] if pose_source == 'global' else TOPICS['pose_local']
    data = {
        'pose': Stream(MAX_AGE['pose']),
        'boundaries': Stream(MAX_AGE['boundaries']),
        'clearance': Stream(MAX_AGE['clearance']),
        'obstacles': Stream(MAX_AGE['obstacles']),
        'detections': Stream(MAX_AGE['detections']),
        'markers': Stream(MAX_AGE['markers']),
        'tree': Stream(MAX_AGE['tree']),
        'solver': Stream(MAX_AGE['solver']),
        'stop': Stream(MAX_AGE['stop']),
        'drive': Stream(MAX_AGE['drive']),
        'map': Stream(None),        # occupancy grid: latched-ish, never expires
        'corridor': Stream(MAX_AGE['corridor']),
        'map_to_odom': Stream(MAX_AGE['map_to_odom']),
    }
    static_tf = {}
    pose_frame = None
    t0 = None
    tend = None
    # Semantic markers are a DIFF stream (ADD/DELETE per (ns, id)), so the set
    # of live tracks has to be accumulated as we read and snapshotted per
    # message -- taking a single MarkerArray in isolation would show only the
    # tracks that happened to change that tick.
    live_tracks = {}
    legacy_solver_msgs = [0]
    dropped = {}

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t = t_ns * 1e-9
        if t0 is None:
            t0 = t
        tend = t
        try:
            msg = deserialize_message(raw, get_message(types[topic]))
        except Exception:                       # noqa: BLE001
            # Only one topic in this set has ever changed layout; anything else
            # failing here is a genuinely corrupt/foreign message and is
            # dropped rather than guessed at.
            if topic == TOPICS['solver']:
                legacy = _decode_legacy_solver_status(raw)
                if legacy is not None:
                    data['solver'].add(t, legacy)
                    legacy_solver_msgs[0] += 1
                    continue
            dropped[topic] = dropped.get(topic, 0) + 1
            continue

        if topic == pose_topic:
            pose_frame = msg.header.frame_id
            p = msg.pose.pose
            tw = msg.twist.twist
            data['pose'].add(t, (p.position.x, p.position.y,
                                 yaw_from_quat(p.orientation),
                                 tw.linear.x))
        elif topic == TOPICS['boundaries']:
            data['boundaries'].add(t, [(c.normal[0], c.normal[1], c.offset)
                                       for c in msg.constraints
                                       if math.isfinite(c.offset)])
        elif topic == TOPICS['clearance']:
            data['clearance'].add(t, float(msg.data))
        elif topic == TOPICS['obstacles']:
            data['obstacles'].add(t, [(o.x, o.y, o.r) for o in msg.obstacles])
        elif topic == TOPICS['detections']:
            dets = []
            for det in msg.detections:
                if not det.results:
                    continue
                h = det.results[0]
                var = max(float(h.pose.covariance[0]), float(h.pose.covariance[7]))
                dets.append((h.hypothesis.class_id, float(h.hypothesis.score),
                             h.pose.pose.position.x, h.pose.pose.position.y,
                             h.pose.pose.position.z,
                             math.sqrt(var) if var > 0.0 else None))
            data['detections'].add(t, (msg.header.frame_id, dets))
        elif topic == TOPICS['markers']:
            for mk in msg.markers:
                key = (mk.ns, mk.id)
                if mk.action == 2:                       # DELETE
                    live_tracks.pop(key, None)
                elif mk.type == 3:                       # CYLINDER = track disk
                    live_tracks[key] = ('disk', mk.ns, mk.pose.position.x,
                                        mk.pose.position.y, mk.scale.x * 0.5, '')
                elif mk.type == 9:                       # TEXT = label
                    live_tracks[key] = ('label', mk.ns, mk.pose.position.x,
                                        mk.pose.position.y, 0.0, mk.text)
            data['markers'].add(t, list(live_tracks.values()))
        elif topic == TOPICS['tree']:
            data['tree'].add(t, (msg.active_lane, list(msg.lane_names),
                                 list(msg.lane_statuses), msg.emergency_trip,
                                 bool(msg.safety_stop_active), msg.stop_source))
        elif topic == TOPICS['solver']:
            data['solver'].add(t, (bool(msg.success), int(msg.status),
                                   msg.status_message, float(msg.solve_dt_sec),
                                   float(msg.control_period_sec), float(msg.cost),
                                   msg.solver, int(msg.n_boundary_constraints),
                                   int(msg.n_obstacles),
                                   list(msg.pred_x), list(msg.pred_y),
                                   list(msg.pred_yaw), list(msg.pred_v),
                                   msg.prediction_frame_id))
        elif topic == TOPICS['stop']:
            # frame_id is 'base_link/emergency' or 'base_link/obstacle' -- the
            # only thing distinguishing the two Stop publishers on this topic.
            src = msg.header.frame_id.split('/')[-1] if msg.header.frame_id else ''
            data['stop'].add(t, (src, float(msg.drive.speed)))
        elif topic == TOPICS['drive']:
            data['drive'].add(t, (float(msg.drive.speed),
                                  float(msg.drive.steering_angle)))
        elif topic == TOPICS['map']:
            grid = np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width)
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            res = msg.info.resolution
            data['map'].add(t, (grid, (ox, ox + msg.info.width * res,
                                       oy, oy + msg.info.height * res)))
        elif topic == TOPICS['corridor']:
            # Three LINE_STRIPs keyed by ns; kept as separate polylines rather
            # than merged, because the fill between left and right is built per
            # frame from the pair (and the centerline is drawn on top).
            walls = {}
            for mk in msg.markers:
                if mk.action != 0 or not mk.points:
                    continue
                walls[mk.ns] = [(pt.x, pt.y) for pt in mk.points]
            if walls:
                data['corridor'].add(t, walls)
        elif topic == TOPICS['tf']:
            for tr in msg.transforms:
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                    data['map_to_odom'].add(t, (
                        tr.transform.translation.x, tr.transform.translation.y,
                        yaw_from_quat(tr.transform.rotation)))
        elif topic == TOPICS['tf_static']:
            for tr in msg.transforms:
                static_tf[tr.child_frame_id] = (
                    tr.header.frame_id,
                    np.array([tr.transform.translation.x,
                              tr.transform.translation.y,
                              tr.transform.translation.z]),
                    quat_to_matrix(tr.transform.rotation.x, tr.transform.rotation.y,
                                   tr.transform.rotation.z, tr.transform.rotation.w))

    return {'streams': data, 'static_tf': static_tf, 't0': t0, 't1': tend,
            'pose_frame': pose_frame, 'pose_topic': pose_topic,
            'legacy_solver_msgs': legacy_solver_msgs[0], 'dropped': dropped}


# --------------------------------------------------------------------------
# extract writing
# --------------------------------------------------------------------------
_SCHEMA = pa.schema([
    pa.field('kind', pa.string()),
    pa.field('t', pa.float64()),
    pa.field('payload', pa.string()),
    pa.field('blob', pa.binary()),
])


def _jsonable(name, value, grid_ids, grid_rows):
    """
    Flatten one stream sample to something json.dumps accepts.

    Only `map` needs real work: its numpy raster is interned into grid_rows by
    content hash so an unchanged map costs one small reference per tick instead
    of a full copy. Everything else is already tuples/lists/scalars of floats
    and strings.
    """
    if name != 'map':
        return value
    grid, extent = value
    digest = hashlib.sha256(grid.tobytes()).hexdigest()
    if digest not in grid_ids:
        grid_ids[digest] = len(grid_ids)
        grid_rows.append({
            'kind': 'grid',
            't': None,
            'payload': json.dumps({'grid_id': grid_ids[digest],
                                   'shape': list(grid.shape),
                                   'sha256': digest}),
            'blob': zlib.compress(grid.tobytes(), 6),
        })
    return {'grid_id': grid_ids[digest], 'extent': list(extent)}


# Fallbacks used only when a run has no params snapshot (hand-made bags, and
# any run recorded before the snapshot was written). They match
# stack_params.yaml's current values, but a run that falls back is FLAGGED as
# such in the extract rather than silently presented as authoritative.
_PADDING_FALLBACK = {'car_radius': 0.20, 'avoidance_margin': 0.12}


def padding_from_params(params_snapshot_path):
    """
    Read this run's own car_radius / avoidance_margin out of its params snapshot.

    THE POINT: these two set how far inside each hard boundary the tightened
    (dashed) line is drawn. They are run properties, not viewing preferences,
    so re-rendering an old run after a config change must keep drawing the
    padding that run actually flew with. Reading them from the live config, or
    from a constant in the renderer, would silently redraw history.

    MPC_corr.py's avoidance_margin is fed from stack_params.yaml's
    obstacle_safety_margin_m (see that key's own comment), so that is the key
    read here -- not a key literally named avoidance_margin, which does not
    exist.
    """
    result = dict(_PADDING_FALLBACK, source='fallback')
    if not params_snapshot_path:
        return result
    path = Path(params_snapshot_path)
    if not path.is_file():
        return result
    try:
        params = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError:
        return result
    for name, key in (('car_radius', 'car_radius'),
                      ('avoidance_margin', 'obstacle_safety_margin_m')):
        entry = params.get(key)
        if isinstance(entry, dict) and 'default' in entry:
            result[name] = float(entry['default'])
            result['source'] = str(path)
    return result


def write_extract(bag, out_path, manifest=None, run_id=None, padding=None):
    """
    Serialize a read_bag() result to `out_path` as Parquet.

    Everything needed to rebuild the bag dict goes in, including the per-stream
    max_age values: those are the renderer's staleness gates and hard-coding
    them on the read side would silently desync the two halves the first time
    one changed.
    """
    grid_ids = {}
    grid_rows = []
    sample_rows = []
    for name, stream in bag['streams'].items():
        for t, value in zip(stream.t, stream.v):
            sample_rows.append({
                'kind': f'sample:{name}',
                't': float(t),
                'payload': json.dumps(_jsonable(name, value, grid_ids, grid_rows)),
                'blob': None,
            })

    static_rows = [{
        'kind': 'static_tf',
        't': None,
        'payload': json.dumps({'child': child, 'parent': parent,
                               'translation': translation.tolist(),
                               'matrix': matrix.tolist()}),
        'blob': None,
    } for child, (parent, translation, matrix) in bag['static_tf'].items()]

    meta_row = {
        'kind': 'meta',
        't': None,
        'payload': json.dumps({
            'extract_version': 1,
            't0': bag['t0'],
            't1': bag['t1'],
            'pose_frame': bag['pose_frame'],
            'pose_topic': bag['pose_topic'],
            'legacy_solver_msgs': bag['legacy_solver_msgs'],
            'dropped': bag['dropped'],
            'max_age': {name: stream.max_age
                        for name, stream in bag['streams'].items()},
            'manifest': manifest or {},
            'run_id': run_id,
            # This run's own boundary padding, NOT the renderer's defaults --
            # see padding_from_params above for why that distinction matters.
            'padding': padding or dict(_PADDING_FALLBACK, source='unset'),
        }),
        'blob': None,
    }

    rows = [meta_row] + grid_rows + static_rows + sample_rows
    table = pa.Table.from_pydict(
        {key: [row[key] for row in rows] for key in ('kind', 't', 'payload', 'blob')},
        schema=_SCHEMA)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, out_path, compression='zstd')
    return out_path


def extract_bag(bag_dir, out_path, pose_source='global'):
    """Read one bag and write its extract. Returns (out_path, bag)."""
    bag_dir = Path(bag_dir)
    bag = read_bag(bag_dir, pose_source)
    manifest = load_manifest_for(bag_dir)
    run_id = manifest.get('run_id', bag_dir.name)
    padding = padding_from_params(manifest.get('params_snapshot_path'))
    return write_extract(bag, out_path, manifest, run_id, padding), bag


def main(argv=None):
    """Extract one or more bags to *.extract.parquet."""
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='*', type=Path,
                    help='bag directories; default is the newest --count under '
                         '--bag-root')
    ap.add_argument('--bag-root', type=Path,
                    default=Path.home() / '.ros' / 'mission_bags')
    ap.add_argument('--count', type=int, default=3)
    ap.add_argument('--out-dir', type=Path, default=None,
                    help='default: alongside each bag directory')
    ap.add_argument('--pose-source', choices=('global', 'local'), default='global')
    cfg = ap.parse_args(argv)

    if cfg.bags:
        bag_dirs = [b.expanduser().resolve() for b in cfg.bags]
    else:
        found = discover_bags(cfg.bag_root.expanduser(), cfg.count)
        if not found:
            print(f'no mission bags found under {cfg.bag_root}', file=sys.stderr)
            return 1
        bag_dirs = [d for _t, d, _m in found]

    for bag_dir in bag_dirs:
        out_dir = cfg.out_dir.expanduser() if cfg.out_dir else bag_dir.parent
        manifest = load_manifest_for(bag_dir)
        run_id = manifest.get('run_id', bag_dir.name)
        out_path, _bag = extract_bag(
            bag_dir, out_dir / f'{run_id}.extract.parquet', cfg.pose_source)
        bag_bytes = sum(f.stat().st_size for f in bag_dir.rglob('*') if f.is_file())
        size = out_path.stat().st_size
        ratio = (bag_bytes / size) if size else float('inf')
        print(f'{run_id}: {size / 1e6:.2f} MB extract from '
              f'{bag_bytes / 1e6:.1f} MB bag ({ratio:.0f}x smaller) -> {out_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
