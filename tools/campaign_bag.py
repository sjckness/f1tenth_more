"""Read selected topics out of a mission bag (sqlite3 or mcap) -- shared by
tools/replay_front_distance.py and tools/m02_orin_report.py.

Needs a sourced workspace (`source install/setup.bash`): rosbag2_py and the
message packages (f1tenth_messages, vision_msgs, ...) come from it. A topic
whose type cannot be imported is skipped with a warning instead of aborting
the whole read, so a bag recorded by a newer stack still yields the rest.
"""

import math
import pathlib
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def resolve_bag(path):
    """A bag directory from a run folder, a bag folder or a bag file."""
    p = pathlib.Path(path).expanduser()
    if p.is_file():
        p = p.parent
    if (p / 'metadata.yaml').exists():
        return p
    if (p / 'bag' / 'metadata.yaml').exists():
        return p / 'bag'
    hits = sorted(p.glob('**/metadata.yaml'))
    if hits:
        return hits[0].parent
    raise FileNotFoundError(f'no rosbag2 metadata.yaml under {p}')


def bag_topics(bag_dir):
    """{topic: type} for every topic in the bag."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def read_topics(bag_dir, wanted, warn=sys.stderr):
    """{topic: [(t_receive_s, msg), ...]} for the wanted topics present in the bag."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {}
    for topic in wanted:
        if topic not in types:
            continue
        try:
            classes[topic] = get_message(types[topic])
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            print(f'[bag] skipping {topic} ({types[topic]}): {exc}', file=warn)
    out = {topic: [] for topic in wanted}
    if not classes:
        return out
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(classes)))
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        out[topic].append((t_ns * 1e-9, deserialize_message(raw, classes[topic])))
    return out


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def static_poses(tf_static_msgs, base_frame='base_link'):
    """{child_frame: (x, y, z, yaw)} for the DIRECT children of base_frame, and
    one level further (base -> a -> b, translations summed, yaws added; good
    enough for planar mounts)."""
    edges = {}
    for _, msg in tf_static_msgs:
        for tr in msg.transforms:
            t = tr.transform.translation
            edges[tr.child_frame_id] = (tr.header.frame_id, (t.x, t.y, t.z,
                                                             yaw_of(tr.transform.rotation)))
    poses = {}

    def resolve(frame, depth=0):
        if frame == base_frame:
            return (0.0, 0.0, 0.0, 0.0)
        if frame in poses:
            return poses[frame]
        if frame not in edges or depth > 8:
            return None
        parent, (x, y, z, yaw) = edges[frame]
        p = resolve(parent, depth + 1)
        if p is None:
            return None
        px, py, pz, pyaw = p
        c, s = math.cos(pyaw), math.sin(pyaw)
        poses[frame] = (px + c * x - s * y, py + s * x + c * y, pz + z, pyaw + yaw)
        return poses[frame]

    for child in list(edges):
        resolve(child)
    return poses


def nearest(series, t):
    """The (t, msg) of a time-sorted series nearest t, or None."""
    if not series:
        return None
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    best = series[lo]
    if lo > 0 and abs(series[lo - 1][0] - t) < abs(best[0] - t):
        best = series[lo - 1]
    return best


def latest_before(series, t):
    """The last (t, msg) of a time-sorted series received at or before t, or None."""
    lo, hi = 0, len(series)
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    return series[lo - 1] if lo > 0 else None


def pct(values, q):
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return math.nan
    values.sort()
    k = (len(values) - 1) * q / 100.0
    f = int(math.floor(k))
    c = min(f + 1, len(values) - 1)
    return values[f] + (values[c] - values[f]) * (k - f)
