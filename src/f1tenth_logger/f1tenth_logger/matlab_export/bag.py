"""An archive rosbag2 -> per-topic structs. The supplement.

Read with ``rosbags`` (pure Python, no ROS runtime). The bag opens read-only
(``immutable=1``), so nothing in the archive can be modified.

Humble bags carry no message definitions, so every type that is not in
rosbags' built-in Humble store is registered from ``.msg`` files: this
repo's ``f1tenth_messages`` plus ``ackermann_msgs`` and ``vision_msgs`` from
the ROS install (``$AMENT_PREFIX_PATH``, then ``/opt/ros/humble``).

Time: ``t_ros`` is the message's header stamp; a message without a header, or
with a zero stamp, uses the bag's receive time instead. The topic's
``time_source`` says which: ``header``, ``receive`` or ``header+receive``.

Typed extractors cover the topics whose layout a MATLAB user expects (odometry
as x y yaw vx vy wz, drive commands, LaserScan as a matrix, the last map);
every other recorded topic goes through ``flatten_messages``.
"""

import dataclasses
import math
import os
from pathlib import Path

import numpy as np

from f1tenth_logger.matlab_export.convert import cell, empty_column, mat_name, str_cell

SOURCE = "bag"

#: Never exported: images, masks, tf, visualisation markers, diagnostics.
EXCLUDED = (
    "/camera/image_annotated", "/camera/detection_masks", "/camera/detection_markers",
    "/costmap/semantic_markers", "/mpc/corridor_markers", "/tf", "/tf_static",
    "/diagnostics", "/diagnostics/system_status",
)

#: topic -> (struct name, extractor kind). Topics not listed, and not
#: excluded, are exported under mat_name(topic) with the generic flattener.
TOPICS = {
    "/odom": ("odom", "odometry"),
    "/odometry/filtered": ("ekf_local", "odometry"),
    "/ekf_global/odometry/filtered": ("ekf_global", "odometry"),
    "/slam/pose": ("slam_pose", "pose_cov"),
    "/slam/pose_calibrated": ("slam_pose_cal", "pose_cov"),
    "/drive": ("drive_mpc", "ackermann"),
    "/ackermann_drive": ("drive_out", "ackermann"),
    "/safety_stop": ("safety_stop", "ackermann"),
    "/scan": ("scan", "scan"),
    "/slam/map": ("map", "map"),
    "/costmap/front_clearance": ("front_clear", "generic"),
    "/mpc/goal_distance": ("goal_distance", "generic"),
    "/mpc/hold": ("hold", "generic"),
    "/mpc/goal_object_end": ("goal_object_end", "generic"),
    "/mpc/goal_pose": ("goal_pose", "pose"),
    "/mpc/solver_status": ("solver", "generic"),
    "/mpc/drive_clamp": ("clamp", "generic"),
    "/mpc/goal_object": ("goal_object", "generic"),
    "/mpc/goal_turn": ("goal_turn", "generic"),
    "/mpc/object_status": ("object_status", "generic"),
    "/costmap/boundaries": ("boundaries", "generic"),
    "/costmap/semantic_tracks": ("sem_tracks", "generic"),
    "/perception/obstacles_2d": ("obstacles2d", "generic"),
    "/camera/detections": ("det2d", "generic"),
    "/camera/detections_3d": ("det3d", "generic"),
    "/behavior/tree_status": ("bt", "generic"),
    "/mission/status": ("mission_status", "generic"),
    "/mission/move_outcome": ("move_outcome", "generic"),
}

MSG_PACKAGES = ("f1tenth_messages", "ackermann_msgs", "vision_msgs")


# --------------------------------------------------------------------------
# type registration
# --------------------------------------------------------------------------

def msg_dirs(package, repo_root=None):
    """Candidate ``msg/`` folders for ``package``, best first."""
    dirs = []
    if repo_root is not None:
        dirs.append(Path(repo_root) / "src" / package / "msg")
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            dirs.append(Path(prefix) / "share" / package / "msg")
    dirs.append(Path("/opt/ros/humble/share") / package / "msg")
    return [d for d in dirs if d.is_dir()]


def make_typestore(repo_root=None, extra_msg_dirs=()):
    """Humble's built-in store plus the custom packages' .msg definitions."""
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    store = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    found = {}
    for package in MSG_PACKAGES:
        dirs = msg_dirs(package, repo_root)
        if dirs:
            found[package] = dirs[0]
    for package, folder in list(found.items()) + [(Path(d).parent.name, Path(d))
                                                  for d in extra_msg_dirs]:
        for path in sorted(Path(folder).glob("*.msg")):
            name = f"{package}/msg/{path.stem}"
            if name in store.fielddefs:
                continue
            types.update(get_types_from_msg(path.read_text(encoding="utf-8"), name))
    store.register(types)
    return store, {k: str(v) for k, v in found.items()}


# --------------------------------------------------------------------------
# message helpers
# --------------------------------------------------------------------------

def _msgtype(obj):
    return getattr(obj, "__msgtype__", None)


def _is_msg(obj):
    return dataclasses.is_dataclass(obj) and not isinstance(obj, type)


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def header_time(msg):
    header = getattr(msg, "header", None)
    if header is not None and _msgtype(header) == "std_msgs/msg/Header":
        t = stamp_s(header.stamp)
        if t > 0:
            return t, header.frame_id
        return None, header.frame_id
    return None, None


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _scalar(value):
    return isinstance(value, (bool, int, float, np.integer, np.floating, np.bool_))


def message_fields(msg, prefix=""):
    """One message -> ordered ``{name: value}``; nested messages flattened.

    Values are numbers, strings, numpy arrays, lists of strings, or lists of
    messages (kept as lists; the caller turns them into struct columns).
    Time/Duration become seconds; a nested header contributes its stamp and
    frame_id.
    """
    out = {}
    for field in dataclasses.fields(msg):
        if field.name.startswith("__"):  # rosbags' __msgtype__ marker
            continue
        value = getattr(msg, field.name)
        name = f"{prefix}{field.name}"
        kind = _msgtype(value) if _is_msg(value) else None
        if kind in ("builtin_interfaces/msg/Time", "builtin_interfaces/msg/Duration"):
            out[name] = stamp_s(value)
        elif kind == "std_msgs/msg/Header":
            out[name + "_stamp"] = stamp_s(value.stamp)
            out[name + "_frame_id"] = value.frame_id
        elif kind is not None:
            out.update(message_fields(value, name + "_"))
        else:
            out[name] = value
    return out


def _values_to_column(values):
    """One flattened field across n messages -> its MATLAB column."""
    first = next((v for v in values if v is not None), None)
    if first is None or _scalar(first):
        return np.array([math.nan if v is None else float(v) for v in values],
                        dtype=float).reshape(-1, 1)
    if isinstance(first, str):
        return str_cell(values)
    if isinstance(first, np.ndarray):
        lengths = {len(v) for v in values}
        if len(lengths) == 1 and first.dtype.kind in "biuf":
            n = lengths.pop()
            return np.vstack([np.asarray(v, dtype=float).reshape(1, n) for v in values]) \
                if values else np.zeros((0, n))
        return cell([np.asarray(v, dtype=float).reshape(-1, 1) if v.dtype.kind in "biuf"
                     else str_cell(list(v)) for v in values])
    if isinstance(first, list):
        if all(isinstance(x, str) for v in values for x in v):
            return cell([str_cell(v) for v in values])
        return cell([messages_struct(v) for v in values])
    return str_cell([str(v) for v in values])


def messages_struct(msgs):
    """A list of messages (no time) -> one struct of n-row columns.

    This is what a variable-length field of messages (``obstacles``,
    ``detections``, ...) becomes inside one cell entry; an empty list is [].
    """
    if not msgs:
        return np.zeros((0, 0))
    rows = [message_fields(m) for m in msgs]
    out = {}
    for key in rows[0]:
        out[mat_name(key, out)] = _values_to_column([r.get(key) for r in rows])
    return out


def flatten_messages(msgs):
    """The generic flattener: n messages of one type -> struct of columns.

    Scalars -> n x 1 double; strings -> n x 1 cell; fixed-length numeric
    arrays -> n x k; variable-length arrays -> n x 1 cell; sequences of
    messages -> n x 1 cell of structs (see messages_struct). The top-level
    header is dropped here: its stamp is t_ros, its frame_id goes in info.
    """
    if not msgs:
        return {}
    rows = []
    for msg in msgs:
        fields = message_fields(msg)
        fields.pop("header_stamp", None)
        fields.pop("header_frame_id", None)
        rows.append(fields)
    out = {}
    for key in rows[0]:
        out[mat_name(key, out)] = _values_to_column([r.get(key) for r in rows])
    return out


# --------------------------------------------------------------------------
# typed extractors: (msgs) -> (fields, info)
# --------------------------------------------------------------------------

def _col(values):
    return np.asarray(values, dtype=float).reshape(-1, 1)


def extract_odometry(msgs):
    pose = [m.pose.pose for m in msgs]
    tw = [m.twist.twist for m in msgs]
    fields = {
        "x": _col([p.position.x for p in pose]),
        "y": _col([p.position.y for p in pose]),
        "yaw": _col([yaw_of(p.orientation) for p in pose]),
        "vx": _col([t.linear.x for t in tw]),
        "vy": _col([t.linear.y for t in tw]),
        "wz": _col([t.angular.z for t in tw]),
    }
    info = {"child_frame_id": msgs[0].child_frame_id} if msgs else {}
    return fields, info


def extract_pose_cov(msgs):
    pose = [m.pose.pose for m in msgs]
    cov = [np.asarray(m.pose.covariance, dtype=float) for m in msgs]
    return {
        "x": _col([p.position.x for p in pose]),
        "y": _col([p.position.y for p in pose]),
        "yaw": _col([yaw_of(p.orientation) for p in pose]),
        "cov_xx": _col([c[0] for c in cov]),
        "cov_yy": _col([c[7] for c in cov]),
        "cov_yawyaw": _col([c[35] for c in cov]),
    }, {}


def extract_pose(msgs):
    pose = [m.pose for m in msgs]
    return {
        "x": _col([p.position.x for p in pose]),
        "y": _col([p.position.y for p in pose]),
        "yaw": _col([yaw_of(p.orientation) for p in pose]),
    }, {}


def extract_ackermann(msgs):
    drive = [m.drive for m in msgs]
    return {
        "speed": _col([d.speed for d in drive]),
        "steering_angle": _col([d.steering_angle for d in drive]),
        "steering_angle_velocity": _col([d.steering_angle_velocity for d in drive]),
        "acceleration": _col([d.acceleration for d in drive]),
        "jerk": _col([d.jerk for d in drive]),
    }, {}


def extract_scan(msgs):
    if not msgs:
        return {"ranges": np.zeros((0, 0), dtype=np.float32)}, {}
    beams = len(msgs[0].ranges)
    same = [m for m in msgs if len(m.ranges) == beams]
    fields = {"ranges": np.vstack([np.asarray(m.ranges, dtype=np.float32) for m in same])}
    if all(len(m.intensities) == beams for m in same) and beams and len(same[0].intensities):
        fields["intensities"] = np.vstack(
            [np.asarray(m.intensities, dtype=np.float32) for m in same])
    first = msgs[0]
    info = {key: float(getattr(first, key)) for key in (
        "angle_min", "angle_max", "angle_increment", "time_increment",
        "scan_time", "range_min", "range_max")}
    info["n_beams"] = float(beams)
    info["n_dropped_other_beam_count"] = float(len(msgs) - len(same))
    return fields, info, same


EXTRACTORS = {
    "odometry": extract_odometry,
    "pose_cov": extract_pose_cov,
    "pose": extract_pose,
    "ackermann": extract_ackermann,
}


def map_struct(msg, t_ros, timing):
    """The last OccupancyGrid -> ``map`` (int8 grid, height x width, row 1 = y0)."""
    info = msg.info
    grid = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
    return {
        "source": SOURCE,
        "time_source": "header" if header_time(msg)[0] else "receive",
        "topic": "/slam/map",
        "t_ros": float(t_ros),
        "t_rel": float(t_ros - timing["drive_start_ros"]),
        "grid": grid,
        "resolution": float(info.resolution),
        "width": float(info.width),
        "height": float(info.height),
        "origin_x": float(info.origin.position.x),
        "origin_y": float(info.origin.position.y),
        "origin_yaw": float(yaw_of(info.origin.orientation)),
        "frame_id": msg.header.frame_id,
    }


# --------------------------------------------------------------------------
# a whole bag
# --------------------------------------------------------------------------

def bag_window(bag_dir):
    """``(start_s, end_s, message_count)`` from metadata.yaml, receive time."""
    import yaml

    with open(Path(bag_dir) / "metadata.yaml", encoding="utf-8") as fh:
        info = (yaml.safe_load(fh) or {}).get("rosbag2_bagfile_information")
    if not info:
        raise ValueError(f"{bag_dir}/metadata.yaml is empty")
    count = int(info.get("message_count") or 0)
    if count == 0:
        return None, None, 0
    start = info["starting_time"]["nanoseconds_since_epoch"] / 1e9
    return start, start + info["duration"]["nanoseconds"] / 1e9, count


def load_bag_topics(bag_dir, timing, typestore, scan_decimate=1):
    """``(topics, topic_table, errors)`` for one bag.

    ``topic_table`` lists every connection in the bag (name, type, count,
    exported struct or ''), ``errors`` one string per topic that failed.
    """
    from rosbags.rosbag2 import Reader

    topics, table, errors = {}, [], []
    with Reader(Path(bag_dir)) as reader:
        conns = {}
        for conn in reader.connections:
            if conn.topic in EXCLUDED:
                struct = ""
            elif conn.topic in TOPICS:
                struct = TOPICS[conn.topic][0]
            else:
                struct = mat_name(conn.topic.strip("/"))
            table.append({"name": conn.topic, "type": conn.msgtype,
                          "count": conn.msgcount, "struct": struct})
            if struct and conn.msgcount > 0:
                conns.setdefault(conn.topic, []).append(conn)
        for topic, topic_conns in conns.items():
            name, kind = TOPICS.get(topic, (mat_name(topic.strip("/")), "generic"))
            try:
                msgs, stamps = [], []
                for conn, timestamp, raw in reader.messages(connections=topic_conns):
                    if kind == "map":  # keep only the last grid
                        msgs, stamps = [raw], [(timestamp, conn.msgtype)]
                        continue
                    msgs.append(typestore.deserialize_cdr(raw, conn.msgtype))
                    stamps.append(timestamp / 1e9)
                if kind == "map":
                    timestamp, msgtype = stamps[0]
                    msg = typestore.deserialize_cdr(msgs[0], msgtype)
                    t_hdr = header_time(msg)[0]
                    topics[name] = map_struct(msg, t_hdr or timestamp / 1e9, timing)
                    continue
                if kind == "scan" and scan_decimate > 1:
                    msgs, stamps = msgs[::scan_decimate], stamps[::scan_decimate]
                topics[name] = bag_topic(name, topic, kind, msgs, stamps, timing,
                                         topic_conns[0].msgtype)
            except Exception as exc:  # noqa: BLE001 - one topic must not sink the run
                errors.append(f"{topic}: {type(exc).__name__}: {exc}")
    return topics, table, errors


def bag_topic(name, topic, kind, msgs, stamps, timing, msgtype):
    """n deserialised messages -> the common topic layout (see campaign.make_topic)."""
    info = {"msg_type": msgtype}
    if kind == "scan":
        fields, extra, msgs_kept = extract_scan(msgs)
        keep = {id(m) for m in msgs_kept}
        stamps = [s for m, s in zip(msgs, stamps) if id(m) in keep]
        msgs = msgs_kept
        info.update(extra)
    elif kind in EXTRACTORS:
        fields, extra = EXTRACTORS[kind](msgs)
        info.update(extra)
    else:
        fields = flatten_messages(msgs)
    t, used = [], set()
    frame = None
    for msg, received in zip(msgs, stamps):
        stamp, frame_id = header_time(msg)
        frame = frame if frame is not None else frame_id
        used.add("header" if stamp is not None else "receive")
        t.append(stamp if stamp is not None else received)
    if frame is not None:
        info["frame_id"] = frame
    t = np.asarray(t, dtype=float).reshape(-1, 1) if t else empty_column()
    out = {
        "source": SOURCE,
        "time_source": "+".join(sorted(used)) if used else "receive",
        "topic": topic,
        "t_ros": t,
        "t_rel": t - timing["drive_start_ros"],
    }
    for key, value in fields.items():
        key = f"{name}_{key}" if key in out or key == "info" else key
        out[mat_name(key, out)] = value
    out["info"] = info
    return out
