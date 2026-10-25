"""
Shared bag-reading helper: a topic's messages, deduplicated.

Runs on: the Orin host (native Humble), the harness container, linus.
Imported by compare_runs.py and mpc_metrics.py.

DUPLICATE DELIVERY: every topic recorded by the old harness's `ros2 bag
record` showed each message delivered exactly twice -- same header.stamp,
same bag-write time, byte-identical payload (Phase 1, Step 3): a transport
artifact of the replay's localhost discovery (most likely SHM + UDP), not
the nodes under test. Consecutive byte-identical messages on the same topic
are collapsed before any analysis; that cannot change a compared value.
"""
from bag_compat import storage_of
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def read_topic(bag_path, topic_name, storage_id=None):
    """Yield (recv_time_ns, msg) for `topic_name` in `bag_path`, deduped."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path),
                                  storage_id=storage_id or storage_of(bag_path)),
        rosbag2_py.ConverterOptions('', ''))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic_name not in type_map:
        return
    msg_type = get_message(type_map[topic_name])
    prev_raw = None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != topic_name:
            continue
        raw = bytes(data)
        if raw == prev_raw:
            continue  # duplicate delivery, see module docstring
        prev_raw = raw
        yield t, deserialize_message(data, msg_type)


def header_stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
