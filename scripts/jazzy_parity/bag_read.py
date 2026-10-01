"""Shared bag-reading helper: read a topic's messages, deduplicated.

DUPLICATE DELIVERY, OBSERVED AND HANDLED: every topic recorded by this
harness's `ros2 bag record` showed each message delivered exactly twice --
same header.stamp, same bag-write time, byte-identical serialized payload
(confirmed: see the Phase 1 report's Step 3 section). This is a
transport-layer artifact of the replay's FastDDS LOCALHOST-discovery setup
(most likely dual SHM+UDP delivery), not anything about the nodes under
test -- the two copies are indistinguishable byte-for-byte. Collapsing
consecutive byte-identical messages on the same topic before any analysis
removes it cleanly without risking a change to the actual values compared.
"""
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_topic(bag_path, topic_name, storage_id=None):
    """Yield (recv_time_ns, msg) for `topic_name` in `bag_path`, deduped."""
    if storage_id is None:
        # mcap (this harness's recorder default) or sqlite3 (original bags)
        import os
        storage_id = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(bag_path)) else 'sqlite3'

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
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
