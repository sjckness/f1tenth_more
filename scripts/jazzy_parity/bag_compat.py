"""Distro-agnostic rosbag2_py helpers for the Jazzy/Humble parity harness.

Jazzy's rosbag2_py.TopicMetadata constructor requires `id: int` as the
first positional/keyword arg (confirmed on this machine: Jazzy
2026-09/2.x). Humble's does not accept it at all. Reading bags (what
compare_runs.py does) is unaffected -- this only matters for WRITING,
which filter_bag_for_layer.py and replay_localization.sh's recorder need.
Try the Jazzy (id=) form first, fall back to the pre-id form on TypeError,
so the same script runs unmodified on both distros.
"""
import rosbag2_py


def make_topic_metadata(topic_id, name, type_name, serialization_format='cdr',
                         offered_qos_profiles=None):
    """offered_qos_profiles MUST be carried through for any topic whose
    durability matters -- /tf_static is TRANSIENT_LOCAL; writing it with
    the default (VOLATILE) profile instead produces a filtered bag that
    `ros2 bag play` publishes with incompatible QoS against tf2's static
    listener. Confirmed live: silent "New publisher discovered... offering
    incompatible QoS... DURABILITY_QOS_POLICY" warning, zero static
    transforms actually delivered, every subsequent /scan dropped by
    slam_toolbox's message filter with "timestamp... earlier than all the
    data in the transform cache" (no base_link->laser edge ever arrived to
    satisfy the filter). See the Phase 2 report's Step 2 section."""
    kwargs = dict(name=name, type=type_name, serialization_format=serialization_format)
    if offered_qos_profiles:
        kwargs['offered_qos_profiles'] = offered_qos_profiles
    try:
        return rosbag2_py.TopicMetadata(id=topic_id, **kwargs)
    except TypeError:
        return rosbag2_py.TopicMetadata(**kwargs)


def open_reader(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    return reader


def open_writer(bag_path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return writer
