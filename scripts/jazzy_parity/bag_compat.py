"""
Distro-agnostic rosbag2_py helpers for the Jazzy / Humble parity harness.

Runs on: the Orin host (native Humble), the harness container (Jazzy), linus.
Imported by the bag tools here (restamp_play.py, filter_bag_for_layer.py,
bag_digest.py, logger_check.sh's topic lister); compares nothing itself.

Jazzy's rosbag2_py.TopicMetadata constructor requires `id: int` as the first
argument, Humble's does not accept it at all. Reading is unaffected; this
only matters for WRITING. Try the Jazzy (id=) form first, fall back to the
pre-id form on TypeError, so the same script runs unmodified on both.

Storage: a bag directory is opened with the plugin its own files name (a
.mcap file -> mcap, else sqlite3). Bags this harness writes are sqlite3: the
car's Humble install has no mcap writer, and its readers in tools/ read .db3.
"""
import os

import rosbag2_py


def storage_of(bag_path):
    """'mcap' if the bag directory holds an .mcap file, else 'sqlite3'."""
    try:
        names = os.listdir(bag_path)
    except OSError:
        return 'sqlite3'
    return 'mcap' if any(n.endswith('.mcap') for n in names) else 'sqlite3'


def make_topic_metadata(topic_id, name, type_name, serialization_format='cdr',
                        offered_qos_profiles=None):
    """
    Return a TopicMetadata for writing, on either distro.

    offered_qos_profiles MUST be carried through for any topic whose
    durability matters: /tf_static is TRANSIENT_LOCAL, and a filtered bag
    written with the default (VOLATILE) profile is played with a QoS tf2's
    static listener rejects -- no static transform delivered, every /scan then
    dropped by slam_toolbox's message filter (Phase 2).
    """
    kwargs = {'name': name, 'type': type_name, 'serialization_format': serialization_format}
    if offered_qos_profiles:
        kwargs['offered_qos_profiles'] = offered_qos_profiles
    try:
        return rosbag2_py.TopicMetadata(id=topic_id, **kwargs)
    except TypeError:
        return rosbag2_py.TopicMetadata(**kwargs)


def open_reader(bag_path, storage_id=None):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path),
                                  storage_id=storage_id or storage_of(bag_path)),
        rosbag2_py.ConverterOptions('', ''))
    return reader


def open_writer(bag_path, storage_id='sqlite3'):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return writer


def topics_of(bag_path):
    """{topic: type} of a bag."""
    reader = open_reader(bag_path)
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


if __name__ == '__main__':
    import sys
    if len(sys.argv) != 2 or sys.argv[1] in ('-h', '--help'):
        print('usage: bag_compat.py BAG_DIR   (prints "topic type" per line)\n\n' + __doc__)
        sys.exit(0 if len(sys.argv) == 2 else 2)
    for topic, typ in sorted(topics_of(sys.argv[1]).items()):
        print(topic, typ)
