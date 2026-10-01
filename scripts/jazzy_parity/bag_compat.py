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


def make_topic_metadata(topic_id, name, type_name, serialization_format='cdr'):
    try:
        return rosbag2_py.TopicMetadata(
            id=topic_id, name=name, type=type_name,
            serialization_format=serialization_format)
    except TypeError:
        return rosbag2_py.TopicMetadata(
            name=name, type=type_name, serialization_format=serialization_format)


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
