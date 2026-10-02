#!/usr/bin/env python3
"""Content digest of a bag: sha256 over every (topic, type, timestamp, message
CONTENT), in recorded order, plus per-topic counts.

Used to prove that a replay input bag built on the Orin (Humble rosbag2)
carries exactly the same messages as the one built on Thor (Jazzy rosbag2),
independent of how each distro lays out its storage file and metadata.yaml
(those differ by design, so a file checksum cannot answer this).

CONTENT, NOT RAW BYTES. The first version hashed the serialized CDR payload.
That is not deterministic for a message the harness serializes itself
(filter_bag_for_layer.py --inject, --diagnostics-drop-prefix): CDR alignment
padding is left uninitialised, so two builds of the same bag differed in the
3 padding bytes after DriveCommand.mode ("straight") while every field was
equal (found in Phase 4). Each message is therefore deserialized and hashed as
canonical JSON of its fields (rosidl_runtime_py.message_to_ordereddict, floats
via repr -- exact), which is the same on both distros for the same values.

Usage: bag_digest.py BAG_DIR
"""
import hashlib
import json
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from bag_compat import open_reader


def _canon(value):
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, dict):
        return {k: _canon(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_canon(v) for v in value]
    if isinstance(value, (bytes, bytearray)):
        return value.hex()
    return value


def main():
    reader = open_reader(sys.argv[1])
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {name: get_message(t) for name, t in types.items()}
    h = hashlib.sha256()
    counts = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        content = json.dumps(_canon(message_to_ordereddict(msg)), sort_keys=True,
                             separators=(',', ':'))
        h.update(topic.encode())
        h.update(types[topic].encode())
        h.update(int(t).to_bytes(8, 'little', signed=True))
        h.update(content.encode())
        counts[topic] = counts.get(topic, 0) + 1
    for topic in sorted(counts):
        print('%7d  %s  %s' % (counts[topic], topic, types[topic]))
    print('content_sha256 %s' % h.hexdigest())


if __name__ == '__main__':
    main()
