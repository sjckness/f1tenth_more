#!/usr/bin/env python3
"""
Content digest of a bag, independent of its storage layout.

sha256 over every (topic, type, timestamp, message CONTENT), in recorded
order, plus per-topic counts.

Runs on: the Orin host (native Humble), the harness container, linus.
Compares: an input bag built on one side (filter_bag_for_layer.py) with the
one built on the other, independent of how each distro lays out its storage
file and metadata.yaml (a file checksum cannot answer that). When the two
digests match, both sides' replays were fed the same messages.

CONTENT, NOT RAW BYTES: CDR alignment padding of a message the harness
serializes itself (--inject) is uninitialised, so raw payloads of equal
messages can differ (Phase 4, DriveCommand.mode). Each message is
deserialized and hashed as canonical JSON of its fields (floats via repr,
exact), the same on both distros for the same values.

Usage: bag_digest.py BAG_DIR
"""
import hashlib
import json
import os
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_compat import open_reader  # noqa: E402, I100


def canon(value):
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, dict):
        return {k: canon(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [canon(v) for v in value]
    if isinstance(value, (bytes, bytearray)):
        return value.hex()
    return value


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1 or argv[0] in ('-h', '--help'):
        print(__doc__)
        return 0 if argv and argv[0] in ('-h', '--help') else 2
    reader = open_reader(argv[0])
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {name: get_message(t) for name, t in types.items()}
    h = hashlib.sha256()
    counts = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        content = json.dumps(canon(message_to_ordereddict(msg)), sort_keys=True,
                             separators=(',', ':'))
        h.update(topic.encode())
        h.update(types[topic].encode())
        h.update(int(t).to_bytes(8, 'little', signed=True))
        h.update(content.encode())
        counts[topic] = counts.get(topic, 0) + 1
    for topic in sorted(counts):
        print('%7d  %s  %s' % (counts[topic], topic, types[topic]))
    print('content_sha256 %s' % h.hexdigest())
    return 0


if __name__ == '__main__':
    sys.exit(main())
