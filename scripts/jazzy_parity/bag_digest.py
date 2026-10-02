#!/usr/bin/env python3
"""Content digest of a bag: sha256 over every (topic, type, timestamp,
serialized payload), in recorded order, plus per-topic counts.

Used to prove that the replay input bag built on the Orin (Humble rosbag2)
carries exactly the same messages as the one built on Thor (Jazzy rosbag2),
independent of how each distro lays out its sqlite file and metadata.yaml
(those differ by design, so a file checksum cannot answer this).

Usage: bag_digest.py BAG_DIR
"""
import hashlib
import sys

from bag_compat import open_reader


def main():
    reader = open_reader(sys.argv[1])
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    h = hashlib.sha256()
    counts = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        h.update(topic.encode())
        h.update(types[topic].encode())
        h.update(int(t).to_bytes(8, 'little', signed=True))
        h.update(bytes(data))
        counts[topic] = counts.get(topic, 0) + 1
    for topic in sorted(counts):
        print('%7d  %s  %s' % (counts[topic], topic, types[topic]))
    print('content_sha256 %s' % h.hexdigest())


if __name__ == '__main__':
    main()
