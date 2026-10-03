#!/usr/bin/env python3
"""Write SRC_BAG repeated N times back to back, with every header.stamp (and
the bag's own record time) shifted forward by one bag length per repetition.

Phase 5 feeds the FULL stack on wall time (production config, no
use_sim_time), and the source bag is only 42.5 s long. `ros2 bag play --loop`
would send the EKFs a 42.5 s backwards jump in measurement time on every
loop, which robot_localization discards as out-of-sequence data, so the
stack would go quiet after the first pass. Shifting the stamps keeps every
consumer's input monotonic across repetitions. /tf_static is written once
(first repetition only): it is latched, and its stamps are not time-indexed.

Only topics whose type has a top-level std_msgs/Header `header` are
re-stamped; any other topic is refused rather than passed through with
stale stamps.

Usage: loop_bag.py SRC_BAG DST_BAG REPETITIONS
"""
import shutil
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

from bag_compat import make_topic_metadata, open_reader, open_writer


def main():
    src, dst, reps = sys.argv[1], Path(sys.argv[2]), int(sys.argv[3])
    if dst.exists():
        shutil.rmtree(dst)

    reader = open_reader(src)
    topics = reader.get_all_topics_and_types()
    types = {t.name: get_message(t.type) for t in topics}
    for name, typ in types.items():
        if name != '/tf_static' and not hasattr(typ(), 'header'):
            raise SystemExit(f'{name} ({typ.__name__}) has no header to re-stamp')

    msgs = []
    while reader.has_next():
        msgs.append(reader.read_next())
    del reader
    t_first, t_last = msgs[0][2], msgs[-1][2]
    period = t_last - t_first + 50_000_000  # 50 ms gap between repetitions

    writer = open_writer(dst)
    for i, t in enumerate(topics):
        writer.create_topic(make_topic_metadata(
            i, t.name, t.type, offered_qos_profiles=t.offered_qos_profiles))

    count = 0
    for k in range(reps):
        shift = k * period
        for topic, data, t in msgs:
            if topic == '/tf_static':
                if k == 0:
                    writer.write(topic, data, t)
                    count += 1
                continue
            if shift:
                msg = deserialize_message(data, types[topic])
                ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec + shift
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
                data = serialize_message(msg)
            writer.write(topic, data, t + shift)
            count += 1
    print(f'wrote {dst}: {reps} x {period / 1e9:.3f} s, {count} messages')


if __name__ == '__main__':
    main()
