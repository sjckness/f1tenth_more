#!/usr/bin/env python3
"""Write a per-layer input bag: pass selected topics through verbatim, and
(optionally) drop specific /tf parent/child frame pairs that the layer under
test publishes itself -- so replaying this bag into that layer's node(s)
never re-injects the layer's own output back as an input.

`ros2 bag play --topics` alone is a whole-topic allowlist; it can't drop
individual TransformStamped entries out of a single multiplexed /tf topic,
which is why this exists (see scripts/jazzy_parity/replay_localization.sh's
own header comment, and the Phase 1 report's Step 3 section, for why
ekf_global needs odom->base_link from /tf -- for TF composition on its
map->odom output, not for its own fusion math -- while must never see its
own map->odom back).

Usage:
  filter_bag_for_layer.py SRC_BAG DST_BAG \
      --topics /odometry/filtered /slam/pose_calibrated \
      --tf-drop map odom [--tf-drop other_parent other_child ...]
"""
import argparse
import shutil
from pathlib import Path

from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

from bag_compat import make_topic_metadata, open_reader, open_writer


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src_bag')
    ap.add_argument('dst_bag')
    ap.add_argument('--topics', nargs='+', required=True,
                     help='Topics to pass through verbatim (and /tf, if '
                          '--tf-drop is also given, filtered as described).')
    ap.add_argument('--tf-drop', nargs=2, action='append', default=[],
                     metavar=('PARENT_FRAME', 'CHILD_FRAME'),
                     help='Drop TransformStamped entries on /tf with this '
                          'exact (frame_id, child_frame_id) pair. Repeatable.')
    args = ap.parse_args()

    drop_pairs = {tuple(p) for p in args.tf_drop}
    dst = Path(args.dst_bag)
    if dst.exists():
        shutil.rmtree(dst)

    reader = open_reader(args.src_bag)
    all_topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for t in args.topics:
        if t not in all_topics:
            raise SystemExit(f'topic {t} not found in {args.src_bag}')

    writer = open_writer(args.dst_bag)
    for topic_id, name in enumerate(args.topics):
        writer.create_topic(make_topic_metadata(topic_id, name, all_topics[name]))

    tf_type = get_message('tf2_msgs/msg/TFMessage') if '/tf' in args.topics and drop_pairs else None

    n_in = {t: 0 for t in args.topics}
    n_out = {t: 0 for t in args.topics}
    n_tf_entries_dropped = 0

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in n_in:
            continue
        n_in[topic] += 1

        if topic == '/tf' and drop_pairs:
            msg = deserialize_message(data, tf_type)
            kept = [tr for tr in msg.transforms
                    if (tr.header.frame_id, tr.child_frame_id) not in drop_pairs]
            n_tf_entries_dropped += len(msg.transforms) - len(kept)
            if not kept:
                continue
            msg.transforms = kept
            data = serialize_message(msg)

        writer.write(topic, data, t)
        n_out[topic] += 1

    print(f'wrote {args.dst_bag}')
    for t in args.topics:
        print(f'  {t}: {n_in[t]} in -> {n_out[t]} out')
    if drop_pairs:
        print(f'  /tf: dropped {n_tf_entries_dropped} individual transform '
              f'entries matching {sorted(drop_pairs)}')


if __name__ == '__main__':
    main()
