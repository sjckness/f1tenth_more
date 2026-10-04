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

  filter_bag_for_layer.py SRC_BAG DST_BAG \
      --topics /scan /tf_static /tf \
      --tf-keep odom base_link
      # --tf-keep is an ALLOWLIST (drop everything else on /tf) --
      # cleaner than an exhaustive --tf-drop list when a layer needs only
      # one or two specific edges out of several published on /tf (e.g.
      # slam_toolbox needs only odom->base_link, not the wheel-joint
      # frames robot_state_publisher also puts on the same topic).
      # --tf-drop and --tf-keep are mutually exclusive.

  filter_bag_for_layer.py SRC_BAG DST_BAG \
      --topics /scan /odom /tf_static \
      --tf-static-keep base_link imu
      # --tf-static-keep: the same allowlist for /tf_static (independent of
      # --tf-keep, which only ever touches /tf). Phase 5: feed the full stack
      # only the static edges whose publishers are hardware launch files that
      # do not run on Thor, so no edge ends up with two publishers. The kept
      # entries are merged into ONE /tf_static message (at the first one's
      # time): `ros2 bag play` publishes every message of a topic through a
      # single writer, and a transient-local late joiner gets only that
      # writer's history, so separate latched messages could be lost.

  filter_bag_for_layer.py SRC_BAG DST_BAG --topics ... \
      --inject /mpc/goal_drive f1tenth_messages/msg/DriveCommand goal.yaml \
      --inject-after /odometry/filtered 0.5
      # --inject writes ONE extra message (fields from a YAML file, e.g.
      # make_mpc_goal.py's output) on a topic the source bag does not
      # carry, timestamped --inject-after's SECONDS after the first message
      # on that topic in the source bag. Being part of the input bag, it
      # reaches the node under test at the same sim time on every replay
      # and on both distros. The topic is created with no offered QoS
      # profile, so `ros2 bag play` offers its default (reliable,
      # volatile), which is what the original publisher used.
"""
import argparse
import shutil
from pathlib import Path

import yaml
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_message

from bag_compat import make_topic_metadata, open_reader, open_writer


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src_bag')
    ap.add_argument('dst_bag')
    ap.add_argument('--topics', nargs='+', required=True,
                     help='Topics to pass through verbatim (and /tf, if '
                          '--tf-drop/--tf-keep is also given, filtered as '
                          'described).')
    ap.add_argument('--tf-drop', nargs=2, action='append', default=[],
                     metavar=('PARENT_FRAME', 'CHILD_FRAME'),
                     help='Drop TransformStamped entries on /tf with this '
                          'exact (frame_id, child_frame_id) pair. Repeatable.')
    ap.add_argument('--tf-keep', nargs=2, action='append', default=[],
                     metavar=('PARENT_FRAME', 'CHILD_FRAME'),
                     help='Keep ONLY TransformStamped entries on /tf with '
                          'this exact pair, dropping everything else on the '
                          'topic. Repeatable. Mutually exclusive with '
                          '--tf-drop.')
    ap.add_argument('--tf-static-keep', nargs=2, action='append', default=[],
                     metavar=('PARENT_FRAME', 'CHILD_FRAME'),
                     help='Keep ONLY TransformStamped entries on /tf_static with '
                          'this exact pair. Repeatable. Messages left empty are '
                          'dropped.')
    ap.add_argument('--rename', nargs=2, action='append', default=[],
                     metavar=('FROM_TOPIC', 'TO_TOPIC'),
                     help='Write FROM_TOPIC (one of --topics) under the name '
                          'TO_TOPIC, same type, QoS and payload. Repeatable. For a '
                          'documented stand-in input that both distros must receive '
                          'byte-identically (Phase 4: /costmap/front_clearance as '
                          '/perception/front_distance).')
    ap.add_argument('--diagnostics-drop-prefix', action='append', default=[],
                     metavar='STATUS_NAME_PREFIX',
                     help='Drop DiagnosticStatus entries on /diagnostics whose name '
                          'starts with this prefix (the node under test publishes '
                          'them itself). Repeatable. Messages left empty are dropped.')
    ap.add_argument('--inject', nargs=3, metavar=('TOPIC', 'TYPE', 'YAML_FILE'),
                     help='Write one message of TYPE with the fields in YAML_FILE '
                          'on TOPIC (see --inject-after).')
    ap.add_argument('--inject-after', nargs=2, metavar=('ANCHOR_TOPIC', 'SECONDS'),
                     help='Timestamp for --inject: the first ANCHOR_TOPIC message '
                          'in the source bag, plus SECONDS.')
    args = ap.parse_args()
    if bool(args.inject) != bool(args.inject_after):
        raise SystemExit('--inject and --inject-after go together')

    if args.tf_drop and args.tf_keep:
        raise SystemExit('--tf-drop and --tf-keep are mutually exclusive')
    renames = dict(args.rename)
    for src_topic in renames:
        if src_topic not in args.topics:
            raise SystemExit(f'--rename source {src_topic} is not in --topics')
    drop_pairs = {tuple(p) for p in args.tf_drop}
    keep_pairs = {tuple(p) for p in args.tf_keep}
    static_keep_pairs = {tuple(p) for p in args.tf_static_keep}
    dst = Path(args.dst_bag)
    if dst.exists():
        shutil.rmtree(dst)

    reader = open_reader(args.src_bag)
    all_topics = {t.name: t for t in reader.get_all_topics_and_types()}
    for t in args.topics:
        if t not in all_topics:
            raise SystemExit(f'topic {t} not found in {args.src_bag}')

    writer = open_writer(args.dst_bag)
    for topic_id, name in enumerate(args.topics):
        src_meta = all_topics[name]
        writer.create_topic(make_topic_metadata(
            topic_id, renames.get(name, name), src_meta.type,
            offered_qos_profiles=src_meta.offered_qos_profiles))

    inject = None
    if args.inject:
        inj_topic, inj_type, inj_yaml = args.inject
        anchor_topic, anchor_sec = args.inject_after[0], float(args.inject_after[1])
        if inj_topic in all_topics:
            raise SystemExit(f'--inject topic {inj_topic} already exists in the source bag')
        anchor_reader = open_reader(args.src_bag)
        anchor_t = None
        while anchor_reader.has_next():
            topic, _, t = anchor_reader.read_next()
            if topic == anchor_topic:
                anchor_t = t
                break
        del anchor_reader
        if anchor_t is None:
            raise SystemExit(f'--inject-after anchor topic {anchor_topic} has no messages')
        inj_msg = get_message(inj_type)()
        with open(inj_yaml) as f:
            set_message_fields(inj_msg, yaml.safe_load(f))
        inject = (inj_topic, serialize_message(inj_msg), anchor_t + int(round(anchor_sec * 1e9)))
        writer.create_topic(make_topic_metadata(len(args.topics), inj_topic, inj_type))

    diag_prefixes = tuple(args.diagnostics_drop_prefix)
    diag_type = get_message('diagnostic_msgs/msg/DiagnosticArray') if diag_prefixes else None
    n_diag_dropped = 0

    tf_filtering = ('/tf' in args.topics) and (drop_pairs or keep_pairs)
    static_filtering = ('/tf_static' in args.topics) and bool(static_keep_pairs)
    tf_type = (get_message('tf2_msgs/msg/TFMessage')
               if tf_filtering or static_filtering else None)
    n_static_entries_dropped = 0
    static_merged, static_merged_t = [], None

    n_in = {t: 0 for t in args.topics}
    n_out = {t: 0 for t in args.topics}
    n_tf_entries_dropped = 0

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in n_in:
            continue
        n_in[topic] += 1

        if topic == '/tf' and tf_filtering:
            msg = deserialize_message(data, tf_type)
            if keep_pairs:
                kept = [tr for tr in msg.transforms
                        if (tr.header.frame_id, tr.child_frame_id) in keep_pairs]
            else:
                kept = [tr for tr in msg.transforms
                        if (tr.header.frame_id, tr.child_frame_id) not in drop_pairs]
            n_tf_entries_dropped += len(msg.transforms) - len(kept)
            if not kept:
                continue
            msg.transforms = kept
            data = serialize_message(msg)

        if topic == '/tf_static' and static_filtering:
            msg = deserialize_message(data, tf_type)
            kept = [tr for tr in msg.transforms
                    if (tr.header.frame_id, tr.child_frame_id) in static_keep_pairs]
            n_static_entries_dropped += len(msg.transforms) - len(kept)
            if kept:
                static_merged.extend(kept)
                if static_merged_t is None:
                    static_merged_t = t
            continue

        if inject is not None and t >= inject[2]:
            writer.write(inject[0], inject[1], inject[2])
            inject = None
        if topic == '/diagnostics' and diag_prefixes:
            msg = deserialize_message(data, diag_type)
            kept = [st for st in msg.status if not st.name.startswith(diag_prefixes)]
            n_diag_dropped += len(msg.status) - len(kept)
            if not kept:
                continue
            if len(kept) != len(msg.status):
                msg.status = kept
                data = serialize_message(msg)

        writer.write(renames.get(topic, topic), data, t)
        n_out[topic] += 1

    if static_merged:
        # Written last, at the first kept message's time: sqlite3/mcap readers
        # return messages in timestamp order, not write order.
        merged = tf_type()
        merged.transforms = static_merged
        writer.write('/tf_static', serialize_message(merged), static_merged_t)
        n_out['/tf_static'] = 1
    if inject is not None:  # anchor time later than every passed-through message
        writer.write(inject[0], inject[1], inject[2])
    if args.inject:
        print(f'  {args.inject[0]}: 1 injected at first {args.inject_after[0]} '
              f'+ {float(args.inject_after[1]):.3f} s')
    if diag_prefixes:
        print(f'  /diagnostics: dropped {n_diag_dropped} status entries named {list(diag_prefixes)}*')
    print(f'wrote {args.dst_bag}')
    for t in args.topics:
        shown = f'{t} (as {renames[t]})' if t in renames else t
        print(f'  {shown}: {n_in[t]} in -> {n_out[t]} out')
    if static_filtering:
        print(f'  /tf_static: dropped {n_static_entries_dropped} transform entries; '
              f'{len(static_merged)} kept, merged into one message: '
              f'{sorted((tr.header.frame_id, tr.child_frame_id) for tr in static_merged)}')
    if drop_pairs:
        print(f'  /tf: dropped {n_tf_entries_dropped} individual transform '
              f'entries matching {sorted(drop_pairs)}')
    elif keep_pairs:
        print(f'  /tf: dropped {n_tf_entries_dropped} individual transform '
              f'entries NOT matching {sorted(keep_pairs)}')


if __name__ == '__main__':
    main()
