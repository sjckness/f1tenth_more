#!/usr/bin/env python3
"""
Write a per-layer input bag from a recorded one.

Selected topics pass through verbatim, and (optionally) specific /tf
parent/child pairs that the layer under test publishes itself are dropped --
so replaying this bag into that layer's node(s) never re-injects the layer's
own output as an input.

Runs on: the Orin host (native Humble), the harness container, linus.
Builds the input bags for phase5_bringup.sh (FEED=bag) and
replay_localization.sh; bag_digest.py then proves that the bag built on one
side carries the same messages as the one built on the other. The output is
sqlite3 (the car's Humble install has no mcap writer).

`ros2 bag play --topics` alone is a whole-topic allowlist; it cannot drop
individual TransformStamped entries out of one multiplexed /tf topic, which
is why this exists.

Usage:
  filter_bag_for_layer.py SRC_BAG DST_BAG --topics T ... [options]

  --tf-drop P C        drop /tf entries with this (frame_id, child_frame_id);
                       repeatable
  --tf-keep P C        keep ONLY these /tf entries (allowlist); repeatable;
                       exclusive with --tf-drop
  --tf-static-keep P C the same allowlist for /tf_static; kept entries are
                       merged into ONE /tf_static message at the first one's
                       time (a transient-local late joiner gets only one
                       writer's history from `ros2 bag play`). Feed the full
                       stack only the static edges of launch files that do not
                       run in the harness (FEED=bag: hardware, perception), so
                       no edge ends up with two publishers.
  --rename FROM TO     write topic FROM (one of --topics) as TO
  --diagnostics-drop-prefix P
                       drop DiagnosticStatus entries on /diagnostics whose name
                       starts with P
  --inject TOPIC TYPE YAML_FILE  --inject-after ANCHOR_TOPIC SECONDS
                       write ONE extra message (fields from YAML_FILE, e.g.
                       make_mpc_goal.py's output) on a topic the source bag
                       does not carry, at the first ANCHOR_TOPIC message +
                       SECONDS; it reaches the node under test at the same sim
                       time on every replay and on both sides
  --start-offset S / --duration S
                       keep only messages from S s after the bag's first one,
                       for at most --duration s (/tf_static always kept)

DST_BAG must not exist: nothing is ever deleted (pick a new name).
"""
import argparse
import os
from pathlib import Path
import sys

from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_message
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_compat import make_topic_metadata, open_reader, open_writer  # noqa: E402, I100


def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('Usage:'):])
    ap.add_argument('src_bag')
    ap.add_argument('dst_bag')
    ap.add_argument('--topics', nargs='+', required=True)
    ap.add_argument('--tf-drop', nargs=2, action='append', default=[],
                    metavar=('PARENT_FRAME', 'CHILD_FRAME'))
    ap.add_argument('--tf-keep', nargs=2, action='append', default=[],
                    metavar=('PARENT_FRAME', 'CHILD_FRAME'))
    ap.add_argument('--tf-static-keep', nargs=2, action='append', default=[],
                    metavar=('PARENT_FRAME', 'CHILD_FRAME'))
    ap.add_argument('--rename', nargs=2, action='append', default=[],
                    metavar=('FROM_TOPIC', 'TO_TOPIC'))
    ap.add_argument('--diagnostics-drop-prefix', action='append', default=[],
                    metavar='STATUS_NAME_PREFIX')
    ap.add_argument('--inject', nargs=3, metavar=('TOPIC', 'TYPE', 'YAML_FILE'))
    ap.add_argument('--inject-after', nargs=2, metavar=('ANCHOR_TOPIC', 'SECONDS'))
    ap.add_argument('--start-offset', type=float, default=0.0)
    ap.add_argument('--duration', type=float, default=0.0)
    a = ap.parse_args(argv)
    if bool(a.inject) != bool(a.inject_after):
        raise SystemExit('--inject and --inject-after go together')
    if a.tf_drop and a.tf_keep:
        raise SystemExit('--tf-drop and --tf-keep are mutually exclusive')
    for src_topic, _ in a.rename:
        if src_topic not in a.topics:
            raise SystemExit('--rename source %s is not in --topics' % src_topic)
    return a


def main(argv=None):
    args = parse_args(argv)
    renames = dict(args.rename)
    drop_pairs = {tuple(p) for p in args.tf_drop}
    keep_pairs = {tuple(p) for p in args.tf_keep}
    static_keep_pairs = {tuple(p) for p in args.tf_static_keep}
    dst = Path(args.dst_bag)
    if dst.exists():
        raise SystemExit('%s exists: pick a new name (nothing is deleted)' % dst)

    reader = open_reader(args.src_bag)
    all_topics = {t.name: t for t in reader.get_all_topics_and_types()}
    for t in args.topics:
        if t not in all_topics:
            raise SystemExit('topic %s not found in %s' % (t, args.src_bag))

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
            raise SystemExit('--inject topic %s already exists in the source bag' % inj_topic)
        anchor_reader = open_reader(args.src_bag)
        anchor_t = None
        while anchor_reader.has_next():
            topic, _, t = anchor_reader.read_next()
            if topic == anchor_topic:
                anchor_t = t
                break
        del anchor_reader
        if anchor_t is None:
            raise SystemExit('--inject-after anchor topic %s has no messages' % anchor_topic)
        inj_msg = get_message(inj_type)()
        with open(inj_yaml) as f:
            set_message_fields(inj_msg, yaml.safe_load(f))
        inject = (inj_topic, serialize_message(inj_msg), anchor_t + int(round(anchor_sec * 1e9)))
        writer.create_topic(make_topic_metadata(len(args.topics), inj_topic, inj_type))

    diag_prefixes = tuple(args.diagnostics_drop_prefix)
    diag_type = get_message('diagnostic_msgs/msg/DiagnosticArray') if diag_prefixes else None
    n_diag_dropped = 0
    tf_filtering = ('/tf' in args.topics) and bool(drop_pairs or keep_pairs)
    static_filtering = ('/tf_static' in args.topics) and bool(static_keep_pairs)
    tf_type = (get_message('tf2_msgs/msg/TFMessage')
               if tf_filtering or static_filtering else None)
    n_static_entries_dropped = 0
    static_merged, static_merged_t = [], None
    n_in = {t: 0 for t in args.topics}
    n_out = {t: 0 for t in args.topics}
    n_tf_entries_dropped = 0
    t_first = None
    window_lo = window_hi = None

    while reader.has_next():
        topic, data, t = reader.read_next()
        if t_first is None:
            t_first = t
            window_lo = t_first + int(args.start_offset * 1e9)
            window_hi = window_lo + int(args.duration * 1e9) if args.duration else None
        if topic not in n_in:
            continue
        n_in[topic] += 1
        if topic != '/tf_static' and (t < window_lo or (window_hi and t > window_hi)):
            continue

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
        # Written last, at the first kept message's time: readers return
        # messages in timestamp order, not write order. Clamped into the
        # window so a --start-offset bag still starts with it.
        merged = tf_type()
        merged.transforms = static_merged
        writer.write('/tf_static', serialize_message(merged), max(static_merged_t, window_lo or 0))
        n_out['/tf_static'] = 1
    if inject is not None:  # anchor time later than every passed-through message
        writer.write(inject[0], inject[1], inject[2])
    del writer
    if args.inject:
        print('  %s: 1 injected at first %s + %.3f s'
              % (args.inject[0], args.inject_after[0], float(args.inject_after[1])))
    if diag_prefixes:
        print('  /diagnostics: dropped %d status entries named %s*'
              % (n_diag_dropped, list(diag_prefixes)))
    print('wrote %s' % args.dst_bag)
    for t in args.topics:
        shown = '%s (as %s)' % (t, renames[t]) if t in renames else t
        print('  %s: %d in -> %d out' % (shown, n_in[t], n_out[t]))
    if static_filtering:
        print('  /tf_static: dropped %d transform entries; %d kept, merged into one message: %s'
              % (n_static_entries_dropped, len(static_merged),
                 sorted((tr.header.frame_id, tr.child_frame_id) for tr in static_merged)))
    if drop_pairs:
        print('  /tf: dropped %d entries matching %s' % (n_tf_entries_dropped, sorted(drop_pairs)))
    elif keep_pairs:
        print('  /tf: dropped %d entries NOT matching %s'
              % (n_tf_entries_dropped, sorted(keep_pairs)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
