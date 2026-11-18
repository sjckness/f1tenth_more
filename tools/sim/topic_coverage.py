#!/usr/bin/env python3
"""Topic coverage of the running graph: is every topic the stack subscribes to published?

Run as a Discovery Server super client (it must see the whole graph), e.g.
    ROS_SUPER_CLIENT=TRUE python3 tools/sim/topic_coverage.py --sim-node isaac_f1tenth_sim \
        --json out.json --md out.md

Built only from the live graph (rclpy graph API, no list kept in this file): for every topic
it records publishers, subscribers and types, then classifies:
  MISSING     subscribed by a stack node, published by nobody
  SIM         published by the simulator node (what Isaac replaces on the car)
  STACK       published by stack nodes only
  UNUSED_SIM  published by the simulator, subscribed by nobody
Exit code 1 if a MISSING topic is not in --allow-missing.
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node

# Graph-introspection nodes this tool and the ros2 CLI create; never part of the stack.
_IGNORE_NODES = ('topic_coverage', '_ros2cli_')


def _names(infos):
    return sorted({(i.node_namespace.rstrip('/') + '/' + i.node_name) for i in infos
                   if not any(x in i.node_name for x in _IGNORE_NODES)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sim-node', default='isaac_f1tenth_sim')
    ap.add_argument('--spin', type=float, default=6.0, help='seconds of discovery before reading')
    ap.add_argument('--allow-missing', nargs='*', default=[],
                    help='topics allowed to have subscribers but no publisher')
    ap.add_argument('--json')
    ap.add_argument('--md')
    a = ap.parse_args()

    rclpy.init()
    node = Node('topic_coverage')
    t_end = time.time() + a.spin
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    sim = '/' + a.sim_node.lstrip('/')
    rows = []
    for topic, types in sorted(node.get_topic_names_and_types()):
        pubs = _names(node.get_publishers_info_by_topic(topic))
        subs = _names(node.get_subscriptions_info_by_topic(topic))
        if not pubs and not subs:
            continue
        if subs and not pubs:
            cls = 'MISSING'
        elif sim in pubs:
            cls = 'UNUSED_SIM' if not [s for s in subs if s != sim] else 'SIM'
        else:
            cls = 'STACK'
        rows.append({'topic': topic, 'types': types, 'class': cls,
                     'publishers': pubs, 'subscribers': subs})
    node.destroy_node()
    rclpy.shutdown()

    missing = [r['topic'] for r in rows if r['class'] == 'MISSING']
    bad = [t for t in missing if t not in a.allow_missing]
    summary = {k: sum(1 for r in rows if r['class'] == k)
               for k in ('SIM', 'STACK', 'MISSING', 'UNUSED_SIM')}
    nodes = sorted({n for r in rows for n in r['publishers'] + r['subscribers']})
    out = {'summary': summary, 'nodes': len(nodes), 'missing': missing,
           'missing_not_allowed': bad, 'rows': rows}
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(out, f, indent=1)
    if a.md:
        with open(a.md, 'w') as f:
            f.write(f'{len(rows)} topics, {len(nodes)} nodes: ' +
                    ', '.join(f'{k} {v}' for k, v in summary.items()) + '\n\n')
            f.write('| class | topic | type | publishers | subscribers |\n|---|---|---|---|---|\n')
            order = {'MISSING': 0, 'SIM': 1, 'UNUSED_SIM': 2, 'STACK': 3}
            for r in sorted(rows, key=lambda r: (order[r['class']], r['topic'])):
                f.write(f"| {r['class']} | `{r['topic']}` | {', '.join(r['types'])} | "
                        f"{', '.join(r['publishers']) or '-'} | "
                        f"{', '.join(r['subscribers']) or '-'} |\n")
    print(json.dumps({'summary': summary, 'nodes': len(nodes), 'missing': missing,
                      'missing_not_allowed': bad}))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
