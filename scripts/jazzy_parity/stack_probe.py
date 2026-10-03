#!/usr/bin/env python3
"""Phase 5: look at a running full stack from outside, as one rclpy participant.

Run it with ROS_DISCOVERY_SERVER set and ROS_SUPER_CLIENT=TRUE: a plain
Discovery Server client only learns about endpoints matching its own, so
graph queries (node names, publishers of a topic) need a super client. Every
subcommand writes JSON to --out.

  nodes  --duration S [--t0 EPOCH] [--stable S2]
         Poll the node list every 0.25 s; first/last time each node was
         seen (seconds after --t0, default: now). Stops early once the set
         has not changed for --stable seconds (if given).
  rates  --duration S TOPIC...
         Message count and rate per topic. BEST_EFFORT/VOLATILE subscription
         (matches reliable and best-effort publishers alike), type looked up
         from the graph.
  tf     --duration S
         Every TF edge on /tf and /tf_static: messages, and how many distinct
         writers sent it. A DDS writer numbers its samples 1, 2, 3...
         (publication_sequence_number), so one edge reaching us in two
         interleaved increasing sequences has two publishers; the count is
         the minimum number of increasing chains. /tf_static is read
         transient-local (one latched sample per writer). Also lists the
         publisher nodes of both topics.
  graph  [TOPIC...]
         Node names, topic names/types, and the publisher/subscriber nodes
         (with QoS) of each given topic.
  params [--skip-prefix P]
         Every node's parameters (name -> value), via its own
         list_parameters/get_parameters services.
  mission --path FILE --wait S
         /mission/load_mission + /mission/start_mission, then record every
         /mission/status state until COMPLETE/ABORTED or --wait runs out.
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from rosidl_runtime_py.utilities import get_message


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def cmd_nodes(node, a):
    t0 = a.t0 if a.t0 else time.time()
    seen = {}
    last_change = time.monotonic()
    end = time.monotonic() + a.duration
    prev = set()
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time() - t0
        cur = {('' if ns == '/' else ns) + '/' + n
               for n, ns in node.get_node_names_and_namespaces()}
        cur.discard('/' + node.get_name())
        for n in cur:
            seen.setdefault(n, {'first': now})['last'] = now
        if cur != prev:
            last_change = time.monotonic()
            prev = cur
        if a.stable and seen and time.monotonic() - last_change > a.stable:
            break
        time.sleep(0.2)
    return {'t0': t0, 'final': sorted(prev), 'seen': seen}


def _types(node):
    return dict((n, t[0]) for n, t in node.get_topic_names_and_types())


def cmd_rates(node, a):
    # A new super client needs several seconds to be told the whole graph
    # (2 s was not enough in the first Step 4 run): wait for every topic, 30 s max.
    end = time.monotonic() + 30.0
    while True:
        spin_for(node, 1.0)
        types = _types(node)
        if all(t in types for t in a.topics) or time.monotonic() > end:
            break
    qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
    counts, subs, missing = {}, [], []
    for t in a.topics:
        if t not in types:
            missing.append(t)
            continue
        counts[t] = 0

        def cb(_msg, t=t):
            counts[t] += 1
        subs.append(node.create_subscription(get_message(types[t]), t, cb, qos, raw=True))
    spin_for(node, a.duration)
    return {'duration_s': a.duration,
            'rates_hz': {t: round(c / a.duration, 2) for t, c in counts.items()},
            'counts': counts, 'not_in_graph': missing}


def _chains(seqs):
    """Minimum number of strictly increasing subsequences covering seqs."""
    tails = []
    for s in seqs:
        best = None
        for i, last in enumerate(tails):
            if last < s and (best is None or last > tails[best]):
                best = i
        if best is None:
            tails.append(s)
        else:
            tails[best] = s
    return len(tails)


def cmd_tf(node, a):
    from tf2_msgs.msg import TFMessage
    edges = {}

    def make_cb(topic):
        def cb(msg, info):
            sig = tuple(sorted(tr.child_frame_id for tr in msg.transforms))
            for tr in msg.transforms:
                e = edges.setdefault((topic, tr.header.frame_id, tr.child_frame_id),
                                     {'n': 0, 'seqs': [], 'signatures': set()})
                e['n'] += 1
                e['seqs'].append(info['publication_sequence_number'])
                e['signatures'].add(sig)
        return cb

    node.create_subscription(TFMessage, '/tf', make_cb('/tf'), QoSProfile(
        depth=200, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE))
    node.create_subscription(TFMessage, '/tf_static', make_cb('/tf_static'), QoSProfile(
        depth=100, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    spin_for(node, a.duration)
    pubs = {t: sorted(f'{p.node_namespace.rstrip("/")}/{p.node_name}'
                      for p in node.get_publishers_info_by_topic(t))
            for t in ('/tf', '/tf_static')}
    out = []
    for (topic, parent, child), e in sorted(edges.items()):
        out.append({'topic': topic, 'parent': parent, 'child': child,
                    'messages': e['n'], 'writers': _chains(e['seqs']),
                    'signatures': sorted(list(s) for s in e['signatures'])})
    return {'duration_s': a.duration, 'edges': out, 'publisher_nodes': pubs}


def cmd_graph(node, a):
    spin_for(node, 10.0)
    res = {'nodes': sorted(('' if ns == '/' else ns) + '/' + n
                           for n, ns in node.get_node_names_and_namespaces()),
           'topics': {n: t for n, t in node.get_topic_names_and_types()}, 'endpoints': {}}
    for t in a.topics:
        def ep(infos):
            return sorted(
                {'node': f'{i.node_namespace.rstrip("/")}/{i.node_name}',
                 'reliability': i.qos_profile.reliability.name,
                 'durability': i.qos_profile.durability.name}.items()
                for i in infos)
        res['endpoints'][t] = {
            'publishers': [dict(x) for x in ep(node.get_publishers_info_by_topic(t))],
            'subscribers': [dict(x) for x in ep(node.get_subscriptions_info_by_topic(t))]}
    return res


def cmd_params(node, a):
    from rcl_interfaces.srv import GetParameters, ListParameters
    from rclpy.parameter import parameter_value_to_python
    spin_for(node, 10.0)
    out, failed = {}, []
    for n, ns in sorted(node.get_node_names_and_namespaces()):
        full = ('' if ns == '/' else ns) + '/' + n
        if n == node.get_name() or any(full.startswith(p) for p in a.skip_prefix):
            continue
        lc = node.create_client(ListParameters, f'{full}/list_parameters')
        gc = node.create_client(GetParameters, f'{full}/get_parameters')
        try:
            if not lc.wait_for_service(timeout_sec=5.0):
                failed.append(full)
                continue
            fut = lc.call_async(ListParameters.Request(depth=0))
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            names = sorted(fut.result().result.names) if fut.result() else None
            if names is None or not gc.wait_for_service(timeout_sec=5.0):
                failed.append(full)
                continue
            fut = gc.call_async(GetParameters.Request(names=names))
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            if fut.result() is None:
                failed.append(full)
                continue
            out[full] = {k: parameter_value_to_python(v)
                         for k, v in zip(names, fut.result().values)}
        finally:
            node.destroy_client(lc)
            node.destroy_client(gc)
    return {'params': out, 'failed': failed}


def cmd_mission(node, a):
    from f1tenth_messages.msg import MissionStatus
    from f1tenth_messages.srv import LoadMission
    from std_srvs.srv import Trigger
    states = []
    t0 = time.time()
    node.create_subscription(
        MissionStatus, '/mission/status',
        lambda m: states.append({'t': round(time.time() - t0, 3), 'state': m.state}),
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    res = {}
    for name, typ, req in (('load', LoadMission, LoadMission.Request(path=a.path)),
                           ('start', Trigger, Trigger.Request())):
        cli = node.create_client(typ, f'/mission/{name}_mission')
        if not cli.wait_for_service(timeout_sec=20.0):
            res[name] = None
            break
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
        r = fut.result()
        res[name] = None if r is None else {'success': r.success, 'message': r.message,
                                            't': round(time.time() - t0, 3)}
        if r is None or not r.success:
            break
    end = time.monotonic() + a.wait
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if states and states[-1]['state'] in ('COMPLETE', 'ABORTED'):
            break
    res['states'] = states
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('nodes')
    p.add_argument('--duration', type=float, required=True)
    p.add_argument('--t0', type=float, default=0.0)
    p.add_argument('--stable', type=float, default=0.0)
    p = sub.add_parser('rates')
    p.add_argument('--duration', type=float, required=True)
    p.add_argument('topics', nargs='+')
    p = sub.add_parser('tf')
    p.add_argument('--duration', type=float, required=True)
    p = sub.add_parser('graph')
    p.add_argument('topics', nargs='*')
    p = sub.add_parser('params')
    p.add_argument('--skip-prefix', nargs='*', default=[])
    p = sub.add_parser('mission')
    p.add_argument('--path', required=True)
    p.add_argument('--wait', type=float, default=60.0)
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node(f'phase5_probe_{a.cmd}')
    res = {'nodes': cmd_nodes, 'rates': cmd_rates, 'tf': cmd_tf,
           'graph': cmd_graph, 'params': cmd_params, 'mission': cmd_mission}[a.cmd](node, a)
    with open(a.out, 'w') as f:
        json.dump(res, f, indent=2, default=str)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
