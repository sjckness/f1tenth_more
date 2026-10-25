#!/usr/bin/env python3
"""
Look at a running full stack from outside, as one rclpy participant.

Runs on: the Orin host with native Humble sourced, inside the harness
container (Jazzy), linus. Same code on both distros; it compares nothing by
itself -- each subcommand writes JSON that phase5_summary.py and
parity_gates.py compare between the native and the container run.

Run it with ROS_DISCOVERY_SERVER set and ROS_SUPER_CLIENT=TRUE: a plain
Discovery Server client only learns about endpoints matching its own, so
graph queries (node names, publishers of a topic) need a super client.
phase5_bringup.sh calls it that way. Every subcommand writes JSON to --out.

  nodes  --duration S [--t0 EPOCH] [--stable S2]
         Poll the node list every 0.25 s; first/last time each node was
         seen (seconds after --t0, default: now). Stops early once the set
         has not changed for --stable seconds (if given).
  rates  --duration S TOPIC...
         Message count and rate per topic. BEST_EFFORT/VOLATILE subscription
         (matches reliable and best-effort publishers alike), type looked up
         from the graph (waits up to --wait-graph s for every topic).
  stamps --duration S --topic T
         Receive wall time (time.time()) of every message on T, plus its
         header stamp when it has one: the wall-clock period of a topic whose
         publisher runs on sim time (mpc_corr in a replay), for mpc_timing.py.
  jsonstat --duration S --topic T --key K
         A std_msgs/String topic carrying JSON (lidar_front_wall_node's
         /perception/virtual_wall/debug): statistics of the numeric field K
         (e.g. tracker_ms, the R6 gate).
  tf     --duration S
         Every TF edge on /tf and /tf_static: messages, and how many distinct
         writers sent it (a DDS writer numbers its samples 1, 2, 3..., so one
         edge reaching us in two interleaved increasing sequences has two
         publishers; the count is the minimum number of increasing chains).
         /tf_static is read transient-local. Also the publisher nodes.
  graph  [TOPIC...]
         Node names, topic names/types, and the publisher/subscriber nodes
         (with QoS) of each given topic.
  params [--skip-prefix P ...]
         Every node's parameters (name -> value), via its own
         list_parameters/get_parameters services.
  io     --wait-node NODE [--timeout S] [--settle S]
         Wait until NODE is in the graph and the endpoints have not changed
         for --settle s (3), then list every node's subscribed
         and published topics with types (replay_localization.sh derives
         what to play and what to record from it, on each side's own code).
  mission --path FILE --wait S
         /mission/load_mission + /mission/start_mission, then record every
         /mission/status state until COMPLETE/ABORTED or --wait runs out.
"""
import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from rosidl_runtime_py.utilities import get_message

BEST_EFFORT = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def full_name(name, ns):
    return ('' if ns == '/' else ns) + '/' + name


def percentiles(values):
    """n, mean, p50, p95, p99, max of a list (None entries dropped)."""
    v = sorted(x for x in values if x is not None and not math.isnan(x))
    if not v:
        return {'n': 0}

    def pct(q):
        if len(v) == 1:
            return v[0]
        k = (len(v) - 1) * q / 100.0
        lo, hi = int(math.floor(k)), int(math.ceil(k))
        return v[lo] + (v[hi] - v[lo]) * (k - lo)
    return {'n': len(v), 'mean': sum(v) / len(v), 'p50': pct(50), 'p95': pct(95),
            'p99': pct(99), 'max': v[-1]}


def _types(node):
    return {n: t[0] for n, t in node.get_topic_names_and_types()}


def wait_for_topics(node, topics, wait_s):
    """Wait for the topics: a new super client needs seconds to learn the graph."""
    end = time.monotonic() + wait_s
    while True:
        spin_for(node, 1.0)
        types = _types(node)
        if all(t in types for t in topics) or time.monotonic() > end:
            return types


def cmd_nodes(node, a):
    t0 = a.t0 if a.t0 else time.time()
    seen = {}
    last_change = time.monotonic()
    end = time.monotonic() + a.duration
    prev = set()
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time() - t0
        cur = {full_name(n, ns) for n, ns in node.get_node_names_and_namespaces()}
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


def cmd_rates(node, a):
    types = wait_for_topics(node, a.topics, a.wait_graph)
    counts, subs, missing = {}, [], []
    for t in a.topics:
        if t not in types:
            missing.append(t)
            continue
        counts[t] = 0

        def cb(_msg, t=t):
            counts[t] += 1
        subs.append(node.create_subscription(get_message(types[t]), t, cb, BEST_EFFORT,
                                             raw=True))
    spin_for(node, a.duration)
    return {'duration_s': a.duration,
            'rates_hz': {t: round(c / a.duration, 2) for t, c in counts.items()},
            'counts': counts, 'not_in_graph': missing}


def cmd_stamps(node, a):
    types = wait_for_topics(node, [a.topic], a.wait_graph)
    if a.topic not in types:
        return {'topic': a.topic, 'not_in_graph': True, 'rx': []}
    rx = []

    def cb(msg):
        h = getattr(msg, 'header', None)
        stamp = h.stamp.sec + h.stamp.nanosec * 1e-9 if h is not None else None
        rx.append([time.time(), stamp])
    node.create_subscription(get_message(types[a.topic]), a.topic, cb, BEST_EFFORT)
    spin_for(node, a.duration)
    periods = [(b[0] - c[0]) * 1e3 for c, b in zip(rx, rx[1:])]
    return {'topic': a.topic, 'duration_s': a.duration, 'n': len(rx),
            'period_ms': percentiles(periods), 'rx': rx}


def cmd_jsonstat(node, a):
    from std_msgs.msg import String
    values, bad = [], [0]

    def cb(msg):
        try:
            v = json.loads(msg.data).get(a.key)
            values.append(float(v) if v is not None else None)
        except (ValueError, AttributeError, TypeError):
            bad[0] += 1
    wait_for_topics(node, [a.topic], a.wait_graph)
    node.create_subscription(String, a.topic, cb, 10)
    spin_for(node, a.duration)
    return {'topic': a.topic, 'key': a.key, 'duration_s': a.duration,
            'messages': len(values) + bad[0], 'unparsable': bad[0],
            'stats': percentiles(values)}


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
    pubs = {t: sorted(full_name(p.node_name, p.node_namespace)
                      for p in node.get_publishers_info_by_topic(t))
            for t in ('/tf', '/tf_static')}
    out = []
    for (topic, parent, child), e in sorted(edges.items()):
        out.append({'topic': topic, 'parent': parent, 'child': child,
                    'messages': e['n'], 'writers': _chains(e['seqs']),
                    'signatures': sorted(list(s) for s in e['signatures'])})
    return {'duration_s': a.duration, 'edges': out, 'publisher_nodes': pubs}


def cmd_graph(node, a):
    spin_for(node, a.wait_graph)
    res = {'nodes': sorted(full_name(n, ns) for n, ns in node.get_node_names_and_namespaces()),
           'topics': dict(node.get_topic_names_and_types()), 'endpoints': {}}

    def ep(infos):
        return [{'node': full_name(i.node_name, i.node_namespace),
                 'reliability': i.qos_profile.reliability.name,
                 'durability': i.qos_profile.durability.name}
                for i in sorted(infos, key=lambda i: (i.node_namespace, i.node_name))]
    for t in a.topics:
        res['endpoints'][t] = {'publishers': ep(node.get_publishers_info_by_topic(t)),
                               'subscribers': ep(node.get_subscriptions_info_by_topic(t))}
    return res


def cmd_params(node, a):
    from rcl_interfaces.srv import GetParameters, ListParameters
    from rclpy.parameter import parameter_value_to_python
    spin_for(node, a.wait_graph)
    out, failed = {}, []
    for n, ns in sorted(node.get_node_names_and_namespaces()):
        full = full_name(n, ns)
        if n == node.get_name() or any(full.startswith(p) for p in a.skip_prefix):
            continue
        lc = node.create_client(ListParameters, full + '/list_parameters')
        gc = node.create_client(GetParameters, full + '/get_parameters')
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


def _endpoints(node):
    """{node: {subscribes, publishes}} of every node but this one and ros2cli daemons."""
    out = {}
    me = '/' + node.get_name()
    for n, ns in node.get_node_names_and_namespaces():
        full = full_name(n, ns)
        if full == me or '_ros2cli_daemon' in n:
            continue
        try:
            subs = node.get_subscriber_names_and_types_by_node(n, ns)
            pubs = node.get_publisher_names_and_types_by_node(n, ns)
        except Exception as exc:  # noqa: BLE001 - a node that left meanwhile
            out[full] = {'error': str(exc)}
            continue
        out[full] = {'subscribes': {t: ty[0] for t, ty in subs},
                     'publishes': {t: ty[0] for t, ty in pubs}}
    return out


def cmd_io(node, a):
    end = time.monotonic() + a.timeout
    target = a.wait_node
    found = False
    while time.monotonic() < end:
        spin_for(node, 0.5)
        names = {full_name(n, ns) for n, ns in node.get_node_names_and_namespaces()}
        if target in names:
            found = True
            break
    # Then until the endpoint set has not changed for --settle s: a launch file
    # brings its nodes up one after the other, and each node creates its
    # endpoints in its constructor.
    seen, last_change = None, time.monotonic()
    while found and time.monotonic() < end:
        spin_for(node, 0.5)
        cur = _endpoints(node)
        if cur != seen:
            seen, last_change = cur, time.monotonic()
        elif time.monotonic() - last_change >= a.settle:
            break
    return {'wait_node': target, 'found': found,
            'nodes': seen if seen is not None else _endpoints(node)}


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
        cli = node.create_client(typ, '/mission/%s_mission' % name)
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


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('  nodes'):])
    ap.add_argument('--out', required=True, help='JSON file to write')
    ap.add_argument('--wait-graph', type=float, default=30.0,
                    help='seconds to wait for the graph (rates/stamps/jsonstat: max; '
                         'graph/params: fixed, default 10)')
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('nodes')
    p.add_argument('--duration', type=float, required=True)
    p.add_argument('--t0', type=float, default=0.0)
    p.add_argument('--stable', type=float, default=0.0)
    p = sub.add_parser('rates')
    p.add_argument('--duration', type=float, required=True)
    p.add_argument('topics', nargs='+')
    p = sub.add_parser('stamps')
    p.add_argument('--duration', type=float, required=True)
    p.add_argument('--topic', required=True)
    p = sub.add_parser('jsonstat')
    p.add_argument('--duration', type=float, required=True)
    p.add_argument('--topic', required=True)
    p.add_argument('--key', required=True)
    p = sub.add_parser('tf')
    p.add_argument('--duration', type=float, required=True)
    p = sub.add_parser('graph')
    p.add_argument('topics', nargs='*')
    p = sub.add_parser('params')
    p.add_argument('--skip-prefix', nargs='*', default=[])
    p = sub.add_parser('mission')
    p.add_argument('--path', required=True)
    p.add_argument('--wait', type=float, default=60.0)
    p = sub.add_parser('io')
    p.add_argument('--wait-node', required=True)
    p.add_argument('--timeout', type=float, default=60.0)
    p.add_argument('--settle', type=float, default=3.0)
    a = ap.parse_args(argv)
    if a.cmd in ('graph', 'params') and '--wait-graph' not in (argv or sys.argv):
        a.wait_graph = 10.0

    rclpy.init()
    node = rclpy.create_node('phase5_probe_' + a.cmd)
    res = {'nodes': cmd_nodes, 'rates': cmd_rates, 'stamps': cmd_stamps,
           'jsonstat': cmd_jsonstat, 'tf': cmd_tf, 'graph': cmd_graph,
           'params': cmd_params, 'mission': cmd_mission, 'io': cmd_io}[a.cmd](node, a)
    with open(a.out, 'w') as f:
        json.dump(res, f, indent=2, default=str)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
