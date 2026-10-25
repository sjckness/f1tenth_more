#!/usr/bin/env python3
"""
Discovery-isolation detection for a running full stack (backlog H1).

Runs on: the Orin host (native Humble), inside the harness container, linus;
started by phase5_bringup.sh isolation, as a Discovery Server SUPER client
(ROS_SUPER_CLIENT=TRUE) so the graph it sees is the server's whole view.
Compares the run against what the stack is expected to bring up (built-in
table for main-jazzy, or the node set of a reference run).

  isolation_check.py watch --out TIMELINE.jsonl
      Poll the node list once a second until stopped; one JSON line per poll:
      {"t": epoch, "nodes": [...]}. Start it right after the bring-up.

  isolation_check.py check --run-dir RUN --timeline TIMELINE.jsonl \
      --t-feed EPOCH --duration S --out RESULT.json [--feed bag|live]
      [--expect-from REF_RUN/nodes_startup.json]
      Three independent signals, combined per node:
      1. graph: an expected node that never appeared, or that appeared and
         was then absent from >= 5 consecutive polls (vanished);
      2. output: each expected input-driven topic must deliver at least one
         message within --duration (BEST_EFFORT/VOLATILE subscription, which
         matches reliable and best-effort publishers);
      3. starvation: the node's own log says it is not receiving its input,
         in a line stamped >= t_feed + STARVE_AFTER_S (12 s) (component logs
         in RUN/supervisor).
      A node is ISOLATED if any signal fires for it. --expect-from replaces
      the expected node set with a reference run's (generated names dropped);
      the output table then only applies to nodes in that set.
"""
import argparse
import glob
import json
import os
import re
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

# node -> input-driven output topic(s) that must flow once the feed runs
# (main-jazzy and the car's campaign line share these names). A bag feed
# (phase5_bringup.sh FEED=bag) provides /scan and /odom; a live feed the real
# drivers.
EXPECTED = {
    '/ekf_filter_node': ['/odometry/filtered'],
    '/ekf_global/ekf_global_filter_node': ['/ekf_global/odometry/filtered'],
    # /slam/pose only moves with the car (minimum travel); the map comes every
    # 5 s whatever the motion.
    '/slam_toolbox': ['/slam/map'],
    '/costmap_boundary_node': ['/costmap/front_clearance'],
    '/lidar_front_wall_node': ['/perception/walls'],
    '/obstacle_clearance_node': ['/obstacle_clearance'],
    '/behavior_executor_node': ['/behavior/tree_status'],
    '/system_observer_node': ['/diagnostics/system_status'],
    '/joint_state_publisher': ['/joint_states'],
    # /drive every control tick, solving or not (no goal: zero command).
    '/mpc_corr': ['/drive'],
    '/ackermann_mux': ['/ackermann_drive'],
}
# Only with FEED=live (the drivers are emptied out of a bag-fed run).
EXPECTED_LIVE = {
    '/urg_node': ['/scan'],
    '/vesc_to_odom_node': ['/odom'],
}
# nodes expected in the graph that have no input-driven output to use
GRAPH_ONLY = ['/component_supervisor_node', '/robot_state_publisher',
              '/static_baselink_to_laser', '/semantic_layer_node', '/costmap_renderer_node',
              '/map_server', '/lifecycle_manager_map', '/twist_to_ackermann_node',
              '/hard_stop_node', '/diagnostics_server_node', '/reset_manager',
              '/slam_pose_relay_node']
# node -> (component log glob, regex of "my input never arrived")
STARVATION = {
    '/mpc_corr': ('*navigation.launch.py.log', r'ODOM non disponibile'),
    '/costmap_boundary_node': ('*costmap.launch.py.log', r'no message ever received'),
}
# Inputs that legitimately arrive late after the feed starts: the SLAM map
# (costmap_boundary_node) is first published several seconds in. A node still
# logging starvation this long after the feed started is not receiving.
STARVE_AFTER_S = 12.0
_STAMP = re.compile(r'\[(\d{10}\.\d+)\]')
GENERATED = re.compile(r'(transform_listener_impl_[0-9a-f]+|launch_ros_\d+|phase5_probe_\w+'
                       r'|isolation_\w+)$')


def full_names(node):
    return sorted(('' if ns == '/' else ns) + '/' + n
                  for n, ns in node.get_node_names_and_namespaces())


def cmd_watch(a):
    rclpy.init()
    node = rclpy.create_node('isolation_watch')
    with open(a.out, 'a') as f:
        try:
            while rclpy.ok():
                end = time.monotonic() + 1.0
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.1)
                f.write(json.dumps({'t': time.time(), 'nodes': full_names(node)}) + '\n')
                f.flush()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
    return 0


def expectations(feed, expect_from=None):
    """Return (expected node names, {node: output topics}) for this run."""
    outputs = dict(EXPECTED)
    if feed == 'live':
        outputs.update(EXPECTED_LIVE)
    if expect_from:
        with open(expect_from) as f:
            nodes = {n for n in json.load(f)['final'] if not GENERATED.search(n)}
        return sorted(nodes), {n: t for n, t in outputs.items() if n in nodes}
    return sorted(set(outputs) | set(GRAPH_ONLY)), outputs


def graph_signal(polls, expected_nodes):
    """{node: reason} for expected nodes that never appeared or vanished."""
    out = {}
    for n in expected_nodes:
        present = [n in p['nodes'] for p in polls]
        if not any(present):
            out[n] = 'never in graph'
            continue
        first = present.index(True)
        run = best = 0
        for x in present[first:]:
            run = 0 if x else run + 1
            best = max(best, run)
        if best >= 5:
            gone = next(i for i in range(first, len(present))
                        if not present[i] and all(not y for y in present[i:i + 5]))
            out[n] = ('vanished: in graph from %.1f, absent from %.1f (%d consecutive polls)'
                      % (polls[first]['t'], polls[gone]['t'], best))
    return out


def output_signal(topics, duration):
    rclpy.init()
    node = rclpy.create_node('isolation_check')
    end = time.monotonic() + 30.0
    while True:
        rclpy.spin_once(node, timeout_sec=0.2)
        types = dict(node.get_topic_names_and_types())
        if topics <= set(types) or time.monotonic() > end:
            break
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    counts = {t: 0 for t in topics}
    for t in topics:
        if t in types:
            node.create_subscription(get_message(types[t][0]), t,
                                     lambda _m, t=t: counts.__setitem__(t, counts[t] + 1),
                                     qos, raw=True)
    end = time.monotonic() + duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    return counts, sorted(topics - set(types))


def starvation_signal(run_dir, t_feed):
    out = {}
    for n, (pattern, regex) in STARVATION.items():
        hits = 0
        for f in glob.glob(os.path.join(run_dir, 'supervisor', pattern)):
            for line in open(f, errors='replace'):
                if re.search(regex, line):
                    m = _STAMP.search(line)
                    if m and float(m.group(1)) >= t_feed + STARVE_AFTER_S:
                        hits += 1
        if hits:
            out[n] = '%d "%s" lines from feed start + %.0f s on' % (hits, regex, STARVE_AFTER_S)
    return out


def cmd_check(a):
    expected_nodes, outputs = expectations(a.feed, a.expect_from)
    with open(a.timeline) as f:
        polls = [json.loads(line) for line in f if line.strip()]
    graph = graph_signal(polls, expected_nodes)
    counts, not_in_graph = output_signal({t for ts in outputs.values() for t in ts}, a.duration)
    starv = starvation_signal(a.run_dir, a.t_feed)
    isolated = {}
    for n in expected_nodes:
        reasons = []
        if n in graph:
            reasons.append('graph: ' + graph[n])
        for t in outputs.get(n, []):
            if counts.get(t, 0) == 0:
                reasons.append('output: no message on %s in %.0f s' % (t, a.duration))
        if n in starv:
            reasons.append('starved: ' + starv[n])
        if reasons:
            isolated[n] = reasons
    res = {'isolated': isolated, 'n_isolated': len(isolated), 'polls': len(polls),
           'expected_nodes': expected_nodes, 'topic_counts': counts,
           'topics_not_in_graph': not_in_graph, 'graph_signal': graph,
           'starvation_signal': starv}
    with open(a.out, 'w') as f:
        json.dump(res, f, indent=2)
    print(json.dumps({'n_isolated': len(isolated), 'isolated': isolated}))
    return 0 if not isolated else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('  isolation_check.py watch'):])
    sub = ap.add_subparsers(dest='cmd', required=True)
    w = sub.add_parser('watch')
    w.add_argument('--out', required=True)
    c = sub.add_parser('check')
    c.add_argument('--run-dir', required=True)
    c.add_argument('--timeline', required=True)
    c.add_argument('--t-feed', type=float, required=True)
    c.add_argument('--duration', type=float, default=8.0)
    c.add_argument('--feed', choices=['bag', 'live'], default='bag')
    c.add_argument('--expect-from')
    c.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    return cmd_watch(a) if a.cmd == 'watch' else cmd_check(a)


if __name__ == '__main__':
    sys.exit(main())
