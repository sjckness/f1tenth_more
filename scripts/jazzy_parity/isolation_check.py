#!/usr/bin/env python3
"""Discovery-isolation detection for a running full stack (fix batch 4, H1).

Run as a Discovery Server SUPER client (ROS_SUPER_CLIENT=TRUE) so the graph
it sees is the server's whole view.

  isolation_check.py watch --out TIMELINE.jsonl
      Poll the node list once a second until killed; one JSON line per poll:
      {"t": epoch, "nodes": [...]}. Start it right after the bringup.

  isolation_check.py check --run-dir RUN --timeline TIMELINE.jsonl \
      --t-feed EPOCH --duration S --out RESULT.json
      Three independent signals, combined per node:
      1. graph: a node from the expected list that never appeared, or that
         appeared and was then absent from >= 5 consecutive polls (vanished);
      2. output: each expected input-driven topic must deliver at least one
         message within --duration (BEST_EFFORT/VOLATILE subscription, which
         matches reliable and best-effort publishers);
      3. starvation: the node's own log says it is not receiving its input,
         in a line stamped >= t_feed + STARVE_AFTER_S (12 s) (component logs in RUN/supervisor).
      A node is ISOLATED if any signal fires for it.

The expected lists are this stack's Phase 5 configuration (no hardware,
perception off, bag-fed /scan and /odom). Same table is the starting point
for the supervisor watchdog design (fix batch 4 report).
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

# node -> input-driven output topic(s) that must flow once the bag is playing
EXPECTED = {
    '/ekf_filter_node': ['/odometry/filtered'],
    '/ekf_global/ekf_global_filter_node': ['/ekf_global/odometry/filtered'],
    '/slam_toolbox': ['/slam/pose'],
    '/slam_pose_relay_node': ['/slam/pose_calibrated'],
    '/costmap_boundary_node': ['/costmap/front_clearance'],
    '/lidar_front_wall_node': ['/perception/lidar_front_wall'],
    '/obstacle_clearance_node': ['/obstacle_clearance'],
    '/swept_clearance_node': ['/perception/swept_clearance/lidar'],
    '/wall_distance_node': ['/perception/d_wall/segment'],
    '/behavior_executor_node': ['/behavior/tree_status'],
    '/system_observer_node': ['/diagnostics/system_status'],
    '/diagnostics_server_node': ['/diagnostics/battery_status'],
    '/joint_state_publisher': ['/joint_states'],
    '/mpc_corr': ['/drive'],
    '/ackermann_mux': ['/ackermann_drive'],
}
# nodes expected in the graph that have no input-driven output we can use
GRAPH_ONLY = ['/component_supervisor_node', '/robot_state_publisher',
              '/static_baselink_to_laser', '/ekf_cost_observer_node',
              '/mission_logger_node', '/semantic_layer_node', '/costmap_renderer_node',
              '/map_server', '/lifecycle_manager_map', '/twist_to_ackermann_node',
              '/foxglove_bridge']
# node -> (component log glob, regex of "my input never arrived")
STARVATION = {
    '/mpc_corr': ('*navigation.launch.py.log', r'ODOM non disponibile'),
    '/costmap_boundary_node': ('*costmap.launch.py.log', r'no message ever received'),
    '/swept_clearance_node': ('*swept_clearance.launch.py.log', r'lidar never received'),
}
# Inputs that legitimately arrive late after the feed starts: the SLAM map
# (costmap_boundary_node) is first published several seconds in. A node still
# logging starvation this long after the feed started is not receiving.
STARVE_AFTER_S = 12.0
_STAMP = re.compile(r'\[(\d{10}\.\d+)\]')


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


def graph_signal(timeline, expected_nodes):
    polls = [json.loads(line) for line in open(timeline) if line.strip()]
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
            out[n] = (f'vanished: in graph from {polls[first]["t"]:.1f}, absent from '
                      f'{polls[gone]["t"]:.1f} ({best} consecutive polls)')
    return out, len(polls)


def output_signal(duration):
    rclpy.init()
    node = rclpy.create_node('isolation_check')
    end = time.monotonic() + 30.0
    topics = {t for ts in EXPECTED.values() for t in ts}
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
            out[n] = f'{hits} "{regex}" lines from feed start + {STARVE_AFTER_S:.0f} s on'
    return out


def cmd_check(a):
    expected_nodes = sorted(set(EXPECTED) | set(GRAPH_ONLY))
    graph, n_polls = graph_signal(a.timeline, expected_nodes)
    counts, not_in_graph = output_signal(a.duration)
    starv = starvation_signal(a.run_dir, a.t_feed)
    isolated = {}
    for n in expected_nodes:
        reasons = []
        if n in graph:
            reasons.append('graph: ' + graph[n])
        for t in EXPECTED.get(n, []):
            if counts.get(t, 0) == 0:
                reasons.append(f'output: no message on {t} in {a.duration:.0f} s')
        if n in starv:
            reasons.append('starved: ' + starv[n])
        if reasons:
            isolated[n] = reasons
    res = {'isolated': isolated, 'n_isolated': len(isolated), 'polls': n_polls,
           'topic_counts': counts, 'topics_not_in_graph': not_in_graph,
           'graph_signal': graph, 'starvation_signal': starv}
    with open(a.out, 'w') as f:
        json.dump(res, f, indent=2)
    print(json.dumps({'n_isolated': len(isolated), 'isolated': isolated}))
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    w = sub.add_parser('watch')
    w.add_argument('--out', required=True)
    c = sub.add_parser('check')
    c.add_argument('--run-dir', required=True)
    c.add_argument('--timeline', required=True)
    c.add_argument('--t-feed', type=float, required=True)
    c.add_argument('--duration', type=float, default=8.0)
    c.add_argument('--out', required=True)
    a = ap.parse_args()
    return cmd_watch(a) if a.cmd == 'watch' else cmd_check(a)


if __name__ == '__main__':
    sys.exit(main())
