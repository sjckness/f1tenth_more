#!/usr/bin/env python3
"""Talk to component_supervisor_node directly, without the ros2 CLI daemon.

Why this exists: under ROS_DISCOVERY_SERVER the `ros2` CLI is an unreliable
narrator about this stack, in two different directions, and both were burning
real debugging time:

  - Right after a launch it under-reports. Discovery needs ~10-25s to relay
    the graph to a new participant, but `ros2 node list` / `ros2 service list`
    query immediately and return 0. "The supervisor's services don't exist"
    and "the supervisor is broken" look identical from the terminal.
  - Later it over-reports. The ros2 daemon caches the graph and survives stack
    restarts, so it happily lists ghost nodes from runs that ended long ago
    (observed: 171 nodes listed for a stack that really had ~35).

This tool creates its own rclpy node, spins it until discovery has actually
settled, and reports what IS there -- then calls the supervisor's services
directly. No daemon, no cache, no guessing which lie you got.

Usage:
  ./scripts/stackctl.py status                 # what's actually up, incl. mission preflight
  ./scripts/stackctl.py restart navigation     # RestartComponent
  ./scripts/stackctl.py start   diagnostics    # ComponentControl START
  ./scripts/stackctl.py stop    intelligence   # ComponentControl SHUTDOWN
  ./scripts/stackctl.py --settle 20 status     # longer discovery wait on a busy graph

Exit code is 0 on success, 1 on failure, so it composes in shell scripts.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from f1tenth_messages.srv import ComponentControl, RestartComponent

# Node names /mission/start_mission's own preflight requires -- see
# f1tenth_behavior/mission/preflight.py. Duplicated here deliberately (a
# diagnostic script must not import the behavior package just to print a
# hint), so if that list changes this one is a stale COPY, not a break --
# it only ever prints, never gates anything.
_PREFLIGHT_NODES = ('mpc_corr', 'ackermann_to_vesc_node')
_ACTIONS = {'stop': 0, 'start': 1, 'restart_action': 2}


def _settle(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def _call(node, cli, req, path, timeout):
    if not cli.wait_for_service(timeout_sec=timeout):
        print(f'FAIL: {path} never became available within {timeout:.0f}s')
        return None
    t0 = time.time()
    fut = cli.call_async(req)
    while not fut.done() and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not fut.done():
        print(f'FAIL: {path} accepted the request but never replied '
              f'({timeout:.0f}s) -- the supervisor is single-threaded, so a '
              f'restart already in flight can block this one.')
        return None
    print(f'  replied in {time.time() - t0:.1f}s')
    return fut.result()


def cmd_status(node):
    names = {n for n, _ in node.get_node_names_and_namespaces()}
    svcs = dict(node.get_service_names_and_types())
    print(f'nodes in graph : {len(names)}')
    print(f'services       : {len(svcs)}')
    print()
    print('supervisor services:')
    for path in ('/restart_component',
                 '/component_supervisor_node/control_component',
                 '/component_supervisor_node/run_calibration'):
        print(f'  {"OK      " if path in svcs else "MISSING "} {path}')
    print()
    print('/mission/start_mission preflight nodes:')
    ok = True
    for req in _PREFLIGHT_NODES:
        present = req in names
        ok = ok and present
        print(f'  {"OK      " if present else "MISSING "} {req}')
    if not ok:
        print('  -> start_mission WILL refuse: preflight needs every node above.')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('command', choices=['status', 'restart', 'start', 'stop'])
    ap.add_argument('component', nargs='?',
                    help='component name (required for restart/start/stop)')
    ap.add_argument('--settle', type=float, default=10.0,
                    help='seconds to let discovery settle before acting (default 10)')
    ap.add_argument('--timeout', type=float, default=90.0,
                    help='seconds to wait for a service reply (default 90)')
    args = ap.parse_args()

    if args.command != 'status' and not args.component:
        ap.error(f'{args.command} needs a component name')

    rclpy.init()
    node = Node('stackctl')
    try:
        print(f'settling discovery for {args.settle:.0f}s ...')
        _settle(node, args.settle)

        if args.command == 'status':
            return cmd_status(node)

        if args.command == 'restart':
            path = '/restart_component'
            cli = node.create_client(RestartComponent, path)
            req = RestartComponent.Request()
            req.component_name = args.component
        else:
            path = '/component_supervisor_node/control_component'
            cli = node.create_client(ComponentControl, path)
            req = ComponentControl.Request()
            req.component_name = args.component
            req.action = (ComponentControl.Request.SHUTDOWN if args.command == 'stop'
                          else ComponentControl.Request.START)

        print(f'{args.command} {args.component!r} via {path}')
        resp = _call(node, cli, req, path, args.timeout)
        if resp is None:
            return 1
        print(f'  success={resp.success}')
        print(f'  message={resp.message}')
        return 0 if resp.success else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
