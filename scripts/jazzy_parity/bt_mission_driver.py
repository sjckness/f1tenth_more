#!/usr/bin/env python3
"""Phase 4: load and start the humble_obstacle_run mission in the replayed BT
the way the live path does, at a fixed point in bag (sim) time.

The live path (llm_planner_node.py): translate the intent, write the mission
JSON (`json.dumps(mission, indent=2, ensure_ascii=False) + '\\n'`,
_write_mission_file), call /mission/load_mission with its path, then
/mission/start_mission. This driver does the same, with the mission built by
make_mpc_goal.build_mission() (translate() of the golden intent, asserted
equal to the bag's mission id and the committed fixture), and waits for sim
time to reach START_BAG_SEC before calling, so every replay -- on either
distro -- starts the mission at the same bag time.

Writes a JSON log (call times in sim and wall time, responses) to --log.

Usage:
  bt_mission_driver.py --repo REPO --bag-start-ns N --start-at-bag-sec S \\
      --mission-dir DIR --log FILE [--ros-args -p use_sim_time:=true]
"""
import argparse
import json
import os
import sys
import time

import rclpy
from f1tenth_messages.srv import LoadMission
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_mpc_goal import build_mission  # noqa: E402


def call(node, client, req, timeout=10.0):
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', required=True)
    ap.add_argument('--bag-start-ns', type=int, required=True)
    ap.add_argument('--start-at-bag-sec', type=float, required=True)
    ap.add_argument('--mission-dir', required=True)
    ap.add_argument('--log', required=True)
    args, ros_args = ap.parse_known_args()

    mission = build_mission(os.path.expanduser(args.repo))
    path = os.path.join(args.mission_dir, mission['mission_id'] + '.json')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(json.dumps(mission, indent=2, ensure_ascii=False) + '\n')

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = rclpy.create_node('bt_mission_driver')
    load_cli = node.create_client(LoadMission, '/mission/load_mission')
    start_cli = node.create_client(Trigger, '/mission/start_mission')
    target_ns = args.bag_start_ns + int(round(args.start_at_bag_sec * 1e9))
    log = {'mission_path': path, 'mission_id': mission['mission_id'],
           'start_at_bag_sec': args.start_at_bag_sec}

    while rclpy.ok() and node.get_clock().now().nanoseconds < target_ns:
        rclpy.spin_once(node, timeout_sec=0.01)

    def stamp():
        return {'bag_sec': (node.get_clock().now().nanoseconds - args.bag_start_ns) * 1e-9,
                'wall': time.time()}

    req = LoadMission.Request()
    req.path = path
    log['load_called'] = stamp()
    r = call(node, load_cli, req)
    log['load_response'] = None if r is None else {'success': r.success, 'message': r.message}
    log['load_returned'] = stamp()
    r = call(node, start_cli, Trigger.Request())
    log['start_response'] = None if r is None else {'success': r.success, 'message': r.message}
    log['start_returned'] = stamp()
    with open(args.log, 'w') as f:
        json.dump(log, f, indent=2)
    print(json.dumps(log, indent=2), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
