#!/usr/bin/env python3
"""
Rebuild the goal message the behaviour tree sends mpc_corr for one move.

Through the real code path, as YAML for filter_bag_for_layer.py --inject.

Runs on: the Orin host (native Humble), the harness container, linus (the
side's install sourced: f1tenth_behavior and its messages are imported).
Used to build the MPC replay input of P9.4: a car bag carries the MPC's
inputs but not always its goal (/mpc/goal_drive is not in the mission
logger's topic list, and a goal sent before recording started is lost), and
without a goal mpc_corr never solves. Build the goal ONCE, on one side, and
inject it into the input bag both sides replay (bag_digest.py proves the
two input bags equal).

Steps, each the stack's own code, not a re-implementation:
  1. mission_config.parse_mission(mission)  -- what the behaviour executor
     loads a mission file with;
  2. PublishMoveGoal.update()               -- the BT behaviour that publishes
     the goal (publish_move_goal.py), run for real with its publishers
     swapped for capturing stubs.
Only the goal is written, not /mpc/stop_target (transient-local: a bag
cannot offer it to mpc_corr's transient-local subscription; -1.0, "no wall
stop", is mpc_corr's own default anyway).

Usage: make_mpc_goal.py MISSION.json OUT.yaml [--move N]
       (default mission for P9.4: src/f1tenth_behavior/missions/
        drive_test_straight_2m.json -- a drive move, which mpc_corr never
        reports reached, so it solves for the whole replay)
Prints `TOPIC TYPE` of the goal on its last line.
"""
import argparse
import json
from pathlib import Path
import sys
import types


class _Capture:
    def __init__(self, topic):
        self.topic = topic
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _Logger:
    def info(self, text):
        print('[PublishMoveGoal] ' + text)

    warn = info


def type_name(msg):
    """'pkg/msg/Type' of a rosidl message instance."""
    module = type(msg).__module__.split('.')
    return '%s/%s/%s' % (module[0], module[1], type(msg).__name__)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('Steps'):])
    ap.add_argument('mission')
    ap.add_argument('out')
    ap.add_argument('--move', type=int, default=0)
    a = ap.parse_args(argv)

    import py_trees
    import rclpy
    from rosidl_runtime_py import message_to_yaml

    from f1tenth_behavior.behaviours.publish_move_goal import PublishMoveGoal
    from f1tenth_behavior.mission.mission_config import parse_mission
    from f1tenth_behavior.mission.runtime import MISSION_KEY

    mission = json.loads(Path(a.mission).read_text())
    cfg = parse_mission(mission)
    move = cfg.moves[a.move]
    rclpy.init()
    clock_node = rclpy.create_node('make_mpc_goal')
    try:
        beh = PublishMoveGoal()
        beh.node = types.SimpleNamespace(get_logger=lambda: _Logger(),
                                         get_clock=clock_node.get_clock)
        caps = {}
        for attr in ('goal_distance_pub', 'goal_pose_pub', 'goal_turn_pub', 'goal_drive_pub',
                     'stop_target_pub'):
            topic = getattr(beh, '_%s_topic' % attr[:-4], None) or attr
            caps[attr] = _Capture(topic)
            setattr(beh, attr, caps[attr])
        state = types.SimpleNamespace(current_move=move, goal_dirty=True)
        setattr(beh.blackboard, MISSION_KEY, state)
        status = beh.update()
    finally:
        clock_node.destroy_node()
        rclpy.try_shutdown()
    if status != py_trees.common.Status.SUCCESS:
        raise SystemExit('PublishMoveGoal.update() returned %s' % status)
    goals = [c for k, c in caps.items() if k != 'stop_target_pub' and c.msgs]
    if len(goals) != 1 or len(goals[0].msgs) != 1:
        raise SystemExit('expected exactly one goal message, got %s'
                         % {c.topic: len(c.msgs) for c in caps.values()})
    msg = goals[0].msgs[0]
    Path(a.out).write_text(message_to_yaml(msg))
    print('mission %s move %d (%s): goal on %s' % (cfg.mission_id, a.move, move.id,
                                                   goals[0].topic))
    print(Path(a.out).read_text(), end='')
    print('%s %s' % (goals[0].topic, type_name(msg)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
