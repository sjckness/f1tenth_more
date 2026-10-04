#!/usr/bin/env python3
"""Phase 3: rebuild the /mpc/goal_drive message the BT sent for the
humble_obstacle_run mission, through the real code path, and write it as YAML
for filter_bag_for_layer.py --inject.

The goal itself was never recorded: /mpc/goal_drive is not in the bag's topic
list, and the move started ~13.5 s before recording did (see the Phase 3
report, Step 1). What the bag does carry is the mission file name,
llm_generated/llm_2b356aac445f.json (/mission/status). That id is
sha1(canonical intent)[:12] (plan_translate.translate), and the committed
golden fixture llm/test/golden/ex6_stop_two_metres_from_wall.json carries the
same id, so the intent is known exactly. This script then repeats every step
the live stack took, using the stack's own functions rather than a
re-implementation:

  1. plan_translate.translate(intent)  -- llm_planner_node.py:1184 calls it
     the same way (default TranslatorConfig, go_to irrelevant: no go_to phase).
     Checked: mission_id matches the one in the bag, and the mission equals
     the golden fixture's committed mission.
  2. mission_config.parse_mission(mission) -- what the behaviour executor
     loads the written mission file with.
  3. PublishMoveGoal.update()           -- the BT behaviour that published the
     goal (publish_move_goal.py, the `elif move.drive is not None` branch),
     run for real with its publisher swapped for a capturing stub.

Usage: make_mpc_goal.py OUT_YAML [REPO]
  REPO defaults to this script's repo; pass it when running from a copy
  outside the repo (the Orin at e47e646 has no scripts/jazzy_parity/).
Runs on Jazzy and Humble (ROS Python env needed for f1tenth_behavior/llm).
"""
import json
import sys
import types
from pathlib import Path

import py_trees
from rosidl_runtime_py import message_to_yaml

from f1tenth_behavior.behaviours.publish_move_goal import PublishMoveGoal
from f1tenth_behavior.mission.mission_config import parse_mission
from f1tenth_behavior.mission.runtime import MISSION_KEY
from llm.plan_translate import translate

GOLDEN_REL = 'src/f1tenth_intelligence/llm/test/golden/ex6_stop_two_metres_from_wall.json'
BAG_MISSION_ID = 'llm_2b356aac445f'  # from the bag's /mission/status json_path


class _Capture:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _Logger:
    def info(self, text):
        print('[PublishMoveGoal] ' + text)


def build_mission(repo):
    """Step 1: the mission dict, via translate() of the golden intent, checked
    against the bag's mission id and the committed fixture. Also used by
    bt_mission_driver.py (Phase 4), so the BT replay loads the same mission."""
    golden = json.loads((Path(repo) / GOLDEN_REL).read_text())
    mission = translate(golden['intent']).mission
    assert mission['mission_id'] == BAG_MISSION_ID, mission['mission_id']
    assert mission == golden['mission'], 'translation differs from the golden fixture'
    return mission


def main():
    out = Path(sys.argv[1])
    repo = Path(sys.argv[2]).expanduser() if len(sys.argv) > 2 else Path(__file__).resolve().parents[2]
    mission = build_mission(repo)
    print('mission %s: translate(intent) == golden fixture' % mission['mission_id'])

    cfg = parse_mission(mission)
    move = cfg.moves[0]
    assert move.drive is not None and len(cfg.moves) == 1

    beh = PublishMoveGoal()
    beh.node = types.SimpleNamespace(get_logger=lambda: _Logger())
    cap = _Capture()
    beh.goal_drive_pub = cap
    state = types.SimpleNamespace(current_move=move, goal_dirty=True)
    setattr(beh.blackboard, MISSION_KEY, state)
    status = beh.update()
    assert status == py_trees.common.Status.SUCCESS
    assert len(cap.msgs) == 1 and state.goal_dirty is False
    msg = cap.msgs[0]

    out.write_text(message_to_yaml(msg))
    print('DriveCommand ->', out)
    print(out.read_text(), end='')


if __name__ == '__main__':
    main()
