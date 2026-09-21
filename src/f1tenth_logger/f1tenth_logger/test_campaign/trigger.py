#!/usr/bin/env python3
"""Drive the logger by hand, with no LLM and no mission node in the loop.

    ros2 run f1tenth_logger test_campaign_trigger 0                # M00_calibration
    ros2 run f1tenth_logger test_campaign_trigger 0 --countdown 5

Publishes a plan_result that looks like a successful "initial" LLM call
(latency 0, no plan), then ``mission_loaded``, waits out the countdown,
publishes ``mission_started`` -- and then drives the calibration manoeuvre
yourself. Press Enter to publish ``mission_finished``; Ctrl+C publishes
``mission_aborted`` instead. Either way the logger closes the test properly.

The prompt text is read from the campaign's ``prompts.yaml`` so the hash the
logger checks matches, exactly as it would from the real planner.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String

from f1tenth_logger.test_campaign.robot_logger import DEFAULT_CAMPAIGN, find_root


def load_prompt(root, campaign, prompt_num):
    """(text, mission) of prompt_num in the campaign's prompt table."""
    path = find_root(root) / campaign / "prompts.yaml"
    if not path.exists():
        raise SystemExit(f"no prompt table at {path}")
    with open(path, encoding="utf-8") as fh:
        table = yaml.safe_load(fh) or []
    for entry in table:
        if int(entry["prompt_num"]) == prompt_num:
            return entry["text"], entry["mission"]
    known = sorted(int(e["prompt_num"]) for e in table)
    raise SystemExit(f"prompt_num {prompt_num} is not in {path} (known: {known})")


class Trigger(Node):
    def __init__(self, plan_topic, event_topic):
        super().__init__("test_campaign_trigger")
        self.plan_pub = self.create_publisher(String, plan_topic, 10)
        self.event_pub = self.create_publisher(String, event_topic, 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_for_logger(self, timeout=5.0):
        """Publishing into the void loses the test, so wait for the subscriber."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (self.plan_pub.get_subscription_count() > 0
                    and self.event_pub.get_subscription_count() > 0):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def send(self, publisher, payload):
        publisher.publish(String(data=json.dumps(payload)))
        rclpy.spin_once(self, timeout_sec=0.05)

    def plan_result(self, prompt_num, text, plan_id):
        stamp = self.now()
        self.send(self.plan_pub, {
            "prompt_num": prompt_num,
            "prompt_text": text,
            "kind": "initial",
            "t_prompt_sent": stamp,
            "t_response_received": stamp,
            "latency_ms": 0.0,
            "status": "ok",
            "error": "",
            "plan_id": plan_id,
            "plan": None,
        })

    def mission_event(self, event, plan_id, reason="", countdown_s=None):
        payload = {"event": event, "plan_id": plan_id, "reason": reason}
        if countdown_s is not None:
            payload["countdown_s"] = countdown_s
        self.send(self.event_pub, payload)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prompt_num", type=int)
    parser.add_argument("--countdown", type=float, default=3.0,
                        help="standstill seconds between loaded and started")
    parser.add_argument("--root", default=None)
    parser.add_argument("--campaign", default=DEFAULT_CAMPAIGN)
    parser.add_argument("--plan-topic", default="/test/plan_result")
    parser.add_argument("--event-topic", default="/test/mission_event")
    args = parser.parse_args(argv)

    print(f"campaign: {find_root(args.root) / args.campaign}")
    text, mission = load_prompt(args.root, args.campaign, args.prompt_num)
    plan_id = f"manual_{int(time.time())}"

    rclpy.init()
    node = Trigger(args.plan_topic, args.event_topic)
    try:
        if not node.wait_for_logger():
            print("WARNING: nothing is subscribed to the test topics -- is "
                  "test_campaign_logger running?", file=sys.stderr)

        print(f"prompt {args.prompt_num} -> {mission}: {text}")
        node.plan_result(args.prompt_num, text, plan_id)
        node.mission_event("mission_loaded", plan_id, countdown_s=args.countdown)
        print(f"countdown {args.countdown:.1f} s (keep still -- this is the "
              f"noise floor)")
        deadline = time.monotonic() + args.countdown
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.mission_event("mission_started", plan_id)
        print("STARTED -- run the manoeuvre, then press Enter to finish "
              "(Ctrl+C aborts)")
        input()
        node.mission_event("mission_finished", plan_id, reason="operator ended it")
        print("finished")
    except KeyboardInterrupt:
        node.mission_event("mission_aborted", plan_id,
                           reason="operator interrupted the trigger")
        print("\naborted")
    finally:
        # let the last message leave before the publisher disappears
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
