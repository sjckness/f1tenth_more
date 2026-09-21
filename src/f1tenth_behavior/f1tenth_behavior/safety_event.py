"""/safety/event: one message per emergency stop, on the tick the stop begins.

    /safety/event  std_msgs/String, JSON
                   {event: "estop", cause, source: "behavior_tree", lane}

The consumer is f1tenth_logger's test-campaign logger, which reads exactly two
event names, ``estop`` and ``contact``. This module publishes the first; the
second comes from f1tenth_perception's obstacle_clearance_node, the only
place in the stack that measures the footprint against a sensor.

WHY THE BT LANE AND NOT EACH TRIGGER. Every emergency stop in this stack --
the operator's /mission/emergency_stop, IsProximityTooClose, IsBatteryLow,
IsSystemOverheated -- stops the car the same way: the ``emergency`` lane wins
the root Selector and its Stop drives /safety_stop. The lane is the one place
all of them pass through, so its rising edge is published here and nowhere
else. Publishing from MissionLoader's service handler as well would report an
operator e-stop twice, 100 ms apart: once from the latch, once from
IsEmergencyStopTriggered tripping the lane on the next tick.

The conditions are levels, re-evaluated every tick, so the edge is kept here:
an event when the lane becomes active, nothing while it stays active, and a
new event if it clears and trips again. ``cause`` is the same string
/behavior/tree_status carries as emergency_trip (the first tripped
condition, with its tripped_reason when it has one), so the two can be
cross-checked in a bag.

Known blind spot, inherent to an edge on the lane: while the lane is already
active for one cause, a second cause tripping produces no new event. After an
operator e-stop the lane is latched for the life of the process, so nothing
after it is reported.

Nothing here can affect the tree: it runs as a post-tick handler, reads
statuses the tick already set, and swallows its own failures.
"""

import json

import py_trees
from std_msgs.msg import String

__all__ = ['active_lane', 'emergency_trip', 'make_safety_event_publisher']

EMERGENCY_LANE = 'emergency'


def active_lane(root):
    """Name of the root child that ran this tick, or '' if none succeeded.

    The first succeeding child is the one a memory=False Selector stopped at.
    """
    for child in root.children:
        if child.status == py_trees.common.Status.SUCCESS:
            return child.name
    return ''


def emergency_trip(root):
    """The first tripped emergency condition, with its reason when it has one.

    Walked by name so this keeps working when the emergency lane's
    composition changes with the enable_lidar_safety_stop/enable_sys_obs
    flags. '' when nothing tripped.
    """
    for child in root.children:
        if child.name != EMERGENCY_LANE:
            continue
        for cond in child.children:
            if cond.name != 'emergency_condition':
                continue
            for leaf in cond.children:
                if leaf.status == py_trees.common.Status.SUCCESS:
                    # tripped_reason is optional -- only IsSystemOverheated
                    # currently exposes it (see that behaviour).
                    reason = getattr(leaf, 'tripped_reason', '')
                    return f'{leaf.name}: {reason}' if reason else leaf.name
    return ''


def make_safety_event_publisher(node, publisher):
    """Post-tick handler: an ``estop`` event on each rising edge of the lane."""
    state = {'active': False}

    def _publish(tree):
        try:
            root = tree.root
            active = active_lane(root) == EMERGENCY_LANE
            rising = active and not state['active']
            state['active'] = active
            if not rising:
                return
            payload = {
                'event': 'estop',
                'cause': emergency_trip(root) or 'emergency lane active',
                'source': 'behavior_tree',
                'lane': EMERGENCY_LANE,
            }
            publisher.publish(String(data=json.dumps(payload)))
        except Exception as exc:  # noqa: BLE001 - telemetry must never stop the tree
            node.get_logger().warn(
                f'/safety/event not published: {type(exc).__name__}: {exc}',
                throttle_duration_sec=5.0)

    return _publish
