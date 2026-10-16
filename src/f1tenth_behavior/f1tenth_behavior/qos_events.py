"""Drop rclpy's default per-publisher/per-subscription QoS event handlers.

WHY (cpu-cleanup, 2026-10-16): rclpy gives every publisher and subscription
a default "incompatible QoS" event handler, and each one is a Waitable the
executor re-adds to its wait set and re-checks on EVERY wake-up, whether or
not an event ever fires. behavior_executor_node has dozens of them (76 on
linus under Jazzy, which adds more event kinds than Humble; at least one per
publisher/subscription the behaviours, MissionLoader and py_trees_ros make)
and wakes several hundred times a second for /scan, both EKF odometries,
the virtual wall and the MPC distances while the car is parked. Measured on
linus with the stack's input rates: 21.7% -> 16.3% of a core for the whole
node with only this change -- about a quarter of the node's CPU spent
re-checking handlers that only ever log.

WHAT IS LOST: the WARN this node would print when a peer offers/requests an
incompatible QoS. The peer still prints its own (an incompatible-QoS event
fires on BOTH ends, and every peer of this node -- urg_node, the EKFs,
mpc_corr, ackermann_mux, lidar_front_wall_node -- keeps its default handler),
so a mismatch is still reported, just by the other process.

bt_keep_qos_event_handlers:=true on behavior_executor_node keeps them.
"""

try:  # Jazzy and later
    from rclpy.event_handler import EventHandler
except ImportError:  # Humble
    from rclpy.qos_event import QoSEventHandler as EventHandler


def drop_qos_event_handlers(node):
    """Remove and destroy every QoS event handler on `node`'s publishers and
    subscriptions. Call once, after the last publisher/subscription exists
    and before spinning. Returns how many were dropped.

    Each entity's own event_handlers list is emptied as well, so a later
    destroy_publisher/destroy_subscription (or destroy_node) does not try to
    remove a waitable that is no longer on the node.
    """
    dropped = 0
    for entity in list(node.publishers) + list(node.subscriptions):
        handlers = list(getattr(entity, 'event_handlers', None) or [])
        for handler in handlers:
            if not isinstance(handler, EventHandler):
                continue
            node.remove_waitable(handler)
            handler.destroy()
            dropped += 1
        if handlers:
            entity.event_handlers = [
                h for h in handlers if not isinstance(h, EventHandler)]
    return dropped
