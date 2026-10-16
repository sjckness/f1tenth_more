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

bt_keep_qos_event_handlers (stack_params.yaml, launch arg of
behavior_bringup.launch.py) true keeps them.
"""

try:  # Jazzy and later
    from rclpy.event_handler import EventHandler
except ImportError:  # Humble: rclpy/qos_event.py:63 QoSEventHandler(Waitable)
    try:
        from rclpy.qos_event import QoSEventHandler as EventHandler
    except ImportError:  # neither: drop_qos_event_handlers becomes a no-op
        EventHandler = None

# The rclpy internals this relies on, all present in Humble rclpy 3.3.21
# (/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy on the Orin):
#   node.py:231/236/261  Node.publishers / .subscriptions / .waitables
#   node.py:1213         Node.remove_waitable(waitable)
#   publisher.py:57, subscription.py:70   entity.event_handlers (a list)
#   node.py:1562/1580    destroy_publisher/destroy_subscription remove each
#                        entity.event_handlers item from the node again
#   qos_event.py:125     QoSEventHandler.destroy()
# If any is missing (another rclpy), nothing is touched: one WARN, return 0.
_NODE_ATTRS = ('publishers', 'subscriptions', 'waitables', 'remove_waitable')


def _unsupported_reason(node):
    if EventHandler is None:
        return 'no rclpy QoS event handler class (rclpy.event_handler / rclpy.qos_event)'
    for attr in _NODE_ATTRS:
        if not hasattr(node, attr):
            return f'Node has no {attr!r}'
    for entity in list(node.publishers) + list(node.subscriptions):
        handlers = getattr(entity, 'event_handlers', None)
        if not isinstance(handlers, list):
            return f'{type(entity).__name__}.event_handlers is not a list'
        if any(isinstance(h, EventHandler) and not hasattr(h, 'destroy') for h in handlers):
            return 'event handler without destroy()'
    return None


def drop_qos_event_handlers(node, logger=None):
    """Remove and destroy every QoS event handler on `node`'s publishers and
    subscriptions. Call once, after the last publisher/subscription exists
    and before spinning. Returns how many were dropped.

    Each entity's own event_handlers list is emptied as well, so a later
    destroy_publisher/destroy_subscription (or destroy_node) does not try to
    remove a waitable that is no longer on the node.

    NEVER RAISES. The internals are checked first and nothing is changed if
    one is missing; an unexpected error part-way is logged and the count so
    far returned. Either way one WARN on `logger` (default: the node's).
    """
    log = logger or node.get_logger()
    try:
        reason = _unsupported_reason(node)
    except Exception as exc:  # noqa: BLE001 - a probe must not take the BT down
        reason = f'probe failed: {exc!r}'
    if reason is not None:
        log.warn(f'QoS event handlers kept ({reason}); executor wake-ups cost more.')
        return 0
    dropped = 0
    try:
        for entity in list(node.publishers) + list(node.subscriptions):
            for handler in list(entity.event_handlers):
                if not isinstance(handler, EventHandler):
                    continue
                # Node and entity stay consistent after every single step, so
                # an error part-way never leaves a handler that destroy_*
                # would try to remove from the node a second time.
                node.remove_waitable(handler)
                entity.event_handlers = [h for h in entity.event_handlers if h is not handler]
                handler.destroy()
                dropped += 1
    except Exception as exc:  # noqa: BLE001
        log.warn(f'QoS event handler drop stopped after {dropped}: {exc!r}')
    return dropped
