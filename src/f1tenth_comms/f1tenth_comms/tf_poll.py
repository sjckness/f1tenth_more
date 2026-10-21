"""
TF for a node that needs a pose once a second, without a callback per /tf message.

rclpy runs one executor iteration per message it receives. On linus that is
~0.35 ms of a core per /tf message whatever the callback does (a
TransformListener, an empty callback, raw bytes: 3.5-4 % of a core at 100
Hz, Jazzy and Humble alike), and the car's /tf carries 20-105 messages/s
(bags of 2026-10-16 and 2026-10-19).

PolledTf puts the /tf and /tf_static subscriptions on a side node that no
executor spins. The messages wait in the DDS reader (KEEP_LAST, depth 500)
and lookup() takes them all at once into a tf2 Buffer: 0.7-0.9 % of a core
at 100 Hz, measured the same way. The side node is named <node>_tf_poll.

It uses Subscription.handle.take_message(), the call rclpy's own executors
make. Should a future rclpy lose it, make_tf() falls back to an ordinary
TransformListener on the node itself.
"""

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

DEPTH = 500             # > one second of /tf at the car's highest rate
MAX_TAKE = 5000         # per lookup, so a flood cannot hold the caller


def _qos(static: bool) -> QoSProfile:
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=DEPTH,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=(DurabilityPolicy.TRANSIENT_LOCAL if static
                                  else DurabilityPolicy.VOLATILE))


class PolledTf:
    def __init__(self, node: Node):
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        # No global arguments: launch's `-r __node:=<name>` would rename this
        # node too, into a second node with the main node's name.
        self.side = Node(f'{node.get_name()}_tf_poll', context=node.context,
                         use_global_arguments=False, start_parameter_services=False,
                         enable_rosout=False)
        self.subs = [
            (self.side.create_subscription(TFMessage, '/tf', _ignore, _qos(False)), False),
            (self.side.create_subscription(TFMessage, '/tf_static', _ignore, _qos(True)),
             True),
        ]
        self.taken = 0
        for sub, _ in self.subs:          # fail here, not at the first lookup
            if not hasattr(sub.handle, 'take_message'):
                self.close()
                raise AttributeError('rclpy Subscription.handle.take_message is missing')

    def _drain(self) -> None:
        for sub, static in self.subs:
            for _ in range(MAX_TAKE):
                with sub.handle:
                    taken = sub.handle.take_message(sub.msg_type, sub.raw)
                if taken is None:
                    break
                for t in taken[0].transforms:
                    if static:
                        self.buffer.set_transform_static(t, 'tf_poll')
                    else:
                        self.buffer.set_transform(t, 'tf_poll')
                self.taken += 1

    def lookup(self, target: str, source: str):
        """Return the latest target <- source TransformStamped; raise TransformException."""
        self._drain()
        return self.buffer.lookup_transform(target, source, Time())

    def close(self) -> None:
        self.side.destroy_node()


class ListenerTf:
    """The ordinary way: a TransformListener on the node, a callback per /tf message."""

    def __init__(self, node: Node):
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)

    def lookup(self, target: str, source: str):
        return self.buffer.lookup_transform(target, source, Time())

    def close(self) -> None:
        self.listener.unregister()


def make_tf(node: Node, polled: bool = True):
    """Return a PolledTf, or a ListenerTf when polled is False or rclpy cannot poll."""
    if polled:
        try:
            return PolledTf(node)
        except AttributeError as exc:
            node.get_logger().warn(f'TF polling unavailable ({exc}): TransformListener')
    return ListenerTf(node)


def _ignore(_msg) -> None:
    pass
