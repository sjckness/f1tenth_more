"""
drop_qos_event_handlers never raises and never half-applies.

Pure Python: fake nodes and a fake handler class stand in for rclpy, so this
runs the same on any distro (no rclpy.init, no node). The real-rclpy check is
TestDropQosEventHandlers in test_bt_tick_and_event_estop.py (run on the Orin).
"""

from unittest.mock import MagicMock

import pytest

from f1tenth_behavior import qos_events


class _Handler:
    def __init__(self):
        self.destroyed = False

    def destroy(self):
        self.destroyed = True


class _Entity:
    def __init__(self, n):
        self.event_handlers = [_Handler() for _ in range(n)]


class _Node:
    """The Humble rclpy Node surface drop_qos_event_handlers uses."""

    def __init__(self, pubs, subs):
        self._pubs, self._subs = pubs, subs
        self._waitables = [h for e in pubs + subs for h in e.event_handlers]
        self.logger = MagicMock()

    @property
    def publishers(self):
        yield from self._pubs

    @property
    def subscriptions(self):
        yield from self._subs

    @property
    def waitables(self):
        yield from self._waitables

    def remove_waitable(self, waitable):
        self._waitables.remove(waitable)

    def get_logger(self):
        return self.logger


@pytest.fixture(autouse=True)
def _fake_handler_class(monkeypatch):
    monkeypatch.setattr(qos_events, 'EventHandler', _Handler)


def test_drops_every_handler_on_a_humble_shaped_node():
    pubs, subs = [_Entity(1), _Entity(2)], [_Entity(1)]
    node = _Node(pubs, subs)
    handlers = [h for e in pubs + subs for h in e.event_handlers]
    assert qos_events.drop_qos_event_handlers(node) == 4
    assert list(node.waitables) == []
    assert all(h.destroyed for h in handlers)
    assert all(e.event_handlers == [] for e in pubs + subs)
    node.logger.warn.assert_not_called()


class _Without:
    """A node whose rclpy surface lacks one attribute (another rclpy)."""

    def __init__(self, node, missing):
        self._node, self._missing = node, missing

    def __getattr__(self, name):
        if name == self._missing:
            raise AttributeError(name)
        return getattr(self._node, name)


@pytest.mark.parametrize('missing', ['publishers', 'subscriptions', 'waitables',
                                     'remove_waitable'])
def test_a_missing_node_attribute_is_a_no_op_with_one_warn(missing):
    inner = _Node([_Entity(1)], [_Entity(1)])
    before = list(inner._waitables)
    assert qos_events.drop_qos_event_handlers(_Without(inner, missing)) == 0
    assert inner._waitables == before
    assert inner.logger.warn.call_count == 1


def test_event_handlers_not_a_list_is_a_no_op_with_one_warn():
    entity = _Entity(1)
    node = _Node([entity], [])
    entity.event_handlers = tuple(entity.event_handlers)
    assert qos_events.drop_qos_event_handlers(node) == 0
    assert len(list(node.waitables)) == 1
    assert node.logger.warn.call_count == 1


def test_no_handler_class_at_all_is_a_no_op_with_one_warn(monkeypatch):
    monkeypatch.setattr(qos_events, 'EventHandler', None)
    node = _Node([_Entity(1)], [])
    assert qos_events.drop_qos_event_handlers(node) == 0
    assert node.logger.warn.call_count == 1


def test_an_error_part_way_warns_and_leaves_node_and_entity_consistent():
    pubs = [_Entity(2)]
    node = _Node(pubs, [])
    second = pubs[0].event_handlers[1]

    def boom():
        raise RuntimeError('destroy failed')
    second.destroy = boom
    assert qos_events.drop_qos_event_handlers(node) == 1
    assert node.logger.warn.call_count == 1
    # Whatever is still on the entity is still on the node, and vice versa,
    # so rclpy's destroy_publisher can remove each of them exactly once.
    assert set(map(id, pubs[0].event_handlers)) == set(map(id, node.waitables))
