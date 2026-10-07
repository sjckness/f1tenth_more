"""mission_logger_node never records foxglove_bridge's /viz relay copies."""
from f1tenth_logger.mission_logger_node import _DEFAULT_TOPICS, recordable_topics


def test_the_default_topic_list_records_no_viz_copy():
    assert recordable_topics(_DEFAULT_TOPICS) == (list(_DEFAULT_TOPICS), [])


def test_a_topics_override_naming_viz_copies_loses_them():
    kept, dropped = recordable_topics([
        '/scan', '/viz/scan', '/viz_internal/camera/image_annotated',
        '/viz/camera/image_annotated/compressed', '/tf'])
    assert kept == ['/scan', '/tf']
    assert dropped == ['/viz/scan', '/viz_internal/camera/image_annotated',
                       '/viz/camera/image_annotated/compressed']


def test_only_the_viz_namespaces_are_dropped():
    names = ['/vizier', '/visualization', '/camera/image_annotated/viz', '/costmap/visualization']
    assert recordable_topics(names) == (names, [])
