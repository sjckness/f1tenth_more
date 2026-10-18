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


def test_every_profile_records_no_viz_copy_and_adds_to_the_defaults():
    # cpu-cleanup R2 block 4a: recording profiles only ADD topics.
    from f1tenth_logger.mission_logger_node import _PROFILE_TOPICS, profile_topics
    assert profile_topics('default') == []
    assert profile_topics('no-such-profile') == []
    for name in _PROFILE_TOPICS:
        topics = _DEFAULT_TOPICS + profile_topics(name)
        kept, dropped = recordable_topics(topics)
        assert dropped == [], name


def test_the_campaign_profile_has_what_the_wall_replay_needs():
    from f1tenth_logger.mission_logger_node import profile_topics
    topics = profile_topics('campaign')
    for t in ('/perception/scene_labels/classes', '/camera/image_raw_5hz',
              '/zed2/zed_node/depth/depth_registered_5hz', '/perception/wall_observations'):
        assert t in topics, t
