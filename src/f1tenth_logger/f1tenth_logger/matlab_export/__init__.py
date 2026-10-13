"""Raw-data export of test-campaign runs into a MATLAB database.

One ``<db_root>/runs/<test_id>.mat`` per campaign test plus
``<db_root>/index/runs.csv``; read back with the ``+f1db`` package in
``tools/matlab``. See ``tools/matlab/README.md``.

    db_root     get_db_root(): where the database lives
    convert     Python/JSON values -> savemat-ready structs and cells
    campaign    a campaign test folder -> per-topic structs (always present)
    bag         an archive rosbag2 -> per-topic structs (where it covers)
    exporter    the ``export_matlab`` command: matching, writing, the index

Kept outside ``f1tenth_logger.test_campaign`` on purpose: it reads the
mission logger's run archive, which the test-campaign side must never touch
(test_test_campaign_isolation.py). It needs numpy, scipy and rosbags, never
rclpy.
"""

EXPORTER_VERSION = "1.0.0"
