"""The test-campaign logger: one folder per test, one row per test.

Kept apart from the mission logger (f1tenth_logger.mission_logger_node) on
purpose, and it shares nothing with it: another node name
(test_campaign_logger), another output folder (<f1tenth_more>/
first_test_campaing, never the run archive), no lock file, and no topic the
mission logger reads or records. It is never started by any bringup; see
TEST_CAMPAIGN.md at the package root for the operator's guide.

    robot_logger         TestLogger: one test folder, thread-safe, no ROS
    logger_node          the recorder node        (ros2 run ... test_campaign_logger)
    trigger              manual calibration runs  (ros2 run ... test_campaign_trigger)
    export_campaign_csv  campaign_results.csv     (ros2 run ... test_campaign_export)
    analyze_tests        report and plots         (ros2 run ... test_campaign_analyze)
    demo_simulated       the whole chain, simulated
"""
