146 topics, 62 nodes: SIM 11, STACK 112, MISSING 13, UNUSED_SIM 10

| class | topic | type | publishers | subscribers |
|---|---|---|---|---|
| MISSING | `/calibration_drive` | ackermann_msgs/msg/AckermannDriveStamped | - | /ackermann_mux |
| MISSING | `/cmd_vel_nav` | geometry_msgs/msg/Twist | - | /twist_to_ackermann_node |
| MISSING | `/ekf_global/set_pose` | geometry_msgs/msg/PoseWithCovarianceStamped | - | /ekf_global/ekf_global_filter_node |
| MISSING | `/imu` | sensor_msgs/msg/Imu | - | /mpc_corr |
| MISSING | `/mission/load_path` | std_msgs/msg/String | - | /behavior_executor_node |
| MISSING | `/model/virtual_robot/odometry` | nav_msgs/msg/Odometry | - | /mpc_corr |
| MISSING | `/perception/d_wall/psi_correction` | std_msgs/msg/Float32 | - | /mpc_corr |
| MISSING | `/sensors/core` | vesc_msgs/msg/VescStateStamped | - | /diagnostics_server_node |
| MISSING | `/set_pose` | geometry_msgs/msg/PoseWithCovarianceStamped | - | /ekf_filter_node |
| MISSING | `/slam_toolbox/feedback` | visualization_msgs/msg/InteractiveMarkerFeedback | - | /slam_toolbox |
| MISSING | `/teleop` | ackermann_msgs/msg/AckermannDriveStamped | - | /ackermann_mux |
| MISSING | `/test_campaign/logger_status` | std_msgs/msg/String | - | /mpc_corr |
| MISSING | `/zed2/joint_states` | sensor_msgs/msg/JointState | - | /zed2/zed2_state_publisher |
| SIM | `/camera/camera_info` | sensor_msgs/msg/CameraInfo | /isaac_f1tenth_sim | /yolo_detector_node |
| SIM | `/camera/image_raw` | sensor_msgs/msg/Image | /isaac_f1tenth_sim | /front_clearance_node, /image_raw_viz_throttle, /yolo_detector_node |
| SIM | `/clock` | rosgraph_msgs/msg/Clock | /isaac_f1tenth_sim | /ackermann_mux, /behavior_executor_node, /costmap_boundary_node, /costmap_renderer_node, /depth_throttle_5hz, /detection_3d_node, /diagnostics_server_node, /ekf_filter_node, /ekf_global/ekf_global_filter_node, /ekf_global/transform_listener_impl_5f3f80baf180, /foxglove_bridge, /front_clearance_node, /image_annotated_viz_throttle, /image_raw_viz_throttle, /joint_state_publisher, /lidar_front_wall_node, /lifecycle_manager_map, /map_server, /mission_logger_node, /mpc_corr, /obstacle_clearance_node, /obstacle_projector_node, /proximity_guard_node, /reset_manager, /robot_state_publisher, /semantic_layer_node, /slam_pose_relay_node, /slam_toolbox, /static_baselink_to_imu, /static_baselink_to_laser, /static_baselink_to_zed2, /swept_clearance_node, /system_observer_node, /transform_listener_impl_5b76007a84c0, /transform_listener_impl_5d9457ad36f0, /twist_to_ackermann_node, /viz_jpeg_node, /viz_relay_container, /viz_throttle_bt_tree_visualization, /viz_throttle_camera_image_annotated, /viz_throttle_costmap_front_clearance, /viz_throttle_costmap_visualization, /viz_throttle_diagnostics_system_status, /viz_throttle_drive, /viz_throttle_ekf_global_odometry_filtered, /viz_throttle_mpc_corridor_markers, /viz_throttle_perception_d_wall, /viz_throttle_perception_front_clearance, /viz_throttle_perception_front_distance, /viz_throttle_perception_lidar_front_wall_virtual, /viz_throttle_perception_obstacles_2d, /viz_throttle_scan, /viz_throttle_sensors_core, /viz_throttle_slam_pose, /viz_throttle_slam_pose_calibrated, /yolo_detector_node, /zed2/zed2_state_publisher |
| SIM | `/odom` | nav_msgs/msg/Odometry | /isaac_f1tenth_sim | /ekf_filter_node |
| SIM | `/parameter_events` | rcl_interfaces/msg/ParameterEvent | /ackermann_mux, /behavior_executor_node, /component_supervisor_node, /costmap_renderer_node, /depth_throttle_5hz, /detection_3d_node, /diagnostics_server_node, /ekf_filter_node, /ekf_global/ekf_global_filter_node, /foxglove_bridge, /front_clearance_node, /guard_vs_bt, /image_annotated_viz_throttle, /image_raw_viz_throttle, /isaac_f1tenth_sim, /joint_state_publisher, /launch_ros_989868, /launch_ros_989878, /lidar_front_wall_node, /lifecycle_manager_map, /map_server, /mission_logger_node, /mpc_corr, /obstacle_clearance_node, /obstacle_projector_node, /reset_manager, /robot_state_publisher, /semantic_layer_node, /slam_pose_relay_node, /slam_toolbox, /static_baselink_to_imu, /static_baselink_to_laser, /static_baselink_to_zed2, /swept_clearance_node, /system_observer_node, /twist_to_ackermann_node, /viz_jpeg_node, /viz_throttle_bt_tree_visualization, /viz_throttle_camera_image_annotated, /viz_throttle_costmap_front_clearance, /viz_throttle_costmap_visualization, /viz_throttle_diagnostics_system_status, /viz_throttle_drive, /viz_throttle_ekf_global_odometry_filtered, /viz_throttle_mpc_corridor_markers, /viz_throttle_perception_d_wall, /viz_throttle_perception_front_clearance, /viz_throttle_perception_front_distance, /viz_throttle_perception_lidar_front_wall_virtual, /viz_throttle_perception_obstacles_2d, /viz_throttle_scan, /viz_throttle_sensors_core, /viz_throttle_slam_pose, /viz_throttle_slam_pose_calibrated, /yolo_detector_node, /zed2/zed2_state_publisher | /ackermann_mux, /depth_throttle_5hz, /ekf_filter_node, /ekf_global/ekf_global_filter_node, /ekf_global/transform_listener_impl_5f3f80baf180, /foxglove_bridge, /image_annotated_viz_throttle, /image_raw_viz_throttle, /lifecycle_manager_map, /map_server, /robot_state_publisher, /slam_toolbox, /static_baselink_to_imu, /static_baselink_to_laser, /static_baselink_to_zed2, /transform_listener_impl_5b76007a84c0, /transform_listener_impl_5d9457ad36f0, /viz_relay_container, /viz_throttle_bt_tree_visualization, /viz_throttle_camera_image_annotated, /viz_throttle_costmap_front_clearance, /viz_throttle_costmap_visualization, /viz_throttle_diagnostics_system_status, /viz_throttle_drive, /viz_throttle_ekf_global_odometry_filtered, /viz_throttle_mpc_corridor_markers, /viz_throttle_perception_d_wall, /viz_throttle_perception_front_clearance, /viz_throttle_perception_front_distance, /viz_throttle_perception_lidar_front_wall_virtual, /viz_throttle_perception_obstacles_2d, /viz_throttle_scan, /viz_throttle_sensors_core, /viz_throttle_slam_pose, /viz_throttle_slam_pose_calibrated, /zed2/zed2_state_publisher |
| SIM | `/scan` | sensor_msgs/msg/LaserScan | /isaac_f1tenth_sim | /behavior_executor_node, /front_clearance_node, /guard_vs_bt, /lidar_front_wall_node, /obstacle_clearance_node, /proximity_guard_node, /slam_toolbox, /swept_clearance_node |
| SIM | `/sensors/imu/raw` | sensor_msgs/msg/Imu | /isaac_f1tenth_sim | /ekf_filter_node |
| SIM | `/tf` | tf2_msgs/msg/TFMessage | /ekf_filter_node, /ekf_global/ekf_global_filter_node, /isaac_f1tenth_sim, /robot_state_publisher, /slam_toolbox, /zed2/zed2_state_publisher | /detection_3d_node, /ekf_global/transform_listener_impl_5f3f80baf180, /mpc_corr, /obstacle_projector_node, /transform_listener_impl_5b76007a84c0, /transform_listener_impl_5d9457ad36f0 |
| SIM | `/tf_static` | tf2_msgs/msg/TFMessage | /isaac_f1tenth_sim, /robot_state_publisher, /static_baselink_to_imu, /static_baselink_to_laser, /static_baselink_to_zed2, /zed2/zed2_state_publisher | /behavior_executor_node, /detection_3d_node, /ekf_global/transform_listener_impl_5f3f80baf180, /front_clearance_node, /guard_vs_bt, /lidar_front_wall_node, /mpc_corr, /obstacle_clearance_node, /obstacle_projector_node, /proximity_guard_node, /semantic_layer_node, /swept_clearance_node, /transform_listener_impl_5b76007a84c0, /transform_listener_impl_5d9457ad36f0, /yolo_detector_node |
| SIM | `/zed2/zed_node/depth/camera_info` | sensor_msgs/msg/CameraInfo | /isaac_f1tenth_sim | /detection_3d_node, /front_clearance_node, /swept_clearance_node |
| SIM | `/zed2/zed_node/depth/depth_registered` | sensor_msgs/msg/Image | /isaac_f1tenth_sim | /depth_throttle_5hz, /detection_3d_node, /front_clearance_node, /swept_clearance_node |
| UNUSED_SIM | `/rosout` | rcl_interfaces/msg/Log | /ackermann_mux, /behavior_executor_node, /component_supervisor_node, /costmap_renderer_node, /depth_throttle_5hz, /detection_3d_node, /diagnostics_server_node, /ekf_filter_node, /ekf_global/ekf_global_filter_node, /ekf_global/transform_listener_impl_5f3f80baf180, /foxglove_bridge, /front_clearance_node, /guard_vs_bt, /image_annotated_viz_throttle, /image_raw_viz_throttle, /isaac_f1tenth_sim, /joint_state_publisher, /launch_ros_989868, /launch_ros_989878, /lidar_front_wall_node, /lifecycle_manager_map, /map_server, /mission_logger_node, /mpc_corr, /obstacle_clearance_node, /obstacle_projector_node, /reset_manager, /robot_state_publisher, /semantic_layer_node, /slam_pose_relay_node, /slam_toolbox, /static_baselink_to_imu, /static_baselink_to_laser, /static_baselink_to_zed2, /swept_clearance_node, /system_observer_node, /transform_listener_impl_5b76007a84c0, /transform_listener_impl_5d9457ad36f0, /twist_to_ackermann_node, /viz_jpeg_node, /viz_relay_container, /viz_throttle_bt_tree_visualization, /viz_throttle_camera_image_annotated, /viz_throttle_costmap_front_clearance, /viz_throttle_costmap_visualization, /viz_throttle_diagnostics_system_status, /viz_throttle_drive, /viz_throttle_ekf_global_odometry_filtered, /viz_throttle_mpc_corridor_markers, /viz_throttle_perception_d_wall, /viz_throttle_perception_front_clearance, /viz_throttle_perception_front_distance, /viz_throttle_perception_lidar_front_wall_virtual, /viz_throttle_perception_obstacles_2d, /viz_throttle_scan, /viz_throttle_sensors_core, /viz_throttle_slam_pose, /viz_throttle_slam_pose_calibrated, /yolo_detector_node, /zed2/zed2_state_publisher | - |
| UNUSED_SIM | `/sim/ground_truth/front_range` | sensor_msgs/msg/Range | /isaac_f1tenth_sim | - |
| UNUSED_SIM | `/sim/ground_truth/objects` | vision_msgs/msg/Detection3DArray | /isaac_f1tenth_sim | - |
| UNUSED_SIM | `/sim/ground_truth/odom` | nav_msgs/msg/Odometry | /isaac_f1tenth_sim | - |
| UNUSED_SIM | `/sim/ground_truth/person_pose` | geometry_msgs/msg/PoseStamped | /isaac_f1tenth_sim | - |
| UNUSED_SIM | `/sim/ground_truth/start_pose` | geometry_msgs/msg/PoseStamped | /isaac_f1tenth_sim | - |
| UNUSED_SIM | `/zed2/zed_node/left/camera_info` | sensor_msgs/msg/CameraInfo | /isaac_f1tenth_sim | - |
| UNUSED_SIM | `/zed2/zed_node/left/image_rect_color` | sensor_msgs/msg/Image | /isaac_f1tenth_sim | - |
| UNUSED_SIM | `/zed2/zed_node/right/camera_info` | sensor_msgs/msg/CameraInfo | /isaac_f1tenth_sim | - |
| UNUSED_SIM | `/zed2/zed_node/right/image_rect_color` | sensor_msgs/msg/Image | /isaac_f1tenth_sim | - |
| STACK | `/ackermann_drive` | ackermann_msgs/msg/AckermannDriveStamped | /ackermann_mux | /isaac_f1tenth_sim, /swept_clearance_node |
| STACK | `/behavior/tree_status` | f1tenth_messages/msg/BehaviorTreeStatus | /behavior_executor_node | /guard_vs_bt |
| STACK | `/bond` | bond/msg/Status | /lifecycle_manager_map, /map_server | /lifecycle_manager_map, /map_server |
| STACK | `/bt/tree_visualization` | sensor_msgs/msg/CompressedImage | /behavior_executor_node | - |
| STACK | `/calibration/in_progress` | std_msgs/msg/Bool | /diagnostics_server_node | - |
| STACK | `/camera/debug_image/compressed` | sensor_msgs/msg/CompressedImage | /yolo_detector_node | - |
| STACK | `/camera/detection_markers` | visualization_msgs/msg/MarkerArray | /detection_3d_node | - |
| STACK | `/camera/detection_masks` | sensor_msgs/msg/Image | /yolo_detector_node | /detection_3d_node, /front_clearance_node |
| STACK | `/camera/detections` | vision_msgs/msg/Detection2DArray | /yolo_detector_node | /behavior_executor_node, /detection_3d_node, /front_clearance_node |
| STACK | `/camera/detections_3d` | vision_msgs/msg/Detection3DArray | /detection_3d_node | /lidar_front_wall_node, /obstacle_projector_node, /semantic_layer_node, /yolo_detector_node |
| STACK | `/camera/image_annotated` | sensor_msgs/msg/Image | /yolo_detector_node | /image_annotated_viz_throttle |
| STACK | `/camera/image_annotated/viz` | sensor_msgs/msg/Image | /image_annotated_viz_throttle | - |
| STACK | `/camera/image_raw/viz` | sensor_msgs/msg/Image | /image_raw_viz_throttle | - |
| STACK | `/corridor` | std_msgs/msg/String | /mpc_corr | - |
| STACK | `/costmap/boundaries` | f1tenth_messages/msg/BoundaryConstraintArray | /costmap_boundary_node | /mpc_corr |
| STACK | `/costmap/front_clearance` | std_msgs/msg/Float32 | /costmap_boundary_node | /lidar_front_wall_node |
| STACK | `/costmap/safe_corridor_report` | std_msgs/msg/String | /costmap_boundary_node | - |
| STACK | `/costmap/semantic_markers` | visualization_msgs/msg/MarkerArray | /semantic_layer_node | /costmap_renderer_node |
| STACK | `/costmap/semantic_tracks` | vision_msgs/msg/Detection3DArray | /semantic_layer_node | /behavior_executor_node, /yolo_detector_node |
| STACK | `/costmap/visualization` | sensor_msgs/msg/Image | /costmap_renderer_node | - |
| STACK | `/costmap/wall_cell_ids` | std_msgs/msg/Int32MultiArray | /semantic_layer_node | - |
| STACK | `/costmap/wall_cells` | nav_msgs/msg/OccupancyGrid | /semantic_layer_node | - |
| STACK | `/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | /ackermann_mux, /behavior_executor_node, /ekf_filter_node, /ekf_global/ekf_global_filter_node, /lifecycle_manager_map | - |
| STACK | `/diagnostics/battery_status` | f1tenth_messages/msg/BatteryStatus | /diagnostics_server_node | /behavior_executor_node |
| STACK | `/diagnostics/system_status` | f1tenth_messages/msg/SystemStatus | /system_observer_node | /behavior_executor_node, /diagnostics_server_node |
| STACK | `/drive` | ackermann_msgs/msg/AckermannDriveStamped | /mpc_corr, /twist_to_ackermann_node | /ackermann_mux |
| STACK | `/ekf_global/odometry/filtered` | nav_msgs/msg/Odometry | /ekf_global/ekf_global_filter_node | /behavior_executor_node, /costmap_boundary_node, /semantic_layer_node |
| STACK | `/joint_states` | sensor_msgs/msg/JointState | /joint_state_publisher | /mpc_corr, /robot_state_publisher |
| STACK | `/map` | nav_msgs/msg/OccupancyGrid | /map_server | - |
| STACK | `/map_server/transition_event` | lifecycle_msgs/msg/TransitionEvent | /map_server | - |
| STACK | `/mission/move_outcome` | f1tenth_messages/msg/MoveOutcome | /behavior_executor_node | - |
| STACK | `/mission/status` | f1tenth_messages/msg/MissionStatus | /behavior_executor_node | /behavior_executor_node, /mission_logger_node, /yolo_detector_node |
| STACK | `/mpc/corridor_markers` | visualization_msgs/msg/MarkerArray | /mpc_corr | - |
| STACK | `/mpc/drive_clamp` | f1tenth_messages/msg/DriveClamp | /mpc_corr | - |
| STACK | `/mpc/goal_distance` | std_msgs/msg/Float32 | /behavior_executor_node | /behavior_executor_node, /mpc_corr |
| STACK | `/mpc/goal_drive` | f1tenth_messages/msg/DriveCommand | /behavior_executor_node | /mpc_corr |
| STACK | `/mpc/goal_object` | f1tenth_messages/msg/ObjectGoal | /behavior_executor_node | /mpc_corr |
| STACK | `/mpc/goal_object_end` | std_msgs/msg/String | /behavior_executor_node | /mpc_corr |
| STACK | `/mpc/goal_pose` | geometry_msgs/msg/PoseStamped | /behavior_executor_node | /mpc_corr |
| STACK | `/mpc/goal_reached` | std_msgs/msg/Bool | /mpc_corr | /behavior_executor_node |
| STACK | `/mpc/goal_turn` | f1tenth_messages/msg/TurnGoal | /behavior_executor_node | /mpc_corr |
| STACK | `/mpc/hold` | std_msgs/msg/Bool | /behavior_executor_node | /mpc_corr |
| STACK | `/mpc/min_obstacle_distance` | std_msgs/msg/Float32 | /mpc_corr | /behavior_executor_node |
| STACK | `/mpc/min_obstacle_distance_forward` | std_msgs/msg/Float32 | /mpc_corr | /behavior_executor_node |
| STACK | `/mpc/object_status` | f1tenth_messages/msg/ObjectApproachStatus | /mpc_corr | /behavior_executor_node |
| STACK | `/mpc/predicted_min_clearance` | std_msgs/msg/Float32 | /mpc_corr | - |
| STACK | `/mpc/solver_status` | f1tenth_messages/msg/MpcSolverStatus | /mpc_corr | - |
| STACK | `/mpc/status` | std_msgs/msg/String | /mpc_corr | - |
| STACK | `/obstacle_clearance` | std_msgs/msg/Float32 | /obstacle_clearance_node | - |
| STACK | `/odometry/filtered` | nav_msgs/msg/Odometry | /ekf_filter_node | /behavior_executor_node, /ekf_global/ekf_global_filter_node, /lidar_front_wall_node, /mpc_corr |
| STACK | `/perception/debug/front_wall` | std_msgs/msg/String | /front_clearance_node | /yolo_detector_node |
| STACK | `/perception/front_blocked` | std_msgs/msg/Bool | /front_clearance_node | - |
| STACK | `/perception/front_clearance` | std_msgs/msg/Float32 | /front_clearance_node | - |
| STACK | `/perception/front_distance` | std_msgs/msg/Float32 | /front_clearance_node | - |
| STACK | `/perception/front_wall` | std_msgs/msg/Bool | /front_clearance_node | - |
| STACK | `/perception/lidar_front_wall` | f1tenth_messages/msg/WallLineFit | /lidar_front_wall_node | /front_clearance_node |
| STACK | `/perception/lidar_front_wall_virtual` | f1tenth_messages/msg/WallEstimate | /lidar_front_wall_node | - |
| STACK | `/perception/obstacles_2d` | f1tenth_messages/msg/Obstacle2DArray | /obstacle_projector_node | /front_clearance_node, /mpc_corr |
| STACK | `/perception/scene_labels` | sensor_msgs/msg/Image | /front_clearance_node | /detection_3d_node, /yolo_detector_node |
| STACK | `/perception/scene_labels/classes` | sensor_msgs/msg/Image | /front_clearance_node | /lidar_front_wall_node |
| STACK | `/perception/scene_labels/depth_mm` | sensor_msgs/msg/Image | /front_clearance_node | /lidar_front_wall_node |
| STACK | `/perception/scene_labels/health` | std_msgs/msg/String | /front_clearance_node | - |
| STACK | `/perception/scene_labels/image` | sensor_msgs/msg/Image | /front_clearance_node | - |
| STACK | `/perception/scene_labels/stats` | std_msgs/msg/String | /front_clearance_node | - |
| STACK | `/perception/scene_labels_debug` | std_msgs/msg/String | /front_clearance_node | /yolo_detector_node |
| STACK | `/perception/stereo_columns` | f1tenth_messages/msg/StereoColumns | /front_clearance_node | /lidar_front_wall_node |
| STACK | `/perception/stereo_wall` | std_msgs/msg/String | /front_clearance_node | /lidar_front_wall_node |
| STACK | `/perception/swept_clearance` | std_msgs/msg/Float32 | /swept_clearance_node | - |
| STACK | `/perception/swept_clearance/camera` | std_msgs/msg/Float32 | /swept_clearance_node | - |
| STACK | `/perception/swept_clearance/lidar` | std_msgs/msg/Float32 | /swept_clearance_node | - |
| STACK | `/perception/swept_clearance/steering` | std_msgs/msg/Float32 | /swept_clearance_node | - |
| STACK | `/perception/unclassified_obstacles` | std_msgs/msg/String | /front_clearance_node | /yolo_detector_node |
| STACK | `/perception/virtual_wall` | std_msgs/msg/Float32 | /lidar_front_wall_node | /behavior_executor_node, /mpc_corr |
| STACK | `/perception/virtual_wall/debug` | std_msgs/msg/String | /lidar_front_wall_node | /yolo_detector_node |
| STACK | `/perception/wall_observations` | f1tenth_messages/msg/WallObservationArray | /lidar_front_wall_node | - |
| STACK | `/perception/wall_planes` | std_msgs/msg/String | /lidar_front_wall_node | /front_clearance_node |
| STACK | `/perception/wall_points` | sensor_msgs/msg/PointCloud2 | /front_clearance_node | /lidar_front_wall_node |
| STACK | `/perception/walls` | f1tenth_messages/msg/MultiWallTrackArray | /lidar_front_wall_node | /semantic_layer_node, /yolo_detector_node |
| STACK | `/perception/walls/markers` | visualization_msgs/msg/MarkerArray | /lidar_front_wall_node | - |
| STACK | `/proximity_guard/decision` | diagnostic_msgs/msg/DiagnosticStatus | /proximity_guard_node | /guard_vs_bt |
| STACK | `/robot_description` | std_msgs/msg/String | /robot_state_publisher | /joint_state_publisher |
| STACK | `/safety/event` | std_msgs/msg/String | /behavior_executor_node, /obstacle_clearance_node | - |
| STACK | `/safety_stop` | ackermann_msgs/msg/AckermannDriveStamped | /behavior_executor_node, /reset_manager | /ackermann_mux, /guard_vs_bt |
| STACK | `/slam/map` | nav_msgs/msg/OccupancyGrid | /slam_toolbox | /costmap_boundary_node, /costmap_renderer_node, /slam_toolbox |
| STACK | `/slam/map_metadata` | nav_msgs/msg/MapMetaData | /slam_toolbox | - |
| STACK | `/slam/pose` | geometry_msgs/msg/PoseWithCovarianceStamped | /slam_toolbox | /slam_pose_relay_node |
| STACK | `/slam/pose_calibrated` | geometry_msgs/msg/PoseWithCovarianceStamped | /slam_pose_relay_node | /ekf_global/ekf_global_filter_node |
| STACK | `/slam_toolbox/graph_visualization` | visualization_msgs/msg/MarkerArray | /slam_toolbox | - |
| STACK | `/slam_toolbox/scan_visualization` | sensor_msgs/msg/LaserScan | /slam_toolbox | - |
| STACK | `/slam_toolbox/transition_event` | lifecycle_msgs/msg/TransitionEvent | /slam_toolbox | /launch_ros_989868 |
| STACK | `/slam_toolbox/update` | visualization_msgs/msg/InteractiveMarkerUpdate | /slam_toolbox | - |
| STACK | `/test/mission_event` | std_msgs/msg/String | /behavior_executor_node | - |
| STACK | `/viz/bt/tree_visualization` | sensor_msgs/msg/CompressedImage | /viz_throttle_bt_tree_visualization | - |
| STACK | `/viz/camera/image_annotated/compressed` | sensor_msgs/msg/CompressedImage | /viz_jpeg_node | - |
| STACK | `/viz/costmap/front_clearance` | std_msgs/msg/Float32 | /viz_throttle_costmap_front_clearance | - |
| STACK | `/viz/costmap/visualization/compressed` | sensor_msgs/msg/CompressedImage | /viz_jpeg_node | - |
| STACK | `/viz/diagnostics/system_status` | f1tenth_messages/msg/SystemStatus | /viz_throttle_diagnostics_system_status | - |
| STACK | `/viz/drive` | ackermann_msgs/msg/AckermannDriveStamped | /viz_throttle_drive | - |
| STACK | `/viz/ekf_global/odometry/filtered` | nav_msgs/msg/Odometry | /viz_throttle_ekf_global_odometry_filtered | - |
| STACK | `/viz/mpc/corridor_markers` | visualization_msgs/msg/MarkerArray | /viz_throttle_mpc_corridor_markers | - |
| STACK | `/viz/perception/front_clearance` | std_msgs/msg/Float32 | /viz_throttle_perception_front_clearance | - |
| STACK | `/viz/perception/front_distance` | std_msgs/msg/Float32 | /viz_throttle_perception_front_distance | - |
| STACK | `/viz/perception/lidar_front_wall_virtual` | f1tenth_messages/msg/WallEstimate | /viz_throttle_perception_lidar_front_wall_virtual | - |
| STACK | `/viz/perception/obstacles_2d` | f1tenth_messages/msg/Obstacle2DArray | /viz_throttle_perception_obstacles_2d | - |
| STACK | `/viz/scan` | sensor_msgs/msg/LaserScan | /viz_throttle_scan | - |
| STACK | `/viz/slam/pose` | geometry_msgs/msg/PoseWithCovarianceStamped | /viz_throttle_slam_pose | - |
| STACK | `/viz/slam/pose_calibrated` | geometry_msgs/msg/PoseWithCovarianceStamped | /viz_throttle_slam_pose_calibrated | - |
| STACK | `/viz/walls` | visualization_msgs/msg/MarkerArray | /lidar_front_wall_node | - |
| STACK | `/viz_internal/camera/image_annotated` | sensor_msgs/msg/Image | /viz_throttle_camera_image_annotated | - |
| STACK | `/viz_internal/costmap/visualization` | sensor_msgs/msg/Image | /viz_throttle_costmap_visualization | - |
| STACK | `/zed2/zed2_description` | std_msgs/msg/String | /zed2/zed2_state_publisher | - |
| STACK | `/zed2/zed_node/depth/depth_registered_5hz` | sensor_msgs/msg/Image | /depth_throttle_5hz | - |
