| parent -> child | /tf_static msgs | /tf msgs | xyz | q (x,y,z,w) |
|---|---|---|---|---|
| base_link -> base_front | 1 | 0 | 0.4050, 0.0000, 0.0000 | 0.00000, 0.00000, 0.00000, 1.00000 |
| base_link -> chassis | 1 | 0 | 0.0000, 0.0000, 0.0500 | 0.00000, 0.00000, 0.00000, 1.00000 |
| base_link -> imu | 1 | 0 | 0.0000, 0.0000, 0.0000 | 0.00000, 0.00000, 0.00000, 1.00000 |
| base_link -> laser | 1 | 0 | 0.3050, 0.0000, 0.1900 | 0.00000, 0.00000, 0.00000, 1.00000 |
| base_link -> zed2_camera_link | 1 | 0 | 0.2399, 0.0000, 0.2050 | 0.00000, -0.01817, 0.00000, 0.99983 |
| chassis -> chassis_inertia | 1 | 0 | 0.0000, 0.0000, 0.0000 | 0.00000, 0.00000, 0.00000, 1.00000 |
| chassis -> left_rear_wheel | 0 | 91 | 0.0000, 0.1000, 0.0000 | 0.70711, 0.00000, 0.00000, 0.70711 |
| chassis -> left_steering_hinge | 0 | 91 | 0.3050, 0.1000, 0.0000 | 0.00000, 0.70711, 0.00000, 0.70711 |
| chassis -> right_rear_wheel | 0 | 91 | 0.0000, -0.1000, 0.0000 | 0.70711, 0.00000, 0.00000, 0.70711 |
| chassis -> right_steering_hinge | 0 | 91 | 0.3050, -0.1000, 0.0000 | 0.00000, 0.70711, 0.00000, 0.70711 |
| left_steering_hinge -> left_front_wheel | 0 | 91 | 0.0000, 0.0000, 0.0000 | 0.70711, 0.00000, 0.00000, 0.70711 |
| map -> odom | 0 | 181 | 0.2486, 0.0785, 0.0000 | 0.00000, 0.00000, 0.16063, 0.98701 |
| odom -> base_link | 0 | 347 | -0.5065, 0.0886, 0.0000 | 0.00000, 0.00000, -0.17321, 0.98489 |
| right_steering_hinge -> right_front_wheel | 0 | 91 | 0.0000, 0.0000, 0.0000 | 0.70711, 0.00000, 0.00000, 0.70711 |
| zed2_camera_center -> zed2_left_camera_frame | 1 | 0 | -0.0100, 0.0600, 0.0000 | 0.00000, 0.00000, 0.00000, 1.00000 |
| zed2_camera_center -> zed2_right_camera_frame | 1 | 0 | -0.0100, -0.0600, 0.0000 | 0.00000, 0.00000, 0.00000, 1.00000 |
| zed2_camera_link -> zed2_camera_center | 1 | 0 | 0.0000, 0.0000, 0.0150 | 0.00000, 0.00000, 0.00000, 1.00000 |
| zed2_left_camera_frame -> zed2_left_camera_optical_frame | 1 | 0 | 0.0000, 0.0000, 0.0000 | 0.50000, -0.50000, 0.50000, -0.50000 |
| zed2_right_camera_frame -> zed2_right_camera_optical_frame | 1 | 0 | 0.0000, 0.0000, 0.0000 | 0.50000, -0.50000, 0.50000, -0.50000 |

/tf publishers: /ekf_filter_node, /ekf_global/ekf_global_filter_node, /isaac_f1tenth_sim, /robot_state_publisher, /slam_toolbox, /slam_toolbox, /zed2/zed2_state_publisher

/tf_static publishers: /isaac_f1tenth_sim, /robot_state_publisher, /static_baselink_to_imu, /static_baselink_to_laser, /static_baselink_to_zed2, /zed2/zed2_state_publisher

odom->base_link vs EKF output: {'tf_msgs': 347, 'ekf_msgs': 256, 'stamp_offset_s': 0.05, 'matched_stamps': 256, 'value_mismatch': 0, 'mismatch_samples': []}

checks: PASS
