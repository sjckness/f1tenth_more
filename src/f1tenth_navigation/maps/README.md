# Map info
File:        track_bw.png
Size:        400×400 px
Resolution:  0.01 m/px
Real extent: 4.0×4.0 m
Origin:      (0.0, 0.0) = bottom-left corner of image

# TF chain
`localization_source` (f1tenth_params/config/stack_params.yaml) selects which of two
mutually-exclusive nodes owns `map → odom`. Default is `raw_odom`, not `ekf` -- keep
that in mind, this used to be the other way around.

    map → odom        raw_odom (DEFAULT): raw_odom_map_tf_node
                       (f1tenth_localization) mirrors /odom onto map -> odom
                       verbatim, unfiltered -- no IMU fusion, no correction.
                       ekf (opt-in, localization_source:=ekf): robot_localization
                       EKF (f1tenth_bringup/config/ekf.yaml, world_frame: map)
                       instead, fusing /odom + VESC IMU. Either way, if the owning
                       node isn't running, map is disconnected from odom/base_link
                       entirely.
    odom → base_link  static_transform_publisher (odom_to_base_link_tf,
                       f1tenth_navigation/launch/nav2.launch.py) -- a FIXED
                       identity transform (odom and base_link coincide at
                       startup); the map -> odom output above is what actually
                       carries the car's real-world pose, not this.
    base_link → laser static_transform_publisher
                       (f1tenth_description/launch/description.launch.py)
    base_link → zed2_camera_link static_transform_publisher
                       (f1tenth_perception/launch/camera.launch.py)
    base_link → imu   static_transform_publisher, identity
                       (f1tenth_hardware/launch/vesc.launch.py)

There is no static `map → odom` transform encoding a fixed car starting pose
anymore (the old map_server_launch_old.py / odom_to_tf_node pair that did this
is gone) -- map → odom is entirely owned by whichever node localization_source
selects, dynamic from the first measurement either way.

# Foxglove / VSCode setup
1. Connect to ws://localhost:8765 (or SSH tunnel: ssh -L 8765:localhost:8765 user@<jetson-ip>)
2. Add 3D panel → Fixed Frame: map
3. Add /map topic  → renders the occupancy grid
4. Add /tf topic   → shows base_link moving over the map
