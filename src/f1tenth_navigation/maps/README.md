# Map info
File:        track_bw.png
Size:        400×400 px
Resolution:  0.01 m/px
Real extent: 4.0×4.0 m
Origin:      (0.0, 0.0) = bottom-left corner of image

# Car starting pose (map frame)
x=2.9639 m, y=2.1302 m, yaw=1.8132 rad (CCW from +X axis)

The static `map → odom` transform encodes this starting pose, so when `/odom`
reads (0,0,0) at startup `base_link` appears at the pose above in the map frame.
(quaternion for yaw=1.8132: qz=0.787412, qw=0.616427)

# TF chain
    map → odom        static_transform_publisher (map_to_odom_tf, this package)
    odom → base_link  vesc_to_odom_node (f1tenth_stack, publish_tf:=true)
    base_link → laser static_transform_publisher (f1tenth_stack bringup)

`odom_to_tf_node` (node `odom_tf_broadcaster`) can also publish `odom → base_link`
from `/odom`, but it is **disabled by default** in `map_server_launch.py`
(`publish_odom_tf:=false`) because `vesc_to_odom_node` already owns that
transform. Enable it only if vesc_to_odom's TF is turned off, otherwise two
publishers will fight over `odom → base_link`.

# Foxglove / VSCode setup
1. Connect to ws://localhost:8765 (or SSH tunnel: ssh -L 8765:localhost:8765 user@<jetson-ip>)
2. Add 3D panel → Fixed Frame: map
3. Add /map topic  → renders the occupancy grid
4. Add /tf topic   → shows base_link moving over the map
