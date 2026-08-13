# Live per-frame 2D obstacle list, published by f1tenth_perception's
# obstacle_projector_node on /perception/obstacles_2d. One detections message in,
# one obstacle list out -- no persistence/smoothing across frames.

std_msgs/Header header
Obstacle2D[] obstacles
