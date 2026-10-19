# /costmap/semantic_objects: every CONFIRMED object of the semantic layer's
# object memory, TRACKED and MEMORY alike, published by semantic_layer_node
# every memory cycle (semantic_layer_memory_rate_hz).
#
# header.frame_id is the map frame. header.stamp is the cycle time at which
# the states, confidences and timers below were evaluated -- NOT a detection
# capture time (/costmap/semantic_tracks carries that).
std_msgs/Header header

# The lifecycle parameters in force, so a consumer or a log reader can
# interpret confidence and the timers without the yaml.
float32 fov_miss_remove_s
float32 memory_decay_s

SemanticObject[] objects
