# One object of f1tenth_costmap's semantic layer object memory
# (semantic_layer_node + object_memory.py), inside a SemanticObjectArray on
# /costmap/semantic_objects. Only CONFIRMED tracks appear.
#
# x/y are in the MAP frame (the array's header.frame_id). An odom-frame
# consumer re-expresses them through the CURRENT map -> odom edge every cycle
# (f1tenth_params.semantic_objects.objects_to_odom_circles) and never caches
# odom coordinates: map -> odom jumps on SLAM corrections, the object does not.

uint8 TRACKED = 0           # detected recently, or in view and undetected for at most fov_miss_remove_s
uint8 MEMORY = 1            # out of view or occluded: kept, confidence decays to 0 over memory_decay_s

uint32 id                   # track id, never reused; the same number as /costmap/semantic_tracks' Detection3D.id
string class_id             # the YOLO class label
float64 x                   # [m] map
float64 y                   # [m] map
float32 radius_m            # [m] footprint radius: fused measured width / 2, else the per-class nominal
float32 effective_radius_m  # [m] radius_m + memory growth (a person in MEMORY), else radius_m
uint8 state                 # TRACKED | MEMORY
float32 confidence          # [0, 1] 1 while TRACKED; in MEMORY 1 - time_in_memory_s / memory_decay_s
float32 score               # the detector score of the last associated detection
float32 time_in_memory_s    # [s] in MEMORY since it was last detected; 0 while TRACKED
float32 memory_remaining_s  # [s] memory_decay_s - time_in_memory_s: how long it may still be remembered
float32 time_since_seen_s   # [s] since the last associated detection
float32 fov_unseen_s        # [s] in view and undetected since the last detection; removed past fov_miss_remove_s
bool in_fov                 # visible to a sensor able to detect this class (YOLO classes: the stereo camera)
bool in_fov_lidar           # LiDAR: in the scan's own sector and range, no closer return on the bearing
bool in_fov_stereo          # camera: live, inside the frustum and depth window, not occluded
