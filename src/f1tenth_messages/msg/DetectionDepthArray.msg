# Per-detection depth for detection_3d_node, depth_source:=ingest.
#
# Published by f1tenth_depth_ingest_rs once per set its synchronizer pairs
# (the Python message_filters ApproximateTimeSynchronizer policy detection_3d_node
# runs, same queue size and slop, over detections, depth and -- use_mask_depth --
# masks): the synchronized Detection2DArray unchanged, the depth frame's header,
# the intrinsics of the latest camera_info at the sync, and one DetectionDepth per
# detection. The depth and mask images stay in the ingest node. Design:
# output/rust/depth_ingest_design.md.

# depth_status
uint8 DEPTH_OK = 0
# No camera_info received yet: nothing was sampled (drop_no_camera_info).
uint8 DEPTH_NO_CAMERA_INFO = 1
# The depth image is not a decodable 32FC1 image (drop_cv_bridge); see error.
uint8 DEPTH_DECODE_FAILED = 2

# mask_status
# No mask in this set (use_mask_depth false: a 2-way sync).
uint8 MASK_NONE = 0
# The zero-size "no instances this frame" marker.
uint8 MASK_EMPTY = 1
uint8 MASK_OK = 2
# A mask of another resolution, nearest-neighbour resized to the depth's.
uint8 MASK_RESIZED = 3
# Not a decodable mono8 image (every detection takes the box region); see error.
uint8 MASK_DECODE_FAILED = 4

# The depth frame's header (stamp, optical frame_id), as the synced depth image carried it.
std_msgs/Header header
vision_msgs/Detection2DArray detections
uint8 depth_status
uint8 mask_status
# Why decoding failed (depth or mask), empty otherwise.
string error
uint32 depth_width
uint32 depth_height
# k[0], k[4], k[2], k[5] of the latest camera_info (not DEPTH_NO_CAMERA_INFO).
float64 fx
float64 fy
float64 cx
float64 cy
# The synced mask image's header stamp, size and encoding (not MASK_NONE).
builtin_interfaces/Time mask_stamp
uint32 mask_width
uint32 mask_height
string mask_encoding
# One per detections.detections, same order (empty unless DEPTH_OK).
DetectionDepth[] depths
# The ingest node's settings (the consumer checks them against its own).
bool use_mask_depth
float64 center_fraction
bool enable_scene_labels
float64 scene_labels_max_age_s
int32 scene_min_other_px
float64 sync_slop
int32 sync_queue_size
# Inputs received since the ingest node started (the consumer's yield report).
uint64 det2d_in
uint64 depth_in
uint64 mask_in
uint64 synced
