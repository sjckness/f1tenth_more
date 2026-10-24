# The depth pixels front_clearance_node reads from one depth frame, depth_source:=ingest.
#
# Published by f1tenth_depth_ingest_rs once per depth frame. front_clearance's
# statistics depend on its own object-confidence EMA, scene labels and floor fit,
# so they stay in that node; this carries the pixels they are computed from, bit
# for bit, and on a scene tick the whole frame. Design:
# output/rust/depth_ingest_design.md.

uint8 STATUS_OK = 0
# Not a decodable 32FC1 image (the raw node's cv_bridge error); see error.
uint8 STATUS_DECODE_FAILED = 1

# The depth frame's own header, unchanged.
std_msgs/Header header
uint8 status
string error
# The depth frame's size [px].
uint32 width
uint32 height
# front_clearance's ROI bounds as it computes them (y0 = max(0, h//2 -
# roi_shift_up_px - roi_half_height_px), ...), and depth[y0:y1, x0:x1] (Python
# slice semantics), row-major.
int32 roi_y0
int32 roi_y1
int32 roi_x0
int32 roi_x1
float32[] roi
# The floor-fit sample, depth[y0:height:ground_stride, x0:x1:ground_stride]
# (np.mgrid), row-major; empty and ground_stride 0 with ground removal off.
uint32 ground_stride
float32[] ground
# front_clearance's two 5 Hz schedules (scene_mask_rate_hz, scene_rate_hz),
# kept here on this node's clock: true on the frame a tick is due.
bool scene_mask_tick
bool scene_labels_tick
# On a tick, the whole depth image exactly as received; empty otherwise.
sensor_msgs/Image frame
# The ingest node's settings (the consumer checks them against its own).
bool scene_mask_enabled
float64 scene_mask_rate_hz
bool scene_labels_enabled
float64 scene_rate_hz
# Depth frames received since the ingest node started.
uint64 depth_in
