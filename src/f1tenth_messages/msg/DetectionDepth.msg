# One detection's depth, as detection_3d_node samples it (depth_source:=ingest).
#
# Published inside DetectionDepthArray by f1tenth_depth_ingest_rs, one per
# entry of the synchronized Detection2DArray and in its order (entry i is
# detections[i], mask label i + 1). Every detection is sampled, also those
# below detection_3d_node's confidence_threshold: the consumer filters first,
# as before, and books its counters from `source`.
#
# depth_m is float(np.median(...)) of the float32 depth values the source
# names (output/rust/parity_depth_ingest.md: bit-identical to numpy), as a
# float32, which holds that value exactly.

# _median_depth: the centre box region (no mask in play this frame).
uint8 SOURCE_BOX = 0
# _mask_median_depth: the instance mask's valid pixels (scene labels off).
uint8 SOURCE_MASK = 1
# The in-mask pixels labelled OTHER by the scene labels nearest the depth stamp.
uint8 SOURCE_SCENE_OTHER = 2
# The mask's valid pixels: no labels within scene_labels_max_age_s, or fewer
# than scene_min_other_px OTHER pixels (det_scene_fallback).
uint8 SOURCE_SCENE_FALLBACK = 3
# No valid depth in the mask: the box region instead (det_drop_mask_fallback,
# with the warning).
uint8 SOURCE_MASK_EMPTY = 4
# Mask label i + 1 above 255: the box region instead (det_drop_mask_fallback).
uint8 SOURCE_LABEL_OVERFLOW = 5
# A NaN box centre or size: no depth. (The raw node's int() raises there;
# documented fail-safe difference.)
uint8 SOURCE_INVALID_BOX = 6

uint8 source
# False: no depth (an empty or degenerate region), the raw node's None.
bool valid
# The median [m]; NaN when not valid.
float32 depth_m
# Valid-depth (finite, > 0) pixels of the box region, for the box sources.
uint32 box_px
# Valid-depth pixels of the instance mask (mask sources and SOURCE_MASK_EMPTY).
uint32 mask_px
# Of those, the pixels labelled OTHER (scene sources).
uint32 other_px
