# Live per-frame hard-boundary-constraint list. Variable length (0..N) --
# consumers must not assume a fixed count; an empty array is a valid,
# meaningful message ("nothing currently trustworthy enough to publish"),
# not a placeholder to be ignored. header.frame_id names the frame every
# entry's (normal, offset) is expressed in (base_link for both current
# publishers -- see BoundaryConstraint.msg's own note on consumer-side
# transforms).

std_msgs/Header header
BoundaryConstraint[] constraints
