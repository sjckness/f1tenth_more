"""
mpc_corr's reference corridor, separated from the node.

geometry  pure functions: the corridor build for every goal shape, the
          lookahead target and its obstacle displacement, the frame helpers.
          No rclpy and no node state; inputs are plain values and a config
          dataclass, results (including log lines) come back as data.
manager   CorridorManager: the per-move state -- the frozen map-frame anchor
          and its reprojection, the cached corridor and its rebuild period,
          and invalidation when a new goal arrives.
markers   construction of the /mpc/corridor_markers MarkerArray and of the
          corridor_debug.jsonl snapshot record.

The shape of the centreline and walls themselves (the S-curve heading ramp
and the widening funnel) lives one level further down, in
f1tenth_params.corridor_geometry, shared with the test-campaign plotter.
"""
