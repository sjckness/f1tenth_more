# Obstacles v2 — decisions

D1. The derived cell view is the latest /scan, not /slam/map. The work order's
persistence (0.5 s, consecutive updates) and decay (1 s once the rays pass
through) need per-scan evidence; /slam/map updates every 5 s and only while
the car moves (slam_toolbox map_update_interval, minimum travel gates) and keeps
every artefact it ever integrated -- the reason A1 moved the default off it.
The grid is local (2 * (range_m + 1) square, 0.05 m), map frame, rebuilt every
cycle; /slam/map is never read or written.

D2. Object ownership frees the cells of CONFIRMED RECOGNISED tracks only. An
unknown track's own cells must stay islands, or the track could never be
observed again; its cluster reaches it through association instead.

D3. A person's two legs are one object: islands are bridged across
cluster_gap_m (0.20 m) before 8-connected labelling (a binary dilation by half
the gap), and the component's cells are the original islands inside it.

D4. Unknown lifecycle is the LiDAR's; recognised lifecycle stays the camera's.
update_tracks_batch no longer ticks misses on unknown tracks and
update_unknown_batch never touches a recognised track's streaks, so neither
sensor's silence kills the other's tracks. A tentative unknown is dropped on
its first missed cycle (the sightings must be consecutive); a confirmed one
goes decay_s after its last hit once the rays have passed through its area,
or unobserved_decay_s (3 s) if the LiDAR could not see it at all.
