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

D5. Wall ownership also takes THIS cycle's long LiDAR wall observations
(/perception/wall_observations, LiDAR-backed, >= unknown_wall_obs_min_span_m =
1.0 m, the tracker's own birth length), not only CONFIRMED /perception/walls
segments. /perception/walls carries at most 4 tracks and never 'other'; on
corridor10m_20261019T131815 the right-hand wall was absent from it most of the
time (the commonest message is front + left + left) and confirmed walls alone
owned 53 cells per frame against 171 islands. With observations: 87 vs 137.
The rule as written (confirmed walls only) is still available with
unknown_wall_obs_min_span_m := 0.

D6. The first-obstacle rule applies to unknown objects: a candidate whose line
of sight from the laser crosses a wall segment (D5's set) more than 0.10 m
before reaching it was seen THROUGH a gap in that wall and is dropped
(drop_behind_walls). corridor bag: 96 -> 86 confirmed unknown objects.

D7. A recognised track no detection has updated for
semantic_recognised_max_age_s (2.0 s) keeps its lifecycle (the work order:
"recognised objects keep today's decay") but no longer owns cells and is
never on the MPC's list. Today's lifecycle counts detection BATCHES, so it
never ticks when the camera stops -- which it did 0.7-1.0 s into both of
today's bags (ZED depth, output/avoidance/log.md A3) -- and the obstacle bag's
chair would otherwise sit in every solve forever: the A3 defect 2 reproduced
on the new path. Its LiDAR returns become an unknown object instead.

D8. One physical object split into two clusters on one cycle: the second
cluster refreshes the track it falls inside (gate max(merge_distance_m, sum of
radii)) and spawns nothing ('split'); an unmatched unknown track whose disk
still contains island cells is still hit (cells_hit) and stays alive. Before
these two rules dedupe_tracks folded 1200 tracks on the corridor bag; after,
96 -- see the report for what the remaining folds are.

D9. MPC timing (B4): the obstacle term's local linearisation evaluated
_d_front_at seven times per horizon step with a Python loop over obstacles,
so each obstacle cost ~0.6 ms of solve on linus and 8 obstacles were 1.5x
the p95 of one -- over the 1.3x budget. _d_front_batch does the same
arithmetic as one numpy operation (bit-identical on 200 random cases):
8 obstacles are now 1.09x of one, and one obstacle is faster than before.
mpc_obstacles_max = 8.

D10. No per-cell or per-obstacle constraint rows exist on the obstacle path:
an obstacle is a disk in the soft w_obs term (one min-distance term per
horizon step), exactly as before v2. Cell-derived rows exist only in
boundary_source:=costmap, unchanged.
