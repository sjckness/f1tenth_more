# H4 — correcting or excluding the front_distance bias

`front_distance_bias_runs.csv` lists every affected run. The window is
`a92e133` (2026-10-15 15:17, the first scene-mask tick, whose cloud publish
aliased the depth message's header) to `d355bbe` (2026-10-16 02:31, which gave
both publishes their own header) -- 17 commits, about 11 hours. **Exactly one
archive run falls inside it.**

1. **Correct**: on a frame whose `/perception/debug/front_wall` payload carries
   `frame_id == "base_link"` (equivalently `front_offset_m == 0.405`), add
   **+0.2294 m** to `front_distance_m` and to `fused_m`. That is
   `0.405 - 0.1756`, the camera optical frame's own x in `base_link`; the
   published value was computed as `depth - 0.405` instead of `depth - 0.1756`.
2. **Or exclude**: drop those frames. On the one affected run that is 42 of 71
   frames (59.2 %), so correcting is worth more than excluding.
3. **Never correct a whole run by a constant.** The contamination is
   per-frame: only the depth frames on which the scene-mask tick had already
   run are wrong, and `frame_id` is the per-frame flag that says which. The
   same run holds both (median 2.6158 m contaminated against 2.8698 m clean).
4. `/perception/front_distance` itself carries no flag -- pair it by stamp with
   `/perception/debug/front_wall`, which does. `front_offset_m` is in that same
   payload, so `offset == 0.405` is the test to use in MATLAB.
5. Nothing else needs correcting: the column cross-check, `/perception/stereo_columns`
   and every wall-tracker quantity were computed before the mutation and are
   unaffected (F2, bit-identical over 146 cycles).
