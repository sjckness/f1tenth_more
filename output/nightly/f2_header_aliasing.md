# F2 — the header-aliasing bug: how much did it falsify?

`d355bbe` fixed it; `c44b503` is the code before it. Both were run over
`corridor10m_20261015T171753` from a temporary worktree
(`git worktree add ... c44b503`, f1tenth_perception and corridor_perception
rebuilt there and overlaid on the main install). The branch was not touched.

## The bug, in one paragraph

`point_cloud2.create_cloud_xyz32(header, ...)` and the StereoColumns publish both
ALIASED the depth frame's `Header` and then set its `frame_id` to the base frame --
the cloud's points and the profile's heights really are in `base_link`. The object
they rewrote is the **depth message's own header**, so everything later in that
same `depth_callback` read the depth image's `frame_id` as `base_link`. It only
happened on cycles where the scene-mask tick ran, which is why two thirds of the
frames are affected and one third is clean.

## What it did NOT touch: the column cross-check

Bit-identical, pre-fix vs post-fix, over 146 replayed cycles:

| range [m] | columns with both | disagree | rate | resid med [m] | pre == post |
|---|---|---|---|---|---|
| 0–1 | 51 | 22 | 0.4314 | 0.041 | yes |
| 1–2 | 10 570 | 7 771 | 0.7352 | 0.2099 | yes |
| 2–3 | 6 192 | 5 914 | 0.9551 | 0.5848 | yes |
| 3–5 | 6 825 | 4 595 | 0.6733 | 0.4061 | yes |
| 5–8 | 7 147 | 3 957 | 0.5537 | 0.6671 | yes |
| 8+ | 6 037 | 4 689 | 0.7767 | 2.5499 | yes |

Why: `_publish_stereo_columns` copies `camera_frame` from `header.frame_id`
**before** rewriting it, and `_scene_mask_tick` looks its own transform up at the
top of the tick. So the columns, the class mask and the WALL cloud were all
computed with the right extrinsic. The recorded run agrees: all 145
`/perception/stereo_columns` messages carry `camera_frame =
zed2_left_camera_optical_frame`. **No column tolerance was touched.**

## What it DID touch, live, on the car

Everything in `depth_callback` after the tick. Straight out of both recordings,
no replay needed:

| | raw bag | archive bag (same window) |
|---|---|---|
| `/perception/debug/front_wall` messages | 241 | 71 |
| of those, `frame_id: base_link` + `front_offset_m: 0.405` | **163 (67.6 %)** | **42 (59 %)** |
| correct (`zed2_left_camera_optical_frame`, 0.1756) | 78 | 29 |

The camera's optical frame sits at x = 0.2299 m in `base_link`, so the correct
bumper offset is 0.405 − 0.2299 = **0.1756 m** and the contaminated one is
**0.405 m**. `front_distance = depth − offset`, so on those frames
**`/perception/front_distance` was 0.2294 m SHORT** -- it reported every surface
nearer than it was. Measured on the recording: median 2.851 m on the 163
contaminated frames against 3.080 m on the 78 clean ones, a 0.229 m step that
matches the offset error exactly.

Contaminated consumers, by code path, all inside `front_clearance_node`:

| consumer | what it got | consequence |
|---|---|---|
| `_front_offset(msg.header.frame_id)` | pose of `base_link` in `base_link` = 0 | **`/perception/front_distance` 0.2294 m short** on 67.6 % of frames; the mission wall guard reads this topic |
| `_ground_roi_mask(depth, msg.header.frame_id, ...)` | identity rotation | the ROI ground-plane mask was fitted as if the camera were axis-aligned with `base_link` |
| `_label_scene(depth, msg.header, ...)` | identity rotation | the old scene_labels grid, `stereo_wall_distance` and `/perception/stereo_wall` on those frames. Live impact on this run was negligible: 133 messages, almost all with a null distance -- the virtual wall took the tracker path |
| `/perception/scene_labels` (Image) | `frame_id: base_link` on all 143 | a debug image in the wrong frame for any viewer |
| `_publish_too_close(..., msg.header.frame_id)` | `base_link` | a report field only |
| the class mask (E2, after `a205fb7`) | identity camera transform | 18 245 samples, **0 inside the image**, coverage 0.000 on every track of the whole run -- and indistinguishable from a wall the camera cannot see |

In the REPLAY the same path fails harder: the replay feeds only `/tf_static`, where
there is no `base_link -> base_link` edge, so `_front_offset` returns None and the
pre-fix node published the **no-reading sentinel −1.0 on all 155 frames** where the
fixed one publishes 1.50 m. Live the node had a full TF buffer, so it got 0.405
instead of None -- a wrong number rather than no number, which is the worse of the two.

## Verdict

The bug falsified **nothing** in the column cross-check and therefore nothing in
the tolerance work that rests on it. It falsified `/perception/front_distance` by
0.2294 m on two thirds of the frames of every run since the StereoColumns publish
was introduced, and it made E2 silently inert. Fixed in `d355bbe`, with a
regression test on the published `frame_id`
(`test_front_clearance_node.py::TestTheClassMaskAsData`).
