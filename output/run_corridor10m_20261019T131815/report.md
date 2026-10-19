# corridor10m_20261019T131815 — offline analysis

Bag `~/bags/corridor10m_20261019T131815` (326.8 MiB, 102.30 s, 113394 msgs, 155 topics).
Analysed offline on the Orin, `campaign-final` @ 868ec8a9, stack stopped throughout,
`ROS_SUPER_CLIENT=TRUE`. Nothing was left running and no default was changed.

## SUMMARY

1. **The bag is NOT complete: it has no camera at all.** Depth, `/perception/stereo_columns`
   and `scene_labels/classes` are 0 messages for the whole run; the gray images are ~20 frames
   over 102 s. The live node *did* have stereo early on, so this is a RECORDING gap, not a
   sensor failure — but it makes the run unusable for every camera question, D7 included.
2. `/scan` is also thin: 33.1 Hz against a nominal 40, with 26 **header-stamp** gaps over 100 ms
   (max 260 ms, 2.9 s of 49 s missing). Header-side, so this is real perception loss, not a
   recorder hole. `/perception/walls` likewise 6.9 Hz with header gaps to 1.05 s.
3. What the car did: one movement, mission `llm_2b356aac445f` "approach the wall", single move
   `move_0_straight`, 19.41 m of path in 49.5 s at ≤0.48 m/s, stopped on
   `stop_condition:front_clear`. Front contract **97.1 % WALL / 2.9 % CLEAR / 0 % NO_DATA**,
   front distance 20.61 m → 1.61 m. Accuracy against the true strip is good:
   median |err| **0.015 m**, p90 **0.094 m**, max 0.182 m. **0 phantom episodes while moving**
   by the strip test (the tool's own looser definition finds 1, non-moving-adjacent).
4. **D7 recommendation: keep V1 (change nothing).** V3 buys back the right wall
   (43.7 % → 84.3 % of cycles) but loses the first-priority metric badly — short tracks
   43 → 110 (2.6×) — and costs **+9.6 ms** tracker p50, 3× over the +3 ms budget.
5. **V2 is undecidable on this bag and V4 does not exist yet.** V2 ≡ V3 here (its camera gate
   has no camera to gate). V4's replay output is byte-identical to V1's because
   `wall_min_span_birth_relaxed_m` and the `wall_birth_relax_*` keys **are not implemented** —
   they appear only in the untracked TDD stub `test_conditional_birth.py`, which is red
   (collection `TypeError`).
6. Top problems: the missing camera recording; a 8.5 s dead-reckoned front wall at the start of
   the move (rel 45.78–54.24 s); and the right wall absent 66 % of the time live.

---

## 0. Bag integrity and window

**Window** = first movement −2 s → end +2 s (|v| > 0.1 m/s on `/odometry/filtered`, gaps < 3 s
merged). Only **one** movement episode exists, so the window is that one ±2 s:

| | abs | rel to bag start |
|---|---|---|
| window start | 1792408738.84 | **+42.69 s** |
| window end | 1792408788.32 | **+92.17 s** |
| duration | | **49.48 s** |

Everything below is inside this window.

### Rates, and which buckets have holes

25 buckets of 2 s. A bucket is a hole when it is below 90 % of nominal.

| topic | in window | Hz | nominal | hole buckets | worst bucket | verdict |
|---|---|---|---|---|---|---|
| `/scan` | 1639 | 33.1 | 40 | 19/25 (76 %) | 43 of 80 | **perception loss** (header-side) |
| `/odometry/filtered` | 840 | 17.0 | 20 | 17/25 (68 %) | 20 of 40 | thin but usable |
| `/perception/virtual_wall/debug` | 352 | 7.1 | 10 | 17/25 (68 %) | 0 | tracks `/scan`; last bucket empty (car stopped) |
| `/perception/walls` | 343 | 6.9 | 10 | 18/25 (72 %) | 3 of 20 | **perception loss** (header gaps to 1.05 s) |
| `/perception/lidar_front_wall` | 1327 | 26.8 | 40 | 23/25 (92 %) | 33 of 80 | follows `/scan` |
| `/zed2/.../depth_registered/*` | **0** | 0 | 5 | 25/25 (100 %) | 0 | **not recorded at all** |
| `/perception/scene_labels/classes` | **0** | 0 | 5 | 25/25 (100 %) | 0 | **not recorded at all** |
| `/perception/stereo_columns` | **0** | 0 | 5 | 25/25 (100 %) | 0 | **not recorded at all** |

**Recorder hole vs perception loss.** I separated the two by comparing receipt timestamps with
header stamps. The recorder wrote in bursts (`/scan` receipt dt p50 0.0010 s, p99 0.54 s, lag
p50 0.330 s) — that is a writer artefact and must not be read as perception. But the **header**
stamps carry the gaps too: `/scan` header dt p50 0.0299 s (33.4 Hz), **48 gaps > 80 ms**,
26 > 100 ms, max 260 ms. The node genuinely did not see those scans. Same for
`/perception/walls`: header dt p50 0.120 s, 23 gaps > 250 ms, max 1.05 s.

Largest `/scan` header gaps (rel s): 219 ms @ 46.71, 180 ms @ 64.13, 200 ms @ 64.66,
200 ms @ 76.40, 170 ms @ 83.51, 200 ms @ 85.86, **260 ms @ 89.36**, 200 ms @ 91.12.

**The camera is absent, and it is a recording gap, not a sensor one.** The first
`/perception/virtual_wall/debug` payload of the bag (rel +12.08 s) carries
`"stereo_m": 2.1798, "stereo_age_s": 0.2127, "stereo_weight": 0.7519` — the live
`front_clearance_node` was producing stereo columns and the tracker was weighting them. None of
that was written to the bag. Inside the window the camera had already stopped contributing:
**0 of 352 cycles have a non-null `stereo_m`** (323 have `lidar_m`), and across the whole bag
only 8 recorded cycles used a stereo profile at all, with 12 observations rejected.

## 1. What the car did LIVE (recorded topics only, no replay)

Mission `llm_2b356aac445f`, prompt "approach the wall" (LLM latency 23.4 s), loaded +28.71 s,
started +43.79 s, finished +89.42 s, `COMPLETE` +98.45 s. One move `move_0_straight`,
`stop_reason = stop_condition:front_clear…`. Path **19.41 m**, |v| max **0.48 m/s**,
mean while moving 0.39 m/s.

### Front contract

| state | s | % of contract time |
|---|---|---|
| WALL | 35.9 | **97.1 %** |
| CLEAR | 1.1 | 2.9 % |
| NO_DATA | 0.0 | **0.0 %** |

Two CLEAR episodes, both at the very start of the move: rel 43.20–44.00 s (0.80 s) and
45.18–45.58 s (0.40 s).

**Error vs the true strip** (`distance_m` − `strip_range_m`, 340 of 349 WALL rows have a strip):

| median \|err\| | p90 \|err\| | max \|err\| | signed median |
|---|---|---|---|
| **0.0154 m** | **0.0938 m** | 0.1820 m | −0.0050 m |

Front distance ran 20.61 m → **1.61 m**. The replay tool's own end-wall metric (strip < 3 m)
agrees: 33 cycles, median +0.001 m, |err| p95 0.022 m, max 0.084 m, **0 miss cycles**.

**Where the WALL came from** (s of WALL time):

| source | s | provenance |
|---|---|---|
| `lidar` | 35.4 | measured |
| `lidar_dr` | 9.5 | **dead_reckoned** |
| `none` | 3.6 | none |

21 % of WALL time was **dead-reckoned**, i.e. the node was propagating a wall it could not
currently fit. The stretches: **rel 45.78–54.24 s (8.46 s)** and 55.94–56.55 s (0.61 s). It had
no lidar fit whatsoever at rel 42.75–45.68 s (2.93 s) and 54.39–55.70 s (1.31 s).

**Phantoms.** By the strip test (WALL, `distance_m` < 8 m, and either no strip or |err| > 1 m):
**0 episodes**. The replay tool's looser definition (WALL episode whose median error < −0.5 m,
empty strip counting as −∞) finds 12 recorded episodes but **11 of 12 are before the car moves**
and the 12th (rel 32.23–33.20 s) is outside this window. No rejection reasons fired inside the
window (`gap_rejected`, `merge_gap_refused`, `stereo_near_rejected`, `short_obs`,
`negative_killed`, `objects_behind_wall` all 0; `rejections[]` empty).

### Confirmed walls per role

343 `/perception/walls` cycles in the window, 48.6 s of coverage:

| role | % of window with a confirmed wall | published instances | source |
|---|---|---|---|
| front | 89.9 % | 625 | lidar 100 % |
| left | 89.5 % | 1063 | lidar 100 % |
| right | **66.2 %** | 283 | lidar 100 % |

(The replay tool's per-cycle figures over the whole bag are left 94.0 %, front 89.0 %,
right **33.6 %** — the window is the kinder number for the right wall.) **Every** published
track was `source = lidar`; 0 stereo, 0 both. Right wall absent for ≥1 s three times:
rel 42.69–48.42 (5.73 s), 54.90–59.31 (4.41 s), 90.15–91.48 (1.33 s).

### Live tracks

| | |
|---|---|
| distinct published ids (window) | **18** |
| distinct ids (whole bag, tool) | 19, 19 births, 11.1 births/min, **0 role switches** |
| lifetime p50 / max | 6.87 s / 45.41 s |
| lifetime < 1 s | **2** |
| max confirmed age | 125.2 s |
| ids that ever served as FRONT | 103, 128 |

### Live timings

| quantity | p50 | p95 | max | n |
|---|---|---|---|---|
| `tracker_ms` (live) | **31.04** | 54.78 | 75.86 | 362 |
| MPC `solve_dt_sec` | **27.00** | 53.73 | 84.62 | 310 (0 failures) |
| `rect_ms` | — | — | — | **0 samples** (no depth) |
| scene_labels | — | — | — | **0** (`scene_labels/stats` 0 msgs; only 158 `health`) |

`tracker_decimation = 4` on every cycle and **`tracker_skipped` summed to 0** — decimation 1/4
is confirmed and nothing was dropped by it. Note the tracker at 31 ms p50 against a ~30 ms scan
period: it is at the edge of its budget, which is consistent with the `/scan` and
`/perception/walls` header gaps above.

### Does a tentative track ever reach the contract, `/perception/walls`, or the semantic layer?

**No.** Of 1037 published track instances in the window, 890 are `state=1` (confirmed) and
147 are `state=2` (coasting); **zero are `state=0` (tentative)**. The three filters:

- [multi_wall_tracker.py:1819](../../src/f1tenth_perception/f1tenth_perception/multi_wall_tracker.py#L1819) —
  `published()` starts from `live = [t for t in self.tracks if t.ever_confirmed]`.
- [lidar_front_wall_node.py:1477](../../src/f1tenth_perception/f1tenth_perception/lidar_front_wall_node.py#L1477) —
  the node publishes only `self.wall_tracker.published(car)`.
- [lidar_front_wall_node.py:1522-1524](../../src/f1tenth_perception/f1tenth_perception/lidar_front_wall_node.py#L1522-L1524) —
  the FRONT contract additionally requires `t.ever_confirmed and not t.contradicted`.

And `state` can never be 0 for an `ever_confirmed` track:
[multi_wall_tracker.py:1536-1540](../../src/f1tenth_perception/f1tenth_perception/multi_wall_tracker.py#L1536-L1540)
sets CONFIRMED/COASTING when `ever_confirmed` and TENTATIVE only when it is not. So `state=0`
is exactly "never confirmed", and that set is filtered out before anything downstream.
The semantic layer saw nothing either: `/costmap/semantic_tracks` has **0** messages in the
window (2 in the whole bag).

### CPU

**Not run.** `output/cpu_test/` holds only the 2026-10-16 runs (`20261016T1023*`–`103455`,
`live`); nothing from today, so there is no table to put against Friday's 719 %.

## 2. Replay V1–V4

Run per variant as its own process, sequentially, `--stereo rec --no-video` for the numbers and
a second pass with video for V1 and V3. The replayed cycle count is 664 for every variant
(730 recorded tracker steps, 66 without their scan, 50 step gaps over 1.5× the 0.110 s median).

### Is V1 ≈ live?

**On the contract, yes. On the right wall, no.**

- Contract state identical on **649/664 cycles (97.7 %)** — inside the 5 % bar. The 15
  differences are 8 WALL→CLEAR, 4 WALL→NO_DATA, 3 CLEAR→NO_DATA. The tool checked all 8
  WALL→CLEAR cycles and found **0** had a confirmed replay track that `front_track()` would have
  returned but for `max_published=4`: the divergence is the publish cap plus tracker history this
  bag does not contain, not the code.
- `virtual_wall` value median |diff| 0.0021 m, p95 0.0314 m. `front_d_ray` median 0.0033 m,
  p95 0.0277 m. Observations: 5414 of 5417 matched by (normal, d).
- **Right-wall coverage is NOT within 5 %: recorded 33.6 % vs V1 replay 43.7 %, +10.1 points.**
  Left 94.0 → 93.5 and front 89.0 → 86.1 are fine. Two reasons, both structural: the car's
  oldest track at its first `/perception/walls` was **83.3 s old** — a head start no cold replay
  has — and 10 observations that were `SOURCE_BOTH` live arrive as `lidar` in the replay because
  the camera half is not in the bag. **So the replay comparison below is sound for the LiDAR
  birth-span question and must not be read as a statement about the camera.**

### Variants, per-variant table (replay pass `rec`, 664 cycles)

| metric (priority order) | V1 = HEAD | V2 | V3 | V4 |
|---|---|---|---|---|
| **tracks < 1 s** | **43** | ≡ V3 | **110** (2.6×) | **43** (= V1) |
| track instances | 78 | ≡ V3 | 170 | 78 |
| distinct ids / births | 19 / 19 | ≡ V3 | 24 / 24 | 19 / 19 |
| role switches | 0 | ≡ V3 | 0 | 0 |
| **phantom episodes** (tool) | 11 | ≡ V3 | 11 | 11 |
| phantom, moving | 1 | ≡ V3 | 1 | 1 |
| near-car walls `rho_bl`<0.5 m, all live | 1 id / 7 cyc | ≡ V3 | **4 ids / 131 cyc** | 1 id / 7 cyc |
| **side coverage, right** | 43.7 % | ≡ V3 | **84.3 %** | 43.7 % |
| side coverage, left | 93.5 % | ≡ V3 | 93.5 % | 93.5 % |
| side coverage, front | 86.1 % | ≡ V3 | 86.4 % | 86.1 % |
| contract WALL / CLEAR / NO_DATA [s] | 79.02 / 9.47 / 0.97 | ≡ V3 | 79.00 / 9.49 / 0.97 | 79.02 / 9.47 / 0.97 |
| NO_DATA % moving | 0.0 | ≡ V3 | 0.0 | 0.0 |
| end-wall err median / p95 / max [m] | 0.002 / 0.024 / 0.027 | ≡ V3 | 0.002 / 0.024 / 0.027 | 0.002 / 0.024 / 0.027 |
| all-WALL err p05 / p95 / **max abs** [m] | −0.148 / 0.016 / **0.182** | ≡ V3 | −0.178 / 0.025 / **0.245** | −0.148 / 0.016 / 0.182 |
| **tracker p50 / p95 [ms]** | **15.75 / 24.29** | ≡ V3 | **25.47 / 35.28** | 15.65 / 22.40 |
| Δ tracker p50 vs V1 | — | ≡ V3 | **+9.72 ms** | −0.10 ms |
| scan callback p50 / p95 [ms] | 25.79 / 35.85 | ≡ V3 | 36.69 / 49.27 | 25.08 / 33.48 |

The video pass reproduces this independently: V1 right 43.7 %, tracker p50 15.80; V3 right
84.3 %, tracker p50 25.35 (+9.55 ms); 43 vs 110 short tracks.

### Why V2 is not its own column

V2 = `camera_seg_min_ray_angle_deg: 0.0` + `wall_min_span_birth_m: 0.5`. The first key gates
camera observations, and this bag has **no camera input** — `/perception/stereo_columns` 0,
depth 0 — so no camera observation ever reaches the tracker and the key cannot change anything.
V2's only live half is the birth span, which is exactly V3. The repo's own survey reaches the
same verdict independently: `tools/wall_variant_videos.py --survey-only` drops this bag with
*"no camera input: neither recorded stereo columns nor depth, so no camera observation ever
reaches the tracker and the camera gates cannot differ between variants"* — 2 of 306 recordings
on this box are usable and this is not one of them.

### Why V4 is byte-identical to V1

**The V4 feature is not implemented.** `wall_min_span_birth_relaxed_m` and every
`wall_birth_relax_*` key in `tools/wall_variants/v4_conditional_birth.yaml` appear **nowhere** in
`multi_wall_tracker.py` or `lidar_front_wall_node.py` (grep count 0 in both). They exist only in
the untracked TDD stub `src/f1tenth_perception/test/test_conditional_birth.py`, which does not
even collect:

```
test_conditional_birth.py:66: in <module>
    P = mwt.TrackerParams(min_span_birth_relaxed_m=FLOOR)
E   TypeError: TrackerParams.__init__() got an unexpected keyword argument
    'min_span_birth_relaxed_m'
```

So the V4 override file is a no-op: the node keeps `wall_min_span_birth_m = 1.0` and reproduces
V1 exactly, down to the identical phantom list and error quantiles. **V4's numbers above are
V1's numbers under a different directory name and carry no information about V4.**

### Videos

| what | path |
|---|---|
| V1 camera | `output/run_corridor10m_20261019T131815/v1_vid/corridor10m_20261019T131815.camera.mp4` |
| V1 top-down | `output/run_corridor10m_20261019T131815/v1_vid/corridor10m_20261019T131815.topdown.mp4` |
| V3 camera | `output/run_corridor10m_20261019T131815/v3_vid/corridor10m_20261019T131815.camera.mp4` |
| V3 top-down | `output/run_corridor10m_20261019T131815/v3_vid/corridor10m_20261019T131815.topdown.mp4` |
| V1 ‖ V3 camera | `output/run_corridor10m_20261019T131815/camera_v1_v3.mp4` (3840×882, 47.6 s) |
| V1 ‖ V3 top-down | `output/run_corridor10m_20261019T131815/topdown_v1_v3.mp4` (2560×1034, 90.1 s) |

The camera videos are near-empty by construction — the bag holds 6 `/camera/image_annotated`
frames and no depth, so there is nothing to draw on. The top-down pair is the one to watch.

## 3. Verdict

### D7: recommend **V1 — change nothing**

Against the stated priority order:

1. **tracks < 1 s and phantoms** — V1 **43**, V3 **110**. V3 is 2.6× worse on the
   first-priority metric, and it pays for the extra tracks twice more: near-car walls
   (`rho_bl` < 0.5 m) go 1 id / 7 cycles → **4 ids / 131 cycles**, and the all-WALL error
   max |err| goes 0.182 m → **0.245 m**. Phantom episode counts tie at 11 (all but one before
   the car moves), so the tie-break inside priority 1 goes to V1 on short tracks.
2. **side coverage** — V3 wins, and wins big: right **43.7 % → 84.3 %**. Left and front
   unchanged. This is the one thing V3 is for, and it delivers.
3. **timing within +3 ms of V1** — V3 is **+9.7 ms** on tracker p50 (15.75 → 25.47) and
   +11.0 ms on p95, confirmed at +9.55 ms by the second pass. That is 3.2× the allowance, on a
   box whose live tracker already sits at 31 ms p50 against a ~30 ms scan period and is already
   dropping scans (§0). V3 fails priority 3 outright.

V3 loses priority 1 and priority 3 and wins only priority 2, so **V1 stands**. V4 cannot be
recommended because it does not exist, and V2 cannot be evaluated on this bag at all. No
default was changed, as instructed.

**The exact diff, if the right wall is later judged to outweigh both** (do not apply now) —
`src/f1tenth_params/config/stack_params.yaml`, key at line 2912:

```diff
 lidar_front_wall_wall_min_span_birth_m:
-  default: 1.0
+  default: 0.5
```

That one line *is* V3 (0.5 is already `wall_min_span_birth_update_m`, so birth becomes equal to
maintenance and the span gate loses its hysteresis). It would need `--param-file` validation on
a bag that actually has depth before it goes anywhere near the car.

**What this run cannot decide.** D7 is a question about the camera radial gate. This bag has no
camera, so this run contributes **nothing** to D7's camera half; the 84.3 % V3 figure is a
LiDAR-only number. The open D7 decision still needs a corridor bag recorded **with depth**.

### Top 3 problems, with timestamps

1. **The recording lost the whole camera half, silently** (rel +42.69 → +92.17, i.e. all of it).
   The live node had stereo at +12.08 s (`stereo_m 2.1798`, `stereo_weight 0.75`), yet depth,
   `stereo_columns` and `scene_labels/classes` are 0 messages for 102 s. In the top-down video
   every wall drawn is a LiDAR wall and the camera pane is blank. This is the single most
   expensive problem here: it cost today's run its ability to answer D7, and it would recur on
   the next one. **Check `record_zed_depth` before the next corridor run.**
2. **An 8.5 s dead-reckoned front wall at the start of the move, rel 45.78–54.24 s**, with no
   lidar fit at all at rel 42.75–45.68 s (2.93 s) and 54.39–55.70 s (1.31 s). For 21 % of WALL
   time the car was driving at a wall it was propagating rather than measuring. The contract
   never said NO_DATA during it — it reported WALL with `source: lidar_dr`. Visible in the
   top-down as the front wall holding still while the scan returns thin out. The wall was
   16–20 m away at the time, which is why it cost nothing today.
3. **The right wall is missing two thirds of the time** — live 66.2 % coverage in the window
   (33.6 % over the whole bag) against 89.5 % left, in a corridor that is geometrically
   symmetric. Absent for ≥1 s at rel **42.69–48.42 (5.73 s)**, **54.90–59.31 (4.41 s)** and
   90.15–91.48 (1.33 s). V3 shows this is the 1.0 m birth span, not the geometry — but the fix
   costs 2.6× the short tracks and 9.7 ms, so the problem stands open.

Secondary, worth noting: `/scan` ran at **33 Hz, not 40**, losing 2.9 s of 49 s in 26
header-stamp gaps up to 260 ms (worst at rel 89.36 s), and the live tracker sits at 31 ms p50
against a ~30 ms period. The rate loss and the tracker cost are plausibly the same fact.

## Files

```
output/run_corridor10m_20261019T131815/
  report.md                 this file
  v1/ v3/ v4/               --no-video replays (summary.md, summary.json, replay.md, *.csv)
  v1_vid/ v3_vid/           same, with camera.mp4 + topdown.mp4
  camera_v1_v3.mp4          V1 ‖ V3 side by side
  topdown_v1_v3.mp4         V1 ‖ V3 side by side
  v1.log v3.log v4.log v1_vid.log v3_vid.log
```

The `.mp4` and `.csv` files are **local only**: `.gitignore` covers CSV output and no `.mp4` is
tracked anywhere in this repo (`git ls-files '*.mp4'` → 0). Only the Markdown, JSON, `info.txt`
and logs above are committed.
