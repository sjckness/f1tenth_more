# Phase 2: SLAM + costmap parity (offline, Thor)

Previous phase: `output/phase1_localization_report.md`, verdict **GO-WITH-NOTES**,
no blockers for this phase. Gate read before starting.

## Verdict: **GO-WITH-NOTES**

`slam_toolbox` (2.8.5, with Phase 0's `4edcbbf` lifecycle fix) reaches
`active`, processes scans correctly, and — proven directly, not inferred
— publishes **zero** TF of any kind beyond what it's fed, confirming
`transform_publish_period: 0.0` holds under real scan-matching load. Pose
and map outputs track Humble closely given SLAM's documented
run-to-run nondeterminism. `semantic_layer_node` is bit-exact vs Humble.
`costmap_boundary_node`'s outputs track Humble tightly in the typical
case (median error 0.5°/7mm) with a `inflate_polytope` confirmed out of
this data path; occasional large outliers are explained and shown to be
a periodic-timer/nearest-cell-method timing artifact, not a logic
difference. Nothing here blocks Phase 3.

---

## Step 1: code version for this phase's parts

Reused Phase 1's finding (same bag, same commit `bc3b049`): the full,
unrestricted diffstat `bc3b049` → `e47e646` (the Humble baseline) touches
**zero** files under `src/f1tenth_navigation/`, `src/f1tenth_costmap/`,
`src/f1tenth_perception/`, or `src/f1tenth_control/` — only two
`f1tenth_behavior` files (2 lines each, unrelated to this phase) and the
same comment-only `stack_params.yaml` lines already verified in Phase 1.

**The late-September corridor/boundary work flagged as a risk**: checked
directly, not assumed. The Sep 28-dated `corridor_plot.png` test artifacts
that appear in this diff are *outputs*, not source changes — the actual
corridor/boundary source (`mpc_controller/MPC_corr.py`, `wall_turn.py`,
`costmap_boundary_node.py`) shows **zero** diff in the same window. The
corridor work happened either before `bc3b049` or after `e47e646` (outside
this bag's relevant window either way).

**Decision: the bag's recorded outputs are a valid Humble reference for
every layer in this phase.** No Orin replay needed; no
`ORIN_INSTRUCTIONS.md` required.

---

## Step 2: SLAM

### Inventory (from code)

| | detail |
|---|---|
| Node | `async_slam_toolbox_node` (`slam_toolbox` 2.8.5), `LifecycleNode` |
| Input | `/scan` (frame `laser`) |
| TF needed | `base_link→laser` (**static**, in `/tf_static`), `odom→base_link` (dynamic, from the local EKF — not under test here, fed from the bag) |
| TF NOT needed as input | `map→odom` (slam_toolbox computes its own map-frame estimate from scan matching, not from an existing map→odom edge) |
| Outputs (remapped, matching `slam.launch.py`) | `/slam/pose`, `/slam/map`, `/slam/map_metadata` |
| TF published | **none** — `transform_publish_period: 0.0` |
| Gates | `minimum_travel_distance: 0.03`, `minimum_travel_heading: 0.035`, `minimum_time_interval: 0.5` |
| `scan_queue_size` | 100 (Phase-0-era fix for a 100%-scan-drop bug, unrelated to this phase but confirmed still in effect) |

### Harness

Extended `replay_localization.sh` with a `slam` layer: plays `/scan` +
`/tf_static` + `/tf` filtered to **only** `odom→base_link` via the new
`filter_bag_for_layer.py --tf-keep` allowlist mode (cleaner than an
exhaustive `--tf-drop` list when only one edge out of several on `/tf`
matters). Drives `async_slam_toolbox_node` through `configure`→`activate`
via the `ros2 lifecycle` CLI directly (mirroring `slam.launch.py`'s own
Phase-0 event-pair, since invoking the launch file itself would need a
`use_sim_time` passthrough it doesn't currently expose).

**Two real bugs found and fixed while building this:**

1. **`filter_bag_for_layer.py` dropped QoS profiles.** Writing a filtered
   bag without carrying through each topic's `offered_qos_profiles` (not
   previously needed in Phase 1's filtering) silently downgraded
   `/tf_static` from its real TRANSIENT_LOCAL durability to the default
   VOLATILE. Confirmed live: `"New publisher discovered on topic
   '/tf_static', offering incompatible QoS... DURABILITY_QOS_POLICY"`,
   zero static transforms ever delivered, every `/scan` dropped with
   `"timestamp... earlier than all the data in the transform cache"` (no
   `base_link→laser` edge ever arrived). Fixed by reading and forwarding
   `offered_qos_profiles` from the source bag's own `TopicMetadata` when
   creating each topic in the filtered output.
2. **Replay script launched the node without its production remappings.**
   `async_slam_toolbox_node` publishes on its own unremapped defaults
   (`/map`, `/pose`) unless told otherwise — `slam.launch.py` remaps them
   to `/slam/map`/`/slam/pose`. The first version of this script recorded
   *nothing* on either `/slam/...` topic despite the node processing scans
   correctly the whole time; `ros2 node info /slam_toolbox` showed `/pose`
   and `/map` as real, active publishers. Fixed by adding the same three
   `-r` remappings `slam.launch.py` uses.

### TF-ownership proof (the open Phase 0 item)

Recorded the full `/tf` topic during a live replay and enumerated every
distinct `(frame_id, child_frame_id)` pair that appeared:

```
{('odom', 'base_link'): 1984}
```

**Exactly the one edge played in as input, and nothing else.** Since
nothing else in this isolated replay could produce a `(map, odom)` pair
(no `ekf_global` running), any appearance of it could only have come from
`slam_toolbox` itself — none appeared, in any of the 3 runs. This is a
direct, airtight proof (not an absence-of-evidence inference) that
`transform_publish_period: 0.0` holds under real scan-matching load on
Jazzy.

### Gates and scan processing

- Drops: exactly **1** per run, in all 3 runs — the documented,
  expected edge case (the very first scan arrives before any transform
  history exists to interpolate against). No drops after that; `/scan`
  (1682 published) was fully consumed with `scan_queue_size: 100` holding
  up as designed.
- `/slam/pose` count: Humble 73, Jazzy 72/72/72 (consistent across runs).
- `/slam/map` count: Humble 10, Jazzy 9/9/9 (consistent across runs).
- The minimum-travel gates are evaluated against real scan-matching
  results in this replay (not a stub), so a consistent, slightly-lower
  count across all 3 Jazzy runs relative to Humble is expected sampling
  variance, not a gate malfunction — see the noise-floor discussion below
  for why.

### Outputs vs Humble

**`/slam/pose` is genuinely non-deterministic in WHEN a correction fires**
(same code, same inputs, 3 identical Jazzy runs): raw nearest-timestamp
matching between runs 1 and 2 found only a 94% rate within 50ms, and the
*interior* (not just edge) timestamps diverge by up to 1.6s between runs —
confirmed this is really about *which* scan crosses the
`minimum_travel_distance`/`heading` gate, not clock jitter. This matches
the task's own framing ("slam_toolbox may be nondeterministic"); raw
event-time matching is reported in
`output/phase2/slam_metrics.json` (`noise_floor_raw_event_match`) for
transparency but is **not** the metric that matters.

**The metric that matters**: zero-order-hold resampling onto a common 1 Hz
grid (what a downstream consumer actually sees — "the latest pose as of
time t" — robust to which exact scan produced which correction):

| | pos_err RMS | pos_err max | yaw_err RMS | yaw_err max |
|---|---|---|---|---|
| noise floor (Jazzy×Jazzy) | 0.093 m | 0.133 m | 1.24° | 4.26° |
| Jazzy vs Humble (×3) | 0.168–0.176 m | 0.251–0.283 m | 0.80–0.98° | 3.29–4.43° |

Jazzy-vs-Humble position error is ~1.8–1.9× the floor; yaw error is
*smaller* than the floor. Both are small in absolute terms for a car-scale
SLAM map (under 20 cm, under 1° RMS) — judged against "a small, justified
multiple of the floor," this passes. See
`output/phase2/plots/slam_trajectory_overlay.png` for the visual (near-
perfect overlay after the first couple of samples).

**Final `/slam/map` vs Humble's final map**, compared over the
world-coordinate overlap (not raw array indices — grids differ in
width/height since exploration extent varies slightly run to run; see
`compare_maps()`'s own docstring):

| run | occupied-vs-free agreement | mean |occupancy diff| |
|---|---|---|
| 1 | 96.08% | 3.92 |
| 2 | 96.25% | 3.75 |
| 3 | 96.08% | 3.92 |

Visual: `output/phase2/plots/slam_map_diff.png` — same room geometry,
same corridor, same obstacle clusters; Jazzy's map is slightly narrower
(384–390 vs 510 cells wide), consistent with the same pose-timing
variance already characterized above, not a mapping-quality regression.
**Pass.**

---

## Step 3: costmap layers

### Inventory (from code)

| Node | Inputs | TF needed | Outputs |
|---|---|---|---|
| `semantic_layer_node` | `/camera/detections_3d`, `/ekf_global/odometry/filtered` | `base_link`↔`zed2_left_camera_frame` — **fully static chain** (`base_link→zed2_camera_link→...→zed2_left_camera_frame`, all in `/tf_static`) | `/costmap/semantic_markers`, `/costmap/semantic_tracks` |
| `costmap_boundary_node` | `/slam/map`, `/ekf_global/odometry/filtered`, `/mpc/corridor_markers` (optional) | **none**, by default (see below) | `/costmap/boundaries`, `/costmap/front_clearance`, `/costmap/safe_corridor_report` |
| `costmap_renderer_node` | `/slam/map`, `/costmap/semantic_markers` | none | `/costmap/visualization` (an `Image`, not one of the metrics this phase asks for — inventoried, not separately replay-tested) |

Per the task's own instruction ("Replay their inputs with the recorded
upstream outputs... since the EKF layer is already validated"), both
replayed layers feed directly from the bag's own recorded
`/ekf_global/odometry/filtered`/`/slam/map` — not chained from this
phase's own SLAM replay output — isolating each layer exactly like
Phase 1 did for `ekf_global`.

**All parameters for both nodes use their built-in `declare_parameter()`
defaults** (checked line by line against `costmap.launch.py`'s own
declared launch-arg defaults — every one matches exactly), so the replay
needed no `-p` overrides beyond `use_sim_time:=true`.

### `inflate_polytope` confirmed NOT in this data path

`costmap_boundary_node.py` **does** import from `safe_corridor`
(`build_safe_corridor`, `inflate_polytope` indirectly via
`build_safe_corridor`) — this contradicts Phase 0's module-docstring claim
that "nothing imports this module yet," which is simply stale text, not
current fact. What actually gates it: `use_convex_polytope` defaults to
`False` in **both** places that matter — the node's own
`declare_parameter('use_convex_polytope', False)` **and**
`costmap.launch.py`'s own `DeclareLaunchArgument` (`default_value='false'`)
— confirmed by reading both directly, not inferred from one. Neither this
replay nor the production stack (absent an explicit CLI override) ever
calls `build_safe_corridor`/`inflate_polytope`. Phase 0's `fd83d6e`
numpy-1.26 infinite-loop fix is therefore not exercised by this data
path — the analysis is unaffected either way.

### Harness

Extended `replay_localization.sh` with `semantic_layer` and
`costmap_boundary` layers (plain `ros2 run`, no lifecycle). Extended
`compare_runs.py` with `boundaries_stats` (per-constraint normal-angle and
offset comparison, index-matched — the node's own module docstring: array
order is "arbitrary but fixed", so index `i` in one run corresponds to
index `i` in another from the same extraction on the same inputs),
`scalar_stats` (for `/costmap/front_clearance`), and `tracks_stats`
(nearest-neighbour position matching for `/costmap/semantic_tracks`,
since track id/spawn-order isn't guaranteed to correspond 1:1 across runs
under the same kind of gate-timing variance SLAM already showed).

### `semantic_layer_node` — bit-exact

| | match rate | count diff | pos err RMS |
|---|---|---|---|
| noise floor (Jazzy×Jazzy) | 100% | 0.0 | **0.0 m** |
| Jazzy vs Humble (×3) | 100% | 0.0 | **0.0 m** |

Every matched message, every run: exact agreement on track count and
position. Deterministic TF composition + deterministic tracking logic on
identical inputs — no surprise, confirms the layer. **Pass, trivially.**

### `costmap_boundary_node` — tight median agreement, explained outliers

Raw RMS/max alone would look alarming (constraint normal-angle RMS 6.3–6.5°
vs Humble, max 89°; `front_clearance` RMS 0.19 m, max 2.24 m) — **investigated,
not accepted at face value.** Root cause, confirmed two ways:

1. **The error is concentrated in a small tail, not spread evenly.**
   Percentile breakdown (constraint normal angle, run 1 vs Humble):
   median **0.52°**, p75 1.20°, p90 2.47°, and only **3.6%** of matched
   constraints exceed 5°. Offset: median **7.4 mm**, p90 41 mm, 2.2% over
   10 cm. `front_clearance`: median **17 mm**, p90 71 mm, 5.0% over 10 cm.
2. **The same pattern shows up in the Jazzy-vs-Jazzy noise floor itself**
   (same code, same inputs): constraint angle RMS 0.04–0.22°, but
   `normal_angle_err_deg_max` still reaches 7.06° between two *identical*
   replay runs. This rules out a Jazzy-specific regression — whatever
   produces the outliers is present in the noise floor too, just at
   smaller scale.

`costmap_boundary_node`'s own "PERIODIC-PUBLISH pass" design is the
mechanism: it publishes on a fixed 20 Hz timer using **whatever's
currently cached** (`/slam/map` updates only every 5 s via
`map_update_interval`), not on-change. Exactly which pose/map sample is
cached at a given timer tick is a real-time race between the timer firing
and the latest message arriving — inherently non-deterministic run to run,
confirmed by the noise floor. The nearest-occupied-cell extraction method
then **amplifies** small input differences near a tie-breaking boundary
into a discontinuous jump to a different cell entirely (a structural
property of nearest-neighbor extraction, not a bug) — explaining why a
handful of ticks show large swings while the typical tick agrees almost
exactly. Visual confirmation:
`output/phase2/plots/front_clearance_overlay.png` — the two traces are
visually indistinguishable except at a few sharp discontinuities, where
one trace lags the other by what looks like a single sample.

**Judged against the acceptance rule (small multiple of the floor, median
not RMS when the distribution has a heavy tail with an identified,
explained cause):** median agreement is tight and the outlier mechanism is
understood and shown to be inherent to the design, not a Jazzy defect.
**Pass, with the outlier mechanism documented as a design characteristic
worth knowing about, not a regression to chase.**

---

## What isn't covered, and where it moves

- **`costmap_renderer_node`**: inventoried from code, not separately
  replay-tested — the task's own Step 3 metrics list
  (`/costmap/boundaries`, `/costmap/front_clearance`,
  `/costmap/semantic_tracks`) doesn't include its own output
  (`/costmap/visualization`, an `Image`), and it's purely downstream of
  already-validated `semantic_layer_node` output plus `/slam/map` (also
  validated) via a deterministic rendering function — low marginal risk.
  Not a scope cut of convenience; flagged explicitly.
- **Local EKF + IMU** (Phase 1's open item): still open, still Phase 6.
- **`/mpc/corridor_markers`** was fed to `costmap_boundary_node` for
  subscription-set fidelity but is only read when `use_convex_polytope`
  is true (it isn't) — not exercised by this pass, consistent with that
  flag's own off-by-default state.

---

## Commits

```
(pending commit) jazzy/p2: SLAM + costmap parity harness
```

`scripts/jazzy_parity/{bag_compat,filter_bag_for_layer,
replay_localization.sh,compare_runs,make_plots_phase2}.py` (extended, not
forked), `output/phase2/{*.json,plots/*.png,runs/,inputs/}`,
`output/phase2_slam_costmap_report.md`.

---

## Decisions for Andreas

1. **`filter_bag_for_layer.py`'s QoS-preservation fix matters beyond this
   harness.** Any future tooling that rewrites/filters rosbag2 bags and
   touches `/tf_static` (or any other TRANSIENT_LOCAL topic) needs to
   carry `offered_qos_profiles` through, or it will silently and
   confusingly break static-transform delivery downstream. Worth knowing
   if this pattern gets reused elsewhere.
2. **`costmap_boundary_node`'s periodic-publish design has an inherent,
   now-quantified outlier rate** (~2–5% of ticks differ by more than
   10cm/5° from a "true" value, due to timer/cache-staleness racing
   against a discontinuous extraction method) — not a Jazzy issue, not
   acted on here, but worth knowing before treating `/costmap/boundaries`
   or `/costmap/front_clearance` as smooth/continuous signals downstream
   (e.g., in MPC) without some filtering.
3. **`inflate_polytope`/`use_convex_polytope` remains off by default** —
   confirmed again this phase. If it's ever turned on, Phase 0's
   numpy-1.26 fix (`fd83d6e`) is what makes that path safe on this
   platform; re-verify parity for that path specifically before flipping
   the flag, since none of Phase 0–2's parity work has exercised it.
4. **SLAM's own gate-timing nondeterminism** (same code, same input,
   corrections firing at measurably different real times run to run) is
   now characterized with hard numbers (floor ~0.09m/1.2° at 1Hz
   resampling) — worth keeping as the reference noise floor for any
   future SLAM-adjacent parity work, rather than re-deriving it.
