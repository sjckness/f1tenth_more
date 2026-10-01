# Phase 1: Localization parity (offline, Thor)

Previous phase: `output/thor_phase0_report.md`, verdict **GO-WITH-NOTES**, no
blockers for this phase. Gate read before starting.

## Verdict: **GO-WITH-NOTES**

`slam_pose_relay_node` is bit-exact identical to the Humble recording
(0.0 error, every sample, all 3 runs). `ekf_global_filter_node`'s
steady-state behavior matches Humble within a small multiple of its own
run-to-run noise floor (position RMS 0.0044 m vs a 0.0012 m floor, yaw RMS
0.020° vs a 0.0025° floor — both still sub-centimeter/sub-tenth-of-a-degree
in absolute terms). One real, severe environment bug was found and fixed
along the way (robot_localization couldn't start at all). One metric
(pose0 rejection rate) could not be validated by the method requested —
documented honestly below, not faked. Local EKF + IMU parity is out of
scope for this phase (no IMU in any archived bag) and moves to Phase 6, as
instructed.

---

## Step 1: which code produced the primary bag?

**Candidate commit:** `bc3b049` ("object corridor: a maximum length, an
end on the target, and a run you can read back"), 2026-09-23 16:38:50
+0200 — the most recent commit across all branches as of the bag's start
time (2026-09-23 17:18:13 local), found via
`git log --all --until="2026-09-23 17:18:13" --format='%H %ad %s' -8`.
`main` and `scene-graph` were identical through this exact commit (their
merge-base with each other is `e47e646` itself, same as `jazzy`'s), so
there is no branch ambiguity — whichever branch name the Orin's checkout
pointed at, the code was the same.

**Diff `bc3b049` → `e47e646` (the Humble baseline), restricted to
everything this replay touches:**
- `src/f1tenth_bringup/config/ekf_global.yaml`, `ekf.yaml`,
  `src/f1tenth_localization/**`: **zero changes.**
- `src/f1tenth_params/config/stack_params.yaml`: **two changes, both
  comment-text only** (a doc-path reference, `mission_analysis_2026-09-01.md`
  → `docs/analysis/mission_analysis_2026-09-01.md`; `workspace_inventory.md`
  → `workspace_inventory.md (since removed)`) — no value changed.

**Decision: the bag's recorded outputs ARE a valid Humble reference** for
everything this phase tests.

**Stated uncertainty, per the gate instructions:** this is the most recent
*committed* code as of the bag's timestamp — whether the Orin's actual
working tree had uncommitted changes at that moment cannot be verified
from here. Given the diff above shows zero functional difference across a
whole week of intervening history (`bc3b049` to `e47e646` spans 2026-09-23
to 2026-09-30) in the files that matter for this replay, the risk this
uncertainty poses to the Step 1 conclusion is low, but it is not zero.

---

## Step 2: localization chain inventory (from code, not assumed)

| Node | Package | Inputs | Outputs | Frames | In primary bag? |
|---|---|---|---|---|---|
| `ekf_node` (local) | robot_localization, `ekf.launch.py`/`ekf.yaml` | `odom0`=`/odom`, `imu0`=VESC IMU (`/sensors/imu/raw` or similar) | `/odometry/filtered`, TF `odom→base_link` | `world_frame: odom` | `/odom` yes, **IMU: no topic in any archived bag** |
| `slam_pose_relay_node` | f1tenth_localization | `/slam/pose` | `/slam/pose_calibrated` (covariance overwritten with fixed 0.1/0.1/0.05 diag) | map-frame passthrough | yes (both topics) |
| `ekf_node` (global) | robot_localization, `ekf_global.launch.py`/`ekf_global.yaml`, namespaced `/ekf_global` | `odom0`=`/odometry/filtered` (fused differential), `pose0`=`/slam/pose_calibrated` (fused absolute) | `/ekf_global/odometry/filtered`, TF `map→odom` | `world_frame: map` | yes (both inputs + output) |
| `raw_odom_map_tf_node` | f1tenth_localization | `/odom` | TF `map→odom` (unfiltered mirror) | — | input yes, **but not the active mode in this bag** (see below) |

**Replayable layers, per the task's own (a)/(b)/(c) split:**

- **(a) `slam_pose_relay_node`**: fully replayable. Pure stateless
  transform (confirmed from source: no TF lookups, no clock use — just
  `out.pose.pose = msg.pose.pose` plus a fixed covariance overwrite).
- **(b) `ekf_global`**: replayable, with one real wrinkle found only by
  trying it live (see Step 3): it needs `odom→base_link` from `/tf` — not
  for its own fusion math (`odom0` is differential, `pose0`'s frame
  already matches `world_frame`, so neither triggers a real cross-frame
  lookup), but to **compose** its own `map→odom` output against the local
  EKF's real-time `odom→base_link`. It must never see its own `map→odom`
  edge played back.
- **(c) `raw_odom_map_tf_node`**: code-complete and trivially replayable
  (pure mirror of `/odom`), but **not testable for parity from this bag**:
  the bag's own `/ekf_global/odometry/filtered` stream (continuous,
  ~1945 msgs over 42.5s) proves `localization_source` was `ekf` when this
  was recorded, not `raw_odom` — the two modes are mutually exclusive
  (`localization.launch.py`'s own `if/elif`). There is no recorded
  `raw_odom_map_tf_node` output in this bag to compare against. Not
  tested; flagged rather than silently skipped.
- **Local EKF (`ekf.launch.py`)**: **not replayable from any archived
  bag.** No bag in `~/bags/humble_reference/` has an IMU topic, and the
  local EKF's `imu0` input is required (not optional) in `ekf.yaml`.
  Per the gate instructions, **moved to Phase 6** (live, on the stand) —
  not faked with a partial-input substitute here.

---

## Step 3: replay harness

New scripts under `scripts/jazzy_parity/` (committed in `jazzy/p1:
localization parity harness`):

- **`bag_compat.py`** — the Jazzy `TopicMetadata(id=...)` vs Humble
  compatibility shim (tries `id=` first, falls back on `TypeError`) for
  any script that *writes* a bag. Reading is distro-agnostic already.
- **`bag_read.py`** — shared topic reader with a documented dedup step
  (see below).
- **`filter_bag_for_layer.py`** — writes a per-layer input bag: pass
  selected topics through verbatim, optionally dropping specific
  `(frame_id, child_frame_id)` pairs out of the multiplexed `/tf` topic.
  `ros2 bag play --topics` alone is a whole-topic allowlist; it cannot
  drop individual `TransformStamped` entries out of one topic, which is
  exactly what `ekf_global` needed (keep `odom→base_link`, drop its own
  `map→odom`) — so this exists, per the gate's own anticipated escape
  hatch ("write a filtered copy of the bag if `--topics` alone isn't
  enough").
- **`replay_localization.sh`** — CLI-only (`ros2 run`/`ros2 bag`), so the
  identical script runs on Humble on the Orin unmodified. Per layer:
  launches the node(s) with `use_sim_time:=true` and this repo's real
  config, records the layer's own output topics, plays the (possibly
  filtered) input bag with `--clock`.
- **`compare_runs.py`** — the metrics (below).
- **`make_plots.py`** — the plots.

**Isolation:** `ROS_DOMAIN_ID=77` (not used elsewhere on this machine) +
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, no Discovery Server — this
replay runs standalone, not as part of the live stack, so the stack's own
`127.0.0.1:11811` Discovery Server setup is neither needed nor started.

**Two real bugs found building this harness, both fixed, both documented
in the script comments so they aren't rediscovered:**

1. **`ros2 run` orphans its child on `kill`.** It spawns the actual node
   as a subprocess rather than exec-replacing itself; killing the PID
   bash's `$!` captures only kills the wrapper. Observed live: 3
   consecutive runs recorded 2×/3×/4× the true message count from
   accumulating orphaned `slam_pose_relay_node`/`ekf_node` processes all
   still live on the shared `ROS_DOMAIN_ID`. Fixed: pattern-based
   `pkill -9` matching both the wrapper's and the real node's cmdline,
   polled until confirmed dead, both before each run (clean up a previous
   crash) and after (stop what this run started). The recorder itself
   (`ros2 bag record`, invoked directly rather than through `ros2 run`)
   doesn't have this problem and is still stopped gracefully by PID
   (SIGTERM, not SIGKILL — confirmed live that SIGKILL-ing it produces a
   bag directory with no `metadata.yaml` at all, silently destroying the
   very output the script exists to capture).
2. **Duplicate message delivery, byte-identical.** Independent of the bug
   above: every topic recorded by this harness showed each message
   delivered exactly twice — same header stamp, same bag-write time,
   byte-identical serialized payload (confirmed by direct comparison).
   Most likely dual SHM+UDP delivery under FastDDS with this replay's
   LOCALHOST discovery setup — a transport artifact of the harness, not
   the nodes under test. `bag_read.py` collapses consecutive
   byte-identical messages on the same topic before any analysis; this is
   documented in its own module docstring rather than silently baked in.

**A third, more serious bug, found only by actually trying a live
replay — not a harness bug, a real environment bug:** `ekf_node`
crashed on startup with `symbol lookup error:
.../librl_lib.so: undefined symbol:
...diagnostic_updater7UpdaterC1E...dh`. Root cause, confirmed by
comparing raw mangled symbols: the installed `robot_localization` 3.8.3
(built 2026-06-14) was compiled against a `diagnostic_updater::Updater`
constructor signature with one more parameter than the installed
`diagnostic_updater` 4.2.6 (built 2026-04-12) actually provides — a
binary ABI mismatch between two apt packages that had drifted out of
sync on this machine. Both packages had newer, mutually-consistent builds
available (`diagnostic_updater` 4.2.7, `robot_localization`
3.8.3-20260902 — a same-version *rebuild*, evidently recompiled against
the newer ABI). **Andreas upgraded both** (`sudo apt-get install
--only-upgrade ros-jazzy-diagnostic-updater ros-jazzy-robot-localization`);
confirmed fixed by re-running the exact failing command. This blocks the
entire EKF localization chain on real hardware too, not just this
replay — worth flagging prominently since it would have surfaced as a
confusing live-car failure otherwise.

---

## Step 4: runs

3 runs each layer, Jazzy only (no Humble access from this host — see Step
1's conclusion for why the bag's own recording stands in as the Humble
reference instead of a live Orin replay).

| Run | Messages (deduped) | Notes |
|---|---|---|
| relay 1/2/3 | 73 / 73 / 70 | run 3 lost 3 messages near the recorder's shutdown window — timing, not logic (see noise floor below: the 70 that WERE matched still show 0.0 error) |
| ekf_global 1/2/3 | 1561 / 1483 / 1569 (`/tf` map→odom edges) | run-to-run count variance from normal scheduling jitter in a live replay |

---

## Step 5: metrics

### `slam_pose_relay_node` — bit-exact

| | n_ref | n_matched | pos_err RMS | pos_err max | yaw_err RMS | yaw_err max |
|---|---|---|---|---|---|---|
| noise floor (Jazzy×Jazzy, 3 pairs) | 73 | 70–73 | **0.0 m** | 0.0 m | **0.0°** | 0.0° |
| Jazzy vs Humble (×3) | 73 | 70–73 | **0.0 m** | 0.0 m | **0.0°** | 0.0° |

Every matched sample, every run: exact zero. Expected — this node copies
`msg.header`/`msg.pose.pose` through unmodified and only overwrites the
covariance diagonal with fixed config values; there is nothing in it that
could differ by platform, numpy version, or distro. **Pass, trivially.**

### `ekf_global_filter_node` — cold-start transient, then steady-state parity

**The methodology problem, found and handled, not hidden:** the primary
bag is a 42.5s **segment of an in-progress mission** — its Humble
`ekf_global` instance had already been running and fusing corrections for
some time before this clip starts (confirmed: the bag's first
`/ekf_global/odometry/filtered` sample is already at `(4.01, 0.21)`,
matching the concurrent `/slam/pose` reading exactly). This replay starts
`ekf_global_filter_node` **fresh** for just this clip, so it begins at
the filter's documented identity seed `(0, 0, 0)` (see `ekf_global.yaml`'s
own "EKF_ONLY-at-startup bootstrap" comment) and has to converge onto the
same trajectory via the clip's own ~13 pose0 corrections before the two
can be compared on equal footing. **Measured, not assumed:** position
error starts at 3.94 m / 17.5° and decays to noise-floor levels by
**t≈18–20s** (see `output/phase1/plots/error_over_time.png` — a sharp
drop around t=13–14s coinciding with a correction, then settling), and by
the end of the clip (t=42s) Jazzy and Humble land within 0.3 mm of each
other (`output/phase1/plots/trajectory_overlay.png`). This is a replay
artifact of starting mid-mission, not a Jazzy-vs-Humble difference — the
noise-floor runs (Jazzy vs Jazzy, same cold start) show the identical
transient shape, confirming it's about the *restart*, not the *distro*.

**Both windows reported for transparency** (full data in
`output/phase1/full_window/ekf_global_metrics.json` and
`output/phase1/ekf_global_metrics.json` respectively):

**Full window (t=0–42s, dominated by the convergence transient):**

| | pos_err RMS | pos_err max | yaw_err RMS | yaw_err max |
|---|---|---|---|---|
| noise floor (Jazzy×Jazzy) | 0.0–0.017 m | 0.012–0.037 m | 0.01–0.03° | 0.13–0.19° |
| Jazzy vs Humble (×3) | **2.06–2.20 m** | 3.90–3.97 m | **9.18–9.71°** | 19.48–19.52° |

**Steady state (t≥20s, after the transient settles — the number that
matters):**

| | pos_err RMS | pos_err p95 | pos_err max | yaw_err RMS | yaw_err p95 | yaw_err max |
|---|---|---|---|---|---|---|
| noise floor (Jazzy×Jazzy, 3 pairs) | 0.0008–0.0014 m | 0–0.0002 m | 0.009–0.012 m | 0.0016–0.0027° | 0–0.0017° | 0.030–0.056° |
| Jazzy vs Humble (×3) | **0.0044 m** | 0.0091 m | 0.0135 m | **0.020°** | 0.045° | 0.12° |

Jazzy-vs-Humble RMS sits at roughly **3× the position noise floor and
~8× the yaw noise floor** — a real, measurable, consistent (identical
across all 3 runs to 3 significant figures) difference, but one whose
*absolute* size is 4.4 mm and 0.02° on a car-scale problem. Judged against
"a small, justified multiple of the floor," this passes; judged against
the plan's old static thresholds (rejection 3±2%, max yaw step ≤5.54°) it
isn't even close to those limits either. **Pass.**

**Output rate:** `humble_rate`/`jazzy_rate` in the metrics JSON — both
land at the configured 50 Hz within normal jitter (`period_ms_p90` well
under the 20 ms nominal period's usual 2–3× tolerance); not a
distinguishing factor.

**Correction steps:** `output/phase1/plots/correction_steps.png` overlays
every pose0-correction yaw step, Humble vs all 3 Jazzy runs, across the
whole clip. After the transient (~t>15s) the four traces are visually
indistinguishable — same peaks at the same times, same magnitudes. Before
it, Jazzy's steps are measurably different from Humble's (expected: a
cold-started filter's covariance, and therefore its Kalman gain on each
correction, genuinely differs from an already-converged one — this is a
state difference, not a step-measurement artifact). Max single-step yaw:
Humble 0.210°, Jazzy 0.258° (identical to 3 sig figs across all 3 runs) —
both tiny compared to the plan's old 5.54° reference figure (that figure
describes a *different*, now-superseded Q tuning; see `ekf_global.yaml`'s
own history comment).

### pose0 rejection rate — **method requested could not be validated; reported honestly, not faked**

The original methodology, as documented in `ekf_global.yaml`'s own
comments, measured rejections directly from **689 pose0-to-pose0 intervals
across 37 archived runs**, comparing `/slam/pose_calibrated` deltas
against `/odom` deltas, against the *internal* Mahalanobis accept/reject
decision inside the filter. **That data no longer exists on this
machine** — this phase has one 42.5s clip with 73 corrections, not 37
runs' worth. Per the gate's own fallback instruction ("derive rejections
from the outputs... and explain the method"), I implemented a substitute:
flag a correction as "rejected" if the resulting `map→odom` yaw step is
no larger than the Jazzy-run-to-run noise floor for that same step
(≤0.096°, the 95th percentile of pairwise Jazzy step differences).

**This substitute does not work, and I'm reporting that rather than the
number it produces.** It returns 94–97% "rejected" across Humble and all
3 Jazzy runs — wildly inconsistent with the documented 3.1% baseline.
Root cause: the proxy conflates "genuinely rejected" with "accepted but
small," and in a well-tuned filter (which this one is, by the yaml's own
account) **most real corrections are small on purpose**, not because they
were thrown out. Step size alone cannot distinguish the two from bag
outputs — the real accept/reject decision happens inside
`FilterBase::checkMahalanobisThreshold` and is never published anywhere
this replay can observe (confirmed: `debug: false` in the config, so even
the Mahalanobis-failure log line never fires). **What this proxy DOES
show, and is worth keeping:** applied identically on both sides, Humble
and all 3 Jazzy runs land within 3 percentage points of each other
(97.2% / 94.4% / 95.7% / 97.2%) — i.e. whatever this flawed metric is
actually measuring, Jazzy and Humble measure the same on it. That's weak
evidence of parity, not validation of the 3.1% tuning claim, and
shouldn't be read as more than that.

**For a real answer:** re-run this with `print_diagnostics: true`'s
`/diagnostics` output recorded too (not done in this pass) and check
whether robot_localization's diagnostic_updater surfaces a per-correction
accept/reject count there, or re-run the original 37-run/689-interval
methodology directly against the Jazzy filter if the archive is ever
recovered. Flagged as a Phase 1 follow-up, not resolved here.

---

## What isn't covered, and where it moves

- **Local EKF (`ekf.launch.py`) + IMU fusion**: no IMU topic in any
  archived bag → cannot be parity-tested offline. **Moves to Phase 6**
  (live, on the stand), per the gate instructions — not faked with a
  partial-input substitute.
- **`raw_odom_map_tf_node`**: code-complete, replayable, but this bag was
  recorded in `ekf` mode, not `raw_odom` mode, so there is no recorded
  output to compare it against. Not tested.
- **pose0 rejection rate**: substitute method didn't validate the
  original claim; see Step 5 above.

---

## Commits

```
9beea9b jazzy/p1: localization parity harness — slam_pose_relay bit-exact, ekf_global within noise floor at steady state
```

Pushed to `origin/jazzy` (confirmed via `git log --oneline origin/jazzy..HEAD`,
2026-10-01 addendum pass — not pushed by this agent; `3781f2e`/`9beea9b` were
already on the remote when checked).

`scripts/jazzy_parity/{bag_compat,bag_read,filter_bag_for_layer,
replay_localization.sh,compare_runs,make_plots}.py`,
`output/phase1/{*.json,plots/*.png,runs/,inputs/}`,
`output/phase1_localization_report.md`. The two apt package upgrades
(`diagnostic_updater`, `robot_localization`) are a machine-state fix, not
a repo change — documented here for completeness, same convention as
Phase 0's `asio_cmake_module` fix.

---

## Decisions for Andreas

1. **`diagnostic_updater`/`robot_localization` ABI mismatch — already
   fixed on this machine, but check the car's Jetson too** before the
   first live EKF test there. If the same apt-package drift exists on the
   vehicle, `ekf_node` will crash identically.
2. **pose0 rejection rate is unresolved.** The 3.1% baseline from the
   original tuning pass could not be reproduced or validated by any
   method available from this one bag. If that number still matters for
   go/no-go on the Q tuning, it needs either the original 37-run archive
   (if it still exists anywhere) or a fresh capture with `/diagnostics`
   recorded through a live run.
3. **Local EKF + IMU parity is now explicitly a Phase 6 (live, on-stand)
   item** — no bag anywhere in the archive has IMU data, so this genuinely
   cannot be done offline, not a scope cut of convenience.
4. **`raw_odom_map_tf_node` has zero parity coverage** — not a problem
   now (it's a simple fallback mirror, already low-risk by inspection),
   but if a bag ever gets recorded in `raw_odom` mode, it should be run
   through the same harness (`replay_localization.sh` would need one new
   `case` branch — trivial addition, not done here since there's nothing
   to compare it against yet).
5. **Harness reusable for later phases**: `replay_localization.sh` /
   `compare_runs.py` are CLI-only and distro-agnostic by construction —
   intended to be the same mechanism later phases use, not a one-off.
