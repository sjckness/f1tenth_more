# Avoidance pass — log

Branch `campaign-final`, Orin, Humble. Backup tag `backup/pre-avoidance-69830ecf`.
Nothing pushed, no stack or sensor launched, no driving, nothing left running.
`ROS_SUPER_CLIENT=TRUE` set for every ROS invocation.

**Stopped after A2 at the user's request.** A1, A2 and A5 are complete and
committed. A3 (beyond the premise check below), A4 and `report.md` were not
started. See "Not done" at the end.

## Commits

| commit | what |
|---|---|
| `d3671b17` | A1 — wall rows as the default boundary source, launch args forwarded |
| `16c91ded` | A2 — last-resort LiDAR stop (`hard_stop_node`) |
| (no commit) | A5 — `.git` cleanup, nothing tracked to commit |

## A5 — `.git` cleanup (done)

- `cp -a .git ~/git_backup_2026-10-19` (597 MB).
- Removed the 6 zero-length loose objects and `.git/gc.log`. The list was
  printed and the count asserted at 6 before anything was deleted; only files
  matching `-type f -size 0` under `.git/objects` were touched, no recursive
  delete.
- `git fsck --no-dangling`: **clean, no output, exit 0**.
- `git log` intact: 304 commits reachable from HEAD, same tip as before.

Those objects were orphaned garbage from an earlier interrupted write — `fsck`
reported no broken links to them even while they existed, so nothing reachable
referenced them. They only ever blocked `gc`'s prune step.

## A1 — wall rows as the default (done)

Evidence: `output/avoidance/a1_launch_evidence.txt`, regenerable with
`python3 tools/avoidance_launch_evidence.py --diff boundary_source:=costmap`.
That tool evaluates the launch description rather than reading the yaml, because
the claim is about what reaches the node.

- `boundary_source` resolves to **`walls`** with no launch overrides.
- `boundary_source:=costmap` changes **exactly one key** in mpc_corr's whole
  parameter set (`keys that differ in total: ['boundary_source']`), so the old
  behaviour comes back exactly.
- `navigation.launch.py` declares and forwards **13** boundary/wall-row keys,
  **0 forwarded-but-undeclared**. It previously forwarded none at all: the
  include had no `launch_arguments`, so every mpc_corr argument given to
  `stack_bringup` or to the supervisor's `navigation` entry was accepted at the
  top level and silently dropped.
- `boundary_max_sources` 3 → **4**. Load-bearing, not cosmetic: with `walls` as
  the default it must be >= `max_wall_constraints` or `MPCController` refuses to
  start, so leaving it at 3 would have meant the car's own launch path could not
  come up. A test pins the pair.
- `boundary_slack_max` is now a real parameter, default **unchanged at 0.5**.
- `wall_row_slack_weight` unchanged at **2.0e6**.
- `back` tracks still never produce rows — now a recorded decision (D3 in
  `decisions.md`): the car never reverses in this campaign.

12 new tests in `f1tenth_navigation`. `mpc_controller` unchanged at 782 pass /
11 pre-existing failures (verified identical with HEAD's `stack_params.yaml`
swapped back in).

## A2 — last-resort LiDAR stop (done)

`src/f1tenth_control/f1tenth_control/f1tenth_control/hard_stop{,_node}.py`,
launched from `ackermann_mux.launch.py` (supervisor component `control`, not
`behavior`). 47 new tests, all four the work order names:

| test | result |
|---|---|
| object at 0.18 m ahead → stop within 2 scans | pass — engages on scan 2, publishes zero speed on `/safety_stop` |
| self-hit points → no stop | pass — a scan that is nothing but a 0.10 m rear arc, 20 times, never engages |
| clear for 0.5 s → release | pass — one clear scan does not release; 0.525 s does; a hit during the hold restarts the clock |
| works with the BT process killed | pass — asserted on the import graph (no `py_trees`, no `f1tenth_behavior`) and functionally (one subscription, `/scan`) |

### Measured geometry, from the bag's own `/tf_static`

`base_link → laser` is `(+0.3050, 0, +0.1900)`, `base_link → base_front` is
`(+0.4050, 0, 0)`. So the laser is **0.10 m behind the bumper** and the 0.20 m
trigger radius reaches `base_link` x = 0.505, i.e. **0.10 m ahead of the
bumper** — the work order's arithmetic checks out.

### The self-hit mask blinds a band, on purpose

The 0.20 m circle about the laser is mostly **inside the car** (half-width
0.136 m, bumper 0.405 m, both grown by the 2 cm self margin). Measured, and
pinned by `test_the_blind_band_is_exactly_the_car_s_own_body`:

| range from laser | blind bearings |
|---|---|
| 0.20 m | 129–135° only (7 of 136) |
| 0.19 m | 16 of 136 |
| 0.18 m | 49–60° and mirror (28 of 136) |
| 0.12 m | **all** of them |

The blind set is exactly the set of points inside the car's own outline, where
there cannot be an external object the car is not already touching. But it means
the stop must engage **on the way in**, at 0.20–0.13 m, and latch — it cannot be
relied on to fire at the moment of contact. That is why the debounce is 2 scans
(50 ms at the measured 40 Hz) and not 20.

### Replay on today's bags: 0 triggers, as expected

Not run as a replay; answered directly from the scan census already taken, which
settles it without one:

- `corridor_obstacles_20261019T135307`: the per-scan nearest valid return over
  all 1549 scans is **min 1.171 m**, median 1.181 m, max 1.188 m, all at
  −74…−77°. **0 scans under 0.20 m**, and 0 under 0.40 m.
- `corridor10m_20261019T131815`: raw laser minimum over the whole 3223-scan,
  19.41 m run is **0.603 m**.

Both are far outside the trigger radius at every scan, so the node triggers 0
times on either bag — the correct result, and it also means neither bag exercises
the self-hit mask (there are no self-hits in them to exercise it with), which is
why that path is covered synthetically instead.

## A3 — premise checked and found false (not completed)

The work order asks why recognised-object avoidance "passes too close" on today's
obstacle run. **It did not pass close, and on the named bag it did not pass at
all.** Measured:

### `corridor_obstacles_20261019T135307` — a standstill, not a run

- `/odometry/filtered` path length **0.00 m**; max |v| **0.000 m/s**.
- All 389 `/drive` messages have speed **exactly 0.000**.
- `/mission/status`: state `IDLE`, no mission json.
- `/mpc/solver_status`: **0 messages** — the MPC never solved.
- `/behavior/tree_status`: `root=FAILURE`, all three lanes FAILURE.
- Nothing was ever in the car's path: the forward strip (|y| <= 0.25 m, x > 0)
  is clear to **19.59 m**, and the nearest return anywhere in 45.7 s is 1.171 m
  at −76°.

### `corridor10m_20261019T131815` — the only moving run, and it kept its margin

19.41 m driven, 312 solves. **Minimum body clearance over the entire run is
0.342 m**, against an intended standoff of `car_radius + obstacle_safety_margin_m`
= 0.32 m. The 518 scans under 0.40 m are all one wall **0.49 m off the car's left
side**, between t = 74 s and t = 90 s, at `base_link` ≈ (−0.07, +0.49) — beside
and slightly behind the car, not an obstacle passage. 11 grouped events, every
one of them that same wall.

So there is no close passage to explain, and none was invented.

### What the bags do show: two real defects

**1. Recognised-object perception was dead for ~98% of both runs.** Detections
exist only in the first second and then stop for good:

| bag | `/camera/detections_3d` | last detection | duration |
|---|---|---|---|
| `corridor_obstacles_…T135307` | 5 msgs | t = 0.84 s | 45.7 s |
| `corridor10m_…T131815` | 7 msgs | t = 1.27 s | 90.4 s |

Cause, from the bag's own logs: `/perception/scene_labels/health` flips to
`stale: true, reason: no_depth` at t = 1.80 s with `depth_age_s 1.013`, i.e. the
last ZED depth frame arrived at t ≈ 0.79 s, right after the ZED logged its static
camera-centre transform. `front_clearance_node` then warns continuously for the
rest of the run ("scene mask STALE (no_depth) … last depth frame 22.512 s ago").
The ZED IMU keeps publishing at 166 Hz and pose at 25 Hz throughout, so the node
is alive — only depth and the rectified image die. This is the known
positional-tracking failure, recurring.

**2. The MPC's obstacle set never decays.** `/mpc/solver_status` reports
`n_obstacles = 1` on **all 312 solves** of the moving run — the single chair
detected at t = 1.27 s, 5.8 m away and 33° off the path, carried as a live
constraint for the remaining 89 seconds. `/mpc/min_obstacle_distance` on the
standstill bag is likewise frozen in a 4.484–4.544 m band for all 391 messages.
`MPC_corr.obstacles_global_live` is overwritten on each `/perception/obstacles_2d`
and never cleared or aged, which its own comment states ("live, no persistence");
with the publisher dead, the last list is latched forever. A removed obstacle is
never forgotten and a stale one keeps deflecting.

### Also missing from the evidence, for next time

- `MpcSolverStatus` carries no per-constraint slack, so "slack used per
  constraint" is not recoverable from these bags. `/mpc/wall_constraints_diag`
  (added in `91fe0b0e`) now carries it going forward.
- `/mpc/corridor_markers` recorded **0 messages**, so corridor width at a given
  instant is not recoverable either.

No fixes were proposed or applied for either defect — that was the part of A3
still outstanding when work stopped.

## Not done

- **A3** — the per-passage table, the 4–6 topdown PNGs, and the proposed minimal
  fix per cause. The premise check above is as far as this got; the two defects
  it surfaced have no proposed diff yet.
- **A4** — costmap island census (isolated clusters < 3 cells per frame, how many
  produced a constraint under the old `costmap` default, how many were real).
  The inputs are present: 9 `/slam/map` messages and 386/845 `/costmap/boundaries`
  messages in the two bags.
- **`output/avoidance/report.md`** — the final summary. `decisions.md` and this
  log stand in the meantime.
