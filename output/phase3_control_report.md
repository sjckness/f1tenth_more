# Phase 3: MPC control parity and timing (offline, Thor)

Previous phase: `output/phase2_slam_costmap_report.md`, verdict **GO-WITH-NOTES**.
Its one pending item, the Orin frozen-input boundary run, is non-blocking here:
Phase 3 replays the *recorded* Humble `/costmap/boundaries`, so it does not
depend on that result. Gate read before starting.

## Verdict: **GO-WITH-NOTES — Orin comparison pending**

What is proven on Thor:

- **The MPC solver is deterministic and portable at the function level.**
  - Re-solving all 419 captured control ticks offline, with no ROS, reproduces
    the live Jazzy node's output bit for bit on Thor.
  - The same 419 frozen ticks give bit-identical results under osqp 1.0.5,
    1.1.0 and 1.1.3.
  - Under an older Python 3.8 / NumPy 1.17 / SciPy 1.5 / osqp 1.0.5 stack they
    stay within OSQP's own convergence tolerance: commands differ by
    ≤ 2.5e-12 rad, and status and iteration counts are identical on every tick.
- **The Jazzy replay harness works and its noise floor is characterised.**
  - The goal was never recorded. It is reconstructed through the stack's own
    code and injected identically on every run.
  - Three Jazzy runs agree on everything that is not a timing artefact.
  - The tick-level floor is dominated by one mechanism, explained below.

What is **not** proven yet, and blocks a plain GO:

- **Jazzy vs Humble.** It needs the Orin replays (`output/phase3/ORIN_MPC_INSTRUCTIONS.md`).
- **The Orin's osqp version (Step 2).** It is bounded to 1.0.0–1.1.3 and still
  pending. The Orin's frozen-input run settles it for this data.

No production code was changed in this phase. All five commits are
harness, data and documentation (see "Commits").

---

## Step 1: inventory (from code, `bc3b049` = `e47e646` = HEAD for every file involved)

### Version check, re-done for everything the MPC and its goal path load

| What | `bc3b049` → `e47e646` | `e47e646` → HEAD |
|---|---|---|
| `src/f1tenth_control/` (`MPC_corr.py`, `mpc_solver.py`, `vehicle_model.py`, `wall_tracker.py`, `wall_turn.py`, `campaign_status.py`, `mpc_corr.launch.py`, …) | 0 files | `mpc_controller/package.xml` only (+`<test_depend>`, Phase 0) |
| `src/f1tenth_params/` Python (`param_defaults.py`, `corridor_geometry.py`, `object_geometry.py`) | 0 | 0 |
| `stack_params.yaml` | 4 comment-only lines; **parsed YAML compared directly: all 288 top-level entries identical** | same check, identical |
| `src/f1tenth_messages/` | 0 | 0 |
| Goal path: `f1tenth_intelligence/llm` (translator), `f1tenth_behavior` (mission parser, `PublishMoveGoal`) | 2 comment-only lines in `f1tenth_behavior` | 0 |
| Golden mission fixture `ex6_stop_two_metres_from_wall.json` | byte-identical (sha256 `fb1739f1…`) | byte-identical |
| `f1tenth_bringup/config/mux.yaml` | 0 | 0 |

Thor's workspace is a `--symlink-install`, so the replay runs exactly the
working-tree source. Checked: `build/mpc_controller/mpc_controller` resolves to
`src/…`.

### `MPC_corr` node (`mpc_controller/MPC_corr.py`, node name `mpc_corr`)

**Timer:** a single `control_loop` timer at `ts = 0.1 s` (10 Hz). The corridor is
rebuilt inside it every `corridor_update_period = 1.0 s`.

**Subscriptions**, with what each one actually drives in *this* mission:

| Topic | Type / QoS | Effect in this mission (straight drive, front-clearance stop) |
|---|---|---|
| `get_odom_topic()` = **`/odometry/filtered`** (`localization_source: ekf`) | Odometry, BEST_EFFORT | **State.** `hw_odom_callback` caches it; `control_loop` copies it into `self.x/y/yaw/v`. |
| `/model/virtual_robot/odometry` | Odometry | Sim only. Absent live and in the bag. |
| **`/mpc/goal_drive`** | DriveCommand | **The goal** (see "Goal" below). Not recorded; injected. |
| `/mpc/goal_distance`, `/goal_pose`, `/goal_object`, `/goal_object_end`, `/goal_turn` | — | Other goal shapes. 0 messages, not used by this mission. |
| **`/mpc/hold`** | Bool | **Stop.** The BT's front-clearance stop sends `true` at t=42.42 s. Replayed from the bag. |
| **`/perception/obstacles_2d`** | Obstacle2DArray | **Obstacle cost (`w_obs`)** and the predicted-clearance outputs. Replayed. |
| **`/costmap/boundaries`** | BoundaryConstraintArray | **Hard QP rows** (`use_hard_boundary_constraints: true`, 3 per stage). Replayed. |
| `/scan` | LaserScan, sensor QoS | Wall tracker only (`wall_track_enable: true`). `_tick_wall_tracker` returns unless `drive_cmd.mode == 'wall_turn'` (`MPC_corr.py:3373-3376`), so **no effect here**. Replayed anyway for subscription and load fidelity. |
| `/imu` | Imu | `ax_imu`, read only for the `mpc_control_compare.csv` debug log (`:4072`). **No effect.** |
| `/joint_states` | JointState | `delta_real`/`v_real_log`, read only for the same CSV and a log line. **No effect.** |
| `/perception/front_distance` | Float32 | Read by `_fresh_front_distance()` (wall_turn branch only, `:4320`) and the corridor debug JSON. `build_straight_corridor` reads it at `:4190`, but on the drive path it is overwritten (wall_turn) or only logged. **No effect.** |
| `/perception/d_wall/psi_correction` (`wall_distance_output_topic` + suffix) | Float32 | **Does affect straight drive:** `psiEnd = psi_base + _fresh_d_wall_correction()` (`:4426-4427`), with `corr_d_wall_correction_enable: true`. Not in the bag. In the replay it is absent on **both** stacks, which is the code's own degraded path (returns 0.0: "no message yet"). Whether `wall_distance_node` ran live is unknown (it publishes no diagnostics). This is a possible contributor to the *sanity* offset only, never to parity. |
| `/test_campaign/logger_status` | String | Only redirects where `corridor_debug.jsonl` is written. **No effect.** |
| `/tf`, `/tf_static` (TransformListener) | — | Lookups: `base_link←laser` (wall_turn only) and `map→odom` (goal-anchor / object paths only; `goal_anchor_map` is `None` in drive mode). **No effect here.** Replayed in full anyway: the MPC publishes no TF, so nothing can collide. |

Not an MPC input, although it was listed in the plan: `/costmap/front_clearance`.
`MPC_corr` doesn't subscribe to it. It feeds the BT's stop condition, whose
result reaches the MPC as `/mpc/hold`. It is not replayed. The same goes for
`/ekf_global/odometry/filtered`, which the MPC doesn't read either; `map→odom`
reaches it via `/tf`.

**Rule applied, as the plan requires:** every input that matters is fed
identically to both stacks, from the bag or (the goal) from one shared
message. `d_wall/psi_correction` gets the same constant on both: absent.

**Publications:**
- `/drive` (AckermannDriveStamped, `frame_id ''`)
- `/mpc/drive_clamp`
- `/mpc/solver_status` (MpcSolverStatus: status, `solve_dt`, cost, boundary
  count, obstacle count, 20-step prediction)
- `/mpc/status` (JSON, including OSQP iterations)
- `/mpc/corridor_markers` and `/corridor` (1 Hz)
- `/mpc/min_obstacle_distance`, `/mpc/min_obstacle_distance_forward`
- `/mpc/predicted_min_clearance`
- `/mpc/goal_reached` (never in drive mode)
- `/mpc/object_status`
- `/mpc/wall_track` (wall_turn only)

**Parameters:** all come from `mpc_corr.launch.py`, about 30 of them, each a
`DeclareLaunchArgument` whose default is `get_default(<stack_params key>)`.
The rest are in-code `declare_parameter(key, get_value(key))`. The replay
launches the production launch file unmodified (see Step 3). The resolved
values are dumped from the live node into `runs/jazzy_*/params.yaml`
(71 lines), and the capture run reuses that file. Key values:
- `ts=0.1`, `N=20`, `use_rti_solver=true`, `boundary_hard=false` (slack),
  `boundary_max_sources=3`
- `max_forward_speed_mps=0.5`, `car_radius=0.20`, `avoidance_margin=0.12`
- `delta` limits −0.283/+0.278 rad
- OSQP is set up with `warm_starting`, `polishing`, `max_iter=200` and
  default `eps_abs = eps_rel = 1e-3` (`mpc_solver.py:1144-1145`)

**State carried across ticks:**
- `warm_start_z` (the previous solution, shifted) and `last_u`
- the cached corridor (rebuilt every 1 s) and `psi_init_corridor`, anchored
  when the goal arrives
- `drive_cmd` and `vdes`
- `goal_anchor_*` (unused in drive mode)
- hold/clamp state
- the wall tracker (idle in straight mode)
- the odometry staleness timers (0.5 s)

### Command chain down to the mux

`MPC_corr` → **`/drive`** (the mux's `navigation` lane, priority 10, 0.2 s
timeout) → `ackermann_mux` → **`/ackermann_drive`** →
`ackermann_to_vesc_node` → VESC.

The other mux lanes are:
- `safety_stop` (200): the BT `Stop` behaviours, `frame_id 'base_link/emergency'` or `'base_link/obstacle'`;
- `teleop` (100);
- `calibration_drive` (50).

In the bag, all 369 `/drive` messages have `frame_id ''`, i.e. they are
`MPC_corr`'s. `/safety_stop` has 0 messages. **`/drive` is the MPC's own output
and is what this phase compares.** `/ackermann_drive` (367) is the mux output.
No mux runs in the replay.

### Goal: how it reached the MPC in `bc3b049`, and how it is reconstructed

- `/mission/status` names `…/missions/llm_generated/llm_2b356aac445f.json`.
- `plan_translate.translate()` sets `mission_id = 'llm_' + sha1(canonical_json(intent))[:12]`
  (`plan_translate.py:975-977`). The committed golden fixture
  `llm/test/golden/ex6_stop_two_metres_from_wall.json` ("vai dritto e fermati a
  due metri dal muro") carries the same id, so the intent is known exactly.
- The mission is one `drive` move: `straight`, speed 0.4, stop on
  `front_clearance ≤ 2.0 m`.
- `PublishMoveGoal` (`publish_move_goal.py:152-168`) sent **one**
  `DriveCommand` on `/mpc/goal_drive` at move start. That topic was not in the
  logger's topic list.
- The move lasted 55.98 s (`/mission/move_outcome`) against a 42.5 s bag, so
  the goal predates the recording by about 13.5 s.

`scripts/jazzy_parity/make_mpc_goal.py` repeats the live path **with the
stack's own functions**, not a re-implementation:
1. `translate(intent)`. Asserted: the id equals the bag's, and the mission
   equals the golden fixture.
2. `mission_config.parse_mission()`.
3. The real `PublishMoveGoal.update()`, with a capturing publisher.

Result (`output/phase3/goal_drive.yaml`):
`{mode: straight, turn_sign: 0.0, turn_mag_deg: 0.0, speed: 0.4, d_safe: -1.0}`.
The Orin rebuilds it with Humble's code and diffs it against Thor's (instructions, Part B).

**Injection, per the decision (option 1):**
- **When:** at bag t = **0.5 s**, i.e. the first `/odometry/filtered` message
  + 0.5 s, written into the replay input bag by `filter_bag_for_layer.py --inject`.
- **Why not exactly t=0:** `goal_drive_callback` drops a goal while
  `self.x is None` (`MPC_corr.py:2767-2771`), and `self.x` is only set inside
  `control_loop` (`:3065`). 0.5 s guarantees several ticks with odometry first.
- **Same message on both stacks:** being part of the input bag, the goal
  arrives at the same sim time on every run and on both distros. The Orin
  rebuilds the input bag itself, and `bag_digest.py` compares content hashes:
  Thor's is `b621d170…`.
- **Every run anchored the same heading:** each Jazzy run logged
  `psi_init_corridor(re-anchored)=+0.3009 rad`.

---

## Step 2: solver version (osqp) — **PENDING**

- **Thor:** osqp **1.1.3** (in `~/.local`, installed in Phase 0).
- **Orin:** **bounded to 1.0.0–1.1.3**, not established:
  - nothing on Thor pins it (Dockerfiles unpinned; no manifest or report
    mentions a version);
  - `mpc_solver.py` dereferences `osqp.SolverStatus` at import and calls
    `solve(raise_error=False)`, and the unpacked 0.6.7.post3 wheel has neither.
    So 0.6.x would crash on import, yet the bag has 376 solver messages.

**Built so a second version is one extra run:**
- `run_mpc_frozen.py --osqp-target DIR` and `replay_localization.sh` with
  `OSQP_TARGET=DIR` prepend a `pip install --no-deps --target DIR` directory
  to the solver's import path only. Never `~/.local`, and numpy is not touched.
- Every run writes the osqp version and file it actually imported.

**Already exercised on Thor:** osqp **1.0.5** and **1.1.0**, in isolated
scratch targets, against the same 419 frozen ticks as 1.1.3. All three are
**bit-identical**: same QP, same solution, same iterations, same status
(`output/phase3/mpc_frozen_osqp_1.1.3_vs_1.0.5.txt`, `…_vs_1.1.0.txt`). Their
run times differ, so the three were genuinely separate solver builds.
1.0.0–1.0.4 have no cp312/aarch64 wheels, so they can't be tested on Thor.
The Orin's own run (cp310) covers whatever it has installed.

Per the plan, until the Orin reports its version, **any Jazzy-vs-Humble
solver-metric difference is treated as "possibly solver-version".** Given the
result above, a difference that *does* appear will need to be explained by
something other than osqp 1.0.5–1.1.3 numerics.

Recorded osqp 1.1.3 defaults, for the record: `eps_abs=eps_rel=1e-3`,
`check_dualgap=1`, `adaptive_rho_interval=50`, `alpha=1.6`, `rho=0.1`. The MPC
overrides only `warm_starting`, `polishing` and `max_iter`.

---

## Step 3: replay harness (open loop)

**Extended, not forked:**
- `scripts/jazzy_parity/replay_localization.sh` gets the new layers `mpc` and
  `mpc_capture`.
- `filter_bag_for_layer.py` gets `--inject` / `--inject-after`.
- New files:
  - `make_mpc_goal.py`
  - `bag_digest.py`
  - `mpc_replay.launch.py`
  - `mpc_capture_node.py`
  - `mpc_frozen_io.py`
  - `freeze_mpc_inputs.py`
  - `run_mpc_frozen.py`
  - `compare_mpc_frozen.py`
  - `mpc_metrics.py`, plus an `mpc` layer in `compare_runs.py`
  - `mpc_timing.py`
  - `make_plots_phase3.py`

`scripts/replay_bag_through_mpc.sh` was read and its approach reused: inputs
only, an isolated domain, recording the MPC's outputs. It was not reused
as-is, for three reasons:
- it plays `/mission/status` and `/behavior/tree_status`, which aren't MPC
  inputs;
- it overrides QoS to best_effort, which is wrong for this bag: every input
  here was recorded RELIABLE;
- it has no `use_sim_time`, no goal injection and no `/tf_static` QoS
  preservation.

| | |
|---|---|
| **Input bag** | `/odometry/filtered`, `/costmap/boundaries`, `/perception/obstacles_2d`, `/mpc/hold`, `/scan`, `/tf`, `/tf_static` + injected `/mpc/goal_drive`. All recorded MPC outputs and everything downstream are left out. Offered QoS is carried through (the Phase 2 `/tf_static` lesson: TRANSIENT_LOCAL kept). |
| **Node** | The production `mpc_corr.launch.py`, unmodified, via `mpc_replay.launch.py`. It only adds `SetParameter(use_sim_time=True)`, so the parameters are exactly production's and nothing is re-listed by hand. `taskset -c 10,11` is the production default. |
| **Isolation** | `ROS_DOMAIN_ID=77`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, no discovery server, no drivers, no mux. The node itself warns "PUB /drive nessun subscriber". Checked before running: no ROS, VESC or mux processes on the machine. |
| **Playback** | `ros2 bag play --clock 1000 --delay 3`, rate 1.0. |
| **Recorded** | Every MPC publication, plus `/mpc/goal_drive` (as proof of delivery). Also the live `params.yaml` dump, the MPC process's CPU time, the imported osqp/numpy/scipy versions and the node's debug files. |
| **Distro** | CLI only: `ros2 launch`, `ros2 bag`, `ros2 param`. Bag writing goes through `bag_compat`. `F1TENTH_REPO` lets it run from a copy outside the Orin's `e47e646` checkout. |

**Two harness defects found and fixed while building it:**

1. **The injected goal could be lost.** In one of the first three runs the
   goal reached neither the node nor the recorder, and the MPC sat in
   "robot fermo in attesa" for the whole run. Cause: a single volatile
   message 0.5 s into playback can go out before DDS discovery has matched
   the player's new publisher to the subscriber. The live BT publisher
   existed long before it published. Fix: `--delay 3`, so the player creates
   its publishers and then waits. All 9 later runs received it.
2. **`solve_dt` in a replay is structurally 0.** First I raised `/clock` to
   1 kHz, believing it would give 1 ms solve-time resolution. It doesn't:
   - the node reads `solve_dt` from its own clock;
   - under sim time that clock advances only in the `/clock` callback;
   - that callback cannot run while `control_loop` holds the single-threaded
     executor.
   
   So `solve_dt_sec` is 0.0 on every replayed tick. The 1 kHz clock is kept
   for a different, valid reason: ticks land within 1 ms of their nominal
   times rather than on a 25 ms grid. Matched ticks across runs are ≤ 1.0 ms
   apart. Timing comes from `perf_counter` instead (Step 5). Both points are
   documented in the script.

**Runs:** 3 Jazzy noise-floor runs (`runs/jazzy_{1,2,3}`), each with 419 ticks
and the goal accepted. Run 2 was re-done once, because I edited the script
while it was executing; all three final runs come from the same harness
version. Plus one capture run (`runs/jazzy_capture`, same inputs, 419 ticks).

---

## Step 4: metrics

Evaluation window: from the first tick after the goal (t = 0.56 s) to
`/mpc/hold` (t = 42.42 s), about 419 ticks. Commands are compared two ways:
tick-matched (≤ 50 ms; actual matches ≤ 1.0 ms apart) and zero-order-hold on
a 50 Hz grid. Full numbers are in `output/phase3/mpc_metrics.json`.

### Noise floor: Jazzy × Jazzy (3 pairs)

| Metric | run1–run2 | run1–run3 | run2–run3 |
|---|---|---|---|
| steering, tick: RMS / p95 / max [rad] | 0.0418 / 0.088 / 0.127 | 0.0190 / 0.043 / 0.112 | 0.0405 / 0.087 / 0.130 |
| steering, ZOH 50 Hz: RMS / max | 0.0420 / 0.127 | 0.0199 / 0.112 | 0.0411 / 0.130 |
| speed, tick: RMS / max [m/s] | 0.0020 / 0.013 | 0.0013 / 0.012 | 0.0020 / 0.017 |
| **steady window (t≈17–41 s): Δ mean steering [rad]** | **0.0008** | **0.0002** | **0.0006** |
| steady window: Δ mean speed [m/s] | 0.0001 | 0.0000 | 0.0000 |
| status disagreements (all `solved` ↔ `solved_inaccurate`, both `success=True`) | 2 | 3 | 1 |
| cost, relative: RMS / max | 0.031 / 0.249 | 0.022 / 0.249 | 0.028 / 0.223 |
| hard-constraint count / horizon length equal | 419/419, 419/419 | 419/419, 419/419 | 418/418, 418/418 |
| `/mpc/drive_clamp` events (a / b) / matched ≤ 50 ms / applied-speed error | 172/169, 160, 0.0 | 172/172, 166, 0.0 | 169/172, 161, 0.0 |
| corridors matched ≤ 0.15 s / centreline max deviation / heading error | 14 of 40, ≤ 0.044 m, ≤ 0.004 rad | 39 of 40, ≤ 0.052 m, ≤ 0.002 rad | 13 of 39, ≤ 0.037 m, ≤ 0.004 rad |

Per run:
- status: `solved` 413/413/414 and `solved_inaccurate` 6/5/4, with nothing
  infeasible, failed or rejected;
- horizon 20 and 3 boundary rows on every tick;
- OSQP iterations: mean about 86, p50 75, p95 175, **max 200 = `max_iter`**,
  reached on a few ticks.

**What produces the floor, from the evidence:**

- **The solver is not a source of it.** Re-solving each captured tick's exact
  inputs offline reproduces the live output bit for bit (`mpc_frozen_thor_vs_live.txt`).
  Every difference between runs therefore enters through *which input sample
  a tick sees*.
- **Corridor rebuild timing.** Once the car settles (t > 17 s) the open-loop
  command is a 1 s sawtooth, one tooth per corridor rebuild
  (`plots/mpc_commands_overlay.png`).
  - The rebuild cadence is 1.0–1.1 s, because the 1 s period is quantised to
    100 ms ticks.
  - Runs drift apart: run 2 rebuilds at 1.66 s where runs 1 and 3 rebuild at
    1.56 s, and it ends 0.3 s out of phase.
  - The in-phase pair (1–3, with 39 of 40 corridors matched) has steering RMS
    0.019. The out-of-phase pairs have 0.041, and their difference trace is
    the same sawtooth shifted by one tick (`plots/mpc_floor_diff.png`).
  - The floor is therefore **bimodal by mechanism**. A phase-independent
    measure, mean steering over the steady window, agrees to **≤ 0.0008 rad**.
- **ADMM capped at 200 iterations, plus warm-start chaining.** Small input
  differences carry forward from tick to tick, which accounts for the
  remaining per-tick spread and the occasional `solved` ↔ `solved_inaccurate`
  flip.

### Acceptance rule for Jazzy vs Humble, fixed now, before the data exists

The Orin will also produce a **Humble floor** from 3 Humble runs. Let *F* be,
per metric, the worst value over all floor pairs of **both** distros. A
Jazzy-vs-Humble pair passes if:

1. Steering and speed (tick and ZOH) RMS, p95 and max are each ≤ **1.5 × F**.
   - The factor 1.5 is not tuning. The floor is bimodal, so any pair is
     either in or out of rebuild phase, and *F* already contains the
     out-of-phase mode.
   - The extra 0.5 covers the fact that a cross-machine pair mixes Thor's and
     the Orin's different tick-time scheduling. Each floor sees only one
     machine's.
2. Steady-window mean steering differs by ≤ **1.5 × F**. This is the primary
   metric, because it is immune to rebuild phase.
3. Status disagreements per pair are ≤ F's count, and none involves
   infeasible, failed or rejected on one side only. Any that does is listed
   with its inputs and root-caused.
4. Hard-constraint count and horizon length are equal on every matched tick.
   Clamp applied speed is equal on matched events.
5. Frozen inputs, Thor vs Orin: `IDENTICAL` or `WITHIN OSQP TOLERANCE`, with
   statuses equal on all 419 ticks.

### Function-level parity (timing removed): frozen inputs

`mpc_capture_node.py` runs the production node class and parameters, the
latter taken from the production launch's dump. It swaps only the
module-level name `solve_mpc_step` inside `MPC_corr` for a wrapper that
deep-copies the keyword arguments and keeps the results.

All **419** ticks of the capture run are frozen, well over the 50 required
and spread over the whole run (`output/phase3/mpc_frozen_inputs.npz`, 5.6 MB).
Each tick holds `x0`, `last_u`, the warm start, the corridor, weights,
limits, obstacles and boundaries. Encoding is exact (`float.hex`) and uses no
pickle. `run_mpc_frozen.py` re-solves each tick independently with plain
Python + NumPy + SciPy + osqp. It also records the QP handed to OSQP (P, q,
A, l, u, settings) and OSQP's raw primal and dual solution, so a difference
can be placed in the QP construction or in the solve.

`compare_mpc_frozen.py` judges differences against **OSQP's own termination
test**, not bit-equality. Each side's raw primal/dual solution is checked
against the same QP, with OSQP's unscaled primal and dual residual criteria
at `eps_abs = eps_rel = 1e-3`. Validated: the solver's own solutions pass
(worst residual/tolerance 0.82, on tick 61, inherent to that solve), and a
perturbed solution fails (×10–12).

| Comparison | Result |
|---|---|
| Thor re-solve vs live Jazzy node (`mpc_frozen_thor_vs_live.txt`) | **IDENTICAL**: u0, zopt, x_pred, cost, status, iterations and slack bit-equal on 419/419 |
| osqp 1.1.3 vs 1.0.5, and vs 1.1.0, on Thor | **IDENTICAL** on 419/419, QP bit-identical |
| Preview only, not the Orin: Thor vs Python 3.8.10 / NumPy 1.17.4 / SciPy 1.5.4 / osqp 1.0.5 (`f1tenth/focal-l4t-foxy` container + scratch target) | **WITHIN OSQP TOLERANCE.** The QP differs at ULP level on most ticks (q bit-equal on only 25 of 419: NumPy/libm). Steering differs by ≤ 2.5e-12 rad and acceleration by ≤ 3.3e-10 m/s². Status and iterations equal on 419/419. Every solution passes the termination test. |
| **Thor vs Orin** | **PENDING.** `ORIN_MPC_INSTRUCTIONS.md` Part A |

### Sanity only, not a parity verdict: Jazzy replay vs the recorded live Humble commands

| | steering ZOH RMS / max [rad] | speed ZOH RMS [m/s] | steady mean steering: replay / recorded | steady mean speed |
|---|---|---|---|---|
| Jazzy 1/2/3 vs recorded | 0.124–0.126 / 0.29–0.34 | 0.030–0.031 | 0.136 / **0.001** | 0.4984 / 0.4985 |

**The structural offset, explained:**
- Recorded odometry yaw is 0.30 rad at bag start (mid-manoeuvre), peaks at
  about 0.54 rad at t≈10 s while passing the obstacle, and is about 0 from
  t≈17 s (travel heading 0.011 rad).
- The live run anchored `psi_init_corridor` about 13.5 s before recording,
  evidently at about 0, since its steady steering averages 0.001 rad.
- The replay can only anchor at bag t=0.5 s, at **+0.3009 rad**.
- Open loop, the replayed car never turns. So from t≈14 s the MPC steadily
  asks for about +0.135 rad, left toward a reference 0.29 rad off the car's
  heading. That is the whole steering offset; speed agrees to 0.0001 m/s in
  the steady window.
- A missing `d_wall/psi_correction` in the replay (live it may have been
  non-zero) could add a smaller term.

**Neither affects parity: both replay stacks get the same anchor and the same
absent correction.**

---

## Step 5: timing — **INDICATIVE ONLY**

The deployment target is the Orin (12 cores, JetPack 6, Jazzy in Docker), not Thor.

| Source (what it measures) | mean | p50 | p95 | p99 | max [ms] |
|---|---|---|---|---|---|
| **Recorded Humble / Orin, live, loaded**: `solve_dt` (system clock) | **33.7** | 28.6 | 69.3 | **101.0** | **118.4** |
| Thor, live node, idle: `perf_counter` around `solve_mpc_step` | 7.1 | 6.7 | 9.2 | 9.6 | 10.8 |
| Thor, live node, `stress-ng` 14×90% | 8.5 | 8.0 | 11.5 | 18.1 | 25.0 |
| Thor, live node, `stress-ng` 28×100% (2× oversubscribed) | 15.0 | 14.4 | 21.2 | 29.4 | 33.2 |
| Thor, frozen solves, pinned to cores 10,11, idle | 5.3 | 4.7 | 6.7 | 7.3 | 8.2 |
| … same, 14×90% | 5.8 | 5.6 | 7.3 | 10.2 | 13.9 |
| … same, 28×100% | 11.2 | 10.4 | 18.5 | 24.0 | 27.1 |

| Control-loop period (target 100 ms) | mean | p95 | p99 | max | > 110 ms | > 150 ms |
|---|---|---|---|---|---|---|
| **Recorded Humble / Orin, live** (`/mpc/solver_status` stamps) | **112.5** | 196.4 | 247.7 | **309.3** | **150** | **53** |
| Thor replay, idle, runs 1–3 (wall-clock stamps of the per-tick log line) | 100.0 | 104.7–104.8 | 105.3–105.6 | 105.8–107.0 | 0 | 0 |
| Thor replay, 14×90% | 100.0 | 106.0 | 110.7 | 122.9 | 6 | 0 |
| Thor replay, 28×100% | 99.9 | 113.0 | 123.0 | 155.1 | 34 | 1 |

`stress-ng` 0.17.06 ran from a `.deb` unpacked into the session scratchpad,
because there was no sudo. Nothing was installed system-wide.

**What this says, cautiously:**

- **The Orin was overrunning in the recorded Humble run.**
  - Only 376 solves in 42.5 s (about 425 expected).
  - 53 periods over 150 ms; worst 309 ms.
  - The Orin's per-core load in the same bag (`/diagnostics/system_status`,
    41 samples) averaged 78–100% on every core, with cores 3–5 at about
    99–100%. The MPC's own cores 10 and 11 were at 96.7% and 95.0%.
- **This is a pre-existing deployment-load issue, not a Jazzy question.** But
  it is the largest timing risk for the Jazzy-on-Orin-in-Docker target and
  must be re-measured there (Phase 6+).
- **Thor is 5–7× faster per solve** and holds 100 ms ticks even under 2×
  oversubscription. Thor numbers cannot stand in for the Orin.
- **The solve is a small part of the node's CPU.** In the idle capture run,
  419 solves × about 5.3 ms ≈ 2.2 s of solver CPU against about 30 CPU-s for
  the MPC process over 43 s of playback at the production-like 40 Hz clock.
  The process uses about 0.70 of a Thor core (0.90 with the harness's 1 kHz
  `/clock`), so **about 7% of it is the solve.** Where the rest goes
  (executor, `/tf` at about 100 msg/s, `/scan` at 40 Hz, per-tick logging,
  marker publishing) was **not** profiled. That is a decision item, not a
  guess.

---

## Step 6: CPU-pinning proposal for the Orin (recommend only; apply later on the Orin)

The current defaults, from each launch file (and `scripts/check_cpu_pinning.py`):

| Cores | Nodes |
|---|---|
| 0,1 | `ekf_filter_node`, `ekf_global_filter_node` |
| 2 | `slam_toolbox` |
| 2–11 | `ekf_cost_observer` |
| 3 | `foxglove_bridge` |
| 4 | `behavior_executor_node` |
| 5 | `wall_distance`, `obstacle_clearance`, `swept_clearance`, `lidar_front_wall` |
| 6,7 | `detection_3d_node`, `obstacle_projector_node` |
| 8,9 | `yolo_detector_node` |
| 10,11 | `mpc_corr` |

Measured load share, from what exists:
- **EKFs:** from the bag's `ekf_cost_observer`, local 49% and global 45% of a
  core, so about 0.95 of the 2-core pair 0,1. Adequate.
- **MPC:** about 0.70 of a Thor core (Thor ≈ 5–7× the Orin per solve). On the
  Orin, expect **more than one full core**. The recorded 34 ms mean solve plus
  overruns are consistent with that.
- **Everything else:** not measured per node. The only per-core data is the
  bag's: every Orin core 78–100% busy.

**Proposal:**

1. **Keep `mpc_corr` on two dedicated cores (10,11).** It needs more than one
   Orin core and its deadline is the hard one.
2. **Make 10,11 actually exclusive.**
   - At 95–97% busy, while one MPC uses about one core, the pair is shared
     with unpinned processes: the ZED wrapper, `robot_state_publisher`,
     `urg_node`, `ackermann_mux`, the VESC driver, the mission logger /
     `ros2 bag record`, and Foxglove's worker threads.
   - Give every unpinned process a default mask of 0–9. In the Docker target
     the cleanest place for that is the container's entrypoint (`taskset` /
     `--cpuset-cpus` for the whole stack, then widen only `mpc_corr`), not
     per-node launch arguments.
3. **Relieve core 5.** Four nodes share it and it measured 99.8%. Spread them
   over 4 and 5, or move `lidar_front_wall`/`swept_clearance` to whichever of
   2–4 has headroom after measuring.
4. **Consider scheduling priority for `mpc_corr` inside the container.**
   `mpc_corr.launch.py` notes that `nice` is a no-op on the native
   deployment, which lacks `CAP_SYS_NICE`. Docker can grant it
   (`--cap-add SYS_NICE`), which would let the existing `nice` argument, or
   `chrt`, work.
5. **Measure before applying:** `pidstat -t -p <pid> 1` per node and
   `scripts/check_cpu_pinning.py` on the Orin, in the container, under the
   real mission load. Then fix the numbers above.

This proposal is not applied, and cannot be validated on Thor. Thor has 14
cores; cores 12 and 13 are irrelevant on the Orin.

---

## What isn't covered, and where it goes

- **Jazzy vs Humble at replay level** and the **Thor-vs-Orin frozen run**:
  pending, on the Orin (`output/phase3/ORIN_MPC_INSTRUCTIONS.md`). The
  comparison commands are pre-written there, and the acceptance rule above is
  fixed.
- **Step 2, the Orin's osqp version:** pending. Same Orin run (`pip show osqp`
  plus the version recorded by the frozen run).
- **Container parity on the Orin.** Jazzy in Docker on the Orin is the real
  target. Real timing, pinning and load are checked there; phases 6–8 run on
  the Orin in the container.
- **Closed-loop behaviour.** Every replay here is open loop, by design.
  Closed-loop parity is a live or simulation question for later phases.
- **Branches not exercised by this mission:** `wall_turn` (wall tracker,
  `/scan`, `base_link←laser` TF, `/perception/front_distance`),
  `goal_distance` / `goal_pose` / `goal_object` / `goal_turn`, and the
  convex-polytope boundary path (off by default). The frozen-input harness
  can take captures from any future bag that exercises them.
- **Local EKF + IMU:** still open from Phase 1, still Phase 6.

---

## Commits

```
19c8703 jazzy/p3: goal injection for MPC replays — the bag never recorded /mpc/goal_drive
48108cd jazzy/p3: mpc and mpc_capture replay layers — Humble/Jazzy MPC parity needs the production launch under sim time
210f380 jazzy/p3: frozen-input MPC solver test (Thor vs Orin) — separate solver numerics from replay timing
684ef1f jazzy/p3: MPC replay metrics, timing summary and plots — Jazzy noise floor and indicative timing
(this commit) jazzy/p3: Phase 3 report and Orin MPC instructions — GO-WITH-NOTES, Orin comparison pending
```

All are local and unpushed. No production code was changed.

---

## Decisions for Andreas

1. **Run `output/phase3/ORIN_MPC_INSTRUCTIONS.md` on the Orin** (about 5 min of
   machine time, plus a 250 MB copy). It settles the Thor-vs-Orin frozen test,
   the Orin's osqp version (Step 2) and the Humble side of the replay
   comparison in one go. Same pattern as the pending Phase 2 boundary run.
2. **Backlog, not this phase: add the missing MPC inputs to the mission
   logger.** `mission_logger_node._DEFAULT_TOPICS` lacks:
   - `/mpc/goal_drive` (the goal of every drive move, the reason this phase
     had to inject one)
   - `/imu`
   - `/joint_states`
   - `/perception/front_distance`
   - `/perception/d_wall/psi_correction` (affects straight-drive heading)
   - `/test_campaign/logger_status`
   - `/mpc/status` (OSQP iterations)

   Without these, no future bag can be replayed through the MPC faithfully.
   Also worth recording: the mission-start window. This bag began about 13.5 s
   after the move started, so the reference heading was unrecoverable.
3. **The recorded Orin run was overrunning the 100 ms MPC deadline** (mean
   112 ms period, 53 periods over 150 ms, worst 309 ms; mean solve 34 ms, p99
   101 ms) with every core 78–100% busy. This is pre-existing and not
   Jazzy-related, but it is the main risk for the Docker-on-Orin target.
   Treat the Step 6 pinning proposal as input to that measurement, not as a
   fix.
4. **The MPC process spends only about 7% of its CPU in the solver.**
   Profiling it on the Orin (`py-spy top --pid …`) would show where the rest
   goes before anyone tunes pinning around it. Not done here: no profiling
   tooling on Thor, and any change it suggests would be tuning, out of scope
   for a migration.
5. **osqp pinning.** The Dockerfiles install osqp unpinned. 1.0.5, 1.1.0 and
   1.1.3 are bit-identical on this data, but a future 2.x could change
   defaults: 1.x already added `check_dualgap`. Pinning `osqp==<the Orin's
   version>` in the Jazzy image once Step 2 is resolved would make solver
   behaviour reproducible. Recommend; your call.
6. **Harness caveat to keep in mind.** Replay `/mpc/solver_status.solve_dt_sec`
   is always 0 under sim time, by design of the executor, not a bug. Use the
   capture/frozen `perf_counter` timings instead (documented in
   `replay_localization.sh`).
