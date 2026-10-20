# Reverse driving chosen by the MPC — report

Branch `mpc_corr_v2_reverse`, from `origin/campaign-final` at `868ec8a` (the Orin's newer
`campaign-final` / `campaign-final-doors` are not in this base; see `recon.md`).
Everything here was run on **linus** (i7-9700K, Ubuntu 24.04, ROS 2 Jazzy, Python 3.12):
pure-Python solver and closed-loop harness only, no ROS stack, no Gazebo. **Nothing has
run on the car or on the Orin.**

## In one paragraph

With `allow_reverse: true`, every 100 ms tick `mpc_corr` solves the same QP three times:
all forward (A), all reverse (B), reverse for 1 s then forward (C1.0). The three differ
only in a per-stage sign rule on the speed. It scores each plan as **QP cost + a wall-aware
cost-to-go V at the plan's end**. A selector with hysteresis, commitment and a dwell time
then picks one direction, and only the first command of the winner is sent. On the
nominal-model harness this **never reverses in a free corridor**, **reverses to a goal
1.5 m behind** and **turns round for a goal 3.5 m behind**. Near the forward/reverse tie it
changes direction once and does not dither. **It does not get the car through a door**, in
either door scenario (b, c). The reason is structural, explained below. With
`allow_reverse: false` (the default) the controller is bit-identical to `868ec8a` on every
existing closed-loop scenario.

## What was built

| Piece | Where | What it does |
|---|---|---|
| Direction candidates | `mpc_controller/reverse_mpc.py` `build_candidates` | A, B and C_T (`reverse_switch_times_s`, default [1.0, 2.0]). A sign rule starts at the first stage from which a jerk- and acceleration-limited stop (`stop_witness`) already satisfies it. Braking therefore happens inside the prediction, and the witness proves that every candidate sent to OSQP has a feasible speed profile. "v ≤ 0" includes standing still. At today's 0.1 s prediction step the horizon is 2.0 s, so C2.0 is the same as B and is dropped as a duplicate. |
| Per-stage QP pieces | `mpc_solver.py` `stage_spec` | Per-stage speed box and reference. Rear-bumper `w_obs` probes on stages that can move backwards: the front pattern mirrored about the bumpers, at +0.07 / −0.08 / −0.23 m, added to the unchanged front probes. A linear reverse-speed cost through a slack. A terminal target. `stage_spec=None` is today's QP, operation for operation. |
| Own linearization | `nominal_controls` | The previous winner, shifted, for last tick's winner. The model rolled out with the candidate's own speed profile for the others. In both cases speeds are clipped into the candidate's box, so a reverse stage is never linearized around forward motion. |
| Cost-to-go | `mpc_controller/cost_to_go.py` | Backward Dijkstra over a lattice of (x, y, heading, gear): 0.10 m cells, 36 headings, a 4 × 4 m window. Forward and reverse primitives at steering {−max, −max/2, 0, +max/2, +max} using the real limits and the MPC's own model, collision-checked with the car's footprint. Edge cost is length, ×k_rev in reverse, plus a gear-switch cost per direction change. Unreachable states get 50, not ∞. Trilinear query with heading wrap. |
| How V enters the QP | `ReversePlanner._policy_path` | While a goal pose exists (goal_pose, turn and object moves), each candidate's forward terminal pull (w_term to the lookahead, w_psi to psiRef) is replaced by a pull toward where V's optimal policy takes the car from its current pose. The policy is restricted to that candidate's own sign schedule. It steps along collision-checked edges, so a target is never on the far side of a wall. |
| Score | `ReversePlanner._score` | J_QP (true cost, same terms for every candidate) + w_ctg · V(x_N) in the candidate's final gear + the boundary slack the QP bought. |
| Selection | `DirectionSelector` | Candidates are grouped by their **first real motion**. To switch, the other direction must win by more than `reverse_switch_threshold` **and** `reverse_switch_threshold_rel` · \|score\|, after `reverse_min_dwell_s`. A switch made while still moving commits until the car is stopped, unless the old direction wins back by `reverse_commit_break_margin`. A winning C plan counts its switch time down. |
| Corridor behind the car | `extend_corridor_backwards` | Extended straight back by `reverse_corr_back_length_m` along its first tangent. The heading reference is unchanged. |
| Min-speed rule | `floor_moving_speed_signed` | Today's rule, symmetric (see below). |
| Debug | `/mpc/reverse/debug` (std_msgs/String, JSON) | Per candidate: score, J_QP, V, slack, solve time, first stage of the sign rule, terminal target and predicted end. Also the winner, direction, reason, commitment and dropped candidates. The topic is added to the mission logger's topic list. A throttled `REVERSE |` log line carries the same information. |
| Flag | `allow_reverse` (default **false**) | With it false, `MPC_corr._mpc_solve` is `solve_mpc_step`, same call, same arguments. |

### Deviations from the task text, and why

- **The score's speed term.** The speed reference is **±vdes in the direction of travel**,
  and `reverse_w_rev` defaults to 2 · w_v · vdes. With a reference of 0 on reverse stages, a
  *standing* reverse candidate scored about 7.5 better than a standing forward one, and a
  blocked car would have "reversed" for that reason alone. Now standing still costs the same
  in both directions, and reversing is never cheaper than standing on its own account.
- **Direction is the plan's first real motion, not its label.** "C: stand until 1 s, then
  forward" is a forward plan.
- **A plan that never moves does not hold a direction.** Without this, a car that had stopped
  after a manoeuvre deadlocked: standing in "reverse" was cheaper than nothing, and the
  forward plan was better but by less than the threshold. The threshold exists to stop
  dithering between two *moving* plans. Dwell still applies.
- **The winning C plan counts down, and fresh C_T plans are suppressed while it runs.**
  Without this, the switch stays T seconds away for ever (receding-horizon procrastination;
  measured).
- **The corridor is not in the cost-to-go's collision set by default**
  (`reverse_ctg_use_corridor_bounds: false`). The corridor is synthetic, rebuilt from the
  car's pose every second and aimed at the goal. Aimed at a goal behind the car it is a
  U-turn, which excludes exactly the reverse path V exists to find (scenario e becomes
  unreachable). It is a switch.
- **The cost-to-go's wall source** on this base is the boundary half-planes the QP uses (the
  same rows). The harness gives the lattice wall *segments*, and gives the QP the half-planes
  of the segments the car is alongside, so that an open door is not a wall row.

### The min-speed rule, documented

Today `min_moving_speed_mps` (0.4) is enforced in one place only: on **object approaches**,
`MPC_corr` publishes `max(v_cmd, 0.4)` while `object_speed_mode == SPEED_DRIVE`. drive,
turn, goal_distance and goal_pose moves are **not** floored (`stack_params.yaml:1449-1451`).
With `allow_reverse` on, the same rule applies in the selected direction: `max(v_cmd, +0.4)`
forward, `min(v_cmd, −0.4)` in reverse. While the car still moves **against** the selected
direction, it is braking for a direction change and the floor is not applied at all; a
floor there would command a jump straight through zero. Flag off: the original line,
untouched.

## Closed-loop results (nominal model)

Harness: `src/f1tenth_control/mpc_controller/test/reverse_harness.py`. It uses the real
`build_straight_corridor` / `compute_local_target` on a stand-in with the **shipped**
corridor size, weights and limits, plus the real `ReversePlanner` and `CostToGo`. The plant
is the solver's own model and tracks the published speed in one tick, with the /drive
clamp applied (reverse limit = `reverse_v_rev_max_mps` 0.3). Plots and GIFs are next to this
file (`<scenario>.png`, `<scenario>.gif`; `__flag_off.png` is the same run with the flag
off). The numbers are in `metrics.json`. Regenerate with
`python3 test/reverse_harness.py --out ../../../output/reverse` from the package.

| Scenario | Set-up | Reached | Time to goal | Reverse | Direction changes | Min body–wall | Verdict |
|---|---|---|---|---|---|---|---|
| a — free corridor, pose goal | 2 m corridor, goal 5 m ahead, V active | yes | 10.7 s (flag off 10.4) | 0 m | **0** | 0.86 m | ✅ never reverses; min cruise command 0.41 vs 0.50 flag off (one ~0.1 m/s dip near the end of the run) |
| a2 — free corridor, goal_distance | same corridor, 5 m, no goal pose → no V | yes | 10.5 s (= flag off) | 0 m | **0** | 0.86 m | ✅ path identical to flag off within 1 cm |
| b — door seen late | 1.3 m hall, 0.9 m door; goal 1 m into the room appears with base_link at x 2.2 | **no** | — | 1.13 m | 1 | 0.34 m | ❌ brakes and reverses 1.1 m (correct), then stops: every forward plan from there predicts a jamb collision |
| c — door seen early | same door, seen at x 0.4 | **no** | — | 1.56 m | **13** | 0.40 m | ❌ dithers in front of the opening |
| d — goal 1.5 m behind | open floor, from rest | yes | 9.4 s | **1.35 m** | **0** | — | ✅ reverses straight to it |
| e — goal 3.5 m behind | open floor, from rest | yes | 26.2 s | **0 m** | **0** | — | ✅ turns instead of a 3.5 m reverse (wide loop, 12.6 m of path) |
| f — near tie | goal 2.6 m behind, 1 cm / 0.5° odometry noise | yes | 24.1 s | 0.11 m | **1** | — | ✅ at most one change |
| flag off ≡ 868ec8a | all 20 existing closed-loop runs | — | — | — | — | — | ✅ bit-identical (SHA-256 per run, inputs and outputs of every solve) |

No run touched a wall: the smallest body-to-wall distance in any reverse-enabled run is
0.34 m.

### Why the door does not work

The QP and the cost-to-go disagree about collisions at a door.

- **The cost-to-go** checks the whole footprint (bumper to bumper, ±half-width, +0.10 m).
- **The QP's collision model is today's:** three probes on the nose's centre line (plus the
  rear ones on reverse stages) against obstacle disks, and boundary rows on the rear-axle
  point with `car_radius + margin`. It cannot see a body corner meeting a jamb.

The QP's forward plan into the door turns in early. Its predicted end lands on a pose the
lattice knows is a collision, so V there is the unreachable value. Four centimetres further
back, the same plan ends free. The forward score therefore flips between about 7 and about
150 as the car creeps, and the selector follows it: scenario c dithers. In scenario b the
car reverses correctly, but from where it stops no forward plan the QP produces ends free.

The lattice is also **optimistic about steering**. Its primitives change steering
instantly, while the QP is held to 0.5 rad/s. At 0.5 m/s a full steering change costs about
0.3 m of path, and in a 1.3 m hall with a 0.9 m door and R_min ≈ 0.96 m there is no 0.3 m
to spare.

**Tried, measured, all worse or no better** (each is still in the code, as a switch that
is off):

| Variant | Effect |
|---|---|
| A local gradient model of V as the terminal cost | V's gradient at one point says nothing about a door 0.5 m away; the plan swerved into walls |
| A nearest-low-V terminal target | picked targets on the far side of a wall |
| Per-stage tracking of V's policy path (`w_path`) | d turned into a dithering 0.5 m reverse; doors unchanged |
| Speed cap on tight path stretches (`ctg_v_tight`) | broke the free corridor (it changed the score, see the next row) |
| Scoring without the speed-reference term | broke the free corridor |
| Dropping the corridor's stage costs when V is active | d turned round instead of reversing |
| Per-candidate warm starts (instead of the task's rule) | neutral on a/d/e; less dithering at the door, still no entry |
| Wider hall (1.6 m) or door (1.0 m), higher thresholds, 2 s dwell | no entry |

**What would fix it, in order of what I would try:**

1. **Footprint-aware QP rows.** The four body corners against the boundary half-planes and
   jamb disks, linearized in heading like everything else in RTI. Then the QP and the
   lattice agree on what a collision is.
2. **A steering-rate-aware lattice.** Steering becomes part of the state, or primitives
   ramp it.
3. **Or don't let the MPC plan the door at all.** Execute the lattice path with a tracking
   controller in tight spaces. This is the separate-manoeuvre direction the addendum's
   Part A takes.

## Timing (linus, i7-9700K, single-threaded Python)

Each tick includes the cost-to-go update check and every candidate.

| Run | Tick p50 | Tick p95 | Tick max | Per-candidate QP p50 / p95 | V rebuilds (max) |
|---|---|---|---|---|---|
| flag off (today) | 4.9 ms | 5.1 ms | 5.5 ms | — | — |
| a — free corridor, V on | 14.9 ms | 22.1 ms | 124 ms¹ | 5.7 / 6.4 ms | 3 (102 ms) |
| a2 — goal_distance, no V | 11.9 ms | 13.7 ms | 18 ms | 5.4 / 6.1 ms | 0 |
| b / c — door (3–4 candidates) | 30 ms | 33 ms | 89 ms¹ | 9.1 / 10.6 ms | 3 (57 ms) |
| d / e / f — open floor | 14–19 ms | 21 ms | 125 ms¹ | 5.5 / 6.2 ms | 1–9 (111 ms) |

¹ The tick on which the cost-to-go is rebuilt. The build takes 40–60 ms with walls and
90–110 ms on an open floor (more free states, more edges).

- **Budget 100 ms at 10 Hz:** inside it on linus except the rebuild ticks on an open floor.
  The **Orin was not measured.** One RTI solve took 7.8 ms p50 here. A Python QP on the
  Orin's A78AE cores is plausibly 3–4× slower (an estimate, not a measurement), which puts
  3–4 candidates at roughly 70–150 ms. That is over budget at the top end.
- **If the Orin is over:**
  - Drop the C candidates first (`reverse_switch_times_s: []`, leaving A and B; a winning C
    plan's countdown then never starts). They cost one QP each, and no scenario here needed
    them to win.
  - Move the cost-to-go build off the control thread. I recommend a **C++ (or Rust)
    implementation** of the lattice build rather than a coarser lattice: the build is a
    textbook Dijkstra over about 115 k states and 1 M edges, an order of magnitude cheaper
    compiled. Halving the resolution would bring back the aliasing problems listed in
    commit 3.

## Proposed defaults

| Parameter | Default | Evidence |
|---|---|---|
| `reverse_ctg_k_rev` | **3.0** | With a gear switch of 1.0, the forward/reverse crossover for a goal straight behind on open floor is at about **2.7 m** (near-tie scan in `metrics.json`): d (1.5 m) reverses and e (3.5 m) turns. Lower k_rev moves the crossover farther, i.e. longer blind reverses. |
| `reverse_ctg_gear_switch_cost` | **1.0** | One forward metre per direction change. It makes a short shunt cost something without forbidding it. The lattice then prefers one clean reverse over several short ones. |
| `reverse_switch_threshold` | **1.0** (+ `reverse_switch_threshold_rel` **0.1**) | Free corridor and near-tie: 0 and 1 changes. Raising it to 5.0 did not help at the door (the dither there is not noise, it is the collision-model disagreement above). |
| `reverse_commit_break_margin` | 3.0 | larger than the threshold, as asked |
| `reverse_min_dwell_s` | 1.0 | |
| `reverse_v_rev_max_mps` | 0.3 | lower than forward (0.5); the rear is blind |
| `reverse_w_rev` | 4.3 | = 2 · w_v · vdes (see "Deviations") |
| `reverse_w_ctg` | 3.0 | d reverses, a never does |

## Open risks

- **The rear is blind** (`recon.md` §5). The LiDAR's rear 90° is unscanned and its apex is
  0.39 m ahead of the rear bumper. The camera faces forward. Nothing checks the rear before
  or during a reverse. The rear `w_obs` probes only see obstacles that something reported,
  and nothing reports anything behind the car. The SLAM map behind the car only holds what
  was seen on the way in.
- **The VESC chain needs a config change to reverse at all.** `max_reverse_speed_mps` is 0.0.
  With `allow_reverse: true` and that value, the node **disables** reverse at start and says
  so. Raising it is an operator decision, and the tests that pin 0.0 move with it
  (`recon.md` §4). The ESC's behaviour at a direction change (brake, current spike) is in
  the VESC firmware, not in this repo, and is unmeasured. The reverse velocity-correction
  curve is an unmeasured mirror of the forward one.
- **Nominal model only.** The plant is the solver's own model: no actuator lag, no slip, no
  ESC deadband (0.11 m/s telemetry deadband in both directions), perfect odometry except in
  scenario f.
- **The wall source on this base** is costmap half-planes (front/left/right, nearest cell).
  A door in a side wall is a wall to them. The Orin's wall rows (A1) are half-planes too.
  Door entry on the car needs segment walls in the cost-to-go (the harness's model) and the
  footprint rows above.
- **Goals beyond the window** (more than about 2.8 m from the car at 4 m) are reached through
  the window border with a straight-line remainder. That approximation is visible as a step
  in the near-tie scan at 2.8 m, and it is why e's turn is a wide 12.6 m loop and not a tight
  one.
- **Cost-to-go build time** is spent on the control thread, once per rebuild (see Timing).
- **Not run with SLSQP.** `stage_spec` requires the RTI solver; `use_rti_solver: false` with
  `allow_reverse: true` raises on the first tick.

## Reproducing, dependencies

- Python packages on linus: numpy 1.26.4, scipy 1.11.4, osqp 1.1.3, matplotlib 3.6.3 +
  Pillow 10.2.0 (plots and GIFs only). **Nothing was installed.**
- APIs used beyond what `mpc_controller` already needs:
  - `scipy.sparse.csgraph.dijkstra`, with a super-source rather than `min_only=`, so any
    scipy ≥ 0.11 works;
  - `scipy.ndimage.correlate` / `uniform_filter`;
  - `scipy.sparse.coo_matrix`;
  - the same `osqp` 1.x calls `_solve_rti` already makes.
- **Orin package versions are not recorded in the repo.** `docker/Dockerfile.jetson-thor` pins
  `numpy<2`, `scipy>=1.13` (that is the Thor image), and the Orin runs natively, so I could
  not check them. Every call above exists in scipy 1.8 (Ubuntu 22.04's) and later.
- **Build:** `MPC_corr.py` imports `f1tenth_messages`, so the worktree was built with
  `colcon build --symlink-install --packages-up-to mpc_controller` (also builds
  f1tenth_messages, f1tenth_params, f1tenth_description and the test-only deps).
- **Tests**, from `src/f1tenth_control/mpc_controller` with the install sourced:
  - `python3 -m pytest test -q` gives **19 failed / 801 passed / 1 skipped / 2 xfailed**. The
    19 failures are the **same 19 that fail at 868ec8a** (stale config-coupled tests:
    steering limits, prediction step, weight set, …). No new failures.
  - New tests:
    - `test_reverse_candidates.py`: witness, schedules, QP sign rules, nominal signs,
      reverse kinematics and Jacobian, selector, floor, back extension, planner;
    - `test_cost_to_go.py`;
    - `test_reverse_flag_off_identity.py`: the pin;
    - `test_reverse_scenarios.py`: a, a2, d, e, f, plus b/c as strict xfails.
- **Lint** (`ament_flake8` / `ament_pep257`, package-wide):
  - mpc_controller: flake8 1157 → 1156, pep257 1197 → 1197;
  - f1tenth_params: 100 → 100, 129 → 129;
  - f1tenth_logger: the one changed file is 40 → 40.

## Next step

Decide between the two directions before anything runs on the car.

1. **Keep the MPC choosing the direction.** Add footprint-aware QP rows (body corners against
   wall half-planes and jamb disks) and segment walls in the cost-to-go. Re-run b and c; they
   are wired as strict xfails, so they will report it the moment they pass.
2. **Go with the addendum's separate reverse manoeuvre (Part A),** which avoids comparing
   costs altogether, and keep this branch's flag off.

Either way, the first car session needs `max_reverse_speed_mps > 0` and a person behind the
car.
