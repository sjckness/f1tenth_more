# First powered session: `d_wall` bringup checklist

Ordered procedure for the first powered session after the `d_wall` pass. The
LiDAR was down for the whole of that pass, so **nothing in it has run on the
car** — see `src/f1tenth_perception/README.md`'s UNVALIDATED section for the
full list of what that means.

## Read this first

**Do not reorder the stages.** Stage 2 before Stage 4 before Stage 5 is the
whole point of the document. Stage 2 is the physical left/right sign check: it
is the one check no offline test can perform, and the one that prevents a car
steering confidently into glass. **Nothing downstream runs until it passes.**

**Every stage records a bag.** A powered session that produces no data is a
wasted one, and each bag makes the next round of work offline-testable. The
stack archives automatically per mission
(`~/f1tenth_archive/complete/<run>/bag/`), but Stages 0–3 are not missions, so
those bags must be started by hand.

**The bag that matters most** is any bag containing `/mpc/wall_track`. Zero bags
in the archive have it — all 14 existing `wall_turn` runs predate the tracker —
and without one the phase machine's silence logic cannot be tested offline at
all. Stage 4 captures it.

**Recording command** for the hand-started stages. Record more than seems
necessary; the expensive part is the session, not the disk:

```bash
ros2 bag record -o ~/f1tenth_archive/manual/$(date +%Y-%m-%dT%H-%M-%S)_stage<N> \
  /scan /tf /tf_static /odom /odometry/filtered /ekf_global/odometry/filtered \
  /perception/d_wall /perception/d_wall/segment \
  /perception/d_wall/psi_correction /perception/d_wall/gate_margin \
  /perception/d_wall/swept_arc /perception/swept_clearance \
  /perception/swept_clearance/lidar /perception/swept_clearance/steering \
  /perception/lidar_front_wall /perception/lidar_front_wall_virtual \
  /perception/front_distance /mpc/wall_track /mpc/hold /mpc/goal_drive \
  /mpc/corridor_markers /mpc/solver_status /drive /ackermann_drive \
  /mission/status /behavior/tree_status /diagnostics
```

**Reading topics under the Discovery Server.** `ros2 node list` and
`ros2 topic list` are blind in this workspace's DDS configuration — an empty
listing means nothing. Probe with a short `rclpy` script, or read the
supervisor's per-component logs in `~/.ros/log/component_supervisor/`.

---

## Stage 0 — Power, no autonomy

**Car on blocks, wheels off the ground, e-stop in hand.** No mission loaded.

1. Bring up `urg_node` and `wall_distance_node` **only**:
   ```bash
   ros2 launch f1tenth_perception lidar.launch.py
   ros2 launch f1tenth_perception wall_distance.launch.py
   ```
   Start the bag before either.
2. **Confirm the topics appear**, with the right shapes:
   - `/perception/d_wall` at ~10 Hz, `valid` either way
   - `/perception/d_wall/segment`, `/psi_correction`, `/gate_margin`,
     `/swept_arc` all publishing
   - `psi_correction` is **exactly 0.0** on every message
3. **Confirm the phase machine sits in `UNKNOWN`.** `/mpc/wall_track` is silent
   (mpc_corr is not running), and `phase` must read 0 on every message. The node
   logs `PHASE | ...` on every transition; there should be none. A `CORRIDOR`
   here would mean silence was read as a turn exit, which is the single failure
   the phase machine is built to prevent.
4. **Confirm the new components did not restart `urg_node`.** This is the
   placement rule two test files already guard, checked live:
   ```bash
   grep -c 'urg_node' ~/.ros/log/component_supervisor/*.log
   ```
   `urg_node`'s PID must be unchanged across a deliberate
   `RestartComponent{name: wall_distance}` and again across
   `{name: swept_clearance}`. Note its PID before and after.
5. **Register that `swept_clearance_node` came up at all.** It has never run.
   Confirm `/perception/swept_clearance` publishes and that
   `/perception/swept_clearance/steering` is NaN until a steering command
   exists. Remember its value is **arc length to body contact**, not a gap.
6. **Measure CPU**, because the offline number is a benchmark loop and not a
   node. All three of `wall_distance_node`, `lidar_front_wall_node` and
   `swept_clearance_node` are pinned to core 5:
   ```bash
   top -H -p $(pgrep -d, -f 'wall_distance_node|swept_clearance_node|lidar_front_wall_node')
   ```
   Offline the per-tick path measured mean 26.8 ms / p95 32.8 ms against a
   100 ms budget (26.8% of one core). **Three nodes on core 5 with one of them
   at 27% is the thing to check.** If core 5 saturates, move
   `swept_clearance_cpu_affinity` off it — never onto core 4, which hosts the
   e-stop.

**Do not proceed if the phase is anything but `UNKNOWN`, or if `urg_node`'s PID
changed.**

---

## Stage 1 — Static ground truth

**Still on blocks.** Park at a tape-measured distance from the pane, wall to one
side.

1. Tape-measure the perpendicular distance from **`base_link`** — the rear axle,
   on the ground, per `base.xacro` — to the pane. Note that the longitudinal
   origin has a known unresolved 0.1525 m ambiguity (the URDF joints say rear
   axle; three other files say between the axles). It does **not** affect a
   lateral distance, which is why `d_wall` is the safer quantity, but record
   which you measured from.
2. Assert `|d_wall|` matches the tape **to within the deadband**, which is
   `max(0.02, 2.0 × fit_rms)`. Log `fit_rms` alongside; offline it ran 3.6–7.2 mm
   on archive scans and 11–20 mm on the glass fixtures, so expect a deadband
   near the 0.02 m floor.
3. **Repeat at three distances** — suggest 0.6 m (the `d_ref` default), 1.1 m
   (the one range glass was ever measured at) and 2.5 m.
4. Record `provenance` at each. `GEOMETRIC` is expected against an opaque wall;
   `GLASS_CONFIRMED` only against the pane.

If `|d_wall|` is off by a **scale factor**, suspect the laser transform
(`0.12, 0, 0.20` are self-described ruler-measured placeholders, not a
calibration). If it is off by a **constant**, suspect the `base_link` origin
ambiguity.

---

## Stage 2 — Physical sign check — **THE GATE**

**Still on blocks. Nothing downstream runs until this passes.**

The unit tests assert internal consistency with the convention
`WallDistance.msg` documents, on both sides, including invariance to the fit's
arbitrary normal flip. **They cannot catch the convention itself being
backwards.** If it is, the correction is smooth, in-saturation, and steers into
the wall.

1. Park with the wall **on the car's left**. Confirm `d_wall > 0`.
2. Reposition with the wall **on the car's right**. Confirm `d_wall < 0`.
   *Reposition the car, do not rely on rotating it 180° in place* — a rotation
   changes yaw, and yaw is an input to the sign, so a 180° rotation tests less
   than it looks like it does. Do both if time allows.
3. At each position, put the wall **closer than `d_ref` (0.60 m)** and read the
   logged `psi_correction`. Force the correction to be computed but not applied
   for this stage — `wall_distance_max_psi_correction` stays at its default so
   the value is non-zero, and `corr_d_wall_correction_enable:=false` in mpc_corr
   (which is not running yet anyway):

   | wall side | `d_wall` | too close ⇒ expected `psi_correction` |
   |---|---|---|
   | left  | `> 0` | **negative** (steers right, away) |
   | right | `< 0` | **positive** (steers left, away) |

   And with the wall **further than `d_ref`**, both signs invert: toward the
   wall. Check that too — a law that only ever pushes away is a law with a
   missing sign.
4. **Both cases must point AWAY from the wall when too close.** If either does
   not, stop. The convention is inverted; fix `signed_wall_offset` or
   `psi_correction` in `wall_distance.py`, re-run
   `test_wall_distance.py::TestSignOnBothSides`, and repeat this stage.

Record a bag of each position and each distance. This bag is the permanent
record of the convention.

---

## Stage 3 — Range sweep, same session

**Still on blocks, pane set up, car powered.** This is the deferred glass
question, and the cheapest time to answer it is while the rig is already
standing. Every glass number in the stack comes from one session at 1.10 m; the
signal budget at 4 m is ~13× smaller and the pane may vanish entirely — which
matters because the `wall_turn` commit distance is around 4 m.

1. **Perpendicular to a clear pane at 1.1 (control), 2, 3, 4, 5 and 6 m.** At
   each: record ≥200 scans, and log valid-return fraction by incidence band
   (0–5, 10–20, 20–30, 30–40, 40–50, 50–60°), median range per band, and
   normalised intensity peak/median within ±20°.
2. **Before/after a wipe at one range** — 3 m is the interesting one. The 1.1 m
   result is credited partly to surface contamination, so a clean pane may be
   *less* visible. Record both.
3. **An open doorway at 2, 3 and 4 m, as the discriminator control.** This is
   what tests whether `glass_see_through_max_gap_m` (0.30) still separates
   transparency from an opening at range: real transparency bands measured 4 cm
   while a 0.9 m doorway measured 0.9 m, and that separation is only known at
   1.1 m.
4. Also record **an ordinary opaque wall** at 2 and 4 m, to check
   `wall_distance_geom_*` on a surface that is not the one archive corridor
   those gates were checked against.

Everything here is offline-analysable afterwards. Nothing in this stage needs
the wheels on the ground, so do it before them.

---

## Stage 4 — Observe a real turn, correction DISABLED

**Wheels on the ground. Lowest speed. E-stop in hand. This is the bag the whole
offline story is missing.**

1. **Force the correction to zero** at both ends, belt and braces:
   - `wall_distance_max_psi_correction: 0.0` (the node publishes 0.0)
   - `corr_d_wall_correction_enable: false` (mpc_corr applies nothing)

   The subscription in mpc_corr is unconditional on purpose, so the trace is on
   the wire and in the bag either way.
2. Bring up the full stack. Run `wall_turn.json` (terminal turn — the existing
   mission) at the lowest speed the mission will accept.
3. **Record everything.** Verify afterwards, offline, from the bag:
   - `/mpc/wall_track` is present at 10 Hz through the turn, with `psi_commit`
     NaN before commit and finite after. **This is the first bag ever to contain
     this topic.**
   - the phase transitions, and their timing against the real tick jitter
   - the real silence pattern at exit: a terminal turn ends with `/mpc/hold` and
     `drive_cmd` is never cleared, so the topic just goes quiet
   - `track_id` continuity, and **how often `coast_cap_first_hit` fires**
   - `provenance` mix: how often the wall is genuinely re-observed through the
     turn versus coasted
4. **Then run `wall_turn_then_straight.json`** — the mission written for this
   pass, with a straight move after the turn. It is the only mission that can
   produce a post-exit corridor at all (every other `wall_turn` mission sets
   `terminal: true` on the turn, and a terminal move publishes `/mpc/hold`, which
   returns from `control_loop` before the corridor rebuild). Confirm the phase
   actually reaches `CORRIDOR`, and that `psi_correction` is still 0.0.

**This bag settles the open design question** in
`src/f1tenth_perception/README.md` §4: replayed offline, one 90° turn produced 6
`track_id`s at the shipping coast caps, and each cap firing clears the phase
machine's "valid track held throughout" condition — so `CORRIDOR` may be
unreachable in practice. Analyse this bag before deciding whether to leave that,
relax the condition to "same `track_id` at exit as at commit", or raise
`max_coast_yaw`. **Do not go to Stage 5 until that decision is made.**

Feed the bag through `tools/wall_distance_replay.py` too — with a real
`/mpc/wall_track` present, the script's synthesised feed can be checked against
the recorded one, which is the only way to find out how wrong the synthesis was.

---

## Stage 5 — Correction at reduced authority

**Only after Stage 4's bag has been analysed AND Stage 2 has passed.**

1. `wall_distance_max_psi_correction: 0.05` (≈2.9°, a quarter of the default).
2. `corr_d_wall_correction_enable: true`.
3. Lowest speed. Short run. **Finger on the e-stop. Someone watching the wall
   side, positioned to call it.**
4. Run `wall_turn_then_straight.json`. Watch for:
   - **the direction of the first correction.** If the car moves *toward* the
     wall when it started too close, stop immediately — Stage 2 passed on a
     static reading and something between there and here has inverted it.
   - **oscillation.** The law is first-order and unconditionally stable *in
     theory*; `w_corr` is live at 1.25 and `w_du_delta` at 3.00 and neither has
     been measured against a non-zero heading bias on this geometry. Oscillation
     means the corridor rebuild cadence (1 Hz) is interacting with the
     correction, and `max_psi_rate` is the first thing to lower.
   - **convergence length.** `convergence_length_m` 3.0 predicts the lateral
     error washing out over ~3 m of travel. Measure it. Do not tune `k`
     directly; change the length and let `k` follow.
5. Record a bag. Compare measured convergence against the 3.0 m prediction.

---

## Stage 6 — Full authority

**Only once Stage 5 shows convergence without oscillation.**

1. `wall_distance_max_psi_correction` back to 0.20.
2. Same mission, same lowest speed first, then step the speed up.
3. Record a bag at each speed. The closed-loop behaviour in **space** should be
   speed-independent — that is the whole point of deriving `k` from a
   convergence length — so a run that converges over 3 m at 0.3 m/s and over 5 m
   at 0.5 m/s means the law is not doing what it claims and something upstream is
   speed-coupled.

---

## Also worth doing while the car is powered

Cheap, and each one closes something currently marked unvalidated or
contradictory elsewhere in the repo.

- **Laser transform.** `0.12, 0, 0.20` with `roll=pitch=0` is self-described as
  "approximate, ruler-measured placeholders, NOT a real calibration". Measure
  it. A pitch error puts the scan plane off horizontal, which biases every
  range at long distance — exactly where Stage 3 is looking.
- **`base_link` longitudinal origin.** Rear axle (URDF joints) vs between the
  axles (three docstrings), a 0.1525 m disagreement that propagates into every
  bumper-referenced distance. One tape measure settles it; then either fix
  `swept_clearance_rear_axle_x_m` or fix the docstrings.
- **Wheelbase.** 0.305 (runtime) vs 0.325 (URDF) vs 0.3302 (F1TENTH spec), and
  the repo calls 0.305 "unmeasured on this car". It sets `R_min` and therefore
  the whole `wall_turn` commit rule.
- **The odometry calibration itself** — distance 17–20% short, gyro yaw gain
  0.93–1.09. It is the reason the coast caps are as tight as they are, and
  fixing it is what would let them be raised.
- **`corridor_heading_return`.** Its `stack_params.yaml` default is `true` while
  its own description says "false (default)" and
  `test_corridor_direction_recovery.py::test_the_default_geometry_freezes_both_ends`
  asserts `false`. That test fails today, on `main` as well as on this branch.
  One of the three is wrong and it is a live corridor-geometry behaviour.
