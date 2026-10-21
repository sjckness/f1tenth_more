# Why the car drives 0.23 m/s when the mission asks for 0.5

Diagnosis only — nothing in this pass was changed, tuned or committed outside
this folder. 2026-10-21, branch `campaign-final`, commit `3feae478`
(Orin, Humble).

**The short version.** There is no speed cap anywhere in the picture. The MPC
plans 0.5 m/s and reaches it inside its own horizon on 168 of 171 ticks. What
reaches the car is not the plan: `MPC_corr.py:5485` publishes
`v_cmd = v_measured + a_cmd * ts`, i.e. the *measured* speed plus one
acceleration step. The car then returns only ~76 % of whatever it is
commanded. Those two facts form a closed loop whose fixed point is

```
v_cmd* = a_cmd * ts / (1 - r) = 0.0855 / (1 - 0.758) = 0.353 m/s
```

against a measured 0.75 * 0.353 = 0.25-0.27 m/s. Both observed numbers
(command 0.33, odometry 0.25) are that fixed point. The loop has no integral
action, so it cannot climb to 0.5 no matter what the solver wants.

---

## 0. Data

### The run that was asked for: M002 R028

```
first_test_campaign/M02_approach_wall/P002-R028-20261021T143550
```

| | |
|---|---|
| start | 2026-10-21T14:35:50.340668 |
| duration | 46.44 s **of which 2.07 s driving** |
| mission | `plan.json`: one move, `drive.mode=straight`, **`drive.speed=0.5`**, stop on `front_clearance` 2.0 m, debounce 3, `timeout_sec` 65 |
| plan id | `llm_2b356aac445f` (qwen2.5-3b-instruct, 1 call, 24.26 s) |
| outcome | `aborted` — "aborted via /mission/abort_mission" |
| path length | **0.320 m** / max speed 0.216 m/s / max acc 3.82 m/s² |
| git commit | `3feae4783d6664f386d938c5867eb9d8f9d6db4c` (= HEAD, clean on the relevant files) |

Its event timeline is why it cannot carry this analysis on its own:

```
t= 0.005  test_start
t=32.323  mission_loaded      countdown_s = 0.0
t=42.173  mission_started     countdown_s = 0.0
t=44.368  mission_aborted
t=46.445  test_end
```

**There is no bag for R028.** It ran with `enable_mission_logger` false, so
`mission_logger_node` never recorded, and the campaign recorder's own files are
all the evidence there is:

| file | rows | content |
|---|---|---|
| `commands.csv` | 484 | `/drive` only: t, cmd_speed, cmd_steer (throttle/brake columns empty) |
| `kinematics.csv` | 969 | pose, yaw, vx, speed, ax from the pose source; corridor/obstacle clearance |
| `mpc.csv` | 22 | status, solve_time_ms, cost, iterations |
| `horizon.jsonl` | 22 | predicted horizon |
| `corridors.jsonl` / `corridor_debug.jsonl` | 3 / ~1600 | corridor geometry |
| `imu.csv`, `tracks.jsonl`, `events.jsonl`, `meta.json`, `plan.json`, `llm_calls.*` | | |

No ERPM, no VESC telemetry, no battery voltage, no mux topic, no TF.

What R028 *does* show, over its 2.07 s (21 commands):

```
cmd_speed  0.106 0.117 0.131 0.227 0.274 0.125 0.288 0.222 0.258 0.261 0.291
           0.259 0.245 0.250 0.217 0.221 0.280 0.280 0.272 0.269 0.279 0.281
measured   0.11-0.22, max 0.2164, with six exact zeros interleaved
```

Consistent with the finding below, far too short for a steady state, and the
run never got past its acceleration transient before it was aborted.

### The run the numbers come from: the 14:20/14:25 archive run

```
~/f1tenth_archive/complete/2026-10-21T12-25-22_mission-20261021T142038-bf7938.mission
```

Same commit `3feae478`, same session, same stack, **same mission**
(`llm_2b356aac445f`, `move_0_straight`, straight at 0.5, stop on
front_clearance 2.0 m) — and it completed instead of being aborted:
`stop_reason = stop_condition:front_clearance`, `duration_s = 18.44`. Mission
window 12:25:34.852–12:25:53.716 UTC (14:25:34–14:25:53 local; the mission
*file* is named for 14:20:38, when it was written).

Bag: 154 MB, 8962 messages, 18.76 s. Topics that matter here:

| topic | msgs | rate |
|---|---|---|
| `/odom` | 886 | 47 Hz |
| `/odometry/filtered` | 354 | 19 Hz |
| `/ekf_global/odometry/filtered` | 353 | 19 Hz |
| `/drive` | 188 | 10 Hz |
| `/ackermann_drive` | 187 | 10 Hz |
| `/mpc/solver_status` | 171 | 9 Hz |
| `/costmap/front_clearance` | 187 | 10 Hz |
| `/perception/virtual_wall` | 131 | 7 Hz |
| `/costmap/boundaries` | 176 | |
| `/tf`, `/tf_static` | 880, 5 | |
| `/mission/status` | 3 | |
| `/mpc/drive_clamp` | **0** | |
| `/mpc/goal_pose`, `/mpc/goal_distance`, `/mpc/goal_turn`, `/mpc/goal_object` | **0** | |

Also present: a params snapshot (`*.params.yaml`, a copy of `stack_params.yaml`
taken at record start, birth 14:25:22) and the extract parquet.

### Topics needed for this question that NO bag records

These are the gap in the evidence, and all four are needed to close section 2
properly:

- **`/commands/motor/speed`** — the commanded ERPM out of `ackermann_to_vesc`.
- **`/sensors/core`** — measured ERPM, **battery voltage**, motor current,
  duty cycle. Battery voltage is therefore unknown for this run; the
  "low voltage saturating the speed loop" hypothesis can be neither confirmed
  nor ruled out.
- **`/sensors/servo_position_command`** — the only servo feedback that exists.
- **`/mpc/stop_target`** — whether the stop ramp had a target at all.
- (useful) **`/costmap/unknown_speed_cap`**, **`/mpc/corridor`**, `/mpc/hold`
  at more than one sample.

The ERPM side is *inferred* below instead: `vesc_to_odom` computes `/odom`
directly from the measured ERPM with the same gain `ackermann_to_vesc` used on
the way out, so `/odom` is measured ERPM in m/s units. That makes the inference
tight, but it is still an inference.

**Exact topic list to record on the next car test** — see section 4.

### Source provenance

The run's manifest records commit `3feae478`, `dirty: true`. The files this
report quotes (`MPC_corr.py`, `drive_limits.py`, `vesc.yaml`,
`param_defaults.py`) are **unmodified** against that commit in the working tree
today, so the line numbers below are the code that ran. The dirty bit at run
time came from `stack_params.yaml` (`obstacle_class_margin_m`,
`unknown_objects_mode`) — neither is on the speed path.

One caveat on the params snapshot: it is copied at record start, so it reflects
the *file*, not necessarily what the running nodes loaded, since nodes read
their parameters at launch.

---

## 1. Where 0.5 becomes 0.35 — **the command law, proved**

### vdes was 0.5, and the solver honoured it

- Bootstrap default `self.vdes = 0.5` — `MPC_corr.py:1701`, kept as
  `self.vdes_default` at `:1711`, restored by `_clear_drive_state` at `:3042`.
- Per-move override, drive moves: `if msg.speed > 0.0: self.vdes = float(msg.speed)`
  — `MPC_corr.py:3158` (goal_drive), the same sentinel at `:3009` for turns.
  R028's and the archive run's `plan.json` both carry `drive.speed = 0.5`, so
  the override set exactly 0.5.
- `stack_params.yaml` has no `vdes` key: 0.5 is the node's own literal.

The solver's predicted speed trajectory, median over the 171 ticks of the
mission window (`pred_v`, 20 stages, from `/mpc/solver_status`):

| stage | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | … | 18 | 19 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| median v | **0.335** | 0.399 | 0.446 | 0.475 | 0.491 | 0.498 | 0.499 | 0.499 | 0.500 | 0.499 | 0.497 |

- `pred_v.max() >= 0.49` on **168 / 171** ticks.
- `pred_v[-1] >= 0.49` on **161 / 171** ticks.
- Stage 0 median 0.3352; `/drive` median 0.3318 — the published command *is*
  stage 0.

So vdes was 0.5, the solver planned 0.5, and only the first stage is 0.35.
**No cap is acting on vdes.** Every candidate is ruled out below.

### The mechanism

`MPC_corr.py:5485`:

```python
v_cmd_now = self.v + a_cmd * self.ts
```

- `self.v` — the measured forward speed, `self.hw_v` from
  `get_odom_topic()` = `/odometry/filtered` with `localization_source: ekf`
  (`MPC_corr.py:1821`, `:2268`, `:3593`).
- `a_cmd` — the solver's first acceleration input.
- `self.ts` = 0.1 s (control period; `mpc_prediction_dt_s` 0.1, N = 20 at
  `MPC_corr.py:846`, so a 2.0 s horizon).

The published speed is therefore an open-loop one-step integration off the
*measurement*. The solver's own speed state never reaches the wheels — only its
acceleration does. Measured, over the steady segment:

```
cmd - v_meas :  mean 0.0847   median 0.0855   std 0.0315   -> a_cmd ~ 0.855 m/s^2
corr(cmd, v_meas) = 0.888
```

That is the signature the work order asked for: `cmd - v_meas` is
**constant**, and the command tracks the measurement rather than driving it.
`a_cmd` ≈ 0.86 m/s² sits well inside `a_max` 3.0 and inside the slew bound
`dAMax * ts` = 0.2 per tick, so nothing is saturated — the solver simply plans a
smooth 0.5 s acceleration to vdes and only its first slice is ever sent.

### The fixed point

With `v_meas = r * v_cmd` (r measured at 0.758, section 2) and
`a_cmd * ts = 0.0855`:

| r | `v_cmd* = a·ts/(1-r)` |
|---|---|
| 0.70 | 0.285 |
| 0.74 | 0.329 |
| **0.758** | **0.353** |
| 0.78 | 0.389 |
| 0.82 | 0.475 |
| 1.00 | unbounded → runs up to the 0.5 clamp |

Observed `/drive` median 0.3318, mean 0.3009, max 0.3959. The loop sits on its
fixed point for the whole run. At r = 1 (a car that delivers what it is asked)
the same law would reach vdes — which is why this has never been visible as a
bug at higher speeds or on a lighter floor.

### Every cap, and whether it was active in this run

| cap | where | value | active? |
|---|---|---|---|
| `/drive` clamp | `drive_limits.clamp_drive_speed`, called from `MPC_corr._publish_drive` (`:4637`); `max_forward_speed_mps` `stack_params.yaml:1453` | 0.5 m/s | **No.** `/mpc/drive_clamp` has 0 messages — it publishes on every clamp, so nothing was ever clamped. |
| stop ramp (caps vdes along the horizon) | `enable_stop_ramp` true, `stop_ramp_a_dec_mps2` 0.6, `MPC_corr.py:3338-3343`, `:3385-3428` | `sqrt(2·0.6·(d_rem−d_final))` | **No.** The ramp reads `get_wall_distance_topic()` = `/perception/virtual_wall`, which was the **CLEAR sentinel 50.0 for the entire mission** except the last ~1 s (first real reading 2.19 m at t≈16.9 s, 2.13 m at the end). Also returns immediately unless `/mpc/stop_target` arrived — not recorded. Decisive either way: an active ramp caps vdes for the whole horizon, and `pred_v` plateaus at 0.500. |
| unknown-object speed cap | `/costmap/unknown_speed_cap`, `MPC_corr.py:1899`, `:3649` | 0.4 m/s within 1.0 m | **No** (and `unknown_objects_mode` was `"off"` in the run-start snapshot). A 0.4 cap would flatten `pred_v` at 0.4; it sits at 0.4996. The last two ticks (t 17.78, 17.89) do show `pred_v[-1]` ≈ 0.40 — that is the approach to the stop, not this cap. |
| `min_moving_speed_mps` floor | `MPC_corr.py:5494-5499` | 0.4 m/s | **Not reached.** Applied only in the object-approach path (`goal_object_odom_xy is not None and object_speed_mode == SPEED_DRIVE`); this was a plain drive move with 0 messages on every `/mpc/goal_object*` topic. (Worth noting: had it applied, it would have *raised* the command to 0.4.) |
| solver box | `MPC_corr.py:1519-1560` | `a ∈ [−2, 3]`, `dA ± 2.0/s`, `v ∈ [0, 3]` | **Not binding.** a_cmd ≈ 0.86. |
| reverse limits | `max_reverse_speed_mps` 0.4, `/mpc/reverse_status` | | Forward run; `vMin` 0 keeps the forward solver out of reverse. |
| curvature / obstacle-proximity speed cap | — | — | **None exists.** Obstacles act through the soft `w_obs` cost and boundary rows, not through a speed limit. |
| BT-side scaling | — | — | **None.** The BT publishes goals; it does not scale speed. |

### Terminal-cost / short-target hypothesis — **ruled out**

The hypothesis was that the lookahead target sits ~0.7 m away, so
`dist/(N·ts)` = 0.7/2.0 = 0.35 would explain the command.

`N·ts·vdes` = 20 × 0.1 × 0.5 = **1.00 m**. Per-tick predicted horizon
geometry from `pred_x`/`pred_y` (171 ticks):

| | mean | p10 | median | p90 | min | max |
|---|---|---|---|---|---|---|
| predicted path length [m] | 0.9079 | 0.8888 | 0.9128 | 0.9251 | 0.7551 | 0.9312 |
| straight-line reach [m] | 0.9072 | 0.8887 | 0.9123 | 0.9240 | 0.7549 | 0.9309 |
| `pred_v[0]` | 0.3242 | 0.2653 | 0.3352 | 0.3655 | 0.1228 | 0.3959 |
| `pred_v[mid]` | 0.4981 | 0.4981 | 0.4996 | 0.5001 | 0.3999 | 0.5015 |
| `pred_v[-1]` | 0.4945 | 0.4915 | 0.4966 | 0.4984 | 0.3974 | 0.5005 |

The horizon covers 0.91 m — 0.09 m short of the 1.00 m a full-speed horizon
would cover, which is exactly the cost of the first stages' low speed, and it
is never near 0.7 m. `dist/(N·ts)` is therefore ≈ 0.46, not 0.35. More
decisively: a target clamped short pulls the **whole** `pred_v` down, and
`pred_v` plateaus at vdes from stage 5 on. The lookahead clamp floor
(`1.25·N·dt·vdes` = 1.25 m at these settings) is above the horizon's reach, so
the clamp is not what shapes it either.

**Ruled out.**

### Weight balance — not the cause

`mpc_w_v` 4.2857 literal → effective 1.50 after the 7/20 stage scaling
(`stack_params.yaml:2085`), against `mpc_w_term` / `mpc_w_corr` / `mpc_w_u_a` /
`mpc_w_du_a`. The test is empirical and it passes: if the weight balance put the
optimum below vdes, `pred_v` would settle below 0.5 — it settles at
0.4996-0.5001, i.e. **the optimum is vdes**, with a far target and the real
weights. The weights are not holding the speed down.

### Offline solver sweep — **could not be run**

The work order asked for a steady-state commanded-v vs lookahead-distance table
(0.5-2.0 m) from the solver itself. **Neither `acados_template` nor `casadi` is
importable in this environment**, so the solver cannot be instantiated offline
here; the table is missing and is the one piece of section 1 that is not closed
by measurement. It would need a machine with the acados Python interface, or a
rebuild of the generated solver.

What stands in for it: the per-tick `pred_v` table above is the solver's own
answer at the lookahead distances that actually occurred (0.76-0.93 m), and it
reaches vdes at every one of them. The sweep would add the 1.0-2.0 m range,
where the horizon is not the binding constraint anyway.

---

## 2. Where 0.35 becomes 0.23 — **the plant, measured; the exact cause, likely**

### The chain, time-aligned on the steady segment (t 0.42-16.49 s)

| hop | median [m/s] | mean [m/s] | n | ratio to previous |
|---|---|---|---|---|
| `/drive` — MPC out | 0.3310 | 0.3136 | 162 | — |
| `/ackermann_drive` — mux out | 0.3305 | 0.3133 | 161 | **0.998** |
| `/odom` vx — from measured ERPM | 0.2504 | 0.2353 | 782 | **0.758** |
| `/odometry/filtered` vx — local EKF | 0.2493 | 0.2359 | 312 | 0.996 |
| `/ekf_global/odometry/filtered` vx | 0.2578 | 0.2338 | 312 | 1.034 |

Whole mission window, for completeness:

| topic | mean | median | max | p90 | exact zeros |
|---|---|---|---|---|---|
| `/drive` | 0.3009 | 0.3318 | 0.3959 | 0.3653 | 8 |
| `/ackermann_drive` | 0.3009 | 0.3318 | 0.3959 | 0.3653 | 8 |
| `/odom` | 0.2329 | 0.2530 | 0.3392 | 0.2906 | **36 / 886** |
| `/odometry/filtered` | 0.2339 | 0.2526 | 0.3392 | 0.2912 | 3 |
| `/ekf_global` | 0.2367 | 0.2607 | 0.2999 | 0.2798 | 0 |

**The entire loss is in one hop**: command → measured ERPM, 0.758. The mux is a
pass-through (0.998, and `ackermann_mux` reports `ok` with the MPC topic
unmasked), and both EKFs agree with `/odom` to within 3 %.

### It is the motor/VESC side, not the conversion

`src/f1tenth_bringup/config/vesc.yaml` has **one `/**` block**, so the same
constants serve both directions:

```
speed_to_erpm_gain:   4516.9
speed_to_erpm_offset: 0.0
erpm_deadband:        500.0        # = 0.111 m/s at this gain
speed_min/speed_max:  ±23250 ERPM  # = ±5.15 m/s, never approached
```

- out: `erpm = 4516.9 · correctVelocity(speed) + 0` (`ackermann_to_vesc.cpp:131-135`),
  **no deadband on commands**;
- back: `speed = (erpm − 0) / 4516.9` (`vesc_ackermann/odom_speed.hpp:27-30`),
  used by `vesc_to_odom` for `/odom`;
- `velocity_correction` is **pass-through** — `vesc.yaml` defines no knots, so
  no low-speed correction is applied in either direction;
- the only `vesc_to_odom_node` overrides are frames, `wheelbase .305`,
  variances, `publish_tf: false` — **no second gain**.

Because one gain is applied symmetrically, a *wrong* gain cancels in the
command/odometry ratio: if the VESC held its setpoint, `/odom` would read the
commanded m/s whatever the gain. It does not. In ERPM:

```
commanded  4516.9 × 0.3310 = 1495 ERPM
measured   4516.9 × 0.2504 = 1131 ERPM      ->  75.6 % of setpoint
```

So the motor runs ~370 ERPM below its speed setpoint, persistently, at a very
low setpoint (1500 ERPM ≈ 0.33 m/s). **Proved that it is the ERPM/plant side
and not the unit conversion.** *Likely* cause, not proved here: the VESC's
speed-mode PID under-delivering at very low ERPM against stiction and rolling
resistance — which is exactly what `velocity_correction` exists for and is
currently unpopulated. Confirming it needs `/commands/motor/speed` and
`/sensors/core` side by side (section 4).

Two second-order effects worth recording:

- **Telemetry deadband.** `erpm_deadband` 500 zeroes `/odom`'s speed below
  0.111 m/s (telemetry only, never commands). 36 of 886 `/odom` samples in the
  mission window are exactly zero, which is why the mean (0.2329) sits below
  the median (0.2530). The medians in the table above are the honest numbers.
- **Absolute floor speed is uncertain.** Whether 1131 ERPM really is 0.25 m/s
  on the floor depends on the gain being right. The archived evidence
  (`output/campaign_final_dirty_diff.md`, and the 2026-09-21 tape measurement:
  a 3 m mission drove 3.8 m while `/odom` logged 3.287 m → gain ≈ 4575) puts
  4516.9 within ~1.3 % today, so 0.25 m/s is about right. Earlier runs needed a
  1.22 factor; this one does not.

### What this means for the MPC

`self.v` feeding `MPC_corr.py:5485` is `/odometry/filtered` — i.e. the
*under-delivered* speed. The command law then adds one acceleration step to it
and publishes that. The 24 % plant deficit and the increment law are not two
independent problems: the deficit is what pins the increment law to 0.35, and
the increment law is what prevents the controller from correcting the deficit.
Either one alone would be invisible.

### Battery voltage

**Not answerable.** No bag records `/sensors/core`, and `/diagnostics` carries
only ZED, urg_node, both EKFs, the BT, Nav2 and the mux — no VESC entry.
`/diagnostics/system_status` is CPU/RAM/GPU only. Flagged in section 4.

---

## 3. Proposed fixes, ranked — **none applied**

1. **Stop deriving the command from the measurement.** Publish the solver's own
   first predicted speed, `pred_v[0]` (or the stage-1 state), instead of
   `self.v + a_cmd*ts` at `MPC_corr.py:5485`. The solver already integrates its
   own model from the measured state, so this keeps the feedback while removing
   the extra dependence on it.
   *Expected effect:* the command stops being pinned to the measurement. With
   today's plant it would rise toward 0.5 and the car would settle wherever the
   VESC actually delivers (~0.38-0.40 m/s at a 0.5 setpoint, if the 0.76 ratio
   holds) instead of at 0.25. *Risk:* this is a real control change; it needs
   the closed-loop rig (`test_stop_ramp_closed_loop.py`,
   `closed_loop_golden.py`) and a low-speed car test, because every stop
   distance in the stack was tuned against the current, slower behaviour.

2. **Add integral action to the increment law.** Keep
   `v_cmd = v + a_cmd*ts` but carry an error integrator on
   `(v_cmd_prev − v_meas)`, clamped, reset per move.
   *Expected effect:* closes the 24 % deficit without touching the solver;
   command converges to 0.5 and measured to ~0.5. *Risk:* an integrator behind
   a solver that already has its own dynamics can fight the stop ramp; it must
   be frozen while the ramp or any stop path is active.

3. **Populate `velocity_correction` in `vesc.yaml`.** Measure a low-speed ERPM
   sweep (commanded vs measured ERPM, 0.1-0.6 m/s) and fill the knots so the
   commanded ERPM compensates the shortfall.
   *Expected effect:* attacks the 0.758 directly, at the one place designed for
   it, and helps every consumer (teleop, calibration, Nav2) not just the MPC.
   *Risk:* needs `/sensors/core` recorded and a careful sweep; the curve is
   per-floor and per-battery to some degree. Note the knots also mirror into
   reverse, which is unmeasured.

4. **Raise `stop_ramp`/`min_moving_speed` awareness of the deficit** — i.e.
   nothing, until 1-3 are decided. Listed only to say explicitly that raising
   vdes above 0.5 is *not* a fix: `max_forward_speed_mps` would clamp it at 0.5,
   and the fixed point `a·ts/(1−r)` does not depend on vdes at all. Raising
   vdes raises `a_cmd` slightly and moves the fixed point by a few cm/s. It
   treats neither cause.

5. **Instrument first, decide after.** Record the four missing topics
   (section 4) on one 10 m straight run at vdes 0.5 before changing any control
   code. It is one run, and it converts the "likely" in section 2 into
   "proved" or kills it.

---

## 4. Topic list to record on the next car test

Add to the mission_logger profile (`enable_mission_logger:=true`, and these are
all low-bandwidth):

```
/commands/motor/speed              # commanded ERPM out of ackermann_to_vesc   <- ESSENTIAL
/sensors/core                      # measured ERPM, battery voltage, current   <- ESSENTIAL
/sensors/servo_position_command    # the only servo feedback that exists
/mpc/stop_target                   # whether the stop ramp had a target
/costmap/unknown_speed_cap         # the 0.4 m/s cap, to rule it in or out from data
/mpc/drive_clamp                   # already recorded; keep (0 messages IS the evidence)
/drive  /ackermann_drive  /odom  /odometry/filtered  /ekf_global/odometry/filtered
/mpc/solver_status  /perception/virtual_wall  /costmap/front_clearance  /tf  /tf_static
/mission/status  /mission/move_outcome
```

With `/commands/motor/speed` and `/sensors/core` the chain table becomes
complete end to end and the VESC-side conclusion stops being an inference.

---

## 5. Side finding, not part of the question

`/costmap/front_clearance` decayed smoothly from the 6.0 sentinel to **0.066 m**
between t ≈ 5 s and t ≈ 14 s while the car kept driving, then jumped back to
~2.9 m and the mission stopped at 1.951 m:

```
t= 4- 6s  median 6.000   min 2.479
t= 6- 8s  median 2.223   min 1.636
t= 8-10s  median 1.348   min 1.083
t=10-12s  median 0.850   min 0.613
t=12-14s  median 0.381   min 0.148
t=14-16s  median 2.933   min 0.066
t=16-18s  median 2.552   min 2.022
t=18-20s  median 1.966   min 1.951   -> front_clearance stop fires
```

Meanwhile `/perception/virtual_wall` — the topic every wall consumer is
supposed to read — held the 50.0 CLEAR sentinel throughout and only reported a
real wall (2.19 m) in the last second. `get_wall_distance_topic()`'s own
docstring says the camera ROI topic was retired as a wall source in 2026-10-15
precisely because "its ROI reaches below the horizon, so with nothing ahead it
reads the FLOOR". The trace above looks exactly like that: a floor reading
closing in as the car drives. It did not affect the speed, and a mission that
stops on `front_clearance < 2.0` surviving a 0.148 m reading for nine seconds is
its own question — worth a separate look at which signal the BT's guard
actually consumes.

---

## Reproducing

```bash
python3 output/speed_gap/chain.py      # the chain table (needs the bag + rclpy)
python3 output/speed_gap/horizon.py    # horizon geometry, ramp check, fixed point
```

Both read the archive run read-only (`file:...?mode=ro`) and write nothing.
