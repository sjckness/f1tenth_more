# Offset-free speed command: design, identification, verification

2026-10-21, branch `campaign-final` (Humble, Orin). Follows `report.md` in this
folder, which found the loop pinned at ~0.35 m/s commanded / ~0.25 m/s real
against vdes 0.5. That happens because `MPC_corr.py` published `v_meas + a·ts`
and nothing integrated the error.

**Result in one line.** `speed_command_mode: offset_free` (the new default)
makes the solver's model carry the command as a state of its own. In that
model the real speed is a first-order lag of the command plus an estimated
disturbance d. On the plant fitted from 104 archived runs, the car now
settles at **0.40 m/s for vdes 0.4** and at **0.43 m/s for vdes 0.5**. At 0.5
the command sits at the unchanged 0.5 ceiling and the car delivers 87 % of it.
Legacy settles at 0.21 and 0.33 on the same plant. `legacy` is kept, and it is
bit-identical to the old code.

Nothing was run on the car. Every number below comes from archived data or
from the closed-loop rig (`mpc_controller/test/speed_channel_harness.py`).

---

## 0. Audit: what the old law touched

**Solver (`mpc_solver.py`).**
- Two backends behind `solve_mpc_step`: RTI/OSQP (`use_rti_solver`, the
  default) and SLSQP (the rollback).
- State `[X, Y, psi, v]` and input `[delta, a]`, with v+ = v + ts·a.
  N = 20, `mpc_prediction_dt_s` 0.1 s, control period 0.1 s.
- x0 is the active odometry, re-seeded every tick: `/odometry/filtered` with
  `localization_source: ekf`.
- Constraints:
  - steering box ±0.31 rad and steering rate ±0.5 rad/s;
  - `a` in [-2, 3] with slew `dA` ±2 m/s³;
  - speed box [vMin 0.0, vMax 3.0] on x[3];
  - corridor rows and soft boundary rows;
  - reverse candidates (`stage_spec`) only with `allow_reverse`.

**Where `a` and v enter.**
- Cost: `w_v (v - vdes)^2`, effective 1.5 per stage. `w_u_a` and `w_du_a`
  are both 0 in `stack_params`.
- `a_min`/`a_max` box and the slew rows.
- Stop ramp: `_apply_stop_ramp` caps vdes, and `_stop_ramp_limits` sets
  `vMax = max(v_ref, v - a_dec·ts)`. **Finding:** that box is binding on
  *every* tick with the ramp enabled, not only near a stop. With no stopping
  point v_ref is vdes, so the plan's speed is hard-capped at vdes. That is
  why `pred_v` sat at exactly 0.4996–0.5001 in the report. It also means the
  terminal cost (`w_term`), which wants more speed, is always held by that
  box.
- Drive clamp: `clamp_drive_speed(max_forward 0.5, max_reverse 0.4)`, plus the
  negative-speed gate, both in `_publish_drive`.
- Operating floor: `floor_moving_speed` on object approaches;
  `floor_moving_speed_signed` with `allow_reverse`.
- Reverse paths:
  - `reverse_mpc` (`allow_reverse`, default off). Its sign schedules and its
    `stop_witness` are built on the integrator model of v.
  - `reverse_manoeuvre` (`reverse_auto_enabled`, default on). It publishes
    ±`v_rev` itself, outside the speed law. Its `StuckDetector` uses
    `forward_infeasible`, `blocked_front` (|v| < 0.05 with something within
    0.6 m) and `no_progress` (x_pred displacement < 0.05 m). Its `RearSolver`
    runs its own 4-state model.

**Consumers of the published speed:** `/drive` → mux → `/ackermann_drive` →
`ackermann_to_vesc` → `/commands/motor/speed`. Also the control CSV
(`v_cmd`, `wheel_speed`), the `CTRL` log line, `/mpc/drive_clamp` and the
model log (`accel_cmd`, documented as "the car is sent v + accel_cmd·ts").

**Consumers of `self.v`:** x0; the ramp box floor; the wall-distance latency
prediction; `/mpc/object_status.speed`; the `ERR` line; the reverse supervisor
(stuck detection, gear-change waits); `floor_moving_speed_signed`; `CTRL`.

**What the change touches:**
- `vehicle_model.py`: the lag model and its Jacobian.
- `mpc_solver.py`: an optional `speed_model`. Without it the QP is
  operation-for-operation the old one; the closed-loop golden digests are
  unchanged.
- New `speed_channel.py`.
- `MPC_corr.py`:
  - `_init_speed_channel`, `_speed_observe`, the plan and command calls
    around the solve;
  - the `_publish_drive` hook and the resets;
  - the `ERR` prediction and the model-log `accel_cmd`.
- 19 keys in `stack_params.yaml`.
- `f1tenth_messages/SpeedChannelStatus.msg`.
- The logger topic list.

Not touched: the reverse manoeuvre, its stuck detector, the rear solver, the
stop-ramp profile, and every weight.

---

## 1. Identification (archived data only)

Script: `identify.py` here. Results: `identification.json`.
- Measured speed: `/odometry/filtered`. It equals `/odom` sample for sample
  (`vx_variance` 0).
- Command: `/ackermann_drive`, identical to `/drive`.
- Timing: header stamps. On car-recorded bags they lag receive time by
  1–14 ms (p95 ≤ 21 ms).
- Data: the 14:20 run, P002-R028, the 10-19 corridor bag (which has
  `/sensors/core`), and 208 archived mission bags from 09-01 to 10-21. Each
  run's ERPM gain was taken from its SLAM/odom distance ratio: 5499.27 until
  09-21 ~12:50 UTC, 4516.9 since.

### 1.1 Model and numbers

Above about 500 ERPM (0.11 m/s):

```
y[k+1] = a y[k] + (1 - a) K(u) u[k - d],   a = exp(-ts / tau)
```

| quantity | value | 95 % interval | basis |
|---|---|---|---|
| **tau** | **0.176 s** | [0.142, 0.198] | median of instrumental-variable fits, 104 runs (run IQR 0.11–0.25) |
| tau, stops only | 0.159 s | [0.139, 0.202] | 35 command steps to zero, exponential in 35/35 |
| **pure delay d** | **0** | dead time p90 0.01 s | the 35 stops (the only exogenous steps). Closed-loop fits are biased to 0 because u[k] contains v[k], so they were not used |
| one-step residual std | 0.0087 m/s | — | median per run |
| rate limit | none seen | — | acceleration proportional to error up to 0.3 m/s / 1.2 m/s² |

The 14:20 run alone cannot pin tau: 0.24–0.44 s depending on the method.

### 1.2 Steady-state gain vs speed

The deficit is a roughly constant 280–400 ERPM, not a fixed percentage.
The offset model `y_ss = K1 (u - u0)` therefore fits better than a constant
gain: leave-one-run-out error 90 ERPM, against 116 for a constant gain.

| | value | 95 % interval |
|---|---|---|
| K1 | 1.04 | [1.00, 1.10] |
| u0 | 0.086 m/s (389 ERPM) | [0.068, 0.107] |
| constant gain instead | 0.887 | [0.880, 0.895] |

| command [m/s] | runs | K | 95 % interval |
|---|---|---|---|
| 0.25–0.30 | 7 | 0.70 | [0.60, 0.73] |
| 0.30–0.35 | 6 | 0.78 | [0.73, 0.83] |
| 0.35–0.40 | 10 | 0.78 | [0.77, 0.80] |
| 0.40–0.45 | 20 | 0.82 | [0.81, 0.83] |
| 0.45–0.50 | 34 | 0.865 | [0.862, 0.867] |
| **0.50–0.55** | 46 | **0.875** | [0.873, 0.878] |
| 0.55–0.65 | 79 | 0.882 | [0.880, 0.884] |

**"~1.0 at 0.5 m/s" is contradicted.** It is 0.875 today, and the 10-19
telemetry agrees: 2258 ERPM commanded, ~1990 delivered. The belief dates from
the old gain 5499.27, which was set on the 0.5 m/s point. That gain
over-commanded by 22 %, so 0.5 really did come out at about 0.5.

### 1.3 Breakaway (the August sweep and the starts)

- The 2026-08-07 sweep (CSVs in commit `6a4b847`) moved the car at its
  smallest command, 0.15 m/s (0.083 actual). Nothing below 0.15 was tried.
- In the 10-19 telemetry, a setpoint of 452 ERPM produces 0 A and 542 ERPM
  produces 10–38 A.
- In all 34 archived starts, the car moved 0.08 s [0.05, 0.11] after the
  first command above 500 ERPM, and it never failed to start.
- So the data do **not** support 0.3–0.4 as a breakaway speed. What cost the
  legacy start was the ramp: a median 0.48 s spent below the threshold.
- `v_breakaway_mps` = **0.25**: 0.20 clears the threshold with its full
  uncertainty, plus margin.

---

## 2. Model

- **States** `[X, Y, psi, v, c]`. c is the command in force; x0's c is what
  the actuator last received. That is what this node published, or the mux
  output when another lane drives. It never comes from the measurement.
- **Input** a is now the command *rate*. `a_min`/`a_max` and the slew rows
  keep their numbers.
- Dynamics:

  ```
  c+ = c + ts·a
  v+ = v + alpha (c_eff + d - v)        alpha = 1 - exp(-ts/tau) = 0.43 at tau 0.18
  ```

  - `c_eff` is the new command, or the one in force when
    `speed_delay_ticks = 1`. Delay is identified as 0 and not modelled. It
    never enters the steady state, and the matrix runs 1–2 ticks against it.
  - d is in **m/s of command**: the car behaves as if sent c + d. The work
    order's `+d'` is the same model with `d' = alpha·d`.
  - alpha is the exact zero-order-hold step, which stays stable even when
    tau < ts. Euler would need tau_euler 0.23 s to match it.
- **Kinematics** use the real v. **Cost** `w_v (v - vdes)^2` is on the real v.
- **Published command** = c at stage 1 = c0 + a0·ts, clipped to the
  requested stage-1 box.
- **Bounds** (`speed_channel.command_box`):
  - Hard box on c: [0, `max_forward_speed_mps`]. Inside it go:
    - the stop ramp's model-consistent cap (box − d);
    - the stall cap;
    - the start-up bound and the moving floor.
  - The box is widened only where the jerk-limited rate rows cannot reach it
    (a witness trajectory), so it is feasible by construction.
  - The real speed gets a **soft** cap, `min(max_forward_speed_mps, ramp
    box)`, from the first stage where less than half of the predicted v is
    measurement (stage 2 at tau 0.18). An earlier cap acted on the 0.02 m/s
    noise and biased the speed 0.012–0.016 m/s low.
- **Command ceiling unchanged at 0.5** (your call). Where `vdes/K > 0.5`, the
  command sits at 0.5 and anti-windup stops d from asking for more.

## 3. Estimator

Current-form Luenberger observer on (v, d), with the two poles at
`exp(-bw·ts)`:

```
v_pred = (1 - alpha) v_hat + alpha (c_eff + d_hat)
v_hat += L_v (v_meas - v_pred),   d_hat += L_d (v_meas - v_pred)
L_v = 1 - p_v p_d / (1 - alpha),   L_d = (1 - p_v)(1 - p_d) / alpha
```

Defaults are bw_v 5 rad/s and bw_d 2 rad/s, so d converges with a time
constant of about 0.5 s. That gives L_v 0.13 and L_d 0.17 at tau 0.18; they
are logged at start.

**Detectability and the offset-free condition.**
- With `A = [[1-alpha, alpha],[0, 1]]` and `C = [1, 0]`,
  `det [C; CA] = alpha > 0`. The pair is observable, so both poles can be
  placed anywhere.
- `rank [[I - A_v, -B_d],[C, C_d]] = rank [[alpha, -alpha],[1, 0]] = 2 = n + n_d`
  (Pannocchia & Rawlings 2003).
- So at any closed-loop steady state the innovation is zero and
  `v_hat = v_meas = c + d_hat`. The model is then exact there, and the
  measured speed equals the plan's own steady speed: vdes, which the ramp box
  holds the plan to. This holds whatever the true gain, lag or delay.

**Saturation.** |d| ≤ `offset_free_d_max_mps` = 0.35.

**Freeze.** d is held and v_hat is re-synced to the measurement on any of
these:
- hold, e-stop, or mission not RUNNING;
- stopping (ramp latch, goal reached);
- reverse manoeuvre;
- odometry stale or repeated;
- override (`/ackermann_drive` ≠ published, by more than 0.03);
- `low_command` (< 0.10) or `low_speed` (< 0.12, above the telemetry
  deadband);
- `stall`;
- `breakaway`;
- directional anti-windup at the ceiling.

**Stall.** Command ≥ 0.2 with |v| < 0.05 for 0.8 s. d is frozen and the
command is capped at its value when the stall was detected, so the controller
never pushes harder. It clears when the car moves.

**Reset.** d = 0 and v_hat = measurement on every goal change, hold engage and
release, e-stop change and forward↔reverse switch. The command in force is
re-synced, not zeroed. After a hold or a gear change it *is* zero, because
that is what was published.

**Stuck → reverse still triggers.** `blocked_front` reads only the measured
speed and the front distance, neither of which the speed law touches.

## 4. Start-up

While `v_meas < offset_free_breakaway_speed_mps` (0.08) and the previous plan
wanted to move, c is lower-bounded at `v_breakaway_mps` (0.25) on the first 3
stages. x0's c is lifted to that value: a jump, outside the rate rows.

- "Wanted to move" means the plan's command beyond those stages reached
  `min(v_breakaway, 0.8·vdes)`. A plan that prefers to stay put (an obstacle
  ahead) is never kicked.
- The bound is released on the tick the car moves.
- `offset_free_moving_floor`: `object` (the default) puts today's ≥ 0.4 m/s
  rule on object approaches as a hard bound on the command. `all` applies it
  to every move; `off` disables it.

## 5. Selection

- `speed_command_mode: legacy | offset_free`, default `offset_free`. Legacy
  runs exactly the old code; the estimator still runs and reports d, but
  nothing acts on it.
- offset_free drops to legacy (with a WARN at start) when `allow_reverse` is
  true or `use_rti_solver` is false. Neither path has the speed model.
- All 19 keys are in `stack_params.yaml` next to the stop ramp, each with its
  derivation.

---

## 6. Verification

The rig is `test/speed_channel_harness.py`.
- Real parts: the RTI QP with `stack_params` weights and limits, the
  `SpeedChannel` object the node uses, the stop ramp, and the `/drive` clamp
  and gate.
- The plant is **not** the model. It has its own gain (constant or the fitted
  curve), its own tau, 0–2 ticks of delay, optional stiction (no motion from
  rest at or below 0.15 m/s), the measured braking (0.144 s, then
  1.5 m/s²), and σ 0.02 m/s noise on the measured speed.

### 6.1 Matrix (180 cells, `test_speed_channel_matrix.py`; all pass)

Cells: gain × tau (0.5×, 1×, 2× the identified 0.176 s) × delay 0/1/2 ×
stiction off/0.15 × vdes 0.5/0.4. Steady state is the true-speed mean over
8–12 s. The target is vdes where the 0.5 ceiling reaches it, otherwise what
the ceiling delivers.

| plant gain | vdes | target | v_ss mean | max \|error\| | max overshoot | median rise | command ss | d ss |
|---|---|---|---|---|---|---|---|---|
| 0.6 | 0.5 | 0.300 (ceiling) | 0.300 | 0.0001 | 0 % | — | 0.500 | −0.18 |
| 0.6 | 0.4 | 0.300 (ceiling) | 0.300 | 0.0002 | 0 % | — | 0.500 | −0.18 |
| 0.76 | 0.5 | 0.380 (ceiling) | 0.380 | 0.0007 | 0 % | — | 0.499 | −0.11 |
| 0.76 | 0.4 | 0.380 (ceiling) | 0.380 | 0.0005 | 0 % | 1.2 s | 0.499 | −0.12 |
| 1.0 | 0.5 | 0.500 | 0.496 | 0.0037 | 0 % | 0.5 s | 0.497 | 0.00 |
| 1.0 | 0.4 | 0.400 | 0.399 | 0.0015 | 8.1 % | 0.4 s | 0.399 | 0.00 |
| 1.15 | 0.5 | 0.500 | 0.499 | 0.0015 | 12.0 % | 0.4 s | 0.434 | +0.06 |
| 1.15 | 0.4 | 0.400 | 0.399 | 0.0017 | 12.8 % | 0.3 s | 0.347 | +0.05 |
| **fitted** | 0.5 | 0.431 (ceiling) | 0.431 | 0.0002 | 0 % | — | 0.500 | −0.05 |
| **fitted** | 0.4 | 0.400 | 0.399 | 0.0015 | 6.9 % | 0.9 s | 0.469 | −0.07 |

- **Steady-state error is below 0.004 m/s in all 180 cells**, against a
  0.02 limit.
- **The command never exceeds 0.5.**
- **Overshoot.** 10 cells exceed 10 %, the worst at 12.8 %. Every one of
  them combines gain 1.15 (above anything measured; the fitted gain is at
  most 0.89 up to 0.65 m/s) with 2 ticks of delay (the measured delay is
  under 20 ms). The car over-delivers, or answers 0.2 s late, before the
  observer can see it.
  - Per your call ("a small overshoot is not a problem"), the test allows
    < 15 % there and < 10 % everywhere else.
  - A model tau of 0.26 s plus a fading start-up margin brought every cell
    under 10 %, at the cost of 0.3 s more rise time. It was not kept.
- The full per-cell table is in the appendix.

### 6.2 Scenarios (`test_speed_channel_scenarios.py`; all pass, fitted plant)

| scenario | result |
|---|---|
| Wall stop 2.0 m (`front_clearance` through the ramp, 2 ticks of sensing delay), 5 seeds × vdes 0.5/0.4 | rests at 1.98–2.02 m. It arrives at the ramp's v_min of 0.4 as designed (legacy arrived at ~0.29), so the hand-over quantisation, one tick or ≤ 4 cm, now shows. Never more than one tick inside the target |
| `goal_distance` 3 m arrival | within ±5 cm. That branch hands over on d_rem without latency prediction; this is pre-existing ramp behaviour |
| Hold 4–6 s then release | zero while held; d reset and frozen; the release restarts from 0 → 0.25 → ramp, with no jump back to the pre-hold 0.47; back at 0.40 ± 0.02 |
| Blocked wheels 4–7 s | d frozen on the first blocked tick; stalled; command ≤ the pre-block command; back at 0.40 after release |
| Reverse manoeuvre (stop, −0.4 m/s for 1.2 s, stop) | d reset on both switches; negative commands only from the manoeuvre; back at 0.40 |
| Stuck detector, car stopped 0.4 m from an obstacle | `blocked_front` fires within `stuck_time_s` + 0.5 s, within 0.25 s of legacy |
| **Legacy sanity check**, constant gain 0.758 | settles at **0.36 commanded / 0.27 real**, against the car's 0.33 / 0.25; `a·ts/(1−r)` reproduces it within 15 % |
| Node level (`test_speed_channel_node.py`, real `MPCController`, private DDS, no stack) | defaults to offset_free; the breakaway jump happens on tick 2; hold, release and override are seen; `/mpc/speed_channel` goes out on every `/drive`; legacy publishes exactly `v + a·ts` |

### 6.3 Solve time (linus, x86; the Orin is slower)

| | p50 | p95 |
|---|---|---|
| legacy | 5.1 ms | 5.9 ms |
| offset_free | 7.0 ms | 7.8 ms |

That is +32 % at p95: 140 QP variables instead of 120, plus one slack. The
0.1 s budget leaves a large margin even at several times this on the Orin.

### 6.4 Gates

- Build: `f1tenth_messages`, `mpc_controller`, `f1tenth_params` and
  `f1tenth_logger` were built on Jazzy (`colcon build --symlink-install`).
  No Humble-only or Jazzy-only API is used.
- Tests: mpc_controller, with pytest run in the install env on linus (Jazzy,
  ROS_DOMAIN_ID 147, localhost only). No new failures against the 21 that
  already fail on a clean `campaign-final` here (steering-calibration,
  wall-turn, weight-set, prediction-step and lint tests); 3 of those 21 now
  pass. 1356+ passed, including every new test. f1tenth_logger: the edited
  test passes; 2 files cannot be collected on linus for lack of `pyarrow`
  (environment, pre-existing).
- flake8: mpc_controller 1177 -> 1175 (no new findings; the new files are clean).
  pep257: 803 -> 803. f1tenth_logger unchanged (3518 / 414).

---

## 7. Logging

- `/mpc/speed_channel` (`SpeedChannelStatus`) on every `/drive`:
  - the estimator: `v_meas`, `v_hat`, `d_hat`, the innovation;
  - the command: `v_cmd`, the command in force, `/ackermann_drive`,
    override;
  - the freeze state and its reasons, stall and stall cap, breakaway;
  - the ceiling, the speed slack, the last reset;
  - the constants: tau, alpha, L_v, L_d.
- The mission_logger default list now includes `/mpc/speed_channel`,
  `/commands/motor/speed`, `/sensors/core` and `/mpc/stop_target`.

## 8. Residual risks

1. **vdes 0.5 tops out at ~0.43–0.44 real** with the command capped at 0.5
   (K(0.5) = 0.875). To get 0.5 real, raise `max_forward_speed_mps` to about
   0.6. The ramp box still holds the real speed at vdes.
2. **The controller regulates *measured* speed.** If odometry reads short, as
   it did before the 09-21 gain fix (a factor of 1.22), the true speed
   exceeds vdes by the same factor. The tape check below covers this.
3. **Stops now arrive at the designed 0.4 m/s,** so the ramp's own one-tick
   hand-over error (≤ 4 cm) is visible where legacy hid it by arriving slower.
4. **Every goal change resets d,** as specified. Chained moves relearn d
   (~0.07) in about 0.5 s, with a small speed dip at each change.
5. **`allow_reverse` or the SLSQP solver means legacy.**
6. **Not modelled:** the VESC speed PID itself (the 10-19 run shows 30–38 A
   with the duty cycle flipping sign at cruise), battery sag, floor changes.
7. **The node test runs faster than real time,** and the node rebuilds its
   corridor on the wall clock, so that test keeps its windows short.

---

## Car test

Build on the Orin:

```
colcon build --symlink-install --packages-select f1tenth_messages f1tenth_params mpc_controller f1tenth_logger
```

`f1tenth_messages` has a new message. Then launch the stack as usual with
`enable_mission_logger:=true`.

**Run A, vdes 0.4.** A straight `drive` at speed 0.4 with a `front_clearance`
2.0 m stop (M02_approach_wall's shape), with at least 6 m of free straight.
At 0.4 the target is reachable under the 0.5 cap.

**Run B, vdes 0.5.** The same at speed 0.5. This one is cap-limited: expect
~0.43.

**Run C, tape check.** A `goal_distance` 4.0 m move at speed 0.5.
- Tape-measure the true distance from start to rest. Compare it with the
  odometry's travelled distance (`GOAL | traveled=` in the log, or
  `/odometry/filtered` in the bag).
- Scale s = tape / odom. **Pass: |s − 1| ≤ 0.03.** The true cruise speed is
  then s × v_meas.
- Optionally, tape a 2.00 m segment in the middle of the straight and time the
  bumper crossing both marks on video; at 0.43 m/s that is about 4.6 s.

**What to look at** (bag: `/mpc/speed_channel`, `/odometry/filtered`,
`/drive`, `/commands/motor/speed`, `/sensors/core`):
- `v_meas` against vdes;
- `d_hat`;
- `v_cmd`;
- `freeze_reasons`;
- `breakaway_active`;
- `/sensors/core` ERPM against `/commands/motor/speed`, and the battery
  voltage.

**Pass, run A:**
- mean `v_meas` from 2 s after the start until the ramp begins: **0.38–0.42**;
- command ≈ 0.45–0.48 and never above 0.5;
- `d_hat` steady around −0.05 to −0.09, not saturated at ±0.35;
- `frozen` false at cruise, with no `override` or `stall`;
- `breakaway_active` only on the first 1–3 ticks; the car moves within 0.3 s
  of the first command, with no lurch;
- peak `v_meas` ≤ 0.44;
- rests **1.96–2.10 m** from the wall and the mission completes.

**Pass, run B:**
- `v_meas` 0.41–0.45 with the command pinned at 0.50;
- `anti_windup` in `freeze_reasons` is expected there;
- `d_hat` ≈ −0.05 to −0.08.

**Fail (then set `speed_command_mode: legacy` and relaunch):**
- steady `v_meas` < 0.37 at vdes 0.4;
- `d_hat` at the saturation;
- the command oscillating more than ±0.05 at cruise;
- a `stall` during normal driving;
- any negative `/drive` outside a reverse manoeuvre;
- a rest closer than 1.95 m.

---

## Appendix: the full matrix

Generated with `python3 test/speed_channel_harness.py` (seed 7, σ 0.02 m/s,
12 s). Read the columns this way:
- `error` is v_ss − target.
- `ok (ceiling)` marks a cell where vdes is past what 0.5 m/s of command
  delivers; there the target is that delivered speed.
- `rise` is 10 → 90 % of vdes, and is `nan` where the ceiling stops the car
  short of 90 % of vdes. The test measures rise to the target.

| gain | tau | delay | stiction | vdes | v_ss | error | ok | rise s | overshoot | cmd peak | cmd ss | d ss |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.6 | 0.5x | 0 | 0.00 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.180 |
| 0.6 | 0.5x | 0 | 0.00 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.168 |
| 0.6 | 0.5x | 0 | 0.15 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.179 |
| 0.6 | 0.5x | 0 | 0.15 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.168 |
| 0.6 | 0.5x | 1 | 0.00 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.178 |
| 0.6 | 0.5x | 1 | 0.00 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.189 |
| 0.6 | 0.5x | 1 | 0.15 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.178 |
| 0.6 | 0.5x | 1 | 0.15 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.189 |
| 0.6 | 0.5x | 2 | 0.00 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.183 |
| 0.6 | 0.5x | 2 | 0.00 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.188 |
| 0.6 | 0.5x | 2 | 0.15 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.183 |
| 0.6 | 0.5x | 2 | 0.15 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.188 |
| 0.6 | 1.0x | 0 | 0.00 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.179 |
| 0.6 | 1.0x | 0 | 0.00 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.188 |
| 0.6 | 1.0x | 0 | 0.15 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.179 |
| 0.6 | 1.0x | 0 | 0.15 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.188 |
| 0.6 | 1.0x | 1 | 0.00 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.182 |
| 0.6 | 1.0x | 1 | 0.00 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.188 |
| 0.6 | 1.0x | 1 | 0.15 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.182 |
| 0.6 | 1.0x | 1 | 0.15 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.188 |
| 0.6 | 1.0x | 2 | 0.00 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.183 |
| 0.6 | 1.0x | 2 | 0.00 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.187 |
| 0.6 | 1.0x | 2 | 0.15 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.183 |
| 0.6 | 1.0x | 2 | 0.15 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.187 |
| 0.6 | 2.0x | 0 | 0.00 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.184 |
| 0.6 | 2.0x | 0 | 0.00 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.187 |
| 0.6 | 2.0x | 0 | 0.15 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.184 |
| 0.6 | 2.0x | 0 | 0.15 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.187 |
| 0.6 | 2.0x | 1 | 0.00 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.182 |
| 0.6 | 2.0x | 1 | 0.00 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.186 |
| 0.6 | 2.0x | 1 | 0.15 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.182 |
| 0.6 | 2.0x | 1 | 0.15 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.186 |
| 0.6 | 2.0x | 2 | 0.00 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.181 |
| 0.6 | 2.0x | 2 | 0.00 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.188 |
| 0.6 | 2.0x | 2 | 0.15 | 0.5 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.181 |
| 0.6 | 2.0x | 2 | 0.15 | 0.4 | 0.300 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.166 |
| 0.76 | 0.5x | 0 | 0.00 | 0.5 | 0.380 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.102 |
| 0.76 | 0.5x | 0 | 0.00 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.20 | 0.0 % | 0.500 | 0.499 | -0.114 |
| 0.76 | 0.5x | 0 | 0.15 | 0.5 | 0.380 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.102 |
| 0.76 | 0.5x | 0 | 0.15 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.20 | 0.0 % | 0.500 | 0.499 | -0.114 |
| 0.76 | 0.5x | 1 | 0.00 | 0.5 | 0.379 | -0.001 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.106 |
| 0.76 | 0.5x | 1 | 0.00 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.20 | 0.0 % | 0.500 | 0.499 | -0.117 |
| 0.76 | 0.5x | 1 | 0.15 | 0.5 | 0.379 | -0.001 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.106 |
| 0.76 | 0.5x | 1 | 0.15 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.20 | 0.0 % | 0.500 | 0.499 | -0.117 |
| 0.76 | 0.5x | 2 | 0.00 | 0.5 | 0.379 | -0.001 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.108 |
| 0.76 | 0.5x | 2 | 0.00 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.20 | 0.0 % | 0.500 | 0.499 | -0.115 |
| 0.76 | 0.5x | 2 | 0.15 | 0.5 | 0.379 | -0.001 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.108 |
| 0.76 | 0.5x | 2 | 0.15 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.20 | 0.0 % | 0.500 | 0.499 | -0.115 |
| 0.76 | 1.0x | 0 | 0.00 | 0.5 | 0.380 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.103 |
| 0.76 | 1.0x | 0 | 0.00 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.10 | 0.0 % | 0.500 | 0.499 | -0.117 |
| 0.76 | 1.0x | 0 | 0.15 | 0.5 | 0.380 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.103 |
| 0.76 | 1.0x | 0 | 0.15 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.10 | 0.0 % | 0.500 | 0.499 | -0.117 |
| 0.76 | 1.0x | 1 | 0.00 | 0.5 | 0.379 | -0.001 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.108 |
| 0.76 | 1.0x | 1 | 0.00 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.10 | 0.0 % | 0.500 | 0.499 | -0.117 |
| 0.76 | 1.0x | 1 | 0.15 | 0.5 | 0.379 | -0.001 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.108 |
| 0.76 | 1.0x | 1 | 0.15 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.20 | 0.0 % | 0.500 | 0.499 | -0.117 |
| 0.76 | 1.0x | 2 | 0.00 | 0.5 | 0.380 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.106 |
| 0.76 | 1.0x | 2 | 0.00 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.20 | 0.0 % | 0.500 | 0.499 | -0.115 |
| 0.76 | 1.0x | 2 | 0.15 | 0.5 | 0.380 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.106 |
| 0.76 | 1.0x | 2 | 0.15 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.20 | 0.0 % | 0.500 | 0.499 | -0.115 |
| 0.76 | 2.0x | 0 | 0.00 | 0.5 | 0.380 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.105 |
| 0.76 | 2.0x | 0 | 0.00 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.30 | 0.0 % | 0.500 | 0.499 | -0.116 |
| 0.76 | 2.0x | 0 | 0.15 | 0.5 | 0.380 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.105 |
| 0.76 | 2.0x | 0 | 0.15 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.30 | 0.0 % | 0.500 | 0.499 | -0.116 |
| 0.76 | 2.0x | 1 | 0.00 | 0.5 | 0.380 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.107 |
| 0.76 | 2.0x | 1 | 0.00 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.30 | 0.0 % | 0.500 | 0.499 | -0.115 |
| 0.76 | 2.0x | 1 | 0.15 | 0.5 | 0.380 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.499 | -0.107 |
| 0.76 | 2.0x | 1 | 0.15 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.30 | 0.0 % | 0.500 | 0.499 | -0.115 |
| 0.76 | 2.0x | 2 | 0.00 | 0.5 | 0.380 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.107 |
| 0.76 | 2.0x | 2 | 0.00 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.40 | 0.0 % | 0.500 | 0.499 | -0.114 |
| 0.76 | 2.0x | 2 | 0.15 | 0.5 | 0.380 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.107 |
| 0.76 | 2.0x | 2 | 0.15 | 0.4 | 0.380 | -0.000 | yes (ceiling) | 1.40 | 0.0 % | 0.500 | 0.499 | -0.114 |
| 1.0 | 0.5x | 0 | 0.00 | 0.5 | 0.497 | -0.003 | yes | 0.40 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 0.5x | 0 | 0.00 | 0.4 | 0.398 | -0.002 | yes | 0.30 | 3.3 % | 0.415 | 0.399 | -0.001 |
| 1.0 | 0.5x | 0 | 0.15 | 0.5 | 0.497 | -0.003 | yes | 0.40 | 0.0 % | 0.500 | 0.497 | -0.000 |
| 1.0 | 0.5x | 0 | 0.15 | 0.4 | 0.398 | -0.002 | yes | 0.30 | 3.3 % | 0.415 | 0.399 | -0.001 |
| 1.0 | 0.5x | 1 | 0.00 | 0.5 | 0.497 | -0.003 | yes | 0.40 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 0.5x | 1 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.30 | 3.7 % | 0.416 | 0.399 | -0.001 |
| 1.0 | 0.5x | 1 | 0.15 | 0.5 | 0.497 | -0.003 | yes | 0.40 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 0.5x | 1 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.30 | 3.7 % | 0.416 | 0.399 | -0.001 |
| 1.0 | 0.5x | 2 | 0.00 | 0.5 | 0.496 | -0.004 | yes | 0.40 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 0.5x | 2 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.30 | 4.5 % | 0.419 | 0.399 | -0.001 |
| 1.0 | 0.5x | 2 | 0.15 | 0.5 | 0.496 | -0.004 | yes | 0.40 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 0.5x | 2 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.30 | 4.5 % | 0.419 | 0.399 | -0.001 |
| 1.0 | 1.0x | 0 | 0.00 | 0.5 | 0.497 | -0.003 | yes | 0.50 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 1.0x | 0 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.40 | 3.7 % | 0.417 | 0.399 | -0.001 |
| 1.0 | 1.0x | 0 | 0.15 | 0.5 | 0.497 | -0.003 | yes | 0.50 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 1.0x | 0 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.40 | 3.7 % | 0.417 | 0.399 | -0.001 |
| 1.0 | 1.0x | 1 | 0.00 | 0.5 | 0.497 | -0.003 | yes | 0.50 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 1.0x | 1 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.40 | 4.0 % | 0.419 | 0.400 | -0.001 |
| 1.0 | 1.0x | 1 | 0.15 | 0.5 | 0.497 | -0.003 | yes | 0.50 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 1.0x | 1 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.40 | 4.0 % | 0.419 | 0.400 | -0.001 |
| 1.0 | 1.0x | 2 | 0.00 | 0.5 | 0.497 | -0.003 | yes | 0.50 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 1.0x | 2 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.40 | 4.4 % | 0.421 | 0.400 | -0.001 |
| 1.0 | 1.0x | 2 | 0.15 | 0.5 | 0.497 | -0.003 | yes | 0.50 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 1.0x | 2 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.40 | 4.4 % | 0.421 | 0.400 | -0.001 |
| 1.0 | 2.0x | 0 | 0.00 | 0.5 | 0.496 | -0.004 | yes | 0.90 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 2.0x | 0 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.70 | 5.6 % | 0.437 | 0.399 | -0.001 |
| 1.0 | 2.0x | 0 | 0.15 | 0.5 | 0.496 | -0.004 | yes | 0.90 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 2.0x | 0 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.70 | 5.6 % | 0.438 | 0.399 | -0.001 |
| 1.0 | 2.0x | 1 | 0.00 | 0.5 | 0.496 | -0.004 | yes | 0.90 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 2.0x | 1 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.70 | 6.3 % | 0.441 | 0.400 | -0.001 |
| 1.0 | 2.0x | 1 | 0.15 | 0.5 | 0.496 | -0.004 | yes | 0.90 | 0.0 % | 0.500 | 0.497 | -0.001 |
| 1.0 | 2.0x | 1 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.70 | 6.4 % | 0.441 | 0.400 | -0.001 |
| 1.0 | 2.0x | 2 | 0.00 | 0.5 | 0.496 | -0.004 | yes | 0.90 | 0.0 % | 0.500 | 0.497 | -0.000 |
| 1.0 | 2.0x | 2 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.70 | 8.1 % | 0.443 | 0.399 | -0.001 |
| 1.0 | 2.0x | 2 | 0.15 | 0.5 | 0.496 | -0.004 | yes | 0.90 | 0.0 % | 0.500 | 0.497 | -0.000 |
| 1.0 | 2.0x | 2 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.80 | 8.1 % | 0.444 | 0.399 | -0.001 |
| 1.15 | 0.5x | 0 | 0.00 | 0.5 | 0.499 | -0.001 | yes | 0.30 | 4.9 % | 0.466 | 0.434 | +0.064 |
| 1.15 | 0.5x | 0 | 0.00 | 0.4 | 0.398 | -0.002 | yes | 0.20 | 7.0 % | 0.383 | 0.347 | +0.051 |
| 1.15 | 0.5x | 0 | 0.15 | 0.5 | 0.499 | -0.001 | yes | 0.30 | 4.8 % | 0.466 | 0.434 | +0.064 |
| 1.15 | 0.5x | 0 | 0.15 | 0.4 | 0.398 | -0.002 | yes | 0.20 | 6.9 % | 0.383 | 0.347 | +0.051 |
| 1.15 | 0.5x | 1 | 0.00 | 0.5 | 0.499 | -0.001 | yes | 0.30 | 8.9 % | 0.487 | 0.434 | +0.064 |
| 1.15 | 0.5x | 1 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.20 | 9.9 % | 0.392 | 0.347 | +0.051 |
| 1.15 | 0.5x | 1 | 0.15 | 0.5 | 0.499 | -0.001 | yes | 0.30 | 8.8 % | 0.486 | 0.434 | +0.064 |
| 1.15 | 0.5x | 1 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.20 | 9.8 % | 0.391 | 0.347 | +0.051 |
| 1.15 | 0.5x | 2 | 0.00 | 0.5 | 0.499 | -0.001 | yes | 0.30 | 12.0 % | 0.493 | 0.434 | +0.064 |
| 1.15 | 0.5x | 2 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.20 | 12.8 % | 0.399 | 0.347 | +0.051 |
| 1.15 | 0.5x | 2 | 0.15 | 0.5 | 0.499 | -0.001 | yes | 0.30 | 12.0 % | 0.493 | 0.434 | +0.064 |
| 1.15 | 0.5x | 2 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.20 | 12.7 % | 0.399 | 0.347 | +0.051 |
| 1.15 | 1.0x | 0 | 0.00 | 0.5 | 0.499 | -0.001 | yes | 0.40 | 3.6 % | 0.488 | 0.434 | +0.064 |
| 1.15 | 1.0x | 0 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.30 | 5.9 % | 0.393 | 0.347 | +0.051 |
| 1.15 | 1.0x | 0 | 0.15 | 0.5 | 0.499 | -0.001 | yes | 0.40 | 3.6 % | 0.488 | 0.434 | +0.064 |
| 1.15 | 1.0x | 0 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.30 | 5.9 % | 0.393 | 0.347 | +0.051 |
| 1.15 | 1.0x | 1 | 0.00 | 0.5 | 0.499 | -0.001 | yes | 0.40 | 8.4 % | 0.497 | 0.434 | +0.064 |
| 1.15 | 1.0x | 1 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.30 | 7.8 % | 0.398 | 0.348 | +0.051 |
| 1.15 | 1.0x | 1 | 0.15 | 0.5 | 0.499 | -0.001 | yes | 0.40 | 8.4 % | 0.497 | 0.434 | +0.064 |
| 1.15 | 1.0x | 1 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.30 | 7.7 % | 0.398 | 0.348 | +0.051 |
| 1.15 | 1.0x | 2 | 0.00 | 0.5 | 0.499 | -0.001 | yes | 0.40 | 11.9 % | 0.500 | 0.435 | +0.064 |
| 1.15 | 1.0x | 2 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.30 | 9.6 % | 0.402 | 0.347 | +0.051 |
| 1.15 | 1.0x | 2 | 0.15 | 0.5 | 0.499 | -0.001 | yes | 0.40 | 11.9 % | 0.500 | 0.435 | +0.064 |
| 1.15 | 1.0x | 2 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.30 | 9.7 % | 0.402 | 0.347 | +0.051 |
| 1.15 | 2.0x | 0 | 0.00 | 0.5 | 0.499 | -0.001 | yes | 0.60 | 8.7 % | 0.500 | 0.434 | +0.064 |
| 1.15 | 2.0x | 0 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.50 | 9.6 % | 0.424 | 0.347 | +0.051 |
| 1.15 | 2.0x | 0 | 0.15 | 0.5 | 0.499 | -0.001 | yes | 0.60 | 8.1 % | 0.500 | 0.434 | +0.064 |
| 1.15 | 2.0x | 0 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.50 | 8.0 % | 0.412 | 0.347 | +0.051 |
| 1.15 | 2.0x | 1 | 0.00 | 0.5 | 0.499 | -0.001 | yes | 0.60 | 9.7 % | 0.500 | 0.434 | +0.064 |
| 1.15 | 2.0x | 1 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.50 | 9.3 % | 0.413 | 0.347 | +0.051 |
| 1.15 | 2.0x | 1 | 0.15 | 0.5 | 0.499 | -0.001 | yes | 0.60 | 9.8 % | 0.500 | 0.434 | +0.064 |
| 1.15 | 2.0x | 1 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.50 | 9.4 % | 0.413 | 0.347 | +0.051 |
| 1.15 | 2.0x | 2 | 0.00 | 0.5 | 0.498 | -0.002 | yes | 0.60 | 10.6 % | 0.500 | 0.434 | +0.064 |
| 1.15 | 2.0x | 2 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.50 | 11.4 % | 0.411 | 0.347 | +0.051 |
| 1.15 | 2.0x | 2 | 0.15 | 0.5 | 0.498 | -0.002 | yes | 0.60 | 10.6 % | 0.500 | 0.434 | +0.064 |
| 1.15 | 2.0x | 2 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.50 | 11.5 % | 0.412 | 0.347 | +0.051 |
| fitted | 0.5x | 0 | 0.00 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.050 |
| fitted | 0.5x | 0 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.90 | 3.3 % | 0.483 | 0.469 | -0.071 |
| fitted | 0.5x | 0 | 0.15 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.050 |
| fitted | 0.5x | 0 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.90 | 3.3 % | 0.483 | 0.469 | -0.071 |
| fitted | 0.5x | 1 | 0.00 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.049 |
| fitted | 0.5x | 1 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.90 | 3.5 % | 0.484 | 0.469 | -0.071 |
| fitted | 0.5x | 1 | 0.15 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.049 |
| fitted | 0.5x | 1 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.90 | 3.5 % | 0.484 | 0.469 | -0.071 |
| fitted | 0.5x | 2 | 0.00 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.048 |
| fitted | 0.5x | 2 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.90 | 4.6 % | 0.488 | 0.470 | -0.071 |
| fitted | 0.5x | 2 | 0.15 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.048 |
| fitted | 0.5x | 2 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.90 | 4.6 % | 0.488 | 0.470 | -0.071 |
| fitted | 1.0x | 0 | 0.00 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.049 |
| fitted | 1.0x | 0 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.80 | 3.4 % | 0.485 | 0.469 | -0.071 |
| fitted | 1.0x | 0 | 0.15 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.049 |
| fitted | 1.0x | 0 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.80 | 3.4 % | 0.485 | 0.469 | -0.071 |
| fitted | 1.0x | 1 | 0.00 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.048 |
| fitted | 1.0x | 1 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.90 | 3.9 % | 0.488 | 0.470 | -0.071 |
| fitted | 1.0x | 1 | 0.15 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.048 |
| fitted | 1.0x | 1 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.90 | 3.9 % | 0.488 | 0.470 | -0.071 |
| fitted | 1.0x | 2 | 0.00 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.048 |
| fitted | 1.0x | 2 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 0.80 | 5.1 % | 0.492 | 0.470 | -0.071 |
| fitted | 1.0x | 2 | 0.15 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.048 |
| fitted | 1.0x | 2 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 0.80 | 5.1 % | 0.492 | 0.470 | -0.071 |
| fitted | 2.0x | 0 | 0.00 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.049 |
| fitted | 2.0x | 0 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 1.00 | 5.9 % | 0.499 | 0.469 | -0.071 |
| fitted | 2.0x | 0 | 0.15 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.049 |
| fitted | 2.0x | 0 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 1.00 | 5.9 % | 0.499 | 0.469 | -0.071 |
| fitted | 2.0x | 1 | 0.00 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.048 |
| fitted | 2.0x | 1 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 1.00 | 6.9 % | 0.500 | 0.469 | -0.071 |
| fitted | 2.0x | 1 | 0.15 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.048 |
| fitted | 2.0x | 1 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 1.00 | 6.9 % | 0.500 | 0.469 | -0.071 |
| fitted | 2.0x | 2 | 0.00 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.048 |
| fitted | 2.0x | 2 | 0.00 | 0.4 | 0.399 | -0.001 | yes | 1.10 | 6.9 % | 0.500 | 0.469 | -0.071 |
| fitted | 2.0x | 2 | 0.15 | 0.5 | 0.431 | -0.000 | yes (ceiling) | nan | 0.0 % | 0.500 | 0.500 | -0.048 |
| fitted | 2.0x | 2 | 0.15 | 0.4 | 0.399 | -0.001 | yes | 1.10 | 6.9 % | 0.500 | 0.469 | -0.071 |
