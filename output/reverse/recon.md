# Reverse driving in mpc_corr: recon

Branch `mpc_corr_v2_reverse`, from `origin/campaign-final` at `868ec8a`. Every
`file:line` below is at `868ec8a`; later commits on this branch move the
`MPC_corr.py` / `mpc_solver.py` numbers.

**Base caveat.** On 2026-10-19 the Orin's `campaign-final` was 8 commits ahead
of origin (`b08675e`), and the Orin itself runs `campaign-final-doors`
(`904d86e`, 10 door-detection commits on top). Those commits add
`mpc_controller/wall_boundaries.py` (wall rows from the multi-wall tracker as
the default MPC boundary source, A1), `f1tenth_control/hard_stop.py` (the
last-resort LiDAR stop, A2) and +320 lines in `MPC_corr.py`. **None of that is
in this base.** Where the recon below says "the boundary source" it means the
costmap half-planes this base uses. The user decided to merge later.

Paths are relative to the repo root. The vesc and ackermann_mux sources are
uninitialised submodules in this worktree; their citations come from the main
checkout, which has the same pinned SHAs (vesc `2848a9f`, ackermann_mux
`42b7cd7`) with a clean tree.

---

## 1. Bounds and clamps on v and on the acceleration input

### 1.1 The limits dict (`MPC_corr.py`)

| Key | Value | Line | Notes |
|---|---|---|---|
| `a_min` / `a_max` | -2.0 / 3.0 m/s² | 1341-1342 | literals, no parameter |
| `dAMin` / `dAMax` | -2.0 / 2.0 m/s³ | 1370-1371 | literals |
| `vMin` / `vMax` | **-1.0** / 3.0 m/s | 1372-1373 | literals. **The solver already allows reverse to -1 m/s.** |
| `delta_min` / `delta_max` | -0.309 / +0.319 rad | 1331-1336 | from `stack_params.yaml:1627-1631` |

### 1.2 RTI / OSQP path (`mpc_solver.py`, the shipping solver)

- **Acceleration box**, every stage: `add_row(row_a, limits["a_min"], limits["a_max"])` (`mpc_solver.py:952-954`).
- **Acceleration rate**, every stage (`:968-977`). Stage 0 spans the control period, later stages the prediction step (`t_step = t_first if k == 0 else ts`, `:958`). Stage 0's window is centred on `last_u[1]`.
- **Speed box on the state**, every stage: `add_row(row_v, limits["vMin"], limits["vMax"])` on `x_{k+1}[3]` (`:1106-1108`). It is one scalar pair for the whole horizon. Nothing in the QP is per-stage or sign-dependent.
- **Speed cost**: `w_v * (v - vdes)^2` with one scalar `vdes` for every stage (`:990-995`). For `v < 0` this term already charges `2 * w_v * vdes * |v|` linearly, so reverse costs something even today.

### 1.3 SLSQP rollback path (`mpc_solver.py`)

- Acceleration box as variable bounds: `lb = np.tile([delta_min, a_min])`, `ub = ...` (`:454-456`).
- Acceleration rate as inequality constraints (`planner_nonlcon_corridor`, `:1424-1433`).
- Speed box as inequality constraints, `c.append(v - vMax)`, `c.append(-v + vMin)` (`:1450-1453`).

### 1.4 What happens after the solve (`MPC_corr.py`)

- **The published speed is not a state.** It is `v_cmd_now = self.v + a_cmd * self.ts` (`:4135`): measured speed plus one control period of the first planned acceleration. It is signed.
- **The min-speed rule** (the "operating floor"): `v_cmd_now = floor_moving_speed(v_cmd_now, self.min_moving_speed)` (`:4136-4140`), applied **only** in object mode with `object_speed_mode == SPEED_DRIVE`. `floor_moving_speed` is `max(v_cmd, min_moving_speed)` (`object_approach.py:390-399`), forward-only. `min_moving_speed_mps` is 0.4 (`stack_params.yaml:1449-1451`), declared at `MPC_corr.py:586-592`. The yaml says it is **not** enforced on drive, turn, goal_distance and goal_pose moves.
- **Object speed decision** (`object_approach.py:363-388`): a goal speed `<= 0` is WAIT, `0 < s < 0.4` is BELOW_FLOOR (held at zero), otherwise DRIVE at the goal speed. `MPC_corr.py:3679-3698` publishes zero and skips the solve for anything but DRIVE.
- **vdes**: `self.vdes = 0.5` (`:1514`). Turn and drive commands only accept `msg.speed > 0.0` (`:2737-2738`, `:2886-2887`); zero means "use the default". `_clear_drive_state` restores the default (`:2771`). Object mode substitutes `_object_vdes` (`:3849-3860`).
- **The publisher clamp**: every `/drive` goes through `_publish_drive` → `clamp_drive_speed(speed, max_forward, max_reverse)` (`:3483-3505`, `drive_limits.py:38-52`). `max_reverse_speed_mps` is **0.0** (`stack_params.yaml:1452-1454`), so any negative command goes out as 0.0 and is reported on `/mpc/drive_clamp`. Forward is capped at 0.5 m/s (`:1440-1442`).
- **Tests that pin "no reverse"**: `test_drive_limits.py:86-90` (`test_reverse_goes_out_as_zero`), `:127-134` (asserts `max_reverse_speed_mps == 0.0`), `test_llm_go_to_end_to_end.py:99-102` (`min(speeds) >= 0.0`).

**What this means.** Today the QP happily plans negative speed (the archive has solver-chosen reverse down to -0.231 m/s in 13 runs, `drive_limits.py:13-18`), and the only thing that stops the car reversing is the publisher clamp at 0.0. The reverse feature therefore has to (a) make reverse a deliberate per-stage choice in the QP instead of an accident of `vMin = -1`, and (b) needs `max_reverse_speed_mps > 0` on the car, which is an operator decision, not a code default.

## 2. Where the model and the RTI linearization assume v > 0

**They don't.** The bicycle model is sign-agnostic: `xdot = v cos(psi + beta)`, `psidot = v / L cos(beta) tan(delta)` (`vehicle_model.py:28-41`). With `v < 0` the car moves backwards and the yaw rate flips sign for the same steering angle, which is the correct reverse kinematics for a rear-axle reference. The analytic Jacobian (`vehicle_model.py:50-110`) has no `abs()` or `max()`.

The forward assumptions are all **around** the model:

| Where | What assumes forward | Line |
|---|---|---|
| RTI reference trajectory | `x_ref` is the model rolled out with the warm start, `last_u` tiled when there is none (`mpc_solver.py:822-853`). Nothing forces the reference to have the sign the solution will have, so a reverse plan can be linearized around a forward reference. | `mpc_solver.py:842-865` |
| Speed tracking | one positive `vdes` for every stage | `mpc_solver.py:990-995` |
| Obstacle cost | three probes on the **nose only**: `p_nose = base_link + L/2 * e`, offsets `(0.10, 0.25, 0.40)` along the heading | `mpc_solver.py:567-570`, `object_geometry.py:70` |
| Terminal position | `w_term * ||p_N - pref_nom||^2`, `pref_nom` is a lookahead point **ahead** on the corridor | `mpc_solver.py:1144-1151` |
| Terminal heading | `w_psi` toward `psiRef`, the heading the car should face at the end | `mpc_solver.py:1153-1164` |
| Shift of the warm start | sign-agnostic, but it carries the previous plan's sign into the next reference | `mpc_solver.py:729-777` |
| Model check in the loop | `ev > 0.5` divergence warning, sign-agnostic | `MPC_corr.py:3835` |

## 3. Corridor code that assumes forward travel

- **The corridor starts at the car and runs forward.** `corridor_curves(X0, Y0, psiStart, psiEnd, L, ...)` (`MPC_corr.py:5081-5083`) builds `s = L * u` from 0 to L (`corridor_geometry.py:197-198`) and integrates the centreline forward from `(X0, Y0)` (`:230-231`). Nothing exists behind the car.
- **Behind the start, the lateral bound degenerates.** The QP's hard corridor row uses the nearest centreline index (`mpc_solver.py:633-645`, `:1027-1034`). For a predicted state behind the car, that is index 0, so the bound becomes an infinitely long slab of the start half-width (0.64 m) along the start tangent. It happens to allow straight reversing, by accident.
- **psiRef** is `psiEnd` (`MPC_corr.py:5200`): in goal_distance mode the frozen start heading, in goal_pose mode `atan2(goal - car)` (`:4746`), in object mode the bearing `psi_c`. **A goal behind the car makes psiRef point backwards**, so the forward corridor becomes a U-turn (clamped by R_min, `:5001-5004`, and logged as infeasible when it needs more than full lock, `:5106-5112`).
- **The anchor.** goal_distance progress is `_project_onto_line(car, anchor, psi_line) >= goal_distance` (`MPC_corr.py:3785-3808`). Reversing makes `traveled` decrease; a negative `goal_distance` ends the move on the first tick.
- **s_cum lookahead.** `compute_local_target` takes the nearest index and adds a positive lookahead along `s_cum` (`MPC_corr.py:5580-5590`). The lookahead floor is `1.25 * N * ts_pred * vdes` (`:62-80`), positive by construction.
- **Obstacle deflection** of the target picks the tangent that agrees with the heading: `if np.dot(t, e) < 0.0: t = -t` (`MPC_corr.py:5667-5669`).
- **pref_nom** (the smoothed lookahead target, `:5715-5722`) is what the terminal cost pulls toward, so every QP is pulled forward along the corridor.
- **Object mode**: `target_behind = abs(alpha) > pi/2` becomes terminal after a persistence time (`object_approach.py:535`, `MPC_corr.py:2564-2570`); the docstring says "This vehicle has no reverse primitive" (`object_approach.py:98-100`).

## 4. Drive chain for a negative speed

`/drive` → ackermann_mux → `/ackermann_drive` → ackermann_to_vesc → `commands/motor/speed` (ERPM) → vesc_driver → `sensors/core` → vesc_to_odom (backup node) → `/odom` → EKF → `/odometry/filtered` → `MPC_corr`.

| Stage | Behaviour with v < 0 | Evidence |
|---|---|---|
| `MPC_corr._publish_drive` | **clamps to 0.0** (max_reverse 0.0) | `MPC_corr.py:3483-3505`, `drive_limits.py:50-51`, `stack_params.yaml:1452-1454` |
| ackermann_mux | passes the message unmodified; priority only; no locks; on timeout the lane is masked, no zero is published | `mux.yaml:1-34`, `topic_handle.hpp:182-200`, `ackermann_mux.cpp:112-115` |
| ackermann_to_vesc | `erpm = speed_to_erpm_gain * correctVelocity(speed) + offset`, signed linear. **No ERPM deadband here, no brake command, no special case at a sign change.** Only `commands/motor/speed` is ever published | `ackermann_to_vesc.cpp:131-135` |
| `speed_to_erpm_gain` | 4516.9, offset 0.0 → -0.3 m/s = -1355 ERPM | `vesc.yaml:14-15` |
| velocity_correction | pass-through today (no knots in vesc.yaml); when populated it mirrors the forward curve in reverse — *"an assumption, not a measured fact"* | `velocity_correction.hpp:88-91, 119-143`, `vesc.yaml:31-43` |
| vesc_driver limits | `speed_min/max` = ±23250 ERPM (±5.15 m/s), signed clip; brake limits exist but nothing publishes a brake | `vesc.yaml:94-97`, `vesc_driver.cpp:363-367, 479-495` |
| telemetry deadband | `erpm_deadband` 500 ERPM, symmetric: \|v\| < 0.111 m/s reads as 0 in both directions (stack_params still quotes 0.091 from the old gain) | `vesc.yaml:69`, `vesc_driver.cpp:214-215` |
| vesc_to_odom (the launched `_backup` node) | signed: `linear.x` is negative when reversing, yaw rate `v tan(delta) / L` flips correctly, \|v\| < 0.05 → 0 | `vesc_to_odom_backup.cpp:126-192`, `vesc.launch.py:219-231` |
| the *unused* fused node | **negates** the speed (wrong for this car per the backup's header); must stay out of the launch | `vesc_to_odom.cpp:292` |
| EKF | fuses vx signed, no sign logic | `ekf.yaml:59-98` |
| MPC odom callback | `self.hw_v = msg.twist.twist.linear.x`, signed | `MPC_corr.py:2000`, `:3142` |

**What would be needed for reverse on the car (report only, nothing changed here):**

1. `max_reverse_speed_mps > 0` in `stack_params.yaml` (≥ `reverse_v_rev_max_mps`), with its description rewritten, and the tests that pin 0.0 updated in the same commit (`test_drive_limits.py:86-90, 127-134`, `test_llm_go_to_end_to_end.py:99-102`, `test_object_approach_closed_loop.py:100`, `test_abort_holds_the_car.py:71`).
2. No change in ackermann_mux, ackermann_to_vesc or vesc_driver is **required** for a signed speed to reach the motor. Recommended:
   - tighten `vesc.yaml speed_min` (e.g. -0.5 m/s × 4516.9 ≈ -2258 ERPM) so the driver itself bounds reverse, because teleop, calibration and Nav2 paths bypass the MPC clamp;
   - measure a reverse speed sweep before populating velocity_correction knots (the mirror is unmeasured);
   - check the VESC firmware's behaviour at ERPM 0 and on a direction change (brake vs coast, current spike). That is outside the repo.
3. Keep `vesc_to_odom.cpp:292`'s negation out of the launch, or fix it.
4. Mission loader rejects negative speeds (`mission_config.py:446, 528, 564`); not needed if reverse is the MPC's own choice, which is this design.
5. Stale numbers to fix when touching these files: `drive_limits.py:10-11` and `stack_params.yaml:1427` (±4.23 m/s at 5499.27 → ±5.15 m/s at 4516.9), `stack_params.yaml:1446, 1484` (0.091 → 0.111 m/s).

## 5. Front-only safety checks and the actual rear coverage

**No check anywhere in the stack reads the sign of the velocity.** Every obstacle-avoidance and stop mechanism is either forward-only or fed by the forward camera.

| Check | Looks | Blind reversing? | Evidence |
|---|---|---|---|
| `w_obs` probes in the solver | 3 points at base_link + 0.2525 / 0.4025 / 0.5525 m along the heading | **yes**, no rear probe | `mpc_solver.py:567-570, 1305-1309`, `object_geometry.py:70` |
| Obstacle source `/perception/obstacles_2d` | ZED, ~100° forward | **yes** | `obstacle_projector_node.py:4-9`, `MPC_corr.py:1706-1708` |
| MPC boundary rows (`/costmap/boundaries`) | front ±35°, sides 45-135°; **no rear window** | **yes** | `costmap_boundary.py:311-315`, `costmap_boundary_node.py:351-353`, `mpc_solver.py:1110-1127` |
| Corridor lateral bound / target | s ∈ [0, L] ahead | yes (index-0 slab behind, see §3) | `corridor_geometry.py:197-231`, `mpc_solver.py:1027-1034` |
| `/mpc/min_obstacle_distance` | omnidirectional, but from the **front** bumper, camera-fed | partly: overstates a rear distance by ~0.49 m | `MPC_corr.py:4195-4219` |
| `/mpc/min_obstacle_distance_forward` | half-plane ahead of the bumper | yes | `MPC_corr.py:4224-4268` |
| Predicted clearance | omnidirectional, rear-axle centre, **warning only** | camera-blind, never acts | `MPC_corr.py:4270-4280, 4102-4107` |
| IsProximityTooClose, front | ±45° from the bumper, 0.15 m | yes | `is_proximity_too_close.py:355-362` |
| IsProximityTooClose, side/rear | raw LiDAR \|θ\| ∈ [45°, 135°], 0.15 m **from the LiDAR** | **effectively yes**: beyond 115° a 0.15 m return would be inside the body; beyond 135° there are no beams | `is_proximity_too_close.py:350, 364-368`, `stack_params.yaml:1991-1992` |
| front_clearance stop (virtual wall) | ±10° strip ahead | yes | `condition_eval.py:326-374`, `lidar_front_wall_node.py:221-227` |
| `/costmap/front_clearance` | strip ahead of the bumper | yes | `costmap_boundary.py:184-185` |
| swept_clearance | x ≥ 0 arc (and unconsumed) | yes | `swept_corridor.py:204, 239` |
| last-resort LiDAR stop | **not in this base** (Orin-only `hard_stop.py`, A2) | to be checked after the merge | — |
| TTC / AEB | none exists | — | — |

**Rear sensor coverage.**

- **LiDAR**: Hokuyo UST-10LX, 270° (±135°), no scan filters. Real mount (static TF) at base_link + (0.305, 0, 0.190), yaw 0, i.e. front-facing (`description.launch.py:203-210`; x, z measured, y and yaw not). The blind wedge is the rear 90°, with its apex at the LiDAR, **0.387 m ahead of the rear bumper**. The whole rear face is unseen. A point d metres behind the bumper is visible only if \|y\| > 0.387 + d (0.5 m behind needs \|y\| > 0.89 m).
- **ZED 2i**: front-facing (`camera.launch.py:73-78`), ~101° horizontal.
- **No other rear sensor** (no rear camera, ultrasonic or bumper switch).
- **The SLAM map** behind the car only holds what was scanned on the way in; the rear is never scanned live.
- **Sim differs from the car**: `sensors.xacro:42` mounts the sim LiDAR **rear-facing** at (-0.12, 0, 0.15, yaw π). A reverse test in sim sees the rear and is blind ahead, the opposite of the car.

**Car geometry for rear probes.**

| Quantity | Value | Where | Status |
|---|---|---|---|
| base_link | rear axle | `base.xacro:5-6` | |
| wheelbase | 0.305 m | `stack_params.yaml:1636-1638` | "unmeasured on this car" |
| front bumper | x = 0.405 m | `base_front.xacro:24` | tape-measured 2026-10-13 |
| **rear bumper** | x = **-0.082 m** | `stack_params.yaml:829-831` (`swept_clearance_body_rear_x_m`) | **URDF chassis-mesh bounds, not a tape measurement**; there is no `base_rear` frame (`stack_params.yaml:2927-2929`) |
| body half-width | 0.136 m | `stack_params.yaml:832-834` | mesh bounds |
| steering | -0.309 / +0.319 rad → R_min ≈ 0.96 m | `stack_params.yaml:1627-1631` | derived from calibration |

## 6. What this recon changes in the design

- The QP needs a **per-stage speed box** (and a per-stage speed reference) to express a direction schedule. The scalar `vMin/vMax` stays the default.
- Reverse stages must be linearized around a **reverse reference**: `x_ref` is built from the warm start, so each candidate needs its own warm start whose speed profile already has the candidate's signs.
- Rear probes must be **added**, not substituted: the front can swing into something while reversing.
- The terminal pull to `pref_nom` and `psiRef` is a forward pull. A reverse candidate scored with it always loses; a wall-aware cost-to-go has to be the terminal term when reverse is on.
- **Horizon length.** `mpc_prediction_dt_s` went back to 0.1 s on 2026-10-13 (`stack_params.yaml:1646-1649`), so the horizon is N × 0.1 = **2.0 s**, not the 4.0 s the task's default `C_k = [1.0, 2.0] s` assumes. `C_2.0` ("reverse until 2.0 s") is then the whole horizon, identical to B, and is removed as a duplicate.
- **Timing headroom.** One forward RTI solve takes p50 7.8 ms / p95 9.2 ms on linus (i7-9700K, `test/closed_loop_golden.py` direct cases). The Orin will be several times slower per solve, so the candidate count matters.
- **Baseline test state at 868ec8a**: `mpc_controller` has **19 pre-existing failures** (721 passed, 1 skipped), all stale config-coupled tests (steering limits, prediction step, weight set, corridor sizing, …). The gate for this branch is "no new failures".
