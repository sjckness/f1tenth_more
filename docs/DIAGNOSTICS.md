# MPC model-validation diagnostics

## Where this fits: step 1 of 3

The car does not turn smoothly. The fix is deliberately sequenced — do not skip ahead:

1. **Log and analyse (this).** Per-control-step CSV logging in `mpc_corr`, plus the offline
   script `tools/mpc_model_check.py`. Additive instrumentation only: no change to the solver,
   cost, weights, constraints, horizon, model equations, `vesc.yaml`,
   `steering_calibration.yaml`, or the MPC steering bounds.
2. **Steering calibration on the car.** Constant-radius circles at several steering commands
   per side, to measure `steering_angle_to_servo_gain_left` and `_right`. Scheduled, not done.
3. **Only then: MPC weight tuning.** Weight tuning is deferred until after calibration. Weights
   tuned on top of a wrong steering model give a fragile, speed-dependent tune that is thrown
   away the moment the model is corrected.

The script in step 1 estimates the same steering gain that step 2 measures, so the two
cross-check each other.

## The open question: is the steering gain right?

The steering calibration lives in
`src/f1tenth_hardware/f1tenth_hardware/config/steering_calibration.yaml`, which overrides
`vesc.yaml` for these keys:

| key | value | status |
|---|---|---|
| `servo_min` / `servo_max` | 0.15 / 0.8318 | |
| `steering_angle_to_servo_offset` | 0.4874 | retuned 2026-09-08 from 50 archived bags (825 SLAM-pose intervals, SEM 0.18 deg) |
| `steering_angle_to_servo_gain_left` / `_right` | −1.2135 / −1.2135 | **placeholder**: the stock F1TENTH value, never measured on this car |

`vesc.yaml` also carries `steering_angle_to_servo_gain: -1.0926`. Only
`vesc_to_odom_node_backup` uses it, to integrate `/odom`'s yaw from the commanded servo. That
makes it a second, different number for the same physical quantity.

The MPC steering bounds (`mpc_steering_angle_min_rad` −0.283, `_max_rad` +0.278 in
`stack_params.yaml`) came from inverting the servo limits through the unverified gain, so they
inherit its error. The offset retune implied a gain-error factor of about 1.494. That would put
the true gain near −0.81, meaning the car can reach about ±0.42 rad while the QP limits itself
to two-thirds of that.

A wheelbase fitted from ordinary driving is the same quantity as the gain, in different units:

```
Lf_eff = L * g_true / g_assumed
```

| `Lf_eff` (L = 0.3302) | implied gain | meaning |
|---|---|---|
| ≈ 0.22 | ≈ −0.81 | bounds too tight: the car has more steering than the QP uses |
| ≈ 0.33 | ≈ −1.21 | stock gain is right, current bounds are correct |
| ≈ 0.49 | ≈ −1.81 | bounds still too loose: the QP commands into servo saturation |

### Read the headline number with these corrections

- **Odometry speed is ~17% short.** On the run archive the LiDAR measured 1.20× the distance
  `/odometry/filtered` reported (2026-09-11). That is consistent with `speed_to_erpm_gain`
  moving 4614 → 5499.27 in commit `1099733`. The fit regresses yaw rate on the logged
  `v × steer_cmd`, so a `v` that is 1.2× too small **inflates the implied gain's magnitude by
  about 1.2×**. Until that gain is fixed, divide the printed implied gain by ~1.2: −0.97 means
  ≈ −0.81, and −1.46 means ≈ −1.21.
- **The logged heading is partly the model under test.** The local EKF fuses `/odom`'s yaw
  differentially, and `/odom`'s yaw is integrated from the commanded servo through −1.0926 and
  L = 0.305. The gyro dominates (the EKF's yaw gain against the LiDAR measured 0.93–1.09). Still,
  treat implied-gain differences of about 10% as unresolvable from this data alone. Run with
  `localization_source: ekf`. Under `raw_odom` the heading is entirely servo-derived, and the
  steering fit just recovers the configured gain.
- **Wheelbase.** The implied gain scales with `--lf`. The repo holds two values: 0.3302 (the
  F1TENTH spec, pinned by `steering_offset_calibration_node`) and 0.305 (`vesc.yaml`'s "about
  30.5 cm", which is also what the MPC model uses). They differ by 8%, less than the gap between
  rows of the table. Measure the axle-to-axle distance during step 2.
- **One gain, both sides.** The fit returns a single effective gain. Left/right asymmetry is
  invisible to it, and step 2 measures each side separately.
- **Small-angle model.** The script uses `delta`, not `tan(delta)`. `tan(delta)/delta` is 1.027
  at 0.278 rad and 1.063 at 0.42 rad, so `Lf_eff` comes out a few percent low (implied-gain
  magnitude a few percent high) when steering sits near the bounds. The controller's own model
  keeps `tan(delta)` (`vehicle_model.py`). Account for this; do not edit the script.

## What gets logged

The logging lives in `src/f1tenth_control/mpc_controller/mpc_controller/MPC_corr.py`,
`MPCController.control_loop`, and writes through `model_log.py`. The file has one header row
`t,x,y,psi,v,steer_cmd,accel_cmd`, then one row per **solved** control step.

| column | exactly |
|---|---|
| `t` | `header.stamp` of the odometry message the solve started from, minus the node's clock at start [s]. The message stamp rather than the receipt time, because it is closer to when the state was true. Real stamps, never the nominal `ts`. |
| `x`, `y`, `psi` | The solver's initial state `x0`, in the **odom frame**. It comes from `/odometry/filtered` (local EKF, 50 Hz) under `localization_source: ekf`, from `/odom` under `raw_odom`, and from `/model/virtual_robot/odometry` only while hardware odometry is stale. Not the map frame, and never `/slam/pose` (~2 Hz). |
| `v` | Forward speed from the same message [m/s]. |
| `steer_cmd` | `delta_cmd` [rad] on the **model side** of the servo mapping, in ROS convention (+ = left, same sense as `psi`). This is exactly the `/drive` `steering_angle`, before `ackermann_to_vesc_node` applies the negative servo gain, so `steering sign +1` is the one that should fit. |
| `accel_cmd` | `a_cmd` [m/s²], the model's acceleration input. The car is actually sent a **speed**, `v + a_cmd·ts`, through the VESC speed loop, so the throttle fit measures how that loop realises `a_cmd`. |

Facts about this controller that the analysis depends on:

- **The control rate is 10 Hz** (`ts = 0.1` s): a timer that solves from the latest odometry
  message. Expect `dt` ≈ 100 ms.
- **Ticks that do not solve are not logged**: no odometry, `/mpc/hold`, goal reached, no goal.
  Those ticks publish a zero speed with no model acceleration, and a stretch of them shows up as
  one long `dt`.
- **A step that reuses the previous odometry message is skipped**, because its `dt` would be 0
  and the script divides by it. The interval reappears as a doubled `dt`, and the node warns.
- **The log records the MPC's command, not the car's.** A row is valid only while the MPC's
  command is what reaches the car. If teleop (mux priority 100) or the safety lane (200) takes
  over mid-run, those rows are wrong.

## Producing a log

1. Build: `colcon build --symlink-install --packages-select mpc_controller f1tenth_control f1tenth_params`
2. Run the stack as usual (`supervisor_bringup.launch.py` or `stack_bringup.launch.py`). Logging
   is on by default.
   - The path is `mpc_model_log_path` in `f1tenth_params/config/stack_params.yaml` (default
     `mpc_log.csv`), handed to `mpc_corr` as its `model_log_path` parameter. To override it,
     pass `mpc_model_log_path:=/abs/path/drive1.csv` on the `ros2 launch` command line that
     brings up `mpc_corr`. `mpc_model_log_path:=''` disables logging entirely.
3. Drive **2–3 minutes under MPC**, with varied steering and speed. Gentle S-curves at two or
   three speeds beat one fast lap, because the parameter fits need excitation. Mission driving
   concentrated near centre steering is exactly where the gain is least identifiable. Prefer
   one continuous `drive` move, so that holds and goal arrivals do not break the log into gaps.
   - **Manual control is not covered by this log.** The identification does not care who
     steers, but this logger records the *MPC's* commands. A teleop drive would pair real motion
     with commands the car never received. For a manual drive, record a mission-logger bag
     instead: it carries `/odometry/filtered` and `/ackermann_drive`, which is what the VESC
     actually got. Converting that bag to this CSV format is not built yet.

**Where the log lands:** the working directory of the `mpc_corr` process, not the source
directory. `ros2 launch` inherits the directory of the shell that started it, and the component
supervisor's launches inherit the supervisor's (it passes no `cwd`). The node prints the absolute
path at startup (`model log -> "..."`). The file is **truncated on every node start**, including
supervisor auto-restarts of `navigation`, so copy it off before relaunching.

To check it: `head -3 mpc_log.csv` should show the header and plausible rows, and `wc -l
mpc_log.csv` should grow by about 10 lines per second while the MPC is solving.

## Running the script

numpy is required; matplotlib is needed only for `--plot` (see `tools/requirements.txt`). The
script's defaults are full-scale-car values, so **always pass `--lf 0.3302`**, and a `--gain`
that matches what `steering_calibration.yaml` currently holds (−1.2135 today).

```
python tools/mpc_model_check.py mpc_log.csv --lf 0.3302 --gain -1.2135
python tools/mpc_model_check.py mpc_log.csv --lf 0.3302 --latency 0.10 --plot out.png
python tools/mpc_model_check.py --demo /tmp/demo.csv
```

Run the self-test first. `--demo` writes a synthetic log with known faults: Lf 2.90, steering
gain 0.80 (so Lf_eff 3.625), offset +0.020 rad, latency 150 ms, drag −0.05.
`python tools/mpc_model_check.py /tmp/demo.csv --lf 2.67` should recover them. On 2026-09-11 it
printed:
- effective wheelbase 3.681 m (R² 0.966);
- offset +0.0203 rad;
- best-fit latency 140 ms;
- drag −0.043;
- steering sign +1 (heading RMS 0.00987 rad) far better than −1 (0.05253).

## Reading the output, in priority order

1. **Sign line.** If `-1` fits much better than `+1`, fix that first. Nothing else in the output
   means anything until the sign is right.
2. **`dt` spread.** The mean should be about 100 ms. A max far above the mean means dropped
   frames, a solver overrun (the node also warns when a solve exceeds `ts`), skipped duplicate
   states, or a non-solving stretch. Wherever that happens, the fixed-`dt` assumption is violated.
3. **Implied servo gain versus the configured −1.2135.** This is the headline number. Read it
   through the corrections above, including ÷ ~1.2 for the odometry speed bias.
4. **Steering offset.** It should now come out near zero, because the offset was already
   retuned. If it does not, the retune did not hold. The bag-to-bag spread behind that retune
   (sd 1.23 deg against a 2.68 deg effect) points at mechanical toe drift, which would mean a
   recalibration cadence rather than a one-time fix.
5. **Latency.** A minimum away from zero must be compensated by rolling the measured state
   forward before the solve. It is the usual cause of weaving that gets worse with speed.
6. **Lateral-acceleration check.** If heading error fans out with `a_lat`, the car is slipping.
   The next step is then a dynamic single-track model, not more tuning.

### Trusting the fits

Check the reported R². A steering fit around 0.97 means those parameters are trustworthy. A low
R² means "do not believe these numbers", not "the subsystem is broken". Usually the drive lacked
excitation in that channel.

### The estimator caveat

The method assumes the logged state is the true state at time `t`. The logged state here is the
local EKF (`/odometry/filtered`, 50 Hz), not raw sensors and not `/slam/pose` (~2 Hz). Any lag
the estimator adds gets attributed to actuator latency by the sweep. It still matters for
control, but it is fixed in the estimator, not the actuator.

## What the follow-up fixes look like

None of these touch the cost function; they all live outside the solver.

- **Wrong gain or effective wheelbase:** a constant. Set the measured
  `steering_angle_to_servo_gain_left/_right` in `steering_calibration.yaml` (step 2), then
  re-derive the MPC bounds from it.
- **Steering offset:** a subtraction on the command (the servo offset).
- **Latency:** forward-propagate the initial state by the measured latency before the solve.

## Notes for step 2 (calibration circles)

- **Drive slowly, 0.5–1 m/s.** At full lock with R ≈ 0.75 m, 1 m/s gives 1.3 m/s² of lateral
  acceleration, so slip is negligible and `delta = atan(L/R)` holds. At 3 m/s the same circle is
  12 m/s², and you would be measuring tire slip instead of geometry.
- **Mark the rear axle centre**, since that is the bicycle model's reference point.
- **Measure the diameter**, not the radius.
- **Measure both sides.** Symmetric servo bounds only give symmetric steering if the two gains
  are equal, and that is exactly what is being tested.
- **Measure the wheelbase while you are there.** `atan(L/R)` needs it, and the repo disagrees
  with itself (0.3302 vs 0.305).

## Controller facts found while adding the logging

These are reported, not changed.

- **Steering rate is bounded, more tightly than the servo allows.** The QP has a hard
  `dDeltaMin/dDeltaMax` of ∓0.5 rad/s, which is ±0.05 rad per 0.1 s step, measured from the
  previous command.
  - The servo's rated 3.2 rad/s is 0.32 rad per step at this rate, so the solver cannot plan
    steps the servo cannot follow.
  - The opposite risk applies instead. A full −0.283 → +0.278 sweep takes the QP 1.1 s, where
    the servo needs about 0.18 s.
  - The code itself marks 0.5 rad/s as an unmeasured guess.
- **Other steering sources can exceed the MPC bounds.**
  - Teleop: `joy_teleop.yaml` scale 0.34 rad, mux priority 100.
  - The Nav2 bridge `twist_to_ackermann_node`: −0.264/+0.314, on the same mux input as the MPC,
    active only with `enable_nav2`.
  - `steering_offset_calibration_node`: −0.264/+0.314, priority 50.
  - The startup sweep (0.18) and the safety stop (0.0) stay inside the bounds.
  - `vesc_driver`'s servo clamp (`servo_min`/`servo_max`) saturates all of them at the hardware.
- **No raceline or global planner is on the default path.** The curvature demands that do exist:
  - `goal_turn` with `steering: full_lock` places its target as if the car could turn at
    `R = 0.305 / tan(35°) ≈ 0.44 m`, but the QP bound allows only `R ≈ 1.07 m` (curvature
    2.3 vs 0.93 rad/m).
  - Nav2's Smac planner uses a placeholder `minimum_turning_radius: 0.6 m`, which would need
    ≈ 0.47 rad of steering. It only applies when `enable_nav2` is on.
