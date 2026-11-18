# Reverse on `mpc_corr_v2_reverse` — status

Last updated 2026-10-21 (night), from the mpc_corr and BT logs of the
2026-10-20 runs in `~/.ros/log` on the Orin. Earlier text: after the 15:37 run.

Two reverse mechanisms exist in the code and are **mutually exclusive** —
`mpc_corr` refuses to start with both on (`validate_reverse_mechanisms`,
commit `f8844004`).

| mechanism | param | state |
|---|---|---|
| stuck manoeuvre (`reverse_manoeuvre.py`) | `reverse_auto_enabled: true` | **live** |
| multi-candidate QP (`reverse_mpc.py`) | `allow_reverse: false` | off, prototype |

The live one is decided by `mpc_corr` when it is stuck. The BT never starts a
reverse; it only grants permission on `/bt/allow_reverse` and stops blocking
(`bt_reverse_integration: true`, needed — with it false nothing publishes that
permission and no reverse can ever start).

## What works

- **Reverse runs on the car, and odometry sees it.** The first two reverses of
  the day, in the 15:33 run (15:35:05 and 15:35:10, BT log), both ended
  `distance_done`. Three more in the 15:37 run, ending on `timeout`,
  `bt_withdrawn: bt_disallowed` and `rear_clearance`, with
  `/odometry/filtered` at -0.23 to -0.36 m/s throughout (`ODOM/hw` and
  `SOLVE/in` lines). Thirteen more in the 17:52-18:24 runs, with
  the per-tick `CTRL` line: 12 `distance_done`, 1 `rear_clearance`, `v_real`
  -0.32 to -0.36 m/s for a -0.40 command, moving 0.18-0.21 s after it.
- **The speed limit is lifted.** `max_reverse_speed_mps` 0.0 → 0.4, equal to
  `v_rev_mps`, so the manoeuvre's own permission check no longer refuses with
  `reverse_speed_limit`.
- **The BT sees the reverse.** `/mpc/reverse_status` is now published on both
  mechanisms; previously the `allow_reverse` path never published it, the
  monitor read `unknown`, and the front cone zeroed the command on the
  `safety_stop` lane.
- **The command chain is sign-transparent, end to end.** `ackermann_mux`
  republishes unchanged, `correctVelocity` is symmetric on the magnitude,
  `vesc_driver` clips to `speed_min -23250`, and `erpmToSpeed` is a pure odd
  function pinned by `test_odom_speed.cpp`.
- **Reverse cannot leak into forward.** The forward solver's `vMin` is 0.0 and
  `_publish_drive` refuses a negative from anything but the running manoeuvre.
- **The manoeuvre cannot run away.** A wall-clock cap independent of measured
  speed: `timeout_factor * (requested / v_rev_mps)` = **2.50 s** today.

## Open

- **Resolved: "odometry may not see the reverse".** It did. "`v_real` stayed
  at zero" came from the old `CTRL` line, which printed only forward ticks.
  The `timeout` reverse (15:39:09) was 0.77 m back in odom at the timeout
  tick (0.81 m at rest after braking), and the clearance to the obstacle
  ahead grew 0.60 m; the `rear_clearance` one (15:39:28) was 0.71 m back at
  the tick it ended (0.75 m at rest). Both passed 0.5 m without
  `distance_done` because `travelled` was the displacement in MAP (the start
  held in map, re-read through the current map -> odom): with odom 0.77 m
  back at the timeout tick, a count under 0.5 m means map -> odom had moved at
  least about 0.27 m with the car (inferred, the edge is not logged). On the
  evening runs the same count sat at 0.000 for 1.0 s of motion and jumped up
  to 0.45 m in one tick. `travelled` is now the larger of two counts: dead
  reckoning in odom from the start pose, and the old map-anchored one, so
  either can end the reverse and neither can stretch it past the 0.5 m strip
  the precheck validated (odometry frozen under vesc_driver's ERPM deadband,
  or map -> odom following the car). The rear corridor stays anchored in map
  (`reverse_manoeuvre.py`, `_reverse_tick`).
- **Resolved: the `bt_disallowed` reverse.** The BT log names it:
  `/bt/allow_reverse False: IsProximityTooClose blocks the reverse` at
  15:39:25.48, 0.20 s before the manoeuvre ended.
- **Resolved: VESC reverse ERPM polarity.** Negative: `/odometry/filtered`
  went to -0.36 m/s, and `vesc_to_odom_backup` is `(erpm - 0) / 4516.9`.
- **Steering during reverse: one window agrees, no more.** The model uses a
  signed `v = -|v_rev|` and nothing in the output chain flips steering on a
  negative speed. In the 15:39:28 reverse (`SOLVE/in`, 15:39:29.4-30.7) the
  EKF yaw fell 0.286 rad in 1.30 s (-0.22 rad/s) under a mean steering of
  +0.197 rad at about -0.34 m/s; `v / L * tan(delta)` with L 0.305 m gives
  -0.22 rad/s. Same sign, same size, one window.

## Next run

```
ros2 bag record /drive /ackermann_drive /commands/motor/speed /commands/servo/position \
  /sensors/core /sensors/imu/raw /odom /odometry/filtered \
  /tf /tf_static /ekf_global/odometry/filtered /slam/pose_calibrated \
  /mpc/reverse_status /bt/allow_reverse /mpc/drive_clamp
```

`/tf` (map -> odom from the global EKF), `/ekf_global/odometry/filtered` and
`/slam/pose_calibrated` show directly whether map -> odom follows the car
during a reverse, the one inferred link above. `/sensors/core`, `/odom` and
`/odometry/filtered` bracket the EKF, and `/sensors/imu/raw` gives the yaw
rate the steering question needs. Measure one reverse with a tape: odom moved
0.81 m in the 15:39:09 reverse while the clearance ahead grew 0.60 m. The `CTRL` line now reports every tick's published command with
its source and the manoeuvre's `travelled`, so the log alone answers what each
reverse commanded.

## TODO

- **Reverse on command.** `/mpc/request_reverse` (`RequestReverse`) is wired and
  works with `reverse_auto_enabled` either way, but nothing calls it. A
  deliberate operator-triggered reverse would make all of the above testable
  without having to get the car stuck first.
