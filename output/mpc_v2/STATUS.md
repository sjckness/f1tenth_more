# Reverse on `mpc_corr_v2_reverse` — status

Last updated 2026-10-20, after the 15:37 run on the car.

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

- **Reverse runs on the car.** Three manoeuvres in the 15:37 run, ending on
  `timeout`, `bt_withdrawn: bt_disallowed` and `rear_clearance`.
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

- **Odometry may not see the reverse.** `v_real` stayed at zero through all
  three manoeuvres. The only deadband left in the chain is `vesc_driver`'s
  `erpm_deadband: 500` (≈0.11 m/s, symmetric), which legitimately zeroes a
  slower crawl. Window 1 ended on `timeout` rather than `distance_done`, so
  `travelled` never reached 0.5 m — consistent with either a genuinely slow
  reverse or a frozen odometry pose. **Undecided: no bag was recorded.**
- **One reverse ended on `bt_disallowed` after 0.5 s.** The BT withdrew
  permission mid-manoeuvre. Which condition fired is not in the log — the
  candidates are a side/rear proximity trip or a front reading under
  `front_hard_floor_m` (0.20 m), both of which call `blocks_reverse()`.
- **VESC reverse ERPM polarity is unverified.** Whether the controller reports
  *negative* ERPM when the wheels turn backwards has never been observed; every
  sample on record is positive. `vesc_to_odom_backup.cpp` notes the sign may
  depend on hall state. If reverse reads back as a positive speed, that is
  where it comes from, and no code change in this repo will fix it.
- **Steering during reverse is unaudited against data.** The model uses a
  signed `v = -|v_rev|` and the rear corridor is expressed in the car's heading
  frame, which agree; nothing in the output chain flips steering on a negative
  speed. But no run has measured yaw rate during a reverse, so this is a code
  argument, not a measurement.

## Next run

```
ros2 bag record /drive /ackermann_drive /commands/motor/speed /commands/servo/position \
  /sensors/core /sensors/imu/raw /odom /odometry/filtered \
  /mpc/reverse_status /bt/allow_reverse /mpc/drive_clamp
```

`/sensors/core` settles the ERPM polarity, `/odom` and `/odometry/filtered`
bracket the EKF, and `/sensors/imu/raw` gives the yaw rate the steering
question needs. The `CTRL` line now reports every tick's published command with
its source and the manoeuvre's `travelled`, so the log alone answers what each
reverse commanded.

## TODO

- **Reverse on command.** `/mpc/request_reverse` (`RequestReverse`) is wired and
  works with `reverse_auto_enabled` either way, but nothing calls it. A
  deliberate operator-triggered reverse would make all of the above testable
  without having to get the car stuck first.
