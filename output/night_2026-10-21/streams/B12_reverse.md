## Stream B (B1 + B2): VESC version and MPC reverse leftovers

Reverse failed because of the MPC, not the VESC. The car reverses and odometry sees it. The first test ended on `timeout` because of how the manoeuvre counted the distance it had backed up. `night-reverse` fixes that count.

- `dd129b9` counts the distance in odom.
- After review, `6536877` makes it the larger of the odom count and the old map count.
- The two docs commits only change STATUS.md.

### B1: which VESC the car runs
- **vesc commit on each branch:**
  - `0e74a60`: campaign-final, origin/mpc_corr_v2_reverse, pwa-control, night-reverse.
  - `2848a9f`: main, local mpc_corr_v2_reverse, origin/jazzy.
  - `0e74a60` is already on GitHub (`git ls-remote`: `refs/heads/campaign-final` and `refs/heads/scene-graph` of sjckness/vesc), so no push is needed.
- **Orin, checked read-only:**
  - `~/dev_ws/f1tenth_more` is on campaign-final `1ee82c8`, with vesc checked out at `0e74a60` and no tracked source changes.
  - The stack runs natively. `install/` is symlinks, and the Python packages are egg-links.
  - `libvesc_ackermann.so` was built on 10-20 at 16:03 from `0e74a60` source dated 16:02.
  - `vesc_driver` was built on 09-07 and is the same in both commits.
  - So the car runs `0e74a60`, but the 15:37 run on 10-20 still ran the 2848a9f-era `vesc_to_odom_backup`.
- **Diff 2848a9f..0e74a60:** it only removes two symmetric deadbands in `vesc_to_odom_backup` (500 ERPM and 0.05 m/s). Speed is still `(erpm - offset) / gain`, sign preserved.
- **The command chain passes the sign through unchanged:**
  - The launched `vesc.yaml` is a symlink into src.
  - Gain is 4516.9 and offset 0 (`vesc.yaml:14-15`); `erpm = gain*v` (`ackermann_to_vesc.cpp:135`).
  - There are no velocity-correction knots.
  - The command is clipped to ±23250 ERPM (`vesc.yaml:96-97`, `vesc_driver.cpp:366`).
  - Nothing publishes the brake or current topics.
  - The telemetry deadband is 500 ERPM, about 0.11 m/s (`vesc_driver.cpp:215`).
  - ERPM polarity is negative in reverse: `/odometry/filtered` reached -0.36 m/s.
- **gtests on linus at `0e74a60`:** `test_odom_speed` 5/5, `test_velocity_correction` 13/13.
- **No VESC Tool or firmware value needs to change.**

### Why the first test "failed"
Source: the Orin's `~/.ros/log`, read with read-only grep scripts.

- **The 15:39 timeout was not the first reverse.** The first two reverses of 10-20 (15:35:05 and 15:35:10, in the 15:33 run) ended `distance_done`.
- **15:37 run:**

| Reverse | How it ended | Odom distance back |
|---|---|---|
| 15:39:09 | `timeout` | 0.77 m at the timeout tick (`SOLVE/in` 15:39:11.79); 0.81 m at rest after braking. The gap to the obstacle ahead grew 0.60 m. |
| 15:39:25 | `bt_disallowed` | – (`IsProximityTooClose`) |
| 15:39:28 | `rear_clearance` | 0.71 m at its end tick; 0.75 m at rest |

- **Cause:** `travelled` was the displacement in the map frame, from a start pose held in map and re-read through map->odom on every tick.
  - With odom 0.77 m back and less than 0.5 m counted, map->odom must have moved at least about 0.27 m with the car.
  - This is inferred: map->odom is not logged.
- **Evening runs, 17:52-18:24 (13 reverses):** 12 ended `distance_done` and 1 `rear_clearance`.
  - `v_real` was -0.32..-0.36 m/s for a -0.40 command.
  - The map count sat at 0.000 for 1.0 s while the car moved, and once jumped by up to 0.45 m in one tick.

### B2: MPC reverse leftovers
- `git log campaign-final..mpc_corr_v2_reverse` is empty, both for local `a14ec2f` and for origin `4f53ab8`.
- **The forward solver cannot output v < 0. This was already done in campaign-final (`a8a03ea`):**
  - `vMin` is 0.0 (`MPC_corr.py:1563`).
  - The speed-channel lower bound is 0 (`speed_channel.py:562`).
  - A final gate in `_publish_drive` (`MPC_corr.py:4767`) stops any negative outside the manoeuvre, and it is the only `/drive` publish (`:4825`).
- **New, in `reverse_manoeuvre.py`:**
  - `travelled = max(0, in_odom, in_map)` (lines 665-667):
    - `in_odom` is dead reckoning from the pose where the reverse began (stored at :642).
    - `in_map` is the old map-anchored count.
  - Either count can end the reverse, and neither can push it past the 0.5 m strip the precheck validated.
  - The rear corridor and the strip check stay anchored in map.
  - `dd129b9` alone (odom count only) would let a reverse with frozen odometry run 0.9 m to its timeout. Reviewer 2 found this, and `6536877` fixes it.
- **Tests** (`test_reverse_manoeuvre.py`):
  - :434: the corridor stays put in map.
  - :442: a map->odom correction does not take back distance already backed up in odom.
  - :465: replays the 15:39 case: localisation holds still in map and odom moves.
  - :476: frozen odom, with the localisation tracking the car.
  - Both cases end `distance_done` at 0.5 m.
  - Mutation check: with the odom-only count, :476 fails. With the map-only count (campaign-final's behaviour), :442 and :465 fail; :465 ends on `timeout`, the car's symptom.
- **James's origin/jazzy commits `a596962` and `4b9b0d4` were not ported.**
  - They implement reverse as a negative `vdes` in the forward solver, which campaign-final's guards block.
  - They also edit `mission_config.py` (BT/mission schema).
  - The reverse path on campaign-final is `/mpc/request_reverse`.
- **`output/mpc_v2/STATUS.md`** (`113ee4c`, `3b8b0b8`):
  - Marks the odometry, ERPM-polarity and `bt_disallowed` items resolved.
  - Corrects the distances to the end-tick values.
  - Adds `/tf`, `/tf_static`, `/ekf_global/odometry/filtered` and `/slam/pose_calibrated` to the next-run bag command.

### Verification
- **`test_reverse_manoeuvre.py`:** 42 passed.
- **All reverse-related test files (11):** 237 passed, 2 xfailed, 1 failed. The failure is known pre-existing #3 (`test_drive_limits` go_to speed).
- **Full mpc_controller suite (pytest on linus, Jazzy):** 21 failed / 1381 passed / 1 skipped / 2 xfailed. The same 21 fail on the night-start/campaign-final baseline, and none of them involve reverse.
- **Lint:** flake8 is clean on the changed files. pep257 shows only the pre-existing D213 at `reverse_manoeuvre.py:111`.
- **The code is Python 3.10 and Humble safe:** plain `math` only, no new parameters, files or entry points.
- **Not verified on the car:** this fix, hard_stop's reverse gate (`b31155e`), and the real reverse distance on the floor.

### Scope note for the lead
`dd129b9` + `6536877` change how every reverse ends, with no flag. They can be dropped together: `6536877` depends on `dd129b9`. The docs commits can be dropped on their own.

### For tomorrow on the car
- `git pull` on the Orin is enough for `mpc_controller`, because it is installed as an egg-link.
- Measure one reverse with a tape.
- Bag `/tf` and the global EKF/SLAM pose topics during a reverse to observe map->odom directly.