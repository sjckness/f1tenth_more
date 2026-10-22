## Stream B3: reverse in the app (pwa-control)

Branch `pwa-control`, worktree `~/dev_ws/f1tenth_more_pwa`, **HEAD `6b10c5b`**. There are 11 commits: 7 from the implementer and 4 review fixes. Nothing was pushed, and the `night-start/*` tags were not touched. `night-start/campaign-final` is an ancestor of HEAD; night-reverse is not merged.

### Merges (instead of the rebase)
- **`876fe12`** merges origin/pwa-control (7f3be61..adb9143). It had 4 conflicts, all additive, and I kept both sides:
  - `ui.tsx` and `ws.ts`: union of the store imports.
  - `store.ts`: both the MOCK block and the QR-pairing block.
  - `test_app_mock.py`: both sets of tests.
  - A reviewer checked the result: each parent's side-diff is intact.
- **`23e11e0`** merges night-start/campaign-final (6002cb0) with no conflicts. Submodule pointers are identical to 6002cb0.

### The "15 s before Start" check
- **adb9143 is a different wait.** `F1T_MISSION_DISCOVER_S=15` is a ceiling: `rosprobe.py:704-709` leaves the loop as soon as `/mission/status` and the service are ready. I did not change it.
- **The stale 15 s was f1tenth_ctl's own copy of the countdown.** Fixed in **`04db46f`**:
  - `missions.armed_countdown` (`missions.py:207`) reads the loader's own answer.
  - The load guard (`cli.py:669-672`) and `describe_state` use what the start actually armed.
  - The env value and its default are now 0.
- **Review fix `0b23bf1`:** Telegram still printed 15 s. `tg.py` and `cli.py` formatted the countdown as `countdown_s or 15`, so 0.0 printed as 15. The bot told the operator "15 s countdown" for a car already moving. Now `flow.when_it_moves` (`flow.py:45`) produces "it moves now, no countdown" for 0, and the README is corrected.
- **Still open:** a start from the app runs two fresh DS participants (`app.py:452,456`). That is about 12-20 s from AVVIA to motion; see open issues.

### Gateway reverse (`4d5ff3f`, tests strengthened in `82f92ec`)
- **Mapping** (`teleop.py:110`, `speed_for`):
  - |thr| ≤ 0.15 gives 0.
  - Below that, the speed is linear down to `-min(max_reverse_speed_mps, limit)`.
  - New params: `max_reverse_speed_mps` 0.4 (0 turns reverse off), `reverse_hold_s` 0.3, `stop_speed_mps` 0.05.
  - There is no 0.4 floor backwards. The reverse command ramps up at `accel_mps2` and drops to 0 at once.
- **Through zero** (`_must_wait`, `teleop.py:258`): at a direction change the command holds 0 for at least 0.3 s, and longer while odometry less than 0.5 s old shows motion the other way above 0.05 m/s.
- **Unchanged from forward:** dead-man, 250 ms watchdog, and release = immediate zero.
- **For the app:** hello carries `max_reverse_speed_mps`; the telemetry `drive` snapshot carries `reverse` and `reverse_wait`.
- **Docs:** SPEC §5 rule 5 and the gateway README replace "Retromarcia non abilitata nella v1".
- **Tests in `test_teleop.py`:**
  - -0.8 maps to -0.306 and -1.0 to -0.4;
  - cap and limit, 0 = off, ramp, hold, measured wait, stale odometry, release and watchdog;
  - a coupling test against stack_params;
  - added by `82f92ec`: an upper bound on the measured wait, 0.03 m/s counting as stopped, release-in-reverse checked at -0.4, and the hold counted from the release.
- **Mutation check:** all 14 reviewer mutants are now caught (previously 11).

### UI (`6bd75b7`, `6b10c5b`)
- **Indicator:** an amber **RETROMARCIA** pill (`Guida.tsx:165`). It is always laid out and only made visible, and shows "attesa arresto" during the hold. THR and CMD turn amber.
- **Review fix `6b10c5b`:** the pad maps the absolute thumb position, and its GAS label sat in the full-reverse zone. In v1 the lower half was a safe resting place. With reverse on, the pad's ends now read **AVANTI / RETRO** (amber), and the SPEC says so. I checked screenshots at 844x390.

### Pre-existing failures from origin, fixed
- `30b76c4`: flake8.
- `ea35899`: the stale e2e test "connection lost".

### Verification
| What | Result |
|---|---|
| Gateway, linus (venv, worktree install, domain 76) | 97 passed |
| f1tenth_ctl, linus | 113 passed |
| f1tenth_telegram, linus | 85 passed / 17 skipped with Jazzy python3; 101 passed including test_tg with the telegram venv interpreter |
| **Humble / Py 3.10** (docker f1tenth_more:cpu) | gateway 97 passed; ctl 98 passed, 15 skipped; telegram 85 passed, 18 skipped |
| tsc and `npm run build` | ok |
| Playwright, all 3 projects | 28/28 in 1.3 min |
| New tests against the pre-fix code | each fails (Telegram 3, ctl 1, teleop mutants 3/3) |

**Not verified:** anything on the real car. That covers VESC reverse (B1), what the car does at small reverse speeds, and DS discovery times.

### Deploy (no sudo)
1. On the Orin: pull pwa-control, then `systemctl --user restart f1tenth-gateway`.
2. On linus: `npm run deploy` in `apps/pwa`.
3. Telegram bot: update its checkout `~/dev_ws/f1tenth_more_telegram`, then `systemctl --user restart f1tenth-telegram`.

The mock gateway (PID 3095461) was not restarted.