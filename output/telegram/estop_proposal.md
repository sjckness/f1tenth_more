# Proposal: an e-stop that holds the mission, and a reset service

Status: proposal only. Nothing in `f1tenth_behavior` was changed. Line
numbers are for `telegram-control` (= `campaign-final` `cd180d8` for these
files).

## The problem

**What the latch does today.** `/mission/emergency_stop` only latches a flag
(`mission/loader.py:609-639`). By design it does not abort the mission and
does not set `/mpc/hold` (`loader.py:44-53`).

**What holds the car meanwhile.** Only the behaviour tree's emergency lane,
which publishes zero on `/safety_stop` at 10 Hz:
- source: `behavior_executor_node.py:547`, read through
  `IsEmergencyStopTriggered` (`behaviours/is_emergency_stop_triggered.py:59-66`);
- the mux lane has priority 200 and a 0.2 s timeout.

A RUNNING mission's mpc_corr keeps publishing its move on `/drive`
underneath.

**Measured in the sim (2026-10-20, `output/telegram/test.md` §7):**
- A plain restart of `behavior`, the only documented way to clear the latch
  (`MissionStatus.msg:23-26`), removed that lane. A lower lane asking for
  0.3 m/s then moved the car **1.53 m**.
- Even while latched, at load 17–28, the tree's 10 Hz zero leaks: single
  lower-lane commands pass the mux, a few mm to cm.
- If `behavior` crashes while latched, the hold disappears with it. The
  supervisor's auto-respawn then starts a node whose latch reads clear.

`f1tenth_ctl` works around this from outside:
- `estop` also calls `/mission/abort_mission`;
- `estop-reset` holds zero drive, aborts, restarts `behavior` and checks the
  latch.

Both are best effort over ssh, seconds late. The fix belongs in the stack.

## The proposal

1. **`emergency_stop` also ends the mission and holds the MPC, in the same
   locked section.** It does what `_on_abort_mission_service` does
   (`loader.py:566-607`):
   - for LOADED, RUNNING or HOLDING: `state.abort()`;
   - always: `self.hold_pub.publish(Bool(data=True))`.

   mpc_corr then holds at zero on its own (`MPC_corr.py:4097-4103`). The car
   stays stopped even without the tree's lane, so a crash or restart of
   `behavior` no longer releases it.
2. **A new `/mission/reset_estop` (`std_srvs/Trigger`).** It clears
   `_emergency_stop_active` in place, instead of restarting the node. It
   refuses (`success=false`) unless all of these hold:
   - the latch is set;
   - no mission is LOADED, RUNNING or HOLDING (the e-stop has aborted it);
   - the car is stopped: odometry speed below a threshold (e.g. 0.05 m/s)
     for ≥ 1 s, read from the odometry the BT already has on its
     blackboard.

   On success it publishes `/mission/status` with
   `emergency_stop_active=false`. `/mpc/hold` stays true until the next
   mission start releases it (`loader.py:529`), so nothing moves after a
   reset.
3. **Keep the rule that `start_mission` is refused while latched**
   (`loader.py:380`, `:462`).

## Risks

- **Semantics change.** The e-stop now also ends the mission. Today a mission
  interrupted by the e-stop stays RUNNING, but it can never resume anyway:
  the latch outlives it and start is refused. Logging changes: the
  test-campaign logger will see a mission-end event, with reason
  `aborted via /mission/emergency_stop`, where today it sees none.
- **A reset is a new way to unlatch.** The "car stopped" condition must not
  be trivially true when odometry is stale. With no odometry for > 0.5 s,
  refuse the reset.
- **Comments and docstrings that say "no reset service exists on purpose"**
  must change with the code:
  - `loader.py:44-53`, `:209`;
  - `is_emergency_stop_triggered.py:13-16`;
  - `MissionStatus.msg:23-26`.
- **Two writers of `/mpc/hold`?** No: it is still only the loader.

## Files and functions to touch

| File | Where | Change |
|---|---|---|
| `src/f1tenth_behavior/f1tenth_behavior/mission/loader.py` | `_on_emergency_stop_service` :609-639 | abort the mission + `hold_pub.publish(True)`, as `_on_abort_mission_service` :566-607 does |
| same | `__init__` :257-258 | create `/mission/reset_estop` next to `/mission/emergency_stop` |
| same | new `_on_reset_estop_service` | the conditions above; clear the flag; `_publish_status()` :704 |
| same | docstrings :44-53, :209 | the reset exists now |
| `src/f1tenth_behavior/f1tenth_behavior/behaviours/is_emergency_stop_triggered.py` | docstring :13-16 | it no longer stays SUCCESS for the node's lifetime |
| `src/f1tenth_messages/msg/MissionStatus.msg` | comment :23-26 | latched until `/mission/reset_estop` or a restart |
| `src/f1tenth_bringup/f1tenth_bringup/reset_manager_node.py` | `/reset_all` | decide whether it should clear the e-stop too (probably not) |

## Tests that change

- `test/test_mission_countdown.py`:
  - `test_emergency_stop_during_countdown_never_starts` (:224) and
    `test_start_is_refused_while_the_emergency_stop_is_latched` (:336) stay;
  - add a test that the e-stop aborts a RUNNING mission and publishes
    `/mpc/hold` true;
  - add a test that start is allowed again after a successful reset.
- `test/test_every_abort_holds.py`: add the e-stop as a fifth abort path that
  must hold.
- New `test_reset_estop.py`, the reset is refused:
  - while moving;
  - with stale odometry;
  - while not latched;
  - while a mission is still active.

  It succeeds when stopped, and the status republishes clear.
- `test/test_battery_low_runtime_estop.py`: unchanged. The emergency lane is
  the same.

## How f1tenth_ctl uses it

- `estop-reset` already prefers a reset service when one is configured
  (`rosprobe.choose_reset_path`). Set it in `ctl.env`:
  `F1T_ESTOP_RESET_SERVICE=/mission/reset_estop`.
- The rest of the sequence stays: zero drive once the mux matched, then
  `/mission/abort_mission`, then the reset, then the check that
  `/mission/status` reads clear. There is no restart and no gap in the
  emergency lane.
- `estop` needs no change. Its `abort_mission` step becomes redundant but
  harmless ("no mission to abort").
