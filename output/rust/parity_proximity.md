# Front-proximity guard: Python vs Rust parity

Branch `jazzy-rust` (jazzy-campaign + campaign-final 0c1cd41), linus, 2026-10-18.
Reference: `f1tenth_behavior/behaviours/is_proximity_too_close.py`, the FRONT check
(cpu-cleanup version: vectorised, event-driven stop). Port:
`src/rust/f1tenth_proximity_guard` (rclrs 0.7.0), node `proximity_guard_node`. Harness:
`tools/rust_parity/proximity/`.

**Result: the front decision differs on 0 of 207,251 inputs (3,184 synthetic, 123,811 recorded
car scans, 80,256 recorded scans with an obstacle injected near the threshold). The Rust stops
where the Python does not only through a named fail-safe rule; none of them fires on any
recorded scan.**

## 1. Recon: how the front check works today

**Correction to the task's premise.** The threshold is **0.15 m, measured from the front
bumper**, not 0.40 m. The 0.40 m derivation (car_radius 0.20 + obstacle_safety_margin_m 0.12 +
proximity_front_extra_margin_m 0.08) was dropped on 2026-09-01 (mission-analysis follow-up). The
threshold is now the direct stack_params key `proximity_front_threshold_m` = 0.15 (and
`proximity_side_threshold_m` = 0.15), "a genuine last-resort contact range, INSIDE car_radius
by design". `car_radius`, `obstacle_safety_margin_m` and `proximity_front_extra_margin_m` are
still declared, but nothing in IsProximityTooClose reads them.

| | |
|---|---|
| Input | `/scan` (sensor_msgs/LaserScan), subscribed RELIABLE depth 10 inside behavior_executor_node. Car: UST-10LX, 1081 beams over +-135 deg, 40 Hz (25 ms, max 31.5 ms in the 2026-10-15 bag), range_min 0.02, publisher reliable. Isaac: the same layout, +inf for no return |
| Laser pose | `StaticFramePoses`: base_link <- `header.frame_id` from **/tf_static only** (TRANSIENT_LOCAL depth 100, tf2 Buffer, no TransformListener), x, y and yaw, cached on first resolve. Car TF: laser (0.305, 0, 0.190), yaw 0 |
| Front cone | each valid return (finite, `>= range_min`) placed in base_link; kept if `dx = lx + r cos(lyaw+a) - front_x > 0` and its bearing **from the bumper** `abs(atan2(dy, dx)) < 45 deg`; `min_front = min hypot(dx, dy)`. `front_x` = base_front_x 0.405 (URDF `base_front.xacro`). The laser sits 0.10 m behind the bumper |
| Trip | `min_front < 0.15` (strict). The side/rear window (raw angle `abs(a) >= 45 deg`, raw range `< 0.15`) is the other half of the same condition; not ported |
| Where | the BT's emergency lane: `Selector(IsBatteryLow, IsEmergencyStopTriggered, IsProximityTooClose[, IsSystemOverheated])` -> `Stop(safety_stop, frame_id base_link/emergency)`. Present iff `enable_lidar_safety_stop` (true); never gated by mission phase |
| What it publishes | `ackermann_msgs/AckermannDriveStamped` speed 0, steering 0, frame_id `base_link/emergency`, on `safety_stop` (= `/safety_stop`): every BT tick (`bt_tick_hz` 10) while tripped, **and** from the /scan callback on every tripping scan (`enable_proximity_event_stop`, default true, cpu-cleanup), so about 40 Hz on the car |
| Lane | **ackermann_mux** (not twist_mux), lane `safety_stop`: priority 200 (above joystick 100, calibration 50, navigation 10), timeout 0.2 s. The mux republishes the winner verbatim on `/ackermann_drive`, so frame_id names the source. Re-arm: the lane times out 0.2 s after the last stop; `update()` holds SUCCESS for `event_stop_hold_s` 0.2 s after the last callback stop |
| No TF yet | laser assumed at base_link (0, 0, 0): returns read 0.305 m nearer the bumper, so it trips early, never late (for this forward mount) |
| Staleness | **none** (see 4) |

## 2. The port

`proximity_guard_node` is an independent rclrs node with the same cone, threshold and bumper
reference, the same /tf_static-only laser pose (with the same fallback), and the same arithmetic
in the same order (f64, glibc libm on both sides).

- **Modes.** `guard:=off` (default) launches nothing. `guard:=shadow` publishes decisions on
  `/proximity_guard/decision` (diagnostic_msgs/DiagnosticStatus: stop, tripped, reason,
  min_front_m, scan stamp, latency, compute time, tf_fallback, beam counts) and **no command**.
  `guard:=active` also publishes, for every stopping decision, the same zero-speed command as the
  BT, on the same `safety_stop` lane (priority 200), with its own frame_id
  `base_link/proximity_guard`, so a bag tells guard stops from BT stops.
- Launched from behavior_bringup.launch.py (`proximity_guard`), outside the Nav2 readiness gate.
  The BT's emergency lane is unchanged in every mode. Parameters: `base_front_x_m` from
  vehicle_frames (required) and `front_distance_threshold_m` = the BT's own
  `proximity_front_threshold_m` launch argument.
- /scan is subscribed best effort (sensor data, depth 5), which matches a reliable and a
  best-effort publisher alike.
- **Fail-safe rules** (Rust only; each is its own `reason`; all stop in active mode):

  | reason | when | Python on the same input |
  |---|---|---|
  | `no_scan_yet` | no /scan since start (watchdog, 50 ms) | not tripped |
  | `stale` | last /scan older than `scan_timeout_ms` 200 (8 scan periods on the car) | keeps its last decision forever |
  | `empty_scan` | `ranges` empty | not tripped |
  | `bad_layout` | angle_min / angle_increment / range_min not finite | not tripped |
  | `too_close_unquantified` | a front-sector beam is -inf (REP-117: "too close to measure") | beam skipped |
  | `no_front_beams` | no beam of the scan points into the front sector | not tripped |
  | `invalid_beams` | > `max_invalid_fraction` 0.5 of the front-sector beams NaN or below range_min | beams skipped -> clear |
  | (tf_fallback flag) | laser TF not on /tf_static yet: the Python's (0,0,0) stand-in, flagged in the decision, not a stop | same |

  None of these fires on the car's data: 0 of 123,811 recorded scans (the UST-10LX via urg_node
  reports no NaN, no inf and no sub-range_min value in the front cone in any bag).
- Gates: `cargo fmt --check`, `clippy --all-targets -D warnings` (unsafe forbidden;
  unwrap/expect/panic denied outside tests), `cargo test` (16 tests), the colcon tests of the
  touched packages (`test_proximity_guard_launch.py`, `test_rust_switches.py`).

## 3. Parity

Same input to both: the Python side runs the real `IsProximityTooClose._scan_callback`
(stand-in node; the laser pose put where `StaticFramePoses` caches it), and its front decision
is `latest_lidar_front_min < front_distance_threshold`. The Rust side is
`proximity_guard_replay`, which runs `decision::decide`, the function the node's scan
callback calls. Compared per input: the decision (gate: 0 differences) and `min_front`.

| Group | Cases | Python trips | Rust trips | Decision differences | Rust stop != Python (fail-safe) |
|---|---:|---:|---:|---:|---:|
| Synthetic | 3,184 | 444 | 444 | **0** | 12 |
| Recorded car scans (217 bags, every /scan, laser pose from each bag's /tf_static) | 123,811 | 4,237 | 4,237 | **0** | 0 |
| Recorded scans + an obstacle injected 0.13-0.17 m from the bumper | 80,256 | 20,969 | 20,969 | **0** | 0 |
| **Total** | **207,251** | 25,650 | 25,650 | **0** | 12 |

- Synthetic: open scan; single returns at the threshold -1 cm, -1 mm, exact, +1 mm, +1 cm on
  seven bearings (0 to +-40 deg); single beams in/out of the cone at 7 ranges x 11 bearings;
  **cone edges measured from the bumper** (44, 44.5, 44.9, 45, 45.1, 45.5, 46 deg; d 0.05-0.14 m);
  walls ahead at 0.10-0.30 m; returns behind the bumper / beside the body; NaN, +inf, -inf,
  0.01 (below range_min), 0.0, as one beam ahead / the whole front / the whole scan / next to a
  real trip; 40/50/60 % of the front NaN; empty scan; no TF at 0.3-0.6 m; a reversed scan
  (negative increment); a rear-facing mount (yaw pi); an offset, rotated mount; 3-beam and
  rear-only layouts; 3,000 random scans (obstacle clusters 0.05-0.6 m, random NaN/inf/0,
  random mounts, 10 % without TF).
- `min_front` (the nearest return's distance from the bumper): **bit-identical on all 201,348 inputs
  that have one** (max difference 0.0). The same libm is used on both sides (numpy 1.26.4 on an
  AVX2-only CPU calls glibc for cos/sin/atan2/hypot).
- The 12 synthetic fail-safe stops: `invalid_beams` 7 (all NaN; front all NaN; front or all below
  range_min; front all 0.0), `too_close_unquantified` 3 (-inf ahead / front / everywhere),
  `empty_scan` 1, `no_front_beams` 1 (rear-only layout).

Stale and no-scan-yet are not per-scan inputs; the live test (5) covers them.

Raw: `output/rust/parity_proximity.json`. Re-run (~10 min):
`tools/rust_parity/proximity/parity_proximity.py`.

## 4. Where the Python is NOT fail-safe (reported, not fixed)

1. **No staleness check.** `latest_lidar_front_min` keeps the last scan's value forever. If the
   LiDAR or urg_node dies while the front is clear, the front check reads clear for the rest of
   the run. The emergency lane has no "no /scan for N ms" condition; only the wall distance has a
   max age (`wall_distance_max_age_s`). If it dies while tripped, the car stays stopped
   (safe direction).
2. **No data at startup = clear.** Until the first /scan, nothing trips (stated design: "no data
   yet must not mean tripped").
3. **-inf skipped.** REP-117 -inf ("detection too close to quantify") is treated like no
   return. urg_node never publishes it on the car (0 in 123,811 scans), but a driver that
   follows REP-117 would make "touching" read as "clear".
4. **Invalid beams read clear.** A front sector of NaN, or of values below range_min (a sensor
   fault reporting 0.0, for example), gives no valid return and no trip. Same for an empty scan
   and a scan whose beams all point backwards.
5. **The fallback pose is safe only for this mount.** Without /tf_static the laser is assumed
   at base_link, yaw 0. That trips early for the forward mount 0.10 m behind the bumper, but a
   rear-facing (yaw pi) or bumper-forward mount would make it trip late or watch the wrong way.
6. **QoS mismatch is silent in the BT.** The subscription is RELIABLE; a best-effort /scan
   publisher would never connect, and cpu-cleanup's `qos_events.py` drops the BT's
   incompatible-QoS warning (only the peer logs it). That leads to case 2 for the whole run.
   urg_node and Isaac publish reliable today, so this is not a live problem.

## 5. Decision latency and live behaviour

`tools/rust_parity/proximity/live_guard_test.py`: the real `proximity_guard_node` (active, plus a
shadow instance) and the real `IsProximityTooClose` with its event stop, in a host node, on an
isolated localhost domain. The driver publishes /tf_static and 40 Hz /scan (wall-clock stamps):
every fifth scan trips (0.10 m from the bumper), then 2 s with no scans, then clear, then a
front of NaN. Latency is the stop command's receipt time minus the triggering scan's stamp,
measured in a third process (DDS transport both ways included).

| Phase | Rust guard (active) | Python IsProximityTooClose (event stop) |
|---|---|---|
| A: 237 tripping scans in 1189 (30 s at 40 Hz): stops | 237 (one per tripping scan) | 237 |
| A: scan stamp -> stop received, p50 / p95 / max | **0.63 / 0.80 / 0.97 ms** | 0.97 / 1.18 / 1.68 ms |
| A: decision compute inside the callback, p50 / p95 / max | 71 / 95 / 192 us | (not instrumented) |
| A: process CPU per scan | 0.32 ms (shadow instance 0.32) | 0.56 ms (host node with only this behaviour) |
| A: shadow instance commands | **0** | - |
| B: no /scan for 2 s | first stop 240 ms after the last scan (timeout 200 + watchdog 50), then every <= 50 ms (36 stops, reason `stale`): the mux lane (0.2 s timeout) is held without a gap | **0 stops** (keeps its last decision, clear) |
| C: clear scans again | 0 stops | 0 |
| D: front sector all NaN, 40 scans | 40 stops (`invalid_beams`) | **0** (reads clear) |


The BT's own path adds its tick: a tripping scan is acted on by the emergency lane's Stop at
the next 10 Hz tick (0-100 ms later); the event stop above is what cpu-cleanup added to remove
that wait.

## 6. In Isaac (2026-10-18)

With the full stack in Isaac Sim on linus (`isaac_test.md` section 5), the guard's front decision
matched the BT's own IsProximityTooClose front decision on every live scan: 35,120 + 9,965 scans
in two e2e sessions (all clear), and 856 scans with a box 0.10 m from the bumper, 589 of them
tripped on both sides. In `guard:=active` the guard's stops reached `/ackermann_drive` through the
mux next to the BT's.


## 7. Scan timeout: the guard vs the BT when /scan stops (2026-10-18, follow-up)

**Result: they differ, by design, and the guard is the stricter side. The BT's front check has no
scan timeout at all; the guard stops after `scan_timeout_ms` (200). Not a bug in the guard; on the
BT side it is the fail-open case 1 of section 4, now measured.**

`tools/rust_parity/proximity/timeout_parity.py`, isolated localhost domain:
- **Two guards, both active:** `use_sim_time` false (the car), and `use_sim_time` true with `/clock`
  from the harness.
- **The real IsProximityTooClose, with both of the BT's stop paths:**
  - *event:* the stop from the /scan callback (cpu-cleanup, `enable_proximity_event_stop`);
  - *tick:* a 10 Hz timer that calls `update()` and publishes the emergency lane's Stop when it
    returns SUCCESS. This is the emergency Sequence of `create_root()`.
- **Each case:** 1 s of scans whose last ones are clear or tripping (0.10 m from the bumper), a gap
  without scans, then clear scans. `/clock` either keeps running through the gap (scans lost) or
  freezes (a simulator pause).
- **Gaps:** 150, 250, 500 and 1000 ms, plus 260 ms. That is the pause recorded in run A
  (`impl:=py`): the guard saw 232 ms and published exactly one stop, so the gap was 232-282 ms.
- **Counted:** stops published during the gap per source (first stop after the last scan in
  brackets). The mux holds each stop 0.2 s.

| last scans | /clock in the gap | gap | BT event path | BT tick path | guard, car (steady) | guard, sim (ROS time) |
|---|---|---:|---:|---:|---|---|
| clear | running | 150 ms | 0 | 0 | 0 | 0 |
| clear | running | 250 ms | 0 | 0 | 2 (first 210 ms) | 2 (first 210 ms) |
| clear | running | 500 ms | 0 | 0 | 6 (first 233 ms) | 6 (first 233 ms) |
| clear | running | 1000 ms | 0 | 0 | 17 (first 207 ms) | 17 (first 207 ms) |
| clear | running | 260 ms (recorded) | 0 | 0 | 1 (first 231 ms) | 1 (first 231 ms) |
| clear | frozen | 150 ms | 0 | 0 | 0 | 0 |
| clear | frozen | 250 ms | 0 | 0 | 1 (first 218 ms) | 0 |
| clear | frozen | 500 ms | 0 | 0 | 6 (first 242 ms) | 0 |
| clear | frozen | 1000 ms | 0 | 0 | 16 (first 215 ms) | 0 |
| clear | frozen | 260 ms (recorded) | 0 | 0 | 1 (first 240 ms) | 0 |
| trip | running | 150 ms | 0 | 1 (first 87 ms) | 0 | 0 |
| trip | running | 250 ms | 0 | 2 (first 111 ms) | 1 (first 228 ms) | 1 (first 228 ms) |
| trip | running | 500 ms | 0 | 5 (first 36 ms) | 7 (first 202 ms) | 7 (first 202 ms) |
| trip | running | 1000 ms | 0 | 10 (first 109 ms) | 16 (first 225 ms) | 16 (first 225 ms) |
| trip | running | 260 ms (recorded) | 0 | 2 (first 82 ms) | 1 (first 249 ms) | 1 (first 249 ms) |
| trip | frozen | 150 ms | 0 | 1 (first 96 ms) | 0 | 0 |
| trip | frozen | 250 ms | 0 | 2 (first 120 ms) | 1 (first 237 ms) | 0 |
| trip | frozen | 500 ms | 0 | 5 (first 44 ms) | 7 (first 210 ms) | 0 |
| trip | frozen | 1000 ms | 0 | 9 (first 117 ms) | 16 (first 234 ms) | 0 |
| trip | frozen | 260 ms (recorded) | 0 | 2 (first 91 ms) | 2 (first 207 ms) | 0 |

Reading it:
- **Last scan clear, scans then stop: the BT never stops**, for any gap length; neither path
  publishes anything (the event path has no scan, the tick path keeps the last decision "clear").
  The guard (car) stops ~200-250 ms after the last scan, every 50 ms, as long as the gap lasts.
- **Last scan tripping:** the BT's tick path keeps stopping through the gap on its latched decision
  (from the next tick, 36-120 ms), the event path stops with the scans. The guard stops from
  ~200-250 ms; before that the mux still holds its last per-scan stop (0.2 s), so the lane does not
  open in between. Both sides keep the car stopped.
- **`/clock` frozen (simulator pause):** the car-mode guard stops (steady clock); the sim-mode guard
  does not (ROS time did not advance). The BT does what it does without a pause.
- **Gaps under the timeout (150 ms) stop neither side.**
- After scans resume, at most one late stop per side (one watchdog or tick period).

What this means on the car: if the LiDAR or urg_node dies while the front reads clear, the BT
keeps driving on the last clear decision. `guard:=active` stops the car 200-250 ms after the last
scan. The difference stays as it is: the BT is not changed (task rule), and the guard is meant to
be stricter.

## 8. Which clock judges a stale scan (follow-up)

**Before:** the guard judged staleness on the steady clock (`std::time::Instant`) on the car and in
sim alike. A simulator pause therefore looked like a dead LiDAR. In run A (`impl:=py`,
`guard:=active`) this produced one stop.

**Now** (`src/rust/f1tenth_proximity_guard/src/watchdog.rs`):
- **Clock choice:** read from the node clock's type at start-up. rclrs switches it to ROS time
  exactly when `use_sim_time` is true. The start log says `(steady clock)` or `(ros clock)`, and
  every decision carries `stale_clock`.
- **Car, `use_sim_time` false: unchanged.** `no_scan_yet` and `stale` are on the steady clock.
  Verified again with `live_guard_test.py`: first stale stop 233 ms after the last scan
  (240 ms before the change), one stop per tripping scan, latency p50 0.65 ms.
- **Sim, `use_sim_time` true:**
  - `stale` follows ROS time. A gap over the timeout in wall time but not in ROS time is a
    `sim_pause`: logged once per pause, counted in the decision's `sim_pauses`, no stop.
  - `no_scan_yet` stays on the steady clock, so a simulator that never sends a scan still stops
    an active guard.
  - A `/clock` that runs backwards (a reset) counts as "just received".
- 3 new unit tests (19 in total); `timeout_parity.py` above confirms both modes.
- `scan_timeout_ms` is still 200.

Re-run of scenario A with `guard:=active`, twice (`tools/rust_parity/isaac/scenario.sh`; raw data
in `isaac_smoke/r1_a_active_py`, `r2_a_active_rust`):

| run | clock | sim pauses logged | guard stops | mission |
|---|---|---:|---:|---|
| R1 `impl:=py` | ros | 0 | **0** | COMPLETE 20.2 s |
| R2 `impl:=rust` | ros | 2 (208 ms wall / 17 ms ROS; 230 ms / 33 ms) | **0** (both would have been stops before) | COMPLETE 20.2 s |
| R3/R4 B2 shadow | ros | 0 / 1 (228 ms / 33 ms) | shadow | BT stop in front of the box, as before |

**What the pauses are (open).** All four pauses seen so far happened while the car was parked: three at
2-3 s after the stack starts, and one 46 s after start in scenario A, before the mission
drove. In each, the guard received neither /scan nor
/clock for 208-232 ms, so its own ROS time advanced only 17-33 ms. A separate recorder never saw
such a gap in any recorded window: 5 windows, about 6 min, including resets and one stack start-up.
The numbers are in `isaac_scan_gaps.json` and the sim-health note of `isaac_test.md`. None of the guard's
pauses fell inside a recorded window, and the one recorded start-up had no pause.

So it is not proven whether Isaac slowed down for those 200 ms (the stack's start-up loads all
8 cores) or only the guard process received nothing. In the second case simulated time did run on
and the car could have moved.

Consequence:
- **In sim**, the ROS-time rule ignores any gap during which /clock also stops reaching the guard.
  That is what was asked for. All observed cases had the car parked.
- **On the car** nothing changes (steady clock).
