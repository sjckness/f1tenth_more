# Why the simulated clock misbehaved in the two-machine sim — and the fix

Setup: Gazebo Harmonic on linus ↔ Jazzy stack on the Thor (`sim:=true`), direct
Ethernet (10.42.0.1 ↔ 10.42.0.2), `ROS_DOMAIN_ID=42`, Fast DDS Discovery Server
on the Thor. Found 2026-10-05, fixed 2026-10-06, branch `jazzy`.

## Symptom

The supervisor's liveness watchdog repeatedly saw several unrelated nodes stop
publishing at the same instant for ~4 s (`/joint_states`, `/behavior/tree_status`,
`/perception/d_wall/segment`, `/mpc/input_status`, briefly the EKF outputs),
while the sim's own data (`/odom`, `/scan`) stayed fresh. `/odometry/filtered`
arrived in bursts with gaps up to 5 s. In enforce mode healthy components were
restarted until FAILED.

Ruled out: Gazebo slow (`/stats` RTF steady 1.00), Thor CPU (idle, top process
13 % of a core), two `/clock` publishers (`pgrep`: one bridge, one server),
time going backwards (probe: `backwards=0`).

## Measurement

`scripts/clock_probe.py` on the Thor, `/clock` at 1000 Hz from Gazebo:

    msgs=38   (8/s)     max_gap=320 ms    sim_advance=0.04 s  in 5 s wall
    msgs=5846 (1169/s)  max_gap=644 ms    sim_advance=13.34 s in 5 s wall
    msgs=0    (0/s)     -                 sim_advance=0.00 s  in 5 s wall
    msgs=2614 (523/s)   max_gap=15926 ms  sim_advance=2.85 s  in 5 s wall
    msgs=6542 (1308/s)  max_gap=7 ms      sim_advance=33.35 s in 5 s wall

Gazebo produced a steady 1000/s; the Thor received it in waves: crawling
phases, holes up to 16 s, then catch-up bursts.

## Mechanism

With `use_sim_time:=true` a node's "now" is the last `/clock` it received, and
every timer fires only when sim time reaches its tick. No `/clock` → every
sim-time timer on the Thor stops at once (unrelated nodes silent together);
data topics from the sim keep arriving ("input fresh, output missing"); the
backlog then arrives and the timers fire in a burst. The supervisor's
"sim clock paused" gate missed the crawling phases (a few msgs/s still came).

## Root cause: `/clock` fan-out across the network

Every node on sim time subscribes to `/clock` (~35–40 on the Thor; confirm
with `ros2 topic info /clock`). Same-host delivery is cheap (shared memory);
across hosts, with the Discovery Server (unicast), the writer sends one UDP
packet per remote reader per message: 1000 Hz × ~40 ≈ 40,000 packets/s for
one tiny type, on top of `/scan`, `/odom`, IMU, camera. The writer falls
behind, stalls, then flushes. The link was fine (ping 0.57 ms avg); the
per-reader packet rate was not. (Flagged in advance in `sim_port_report.md`.)

Proof: 4 ms physics step (`/clock` at 250 Hz) gave exactly 1250 msgs / 5 s,
max_gap ≤ 35 ms, sim_advance = 5.00 s, no liveness failures, closed-loop
LLM-planned mission completed.

## Fix (option B, implemented)

Physics stays at 1 ms; only the published clock is decimated.

- `f1tenth_sim/config/ros_gz_bridge.yaml`: Gazebo `/clock` → ROS
  `/sim/clock_raw` (no reader on the Thor, so it never crosses the cable).
- `f1tenth_sim/clock_throttle.py` (`clock_throttle` executable, node
  `sim_clock_throttle`): `/sim/clock_raw` → `/clock` at `rate_hz`.
  - Decimates on **sim** time snapped to a grid (forward when
    `floor(t/period)` grows): with 1 ms steps and 200 Hz the forwarded stamps
    are exact multiples of 5 ms, so 50 Hz timers fire exactly on their tick;
    average rate is `rate_hz` of sim time at any RTF.
  - Forwards Gazebo's own values only: a paused sim forwards nothing new (the
    supervisor's pause gate still works); time going back (world reset) is
    forwarded immediately and re-anchors.
  - Serialized in/out (reads only the 8-byte time from the CDR, republishes
    the same bytes); input BEST_EFFORT depth 1, output RELIABLE depth 1 (stays
    compatible with reliable readers like probes/recorders; never queues a
    stale backlog).
  - Forces `use_sim_time:=false` on itself (on sim time it would subscribe to
    its own output). Logs in/out counts every `stats_period_sec` (30 s).
- `sim_bringup.launch.py`: new arg `clock_rate` (default `200.0`;
  `0` = forward every step, the old behaviour, for A/B).
- Tests: `f1tenth_sim/test/test_clock_throttle.py` (decimation, grid, pause,
  reset, passthrough, CDR endianness, bridge/launch wiring; no ROS needed).
- Probe: `scripts/clock_probe.py` (BEST_EFFORT depth 1, like TimeSource).

Rate choice: the clock rate is the time resolution of every sim-time timer.
Fastest timers are 50 Hz (20 ms) → 200 Hz (5 ms) keeps jitter well under a
period; 100 Hz is the practical minimum. 200 Hz ≈ 8,000 packets/s with ~40
readers — below the 250 Hz rate already proven clean.

Other options considered: A (coarser physics, the 2026-10-05 workaround —
loses fidelity), C (clock relay on the Thor — same topic name on both hosts in
one domain is awkward), D (fewer subscribers — not realistic).

## Acceptance test

1. linus: rebuild `f1tenth_sim`, launch the sim (default `clock_rate:=200`).
   `ros2 topic hz /sim/clock_raw` ≈ 1000, `ros2 topic hz /clock` ≈ 200.
2. Thor: `python3 scripts/clock_probe.py` for ≥ 2 min → msgs/s ≈ 200,
   max_gap < ~20–35 ms, sim_advance ≈ wall, backwards=0.
3. Thor: stack with `health_watchdog:=alert`, ≥ 5 min, no liveness failures.
4. Check `ros2 topic info /sim/clock_raw -v` shows only linus-side readers.
5. Note `sim_clock_throttle`'s CPU on linus (`top`); if a rclpy callback at
   1 kHz is too costly, port it to a C++ component (same logic).

## Related findings (same session)

- Watchdog not sim-aware enough: rate checks scale with RTF, age checks don't
  (fix batch 6).
- Supervisor crash "Logger severity cannot be changed between calls" in the
  health tick (fix batch 6, high priority — can also happen on the car).
- Start the Gazebo GUI from a normal terminal: the snap VS Code terminal loads
  a wrong libpthread and crashes it.
- `worlds/husarion_office_2x.sdf` has two `<physics>` blocks (line 4,
  `type="ignored"`, and line ~7494, `type="ode"` with
  `real_time_update_rate 1000`); a step-size change has to touch the one
  Gazebo actually uses — worth deduplicating.
