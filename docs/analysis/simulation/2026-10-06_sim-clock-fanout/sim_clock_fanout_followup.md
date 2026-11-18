# Why sim time on the Thor is still unstable after the /clock throttle — follow-up

Date: 2026-10-06. Run from **linus** (sim host), read-only, no code edited, nothing
running was restarted. Companion to `output/sim_clock_fanout.md` (the throttle fix)
and `f1tenth_sim/clock_throttle.py`.

## TL;DR — stopped at the first failing check (on linus)

**The clock is already irregular at the source, on linus, before the throttle and
before the cable.** Not because of the throttle (it works: exactly one `/clock`
publisher, `rate_hz=200`, raw clock stays local) and not because of a stale
publisher. The cause is an **unstable real-time factor**: RTF over 6350 `/stats`
samples is **min 0.05, max 1.40, mean 0.978** — the sim keeps up *on average* but
advances sim time in violent jerks. Gazebo's own server is at **154 % CPU**
rendering the ZED (rgb 27 Hz, depth 27 Hz, pointcloud 16 Hz) in the 2× office
world, and on this particular instance the **GUI is also running** (+54 % CPU).
Because sim time stalls (RTF→0.05) then catches up (RTF→1.4), `/clock` arrives in
wall time with gaps up to ~130 ms (raw) / ~270 ms (throttled), even though the
*average* rates are on target. The throttle faithfully reproduces that jerky
sim-time advance; it cannot smooth a wall-time gap the simulator creates.

This is upstream of the cross-cable fan-out the throttle was built to fix. Until
the sim advances time smoothly on linus, the Thor will see an uneven `/clock`
regardless of fan-out.

**Caveat on scope:** the running sim is a **non-default** configuration —
`ROS_DOMAIN_ID=42` (stack default is 0) and **GUI on** (default is `gui:=false`,
headless). Both add load / deviate from how the stack normally runs. See §6.

**Not done:** the Thor A/B/C/D `clock_probe` matrix (the core of the request)
could not be run — it needs the stack + clean sim relaunches + NIC-drop counters
on the Thor, and this session is on linus only. See §7 for exactly what to run and
§8 for the option-C design requested.

---

## Environment discovered (and a measurement trap)

The running sim is on **`ROS_DOMAIN_ID=42`** behind the Discovery Server at the
**Thor, `10.42.0.2:11811`** (`RMW=rmw_fastrtps_cpp`, the repo fastdds profile).
From `/proc/<pid>/environ`:

| process | ROS_DOMAIN_ID | ROS_DISCOVERY_SERVER |
|---|---|---|
| fresh gz server / clock_throttle (2415770/2415691) | 42 | 10.42.0.2:11811 |
| stale orphans (1174153/1175117/1175121) | (unset → 0) | (unset, simple disc.) |
| my login shell | (unset → 0) | (unset, simple disc.) |

A plain `ros2` here sees only the **stale domain-0 orphans**, not the sim — so the
first `ros2 topic info /clock` wrongly showed `Publisher count: 0`. All numbers
below were taken as a **super client** on domain 42:
`export F1TENTH_DISCOVERY_SERVER=10.42.0.2:11811 ROS_DOMAIN_ID=42;
source scripts/env/jazzy.sh; ros2cli … / ROS_SUPER_CLIENT=TRUE ros2 …`.
DS host reachable: `ping 10.42.0.2` 0.23 ms avg, 0 % loss.

## 1. Stale processes (flagged, not the cause)

`ps -eo pid,etime,args` at 14:24, now-launch age 00:37, these are ~21 h old:

```
1174153  21:25:58  …/f1tenth_sim/lib/f1tenth_sim/drive_bridge   (f1tenth_sim_drive_bridge)
1175121  21:25:19  …/f1tenth_sim/lib/f1tenth_sim/drive_bridge   (f1tenth_sim_drive_bridge)
1175117  21:25:19  robot_state_publisher                        (sim_robot_state_publisher)
```

Fresh launch (00:37) is complete: `gz sim server`+`gui`, `sim_ros_gz_bridge`,
`sim_ros_gz_camera_bridge`, `sim_clock_throttle`, `drive_bridge`, RSP.

- **No stale `parameter_bridge`** → nothing old is publishing `/clock` directly
  past the throttle.
- The three orphans are on **domain 0**, so they are *not* subscribers to the
  sim's domain-42 `/clock` and do **not** add to the fan-out. They are leftover
  cruft (two duplicate `f1tenth_sim_drive_bridge`, one duplicate RSP) worth
  killing, but irrelevant to the clock instability.

## 2. `/clock` has exactly one publisher — PASS

`ros2cli topic info /clock -v` (super client):
- **Publisher count: 1**, RELIABLE  → the throttle (`sim_clock_throttle`).
- **Subscription count: 44**, all BEST_EFFORT (the TimeSource readers; ~40 on the
  Thor + a few linus sim nodes). Node names came back `_UNKNOWN_` under the
  super client so the exact linus/Thor split could not be attributed per-endpoint;
  count is 44.

`ros2cli topic info /sim/clock_raw -v`:
- Publisher count 1 = `sim_ros_gz_bridge` (RELIABLE); Subscription count 1 =
  `sim_clock_throttle` (BEST_EFFORT). Exactly as designed — raw clock has one
  local reader, never crosses the cable.

## 3. Rates — averages on target, but delivery is gappy

`gz topic -e` counts and `ros2 topic hz` (super client), steady state:

| signal | rate | note |
|---|---|---|
| gz `/clock` (gz transport) | **~737 Hz** | 2212 msgs/3 s; = 1000 Hz physics × RTF |
| ROS `/sim/clock_raw` (`ros2 topic hz`) | **~670–700 Hz** | bridge forwards raw clock fine |
| ROS `/clock` (`ros2 topic hz`) | **~136 Hz** | = 200 Hz sim-time × RTF ✓ |
| `/scan` | ~30 Hz | main bridge healthy |
| `/odom` | ~42 Hz | |

(An initial `/clock` ≈ 10 Hz / `/sim/clock_raw` = 0 msgs reading was a startup
transient right after launch; it cleared within a minute.)

But with the **real BEST_EFFORT depth-1 QoS** that every node's TimeSource uses
(`scripts/clock_probe.py`, on linus, period 5 s):

```
/sim/clock_raw : msgs=1901 (380/s) max_gap= 95 ms  sim_advance=2.50 s /5 s  backwards=0
                 msgs=3046 (609/s) max_gap= 65 ms  sim_advance=3.85 s /5 s  backwards=0
                 msgs=2782 (556/s) max_gap=132 ms  sim_advance=4.26 s /5 s  backwards=0

/clock         : msgs=185  (37/s)  max_gap= 65 ms  sim_advance=2.62 s /5 s  backwards=0
                 msgs=433  (87/s)  max_gap=269 ms  sim_advance=4.33 s /5 s  backwards=0
                 msgs=274  (55/s)  max_gap=180 ms  sim_advance=2.95 s /5 s  backwards=0
                 msgs=297  (59/s)  max_gap=189 ms  sim_advance=3.29 s /5 s  backwards=0
                 msgs=229  (46/s)  max_gap=230 ms  sim_advance=2.28 s /5 s  backwards=0
```

The decisive columns are `max_gap` and `sim_advance`:
- **Raw clock, local to linus, is already gappy: max_gap 65–132 ms** — and the
  probe sustained 380–609 msgs/s there, so its executor is *not* the bottleneck;
  the gaps are real stalls in the source signal.
- **`sim_advance` swings 2.28–4.33 s per 5 s wall** → instantaneous RTF 0.46–0.87,
  i.e. the sim advances time jerkily.
- `/clock` inherits both: max_gap up to **269 ms**. `backwards=0` throughout.
  (The lower msgs/s on `/clock` is the depth-1 reader coalescing during the
  bursts; the gap is the point, not the count.)

## 4. Throttle health

- `rate_hz` parameter = **200.0** (correct).
- CPU (`top`): idle in a lull (0–1 %) but **45 % of a core** while the ~700 Hz raw
  clock flows (PID 2415691, cumulative TIME+ 9:22). A Python rclpy callback at
  ~700 Hz is not free — matches the acceptance note (step 5) to port it to a C++
  component if the cost matters. Not the instability cause, but a standing cost.

## 5. RTF and camera load — the mechanism

`gz topic -e -t /stats`, 6350 `real_time_factor` samples:

```
min = 0.05    max = 1.40    mean = 0.978
```

Sample-to-sample it lurches, e.g. `… 0.85, 0.99, 1.06, 0.30, 0.73, 1.04, 0.85,
0.61, 0.80, 1.12, 1.16 …`. Mean ~1.0 means the host *can* keep up overall, but the
per-step pacing is unstable: stalls to RTF 0.05 (near-frozen) then catches up at
1.4.

Load at that moment (`top`):

```
2415770 ruby  154.5% CPU  1.6g  gz sim server   (physics + sensors + render)
2415773 ruby   54.5% CPU  1.4g  gz sim gui       (GUI on — see §6)
2415691 clock_throttle 45.5% CPU               (Python @ ~700 Hz)
```

ZED render on the gz side: `/sim/zed2/rgb/image` ~27 Hz, `/sim/zed2/depth/image`
~27 Hz, `/sim/zed2/depth/image/points` ~16 Hz. The camera is rendering near its
30 Hz target and is the dominant server load; the pointcloud adds more.

**Chain:** camera render + 2× office world (+ GUI) overloads the gz server →
sim-time advance is jerky (RTF 0.05–1.40) → `/sim/clock_raw` and `/clock` arrive
in wall time with ~100–270 ms gaps → every sim-time timer on every reader (linus
*and* Thor) freezes for that gap and then bursts. The throttle changed the *rate*
(40 k → ~5 k pkt/s across the cable: 136 Hz × ~40 readers) but cannot change the
fact that the underlying sim-time stream is bursty.

## 6. Two things that make THIS instance non-representative

1. **GUI is on.** The running server was started with `--gui-config … -r` and a
   separate `gz sim gui` process (no `-s`/`--headless`), i.e. `gui:=true`. The
   default bringup is `gui:=false` (headless). The GUI's 54 % CPU competes with
   the render/physics and worsens RTF. A fair measurement must be headless.
2. **`ROS_DOMAIN_ID=42`**, not the stack default 0. Harmless to the clock shape,
   but means this was a hand-isolated test session, not the normal stack wiring.

Also relevant (from the fan-out doc): `worlds/husarion_office_2x.sdf` has **two
`<physics>` blocks** (an `ignored` one and the real `ode` one with
`real_time_update_rate 1000`). Any step-rate experiment must edit the block
Gazebo actually uses.

## 7. What still must be measured on the Thor (could not run from linus)

The request's A/B/C/D matrix is the right test but needs: the stack up on the
Thor, a clean sim relaunch between runs (kill the stale orphans first), and
`ip -s link` RX/TX drops on the cable NIC on **both** hosts. None is doable from
this linus session. Run on the Thor, 2 min each, `scripts/clock_probe.py`:

| run | args | isolates |
|---|---|---|
| a | `camera:=true  clock_rate:=200` (default) | current behaviour |
| b | `camera:=false clock_rate:=200` | camera render load |
| c | `camera:=false clock_rate:=100` | fan-out at half rate |
| d | `camera:=true  clock_rate:=100` | rate vs camera |

Record per run: msgs/s, max_gap, sim_advance vs wall, backwards, plus `/stats` RTF
and NIC drops before/after on both hosts. **Launch headless** (`gui:=false`) so
the GUI does not skew RTF (§6).

Prediction from the linus data: because the *source* clock is already gappy with
camera on (§3, §5) and RTF is unstable even on a 154 % server, **(a) will be bad
and (b) materially better** — i.e. camera render on linus is a primary driver, not
only the cross-cable fan-out. (c)/(d) separate the residual fan-out from render.

## 8. Proposals (design only — not implemented, per instructions)

Ordered by what the linus data most supports:

**P1 — Fix the source first: reduce camera render load on linus.** Options, cheap→
invasive: run the sim **headless** by default for two-machine use; **lower the ZED
rate** (30→15 Hz) and/or **drop the depth pointcloud** (`/sim/zed2/depth/image/
points`, ~16 Hz, pure render cost if no consumer needs it); bridge **compressed /
lower-res** images. Re-check `/stats` RTF stability after each — the goal is RTF
pinned near a *steady* value (even 0.8 steady beats 0.978 mean with 0.05–1.4
swings). This is the branch the numbers point at; confirm with run (b).

**P2 — Option C (clock relay on the Thor), as requested.** Keep `/clock` from
crossing the cable at all:
- Throttle on linus publishes **`/sim/clock_net`** (not `/clock`): RELIABLE,
  depth 1, the single cross-cable clock stream — **one** remote reader.
- A tiny **relay node on the Thor** subscribes `/sim/clock_net` and republishes
  **`/clock`** locally; every Thor node gets `/clock` over shared memory. Fan-out
  (×~40) is now intra-Thor SHM, not UDP over the cable. Cross-cable clock traffic
  drops from 136 Hz × ~40 readers to 136 Hz × 1.
- **Remaps so `/clock` exists only on the Thor:**
  - linus `clock_throttle`: `output_topic:=/sim/clock_net` (param already exists).
  - linus sim nodes that are on sim time (`sim_robot_state_publisher`,
    `f1tenth_sim_drive_bridge`, the gz_ros2_control / spawned controllers):
    remap `/clock:=/sim/clock_raw` so they read the local raw clock directly and
    never need the networked `/clock`. (They are on linus; raw clock is local and
    free.) Equivalently, run them with `use_sim_time` against `/sim/clock_raw`.
  - Thor relay: subscribe `/sim/clock_net`, publish `/clock` (RELIABLE depth 1).
  - Thor stack: unchanged — it already reads `/clock`.
- Open question to resolve before building: one domain, same topic name `/clock`
  on both hosts is what we are *avoiding*; option C does this cleanly only because
  linus nodes are remapped off `/clock` entirely. Verify no linus node still needs
  a networked `/clock`.
- **Note:** P2 only helps if (a) is bad but (b) is clean (pure fan-out problem).
  If (a) and (b) are *both* bad (the linus source is the issue, as §5 suggests),
  **do P1 first** — a relay cannot smooth a clock that is already bursty at the
  source.

**P3 — Port `clock_throttle` to a C++ component** (same logic) to shed the ~45 %
core it costs at ~700 Hz raw input (acceptance step 5). Secondary.

**P4 — Housekeeping:** kill the three stale domain-0 orphans (§1); de-duplicate
the two `<physics>` blocks in `husarion_office_2x.sdf` (§6).

## Numbers at a glance

| check | result | verdict |
|---|---|---|
| stale processes | 3 orphans ~21 h old, **domain 0** (off the sim's graph) | flag, not cause |
| `/clock` publishers | **1** (throttle, RELIABLE); 44 subs | PASS |
| `/sim/clock_raw` endpoints | 1 pub (bridge) / 1 sub (throttle) | PASS |
| gz `/clock` | ~737 Hz | healthy |
| `/sim/clock_raw` rate | ~670–700 Hz; **probe max_gap 65–132 ms** | gappy at source |
| `/clock` rate | ~136 Hz (=200×RTF); **probe max_gap up to 269 ms** | gappy |
| throttle `rate_hz` / CPU | 200.0 / up to **45 %** of a core | OK / costly |
| RTF (`/stats`, n=6350) | **min 0.05, max 1.40, mean 0.978** | **unstable — root cause** |
| ZED gz streams | rgb 27 / depth 27 / points 16 Hz; server 154 % CPU | render load |
| backwards | 0 everywhere | OK |

First failing check: **the sim-time stream is already irregular on linus (RTF
0.05–1.40), from camera render load — upstream of the throttle and the cable.**
