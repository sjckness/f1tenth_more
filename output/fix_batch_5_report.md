# Fix batch 5: discovery isolation (H1) — lease fix and topic-liveness watchdog

- **Where:** Thor, branch `jazzy`. Pushed with the merge of origin/jazzy (sim work).
- **Commits:** prefix `fix/<area>:`, one logical change each (list at the end).
- **Data:** `output/fix_batch_5/`. Full-stack mission bags (`*.mcap`) are not
  committed.
- **Decisions this batch implements:** your answers to fix batch 4's open
  decisions (lease 5 s / 1 s; 60 proof runs with the watchdog alert-only;
  20 runs under CPU stress; per-component `restart`/`alert`; a give-up after 3
  restarts that never got healthy; restarting only the failing launch file; a
  `/clock` pause gate in sim only; 2c and the lifecycle re-send left out).
  2b (a heartbeat) moved in, for `mpc_corr` only.

## Verdict: **GO-WITH-NOTES**

- **A. Lease fix: done and proven.**
  - Every participant loads one profile: 5 s lease, 1 s announcement.
  - A killed node now leaves the Discovery Server after **4.4–4.7 s**,
    down from 19.3–19.5 s.
  - **Isolation rate:** 1 of 60 bringups, against 8 of 30 before.
    - p = 0.0005 (one-sided Fisher exact).
    - 95% upper bound on the remaining rate: 7.7%, against 27% before.
  - **The main failure, slam_toolbox never activating:** 0 of 60, against
    7 of 30 before (p = 0.0003).
  - **Under CPU saturation** (`stress`, 94.7% CPU busy): 0 of 20 runs
    isolated, and no live node dropped.
  - **Discovery cost:** discovery traffic on loopback doubles at idle (UDP 19.4 →
    39.1 datagrams/s, 13.6 → 26.7 kB/s); the Fast DDS threads of all 29
    participants together go from 0.034 to 0.036 cores, the Discovery
    Server from 0.0012 to 0.0021 cores. No receive-buffer drops (A5).
- **B. Topic-liveness watchdog: done, with all four live tests passing.**
  - **No false triggers in the planned tests** (one found afterwards in
    the discovery-load data, note 5):
    - none in 10 minutes of normal running;
    - none in 80 bringups (alert mode), 20 of them under CPU saturation;
    - none during a 34 s `/clock` pause in sim.
  - **A frozen node:** with `swept_clearance_node` SIGSTOPped, the watchdog
    detected it after 0.94 s, logged the ERROR at 3.9 s, restarted it, and
    the component was OK again at 34.4 s.
  - **A component that keeps failing:** restarted 3 times, then FAILED, and
    it stays FAILED.
- **Notes (details below):**
  1. **The lease does not remove every isolation.** The remaining case
     (run 29) is a different kind of failure: a whole participant
     (`semantic_layer_node`) never appeared in the graph. Its cause is not
     established. That node also has no output the watchdog could watch.
  2. **The live tests overturned three of fix batch 4's assumptions about
     what a topic proves.** All three were fixed before the long runs (B5):
     - `/slam/map` by age;
     - the EKF's header stamp as proof of input;
     - `/drive` as proof that `mpc_corr` gets odometry.
  3. **Some components are only judged on "it runs":** wall_distance,
     behavior and system_observer have no output that depends on their
     input (B2 table, OUTPUT-ONLY).
  4. **The lease under real load still needs the Orin.** Thor under
     `stress` is the stand-in used here.
  5. **One false trigger, found afterwards in the discovery-load data,
     now fixed** (B6, `f9e0fea`): a `once` check failed immediately when
     its input appeared after the grace period, and restarted a healthy
     slam once in 5 enforce-mode runs. `once` checks now wait `settle_sec`
     (15 s) after their input appears: 0 watchdog events in 10 late-input
     bringups (5 live, 5 sim at RTF 0.74) and 5 normal ones.

---

## A. Lease fix

### A1. What changed (`7b301b2`, `dac4bef`)

- **One profile file:** `src/f1tenth_bringup/config/fastdds_profile.xml`.
  - Default participant profile: `leaseDuration` 5 s, `leaseAnnouncement`
    1 s. Nothing else.
  - Discovery protocol, server address and transports still come from the
    environment.
- **The launch files set it.**
  - Both `supervisor_bringup.launch.py` and `stack_bringup.launch.py` set
    `FASTRTPS_DEFAULT_PROFILES_FILE`, right after `ROS_DISCOVERY_SERVER`.
  - Every participant of the launch tree inherits it:
    - the Discovery Server (`ensure_discovery_server.py`);
    - the supervisor;
    - every component;
    - the mission logger and foxglove super clients.
  - Launch argument `fastdds_profile` (default from `stack_params.yaml`)
    overrides it.
  - It overrides whatever the shell had, so the stack cannot run without it.
- **The shell sets it too.** `scripts/env/jazzy.sh` exports the same file
  for everything started by hand:
  - `ros2` and `ros2cli`, rosbag, `stackctl.py`, the test harnesses;
  - the simulator on the sim host.

  Paths are now relative to the script, so the repo can live anywhere. It
  works from bash and from zsh:
  - **bash:** sourced from inside and outside the repo.
  - **zsh 5.9:** unpacked from the Ubuntu package, since zsh is not
    installed on Thor. It sources the workspace, resolves the profile, and
    `ros2cli` works.
- **The car's Docker image** starts the stack through the same launch files.

### A2. Does it take effect, and where must it be set? (`scripts/fix_batch_5/lease_probe.py`)

**Method:** a Discovery Server client is SIGKILLed (it cannot dispose); a
super client times how long it stays in the graph. Own server port and
domain, 3 trials each.

| Who loads the profile | Node removed after SIGKILL |
|---|---|
| nobody | 19.3–19.5 s |
| Fast DDS defaults written out (`profiles/default_lease.xml`, the "before" condition below) | 19.4–19.5 s |
| everybody | **4.4–4.7 s** |
| the dying node only | **4.4–4.5 s** |
| server and observer, not the dying node | 19.4 s |

**The lease belongs to the participant that dies.** Every process that can
die must carry the profile, including those on the sim host. The server's
own setting changes nothing.

### A3. The trade-off, with numbers

**How fast a dead node is noticed:**
- **5 s lease:** within 5 s of its last announcement; measured 4.4–4.7 s
  after SIGKILL.
- **20 s default:** 19.3–19.5 s.
- **A respawn by the supervisor** comes back about 2 s after the old
  process died:
  - with 20 s, the dead one is always still registered;
  - with 5 s, it overlaps for only about 3 s.

**When a live node is dropped by mistake:** only if nothing arrives from it
for 5 s. With announcements every 1 s, that takes one of:
- about 4 announcements in a row not sent: its Fast DDS event thread
  (`dds.ev.*`) starved for about 4 s or more;
- 4 datagrams in a row lost on loopback.

| | default 20 s / 3 s | 5 s / 1 s |
|---|---|---|
| Announcements missed before a live node is dropped | ~6 | ~4 |
| Event-thread stall that drops a live node | ≥ ~17 s | ≥ ~4 s |
| Time a dead node stays registered | ≤ 20 s (19.4 measured) | ≤ 5 s (4.5 measured) |

- **Event thread under CPU saturation (`stress --cpu 12` on 14 cores):**
  measured as its outcome, not its stall time.
  - 20 bringups ran at 94.7% CPU busy (load 11–19).
  - `isolation_check.py` polls the graph every second. In 20 runs it
    flagged no node as vanished (absent for 5 or more polls).
  - No isolation, no watchdog alert.
  - So no live participant was dropped. A longer starvation (the Orin at
    100% for minutes) is still to be measured there.
- **UDP receive-buffer drops** (`RcvbufErrors`) during the runs:
  0 in all 25 discovery-load runs (before, after, with the watchdog), idle
  and fed. Not recorded in the proof and stress batches.

### A4. Proof: 60 full-stack bringups with the fix

**Same conditions as fix batch 4's baseline:**
- `phase5_bringup.sh isolation`;
- the Discovery Server reused across runs (started fresh, with the profile,
  by run 1);
- a 2 s gap between a shutdown and the next bringup;
- `isolation_check.py` per run.

**Changed:** the shipped profile (the launch default). The watchdog runs in
alert mode, so it reports but never restarts and cannot hide an isolation.

**Run count:** 60 runs, 52 minutes. Against 8/30, 0 of 60 would give
p = 2×10⁻⁵ and an upper bound under 5%. The count was chosen for that
upper bound, not just for significance.

| Batch | Lease | Runs affected | slam never ACTIVE | other |
|---|---|---|---|---|
| FB4 baseline | 20 s | **8 / 30** | 7 | 1 (localization) |
| FB4 `lease` (experiment profile) | 5 s | 0 / 30 | 0 | 0 |
| **FB5 proof (shipped profile)** | 5 s | **1 / 60** | **0** | 1 (run 29, below) |
| FB5 stress (`stress --cpu 12`, 94.7% CPU busy) | 5 s | **0 / 20** | 0 | 0 (no node vanished) |

- **Statistics** (one-sided Fisher exact; exact Clopper–Pearson bound):
  - proof against baseline: p = **0.0005**;
  - rate 1.7%, 95% upper bound **7.7%**, against 26.7% before;
  - FB4 lease + FB5 proof (1/90) against baseline: p = 5×10⁻⁵, 95% upper
    bound 5.2%;
  - slam lifecycle hang 0/60 against 7/30: p = 0.0003, upper bound 4.9%.
- **CPU busy:** mean 21.7% (FB4: 20.7%).
- **Watchdog alerts in the 60 runs:** **none**, and none in the 20 stress
  runs either. That makes 80 bringups plus the 10-minute run without a false
  trigger. Under saturation, both EKFs stayed above the 30 Hz rate check.
- **Stress batch against baseline:** 0/20, p = 0.011, 95% upper bound 14%.

**Run 29, the remaining case.**
- **What the run showed:**
  - The `semantic_layer_node` process was running and logged `started`.
  - It was never in the graph, from bringup to shutdown.
  - Every other node was present.
  - No node of the previous run appears in run 29's graph timeline.
- **Why it differs from FB4's failures.** FB4's failures were single
  endpoint pairs of participants that *were* in the graph. Here a whole
  participant is missing.
- **Timing:** run 28's processes died about 4 s before run 29's nodes
  started. With a 5 s lease and a 2 s gap, the old and new windows can still
  just touch.
- **Unconfirmed:** whether this is the same mechanism at its edge. One event
  in 60 is not enough to tell.
- **Coverage:** `semantic_layer_node` publishes only when there are
  detections, so the watchdog has nothing to watch for it (B2). This is a
  new backlog item.

### A5. Discovery cost before and after (`discovery_load.py`, `phase5_bringup.sh discload`)

**Method:** per run, bringup until settled, then 60 s idle (unfed), then
30 s fed by the bag. Measured over every participant of the domain:
- CPU time per Fast DDS thread class: `dds.ev` (event thread:
  announcements, lease checks), `dds.udp` (UDP receive, mostly discovery),
  `dds.shm` (same-host data);
- the Discovery Server's CPU;
- the host's UDP and loopback counters.

**The two conditions:**
- **Before:** `fastdds_profile:=profiles/default_lease.xml`, which is Fast
  DDS's defaults written out. Same code, same file mechanism; only the
  lease differs.
- **After:** the shipped profile.

The watchdog was disabled in both. A third condition measures its own cost.

**Results** (median over runs; cores = CPU seconds per second, summed over
every participant of the domain; 29 participants in every run):

| | before (20 s / 3 s) | after (5 s / 1 s) | after + watchdog |
|---|---|---|---|
| Runs | 10 | 10 | 4 of 5 (see below) |
| **Idle:** UDP datagrams/s (all discovery) | 19.4 | **39.1** | 39.1 |
| Idle: loopback kB/s | 13.6 | **26.7** | 26.7 |
| Idle: `dds.ev` cores | 0.0091 | 0.0119 | 0.0129 |
| Idle: all Fast DDS threads, cores | 0.0343 | 0.0364 | 0.0440 |
| Idle: Discovery Server, cores | 0.0012 | 0.0021 | 0.0023 |
| Idle: supervisor, cores | 0.0012 | 0.0011 | **0.112** |
| **Fed:** UDP datagrams/s | 67.2 | 87.9 | 88.8 |
| Fed: loopback kB/s | 31.3 | 45.8 | 46.3 |
| Fed: all Fast DDS threads, cores | 0.109 | 0.115 | 0.129 |
| Fed: Discovery Server, cores | 0.0032 | 0.0043 | 0.0040 |
| Fed: supervisor, cores | 0.0013 | 0.0015 | **0.440** |
| Fed: node code (`app`), cores | 2.08 | 2.17 | 2.60 |
| `RcvbufErrors` (any run, any phase) | 0 | 0 | 0 |
| Bringup: Fast DDS thread CPU-s (mean ± sd) | 4.9 ± 0.8 | 4.0 ± 1.5 | 3.9 ± 0.4 |

- **The lease's cost is traffic, not CPU.** Announcements every 1 s instead
  of 3 s double the idle discovery datagrams; at 27 kB/s on loopback that is
  negligible. Every DDS CPU figure stays within a few thousandths of a core
  for the whole stack. On the two-machine sim setup this traffic crosses the
  LAN, still in the tens of kB/s.
- **Outlier, `after` run 7:** `dds.ev` 0.15 cores (others 0.011–0.013) and
  `dds.shm` 0.13, idle and fed alike, with no other sign in the run. It is
  why the medians are used; the means are in `discload/summary.json`.
- **The watchdog's cost is the supervisor's own thread:** 0.44 cores while
  fed. The node-code line rises by the same amount (2.17 → 2.60); nothing
  else changes. On the Orin's slower cores this is to be re-measured.
- **`after_watchdog` run 5 is excluded** from this column: the watchdog
  (enforce) restarted slam 6 s after the feed started, which changes the
  participants (27) and the traffic (402 datagrams/s during the restart).
  That restart was **a false trigger**, see B6.
- `noprofile_pre/`: three runs of the pre-implementation code without any
  profile, reference only (README there).

---

## B. Supervisor topic-liveness watchdog

### B1. What it does (`96a5b12`, corrected in `725f256`; `88fbd42`, `0f151aa`)

**Where it runs:** inside `component_supervisor_node`, on its executor, every
0.5 s. The logic is pure and unit-tested: `topic_watchdog.py`.

**Configuration:** `components.yaml`, top-level `health:` and
`health_topic_types:`, next to `components:`.
- Every component is in `health:`, or in `unwatched:` with a reason.
- A test enforces that, so a component cannot be added without stating its
  health.
- Not watched:
  - `calibrate_hardware` and `startup_sequence` (on demand);
  - `intelligence` (llama-server, not a ROS node);
  - `dev_tools` (foxglove; the operator sees it).

**A check:**
- `topic`;
- `max_age_sec`, or `once: true` (at least one message since the launch
  file (re)started);
- optionally:
  - `when_fresh: {input: max_age}`: judged only while those inputs are
    fresh (`.inf` = received at least once);
  - `expect: {field: [values]}`: content of the last message;
  - `min_rate_hz` over `rate_window_sec`;
- `launch_file`: who gets restarted.

**Per component:** `action` (`restart` | `alert`), `grace_sec` (20; slam
30), `fail_for_sec` (3), and `enabled_if` (stack_params values, e.g.
`use_lidar: true`).

**Verdicts, every tick:**
- **skip:** the launch file is not running or is in its grace period, or an
  input is stale.
- **fail:**
  - older than its max age (never received counts as infinitely old);
  - no message since start (a `once` check);
  - wrong content;
  - below the minimum rate.
- **ok:** otherwise.

**Inputs that come back:** an output's age counts only from the moment its
inputs came back. Content and rate are judged a full max age or window
later. The silence during an upstream outage is not the node's fault.

**On failure:** after `fail_for_sec` of continuous failure:
1. A loud **ERROR** naming the component, topic, age and input age. For
   example:

   `[health] 'swept_clearance' LIVENESS FAILURE: /perception/swept_clearance/lidar: no message for 4.0 s (max 0.5 s) while its input is fresh (/scan 0.0 s) -- restarting swept_clearance.launch.py (watchdog restart 1/3)`
2. **`restart`:** only the failing launch file is restarted. It goes through
   the supervisor's existing stop path (`_stop_process()`: SIGINT, SIGKILL
   after `restart_timeout_sec`) and is charged to the **same per-process
   restart budget as a crash** (3 per 60 s). If the budget says no, the
   component is FAILED.
3. **`alert`:** reported, never restarted. This is the action for
   `hardware` and `perception`: restarting either would restart the VESC
   driver or the e-stop's only `/scan`.
4. **No restart loops:** after 3 watchdog restarts with no 10 s healthy
   period in between, the component is **FAILED**. No more restarts; it
   stays reported until a manual START or RESTART resets it.
   - The existing budget cannot do this alone. One watchdog cycle (grace +
     `fail_for` + restart) takes more than 20 s, so 3 restarts never fall
     inside its 60 s window.
   - Measured: restarts 24 s apart.

**Status:**
- **`/supervisor/health`:** a DiagnosticArray, one status per component.
  States: OK, STARTING, FAILING, RESTARTING, ALERT, FAILED, UPSTREAM_STALE,
  NOT_RUNNING, PAUSED.
  - Values: action, failures, watchdog restarts, consecutive restarts,
    gave up.
  - Published at 1 Hz and on every change of state; transient-local.
  - Also published on `/diagnostics`, so the diagnostics tools show it.
- **`stackctl.py status`** prints one line per component and exits 1 on any
  ERROR:

```
component liveness (/supervisor/health):
  OK     behavior             OK
  WARN   diagnostics          UPSTREAM_STALE: input /sensors/core stale (never received)
  ...
```

  (That WARN came from an early build. A component whose judged checks all
  pass is now OK, with the unjudged inputs listed as "not judged".)
- **`health_watchdog`** parameter / launch argument: `enforce` (default),
  `alert` or `disabled`.
  - It is not `on`/`off`: YAML turned `'on'` into a boolean in the launch
    parameter file, and the supervisor refused to start. This is now in
    CLAUDE.md.

### B2. What each check proves

Each check is marked:
- **INPUT-PROVING:** the message exists only because the node got its input
  recently.
- **AT START:** the node publishes only after its input arrived once.
- **OUTPUT-ONLY:** published on a timer whatever its inputs do.

| Component | Check | Gate (`when_fresh`) | Proves | Why |
|---|---|---|---|---|
| hardware (alert) | `/sensors/core` 0.5 s | – | OUTPUT-ONLY | the driver's own output |
| hardware (alert) | `/odom` 0.5 s | `/sensors/core` | INPUT-PROVING | one `/odom` per `/sensors/core` |
| perception (alert) | `/scan` 0.5 s | – | OUTPUT-ONLY | the driver's own output; camera not watched (a full copy per frame) |
| localization | `/odometry/filtered` 0.5 s, **≥ 30 Hz over 2 s** | `/odom` | INPUT-PROVING (rate) | measured: 48 Hz fed, ~9 Hz with `/odom` cut (predicting) |
| localization | `/ekf_global/odometry/filtered` 0.5 s, ≥ 30 Hz | `/odom` | INPUT-PROVING (rate) | 48 Hz fed, ~16 Hz cut |
| localization | `/joint_states` 1 s | – | AT START | joint_state_publisher waits for `/robot_description` (FB4 run 4) |
| slam | `/slam/map` **once** per start | `/scan` | AT START | only after ACTIVE and a processed scan, which the FB4 lifecycle loss never reaches |
| slam (costmap) | `/costmap/front_clearance` 0.5 s | `/slam/map` (ever), `/ekf_global/...` | AT START | withheld until map and pose arrive, then republished from cache at 20 Hz |
| lidar_front_wall | `/perception/lidar_front_wall` 0.5 s | `/scan` | INPUT-PROVING | one fit per scan |
| obstacle_clearance | `/obstacle_clearance` 0.5 s | `/scan` | INPUT-PROVING | one per scan |
| swept_clearance | `/perception/swept_clearance/lidar` 0.5 s | `/scan` | INPUT-PROVING | one per scan |
| wall_distance | `/perception/d_wall/segment` 0.5 s | – | **OUTPUT-ONLY** | 10 Hz timer; `valid=false` cannot tell "no scan" from "no wall" |
| control | `/ackermann_drive` 0.5 s | `/drive` | INPUT-PROVING | the mux republishes its active lane's messages, nothing without one |
| navigation | `/mpc/input_status` 1.5 s | – | OUTPUT-ONLY | `mpc_corr`'s control loop runs |
| navigation | `/mpc/input_status` level OK | `/odometry/filtered` | **INPUT-PROVING** | new in this batch (2b), below |
| behavior | `/behavior/tree_status` 0.5 s | – | **OUTPUT-ONLY** | one per BT tick, whatever the inputs |
| diagnostics | `/diagnostics/system_status` 5 s | – | OUTPUT-ONLY | reads `/proc`/jtop, no ROS input: this is its whole liveness |
| diagnostics | `/diagnostics/battery_status` 1.5 s | – | OUTPUT-ONLY | 2 Hz tick |
| diagnostics | `/diagnostics/battery_status` `has_data` true | `/sensors/core` | INPUT-PROVING | true only if samples arrived since the last tick |

**`mpc_corr` (decision 2b, `0f151aa`).** I checked every existing output
first.
- **No existing output shows odometry freshness:**
  - `/drive` is published every tick, with or without odometry: a hold when
    it is fresh, a hard zero when it is not.
  - `/mpc/solver_status` and `/mpc/status` appear only after a solve.
  - The only signal is the `ODOMSEL` info line on `/rosout`, which is not an
    interface.
- **So `mpc_corr` now publishes `/mpc/input_status`:**
  - a DiagnosticStatus at 2 Hz, from the control loop;
  - level OK while the odometry source `_update_active_odom()` picked is
    fresher than `odom_stale_timeout_sec` (0.5 s), ERROR when there is none;
  - values: source, the hardware and sim odometry ages, the timeout.
- **The watchdog's rule:** level ERROR for 1.5 s or more while
  `/odometry/filtered` is fresh at the supervisor means `mpc_corr` is not
  getting its odometry. Its arrival also proves the loop runs.

**Gaps that remain:**
- wall_distance, behavior and system_observer are judged on "it runs" only.
- Two nodes have no watchable output at all:
  - `semantic_layer_node` (run 29) publishes only when there are detections;
  - `costmap_renderer`'s output is a visualization image.
- The localization rate check catches a starved EKF only while `/odom`
  itself reaches the supervisor.

### B3. Receipt in wall time, QoS, and a paused simulation

**Wall time:**
- Every message is stamped with the monotonic clock on receipt. ROS time
  and header stamps are never used.
- The supervisor itself is not on `use_sim_time`.

**QoS:** every watched topic is read with one QoS: BEST_EFFORT, VOLATILE,
depth 1. It is right for every topic here.
- **Compatible with every publisher:** reliable or best-effort, volatile or
  transient-local.
- **Changes no publisher's behaviour:** a reliable writer keeps no history
  for a best-effort reader and is never held back by it (no ACKs, no
  repairs).
- **Volatile matters for latched topics** (`/slam/map`, `/robot_description`):
  the old sample is not replayed on connect, where it would look like a
  fresh receipt.
- **Cost:** topics are subscribed raw (no deserialization) unless a check
  reads the message (`/mpc/input_status`, `/diagnostics/battery_status`).
  The supervisor's CPU with the watchdog: **0.11 cores idle, 0.44 cores fed**
  (one thread), against 0.001 without it (A5). That is subscriptions to 18
  topics, among them `/scan`, `/odom` and both EKFs.

**A paused simulation (`sim:=true` only):**
- **When it counts as paused:** `/clock` has not advanced for 0.5 s (a
  simulator publishes it at 100 Hz or more). This also covers the time
  before the simulator starts.
- **While paused:** nothing is judged and failure timers are cleared. Every
  status reads PAUSED.
- **After resume:** a 5 s grace for every component, because the
  sim-time nodes' timers and inputs need a moment to flow again.
- **Rate checks in sim:** scaled by the real-time factor measured from
  `/clock`; nodes count Hz in sim time.

**The gate cannot switch the watchdog off on the car:**
- Without `sim:=true`, there is no `/clock` subscription and no gate.
- Test 2 below ran with `sim:=false`: `ros2 topic info /clock` says
  "Unknown topic", and the SIGSTOPped node was caught in 0.94 s.
- The supervisor warns if a component's `fail_for_sec` is not above the
  gate's 0.5 s. Otherwise a silence that begins with a pause could act
  before the gate engages.

**The supervisor blocked:** a tick more than 2 s late (for example, blocked
in a restart's wait) clears the failure timers instead of judging ages it
could not keep current.

### B4. Tests

**Unit tests:**
- `test_topic_watchdog.py` (37; 46 with B6's fix): config validation; verdicts; gating and the
  input-returned rule; `once`; rate and its sim scaling; grace; fail_for;
  alert-once; give-up and reset; pause and resume; no gate without the
  flag; stall; restarting only the failing launch file.
- `test_component_health_config.py` (7):
  - every component is watched or explained;
  - every type imports;
  - hardware and perception are alert-only;
  - navigation is never judged by `/drive`, slam never by `/slam/pose`;
  - `enabled_if` names real parameters.
- `mpc_controller/test/test_input_status.py` (6).
- `f1tenth_bringup`: 84 pass. Its flake8, pep257 and copyright checks fail
  as they did before (55 flake8 findings in the supervisor at HEAD~7 and
  now). The new files are clean.

**Live, in the Phase 5 harness** (`scripts/fix_batch_5/watchdog_live.sh`,
final code `725f256`, `output/fix_batch_5/watchdog_live/{normal,failing,sim}`):

| # | Test | Result |
|---|---|---|
| 1 | Normal bringup, sim:=false, 10 min (605.8 s) | **0 triggers.** The only non-OK states: inputs not yet fed and slam's grace at startup. |
| 2 | SIGSTOP `swept_clearance_node` | **FAILING at +0.94 s** (max age 0.5 s, 0.5 s ticks); ERROR + restart started at +3.9 s; restart done +14.0 s; **OK at +34.4 s**. No other component affected. |
| 3 | Check patched to a topic nothing publishes (a component that keeps failing) | Restarts at +3.9, +27.8, +51.8 s; **FAILED at +75.3 s**, and still FAILED at shutdown. No other component affected. |
| 4 | sim:=true, `/clock` paused 33.8 s (`/rosbag2_player/pause`) | **0 triggers.** Every component PAUSED within 0.5 s, then STARTING (5 s), then OK. |
| – | Feed stopped (after test 2) | **0 triggers.** Components with a stale input go to "not judged" or UPSTREAM_STALE. |

**About test 2's restart time:**
- The restart took 10 s: a stopped process cannot act on SIGINT, so the
  existing stop path waited `restart_timeout_sec` and then used SIGKILL.
- During those 10 s the single-threaded supervisor is blocked. The stall
  rule kept that from becoming a failure elsewhere.
- The restarted node is pid 865625 (from the launch log).

### B5. What the live tests corrected

Fixed before any long run. The superseded runs are kept as evidence
(`watchdog_live/*_v1`, `*_v2`).

1. **`/slam/map` by age (15 s) restarted a healthy slam.**
   - **Why:** before each publish, slam_toolbox rebuilds the whole grid from
     every scan so far, under the mapper lock (`updateMap()`, 2.8.5). The
     5 s interval therefore stretches as the session grows.
   - **Measured in the 10-minute test:** 19 maps; median gap 5 s in the
     first third, 12.5 s in the last; **one gap of 415 s**.
   - **Now:** `once` per start.
   - A smoke run also restarted slam 3 s after a 40 s `/scan` outage ended.
     That led to the input-returned rule.
2. **"A new header stamp proves EKF input" is false.**
   - **Measured** (`ekf_stamps/`, super-client probe): with `/odom` cut, both
     EKFs keep publishing with a new stamp on every message, at about 9 and
     16 Hz.
   - **Now:** a rate check (≥ 30 Hz), which the rate drop shows reliably.
3. **`mpc_corr` showed ERROR for 0.5 s at feed start.** The supervisor sees
   `/odometry/filtered` before `mpc_corr` does. Content checks now wait
   their max age after the input returns.
4. **`/supervisor/health` flooded.** It was republished every tick while
   failing, because the message quotes ages. It now publishes on a change
   of state.

### B6. A false trigger the live tests missed: `once` after a late input

Found while writing up A5, after the batches ran; **fixed in `f9e0fea`**
(your decision of 2026-10-05; the fix and its tests are at the end of this
section).

- **What happened** (`discload/after_watchdog/run_05`, watchdog enforce):
  - slam_toolbox started, configured and was ACTIVE 3.6 s after bringup.
  - The harness then ran 60 s idle without `/scan`, so slam's 30 s grace
    ran out with `/scan` stale; the check was skipped, correctly.
  - The feed started; **6 s later** the watchdog logged `'slam' LIVENESS
    FAILURE: /slam/map: no message never` and restarted slam.launch.py.
  - slam needs a scan plus its map interval (≈ 5 s) for the first map.
- **Why:** the input-returned rule (B5 item 3) only applies to age checks.
  A `once` check (`topic_watchdog.py`, `check_verdict`) fails as soon as
  its `when_fresh` input is fresh and nothing has arrived since the launch
  file started, with no time for the node to answer the input.
- **When it can happen:** whenever `/slam/map`'s input first appears more
  than the grace (30 s) after slam starts: the lidar coming up late on the
  car, or a sim host that starts publishing `/scan` late (after the
  `/clock` gate's `resume_grace_sec`). It costs one slam restart (the map
  restarts), then the check passes. 4 of 5 runs got their map just in time.
- **Not seen before** because the proof, stress and live-test runs feed the
  bag inside slam's grace.

**The fix (`f9e0fea`).**
- **`settle_sec`** (`once` checks only; default `ONCE_SETTLE_SEC` = 15 s;
  slam sets it explicitly in `components.yaml`): no verdict until the
  check's `when_fresh` inputs have been fresh that long. Meanwhile the
  component reads STARTING, `/slam/map: settling, inputs fresh for N of
  15 s`.
- **Counted from the inputs only.** The node's own startup stays the grace
  period's job: with `/scan` already flowing, a restarted slam is judged
  when its grace ends, as before.
- **An input outage before the first map starts the settle time again.**
- **What it costs:** a slam that never maps after a late input is restarted
  at input + 15 s + `fail_for_sec` (3 s) instead of input + 3 s.

**Unit tests** (`test_topic_watchdog.py`, 9 new; 72 pass with the health
config, preflight and `/mpc/input_status` tests):

| Case | Expected | |
|---|---|---|
| Input inside the grace, map 6 s later | never triggers | pass |
| Input after the grace (B6: `/scan` at 60 s, map at 67 s) | no restart; STARTING "settling", then OK | pass |
| Same with `settle_sec: 0` (the old behaviour) | restart at 63 s: B6 reproduced | pass |
| Input never | slam never restarted; UPSTREAM_STALE (WARN, naming `/scan`); the lidar's own `/scan` check goes ALERT | pass |
| Output never after settle (input at 60 s) | restart at 60 + 15 + 3 = 78 s | pass |
| Output never, input inside the grace | restart when the grace ends (+ `fail_for`); settle does not extend the grace | pass |
| Input outage before the first map | settle starts again | pass |
| sim: `/clock` and `/scan` start at 60 s | no restart (resume grace, then settle) | pass |
| `settle_sec` on an age check / below 0 | config error at startup | pass |

**"Input never" does not restart slam, by design.** A lidar that never
publishes is not slam's fault. Restarting slam would not help and would
hide the real cause. The watchdog reports it in two places:
- slam: UPSTREAM_STALE (WARN);
- the component that owns `/scan`: its own check (`perception`, alert
  only, since restarting it restarts urg_node, the e-stop's sensor).

**Live** (`phase5_bringup.sh latefeed`, new: the feed starts N s after the
launch; `first_rx.py` records the first `/scan` and `/slam/map` receipts;
data `output/fix_batch_5/once_settle/`):

| Batch | Code | Runs | `/scan` starts (after launch) | slam restarts | Other watchdog events | First map after first scan |
|---|---|---|---|---|---|---|
| `control_live` | before the fix | 4 | +37 … +43 s (after the grace) | 0 | 0 | 0.1–2.6 s |
| `control_sim` (sim:=true, `/clock` with the feed, RTF 0.74) | before the fix | 3 | +27 … +42 s | 0 | 0 | 0.1–2.2 s |
| **`late_live`** | **fixed** | **5** | +22 … +42 s | **0** | **0** | 0.9–4.8 s |
| **`late_sim`** (RTF 0.74) | **fixed** | **5** | +24 … +42 s | **0** | **0** | 0.2–4.7 s |
| **`normal`** (feed once settled) | **fixed** | **5** | +25 … +28 s | **0** | **0** | 1.1–4.3 s |

- **The control did not reproduce the restart in 7 runs, but it shows the
  race.** In 3 of the 4 live control runs slam went FAILING ("no message
  never"); each time the map arrived before `fail_for_sec` (3 s) ran out.
  A restart needs a gap above 3 s. That happened in B6, and in 6 of the 15
  runs with the fix (3.2–4.8 s). The unit test reproduces it every time.
- **With the fix the settle time was used:** in `late_live` runs 3–5,
  where `/scan` came after the grace, slam read STARTING "settling", then
  OK. Run 3: settling at +34.6 s, map 4.4 s later, OK at +39.1 s. The old
  code would have restarted slam about 3 s after the first scan.
- **The sim runs did not need it.** In sim mode, the 5 s resume grace after
  `/clock` starts already covered gaps up to 4.7 s. The settle time is the
  margin beyond that.
- **No non-OK state elsewhere** in the fixed runs, apart from `hardware`
  and `perception` NOT_RUNNING in sim (their drivers are skipped there).

**Why 15 s.**
- **Measured:** over all 22 runs, the first map came a median 2.1 s and at
  most 4.8 s after the first scan.
- **RTF did not stretch it:** sim at RTF 0.74 gave at most 4.7 s. That fits
  slam's map timer running on wall time.
- **If it did run on sim time,** the worst case would be 5 s / 0.74 = 6.8 s
  plus the rebuild.
- **So:** 15 s is about 3× the measured maximum and 2× that theoretical
  worst case. It holds up a real failure by 15 s only after a late input.

---

## Phase S: what the sim host (linus / TPad) must set

**Both machines run the jazzy branch of this repo.**

**On the sim host, in the shell that starts the simulator, the bridge and
any ROS tool:**

```bash
source <repo>/scripts/env/jazzy.sh            # bash or zsh: sets FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISCOVERY_SERVER=<Thor LAN IP>:11811   # jazzy.sh defaults to 127.0.0.1
export ROS_DOMAIN_ID=<same as Thor>           # 0 in production
ros2 daemon stop                              # the daemon keeps its old environment
echo $FASTRTPS_DEFAULT_PROFILES_FILE          # must print <repo>/src/f1tenth_bringup/config/fastdds_profile.xml
```

**If the sim host does not use `jazzy.sh`** (for example a Humble install):
- Export `FASTRTPS_DEFAULT_PROFILES_FILE=<repo>/src/f1tenth_bringup/config/fastdds_profile.xml`
  yourself, before starting anything.
- The same XML and the same variable work on Fast DDS 2.6 (Humble) and 2.14
  (Jazzy).

**On Thor:**

```bash
ros2 launch f1tenth_bringup supervisor_bringup.launch.py sim:=true discovery_server_address:=<Thor LAN IP>
```

**Why the sim host needs it:**
- The lease belongs to each participant (A2).
- A simulator, bridge or tool that dies on the sim host without the profile
  stays registered in Thor's Discovery Server for 20 s.
- That is the same window H1 came from.

**Check:**
1. `kill -9` a sim-side node.
2. On Thor, `ros2cli node list` must stop listing it within about 5 s.

**Already true:**
- the watchdog needs `/clock` from the simulator (which it publishes);
- the rest of M10 is unchanged.

---

## Backlog changes (`output/backlog.md`)

- **H1 closed.** Fixed by the lease profile and the watchdog. The remaining
  1-in-60 participant miss is a new item.
- **New, MEDIUM:**
  - the run 29 participant miss (cause open);
  - nodes without input-proving outputs (wall_distance, behavior,
    semantic_layer): heartbeats as for `mpc_corr`, if wanted.
  - the `once` false trigger (B6): opened as M19, closed by `f9e0fea`.
- **New, LOW:** slam_toolbox's map interval grows with the session (415 s
  gap in a looped-bag run). `costmap_boundary` works from a map that can be
  minutes old.
- **M10 (two-machine checklist):** the sim host's profile.
- **Orin items:** the lease under the Orin's real load.

## Commits

```
7b301b2 fix/discovery: 5 s participant lease profile for every participant of the stack, set by both bringup launch files — a killed node now leaves the Discovery Server in 4.5 s instead of 19.4 s
dac4bef fix/env: jazzy.sh exports the Fast DDS lease profile for hand-started participants, and works from zsh as well as bash
0f151aa fix/control: mpc_corr publishes /mpc/input_status, an odometry-freshness heartbeat — /drive is published with or without odometry, so nothing showed an isolated mpc_corr
96a5b12 fix/bringup: supervisor topic-liveness watchdog — each component's output topics, received by the supervisor in wall time, restart an isolated component within its restart budget
88fbd42 fix/tools: stackctl.py status prints the supervisor's liveness watchdog, one line per watched component, and fails on any ERROR
725f256 fix/bringup: liveness checks corrected by the live tests — slam judged by one map per start, the EKFs by their rate, not by age or header.stamp
009813c fix/discovery: fix batch 5 measurement harness — lease probe, discovery CPU/traffic per thread class, watchdog live tests, isolation and discovery-load batches
39f7856 fix/batch5: report, backlog, data — H1 closed by the lease profile and the liveness watchdog; GO-WITH-NOTES
```

Pushed together with the merge of origin/jazzy (`39b2b71`). After your
decisions of 2026-10-05:

```
f9e0fea fix/bringup: liveness watchdog `once` checks wait settle_sec after their input appears — a /scan that first came after slam's grace restarted a healthy slam
```

followed by the report, backlog and live-data update.

## Decisions (Andreas, 2026-10-05)

1. **Decision 2c: stop on `/supervisor/health` ERROR:** to be done; HIGH,
   required before Phase 8 (ground driving). Backlog H7 (was M18).
2. **Heartbeats for the OUTPUT-ONLY components:** to be done, wall_distance
   first; MEDIUM. Backlog M17.
3. **A shorter stop for watchdog restarts:** no; kept as is.
4. **The lease on the Orin under its real load:** on the Orin session list
   in the backlog.
5. **The `once` false trigger (B6):** fixed now, before Phase S (`f9e0fea`,
   B6). Backlog M19 closed.
6. **The watchdog's CPU (A5):** on the Orin session list in the backlog.
7. **`f1tenth_more` build failure on Thor** (merge check): Thor's
   `~/.colcon/defaults.yaml` skips it for build and test, next to
   `f1tenth_sim`; noted in CLAUDE.md, "Thor". No repo change to the
   metapackage.
8. **`test_grace_then_target_lost`** (fails on the merge base too): backlog
   M20, MEDIUM.
