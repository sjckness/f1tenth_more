# Fix batch 4: steering calibration preflight (H6) and discovery isolation (H1)

- **Where:** Thor, branch `jazzy`. Nothing has been pushed.
- **Commits:** prefix `fix/<area>:`.
- **Data:** `output/fix_batch_4/`. Full-stack mission bags (`*.mcap`) are
  not committed.

## Verdict: **GO-WITH-NOTES**

- **A (H6) is done.** The steering calibration's e-stop check now waits up
  to 5 s for the ROS graph.
  - With the behaviour tree running: **10 of 10** starts passed.
  - Without it: **3 of 3** refused.
  - It can never command anything before the check passes.
- **B (H1) is root-caused, not fixed.** This part was experiments only.
  - **Most likely cause:** participants from the previous session that
    left without a clean exit are still registered in the reused Discovery
    Server. That window is the default 20 s lease. A new stack started
    inside it intermittently fails to match some endpoints.
  - **Measured over 150 full-stack bringups:**
    - production default (server reused, 2 s gap): **8 of 30** runs
      affected;
    - fresh server per run: **0 of 30**;
    - 30 s gap between runs: **0 of 30**;
    - 5 s participant lease: **0 of 30**;
    - UDP only: still **3 of 30**.
  - **Proposed fix:** a 5 s participant lease profile (Decision 1).
  - **Watchdog design:** proposed below for your approval (Decision 2).

---

## A. Steering calibration e-stop preflight (`b31b76c`)

**Change** (`steering_offset_calibration_node.py`):
- `_check_estop_path()` now gets its two graph counts from a new
  `_wait_for_estop_graph()`:
  - the subscribers on the drive lane (`/calibration_drive`);
  - the publishers on `/safety_stop`.
- It polls both counts while spinning, until both are present or
  `estop_graph_wait_sec` has passed.
  - `estop_graph_wait_sec` is a new parameter, default 5.0, also exposed as
    a launch argument.
- It logs how long it waited, for example: `e-stop graph check after
  0.13 s: 1 subscriber(s) on /calibration_drive, 1 publisher(s) on
  /safety_stop (both present).`
- The decision rules after the wait are unchanged, so a timeout refuses
  exactly as the single read did.
- The flake8 count of the three touched files is unchanged.

**Unit tests** (`test_steering_offset_calibration_node.py`):
- They use a fake clock and a fake `spin_once`, so no real waiting.

| Case | Result |
|---|---|
| graph already complete | pass, no spin |
| graph completes after 2 s | pass after 2.0 s |
| graph never completes | refuse at the 5.0 s default |
| limit set to 1.5 s | refuse at 1.5 s |
| lane present, no `/safety_stop` | refuse |

- 3 of the 5 fail on the old code.
- Suite: 81 passed.

**Live test** (`output/fix_batch_4/estop_live/`):
- **Setup:** full stack, Phase 5 harness, plain clients, launched through
  `steering_offset_calibration.launch.py`.
- **How nothing could move:** `amplitude_rad:=-0.1` forces a stage-1
  refusal, so the nudge is unreachable. Every e-stop result is still
  evaluated and logged.

| | e-stop check | graph wait | outcome |
|---|---|---|---|
| BT running, 10 starts | **10 of 10 pass** | 0.00 s (8), 0.13 s, 0.16 s | "preflight refused -- nothing was commanded" (amplitude) ×10 |
| `behavior` shut down via `stackctl.py stop behavior`, 3 starts | **3 of 3 refuse**: "NO publisher on /safety_stop" | 5.00, 5.01, 5.00 s (limit) | refused, nothing commanded ×3 |

- In this session the graph was usually complete by the time of the check.
  The preflight window ran 1.0–3.4 s, against 2.15 s in fix batch 3.
- The wait covers the cases where it is not. The unit tests cover the slow
  case deterministically.

**Proof that nothing is commanded before the check passes.**
`_publish_drive()` is the only method that publishes on the drive topic
(`drive_pub.publish`). Its callers are:
- **The nudge** (`_nudge_and_confirm_pose`). It is called only on the last
  line of `run_preflight()`, after
  `checks = (…, self._check_estop_path(), …); if not all(checks): return False`.
- **`start_driving()`** (mode B) and **`run_static_sweep()`** (mode A).
  `main()` calls them only in the `else` / `elif` branches after a preflight
  returned True.
- **`publish_stop()`**, which sends explicit zeros (speed 0, steering 0).
  `main()` sends it on every exit path, including a refusal. It is a stop,
  not motion.

---

## B. Discovery isolation (H1): investigation

### B1. Evidence so far, re-read

| Event | Server | What the logs show |
|---|---|---|
| Phase 5 cycle 5: `swept_clearance_node` | reused (cycles 2–10) | Process alive. Never in the graph, never got `/tf_static` or `/scan`. No Fast DDS or server error. |
| Fix batch 3, `hold_before`: `mpc_corr`, `costmap_boundary_node` | fresh | In the graph at 22.8 s, gone by about 40 s (consistent with the 20 s lease). `mpc_corr` never got odometry; `costmap_boundary_node` never got map or pose. Server output: start banner only. |
| Phase 5 cycle 3 (found only now, by the new detector) | reused | `/slam/pose` 0 Hz: slam_toolbox never activated ("Abandoning wait for the '/slam_toolbox/change_state' service response"). |

- No run logged any Fast DDS error. That includes "Matching unexisting
  participant", across every run in Phase 5 and fix batches 2–4.
- **Fast DDS discovery settings in use:** the library defaults
  (`RTPSParticipantAttributes.h`). There is no XML profile anywhere in the
  repo.

| Setting | Value |
|---|---|
| `leaseDuration` | 20 s |
| `leaseDuration_announcementperiod` | 3 s |
| `discoveryServer_client_syncperiod` | 450 ms |

### B2. Detection (reusable for Phase S): `scripts/jazzy_parity/isolation_check.py`

- **`watch`:** a super client polls the node list every second from
  bringup to shutdown and writes a timeline.
- **`check`:** after the bag has played for 12 s, three independent
  signals per node:
  1. **Graph:** the node never appeared, or it appeared and was then absent
     for 5 or more consecutive polls ("vanished", reported separately).
  2. **Output:** a table of input-driven topics, one or more per node (for
     example `/odometry/filtered`, `/slam/pose`, `/costmap/front_clearance`,
     `/perception/swept_clearance/lidar`). Each must deliver a message within
     8 s.
  3. **Starvation:** the node's own log still reports a missing input 12 s
     or more after the feed started (`ODOM non disponibile`, `no message
     ever received`, `lidar never received`).
- **Validated on the old runs:** it flags `mpc_corr` and
  `costmap_boundary_node` in fix batch 3 `hold_before` and
  `swept_clearance_node` in Phase 5 cycle 5. It flags nothing in the normal
  runs.
- **Harness:** `phase5_bringup.sh isolation N` runs N bringups with it.
  `isolation_summary.py` classifies each affected run: does slam_toolbox
  reach ACTIVE?

### B3. Experiments: 30 full-stack bringups per condition

**Why 30.** The rate seen before was about 1 in 10 runs.
- At that rate, P(no event in 30 runs) = 0.9³⁰ = 4%.
- So a batch with zero events is evidence (95%) that its condition pushes
  the rate below 1 in 10.
- 30 is also enough for Fisher's exact test to separate 8/30 from 0/30.

**Every run:**
1. Bringup.
2. Settle (node set unchanged for 15 s).
3. Re-stamped bag.
4. Isolation check.
5. Shutdown.

Each run takes about 55 s, about 80 s with the 30 s gap.

| Batch | Server | Gap | Transport | Lease | Runs affected | slam_toolbox never ACTIVE | other | CPU busy (mean) |
|---|---|---|---|---|---|---|---|---|
| **baseline** (production) | reused | 2 s | default (SHM+UDP) | 20 s | **8 / 30** | 7 | 1 (run 4: localization) | 20.7% |
| **fresh** | fresh per run | 2 s | default | 20 s | **0 / 30** | 0 | 0 | 20.6% |
| **udp** | reused | 2 s | `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (0 SHM files confirmed) | 20 s | **3 / 30** | 3 | 0 | 20.8% |
| **gap** | reused | **30 s** | default | 20 s | **0 / 30** | 0 | 0 | 20.6% |
| **lease** | reused | 2 s | default | **5 s** (`profiles/short_lease.xml`) | **0 / 30** | 0 | 0 | 20.7% |

**Statistics** (Fisher exact, one-sided):
- baseline vs fresh: p = 0.002;
- baseline vs the three zero batches combined (0/90): p = 7×10⁻⁶;
- baseline vs udp: p = 0.09.

**What fails.** In 10 of the 11 affected runs, slam_toolbox never became
ACTIVE.
- launch_ros's lifecycle manager sends `/slam_toolbox/change_state` and
  logs "Abandoning wait for the '/slam_toolbox/change_state' service
  response" at shutdown. Either no configure was logged, or "Configuring"
  appeared without "Activating": the `transition_event` that triggers the
  activate was never seen.
- Everything downstream of `/slam/*` then starves: `slam_pose_relay`,
  `costmap_boundary`, the MPC's map inputs.
- In baseline run 4, the localization component's subscriptions never
  matched their publishers:
  - the EKFs got no `/odom`;
  - joint_state_publisher never got `/robot_description`;
  - `mpc_corr` and slam starved downstream.

  Same signature as fix batch 3's `mpc_corr`: its `/drive` publisher
  matched, but its `/odometry/filtered` subscription never did.
- **Common thread:** individual endpoint pairs, service or topic, that never
  match. The participants themselves are in the graph.

### B4. The hypotheses

| | Hypothesis | Result |
|---|---|---|
| a | **Reused vs fresh Discovery Server** | **Supported, refined.** Reused 8/30 vs fresh 0/30. The server's age is not the cause: a reused server with a 30 s gap is clean (0/30). What matters is a new session starting **within the lease of the previous session's participants.** |
| b | **Lease/liveliness expiry under load** | **Rejected as the trigger.** CPU busy is about 20% in every batch, including the affected runs (17–20%). Nothing logged a lease expiry. The lease matters the other way round: a *long* lease keeps dead participants registered. A 5 s lease (0/30) removes the effect. |
| c | **SHM transport** | **Not the cause.** It still happens with UDP only (3/30; no `/dev/shm/fastrtps*` files during those runs). The lower count is not significant (p = 0.09). |
| d | **From the logs: unclean exits leave stale participants** | **Consistent.** Every Phase 5 shutdown has Python nodes dying mid-cleanup from the double SIGINT (backlog L2: 13–16 tracebacks per shutdown). Some shutdowns end in the supervisor's force-kill path (6 of 30 in baseline). A participant destroyed that way never disposes; the server keeps it for the full 20 s. The next bringup started 5–10 s later in baseline, inside that window. With a 30 s gap or a 5 s lease the window is closed, and nothing fails. |

### B5. Humble comparison: a stack-free reproduction (`scripts/fix_batch_4/synthetic_lifecycle.py`)

The full stack does not run in the Humble container, so I reproduced the
pattern without it, on both distros:
- one Discovery Server;
- `slam.launch.py`'s lifecycle sequence on a minimal rclpy lifecycle node;
- 30 plain-client worker processes started in the same burst;
- each round ends with SIGINT, then SIGKILL after 3 s, and 1 s before the
  next round.

| | server reused | fresh server per round |
|---|---|---|
| **Jazzy** (Fast DDS 2.14.6) | **2 / 30** lifecycle lost (no configure; "Abandoning wait … service response") | **1 / 30** (configured, never activated) |
| **Humble** (Fast DDS 2.6.11, container) | **0 / 30** | **0 / 30** |

- **The mechanism exists without the stack:** the same launch_ros
  lifecycle loss under a Discovery Server.
- **On Jazzy it appears at a low rate** (3/60); on Humble never (0/60).
  That suggests Jazzy (or Fast DDS 2.14) is more exposed. It is not
  conclusive at these counts (Jazzy 3/60 vs Humble 0/60, p ≈ 0.12).
- **The synthetic shows almost no effect of reuse.** It does not carry the
  stack's dozens of non-disposed participants per shutdown.
- **A full-stack Humble comparison needs the Orin:** the same
  `phase5_bringup.sh isolation 30` there.

### B6. Most likely cause and proposed fix

**Cause.** Participants of the previous session that ended without disposing
(killed, or interrupted mid-cleanup) stay in the Discovery Server's database
for their 20 s lease. When the stack is brought up again inside that
window, some new endpoint pairs intermittently never match:
- **services:** the lifecycle `change_state`;
- **topics:** subscriptions.

The node is alive and in the graph, and receives nothing. Jazzy appears
more exposed than Humble (B5).

**The same window opens inside a session.** A component the supervisor
respawns after a crash comes back about 2 s after its old participants
died uncleanly. That case was not measured here; it follows from the
mechanism.

**Proposed fix** (not applied; Decision 1): ship a Fast DDS participant
profile with a **5 s lease and a 1 s announcement**, as tested
(`scripts/fix_batch_4/profiles/short_lease.xml`).
- It would be set as `FASTRTPS_DEFAULT_PROFILES_FILE` by
  `supervisor_bringup.launch.py` and `stack_bringup.launch.py`, in the same
  place they set `ROS_DISCOVERY_SERVER`. It covers every component and the
  server.
- **Why this one:**
  - it closes the window for both cases above, session restarts and
    in-session respawns;
  - it keeps the documented "server outlives the session" design;
  - it measured 0/30.
- **Cost:** about 35 participants announcing at 1 Hz instead of every 3 s,
  which is negligible.
- **Risk:** a participant whose Fast DDS event thread is starved for more
  than 5 s would be dropped. Measure on the Orin under its 78–100% load
  (Phase 3) before relying on it.
- **Alternatives:**
  - **Fresh server per bringup:** 0/30 between sessions, but it does not
    protect in-session respawns.
  - **Wait more than 20 s before starting again:** 0/30, but slow, and
    the same respawn gap remains.
  - **Fix the unclean shutdowns** (backlog L1/L2): reduces how many stale
    participants there are; complements the lease fix rather than
    replacing it.
- **Either way:** the watchdog below is needed to catch what remains.

### B7. Design proposal: supervisor topic-liveness watchdog (not implemented, for approval)

- **Where:** inside `component_supervisor_node`, on its existing executor.
  - It runs as a plain client: its own subscriptions match their publishers
    without super-client rights (fix batch 3).
  - Wall time, like the rest of the supervisor.
- **Configuration:** per component in `components.yaml`, next to the
  launch files, so a new component cannot be added without stating its
  health:

```yaml
slam:
  - package: f1tenth_navigation
    launch_file: slam.launch.py
  health:
    grace_sec: 20            # after (re)start, before checks apply
    lifecycle: [/slam_toolbox]   # must reach ACTIVE (lifecycle_msgs get_state)
    topics:
      - {topic: /slam/pose, max_age_sec: 3.0, when_fresh: /scan}
```

- **What the watchdog watches.** The `when_fresh` field makes each check
  conditional on that input being fresh. This separates *isolation*
  (input flowing, output silent) from an *upstream outage* (input missing
  too), which is not this component's fault.

| Component | Topic(s) | max age | `when_fresh` |
|---|---|---|---|
| localization | `/odometry/filtered`, `/ekf_global/odometry/filtered` | 0.5 s | `/odom` |
| localization | `/joint_states` | 1 s | – |
| slam | `/slam/pose`; lifecycle `/slam_toolbox` ACTIVE | 3 s | `/scan` |
| slam (costmap) | `/costmap/front_clearance` | 1 s | `/slam/map` |
| lidar_front_wall, obstacle_clearance, swept_clearance | `/perception/lidar_front_wall`, `/obstacle_clearance`, `/perception/swept_clearance/lidar` | 0.5 s | `/scan` |
| wall_distance | `/perception/d_wall/segment` | 0.5 s | `/scan` |
| control | `/ackermann_drive` | 0.5 s | `/drive` |
| behavior | `/behavior/tree_status` | 0.5 s | – |
| diagnostics | `/diagnostics/system_status`, `/diagnostics/battery_status` | 3 s | – / `/sensors/core` |
| navigation | (see below) | | |

- **The `mpc_corr` gap.** Without a goal, `mpc_corr` has no output that
  depends on its odometry input (`/drive` is published regardless). The
  detector only caught it through its log.
  - **Proposal:** each such node publishes a small heartbeat per input, for
    example a `DiagnosticStatus` stating the age of its last message on
    each subscribed input.
  - That is a node change, to decide separately (Decision 2b).

- **On failure**, after `fail_for_sec` (default 3 s) of continuous failure
  past the grace period:
  1. **A loud ERROR** naming the component, the topic, its age, and the age
     of its `when_fresh` input.
  2. **A `/supervisor/health` publication** (DiagnosticArray, one status
     per component). The BT or a status display can consume it.
  3. **A per-component failure counter**, in the log and on
     `/supervisor/health`.
  4. **A restart of the component.** It uses the existing restart path and
     restart budget (3 per 60 s), so an isolation and a crash share one
     budget.
  5. **When the budget is exhausted:** the component stays down and
     `/supervisor/health` goes ERROR.
- **Decision 2c:** whether the BT's emergency lane should stop the car on
  `/supervisor/health` ERROR for safety-relevant components (localization,
  slam, control).
- **Lifecycle nodes:** with the lease fix (B6) the lost transition should
  not happen. If one does, the watchdog first re-sends the missing
  `change_state` itself once, before falling back to a restart.
- **Re-isolation after a restart:** a component restarted by the watchdog
  can be isolated again. That is the same in-session window as B6. The
  short lease and the grace period make a second restart start clean.
- **Testing:** the watchdog's evaluation is a pure function (ages in,
  verdicts out) for unit tests. Live, re-run `phase5_bringup.sh isolation`
  on the baseline condition (8/30 today): every affected run must be
  detected and recovered within `grace_sec + fail_for_sec + restart time`.

---

## Backlog changes (`output/backlog.md`)

- **H6 closed** (A).
- **H1 kept HIGH, now with:**
  - the cause;
  - the proposed lease fix;
  - the watchdog design;
  - the slam lifecycle loss as its most common form.
- **L2 (double-SIGINT unclean shutdown) raised to MEDIUM, now M13:** it feeds H1.
- **New:** "Full-stack isolation comparison on the Orin (Humble)".

## Commits

```
b31b76c fix/diagnostics: steering calibration e-stop check waits up to 5 s for the graph — a single read under a new super client refused 3 of 4 starts with the mux and the BT both up
(next) fix/discovery: isolation investigation — scripts, profile, data
(next) fix/batch4: report, backlog, e-stop live data
```

None pushed.

## Decisions for Andreas

1. **Apply the 5 s lease profile?** It would be set as
   `FASTRTPS_DEFAULT_PROFILES_FILE` by both bringup launch files; 0/30
   here, against 8/30. The alternatives are in B6. Recommendation: the
   lease, plus an Orin measurement under load.
2. **The watchdog design (B7).**
   - a. The `components.yaml` health schema and its failure action
     (ERROR, `/supervisor/health`, counter, restart within the existing
     budget).
   - b. Heartbeat diagnostics for nodes with no input-driven output
     (`mpc_corr`).
   - c. Whether the BT stops the car on `/supervisor/health` ERROR.
3. **Run `phase5_bringup.sh isolation 30` on the Orin, natively on
   Humble,** for a full-stack Humble rate. The synthetic test suggests
   Humble is less exposed, but its counts are too small to settle it.
