# Fix batch 2: Phase 5 decisions (Thor, branch `jazzy`)

These are non-migration changes, one `fix/<area>:` commit each, none pushed.
The data is in `output/fix_batch_2/`; mission bags (`*.mcap`) are not committed.

## Verdict: **GO**

All four items are done and verified as asked. One follow-up comes out of
item 2 and is now backlog H6: the stack's other nodes are now plain
Discovery Server clients, unlike (presumably) on the Orin.

---

## 1. Environment script (`d9a7f00`) and `~/.bashrc`

**`scripts/env/jazzy.sh`** is the Phase 5 proposal with these changes:

- **`ROS_SUPER_CLIENT` is no longer exported.** The header explains why:
  every node started from this shell would inherit it.
- **New `ros2cli` helper** runs one `ros2` command as a super client.
  - For the subcommands that read the graph (node, topic list/info/echo/
    find/type, service list/info, param), it also adds `--no-daemon` and
    `--spin-time 5`. These are exactly the subcommands that accept both
    flags in Jazzy's ros2cli.
  - `--no-daemon` is needed because the ros2 daemon keeps the environment
    it was started with.
  - `--spin-time 5` is needed because a new super client needs a few
    seconds to learn the whole graph:

| `ros2cli topic list` / `node list` against the running stack | 3 tries |
|---|---|
| default spin time | 2 / 44 / 2 topics, 0 nodes |
| `--spin-time 1` | full graph in 1 of 3 |
| `--spin-time 2` | full graph in 2 of 3 |
| `--spin-time 3` | full graph in 3 of 3 |
| `--spin-time 5` | full graph in 3 of 3 |

(`output/fix_batch_2/ros2cli_spin_check.txt`)

**`~/.bashrc`** (backup in `~/.bashrc.bak-2026-10-03`), one line changed:

```diff
-source /opt/ros/jazzy/setup.bash
+source ~/dev_ws/f1tenth_more/scripts/env/jazzy.sh
```

**Verification.** The stack was held up with `phase5_bringup.sh hold`,
started from a fresh `env -i … bash -i` shell, so the new `~/.bashrc` was
sourced. `ROS_DOMAIN_ID=85` was exported before the script ran, which keeps
it.

- **Fresh shell:** `ROS_DISCOVERY_SERVER=127.0.0.1:11811`,
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_SUPER_CLIENT` unset, `ros2cli`
  defined (`launch_shell_env.txt`).
- **`ros2cli` against the stack:** 88 topics and 33 nodes in 5 of 5 tries.
  It includes `/scan`, `/odometry/filtered`, `/mission/status`, `/tf` and
  `/mpc/goal_drive`. The same shell's plain `ros2 topic list --no-daemon
  --spin-time 5` shows 2 (`ros2cli_check.txt`).
- **Per-process environment:** read from `/proc/<pid>/environ` for all 45
  stack processes, not a sample (`hold/environ_settled.txt`).
  - **4 have `ROS_SUPER_CLIENT=TRUE`**, exactly those whose launch file
    sets it: `foxglove_bridge`, its two `throttle` nodes, and
    `mission_logger_node`.
  - The other 41 do not.
  - All 45 have `ROS_DISCOVERY_SERVER`.

One caveat on how the stack was started: `phase5_bringup.sh` runs inside
that shell, and it exports the same `ROS_DISCOVERY_SERVER` and domain. It
also unsets `ROS_SUPER_CLIENT`, which is a no-op here because the shell no
longer sets it.

## 2. Mission logger as a super client (`84a12f1`)

**Change:** `mission_logger.launch.py` now sets
`SetEnvironmentVariable('ROS_SUPER_CLIENT', 'TRUE')`. This is scoped to that
launch tree, the same pattern as `foxglove_bridge.launch.py`. Its flake8
count is unchanged (13 findings before and after, all pre-existing).

**Proof:** a full-stack bringup from the new env-script shell, with no
super-client setting in the shell, and one mission through the BT
(`fix_batch_2/logger_from_env/`):

| | Phase 5 Step 4 (no fix) | Phase 5 with super-client shell | **This fix, plain shell** |
|---|---|---|---|
| mission | COMPLETE | COMPLETE | **COMPLETE** |
| bag | mcap, 0 messages, 0 topics | mcap, 3,970 messages, 35 topics | **mcap, 3,668 messages, 35 topics** |
| topic set vs. the 35 | – | reference | **identical** |

- In that run, `ROS_SUPER_CLIENT=TRUE` is present only in the logger and
  the foxglove tree (4 of 45 processes).
- Shutdown exit 0. 0 "Matching unexisting participant".
- The message count differs because the bag-playback timing differs; the
  topic set is the check.

**Side finding (now backlog H6).**
- `foxglove_bridge.launch.py`'s comment says the Orin's `~/.bashrc` also
  sets `ROS_SUPER_CLIENT`. If so, every Humble node is a super client.
  That fits Phase 5's observation that the archived Humble bags have
  content.
- With this batch, Jazzy runs every node except two as a plain client. A
  plain client sees node names but not other participants' topics.
- Code that reads topics from the graph may therefore get nothing. Two
  such call sites, not yet checked live:
  - `ekf_cost_observer_node._subscribe()` (`get_topic_names_and_types()`);
  - `steering_offset_calibration_node`'s `count_publishers(/safety_stop)`
    precheck.

## 3. CLAUDE.md: rebuild policy (`b0740ea`)

The new "Rebuild policy" section says:
- **After any `ros-jazzy-*` apt upgrade**, clean-rebuild every
  `ament_cmake` package with `--cmake-clean-first`. A plain rebuild relinks
  stale objects, because dpkg keeps the headers' original mtimes.
- **Then run the `ldd -r` scan** of `install/`, which must print nothing.
- **The Docker image** builds the workspace against the same apt snapshot
  it runs.

Both commands were run exactly as written on Thor: 13 packages, scan empty.
The scan's filter excludes only message-typesupport symbols; it still
catches the `diagnostic_updater` case.

## 4. `output/backlog.md` (`715907c`)

**New file.** It collects the deferred items from the Phase 0–5, fix batch 1
and migration plan reports, each with its source report and a priority:
- 6 HIGH, to settle before any test drive;
- 12 MEDIUM;
- 13 LOW;
- the 4 Orin runs still pending;
- a "Done" table of what the fix batches closed.

**H1 is the requested item:** the supervisor topic-liveness check, with the
Phase 5 `swept_clearance` evidence and the Orin comparison it needs.

## Commits

```
d9a7f00 fix/env: scripts/env/jazzy.sh for interactive Jazzy shells — hand-started ROS tools need the Discovery Server settings the launch files give the stack
84a12f1 fix/logger: mission_logger.launch.py sets ROS_SUPER_CLIENT for the logger — under the Discovery Server a plain client's recorder finds no topics and the bag is empty
b0740ea fix/docs: CLAUDE.md rebuild policy — an apt ABI change left ackermann_mux unloadable and only a full-stack run found it
715907c fix/docs: output/backlog.md, one list of post-migration items — they were spread over seven reports' decision sections
e248d33 fix/batch2: report and evidence — env script, logger super client, rebuild policy, backlog
```

**Machine change outside the repo:** `~/.bashrc` (one line, backup kept).

`d9a7f00` also carries two additions to `phase5_bringup.sh` that the
verification needed: the per-process environment capture and the `hold`
mode.

## Decisions for Andreas

1. **Backlog H6, plain clients.** Two options:
   - Confirm on the Orin whether `ROS_SUPER_CLIENT` is in the running
     stack's environment.
   - Or accept the Jazzy design (plain by default) and add launch-scoped
     super clients only where live checks show a node needs one. The
     candidates are `ekf_cost_observer` and the steering calibration
     precheck.
