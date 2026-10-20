# f1tenth_ctl

`f1tenth_ctl` starts, stops and checks the car's ROS 2 processes. It runs on
the car (the Orin, Humble). For tests it also runs on linus against the
Gazebo sim (Jazzy). The Telegram bot (`f1tenth_telegram`) calls it over ssh.

- Every command prints one JSON object on stdout (`--json`, the default).
  `--human` prints a short text instead.
- Exit status: 0 = ok, 1 = the command failed, 2 = bad usage or config.
- The CLI is stdlib Python (plus PyYAML) and needs no ROS sourced. Only its
  probe and monitor import rclpy, inside the environment of the env file.

```bash
f1tenth_ctl connect                          # discovery server up + environment check
f1tenth_ctl discovery up|down|status [--force]
f1tenth_ctl start|stop stack|planner|logger
f1tenth_ctl status                           # processes + READY, in < 5 s
f1tenth_ctl estop                            # e-stop service + zero drive; never kills
f1tenth_ctl estop-reset [--force]            # clear the e-stop latch, and check it
f1tenth_ctl killall [--reset-discovery] [--dry-run]
f1tenth_ctl check                            # environment check; warns if it differs from the shell
f1tenth_ctl send planner "go ahead 2 meters" # one line (<= 500 chars) to the planner's stdin
f1tenth_ctl planner-log [--since N]          # the planner's output after byte offset N
f1tenth_ctl missions [-n N]                  # the stored plans, newest first
f1tenth_ctl mission-load <id> [--allow-old]  # validate + load a stored plan
f1tenth_ctl mission-start [--id ID]          # start it: READY, no e-stop, nothing running
f1tenth_ctl mission-status                   # idle/loaded/starting/running/finished/aborted
f1tenth_ctl logs stack|planner|logger|discovery|monitor [-n 30]
```

## Running it

- **From the checkout**, no build needed: `src/f1tenth_ctl/bin/f1tenth_ctl`.
  The bot uses this path, because a non-interactive ssh session does not read
  `~/.bashrc`.
- **Installed**: after `colcon build`, `ros2 run f1tenth_ctl f1tenth_ctl`
  works, and so does `f1tenth_ctl` on PATH once the workspace is sourced.

## The env file

`config/ctl.env` (the car) is the whole configuration. Another file can be
chosen with `--env FILE` or `F1T_CTL_ENV=FILE`. It is a bash file, sourced
with `set -a`, and it holds:

- the ROS and workspace `setup.bash` lines;
- the network variables;
- what each process runs (`F1T_<NAME>_CMD`) and how to recognise one that was
  started by hand (`F1T_<NAME>_NODES` node names, `F1T_<NAME>_MATCH`
  command-line regex);
- the stop waits, the e-stop settings, and the thresholds file.

**Every process f1tenth_ctl starts sources this one file:**

- the units: stack, planner, logger, discovery server, monitor;
- the one-shot probe;
- `ros2 daemon stop`.

**It is always sourced on the same minimal base:** `HOME`, `USER`,
`PATH=~/.local/bin:/usr/...:/bin`, `XDG_RUNTIME_DIR` and a few others. It is
never sourced on top of the caller's shell. A test checks that the units and
the CLI see the same variables.

**One rule for every target: the env file gives exactly the ROS environment
of a shell on that target.** The details are in `output/telegram/discovery.md`.

- **Car:** `config/ctl.env` is a literal copy of the ROS part of the Orin's
  `~/.bashrc`:
  - Humble and the workspace;
  - `ROS_DISCOVERY_SERVER=127.0.0.1:11811`;
  - `ROS_SUPER_CLIENT=TRUE`;
  - the CUDA `LD_LIBRARY_PATH`.

  Read on 2026-10-20, it was identical on all 13 compared variables.
- **Sim:** `config/sim_linus.env` sources jazzy's own `scripts/sim/sim_net.sh`,
  with the discovery server on loopback. See also `output/telegram/test.md`.

`check` compares the env file with the reference shell `F1T_SHELL_REF`:

- `interactive` (the default) means `bash -i`, i.e. `~/.bashrc`; any other
  value is a bash snippet;
- it checks the domain, discovery server, super client, RMW, Fast DDS/Cyclone
  profiles, `AMENT_PREFIX_PATH` and `LD_LIBRARY_PATH`;
- a difference is a warning (`shell_env`), never a failure: it means the bot
  and a shell on the target see, or run, different things.
- `F1T_SHELL_REF=off` disables the comparison.

## Processes are transient systemd user units

Each process runs as `systemd-run --user --unit=f1t-<name>`:

| Name | What it runs |
|---|---|
| `stack` | `F1T_STACK_CMD` |
| `planner` | `F1T_PLANNER_CMD` |
| `logger` | `F1T_LOGGER_CMD` |
| `discovery` | `fastdds discovery ...` |
| `monitor` | the READY monitor |

The reasons, and the comparison with tmux, are in `output/telegram/recon.md`
section 7.

Logs: `~/.local/state/f1tenth_ctl/logs/<name>.log`, or `f1tenth_ctl logs <name>`.

**States:**

| State | When |
|---|---|
| `running` | the unit is active |
| `stopped` | there is no unit, or it was stopped by f1tenth_ctl |
| `crashed` | the unit failed (exit code or signal reported), or it went away although nobody stopped it |

A process that runs but was not started by f1tenth_ctl shows as `running`
with `managed: false`. It is found by its command line, or by its node name
in the graph.

**Start refuses to run twice:**

- the unit is already active;
- a matching process runs outside the unit;
- or one of its nodes is already in the graph.

This is the `ros2 node list` check: the node list is read as a super client.

**Stop is a ladder:** SIGINT, up to 10 s; SIGTERM, up to 5 s; SIGKILL.

- SIGINT goes to the main process's process group, which is exactly what
  Ctrl-C in a terminal does.
- `ros2 run` relies on this: it does not forward SIGINT to its child.
- Process groups the stack made for itself (the supervisor's components) are
  left to the stack, as with Ctrl-C.
- `stop` also applies the ladder to a copy that was started by hand.

**Survives the ssh session:** the user manager keeps the units. On the Orin
Linger is on; check it with `loginctl show-user $USER -p Linger`.

## status and READY

READY needs all of these:

- the discovery server listens (when one is configured);
- the stack runs;
- every check with `required: true` in `config/ready.yaml` passes.

| Check | Topic or source | Threshold |
|---|---|---|
| odometry | `/odometry/filtered` rate | ≥ 10 Hz |
| scan | `/scan` rate | ≥ 20 Hz |
| `tf_map_base_link` | age of the latest map→base_link | ≤ 0.5 s |
| `mission_service` | `/mission/load_mission` available | |
| `estop_latched` | `/mission/status` received and the e-stop not latched | |
| battery | `has_data` and voltage | ≥ 10.8 V |

`wheel_odom` (`/odom`) is shown but not required.

The answer lists the failing checks and the values: rates, TF age, battery,
mission state.

**The measurements come from the monitor.** The monitor is a long-running
node (`f1t-monitor`):

- it rewrites a JSON snapshot every 0.5 s, and status reads that file;
- it runs only while a stack runs: `start stack` starts it, and a status
  with no stack stops it;
- that leaves no extra node on an idle car.

**Why not a one-shot probe:** on a loaded machine (Gazebo + stack on linus,
load 17 on 8 cores) a fresh probe needed more than the 5 s budget to match.
It reported a running stack as silent about half the time.

With the monitor, status answers in about 1.2 s and READY was stable over
every run. `F1T_MONITOR=false` falls back to the one-shot probe. It costs one
Python node with raw subscriptions to the topics above.

## estop

**Three steps, started together.** Each is best effort with its own timeout,
and none waits for another:

| Step | What it does | Timeout |
|---|---|---|
| `zero_drive` | Zero speed and steering on `/safety_stop`, the mux's highest-priority lane (priority 200, 0.2 s timeout). It starts at once, at 50 Hz, and lasts 2 s counted from the moment the mux matched; only messages after that count | waits up to 3 s for the mux |
| `emergency_stop` | `/mission/emergency_stop`: latches the behaviour tree's own zero on the same lane until `behavior` restarts. Status then says `estop_latched: true`, and READY fails | 0.5 s per call; a later answer inside the window is taken, marked `late` |
| `abort_mission` | `/mission/abort_mission`: ends a LOADED/RUNNING mission and puts mpc_corr on `/mpc/hold`, so nothing resumes when the latch is cleared. "No mission to abort" counts as ok | 1 s per call |

**The answer.** Each step reports `status` (`ok` / `failed` / `skipped`),
`message`, `duration_ms` (from the start of the estop) and, for a service,
`call_ms`. Overall:

`car_held = zero_drive ok AND (emergency_stop ok OR abort_mission ok)`

- With the discovery server down, nothing can be reached: every step is
  `skipped` and the answer comes in about 0.5 s.
- With no graph at all, the steps fail after the 5 s window.
- It never stops or kills anything.

**What it guarantees, and what it does not.**
- **When `car_held` is true:** at that moment the mux had a zero command on
  its top lane, and either the tree's own latched zero or the MPC hold was
  engaged.
- **It does not confirm the car stopped.** It reads no odometry, and a car
  already rolling still needs its braking distance.
- **The 0.2 s mux timeout.** The zero drive ends after 2 s. Afterwards only
  the tree's 10 Hz zero (the latch) and the MPC hold keep the car still. On a
  loaded machine the mux can let a single lower-lane command through between
  two late zeros (`output/telegram/test.md` §7).
- **If `behavior` crashes or is restarted while latched,** the latch is gone;
  only the MPC hold from `abort_mission` remains.
  `output/telegram/estop_proposal.md` proposes the fix.
- **It needs the stack's mux, `behavior` and the network path.** None of them
  is guaranteed when something is badly wrong.

**Latency.** Over Telegram and ssh it takes seconds:

| Step | Time |
|---|---|
| ssh | ~0.5 s |
| env sourcing | ~0.6 s |
| Python + rclpy | ~1 s |
| matching | 1–2 s on a loaded machine |

**This is not the primary e-stop.** The RC transmitter, or the physical
switch, is.

## estop-reset

**The latch.**
- `/mission/emergency_stop` latches for the lifetime of
  `behavior_executor_node`.
- There is no reset service, on purpose: see the `mission/loader.py` and
  `MissionStatus.msg` docstrings.
- The documented way out is restarting the node.

**The paths, cleanest first.** `estop-reset` takes the first one available:

1. **Reset service** (`F1T_ESTOP_RESET_SERVICE`). Empty: none exists today.
   The setting is there for when one does.
2. **Restart `behavior` alone**, through the supervisor's
   `/restart_component {component_name: behavior}`. This is the path on the
   car today.
3. **Full stack stop + start.** Only when there is no supervisor (e.g.
   `stack_bringup.launch.py`), and only if f1tenth_ctl started the stack.

**Why path 2 is guarded.** The e-stop does not touch the mission or
`/mpc/hold`. While the latch holds, the behaviour tree's emergency lane
publishes zero on `/safety_stop` (priority 200) and masks whatever the lower
lanes ask for. Restarting `behavior` removes that lane: whatever a lower lane
still commands, for example a RUNNING mission's mpc_corr on `/drive`, then
reaches the motor.

Measured in the sim, with `/teleop` (priority 100) asking for 0.3 m/s the
whole time:

| Restart | Car moved before the latch read clear |
|---|---|
| plain restart | **1.53 m** |
| `estop-reset` | 0.042 m and 0.000 m in two runs |

At load 17–28 the mux lets a single lower-lane command through now and then
when the top lane is a few ms late. In the 0.042 m run, the latched tree
itself leaked 0.008 m in 3 s; in the 0.000 m run, the logged mux output shows
one such sample on each side.

**The sequence.** `estop-reset`:

1. reads the latch on `/mission/status`. Nothing to do if it is clear;
   `--force` runs anyway.
2. publishes zero drive on `/safety_stop` at 50 Hz, and goes on only once the
   mux has matched it.
3. calls `/mission/abort_mission`, so a LOADED/RUNNING mission is aborted and
   mpc_corr is put on `/mpc/hold`. Nothing resumes after the reset.
4. restarts `behavior` (path 2).
5. waits for the new node's `/mission/status` with the latch clear. That is
   the verification, `verified: true`. Then it keeps the zero drive 0.5 s
   longer.

The JSON names the path taken (`path`), `latched_before`, `latched_after`,
`verified`, and the result of each step. In the sim: path 2 took 7–12 s,
path 3 took 18 s.

## The planner's stdin and output

**`send planner "<text>"`** writes one line to the planner's stdin FIFO.
- It refuses a text with a line break, an empty one, or one over 500
  characters.
- It answers `cursor`: the size of the planner's log just before the write.

**`planner-log --since <cursor>`** returns the complete lines the planner
printed after that byte offset, ANSI codes stripped, and the next `cursor`.
- A line still being written comes whole on the next call.
- `running` says whether the planner unit runs.
- Without `--since`, it returns only the current end, as `cursor`.
- A cursor past the end means the log was replaced: it reads from the start
  (`reset: true`).

The Telegram bot uses the pair to follow one command's answer
(`src/f1tenth_telegram/README.md`, "Talking to the planner").

## Planned missions: plan-only planner, then load and start

**The planner runs plan-only.** `F1T_PLANNER_CMD` is
`ros2 run llm llm_planner_node --dry-run`, the planner's own flag:
- each command is translated, printed and written to its file;
- it **never** calls `/mission/abort_mission`, `/mission/load_mission` or
  `/mission/start_mission`;
- run by hand without `--dry-run`, the planner is unchanged and starts what
  it plans.
- `planner-log` reports `plan_only`. It is `true` only when one of the
  planner unit's processes runs `llm_planner_node` with `--dry-run`, read
  from `/proc/<pid>/cmdline`, not from the env file. It is `null` when the
  planner is stopped. The bot refuses to plan unless it is `true`.

**Storing the plan.** Its last line per command is `RESULT {json}`; with
status `ok` it carries the whole mission (`mission`). When `planner-log`
reads such a line:
- the mission is stored under `~/.local/state/f1tenth_ctl/missions/` as
  `<id>.mission.json` (byte for byte what the planner made) and `<id>.json`
  (summary, unsupported parts, notes);
- the id is `<YYYYmmddTHHMMSS>-<sha1 6>`;
- the answer lists it under `results` (`stored`, `mission`);
- one RESULT line is stored once, keyed by its byte offset in the planner
  log;
- a plan is never regenerated.

| Command | What it does | Refuses |
|---|---|---|
| `missions [-n N]` | the stored plans, newest first: id, age, summary, whether still startable | — |
| `mission-load <id>` | validates the stored file with `f1tenth_behavior.mission.mission_config.load_mission_file`, the parser `/mission/load_mission` itself uses, then loads it (`/mission/load_mission`, `f1tenth_messages/srv/LoadMission`, `path`) | a plan older than `F1T_MISSION_MAX_AGE_S` (300 s; `--allow-old`), an invalid file, a stack that is not running, a RUNNING/HOLDING mission (the loader would replace it mid-run), a start still counting down |
| `mission-start [--id ID]` | starts the mission f1tenth_ctl loaded (`/mission/start_mission`, Trigger). The loader then counts down `mission_countdown_sec` (15 s) | nothing loaded by f1tenth_ctl, another id loaded, a plan too old, READY false (which covers a latched e-stop), another mission running, a different file loaded in the meantime |
| `mission-status` | `/mission/status` + what f1tenth_ctl loaded: state, id, elapsed since the start | — |

**States:**

| `/mission/status` | `mission-status` |
|---|---|
| IDLE | idle |
| LOADED | loaded, or `starting` during the countdown after our start |
| RUNNING / HOLDING | running |
| COMPLETE | finished |
| ABORTED | aborted |

`status` carries the same `mission` field, for the panel.

**Every refusal** has its reason in `refused`.

**The summary** (`missions.summarize`) is read from the mission document:
- move by move: action, speed, stop condition, timeout;
- max speed, targets and length (the sum of distance-based stops; "not known"
  when a move ends on a condition);
- the longest it can take (the sum of the timeouts);
- reverse: always no, since this schema has no reverse.

## killall

1. Stops the managed units (stack, planner, logger, monitor) in parallel,
   each with the stop ladder.
2. Signals every other ROS process of this user:
   - SIGINT, up to 5 s; SIGTERM, up to 3 s; SIGKILL;
   - the matching rules are those of `scripts/kill_ros2.py`, on `/proc`;
   - f1tenth_ctl never matches itself or its ancestors.
3. Runs `ros2 daemon stop`.
4. Runs `fastdds shm clean`, which removes only segments no live process holds.

**The discovery server stays up** unless `--reset-discovery`:

- If it went down with the stack (the stack's launch had started it), it is
  started again.
- `--dry-run` lists what would be signalled.

## On the Orin

These steps were not done yet.

1. Merge `telegram-control` into the branch the Orin runs, then `git pull`
   there. No build is needed for the CLI: `bin/f1tenth_ctl` runs from the
   checkout.
2. On the Orin, check the environment:
   `~/dev_ws/f1tenth_more/src/f1tenth_ctl/bin/f1tenth_ctl check --human`.
3. With the car on stands:
   1. `connect`, then `start stack`;
   2. `status --human` until READY;
   3. `estop`, then status shows `estop_latched: true`;
   4. `estop-reset`, then status is READY again;
   5. `stop stack`.
   Check `status` timings on the Orin's CPU.
4. Optional: `colcon build --symlink-install --packages-select f1tenth_ctl`
   for `ros2 run f1tenth_ctl f1tenth_ctl`.
