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
f1tenth_ctl killall [--reset-discovery] [--dry-run]
f1tenth_ctl check                            # environment check only
f1tenth_ctl send planner "go ahead 2 meters" # one line to the planner's stdin
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

**Sync with the Orin's `~/.bashrc`.** The ROS part of `ctl.env` copies it on
purpose (see `output/telegram/recon.md`):

- Humble and the workspace;
- `ROS_DISCOVERY_SERVER=127.0.0.1:11811`;
- `ROS_SUPER_CLIENT=TRUE`;
- the CUDA `LD_LIBRARY_PATH`.

When one of the two changes, change the other.

`config/sim_linus.env` is the profile for the Gazebo sim on linus; see that
file and `output/telegram/test.md`.

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
| `estop_clear` | `/mission/status` received and the e-stop not latched | |
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

Two parts run at once, and each reports for itself:

- **The service.** It calls `/mission/emergency_stop` (`std_srvs/Trigger`).
  - The call timeout is 0.5 s; a later answer inside the publish window is
    reported as late.
  - The service latches until `behavior` restarts, and READY then shows
    `estop_clear` failing.
- **Zero drive.** It publishes zero speed and steering on `/safety_stop`,
  the mux's highest-priority lane (priority 200, 0.2 s timeout).
  - Rate 50 Hz, for 2 s counted from the moment the mux matched.

It never stops or kills anything.

**Latency.** Over Telegram and ssh it takes seconds:

| Step | Time |
|---|---|
| ssh | ~0.5 s |
| env sourcing | ~0.6 s |
| Python + rclpy | ~1 s |
| matching | 1–2 s on a loaded machine |

This is not the primary e-stop.

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
   3. `estop` (it latches: restart `behavior` afterwards);
   4. `stop stack`.
   Check `status` timings on the Orin's CPU.
4. Optional: `colcon build --symlink-install --packages-select f1tenth_ctl`
   for `ros2 run f1tenth_ctl f1tenth_ctl`.
