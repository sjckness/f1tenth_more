# Discovery server and ROS_SUPER_CLIENT (2026-10-20)

The questions from the follow-up, answered one by one. Read-only checks on
the Orin: two environment dumps over ssh to stdout. Nothing was written there.

## 1. What the first round changed

| File | Change | Processes affected |
|---|---|---|
| `config/sim_linus.env` | During the first sim test, the first version said `ROS_SUPER_CLIENT=TRUE`, copied from the car profile. It became `unset ROS_SUPER_CLIENT ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS ROS_LOCALHOST_ONLY` before commit `c9338de` | everything started with the sim profile: the stack unit (the supervisor and every component launch it spawns), the planner and logger units, and Gazebo, started by hand with the same env file |
| `f1tenth_ctl/cli.py` `_probe_env` | sets `ROS_SUPER_CLIENT=TRUE` for the one-shot probe when a discovery server is configured | the probe only |
| `f1tenth_ctl/monitor.py` | the unit runs `env ROS_SUPER_CLIENT=TRUE python3 -m f1tenth_ctl.rosprobe monitor` | the monitor only |
| `config/ctl.env` (car) | nothing: `ROS_SUPER_CLIENT=TRUE`, as `~/.bashrc` | — |

**It was sim-only.** The Orin profile kept the global super client.

**The load claim, made precise:**
- Two changes went in together: `/clock` 250 → 50 Hz, and super client off.
- Load went from 37 to 17–20 on 8 cores.
- I did not measure the two separately.
- What was measured: the discovery server alone used 50 % of a core while
  ~40 participants were super clients.

## 2. This round

- **`config/ctl.env` is now a literal copy of the ROS part of `~/.bashrc`.**
  It no longer spells out `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and
  `ROS_DOMAIN_ID=0`. Those were the defaults the bashrc leaves unset, so
  behaviour is unchanged, but a raw diff showed them.
  It sets `F1T_SHELL_REF=interactive`, the reference for `check` (§5).
- **`config/sim_linus.env` now sources jazzy's own `scripts/sim/sim_net.sh`**,
  with the discovery server on loopback. It no longer re-declares the
  variables by hand. Compared with the sim scripts, the hand-made copy
  differed in two variables, which `check` now reports:
  - `FASTRTPS_DEFAULT_PROFILES_FILE`: unset vs the stack's
    `fastdds_profile.xml` (5 s participant lease);
  - `ROS_AUTOMATIC_DISCOVERY_RANGE`: unset vs `SUBNET`.

  Now they are equal by construction. Its `F1T_SHELL_REF` is the same
  `sim_net.sh` snippet.

## 3. Does `config/ctl.env` match the Orin's `~/.bashrc`?

How it was compared:
- **Shell side:** `env -i HOME=… bash -i -c 'env -0'` on the Orin (reads
  `~/.bashrc`).
- **ctl.env side:** `env -i HOME=… bash -c 'set -a; source /dev/stdin; env -0' < ctl.env`.
- Both dumps came back on stdout and were compared on linus with
  `envcheck.diff` (13 variables, below).

| Variable | Orin `~/.bashrc` | ctl.env, first round | ctl.env, now |
|---|---|---|---|
| `ROS_DISTRO` | `humble` | same | same |
| `ROS_DOMAIN_ID` | unset (0) | `0` (raw diff, same behaviour) | unset |
| `RMW_IMPLEMENTATION` | unset (`rmw_fastrtps_cpp`) | `rmw_fastrtps_cpp` (raw diff, same behaviour) | unset |
| `ROS_LOCALHOST_ONLY` | `0` | same | same |
| `ROS_DISCOVERY_SERVER` | `127.0.0.1:11811` | same | same |
| `ROS_SUPER_CLIENT` | `TRUE` | same | same |
| `ROS_AUTOMATIC_DISCOVERY_RANGE`, `ROS_STATIC_PEERS` | unset (Humble) | same | same |
| `FASTRTPS_/FASTDDS_DEFAULT_PROFILES_FILE`, `CYCLONEDDS_URI` | unset | same | same |
| `AMENT_PREFIX_PATH` | 39 entries | identical | identical |
| `LD_LIBRARY_PATH` | 15 entries, CUDA first | identical | identical |

**Result: identical on every variable now.** In the first round the only
differences were two explicit defaults.

## 4. Consequences when the env file and the shell differ

**Domain, discovery server address or RMW differ.**
- These are two disjoint graphs.
- `ros2 topic list` in an ssh shell does not show the stack the bot started,
  and the bot's status does not show a stack started from the shell.
- `start` refuses a second stack by process table and by node list. The
  process table still catches a hand-started stack on the same host; the node
  list would not.
- Two stacks could then run at once and fight over the VESC serial port and
  the lidar.

**`AMENT_PREFIX_PATH` differs.** The bot's stack runs different code (another
install space) from a hand-started one.

**`LD_LIBRARY_PATH` differs.** Without the CUDA path, `import torch` fails
(libcudss), so perception crashes in one of the two stacks only.

**Only `ROS_SUPER_CLIENT` differs.**
- Same graph, same data flow: plain and super clients of one server exchange
  data normally.
- What changes is what a participant can list, and the server's load.
- Measured in the sim, one participant listening for 5 s:

  | | Nodes seen | Topics seen |
  |---|---|---|
  | plain client | 67 | 2 |
  | super client | 67 | 137 |

  Node names come through; topic names do not.
- **For the bot: no difference.** Its probe and monitor are super clients
  either way.
- **For a shell:** `ros2 topic list`, `ros2 topic echo` and the ros2 daemon
  need it. That is why `~/.bashrc` sets it. Run the CLI with `--no-daemon`
  (`ros2 topic list --no-daemon`): a daemon started from another environment
  answers for its own graph, not this one.
- **For the stack:** anything that finds topics through the graph needs it:
  - the mission logger's `rosbag2_py.Recorder` learns the types of the topics
    it records from the graph;
  - so do topic_tools throttles;
  - `viz_relays.launch.py` and `foxglove_bridge.launch.py` already set it
    themselves for that reason.
  - Node-name lookups work without it: mission preflight and
    diagnostics_server use `get_node_names()`.
- **The ros2 daemon** keeps the environment of the shell that started it.
  f1tenth_ctl does not use it, and `killall` stops it.

## 5. The proposed setup

### Rule, implemented now

**f1tenth_ctl's env file for a target gives exactly the ROS environment of a
shell on that target.**

| Target | Env file | Equals |
|---|---|---|
| car | `ctl.env` | `~/.bashrc` |
| sim | `sim_linus.env` | jazzy's `sim_net.sh`, by sourcing it |

`f1tenth_ctl check` (and so the bot's 🔌 Connect) runs the reference shell
`F1T_SHELL_REF` on the target:
- `interactive` means `bash -i`, i.e. `~/.bashrc`;
- anything else is a bash snippet.

It compares the 13 variables above and adds a non-blocking warning,
`shell_env`, that lists each difference. Defaults count as equal (an unset
domain is 0). For paths it names the entries found on one side only.

**Better still: make `~/.bashrc` source `ctl.env`.** Replace the ROS block in
`~/.bashrc` (the two `source` lines and the three exports) with:

```bash
set -a; source ~/dev_ws/f1tenth_more/src/f1tenth_ctl/config/ctl.env; set +a
```

The `set -a` is needed: `ctl.env` assigns without `export`, the way
f1tenth_ctl sources it.

That makes one file the single source of truth for both the shell and the
bot. It is your call; I did not touch `~/.bashrc`.

### Model, proposed but not applied to the car

**Stack participants are plain discovery-server clients; only introspection
is a super client.**

That is what `~/.bashrc`'s own comment says it wants: "the actual pub/sub
nodes in the stack … deliberately do NOT get this". `foxglove_bridge.launch.py`
says the same, and the jazzy line already does it. Today it does not happen:
the export reaches every node of a stack started from a shell, and every
participant then receives every endpoint from the server.

It is not applied to the car because it changes the stack's runtime and
needs four things done together, then a run on the car. If done at all, it
should be done as one change:

1. **`~/.bashrc`:** drop `export ROS_SUPER_CLIENT=TRUE`. Make only the
   introspection verbs super clients, so that `ros2 launch` and `ros2 run`
   stay plain:

   ```bash
   ros2() {
     case "$1" in
       node|topic|service|param|action|daemon|doctor|bag)
         ROS_SUPER_CLIENT=TRUE command ros2 "$@" ;;
       *) command ros2 "$@" ;;
     esac
   }
   ```

2. **`mission_logger.launch.py`:** `SetEnvironmentVariable('ROS_SUPER_CLIENT', 'TRUE')`
   for its recorder and throttles, as `viz_relays.launch.py` does. Without it
   the recorder cannot see the topics it is told to record.
3. **Other graph users:** `steering_offset_calibration_node` uses
   `count_publishers(/safety_stop)`. Check whether it needs the same.
4. **`ctl.env`:** drop `ROS_SUPER_CLIENT=TRUE` in the same commit. `check`
   would flag any mismatch with the bashrc.

**On the car, then check:**
- a mission passes preflight (`get_node_names()` as a plain client on
  Humble's Fast DDS 2.6; the sim result above is Jazzy, 2.14);
- the logger's bag has every topic;
- foxglove still sees the graph;
- the discovery server's CPU, before and after.

Until then the car keeps the global super client. The rule keeps the bot and
the shell identical.
