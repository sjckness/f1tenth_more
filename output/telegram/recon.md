# Telegram control: recon (2026-10-20)

Read before any code was written. Branch `telegram-control`, cut from
`campaign-final` `cd180d8`. Paths are relative to the repository root and
refer to that commit unless a branch is named.

## 0. Branch

| Branch | Tip | Date | What it is |
|---|---|---|---|
| `campaign-final` | `cd180d8` | 10-20 | Humble car line. Same commit on local, `origin` and `orin` |
| `jazzy` | `3dbe37f` | 10-13 | Jazzy port + Gazebo sim (two-machine: sim on linus, stack on the Thor) |
| `jazzy-campaign` | `60f9af7` | 10-19 | Jazzy + Isaac sim line, main checkout on linus (dirty) |

On 2026-10-20 the Orin had `door-fixes` (`486a157`) checked out: that is
`campaign-final` plus 15 door-detection commits, not on GitHub. The base is
`campaign-final`, the line the Orin builds from. `telegram-control` touches
none of the door files, so it merges into either one.

## 1. The stack and what runs inside it

**The stack is `f1tenth_bringup supervisor_bringup.launch.py`.** It is the
process that was running on the Orin (`ros2 launch f1tenth_bringup
supervisor_bringup.launch.py`, `ps` on 2026-10-20). It starts the discovery
server and `component_supervisor_node`, which then runs one `ros2 launch` per
component listed in `src/f1tenth_bringup/config/components.yaml`.
`stack_bringup.launch.py` is the older single-process fallback. It starts
neither the planner nor the logger.

**`llm_planner_node` is never started by the stack.**
- `src/f1tenth_intelligence/llm/launch/llm.launch.py:1-16`: the
  `intelligence` component starts llama-server only. The planner node was
  removed from the launch on purpose because it reads its commands from argv
  or stdin, and `ros2 launch` gives it no terminal.
- `stack_params.yaml` `enable_intelligence` (default `true`) gates
  llama-server only. Its description says: "llm_planner_node is never
  auto-launched by this or any flag".
- The planner is a CLI: `ros2 run llm llm_planner_node "<command>"`
  (one-shot), or with no argument an interactive loop that reads `comando> `
  lines from stdin and exits on EOF
  (`llm/llm_planner_node.py:1003-1011, 1458-1500`).

**`mission_logger_node` is inside the stack, and off by default.**
- `components.yaml` `diagnostics` includes `f1tenth_logger
  mission_logger.launch.py`.
- It is gated by `enable_mission_logger`:
  - a real launch argument of `supervisor_bringup.launch.py`;
  - default `false` since 2026-10-16 (`stack_params.yaml:2301`);
  - forwarded by `component_supervisor_node.py:1021-1051`.
- `mission_logger.launch.py enable_mission_logger:=true` already runs
  standalone. It declares all its own arguments.
- `mission_logger_node` holds a PID lock (`/tmp/mission_logger.lock`), so a
  second instance refuses to start.

**Decision: no launch file changes.** Both proposed switches already exist,
with the default unchanged:

| Wanted | What exists |
|---|---|
| `with_logger` | `enable_mission_logger`, default `false` |
| standalone logger | `ros2 launch f1tenth_logger mission_logger.launch.py enable_mission_logger:=true` |
| `with_planner` | nothing to switch: the stack never starts the planner |
| standalone planner | cannot be a launch file (it reads stdin) |

- Adding `with_logger` would give one behaviour two names.
- `f1tenth_ctl` runs the planner as `ros2 run llm llm_planner_node` in
  interactive mode, with stdin on a FIFO that the unit holds open (§7). It
  stays up, and `f1tenth_ctl send planner "<command>"` feeds it one line.
- llama-server stays in the stack (`intelligence`). Without it the planner's
  warm-up fails, and the planner exits 1, which `f1tenth_ctl` reports as
  crashed.

## 2. ROS networking

### Orin (Humble, native, user `fabiocar`)

Read over ssh on 2026-10-20. The ROS part of `~/.bashrc` is:

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/f1tenth_more/install/setup.bash
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_SUPER_CLIENT=TRUE
export LD_LIBRARY_PATH="/home/fabiocar/.local/lib/python3.10/site-packages/nvidia/cu12/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

Nothing ROS-related is in `~/.profile`, `/etc/environment` or
`/etc/profile.d/`.

| Setting | Value | Where |
|---|---|---|
| `RMW_IMPLEMENTATION` | unset, so the Humble default `rmw_fastrtps_cpp` | nowhere on the car. Only the dev containers set it (`.devcontainer/docker-compose.yml:37,61,97`) |
| `ROS_DOMAIN_ID` | unset, so 0 | nowhere on the car |
| `ROS_DISCOVERY_SERVER` | `127.0.0.1:11811` | `~/.bashrc`, and `SetEnvironmentVariable` in `supervisor_bringup.launch.py:66-67` and `stack_bringup.launch.py`. Composed from `stack_params.yaml:2546-2551` `discovery_server_address` / `discovery_server_port` |
| `ROS_SUPER_CLIENT` | `TRUE` for every process started from a login shell | `~/.bashrc`. In the repo, only `foxglove_bridge.launch.py:111` and `viz_relays.launch.py:112` set it |
| XML profiles | none | `FASTRTPS_DEFAULT_PROFILES_FILE` is never set on this line |
| Docker | not used on the car today | `docker/orin/` is the Jazzy image of Phase 5d |

**Discovery server.**
- The stack starts it with `python3
  <share>/f1tenth_bringup/scripts/ensure_discovery_server.py 127.0.0.1 11811`
  (`supervisor_bringup.launch.py:98-106`, `respawn=True`).
- That script tries to bind the UDP port.
  - Port free: it execs `fastdds discovery --server-id 0 --udp-address
    127.0.0.1 --udp-port 11811` through `/bin/sh`, because Humble's `fastdds`
    has no shebang.
  - Port taken: it reuses the running server and idles on `tail -f
    /dev/null`.
- So a server started before the stack is reused, and it outlives the stack.
  `f1tenth_ctl discovery up` relies on this.
- Server ID 0, UDP, loopback only.

**Contradiction, kept as it is.**
- The `foxglove_bridge.launch.py:95-99` docstring says the super client is
  deliberately not stack-wide.
- But the stack is started from a shell that exports it, so every node is a
  super client.
- `config/ctl.env` copies the bashrc, so a stack started by `f1tenth_ctl`
  runs exactly as one started by hand.

### linus (Ubuntu 24.04, Jazzy, user `andreas`)

- `~/.zshrc:126` only runs `source /opt/ros/jazzy/setup.zsh`.
- That sets `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, Jazzy's set-if-unset
  default (`/opt/ros/jazzy/share/ros_environment/environment/1.ros_discovery.dsv`).
- No domain and no discovery server are set in the shell.
- The sim scripts set their own:
  - `jazzy:scripts/sim/sim_net.sh`: domain 42, `ROS_DISCOVERY_SERVER=10.42.0.2:11811`
    (the Thor), `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and
    `jazzy:src/f1tenth_bringup/config/fastdds_profile.xml` (5 s lease).
  - `jazzy-campaign:scripts/sim/sim_isaac.sh`: `--net local|eth`, domain 42,
    `config/sim/dds/{local,isaac_local,eth.xml.in}`.
- linus has no `tmux`, Linger is off, and sshd runs.

## 3. Emergency stop and the mux

**Emergency stop service: `/mission/emergency_stop`, `std_srvs/srv/Trigger`.**
- Absolute name, empty request. Served by `MissionLoader`
  (`f1tenth_behavior/mission/loader.py:257-258`) inside
  `behavior_executor_node` (component `behavior`).
- A call (`loader.py:609-639`):
  - latches `emergency_stop_active`;
  - if a start countdown is running, cancels it, aborts the LOADED mission
    and publishes `/mpc/hold`=true;
  - republishes `/mission/status`;
  - always answers `success=true`.
- The BT reads the latch from `/mission/status` (`IsEmergencyStopTriggered`).
  Its emergency lane then publishes zero speed on `/safety_stop` every tick,
  at 10 Hz (`bt_tick_hz`).
- **The latch has no reset service.** Only a restart of `behavior` clears it:
  `/restart_component {component_name: behavior}`.
- `/mission/abort_mission` (Trigger) is the gentler stop that does not latch.

**ackermann_mux lanes** (`src/f1tenth_bringup/config/mux.yaml`, each with a
0.2 s timeout, no locks):

| Lane | Topic | Priority |
|---|---|---|
| `safety_stop` | `/safety_stop` | **200 (highest)** |
| `joystick` | `/teleop` | 100 |
| `calibration` | `/calibration_drive` | 50 |
| `navigation` | `/drive` | 10 |

- Inputs are `ackermann_msgs/msg/AckermannDriveStamped`.
- The output `ackermann_cmd` is remapped to `/ackermann_drive`
  (`ackermann_mux.launch.py:87-93`).
- Because of the 0.2 s timeout, a zero command on `/safety_stop` must be
  repeated at **≥ 20 Hz**. `reset_manager.yaml` and `hard_stop_node` both use
  20 Hz. `f1tenth_ctl estop` publishes at 50 Hz for 2 s.
- **Sim:** the Gazebo `drive_bridge` reads the mux output on `jazzy`, so the
  override works there. On `campaign-final` it reads `/drive` and bypasses the
  mux.

## 4. READY signals

| Check | Topic / source | Type | Nominal | READY threshold (`config/ready.yaml`) |
|---|---|---|---|---|
| Odometry (EKF) | `/odometry/filtered` | `nav_msgs/Odometry` | 20 Hz (`ekf_frequency_hz`, `stack_params.yaml:2194`) | ≥ 10 Hz |
| Wheel odometry | `/odom` | `nav_msgs/Odometry` | ~50 Hz (VESC) | reported only |
| Scan | `/scan` | `sensor_msgs/LaserScan` | 40 Hz (urg_node, `sensors.yaml`) | ≥ 20 Hz |
| TF map→base_link | `/tf`: map→odom from `ekf_global` (`ekf_global.yaml:102`), odom→base_link from `ekf_filter_node` (`ekf.yaml:34,50`) | | 20 Hz | age ≤ 0.5 s |
| Mission service | `/mission/load_mission` | `f1tenth_messages/srv/LoadMission` | | available |
| E-stop latch | `/mission/status.emergency_stop_active` | `f1tenth_messages/MissionStatus`, transient-local | on change | must be false |
| Battery | `/diagnostics/battery_status` | `f1tenth_messages/BatteryStatus` (`has_data`, `voltage`, `ok`) | 2 Hz | `has_data` and voltage ≥ 10.8 V (`min_battery_voltage`, `stack_params.yaml:154`) |

- slam_toolbox publishes no TF on the car
  (`slam_toolbox_params.yaml:67`, `transform_publish_period: 0.0`). map→odom
  comes from the global EKF, which fuses `/slam/pose`.
- There is no ready aggregator in the stack:
  - `component_supervisor_node` has no status topic or service;
  - `mission/preflight.py` checks only per mission (nodes `mpc_corr`,
    `ackermann_to_vesc_node`, odometry).
- `f1tenth_ctl status` therefore measures directly.

## 5. Orin SSH host

- linus `~/.ssh/config`: `Host jetson-orin`, `HostName 100.96.201.7`,
  `User fabiocar`.
- Tailscale name `jetson-orin`, MagicDNS `jetson-orin.tail5ddb73.ts.net`.
- The git remote `orin` is `fabiocar@jetson-orin:dev_ws/f1tenth_more`.
- `config/telegram.yaml` uses `host: jetson-orin`, a config value.
- On the Orin:
  - Linger is on, and `KillUserProcesses` is at its default `no`;
  - `systemctl --user` works over ssh;
  - tmux 3.2a is installed;
  - a user unit already exists: `f1tenth-archive.service`/`.timer`.

## 6. Prior art reused

- **`scripts/kill_ros2.py`.** It needs psutil, which is not stdlib.
  `f1tenth_ctl killall` reimplements its process matching on `/proc`, with two
  changes:
  - it never matches its own process or its ancestors;
  - it leaves the discovery server alone unless asked.
- **`scripts/stackctl.py`.** It works through the supervisor's services and
  needs a 10 s settle. `f1tenth_ctl` manages processes instead, and is
  limited to 5 s for status.

## 7. Process manager: transient systemd user units, not tmux

Each managed process is `systemd-run --user --unit=f1t-<name>`. The reasons,
in order of weight:

1. **The cgroup holds the whole tree.**
   - `component_supervisor_node` spawns its own `ros2 launch` children, and
     llama-server runs under them.
   - Stopping the unit reaches every one of them.
   - A tmux pane only owns its process group. That is why `kill_ros2.py`
     exists.
2. **Crash state comes from systemd.**
   - A unit that exits non-zero stays `failed` with its `Result` and exit
     code, which is reported as `crashed`.
   - A stop by `f1tenth_ctl` clears it with `reset-failed`, which is reported
     as `stopped`.
   - tmux needs a wrapper to keep exit codes.
3. **It survives the ssh session.**
   - The Orin has Linger on, and already runs `f1tenth-archive` as a user
     unit.
   - On linus the desktop session keeps the user manager alive.
   - The bot's own unit needs `loginctl enable-linger` there anyway.
4. **No new dependency.** linus has no tmux.

The cost is that there is no terminal to attach to. Output goes to
`~/.local/state/f1tenth_ctl/logs/<name>.log` (`StandardOutput=append:`). The
planner gets its stdin from a FIFO instead of a terminal.

## 8. Changed after the sim test

Recorded here so this file does not mislead. Details are in
`output/telegram/test.md` and `src/f1tenth_ctl/README.md`.

- **READY comes from a monitor unit** (`f1t-monitor`) that keeps a snapshot
  file current, not from a one-shot probe per `status`.
  - On a loaded host the one-shot probe could not match its subscriptions
    inside the 5 s budget.
  - The monitor runs only while a stack runs.
- **Stop sends SIGINT to the main process's process group**, as Ctrl-C does.
  `ros2 run` does not forward SIGINT to its child: the planner needed SIGKILL
  when only the main pid got it.
- **The sim profile does not make every node a super client.** Only the
  probe and the monitor are. On one host, ~40 super clients overloaded the
  discovery server.

