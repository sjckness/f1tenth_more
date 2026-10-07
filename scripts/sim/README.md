# Two-machine sim: one-command launchers

Gazebo on the **sim host** (linus) ↔ Jazzy stack on the **stack host** (the Thor),
direct Ethernet, Fast DDS Discovery Server on the Thor. These scripts set the ROS
network env in one place and launch each side, so you don't type exports or long
`ros2 launch` lines.

## Normal bringup

```bash
# on the Thor (starts the Discovery Server + the stack):
scripts/sim/stack.sh

# on linus (connects to the Thor's Discovery Server, starts Gazebo):
scripts/sim/sim.sh
```

Start the Thor first (it hosts the Discovery Server); linus's sim will connect
when it comes up. Both are headless/standard by default. Ctrl-C to stop.

## Options (both scripts)

- `--clean`   wipe this machine first (`kill_ros2.py -y`: processes **and** the
  Fast DDS `/dev/shm` + supervisor pgid state that otherwise breaks the next run)
- `--dry-run` print the resolved env + command, launch nothing
- any `ros2 launch` arg is passed through and overrides a default, e.g.
  `scripts/sim/sim.sh gui:=true yaw:=0`,
  `scripts/sim/stack.sh health_watchdog:=enforce`

## Clear a machine

```bash
scripts/sim/clean.sh          # kill all ROS + purge runtime state (no prompt)
scripts/sim/clean.sh -n       # dry run: just list what would be killed
```

## Network config (defaults, override by exporting before launch)

Defined once in `sim_net.sh`, then it sources `scripts/env/jazzy.sh`:

| var | default | meaning |
|---|---|---|
| `F1TENTH_SIM_DISCOVERY_IP`   | `10.42.0.2` | LAN IP of the Discovery Server host (the Thor) |
| `F1TENTH_SIM_DISCOVERY_PORT` | `11811`     | Discovery Server port |
| `ROS_DOMAIN_ID`              | `42`        | sim domain |

Example: `F1TENTH_SIM_DISCOVERY_IP=10.42.0.9 scripts/sim/stack.sh`.

## Introspecting the running graph

A plain `ros2 topic list` as a Discovery-Server client shows only
`/parameter_events` + `/rosout` by design. These scripts source
`scripts/env/jazzy.sh`, which gives you the **`ros2cli`** helper (super-client):

```bash
source scripts/env/jazzy.sh      # in any shell on either host (sets domain/DS too if you export them first)
ros2cli topic list
ros2cli node list
```
