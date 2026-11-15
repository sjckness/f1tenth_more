# Orin HIL setup: state and next steps (2026-11-11)

Goal: the Orin (`jetson-orin`, user `fabiocar`) runs the car's stack, unchanged, inside its
Jazzy Docker container against Isaac Sim on linus, over a direct Ethernet cable.
Runbook: `output/hil/setup.md` on branch `hil-hydra-integration` (steps 5-9 are the Orin's).
Hard limits: don't start the sim or the stack, don't touch the car's own setup, leave the Orin
in native mode, don't push, and don't build images.

| | |
|---|---|
| linus | 10.42.60.1/24 (direct cable) |
| Orin | 10.42.60.2/24 on `eno1`; also reachable on work Wi-Fi 172.26.12.61 and Tailscale 100.96.201.7 |
| DDS | `ROS_DOMAIN_ID=44`, `rmw_fastrtps_cpp`, Discovery Server on the Orin at **10.42.60.2:11812** (11811 belongs to the car) |
| Stack | `ros2 launch f1tenth_bringup supervisor_bringup.launch.py sim:=isaac use_sim_time:=true discovery_server_address:=10.42.60.2 discovery_server_port:=11812 fastdds_profile:=/tmp/f1tenth_hil_profile.xml` |

## State

| Task | State |
|---|---|
| 1 Survey | Done, nothing changed |
| 2 Link | **Blocked on linus**: the Orin side is configured, but 10.42.60.1 doesn't answer ARP |
| 3 HIL branch | Not started. GitHub is reachable; `origin/hil-hydra-integration` is at `f686d815` |
| 4 Config | Not started |
| 5 Container | Not started. **Needs a decision** (container name, see below) |
| 6 Verify | Not started |
| 7 Report | Not started |

### What changed on the Orin

These are the only changes. All are runtime only and **disappear on reboot**; re-run them after one.

```
sudo ip addr add 10.42.60.2/24 dev eno1 ; sudo ip link set eno1 up
sudo sysctl -w net.core.rmem_max=16777216 net.core.wmem_max=16777216
```

- `eno1` now has `192.168.0.15/24` (lidar profile `hokuyo-static`, untouched) **and**
  `10.42.60.2/24`.
- Route: `10.42.60.0/24 dev eno1 scope link src 10.42.60.2`, with no gateway. Correct.
- `rmem_max` and `wmem_max` are 16777216. Correct.

### The Task 2 blocker

- `eno1` link: up at 10 Gb/s full duplex, so the direct cable is in it (a Hokuyo runs at 100 Mb/s).
- `ping 10.42.60.1` gives `Destination Host Unreachable`, and the ARP entry stays `INCOMPLETE` /
  `FAILED`. The Orin sends ARP requests and no reply comes back.
- The peer did send some frames earlier (eno1 rx went from 10 to 192 packets), then went quiet.
- Most likely cause: the NIC on linus that is wired to the Orin doesn't have 10.42.60.1/24, or
  isn't up. NetworkManager's "Shared to other computers" mode would give it 10.42.0.1 instead.
- The Orin's `eno1` MAC is `3c:6d:66:86:42:e8`.

### Survey snapshot

- **Mode:** `stack_mode status` reports **native**.
  - `f1tenth-gateway.service` (user unit, from `f1tenth_more_two_llm`) is active and holds TCP 8770.
  - `f1tenth-car-container.service` is inactive.
  - The `~/.config/f1tenth/car_mode` file says `container`; that's left over from 2026-11-10. Not touched.
- **Discovery Server:** none running. UDP 11811 and 11812 are free.
- **Disk:** `/` has 15 GB free (73% used); `/scratch` has 483 GB free.
- **Car image pin:** `~/.config/f1tenth/car_image` holds `f1tenth:jazzy-orin-ecb457b-norust`
  (revision `ecb457b5`, pinned 2026-11-10 20:10). The image is present.
- **Containers:** one, `f1tenth_car` (image ecb457b, role `car`, Docker Compose project
  `f1tenth_car`). It exited with 255 at 2026-11-10 20:18; the Orin rebooted at 08:41 today.
- **Checkouts:** `f1tenth_more` worktrees under `~/dev_ws`.
  - `f1tenth_car` is on `door-goal-debug` 262199f7 with 17 tracked files modified. It is the
    checkout the car's image and container come from, and it has `scripts/docker_orin.sh` and
    `docker/orin/compose.yaml`.
  - The other checkouts are not involved.

### Differences from the work order

1. **There is no image built from `main-jazzy-rust` alone.** Every `f1tenth:jazzy-orin-*-norust`
   tag is a `door-goal-debug` commit. `door-goal-debug` contains `main-jazzy-rust` (`425305cb`,
   current on origin). The usable image is the pinned `ecb457b`.
2. **The source tree is not bind-mounted** (compose mounts only state, caches and read-only
   inputs). The uncommitted edits in `f1tenth_car` therefore don't reach the container. The image's
   `/opt/f1tenth_ws` is baked in.
3. **The container name collides with the car's container.** `compose.yaml` has
   `container_name: f1tenth_car`, and `docker_orin.sh` has `CTR=f1tenth_car`. With
   `orin.container: f1tenth_car`, `docker_orin.sh up` would recreate the **car's own** container.
   It would also bind the car's state into the HIL run: `.ros`, `ctl`, `gateway` state,
   `mission_reports`, the archive, and the `archive_trigger` that starts the rsync to linus. So
   HIL runs would mix into the car's archive. It would also make `stack_mode` read the Orin as
   running a container while the native gateway is up.

## Next steps

### A. On linus: make 10.42.60.1 answer

```
ip -br a                              # find the NIC wired to the Orin (ethtool <if>: 10000Mb/s, link yes)
sudo ip addr add 10.42.60.1/24 dev <if> ; sudo ip link set <if> up    # if it lacks the address
ping -c 3 10.42.60.2
sudo tcpdump -eni <if> arp            # while the Orin pings: expect "who-has 10.42.60.1 tell 10.42.60.2"
```

If `tcpdump` shows no ARP from `3c:6d:66:86:42:e8`, the cable goes somewhere else.

### B. On the Orin: finish Task 2

```
ping -c 3 10.42.60.1                  # must answer
ip route get 10.42.60.1               # must be "dev eno1 src 10.42.60.2", no "via"
sysctl net.core.rmem_max              # 16777216
```

### C. Task 3: HIL checkout (proposed path `~/dev_ws/f1tenth_hil`)

```
git -C ~/dev_ws/f1tenth_more fetch origin
git -C ~/dev_ws/f1tenth_more worktree add ~/dev_ws/f1tenth_hil -b hil-hydra-integration origin/hil-hydra-integration
git -C ~/dev_ws/f1tenth_hil submodule update --init src/husarion_gz_worlds
```

Don't init `third_party/hydra_ros`, and don't use `--packages-up-to`. Then read
`~/dev_ws/f1tenth_hil/output/hil/setup.md`.

### D. Task 4: config, from a clean shell

```
env -i HOME=$HOME USER=$USER PATH=/usr/bin:/bin bash --noprofile --norc
export F1TENTH_WS=$HOME/dev_ws/f1tenth_hil
source $F1TENTH_WS/scripts/env/jazzy.sh
cd $F1TENTH_WS
cp sim/isaac/config/hil.yaml sim/isaac/config/hil.local.yaml    # gitignored
#   orin.ssh, orin.repo: ~/dev_ws/f1tenth_hil, orin.container: (decision below),
#   orin.image: f1tenth:jazzy-orin-ecb457b-norust (if the fallback is used)
scripts/sim/hil_orin env                                        # ROS_DOMAIN_ID must be 44
```

### E. Task 5: decision needed before any container starts

- **Recommended:** a separate HIL container, for example `f1tenth_hil`, through the `docker run`
  that `hil_orin up` prints, with `--network host`, `--ipc host` and `F1T_AUTOSTART=none`. Before
  running it, check that its mounts don't include the car's state or archive dirs. This keeps
  `f1tenth_car` and its state untouched. It depends on `hil_orin` / `hil.yaml` accepting another
  container name; check that in setup.md.
- **Alternative:** reuse `f1tenth_car` via `docker_orin.sh up` from `~/dev_ws/f1tenth_car`, as the
  work order says. This recreates the car's container, and HIL runs land in the car's state and
  archive.
- If the image were missing: stop and ask; don't build it. It isn't missing; `ecb457b` is present.

### F. Task 6: verify, without launching the stack

```
scripts/sim/hil_orin preflight
docker exec <ctr> bash -lc 'ros2 pkg prefix f1tenth_bringup' </dev/null
docker exec <ctr> bash -lc 'ros2 launch f1tenth_bringup supervisor_bringup.launch.py --show-args' </dev/null
#   must list: sim, use_sim_time, discovery_server_address, discovery_server_port, fastdds_profile
```

Run `ldd -r` on binaries only if something fails to start.

### G. Then (Task 7 and the run)

linus starts Isaac first; the first `/clock` takes 20-70 s. Then, on the Orin from the HIL
checkout: `scripts/sim/hil_orin stack`, then `scripts/sim/hil_orin check`.

## Rules to keep in mind

- **Host networking only:** Docker on the Orin has no bridge network. Always use
  `--network host` / `network_mode: host`; a container without it comes up silently with only
  `lo`. `hil_orin` refuses such a container; don't work around that.
- **Mode switches:** only through `~/.local/bin/stack_mode native|container`, and stay native.
- **Remote work:** through a script with `set -euo pipefail`. In `ssh ... bash -s < script`, every
  `docker exec` / `docker_orin exec` needs `</dev/null`.
- **Deletes and kills:** never `rm -rf` a path built from a variable without `${VAR:?}` and a
  prefix check. Never `pkill -f` / `pgrep -f`; kill by PID.
- **Git:** no `git push`.
