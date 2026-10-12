# Phase 5d — D2 resume notes (work in progress)

Date: 2026-10-08. Machine: `jetson-orin`. Branch `jazzy` at `c67aa76` plus
uncommitted changes (section 2). **Paused on request before D2 was built.**
Read this file, then continue at section 5.

Work order being executed: "Phase 5D — D1 review: accepted, decisions, go for
D2". The final report goes to `output/phase5d_runtime.md` (D2 + D3 #3).

## 1. The work order, condensed

**D1 accepted.** The image for D2/D3 is `f1tenth:jazzy-orin-a515f27`
(`sha256:2fef94aa0fe11127feb60c77e3f34237891a1c764bc2f6946b888b95ea436b20`).

**§1 Safety rules.** No `rm -rf` built from a variable without `${VAR:?}` and
a prefix check. Remote Thor work runs as a copied script with
`set -euo pipefail`, never as a here-doc inside `ssh '…'`. Both rules go in
`CLAUDE.md`.

**§2 Decisions:**
- Thor v5 is the reference. Record why in `phase5d_analysis.md` §7 #2.
- Image size: leave it as is. Backlog LOW.
- Ubuntu archive: not snapshotted. Backlog LOW (`snapshot.ubuntu.com` at the next rebuild). Every D3/D4 result records the image ID.
- Logger path dependency (`test_robot_logger` ×2):
  - D3 #2 counts both tests as environment-caused.
  - D3 #10 checks whether the logger **code** depends on the directory being named `f1tenth_more`.
  - Tests only: backlog LOW. The code too: report before D4, with a fix through `F1TENTH_MORE_ROOT` or a marker file.
- Installed `stack_params.yaml` llama paths: accepted. Backlog LOW (env var or launch arg).
- ZED settings: `~/jazzy_orin/zed_settings`, seeded once with a **copy** of the host's `SN36866199.conf`, mounted read-write at `/usr/local/zed/settings`. The host's 4.2.5 dir is never mounted.
- ZED `PERFORMANCE` depth mode deprecated: a Phase 7 question.
- Old images: remove them. Keep the ccache volumes and `ubuntu:24.04`. Report `docker system df` before and after.
- Uncommitted verbose logs: leave them.

**§3 D2.** As designed in `phase5d_analysis.md` §6, with these specifics.

`docker/orin/compose.yaml`:
- `runtime: nvidia`, `network_mode: host`, `ipc: host`, private PID.
- `user: fabiocar` (1004).
- `group_add`: video 44, render 104, dialout 20, zed 1005, jtop 1003. Resolve them from the host at `up` time, check them against the image's GIDs, and refuse on a mismatch.
- `cap_add: SYS_NICE`, `rtprio` and `memlock` ulimits, no cpuset.
- `/dev` bind mount, plus cgroup rules 166, 188, 189, the hidraw major (read from `/proc/devices`), 81 and 13.
- Volumes:
  - `~/jazzy_orin/archive` at `~/f1tenth_archive`;
  - `~/jazzy_orin/ros` at `~/.ros`;
  - `~/jazzy_orin/zed_settings` at `/usr/local/zed/settings`;
  - a named volume at `/var/cache/f1tenth/cuda`;
  - `/run/jtop.sock`;
  - the LLM models dir, read-only;
  - `/etc/localtime`, read-only.
- No `privileged`.

`scripts/docker_orin.sh up|down|shell|status|logs`:
- runs the guard before `up` and `shell`;
- always uses host networking;
- prints the image ID it started.

Other D2 items:
- `mission_logger_node.py:919` (`systemctl --user`): confirm it fails inside the container with a logged warning, not an exception.
- Env table: Thor native vs Orin container vs Orin Humble, from `/proc/<supervisor pid>/environ`.

D2 checks:
1. Container up/down 5×, with no leftover processes, SHM files or ports.
2. Device nodes are visible after a hot-plug: the VESC if it is connected, otherwise the ZED.
3. `check_cpu_pinning.py` shows `SYS_NICE` effective.
4. Every file in `~/jazzy_orin/*` is owned by `fabiocar`.

**If every D2 check passes, go straight on to D3 #3:**
- the launch smoke test: `step4`, 10 cycles, a 5 min `hold`, and the duplicate-process check;
- then STOP and write one report, `output/phase5d_runtime.md`.

If any D2 check fails, stop at D2. No drive commands and no actuation; if
anything is plugged in, the car stays on the stand.

## 2. Done so far (uncommitted)

| Item | State |
|---|---|
| §1 rules in `CLAUDE.md` ("Harness gotchas") | done |
| `docker/orin/build.sh`, `stage_sources.sh`: `${VAR:?}` + prefix check before `rm -rf` | done |
| Backlog L17 (CUDA static libs), L18 (Ubuntu snapshot, record image IDs), L19 (llama path selectable) | done |
| `phase5d_analysis.md` §7 #2: v5 reference and why | done |
| Old images removed (`7420a11`, `spike`, `spike-zed540`, 3 dangling), non-cache build records pruned, 3 ccache mounts kept | done |
| `docker system df` **before** the cleanup | **lost**: it was saved to a session temp dir that no longer exists. The D1 report recorded 18.4 GB reclaimable before the cleanup |
| `docker system df` after (2026-10-08) | images 46.51 GB (7 images, all others are not f1tenth), build cache 19.98 GB / 101 MB reclaimable, 1 volume (`aarch64_zed-resources`, not ours) |
| `~/jazzy_orin/` | **not created yet** |

Not committed yet: `CLAUDE.md`, `docker/orin/build.sh`,
`docker/orin/stage_sources.sh`, `output/backlog.md`,
`output/phase5d_analysis.md`, this file.

## 3. Findings so far (all measured 2026-10-08)

### F1. `cap_add: SYS_NICE` is inert for a non-root container user

`docker run --cap-add SYS_NICE --ulimit rtprio=99:99 --ulimit memlock=-1:-1`
with the image (user `fabiocar`) gives:

- `CapInh/CapPrm/CapEff/CapAmb = 0`, `CapBnd = a88425fb`. Docker only puts added capabilities in the bounding set for a non-root user; there are no ambient capabilities.
- `ulimit -r 99`, `ulimit -e 0`, `ulimit -l unlimited`.
- `os.nice(-5)` (the call `MPC_corr.py:1937` makes): **PermissionError**. `nice -n -5 true` prints "cannot set niceness" but still exits 0, which is misleading.
- `chrt -f 10 true`: works, through `RLIMIT_RTPRIO`, not through the capability.

**Proposal:** add a `nice` ulimit (soft/hard 40, i.e. nice down to −20) to
the compose file. Keep `cap_add: SYS_NICE`, as ordered, with a comment saying
it only matters for root `docker exec -u 0`. Without the ulimit, D2 check 3
fails. **Needs Andreas's OK.**

### F2. `render` GID: image 994, host 104

`getent` in the image gives video 44, dialout 20, zed 1005, jtop 1003, all
equal to the host, and **render 994**: the `udev` package created it in the
`base` stage. On the host it is 104.

- With numeric `group_add`, the process gets 104 anyway, so access to the host's `/dev` nodes works. Nothing in the image is known to depend on `render`.
- Not yet verified: that no file in the image is owned by GID 994. Check with `find / -xdev -gid 994`.
- The order says "refuse on mismatch", so a strict `up` refuses the accepted image.

**Proposal:** strict check on all five groups, with one narrow known
exception (`render` = 994 in the image, only that value). Add a backlog item
to put `groupmod -g 104 render` in the `base` stage at the next rebuild, then
remove the exception. **Needs Andreas's OK.**

### F3. The logger's archive sync fails silently in the container, with no warning

`_spawn_sync` (`mission_logger_node.py:914-923`):

- It runs `subprocess.Popen(['systemctl','--user','--no-block','start','f1tenth-archive.service'], stdout=DEVNULL, stderr=DEVNULL, start_new_session=True)`, never waits for it, and logs INFO "archive sync spawned … (detached, not awaited)".
- The image **has** `/usr/bin/systemctl` (pulled in through udev/systemd), so `Popen` succeeds. `systemctl --user` then fails with no bus (no `XDG_RUNTIME_DIR` / `DBUS_SESSION_BUS_ADDRESS`, `/run/user/1004` not mounted). That failure goes to `/dev/null`.
- Result: **no exception and no warning**, only a misleading INFO line. The child is also never reaped, so it stays a zombie until the next `Popen`.
- If `systemctl` were absent, `FileNotFoundError` would be logged as ERROR by `_finalize_run`'s per-step except. Not fatal either way.

Still to do: confirm live. Run
`systemctl --user --no-block start f1tenth-archive.service; echo $?` in the
container, then find the INFO line in a D3 step4 logger run. Proposal: backlog
LOW (skip with a warning when no user bus exists, or check the exit status
asynchronously). No code change.

### F4. Other facts for the next steps

- **Devices now:**
  - ZED 2i on USB (`2b03:f880` / `f881`), `/dev/hidraw0` (major 510, 0666), `/dev/video0-1` (81).
  - **No VESC** (no `/dev/ttyACM*`, no `/dev/sensors`). Lidar `192.168.0.10` does not answer ping.
  - So the hot-plug check uses the **ZED**, and it needs a physical unplug and replug by a person, with the container up.
- **`/proc/devices`:** input 13, video4linux 81, usb 180, usb_device 189, hidraw 510.
- **Host groups of `fabiocar`:** dialout 20, video 44, render 104, jtop 1003, zed 1005 (and docker 994).
- **Image:**
  - `/usr/local/zed` and `/usr/local/zed/settings` are `root:zed 0770` (settings is empty).
  - `taskset`, `systemctl`, `ros2` and `tf2_tools/view_frames` are present.
- **Docker** 29.6.0, compose v5.1.4. Power: MODE_50W.
- **LLM models:** `~/projects/llm/models` (4 `.gguf`, stack default `qwen2.5-3b-instruct-q5_k_m.gguf`).
- **D3 #3 input bag:**
  - `output/phase5/inputs/bringup_input` is **not on the Orin** (gitignored). It is on the Thor at `~/dev_ws/f1tenth_more/output/phase5/inputs/bringup_input/`: `bringup_input_0.db3`, 16,498,688 B, sha256 `1678abb45f252b11e651311edfd18b9f4d113d64f88667290d7726dd944179a6`; `metadata.yaml` sha256 `80da7855…f095744`.
  - Copy it with `scp` and check both hashes. The exact `filter_bag_for_layer.py` command that made it is not recorded anywhere, so copying it is better than rebuilding it.
- **Thor:** reachable as `andre@jetson-thor`. **No `component_supervisor_node` running** (checked 12:55 by a `/proc` scan; `pgrep -f` over ssh matches its own command line).

## 4. Design decided while reading (not yet written)

- **`compose.yaml` has no `build:` section.** The image is built only by `build.sh` (host network, staged named contexts), so compose sets `pull_policy: never`. A comment points to the CLAUDE.md rule: if a `build:` section is ever added, it must say `network: host`.
- **Image selection.** `docker_orin.sh` exports `F1TENTH_IMAGE` (default `f1tenth:jazzy-orin-a515f27`); compose uses `${F1TENTH_IMAGE:?}`. `up` prints the image ID and container ID.
- **Main process.** `command: sleep infinity` with `init: true`. The container is an environment; the stack is started with `docker exec` (harness or `shell`).
  - With `init: true`, orphans are reparented to tini (PID 1 in the container). The "no process whose parent is gone" check therefore looks for stack processes with PPID 1.
- **Values `docker_orin.sh` resolves.** The host GIDs (`GID_VIDEO`, `GID_RENDER`, `GID_DIALOUT`, `GID_ZED`, `GID_JTOP`) and `HIDRAW_MAJOR` are resolved at `up` and exported. Compose uses `${…:?}`, so a bare `docker compose up` fails instead of starting wrong.
- **`docker_orin.sh down` stops ROS gracefully first.**
  - `docker stop` only signals PID 1, and exec'd processes are SIGKILLed when the namespace's init dies, which leaves stale Fast DDS SHM files.
  - So `down` first runs `docker exec <ctr> kill -INT -1` (every container process except PID 1), waits up to 20 s, sends SIGTERM, then runs `docker compose down`. Afterwards it reports leftover processes, new `/dev/shm` files, and UDP 11811 / listening ports.
- **`docker_orin.sh shell [cmd…]`.** It runs the guard with `GUARD_OWN_PREFIX=f1tenth_jazzy_orin`, so the container's own processes and its DS on 11811 are not a conflict. Then:
  - with no command: `docker exec -it <ctr> bash` (`/etc/bash.bashrc` sources `jazzy.sh`);
  - with a command: `docker exec <ctr> /opt/f1tenth_ws/entrypoint.sh cmd…`. A non-interactive `docker exec` does not get the environment otherwise.
- **`scripts/orin_stack_guard.sh [--for jazzy|humble]`** (analysis §6.3; the `lib_guard.sh` header already announces it). It wraps `orin_guard` and adds:
  - check 3: a `/dev/shm/*fastrtps*` file mapped by a live process, scanned in `/proc/*/maps`;
  - check 4: `--for humble` refuses while `f1tenth_jazzy_orin` runs.
- **`docker/orin/compose.harness.yaml`** (for D3/D4 only, used by `up --harness`):
  - mounts the worktree's `scripts/` read-only and `output/` read-write, at the same paths;
  - this works because `phase5_bringup.sh` computes `REPO` from its own path;
  - the stack itself still comes only from the image's `/opt/f1tenth_ws/install`.
- **Volumes as ordered.** Not added: a `zed_resources` volume (the analysis sketch had one; the order does not). The stack's depth mode needs no AI models. Note this in the report.
- **Env table method, the same on all three sides:**
  - a supervisor with **every component emptied**: `phase5_components.py OUT --empty <all 16/17 names>`;
  - launched as `supervisor_bringup.launch.py components_config:=OUT enable_intelligence:=false log_dir:=<tmp> discovery_server_port:=11897`, so the production DS on 11811 is never touched. Both the Jazzy and the Humble launch files accept these arguments;
  - started from the side's real shell: `bash -ic` (Thor: andre's `~/.bashrc`; Orin Humble: fabiocar's `~/.bashrc`, which has the global `ROS_SUPER_CLIENT=TRUE` and the PyPI-cuDSS `LD_LIBRARY_PATH`), and the container's interactive shell;
  - dump every variable name, and values only for the stack-relevant keys (script below). Then SIGINT the launch and kill the DS on 11897 by PID;
  - note in the table that `ROS_DISCOVERY_SERVER` shows `:11897`;
  - on the Thor, copy the script and `phase5_components.py` to `~/p5d_env_capture/`, run them, fetch the output, then remove that dir under the prefix rule;
  - Orin Humble only with the container down (`orin_stack_guard.sh --for humble`).

Read-only Thor script already used (re-create it, since the temp copy is gone):

```bash
#!/usr/bin/env bash
# Read-only: list component_supervisor_node processes (scan /proc, not
# pgrep -f) and print every variable NAME, values only for stack-relevant keys.
set -euo pipefail
KEEP='^(ROS_|RMW_|FASTRTPS_|FASTDDS_|F1TENTH_|COMPONENT_SUPERVISOR|MISSION_LOGGER|ZED_|CUDA_|NVIDIA_|LD_LIBRARY_PATH|PYTHONPATH|PYTHONNOUSERSITE|AMENT_PREFIX_PATH|COLCON_PREFIX_PATH|CMAKE_PREFIX_PATH|PATH|HOME|USER|LANG|LC_ALL|TZ|SHELL|ROS_LOG_DIR)='
found=0
for d in /proc/[0-9]*; do
  pid=${d#/proc/}; [ "$pid" = "$$" ] && continue
  cmd=$( { tr '\0' ' ' < "$d/cmdline"; } 2>/dev/null) || continue
  case "$cmd" in *component_supervisor_node*--ros-args*|*/component_supervisor_node\ *|*/component_supervisor_node) ;; *) continue ;; esac
  case "$cmd" in *thor_supervisor_env*) continue ;; esac
  found=1
  echo "=== pid $pid  started $(ps -o lstart= -p "$pid")  user $(ps -o user= -p "$pid")"
  echo "cmd: ${cmd:0:300}"
  echo "--- names (all):"; { tr '\0' '\n' < "$d/environ"; } 2>/dev/null | cut -d= -f1 | sort | tr '\n' ' '; echo
  echo "--- values (stack-relevant):"; { tr '\0' '\n' < "$d/environ"; } 2>/dev/null | grep -E "$KEEP" | sort || true
done
[ "$found" = 1 ] || echo "no component_supervisor_node running"
echo "--- host: $(hostname) $(uptime)"
```

Run with `ssh andre@jetson-thor bash -s < script.sh`.

## 5. Next steps, in order

0. **Get Andreas's answers on F1** (the `nice` ulimit) **and F2** (the `render` exception).
1. **Host dirs:**
   ```bash
   mkdir -p ~/jazzy_orin/{archive,ros,zed_settings}
   cp -p /usr/local/zed/settings/SN36866199.conf ~/jazzy_orin/zed_settings/
   ```
   The copy is made once and never overwritten.
2. **Write the D2 files** (section 4):
   - `docker/orin/compose.yaml`, `docker/orin/compose.harness.yaml`;
   - `scripts/docker_orin.sh`, `scripts/orin_stack_guard.sh`;
   - the check 3 / check 4 additions in `lib_guard.sh`.
3. **Smoke test:** `up`, then `status`, `shell id`, `shell` with `-it` (`/dev:/dev` can interfere with docker exec ptys, so test it), `logs`, then `down`. Check that `HostConfig.NetworkMode` is `host`.
4. **D2 check 1, up/down 5×.** Each cycle:
   - in the container: a DS plus a `ros2 topic pub` / `echo` pair, so that SHM files and ports exist;
   - `down`;
   - compare the host's `/dev/shm`, `ss -Hulpn` / `-Htlpn` and `/proc` before and after.
   - Logs under `output/phase5d_logs/d2/`.
5. **D2 check 2, hot-plug.** With the container up, ask the user to unplug and replug the ZED. Then, inside:
   - list the nodes;
   - `open()` `/dev/hidraw*`, `/dev/video*` and `/dev/bus/usb/001/*`;
   - `lsusb -d 2b03:`;
   - a short ZED grab with `docker/orin/spike/gates/zed_grab.cpp` / `g3_zed.sh`. D1 grabbed 300 frames at 59.7 Hz.
6. **D2 check 3:**
   - in the container: `os.nice(-5)`, `chrt -f 10`, `ulimit -e -r`, and the nice value read back from `/proc/<pid>/stat`;
   - `check_cpu_pinning.py` while a short `phase5_bringup.sh hold` (`HOLD_SEC≈90`) runs in the container, after `up --harness` and the bag copy.
7. **D2 check 4:** `find ~/jazzy_orin ! -user fabiocar` must be empty, plus a write test from inside the container to each read-write mount.
8. **F3 live confirmation and the env table** (section 4).
9. **If all D2 checks pass, D3 #3 in the container** (about 30 min):
   - `phase5_bringup.sh step4`, `cycles OUT 10`, `hold` (`HOLD_SEC=300`);
   - pass criteria in `phase5d_analysis.md` §7 #3: 32 nodes; rates within ±5 % of the Thor (`/odometry/filtered` 48.1, `/ekf_global` 48.0, `/tf` 105.9, `/costmap/boundaries` 20.1, `/drive` 10.0 Hz); 0 crashes; 0 "Matching unexisting participant"; the logger records as a super client;
   - after the cycles and after the hold: one process per `__node:=` name and no stack process with PPID 1;
   - every result records the image ID.
10. **Write `output/phase5d_runtime.md`** (D2 + D3 #3), commit, STOP.

Nothing ROS may run on the host while the container runs, or the other way
round: the guard enforces this. Kill everything you start.
