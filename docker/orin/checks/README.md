# Orin container checks: D2 runtime (plan P8) and D3 parity (plan P9)

Every check of the plan's P8 (D2 runtime, no actuation) and P9 (D3 parity vs
native Humble on the same Orin) is mapped below to the script that runs it, its
pass criterion and where it runs. The plan is
`~/.claude/plans/fuzzy-enchanting-teapot.md`; the shared contracts are in
`output/orin_container/design.md`.

Two directories hold the tools:

- `docker/orin/checks/` (this one): host-side drivers of the container
  checks, the spike gates of the jazzy line made into checks (`zed_grab.cpp`,
  `g2_trt.py`, `yolo_run.py`, `yolo_compare.py`, `dds_probe.py`) and the
  comparison helpers. `lib_checks.sh` is shared.
- `scripts/jazzy_parity/`: the D3 harness ported from the jazzy line
  (`phase5_bringup.sh`, `replay_localization.sh`, the probes, `parity_gates.py`
  for the verdicts). `lib_parity.sh` is shared by both directories.

Every script answers `--help` with what it does, where it runs and what it
compares.

## Where things run

| Tag | Meaning |
|---|---|
| **host** | the Orin host shell (user fabiocar), in `~/dev_ws/f1tenth_car` (the P7.2 worktree) |
| **container** | inside `f1tenth_car`, reached from the host with `docker exec` (the car role: what the container runs on the car) |
| **harness** | inside `f1tenth_car` started with `docker_orin.sh up --harness` (compose.harness.yaml: no stack running; of the checkout only `scripts/`, `tools/` and `docker/orin/checks/` are mounted, read-only, and `output/` read-write, each at its host path; compose.yaml's state mounts as in the car role) |
| **both** | the same command on the native side (host, native Humble env) and on the container side (harness), through `d3_side.sh native|container OUT -- CMD`, which records the image ID, nvpmodel and clocks first and refuses to run the two sides together |
| **linus** | the dev PC, over ssh / Tailscale |

Running a harness step on both sides, the pattern every P9 row uses:

```bash
cd ~/dev_ws/f1tenth_car                     # the checkout (F1T_REPO of the harness)
R=$PWD/output/orin_container/d3/$(date +%Y%m%dT%H%M)   # results root: under output/
C=docker/orin/checks; P=scripts/jazzy_parity
# The stand (ESC off or the VESC unplugged, see FEED=live below), or the bag fallback:
export FEED=live LIVE_VESC_OK=1 HOLD_SEC=300
#   export FEED=bag BAG=$R/inputs/bringup_input      # inputs under $R/inputs too
# native: the car in native mode (stack_mode.sh native), the native gateway stopped
$C/d3_side.sh native    $R/native -- $P/phase5_bringup.sh hold $R/native/hold
# container: stack_mode.sh container, then docker_orin.sh up --harness
$C/d3_side.sh container $R/cont   -- $P/phase5_bringup.sh hold $R/cont/hold
```

`d3_side.sh` sources the side's f1tenth_ctl env (native: the native
`ctl.env`; container: the container's own `F1T_CTL_ENV`, i.e. W1's
`ctl_container.env`) and exports it as `CTL_ENV`, so `phase5_bringup.sh`
starts the stack exactly as each side's f1tenth_ctl does (the car profile's
launch arguments included). It passes the harness knobs that are set (`FEED`,
`BAG`, `HOLD_SEC`, `STACK_ARGS`, `LIVE_MISSION`, `LIVE_VESC_OK`, the timing
and replay knobs; `d3_side.sh --help` lists them; `D3_PASS` adds more) into
the container, so both sides run with the same ones; the linus-only knobs are
never passed.

The harness container sees only `output/` writable (and `scripts/`, `tools/`,
`docker/orin/checks/` read-only) of the host. So **every container-side
result and every input lives under `$R` inside `output/`**, the same path on
both sides: `d3_side.sh container` refuses an `OUT_DIR`, `BAG` or `MPC_PARAMS`
outside `output/` (unless `D3_PATH_MAP=HOST_PREFIX:CONTAINER_PREFIX` says the
caller mapped the paths), and warns about other host paths among the
arguments. Car bags from the archive are copied into `$R/inputs/` first (the
harness mounts its own archive directory, not the native `~/f1tenth_archive`).
Scratch that is large and not a result (the P9.1 build) goes into the
container's own `/tmp`.

## Ground rules the scripts enforce

- **Native and container never together.** `phase5_bringup.sh` refuses on a host
  where `f1tenth_car` runs, and in a container where the Discovery Server port is
  held by a process it cannot see; `d3_side.sh` checks the side first.
- **Nothing ROS on the side.** Every stack harness refuses to start while any ROS
  stack process runs (precheck), and audits / stops only what it started, by PID
  or process group (never `pkill -f`).
- **Nothing is deleted.** Every run writes into a NEW (or empty) directory; a
  reused one is refused. No `rm -rf` exists in either directory.
- **No live drive without an e-stop.** With `FEED=live` the stack runs on the
  harness domain, where no e-stop of the car's graph (ctl, telegram, PWA)
  reaches it: every `FEED=live` run needs the ESC off or the VESC unplugged.
  `phase5_bringup.sh` refuses `FEED=live` while `/dev/sensors/vesc` or a
  `/dev/ttyACM*` exists unless `LIVE_VESC_OK=1` says the ESC is off.
- **No archive sync from a check.** A harness mission makes the stack's logger
  request the real archive sync (native: `systemctl --user start
  f1tenth-archive.service`; container: the trigger file the host's .path unit
  turns into an rsync to linus). `phase5_bringup.sh` and `logger_check.sh`
  redirect both into the run (`F1TENTH_ARCHIVE_TRIGGER=<run>/archive_trigger/request`
  after the ctl env, a recording `systemctl` shim first in PATH);
  `parity_gates.py smoke` prints what was requested.
- **Isolated DDS** for every tool that does not test the production setup:

| Script | ROS_DOMAIN_ID | Discovery |
|---|---|---|
| `phase5_bringup.sh` | 85 (`HARNESS_DOMAIN_ID`) | the stack's own server, 127.0.0.1:11811 (`DS_PORT`) |
| `ds_probe.sh` | 87 | its own server on the given port |
| `replay_localization.sh` | 77 (argument) | none, localhost only |
| `logger_check.sh` | 79 | none, localhost only |
| `diag_start_check.sh` | 80 | none, localhost only |
| `lease_probe.py` | 94 | its own server, 11911 |
| `colcon_test_ids.sh` | 96 | none, localhost only |
| `yolo_engine_check.sh` (detection) | 98 | none, localhost only |

- **linus-only knobs** (never on the Orin): `CPU_REMAP=1` (the launch files pin
  to the Orin's cores 8-11; a `taskset` shim maps them onto an 8-core host) and
  `PARITY_IGNORE=<regex>` (other worktrees' isolated runs that the precheck and the
  process audits should not count).

## P8: D2 runtime checks (no actuation: ESC off / VESC unplugged)

Prerequisite (W2, not a check): `docker/orin/host/install.sh`, then
`stack_mode.sh container --archive separate`. Results go to
`output/orin_container/d2_runtime.md`.

| # | Plan check | Script (where) | Pass criterion |
|---|---|---|---|
| 8.1 | up/down x5 without leftovers | `updown_cycles.sh OUT 5` (host); calls `host_leftovers.sh` after every down | 5 ups reach `healthy`; 5 downs exit 0; after each: no `f1tenth_car`, no ROS stack process on the host, UDP 11811 and TCP 8770/8083/8765 free, no Fast DDS file in /dev/shm beyond the baseline taken before the first up |
| 8.2 | devices: VESC `/dev/ttyACM0` + `/dev/sensors/vesc`, joystick, ZED hidraw/video/usb | `devices_check.sh OUT [--open]` (host -> container); `--open` with the stack DOWN | every device node the host has is present in the container and read+writable by its user; `--open`: each serial port opens |
| 8.2 | lidar /scan rate | `devices_check.sh OUT --scan` (host -> container, stack UP) | `/scan` > 30 Hz in the container (`ros2 topic hz` as a super client) |
| 8.3 | [U-car] ZED unplug / replug with the container up | `devices_check.sh` before and after the replug; then `zed_grab --frames 30` in the container | the ZED's hidraw / video / usb nodes reappear in the container rw without a restart; `zed_grab` opens the camera (exit 0) |
| 8.4 | `os.nice(-5)` / `chrt` effective, CPU pinning | `sched_check.sh OUT [--pinning]` (host -> container; `--pinning` with the stack up) | `os.nice(-5)` returns -5; `chrt -f 10 true` and `taskset -c 10,11 true` exit 0; `scripts/check_cpu_pinning.py` (run on the host, it reads /proc) PASS |
| 8.5 | ownership of all state + `docker diff` after a 10 min run | `state_audit.sh OUT` (host, the car role), after >= 10 min up | every file under `/scratch/fabiocar/f1tenth_container` owned by uid 1004, and inside the container the named volumes (`/var/cache/f1tenth/{cuda,trt}`, `/usr/local/zed/resources`) and the mission dirs at compose.yaml's mount targets (`src/f1tenth_control/corridors_jsons`, `src/f1tenth_behavior/mission_reports`, `mission_videos`, `install/share/f1tenth_behavior/missions/llm_generated`; a missing one fails); `docker_diff_check.py`: every `docker diff` entry matches `docker_diff_allow.txt`, a changed parent only with an allowed entry beneath it. Bind mounts and volumes never show in the diff, so an entry below a mount point means that mount is missing (review the list on the first image; a path holding state is a missing mount, not an allow entry) |
| 8.6 | ctl via the wrapper from linus over ssh (start, stop, status, estop, killall, send, missions; JSON + exit codes = native; a killed node reports crashed) | `ctl_contract.sh OUT --kill '<kill cmd {pid}>' -- ssh fabiocar@orin ~/.local/bin/f1tenth_ctl_car` (linus), once native and once in container mode; then `ctl_contract_compare.py NATIVE CONTAINER` | the compare passes: same exit codes, same JSON key structure (backend keys such as unit/pid/since excepted), same ok / ready / e-stop / stack state per step; after the kill `status` says `crashed` on both |
| 8.7 | gateway over Tailscale: token, websocket, runs with image provenance | `gateway_check.py https://<orin>.<tailnet>.ts.net --token-file <env> --expect-commit <image commit>` (linus), native and container | auth refused without a token (401), status 200, WebSocket `hello` with the token, close 4401 with a wrong one, `/api/runs` a list, the newest finalized run's report names the image commit. At most two runs a minute (each spends two refused attempts of the 5/min budget) |
| 8.7 | a gateway crash leaves the stack running | `gateway_crash.sh OUT` (host, stack started) | after SIGKILL of the gateway: the stack's unit still `running`, the gateway listening on 8770 again within 30 s from a new process |
| 8.8 | archive trigger -> host rsync | `archive_trigger_check.sh OUT` (host) | the trigger file written from inside the container (at `$F1TENTH_ARCHIVE_TRIGGER`) starts `f1tenth-archive.service` within 60 s, which ends `Result=success` |
| 8.9 | jtop (M3) | `docker exec -i f1tenth_car python3 - < jtop_check.py` (container); `diag_start_check.sh` (harness) | jtop connects through `/run/jtop.sock` and reads a GPU load; `diag_start_check.sh`'s system_observer line says `jtop=yes` with non-zero GPU fields |
| 8.10 | `docker_orin.sh engine` builds the TRT engine, `yolo_backend:=engine` loads it, a missing engine falls back to `.pt` | `yolo_engine_check.sh OUT` (host -> container, stack DOWN) | `docker_orin.sh engine` exit 0; `g2_trt.py --engine` PASS (toy engines run, the engine deserializes); detection logs `yolo_backend=engine: running "<engine>"`; with a missing engine path it logs `engine "..." not used` |
| 8.11 | ZED SDK 5.4 first start (PERFORMANCE depth, resource downloads) | `build_zed_grab.sh /tmp/zed_grab` then `/tmp/zed_grab --frames 300 --depth-mode PERFORMANCE --json OUT.json` (container) | `open SUCCESS`, all frames grabbed, IMU read (exit 0); `open_s` of the first start recorded (it includes the SDK's resource download) |
| 8.12 | environment table vs native | `env_capture.sh native.env native` (native stack up) and `env_capture.sh cont.env container` (container stack up), both on the host; `env_compare.py native.env cont.env --markdown table.md --strict` | the table is produced; no marked variable `DEVIATES` from design.md / analysis section 6.5 (discovery server, domain, RMW, super client, the 5 s lease profile, the ctl backend) |
| 8.13 | healthcheck | `updown_cycles.sh` (each up) | `docker inspect --type container` health `healthy` and the image's `/opt/f1tenth_ws/docker/orin/container/healthcheck.sh` (the Dockerfile HEALTHCHECK's path; `F1T_HEALTHCHECK`) exits 0 in the container |
| 8.14 | native <-> container x3 with the guard refusing the wrong direction | `mode_switch_cycles.sh OUT 3` (host) | each `stack_mode.sh` switch exits 0 in <= 60 s; `orin_stack_guard.sh --for humble` refuses while `f1tenth_car` runs; `f1tenth_car` gone after `native` (the `--for jazzy` exit code is recorded) |

## P9: D3 parity vs native Humble (car on the stand, no drive commands)

Native and container alternate, never together. Every result records image ID,
nvpmodel and the clocks: `d3_side.sh` writes `context_host.json` (host view:
the running image's ID, `nvpmodel -q`, CPU/GPU min/max/current clocks,
`jetson_clocks_likely`), every harness run writes its own `context.json` (the
image's `SOURCES` commit inside the container). Results go to
`output/orin_container/d3_parity.md`.

`FEED=live` (the real drivers) is the stand configuration; `FEED=bag` (drivers
emptied, a re-stamped bag) is the fallback when a sensor is unavailable and the
linus dry-smoke mode. The bag input is built once with
`filter_bag_for_layer.py` (example in "Inputs" below), under `$R/inputs/`.
**Every `FEED=live` run needs the ESC off or the VESC unplugged**: the harness
stack runs on its own domain (85), so the ctl / telegram / PWA e-stop (domain
0, native gateway stopped) cannot reach it. `phase5_bringup.sh` refuses
`FEED=live` while `/dev/sensors/vesc` or a `/dev/ttyACM*` exists unless
`LIVE_VESC_OK=1` (the operator's statement that the ESC is off).

| # | Plan check | Script (where) | Pass criterion |
|---|---|---|---|
| 9.1 | colcon test in the harness container: failing IDs = linus at the same commit, aarch64 cases documented | `colcon_test_ids.sh /tmp/d3_colcon --copy-results $R/cont/colcon` (harness: WORK in the container's own `/tmp`, the results copied to `output/`; copies `/opt/f1tenth_ws/src`, builds and tests it there, ~1 h); the reference: `colcon_test_ids.sh WORK --src <checkout of the same commit>/src` on linus. No Rust toolchain on a side (the script refuses without `cargo`): `--skip-cargo` on BOTH sides. Then `parity_gates.py tests LINUS_IDS CONTAINER_IDS --allow aarch64_allow.txt` | the same failing / erroring / skipped test IDs; every difference listed (with its reason) in the allow file |
| 9.2 | launch smoke x10 | `phase5_bringup.sh cycles OUT 10` (both) | `parity_gates.py smoke OUT/cycle_*`: per cycle every process audit clean (no duplicate node process, no orphan: M23), nothing left after shutdown, launch exit 0, 0 "Matching unexisting participant", 0 watchdog respawns; the Discovery Server outliving the launch and a stale (dead-pid) lock are reported, not gated |
| 9.2 | 5 min hold | `phase5_bringup.sh hold OUT` (both; `HOLD_SEC=300`) | `parity_gates.py smoke OUT`: audits clean at settle, mid-way and at the end; nothing left after shutdown |
| 9.2 | logger recording | `phase5_bringup.sh logger OUT` (both; with FEED=live only `LIVE_MISSION=1`, ESC off / wheels up: a 12 s zero-speed hold mission) and `logger_check.sh $R/<side>/logger_check $R/inputs/<car bag>` (both) | smoke gate: the mission COMPLETEs and a complete logger run with messages exists (the archive sync it requests is captured in the run and printed, never run); `logger_check.sh` RESULT PASS on both sides, and `compare_extracts.py` of the two extracts IDENTICAL CONTENT when both played the same window |
| 9.3 | topic rates (`/scan`, camera, `/odometry/filtered`, `/tf`, costmap boundaries, `/drive`) within +-5 % of native | the `hold` runs (both; `RATE_S` window after `WARMUP_S`, `KEY_TOPICS`) | `parity_gates.py rates NATIVE_HOLD CONTAINER_HOLD`: each gate topic (`/scan`, `/camera/detections`, `/odometry/filtered`, `/tf`, `/costmap/boundaries`, `/drive`) within 5 % of native; all other topics tabled. With `FEED=bag` (no camera) pass `--topics` without `/camera/detections` |
| 9.3 | CPU per core and memory tabled | the `hold` runs (`sys_sample.py` over the same window) | `parity_gates.py table NATIVE_HOLD CONTAINER_HOLD`: the per-core, total, stack, per-node CPU and memory table (no gate) |
| 9.4 | MPC timing on a car bag replay: container p95 period <= 1.05 x native | inputs once: `make_mpc_goal.py <mission> goal.yaml`, `filter_bag_for_layer.py <car bag> mpc_input --topics ... --inject /mpc/goal_drive f1tenth_messages/msg/DriveCommand goal.yaml --inject-after /odometry/filtered 0.5`, `bag_digest.py` on both sides; then **at least 3** `replay_localization.sh mpc $R/inputs/mpc_input OUT_i` per side (both; and `mpc_capture` with `MPC_PARAMS=OUT_1/params.yaml`); per side one `mpc_timing.py --out T.json wallrx:r1=OUT_1/solver_status_rx.json wallrx:r2=... wallrx:r3=... capture:c1=CAP/capture.npz` | `parity_gates.py mpc-timing NATIVE_T.json CONTAINER_T.json`: the median over the side's replays of the control-loop period p95 (wall clock, `wallrx`), container <= 1.05 x native; fewer than 3 replays on a side fails (`--min-runs`). Each side's run-to-run spread (max / min p95) is printed beside the gate: the noise floor to read the ratio against (two same-side replays on loaded linus: 1.078). Solve-time p95 (`capture`) tabled; `compare_runs.py mpc NATIVE_BAG CONTAINER_BAGS...` for the commands (no gate) |
| 9.5 | LLM golden check 10/10 | `llm_golden_check.py <native checkout>` (host, native env) and `llm_golden_check.py /opt/f1tenth_ws` (harness) | `parity_gates.py golden native.txt container.txt`: every fixture reproduced on both sides, identical sha256 per fixture |
| 9.5 | YOLO container (torch 2.8) vs native (2.11): IoU >= 0.99, abs dconf <= 2e-3 | `extract_frames.py <car bag> /camera/debug_image/clean/compressed IMGS` once; `yolo_run.py <model>.pt IMGS native.npz` (host) and `... container.npz` (harness) | `yolo_compare.py native.npz container.npz`: matched one to one, box and mask IoU >= 0.99, abs dconf <= 2e-3; a detection on one side only passes only within 2e-3 of the 0.30 threshold |
| 9.5 | engine vs `.pt` the same | `yolo_run.py <engine> IMGS engine.npz --task segment` (harness) | `yolo_compare.py container.npz engine.npz` with the same gates (an FP16 engine may exceed dconf 2e-3: that is reported as a finding, see `output/orin_container/w3.md`) |
| 9.5 | ZED depth medians within 2 % of native | `build_zed_grab.sh` on each side, then `zed_grab --frames 300 --json` native (host, SDK 4.2.5) and container (SDK 5.4.0), back to back, the scene still | `parity_gates.py zed-depth native.json container.json`: each of the 9 ROI medians within 2 % |
| 9.5 | DS lease 4.4-4.7 s | `lease_probe.py --profile installed --trials 5 --out $R/cont/lease.json` (harness); the native side `--profile none` for the record | `parity_gates.py lease lease.json`: every trial's removal in [3.8, 5.5] s; trials outside the plan's 4.4-4.7 s are printed as a note. The 5 s lease is re-announced every 1 s, so removal lands anywhere in about [4.0, 5.0] s depending on where in that cycle the kill falls (measured with the profile: 4.47 s on idle linus, 4.16-5.01 s under load); the 20 s default (19.4 s) stays far outside (decisions.md, W3 round 1) |
| 9.5 | watchdog 0 false triggers | the `cycles` and `hold` runs | `parity_gates.py watchdog RUN...`: 0 respawns, 0 restart-budget exhaustions (main-jazzy has no topic watchdog; the supervisor's process watchdog is what triggers) |
| 9.6 | Rust parity harnesses on Orin bags | `tools/rust_parity/{proximity,costmap_boundary,hard_stop,corridor,mpc,wall_tracker}/*` (main-jazzy-rust; harness) | the tolerance gates of `output/rust/parity_*.md` (bit identity not promised on aarch64) |
| 9.6 | on the stand, one switch at a time vs py | `phase5_bringup.sh hold OUT` (container; `FEED=live`, `STACK_ARGS="impl:=rust"`, then `corridor_impl:=rust`, `mpc_solver_impl:=rust`, `hard_stop_impl:=rust`, `wall_tracker_impl:=rust`, `guard:=shadow`) against the py hold | `parity_gates.py smoke` and `rates PY_HOLD RUST_HOLD` per switch |
| 9.6 | R6 gate tracker_ms p95 < 50 ms | `stack_probe.py --out T.json jsonstat --duration 60 --topic /perception/virtual_wall/debug --key tracker_ms` during a hold with `wall_tracker_impl:=rust` (harness, as a super client) | `parity_gates.py tracker T.json`: p95 < 50 ms |

Also available for D3 (the old jazzy D3 list), run on both sides and compared:

| Check | Script | Compare / pass |
|---|---|---|
| Discovery Server behaviour | `ds_probe.sh OUT [PORT]` | PASS on both (plain client: no topics; super client: all; reuse branch idles) |
| Diagnostics executables | `diag_start_check.sh OUT` | `diff native/states.txt container/states.txt` empty |
| Discovery isolation over N bring-ups | `phase5_bringup.sh isolation OUT N` | `isolation.json` `n_isolated` 0 per run |
| Process list and every parameter | `phase5_bringup.sh snapshot OUT` | `compare_snapshots.py NATIVE CONTAINER OUT.json --prefix-a <native install> --prefix-b /opt/f1tenth_ws/install`: only the car-profile delta (`params_delta.md`) |
| Layer replays (EKF, SLAM, costmap) | `replay_localization.sh ekf_global|slam|costmap IN OUT` | `compare_runs.py ekf_global|relay|slam|semantic_layer|costmap_boundary NATIVE_BAG CONTAINER_BAGS... --out DIR`, judged against the container's run-to-run floor |

## Inputs

Built once on the host (native env, in the checkout) under `$R/inputs/`, the
only place both sides can read them (see "Where things run"):

```bash
cd $R/inputs; P=~/dev_ws/f1tenth_car/scripts/jazzy_parity
cp -a ~/f1tenth_archive/mission_row/<run>/bag car_bag     # the car bag, copied in
# bag-fed bring-up input (FEED=bag): /scan, /odom and the static edges of the
# emptied hardware / perception components only
python3 $P/filter_bag_for_layer.py car_bag bringup_input --topics /scan /odom /tf_static \
  --tf-static-keep base_link imu --tf-static-keep base_link zed2_camera_link \
  --tf-static-keep zed2_camera_link zed2_camera_center \
  --tf-static-keep zed2_camera_center zed2_left_camera_frame \
  --tf-static-keep zed2_left_camera_frame zed2_left_camera_optical_frame \
  --tf-static-keep zed2_camera_center zed2_right_camera_frame \
  --tf-static-keep zed2_right_camera_frame zed2_right_camera_optical_frame
# MPC replay input: the whole car bag plus the goal the BT would send
python3 $P/make_mpc_goal.py ~/dev_ws/f1tenth_car/src/f1tenth_behavior/missions/drive_test_straight_2m.json goal.yaml
python3 $P/filter_bag_for_layer.py car_bag mpc_input \
  --topics $(python3 $P/bag_compat.py car_bag | cut -d' ' -f1) \
  --inject /mpc/goal_drive f1tenth_messages/msg/DriveCommand goal.yaml \
  --inject-after /odometry/filtered 0.5
python3 $P/bag_digest.py mpc_input   # on both sides (d3_side.sh): the same content_sha256
```

Car bags: a mission run's `bag/` from the archive (sqlite3, every topic the
logger records), e.g. `~/f1tenth_archive/mission_row/<run>/bag` (the Orin's
native archive, or linus's copy).

## Tests

`python3 -m pytest docker/orin/checks/test scripts/jazzy_parity/test` (no ROS
graph needed): the pure helpers, and the host scripts (`updown_cycles.sh`,
`mode_switch_cycles.sh`, `host_leftovers.sh`, `ctl_contract.sh`, `d3_side.sh`)
against fake `docker`, `docker_orin.sh`, `stack_mode.sh`, guard and ctl. The
fake docker behaves like the real one where the checks depend on it: `inspect`
answers only with `--type container`, `exec` runs in a clean environment (only
`-e` variables and the container's `F1T_CTL_ENV`), and of the image's files only
the healthcheck at its contract path exists.
