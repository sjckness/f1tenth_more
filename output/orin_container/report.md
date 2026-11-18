# The car's Jazzy container: status (2026-10-25, morning)

Plan: `~/.claude/plans/fuzzy-enchanting-teapot.md` (P0–P12). Branch `main-jazzy-rust`
(the image is built from it; `main-jazzy` carries everything but the Rust crates). Nothing pushed.
**The Orin is in native mode, checked** (host gateway active, container absent).

## Image
`f1tenth:jazzy-orin-e257b03` (`sha256:1a704265ed20…`), built on the Orin from main-jazzy-rust
e257b036 with `RUST=build`: **9.84 GB** (budget 12), every build check passed. 87 pinned
components against the manifest, 2 differ: colcon_core 0.21.1 (the Thor's 0.20.1, known) and
`cargo_ament_build` reported missing by `image_versions.sh` (the build used it; the version dump
does not find it at run time: to fix). Ubuntu archive pinned to snapshot `20261024T000000Z`, ROS
to `2026-09-11`, ZED SDK 5.4.0, CUDA 12.9 + compat, TensorRT 10.3.0.26, torch 2.8.0.
Inside: the stack, discovery server, f1tenth_ctl (systemd-free proc backend), gateway (+ whisper),
llama.cpp, YOLO/TensorRT, ZED, the five Rust packages (all switches default py).

## Done
| phase | result | commits |
|---|---|---|
| P0 | linus: `f1tenth_more:gpu`/`:cpu` removed (50 GB free; Isaac image kept); R5 WIP parked on `r5-depth-ingest-wip` | fd6d3e2a |
| P1 | car-sync: the car's 3 wall_turn commits cherry-picked onto `main`, margin 0.7, 4 of 5 missions (`straight_until_potted_plant.json` left out: the loader refuses it, stop type `' '`), merged into main-jazzy and main-jazzy-rust; `docker/orin/car_sync.json`; drift check clean against the car | ca66fc27, 137e9e4f |
| P2 | linus on the ROS snapshot 2026-09-11 (your sudo, 23:09): 407 upgrades, 0 removals; 362 packages shared with the Thor, 0 version differences; slam_toolbox, py_trees_ros, topic_tools, test_msgs, serial_driver now from apt | 403b088e, 78a52e3f |
| P3 (R0b) | rclrs 0.8 / ros-env 0.3 port of the three rclrs nodes, QDLDL vendored (offline build); linus: 47/47 recorded parity results equal, 5,669 tests no new failure | 4a42464b |
| P4 (W1) | f1tenth_ctl proc backend, container main, car profile, logger archive trigger, drift check, params delta | 1b813a7a, 4bb39fb0 |
| P5/P6 (W2) | Dockerfile completed, compose, docker_orin.sh, guard, stack_mode, ctl wrapper, user units, install | 10edca8c, a996f477, 12f21a90, cfb9e545, e257b036 |
| P6.6 (W3) | D3 parity harness (scripts/jazzy_parity, docker/orin/checks) | 393b059d |
| P7 | image built on the Orin (see above) | — |

## P8 runtime checks on the Orin (no actuation: VESC unplugged, lidar link down)
| check | result |
|---|---|
| container up, healthy (discovery + gateway) | 22–23 s |
| user/groups (dialout, video, input, render 104, jtop, zed), `nice -5`, `chrt -f 10`, rtprio 99, memlock unlimited | ok |
| GPU: torch CUDA 12.9 sees the Orin; TensorRT 10.3.0; jtop via the host socket | ok |
| stack in the container (car profile) | 63 nodes; start/stop clean; READY fails only on odometry/scan/tf/battery (no VESC, no lidar) |
| TensorRT engine into its volume (`docker_orin engine`) | 555 s; engine 18 ms vs pt 46 ms; box IoU ≥ 0.993, mask IoU ≥ 0.997 |
| guard refuses native Humble while the container runs | ok |
| archive trigger → host `f1tenth-archive.service` | ok |
| native ↔ container ×5, stale-unit case | ok (after the killall retry fix) |
| `docker_orin report` container-layer audit | 0 unexpected after the fix (381 nvidia-runtime files classed) |
| ctl wrapper overhead in native mode | +60 ms (1.23 s vs 1.17 s bare) |
Fixed on the way: discovery server start wait (1 s → up to 15 s), killall retry on back-to-back
switches, report classification, pip timeout/retries (a PyPI timeout over Wi-Fi), the Rust
`test_msgs` check (see below).

## P9.6 Rust parity on aarch64 (harness container, data-free harnesses)
| harness | result |
|---|---|
| corridor (`--sweep 10000`) | **GATE PASS** |
| hard_stop (tests, format, functions, sessions; no bags) | **638,592 ops, 0 differences** |
| MPC solver | OSQP on the same QP **bit-identical** (L2); QP assembly max 2.8e-14; per tick max 5e-6, every outlier ≤ 1e-3, statuses identical; the "≥ 99 % within 1e-9" gate FAILS (e.g. 358/523) |
| MPC closed loop | 30/32 pass; 2 speed/legacy rigs differ by 0.24 mm / 5e-4 rad and 8.6 mm / 0.026 rad |
| wall tracker numerics / fusion | last-ulp mismatches (numpy on aarch64 uses OpenBLAS kernels with FMA; the Rust port replicates x86 numpy bit for bit); fusion 1,828/3,000 cases differ at ~1e-15; rectangles 0 |
Reading: on x86 the ports are bit-identical; on aarch64 the Python reference itself differs from
x86 (BLAS/FMA), so bit identity is gone and the tolerance analysis (Python-aarch64 vs the x86
recordings as the noise floor, then bag replays on the Orin) is the next step before any Rust
switch is trusted on the car. **All switches stay py**; nothing here affects the default image.

## Waiting for you
1. **ZED camera on USB 2.0**: it enumerates at 480 Mb/s (bus 1, `1-1.1`) and fails to open in
   both the container (SDK 5.4) and natively (SDK 4.2.5); raw V4L2 frames read fine. Re-seat the
   cable / use a USB 3 port (`speed` must read 5000). The native car is affected too.
2. **Two wall_turn tests** (`TestCommitLead`, `TestChatter`) fail since the car-sync merge: the
   car's own commits changed the behaviour they encode; updating them needs your OK.
3. **params delta** (`output/orin_container/params_delta.md` on main-jazzy): five main-jazzy
   defaults differ from the car without a launch argument (YOLO 15 Hz cap, 2 threads, GPU
   pipeline, blocking sync; swept_clearance off).
4. P9 parity vs native (rates, MPC timing, LLM golden, YOLO, logger) needs the camera and the
   lidar link; P10 wheels-up and P11 floor need you at the car; P12 cutover after that.

## Open items
- aarch64 Rust tolerance analysis and Orin bag replays (above).
- Rust binaries on aarch64 keep 52 typesupport libraries in DT_NEEDED (x86 drops the unused);
  all installed, reported by build_checks, a link-flag cleanup later.
- `image_versions.sh` does not find cargo-ament-build at run time.
- The Orin repo's auto-gc logs `bad object worktrees/f1tenth_car/HEAD` during bundle fetches
  (all objects present; `.git/gc.log` now suppresses auto-gc there): to look at.
- `docker_orin exec` reads stdin (scripts must pass `</dev/null`).

## Performance on the Orin (2026-10-25 evening, MODE_50W, no jetson_clocks)
Same conditions both sides: full car stack, no VESC, no lidar link, ZED unusable (USB 2), so
perception is near idle: this is the stack's base load, not driving load. 90 s warm-up, 120 s
window, `scripts/jazzy_parity/sys_sample.py` on the host (`perf/`, `perf/compare.txt`).
| | native Humble | Jazzy container |
|---|---:|---:|
| Orin total CPU (12 cores) | 24.8 % | **17.5 %** |
| stack CPU, sum (% of one core) | 179 | **101** |
| of which the Discovery Server | 72.1 | **1.1** |
| stack without the Discovery Server | 107 | 100 |
| stack RSS | 8.15 GB | 9.55 GB |
Where it differs: Humble's fast-discovery-server burns 72 % of a core (Jazzy's 1 %);
swept_clearance retired (7.6 % → 0); detection_3d / obstacle_projector with /tf_static-only TF
(2.9 / 2.5 % → 0.2); foxglove 5.1 → 1.0 %. Higher in the container: mpc_corr 27.0 → 31.8 %,
semantic_layer 4.9 → 7.1 %, hard_stop 4.2 → 5.3 %. Memory: yolo_detector 705 → 1,513 MB (torch
2.8 / CUDA 12.9), the other nodes ~+10 MB each.

MPC solver on the Orin (`bench_solve.py`, 723 ticks x 7, CPU 5, harness container): Python mean
18.2 ms, p95 24.4 ms; **Rust mean 2.6 ms, p95 4.2 ms (5.8x)**; live configuration
(offset_free, 8 obstacles, 4 walls) p95 24.7 → 4.4 ms. linus x86 for comparison: 10.4 → 2.0 ms.
