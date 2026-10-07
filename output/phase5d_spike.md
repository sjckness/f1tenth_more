# Phase 5d — D1a spike: option C on the Orin

Date: 2026-10-07, 14:24–15:35. Machine: `jetson-orin` (AGX Orin 64 GB, L4T
R36.5.0, MODE_50W, jetson_clocks off). Branch `jazzy` at `083c8e8`.
Spike wall time about 1 h of the 2 h budget.

**Reviewed 2026-10-07** (decisions in section 9). G3's camera run passed
afterwards; D1 follows.

## 0. Verdict

**Option C works on this Orin.** `ubuntu:24.04` with the Thor's apt Jazzy
(snapshot `2026-09-11`) and 22.04-Tegra CUDA 12.9 runs the GPU, TensorRT, the
ZED SDK, llama-server and the 32-package workspace. It needed seven fixes, all
understood and all D1 inputs (section 4).

| Gate | Result | Key number |
|---|---|---|
| G1 CUDA + YOLO | **PASS** | torch 2.8.0 / CUDA 12.9 / cuDNN 9.10.2 on the Orin GPU. YOLO vs Orin Humble torch 2.11: 35/35 detections matched, box IoU ≥ 0.99935, mask IoU ≥ 0.9937, conf Δ ≤ 0.0017; 1 extra detection is a 0.29998 vs 0.30037 threshold crossing |
| G2 TensorRT | **PASS**, one note | TRT 10.3.0.26 cp312 builds and runs engines (max rel. err 1.5e-7). The host's Humble `.engine` does not load: JetPack ships 10.3.0.30, and plans need the exact build |
| G3 ZED | **PASS** (camera run 15:43, after the review) | SDK 5.4.1 and 5.4.0 install on noble; `ldd -r` clean; `zed_components` builds, loads with `RTLD_NOW`, registers `ZedCamera`/`ZedCameraOne`. **ZED 2i, SDK 5.4.0 in the container: 300/300 frames, 0 failed, 59.5 Hz, IMU 300/300 with 300 distinct timestamps** (host SDK 4.2.5 reference: 58.9 Hz) |
| G4 LLM | **PASS**, one note | Host llama-server on the image's CUDA 12.9: 37/37 layers on the GPU, correct answers, steady state 350 tok/s prompt / 33.5 tok/s generation. Cold start costs ~2 min of PTX JIT (binary built without sm_87) |
| G5 ROS (definition confirmed by Andreas 2026-10-07) | **PASS**, with D1 additions | 305/305 apt `ros-jazzy-*` at the Thor's exact versions; 32/32 packages built; `ldd -r` scan empty; Discovery Server + SHM across two containers: 956/956 messages, 0 gaps |

**Recommendation: C for D1**, with the fixes in section 4. The camera half of
G3 ran at 15:43 once the ZED was plugged in and passed (section 5.3).
Nothing found points to A′.

## 1. Blockers B1–B3: verified

| | Claim | Checked |
|---|---|---|
| B1 | dockerd up with `bridge: none`, `iptables: false`, `ip6tables: false`; `daemon.json.pre-5d` kept; nvidia runtime registered | yes: Docker 29.6.0 active, runtimes `nvidia`/`runc`, default `runc`, data-root `/scratch/docker-data` |
| B2 | `fabiocar` in `docker` | yes (GID 994); `docker` without sudo works |
| B3 | `ssh andre@jetson-thor` with the Orin's key | yes, used read-only except for the approved test clone (section 3) |

What host networking means in practice, measured on this daemon:
- `docker build` without `--network=host` fails at its first `RUN`: `network bridge not found`.
- `docker run` without `--network host` starts **silently** with only `lo` and no DNS.

This is now in `CLAUDE.md` ("Orin Docker") and in the D2 spec
(`phase5d_analysis.md` §6.2: compose `network_mode: host` + `build: {network: host}`;
`docker_orin.sh` passes `--network=host` to every build and run).
`docker_orin.sh` itself is a D2 file and does not exist yet. The spike's
`docker/orin/spike/spike.sh` already enforces the rule. Kernel items:
backlog **M21** (modules from a different build) and **M22** (what retries
`ipv6` every ~20 s).

## 2. Thor manifest: complete, sync confirmed

`docker/orin/jazzy_versions.manifest` is complete; raw captures are in
`docker/orin/thor_capture/`.

- **apt sync `jazzy/2026-09-11`: confirmed.** All 386 installed `ros-jazzy-*` versions are byte-equal to `snapshots.ros.org/jazzy/2026-09-11` (arm64). The 2026-06-18 snapshot matches only 120.
  - My own first guess against it was wrong. The Thor's fastrtps 2.14.6 is older than the 2.14.7 the live repo offers now, but that 2.14.7 came with the 2026-10-05 sync.
  - Snapshot key `4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA`, verified against the InRelease.
- **ZED SDK on the Thor: 5.4.0**, not 5.4.1 (installed 2026-06-23). Both L4T36.5 installers exist, so D1 uses **5.4.0**, as the work order said. D0 had taken 5.4.1 from the wrapper's `package.xml`.
- **`.bashrc`** sources `scripts/env/jazzy.sh` and exports `ZED_DIR`. That confirms the FB2 environment is live (open item in D0 §6.5).
- **Submodules:** the five commits are identical on the Thor's checkout.
- **The Thor's Python has changed since Phases 0–5. It is reported here, not touched.**
  - On 2026-10-06 at 17:25–17:26, `~/.local` got `pip install --user ultralytics`, then `torch`/`torchvision` force-reinstalled from `download.pytorch.org/whl/cu130`.
  - Result: torch 2.14.1+cu130, torchvision 0.29.1+cu130, ultralytics 8.4.174, triton, and the **PyPI CUDA 13 stack** (`cuda-toolkit 13.0.3.0`, `nvidia-cudnn-cu13`, `nvidia-cublas`, `nvidia-nccl-cu13`, `nvidia-cusparselt-cu13`, …).
  - `setuptools` in `~/.local` was 84.0.0 at 14:27 today and **79.0.1 at 14:42**. It shadows apt's 68.1.2 for every `colcon build` on the Thor.
  - Effects:
    - It is the PyPI-CUDA pattern this phase's hard rule (and M9) forbids.
    - "No torch on the Thor" (D0, M9) is no longer true.
    - Any Thor test result from now on runs on top of it.
  - The manifest's parity reference stays the validated pre-10-06 set.
  - **Update (Andreas, 2026-10-07):** `andre@jetson-thor` is Andreas's account and the install was Phase 7 prep. Andreas cleans it up after Phase 5d; it is not to be changed here. The Thor reference test runs with `PYTHONNOUSERSITE=1` (section 3).

## 3. Thor `colcon test` at `083c8e8`: armed, not run yet

**Not run yet.** The Thor is not at `083c8e8`:
- HEAD is `13ba262c`, merged today at 14:23 (`083c8e8` + `a4488e58` + a local "updated sim clock" commit);
- `install/` is from the 10-05 build at `85715581`;
- since then `f1tenth_description` (new data file), `f1tenth_bringup` and `f1tenth_perception` (new launch file) have changed.

Approved option: a separate checkout. What is on the Thor:
- `~/dev_ws/f1tenth_p5d_083c8e8` is a local `git clone` at `083c8e8`, not a `git worktree`.
  - It gives the same isolation but writes nothing into andre's `.git`; a worktree adds metadata there.
  - Submodules come from andre's local module repos, with no network.
  - `husarion_gz_worlds` is not initialised, as on the Thor and in the image.
- `~/p5d_thor_test_083c8e8/run.sh` (background, `nice -n 10`):
  - waits until no ROS stack has run for 5 consecutive minutes, giving up after 8 h;
  - records `env`, `pip freeze` (system and user) and `colcon list`;
  - runs `colcon build --symlink-install` and `colcon test`, both with `--test-result-base ~/p5d_thor_test_083c8e8/test_results`;
  - test environment: `ROS_DOMAIN_ID=87`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, no `ROS_DISCOVERY_SERVER`, the 5 s lease profile;
  - andre's workspace and install are never touched.
- **v3, 15:36 (decision 2026-10-07):** the waiting job was restarted with `PYTHONNOUSERSITE=1` exported for the build **and** the test, so `~/.local` is invisible. The run log records `python3 -c "import setuptools, sys; print(setuptools.__version__, sys.flags.no_user_site)"` before the build and again after sourcing the install. Preview from the same shell setup: `68.1.2 1` (apt setuptools, user site off). `pip list --user` is still saved, before the export, as a record. The result is the clean apt-only reference.
- Status at 15:30: still waiting. The two-machine sim stack (`supervisor_bringup sim:=true`, DS on 10.42.0.2) has been restarted repeatedly since 14:20.
- When `run.log` says `done`: `scp -r andre@jetson-thor:p5d_thor_test_083c8e8 output/phase5d_logs/thor_test/` and summarise per package from the junit XML.

Side observation, Thor only: during that sim run there were 8+ concurrent
`ekf_cost_observer_node` processes, started in bursts about a minute apart.
Not investigated. Now backlog **M23** (MEDIUM, before Phase 6), with the
evidence; D3 #3/#14 check the container for duplicate node processes.

## 4. What the image needed: D1 inputs

The spike image is `f1tenth:jazzy-orin-spike`, `sha256:4809b678…`, 12.0 GB
uncompressed. It includes a superseded cuDNN layer, so D1's single-stage
install will be smaller. Recipe: `docker/orin/spike/Dockerfile`. Build logs:
`output/phase5d_logs/spike_build_*.log`. Every failure below was hit, fixed
and rebuilt.

| # | Problem | Cause | Spike fix | D1 |
|---|---|---|---|---|
| F1 | `cuda-compat-12-9` will not install | it `Pre-Depends: nvidia-l4t-core`, the host's L4T package | `dpkg-deb -x` its payload (`/usr/local/cuda-12.9/compat/*`) | same |
| F2 | torch refuses to load: "compiled against cuDNN 9.10.2, found 9.8.0" | the jp6/cu129 torch 2.8.0 wheel is built against 9.10.2 | extra layer with `cudnn-local-tegra-repo-ubuntu2404-9.10.2` | install **9.10.2** in the cuda stage |
| F3 | `zed_debug` link: `libnvargus_socketclient.so: undefined reference to jpeg_set_hardware_acceleration_parameters_enc` | two different libraries are both called `libnvjpeg.so`: L4T's Tegra libjpeg (needed by argus → `libsl_zed.so`) and CUDA's nvJPEG dev symlink. With CUDA first on the path the wrong one wins, at link **and** at run time. The host has no CUDA nvJPEG dev symlink | remove CUDA's unversioned `libnvjpeg.so` (torchvision uses `.so.12`); search order `cuda/compat : L4T nvidia : cuda/lib64` | do not install `libnvjpeg-dev-12-9`; set that order in the cuda stage |
| F4 | TensorRT builder: "Unable to load library: `libnvinfer_builder_resource.so.10.3.0`" | TRT `dlopen`s it by file name, but its SONAME is `do_not_link_against_nvinfer_builder_resource`, so the ld cache never has the file name. JetPack's debs avoid this by installing into `/usr/lib/aarch64-linux-gnu` | `/opt/tensorrt/lib` on `LD_LIBRARY_PATH` | same, or unpack the TRT libs into the default lib dir |
| F5 | every ZED library "not found" for the container user | the SDK installer creates group `zed` with the image's next free GID (1001) and makes `/usr/local/zed` `root:zed 0770`; the user carries the host's `zed` GID 1005. Also, the installer never runs `ldconfig` | `groupmod -g 1005 zed`, `chgrp -R`, `ldconfig` | `groupadd -g 1005 zed` **before** the installer, then `ldconfig` |
| F6 | rosdep: 6 system deps missing; pip took scipy **1.17.1** and matplotlib **3.11.2** from PyPI | the closure's apt Python (`python3-scipy`, `-matplotlib`, `-jsonschema`, `-requests`, `-ruamel.yaml`, `-tk`) was not in the image, so pip filled what ultralytics needs. **`jsonschema` was missing entirely** | none (spike) | apt-install those six **before** pip, so the Thor's scipy 1.11.4 / matplotlib 3.6.3 / jsonschema 4.10.3 are what pip sees |
| F7 | link step needs L4T libs that only the nvidia runtime mounts | `libsl_zed.so` links `libnvbufsurface`, `libnvargus_socketclient`, … | host `/usr/lib/aarch64-linux-gnu/nvidia` as a named build context, bind-mounted for the colcon step only | same; nothing from it lands in the image |

Smaller points:
- The torch wheels were fetched from the index and their sha256 checked against it.
- The image has no `nvidia-*` / PyPI CUDA package. `spike.sh` refuses the build if one appears. `nvidia-ml-py`, which ultralytics needs, is pure-Python NVML bindings.
- D0's "31 packages built" was a miscount: **32** build (34 minus `f1tenth_sim`, `f1tenth_more`), the same as the Thor's 10-05 build.
- **cuda-compat is optional on this host.** Without it, the host's r36.5 `libcuda` (driver API 12.6) runs torch's 12.9 kernels by minor-version compatibility; with it the driver API is 12.9. The spike keeps compat, as jetson-containers does. D1 can drop it if D3 shows no difference.

## 5. Gate details

Logs are in `output/phase5d_logs/spike/`. Every container went through
`spike.sh run`:
- the guard: `/proc` scan for Humble/ROS processes, UDP 11811, other `f1tenth:*` containers;
- `--runtime nvidia --network host --ipc host`;
- the host's uid/gid plus groups video, render, dialout, zed;
- `/dev` bind mount with cgroup rules (81, 189, hidraw 510): the D2 design, no privileged mode.

The guard passed every time. No Humble process ran during the spike.

### 5.1 G1: CUDA and YOLO

`g1_torch.log`: Python 3.12.3, torch 2.8.0, CUDA 12.9, cuDNN 9.10.2, device
`Orin` sm_87.

| Check | Result |
|---|---|
| fp32 matmul 2048² vs CPU fp64 | rel. err 9.5e-7 (1.7 TFLOPS) |
| fp16 matmul 2048² vs CPU fp64 | rel. err 2.7e-4 (7.7 TFLOPS) |
| cuDNN conv vs CPU fp64 | rel. err 3.2e-7 |

YOLO comparison (`yolo_compare_torch211_vs_torch28.json`):
- **Setup:** `yolo26s-seg.pt`, called the way `yolo_detector_node` calls it (`conf=0.3`, `retina_masks=True`, BGR input). Ultralytics 8.4.90, opencv 4.11.0.86 and numpy 1.26.4 on both sides, so torch is the only variable.
- **Images:** 95 = the 93 archived frames `f1tenth_archive/verify/bottle_2026-09-02T15-27-43_frames` + Ultralytics' `bus.jpg` and `zidane.jpg` (mission bags only hold annotated images).

| | Host Humble (py3.10, torch 2.11.0, cu12.6, cuDNN 9.3) | Image (py3.12, torch 2.8.0, cu12.9, cuDNN 9.10.2) |
|---|---|---|
| detections | 35 | 36 |
| latency mean / p95 | 47.4 / 47.5 ms | 48.6 / 48.9 ms |

- **Floor:** host vs host, two runs: bit-identical (IoU 1.0, Δconf 0).
- **Host vs image:** 35 matched; worst box IoU 0.99935, worst corner 0.11 px, worst mask IoU 0.9937, worst |Δconf| 0.00168.
- **The 36th detection** (`frame_0006`, class 25): the same box in both. torch 2.11 scores it 0.29998, torch 2.8 scores it 0.30037. It is a threshold crossing, not a new object.
- **Against D0 §7 #13's provisional tolerance:** box IoU ≥ 0.99 passes; **conf Δ ≤ 1e-3 does not (1.7e-3)**. That gap is a torch-version effect: torch 2.11 is not available for cp312 on JP6, so it cannot be removed with option C.
- **For D3:** compare detections as objects, with conf Δ ≤ 2e-3 and threshold crossings within that band counted as agreement. Your call.

### 5.2 G2: TensorRT

`g2_trt.log`: TRT 10.3.0 (tarball 10.3.0.26, cp312 wheel).
- **Small engine:** a conv+ReLU engine builds in fp32 and fp16, runs on torch CUDA buffers and matches the CPU reference (rel. err 1.5e-7 / 1.7e-7).
- **Host engine:** the Humble `models/yolo26s.engine` (built 2026-07-08 by JetPack's TRT **10.3.0.30**) fails to deserialise: "expecting library version 10.3.0.26".
- **Impact:** none by default, since the deployed default is `yolo26s-seg.pt`. An `.engine` must be rebuilt in the image; the alternative is to take JetPack's 10.3.0.30 libs from NVIDIA's r36.5 apt repo, as with CUDA. The ZED SDK's own TRT use is unaffected: the stack runs depth `PERFORMANCE`, which needs no AI models.

### 5.3 G3: ZED

`g3_zed.log` (5.4.1) and `g3_zed_540.log` (5.4.0, image `f1tenth:jazzy-orin-spike-zed540`) give identical results.

- **Install:** the installer runs on noble (`silent skip_tools skip_python skip_drivers skip_cuda`), with one warning: "container detected, platform cannot be identified at build time". Its `sysctl net.core.rmem_max` write is ignored in a build (read-only `/proc`). It matters only for ZED network streaming, which the stack does not use.
- **Libraries:** `ldd -r` is empty for `libsl_zed.so`, `libsl_ai.so`, `libzed_camera_component.so`, `libzed_camera_one_component.so` and `zed_debug_proc`, with the runtime's L4T mounts.
- **Components:** both load with `RTLD_NOW` and are registered (`stereolabs::ZedCamera`, `stereolabs::ZedCameraOne`).
- **SDK call:** a C++ test program (`gates/zed_grab.cpp`), compiled and linked in the image, prints `sdk_version 5.4.1` (or 5.4.0) and `devices 0`. The Argus / `zed_x_daemon` messages are the SDK probing for GMSL ZED X cameras and are harmless for a USB ZED 2i.
- **300 frames: PASS (15:43, `g3_zed_540_camera.log`).** The camera was not connected during the spike; Andreas plugged it in after the review. The gate command:

  ```bash
  SPIKE_EXTRA_ARGS="-v /scratch/fabiocar/p5d_spike_ctx/zed_settings:/usr/local/zed/settings" \
    docker/orin/spike/spike.sh run bash /spike/gates/g3_zed.sh 300
  ```

  - It opens the camera with the stack's settings: HD720 @ 60 from `zed2i.yaml`, depth `PERFORMANCE` from `zed2_perception.yaml`.
  - It grabs 300 frames, retrieving the left image and the depth measure each frame, and reads the IMU.
  - Pass is 300/300 grabbed.
  - The calibration is a **copy** of the host's `SN36866199.conf`, so the SDK cannot write the host's file.
  - It reads only. No firmware tool is installed (`skip_tools`).

  **Camera result (image `f1tenth:jazzy-orin-spike-zed540`, SDK 5.4.0).** USB: video `2b03:f880` on root port 2-2 at 5 Gb/s (direct, no hub); HID `2b03:f881` at 12 Mb/s behind the camera's own USB 2 hub.

  | | Container, SDK 5.4.0 | Host Humble, SDK 4.2.5 (reference) |
  |---|---|---|
  | device | ZED 2i S/N 36866199, AVAILABLE, `/dev/video0` | same |
  | open | SUCCESS in 3.13 s; FW 1523, sensors FW 777, HD720@60 | SUCCESS in 2.09 s; same FW and mode |
  | frames | **300/300, 0 failed** | 300/300, 0 failed |
  | grab rate | **59.5 Hz** (image + depth retrieved every frame) | 58.9 Hz |
  | depth | 1280×720, centre depth valid 300/300, mean 7.18 m | 1280×720, 300/300, 7.27 m |
  | IMU | 300/300 reads, 300 distinct timestamps, \|a\| mean 9.815 m/s² | same, 9.815 m/s² |

  - **Rate vs the expected ≈22 Hz.** Both SDKs grab at the camera's 60 Hz with PERFORMANCE depth at HD720. The SDK is not the limit, and 5.4.0 in the container matches 4.2.5 on the host. A ≈22 Hz figure would be a downstream topic rate (wrapper publishing, `pub_frame_rate` 30, downscale, positional tracking, the detector's subscription), which D3 #3 and D6 measure on topics. Not a G3 criterion.
  - **Other messages:** SDK 5.4.0 warns "PERFORMANCE is deprecated, use NEURAL"; `zed2_perception.yaml` still sets `depth_mode: 'PERFORMANCE'`. It works on 5.4 as on the Thor; a change of depth mode is a Phase 7 perception decision, not 5D.
  - **Reference run:** the same `zed_grab.cpp`, compiled against the host's `/usr/local/zed` (4.2.5) into the scratchpad and run on the host while no container ran (`g3_zed_host_sdk425_reference.log`). The host's `SN36866199.conf` md5 is unchanged before and after (`7a7bc6f5…`).

### 5.4 G4: llama-server

`g4_llm.log`. Binary and model are mounted read-only at their host paths and
run with the stack's arguments (`-ngl 999`, port 8083, cwd = llama.cpp root).
- **Linking:** `libcudart`/`libcublas` resolve to the image's 12.9, and `libcuda.so.1` to compat.
- **Offload:** 37/37 layers on CUDA0 (Orin), 3.6 GB.
- **Answers:** "Rome" (8 tokens), then "Paris" for both 64-token requests.
- **Steady state:** 353 / 347 tok/s prompt, 33.4 / 33.7 tok/s generation.
- **Cold start: about 2 min** (health after 52 s, first prompt 57 s).
  - The binary was built for `ARCHS = 500,610,700,750,800,860,890`, **without 87**, so every kernel is PTX-JIT-compiled on first use.
  - On the host the JIT cache in `~/.nv/ComputeCache` hides this. In a fresh container there is no cache.
  - The stack starts llama-server per mission, so this matters. For D2: rebuild llama.cpp `42532afff` with `CMAKE_CUDA_ARCHITECTURES=87` (best), or give the container a persistent `CUDA_CACHE_PATH`.
- Port 8083 was free before and after.

### 5.5 G5: ROS

- **apt versions** (`g5_ros.log`): 305 `ros-jazzy-*` installed, **all 305 at the Thor's exact version**. The 81 Thor-only packages are the excluded GUI/demo/sim ones: rviz, rqt, desktop, demos, examples, gz vendors, ros2_control.
- **Build:** 32/32 packages finished (`--merge-install`, Release; 1 min 10 s with ccache, ~5.5 min cold at MODE_50W).
- **`ldd -r` scan (CLAUDE.md):** empty.
- **rosdep** (`g5_rosdep.log`): the six apt Python keys of F6. Also, `ament_python` "has no rosdep definition" in `f1tenth_localization`, `vesc_tuning` and `f1tenth_diagnostics`: their `package.xml` declare it as a key. It is the same on any host and harmless; LOW debt.
- **DDS across two containers** (`dds_two_containers.sh`; `g5_dds_*.log`):
  - Setup: container A runs `fastdds discovery` on 127.0.0.1:11811 plus a 50 Hz talker; container B runs a listener. Both have the stack's env and the 5 s lease profile.
  - Result: **956/956** messages in the 20 s window, 0 gaps, first message 0.92 s after start.
  - SHM: Fast DDS segments (`fastrtps_<guid>`, `fastrtps_port74xx`) appeared in the **host's** `/dev/shm`, owned by uid 1004. So `--ipc host` gives the same SHM path as bare metal, with no root-owned files.
  - The Discovery Server, killed by the script, left 5 segment files behind. I removed them after checking no process mapped them.

## 6. Changes to the D0 plan

1. **ZED SDK 5.4.0** (Thor parity), not 5.4.1.
2. **cuDNN 9.10.2**, not "cuDNN 9 (24.04 repo)": the torch wheel fixes the version.
3. **Python:** apt first (Thor set), then pip for osqp 1.1.3, pyarrow 25.0.1, torch/torchvision from the JP6 wheels, ultralytics 8.4.90 and opencv-python 4.11.0.86. Add a `pip check` and a no-PyPI-CUDA check.
4. **D3 GPU tolerance:** conf Δ ≤ 2e-3 with threshold crossings allowed (needs your decision). Torch 2.8 vs 2.11 cannot be equalised on cp312.
5. **D2:** llama.cpp sm_87 build or a persistent CUDA JIT cache; `group_add` zed = 1005 must match the image's `zed` GID.
6. **Package count 32**, not 31 (D0 §4.1 and §7 #1).

## 7. Files

New (committed with this report, no push):
- `docker/orin/spike/`:
  - `Dockerfile`, `spike.sh` (build/run/guard, host networking), `constraints.txt`, `ros_packages.txt` (288 names from the Thor's list);
  - `dds_two_containers.sh`;
  - `gates/`: `g1_torch.py`, `yolo_run.py`, `yolo_compare.py`, `g2_trt.py`, `g3_zed.sh`, `zed_grab.cpp`, `g4_llm.sh`, `g5_ros.sh`, `dds_probe.py`.
- `docker/orin/jazzy_versions.manifest` (complete), `docker/orin/thor_capture/` (raw Thor outputs).
- `output/phase5d_logs/` (build logs, `spike/` gate outputs, 2.7 MB).

Changed: `CLAUDE.md` ("Orin Docker"), `output/backlog.md` (M21, M22, M23,
L16, M9 status), `output/phase5d_analysis.md` (§6.2 host-networking rule,
§6.4 H1/H2 done, §4.1/§7 package count 32, §7 #3/#12/#13/#14 decisions).

Outside the repo:
- `/scratch/fabiocar/p5d_downloads` (4.6 GB of installers and wheels);
- `/scratch/fabiocar/p5d_spike_ctx` (source tree, YOLO images, ZED settings copy);
- Docker images `f1tenth:jazzy-orin-spike`, `f1tenth:jazzy-orin-spike-zed540`, `ubuntu:24.04`;
- on the Thor: `~/dev_ws/f1tenth_p5d_083c8e8` and `~/p5d_thor_test_083c8e8` (waiting job).

The host's Humble install, `/opt/ros/humble`, host Python and udev were not
changed. Ultralytics' host-side settings went to the scratchpad
(`YOLO_CONFIG_DIR`).

## 8. Open

- ~~G3 camera~~: done, PASS (§5.3).
- **Thor `colcon test`:** waits for a quiet Thor (§3).
- **Decisions for you:**
  - confirm C for D1;
  - the D3 YOLO tolerance (§5.1);
  - llama.cpp sm_87 rebuild vs JIT cache (§5.4);
  - whether to report the Thor's PyPI-CUDA install to andre, and how (§2).

## 9. Decisions (Andreas, 2026-10-07)

| Item | Decision |
|---|---|
| Order | G3 camera first (ZED 2i on direct USB-A, no hub): 300/300 frames, grab rate reported (≈22 Hz expected, PERFORMANCE at HD720), IMU read. A failure stops the phase for a report, with no workaround. Then commit D0 + D1a on `jazzy` (no push, no tag), then D1, then `output/phase5d_image.md`, then STOP before D2 |
| Base option | **C**, subject to G3 |
| D3 YOLO tolerance | Objects matched one-to-one, box IoU ≥ 0.99, mask IoU ≥ 0.99, \|Δconf\| ≤ 2e-3. A one-sided detection counts as agreement only if its conf on the other side is within 2e-3 of the 0.30 threshold. Recorded as a torch 2.8 vs 2.11 effect that C cannot remove (`phase5d_analysis.md` §7 #13) |
| llama.cpp | Rebuilt in the image: commit `42532afff`, `-DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87`, Release, under `/opt/llama.cpp`. The host binary is left alone. The container sets `llama_server_path` to the image binary through the existing parameter, no symlinks. Persistent `CUDA_CACHE_PATH` on a named volume. D3 #12: golden 10/10 + live missions equal to the Orin Humble live run, cold-start time reported |
| TensorRT | Keep the 10.3.0.26 tarball. `.engine` files are always rebuilt in the image (Phase 7 rule). The detector must refuse an engine of a different TRT version with a clear log line, not a crash; if that guard exists, confirm it |
| cuda-compat | Kept |
| ZED SDK | 5.4.0 |
| cuDNN | 9.10.2 |
| Python order | apt first (the six F6 packages included), then pip with constraints, then `pip check` + the no-PyPI-CUDA check, both failing the build |
| F1–F7 | All into D1. F7's build-only bind mount of the host L4T libs is documented in the Dockerfile header: the image builds only on a JP6 / r36.5 host |
| Package count | 32; D0 §4.1 and §7 #1 fixed |
| `ament_python` rosdep key | Backlog L16 (LOW), not this phase |
| Thor | Andreas's account; ~/.local cleaned up by Andreas after 5D. Reference test with `PYTHONNOUSERSITE=1` (done, section 3). Orphaned `ekf_cost_observer_node`: backlog M23; D3 #3/#14 check duplicate node processes after the 10 bringup cycles (`phase5d_analysis.md` §7) |
| Unchanged | D2 design, D3 list (with the tolerance above), D4 (both power modes, with perception load), D5, D6 |
