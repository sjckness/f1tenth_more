# Phase 5d — D1: the Orin Jazzy image

Date: 2026-10-07. Machine: `jetson-orin` (AGX Orin 64 GB, L4T R36.5.0,
MODE_50W, jetson_clocks off). Branch `jazzy`. **STOP before D2.**

## 0. Summary

| | |
|---|---|
| Image | **`f1tenth:jazzy-orin-a515f27`**, `sha256:2fef94aa0fe11127feb60c77e3f34237891a1c764bc2f6946b888b95ea436b20` |
| Built from | commit `a515f27` (label `org.opencontainers.image.revision`); sources tree hash `39d8efe3…` (`/opt/f1tenth_ws/SOURCES`) |
| Size | 10,823,262,420 bytes uncompressed (10.8 GB) |
| Build | `docker/orin/build.sh` (section 1). Cold, measured: CUDA 1.7 min, ROS apt 6.9 min, pip 1.3 min, ZED 0.6 min, llama.cpp ≈12 min (in parallel with ROS/Python), workspace 5.5 min; ≈25 min end to end. From cache: under 1 min |
| Manifest | **All 305 `ros-jazzy-*` at the Thor's exact Debian version**; every named ROS pin, the Python set, ZED 5.4.0 and the five submodules equal the manifest. One mismatch: colcon-core 0.21.1 vs 0.20.1 (section 2.2) |
| Build checks (fatal) | `pip check`, no PyPI CUDA, apt Python not shadowed by pip, closure imports, F3/F5 invariants, CLAUDE.md `ldd -r` scan over the workspace + ZED SDK + llama.cpp: **all passed** |
| Runtime checks | GPU: torch 2.8.0 / CUDA 12.9 / cuDNN 9.10.2 on the Orin. **llama-server built in the image: healthy after 3.1 s, first completion 0.38 s** (host binary in the spike: 52 s / 57 s). **ZED 2i: 300/300 frames, 59.7 Hz, IMU 300/300.** TensorRT: an engine built in the image is accepted and runs; the Humble engine is refused with one log line |
| Thor reference `colcon test` at `083c8e8` | **3231 tests, 0 errors, 113 failures, 128 skipped** (v5, section 6); per package and the failing IDs are in `output/phase5d_logs/thor_test/v5/` |

D1 needed three fixes beyond the spike's F1–F7 (section 4), each found by a
check, fixed and rebuilt. Commits on `jazzy` (not pushed): `6011b55`
(image + detector guard), `7420a11` (llama.cpp stub link), `a515f27`
(GPU env, `USER` by name), plus this report.

## 1. How the image is built

```bash
docker/orin/fetch_inputs.sh            # once: ~4.2 GB into /scratch/fabiocar/p5d_downloads, sha256-checked
docker/orin/build.sh [commit]          # default HEAD; background-friendly; log in output/phase5d_logs/
docker/orin/image_report.sh [image]    # versions + diff against the manifest (runs the guard first)
```

- **One commit in, one tag out.** `build.sh` stages *everything* from the commit with `git archive`, never the working tree: the workspace (`stage_sources.sh`: `src/` plus each submodule at the commit the tree pins, from the local module stores or the `.gitmodules` URL; `husarion_gz_worlds` skipped), `scripts/env/`, and `docker/orin/` itself (the Dockerfile and helpers). The tag is `f1tenth:jazzy-orin-<short sha>`. Uncommitted edits are never in the image; `build.sh` says so when there are any.
- **Pinned inputs.** `inputs.sha256` lists the six binary inputs with URL and sha256: CUDA 12.9.1 Tegra repo, cuDNN 9.10.2 Tegra 24.04 repo, the TensorRT 10.3.0.26 tarball, ZED SDK 5.4.0 for L4T 36.5, and the torch / torchvision cp312 wheels. `build.sh` refuses on any mismatch. The ROS snapshot key is in the repo (`keys/ros-snapshots.asc`), and its fingerprint `4B63…25EA` is checked in the build. Network inputs pinned by content: the apt snapshot `jazzy/2026-09-11` and llama.cpp by full commit hash. PyPI packages by version: osqp, pyarrow, ultralytics, opencv-python, jetson-stats and their dependencies. Ubuntu's own archive is **not** snapshotted (section 2.2).
- **Host requirements** (Dockerfile header):
  - Host networking (CLAUDE.md "Orin Docker").
  - **An L4T r36.5 host:** `build.sh` refuses elsewhere. The workspace build and the build checks bind-mount the host's `/usr/lib/aarch64-linux-gnu/nvidia` (F7); nothing from it is copied into the image.
- **Stages:**
  - `base`: Ubuntu, harness tools, user `fabiocar` 1004 and groups `zed` 1005 / `jtop` 1003.
  - `cuda`: CUDA, compat, cuDNN, TensorRT.
  - `ros`: snapshot apt + the apt Python.
  - `py`: pip + checks.
  - `zed`: the SDK.
  - `llama`: built in parallel from `cuda`.
  - `ws`: workspace, llama path edit, checks, metadata.
  - Per-commit metadata comes last, so a commit that doesn't change the sources reuses every layer.
- **Layers:**

  | Layer | Size |
  |---|---|
  | CUDA + cuDNN | 5.28 GB |
  | ROS apt | 2.28 GB |
  | pip (torch, …) | 1.35 GB |
  | base apt | 0.83 GB |
  | TensorRT | 0.45 GB |
  | workspace install | 0.16 GB |
  | ZED | 0.16 GB |
  | sources (incl. the `.pt` models) | 0.13 GB |
  | llama.cpp | 0.08 GB |

  Most of the CUDA layer is the `-dev` packages' static `.a` libraries, which nothing here links. Dropping them through a copy stage would save roughly 2–3 GB. Not done: size is not a gate.

## 2. Resolved versions vs `jazzy_versions.manifest`

Full outputs:
- `output/phase5d_logs/d1_image_versions_jazzy-orin-a515f27.txt`: every resolved version;
- `…_diff_jazzy-orin-a515f27.md`: the generated diff, reproduced below;
- `…_inspect_….txt`: image ID, size, revision.

### 2.1 Table

| Area | Component | Manifest (Thor) | Image | |
|---|---|---|---|---|
| ros | fastrtps / rmw_fastrtps_cpp | 2.14.6 / 8.4.4 | 2.14.6 / 8.4.4 | = |
| ros | rclcpp / rclpy / ros_base | 28.1.22 / 7.1.12 / 0.11.0 | same | = |
| ros | diagnostic_updater / robot_localization / slam_toolbox | 4.2.7 / 3.8.3 / 2.8.5 | same | = |
| ros | py_trees / py_trees_ros | 2.5.0 / 2.5.0 | same | = |
| ros | rosbag2 / rosbag2_storage_mcap | 0.26.11 / 0.26.11 | same | = |
| ros | cv_bridge / system OpenCV | 4.1.0 / 4.6.0 | same | = |
| ros | foxglove_bridge / urg_node / serial_driver | 3.5.0 / 1.1.2 / 1.2.0 | same | = |
| ros | nav2_* | 1.3.13 | 1.3.13 | = |
| ros | topic_tools / rosbridge_server / joy / backward_ros | 1.3.4 / 2.7.1 / 3.3.0 / 1.0.8 | same | = |
| ros | image_transport_plugins / v4l2_camera / xacro / robot_state_publisher / zed_description | 4.0.7 / 0.7.3 / 2.1.1 / 3.3.4 / 0.1.5 | same | = |
| ros | **all `ros-jazzy-*`, full Debian version** | 386 on the Thor | **305 installed, 305 equal, 0 differ** | = |
| python | numpy / scipy / matplotlib / jsonschema | 1.26.4 / 1.11.4 / 3.6.3 / 4.10.3 (apt) | same, **from apt** (checked: files under `/usr/lib/python3/dist-packages`) | = |
| python | psutil / pydot / pyyaml / requests / ruamel.yaml / pytest / empy | 5.9.8 / 1.4.2 / 6.0.1 / 2.31.0 / 0.17.21 / 7.4.4 / 3.3.4 | same (apt) | = |
| python | setuptools | 68.1.2 (apt) | 68.1.2 (apt) | = |
| python | osqp / pyarrow | 1.1.3 / 25.0.1 | 1.1.3 / 25.0.1 | = |
| python | jetson-stats | 4.3.2 (= Orin host jtop service) | 4.3.2 | = |
| python | ultralytics / opencv-python | 8.4.90 / 4.11.0.86 (= Orin Humble) | same | = |
| python | torch / torchvision | not in the Thor reference | 2.8.0 / 0.23.0 (jp6/cu129) | n/a |
| python | **colcon-core** | 0.20.1 | **0.21.1** | differs, 2.2 |
| zed | ZED SDK | 5.4.0 | 5.4.0 | = |
| submodules | ackermann_mux, vesc, teleop_tools, zed-ros2-interfaces, zed_ros2_wrapper | `b3c0b08`, `2848a9f`, `4337558`, `e970087`, `86171bc` | same | = |
| gpu | CUDA / cuDNN / TensorRT | Thor: CUDA 13.0 (JP7) | CUDA 12.9.1 (+ compat, driver API 12.9) / 9.10.2 / 10.3.0.26 | by design (JP6) |
| llama.cpp | commit | Orin Humble host `42532afff` (no sm_87) | `42532af`, sm_87, GCC 13.3 | = commit |
| platform | Ubuntu / Python / glibc / GCC / CMake | 24.04.4 / 3.12.3 / 2.39 / 13.3.0 / 3.28.3 | 24.04 / 3.12.3 / 2.39 / 13.3.0 / 3.28.3 | = |

### 2.2 Every mismatch, explained

1. **colcon tooling and the ROS Python modules.** These come from packages.ros.org but are not `ros-jazzy-*`.

   | Package | Thor | Image |
   |---|---|---|
   | `python3-colcon-core` | 0.20.1 | 0.21.1 |
   | `python3-colcon-cmake` | 0.2.29-100 | 0.2.30 |
   | `python3-catkin-pkg-modules` | 1.1.0 | 1.1.1 |
   | `python3-rosdistro-modules` | 1.0.1 | 1.1.0 |
   | `python3-rospkg-modules` | 1.6.1 | 1.6.2 |

   - The Thor installed them in May–June and its 2026-10-01 upgrade only touched `ros-jazzy-*`.
   - The image takes what the `jazzy/2026-09-11` snapshot carries.
   - The Thor's colcon-core 0.20.1 and colcon-cmake 0.2.29 are **in no snapshot** (checked 2026-06-18 and 2026-09-11), so they cannot be reproduced. Pinning the other three from an older snapshot would mix syncs for no gain.
   - **D3 #2:** if container and Thor test counts differ, the colcon version is a candidate cause, alongside the path effect in section 6.
2. **Ubuntu security updates.** `python3-idna` 3.6-2ubuntu0.2, `python3-pil` 10.2.0-1ubuntu1.3, `python3-requests` 2.31.0+dfsg-1ubuntu1.2 and `python3-urllib3` 2.0.7-1ubuntu0.7 are one Debian revision newer than the Thor's.
   - The upstream version is the same; these are Ubuntu security patches.
   - Ubuntu's archive is not snapshotted. The image takes current `noble-updates`/`noble-security` at build time; pinning old security revisions is worse than this drift.
3. **81 Thor packages are not installed:** rviz, rqt, desktop, demos, examples, turtlesim, the gz vendors, ros2_control and the controllers, depthimage_to_laserscan, cob_srvs. They are GUI, demo or sim, and nothing in the 32 packages needs them: rosdep is clean apart from the `ament_python` key, backlog L16.
4. **The installed `stack_params.yaml` differs from the repo in two values.** `llama_server_path` is `/opt/llama.cpp/bin/llama-server` and `llama_server_cwd` is `/opt/llama.cpp` (decision 2026-10-07).
   - `set_llama_paths.py` refuses unless both keys exist with the Humble defaults, and checks that nothing else changed.
   - The repo keeps the Humble host's path. No test pins these keys.

## 3. Decisions of 2026-10-07: where each one is implemented

| Decision | Implementation | Verified |
|---|---|---|
| Base option C | `docker/orin/Dockerfile` | this report |
| ZED SDK 5.4.0 | `ZED_RUN` default + `inputs.sha256` | header 5.4.0; 300-frame grab |
| cuDNN 9.10.2 | `libcudnn9-cuda-12=9.10.2.21-1` | torch reports cuDNN 91002 |
| cuda-compat kept | unpacked (F1) | driver API 12090 |
| Python order: apt first, pip with constraints, `pip check` + no-PyPI-CUDA fatal | `ros` stage apt list (the six F6 packages included), `py` stage, `build_checks.sh` | build log; scipy 1.11.4 / matplotlib 3.6.3 / jsonschema 4.10.3 from apt |
| F1–F7 | Dockerfile, each marked; F7 documented in the header; `build.sh` refuses on a non-r36.5 host | build checks (F3: no CUDA `libnvjpeg.so`; F5: `/usr/local/zed` group 1005) and `ldd -r` |
| llama.cpp `42532afff`, `-DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87`, Release, `/opt/llama.cpp`; host binary untouched; selected through the existing parameter, no symlinks | `llama` stage; `set_llama_paths.py` on the installed yaml | `ARCHS = 870`; cold start section 5 |
| Persistent `CUDA_CACHE_PATH` on a named volume | `ENV CUDA_CACHE_PATH=/var/cache/f1tenth/cuda` (owner 1004, 1 GiB); D2 mounts the volume there | env dump |
| TensorRT: keep 10.3.0.26, engines rebuilt in the image, the detector refuses a mismatched engine with a clear line | **The guard did not exist** (a mismatched engine gave Ultralytics' `'NoneType' object has no attribute 'create_execution_context'` → passthrough; a missing `tensorrt` triggered Ultralytics' pip install). Added `check_trt_engine()` in `yolo_detector_node.py` (`6011b55`) | 8 new unit tests (43/43 in the file, no new lint findings) + both live cases, section 5 |
| Package count 32 | `phase5d_analysis.md` §4.1, §7 #1 | 32 in the image, 32 on the Thor |
| Thor reference without `~/.local` | `run.sh` v4 → v5 | section 6 |

## 4. Found during D1

Each was caught by a check, fixed in its own commit and rebuilt:

| # | Problem | Cause | Fix |
|---|---|---|---|
| D1-1 | llama.cpp executables failed to link: `compat/libcuda.so.1: undefined reference to NvRmGpuClockSet …` | on `LD_LIBRARY_PATH`, `libcuda.so.1` is the compat driver, whose dependencies are host L4T libraries absent at build time (F7's mechanism, now for the driver) | link against CUDA's driver stub (`stubs/libcuda.so`, same SONAME) in that stage only; at run time the compat driver loads, and the `ldd -r` check proves it resolves (`7420a11`) |
| D1-2 | In a plain `docker run --runtime nvidia` the container had no GPU: `libnvrm_gpu.so` not found, TensorRT import failed | the nvidia runtime injects devices and L4T libs only when `NVIDIA_VISIBLE_DEVICES` is set; NVIDIA's images set it, `ubuntu:24.04` does not (the spike passed `-e`) | `ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all` (`a515f27`) |
| D1-3 | The image user had no `zed`/`jtop` group (`Permission denied` on the ZED headers) | with a numeric `USER 1004:1004` Docker does not apply `/etc/group` memberships | `USER fabiocar`; D2's compose still passes the GIDs (`a515f27`) |
| D1-4 | `jetson-stats` installed as root writes `/etc/systemd/system/jtop.service` and creates `jtop` with the image's next free GID (1006) | its setup script | `jtop` pre-created with the host's 1003; the unit removed (inert without systemd anyway). Same trap as the ZED group (F5) |

## 5. Runtime checks of the image

All through the guard, uid 1004, `--runtime nvidia --network host`.

| Check | Result | Log |
|---|---|---|
| GPU / torch | torch 2.8.0, CUDA 12.9, cuDNN 9.10.2, `cuda.is_available()` True, device Orin; compat driver API 12090 | `d1_image_versions_…a515f27.txt` |
| Entrypoint environment (= the Thor's `jazzy.sh`) | `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`, `ROS_DISCOVERY_SERVER=127.0.0.1:11811`, `FASTRTPS_DEFAULT_PROFILES_FILE=/opt/f1tenth_ws/src/f1tenth_bringup/config/fastdds_profile.xml`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, no `ROS_SUPER_CLIENT` | same |
| **llama-server cold start** (image binary, fresh container, empty JIT cache; stack arguments `-ngl 999`, port 8083, cwd `/opt/llama.cpp`) | **healthy after 3.1 s; first completion 0.38 s**; 37/37 layers on the GPU, `ARCHS = 870`; answers "Rome" / "Paris" as in the spike; steady state 288–363 tok/s prompt, 33.3–33.7 tok/s generation. Spike with the host binary: 52 s + 57 s of PTX JIT | `d1_llama_coldstart_a515f27.log` |
| ZED 2i, SDK 5.4.0 | 300/300 frames, 0 failed, **59.7 Hz**, IMU 300/300 with 300 distinct timestamps, depth valid 300/300 | `d1_zed_grab_a515f27.log` |
| TensorRT, engine built in the image | `trtexec --onnx=yolo26s.onnx --fp16`: PASSED in 497 s, 198 qps (GPU compute 5.03 ms). `check_trt_engine` → accepted; `yolo_detector_node` loads it (80 classes via the sibling `.pt`) and runs inference | console (below) |
| TensorRT, Humble engine (JetPack 10.3.0.30) | `yolo_detector_node` logs `TensorRT engine "…/yolo26s.engine" refused: it was not built by this TensorRT (10.3.0). An engine only loads in the exact TensorRT build that produced it; rebuild it on this machine, e.g. trtexec … Falling back to passthrough mode.` The node comes up; no crash | same |
| Port 8083, UDP 11811, `/dev/shm` | free / free / no Fast DDS leftovers after every run | – |

The engine test's console output was not saved to a file; the run is in this
session's transcript, and it repeats with the command in section 8.

## 6. Thor reference `colcon test` at `083c8e8`

- **Where:** a separate clone `~/dev_ws/f1tenth_p5d_083c8e8` on the Thor; andre's workspace untouched.
- **How:** `colcon build --symlink-install` then `colcon test`, both with `--test-result-base`, under `nice -n 10`.
- **Environment:** `ROS_DOMAIN_ID=87`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, no Discovery Server, the 5 s lease profile, `PYTHONNOUSERSITE=1`.
- **When:** started automatically once the Thor had been quiet for 5 min, at 16:32.

| Run | Python | Tests | Errors | Failures | Skipped |
|---|---|---:|---:|---:|---:|
| v4 (the decision as written) | apt only, `~/.local` hidden | 2937 | **158** | 148 | 143 |
| **v5 (the reference)** | apt + the validated pip-user `osqp` 1.1.3 and `pyarrow` 25.0.1 | **3231** | **0** | **113** | **128** |
| P0 (older commit, for scale) | – | 3147 | – | 107 | 128 |

**Why v5.** `PYTHONNOUSERSITE=1` also hid osqp and pyarrow, which the
validated Thor set has in `~/.local` since 2026-09-30:
- v4 lost all 157 `mpc_controller` tests at setup ("solver='rti' requires the 'osqp' package");
- `f1tenth_logger` failed to collect at all, so none of its tests ran.

v5 keeps `PYTHONNOUSERSITE=1` and exposes exactly those two (plus osqp's
`joblib`) by read-only symlinks in `~/p5d_thor_test_083c8e8/pydeps_validated`.
torch, the PyPI CUDA wheels and `~/.local`'s setuptools stay hidden:
- the run log records setuptools 68.1.2 and `no_user_site 1`, and `import torch` fails;
- osqp 1.1.3 and pyarrow 25.0.1 were confirmed in the same environment.

**Nothing was installed or changed on the Thor.** One mishap on my side:
- v3 died at its first `source` (`set -u` against `/opt/ros/jazzy/setup.bash`) before building anything.
- A broken SSH here-doc ran a stray `rm -rf /pydeps_validated` with an empty variable. That path does not exist; the Thor's state was checked afterwards, with nothing removed.

**v5 per package:**

| Package | Tests | Pass | Errors | Failures | Skipped |
|---|---:|---:|---:|---:|---:|
| ackermann_mux | 90 | 38 | 0 | 43 | 9 |
| corridor_perception | 56 | 37 | 0 | 0 | 19 |
| f1tenth_behavior | 345 | 344 | 0 | 1 | 0 |
| f1tenth_bringup | 100 | 95 | 0 | 5 | 0 |
| f1tenth_control | 5 | 5 | 0 | 0 | 0 |
| f1tenth_costmap | 190 | 190 | 0 | 0 | 0 |
| f1tenth_diagnostics | 164 | 161 | 0 | 3 | 0 |
| f1tenth_localization | 14 | 14 | 0 | 0 | 0 |
| f1tenth_logger | 295 | 290 | 0 | 5 | 0 |
| f1tenth_messages | 5 | 5 | 0 | 0 | 0 |
| f1tenth_navigation | 2 | 2 | 0 | 0 | 0 |
| f1tenth_params | 43 | 43 | 0 | 0 | 0 |
| f1tenth_perception | 540 | 540 | 0 | 0 | 0 |
| joy_teleop | 18 | 16 | 0 | 2 | 0 |
| key_teleop | 3 | 3 | 0 | 0 | 0 |
| llm | 270 | 231 | 0 | 14 | 25 |
| mouse_teleop | 4 | 4 | 0 | 0 | 0 |
| mpc_controller | 716 | 704 | 0 | 11 | 1 |
| teleop_tools_msgs | 125 | 96 | 0 | 0 | 29 |
| vesc_ackermann | 56 | 42 | 0 | 6 | 8 |
| vesc_driver | 67 | 48 | 0 | 6 | 13 |
| vesc_msgs | 5 | 5 | 0 | 0 | 0 |
| vesc_tuning | 16 | 16 | 0 | 0 | 0 |
| zed_components | 76 | 43 | 0 | 10 | 23 |
| zed_debug | 14 | 8 | 0 | 5 | 1 |
| zed_msgs | 7 | 5 | 0 | 2 | 0 |
| zed_ros2 | 5 | 5 | 0 | 0 | 0 |
| **total** | **3231** | **2990** | **0** | **113** | **128** |

(`f1tenth_description`, `f1tenth_hardware`, `teleop_tools`, `vesc` and
`zed_wrapper` have no tests.)

**The 113 failures, grouped** (IDs in `thor_test/v5/nonpass_test_ids.txt`):
- **Lint:** cpplint, flake8, pep257, copyright, uncrustify, lint_cmake. Mostly `ackermann_mux` (40), plus the `vesc_*` and `zed_*` packages and the f1tenth packages' own flake8/pep257/copyright (L9).
- **`llm`:** 12 in `test_intent_go_to` / `test_planner_path` (L10's stubs).
- **`mpc_controller`:** 9 stale-default tests (M12).
- **`f1tenth_behavior`:** M20 `test_grace_then_target_lost`.
- **`joy_teleop`:** `test_parameter_failures` (upstream).
- **`f1tenth_bringup` `test_sim_mode` (2):** genuine at `083c8e8`. The test expects `sim_camera_tf.launch.py` and `zed2i`, which only arrive in `a4488e58`.
- **`f1tenth_logger` `test_robot_logger` (2): path-dependent.**
  - Both assert that the workspace directory found by the upward search is named `f1tenth_more` (`assert 'f1tenth_p5d_083c8e8' == 'f1tenth_more'`).
  - **In the container the workspace is `/opt/f1tenth_ws`, so these two will fail there for the same reason.**
  - D3 #2 counts them as environment-caused; D3 #10 checks whether the logger itself depends on the directory name.

**For D3 #2:**
- `scripts/jazzy_parity/test_results_summary.py` produces this table and the ID list from any result dir, and `--compare A B` diffs two ID lists.
- The container run uses the same environment (domain 87, localhost, no DS, `nice`) and the same command.

## 7. What D2 and D3 need to know

- **D2 compose:**
  - `group_add` must include `zed` 1005 and `jtop` 1003 (the image's GIDs match the host), plus video 44, render 104 and dialout 20.
  - Mount a named volume at `/var/cache/f1tenth/cuda`.
  - The image already sets `NVIDIA_VISIBLE_DEVICES` and runs as `fabiocar`.
  - The entrypoint sources `jazzy.sh`, so `ROS_DISCOVERY_SERVER=127.0.0.1:11811` and domain 0 are the defaults in every container shell, as on the Thor; compose `environment:` overrides them.
- **ZED settings:** the SDK may write calibration files. Mount a copy of `/usr/local/zed/settings` (as the gates did) or the host dir read-write: D2 decision.
- **ZED depth mode:** SDK 5.4 warns that `PERFORMANCE` is deprecated (`zed2_perception.yaml`). It works on the Thor and here; changing it is a Phase 7 question.
- **TensorRT engines:** build them in the image (section 5 command). The Humble `models/yolo26s.engine` will be refused (by design).
- **D3 #2:** see section 6: the logger's two path-dependent tests, the `test_sim_mode` pair, and colcon 0.21.1 vs 0.20.1.

## 8. Files

Committed on `jazzy`:
- `docker/orin/`:
  - `Dockerfile`, `.dockerignore`;
  - `build.sh`, `stage_sources.sh`, `fetch_inputs.sh`, `inputs.sha256`, `keys/ros-snapshots.asc`;
  - `build_checks.sh`, `set_llama_paths.py`, `entrypoint.sh`, `constraints.txt`, `ros_packages.txt`;
  - `image_report.sh`, `image_versions.sh`, `diff_manifest.py`;
  - `lib_guard.sh` (the guard, now shared with `spike/spike.sh`).
- `src/f1tenth_perception/f1tenth_perception/yolo_detector_node.py` and `test/test_yolo_detector_node.py` (the TensorRT guard).
- `scripts/jazzy_parity/test_results_summary.py`.
- `output/phase5d_logs/`:
  - `d1_build_*.log` (including the failed `6011b55` build that found D1-1);
  - `d1_image_{versions,diff,inspect}_*`;
  - `d1_llama_coldstart_a515f27.log`, `d1_zed_grab_a515f27.log`;
  - `thor_test/v5/` (full: run log, env, pip lists, build/test logs, raw results, summary, IDs);
  - `thor_test/v4/` and `v3/` (run logs, summary, IDs).
- `docker/orin/spike/gates/g4_llm.sh`: `LLAMA_CWD` and cold-start timing.

Not committed (size, reproducible from the files kept):
- `thor_test/v4/test_results/` (1.9 MB);
- `thor_test/v4/test_result_verbose.txt` (324 KB);
- `thor_test/v5/test_result_verbose.txt` (648 KB).

Outside the repo:
- the images `f1tenth:jazzy-orin-7420a11` (superseded) and `f1tenth:jazzy-orin-a515f27`, plus the spike's two; the failed `6011b55` build left no tag;
- `/scratch/fabiocar/p5d_build/` (staged sources);
- `/scratch/fabiocar/p5d_spike_ctx/engines/` (the test engine);
- the BuildKit ccache volumes;
- on the Thor: the clone, `~/p5d_thor_test_083c8e8/` (v4, v5, `pydeps_validated` symlinks).

Repeat the TensorRT check (section 5):

```bash
SPIKE_IMAGE=f1tenth:jazzy-orin-a515f27 SPIKE_EXTRA_ARGS="-v <onnx>:/spike/onnx/yolo26s.onnx:ro -v <dir>:/spike/engines" \
  docker/orin/spike/spike.sh run trtexec --onnx=/spike/onnx/yolo26s.onnx --saveEngine=/spike/engines/yolo26s.engine --fp16
```

## 9. Open, for review

- **D2 starts after your review** (compose, `docker_orin.sh`, the env table), per the plan.
- **Size:** dropping the CUDA static libraries would save roughly 2–3 GB. Do it now or leave it?
- **Logger path dependency:** the two `test_robot_logger` tests assert the directory name `f1tenth_more`, which fails in any other checkout, the container included. Fix the tests, fix the code, or run the container's tests from a directory with that name?
- `f1tenth:jazzy-orin-7420a11` and the two spike images can be removed once `a515f27` is accepted (the images share most layers; `docker system df` reports 18.4 GB reclaimable on `/scratch` now).
