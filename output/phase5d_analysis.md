# Phase 5d — D0 analysis: a Jazzy container for the Orin

Date: 2026-10-07. Machine: `jetson-orin` (AGX Orin 64 GB, the car's current
computer). Branch: `jazzy` at `083c8e8`, checked out as a worktree at
`~/dev_ws/f1tenth_jazzy`. The Humble workspace `~/dev_ws/f1tenth_more` and
`/opt/ros/humble` were only read.

**D0 is read-only. Nothing was built and no host setting was changed.** This
file stops for review. The base option has to be picked (section 5), and the
host changes in section 6.4 need approval, before D1 starts.

## 0. Summary

**Blockers. Nothing in D1+ can start until these are cleared:**

1. **Docker is down on the Orin.**
   - `docker.service` failed at this boot (`start-limit-hit`).
   - The cause is not readable without root (`sudo journalctl -u docker -b`).
   - The docker-ce 29.6.0 upgrade and the custom 5.15.185 kernel were installed on the same day (2026-06-23).
   - `fabiocar` is not in the `docker` group.
2. **The Thor version manifest is partial.**
   - The Thor was reachable on Tailscale, but `andre@jetson-thor` rejects the Orin's key.
   - `docker/orin/jazzy_versions.manifest` carries every version the repo documents. The full `dpkg -l 'ros-jazzy-*'` and `pip list` are PENDING, with the capture commands in the file header.

**Decisions for Andreas:**

- Pick the base image (section 5). **Recommendation: C**, Ubuntu 24.04 with the Thor's own apt Jazzy binaries and Python 3.12, plus 22.04-Tegra CUDA 12.9 + cuda-compat. It is the only option that runs the binaries Phases 0–5 validated. It does not fully meet your "only if all available" rule: ZED SDK L4T36.5 on noble is unverified, and CUDA for 24.04 exists only as 22.04 Tegra debs. So C goes ahead only after a 1–2 h spike (D1a) passes four gates. The fallback is **A′**: Stereolabs' r36.5 image + Jazzy from source, Python 3.10.
- Approve the host changes in section 6.4.
- Decide the Humble side of D4 (section 8.2): build `e47e646` in a separate directory, or measure the installed test-campaign build.
- Pick the archive and log directory layout for container runs (section 6.2).
- Decide the nav2 scope (section 4.3).

**Facts that change the plan:**

- **The replay bag is on the Orin.** Every Phase 1–5 replay uses one bag, `humble_obstacle_run`, which the reports place on the Thor only. It is this machine's archive run `~/f1tenth_archive/complete/2026-09-23T15-17-59_mission-llm_2b356aac445f/bag/bag_0.db3`; the sha256 matches (`7f9327a7…`).
- **There is no Thor baseline for the GPU path.** torch is not installed on the Thor (backlog M9 open), YOLO never ran there, and llama-server never ran live there. The D3 GPU and live-LLM checks can only compare against the Orin's own Humble install.
- **The Orin Humble timing numbers were never measured on the Orin.** The reference numbers (MPC period mean 112 ms, worst 309 ms, cores 78–100 %) were computed on the Thor from the recorded bag of that 2026-09-23 mission. The power mode at recording time is unknown. Today the Orin is at MODE_50W with jetson_clocks off.
- **osqp is settled.** The Orin's Humble install runs osqp 1.1.3, the same as the Thor. This closes backlog M6: pin `osqp==1.1.3`.

## 1. Existing Docker material

Every file is identical on `jazzy` and on `main`/`test-campaign`; `git diff` over these paths is empty.

| File | What it does | Reusable for the Orin Jazzy image |
|---|---|---|
| `docker/Dockerfile.cpu` | x86, `ros:humble-ros-base-jammy`, apt deps, pip numpy/scipy/osqp unpinned | apt list as a checklist only |
| `docker/Dockerfile.gpu` | x86, `stereolabs/zed:5.2-runtime-cuda12.8-ubuntu22.04`, CUDA 12.8 toolkit, Humble, `pip install ultralytics` (PyPI torch) | The "use the stereolabs image instead of wget'ing the .run" idea; the numpy<2 + opencv-python<5 single-resolver comment. PyPI torch must not be copied |
| `docker/Dockerfile.jetson-thor` | arm64 22.04 Humble on a JP7 host, CUDA 13.2 SBSA toolkit, ZED 5.2 for L4T 38.4, CPU-only torch | Layer lessons (a)–(m): tzdata first, universe, `safe.directory`, sourcing in `/etc/bash.bashrc` and the entrypoint. Its CUDA/ZED/torch choices are Thor-specific |
| `docker/entrypoint.jetson.sh` | Sources Humble; recreates `/dev/sensors/vesc` from `/dev/ttyACM0`; runs `rosdep install` at every start | Sourcing pattern only. A runtime `rosdep install` breaks reproducibility (it is `-r` and best effort), so it is dropped |
| `.devcontainer/docker-compose.yml` (+ 3 `devcontainer.json`) | Three Humble services. Workspace bind-mounted, `network_mode: host`, `/dev:/dev` + `device_cgroup_rules` (166, 188, 81), no privileged | The `/dev` + cgroup-rule pattern and host networking. Missing `ipc: host`, user mapping and caps |
| `scripts/docker_build.sh`, `scripts/docker_run.sh` | `docker compose build/run` for cpu/gpu | **Broken as written:** they `cd` to the repo root, but the compose file lives in `.devcontainer/`. Humble-only targets |
| `docs/setup/README_DOCKER.md`, `docs/setup/JETSON.md` | Docs for the above (Humble) | Superseded by D2's docs |
| `src/scripts/run_container.sh`, `resume_container.sh` | Legacy F1TENTH foxy container: `sudo docker run --privileged … f1tenth/focal-l4t-foxy` | No |
| `zed_ros2_wrapper/docker/` (submodule `86171bc`) | Stereolabs' own image: `build_jetson.sh --ros-distro jazzy --os jp6.2.2` builds Jazzy **from source** (`ros2.repos`, Tier 3) on `stereolabs/zed:<sdk>-devel-l4t-r36.5`, then the wrapper | **Yes.** A maintained, working recipe for exactly "Jazzy on JP6". It is the basis of option A′ (section 5) |

Everything in the repo's own Docker files is Humble, x86, or Thor. None of it sets `ipc: host`, maps the host UID, adds `SYS_NICE`, mounts the jtop socket, or sets the Discovery Server environment.

## 2. Host facts (Orin, 2026-10-07)

| Item | Value | How measured |
|---|---|---|
| Board | NVIDIA Jetson AGX Orin Developer Kit, 64 GB, 12 cores | `/proc/device-tree/model`, jtop |
| L4T | **R36.5.0** (GCID 43688277, 2026-01-16), `nvidia-l4t-core 36.5.0-20260115194252` | `/etc/nv_tegra_release`, dpkg |
| JetPack | **6.2.2** (`nvidia-jetpack 6.2.2+b24`) | dpkg |
| CUDA | 12.6.85 (`nvcc`), `cuda-toolkit-12-6` | `nvcc --version` |
| cuDNN / TensorRT / VPI | 9.3.0.75 / 10.3.0.30 / 3.2.4 | jtop libraries |
| OS / Python / kernel | Ubuntu 22.04.5, Python 3.10.12, **custom kernel** 5.15.185-tegra (built 2026-05-29, `/boot/Image` replaced 2026-06-23; the initrd is still for 5.15.148) | `uname -a`, `/boot` |
| Power | `nvpmodel` **MODE_50W** (id 3). **jetson_clocks off**: governor `schedutil` on all 12 cores, cpu0 at 806 MHz of a 1497.6 MHz max, GPU at 306 of 816 MHz | `nvpmodel -q`, sysfs. `jetson_clocks --show` needs root |
| RAM / swap | 61 GiB / 30 GiB | `free -h` |
| Disk | `/` 57 GB, 16 GB free (eMMC). **Docker data-root `/scratch/docker-data` on NVMe**: 916 GB, 584 GB free | `df`, `/etc/docker/daemon.json` |
| `/dev/shm` | tmpfs, 31 GB | `df` |
| Docker | docker-ce **29.6.0**, containerd 2.2.5, compose v5.1.4, buildx 0.34.1. **Daemon failed at boot**; `docker.socket` start-limit-hit | dpkg, `systemctl status` |
| Docker access | `fabiocar` not in `docker` (GID 994). Socket is `root:docker 0660`. No passwordless sudo | `id`, `ls -l` |
| `daemon.json` | `data-root: /scratch/docker-data`; runtime `nvidia` registered; **no** `default-runtime` | file |
| nvidia-container-toolkit | 1.16.2 (`libnvidia-container 1.16.2`, `nvidia-container 6.2.2+b24`). Mode `auto` → CSV on Jetson: `drivers.csv` (283 lines: libcuda, nvhost/nvmap libs) + `devices.csv`. **CUDA/cuDNN/TensorRT are not injected; the image must carry them.** CDI dir has only `nvidia-pva.yaml` | `/etc/nvidia-container-runtime/` |
| cgroup | v2 | `stat -fc %T /sys/fs/cgroup` |
| Kernel modules for Docker | `overlay` built in; bridge, veth, br_netfilter, iptable_nat, xt_MASQUERADE, xt_conntrack, xt_addrtype present. **`xt_connmark` is missing** (`CONFIG_…=m`, but no `.ko`); tailscale's health check reports it | `/lib/modules`, `/proc/config.gz` |
| Groups (GID) | dialout 20, video 44, render 104, jtop 1003, zed 1005; `fabiocar` uid/gid 1004 | `getent` |
| jtop | `jetson-stats` 4.3.2 service, `/run/jtop.sock` `root:jtop 0660` | |
| ZED (host, Humble) | **ZED SDK 4.2.5** (pyzed 4.2.5) in `/usr/local/zed`. Calibration `settings/SN36866199.conf` (dir `fabiocar:zed 0770`). `resources/` holds 4.2.5 AI models. ZED 2i on USB (`2b03:f880/f881`), `/dev/hidraw0` (major **510**), `/dev/video0-1` (81). `zed_media_server_cli.service` inactive | |
| udev | `99-vesc.rules`: `ttyACM*` on `KERNELS==1-4.4`, 0483:5740 → `/dev/sensors/vesc` (VESC unplugged now). `99-slabs.rules`: ZED 0666 + no autosuspend. No joystick rule; no joystick plugged | `/etc/udev/rules.d` |
| Humble Python (user site `~/.local`, Python 3.10) | numpy 1.26.4, scipy 1.15.3, **osqp 1.1.3**, pyarrow 25.0.1, torch **2.11.0** (`cp310-linux_aarch64`, CUDA 12.6, `cuda.is_available()` True), torchvision 0.26.0, ultralytics 8.4.90, opencv-python 4.11.0, open3d 0.18.0, jsonschema 4.26.0, ruamel.yaml 0.19.1, matplotlib 3.10.9; apt: psutil 5.9.0, pydot 1.4.2, PyYAML 5.4.1, tensorrt 10.3.0 | import check |
| **Live PyPI CUDA on the host** | `nvidia-cudss-cu12 0.8.0.10` (2026-09-01) plus `~/.local/…/nvidia/{cu12,cublas,cuda_nvrtc}`. `~/.bashrc` prepends `~/.local/…/nvidia/cu12/lib` to `LD_LIBRARY_PATH`. This is the cuDSS shadowing setup the work order warns about, active on the host now (torch 2.11 loads `libcudss.so.0` from it) | `pip list`, `~/.bashrc:157` |
| Humble shell environment | `~/.bashrc`: `source /opt/ros/humble/setup.bash`, `source ~/dev_ws/f1tenth_more/install/setup.bash`, `ROS_DISCOVERY_SERVER=127.0.0.1:11811`, **`ROS_SUPER_CLIENT=TRUE` (global)**, the `LD_LIBRARY_PATH` line above. `ROS_LOCALHOST_ONLY=0`; `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` unset | `~/.bashrc`, `env` |
| How the stack starts | From a shell (no systemd unit for the stack). User units: `f1tenth-archive.service` + `.timer` (rsync of `~/f1tenth_archive/complete` to linus every 15 min) | `systemctl --user` |
| llama.cpp | Binary `/scratch/fabiocar/llama.cpp/build/bin/llama-server`, commit `42532afff`, CUDA build linked to `/usr/local/cuda` 12.6 and the host `libcuda`. Models in `~/projects/llm/models/` (stack default `qwen2.5-3b-instruct-q5_k_m.gguf`) and `/scratch/fabiocar/*.gguf` | `ldd`, `stack_params.yaml:1940`, `models.yaml:26` |
| Humble workspace state | `~/dev_ws/f1tenth_more` on branch `test-campaign` (`1d00853`), symlink-install. **Not** `e47e646`, the commit `jazzy` branched from (section 8.2) | git |
| ROS processes now | none | `ps` |

## 3. Version manifest

`docker/orin/jazzy_versions.manifest` (new, partial). What it pins and why:

- **ROS:** the Thor runs apt Jazzy from `packages.ros.org` noble.
  - Known from the reports: Fast DDS 2.14.6, `rmw_fastrtps_cpp` 8.4.4, robot_localization 3.8.3, slam_toolbox 2.8.5, diagnostic_updater 4.2.7, py_trees and py_trees_ros 2.5.0, rosbag2 0.26.11, cv_bridge 4.1.0.
  - The **apt sync date is PENDING**. It is the sync installed after the 2026-10-01 upgrade.
  - For a source build (options A, A′, B), the reproducible pin is the matching `ros/rosdistro` sync tag (`jazzy/<date>`). Feed it to `rosinstall_generator` through `ROSDISTRO_INDEX_URL`, so every source package lands on exactly the release version the Thor's apt sync carries. That needs the Thor's `dpkg -l` to identify the sync.
- **Python:** numpy 1.26.4, scipy 1.11.4 (Thor apt), osqp 1.1.3, pyarrow 25.0.1. torch, torchvision and ultralytics are not on the Thor. The Orin image takes them from the JP6 index (section 5); the reference for those is the Orin's own Humble set (torch 2.11.0, torchvision 0.26.0, ultralytics 8.4.90). `jetson-stats` must equal the host service, 4.3.2.
- **ZED:** wrapper `86171bc` (`v5.4.0-6`, package 5.4.1). SDK 5.4 matches the Thor's pyzed 5.4; the exact patch is PENDING. The wrapper README still says SDK 5.2 and the migration plan says 5.5. Both are stale against this pin.
- **Submodules:** recorded from `git ls-tree origin/jazzy`.
  - `vesc` (`2848a9f`), `teleop_tools` (`4337558`) and `zed-ros2-interfaces` (`e970087`, tag 5.3.0) are the same commits the Orin's Humble checkout uses.
  - `ackermann_mux` differs: `b3c0b08` (fork, branch `jazzy`) vs `42b7cd7` on Humble. The local Humble object exists; cpp_port_report.md:286-313 asks for it to be checked before any `git pull`.
  - `transport_drivers` is no longer a submodule on `jazzy`; it comes from apt.

## 4. Dependency closure

Computed with `rosdep keys --from-paths <jazzy src> --ignore-src` under a clean environment. The Humble `AMENT_PREFIX_PATH` otherwise hides every key Humble already provides. Submodules are extracted at their `jazzy` commits; `f1tenth_sim`, `f1tenth_more` and `husarion_gz_worlds` are excluded, as on the Thor.

### 4.1 Workspace

34 packages; 32 are built (the Thor skip list removes `f1tenth_sim` and `f1tenth_more`; `husarion_gz_worlds` is not initialised). Corrected 2026-10-07: this said 31, the D1a spike and the Thor's own 10-05 build both build 32. 13 are ament_cmake:
- `ackermann_mux`, `f1tenth_messages`;
- `teleop_tools`, `teleop_tools_msgs`;
- `vesc`, `vesc_driver`, `vesc_msgs`, `vesc_ackermann`;
- `zed_components`, `zed_debug`, `zed_msgs`, `zed_ros2`, `zed_wrapper`.

The rest are ament_python.

### 4.2 ROS keys (105) and what each one is for

| Group | Keys |
|---|---|
| Core / build / test | `ament_cmake`, `ament_cmake_auto`, `ament_python`, `ament_index_python`, `ament_lint_auto`, `ament_lint_common`, `ament_cmake_{copyright,cppcheck,gtest,lint_cmake,pep257,uncrustify,xmllint}`, `ament_{copyright,flake8,pep257,xmllint}`, `rclcpp`, `rclcpp_components`, `rclpy`, `rcutils`, `rcl_interfaces`, `launch`, `launch_ros`, `launch_testing`, `launch_testing_ament_cmake`, `launch_testing_ros`, `ros2launch`, `rosidl_default_generators`, `rosidl_default_runtime`, `rosidl_runtime_py`, `message_filters`, `rosbag2_py`, `test_msgs`, `example_interfaces`, `action_tutorials_interfaces` |
| Messages | `std_msgs`, `std_srvs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `shape_msgs`, `stereo_msgs`, `trajectory_msgs`, `visualization_msgs`, `diagnostic_msgs`, `action_msgs`, `builtin_interfaces`, `rosgraph_msgs`, `tf2_msgs`, `ackermann_msgs`, `control_msgs`, `vision_msgs`, `geographic_msgs`, `nmea_msgs`, `nav2_msgs` |
| TF / description | `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `robot_state_publisher`, `joint_state_publisher`, `xacro`, `zed_description` (not in the workspace: apt on the Thor) |
| Stack runtime | `robot_localization` (also a build dep of `zed_components`), `urg_node`, `foxglove_bridge`, `rosbridge_server`, `topic_tools`, `diagnostic_updater`, `serial_driver` (vesc_driver; brings `io_context`, `asio_cmake_module`), `joy`, `py_trees`, `py_trees_ros`, `backward_ros` (zed) |
| Camera / transport | `cv_bridge`, `image_transport`, `image_transport_plugins`, `compressed_image_transport`, `compressed_depth_image_transport`, `theora_image_transport`, `zstd_image_transport`, `v4l2_camera` (UVC fallback only) |
| Nav2 (declared by `f1tenth_navigation`) | `nav2_map_server`, `nav2_lifecycle_manager`, `nav2_planner`, `nav2_smac_planner`, `nav2_controller`, `nav2_regulated_pure_pursuit_controller`, `nav2_costmap_2d`, `nav2_behaviors`, `nav2_bt_navigator` |
| System | `nlohmann-json-dev`, `graphviz`, `python3-{numpy,scipy,yaml,requests,psutil,pyarrow,pydot,jsonschema,matplotlib,tk,ruamel.yaml,pytest}` |

**Used but not declared in any `package.xml` (they must be added explicitly):**

| Dependency | Used by |
|---|---|
| `slam_toolbox` | `slam.launch.py`. `enable_slam` defaults to true; only `f1tenth_sim` declares it |
| `rosbag2_storage_mcap` | logger default storage `mcap`, `mission_logger.launch.py:50` |
| `fastdds` CLI (part of `fastrtps`) | `ensure_discovery_server.py` |
| `ros2lifecycle`, `ros2bag`, `ros2topic`, `tf2_tools` | the D3 harness |
| `isaac_ros_nitros` | only on the wrapper's NITROS path; not used, skip |

**Python closure** (third-party imports in stack code; tests excluded):
- `numpy`, `scipy`, `osqp`, `cv2`, `torch`, `ultralytics`;
- `pyarrow`, `psutil`, `jsonschema`, `matplotlib`, `pydot`, `yaml`, `ruamel.yaml`, `requests`, `py_trees`;
- `jtop`.

`pyzed` and `open3d` are not imported by any stack code on `jazzy`.

### 4.3 What to build from source

Options A, A′ and B need this; C does not.

Start from `ros_base`-equivalent core (`ros2.repos` minus GUI/rviz/demos/Connext). Then add:
- `robot_localization`, `slam_toolbox`;
- `urg_node` (+`urg_c`, `laser_proc`), `foxglove_bridge`, `rosbridge_suite`, `topic_tools`;
- `diagnostics` (`diagnostic_updater`), `transport_drivers` (`serial_driver`, `io_context`, `asio_cmake_module`, `udp_msgs`);
- `joy` (`teleop_twist_joy` is not needed), `py_trees`, `py_trees_ros` (+`py_trees_ros_interfaces`);
- `vision_opencv` (`cv_bridge`), `image_common`, `image_transport_plugins` (+`zstd`), `v4l2_camera`;
- `xacro`, `robot_state_publisher`, `joint_state_publisher`, `zed-ros2-description`;
- `backward_ros`, `ackermann_msgs`, `control_msgs`, `vision_msgs`, `geographic_info`, `nmea_msgs`, `rosbag2` (+`mcap` storage);
- `nav2_msgs`, plus the nav2 set below.

**Nav2 scope.** `enable_nav2` defaults to **false** (`stack_params.yaml:104-105`); the only nav2 thing the default stack touches is `nav2_msgs` (two behaviour imports).

- The full set declared by `f1tenth_navigation` includes `nav2_smac_planner` → OMPL, plus `behaviortree_cpp` for `bt_navigator`. Building it costs about an extra hour.
- Option 1: build only `nav2_msgs` + `nav2_util`, `nav2_map_server`, `nav2_lifecycle_manager` (cheap). `enable_nav2:=true` then becomes a known gap.
- Option 2: build the full declared set (Thor parity of the installed set).
- **Recommendation: option 1, because enable_nav2 is off.** D1 resolves the exact transitive closure with `rosinstall_generator --deps` against the pinned rosdistro tag, and writes the list into `output/phase5d_image.md`.

## 5. Base image options

Researched 2026-10-07 against NGC, Docker Hub, the CUDA and ROS apt indexes, the jetson-ai-lab wheel index, Stereolabs downloads, jetson-containers and REP 2000.

- **[V]** = checked against a primary source.
- **[U]** = not verified.

Docker is down here, so **no image was pulled or run**. Every runtime claim is [U] until the D1 gates. Sizes are compressed registry sizes.

### 5.1 What exists

**JetPack 6 userspace images:**
- `nvcr.io/nvidia/l4t-jetpack` exists only for r36.2.0, r36.3.0 and **r36.4.0** (5.61 GB, 2024-11). There is **no r36.5 tag** [V].
- `l4t-cuda` has no 12.6 devel tag [V].
- The only r36.5 JetPack userspace image is Stereolabs': `stereolabs/zed:5.4.1-{devel,runtime,py-devel}-l4t-r36.5` (5.83 GB, 2026-07-23) [V].
  - Built `FROM ubuntu:22.04` with the full `nvidia-jetpack` r36.5 from NVIDIA's apt repo, then ZED SDK 5.4.1 (`silent skip_tools skip_drivers`).
  - `5.5.x` tags exist too.

**ZED SDK:** `ZED_SDK_Tegra_L4T36.5_v5.4.1.zstd.run` and the 36.4 equivalent exist [V]. The installer requires CUDA 12.x (`REQUIRED_CUDA_VERSION=12.6`).

**ROS on JP6:**
- `dustynv/ros` has only `jazzy-{ros-base,desktop}-r36.4.0-cu128-24.04` (2025-03, 19 months old) [V].
- Jazzy on Ubuntu 22.04 arm64 is **not listed in REP 2000 at all**; Jammy is Tier 3 for amd64 only [V].
- **The Thor's apt packages are available for noble arm64 at the same versions** [V]: robot_localization 3.8.3, slam_toolbox 2.8.5, nav2 1.3.13, foxglove-bridge 3.6.0, transport_drivers 1.2.0, rosbag2 + mcap 0.26.11, cv_bridge 4.1.0, zed-msgs 5.3.0.
- Pinnable snapshots: `snapshots.ros.org/jazzy/2026-09-11` (apt) and the rosdistro tags `jazzy/2026-09-11` and `jazzy/2026-10-05` (source) [V].

**CUDA for Jetson on Ubuntu 24.04:** there is **no official package** for CUDA 12.6/12.8/12.9 [V]. The `ubuntu2404/arm64` repo carries only cuDNN 9.x, cuDSS and the container toolkit.
- jetson-containers' published 24.04/r36 images install the **22.04 Tegra CUDA debs into a 24.04 image**, with `cuda-compat` on `LD_LIBRARY_PATH` [V, code].
- NVIDIA's CUDA-for-Tegra appnote supports 12.8/12.9 upgrade (compat) packages on JP6 [V].
- cuDNN 9.3 (CUDA 12.6) for 24.04 Tegra returns 404 [V].

**TensorRT:** the L4T tarballs `10.3.0.26 (cu12.6)` and `10.7.0.23 (cu12.8)` include **cp312** wheels [V].

**torch from jetson-ai-lab (JP6):**
- `jp6/cu126` has **cp310 only**: torch 2.8.0 to 2.11.0 and torchvision 0.23.0 to 0.26.0 [V].
- **The only cp312 torch on JP6 is torch 2.8.0 on `jp6/cu129`**, with torchvision 0.23.0 cp312 [V].
- torch 2.9.1 to 2.11.0 link `libcudss.so.0`; **2.8.0 does not** [V, from the wheels' ELF dependencies].
- Non-PyPI cuDSS: `libcudss0-cuda-12=0.8.0.10-1` from NVIDIA's `ubuntu2204/arm64` CUDA repo [V]. It installs to `/usr/lib/aarch64-linux-gnu/libcudss/12/` and adds no `ld.so.conf` entry.

**pyzed 5.x** requires `numpy>=2` [V]. No stack code imports pyzed (section 4), so the image installs the SDK with `skip_python`, and that conflict does not arise.

### 5.2 Comparison

| | **A** l4t-jetpack r36.4.0 + Jazzy source | **A′** stereolabs/zed 5.4.1 l4t-r36.5 + Jazzy source | **B** jetson-containers | **C** ubuntu:24.04 + apt Jazzy + 22.04-Tegra CUDA |
|---|---|---|---|---|
| Userspace / Python | 22.04 / **3.10** | 22.04 / **3.10** | 22.04 → Humble only; 24.04 → 3.12 | **24.04 / 3.12** (= Thor) |
| ROS | ros2.repos + rosinstall_generator, pinned to `release-jazzy-*` + rosdistro `jazzy/2026-09-11` | same | source, **unpinned** (current rosdistro, ament HEAD); no custom-repos input | **apt binaries**, `snapshots.ros.org/jazzy/2026-09-11`: same packages and build as the Thor (if the Thor is on that sync) |
| Platform support for Jazzy | arm64 Jammy not in REP 2000 | same | same on 22.04; source on 24.04 | Noble arm64: **Tier 1** |
| CUDA / cuDNN / TRT | 12.6 / 9 / 10.3 from the image (r36.4 userspace on an r36.5 host) | **r36.5 JetPack set**, identical to the host | 22.04 Tegra debs (12.6/12.8/12.9) + compat | CUDA **12.9** (22.04 Tegra debs) + `cuda-compat-12-9`; cuDNN 9 (24.04 repo); TRT 10.3 tarball (cp312) |
| ZED SDK 5.4.1 | `.run` for L4T36.4 | **preinstalled**, r36.5 | `.run` | `.run` L4T36.5 **on noble: [U]**. Stereolabs ships no 24.04+r36 image |
| torch | 2.11.0 cp310 cu126 + apt cuDSS (= Orin Humble) | same | cp310 or cp312 | **2.8.0 cp312 cu129**, no cuDSS (≠ Orin Humble 2.11) |
| Build time on this Orin | 2–3 h+ at MODE_50W [U] | same [U] | 0.5–1 h (cache) / 2–4 h [U] | **20–40 min**, mostly downloads [U] |
| Image size | ~8–10 GB [U] | ~7.5–9 GB [U] | 5.5 GB published | ~7–9 GB [U] |
| Closeness to the Thor | ROS versions pinnable, but Python 3.10, gcc 11, CMake 3.22, a different build of every ROS package | same as A | not pinnable as shipped | **Same apt binaries, Python 3.12, numpy 1.26.4 (apt), cv_bridge 4.1.0 / OpenCV 4.6.0**; differs only in the CUDA/torch stack (which differs on every option, since the Thor is JP7/CUDA 13 without torch) |
| Main risk | Jazzy-on-22.04 build issues: EmPy 4, cv_bridge/Boost 1.74, `gz_*_vendor`, `rosidl_generator_dds_idl`; foxglove-sdk downloads at configure time; NVIDIA has no r36.5 tag | the same build issues; 5.8 GB base | stale, unpinned, mixes apt and source | **three unverified gates** (5.4) |

### 5.3 Parity risks of Python 3.10 (A, A′)

- **rclpy and the generated message modules are a different build.** Every ROS package is built from source by us rather than taken from the binaries the Thor ran. P3's Py3.8/NumPy 1.17 preview stayed WITHIN OSQP TOLERANCE, so numerics are likely fine. "Same package versions" holds only at the source level.
- **numpy:** Jammy's apt numpy is 1.21.5. Pin pip numpy 1.26.4 before building cv_bridge.
- **scipy:** the Thor runs 1.11.4 and the Orin Humble 1.15.3. scipy 1.18.x needs Python ≥3.12 and numpy ≥2. Pin the Thor's 1.11.4 on both sides.
- **ultralytics** pulls in `opencv-python` beside the system cv2: 4.5.4 on Jammy vs 4.6.0 on Noble. Pin `opencv-python<5` in the same resolver call as numpy (existing Dockerfile lesson).
- **Language and stdlib differences:** 3.12-only stdlib and syntax used anywhere on `jazzy` (`tomllib`, `ExceptionGroup`, `itertools.batched`, `typing.override`, PEP 695/701 f-strings) would break on 3.10. In the other direction, `distutils` and `imp` exist only on 3.10.
- **Build tooling:** colcon, setuptools and ament_python test discovery differ by Ubuntu release. This is the same area as CLAUDE.md's `extras_require` trap, and a skip-count risk for D3 #2.
- **CMake 3.22 vs 3.28 and gcc 11 vs 13** for the 13 CMake packages.

### 5.4 Recommendation: C, gated by a short feasibility spike; A′ as the fallback

**C is the only option that runs the Thor's own ROS binaries and its Python.** Everything Phases 0–5 validated would be the same binaries, Python 3.12, numpy 1.26.4 and cv_bridge 4.1.0. D3's parity claims stay meaningful, and the build takes tens of minutes, not hours.

**Your rule for C ("only if CUDA 12.x for Jetson, the ZED SDK for L4T r36 and JP6 cp312 torch are all actually available") is only partly met:**

| Requirement | Status |
|---|---|
| JP6 cp312 torch | **Available**: exactly one, torch 2.8.0 + torchvision 0.23.0 on `jp6/cu129`. It forces CUDA 12.9 in the image, and differs from the Orin Humble torch 2.11 |
| CUDA 12.x for Jetson on 24.04 | **Not officially packaged.** Usable as 22.04 Tegra debs + cuda-compat, which is how jetson-containers builds its published r36/24.04 images. NVIDIA supports the 12.9 compat package on JP6 |
| ZED SDK L4T r36 on noble | **Unverified.** The installer targets 22.04 and Stereolabs ships no such combination |

So I do not recommend C outright. I recommend **D1a, a throwaway spike image of about 1–2 h** (after Docker works), before the real D1. C goes ahead only if all of these pass on this host:

- **G1, CUDA:** `ubuntu:24.04` + CUDA 12.9 Tegra debs + `cuda-compat-12-9` on the r36.5 host's mounted driver.
  - `torch.cuda.is_available()`.
  - A GPU matmul.
  - `yolo26s-seg.pt` inference on a stored image, compared with the Orin Humble torch 2.11 output.
- **G2, TensorRT:** the TRT 10.3.0.26 tarball's cp312 wheel imports and builds a small engine.
- **G3, ZED:**
  - `ZED_SDK_Tegra_L4T36.5_v5.4.1.zstd.run` installs on noble (`skip_python skip_tools skip_drivers`).
  - `ldd -r` on `libsl_zed.so` is clean.
  - The SDK reports 5.4.1.
  - `zed_components` from `86171bc` builds and loads.
  - The camera is **not** opened (that is D6).
- **G4, LLM:** the host's `llama-server` (CUDA 12.6 build, mounted) runs against the image's 12.9 libraries and answers `/completion`. Otherwise rebuild llama.cpp `42532afff` in the image and point `llama_server_path` at it, through the configurable mount.

If G3 fails (the likely failure point), fall back to **A′**:
- `stereolabs/zed:5.4.1-devel-l4t-r36.5`.
- Jazzy from `release-jazzy-*` plus `rosinstall_generator` pinned to `jazzy/2026-09-11` (or the Thor's sync).
- torch 2.11.0 cp310 cu126 and `libcudss0-cuda-12 0.8.0.10` from apt.
- Python 3.10 recorded as a deviation from the Thor, with the 5.3 risks checked in D3.

If G1 or G2 fail, C is out for the same reason. **A** loses to A′ everywhere, because NVIDIA has no r36.5 tag. **B** is not reproducible as shipped.

The apt sync that matches the Thor is still PENDING. The Thor's Fast DDS build stamp (2026-09-02) and its 2026-10-01 upgrade point to `jazzy/2026-09-11` (inference). The Thor's `dpkg -l` confirms or corrects that before D1.

## 6. Runtime design (D2) and host changes

### 6.1 Principles

1. **Same paths inside and outside.**
   - The container runs as `fabiocar` (uid/gid 1004) with `HOME=/home/fabiocar`.
   - Every host path the stack reads is mounted at the same path: `~/f1tenth_archive`, `~/.ros`, `~/projects/llm/models`, `/scratch/fabiocar/llama.cpp`, `/usr/local/zed/settings`.
   - Then `stack_params.yaml` needs no container-specific values: `llama_server_path`, `mission_logger_runs_dir: ~/f1tenth_archive` and `models.yaml`'s `~/projects/…` resolve unchanged. Files written are owned by fabiocar, never root.
2. **The workspace is baked into the image** (D1 `ws` stage, `--merge-install`, Release), not bind-mounted. The image is then one reproducible thing tied to a git sha. An optional `compose.dev.yaml` override can bind the worktree's `src/` for iteration; D3/D4 numbers come only from the baked image.
3. **Environment parity with the Thor:** the container sets what `scripts/env/jazzy.sh` sets and nothing more. The Orin's Humble `~/.bashrc` sets two things the Thor does not:
   - a global `ROS_SUPER_CLIENT=TRUE`;
   - the PyPI-CUDA `LD_LIBRARY_PATH`.
   Neither is carried into the container.

### 6.2 `docker/orin/compose.yaml` (sketch, written in D2)

```yaml
services:
  jazzy:
    image: f1tenth:jazzy-orin-${GIT_SHA}
    build: {network: host}      # REQUIRED: dockerd runs bridge:none (B1), see below
    container_name: f1tenth_jazzy_orin
    runtime: nvidia
    network_mode: host          # lidar (192.168.0.10), DS on 127.0.0.1:11811, linus over Tailscale
    ipc: host                   # host /dev/shm (31 GB tmpfs): Fast DDS SHM as on bare metal
    # pid: private (default), see 6.3
    user: "1004:1004"
    group_add: ["20", "44", "104", "1003", "1005"]   # dialout video render jtop zed (resolved at up time)
    cap_add: [SYS_NICE]
    ulimits: {rtprio: {soft: 99, hard: 99}, memlock: {soft: -1, hard: -1}}
    # no cpuset/cpus: taskset prefixes need all 12 cores visible (plan:363-369)
    volumes:
      - /dev:/dev               # hot-plug: VESC/joystick appear after start (same pattern as .devcontainer)
      - /home/fabiocar/f1tenth_archive:/home/fabiocar/f1tenth_archive   # or a separate dir, see decision below
      - /home/fabiocar/.ros:/home/fabiocar/.ros                         # or ~/.ros_jazzy, see decision below
      - /usr/local/zed/settings:/usr/local/zed/settings                 # SN36866199.conf
      - zed_resources:/usr/local/zed/resources                          # SDK 5.4 models, NOT the host's 4.2.5 ones; persisted
      - /run/jtop.sock:/run/jtop.sock
      - /scratch/fabiocar/llama.cpp:/scratch/fabiocar/llama.cpp:ro      # ${LLAMA_DIR}, configurable
      - /home/fabiocar/projects/llm/models:/home/fabiocar/projects/llm/models:ro   # ${LLM_MODELS_DIR}
      - /etc/localtime:/etc/localtime:ro
    device_cgroup_rules:
      - 'c 166:* rmw'   # ttyACM (VESC)
      - 'c 188:* rmw'   # ttyUSB
      - 'c 189:* rmw'   # usb_device (/dev/bus/usb, ZED)
      - 'c 510:* rmw'   # hidraw (ZED IMU/HID), dynamic major: docker_orin.sh reads it from /proc/devices
      - 'c 81:* rmw'    # video4linux
      - 'c 13:* rmw'    # input (joystick js*/event*)
    environment:
      RMW_IMPLEMENTATION: rmw_fastrtps_cpp
      ROS_DOMAIN_ID: "0"
      ROS_DISCOVERY_SERVER: 127.0.0.1:11811
      FASTRTPS_DEFAULT_PROFILES_FILE: /opt/f1tenth_ws/install/share/f1tenth_bringup/config/fastdds_profile.xml
      ZED_DIR: /usr/local/zed
      HOME: /home/fabiocar
      NVIDIA_VISIBLE_DEVICES: all
      NVIDIA_DRIVER_CAPABILITIES: all
```

**No privileged mode.** ZED access is `/dev/bus/usb` + hidraw + video cgroup rules; the host's `99-slabs.rules` already makes the ZED nodes 0666. Hot-plugged devices appear because `/dev` is bind-mounted, and the udev symlink `/dev/sensors/vesc` → `ttyACM0` resolves inside. D2 verifies this before considering `--privileged`.

**Decision: where container runs write.**

- (a) **Same `~/f1tenth_archive` and `~/.ros` as Humble.**
  - Container runs are synced to linus by the host timer, as today.
  - Humble and Jazzy runs share one archive (each run records its params and code version).
- (b) **Separate host dirs** `~/jazzy_orin/{archive,ros}` mounted at the same in-container paths.
  - Nothing mixes, and the D4 comparison stays clean.
  - Container runs are not synced unless the timer gets a second source.
- **Recommendation: (b) for D3/D4.** Revisit for the car.

**Inside the container there is no user systemd.** `mission_logger_node.py:919` calls `systemctl --user --no-block start f1tenth-archive.service` after every run. In the container that call fails, and the host timer's 15-min catch-up covers option (a). D2 makes sure the failure is logged, not fatal; FB/P4 already saw it as a no-op on the Thor.

**Host networking is mandatory, not a preference (B1, cleared 2026-10-07).** The Orin's kernel cannot load `bridge.ko` (CLAUDE.md "Orin Docker", backlog M21), so dockerd runs with `bridge: none`, `iptables: false`, `ip6tables: false`. A `docker build` without `--network=host` fails at its first `RUN` (`network bridge not found`); a `docker run` without `--network host` starts silently with only `lo` and no DNS. Every container and every build uses host networking: compose `network_mode: host` + `build: {network: host}`, and `docker_orin.sh` passes `--network=host` to every `docker build`/`docker run` it issues and refuses an override.

**`scripts/docker_orin.sh up|down|shell|status`:**
- uses host networking for every build and container (above);
- runs the guard (6.3) before `up`/`shell`;
- resolves group GIDs and the hidraw major from the host;
- exports `GIT_SHA`;
- `status` prints the container state plus the guard's view of the host.

### 6.3 The guard (hard rule 2)

`scripts/orin_stack_guard.sh [--for jazzy|humble]` is called by `docker_orin.sh up/shell`, by every D3/D4 harness wrapper, and by the Humble-side D4 wrapper with `--for humble`. It refuses (exit 1, says why) if:

1. **A host ROS process exists.** It scans `/proc/*/cmdline` and `/proc/*/exe`, never `pgrep -f`, which matches its own shell (CLAUDE.md "Harness gotchas"). Matches: `/opt/ros/humble`, `~/dev_ws/f1tenth_more/install`, `component_supervisor_node`, `_ros2_daemon`, `fastdds`/`fast-discovery-server`, `ros2 launch|run|bag`, `llama-server`.
2. **UDP 11811 is bound by anyone** (`ss -Hulpn 'sport = :11811'`). `ensure_discovery_server.py` would otherwise silently reuse a Humble server.
3. **A `/dev/shm/*fastrtps*` file is mapped by a live process** (scan `/proc/*/maps`). Stale files are fine; the supervisor sweeps them.
4. For `--for humble`: the `f1tenth_jazzy_orin` container is running.
5. For `--for jazzy`: a container from any other `f1tenth:*` image is running.

**`pid:` stays private.**

- The guard is what keeps the two stacks apart.
- `pid: host` would let the supervisor's SHM sweep and `phase5_bringup.sh`'s precheck see host processes. It would also let `kill_ros2.py` in the container kill host processes.
- From the host, `pidstat`/`check_cpu_pinning.py` see container processes either way.

### 6.4 Host-side changes needed (for approval)

| # | Change | Why | Reversible |
|---|---|---|---|
| H1 | **Done 2026-10-07** (Andreas): `daemon.json` `bridge: none`, `iptables: false`, `ip6tables: false`; root cause the kernel/modules mismatch (backlog M21). Original text: find out why `dockerd` fails (`sudo journalctl -u docker -b`) and fix it. Most likely candidates, unconfirmed: the custom kernel's netfilter set (missing `xt_connmark`), or a daemon.json/containerd 2.2 issue. A fix may mean `daemon.json` `"iptables": false` + `"bridge": "none"` (all our containers use host networking), or loading a module | Nothing works without it | yes |
| H2 | **Done 2026-10-07**: `fabiocar` is in `docker`. Original text: Docker access for `fabiocar`: `usermod -aG docker fabiocar` (root-equivalent), or every docker call through `sudo` | Builds run in the background from this account | yes |
| H3 | **No** `default-runtime: nvidia`. No build step needs the GPU (TensorRT engines and ZED model optimisation happen at run time on the device) | – | – |
| H4 | **No** nvidia-container-toolkit change. 1.16.2 is registered and its CSV mode (host `libcuda` + device nodes) is the JP6 mechanism for every option. Option C's `cuda-compat-12-9` lives inside the image | – | – |
| H5 | Optional, only if you want faster builds: `nvpmodel -m 0` (MAXN) during D1, then back to MODE_50W. **D4 runs both stacks at whatever mode you choose, recorded** | MODE_50W caps the CPU at 1.5 GHz | yes |

No change to udev rules, `/opt/ros/humble`, the Humble workspace, host Python or the jtop service is needed.

### 6.5 Environment: Thor native, Orin container, Orin Humble

| Variable | Thor (native Jazzy) | Orin container (proposed) | Orin Humble native (today) |
|---|---|---|---|
| ROS setup | `source /opt/ros/jazzy/setup.bash`. The proposal to source `scripts/env/jazzy.sh` from `.bashrc` (`output/phase5/env_proposal/bashrc.diff`) is unconfirmed: capture | entrypoint + `/etc/bash.bashrc`: `/opt/ros/jazzy` (or the source-build prefix) + `/opt/f1tenth_ws/install` | `/opt/ros/humble` + `~/dev_ws/f1tenth_more/install` |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` (jazzy.sh:45) | same | unset (default Fast DDS) |
| `ROS_DOMAIN_ID` | `${ROS_DOMAIN_ID:-0}` | 0 | unset (0) |
| `ROS_DISCOVERY_SERVER` | `127.0.0.1:11811` via `F1TENTH_DISCOVERY_SERVER` | same | `127.0.0.1:11811` (`.bashrc`) |
| `ROS_SUPER_CLIENT` | **not exported**; scoped per launch file + `ros2cli()` | same as the Thor | **`TRUE` globally** (`.bashrc`) |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | `<ws>/src/f1tenth_bringup/config/fastdds_profile.xml` (jazzy.sh:39); the launch files set it for the stack | the installed share path | unset (the profile is Jazzy-branch work) |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | SUBNET (default) | SUBNET | n/a (Humble: `ROS_LOCALHOST_ONLY=0`) |
| `ZED_DIR` | `/usr/local/zed` | same | – |
| `LD_LIBRARY_PATH` extra | none | none | `~/.local/…/nvidia/cu12/lib` (PyPI cuDSS) |
| Launch-time variables | `COMPONENT_SUPERVISOR_LOCK`, `MISSION_LOGGER_LOCK`, `F1TENTH_RUNS_DIR`, `F1TENTH_MORE_ROOT`, `F1TENTH_ROBOT_NAME`, `F1TENTH_LLM_MODEL` unset by default | same: unset unless a harness sets them | same |

D2 produces the definitive table from `tr '\0' '\n' < /proc/<supervisor pid>/environ` on each side (P5 decision 3).

## 7. Parity test list for D3

Inputs that are on the Orin:
- the reference bag (sha above);
- the frozen `.npz` inputs and Thor outputs in the repo (`output/phase2/boundary_*.npz`, `output/phase3/mpc_*.npz`);
- the metrics JSONs with the Thor Jazzy×Jazzy floors.

The Thor's raw Jazzy run bags (`output/phase*/runs/`, `inputs/`) are gitignored and stay on the Thor. Input bags are rebuilt here with `filter_bag_for_layer.py` and checked against the digests in the ORIN_* instructions.

| # | Test | Command (repo `scripts/jazzy_parity/` unless noted) | Pass criterion | Thor reference |
|---|---|---|---|---|
| 1 | Build + ABI | `colcon build --merge-install` (Release); `ldd -r` scan from CLAUDE.md | 32/32 built (was 31/31, a miscount); scan prints nothing | 13 CMake pkgs, empty scan (FB2, P5) |
| 2 | `colcon test`, all packages | `colcon test --test-result-base <isolated dir>` | **Same failing test IDs as the Thor at `083c8e8`:** lint failures, mpc_controller's 10 stale defaults, llm's 12 `_note_llm_sent` stubs, M20 `test_grace_then_target_lost`. Every skip difference explained by environment: archive mounted, osqp, torch + `.pt`, llama-server :8083 | P0: 3147 tests / 107 fail / 128 skip, at an **older commit**, no per-package totals. A Thor rerun at `083c8e8` is needed for an exact count (section 8.1) |
| 3 | Launch smoke | `phase5_bringup.sh step4`, `cycles 10`, `hold` (5 min) | 32 nodes up; rates within the Thor table ±5 % (`/odometry/filtered` 48.1, `/ekf_global` 48.0, `/tf` 105.9, `/costmap/boundaries` 20.1, `/drive` 10.0 Hz); 0 crashes; 0 "Matching unexisting participant"; logger records as a super client; **after the 10 cycles and after `hold`, no duplicate node processes** (one process per node name, no process whose parent is gone; decided 2026-10-07 after the Thor's orphaned `ekf_cost_observer_node`, backlog M23) | P5 tables |
| 4 | EKF replays | `replay_localization.sh relay` / `ekf_global` ×3; `compare_runs.py` | relay bit-exact; ekf_global steady state (t ≥ 20 s) RMS within a small multiple of the floor | relay 0.0 m; pos 0.0044 m / yaw 0.020° (floor 0.0008–0.0014 m / 0.0016–0.0027°) |
| 5 | SLAM / semantic / boundary replays | `replay_localization.sh` with `slam`, `semantic_layer`, `costmap_boundary` ×3 | slam: TF silence, pose within a floor multiple, map agreement ≈96 %; semantic bit-exact; boundary medians ≈ Thor | slam 96.08–96.25 %; boundary angle median 0.52°, offset 7.4 mm, front 17 mm |
| 6 | Boundary frozen | `run_boundary_frozen.py` on `boundary_frozen_inputs.npz`, `compare_boundary_frozen.py` vs `boundary_thor_outputs.npz` | bit-identical, or float-only with **0 decision flips** | Thor self-consistent; smallest margins 2.6e-6 m / 7.4e-7 rad |
| 7 | MPC frozen | `run_mpc_frozen.py`, `compare_mpc_frozen.py` vs `mpc_thor_outputs.npz` | IDENTICAL or WITHIN OSQP TOLERANCE, statuses equal 419/419 | Thor = live, IDENTICAL 419/419 |
| 8 | MPC replays | `replay_localization.sh mpc` ×3 + capture; `compare_runs.py mpc --recorded-humble`; `mpc_timing.py` | P3 acceptance rule: steering/speed RMS, p95 and max ≤ 1.5 × F; status disagreements ≤ F's count | floor 0.019 / 0.041 rad |
| 9 | BT replays | `replay_localization.sh bt` ×3, `bt_metrics.py` | same `/mpc/goal_drive`, status sequence, `move_outcome`, tree sequence; stop delay within 0.1 s | RUNNING 39.506 s, hold 42.160 s, stop delay 0.2125–0.2145 s |
| 10 | Logger + extract | `logger_check.sh WORK SRC mcap` and `sqlite3`; `mission_extract`/`mission_render`, `compare_extracts.py` | all checks pass; extract 7,667 rows for the reference bag; frame hashes identical | P4, FB1 |
| 11 | Diagnostics | `diag_start_check.sh`; `ekf_cost_observer` replay; FB1 `system_observer_soak.sh` | all designed paths; `system_observer` reads GPU through `/run/jtop.sock` (not 0.0) | P4, FB1 |
| 12 | LLM translation | `llm_golden_check.py` (10 fixtures) + **live**: llama-server **built in the image** (`/opt/llama.cpp`, commit `42532afff`, sm_87; decided 2026-10-07), same model, same prompts → mission JSON | golden 10/10 sha-identical; live missions equal to the Orin Humble live run (no Thor live reference exists); cold-start time reported | golden 10/10 on py3.12 and py3.10 |
| 13 | GPU path | `torch.cuda.is_available()`; YOLO (`yolo26s-seg.pt`) on stored images; `zed_components` loads + `sl::Camera::getSDKVersion()` (no camera open) | CUDA true; detections vs Orin Humble torch 2.11 (**decided 2026-10-07**): objects matched one-to-one, box IoU ≥ 0.99, mask IoU ≥ 0.99, \|Δconf\| ≤ 2e-3; a detection present on one side only counts as agreement only if its conf on the other side is within 2e-3 of the 0.30 threshold. This residual is a torch 2.8 (image) vs 2.11 (host) effect that option C cannot remove. ZED SDK 5.4.0 loads | **No Thor reference** (M9) |
| 14 | Discovery / lease / watchdog | `ds_probe.sh`; `fix_batch_5/lease_probe.py --profile`; `fix_batch_5/watchdog_live.sh` | plain client sees nothing, super client everything; lease removal 4.4–4.7 s; watchdog 0 false triggers; FAILING/OK/FAILED timings ≈ Thor; **no duplicate node processes after the watchdog restarts** (backlog M23) | FB5 |
| 15 | Pinning in the container | `scripts/check_cpu_pinning.py` while `hold` runs; `chrt`/`nice` with negative values inside | every pinned thread on its cores; `SYS_NICE` effective | – |

**Harness adaptations D3 needs:**
- `phase5_bringup.sh:40` matches `/opt/ros/jazzy/lib/` (fine only if the image installs there).
- `run_batches.sh:21` hard-codes `cd ~/dev_ws/f1tenth_more`.
- The harness needs `bc`, `iproute2`, `procps`, `stress`/`stress-ng` and `sqlite3` in the image.
- The P1–P4 Thor runs predate the 5 s lease profile; a container shell that sources the env adds it. Note it, don't hide it.

## 8. D4 plan (same-Orin comparison)

### 8.1 Measurements

| Item | Both sides | Thor column |
|---|---|---|
| MPC solve time and control-loop period, live mission load and replay (mean, p95, p99, max, >110 / >150 ms) | `mpc_timing.py` on the replays plus a live-load window | P3 tables (idle / 14×90 % / 28×100 %) |
| Boundary/costmap timing, BT replays | as D3 #5/#9 | P2/P4 |
| CPU per core and per node, memory, GPU | `pidstat -t -p <pid> 1`, `/proc/stat` per core, jtop | full stack 20.7–21.7 % of 14 cores; per core and memory not recorded |
| Lease + watchdog CPU under full load | `fix_batch_5/run_batches.sh BATCHES="proof stress dl_after dl_wd"` | FB5 table (supervisor 0.44 cores fed) |
| osqp version and settings in effect | print from inside the running MPC process | 1.1.3, eps 1e-3, max_iter 200 |
| "Matching unexisting participant" count | grep over the runs | 0 in 311 files |
| Power mode | `nvpmodel -q`, clocks, recorded in every run dir | – |

### 8.2 Humble side: which code

The work order compares Humble bare-metal against the Jazzy container on the same Orin. The installed Humble build is `test-campaign` (`1d00853`), not `e47e646`. The ORIN_* instructions (`output/phase{2,3,4}/ORIN_*_INSTRUCTIONS.md`) all check `HEAD == e47e646`.

- **Recommendation:**
  - Build `e47e646` in a **separate** directory (`~/dev_ws/f1tenth_humble_e47e646`, `colcon build` against the untouched `/opt/ros/humble`).
  - That adds a directory; it changes no existing install, Python package or setting.
  - Then run the Humble side of D4 from it.
- **Alternative:** measure the installed test-campaign build and state the code difference next to every number.

**Your call (hard rule 1).**

## 9. Premises in the work order that do not match the repo

1. **"plan §0.1 / §0.2"** are not in `output/jazzy_migration_plan.md`, which has no section 0.
   - The in-repo equivalents are plan Decision 7 (target Jetson) and `backlog.md:74-85` ("Migration runs still pending on the Orin").
   - D4 folds in that backlog list: boundary/MPC/BT replays, osqp, environment, lease + watchdog under load, isolation rate, H3 deadline overrun.
2. **"Same pass/skip/fail counts as on the Thor."** The only full Thor count is P0's (3147/107/128) at an older commit, with no per-package totals. Exact parity needs a Thor `colcon test` at `083c8e8`, and skips depend on mounted data, models and llama-server.
3. **GPU and LLM parity "with the Thor".** Not possible: no torch, YOLO, ZED or live llama-server on the Thor. The reference is the Orin's Humble install.
4. **"Humble MPC period mean 112 ms, worst 309 ms, cores 78–100 %".**
   - These come from post-processing the 2026-09-23 bag on the Thor, under live mission load (GPU up to 99 %).
   - The power mode then is unknown.
   - A replay on the Orin is a different load. D4 reports both a replay and, if wanted, a live-load-equivalent run, and labels them.
5. **"ZED SDK 5.4.0".** The wrapper pin is 5.4.1 (`86171bc`) and the Thor's pyzed was 5.4.x; the exact patch is pending. The host's SDK is 4.2.5: its `resources/` must not be mounted into the container, its `settings/` can be.
6. **"Never reinstall a pip cuda-toolkit".** Agreed for the image. Note that the host's Humble torch currently **depends on** the PyPI `nvidia-cudss-cu12` and a `.bashrc` `LD_LIBRARY_PATH` override. The D4 Humble numbers run with that in place; nothing here changes it.

## 10. What D1 will do once options and approvals are in

- If C is picked: **D1a spike first** (section 5.4 gates G1–G4). It is a throwaway image under `docker/orin/spike/`, with a log in `output/phase5d_logs/`. Its result decides C or A′, and D1 proper starts after you see it.
- `docker/orin/Dockerfile`: multi-stage `base` → `ros` → `deps` → `ws`, built to the pinned manifest.
- `.dockerignore`: `bags/`, `output/`, `log/`, `build/`, `install/`, `first_test_campaing/`, `**/__pycache__`.
- ccache mount; build log in `output/phase5d_logs/`, run in the background.
- Tag `f1tenth:jazzy-orin-<git-sha>`.
- `output/phase5d_image.md` with the resolved versions diffed against the manifest.
