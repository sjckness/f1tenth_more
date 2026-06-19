# Running on Jetson AGX Thor (arm64)

The `f1tenth_jetson` container brings the f1tenth_more workspace up on a
**Jetson AGX Thor running JetPack 7.2**. To match the rest of the workspace the
container stays on **Ubuntu 22.04 + ROS 2 Humble** even though the Thor host is
Ubuntu 24.04. It is built **natively on the Thor itself — no QEMU emulation.**

- Dockerfile: [docker/Dockerfile.jetson-thor](docker/Dockerfile.jetson-thor)
- Entrypoint: [docker/entrypoint.jetson.sh](docker/entrypoint.jetson.sh)
- Compose service: `f1tenth_jetson` in [.devcontainer/docker-compose.yml](.devcontainer/docker-compose.yml) (guarded by the `jetson` profile)
- Dev Container: [.devcontainer/jetson-thor/devcontainer.json](.devcontainer/jetson-thor/devcontainer.json)

## Build & run

```bash
# On the Thor (arm64), from the repo root:
docker compose --profile jetson build          # builds docker/Dockerfile.jetson-thor natively
docker compose --profile jetson up -d
docker compose --profile jetson exec f1tenth_jetson bash

# First time inside the container: build the workspace
colcon build --symlink-install
source install/setup.bash
```

The `jetson` profile keeps `docker compose up` on the x86_64 cpu/gpu machines
unchanged — the arm64 image is only touched when you pass `--profile jetson`.

Or open **[.devcontainer/jetson-thor/devcontainer.json](.devcontainer/jetson-thor/devcontainer.json)**
in VS Code on the Thor (Dev Containers: Reopen in Container).

## What's baked into the image

- Full **CUDA 13.2 toolkit** (`cuda-toolkit-13-2`, NVIDIA SBSA arm64 repo) — for
  building `zed_components`. The runtime driver comes from the host L4T stack.
- **ZED SDK 5.2.3** (`ZED_SDK_Tegra_L4T38.4_v5.2.3`, pinned to match the
  `zed_ros2_wrapper` submodule — do **not** bump it independently).
- ROS 2 Humble (ros-base) + Gazebo Fortress + every workspace apt dependency.
- `transport_drivers` (io_context / serial_driver / udp_driver / asio_cmake_module)
  is vendored in `src/` and built by colcon — no external source overlay needed.

## ⚠️ 22.04-container-on-24.04-host caveat

The ROS-Humble + CPU side of the workspace builds and runs fine. The CUDA
**toolkit** (nvcc/headers) is for *building*; the CUDA **runtime** driver
(`libcuda.so`) and the L4T-38.4-built ZED SDK are compiled against the host's
glibc 2.39 (Ubuntu 24.04). Loading them inside this glibc-2.35 (22.04) container
is the known "old ROS distro on new Tegra" risk. If GPU/ZED **runtime** fails to
load, the fix is a 24.04 base — not a change in the Dockerfile.

## Hardware

Same model as the cpu/gpu containers: host `/dev` is bind-mounted and char-device
access is granted via `device_cgroup_rules` (no `runtime: nvidia` — Thor's Tegra
iGPU is reached through `/dev`, not the discrete-GPU runtime). The container
starts whether or not the VESC / LiDAR / ZED / GPS are connected.

## Verification (run once inside the container after first boot)

```bash
bash -lc '
echo "ROS_DISTRO=$ROS_DISTRO" && [ "$ROS_DISTRO" = humble ] && echo "[OK] ROS sourced" || { echo "[FAIL] ROS not sourced"; exit 1; }
command -v nvcc >/dev/null && echo "[OK] nvcc: $(nvcc --version | tail -1)" || { echo "[FAIL] nvcc not on PATH"; exit 1; }
ros2 pkg prefix serial_driver >/dev/null 2>&1 && echo "[OK] transport_drivers built (serial_driver found)" || echo "[note] serial_driver not yet built — run colcon build first"
cd /f1tenth_more && colcon build --symlink-install && echo "[OK] colcon build succeeded" || { echo "[FAIL] colcon build failed"; exit 1; }
'
```

> Note: `transport_drivers` is built by colcon (vendored in `src/`), so there is
> no separate `/opt/ros_external_install` overlay to check — its presence is
> confirmed by the workspace build itself.
