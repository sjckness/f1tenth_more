# F1TENTH Docker Dev Environments

Two container configurations, both mirroring the Jetson Orin AGX software
environment (**Ubuntu 22.04 + ROS 2 Humble**):

| Service          | Machine                         | GPU | Extras                          |
|------------------|---------------------------------|-----|---------------------------------|
| `f1tenth_cpu`    | ThinkPad T15 Gen 2 (x86_64)     | no  | lean ROS only                   |
| `f1tenth_gpu`    | Desktop, RTX 5050 (Blackwell)   | yes | CUDA 12.8 + ZED SDK + Ultralytics |
| `f1tenth_jetson` | Jetson AGX Thor (arm64, JP 7.2) | yes | CUDA 13.2 + ZED SDK 5.2 — see [JETSON.md](JETSON.md) |

The workspace (`.`) is **bind-mounted** at `/f1tenth_more` in both containers —
source is never copied into the image, so edits on the host are live inside.

---

## 1. Prerequisites

### CPU machine (ThinkPad)
- Docker Engine + the Compose plugin. That's it (no GPU stack).

### GPU machine (RTX 5050 desktop)
- Docker Engine + Compose plugin
- Recent NVIDIA driver supporting **Blackwell / CUDA 12.8** (sm_120)
- **nvidia-container-toolkit** (exposes the GPU to containers):

```bash
# nvidia-container-toolkit one-liner (Ubuntu)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
  && sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit \
  && sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
```

Verify: `docker run --rm --runtime nvidia --gpus all nvidia/cuda:12.8.0-runtime-ubuntu22.04 nvidia-smi`

---

## 2. First-time setup

```bash
# 1) Build the image for your machine
./scripts/docker_build.sh cpu      # ThinkPad
./scripts/docker_build.sh gpu      # RTX 5050 desktop
# (or ./scripts/docker_build.sh all)

# 2) Enter the container
./scripts/docker_run.sh cpu        # or: gpu

# 3) Inside the container (first time only): resolve deps + build the workspace
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`rosdep` walks every `package.xml` under `src/` (including the submodules) and
installs anything not already baked into the image.

> When using the VS Code **Dev Containers** extension, opening the folder uses
> `.devcontainer/devcontainer.json`, which targets the **CPU** service as a safe
> default. To develop against the GPU image, run `./scripts/docker_run.sh gpu`
> directly (or change the `service` field in the devcontainer file).

---

## 3. Running nodes

ROS is sourced automatically by the container entrypoint. After building once,
source the overlay (`source install/setup.bash`) in each new shell.

```bash
# Full stack bringup (VESC + EKF + MPC + startup steer sweep + camera + YOLO).
# camera_source selects the (mutually exclusive) camera: 'zed' (default) or 'webcam'.
ros2 launch f1tenth_bringup stack_bringup_launch.py camera_source:=zed
ros2 launch f1tenth_bringup stack_bringup_launch.py camera_source:=webcam

# Perception only (Hokuyo LiDAR + ZED2 + YOLO; ZED2 needs the GPU container)
ros2 launch f1tenth_perception perception.launch.py
```

One-liner via the run helper (runs the command, then exits):

```bash
./scripts/docker_run.sh gpu "ros2 launch f1tenth_bringup stack_bringup_launch.py"
```

> **Note on package names:** this branch (`new-structure`) renamed the old
> `f1tenth_stack` package to **`f1tenth_bringup`**, and there is no
> `sensors_stack` — sensor/camera bringup lives in **`f1tenth_perception`**.

---

## 4. Hardware access

- **USB peripherals (VESC, serial adapters, cameras):** the host `/dev` is
  bind-mounted into the container (`volumes: - /dev:/dev`) and char-device
  access is granted via `device_cgroup_rules` (majors `166` ttyACM, `188`
  ttyUSB, `81` video4linux). No specific device file is pinned, so:
    - the container **always starts**, even with nothing plugged in;
    - devices plugged in **after** start are visible live, with no restart.
  The host's udev rules still own enumeration/symlinks. Privileged mode is
  intentionally **not** used — the cgroup rules are sufficient.
- **Hokuyo LiDAR (Ethernet, 192.168.0.10:10940):** works because both services
  use `network_mode: host`. No extra config needed.
- **ZED2 (USB3):** **GPU container only** — it needs the ZED SDK + CUDA, which
  are not installed in the CPU image. `perception.launch.py` on the CPU
  container will start the Hokuyo + YOLO node but the ZED wrapper will not run.

---

## 5. Foxglove

The Foxglove bridge is exposed on port **8765** (host networking). Open
[Foxglove Studio](https://app.foxglove.dev/) (or the desktop app) and connect to:

```
ws://localhost:8765
```

---

## 6. RTX 5050 / Blackwell (sm_120) warning  ⚠️

The RTX 5050 is **Blackwell** (compute capability **sm_120**). Two pieces must
support it, and **both may need newer-than-default builds**:

1. **PyTorch** (pulled in by `ultralytics`): the default wheel may lack sm_120
   kernels. If GPU inference throws *"no kernel image is available for execution
   on the device"*, install the nightly CUDA 12.8 build inside the container:
   ```bash
   pip install --pre torch torchvision --index-url https://download.pytorch.org/whl/nightly/cu128
   ```

2. **ZED SDK**: the Dockerfile downloads the *latest cu12 / ubuntu22* installer
   via the `ZED_SDK_URL` build arg. **This URL must be verified before building**
   — confirm the resolved version supports CUDA 12.8 + Blackwell (>= 4.2.5; the
   `zed_ros2_wrapper` submodule is pinned to `humble-v4.2.5`). Override with a
   pinned installer if needed:
   ```bash
   docker compose build --build-arg ZED_SDK_URL=<verified .run url> f1tenth_gpu
   ```

---

## 7. Notes / Jetson parity

- No first-party `package.xml` dependency is Jetson-only: every ROS dep
  (`urg_node`, `robot_localization`, `vision_msgs`, `nav2-*`, `rosbridge-server`,
  `cv_bridge`, etc.) has an x86 `ros-humble-*` apt binary, all installed in the
  images.
- The CPU image deliberately omits CUDA / ZED / torch to stay lean.
- The existing `src/scripts/*.sh` helpers are Jetson (l4t, `--runtime nvidia`)
  artifacts and are unrelated to these x86 containers.
