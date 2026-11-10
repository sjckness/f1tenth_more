# Hydra scene graph in the Isaac HIL setup

Hydra (MIT-SPARK) builds a 3D scene graph from the simulated ZED 2i. Everything here is behind
flags: with no flags the stack, Isaac and the logger behave exactly as before.

Branch `hydra-hil`, base `jazzy-doors` 698faa14.

## 1. Camera: the ZED 2i contract

Hydra reads only the topics in `src/f1tenth_hydra/config/camera_zed2i.yaml`. That file is the
contract, copied from the real car's config. The Isaac sim (`~/isaac_ws`, not in this branch)
already publishes it under the same names.

| | Real car (ZED 2i, wrapper) | Isaac sim (`~/isaac_ws` + `sim_isaac.sh`) |
|---|---|---|
| Model / name | zed2i, camera_name `zed2` | stereo pair "ZED2-like", same names |
| Resolution | 640x360 (HD720 grab, `pub_downscale_factor` 2) | 640x360 |
| Rate | 30 Hz configured, **~13.7 Hz delivered** (bag) | 15 Hz (render every 4th 60 Hz physics step) |
| Intrinsics | fx=fy 262.428, cx 318.742, cy 179.294, rectified, D = 0 | same K (focal length derived from fx), D = 0 |
| FOV | 101.3 x 68.9 deg | same (from K) |
| Baseline | 0.12 m | 0.12 m |
| Depth | PERFORMANCE, 0.3-10 m, 32FC1 m, -inf/+inf, stabilization 0 | render depth clipped 0.3-10 m, same encoding and conventions, noise off |
| RGB encoding | bgra8 | bgra8 |
| Mount (base_link -> zed2_camera_link) | (0.2399, 0, 0.205), pitch -2.083 deg (deployed TF) | same: `sim_isaac.sh` sets lens centre (0.22936, 0, 0.21963) and pitch -0.03635, the lens midpoint of that mount |
| Frames | zed2_camera_link, _center, _left_camera_frame; optical `zed2_left_camera_frame_optical` (v5.4.1, Jazzy) | same, optical `zed2_left_camera_optical_frame` (v4 name, as the stack's sim TF) |
| RGB topic | `/camera/image_raw` + `/camera/camera_info` (remap of `rgb/color/rect/image`) | same (also `/zed2/zed_node/left/image_rect_color`) |
| Depth topic | `/zed2/zed_node/depth/depth_registered` + `depth/camera_info` | same |
| QoS | reliable | reliable, depth 5 |
| GT semantics | none | `/sim/ground_truth/left/semantic`, mono8 class ids, same render product as RGB; **off by default** (`sensors.stereo.semantic.enabled`) |
| GT pose | none | `/sim/ground_truth/odom` (map_sim -> base_link, 50 Hz) |

Two differences are deliberate and stay. The rate is 15 Hz in the sim, against the car's 13.7 Hz
delivered (30 configured). The optical frame name differs by wrapper version; Hydra takes it from
the `camera_info` header, so neither name is hard-coded.

### What the Isaac side must publish (hil-isaac)

Nothing new beyond what `~/isaac_ws` already has:

- the topics above, under these names;
- GT semantics: `--set sensors.stereo.semantic.enabled=true`. `sim_isaac.sh` adds it when
  `hydra:=true seg_source:=gt`;
- class ids from the scene's `semantics:` block. `f1tenth_hydra/config/labels/isaac_to_ade20k_mp3d.yaml`
  maps class names to Hydra labels. Office labels only `floor` and `wall` today. Adding patterns to
  `environment.labels` in `office.yaml` (door, chair, table, ...) gives Hydra objects in GT
  mode. Every name in that list already has a Hydra label.

## 2. Hydra

### Version and how it is built

- **Version.** Hydra-ROS `main` 8f3b7e3 (2026-01-12) and Hydra `main` 2e58a35 (2026-01-14),
  plus their 9 dependencies. The heads of the branches in Hydra-ROS's `install/ros2.yaml`
  were read on 2026-11-10 and pinned by SHA in
  `src/f1tenth_hydra/config/hydra/hydra_jazzy.repos`.
- **Sources: submodules in this repo.** The 11 repositories are separate git submodules under
  `third_party/hydra_ros/`, each at the commit the `.repos` names (`third_party/hydra_ros/README.md`).
  The `.repos` file stays the single source of truth for the version;
  `scripts/hydra/check_hydra_pins.py` fails if a submodule SHA or URL diverges from it, and
  `build_hydra_ws.sh` runs that check before building. Nothing is cloned at build time.
  `third_party/` carries a `COLCON_IGNORE`, so the stack's own `colcon build` never builds Hydra
  or GTSAM, and a clone **without** `--recurse-submodules` builds and runs the stack unchanged --
  only Hydra itself is missing. Get them with
  `git submodule update --init third_party/hydra_ros` (non-recursive on purpose: `semantic_inference`
  has a nested `Mask2Former` submodule that the closed-set path does not need).
- **Jazzy support.** Upstream tests Ubuntu 24.04 + Jazzy and Iron. The README still calls the
  ROS 2 port "in active development". Jazzy is the newest distro they name.
- **Dependencies.** GTSAM (`ros-jazzy-gtsam`), PCL, OpenCV, glog/gflags, zmq and nanoflann come
  from rosdep. TensorRT + CUDA are needed only for `seg_source:=mobilenet` (semantic_inference).
  The spark_dsg Python bindings are built from the same pinned source with pip `--target`.
- **Native vs Docker.** Native on the workstation. The sources are in this repo (above), the
  build and install space is its own workspace, `~/hydra_ws` (`--ws DIR`), outside the repo so
  that the stack's `install/` never mixes with Hydra's. Reasons: the stack and RViz are native Jazzy, a Docker Hydra would only add a DDS
  boundary, and upstream's `docker/` targets their own datasets. The f1tenth workspace builds and
  runs without Hydra. `f1tenth_hydra` finds Hydra at launch time.
- **Segmentation.** semantic_inference's closed-set node with **MobileNetV2-dilated C1 deepsup
  (ADE20K)**, `ade20k-mobilenetv2dilated-c1_deepsup.onnx`. The TensorRT engine is built from the
  ONNX on the first run, on the GPU that runs it. Labels are grouped to `ade20k_mp3d`, the same
  label space the GT map uses. I found no record of the 8/11 investigation on this machine, so the
  model is semantic_inference's own MobileNetV2 export. Check that it is the one 8/11 used.

```bash
# once, after the sudo steps at the end of this file
git submodule update --init third_party/hydra_ros   # the pinned Hydra sources (about 81 MB)
scripts/hydra/check_hydra_pins.py            # submodule SHAs == hydra_jazzy.repos
scripts/hydra/build_hydra_ws.sh --models     # --check first lists what is missing
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select f1tenth_hydra f1tenth_logger f1tenth_params
source scripts/hydra/hydra_env.sh            # ROS + ~/hydra_ws + this workspace + spark_dsg

# smoke test: Hydra runs, seg_source:=gt + isaac_gt on a synthetic ZED/GT feed -> non-empty graph
python3 -m pytest src/f1tenth_hydra/test/test_hydra_smoke.py -v -s
```

### Flags

`hydra.launch.py` (package `f1tenth_hydra`) is never part of `supervisor_bringup`.
`scripts/sim/sim_isaac.sh` starts it only with `hydra:=true`, on the host it is given to.

| flag | values | default |
|---|---|---|
| `hydra:=` (sim_isaac.sh) | true / false | false |
| `hydra_profile:=` | workstation_full / jetson_min | workstation_full |
| `seg_source:=` | gt (Isaac GT ids, label map) / mobilenet (TensorRT) | the profile's |
| `pose_source:=` | isaac_gt (`/sim/ground_truth/odom` -> TF `map_sim -> hydra_gt_base_link` + deployed mount) / stack_tf (the stack's `odom -> base_link` and camera TF) | isaac_gt |

Frames: Hydra's map frame is `hydra_map`, never `map`. With `stack_tf` Hydra does not publish
TF, so the stack's `odom` keeps one parent. With `isaac_gt` it publishes `hydra_map -> map_sim`,
a separate tree. Loop closures run only with `stack_tf`, because GT pose cannot drift.

Topics: `/hydra/backend/dsg` (graph, hydra_msgs/DsgUpdate), `/hydra/backend/dsg_mesh` (mesh),
`/hydra/status` (JSON heartbeat). With mobilenet also `/hydra_seg/semantic*/image_raw`. On exit
(SIGINT) Hydra writes the graph, the timing data and the logs to `~/.hydra/f1tenth/<profile>_<stamp>/`.

Print the exact Hydra command line without Hydra installed:
`python3 -m f1tenth_hydra.hydra_args --profile workstation_full --seg-source gt`.

## 3. Profiles (`hydra_profile:=`)

YAML only: `src/f1tenth_hydra/config/profiles/<name>.yaml` (what runs) plus
`config/hydra/<name>.yaml` (Hydra's own parameters; diff the two Hydra files to see every value).

| | workstation_full | jetson_min |
|---|---|---|
| Where | Isaac workstation | Orin (Jazzy container); workstation only views |
| Voxel / truncation | 0.05 m / 0.15 m | 0.10 m / 0.30 m (Hydra stock) |
| Range | 0.3-10 m (all ZED depth) | 0.3-5 m |
| Active window | 15 m | 8 m |
| Layers | mesh, objects, mesh places, GVD places, rooms, buildings, agents | same layers (rooms need GVD places), coarser |
| Graph topic | <= 2 Hz | <= 1 Hz |
| Mesh topic | yes (`/hydra/backend/dsg_mesh`, 0.5 Hz) | no |
| Viz sinks, visualizer, RViz | yes | no |
| Segmentation | gt (default) or MobileNetV2 every frame | MobileNetV2 (default) at ~3 Hz |
| Loop closures | with `stack_tf` | off |

### Run commands

The `sim_isaac.sh` options (`--net`, `--scene`, ...) are unchanged.

```bash
# workstation_full: Isaac + Hydra on the workstation, the stack on the Orin (link)
scripts/sim/sim_isaac.sh isaac --net eth --scene office --gui hydra:=true          # gt, isaac_gt
scripts/sim/sim_isaac.sh isaac --net eth --scene office hydra:=true seg_source:=mobilenet
# on the Orin, as before:
scripts/sim/sim_isaac.sh stack --net eth

# workstation_full, everything on one host (Isaac, stack, Hydra)
scripts/sim/sim_isaac.sh all --scene office hydra:=true

# jetson_min: Hydra next to the stack on the Orin, the workstation only views
scripts/sim/sim_isaac.sh isaac --net eth --scene office seg_source:=gt   # only if Hydra uses gt
scripts/sim/sim_isaac.sh stack --net eth hydra:=true hydra_profile:=jetson_min   # on the Orin
scripts/sim/sim_isaac.sh run --net eth ros2 launch f1tenth_hydra hydra_viewer.launch.py  # workstation

# Hydra alone, against whatever publishes the topics (any profile):
source scripts/hydra/hydra_env.sh
ros2 launch f1tenth_hydra hydra.launch.py hydra_profile:=workstation_full seg_source:=gt
```

### Orin (jetson_min): build and run steps (not done yet)

Hydra has **not** been built for aarch64. The Orin's Jazzy image (`docker/orin`, Ubuntu 24.04,
Jazzy snapshot 2026-09-11, CUDA 12.9, TensorRT 10.3) has what semantic_inference needs. Steps:

1. Derive a Hydra image from the car image, on the Orin. Use host networking for both the build
   and the run (CLAUDE.md "Orin Docker"). The apt packages must come from the image's own Jazzy
   snapshot (CLAUDE.md "Rebuild policy"). The sources are **copied from the submodules**, so the
   image gets exactly the pinned commits and the build needs no network of its own:
   ```dockerfile
   FROM f1tenth:jazzy-orin-<sha>
   USER root
   RUN apt-get update && apt-get install -y ros-jazzy-gtsam \
       libgoogle-glog-dev libgflags-dev libzmq3-dev nanoflann-dev && rm -rf /var/lib/apt/lists/*
   COPY third_party/hydra_ros /opt/hydra_src
   RUN . /opt/ros/jazzy/setup.sh && rosdep install --from-paths /opt/hydra_src --ignore-src -r -y \
    && mkdir -p /opt/hydra_ws && cd /opt/hydra_ws \
    && MAKEFLAGS=-j6 colcon build --base-paths /opt/hydra_src --merge-install \
         --cmake-args -DCMAKE_BUILD_TYPE=Release \
         --packages-up-to hydra_ros semantic_inference_ros \
    && python3 -m pip install --break-system-packages --no-deps \
         --target /opt/hydra_ws/install/spark_dsg_python /opt/hydra_src/spark_dsg
   ```
   `docker build --network=host -t f1tenth:hydra-orin-<sha> .` from the repo root (the build
   context must contain `third_party/hydra_ros`, so run `git submodule update --init
   third_party/hydra_ros` on the Orin first, or send the sources in a bundle the way
   `docker/orin/build.sh` does). `COPY` drops the submodules' `.git` files, which is what we
   want in an image; the version is the one recorded in the commit being built. Plan for about
   2 GB of RAM per make job. The Orin has 16 GB shared with the GPU, so keep `-j6` or lower.

   **aarch64 notes.** Nothing here is x86-only, but none of it has been compiled for aarch64 yet:
   - GTSAM comes from `ros-jazzy-gtsam`, which exists for arm64 in the Jazzy snapshot the image
     uses; `teaser_plusplus` and `spatial_hash` are header-heavy CMake packages and are the
     likeliest to need a flag (`-DBUILD_TEASER_FPFH=OFF` is already off by default).
   - `semantic_inference` needs TensorRT 10.3 + CUDA 12.9 from the image, not from pip.
   - Build on the Orin itself (native), not with qemu: the build is CPU-bound and qemu is
     roughly 10x slower.
   - The submodules are the same commits on both architectures, so the x86 workstation and the
     Orin run the same Hydra version (binaries differ, see CLAUDE.md on aarch64 parity).
2. Model and engine: copy `ade20k-mobilenetv2dilated-c1_deepsup.onnx` to `~/.semantic_inference`
   and bind-mount it. The TensorRT engine is built on the Orin at the first run (minutes on
   the Orin), then cached next to the ONNX.
3. Run it inside the stack container, or as a sibling `docker run --network host` with the same
   `ROS_DOMAIN_ID` and Discovery Server:
   `ros2 launch f1tenth_hydra hydra.launch.py hydra_profile:=jetson_min pose_source:=stack_tf use_sim_time:=false`
   on the car, or `sim_isaac.sh stack ... hydra:=true hydra_profile:=jetson_min` in HIL.
   `pose_source:=isaac_gt` only works in HIL, where `/sim/ground_truth/odom` crosses the link.
4. In HIL with jetson_min, Hydra on the Orin subscribes to the raw RGB, depth (and with gt the
   labels) from Isaac over the link. RGB and depth are the streams the stack already takes; the
   labels add about 230 kB per frame. Isaac's RTF already drops when raw images cross the link
   (`output/sim_next/decisions.md` S19), so measure with `hydra_cost:=true` before judging the
   graph.

## 4. Viewers (workstation)

- **Hydra's own visualizer** (`hydra_visualizer`, part of Hydra-ROS) plus RViz with
  `src/f1tenth_hydra/rviz/hydra_hil.rviz`. It shows the live backend mesh
  (`/hydra/backend/dsg_mesh`, workstation_full only), the scene graph layers
  (`/hydra_visualizer/graph`), agent poses, the RGB image and the semantics. The fixed frame is
  `hydra_map`. `hydra.launch.py` starts it when the profile says so. On its own (e.g. viewing a
  jetson_min Hydra on the Orin):
  `ros2 launch f1tenth_hydra hydra_viewer.launch.py` (`viewer:=rviz|custom|both`).
- **Custom Python viewer** `hydra_dsg_viewer` (spark_dsg + Open3D, matplotlib fallback). It reads
  the graph topic only, so it also works with jetson_min, and draws nodes, edges and labels.
  Layers are lifted by `--z-offset` (3 m). Object colour comes from the semantic label; object,
  room and building nodes get text labels.
  ```bash
  ros2 run f1tenth_hydra hydra_dsg_viewer                     # live /hydra/backend/dsg
  ros2 run f1tenth_hydra hydra_dsg_viewer --summary           # one graph: layer counts, exit
  ros2 run f1tenth_hydra hydra_dsg_viewer --file G.sparkdsg   # a saved graph
  ```
  Open3D is optional (`pip install --user open3d`); without it the viewer uses matplotlib.

With `pose_source:=stack_tf` Hydra publishes no TF, so `hydra.launch.py` anchors `hydra_map` to
`odom` with an identity static TF. The graph then shows under `map` too. Hydra's own
loop-closure correction is not shown by this anchor.

## 5. Cost analyzer (`tools/hydra_cost/`)

`hydra_cost:=true` (on `hydra.launch.py` or `sim_isaac.sh`) starts `tools/hydra_cost/hydra_cost.py`
next to Hydra. It runs alone too: `python3 tools/hydra_cost/hydra_cost.py [--duration S]`.
It samples at 1 Hz and writes one long CSV row per metric
(`t_wall, t_rel, kind, entity, metric, value`):

- CPU per process and its top 8 threads (`/proc` thread names), RSS, thread count. Processes
  matched: Hydra, segmentation, visualizer, RViz, the Isaac sim, the logger.
- System CPU and RAM. GPU util and VRAM per GPU and VRAM (+ SM % with NVML) per process:
  `pynvml` (`pip install --user nvidia-ml-py`), else `nvidia-smi`.
- Rate and kB/s of the camera, depth, labels, graph, mesh and TF topics.
  `--no-image-topics` when Hydra runs on another host.
- Graph size per layer (nodes, edges) and graph message size, from `/hydra/backend/dsg`.
- Hydra's heartbeat (`/hydra/status`). At exit, Hydra's own `timing_stats.csv` from its log dir.

Output: `~/.ros/hydra_cost/<stamp>/samples.csv` + `summary.md` (rewritten every 60 s and at exit).
`~/.ros/hydra_cost/latest` points to the newest run.

## 6. Logger: scene graphs in the run folder

`f1tenth_logger`'s mission logger takes the parameter `log_scene_graph`, default **false**. The
logger itself stays off by default (`enable_mission_logger`). When both are on, each mission run
folder gets:

```
<run>/scene_graph/dsg_t00010.sparkdsg, dsg_t00020.sparkdsg, ...   every scene_graph_period_sec (10 s)
<run>/scene_graph/dsg_final.sparkdsg                              at mission end
<run>/scene_graph/hydra_cost/{samples.csv,summary.md}             if hydra_cost runs on this host
```

Launch arguments (`mission_logger.launch.py`, defaults in `stack_params.yaml`):
`mission_logger_log_scene_graph:=true`, `mission_logger_scene_graph_period_sec:=10.0`. Under
the supervisor in sim, add them to `config/sim/isaac.yaml` ->
`component_supervisor_node.sim_component_args.diagnostics`. The logger needs the Hydra
workspace sourced (hydra_msgs, spark_dsg). Without the spark_dsg bindings it writes the raw
graph bytes (`dsg_*.bin`) instead. With nothing Hydra-related sourced it logs one error and
records the run as usual.

Open a saved graph:

```bash
scripts/hydra/open_graph.sh ~/f1tenth_sim_archive/complete/<run_id>        # dsg_final
scripts/hydra/open_graph.sh <run>/scene_graph/dsg_t00120.sparkdsg --summary
scripts/hydra/open_graph.sh ~/.hydra/f1tenth/<profile>_<stamp>             # Hydra's own output
```

## 7. Keyboard teleop (`keyboard:=true`)

`tools/isaac_keyboard/isaac_keyboard.py` is one self-contained module. **Inside the Isaac
process** it reads the workstation keyboard with `carb.input` while the Isaac window has focus.
It publishes `ackermann_msgs/AckermannDriveStamped` on `/teleop`, the ackermann_mux
`joystick` lane (priority 100, timeout 0.2 s). The Orin's mux receives it over the link. The
BT's `safety_stop` lane (200) still wins. If `carb.input` has no keyboard (headless Isaac) it
falls back to evdev. evdev reads the keyboard **whatever window has focus**, and the module logs
this loudly.

Keys: W/Up forward, S/Down reverse, A/Left and D/Right steer (full lock while held), E/PgUp
faster, Q/PgDn slower (0.4 to 1.0 m/s in 0.1 steps), Space stop (latched until every drive key
is released).

Safety (unit-tested, `tools/isaac_keyboard/test_isaac_keyboard.py`):
- Dead-man: the command is zero unless the latest input sample, no older than 0.3 s, holds a
  drive key. Releasing a key stops the car on the next tick. Input that stops arriving stops it
  within 0.3 s, and an Isaac process that stops publishing hits the mux timeout (0.2 s).
- When idle it keeps publishing zero, so the teleop lane holds the car instead of handing it back
  to the MPC. To let autonomy drive, use `keyboard:=false`, or `hold_when_idle: false` in the
  config below.
- Speed is 0 or at least 0.4 m/s, never in between. Steering is at most +0.319 / -0.309 rad
  (stack_params `mpc_steering_angle_max/min_rad`, the servo envelope).

### The Isaac-side hook (hil-isaac / `~/isaac_ws`, not in this branch)

`sim_isaac.sh isaac ... keyboard:=true` passes `--set ros_interface.keyboard.enabled=true`.
Isaac refuses unknown keys, so these 2 additions must be in place first:

`config/ros_interface.yaml`:
```yaml
keyboard:
  enabled: false
  # this repo, as the compose mounts it (/workspaces/f1tenth_more is ~/dev_ws/f1tenth_more, ro);
  # with the sim in the repo (hil-isaac) point it at that checkout's tools/isaac_keyboard
  module: /workspaces/f1tenth_more/tools/isaac_keyboard/isaac_keyboard.py
  params: {topic: /teleop, max_speed: 1.0, hold_when_idle: true}
```
`sim/main.py`, after `ros = RosBridge(...)` and inside the loop after `ros.spin_once()`:
```python
    kb = None
    if cfg.ros.get("keyboard", {}).get("enabled"):
        import importlib.util
        _spec = importlib.util.spec_from_file_location("isaac_keyboard", cfg.ros["keyboard"]["module"])
        _kbm = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_kbm)
        kb = _kbm.IsaacKeyboardTeleop(ros.node, _kbm.TeleopParams.from_dict(cfg.ros["keyboard"].get("params")), log=log.info)
    ...
        while app.is_running() and not stop["flag"]:
            ros.spin_once()
            if kb: kb.step()          # wall-clock dead-man and rate, independent of the RTF
    ...
    if kb: kb.close()                 # in the finally/teardown
```
It needs the GUI (`--gui`) for carb.input. For the evdev fallback inside the container, add
`devices: [/dev/input:/dev/input]`, group `input`, and
`/isaac-sim/python.sh -m pip install evdev`.

Standalone, without the hook, on any host with the stack's ROS env (evdev, see sudo steps):
`scripts/sim/sim_isaac.sh run --net eth python3 tools/isaac_keyboard/isaac_keyboard.py`.

## Tests

| test | what | runs without Hydra |
|---|---|---|
| `src/f1tenth_hydra/test/test_hydra_smoke.py` | Hydra runs with seg_source:=gt + isaac_gt on a synthetic ZED 2i + GT feed (`hydra_synthetic_rgbd`: ray-cast room, boxes, plant, loop path) and publishes a graph with PLACES and objects/places nodes within 150 s. Also builds the command line for every profile x seg x pose | the command-line half only (the graph half is skipped) |
| `tools/isaac_keyboard/test_isaac_keyboard.py` | dead-man, stop latch, speed 0 or >= 0.4, steering limits | yes |
| `tools/hydra_cost/test_hydra_cost.py` | the sampler runs, writes CSV + summary, catches a busy process | yes |
| `scripts/hydra/check_hydra_pins.py` | the `third_party/hydra_ros` submodules are the commits and URLs `hydra_jazzy.repos` names (also run by `build_hydra_ws.sh`) | yes, and without the submodules initialized |

## Not verified yet

Written and checked on linus (RTX 5050, no Hydra, no GTSAM). Not run on the workstation:

- **Hydra itself.** Nothing here has run the real Hydra, so the smoke test's graph half has not
  run. The launch composition was checked against a fake Hydra prefix: node command lines,
  remaps, the included semantic_inference and visualizer launch files. Hydra config keys are
  copied from upstream configs (habitat, uhumans2, zed2i) and checked against the source, not
  against a running parser.
- **TensorRT MobileNetV2 on the 4090.** The engine is built at the first run.
- **Jetson aarch64 build** (jetson_min): only the profile and the steps above.
- **`pose_source:=stack_tf`.** The TF lookups against the stack's EKF + camera TF over the link,
  and the `odom -> hydra_map` anchor.
- **hil-isaac integration.** The keyboard hook in `sim/main.py` + `ros_interface.yaml`, carb.input
  in Isaac 5.1 (API names from Kit's carb.input), and the GT semantics at the HIL frame rate
  (Isaac RTF with the semantic annotator on).
- Open3D window of `hydra_dsg_viewer`. The matplotlib path, `--summary`, the live topic path
  (against a Python DSG publisher), `open_graph.sh`, the logger's saver and `hydra_cost` were
  run on linus with the pinned spark_dsg built from source.

## sudo steps (run once, last)

On the workstation (Ubuntu 24.04, ROS 2 Jazzy):

```bash
sudo apt install python3-rosdep python3-colcon-common-extensions python3-pip
sudo rosdep init 2>/dev/null; rosdep update
# Hydra dependencies (GTSAM, PCL, glog, zmq, nanoflann, ...), from the pinned submodules
# (git submodule update --init third_party/hydra_ros first; build_hydra_ws.sh stops and prints
# this command when anything is missing):
rosdep install --from-paths third_party/hydra_ros --ignore-src -r -y   # asks for sudo itself
# TensorRT + nvcc for seg_source:=mobilenet (CUDA repo; match the cudaXX.Y of libnvinfer-dev):
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb && sudo apt update
apt-cache policy libnvinfer-dev                                       # note +cudaXX.Y
sudo apt install libnvinfer-dev libnvonnxparsers-dev libnvinfer-plugin-dev cuda-nvcc-XX-Y
# keyboard: evdev fallback / standalone keyboard node only (log out and in afterwards)
sudo usermod -aG input "$USER"
```
