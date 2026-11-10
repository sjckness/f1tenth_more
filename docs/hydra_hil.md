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
- **Jazzy support.** Upstream tests Ubuntu 24.04 + Jazzy and Iron. The README still calls the
  ROS 2 port "in active development". Jazzy is the newest distro they name.
- **Dependencies.** GTSAM (`ros-jazzy-gtsam`), PCL, OpenCV, glog/gflags, zmq and nanoflann come
  from rosdep. TensorRT + CUDA are needed only for `seg_source:=mobilenet` (semantic_inference).
  The spark_dsg Python bindings are built from the same pinned source with pip `--target`.
- **Native vs Docker.** Native on the workstation, in its own workspace `~/hydra_ws`, outside this
  repo. Reasons: the stack and RViz are native Jazzy, a Docker Hydra would only add a DDS
  boundary, and upstream's `docker/` targets their own datasets. The f1tenth workspace builds and
  runs without Hydra. `f1tenth_hydra` finds Hydra at launch time.
- **Segmentation.** semantic_inference's closed-set node with **MobileNetV2-dilated C1 deepsup
  (ADE20K)**, `ade20k-mobilenetv2dilated-c1_deepsup.onnx`. The TensorRT engine is built from the
  ONNX on the first run, on the GPU that runs it. Labels are grouped to `ade20k_mp3d`, the same
  label space the GT map uses. I found no record of the 8/11 investigation on this machine, so the
  model is semantic_inference's own MobileNetV2 export. Check that it is the one 8/11 used.

```bash
# once, after the sudo steps at the end of this file (~25 min with 32 jobs)
scripts/hydra/build_hydra_ws.sh --models
source scripts/hydra/hydra_env.sh       # ROS + ~/hydra_ws + this workspace + spark_dsg
colcon build --symlink-install --packages-select f1tenth_hydra f1tenth_logger
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
   snapshot (CLAUDE.md "Rebuild policy"):
   ```dockerfile
   FROM f1tenth:jazzy-orin-<sha>
   USER root
   RUN apt-get update && apt-get install -y ros-jazzy-gtsam python3-vcstool \
       libgoogle-glog-dev libgflags-dev libzmq3-dev nanoflann-dev && rm -rf /var/lib/apt/lists/*
   COPY hydra_jazzy.repos /opt/hydra_ws/
   RUN mkdir -p /opt/hydra_ws/src && cd /opt/hydra_ws && vcs import --workers 1 src < hydra_jazzy.repos \
    && . /opt/ros/jazzy/setup.sh && rosdep install --from-paths src --ignore-src -r -y \
    && MAKEFLAGS=-j6 colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
         --packages-up-to hydra_ros semantic_inference_ros \
    && python3 -m pip install --no-deps --target /opt/hydra_ws/install/spark_dsg_python src/spark_dsg
   ```
   `docker build --network=host -t f1tenth:hydra-orin-<sha> .`, with `hydra_jazzy.repos` copied
   next to the Dockerfile. Plan for about 2 GB of RAM per make job. The Orin has 16 GB shared with
   the GPU, so keep `-j6` or lower.
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
