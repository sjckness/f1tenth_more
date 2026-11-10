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
