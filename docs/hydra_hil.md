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
