# Simulated ZED over the two-machine link: compress it

Setup: Gazebo Harmonic on **linus** ↔ Jazzy stack on the **Thor** (`sim:=true`),
direct Ethernet (10.42.0.1 ↔ 10.42.0.2), `ROS_DOMAIN_ID=42`, Fast DDS Discovery
Server on the Thor. Found and fixed 2026-10-07, branch `jazzy`.

## Symptom

YOLO produced no detections in sim. `/camera/detections` was empty, the
annotated image showed the raw frame with no boxes, a chair was clearly in
`/camera/image_raw`.

## It was not YOLO, it was starvation

- The detector loads fine: the perception component log shows
  `Loaded YOLO model ".../yolo26s-seg.pt" on device="cuda" task="segment"
  (80 classes)` — not passthrough. `/usr/bin/python3` (the node's interpreter)
  imports `ultralytics` 8.4.174 with `torch.cuda.is_available() == True`.
- The sim renders fine: `gz topic` measures the RGB sensor at ~36 Hz, RTF 1.0.
- But a persistent subscriber on the **Thor** received only **3 frames in 60 s**
  (0.05 Hz) on `/camera/image_raw`. The detector was starved; empty detections
  were the only possible result.

## Root cause

The raw ZED Image streams are too heavy for the cross-machine Discovery-Server
link: RGB 640×360 rgb8 ≈ **675 KB/frame**, depth 640×360 32FC1 ≈ **921 KB/frame**.
Each frame fragments into tens of UDP datagrams; over the unicast DS link they
do not arrive intact at any usable rate. Same class of problem as the `/clock`
fan-out (`output/sim_clock_fanout.md`), one notch worse — there it was packet
*count*, here it is packet *size*.

## Fix: compress on linus, decompress on the Thor

Only small compressed topics cross the cable; the canonical raw topic names the
stack reads on the car are reconstructed on the Thor, so `camera.launch.py`,
`detection.launch.py` and `yolo_detector_node` are unchanged.

1. **Bridge → local staging** (`f1tenth_sim/config/ros_gz_bridge_camera.yaml`):
   the two heavy Image streams are bridged onto sim-PC-local topics
   `/sim/camera/image_raw` and `/sim/camera/depth_raw` (not the canonical
   names). camera_info is tiny and stays on the canonical names.
2. **Compressors on linus** (`f1tenth_sim/launch/sim_bringup.launch.py`, step
   3c, `camera:=true`): `image_transport republish`
   - `/sim/camera/image_raw` → `/camera/image_raw/compressed` (JPEG)
   - `/sim/camera/depth_raw` → `/zed2/zed_node/depth/depth_registered/compressedDepth` (PNG)
3. **Decompressors on the Thor** (`f1tenth_perception/launch/sim_camera_tf.launch.py`,
   the sim-mode swap for `camera.launch.py`): `image_transport republish`
   - `/camera/image_raw/compressed` → `/camera/image_raw`
   - `.../depth_registered/compressedDepth` → `/zed2/zed_node/depth/depth_registered`

Deps added to both packages' `package.xml`: `image_transport`,
`compressed_image_transport`, `compressed_depth_image_transport`.

### `republish` invocation gotchas (Jazzy)

- The transports are **parameters**, not positional: `-p in_transport:=raw
  -p out_transport:=compressed`. Passing them positionally leaves
  `out_transport` empty and it silently publishes raw.
- Only the **fully-qualified** topics remap; the `in` / `out` base-name remaps
  are ignored (the node published `/out/compressed` regardless). Remap
  `/in`, `/out/compressed`, `/in/compressed`, `/out` etc. explicitly.

## Measured after the fix (linus, super client)

| stream | raw size | compressed size | reduction |
|---|---|---|---|
| RGB `/camera/image_raw` | ~675 KB | **21.4 KB** JPEG | ~31× |
| depth depth_registered | ~921 KB | **20.1 KB** PNG | ~45× |

Compressed RGB ran at ~26 Hz, depth ~25 Hz. A 20 KB message fits in a handful
of datagrams and crosses the link reliably like `/scan`.

## To verify on the Thor (after `git pull` + restart perception)

```bash
# the decompressors reconstruct the canonical topics
ros2 topic hz /camera/image_raw          # ~expected sim rate, steady
ros2 topic hz /zed2/zed_node/depth/depth_registered
# detections should now be non-empty when something recognisable is in frame
ros2 topic echo /camera/detections --once
```
Needs `ros-jazzy-compressed-image-transport` and
`ros-jazzy-compressed-depth-image-transport` installed on the Thor.

## Residual

- gz render RTF still swings on linus under camera load; headless keeps it under
  the 0.5 s clock-pause gate (`output/sim_clock_yolo_handoff.md` §1). Unrelated
  to this fix.
- COCO weights on Gazebo-rendered furniture are weak; if detections are sparse
  with frames now flowing, lower `confidence_threshold` (0.3) or expect sim
  fidelity limits — a scene issue, not a transport one.
