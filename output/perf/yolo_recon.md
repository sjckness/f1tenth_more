# YOLO node GPU/CPU recon (2026-10-22, branch campaign-final-yolo-gpu @ fa5471f)

Read-only reconnaissance for making `yolo_detector_node` cheap on CPU. Tags: **[V]** verified
by a command in this session, **[R]** read in code, **[I]** inferred. Line numbers refer to
this worktree unless a path says otherwise.

---

## A. Versions

### A.1 Orin vs linus

| | Orin (target, `ssh jetson-orin`) | linus (dev) |
|---|---|---|
| Board / OS | Jetson AGX Orin Developer Kit, Ubuntu 22.04.5, L4T R36.5.0 (`nvidia-l4t-core 36.5.0-20260115194252`), JetPack `nvidia-jetpack 6.2.2+b24`, nvpmodel MODE_50W [V] | Ubuntu 24.04.4, kernel 7.0.0-38, i7-9700K [V] |
| CPU | 12 x Cortex-A78AE, ids 0-11 online, 1 thread/core, 2.2 GHz max [V] | 8 x i7-9700K, ids 0-7, no SMT [V] |
| RAM | 61 GiB + 30 GiB swap (shared with the iGPU) [V] | 15 GiB, ~3.7 GiB available [V] |
| ROS | Humble, `/opt/ros/humble` [V] | Jazzy, `/opt/ros/jazzy` [V] |
| Python of the stack | `/usr/bin/python3` 3.10.12, packages in user site `~/.local/lib/python3.10/site-packages` (no venv) [V] | `/home/andreas/dev_ws/jc_venv/bin/python` 3.12.3 (`include-system-site-packages = true`) [V] |
| GPU | Orin iGPU, capability (8, 7), torch arch_list `['sm_87']` [V] | RTX 5050 8 GB, compute 12.0, driver 580.178.04 (CUDA 13.0 driver), ~4.3 GB already used by other processes (telegram venv 2 GB) [V] |
| CUDA toolkit | 12.6 (`cuda-toolkit-12-6 12.6.11`, `nvcc` V12.6.85, `/usr/local/cuda-12.6`) [V] | none (no nvcc); torch ships cu128 [V] |
| cuDNN | `libcudnn9-cuda-12 9.3.0.75`; torch reports 90300 [V] | torch-bundled 91002 [V] |
| TensorRT | dpkg `tensorrt 10.3.0.30-1+cuda12.5`, `libnvinfer10`, `libnvinfer-bin` (trtexec), `python3-libnvinfer 10.3.0`; `import tensorrt` -> 10.3.0 from `/usr/lib/python3.10/dist-packages` (also `tensorrt_lean`, `tensorrt_dispatch`) [V] | **not installed** (no Python bindings) [V] |
| torch | 2.11.0 (`~/.local`), `torch.version.cuda` 12.6, `cuda.is_available()` True, default threads 12/12 (unpinned). **Needs `LD_LIBRARY_PATH=$HOME/.local/lib/python3.10/site-packages/nvidia/cu12/lib`** (else `ImportError: libcudss.so.0`); set in `~/.bashrc` and `src/f1tenth_ctl/config/ctl.env`, inherited by the live node [V] | jc_venv: 2.8.0+cu128, arch_list includes sm_120, `torch.zeros(1).cuda()` OK. py_venv: 2.7.1+cu126, **no sm_120: "no kernel image is available"**, unusable on this GPU [V] |
| torchvision | 0.26.0, `ops.nms` on CUDA OK, fp16 `F.interpolate` on CUDA OK [V] | jc_venv 0.23.0+cu128, `ops.nms` on CUDA OK [V] |
| ultralytics | 8.4.90 (`~/.local`) [V] | jc_venv 8.4.90 (py_venv 8.4.6). **Every `*.py` of the package is byte-identical to the Orin's** (md5 of the sorted per-file md5 list `002504359dd15cb2d82273f25d8287ea` on both) [V] |
| numpy | 1.26.4 (`~/.local`, shadows system 1.21.5) [V] | 1.26.4 (`/usr/lib/python3/dist-packages`) [V] |
| OpenCV (Python) | `opencv-python` and `opencv-python-headless` 4.11.0.86 in `~/.local` -> cv2 4.11.0; `cv2.cuda.getCudaEnabledDeviceCount()` = 0, no CUDA in build info, parallel framework pthreads. System `nvidia-opencv`/`libopencv 4.8.0` installed but shadowed [V] | `opencv-python 4.11.0.86` (jc_venv), no CUDA, count 0 [V] |
| cupy | not installed [V] | not installed [V] |
| onnx / onnxruntime / onnxslim | 1.22.0 / 1.23.2 (providers: Azure, **CPU only**) / 0.1.94 [V] | none in jc_venv [V] |
| pycuda, cuda-python, polygraphy, torch_tensorrt, triton | none importable [V] | not checked (not needed) |
| cv_bridge | `ros-humble-cv-bridge 3.2.1`; `imgmsg_to_cv2(msg, desired_encoding)`, `cv2_to_imgmsg(cvim, encoding, header=None)` same signatures as Jazzy [V] | Jazzy cv_bridge 4.1.0 [V] |
| vision_msgs | 4.1.1 [V] | 4.1.1 [V] |
| ZED | ZED SDK 4.2.5 (`/usr/local/zed`), `ros-humble-zed-msgs 5.3.0` [V] | none |
| Monitoring | `/usr/bin/tegrastats`; `jtop --force` already running [V] | `nvidia-smi` [V] |

### A.2 How the stack runs on the Orin [V]

- Checkout `/home/fabiocar/dev_ws/f1tenth_more`, branch `stack-campaign-final-2026-10-22` at
  `fa5471f3` (this commit) with **staged** changes in f1tenth_behavior (M `README.md`,
  `behavior_executor_node.py`, `behaviours/check_stop_condition.py`, `mission/condition_eval.py`;
  D `mission/wall_lock.py`). Not touched.
- Symlink install: `install/f1tenth_perception/lib/python3.10/site-packages/f1tenth-perception.egg-link`
  -> `build/f1tenth_perception`; launch files in `install/.../share` are symlinks. A `git pull` of
  Python source takes effect without a rebuild for this package.
- Launch: f1tenth_ctl `ctl.env` sources `/opt/ros/humble/setup.bash` +
  `~/dev_ws/f1tenth_more/install/setup.bash`, sets the `nvidia/cu12/lib` LD_LIBRARY_PATH,
  `ROS_DISCOVERY_SERVER=127.0.0.1:11811`, `F1T_STACK_CMD="ros2 launch f1tenth_bringup supervisor_bringup.launch.py"`.
  Native (no container running). The YOLO process is `/usr/bin/python3 .../install/f1tenth_perception/lib/f1tenth_perception/yolo_detector_node --params-file /tmp/launch_params_*`.
- Models on the Orin (`src/f1tenth_perception/models/`): `yolo26s-seg.pt` (deployed, md5
  `91df624caffc982a5ed76f6061c01bf1`, identical to every linus copy), `yolo26s.engine` (Jul 8,
  detect-only, built by `trtexec` from `yolo26s.onnx`, 640x640, no Ultralytics metadata),
  `yolo26s.onnx`, `yolo26s.pt`, `yolo26m.pt`, `yoloe-26s-seg.pt`. **No segmentation engine exists.**

### A.3 linus environment that works [V]

```bash
source /opt/ros/jazzy/setup.bash
source /home/andreas/dev_ws/f1tenth_more_campaign/install/setup.bash   # f1tenth_messages/params/description, zed_msgs
export PYTHONPATH=/home/andreas/dev_ws/f1tenth_more_yolo_gpu/src/f1tenth_perception:$PYTHONPATH
/home/andreas/dev_ws/jc_venv/bin/python ...
```

Verified in one interpreter: torch 2.8 CUDA on the RTX 5050, ultralytics 8.4.90, rclpy, cv_bridge,
sensor_msgs, vision_msgs, tf2_ros, `f1tenth_messages.msg`, `f1tenth_params.param_defaults`
(`get_value('yolo_model')` -> `yolo26s-seg.pt`), `f1tenth_description.vehicle_frames.base_front_x()`
(0.405) and `f1tenth_perception.yolo_detector_node` all import together.
- **Without the PYTHONPATH line, `f1tenth_perception` resolves to
  `/home/andreas/dev_ws/f1tenth_more_campaign/build/f1tenth_perception/f1tenth_perception`, a symlink
  to the campaign worktree's src**, not this worktree. The yolo_gpu worktree has no install.
- numpy comes from `/usr/lib/python3/dist-packages` (1.26.4), cv2 from jc_venv.
- `rosbags 0.11.8` is in `~/.local` (user site, hidden under `python -I`); `rosbag2_py` from Jazzy works.
- First YOLO call takes ~4.4 s (CUDA init + warmup) [V].

### A.4 Ultralytics 8.4.90 code paths (same on both machines)

Source on linus: `/home/andreas/dev_ws/jc_venv/lib/python3.12/site-packages/ultralytics/`
(Orin: `~/.local/lib/python3.10/site-packages/ultralytics/`). No download was needed: the trees are
identical, so there are no signature differences between the two installs.

- `engine/model.py:528` `Model.predict`: per call `custom = {conf: 0.25, batch: 1, save, mode: predict,
  rect: True, embed: None}` merged with kwargs. The predictor is created once; it is re-created only if
  `device` differs from `predictor.args.device`; otherwise `get_cfg(predictor.args, args)` every call.
- `engine/predictor.py`
  - `preprocess` :155-177: `np.stack(pre_transform(im))` -> BGR->RGB `[..., ::-1]` -> transpose BCHW ->
    `np.ascontiguousarray` -> `torch.from_numpy` -> `.to(device)` -> `.half()` if `model.fp16` else
    `.float()` -> `/= 255`. Tensor input skips the numpy part and the `/255`.
  - `inference` :179-186 -> `self.model(im, augment, visualize, embed)`.
  - `pre_transform` :188-205: `LetterBox(imgsz, auto=same_shapes and args.rect and (format == 'pt' or
    dynamic), stride=model.stride)`.
  - `stream_inference` :281-388: `setup_source` (check_imgsz + `load_inference_source`) **every call**;
    warmup once; three `ops.Profile(device=...)` (:322-324), each `torch.cuda.synchronize()` on enter and
    exit (6 syncs per frame); `results[i].speed` = ms per stage.
  - `setup_model` :391-418: `AutoBackend(model, device, dnn, data, fp16=args.quantize == 16 (:408),
    fuse=True)`; for an exported non-dynamic model `args.imgsz = model.imgsz` from metadata (:416);
    `attempt_compile(..., mode=args.compile)` (:418).
  - **fp16 is decided only in setup_model.** Verified: `half=True` passed on a later call left
    `predictor.model.fp16 == False`; a fresh model whose first call had `quantize=16` gave fp16 True [V].
    `half` is deprecated and forwarded to `quantize` with a warning (`cfg/__init__.py:525-534`).
- `data/augment.py` LetterBox :1605 (`new_shape=(640,640), auto=False, scale_fill=False, scaleup=True,
  center=True, stride=32, padding_value=114, interpolation=cv2.INTER_LINEAR`); `get_params` :1700-1747:
  `r = min(nh/h, nw/w)`, `new_unpad = (round(w r), round(h r))`, `auto` -> `dw, dh = mod(dw, stride),
  mod(dh, stride)`, `center` -> `top = round(dh/2 - 0.1)`, `bottom = round(dh/2 + 0.1)`; `apply_image`
  :1749-1782: `cv2.resize` only if the size changes, then `cv2.copyMakeBorder(..., 114)`.
  - **For the 640x360 ZED frame:** `.pt` (rect, auto): r = 1, no resize, pad 24 -> 12 top + 12 bottom ->
    tensor **(1, 3, 384, 640)** [V]. A static engine exported at `imgsz=(384, 640)` gives the same
    letterbox (auto False, r = min(384/360, 640/640) = 1, pad 12/12) [I from the code]. A 640x640 engine
    (the old detect engine) pads 140/140.
- `models/yolo/segment/predict.py` :48-66 `postprocess`: `protos = preds[0][1] if isinstance(preds[0],
  tuple) else preds[1]`, then DetectionPredictor.postprocess. `construct_result` :86-111: empty ->
  masks None; `retina_masks` -> `scale_boxes(img.shape[2:], boxes, orig.shape)` then
  `process_mask_native(proto, pred[:, 6:], pred[:, :4], orig.shape[:2])`; else `process_mask(...,
  img.shape[2:], upsample=True)` then scale_boxes. Then `keep = masks.amax((-2, -1)) > 0;
  if not (all(keep) or ...)` (:108-110) — Python `all()` over a CUDA tensor, one sync per detection —
  drops mask-less detections. `Results(orig_img, names, boxes=pred[:, :6], masks=masks)`.
- `models/yolo/detect/predict.py` :33-80: `nms.non_max_suppression(preds, conf, iou, classes,
  agnostic, max_det, nc=len(names) for seg, end2end=getattr(model, 'end2end', False))`;
  `construct_result` :109-122 (scale_boxes).
- `utils/nms.py` :13-29 signature; **end2end branch :66-70: `[pred[pred[:, 4] > conf][:max_det]]`** —
  no NMS, strict `>`, then class filter. The classic NMS path (:72-166) only runs for non-end2end models.
- `nn/modules/head.py`: YOLO26 top-k is inside the head (`postprocess` :219-233, `get_topk_index`
  :235-258; `k = max_det` when exporting, else `min(max_det, anchors)`); `Segment26` :369.
- `utils/ops.py`: `scale_boxes` :103-142; `crop_mask` :461-481 (in-place separable multiply);
  `process_mask` :484-510 (`masks_in @ protos`, crop at proto resolution, bilinear upsample, `> 0` ->
  uint8); `process_mask_native` :513-538 (`masks_in @ protos.view(c, -1)`, chunked `scale_masks` to the
  original shape (32 Mpx budget), `.gt_(0).byte()`, `crop_mask` with original-resolution boxes);
  `scale_masks` :541-578 (crop letterbox padding, `F.interpolate(..., mode='bilinear')`, float).
- `nn/autobackend.py` ctor :166-175 `(model, device=cpu, dnn=False, data=None, fp16=False, fuse=True,
  verbose=True)`; fp16 only for pt/torchscript/onnx/openvino/engine/triton (:192); `forward` :248-297
  (casts input to half when the backend is fp16; 999 placeholder names for segment outputs without
  metadata). `nn/backends/tensorrt.py` :25-114 `load_model`: optional 4-byte length + JSON metadata
  header, `deserialize_cuda_engine`, TRT10 named I/O tensors, fp16 from the input dtype, dynamic if -1;
  `forward` :116-144: `context.execute_v2` (**synchronous**, default stream), outputs sorted by name.
  Rejects TRT 10.2.0 only. `nn/backends/base.py:112-140` `apply_metadata`: stride, names, imgsz, task,
  `end2end = metadata end2end or args.nms`, `dynamic`.
- `engine/exporter.py:1196-1218` `export_engine` (asserts a CUDA device; `export_onnx` first, simplify
  default True) -> `utils/export/engine.py:172 onnx2engine(onnx_file, output_file, workspace,
  quantize, dynamic, shape, dla, dataset, metadata, verbose, prefix)`. Export args (cfg/default.yaml):
  `imgsz` (int or [h, w]), `quantize` 16/8 (:56, `half` deprecated), `end2end` (:59), `dynamic` False
  (:87), `simplify` True (:88), `workspace` GiB (:90), `nms` False (:91), `device`. The metadata header
  keeps names/task/imgsz/end2end, unlike the manual `trtexec` recipe in `yolo_detector_node.py:56-60`.
- **YOLO26-seg outputs, `.pt`, 384x640 input [V]:**
  `[ (Tensor(1, 300, 38), Tensor(1, 32, 96, 160)), {one2many: {proto}, one2one: {boxes (1,4,5040),
  scores (1,80,5040), feats [(1,128,48,80), (1,256,24,40), (1,512,12,20)], mask_coefficient (1,32,5040),
  proto (1,32,96,160)}} ]`. 38 = x1 y1 x2 y2 (letterbox px), score, class, 32 mask coefficients.
  Head `Segment26`: end2end True, max_det 300, nc 80, nm 32, npr 128, reg_max 1. Results:
  `boxes.data` (N, 6) float32 cuda; `masks.data` **(N, 360, 640) torch.uint8** cuda with
  `retina_masks=True`, (N, 384, 640) uint8 with `retina_masks=False` [V]. An exported engine should
  have only the two one2one outputs (1,300,38) and (1,32,96,160) [I, not exported].
- linus timing, `.pt` fp32 on the RTX 5050 [V]: Ultralytics speed preprocess ~0.6 ms, inference ~9.3 ms,
  postprocess ~0.7 ms. A node-like frame (cvtColor + call + the node's two mask copies, exclusion and
  label loops) = **15.4 ms wall and 15.4 ms process CPU**: the eager `.pt` path keeps one core busy for
  the whole frame. cProfile: 117 `torch.conv2d` launches and ~404 `nn.Module` calls per frame,
  `conv2d` tottime alone 5.6 ms/frame (launch overhead, the GPU work is async). On the much slower
  A78AE cores this is the [I] main reason the live main thread sits at ~87 % (B.3).

### A.5 APIs the new code must avoid (Orin limits)

1. **Python 3.10:** no `except*`/ExceptionGroup, `typing.Self`/`Never`/`LiteralString`/`assert_never`
   (3.11), `typing.override` (3.12), `tomllib`, `enum.StrEnum`, `datetime.UTC`, `asyncio.TaskGroup`/
   `asyncio.timeout` (3.11), `itertools.batched`, `Path.walk` (3.12), PEP 695 (`type X = ...`,
   `def f[T]()`), PEP 701 f-strings (same quote reused inside, backslashes or comments in the
   expression part). `match`, `X | Y` annotations and `zip(strict=)` are fine.
2. **ROS Humble rclpy:** no `rclpy.event_handler` (Humble has `rclpy.qos_event`), no
   `create_timer(..., autostart=)` (Jazzy only) [V]. cv_bridge signatures are the same [V].
3. **torch:** the Orin has 2.11, linus 2.8; code that runs on 2.8 should run, but avoid deprecated APIs
   (e.g. `torch.cuda.amp.*`, use `torch.autocast('cuda', ...)`). **No triton and no torch_tensorrt on the
   Orin**, so `torch.compile` (inductor) and Ultralytics `compile=True` are out [V].
4. **No cupy, no `cv2.cuda`** on either machine; **onnxruntime on the Orin is CPU-only**; no pycuda,
   cuda-python, polygraphy [V]. GPU pre/post-processing has to be plain torch (+ `tensorrt` on the
   Orin only).
5. **TensorRT is Orin-only:** no bindings on linus (and TRT 10.3 predates Blackwell sm_120 [I]). The
   engine path can only be exercised on the Orin; linus can only run the `.pt` path. Engines are
   per-device and not built here.
6. **Ultralytics:** same version, so no argument differences. Gotchas: pass `quantize=16` (or `half`)
   on the **first** call; `rect=True` is the predict default (different letterbox for `.pt` and a
   static engine unless the engine is exported at 384x640); masks are **uint8**, not float;
   `retina_masks=False` masks are at letterbox resolution (384x640), not 360x640.
7. **numpy 1.26 on both:** no numpy-2-only APIs.
8. **LD_LIBRARY_PATH:** torch on the Orin only imports with the `nvidia/cu12/lib` entry. A new
   `additional_env` must not replace LD_LIBRARY_PATH (launch_ros `additional_env` merges keys, so do
   not set that key at all).

---

## B. The YOLO node

### B.1 Files and deployed configuration

- Node: `src/f1tenth_perception/f1tenth_perception/yolo_detector_node.py` (1396 lines). Helpers:
  `latest_frame_qos.py` (depth 1, BEST_EFFORT), `stage_timing.py`, `perception_debug.py`,
  `cpu_affinity.py` (nice only), `front_clearance_node.from_front`, `ground_plane.quat_to_matrix`.
- Launch: `src/f1tenth_perception/launch/detection.launch.py:379-418` (Node, `prefix=['taskset -c ',
  yolo_cpu_affinity]`, default `8,9`), included by `f1tenth_bringup/launch/stack_bringup.launch.py:181`
  and `f1tenth_perception/launch/perception.launch.py:53`. `model_path` = `<src>/f1tenth_perception/models/<yolo_model>` (:389-391).
- Deployed values (stack_params.yaml and the live `/tmp/launch_params_8ot749c3` on the Orin, read
  14:29) [V]:

| param | value | source |
|---|---|---|
| model | `yolo26s-seg.pt` (task segment, from the checkpoint) | stack_params.yaml:237-238, :247-248 |
| device | `cuda` (fp32: no half/quantize anywhere) | :209-210 |
| conf | 0.3; 0.1 on 5 Hz debug ticks (split after inference) | :205-206; node :585-587, :1147-1177 |
| imgsz / letterbox | Ultralytics default 640 + rect -> **384x640, pad 12 top/bottom, no resize** | [V] A.4 |
| retina_masks | True (hard-coded) | node :601-603 |
| iou / max_det | 0.7 (unused, end2end) / 300 | Ultralytics defaults |
| use_mask_depth | true (also auto from '-seg') | stack_params.yaml:266-267, detection.launch.py:146-158 |
| debug image | 5 Hz, 480 px wide, JPEG q50, min conf 0.1 | stack_params.yaml:344-357 |
| enable_scene_labels | true | :770-771 |
| stage_timing | false (`perception_stage_timing`) | :359-360 |
| lidar exclusion | x 0.75-1.0, y 0.55-1.0, overlap 0.5 | live params |
| nice | 0 | detection.launch.py:346-348 |

- Camera [V]: ZED 2i, wrapper on the Orin: `grab_resolution HD720`, `grab_frame_rate 30` (zed2i.yaml:9-10),
  `pub_resolution CUSTOM`, `pub_downscale_factor 2.0`, `pub_frame_rate 30`
  (`src/f1tenth_perception/config/zed2_perception.yaml:15-18`) -> **640x360 bgra8 at 30 Hz** on
  `/zed2/zed_node/rgb/image_rect_color`, remapped to **`/camera/image_raw`**
  (`camera.launch.py:119-120`); depth `depth_mode PERFORMANCE`, `openni_depth_mode false` -> 32FC1 metres,
  640x360, `/zed2/zed_node/depth/depth_registered`. Bag check: 640x360 bgra8 step 2560, frame
  `zed2_left_camera_optical_frame`, fx = fy = 262.428, cx = 318.742, cy = 179.294, rectified (d = 0).
- Conversion: `imgmsg_to_cv2(msg, 'bgr8')` (:566; bgra8 -> bgr8 via cvtColor); masks out
  `cv2_to_imgmsg(label, 'mono8')` (:811); annotated `cv2_to_imgmsg(img, 'bgr8')` (:733); empty mask
  built by hand (:740-768).
- Subscription: `/camera/image_raw`, LATEST_FRAME_QOS (:462-463). `rclpy.spin(node)` (:1386):
  single-threaded executor, so every debug subscription runs on the inference thread.

### B.2 Rates and latencies on record

- 13.4 Hz detections in the 2026-10-01 campaign bags (`yolo_detector_node.py:225-228`).
- "30 Hz camera and a ~15 Hz callback" (:459-461); `output/cpu_cleanup/rate_audit.md:112`: ~15 Hz
  callback, 3D ~5-8 Hz (also `stack_params.yaml:805`).
- Latency audit: ~22 ms in-callback inference (old TRT box engine), `/camera/detections` ~278 ms behind
  `/camera/image_raw`, nonvoluntary:voluntary context switches ~6:1 (`yolo_detector_node.py:286-299`,
  `cpu_affinity.py:11-19`).
- Annotated drawing + conversion ~7 ms per frame on the car (2026-10-14 stage_timing, :696-698).
- Detector latency ~0.45 s with the old input queue, ~0.15 s expected after LATEST_FRAME_QOS
  (`detection_3d_node.py:118-121`).
- GPU latency: TRT box engine 25.4 ms, `yolo26s.pt` detect 40.7 ms, `yolo26s-seg.pt` ~52.3 ms
  (`docs/history/LOG.md:231-232`).
- Clean debug JPEG 0.43 ms per frame on linus (`output/night_2026-10-21/night_report.md:239`).
- From bag message counts (lower bounds, the recorder drops): masks per second 10-22T12-11-04 290/32.4 s
  = 9.0 Hz; 10-21T12-25-22 173/18.8 s = 9.2 Hz; 10-14T12-05-34 804/73.5 s = 10.9 Hz; 10-08T11-27-18
  4190/363 s = 11.5 Hz [V].
- No STAGE_TIMING output of the YOLO node is stored anywhere in `output/` or `docs/` [V].

### B.3 Live snapshot on the Orin (a stack started by fabiocar at ~14:24; stopped again by 14:43) [V]

`yolo_detector_node`: `Cpus_allowed_list 8-9`, 16 threads, RSS 1.6 GB, **main thread 86.7-87.2 %**
of a core (`top -H`), process 60-90 % (`ps`); GPU load (sysfs) samples 29-99 % (shared with ZED depth);
load average 12-17.5 on 12 cores. detection_3d 37 % (6-7), obstacle_projector 24 % (6-7),
front_clearance 43 % (unpinned, 22 threads), semantic_layer 27 % (unpinned, 33 threads).

### B.4 Per-frame CPU work in the hot path

Sizes for 640x360 and N detections. "sync" = a GPU->CPU synchronisation.

| # | where | operation | data | when |
|---|---|---|---|---|
| 1 | :566 | `imgmsg_to_cv2(msg, 'bgr8')`: frombuffer + `cv2.cvtColor` BGRA2BGR | 921,600 B in, 691,200 B out | every frame |
| 2 | :601-603 -> Ultralytics | per-call `get_cfg` + `setup_source` + `LoadPilAndNumpy` (Python); `copyMakeBorder` -> 640x384x3 (737,280 B); `np.stack` + `[..., ::-1]` + transpose + `ascontiguousarray` (737 KB copy); `from_numpy().to(cuda)` (737 KB H2D, sync copy from pageable memory); `.float()/255` on GPU (2.95 MB); eager `.pt` forward (117 conv launches, CPU-bound, A.4); postprocess on GPU (end2end filter, scale_boxes, `process_mask_native`: N x 32 @ 32 x 15,360, bilinear to N x 360 x 640 float32, `> 0` -> uint8 N x 230,400 B, crop_mask); `all(keep)` over a CUDA tensor = N syncs (segment/predict.py:109); 3 Profiles = 6 `cuda.synchronize` | every frame |
| 3 | :609 -> `_debug_split` :1147-1177 | `boxes.conf.cpu().numpy()` (:1159); for every below-threshold box `xyxy[0].tolist()`, `cls[0].item()`, `conf[0].item()` (:1166-1173, 3 syncs each); `result[~above]`, `result[above]` (GPU indexing of the N x 360 x 640 masks) | 5 Hz ticks |
| 4 | :632 -> `_lidar_exclusion_keep_mask` :526-559 | **`result.masks.data.cpu().numpy()`** (:548, N x 230,400 B D2H); per detection `np.any(masks_np[i] > 0.5)` and again `masks_np[i] > 0.5` -> `_mask_overlap_fraction` (`count_nonzero` over 230,400 px + slice) (:552-553, :257-274); box path `xyxy[0].tolist()` (:555) | every frame with detections |
| 5 | :638 | `result[keep]` when something is suppressed | rare |
| 6 | :643 | **second `result.masks.data.cpu().numpy()`** of the same data | every seg frame with detections |
| 7 | :646-654 | loop over `result.boxes` (a Boxes wrapper per index) with `xyxy[0].tolist()`, `cls[0].item()`, `conf[0].item()` (3 syncs per detection); Detection2D construction (:1362-1379) | every frame |
| 8 | :682-694 -> `_publish_masks` :770-817 | `np.zeros((360, 640), uint8)` (:804); per detection `masks_np[i] > 0.5` (full-frame bool) + boolean-index assignment (:805-808); `cv2_to_imgmsg` mono8 (copy 230,400 B) + publish/serialise (:811-816). Zero-detection frames: hand-built 0x0 message (:740-768) | every frame |
| 9 | :699-721 | annotated image: `_draw_mask_overlay` (:819-908) two full copies (:846-847, 691 KB each), per detection `> 0.5`, `np.any`, `overlay[mask] = color`, `cv2.addWeighted` full frame (:864), `np.where(mask)` (two int64 arrays of the mask area, :869), text; `cv2_to_imgmsg` bgr8 691 KB (:733). Detect models: `result.plot()` | only while subscribed (Foxglove relay `/camera/image_annotated/viz`, or `mission_logger_record_annotated_image`) |
| 10 | :722-728 -> `_debug_capture` :1179-1202 | per detection `perception_debug.mask_contours(masks_np[i] > 0.5, size)` (`perception_debug.py:247-251`: full-frame bool, `astype(uint8)` copy, `cv2.resize` NEAREST to 480x270, `findContours`); `cv2.resize(cv_image, 480x270, INTER_LINEAR)` (:1197); clean JPEG `cv2.imencode` q50 + `array.array` copy only while subscribed (:1204-1223) | 5 Hz |
| 11 | `_debug_finish` :1225-1245 (from the tracks callback :1073-1077 or the next tick :1181-1182) | `_debug_frame` dict assembly (:1247-1323), `draw_debug_frame` (cv2 drawing on 480x270 + text strip), `encode_jpeg` q50 | 5 Hz |
| 12 | same thread, `_debug_setup` :996-1051 | `/camera/camera_info` (~30 Hz, latest only, :1027-1028); `/perception/virtual_wall/debug`, `/perception/scene_labels_debug`, `/perception/unclassified_obstacles` with `json.loads` on **every** message, depth 10 (:1016-1020, :1120-1124; virtual_wall/debug up to ~40 Hz per `rate_audit.md:325-329`); `/perception/scene_labels` `imgmsg_to_cv2` mono8 160x90 (:1014-1015, :1082-1086); `/perception/walls` (:1025-1026); `/camera/detections_3d`, `/costmap/semantic_tracks` (:1044-1047); `/perception/debug/front_wall` (:1048-1049); `/tf_static`; `/mission/status` | continuous (~0.35 ms rclpy cost per message on linus) |

### B.5 What stage_timing measures (stage_timing:=true, `perception_stage_timing` launch arg)

`_timing_setup` :921-946, marks :948-987: `age.entry`, `gap_between_processed`, counters
`frames_processed`/`frames_skipped` (stamp gaps against `stage_timing_frame_period_s` = 1/30);
`age.`/`dur.` for `convert`, `model` (the whole Ultralytics call), `split_exclusion`, `masks_to_cpu`,
`build`, `pub_det`, `pub_masks`, `annotate_draw`, `pub_annotated`, `debug_capture`, `age.exit`;
`dur.preprocess`/`inference`/`postprocess` from Ultralytics' CUDA-synchronised `result.speed`;
`dur.post_mask` from a CUDA-synchronised wrapper monkeypatched over `ultralytics.utils.ops.process_mask_native`
(only with a CUDA device; global patch); `dur.post_nms_other` = postprocess - post_mask;
`dur.debug_finish`. `StageTimer` (`stage_timing.py`) logs one `STAGE_TIMING {json}` line every
`stage_timing_period_s` (30 s) with p50/p95/max/n.

### B.6 Existing tests (`src/f1tenth_perception/test/test_yolo_detector_node.py`, 977 lines)

- Construction (`_construct_with_params` :96-106): real node under `rclpy.init()` (module fixture
  :88-93), `YoloDetectorNode(parameter_overrides=[...])`, default `model_path=''` -> **passthrough, model
  None**; `det_pub.publish`, `annotated_pub.publish`, `debug_pub.publish` replaced by MagicMock,
  `annotated_pub.get_subscription_count` -> 1.
- Mocking:
  - `TestModelTaskResolution` (:216-268, 4 tests): `sys.modules['ultralytics']` and `['torch']`
    replaced by fake modules (monkeypatch). Fake `YOLO(path, task=None)` returns `.task`, `.names`,
    `.to()`; fake torch has only `cuda.is_available` and `__version__`. **Any new torch/ultralytics use
    in `__init__`** (set_num_threads, `torch.backends`, streams, `ultralytics.utils` imports, a warmup
    call, `model.predictor`, export) breaks these 4 unless guarded.
  - `_FakeCallableModel` (:712-723): `model(image, **kwargs)` records `kwargs['conf']` and returns
    `[SimpleNamespace(boxes=_FakeBoxes([]), masks=None, names={}, plot=...)]`.
- **Tests that break if image_callback stops calling `self.model(img, conf=..., ...)` once per frame**:
  `TestDebugImage.test_only_tick_frames_run_yolo_below_the_confidence_threshold` (:743-755, conf
  sequence `[0.1, 0.3, 0.3, 0.1, 0.3, 0.3]`, `_debug_capture` count 2),
  `test_with_the_debug_image_off_every_frame_runs_at_the_threshold` (:757-766),
  `test_a_failing_split_reruns_the_frame_at_the_threshold_and_disables_debug` (:768-781, `_debug_split`
  mocked to raise, rerun at 0.3), `test_annotated_image_is_drawn_and_published_only_while_subscribed`
  (:965-977, `_publish_annotated` count, `det_pub.publish` count 1).
- Method signatures pinned by tests: `_publish_masks(masks_np (N,H,W) numpy, shape_hw, header)`
  (TestPublishMasks, 4, :145-214); `_draw_mask_overlay(cv_image, masks_np, detections)` (5, :302-432);
  `_lidar_exclusion_keep_mask(result, h, w)` with duck-typed `_FakeResult` (`.boxes` list of objects
  with `.xyxy`, `.masks.data.cpu().numpy()`) (8, :531-671); `_debug_split(result, h, w)` with
  `_FakeSplitResult` (`boxes.conf.cpu().numpy()`, boolean `__getitem__`) (3, :783-818);
  `_debug_capture(cv_image, header, detections, masks_np, below)` (5, :820-944); module functions
  `_bbox_overlap_fraction`, `_mask_overlap_fraction` (10); `test_image_input_holds_only_the_newest_frame`
  (:952-962, QoS); `TestClassColorBgr` (3).
- Related: `test_detection_launch_config.py` pins launch defaults (yolo_model `yolo26s-seg.pt`, task
  segment, use_mask_depth auto, device cuda); `test_empty_mask_invariant.py` covers the detection_3d side
  of the zero-size mask; `scripts/verify_seg_live_deploy.py` builds the node with real models (manual).

---

## C. Outputs and consumers

Topic params of the node: `image_topic`, `detections_topic`, `annotated_topic`, `masks_topic`,
`debug_image_topic`, `debug_clean_image_topic` (:306-372); launch sets the first four explicitly
(detection.launch.py:386-400).

| topic | consumer | use |
|---|---|---|
| `/camera/detections` (Detection2DArray, header = image header) | detection_3d_node (detection.launch.py:439-465) | 3-way sync, below |
| | front_clearance_node (:508-541) | object exclusion, below |
| | BT `mission/detected_classes_bridge.py:33-55` | blackboard class -> (last_seen, score) |
| | mission_logger (`mission_logger_node.py:263`), mission_extract, tools | recorded / offline |
| `/camera/detection_masks` (mono8, label i+1 = detections[i]; 0x0 on empty frames; **same header as the detections**) | detection_3d_node | mask-median depth |
| | front_clearance_node | binary object footprint, used **only if its stamp equals the latest detections' stamp** (`_raw_object_mask` :1515-1590) |
| | mission_logger (:265), `mission_overlay.py:104`, tools (`corridor_wall_replay.py`, `front_phantom_replay.py`, `latency_probe.py`, `mask_stage_profile.py`) | recorded / offline |
| `/camera/image_annotated` | `f1tenth_bringup` viz relays (`foxglove_bridge.launch.py:190-221` throttle -> `/camera/image_annotated/viz`, `viz_relays.yaml`, `viz_jpeg_node.py`), logger with `record_annotated_image` (:789) | display only |
| `/camera/debug_image/compressed` | mission_logger, `mission_debug_video.py`, tools | display / reports |
| `/camera/debug_image/clean/compressed` | mission_logger (:278, only while recording), `mission_overlay.py` | run-report stills |

Downstream of detection_3d: `/camera/detections_3d` -> obstacle_projector_node ->
`/perception/obstacles_2d` (MPC_corr `w_obs`, front_clearance), semantic_layer_node
(`/costmap/semantic_tracks`; needs one message per frame incl. empty ones), BT `is_obstacle_detected.py:43`,
`hard_stop.py`, lidar_front_wall_node (scan cut), the YOLO node's own debug image.

**Invariants the new code must keep:** one mask message per processed frame (0x0 when empty,
:660-694); identical header on detections and mask (front_clearance compares stamps exactly, detection_3d
syncs them); mask label = detection index + 1, later index wins on overlap, cap 254 (:207-211, :795-808);
mask resolution = image resolution = depth resolution (640x360).

### C.1 detection_3d_node: mask -> depth (`src/f1tenth_perception/f1tenth_perception/detection_3d_node.py`) [R]

- Sync: `message_filters.ApproximateTimeSynchronizer([detections, depth_registered, masks],
  queue_size=15, slop=0.1)` (:293-308, params :156-175); intrinsics cached from
  `/zed2/zed_node/depth/camera_info` (:498-503).
- Per detection i (:619-676): drop `score < confidence_threshold` 0.3 (:622); `z = _mask_median_depth`
  (use_mask_depth and a non-empty mask) else `_median_depth` (:635-641); drop `z is None or z <
  min_valid_depth` 0.2 (:642, :199); optical `x = (cx - K[2]) z / fx`, `y = (cy - K[5]) z / fy`,
  `width = box_w z / fx`, `height = box_h z / fy` (:646-649); pose transformed optical ->
  `zed2_left_camera_frame` with the static tf (:599-602, :657-663); `bbox.size = (width, height,
  default_depth_extent 0.3)` (:811-817); covariance from `_position_sigma(score, z)` (:764-786). Output
  header stamp = depth stamp.
- `_median_depth` (:684-703): centre `center_fraction` 0.5 of the box in each axis, ints clipped to
  the image, `valid = finite & > 0`, `np.median`.
- `_mask_median_depth` (:705-762): mask resized `INTER_NEAREST` if its shape differs from depth;
  `sel = (mask == i + 1) & isfinite(depth) & (depth > 0)`; empty -> `_median_depth` fallback; with
  scene labels on: `_scene_other` (:515-528) takes the `/perception/scene_labels` frame nearest the depth
  stamp within 0.3 s, keeps `LABEL_OTHER` pixels (nearest-neighbour lookup `rr * lh // h`); if >= 20 px
  the median over those, else the median over `sel`. **No erosion, no percentile.**
- Not pure functions (instance methods), but they only touch `self.center_fraction`, `scene_enabled`,
  `scene_max_age`, `scene_min_other_px`, `_scene_labels`, `_stats`, `get_logger()`, so they can be
  called unbound on a stub. **Validated [V]:** `/home/andreas/bags/yolo_gpu/frames_corridor10m_20261015T171753/tools/offline_3d.py`
  recomputes frame 15 of the extracted set and matches the recorded `/camera/detections_3d` to 4
  decimals (suitcase z 1.1100, cam (1.1100, -1.2252, 0.4286), size (0.2618, 0.1771); chair z 4.9866)
  with scene labels off. (Frame 7 differs because the live node paired a different depth frame.)
- Pure function downstream: `obstacle_projector_node.obstacle_radius(size_x, size_y, size_z, source,
  ...)` (`obstacle_projector_node.py:84`): `footprint` = max(size.x, size.z)/2 when size.z is measured,
  else size.x/2; `legacy` = max(size.x, size.y)/2.

### C.2 Offline recipe for the parity gate

Inputs per frame: detections `(class, score, cx, cy, w, h)` in 640x360 pixels, label mask M (mono8),
depth D (float32 m), depth camera_info K.
1. Drop `score < 0.3`.
2. `z_i = Detection3DNode._mask_median_depth(stub, D, M, i, cx, cy, w, h, 640, 360, depth_stamp=...)`
   (stub as in `offline_3d.py`; `scene_enabled=False`, or True with the recorded scene labels).
3. Drop `z is None or z < 0.2`.
4. Optical `x = (cx - K[2]) z / K[0]`, `y = (cy - K[5]) z / K[4]`; `zed2_left_camera_frame` =
   `(z, -x, -y)` (pure rotation, validated above); size `(w z / K[0], h z / K[4], 0.3)`.
5. Radius: `obstacle_radius(size_x, size_y, 0.3, source)`; base_link positions need
   `base_link -> zed2_left_camera_frame` from `/tf_static` (in the bag).
Compare per matched detection (same class, IoU or index): z, X/Y, size_x/size_y, radius; plus the
detection set itself (class, score, box) and the mask (IoU per instance).

---

## D. Threads, affinity, nice

- **No `torch.set_num_threads`, `cv2.setNumThreads`, OMP/MKL/OPENBLAS env for the YOLO node anywhere**
  (grep over src, tools, scripts, docker) [V].
- The only thread env in the stack: front_clearance_node `additional_env={'OPENBLAS_NUM_THREADS': '1'}`
  (detection.launch.py:513-518, measured 15 ms vs 0.06 ms per mat-vec with a 12-thread pool).
- Affinity is a `taskset -c` launch prefix (detection.launch.py:74-93 explains why self-pinning leaked
  threads): yolo `8,9` (:342-345, :384), detection_3d `6,7` (:349-352, :444), obstacle_projector `6,7`
  (:356-359, :476); lidar_front_wall `5` (lidar_front_wall.launch.py:235, :257), swept_clearance `5`
  (swept_clearance.launch.py:108, :122), wall_distance `5` (wall_distance.launch.py:187, :217),
  obstacle_clearance `5` (obstacle_clearance.launch.py:78, :89); others: ekf `0,1`, slam `2`, foxglove
  and viz relays `3`, behavior `4`, mpc_corr `10,11`. Unpinned: front_clearance, semantic_layer,
  costmap_renderer, costmap_boundary, the ZED container.
- nice: `cpu_affinity.py` `declare_nice_param`/`apply_nice` (os.nice of the main thread), launch args
  `yolo_nice` etc., default 0.
- Launch tests pin the taskset prefix of lidar_front_wall, swept_clearance and wall_distance
  (`test_*_launch.py`), not of detection.launch.py.
- Measured pool sizes [V]: under `taskset -c 0,1` on linus torch intra-op = 2 (affinity-aware),
  **inter-op = 8 (= cpu_count, not affinity-aware)**, `cv2.getNumThreads()` = 2; unpinned 8/8/8. Unpinned
  python on the Orin: torch 12/12. Live YOLO node: 16 threads.
- Python perception nodes using numpy/cv2 (candidates for thread-limit env vars):

| node | launch file : Node() | pinning / env | libs |
|---|---|---|---|
| yolo_detector_node | detection.launch.py:379 | taskset 8,9 | torch, cv2, numpy |
| detection_3d_node | detection.launch.py:439 | taskset 6,7 | numpy, cv2 |
| obstacle_projector_node | detection.launch.py:471 | taskset 6,7 | (no numpy import) |
| front_clearance_node | detection.launch.py:508 | unpinned, OPENBLAS_NUM_THREADS=1 | numpy, cv2 (heavy) |
| lidar_front_wall_node | lidar_front_wall.launch.py:251 | taskset 5 | numpy |
| swept_clearance_node | swept_clearance.launch.py:119 | taskset 5 | numpy |
| wall_distance_node | wall_distance.launch.py:214 | taskset 5 | numpy |
| obstacle_clearance_node | obstacle_clearance.launch.py:83 | taskset 5 | numpy |
| semantic_layer_node | f1tenth_costmap/launch/costmap.launch.py:159 | unpinned | numpy |
| costmap_boundary_node | costmap.launch.py:265 | unpinned | numpy |

  The lidar-side launch files are started as separate components by
  `f1tenth_bringup/f1tenth_bringup/component_supervisor_node.py`; detection and costmap via
  `stack_bringup.launch.py:181, :223`.

---

## E. Offline data

### E.1 What exists on the Orin [V]

All 331 `metadata.yaml` under `~` (depth 7) were read: `~/f1tenth_archive/{active,complete,mission_row,...}`
(mission logger, `mission_logger_runs_dir` default `~/f1tenth_archive`, stack_params.yaml:2697),
`~/bags`, `~/raw_data`, `~/.ros/mission_bags`, `~/f1tenth_data/runs` (empty dir). **No bag holds the
YOLO input at full rate.** Neither `/camera/image_raw_5hz` nor `/camera/debug_image/clean/compressed`
(campaign profile / new clean frame) was recorded in any of them. Only one bag holds clean
`/camera/image_raw` frames at all, and it is the only one with clean frames + depth + camera_info.

### E.2 Ranking

1. **`~/raw_data/corridor10m_20261015T171753`** (copied). sqlite3, 98.7 s, 523 MiB. `/camera/image_raw`
   **28** clean bgra8 640x360 frames (recorder kept bursts), depth `depth_registered` 31 +
   `depth_registered_5hz` 155 (32FC1), `/camera/camera_info` 152, depth camera_info 238,
   `/zed2/zed_node/left/camera_info` 104, plus the live outputs: `/camera/detections` 116 (99 non-empty,
   suitcase + chair), `/camera/detection_masks` 149, `/camera/detections_3d` 205,
   `/perception/obstacles_2d` 185, `/perception/scene_labels` 143, debug JPEG 159, `/tf_static`.
   Corridor with glass walls (stack_params.yaml:639 traced the self mask from these 28 frames).
   **No people.** Only candidate usable for a depth/distance parity gate.
2. `~/f1tenth_archive/active/2026-10-08T11-23-51_mission-llm_926f7913d041/bag` (~631 MB, 55 s):
   `/camera/image_annotated` 679 (bgr8 640x360 YOLO input **with the seg overlay drawn on frames that
   have detections**, clean on empty frames), detections/masks/detections_3d 679 each; no depth, no
   camera_info. Good for a throughput/CPU soak, unusable for depth parity, biased for detection parity.
   Sibling `complete/2026-10-08T11-27-18_mission-llm_926f7913d041` has 4190 such frames but is 3.8 GB.
3. `~/f1tenth_archive/complete/2026-09-08T12-27-37_mission-bottle_then_person/bag` (645 MB, 49 s):
   749 annotated frames from a person mission (by name; content not inspected), no depth. Same caveats
   as 2.

Not a bag, noted for completeness: SVO2 recordings `~/raw_data/2026-10-14T13-*_calib_wall_*/*.svo2`
(H265_LOSSLESS stereo at full rate) with replay bags holding `depth_registered` at ~16.7 Hz; static wall
scenes without objects, and decoding needs the ZED SDK (Orin only).

### E.3 Copy [V]

`rsync -a --partial --info=progress2 --rsync-path='nice -n 19 rsync'
jetson-orin:/home/fabiocar/raw_data/corridor10m_20261015T171753 /home/andreas/bags/yolo_gpu/` at 14:28:
548,529,513 B in 32 s. **Not bandwidth-limited:** the `ps` before the copy (14:23) showed no stack, but
fabiocar started one at ~14:24:40 and the `ps` printed in the same command as the rsync showed it running;
the copy ran unthrottled (16-36 MB/s over Tailscale, nice 19 on the Orin side) for 32 s while that stack
was up. Nothing else was copied or written on the Orin. `ros2 bag info` (Jazzy) reads it on linus.

### E.4 Frames

`/home/andreas/bags/yolo_gpu/frames_corridor10m_20261015T171753/` (52 MB), format in its `README.txt`:
- `index.json` with the requested fields plus `depth_topic`, `depth_offset_ms`,
  `depth_within_sync_slop`, `ref_detections` (live, exact stamp, 3 frames), `ref_mask`
  (`ref_masks/*.npy`, 5 frames), `ref_detections_3d` (4 frames), `scene_labels` (nearest).
- `rgb/NNNNNN.npy` uint8 (360, 640, 4) bgra8 exactly as published; `depth/NNNNNN.npy` float32 (360, 640)
  metres, nearest header stamp over both depth topics.
- 28 frames, 640x360, bgra8; source stream 30 Hz (stamps 33.3 ms apart inside the bursts).
- Depth pairing: offset 0 for 15 frames, |offset| <= 100 ms (the 0.1 s sync slop) for 25 of 28, max
  399.9 ms (frame 0).
- `yolo26s-seg.pt` on linus (deployed call, conf 0.3, retina_masks): suitcase + chair in 26 of 28
  frames, nothing in frames 26-27, **0 frames with a person**.
- Scripts next to the data (`tools/`): `extract_frames.py` (rosbag2_py), `offline_3d.py` (C.2 check),
  `probe_yolo.py` (output shapes, per-frame detections), `profile_call.py` (cProfile of the per-frame
  path).

**Shortfall:** 28 frames instead of 300-900, and no people. A larger set with people would need either
a new recording with `mission_logger_profile:=campaign` (records `/camera/image_raw_5hz`, full-res bgra8,
plus `depth_registered_5hz`) or the clean debug frame (480x270 JPEG only), or a second copy (candidate 2
or 3 above) accepting overlay-drawn frames and no depth.
