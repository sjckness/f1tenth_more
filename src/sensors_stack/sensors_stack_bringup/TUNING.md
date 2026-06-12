# ZED 2 tuning notes (Jetson Orin AGX)

The main runtime/quality knob is `depth_mode` in
[`config/zed2_params.yaml`](config/zed2_params.yaml). The ZED SDK computes depth on
the GPU, so `depth_mode` trades **GPU load + latency** against **depth accuracy and
density**. On the Jetson Orin AGX the GPU is shared with everything else in the
stack (perception, planning), so depth mode directly affects how much GPU headroom
is left for the rest of the pipeline.

## `depth_mode` options

| Mode | GPU / power | Latency | Depth quality | When to use |
|---|---|---|---|---|
| `PERFORMANCE` | **Lowest** | **Lowest** | Coarser, more holes at range | **Default on the car.** Leaves the most GPU for the rest of the stack; fine for nearby obstacle sensing at racing frame rates. |
| `QUALITY` | Medium | Medium | Denser, smoother | Mapping / careful low-speed runs where you can spare GPU and want cleaner depth/point clouds. |
| `ULTRA` (a.k.a. NEURAL on newer SDKs) | **Highest** | **Highest** | Best edges + fill, longest effective range | Offline/bench analysis or slow precise tasks; usually too heavy to run alongside a full racing stack in real time. |

### Practical guidance
- Start at **`PERFORMANCE`** (the configured default). Only raise it if depth is the
  bottleneck for your task *and* `tegrastats` shows GPU headroom to spare.
- Higher modes raise end-to-end latency, which hurts control loops that consume
  `~/odom` / depth. Watch the actual publish rate vs `pub_frame_rate: 30.0`.
- `grab_resolution` and `point_cloud_freq` also scale GPU/bandwidth cost. If you
  bump `depth_mode`, consider keeping `grab_resolution: HD720` and a modest
  `point_cloud_freq` (10.0 Hz here) to stay within budget.
- Use `tegrastats` (or `jtop`) while tuning to watch GPU%, EMC bandwidth and power.

## Related knobs
- `depth_stabilization: 1` — temporal smoothing of depth; cheap, keep on.
- `pub_frame_rate: 30.0` — caps how often topics are published (independent of the
  internal grab rate). Lower it to shed CPU/bandwidth if needed.

> TODO: LiDAR — once the LiDAR is integrated, document here how depth fusion vs
> LiDAR ranging should be balanced on the Jetson.
