# Handoff: sim clock stutter + YOLO no-detections (2026-10-06)

Two-machine sim: Gazebo on **linus** (user `andreas`), Jazzy stack on the **Thor**
(`jetson-thor`, user `andre`, aarch64 / JetPack 7 / L4T R38.4), `ROS_DOMAIN_ID=42`,
Fast DDS Discovery Server at `10.42.0.2:11811`. Two separate problems surfaced;
one is essentially fixed, one is still open. Pick up from §"Open for tomorrow".

Probe caveat: everything below was measured **from linus** as a Discovery-Server
super client (`F1TENTH_DISCOVERY_SERVER=10.42.0.2:11811 ROS_DOMAIN_ID=42; source
scripts/env/jazzy.sh; ROS_SUPER_CLIENT=TRUE ...`). An ad-hoc super client needs
~30 s to be told the graph, so the first ~30 s of any probe reads as 0 — not a
real outage. The Thor itself is not reachable from linus (SSH needs a password;
no passwordless key).

---

## Issue 1 — sim clock "paused/running" flapping  → essentially FIXED

Symptom: supervisor spammed
`[health] sim clock paused: liveness checks suspended` /
`sim clock running again` every ~0.5–1.5 s.

Root cause (NOT the throttle, NOT the network): the Gazebo sim does not advance
time smoothly on linus — RTF swings wildly under render load. When a near-stall
lasts >0.5 s the supervisor's pause gate trips, then releases when it catches up.
Same mechanism as `output/sim_clock_fanout_followup.md`.

Measured with the GUI on (the bad case):
- RTF min **0.139**, max **2.681**, mean ~0.99 (n≈3500 over ~15 s)
- gz server 82 % CPU + **gz GUI another 54 %** (8 cores) — the GUI competes with
  the camera sensor rendering for the render loop.

What was done:
1. **4 ms physics step + /clock 250 Hz** (committed + pushed, see §Commits):
   single `<physics name="4ms">` block in both world files, `clock_rate`
   default 200→250. Fixed the *average* RTF (collapses at 1 ms on the 2× world).
2. **Run the sim headless** (`gui:=false`). This is the key operational fix.

Current state after relaunch (headless, `--headless-rendering -s`, camera on):
- No `gz sim gui` process (headless confirmed).
- RTF min **0.266**, max **2.853**, mean **1.015** — still swings, but no full
  stall.
- `/clock` max_gap **344 ms** — **under the 0.5 s pause gate**, so the flapping
  should be gone. `rate_hz=250`, throttle healthy.

Verdict: acceptable. Residual RTF swing is the camera (ZED rgb+depth+pointcloud)
+ 2× office world render cost on linus; it no longer produces >0.5 s stalls
headless. If the flapping returns, next levers (cheapest first): confirm GUI is
off, then `camera:=false` / lower ZED rate / drop the depth pointcloud.
**TODO (unconfirmed): verify on the Thor that the supervisor log stopped
flapping.**

---

## Issue 2 — YOLO: no annotated boxes, no detections, no tracking  → STILL OPEN

Symptom: `/camera/image_annotated` shows the raw frame with no boxes;
`/camera/detections` arrays are empty; `/camera/detections_3d`,
`/camera/detection_markers`, `/costmap/semantic_tracks` all silent. A chair was
clearly visible in `/camera/image_raw`.

Diagnosis (confirmed): the detector node ran in **passthrough mode**. Per
`yolo_detector_node.py`, if `model_path` is empty OR Ultralytics is not
importable OR the model fails to load, the node publishes an **empty
Detection2DArray** and re-publishes the incoming frame on the annotated topic —
exactly the observed symptom. The node's `model_path` was set
(`/home/andre/.../src/f1tenth_perception/models/yolo26s-seg.pt`, file present and
git-tracked) and `device=cuda`, so the trigger was the **import**:
on the Thor `python3 -c "import ultralytics"` → `ModuleNotFoundError`, and
`import torch` → `ModuleNotFoundError`. This is backlog **M9**
(perception needs torch/ultralytics).

Fix applied on the Thor (JetPack 7 / Thor / CUDA 13, Python 3.12, `/usr/bin/python3`):
```bash
PYBIN=/usr/bin/python3
$PYBIN -m pip install --user --break-system-packages ultralytics
$PYBIN -m pip install --user --break-system-packages --force-reinstall --no-deps \
  torch torchvision --index-url https://download.pytorch.org/whl/cu130
$PYBIN -c "import ultralytics, torch; print(torch.cuda.is_available())"   # -> cuda True
```
(Recipe source: Ultralytics NVIDIA-Jetson guide + PyTorch cu130 index. The
`pip install ultralytics` default torch is CPU-only on JP7, so torch+torchvision
are force-swapped for the cu130 build.) **Result: `cuda True`.**

Side effect to undo: the install bumped `setuptools` to 84.0.0 in
`~/.local`, which conflicts with `colcon-core` (<80). Runtime/launch unaffected,
but **before any `colcon build`** run:
```bash
/usr/bin/python3 -m pip install --user --break-system-packages "setuptools<80"
```

**Still open after installing deps + relaunching the whole stack:**
`/camera/detections` = **still 0 objects** (3 msgs/45 s, empty), `detections_3d`
= 0. So either:
1. the detector is **still in passthrough** (the stack's node isn't importing the
   just-installed `~/.local` ultralytics — e.g. a different user/python/env than
   the interactive shell where `cuda True` was confirmed, or a secondary import
   failure), OR
2. the detector **loaded YOLO but sees nothing** — note the car spawn was changed
   to **1 m ahead, 90° right** (`x:=1.0 yaw:=-1.5708`), so the chair may no longer
   be in frame (facing a wall / open floor), plus COCO-on-Gazebo is weak anyway.

### Decisive check for tomorrow (on the Thor)
```bash
python3 scripts/stackctl.py restart perception
# in the perception / yolo_detector_node log, look for ONE of:
#   'Loaded YOLO model ".../yolo26s-seg.pt" on cuda'   -> model loaded; it's a scene/aim issue
#   '... back to passthrough mode'                     -> still can't import ultralytics
```
- If **loaded**: aim the camera at an object — relaunch the sim with `yaw:=0`
  (or `yaw:=1.5708`) / drive toward the chair, and/or lower
  `confidence_threshold` (0.3 → 0.15). Check `/camera/image_raw` in Foxglove to
  see what it's actually looking at.
- If **passthrough**: compare the node's python/env to where ultralytics landed:
  `head -1 $(ros2 pkg prefix f1tenth_perception)/lib/f1tenth_perception/yolo_detector_node`
  and `python3 -c "import ultralytics; print(ultralytics.__file__)"` as `andre`.
  If the stack runs in a container/venv, `~/.local` won't be on its path — install
  into that env instead.

---

## Other changes made today (linus repo)

- **Spawn pose** (`sim_bringup.launch.py`, UNCOMMITTED): `x` 0.0→**1.0**,
  `yaw` 0.0→**-1.5708** (1 m ahead, 90° right). Affects the next launch; override
  with `x:=… yaw:=…`. Revert to `0.0`/`0.0` if the old spawn is wanted.
- Simulated LiDAR/ZED mounts moved earlier (committed, `e67910c`) — unrelated to
  these two issues.

## Commits pushed to origin/jazzy today
```
083c8e8 f1tenth_sim: 4 ms physics step, /clock 250 Hz
0d233e2 f1tenth_sim: throttle /clock (... -> /sim/clock_raw -> clock_throttle -> /clock), clock_probe, fan-out report
e67910c f1tenth_sim: move simulated LiDAR/ZED mounts (car TFs unchanged)
```
Uncommitted on linus: the spawn-pose change (above) + pre-existing unrelated work.

## Open for tomorrow (in order)
1. **YOLO**: run the decisive check above — loaded vs passthrough — and act on it.
   This is the real blocker.
2. Point the car at the chair (spawn `yaw:=0`) and confirm non-empty
   `/camera/detections` → `detections_3d` → tracking.
3. Confirm on the Thor that the clock-pause flapping is actually gone (headless).
4. Restore `setuptools<80` on the Thor before the next `colcon build`.
5. Decide whether to commit: (a) the spawn-pose change, (b) a short
   "JetPack 7: torch cu130 + ultralytics" install note in f1tenth_perception so a
   reflash doesn't lose it (closes M9).

## Quick probe recipe (from linus)
```bash
cd ~/dev_ws/f1tenth_more
export F1TENTH_DISCOVERY_SERVER=10.42.0.2:11811 ROS_DOMAIN_ID=42 ROS_SUPER_CLIENT=TRUE
source scripts/env/jazzy.sh
# clock:
timeout -s INT 50 python3 scripts/clock_probe.py --topic /clock --period 5
# RTF (gz, no warmup, run from a plain ROS shell):
gz topic -e -t /stats | grep real_time_factor
# detections content: /tmp/probe_all.py (see transcript) counts objects + labels
```
