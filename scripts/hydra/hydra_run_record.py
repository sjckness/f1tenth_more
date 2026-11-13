#!/usr/bin/env python3
"""hydra_run_record.py -- drive the validated office route and record what Hydra sees.

Started by scripts/hydra/hydra_run.sh, which brings up Hydra and the bag around it. Run it by
hand only if you already have a sim, Hydra and a DDS profile in the environment.

    scripts/hydra/hydra_run_record.py --out DIR [--segments route.json] [--no-drive]

WHAT IT WRITES into --out
    run.json                       the timeline: per segment the pose, the path length, what the
                                   guards did, and the graph counts sampled along the way
    mask/mask_tNNNNN.png           the GT semantic mask Hydra consumes, colourised (1 per second)
    scene_graph/dsg_tNNNNN.sparkdsg   graph snapshots, and dsg_final.sparkdsg at the end
                                   (the layout scripts/hydra/open_graph.sh already looks for, so
                                   `open_graph.sh <out>` just works)

THE GUARDS, and why they are not optional. The Leatherback's USD velocity drives have no force
limit, so a car that stalls against an obstacle while still being commanded builds unbounded wheel
torque until PhysX goes NaN (sim/isaac/config/car_spec.yaml:68, "office, 170 s stalled at
0.4 m/s"). Driving into furniture does not merely stop the car, it can corrupt the simulation.
    1. STALL -- commanded but not moving: the one that actually prevents the NaN.
    2. FRONT RANGE -- /sim/ground_truth/front_range below a threshold. A single ray along the
       heading, so it cannot see an obstacle offset sideways; forward motion only.
    3. NON-FINITE POSE -- by the time this fires the damage is done; it only stops us adding to it.
Any of them ends the drive, zeroes the command and is recorded in run.json.

The route itself is checked against the world's collision geometry BEFORE driving, by
sim/isaac/tools/check_drive_path.py, which is also where the default segment list lives.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))          # scripts/hydra -> scripts -> repo
ISAAC_DIR = os.path.join(REPO, "sim", "isaac")
TOOLS = os.path.join(ISAAC_DIR, "tools")
DEFAULT_ROUTE = os.path.join(ISAAC_DIR, "config", "routes", "office_corridor_loop.json")

# ros_checks reads WS at import time and we reuse its Checker (command publisher, sim clock,
# /sim/reset client, one-shot grab) instead of writing those again.
os.environ.setdefault("WS", ISAAC_DIR)
sys.path.insert(0, TOOLS)

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image, Range  # noqa: E402

from ros_checks import Checker, load_cfg, rel_qos  # noqa: E402


def load_route(path: str):
    """A route file is either a bare list of segments or {"segments": [...], ...}.

    Deliberately NOT imported from check_drive_path: that module pulls in the SDF parser and the
    occupancy grid, which have no business being loaded inside a ROS node.
    """
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    return (data.get("segments") or []) if isinstance(data, dict) else data

from semantic_palette import colourise as _colourise  # noqa: E402  (same colours as the video)

CMD_RATE = 20.0            # Hz, on sim time; the sim's command watchdog is 0.5 s
STALL_WINDOW_S = 1.5
STALL_MOVE_M = 0.02
FRONT_STOP_M = 0.8
JUMP_LIMIT_M = 1.0         # GT odom is 50 Hz at 0.3 m/s: a metre in one sample is a blow-up


def colourise(ids: np.ndarray):
    """mono8 class ids -> (PIL image, ids present), with the video's colours."""
    from PIL import Image as PILImage
    rgb, seen = _colourise(ids)
    return PILImage.fromarray(rgb, "RGB"), seen


class Runner(Checker):
    def __init__(self, cfg: dict, out: str, mask_period: float, graph_period: float) -> None:
        super().__init__(cfg)
        self.out = out
        self.mask_dir = os.path.join(out, "mask")
        self.graph_dir = os.path.join(out, "scene_graph")
        os.makedirs(self.mask_dir, exist_ok=True)
        os.makedirs(self.graph_dir, exist_ok=True)
        self.mask_period = mask_period
        self.graph_period = graph_period

        self.pose = None            # (x, y) latest plausible GT pose
        self.bad_pose = False
        self.diverged = None
        self.path_m = 0.0
        self.hist: deque = deque()  # (sim_time, x, y) for the stall guard
        self.front = float("inf")
        self.cmd_speed = 0.0
        self.t_start = None

        self.masks = 0
        self.mask_ids: set = set()
        self._last_mask = -1e9
        self.graphs = 0
        self._last_graph = -1e9
        self._graph_bytes = None
        self.graph_log: list = []

        t = self.t
        self.create_subscription(Odometry, t["gt_odom"], self._on_odom, rel_qos())
        self.create_subscription(Range, t["gt_front_range"], self._on_front, qos_profile_sensor_data)
        self.create_subscription(Image, t["gt_semantic"], self._on_mask, qos_profile_sensor_data)

        self._dsg = None
        try:
            import spark_dsg
            from hydra_msgs.msg import DsgUpdate
            self._dsg = spark_dsg
            self.create_subscription(DsgUpdate, "/hydra/backend/dsg", self._on_graph, 10)
            from f1tenth_hydra.dsg_tools import graph_summary
            self._summary = graph_summary
        except Exception as exc:  # noqa: BLE001
            print(f"[run] no Hydra graph recording ({type(exc).__name__}: {exc})", flush=True)
            self._summary = None

    # ---- inputs ---------------------------------------------------------------------------
    def _on_odom(self, m: Odometry) -> None:
        p = m.pose.pose.position
        if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)):
            self.bad_pose = True
            return
        # Divergence, caught BEFORE the values stop being finite. When the articulation blows up
        # the pose first grows absurd but still finite (we have seen 2.7e12), which sails past a
        # finiteness test and poisons the path length. GT odom runs at 50 Hz and the car does
        # 0.3 m/s, so anything over a metre between samples is not motion.
        if self.pose is not None and math.dist(self.pose, (p.x, p.y)) > JUMP_LIMIT_M:
            self.diverged = (f"pose jumped {math.dist(self.pose, (p.x, p.y)):.1f} m in one "
                             f"sample, from {self.pose} to ({p.x:.1f}, {p.y:.1f})")
            return
        if self.pose is not None:
            self.path_m += math.dist(self.pose, (p.x, p.y))
        self.pose = (p.x, p.y)
        if self.sim_time is not None:
            self.hist.append((self.sim_time, p.x, p.y))
            while self.hist and self.sim_time - self.hist[0][0] > STALL_WINDOW_S:
                self.hist.popleft()

    def _on_front(self, m: Range) -> None:
        self.front = float(m.range)

    def _on_mask(self, m: Image) -> None:
        if self.sim_time is None or self.sim_time - self._last_mask < self.mask_period:
            return
        self._last_mask = self.sim_time
        try:
            ids = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)[:, :, 0]
            img, seen = colourise(ids)
            rel = self.sim_time - (self.t_start if self.t_start is not None else self.sim_time)
            img.save(os.path.join(self.mask_dir, f"mask_t{rel:05.0f}.png"))
            self.masks += 1
            self.mask_ids.update(seen)
        except Exception as exc:  # noqa: BLE001
            print(f"[run] mask save failed: {exc}", flush=True)

    def _on_graph(self, msg) -> None:
        if not msg.full_update:
            return
        self._graph_bytes = msg.layer_contents.tobytes()
        if self.sim_time is None or self.sim_time - self._last_graph < self.graph_period:
            return
        self._last_graph = self.sim_time
        rel = self.sim_time - (self.t_start if self.t_start is not None else self.sim_time)
        self.save_graph(f"dsg_t{rel:05.0f}")

    def save_graph(self, name: str) -> None:
        """Same format and atomic rename as f1tenth_logger's SceneGraphSaver."""
        if self._graph_bytes is None or self._dsg is None:
            return
        final = os.path.join(self.graph_dir, name + ".sparkdsg")
        tmp = os.path.join(self.graph_dir, ".tmp_" + name + ".sparkdsg")
        try:
            g = self._dsg.DynamicSceneGraph.from_binary(self._graph_bytes)
            g.save(tmp, False)
            os.replace(tmp, final)
            self.graphs += 1
            if self._summary is not None:
                s = self._summary(g)
                self.graph_log.append({"name": name, "sim_t": round(self.sim_time, 2),
                                       "path_m": round(self.path_m, 2), "summary": s})
                L = s["layers"]
                g_ = lambda k: L.get(k, {}).get("nodes", 0)  # noqa: E731
                print(f"[graph] {name}  PLACES {g_('PLACES'):4d}  OBJECTS {g_('OBJECTS'):3d}  "
                      f"MESH_PL {g_('MESH_PLACES'):3d}  AGENTS {g_('AGENTS'):4d}  "
                      f"ROOMS {g_('ROOMS'):3d}  path {self.path_m:5.2f} m", flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"[run] graph save {name} failed: {exc}", flush=True)

    # ---- guards ---------------------------------------------------------------------------
    def guard(self) -> str | None:
        if self.diverged:
            return self.diverged
        if self.bad_pose:
            return "pose not finite"
        if abs(self.cmd_speed) > 0.05 and len(self.hist) >= 2:
            t0, x0, y0 = self.hist[0]
            if self.sim_time - t0 >= STALL_WINDOW_S:
                if math.dist((x0, y0), self.pose) < STALL_MOVE_M:
                    return (f"stalled: {math.dist((x0, y0), self.pose) * 100:.1f} cm in "
                            f"{STALL_WINDOW_S} s while commanding {self.cmd_speed:.2f} m/s")
        if self.cmd_speed > 0.05 and math.isfinite(self.front) and self.front < FRONT_STOP_M:
            return f"front range {self.front:.2f} m < {FRONT_STOP_M} m"
        return None

    # ---- the drive ------------------------------------------------------------------------
    def drive(self, segments) -> list:
        while self.sim_time is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.t_start = self.sim_time
        done = []
        for seg in segments:
            t0 = self.sim_time
            p0, s0 = self.pose, self.path_m
            self.cmd_speed = float(seg["speed"])
            # Start the stall window fresh. Without this a `settle` segment leaves 1.5 s of
            # standing still in the buffer, and the guard fires the instant the next segment asks
            # for speed -- which is exactly what happened on the first run.
            self.hist.clear()
            last = -1e9
            stop = None
            while self.sim_time - t0 < float(seg["dur"]):
                if self.sim_time - last >= 1.0 / CMD_RATE:
                    self.send(seg["speed"], seg["steer"])
                    last = self.sim_time
                rclpy.spin_once(self, timeout_sec=0.005)
                stop = self.guard()
                if stop:
                    break
            self.cmd_speed = 0.0
            rec = {"name": seg["name"], "speed": seg["speed"], "steer": seg["steer"],
                   "dur_s": seg["dur"], "ran_s": round(self.sim_time - t0, 2),
                   "from": [round(v, 3) for v in (p0 or (float('nan'),) * 2)],
                   "to": [round(v, 3) for v in (self.pose or (float('nan'),) * 2)],
                   "path_m": round(self.path_m - s0, 3), "front_m": round(self.front, 2)
                   if math.isfinite(self.front) else None, "guard": stop}
            done.append(rec)
            print(f"[seg] {seg['name']:12} {rec['ran_s']:6.1f}/{seg['dur']:.1f}s  "
                  f"{rec['path_m']:5.2f} m  -> {rec['to']}"
                  f"{'  GUARD: ' + stop if stop else ''}", flush=True)
            if stop:
                break
        for _ in range(40):            # zero, and keep saying it: the watchdog is 0.5 s
            self.send(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
        return done


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--scene", default="office")
    ap.add_argument("--segments", default=DEFAULT_ROUTE,
                    help="route JSON; default config/routes/office_corridor_loop.json")
    ap.add_argument("--mask-period", type=float, default=1.0, help="sim seconds between mask PNGs")
    ap.add_argument("--graph-period", type=float, default=10.0, help="sim seconds between graphs")
    ap.add_argument("--settle", type=float, default=8.0, help="wait this long before driving")
    ap.add_argument("--no-drive", action="store_true", help="record only, never command the car")
    ap.add_argument("--no-reset", action="store_true", help="do not call /sim/reset first")
    a = ap.parse_args()

    segments = load_route(a.segments)
    if not segments:
        raise SystemExit(f"{a.segments}: no segments")
    print(f"[run] route {os.path.basename(a.segments)}: {len(segments)} segments, "
          f"{sum(float(s['dur']) for s in segments):.1f} s", flush=True)

    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = Runner(load_cfg(a.scene), a.out, a.mask_period, a.graph_period)
    result = {"out": a.out, "scene": a.scene, "segments_requested": segments}
    try:
        if not a.no_reset:
            ok, msg = node.reset()
            print(f"[run] /sim/reset: {ok} {msg}", flush=True)
            result["reset"] = [ok, msg]
        print(f"[run] settling {a.settle:.0f} s (Hydra needs a few frames before it says anything)",
              flush=True)
        node.spin_for(a.settle)
        if a.no_drive:
            print("[run] --no-drive: recording for the nominal duration without commanding",
                  flush=True)
            node.t_start = node.sim_time
            node.spin_for(sum(float(s["dur"]) for s in segments))
            result["segments"] = []
        else:
            result["segments"] = node.drive(segments)
        print("[run] holding 5 s so the last frames land", flush=True)
        node.spin_for(5.0)
        node.save_graph("dsg_final")
        result.update(path_m=round(node.path_m, 3), masks=node.masks,
                      mask_ids_seen=sorted(node.mask_ids), graphs=node.graphs,
                      graph_log=node.graph_log, end_pose=node.pose,
                      pose_was_finite=not node.bad_pose,
                      guards_fired=[s["guard"] for s in result.get("segments", []) if s["guard"]])
    finally:
        with open(os.path.join(a.out, "run.json"), "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)
        node.destroy_node()
        rclpy.shutdown()

    print(f"\n[run] {result.get('path_m')} m driven, {result.get('masks')} masks, "
          f"{result.get('graphs')} graphs, mask ids {result.get('mask_ids_seen')}", flush=True)
    print(f"[run] guards: {result.get('guards_fired') or 'none fired'}", flush=True)
    print(f"[run] {a.out}/run.json", flush=True)
    return 0 if not result.get("guards_fired") and result.get("pose_was_finite") else 1


if __name__ == "__main__":
    raise SystemExit(main())
