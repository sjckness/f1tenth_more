"""Walking people (optional, off by default): characters that follow a waypoint path.

Why not ``omni.anim.people``: that extension needs a navmesh, an animation graph and a command
file, and nothing in this repository can be run against Isaac on the machine where this code was
written (linus has no Isaac install). A waypoint walker is small enough to read and reason about,
needs no extension, and gives the stack what it actually consumes: a person-sized obstacle that
MOVES, with a collider the lidar hits, a mesh YOLO detects and a ground-truth pose to score
against. What it does not give is a walk CYCLE - the mesh glides along the path, legs still. For
visually correct walking, enable ``omni.anim.people`` later and drive these same waypoints
through its command file (noted in output/hil/setup.md).

Geometry: each character is a kinematic rigid body (so PhysX follows the teleport each step) with
an invisible capsule collider, the same radius/height as the static person in scene_builder.
"""
from __future__ import annotations

import logging
import math

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from sim.config import SimConfig
from sim.person_pose import pose_arms_down

log = logging.getLogger(__name__)

WORLD = "/World"


class WalkingPeople:
    """Characters moving along closed or open polylines at constant speed."""

    def __init__(self, stage: Usd.Stage, cfg: SimConfig, assets_root: str, app) -> None:
        from isaacsim.core.utils.stage import add_reference_to_stage

        from sim.scene_builder import resolve_usd

        self.stage = stage
        pc = cfg.scene["people"]
        self.default_usd = str(pc.get("usd", "/Isaac/People/Characters/F_Business_02/F_Business_02.usd"))
        self.asset_yaw_offset = math.radians(float(pc.get("asset_yaw_offset_deg", 90.0)))
        self.chars: list[dict] = []
        radius = float(pc.get("collider", {}).get("radius", 0.175))
        height = float(pc.get("collider", {}).get("height", 1.75))
        for i, spec in enumerate(pc.get("characters", []) or []):
            path_pts = np.array([[float(p[0]), float(p[1])] for p in spec["path"]], dtype=float)
            if len(path_pts) < 2:
                log.warning("people: character %d has fewer than 2 waypoints, skipped", i)
                continue
            root = f"{WORLD}/people/char_{i}"
            xf_prim = UsdGeom.Xform.Define(stage, root)
            xf = UsdGeom.Xformable(xf_prim)
            t_op = xf.AddTranslateOp()
            r_op = xf.AddRotateZOp()
            t_op.Set(Gf.Vec3d(float(path_pts[0][0]), float(path_pts[0][1]), 0.0))
            r_op.Set(0.0)
            # kinematic rigid body: PhysX tracks the transform we set every step
            rb = UsdPhysics.RigidBodyAPI.Apply(xf_prim.GetPrim())
            rb.CreateKinematicEnabledAttr(True)
            cap = UsdGeom.Capsule.Define(stage, f"{root}/collider")
            cap.CreateAxisAttr("Z")
            cap.CreateRadiusAttr(radius)
            cap.CreateHeightAttr(max(0.01, height - 2 * radius))
            UsdGeom.Xformable(cap).AddTranslateOp().Set(Gf.Vec3d(0, 0, height / 2))
            UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
            usd = resolve_usd(str(spec.get("usd", self.default_usd)), assets_root)
            visual_ok = False
            try:
                add_reference_to_stage(usd, f"{root}/visual")
                for _ in range(10):
                    app.update()
                if str(pc.get("pose", "arms_down")) == "arms_down":
                    pose_arms_down(stage, f"{root}/visual", float(pc.get("arm_abduction_deg", 4.0)))
                visual_ok = True
            except Exception as exc:  # noqa: BLE001 - a missing character asset must not kill the run
                log.warning("people: character %d asset %s failed (%s): capsule only", i, usd, exc)
            if visual_ok:
                UsdGeom.Imageable(cap).MakeInvisible()
                self._label(stage.GetPrimAtPath(f"{root}/visual"))
            self._label(cap.GetPrim())
            segs = np.linalg.norm(np.diff(path_pts, axis=0), axis=1)
            self.chars.append({
                "id": f"person_{i}", "root": root, "t_op": t_op, "r_op": r_op,
                "pts": path_pts, "seg": segs, "total": float(segs.sum()),
                "speed": float(spec.get("speed", pc.get("speed", 1.0))),
                "loop": bool(spec.get("loop", True)),
                "phase": float(spec.get("phase", 0.0)),
                "radius": radius, "height": height,
            })
            log.info("people: character %d on %d waypoints, %.1f m, %.2f m/s, loop=%s", i, len(path_pts),
                     float(segs.sum()), self.chars[-1]["speed"], self.chars[-1]["loop"])

    @staticmethod
    def _label(prim: Usd.Prim) -> None:
        from isaacsim.core.utils.semantics import add_labels

        add_labels(prim, ["person"], instance_name="class")

    def _at(self, ch: dict, t: float) -> tuple[float, float, float]:
        """Position and heading on the polyline at sim time ``t``."""
        total = ch["total"]
        if total <= 0.0:
            p = ch["pts"][0]
            return float(p[0]), float(p[1]), 0.0
        s = ch["speed"] * t + ch["phase"] * total
        if ch["loop"]:
            s = s % (2.0 * total)              # out and back: no teleport at the end
            if s > total:
                s = 2.0 * total - s
        else:
            s = min(s, total)
        acc = 0.0
        for i, seg in enumerate(ch["seg"]):
            if s <= acc + seg or i == len(ch["seg"]) - 1:
                u = 0.0 if seg <= 0 else (s - acc) / seg
                a, b = ch["pts"][i], ch["pts"][i + 1]
                p = a + np.clip(u, 0.0, 1.0) * (b - a)
                yaw = math.atan2(b[1] - a[1], b[0] - a[0])
                return float(p[0]), float(p[1]), yaw
            acc += seg
        p = ch["pts"][-1]
        return float(p[0]), float(p[1]), 0.0

    def update(self, t: float) -> None:
        for ch in self.chars:
            x, y, yaw = self._at(ch, t)
            ch["t_op"].Set(Gf.Vec3d(x, y, 0.0))
            ch["r_op"].Set(math.degrees(yaw + self.asset_yaw_offset))
            ch["pose"] = (x, y, yaw)

    def objects(self) -> list[dict]:
        """Ground-truth entries (/sim/ground_truth/objects), recomputed from the last update."""
        out = []
        for ch in self.chars:
            x, y, yaw = ch.get("pose", (float(ch["pts"][0][0]), float(ch["pts"][0][1]), 0.0))
            out.append({"id": ch["id"], "class": "person", "center": [x, y, ch["height"] / 2],
                        "size": [2 * ch["radius"], 2 * ch["radius"], ch["height"]],
                        "yaw": float(yaw + self.asset_yaw_offset)})
        return out


def build_people(stage: Usd.Stage, cfg: SimConfig, assets_root: str, app, info) -> WalkingPeople:
    people = WalkingPeople(stage, cfg, assets_root, app)
    people.update(0.0)
    info.people = people
    info.notes.append(f"{len(people.chars)} walking characters (kinematic waypoint walker, no walk cycle)")
    return people
