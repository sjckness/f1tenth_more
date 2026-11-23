"""Static standing pose for the rigged NVIDIA people (no animation playback).

The characters under /Isaac/People/Characters ship in their bind T-pose (arms straight out, ~2 m span)
and the clips in /Isaac/People/Animations use another skeleton (Root/Pelvis/..., 0 joints in common with
the character's RL_BoneRoot/...), so they need retargeting (omni.anim.people). Instead a one-frame
UsdSkel animation is authored: the rest pose with each upper arm rotated down to hang beside the body,
``abduction_deg`` away from vertical. Kit skins it like any UsdSkel animation; nothing moves at run time.
"""
from __future__ import annotations

import logging
import math

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt

log = logging.getLogger(__name__)
T0 = Usd.TimeCode.Default()


def _skeleton(stage: Usd.Stage, root_path: str):
    for p in Usd.PrimRange(stage.GetPrimAtPath(root_path), Usd.TraverseInstanceProxies()):
        if p.IsA(UsdSkel.Skeleton):
            return UsdSkel.Skeleton(p)
    return None


def _skel_space(topo, local):
    out = [None] * len(local)
    for i, m in enumerate(local):   # joints are ordered parent-first
        p = topo.GetParent(i)
        out[i] = m if p < 0 else m * out[p]       # Gf row-vector convention: child = local * parent
    return out


def pose_arms_down(stage: Usd.Stage, root_path: str, abduction_deg: float = 8.0) -> bool:
    """Author and bind a static arms-down pose for the character under ``root_path``."""
    skel = _skeleton(stage, root_path)
    if skel is None:
        log.warning("person pose: no UsdSkel.Skeleton under %s", root_path)
        return False
    skel_root = UsdSkel.Root.Find(skel.GetPrim())
    cache = UsdSkel.Cache()
    cache.Populate(skel_root, Usd.TraverseInstanceProxies())
    q = cache.GetSkelQuery(skel)
    joints = list(q.GetJointOrder())
    topo = q.GetTopology()
    local = list(q.ComputeJointLocalTransforms(T0, True))      # at rest
    sk = _skel_space(topo, local)
    skel_to_world = UsdGeom.Xformable(skel.GetPrim()).ComputeLocalToWorldTransform(T0)
    down = skel_to_world.GetInverse().TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
    a = math.radians(abduction_deg)
    for side in ("L", "R"):
        try:
            iu = next(i for i, j in enumerate(joints) if j.endswith(f"/{side}_Upperarm"))
            ifa = next(i for i, j in enumerate(joints) if j.endswith(f"/{side}_Upperarm/{side}_Forearm"))
        except StopIteration:
            log.warning("person pose: no %s_Upperarm/%s_Forearm joints", side, side)
            return False
        pu, pf = sk[iu].ExtractTranslation(), sk[ifa].ExtractTranslation()
        d = (pf - pu).GetNormalized()
        out = d - down * Gf.Dot(d, down)
        out = out.GetNormalized() if out.GetLength() > 1e-6 else Gf.Vec3d(0, 0, 0)
        target = (down * math.cos(a) + out * math.sin(a)).GetNormalized()
        rot = Gf.Matrix4d().SetRotate(Gf.Rotation(d, target))
        pivot = Gf.Matrix4d().SetTranslate(-pu) * rot * Gf.Matrix4d().SetTranslate(pu)
        parent = topo.GetParent(iu)
        local[iu] = (sk[iu] * pivot) * sk[parent].GetInverse()
        sk = _skel_space(topo, local)
    anim = UsdSkel.Animation.Define(stage, skel_root.GetPath().AppendChild("StandingPose"))
    anim.CreateJointsAttr().Set(Vt.TokenArray(joints))
    anim.SetTransforms(Vt.Matrix4dArray(local), T0)
    UsdSkel.BindingAPI.Apply(skel.GetPrim()).CreateAnimationSourceRel().SetTargets([anim.GetPath()])
    log.info("person pose: arms down (%.0f deg from vertical), animation %s", abduction_deg, anim.GetPath())
    return True


def skinned_extent(stage: Usd.Stage, root_path: str, frame_path: str) -> tuple[np.ndarray, np.ndarray] | None:
    """(min, max) of the SKINNED mesh points in the frame of ``frame_path`` (BBoxCache ignores skinning)."""
    skel = _skeleton(stage, root_path)
    if skel is None:
        return None
    skel_root = UsdSkel.Root.Find(skel.GetPrim())
    cache = UsdSkel.Cache()
    cache.Populate(skel_root, Usd.TraverseInstanceProxies())
    world_to_frame = UsdGeom.Xformable(stage.GetPrimAtPath(frame_path)).ComputeLocalToWorldTransform(T0).GetInverse()
    pts_all = []
    for binding in cache.ComputeSkelBindings(skel_root, Usd.TraverseInstanceProxies()):
        sq = cache.GetSkelQuery(binding.GetSkeleton())
        xforms = sq.ComputeSkinningTransforms(T0)
        skel_to_frame = UsdGeom.Xformable(binding.GetSkeleton().GetPrim()).ComputeLocalToWorldTransform(T0) \
            * world_to_frame
        for target in binding.GetSkinningTargets():
            mesh = UsdGeom.Mesh(target.GetPrim())
            if not mesh:
                continue
            pts = mesh.GetPointsAttr().Get(T0)
            if pts is None or len(pts) == 0:
                continue
            pts = Vt.Vec3fArray(pts)
            if not target.ComputeSkinnedPoints(xforms, pts, T0):
                continue
            # skinned points are in skeleton space unless the mesh is rigidly deformed (then gprim space)
            m = np.array(skel_to_frame, dtype=float)
            p = np.asarray(pts, dtype=float)
            pts_all.append(p @ m[:3, :3] + m[3, :3])
    if not pts_all:
        return None
    p = np.concatenate(pts_all)
    return p.min(axis=0), p.max(axis=0)
