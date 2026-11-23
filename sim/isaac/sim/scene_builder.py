"""Build scenes from ``config/scenes/<name>.yaml``.

Two scene types:
* ``procedural_room`` - floor + four walls (+ optional ceiling), lights, a static person.
* ``usd_environment`` - an NVIDIA environment referenced from the asset server, plus the person.
Every collider used by the lidar is a PhysX collider; semantic class labels are added with
``isaacsim.core.utils.semantics.add_labels``.
"""
from __future__ import annotations

import fnmatch
import logging
import math
import os
from dataclasses import dataclass, field

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

from sim.config import WS, SimConfig, get
from sim.person_pose import pose_arms_down, skinned_extent

log = logging.getLogger(__name__)

WORLD = "/World"

# Vertical structure of the NVIDIA environments, by mesh-name pattern (lower case, fnmatch). Used for the
# collider audit (tools/office_colliders.py) and for environment.vertical_colliders (scene YAML).
VERTICAL_CATEGORIES = {
    "wall": ["*wall*"],
    "partition": ["*partition*", "*divider*"],
    "door": ["*door*"],
    "glass": ["*glass*", "*window*"],
    "column": ["*column*", "*pillar*"],
    "furniture": ["*desk*", "*table*", "*cabinet*", "*shelf*", "*shelves*", "*sofa*", "*couch*", "*armchair*",
                  "*locker*", "*bookcase*", "*counter*", "*reception*", "*fridge*", "*printer*", "*drawer*"],
}


def resolve_usd(path: str, assets_root: str) -> str:
    """Scene asset reference -> something ``add_reference_to_stage`` can open.

    * ``/Isaac/...``            on the NVIDIA asset server (``assets_root`` prepended), as before;
    * ``omniverse://``, ``http(s)://``, absolute path   used as given;
    * anything else            relative to this sim directory (``sim/isaac`` in the car repo), so a
      wrapper USD committed next to the scene files is found after a plain ``git clone``.

    A referenced asset is never modified: USD composition only reads it.
    """
    if path.startswith(("omniverse://", "http://", "https://")):
        return path
    if path.startswith("/Isaac/"):
        return f"{assets_root}{path}"
    if os.path.isabs(path):
        return path
    return os.path.join(WS, path)


@dataclass
class SceneInfo:
    """What the rest of the sim needs to know about the built scene."""

    person_path: str | None = None
    person_pose: tuple[np.ndarray, float] | None = None      # (xyz, facing yaw)
    person_asset_yaw: float = 0.0                              # rotateZ applied to the prim (facing + asset offset)
    person_fallback: bool = False
    person_extent: tuple | None = None                         # (min, max) xyz of the posed mesh, person frame
    people: object | None = None                               # sim.people.WalkingPeople (scene 'people' block)
    # ground-truth objects (/sim/ground_truth/objects): {id, class, center [xyz], size [xyz], yaw}
    objects: list[dict] = field(default_factory=list)
    labels_report: dict = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)


def _material(stage: Usd.Stage, path: str, color, roughness: float, metallic: float,
              opacity: float = 1.0, ior: float | None = None) -> UsdShade.Material:
    """UsdPreviewSurface. ``opacity`` < 1 (+ ``ior``) is how the glass wall is drawn: the camera
    sees through it, the PhysX collider on the same prim still stops the lidar beam - which is
    exactly the real failure mode the glass detector exists for."""
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    if opacity < 1.0:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
        sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(float(ior if ior is not None else 1.52))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _materials(stage: Usd.Stage, specs: dict) -> dict:
    """``materials:`` block of a scene file -> {name: UsdShade.Material} under /World/Looks."""
    looks = f"{WORLD}/Looks"
    UsdGeom.Scope.Define(stage, looks)
    return {k: _material(stage, f"{looks}/{k}", v["color"], v["roughness"], v["metallic"],
                         float(v.get("opacity", 1.0)), v.get("ior"))
            for k, v in specs.items()}


def physics_material(stage: Usd.Stage, path: str, static: float, dynamic: float, restitution: float):
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(float(static))
    api.CreateDynamicFrictionAttr(float(dynamic))
    api.CreateRestitutionAttr(float(restitution))
    return mat


def bind_physics_material(prim: Usd.Prim, mat: UsdShade.Material) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.strongerThanDescendants, "physics")


def _box(stage: Usd.Stage, path: str, center, size, material: UsdShade.Material | None) -> Usd.Prim:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
    return cube.GetPrim()


def _label(prim: Usd.Prim, cls: str) -> None:
    from isaacsim.core.utils.semantics import add_labels

    add_labels(prim, [cls], instance_name="class")


def build_scene(stage: Usd.Stage, cfg: SimConfig, assets_root: str, app) -> SceneInfo:
    """Create the scene described by ``cfg.scene`` on ``stage``."""
    sc = cfg.scene
    UsdGeom.Xform.Define(stage, WORLD)
    info = SceneInfo()
    if sc["type"] == "procedural_room":
        info.objects.extend(_build_room(stage, cfg))
        info.labels_report = {"added": ["floor", "wall_0..wall_3 (class wall)"], "existing": "none (procedural)"}
    elif sc["type"] == "procedural_corridor":
        info.objects.extend(_build_corridor(stage, cfg, info))
        info.labels_report = {"added": ["floor", "wall_* (class wall)", "glass_* (class glass)"],
                              "existing": "none (procedural)"}
    else:
        info.labels_report = _build_environment(stage, cfg, assets_root, app)
    _lights(stage, sc)
    if get(sc, "person.enabled"):
        _person(stage, cfg, assets_root, app, info)
    if (sc.get("people") or {}).get("enabled"):
        from sim.people import build_people

        build_people(stage, cfg, assets_root, app, info)
    if (sc.get("plant") or {}).get("enabled"):
        _plant(stage, cfg, assets_root, app, info)
    if (sc.get("obstacle") or {}).get("enabled"):
        _obstacle(stage, cfg, info)
    for o in info.objects:
        log.info("object %-8s %-13s center (%.2f, %.2f, %.2f) size (%.2f, %.2f, %.2f) yaw %.0f deg", o["id"], o["class"],
                 *o["center"], *o["size"], math.degrees(o["yaw"]))
    return info


def _bound(stage: Usd.Stage, path: str):
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = bb.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    return None if rng.IsEmpty() else (np.array(rng.GetMin(), dtype=float), np.array(rng.GetMax(), dtype=float))


def _plant(stage: Usd.Stage, cfg: SimConfig, assets_root: str, app, info: SceneInfo) -> None:
    """Static potted plant from the NVIDIA library: uniform scale, static triangle-mesh collider (lidar)."""
    from isaacsim.core.utils.stage import add_reference_to_stage

    pc = cfg.scene["plant"]
    path = f"{WORLD}/plant"
    root = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(root)
    x, y = (float(v) for v in pc["position"][:2])
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(float(pc.get("yaw_deg", 0.0)))
    xf.AddScaleOp().Set(Gf.Vec3f(*[float(pc.get("scale", 1.0))] * 3))
    add_reference_to_stage(resolve_usd(str(pc["usd"]), assets_root), f"{path}/visual")
    for _ in range(5):
        app.update()
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
        if prim.IsA(UsdGeom.Mesh):
            # static (no rigid body): an exact triangle-mesh collider is allowed and the lidar sees the real
            # pot and leaf outline at its height, like the car's UST-10LX would
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
            n += 1
    cls = str(pc.get("label", "potted plant"))
    _label(stage.GetPrimAtPath(path), cls)
    b = _bound(stage, path)
    if b is None:
        log.error("plant %s: empty bound (asset not loaded?)", pc["usd"])
        return
    mn, mx = b
    log.info("plant %s scale %.2f: %.3f x %.3f x %.3f m, %d mesh colliders, label '%s'", pc["usd"],
             float(pc.get("scale", 1.0)), *(mx - mn), n, cls)
    info.notes.append(f"plant bbox {(mx - mn)[0]:.2f} x {(mx - mn)[1]:.2f} x {(mx - mn)[2]:.2f} m")
    info.objects.append({"id": "plant", "class": cls, "center": ((mn + mx) / 2).tolist(), "size": (mx - mn).tolist(),
                         "yaw": 0.0})


def _obstacle(stage: Usd.Stage, cfg: SimConfig, info: SceneInfo) -> None:
    """Static box obstacle (cardboard-like), box collider."""
    oc = cfg.scene["obstacle"]
    size = [float(v) for v in oc["size"]]
    x, y = (float(v) for v in oc["position"][:2])
    yaw = float(oc.get("yaw_deg", 0.0))
    looks = f"{WORLD}/Looks"
    UsdGeom.Scope.Define(stage, looks)
    mat = _material(stage, f"{looks}/obstacle", oc.get("color", [0.55, 0.42, 0.28]), 0.85, 0.0)
    prim = _box(stage, f"{WORLD}/obstacle", (x, y, size[2] / 2), size, mat)
    UsdGeom.Xformable(prim).AddRotateZOp().Set(yaw)   # after translate, before scale: rotates the unit cube
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    xf.SetXformOpOrder([ops[0], ops[2], ops[1]])        # translate, rotateZ, scale
    cls = str(oc.get("label", "box"))
    _label(prim, cls)
    info.objects.append({"id": "obstacle", "class": cls, "center": [x, y, size[2] / 2], "size": size,
                         "yaw": math.radians(yaw)})


def _build_room(stage: Usd.Stage, cfg: SimConfig) -> list[dict]:
    sc = cfg.scene
    sx, sy = get(sc, "room.size_x"), get(sc, "room.size_y")
    h, t = get(sc, "room.wall_height"), get(sc, "room.wall_thickness")
    looks = f"{WORLD}/Looks"
    shader = _materials(stage, sc["materials"])
    floor_mat = shader[get(sc, "floor.material")]
    wall_mat = shader.get("wall_white")
    fr = get(cfg.car, "friction")
    phys = physics_material(stage, f"{looks}/floor_physics", get(fr, "static"), get(fr, "dynamic"),
                            get(fr, "restitution"))
    margin = 1.0
    floor = _box(stage, f"{WORLD}/room/floor", (sx / 2, sy / 2, -0.05), (sx + 2 * margin, sy + 2 * margin, 0.1),
                 floor_mat)
    bind_physics_material(floor, phys)
    _label(floor, "floor")
    walls = [  # inner faces at x=0, x=sx, y=0, y=sy
        ((-t / 2, sy / 2, h / 2), (t, sy + 2 * t, h)),
        ((sx + t / 2, sy / 2, h / 2), (t, sy + 2 * t, h)),
        ((sx / 2, -t / 2, h / 2), (sx, t, h)),
        ((sx / 2, sy + t / 2, h / 2), (sx, t, h)),
    ]
    objects = []
    for i, (c, s) in enumerate(walls):
        _label(_box(stage, f"{WORLD}/room/wall_{i}", c, s, wall_mat), "wall")
        objects.append({"id": f"wall_{i}", "class": "wall", "center": list(c), "size": list(s), "yaw": 0.0})
    if get(sc, "room.ceiling"):
        _box(stage, f"{WORLD}/room/ceiling", (sx / 2, sy / 2, h + 0.05), (sx + 2 * t, sy + 2 * t, 0.1), wall_mat)
    log.info("room %.2f x %.2f m, walls %.2f m high, %.2f m thick", sx, sy, h, t)
    return objects


def _build_environment(stage: Usd.Stage, cfg: SimConfig, assets_root: str, app) -> dict:
    """Reference an NVIDIA environment, report its labels, add floor/wall labels and colliders."""
    from isaacsim.core.utils.semantics import get_labels
    from isaacsim.core.utils.stage import add_reference_to_stage

    sc = cfg.scene
    env_path = f"{WORLD}/env"
    env_usd = resolve_usd(str(get(sc, "environment.usd")), assets_root)
    log.info("environment USD: %s", env_usd)
    add_reference_to_stage(env_usd, env_path)
    for _ in range(10):
        app.update()
    existing: dict[str, int] = {}
    n_mesh = n_coll = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(env_path)):
        try:
            for _inst, labels in (get_labels(prim) or {}).items():
                for lab in labels:
                    existing[lab] = existing.get(lab, 0) + 1
        except Exception:  # noqa: BLE001 - older label schema on some prims
            pass
        # (legacy SemanticsAPI labels: none in office.usd, checked with tools/office_probe.py)
        if prim.IsA(UsdGeom.Mesh):
            n_mesh += 1
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                n_coll += 1
    added: dict[str, int] = {}
    patterns: dict = sc["environment"]["labels"]
    add_colliders = bool(sc["environment"].get("add_colliders", True))
    # only these (non-instanced) meshes get a collider; adding colliders to everything also hits huge
    # backdrop / degenerate meshes (PhysX "Illegal BroadPhaseUpdateData" and NaN states were observed)
    coll_patterns = [p.lower() for p in sc["environment"].get("collider_patterns", ["*"])]
    fr = get(cfg.car, "friction")
    phys = physics_material(stage, f"{WORLD}/Looks/floor_physics", get(fr, "static"), get(fr, "dynamic"),
                            get(fr, "restitution"))
    # Static vertical structure (walls, partitions, doors, glass, columns, large furniture): category ->
    # PhysX approximation. Only non-instanced meshes can be edited; instanced ones keep their prototype's.
    vc = sc["environment"].get("vertical_colliders") or {}
    vert = dict(vc.get("approximation", {})) if vc.get("enabled") else {}
    vert_limits = {"max_size_m": float(vc.get("max_size_m", 40.0)), "min_height_m": float(vc.get("min_height_m", 0.3)),
                   "min_thickness_m": float(vc.get("min_thickness_m", 0.002))}
    vert_added: dict[str, int] = {}
    vert_skipped: list[dict] = []
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    for prim in Usd.PrimRange(stage.GetPrimAtPath(env_path)):
        name = prim.GetName().lower()
        path = str(prim.GetPath()).lower()
        for cls, pats in patterns.items():
            if any(fnmatch.fnmatch(name, p) for p in pats) and (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Xform)):
                # label the highest matching prim only
                parent = prim.GetParent()
                if parent and any(fnmatch.fnmatch(parent.GetName().lower(), p) for p in pats):
                    continue
                _label(prim, cls)
                added[cls] = added.get(cls, 0) + 1
                if cls == "floor":
                    bind_physics_material(prim, phys)
        if (add_colliders and prim.IsA(UsdGeom.Mesh) and not prim.HasAPI(UsdPhysics.CollisionAPI)
                and any(fnmatch.fnmatch(name, p) for p in coll_patterns)):
            UsdPhysics.CollisionAPI.Apply(prim)
            mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mc.CreateApproximationAttr("none")   # static triangle mesh
            _ = path
        if prim.IsA(UsdGeom.Mesh) and not prim.HasAPI(UsdPhysics.CollisionAPI) and vert:
            cat = next((c for c, pats in VERTICAL_CATEGORIES.items() if c in vert
                        and any(fnmatch.fnmatch(name, p) for p in pats)), None)
            if cat is not None:
                ok, why = _sane_for_collider(bbc, prim, vert_limits)
                if ok:
                    UsdPhysics.CollisionAPI.Apply(prim)
                    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(vert[cat])
                    vert_added[cat] = vert_added.get(cat, 0) + 1
                else:
                    vert_skipped.append({"path": str(prim.GetPath()), "category": cat, "why": why})
    # Heavy parts off (GPU budget: the HIL workstation has an RTX 5050, 8 GB). Pattern -> the prim is
    # made invisible AND loses its collider, so it costs neither raster nor PhysX work. Use for
    # foliage, ceiling-light arrays, outdoor backdrops: --set scene.environment.hide_patterns=['*foliage*']
    hidden: dict[str, int] = {}
    for pat in [str(x).lower() for x in (sc["environment"].get("hide_patterns") or [])]:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(env_path)):
            if fnmatch.fnmatch(prim.GetName().lower(), pat):
                UsdGeom.Imageable(prim).MakeInvisible()
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
                hidden[pat] = hidden.get(pat, 0) + 1
    if hidden:
        log.info("environment: hidden prims per pattern %s", hidden)
    n_added = sum(1 for prim in Usd.PrimRange(stage.GetPrimAtPath(env_path))
                  if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI)) - n_coll
    report = {"hidden": hidden,
              "meshes_non_instanced": n_mesh, "meshes_with_collision_before": n_coll, "existing_labels": existing,
              "added_labels": added, "colliders_added": n_added, "collider_patterns": coll_patterns,
              "vertical_colliders_added": vert_added, "vertical_skipped": vert_skipped[:50],
              "vertical_approximation": vert}
    log.info("environment labels: %s", report)
    return report


def _sane_for_collider(bbc, prim: Usd.Prim, lim: dict) -> tuple[bool, str]:
    """Skip what broke PhysX before (D20): huge backdrops and degenerate (flat/empty) meshes."""
    rng = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        return False, "empty bound"
    size = rng.GetMax() - rng.GetMin()
    if max(size) > lim["max_size_m"]:
        return False, f"too large {max(size):.1f} m"
    if size[2] < lim["min_height_m"]:
        return False, f"too low {size[2]:.2f} m"
    if sorted(size)[1] < lim["min_thickness_m"]:
        return False, "degenerate"
    pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
    if pts is None or len(pts) < 4 or not np.all(np.isfinite(np.asarray(pts, dtype=float))):
        return False, "no/invalid points"
    return True, ""


def _lights(stage: Usd.Stage, sc: dict) -> None:
    lt = sc["lighting"]
    shadows = bool(lt.get("shadows", False))
    dome_i = lt.get("dome_intensity", lt.get("extra_dome_intensity", 0.0))
    if dome_i:
        dome = UsdLux.DomeLight.Define(stage, f"{WORLD}/lights/dome")
        dome.CreateIntensityAttr(float(dome_i))
        UsdLux.ShadowAPI.Apply(dome.GetPrim()).CreateShadowEnableAttr(shadows)
    for i, sl in enumerate(lt.get("sphere_lights", []) or []):
        sp = UsdLux.SphereLight.Define(stage, f"{WORLD}/lights/sphere_{i}")
        sp.CreateIntensityAttr(float(sl["intensity"]))
        sp.CreateRadiusAttr(float(sl.get("radius", 0.15)))
        UsdGeom.Xformable(sp).AddTranslateOp().Set(Gf.Vec3d(*map(float, sl["position"])))
        UsdLux.ShadowAPI.Apply(sp.GetPrim()).CreateShadowEnableAttr(shadows)
    if lt.get("distant_intensity"):
        d = UsdLux.DistantLight.Define(stage, f"{WORLD}/lights/distant")
        d.CreateIntensityAttr(float(lt["distant_intensity"]))
        pitch, yaw = lt.get("distant_angle_deg", [-45.0, 0.0])
        UsdGeom.Xformable(d).AddRotateXYZOp().Set(Gf.Vec3f(float(90 + pitch), 0.0, float(yaw)))
        UsdLux.ShadowAPI.Apply(d.GetPrim()).CreateShadowEnableAttr(shadows)


def _extent_str(ext) -> str:
    mn, mx = ext
    sz = mx - mn
    return (f"{sz[0]:.3f} x {sz[1]:.3f} x {sz[2]:.3f} m (width x depth x height, person frame; "
            f"z {mn[2]:.3f}..{mx[2]:.3f})")


def person_yaw(sc: dict) -> float:
    """Yaw [rad] of the person: facing the car start when ``face_car``."""
    px, py = get(sc, "person.position")[:2]
    if get(sc, "person.face_car"):
        return math.atan2(get(sc, "car.start.y") - py, get(sc, "car.start.x") - px)
    return math.radians(get(sc, "person.yaw_deg"))


def _person(stage: Usd.Stage, cfg: SimConfig, assets_root: str, app, info: SceneInfo) -> None:
    """Static human: referenced character mesh (no animation) + invisible capsule collider."""
    from isaacsim.core.utils.stage import add_reference_to_stage

    sc = cfg.scene
    pos = np.array(get(sc, "person.position"), dtype=float)
    yaw = person_yaw(sc)
    root = UsdGeom.Xform.Define(stage, f"{WORLD}/person")
    xf = UsdGeom.Xformable(root)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    asset_yaw = yaw + math.radians(float(sc["person"].get("asset_yaw_offset_deg", 0.0)))
    xf.AddRotateZOp().Set(math.degrees(asset_yaw))
    r, height = get(sc, "person.collider.radius"), get(sc, "person.collider.height")
    cap = UsdGeom.Capsule.Define(stage, f"{WORLD}/person/collider")
    cap.CreateAxisAttr("Z")
    cap.CreateRadiusAttr(float(r))
    cap.CreateHeightAttr(float(height - 2 * r))
    UsdGeom.Xformable(cap).AddTranslateOp().Set(Gf.Vec3d(0, 0, height / 2))
    UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
    ok = False
    try:
        add_reference_to_stage(resolve_usd(str(get(sc, "person.usd")), assets_root),
                               f"{WORLD}/person/visual")
        for _ in range(10):
            app.update()
        vis = f"{WORLD}/person/visual"
        before = skinned_extent(stage, vis, f"{WORLD}/person")
        if before is not None:
            log.info("person asset as shipped (bind pose): %s", _extent_str(before))
            info.notes.append(f"person as shipped {_extent_str(before)}")
        if sc["person"].get("pose", "arms_down") == "arms_down":
            pose_arms_down(stage, vis, float(sc["person"].get("arm_abduction_deg", 4.0)))
        ext = skinned_extent(stage, vis, f"{WORLD}/person")
        if ext is not None:
            # footprint in the person's own frame (x across the shoulders, y front-back), z above the floor
            info.person_extent = ext
            log.info("person bbox size %s", _extent_str(ext))
            info.notes.append(f"person bbox {_extent_str(ext)}")
            ok = 1.2 < float(ext[1][2] - ext[0][2]) < 2.2
            if not ok:
                info.notes.append("person asset has implausible height -> capsule fallback")
    except Exception as exc:  # noqa: BLE001
        log.exception("person asset failed to load: %s", exc)
    if ok:
        UsdGeom.Imageable(cap).MakeInvisible()
        _label(stage.GetPrimAtPath(f"{WORLD}/person/visual"), "person")
    else:
        stage.RemovePrim(f"{WORLD}/person/visual")
        info.person_fallback = True
        log.warning("using capsule fallback for the person")
    _label(cap.GetPrim(), "person")
    info.person_path = f"{WORLD}/person"
    info.person_pose = (pos, yaw)
    info.person_asset_yaw = asset_yaw
    if info.person_extent is not None and not info.person_fallback:
        mn, mx = info.person_extent
        c_local = (mn + mx) / 2
        ca, sa = math.cos(asset_yaw), math.sin(asset_yaw)
        center = [pos[0] + ca * c_local[0] - sa * c_local[1], pos[1] + sa * c_local[0] + ca * c_local[1],
                  pos[2] + c_local[2]]
        size = (mx - mn).tolist()
    else:
        center, size = [pos[0], pos[1], pos[2] + height / 2], [2 * r, 2 * r, height]
    # yaw = the asset's x axis (across the shoulders): size[0] is the width, size[1] the depth
    info.objects.append({"id": "person", "class": "person", "center": [float(v) for v in center], "size": size,
                         "yaw": float(asset_yaw)})


def _wall_segments(span: tuple[float, float], hole: tuple[float, float] | None) -> list[tuple[float, float]]:
    """``span`` minus ``hole`` -> the pieces that stay solid (empty list if the hole covers it)."""
    a, b = span
    if hole is None:
        return [(a, b)]
    h0, h1 = max(a, min(hole)), min(b, max(hole))
    if h1 <= h0:
        return [(a, b)]
    return [seg for seg in ((a, h0), (h1, b)) if seg[1] - seg[0] > 1e-6]


def _build_corridor(stage: Usd.Stage, cfg: SimConfig, info: SceneInfo) -> list[dict]:
    """Straight corridor along +x: interior x in [0, length], y in [0, width].

    The floor material is the point of this scene: ``glossy_grey`` is the real building's polished
    floor, on which the car's ZED returns almost no depth. Isaac's depth annotator is geometric and
    does NOT degrade on a shiny material, so the scene also turns on the floor-NaN depth model
    (``sensor_overrides`` in scenes/corridor.yaml, applied by sim.config) - that, not the material,
    is what reproduces the real behaviour. The material keeps the RGB image honest (specular
    highlights, which is what YOLO and the scene labels see).

    ``glass`` replaces a span of one side wall with a transparent panel that still has a collider:
    invisible to the camera, solid to the lidar.
    """
    sc = cfg.scene
    cc = sc["corridor"]
    L, W = float(get(sc, "corridor.length")), float(get(sc, "corridor.width"))
    h, t = float(get(sc, "corridor.wall_height")), float(get(sc, "corridor.wall_thickness"))
    shader = _materials(stage, sc["materials"])
    floor_mat = shader[get(sc, "floor.material")]
    wall_mat = shader.get("wall_white")
    fr = get(cfg.car, "friction")
    phys = physics_material(stage, f"{WORLD}/Looks/floor_physics", get(fr, "static"), get(fr, "dynamic"),
                            get(fr, "restitution"))
    margin = 1.0
    floor = _box(stage, f"{WORLD}/corridor/floor", (L / 2, W / 2, -0.05),
                 (L + 2 * margin, W + 2 * margin, 0.1), floor_mat)
    bind_physics_material(floor, phys)
    _label(floor, "floor")
    glass = cc.get("glass") or {}
    glass_on = bool(glass.get("enabled"))
    glass_side = str(glass.get("side", "left"))
    hole = (float(glass["x_from"]), float(glass["x_to"])) if glass_on else None
    objects: list[dict] = []
    # side walls: 'left' is +y (the car drives +x), 'right' is y = 0
    for side, y_c in (("right", -t / 2), ("left", W + t / 2)):
        segs = _wall_segments((0.0 - t, L + t), hole if glass_on and glass_side == side else None)
        for i, (x0, x1) in enumerate(segs):
            c = ((x0 + x1) / 2, y_c, h / 2)
            sz = (x1 - x0, t, h)
            _label(_box(stage, f"{WORLD}/corridor/wall_{side}_{i}", c, sz, wall_mat), "wall")
            objects.append({"id": f"wall_{side}_{i}", "class": "wall", "center": list(c), "size": list(sz),
                            "yaw": 0.0})
    if bool(cc.get("end_walls", True)):
        for name, x_c in (("start", -t / 2), ("end", L + t / 2)):
            c = (x_c, W / 2, h / 2)
            sz = (t, W + 2 * t, h)
            _label(_box(stage, f"{WORLD}/corridor/wall_{name}", c, sz, wall_mat), "wall")
            objects.append({"id": f"wall_{name}", "class": "wall", "center": list(c), "size": list(sz),
                            "yaw": 0.0})
    if glass_on:
        gm = shader.get("glass_clear") or wall_mat
        y_c = -t / 2 if glass_side == "right" else W + t / 2
        x0, x1 = hole                      # type: ignore[misc]
        gh = float(glass.get("height", h))
        c = ((x0 + x1) / 2, y_c, gh / 2)
        sz = (x1 - x0, float(glass.get("thickness", 0.02)), gh)
        prim = _box(stage, f"{WORLD}/corridor/glass_0", c, sz, gm)
        _label(prim, "glass" if "glass" in sc["semantics"] else "wall")
        objects.append({"id": "glass_0", "class": "glass", "center": list(c), "size": list(sz), "yaw": 0.0})
        info.notes.append(f"glass panel on the {glass_side} wall, x {x0:.2f}..{x1:.2f} m, "
                          f"{sz[0]:.2f} x {gh:.2f} m (collider on, opacity "
                          f"{(sc['materials'].get('glass_clear') or {}).get('opacity', '?')})")
    if get(sc, "corridor.ceiling"):
        _box(stage, f"{WORLD}/corridor/ceiling", (L / 2, W / 2, h + 0.05),
             (L + 2 * t, W + 2 * t, 0.1), wall_mat)
    log.info("corridor %.2f x %.2f m, walls %.2f m high, floor '%s', glass %s", L, W, h,
             get(sc, "floor.material"), "on" if glass_on else "off")
    return objects
