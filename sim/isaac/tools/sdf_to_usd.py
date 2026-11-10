#!/usr/bin/env python3
"""Convert the husarion office Gazebo world (SDF) into a USD stage for Isaac Sim.

    sim/isaac/tools/sdf_to_usd.py --plan                      # anywhere (no Isaac, no USD libs)
    sim/isaac/tools/sdf_to_usd.py --convert                   # on the workstation, inside Isaac

WHY THIS SCRIPT EXISTS. Isaac Sim 5.1 ships importers for URDF, MJCF, CAD/Onshape and plain
meshes (``isaacsim.asset.importer.urdf`` / ``.mjcf`` / ``.cad``, ``omni.kit.asset_converter``).
There is NO SDF importer: SDF is Gazebo's world format and NVIDIA ships nothing that reads it
(checked against the installed extension list in the 5.1 image and NVIDIA's importer docs; the
only SDF paths in the Omniverse world are the Gazebo-side connectors, which we do not have).
So the conversion is split the way the available tools allow:

  1. the MESHES (.obj / .stl / .dae) go through ``omni.kit.asset_converter``, one USD each,
     cached by content hash so a re-run converts nothing;
  2. the WORLD STRUCTURE (model tree, poses, scales, static flags, which geometry is visual and
     which is collision) is read from the SDF here and authored with pxr as references to those
     mesh USDs - no geometry is duplicated and no original file is modified.

ONE SOURCE OF TRUTH. The input is the same SDF file the Gazebo sim loads
(``src/f1tenth_sim/worlds/husarion_office_2x.sdf``, the 2x-scaled office made by
``tools/scale_office_world.py`` from the submodule's ``worlds/husarion_office.sdf``), so Gazebo
and Isaac show the same office at the same metric scale, same origin and same axes. Point --sdf at
the 1x original to convert that instead: a ``<state>`` block, which is where the upstream world
hides its wall and floor poses, is baked in here exactly as the scaling tool bakes it.

MESH PATHS. ``model://Pkg/meshes/x.obj`` resolves inside the submodule
(``src/husarion_gz_worlds/models``). The world also references Gazebo Fuel meshes by https URL;
Gazebo downloads those into ``~/.gz/fuel`` and this script reads the same cache (--fuel-cache),
optionally fetching what is missing (--download). The Fuel assets are the one part of the office
that is not in the repository - see output/hil/setup.md.

OUTPUT. ``sim/isaac/usd/generated/office/`` (gitignored): ``office.usd`` plus ``meshes/*.usd``
and ``manifest.json``. It is tens of megabytes of converted geometry, so it is generated on the
workstation, not committed. The thin wrapper ``sim/isaac/usd/office_hil.usda`` (in the repo)
references ``office.usd``, and ``config/scenes/office.yaml`` references the wrapper.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import logging
import math
import os
import struct
import sys
import urllib.request
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

log = logging.getLogger("sdf_to_usd")

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC_DIR = os.path.dirname(HERE)                       # sim/isaac
REPO = os.path.dirname(os.path.dirname(ISAAC_DIR))      # repository root
DEFAULT_SDF = os.path.join(REPO, "src", "f1tenth_sim", "worlds", "husarion_office_2x.sdf")
DEFAULT_MODELS = os.path.join(REPO, "src", "husarion_gz_worlds", "models")
DEFAULT_FUEL = os.path.join(os.path.expanduser("~"), ".gz", "fuel")
DEFAULT_OUT = os.path.join(ISAAC_DIR, "usd", "generated", "office")

# Model-name patterns that make up the building itself: their collision must exist, and --light
# never drops them.
STRUCTURE = ("surfaces", "wall", "floor", "door", "ceiling")


# --------------------------------------------------------------------------- SDF data model
@dataclass
class Geom:
    """One <visual> or <collision> of a link."""

    kind: str                       # 'visual' | 'collision'
    name: str
    pose: np.ndarray                # 4x4, in the link frame
    gtype: str                      # 'mesh' | 'box' | 'plane' | 'cylinder' | 'sphere' | '?'
    uri: str = ""
    scale: tuple = (1.0, 1.0, 1.0)
    size: tuple = ()                # box size / plane size / (radius, length)


@dataclass
class Link:
    name: str
    pose: np.ndarray
    geoms: list = field(default_factory=list)


@dataclass
class Model:
    name: str
    pose: np.ndarray
    static: bool
    links: list = field(default_factory=list)


def pose_matrix(text: str | None) -> np.ndarray:
    """SDF ``<pose>x y z roll pitch yaw</pose>`` (metres, radians, Z-up) -> 4x4."""
    m = np.eye(4)
    if not text:
        return m
    v = [float(x) for x in text.split()]
    v += [0.0] * (6 - len(v))
    cr, sr, cp, sp, cy, sy = (math.cos(v[3]), math.sin(v[3]), math.cos(v[4]), math.sin(v[4]),
                              math.cos(v[5]), math.sin(v[5]))
    m[:3, :3] = np.array([                      # Rz(yaw) Ry(pitch) Rx(roll), the SDF convention
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])
    m[:3, 3] = v[:3]
    return m


def _geoms(link_el: ET.Element) -> list[Geom]:
    out = []
    for kind in ("visual", "collision"):
        for el in link_el.findall(kind):
            g = el.find("geometry")
            if g is None:
                continue
            pose = pose_matrix(el.findtext("pose"))
            name = el.get("name") or kind
            mesh = g.find("mesh")
            if mesh is not None:
                scale = mesh.findtext("scale") or "1 1 1"
                out.append(Geom(kind, name, pose, "mesh", (mesh.findtext("uri") or "").strip(),
                                tuple(float(x) for x in scale.split())))
                continue
            for gtype, reader in (("box", lambda e: tuple(float(x) for x in (e.findtext("size") or "1 1 1").split())),
                                  ("plane", lambda e: tuple(float(x) for x in (e.findtext("size") or "1 1").split())),
                                  ("cylinder", lambda e: (float(e.findtext("radius") or 0.5),
                                                          float(e.findtext("length") or 1.0))),
                                  ("sphere", lambda e: (float(e.findtext("radius") or 0.5),))):
                el2 = g.find(gtype)
                if el2 is not None:
                    out.append(Geom(kind, name, pose, gtype, size=reader(el2)))
                    break
            else:
                out.append(Geom(kind, name, pose, "?"))
    return out


def parse_world(path: str) -> tuple[list[Model], dict]:
    """Read an SDF world. A ``<state>`` block (the upstream office hides its wall/floor poses
    there) is baked into the model poses first, exactly as tools/scale_office_world.py does."""
    root = ET.parse(path).getroot()
    world = root.find("world")
    if world is None:
        raise SystemExit(f"{path}: no <world> element")
    defs = {m.get("name"): m for m in world.findall("model")}
    baked = 0
    for st in world.findall("state"):
        for sm in st.findall("model"):
            dm = defs.get(sm.get("name"))
            if dm is None or sm.findtext("pose") is None:
                continue
            p = dm.find("pose")
            if p is None:
                p = ET.Element("pose")
                dm.insert(0, p)
            p.text = sm.findtext("pose")
            baked += 1
            for sl in sm.findall("link"):                 # link poses inside the state too
                dl = next((x for x in dm.findall("link") if x.get("name") == sl.get("name")), None)
                if dl is not None and sl.findtext("pose") is not None:
                    lp = dl.find("pose")
                    if lp is None:
                        lp = ET.Element("pose")
                        dl.insert(0, lp)
                    lp.text = sl.findtext("pose")
    models = []
    for mel in world.findall("model"):
        m = Model(name=mel.get("name") or "model", pose=pose_matrix(mel.findtext("pose")),
                  static=(mel.findtext("static") or "false").strip().lower() in ("true", "1"))
        for lel in mel.findall("link"):
            m.links.append(Link(lel.get("name") or "link", pose_matrix(lel.findtext("pose")),
                                _geoms(lel)))
        models.append(m)
    meta = {"sdf": path, "world": world.get("name"), "models": len(models), "state_poses_baked": baked,
            "lights": len(world.findall("light"))}
    return models, meta


# --------------------------------------------------------------------------- mesh resolution
FUEL_PREFIXES = ("https://fuel.gazebosim.org/", "https://fuel.ignitionrobotics.org/")


def resolve_uri(uri: str, models_dir: str, fuel_cache: str) -> tuple[str, str]:
    """mesh URI -> (local path, source kind). The path may not exist yet."""
    if uri.startswith("model://"):
        rest = uri[len("model://"):]
        return os.path.join(models_dir, rest), "submodule"
    for pref in FUEL_PREFIXES:
        if uri.startswith(pref):
            host = pref.split("//", 1)[1].rstrip("/")
            # https://host/1.0/<owner>/models/<name>/<ver>/files/<rest>
            parts = uri[len(pref):].split("/")
            if len(parts) >= 6 and parts[0] == "1.0" and parts[2] == "models":
                owner, name, ver = parts[1], parts[3], parts[4]
                rest = "/".join(parts[6:]) if parts[5] == "files" else "/".join(parts[5:])
                # Gazebo's own cache drops the URL's "files" segment
                # (~/.gz/fuel/<host>/<owner>/models/<name>/<ver>/meshes/X.obj); older layouts keep
                # it. Try the cache layout first, fall back to the URL layout.
                base = os.path.join(fuel_cache, host, owner, "models", name, ver)
                flat, nested = os.path.join(base, rest), os.path.join(base, "files", rest)
                return (nested if (os.path.isfile(nested) and not os.path.isfile(flat)) else flat), "fuel"
            return os.path.join(fuel_cache, host, *parts), "fuel"
    if uri.startswith("file://"):
        return uri[len("file://"):], "file"
    if os.path.isabs(uri):
        return uri, "abs"
    return os.path.join(os.path.dirname(models_dir), uri), "relative"


def download(uri: str, dest: str) -> bool:
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    try:
        with urllib.request.urlopen(uri, timeout=60) as r, open(dest, "wb") as f:   # noqa: S310
            f.write(r.read())
        return True
    except Exception as exc:                                        # noqa: BLE001
        log.warning("download failed %s: %s", uri, exc)
        return False


# --------------------------------------------------------------------------- mesh inspection
def _obj_info(path: str) -> dict:
    verts, tris, normals, faces = [], 0, 0, []
    with open(path, errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                verts.append([float(x) for x in line.split()[1:4]])
            elif line.startswith("vn "):
                normals += 1
            elif line.startswith("f "):
                idx = [int(tok.split("/")[0]) for tok in line.split()[1:]]
                tris += max(0, len(idx) - 2)
                if len(idx) >= 3:
                    faces.append(idx[:3])
    return {"vertices": len(verts), "triangles": tris, "has_normals": normals > 0,
            **_bbox_and_volume(np.asarray(verts, dtype=float), faces)}


def _stl_info(path: str) -> dict:
    with open(path, "rb") as f:
        head = f.read(84)
        f.seek(0)
        ascii_stl = head[:5].lower() == b"solid" and b"facet" in f.read(512)
    pts: list = []
    flipped = 0
    n_tri = 0
    if ascii_stl:
        normal = None
        tri: list = []
        with open(path, errors="ignore") as fh:
            for line in fh:
                s = line.split()
                if not s:
                    continue
                if s[0] == "facet" and len(s) >= 6:
                    normal = np.array([float(x) for x in s[2:5]])
                    tri = []
                elif s[0] == "vertex" and len(s) >= 4:
                    tri.append([float(x) for x in s[1:4]])
                elif s[0] == "endfacet":
                    n_tri += 1
                    if len(tri) == 3:
                        pts.extend(tri)
                        flipped += _flipped(np.asarray(tri), normal)
    else:
        with open(path, "rb") as fh:
            fh.seek(80)
            n_tri = struct.unpack("<I", fh.read(4))[0]
            for _ in range(n_tri):
                raw = fh.read(50)
                if len(raw) < 50:
                    break
                vals = struct.unpack("<12fH", raw)
                normal = np.array(vals[0:3])
                tri = np.array(vals[3:12]).reshape(3, 3)
                pts.extend(tri.tolist())
                flipped += _flipped(tri, normal)
    arr = np.asarray(pts, dtype=float)
    faces = [[i * 3 + 1, i * 3 + 2, i * 3 + 3] for i in range(len(arr) // 3)]
    return {"vertices": len(arr), "triangles": n_tri, "has_normals": True,
            "stl_normal_disagrees": flipped, **_bbox_and_volume(arr, faces)}


def _flipped(tri: np.ndarray, normal: np.ndarray | None) -> int:
    if normal is None or not np.any(normal):
        return 0
    c = np.cross(tri[1] - tri[0], tri[2] - tri[0])
    if not np.any(c):
        return 0
    return int(float(np.dot(c / np.linalg.norm(c), normal / np.linalg.norm(normal))) < 0.0)


def _dae_info(path: str) -> dict:
    root = ET.parse(path).getroot()
    ns = {"c": root.tag.split("}")[0].strip("{")} if root.tag.startswith("{") else {}

    def find(tag):
        return root.iter(f"{{{ns['c']}}}{tag}" if ns else tag)
    unit = next((el.get("meter") for el in find("unit")), None)
    up = next((el.text for el in find("up_axis")), None)
    pts: list = []
    for src in find("source"):
        if "position" in (src.get("id") or "").lower() or "vert" in (src.get("id") or "").lower():
            for fa in src.iter():
                if fa.tag.endswith("float_array") and fa.text:
                    vals = [float(x) for x in fa.text.split()]
                    pts.extend(np.asarray(vals[: len(vals) - len(vals) % 3]).reshape(-1, 3).tolist())
    arr = np.asarray(pts, dtype=float)
    out = {"vertices": len(arr), "triangles": None, "has_normals": True,
           "collada_unit_meter": float(unit) if unit else 1.0, "collada_up_axis": (up or "Y_UP").strip()}
    out.update(_bbox_and_volume(arr, []))
    return out


def _bbox_and_volume(verts: np.ndarray, faces: list) -> dict:
    if verts.size == 0:
        return {"bbox_min": None, "bbox_max": None, "signed_volume": None}
    out = {"bbox_min": verts.min(axis=0).tolist(), "bbox_max": verts.max(axis=0).tolist(),
           "signed_volume": None}
    if faces:
        v = verts
        vol = 0.0
        for a, b, c in faces:
            try:
                p, q, r = v[a - 1], v[b - 1], v[c - 1]
            except IndexError:
                return out
            vol += float(np.dot(p, np.cross(q, r))) / 6.0
        out["signed_volume"] = vol
    return out


def mesh_triangles(path: str) -> np.ndarray | None:
    """Triangles of a mesh as an (N, 3, 3) array, or None for a format we do not read.

    The per-TRIANGLE geometry matters for anything that asks "is this spot free": the office's
    walls and floors are each ONE mesh spanning the whole building, so a per-mesh bounding box
    marks the entire floor plan as occupied (measured: 2.8 m2 of the office left free).
    """
    ext = os.path.splitext(path)[1].lower()
    try:
        if ext == ".obj":
            verts: list = []
            faces: list = []
            with open(path, errors="ignore") as f:
                for line in f:
                    if line.startswith("v "):
                        verts.append([float(x) for x in line.split()[1:4]])
                    elif line.startswith("f "):
                        idx = [int(tok.split("/")[0]) for tok in line.split()[1:]]
                        idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
                        for k in range(1, len(idx) - 1):
                            faces.append([idx[0], idx[k], idx[k + 1]])
            if not faces:
                return None
            v = np.asarray(verts, dtype=float)
            f_arr = np.asarray(faces, dtype=int)
            if f_arr.max() >= len(v):
                return None
            return v[f_arr]
        if ext == ".stl":
            info = _stl_triangles(path)
            return info
    except Exception as exc:                                        # noqa: BLE001
        log.warning("mesh_triangles(%s): %s", path, exc)
    return None


def _stl_triangles(path: str) -> np.ndarray | None:
    with open(path, "rb") as f:
        head = f.read(512)
        ascii_stl = head[:5].lower() == b"solid" and b"facet" in head
    tris: list = []
    if ascii_stl:
        cur: list = []
        with open(path, errors="ignore") as fh:
            for line in fh:
                s = line.split()
                if not s:
                    continue
                if s[0] == "vertex" and len(s) >= 4:
                    cur.append([float(x) for x in s[1:4]])
                elif s[0] == "endfacet":
                    if len(cur) == 3:
                        tris.append(cur)
                    cur = []
    else:
        with open(path, "rb") as fh:
            fh.seek(80)
            n = struct.unpack("<I", fh.read(4))[0]
            for _ in range(n):
                raw = fh.read(50)
                if len(raw) < 50:
                    break
                vals = struct.unpack("<12fH", raw)
                tris.append(np.asarray(vals[3:12]).reshape(3, 3).tolist())
    return np.asarray(tris, dtype=float) if tris else None


def mesh_info(path: str) -> dict:
    ext = os.path.splitext(path)[1].lower()
    try:
        if ext == ".obj":
            return _obj_info(path)
        if ext == ".stl":
            return _stl_info(path)
        if ext == ".dae":
            return _dae_info(path)
    except Exception as exc:                                        # noqa: BLE001
        return {"error": f"{type(exc).__name__}: {exc}"}
    return {"error": f"unsupported extension {ext}"}


# --------------------------------------------------------------------------- plan / verify
def world_bbox(models: list[Model], meshes: dict, only_structure: bool = True) -> dict:
    """World-space bounding box from the SDF poses and the mesh vertex bounds."""
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    counted = 0
    for m in models:
        if only_structure and not any(s in m.name.lower() for s in STRUCTURE):
            continue
        for link in m.links:
            for g in link.geoms:
                if g.gtype != "mesh":
                    continue
                info = meshes.get(g.uri, {})
                if not info.get("bbox_min"):
                    continue
                mn, mx = np.array(info["bbox_min"]), np.array(info["bbox_max"])
                s = np.array(g.scale, dtype=float)
                corners = np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1])
                                    for z in (mn[2], mx[2])]) * s
                tf = m.pose @ link.pose @ g.pose
                w = (tf[:3, :3] @ corners.T).T + tf[:3, 3]
                lo = np.minimum(lo, w.min(axis=0))
                hi = np.maximum(hi, w.max(axis=0))
                counted += 1
    if counted == 0:
        return {"geoms": 0}
    return {"geoms": counted, "min": lo.tolist(), "max": hi.tolist(), "size": (hi - lo).tolist()}


def plan(args) -> dict:
    models, meta = parse_world(args.sdf)
    uris: dict[str, dict] = {}
    missing, downloaded = [], []
    n_vis = n_col = 0
    for m in models:
        for link in m.links:
            for g in link.geoms:
                n_vis += g.kind == "visual"
                n_col += g.kind == "collision"
                if g.gtype != "mesh" or not g.uri:
                    continue
                if g.uri in uris:
                    continue
                path, kind = resolve_uri(g.uri, args.models_dir, args.fuel_cache)
                if not os.path.isfile(path) and args.download and g.uri.startswith(FUEL_PREFIXES):
                    if download(g.uri, path):
                        downloaded.append(g.uri)
                entry = {"path": path, "source": kind, "exists": os.path.isfile(path)}
                if entry["exists"]:
                    entry.update(mesh_info(path))
                else:
                    missing.append(g.uri)
                uris[g.uri] = entry
    # structure models must have collision, or the car falls through / drives through walls
    structure = [m for m in models if any(s in m.name.lower() for s in STRUCTURE)]
    no_collision = [m.name for m in structure
                    if not any(g.kind == "collision" for link in m.links for g in link.geoms)]
    tri = sum(int(e.get("triangles") or 0) for e in uris.values())
    # Metric check. Several meshes are authored in MILLIMETRES (Tv, Kitchen, Laptop) and the SDF
    # compensates with <scale>0.001</scale>; what must be plausible is therefore the mesh extent
    # TIMES the scale of each instance that uses it. The native units are reported separately
    # because they are a conversion trap: the mesh USD keeps the raw numbers and the scale has to
    # be authored on the referencing prim (which convert() does), not guessed by the converter.
    scaled, native_mm = {}, {}
    for m in models:
        for link in m.links:
            for g in link.geoms:
                e = uris.get(g.uri) if g.gtype == "mesh" else None
                if not e or not e.get("bbox_max"):
                    continue
                ext = (np.array(e["bbox_max"]) - np.array(e["bbox_min"])) * np.array(g.scale, dtype=float)
                if float(np.max(np.abs(ext))) > 60.0:          # nothing in an office is 60 m across
                    scaled[f"{m.name}/{link.name}/{g.name}"] = [round(float(x), 3) for x in ext]
                if float(np.max(np.abs(np.array(e["bbox_max"]) - np.array(e["bbox_min"])))) > 60.0:
                    native_mm[g.uri] = {"native_extent": [round(float(x), 1) for x in
                                                          (np.array(e["bbox_max"]) - np.array(e["bbox_min"]))],
                                        "sdf_scale": list(g.scale)}
    suspect_scale = scaled
    non_z_up = {u: e.get("collada_up_axis") for u, e in uris.items()
                if e.get("collada_up_axis") and e["collada_up_axis"] != "Z_UP"}
    flipped = {u: e["stl_normal_disagrees"] for u, e in uris.items() if e.get("stl_normal_disagrees")}
    report = {
        "meta": meta,
        "geoms": {"visual": n_vis, "collision": n_col},
        "meshes": {"unique": len(uris), "missing": len(missing), "triangles": tri,
                   "by_source": _count([e["source"] for e in uris.values()])},
        "missing_uris": missing[:40],
        "downloaded": downloaded,
        "structure_models": [m.name for m in structure],
        "structure_without_collision": no_collision,
        "world_bbox_structure_m": world_bbox(models, uris, only_structure=True),
        "world_bbox_all_m": world_bbox(models, uris, only_structure=False),
        "checks": {
            "scaled_geometry_in_metres": not suspect_scale,
            "suspect_scaled_extent_m": suspect_scale,
            "meshes_not_authored_in_metres": native_mm,
            "collada_not_z_up": non_z_up,
            "stl_facets_disagreeing_with_winding": flipped,
        },
    }
    return report


def _count(items) -> dict:
    out: dict = {}
    for i in items:
        out[i] = out.get(i, 0) + 1
    return out


# --------------------------------------------------------------------------- convert (Isaac only)
def convert(args, report: dict) -> dict:
    """Mesh -> USD with omni.kit.asset_converter, then author the world stage with pxr.

    Runs only inside Isaac's python (``/isaac-sim/python.sh``): both imports fail elsewhere.
    """
    import asyncio

    import omni.kit.asset_converter as converter
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    models, _meta = parse_world(args.sdf)
    mesh_dir = os.path.join(args.out, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)

    async def _convert_one(src: str, dst: str) -> bool:
        task = converter.get_instance().create_converter_task(src, dst, None)
        ok = await task.wait_until_finished()
        if not ok:
            log.error("asset_converter failed for %s: %s", src, task.get_status_message())
        return bool(ok)

    cache: dict[str, str] = {}
    for uri, entry in report["meshes_detail"].items():
        if not entry["exists"]:
            continue
        key = hashlib.sha1(entry["path"].encode()).hexdigest()[:16]        # noqa: S324 - cache key
        dst = os.path.join(mesh_dir, f"{key}.usd")
        if not os.path.isfile(dst):
            if not asyncio.get_event_loop().run_until_complete(_convert_one(entry["path"], dst)):
                continue
        cache[uri] = dst

    stage = Usd.Stage.CreateNew(os.path.join(args.out, "office.usd")) \
        if not os.path.isfile(os.path.join(args.out, "office.usd")) \
        else Usd.Stage.Open(os.path.join(args.out, "office.usd"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    drop_visual = {x.strip().lower() for x in (args.drop_visual or "").split(",") if x.strip()}
    n_geom = n_coll = 0
    for m in models:
        is_structure = any(s in m.name.lower() for s in STRUCTURE)
        mp = f"/World/{_safe(m.name)}"
        mx = UsdGeom.Xform.Define(stage, mp)
        _set_matrix(UsdGeom.Xformable(mx), m.pose, Gf)
        for link in m.links:
            lp = f"{mp}/{_safe(link.name)}"
            lx = UsdGeom.Xform.Define(stage, lp)
            _set_matrix(UsdGeom.Xformable(lx), link.pose, Gf)
            for g in link.geoms:
                if g.gtype != "mesh" or g.uri not in cache:
                    continue
                if g.kind == "visual" and not is_structure and (
                        args.light or any(d in m.name.lower() for d in drop_visual)):
                    continue                       # --light: furniture keeps collision, loses pixels
                leaf = g.kind if g.name in (g.kind, "") else f"{g.kind}_{g.name}"
                gp = f"{lp}/{_safe(leaf)}"
                prim = stage.OverridePrim(gp) if stage.GetPrimAtPath(gp) else \
                    UsdGeom.Xform.Define(stage, gp).GetPrim()
                prim.GetReferences().AddReference(Sdf.Reference(os.path.relpath(cache[g.uri], args.out)))
                tf = g.pose.copy()
                xf = UsdGeom.Xformable(prim)
                _set_matrix(xf, tf, Gf)
                xf.AddScaleOp().Set(Gf.Vec3f(*[float(s) for s in g.scale]))
                n_geom += 1
                if g.kind == "collision":
                    UsdPhysics.CollisionAPI.Apply(prim)
                    approx = "none" if m.static or is_structure else "convexHull"
                    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approx)
                    UsdGeom.Imageable(prim).MakeInvisible()
                    n_coll += 1
    stage.GetRootLayer().Save()
    # verify: the stage's own bounding box against the one computed from the SDF
    cache_bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache_bb.ComputeWorldBound(stage.GetPrimAtPath("/World")).ComputeAlignedRange()
    out = {"converted_meshes": len(cache), "geoms_authored": n_geom, "colliders": n_coll,
           "stage": os.path.join(args.out, "office.usd"),
           "stage_bbox_m": None if rng.IsEmpty() else
           {"min": list(rng.GetMin()), "max": list(rng.GetMax()),
            "size": list(rng.GetMax() - rng.GetMin())}}
    return out


def _safe(name: str) -> str:
    out = "".join(c if c.isalnum() else "_" for c in name)
    return out if out[:1].isalpha() or out[:1] == "_" else f"_{out}"


def _set_matrix(xformable, m: np.ndarray, Gf) -> None:   # noqa: N803 - Gf passed in to keep pxr out of the module scope
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(Gf.Matrix4d(m.T.tolist()))


# --------------------------------------------------------------------------- main
def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sdf", default=DEFAULT_SDF, help="input world (default: the Gazebo sim's own 2x office)")
    p.add_argument("--models-dir", default=DEFAULT_MODELS, help="model:// root (husarion_gz_worlds/models)")
    p.add_argument("--fuel-cache", default=DEFAULT_FUEL, help="Gazebo Fuel cache (default ~/.gz/fuel)")
    p.add_argument("--out", default=DEFAULT_OUT, help="output directory for the generated USD")
    p.add_argument("--plan", action="store_true", help="parse, resolve and check only (no USD, no Isaac)")
    p.add_argument("--convert", action="store_true", help="convert the meshes and author the stage (Isaac only)")
    p.add_argument("--download", action="store_true", help="fetch missing Fuel meshes over https")
    p.add_argument("--light", action="store_true",
                   help="drop VISUAL meshes of non-structural models (furniture); collision is kept")
    p.add_argument("--drop-visual", default="", help="comma-separated model-name substrings to strip visuals from")
    p.add_argument("--report", default="", help="write the JSON report here")
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    if not args.plan and not args.convert:
        args.plan = True
    if not os.path.isfile(args.sdf):
        log.error("no such SDF: %s", args.sdf)
        return 2
    report = plan(args)
    detail = {}
    models, _ = parse_world(args.sdf)
    for m in models:                       # keep the per-URI detail out of the printed summary
        for link in m.links:
            for g in link.geoms:
                if g.gtype == "mesh" and g.uri and g.uri not in detail:
                    path, kind = resolve_uri(g.uri, args.models_dir, args.fuel_cache)
                    detail[g.uri] = {"path": path, "source": kind, "exists": os.path.isfile(path)}
    report["meshes_detail"] = detail
    if args.convert:
        os.makedirs(args.out, exist_ok=True)
        report["convert"] = convert(args, report)
        bb_sdf = report["world_bbox_structure_m"].get("size")
        bb_usd = (report["convert"].get("stage_bbox_m") or {}).get("size")
        if bb_sdf and bb_usd:
            d = max(abs(a - b) for a, b in zip(bb_sdf, bb_usd[:3]))
            report["convert"]["bbox_agreement_m"] = d
            log.info("bounding box: SDF %s vs stage %s (max difference %.3f m)",
                     [round(x, 2) for x in bb_sdf], [round(x, 2) for x in bb_usd], d)
    printable = {k: v for k, v in report.items() if k != "meshes_detail"}
    print(json.dumps(printable, indent=1, default=str))
    if args.report:
        with open(args.report, "w") as f:
            json.dump(report, f, indent=1, default=str)
        log.info("report written to %s", args.report)
    bad = report["structure_without_collision"] or report["meshes"]["missing"]
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
