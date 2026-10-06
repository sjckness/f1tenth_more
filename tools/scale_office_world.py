#!/usr/bin/env python3
"""Make a scaled copy of husarion_office.sdf (walls/floors bigger, furniture spread out).

Usage (from the husarion_gz_worlds package root):
    python3 scale_office_world.py            # 2x  -> worlds/husarion_office_2x.sdf
    python3 scale_office_world.py 1.5        # 1.5x -> worlds/husarion_office_1.5x.sdf

What it does:
- Wall + floor meshes (model 'Surfaces') get <scale>S S 1</scale>: footprint scales, height stays.
- Every top-level model position is scaled in x,y (z and orientation untouched),
  so furniture keeps its own size but moves with the bigger rooms.
- Items sitting on desks (laptops, Rosbots) keep their original offset from the
  desk they stand on, so they don't fall off.
- The saved <state> block is dropped (it would override the new poses on load).
"""
import sys
import xml.etree.ElementTree as ET

S = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
SRC = "worlds/husarion_office.sdf"
DST = f"worlds/husarion_office_{S:g}x.sdf"
SUPPORT_KEYS = ("AdjTable", "ConferenceTable", "Table")  # things objects can sit on
ON_DESK_Z = (0.6, 1.0)  # z range of objects resting on desks (laptops, rosbots)


def get_pose(el):
    p = el.find("pose")
    return (p, [float(v) for v in p.text.split()]) if p is not None else (None, None)


def set_pose(p_el, v):
    p_el.text = " ".join(f"{x:.6g}" for x in v)


tree = ET.parse(SRC)
world = tree.getroot().find("world")

# Bake the saved <state> into the model definitions, then drop it.
# (The original file places the wall/floor links only via <state>.)
defs = {m.get("name"): m for m in world.findall("model")}
for st in world.findall("state"):
    for sm in st.findall("model"):
        dm = defs.get(sm.get("name"))
        if dm is None:
            continue
        _, mp = get_pose(sm)
        p_el = dm.find("pose")
        if p_el is None:
            p_el = ET.Element("pose")
            dm.insert(0, p_el)
        set_pose(p_el, mp)
        if sm.get("name") == "Surfaces":  # static links: store pose relative to model
            dlinks = {l.get("name"): l for l in dm.findall("link")}
            for sl in sm.findall("link"):
                dl = dlinks.get(sl.get("name"))
                _, lp = get_pose(sl)
                if dl is None or lp is None:
                    continue
                rel = [lp[0] - mp[0], lp[1] - mp[1], lp[2] - mp[2]] + lp[3:]
                lp_el = dl.find("pose")
                if lp_el is None:
                    lp_el = ET.Element("pose")
                    dl.insert(0, lp_el)
                set_pose(lp_el, rel)
    world.remove(st)

models = [m for m in world.findall("model") if m.find("pose") is not None]
orig = {m.get("name"): get_pose(m)[1][:] for m in models}
supports = {n: p for n, p in orig.items()
            if any(k in n for k in SUPPORT_KEYS) and p[2] < 0.5}


def nearest_support(p):
    best, bd = None, 1e9
    for n, sp in supports.items():
        d = (sp[0] - p[0]) ** 2 + (sp[1] - p[1]) ** 2
        if d < bd:
            best, bd = n, d
    return best if bd ** 0.5 < 1.2 else None


for m in models:
    name = m.get("name")
    p_el, p = get_pose(m)
    sup = nearest_support(p) if ON_DESK_Z[0] < p[2] < ON_DESK_Z[1] else None
    if sup:  # keep relative offset to the desk it stands on
        sp = orig[sup]
        p[0] = sp[0] * S + (p[0] - sp[0])
        p[1] = sp[1] * S + (p[1] - sp[1])
    else:
        p[0] *= S
        p[1] *= S
    set_pose(p_el, p)

    if name == "Surfaces":  # walls + floors: scale meshes and link offsets
        for link in m.findall("link"):
            lp_el, lp = get_pose(link)
            if lp_el is not None:
                lp[0] *= S
                lp[1] *= S
                set_pose(lp_el, lp)
            for mesh in link.iter("mesh"):
                sc = mesh.find("scale")
                if sc is None:
                    sc = ET.SubElement(mesh, "scale")
                sc.text = f"{S:g} {S:g} 1"

tree.write(DST, xml_declaration=True, encoding="utf-8")
print(f"wrote {DST} (scale {S:g}x)")
