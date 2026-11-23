"""Joints of a skel animation clip vs a character skeleton.  /isaac-sim/python.sh tools/anim_probe.py <anim> <character>"""
import functools
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Usd, UsdSkel  # noqa: E402

print = functools.partial(print, flush=True)  # noqa: A001
root = get_assets_root_path()
an = Usd.Stage.Open(f"{root}{sys.argv[1]}")
ch = Usd.Stage.Open(f"{root}{sys.argv[2]}")
aj = []
for p in an.Traverse():
    if p.IsA(UsdSkel.Animation):
        a = UsdSkel.Animation(p)
        aj = list(a.GetJointsAttr().Get())
        print("ANIM", p.GetPath(), len(aj), "time", an.GetStartTimeCode(), an.GetEndTimeCode(), an.GetTimeCodesPerSecond())
        print("  first joints", aj[:5])
cj = []
for p in ch.Traverse():
    if p.IsA(UsdSkel.Skeleton):
        cj = list(UsdSkel.Skeleton(p).GetJointsAttr().Get())
print("character joints", len(cj), "anim joints", len(aj), "common", len(set(cj) & set(aj)))
short = lambda js: {j.split("/")[-1] for j in js}  # noqa: E731
print("common by leaf name", len(short(cj) & short(aj)))
print("anim-only leafs", sorted(short(aj) - short(cj))[:20])
app.close()
