"""Print the skeleton joints (rest/bind transforms) of a character USD and list animation clips.
    /isaac-sim/python.sh tools/skel_probe.py <asset> [anim folder]
"""
import functools
import sys

print = functools.partial(print, flush=True)  # noqa: A001 - fast shutdown drops buffered stdout

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.client  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Usd, UsdSkel  # noqa: E402

root = get_assets_root_path()
st = Usd.Stage.Open(f"{root}{sys.argv[1]}")
for p in st.Traverse():
    if p.IsA(UsdSkel.Skeleton):
        sk = UsdSkel.Skeleton(p)
        joints = sk.GetJointsAttr().Get()
        print("SKELETON", p.GetPath(), len(joints))
        for j in joints:
            print("  ", j)
    if p.IsA(UsdSkel.Animation):
        print("ANIM", p.GetPath())
    b = UsdSkel.BindingAPI(p)
    if p.HasAPI(UsdSkel.BindingAPI) and b.GetSkeletonRel().GetTargets():
        print("BOUND", p.GetPath(), p.GetTypeName(), b.GetSkeletonRel().GetTargets(), b.GetAnimationSourceRel().GetTargets())
for sub in sys.argv[2:]:
    res, entries = omni.client.list(f"{root}{sub}")
    print("LIST", sub, res)
    for e in entries:
        print("  ", e.relative_path, e.size)
app.close()
