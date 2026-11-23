"""Structure-only occupancy -> building layout + real wall openings."""
import sys, os, json, math
import numpy as np
REPO = r"f:/Users/Andreas Steffens/dev_ws/f1tenth_hil"
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(REPO, "sim/isaac/tools"))
import pick_spawn_poses as P
from sdf_to_usd import parse_world, DEFAULT_SDF, DEFAULT_MODELS, DEFAULT_FUEL

STRUCT = ("Surfaces", "Glass_wall_1", "Glass_wall_2", "OpenRobotics/Door", "Stairs")
models, meta = parse_world(DEFAULT_SDF)
struct = [m for m in models if m.name in STRUCT]
furn = [m for m in models if m.name not in STRUCT]
print("structure models:", [m.name for m in struct])
occ_s, o_s, res, floor_s, r_s = P.occupancy(struct, DEFAULT_MODELS, DEFAULT_FUEL, 0.05)
print("struct grid", occ_s.shape, o_s, r_s)
np.savez_compressed(os.path.join(HERE, "grid_struct.npz"), occ=occ_s, floor=floor_s,
                    origin=np.array(o_s), res=res)

# ASCII map of the structure at 0.5 m per char, with world coords
step = int(round(0.5/res))
nx, ny = occ_s.shape
ox, oy = o_s
rows = []
for j in range(ny-1, -1, -step):
    line = []
    for i in range(0, nx, step):
        blk = occ_s[i:i+step, max(0,j-step+1):j+1]
        fl = np.isfinite(floor_s[i:i+step, max(0,j-step+1):j+1]).any()
        line.append('#' if blk.any() else ('.' if fl else ' '))
    y = oy + (j+0.5)*res
    rows.append(f"{y:7.1f} " + "".join(line))
hdr = "        " + "".join(str(int(abs(ox + (i+0.5)*res))%10) for i in range(0, nx, step))
print(hdr)
print("\n".join(rows))
print(hdr)
print(f"x from {ox:.2f} to {ox+nx*res:.2f}; each char 0.5 m")
