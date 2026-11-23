import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from prim import *
def runs(mask, cell=0.10):
    st = int(round(cell/RES)); out = []
    for j in range(0, NY, st):
        row = mask[:, j:j+st].any(axis=1)
        i = 0
        while i < NX:
            if row[i]:
                k = i
                while k+st < NX and row[k:k+st].any():
                    k += st
                # extend while any cell in the block is set
                k2 = i
                while k2 < NX and row[k2:min(NX, k2+st)].any():
                    k2 += st
                out.append([round(OX+i*RES, 2), round(OY+j*RES, 2),
                            round((k2-i)*RES, 2), round(st*RES, 2)])
                i = k2
            else:
                i += st
    return out
objs = json.load(open(os.path.join(HERE, "objects2.json")))
ops = json.load(open(os.path.join(HERE, "openings2.json")))
data = {
  "bounds": [round(float(OX),2), round(float(OY),2), round(float(OX+NX*RES),2), round(float(OY+NY*RES),2)],
  "floor": runs(INSIDE & ~WALL, 0.20),
  "wall": runs(WALL, 0.10),
  "objects": [{"n": o["name"].split("/")[-1], "c": o["class"], "lo": [round(o["lo"][0],2), round(o["lo"][1],2)],
               "hi": [round(o["hi"][0],2), round(o["hi"][1],2)], "h": round(o["hi"][2],2),
               "r": o["room"]} for o in objs],
  "openings": ops,
  "lights": [[16.8,0.0],[16.8,-4.0],[16.8,-8.0],[16.8,-12.0],[15.4,-11.6],[10.0,-19.4],[8.0,-19.4]],
  "rooms": {"1":"corridoio SW","2":"sala SW","3":"ufficio SE","4":"corridoio centrale N","5":"bagno NE",
            "6":"cucina / sala NE","7":"stanza NW","8":"sala riunioni W","9":"passaggio W","10":"stanza W",
            "11":"open space centrale","12":"ufficio E","13":"ripostiglio N","14":"stanza N-centro"},
  "room_centers": {}
}
for rid in range(1, 15):
    idx = np.argwhere(ROOM == rid)
    if len(idx):
        data["room_centers"][str(rid)] = [round(float(OX+idx[:,0].mean()*RES),2), round(float(OY+idx[:,1].mean()*RES),2)]
json.dump(data, open(os.path.join(HERE, "map_geo.json"), "w"), separators=(",", ":"))
print("floor runs", len(data["floor"]), "wall runs", len(data["wall"]),
      "bytes", os.path.getsize(os.path.join(HERE, "map_geo.json")))
