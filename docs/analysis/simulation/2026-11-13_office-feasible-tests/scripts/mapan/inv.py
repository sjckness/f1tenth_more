import os, json, math, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from prim import *

objs = json.load(open(os.path.join(HERE, "objects.json")))
CLS = [("chair", ("ConfOfficeChairBlack", "DeskChair", "OfficeChairBlack", "WoodenChair")),
       ("dining table", ("AdjTable", "ConferenceTable", "CoffeeTable", "Table")),
       ("laptop", ("Laptop",)), ("tv", ("Tv",)), ("refrigerator", ("Fridge",)),
       ("sink", ("BathroomSink",)), ("toilet", ("Toilet",)),
       ("cabinet*", ("WhiteCabinet", "SquaredShelf", "Shelf", "Drawer")),
       ("bin*", ("TrashBin",)), ("cart*", ("MopCart", "WaterDispenser", "Kitchen")),
       ("robot*", ("_Rosbot",))]
def cls_of(name):
    base = name.split("/")[-1]
    for c, pats in CLS:
        for p in pats:
            if base.startswith(p):
                return c
    return "other"
ROOMNAME = {1:"corridoio SW stretto",2:"sala SW",3:"ufficio SE",4:"corridoio centrale N",
            5:"bagno NE-centro",6:"cucina/sala NE",7:"stanza NW",8:"sala riunioni W",
            9:"passaggio centro-W",10:"stanza W grande",11:"open space centrale (grande)",
            12:"ufficio E-centro",13:"ripostiglio N",14:"stanza N-centro"}
by_room = {}
for o in objs:
    o["class"] = cls_of(o["name"])
    i, j = gij(np.array([o["c"][0]]), np.array([o["c"][1]]))
    o["room"] = int(ROOM[i[0], j[0]])
    by_room.setdefault(o["room"], []).append(o)
print("== OBJECTS PER ROOM ==")
for r in sorted(by_room):
    cs = {}
    for o in by_room[r]:
        cs.setdefault(o["class"], []).append(o["name"].split("/")[-1])
    print(f"\nroom {r:2d} {ROOMNAME.get(r,'?'):26s}")
    for c in sorted(cs):
        print(f"   {c:14s} {len(cs[c]):2d}  {', '.join(sorted(cs[c]))[:110]}")
print("\n== CHAIRS ==")
chairs = [o for o in objs if o["class"] == "chair"]
for o in sorted(chairs, key=lambda d: (d["room"], d["c"][0])):
    i, j = gij(np.array([o["c"][0]]), np.array([o["c"][1]]))
    print(f"{o['name'].split('/')[-1]:24s} room {o['room']:2d}  c=({o['c'][0]:6.2f},{o['c'][1]:7.2f})  "
          f"h={o['hi'][2]:.2f}  clear_around={CL_O[i[0],j[0]]:.2f}")
json.dump(objs, open(os.path.join(HERE, "objects2.json"), "w"), indent=1)
