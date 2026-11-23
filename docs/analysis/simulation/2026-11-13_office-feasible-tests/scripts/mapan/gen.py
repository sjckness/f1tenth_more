import json, os
HERE = os.path.dirname(os.path.abspath(__file__))
geo = json.load(open(os.path.join(HERE, "map_geo.json")))
mis = json.load(open(os.path.join(HERE, "missions.json")))
head = open(os.path.join(HERE, "page_head.html"), encoding="utf-8").read()
tail = open(os.path.join(HERE, "page_tail.html"), encoding="utf-8").read()
out = head.replace("/*__GEO__*/", json.dumps(geo, separators=(",", ":"))) \
          .replace("/*__MIS__*/", json.dumps(mis, ensure_ascii=False, separators=(",", ":")))
open(os.path.join(HERE, "office_hil.html"), "w", encoding="utf-8").write(out + tail)
print("wrote", os.path.getsize(os.path.join(HERE, "office_hil.html")), "bytes")
