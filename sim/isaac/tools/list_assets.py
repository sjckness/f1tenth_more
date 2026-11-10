"""List asset-server folders (helper for choosing scene assets)."""
import sys
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.client
from isaacsim.storage.native import get_assets_root_path
root = get_assets_root_path()
for sub in sys.argv[1:]:
    res, entries = omni.client.list(f"{root}{sub}")
    print("LIST", sub, res, flush=True)
    for e in entries:
        print("  ", e.relative_path, e.size, flush=True)
app.close()
