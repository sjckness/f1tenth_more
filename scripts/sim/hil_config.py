#!/usr/bin/env python3
"""Read sim/isaac/config/hil.yaml and print it as shell variables (or JSON).

    eval "$(scripts/sim/hil_config.py --sh)"     # HIL_LINK_ORIN_IP=..., HIL_ROS_DOMAIN_ID=...
    scripts/sim/hil_config.py --json
    scripts/sim/hil_config.py --get ros.domain_id

One config file for three scripts on two machines: sim_isaac_hil (workstation), hil_orin (Orin)
and hil_check (inside the Orin container) all read it through here, so a value exists in exactly
one place. config/hil.local.yaml wins over config/hil.yaml when it exists, which is how a machine
keeps its own addresses out of git.

Keys become HIL_<SECTION>_<KEY> in upper case, nested keys joined with '_'; lists are joined with
spaces. Values are shell-quoted, so `eval` is safe for the values this file can hold.
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CONFIG_DIR = os.path.join(REPO, "sim", "isaac", "config")
REQUIRED = ("link.workstation_ip", "link.orin_ip", "ros.domain_id", "ros.rmw",
            "ros.discovery_server_host", "ros.discovery_server_port", "sim.isaac_dir", "sim.image",
            "sim.container", "sim.world", "orin.ssh", "orin.container")


def config_path() -> str:
    local = os.path.join(CONFIG_DIR, "hil.local.yaml")
    return local if os.path.isfile(local) else os.path.join(CONFIG_DIR, "hil.yaml")


def load(path: str | None = None) -> dict:
    import yaml

    path = path or config_path()
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    missing = [k for k in REQUIRED if _get(data, k) is None]
    if missing:
        raise SystemExit(f"{path}: missing required keys: {', '.join(missing)}")
    host = str(_get(data, "ros.discovery_server_host"))
    if host not in ("orin", "workstation"):
        raise SystemExit(f"{path}: ros.discovery_server_host must be 'orin' or 'workstation', "
                         f"not {host!r}")
    if str(_get(data, "ros.rmw")) != "rmw_fastrtps_cpp":
        raise SystemExit(f"{path}: ros.rmw must be rmw_fastrtps_cpp (the Isaac 5.1 bridge ships "
                         "Fast DDS and Cyclone only; output/zenoh/recon.md section 3.3)")
    return data


def _get(data: dict, dotted: str):
    cur = data
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


def flatten(data: dict, prefix: str = "HIL") -> dict:
    out: dict[str, str] = {}
    for key, value in data.items():
        name = f"{prefix}_{str(key).upper()}"
        if isinstance(value, dict):
            out.update(flatten(value, name))
        elif isinstance(value, (list, tuple)):
            out[name] = " ".join(str(v) for v in value)
        elif isinstance(value, bool):
            out[name] = "true" if value else "false"
        elif value is None:
            out[name] = ""
        else:
            out[name] = str(value)
    return out


def derived(data: dict) -> dict:
    """Values every script would otherwise compute itself."""
    ds_host = str(_get(data, "ros.discovery_server_host"))
    ds_ip = str(_get(data, "link.orin_ip") if ds_host == "orin" else _get(data, "link.workstation_ip"))
    port = str(_get(data, "ros.discovery_server_port"))
    st = _get(data, "sim.stereo") or {}
    w, h = int(st.get("width", 640)), int(st.get("height", 360))
    rate = float(st.get("rate", 15.0))
    readers = 5          # 2 for /camera/image_raw, 3 for depth_registered (one unicast copy each)
    mbit = w * h * 4 * rate * readers * 8 / 1e6
    return {
        "HIL_DISCOVERY_SERVER_IP": ds_ip,
        "HIL_DISCOVERY_SERVER": f"{ds_ip}:{port}",
        "HIL_CONFIG_FILE": config_path(),
        "HIL_REPO": REPO,
        "HIL_CAMERA_MBIT": f"{mbit:.0f}",
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sh", action="store_true", help="print shell assignments (default)")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--get", default="", metavar="DOTTED.KEY")
    ap.add_argument("--file", default="", help="read this config file instead")
    args = ap.parse_args()
    data = load(args.file or None)
    if args.get:
        value = _get(data, args.get)
        if value is None:
            raise SystemExit(f"no such key: {args.get}")
        print(value)
        return 0
    if args.json:
        print(json.dumps({**flatten(data), **derived(data)}, indent=1))
        return 0
    for key, value in {**flatten(data), **derived(data)}.items():
        print(f"{key}={shlex.quote(value)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
