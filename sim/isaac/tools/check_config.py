#!/usr/bin/env python3
"""Static check of the sim configuration: load every scene the way main.py does.

    sim/isaac/tools/check_config.py [--scene NAME ...] [--layouts]

Needs only PyYAML and numpy - no Isaac, no USD - so it runs on any machine and in CI. It catches
what a scene file can get wrong before a 20 GB container is started: a missing required key, a
layout preset that merges into nothing, a `sensor_overrides` entry whose target key does not
exist, an unexpanded ${VAR}, a USD reference that is neither /Isaac/... nor a file in the repo.
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.config import CONFIG_DIR, ConfigError, get, load_config  # noqa: E402


def _references(usd_path: str) -> list[str]:
    """Asset paths a text USD layer references (@...@). Binary .usd files are skipped."""
    try:
        with open(usd_path, "rb") as f:
            head = f.read(16)
            if not head.startswith(b"#usda"):
                return []
        with open(usd_path, errors="ignore") as f:
            text = f.read()
    except OSError:
        return []
    return re.findall(r"references\s*=\s*@([^@]+)@", text)


def check_scene(name: str, layout: str | None, strict: bool = False) -> list[str]:
    """Load one scene (and layout). Returns the errors found; warnings are printed."""
    errors: list[str] = []
    warnings: list[str] = []
    overrides = [f"scene.layout={layout}"] if layout else []
    try:
        cfg = load_config(name, overrides)
    except ConfigError as exc:
        return [f"{name}{'/' + layout if layout else ''}: {exc}"]
    sc = cfg.scene
    tag = f"{name}{'/' + layout if layout else ''}"
    if sc["type"] == "usd_environment":
        usd = str(get(sc, "environment.usd"))
        if "${" in usd:
            errors.append(f"{tag}: unexpanded variable in environment.usd: {usd}")
        elif not usd.startswith(("/Isaac/", "omniverse://", "http://", "https://")):
            path = usd if os.path.isabs(usd) else os.path.join(os.path.dirname(CONFIG_DIR), usd)
            if not os.path.exists(path):
                errors.append(f"{tag}: environment.usd not found: {path} "
                              "(generated world? run tools/sdf_to_usd.py --convert)")
            elif path.endswith((".usda", ".usd")):
                # a wrapper layer is useless if what it references is not there yet: that is the
                # "the office came up empty" failure, and it costs a full container start to see
                for ref in _references(path):
                    target = ref if os.path.isabs(ref) else os.path.join(os.path.dirname(path), ref)
                    if not os.path.exists(target):
                        msg = (f"{tag}: {os.path.basename(path)} references a missing file: {target}"
                               " - generate it with tools/sdf_to_usd.py --convert (needs Isaac)")
                        (errors if strict else warnings).append(msg)
    for key in ("car.start.x", "car.start.y", "car.start.yaw_deg"):
        if not isinstance(get(sc, key), (int, float)):
            errors.append(f"{tag}: {key} is not a number")
    sem = sc["semantics"]
    for cls in ("unlabeled", "floor", "wall", "person", "car"):
        if cls not in sem:
            errors.append(f"{tag}: semantics misses '{cls}'")
    for i, ch in enumerate((sc.get("people") or {}).get("characters", []) or []):
        if len(ch.get("path", [])) < 2:
            errors.append(f"{tag}: people character {i} needs at least 2 waypoints")
    rate = float(cfg.sensors["stereo"]["rate"])
    if not 1.0 <= rate <= 60.0:
        errors.append(f"{tag}: stereo rate {rate} Hz outside 1..60")
    for w in warnings:
        print(f"  WARNING {w}")
    print(f"  {tag:24s} type={sc['type']:20s} start=({get(sc, 'car.start.x'):.3f}, "
          f"{get(sc, 'car.start.y'):.3f}, {get(sc, 'car.start.yaw_deg'):.1f} deg) "
          f"stereo={rate:.0f} Hz depth_noise={cfg.sensors['stereo']['depth_noise']['enabled']} "
          f"people={bool((sc.get('people') or {}).get('enabled'))}")
    return errors


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", action="append", default=[])
    ap.add_argument("--layouts", action="store_true", help="also load every layout preset of every scene")
    ap.add_argument("--strict", action="store_true",
                    help="a generated world that is not there yet is an error, not a warning "
                         "(use after tools/sdf_to_usd.py --convert)")
    args = ap.parse_args()
    scenes = args.scene or sorted(os.path.splitext(os.path.basename(p))[0]
                                  for p in glob.glob(os.path.join(CONFIG_DIR, "scenes", "*.yaml")))
    errors: list[str] = []
    print(f"scenes in {os.path.join(CONFIG_DIR, 'scenes')}:")
    for s in scenes:
        errors += check_scene(s, None, args.strict)
        if args.layouts:
            import yaml
            with open(os.path.join(CONFIG_DIR, "scenes", f"{s}.yaml")) as f:
                presets = (yaml.safe_load(f) or {}).get("presets") or {}
            for lay in sorted(presets):
                errors += check_scene(s, lay, args.strict)
    if errors:
        print("\nFAILED:")
        for e in errors:
            print(f"  {e}")
        return 1
    print(f"\nOK: {len(scenes)} scenes" + (" and every layout" if args.layouts else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
