#!/usr/bin/env python3
"""Run sim/isaac/tools/sdf_to_usd.py inside a headless Kit app (native Windows).

    convert_runner.py <path to sdf_to_usd.py> [its own arguments ...]

sdf_to_usd.py --convert imports omni.kit.asset_converter, which exists only while Kit runs: this
starts a headless SimulationApp, enables the extension, then runs the script unchanged.
"""
import os
import runpy
import sys

script, args = sys.argv[1], sys.argv[2:]
from isaacsim import SimulationApp  # noqa: E402  (must come before any omni import)

app = SimulationApp({"headless": True})
rc = 1
try:
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.kit.asset_converter")
    app.update()
    sys.argv = [script, *args]
    sys.path.insert(0, os.path.dirname(os.path.abspath(script)))
    try:
        runpy.run_path(script, run_name="__main__")
        rc = 0
    except SystemExit as exc:
        rc = exc.code if isinstance(exc.code, int) else (0 if exc.code is None else 1)
finally:
    app.close()
sys.exit(rc)
