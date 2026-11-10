"""SimulationApp creation (headless by default) and extension enabling.

Must be imported/called before any ``omni``/``isaacsim.*`` module other than ``isaacsim``.
"""
from __future__ import annotations

import logging
import os
import sys

log = logging.getLogger(__name__)

EXTENSIONS = [
    "isaacsim.ros2.bridge",
    "isaacsim.sensors.physx",
    "isaacsim.sensors.physics",
    "isaacsim.sensors.camera",
    "isaacsim.robot.wheeled_robots",
]


def create_app(gui: bool, width: int = 1280, height: int = 720):
    """Create the SimulationApp. Headless unless ``gui``; lean renderer settings to save VRAM."""
    from isaacsim import SimulationApp

    app = SimulationApp({
        "headless": not gui,
        "width": width,
        "height": height,
        "renderer": "RaytracedLighting",
        "anti_aliasing": NATIVE_AA_OP,   # DLAA: native resolution (see enforce_native_resolution)
        "hide_ui": not gui,
    })
    from isaacsim.core.utils.extensions import enable_extension

    for ext in EXTENSIONS:
        enable_extension(ext)
    app.update()
    _settings()
    _ensure_rclpy_path()
    return app


def _settings() -> None:
    """Renderer settings that cut VRAM / GPU time; none of them affect sensor geometry."""
    import carb

    s = carb.settings.get_settings()
    s.set("/rtx/translucency/enabled", False)
    s.set("/rtx/reflections/enabled", False)
    s.set("/rtx/indirectDiffuse/enabled", False)
    s.set("/rtx/ambientOcclusion/enabled", False)
    s.set("/app/asyncRendering", False)
    s.set("/app/runLoops/main/rateLimitEnabled", False)
    enforce_native_resolution()


RENDER_KEYS = ["/rtx/rendermode", "/rtx/post/aa/op", "/rtx/post/dlss/execMode",
               "/rtx-transient/resourcemanager/texturestreaming/memoryBudget"]


# /rtx/post/aa/op: the RTX real-time renderer of Kit 107 (Isaac Sim 5.1) knows only 3 = DLSS and 4 = DLAA
# (omni.rtx.settings.core rt_widgets.py: anything else is "Invalid AA Mode"; measured: op 0 or 2 render the
# same upscaled image as 3). The app config's anti_aliasing=0 did not stick either (op read back 3). DLSS
# "Performance" renders a 640x360 camera at 320x180 and upscales it (Kit warning "DLSS increasing input
# dimensions ... 320, 180"). DLAA is the same network at render scale 1.0: native resolution, no upscaling.
NATIVE_AA_OP = 4


def enforce_native_resolution() -> None:
    """Cameras render at their own resolution: DLAA (anti-aliasing at render scale 1.0), never DLSS upscaling."""
    import carb

    s = carb.settings.get_settings()
    for root in ("/rtx", "/rtx-defaults"):
        s.set(f"{root}/post/aa/op", NATIVE_AA_OP)


def render_settings_report() -> dict:
    """Current values of the renderer settings that decide the cameras' render resolution."""
    import carb

    s = carb.settings.get_settings()
    return {k: s.get(k) for k in RENDER_KEYS}


def disable_viewport() -> None:
    """GUI without viewport rendering: the active viewport stops updating (its render product is not drawn)."""
    try:
        from omni.kit.viewport.utility import get_active_viewport

        vp = get_active_viewport()
        if vp is not None:
            vp.updates_enabled = False
            log.info("GUI viewport updates disabled (--viewport off)")
    except Exception as exc:  # noqa: BLE001
        log.warning("could not disable the viewport: %s", exc)


def _ensure_rclpy_path() -> None:
    """Use the bridge's internal rclpy of ROS_DISTRO (Python 3.11), never host ROS."""
    distro = os.environ.get("ROS_DISTRO", "jazzy")
    path = f"/isaac-sim/exts/isaacsim.ros2.bridge/{distro}/rclpy"
    sys.path[:] = [p for p in sys.path if not p.startswith("/opt/ros")]
    if os.path.isdir(path) and path not in sys.path:
        sys.path.append(path)
    log.info("rclpy path: %s (ROS_DISTRO=%s, LD_LIBRARY_PATH has bridge: %s)", path, distro,
             f"isaacsim.ros2.bridge/{distro}/lib" in os.environ.get("LD_LIBRARY_PATH", ""))
