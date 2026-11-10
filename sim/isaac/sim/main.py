"""F1Tenth-like car in Isaac Sim 5.1 - entry point.

    /isaac-sim/python.sh sim/main.py --scene base_room [--gui] [--rtf-log] [--duration S] [--no-realtime]

Physics at 60 Hz; cameras rendered only at their publish rate; all sensor data published from
this loop with sim-time stamps (see ros_bridge.py).
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.app import create_app  # noqa: E402  (must come before omni imports)
from sim.config import ConfigError, get, load_config  # noqa: E402

log = logging.getLogger("sim")
PHYSICS_HZ = 60.0


class RateTimer:
    """Fires when sim time crosses k/rate (average rate exact, jitter = one physics step)."""

    def __init__(self, rate: float) -> None:
        self.period = 1.0 / float(rate)
        self.next = 0.0

    def due(self, t: float) -> bool:
        if t + 1e-9 >= self.next:
            self.next += self.period
            if self.next < t:            # after a reset / stall: resync
                self.next = t + self.period
            return True
        return False


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scene", default="base_room")
    p.add_argument("--gui", action="store_true", help="open the Isaac Sim GUI (default headless)")
    p.add_argument("--viewport-res", default="640x360", metavar="WxH",
                   help="GUI viewport render resolution (default 640x360; the window size is separate)")
    p.add_argument("--viewport", choices=("on", "off"), default="on",
                   help="GUI viewport rendering; 'off' keeps the stage/property windows only (cheapest GUI)")
    p.add_argument("--rtf-log", action="store_true", help="log real-time factor every 10 s")
    p.add_argument("--duration", type=float, default=0.0, help="stop after this many sim seconds (0 = run forever)")
    p.add_argument("--no-realtime", action="store_true", help="run as fast as possible (no real-time pacing)")
    p.add_argument("--stats-file", default="", help="write run statistics (json) here on exit")
    p.add_argument("--set", action="append", default=[], metavar="FILE.KEY=VALUE",
                   help="config override, e.g. --set sensors.stereo.semantic.enabled=true (repeatable)")
    p.add_argument("--snapshot-at", default="", metavar="T1,T2,...",
                   help="save the left camera image (PNG, as published) at these sim times [s]")
    p.add_argument("--snapshot-dir", default="/home/isaac_ws/output/snapshots", help="where --snapshot-at writes")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    try:
        cfg = load_config(args.scene, args.set)
    except ConfigError as exc:
        logging.error("configuration error: %s", exc)
        return 2

    vw, vh = (int(v) for v in args.viewport_res.lower().split("x"))
    app = create_app(gui=args.gui, width=vw, height=vh)
    if args.gui and args.viewport == "off":
        from sim.app import disable_viewport
        disable_viewport()
    stop = {"flag": False}

    def _sig(_s, _f):
        stop["flag"] = True
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    try:
        return run(app, cfg, args, stop)
    finally:
        app.close()


def _keyboard_teleop(cfg, node):
    """Keyboard teleop inside the Isaac process, or None when it is off (the default).

    ``ros_interface.yaml`` -> ``keyboard.enabled``. The module is loaded by path, not imported:
    ``tools/isaac_keyboard/isaac_keyboard.py`` is a plain file in the car repo, which the
    container mounts read-only, and the sim directory is not on its import path. A keyboard that
    cannot be set up is logged and skipped -- it must never stop the simulation.
    """
    kb_cfg = (cfg.ros.get("keyboard") or {})
    if not kb_cfg.get("enabled"):
        return None
    module = kb_cfg.get("module") or ""
    if not module or not os.path.exists(module):
        sim_isaac = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # sim/isaac
        fallback = os.path.join(os.path.dirname(os.path.dirname(sim_isaac)),
                                "tools", "isaac_keyboard", "isaac_keyboard.py")
        if module:
            log.warning("keyboard module %s not found, trying %s", module, fallback)
        module = fallback
    try:
        import importlib.util

        spec = importlib.util.spec_from_file_location("isaac_keyboard", module)
        if spec is None or spec.loader is None:
            raise ImportError(f"cannot load {module}")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        kb = mod.IsaacKeyboardTeleop(node, mod.TeleopParams.from_dict(kb_cfg.get("params")),
                                     log=log.info)
    except Exception as exc:  # noqa: BLE001 - no keyboard is never a reason to stop the sim
        log.error("keyboard teleop disabled: %s (%s)", exc, module)
        return None
    log.warning("keyboard teleop ACTIVE on %s: this lane holds the car while it publishes",
                kb.p.topic)
    return kb


def run(app, cfg, args, stop) -> int:
    import numpy as np
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.storage.native import get_assets_root_path

    from sim.car import CAR_ROOT, CarController, CarState, spawn_car
    from sim.noise import NoiseModels, WheelOdometry
    from sim.reset import Resetter
    from sim.ros_bridge import RosBridge
    from sim.scene_builder import build_scene
    from sim.sensors import Sensors
    from sim.transforms import quat_from_rpy

    t0_wall = time.time()
    assets_root = get_assets_root_path()
    if not assets_root:
        log.error("NVIDIA asset root not reachable (internet / OMNI_SERVER?)")
        return 3
    dt = 1.0 / PHYSICS_HZ
    world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt)
    stage = omni.usd.get_context().get_stage()
    scene_info = build_scene(stage, cfg, assets_root, app)
    geom = spawn_car(stage, cfg, assets_root, app)
    sensors = Sensors(stage, cfg, geom)
    sensors.create_imu()
    art = SingleArticulation(CAR_ROOT, name="car")
    world.scene.add(art)
    state = CarState(geom)
    world.reset()
    art.initialize()
    state.initialize()
    controller = CarController(cfg, geom)
    controller.initialize(art)
    wheel_odom = WheelOdometry(get(cfg.car, "geometry.wheelbase"), cfg.sensors["odometry"])
    sem = cfg.scene["semantics"]
    noise = NoiseModels(cfg.sensors, sem.get("floor"))
    from sim.car import RigidBodyImu

    rb_imu = RigidBodyImu(state, sensors.frames_in_base[cfg.ros["frames"]["imu"]])
    imu_source = cfg.sensors["imu"].get("source", "physics_sensor")
    resetter = Resetter(stage, art, controller, wheel_odom, scene_info.person_path, scene_info.person_pose,
                        scene_info.person_asset_yaw)
    log.info("scene ready in %.1f s (wall)", time.time() - t0_wall)

    sim_t = {"t": 0.0}
    ros = RosBridge(cfg, on_command=lambda v, d: controller.command(v, d, sim_t["t"]), on_reset=resetter)
    kb = _keyboard_teleop(cfg, ros.node)
    fr, st = cfg.ros["frames"], cfg.sensors["stereo"]
    if cfg.ros["publish_static_tf"]:
        ros.static_tf(0.0, sensors.static_tree)
    ros.start_pose(0.0, cfg.scene["car"]["start"])
    K = sensors.cameras["left"].K
    baseline = float(get(cfg.car, "sensors.zed.baseline"))
    render_every = max(1, int(round(PHYSICS_HZ / float(st["rate"]))))
    timers = {k: RateTimer(r) for k, r in (("scan", cfg.sensors["lidar"]["rate"]), ("odom", cfg.sensors["odometry"]["rate"]),
                                           ("imu", cfg.sensors["imu"]["rate"]),
                                           ("gt", cfg.sensors["ground_truth"]["rate"]),
                                           ("objects", 1.0), ("front", 20.0))}
    bumper_x = float(get(cfg.car, "geometry.front_bumper_x"))
    laser_z = float(get(cfg.car, "sensors.laser.xyz")[2])
    cov = cfg.sensors["odometry"]["covariance"]
    sem_lut_cache: dict = {}
    stats = {"scene": cfg.scene_name, "layout": cfg.scene.get("layout"), "objects": scene_info.objects,
             "person_fallback": scene_info.person_fallback, "notes": scene_info.notes,
             "labels": scene_info.labels_report,
             "geometry": {"wheelbase": geom.wheelbase, "track": geom.track, "wheel_radius": geom.wheel_radius,
                          "scale": geom.scale, "total_mass": geom.total_mass},
             "camera_K": K.tolist(), "frames": 0, "renders": 0}
    snapshots = sorted(float(s) for s in args.snapshot_at.split(",") if s.strip())
    from sim.app import enforce_native_resolution, render_settings_report
    enforce_native_resolution()
    stats["render_settings"] = render_settings_report()
    log.info("render settings: %s", stats["render_settings"])
    rtf_t0_wall, rtf_t0_sim = time.time(), 0.0
    run_wall0 = time.time()
    k = 0
    log.info("running: physics %.0f Hz, render every %d steps (%.1f Hz), realtime=%s", PHYSICS_HZ, render_every,
             PHYSICS_HZ / render_every, not args.no_realtime)
    exit_reason = "loop end"
    try:
        while app.is_running() and not stop["flag"]:
            ros.spin_once()
            if kb is not None:
                kb.step()        # wall-clock dead-man and rate, independent of the RTF
            controller.step(sim_t["t"], dt)
            camera_frame = (k % render_every == 0)
            # Every render renders ALL render products (both cameras + the GUI viewport), so the GUI viewport
            # refreshes with the cameras (15 Hz) instead of every physics step (60 Hz = 4x the camera work).
            world.step(render=camera_frame)
            k += 1
            t = k * dt
            sim_t["t"] = t
            ros.clock(t)
            # wheel odometry integrates every step (like the VESC at its own rate)
            wheel_odom.update(controller.rear_wheel_speed(), controller.steer, dt)
            if timers["scan"].due(t):
                r = sensors.read_scan()
                if r.size:
                    ros.scan(t, noise.lidar(r))
            if timers["odom"].due(t):
                ros.odom(t, "odom", fr["odom"], fr["base_link"], (wheel_odom.x, wheel_odom.y, 0.0),
                         quat_from_rpy(0, 0, wheel_odom.yaw), (wheel_odom.v, 0, 0), (0, 0, wheel_odom.w), cov,
                         tf=bool(cfg.ros["publish_odom_tf"]))
            if timers["gt"].due(t):
                p, q = state.base_pose()
                if controller.debug and k % 120 == 0:
                    w_, x_, y_, z_ = q
                    pitch = float(np.degrees(np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1))))
                    roll = float(np.degrees(np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_))))
                    log.info("pose dbg t=%.2f base_link %s roll %.2f pitch %.2f deg", t, np.round(p, 3), roll, pitch)
                v, w = state.base_twist()
                ros.odom(t, "gt_odom", fr["map_sim"], fr["base_link"], p, q, v, w, None)
                if scene_info.person_pose:
                    ros.person(t, scene_info.person_pose[0], scene_info.person_pose[1])
            if scene_info.people is not None:
                scene_info.people.update(t)
            if timers["objects"].due(t):
                ros.objects(t, scene_info.objects + (scene_info.people.objects()
                                                     if scene_info.people is not None else []))
            if timers["front"].due(t):
                p, q = state.base_pose()
                ros.front_range(t, _front_range(p, q, bumper_x, laser_z), 30.0)
            rb_imu.update(dt)
            if timers["imu"].due(t):
                f = sensors.read_imu() if imu_source == "physics_sensor" else {}
                g = np.asarray(f.get("ang_vel", [np.nan] * 3), dtype=float)
                a = np.asarray(f.get("lin_acc", [np.nan] * 3), dtype=float)
                q_imu = np.asarray(f.get("orientation", [np.nan] * 4), dtype=float)
                if not (np.all(np.isfinite(g)) and np.all(np.isfinite(a)) and np.all(np.isfinite(q_imu))):
                    if imu_source == "physics_sensor":
                        _debug_once("IMUSensor reading not finite -> rigid-body IMU fallback: gyro %s acc %s q %s"
                                    % (rb_imu.gyro, rb_imu.acc, rb_imu.q))
                        stats["imu_fallback_count"] = stats.get("imu_fallback_count", 0) + 1
                    g, a, q_imu = rb_imu.gyro, rb_imu.acc, rb_imu.q
                g, a = noise.imu(g, a)
                ros.imu(t, g, a, q_imu)
            if camera_frame:
                stats["renders"] += 1
                _publish_cameras(ros, sensors, noise, cfg, t, K, baseline, fr, sem, sem_lut_cache, stats["renders"])
                if snapshots and t + 1e-9 >= snapshots[0]:
                    _snapshot(sensors, args.snapshot_dir, cfg.scene_name, snapshots.pop(0), state, cfg)
            # pacing / statistics
            if not args.no_realtime:
                ahead = t - (time.time() - run_wall0)
                if ahead > 0:
                    time.sleep(ahead)
            if args.rtf_log and time.time() - rtf_t0_wall >= 10.0:
                log.info("RTF %.3f (sim %.1f s / wall %.1f s)", (t - rtf_t0_sim) / (time.time() - rtf_t0_wall),
                         t - rtf_t0_sim, time.time() - rtf_t0_wall)
                rtf_t0_wall, rtf_t0_sim = time.time(), t
            if args.duration and t >= args.duration:
                break
    except Exception as exc:  # noqa: BLE001 - e.g. Kit shutting down under us after SIGTERM
        exit_reason = f"exception: {exc!r}"
        log.warning("main loop stopped: %s", exit_reason)
    finally:
        wall = time.time() - run_wall0
        stats.update(exit_reason=exit_reason, frames=k, sim_time=k * dt, wall_time=wall, rtf=(k * dt) / wall if wall > 0 else 0.0,
                     commands_received=ros.n_cmd, resets=resetter.count,
                     lidar=getattr(sensors, "lidar_azimuth_info", None))
        log.info("stopped: sim %.1f s, wall %.1f s, RTF %.3f, %d commands", k * dt, wall, stats["rtf"], ros.n_cmd)
        if args.stats_file:
            with open(args.stats_file, "w") as fh:
                json.dump(stats, fh, indent=1, default=str)
        try:
            if kb is not None:
                kb.close()
            ros.shutdown()
            world.stop()
        except Exception:  # noqa: BLE001
            pass

    return 0


# The annotators return empty buffers for the first rendered frames (the render pipeline is a few frames deep):
# expected, logged at debug level; a camera still empty after this many renders is a real problem.
CAMERA_WARMUP_RENDERS = 5


def _publish_cameras(ros, sensors, noise, cfg, t, K, baseline, fr, sem, lut_cache, n_render: int = 0) -> None:
    import numpy as np

    st = cfg.sensors["stereo"]
    left = sensors.read_camera("left")
    rgb = np.asarray(left.get("rgb"))
    if rgb.ndim != 3 or rgb.size == 0:
        msg = "camera rgb not ready (render %d): type=%s shape=%s" % (n_render, type(left.get("rgb")).__name__, rgb.shape)
        if n_render <= CAMERA_WARMUP_RENDERS:
            log.debug(msg)
        else:
            _debug_once(msg)
        return
    bgra = np.ascontiguousarray(rgb[:, :, [2, 1, 0, 3]]) if st["image_encoding"] == "bgra8" else \
        np.ascontiguousarray(rgb[:, :, :3])
    enc = st["image_encoding"]
    ros.image(t, ["left_image", "camera_image"], fr["zed_left_optical"], bgra, enc)
    ros.camera_info(t, K, fr["zed_left_optical"], ["left_info", "camera_info", "depth_info"])
    sem_ids = None
    if "semantic" in left:
        sem_ids = _semantic_ids(left["semantic"], sem, lut_cache)
        if sem_ids is not None and "gt_semantic" in ros.pub:
            ros.image(t, ["gt_semantic"], fr["zed_left_optical"], sem_ids.astype(np.uint8), "mono8")
    depth = np.asarray(left.get("depth"), dtype=np.float32)
    if depth.ndim == 2 and depth.size:
        d = depth.copy()
        d[~np.isfinite(d)] = np.inf
        too_close = d < float(st["depth_min"])
        too_far = d > float(st["depth_max"])
        d[too_close] = float(st["depth_too_close_value"])
        d[too_far] = float(st["depth_too_far_value"])
        d, conf = noise.depth(d, sem_ids)
        ros.image(t, ["depth"], fr["zed_left_optical"], np.ascontiguousarray(d, dtype=np.float32), "32FC1")
        if conf is not None and "confidence" in ros.pub:
            ros.image(t, ["confidence"], fr["zed_left_optical"], conf, "32FC1")
    if "right" in sensors.cameras and "right_image" in ros.pub:
        rr = np.asarray(sensors.read_camera("right").get("rgb"))
        if rr.ndim == 3 and rr.size:
            img = np.ascontiguousarray(rr[:, :, [2, 1, 0, 3]]) if enc == "bgra8" else np.ascontiguousarray(rr[:, :, :3])
            ros.image(t, ["right_image"], fr["zed_right_optical"], img, enc)
            ros.camera_info(t, K, fr["zed_right_optical"], ["right_info"], tx=-K[0, 0] * baseline)


def _front_range(p, q, bumper_x: float, z: float, max_range: float = 30.0) -> float:
    """PhysX raycast from the front bumper along the car heading at laser height (inf: no hit)."""
    import math

    from omni.physx import get_physx_scene_query_interface

    w, x, y, zq = (float(v) for v in q)
    yaw = math.atan2(2 * (w * zq + x * y), 1 - 2 * (y * y + zq * zq))
    c, s = math.cos(yaw), math.sin(yaw)
    origin = (float(p[0]) + bumper_x * c, float(p[1]) + bumper_x * s, float(p[2]) + z)
    hit = get_physx_scene_query_interface().raycast_closest(origin, (c, s, 0.0), max_range)
    return float(hit["distance"]) if hit["hit"] else float("inf")


def _snapshot(sensors, out_dir: str, scene: str, t: float, state=None, cfg=None) -> None:
    """Left camera RGB exactly as rendered (no resize) -> <out_dir>/<scene>_left_t<t>.png, and the lidar scan
    with the ground-truth base_link pose -> <scene>_scan_t<t>.npz (ranges, angles, pose x y yaw)."""
    import math

    import numpy as np
    from PIL import Image

    os.makedirs(out_dir, exist_ok=True)
    rgb = np.asarray(sensors.read_camera("left").get("rgb"))
    if rgb.ndim != 3 or rgb.size == 0:
        log.warning("snapshot at t=%.2f: camera not ready", t)
    else:
        path = os.path.join(out_dir, f"{scene}_left_t{t:05.1f}.png")
        Image.fromarray(np.ascontiguousarray(rgb[:, :, :3])).save(path)
        log.info("snapshot %s (%dx%d)", path, rgb.shape[1], rgb.shape[0])
    if state is not None and cfg is not None:
        r = sensors.read_scan()
        lc = cfg.sensors["lidar"]
        p, q = state.base_pose()
        w, x, y, z = (float(v) for v in q)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        ang = float(lc["angle_min"]) + np.arange(r.size) * float(lc["angle_increment"])
        path = os.path.join(out_dir, f"{scene}_scan_t{t:05.1f}.npz")
        np.savez(path, ranges=r, angles=ang, pose=np.array([p[0], p[1], yaw]))
        log.info("snapshot %s: %d beams, %d finite, %d < 10 m", path, r.size, int(np.isfinite(r).sum()),
                 int((r < 10.0).sum()))


_DEBUG_SEEN: dict = {}


def _debug_once(msg: str, limit: int = 3) -> None:
    n = _DEBUG_SEEN.get(msg[:40], 0)
    if n < limit:
        log.warning(msg)
        _DEBUG_SEEN[msg[:40]] = n + 1


def _semantic_ids(data, sem: dict, lut_cache: dict):
    """Annotator ids -> config class ids (unlabeled 0)."""
    import numpy as np

    if not isinstance(data, dict) or "data" not in data:
        return None
    raw = np.asarray(data["data"]).astype(np.int64)
    id2 = data.get("info", {}).get("idToLabels", {})
    out = np.zeros(raw.shape, dtype=np.uint8)
    for sid, lab in id2.items():
        cls = lab.get("class", "") if isinstance(lab, dict) else str(lab)
        cid = sem.get(cls.split(",")[0].strip().lower(), sem.get("unlabeled", 0))
        out[raw == int(sid)] = cid
    return out


if __name__ == "__main__":
    sys.exit(main())
