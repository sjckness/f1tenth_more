"""Configuration loading and validation.

All numbers used by the simulation live in YAML under ``config/``. This module loads
them once, validates that every required key exists (fail fast with a clear message)
and exposes them as plain dictionaries. ``val()`` unwraps the ``{value:, source:}``
records used in ``car_spec.yaml``.
"""
from __future__ import annotations

import logging
import os
import re
from dataclasses import dataclass
from typing import Any

import yaml

log = logging.getLogger(__name__)

WS = os.environ.get("ISAAC_WS", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CONFIG_DIR = os.path.join(WS, "config")


class ConfigError(RuntimeError):
    """Raised when a config file is missing or lacks a required key."""


def val(node: Any) -> Any:
    """Return ``node['value']`` for spec records, the node itself otherwise."""
    if isinstance(node, dict) and "value" in node:
        return node["value"]
    return node


def get(cfg: dict, dotted: str, file: str = "?") -> Any:
    """Fetch ``a.b.c`` from a nested dict, raising ConfigError if missing."""
    cur: Any = cfg
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            raise ConfigError(f"{file}: missing required key '{dotted}'")
        cur = cur[part]
    return val(cur)


_ENV_RE = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)(?::-([^}]*))?\}")


def expand_env(value: str, where: str = "?") -> str:
    """Expand ``${VAR}`` / ``${VAR:-default}`` in a config string.

    An unset variable without a default is an error, never an empty string: a machine-specific
    path (the office USD of the HIL scene) that silently expanded to '' would reach Isaac as a
    missing asset and the scene would come up empty.
    """
    def sub(m: "re.Match[str]") -> str:
        var, default = m.group(1), m.group(2)
        val = os.environ.get(var)
        if val:
            return val
        if default is not None:
            return default
        raise ConfigError(f"{where}: environment variable {var} is not set (used as ${{{var}}})")
    return _ENV_RE.sub(sub, value)


def _expand_tree(node: Any, where: str) -> Any:
    if isinstance(node, str):
        return expand_env(node, where) if "${" in node else node
    if isinstance(node, dict):
        return {k: _expand_tree(v, where) for k, v in node.items()}
    if isinstance(node, list):
        return [_expand_tree(v, where) for v in node]
    return node


def _load(path: str) -> dict:
    if not os.path.isfile(path):
        raise ConfigError(f"config file not found: {path}")
    with open(path) as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: expected a mapping at top level")
    return _expand_tree(data, os.path.basename(path))


REQUIRED = {
    "car_spec.yaml": [
        "geometry.wheelbase", "geometry.track_front", "geometry.wheel_radius",
        "mass.total", "limits.max_steering_left", "limits.max_steering_right", "limits.max_speed",
        "limits.max_reverse_speed", "limits.steering_rate", "limits.acceleration", "limits.speed_tau",
        "limits.steering_tau", "limits.command_timeout", "friction.static", "friction.dynamic",
        "friction.restitution", "sensors.laser.xyz", "sensors.laser.rpy", "sensors.imu.xyz", "sensors.imu.rpy",
        "sensors.zed.lens_center_xyz", "sensors.zed.pitch", "sensors.zed.baseline",
        "sensors.zed.link_to_center_xyz", "sensors.zed.center_to_left_xyz", "sensors.zed.center_to_right_xyz",
        "leatherback.usd", "leatherback.wheelbase", "leatherback.track", "leatherback.wheel_radius",
        "leatherback.base_mass_explicit", "leatherback.steering_joints", "leatherback.drive_joints",
        "leatherback.rear_wheel_bodies", "leatherback.chassis_body", "leatherback.invert_steering",
        "leatherback.scale", "leatherback.hide_body_shell",
    ],
    "sensors.yaml": [
        "lidar.angle_min", "lidar.angle_max", "lidar.angle_increment", "lidar.range_min", "lidar.range_max",
        "lidar.rate", "lidar.time_increment", "lidar.scan_time", "lidar.physx_max_range", "lidar.noise.enabled",
        "lidar.noise.stddev", "lidar.no_return_value",
        "stereo.width", "stereo.height", "stereo.rate", "stereo.fx", "stereo.fy", "stereo.cx", "stereo.cy",
        "stereo.distortion_model", "stereo.horizontal_aperture_mm", "stereo.clipping_near", "stereo.clipping_far",
        "stereo.depth_min", "stereo.depth_max", "stereo.depth_too_close_value", "stereo.depth_too_far_value",
        "stereo.image_encoding", "stereo.right_camera", "stereo.semantic.enabled", "stereo.depth_noise.enabled",
        "stereo.depth_noise.floor_nan_beyond", "stereo.depth_noise.floor_nan_probability",
        "stereo.depth_noise.k_quadratic", "stereo.depth_noise.publish_confidence", "stereo.depth_noise.seed",
        "imu.rate", "imu.noise.enabled", "imu.noise.gyro_stddev", "imu.noise.accel_stddev",
        "odometry.rate", "odometry.noise.enabled", "odometry.noise.speed_scale_error", "odometry.noise.yaw_rate_bias",
        "odometry.covariance.pose_xy", "odometry.covariance.pose_yaw", "odometry.covariance.twist_vx",
        "odometry.covariance.twist_wz", "ground_truth.rate",
    ],
    "ros_interface.yaml": [
        "ros.domain_id", "ros.node_name", "publish_static_tf", "publish_odom_tf",
        "frames.map_sim", "frames.odom", "frames.base_link", "frames.laser", "frames.imu", "frames.zed_link",
        "frames.zed_center", "frames.zed_left", "frames.zed_left_optical", "frames.zed_right",
        "frames.zed_right_optical",
        "topics.clock", "topics.command", "topics.scan", "topics.odom", "topics.imu", "topics.left_image",
        "topics.left_info", "topics.right_image", "topics.right_info", "topics.depth", "topics.depth_info",
        "topics.confidence", "topics.camera_image", "topics.camera_info", "topics.gt_odom", "topics.gt_person",
        "topics.gt_semantic", "topics.reset_service",
        "qos.sensor", "qos.command", "qos.clock", "qos.tf_static", "qos.tf", "qos.odom",
    ],
}
SCENE_REQUIRED = {
    "procedural_room": ["name", "room.size_x", "room.size_y", "room.wall_height", "room.wall_thickness",
                        "room.ceiling", "floor.material", "materials", "lighting.dome_intensity",
                        "lighting.distant_intensity", "lighting.distant_angle_deg", "lighting.shadows",
                        "person.enabled", "person.usd", "person.position", "person.face_car", "person.yaw_deg",
                        "person.collider.radius", "person.collider.height", "car.start.x", "car.start.y",
                        "car.start.yaw_deg", "semantics"],
    # Long corridor (HIL): one straight corridor, glossy floor by default, optional glass wall.
    "procedural_corridor": ["name", "corridor.length", "corridor.width", "corridor.wall_height",
                            "corridor.wall_thickness", "corridor.ceiling", "floor.material", "materials",
                            "lighting.dome_intensity", "lighting.shadows", "person.enabled", "person.usd",
                            "person.position", "person.face_car", "person.yaw_deg", "person.collider.radius",
                            "person.collider.height", "car.start.x", "car.start.y", "car.start.yaw_deg",
                            "semantics"],
    "usd_environment": ["name", "environment.usd", "environment.labels", "lighting.extra_dome_intensity",
                        "person.enabled", "person.usd", "person.position", "person.face_car", "person.yaw_deg",
                        "person.collider.radius", "person.collider.height", "car.start.x", "car.start.y",
                        "car.start.yaw_deg", "semantics"],
}


@dataclass
class SimConfig:
    """All configuration of one run."""

    car: dict
    sensors: dict
    ros: dict
    scene: dict
    scene_name: str


def apply_override(files: dict, scene_cfg: dict, spec: str) -> None:
    """Apply ``<file>.<dotted.key>=<yaml value>``; file in {car_spec, sensors, ros_interface, scene}."""
    if "=" not in spec:
        raise ConfigError(f"override '{spec}' must look like sensors.stereo.semantic.enabled=true")
    key, raw = spec.split("=", 1)
    file, _, dotted = key.partition(".")
    target = scene_cfg if file == "scene" else files.get(f"{file}.yaml")
    if target is None or not dotted:
        raise ConfigError(f"override '{spec}': unknown file '{file}'")
    parts = dotted.split(".")
    cur = target
    for p in parts[:-1]:
        if p not in cur or not isinstance(cur[p], dict):
            raise ConfigError(f"override '{spec}': key '{dotted}' not found")
        cur = cur[p]
    if parts[-1] not in cur:
        raise ConfigError(f"override '{spec}': key '{dotted}' not found")
    value = yaml.safe_load(raw)
    if isinstance(cur[parts[-1]], dict) and "value" in cur[parts[-1]]:
        cur[parts[-1]]["value"] = value
    else:
        cur[parts[-1]] = value
    log.warning("config override: %s = %r", key, value)


def _merge(dst: dict, src: dict) -> None:
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _merge(dst[k], v)
        else:
            dst[k] = v


def apply_layout(scene_cfg: dict, scene_file: str = "?") -> None:
    """Merge ``presets[<layout>]`` into the scene (car start, person, plant, obstacle per named layout)."""
    layout = scene_cfg.get("layout")
    presets = scene_cfg.get("presets") or {}
    if not layout:
        return
    if layout not in presets:
        raise ConfigError(f"{scene_file}: layout '{layout}' not in presets {sorted(presets)}")
    _merge(scene_cfg, presets[layout])
    log.info("scene layout: %s", layout)


def _scene_sensor_overrides(scene_cfg: dict, scene_file: str) -> list[str]:
    """``sensor_overrides: {sensors.stereo.rate: 10}`` in a scene file -> ``--set`` specs."""
    out = []
    for key, value in (scene_cfg.get("sensor_overrides") or {}).items():
        file = str(key).split(".", 1)[0]
        if file not in ("sensors", "car_spec", "ros_interface"):
            raise ConfigError(f"{scene_file}: sensor_overrides key '{key}' must start with "
                              "sensors., car_spec. or ros_interface.")
        out.append(f"{key}={yaml.safe_dump(value, default_flow_style=True).strip()}")
    return out


def load_config(scene: str, overrides: list[str] | None = None) -> SimConfig:
    """Load and validate all YAML files for ``scene`` (name of ``config/scenes/<scene>.yaml``)."""
    files = {name: _load(os.path.join(CONFIG_DIR, name)) for name in REQUIRED}
    for name, keys in REQUIRED.items():
        for k in keys:
            get(files[name], k, name)
    scene_file = os.path.join(CONFIG_DIR, "scenes", f"{scene}.yaml")
    scene_cfg = _load(scene_file)
    overrides = list(overrides or [])
    # 1) the layout choice, 2) the layout preset over the scene defaults, 3) every other override on top
    for spec in [o for o in overrides if o.split("=", 1)[0] == "scene.layout"]:
        apply_override(files, scene_cfg, spec)
    apply_layout(scene_cfg, scene_file)
    # A scene may declare sensor/car overrides of its own (scenes/corridor.yaml: the glossy floor
    # turns on the ZED floor-NaN depth model, which is what the real car sees on that floor).
    # Applied BEFORE the command line, so --set always wins.
    for spec in _scene_sensor_overrides(scene_cfg, scene_file):
        apply_override(files, scene_cfg, spec)
    for spec in [o for o in overrides if o.split("=", 1)[0] != "scene.layout"]:
        apply_override(files, scene_cfg, spec)
    stype = scene_cfg.get("type")
    if stype not in SCENE_REQUIRED:
        raise ConfigError(f"{scene_file}: 'type' must be one of {list(SCENE_REQUIRED)}, got {stype!r}")
    for k in SCENE_REQUIRED[stype]:
        get(scene_cfg, k, scene_file)
    log.info("config loaded: scene=%s (%s)", scene, stype)
    return SimConfig(car=files["car_spec.yaml"], sensors=files["sensors.yaml"], ros=files["ros_interface.yaml"],
                     scene=scene_cfg, scene_name=scene)
