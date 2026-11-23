"""Sensors: PhysX 2D lidar, ZED2-like stereo pair (+ depth, optional semantics) and an IMU.

All sensor prims are children of the chassis rigid body. Their poses come from
``car_spec.yaml`` (in base_link) and are converted to the chassis frame, compensating the
uniform scale of the car so that sensor world poses are exact. Prim names are the ROS frame ids.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field

import numpy as np
from pxr import Gf, Usd, UsdGeom

from sim.car import CarGeometry, _world_matrix
from sim.config import SimConfig, get
from sim.transforms import R_BODY_OPTICAL, R_BODY_USDCAM, make_tf, quat_from_rpy, quat_to_matrix, tf_inv

log = logging.getLogger(__name__)


@dataclass
class StaticFrame:
    """A static transform parent -> child (4x4)."""

    parent: str
    child: str
    tf: np.ndarray


def sensor_frames(cfg: SimConfig) -> tuple[dict[str, np.ndarray], list[StaticFrame]]:
    """Sensor frames in base_link (4x4) and the static TF tree, from car_spec + ros_interface."""
    s, fr = cfg.car["sensors"], cfg.ros["frames"]
    base = fr["base_link"]
    laser = make_tf(get(s, "laser.xyz"), quat_from_rpy(*get(s, "laser.rpy")))
    imu = make_tf(get(s, "imu.xyz"), quat_from_rpy(*get(s, "imu.rpy")))
    pitch = float(get(s, "zed.pitch"))
    r_link = quat_to_matrix(quat_from_rpy(0.0, pitch, 0.0))
    l2c = np.array(get(s, "zed.link_to_center_xyz"), dtype=float)
    c2l = np.array(get(s, "zed.center_to_left_xyz"), dtype=float)
    c2r = np.array(get(s, "zed.center_to_right_xyz"), dtype=float)
    half_b = float(get(s, "zed.baseline")) / 2.0
    c2l[1], c2r[1] = half_b, -half_b      # baseline from config wins over the wrapper chain
    lens_mid_in_link = l2c + (c2l + c2r) / 2.0
    lens = np.array(get(s, "zed.lens_center_xyz"), dtype=float)
    link = make_tf(lens - r_link @ lens_mid_in_link, rot=r_link)
    link_center = make_tf(l2c)
    center_left, center_right = make_tf(c2l), make_tf(c2r)
    optical = make_tf(np.zeros(3), rot=R_BODY_OPTICAL)
    tree = [
        StaticFrame(base, fr["laser"], laser),
        StaticFrame(base, fr["imu"], imu),
        StaticFrame(base, fr["zed_link"], link),
        StaticFrame(fr["zed_link"], fr["zed_center"], link_center),
        StaticFrame(fr["zed_center"], fr["zed_left"], center_left),
        StaticFrame(fr["zed_center"], fr["zed_right"], center_right),
        StaticFrame(fr["zed_left"], fr["zed_left_optical"], optical),
        StaticFrame(fr["zed_right"], fr["zed_right_optical"], optical),
    ]
    in_base: dict[str, np.ndarray] = {base: np.eye(4)}
    for f in tree:   # ordered parent-first
        in_base[f.child] = in_base[f.parent] @ f.tf
    return in_base, tree


def _place(stage: Usd.Stage, path: str, world: np.ndarray, chassis_world_usd: np.ndarray, typ: str = "Xform"):
    """Define a prim at ``path`` (child of the chassis) with the given world pose (unit scale in world)."""
    local = tf_inv_general(chassis_world_usd) @ world
    prim = stage.DefinePrim(path, typ)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d(local.T.tolist()))
    return prim


def tf_inv_general(m: np.ndarray) -> np.ndarray:
    return np.linalg.inv(m)


@dataclass
class CameraHandle:
    name: str
    path: str
    render_product: object
    annotators: dict = field(default_factory=dict)
    K: np.ndarray | None = None


class Sensors:
    """Creates the sensor prims and reads their data."""

    def __init__(self, stage: Usd.Stage, cfg: SimConfig, geom: CarGeometry) -> None:
        self.cfg, self.geom, self.stage = cfg, geom, stage
        self.frames_in_base, self.static_tree = sensor_frames(cfg)
        self.chassis_world_usd = _world_matrix(stage.GetPrimAtPath(geom.chassis_path))
        self.base_world0 = geom.base_in_world0
        fr = cfg.ros["frames"]
        self.fr = fr
        self.lidar_path = f"{geom.chassis_path}/{fr['laser']}"
        self.imu_path = f"{geom.chassis_path}/{fr['imu']}"
        self.cameras: dict[str, CameraHandle] = {}
        self._lidar_iface = None
        self._lidar_map: np.ndarray | None = None
        self.imu = None
        self._create_lidar()
        self._create_cameras()

    # ------------------------------------------------------------------ lidar
    def _create_lidar(self) -> None:
        import omni.isaac.RangeSensorSchema as RangeSensorSchema  # noqa: N813 - NVIDIA module name
        from pxr import Sdf

        lc = self.cfg.sensors["lidar"]
        world = self.base_world0 @ self.frames_in_base[self.fr["laser"]]
        _place(self.stage, self.lidar_path, world, self.chassis_world_usd, "Xform")
        self.stage.RemovePrim(self.lidar_path)
        lidar = RangeSensorSchema.Lidar.Define(self.stage, Sdf.Path(self.lidar_path))
        xf = UsdGeom.Xformable(lidar)
        xf.AddTransformOp().Set(Gf.Matrix4d((tf_inv_general(self.chassis_world_usd) @ world).T.tolist()))
        fov_deg = math.degrees(float(lc["angle_max"]) - float(lc["angle_min"]))
        res_deg = math.degrees(float(lc["angle_increment"]))
        # PhysX columns sit at -fov/2 + i*res (i < fov/res, rounded down). Widening by two steps on each side
        # keeps a column exactly on every real beam angle (angle_min + i*increment), both end beams included
        # (one step per side is not enough: the column count is truncated). Extra columns are discarded.
        lidar.CreateHorizontalFovAttr().Set(float(fov_deg + 4 * res_deg))
        lidar.CreateHorizontalResolutionAttr().Set(float(res_deg))
        lidar.CreateVerticalFovAttr().Set(1.0)
        lidar.CreateVerticalResolutionAttr().Set(1.0)
        lidar.CreateMinRangeAttr().Set(float(lc["range_min"]))
        lidar.CreateMaxRangeAttr().Set(float(lc["physx_max_range"]))
        lidar.CreateRotationRateAttr().Set(0.0)       # full scan every physics step
        lidar.CreateHighLodAttr().Set(False)          # single horizontal row
        lidar.CreateDrawPointsAttr().Set(False)
        lidar.CreateDrawLinesAttr().Set(False)
        lidar.CreateYawOffsetAttr().Set(0.0)
        lidar.CreateEnableSemanticsAttr().Set(False)
        log.info("lidar %s: fov %.3f deg res %.4f deg range %.2f-%.1f m", self.lidar_path, fov_deg + 4 * res_deg,
                 res_deg, lc["range_min"], lc["physx_max_range"])

    def read_scan(self) -> np.ndarray:
        """Ranges [m] at the real scanner's angles (angle_min + i*increment), REP-117 inf for no return."""
        from isaacsim.sensors.physx import _range_sensor

        lc = self.cfg.sensors["lidar"]
        if self._lidar_iface is None:
            self._lidar_iface = _range_sensor.acquire_lidar_sensor_interface()
        depth = np.asarray(self._lidar_iface.get_linear_depth_data(self.lidar_path), dtype=np.float32).reshape(-1)
        if self._lidar_map is None:
            az = np.asarray(self._lidar_iface.get_azimuth_data(self.lidar_path), dtype=np.float64).reshape(-1)
            if az.size == 0 or depth.size == 0:
                return np.array([])
            n = int(round((float(lc["angle_max"]) - float(lc["angle_min"])) / float(lc["angle_increment"]))) + 1
            target = float(lc["angle_min"]) + np.arange(n) * float(lc["angle_increment"])
            idx = np.array([int(np.argmin(np.abs(az - a))) for a in target])
            err = np.max(np.abs(az[idx] - target))
            log.info("lidar: %d physx columns, azimuth %.4f..%.4f rad; mapped to %d beams, max angle err %.5f rad",
                     az.size, az.min(), az.max(), n, err)
            self._lidar_map = idx
            self.lidar_azimuth_info = {"physx_columns": int(az.size), "az_min": float(az.min()),
                                       "az_max": float(az.max()), "beams": n, "max_angle_err": float(err)}
        r = depth[self._lidar_map].astype(np.float32)
        r[(r >= float(lc["physx_max_range"]) * 0.999) | (r <= 0.0)] = float(lc["no_return_value"])
        return r

    # ---------------------------------------------------------------- cameras
    def _create_cameras(self) -> None:
        import omni.replicator.core as rep

        sc = self.cfg.sensors["stereo"]
        w, h = int(sc["width"]), int(sc["height"])
        ha = float(sc["horizontal_aperture_mm"])
        focal = float(sc["fx"]) * ha / w
        va = ha * h / w * float(sc["fx"]) / float(sc["fy"])
        names = [("left", self.fr["zed_left"], self.fr["zed_left_optical"])]
        if sc["right_camera"]:
            names.append(("right", self.fr["zed_right"], self.fr["zed_right_optical"]))
        for name, body_frame, optical_frame in names:
            body_world = self.base_world0 @ self.frames_in_base[body_frame]
            cam_world = body_world @ make_tf(np.zeros(3), rot=R_BODY_USDCAM)
            path = f"{self.geom.chassis_path}/{optical_frame}"
            prim = _place(self.stage, path, cam_world, self.chassis_world_usd, "Camera")
            cam = UsdGeom.Camera(prim)
            cam.CreateFocalLengthAttr(focal)
            cam.CreateHorizontalApertureAttr(ha)
            cam.CreateVerticalApertureAttr(va)
            cam.CreateHorizontalApertureOffsetAttr(0.0)
            cam.CreateVerticalApertureOffsetAttr(0.0)
            cam.CreateClippingRangeAttr(Gf.Vec2f(float(sc["clipping_near"]), float(sc["clipping_far"])))
            cam.CreateProjectionAttr("perspective")
            rp = rep.create.render_product(path, (w, h), name=f"rp_{name}")
            handle = CameraHandle(name=name, path=path, render_product=rp)
            handle.annotators["rgb"] = rep.AnnotatorRegistry.get_annotator("rgb")
            handle.annotators["rgb"].attach([rp])
            if name == "left":
                handle.annotators["depth"] = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
                handle.annotators["depth"].attach([rp])
                if sc["semantic"]["enabled"] or sc["depth_noise"]["enabled"]:
                    handle.annotators["semantic"] = rep.AnnotatorRegistry.get_annotator(
                        "semantic_segmentation", init_params={"colorize": False})
                    handle.annotators["semantic"].attach([rp])
            # K actually rendered (pinhole from USD attributes; aperture offsets are 0 -> principal point centred)
            fx = focal * w / ha
            fy = focal * h / va
            handle.K = np.array([[fx, 0.0, w / 2.0], [0.0, fy, h / 2.0], [0.0, 0.0, 1.0]])
            self.cameras[name] = handle
            log.info("camera %s: %dx%d f=%.4f mm ha=%.3f va=%.3f -> K fx=%.3f fy=%.3f cx=%.1f cy=%.1f", name, w, h,
                     focal, ha, va, fx, fy, w / 2.0, h / 2.0)

    def read_camera(self, name: str) -> dict:
        out = {}
        for key, ann in self.cameras[name].annotators.items():
            out[key] = ann.get_data()
        return out

    # -------------------------------------------------------------------- imu
    def create_imu(self) -> None:
        """IMU must be created after the stage is set up (needs the body prim)."""
        from isaacsim.sensors.physics import IMUSensor

        world = self.base_world0 @ self.frames_in_base[self.fr["imu"]]
        local = tf_inv_general(self.chassis_world_usd) @ world
        r = local[:3, :3]
        sc = np.linalg.norm(r[:, 0])
        from sim.transforms import matrix_to_quat
        q = matrix_to_quat(r / sc)
        self.imu = IMUSensor(prim_path=self.imu_path, name="imu", frequency=int(self.cfg.sensors["imu"]["rate"]),
                             translation=local[:3, 3], orientation=q)
        log.info("imu at %s", self.imu_path)

    def read_imu(self) -> dict:
        return self.imu.get_current_frame()
