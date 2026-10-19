"""
Object memory with visibility-aware decay. Pure: no rclpy, no messages.

The semantic layer used to forget a confirmed object lost_miss_count detection
batches after the camera stopped reporting it (~0.3 s at camera rate), with
no idea WHY it stopped: the object may have left, or the car may simply have
turned away from it, or something may stand in front of it. Those are
different facts and deserve different answers. This module gives each
confirmed track an ObjectMemory and decides, once per memory cycle, which one
applies (semantic_layer_node.py wires the sensors in; nothing here touches
ROS).

VISIBILITY, PER SENSOR (object_visibility):
  LiDAR   the object centre, in the scan's own frame, lies within the scan's
          OWN angle_min..angle_max (so the rear blind cone of the 270 deg
          Hokuyo is out of view by construction, nothing hard-coded) and its
          near surface within range_min..range_max, and the beams spanning
          the object do not mostly return closer than its near surface
          (occlusion_margin_m slack). A beam with no valid return says
          nothing and never occludes.
  Stereo  the camera is live (a detection batch was processed recently), the
          whole footprint is inside the horizontal frustum (|bearing| plus
          the object's own angular radius within hfov/2), its depth along the
          optical axis is within the depth range in which a missed detection
          means something, and it is not occluded. Occlusion uses two sources:
          the StereoColumns depth profile (per image column, the near
          percentile of the depth of WALL-labelled pixels -- structure; YOLO
          object pixels are masked out of it upstream, so another object is
          NOT an occluder there), and, when enabled, the LiDAR verdict on the
          same object (the scan origin is within ~7 cm of the left lens, 3 cm
          lower, so a closer return on that bearing blocks the camera too;
          this is what catches a person standing in front of another).
  in_fov  visible to a sensor that can DETECT that class. Every class in this
          layer comes from the YOLO pipeline, so it is the stereo verdict;
          a class in LIDAR_ONLY_CLASSES (none today: no LiDAR object source
          feeds this layer) would use the LiDAR verdict instead.

LIFECYCLE (step_memory), per confirmed track and memory cycle:
  detected since the last cycle      -> TRACKED, confidence 1, timers reset.
  not detected, in FOV                -> fov_unseen_s grows; past
                                         fov_miss_remove_s it is REMOVED (we
                                         looked, it is not there).
  not detected, not in FOV            -> MEMORY: kept, confidence falls
     (out of view or occluded)           linearly to 0 over memory_decay_s,
                                         then REMOVED.
  MEMORY and back in FOV, undetected  -> still MEMORY, decay continues, and
                                         the fov_unseen_s rule applies.
  MEMORY and detected again           -> TRACKED; the id is kept when the
                                         tracker associates the detection
                                         with this track (memory zeroes its
                                         velocity, so it is matched at its
                                         stored position, with the gate
                                         widened by the person growth).
fov_unseen_s counts every in-view moment since the last detection, not only
the latest stretch: an object at the edge of the view that flickers in and
out cannot reset its own clock that way.

GROWTH: a person in MEMORY may be walking, so its effective radius grows by
person_growth_mps per second in memory, the GROWTH capped at
person_growth_max_m. Every other class is assumed static: no growth.

FRAMES: objects are stored in the MAP frame and nothing here re-projects
them. A map -> odom correction moves the car's map pose, which changes what
the sensors can see, never where a remembered object is. Odom-frame consumers
re-express positions through the current map -> odom
(f1tenth_params.semantic_objects).
"""

from dataclasses import dataclass
import math
from typing import Optional

from f1tenth_params.object_geometry import nominal_footprint_radius

# Mirrors f1tenth_messages/SemanticObject.msg (TRACKED, MEMORY) and
# f1tenth_params.semantic_objects; test_object_memory_node pins all three.
TRACKED = 0
MEMORY = 1

#: The allowed memory_decay_s range [s] (the task's 10..20, default 15).
MEMORY_DECAY_RANGE_S = (10.0, 20.0)

#: Classes whose effective radius grows while in MEMORY. People walk; every
#: other class this layer sees (chairs, bottles, tvs ...) is treated as static.
GROWTH_CLASSES = frozenset({'person'})

#: Classes only the LiDAR can detect. Empty: every object in this layer comes
#: from the camera YOLO pipeline, so its in_fov is the stereo verdict.
LIDAR_ONLY_CLASSES = frozenset()

#: A memory cycle never advances the timers by more than this [s]. Guards a
#: stalled executor or a jumping sim clock from expiring everything at once.
MAX_STEP_DT_S = 1.0

#: Fraction of the beams / image columns spanning an object that must read
#: closer than its near surface for it to count as occluded. The rule, not a
#: knob: half the object hidden is where a detector stops being reliable.
OCCLUDED_FRACTION = 0.5

# Visibility verdicts. VISIBLE is the only one that means "in view".
VIS_VISIBLE = 'visible'
VIS_NO_DATA = 'no_data'                 # sensor silent/stale, no pose, no transform
VIS_OUT_OF_ANGLE = 'out_of_angle'       # outside the scan's sector / the frustum
VIS_OUT_OF_RANGE = 'out_of_range'       # outside the range / depth window
VIS_OCCLUDED = 'occluded'               # LiDAR: closer returns on the bearing
VIS_OCCLUDED_DEPTH = 'occluded_depth'   # stereo: closer WALL depth in the columns
VIS_OCCLUDED_LIDAR = 'occluded_lidar'   # stereo: the LiDAR says it is occluded


@dataclass(frozen=True)
class MemoryParams:
    """Lifecycle parameters (stack_params.yaml semantic_layer_*)."""

    fov_miss_remove_s: float = 1.5
    memory_decay_s: float = 15.0
    person_growth_mps: float = 0.2
    person_growth_max_m: float = 0.6

    def __post_init__(self):
        """Reject values the lifecycle cannot mean anything with."""
        lo, hi = MEMORY_DECAY_RANGE_S
        if not lo <= float(self.memory_decay_s) <= hi:
            raise ValueError(
                f'memory_decay_s must be within [{lo}, {hi}] s, got {self.memory_decay_s}')
        if not float(self.fov_miss_remove_s) > 0.0:
            raise ValueError(f'fov_miss_remove_s must be > 0, got {self.fov_miss_remove_s}')
        if float(self.person_growth_mps) < 0.0 or float(self.person_growth_max_m) < 0.0:
            raise ValueError('person growth rate and cap must be >= 0')


@dataclass(frozen=True)
class VisibilityParams:
    """Sensor-view parameters (stack_params.yaml semantic_layer_*)."""

    stereo_hfov_rad: float = math.radians(101.0)
    stereo_depth_min_m: float = 0.8
    stereo_depth_max_m: float = 5.0
    occlusion_margin_m: float = 0.15
    stereo_occlusion_use_lidar: bool = True


@dataclass
class ObjectMemory:
    """The memory state of one confirmed track."""

    state: int = TRACKED
    confidence: float = 1.0
    time_in_memory_s: float = 0.0
    fov_unseen_s: float = 0.0
    time_since_seen_s: float = 0.0
    in_fov: bool = False
    in_fov_lidar: bool = False
    in_fov_stereo: bool = False
    lidar_view: str = VIS_NO_DATA
    stereo_view: str = VIS_NO_DATA
    # The track's hit_count when last looked at: a different value next cycle
    # means at least one detection was associated with it in between.
    last_hit_count: int = 0

    def remaining_s(self, params: MemoryParams) -> float:
        """Return the seconds of memory left: memory_decay_s while TRACKED."""
        return max(0.0, float(params.memory_decay_s) - self.time_in_memory_s)


@dataclass(frozen=True)
class Visibility:
    """One object's verdict per sensor (one of the VIS_* strings each)."""

    lidar: str = VIS_NO_DATA
    stereo: str = VIS_NO_DATA

    @property
    def in_fov_lidar(self) -> bool:
        """Return True when the LiDAR could see the object."""
        return self.lidar == VIS_VISIBLE

    @property
    def in_fov_stereo(self) -> bool:
        """Return True when the camera could see the object."""
        return self.stereo == VIS_VISIBLE


# ---------------------------------------------------------------------------
# lifecycle

def step_memory(mem: ObjectMemory, dt: float, detected: bool, in_fov: bool,
                params: MemoryParams) -> bool:
    """Advance one track's memory by dt seconds; False means remove it now."""
    dt = min(max(float(dt), 0.0), MAX_STEP_DT_S)
    mem.in_fov = bool(in_fov)
    if detected:
        mem.state = TRACKED
        mem.confidence = 1.0
        mem.time_in_memory_s = 0.0
        mem.fov_unseen_s = 0.0
        mem.time_since_seen_s = 0.0
        return True
    mem.time_since_seen_s += dt
    if in_fov:
        mem.fov_unseen_s += dt
    else:
        mem.state = MEMORY
    if mem.state == MEMORY:
        mem.time_in_memory_s += dt
        mem.confidence = max(0.0, 1.0 - mem.time_in_memory_s / float(params.memory_decay_s))
    if mem.fov_unseen_s > float(params.fov_miss_remove_s):
        return False
    return not (mem.state == MEMORY and mem.time_in_memory_s >= float(params.memory_decay_s))


def memory_growth_m(class_id: str, mem: ObjectMemory, params: MemoryParams) -> float:
    """Return the radius growth [m]: people in MEMORY only, capped."""
    if mem.state != MEMORY or class_id not in GROWTH_CLASSES:
        return 0.0
    return min(float(params.person_growth_mps) * mem.time_in_memory_s,
               float(params.person_growth_max_m))


def track_base_radius(trk) -> float:
    """Return the footprint radius [m]: fused width / 2, else the class nominal."""
    width = getattr(trk, 'width', None)
    if width is not None and width > 0.0:
        return 0.5 * float(width)
    return nominal_footprint_radius(trk.class_id)


def effective_radius(trk, mem: ObjectMemory, params: MemoryParams) -> float:
    """Return the base radius plus the memory growth [m]."""
    return track_base_radius(trk) + memory_growth_m(trk.class_id, mem, params)


def in_fov_for_class(class_id: str, vis: Visibility,
                     lidar_only_classes=LIDAR_ONLY_CLASSES) -> bool:
    """Return True when a sensor able to detect class_id could see the object."""
    if class_id in lidar_only_classes:
        return vis.in_fov_lidar
    return vis.in_fov_stereo


def apply_memory_to_track(trk, mem: ObjectMemory, params: MemoryParams):
    """
    Tell the tracker how to re-acquire a remembered object.

    In MEMORY the velocity estimate is stale (nobody has seen the object for
    a while), so it is zeroed: the tracker then matches a new detection
    against the stored position, not against an extrapolation of an old
    EMA. The gate is widened by the growth, so a person who walked up to
    that far while unseen still keeps their id.
    """
    if mem.state == MEMORY:
        trk.vx_map = 0.0
        trk.vy_map = 0.0
    trk.extra_gate_m = memory_growth_m(trk.class_id, mem, params)


def memory_cycle(tracks: list, memories: dict, dt: float, visibility_of,
                 params: MemoryParams, enabled: bool = True,
                 lidar_only_classes=LIDAR_ONLY_CLASSES) -> list:
    """
    Run one memory cycle over the tracker's list; return the tracks to keep.

    tracks: the tracker's tracks (semantic_layer.SemanticObject or anything
        with track_id, class_id, confirmed, hit_count, width, x_map, y_map,
        vx_map, vy_map, extra_gate_m). Tentative tracks pass through untouched
        -- the tracker's lost_miss_count still owns them.
    memories: {track_id: ObjectMemory}, owned by the caller and updated in
        place (entries are created on confirmation and dropped with the track).
    visibility_of: callable(track) -> Visibility, the caller's sensor wiring.
    enabled: False reports visibility and timers but never moves a track to
        MEMORY and never removes one (the tracker prunes, as before).
    """
    kept = []
    alive_ids = set()
    for trk in tracks:
        if not trk.confirmed:
            kept.append(trk)
            continue
        mem = memories.get(trk.track_id)
        if mem is None:
            # First cycle since confirmation, which takes a hit by definition.
            mem = memories[trk.track_id] = ObjectMemory(last_hit_count=trk.hit_count)
            detected = True
        else:
            detected = trk.hit_count != mem.last_hit_count
            mem.last_hit_count = trk.hit_count
        vis = visibility_of(trk)
        mem.lidar_view, mem.stereo_view = vis.lidar, vis.stereo
        mem.in_fov_lidar, mem.in_fov_stereo = vis.in_fov_lidar, vis.in_fov_stereo
        in_fov = in_fov_for_class(trk.class_id, vis, lidar_only_classes)
        if enabled:
            alive = step_memory(mem, dt, detected, in_fov, params)
        else:
            mem.in_fov = in_fov
            mem.time_since_seen_s = 0.0 if detected else (
                mem.time_since_seen_s + min(max(float(dt), 0.0), MAX_STEP_DT_S))
            alive = True
        if not alive:
            del memories[trk.track_id]
            continue
        apply_memory_to_track(trk, mem, params)
        kept.append(trk)
        alive_ids.add(trk.track_id)
    for stale in [k for k in memories if k not in alive_ids]:
        del memories[stale]
    return kept


# ---------------------------------------------------------------------------
# geometry

def map_to_base(x_map: float, y_map: float, robot_pose_map) -> tuple:
    """
    Express a map point in base_link, given the robot's (x, y, yaw) in map.

    The exact inverse of semantic_layer.compose_base_link_to_map.
    """
    x_r, y_r, yaw = robot_pose_map
    dx, dy = float(x_map) - x_r, float(y_map) - y_r
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def rotate(q, v) -> tuple:
    """Rotate vector v = (x, y, z) by the unit quaternion q = (x, y, z, w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # v' = v + 2 w (u x v) + 2 u x (u x v), u = (qx, qy, qz)
    cx = qy * vz - qz * vy
    cy = qz * vx - qx * vz
    cz = qx * vy - qy * vx
    ccx = qy * cz - qz * cy
    ccy = qz * cx - qx * cz
    ccz = qx * cy - qy * cx
    return (vx + 2.0 * (qw * cx + ccx),
            vy + 2.0 * (qw * cy + ccy),
            vz + 2.0 * (qw * cz + ccz))


def apply_transform(tf, p) -> tuple:
    """
    Apply tf = ((tx, ty, tz), (qx, qy, qz, qw)) to point p: R p + t.

    tf is what tf2's lookup_transform(target, source) returns, so the result
    is p (given in source) expressed in target.
    """
    t, q = tf
    rx, ry, rz = rotate(q, p)
    return rx + t[0], ry + t[1], rz + t[2]


def source_origin_height(tf) -> float:
    """
    Return the z of the TARGET frame's origin in the SOURCE frame.

    For lookup_transform(sensor, base_link) that is the sensor's height above
    base_link: a point at that height lies in the sensor's own horizontal
    plane, which is where a planar scan looks.
    """
    t, (qx, qy, qz, qw) = tf
    # origin_target in source = -R^T t
    return -rotate((-qx, -qy, -qz, qw), t)[2]


# ---------------------------------------------------------------------------
# visibility

def _span_indices(centre: float, half: float, lo_value: float, step: float, count: int):
    """Return the indices [i0, i1] of a uniform grid covering centre +- half, clamped."""
    i0 = int(math.ceil((centre - half - lo_value) / step - 1e-9))
    i1 = int(math.floor((centre + half - lo_value) / step + 1e-9))
    i0, i1 = max(i0, 0), min(i1, count - 1)
    if i0 > i1:
        nearest = int(round((centre - lo_value) / step))
        if not 0 <= nearest < count:
            return None
        i0 = i1 = nearest
    return i0, i1


def _occluded(values, threshold: float, valid_min: float, valid_max: float) -> bool:
    """Return True when OCCLUDED_FRACTION of the values read closer than threshold."""
    values = list(values)
    if not values:
        return False
    blocked = sum(
        1 for v in values
        if math.isfinite(v) and valid_min <= v <= valid_max and v < threshold)
    return blocked >= OCCLUDED_FRACTION * len(values)


def lidar_visibility(p_laser, radius: float, scan, occlusion_margin_m: float) -> str:
    """
    Return one VIS_* verdict for an object at p_laser = (x, y) in the scan frame.

    scan: a sensor_msgs/LaserScan or anything with its angle_min,
    angle_increment, angle_max, range_min, range_max and ranges. Its OWN
    limits define the field of view; nothing about the device is assumed.
    """
    if scan is None or p_laser is None:
        return VIS_NO_DATA
    ranges = scan.ranges
    count = len(ranges)
    inc = float(scan.angle_increment)
    if count == 0 or not inc > 0.0:
        return VIS_NO_DATA
    x, y = float(p_laser[0]), float(p_laser[1])
    d = math.hypot(x, y)
    near = max(d - float(radius), 0.0)
    if not float(scan.range_min) <= near <= float(scan.range_max):
        return VIS_OUT_OF_RANGE
    bearing = math.atan2(y, x)
    a_min = float(scan.angle_min)
    # The beams actually present, if the message claims more sector than it has.
    a_max = min(float(scan.angle_max), a_min + (count - 1) * inc)
    if not a_min <= bearing <= a_max:
        return VIS_OUT_OF_ANGLE
    half = math.asin(min(1.0, float(radius) / d)) if d > 0.0 else math.pi
    span = _span_indices(bearing, half, a_min, inc, count)
    if span is None:
        return VIS_OUT_OF_ANGLE
    i0, i1 = span
    threshold = near - float(occlusion_margin_m)
    if _occluded((float(r) for r in ranges[i0:i1 + 1]), threshold,
                 float(scan.range_min), float(scan.range_max)):
        return VIS_OCCLUDED
    return VIS_VISIBLE


def stereo_frustum(p_cam, radius: float, vp: VisibilityParams) -> str:
    """Return one VIS_* verdict for p_cam = (x forward, y left) in the camera body frame."""
    if p_cam is None:
        return VIS_NO_DATA
    depth, lateral = float(p_cam[0]), float(p_cam[1])
    if depth <= 0.0:
        return VIS_OUT_OF_ANGLE           # behind the lens
    if not float(vp.stereo_depth_min_m) <= depth <= float(vp.stereo_depth_max_m):
        return VIS_OUT_OF_RANGE
    # The WHOLE footprint inside the image: only then is a missed detection
    # evidence that the object is gone, rather than that it was cut in half
    # by the image edge.
    if abs(math.atan2(lateral, depth)) + math.atan2(float(radius), depth) \
            > 0.5 * float(vp.stereo_hfov_rad):
        return VIS_OUT_OF_ANGLE
    return VIS_VISIBLE


def columns_occluded(p_opt, radius: float, columns, occlusion_margin_m: float) -> Optional[bool]:
    """
    Return whether StereoColumns depth hides an object; None when it cannot say.

    p_opt: the object centre in the columns' OPTICAL frame (X right, Y down,
    Z forward). columns: f1tenth_messages/StereoColumns or a stand-in with
    width, fx, cx and distance_m (optical Z per column, NaN where empty).
    """
    if columns is None or p_opt is None:
        return None
    x_o, z_o = float(p_opt[0]), float(p_opt[2])
    width = int(columns.width)
    fx = float(columns.fx)
    if z_o <= 0.0 or width <= 0 or not fx > 0.0 or len(columns.distance_m) < width:
        return None
    u_centre = float(columns.cx) + fx * x_o / z_o
    span = _span_indices(u_centre, fx * float(radius) / z_o, 0.0, 1.0, width)
    if span is None:
        return None
    i0, i1 = span
    threshold = z_o - float(radius) - float(occlusion_margin_m)
    return _occluded((float(v) for v in columns.distance_m[i0:i1 + 1]),
                     threshold, 0.0, math.inf)


def object_visibility(radius: float, p_laser, scan, p_cam, camera_live: bool,
                      p_opt, columns, vp: VisibilityParams,
                      view_radius: Optional[float] = None) -> Visibility:
    """
    Combine both sensors' verdicts for one object (module docstring).

    radius is the footprint (what can occlude or be occluded); view_radius,
    default radius, is what must fit inside the frustum -- a remembered person
    passes radius + growth, so "we looked" means we looked at every place
    they could have walked to.
    """
    lidar = lidar_visibility(p_laser, radius, scan, vp.occlusion_margin_m)
    if not camera_live:
        return Visibility(lidar=lidar, stereo=VIS_NO_DATA)
    stereo = stereo_frustum(p_cam, radius if view_radius is None else view_radius, vp)
    if stereo == VIS_VISIBLE:
        if columns_occluded(p_opt, radius, columns, vp.occlusion_margin_m):
            stereo = VIS_OCCLUDED_DEPTH
        elif vp.stereo_occlusion_use_lidar and lidar == VIS_OCCLUDED:
            stereo = VIS_OCCLUDED_LIDAR
    return Visibility(lidar=lidar, stereo=stereo)
