"""multi_wall_tracker.py: up to four tracked walls in the odom frame, from /scan.

Pure: numpy, scipy and corridor_perception, no rclpy. lidar_front_wall_node
hosts it behind use_wall_tracker. That switch needs enable_scene_labels,
because the virtual wall it feeds exists only with scene labels.

PER SCAN
  1. Object cut. Each /camera/detections_3d position is carried from its own
     header stamp to the scan stamp on /odometry/filtered
     (propagate_objects), and its bearing sector is cut out of the scan: its
     angular half-width plus detection_margin_deg, seen from the laser
     (scene_labels.detection_sectors). Cut beams become no-returns, so a person
     splits a wall instead of bending it. Detections older than
     max_detection_age_s (1.0 s) are dropped.
  2. Extraction (extract_lines): corridor_perception.extraction with its own
     defaults: adaptive breakpoints, IEPF split 0.045 m, TLS with a
     (rho, alpha) covariance, collinear merge 6 deg / 0.05 m, and segments of at
     least 10 points and 0.5 m. Lines are found in the laser frame, then moved
     to base_link and to odom at the scan stamp.
  3. Span gate (min_span_update_m / min_span_birth_m). An observation shorter
     than min_span_update_m is not a wall at all and is dropped before the
     tracker sees it. One between that and min_span_birth_m may only UPDATE a
     track that is already confirmed: it can neither start a track nor feed a
     tentative one. See WHY THE SPAN GATE IS HYSTERETIC below.
  4. Tracking (MultiWallTracker.step) on (rho, theta) in odom, one Kalman
     filter per track:
     - Static model with process noise q per second on both states, plus
       noise proportional to the odometry yaw change since the last scan
       (k_yaw), for the odometry's yaw scale error.
     - Association: Hungarian assignment of Mahalanobis distances, gated at
       chi2(2 DOF, 99%) = 9.21, and the segments must overlap by
       min_overlap_m along the line.
     - Lifecycle: TENTATIVE becomes CONFIRMED at 3 hits in the last 5 scans.
       A confirmed track that goes unseen is COASTING while its covariance
       grows. A track is deleted after delete_after_s unseen, or when
       sigma_rho > max_sigma_rho_m, or (if still TENTATIVE) when 5 scans hold
       fewer than 3 hits.
     - Two tracks closer than the gate are merged into the older one.
     - Published: at most max_published (4) tracks that were ever confirmed,
       ranked by length * hits / distance.
     - Label: front, left, right or back, from the car-to-wall normal against
       the heading (+-45 deg sectors). Ids are never reused.
     - Front track (front_track): the published track labelled front that the
       heading ray from the bumper meets nearest, if it was matched on this
       scan or has been coasting for at most front_max_coast_s (0.3 s). A
       front track that coasts longer stops being the front wall
       (coast_expired) although it lives on until delete_after_s.

WHY THE SPAN GATE IS HYSTERETIC
------------------------------
A wall is long. An object face, a chair leg, the end of a counter, a doorframe
reveal and a person's two legs are all short, and the extraction cannot tell
them apart from a wall fragment: it reports geometry, not semantics. The
single-threshold version of this rule has to choose between two failures --
set it high and a wall seen edge-on or mostly occluded stops being tracked
every time it shortens; set it low and every short face in the room starts a
wall track, which is how a 1.39 m "front" wall sat 2 m ahead of the car for 30
s of an empty corridor (corridor10m_20261015T171753, track 8).

Birth and maintenance are different questions, so they get different
thresholds. STARTING a wall is a claim about the world and needs real extent:
min_span_birth_m. KEEPING one is a claim about a wall that has already proven
itself over confirm_hits scans, and a confirmed wall briefly seen short --
someone walks past it, the car turns and grazes it, half of it leaves the
sector -- is still that wall: min_span_update_m. The asymmetry is the same
shape as HysteresisLatch's in front_clearance_node and the role hysteresis
below, and for the same reason: the cost of being wrong is not symmetric.

Note the gate is on the OBSERVATION's span, not the track's. A track's extent
only ever grows (`_update` takes min/max), so gating on the track would be a
one-way door.

THE COLLINEAR MERGE COMES FIRST, and it already exists: step 2's extraction
reunifies fragments of one wall across a gap of up to merge_max_gap_m with the
same 6 deg / 5 cm tolerances it uses everywhere (corridor_perception.
extraction._merge_clusters), globally rather than only between scan-adjacent
groups, which is what survives an occluder standing in front of a wall. So a
span is measured on the whole wall, not on whichever piece of it a person left
visible -- which has to be true before any span threshold means anything.

FRAMES. A line is rho = x cos(theta) + y sin(theta) with rho >= 0
(corridor_perception.geometry.Line), the convention scene_labels.WallLine
uses in base_link as n . (x, y) = d.
"""

import collections
import itertools
import math
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment

from corridor_perception.extraction import ExtractionParams, extract_segments
from corridor_perception.geometry import Line, Pose2D, normalise_line, wrap_pi
from corridor_perception.scan import clean_ranges
from f1tenth_perception.scene_labels import WallLine, in_sectors
from f1tenth_perception import wall_fusion

CHI2_2DOF_99 = 9.21

STATE_TENTATIVE = 0
STATE_CONFIRMED = 1
STATE_COASTING = 2
STATE_NAMES = {STATE_TENTATIVE: 'tentative', STATE_CONFIRMED: 'confirmed',
               STATE_COASTING: 'coasting'}


# ============================================================================
# Poses and object propagation
# ============================================================================

class PoseHistory:
    """Recent (stamp, x, y, yaw) odometry poses, interpolated at any stamp inside them."""

    def __init__(self, max_age_s=2.0):
        self.max_age_s = float(max_age_s)
        self._poses = collections.deque()

    def add(self, stamp, x, y, yaw):
        stamp = float(stamp)
        if self._poses and stamp <= self._poses[-1][0]:
            return
        self._poses.append((stamp, float(x), float(y), float(yaw)))
        while self._poses and stamp - self._poses[0][0] > self.max_age_s:
            self._poses.popleft()

    def at(self, stamp, max_extrapolation_s=0.1):
        """Pose2D (odom from base_link) at stamp, or None outside the history.

        Linear between the two neighbouring poses (yaw through its wrapped
        difference). A stamp beyond either end by at most
        max_extrapolation_s gets the end pose.
        """
        if not self._poses:
            return None
        stamp = float(stamp)
        first, last = self._poses[0], self._poses[-1]
        if stamp <= first[0]:
            return Pose2D(*first[1:]) if first[0] - stamp <= max_extrapolation_s else None
        if stamp >= last[0]:
            return Pose2D(*last[1:]) if stamp - last[0] <= max_extrapolation_s else None
        for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(self._poses, itertools.islice(
                self._poses, 1, None)):
            if t0 <= stamp <= t1:
                f = (stamp - t0) / (t1 - t0) if t1 > t0 else 0.0
                return Pose2D(x0 + f * (x1 - x0), y0 + f * (y1 - y0),
                              wrap_pi(a0 + f * wrap_pi(a1 - a0)))
        return None


def propagate_objects(objects_xyr, pose_then, pose_now):
    """(x, y, half_width) in base_link at one stamp, moved into base_link at another.

    pose_then and pose_now are odom from base_link at the detection stamp and
    at the scan stamp. The objects are assumed static in odom.
    """
    if not objects_xyr:
        return []
    pts = np.array([[x, y] for x, y, _ in objects_xyr], dtype=float)
    now = pose_now.inverse().transform_points(pose_then.transform_points(pts))
    return [(float(p[0]), float(p[1]), float(r)) for p, (_, _, r) in zip(now, objects_xyr)]


# ============================================================================
# Extraction
# ============================================================================

def extraction_params(merge_max_gap_m=None, quality_gates=None):
    """The extraction's own defaults, with the merge gap and the gates overridden.

    The merge is what makes a span mean "the length of this wall" rather than
    "the length of the piece of it nothing was standing in front of", so the
    one tolerance a wall-span threshold has to be set together with is this
    gap. `quality_gates` is corridor_perception's seed/rms/fill gates (see
    extraction.py, WHY THE QUALITY GATES); off, the extraction reports every
    collinear triple it can find, which is what the span gate below then has
    to clean up on its own. Everything else stays corridor_perception's own
    default.
    """
    p = ExtractionParams()
    if merge_max_gap_m is not None:
        p.merge_gap = float(merge_max_gap_m)
    if quality_gates is not None:
        p.quality_gates = bool(quality_gates)
    return p


def extract_lines(ranges, angle_min, angle_increment, range_min, range_max, laser_pose,
                  sectors=(), params=None, stats=None):
    """Wall segments of one scan as corridor_perception Lines in base_link.

    laser_pose: (x, y, yaw) of the laser in base_link. sectors: bearing
    sectors [(lo, hi)] in base_link (scene_labels.detection_sectors) whose
    beams are treated as no-returns. params: ExtractionParams, normally built
    by `extraction_params` from the node's merge-gap and quality-gate
    parameters. stats: filled with what each extraction gate dropped.
    """
    r = clean_ranges(ranges, range_min, range_max)
    lx, ly, lyaw = laser_pose
    angles = angle_min + np.arange(r.size) * angle_increment
    if sectors:
        bearings = wrap_pi(angles + lyaw)
        cut = in_sectors(bearings, sectors)
        if cut.any():
            r = np.where(cut, np.inf, r).view(type(r))
    lines = extract_segments(r, angles, params or ExtractionParams(), stats=stats)
    laser = Pose2D(float(lx), float(ly), float(lyaw))
    return [line.transform(laser, 'base_link') for line in lines]


# ============================================================================
# Tracker
# ============================================================================

@dataclass
class TrackerParams:
    process_noise_rho: float = 1e-4        # [m^2 / s]
    process_noise_theta: float = 1e-4      # [rad^2 / s]
    # Odometry yaw scale error (/odometry/filtered's yaw gain is 0.93-1.09):
    # each scan adds (k_yaw |dyaw|)^2 to theta and (k_yaw |dyaw| rho)^2 to rho.
    k_yaw: float = 0.1
    meas_sigma_rho_min: float = 0.02       # [m] floor on the TLS sigma (odometry error)
    meas_sigma_theta_min: float = math.radians(0.5)
    gate_chi2: float = CHI2_2DOF_99
    min_overlap_m: float = 0.2
    confirm_hits: int = 3
    confirm_window: int = 5
    delete_after_s: float = 3.0
    max_sigma_rho_m: float = 0.3
    max_published: int = 4
    label_half_sector_rad: float = math.radians(45.0)
    role_hysteresis_rad: float = math.radians(5.0)   # a role sticks this far past its edge
    ray_margin_m: float = 0.3               # extent slack for d_ray_front
    front_max_coast_s: float = 0.3          # a coasting front track's last say
    # --- measurement noise, by SOURCE and RANGE (observation_covariance) ---
    lidar_sigma_base_m: float = 0.02
    lidar_sigma_per_m: float = 0.005
    stereo_sigma_base_m: float = 0.03
    stereo_sigma_quad_per_m2: float = 0.012
    min_span_for_angle_m: float = 0.3
    # --- lifecycle: stereo-only evidence is weaker, so it needs more of it --
    stereo_confirm_hits: int = 5
    stereo_confirm_window: int = 7
    # --- minimum width for a wall (see WHY THE SPAN GATE IS HYSTERETIC) ---
    min_span_birth_m: float = 1.0           # to START a track
    min_span_update_m: float = 0.5          # to keep a CONFIRMED one
    merge_max_gap_m: float = 1.5            # extraction's collinear merge gap
    # --- what may be called FRONT -----------------------------------------
    strip_half_width_m: float = 0.186       # vehicle_frames.front_side_limit_m()
    side_edge_half_width_m: float = 0.136   # swept_clearance_body_half_width_m
    stereo_front_max_normal_rad: float = math.radians(30.0)


def observation_covariance(source, distance_m, span_m, p, stereo_weight=1.0):
    """2x2 (rho, theta) measurement covariance of an observation.

    An observation carries no covariance of its own -- it is a line, not a
    fit with a Jacobian -- so the tracker synthesises one from the two things
    that actually determine the error: WHICH SENSOR saw it and HOW FAR AWAY.

      LiDAR   sigma_rho = base + per_m * range. Nearly range-independent:
              the Hokuyo's own range error is millimetres and what grows is
              the beam footprint and the odometry the line is placed with.
      stereo  sigma_rho = base + quad * range^2, the same quadratic the
              cross-check's tolerance uses and for the same reason
              (sigma_Z = Z^2 sigma_d / (fx b)). At 6 m this is 46 cm against
              the LiDAR's 5 cm, which is the honest ratio between them.
      both    the inverse-variance combination of the two, so a wall both
              sensors see is tighter than either alone -- which is the whole
              point of having agreed.

    sigma_theta is sigma_rho over the observed span: a 5 cm uncertainty on a
    3 m wall is a much better-determined angle than the same 5 cm on a 40 cm
    stub, and a short segment's angle is exactly what should not be trusted.

    `stereo_weight` (wall_fusion.age_weight) is how much the stereo half was
    worth. It divides the stereo sigma, which is the only honest way to spend
    it here: the inverse-variance combination then leans on the LiDAR in
    exactly the proportion the weight says, and at the floor weight 0.3 the
    stereo sigma is 3.3x its fresh value, so a 0.6 s profile moves the fused
    distance by centimetres instead of decimetres. A weight of 0 is LiDAR
    only, which is also what the caller does by not pairing at all.
    """
    d = max(float(distance_m), 0.0)
    w = min(1.0, max(0.0, float(stereo_weight)))
    lidar = p.lidar_sigma_base_m + p.lidar_sigma_per_m * d
    stereo = p.stereo_sigma_base_m + p.stereo_sigma_quad_per_m2 * d * d
    stereo = stereo / w if w > 0.0 else math.inf
    if source == wall_fusion.SOURCE_STEREO:
        sigma_rho = stereo
    elif source == wall_fusion.SOURCE_BOTH and math.isfinite(stereo):
        sigma_rho = 1.0 / math.sqrt(1.0 / lidar ** 2 + 1.0 / stereo ** 2)
    else:
        sigma_rho = lidar
    span = max(float(span_m), p.min_span_for_angle_m)
    sigma_theta = max(sigma_rho / span, p.meas_sigma_theta_min)
    return np.diag([max(sigma_rho, p.meas_sigma_rho_min) ** 2, sigma_theta ** 2])


def line_to_wall_line(line, source='lidar'):
    """A corridor_perception Line (base_link) as a scene_labels.WallLine.

    The extraction's own (rho, alpha) covariance is the honest stand-in for a
    residual rms: sqrt(cov[0, 0]) is how well that segment's points pinned its
    distance, which is exactly what the confidence model wants to know.
    """
    t = _along(line.alpha, np.vstack([line.p_start, line.p_end]))
    cov = np.asarray(line.cov, dtype=float).reshape(2, 2)
    return WallLine(math.cos(line.alpha), math.sin(line.alpha), float(line.rho),
                    float(t.min()), float(t.max()), int(line.n_points),
                    float(math.sqrt(max(cov[0, 0], 0.0))), source)


def observation_line(obs, car_pose, p):
    """A wall_fusion.Observation (base_link) as a corridor_perception Line in odom."""
    wl = obs.line
    rho, alpha = normalise_line(float(wl.d), math.atan2(wl.ny, wl.nx))
    (ax, ay), (bx, by) = wl.endpoints()
    line = Line(rho, alpha,
                observation_covariance(obs.source, wl.d, wl.span, p,
                                       getattr(obs, 'stereo_weight', 1.0)),
                np.array([ax, ay], dtype=float), np.array([bx, by], dtype=float),
                max(int(wl.inliers), 1), 'base_link')
    return line.transform(car_pose, 'odom')


@dataclass
class Track:
    id: int
    x: np.ndarray                          # (rho, theta), odom
    P: np.ndarray
    t0: float                              # extent along the tangent (-sin, cos)
    t1: float
    born: float
    last_seen: float
    hits: int = 1
    history: collections.deque = field(default_factory=lambda: collections.deque(maxlen=5))
    ever_confirmed: bool = False
    state: int = STATE_TENTATIVE
    # What the evidence behind this track has been (step 4).
    source: int = wall_fusion.SOURCE_LIDAR
    has_lidar: bool = False     # any LIDAR or BOTH hit, ever
    has_both: bool = False      # any BOTH hit, ever
    z_min: float = 0.0
    z_max: float = 2.5
    height_measured: bool = False
    confidence: float = 0.0
    role: str = ''              # sticky: see MultiWallTracker.update_roles

    @property
    def rho(self):
        return float(self.x[0])

    @property
    def theta(self):
        return float(self.x[1])

    @property
    def length(self):
        return self.t1 - self.t0

    def endpoints(self):
        """((x0, y0), (x1, y1)) of the extent, odom."""
        n = np.array([math.cos(self.theta), math.sin(self.theta)])
        tan = np.array([-n[1], n[0]])
        base = n * self.rho
        return base + tan * self.t0, base + tan * self.t1


def _along(theta, pts):
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    return -math.sin(theta) * pts[:, 0] + math.cos(theta) * pts[:, 1]


def _innovation(track_x, rho, alpha):
    """(rho, theta) innovation of a measurement, in whichever sign of the line is closer."""
    best = None
    for r, a in ((rho, alpha), (-rho, alpha + math.pi)):
        v = np.array([r - track_x[0], wrap_pi(a - track_x[1])])
        if best is None or abs(v[1]) < abs(best[1]):
            best = v
    return best


def line_label(theta, rho, car_pose, half_sector_rad=math.radians(45.0)):
    """Label a wall front / left / right / back by its normal as seen from the car."""
    n = np.array([math.cos(theta), math.sin(theta)])
    to_wall = theta if float(n @ car_pose.t) <= rho else theta + math.pi
    b = wrap_pi(to_wall - car_pose.theta)
    if abs(b) <= half_sector_rad:
        return 'front'
    if abs(b) >= math.pi - half_sector_rad:
        return 'back'
    return 'left' if b > 0.0 else 'right'


def role_with_hysteresis(theta, rho, car_pose, current, half_sector_rad, hysteresis_rad):
    """The wall's role, keeping `current` until the bearing is past its edge.

    Without this a wall sitting on a sector boundary flips role every scan as
    the car wobbles, and so does every consumer keyed on the role.
    """
    fresh = line_label(theta, rho, car_pose, half_sector_rad)
    if not current or current == fresh or hysteresis_rad <= 0.0:
        return fresh
    # Still `current` if it would survive a sector widened by the hysteresis.
    widened = line_label(theta, rho, car_pose, half_sector_rad + hysteresis_rad)
    if widened == current:
        return current
    narrowed = line_label(theta, rho, car_pose, max(
        0.0, half_sector_rad - hysteresis_rad))
    return current if narrowed == current else fresh


def segment_in_base_link(track, car_pose):
    """The track's two endpoints as (2, 2) base_link coordinates."""
    p0, p1 = track.endpoints()
    return car_pose.inverse().transform_points(np.vstack([p0, p1]))


def crosses_strip(track, car_pose, front_x, half_width_m):
    """Does the segment cross the car-width strip AHEAD of the bumper?

    The strip is {x > front_x, |y| <= half_width_m} in base_link: the band the
    car sweeps going straight. The segment is clipped to |y| <= half_width_m
    and what survives must reach past the bumper.

    This is the test that separates "a wall is in the way" from "a wall's
    normal points roughly forwards". On an open corridor the side wall
    satisfies the second for most of a turn and is nowhere near the car's
    path, which is how a phantom front wall appeared at 1.1 m.
    """
    (x0, y0), (x1, y1) = segment_in_base_link(track, car_pose)
    half = abs(float(half_width_m))
    lo, hi = 0.0, 1.0
    dy = y1 - y0
    for sign in (1.0, -1.0):
        # sign * y <= half  ->  sign * (y0 + t dy) <= half
        a, b = sign * dy, half - sign * y0
        if abs(a) < 1e-12:
            if b < 0.0:
                return False
        elif a > 0.0:
            hi = min(hi, b / a)
        else:
            lo = max(lo, b / a)
    if lo > hi:
        return False
    xs = (x0 + lo * (x1 - x0), x0 + hi * (x1 - x0))
    return max(xs) > float(front_x)


def side_distance(track, car_pose, half_width_m):
    """Perpendicular distance [m] from the car's SIDE EDGE to the wall, or NaN.

    For a left/right wall, what a driver cares about is the gap from the
    bodywork, not from the centreline: the car is half_width_m wide either
    side. Negative would mean the bodywork is already through the wall, so it
    is clamped at 0.
    """
    n = np.array([math.cos(track.theta), math.sin(track.theta)])
    d_centre = abs(track.rho - float(n @ car_pose.t))
    return max(0.0, d_centre - abs(float(half_width_m)))


def front_distances(track, car_pose, front_x, margin_m=0.3):
    """(perpendicular, along-heading) distance [m] from base_front to the track.

    The ray distance is NaN when the heading ray from the bumper misses the
    segment's extent (plus margin_m) or points away from the line.
    """
    n = np.array([math.cos(track.theta), math.sin(track.theta)])
    h = np.array([math.cos(car_pose.theta), math.sin(car_pose.theta)])
    p_front = car_pose.t + front_x * h
    off = track.rho - float(n @ p_front)
    d_perp = abs(off)
    d_ray = math.nan
    nh = float(n @ h)
    if abs(nh) > 1e-6:
        s = off / nh
        if s >= 0.0:
            t = float(_along(track.theta, p_front + s * h)[0])
            if track.t0 - margin_m <= t <= track.t1 + margin_m:
                d_ray = s
    return d_perp, d_ray


class MultiWallTracker:
    """Wall tracks in odom; see the module docstring."""

    def __init__(self, params=None):
        self.p = params or TrackerParams()
        self.tracks = []
        self._next_id = 1
        self._last_stamp = None
        self._last_yaw = None
        #: Observations the span gate dropped on the last step, for the debug
        #: payload: a run of these is the signal that the room is full of short
        #: faces, or that the merge gap is too tight to reunify a wall.
        self.n_short_dropped = 0

    # ---- filter -----------------------------------------------------------

    def _meas_cov(self, line):
        cov = np.array(line.cov, dtype=float).reshape(2, 2)
        floor = np.diag([self.p.meas_sigma_rho_min ** 2, self.p.meas_sigma_theta_min ** 2])
        return cov + floor

    def _predict(self, dt, dyaw=0.0):
        q = np.diag([self.p.process_noise_rho, self.p.process_noise_theta]) * max(dt, 0.0)
        turn = self.p.k_yaw * abs(dyaw)
        for trk in self.tracks:
            trk.P = trk.P + q + np.diag([(turn * trk.rho) ** 2, turn ** 2])

    def _overlap(self, trk, line):
        m = _along(trk.theta, np.vstack([line.p_start, line.p_end]))
        return min(trk.t1, m.max()) - max(trk.t0, m.min())

    def _cost(self, trk, line, may_birth=True):
        """Mahalanobis distance squared, or inf when gated out.

        `may_birth` False means the observation is shorter than
        min_span_birth_m: it may only reach a track that is ALREADY confirmed
        (module docstring, WHY THE SPAN GATE IS HYSTERETIC). Gating it here
        rather than after the assignment matters -- a short segment that was
        allowed to compete for a tentative track would both feed that track
        and consume the assignment a long segment needed.
        """
        if not may_birth and not trk.ever_confirmed:
            return math.inf
        v = _innovation(trk.x, line.rho, line.alpha)
        S = trk.P + self._meas_cov(line)
        d2 = float(v @ np.linalg.solve(S, v))
        if d2 > self.p.gate_chi2 or self._overlap(trk, line) < self.p.min_overlap_m:
            return math.inf
        return d2

    def _absorb(self, trk, obs):
        """Carry an observation's own attributes onto the track it matched."""
        trk.confidence = float(obs.confidence)
        if obs.source == wall_fusion.SOURCE_BOTH:
            trk.has_lidar = trk.has_both = True
        elif obs.source == wall_fusion.SOURCE_LIDAR:
            trk.has_lidar = True
        # The track's source is the BEST evidence it has EVER had, in the
        # order both > lidar > stereo. A wall both sensors confirmed does not
        # become stereo-only because one later frame missed it with the
        # LiDAR, and -- the one that matters -- a wall the LiDAR has seen can
        # never decay into a stereo-only wall, which is the only kind the
        # FRONT rule treats with suspicion.
        trk.source = (wall_fusion.SOURCE_BOTH if trk.has_both
                      else wall_fusion.SOURCE_LIDAR if trk.has_lidar
                      else wall_fusion.SOURCE_STEREO)
        if obs.height_measured:
            trk.z_min, trk.z_max, trk.height_measured = obs.z_min, obs.z_max, True

    def _update(self, trk, line, stamp):
        v = _innovation(trk.x, line.rho, line.alpha)
        R = self._meas_cov(line)
        S = trk.P + R
        K = trk.P @ np.linalg.inv(S)
        trk.x = trk.x + K @ v
        trk.P = (np.eye(2) - K) @ trk.P
        rho, theta = normalise_line(float(trk.x[0]), float(trk.x[1]))
        if rho != float(trk.x[0]):
            trk.t0, trk.t1 = -trk.t1, -trk.t0
        trk.x = np.array([rho, theta])
        m = _along(trk.theta, np.vstack([line.p_start, line.p_end]))
        trk.t0, trk.t1 = min(trk.t0, float(m.min())), max(trk.t1, float(m.max()))
        trk.hits += 1
        trk.last_seen = stamp

    def _spawn(self, line, stamp, obs=None):
        m = _along(line.alpha, np.vstack([line.p_start, line.p_end]))
        trk = Track(self._next_id, np.array([line.rho, line.alpha], dtype=float),
                    self._meas_cov(line), float(m.min()), float(m.max()), stamp, stamp)
        # The window is the LONGER of the two: a track that turns out to be
        # stereo-only must not have already dropped the scans it needs.
        trk.history = collections.deque([True], maxlen=max(
            self.p.confirm_window, self.p.stereo_confirm_window))
        if obs is not None:
            self._absorb(trk, obs)
        self._next_id += 1
        self.tracks.append(trk)

    def _merge(self):
        merged = True
        while merged:
            merged = False
            for a, b in itertools.combinations(sorted(self.tracks, key=lambda t: t.id), 2):
                v = _innovation(a.x, b.rho, b.theta)
                S = a.P + b.P
                if float(v @ np.linalg.solve(S, v)) > self.p.gate_chi2:
                    continue
                info_a, info_b = np.linalg.inv(a.P), np.linalg.inv(b.P)
                P = np.linalg.inv(info_a + info_b)
                x = P @ (info_a @ a.x + info_b @ (a.x + v))
                rho, theta = normalise_line(float(x[0]), float(x[1]))
                ends = np.vstack([*a.endpoints(), *b.endpoints()])
                m = _along(theta, ends)
                a.x, a.P = np.array([rho, theta]), P
                a.t0, a.t1 = float(m.min()), float(m.max())
                a.hits += b.hits
                a.last_seen = max(a.last_seen, b.last_seen)
                a.born = min(a.born, b.born)
                a.ever_confirmed = a.ever_confirmed or b.ever_confirmed
                a.history = collections.deque(
                    [x or y for x, y in itertools.zip_longest(a.history, b.history,
                                                              fillvalue=False)],
                    maxlen=self.p.confirm_window)
                self.tracks.remove(b)
                merged = True
                break

    # ---- one scan ---------------------------------------------------------

    def step(self, observations, stamp, car_pose):
        """Update with this cycle's OBSERVATIONS (base_link). Published tracks out.

        The tracker's input is exactly what goes out on
        /perception/wall_observations -- nothing else reaches it. It no longer
        fits lines to the scan itself, so the stereo can no longer slip in as
        a fallback distance behind its back.
        """
        stamp = float(stamp)
        # The span gate, before anything persistent is built on an observation
        # (module docstring, WHY THE SPAN GATE IS HYSTERETIC). The span is the
        # observation's own extent: its inliers projected onto its line, which
        # is what WallLine.span is and what the extraction's collinear merge
        # has already reunified across occluders.
        kept, may_birth = [], []
        self.n_short_dropped = 0
        for obs in observations:
            span = float(obs.line.span)
            if span < self.p.min_span_update_m:
                self.n_short_dropped += 1
                continue
            kept.append(obs)
            may_birth.append(span >= self.p.min_span_birth_m)
        observations = kept
        lines_odom = [observation_line(o, car_pose, self.p) for o in observations]
        dt = 0.0 if self._last_stamp is None else stamp - self._last_stamp
        self._last_stamp = stamp
        last_yaw, self._last_yaw = self._last_yaw, car_pose.theta
        self._predict(dt, 0.0 if last_yaw is None else wrap_pi(car_pose.theta - last_yaw))

        used_lines, hit_ids = set(), set()
        if self.tracks and lines_odom:
            cost = np.array([[self._cost(t, ln, may_birth[j])
                              for j, ln in enumerate(lines_odom)] for t in self.tracks])
            finite = np.isfinite(cost)
            if finite.any():
                big = cost[finite].max() * 10.0 + 1e6
                rows, cols = linear_sum_assignment(np.where(finite, cost, big))
                for i, j in zip(rows, cols):
                    if finite[i, j]:
                        self._update(self.tracks[i], lines_odom[j], stamp)
                        self._absorb(self.tracks[i], observations[j])
                        used_lines.add(j)
                        hit_ids.add(self.tracks[i].id)
        for trk in self.tracks:
            hit = trk.id in hit_ids
            trk.history.append(hit)
            if self._confirmed_by_history(trk):
                trk.ever_confirmed = True
            if trk.ever_confirmed:
                trk.state = STATE_CONFIRMED if hit else STATE_COASTING
            else:
                trk.state = STATE_TENTATIVE
        for j, line in enumerate(lines_odom):
            # Only a segment with real extent may START a wall. A shorter one
            # that matched nothing is simply not evidence of a new wall: it is
            # dropped here, not held as a tentative track that a later long
            # segment could confirm.
            if j not in used_lines and may_birth[j]:
                self._spawn(line, stamp, observations[j])

        self.tracks = [t for t in self.tracks if not self._expired(t, stamp)]
        self._merge()
        self.update_roles(car_pose)
        return self.published(car_pose)

    def _confirm_rule(self, trk):
        """(hits, window) this track must meet: stereo-only evidence needs more.

        A track the LiDAR has never seen rests entirely on stereo, which on
        this building's floor invents whole surfaces out of reflections, so it
        must survive stereo_confirm_hits of stereo_confirm_window scans. A
        LiDAR or BOTH hit promotes the track to the ordinary rule IMMEDIATELY
        -- it is no longer stereo-only, so the stricter bar no longer applies
        and the standard confirm_hits-in-confirm_window decides it. (Note it
        does NOT confirm the track on the spot: one spurious line must still
        not make a wall, which is the lifecycle this keeps unchanged.)
        """
        if trk.has_lidar:
            return self.p.confirm_hits, self.p.confirm_window
        return self.p.stereo_confirm_hits, self.p.stereo_confirm_window

    def _confirmed_by_history(self, trk):
        hits, window = self._confirm_rule(trk)
        return sum(itertools.islice(reversed(trk.history), window)) >= hits

    def _expired(self, trk, stamp):
        if stamp - trk.last_seen > self.p.delete_after_s:
            return True
        if math.sqrt(max(trk.P[0, 0], 0.0)) > self.p.max_sigma_rho_m:
            return True
        hits, window = self._confirm_rule(trk)
        return (not trk.ever_confirmed and len(trk.history) >= window
                and sum(itertools.islice(reversed(trk.history), window)) < hits)

    def score(self, trk, car_pose):
        n = np.array([math.cos(trk.theta), math.sin(trk.theta)])
        dist = max(abs(trk.rho - float(n @ car_pose.t)), 0.1)
        return trk.length * trk.hits / dist

    def published(self, car_pose):
        """Return the ever-confirmed tracks, the best max_published by score."""
        live = [t for t in self.tracks if t.ever_confirmed]
        live.sort(key=lambda t: self.score(t, car_pose), reverse=True)
        return live[:self.p.max_published]

    def update_roles(self, car_pose):
        """Give every track its role, with hysteresis at the sector edges.

        A wall sitting exactly on a 45 deg boundary would otherwise flip
        between front and left every scan as the car wobbles, and every
        consumer keyed on the role would flip with it. A track keeps the role
        it has until the bearing is role_hysteresis_rad PAST the edge, so the
        boundary is 45 +- 5 deg depending on which way it is being crossed.
        """
        for trk in self.tracks:
            trk.role = role_with_hysteresis(
                trk.theta, trk.rho, car_pose, trk.role,
                self.p.label_half_sector_rad, self.p.role_hysteresis_rad)

    def label(self, trk, car_pose):
        """The track's role. Sticky once update_roles has run (step 4)."""
        return trk.role or line_label(
            trk.theta, trk.rho, car_pose, self.p.label_half_sector_rad)

    def coast_expired(self, trk):
        """True once trk has gone unseen for longer than front_max_coast_s."""
        return (self._last_stamp is not None
                and self._last_stamp - trk.last_seen > self.p.front_max_coast_s)

    def track(self, track_id):
        """The live track with this id, or None (deleted or merged away)."""
        for trk in self.tracks:
            if trk.id == track_id:
                return trk
        return None

    def is_front(self, trk, car_pose, front_x):
        """Whether this track may be called FRONT -- the one wall in the way.

        Three conditions, and the second is the one the open corridor needed:

          * CONFIRMED (ever confirmed, and not coasting past
            front_max_coast_s). A tentative track is a maybe, and a maybe
            must never stop the car.
          * its segment actually CROSSES THE CAR-WIDTH STRIP ahead of the
            bumper. Not "its normal points forwards": on an open corridor the
            side wall's normal is within 45 deg of the heading for most of a
            turn, and reading it as the front wall is exactly what put a
            phantom wall at 1.1 m (corridor_phantom, 2026-10-15). A wall is in
            the way when the car would drive into it going straight.
          * if the evidence is STEREO-ONLY, its normal must also be within
            stereo_front_max_normal_rad of the heading. A stereo-only wall is
            the one nothing cross-checks, so it may only stop the car when it
            is squarely across the path, not when it is a glancing surface.
        """
        if not trk.ever_confirmed or self.coast_expired(trk):
            return False
        if not crosses_strip(trk, car_pose, front_x, self.p.strip_half_width_m):
            return False
        if trk.source == wall_fusion.SOURCE_STEREO:
            n = np.array([math.cos(trk.theta), math.sin(trk.theta)])
            to_wall = trk.theta if float(n @ car_pose.t) <= trk.rho else trk.theta + math.pi
            if abs(wrap_pi(to_wall - car_pose.theta)) > self.p.stereo_front_max_normal_rad:
                return False
        return True

    def front_track(self, car_pose, front_x):
        """The published FRONT track the heading ray meets nearest, or None."""
        best, best_d = None, math.inf
        for trk in self.published(car_pose):
            if not self.is_front(trk, car_pose, front_x):
                continue
            _d_perp, d_ray = front_distances(trk, car_pose, front_x, self.p.ray_margin_m)
            d = d_ray if math.isfinite(d_ray) else math.inf
            if d < best_d:
                best, best_d = trk, d
        return best


def track_in_base_link(trk, car_pose):
    """Return the track as a corridor_perception Line in base_link at car_pose."""
    p0, p1 = trk.endpoints()
    line = Line(trk.rho, trk.theta, trk.P.copy(), np.asarray(p0), np.asarray(p1), trk.hits,
                'odom')
    return line.transform(car_pose.inverse(), 'base_link')


def to_wall_line(trk, car_pose, rms=0.0):
    """Return the track as a scene_labels.WallLine in base_link (the heading wall)."""
    bl = track_in_base_link(trk, car_pose)
    nx, ny = math.cos(bl.alpha), math.sin(bl.alpha)
    t = _along(bl.alpha, np.vstack([bl.p_start, bl.p_end]))
    return WallLine(nx, ny, float(bl.rho), float(t.min()), float(t.max()), int(trk.hits),
                    float(rms), 'tracker')
