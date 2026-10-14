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
  3. Tracking (MultiWallTracker.step) on (rho, theta) in odom, one Kalman
     filter per track:
     - Static model with process noise q per second on both states.
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

def extract_lines(ranges, angle_min, angle_increment, range_min, range_max, laser_pose,
                  sectors=(), params=None):
    """Wall segments of one scan as corridor_perception Lines in base_link.

    laser_pose: (x, y, yaw) of the laser in base_link. sectors: bearing
    sectors [(lo, hi)] in base_link (scene_labels.detection_sectors) whose
    beams are treated as no-returns.
    """
    r = clean_ranges(ranges, range_min, range_max)
    lx, ly, lyaw = laser_pose
    angles = angle_min + np.arange(r.size) * angle_increment
    if sectors:
        bearings = wrap_pi(angles + lyaw)
        cut = in_sectors(bearings, sectors)
        if cut.any():
            r = np.where(cut, np.inf, r).view(type(r))
    lines = extract_segments(r, angles, params or ExtractionParams())
    laser = Pose2D(float(lx), float(ly), float(lyaw))
    return [line.transform(laser, 'base_link') for line in lines]


# ============================================================================
# Tracker
# ============================================================================

@dataclass
class TrackerParams:
    process_noise_rho: float = 1e-4        # [m^2 / s]
    process_noise_theta: float = 1e-4      # [rad^2 / s]
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
    ray_margin_m: float = 0.3               # extent slack for d_ray_front


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

    # ---- filter -----------------------------------------------------------

    def _meas_cov(self, line):
        cov = np.array(line.cov, dtype=float).reshape(2, 2)
        floor = np.diag([self.p.meas_sigma_rho_min ** 2, self.p.meas_sigma_theta_min ** 2])
        return cov + floor

    def _predict(self, dt):
        q = np.diag([self.p.process_noise_rho, self.p.process_noise_theta]) * max(dt, 0.0)
        for trk in self.tracks:
            trk.P = trk.P + q

    def _overlap(self, trk, line):
        m = _along(trk.theta, np.vstack([line.p_start, line.p_end]))
        return min(trk.t1, m.max()) - max(trk.t0, m.min())

    def _cost(self, trk, line):
        """Mahalanobis distance squared, or inf when gated out."""
        v = _innovation(trk.x, line.rho, line.alpha)
        S = trk.P + self._meas_cov(line)
        d2 = float(v @ np.linalg.solve(S, v))
        if d2 > self.p.gate_chi2 or self._overlap(trk, line) < self.p.min_overlap_m:
            return math.inf
        return d2

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

    def _spawn(self, line, stamp):
        m = _along(line.alpha, np.vstack([line.p_start, line.p_end]))
        trk = Track(self._next_id, np.array([line.rho, line.alpha], dtype=float),
                    self._meas_cov(line), float(m.min()), float(m.max()), stamp, stamp)
        trk.history = collections.deque([True], maxlen=self.p.confirm_window)
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

    def step(self, lines_odom, stamp, car_pose):
        """Update with this scan's lines (odom). Returns the published tracks."""
        stamp = float(stamp)
        dt = 0.0 if self._last_stamp is None else stamp - self._last_stamp
        self._last_stamp = stamp
        self._predict(dt)

        used_lines, hit_ids = set(), set()
        if self.tracks and lines_odom:
            cost = np.array([[self._cost(t, ln) for ln in lines_odom] for t in self.tracks])
            finite = np.isfinite(cost)
            if finite.any():
                big = cost[finite].max() * 10.0 + 1e6
                rows, cols = linear_sum_assignment(np.where(finite, cost, big))
                for i, j in zip(rows, cols):
                    if finite[i, j]:
                        self._update(self.tracks[i], lines_odom[j], stamp)
                        used_lines.add(j)
                        hit_ids.add(self.tracks[i].id)
        for trk in self.tracks:
            hit = trk.id in hit_ids
            trk.history.append(hit)
            if sum(trk.history) >= self.p.confirm_hits:
                trk.ever_confirmed = True
            if trk.ever_confirmed:
                trk.state = STATE_CONFIRMED if hit else STATE_COASTING
            else:
                trk.state = STATE_TENTATIVE
        for j, line in enumerate(lines_odom):
            if j not in used_lines:
                self._spawn(line, stamp)

        self.tracks = [t for t in self.tracks if not self._expired(t, stamp)]
        self._merge()
        return self.published(car_pose)

    def _expired(self, trk, stamp):
        if stamp - trk.last_seen > self.p.delete_after_s:
            return True
        if math.sqrt(max(trk.P[0, 0], 0.0)) > self.p.max_sigma_rho_m:
            return True
        return (not trk.ever_confirmed and len(trk.history) >= self.p.confirm_window
                and sum(trk.history) < self.p.confirm_hits)

    def score(self, trk, car_pose):
        n = np.array([math.cos(trk.theta), math.sin(trk.theta)])
        dist = max(abs(trk.rho - float(n @ car_pose.t)), 0.1)
        return trk.length * trk.hits / dist

    def published(self, car_pose):
        """Return the ever-confirmed tracks, the best max_published by score."""
        live = [t for t in self.tracks if t.ever_confirmed]
        live.sort(key=lambda t: self.score(t, car_pose), reverse=True)
        return live[:self.p.max_published]

    def label(self, trk, car_pose):
        return line_label(trk.theta, trk.rho, car_pose, self.p.label_half_sector_rad)

    def front_track(self, car_pose, front_x):
        """Return the published front track the heading ray meets nearest, or None."""
        best, best_d = None, math.inf
        for trk in self.published(car_pose):
            if self.label(trk, car_pose) != 'front':
                continue
            d_perp, d_ray = front_distances(trk, car_pose, front_x, self.p.ray_margin_m)
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
