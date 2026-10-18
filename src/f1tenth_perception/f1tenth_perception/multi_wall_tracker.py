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
       chi2(2 DOF, 99%) = 9.21, and the observation's nearer end must lie
       within assoc_max_gap_m of the track's MEASURED extent (THE MEASURED
       EXTENT). A STEREO-only line nearer base_link than stereo_min_rho_m is
       the car's own chassis and never reaches it (THE NEAR FIELD).
     - Lifecycle: TENTATIVE becomes CONFIRMED at 3 hits in the last 5 scans.
       A confirmed track that goes unseen is COASTING while its covariance
       grows. A track is deleted after delete_after_s (2 s) unseen, after
       coast_max_odom_m (1 m) of ODOMETRY since its last association,
       when sigma_rho > max_sigma_rho_m, or (if still TENTATIVE) when 5 scans
       hold fewer than 3 hits. Time and distance are both needed: standing
       still costs a wall nothing and driving a metre past one without a
       single association means the wall is not where the track says it is.
     - NEGATIVE EVIDENCE (step 5): a confirmed, non-glass track whose own
       angular span is full of beams PASSING THROUGH it is being contradicted
       by the sensor, and five such scans in a row kill it. This is the only
       mechanism that can remove a wall the observations have merely stopped
       mentioning; the lifecycle can only wait for a timeout, which a phantom
       survives for as long as the room keeps producing collinear triples
       near it. Glass is exempt: it is the surface beams are meant to pass
       through.
     - Two tracks closer than the gate AND no more than assoc_max_gap_m
       apart along their line are merged into the older one.
     - THE TRAVEL STRIP'S OWN BEAMS (_apply_strip_evidence): the nearest
       return in the strip against where the strip meets each track; a track
       they see through for front_contradict_cycles cycles is CONTRADICTED.
     - Published: at most max_published (4) tracks that were ever confirmed,
       ranked by length * hits / distance.
     - ROLE: front, back, left, right or OTHER, from GEOMETRY (geometric_role)
       with hysteresis at the thresholds. Ids are never reused.

WHAT A ROLE IS, AND WHY IT IS NO LONGER A BEARING
-------------------------------------------------
A role used to be the bearing of the wall as seen from the car, in +-45 deg
sectors. That makes the far end of a corridor "front" and the near stretch of
the SAME wall "left" -- one surface with two roles depending on where the car
happens to be along it -- and a wall whose normal happened to point forwards
became the front wall at 1.1 m (corridor_phantom, 2026-10-15). A role is now
three geometric questions, asked in base_link:

  front / back  the segment, clipped to the car-width strip, reaches past the
                bumper or behind the rear edge. This is crosses_strip's own
                test, both ways, and crosses_strip is now one of its callers.
  left / right  the segment reaches into x in [-1, 3] AND its normal is
                within 45 deg of the LATERAL axis. The side is the sign of
                the mean y of the part inside that window.
  other         none of the above -- or the rectangle REJECTED the track (E2),
                or it is behind another confirmed wall (update_occlusion).

'other' is a first-class answer, not a failure: the track is real, it is
tracked, it is drawn, it still occludes other walls, and it is a reference
wall for nothing. is_front refuses it outright.

FIRST OBSTACLE, WALLS ONLY. A LiDAR beam reports its FIRST return, so a line
fitted behind a wall the same scan also saw cannot be a surface that scan
observed: it is the far side of the room through a gap, or a reflection. Five
points along each segment, three of them behind a confirmed wall (glass
included -- a pane really does stop the LiDAR), and the role becomes 'other'.
OBJECTS ARE UNCHANGED by this: an object behind a wall is still an object and
its sector is still cut out of the scan; it is only flagged
(lidar_front_wall_node._count_objects_behind_walls).
     - Front track (front_track): see WHAT FRONT REQUIRES.

THE MEASURED EXTENT (block 3, corridor10m_20261016T144537)
-----------------------------------------------------------
Track 955 was 13.67 m long, 3.5 m ahead, and made of the two wall faces LEFT
and RIGHT of a corridor mouth; the strip between them read 6.75-6.96 m. It
held FRONT for 5 s and stopped the car ~5 m early. Three things let it:

  * _merge joined any two tracks on the same line, with no look at the gap
    between them, and the merged extent was the UNION of both. Association
    needed an overlap; the merge did not. Now both need the nearer ends to be
    at most assoc_max_gap_m apart -- counting only the part of the gap the
    beams saw OPEN (_gap_between): behind a detected person or a nearer
    surface the wall is unobserved, not absent.
  * The extent only ever grew. It is now rebuilt every step from the
    observations of the last extent_decay_s (_refresh_extent), across open
    gaps of at most assoc_max_gap_m only, so an end nobody re-observes goes.
  * FRONT was the heading ray against the extent plus 0.3 m. It is now where
    the travel strip meets the MEASURED segment (strip_distance).

WHAT FRONT REQUIRES (front_reason, in order)
--------------------------------------------
Confirmed; not CONTRADICTED by the strip's own beams; role not 'other'; the
measured segment meets the TRAVEL STRIP, |y| <= side_edge_half_width_m +
front_strip_margin_m ahead of the bumper (front_x_m); if coasting, for at
most front_max_coast_s AND front_max_coast_m, over a strip that does not see
through it; front_confidence >= front_min_confidence; a LiDAR-backed
association in the last front_lidar_recent_cycles cycles OR the rectangle
VERIFIED it; and, stereo-only, squarely across the path. Without a FRONT the
contract is CLEAR -- NO_DATA is only for missing data (the node).

THE STRIP'S OWN BEAMS. The nearest valid return in the travel strip
(strip_returns) is compared with where the strip meets each confirmed track:
more than front_contradict_margin_m beyond it means every in-strip beam went
through where the wall should be. Beams cut for an object are NOT free space
(they are inf, like a no-return, so they need the cut mask), and a strip with
no return at all is no evidence either way: that is what glass does.

THE NEAR FIELD. The camera sees part of the chassis in the lower right of its
image; where the self polygons miss it, the stereo reports it as a wall line
~0.2 m from base_link. No wall line can pass that close without touching the
car, so a STEREO-only one is dropped (stereo_min_rho_m); a LiDAR one stays.

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
is the union of its recent observations, so gating on the track would let one
long observation license every short one after it.

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

#: both > lidar > stereo, the order Track.source keeps (_absorb).
_SOURCE_RANK = {wall_fusion.SOURCE_STEREO: 0, wall_fusion.SOURCE_LIDAR: 1,
                wall_fusion.SOURCE_BOTH: 2}


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

def extraction_params(merge_max_gap_m=None, quality_gates=None, backend=None):
    """The extraction's own defaults, with the merge gap and the gates overridden.

    The merge is what makes a span mean "the length of this wall" rather than
    "the length of the piece of it nothing was standing in front of", so the
    one tolerance a wall-span threshold has to be set together with is this
    gap. `quality_gates` is corridor_perception's seed/rms/fill gates (see
    extraction.py, WHY THE QUALITY GATES); off, the extraction reports every
    collinear triple it can find, which is what the span gate below then has
    to clean up on its own. `backend` is 'c' (corridor_perception_c's
    libextract_lines.so, 1.1 ms a scan) or 'python' (the reference, 24.6 ms);
    the two agree to 7e-15 over every scan of corridor10m_20261015T171753.
    Everything else stays corridor_perception's own default.
    """
    p = ExtractionParams()
    if merge_max_gap_m is not None:
        p.merge_gap = float(merge_max_gap_m)
    if quality_gates is not None:
        p.quality_gates = bool(quality_gates)
    if backend is not None:
        p.backend = str(backend)
    return p


def scan_in_base_link(ranges, angle_min, angle_increment, range_min, range_max,
                      laser_pose, sectors=(), with_cut=False):
    """(ranges, base_link bearings, laser-frame angles[, cut]) of one scan, sectors cut.

    One implementation for the extraction and for the negative evidence
    (MultiWallTracker.negative_evidence): both have to see exactly the same
    beams, because a beam cut out because an OBJECT is standing there is not
    evidence against a wall -- it is evidence about the object.

    A cut beam becomes inf, which is also what clean_ranges makes of a
    no-return. `with_cut` adds the bool mask that tells the two apart: the
    evidence rules need it, because "nothing came back" is a statement about
    the space ahead and "an object was cut out here" is not.
    """
    r = clean_ranges(ranges, range_min, range_max)
    _lx, _ly, lyaw = laser_pose
    angles = angle_min + np.arange(r.size) * angle_increment
    bearings = wrap_pi(angles + lyaw)
    cut = np.zeros(r.size, dtype=bool)
    if sectors:
        cut = in_sectors(bearings, sectors)
        if cut.any():
            r = np.where(cut, np.inf, r).view(type(r))
    if with_cut:
        return r, bearings, angles, cut
    return r, bearings, angles


def extract_lines(ranges, angle_min, angle_increment, range_min, range_max, laser_pose,
                  sectors=(), params=None, stats=None):
    """Wall segments of one scan as corridor_perception Lines in base_link.

    laser_pose: (x, y, yaw) of the laser in base_link. sectors: bearing
    sectors [(lo, hi)] in base_link (scene_labels.detection_sectors) whose
    beams are treated as no-returns. params: ExtractionParams, normally built
    by `extraction_params` from the node's merge-gap and quality-gate
    parameters. stats: filled with what each extraction gate dropped.
    """
    r, _bearings, angles = scan_in_base_link(
        ranges, angle_min, angle_increment, range_min, range_max, laser_pose, sectors)
    lx, ly, lyaw = laser_pose
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
    # The COLLINEAR GAP (THE MEASURED EXTENT, below): an observation joins a
    # track, and two tracks merge, only if the nearer ends are at most this
    # far apart along the line. It replaces the old 0.2 m OVERLAP test, which
    # held for association but not for _merge -- and _merge is what built a
    # 13.67 m front wall out of the two faces either side of a corridor mouth.
    assoc_max_gap_m: float = 0.5
    # An end of the extent that no associated observation has reached for
    # this long is dropped (THE MEASURED EXTENT).
    extent_decay_s: float = 1.0
    confirm_hits: int = 3
    confirm_window: int = 5
    # Coasting is bounded on TIME and on DISTANCE (coast_max_odom_m), and
    # the time was 3.0 s: at the mission speed that is 1.5 m of travel on a
    # wall nobody has seen, with the covariance growing the whole way.
    delete_after_s: float = 2.0
    max_sigma_rho_m: float = 0.3
    max_published: int = 4
    label_half_sector_rad: float = math.radians(45.0)
    role_hysteresis_rad: float = math.radians(5.0)   # a role sticks this far past its edge
    # Extent slack for front_distances' heading ray. 0: a wall is where it
    # was MEASURED, not 30 cm further (THE MEASURED EXTENT).
    ray_margin_m: float = 0.0
    # A coasting FRONT track's last say, on time AND on odometry, whichever
    # comes first (WHAT FRONT REQUIRES).
    front_max_coast_s: float = 0.5
    front_max_coast_m: float = 0.3
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
    # --- what may be called FRONT (WHAT FRONT REQUIRES) --------------------
    front_x_m: float = 0.405                # vehicle_frames.base_front_x(): the bumper
    side_edge_half_width_m: float = 0.136   # swept_clearance_body_half_width_m
    front_strip_margin_m: float = 0.15      # the travel strip past the bodywork
    stereo_front_max_normal_rad: float = math.radians(30.0)
    front_min_confidence: float = 0.5       # see front_confidence
    front_lidar_recent_cycles: int = 3      # a LiDAR fit this recent, or rect VERIFIED
    # The strip's own beams against the FRONT candidate (strip_verdict).
    front_contradict_margin_m: float = 0.5
    front_contradict_cycles: int = 2
    # A STEREO-only observation whose line passes nearer base_link than this
    # is the car's own chassis, not a wall (step, THE NEAR FIELD).
    stereo_min_rho_m: float = 0.4
    # --- CAMERA OBSERVATION GATES (Q2, 2026-10-18) ------------------------
    # A surface lying ALONG the viewing ray is not observable: one pixel of
    # lateral error moves it metres, and a column profile turns that into the
    # 45 deg "fans" that litter /perception/walls. The LiDAR gates the same
    # thing by incidence; the camera never did. Measured on
    # corridor10m_20261016T144537 (output/wall_v2/q1): 437 of 536 stereo
    # segments dropped, spurious segments per frame 0.932 -> 0.112, residual
    # unchanged. THE PRICE, deliberately accepted and measured: side-wall
    # coverage falls 58.9 % -> 14.2 %, because a corridor wall seen from a car
    # driving along it IS nearly radial. LiDAR observations never see these.
    camera_seg_min_ray_angle_rad: float = math.radians(25.0)
    camera_seg_min_bearing_span_rad: float = math.radians(3.0)
    #: A stereo-only line this badly fitted, this flat, or this short is not a
    #: wall: nothing cross-checks it, so the bar is a shape the LiDAR could
    #: not plausibly have missed by accident.
    stereo_max_rms_m: float = 0.05
    stereo_min_vertical_extent_m: float = 0.4
    stereo_min_length_m: float = 0.8
    #: Count a camera-only hit towards confirmation ONLY on a cycle whose
    #: camera data was fresh (the node sets `camera_fresh`). A stale profile
    #: repeated over three cycles is one observation, not three.
    camera_confirm_fresh_only: bool = True
    # --- the geometric roles (WHAT A ROLE IS NOW) -------------------------
    back_x_m: float = 0.0                   # the rear edge of the strip, base_link
    side_x_min_m: float = -1.0              # the x window a side wall must reach
    side_x_max_m: float = 3.0
    side_max_normal_rad: float = math.radians(45.0)
    #: A CAMERA-only track gets its own, tighter side zone: the camera's
    #: angular error grows with range where the LiDAR's does not, so a
    #: camera wall must be more clearly lateral and must sit at a plausible
    #: corridor offset before it is called a reference wall.
    camera_side_x_max_m: float = 4.0
    camera_side_max_normal_rad: float = math.radians(30.0)
    camera_side_offset_min_m: float = 0.3
    camera_side_offset_max_m: float = 3.0
    role_hysteresis_m: float = 0.05         # the length half of the hysteresis
    # --- first obstacle: a wall behind a wall is not a reference wall ------
    occlusion_points: int = 5               # samples along the segment
    occlusion_min_blocked: int = 3          # of them, to call it occluded
    occlusion_margin_m: float = 0.10
    # --- negative evidence ------------------------------------------------
    negative_shrink_beams: int = 2          # drop this many beams per side
    negative_tol_m: float = 0.15
    negative_min_beams: int = 5
    negative_pass_frac: float = 0.7
    negative_miss_to_kill: int = 5
    # --- coasting ---------------------------------------------------------
    coast_max_odom_m: float = 1.0           # metres of odometry, not displacement

    @property
    def strip_half_width_m(self):
        """Half-width of the TRAVEL STRIP: the bodywork plus front_strip_margin_m.

        One strip for the roles, for FRONT, for the FRONT distance and for the
        strip's own beams, so "in the way" means one geometry in this module.
        """
        return float(self.side_edge_half_width_m) + float(self.front_strip_margin_m)


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
    # --- the rectangle verification (wall_fusion, THE RECTANGLE) -----------
    #: Sticky, hysteretic: UNVERIFIED (= lidar_only, not a doubt), VERIFIED or
    #: REJECTED. Only a verdict with coverage >= coverage_min moves it.
    verify: int = wall_fusion.VERIFY_UNVERIFIED
    #: VERIFIED, and the stereo saw the scene BEYOND the line over almost all
    #: of the rectangle: a pane. Its distance comes from the LiDAR alone.
    glass: bool = False
    rect_coverage: float = 0.0
    rect_support: float = 0.0
    rect_beyond_share: float = 0.0
    #: Did the rectangle run INFORMATIVELY on this cycle (coverage >= min)?
    #: Only then may this cycle's evidence be called "both".
    rect_informative: bool = False
    #: Consecutive VERIFIABLE cycles whose support was at or below
    #: support_reject. Verification is immediate; rejection is not.
    rect_reject_streak: int = 0
    #: Consecutive scans of NEGATIVE EVIDENCE (see negative_evidence).
    miss: int = 0
    #: The tracker's odometer when this track was last associated, so a
    #: coasting track can be killed on DISTANCE as well as on time.
    odo_at_last_seen: float = 0.0
    #: Is this wall behind another confirmed wall this cycle (first obstacle)?
    occluded: bool = False
    #: (stamp, (2, 2) odom endpoints) of every associated observation inside
    #: extent_decay_s: what t0/t1 are rebuilt from (THE MEASURED EXTENT).
    obs_ext: collections.deque = field(default_factory=lambda: collections.deque(maxlen=64))
    #: Was the track associated with a LiDAR-backed (LIDAR or BOTH)
    #: observation, one entry per tracker cycle, newest last.
    lidar_hist: collections.deque = field(default_factory=lambda: collections.deque(maxlen=3))
    #: Like `history`, but a camera-only hit on a stale camera cycle counts as
    #: a miss. Confirmation of a camera-only track reads this one.
    fresh_hist: collections.deque = field(default_factory=lambda: collections.deque(maxlen=12))  # noqa: E501
    #: The confidences of the last matched observations (front_confidence).
    conf_hist: collections.deque = field(default_factory=lambda: collections.deque(maxlen=5))
    #: The strip's own beams against this track (strip_verdict) this cycle,
    #: the distance [m, from the bumper] they were judged against, the run of
    #: consecutive contradicting cycles, and the sticky verdict: CONTRADICTED
    #: stays until a cycle's strip beams land on the line again.
    strip_verdict: str = ''
    strip_pred_m: float = math.nan
    strip_miss: int = 0
    contradicted: bool = False

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
    span = _clip_param(y0, y1, -half, half)
    if span is None:
        return False
    xs = (x0 + span[0] * (x1 - x0), x0 + span[1] * (x1 - x0))
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


def strip_distance(seg, front_x, half_width_m):
    """Distance [m] from the bumper to where the travel strip meets a segment, or NaN.

    `seg` is ((x0, y0), (x1, y1)) in base_link. The segment is clipped to
    |y| <= half_width_m; NaN when nothing survives or what survives is all
    behind the bumper. Otherwise the NEAREST x of the clipped part, minus
    front_x, floored at 0 (a segment reaching back under the bumper touches).

    This is THE front distance since the measured extent (WHAT FRONT
    REQUIRES): where the strip the car sweeps meets the wall that was
    measured, not where the heading ray meets the wall's infinite line.
    """
    (x0, y0), (x1, y1) = seg
    half = abs(float(half_width_m))
    span = _clip_param(float(y0), float(y1), -half, half)
    if span is None:
        return math.nan
    xs = (x0 + span[0] * (x1 - x0), x0 + span[1] * (x1 - x0))
    if max(xs) <= float(front_x):
        return math.nan
    return max(float(min(xs)) - float(front_x), 0.0)


class StripReading:
    """The LiDAR's own view of the travel strip on one scan (strip_returns)."""

    __slots__ = ('range_m', 'n_returns', 'n_cut', 'n_no_return')

    def __init__(self, range_m=None, n_returns=0, n_cut=0, n_no_return=0):
        #: Nearest return inside the strip, from the bumper [m]; None when the
        #: strip holds no return at all (glass, out of range, or cut).
        self.range_m = range_m
        self.n_returns = int(n_returns)
        self.n_cut = int(n_cut)
        self.n_no_return = int(n_no_return)


def strip_returns(scan, front_x, half_width_m):
    """StripReading of one scan: the nearest valid return in the travel strip.

    `scan` is (ranges, base_link bearings, laser xy[, cut]) as step takes
    it. A return is in the strip when its base_link point has x > front_x
    and |y| <= half_width_m; the range is x - front_x, from the bumper, the
    same reference as strip_distance.

    CUT BEAMS ARE NOT FREE SPACE. scan_in_base_link turns a beam cut out for
    an object into inf, exactly what clean_ranges makes of a no-return, so
    without the mask a person standing in the strip would read as "nothing
    there". Cut beams are excluded here and only counted; so are no-returns,
    which leave range_m None rather than infinite: "nothing came back" is
    no evidence of where a wall is NOT (it is what glass does).
    """
    ranges, bearings, laser_xy = scan[0], scan[1], scan[2]
    r = np.asarray(ranges, dtype=float)
    b = np.asarray(bearings, dtype=float)
    cut = (np.asarray(scan[3], dtype=bool) if len(scan) > 3 and scan[3] is not None
           else np.zeros(r.shape, dtype=bool))
    lx, ly = (float(v) for v in np.asarray(laser_xy, dtype=float).reshape(2))
    c, s = np.cos(b), np.sin(b)
    half = abs(float(half_width_m))
    fx = float(front_x)
    # Beams that run through the strip ahead of the bumper: still inside the
    # band when they reach x = front_x (the laser sits inside the band).
    with np.errstate(divide='ignore', invalid='ignore'):
        t_in = np.where(c > 1e-6, (fx - lx) / c, np.inf)
        through = (c > 1e-6) & (np.abs(ly + t_in * s) <= half)
    finite = np.isfinite(r) & ~cut
    with np.errstate(invalid='ignore'):
        px = lx + np.where(finite, r, 0.0) * c
        py = ly + np.where(finite, r, 0.0) * s
    inside = finite & (px > fx) & (np.abs(py) <= half)
    n_cut = int(np.count_nonzero(through & cut))
    n_no = int(np.count_nonzero(through & ~cut & ~np.isfinite(r)))
    if not inside.any():
        return StripReading(None, 0, n_cut, n_no)
    return StripReading(float(px[inside].min()) - fx, int(np.count_nonzero(inside)),
                        n_cut, n_no)


STRIP_SUPPORTED = 'supported'        # the strip's nearest return is ON the line
STRIP_CONTRADICTED = 'contradicted'  # it is BEYOND the line: the beams pass through
STRIP_BLOCKED = 'blocked'            # something nearer: says nothing about the line
STRIP_NO_RETURNS = 'no_returns'      # nothing came back (glass, range, cut)


def strip_verdict(pred_m, strip_range_m, margin_m):
    """What the strip's nearest return says about a wall predicted at pred_m.

    '' when the track does not meet the strip (pred_m NaN). Within margin_m
    of the prediction is SUPPORTED; more than margin_m BEYOND it is
    CONTRADICTED -- every in-strip beam went past where the wall should be,
    the doorway of corridor10m_20261016T144537; more than margin_m SHORT of it
    is BLOCKED, and a nearer surface is no evidence either way. A strip with
    no return at all is NO_RETURNS.
    """
    if pred_m is None or not math.isfinite(pred_m):
        return ''
    if strip_range_m is None:
        return STRIP_NO_RETURNS
    if strip_range_m > pred_m + margin_m:
        return STRIP_CONTRADICTED
    if strip_range_m < pred_m - margin_m:
        return STRIP_BLOCKED
    return STRIP_SUPPORTED


def contract_reason(front_reason, strip_range_m, clear_range_m):
    """front_reason for /perception/virtual_wall/debug, with the strip folded in.

    The tracker's own answer ('front', a reject code, or 'no_candidate'),
    except that NO candidate while the strip holds a return nearer than
    clear_range_m is 'unexplained_returns': the contract still says CLEAR --
    the wall contract is about walls, an object is front_clearance's and
    IsProximityTooClose's business, and a wall invented from raw returns is
    the phantom path all over again -- but the debug output must say that
    the strip was not empty.
    """
    if (front_reason == 'no_candidate' and strip_range_m is not None
            and strip_range_m < float(clear_range_m)):
        return 'unexplained_returns'
    return front_reason


def front_distances(track, car_pose, front_x, margin_m=0.0):
    """(perpendicular, along-heading) distance [m] from base_front to the track.

    The ray distance is NaN when the heading ray from the bumper misses the
    segment's extent (plus margin_m) or points away from the line. The
    contract's FRONT distance is strip_distance, not this: this is the
    heading-ray figure the MultiWallTrack message has always carried.
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


def _clip_param(a, b, lo, hi):
    """(t_lo, t_hi) of the part of a + t (b - a) inside [lo, hi], or None.

    Liang-Barsky on one coordinate, which is all crosses_strip ever needed
    and all a role test needs either.
    """
    d = b - a
    t0, t1 = 0.0, 1.0
    for sign, limit in ((1.0, hi), (-1.0, -lo)):
        num, den = limit - sign * a, sign * d
        if abs(den) < 1e-12:
            if num < 0.0:
                return None
        elif den > 0.0:
            t1 = min(t1, num / den)
        else:
            t0 = max(t0, num / den)
    return (t0, t1) if t0 <= t1 else None


def strip_role(track, car_pose, front_x, back_x, half_width_m, seg=None):
    """'front', 'back' or '' -- the car-width strip test, both ways.

    The segment is clipped to |y| <= half_width_m, exactly as crosses_strip
    does it (and this is now the implementation both use), and what survives
    decides: past the bumper is FRONT, behind the rear edge is BACK. A
    segment that survives the clip but lies between the two is beside the
    car, not in its way, and gets neither.
    """
    (x0, y0), (x1, y1) = segment_in_base_link(track, car_pose) if seg is None else seg
    half = abs(float(half_width_m))
    span = _clip_param(y0, y1, -half, half)
    if span is None:
        return ''
    xs = (x0 + span[0] * (x1 - x0), x0 + span[1] * (x1 - x0))
    if max(xs) > float(front_x):
        return 'front'
    return 'back' if min(xs) < float(back_x) else ''


def side_role(track, car_pose, x_min, x_max, max_normal_rad, seg=None):
    """'left', 'right' or '' -- a wall the car is driving ALONG.

    Two conditions, and both are geometric rather than about bearing:

      * the segment must reach into x in [x_min, x_max] in base_link. A wall
        20 m behind the car is still a wall and still tracked, but it is not
        a reference for anything: the window is the stretch the car is
        actually beside or about to be beside.
      * its normal must be within max_normal_rad of the LATERAL axis. The old
        role came from the bearing of the wall as seen from the car, which
        makes the far end of a corridor "front" and the near stretch of the
        same wall "left" -- the same surface with two roles, depending on
        where the car happens to be along it. A wall's normal does not depend
        on that.

    The side is the sign of the mean y of the part inside the window, so a
    wall the car is level with is named by where it actually is.
    """
    (x0, y0), (x1, y1) = segment_in_base_link(track, car_pose) if seg is None else seg
    span = _clip_param(x0, x1, float(x_min), float(x_max))
    if span is None:
        return ''
    nx, ny = math.cos(track.theta - car_pose.theta), math.sin(track.theta - car_pose.theta)
    if abs(ny) < math.cos(float(max_normal_rad)):
        return ''
    ys = (y0 + span[0] * (y1 - y0), y0 + span[1] * (y1 - y0))
    return 'left' if 0.5 * (ys[0] + ys[1]) > 0.0 else 'right'


def camera_side_role(track, car_pose, p, seg=None, widen_m=0.0, widen_rad=0.0):
    """side_role with the CAMERA zone, plus a plausible lateral offset.

    The camera's angular error grows with range where the LiDAR's does not,
    so a camera-only wall must be more clearly lateral (30 deg, not 45), must
    reach into a shorter x window, and must sit at a corridor-like distance
    from the car before it is called a reference wall. A "side wall" 10 cm
    away is the chassis; one 5 m away is the far side of a hall.
    """
    role = side_role(track, car_pose, p.side_x_min_m - widen_m,
                     p.camera_side_x_max_m + widen_m,
                     p.camera_side_max_normal_rad + widen_rad, seg)
    if not role:
        return ''
    (x0, y0), (x1, y1) = segment_in_base_link(track, car_pose) if seg is None else seg
    offset = abs(0.5 * (y0 + y1))
    if not (p.camera_side_offset_min_m - widen_m <= offset
            <= p.camera_side_offset_max_m + widen_m):
        return ''
    return role


def geometric_role(track, car_pose, p, widen_m=0.0, widen_rad=0.0, seg=None):
    """The track's role from GEOMETRY: front, back, left, right or other.

    'other' is a first-class answer, not a failure: the track is still
    tracked, still drawn and still occludes other walls, it is simply not a
    reference wall for anything. Before this every track got one of four
    roles whether or not it was anywhere near the car, and a far wall whose
    normal happened to point forwards became the FRONT wall.

    `widen_m` and `widen_rad` relax the length thresholds and the normal
    sector: the hysteresis below asks this same question with them widened
    rather than keeping a second rule. They are separate because they are
    separate units -- the one bug a single "widen" invites.
    """
    if seg is None:
        seg = segment_in_base_link(track, car_pose)
    # front_x is the BUMPER (p.front_x_m). It used to be passed the strip
    # half-width (0.186) in this slot, so 'front' began under the car.
    strip = strip_role(track, car_pose, p.front_x_m - widen_m,
                       p.back_x_m + widen_m, p.strip_half_width_m + widen_m, seg)
    if strip:
        return strip
    if track.source == wall_fusion.SOURCE_STEREO:
        return camera_side_role(track, car_pose, p, seg, widen_m, widen_rad) or 'other'
    return side_role(track, car_pose, p.side_x_min_m - widen_m,
                     p.side_x_max_m + widen_m,
                     p.side_max_normal_rad + widen_rad, seg) or 'other'


def role_with_hysteresis(track, car_pose, current, p, seg=None):
    """The geometric role, keeping `current` while the widened test still holds.

    Same shape and same reason as the bearing-sector hysteresis it replaces:
    a wall sitting on a threshold would otherwise change role every scan as
    the car wobbles, and every consumer keyed on the role would change with
    it.
    """
    if seg is None:
        seg = segment_in_base_link(track, car_pose)
    fresh = geometric_role(track, car_pose, p, seg=seg)
    if not current or current == fresh:
        return fresh
    widened = geometric_role(track, car_pose, p, float(p.role_hysteresis_m),
                             float(p.role_hysteresis_rad), seg=seg)
    return current if widened == current else fresh


def radial_verdict(seg, origin_xy, min_ray_rad, min_span_rad):
    """(keep, ray_angle_rad, bearing_span_rad) for a camera segment.

    A segment whose direction lies along the ray from the camera to its own
    midpoint is seen end-on: its range is a guess. So is one that subtends
    almost no bearing. Both are cheap scalar tests on the base_link segment.
    """
    (x0, y0), (x1, y1) = seg
    ox, oy = float(origin_xy[0]), float(origin_xy[1])
    mx, my = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    rx, ry = mx - ox, my - oy
    dx, dy = x1 - x0, y1 - y0
    nr, nd = math.hypot(rx, ry), math.hypot(dx, dy)
    if nr < 1e-6 or nd < 1e-6:
        return False, 0.0, 0.0
    cosang = min(1.0, abs(rx * dx + ry * dy) / (nr * nd))
    ray = math.acos(cosang)
    ray = min(ray, math.pi - ray)
    b0 = math.atan2(y0 - oy, x0 - ox)
    b1 = math.atan2(y1 - oy, x1 - ox)
    span = abs(wrap_pi(b1 - b0))
    return (ray >= float(min_ray_rad) and span >= float(min_span_rad)), ray, span


def _bearing_sector(seg, laser_xy):
    """(centre, half width) of a base_link segment's bearing span from the laser."""
    (ax, ay), (bx, by) = seg
    b0 = math.atan2(ay - laser_xy[1], ax - laser_xy[0])
    b1 = math.atan2(by - laser_xy[1], bx - laser_xy[0])
    d = wrap_pi(b1 - b0)
    return wrap_pi(b0 + 0.5 * d), 0.5 * abs(d)


def _sectors_overlap(a, b):
    """Do two (centre, half) bearing sectors share a bearing?"""
    return abs(wrap_pi(a[0] - b[0])) <= a[1] + b[1]


def ray_blocked_by(points, laser_xy, blockers, margin_m=0.10):
    """Bool (N,): the ray laser -> point crosses a blocker segment before it.

    `blockers` is [((x0, y0), (x1, y1))] in base_link. A point is blocked when
    some blocker is crossed at least margin_m nearer than the point itself:
    the wall the LiDAR actually ranged along that bearing is the FIRST one,
    so anything the beam could not have reached is not an observation of a
    surface -- it is an observation of this blocker.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    out = np.zeros(pts.shape[0], dtype=bool)
    if pts.shape[0] == 0:
        return out
    lxy = np.asarray(laser_xy, dtype=float).reshape(2)
    d = pts - lxy
    norm = np.hypot(d[:, 0], d[:, 1])
    ok = norm > 1e-6
    if not ok.any():
        return out
    for (qx0, qy0), (qx1, qy1) in blockers:
        e = np.array([qx1 - qx0, qy1 - qy0], dtype=float)
        q0 = np.array([qx0, qy0], dtype=float) - lxy
        den = d[:, 0] * e[1] - d[:, 1] * e[0]
        with np.errstate(divide='ignore', invalid='ignore'):
            s = (q0[0] * e[1] - q0[1] * e[0]) / den       # along the ray, 0..1
            u = (d[:, 0] * q0[1] - d[:, 1] * q0[0]) / -den  # along the blocker
            near = s < 1.0 - margin_m / np.maximum(norm, 1e-6)
        hit = (np.abs(den) > 1e-12) & ok & (s > 1e-6) & near & (u >= 0.0) & (u <= 1.0)
        out |= hit
    return out


class MultiWallTracker:
    """Wall tracks in odom; see the module docstring."""

    def __init__(self, params=None, rect_params=None):
        self.p = params or TrackerParams()
        #: The rectangle verification's own numbers (wall_fusion.RectParams).
        #: The tracker holds them because the sticky state they drive lives on
        #: the tracks, not on the per-cycle observations.
        self.rect = rect_params or wall_fusion.RectParams()
        self.tracks = []
        #: The laser's (x, y) in base_link, for the occlusion rays. None means
        #: base_link's origin, which is 30 cm off on this car.
        self.laser_xy = None
        #: Where the camera looks from, base_link, for the radial test. The
        #: node sets it from /tf_static; without it the laser stands in.
        self.camera_xy = None
        #: Did THIS cycle have fresh camera data? The node sets it per step.
        self.camera_fresh = True
        self._next_id = 1
        self._last_stamp = None
        self._last_yaw = None
        #: Observations the span gate dropped on the last step, for the debug
        #: payload: a run of these is the signal that the room is full of short
        #: faces, or that the merge gap is too tight to reunify a wall.
        self.n_short_dropped = 0
        #: Metres of odometry since the tracker started. A coasting track is
        #: killed on DISTANCE as well as on time: 2 s of standing still costs
        #: a wall nothing, while 1 m of driving past it without a single
        #: association means the wall is not where the track says it is.
        self._odometer = 0.0
        self._last_car_t = None
        #: Tracks the negative evidence killed on the last step, for the
        #: debug payload.
        self.n_negative_killed = 0
        #: Last step's collinear-gap refusals: observations that matched a
        #: track's line but not its extent (and so started or fed another
        #: track), and track pairs _merge refused to join. Same pattern as
        #: n_short_dropped; `rejections` carries a reason code per event.
        self.n_gap_rejected = 0
        self.n_merge_gap_refused = 0
        #: Last step's STEREO-only observations dropped as the car's own
        #: chassis (stereo_min_rho_m).
        self.n_stereo_near_rejected = 0
        self.n_camera_rejected = 0
        #: [(code, track id or None, value)] of this step's refusals:
        #: 'gap_assoc' (value = gap m), 'gap_merge' (gap m), 'stereo_near'
        #: (rho_bl m). Capped: it is a debug trail, not a log.
        self.rejections = []
        #: The travel strip on the last scan step was given (StripReading).
        self.strip = StripReading()
        #: What front_track decided last: {'id', 'reason', 'distance_m',
        #: 'rejects': {id: code}}. 'reason' is 'front', the nearest
        #: candidate's reject code, or 'no_candidate'.
        self.front_status = {'id': None, 'reason': 'no_candidate',
                             'distance_m': math.nan, 'rejects': {}}
        #: (car pose, scan) of the step in progress, for _open_length; None
        #: when the step was given no scan.
        self._scan_ctx = None

    _MAX_REJECTIONS = 16

    def _reject(self, code, track_id, value):
        if len(self.rejections) < self._MAX_REJECTIONS:
            self.rejections.append((code, track_id, round(float(value), 3)))

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

    def _extent_gap(self, trk, p_a, p_b):
        """OPEN gap [m] along trk's line between its extent and the segment p_a-p_b.

        Negative when they overlap. Both ends are projected onto the TRACK's
        tangent, so this is the collinear gap the association and the merge
        gate on (assoc_max_gap_m); see _gap_between for "open".
        """
        m = _along(trk.theta, np.vstack([p_a, p_b]))
        return self._gap_between(trk.theta, trk.rho, trk.t0, trk.t1,
                                 float(m.min()), float(m.max()))

    def _gap_between(self, theta, rho, a_lo, a_hi, b_lo, b_hi):
        """Gap [m] between two intervals along one line, counting only what is OPEN.

        A gap wider than assoc_max_gap_m is checked against this step's scan
        (_open_length): the part of it the LiDAR could not see -- beams cut
        for a detected object, or stopped short by something nearer -- is
        not an opening, it is a wall with a person in front of it (the M02
        case the extraction's own 1.5 m merge exists for). Only what the
        beams went THROUGH counts. With no scan, the whole gap counts.
        """
        raw = max(b_lo - a_hi, a_lo - b_hi)
        if raw <= self.p.assoc_max_gap_m:
            return raw
        lo, hi = (a_hi, b_lo) if b_lo - a_hi >= a_lo - b_hi else (b_hi, a_lo)
        return self._open_length(theta, rho, lo, hi)

    _GAP_STEP_M = 0.05

    def _open_length(self, theta, rho, t_lo, t_hi):
        """Metres of the odom line stretch t_lo..t_hi that this step's beams saw OPEN.

        Sampled every 5 cm. A sample is OPEN when the beam towards it came
        back more than negative_tol_m BEYOND it or not at all; it is NOT open
        when that beam was cut (an object sector), stopped short of it, or
        landed on it -- or when no beam points at it at all (the 90 deg blind
        sector behind the laser): a stretch the car has its back to is not
        an opening, it is a wall nobody is looking at.
        """
        length = float(t_hi) - float(t_lo)
        ctx = self._scan_ctx
        if ctx is None or length <= 0.0:
            return length
        car_pose, scan = ctx
        ts = np.arange(float(t_lo) + 0.5 * self._GAP_STEP_M, float(t_hi), self._GAP_STEP_M)
        if ts.size == 0:
            return length
        n = np.array([math.cos(theta), math.sin(theta)])
        tan = np.array([-n[1], n[0]])
        pts = car_pose.inverse().transform_points(n * float(rho) + ts[:, None] * tan)
        lxy = np.asarray(scan[2], dtype=float).reshape(2)
        d = pts - lxy
        dist = np.hypot(d[:, 0], d[:, 1])
        bear = np.arctan2(d[:, 1], d[:, 0])
        r = np.asarray(scan[0], dtype=float)
        b = np.asarray(scan[1], dtype=float)
        cut = (np.asarray(scan[3], dtype=bool) if len(scan) > 3 and scan[3] is not None
               else np.zeros(r.shape, dtype=bool))
        if b.size < 2:
            return length
        inc = abs(float(b[1] - b[0]))
        if b[-1] > b[0]:
            idx = np.clip(np.searchsorted(b, bear), 1, b.size - 1)
            idx = np.where(np.abs(b[idx - 1] - bear) < np.abs(b[idx] - bear), idx - 1, idx)
        else:   # a scan whose bearings wrap: the slow way
            idx = np.argmin(np.abs(wrap_pi(b[None, :] - bear[:, None])), axis=1)
        in_fov = np.abs(wrap_pi(b[idx] - bear)) <= 1.5 * inc
        rv = r[idx]
        tol = float(self.p.negative_tol_m)
        with np.errstate(invalid='ignore'):
            open_ = in_fov & ~cut[idx] & (~np.isfinite(rv) | (rv > dist + tol))
        return min(length, float(np.count_nonzero(open_)) * self._GAP_STEP_M)

    def _gate(self, trk, line, may_birth=True):
        """(Mahalanobis distance squared or inf, refusal code or '').

        `may_birth` False means the observation is shorter than
        min_span_birth_m: it may only reach a track that is ALREADY confirmed
        (module docstring, WHY THE SPAN GATE IS HYSTERETIC). Gating it here
        rather than after the assignment matters -- a short segment that was
        allowed to compete for a tentative track would both feed that track
        and consume the assignment a long segment needed.

        Then the line must pass the chi2 gate AND the observation's nearer end
        must lie within assoc_max_gap_m of the track's MEASURED extent: a
        collinear face across a doorway is the same line and a different
        wall ('gap').
        """
        if not may_birth and not trk.ever_confirmed:
            return math.inf, 'birth'
        v = _innovation(trk.x, line.rho, line.alpha)
        S = trk.P + self._meas_cov(line)
        d2 = float(v @ np.linalg.solve(S, v))
        if d2 > self.p.gate_chi2:
            return math.inf, 'chi2'
        if self._extent_gap(trk, line.p_start, line.p_end) > self.p.assoc_max_gap_m:
            return math.inf, 'gap'
        return d2, ''

    def _cost(self, trk, line, may_birth=True):
        """Mahalanobis distance squared, or inf when gated out (see _gate)."""
        return self._gate(trk, line, may_birth)[0]

    def _absorb(self, trk, obs):
        """Carry an observation's own attributes onto the track it matched."""
        trk.confidence = float(obs.confidence)
        trk.conf_hist.append(float(obs.confidence))
        if obs.source == wall_fusion.SOURCE_BOTH:
            # "BOTH" now means both sensors looked at the WALL, not just at
            # the bearing: a column agreement alone is one row of pixels
            # through the scan plane, which is the measurement the stereo is
            # worst at. The rectangle must have had something to say about
            # this track (coverage >= coverage_min) on its latest run, or the
            # cycle counts as LiDAR evidence -- which it is.
            trk.has_lidar = True
            trk.has_both = trk.has_both or trk.rect_informative
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

    def _refresh_extent(self, trk, now=None):
        """Rebuild t0/t1 from the observations inside extent_decay_s.

        THE MEASURED EXTENT. The extent used to be the union of everything
        ever associated, so it only grew, and _merge's union could bridge any
        opening between two collinear tracks. Now it is the observations of
        the last extent_decay_s (the newest always kept, so a coasting track
        keeps what it was last seen as), projected onto the CURRENT line, and
        of those only the run of intervals that touches the newest one with
        no OPEN gap wider than assoc_max_gap_m (_gap_between): an end nobody
        has re-observed for extent_decay_s goes, and two pieces with an
        opening between them are never one extent.
        """
        ext = trk.obs_ext
        if not ext:
            return
        cutoff = (ext[-1][0] if now is None else float(now)) - float(self.p.extent_decay_s)
        while len(ext) > 1 and ext[0][0] < cutoff:
            ext.popleft()
        newest = ext[-1][1]
        m = _along(trk.theta, newest)
        lo, hi = float(m.min()), float(m.max())
        spans = sorted((float(v.min()), float(v.max()))
                       for v in (_along(trk.theta, pts) for _s, pts in ext))
        gap = float(self.p.assoc_max_gap_m)
        grew = True
        while grew:
            grew = False
            for a, b in spans:
                if (a < lo or b > hi) and self._gap_between(
                        trk.theta, trk.rho, lo, hi, a, b) <= gap:
                    lo, hi, grew = min(lo, a), max(hi, b), True
        trk.t0, trk.t1 = lo, hi

    def _update(self, trk, line, stamp):
        v = _innovation(trk.x, line.rho, line.alpha)
        R = self._meas_cov(line)
        S = trk.P + R
        K = trk.P @ np.linalg.inv(S)
        trk.x = trk.x + K @ v
        trk.P = (np.eye(2) - K) @ trk.P
        rho, theta = normalise_line(float(trk.x[0]), float(trk.x[1]))
        trk.x = np.array([rho, theta])
        # The extent is rebuilt from the odom endpoints themselves, so a sign
        # flip of the line (rho < 0 -> theta + pi) needs no mirroring here.
        trk.obs_ext.append((stamp, np.vstack([line.p_start, line.p_end]).astype(float)))
        self._refresh_extent(trk, stamp)
        trk.hits += 1
        trk.last_seen = stamp
        trk.odo_at_last_seen = self._odometer

    def _spawn(self, line, stamp, obs=None):
        m = _along(line.alpha, np.vstack([line.p_start, line.p_end]))
        trk = Track(self._next_id, np.array([line.rho, line.alpha], dtype=float),
                    self._meas_cov(line), float(m.min()), float(m.max()), stamp, stamp)
        trk.odo_at_last_seen = self._odometer
        # The window is the LONGER of the two: a track that turns out to be
        # stereo-only must not have already dropped the scans it needs.
        trk.history = collections.deque([True], maxlen=self._history_len())
        # Seeded exactly like `history`, and with the same window: a camera
        # cycle that was fresh enough to BIRTH the track was fresh enough to
        # count, and a fresh_hist one entry short of history would never
        # catch up inside stereo_confirm_window.
        trk.fresh_hist = collections.deque(
            [self.camera_fresh or not self.p.camera_confirm_fresh_only
             or (obs is not None and obs.source != wall_fusion.SOURCE_STEREO)],
            maxlen=self._history_len())
        trk.obs_ext.append((stamp, np.vstack([line.p_start, line.p_end]).astype(float)))
        trk.lidar_hist = collections.deque(
            [obs is None or obs.source != wall_fusion.SOURCE_STEREO],
            maxlen=max(1, int(self.p.front_lidar_recent_cycles)))
        trk.conf_hist = collections.deque(maxlen=max(1, int(self.p.confirm_window)))
        if obs is not None:
            self._absorb(trk, obs)
        self._next_id += 1
        self.tracks.append(trk)

    def _history_len(self):
        return max(self.p.confirm_window, self.p.stereo_confirm_window)

    def _merge(self):
        """Fold every pair of tracks that are ONE wall into the older one.

        One wall means the same line (chi2 on (rho, theta)) AND a collinear
        gap of at most assoc_max_gap_m between the measured extents: two
        faces either side of a corridor mouth are one line and two walls, and
        merging them is what put track 955's 13.67 m front wall across an
        open doorway (corridor10m_20261016T144537).
        """
        refused = set()
        merged = True
        while merged:
            merged = False
            for a, b in itertools.combinations(sorted(self.tracks, key=lambda t: t.id), 2):
                v = _innovation(a.x, b.rho, b.theta)
                S = a.P + b.P
                if float(v @ np.linalg.solve(S, v)) > self.p.gate_chi2:
                    continue
                gap = self._extent_gap(a, *b.endpoints())
                if gap > self.p.assoc_max_gap_m:
                    if (a.id, b.id) not in refused:
                        refused.add((a.id, b.id))
                        self.n_merge_gap_refused += 1
                        self._reject('gap_merge', a.id, gap)
                    continue
                info_a, info_b = np.linalg.inv(a.P), np.linalg.inv(b.P)
                P = np.linalg.inv(info_a + info_b)
                x = P @ (info_a @ a.x + info_b @ (a.x + v))
                rho, theta = normalise_line(float(x[0]), float(x[1]))
                a.x, a.P = np.array([rho, theta]), P
                a.obs_ext = collections.deque(
                    sorted(list(a.obs_ext) + list(b.obs_ext), key=lambda e: e[0]),
                    maxlen=a.obs_ext.maxlen)
                self._refresh_extent(a, self._last_stamp)
                a.hits += b.hits
                a.last_seen = max(a.last_seen, b.last_seen)
                a.odo_at_last_seen = max(a.odo_at_last_seen, b.odo_at_last_seen)
                a.born = min(a.born, b.born)
                # MAX, not min: a track spawned this cycle has miss 0, and min
                # reset the merged phantom's negative-evidence count every
                # cycle a second piece was seen, so five in a row never came.
                a.miss = max(a.miss, b.miss)
                a.strip_miss = max(a.strip_miss, b.strip_miss)
                a.contradicted = a.contradicted or b.contradicted
                a.rect_reject_streak = max(a.rect_reject_streak, b.rect_reject_streak)
                a.verify = max(a.verify, b.verify) if not (
                    a.verify == wall_fusion.VERIFY_REJECTED
                    or b.verify == wall_fusion.VERIFY_REJECTED
                ) else wall_fusion.VERIFY_REJECTED
                a.glass = a.glass or b.glass
                a.ever_confirmed = a.ever_confirmed or b.ever_confirmed
                a.has_lidar = a.has_lidar or b.has_lidar
                a.has_both = a.has_both or b.has_both
                a.source = max(a.source, b.source, key=_SOURCE_RANK.get)
                # Newest-aligned: the histories end on the same cycle, so they
                # are OR-ed from the newest entry back (zip over the reversed
                # deques), in the full window the confirm rule needs.
                a.history = collections.deque(reversed(
                    [x or y for x, y in itertools.zip_longest(
                        reversed(a.history), reversed(b.history), fillvalue=False)]),
                    maxlen=self._history_len())
                a.lidar_hist = collections.deque(reversed(
                    [x or y for x, y in itertools.zip_longest(
                        reversed(a.lidar_hist), reversed(b.lidar_hist), fillvalue=False)]),
                    maxlen=a.lidar_hist.maxlen)
                a.conf_hist.extend(b.conf_hist)
                self.tracks.remove(b)
                merged = True
                break

    # ---- one scan ---------------------------------------------------------

    def step(self, observations, stamp, car_pose, scan=None, defer_roles=False):
        """Update with this cycle's OBSERVATIONS (base_link). Published tracks out.

        The tracker's input is exactly what goes out on
        /perception/wall_observations -- nothing else reaches it. It no longer
        fits lines to the scan itself, so the stereo can no longer slip in as
        a fallback distance behind its back.

        `scan`, when given, is (ranges, base_link bearings, laser xy[, cut])
        from scan_in_base_link -- the SAME beams the extraction saw, object
        sectors and all, plus the mask of the beams those sectors cut. It is
        EVIDENCE only, never fitted: the whole-span negative evidence, and the
        travel strip's nearest return (strip_returns), which every track that
        meets the strip is then judged against (_apply_strip_evidence).

        `defer_roles` leaves the roles alone, for a caller that is about to
        fold in this cycle's rectangle verdicts and would otherwise pay for
        the role and occlusion pass twice.
        """
        stamp = float(stamp)
        self._scan_ctx = None if scan is None else (car_pose, scan)
        if self._last_car_t is not None:
            self._odometer += float(np.hypot(*(car_pose.t - self._last_car_t)))
        self._last_car_t = car_pose.t
        # The span gate, before anything persistent is built on an observation
        # (module docstring, WHY THE SPAN GATE IS HYSTERETIC). The span is the
        # observation's own extent: its inliers projected onto its line, which
        # is what WallLine.span is and what the extraction's collinear merge
        # has already reunified across occluders.
        kept, may_birth = [], []
        self.n_short_dropped = 0
        self.n_stereo_near_rejected = 0
        self.n_camera_rejected = 0
        self.n_gap_rejected = 0
        self.n_merge_gap_refused = 0
        self.rejections = []
        for obs in observations:
            span = float(obs.line.span)
            if span < self.p.min_span_update_m:
                self.n_short_dropped += 1
                continue
            # THE NEAR FIELD. A stereo-only line passing nearer base_link than
            # stereo_min_rho_m runs through the car's own footprint: it is the
            # chassis the camera sees in the lower right of its image, not a
            # wall (23 throwaway 'right' tracks at rho 0.20 m on
            # corridor10m_20261016T144537). Only STEREO: a LiDAR line that
            # close is a real surface touching the car, and must stay.
            if (obs.source == wall_fusion.SOURCE_STEREO
                    and abs(float(obs.line.d)) < self.p.stereo_min_rho_m):
                self.n_stereo_near_rejected += 1
                self._reject('stereo_near', None, abs(float(obs.line.d)))
                continue
            if obs.source == wall_fusion.SOURCE_STEREO:
                # THE CAMERA QUALITY GATES (Q2). Nothing cross-checks a
                # stereo-only line, so it must look like a wall on its own:
                # tightly fitted, tall enough to be a surface rather than a
                # kerb, and long enough that the LiDAR could not plausibly
                # have missed it.
                if span < self.p.stereo_min_length_m:
                    self.n_camera_rejected += 1
                    self._reject('stereo_short', None, span)
                    continue
                if float(obs.line.rms) > self.p.stereo_max_rms_m:
                    self.n_camera_rejected += 1
                    self._reject('stereo_rms', None, float(obs.line.rms))
                    continue
                height = float(obs.z_max) - float(obs.z_min)
                if obs.height_measured and height < self.p.stereo_min_vertical_extent_m:
                    self.n_camera_rejected += 1
                    self._reject('stereo_flat', None, height)
                    continue
            if obs.source != wall_fusion.SOURCE_LIDAR:
                # THE RADIAL GATE (Q2), on every observation the camera had a
                # hand in. Judged from where the CAMERA looks, not the laser.
                origin = self.camera_xy if self.camera_xy is not None else \
                    (self.laser_xy if self.laser_xy is not None else (0.0, 0.0))
                keep, ray, bspan = radial_verdict(
                    obs.line.endpoints(), origin,
                    self.p.camera_seg_min_ray_angle_rad,
                    self.p.camera_seg_min_bearing_span_rad)
                if not keep:
                    self.n_camera_rejected += 1
                    self._reject('camera_radial', None, math.degrees(ray))
                    continue
            kept.append(obs)
            may_birth.append(span >= self.p.min_span_birth_m)
        observations = kept
        lines_odom = [observation_line(o, car_pose, self.p) for o in observations]
        dt = 0.0 if self._last_stamp is None else stamp - self._last_stamp
        self._last_stamp = stamp
        last_yaw, self._last_yaw = self._last_yaw, car_pose.theta
        self._predict(dt, 0.0 if last_yaw is None else wrap_pi(car_pose.theta - last_yaw))

        used_lines, hit_ids, lidar_ids = set(), set(), set()
        gap_refused = {}   # observation j -> (track id, gap) it was refused by
        if self.tracks and lines_odom:
            cost = np.full((len(self.tracks), len(lines_odom)), math.inf)
            for i, trk in enumerate(self.tracks):
                for j, ln in enumerate(lines_odom):
                    cost[i, j], why = self._gate(trk, ln, may_birth[j])
                    if why == 'gap' and j not in gap_refused:
                        gap_refused[j] = (trk.id, self._extent_gap(trk, ln.p_start, ln.p_end))
            finite = np.isfinite(cost)
            if finite.any():
                big = cost[finite].max() * 10.0 + 1e6
                rows, cols = linear_sum_assignment(np.where(finite, cost, big))
                for i, j in zip(rows, cols):
                    if finite[i, j]:
                        if self.tracks[i].glass:
                            # A pane: the stereo's distance is the room BEHIND
                            # it, so it must not pull the fused one. The gate
                            # above already passed on the combined covariance;
                            # only the update is LiDAR-only, which is the
                            # narrowest place this can be true.
                            lines_odom[j].cov = observation_covariance(
                                wall_fusion.SOURCE_LIDAR, observations[j].line.d,
                                observations[j].line.span, self.p)
                        self._update(self.tracks[i], lines_odom[j], stamp)
                        self._absorb(self.tracks[i], observations[j])
                        used_lines.add(j)
                        hit_ids.add(self.tracks[i].id)
                        if observations[j].source != wall_fusion.SOURCE_STEREO:
                            lidar_ids.add(self.tracks[i].id)
        for j, (tid, gap) in gap_refused.items():
            # Counted only when the gap is WHY the observation stayed
            # unassociated: it matched a track's line and not its extent.
            if j not in used_lines:
                self.n_gap_rejected += 1
                self._reject('gap_assoc', tid, gap)
        for trk in self.tracks:
            hit = trk.id in hit_ids
            trk.history.append(hit)
            trk.lidar_hist.append(trk.id in lidar_ids)
            # A camera-only hit counts towards CONFIRMATION only on a cycle
            # whose camera data was fresh: the same stale profile repeated
            # over three cycles is one observation, not three. The state
            # machine still runs on `hit`, so a track does not start coasting
            # merely because the camera was late.
            trk.fresh_hist.append(
                hit and (trk.id in lidar_ids or not self.p.camera_confirm_fresh_only
                         or self.camera_fresh))
            if self._confirmed_by_history(trk):
                trk.ever_confirmed = True
            if trk.ever_confirmed:
                trk.state = STATE_CONFIRMED if hit else STATE_COASTING
            else:
                trk.state = STATE_TENTATIVE
            if not hit:
                # The decay runs on every track, not only on the updated ones.
                self._refresh_extent(trk, stamp)
        for j, line in enumerate(lines_odom):
            # Only a segment with real extent may START a wall. A shorter one
            # that matched nothing is simply not evidence of a new wall: it is
            # dropped here, not held as a tentative track that a later long
            # segment could confirm.
            if j not in used_lines and may_birth[j]:
                self._spawn(line, stamp, observations[j])

        self.n_negative_killed = 0
        # One base_link segment per track for the whole cycle: the negative
        # evidence and the roles both need it, and it is a Line transform.
        segments = {t.id: segment_in_base_link(t, car_pose) for t in self.tracks}
        if scan is not None:
            self._apply_negative_evidence(car_pose, scan, segments)
        self.tracks = [t for t in self.tracks if not self._expired(t, stamp)]
        self._merge()
        self._apply_strip_evidence(car_pose, scan)
        if defer_roles:
            # The caller decides the roles itself and asks for the published
            # set afterwards, so ranking them here would be the second of two
            # identical sorts.
            return []
        self.update_roles(car_pose)
        return self.published(car_pose)

    # ---- the travel strip's own beams --------------------------------------

    def _apply_strip_evidence(self, car_pose, scan):
        """Judge every confirmed track that meets the strip by the strip's beams.

        The nearest return in the travel strip (strip_returns) against the
        distance at which the strip meets the track's MEASURED segment
        (strip_distance). front_contradict_cycles consecutive CONTRADICTED
        cycles make the track CONTRADICTED; it then is not FRONT, has the role
        'other', and no longer occludes what is behind it, so the wall the
        beams actually reach can be FRONT on the same cycle. Only a SUPPORTED
        cycle (the beams land on the line again) clears it.

        The track is not deleted: deleting a FRONT track used to latch the
        virtual wall at -1 (NO_DATA), which holds the car -- the incident's
        failure in another form. GLASS IS NOT EXEMPT, unlike the whole-span
        rule: a pane the LiDAR sees through in the strip and a doorway look
        the same to the rectangle (the scene beyond reads as 'beyond'), and
        only one of them is open (decisions: the cost is a pane read as
        CLEAR, which front_clearance and IsProximityTooClose still see).
        """
        if scan is None:
            self.strip = StripReading()
            for trk in self.tracks:
                trk.strip_verdict, trk.strip_pred_m = '', math.nan
            return
        half = self.p.strip_half_width_m
        self.strip = strip_returns(scan, self.p.front_x_m, half)
        for trk in self.tracks:
            pred = strip_distance(segment_in_base_link(trk, car_pose), self.p.front_x_m, half)
            trk.strip_pred_m = pred
            verdict = strip_verdict(pred, self.strip.range_m, self.p.front_contradict_margin_m)
            trk.strip_verdict = verdict if trk.ever_confirmed else ''
            if not trk.ever_confirmed:
                continue
            if verdict == STRIP_CONTRADICTED:
                trk.strip_miss += 1
                if trk.strip_miss >= int(self.p.front_contradict_cycles):
                    trk.contradicted = True
            else:
                trk.strip_miss = 0
                if verdict == STRIP_SUPPORTED:
                    trk.contradicted = False

    # ---- negative evidence ------------------------------------------------

    def negative_evidence(self, trk, car_pose, scan, seg=None):
        """(contradicted, n_pass, n_hit) for one track against one scan.

        Over the beams whose bearing falls inside the track's own angular
        span, shrunk by negative_shrink_beams at each end -- the ends are
        where the beam grazes the wall and where the span itself is least
        certain, and a grazing beam that misses proves nothing.

        For each of those beams, s is the range at which it would meet the
        track's line. The beam either PASSES (its return is more than the
        tolerance beyond s, so the beam went through where the wall should be
        and found something further away or nothing at all) or HITS (within
        the tolerance of s). A beam that returns SHORT of s is blocked by
        something nearer and says nothing about this wall.

        Contradicted when there are at least negative_min_beams opinions and
        more than negative_pass_frac of them passed. A no-return beam counts
        as a PASS: on this LiDAR, at these ranges, a wall returns. A CUT beam
        (scan[3], the object sectors) has no opinion at all: it is inf like a
        no-return, and counting it as a pass made a person standing in front
        of a wall evidence that the wall was not there.
        """
        ranges, bearings, laser_xy = scan[0], scan[1], scan[2]
        cut = scan[3] if len(scan) > 3 else None
        r = np.asarray(ranges, dtype=float)
        b = np.asarray(bearings, dtype=float)
        p0, p1 = segment_in_base_link(trk, car_pose) if seg is None else seg
        lxy = np.asarray(laser_xy, dtype=float).reshape(2)
        b0 = math.atan2(p0[1] - lxy[1], p0[0] - lxy[0])
        b1 = math.atan2(p1[1] - lxy[1], p1[0] - lxy[0])
        half = 0.5 * abs(wrap_pi(b1 - b0))
        centre = wrap_pi(b0 + 0.5 * wrap_pi(b1 - b0))
        # The bearings of a single scan are monotonic, so the span is a
        # SLICE found by two binary searches rather than a comparison over
        # every beam: three full-length 1081-element arrays per track per
        # scan was most of this rule's cost.
        if b.size > 1 and b[-1] > b[0]:
            lo, hi = np.searchsorted(b, (centre - half, centre + half))
            inside = np.arange(lo, hi)
        else:
            inside = np.flatnonzero(np.abs(wrap_pi(b - centre)) <= half)
        k = int(self.p.negative_shrink_beams)
        if inside.size <= 2 * k:
            return False, 0, 0
        inside = inside[k:inside.size - k]
        theta_bl = trk.theta - car_pose.theta
        n = np.array([math.cos(theta_bl), math.sin(theta_bl)])
        # The line in base_link through the segment's own endpoints, so the
        # track's extent and its distance are the same object here.
        d = float(n @ np.asarray(p0, dtype=float))
        bv = b[inside]
        inc = n[0] * np.cos(bv) + n[1] * np.sin(bv)
        usable = np.abs(inc) > 0.05
        if not usable.any():
            return False, 0, 0
        s = (d - float(n @ lxy)) / inc[usable]
        rv = r[inside][usable]
        ahead = s > 0.05
        if cut is not None:
            ahead = ahead & ~np.asarray(cut, dtype=bool)[inside][usable]
        tol = float(self.p.negative_tol_m)
        with np.errstate(invalid='ignore'):
            hit = ahead & (np.abs(rv - s) <= tol)
            # A no-return beam is a PASS: at these ranges a wall returns.
            passed = ahead & ~hit & (~np.isfinite(rv) | (rv > s + tol))
        n_pass, n_hit = int(np.count_nonzero(passed)), int(np.count_nonzero(hit))
        total = n_pass + n_hit
        if total < int(self.p.negative_min_beams):
            return False, n_pass, n_hit
        return n_pass / total > float(self.p.negative_pass_frac), n_pass, n_hit

    def _apply_negative_evidence(self, car_pose, scan, segments=None):
        """Count consecutive contradicted scans; kill at negative_miss_to_kill.

        CONFIRMED tracks only, and never a GLASS one: glass is the surface
        the beams are SUPPOSED to pass through, so the rule that kills a
        phantom would kill every pane in the building.
        """
        for trk in self.tracks:
            if not trk.ever_confirmed or trk.glass:
                trk.miss = 0
                continue
            contradicted, _n_pass, _n_hit = self.negative_evidence(
                trk, car_pose, scan, None if segments is None else segments.get(trk.id))
            trk.miss = trk.miss + 1 if contradicted else 0
        killed = [t for t in self.tracks if t.miss >= int(self.p.negative_miss_to_kill)]
        if killed:
            self.n_negative_killed = len(killed)
            self.tracks = [t for t in self.tracks if t not in killed]

    # ---- first obstacle ---------------------------------------------------

    def update_occlusion(self, car_pose, segments=None):
        """Mark every track that lies behind a CONFIRMED wall (first obstacle).

        A LiDAR beam reports its FIRST return, so a line fitted behind a wall
        the same scan also saw cannot be a surface that scan observed: it is
        the far side of the room showing through a gap, or a reflection. Five
        points along the segment, and if at least occlusion_min_blocked of
        them are behind a confirmed wall the track is not a reference wall --
        its role becomes 'other'. It is still tracked and still drawn.

        GLASS COUNTS AS A BLOCKER. A pane is a real surface that really does
        stop the LiDAR, which is exactly why it has a track at all, so a wall
        behind glass is behind a wall.

        ONLY THE PAIRS WHOSE BEARING SECTORS OVERLAP are tested. Occlusion is
        a statement about one bearing: a wall to the left cannot hide a wall
        to the right, however far apart they are. Comparing every track with
        every confirmed one is 72 ray tests a cycle on this bag against about
        a dozen that can possibly intersect, and the sectors are one cheap
        interval test each from geometry that is already in hand.
        """
        # A CONTRADICTED track is not a blocker: the strip's beams went
        # straight through it, so it cannot hide the wall they reached.
        confirmed = [t for t in self.tracks if t.ever_confirmed and not t.contradicted]
        if segments is None:
            segments = {t.id: segment_in_base_link(t, car_pose) for t in self.tracks}
        laser_xy = (0.0, 0.0) if self.laser_xy is None else self.laser_xy
        n_pts = max(2, int(self.p.occlusion_points))
        sectors = {tid: _bearing_sector(seg, laser_xy) for tid, seg in segments.items()}
        for trk in self.tracks:
            mine = sectors[trk.id]
            blockers = [segments[t.id] for t in confirmed if t.id != trk.id
                        and _sectors_overlap(mine, sectors[t.id])]
            if not blockers:
                trk.occluded = False
                continue
            a, b = segments[trk.id]
            f = np.linspace(0.0, 1.0, n_pts)[:, None]
            pts = np.asarray(a, dtype=float) + f * (np.asarray(b, dtype=float)
                                                    - np.asarray(a, dtype=float))
            blocked = ray_blocked_by(pts, laser_xy, blockers,
                                     self.p.occlusion_margin_m)
            trk.occluded = int(np.count_nonzero(blocked)) >= int(
                self.p.occlusion_min_blocked)

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
        # A track the LiDAR has never seen is confirmed on the FRESH-cycle
        # history; one the LiDAR has seen keeps the ordinary one.
        hist = trk.history if trk.has_lidar else trk.fresh_hist
        return sum(itertools.islice(reversed(hist), window)) >= hits

    def _expired(self, trk, stamp):
        if stamp - trk.last_seen > self.p.delete_after_s:
            return True
        if self._odometer - trk.odo_at_last_seen > self.p.coast_max_odom_m:
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
        """The ever-confirmed tracks that fit the cap, BY ROLE.

        WHY NOT BY SCORE. The cap used to rank by length * hits / distance,
        which sounds like importance and is not: a far wall dead ahead is
        SHORT and FAR, so that ranking puts it below the long near side walls
        unless its hit count is large -- and hits grow with node UPTIME, not
        with evidence. On corridor10m_20261015T171753 that hid a confirmed
        front wall behind the cap on 36 of 206 replayed cycles, at rank 4 of 7
        tracks held, 15.5 m dead ahead; the recorded run only reported it
        because its track was 435 s old. A cap is a budget, and a budget is
        spent on roles, not on a score.

        THE ORDER: every FRONT wall first, nearest along the heading, because
        the front wall is the one that can stop the car and it must never be
        the thing that drops off the end of a list. Then the nearest LEFT and
        the nearest RIGHT -- one slot each, measured from the bodywork, which
        is what a corridor needs to stay centred in. Then BACK. Then any
        remaining side walls, so the budget is never left unspent.

        'OTHER' NEVER TAKES A SLOT. It is the role for a wall that is a
        reference for nothing (update_roles): not in the strip, not beside the
        car, rejected by the stereo rectangle, or behind another wall. It is
        still tracked, still drawn and still an occluder, and it is on
        /perception/virtual_wall/debug's `tracks` list when
        wall_debug_tracks is on -- it is simply not one of the four walls this
        node asserts to the rest of the stack.
        """
        live = [t for t in self.tracks if t.ever_confirmed]
        buckets = {'front': [], 'left': [], 'right': [], 'back': []}
        for trk in live:
            role = trk.role or geometric_role(trk, car_pose, self.p)
            if role in buckets:
                buckets[role].append(trk)

        def along_heading(trk):
            # Where the travel strip meets the measured segment, as FRONT
            # itself is measured; the perpendicular distance when it does not.
            d = strip_distance(segment_in_base_link(trk, car_pose), self.p.front_x_m,
                               self.p.strip_half_width_m)
            if math.isfinite(d):
                return d
            n = np.array([math.cos(trk.theta), math.sin(trk.theta)])
            return abs(trk.rho - float(n @ car_pose.t))

        def from_the_side(trk):
            return side_distance(trk, car_pose, self.p.side_edge_half_width_m)

        def perpendicular(trk):
            n = np.array([math.cos(trk.theta), math.sin(trk.theta)])
            return abs(trk.rho - float(n @ car_pose.t))

        buckets['front'].sort(key=along_heading)
        buckets['left'].sort(key=from_the_side)
        buckets['right'].sort(key=from_the_side)
        buckets['back'].sort(key=perpendicular)
        order = list(buckets['front'])
        order += buckets['left'][:1] + buckets['right'][:1]
        order += buckets['back']
        order += buckets['left'][1:] + buckets['right'][1:]
        return order[:self.p.max_published]

    def apply_rect_verdicts(self, verdicts):
        """Fold one rectangle verdict per track into its sticky verify state.

        `verdicts` maps a track id to a wall_fusion.verify_rectangles entry.
        A track with no entry -- the rectangle did not run this cycle, or the
        track was born after it did -- keeps the state it had and is marked
        not-informative for this cycle, which is what stops it being called
        "both" on evidence it did not get.
        """
        for trk in self.tracks:
            verdict = verdicts.get(trk.id)
            if verdict is None:
                trk.rect_informative = False
                continue
            trk.rect_coverage = float(verdict['coverage'])
            trk.rect_support = float(verdict['support'])
            trk.rect_beyond_share = float(verdict['beyond_share'])
            trk.rect_informative = trk.rect_coverage >= self.rect.coverage_min
            trk.verify, trk.glass, trk.rect_reject_streak = wall_fusion.verify_step(
                trk.verify, trk.glass, verdict, self.rect, trk.rect_reject_streak)

    def update_roles(self, car_pose):
        """Give every track its role: front, back, left, right or OTHER.

        Three ways to be 'other', and all three mean the same thing -- the
        track is real, tracked and drawn, and is not a reference wall:

          * the geometry says so (geometric_role): it is not in the car's
            strip and not beside the car with a lateral normal.
          * the rectangle REJECTED it (E2): the stereo says an object, or a
            nearer surface, is there instead.
          * it is behind a confirmed wall (update_occlusion): a beam reports
            its first return, so a line behind a wall the same scan saw is
            not a surface that scan observed.
          * the travel strip's own beams CONTRADICTED it
            (_apply_strip_evidence): they pass through where it should be.

        The hysteresis is on the geometric test only: the other two are
        already sticky in their own right.
        """
        # The base_link segments are computed ONCE here and shared with every
        # test below: the strip test, the side test, their widened repeats and
        # the occlusion rays all used to take their own, which is five
        # transforms per track per cycle for one piece of geometry.
        segments = {t.id: segment_in_base_link(t, car_pose) for t in self.tracks}
        self.update_occlusion(car_pose, segments)
        for trk in self.tracks:
            if (trk.verify == wall_fusion.VERIFY_REJECTED or trk.occluded
                    or trk.contradicted):
                trk.role = 'other'
                continue
            trk.role = role_with_hysteresis(trk, car_pose, trk.role, self.p,
                                            segments[trk.id])

    def label(self, trk, car_pose):
        """The track's role. Sticky once update_roles has run (step 4)."""
        return trk.role or geometric_role(trk, car_pose, self.p)

    def coasting(self, trk):
        """Was trk left unassociated on the last step?"""
        return self._last_stamp is not None and trk.last_seen < self._last_stamp

    def coast_expired(self, trk):
        """True once trk has coasted past front_max_coast_s OR front_max_coast_m.

        Time alone was the old limit (0.3 s); a coasting wall is a PREDICTION,
        and how far the car drove on it matters as much as how long.
        """
        if not self.coasting(trk):
            return False
        return (self._last_stamp - trk.last_seen > self.p.front_max_coast_s
                or self._odometer - trk.odo_at_last_seen > self.p.front_max_coast_m)

    def track(self, track_id):
        """The live track with this id, or None (deleted or merged away)."""
        for trk in self.tracks:
            if trk.id == track_id:
                return trk
        return None

    def front_confidence(self, trk):
        """The track's confidence [0, 1] for the FRONT gate.

        Track.confidence used to be the LAST observation's and nothing read
        it. This is the mean of the last confirm_window matched observations'
        wall_fusion.confidence -- one poor fit does not flip FRONT -- raised to
        the rectangle's support when the rectangle VERIFIED the track (support
        >= support_verify, 0.5, by the verify rule). On that scale a LiDAR
        wall with a few dozen tight inliers is 0.6-0.9 and a stereo-only line
        is capped at 0.35 by design, so front_min_confidence 0.5 means: LiDAR
        quality, or the stereo verified over the wall's whole rectangle.
        """
        conf = (sum(trk.conf_hist) / len(trk.conf_hist)) if trk.conf_hist \
            else float(trk.confidence)
        if trk.verify == wall_fusion.VERIFY_VERIFIED:
            conf = max(conf, float(trk.rect_support))
        return conf

    def front_reason(self, trk, car_pose, front_x=None):
        """Why this track may NOT be FRONT, or '' when it may.

        The codes, in the order they are checked:

          unconfirmed        never confirmed: a maybe must never stop the car.
          contradicted       the strip's own beams passed through it for
                             front_contradict_cycles cycles.
          role_other         rect REJECTED, behind another wall, or nowhere
                             near the car (update_roles).
          off_strip          its MEASURED segment does not meet the travel
                             strip ahead of the bumper.
          coast_expired      unassociated for more than front_max_coast_s or
                             front_max_coast_m of odometry.
          coast_contradicted unassociated, and this cycle's strip has a
                             return BEYOND it: a coast is only for a strip
                             that says nothing (no returns -- glass, cut,
                             range -- or a nearer surface) or agrees.
          low_confidence     front_confidence below front_min_confidence.
          unverified         no LiDAR-backed association in the last
                             front_lidar_recent_cycles cycles and not rect
                             VERIFIED: nothing has measured it lately.
          stereo_oblique     stereo-only and not squarely across the path.
        """
        front_x = self.p.front_x_m if front_x is None else float(front_x)
        if not trk.ever_confirmed:
            return 'unconfirmed'
        if trk.contradicted:
            return 'contradicted'
        # 'other' is the one role that is never a reference wall: the
        # rectangle rejected it, or it is behind another wall, or the geometry
        # puts it nowhere near the car's path (update_roles).
        if trk.role == 'other':
            return 'role_other'
        d = strip_distance(segment_in_base_link(trk, car_pose), front_x,
                           self.p.strip_half_width_m)
        if not math.isfinite(d):
            return 'off_strip'
        if self.coasting(trk):
            if self.coast_expired(trk):
                return 'coast_expired'
            if trk.strip_verdict == STRIP_CONTRADICTED:
                return 'coast_contradicted'
        if self.front_confidence(trk) < self.p.front_min_confidence:
            return 'low_confidence'
        if not any(trk.lidar_hist) and trk.verify != wall_fusion.VERIFY_VERIFIED:
            return 'unverified'
        if trk.source == wall_fusion.SOURCE_STEREO:
            n = np.array([math.cos(trk.theta), math.sin(trk.theta)])
            to_wall = trk.theta if float(n @ car_pose.t) <= trk.rho else trk.theta + math.pi
            if abs(wrap_pi(to_wall - car_pose.theta)) > self.p.stereo_front_max_normal_rad:
                return 'stereo_oblique'
        return ''

    def is_front(self, trk, car_pose, front_x):
        """Whether this track may be called FRONT -- the one wall in the way.

        See front_reason for the full gate (WHAT FRONT REQUIRES in the module
        docstring). The second check is the one the open corridor needed: the
        MEASURED segment must meet the strip the car sweeps, not "its normal
        points forwards" -- on an open corridor the side wall's normal is
        within 45 deg of the heading for most of a turn, and reading it as the
        front wall is what put a phantom wall at 1.1 m (corridor_phantom,
        2026-10-15).
        """
        return not self.front_reason(trk, car_pose, front_x)

    def front_track(self, car_pose, front_x):
        """The published FRONT track the travel strip meets nearest, or None.

        Fills front_status: the chosen id and its strip_distance, or else the
        reason of the NEAREST track that meets the strip and was refused (or
        'no_candidate'), and every refused strip track's code.
        """
        front_x = float(front_x)
        half = self.p.strip_half_width_m
        rejects, nearest_reject = {}, (math.inf, 'no_candidate')
        for trk in self.tracks:
            if not trk.ever_confirmed:
                continue
            d = strip_distance(segment_in_base_link(trk, car_pose), front_x, half)
            if not math.isfinite(d):
                continue
            code = self.front_reason(trk, car_pose, front_x)
            if code:
                rejects[trk.id] = code
                if d < nearest_reject[0]:
                    nearest_reject = (d, code)
        best, best_d = None, math.inf
        for trk in self.published(car_pose):
            if trk.id in rejects or not self.is_front(trk, car_pose, front_x):
                continue
            d = strip_distance(segment_in_base_link(trk, car_pose), front_x, half)
            if d < best_d:
                best, best_d = trk, d
        self.front_status = {
            'id': None if best is None else best.id,
            'reason': 'front' if best is not None else nearest_reject[1],
            'distance_m': best_d if best is not None else math.nan,
            'rejects': rejects}
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
