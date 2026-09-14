"""Glass wall/door detection from one LiDAR scan, and the temporal track.

Pure logic, no rclpy: glass_detector_node.py is the ROS glue, and every
number that node publishes is decided here, so it can be unit-tested and
replayed against archived scans without a running graph (the same split
lidar_front_wall.py / lidar_front_wall_node.py and f1tenth_costmap's
costmap_boundary.py / costmap_boundary_node.py already use).

WHY GLASS NEEDS ITS OWN DETECTOR
--------------------------------
At oblique incidence a pane transmits ~92% of 905 nm light and reflects the
rest specularly away from the receiver, so the beam reports whatever stands
BEHIND the pane. Only within roughly +-0.2-0.4 deg of normal incidence does
the ~4% Fresnel reflection couple back into the aperture. A standard obstacle
layer therefore marks the pane on the rare near-normal beam and then clears
it on every subsequent scan that passes straight through: the pane can never
persist. Everything below follows from that, and so does the one hard rule at
the costmap end -- glass is published as a marking-only observation, never as
something a ray may clear.

THE HONEST SCAN IS NEVER TOUCHED. Nothing here writes to /scan. Synthetic
returns injected into the scan the matcher consumes would corrupt every
pose-graph constraint built from it. SLAM gets the raw scan; navigation gets
this module's output on a separate topic.

WHAT THIS RIG ACTUALLY DOES -- MEASURED 2026-09-14, NOT ASSUMED
---------------------------------------------------------------
The sensor is a Hokuyo UST-10LX (device B2128130, 1081 beams over 270 deg at
0.25 deg, 40 Hz, laser 0.12 m ahead of and 0.20 m above base_link). Four
measurements decide this design, and two of them contradict the theory above.

  1. INTENSITY IS AVAILABLE. The driver reports "Connected to network device
     with intensity"; with publish_intensity: true, intensities[] carries
     1081 values with 1475 distinct levels over 0-1805 counts. A missed beam
     reads exactly 0.0.
  2. INTENSITY IS NOT RAW AMPLITUDE. Over 195,591 valid returns a log-log fit
     of intensity against range gives slope -0.24 (corr -0.30), where raw
     amplitude would give -2.0, and the per-band medians are not monotonic.
     It is AGC-normalized or quantized. See normalize_intensity: this is why
     the r^2 correction defaults to OFF.
  3. A NO RETURN IS 65.533 m, NOT inf OR NaN. urg_node passes the sensor's
     65535 mm sentinel through unchanged while range_max reads 30.0. Across
     200 scans, 23.8% of beams carry 65.533 and ZERO carry inf or NaN. A void
     detector keyed on isinf() finds nothing here, which is why
     no_return_mask() tests the range WINDOW.
  4. THIS BUILDING'S CLEAR GLASS IS MOSTLY VISIBLE TO THIS SENSOR. Parked
     1.10 m (perpendicular, laser frame) from a clear pane, with a 4 m
     corridor and a second pane behind it, the near pane returned on
     essentially every forward beam out to 60 DEG of incidence -- valid
     fraction 1.000 / 1.000 / 0.934 / 0.975 / 1.000 / 0.928 for the 0-5,
     10-20, 20-30, 30-40, 40-50 and 50-60 deg bands, with median ranges
     1.10 / 1.14 / 1.23 / 1.35 / 1.57 / 1.88 m, a textbook 1.10/cos(theta)
     plane. The theory above predicts invisibility beyond about 0.4 deg.
     Short range (1.1 m leaves an enormous signal budget) plus real-world
     surface contamination is the likely reason. IT HAS NOT BEEN RETESTED AT
     3-4 m, WHERE THE BUDGET IS ~13x SMALLER AND THE PANE MAY WELL VANISH.
     Treat the numbers above as valid at ~1 m only.

     Consequence: NO FRESNEL SPIKE EXISTS TO FIND on that pane. Peak/median
     normalized intensity within +-20 deg was 1.84 against a spike_factor of
     6.0, and intensity falls SMOOTHLY with incidence (1153 counts at 0-2 deg
     to 793 at 15-20 deg), which is diffuse behaviour. The spike path below
     is implemented and correct, and it is what a genuinely specular pane
     needs, but on this glass at this range it does not fire and is not what
     the detector rests on.

SO WHAT IS THE ACTUAL HAZARD HERE? PARTIAL TRANSPARENCY.
--------------------------------------------------------
The same capture showed narrow bands of beams that persistently report
straight PAST the pane: beams 451-458 (-22.3 to -20.5 deg, 8 beams) read
4.64 m in every scan where their immediate neighbours read 1.21 m, and three
narrower bands (3-4 beams, at -26, -16 and -9.5 deg) do the same. 4.64 m is
the SECOND pane across the 4 m corridor. Those beams went through the first
one.

That is the whole problem in miniature, and it is worse than a pane that is
merely unseen. A costmap ray-traces free space along every beam, so roughly
3% of forward beams assert that the cells the pane occupies are empty, and
clearing is aggressive: those few rays punch holes straight through a wall
that the other 97% of beams had correctly marked, and they also mark a
phantom obstacle at 4.64 m on the far side of it. The pane does not fail to
appear; it appears and is then carved open.

Hence the detector's primary evidence, and the one validated against real
glass, is SEE-THROUGH: a run of beams reporting persistently far beyond a
line that the returns flanking it lie on.

THE OPEN-DOORWAY PROBLEM -- READ THIS BEFORE LOOSENING ANY GATE
---------------------------------------------------------------
A doorway produces the same shape as a see-through run: beams reporting the
far scene, bounded by frame returns. Persistence does NOT separate them -- a
doorway persists exactly as well as a pane, so raising persistence_hits buys
confidence that the OPENING is real, never that it is GLASS.

What separates them is GAP WIDTH AGAINST SURFACE EXTENT, and it separates
them cleanly on the measured data. Transparency shows up as a few centimetres
of gap inside metres of continuous surface: the widest see-through band above
is 8 beams, which at 1.2 m is 4 cm, while the pane itself spans about 3.5 m.
A doorway is the opposite -- 0.9 m of gap, which at 1.2 m is 43 deg or 172
beams. So a see-through run counts as transparency only while it stays under
see_through_max_gap_m; anything wider is an opening and is refused.

EVIDENCE_WIDE IS NOT SUFFICIENT ON ITS OWN, and an earlier version of this
module was wrong to treat it so. Extent plus support describes every ordinary
wall that happens to have a door in it, so accepting on extent alone would
mark every doorway in the building as lethal and strand the planner.
Acceptance requires actual transparency evidence -- SEE_THROUGH, ON_LINE or
SPIKE -- and WIDE only reports partition scale alongside it.

An earlier version also accepted `extent >= L OR n_voids >= 2`, which on 200
live scans produced six false positives 0.31-0.64 m long: frame returns from
up to five unrelated openings scattered across the 270 deg field of view were
collinear within the 5 cm inlier distance. Openings on opposite sides of a
room are not the mullions of one partition. Counting only openings whose
frame returns fall INSIDE the contiguous extent, and requiring that extent to
exceed a door's width, removed all six.

WHAT REMAINS UNDETECTABLE. A fully specular pane with nothing within range
behind it returns nothing anywhere and transmits into empty space: no frame
returns, no far returns, no spike. There is no signal of any kind, and no
gate setting reaches it. That case needs a different sensor, not tuning.

FRAMES
------
Returns are unprojected in base_link (x forward, y left) using the laser's
static pose, then placed in odom with the car pose the scan was taken at.
Candidate lines are fitted in odom and tracks are held in odom -- a wall is
fixed in the world, and odom is the frame the corridor, the MPC state and
the costmap boundary consumer already share.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Verdicts. A rejected candidate reports the FIRST gate it failed, in the
# order the gates are applied, so a log shows what the fit landed on and why
# it was refused. Mirrors f1tenth_messages/GlassSegment.msg's REASON_*.
# ---------------------------------------------------------------------------
REASON_ACCEPTED = 0
REASON_TOO_FEW_INLIERS = 1
REASON_TOO_SHORT = 2
REASON_TOO_LONG = 3
REASON_NO_EVIDENCE = 4
REASON_SPIKE_OFF_LINE = 5
REASON_SPIKE_OFF_NORMAL = 6
REASON_TOO_NEAR = 7
REASON_NAMES = {
    REASON_ACCEPTED: 'accepted',
    REASON_TOO_FEW_INLIERS: 'too_few_inliers',
    REASON_TOO_SHORT: 'too_short',
    REASON_TOO_LONG: 'too_long',
    REASON_NO_EVIDENCE: 'no_evidence',
    REASON_SPIKE_OFF_LINE: 'spike_off_line',
    REASON_SPIKE_OFF_NORMAL: 'spike_off_normal',
    REASON_TOO_NEAR: 'too_near',
}

# Evidence that made a candidate acceptable. Mirrors GlassSegment.msg.
EVIDENCE_NONE = 0
EVIDENCE_SPIKE = 1        # intensity spike on the line at near-normal incidence
EVIDENCE_ON_LINE = 2      # range returns on the line inside an opening
EVIDENCE_WIDE = 4         # partition-scale extent (reported, never sufficient)
EVIDENCE_SEE_THROUGH = 8  # a narrow run reporting far past a line its flanks lie on
EVIDENCE_NAMES = {
    EVIDENCE_SPIKE: 'spike', EVIDENCE_ON_LINE: 'on_line', EVIDENCE_WIDE: 'wide',
    EVIDENCE_SEE_THROUGH: 'see_through'}
# Only these prove transparency. EVIDENCE_WIDE describes every wall with a
# door in it, so it can never stand alone -- see the module docstring.
EVIDENCE_TRANSPARENCY = EVIDENCE_SPIKE | EVIDENCE_ON_LINE | EVIDENCE_SEE_THROUGH

# Void kinds.
VOID_NO_RETURN = 'no_return'   # the beams came back with nothing at all
VOID_FAR = 'far'               # the beams came back from behind the plane

# A hypothesis line through two points closer together than this is dominated
# by range noise in its direction (2.8 mm median residual on this Hokuyo).
_MIN_HYPOTHESIS_BASELINE_M = 0.05
# Openings whose projections along a shared line are further apart than this
# belong to different surfaces even when collinear, so they do not combine
# into one partition. A mullion is a few cm; this allows a generous one.
_MULLION_MAX_GAP_M = 0.6


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


# ===========================================================================
# Part 1 -- the scan: returns, voids, frames, intensity
# ===========================================================================

def no_return_mask(ranges: np.ndarray, range_min: float, range_max: float) -> np.ndarray:
    """True where a beam carries no usable return.

    Tests the range WINDOW, not finiteness: this sensor reports a miss as
    65.533 m (its 65535 mm sentinel) with range_max 30.0, and never as inf
    or NaN. See the module docstring. Non-finite values are caught too, for
    a driver or a bag that does use inf.
    """
    with np.errstate(invalid='ignore'):
        return ~(np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max))


def intensity_usable(intensities: Optional[Sequence[float]], n_ranges: int) -> bool:
    """Whether intensity can drive the spike test: present, the right length,
    and actually varying. A constant array is a driver publishing a filler
    value, which is indistinguishable from no information."""
    if intensities is None or len(intensities) != n_ranges or n_ranges == 0:
        return False
    values = np.asarray(intensities, dtype=np.float64)
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return False
    return float(finite.max() - finite.min()) > 0.0


def incidence_cosine(ranges: np.ndarray, angles: np.ndarray, valid: np.ndarray,
                     window: int, floor: float) -> np.ndarray:
    """cos(theta) between each beam and the local surface normal.

    The local surface tangent is the CHORD between the returns half a window
    either side of each beam, and the normal is its perpendicular; cos(theta)
    is that normal against the beam direction. Clamped at `floor` because the
    estimate degenerates at a depth edge, where the chord straddles two
    surfaces and means nothing -- an unclamped division there manufactures a
    spike out of an edge, which is the one artefact the spike test must not
    chase.

    A CHORD, NOT A PER-BEAM SVD, FOR SPEED. Fitting a line to each beam's
    neighbourhood is one SVD per beam, ~965 of them on this sensor, and
    measured at 145 ms per scan -- 580% of a core at 40 Hz, i.e. unusable.
    The chord is a single vectorized pass and is a perfectly adequate tangent
    estimate at this window size; what it costs is accuracy on a tightly
    curved surface, where the incidence correction is a rough figure anyway.
    """
    n = ranges.size
    half = max(int(window) // 2, 1)
    x = np.where(valid, ranges * np.cos(angles), np.nan)
    y = np.where(valid, ranges * np.sin(angles), np.nan)
    # Pad rather than roll: this is a 270 deg sweep, so wrapping would join
    # the two ends of the arc into a fictitious surface.
    pad = ((half, half),)
    xp = np.pad(x, pad, constant_values=np.nan)
    yp = np.pad(y, pad, constant_values=np.nan)
    tx = xp[2 * half:] - xp[:n]
    ty = yp[2 * half:] - yp[:n]
    length = np.hypot(tx, ty)
    with np.errstate(invalid='ignore', divide='ignore'):
        # normal = (-ty, tx) / |t|; cos(theta) = |normal . beam|
        cos_t = np.abs(-ty * np.cos(angles) + tx * np.sin(angles)) / length
    return np.clip(np.where(np.isfinite(cos_t), cos_t, 1.0), floor, 1.0)


def normalize_intensity(intensities: np.ndarray, ranges: np.ndarray,
                        cos_theta: np.ndarray, gain_lut=None,
                        range_exponent: float = 0.0) -> np.ndarray:
    """rho_hat = intensity * range^range_exponent / (gain(range) * cos(theta)).

    range_exponent DEFAULTS TO 0, NOT 2 -- MEASURED, NOT ASSUMED. The textbook
    correction for a raw-amplitude sensor is intensity * r^2, undoing the
    1/r^2 fall-off. This sensor does not have one. Over 195,591 valid returns
    from 200 live scans (2026-09-14, office scene) a log-log fit of intensity
    against range gave SLOPE -0.240, CORRELATION -0.302, and the per-band
    medians are not even monotonic: 594 counts at 0.5-1 m, 1131 at 1-2 m, 663
    at 2-4 m, 485 at 4-6 m, 609 at 6-9 m. Intensity here is dominated by
    surface reflectivity, not by range -- an AGC-normalized or quantized
    quality figure, which is exactly the case Section 0.6 of the work order
    asked to be documented rather than worked around.

    Applying r^2 to data with a true slope of -0.24 would leave an effective
    +1.76: a 6 m return would read roughly thirty times brighter than a 1 m
    one, and the spike test would fire on every distant surface in the room.
    So the exponent is a parameter with a measured default of 0, and raising
    it to 2 is only correct on a sensor whose own fall-off has been measured
    to be 1/r^2.

    The cos(theta) correction is kept on by default: the AGC compensates
    RANGE, and there is no evidence it compensates incidence angle, which is
    a physically independent effect. Neither correction has been validated
    against a real pane yet.
    """
    gain = np.ones_like(ranges) if gain_lut is None else np.asarray(gain_lut(ranges), float)
    gain = np.where(np.abs(gain) < 1e-9, 1.0, gain)
    scale = 1.0 if range_exponent == 0.0 else ranges ** range_exponent
    return intensities * scale / (gain * cos_theta)


@dataclass(frozen=True)
class Void:
    """One run of beams that missed a plane, with the returns bounding it.

    left/right are the indices of the bounding VALID returns -- the frame:
    the mullion, door jamb or pane edge the beams passed between. They are
    this detector's primary fitting support, because they are the only
    returns that come from the plane itself at oblique incidence.
    """

    start: int          # first index of the run
    end: int            # last index of the run, inclusive
    kind: str           # VOID_NO_RETURN or VOID_FAR
    left: int           # index of the valid return bounding on the low side
    right: int          # index of the valid return bounding on the high side

    @property
    def width_beams(self) -> int:
        return self.end - self.start + 1


def find_voids(ranges: np.ndarray, no_return: np.ndarray, min_void_beams: int,
               void_range_jump: float) -> List[Void]:
    """Every opening in the scan, of both kinds, bounded on BOTH sides by a
    valid return.

    VOID_NO_RETURN  a run of beams with no usable return.
    VOID_FAR        a run of VALID beams whose ranges all exceed BOTH
                    bounding returns by more than void_range_jump -- the
                    beams passed through the plane and came back from the
                    scene behind it. On a transparent surface with anything
                    at all behind it this is the COMMON case, more common
                    than a pure miss, and a detector that only looked for
                    missing returns would see nothing in a furnished room.

    Requiring a bound on both sides is what makes a void an opening IN
    something rather than the edge of the sensor's field of view or the far
    end of an empty corridor.
    """
    n = ranges.size
    valid_idx = np.flatnonzero(~no_return)
    if valid_idx.size < 2:
        return []
    voids: List[Void] = []

    # --- kind 1: runs of missing returns, between consecutive valid beams ---
    for a, b in zip(valid_idx, valid_idx[1:]):
        if b - a - 1 >= min_void_beams:
            voids.append(Void(a + 1, b - 1, VOID_NO_RETURN, int(a), int(b)))

    # --- kind 2: runs of valid-but-far returns ------------------------------
    # Walk the valid beams; a maximal stretch whose range exceeds both of its
    # own bounding valid beams by the jump is a see-through opening.
    i = 0
    while i < valid_idx.size - 1:
        a = valid_idx[i]
        j = i + 1
        while j < valid_idx.size and ranges[valid_idx[j]] > ranges[a] + void_range_jump:
            j += 1
        if j < valid_idx.size and j - i - 1 >= min_void_beams:
            b = valid_idx[j]
            run = valid_idx[i + 1:j]
            if np.all(ranges[run] > ranges[b] + void_range_jump):
                voids.append(Void(int(run[0]), int(run[-1]), VOID_FAR, int(a), int(b)))
            i = j
        else:
            i += 1
    return voids


def find_spikes(rho_hat: np.ndarray, valid: np.ndarray, spike_factor: float,
                spike_window: int, max_spike_width: int,
                min_gradient_ratio: float) -> np.ndarray:
    """Indices whose normalized intensity is an isolated bright spike.

    Two tests, and the second is what separates glass from retroreflective
    tape. A pane's spike is ISOLATED: it exists only within a fraction of a
    degree of normal incidence, so it collapses within a beam or two. Floor
    tape, a safety vest or a road sign is a CONTIGUOUS bright patch many
    beams wide. Amplitude alone cannot tell them apart -- the gradient to the
    immediate neighbours and the width of the bright run can.

    A ROLLING MEDIAN OVER A STRIDED VIEW, NOT A PER-BEAM LOOP. Taking the
    median of each beam's neighbourhood separately is one np.median call per
    valid beam -- ~965 of them, measured at 78 ms per scan, which was the
    single largest cost in the whole detector. One strided view and one
    nanmedian along its second axis give the identical answer.
    """
    n = rho_hat.size
    half = max(int(spike_window) // 2, 1)
    # Invalid beams become NaN so they are ignored by the median exactly as
    # the boolean mask used to exclude them; padding with NaN keeps the
    # window aligned at both ends without wrapping the 270 deg sweep round.
    masked = np.where(valid, rho_hat, np.nan)
    padded = np.pad(masked.astype(np.float64), (half, half), constant_values=np.nan)
    windows = np.lib.stride_tricks.sliding_window_view(padded, 2 * half + 1)
    # Only rows with data are reduced: an all-NaN neighbourhood has no median,
    # and asking for one raises a RuntimeWarning on every single scan.
    enough = np.count_nonzero(np.isfinite(windows), axis=1) >= 3
    median = np.full(n, np.nan)
    if enough.any():
        median[enough] = np.nanmedian(windows[enough], axis=1)
    with np.errstate(invalid='ignore'):
        bright = (valid & enough & np.isfinite(median) & (median > 0.0)
                  & (rho_hat > spike_factor * median))
    if not bright.any():
        return np.empty(0, dtype=int)

    # Reject bright RUNS longer than max_spike_width, and require a steep
    # gradient out of the run on at least one side.
    out: List[int] = []
    edges = np.flatnonzero(np.diff(bright.astype(np.int8)) != 0) + 1
    starts = np.concatenate(([0], edges))
    ends = np.concatenate((edges, [n]))
    for s, e in zip(starts, ends):
        if not bright[s]:
            continue
        if e - s > max_spike_width:
            continue
        peak = s + int(np.argmax(rho_hat[s:e]))
        neighbours = [k for k in (s - 1, e) if 0 <= k < n and valid[k] and rho_hat[k] > 0.0]
        if not neighbours:
            continue
        if max(rho_hat[peak] / rho_hat[k] for k in neighbours) >= min_gradient_ratio:
            out.append(peak)
    return np.asarray(out, dtype=int)


# ===========================================================================
# Part 2 -- the line
# ===========================================================================

@dataclass(frozen=True)
class Line:
    """n . p = c, |n| = 1, in whatever frame the points were given in."""

    nx: float
    ny: float
    c: float

    @property
    def normal(self) -> np.ndarray:
        return np.array([self.nx, self.ny])

    @property
    def tangent(self) -> np.ndarray:
        return np.array([-self.ny, self.nx])

    def distance(self, points: np.ndarray) -> np.ndarray:
        return np.abs(np.atleast_2d(points) @ self.normal - self.c)

    def signed(self, px: float, py: float) -> float:
        return self.nx * px + self.ny * py - self.c

    def project(self, points: np.ndarray) -> np.ndarray:
        return np.atleast_2d(points) @ self.tangent


def tls_line(points: np.ndarray) -> Line:
    """Total-least-squares line through `points`."""
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vt[-1]
    return Line(float(normal[0]), float(normal[1]), float(normal @ centroid))


def ransac_line(points: np.ndarray, inlier_dist: float, max_pairs: int = 400):
    """Best line through `points` by MSAC over two-point hypotheses, refined
    by two TLS refits on the inliers. Returns (Line, inlier_mask) or None.

    Deterministic: hypotheses are every pair, strided down to max_pairs in
    lexicographic order when there are more. The same scan must produce the
    same verdict in a replay and in a test.
    """
    n = len(points)
    if n < 2:
        return None
    i, j = np.triu_indices(n, 1)
    if i.size > max_pairs:
        step = -(-i.size // max_pairs)
        i, j = i[::step], j[::step]
    delta = points[j] - points[i]
    length = np.hypot(delta[:, 0], delta[:, 1])
    keep = length > _MIN_HYPOTHESIS_BASELINE_M
    if not np.any(keep):
        return None
    i, delta, length = i[keep], delta[keep], length[keep]
    normals = np.column_stack([-delta[:, 1], delta[:, 0]]) / length[:, None]
    offsets = np.einsum('ij,ij->i', normals, points[i])
    residuals = np.abs(points @ normals.T - offsets)
    # Truncated (MSAC) cost, not an inlier count: a tie between two surfaces
    # goes to the one the points sit tighter on.
    best = int(np.argmin(np.minimum(residuals, inlier_dist).sum(axis=0)))
    inliers = residuals[:, best] <= inlier_dist
    line = None
    for _ in range(2):
        if np.count_nonzero(inliers) < 2:
            return None
        line = tls_line(points[inliers])
        inliers = line.distance(points) <= inlier_dist
    if np.count_nonzero(inliers) < 2:
        return None
    return line, inliers


def contiguous_extent(values: np.ndarray, max_gap: float) -> Tuple[float, float]:
    """(lo, hi) of the longest run of `values` with no gap wider than
    max_gap. Used for a candidate's extent along its own line: points
    scattered along a room that happen to be collinear are not one wall."""
    s = np.sort(values)
    if s.size == 0:
        return 0.0, 0.0
    breaks = np.flatnonzero(np.diff(s) > max_gap)
    starts = np.concatenate(([0], breaks + 1))
    ends = np.concatenate((breaks, [s.size - 1]))
    k = int(np.argmax(s[ends] - s[starts]))
    return float(s[starts[k]]), float(s[ends[k]])


# ===========================================================================
# Part 3 -- one scan's candidates
# ===========================================================================

@dataclass(frozen=True)
class Candidate:
    line: Line
    p0: np.ndarray          # segment endpoints, on the line, odom frame
    p1: np.ndarray
    length_m: float
    inlier_count: int
    n_voids: int
    evidence: int           # OR of EVIDENCE_*
    reason: int             # REASON_ACCEPTED, or the first gate it failed
    rms_m: float
    spike_angle_err_rad: float   # NaN when no spike was tested

    @property
    def accepted(self) -> bool:
        return self.reason == REASON_ACCEPTED

    @property
    def midpoint(self) -> np.ndarray:
        return 0.5 * (self.p0 + self.p1)


@dataclass
class DetectorConfig:
    """Every gate, defaulted to stack_params.yaml's glass_* values."""

    cos_theta_floor: float = 0.2
    normal_window: int = 5
    gain_lut = None
    # 0, not 2 -- measured on this sensor; see normalize_intensity.
    intensity_range_exponent: float = 0.0
    spike_factor: float = 6.0
    spike_window: int = 31
    max_spike_width: int = 3
    min_gradient_ratio: float = 3.0
    # 3, not 5: the measured see-through bands on real glass were 8, 4, 3 and
    # 3 beams wide, so a floor of 5 caught only the widest of the four.
    min_void_beams: int = 3
    void_range_jump: float = 0.5
    # Beams either side of an opening whose on-surface returns join the line
    # fit. This pane returns on ~97% of forward beams, so there is abundant
    # support next to each gap; fitting the two frame beams alone (which is
    # all a fully invisible pane would offer) throws that away and yields a
    # two-point line through noise.
    support_window_beams: int = 60
    # NOTE this bounds how long a fitted segment can be, and therefore when
    # EVIDENCE_WIDE can fire: one opening's support reaches +-15 deg, which at
    # 1.2 m is only +-0.32 m of wall, well under geom_min_length_m. That is
    # deliberate and not a bug -- a wider window starts pulling an adjoining
    # wall across a corner into the fit. WIDE therefore needs either a
    # partition with SEVERAL openings, whose support windows union into a long
    # extent (which is what the real capture shows: 2.07 m from five
    # openings), or a pane far enough away that 15 deg is already metres.
    # A see-through run wider than this at the pane is an OPENING, not
    # transparency. Measured: real transparency bands were 4 cm; a 0.9 m
    # doorway at the same range is 0.9 m. See the module docstring.
    see_through_max_gap_m: float = 0.30
    min_line_inliers: int = 6
    line_inlier_dist: float = 0.05
    min_segment_length: float = 0.3
    max_segment_length: float = 6.0
    # Nearer than this and it is the car's own structure or near-field
    # clutter, not a partition worth constraining the planner with. The
    # LiDAR sits 0.12 m ahead of base_link and the body reaches 0.443 m.
    min_range_m: float = 0.8
    normal_tolerance_deg: float = 2.0
    on_line_min_beams: int = 2
    geom_min_length_m: float = 1.2
    geom_min_voids: int = 2
    max_candidates: int = 4
    point_spacing: float = 0.025


def scan_points(ranges: np.ndarray, angles: np.ndarray, laser_pose, car_pose):
    """Endpoints of every beam in odom, whether the return was valid or not
    (an invalid beam's point is meaningless but keeps indices aligned)."""
    lx, ly, lyaw = laser_pose
    cx, cy, cyaw = car_pose
    bx = lx + ranges * np.cos(angles + lyaw)
    by = ly + ranges * np.sin(angles + lyaw)
    c, s = math.cos(cyaw), math.sin(cyaw)
    return np.column_stack([cx + c * bx - s * by, cy + s * bx + c * by])


def detect(ranges, angle_min, angle_increment, range_min, range_max,
           laser_pose, car_pose, cfg: DetectorConfig,
           intensities=None) -> Tuple[List[Candidate], bool]:
    """One scan's candidate glass segments, plus whether the intensity path
    was used.

    Steps, in the order the module docstring sets out: classify returns,
    normalize intensity if usable, find openings, take their frame returns as
    support, fit lines, then verify each against the available evidence.
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    n = ranges.size
    angles = angle_min + np.arange(n) * angle_increment
    miss = no_return_mask(ranges, range_min, range_max)
    valid = ~miss
    pts = scan_points(ranges, angles, laser_pose, car_pose)

    use_intensity = intensity_usable(intensities, n)
    spikes = np.empty(0, dtype=int)
    if use_intensity:
        cos_theta = incidence_cosine(
            ranges, angles + laser_pose[2], valid, cfg.normal_window, cfg.cos_theta_floor)
        rho_hat = normalize_intensity(
            np.asarray(intensities, dtype=np.float64), ranges, cos_theta, cfg.gain_lut,
            cfg.intensity_range_exponent)
        spikes = find_spikes(rho_hat, valid, cfg.spike_factor, cfg.spike_window,
                             cfg.max_spike_width, cfg.min_gradient_ratio)

    voids = find_voids(ranges, miss, cfg.min_void_beams, cfg.void_range_jump)
    if not voids:
        return [], use_intensity

    # SUPPORT = the frame returns bounding every opening, PLUS the
    # on-surface returns flanking them, and never a return from inside any
    # opening (those come from whatever lies behind the plane). Pooling
    # across openings is what lets one fitted line span a whole partition
    # rather than yielding a two-point line per gap; including the flanks is
    # what makes the fit robust on a pane that does return at oblique
    # incidence, which the measured one does on ~97% of forward beams.
    interior = np.zeros(n, dtype=bool)
    for v in voids:
        interior[v.start:v.end + 1] = True
    eligible = valid & ~interior
    support_beams = set()
    for v in voids:
        lo = max(v.left - cfg.support_window_beams, 0)
        hi = min(v.right + cfg.support_window_beams + 1, n)
        support_beams.update(int(k) for k in np.flatnonzero(eligible[lo:hi]) + lo)
    frame_idx = sorted(support_beams)
    if len(frame_idx) < 2:
        return [], use_intensity
    support = pts[frame_idx]

    candidates: List[Candidate] = []
    remaining = np.ones(len(support), dtype=bool)
    for _ in range(cfg.max_candidates):
        if np.count_nonzero(remaining) < 2:
            break
        subset = support[remaining]
        fit = ransac_line(subset, cfg.line_inlier_dist)
        if fit is None:
            break
        line, sub_mask = fit
        chosen = np.flatnonzero(remaining)[sub_mask]
        candidates.append(_verify(line, pts, chosen, frame_idx, voids, spikes, angles,
                                 ranges, valid, laser_pose, car_pose, cfg,
                                 angle_increment))
        remaining[chosen] = False
    return candidates, use_intensity


def _verify(line: Line, pts: np.ndarray, chosen: np.ndarray, frame_idx: List[int],
            voids: List[Void], spikes: np.ndarray, angles: np.ndarray,
            ranges: np.ndarray, valid: np.ndarray, laser_pose, car_pose,
            cfg: DetectorConfig, angle_increment: float) -> Candidate:
    """Gate one fitted line. See the module docstring for what each piece of
    evidence means and why geometry-only mode demands one of two of them."""
    # `chosen` indexes the pooled frame-return support, not the scan; map it
    # back to beam indices before touching `pts`.
    beams = np.asarray(frame_idx)[chosen]
    inliers = pts[beams]
    count = int(len(inliers))
    proj = line.project(inliers)
    lo, hi = contiguous_extent(proj, _MULLION_MAX_GAP_M)
    length = hi - lo
    rms = float(np.sqrt(np.mean((inliers @ line.normal - line.c) ** 2)))
    # Endpoints, put back ON the line rather than at the outermost inliers,
    # so the published segment is the fitted geometry and not a noisy sample.
    base = line.normal * line.c
    p0, p1 = base + lo * line.tangent, base + hi * line.tangent

    # Only the support INSIDE the contiguous extent counts, and only the
    # openings whose frame returns lie in it are this line's own -- see the
    # module docstring's measured note on why the looser version produced
    # six false positives from collinear clutter across the room.
    in_extent = (proj >= lo - 1e-9) & (proj <= hi + 1e-9)
    count_in_extent = int(np.count_nonzero(in_extent))
    beams_in_extent = {int(b) for b in beams[in_extent]}
    own_voids = [v for v in voids
                 if v.left in beams_in_extent or v.right in beams_in_extent]
    n_voids = len(own_voids)

    evidence = EVIDENCE_NONE
    spike_err = math.nan

    # --- evidence 1: an intensity spike where the geometry predicts one ----
    # A SPIKE CAN ONLY ADD EVIDENCE, NEVER VETO. An earlier version set the
    # rejection reason here whenever the scan held a spike that did not land
    # on THIS line, which meant a strip of retroreflective tape anywhere in
    # the room rejected every genuine pane in the scan -- it broke all four
    # real-glass assertions the moment the intensity pipeline started finding
    # any spikes at all. The spike codes below are diagnostics, chosen at the
    # end only when the spike was the sole candidate evidence.
    spike_on_line = False
    if spikes.size:
        on_line = [k for k in spikes if line.distance(pts[k])[0] <= cfg.line_inlier_dist]
        if on_line:
            spike_on_line = True
            # The ray to the spike must be near the line's own normal: that
            # is the only incidence at which a pane reflects back. A bright
            # return from elsewhere on the plane is a retroreflector.
            best = min(on_line, key=lambda k: _normal_error(line, pts[k], car_pose))
            spike_err = _normal_error(line, pts[best], car_pose)
            if spike_err <= math.radians(cfg.normal_tolerance_deg):
                evidence |= EVIDENCE_SPIKE

    # --- evidence 2: range returns ON the line inside an opening ----------
    # The near-normal Fresnel reflection puts a few beams back at the pane's
    # own distance. Only VOID_FAR openings can show it -- a run that returned
    # nothing returned nothing.
    on_line_beams = 0
    for v in own_voids:
        if v.kind != VOID_FAR:
            continue
        idx = np.arange(v.start, v.end + 1)
        idx = idx[valid[idx]]
        if idx.size:
            on_line_beams += int(np.count_nonzero(
                line.distance(pts[idx]) <= cfg.line_inlier_dist))
    if on_line_beams >= cfg.on_line_min_beams:
        evidence |= EVIDENCE_ON_LINE

    # --- evidence 3: SEE-THROUGH, the one validated against real glass ----
    # A narrow run whose interior returns all sit well BEYOND this line while
    # the returns flanking it sit ON it: those beams went through the surface.
    # Narrowness is the doorway discriminator -- measured transparency bands
    # were 4 cm wide against a pane spanning 3.5 m, where a doorway is 0.9 m.
    for v in own_voids:
        if v.kind != VOID_FAR:
            continue
        gap_m = float(np.hypot(*(pts[v.right] - pts[v.left])))
        if gap_m > cfg.see_through_max_gap_m:
            continue
        idx = np.arange(v.start, v.end + 1)
        idx = idx[valid[idx]]
        if idx.size == 0:
            continue
        # Beyond the line, on the far side from the car.
        car_side = math.copysign(1.0, line.signed(car_pose[0], car_pose[1]))
        beyond = -car_side * (pts[idx] @ line.normal - line.c)
        if np.all(beyond > cfg.void_range_jump):
            evidence |= EVIDENCE_SEE_THROUGH
            break

    # --- partition scale: REPORTED, never sufficient (see the docstring) --
    if (length >= cfg.geom_min_length_m
            and (n_voids >= cfg.geom_min_voids or count_in_extent >= 3)):
        evidence |= EVIDENCE_WIDE

    # Gates, in report order.
    stand_off = abs(line.signed(car_pose[0], car_pose[1]))
    reason = REASON_ACCEPTED
    if count < cfg.min_line_inliers:
        reason = REASON_TOO_FEW_INLIERS
    elif length < cfg.min_segment_length:
        reason = REASON_TOO_SHORT
    elif length > cfg.max_segment_length:
        reason = REASON_TOO_LONG
    elif stand_off < cfg.min_range_m:
        reason = REASON_TOO_NEAR
    elif not (evidence & EVIDENCE_TRANSPARENCY):
        # EVIDENCE_WIDE alone lands here, deliberately: it describes every
        # wall with a door in it. See the module docstring. The spike codes
        # are the more specific diagnosis when a spike was in play.
        if spike_on_line:
            reason = REASON_SPIKE_OFF_NORMAL
        elif spikes.size:
            reason = REASON_SPIKE_OFF_LINE
        else:
            reason = REASON_NO_EVIDENCE
    return Candidate(line, p0, p1, length, count, n_voids, evidence, reason, rms, spike_err)


def _normal_error(line: Line, point: np.ndarray, car_pose) -> float:
    """Angle between the ray from the sensor to `point` and the line's own
    normal, folded into [0, pi/2] because a fit gives the normal only up to
    sign."""
    ray = point - np.array([car_pose[0], car_pose[1]])
    err = abs(_wrap(math.atan2(ray[1], ray[0]) - math.atan2(line.ny, line.nx)))
    return min(err, math.pi - err)


# ===========================================================================
# Part 4 -- temporal persistence
# ===========================================================================

@dataclass
class Track:
    """A candidate segment accumulating confirmations across scans, in odom."""

    line: Line
    p0: np.ndarray
    p1: np.ndarray
    evidence: int
    hits: int = 0
    misses: int = 0
    last_seen: float = 0.0
    history: List[bool] = field(default_factory=list)

    @property
    def length_m(self) -> float:
        return float(np.hypot(*(self.p1 - self.p0)))

    def confirmed(self, window: int, hits: int) -> bool:
        return sum(self.history[-window:]) >= hits


class GlassTracker:
    """Confirms candidates across scans before anything is published.

    A segment is published only after being seen in at least
    persistence_hits of the last persistence_window scans. Matching is by
    endpoint proximity plus line angle, in odom.

    NEVER DECAY ON ABSENCE ALONE. Absence is the NORMAL state for glass: at
    any oblique angle the pane is invisible, so a track that is not
    re-observed is the expected case, not evidence against it. Confidence is
    decayed only when the segment sits inside the field of view AT AN ANGLE
    WHERE A RETURN WAS EXPECTED -- near normal incidence -- and still was not
    seen. A tracker that decayed on plain absence would drop every pane the
    moment the car turned away from it, which is exactly the clearing bug
    this whole design exists to prevent, reintroduced one layer up.
    """

    def __init__(self, *, persistence_window: int, persistence_hits: int,
                 match_endpoint_tol: float, match_angle_tol_deg: float,
                 expect_return_tol_deg: float, fov_half_angle_rad: float,
                 max_range_m: float, geometry_only_multiplier: int = 2):
        self.persistence_window = int(persistence_window)
        self.persistence_hits = int(persistence_hits)
        self.match_endpoint_tol = float(match_endpoint_tol)
        self.match_angle_tol = math.radians(float(match_angle_tol_deg))
        self.expect_return_tol = math.radians(float(expect_return_tol_deg))
        self.fov_half_angle_rad = float(fov_half_angle_rad)
        self.max_range_m = float(max_range_m)
        self.geometry_only_multiplier = int(geometry_only_multiplier)
        self.tracks: List[Track] = []

    def required_hits(self, use_intensity: bool) -> int:
        """Geometry-only mode needs more confirmations, because its evidence
        is weaker per scan -- but see the module docstring: this buys
        confidence that the opening is real, NOT that it is glass."""
        if use_intensity:
            return self.persistence_hits
        return self.persistence_hits * self.geometry_only_multiplier

    def _match(self, cand: Candidate) -> Optional[Track]:
        best, best_cost = None, math.inf
        cand_angle = math.atan2(cand.line.ny, cand.line.nx)
        for track in self.tracks:
            angle = math.atan2(track.line.ny, track.line.nx)
            err = abs(_wrap(angle - cand_angle))
            err = min(err, math.pi - err)
            if err > self.match_angle_tol:
                continue
            # Endpoints may be found in either order, and a partly-occluded
            # re-observation is shorter, so match on the closer pairing.
            direct = (np.hypot(*(track.p0 - cand.p0)) + np.hypot(*(track.p1 - cand.p1)))
            flipped = (np.hypot(*(track.p0 - cand.p1)) + np.hypot(*(track.p1 - cand.p0)))
            cost = 0.5 * min(direct, flipped)
            if cost <= self.match_endpoint_tol and cost < best_cost:
                best, best_cost = track, cost
        return best

    def _expected_return(self, track: Track, car_pose) -> bool:
        """Whether this track should have produced a return in this scan:
        inside the field of view, within range, and near enough to normal
        incidence for a pane to reflect. Only then does a miss count."""
        cx, cy, cyaw = car_pose
        mid = 0.5 * (track.p0 + track.p1)
        ray = mid - np.array([cx, cy])
        dist = float(np.hypot(*ray))
        if dist > self.max_range_m or dist < 1e-6:
            return False
        if abs(_wrap(math.atan2(ray[1], ray[0]) - cyaw)) > self.fov_half_angle_rad:
            return False
        return _normal_error(track.line, mid, car_pose) <= self.expect_return_tol

    def update(self, candidates: Sequence[Candidate], car_pose, now: float,
               use_intensity: bool) -> List[Track]:
        """Fold one scan's accepted candidates in; return the confirmed set."""
        accepted = [c for c in candidates if c.accepted]
        seen = set()
        for cand in accepted:
            track = self._match(cand)
            if track is None:
                track = Track(cand.line, cand.p0, cand.p1, cand.evidence)
                self.tracks.append(track)
            else:
                # Refresh the geometry and keep the longest extent seen: a
                # partly-occluded view of a known pane must not shrink it.
                track.line = cand.line
                if cand.length_m >= track.length_m:
                    track.p0, track.p1 = cand.p0, cand.p1
                track.evidence |= cand.evidence
            track.hits += 1
            track.last_seen = float(now)
            track.history.append(True)
            seen.add(id(track))

        for track in self.tracks:
            if id(track) in seen:
                continue
            if self._expected_return(track, car_pose):
                track.misses += 1
                track.history.append(False)
            # else: absence proves nothing -- history is not extended at all,
            # so the confirmation ratio is computed only over scans that
            # could actually have seen this pane.
            del track.history[:-self.persistence_window]

        need = self.required_hits(use_intensity)
        return [t for t in self.tracks if t.confirmed(self.persistence_window, need)]

    def drop_stale(self, now: float, timeout_sec: float) -> int:
        """Forget tracks not re-observed for timeout_sec. Non-positive
        timeout disables forgetting, which is the safe default for glass."""
        if timeout_sec <= 0.0:
            return 0
        before = len(self.tracks)
        self.tracks = [t for t in self.tracks if now - t.last_seen <= timeout_sec]
        return before - len(self.tracks)


def sample_segment(p0: np.ndarray, p1: np.ndarray, spacing: float) -> np.ndarray:
    """Points along a segment at `spacing`, endpoints included.

    Dense sampling is not cosmetic: a costmap marks CELLS, and a segment
    sampled coarser than the cell size leaves gaps a planner will happily
    thread a path through. Default spacing is half the costmap resolution.
    """
    length = float(np.hypot(*(p1 - p0)))
    if length < 1e-9:
        return np.atleast_2d(p0)
    count = max(int(math.ceil(length / max(spacing, 1e-6))) + 1, 2)
    t = np.linspace(0.0, 1.0, count)[:, None]
    return p0 + t * (p1 - p0)
