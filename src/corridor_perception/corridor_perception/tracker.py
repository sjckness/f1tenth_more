"""Wall tracking across scans, in the odom frame (Step 1).

Up to four walls -- left, right, front, back -- each a small EKF on its Hesse
form (rho, alpha) in odom. Nothing else is tracked here: no obstacles, no
mission objects, no controller interface. TrackerSnapshot is a debugging
record, not the thing a controller will consume.

One update, in order:

  1. predict   compound the frame's odometry step onto each track's own motion
               since its last correction (delta, delta_cov), and express the
               track in the current laser frame;
  2. extract   extract_segments on the frame's CleanRanges;
  3. axis      update the carried corridor axis, then classify every segment
               against it;
  4. associate anchored chi-square gate + support overlap + class match,
               nearest in Mahalanobis, one-to-one;
  5. correct   Kalman update of matched tracks, then reset their delta;
  6. lifecycle birth, confirmation, coasting, death.

WHY THE AXIS IS STATE. The x4 fold gives the grid orientation only modulo
90 degrees. Choosing the family per scan by support balance swaps the axis by
90 degrees mid-corridor on real data, and every FRONTAL label with it. Here the
axis is seeded once from a forward hint, then carried: each scan's fold is
resolved to the candidate nearest the carried axis, gated at a few degrees, and
filtered. In odom a corridor cannot rotate, so a large residual is always a bad
measurement and never a real change. seed() refuses to run twice.

WHY RELATIVE MOTION PER TRACK. Absolute pose covariance measures drift since
the start of the run, which is irrelevant: a track needs only how wrong the
odometry can have been since that track was last corrected. Each track
compounds that itself (process_noise.compound, O(1) per cycle) and resets it on
a correction. Summing per-step covariances instead drops the term where an
early heading error rotates every later translation.

WHY ANCHORED GATING. A distant line's origin-anchored rho variance is dominated
by t_c^2 var(alpha): its position extrapolated far along its own tangent. Near
where it was seen it is known far better. Gating and updating happen with both
lines re-expressed about the midpoint of their shared support, where the
observation's rho and alpha decorrelate and where the measurement-model floor
belongs. Storage and reporting stay origin-anchored. (To first order a 2-DOF
Mahalanobis distance is unchanged by any common re-anchoring; what anchoring
removes is the loose rho-marginal gate. test_tracker's anchored-gating test
shows both.)

WHY TWO WAYS TO DIE. A coasting track dies when its perpendicular uncertainty
from motion alone passes coast_sigma: faster driving coasts for less, and a
wall hidden behind something survives as long as the odometry can still place
it. Independently, a wall the sensor should see and does not -- beams aimed at
where it was last seen come back from beyond it, or not at all -- is counted
on its own miss counter and dies on that, whatever its sigma. "Hidden" is a
return NEARER than the predicted wall; "absent" is one farther or none. That
is a per-beam comparison of the scan against one identified track's
prediction, used for that track's lifecycle only; no range enters a
navigation decision.

numpy only; nothing here imports ROS.
"""

from __future__ import annotations

import copy
import logging
import math
from dataclasses import dataclass, field
from typing import Iterable, Literal

import numpy as np

from .axis import (AxisParams, SurfaceClass, _fold_error, classify, estimate_axis,
                   fold_orientation, nearest_candidate)
from .extraction import ExtractionParams, extract_segments, point_sigma
from .geometry import Line, Pose2D, angdiff, wrap_pi
from .process_noise import ProcessNoiseParams, compound
from .scan import require_clean

log = logging.getLogger(__name__)

WALL_CLASSES = (SurfaceClass.LATERAL_LEFT, SurfaceClass.LATERAL_RIGHT,
                SurfaceClass.FRONTAL, SurfaceClass.REAR)

Status = Literal['tentative', 'confirmed', 'coasting', 'dead']
LIVE = ('tentative', 'confirmed', 'coasting')
ESTABLISHED = ('confirmed', 'coasting')


@dataclass
class TrackerParams:
    """Every tunable the tracker has. Nothing else in tracker.py is a constant."""

    # Association (5.4)
    gate_chi2: float = 9.21                  # 2 DOF, 99 %
    min_overlap: float = 0.30                # m of support shared along the wall
    # Measurement-model floor, added at the anchor. The TLS covariance assumes
    # independent point noise, so it shrinks as 1/N; the sensor's systematic,
    # surface-dependent range error does not average away within a scan.
    meas_rho_floor: float = 0.01             # m
    meas_alpha_floor: float = math.radians(0.2)

    # Lifecycle (5.5)
    n_confirm: int = 5                       # consecutive hits, tentative -> confirmed
    coast_sigma: float = 0.05                # m, perpendicular, from delta_cov alone
    absent_misses: int = 8                   # expected-but-absent verdicts that kill
    absent_min_beams: int = 10               # fewer expected beams: not in view
    absent_fraction: float = 0.8             # share of expected beams seen through
    absent_k_sigma: float = 3.0              # range band, in sigma, that counts as the wall

    # Axis (5.1)
    axis_gate_deg: float = 3.0               # nearest candidate beyond this: rejected
    axis_meas_floor_deg: float = 0.5         # walls not quite square; 0.4 observed
    axis_reject_inflation_deg: float = 0.5   # added (as a variance) per rejection
    axis_valid_sigma_deg: float = 3.0        # beyond this the axis reports invalid
    axis_min_support: float = 2.0            # m of segment length for a measurement

    process: ProcessNoiseParams = field(default_factory=ProcessNoiseParams)
    extraction: ExtractionParams = field(default_factory=ExtractionParams)
    axis: AxisParams = field(default_factory=AxisParams)


@dataclass
class SurfaceTrack:
    id: int
    state: np.ndarray            # (rho, alpha) in odom
    cov: np.ndarray              # 2x2, origin-anchored in odom
    support: tuple[np.ndarray, np.ndarray]   # endpoints, odom: hull of all support seen
    cls: SurfaceClass
    status: Status
    hits: int
    misses: int
    age: float
    last_update: float
    delta: Pose2D                # motion since last correction
    delta_cov: np.ndarray        # 3x3, compounded
    # Beyond the brief's fields, each needed by one rule below.
    born: float = 0.0            # stamp of birth; age = now - born
    anchor: Pose2D = field(default_factory=Pose2D)   # base in odom at last correction
    seen: tuple[np.ndarray, np.ndarray] | None = None  # extent of the LAST observation, odom
    max_range: float = 0.0       # farthest this surface has returned from, m (evidence)
    absent: int = 0              # expected-but-absent verdicts since last evidence
    death: str = ''

    def line(self) -> Line:
        return Line(float(self.state[0]), float(self.state[1]), self.cov.copy(),
                    self.support[0].copy(), self.support[1].copy(), 0, 'odom')

    def sigma_perp(self) -> float:
        """Perpendicular sigma of this wall from motion since its last correction.

        delta_cov is in the base frame at the last correction; the wall's normal
        in that frame is its odom normal rotated by the anchor heading.
        """
        a = float(self.state[1]) - self.anchor.theta
        n = np.array([math.cos(a), math.sin(a)])
        return math.sqrt(max(float(n @ self.delta_cov[:2, :2] @ n), 0.0))

    def distance_from(self, pose: Pose2D) -> float:
        return self.line().distance_from(pose)


@dataclass
class CorridorAxis:
    theta: float                 # corridor direction in odom, "forward" end
    theta_var: float
    valid: bool
    seeded_at: float
    hint: float = 0.0            # the forward hint seed() was given
    resolved_at: float | None = None   # stamp of the scan that fixed the family
    rejected: int = 0            # measurements refused by the gate, in total


@dataclass
class AxisUpdate:
    measured: float | None       # fold g, modulo 90 deg
    candidate: float | None      # g + k*90 nearest the carried axis
    residual: float | None
    accepted: bool
    reason: str


@dataclass
class Association:
    track_id: int
    segment: int
    chi2: float
    overlap: float
    innovation: np.ndarray       # anchored (d_rho, d_alpha), observed - predicted


@dataclass
class Rejection:
    track_id: int
    segment: int
    reason: str                  # 'overlap' | 'gate' | 'taken'
    chi2: float
    overlap: float


@dataclass
class Visibility:
    track_id: int
    expected: int                # beams aimed at where the wall was last seen
    present: int                 # returned at the wall
    blocked: int                 # returned nearer: something in front
    through: int                 # returned from beyond it, or not at all
    verdict: str                 # 'unseen' | 'absent' | 'present' | 'hidden' | 'mixed'


@dataclass
class TrackEvent:
    stamp: float
    track_id: int
    kind: str                    # 'born' | 'confirmed' | 'coasting' | 'resumed' | 'dead'
    cls: SurfaceClass
    detail: str = ''


@dataclass
class TrackerSnapshot:
    """One cycle, for debugging and for tests. Not the controller interface."""

    stamp: float
    index: int
    pose: Pose2D                         # base in odom
    axis: CorridorAxis
    axis_update: AxisUpdate
    tracks: list[SurfaceTrack]           # live after this cycle (copies)
    segments: list[Line]                 # this scan's segments, odom
    classes: list[SurfaceClass]
    associations: list[Association]
    rejections: list[Rejection]
    visibility: list[Visibility]
    events: list[TrackEvent]

    def established(self, cls: SurfaceClass) -> SurfaceTrack | None:
        """The confirmed or coasting track of a class; there is at most one."""
        for t in self.tracks:
            if t.cls == cls and t.status in ESTABLISHED:
                return t
        return None

    def association(self, track_id: int) -> Association | None:
        for a in self.associations:
            if a.track_id == track_id:
                return a
        return None


# -- anchoring ---------------------------------------------------------------

def _tangent(alpha: float) -> np.ndarray:
    return np.array([-math.sin(alpha), math.cos(alpha)])


def anchor(vec, cov, m) -> tuple[np.ndarray, np.ndarray]:
    """(rho, alpha) about the frame origin -> (rho_m, alpha) about point m.

    rho_m = rho - n(alpha).m is the signed offset of the line from m, and is
    deliberately NOT normalised: at an anchor on the line it is ~0 and either
    sign is legitimate.
    """
    a = float(vec[1])
    m = np.asarray(m, dtype=float)
    J = np.array([[1.0, -float(_tangent(a) @ m)], [0.0, 1.0]])
    rho_m = float(vec[0]) - (math.cos(a) * m[0] + math.sin(a) * m[1])
    return np.array([rho_m, a]), J @ np.asarray(cov, dtype=float) @ J.T


def unanchor(vec, cov, m) -> tuple[np.ndarray, np.ndarray]:
    """Inverse of anchor(), normalised back to rho >= 0."""
    a = float(vec[1])
    m = np.asarray(m, dtype=float)
    J = np.array([[1.0, float(_tangent(a) @ m)], [0.0, 1.0]])
    rho = float(vec[0]) + (math.cos(a) * m[0] + math.sin(a) * m[1])
    cov = J @ np.asarray(cov, dtype=float) @ J.T
    if rho < 0.0:
        F = np.diag([-1.0, 1.0])
        return np.array([-rho, wrap_pi(a + math.pi)]), F @ cov @ F.T
    return np.array([rho, wrap_pi(a)]), cov


def _oriented_like(line: Line, alpha_ref: float) -> tuple[np.ndarray, np.ndarray]:
    """line's (rho, alpha) and cov, flipped to the antipodal form if its normal
    points away from alpha_ref. A surface seen from the other side of the
    origin is (-rho, alpha + pi) in the orientation its track uses."""
    if abs(angdiff(line.alpha, alpha_ref)) > math.pi / 2.0:
        F = np.diag([-1.0, 1.0])
        return (np.array([-line.rho, wrap_pi(line.alpha + math.pi)]),
                F @ np.asarray(line.cov, dtype=float) @ F.T)
    return line.as_vector(), np.asarray(line.cov, dtype=float)


def shared_midpoint(pred: Line, obs: Line) -> np.ndarray:
    """Midpoint of the two supports' overlap, placed on the observed line.

    For disjoint supports the same formula gives the middle of the gap.
    """
    tan = obs.tangent
    a = sorted((float(obs.p_start @ tan), float(obs.p_end @ tan)))
    b = sorted((float(pred.p_start @ tan), float(pred.p_end @ tan)))
    mid_t = 0.5 * (max(a[0], b[0]) + min(a[1], b[1]))
    return obs.rho * obs.normal + mid_t * tan


@dataclass
class AnchoredInnovation:
    m: np.ndarray                # anchor point
    x_pred: np.ndarray           # predicted (rho_m, alpha)
    P_pred: np.ndarray
    z: np.ndarray                # observed (rho_m, alpha), in the prediction's orientation
    R: np.ndarray                # observation covariance at m, floor included
    nu: np.ndarray
    S: np.ndarray
    chi2: float


def anchored_innovation(pred: Line, obs: Line, floor: np.ndarray,
                        m: np.ndarray | None = None) -> AnchoredInnovation:
    """Innovation of obs against pred, both re-expressed about their shared
    midpoint (or about m if given). floor is a 2x2 added to the observation
    covariance at the anchor."""
    m = shared_midpoint(pred, obs) if m is None else np.asarray(m, dtype=float)
    z0, R0 = _oriented_like(obs, pred.alpha)
    x, P = anchor(pred.as_vector(), pred.cov, m)
    z, R = anchor(z0, R0, m)
    R = R + floor
    nu = np.array([z[0] - x[0], angdiff(z[1], x[1])])
    S = P + R
    chi2 = float(nu @ np.linalg.solve(S, nu))
    return AnchoredInnovation(m, x, P, z, R, nu, S, chi2)


def origin_innovation(pred: Line, obs: Line) -> tuple[np.ndarray, np.ndarray]:
    """Innovation and its covariance about the frame origin, for comparison
    with anchored_innovation. Not used for gating."""
    z0, R0 = _oriented_like(obs, pred.alpha)
    nu = np.array([z0[0] - pred.rho, angdiff(z0[1], pred.alpha)])
    return nu, np.asarray(pred.cov, dtype=float) + R0


# -- the tracker ---------------------------------------------------------------

class CorridorTracker:
    def __init__(self, params: TrackerParams | None = None):
        self.p = params or TrackerParams()
        self.axis: CorridorAxis | None = None
        self.tracks: list[SurfaceTrack] = []      # live only
        self.events: list[TrackEvent] = []        # whole history
        self.seed_record: dict | None = None
        self._next_id = 1
        self._floor = np.diag([self.p.meas_rho_floor ** 2, self.p.meas_alpha_floor ** 2])

    # -- seeding -------------------------------------------------------------
    def seed(self, forward_hint: float, stamp: float) -> None:
        """Set the forward direction once, at corridor entry.

        The family and the exact angle are fixed by the first scan with enough
        structure (the candidate nearest this hint); from then on the axis is
        carried. Never call this again mid-corridor with the current heading:
        that is the original bug, which is why a second call raises.
        """
        if self.axis is not None:
            raise RuntimeError(
                'CorridorTracker.seed() called twice. The axis is seeded once at '
                'corridor entry and carried; re-seeding from the current heading '
                'is exactly the bug the carried axis removes.')
        hint = wrap_pi(forward_hint)
        self.axis = CorridorAxis(theta=hint, theta_var=(math.pi / 4.0) ** 2, valid=False,
                                 seeded_at=float(stamp), hint=hint)
        log.info('corridor axis seeded: forward hint %.2f deg at t=%.3f',
                 math.degrees(hint), stamp)

    # -- one cycle -----------------------------------------------------------
    def update(self, frame) -> TrackerSnapshot:
        if self.axis is None:
            raise RuntimeError('CorridorTracker.update() before seed()')
        p = self.p
        stamp = float(frame.stamp)

        # 1. Predict: motion since each track's last correction.
        q = p.process.step_cov(frame.ds, frame.dtheta)
        for t in self.tracks:
            t.delta, t.delta_cov = compound(t.delta, t.delta_cov, frame.odom_step, q)
            t.age = stamp - t.born
        self.axis.theta_var += float(q[2, 2])

        # 2. Extract.
        ranges = require_clean(frame.ranges, 'CorridorTracker.update')
        seg_l = extract_segments(frame.ranges, frame.angles, p.extraction)
        sensor = frame.odom_sensor
        seg_o = [s.transform(sensor, 'odom') for s in seg_l]

        # 3. Axis, then classes.
        axis_update = self._update_axis(seg_o, stamp)
        events: list[TrackEvent] = []
        if self.axis.resolved_at is None:
            return self._snapshot(frame, axis_update, seg_o, [], [], [], [], events)
        classes = [classify(s, self.axis, frame.odom, p.axis) for s in seg_o]

        # 4. Associate.
        preds = {t.id: self._predict(t, frame) for t in self.tracks}
        assoc, rejections, gated = self._associate(seg_l, classes, preds)

        # 5. Correct.
        matched = {a.track_id for a in assoc}
        by_id = {t.id: t for t in self.tracks}
        for a in assoc:
            self._correct(by_id[a.track_id], gated[a.track_id, a.segment],
                          seg_l[a.segment], frame, events)

        # 6. Lifecycle.
        visibility: list[Visibility] = []
        for t in self.tracks:
            if t.id in matched:
                continue
            if t.status == 'tentative':
                self._kill(t, 'tentative_miss', stamp, events)
                continue
            if t.status == 'confirmed':
                t.status = 'coasting'
                self._event(events, stamp, t, 'coasting')
            t.misses += 1
            v = self._visibility(t, preds[t.id], frame, ranges)
            visibility.append(v)
            if v.verdict == 'absent':
                t.absent += 1
            elif v.verdict == 'present':
                t.absent = 0
            sigma = t.sigma_perp()
            if t.absent >= p.absent_misses:
                self._kill(t, f'absent: {t.absent} expected-but-absent scans', stamp, events)
            elif sigma > p.coast_sigma:
                self._kill(t, f'coast_sigma: {sigma * 100:.1f} cm', stamp, events)

        for t in self.tracks:
            if t.status == 'tentative' and t.id in matched and t.hits >= p.n_confirm:
                self._confirm(t, stamp, events)

        taken = {a.segment for a in assoc}
        for j, (s, c) in enumerate(zip(seg_l, classes)):
            if j not in taken and c in WALL_CLASSES:
                self._birth(s, c, frame, events)

        self.tracks = [t for t in self.tracks if t.status != 'dead']
        return self._snapshot(frame, axis_update, seg_o, classes, assoc, rejections,
                              visibility, events)

    # -- axis ----------------------------------------------------------------
    def _update_axis(self, seg_o: list[Line], stamp: float) -> AxisUpdate:
        ax, p = self.axis, self.p
        fold = fold_orientation(seg_o, p.axis)
        if fold is None or fold.strength < p.axis.min_strength \
                or fold.support < p.axis_min_support:
            return AxisUpdate(None if fold is None else fold.g, None, None, False,
                              'no_structure')
        r = fold.variance + math.radians(p.axis_meas_floor_deg) ** 2

        if ax.resolved_at is None:
            theta0 = nearest_candidate(fold.g, ax.hint)
            # Support balance, at seed time only, as a sanity check on the hint.
            balance = estimate_axis(seg_o, None, p.axis)
            agrees = _fold_error(balance.theta, theta0) < math.pi / 4.0
            ax.theta, ax.theta_var, ax.resolved_at, ax.valid = theta0, r, stamp, True
            self.seed_record = dict(stamp=stamp, hint=ax.hint, fold_g=fold.g, theta=theta0,
                                    offset_from_hint=angdiff(theta0, ax.hint),
                                    support_balance_theta=balance.theta,
                                    agrees_with_support_balance=agrees,
                                    strength=fold.strength, support=fold.support)
            log.info('corridor axis resolved at t=%.3f: %.2f deg (hint %.2f, fold %.2f, '
                     'support-balance family %s)', stamp, math.degrees(theta0),
                     math.degrees(ax.hint), math.degrees(fold.g),
                     'agrees' if agrees else 'DISAGREES')
            if not agrees:
                log.warning('seed family disagrees with support balance: check the '
                            'forward hint (%.1f deg) against the corridor',
                            math.degrees(ax.hint))
            return AxisUpdate(fold.g, theta0, angdiff(theta0, ax.hint), True,
                              'seed' if agrees else 'seed_disagrees_with_support_balance')

        cand = nearest_candidate(fold.g, ax.theta)
        resid = angdiff(cand, ax.theta)
        if abs(resid) > math.radians(p.axis_gate_deg):
            ax.theta_var += math.radians(p.axis_reject_inflation_deg) ** 2
            ax.rejected += 1
            ax.valid = math.sqrt(ax.theta_var) <= math.radians(p.axis_valid_sigma_deg)
            return AxisUpdate(fold.g, cand, resid, False, 'gate')
        k = ax.theta_var / (ax.theta_var + r)
        ax.theta = wrap_pi(ax.theta + k * resid)
        ax.theta_var *= (1.0 - k)
        ax.valid = math.sqrt(ax.theta_var) <= math.radians(p.axis_valid_sigma_deg)
        return AxisUpdate(fold.g, cand, resid, True, 'update')

    # -- predict / associate / correct ---------------------------------------
    def _predict(self, t: SurfaceTrack, frame) -> Line:
        """The track in the current laser frame, motion uncertainty included.

        odom -> base is exact (it is the odom frame); what is uncertain is how
        far the odometry has drifted since the track was last corrected, which
        is delta_cov. With delta = (tx, ty, phi) mapping the current base into
        the base at that correction, a line (rho_k, alpha_k) there appears here
        as rho = rho_k - n_k . (tx, ty), alpha = alpha_k - phi.
        """
        lb = t.line().transform(frame.odom.inverse(), 'base')
        a_k = lb.alpha + t.delta.theta
        Jd = np.array([[-math.cos(a_k), -math.sin(a_k), 0.0],
                       [0.0, 0.0, -1.0]])
        lb.cov = lb.cov + Jd @ t.delta_cov @ Jd.T
        return lb.transform(frame.sensor_in_base.inverse(), 'laser')

    def _associate(self, seg_l, classes, preds):
        p = self.p
        rejections: list[Rejection] = []
        gated: dict[tuple[int, int], AnchoredInnovation] = {}
        cands = []
        for t in self.tracks:
            pred = preds[t.id]
            for j, (s, c) in enumerate(zip(seg_l, classes)):
                if c != t.cls:
                    continue
                ov = s.overlap(pred)
                ai = anchored_innovation(pred, s, self._floor)
                if ov < p.min_overlap:
                    rejections.append(Rejection(t.id, j, 'overlap', ai.chi2, ov))
                elif ai.chi2 > p.gate_chi2:
                    rejections.append(Rejection(t.id, j, 'gate', ai.chi2, ov))
                else:
                    gated[t.id, j] = ai
                    cands.append((ai.chi2, t.id, j, ov))
        cands.sort()
        used_t, used_s = set(), set()
        assoc: list[Association] = []
        for chi2, tid, j, ov in cands:
            if tid in used_t or j in used_s:
                rejections.append(Rejection(tid, j, 'taken', chi2, ov))
                continue
            used_t.add(tid)
            used_s.add(j)
            assoc.append(Association(tid, j, chi2, ov, gated[tid, j].nu.copy()))
        return assoc, rejections, gated

    def _correct(self, t: SurfaceTrack, ai: AnchoredInnovation, obs: Line, frame,
                 events: list[TrackEvent]) -> None:
        K = ai.P_pred @ np.linalg.inv(ai.S)
        x = ai.x_pred + K @ ai.nu
        IK = np.eye(2) - K
        P = IK @ ai.P_pred @ IK.T + K @ ai.R @ K.T          # Joseph form
        vec, cov = unanchor(x, P, ai.m)
        post = Line(float(vec[0]), float(vec[1]), cov).transform(frame.odom_sensor, 'odom')

        # Support: hull of what was known and what was just seen, on the new line.
        seen = frame.odom_sensor.transform_points(np.vstack([obs.p_start, obs.p_end]))
        foot, tan = post.rho * post.normal, post.tangent
        s_seen = (seen - foot) @ tan
        s_all = np.concatenate([s_seen, (np.vstack(t.support) - foot) @ tan])
        t.state = post.as_vector()
        t.cov = post.cov
        t.support = (foot + s_all.min() * tan, foot + s_all.max() * tan)
        t.seen = (foot + s_seen.min() * tan, foot + s_seen.max() * tan)
        t.max_range = max(t.max_range, float(np.linalg.norm(obs.p_start)),
                          float(np.linalg.norm(obs.p_end)))
        t.delta, t.delta_cov = Pose2D(), np.zeros((3, 3))
        t.anchor = frame.odom
        t.last_update = float(frame.stamp)
        t.hits += 1
        t.misses = 0
        t.absent = 0
        if t.status == 'coasting':
            t.status = 'confirmed'
            self._event(events, frame.stamp, t, 'resumed')

    # -- lifecycle -----------------------------------------------------------
    def _birth(self, s: Line, cls: SurfaceClass, frame, events) -> None:
        mid = s.midpoint
        vec, cov = anchor(s.as_vector(), s.cov, mid)
        vec, cov = unanchor(vec, cov + self._floor, mid)
        o = Line(float(vec[0]), float(vec[1]), cov, s.p_start, s.p_end,
                 s.n_points).transform(frame.odom_sensor, 'odom')
        ends = (o.p_start.copy(), o.p_end.copy())
        t = SurfaceTrack(
            id=self._next_id, state=o.as_vector(), cov=o.cov, support=ends, cls=cls,
            status='tentative', hits=1, misses=0, age=0.0, last_update=float(frame.stamp),
            delta=Pose2D(), delta_cov=np.zeros((3, 3)), born=float(frame.stamp),
            anchor=frame.odom, seen=ends,
            max_range=max(float(np.linalg.norm(s.p_start)), float(np.linalg.norm(s.p_end))))
        self._next_id += 1
        self.tracks.append(t)
        self._event(events, frame.stamp, t, 'born')

    def _confirm(self, t: SurfaceTrack, stamp: float, events) -> None:
        rivals = [u for u in self.tracks
                  if u is not t and u.cls == t.cls and u.status in ESTABLISHED]
        if rivals:
            keep = max([t] + rivals, key=lambda u: (u.age, -u.id))
            for u in [t] + rivals:
                if u is not keep:
                    log.info('class conflict on %s: keeping #%d (age %.2f s), dropping #%d',
                             t.cls.value, keep.id, keep.age, u.id)
                    self._kill(u, f'class_conflict: kept #{keep.id}', stamp, events)
            if keep is not t:
                return
        t.status = 'confirmed'
        self._event(events, stamp, t, 'confirmed')

    def _kill(self, t: SurfaceTrack, reason: str, stamp: float, events) -> None:
        t.status = 'dead'
        t.death = reason
        self._event(events, stamp, t, 'dead', reason)

    def _event(self, events, stamp, t, kind, detail='') -> None:
        e = TrackEvent(float(stamp), t.id, kind, t.cls, detail)
        events.append(e)
        self.events.append(e)

    def _visibility(self, t: SurfaceTrack, pred: Line, frame, ranges) -> Visibility:
        """Were beams aimed at where this wall was last seen, and what came back?

        Expected beams: inside the bearing span of the last observed extent,
        within the grazing angle extraction believes (lambda_abd), and no
        farther than this surface has ever returned from -- the horizon is the
        surface's own evidence, never a constant. Each expected beam is then
        'present' (returned at the wall), 'blocked' (returned nearer: an
        occluder, tracked or not) or 'through' (returned from beyond it, or no
        return at all).
        """
        p = self.p
        none = Visibility(t.id, 0, 0, 0, 0, 'unseen')
        if t.seen is None:
            return none
        ends = frame.odom_sensor.inverse().transform_points(np.vstack(t.seen))
        mid = ends.mean(axis=0)
        bm = math.atan2(mid[1], mid[0])
        d = [angdiff(math.atan2(e[1], e[0]), bm) for e in ends]
        rel = wrap_pi(np.asarray(frame.angles, dtype=float) - bm)
        sel = (rel >= min(d)) & (rel <= max(d))
        if not sel.any():
            return none
        ang = np.asarray(frame.angles, dtype=float)[sel]
        r = ranges[sel]
        cos_i = np.cos(ang - pred.alpha)
        ok = cos_i >= math.sin(p.extraction.lambda_abd)
        r_hat = np.where(ok, pred.rho / np.where(ok, cos_i, 1.0), np.inf)
        ok &= r_hat <= t.max_range
        n = int(ok.sum())
        if n < p.absent_min_beams:
            return Visibility(t.id, n, 0, 0, 0, 'unseen')
        ang, r, r_hat, cos_i = ang[ok], r[ok], r_hat[ok], cos_i[ok]
        # Perpendicular sigma of the prediction at each hit point, then along the beam.
        hit = np.column_stack([r_hat * np.cos(ang), r_hat * np.sin(ang)])
        t_h = hit @ pred.tangent
        P = pred.cov
        var_perp = np.maximum(P[0, 0] - 2.0 * t_h * P[0, 1] + t_h * t_h * P[1, 1], 0.0)
        sig = np.sqrt(point_sigma(r_hat, p.extraction) ** 2 + var_perp) / cos_i
        margin = p.absent_k_sigma * sig
        finite = np.isfinite(r)
        blocked = int(np.sum(finite & (r < r_hat - margin)))
        present = int(np.sum(finite & (np.abs(r - r_hat) <= margin)))
        through = n - blocked - present
        if through >= p.absent_fraction * n:
            verdict = 'absent'
        elif present >= 0.5 * n:
            verdict = 'present'
        elif blocked >= 0.5 * n:
            verdict = 'hidden'
        else:
            verdict = 'mixed'
        return Visibility(t.id, n, present, blocked, through, verdict)

    def _snapshot(self, frame, axis_update, seg_o, classes, assoc, rejections,
                  visibility, events) -> TrackerSnapshot:
        return TrackerSnapshot(
            stamp=float(frame.stamp), index=int(frame.index), pose=frame.odom,
            axis=copy.copy(self.axis), axis_update=axis_update,
            tracks=[copy.deepcopy(t) for t in self.tracks if t.status != 'dead'],
            segments=seg_o, classes=classes, associations=assoc,
            rejections=rejections, visibility=visibility, events=events)


def track_recording(frames: Iterable, params: TrackerParams | None = None,
                    forward_hint: float | None = None) -> tuple[CorridorTracker,
                                                                list[TrackerSnapshot]]:
    """Run a fresh tracker over a recording's frames.

    Without a forward_hint the seed is the vehicle's heading at the FIRST frame,
    i.e. at corridor entry, where the vehicle is roughly aligned with the
    corridor. That is the only time the heading is consulted.
    """
    tracker = CorridorTracker(params)
    snaps: list[TrackerSnapshot] = []
    for f in frames:
        if tracker.axis is None:
            tracker.seed(f.odom.theta if forward_hint is None else forward_hint, f.stamp)
        snaps.append(tracker.update(f))
    return tracker, snaps
