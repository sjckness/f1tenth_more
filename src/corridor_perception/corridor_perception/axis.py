"""Corridor axis estimation and axis-relative surface classification.

The axis is the 2D form of a Manhattan-world structure compass: segment normals
are folded by 4 (period pi/2 becomes 2 pi), averaged on the unit circle weighted
by support length, and unfolded, giving a grid orientation g (fold_orientation).
The resultant length doubles as a self-diagnosis: when it collapses the scene is
not corridor-like.

g is only known modulo 90 degrees, so it names four candidate axis directions,
g + k * 90. WHICH ONE IS THE AXIS IS NOT A PER-SCAN QUESTION. The first version
of this module answered it per scan by support balance -- the family with more
length was taken as the laterals -- and that is wrong on real data: partway down
a corridor the end wall and the far side of an opening outweigh the side walls,
the balance tips, and every label rotates by 90 degrees (2026-09-11T16-11-08,
around frame 520). The candidate is now chosen by proximity to the previous axis
(nearest_candidate), which resolves the 90 and the 180 degree ambiguity at once
and never consults support at all. Support balance survives only as the
no-hint path of estimate_axis, which is a seed-time sanity check and nothing
else.

THE 180 DEGREE AMBIGUITY IS IRREDUCIBLE. A corridor looks the same from both
ends and no geometry resolves it. forward_hint picks which way is forward, and
a caller must seed it once at corridor entry and carry the previous estimate
forward after that. Re-deriving it from the instantaneous heading is the bug
this package exists to remove: the robot yaws, the hint follows, the axis flips,
and FRONTAL becomes the wall behind it. The tracker (tracker.CorridorTracker)
is the owner of that carried state.

classify() compares a segment's normal with the AXIS, never with the vehicle
heading, so the category survives yaw. The observer pose only decides the
genuinely observer-relative parts: left/right and ahead/behind.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np

from .geometry import Line, Pose2D, angdiff, wrap_pi


class SurfaceClass(str, Enum):
    LATERAL_LEFT = 'lateral_left'
    LATERAL_RIGHT = 'lateral_right'
    FRONTAL = 'frontal'
    REAR = 'rear'
    UNSTRUCTURED = 'unstructured'


@dataclass
class AxisParams:
    min_strength: float = 0.75
    min_lateral_length: float = 2.0
    class_tol: float = math.radians(25.0)
    min_seg_length: float = 0.4


@dataclass
class AxisEstimate:
    theta: float            # corridor centreline direction, in the segments' frame
    strength: float         # 0..1 resultant length of the folded mean
    n_segments: int
    lateral_length: float
    frontal_length: float
    min_strength: float = AxisParams.min_strength
    min_lateral_length: float = AxisParams.min_lateral_length

    @property
    def is_corridor(self) -> bool:
        return (self.strength >= self.min_strength
                and self.lateral_length >= self.min_lateral_length)


@dataclass
class FoldEstimate:
    """The x4-folded mean of segment normals: grid orientation modulo 90 deg."""

    g: float                # in [-pi/4, pi/4)
    strength: float         # 0..1 resultant length
    variance: float         # first-order var(g) from the segments' own var(alpha)
    n_segments: int
    support: float          # total length that entered the mean, metres


def _fold_error(alpha: float, direction: float) -> float:
    """Angle between a normal and a direction, modulo pi, in [0, pi/2]."""
    return abs(wrap_pi(2.0 * (alpha - direction))) / 2.0


def fold_orientation(segments: list[Line], p: AxisParams | None = None) -> FoldEstimate | None:
    """Grid orientation g modulo 90 deg, or None when no segment is long enough.

    variance is the linearised spread of the weighted circular mean,
        var(g) = sum((w_i c_i)^2 var(alpha_i)) / (sum(w_i c_i))^2,
    c_i = cos(4 (alpha_i - g)). It covers fitting noise only. It does NOT cover
    walls that are not quite square to each other, which on the reference
    traverse is the larger term (0.15 deg modelled, 0.4 deg observed); a
    consumer that filters g must add its own model floor.
    """
    p = p or AxisParams()
    used = [s for s in segments if s.length >= p.min_seg_length]
    if not used:
        return None
    alphas = np.array([s.alpha for s in used])
    w = np.array([s.length for s in used])
    z = np.sum(w * np.exp(4j * alphas)) / np.sum(w)
    g = float(np.angle(z)) / 4.0
    wc = w * np.cos(4.0 * (alphas - g))
    var_a = np.array([float(np.asarray(s.cov)[1, 1]) for s in used])
    den = float(np.sum(wc))
    variance = float(np.sum(wc * wc * var_a) / (den * den)) if den > 1e-9 else math.inf
    return FoldEstimate(wrap_pi(g), float(abs(z)), variance, len(used), float(np.sum(w)))


def nearest_candidate(g: float, reference: float) -> float:
    """The one of g + k * 90 deg closest to reference, wrapped to [-pi, pi).

    This is the whole fix for the family swap: the choice depends on where the
    axis was, never on which family happens to have more support in this scan.
    """
    return wrap_pi(reference + wrap_pi(4.0 * (g - reference)) / 4.0)


def family_lengths(segments: list[Line], theta: float,
                   p: AxisParams | None = None) -> tuple[float, float]:
    """(lateral, frontal) support length relative to a given axis direction."""
    p = p or AxisParams()
    lateral = frontal = 0.0
    for s in segments:
        if s.length < p.min_seg_length:
            continue
        err = _fold_error(s.alpha, theta)
        if err <= p.class_tol:
            frontal += s.length
        elif abs(err - math.pi / 2.0) <= p.class_tol:
            lateral += s.length
    return lateral, frontal


def estimate_axis(segments: list[Line], forward_hint: float | None = None,
                  p: AxisParams | None = None) -> AxisEstimate:
    """One scan's axis.

    With a forward_hint the axis is the candidate g + k * 90 nearest the hint:
    pass the previous axis and the family cannot swap. Without one, the family
    is chosen by support balance and the direction is arbitrary; that path is a
    seed-time sanity check only, and calling it per scan reintroduces the
    90 degree swap this module's docstring describes.
    """
    p = p or AxisParams()
    fold = fold_orientation(segments, p)
    if fold is None:
        return AxisEstimate(0.0 if forward_hint is None else wrap_pi(forward_hint),
                            0.0, 0, 0.0, 0.0, p.min_strength, p.min_lateral_length)

    if forward_hint is not None:
        theta = nearest_candidate(fold.g, forward_hint)
    else:
        a_lat, a_front = family_lengths(segments, fold.g + math.pi / 2.0, p)
        theta = wrap_pi(fold.g + math.pi / 2.0) if a_lat >= a_front else fold.g
    lateral_len, frontal_len = family_lengths(segments, theta, p)
    return AxisEstimate(theta, fold.strength, fold.n_segments, lateral_len, frontal_len,
                        p.min_strength, p.min_lateral_length)


def classify(segment: Line, axis, observer: Pose2D,
             p: AxisParams | None = None) -> SurfaceClass:
    """Label a segment against an axis (anything with a .theta: an AxisEstimate
    or the tracker's carried CorridorAxis). segment, axis and observer must all
    be in the same frame."""
    p = p or AxisParams()
    err = _fold_error(segment.alpha, axis.theta)
    rel = segment.midpoint - observer.t
    if err <= p.class_tol:
        ahead = rel @ np.array([math.cos(axis.theta), math.sin(axis.theta)])
        return SurfaceClass.FRONTAL if ahead > 0.0 else SurfaceClass.REAR
    if abs(err - math.pi / 2.0) <= p.class_tol:
        left = rel @ np.array([-math.sin(axis.theta), math.cos(axis.theta)])
        return SurfaceClass.LATERAL_LEFT if left > 0.0 else SurfaceClass.LATERAL_RIGHT
    return SurfaceClass.UNSTRUCTURED


def classify_all(segments: list[Line], axis: AxisEstimate, observer: Pose2D,
                 p: AxisParams | None = None) -> list[SurfaceClass]:
    return [classify(s, axis, observer, p) for s in segments]
