"""Corridor axis estimation and axis-relative surface classification.

The axis is the 2D form of a Manhattan-world structure compass: segment normals
are folded by 4 (period pi/2 becomes 2 pi), averaged on the unit circle weighted
by support length, and unfolded, giving a grid orientation g. Two normal
families exist, near g and near g + pi/2; the longer-support family is taken as
the lateral walls (laterals are long, end walls are short) and the axis is
perpendicular to their normals. The resultant length doubles as a
self-diagnosis: when it collapses the scene is not corridor-like.

THE 180 DEGREE AMBIGUITY IS IRREDUCIBLE. A corridor looks the same from both
ends and no geometry resolves it. forward_hint picks which way is forward, and
a caller must seed it once at corridor entry and carry the previous estimate
forward after that. Re-deriving it from the instantaneous heading is the bug
this package exists to remove: the robot yaws, the hint follows, the axis flips,
and FRONTAL becomes the wall behind it.

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


def _fold_error(alpha: float, direction: float) -> float:
    """Angle between a normal and a direction, modulo pi, in [0, pi/2]."""
    return abs(wrap_pi(2.0 * (alpha - direction))) / 2.0


def estimate_axis(segments: list[Line], forward_hint: float | None = None,
                  p: AxisParams | None = None) -> AxisEstimate:
    p = p or AxisParams()
    used = [s for s in segments if s.length >= p.min_seg_length]
    if not used:
        return AxisEstimate(0.0 if forward_hint is None else wrap_pi(forward_hint),
                            0.0, 0, 0.0, 0.0, p.min_strength, p.min_lateral_length)

    alphas = np.array([s.alpha for s in used])
    w = np.array([s.length for s in used])
    z = np.sum(w * np.exp(4j * alphas)) / np.sum(w)
    strength = float(abs(z))
    g = float(np.angle(z)) / 4.0

    len_a = len_b = 0.0
    for s in used:
        err = _fold_error(s.alpha, g)
        if err <= p.class_tol:
            len_a += s.length
        elif abs(err - math.pi / 2.0) <= p.class_tol:
            len_b += s.length
    if len_a >= len_b:
        lateral_normal, lateral_len, frontal_len = g, len_a, len_b
    else:
        lateral_normal, lateral_len, frontal_len = g + math.pi / 2.0, len_b, len_a

    theta = wrap_pi(lateral_normal + math.pi / 2.0)
    if forward_hint is not None and abs(angdiff(theta, forward_hint)) > math.pi / 2.0:
        theta = wrap_pi(theta + math.pi)
    return AxisEstimate(theta, strength, len(used), lateral_len, frontal_len,
                        p.min_strength, p.min_lateral_length)


def classify(segment: Line, axis: AxisEstimate, observer: Pose2D,
             p: AxisParams | None = None) -> SurfaceClass:
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
