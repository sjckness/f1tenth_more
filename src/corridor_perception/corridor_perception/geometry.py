"""Planar poses and lines in Hesse normal form.

A line is rho = x*cos(alpha) + y*sin(alpha) with rho >= 0, so alpha is the
direction of the outward normal from the frame origin towards the line. This is
what makes the representation yaw-invariant: when the observer rotates in place
alpha changes by exactly the rotation and rho does not change at all. A raw
range has neither property, which is the whole reason this module exists.

Poses follow the "A from B" convention: Pose2D(x, y, theta) maps coordinates
expressed in frame B into frame A. compose() chains A<-B with B<-C into A<-C.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

TWO_PI = 2.0 * math.pi


def wrap_pi(a):
    """Wrap an angle (scalar or array) to [-pi, pi)."""
    if np.ndim(a) == 0:
        return (float(a) + math.pi) % TWO_PI - math.pi
    return (np.asarray(a, dtype=float) + np.pi) % TWO_PI - np.pi


def angdiff(a, b):
    """Smallest signed difference a - b, wrapped to [-pi, pi)."""
    return wrap_pi(a - b)


def normalise_line(rho: float, alpha: float) -> tuple[float, float]:
    """Enforce rho >= 0 by flipping alpha by pi when needed."""
    if rho < 0.0:
        return -rho, wrap_pi(alpha + math.pi)
    return rho, wrap_pi(alpha)


def _unit(angle: float) -> np.ndarray:
    return np.array([math.cos(angle), math.sin(angle)])


@dataclass
class Pose2D:
    """Frame A from frame B: p_A = R(theta) @ p_B + (x, y)."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    @property
    def t(self) -> np.ndarray:
        return np.array([self.x, self.y], dtype=float)

    @property
    def R(self) -> np.ndarray:
        c, s = math.cos(self.theta), math.sin(self.theta)
        return np.array([[c, -s], [s, c]])

    def compose(self, other: 'Pose2D') -> 'Pose2D':
        """self (A<-B) then other (B<-C) gives A<-C."""
        t = self.t + self.R @ other.t
        return Pose2D(float(t[0]), float(t[1]),
                      wrap_pi(self.theta + other.theta))

    def inverse(self) -> 'Pose2D':
        t = -(self.R.T @ self.t)
        return Pose2D(float(t[0]), float(t[1]), wrap_pi(-self.theta))

    def transform_points(self, pts) -> np.ndarray:
        """(N, 2) points in B to (N, 2) points in A."""
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        return pts @ self.R.T + self.t

    def as_vector(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta], dtype=float)


@dataclass
class Line:
    """An infinite line (rho, alpha) plus the finite support it was fitted on.

    cov is the 2x2 covariance of (rho, alpha) anchored at the frame origin.
    That anchoring is honest for reporting, but a distant line's rho variance
    is dominated by its tangential lever arm; see fit_line_tls().
    """

    rho: float
    alpha: float
    cov: np.ndarray = field(default_factory=lambda: np.zeros((2, 2)))
    p_start: np.ndarray = field(default_factory=lambda: np.zeros(2))
    p_end: np.ndarray = field(default_factory=lambda: np.zeros(2))
    n_points: int = 0
    frame: str = ''

    @property
    def normal(self) -> np.ndarray:
        return _unit(self.alpha)

    @property
    def tangent(self) -> np.ndarray:
        return np.array([-math.sin(self.alpha), math.cos(self.alpha)])

    @property
    def length(self) -> float:
        return float(np.linalg.norm(np.asarray(self.p_end) - self.p_start))

    @property
    def midpoint(self) -> np.ndarray:
        return 0.5 * (np.asarray(self.p_start) + np.asarray(self.p_end))

    def signed_distance(self, p) -> float:
        """p . n - rho: positive on the far side of the line from the origin."""
        return float(np.dot(np.asarray(p, dtype=float), self.normal) - self.rho)

    def distance_from(self, pose: Pose2D) -> float:
        return abs(self.signed_distance(pose.t))

    def bearing_from(self, pose: Pose2D) -> float:
        return wrap_pi(self.alpha - pose.theta)

    def transform(self, pose: Pose2D, frame: str | None = None) -> 'Line':
        """Express this line (in frame B) in frame A, where pose is A from B."""
        alpha_a = wrap_pi(self.alpha + pose.theta)
        n_a = _unit(alpha_a)
        tx, ty = pose.x, pose.y
        rho_a = self.rho + tx * n_a[0] + ty * n_a[1]
        J = np.array([[1.0, -tx * n_a[1] + ty * n_a[0]],
                      [0.0, 1.0]])
        cov_a = J @ np.asarray(self.cov, dtype=float) @ J.T
        ends = pose.transform_points(np.vstack([self.p_start, self.p_end]))
        rho_n, alpha_n = normalise_line(rho_a, alpha_a)
        if rho_n != rho_a:
            # d(-rho)/d(rho) = -1, alpha only offset: the cross term flips.
            F = np.diag([-1.0, 1.0])
            cov_a = F @ cov_a @ F.T
        return Line(rho_n, alpha_n, cov_a, ends[0].copy(), ends[1].copy(),
                    self.n_points, self.frame if frame is None else frame)

    def as_vector(self) -> np.ndarray:
        return np.array([self.rho, self.alpha], dtype=float)

    def innovation(self, other: 'Line') -> np.ndarray:
        """(d_rho, d_alpha) = self - other, resolving the antipodal form.

        A surface seen from the opposite side of the origin appears as
        (rho, alpha + pi); comparing against other's flipped form keeps the
        same physical surface from reading as a different one.
        """
        d_alpha = angdiff(self.alpha, other.alpha)
        if abs(d_alpha) > math.pi / 2.0:
            return np.array([self.rho + other.rho,
                             angdiff(self.alpha, other.alpha + math.pi)])
        return np.array([self.rho - other.rho, d_alpha])

    def overlap(self, other: 'Line') -> float:
        """Overlap of the two supports along self.tangent, metres.

        Negative is the gap between disjoint supports: two collinear stretches
        either side of a doorway overlap by minus the doorway width.
        """
        t = self.tangent
        a = sorted((float(np.dot(self.p_start, t)), float(np.dot(self.p_end, t))))
        b = sorted((float(np.dot(other.p_start, t)),
                    float(np.dot(other.p_end, t))))
        return min(a[1], b[1]) - max(a[0], b[0])
