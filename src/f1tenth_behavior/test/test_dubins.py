"""dubins.py: the six words, the shortest choice, sampling, projection, footprint."""

import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from f1tenth_behavior.behaviours import dubins as db  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402


# One hand-picked pose pair per word, rho = 1, where that word is the shortest.
WORD_CASES = {
    'LSL': ((0.0, 0.0, 0.0), (4.0, 4.0, math.pi / 2.0)),
    'RSR': ((0.0, 0.0, 0.0), (4.0, -4.0, -math.pi / 2.0)),
    'LSR': ((0.0, 0.0, 0.0), (4.0, 4.0, -math.pi / 2.0)),
    'RSL': ((0.0, 0.0, 0.0), (4.0, -4.0, math.pi / 2.0)),
    'RLR': ((0.0, 0.0, 0.0), (1.0, 0.5, 0.9 * math.pi)),
    'LRL': ((0.0, 0.0, math.pi / 2.0), (0.2, 0.0, -math.pi / 2.0)),
}


def _pose_err(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(db.wrap_pi(a[2] - b[2]))


@pytest.mark.parametrize('word', db.WORDS)
def test_each_word_is_the_shortest_somewhere(word):
    q0, q1 = WORD_CASES[word]
    path = db.shortest_path(q0, q1, 1.0)
    assert path is not None and path.word == word
    pos, yaw = _pose_err(path.end(), q1)
    assert pos < 1e-9 and yaw < 1e-9


# ---------------------------------------------------------------------------
# Brute force: an independent solver. For every word XYZ, scan the first arc
# t over [0, 2 pi) and root-find the one condition that closes the path:
#   CSC: the straight leaving the first arc is tangent to the last circle
#        (the circle the goal pose turns on), on the correct side;
#   CCC: the middle circle is 2 rho from the last circle.
# Everything else (p, q) follows geometrically.
# ---------------------------------------------------------------------------
def _centre(pose, sigma, rho):
    x, y, th = pose
    return (x - sigma * rho * math.sin(th), y + sigma * rho * math.cos(th))


def _arc(pose, sigma, angle, rho):
    x, y, th = pose
    cx, cy = _centre(pose, sigma, rho)
    phi = math.atan2(y - cy, x - cx) + sigma * angle
    return (cx + rho * math.cos(phi), cy + rho * math.sin(phi), th + sigma * angle)


def _brute_word(q0, q1, rho, word, n=4000):
    s1 = 1 if word[0] == 'L' else -1
    s3 = 1 if word[2] == 'L' else -1
    cz = _centre(q1, s3, rho)
    csc = word[1] == 'S'

    def f(t):
        a = _arc(q0, s1, t, rho)
        if csc:
            lat = -math.sin(a[2]) * (cz[0] - a[0]) + math.cos(a[2]) * (cz[1] - a[1])
            return lat - s3 * rho
        cm = _centre(a, -s1, rho)
        return math.hypot(cm[0] - cz[0], cm[1] - cz[1]) - 2.0 * rho

    def length(t):
        a = _arc(q0, s1, t, rho)
        if csc:
            p = math.cos(a[2]) * (cz[0] - a[0]) + math.sin(a[2]) * (cz[1] - a[1])
            if p < -1e-6:
                return None
            q = db.mod2pi(s3 * (q1[2] - a[2]))
            return rho * t + p + rho * q
        cm = _centre(a, -s1, rho)
        tx, ty = (cm[0] + cz[0]) / 2.0, (cm[1] + cz[1]) / 2.0
        p = db.mod2pi(-s1 * (math.atan2(ty - cm[1], tx - cm[0])
                             - math.atan2(a[1] - cm[1], a[0] - cm[0])))
        q = db.mod2pi(s1 * (math.atan2(q1[1] - cz[1], q1[0] - cz[0])
                            - math.atan2(ty - cz[1], tx - cz[0])))
        return rho * (t + p + q)

    ts = np.linspace(0.0, 2.0 * math.pi, n + 1)
    fs = [f(t) for t in ts]
    best = None
    for i in range(n):
        if fs[i] == 0.0 or fs[i] * fs[i + 1] < 0.0:
            lo, hi = ts[i], ts[i + 1]
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                if f(lo) * f(mid) <= 0.0:
                    hi = mid
                else:
                    lo = mid
            ln = length(0.5 * (lo + hi))
            if ln is not None and (best is None or ln < best):
                best = ln
    return best


def _brute_shortest(q0, q1, rho):
    out = [v for v in (_brute_word(q0, q1, rho, w) for w in db.WORDS) if v is not None]
    return min(out) if out else None


def test_the_shortest_choice_matches_a_brute_force_solver_within_1_cm():
    rng = random.Random(7)
    cases = list(WORD_CASES.values())
    for _ in range(25):
        cases.append(((rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-math.pi, math.pi)),
                      (rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-math.pi, math.pi))))
    for q0, q1 in cases:
        for rho in (0.6, 1.05):
            path = db.shortest_path(q0, q1, rho)
            brute = _brute_shortest(q0, q1, rho)
            assert path is not None and brute is not None
            assert path.length == pytest.approx(brute, abs=0.01), (q0, q1, rho, path)


def test_sample_ends_on_the_goal_for_every_feasible_word():
    rng = random.Random(3)
    for _ in range(300):
        q0 = (rng.uniform(-4, 4), rng.uniform(-4, 4), rng.uniform(-4, 4))
        q1 = (rng.uniform(-4, 4), rng.uniform(-4, 4), rng.uniform(-4, 4))
        rho = rng.uniform(0.3, 2.0)
        for path in db.all_paths(q0, q1, rho):
            pts = path.sample(0.05)
            pos, yaw = _pose_err(pts[-1], q1)
            assert pos < 1e-3 and math.degrees(yaw) < 0.1, path
            pos0, yaw0 = _pose_err(pts[0], q0)
            assert pos0 < 1e-9 and yaw0 < 1e-9
            # evenly spaced, never more than the step apart
            gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
            assert max(gaps) <= 0.05 + 1e-9


def test_lengths_are_metres_and_scale_with_rho():
    q0, q1 = WORD_CASES['LSR']
    p1 = db.shortest_path(q0, q1, 1.0)
    p2 = db.shortest_path(tuple(v * 2 if i < 2 else v for i, v in enumerate(q0)),
                          tuple(v * 2 if i < 2 else v for i, v in enumerate(q1)), 2.0)
    assert p2.word == p1.word
    assert p2.length == pytest.approx(2.0 * p1.length)
    assert sum(p1.lengths) == pytest.approx(p1.length)
    # a straight run: t = q = 0, p = the distance
    s = db.shortest_path((1.0, 2.0, 0.3), (1.0 + 3 * math.cos(0.3), 2.0 + 3 * math.sin(0.3), 0.3),
                         1.0)
    assert s.length == pytest.approx(3.0) and s.p == pytest.approx(3.0)


@pytest.mark.parametrize('dist', [0.0, 1e-12, 1e-9, 1e-6, 1e-3])
@pytest.mark.parametrize('dyaw', [0.0, 1e-9, 0.5, math.pi])
def test_no_nan_at_or_near_coincident_poses(dist, dyaw):
    q0 = (1.0, -2.0, 0.7)
    q1 = (1.0 + dist * math.cos(1.1), -2.0 + dist * math.sin(1.1), 0.7 + dyaw)
    path = db.shortest_path(q0, q1, 0.9)
    assert path is not None
    assert all(math.isfinite(v) for v in (*path.lengths, path.length))
    end = path.end()
    assert all(math.isfinite(v) for v in end)
    pos, yaw = _pose_err(end, q1)
    assert pos < 1e-6 and yaw < 1e-6
    for pt in path.sample(0.05):
        assert all(math.isfinite(v) for v in pt)
    if dist == 0.0 and dyaw == 0.0:
        assert path.length == 0.0


def test_project_is_monotonic_along_the_path_and_signs_the_lateral_error():
    for q0, q1 in WORD_CASES.values():
        path = db.shortest_path(q0, q1, 1.0)
        prev = -1.0
        for i, (x, y, yaw) in enumerate(path.sample(0.1)):
            off = 0.05 if i % 2 else -0.05          # zig-zag beside the path
            s, lat = path.project(x - off * math.sin(yaw), y + off * math.cos(yaw),
                                  s_min=max(prev - 0.3, 0.0), s_max=prev + 1.0)
            assert s >= prev - 1e-6
            assert lat == pytest.approx(off, abs=2e-3)
            prev = s
    straight = db.shortest_path((0.0, 0.0, 0.0), (2.0, 0.0, 0.0), 1.0)
    s, lat = straight.project(0.73, 0.2)
    assert s == pytest.approx(0.73, abs=1e-6) and lat == pytest.approx(0.2, abs=1e-9)
    assert straight.pose_at(0.73) == pytest.approx((0.73, 0.0, 0.0))
    assert straight.pose_at(-1.0) == pytest.approx((0.0, 0.0, 0.0))
    assert straight.pose_at(9.0) == pytest.approx((2.0, 0.0, 0.0))


# ---------------------------------------------------------------------------
# The footprint check
# ---------------------------------------------------------------------------
def _grid(occupied=(), unknown=(), size=8.0, res=0.05, origin=(-4.0, -4.0)):
    """Build a free grid; `occupied` / `unknown` are map-frame boxes (x0, y0, x1, y1)."""
    n = int(round(size / res))
    data = np.zeros((n, n), dtype=np.int16)
    for boxes, val in ((unknown, -1), (occupied, 100)):
        for x0, y0, x1, y1 in boxes:
            i0, i1 = int((x0 - origin[0]) / res), int(math.ceil((x1 - origin[0]) / res))
            j0, j1 = int((y0 - origin[1]) / res), int(math.ceil((y1 - origin[1]) / res))
            data[max(j0, 0):max(j1, 0), max(i0, 0):max(i1, 0)] = val
    return db.OccGrid(data, res, origin[0], origin[1])


def test_path_is_free_on_a_free_grid_and_blocked_by_a_wall_across_it():
    path = db.shortest_path((0.0, 0.0, 0.0), (3.0, 0.0, 0.0), 1.0)
    free = db.path_is_free(path, _grid(), 0.136, 0.05)
    assert free.free and free.first_blocked_s is None and free.unknown_cells == 0
    ok, first, unknown = free                       # unpacks as a 3-tuple
    assert ok
    # a wall at x = 2.0: the bumper (0.405 ahead) + margin reaches it at s ~ 1.5
    wall = _grid(occupied=[(2.0, -1.0, 2.1, 1.0)])
    chk = db.path_is_free(path, wall, 0.136, 0.05)
    assert not chk.free
    assert 1.4 <= chk.first_blocked_s <= 1.6
    # a wall beside the path, clear of the body + margin: free
    side = _grid(occupied=[(0.0, 0.30, 3.0, 0.40)])
    assert db.path_is_free(path, side, 0.136, 0.05).free
    # ... the same wall 0.15 m closer: inside half-width 0.136 + margin 0.05
    assert not db.path_is_free(path, _grid(occupied=[(0.0, 0.15, 3.0, 0.25)]),
                               0.136, 0.05).free


def test_unknown_cells_are_free_but_counted():
    path = db.shortest_path((0.0, 0.0, 0.0), (3.0, 0.0, 0.0), 1.0)
    chk = db.path_is_free(path, _grid(unknown=[(1.0, -0.1, 1.5, 0.1)]), 0.136, 0.05)
    assert chk.free and chk.unknown_cells == 10 * 4
    # off the grid entirely is not counted (no cells there)
    far = db.shortest_path((0.0, 0.0, 0.0), (9.0, 0.0, 0.0), 1.0)
    assert db.path_is_free(far, _grid(), 0.136, 0.05).free
    # no grid: free, nothing counted (the caller says so)
    assert db.path_is_free(path, None, 0.136, 0.05) == (True, None, 0)
    assert db.path_is_free(None, _grid(), 0.136, 0.05).free is False


def test_straight_back_checks_only_the_strip_behind_the_rear_bumper():
    q0 = (0.0, 0.0, 0.0)
    # a wall 0.5 m behind the rear bumper (bumper at -0.082)
    wall = _grid(occupied=[(-0.70, -1.0, -0.582, 1.0)])
    assert db.straight_back_is_free(q0, 0.3, wall, 0.136, 0.05).free
    chk = db.straight_back_is_free(q0, 0.6, wall, 0.136, 0.05)
    assert not chk.free and 0.35 <= chk.first_blocked_s <= 0.5
    # something right in front of the car does not matter to a reverse
    front = _grid(occupied=[(0.45, -1.0, 0.6, 1.0)])
    assert db.straight_back_is_free(q0, 0.5, front, 0.136, 0.05).free
    assert db.straight_back_is_free(q0, 0.0, wall, 0.136, 0.05).free


def test_occgrid_from_a_message_reads_resolution_origin_and_yaw():
    from types import SimpleNamespace as NS
    yaw = math.pi / 2.0
    msg = NS(info=NS(width=4, height=2, resolution=0.5,
                     origin=NS(position=NS(x=1.0, y=2.0),
                               orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2),
                                              w=math.cos(yaw / 2)))),
             data=[0, 0, 0, 100, 0, -1, 0, 0])
    g = db.OccGrid.from_msg(msg)
    assert g.data.shape == (2, 4) and g.origin_yaw == pytest.approx(yaw)
    xs, ys, vals = g.cells_in_box(-5, -5, 5, 5)
    # cell (row 0, col 3): grid (1.75, 0.25) rotated +90 deg -> map (1 - 0.25, 2 + 1.75)
    i = int(np.nonzero(vals == 100)[0][0])
    assert (xs[i], ys[i]) == pytest.approx((0.75, 3.75))
