"""safe_corridor.py tests -- synthetic grids only, no rclpy, no hardware.

Same convention test_costmap_boundary.py's own module docstring already
establishes for this package's pure-logic modules.

WHAT THIS PINS. Three things, in descending order of how much they matter:

  1. THE CONTAINMENT INVARIANT. The corridor centreline satisfies A x <= b
     at every covered sample, on every geometry exercised here. That is the
     entire justification for seeding on the reference instead of the car
     pose, and if it ever fails the seeding is broken -- so it is asserted
     directly, not inferred from a proxy.

  2. THE POLYTOPE BEATS THE NEAREST-CELL METHOD ON A KNOWN GEOMETRY. One
     oblique wall cell, one reachable point of free floor. The existing
     extract_boundary_constraints cuts that floor off; the polytope does
     not. That is the whole point of the change, so it is pinned against
     the REAL old implementation rather than a description of it.

  3. THE THREE CELL CASES. occupied / unknown / out-of-bounds each block,
     each by its own named branch, each independently checked -- including
     the direct contrast with costmap_boundary.py's opposite (permissive)
     convention, so a future reader can see the inversion is deliberate.

Run standalone: python3 -m pytest test/test_safe_corridor.py -v
"""

import math

import numpy as np
import pytest

from f1tenth_costmap.costmap_boundary import extract_boundary_constraints
from f1tenth_costmap.safe_corridor import (
    assert_reference_contained,
    build_safe_corridor,
    classify_local_cells,
    faces_to_car_frame,
    inflate_polytope,
    polygon_area,
    polytope_report,
    polytope_vertices,
    seed_clearance,
)

_OCC = 100      # a confidently-occupied cell
_FREE = 0       # observed and clear
_UNKNOWN = -1   # never observed -- OccupancyGrid's own sentinel
_THRESH = 65


def _blank_grid(width, height, fill=_FREE):
    return np.full((height, width), fill, dtype=np.int16)


def _corridor_grid(res=0.05, x_m=6.0, y_m=4.0, wall_y=(1.0, 3.0)):
    """Straight corridor: two full-length walls, observed-free between."""
    w, h = int(x_m / res), int(y_m / res)
    grid = _blank_grid(w, h)
    for y in wall_y:
        grid[int(y / res), :] = _OCC
    return grid.flatten().tolist(), w, h, res


def _straight_centreline(x0=0.5, x1=3.5, y=2.0, n=60):
    return np.array([[x, y] for x in np.linspace(x0, x1, n)], dtype=float)


# =====================================================================
# 1. The containment invariant -- the assertion that carries the change.
# =====================================================================
class TestReferenceStaysInsideItsOwnPolytope:
    """The corridor centreline must satisfy A x <= b at every covered point."""

    def test_straight_corridor_between_two_walls(self):
        grid, w, h, res = _corridor_grid()
        centreline = _straight_centreline()
        out = build_safe_corridor(
            grid, w, h, res, 0.0, 0.0, centreline,
            r_local=3.0, occupied_threshold=_THRESH)
        poly = out['polytope']

        # Explicit, not merely trusting inflate_polytope's own internal
        # assert: this is the property the whole design exists to provide.
        assert_reference_contained(poly['A'], poly['b'], poly['covered_points'])
        assert poly['covered_fraction'] == 1.0
        residual = centreline @ poly['A'].T - poly['b'][None, :]
        assert float(np.max(residual)) < 0.0

    def test_bend_truncates_coverage_but_never_breaks_containment(self):
        """A curved arc cannot fit one convex body -- coverage gives way.

        The invariant is unconditional; the COVERAGE is what degrades, and
        it degrades visibly (a fraction the caller can refuse) rather than
        silently handing back a polytope the reference violates.
        """
        res = 0.05
        w, h = int(6.0 / res), int(6.0 / res)
        grid = _blank_grid(w, h)
        # A blocking column on the inside of the bend, at x = 3.0.
        grid[:, int(3.0 / res)] = _OCC

        # Arc that starts heading +x then turns hard +y around the column.
        t = np.linspace(0.0, 1.0, 80)
        arc = np.stack([1.0 + 2.5 * t, 1.0 + 3.0 * t ** 2], axis=1)

        out = build_safe_corridor(
            grid, w, h, res, 0.0, 0.0, arc,
            r_local=3.0, occupied_threshold=_THRESH)
        poly = out['polytope']

        assert_reference_contained(poly['A'], poly['b'], poly['covered_points'])
        assert 0.0 <= poly['covered_fraction'] <= 1.0
        # Whatever is reported as covered really is covered.
        n_cov = len(poly['covered_points'])
        assert np.array_equal(poly['covered_points'], arc[:n_cov])

    def test_a_violated_reference_raises_rather_than_degrading(self):
        """assert_reference_contained fails loudly, per the spec."""
        A = np.array([[0.0, 1.0]])
        b = np.array([1.0])
        with pytest.raises(AssertionError, match='seeding is broken'):
            assert_reference_contained(A, b, np.array([[0.0, 2.0]]))

    def test_containment_tolerance_is_roundoff_not_a_slack_budget(self):
        A = np.array([[0.0, 1.0]])
        b = np.array([1.0])
        # Inside round-off: accepted.
        assert_reference_contained(A, b, np.array([[0.0, 1.0 + 1e-12]]))
        # A real breach of one millimetre: rejected.
        with pytest.raises(AssertionError):
            assert_reference_contained(A, b, np.array([[0.0, 1.001]]))


# =====================================================================
# 2. The polytope vs. the nearest-cell method it replaces.
# =====================================================================
class TestPolytopeKeepsFreeSpaceTheNearestCellMethodCarvesOff:
    """The whole point of the change, pinned against the REAL old code.

    GEOMETRY, chosen so the arithmetic is checkable by hand rather than
    just observed: the robot sits at the origin facing +x. ONE occupied
    cell sits at (1, 1) -- 45 deg off to the left-front, inside
    extract_boundary_constraints' own left window [45, 135] deg. The point
    under test is (2.0, 0.1): straight ahead, 2 m away, 0.9 m clear of the
    obstacle in y, unmistakably reachable free floor.

    THE OLD METHOD puts the normal along robot->cell, i.e. (0.707, 0.707),
    with offset |(1,1)| = 1.414. Its half-plane is therefore
    0.707x + 0.707y <= 1.414, i.e. x + y <= 2. The test point has
    x + y = 2.1 and is CUT OFF -- by a plane generated from one cell 1.4 m
    away on the diagonal, applied to space 2 m straight ahead where nothing
    was ever observed. That is problems 1 and 2 from the module docstring
    in a single number.

    THE POLYTOPE seeds on the centreline running straight ahead, so the
    seed point nearest the obstacle is (1, 0) directly below it, the normal
    is (0, 1), and the face is y <= 1. The test point has y = 0.1 and
    survives.
    """

    RES = 0.05
    OBSTACLE = (1.0, 1.0)
    REACHABLE = np.array([2.0, 0.1])
    R_LOCAL = 2.5
    # Origin pushed well clear of the action so the ONLY blocked cell in
    # play is the obstacle itself. With the grid starting at (0, 0) the seed
    # sat on the map's own corner, and the out-of-bounds cells behind and
    # below it (correctly blocked, see classify_local_cells) generated the
    # faces instead -- measuring the map edge, not the geometry under test.
    ORIGIN = -4.0
    EXTENT = 12.0

    def _grid(self):
        w = h = int(self.EXTENT / self.RES)
        grid = _blank_grid(w, h)
        col = int((self.OBSTACLE[0] - self.ORIGIN) / self.RES)
        row = int((self.OBSTACLE[1] - self.ORIGIN) / self.RES)
        grid[row, col] = _OCC
        return grid.flatten().tolist(), w, h

    def _centreline(self):
        # Length <= R_LOCAL, matching build_safe_corridor's own contract
        # that the arc handed in is already trimmed to the local window.
        return np.array([[x, 0.0] for x in np.linspace(0.0, self.R_LOCAL, 60)])

    def test_the_old_nearest_cell_method_cuts_off_reachable_floor(self):
        grid, w, h = self._grid()
        extraction = extract_boundary_constraints(
            grid, w, h, self.RES, self.ORIGIN, self.ORIGIN,
            robot_x=0.0, robot_y=0.0, robot_yaw=0.0,
            front_facing_max_rad=math.radians(35.0),
            side_window_min_rad=math.radians(45.0),
            side_window_max_rad=math.radians(135.0),
            occupied_threshold=_THRESH, max_range_m=5.0)

        left = extraction['left']
        assert left is not None, 'fixture broken: the cell must land in the left window'
        nx, ny, offset = left
        # Robot->cell diagonal, as described above.
        assert nx == pytest.approx(math.sqrt(0.5), abs=0.02)
        assert ny == pytest.approx(math.sqrt(0.5), abs=0.02)
        # THE FINDING: reachable free floor is outside the old constraint.
        assert nx * self.REACHABLE[0] + ny * self.REACHABLE[1] > offset

    def test_the_polytope_keeps_that_same_floor(self):
        grid, w, h = self._grid()
        out = build_safe_corridor(
            grid, w, h, self.RES, self.ORIGIN, self.ORIGIN, self._centreline(),
            r_local=self.R_LOCAL, occupied_threshold=_THRESH)
        poly = out['polytope']

        residual = poly['A'] @ self.REACHABLE - poly['b']
        assert float(np.max(residual)) < 0.0, (
            'the polytope cut off the free floor it exists to preserve')

    def test_and_still_excludes_the_obstacle_itself(self):
        """Keeping free space is only a win if the obstacle stays out."""
        grid, w, h = self._grid()
        out = build_safe_corridor(
            grid, w, h, self.RES, self.ORIGIN, self.ORIGIN, self._centreline(),
            r_local=self.R_LOCAL, occupied_threshold=_THRESH)
        poly = out['polytope']

        # The obstacle cell centre must violate at least one face.
        cell_centre = np.array([
            self.ORIGIN + (int((self.OBSTACLE[0] - self.ORIGIN) / self.RES) + 0.5) * self.RES,
            self.ORIGIN + (int((self.OBSTACLE[1] - self.ORIGIN) / self.RES) + 0.5) * self.RES,
        ])
        assert float(np.max(poly['A'] @ cell_centre - poly['b'])) >= -1e-9


# =====================================================================
# 3. The three cell cases, each by name.
# =====================================================================
class TestOccupiedUnknownAndOutOfBoundsAllBlock:

    RES = 1.0

    def test_occupied_cells_block(self):
        grid = _blank_grid(10, 10)
        grid[5, 7] = _OCC
        pts, stats = classify_local_cells(
            grid.flatten().tolist(), 10, 10, self.RES, 0.0, 0.0,
            seed_x=5.5, seed_y=5.5, r_local=3.0, occupied_threshold=_THRESH)
        assert stats['n_occupied'] == 1
        assert stats['n_unknown'] == 0
        assert stats['n_out_of_bounds'] == 0
        assert any(np.allclose(p, [7.5, 5.5]) for p in pts)

    def test_unknown_cells_block_and_are_counted_separately(self):
        """-1 is BLOCKED here -- the inverse of costmap_boundary.py."""
        grid = _blank_grid(10, 10)
        grid[5, 7] = _UNKNOWN
        pts, stats = classify_local_cells(
            grid.flatten().tolist(), 10, 10, self.RES, 0.0, 0.0,
            seed_x=5.5, seed_y=5.5, r_local=3.0, occupied_threshold=_THRESH)
        assert stats['n_unknown'] == 1
        assert stats['n_occupied'] == 0
        assert any(np.allclose(p, [7.5, 5.5]) for p in pts)

    def test_an_unknown_cell_is_never_silently_treated_as_free(self):
        """The distinction that matters: below-threshold is not the same
        as unobserved. A value of 10 is observed-and-mostly-clear and does
        NOT block; -1 has never been seen and DOES."""
        grid = _blank_grid(10, 10)
        grid[5, 7] = 10        # observed, well below threshold
        grid[5, 3] = _UNKNOWN  # never observed
        _pts, stats = classify_local_cells(
            grid.flatten().tolist(), 10, 10, self.RES, 0.0, 0.0,
            seed_x=5.5, seed_y=5.5, r_local=3.0, occupied_threshold=_THRESH)
        assert stats['n_blocked'] == 1
        assert stats['n_unknown'] == 1
        assert stats['n_occupied'] == 0

    def test_out_of_bounds_blocks_through_an_explicit_branch(self):
        """A seed near the grid edge must see the edge as a wall."""
        grid = _blank_grid(10, 10)  # covers [0, 10) x [0, 10)
        _pts, stats = classify_local_cells(
            grid.flatten().tolist(), 10, 10, self.RES, 0.0, 0.0,
            seed_x=0.5, seed_y=5.5, r_local=3.0, occupied_threshold=_THRESH)
        assert stats['n_out_of_bounds'] > 0
        assert stats['oob_fraction'] > 0.0

    def test_a_seed_entirely_outside_the_grid_is_entirely_blocked(self):
        grid = _blank_grid(10, 10)
        _pts, stats = classify_local_cells(
            grid.flatten().tolist(), 10, 10, self.RES, 0.0, 0.0,
            seed_x=50.0, seed_y=50.0, r_local=3.0, occupied_threshold=_THRESH)
        assert stats['n_cells'] > 0
        assert stats['n_out_of_bounds'] == stats['n_cells']
        assert stats['oob_fraction'] == 1.0

    def test_this_inverts_costmap_boundary_and_that_is_the_point(self):
        """The contrast, pinned so the inversion reads as deliberate.

        Same grid, same robot/seed position outside the map. The old
        extraction reports NO constraint (which its consumer turns into
        "clear at least max_range ahead"); this module reports every cell
        blocked. Permissive is right for an advisory scalar and wrong for a
        hard constraint -- see safe_corridor.py's own module docstring and
        the measurement in classify_local_cells'.
        """
        grid = _blank_grid(10, 10)
        flat = grid.flatten().tolist()

        old = extract_boundary_constraints(
            flat, 10, 10, self.RES, 0.0, 0.0,
            robot_x=50.0, robot_y=50.0, robot_yaw=0.0,
            front_facing_max_rad=math.radians(35.0),
            side_window_min_rad=math.radians(45.0),
            side_window_max_rad=math.radians(135.0),
            occupied_threshold=_THRESH, max_range_m=5.0)
        assert old['front'] is None and old['left'] is None and old['right'] is None

        _pts, stats = classify_local_cells(
            flat, 10, 10, self.RES, 0.0, 0.0,
            seed_x=50.0, seed_y=50.0, r_local=3.0, occupied_threshold=_THRESH)
        assert stats['n_blocked'] == stats['n_cells'] > 0

    def test_unknown_fraction_is_reported(self):
        grid = _blank_grid(20, 20, fill=_UNKNOWN)
        grid[8:12, 8:12] = _FREE
        _pts, stats = classify_local_cells(
            grid.flatten().tolist(), 20, 20, self.RES, 0.0, 0.0,
            seed_x=10.0, seed_y=10.0, r_local=5.0, occupied_threshold=_THRESH)
        assert 0.0 < stats['unknown_fraction'] < 1.0
        assert stats['n_unknown'] + stats['n_occupied'] + stats['n_out_of_bounds'] == \
            stats['n_blocked']


# =====================================================================
# 4. The inflation loop's own contract.
# =====================================================================
class TestInflationLoop:

    def test_every_face_has_an_obstacle_touching_it(self):
        """No face may cut across space nothing was observed in.

        This is the property that distinguishes the polytope from an
        infinite half-plane generated by a distant cell: for each face
        there must exist a blocked point p with n . p == b exactly (the
        face is TANGENT at p), or -- for a face that was later tightened by
        a merge -- a blocked point at or beyond it.
        """
        blocked = np.array([[2.0, 1.0], [2.0, -1.0], [4.0, 0.0]])
        seeds = np.array([[0.0, 0.0], [1.0, 0.0]])
        poly = inflate_polytope(seeds, blocked, max_faces=8)
        for i in range(poly['n_faces']):
            touching = blocked @ poly['A'][i] - poly['b'][i]
            assert float(np.max(touching)) >= -1e-9

    def test_the_seed_is_strictly_inside(self):
        blocked = np.array([[2.0, 0.0], [-2.0, 0.0], [0.0, 2.0], [0.0, -2.0]])
        seeds = np.array([[0.0, 0.0]])
        poly = inflate_polytope(seeds, blocked)
        assert seed_clearance(poly['A'], poly['b'], 0.0, 0.0) == pytest.approx(2.0)

    def test_no_blocked_points_means_no_faces(self):
        poly = inflate_polytope(np.array([[0.0, 0.0]]), np.zeros((0, 2)))
        assert poly['n_faces'] == 0
        assert poly['cap_hit'] is False
        assert poly['covered_fraction'] == 1.0
        assert seed_clearance(poly['A'], poly['b'], 0.0, 0.0) == math.inf

    def test_the_face_cap_is_enforced_and_reported(self):
        """A ring of scattered obstacles cannot be separated in 3 faces."""
        angles = np.linspace(0.0, 2.0 * math.pi, 40, endpoint=False)
        blocked = np.stack([3.0 * np.cos(angles), 3.0 * np.sin(angles)], axis=1)
        poly = inflate_polytope(np.array([[0.0, 0.0]]), blocked, max_faces=3)
        assert poly['n_faces'] == 3
        assert poly['cap_hit'] is True
        assert poly['n_unseparated'] > 0

    def test_the_cap_is_not_reported_when_the_set_was_exhausted(self):
        blocked = np.array([[2.0, 0.0]])
        poly = inflate_polytope(np.array([[0.0, 0.0]]), blocked, max_faces=8)
        assert poly['cap_hit'] is False
        assert poly['n_unseparated'] == 0

    def test_parallel_wall_faces_are_merged_not_fanned(self):
        """Regression: the un-merged loop spent its whole budget on one wall.

        Measured before the merge step existed, on exactly the corridor
        fixture below: 8 faces, SIX of them near-identical planes on the
        lower wall (normals within a degree of each other, offsets within
        8 mm), and not one face placed on the upper wall -- a polytope that
        did not exclude a wall the car could drive into. The merge step is
        therefore a correctness fix, not tidying, and this pins it.
        """
        grid, w, h, res = _corridor_grid()
        out = build_safe_corridor(
            grid, w, h, res, 0.0, 0.0, _straight_centreline(),
            r_local=3.0, occupied_threshold=_THRESH)
        poly = out['polytope']

        assert poly['n_faces'] <= 8
        assert poly['cap_hit'] is False
        assert poly['n_unseparated'] == 0

        # BOTH walls are represented: a face pointing -y (lower wall) and a
        # face pointing +y (upper wall).
        normals = poly['A']
        assert np.any(normals[:, 1] < -0.9), 'lower wall never got a face'
        assert np.any(normals[:, 1] > 0.9), 'upper wall never got a face'

    def test_merging_only_ever_tightens(self):
        """A merge may shrink the polytope; it may never grow it.

        Growing would let an already-separated obstacle back inside, which
        is the one thing the merge step must not do.
        """
        grid, w, h, res = _corridor_grid()
        centreline = _straight_centreline()
        out = build_safe_corridor(
            grid, w, h, res, 0.0, 0.0, centreline,
            r_local=3.0, occupied_threshold=_THRESH)
        poly = out['polytope']
        blocked, _stats = classify_local_cells(
            grid, w, h, res, 0.0, 0.0, centreline[0][0], centreline[0][1],
            3.0, _THRESH)
        # Nothing blocked remains strictly inside the final polytope.
        inside = np.all(blocked @ poly['A'].T < poly['b'][None, :] - 1e-9, axis=1)
        assert not np.any(inside)

    def test_an_empty_seed_set_is_a_programming_error(self):
        with pytest.raises(ValueError, match='at least one seed'):
            inflate_polytope(np.zeros((0, 2)), np.array([[1.0, 0.0]]))


# =====================================================================
# 5. Degeneracy is reported, never silently absorbed.
# =====================================================================
class TestDegeneracyIsReportable:

    def test_a_boxed_in_seed_is_flagged_degenerate(self):
        """Unknown cells 12 cm away in every direction -> unusable."""
        res = 0.02
        w = h = 100
        grid = _blank_grid(w, h, fill=_UNKNOWN)
        grid[45:55, 45:55] = _FREE  # a 20 cm observed island
        out = build_safe_corridor(
            grid.flatten().tolist(), w, h, res, 0.0, 0.0,
            np.array([[1.0, 1.0]]), r_local=0.5, occupied_threshold=_THRESH,
            min_seed_clearance=0.15, min_area=0.25, min_covered_fraction=0.25)
        report = out['report']
        assert report['degenerate'] is True
        assert report['degenerate_reasons']
        assert report['unknown_fraction'] > 0.5

    def test_a_healthy_corridor_is_not_flagged(self):
        grid, w, h, res = _corridor_grid()
        out = build_safe_corridor(
            grid, w, h, res, 0.0, 0.0, _straight_centreline(),
            r_local=3.0, occupied_threshold=_THRESH,
            min_seed_clearance=0.15, min_area=0.25, min_covered_fraction=0.25)
        assert out['report']['degenerate'] is False
        assert out['report']['degenerate_reasons'] == []

    def test_each_failing_test_names_itself(self):
        """Physically tiny and bends-out-of-the-body are different problems
        with different fixes, so they are reported separately."""
        poly = {
            'A': np.array([[1.0, 0.0], [-1.0, 0.0]]),
            'b': np.array([0.05, 0.05]),
            'n_faces': 2, 'cap_hit': False, 'n_unseparated': 0,
            'covered_points': np.zeros((0, 2)), 'covered_fraction': 0.1,
            'n_merged': 0,
        }
        report = polytope_report(
            poly, {'n_cells': 1}, 0.0, 0.0, 1.0,
            min_seed_clearance=0.15, min_area=0.25, min_covered_fraction=0.25)
        joined = ' '.join(report['degenerate_reasons'])
        assert 'seed_clearance' in joined
        assert 'area' in joined
        assert 'covered_fraction' in joined

    def test_build_returns_none_when_there_is_nothing_to_seed_on(self):
        grid, w, h, res = _corridor_grid()
        assert build_safe_corridor(
            grid, w, h, res, 0.0, 0.0, np.zeros((0, 2)),
            r_local=3.0, occupied_threshold=_THRESH) is None


# =====================================================================
# 6. Geometry helpers.
# =====================================================================
class TestVerticesAndArea:

    def test_a_unit_box_polytope_has_area_four(self):
        A = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
        b = np.array([1.0, 1.0, 1.0, 1.0])
        verts = polytope_vertices(A, b, (0.0, 0.0), 5.0)
        assert polygon_area(verts) == pytest.approx(4.0)

    def test_an_open_polytope_is_bounded_by_the_box(self):
        """Fewer than three faces does not enclose -- the box keeps the
        area finite and reportable instead of infinite."""
        A = np.array([[0.0, 1.0]])
        b = np.array([1.0])
        verts = polytope_vertices(A, b, (0.0, 0.0), 2.0)
        # Box is 4x4 centred at origin, clipped to y <= 1 -> 4 wide, 3 tall.
        assert polygon_area(verts) == pytest.approx(12.0)

    def test_infeasible_constraints_give_no_vertices_and_zero_area(self):
        A = np.array([[1.0, 0.0], [-1.0, 0.0]])
        b = np.array([-2.0, -2.0])  # x <= -2 and x >= 2
        verts = polytope_vertices(A, b, (0.0, 0.0), 5.0)
        assert verts.shape[0] == 0
        assert polygon_area(verts) == 0.0

    def test_fewer_than_three_vertices_has_no_area(self):
        assert polygon_area(np.array([[0.0, 0.0], [1.0, 1.0]])) == 0.0


class TestCarFrameConversion:

    def test_a_pure_translation_shifts_only_the_offset(self):
        A = np.array([[0.0, 1.0]])
        b = np.array([3.0])  # map frame: y <= 3
        faces = faces_to_car_frame(A, b, robot_x=0.0, robot_y=1.0, robot_yaw=0.0)
        (nx, ny, offset), = faces
        assert (nx, ny) == pytest.approx((0.0, 1.0))
        assert offset == pytest.approx(2.0)  # 2 m of room left above the car

    def test_a_pure_rotation_turns_the_normal_into_the_car_frame(self):
        A = np.array([[0.0, 1.0]])
        b = np.array([3.0])
        # Car yawed +90 deg: map +y is the car's own -x... i.e. straight
        # ahead in map +y becomes the car's own +x axis pointing map +y, so
        # the map-frame +y normal reads as (1, 0) in car frame.
        faces = faces_to_car_frame(A, b, 0.0, 0.0, math.radians(90.0))
        (nx, ny, offset), = faces
        assert nx == pytest.approx(1.0, abs=1e-9)
        assert ny == pytest.approx(0.0, abs=1e-9)
        assert offset == pytest.approx(3.0)

    def test_normals_stay_unit_and_the_constraint_is_preserved(self):
        """A rigid transform must not change WHICH points satisfy a face."""
        rng = np.random.default_rng(0)
        A = rng.normal(size=(5, 2))
        A /= np.linalg.norm(A, axis=1, keepdims=True)
        b = rng.normal(size=5)
        rx, ry, yaw = 1.3, -0.7, 0.9

        faces = faces_to_car_frame(A, b, rx, ry, yaw)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        for i, (nx, ny, offset) in enumerate(faces):
            assert math.hypot(nx, ny) == pytest.approx(1.0)
            # A map-frame point and its car-frame coordinates must agree.
            p_car = np.array([0.4, -0.2])
            p_map = np.array([rx, ry]) + np.array([
                cos_y * p_car[0] - sin_y * p_car[1],
                sin_y * p_car[0] + cos_y * p_car[1],
            ])
            lhs_map = float(A[i] @ p_map - b[i])
            lhs_car = float(np.array([nx, ny]) @ p_car - offset)
            assert lhs_car == pytest.approx(lhs_map)
