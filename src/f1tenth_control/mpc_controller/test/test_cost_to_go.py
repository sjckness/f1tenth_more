"""
cost_to_go: the wall-aware lattice V(x, y, theta, gear) the reverse candidates use.

Pure Python. Geometry in metres, base_link at the rear axle.
"""

import math

from mpc_controller.cost_to_go import (
    build_field, CostToGo, LatticeConfig, occupancy_grid, window_center)
import numpy as np
import pytest

CFG = LatticeConfig(steer_cost_per_m=0.0)
HALL = [((-5.0, 0.8), (5.0, 0.8)), ((-5.0, -0.8), (5.0, -0.8))]

# A 1.3 m hall with a 0.9 m door in its left wall into a room (the closed-loop
# door scenario's geometry, test/reverse_harness.py).
DOOR_WALLS = [((-3.0, 0.65), (2.0, 0.65)), ((2.9, 0.65), (9.0, 0.65)),
              ((-3.0, -0.65), (9.0, -0.65)), ((1.0, 0.65), (1.0, 3.0)),
              ((4.0, 0.65), (4.0, 3.0)), ((1.0, 3.0), (4.0, 3.0))]
DOOR_JAMBS = [(2.0, 0.65, 0.05), (2.9, 0.65, 0.05)]
DOOR_GOAL = (2.45, 1.65)


def _body_clearance(x, y, psi, walls, front=0.405, rear=-0.082, hw=0.136):
    """Smallest distance from the body's outline (sampled) to the walls."""
    c, s = math.cos(psi), math.sin(psi)
    corners = [(front, hw), (rear, hw), (rear, -hw), (front, -hw)]
    pts = []
    for (a1, b1), (a2, b2) in zip(corners, corners[1:] + corners[:1]):
        for t in np.linspace(0.0, 1.0, 11):
            a, b = a1 + t * (a2 - a1), b1 + t * (b2 - b1)
            pts.append((x + c * a - s * b, y + s * a + c * b))
    best = math.inf
    for (x1, y1), (x2, y2) in walls:
        d = np.array([x2 - x1, y2 - y1])
        for p in pts:
            t = np.clip(((p[0] - x1) * d[0] + (p[1] - y1) * d[1]) / (d @ d), 0.0, 1.0)
            best = min(best, math.hypot(p[0] - x1 - t * d[0], p[1] - y1 - t * d[1]))
    return best


@pytest.fixture(scope='module')
def hall_behind():
    return build_field(CFG, (0.0, 0.0), (-1.5, 0.0, None), segments=HALL)


class TestCosts:

    def test_a_goal_straight_behind_in_a_corridor_is_reverse_metres_times_k_rev(
            self, hall_behind):
        # 1.5 m of reverse at k_rev 3, already in reverse gear ...
        assert hall_behind.value(0.0, 0.0, 0.0, -1) == pytest.approx(4.5, abs=0.31)
        # ... plus exactly one gear switch from forward gear.
        assert (hall_behind.value(0.0, 0.0, 0.0, +1) - hall_behind.value(0.0, 0.0, 0.0, -1)
                == pytest.approx(CFG.gear_switch_cost, abs=1e-6))

    def test_facing_the_goal_it_is_forward_metres(self, hall_behind):
        assert hall_behind.value(0.0, 0.0, math.pi, +1) == pytest.approx(1.5, abs=0.31)

    def test_far_behind_on_open_floor_turning_beats_a_long_reverse(self):
        f = build_field(CFG, (0.0, 0.0), (-3.5, 0.0, None))
        pure_reverse = 3.5 * CFG.k_rev + CFG.gear_switch_cost
        assert f.value(0.0, 0.0, 0.0, +1) < pure_reverse - 2.0
        path = f.policy_path(0.0, 0.0, 0.0, +1, 1.2)
        assert path[1][3] == +1          # the first move is forward

    def test_an_enclosed_goal_is_unreachable_but_finite(self):
        box = [((-2.2, -0.4), (-0.8, -0.4)), ((-0.8, -0.4), (-0.8, 0.4)),
               ((-0.8, 0.4), (-2.2, 0.4)), ((-2.2, 0.4), (-2.2, -0.4))]
        f = build_field(CFG, (0.5, 0.0), (-1.5, 0.0, None), segments=box)
        v = f.value(0.5, 0.0, 0.0, +1)
        assert math.isfinite(v) and v == pytest.approx(CFG.unreachable_cost)

    def test_steering_cost_only_breaks_ties(self):
        f0 = build_field(CFG, (0.0, 0.0), (1.8, 0.0, None))
        f1 = build_field(LatticeConfig(steer_cost_per_m=0.15), (0.0, 0.0), (1.8, 0.0, None))
        # Straight ahead the cost is the same; the policy is straight with it.
        assert f1.value(0.0, 0.0, 0.0, 1) == pytest.approx(f0.value(0.0, 0.0, 0.0, 1), abs=0.05)
        path = f1.policy_path(0.0, 0.0, 0.0, 1, 0.9)
        assert all(abs(p[1]) < 1e-9 and abs(p[2]) < 1e-9 for p in path)


class TestCollision:

    def test_a_pose_whose_body_overlaps_a_wall_is_unreachable(self, hall_behind):
        # Body half-width 0.136 + margin 0.10: base_link 0.15 m from the wall.
        assert hall_behind.value(0.5, 0.65, 0.0, +1) == pytest.approx(CFG.unreachable_cost)

    def test_the_policy_path_through_a_door_never_touches_a_wall(self):
        f = build_field(LatticeConfig(), (0.6, 0.0), (DOOR_GOAL[0], DOOR_GOAL[1], None),
                        segments=DOOR_WALLS, circles=DOOR_JAMBS)
        path = f.policy_path(0.6, 0.0, 0.0, +1, 6.0)
        assert len(path) > 5
        assert path[-1][1] > 0.65 + 0.3      # it does go through the door
        for x, y, psi, _g in path:
            assert _body_clearance(x, y, psi, DOOR_WALLS) > 0.0

    def test_halfplanes_mark_the_far_side(self):
        occ = occupancy_grid((0.0, 0.0), (5, 5), 0.1, halfplanes=[(0.0, 1.0, 0.25)])
        assert not occ[:, :3].any() and occ[:, 3:].all()


class TestQuery:

    def test_heading_wraps(self, hall_behind):
        assert hall_behind.value(0.0, 0.0, 2 * math.pi - 1e-9, -1) == pytest.approx(
            hall_behind.value(0.0, 0.0, -1e-9, -1), abs=1e-6)

    def test_outside_the_window_adds_the_distance(self, hall_behind):
        o = hall_behind.origin
        edge_x = o[0] + (hall_behind.shape[0] - 1) * CFG.resolution_m
        inside = hall_behind.value(edge_x, 0.0, math.pi, +1)
        assert hall_behind.value(edge_x + 0.5, 0.0, math.pi, +1) == pytest.approx(
            inside + 0.5, abs=1e-9)

    def test_a_goal_beyond_the_window_gives_a_clean_slope(self):
        # Reached through the window border, one primitive deep: V falls 1 per
        # forward metre everywhere on the straight, no 0.3 m staircase.
        f = build_field(CFG, (0.0, 0.0), (9.0, 0.0, None), segments=HALL)
        xs = np.arange(-0.8, 1.6, 0.1)
        v = np.array([f.value(x, 0.0, 0.0, 1) for x in xs])
        np.testing.assert_allclose(np.diff(v) / 0.1, -1.0, atol=0.05)

    def test_policy_target_in_reverse_backs_straight_to_the_goal(self):
        f = build_field(LatticeConfig(), (0.0, 0.0), (-1.5, 0.0, None), segments=HALL)
        x, y, psi, gear = f.policy_target(0.0, 0.0, 0.0, -1, 1.2, direction=-1)
        assert x == pytest.approx(-1.2, abs=1e-9) and abs(y) < 1e-9 and gear == -1
        assert psi == pytest.approx(0.0)


class TestWindowAndRebuild:

    def test_the_window_is_the_midpoint_or_slides_from_the_car(self):
        assert window_center((0, 0), (-1.5, 0), CFG) == pytest.approx((-0.75, 0.0))
        cx, cy = window_center((0, 0), (8.0, 0), CFG)
        assert cx == pytest.approx(CFG.window_m / 2 - CFG.car_edge_margin_m) and cy == 0.0

    def test_rebuilds_only_on_goal_walls_or_recentring(self):
        ctg = CostToGo(CFG, min_rebuild_period_s=0.5)
        assert ctg.update(0.0, (0, 0), (-1.5, 0.0), segments=HALL)
        assert not ctg.update(0.1, (0, 0), (-1.5, 0.0), segments=HALL)
        assert ctg.update(0.2, (0, 0), (-1.8, 0.0), segments=HALL)      # goal moved
        assert ctg.last_rebuild_reason == 'goal'
        moved = [((-5.0, 0.85), (5.0, 0.85)), HALL[1]]
        assert not ctg.update(0.3, (0, 0), (-1.8, 0.0), segments=moved)  # 5 cm: below
        far = [((-5.0, 1.0), (5.0, 1.0)), HALL[1]]
        assert not ctg.update(0.4, (0, 0), (-1.8, 0.0), segments=far)    # rate limit
        assert ctg.update(0.8, (0, 0), (-1.8, 0.0), segments=far)
        assert ctg.last_rebuild_reason == 'walls'
        assert ctg.update(1.4, (1.5, 0.0), (-1.8, 0.0), segments=far)
        assert ctg.last_rebuild_reason == 'recenter'
        assert len(ctg.build_times_ms) == 4 and all(t > 0 for t in ctg.build_times_ms)

    def test_clear_forgets_the_goal(self):
        ctg = CostToGo(CFG)
        ctg.update(0.0, (0, 0), (-1.5, 0.0))
        assert ctg.ready
        ctg.clear()
        assert not ctg.ready
