"""The pure planning pieces: geometry, Dubins, RRT*, pure pursuit, moves.

No ROS. Run standalone: python3 -m pytest test/test_planners_pure.py -q
"""

import math

from f1tenth_params.corridor_geometry import corridor_curves_to_pose
from f1tenth_planners import dubins
from f1tenth_planners.corridor_rrt_node import build_corridor, plan_in_corridor
from f1tenth_planners.geometry import (
    corridor_polygon, nearest_on_path, Obstacles, point_at_arclength, point_segment_distance,
    points_in_polygon, pose_map_to_odom, project_onto_line, segment_segments_distance)
from f1tenth_planners.moves import (
    DistanceMove, DriveMove, PlanTarget, PoseMove, TURN_LEAD_M, TURN_OVERSHOOT_DEG, turn_reach,
    TurnMove)
from f1tenth_planners.p2p_geometric_node import blocked_cap, geometric_path
from f1tenth_planners.pure_pursuit import (
    min_turning_radius, PurePursuit, PurePursuitConfig, rate_limit)
from f1tenth_planners.rrt import rrt_star, RRTConfig, shortcut
import numpy as np
import pytest

WHEELBASE = 0.305
DELTA = 0.309
R_MIN = min_turning_radius(WHEELBASE, -DELTA, 0.319)


# ---------------------------------------------------------------- geometry
class TestGeometry:

    def test_point_segment_distance(self):
        assert point_segment_distance(0.0, 1.0, -1.0, 0.0, 1.0, 0.0) == pytest.approx(1.0)
        assert point_segment_distance(3.0, 0.0, -1.0, 0.0, 1.0, 0.0) == pytest.approx(2.0)

    def test_crossing_segments_have_zero_distance(self):
        d = segment_segments_distance((0, -1), (0, 1), [(-1, 0)], [(1, 0)])
        assert d[0] == 0.0
        d = segment_segments_distance((0, 1), (1, 1), [(0, 0)], [(1, 0)])
        assert d[0] == pytest.approx(1.0)

    def test_polygon_containment(self):
        square = np.array([[0, 0], [2, 0], [2, 2], [0, 2]], dtype=float)
        inside = points_in_polygon([1.0, 3.0, 0.5], [1.0, 1.0, 1.9], square)
        assert inside.tolist() == [True, False, True]

    def test_corridor_polygon_contains_its_centreline(self):
        c = corridor_curves_to_pose(0, 0, 0, 3, 0.5, 0.3, 60, w0=0.64, w1=1.13)
        poly = corridor_polygon(c)
        assert points_in_polygon(c['xc'][5:-1], c['yc'][5:-1], poly).all()
        assert not points_in_polygon([1.5], [3.0], poly)[0]

    def test_path_lookup_and_extrapolation(self):
        path = np.array([[0, 0], [1, 0], [2, 0]], dtype=float)
        i, s, d = nearest_on_path(path, 1.5, 0.3)
        assert (i, s, d) == (1, pytest.approx(1.5), pytest.approx(0.3))
        x, y, h = point_at_arclength(path, 2.5)
        assert (x, y, h) == (pytest.approx(2.5), pytest.approx(0.0), pytest.approx(0.0))

    def test_map_to_odom_inverts_the_edge(self):
        # odom origin at (1, 2) in map, rotated 90 deg: the map point (1, 3) is
        # one metre along odom's +x.
        x, y, psi = pose_map_to_odom(1.0, 3.0, math.pi / 2, 1.0, 2.0, math.pi / 2)
        assert (x, y, psi) == (pytest.approx(1.0), pytest.approx(0.0), pytest.approx(0.0))

    def test_along_line_projection_ignores_the_sideways_part(self):
        assert project_onto_line((2.0, 5.0), (0.0, 0.0), 0.0) == pytest.approx(2.0)

    def test_obstacles(self):
        obs = Obstacles(circles=[(1.0, 0.0, 0.3)], walls=[(0, 1, 3, 1)], wall_clearance=0.2)
        assert not obs.point_free(1.1, 0.0)
        assert not obs.point_free(2.0, 0.9)
        assert obs.point_free(2.0, 0.5)
        assert not obs.segment_free((0, 0), (2, 0))
        assert obs.segment_free((0, 0.5), (3, 0.5))
        path = np.array([[0, 0], [0.5, 0], [1.0, 0], [1.5, 0]], dtype=float)
        assert obs.first_blocked_arclength(path) == pytest.approx(0.5)


# ------------------------------------------------------------------ dubins
def _end(start, segments):
    return dubins.integrate(start, segments, R_MIN, ds=0.01)[-1]


class TestDubins:

    @pytest.mark.parametrize('goal', [
        (3.0, 0.0, 0.0), (2.0, 2.0, math.pi / 2), (0.5, 1.5, math.pi), (-2.0, 1.0, -2.5),
        (1.0, -3.0, -math.pi / 2), (0.3, 0.1, 0.0), (4.0, 0.2, 0.0)])
    def test_to_pose_lands_on_the_pose(self, goal):
        segments = dubins.to_pose((0.0, 0.0, 0.0), goal, R_MIN)
        assert segments is not None
        x, y, yaw = _end((0.0, 0.0, 0.0), segments)
        assert math.hypot(x - goal[0], y - goal[1]) < 0.02
        assert abs(math.atan2(math.sin(yaw - goal[2]), math.cos(yaw - goal[2]))) < 0.02
        assert dubins.path_length(segments) >= math.hypot(goal[0], goal[1]) - 1e-6

    @pytest.mark.parametrize('goal', [(3.0, 0.0), (1.0, 2.0), (-1.0, 0.5), (0.2, 0.6)])
    def test_to_point_reaches_the_point(self, goal):
        x, y, _ = _end((0.0, 0.0, 0.0), dubins.to_point((0.0, 0.0, 0.0), goal, R_MIN))
        assert math.hypot(x - goal[0], y - goal[1]) < 0.03

    def test_straight_ahead_is_straight(self):
        segments = dubins.to_point((0.0, 0.0, 0.0), (2.0, 0.0), R_MIN)
        assert dubins.path_length(segments) == pytest.approx(2.0, abs=1e-6)

    @pytest.mark.parametrize('delta', [math.pi / 2, -math.pi / 2, math.pi / 4, math.pi])
    def test_a_full_lock_turn_is_one_arc_and_the_lead(self, delta):
        # TurnMove's target is the end of the full-lock circle plus the lead:
        # the Dubins path to it is that arc and a straight, not a loop.
        target = TurnMove((0.0, 0.0, 0.0), delta, 'full_lock', WHEELBASE, DELTA,
                          0.15, 0.4).target((0.0, 0.0, 0.0))
        segments = dubins.to_pose((0.0, 0.0, 0.0), (target.x, target.y, target.yaw), R_MIN)
        turned = abs(delta) + math.radians(TURN_OVERSHOOT_DEG)
        assert dubins.path_length(segments) == pytest.approx(
            R_MIN * turned + TURN_LEAD_M, rel=0.02)


# --------------------------------------------------------------------- rrt
def _box_sampler(lo, hi):
    def sample(rng):
        return rng.uniform(lo[0], hi[0]), rng.uniform(lo[1], hi[1])
    return sample


class TestRrt:

    def test_free_space_is_one_straight_edge(self):
        obs = Obstacles()
        r = rrt_star((0, 0), 0.0, (3, 0), _box_sampler((0, -1), (3, 1)),
                     obs.point_free, obs.segment_free, RRTConfig(seed=1))
        assert r.solved and r.direct and len(r.path) == 2

    def test_goes_around_an_obstacle_and_keeps_clear(self):
        obs = Obstacles(circles=[(1.5, 0.0, 0.5)])
        cfg = RRTConfig(seed=3, time_budget_s=1.0)
        r = rrt_star((0, 0), 0.0, (3, 0), _box_sampler((0, -1.5), (3.5, 1.5)),
                     obs.point_free, obs.segment_free, cfg)
        assert r.solved and not r.direct
        for a, b in zip(r.path[:-1], r.path[1:]):
            assert obs.segment_free(a, b)
        assert r.cost < 4.5

    def test_root_cone_forbids_starting_sideways(self):
        # Goal straight to the left: the first edge may not leave at 90 deg.
        obs = Obstacles()
        cone = math.radians(50.0)
        r = rrt_star((0, 0), 0.0, (0, 2), _box_sampler((-1, -1), (3, 3)),
                     obs.point_free, obs.segment_free,
                     RRTConfig(seed=5, root_cone=cone, time_budget_s=1.0))
        assert r.solved
        first = r.path[1] - r.path[0]
        assert abs(math.atan2(first[1], first[0])) <= cone + 1e-9

    def test_a_wall_to_wall_block_has_no_path(self):
        obs = Obstacles(walls=[(1.5, -2.0, 1.5, 2.0)], wall_clearance=0.2)
        r = rrt_star((0, 0), 0.0, (3, 0), _box_sampler((0, -1), (3, 1)),
                     obs.point_free, obs.segment_free,
                     RRTConfig(seed=2, max_iterations=400, time_budget_s=1.0))
        assert not r.solved and r.path is None

    def test_shortcut_keeps_visibility(self):
        zigzag = np.array([[0, 0], [1, 0.3], [2, -0.3], [3, 0.3], [4, 0]], dtype=float)
        out = shortcut(zigzag, 0.0, Obstacles().segment_free)
        assert len(out) == 2

    def test_plans_inside_the_corridor_around_an_obstacle(self):
        pose = (0.0, 0.0, 0.0)
        target = PlanTarget(4.0, 0.0, 0.0, True)
        corridor = build_corridor(pose, target, 0.64, R_MIN)
        obs = Obstacles(circles=[(2.0, 0.1, 0.25 + 0.32)])
        result, poly = plan_in_corridor(pose, target, obs, corridor,
                                        RRTConfig(seed=7, time_budget_s=1.0))
        assert result.solved
        for a, b in zip(result.path[:-1], result.path[1:]):
            assert obs.segment_free(a, b)
        pts = np.linspace(result.path[:-1], result.path[1:], 9).reshape(-1, 2)
        # the corridor's end edges pass through the car and the goal themselves
        pts = pts[(np.hypot(pts[:, 0], pts[:, 1]) > 0.3)
                  & (np.hypot(pts[:, 0] - 4.0, pts[:, 1]) > 0.3)]
        assert points_in_polygon(pts[:, 0], pts[:, 1], poly).all()


# ------------------------------------------------------------ pure pursuit
def _simulate(path, start, steps=400, v=0.5, dt=0.05, config=None):
    pp = PurePursuit(config or PurePursuitConfig())
    pp.set_path(path)
    x, y, yaw = start
    trace = []
    for _ in range(steps):
        cmd = pp.step((x, y, yaw), v)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += v / WHEELBASE * math.tan(cmd.steering) * dt
        trace.append((x, y, yaw, cmd))
        if cmd.remaining < 0.05:
            break
    return trace


class TestPurePursuit:

    def test_steers_toward_the_path(self):
        pp = PurePursuit(PurePursuitConfig())
        pp.set_path(np.array([[0, 0], [5, 0]], dtype=float))
        assert pp.step((0.0, -0.3, 0.0), 0.5).steering > 0.0
        assert pp.step((0.0, 0.3, 0.0), 0.5).steering < 0.0

    def test_converges_onto_a_line_from_an_offset(self):
        trace = _simulate(np.array([[0, 0], [6, 0]], dtype=float), (0.0, 0.4, 0.0))
        x, y, yaw, _ = trace[-1]
        assert abs(y) < 0.05 and abs(yaw) < 0.1

    def test_steering_stays_inside_the_bounds(self):
        pp = PurePursuit(PurePursuitConfig())
        pp.set_path(np.array([[0, 0], [0, 5]], dtype=float))
        cmd = pp.step((0.0, 0.0, -math.pi / 2), 0.5)
        assert -0.309 <= cmd.steering <= 0.319

    def test_rate_limit(self):
        assert rate_limit(0.0, 1.0, 2.0, 0.1) == pytest.approx(0.2)
        assert rate_limit(0.0, -0.1, 2.0, 0.1) == pytest.approx(-0.1)


# ------------------------------------------------------------------- moves
class TestMoves:

    def test_distance_counts_along_the_line_only(self):
        m = DistanceMove((0.0, 0.0, 0.0), 2.0, stop_lead=0.14)
        assert m.progress((1.0, 0.8, 0.0)).remaining == pytest.approx(1.0)
        assert m.progress((1.87, 0.0, 0.0)).reached
        t = m.target((0.5, 0.2, 0.1))
        assert (t.x, t.y, t.yaw, t.terminal) == (2.0, 0.0, 0.0, True)

    def test_pose_arrives_inside_tolerance_or_passing(self):
        m = PoseMove((0.0, 0.0, 0.0), (2.0, 0.0), None, 0.15, 0.4)
        assert m.progress((1.9, 0.05, 0.0)).reached
        assert not m.progress((1.7, 0.3, 0.0)).reached
        assert m.progress((2.2, 0.25, 0.0)).reached        # passed it, 0.32 m off
        assert not m.progress((2.6, 0.0, 0.0)).reached      # passed, too far

    def test_turn_reach_is_mpc_corrs(self):
        # chord of R = L / tan(steering), clipped to [0.3, 3.0]
        r = WHEELBASE / math.tan(DELTA)
        assert turn_reach(math.pi / 2, 'full_lock', WHEELBASE, DELTA) == pytest.approx(
            2 * r * math.sin(math.pi / 4))
        assert turn_reach(math.pi / 2, 'partial:5', WHEELBASE, DELTA) == pytest.approx(3.0)

    @pytest.mark.parametrize('sign', [1.0, -1.0])
    def test_turn_ends_past_the_new_heading_on_the_circle(self, sign):
        m = TurnMove((0.0, 0.0, 0.0), sign * math.pi / 2, 'full_lock', WHEELBASE, DELTA,
                     0.15, 0.4)
        t = m.target((0.0, 0.0, 0.0))
        yaw = sign * (math.pi / 2 + math.radians(TURN_OVERSHOOT_DEG))
        assert t.yaw == pytest.approx(yaw)
        # on the circle of radius R around (0, sign R), then the lead
        ax, ay = t.x - TURN_LEAD_M * math.cos(yaw), t.y - TURN_LEAD_M * math.sin(yaw)
        assert math.hypot(ax, ay - sign * R_MIN) == pytest.approx(R_MIN)

    def test_drive_is_open_ended_and_wall_turn_commits_once(self):
        m = DriveMove((0.0, 0.0, 0.0), 'wall_turn', 1.0, math.pi / 2, 3.0, 1.6)
        assert m.open_ended and not m.progress((5.0, 0.0, 0.0)).reached
        assert not m.update_front_distance(2.0)
        assert not m.update_front_distance(-1.0)         # no reading
        assert m.update_front_distance(1.5)
        assert m.psi == pytest.approx(math.pi / 2)
        assert not m.update_front_distance(1.0)          # once
        t = m.target((1.0, 0.0, 0.0))
        assert (t.x, t.y) == (pytest.approx(1.0), pytest.approx(3.0))

    def test_wall_turn_without_a_side_is_straight(self):
        assert DriveMove((0, 0, 0), 'wall_turn', 0.0, 1.0, 3.0, 1.6).mode == 'straight'


# --------------------------------------------------------------------- p2p
class TestP2p:

    def test_a_small_lateral_offset_does_not_loop(self):
        # 10 cm off the line, 0.6 m from its end: the Dubins path to the line's
        # end pose would loop; the free-heading one is taken instead.
        path = geometric_path((0.0, 0.1, 0.0), PlanTarget(0.6, 0.0, 0.0, True), R_MIN)
        length = float(np.sum(np.hypot(*np.diff(path, axis=0).T)))
        assert length < 1.0

    def test_a_reachable_heading_is_honoured(self):
        path = geometric_path((0.0, 0.0, 0.0), PlanTarget(3.0, 1.5, math.pi / 2, True), R_MIN)
        d = path[-1] - path[-2]
        assert math.atan2(d[1], d[0]) == pytest.approx(math.pi / 2, abs=0.1)

    def test_blocked_cap_slows_then_stops(self):
        path = np.column_stack([np.linspace(0, 3, 31), np.zeros(31)])
        obs = Obstacles(circles=[(2.0, 0.0, 0.3)])
        far = blocked_cap(obs, path, 0.0, 1.5, 0.5, 0.5)
        assert far == math.inf                           # 1.7 m ahead, outside 1.5
        mid = blocked_cap(obs, path, 0.6, 1.5, 0.5, 0.5)
        assert 0.0 < mid < 0.5
        assert blocked_cap(obs, path, 1.3, 1.5, 0.5, 0.5) == 0.0
