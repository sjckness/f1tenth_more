"""
Coverage for the corridor lookahead target -- pref_nom, the terminal cost's
only tie to the corridor.

WHAT THIS PINS, and the bug it replaces. compute_local_target projects the car
onto the corridor centerline (argmin distance), advances `lookahead` along the
cumulative arclength, and takes that point. The advance is the whole
mechanism: it is what makes the target sit AHEAD of the car on the returning
arc, and what makes it move forward as the car does.

That advance was dead. `lookahead` was a bare literal 1.5 inside
compute_local_target while corr_L_base had been shortened to 1.5 as well --
two independent constants describing one geometric relationship. On a corridor
1.5 m long, s_target = s_cum[idx] + 1.5 is >= s_cum[-1] for EVERY idx, so
searchsorted clamped to the last index on every single cycle. The target was
simply the corridor endpoint, and no test noticed because nothing asserted the
index moved.

The lookahead is derived from the corridor's own length now
(MPCController._corridor_lookahead), with a floor at the horizon's physical
reach. Both halves of that relationship are pinned below.

Same "testable without constructing a real MPCController" shape as
test_corridor_direction_recovery.py: a duck-typed stand-in carrying just the
instance state the two methods read.

Run standalone: python3 -m pytest test/test_corridor_lookahead_target.py -v
"""

import inspect
import math
import re
import unittest

import numpy as np

from f1tenth_params.param_defaults import get_value
from mpc_controller.MPC_corr import MPCController


class _FakeLogger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass


class _FakeMPC:
    """
    Just enough instance state for _corridor_lookahead/compute_local_target.

    Defaults mirror MPCController.__init__'s real ones.
    """

    def __init__(self, corr_L_base=3.0):
        # Lookahead geometry.
        self.corr_L_base = corr_L_base
        self.corr_lookahead_frac = 0.5
        self.corr_lookahead_reach_margin = 1.25
        self.N = 20
        self.ts = 0.1
        self.vdes = 0.5
        # Obstacle deflection.
        self.car_radius = 0.20
        self.avoidance_margin = 0.12
        # READ, NOT COPIED. This is the same stack_params.yaml key
        # MPCController.__init__ declares its obstacle_target_shift_m
        # parameter default from, so this stand-in cannot drift from the
        # deployed number the way a hand-mirrored literal would -- which is
        # exactly how corridor_update_period ended up with four spellings,
        # three of them wrong.
        self.obstacle_target_shift = float(get_value('obstacle_target_shift_m'))
        self.last_deflection_vec = np.zeros(2)
        self.deflection_decay_remaining = 0
        self.deflection_decay_ticks = 5
        # Smoothing. None on every call in these tests, so the RAW geometry is
        # what comes back -- the smoothing filter is exercised on its own
        # below rather than blurring the projection/advance assertions.
        self.smoothed_target = None
        self.target_smoothing_alpha = 0.5

    def get_logger(self):
        return _FakeLogger()

    def _corridor_lookahead(self, corridor):
        # compute_local_target calls this on self; delegate to the real
        # implementation so the two are exercised together rather than the
        # fake inventing its own lookahead.
        return MPCController._corridor_lookahead(self, corridor)


def _straight_corridor(length=3.0, n=120, psi=0.0, origin=(0.0, 0.0)):
    """Build a bare centerline dict.

    Only the keys compute_local_target actually reads.
    """
    s = np.linspace(0.0, length, n)
    return {
        "xc": origin[0] + s * math.cos(psi),
        "yc": origin[1] + s * math.sin(psi),
        "halfWidth": np.full(n, 0.4333),
        "L": float(length),
        "obstacles_world": [],
        "d_safe": 0.32,
    }


def _index_of(corridor, point):
    """Index of the centerline sample nearest *point*."""
    d2 = ((corridor["xc"] - point[0]) ** 2 + (corridor["yc"] - point[1]) ** 2)
    return int(np.argmin(d2))


def _on_centerline(corridor, point, tol=1e-9):
    """Distance from *point* to the nearest centerline sample."""
    idx = _index_of(corridor, point)
    return math.hypot(corridor["xc"][idx] - point[0],
                      corridor["yc"][idx] - point[1]) <= tol


class TestLookaheadIsDerivedFromCorridorLength(unittest.TestCase):
    """The relationship that used to be two colliding literals."""

    def test_the_reference_geometry_is_reproduced_exactly(self):
        """
        Confirm L=3.0 gives lookahead 1.5 -- the reference implementation's pair.

        corr_lookahead_frac is 0.5 precisely so that this holds; the ratio, not
        the absolute value, is the tuned quantity.
        """
        fake = _FakeMPC()
        self.assertAlmostEqual(
            MPCController._corridor_lookahead(fake, _straight_corridor(3.0)),
            1.5, places=9)

    def test_the_lookahead_lies_beyond_the_horizons_physical_reach(self):
        """
        Confirm the target stays a DIRECTION pull, not an arrival target.

        The horizon reaches N*ts*vdes = 20*0.1*0.5 = 1.0 m. A target inside
        that is a point the solver can plan to ARRIVE at within the horizon; a
        target beyond it can only be steered TOWARD, which is the
        pure-pursuit-like character the reference had (its target sat well
        past its own 0.28 m reach) and the one these weights were tuned for.
        """
        fake = _FakeMPC()
        reach = fake.N * fake.ts * fake.vdes
        self.assertAlmostEqual(reach, 1.0, places=9)
        self.assertGreater(
            MPCController._corridor_lookahead(fake, _straight_corridor(3.0)),
            reach)

    def test_it_tracks_the_corridor_length_rather_than_being_a_constant(self):
        """Confirm a longer corridor moves the target out with it."""
        fake = _FakeMPC()
        self.assertAlmostEqual(
            MPCController._corridor_lookahead(fake, _straight_corridor(6.0)),
            3.0, places=9)

    def test_a_too_short_corridor_is_floored_not_silently_degraded(self):
        """
        Pins the 1.5/1.5 collision, so a regression is legible as one.

        At corr_L_base 1.5 the length-derived lookahead is 0.75 m -- inside
        the 1.0 m horizon reach. That is a mis-configured geometry, and it now
        takes the reach floor and logs, rather than quietly turning the
        terminal cost into an arrival target the way the bare literal did.
        """
        fake = _FakeMPC(corr_L_base=1.5)
        lookahead = MPCController._corridor_lookahead(
            fake, _straight_corridor(1.5))
        self.assertAlmostEqual(lookahead, 1.25, places=9)
        self.assertGreater(lookahead, fake.N * fake.ts * fake.vdes)


class TestTheTargetSitsOnTheCenterlineAndAdvances(unittest.TestCase):
    """The projection + advance itself, with no obstacles in play."""

    def test_the_target_is_a_point_on_the_centerline(self):
        """Confirm the un-deflected target is exactly a centerline sample."""
        corridor = _straight_corridor(3.0)
        target = MPCController.compute_local_target(
            _FakeMPC(), [0.0, 0.0, 0.0, 0.5], corridor)
        self.assertTrue(_on_centerline(corridor, target))

    def test_it_sits_a_lookahead_ahead_of_the_cars_projection(self):
        """
        Confirm the advance distance is the lookahead, along arclength.

        Car at the origin of a straight 3 m corridor: its projection is index
        0 and the target lands at 1.5 m, to within one sample spacing.
        """
        corridor = _straight_corridor(3.0)
        target = MPCController.compute_local_target(
            _FakeMPC(), [0.0, 0.0, 0.0, 0.5], corridor)
        self.assertAlmostEqual(float(target[0]), 1.5, delta=3.0 / 119.0)
        self.assertAlmostEqual(float(target[1]), 0.0, places=9)

    def test_the_arclength_index_increases_as_the_car_progresses(self):
        """
        THE regression test for the dead advance.

        Same corridor, car stepped forward along it. The target's centerline
        index must strictly increase. With the old 1.5-lookahead-on-a-1.5m-
        corridor pairing every one of these returned the LAST index, so this
        assertion is exactly the one that would have caught it.
        """
        corridor = _straight_corridor(3.0)
        indices = []
        for x in (0.0, 0.25, 0.5, 0.75, 1.0):
            fake = _FakeMPC()
            target = MPCController.compute_local_target(
                fake, [x, 0.0, 0.0, 0.5], corridor)
            self.assertTrue(_on_centerline(corridor, target))
            indices.append(_index_of(corridor, target))

        for a, b in zip(indices, indices[1:]):
            self.assertGreater(b, a)
        # ...and none of them is the clamp: the advance is operating, not
        # bottoming out on the corridor end.
        self.assertLess(max(indices), len(corridor["xc"]) - 1)

    def test_it_still_clamps_gracefully_at_the_corridor_end(self):
        """Confirm a car past the last lookahead point takes the endpoint."""
        corridor = _straight_corridor(3.0)
        target = MPCController.compute_local_target(
            _FakeMPC(), [2.9, 0.0, 0.0, 0.5], corridor)
        self.assertEqual(
            _index_of(corridor, target), len(corridor["xc"]) - 1)

    def test_the_target_rides_the_returning_arc_of_a_bent_corridor(self):
        """
        Confirm the target is what ties the car back to the reference heading.

        On a corridor that bends from the car's live yaw back to the frozen
        one, the lookahead point sits ON that arc -- so a terminal cost that
        only knows this point still pulls the car around the bend. This is why
        the reference runs with w_psi = 0 and w_corr = 0.
        """
        n = 120
        s = np.linspace(0.0, 3.0, n)
        theta = np.linspace(0.4, 0.0, n)
        ds = np.zeros_like(s)
        ds[1:] = np.diff(s)
        corridor = {
            "xc": np.cumsum(np.cos(theta) * ds),
            "yc": np.cumsum(np.sin(theta) * ds),
            "halfWidth": np.full(n, 0.4333),
            "L": 3.0,
            "obstacles_world": [],
            "d_safe": 0.32,
        }
        target = MPCController.compute_local_target(
            _FakeMPC(), [0.0, 0.0, 0.4, 0.5], corridor)
        self.assertTrue(_on_centerline(corridor, target))
        # The bearing from the car to the target is turned back toward the
        # frozen heading (0.0), i.e. it is well inside the car's own 0.4 rad.
        bearing = math.atan2(float(target[1]), float(target[0]))
        self.assertLess(bearing, 0.4)
        self.assertGreater(bearing, 0.0)


class TestObstaclesReachTheTargetBeforeItIsComputed(unittest.TestCase):
    """The ordering the reference's own docstring flags as load-bearing."""

    def test_an_obstacle_on_the_target_displaces_it_off_the_centerline(self):
        """
        Confirm the deflection loop actually fires.

        It only fires if corridor["obstacles_world"] is populated BEFORE
        compute_local_target runs. It used to be written after, so the loop
        always saw an empty list and the displacement was dead code.
        """
        corridor = _straight_corridor(3.0)
        clean = MPCController.compute_local_target(
            _FakeMPC(), [0.0, 0.0, 0.0, 0.5], corridor)

        corridor["obstacles_world"] = [(1.5, 0.0, 0.15)]
        deflected = MPCController.compute_local_target(
            _FakeMPC(), [0.0, 0.0, 0.0, 0.5], corridor)

        self.assertFalse(_on_centerline(corridor, deflected, tol=1e-3))
        self.assertGreater(
            math.hypot(deflected[0] - clean[0], deflected[1] - clean[1]), 0.1)
        # Displaced clear of the obstacle's safety disc, not merely nudged.
        self.assertGreaterEqual(
            math.hypot(deflected[0] - 1.5, deflected[1] - 0.0),
            0.15 + 0.20 + 0.12 - 1e-6)

    def test_the_tangential_shove_is_the_parameter_not_a_literal(self):
        """
        Pin the SIZE of the sideways displacement to obstacle_target_shift_m.

        The ceiling used to be `0.6 * mean(corridor["halfWidth"])` -- an
        unnamed literal that silently retuned itself whenever corr_wmin/
        corr_wmax moved, and that no config file recorded. It is a declared
        parameter now, so this asserts the parameter is what actually reaches
        the geometry: two different values, two matching displacements.

        Geometry: the lookahead target lands exactly on the obstacle centre
        (obstacle at 1.5 m, lookahead 1.5 m), which is the degenerate branch
        -- penetration saturates at 1.0, so the tangential term is the full
        ceiling and reads straight off the y coordinate. The radial term goes
        entirely into x, so the two do not mix.
        """
        for shift in (0.30, 0.12):
            with self.subTest(shift=shift):
                corridor = _straight_corridor(3.0)
                corridor["obstacles_world"] = [(1.5, 0.0, 0.15)]
                fake = _FakeMPC()
                fake.obstacle_target_shift = shift

                target = MPCController.compute_local_target(
                    fake, [0.0, 0.0, 0.0, 0.5], corridor)

                # Tangential (+y, since the car heads +x) == the parameter.
                self.assertAlmostEqual(float(target[1]), shift, places=6)
                # Radial (+x) is R_safe past the obstacle, untouched by this
                # change -- guards against the parameter leaking into the
                # radial term.
                r_safe = 0.15 + fake.car_radius + fake.avoidance_margin
                self.assertAlmostEqual(float(target[0]), 1.5 + r_safe, places=6)

    def test_no_hardcoded_deflection_ceiling_survives_in_the_source(self):
        """The old corridor-derived literal is gone, not merely bypassed."""
        src = inspect.getsource(MPCController.compute_local_target)
        self.assertNotIn('0.6 * float(np.mean(', src)
        self.assertIn('self.obstacle_target_shift', src)

    def test_the_shipping_shift_fits_inside_the_narrow_corridor(self):
        """
        The sanity check that makes 0.30 a legal value, kept executable.

        A deflection wider than the corridor half-width puts the terminal
        cost's own target outside the corridor the corridor cost is pulling
        the car back into -- the two costs then fight. corr_wmin is the
        narrow end and is still a bare literal in __init__, so it is read
        out of the source rather than copied here.
        """
        shift = float(get_value('obstacle_target_shift_m'))
        src = inspect.getsource(MPCController.__init__)
        wmin = float(re.search(r'self\.corr_wmin\s*=\s*([0-9.]+)', src).group(1))
        self.assertLess(shift, wmin)

    def test_control_loop_populates_the_dict_before_taking_the_target(self):
        """
        Pins the ORDERING in control_loop, which no geometry test can see.

        Reads the source: the obstacles_world/d_safe writes must appear before
        the compute_local_target call, and both must sit OUTSIDE the
        `if need_update:` corridor-rebuild block so they refresh every tick
        even when the corridor is served from cache.
        """
        src = inspect.getsource(MPCController.control_loop)
        i_obs = src.index('corridor["obstacles_world"] = ')
        i_safe = src.index('corridor["d_safe"] = ')
        i_target = src.index("self.compute_local_target(")
        i_rebuild = src.index("self.cached_corridor = self.build_straight_corridor")
        i_uncached = src.index("corridor = self.cached_corridor")

        self.assertLess(i_obs, i_target)
        self.assertLess(i_safe, i_target)
        # After the rebuild block has been left behind, so this runs on cached
        # corridors too.
        self.assertLess(i_rebuild, i_uncached)
        self.assertLess(i_uncached, i_obs)


class TestEveryGoalInvalidatesTheCorridorCache(unittest.TestCase):
    """A new move must not solve against the previous move's geometry."""

    def _fake_node(self):
        fake = _FakeMPC()
        fake.cached_corridor = {"stale": True}
        fake.last_corridor_time = 123.0
        fake.last_corridor_stamp = object()
        fake.cached_pref_nom = np.array([9.0, 9.0])
        fake.smoothed_target = np.array([9.0, 9.0])
        fake.last_deflection_vec = np.array([1.0, 1.0])
        fake.deflection_decay_remaining = 3
        return fake

    def test_it_clears_the_corridor_and_the_target_state(self):
        """Confirm every cached field a stale move could leak through is reset."""
        fake = self._fake_node()
        MPCController._invalidate_move_state(fake)

        self.assertIsNone(fake.cached_corridor)
        self.assertIsNone(fake.last_corridor_time)
        self.assertIsNone(fake.last_corridor_stamp)
        self.assertIsNone(fake.cached_pref_nom)
        self.assertIsNone(fake.smoothed_target)
        self.assertEqual(fake.deflection_decay_remaining, 0)
        self.assertTrue(np.allclose(fake.last_deflection_vec, 0.0))

    def test_clearing_last_corridor_time_forces_a_rebuild_next_tick(self):
        """
        Confirm the cleared value is the one control_loop's gate reads.

        The gate is `cached_corridor is None or last_corridor_time is None`,
        so clearing either forces need_update -- which rebuilds AND publishes
        the markers, the two things a silently-reused corridor skipped.
        """
        src = inspect.getsource(MPCController.control_loop)
        self.assertIn(
            "if self.cached_corridor is None or self.last_corridor_time is None:",
            src)

    def test_all_three_goal_callbacks_invalidate(self):
        """
        Confirm no goal entry point can skip the invalidation.

        Structural rather than behavioural: the callbacks need a live rclpy
        node (parameters, TF, publishers) that this fake deliberately does not
        provide, so what is checked is that each one calls the helper the
        tests above pin the behaviour of.
        """
        for name in ("goal_distance_callback",
                     "goal_pose_callback",
                     "goal_turn_callback"):
            with self.subTest(callback=name):
                src = inspect.getsource(getattr(MPCController, name))
                self.assertIn("self._invalidate_move_state()", src)


if __name__ == '__main__':
    unittest.main()
