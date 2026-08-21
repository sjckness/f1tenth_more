"""Hard boundary constraint tests (MPC hard boundary constraints task):
mpc_solver.py's fixed-3-slot padding + per-constraint QP bound math,
MPC_corr.py's base_link -> world halfspace transform, and (added by the
dual-EKF + costmap-derived-MPC-boundaries pass) MPC_corr.py's own
_select_live_boundaries staleness gate for costmap_boundary_node.py's
single-source /costmap/boundaries feed. Pure-function-level throughout
except test_hard_boundary_actually_constrains_the_solve /
TestInfeasibilityDetection, which do real (light) OSQP solves to prove the
wiring genuinely affects the output, not just the isolated math -- matches
test_solver_rti.py's own "no prior synthetic-scenario test infrastructure
existed" precedent.

TestInfeasibilityDetection covers a real bug THIS task's own testing
found live (not hypothesized in advance): a tight hard boundary combined
with an aggressive initial speed/vdes can make the linearized QP genuinely
INFEASIBLE for that tick's reference trajectory -- OSQP then returns an
INFEASIBILITY CERTIFICATE in results.x, which can be finite (so the old
isfinite()-only success check let it through) while being an arbitrarily
large, physically meaningless number (observed: ~2e9 in a control channel
bounded to [-1.05, 1.05]). Fixed in mpc_solver.py by also checking OSQP's
own status against _OSQP_INFEASIBLE_STATUSES -- see that fix's own
comments for the full story.

Run standalone: python3 -m pytest test/test_boundary_constraints.py -v
"""

import math

import numpy as np
import pytest

from mpc_controller.mpc_solver import (
    BOUNDARY_DISABLED,
    boundary_constraint_bounds,
    pad_boundary_constraints,
    solve_mpc_step,
)
from mpc_controller.MPC_corr import _boundary_to_world, _select_live_boundaries

WHEELBASE = 0.305
LR = 0.17
TS = 0.1
HORIZON = 7
CAR_RADIUS = 0.20
MARGIN = 0.12

LIMITS = {
    "delta_min": -1.05, "delta_max": 1.05,
    "a_min": -2.0, "a_max": 3.0,
    "dDeltaMin": -0.5, "dDeltaMax": 0.5,
    "dAMin": -2.0, "dAMax": 2.0,
    "vMin": -1.0, "vMax": 3.0,
}
WEIGHTS = {
    "w_term": 3.0, "w_v": 8.0, "w_psi": 0.0, "w_u_a": 0.0,
    "w_du_delta": 15.0, "w_du_a": 0.0, "w_delta0": 0.0,
    "w_obs": 8.0, "w_corr": 0.0,
}
PARAMS = {"L": WHEELBASE, "lr": LR}


# ==============================================================================
# pad_boundary_constraints -- fixed-3-slot padding (see mpc_solver.py's own
# module docstring, "Hard boundary constraints" section).
# ==============================================================================

class TestPadBoundaryConstraints:

    @pytest.mark.parametrize("n_active", [0, 1, 2, 3])
    def test_always_returns_exactly_max_sources(self, n_active):
        active = [(1.0, 0.0, 2.0 + i) for i in range(n_active)]
        padded = pad_boundary_constraints(active)
        assert len(padded) == 3, (
            f'{n_active} active constraints must still pad to exactly 3 -- '
            'structurally identical QP shape regardless of how many sources '
            'are actually live this tick'
        )

    def test_more_than_max_sources_truncated_not_errored(self):
        active = [(1.0, 0.0, float(i)) for i in range(5)]
        padded = pad_boundary_constraints(active)
        assert len(padded) == 3
        assert padded == active[:3]

    def test_active_entries_preserved_in_order(self):
        active = [(1.0, 0.0, 2.0), (0.0, 1.0, 1.5)]
        padded = pad_boundary_constraints(active)
        assert padded[0] == active[0]
        assert padded[1] == active[1]

    def test_disabled_slots_use_the_sentinel(self):
        padded = pad_boundary_constraints([(1.0, 0.0, 2.0)])
        assert padded[1] == BOUNDARY_DISABLED
        assert padded[2] == BOUNDARY_DISABLED

    def test_zero_active_all_three_disabled(self):
        padded = pad_boundary_constraints([])
        assert padded == [BOUNDARY_DISABLED, BOUNDARY_DISABLED, BOUNDARY_DISABLED]

    def test_custom_max_sources(self):
        assert len(pad_boundary_constraints([], max_sources=1)) == 1
        assert len(pad_boundary_constraints([(1.0, 0.0, 1.0)] * 2, max_sources=1)) == 1


# ==============================================================================
# boundary_constraint_bounds -- per-stage (lo, hi) OSQP bound for one
# constraint row.
# ==============================================================================

class TestBoundaryConstraintBounds:

    def test_normal_case_subtracts_margin(self):
        lo, hi = boundary_constraint_bounds(1.0, 0.0, 2.0, CAR_RADIUS, MARGIN)
        assert lo == -math.inf
        assert hi == pytest.approx(2.0 - CAR_RADIUS - MARGIN)

    def test_disabled_sentinel_produces_an_unconditionally_satisfied_bound(self):
        nx, ny, offset = BOUNDARY_DISABLED
        lo, hi = boundary_constraint_bounds(nx, ny, offset, CAR_RADIUS, MARGIN)
        assert lo == -math.inf
        assert hi == math.inf, (
            'a disabled slot must produce a very large (here: infinite) hi '
            'bound -- combined with its all-zero normal (applied by the '
            'caller), the resulting row is unconditionally satisfied'
        )

    def test_zero_margin_and_radius_reduces_to_the_raw_offset(self):
        lo, hi = boundary_constraint_bounds(1.0, 0.0, 2.0, 0.0, 0.0)
        assert hi == pytest.approx(2.0)


# ==============================================================================
# _boundary_to_world (MPC_corr.py) -- base_link -> world/odom halfspace
# transform. See that function's own docstring for the derivation.
# ==============================================================================

class TestBoundaryToWorld:

    def test_identity_when_robot_at_origin_facing_forward(self):
        nx, ny, offset = _boundary_to_world(1.0, 0.0, 2.0, 0.0, 0.0, 0.0)
        assert (nx, ny, offset) == pytest.approx((1.0, 0.0, 2.0))

    def test_pure_translation_shifts_offset_only(self):
        # Robot sitting at world (5, 0), facing +x (yaw=0) -- a wall 2m
        # ahead in base_link frame (normal=(1,0), offset=2) is now a wall
        # at world x=7, i.e. world-frame offset=7, normal unchanged.
        nx, ny, offset = _boundary_to_world(1.0, 0.0, 2.0, 5.0, 0.0, 0.0)
        assert (nx, ny) == pytest.approx((1.0, 0.0))
        assert offset == pytest.approx(7.0)

    def test_pure_rotation_rotates_the_normal(self):
        # Robot at world origin but facing world +y (yaw=90deg) -- a wall
        # "ahead" in base_link (normal=(1,0)) is now a wall to the world's
        # +y side (normal=(0,1)); offset (distance from origin) unchanged
        # since the robot hasn't translated.
        nx, ny, offset = _boundary_to_world(1.0, 0.0, 2.0, 0.0, 0.0, math.pi / 2)
        assert (nx, ny) == pytest.approx((0.0, 1.0), abs=1e-9)
        assert offset == pytest.approx(2.0)

    def test_combined_translation_and_rotation(self):
        # Robot at world (1, 1), facing world +y (yaw=90deg), wall 2m
        # ahead in base_link. World-frame normal is (0,1) (from the
        # rotation test above); offset = 2 + dot((0,1), (1,1)) = 2 + 1 = 3.
        nx, ny, offset = _boundary_to_world(1.0, 0.0, 2.0, 1.0, 1.0, math.pi / 2)
        assert (nx, ny) == pytest.approx((0.0, 1.0), abs=1e-9)
        assert offset == pytest.approx(3.0)

    def test_robots_own_base_link_origin_lands_on_the_free_side_in_world_frame(self):
        # Sanity check tying the transform back to the halfspace's own
        # meaning: the robot's OWN current position (robot_x, robot_y),
        # expressed in world frame, must still satisfy normal_w . p <=
        # offset_w for any physically-sane (offset >= 0) base_link boundary.
        robot_x, robot_y, robot_yaw = 3.0, -2.0, math.radians(37.0)
        nx, ny, offset = _boundary_to_world(1.0, 0.0, 1.5, robot_x, robot_y, robot_yaw)
        assert nx * robot_x + ny * robot_y <= offset + 1e-9


# ==============================================================================
# _select_live_boundaries (MPC_corr.py) -- staleness gate for costmap_
# boundary_node.py's single /costmap/boundaries source, factored out of
# MPCController._get_live_boundaries() for pure-function-level testability
# (see that function's own docstring for why -- this replaces the earlier
# two-source OR-combine that existed before the dual-EKF + costmap-derived-
# MPC-boundaries pass collapsed wall_detector_node's/lidar_boundary_node's
# two topics into costmap_boundary_node's one).
# ==============================================================================

class TestSelectLiveBoundaries:

    def test_fresh_values_pass_through_unchanged(self):
        values = [(1.0, 0.0, 2.0)]
        result = _select_live_boundaries(values, last_time=10.0, now_sec=10.2, timeout_sec=0.5)
        assert result == values

    def test_stale_values_return_empty(self):
        values = [(1.0, 0.0, 2.0)]
        result = _select_live_boundaries(values, last_time=10.0, now_sec=11.0, timeout_sec=0.5)
        assert result == []

    def test_never_received_last_time_none_returns_empty(self):
        result = _select_live_boundaries(
            [(1.0, 0.0, 2.0)], last_time=None, now_sec=10.0, timeout_sec=0.5)
        assert result == []

    def test_exactly_at_timeout_boundary_is_stale(self):
        # now_sec - last_time == timeout_sec exactly -- the comparison is
        # strict (<), matching _update_active_odom's own hw/sim odom
        # staleness check this reuses the shape of.
        result = _select_live_boundaries(
            [(1.0, 0.0, 2.0)], last_time=10.0, now_sec=10.5, timeout_sec=0.5)
        assert result == []

    def test_empty_input_list_stays_empty_when_fresh(self):
        result = _select_live_boundaries([], last_time=10.0, now_sec=10.1, timeout_sec=0.5)
        assert result == []

    def test_returns_a_copy_not_the_same_list_object(self):
        values = [(1.0, 0.0, 2.0)]
        result = _select_live_boundaries(values, last_time=10.0, now_sec=10.1, timeout_sec=0.5)
        assert result is not values
        assert result == values


# ==============================================================================
# End-to-end: a hard boundary constraint genuinely changes the solve output
# (not just the isolated padding/bounds math) -- ONE light real solve, per
# the task's own "verify correct rows/bounds at each stage" ask, kept to a
# single scenario since the padding/bounds unit tests above already cover
# the construction exhaustively.
# ==============================================================================

def _straight_corridor(x0, length=5.0, wmin=2.0, wmax=2.0, n=200):
    X0, Y0, psi0 = float(x0[0]), float(x0[1]), float(x0[2])
    u = np.linspace(0.0, 1.0, n)
    s = length * u
    xc = X0 + s * np.cos(psi0)
    yc = Y0 + s * np.sin(psi0)
    half_width = np.full(n, wmin + (wmax - wmin) * u)
    nx = np.full(n, -np.sin(psi0))
    ny = np.full(n, np.cos(psi0))
    return {
        "xc": xc, "yc": yc,
        "xL": xc + wmin * nx, "yL": yc + wmin * ny,
        "xR": xc - wmin * nx, "yR": yc - wmin * ny,
        "tx": np.full(n, np.cos(psi0)), "ty": np.full(n, np.sin(psi0)),
        "nx": nx, "ny": ny,
        "halfWidth": half_width,
        "psiRef": psi0, "t": 0.0,
        "Pend": np.array([xc[-1], yc[-1]]),
        "dFront": 10.0, "dpsi": 0.0,
        "obstacles_world": [],
        "car_radius": CAR_RADIUS,
        "avoidance_margin": MARGIN,
    }


class TestHardBoundaryEndToEnd:

    def test_hard_boundary_actually_constrains_the_solve(self):
        # Wide-open straight corridor, no soft obstacles at all, gentle
        # initial speed (0.3 m/s, matching test_solver_rti.py's own
        # "moderate speed" scenarios -- an aggressive initial speed/vdes
        # here pushes this short (7-stage/0.7s) horizon's linearization
        # into a much more nonlinear regime, which isn't what this test is
        # checking). Without a boundary, the solver pushes X forward
        # toward the pref_nom target 5m ahead. With a hard boundary wall
        # at world x=0.6m (robot starts at the origin facing +x), the
        # predicted trajectory must stay behind that wall (minus margin)
        # at every stage.
        x0 = np.array([0.0, 0.0, 0.0, 0.3])
        corridor = _straight_corridor(x0)
        last_u = np.array([0.0, 0.0])
        common_kwargs = dict(
            x0=x0, last_u=last_u, pref_nom=corridor["Pend"], corridor=corridor,
            horizon=HORIZON, ts=TS, params=PARAMS, limits=LIMITS, weights=WEIGHTS,
            obstacles=[], dmin=0.9, vdes=1.0, solver='rti',
        )

        wall_offset = 0.6
        limit = wall_offset - CAR_RADIUS - MARGIN

        _, info_unconstrained = solve_mpc_step(**common_kwargs)
        x_max_unconstrained = max(s[0] for s in info_unconstrained["x_pred"])
        assert x_max_unconstrained > limit, (
            'fixture sanity check: without a boundary the trajectory should '
            'clear the intended limit on its own, so the constrained case '
            'below actually exercises the boundary, not a no-op'
        )

        boundary = [(1.0, 0.0, wall_offset)]  # normal +x, wall at world x=0.6
        _, info_constrained = solve_mpc_step(**common_kwargs, boundaries=boundary)

        x_max_constrained = max(s[0] for s in info_constrained["x_pred"])
        assert x_max_constrained <= limit + 1e-6, (
            f'predicted trajectory crossed the hard boundary: '
            f'x_max={x_max_constrained:.4f} > limit={limit:.4f}'
        )
        assert x_max_constrained < x_max_unconstrained, (
            'the boundary should measurably hold the trajectory back '
            'compared to the unconstrained case'
        )


# ==============================================================================
# Regression test for a real bug this task's own testing found live -- see
# module docstring's own note. A tight hard boundary + an aggressive
# initial-speed scenario the reference-trajectory linearization can't
# satisfy must fall back to the warm start (last_u), never hand back an
# out-of-bounds control command.
# ==============================================================================

class TestInfeasibilityDetection:

    def test_infeasible_boundary_scenario_falls_back_not_garbage(self):
        # Same aggressive scenario that originally surfaced this: fast
        # initial speed accelerating hard toward vdes, with a tight wall
        # close enough that the reference trajectory (built by forward-
        # simulating the warm start BEFORE linearizing) overshoots what the
        # actual constrained QP can achieve -- genuinely infeasible for
        # THIS tick's linearization, not just a tight-but-solvable case.
        x0 = np.array([0.0, 0.0, 0.0, 1.0])
        corridor = _straight_corridor(x0)
        last_u = np.array([0.3, 0.0])
        boundary = [(1.0, 0.0, 0.6)]

        u0, info = solve_mpc_step(
            x0=x0, last_u=last_u, pref_nom=corridor["Pend"], corridor=corridor,
            horizon=HORIZON, ts=TS, params=PARAMS, limits=LIMITS, weights=WEIGHTS,
            obstacles=[], dmin=0.9, vdes=2.0, solver='rti', boundaries=boundary,
        )

        # The fixture itself must actually be infeasible (not a stale
        # scenario that happens to solve cleanly now) -- otherwise this
        # test would silently stop testing anything.
        assert not info["success"], (
            'fixture sanity check: this scenario is expected to be '
            'genuinely infeasible for the RTI linearization -- if it now '
            'solves cleanly, replace it with one that still reproduces '
            'the infeasible-certificate case this test guards against'
        )

        # The REAL bug: u0 must never leave the declared control bounds,
        # regardless of solver status -- this is what actually reaches
        # AckermannDriveStamped and the real vehicle.
        assert LIMITS["delta_min"] - 1e-6 <= u0[0] <= LIMITS["delta_max"] + 1e-6, (
            f'steering command {u0[0]} escaped its own declared bounds '
            f'[{LIMITS["delta_min"]}, {LIMITS["delta_max"]}] -- an '
            'infeasibility certificate leaked through as a real command'
        )
        assert LIMITS["a_min"] - 1e-6 <= u0[1] <= LIMITS["a_max"] + 1e-6, (
            f'acceleration command {u0[1]} escaped its own declared bounds '
            f'[{LIMITS["a_min"]}, {LIMITS["a_max"]}] -- an infeasibility '
            'certificate leaked through as a real command'
        )
        # Specifically: falls back to holding last_u (the documented
        # fallback), not merely "happens to be in-bounds".
        assert u0 == pytest.approx(last_u)


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
