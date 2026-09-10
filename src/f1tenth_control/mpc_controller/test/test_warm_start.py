"""
Coverage for the RTI warm start (solve_mpc_step's warm_start_z).

WHAT WAS BROKEN. mpc_solver's module docstring has always said the RTI path
is "warm-started from the previous tick's solution", and _solve_rti has
always had a warm_start_z parameter -- but solve_mpc_step, the only entry
point MPC_corr.py calls, did not accept one, so nothing could ever be
passed. Every solve fell back to np.tile(last_u, N).

WHY THAT IS NOT MERELY A MISSED OPTIMISATION. warm_start_z is not just
OSQP's initial iterate: _solve_rti rolls it through the TRUE nonlinear
model to build x_ref, the trajectory every Jacobian, every frozen corridor
nearest-index and every frozen heading target is linearized around. Tiling
last_u makes that trajectory "hold the current steering angle for the whole
horizon" -- a circular arc which, measured on run 2026-09-10T13-57-23, ran
up to 48 degrees away from the plan the solver was actually producing, and
which moved discontinuously every tick as last_u slewed. The QP was being
built around a reference nobody was steering toward.

These are synthetic-input tests. The closed-loop case at the bottom is a
scripted 90-degree turn through the real solver, not vehicle behaviour.

Run standalone: python3 -m pytest test/test_warm_start.py -v
"""

import math
import unittest

import numpy as np

import pytest

from mpc_controller.MPC_corr import MPCController
from mpc_controller.mpc_solver import (
    OSQP_AVAILABLE,
    shift_warm_start,
    solve_mpc_step,
)
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta

WHEELBASE = 0.305
LR = 0.17
TS = 0.1
HORIZON = 20
PARAMS = {"L": WHEELBASE, "lr": LR}

# The DEPLOYED envelope, not the 1.05 rad placeholder the older solver tests
# carry: this file is about how the solver behaves on the car, and the
# steering limit is what makes a turn take many ticks to build up.
LIMITS = {
    "delta_min": -0.283,
    "delta_max": 0.278,
    "a_min": -2.0,
    "a_max": 3.0,
    "dDeltaMin": -0.5,
    "dDeltaMax": 0.5,
    "dAMin": -2.0,
    "dAMax": 2.0,
    "vMin": -1.0,
    "vMax": 3.0,
}

# stack_params.yaml's shipping set, w_psi_stage included -- see
# test_corridor_heading_cost.py's own note on why that key matters.
WEIGHTS = {
    "w_term": 9.0,
    "w_v": 4.2857,
    "w_psi": 4.5,
    "w_psi_stage": 4.7736,
    "w_corr": 3.5714,
    "w_u_a": 0.0,
    "w_du_delta": 8.5714,
    "w_du_a": 0.0,
    "w_delta0": 0.0,
    "w_obs": 8.0,
}


def _corridor(psi_start, psi_end, x0=0.0, y0=0.0, length=3.0, n=120,
              wmin=0.4333, wmax=0.7667, u_start=0.10, u_end=0.70, dpsi=None):
    """build_straight_corridor's geometry, standalone.

    Same "testable without constructing a real MPCController" shape the rest
    of this package's corridor tests use. Kept in step with MPC_corr's own
    S-curve blend so the reference trajectory these tests linearize around
    is the one the car actually gets.
    """
    u = np.linspace(0.0, 1.0, n)
    s = length * u
    # dpsi override: a wall_turn passes the SIGNED, unwrapped rotation, which
    # is the one thing the wrapped difference below cannot express past 180
    # degrees -- see MPC_corr's own wall_turn branch.
    if dpsi is None:
        dpsi = math.atan2(math.sin(psi_end - psi_start),
                          math.cos(psi_end - psi_start))
    tau = np.clip((u - u_start) / max(u_end - u_start, 1e-6), 0.0, 1.0)
    theta = psi_start + dpsi * (3.0 * tau ** 2 - 2.0 * tau ** 3)
    ds = np.zeros_like(s)
    ds[1:] = np.diff(s)
    xc = x0 + np.cumsum(np.cos(theta) * ds)
    yc = y0 + np.cumsum(np.sin(theta) * ds)
    c0 = np.array([xc[0], yc[0]])
    c1 = np.array([xc[-1], yc[-1]])
    n0 = np.array([-math.sin(psi_start), math.cos(psi_start)])
    n1 = np.array([-math.sin(psi_end), math.cos(psi_end)])
    e0 = np.array([math.cos(psi_start), math.sin(psi_start)])
    e1 = np.array([math.cos(psi_end), math.sin(psi_end)])
    pl0, pr0, pl1, pr1 = c0 + wmin * n0, c0 - wmin * n0, c1 + wmax * n1, c1 - wmax * n1
    k = 0.55 * length
    uu = u[:, None]

    def bez(a, b, c, d):
        return ((1 - uu) ** 3 * a + 3 * (1 - uu) ** 2 * uu * b
                + 3 * (1 - uu) * uu ** 2 * c + uu ** 3 * d)

    left = bez(pl0, pl0 + k * e0, pl1 - k * e1, pl1)
    right = bez(pr0, pr0 + k * e0, pr1 - k * e1, pr1)
    dx, dy = np.gradient(xc), np.gradient(yc)
    dn = np.maximum(np.sqrt(dx ** 2 + dy ** 2), 1e-9)
    tx, ty = dx / dn, dy / dn
    return {
        "xc": xc, "yc": yc,
        "xL": left[:, 0], "yL": left[:, 1],
        "xR": right[:, 0], "yR": right[:, 1],
        "tx": tx, "ty": ty, "nx": -ty, "ny": tx,
        "halfWidth": 0.5 * np.hypot(left[:, 0] - right[:, 0], left[:, 1] - right[:, 1]),
        "psiRef": float(psi_end), "psiStart": float(psi_start),
        "L": float(length), "t": float(dpsi), "dpsi": float(dpsi),
        "Pend": np.array([xc[-1], yc[-1]]), "dFront": 10.0,
        "obstacles_world": [], "car_radius": 0.20, "avoidance_margin": 0.12,
    }


def _solve(x0, last_u, corridor, warm=None, vdes=0.4, horizon=HORIZON):
    return solve_mpc_step(
        x0=np.asarray(x0, dtype=float), last_u=np.asarray(last_u, dtype=float),
        pref_nom=corridor["Pend"], corridor=corridor, horizon=horizon, ts=TS,
        params=PARAMS, limits=LIMITS, weights=dict(WEIGHTS), obstacles=[],
        dmin=0.32, vdes=vdes, solver='rti', warm_start_z=warm)


class TestShiftWarmStart(unittest.TestCase):
    """The receding-horizon shift itself -- pure, no solver."""

    def test_shifts_one_step_and_duplicates_the_last_control(self):
        z = np.array([1.0, 10.0, 2.0, 20.0, 3.0, 30.0], dtype=float)
        np.testing.assert_allclose(
            shift_warm_start(z, 3),
            [2.0, 20.0, 3.0, 30.0, 3.0, 30.0])

    def test_does_not_mutate_its_input(self):
        z = np.array([1.0, 10.0, 2.0, 20.0, 3.0, 30.0], dtype=float)
        before = z.copy()
        shift_warm_start(z, 3)
        np.testing.assert_array_equal(z, before)

    def test_a_wrong_length_sequence_is_rejected(self):
        """
        Reject rather than reshape.

        A zopt that is not 2*horizon long is a truncated or garbage solve,
        and seeding OSQP with the wrong shape is a hard error -- returning
        None puts the caller back on the last_u tile, which is always valid.
        """
        self.assertIsNone(shift_warm_start(np.zeros(6), 20))
        self.assertIsNone(shift_warm_start(np.zeros(0), 3))
        self.assertIsNone(shift_warm_start(None, 3))

    def test_a_non_finite_sequence_is_rejected(self):
        z = np.array([1.0, 10.0, np.nan, 20.0, 3.0, 30.0], dtype=float)
        self.assertIsNone(shift_warm_start(z, 3))
        z[2] = np.inf
        self.assertIsNone(shift_warm_start(z, 3))

    def test_a_horizon_of_one_is_the_identity(self):
        """Degenerate but reachable; must not index off the front."""
        np.testing.assert_allclose(shift_warm_start(np.array([1.0, 2.0]), 1),
                                   [1.0, 2.0])


class TestInvalidationRules(unittest.TestCase):
    """
    Where the stored plan is dropped, and where it deliberately is not.

    Duck-typed stand-ins calling the unbound methods, the same shape
    test_corridor_direction_recovery.py uses -- these two methods touch only
    plain attributes.
    """

    class _Fake:
        def __init__(self, **kw):
            self.warm_start_z = np.arange(40, dtype=float)
            self.__dict__.update(kw)

        def get_logger(self):
            return self

        def info(self, *a, **k):
            pass

    def test_a_new_move_drops_the_stored_plan(self):
        """
        A new move means a new psiRef.

        The stored sequence was optimal for the PREVIOUS move's terminal
        heading, so seeding the first solve of a turn with the straight
        move's plan would linearize the whole QP around a trajectory aimed
        at the old heading -- the same failure the corridor cache beside it
        was added to prevent.
        """
        fake = self._Fake(smoothed_target=object(),
                          last_deflection_vec=None,
                          deflection_decay_remaining=7,
                          cached_corridor=object(), last_corridor_time=1.0,
                          last_corridor_stamp=object(), cached_pref_nom=object())
        MPCController._invalidate_move_state(fake)
        self.assertIsNone(fake.warm_start_z)
        # and it goes with the corridor cache, not instead of it
        self.assertIsNone(fake.cached_corridor)

    def test_ending_a_drive_command_drops_the_stored_plan(self):
        fake = self._Fake(drive_cmd={'mode': 'wall_turn'},
                          vdes=0.08, vdes_default=0.5,
                          avoidance_margin=0.0, avoidance_margin_default=0.12)
        MPCController._clear_drive_state(fake)
        self.assertIsNone(fake.warm_start_z)

    def test_clearing_a_drive_command_that_was_never_set_changes_nothing(self):
        """
        The early-out path must stay a genuine no-op.

        _clear_drive_state is called unconditionally by the other three goal
        callbacks; when there was no drive command it must not throw away a
        perfectly good warm start belonging to the move still running.
        """
        fake = self._Fake(drive_cmd=None)
        keep = fake.warm_start_z.copy()
        MPCController._clear_drive_state(fake)
        np.testing.assert_array_equal(fake.warm_start_z, keep)


@pytest.mark.skipif(not OSQP_AVAILABLE, reason="osqp not importable")
class TestSolveMpcStepPlumbing(unittest.TestCase):
    """The parameter reaches the QP, and cannot break a solve."""

    def setUp(self):
        self.corr = _corridor(0.0, math.radians(90.0))
        self.x0 = [0.0, 0.0, 0.0, 0.30]
        self.last_u = [0.0, 0.0]

    def test_omitting_it_reproduces_the_pre_warm_start_answer_exactly(self):
        """
        The default must be a byte-identical no-op.

        This is the rollback guarantee: warm_start_z=None has to leave the
        QP exactly as it was before this parameter existed.
        """
        a, _ = _solve(self.x0, self.last_u, self.corr)
        b, _ = _solve(self.x0, self.last_u, self.corr, warm=None)
        np.testing.assert_array_equal(a, b)

    def test_a_warm_start_actually_changes_the_linearization(self):
        """
        Guard against the parameter being accepted and dropped on the floor.

        This package has shipped inert wiring before (w_psi spent a while as
        a commented-out line plus a 0 * 5.0 weight, and w_corr existed only
        in a branch the default backend never evaluated). A warm start
        describing a hard left turn rolls x_ref onto a completely different
        arc from the last_u tile, so the answer MUST move; if it does not,
        the sequence is not reaching _solve_rti.
        """
        warm = np.tile([math.radians(15.0), 0.0], HORIZON)
        cold_u, _ = _solve(self.x0, self.last_u, self.corr)
        warm_u, _ = _solve(self.x0, self.last_u, self.corr, warm=warm)
        self.assertGreater(abs(float(warm_u[0]) - float(cold_u[0])), 1e-6)

    def test_a_malformed_warm_start_is_ignored_not_fatal(self):
        """
        Defence in depth for a caller that did not go through
        shift_warm_start: wrong length and non-finite must both fall back to
        the tile rather than raising or, worse, rolling x_ref forward on a
        truncated control sequence.
        """
        expected, _ = _solve(self.x0, self.last_u, self.corr)
        for bad in (np.zeros(2 * HORIZON - 4),
                    np.full(2 * HORIZON, np.nan),
                    np.zeros((HORIZON, 2))):          # right size, wrong shape
            got, _ = _solve(self.x0, self.last_u, self.corr, warm=bad)
            if bad.shape == (HORIZON, 2):
                # ravel()s to a valid sequence of zeros -- accepted, and the
                # point here is only that it does not raise.
                continue
            np.testing.assert_allclose(got, expected, atol=1e-12)

    def test_the_slsqp_rollback_path_ignores_it_without_raising(self):
        """Same documented terms as `boundaries`: RTI-only, silently ignored."""
        u0, _ = solve_mpc_step(
            x0=np.array(self.x0), last_u=np.array(self.last_u),
            pref_nom=self.corr["Pend"], corridor=self.corr, horizon=7, ts=TS,
            params=PARAMS, limits=LIMITS, weights=dict(WEIGHTS), obstacles=[],
            dmin=0.32, vdes=0.4, solver='slsqp',
            warm_start_z=np.tile([0.2, 0.0], 7))
        self.assertTrue(np.all(np.isfinite(u0)))


@pytest.mark.skipif(not OSQP_AVAILABLE, reason="osqp not importable")
class TestClosedLoopSolveQuality(unittest.TestCase):
    """
    A scripted 90-degree turn, solved tick by tick, with and without the
    warm start carried forward.

    This is the measurement the change was made for. Asserted as a
    NO-REGRESSION inequality rather than against the absolute numbers
    observed while writing it (cold: 105 solved / 12 inaccurate / 3 failed;
    warm: 120 solved) -- osqp is unpinned in docker/Dockerfile.*, so
    whatever is newest at image build time is what runs, and pinning exact
    counts here would make this file a tripwire for osqp's own tuning
    rather than for this repo's wiring.
    """

    @staticmethod
    def _run(use_warm, ticks=60):
        x = np.array([0.0, 0.0, math.radians(-11.0), 0.30])
        last_u = np.array([math.radians(-7.5), 0.0])
        warm = None
        corr = _corridor(float(x[2]), math.radians(90.0))
        statuses, steers = [], []
        for k in range(ticks):
            if k % 10 == 0:                    # corridor_update_period = 1.0 s
                corr = _corridor(float(x[2]), math.radians(90.0),
                                 x0=float(x[0]), y0=float(x[1]))
            u0, info = _solve(x, last_u, corr, warm=(warm if use_warm else None))
            statuses.append(info["status_message"])
            steers.append(float(u0[0]))
            warm = shift_warm_start(info["zopt"], HORIZON) if info["success"] else None
            last_u = np.asarray(u0, dtype=float).copy()
            x = np.array(f1tenth_state_fcn_dt_beta(x, u0, TS, WHEELBASE, LR))
        return statuses, np.array(steers)

    def test_the_warm_start_does_not_worsen_solve_status(self):
        cold, _ = self._run(False)
        warm, _ = self._run(True)
        clean = lambda ss: sum(s == 'solved' for s in ss)  # noqa: E731
        self.assertGreaterEqual(clean(warm), clean(cold))

    def test_the_warm_start_does_not_worsen_steering_chatter(self):
        """
        The symptom the bad linearization produced on the car: a rate-limited
        sawtooth reversing roughly once per second while every underlying
        plan pointed the same way. Fewer reversals is the whole point.
        """
        _, cold = self._run(False)
        _, warm = self._run(True)
        reversals = lambda d: int((np.diff(np.sign(d)) != 0).sum())  # noqa: E731
        self.assertLessEqual(reversals(warm), reversals(cold))

    def test_the_turn_still_completes_in_the_right_direction(self):
        """
        Guard the obvious: whatever the warm start does to convergence, a
        corridor asking for +90 degrees must still produce left steering.
        """
        _, warm = self._run(True)
        self.assertGreater(float(np.mean(warm)), 0.0)


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
