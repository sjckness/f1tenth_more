"""Per-stage path tracking: the cross-track term and the new heading term.

WHAT THE WORK ORDER ASKED FOR, AND WHAT WAS ALREADY THERE. The brief asked
whether w_corr is wired into the RTI path or only into planner_cost_corridor
(which the default RTI backend never calls). It is wired -- has been since
the pass that enabled it -- and it is already exactly the shape the brief
specified:

  - PER STAGE. It is applied at every k in the horizon loop, and it is in
    _STAGE_WEIGHT_KEYS so scale_stage_weights normalises it.
  - CROSS-TRACK ONLY. The row is n . (p - pc) with n the corridor NORMAL at
    the nearest centreline index, so the penalised quantity is the
    perpendicular distance to the path and nothing else. There is no
    longitudinal component to fight w_v with -- not "small", zero, by
    construction.

The brief's 2b (sample the centreline at s_k = s_0 + v_pred*k*ts and take
cross-track there) was NOT implemented, because nearest-index projection is
the same idea with strictly less that can go wrong: a scheduled sample is
only in the right place while the car is on schedule, and being off schedule
would move the reference point along the path -- reintroducing exactly the
longitudinal coupling 2c exists to avoid. Nearest-index has no schedule to
be wrong about. Pinned below so the choice is visible rather than implied.

WHAT IS ACTUALLY NEW HERE is w_psi_stage, a per-stage HEADING cost. Nothing
in either backend penalised heading anywhere except at the terminal state,
so the weight set that follows had nothing to attach its w_psi_stage 1.67
to. It is wired in both backends and left at 0.0, so this commit changes
nothing the car can feel at the time it landed -- the weight-set commit
that follows turns it on, and test_weight_set.py pins what it turns it on
to.

Run standalone: python3 -m pytest test/test_stage_tracking.py -v
"""

import math

import numpy as np
import pytest

from mpc_controller.mpc_solver import (
    _STAGE_WEIGHT_KEYS,
    corridor_heading_at,
    scale_stage_weights,
    solve_mpc_step,
    unwrapped_heading_target,
)

HORIZON = 20
TS = 0.1
PARAMS = {"L": 0.305, "lr": 0.17}
LIMITS = {
    "delta_min": -0.283, "delta_max": 0.278,
    "a_min": -2.0, "a_max": 3.0,
    "dDeltaMin": -0.5, "dDeltaMax": 0.5,
    "dAMin": -2.0, "dAMax": 2.0,
    "vMin": -1.0, "vMax": 3.0,
}
BASE_WEIGHTS = {
    "w_term": 3.0, "w_v": 8.0, "w_psi": 1.5, "w_u_a": 0.0,
    "w_du_delta": 15.0, "w_du_a": 0.0, "w_delta0": 0.0,
    "w_obs": 8.0, "w_corr": 0.0, "w_psi_stage": 0.0,
}


def _straight_corridor(psi=0.0, n=120, length=3.0, origin=(0.0, 0.0)):
    """Straight centreline from `origin` along `psi`, with the tx/ty/nx/ny
    frames build_straight_corridor produces."""
    s = np.linspace(0.0, length, n)
    xc = origin[0] + s * math.cos(psi)
    yc = origin[1] + s * math.sin(psi)
    tx = np.full(n, math.cos(psi))
    ty = np.full(n, math.sin(psi))
    return {
        "xc": xc, "yc": yc,
        "tx": tx, "ty": ty,
        "nx": -ty, "ny": tx,
        "halfWidth": np.full(n, 0.4333),
        "L": float(length),
        "obstacles_world": [],
        "d_safe": 0.32,
        "car_radius": 0.20,
        "avoidance_margin": 0.12,
        "Pend": np.array([xc[-1], yc[-1]], dtype=float),
        "psiRef": psi,
    }


# =====================================================================
# 2a. w_corr was already wired, per-stage, and already cross-track.
# =====================================================================
class TestCrossTrackWasAlreadyWired:

    def test_w_corr_is_normalised_as_a_stage_weight(self):
        assert "w_corr" in _STAGE_WEIGHT_KEYS
        scaled = scale_stage_weights({"w_corr": 20.0}, horizon=20)
        assert scaled["w_corr"] == pytest.approx(7.0)  # 20 * 7/20

    def test_it_changes_the_rti_solve_which_is_the_default_backend(self):
        """The question the brief actually asked: does it reach the QP, or
        only planner_cost_corridor? It reaches the QP.

        Measured as the INTEGRATED cross-track error over the horizon, not
        the error at the last stage. That is the quantity a stage cost
        actually minimises, and using the endpoint instead gives the wrong
        answer here -- see
        test_at_the_old_weights_it_overshoots_the_centreline below, which
        pins the reason.
        """
        corridor = _straight_corridor()
        # Start 0.25 m off the centreline so a centring term has work to do.
        common = dict(
            x0=np.array([0.5, 0.25, 0.0, 0.5]), last_u=np.array([0.0, 0.0]),
            pref_nom=corridor["Pend"], corridor=corridor, horizon=HORIZON,
            ts=TS, params=PARAMS, limits=LIMITS, obstacles=[], dmin=0.9,
            vdes=0.5, solver='rti')

        _u_off, info_off = solve_mpc_step(weights=dict(BASE_WEIGHTS), **common)
        _u_on, info_on = solve_mpc_step(
            weights=dict(BASE_WEIGHTS, w_corr=20.0), **common)

        integ_off = sum(abs(s[1]) for s in info_off["x_pred"])
        integ_on = sum(abs(s[1]) for s in info_on["x_pred"])
        assert integ_on < integ_off, (
            'w_corr did not pull the horizon toward the centreline -- '
            'if this fails it is inert in the RTI path again')

    def test_at_the_old_weights_it_overshoots_the_centreline(self):
        """A finding, pinned so the next tuning pass does not rediscover it.

        Turning w_corr on against the OLD weight set makes the horizon cross
        the centreline and end on the far side: end-of-horizon lateral goes
        0.068 -> -0.149 m even though the integrated error improves
        3.39 -> 2.67. It is doing its job -- a stage cost minimises the sum,
        and buying that sum early costs an overshoot later, because
        w_du_delta is the only thing resisting and at the old set its
        effective value is 5.25.

        This is why the weight set that follows cannot be read one term at a
        time: w_corr's useful magnitude is coupled to whatever is damping
        the steering.
        """
        corridor = _straight_corridor()
        common = dict(
            x0=np.array([0.5, 0.25, 0.0, 0.5]), last_u=np.array([0.0, 0.0]),
            pref_nom=corridor["Pend"], corridor=corridor, horizon=HORIZON,
            ts=TS, params=PARAMS, limits=LIMITS, obstacles=[], dmin=0.9,
            vdes=0.5, solver='rti')
        _u, info = solve_mpc_step(
            weights=dict(BASE_WEIGHTS, w_corr=20.0), **common)
        ys = [float(s[1]) for s in info["x_pred"]]
        assert ys[0] > 0.0
        assert min(ys) < 0.0, 'expected the horizon to cross the centreline'

    def test_the_penalised_quantity_has_no_longitudinal_component(self):
        """2c, checked on the geometry rather than taken on trust.

        Sliding the car ALONG a straight corridor changes nothing about its
        cross-track error, so a purely cross-track term must produce an
        IDENTICAL lateral outcome from either start. A full 2D position
        error would not: it would also be pulling on the along-track
        distance to pref_nom, and the two starts would differ.

        w_term and w_psi are zeroed here on purpose. Both reference the
        corridor ENDPOINT, whose distance genuinely does depend on where
        along the corridor the car starts, so leaving them in would measure
        their translation dependence rather than w_corr's absence of one.
        """
        corridor = _straight_corridor()
        weights = dict(BASE_WEIGHTS, w_corr=20.0, w_term=0.0, w_psi=0.0)
        lat = []
        for x_start in (0.4, 1.2):
            _u, info = solve_mpc_step(
                x0=np.array([x_start, 0.20, 0.0, 0.5]),
                last_u=np.array([0.0, 0.0]), pref_nom=corridor["Pend"],
                corridor=corridor, horizon=HORIZON, ts=TS, params=PARAMS,
                limits=LIMITS, weights=weights, obstacles=[], dmin=0.9,
                vdes=0.5, solver='rti')
            lat.append(abs(info["x_pred"][-1][1]))
        # Identical to five decimals, not merely close.
        assert lat[0] == pytest.approx(lat[1], abs=1e-5)

    def test_and_without_it_nothing_corrects_the_lateral_error_at_all(self):
        """The counterpart to the test above: with w_term/w_psi zeroed and
        w_corr off, the horizon holds its 0.20 m offset -- there is no other
        stage term that references the path laterally. That is the gap the
        weight set is being asked to close."""
        corridor = _straight_corridor()
        weights = dict(BASE_WEIGHTS, w_corr=0.0, w_term=0.0, w_psi=0.0)
        _u, info = solve_mpc_step(
            x0=np.array([0.4, 0.20, 0.0, 0.5]), last_u=np.array([0.0, 0.0]),
            pref_nom=corridor["Pend"], corridor=corridor, horizon=HORIZON,
            ts=TS, params=PARAMS, limits=LIMITS, weights=weights,
            obstacles=[], dmin=0.9, vdes=0.5, solver='rti')
        assert abs(info["x_pred"][-1][1]) == pytest.approx(0.20, abs=1e-3)


# =====================================================================
# The new stage heading term.
# =====================================================================
class TestCorridorHeadingHelper:

    def test_the_tangent_is_recovered_from_the_stored_normal(self):
        for psi in (0.0, 0.7, -1.3, 3.0):
            corridor = _straight_corridor(psi=psi)
            got = corridor_heading_at(np.array([0.5, 0.0]), corridor)
            assert math.cos(got - psi) == pytest.approx(1.0, abs=1e-9)

    def test_it_needs_no_key_the_lateral_bound_does_not_already_need(self):
        """Deriving the tangent from nx/ny rather than tx/ty means every
        corridor dict that can supply the lateral bound can also supply the
        heading -- no caller or test stand-in has to grow a field."""
        corridor = _straight_corridor(psi=0.4)
        del corridor["tx"]
        del corridor["ty"]
        assert corridor_heading_at(np.array([0.5, 0.0]), corridor) == pytest.approx(0.4)

    def test_the_target_is_unwrapped_onto_the_linearization_branch(self):
        """psi is unbounded in this model, so a raw target near +-pi would
        ask for a ~2pi turn to reach an orientation the car is already at."""
        # Car at +3.10 rad, reference at -3.10 rad: really a 0.083 rad turn.
        out = unwrapped_heading_target(-3.10, 3.10)
        assert out == pytest.approx(3.10 + 0.0831, abs=1e-3)
        assert abs(out - 3.10) < 0.2

    def test_unwrapping_is_identity_when_already_on_the_same_branch(self):
        assert unwrapped_heading_target(0.3, 0.25) == pytest.approx(0.3)


class TestStageHeadingCost:

    def test_it_is_normalised_as_a_stage_weight(self):
        assert "w_psi_stage" in _STAGE_WEIGHT_KEYS

    def test_it_straightens_the_horizon_onto_the_corridor_heading(self):
        corridor = _straight_corridor()
        # Car on the line but yawed 0.30 rad off it.
        x0 = np.array([0.5, 0.0, 0.30, 0.5])
        common = dict(
            x0=x0, last_u=np.array([0.0, 0.0]), pref_nom=corridor["Pend"],
            corridor=corridor, horizon=HORIZON, ts=TS, params=PARAMS,
            limits=LIMITS, obstacles=[], dmin=0.9, vdes=0.5, solver='rti')

        _u_off, info_off = solve_mpc_step(weights=dict(BASE_WEIGHTS), **common)
        _u_on, info_on = solve_mpc_step(
            weights=dict(BASE_WEIGHTS, w_psi_stage=30.0), **common)

        assert abs(info_on["x_pred"][-1][2]) < abs(info_off["x_pred"][-1][2]), (
            'the stage heading cost did not reduce the horizon-end heading error')

    def test_it_acts_at_every_stage_not_only_the_last(self):
        """The distinction from the terminal w_psi, which is the whole
        reason this term exists: the heading error must come down EARLY in
        the horizon, not merely arrive at zero at the end."""
        corridor = _straight_corridor()
        common = dict(
            x0=np.array([0.5, 0.0, 0.30, 0.5]), last_u=np.array([0.0, 0.0]),
            pref_nom=corridor["Pend"], corridor=corridor, horizon=HORIZON,
            ts=TS, params=PARAMS, limits=LIMITS, obstacles=[], dmin=0.9,
            vdes=0.5, solver='rti')

        _u_t, info_terminal = solve_mpc_step(
            weights=dict(BASE_WEIGHTS, w_psi=30.0), **common)
        _u_s, info_stage = solve_mpc_step(
            weights=dict(BASE_WEIGHTS, w_psi_stage=30.0), **common)

        mid = HORIZON // 2
        assert abs(info_stage["x_pred"][mid][2]) < abs(info_terminal["x_pred"][mid][2])

    def test_it_reaches_the_reported_true_cost_too(self):
        """planner_cost_corridor is where info['cost'] comes from, including
        for RTI solves, so both must agree on what a solution costs."""
        corridor = _straight_corridor()
        common = dict(
            x0=np.array([0.5, 0.0, 0.30, 0.5]), last_u=np.array([0.0, 0.0]),
            pref_nom=corridor["Pend"], corridor=corridor, horizon=HORIZON,
            ts=TS, params=PARAMS, limits=LIMITS, obstacles=[], dmin=0.9,
            vdes=0.5, solver='rti')
        _u_off, info_off = solve_mpc_step(weights=dict(BASE_WEIGHTS), **common)
        _u_on, info_on = solve_mpc_step(
            weights=dict(BASE_WEIGHTS, w_psi_stage=30.0), **common)
        assert info_on["cost"] != info_off["cost"]

    def test_a_corridor_already_aligned_costs_nothing_extra(self):
        """Sanity on the sign/branch handling: no heading error, no penalty,
        so the solution must be the one the term-free problem gives."""
        corridor = _straight_corridor()
        common = dict(
            x0=np.array([0.5, 0.0, 0.0, 0.5]), last_u=np.array([0.0, 0.0]),
            pref_nom=corridor["Pend"], corridor=corridor, horizon=HORIZON,
            ts=TS, params=PARAMS, limits=LIMITS, obstacles=[], dmin=0.9,
            vdes=0.5, solver='rti')
        u_off, _i = solve_mpc_step(weights=dict(BASE_WEIGHTS), **common)
        u_on, _j = solve_mpc_step(
            weights=dict(BASE_WEIGHTS, w_psi_stage=30.0), **common)
        assert np.allclose(u_off, u_on, atol=1e-4)

    def test_it_works_on_a_corridor_that_is_not_axis_aligned(self):
        """Guards the tangent recovery: an axis-aligned fixture would pass
        even if nx/ny were swapped."""
        psi = 0.9
        corridor = _straight_corridor(psi=psi)
        common = dict(
            x0=np.array([0.5 * math.cos(psi), 0.5 * math.sin(psi), psi + 0.30, 0.5]),
            last_u=np.array([0.0, 0.0]), pref_nom=corridor["Pend"],
            corridor=corridor, horizon=HORIZON, ts=TS, params=PARAMS,
            limits=LIMITS, obstacles=[], dmin=0.9, vdes=0.5, solver='rti')
        _u_off, info_off = solve_mpc_step(weights=dict(BASE_WEIGHTS), **common)
        _u_on, info_on = solve_mpc_step(
            weights=dict(BASE_WEIGHTS, w_psi_stage=30.0), **common)
        err_off = abs(info_off["x_pred"][-1][2] - psi)
        err_on = abs(info_on["x_pred"][-1][2] - psi)
        assert err_on < err_off


# =====================================================================
# This commit must not change anything on the car.
# =====================================================================
class TestTheTermIsNeutralWhenOff:
    """A zero (or absent) w_psi_stage must change nothing.

    This started life as TestThisCommitIsInert, proving the stage-heading
    commit could not move the car because the weight shipped at 0.0. The
    weight-set commit that followed turned it on, so the "shipped default is
    zero" assertion is gone -- test_weight_set.py pins the shipped values
    now. What survives is the property that outlives any particular default:
    the term is exactly neutral when its weight is not set, which is what
    lets it be switched off in the field without side effects.
    """

    def test_a_zero_weight_leaves_the_solve_bit_for_bit_unchanged(self):
        corridor = _straight_corridor()
        common = dict(
            x0=np.array([0.5, 0.15, 0.20, 0.5]), last_u=np.array([0.0, 0.0]),
            pref_nom=corridor["Pend"], corridor=corridor, horizon=HORIZON,
            ts=TS, params=PARAMS, limits=LIMITS, obstacles=[], dmin=0.9,
            vdes=0.5, solver='rti')
        without = dict(BASE_WEIGHTS)
        del without["w_psi_stage"]          # key absent entirely
        with_zero = dict(BASE_WEIGHTS)      # key present, 0.0

        u_a, info_a = solve_mpc_step(weights=without, **common)
        u_b, info_b = solve_mpc_step(weights=with_zero, **common)
        assert np.array_equal(u_a, u_b)
        assert info_a["cost"] == info_b["cost"]
        assert np.array_equal(info_a["x_pred"], info_b["x_pred"])

    def test_an_absent_key_never_raises(self):
        """Every caller that predates this commit passes a weights dict with
        no w_psi_stage in it -- including planner_cost_corridor's own
        external callers replaying recorded frames."""
        corridor = _straight_corridor()
        weights = dict(BASE_WEIGHTS)
        del weights["w_psi_stage"]
        for solver in ('rti', 'slsqp'):
            solve_mpc_step(
                x0=np.array([0.5, 0.1, 0.0, 0.5]), last_u=np.array([0.0, 0.0]),
                pref_nom=corridor["Pend"], corridor=corridor, horizon=5,
                ts=TS, params=PARAMS, limits=LIMITS, weights=weights,
                obstacles=[], dmin=0.9, vdes=0.5, solver=solver)
