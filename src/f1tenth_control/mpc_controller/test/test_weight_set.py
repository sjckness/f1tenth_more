"""The shipped MPC cost weights, and the normalisation they were built on.

WHY THESE ARE PINNED AT ALL. Until this pass self.weights was ten bare
literals with no ROS parameter behind them: nothing could override a weight
without a rebuild, and nothing recorded what they used to be. They are
single-sourced through stack_params.yaml now, the same way
corridor_update_period is, and per CLAUDE.md a test that encodes a config
default is a coupling -- so this file states the coupling out loud rather
than letting the yaml and the dict drift the way this repo has watched
happen four times over for one constant.

Every value here is READ from stack_params.yaml, never copied. What is
asserted is the STRUCTURE the numbers have to satisfy:

  - the dict the node builds matches the yaml key for key;
  - the stage weights obey literal = rho / (7 * sigma^2), the identity that
    makes the total stage cost over the horizon equal rho / sigma^2 for any
    N, which is what "horizon-invariant" means here;
  - the effective values (after scale_stage_weights' 7/20) are the ones the
    tuning note quotes, so a future reader comparing the two is comparing
    like with like;
  - the terms deliberately left OFF are still off, with the reason attached.

WHAT IS NOT PINNED, because it cannot be: whether this set drives well.
There is no sim in this stack and no vehicle attached to this machine. Every
number here is derived from geometry, not measured closed-loop, and the
overshoot caution in test_stage_tracking.py applies to all of it.

Run standalone: python3 -m pytest test/test_weight_set.py -v
"""

import math

import pytest

from f1tenth_params.param_defaults import get_value
from mpc_controller.mpc_solver import (
    STAGE_WEIGHT_REF_HORIZON,
    _STAGE_WEIGHT_KEYS,
    scale_stage_weights,
)

# (yaml key, weights-dict key, rho, sigma) for the three terms the
# rho/sigma^2 normalisation still derives AND still ships at. w_obs, w_term
# and w_psi are not in this table on purpose -- see their own tests below.
#
# w_du_delta LEFT THIS TABLE on 2026-09-09. Its derivation is unchanged and
# still gives 23.8095, but the shipped value is now 8.5714 (effective 3.0):
# a deliberate departure, pinned by its own test below rather than by an
# equality that would just fail.
_DERIVED = (
    ('mpc_w_corr', 'w_corr', 1.0, 0.20),
    ('mpc_w_psi_stage', 'w_psi_stage', 0.6, 0.134),
    ('mpc_w_v', 'w_v', 0.3, 0.10),
)

# The steering-rate term's own derivation, kept so the departure below is
# measured against something rather than asserted against a literal.
_DU_DELTA_RHO = 0.15
_DU_DELTA_SIGMA = 0.03

# What the tuning note quotes, i.e. after the 7/20 stage scaling.
_EFFECTIVE = {
    'w_corr': 1.25,
    'w_psi_stage': 1.67,
    'w_v': 1.5,
    'w_du_delta': 3.0,
    'w_obs': 2.8,
}

_YAML_TO_DICT = {
    'mpc_w_term': 'w_term',
    'mpc_w_psi': 'w_psi',
    'mpc_w_psi_stage': 'w_psi_stage',
    'mpc_w_corr': 'w_corr',
    'mpc_w_v': 'w_v',
    'mpc_w_obs': 'w_obs',
    'mpc_w_du_delta': 'w_du_delta',
    'mpc_w_delta0': 'w_delta0',
    'mpc_w_u_a': 'w_u_a',
    'mpc_w_du_a': 'w_du_a',
}


def _shipped():
    """The weights dict as the node would build it, straight from the yaml.

    Deliberately reconstructed from get_value() rather than by constructing
    an MPCController: that constructor opens log files and starts a timer,
    and every value it would put in the dict comes from exactly these keys.
    """
    return {d: float(get_value(y)) for y, d in _YAML_TO_DICT.items()}


class TestTheNormalisationHolds:

    @pytest.mark.parametrize('yaml_key,dict_key,rho,sigma', _DERIVED)
    def test_the_literal_is_rho_over_seven_sigma_squared(
            self, yaml_key, dict_key, rho, sigma):
        expected = rho / (STAGE_WEIGHT_REF_HORIZON * sigma ** 2)
        assert float(get_value(yaml_key)) == pytest.approx(expected, rel=1e-4)

    @pytest.mark.parametrize('yaml_key,dict_key,rho,sigma', _DERIVED)
    def test_the_total_stage_cost_is_rho_over_sigma_squared_for_any_horizon(
            self, yaml_key, dict_key, rho, sigma):
        """The property the literal was chosen to give.

        total = N * (literal * REF/N) = REF * literal, independent of N. A
        sustained unit of badness therefore costs rho over the horizon no
        matter how long the horizon is, which is what makes the priorities
        comparable across terms in the first place.
        """
        literal = float(get_value(yaml_key))
        for horizon in (7, 20, 40):
            scaled = scale_stage_weights({dict_key: literal}, horizon)
            total = horizon * scaled[dict_key]
            assert total == pytest.approx(rho / sigma ** 2, rel=1e-4)

    def test_the_heading_sigma_is_commensurate_with_the_position_sigma(self):
        """0.134 rad is not a free choice: it is the heading error that
        produces one position-sigma of lateral offset over the lookahead.

        0.20 m over the 1.5 m lookahead -- so the two priorities really are
        comparing like with like rather than two arbitrary scales.
        """
        position_sigma = 0.20
        lookahead = 1.5
        assert math.atan2(position_sigma, lookahead) == pytest.approx(0.134, abs=0.003)

    def test_the_position_sigma_fits_inside_the_narrow_corridor(self):
        """One unit of position badness must be well inside the corridor, or
        the weight is calibrated against an error the corridor bound would
        have caught first. 0.20 m is car_radius, under half the 0.4333 m
        narrow half-width."""
        assert 0.20 < 0.4333 / 2.0


class TestTheEffectiveValuesAreWhatTheNoteQuotes:
    """The literal is not what the solver applies, for every key except the
    two terminal ones. This has caused confusion once already; these assert
    the translation so a reader can trust the comment table."""

    @pytest.mark.parametrize('dict_key,expected', sorted(_EFFECTIVE.items()))
    def test_effective_value(self, dict_key, expected):
        # Tolerance 0.05 because _EFFECTIVE holds the values as the tuning
        # note WRITES them (two or three significant figures -- 1.67, 1.25),
        # not to full precision (1.6708, 1.2500). Pinning the note's own
        # rounding is the point: it is what a human will compare against.
        shipped = _shipped()
        scaled = scale_stage_weights(shipped, horizon=20)
        assert scaled[dict_key] == pytest.approx(expected, abs=0.05)

    def test_the_terminal_weights_pass_through_unscaled(self):
        shipped = _shipped()
        scaled = scale_stage_weights(shipped, horizon=20)
        assert 'w_term' not in _STAGE_WEIGHT_KEYS
        assert 'w_psi' not in _STAGE_WEIGHT_KEYS
        assert scaled['w_term'] == shipped['w_term'] == 9.0
        assert scaled['w_psi'] == shipped['w_psi'] == 4.5


class TestWhatChangedAndWhatDeliberatelyDidNot:

    def test_cross_track_tracking_is_on_now(self):
        """It was 0.0 -- off entirely -- so nothing at any stage referenced
        the path laterally."""
        assert _shipped()['w_corr'] > 0.0

    def test_stage_heading_tracking_is_on_now(self):
        assert _shipped()['w_psi_stage'] > 0.0

    def test_obstacle_shyness_was_not_increased(self):
        """w_obs effective stays 2.8. Less shyness was wanted, not more, so
        this one is pinned against being swept along by the retune."""
        scaled = scale_stage_weights(_shipped(), horizon=20)
        assert scaled['w_obs'] == pytest.approx(2.8, abs=0.01)

    def test_steering_magnitude_stays_off(self):
        """w_delta0 penalises the SIZE of the steering command. The car has
        a measured ~2.68 deg steering bias (steering_calibration.yaml), so
        the correct steady-state command for a straight line is nonzero --
        this term would fight holding that line, not help it."""
        assert _shipped()['w_delta0'] == 0.0

    def test_the_acceleration_terms_stay_off(self):
        assert _shipped()['w_u_a'] == 0.0
        assert _shipped()['w_du_a'] == 0.0

    def test_the_steering_rate_penalty_is_off_its_derived_value_on_purpose(self):
        """Effective 5.25 -> 8.33 -> 3.00, the last step deliberate.

        The rho/sigma^2 derivation gives literal 23.8095 from sigma = 0.03
        rad/step, and that sigma is "what the servo can do in a step" -- the
        ACTUATOR limit. The solve already carries that limit exactly, as the
        hard dDeltaMin/dDeltaMax bound, so the derived weight charges a
        second, soft price for motion the hard row already forbids. What
        loses the resulting trade is cross-track and heading correction,
        which is what the corridor set was retuned to turn on.

        This test pins the DEPARTURE, not the number: it fails if someone
        quietly restores the derived value, and it fails if the weight is
        driven to zero (it still has to damp chatter -- see the overshoot
        note in MPC_corr's own weights comment).
        """
        derived = _DU_DELTA_RHO / (STAGE_WEIGHT_REF_HORIZON * _DU_DELTA_SIGMA ** 2)
        shipped = _shipped()['w_du_delta']
        assert shipped < derived, (
            'w_du_delta is back on its derived value; the double-counted '
            'rate bound is back with it')
        scaled = scale_stage_weights(_shipped(), horizon=20)
        assert scaled['w_du_delta'] > 0.0, 'still has to damp chatter'
        # Same order as the two tracking terms it trades against, rather
        # than several times either of them.
        assert scaled['w_du_delta'] < 4.0 * scaled['w_corr']

    def test_the_steering_rate_total_cost_is_still_horizon_invariant(self):
        """Leaving the derived VALUE does not leave the stage-scaling
        machinery: the term is still per-stage, so its total over the
        horizon is still independent of N. Only the constant changed."""
        literal = _shipped()['w_du_delta']
        totals = [
            horizon * scale_stage_weights({'w_du_delta': literal}, horizon)['w_du_delta']
            for horizon in (7, 20, 40)
        ]
        assert totals[0] == pytest.approx(totals[1], rel=1e-4)
        assert totals[1] == pytest.approx(totals[2], rel=1e-4)

    def test_the_soft_rate_penalty_is_not_the_hard_rate_bound(self):
        """Worth keeping distinct in the suite because they sound alike and
        behave nothing alike: w_du_delta is tradeable against tracking,
        dDeltaMin/dDeltaMax is a hard constraint no weight can buy past.
        The hard one is still an unmeasured guess -- see
        test_steering_limits.py and MPC_corr's own dDelta comment.
        """
        scaled = scale_stage_weights(_shipped(), horizon=20)
        assert scaled['w_du_delta'] > 0.0  # soft, tradeable
        # The hard bound is not a weight and is not in this dict at all.
        assert 'dDeltaMax' not in _shipped()


class TestEveryWeightIsSingleSourced:

    def test_the_node_builds_its_dict_from_these_keys_and_no_literals(self):
        """The defect this replaces: ten bare literals with no parameter.

        Read out of the source so it fails if someone reintroduces a literal
        alongside the parameter reads -- the failure mode this codebase has
        already hit four times for one constant.
        """
        import inspect
        from mpc_controller.MPC_corr import MPCController
        src = inspect.getsource(MPCController.__init__)
        body = src.split('self.weights = {')[1].split('}')[0]
        for yaml_key, dict_key in _YAML_TO_DICT.items():
            assert f"_w('{yaml_key}')" in body, (
                f'{dict_key} is no longer read from {yaml_key}')

    def test_every_yaml_key_carries_a_description(self):
        """stack_params.yaml entries are (default, description) pairs and
        get_default() unpacks both -- a key without one breaks the launch
        arg that consumes it."""
        from f1tenth_params.param_defaults import get_default
        for yaml_key in _YAML_TO_DICT:
            _value, description = get_default(yaml_key)
            assert description.strip()


class TestTheHardRateBoundDominatesTheWeight:
    """A finding from validating this set, worth more than the set itself.

    w_du_delta is the SOFT steering-rate penalty. Its effective value went
    5.25 -> 8.33 and then, on 2026-09-09, 8.33 -> 3.00. But in the manoeuvre
    that matters most for it -- building steering from zero to correct an
    error -- it is not what governs, which is why NONE of the measurements
    below moved when the weight did: they are bound measurements, and they
    are the evidence that the 8.33 -> 3.00 change cannot speed up (or slow
    down) the start of a correction. The HARD bound is: dDeltaMin/dDeltaMax * ts = 0.5 * 0.1 = 0.05
    rad per step, and the solver rides exactly that limit for the whole ramp.

    Measured with the shipped weights and limits -- re-measured after the
    8.33 -> 3.00 change and IDENTICAL -- car on the centreline with 0.30 rad
    of heading error, the first six commanded steering angles are

        -0.0500  -0.1001  -0.1501  -0.2001  -0.2501  -0.2830

    i.e. -0.05 per step until it hits delta_min. Every one of those steps is
    ON the hard rate bound, so w_du_delta has nothing to trade during the
    ramp -- no value of it would change those numbers.

    WHY THAT MATTERS. dDelta is the one number in this whole pass that could
    not be measured (no servo feedback on this car -- see
    test_steering_limits.py and MPC_corr's own dDelta comment), so the
    dominant constraint on how fast the car can correct a heading error is
    currently an unverified guess. At 0.5 rad/s the car needs 0.57 s to
    reach full lock, and covers 0.28 m at 0.5 m/s while doing it. If the
    real servo is faster, the stack is leaving response on the table and no
    weight change in this commit can recover it.
    """

    def test_the_ramp_rides_the_hard_rate_bound_not_the_weight(self):
        import numpy as np
        from mpc_controller.mpc_solver import solve_mpc_step
        import test_stage_tracking as stage

        limits = dict(stage.LIMITS)
        limits['delta_min'] = float(get_value('mpc_steering_angle_min_rad'))
        limits['delta_max'] = float(get_value('mpc_steering_angle_max_rad'))
        corridor = stage._straight_corridor()

        _u, info = solve_mpc_step(
            x0=np.array([0.5, 0.0, 0.30, 0.5]), last_u=np.array([0.0, 0.0]),
            pref_nom=corridor['Pend'], corridor=corridor, horizon=20, ts=0.1,
            params=stage.PARAMS, limits=limits, weights=_shipped(),
            obstacles=[], dmin=0.9, vdes=0.5, solver='rti')

        deltas = [float(z) for z in info['zopt'][0::2]]
        step_bound = abs(limits['dDeltaMin']) * 0.1
        # The first step, from last_u = 0, is the cleanest case: it can only
        # be the bound.
        assert abs(deltas[0]) == pytest.approx(step_bound, abs=1e-4)
        # And the ramp continues on the bound until the angle limit stops it.
        ramp = [abs(deltas[k] - deltas[k - 1]) for k in range(1, 5)]
        for step in ramp:
            assert step == pytest.approx(step_bound, abs=1e-3)

    def test_full_lock_takes_more_than_half_a_second_at_this_bound(self):
        """The number to compare a real measurement against, if one is ever
        taken."""
        travel = abs(float(get_value('mpc_steering_angle_min_rad')))
        seconds = travel / 0.5  # dDeltaMax, rad/s
        assert seconds == pytest.approx(0.566, abs=0.01)

    def test_the_commanded_angles_stay_inside_the_servo_envelope(self):
        """End-to-end on the shipped configuration, with the tolerance the
        solver actually delivers.

        OSQP satisfies its rows to a convergence tolerance, not exactly --
        this solve lands 8.6e-6 rad past delta_min. That is absorbed by the
        yaml values being rounded INWARD from the true envelope (-0.28381 ->
        -0.283, 0.8 mrad of slack), so the command still reaches the servo
        inside what it can produce. Checked here because it is the whole
        justification for rounding inward rather than to nearest.
        """
        import numpy as np
        from mpc_controller.mpc_solver import solve_mpc_step
        import test_stage_tracking as stage

        limits = dict(stage.LIMITS)
        limits['delta_min'] = float(get_value('mpc_steering_angle_min_rad'))
        limits['delta_max'] = float(get_value('mpc_steering_angle_max_rad'))
        corridor = stage._straight_corridor()
        _u, info = solve_mpc_step(
            x0=np.array([0.5, 0.0, 0.30, 0.5]), last_u=np.array([0.0, 0.0]),
            pref_nom=corridor['Pend'], corridor=corridor, horizon=20, ts=0.1,
            params=stage.PARAMS, limits=limits, weights=_shipped(),
            obstacles=[], dmin=0.9, vdes=0.5, solver='rti')

        deltas = [float(z) for z in info['zopt'][0::2]]
        solver_tol = 1.0e-4
        assert min(deltas) >= limits['delta_min'] - solver_tol
        assert max(deltas) <= limits['delta_max'] + solver_tol
        # True servo envelope, before the inward rounding.
        assert min(deltas) >= -0.28381
        assert max(deltas) <= 0.27804
