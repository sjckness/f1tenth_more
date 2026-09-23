"""object_corridor_mode: which shape, when it switches, and what w_line sees.

The geometry itself is pinned in f1tenth_params' test_corridor_geometry.py,
against the 24 archived rebuilds. What is pinned HERE is the decision around
it: the hysteresis band, that the switch radius follows the lookahead clamp
instead of restating it, and that the target line is structurally absent
wherever it would be meaningless.

Same duck-typed stand-in shape as test_campaign_status.py.

Run standalone: python3 -m pytest test/test_object_corridor_mode.py -v
"""

import math

import pytest

from f1tenth_params.corridor_geometry import wrap_pi
from mpc_controller.MPC_corr import MPCController
from mpc_controller.mpc_solver import _STAGE_WEIGHT_KEYS, scale_stage_weights


class FakeNode:
    """Only what the switch decision reads."""

    def __init__(self, mode='arc_far', hi=1.24, lo=1.00,
                 n=20, ts=0.1, vdes=0.5, margin=1.25):
        self.object_corridor_mode = mode
        self.object_arc_switch_hi_frac = hi
        self.object_arc_switch_lo_frac = lo
        self.N, self.ts, self.vdes = n, ts, vdes
        self.corr_lookahead_reach_margin = margin
        self._object_arc_active = False

    clamp_length = MPCController._lookahead_clamp_length


def decide(node, r_goal):
    """The branch's own switch, in isolation: enter above hi, leave below lo,
    hold inside the band. Mirrors MPC_corr's object branch."""
    clamp = node.clamp_length()
    if node.object_corridor_mode == 'arc':
        use_arc = True
    elif node.object_corridor_mode == 'arc_far':
        if r_goal >= node.object_arc_switch_hi_frac * clamp:
            use_arc = True
        elif r_goal <= node.object_arc_switch_lo_frac * clamp:
            use_arc = False
        else:
            use_arc = node._object_arc_active
    else:
        use_arc = False
    node._object_arc_active = use_arc
    return use_arc


# ---------------------------------------------------------------------------
# the switch radius is derived, not restated
# ---------------------------------------------------------------------------

def test_the_clamp_length_is_the_lookahead_floor():
    """1.25 m at the shipping geometry, and it is a DERIVATION: N, ts, vdes and
    the margin all move it. Writing 1.25 into the switch would leave the band
    behind the moment any of them changed."""
    assert FakeNode().clamp_length() == pytest.approx(1.25)


@pytest.mark.parametrize('n, ts, vdes, margin, expected', [
    (20, 0.1, 0.5, 1.25, 1.25),
    (20, 0.1, 1.0, 1.25, 2.50),     # faster: the clamp, and the band, move out
    (40, 0.1, 0.5, 1.25, 2.50),     # longer horizon: same
    (20, 0.1, 0.5, 1.00, 1.00),     # smaller margin
])
def test_the_band_follows_the_clamp(n, ts, vdes, margin, expected):
    node = FakeNode(n=n, ts=ts, vdes=vdes, margin=margin)
    assert node.clamp_length() == pytest.approx(expected)
    # and the band is a multiple of it, so it tracks automatically
    assert decide(node, expected * 1.30) is True
    assert decide(node, expected * 0.50) is False


# ---------------------------------------------------------------------------
# hysteresis
# ---------------------------------------------------------------------------

def test_above_the_band_is_the_arc_and_below_it_is_not():
    node = FakeNode()
    assert decide(node, 3.0) is True
    assert decide(node, 0.5) is False


def test_inside_the_band_the_previous_choice_holds():
    """THE POINT OF THE BAND. The two shapes differ by the lead-in origin even
    at dpsi == 0 -- measured 0.31 m of reference step at +1.0 deg -- so a bare
    threshold chatters across it. Inside the band nothing changes."""
    node = FakeNode()
    decide(node, 3.0)                      # enter the arc
    for r in (1.54, 1.40, 1.26):           # the whole band, descending
        assert decide(node, r) is True     # still the arc
    assert decide(node, 1.24) is False     # only at/below lo does it leave

    for r in (1.26, 1.40, 1.54):           # and back up through the band
        assert decide(node, r) is False    # still today's geometry
    assert decide(node, 1.56) is True


def test_the_band_cannot_be_crossed_twice_without_leaving_it():
    """A corridor length dithering by a few cm mid-band must not flip the
    shape: that is exactly the chatter the band exists to stop."""
    node = FakeNode()
    decide(node, 3.0)
    flips = 0
    previous = node._object_arc_active
    for r in (1.30, 1.45, 1.28, 1.50, 1.35, 1.42):
        now = decide(node, r)
        flips += int(now != previous)
        previous = now
    assert flips == 0


# ---------------------------------------------------------------------------
# the modes
# ---------------------------------------------------------------------------

def test_off_is_never_the_arc_at_any_range():
    node = FakeNode(mode='off')
    for r in (0.1, 1.25, 5.0):
        assert decide(node, r) is False


def test_arc_is_always_the_arc_at_any_range():
    node = FakeNode(mode='arc')
    for r in (0.1, 1.25, 5.0):
        assert decide(node, r) is True


def test_off_is_the_default_so_nothing_changes_until_it_is_asked_for():
    from f1tenth_params.param_defaults import get_value
    assert get_value('object_corridor_mode') == 'off'


def test_the_band_bounds_are_ordered():
    from f1tenth_params.param_defaults import get_value
    assert (get_value('object_arc_switch_lo_frac')
            < get_value('object_arc_switch_hi_frac'))


# ---------------------------------------------------------------------------
# w_line: a per-stage weight, structurally gated
# ---------------------------------------------------------------------------

def test_w_line_is_stage_scaled_like_w_corr():
    """It is applied at every horizon stage, so without the normalisation its
    total would scale with the horizon while the terminal weights act once."""
    assert 'w_line' in _STAGE_WEIGHT_KEYS
    scaled = scale_stage_weights({'w_line': 3.5714, 'w_term': 9.0}, 20)
    assert scaled['w_line'] == pytest.approx(3.5714 * 7 / 20)
    assert scaled['w_term'] == pytest.approx(9.0)      # terminal, untouched


def test_w_line_is_on_the_same_derivation_as_the_weights_it_joins():
    """NOT a fitted number. The stage weights obey literal = rho / (7 sigma^2)
    -- see test_weight_set.py -- and w_line ships at rho 1.0 over sigma 0.20 m,
    the same pair w_corr uses, so its effective weight is the lateral authority
    w_corr already provided on the object branch. It is a transfer expressed in
    the set's own units, not an exception to them.
    """
    from f1tenth_params.param_defaults import get_value
    rho, sigma = 1.0, 0.20
    assert get_value('mpc_w_line') == pytest.approx(rho / (7 * sigma ** 2), rel=1e-4)
    w_line = scale_stage_weights({'w_line': get_value('mpc_w_line')}, 20)['w_line']
    assert w_line == pytest.approx(1.25, abs=1e-3)


def test_the_global_w_corr_is_untouched_by_the_arc():
    """The arc's tolerances are ARC-SCOPED. The straight and turn branches keep
    the weight set they are tuned at, which is the whole reason the split is
    applied per corridor rather than in the yaml."""
    from f1tenth_params.param_defaults import get_value
    assert get_value('mpc_w_corr') == pytest.approx(1.0 / (7 * 0.20 ** 2), rel=1e-4)


class _WeightNode:
    """Only what _weights_for reads."""

    def __init__(self, corr_sigma=0.25, line_sigma=0.35):
        from f1tenth_params.param_defaults import get_value
        self.arc_corr_sigma_m = corr_sigma
        self.arc_line_sigma_m = line_sigma
        self.weights = {'w_corr': get_value('mpc_w_corr'),
                        'w_line': get_value('mpc_w_line'),
                        'w_term': get_value('mpc_w_term')}

    weights_for = MPCController._weights_for


@pytest.mark.parametrize('shape', ['none', 'straight', None])
def test_a_non_arc_corridor_gets_the_untouched_weight_set(shape):
    node = _WeightNode()
    got = node.weights_for({} if shape is None else {'objectShape': shape})
    assert got is node.weights


def test_an_arc_corridor_derives_both_from_its_own_sigmas():
    """literal = rho / (7 sigma^2), the stage set's own identity, with a
    DIFFERENT sigma for each term because they own different curves."""
    node = _WeightNode(corr_sigma=0.25, line_sigma=0.35)
    got = node.weights_for({'objectShape': 'arc'})
    assert got['w_corr'] == pytest.approx(1.0 / (7 * 0.25 ** 2), rel=1e-9)
    assert got['w_line'] == pytest.approx(1.0 / (7 * 0.35 ** 2), rel=1e-9)
    assert got['w_term'] == node.weights['w_term']        # untouched
    assert node.weights['w_corr'] != got['w_corr']        # the original stands


def test_the_shipped_pair_keeps_the_total_near_what_the_car_is_tuned_at():
    """0.800 + 0.408 = 1.208 effective, against 1.25, and the untested term is
    the weaker one. PROVISIONAL -- both are parameters for exactly that reason.
    """
    from f1tenth_params.param_defaults import get_value
    node = _WeightNode(get_value('mpc_arc_corr_sigma_m'),
                       get_value('mpc_arc_line_sigma_m'))
    scaled = scale_stage_weights(node.weights_for({'objectShape': 'arc'}), 20)
    assert scaled['w_corr'] == pytest.approx(0.800, abs=0.005)
    assert scaled['w_line'] == pytest.approx(0.408, abs=0.005)
    assert scaled['w_line'] < scaled['w_corr']
    assert scaled['w_corr'] + scaled['w_line'] == pytest.approx(1.208, abs=0.01)


def test_the_line_tolerance_is_looser_than_the_arc_it_must_not_fight():
    """The measured numbers the two sigmas are chosen against: the arc departs
    from the target line by 0.194 m mean and 0.466 m max over the archived
    rebuilds, and the narrowest half-width there is 0.4327 m. The line sigma
    must sit between mean and max; the corridor sigma inside the half-width.
    """
    from f1tenth_params.param_defaults import get_value
    corr_sigma = get_value('mpc_arc_corr_sigma_m')
    line_sigma = get_value('mpc_arc_line_sigma_m')
    assert 0.194 < line_sigma < 0.466
    assert corr_sigma < 0.4327
    assert corr_sigma < line_sigma


@pytest.mark.parametrize('bad', [0.0, -0.1])
def test_a_non_positive_sigma_is_rejected(bad):
    """rho / (7 sigma^2) is not defined there, and a zero would silently turn
    the term into an infinite weight rather than off."""
    node = _WeightNode(corr_sigma=bad)
    with pytest.raises(ZeroDivisionError if bad == 0.0 else AssertionError):
        got = node.weights_for({'objectShape': 'arc'})
        assert got['w_corr'] > 0 and bad > 0


@pytest.mark.parametrize('corridor', [
    {},                                   # no key at all: every other branch
    {'targetLine': None},                 # explicitly cleared
])
def test_no_target_line_means_no_rows_at_all(corridor):
    """Structural absence, not a zero weight. A drive corridor, a lost target
    and a target behind the car all take this path."""
    assert corridor.get('targetLine') is None


def test_a_target_line_is_a_point_and_a_heading():
    line = {'p': [1.0, 2.0], 'psi': 0.3}
    n = (-math.sin(line['psi']), math.cos(line['psi']))
    # a point ON the line has zero cross-track, one beside it has the offset
    on = (1.0 + math.cos(0.3), 2.0 + math.sin(0.3))
    beside = (on[0] + 0.25 * n[0], on[1] + 0.25 * n[1])
    d_on = n[0] * (on[0] - line['p'][0]) + n[1] * (on[1] - line['p'][1])
    d_beside = n[0] * (beside[0] - line['p'][0]) + n[1] * (beside[1] - line['p'][1])
    assert d_on == pytest.approx(0.0, abs=1e-12)
    assert d_beside == pytest.approx(0.25)


# ---------------------------------------------------------------------------
# the straight branch's own feasibility limit
# ---------------------------------------------------------------------------

def test_the_straight_branch_limit_moves_with_the_ramp_span():
    """Unprotected without the clamp, and the limit is not a constant: it is
    span / (1.5 * R_min), so any u_start/u_end retuning moves it. Pinned so a
    retune has to come past this test."""
    from f1tenth_params.param_defaults import get_value
    from mpc_controller.wall_turn import min_turn_radius
    u_start = get_value('corr_turn_u_start')
    u_end = get_value('corr_turn_u_end')
    # corr_L_base is a bare literal in MPC_corr, not a yaml key -- see
    # stack_params' own note that corr_wmin/corr_wmax/corr_L_base are literals.
    span = (u_end - u_start) * 3.0
    r_min = min_turn_radius(get_value('mpc_wheelbase_m'),
                            get_value('mpc_steering_angle_max_rad'))
    limit = span / (1.5 * r_min)
    assert math.degrees(limit) == pytest.approx(49.7, abs=1.0)
    assert wrap_pi(limit) > 0
