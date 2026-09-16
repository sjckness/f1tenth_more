"""Tests for the pure-pursuit geometry."""

from math import atan, cos, degrees, hypot, isclose, pi, radians, sin

import pytest

from go_to_object.pursuit_geometry import (
    CurvatureLimiter,
    curvature_from_steering,
    PursuitParams,
    PursuitSolution,
    solve_pursuit,
    wrap_pi,
)

# Tiny limits so that nothing clamps and the raw formula is under test.
FREE = PursuitParams(r_min=1e-3, d_lookahead_min=1e-6, d_stop=0.0,
                     straight_eps=1e-9)


def _integrate(x, y, psi, curvature, arc_length, steps=20000):
    """Roll the unicycle forward along a constant curvature."""
    ds = arc_length / steps
    for _ in range(steps):
        x += ds * cos(psi)
        y += ds * sin(psi)
        psi += curvature * ds
    return x, y, wrap_pi(psi)


def _arc_length(solution: PursuitSolution) -> float:
    if solution.curvature == 0.0:
        return solution.distance
    return 2.0 * solution.alpha / solution.curvature


# -- 1. the property test that actually matters ---------------------------

@pytest.mark.parametrize('psi_deg', [0.0, 37.0, 90.0, 179.0, -120.0])
@pytest.mark.parametrize('bearing_deg', [-85.0, -45.0, -7.0, 0.0, 7.0, 45.0, 85.0])
@pytest.mark.parametrize('distance', [0.8, 4.0, 21.0])
def test_commanded_arc_passes_through_the_object(psi_deg, bearing_deg, distance):
    """Integrating the command must actually arrive at the object.

    The formula is easy to get subtly wrong in a way that still plots
    plausibly, so this closes the loop numerically rather than restating
    ``2*sin(a)/d`` back at itself.
    """
    psi = radians(psi_deg)
    vehicle = (-3.0, 1.5)
    heading_to_object = psi + radians(bearing_deg)
    obj = (vehicle[0] + distance * cos(heading_to_object),
           vehicle[1] + distance * sin(heading_to_object))

    sol = solve_pursuit(vehicle, psi, obj, FREE)
    x, y, psi_final = _integrate(
        vehicle[0], vehicle[1], psi, sol.curvature, _arc_length(sol))

    assert hypot(x - obj[0], y - obj[1]) < 2e-3
    assert abs(wrap_pi(psi_final - sol.psi_end)) < 2e-3


# -- 2. dead ahead ---------------------------------------------------------

@pytest.mark.parametrize('psi_deg', [0.0, 45.0, 123.0, -90.0, 180.0])
def test_object_dead_ahead_is_exactly_straight(psi_deg):
    psi = radians(psi_deg)
    obj = (7.0 * cos(psi), 7.0 * sin(psi))

    sol = solve_pursuit((0.0, 0.0), psi, obj)

    assert sol.curvature == 0.0
    assert sol.straight is True
    assert sol.radius is None
    assert abs(sol.alpha) < 1e-12


# -- 3. the formula and its 1/d falloff ------------------------------------

@pytest.mark.parametrize('distance', [2.0, 4.0, 8.0, 16.0])
def test_curvature_matches_formula_and_falls_as_inverse_distance(distance):
    alpha = radians(20.0)
    obj = (distance * cos(alpha), distance * sin(alpha))

    sol = solve_pursuit((0.0, 0.0), 0.0, obj, FREE)

    assert isclose(sol.curvature, 2.0 * sin(alpha) / distance, rel_tol=1e-12)


def test_curvature_halves_when_distance_doubles():
    alpha = radians(20.0)
    near = solve_pursuit((0.0, 0.0), 0.0,
                         (5.0 * cos(alpha), 5.0 * sin(alpha)), FREE)
    far = solve_pursuit((0.0, 0.0), 0.0,
                        (10.0 * cos(alpha), 10.0 * sin(alpha)), FREE)

    assert isclose(near.curvature, 2.0 * far.curvature, rel_tol=1e-12)


# -- 4. sign and symmetry --------------------------------------------------

@pytest.mark.parametrize('bearing_deg', [5.0, 30.0, 60.0, 89.0])
def test_curvature_sign_follows_the_side_and_is_symmetric(bearing_deg):
    a = radians(bearing_deg)
    left = solve_pursuit((0.0, 0.0), 0.0, (9.0 * cos(a), 9.0 * sin(a)), FREE)
    right = solve_pursuit((0.0, 0.0), 0.0, (9.0 * cos(a), -9.0 * sin(a)), FREE)

    assert left.curvature > 0.0, 'object to the left must turn left (CCW)'
    assert right.curvature < 0.0
    assert isclose(left.curvature, -right.curvature, rel_tol=1e-12)


# -- 5. continuity across the centreline -----------------------------------

def test_curvature_is_continuous_across_the_centreline():
    distance = 12.0
    steps = 4001
    lateral = [-0.5 + i * (1.0 / (steps - 1)) for i in range(steps)]
    curvatures = [
        solve_pursuit((0.0, 0.0), 0.0, (distance, y)).curvature for y in lateral
    ]

    jumps = [abs(b - a) for a, b in zip(curvatures, curvatures[1:])]
    assert max(jumps) < 1e-3
    assert curvatures[0] < 0.0 < curvatures[-1]


# -- 6. clamping and the lookahead floor -----------------------------------

def test_curvature_clamps_to_minimum_turning_radius():
    par = PursuitParams(r_min=2.0)
    sol = solve_pursuit((0.0, 0.0), 0.0, (0.35, 1.4), par)

    assert isclose(abs(sol.curvature), 1.0 / par.r_min, rel_tol=1e-12)
    assert isclose(abs(sol.radius), par.r_min, rel_tol=1e-12)


def test_lookahead_floor_bounds_the_close_range_command():
    # r_min small => a large curvature limit, so only the floor can bind.
    par = PursuitParams(r_min=0.01, d_lookahead_min=1.0)
    alpha = radians(10.0)
    close = 0.05
    obj = (close * cos(alpha), close * sin(alpha))

    sol = solve_pursuit((0.0, 0.0), 0.0, obj, par)

    unfloored = 2.0 * sin(alpha) / close
    assert isclose(sol.curvature, 2.0 * sin(alpha) / par.d_lookahead_min,
                   rel_tol=1e-12)
    assert abs(sol.curvature) < unfloored / 10.0
    assert sol.distance == pytest.approx(close), 'distance stays the true one'


def test_psi_end_uses_raw_alpha_even_while_curvature_saturates():
    par = PursuitParams(r_min=2.0)
    sol = solve_pursuit((0.0, 0.0), 0.0, (0.35, 1.4), par)

    assert isclose(abs(sol.curvature), 1.0 / par.r_min, rel_tol=1e-12)
    assert isclose(sol.psi_end, wrap_pi(2.0 * sol.alpha), abs_tol=1e-12)


# -- 7. end heading --------------------------------------------------------

@pytest.mark.parametrize('psi_deg', [0.0, 60.0, 170.0, -150.0])
@pytest.mark.parametrize('bearing_deg', [-70.0, -20.0, 20.0, 70.0])
def test_psi_end_is_psi_plus_two_alpha(psi_deg, bearing_deg):
    psi = radians(psi_deg)
    heading = psi + radians(bearing_deg)
    obj = (6.0 * cos(heading), 6.0 * sin(heading))

    sol = solve_pursuit((0.0, 0.0), psi, obj, FREE)

    assert isclose(sol.psi_end, wrap_pi(psi + 2.0 * sol.alpha), abs_tol=1e-12)
    assert -pi < sol.psi_end <= pi


# -- 8. reachability -------------------------------------------------------

def test_target_inside_the_minimum_turning_circle_is_unreachable():
    par = PursuitParams(r_min=1.5)

    inside = solve_pursuit((0.0, 0.0), 0.0, (0.0, 1.0), par)
    outside = solve_pursuit((0.0, 0.0), 0.0, (0.0, 5.0), par)
    ahead = solve_pursuit((0.0, 0.0), 0.0, (0.4, 0.0), par)

    assert inside.reachable is False
    assert outside.reachable is True
    assert ahead.reachable is True, 'anything dead ahead is reachable'


# -- 9. object behind ------------------------------------------------------

@pytest.mark.parametrize('lateral,expected_sign', [(1.0, +1.0), (-1.0, -1.0)])
def test_object_behind_commands_maximum_curvature_toward_it(lateral, expected_sign):
    par = PursuitParams(r_min=1.5)
    sol = solve_pursuit((0.0, 0.0), 0.0, (-5.0, lateral), par)

    assert sol.behind is True
    assert isclose(sol.curvature, expected_sign / par.r_min, rel_tol=1e-12)


def test_object_directly_behind_does_not_coast_straight_past_it():
    """The bare formula decays to ~0 at alpha -> pi; the override must not."""
    par = PursuitParams(r_min=1.5)
    sol = solve_pursuit((0.0, 0.0), 0.0, (-8.0, 1e-6), par)

    assert sol.behind is True
    assert abs(sol.curvature) == pytest.approx(1.0 / par.r_min)


# -- 10. degenerate pose ---------------------------------------------------

def test_coincident_pose_does_not_divide_by_zero():
    sol = solve_pursuit((2.0, -3.0), 1.1, (2.0, -3.0))

    assert sol.curvature == 0.0
    assert sol.straight is True
    assert sol.arrived is True
    assert sol.distance == 0.0


# -- supporting invariants -------------------------------------------------

@pytest.mark.parametrize('angle_deg,expected_deg', [
    (0.0, 0.0), (180.0, 180.0), (-180.0, 180.0), (190.0, -170.0),
    (-190.0, 170.0), (540.0, 180.0), (359.0, -1.0),
])
def test_wrap_pi_maps_onto_the_half_open_interval(angle_deg, expected_deg):
    assert degrees(wrap_pi(radians(angle_deg))) == pytest.approx(expected_deg)
    assert -pi < wrap_pi(radians(angle_deg)) <= pi


def test_straight_deadband_emits_exact_zero_not_a_flickering_residue():
    par = PursuitParams(straight_eps=1e-3)
    sol = solve_pursuit((0.0, 0.0), 0.0, (10.0, 10.0 * 5e-4), par)

    assert sol.curvature == 0.0
    assert sol.straight is True


def test_arrived_is_reported_at_the_stop_distance():
    par = PursuitParams(d_stop=0.5)
    assert solve_pursuit((0.0, 0.0), 0.0, (0.4, 0.0), par).arrived is True
    assert solve_pursuit((0.0, 0.0), 0.0, (0.6, 0.0), par).arrived is False


@pytest.mark.parametrize('kwargs', [
    {'r_min': 0.0}, {'r_min': -1.0}, {'d_lookahead_min': 0.0},
    {'d_stop': -0.1}, {'straight_eps': -1e-9},
])
def test_invalid_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):
        PursuitParams(**kwargs)


# -- CurvatureLimiter ------------------------------------------------------

def test_limiter_starts_synchronised_to_centred_wheels_and_limits_immediately():
    """Cold start assumes wheels centred, and the very first call is limited.

    The old behaviour seeded on the first call and passed it through, which
    bypassed the limiter at exactly the moment a step occurs.
    """
    limiter = CurvatureLimiter(max_kappa_rate=0.5)
    assert limiter.previous == 0.0

    assert limiter.apply(0.4, 0.01) == pytest.approx(0.005)
    assert limiter.saturated is True


def test_limiter_honours_an_explicit_cold_start_seed():
    limiter = CurvatureLimiter(max_kappa_rate=0.5, initial_kappa=-0.2)
    assert limiter.previous == -0.2

    limiter.seed(0.3)
    assert limiter.previous == 0.3
    assert limiter.apply(0.3, 0.01) == pytest.approx(0.3), 'already there'


@pytest.mark.parametrize('rate', [0.05, 0.5, 2.0, 21.0])
@pytest.mark.parametrize('dt', [0.005, 0.01, 0.05])
@pytest.mark.parametrize('target', [5.0, -5.0])
def test_limiter_bounds_every_step_by_rate_times_dt(rate, dt, target):
    limiter = CurvatureLimiter(max_kappa_rate=rate)
    limiter.apply(0.0, dt)

    previous = 0.0
    for _ in range(20):
        current = limiter.apply(target, dt)
        assert abs(current - previous) <= rate * dt + 1e-12
        previous = current


def test_limiter_passes_steps_smaller_than_the_limit_unchanged():
    limiter = CurvatureLimiter(max_kappa_rate=0.5)
    limiter.seed(0.1)

    assert limiter.apply(0.1005, 0.01) == pytest.approx(0.1005)
    assert limiter.saturated is False


def test_limiter_reaches_the_target_in_the_expected_number_of_cycles():
    limiter = CurvatureLimiter(max_kappa_rate=0.5)
    limiter.apply(0.0, 0.01)

    cycles = 0
    while limiter.apply(0.05, 0.01) < 0.05 - 1e-12:
        cycles += 1
        assert cycles < 100
    assert cycles == pytest.approx(0.05 / (0.5 * 0.01), abs=1)


def test_resync_without_a_measurement_keeps_the_last_commanded_curvature():
    """It resynchronises to physical state; it never forgets it."""
    limiter = CurvatureLimiter(max_kappa_rate=0.5)
    limiter.seed(0.4)

    assert limiter.resync() == pytest.approx(0.4)
    assert limiter.previous == pytest.approx(0.4)
    assert limiter.apply(2.0, 0.01) == pytest.approx(0.405), 'still limited'


def test_resync_with_a_measurement_overrides_the_last_commanded_curvature():
    """Where the wheels *are* beats where they were last told to go."""
    limiter = CurvatureLimiter(max_kappa_rate=0.5)
    limiter.seed(0.40)

    limiter.resync(measured_kappa=-0.25)

    assert limiter.previous == pytest.approx(-0.25)
    assert limiter.apply(2.0, 0.01) == pytest.approx(-0.245)


def test_measured_steering_maps_to_curvature_through_the_wheelbase():
    wheelbase = 0.33
    for kappa in (-0.6, 0.0, 0.25, 0.66):
        delta = atan(wheelbase * kappa)
        assert curvature_from_steering(delta, wheelbase) == pytest.approx(kappa)

    with pytest.raises(ValueError):
        curvature_from_steering(0.1, 0.0)


def test_limiter_counts_its_own_saturations():
    limiter = CurvatureLimiter(max_kappa_rate=0.5)
    for _ in range(10):
        limiter.apply(1.0, 0.01)
    saturated_ramp = limiter.saturations

    for _ in range(5):
        limiter.apply(limiter.previous, 0.01)

    assert saturated_ramp == 10, 'every cycle of a long ramp is saturated'
    assert limiter.saturations == 10, 'and holding station is not'
    assert limiter.applications == 15


@pytest.mark.parametrize('dt', [0.0, -0.01])
def test_limiter_holds_when_no_time_has_elapsed(dt):
    """Zero elapsed time permits zero slew -- the wheels cannot have moved."""
    limiter = CurvatureLimiter(max_kappa_rate=0.5)
    limiter.seed(0.2)

    assert limiter.apply(3.0, dt) == pytest.approx(0.2), 'held, not passed through'
    assert limiter.previous == pytest.approx(0.2)
    assert limiter.saturated is True


@pytest.mark.parametrize('rate', [0.0, -1.0])
def test_limiter_rejects_a_non_positive_rate(rate):
    with pytest.raises(ValueError):
        CurvatureLimiter(max_kappa_rate=rate)


# -- why the limit is on curvature and not on the position estimate --------

@pytest.mark.parametrize('distance,expected', [
    (3.00, 0.00333), (2.00, 0.00750), (1.75, 0.00980),
    (1.50, 0.01333), (1.25, 0.01920), (1.00, 0.03000),
])
def test_position_rate_limiting_cannot_bound_a_curvature_step(distance, expected):
    """The reason the position limiter was deleted, as a table.

    A limiter on the *position* estimate shifts it laterally by ``rate * dt``
    per cycle; the resulting curvature step is ``2 * rate * dt / d**2``. That
    grows without bound as the vehicle closes, so no constant position rate
    satisfies a distance-independent curvature bound -- picking a smaller
    rate only moves the distance at which it is first violated. These are
    the figures for the old 1.5 m/s default at a 10 ms cycle; every row from
    1.75 m inward exceeds the 0.005 bound it was supposed to hold.
    """
    rate, dt, cruise_bound = 1.5, 0.01, 0.005
    lateral = rate * dt

    k0 = solve_pursuit((0.0, 0.0), 0.0, (distance, 0.0)).curvature
    k1 = solve_pursuit((0.0, 0.0), 0.0, (distance, lateral)).curvature
    step = abs(k1 - k0)

    assert step == pytest.approx(expected, rel=2e-2)
    assert step == pytest.approx(2.0 * lateral / distance ** 2, rel=2e-2)
    if distance <= 1.75:
        assert step > cruise_bound, 'a constant position rate cannot hold it'
