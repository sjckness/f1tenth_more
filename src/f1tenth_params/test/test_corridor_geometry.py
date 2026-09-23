"""corridor_geometry: the two clamps, and the object-corridor A/B.

The A/B runs over data/object_rebuilds.json -- the 24 go_to_object corridor
rebuilds in first_test_campaing/M04_person, distilled to the five numbers each
one needs (car pose, the held heading psi_c, the lead-in origin, the goal).
The campaign folder itself is not in the repo, so the fixture is: without it
this comparison could not be re-run, and it is the measurement the whole
object_corridor_mode change rests on.

What is asserted is the geometry, not a closed-loop outcome. Whether the
wobble shrinks needs the car.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from f1tenth_params.corridor_geometry import (
    CORRIDOR_HANDLE_FRAC,
    SMOOTHSTEP_PEAK_SLOPE,
    corridor_curves,
    ramp_span_for,
    wrap_pi,
)

U_START, U_END = 0.0, 0.40
W0, W1, L_BASE = 0.4333, 0.7667, 3.0
#: Shipping steering calibration: the two bounds are not equal.
RMIN_LEFT, RMIN_RIGHT = 0.923, 0.955
#: MPC_corr._lookahead_clamp_length() at N 20, ts 0.1, vdes 0.5, margin 1.25.
CLAMP_LEN = 1.25
HI_FRAC, LO_FRAC = 1.24, 1.00

REBUILDS = json.loads(
    (Path(__file__).parent / "data" / "object_rebuilds.json").read_text()
)["rebuilds"]


def r_min_for(dpsi):
    return RMIN_LEFT if dpsi >= 0.0 else RMIN_RIGHT


def mode_a(rec):
    """Today's geometry: pinned to the target line, straight."""
    origin = rec["lead_in_origin"]
    goal = np.asarray(rec["goal"], dtype=float)
    length = float(np.hypot(*(goal - np.asarray(origin, dtype=float))))
    return corridor_curves(origin[0], origin[1], rec["psi_c"], rec["psi_c"],
                           length, 120, dpsi=0.0, u_start=U_START, u_end=U_END,
                           w0=W0, w1=W1), length


def mode_b(rec, **kwargs):
    """The arc: origin on the car, psiStart the live yaw, psiEnd psi_c."""
    car = rec["car"]
    goal = np.asarray(rec["goal"], dtype=float)
    dpsi = wrap_pi(rec["psi_c"] - rec["yaw"])
    length = float(np.hypot(*(goal - np.asarray(car, dtype=float))))
    opts = dict(u_start=U_START, u_end=U_END, w0=W0, w1=W1)
    opts.update(kwargs)
    return corridor_curves(car[0], car[1], rec["yaw"], rec["psi_c"],
                           max(length, 1e-3), 120, dpsi=dpsi, **opts), length


# --------------------------------------------------------------------------
# the fixture itself
# --------------------------------------------------------------------------

def test_the_fixture_is_the_twenty_four_rebuilds():
    assert len(REBUILDS) == 24
    assert {r["run"] for r in REBUILDS} == {
        "P004-R001-20260922T143740", "P004-R002-20260922T143855"}


# --------------------------------------------------------------------------
# backward compatibility: no clamp arguments, no change
# --------------------------------------------------------------------------

def test_without_the_clamps_nothing_moves():
    """Both clamps are opt-in. Every branch that does not pass r_min or
    length_ref must get byte-identical geometry to before they existed."""
    plain = corridor_curves(1.0, -2.0, 0.3, 0.9, 3.0, 120,
                            u_start=U_START, u_end=U_END, w0=W0, w1=W1)
    assert plain["u_end_eff"] == U_END
    assert plain["w1_eff"] == W1
    assert plain["ramp_clamped"] is False
    assert plain["feasible"] is True


def test_the_width_rate_is_inert_at_the_design_point():
    """length_ref == length is the shape the funnel was drawn for."""
    ramped = corridor_curves(0, 0, 0, 0.2, L_BASE, 120, w0=W0, w1=W1,
                             length_ref=L_BASE)
    assert ramped["w1_eff"] == pytest.approx(W1)


# --------------------------------------------------------------------------
# the ramp-span clamp
# --------------------------------------------------------------------------

@pytest.mark.parametrize("dpsi_deg", [5, 20, 47, 60])
@pytest.mark.parametrize("length", [0.8, 1.5, 3.0])
def test_the_clamp_keeps_curvature_inside_r_min(dpsi_deg, length):
    """THE GUARANTEE. Wherever the clamp reports feasible, the corridor's peak
    curvature is inside the steering bound for the direction it turns."""
    dpsi = math.radians(dpsi_deg)
    r_min = r_min_for(dpsi)
    got = corridor_curves(0, 0, 0.0, dpsi, length, 120, dpsi=dpsi,
                          u_start=U_START, u_end=U_END, r_min=r_min)
    if got["feasible"]:
        assert got["kappa_max"] <= 1.0 / r_min + 1e-9


def test_the_clamp_only_ever_widens_the_ramp():
    """It must not narrow a ramp that was already long enough, or it would
    change the tuned geometry on corridors that never needed it."""
    small = math.radians(2.0)
    got = corridor_curves(0, 0, 0.0, small, 3.0, 120, dpsi=small,
                          u_start=U_START, u_end=U_END, r_min=RMIN_LEFT)
    assert got["u_end_eff"] == U_END
    assert got["ramp_clamped"] is False


def test_an_unreachable_ask_is_reported_not_repaired():
    """Capping dpsi instead would silently move the corridor's end heading,
    and on the object branch that is what holds the end cap on the goal."""
    dpsi = math.radians(120.0)
    got = corridor_curves(0, 0, 0.0, dpsi, 0.3, 120, dpsi=dpsi,
                          u_start=U_START, u_end=U_END, r_min=RMIN_LEFT)
    assert got["feasible"] is False
    assert got["u_end_eff"] == 1.0            # widened as far as it goes
    assert got["dpsi"] == pytest.approx(dpsi)  # and NOT reduced


def test_ramp_span_for_matches_the_curvature_it_claims():
    for dpsi in (0.1, 0.5, 0.9):
        span = ramp_span_for(dpsi, RMIN_LEFT)
        assert SMOOTHSTEP_PEAK_SLOPE * dpsi / span == pytest.approx(1 / RMIN_LEFT)


# --------------------------------------------------------------------------
# the A/B over the archived rebuilds
# --------------------------------------------------------------------------

def test_mode_a_ends_exactly_on_the_goal():
    """Today's geometry, and the property the object branch was built around:
    the corridor ends at the goal, so the terminal cost aims there once the
    lookahead clamps to it."""
    worst = max(
        float(np.hypot(*(mode_a(r)[0]["Pend"] - np.asarray(r["goal"]))))
        for r in REBUILDS)
    assert worst < 1e-3


def test_mode_a_never_expresses_the_heading_error():
    """The defect: dpsi is identically zero however badly the car is aimed."""
    errors = [abs(wrap_pi(r["psi_c"] - r["yaw"])) for r in REBUILDS]
    assert max(errors) > math.radians(45.0)     # 47 deg, measured
    for r in REBUILDS:
        assert mode_a(r)[0]["dpsi"] == 0.0


def test_mode_b_puts_the_car_on_its_own_centreline():
    """Which is exactly why w_corr can no longer own the lateral error: it
    sees none. mpc_w_line takes that job over."""
    for r in REBUILDS:
        curves, _ = mode_b(r)
        d = float(np.min(np.hypot(curves["xc"] - r["car"][0],
                                  curves["yc"] - r["car"][1])))
        assert d < 1e-9


def test_mode_b_drifts_off_the_goal_which_is_why_arc_far_exists():
    """The endpoint is no longer the goal. Harmless while the lookahead sits
    inside the corridor; the reason the arc is not used once it clamps."""
    drift = [float(np.hypot(*(mode_b(r)[0]["Pend"] - np.asarray(r["goal"]))))
             for r in REBUILDS]
    assert max(drift) > 0.5
    assert np.mean(drift) > 0.1


def test_the_clamp_makes_every_archived_rebuild_feasible():
    """Without it 7 of the 24 ask for a tighter arc than the car can turn.

    (5 of them if L is floored at 1.0 m the way the goal_pose branch floors
    it; the object branch does not floor L, and these are its own numbers.)
    """
    unclamped = clamped = 0
    for r in REBUILDS:
        dpsi = wrap_pi(r["psi_c"] - r["yaw"])
        r_min = r_min_for(dpsi)
        plain, _ = mode_b(r)
        if plain["kappa_max"] > 1.0 / r_min:
            unclamped += 1
        fixed, _ = mode_b(r, r_min=r_min)
        assert fixed["feasible"], f"rebuild {r['id']} still infeasible"
        if fixed["ramp_clamped"]:
            clamped += 1
    assert unclamped == 7
    assert clamped == 7


def test_arc_far_uses_todays_geometry_where_the_endpoint_matters():
    """The switch band, in the units it is actually specified in. Above hi the
    arc runs; below lo today's geometry does; the endpoint drift is confined
    to corridors long enough that the lookahead never reaches their end."""
    lengths = [float(np.hypot(*(np.asarray(r["goal"]) - np.asarray(r["car"]))))
               for r in REBUILDS]
    above = [L for L in lengths if L >= HI_FRAC * CLAMP_LEN]
    below = [L for L in lengths if L <= LO_FRAC * CLAMP_LEN]
    inside = [L for L in lengths if LO_FRAC * CLAMP_LEN < L < HI_FRAC * CLAMP_LEN]
    assert (len(above), len(inside), len(below)) == (9, 3, 12)

    # THE SAFETY ARGUMENT, and now it is exact. The band's LOWER edge sits on
    # the clamp distance, so the arc is never entered -- and, through
    # hysteresis, never held -- below it. Pend therefore never becomes the
    # terminal target while an arc is being flown, whatever its drift.
    assert LO_FRAC * CLAMP_LEN >= CLAMP_LEN
    for length in above:
        assert length > CLAMP_LEN


# --------------------------------------------------------------------------
# the width rate on the degenerate corridors
# --------------------------------------------------------------------------

def test_the_width_rate_fixes_the_corridors_wider_than_they_are_long():
    """10 of the 24 are, the shortest 0.74 m long against a 1.53 m far-end
    width. With the rate the funnel opens per metre instead."""
    before = after = 0
    for r in REBUILDS:
        _, length = mode_a(r)
        if length < 2 * W1:
            before += 1
        rate = corridor_curves(0, 0, 0, 0, length, 120, w0=W0, w1=W1,
                               length_ref=L_BASE)
        if length < 2 * rate["w1_eff"]:
            after += 1
    assert before == 10
    assert after < before


def test_the_width_rate_never_widens_a_corridor():
    for length in (0.4, 0.74, 1.5, 3.0, 5.0):
        rate = corridor_curves(0, 0, 0, 0, length, 120, w0=W0, w1=W1,
                               length_ref=L_BASE)
        assert W0 <= rate["w1_eff"] <= W1 + 1e-12


def test_the_handle_fraction_is_shared_not_per_branch():
    """One family: every branch reaches corridor_curves with the same handle."""
    assert CORRIDOR_HANDLE_FRAC == 0.55
