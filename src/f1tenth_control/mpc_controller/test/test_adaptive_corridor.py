"""
Adaptive corridor (Part B item B4, default off): adaptive_corridor.py.

The start half-width an obstacle in the way needs is side * c + r +
car_radius + margin + tolerance, the side being pass_side's (committed, else
the cheaper one); obstacles the keep-out rows ignore never widen anything; the
answer is clamped to [corr_wmin, the cap]. The closed loop is in
test_wide_corridor.py.
"""

from f1tenth_params.corridor_geometry import corridor_curves, funnel_half_widths
from f1tenth_params.param_defaults import get_value
from mpc_controller.adaptive_corridor import (
    adaptive_start_half_width, required_start_half_width)
from mpc_controller.blocked import BodyBox, keepout_relevance
from mpc_controller.pass_side import LEFT, PassSideMemory
import pytest

CAR_R = 0.20
MARGIN = 0.12
TOL = 0.08
REACH = 20 * 0.2 * 0.5   # N * dt * vdes, as control_loop passes it


def _body():
    return BodyBox(front=float(get_value('swept_clearance_body_front_x_m')),
                   rear=float(get_value('swept_clearance_body_rear_x_m')),
                   half_width=float(get_value('swept_clearance_body_half_width_m')))


def _corridor():
    w0, w1 = funnel_half_widths(0.65, 2.3 / 1.3)
    return corridor_curves(0.0, 0.0, 0.0, 0.0, 3.0, 120, w0=w0, w1=w1, length_ref=3.0)


def _need(obstacles, memory=None):
    return required_start_half_width(
        obstacles, _corridor(), (0.0, 0.0), CAR_R, MARGIN, TOL,
        keepout_relevance(CAR_R, MARGIN, _body(), REACH), memory=memory)


def test_nothing_in_the_way_needs_nothing_and_keeps_the_default():
    need, records = _need([])
    assert need == 0.0 and records == []
    assert adaptive_start_half_width(need, 0.65, 0.90) == pytest.approx(0.65)


def test_a_centred_person_needs_r_car_radius_margin_and_tolerance():
    need, records = _need([(2.0, 0.0, 0.25)])
    assert need == pytest.approx(0.25 + CAR_R + MARGIN + TOL)
    assert records[0]['side'] == 'left' and records[0]['side_source'] == 'cheaper'
    # exactly the default start: a centred person never widens it
    assert adaptive_start_half_width(need, 0.65, 0.90) == pytest.approx(0.65)


def test_a_wide_centred_object_widens_the_start_up_to_the_cap():
    need, _ = _need([(2.0, 0.0, 0.50)])
    assert need == pytest.approx(0.90)
    assert adaptive_start_half_width(need, 0.65, 0.90) == pytest.approx(0.90)
    need, _ = _need([(2.0, 0.0, 0.70)])
    assert adaptive_start_half_width(need, 0.65, 0.90) == pytest.approx(0.90)


def test_an_offset_obstacle_is_passed_on_its_far_side_for_less_room():
    need, records = _need([(2.0, 0.3, 0.25)])
    assert records[0]['side'] == 'right'
    assert need == pytest.approx(0.25 + CAR_R + MARGIN - 0.3 + TOL)


def test_a_committed_side_is_kept_even_when_the_other_is_cheaper():
    memory = PassSideMemory()
    memory.commit(2.0, 0.3, LEFT, now=0.0)
    need, records = _need([(2.0, 0.3, 0.25)], memory=memory)
    assert records[0]['side'] == 'left' and records[0]['side_source'] == 'committed'
    assert need == pytest.approx(0.3 + 0.25 + CAR_R + MARGIN + TOL)


def test_the_lookup_never_commits_a_side():
    memory = PassSideMemory()
    _need([(2.0, 0.0, 0.25)], memory=memory)
    assert memory.entries == []


def test_obstacles_the_keepouts_ignore_never_widen():
    body = _body()
    obstacles = [
        (-1.0, 0.0, 0.25),                                        # behind
        (REACH + 0.25 + CAR_R + MARGIN + body.front + 0.3, 0.0, 0.25),  # beyond reach
        (1.5, 3.0, 0.25),                                         # far outside
    ]
    need, records = _need(obstacles)
    assert need == 0.0 and records == []


def test_never_narrower_than_the_default_even_under_a_lower_cap():
    assert adaptive_start_half_width(0.30, 0.65, 0.90) == pytest.approx(0.65)
    assert adaptive_start_half_width(0.80, 0.65, 0.50) == pytest.approx(0.65)
