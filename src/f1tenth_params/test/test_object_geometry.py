"""f1tenth_params.object_geometry: the solver's front, footprints, gap_min.

Run standalone: python3 -m pytest test/test_object_geometry.py -v
"""

import pytest

from f1tenth_params.object_geometry import (
    SOLVER_FRONT_POINT_OFFSETS_M,
    GapLimits,
    class_margin_for,
    default_gap,
    footprint_radius,
    gap_limits,
    gap_min,
    nominal_footprint_radius,
    nose_reach,
)
from f1tenth_params.param_defaults import get_value


def test_nose_reach_is_half_the_wheelbase_plus_the_farthest_front_point():
    assert SOLVER_FRONT_POINT_OFFSETS_M == (0.10, 0.25, 0.40)
    assert nose_reach(0.305) == pytest.approx(0.5525)


def test_footprint_is_the_width_unless_depth_is_measured():
    assert footprint_radius(0.5, 0.3) == pytest.approx(0.25)
    assert footprint_radius(0.5, 0.8, depth_extent_is_measured=True) == pytest.approx(0.4)


@pytest.mark.parametrize('cls, radius', [('person', 0.25), ('chair', 0.225), ('tv', 0.15)])
def test_nominal_footprints(cls, radius):
    assert nominal_footprint_radius(cls) == pytest.approx(radius)


def test_gap_min_is_the_sum():
    assert gap_min(0.20, 0.12, 0.3, 0.10) == pytest.approx(0.72)


@pytest.mark.parametrize('gmin, default', [(0.42, 0.5), (0.5, 0.5), (0.500000001, 0.5),
                                           (0.51, 0.6), (0.72, 0.8), (0.1, 0.1)])
def test_default_gap_rounds_up_to_a_tenth(gmin, default):
    assert default_gap(gmin) == pytest.approx(default)


def test_class_margin_accepts_the_map_or_its_json_text():
    assert class_margin_for({'person': 0.3}, 'person') == pytest.approx(0.3)
    assert class_margin_for('{"person": 0.3}', 'chair') == 0.0
    assert class_margin_for('', 'person') == 0.0


def test_the_stack_params_limits_read_the_solver_and_projector_keys():
    limits = gap_limits('person')
    assert limits.car_radius == pytest.approx(float(get_value('car_radius')))
    assert limits.avoidance_margin == pytest.approx(float(get_value('obstacle_safety_margin_m')))
    assert limits.settle_buffer == pytest.approx(float(get_value('object_gap_settle_buffer_m')))
    assert limits.wheelbase == pytest.approx(float(get_value('mpc_wheelbase_m')))
    assert limits.class_margin == pytest.approx(
        class_margin_for(get_value('obstacle_class_margin_m'), 'person'))


def test_the_shipped_person_limits():
    """{} margin (no m*): 0.20 + 0.12 + 0 + 0.10 = 0.42, default gap 0.5."""
    limits = gap_limits('person')
    assert limits.gap_min == pytest.approx(0.42)
    assert limits.default_gap == pytest.approx(0.5)
    assert 'gap_min(person)' in limits.explain()


def test_gap_limits_can_be_built_by_hand():
    limits = GapLimits('chair', 0.2, 0.12, 0.0, 0.1, 0.305)
    assert limits.nose_reach == pytest.approx(0.5525)
