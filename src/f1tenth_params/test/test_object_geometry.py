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
    ObstacleFilter,
    obstacle_filter,
    passes_obstacle_filter,
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


GATE = ObstacleFilter(z_min=-0.1, z_max=2.0, r_min=0.03, r_max=1.5)


@pytest.mark.parametrize('z, r, kept', [
    (0.9, 0.25, True),
    (-0.1, 0.03, True), (2.0, 1.5, True),          # the bounds are inside
    (-0.11, 0.25, False), (2.01, 0.25, False),     # ground / overhead
    (0.9, 0.029, False), (0.9, 1.51, False),       # noise-sized / bad depth
    (float('nan'), 0.25, False), (0.9, float('nan'), False),
])
def test_the_obstacle_gate_is_the_height_band_and_the_radius_bounds(z, r, kept):
    assert passes_obstacle_filter(z, r, GATE) is kept


def test_the_obstacle_gate_reads_the_projectors_stack_params_keys():
    assert obstacle_filter() == ObstacleFilter(
        z_min=float(get_value('obstacle_z_min')), z_max=float(get_value('obstacle_z_max')),
        r_min=float(get_value('min_obstacle_radius')),
        r_max=float(get_value('max_obstacle_radius')))


def test_the_radius_bounds_kept_the_projectors_old_literals():
    """Moved from obstacle_projector_node's declare_parameter literals, values unchanged."""
    assert (float(get_value('min_obstacle_radius')),
            float(get_value('max_obstacle_radius'))) == (0.03, 1.5)
