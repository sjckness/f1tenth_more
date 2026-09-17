"""Object geometry shared by the solver, the projector, the tracker and the mission.

Pure Python, no ROS. Lives in f1tenth_params, the dependency-free leaf package
every consumer already depends on (mpc_controller, f1tenth_perception,
f1tenth_costmap, f1tenth_behavior, llm), because none of those may depend on
each other for it and each needs the SAME numbers:

* SOLVER_FRONT_POINT_OFFSETS_M -- where mpc_solver measures the car's front
  from: points 0.10 / 0.25 / 0.40 m ahead of p_nose = base_link + L/2. Its
  w_obs term penalises softplus((car_radius + avoidance_margin) - d_front),
  d_front = min over those points of |p - obstacle| - r.
* nose_reach(L) = L/2 + max offset: how far ahead of base_link that front is.
* footprint_radius -- obstacle_projector_node's footprint rule (the object's
  WIDTH, never its height), also used to size semantic tracks.
* gap_min -- the closest a go_to_object move may ask to stop from an object.

WHY gap_min IS WHAT IT IS. The target of a go_to_object move is also an
obstacle, and the solver's w_obs keeps the car's front car_radius +
avoidance_margin + class_margin from an obstacle's footprint edge:

    centre distance ~= nose_reach + r_target + car_radius
                       + avoidance_margin + class_margin

Measured as a GAP from the car's front (nose_reach ahead of base_link) to the
object's near edge (r_target from its centre), that clearance is
car_radius + avoidance_margin + class_margin, independent of the object's
size; settle_buffer is added on top.

WHAT ENFORCES IT CHANGED WITH THE 0.4 m/s OPERATING FLOOR (2026-09-17). Before
it, w_obs also slowed the car to rest at that clearance, so a commanded gap
below gap_min could not complete: the car settled farther out and
object_reached never fired. Under the floor the solver can no longer slow the
car -- the published speed is held at min_moving_speed_mps until mpc_corr's
stop latch trips -- so a smaller commanded gap WOULD be driven to: the
closed-loop rig brings a commanded gap of 0 to about 0.1 m from the object's
edge. gap_min is therefore a SAFETY minimum now, enforced where the gap is
set: the mission loader rejects a smaller gap_m and the LLM translator raises
one to it. A go_to never asks the car to stop inside the clearance the
obstacle model keeps from everything else.
"""

import math
from dataclasses import dataclass

__all__ = [
    'NOMINAL_FOOTPRINT_WIDTH_M',
    'SOLVER_FRONT_POINT_OFFSETS_M',
    'GapLimits',
    'class_margin_for',
    'default_gap',
    'footprint_radius',
    'gap_limits',
    'gap_min',
    'nose_reach',
    'nominal_footprint_radius',
]

SOLVER_FRONT_POINT_OFFSETS_M = (0.10, 0.25, 0.40)

# Fallback footprint widths [m] for a track that carries no measured width:
# a standing adult's shoulders, a typical chair. Anything else 0.3, the size
# semantic tracks used to carry for every class.
NOMINAL_FOOTPRINT_WIDTH_M = {'person': 0.50, 'chair': 0.45}
_NOMINAL_DEFAULT_WIDTH_M = 0.30


def nose_reach(wheelbase_m: float) -> float:
    """Distance [m] from base_link to the solver's farthest front point."""
    return float(wheelbase_m) / 2.0 + max(SOLVER_FRONT_POINT_OFFSETS_M)


def footprint_radius(size_x: float, size_z: float = 0.0,
                     depth_extent_is_measured: bool = False) -> float:
    """Ground-disk radius [m] from a detection's bbox: width, or width vs measured depth."""
    if depth_extent_is_measured:
        return max(float(size_x), float(size_z)) / 2.0
    return float(size_x) / 2.0


def nominal_footprint_radius(class_id: str) -> float:
    """Fallback radius [m] when no measured width is available."""
    return NOMINAL_FOOTPRINT_WIDTH_M.get(class_id, _NOMINAL_DEFAULT_WIDTH_M) / 2.0


def class_margin_for(margins, class_id: str) -> float:
    """obstacle_class_margin_m's value for class_id (0 when unlisted).

    `margins` is the parsed map, or the JSON text stack_params/ROS carries.
    """
    if isinstance(margins, str):
        import json
        margins = json.loads(margins) if margins.strip() else {}
    return float((margins or {}).get(class_id, 0.0))


def gap_min(car_radius: float, avoidance_margin: float, class_margin: float,
            settle_buffer: float) -> float:
    """Smallest allowed front-to-edge gap [m]; see the module docstring."""
    return float(car_radius) + float(avoidance_margin) + float(class_margin) + float(settle_buffer)


def default_gap(gap_min_m: float) -> float:
    """gap_min rounded UP to the next 0.1 m (an exact multiple stays put).

    Rounded to micrometres first, so float noise on a multiple (0.1 * 5 is
    0.5000000000000001) does not push it up a whole tenth.
    """
    return math.ceil(round(float(gap_min_m) * 10.0, 5)) / 10.0


@dataclass(frozen=True)
class GapLimits:
    """Everything a go_to_object gap depends on, for one target class."""

    target_class: str
    car_radius: float
    avoidance_margin: float
    class_margin: float
    settle_buffer: float
    wheelbase: float

    @property
    def gap_min(self) -> float:
        """Return the smallest allowed gap [m] for this class."""
        return gap_min(self.car_radius, self.avoidance_margin, self.class_margin,
                       self.settle_buffer)

    @property
    def default_gap(self) -> float:
        """Return gap_min rounded up to 0.1 m."""
        return default_gap(self.gap_min)

    @property
    def nose_reach(self) -> float:
        """Return base_link to the solver's farthest front point [m]."""
        return nose_reach(self.wheelbase)

    def explain(self) -> str:
        """Return gap_min spelled out term by term, for rejection messages."""
        return (f'gap_min({self.target_class}) = car_radius {self.car_radius:.2f} + '
                f'avoidance_margin {self.avoidance_margin:.2f} + class_margin '
                f'{self.class_margin:.2f} + settle_buffer {self.settle_buffer:.2f} = '
                f'{self.gap_min:.2f} m')


def gap_limits(target_class: str, get_value=None) -> GapLimits:
    """Build the class's GapLimits from stack_params.yaml (the solver's and projector's keys).

    car_radius and obstacle_safety_margin_m feed MPC_corr's car_radius and
    avoidance_margin (mpc_corr.launch.py); obstacle_class_margin_m feeds
    obstacle_projector_node; mpc_wheelbase_m is the solver's L.
    """
    if get_value is None:
        from f1tenth_params.param_defaults import get_value
    return GapLimits(
        target_class=target_class,
        car_radius=float(get_value('car_radius')),
        avoidance_margin=float(get_value('obstacle_safety_margin_m')),
        class_margin=class_margin_for(get_value('obstacle_class_margin_m'), target_class),
        settle_buffer=float(get_value('object_gap_settle_buffer_m')),
        wheelbase=float(get_value('mpc_wheelbase_m')),
    )
