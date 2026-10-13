"""
The corridor's size: big enough for the whole car to pass a person.

Design case (campaign-final): a person standing on the centreline 1.0 m ahead
of the car's front bumper when the corridor is built. The corridor starts at
base_link (the rear axle), so the person sits at s = bumper offset + 1.0. At
that s the corridor's half-width must hold the passing car's axis at the
clearance the solver keeps from the person, plus the car's far side:

    hw(s) >= r_person + class margin + perception tolerance
             + obstacle_safety_margin_m + 2 * car_radius

Every input except the perception tolerance is read from where the stack
reads it, so a change to any of them that the corridor no longer covers
fails here. ROS-free: MPC_corr.py is read as text, not imported.

Run standalone: python3 -m pytest test/test_corridor_sizing.py -v
"""

import pathlib
import re

from f1tenth_params.corridor_geometry import (
    corridor_curves, CORRIDOR_WIDTH_RATIO)
from f1tenth_params.object_geometry import (
    class_margin_for, nominal_footprint_radius)
from f1tenth_params.param_defaults import get_value
import numpy as np

#: Lateral error of the person's perceived position at 1 m [m]: detection
#: jitter 0.01 (p95) + box-centre bias 0.05 + yaw-coupled track wander 0.04.
#: The derivation is the Part D sizing design (campaign-final).
PERCEPTION_LATERAL_TOLERANCE_M = 0.10

#: Distance from the front bumper to the person's centre [m].
PERSON_AHEAD_OF_BUMPER_M = 1.0

_MPC_CORR = (pathlib.Path(__file__).resolve().parents[1]
             / 'mpc_controller' / 'MPC_corr.py')


def _corr_l_base():
    src = _MPC_CORR.read_text()
    return float(re.search(r'self\.corr_L_base\s*=\s*([0-9.]+)', src).group(1))


def _required_half_width():
    return (nominal_footprint_radius('person')
            + class_margin_for(get_value('obstacle_class_margin_m'), 'person')
            + PERCEPTION_LATERAL_TOLERANCE_M
            + float(get_value('obstacle_safety_margin_m'))
            + 2.0 * float(get_value('car_radius')))


def _shipped_half_width_at(s, length):
    w0 = float(get_value('corr_half_width_start_m'))
    geom = corridor_curves(0.0, 0.0, 0.0, 0.0, length, 120,
                           u_start=0.0, u_end=0.40,
                           w0=w0, w1=w0 * CORRIDOR_WIDTH_RATIO,
                           walls='trapezoid')
    idx = int(np.argmin(np.abs(geom['xc'] - s)))
    return float(geom['halfWidth'][idx])


def test_corridor_holds_the_whole_car_passing_a_person_one_metre_ahead_of_the_bumper():
    s_person = (float(get_value('swept_clearance_body_front_x_m'))
                + PERSON_AHEAD_OF_BUMPER_M)
    have = _shipped_half_width_at(s_person, _corr_l_base())
    need = _required_half_width()
    assert have >= need, (
        f'corridor half-width {have:.4f} m at s = {s_person:.3f} m is below '
        f'the {need:.4f} m the passing car needs')


def test_the_old_corridor_could_not_hold_the_passing_car():
    """The 0.4333 start that ran in the first campaign falls short here."""
    s_person = (float(get_value('swept_clearance_body_front_x_m'))
                + PERSON_AHEAD_OF_BUMPER_M)
    geom = corridor_curves(0.0, 0.0, 0.0, 0.0, 3.0, 120,
                           w0=0.4333, w1=0.4333 * CORRIDOR_WIDTH_RATIO,
                           walls='trapezoid')
    idx = int(np.argmin(np.abs(geom['xc'] - s_person)))
    assert float(geom['halfWidth'][idx]) < _required_half_width()
