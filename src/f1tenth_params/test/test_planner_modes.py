"""planner_modes.py and the planner_controller_mode key it validates.

Run standalone: python3 -m pytest test/test_planner_modes.py -v
"""

from f1tenth_params.param_defaults import get_default
from f1tenth_params.planner_modes import (
    controller_node_name, CONTROLLER_NODE_NAMES, default_planner_controller_mode,
    PLANNER_CONTROLLER_MODES, validate_planner_controller_mode)
import pytest


def test_the_shipped_default_is_the_mpc():
    # Every campaign so far ran corridor_mpc; switching the default is a
    # decision, not a side effect of adding modes.
    assert default_planner_controller_mode() == 'corridor_mpc'
    assert PLANNER_CONTROLLER_MODES[0] == 'corridor_mpc'


def test_every_mode_has_a_node():
    assert set(CONTROLLER_NODE_NAMES) == set(PLANNER_CONTROLLER_MODES)
    assert controller_node_name('corridor_mpc') == 'mpc_corr'
    assert controller_node_name('corridor_rrt') == 'corridor_rrt'
    assert controller_node_name('p2p_geometric') == 'p2p_geometric'


def test_validation_normalises_and_refuses():
    assert validate_planner_controller_mode('  Corridor_RRT ') == 'corridor_rrt'
    with pytest.raises(ValueError):
        validate_planner_controller_mode('rrt')
    with pytest.raises(ValueError):
        validate_planner_controller_mode(True)


def test_the_description_names_every_mode():
    _default, description = get_default('planner_controller_mode')
    for mode in PLANNER_CONTROLLER_MODES:
        assert mode in description
