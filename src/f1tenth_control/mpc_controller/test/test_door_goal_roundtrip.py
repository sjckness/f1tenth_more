"""Door pose (map) -> goal in odom -> back to map is the identity.

Pins the frame maths between enter_door's map-frame goal and what mpc_corr
drives at (odom, via _pose_map_to_odom), with a rotated map -> odom edge and a
door yaw near +-180 deg, where a missing wrap would show as a ~360 deg error.
"""
import math

import pytest

from mpc_controller import door_goal_debug as dgd

@pytest.fixture(scope='module')
def poses():
    """MPC_corr's own conversions (importorskip per CLAUDE.md: never at module level)."""
    mod = pytest.importorskip('mpc_controller.MPC_corr')
    return mod._pose_map_to_odom, mod._pose_odom_to_map

TF = (3.2, -1.4, math.radians(37.0))   # non-zero map -> odom rotation


@pytest.mark.parametrize('yaw_deg', [179.0, -179.0, 180.0, -180.0, 175.0, 0.0, 90.0])
def test_map_odom_map_is_identity(poses, yaw_deg):
    _pose_map_to_odom, _pose_odom_to_map = poses
    door = (5.0, 2.5, math.radians(yaw_deg))
    odom = _pose_map_to_odom(*door, *TF)
    back = _pose_odom_to_map(*odom, *TF)
    assert back[0] == pytest.approx(door[0], abs=1e-9)
    assert back[1] == pytest.approx(door[1], abs=1e-9)
    dyaw = math.atan2(math.sin(back[2] - door[2]), math.cos(back[2] - door[2]))
    assert abs(dyaw) < 1e-9


@pytest.mark.parametrize('yaw_deg', [179.0, -179.0, 180.0])
def test_debug_gap_is_zero_for_the_goal_enter_door_would_send(poses, yaw_deg):
    _pose_map_to_odom, _pose_odom_to_map = poses
    yaw = math.radians(yaw_deg)
    centre = (5.0, 2.5)
    doors = {7: (centre, yaw)}
    # enter_door's APPROACH goal: 1.24 m back on the axis, carrying the door yaw.
    pre = (centre[0] - dgd.PRE_POINT_DIST_M * math.cos(yaw), centre[1] - dgd.PRE_POINT_DIST_M * math.sin(yaw), yaw)
    goal_odom = _pose_map_to_odom(*pre, *TF)
    ev = dgd.evaluate(goal_odom, TF, doors)
    assert ev['door_id'] == 7 and ev['kind'] == 'pre_point'
    assert ev['dist_m'] < 1e-9
    assert abs(ev['dyaw_deg']) < 1e-6


def test_debug_gap_reports_a_flipped_axis_as_180(poses):
    _pose_map_to_odom, _pose_odom_to_map = poses
    doors = {1: ((0.0, 0.0), math.radians(179.0))}
    goal_odom = _pose_map_to_odom(0.0, 0.0, math.radians(-1.0), *TF)  # axis flipped
    ev = dgd.evaluate(goal_odom, TF, doors)
    assert abs(abs(ev['dyaw_deg']) - 180.0) < 1e-6
