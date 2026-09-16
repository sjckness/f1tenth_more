"""Passing a standing person on a goal_distance move, with camera field-of-view dropout.

Nominal-model closed loop, same rig as test_object_approach_closed_loop.py
(real build_straight_corridor goal_distance branch, compute_local_target and
solve_mpc_step around f1tenth_state_fcn_dt_beta), with what the real obstacle
pipeline does to a person the car is passing:

* The person is in /perception/obstacles_2d only while the camera can see
  them. The half field of view comes from the camera's own intrinsics, as
  recorded: fx 262.4 px, cx 318.7 px, 640 px wide (recovered from paired
  /camera/detections and /camera/detections_3d in the archive; no bag records
  camera_info). Left edge atan(318.7 / 262.4) = 50.54 deg, right edge
  atan(321.3 / 262.4) = 50.76 deg; the smaller is used. Bearing is taken from
  the camera, 0.12 m ahead of base_link (camera.launch.py static TF).
* Real persistence is none. yolo_detector_node publishes every frame, empty or
  not; obstacle_projector_node turns an empty frame into an empty list; and
  MPC_corr.obstacles_2d_callback replaces its whole list per message. So the
  person is gone on the first detection frame after they leave the view. The
  list is only refreshed on detection frames (8.2 Hz, measured in the
  2026-09-01 mission analysis) and held in between, as MPC_corr holds its
  last message. compute_local_target's own 5-tick deflection coast is inside
  the real method and applies unchanged.
* mpc_corr's /drive clamp (+1.0 / 0.0 m/s) is applied to the plant.

The person is 0.50 m wide; the obstacle radius is what obstacle_projector_node
would publish: 0.875 in legacy mode, 0.25 + class margin in footprint mode.
Body gap = centre distance - 0.25 - car_radius (disk model).

Print the table:
    python3 -m pytest test/test_person_pass_closed_loop.py -s -k table
"""

import math

import numpy as np
import pytest

from mpc_controller.MPC_corr import MPCController, _project_onto_line
from mpc_controller.drive_limits import clamp_drive_speed
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta

import test_object_approach_closed_loop as rig

CAMERA_FX_PX = 262.4
CAMERA_CX_PX = 318.7
IMAGE_WIDTH_PX = 640
HALF_FOV_RAD = min(math.atan(CAMERA_CX_PX / CAMERA_FX_PX),
                   math.atan((IMAGE_WIDTH_PX - CAMERA_CX_PX) / CAMERA_FX_PX))
CAMERA_AHEAD_OF_BASE_LINK_M = 0.12
DETECTION_PERIOD_S = 1.0 / 8.2

GOAL_DISTANCE_M = 4.0
PERSON_AHEAD_M = 2.0
PERSON_HALF_WIDTH_M = 0.25
OFFSETS_M = (0.0, 0.3, 0.6)
CANDIDATE_MARGINS_M = (0.2, 0.3, 0.4)
REQUIRED_BODY_GAP_M = 0.25
SPEED_LIMITS = (1.0, 0.0)

RADIUS_ROWS = (('legacy', 0.875),) + tuple(
    (f'footprint+{m:.1f}', PERSON_HALF_WIDTH_M + m) for m in (0.0,) + CANDIDATE_MARGINS_M)


def _bearing_from_camera(state, point):
    cam_x = state[0] + CAMERA_AHEAD_OF_BASE_LINK_M * math.cos(state[2])
    cam_y = state[1] + CAMERA_AHEAD_OF_BASE_LINK_M * math.sin(state[2])
    b = math.atan2(point[1] - cam_y, point[0] - cam_x) - state[2]
    return math.atan2(math.sin(b), math.cos(b))


def run_pass(obstacle_r, offset, *, half_fov=HALF_FOV_RAD, duration=25.0):
    """Drive the goal_distance move past the person; return the per-run metrics."""
    fake = rig._ObjectMPC(standoff=1.0, speed=0.5)
    fake.goal_distance = GOAL_DISTANCE_M
    fake.goal_start_xy = (0.0, 0.0)
    fake.goal_anchor_odom = (0.0, 0.0, 0.0)
    fake.psi_init_corridor = 0.0
    person = (PERSON_AHEAD_M, offset)

    x = np.zeros(4)
    last_u = np.zeros(2)
    warm = None
    corridor = None
    last_build = None
    obstacles = []
    next_detection = 0.0
    seen = False
    out = {'min_centre': math.inf, 'dropout_tick': None, 'peak_cross_track': 0.0,
           'peak_speed': 0.0, 'reached_s': None}
    for tick in range(int(round(duration / rig.TS))):
        t = tick * rig.TS
        if t >= next_detection - 1e-9:
            visible = abs(_bearing_from_camera(x, person)) <= half_fov
            if visible:
                seen = True
            elif seen and out['dropout_tick'] is None:
                out['dropout_tick'] = tick
            obstacles = [(person[0], person[1], float(obstacle_r))] if visible else []
            next_detection += DETECTION_PERIOD_S
        if _project_onto_line((x[0], x[1]), (0.0, 0.0), 0.0) >= GOAL_DISTANCE_M:
            out['reached_s'] = t
            break
        if corridor is None or t - last_build >= 1.0:
            corridor = MPCController.build_straight_corridor(fake, x)
            last_build = t
        corridor['obstacles_world'] = obstacles
        corridor['d_safe'] = rig.DMIN
        corridor['car_radius'] = rig.CAR_RADIUS
        corridor['avoidance_margin'] = rig.AVOIDANCE_MARGIN
        pref_nom = MPCController.compute_local_target(fake, x, corridor)
        u0, info = solve_mpc_step(
            x0=x, last_u=last_u, pref_nom=pref_nom, corridor=corridor,
            horizon=rig.HORIZON, ts=rig.TS, params=rig.PARAMS, limits=rig.LIMITS,
            weights=dict(rig.WEIGHTS), obstacles=obstacles, dmin=rig.DMIN, vdes=fake.vdes,
            solver='rti', warm_start_z=warm)
        warm = shift_warm_start(info.get('zopt'), rig.HORIZON) if info else None
        v_cmd, _ = clamp_drive_speed(x[3] + u0[1] * rig.TS, *SPEED_LIMITS)
        u_plant = np.array([u0[0], (v_cmd - x[3]) / rig.TS])
        x = np.array(f1tenth_state_fcn_dt_beta(x, u_plant, rig.TS, rig.WHEELBASE, rig.LR))
        last_u = np.asarray(u0, dtype=float)
        out['min_centre'] = min(out['min_centre'],
                                math.hypot(person[0] - x[0], person[1] - x[1]))
        out['peak_cross_track'] = max(out['peak_cross_track'], abs(x[1]))
        out['peak_speed'] = max(out['peak_speed'], x[3])
    out['min_body_gap'] = out['min_centre'] - PERSON_HALF_WIDTH_M - rig.CAR_RADIUS
    return out


@pytest.fixture(scope='module')
def table():
    return {(label, offset): run_pass(r, offset)
            for label, r in RADIUS_ROWS for offset in OFFSETS_M}


def test_the_half_fov_comes_from_the_camera_intrinsics():
    assert math.degrees(HALF_FOV_RAD) == pytest.approx(50.54, abs=0.01)


def test_print_the_table(table, capsys):
    with capsys.disabled():
        print(f'\n{"obstacle":<14} {"offset":>6} {"min centre":>10} {"body gap":>9} '
              f'{"dropout":>8} {"x-track":>8} {"peak v":>7}  reached')
        for (label, offset), res in table.items():
            dropout = '-' if res['dropout_tick'] is None else str(res['dropout_tick'])
            reached = 'no' if res['reached_s'] is None else f'{res["reached_s"]:.1f} s'
            print(f'{label:<14} {offset:>6.1f} {res["min_centre"]:>10.3f} '
                  f'{res["min_body_gap"]:>+9.3f} {dropout:>8} {res["peak_cross_track"]:>8.3f} '
                  f'{res["peak_speed"]:>7.2f}  {reached}')


def test_no_candidate_margin_keeps_a_quarter_metre_gap_with_dropout(table):
    """The finding that left obstacle_class_margin_m at {} (no m*).

    For every candidate class margin there is an offset whose body gap falls
    below 0.25 m once the person drops out of view. If a controller change
    makes one pass, this fails, and the margin decision is due again.
    """
    passing = [m for m in CANDIDATE_MARGINS_M
               if all(table[(f'footprint+{m:.1f}', off)]['min_body_gap'] >= REQUIRED_BODY_GAP_M
                      for off in OFFSETS_M)]
    assert passing == []


def test_the_person_drops_out_of_view_on_every_completed_pass(table):
    for (label, offset), res in table.items():
        if res['reached_s'] is not None:
            assert res['dropout_tick'] is not None, (label, offset)


def test_the_clamp_holds_throughout(table):
    for res in table.values():
        assert res['peak_speed'] <= SPEED_LIMITS[0] + 1e-9


def test_legacy_radius_blocks_a_person_dead_ahead(table):
    """0.875 m disk on the line: w_obs holds the car short and the move never completes."""
    assert table[('legacy', 0.0)]['reached_s'] is None
