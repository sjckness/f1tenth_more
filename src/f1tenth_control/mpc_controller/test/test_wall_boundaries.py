"""Wall-row tests: the tracked-wall half-plane source for the MPC's boundary rows.

Synthetic throughout -- no stack, no node, no bag. The admission rule, the
geometry, the merge, the row identity and the staleness gate are pure functions
in mpc_controller/wall_boundaries.py; the three scenarios that have to prove the
rows reach the SOLVE do real (light) OSQP solves, reusing
test_boundary_constraints.py's own straight-corridor harness and the same
"no prior synthetic-scenario infrastructure existed" precedent test_solver_rti.py
set.

MPCController itself is never constructed, here or anywhere in this suite: its
__init__ opens real log files and starts a real control-loop timer. Everything
this file needs from the node is therefore a module-level pure function
(wb.check_row_capacity, wb.stale_reason), which is why those two exist in that
shape rather than as inline checks.

Run standalone: python3 -m pytest test/test_wall_boundaries.py -v
"""

import math

import numpy as np
import pytest

from mpc_controller import wall_boundaries as wb
from mpc_controller.mpc_solver import pad_boundary_constraints, solve_mpc_step

WHEELBASE = 0.305
LR = 0.17
TS = 0.1
HORIZON = 7
CAR_RADIUS = 0.20
MARGIN = 0.12

LIMITS = {
    'delta_min': -1.05, 'delta_max': 1.05,
    'a_min': -2.0, 'a_max': 3.0,
    'dDeltaMin': -0.5, 'dDeltaMax': 0.5,
    'dAMin': -2.0, 'dAMax': 2.0,
    'vMin': -1.0, 'vMax': 3.0,
}
WEIGHTS = {
    'w_term': 3.0, 'w_v': 8.0, 'w_psi': 0.0, 'w_u_a': 0.0,
    'w_du_delta': 15.0, 'w_du_a': 0.0, 'w_delta0': 0.0,
    'w_obs': 8.0, 'w_corr': 0.0,
}
PARAMS = {'L': WHEELBASE, 'lr': LR}

# The shipped wall-row slack weight (stack_params wall_row_slack_weight). The
# tests that solve use the real number, not a convenient one: its whole claim
# is that OSQP stays usable at this magnitude, and a test at 1e4 would not
# check that.
WALL_SLACK_WEIGHT = 2.0e6

# wall_boundaries.py mirrors MultiWallTrack.msg's enumerations rather than
# importing them (it is deliberately free of f1tenth_messages). This is the
# test that keeps the copy honest -- see test_mirrored_constants_match_the_message.
MSG_STATES = {'STATE_TENTATIVE': 0, 'STATE_CONFIRMED': 1, 'STATE_COASTING': 2}
MSG_SOURCES = {'SOURCE_LIDAR': 0, 'SOURCE_STEREO': 1, 'SOURCE_BOTH': 2}


def wall(role='left', state=wb.STATE_CONFIRMED, source=wb.SOURCE_LIDAR,
         rho=1.0, theta=math.pi / 2.0, p0=(-5.0, 1.0), p1=(5.0, 1.0), track_id=1):
    """One WallTrackView. Defaults: a 10 m left wall 1 m from the odom origin."""
    return wb.WallTrackView(track_id=track_id, role=role, state=state, source=source,
                            rho=rho, theta=theta, p0=p0, p1=p1)


def line_through(p0, p1):
    """(rho, theta) of the infinite line through p0, p1, as the tracker states it.

    rho >= 0 and theta is the normal from the ODOM ORIGIN toward the line --
    the convention MultiWallTrack.msg documents, reproduced here so the test
    fixtures are built the same way the tracker builds them rather than by
    hand-picking numbers that happen to work.
    """
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    theta = math.atan2(dy, dx) + math.pi / 2.0
    rho = p0[0] * math.cos(theta) + p0[1] * math.sin(theta)
    if rho < 0.0:
        theta = math.atan2(math.sin(theta + math.pi), math.cos(theta + math.pi))
        rho = -rho
    return rho, theta


def room_walls(half=3.0, state=wb.STATE_CONFIRMED, source=wb.SOURCE_LIDAR):
    """The four walls of a 2*half square room centred on the odom origin.

    Roles are the geometric ones the tracker would assign to a car at the
    centre facing +x: the wall ahead is 'front', the two beside it are
    'left'/'right'. The wall BEHIND is deliberately role 'back' -- which the
    admission rule excludes along with 'other' -- so this fixture also carries
    a fourth wall that must NOT produce a row, and the four-row expectation in
    test 1 is built from three real walls plus one admitted 'front' at the far
    end. See the test for which four.
    """
    corners = {
        'front': ((half, -half), (half, half)),
        'left': ((-half, half), (half, half)),
        'right': ((-half, -half), (half, -half)),
        'back': ((-half, -half), (-half, half)),
    }
    out = []
    for i, (role, (a, b)) in enumerate(corners.items()):
        rho, theta = line_through(a, b)
        out.append(wall(role=role, state=state, source=source, rho=rho, theta=theta,
                        p0=a, p1=b, track_id=10 + i))
    return out


def _straight_corridor(x0, length=5.0, half_width=2.0, n=60):
    """test_boundary_constraints.py's own straight-corridor fixture, verbatim in shape."""
    X0, Y0, psi0 = float(x0[0]), float(x0[1]), float(x0[2])
    u = np.linspace(0.0, 1.0, n)
    s = length * u
    xc = X0 + s * np.cos(psi0)
    yc = Y0 + s * np.sin(psi0)
    nx = np.full(n, -np.sin(psi0))
    ny = np.full(n, np.cos(psi0))
    return {
        'xc': xc, 'yc': yc,
        'xL': xc + half_width * nx, 'yL': yc + half_width * ny,
        'xR': xc - half_width * nx, 'yR': yc - half_width * ny,
        'tx': np.full(n, np.cos(psi0)), 'ty': np.full(n, np.sin(psi0)),
        'nx': nx, 'ny': ny,
        'halfWidth': np.full(n, half_width),
        'psiRef': psi0, 't': 0.0,
        'Pend': np.array([xc[-1], yc[-1]]),
        'dFront': 10.0, 'dpsi': 0.0,
        'obstacles_world': [],
        'car_radius': CAR_RADIUS,
        'avoidance_margin': MARGIN,
    }


def build(tracks, car_xy=(0.0, 0.0), path=None, stamp=1.0, clock=None, identity=None,
          max_rows=4, max_coast_s=1.0, coast_margin_m=0.10, allow_stereo_only=False,
          end_margin_m=0.3, merge_angle_deg=10.0, merge_offset_m=0.3):
    """build_wall_rows at the shipped defaults, with the clock already updated."""
    clock = clock if clock is not None else wb.CoastClock()
    identity = identity if identity is not None else wb.RowIdentity(
        merge_angle_deg, merge_offset_m)
    clock.update(tracks, stamp)
    if path is None:
        path = [(x, 0.0) for x in np.linspace(0.0, 2.0, 11)]
    return wb.build_wall_rows(
        tracks=tracks, car_xy=car_xy, path_xy=path, stamp=stamp,
        coast_clock=clock, identity=identity, max_rows=max_rows,
        max_coast_s=max_coast_s, coast_margin_m=coast_margin_m,
        allow_stereo_only=allow_stereo_only, end_margin_m=end_margin_m,
        merge_angle_deg=merge_angle_deg, merge_offset_m=merge_offset_m)


def solve(boundaries, x0, vdes=1.0, slack_weight=WALL_SLACK_WEIGHT, max_sources=4,
          corridor=None):
    """One light RTI solve with `boundaries` as the wall rows."""
    corridor = corridor if corridor is not None else _straight_corridor(x0)
    return solve_mpc_step(
        x0=x0, last_u=np.array([0.0, 0.0]), pref_nom=corridor['Pend'],
        corridor=corridor, horizon=HORIZON, ts=TS, params=PARAMS, limits=LIMITS,
        weights=WEIGHTS, obstacles=[], dmin=0.9, vdes=vdes, solver='rti',
        boundaries=boundaries, boundary_max_sources=max_sources,
        boundary_hard=False, boundary_slack_weight=slack_weight)


def test_mirrored_constants_match_the_message():
    """wall_boundaries' copies of MultiWallTrack's enumerations are still right.

    The module mirrors them rather than importing f1tenth_messages so it stays
    pure (its own docstring says so). A mirror nobody checks is a trap: the
    message could renumber STATE_COASTING and every admission test would keep
    passing while admitting the wrong tracks.
    """
    try:
        from f1tenth_messages.msg import MultiWallTrack
    except ImportError:  # pragma: no cover - messages not built in this env
        pytest.skip('f1tenth_messages not available')
    for name, value in MSG_STATES.items():
        assert getattr(MultiWallTrack, name) == value
    for name, value in MSG_SOURCES.items():
        assert getattr(MultiWallTrack, name) == value
    assert wb.STATE_TENTATIVE == MultiWallTrack.STATE_TENTATIVE
    assert wb.STATE_CONFIRMED == MultiWallTrack.STATE_CONFIRMED
    assert wb.STATE_COASTING == MultiWallTrack.STATE_COASTING
    assert wb.SOURCE_LIDAR == MultiWallTrack.SOURCE_LIDAR
    assert wb.SOURCE_STEREO == MultiWallTrack.SOURCE_STEREO
    assert wb.SOURCE_BOTH == MultiWallTrack.SOURCE_BOTH


# ==============================================================================
# 1. Four-wall room: four rows, zero slack, the trajectory stays inside.
# ==============================================================================

class TestFourWallRoom:

    def test_three_eligible_walls_of_a_room_become_rows_the_back_one_does_not(self):
        # A 6x6 room, car at the centre facing +x, corridor straight ahead.
        # front/left/right are eligible roles; 'back' is not (it is behind the
        # car by construction -- a half-plane from it could only ever constrain
        # a reversing horizon).
        tracks = room_walls(half=3.0)
        path = [(x, 0.0) for x in np.linspace(0.0, 2.5, 11)]
        rows, diag = build(tracks, car_xy=(0.0, 0.0), path=path)

        assert sorted(r.role for r in rows) == ['front', 'left', 'right']
        back = [d for d in diag if d['role'] == 'back']
        assert len(back) == 1 and back[0]['reason'] == 'role_back', (
            'the wall behind the car must be rejected BY ROLE, with that reason '
            f'reported rather than silently dropped -- got {back}')

    def test_every_row_points_away_from_the_car_and_the_car_satisfies_it(self):
        tracks = room_walls(half=3.0)
        rows, _ = build(tracks, car_xy=(0.0, 0.0))
        for row in rows:
            value = row.normal_x * 0.0 + row.normal_y * 0.0
            assert value <= row.offset + 1e-9, (
                f'{row} -- the car at the odom origin must satisfy its own row; '
                'a normal pointing the wrong way encloses the car on the wall side')
            assert row.offset > 0.0, f'{row} -- a room 3 m away cannot give offset <= 0'

    def test_the_rows_hold_the_solve_inside_the_room_with_no_slack(self):
        # Four rows at once (front at 3 m plus left/right at 3 m, padded to 4),
        # a 5 m corridor target that WOULD leave the room, and a gentle
        # starting speed. The front row has to bite and nothing may be violated.
        x0 = np.array([0.0, 0.0, 0.0, 0.3])
        tracks = room_walls(half=3.0)
        path = [(x, 0.0) for x in np.linspace(0.0, 5.0, 21)]
        rows, _ = build(tracks, car_xy=(0.0, 0.0), path=path)
        _u0, info = solve([r.constraint for r in rows], x0)

        assert info['success'], f'a 6x6 room must stay solvable: {info.get("status")}'
        assert max(info['boundary_slack']) < 1e-4, (
            'the car starts at the centre of a 6x6 room: no row needs violating, '
            f'so every slack must be ~0 -- got {info["boundary_slack"]}')
        x_pred = np.asarray(info['x_pred'], dtype=float)
        for row in rows:
            margin = row.offset - CAR_RADIUS - MARGIN
            worst = np.max(x_pred[:, 0] * row.normal_x + x_pred[:, 1] * row.normal_y)
            assert worst <= margin + 1e-3, (
                f'{row} -- predicted trajectory crosses its own half-plane by '
                f'{worst - margin:.4f} m')


# ==============================================================================
# 2. A short wall with an open end: the gap past the end stays feasible.
# ==============================================================================

class TestOpenEnd:

    def test_a_row_is_inactive_where_the_path_passes_beyond_the_wall_s_end(self):
        # A 1 m partition from x=-1 to x=0, 0.5 m to the car's left. The car
        # drives +x, i.e. past its end: nothing the partition bounds is in the
        # way, and the row must switch off rather than close the whole +x half
        # of the room at y=0.5.
        p0, p1 = (-1.0, 0.5), (0.0, 0.5)
        rho, theta = line_through(p0, p1)
        track = wall(role='left', rho=rho, theta=theta, p0=p0, p1=p1)
        path = [(x, 0.0) for x in np.linspace(1.0, 4.0, 13)]
        rows, diag = build([track], car_xy=(1.0, 0.0), path=path)

        assert rows == [], f'the row must be inactive past the wall end -- got {rows}'
        assert diag[0]['reason'] == 'path_past_segment_end'

    def test_the_same_wall_does_produce_a_row_while_the_path_is_beside_it(self):
        p0, p1 = (-1.0, 0.5), (0.0, 0.5)
        rho, theta = line_through(p0, p1)
        track = wall(role='left', rho=rho, theta=theta, p0=p0, p1=p1)
        path = [(x, 0.0) for x in np.linspace(-1.0, 0.0, 11)]
        rows, _ = build([track], car_xy=(-1.0, 0.0), path=path)
        assert len(rows) == 1 and rows[0].active

    def test_the_end_margin_is_what_decides_the_boundary_case(self):
        # Path starting 0.2 m past the end: inside the 0.3 m end margin, so the
        # row is still active. Starting 0.4 m past: outside, so it is not.
        p0, p1 = (-1.0, 0.5), (0.0, 0.5)
        rho, theta = line_through(p0, p1)
        track = wall(role='left', rho=rho, theta=theta, p0=p0, p1=p1)
        near, far = build([track], path=[(0.2, 0.0)])[0], build(
            [track], path=[(0.4, 0.0)])[0]
        assert len(near) == 1, 'a path 0.2 m past the end is inside the 0.3 m margin'
        assert far == [], 'a path 0.4 m past the end is outside it'

    def test_the_gap_past_the_end_stays_solvable(self):
        # The real consequence: with the row switched off, a target 5 m along
        # +x -- straight through where the partition's infinite half-plane
        # would have been -- solves and the trajectory goes there.
        x0 = np.array([1.0, 0.0, 0.0, 0.3])
        p0, p1 = (-1.0, 0.5), (0.0, 0.5)
        rho, theta = line_through(p0, p1)
        track = wall(role='left', rho=rho, theta=theta, p0=p0, p1=p1)
        corridor = _straight_corridor(x0)
        rows, _ = build([track], car_xy=(1.0, 0.0),
                        path=list(zip(corridor['xc'], corridor['yc'])))
        _u0, info = solve([r.constraint for r in rows], x0, corridor=corridor)
        assert info['success']
        x_pred = np.asarray(info['x_pred'], dtype=float)
        assert x_pred[-1, 0] > x0[0], 'the car must still be allowed forward'


# ==============================================================================
# 3. tentative -> no row; confirmed 'other' -> no row, but a soft obstacle.
# ==============================================================================

class TestAdmission:

    def test_a_tentative_wall_never_becomes_a_row(self):
        rows, diag = build([wall(state=wb.STATE_TENTATIVE)])
        assert rows == []
        assert diag[0]['reason'] == 'tentative'

    def test_a_confirmed_other_wall_never_becomes_a_row(self):
        rows, diag = build([wall(role='other')])
        assert rows == []
        assert diag[0]['reason'] == 'role_other'

    def test_a_confirmed_other_wall_becomes_soft_obstacle_disks(self):
        track = wall(role='other', p0=(2.0, 1.0), p1=(3.0, 1.0))
        disks = wb.other_wall_obstacles([track], spacing_m=0.25, radius_m=0.0)
        assert len(disks) == 5, f'1 m at 0.25 m spacing is 5 samples -- got {len(disks)}'
        assert all(abs(y - 1.0) < 1e-9 for _x, y, _r in disks)
        assert min(x for x, _y, _r in disks) == pytest.approx(2.0)
        assert max(x for x, _y, _r in disks) == pytest.approx(3.0)

    def test_a_tentative_other_wall_becomes_nothing_at_all(self):
        track = wall(role='other', state=wb.STATE_TENTATIVE, p0=(2.0, 1.0), p1=(3.0, 1.0))
        assert wb.other_wall_obstacles([track], 0.25, 0.0) == [], (
            'an unconfirmed track nowhere near the path has cleared no test; the '
            'w_obs cost is exactly where a phantom would quietly bend every plan')

    def test_an_eligible_role_is_not_turned_into_an_obstacle_as_well(self):
        assert wb.other_wall_obstacles([wall(role='left')], 0.25, 0.0) == [], (
            'a wall that is already a ROW must not be double-counted as a disk')

    def test_a_stereo_only_wall_never_becomes_a_row_by_default(self):
        rows, diag = build([wall(source=wb.SOURCE_STEREO)])
        assert rows == []
        assert diag[0]['reason'] == 'stereo_only'

    def test_a_stereo_only_wall_is_admitted_when_the_param_says_so(self):
        rows, _ = build([wall(source=wb.SOURCE_STEREO)], allow_stereo_only=True)
        assert len(rows) == 1

    def test_source_both_is_always_eligible(self):
        rows, _ = build([wall(source=wb.SOURCE_BOTH)])
        assert len(rows) == 1

    def test_the_role_test_runs_before_the_evidence_test(self):
        # A stereo-only 'other' wall fails both. The reported reason must be
        # the role: it is a reference wall for nothing however well observed,
        # so 'stereo_only' would name a cause that is not why it was rejected.
        _rows, diag = build([wall(role='other', source=wb.SOURCE_STEREO)])
        assert diag[0]['reason'] == 'role_other'


# ==============================================================================
# 4. Coasting: 0.5 s -> a row with extra margin; 2 s -> no row.
# ==============================================================================

class TestCoasting:

    def _coasted(self, coast_for, max_coast_s=1.0):
        """Confirm a wall at t=0, then coast the same track id for `coast_for`."""
        clock = wb.CoastClock()
        identity = wb.RowIdentity(10.0, 0.3)
        confirmed = wall(state=wb.STATE_CONFIRMED, track_id=7)
        build([confirmed], stamp=0.0, clock=clock, identity=identity)
        coasting = wall(state=wb.STATE_COASTING, track_id=7)
        return build([coasting], stamp=coast_for, clock=clock, identity=identity,
                     max_coast_s=max_coast_s)

    def test_half_a_second_of_coasting_still_produces_a_row(self):
        rows, _ = self._coasted(0.5)
        assert len(rows) == 1
        assert rows[0].state == 'coasting'
        assert rows[0].coast_age_s == pytest.approx(0.5)

    def test_that_row_carries_the_extra_coast_margin(self):
        coasted, _ = self._coasted(0.5)
        fresh, _ = build([wall(state=wb.STATE_CONFIRMED)])
        assert coasted[0].offset == pytest.approx(fresh[0].offset - 0.10), (
            'a coasting wall is a prediction carried on odometry, so its row '
            'must sit wall_row_coast_margin_m further away than a confirmed '
            f'one: {coasted[0].offset} vs {fresh[0].offset}')

    def test_two_seconds_of_coasting_produces_no_row(self):
        rows, diag = self._coasted(2.0)
        assert rows == []
        assert diag[0]['reason'] == 'coast_expired'
        assert diag[0]['coast_age_s'] == pytest.approx(2.0)

    def test_a_confirmed_wall_has_no_coast_margin_however_old_the_track(self):
        clock = wb.CoastClock()
        identity = wb.RowIdentity(10.0, 0.3)
        for stamp in (0.0, 1.0, 5.0):
            rows, _ = build([wall(state=wb.STATE_CONFIRMED, track_id=7)],
                            stamp=stamp, clock=clock, identity=identity)
        assert rows[0].coast_age_s == 0.0
        assert rows[0].offset == pytest.approx(1.0), (
            'a CONFIRMED track is being observed right now, so no amount of age '
            'earns it the coasting standoff')

    def test_a_track_first_seen_already_coasting_starts_its_clock_then(self):
        # The MPC restarting mid-coast, or joining a run in progress. There is
        # no last-confirmed stamp to measure from, so the clock starts at first
        # sight: the wall gets at most wall_row_max_coast_s from now, which
        # expires it sooner than the truth -- the safe direction.
        clock = wb.CoastClock()
        identity = wb.RowIdentity(10.0, 0.3)
        coasting = wall(state=wb.STATE_COASTING, track_id=99)
        rows, _ = build([coasting], stamp=100.0, clock=clock, identity=identity)
        assert len(rows) == 1 and rows[0].coast_age_s == 0.0
        rows, diag = build([coasting], stamp=101.5, clock=clock, identity=identity)
        assert rows == [] and diag[0]['reason'] == 'coast_expired'


# ==============================================================================
# 5. Two tracks on one wall (offsets 0.67 / 0.85, same normal) -> one row.
# ==============================================================================

class TestMerge:

    def _two_tracks_one_wall(self):
        near = wall(role='left', rho=0.67, theta=math.pi / 2.0,
                    p0=(-2.0, 0.67), p1=(2.0, 0.67), track_id=1)
        far = wall(role='left', rho=0.85, theta=math.pi / 2.0,
                   p0=(-2.0, 0.85), p1=(2.0, 0.85), track_id=2)
        return [near, far]

    def test_the_two_collapse_to_one_row(self):
        rows, _ = build(self._two_tracks_one_wall())
        assert len(rows) == 1, (
            'the tracker legitimately holds two tracks for one surface; two rows '
            f'cost two of four slots and two slack variables for it -- got {rows}')

    def test_the_surviving_row_is_the_restrictive_one(self):
        rows, _ = build(self._two_tracks_one_wall())
        assert rows[0].offset == pytest.approx(0.67), (
            'most restrictive means SMALLEST offset: the row is n . p <= offset, '
            f'so 0.67 is the tighter of the two -- got {rows[0].offset}')

    def test_the_merged_row_still_names_both_tracks(self):
        rows, _ = build(self._two_tracks_one_wall())
        assert rows[0].track_ids == (1, 2), (
            'provenance survives the merge, or the diagnostic cannot say which '
            f'tracks produced the row -- got {rows[0].track_ids}')

    def test_two_genuinely_different_walls_do_not_merge(self):
        left = wall(role='left', rho=0.67, theta=math.pi / 2.0,
                    p0=(-2.0, 0.67), p1=(2.0, 0.67), track_id=1)
        right_rho, right_theta = line_through((-2.0, -1.2), (2.0, -1.2))
        right = wall(role='right', rho=right_rho, theta=right_theta,
                     p0=(-2.0, -1.2), p1=(2.0, -1.2), track_id=2)
        rows, _ = build([left, right])
        assert len(rows) == 2, (
            'opposite walls of a corridor have opposite normals once oriented '
            'away from the car; the angle test must keep them apart')

    def test_the_offset_tolerance_is_what_keeps_two_parallel_walls_apart(self):
        near = wall(role='left', rho=0.67, theta=math.pi / 2.0,
                    p0=(-2.0, 0.67), p1=(2.0, 0.67), track_id=1)
        far = wall(role='left', rho=1.4, theta=math.pi / 2.0,
                   p0=(-2.0, 1.4), p1=(2.0, 1.4), track_id=2)
        rows, _ = build([near, far])
        assert len(rows) == 2, '0.73 m apart is past the 0.3 m merge tolerance'


# ==============================================================================
# 6. Track-id re-birth, same geometry -> same row id, no slack jump.
# ==============================================================================

class TestRowIdentity:

    def test_a_rebirth_under_a_new_track_id_keeps_the_row_id(self):
        clock, identity = wb.CoastClock(), wb.RowIdentity(10.0, 0.3)
        first, _ = build([wall(track_id=41)], stamp=0.0, clock=clock, identity=identity)
        # Same surface, new id -- what the tracker does when association loses
        # a wall and re-finds it. Ids are never reused there.
        again, _ = build([wall(track_id=42)], stamp=0.2, clock=clock, identity=identity)
        assert first[0].row_id == again[0].row_id, (
            'row identity must be geometry, not a track id: a diagnostic keyed on '
            'the id flickers on every re-birth and a slack figure keyed on it '
            'reads as a fresh violation')
        assert again[0].track_ids == (42,), 'the new track id is still reported'

    def test_the_row_geometry_is_unchanged_by_the_rebirth(self):
        clock, identity = wb.CoastClock(), wb.RowIdentity(10.0, 0.3)
        first, _ = build([wall(track_id=41)], stamp=0.0, clock=clock, identity=identity)
        again, _ = build([wall(track_id=42)], stamp=0.2, clock=clock, identity=identity)
        assert again[0].offset == pytest.approx(first[0].offset)
        assert again[0].normal_x == pytest.approx(first[0].normal_x)
        assert again[0].normal_y == pytest.approx(first[0].normal_y)

    def test_the_slack_the_solver_buys_does_not_jump_across_the_rebirth(self):
        # The consequence that matters: identical rows in, identical slack out.
        # A row id is bookkeeping; this is the number a human reads.
        x0 = np.array([0.0, 0.0, 0.0, 0.3])
        clock, identity = wb.CoastClock(), wb.RowIdentity(10.0, 0.3)
        tight = dict(role='front', rho=0.6, theta=0.0, p0=(0.6, -2.0), p1=(0.6, 2.0))
        first, _ = build([wall(track_id=41, **tight)], stamp=0.0,
                         clock=clock, identity=identity)
        again, _ = build([wall(track_id=42, **tight)], stamp=0.2,
                         clock=clock, identity=identity)
        s1 = solve([r.constraint for r in first], x0)[1]['boundary_slack']
        s2 = solve([r.constraint for r in again], x0)[1]['boundary_slack']
        assert s1 == pytest.approx(s2, abs=1e-9)

    def test_a_genuinely_new_wall_gets_a_genuinely_new_row_id(self):
        clock, identity = wb.CoastClock(), wb.RowIdentity(10.0, 0.3)
        first, _ = build([wall(track_id=1)], stamp=0.0, clock=clock, identity=identity)
        other = wall(role='left', rho=2.0, theta=math.pi / 2.0,
                     p0=(-2.0, 2.0), p1=(2.0, 2.0), track_id=2)
        rows, _ = build([wall(track_id=1), other], stamp=0.1,
                        clock=clock, identity=identity)
        ids = {r.row_id for r in rows}
        assert first[0].row_id in ids and len(ids) == 2


# ==============================================================================
# 7. A stale (or wrong-frame) wall message -> no rows at all, and a reason.
# ==============================================================================

class TestStaleness:

    def test_a_fresh_odom_frame_message_is_usable(self):
        assert wb.stale_reason('odom', 'odom', 0.05, 0.5) == ''

    def test_a_message_past_the_age_limit_withholds_every_row(self):
        reason = wb.stale_reason('odom', 'odom', 0.9, 0.5)
        assert reason and '0.90 s old' in reason, (
            'no rows is "I am not constraining you", which the solver already '
            'treats as unconstrained; a held-over row is a wall asserted where '
            f'the car has since driven -- got {reason!r}')

    def test_never_having_received_a_message_withholds_every_row(self):
        assert wb.stale_reason('', 'odom', None, 0.5) == (
            'walls: no message ever received')

    def test_a_frame_that_is_not_the_mpc_s_odom_withholds_every_row(self):
        reason = wb.stale_reason('map', 'odom', 0.05, 0.5)
        assert reason and 'not the MPC odom frame' in reason, (
            'a frame mismatch is a configuration error and must not be papered '
            'over by transforming: these two agree about odom by construction')

    def test_the_frame_check_runs_before_the_age_check(self):
        # Both wrong. The frame is the one worth reporting: an age warning sends
        # somebody looking at the LiDAR when the fault is a remap.
        assert 'not the MPC odom frame' in wb.stale_reason('map', 'odom', 9.0, 0.5)


# ==============================================================================
# 8. Four admitted walls against a pad of 3 -> a startup error.
# ==============================================================================

class TestRowCapacity:

    def test_four_wall_rows_against_a_pad_of_three_is_a_startup_error(self):
        error = wb.check_row_capacity('walls', 4, 3)
        assert error, 'a truncation this silent must stop the node starting'
        assert 'TRUNCATE 1 wall row' in error, error

    def test_the_error_is_what_the_node_raises(self):
        # Mirrors MPCController.__init__'s one line, so the test covers the
        # actual failure mode rather than a predicate nobody calls.
        error = wb.check_row_capacity('walls', 4, 3)
        with pytest.raises(RuntimeError, match='boundary_max_sources'):
            raise RuntimeError(error)

    def test_a_pad_matching_the_cap_is_fine(self):
        assert wb.check_row_capacity('walls', 4, 4) == ''

    def test_a_bigger_pad_than_the_cap_is_fine(self):
        assert wb.check_row_capacity('walls', 4, 8) == ''

    def test_the_check_does_not_fire_in_the_other_two_modes(self):
        # The costmap path is still fine at 3 rows per stage (front/left/right
        # is all it ever produces), so a run that goes back to it by lowering
        # boundary_max_sources with it must not be refused for a cap that only
        # applies to wall rows. The shipped pair is 4/4 since the avoidance
        # pass -- see test_navigation_forwarding.py, which pins that.
        assert wb.check_row_capacity('costmap', 4, 3) == ''
        assert wb.check_row_capacity('none', 4, 3) == ''

    def test_the_cap_itself_is_honoured_and_the_surplus_is_reported(self):
        # Five eligible parallel walls, far enough apart not to merge, max 4.
        tracks = []
        for i, y in enumerate((0.5, 1.0, 1.5, 2.0, 2.5)):
            tracks.append(wall(role='left', rho=y, theta=math.pi / 2.0,
                               p0=(-2.0, y), p1=(2.0, y), track_id=i + 1))
        rows, diag = build(tracks, max_rows=4)
        assert len(rows) == 4
        assert [r.offset for r in rows] == sorted(r.offset for r in rows), (
            'the kept rows are the nearest ones: ranked by clearance to the path')
        dropped = [d for d in diag if d['reason'] == 'over_max_wall_constraints']
        assert len(dropped) == 1 and dropped[0]['offset'] == pytest.approx(2.5)

    def test_the_pad_count_is_what_the_solver_actually_builds(self):
        # The mechanism behind the startup error, stated once: four rows handed
        # to a pad of three lose one, with nothing to say which.
        padded = pad_boundary_constraints(
            [(1.0, 0.0, 1.0), (0.0, 1.0, 2.0), (-1.0, 0.0, 3.0), (0.0, -1.0, 4.0)],
            max_sources=3)
        assert len(padded) == 3 and (0.0, -1.0, 4.0) not in padded


# ==============================================================================
# 9. The car already inside the margin: the solve returns, slack > 0, usable.
# ==============================================================================

class TestAlreadyInsideTheMargin:

    def _inside(self):
        """Build a front wall 0.25 m ahead: inside car_radius + margin (0.32 m)."""
        rho, theta = line_through((0.25, -2.0), (0.25, 2.0))
        return wall(role='front', rho=rho, theta=theta,
                    p0=(0.25, -2.0), p1=(0.25, 2.0))

    def test_the_row_is_already_violated_by_the_car_s_own_position(self):
        rows, _ = build([self._inside()], car_xy=(0.0, 0.0), path=[(0.0, 0.0)])
        assert len(rows) == 1
        assert rows[0].offset - CAR_RADIUS - MARGIN < 0.0, (
            'the premise of this scenario: the solver bound is negative, so even '
            'the car standing still violates the row')

    def test_the_solve_still_returns_a_usable_command(self):
        x0 = np.array([0.0, 0.0, 0.0, 0.2])
        rows, _ = build([self._inside()], car_xy=(0.0, 0.0), path=[(0.0, 0.0)])
        u0, info = solve([r.constraint for r in rows], x0, vdes=0.5)
        assert info['success'], (
            'soft rows exist precisely so this degrades to a reported violation '
            f'instead of an infeasible QP: {info.get("status")}')
        assert np.all(np.isfinite(u0))
        assert LIMITS['delta_min'] <= u0[0] <= LIMITS['delta_max']
        assert LIMITS['a_min'] <= u0[1] <= LIMITS['a_max']

    def test_the_violation_is_reported_rather_than_hidden(self):
        x0 = np.array([0.0, 0.0, 0.0, 0.2])
        rows, _ = build([self._inside()], car_xy=(0.0, 0.0), path=[(0.0, 0.0)])
        _u0, info = solve([r.constraint for r in rows], x0, vdes=0.5)
        assert max(info['boundary_slack']) > 1e-3, (
            'a violated soft row must show up as metres of slack -- that is the '
            f'whole bargain against a hard row: {info["boundary_slack"]}')
        assert max(info['boundary_slack']) <= 0.5 + 1e-6, (
            'and it stays inside boundary_slack_max, which keeps the QP bounded')

    def test_the_wall_slack_weight_keeps_the_solver_usable(self):
        # The claim wall_row_slack_weight's own derivation makes: 2.0e6 is large
        # enough to dominate the objective and small enough that OSQP still
        # converges. Checked at the shipped number, against a row the solve can
        # satisfy by steering.
        x0 = np.array([0.0, 0.0, 0.0, 0.3])
        rho, theta = line_through((1.0, -2.0), (1.0, 2.0))
        track = wall(role='front', rho=rho, theta=theta, p0=(1.0, -2.0), p1=(1.0, 2.0))
        rows, _ = build([track], path=[(x, 0.0) for x in np.linspace(0.0, 3.0, 13)])
        _u0, info = solve([r.constraint for r in rows], x0,
                          slack_weight=WALL_SLACK_WEIGHT)
        assert info['success'], (
            f'OSQP must still converge at the shipped slack weight: {info.get("status")}')
        assert max(info['boundary_slack']) < 1e-3, (
            'a satisfiable wall must cost no slack at all at this weight -- the '
            'linear term exists to stop a free fraction of a millimetre: '
            f'{info["boundary_slack"]}')


# ==============================================================================
# Geometry details the scenarios above rest on.
# ==============================================================================

class TestHalfPlaneGeometry:

    def test_the_normal_flips_when_the_car_crosses_the_wall_s_plane(self):
        # The same track, the car on either side. (rho, theta) is measured from
        # the ODOM ORIGIN, so "away from the car" is a question that has to be
        # re-asked every cycle -- a car driving through a doorway flips sides
        # with the track unchanged.
        track = wall(role='left', rho=1.0, theta=math.pi / 2.0,
                     p0=(-2.0, 1.0), p1=(2.0, 1.0))
        below = wb.half_plane_from_track(track, (0.0, 0.0))
        above = wb.half_plane_from_track(track, (0.0, 2.0))
        assert below[1] == pytest.approx(1.0) and below[2] == pytest.approx(1.0)
        assert above[1] == pytest.approx(-1.0) and above[2] == pytest.approx(-1.0)
        for nx, ny, offset in (below, above):
            car = (0.0, 0.0) if (nx, ny, offset) == below else (0.0, 2.0)
            assert nx * car[0] + ny * car[1] <= offset + 1e-9

    def test_the_offset_is_raw_geometry_with_no_margin_baked_in(self):
        # The margin is the SOLVER's, applied once in
        # boundary_constraint_bounds. Subtracting it here as well would stop
        # the car a whole footprint early.
        track = wall(role='left', rho=1.0, theta=math.pi / 2.0)
        _nx, _ny, offset = wb.half_plane_from_track(track, (0.0, 0.0))
        assert offset == pytest.approx(1.0), (
            f'1 m away must read 1 m, not {offset} -- see BoundaryConstraint.msg')

    def test_the_span_and_the_activity_test_use_the_same_tangent(self):
        track = wall(role='left', p0=(-5.0, 1.0), p1=(5.0, 1.0))
        nx, ny, _offset = wb.half_plane_from_track(track, (0.0, 0.0))
        s_min, s_max = wb.segment_span(track.p0, track.p1, nx, ny)
        assert s_max - s_min == pytest.approx(10.0)
        assert wb.row_is_active(track.p0, track.p1, nx, ny, [(0.0, 0.0)], 0.0)
        assert not wb.row_is_active(track.p0, track.p1, nx, ny, [(9.0, 0.0)], 0.3)

    def test_clearance_is_measured_to_the_path_not_to_the_car(self):
        track = wall(role='front', rho=3.0, theta=0.0, p0=(3.0, -2.0), p1=(3.0, 2.0))
        nx, ny, offset = wb.half_plane_from_track(track, (0.0, 0.0))
        assert wb.clearance_to_path(nx, ny, offset, [(0.0, 0.0)]) == pytest.approx(3.0)
        assert wb.clearance_to_path(nx, ny, offset, [(2.5, 0.0)]) == pytest.approx(0.5)

    def test_an_empty_path_activates_nothing(self):
        track = wall()
        nx, ny, _o = wb.half_plane_from_track(track, (0.0, 0.0))
        assert not wb.row_is_active(track.p0, track.p1, nx, ny, [], 0.3)
        rows, _ = build([track], path=[])
        assert rows == []
