"""go_to_person end to end, without hardware: tracks -> handler -> MPC -> reached -> hold.

Every piece in the chain is the shipped code; only the transport and the plant
are stand-ins:

  fake /costmap/semantic_tracks  a map-frame person, stamped 0.2 s in the past
  GoToObject                     the real behaviour (f1tenth_behavior), ticked
                                 at 10 Hz, publishing real ObjectGoal messages
  MPCController                  the real goal_object_callback, the real
                                 _refresh_object_target (through a non-identity
                                 map -> odom edge), build_straight_corridor,
                                 compute_local_target, _assess_object_tick and
                                 _publish_object_status (a real
                                 ObjectApproachStatus), solve_mpc_step
  plant                          f1tenth_state_fcn_dt_beta, the nominal model
  CheckStopCondition             the real behaviour, judging object_reached
                                 from the real status
  hold                           the real hold_callback, which ends object mode

It is a nominal-model study with a perfect detector, like the closed-loop rig
it borrows from; it proves the wiring and the frames, not the car.

Run standalone: python3 -m pytest test/test_go_to_object_end_to_end.py -v
"""

import collections
import math
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

py_trees = pytest.importorskip('py_trees')

from f1tenth_behavior.behaviours.check_stop_condition import (  # noqa: E402
    CheckStopCondition,
)
from f1tenth_behavior.behaviours.go_to_object import GoToObject  # noqa: E402
from f1tenth_behavior.mission.detected_classes_bridge import (  # noqa: E402
    DETECTED_CLASSES_KEY,
)
from f1tenth_behavior.mission.mission_config import load_mission_file  # noqa: E402
from f1tenth_behavior.mission.object_handler import object_move_wire_id  # noqa: E402
from f1tenth_behavior.mission.runtime import (  # noqa: E402
    FRONT_CLEARANCE_KEY,
    GLOBAL_TURN_ACCUM_KEY,
    GLOBAL_XY_KEY,
    GLOBAL_YAW_KEY,
    MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
    MIN_OBSTACLE_DISTANCE_KEY,
    MISSION_KEY,
    OBJECT_STATUS_KEY,
    MissionRuntimeState,
)

from mpc_controller.MPC_corr import MPCController, _pose_odom_to_map  # noqa: E402
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step  # noqa: E402
from mpc_controller.object_approach import (  # noqa: E402
    SPEED_DRIVE, floor_moving_speed, object_speed_decision)
from mpc_controller.object_guard import RefreshWatchdog, SteeringRamp  # noqa: E402
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta  # noqa: E402

import test_object_approach_closed_loop as rig  # noqa: E402

BEHAVIOR_MISSIONS = (Path(__file__).resolve().parents[3]
                     / 'f1tenth_behavior' / 'missions')

# map -> odom edge (x, y, yaw): deliberately not identity, so a frame mix-up
# anywhere in the chain moves the car to the wrong place.
MAP_ODOM = (0.30, -0.20, 0.05)
PERSON_MAP = (4.0, 0.8)
PERSON_RADIUS = rig.PERSON_RADIUS['footprint']
PERSON_WIDTH = 2.0 * PERSON_RADIUS
TS = rig.TS


class _SimClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def now(self):
        return SimpleNamespace(nanoseconds=int(round(self.t * 1e9)),
                               to_msg=lambda: _time_msg(self.t))


def _time_msg(t):
    from builtin_interfaces.msg import Time
    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, *_a, **_k):
        self.lines.append(str(msg))

    warn = warning = debug = error = info


class _Pub:
    def __init__(self, sink=None):
        self.msgs = []
        self.sink = sink

    def publish(self, msg):
        self.msgs.append(msg)
        if self.sink is not None:
            self.sink(msg)


class _NodeMPC(rig._ObjectMPC):
    """The closed-loop rig's stand-in, plus what the callbacks and status need."""

    def __init__(self, clock, standoff):
        super().__init__(standoff=standoff)
        self.clock = clock
        self.logger = _Logger()
        self.x = self.y = self.yaw = 0.0
        self.odom_frame = 'odom'
        self.map_odom_max_age_sec = 0.5
        self.goal_object_move_id = None
        self.goal_object_target_class = ''
        self.goal_object_map_xy = None
        self.goal_object_stamp = None
        self.goal_distance = None
        self.goal_start_xy = None
        self.goal_anchor_map = None
        self.goal_reached = False
        self.goal_pose_yaw = None
        self.pose_goal_reached = False
        self._no_goal_warned = False
        self.wall_turn_committed = False
        self.wall_turn_commanded_rot = None
        self.cached_corridor = None
        self.last_corridor_time = None
        self.last_corridor_stamp = None
        self.cached_pref_nom = None
        self.warm_start_z = None
        self.object_ended_move_ids = collections.deque(maxlen=64)
        self.object_exit_ramp = SteeringRamp(0.6)
        self.object_exit_ramping = False
        self._object_exit_last_sec = None
        self.object_goal_watchdog = RefreshWatchdog(0.5)
        self.object_goal_watchdog_tripped = False
        self._last_published_steer = 0.0
        self.delta_real = None
        self.hold = False
        self.drive = []
        self.status_pub = None

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return self.clock

    def _clear_drive_state(self):
        self.drive_cmd = None

    def _lookup_map_odom(self, max_age_sec=None):
        return MAP_ODOM

    def _publish_drive(self, speed, steering):
        self._last_published_steer = float(steering)
        self.drive.append((float(speed), float(steering)))

    def __getattr__(self, name):
        # Bind every other MPCController method this chain calls to this fake.
        method = getattr(MPCController, name, None)
        if callable(method):
            return method.__get__(self, _NodeMPC)
        raise AttributeError(name)


def _tracks_msg(clock):
    stamp = clock.t - 0.2
    sec = int(math.floor(stamp))
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='map', stamp=SimpleNamespace(
            sec=sec, nanosec=int(round((stamp - sec) * 1e9)))),
        detections=[SimpleNamespace(
            id='7',
            results=[SimpleNamespace(hypothesis=SimpleNamespace(class_id='person', score=0.9))],
            bbox=SimpleNamespace(
                center=SimpleNamespace(position=SimpleNamespace(x=PERSON_MAP[0], y=PERSON_MAP[1])),
                size=SimpleNamespace(x=PERSON_WIDTH, y=PERSON_WIDTH, z=0.0)))])


def _behaviours(clock, state):
    bb = SimpleNamespace()
    setattr(bb, MISSION_KEY, state)
    for key in (GLOBAL_XY_KEY, GLOBAL_YAW_KEY, GLOBAL_TURN_ACCUM_KEY, OBJECT_STATUS_KEY,
                MIN_OBSTACLE_DISTANCE_KEY, MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
                FRONT_CLEARANCE_KEY):
        setattr(bb, key, None)
    setattr(bb, DETECTED_CLASSES_KEY, {})
    logger = _Logger()
    node = SimpleNamespace(get_logger=lambda: logger)

    goto = GoToObject(clock=clock)
    goto.node, goto.blackboard = node, bb
    goto.goal_pub, goto.end_pub, goto.hold_pub = _Pub(), _Pub(), _Pub()

    check = CheckStopCondition()
    check.node, check.blackboard = node, bb
    check.hold_pub, check.object_end_pub = _Pub(), _Pub()
    return goto, check, bb


def _run_go_to_person(duration=40.0, mission_path=None,
                      speed_limits=rig.SHIPPED_SPEED_LIMITS):
    """Run a one-move person mission; go_to_person.json unless `mission_path` is given.

    The MPC tick is MPC_corr's object branch, as the closed-loop rig's
    run_approach applies it: live r into the stop latch, object_speed_decision,
    stop-and-wait (zero speed, steering held, no solve) on anything but DRIVE,
    and on DRIVE a solve whose published command is floored at
    min_moving_speed_mps. A zero command is braked through the rig's measured
    braking model. `speed_limits` (max_forward, max_reverse) is MPC_corr's
    /drive clamp, the shipped pair by default; None leaves it off.
    """
    clock = _SimClock()
    config = load_mission_file(str(mission_path or BEHAVIOR_MISSIONS / 'go_to_person.json'))
    spec = config.moves[0].go_to_object
    state = MissionRuntimeState()
    goto, check, bb = _behaviours(clock, state)
    centre_standoff = spec.gap_m + spec.nose_reach_m + PERSON_RADIUS
    mpc = _NodeMPC(clock, centre_standoff)
    mpc.object_status_pub = _Pub(sink=check._object_status_cb)
    goto.goal_pub.sink = mpc.goal_object_callback

    state.load(config, time.monotonic())
    state.begin(time.monotonic())
    wire = object_move_wire_id(config.mission_id, state.run_generation, config.moves[0].id)

    x = np.array([0.0, 0.0, 0.0, 0.0])
    last_u = np.zeros(2)
    warm = None
    corridor = None
    last_build = None
    person_odom = None
    min_centre = math.inf
    reached_at = None
    speeds = []
    brake_t, brake_v0 = None, 0.0
    for tick in range(int(duration / TS)):
        clock.t += TS
        mpc.x, mpc.y, mpc.yaw, mpc.v = float(x[0]), float(x[1]), float(x[2]), float(x[3])
        gx, gy, gyaw = _pose_odom_to_map(x[0], x[1], x[2], *MAP_ODOM)
        setattr(bb, GLOBAL_XY_KEY, (gx, gy))
        setattr(bb, GLOBAL_YAW_KEY, gyaw)
        setattr(bb, GLOBAL_TURN_ACCUM_KEY, 0.0)
        check.global_x, check.global_y, check.global_yaw = gx, gy, gyaw
        check.x, check.y, check.yaw = float(x[0]), float(x[1]), float(x[2])

        # perception -> behaviour tree (10 Hz, like the tree)
        goto._tracks_cb(_tracks_msg(clock))
        assert goto.update() == py_trees.common.Status.SUCCESS
        if check.update() == py_trees.common.Status.SUCCESS:
            reached_at = tick
            break

        # MPC control tick
        if mpc.goal_object_map_xy is None:
            continue
        mpc._refresh_object_target()
        if mpc.goal_object_odom_xy is None:
            continue
        person_odom = mpc.goal_object_odom_xy
        need = (corridor is None or clock.t - last_build >= 1.0
                or mpc._object_target_moved_since_build())
        if need:
            corridor = MPCController.build_straight_corridor(mpc, x)
            last_build = clock.t
        mpc.object_stop_latch.update(mpc._object_range())
        decision = object_speed_decision(
            mpc.goal_object_speed, mpc.object_stop_latch.latched, mpc.min_moving_speed)
        mpc.object_speed_mode = decision.mode
        flags = mpc._assess_object_tick(x, clock.t)
        mpc._publish_object_status(mpc.object_last_step, flags, decision.speed_ref)

        if decision.mode != SPEED_DRIVE:
            # stop-and-wait: zero speed, steering held, no solve
            mpc._publish_drive(0.0, mpc._last_published_steer)
            if brake_t is None:
                brake_t, brake_v0 = 0.0, float(x[3])
            s = (rig._braking_position(brake_t + TS, brake_v0, rig.BRAKING_MEASURED)
                 - rig._braking_position(brake_t, brake_v0, rig.BRAKING_MEASURED))
            x = rig._advance_by_distance(x, mpc._last_published_steer, s)
            brake_t += TS
            x[3] = rig._braking_speed(brake_t, brake_v0, rig.BRAKING_MEASURED)
        else:
            brake_t = None
            obstacles = [(person_odom[0], person_odom[1], PERSON_RADIUS)]
            corridor['obstacles_world'] = obstacles
            corridor['d_safe'] = rig.DMIN
            corridor['car_radius'] = rig.CAR_RADIUS
            corridor['avoidance_margin'] = rig.AVOIDANCE_MARGIN
            pref_nom = MPCController.compute_local_target(mpc, x, corridor)
            u0, info = solve_mpc_step(
                x0=x, last_u=last_u, pref_nom=pref_nom, corridor=corridor,
                horizon=rig.HORIZON, ts=TS, params=rig.PARAMS, limits=rig.LIMITS,
                weights=dict(rig.WEIGHTS), obstacles=obstacles, dmin=rig.DMIN,
                vdes=decision.speed_ref, solver='rti', warm_start_z=warm)
            warm = shift_warm_start(info.get('zopt'), rig.HORIZON) if info else None
            v_pub = floor_moving_speed(x[3] + float(u0[1]) * TS, mpc.min_moving_speed)
            if speed_limits is not None:
                v_pub, _ = rig.clamp_drive_speed(v_pub, *speed_limits)
            mpc._publish_drive(v_pub, float(u0[0]))
            u_plant = np.array([float(u0[0]), (v_pub - x[3]) / TS])
            x = np.array(f1tenth_state_fcn_dt_beta(x, u_plant, TS, rig.WHEELBASE, rig.LR),
                         dtype=float)
            last_u = np.asarray(u0, dtype=float)
        speeds.append(float(x[3]))
        min_centre = min(min_centre, math.hypot(person_odom[0] - x[0], person_odom[1] - x[1]))

    return SimpleNamespace(clock=clock, state=state, goto=goto, check=check, mpc=mpc,
                           wire=wire, x=x, reached_at=reached_at, spec=spec,
                           centre_standoff=centre_standoff, speeds=speeds,
                           person_odom=person_odom, min_centre=min_centre)


@pytest.fixture(scope='module')
def run():
    return _run_go_to_person()


class TestGoToPersonEndToEnd:

    def test_the_mission_reaches_the_person(self, run):
        assert run.reached_at is not None, 'object_reached never fired'
        assert run.state.object_record.outcome == 'reached'

    def test_it_rests_in_the_stop_window_past_the_commanded_gap(self, run):
        """Gap = centre distance - nose_reach - the person's footprint radius.

        The stop latches at object_reach_tol_m + object_stop_distance_m past the
        commanded gap and the car brakes in from there, so it rests a little
        OUTSIDE the commanded gap -- never inside it.
        """
        d = math.hypot(run.person_odom[0] - run.x[0], run.person_odom[1] - run.x[1])
        gap = d - run.spec.nose_reach_m - PERSON_RADIUS
        lo, hi = rig.stop_window()
        assert run.spec.gap_m + lo <= gap <= run.spec.gap_m + hi

    def test_the_goal_carried_the_centre_distance_for_the_default_gap(self, run):
        assert run.spec.gap_m == pytest.approx(0.5)
        assert run.goto.goal_pub.msgs[-1].standoff == pytest.approx(run.centre_standoff)

    def test_the_person_went_through_the_map_to_odom_edge(self, run):
        """The MPC's target is the map point reprojected, not the raw numbers."""
        assert run.person_odom != pytest.approx(PERSON_MAP, abs=0.05)

    def test_one_move_id_for_the_whole_move(self, run):
        assert {m.move_id for m in run.goto.goal_pub.msgs} == {run.wire}
        assert len(run.goto.goal_pub.msgs) > 20

    def test_the_status_it_stopped_on_is_this_moves_and_fresh(self, run):
        status = run.state.object_record.last_status
        assert status.move_id == run.wire
        # object_reached's own two ways to fire: r within tolerance, or the
        # stop latched with the car at rest.
        assert status.r <= rig.REACH_TOL_M or (status.stop_latched and status.speed <= 0.05)
        assert status.target_age_s == pytest.approx(0.2, abs=1e-6)
        assert status.target_behind_terminal is False

    def test_the_car_never_came_closer_than_the_commanded_gap(self, run):
        """The latched stop rests outside the commanded gap over the whole run."""
        assert run.min_centre >= run.centre_standoff

    def test_it_never_drove_below_the_floor(self, run):
        """Every /drive speed the chain published is zero or at least the floor."""
        assert all(v == 0.0 or v >= rig.MIN_MOVING_SPEED - 1e-9 for v, _ in run.mpc.drive)
        assert any(v > 0.0 for v, _ in run.mpc.drive)

    def test_hold_then_ends_object_mode_and_ramps_the_steering_out(self, run):
        """What AdvanceMove's completion publishes, delivered to the real callback."""
        mpc = run.mpc
        steer_before = mpc._last_published_steer
        mpc.hold_callback(SimpleNamespace(data=True))
        assert mpc.object_psi_c is None
        assert run.wire in mpc.object_ended_move_ids
        for _ in range(8):
            mpc._publish_stop()
            run.clock.t += TS
        tail = [steer_before] + [s for _, s in mpc.drive[-8:]]
        assert all(v == 0.0 for v, _ in mpc.drive[-8:])
        steps = [abs(b - a) for a, b in zip(tail, tail[1:])]
        assert max(steps) <= 0.06 + 1e-9
        assert tail[-1] == 0.0

    def test_a_late_objectgoal_for_the_ended_move_is_ignored(self, run):
        mpc = run.mpc
        if mpc.object_psi_c is not None:
            mpc.hold_callback(SimpleNamespace(data=True))
        late = run.goto.goal_pub.msgs[-1]
        mpc.goal_object_callback(late)
        assert mpc.object_psi_c is None
        assert mpc.goal_object_odom_xy is None
