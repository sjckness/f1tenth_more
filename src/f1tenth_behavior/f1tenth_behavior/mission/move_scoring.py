"""Closed-loop per-move verification (3.2) and post-mission precision
scoring (3.5) -- two robustness items from this session's own follow-up
list, implemented together because they share one data source: the GLOBAL
EKF pose (/ekf_global/odometry/filtered) at each move's start and end.

"Global odom" here specifically means /ekf_global/odometry/filtered (the
map-frame EKF output, see runtime.py's own GLOBAL_XY_KEY comment) -- NOT
raw wheel odometry, which would defeat the point (it has no way to know
the car's heading actually changed after a turn), and NOT the LOCAL EKF
CheckStopCondition already uses to drive stop_condition evaluation, which
stays on that faster/lower-latency source for real-time decisions. This
distinction was confirmed against the task's own note before wiring it.

Scoring convention (decided explicitly with the user, not guessed):
  - A move's score is a ratio, not an error-subtracted-from-100 formula:
    actual / commanded * 100. This means both undershoot (<100%) and
    overshoot (>100%) read as "not 100%" while still showing clearly which
    one happened, per the task's own requirement.
  - Turn moves: UNSIGNED ratio (abs(actual_deg) / abs(commanded_deg) * 100)
    -- direction is heading_delta_deg's own concern, not the score's.
  - Straight (goal_distance) moves: SIGNED ratio, where "actual" is the
    real displacement PROJECTED onto the move's own intended heading
    (state.move_start_global_yaw, captured the same lazy way move_start_yaw
    already is -- see runtime.py). Projecting (rather than using raw
    straight-line distance travelled) means a move that drifted sideways or
    even went backward relative to its intended heading shows up as a low
    or negative score, not a falsely-good one.
  - go_to_object moves (schema 5.0): commanded = gap_m, actual = the FINAL
    GAP between the car's front and the object's near edge, i.e. mpc_corr's
    last /mpc/object_status r for this move plus gap_m (r is the centre
    distance minus gap + nose_reach + target radius) -- the same single range
    source object_reached stops on. Mismatch uses the distance policy below. The arrival bearing
    error (|alpha| at the end, degrees) is recorded as its own field and is
    deliberately NOT part of the mismatch: a car at the right range pointing
    37 degrees off is a different finding from one at the wrong range. The
    track-based range (global EKF pose to the handler's last target point) is
    recorded and logged beside it, as a cross-check on the controller's frame.
  - `goal_pose` moves are NOT scored this pass -- the task's own examples
    only ever covered turn/straight; inventing a formula for pose-mode
    arrival (e.g. remaining distance-to-target) wasn't asked for, so it's
    left unscored (None) rather than guessed. Still recorded (start/end
    pose, stop_reason) for the summary, just without a score_percent.
  - Mismatch tolerance (decided explicitly): flagged when the absolute
    error exceeds max(10% of the commanded magnitude, an absolute floor) --
    0.05m for distance, 3deg for angle. Below the floor, small commanded
    moves would otherwise get an unreasonably tight tolerance.
  - Aggregate mission score (decided explicitly): the mean of all SCORABLE
    per-move scores (goal_pose moves excluded, not counted as 0 or 100).
    None if a mission had no scorable moves at all.

Known gap, flagged not hidden: record_move_outcome() is called from
AdvanceMove (normal advance / mission complete), CheckStopCondition's own
timeout-abort branch, and HandleObjectAction's abort_mission/skip_to_move
branches -- every place a move ends INSIDE the BT. It is NOT called from
mission/loader.py's /mission/abort_mission service (an external,
operator-initiated abort) -- that path ends whatever move was in flight
without recording its (necessarily incomplete) outcome. Not wired this
pass; flagged as a follow-up rather than silently left uncovered.
"""

import json
import math
import os
import tempfile
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

# Tolerance policy for 3.2's mismatch flag, decided explicitly with the user:
# flag when the absolute error exceeds 10% of the commanded magnitude, with a
# floor so small commanded moves aren't held to an unreasonably tight bound.
MISMATCH_TOLERANCE_FRACTION = 0.10
MISMATCH_DISTANCE_FLOOR_M = 0.05
MISMATCH_ANGLE_FLOOR_DEG = 3.0


@dataclass
class MoveOutcome:
    move_id: str
    move_type: str  # 'turn' | 'goal_distance' | 'goal_pose' | 'drive' | 'go_to_object'
    stop_reason: str
    start_time: float  # time.monotonic()
    end_time: float
    start_global_xy: Optional[Tuple[float, float]]
    end_global_xy: Optional[Tuple[float, float]]
    start_global_yaw: Optional[float]
    end_global_yaw: Optional[float]
    commanded: Optional[float] = None  # degrees (turn) or meters (goal_distance); None for goal_pose
    actual: Optional[float] = None  # same units/sign convention as `commanded`
    score_percent: Optional[float] = None  # None if unscored (goal_pose, or missing pose data)
    mismatch_flagged: bool = False
    note: Optional[str] = None  # e.g. why unscored
    # go_to_object only (None otherwise). outcome is one of object_handler.
    # OUTCOMES. arrival_bearing_error_deg is NOT part of mismatch_flagged.
    outcome: Optional[str] = None
    arrival_bearing_error_deg: Optional[float] = None
    track_gap_m: Optional[float] = None
    target_track_id: Optional[str] = None
    wire_move_id: Optional[str] = None


def score_turn(commanded_deg: float, actual_deg: float) -> float:
    """Unsigned ratio, percent -- see module docstring. Degenerate
    commanded_deg==0.0 (legal per the schema, see condition_eval.py's own
    0deg test) scores 100% only if actual is also exactly 0; any nonzero
    actual against a 0 commanded is an infinite ratio, reported as such
    (caller decides how to display/flag that) rather than divided-by-zero
    crashing."""
    if commanded_deg == 0.0:
        return 100.0 if actual_deg == 0.0 else math.inf
    return abs(actual_deg) / abs(commanded_deg) * 100.0


def score_straight(
    commanded_distance: float,
    start_xy: Tuple[float, float],
    end_xy: Tuple[float, float],
    intended_heading_rad: float,
) -> Tuple[float, float]:
    """Returns (signed_actual_distance_m, score_percent) -- see module
    docstring for why the actual distance is the displacement PROJECTED
    onto intended_heading_rad, not raw straight-line distance travelled."""
    dx = end_xy[0] - start_xy[0]
    dy = end_xy[1] - start_xy[1]
    signed_actual = dx * math.cos(intended_heading_rad) + dy * math.sin(intended_heading_rad)
    if commanded_distance == 0.0:
        return signed_actual, (100.0 if signed_actual == 0.0 else math.inf)
    return signed_actual, signed_actual / commanded_distance * 100.0


def is_mismatch(commanded_magnitude: float, actual_magnitude: float, floor: float) -> bool:
    """10%-of-commanded-or-floor tolerance policy -- decided explicitly with
    the user (see module docstring), not guessed."""
    tolerance = max(MISMATCH_TOLERANCE_FRACTION * abs(commanded_magnitude), floor)
    return abs(actual_magnitude - commanded_magnitude) > tolerance


def build_move_outcome(
    move,
    stop_reason: str,
    start_time: float,
    end_time: float,
    start_global_xy: Optional[Tuple[float, float]],
    end_global_xy: Optional[Tuple[float, float]],
    start_global_yaw: Optional[float],
    end_global_yaw: Optional[float],
    global_turn_accum_deg: Optional[float],
    object_record=None,
) -> MoveOutcome:
    """Pure function: everything record_move_outcome() (the ROS-adjacent
    caller-facing entry point below) needs, minus the actual state mutation
    and logging -- kept separate so the scoring math itself is testable
    without constructing a MissionRuntimeState (see test/test_move_scoring.py).

    global_turn_accum_deg: the FINAL value of CheckStopCondition's own
    global-EKF turn accumulator for the move that just ended (see that
    module's "Turn accumulation" docstring paragraph and runtime.py's
    GLOBAL_TURN_ACCUM_KEY comment) -- deliberately NOT re-derived from
    start_global_yaw/end_global_yaw here, for the exact same reason
    orientation_delta itself was fixed to use an accumulator instead of a
    wrapped current-minus-start delta: a naive difference is bounded to
    (-180, 180] deg and would silently misscore any turn at or beyond that
    boundary.
    """
    if getattr(move, 'go_to_object', None) is not None:
        return _object_outcome(
            move, stop_reason, start_time, end_time, start_global_xy, end_global_xy,
            start_global_yaw, end_global_yaw, object_record)

    if move.turn is not None:
        move_type = 'turn'
        commanded = float(move.turn.heading_delta_deg)
        if global_turn_accum_deg is None:
            return MoveOutcome(
                move_id=move.id, move_type=move_type, stop_reason=stop_reason,
                start_time=start_time, end_time=end_time,
                start_global_xy=start_global_xy, end_global_xy=end_global_xy,
                start_global_yaw=start_global_yaw, end_global_yaw=end_global_yaw,
                commanded=commanded, note='no global-EKF yaw data for this move -- unscored',
            )
        actual = float(global_turn_accum_deg)
        score = score_turn(commanded, actual)
        mismatch = is_mismatch(abs(commanded), abs(actual), MISMATCH_ANGLE_FLOOR_DEG)
        return MoveOutcome(
            move_id=move.id, move_type=move_type, stop_reason=stop_reason,
            start_time=start_time, end_time=end_time,
            start_global_xy=start_global_xy, end_global_xy=end_global_xy,
            start_global_yaw=start_global_yaw, end_global_yaw=end_global_yaw,
            commanded=commanded, actual=actual, score_percent=score, mismatch_flagged=mismatch,
        )

    if move.goal_distance is not None:
        move_type = 'goal_distance'
        # THE commanded distance for a goal_distance move is NOT always
        # move.goal_distance -- that field is sometimes just a safety
        # ceiling, not the intended travel distance. Mirrors condition_eval.
        # evaluate()'s own distance_reached branch exactly:
        #   - distance_reached: the stop_condition's own `distance` override
        #     if present, else move.goal_distance (its implicit target) --
        #     e.g. a move with goal_distance=10.0 (ceiling) and
        #     stop_condition={type: distance_reached, distance: 0.5} is
        #     actually commanded to travel 0.5m, not 10.0m. Found live
        #     generating this pass's own sample summary -- see move_scoring.
        #     py's module docstring history.
        #   - goal_reached: mpc_corr's own /mpc/goal_reached fires exactly at
        #     move.goal_distance for a distance-mode goal, so that IS the
        #     commanded value here.
        #   - anything else (front_clearance, obstacle_distance_below,
        #     time_elapsed, object_seen/object_cleared, manual): the move
        #     stops on a sensor/time signal, not at a predetermined
        #     distance -- move.goal_distance is only ever a safety ceiling
        #     for these, never the intended target, so there is no
        #     well-defined "commanded distance" to score against. Left
        #     unscored (like goal_pose) rather than silently scored against
        #     a ceiling nobody expected the car to reach.
        sc_type = move.stop_condition.type
        if sc_type == 'distance_reached':
            commanded = float(move.stop_condition.params.get('distance', move.goal_distance))
        elif sc_type == 'goal_reached':
            commanded = float(move.goal_distance)
        else:
            return MoveOutcome(
                move_id=move.id, move_type=move_type, stop_reason=stop_reason,
                start_time=start_time, end_time=end_time,
                start_global_xy=start_global_xy, end_global_xy=end_global_xy,
                start_global_yaw=start_global_yaw, end_global_yaw=end_global_yaw,
                commanded=None,
                note=(
                    f'stop_condition.type={sc_type!r} stops on a sensor/time signal, not a '
                    'predetermined distance -- move.goal_distance is only a safety ceiling '
                    'here, not scored against'
                ),
            )
        if start_global_xy is None or end_global_xy is None or start_global_yaw is None:
            return MoveOutcome(
                move_id=move.id, move_type=move_type, stop_reason=stop_reason,
                start_time=start_time, end_time=end_time,
                start_global_xy=start_global_xy, end_global_xy=end_global_xy,
                start_global_yaw=start_global_yaw, end_global_yaw=end_global_yaw,
                commanded=commanded, note='no global-EKF pose data for this move -- unscored',
            )
        actual, score = score_straight(commanded, start_global_xy, end_global_xy, start_global_yaw)
        mismatch = is_mismatch(commanded, actual, MISMATCH_DISTANCE_FLOOR_M)
        return MoveOutcome(
            move_id=move.id, move_type=move_type, stop_reason=stop_reason,
            start_time=start_time, end_time=end_time,
            start_global_xy=start_global_xy, end_global_xy=end_global_xy,
            start_global_yaw=start_global_yaw, end_global_yaw=end_global_yaw,
            commanded=commanded, actual=actual, score_percent=score, mismatch_flagged=mismatch,
        )

    # goal_pose or drive (mission_config.py guarantees exactly one goal shape)
    # -- neither is scored this pass, see module docstring. Drive moves used to
    # fall through here labelled 'goal_pose'; only the label is fixed.
    move_type = 'drive' if getattr(move, 'drive', None) is not None else 'goal_pose'
    return MoveOutcome(
        move_id=move.id, move_type=move_type, stop_reason=stop_reason,
        start_time=start_time, end_time=end_time,
        start_global_xy=start_global_xy, end_global_xy=end_global_xy,
        start_global_yaw=start_global_yaw, end_global_yaw=end_global_yaw,
        note=f'{move_type} moves are not scored this pass -- see module docstring',
    )


def _object_outcome(move, stop_reason, start_time, end_time, start_global_xy,
                    end_global_xy, start_global_yaw, end_global_yaw, record):
    """go_to_object scoring -- see the module docstring."""
    spec = move.go_to_object
    commanded = float(spec.gap_m)
    common = dict(
        move_id=move.id, move_type='go_to_object', stop_reason=stop_reason,
        start_time=start_time, end_time=end_time,
        start_global_xy=start_global_xy, end_global_xy=end_global_xy,
        start_global_yaw=start_global_yaw, end_global_yaw=end_global_yaw,
        commanded=commanded,
    )
    outcome = record.outcome if record is not None else None
    if outcome is None and stop_reason.startswith('timeout'):
        outcome = 'timeout'
    track_range = None
    track_id = None
    wire_id = record.wire_move_id if record is not None else None
    if record is not None:
        track_id = record.track_id
        if (record.target_xy is not None and end_global_xy is not None
                and record.target_radius is not None):
            # Track-based GAP, same geometry as the commanded one, from the
            # global EKF pose rather than mpc_corr's odom frame.
            track_range = (math.hypot(record.target_xy[0] - end_global_xy[0],
                                      record.target_xy[1] - end_global_xy[1])
                           - spec.nose_reach_m - record.target_radius)
    status = record.last_status if record is not None else None
    if status is None:
        return MoveOutcome(
            **common, outcome=outcome, track_gap_m=track_range, target_track_id=track_id,
            wire_move_id=wire_id, note='no /mpc/object_status for this move -- unscored')
    actual = float(status.r) + commanded
    return MoveOutcome(
        **common, actual=actual, score_percent=actual / commanded * 100.0,
        mismatch_flagged=is_mismatch(commanded, actual, MISMATCH_DISTANCE_FLOOR_M),
        outcome=outcome, arrival_bearing_error_deg=math.degrees(abs(float(status.alpha))),
        track_gap_m=track_range, target_track_id=track_id, wire_move_id=wire_id)


def record_move_outcome(state, logger, move, stop_reason: str, now: float,
                         end_global_xy, end_global_yaw, global_turn_accum_deg) -> MoveOutcome:
    """ROS-adjacent entry point: builds this move's MoveOutcome (see
    build_move_outcome() above), appends it to state.move_outcomes, and logs
    a warning if it's mismatch_flagged or was left unscored. `move` is the
    move that just ended (state.current_move BEFORE any transition/reset --
    every call site must read it before calling goto_move()/complete()/
    abort()). `state` is the MissionRuntimeState (for move_start_time/
    move_start_global_xy/move_start_global_yaw -- the move's own start
    reference -- and move_outcomes to append to)."""
    outcome = build_move_outcome(
        move=move,
        stop_reason=stop_reason,
        start_time=state.move_start_time,
        end_time=now,
        start_global_xy=state.move_start_global_xy,
        end_global_xy=end_global_xy,
        start_global_yaw=state.move_start_global_yaw,
        end_global_yaw=end_global_yaw,
        global_turn_accum_deg=global_turn_accum_deg,
        object_record=getattr(state, 'object_record', None),
    )
    state.move_outcomes.append(outcome)

    if outcome.move_type == 'go_to_object':
        bearing = outcome.arrival_bearing_error_deg
        track = outcome.track_gap_m
        logger.info(
            f"[mission] Move '{move.id}' (go_to_object) outcome={outcome.outcome!r}: "
            f'final gap (mpc_corr r + gap_m)='
            + (f'{outcome.actual:.3f}' if outcome.actual is not None else 'n/a')
            + f' commanded gap={outcome.commanded:.3f} track-based gap='
            + (f'{track:.3f}' if track is not None else 'n/a')
            + ' arrival bearing error='
            + (f'{bearing:.1f} deg' if bearing is not None else 'n/a')
            + f' track={outcome.target_track_id!r}'
        )

    if outcome.score_percent is None:
        logger.info(f"[mission] Move '{move.id}' outcome: {outcome.note}")
    else:
        logger.info(
            f"[mission] Move '{move.id}' ({outcome.move_type}) outcome: "
            f'commanded={outcome.commanded:.3f} actual={outcome.actual:.3f} '
            f'score={outcome.score_percent:.1f}% stop_reason={stop_reason!r}'
        )
        if outcome.mismatch_flagged:
            logger.warn(
                f"[mission] Move '{move.id}': commanded-vs-actual mismatch beyond "
                f'tolerance (score={outcome.score_percent:.1f}%) -- commanded='
                f'{outcome.commanded:.3f} actual={outcome.actual:.3f}.'
            )
    return outcome


def aggregate_score(outcomes: List[MoveOutcome]) -> Optional[float]:
    """Mean of per-move scorable scores (decided explicitly with the user)
    -- None (not 0 or 100) if the mission had no scorable move at all."""
    scored = [
        o.score_percent for o in outcomes
        if o.score_percent is not None and math.isfinite(o.score_percent)
    ]
    if not scored:
        return None
    return sum(scored) / len(scored)


def _resolve_mission_reports_dir() -> Path:
    """Resolve <ws_root>/src/f1tenth_behavior/mission_reports/, mirroring
    mpc_controller's MPC_corr.py._resolve_debug_output_path() technique
    exactly (reimplemented standalone rather than imported, to avoid an
    unwanted cross-package dependency between f1tenth_behavior and
    mpc_controller for one path computation) -- see that function's own
    docstring for the full colcon install/symlink-install reasoning this
    walks around."""
    this_file = Path(__file__).resolve()
    for parent in this_file.parents:
        if parent.name == 'install':
            return parent.parent / 'src' / 'f1tenth_behavior' / 'mission_reports'
    # No 'install' ancestor -- already running from source. parents[0]=mission
    # (this file's dir), parents[1]=f1tenth_behavior (the python package),
    # parents[2]=f1tenth_behavior (the top-level dir, sibling of missions/).
    return this_file.parents[2] / 'mission_reports'


def _write_atomic(path: Path, text: str) -> None:
    """Write `text` to `path` so the path never holds a partial file.

    THE FAILURE THIS REPLACES. Path.write_text opens with mode 'w', which
    truncates the destination before any content is written, so an
    interruption anywhere between the open and the close leaves a 0-byte file
    AT THE FINAL PATH. The mission report for the third go_to_person run of
    2026-09-15 is exactly that: 0 bytes, at the name a complete report would
    have had. Reproduced directly -- killing a process between the open and
    the flush leaves the destination empty.

    Why that is worth fixing rather than noting: a mission report is read back
    as evidence about a run, and a truncated one is not distinguishable from a
    run that produced nothing. Every consumer would have to learn to tell a
    0-byte report from a real one, which is a rule nobody remembers.

    The temp file is created in the DESTINATION DIRECTORY, not in /tmp:
    os.replace is only atomic within a filesystem, and mission_reports/ lives
    in the source tree while /tmp may well be a different mount. delete=False
    plus an explicit replace, because the file must outlive the handle.

    flush + fsync before the replace: os.replace makes the RENAME atomic
    against other readers, but it does not promise the CONTENT reached the
    disk first. Without the fsync a power loss can leave the new name pointing
    at unwritten blocks -- a 0-byte file again, by a slower route.

    Failures are left to the caller, which already catches OSError and logs;
    a partial temp file is cleaned up here so a failed write cannot silently
    litter the reports directory with .tmp entries.
    """
    fd, tmp_name = tempfile.mkstemp(
        dir=str(path.parent), prefix=f'.{path.name}.', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_name, str(path))
    except BaseException:
        # BaseException, not Exception: a KeyboardInterrupt/SystemExit landing
        # mid-write is precisely the interruption this function exists to
        # survive, and it must not leave the temp file behind either.
        try:
            os.unlink(tmp_name)
        except OSError:
            pass
        raise


def write_mission_summary(
    mission_id: str, outcomes: List[MoveOutcome], final_state: str,
    mission_start_wall_time: Optional[float], logger,
) -> Optional[Path]:
    """Writes <mission_reports>/<mission_id>_<timestamp>.json (3.5) -- one
    file per mission run, timestamped so runs are comparable over time (see
    3.3's own regression-set framing: this is what makes it valuable to
    re-run against). Returns the path written, or None if writing failed
    (logged, never raised -- a summary-writing bug must never crash the
    mission-ending tick that triggers it).

    Written atomically (see _write_atomic): the destination either does not
    exist or holds a complete report, never a truncated one."""
    ts = mission_start_wall_time if mission_start_wall_time is not None else time.time()
    stamp = time.strftime('%Y%m%dT%H%M%S', time.localtime(ts))
    agg = aggregate_score(outcomes)
    payload = {
        'mission_id': mission_id,
        'final_state': final_state,
        'timestamp': stamp,
        'aggregate_score_percent': agg,
        'aggregate_score_method': 'mean_of_scorable_moves',
        'moves': [asdict(o) for o in outcomes],
    }
    try:
        out_dir = _resolve_mission_reports_dir()
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / f'{mission_id}_{stamp}.json'
        _write_atomic(path, json.dumps(payload, indent=2))
        logger.info(
            f"[mission] '{mission_id}' summary written to {path} "
            f'(aggregate score={agg!r}).'
        )
        return path
    except OSError as exc:
        logger.error(f"[mission] '{mission_id}' summary write FAILED: {exc}")
        return None
