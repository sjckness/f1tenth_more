"""
CorridorManager: mpc_corr's per-move corridor state, without the node.

It owns four things, and nothing else:

  * the FROZEN straight-move anchor -- captured in the map frame at move
    start, reprojected into the live odom frame once per control tick, held
    at its last good value whenever map -> odom is unavailable or stale;
  * the CACHED corridor and the rebuild period that governs it;
  * the two pieces of rebuild-to-rebuild memory the corridor build carries:
    the object arc's hysteresis latch and the previous reference (for
    refStep);
  * the cached lookahead target the debug snapshot records.

The map -> odom lookup is INJECTED: refresh_anchor and capture_anchor take a
callable with the node's _lookup_map_odom(max_age_sec=None) signature, which
returns (x, y, yaw) or None. The age guard itself -- how old a transform is,
judged against the node's clock -- stays where the transform and the clock
are; this class decides WHEN it applies (refresh: map_odom_max_age_sec;
capture: no guard) and what happens when it trips (hold, never revert).

The control loop calls it in two steps that match its own order:
refresh_anchor() before the goal_distance termination check, which reads the
reprojected anchor, and maybe_rebuild() after it.
"""

from typing import Any, Callable, Optional, Tuple

from mpc_controller.corridor.geometry import pose_map_to_odom, pose_odom_to_map

Pose = Tuple[float, float, float]


class CorridorManager:
    """Per-move corridor state: anchor, cached corridor, rebuild gate."""

    def __init__(self, update_period: float, map_odom_max_age_sec: float, logger):
        # Rebuild period [s], the node's corridor_update_period parameter --
        # a PERIOD, bigger is slower. See that parameter's own note in
        # MPC_corr's __init__ for why 1.0 is what ships.
        self.update_period = float(update_period)
        # Staleness ceiling applied to the per-tick reprojection; see
        # MPC_corr._lookup_map_odom's AGE GUARD note.
        self.map_odom_max_age_sec = float(map_odom_max_age_sec)
        self._logger = logger

        # ---- Frozen straight-move anchor, map-frame (drift-correction pass) --
        # goal_start_xy/psi_init_corridor freeze the move's reference line
        # in ODOM coordinates, which is the frame this stack's dual-EKF split
        # deliberately keeps free of absolute-heading error -- see
        # geometry.pose_odom_to_map's own docstring for the measurement and
        # the full reason that made the frozen line unable to present a
        # heading error to the solver at all.
        #
        # anchor_map: (x, y, psi) of the move's start pose in the MAP frame,
        #   captured once in capture_anchor and never touched again for
        #   that move. None means "no map-frame anchor for this move" --
        #   either no move is active, or the map -> odom lookup failed at
        #   move start, in which case everything below degrades to the
        #   pre-existing odom-frame-only behaviour rather than blocking
        #   driving.
        # anchor_odom: that same anchor reprojected into the CURRENT odom
        #   frame, refreshed once per control_loop tick by refresh_anchor().
        #   Read by two consumers that take DIFFERENT parts of it and must not
        #   be conflated: control_loop's progress check uses the whole pose
        #   (x, y AND psi) as the move's frozen origin and direction, while
        #   the corridor build's goal_distance branch takes ONLY the psi --
        #   its own origin tracks the car (see that branch's two-anchor
        #   note). Seeded at move start with the raw odom anchor, which is
        #   what the reprojection evaluates to at that instant anyway, so it
        #   is never None while a move is active and there is always a
        #   last-known-good value to hold.
        self.anchor_map: Optional[Pose] = None
        self.anchor_odom: Optional[Pose] = None

        # The cached corridor and the float second-count the period gate
        # compares against.
        self.cached_corridor: Optional[dict] = None
        self.last_corridor_time: Optional[float] = None
        # Real computation time of the cached corridor (rclpy Time, not a
        # float) -- used ONLY to stamp /mpc/corridor_markers headers with
        # when the corridor was actually computed, not whatever instant the
        # marker happens to be published at. Kept separate from
        # last_corridor_time (a float second-count used purely for the
        # update_period gate) since the two need different representations.
        self.last_corridor_stamp: Any = None
        self.cached_pref_nom = None

        # ---- REFERENCE STEP --------------------------------------------
        # The previous corridor, kept only to measure how far the reference
        # MOVES between rebuilds. At 1 Hz with the car at ~0.45 m/s the
        # centreline is re-laid from a pose half a metre further on every
        # second, so the solver is handed a visibly different reference each
        # time; that step is a wobble candidate in its own right and nothing
        # was measuring it. Three numbers, because they fail differently:
        # the centreline can shift while the tangent at the car holds (pure
        # lateral re-anchoring), the tangent can swing while the centreline
        # holds (a heading rethink), and Pend can jump on its own when the
        # target estimate moves.
        self.prev_corridor_ref = None
        # Which side of the object arc's switch band the last rebuild landed
        # on. False until the first object rebuild; reset with the rest of
        # the corridor state.
        self.object_arc_active = False

    # ---- anchor ---------------------------------------------------------
    def capture_anchor(self, x: float, y: float, yaw: float,
                       lookup: Callable[..., Optional[Pose]]) -> Optional[Pose]:
        """
        Freeze a new straight move's anchor; return the edge used, or None.

        The odom anchor is seeded with the raw pose either way: with a map
        anchor it is simply this instant's reprojection (an identity right
        now, diverging as map -> odom is corrected); without one it stays
        that raw value for the whole move, which is exactly the pre-existing
        behaviour. NO age guard here: the lookup is called without
        max_age_sec, so any edge tf2 still holds is used for the capture.
        """
        self.anchor_odom = (x, y, float(yaw))
        self.anchor_map = None
        tf_map_odom = lookup()
        if tf_map_odom is not None:
            self.anchor_map = pose_odom_to_map(x, y, float(yaw), *tf_map_odom)
        return tf_map_odom

    def clear_anchor(self):
        """Drop the anchor: the move that owned it has been superseded."""
        self.anchor_map = None
        self.anchor_odom = None

    def refresh_anchor(self, lookup: Callable[..., Optional[Pose]]):
        """
        Recompute anchor_odom -- the frozen anchor in THIS tick's odom frame.

        Called once per control_loop tick, BEFORE both consumers (the
        goal_distance progress check and the corridor build's goal_distance
        branch), so that one lookup per tick serves both instead of each
        doing its own at its own instant -- the corridor is rebuilt on its
        own slower cadence (update_period, 1.0 s by default) than the
        progress check runs (every tick), so two independent lookups would
        disagree by however much map -> odom moved in between. The two
        consumers take different parts of the result (see anchor_odom's own
        comment), but the CORRECTION they apply is the same one, which is
        the point.

        FLICKER NOTE: this runs at the control-loop rate, so the correction
        it tracks is continuous at 10 Hz; the corridor only samples it when
        it rebuilds, so a discrete SLAM correction landing between rebuilds
        still reaches the corridor as one step of up to update_period's
        worth of accumulated change. Since the corridor's origin stopped
        depending on this value that step is now a pure rotation about the
        car rather than a translation plus a rotation -- see the corridor
        build's own comment.

        No map anchor (never captured, or the feature is off) -> nothing to
        do: anchor_odom keeps the raw odom-frame value capture_anchor seeded
        it with, i.e. the pre-existing frozen-in-odom behaviour.

        Lookup failure OR a transform older than map_odom_max_age_sec (the
        case that actually happens -- see _lookup_map_odom's AGE GUARD: the
        lookup itself does not fail during a delivery stall, it silently
        returns the last transform tf2 holds) with a map anchor present ->
        HOLD THE LAST KNOWN GOOD reprojection, explicitly, rather than
        falling back to the raw odom anchor: that raw value is precisely the
        thing being corrected for, so reverting to it for one tick would
        silently reintroduce the bug (and, worse, jump the reference line
        mid-move). Holding the last good value instead is the same shape as
        costmap_boundary_node's own periodic-publish pass, which likewise
        keeps recomputing from its cached grid/pose of whatever age and only
        fails safe when the input has NEVER arrived -- and that "never" case
        cannot occur here, since capture seeds a value.
        """
        if self.anchor_map is None:
            return
        tf_map_odom = lookup(max_age_sec=self.map_odom_max_age_sec)
        if tf_map_odom is None:
            self._logger.warn(
                'ANCHOR/hold | map -> odom unavailable OR STALE this tick: '
                'holding last reprojected anchor (NOT reverting to the raw '
                'odom anchor)', throttle_duration_sec=5.0)
            return
        self.anchor_odom = pose_map_to_odom(*self.anchor_map, *tf_map_odom)

    # ---- cached corridor --------------------------------------------------
    def invalidate(self):
        """
        Drop the cached corridor and its rebuild-to-rebuild memory.

        Forces a rebuild (and a marker publish) on the next tick. A new
        move's first corridor has nothing meaningful to be a step FROM --
        differencing it against the last move's would report the whole
        re-anchoring as reference movement. Same reason the arc's hysteresis
        latch starts cold. The anchor is NOT touched: a goal callback clears
        or re-captures it itself.
        """
        self.cached_corridor = None
        self.prev_corridor_ref = None
        self.object_arc_active = False
        self.last_corridor_time = None
        self.last_corridor_stamp = None
        self.cached_pref_nom = None

    def needs_rebuild(self, now_sec: float, target_moved: Callable[[], bool]) -> bool:
        """
        Say whether this tick must rebuild the corridor.

        Yes when nothing is cached (a new move, see invalidate), when
        update_period has elapsed since the last rebuild, or -- consulted
        only when neither holds -- when target_moved() says the object
        target has moved far enough to pre-empt the period. That is the one
        place the period is legitimately pre-empted, and it is pre-empted by
        real target MOTION rather than by message arrival. Every other
        mode's corridor describes geometry that does not move, so
        target_moved returns False for them.
        """
        if self.cached_corridor is None or self.last_corridor_time is None:
            return True
        if (now_sec - self.last_corridor_time) >= self.update_period:
            return True
        return bool(target_moved())

    def maybe_rebuild(self, now_sec: float, now_stamp: Any,
                      build: Callable[[], dict],
                      target_moved: Callable[[], bool]) -> bool:
        """
        Rebuild the cached corridor if this tick needs it; say whether it did.

        now_stamp is the rclpy Time of this tick, kept as the corridor's
        computation stamp for the markers -- NOT re-derived from a fresh
        clock read at publish time, which would be off by however long the
        rest of the tick takes.
        """
        if not self.needs_rebuild(now_sec, target_moved):
            return False
        self.cached_corridor = build()
        self.last_corridor_time = now_sec
        self.last_corridor_stamp = now_stamp
        return True
