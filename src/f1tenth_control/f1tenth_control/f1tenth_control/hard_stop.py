"""hard_stop.py: the last-resort LiDAR stop's decision, as pure functions.

No rclpy and no message imports, so the whole rule is testable without a node,
a LiDAR or a running stack -- the same split mpc_controller's wall_boundaries.py
and f1tenth_costmap's costmap_boundary.py already use.

WHAT THIS IS FOR, AND WHAT IT IS NOT.

The MPC avoids only what perception RECOGNISES: tracked walls become boundary
rows, recognised objects from /camera/detections_3d become obstacle constraints.
That is a deliberate decision, and it leaves a gap by construction -- a chair
leg, a cable drum, a box, anything the detector has no class for and the wall
tracker will not fit a line to, is invisible to the solver. This is the backstop
for exactly that gap, and nothing else:

  * It does not avoid. It stops. There is no steering, no deflection, no
    corridor. A last resort that tries to be clever is a last resort that can be
    wrong in a new way.
  * It does not reason about classes, tracks, persistence or geometry beyond a
    radius. Every one of those is a thing that can fail to fire.
  * It is NOT the 0.40 m behaviour-tree proximity e-stop (IsProximityTooClose),
    which stays exactly as it is. That one is tuned to stop the car BEFORE a
    collision with room to spare. This one triggers at 0.20 m from the LiDAR,
    0.10 m ahead of the bumper, where the only remaining question is whether the
    wheels turn at all.

WHY IT IS OUTSIDE THE BEHAVIOUR TREE. The BT is the thing it has to survive. A
BT leaf cannot stop the car when the BT process is dead, has deadlocked, or is
ticking a tree whose root has gone FAILURE -- and those are precisely the
moments a last resort exists for. On the 2026-10-19 standstill run the BT
reported root=FAILURE with all three lanes FAILURE for the whole recording; a
leaf would have been reasoning about nothing.

THE SELF-HIT PROBLEM is the whole reason this file has more than one function.
The LiDAR sits at base_link x = +0.305, z = +0.190, scanning -135..+135 deg, and
range_min is 0.02 m. The car's own body is inside that field of view: at bearings
past roughly +-100 deg the beam leaves across the chassis and rear deck. A naive
"any return under 0.20 m stops the car" therefore latches permanently on the
car's own shell and the vehicle never moves again -- a failure that looks
identical to a real obstacle and cannot be driven out of. So every candidate
return is checked against the footprint polygon (plus a margin) FIRST, and a
return inside it is discarded as a self-hit, never counted.

The footprint comes from the URDF's own numbers, not from constants invented
here: base_front_x for the bumper and swept_clearance_body_half_width_m for the
half-width, the same two the front-distance pipeline uses.
"""

import math

#: Self-hit margin [m] grown around the footprint polygon. A return this far
#: outside the car's own shell is still treated as the car: the polygon is a
#: rectangle and the real body is not, the LiDAR has its own range noise, and
#: mounting tolerances move the sensor by millimetres. Erring outward costs a
#: 2 cm shell of blindness at the body; erring inward costs a permanent latched
#: stop on the car's own rear deck, which is not recoverable from the driver's
#: seat.
SELF_MARGIN_M = 0.02


def footprint_polygon(front_x, rear_x, half_width, margin=SELF_MARGIN_M):
    """Return the car's footprint as (x_min, x_max, y_min, y_max) in base_link, grown by margin.

    A rectangle rather than the true hull, deliberately: this is a mask for
    discarding self-hits, so over-covering the body is the safe direction (it
    only ever discards a return the car is already touching) while
    under-covering it latches the stop forever.
    """
    return (rear_x - margin, front_x + margin,
            -(half_width + margin), half_width + margin)


def is_self_hit(x, y, polygon):
    """Tell whether a base_link point lies inside the grown footprint polygon."""
    x_min, x_max, y_min, y_max = polygon
    return x_min <= x <= x_max and y_min <= y <= y_max


def scan_points_base_link(ranges, angle_min, angle_increment, range_min, range_max,
                          laser_x, laser_y=0.0):
    """Yield (range, bearing, x, y) for every VALID return, in base_link.

    `range` and `bearing` stay in the LASER's own frame because that is what the
    trigger radius is measured in (see trigger_range_hits) and what a log entry
    has to quote to be checkable against a raw scan. x/y are base_link, for the
    self-hit test. The laser is yaw-aligned with base_link on this rig
    (base_link -> laser is a pure translation, verified against the bag's own
    /tf_static), so the bearing needs no rotation.

    A return is valid when it is finite and inside [range_min, range_max]. NaN
    and inf are no-returns, and a value below range_min is the sensor saying
    "too close to measure", NOT "something at 0 m" -- treating that as a
    detection would stop the car on every dropout.
    """
    for i, r in enumerate(ranges):
        r = float(r)
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        bearing = angle_min + i * angle_increment
        yield r, bearing, laser_x + r * math.cos(bearing), laser_y + r * math.sin(bearing)


def trigger_range_hits(ranges, angle_min, angle_increment, range_min, range_max,
                       laser_x, polygon, trigger_range_m):
    """Return [(range, bearing, x, y), ...] for returns that should trigger a stop.

    A return triggers when its range FROM THE LASER ORIGIN is at or under
    trigger_range_m and it is not a self-hit. Measured from the laser, not from
    the bumper, because that is the one distance the sensor reports directly:
    re-referencing it to the bumper would make the threshold depend on bearing,
    and at these distances the bearing is exactly what is least trustworthy.

    Sorted nearest first, so a caller logging one representative hit logs the
    worst one.
    """
    hits = [(r, b, x, y)
            for r, b, x, y in scan_points_base_link(
                ranges, angle_min, angle_increment, range_min, range_max, laser_x)
            if r <= trigger_range_m and not is_self_hit(x, y, polygon)]
    hits.sort(key=lambda h: h[0])
    return hits


#: The mux lanes under /safety_stop (f1tenth_bringup/config/mux.yaml), by
#: priority, and their common timeout: what the car would get if this node
#: were quiet. Used only by the reverse gate below.
LOWER_LANES = (('teleop', 100), ('calibration_drive', 50), ('drive', 10))
LANE_TIMEOUT_S = 0.2


def winning_speed(latest, now, lanes=LOWER_LANES, timeout_s=LANE_TIMEOUT_S):
    """Return the speed the mux would pass from the lanes under the safety lane, or None.

    latest: {lane: (receipt time, speed)}. The mux's own rule: the highest
    priority lane whose last message is not older than its timeout.
    """
    for lane, _prio in sorted(lanes, key=lambda lp: -lp[1]):
        got = latest.get(lane)
        if got is not None and float(now) - got[0] <= timeout_s:
            return float(got[1])
    return None


def all_ahead(hits, polygon):
    """Tell whether every triggering return lies ahead of the bumper (the mask's front edge)."""
    return bool(hits) and all(x > polygon[1] for _r, _b, x, _y in hits)


def stop_now(engaged, allow_reverse, ahead_only, speed):
    """Return whether to claim the safety lane with a zero right now.

    hard_stop_allow_reverse (2026-10-20): with every triggering return AHEAD of
    the bumper, the stop blocks only motion TOWARDS it -- it stays quiet while
    the command the mux would pass is negative, so the MPC's reverse
    manoeuvre (or a joystick) can back the car out. Zero and positive
    commands are stopped as before: the gate re-claims the lane on the first
    non-negative command, and a reverse always passes through a stop
    (gear_change_dwell_s), so a forward command finds the lane claimed. A
    return beside the nose (the body would swing into it) and no command at
    all keep the stop as it always was. Quiet is not a release: the latch,
    its clear hold and every other /safety_stop source are unchanged.
    """
    if not engaged:
        return False
    if not allow_reverse or not ahead_only:
        return True
    return not (speed is not None and speed < 0.0)


class HardStopState:
    """The debounce and the latch, as a state machine over scans.

    Two separate mechanisms, and they are deliberately asymmetric:

      * DEBOUNCE (debounce_scans, default 2) before engaging. One scan is not
        evidence: a single beam clipping a passing hand, a reflection off a
        polished floor, one bad return. Two consecutive scans at 40 Hz is 50 ms,
        which is far inside the mechanical response of the drive train, so the
        cost of waiting is nothing and the cost of not waiting is a car that
        stops at random.

      * CLEAR HOLD (clear_s, default 0.5) before releasing, measured from the
        last scan that still had a hit. Asymmetric on purpose: engaging needs
        evidence, releasing needs CONFIDENCE, and the object at 0.18 m is
        exactly the one the LiDAR is about to lose (inside range_min on the next
        sweep, or shadowed by the body). Releasing on the first clear scan would
        stutter the car forward into whatever it just stopped for.

    `engaged` is the only thing the caller acts on. Time comes in from the
    caller as a monotonic clock reading, so this class owns no clock and the
    tests need no sleeps.
    """

    def __init__(self, debounce_scans=2, clear_s=0.5):
        self.debounce_scans = max(int(debounce_scans), 1)
        self.clear_s = float(clear_s)
        self.engaged = False
        #: Consecutive scans WITH a hit, reset by any clear scan.
        self.consecutive_hits = 0
        #: Monotonic time of the most recent scan that had a hit, or None.
        self.last_hit_time = None
        #: Engage events since construction -- the diagnostic's trigger count.
        self.trigger_count = 0

    def update(self, has_hit, now):
        """Fold one scan's verdict in; return True when the stop is engaged.

        A newly-engaged stop increments trigger_count, so the count is EVENTS
        (how many times the car was stopped) and not scans (how long it was
        stopped for) -- the first is what a replay or a run report wants.
        """
        if has_hit:
            self.consecutive_hits += 1
            self.last_hit_time = float(now)
            if not self.engaged and self.consecutive_hits >= self.debounce_scans:
                self.engaged = True
                self.trigger_count += 1
            return self.engaged

        self.consecutive_hits = 0
        if self.engaged and self.last_hit_time is not None:
            if float(now) - self.last_hit_time >= self.clear_s:
                self.engaged = False
        return self.engaged

    def tick(self, now):
        """Re-evaluate the clear hold without a scan; return True when still engaged.

        Needed because the latch releases on TIME, and the only thing that
        delivers time here is a scan. If the LiDAR stops publishing while the
        stop is engaged -- which is one of the ways the situation this exists
        for actually arises -- nothing would re-evaluate the hold and the stop
        would never release. It also must not release for that reason: see the
        node's own republish timer, which keeps the mux lane alive so a dead
        LiDAR holds the car rather than freeing it.
        """
        if not self.engaged or self.last_hit_time is None:
            return self.engaged
        if float(now) - self.last_hit_time >= self.clear_s:
            self.engaged = False
        return self.engaged

    def as_dict(self, n_hits=0, nearest=None):
        """JSON-ready diagnostic record for /safety/hard_stop."""
        return {
            'engaged': self.engaged,
            'trigger_count': self.trigger_count,
            'consecutive_hits': self.consecutive_hits,
            'debounce_scans': self.debounce_scans,
            'clear_s': self.clear_s,
            'n_hits_this_scan': int(n_hits),
            'nearest_range_m': (None if nearest is None else round(nearest[0], 4)),
            'nearest_bearing_deg': (None if nearest is None
                                    else round(math.degrees(nearest[1]), 2)),
        }
