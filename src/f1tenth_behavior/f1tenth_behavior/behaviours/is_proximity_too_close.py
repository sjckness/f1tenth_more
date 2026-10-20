"""py_trees Condition: is anything too close, per raw hardware sensors alone?

Last-resort emergency stop, deliberately independent of the YOLO/
classification/obstacle-array pipeline (Detection2DArray/Detection3DArray/
Obstacle2DArray) -- built directly from a single raw sensor source, raw /scan
(sensor_msgs/LaserScan), read as two disjoint angular windows so it keeps
working even if perception/classification is degraded or lagging:

  - Front (LiDAR-based -- REPLACES a prior ZED-depth-based check, see "Front
    check: camera -> lidar" below): the ~90 deg forward cone, |angle| <
    front_cone_half_angle_deg (default 45 deg) around raw angle 0. Trips if
    the minimum in-cone range < front_distance_threshold (default 0.15 m).
  - Sides + rear: the remaining ~270 deg arc, |angle| >= lidar_blind_cone_
    half_angle_deg (default 45 deg -- kept as its own independently-named/
    valued parameter from front_cone_half_angle_deg even though the two
    default to the same angle and, by construction, exactly partition the
    circle between them; see _in_front_cone's own docstring for why they're
    not forced to be literally the same parameter). Trips if the minimum
    in-window range < lidar_distance_threshold (default 0.15 m).

Both windows use the RAW scan frame: each sample's angle is computed as
msg.angle_min + i * msg.angle_increment, where angle 0 is the lidar's OWN
forward axis -- physically FRONT-facing on this car as of the lidar remount
(rear-facing -> front-facing) done alongside the first slam_toolbox
integration pass, confirmed against f1tenth_bringup/config/sensors.yaml
(angle_min/max = +-3.14, a full-circle scan) and f1tenth_description/launch/
description.launch.py's own base_link->laser static transform, live-confirmed
mounted at yaw=0.0 (was pi before the remount) with the laser positioned
0.12m ahead of base_link, matching the ZED camera's own forward offset --
i.e. this is a genuine forward-facing scan sector, not a spot the remount
left unaddressed (do NOT reuse this angle logic anywhere that assumes angle 0
is the car's rear -- that assumption is stale post-remount). inf/nan samples
and anything below msg.range_min are skipped before either window's filter,
never treated as a valid near-zero hit, in both windows identically.

Front check: camera -> lidar (this pass). Previously: raw ZED depth via
f1tenth_perception's front_depth_monitor_node, /perception/front_distance
(std_msgs/Float32) -- REMOVED here, not merely gated off, since the
replacement is meant to be the actual front check going forward, not an
optional alternative left to bit-rot. Reason: ZED stereo depth is a
documented, known-unreliable source against large flat/featureless walls,
and was a live candidate cause of the intermittent mid-mission stop-then-
resume behavior investigated separately -- a noisy near-threshold depth
reading could trip a real-but-spurious e-stop that self-clears next frame.
LiDAR is what sides/rear already trusted for exactly this kind of raw-range
proximity check, so the front cone now gets the same treatment, off the same
/scan subscription (one topic, one has-data guard, not two -- see below).
front_depth_monitor_node ITSELF is untouched and still runs: /perception/
front_distance is also independently consumed by MPC_corr.py's own
front_distance-based corridor-length logic, which has nothing to do with
this behaviour and is out of scope here (see that node's own docstring).

Known trade-off, stated plainly rather than silently absorbed: LiDAR cannot
see glass barriers (an already-known, already-deferred gap, unrelated to
this pass). The removed ZED front check may have been offering incidental,
unintended protection against a glass barrier at close range, even though
that was never its stated purpose. Removing it does not make the glass gap
itself any worse than it already was -- LiDAR-based sides/rear already
couldn't see glass either, and that gap was already accepted/deferred -- but
the front cone had a form of camera-based coverage against it that no longer
exists after this change, and that reduction in incidental coverage is a
real (if narrow) consequence of this pass, not a pre-existing condition.

front_distance_threshold: kept at its pre-existing 0.40 m default, NOT
recomputed down to the sides/rear's 0.20 m -- confirmed from stack_params.
yaml's own derivation (see below) that 0.40 m is a safety-margin composition
(car_radius + obstacle_safety_margin_m + proximity_front_extra_margin_m),
not a value chosen to compensate for ZED-specific lag or noise, so there is
no reason tied to the sensor swap to change it. It still trips at the same
physical distance as before, now measured by a different sensor.

Either condition tripping is enough -- wired into the emergency lane's
Selector alongside IsBatteryLow/IsSystemOverheated/IsEmergencyStopTriggered
(see behavior_executor_node.create_root()), reusing the existing Stop
behaviour onto the safety_stop mux lane (priority 200): no new actuation path,
the mux's priority ordering is what actually stops the car. Unconditional, not
gated behind any enable_* toggle -- same reasoning as IsBatteryLow: core
hardware proximity safety should not default to toggleable-off. (Flagged per
the task: if an override is ever wanted, that's a deliberate follow-up
decision, not something added here by default.)

Fails (does not trip) until at least one /scan message has been received --
same "no data yet must not mean tripped" reasoning as IsBatteryLow's
has_data guard. Both latest_lidar_front_min and latest_lidar_min_in_window
are populated together by the same _scan_callback, off the same single
subscription (this used to be two independent per-sensor guards, front and
lidar, before this pass -- now genuinely one, since there is only one raw
sensor left). This also correctly covers "a /scan message arrived but
nothing was within range in one of the windows" (all inf/filtered) -- that's
a genuinely clear reading, not missing data, and both cases resolve to "not
tripped" either way, so no extra state is needed to tell them apart.

front_distance_threshold/lidar_distance_threshold: normally passed in by
behavior_executor_node.create_root().

RETUNED BY THE 2026-09-01 MISSION-ANALYSIS FOLLOW-UP (see
docs/analysis/mission_analysis_2026-09-01.md). These used to be DERIVED from
stack_params.yaml's car_radius/obstacle_safety_margin_m/
proximity_front_extra_margin_m, giving 0.40 m front and 0.20 m side/rear.
They are now direct stack_params keys of their own -- proximity_front_
threshold_m / proximity_side_threshold_m, both defaulting to 0.15 m.

Two reasons the derivation was dropped rather than just rescaled:

  1. INTENT. That derivation composes a safety margin ON TOP OF the car's
     physical radius. A last-resort contact threshold is the opposite: it is
     deliberately INSIDE that margin, and should trip only once something is
     nearer than the car's own nominal radius (0.20 m). Deriving one from the
     other would keep the numbers linked while their meanings diverged.
  2. ROLE. The old 0.40 m front threshold existed partly because the front
     also had camera coverage via the handle_obstacle lane. That lane is now
     disabled by default (camera obstacles reach the MPC as soft avoidance
     instead -- see create_root()'s docstring), so the front threshold's job
     changed from "conservative backstop behind another check" to "the floor
     under MPC avoidance". Tightening it hands avoidance the whole 0.40-0.15 m
     band to work in; leaving it at 0.40 m would have this condition stopping
     the car in exactly the range avoidance is supposed to be steering
     through, which would make the test unable to answer its own question.

The 0.15/0.15 defaults here mirror those stack_params defaults -- kept in
sync for standalone construction/tests, not re-derived independently.

NOTE the sides/rear have NO camera and no MPC-avoidance coverage at all
(obstacle_projector_node only sees what the forward-facing camera sees), so
the side/rear threshold is the only protection in those directions. It was
tightened rather than removed for that reason.

THE FRONT CONE IS MEASURED FROM THE FRONT BUMPER (2026-10-12), not from the
LiDAR. Each valid return is placed in base_link through the laser's pose from
/tf_static; the front check is its distance from base_front (the URDF frame
f1tenth_description/urdf/base_front.xacro, 0.405 m ahead of base_link since
the 2026-10-13 tape measurement, 0.443 before), for
returns whose bearing FROM THE BUMPER is within front_cone_half_angle_deg of
the car's heading. A return behind the bumper (beside the body) is not
"front" any more. The side/rear window is unchanged: raw LiDAR ranges.

Why: the old raw-range check put its stopping point wherever the LiDAR sits.
With the car's TF of the time (laser 0.12 m ahead of base_link) and the bumper
at 0.443 m, the default 0.15 m front threshold tripped only once a return was
0.173 m INSIDE the front face -- after contact. From the bumper, the same
0.15 m is 0.15 m of real clearance ahead of the car: the stopping point moved
out, never in. (Measured 2026-10-13: laser 0.305, bumper 0.405, so the laser
is 0.10 m behind the bumper face.) Until the laser's transform has arrived the laser is assumed
at base_link's origin, yaw 0 -- returns then read closer to the bumper than
they are, so the check can only trip early, never late.
"""

import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_behavior.behaviours.static_frames import StaticFramePoses
from f1tenth_behavior.behaviours.stop import make_stop_msg
from f1tenth_description.vehicle_frames import base_front_x
import numpy as np
import py_trees
from sensor_msgs.msg import LaserScan


# ==============================================================================
# Windowing -- pure functions, no rclpy dependency, independently unit-
# testable (same "pure logic separate from ROS glue" convention
# f1tenth_perception/lidar_boundary_node.py's own _classify_side/
# _scan_angle_to_car_frame already use). _in_lidar_window was extracted here
# (was previously inline in _scan_callback) specifically so the front-facing-
# remount window flip was covered by a real test, not just eyeballed;
# _in_front_cone is its counterpart, added for the camera -> lidar front-cone
# swap (see module docstring).
# ==============================================================================

def _in_lidar_window(raw_angle: float, blind_cone_half_angle_rad: float) -> bool:
    """True if `raw_angle` (RAW scan frame -- 0 = lidar's own forward axis,
    physically the car's FRONT post-remount, see module docstring) falls
    OUTSIDE the front cone, i.e. should count toward this behaviour's
    side/rear proximity check. The excluded cone is +-blind_cone_half_angle_rad
    around raw angle 0 -- excluded here because _in_front_cone below (the
    dedicated front-cone check) already covers that direction; unchanged by
    the camera -> lidar front-cone swap (see module docstring) other than
    this docstring paragraph itself, which used to say the ZED-based front
    check covered that direction instead."""
    return abs(raw_angle) >= blind_cone_half_angle_rad


def _in_front_cone(raw_angle: float, front_cone_half_angle_rad: float) -> bool:
    """True if `raw_angle` falls INSIDE the front cone, i.e. should count
    toward this behaviour's front proximity check. Since 2026-10-12
    _scan_callback passes the return's bearing FROM THE FRONT BUMPER (0 = the
    car's heading), not the raw scan angle -- see the module docstring. The front
    cone is |raw_angle| < front_cone_half_angle_rad -- strict `<`, the exact
    complement of _in_lidar_window's `>=`, so that with equal half-angles
    (the default for both) every sample belongs to exactly one of the two
    windows, never both and never neither. Kept as its own parameter rather
    than reusing lidar_blind_cone_half_angle_rad directly so the two windows
    stay independently tunable if a future reason to make them unequal shows
    up -- _scan_callback below evaluates both independently per sample
    either way, so an unequal pair degrades gracefully (a small gap or
    overlap) rather than breaking."""
    return abs(raw_angle) < front_cone_half_angle_rad


class IsProximityTooClose(py_trees.behaviour.Behaviour):

    def __init__(
        self,
        name='IsProximityTooClose',
        scan_topic='/scan',
        front_distance_threshold=0.15,
        lidar_distance_threshold=0.15,
        lidar_blind_cone_half_angle_deg=45.0,
        front_cone_half_angle_deg=45.0,
        base_front_x_m=None,
        immediate_stop_topic=None,
        immediate_stop_frame_id='base_link/emergency',
        event_stop_hold_s=0.2,
        clock=time.monotonic,
        reverse_mode=None,
        front_hard_floor_m=0.20,
    ):
        super().__init__(name=name)
        # DIRECTION-AWARE FRONT STOP (Part D, reverse_integration.py). With a
        # ReverseModeMonitor here, the front trip stops only forward motion:
        # while the MPC stops for or drives a reverse it does not count,
        # unless the front return is under front_hard_floor_m. Side/rear
        # always count. None (direction_aware_front_stop false, or no
        # bt_reverse_integration) keeps the front trip a stop in every
        # direction. The gate lives in _tripped(), i.e. in should_stop(), so
        # the scan callback and the tick still decide the same way.
        self.reverse_mode = reverse_mode
        self.front_hard_floor_m = float(front_hard_floor_m)
        self.scan_topic = scan_topic
        # The front cone's apex: the URDF's base_front (module docstring).
        self.front_x = base_front_x() if base_front_x_m is None else float(base_front_x_m)
        self._frames = StaticFramePoses()
        self.front_distance_threshold = front_distance_threshold
        self.lidar_distance_threshold = lidar_distance_threshold
        self.lidar_blind_cone_half_angle_rad = math.radians(lidar_blind_cone_half_angle_deg)
        self.front_cone_half_angle_rad = math.radians(front_cone_half_angle_deg)

        self.node = None
        self.scan_sub = None

        self.latest_lidar_front_min = None
        self.latest_lidar_min_in_window = None

        # Diagnostic-only (live investigation, see chat) -- logged once on the
        # first /scan message received, not every callback.
        self._logged_scan_layout_once = False
        # The throttled DIAG line is throttled HERE, before the f-string and
        # the rclpy logger call: rclpy's own throttle_duration_sec runs only
        # after both, and the logger call alone measured ~1.5 ms per /scan on
        # linus -- at 40 Hz that was paid for a line printed every 2 s.
        self._diag_period_s = 2.0
        self._last_diag_mono = None

        # Per-layout trig tables (cpu-cleanup): cos/sin of every beam's angle
        # in base_link depend only on the scan layout and the laser's mount,
        # so they are computed once and reused while neither changes.
        self._layout_key = None
        self._cos = self._sin = self._side_mask = None

        # EVENT-DRIVEN E-STOP (cpu-cleanup). With immediate_stop_topic set,
        # the scan callback publishes the same zero-speed command the
        # emergency lane's Stop publishes (same topic, same frame_id) the
        # moment a scan trips, instead of waiting up to one BT tick period
        # for update() to be ticked. None (the default for standalone
        # construction/tests) keeps the old tick-only path.
        #
        # ONE DECISION, TWO CALLERS: both the callback and update() decide
        # through should_stop() = _active() and _tripped(). Today _active()
        # is always True -- the only enable condition is structural
        # (enable_lidar_safety_stop decides whether this object exists at
        # all, behavior_executor_node.create_root), mission phase, hold,
        # go_to_object, countdown and preflight never gate it, and it reads
        # no blackboard key. Any future gate (a docking/reverse override, a
        # blackboard flag) MUST go into should_stop() -- _active(), or
        # _tripped() as the direction-aware front gate does -- never into the
        # tree's structure, or the callback would keep stopping a car the
        # tree has released; test_bt_tick_and_event_estop.py enforces the
        # structure.
        #
        # RE-ARM. The callback publishes only while scans trip. The mux holds
        # the safety_stop lane for its timeout after the LAST stop (0.2 s,
        # f1tenth_bringup/config/mux.yaml), then the MPC's /drive wins again.
        # update() reports SUCCESS for event_stop_hold_s (= that timeout)
        # after the last callback stop, so the tree never shows "resumed"
        # while the mux still holds a callback stop -- including a trip that
        # starts and clears between two ticks, which before this would have
        # stopped the car for ~200 ms with nothing in /behavior/tree_status
        # or /safety/event. The reverse (tree stopping, callback not) is the
        # tree's own Stop, which re-arms on the same 0.2 s mux timeout.
        self.immediate_stop_topic = immediate_stop_topic
        self.immediate_stop_frame_id = immediate_stop_frame_id
        self.immediate_stop_pub = None
        self.immediate_stop_count = 0
        self.event_stop_hold_s = float(event_stop_hold_s)
        self._clock = clock
        self._last_event_stop = None
        # Whether that last callback stop was a front-only trip, which a
        # reverse releases (_event_stop_held).
        self._last_event_stop_front_only = False

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError(
                "IsProximityTooClose.setup() didn't find 'node' in kwargs") from e
        if self.immediate_stop_topic:
            self.immediate_stop_pub = self.node.create_publisher(
                AckermannDriveStamped, self.immediate_stop_topic, 10)
        self.scan_sub = self.node.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, 10)
        # The laser's mount, to measure the front cone from the bumper.
        self.tf_static_sub = self._frames.subscribe(self.node)

    def _scan_callback(self, msg: LaserScan):
        # ---- DIAGNOSTIC (live investigation, see chat) -- one-time raw
        # message layout, so real angle_min/max/increment/range_min can be
        # compared against what the window math below assumes. ----
        if not self._logged_scan_layout_once:
            self._logged_scan_layout_once = True
            self.node.get_logger().info(
                f'[IsProximityTooClose] DIAG /scan layout (first message): '
                f'angle_min={msg.angle_min:.4f} rad '
                f'({math.degrees(msg.angle_min):+.1f} deg), '
                f'angle_max={msg.angle_max:.4f} rad '
                f'({math.degrees(msg.angle_max):+.1f} deg), '
                f'angle_increment={msg.angle_increment:.6f} rad '
                f'({math.degrees(msg.angle_increment):.4f} deg), '
                f'range_min={msg.range_min:.3f} m, len(ranges)={len(msg.ranges)}'
            )

        total = len(msg.ranges)

        # The laser's pose in base_link, for the front cone (module docstring).
        laser = self._frames.pose(msg.header.frame_id)
        if laser is None:
            laser = (0.0, 0.0, 0.0)
            self.node.get_logger().warn(
                f'[IsProximityTooClose] no base_link <- "{msg.header.frame_id}" on '
                '/tf_static yet: front cone measured as if the LiDAR sat at '
                'base_link (trips early, never late).', throttle_duration_sec=5.0)
        lx, ly, lyaw = laser

        # VECTORISED (cpu-cleanup). This was a per-beam Python loop: 1081
        # beams x 40 Hz of cos/sin/atan2/hypot plus two helper calls each,
        # the single largest cost in behavior_executor_node while the car sat
        # still (~3.5 ms per scan on linus, several times that on the Orin).
        # Same per-sample arithmetic, same order of operations, now over
        # arrays; test_bt_tick_and_event_estop.py (TestVectorisedMatchesTheLoop)
        # checks it against the loop.
        #
        # Both windows are evaluated independently per sample (not via a
        # continue-past-one-into-the-other chain) so this stays correct even
        # if front_cone_half_angle_rad and lidar_blind_cone_half_angle_rad
        # are ever configured unequal -- see _in_front_cone's own docstring.
        # Side/rear counting (in_window_count/excluded_by_filter/
        # min_in_window) is the same computation as before the camera ->
        # lidar front-cone swap; only the front half is new.
        r = np.asarray(msg.ranges, dtype=np.float64)
        layout_key = (total, float(msg.angle_min), float(msg.angle_increment), lyaw)
        if layout_key != self._layout_key:
            angle = msg.angle_min + np.arange(total, dtype=np.float64) * msg.angle_increment
            self._cos = np.cos(lyaw + angle)
            self._sin = np.sin(lyaw + angle)
            # _in_lidar_window, over the whole array.
            self._side_mask = np.abs(angle) >= self.lidar_blind_cone_half_angle_rad
            self._layout_key = layout_key
        with np.errstate(invalid='ignore'):
            valid = np.isfinite(r) & (r >= msg.range_min)

        # FRONT: the return's distance and bearing from the bumper.
        rv = np.where(valid, r, 0.0)
        dx = lx + rv * self._cos - self.front_x
        dy = ly + rv * self._sin
        front = valid & (dx > 0.0)
        # _in_front_cone, over the candidates.
        front &= np.abs(np.arctan2(dy, dx)) < self.front_cone_half_angle_rad
        min_front = float(np.hypot(dx[front], dy[front]).min()) if front.any() else None

        side = self._side_mask
        in_window_count = int(np.count_nonzero(side))
        excluded_by_filter = int(np.count_nonzero(side & ~valid))
        side_valid = side & valid
        min_in_window = float(r[side_valid].min()) if side_valid.any() else None

        self.latest_lidar_front_min = min_front
        self.latest_lidar_min_in_window = min_in_window

        if self.immediate_stop_pub is not None and self.should_stop():
            self.immediate_stop_pub.publish(
                make_stop_msg(self.node, self.immediate_stop_frame_id))
            self.immediate_stop_count += 1
            self._last_event_stop = self._clock()
            self._last_event_stop_front_only = not (
                self._side_tripped() or self._front_full_stop())

        # ---- DIAGNOSTIC (live investigation, see chat) -- throttled, every
        # callback, so "not firing at all" vs. "firing but computing wrong"
        # can be told apart from the logs alone. front_min added alongside
        # the camera -> lidar front-cone swap -- this is now the input that
        # decides the front e-stop, worth the same visibility min_in_window
        # already had. ----
        now = time.monotonic()
        if self._last_diag_mono is None or now - self._last_diag_mono >= self._diag_period_s:
            self._last_diag_mono = now
            self.node.get_logger().info(
                f'[IsProximityTooClose] DIAG scan: total={total} '
                f'in_window={in_window_count} excluded_by_filter={excluded_by_filter} '
                f'min_in_window='
                + (f'{min_in_window:.3f}' if min_in_window is not None else 'none')
                + ' front_min(from bumper)='
                + (f'{min_front:.3f}' if min_front is not None else 'none'),
            )

    def _front_raw_tripped(self) -> bool:
        return (
            self.latest_lidar_front_min is not None
            and self.latest_lidar_front_min < self.front_distance_threshold
        )

    def _side_tripped(self) -> bool:
        return (
            self.latest_lidar_min_in_window is not None
            and self.latest_lidar_min_in_window < self.lidar_distance_threshold
        )

    def _front_full_stop(self) -> bool:
        """Return True for a front trip that stops the car in every direction.

        Always without the direction-aware gate, else only under
        front_hard_floor_m.
        """
        return self._front_raw_tripped() and (
            self.reverse_mode is None
            or self.latest_lidar_front_min < self.front_hard_floor_m)

    def _front_released(self) -> bool:
        """Return True while the MPC stops for or drives a reverse (fresh status only)."""
        return self.reverse_mode is not None and self.reverse_mode.reverse_active()

    def _tripped(self) -> bool:
        front_tripped = self._front_full_stop() or (
            self._front_raw_tripped() and not self._front_released())
        return front_tripped or self._side_tripped()

    def blocks_reverse(self) -> bool:
        """Return True for a reading that must also withdraw /bt/allow_reverse.

        A side/rear trip, or a front trip under the floor. The front trip
        above the floor is the one a reverse is for.
        """
        return self._side_tripped() or self._front_full_stop()

    def _active(self) -> bool:
        """The single enable predicate shared by both stop paths (see
        __init__, ONE DECISION). No gate exists today."""
        return True

    def should_stop(self) -> bool:
        """What both the scan callback and the tick decide on."""
        return self._active() and self._tripped()

    def _event_stop_held(self) -> bool:
        """Return True for a callback stop the mux still holds (RE-ARM in __init__).

        A front-only one is not re-published once a reverse has released it.
        """
        if self._last_event_stop_front_only and self._front_released():
            return False
        return (self._last_event_stop is not None and self._active()
                and self._clock() - self._last_event_stop <= self.event_stop_hold_s)

    def update(self):
        return (
            py_trees.common.Status.SUCCESS
            if self.should_stop() or self._event_stop_held()
            else py_trees.common.Status.FAILURE
        )
