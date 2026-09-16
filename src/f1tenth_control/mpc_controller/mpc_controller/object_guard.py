"""Leaving object mode: the steering ramp-out and the goal-refresh watchdog.

Pure: no rclpy, no numpy. Ported from the 2026-09-15 go_to_object prototype
(go_to_object/mission_state.py and the CurvatureLimiter in
go_to_object/pursuit_geometry.py, snapshotted in commit 36df2b7 and removed
once this port passed its tests), whose tracker was NOT ported --
semantic_layer_node is the one tracker in this stack. What carried over is
the two rules that prototype's tests pinned about stopping.

1. WHEN A MOVE ENDS, THE WHEELS RAMP OUT; THEY DO NOT STEP.
   The prototype's LOST state dropped the drive gate at once and brought the
   steering back to centre under a rate limit, because the servo can no more
   step to centre than step anywhere else. mpc_corr's stop paths (hold, no
   goal) publish steering 0 outright. When an object approach ends -- the
   mission says so on /mpc/goal_object_end, or /mpc/hold engages -- speed
   goes to zero on the same tick and SteeringRamp brings the wheels back from
   where they were, not from where they were last told to be when that is
   known (resync to the measured angle).

   Rate-limited on the STEERING ANGLE, not on curvature as the prototype did:
   mpc_corr commands an angle. d(delta)/dt = L / (1 + (L kappa)^2) *
   d(kappa)/dt, so the two limits differ by at most 8 % on this car (full
   lock, L kappa = 0.29). The default rate is a smoothness choice, not an
   actuator limit -- the same honesty the prototype's own default carried.

2. A SILENT SENDER IS A HARD STOP, NOT A RAMP.
   The ramp assumes someone is still ticking. The prototype's watchdog fired
   when that was in doubt and published hard zero, dragging the limiter's
   synchronisation point to zero with it, because a stalled node must not
   leave the last command latched. Here the sender is the mission's object
   handler, republishing ObjectGoal every behaviour-tree tick: if those
   refreshes stop for object_goal_timeout_sec, mpc_corr stops the car at
   once, stays in object mode, and resumes if they return. Its other input,
   odometry, already has exactly this failsafe: _update_active_odom drops the
   state to None past odom_stale_timeout_sec and control_loop publishes zero.
"""

from typing import Optional

__all__ = ['RefreshWatchdog', 'SteeringRamp']


class SteeringRamp:
    """Slew-rate limit on the commanded steering angle [rad].

    There is no unseeded state, deliberately: the steered wheels are always at
    SOME angle, and an earlier prototype version that cleared to "unseeded" on
    a mode change passed the next command through unlimited -- a step handed
    to the servo at exactly the moment one occurs. So:

    * seed()   -- set the synchronisation point outright (cold start, or the
                  watchdog dragging it to zero).
    * resync() -- a mode change: snap to the MEASURED angle when there is one,
                  otherwise keep the last value. It never clears.
    """

    def __init__(self, max_rate_rad_s: float, initial: float = 0.0):
        """Allow at most max_rate_rad_s of steering change per second."""
        if not max_rate_rad_s > 0.0:
            raise ValueError(f'max_rate_rad_s must be > 0, got {max_rate_rad_s}')
        self.max_rate = float(max_rate_rad_s)
        self._previous = float(initial)
        self.saturated = False
        self.saturations = 0
        self.applications = 0

    @property
    def previous(self) -> float:
        """Return the last angle this ramp emitted. Never None."""
        return self._previous

    def seed(self, angle: float) -> None:
        """Set the synchronisation point outright."""
        self._previous = float(angle)

    def resync(self, measured: Optional[float] = None) -> float:
        """Snap to the measured angle when known; otherwise keep the last one."""
        if measured is not None:
            self._previous = float(measured)
        return self._previous

    def apply(self, target: float, dt: float) -> float:
        """Rate-limit `target` against the previous output.

        dt <= 0 HOLDS the previous value: zero elapsed time permits zero slew.
        Passing the demand through there would hand the servo a step on the
        first tick after a mode change, where there is no previous tick to
        measure from.
        """
        target = float(target)
        self.applications += 1
        if dt <= 0.0:
            self.saturated = target != self._previous
            if self.saturated:
                self.saturations += 1
            return self._previous
        max_step = self.max_rate * float(dt)
        delta = target - self._previous
        if delta > max_step:
            limited = self._previous + max_step
            self.saturated = True
        elif delta < -max_step:
            limited = self._previous - max_step
            self.saturated = True
        else:
            limited = target
            self.saturated = False
        if self.saturated:
            self.saturations += 1
        self._previous = limited
        return limited


class RefreshWatchdog:
    """Trip when refreshes stop arriving for longer than timeout_sec."""

    def __init__(self, timeout_sec: float):
        """Trip after timeout_sec without a refresh."""
        if not timeout_sec > 0.0:
            raise ValueError(f'timeout_sec must be > 0, got {timeout_sec}')
        self.timeout_sec = float(timeout_sec)
        self._last = None

    def note(self, now_sec: float) -> None:
        """Record a refresh at now_sec."""
        self._last = float(now_sec)

    def reset(self) -> None:
        """Forget the last refresh (a new move starts un-refreshed)."""
        self._last = None

    def tripped(self, now_sec: float) -> Optional[str]:
        """Return why the watchdog tripped, or None while refreshes are fresh.

        Never refreshed is not tripped: the caller only asks while a move it
        started by a refresh is active.
        """
        if self._last is None:
            return None
        age = float(now_sec) - self._last
        if age > self.timeout_sec:
            return f'no ObjectGoal refresh for {age:.2f} s (timeout {self.timeout_sec:.2f} s)'
        return None
