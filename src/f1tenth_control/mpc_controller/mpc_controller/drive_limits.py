"""The one speed clamp every /drive command from mpc_corr passes through.

Pure: no rclpy. MPC_corr._publish_drive calls clamp_drive_speed on every
command it publishes, whatever produced it (the solve, the stop paths, the
watchdog), so the limit holds even when the solver chooses otherwise.

WHY A CLAMP AT THE PUBLISHER AND NOT (ONLY) A SOLVER BOUND. The solver's own
speed box is vMin -1.0 / vMax 3.0 (MPC_corr.py's self.limits), and nothing
downstream is tighter: ackermann_mux arbitrates by priority only, and
vesc_driver clips ERPM at +-23250, which at speed_to_erpm_gain 5499.27 is
+-4.23 m/s, reverse included. Every speed a mission, the LLM translator or
mpc_corr's own defaults REQUESTS is at most 0.5 m/s and none requests reverse,
yet the archive holds solver-commanded reverse to -0.231 m/s in 13 runs and
up to 1.206 m/s in 3. In the closed-loop rig, a person walking across close
in front drove the command to +2.07 and -1.04 m/s against a zero reference:
with the steering saturated, speed is the only way the solver can rotate the
car toward the heading costs (w_psi, w_psi_stage), and reverse rotates it
the other way. See docs/analysis/2026-09-16_chase_overspeed.md.
"""

import math
from typing import Tuple

__all__ = ['clamp_drive_speed', 'validate_speed_limits']


def validate_speed_limits(max_forward_mps: float, max_reverse_mps: float) -> None:
    """Raise ValueError unless forward > 0 and reverse >= 0, both finite."""
    if not (math.isfinite(max_forward_mps) and max_forward_mps > 0.0):
        raise ValueError(
            f'max_forward_speed_mps must be finite and > 0, got {max_forward_mps!r}')
    if not (math.isfinite(max_reverse_mps) and max_reverse_mps >= 0.0):
        raise ValueError(
            f'max_reverse_speed_mps must be finite and >= 0 (a magnitude), '
            f'got {max_reverse_mps!r}')


def clamp_drive_speed(speed: float, max_forward_mps: float,
                      max_reverse_mps: float) -> Tuple[float, bool]:
    """Return (speed within [-max_reverse, +max_forward], whether it was clamped).

    A non-finite request is treated as a stop and reported as clamped: a NaN
    reaching the VESC is not a speed.
    """
    speed = float(speed)
    if not math.isfinite(speed):
        return 0.0, True
    if speed > max_forward_mps:
        return float(max_forward_mps), True
    if speed < -max_reverse_mps:
        return -float(max_reverse_mps), True
    return speed, False
