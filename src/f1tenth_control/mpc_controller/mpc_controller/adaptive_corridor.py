"""
Adaptive corridor (Part B item B4): widen the funnel only for an obstacle in the way.

Pure: no rclpy. Off by default (stack_params corridor_adaptive_enabled); with
it off the corridor is the fixed funnel corr_wmin -> corr_wmin x
corr_width_ratio and nothing here runs.

WHAT IT ANSWERS. Given the corridor the car is driving and the obstacles the
keep-out rows would consider (blocked.keepout_relevance, the same filter
plan_keepouts applies), how wide must the corridor's START be so that the
committed pass side still fits with the perception tolerance on top:

    passing LEFT  needs the rear axle at  c + d   (c = obstacle offset, + left)
    passing RIGHT needs the rear axle at  c - d   d = r + car_radius + margin
    required start half-width = side * c + d + tolerance

The side is pass_side's: the committed one when PassSideMemory has the
obstacle (read only -- committing stays plan_keepouts' job), else the cheaper
one (the side away from the obstacle's offset; a centred obstacle -> left,
decide_sides' own tie-break). One source of truth with the keep-out rows.

WHY THE START AND NOT THE OBSTACLE'S STATION. mpc_corr rebuilds the corridor
from the car's own pose every corridor_update_period, so the car always meets
an obstacle in the first metre of the corridor it is driving, where the
funnel is narrowest. Sizing the start is what still holds when the car gets
there; sizing the station would have to be redone, wider, on every rebuild of
the approach.

The answer is a START half-width; the far end follows the fixed ratio, so the
funnel keeps its shape (scaled, never reshaped). The caller clamps it to
[corr_wmin, corridor_adaptive_max_half_width_m]: never narrower than the
default, never wider than the cap.
"""

from typing import Dict, List, Optional, Sequence, Tuple

from mpc_controller.pass_side import LEFT, nearest_frame, PassSideMemory, RIGHT, side_name

#: A wider start than the corridor being driven pre-empts the periodic
#: rebuild only past this [m]: sub-centimetre growth (the obstacle's offset
#: jittering with the detection) waits for the next periodic rebuild instead
#: of re-anchoring the corridor every tick.
REBUILD_EPS_M = 0.01


def required_start_half_width(obstacles: Sequence[Tuple[float, float, float]],
                              corridor,
                              car_xy: Tuple[float, float],
                              car_radius: float,
                              margin: float,
                              tolerance: float,
                              relevant,
                              memory: Optional[PassSideMemory] = None,
                              ) -> Tuple[float, List[Dict]]:
    """
    Return (start half-width the obstacles in the way need [m], records).

    One record per obstacle considered. relevant(s_ahead, c, d, half_w) ->
    None to consider the obstacle, or a skip reason
    (blocked.keepout_relevance). 0.0 when nothing is in the way.
    """
    need = 0.0
    records: List[Dict] = []
    cx, cy = float(car_xy[0]), float(car_xy[1])
    for ox, oy, r in obstacles:
        ox, oy, r = float(ox), float(oy), float(r)
        pcx, pcy, nx, ny, hw = nearest_frame(ox, oy, corridor)
        tx, ty = ny, -nx  # build_straight_corridor: n = (-ty, tx)
        d = r + float(car_radius) + float(margin)
        c = nx * (ox - pcx) + ny * (oy - pcy)
        s_ahead = tx * (ox - cx) + ty * (oy - cy)
        if relevant(s_ahead, c, d, hw):
            continue
        committed = memory.lookup(ox, oy) if memory is not None else None
        if committed is not None:
            side, source = committed.side, 'committed'
        else:
            side, source = (RIGHT if c > 0.0 else LEFT), 'cheaper'
        need_i = side * c + d + float(tolerance)
        need = max(need, need_i)
        records.append({'o': [round(ox, 3), round(oy, 3)], 'r': round(r, 3),
                        'c': round(c, 3), 's_ahead': round(s_ahead, 3),
                        'side': side_name(side), 'side_source': source,
                        'need_w0': round(need_i, 3), 'half_w_here': round(hw, 3)})
    return need, records


def adaptive_start_half_width(need: float, base_w0: float, max_w0: float) -> float:
    """Return the start half-width to build: need clamped to [base_w0, max(max_w0, base_w0)]."""
    return min(max(float(need), float(base_w0)), max(float(max_w0), float(base_w0)))
