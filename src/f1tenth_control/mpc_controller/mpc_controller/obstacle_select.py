"""
obstacle_select.py: which obstacles reach the solve, and the unknown-object speed cap.

Pure, no rclpy; MPC_corr calls both every control tick (obstacles v2,
2026-10-19).

ONE PATH. Since obstacles v2 every obstacle reaches the MPC from the semantic
layer's object list (/costmap/semantic_obstacles: recognised objects AND
LiDAR-only "unknown" ones, base_link, Obstacle2DArray), through the same
soft w_obs cost and the same margins (r + car_radius + avoidance_margin) as
before. The list MPC_corr keeps from it used to be overwritten on arrival and
never aged, so a publisher that went quiet left its last obstacles in the
solve forever (output/avoidance/log.md, A3: one chair for 89 s). It is now
dropped once older than max_age_s, and only the max_n nearest obstacles are
kept.
"""

import math


def select_obstacles(obstacles, x, y, age_s, max_age_s, max_n):
    """
    Return the obstacles the solve may use: fresh, the max_n nearest to (x, y).

    `obstacles` is [(x, y, r)] world frame; `age_s` the time since the list
    arrived (None: never). A list older than max_age_s is no list at all.
    max_n <= 0 keeps every obstacle. Distance is to the obstacle's edge.
    """
    if not obstacles or age_s is None or age_s > max_age_s:
        return []
    if max_n <= 0 or len(obstacles) <= max_n or x is None or y is None:
        return list(obstacles)
    return sorted(obstacles, key=lambda o: math.hypot(o[0] - x, o[1] - y) - o[2])[:max_n]


def apply_speed_cap(vdes, cap, age_s, max_age_s):
    """
    Return vdes limited by the unknown-object speed cap, when one is in force.

    `cap` <= 0 (or NaN, or a message older than max_age_s) means no cap: the
    semantic layer publishes -1.0 when no unknown object is in the strip.
    """
    if cap is None or age_s is None or age_s > max_age_s:
        return vdes
    if not math.isfinite(cap) or cap <= 0.0:
        return vdes
    return min(vdes, cap)
