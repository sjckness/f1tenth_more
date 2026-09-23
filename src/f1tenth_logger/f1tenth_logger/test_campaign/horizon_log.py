"""Read horizon.jsonl, and measure a plan against what the car then did.

ONE HORIZON IS ONE SOLVE'S PLAN: the states the MPC expected to occupy over
the next N control periods, logged every solve (see robot_logger.log_horizon
and mpc_controller.campaign_status.horizon_payload). A test folder holds
hundreds of them, which is why nothing here draws them all and why the
selection helpers below exist.

WHY THE ERROR LIVES HERE rather than in the plotter or the export. Both need
it -- the figure to be honest about what it is showing, the campaign CSV to
carry it as a number -- and a metric computed twice is a metric that will
eventually be computed two ways. :func:`prediction_error` is the only
implementation.

WHAT THE ERROR IS, precisely. Horizon step k of a solve logged at t0 is a
prediction for time ``t0 + (k + 1) * ts``: the states are x_1..x_N and the
current state x0 is deliberately not among them. The error of that step is the
distance from its (x, y) to where the car ACTUALLY was at that same time,
linearly interpolated between the two nearest kinematics samples. A step whose
time falls outside the logged trajectory is DROPPED, not extrapolated -- the
last horizon of every test predicts past the end of the log, and counting that
against the controller would charge it for the recording stopping.

Stdlib only, like corridor_def and for the same reason: loading a test folder
must work on a machine with no ROS and no numpy.
"""

from __future__ import annotations

import json
from bisect import bisect_left

__all__ = [
    'HorizonRecord',
    'load_horizons',
    'prediction_error',
    'select_by_corridor',
    'select_every',
]


class HorizonRecord:
    """One line of horizon.jsonl.

    ``ts`` is the control period the solve ran at and ``corridor_id`` the
    corridor that was current when it ran, both stamped at log time. A record
    missing ``ts`` cannot have its steps placed in time, so it is usable for
    drawing but not for :func:`prediction_error`; :attr:`timed` says which.
    """

    __slots__ = ('t', 'i', 'corridor_id', 'frame_id', 'ts', 'x', 'y',
                 'yaw', 'v', 'steer', 'accel')

    def __init__(self, raw):
        self.t = _number(raw.get('t'))
        self.i = raw.get('i')
        self.corridor_id = raw.get('corridor_id')
        self.frame_id = raw.get('frame_id')
        self.ts = _number(raw.get('ts'))
        self.x = _floats(raw.get('x'))
        self.y = _floats(raw.get('y'))
        self.yaw = _floats(raw.get('yaw'))
        self.v = _floats(raw.get('v'))
        self.steer = _floats(raw.get('steer'))
        self.accel = _floats(raw.get('accel'))

    def __len__(self):
        return min(len(self.x), len(self.y))

    @property
    def timed(self):
        """True when this record's steps can be placed on the clock."""
        return (self.t is not None and self.ts is not None
                and self.ts > 0.0 and len(self) > 0)

    def step_times(self):
        """Wall time of each step, or [] when the record is not :attr:`timed`.

        Step k is at ``t + (k + 1) * ts``: x_pred holds x_1..x_N, so the first
        entry is already one period ahead of the solve, never at it.
        """
        if not self.timed:
            return []
        return [self.t + (k + 1) * self.ts for k in range(len(self))]

    def __repr__(self):
        return (f'<HorizonRecord i={self.i} t={self.t} '
                f'corridor={self.corridor_id} {len(self)} steps>')


def _number(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if value == value and abs(value) != float('inf') else None


def _floats(values):
    """A list of finite floats, or [] -- never a list with a hole in it."""
    if not isinstance(values, (list, tuple)):
        return []
    out = []
    for value in values:
        number = _number(value)
        if number is None:
            return []
        out.append(number)
    return out


def load_horizons(path):
    """Every well-formed line of a horizon.jsonl, as HorizonRecords.

    A malformed or truncated line is skipped rather than raising, and a missing
    file is an empty list: same contract as corridor_def.load_corridors, and
    for the same reasons -- the file is appended to by a live node, and a test
    that logged no horizon is a real outcome rather than a fault.
    """
    records = []
    try:
        handle = open(path, 'r', encoding='utf-8')
    except (FileNotFoundError, NotADirectoryError, IsADirectoryError):
        return records
    with handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                raw = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(raw, dict):
                record = HorizonRecord(raw)
                if len(record):
                    records.append(record)
    return records


def select_by_corridor(records):
    """The FIRST horizon solved under each corridor, in corridor order.

    This is the plot's default, and the reason the logger stamps corridor_id
    onto every horizon instead of leaving the plotter to match by time: it
    shows each corridor beside the plan actually made under it, which is the
    comparison the figure exists for. Taking the first rather than the last
    means the plan shown is the one the fresh corridor produced, before the
    reference had moved under it.

    Records with no corridor_id (logged before any corridor, i.e. before the
    move started) are grouped under None and contribute at most one horizon.
    """
    seen = {}
    for record in records:
        key = record.corridor_id
        if key not in seen:
            seen[key] = record
    return [seen[key] for key in seen]


def select_every(records, n):
    """Every ``n``-th horizon, starting at the first. ``n`` < 1 selects none.

    The escape hatch from :func:`select_by_corridor` for a test where the
    question is how the plan evolves BETWEEN rebuilds rather than at them.
    """
    if n is None or n < 1:
        return []
    return list(records[::int(n)])


def _interpolate(t_query, times, xs, ys):
    """(x, y) of the trajectory at ``t_query``, or None outside its span.

    ``times`` must be non-decreasing. Outside the span returns None rather than
    clamping to an endpoint: a clamped value is a real number that is not a
    measurement, and it would quietly become the error of every step past the
    end of the log.
    """
    if not times or t_query < times[0] or t_query > times[-1]:
        return None
    index = bisect_left(times, t_query)
    if index == 0:
        return xs[0], ys[0]
    t0, t1 = times[index - 1], times[index]
    if t1 == t0:
        return xs[index], ys[index]
    frac = (t_query - t0) / (t1 - t0)
    return (xs[index - 1] + frac * (xs[index] - xs[index - 1]),
            ys[index - 1] + frac * (ys[index] - ys[index - 1]))


def prediction_error(records, times, xs, ys, window=None):
    """``(mean_m, max_m, n_steps)`` between predicted horizons and the drive.

    Pools every (horizon, step) pair whose predicted time falls inside the
    logged trajectory: ``mean_m`` is the average distance over all of them and
    ``max_m`` the worst single one. Pooling over steps rather than averaging
    per-horizon means a solve whose horizon is mostly off-log does not carry
    the same weight as one measured over its full length.

    ``window`` is an optional ``(t_start, t_end)`` -- normally the drive
    window -- restricting which SOLVES count. Horizons solved while the car was
    stationary before the mission started are trivially accurate and would
    otherwise drag the mean toward zero.

    Returns ``(None, None, 0)`` when nothing could be measured: no horizons, no
    trajectory, or no record carrying the ``ts`` its steps need to be placed in
    time. A caller must distinguish that from an error of zero.
    """
    pairs = list(zip(times, xs, ys))
    pairs = [(t, x, y) for t, x, y in pairs
             if t is not None and x is not None and y is not None
             and t == t and x == x and y == y]
    pairs.sort(key=lambda row: row[0])
    if not pairs or not records:
        return None, None, 0
    t_traj = [row[0] for row in pairs]
    x_traj = [row[1] for row in pairs]
    y_traj = [row[2] for row in pairs]

    total, worst, count = 0.0, None, 0
    for record in records:
        if not record.timed:
            continue
        if window is not None and not (window[0] <= record.t <= window[1]):
            continue
        for step_t, px, py in zip(record.step_times(), record.x, record.y):
            actual = _interpolate(step_t, t_traj, x_traj, y_traj)
            if actual is None:
                continue
            error = ((px - actual[0]) ** 2 + (py - actual[1]) ** 2) ** 0.5
            total += error
            count += 1
            if worst is None or error > worst:
                worst = error
    if not count:
        return None, None, 0
    return total / count, worst, count
