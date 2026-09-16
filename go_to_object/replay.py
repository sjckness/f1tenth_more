"""Offline replay: reproduce a live command sequence from a recorded bag.

The point is determinism. Everything the node computes is a pure function of
the *ordered sequence* of inputs -- odom samples, detections, and control
ticks -- so feeding that sequence back through :mod:`object_tracker`,
:mod:`pursuit_geometry` and :mod:`mission_state` must reproduce the published
commands bit for bit. When it does not, the bug is in the node: some
wall-clock read, some unordered iteration, some state that did not come from
a message.

That makes every drive a regression test, and it is the only way to debug a
run you cannot repeat.

Two things this depends on, both of which are properties of the node rather
than of this file:

* **Ordering.** The node processes callbacks in arrival order, so replay has
  to as well. Bags record a reception timestamp alongside the message
  timestamp; order events by reception and the interleaving is recovered.
* **Tick times.** The control tick reads the clock, and that value is not
  otherwise recoverable. The node therefore stamps every diagnostics message
  with the exact clock value the tick used, and replay reads its ticks from
  that topic. Without it, replay could only approximate the tick schedule and
  would never be bit-identical.

The core (:func:`replay`) is pure and has no ROS dependency; only
:func:`read_bag` does, and it imports lazily.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from typing import Iterable, Sequence

from .diagnostics_format import parse_float, parse_optional_float
from .mission_state import GoToObjectMission, MissionParams, MissionState
from .object_tracker import ObjectTracker, TrackerParams
from .pursuit_geometry import CurvatureLimiter, PursuitParams

__all__ = ['OdomEvent', 'DetectionEvent', 'TickEvent', 'ReplayConfig',
           'ReplayedCommand', 'ReplayGapError', 'check_tick_continuity',
           'replay', 'compare', 'read_bag', 'main']

DEFAULT_CONTROL_PERIOD = 0.01
"""Matches the node's default publish_rate_hz of 100 Hz."""

DEFAULT_TICK_TOLERANCE = 0.5
"""An interval may differ from the control period by this fraction of it.

Half a period is loose enough for ordinary scheduling jitter and tight
enough that a single dropped tick -- which doubles the interval, a full
period of error -- is always caught.
"""


class ReplayGapError(RuntimeError):
    """The recorded tick sequence is not continuous, so replay is meaningless."""


@dataclass(frozen=True)
class OdomEvent:
    stamp: float
    x: float
    y: float
    psi: float


@dataclass(frozen=True)
class DetectionEvent:
    stamp: float
    """Frame *capture* time, as the node used it."""
    bx: float
    by: float


@dataclass(frozen=True)
class TickEvent:
    stamp: float
    measured_kappa: float | None = None
    last_odom_stamp: float | None = None


@dataclass(frozen=True)
class ReplayConfig:
    pursuit: PursuitParams = field(default_factory=PursuitParams)
    tracker: TrackerParams = field(default_factory=TrackerParams)
    mission: MissionParams = field(default_factory=MissionParams)
    max_kappa_rate: float = 0.5
    enabled: bool = True
    """Mirror of the node's shadow-mode flag: when false the published
    curvature is zero while everything else is computed as normal."""


@dataclass(frozen=True)
class ReplayedCommand:
    stamp: float
    state: str
    curvature: float
    curvature_raw: float
    limiter_saturated: bool
    drive_enable: bool
    watchdog: bool


def replay(events: Sequence[object], config: ReplayConfig | None = None,
           ) -> list[ReplayedCommand]:
    """Re-run the pipeline over an ordered event sequence.

    ``events`` must be in the order the node saw them, not sorted by message
    stamp -- a detection is processed when it *arrives*, which is by design
    later than the moment it describes.
    """
    config = config if config is not None else ReplayConfig()
    tracker = ObjectTracker(config.tracker)
    machine = GoToObjectMission(config.pursuit, config.mission,
                                CurvatureLimiter(config.max_kappa_rate))

    pose = (0.0, 0.0, 0.0)
    out: list[ReplayedCommand] = []

    for event in events:
        if isinstance(event, OdomEvent):
            pose = (event.x, event.y, event.psi)
            tracker.push_odom(event.stamp, event.x, event.y, event.psi)
        elif isinstance(event, DetectionEvent):
            tracker.push_detection(event.stamp, (event.bx, event.by))
        elif isinstance(event, TickEvent):
            track = tracker.state(event.stamp)
            command = machine.update(
                event.stamp, (pose[0], pose[1]), pose[2], track,
                measured_kappa=event.measured_kappa,
                last_odom_stamp=event.last_odom_stamp)
            published = command.curvature if config.enabled else 0.0
            out.append(ReplayedCommand(
                stamp=event.stamp,
                state=command.state.value,
                curvature=published,
                curvature_raw=command.curvature_raw,
                limiter_saturated=command.limiter_saturated,
                drive_enable=command.drive_enable,
                watchdog=command.watchdog))
        else:
            raise TypeError(f'unknown replay event: {type(event).__name__}')

    return out


def check_tick_continuity(stamps: Sequence[float],
                          control_period: float = DEFAULT_CONTROL_PERIOD,
                          tolerance: float = DEFAULT_TICK_TOLERANCE) -> None:
    """Raise on the first tick interval inconsistent with the control period.

    If the bag dropped, throttled or lost diagnostics messages, replay runs a
    *different* schedule from the one the node ran: it ticks where the
    recording has ticks, and the missing ones simply never happen. Every
    downstream number then differs for a reason that has nothing to do with
    the node, while ``compare`` still cheerfully reports how many matched.

    So this fails loudly rather than interpolating the gap, skipping it, or
    downgrading to a warning. A bag that cannot support replay should say so
    before anyone reads a conclusion off it.
    """
    if not control_period > 0.0:
        raise ValueError(f'control_period must be > 0, got {control_period}')
    if len(stamps) < 2:
        return

    low = control_period * (1.0 - tolerance)
    high = control_period * (1.0 + tolerance)

    for index, (first, second) in enumerate(zip(stamps, stamps[1:])):
        interval = second - first
        if interval <= 0.0:
            raise ReplayGapError(
                f'diagnostics tick {index + 1} at t={second!r} does not '
                f'advance on tick {index} at t={first!r} (interval '
                f'{interval!r} s). The recording is out of order or contains '
                'duplicate ticks; replay cannot reconstruct the schedule.')
        if low <= interval <= high:
            continue
        missing = interval / control_period - 1.0
        raise ReplayGapError(
            f'diagnostics gap between tick {index} at t={first!r} and tick '
            f'{index + 1} at t={second!r}: interval {interval:.6g} s against '
            f'a control period of {control_period:.6g} s '
            f'(tolerance +/-{tolerance * 100:.0f}%), about {missing:.1f} tick(s) '
            'missing. Replay would run a different schedule from the node, so '
            'the comparison would be meaningless. Re-record without dropping '
            'or throttling the diagnostics topic.')


def compare(replayed: Sequence[ReplayedCommand],
            recorded: Sequence[dict], tolerance: float = 0.0) -> list[str]:
    """Differences between a replay and what the node published live.

    ``tolerance`` defaults to zero: this is an exactness check, and a replay
    that only *nearly* matches has found a nondeterminism worth naming rather
    than a rounding difference worth absorbing.
    """
    problems: list[str] = []
    if len(replayed) != len(recorded):
        problems.append(
            f'tick count differs: replayed {len(replayed)}, '
            f'recorded {len(recorded)}')

    for i, (got, want) in enumerate(zip(replayed, recorded)):
        for key, value in (('state', got.state),
                           ('curvature', got.curvature),
                           ('curvature_raw', got.curvature_raw),
                           ('drive_enable', got.drive_enable)):
            if key not in want:
                continue
            expected = want[key]
            if isinstance(value, float):
                if abs(value - float(expected)) > tolerance:
                    problems.append(
                        f'tick {i} @ {got.stamp:.6f}: {key} '
                        f'replayed {value!r} vs recorded {expected!r}')
            elif value != expected:
                problems.append(
                    f'tick {i} @ {got.stamp:.6f}: {key} '
                    f'replayed {value!r} vs recorded {expected!r}')
    return problems


def read_bag(path: str, *, odom_topic: str, detection_topic: str,
             diagnostics_topic: str) -> tuple[list[object], list[dict]]:
    """Build the ordered event list and the recorded commands from a bag.

    Imports rosbag2 lazily, so this module is importable without ROS.
    """
    import rosbag2_py                      # noqa: F401  (ROS-only)
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=''),
        rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    events: list[object] = []
    recorded: list[dict] = []

    while reader.has_next():
        topic, raw, _receive_ns = reader.read_next()
        if topic not in (odom_topic, detection_topic, diagnostics_topic):
            continue
        message = deserialize_message(raw, get_message(types[topic]))

        if topic == odom_topic:
            from .mission_node import _stamp_to_sec, _yaw_from_quaternion
            p = message.pose.pose.position
            events.append(OdomEvent(
                _stamp_to_sec(message.header.stamp), p.x, p.y,
                _yaw_from_quaternion(message.pose.pose.orientation)))
        elif topic == detection_topic:
            from .mission_node import _stamp_to_sec
            events.append(DetectionEvent(
                _stamp_to_sec(message.header.stamp),
                message.point.x, message.point.y))
        else:
            fields = {kv.key: kv.value for kv in message.status[0].values}
            events.append(TickEvent(
                parse_float(fields['stamp']),
                measured_kappa=parse_optional_float(
                    fields.get('measured_kappa', '')),
                last_odom_stamp=parse_optional_float(
                    fields.get('last_odom_stamp', ''))))
            recorded.append(dict(
                state=fields['state'],
                curvature=parse_float(fields['kappa_limited']),
                curvature_raw=parse_float(fields['kappa_raw']),
                drive_enable=fields['drive_enable'] == 'True'))

    return events, recorded


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bag')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--detection-topic', default='/go_to_object/detection_point')
    parser.add_argument('--diagnostics-topic', default='/go_to_object/diagnostics')
    parser.add_argument('--control-period', type=float,
                        default=DEFAULT_CONTROL_PERIOD,
                        help='expected diagnostics tick interval [s]')
    parser.add_argument('--tick-tolerance', type=float,
                        default=DEFAULT_TICK_TOLERANCE,
                        help='permitted fraction of a period of jitter')
    parser.add_argument('--json', action='store_true',
                        help='write the replayed command sequence to stdout')
    args = parser.parse_args(list(argv) if argv is not None else None)

    events, recorded = read_bag(
        args.bag, odom_topic=args.odom_topic,
        detection_topic=args.detection_topic,
        diagnostics_topic=args.diagnostics_topic)
    # Before anything is compared: a gapped recording makes the comparison
    # meaningless while still reporting a result.
    check_tick_continuity([e.stamp for e in events if isinstance(e, TickEvent)],
                          args.control_period, args.tick_tolerance)

    replayed = replay(events)

    if args.json:
        json.dump([c.__dict__ for c in replayed], sys.stdout, indent=1)
        return 0

    problems = compare(replayed, recorded)
    print(f'{len(events)} events, {len(replayed)} ticks replayed')
    if not problems:
        print('replay is bit-identical to the recorded commands')
        return 0
    print(f'{len(problems)} divergence(s) -- this is a bug in the node, '
          'not in replay:')
    for line in problems[:20]:
        print(f'  {line}')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
