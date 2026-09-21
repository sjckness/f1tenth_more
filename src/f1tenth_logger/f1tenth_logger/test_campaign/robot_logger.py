"""Per-test logging for LLM-driven F1TENTH campaigns.

One :class:`TestLogger` instance covers exactly one test run: it owns a folder
under ``<ROOT>/<campaign>/<MISSION>/<TEST_ID>/`` and writes every stream the
analysis needs (kinematics, IMU, commands, LLM calls, corridors, events, meta).

Standard library only, plus PyYAML to read the prompt table.

Thread safety
-------------
ROS callbacks arrive on different threads, so every mutation of logger state
and every write to a stream is taken under a single re-entrant lock. The only
work deliberately left outside the lock is the body of :meth:`TestLogger.llm_call`
-- holding a lock across a multi-second network request would serialise the
whole node.

Conventions
-----------
* Body frame: **x forward, y left, z up**.
* IMU acceleration in **m/s^2, gravity included** (a level, still robot reads
  ``az ~ +9.81``).
* Gyro rates in **rad/s**.
* ``yaw`` in **rad, world frame, counter-clockwise positive**.
* ``cmd_steer`` in **rad** (left positive, matching yaw).
* Every ``t`` column is **seconds since test start**, taken from
  ``time.monotonic()``. Wall-clock start/end live in ``meta.json``.
* Clearances are **robot edge to boundary**: the robot is a circle of
  ``robot_radius`` and a negative value means the footprint is over the line.
  ``obstacle_clearance`` follows the same rule -- it is the distance from the
  robot's **edge** to the nearest obstacle, as perception measures it, so a
  negative value means contact.
* ``finish()`` only records an **automatic hint** (``auto_outcome`` /
  ``auto_success``). The real pass/fail is the ``success`` column a human
  fills in ``campaign_results.csv`` (see ``export_campaign_csv``).
* Safety events go through :meth:`TestLogger.log_event` under two reserved
  names, each carrying a ``cause``::

      log.log_event("contact", cause="hit the left wall")
      log.log_event("estop", cause="operator pressed the button")

  ``export_campaign_csv`` reads exactly those two names.
* The mission lifecycle uses four more reserved event names, which decide
  the window the export measures over::

      log.log_event("mission_loaded", plan_id=..., countdown_s=3.0)
      log.log_event("mission_started")
      log.log_event("mission_finished", reason="...")   # or mission_aborted

  Data is recorded from ``mission_loaded`` (earlier with pre-roll), but the
  driving metrics are computed only between ``mission_started`` and the end.
* The plan a test ran is saved next to its streams by :meth:`TestLogger.save_plan`:
  ``plan.json`` for the initial one, ``plan_replan_<N>.json`` for the N-th
  replan. Its identity is :func:`plan_hash` -- sha256 of the canonical JSON
  (sorted keys, no whitespace), so formatting never changes it. If the file
  the mission loader was handed (``missions/llm_generated/<plan_id>.json``)
  hashes differently, or cannot be read, a ``plan_file_mismatch`` event
  says so.
* In ``llm_calls.jsonl``, ``response`` is **deprecated** for calls recorded
  from a planner message: it held the translator's output, never the model's
  reply. Read ``translated_plan`` (the translator's output) and ``llm_raw``
  (what the model wrote) instead; such records carry
  ``"deprecated": {"response": "translated_plan"}``.
"""

from __future__ import annotations

import atexit
import csv
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path

try:
    import yaml
except ImportError as exc:  # pragma: no cover - environment problem, not logic
    raise ImportError(
        "robot_logger needs PyYAML to read the prompt table: pip install PyYAML"
    ) from exc

__all__ = [
    "TestLogger",
    "find_root",
    "make_test_id",
    "parse_test_id",
    "plan_hash",
    "signed_clearance",
    "corridor_from_centerline",
    "TEST_ID_RE",
    "SUMMARY_FIELDS",
]

ROOT_NAME = "f1tenth_more"
DEFAULT_CAMPAIGN = "first_test_campaing"
TEST_ID_RE = re.compile(r"^P(\d{3})-R(\d{3})-(\d{8}T\d{6})$")
_TS_FMT = "%Y%m%dT%H%M%S"
_FLUSH_PERIOD_S = 0.5

#: Column order of ``results.csv`` and of the ``summary`` block in ``meta.json``.
SUMMARY_FIELDS = [
    "test_id",
    "mission",
    "prompt_num",
    "repetition",
    "prompt_hash",
    "git_commit",
    "start_time",
    "duration_s",
    "auto_outcome",
    "auto_success",
    "reason",
    "n_llm_calls",
    "llm_latency_mean_ms",
    "llm_latency_max_ms",
    "n_corridors",
    "min_corridor_clearance",
    "min_obstacle_clearance",
    "max_speed",
    "max_acc",
    "path_length",
    "max_imu_horiz_acc",
    "max_abs_yaw_rate",
    "pose_rate_hz",
    "imu_rate_hz",
    "cmd_rate_hz",
    "n_imu",
    "n_cmd",
]

_KINEMATICS_COLS = [
    "t", "x", "y", "yaw", "yaw_rate", "vx", "vy", "speed", "ax", "ay", "acc",
    "corridor_clearance", "obstacle_clearance",
]
_IMU_COLS = ["t", "sensor_stamp", "ax", "ay", "az", "gx", "gy", "gz", "imu_yaw"]
_COMMAND_COLS = [
    "t", "cmd_speed", "cmd_steer", "cmd_yaw_rate", "cmd_throttle", "cmd_brake",
    "source",
]
_MPC_COLS = ["t", "status", "solve_time_ms", "cost", "iterations"]
_LLM_COLS = [
    "call_idx", "tag", "t_sent", "t_received", "latency_ms", "ttft_ms",
    "prompt_chars", "response_chars", "ok", "error",
]


# --------------------------------------------------------------------------
# paths and ids
# --------------------------------------------------------------------------

#: What marks the workspace root: the directory holding this package's source.
_ROOT_MARKER = Path("src") / "f1tenth_logger" / "package.xml"


def find_root(explicit=None, anchors=None):
    """Locate the ``f1tenth_more`` workspace folder.

    Order: explicit argument (the node's ``root`` parameter), then
    ``$F1TENTH_MORE_ROOT`` (test_campaign_logger.launch.py sets both), then an
    upward search for the directory holding ``src/f1tenth_logger/package.xml``.

    The search starts from this file as imported AND as resolved, and from
    the package's install prefix, so it gives the same answer run from
    source, from a colcon install (``install/.../site-packages``) and from a
    ``--symlink-install`` one (``build/f1tenth_logger/...``, which resolves
    into ``src/``). It deliberately does not look for a folder merely NAMED
    ``f1tenth_more``: the workspace holds two of those that are not it
    (``src/f1tenth_more`` and ``install/f1tenth_more``, the f1tenth_more
    package), and the old name-based walk found the first from an installed
    copy. An install outside the workspace finds nothing and says so; set
    F1TENTH_MORE_ROOT there. The current working directory is never
    consulted -- ``ros2 run`` and ``ros2 launch`` both change it.

    ``anchors`` replaces the starting points (for tests).
    """
    if explicit is not None and str(explicit) != "":
        root = Path(explicit).expanduser().resolve()
        if not root.is_dir():
            raise NotADirectoryError(f"root={explicit!r} is not a directory")
        return root

    env = os.environ.get("F1TENTH_MORE_ROOT")
    if env:
        root = Path(env).expanduser().resolve()
        if not root.is_dir():
            raise NotADirectoryError(
                f"F1TENTH_MORE_ROOT={env!r} is not a directory"
            )
        return root

    starts = list(anchors) if anchors is not None else _default_anchors()
    for start in starts:
        start = Path(start)
        for parent in (start, *start.parents):
            if (parent / _ROOT_MARKER).is_file():
                return parent.resolve()

    raise RuntimeError(
        f"cannot locate the {ROOT_NAME!r} workspace: no ancestor of "
        f"{', '.join(str(s) for s in starts)} holds {_ROOT_MARKER}. Pass "
        f"root:=<path> to the node (or --root to the tools), or set "
        f"F1TENTH_MORE_ROOT."
    )


def _default_anchors():
    here = Path(os.path.abspath(__file__))
    anchors = [here] if here.resolve() == here else [here, here.resolve()]
    try:
        from ament_index_python.packages import get_package_prefix
        anchors.append(Path(get_package_prefix("f1tenth_logger")))
    except Exception:  # noqa: BLE001 - not sourced, or not installed: fine
        pass
    return anchors


def make_test_id(prompt_num, repetition, when=None):
    """``P<nnn>-R<rrr>-<YYYYMMDDTHHMMSS>``, e.g. ``P004-R012-20260918T143512``."""
    prompt_num = _check_index(prompt_num, "prompt_num")
    repetition = _check_index(repetition, "repetition")
    when = when or datetime.now()
    return f"P{prompt_num:03d}-R{repetition:03d}-{when.strftime(_TS_FMT)}"


def parse_test_id(test_id):
    """Inverse of :func:`make_test_id`. Raises ``ValueError`` if malformed."""
    match = TEST_ID_RE.match(str(test_id).strip())
    if match is None:
        raise ValueError(
            f"not a test id: {test_id!r} "
            f"(expected P<nnn>-R<rrr>-<YYYYMMDDTHHMMSS>)"
        )
    return {
        "prompt_num": int(match.group(1)),
        "repetition": int(match.group(2)),
        "datetime": datetime.strptime(match.group(3), _TS_FMT),
    }


def plan_hash(plan):
    """sha256 of ``plan`` as canonical JSON: sorted keys, no whitespace.

    Two copies of the same plan hash the same however they were indented,
    so the test folder's ``plan.json`` and the loader's file compare equal
    exactly when their content does.
    """
    canonical = json.dumps(plan, sort_keys=True, separators=(",", ":"),
                           ensure_ascii=False, default=str)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _check_index(value, name):
    try:
        value = int(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must be an integer, got {value!r}") from None
    if not 0 <= value <= 999:
        raise ValueError(f"{name} must be in 0..999, got {value}")
    return value


# --------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------

def _point_segment_distance(px, py, ax, ay, bx, by):
    abx, aby = bx - ax, by - ay
    denom = abx * abx + aby * aby
    if denom <= 0.0:
        return math.hypot(px - ax, py - ay)
    u = ((px - ax) * abx + (py - ay) * aby) / denom
    u = min(1.0, max(0.0, u))
    return math.hypot(px - (ax + u * abx), py - (ay + u * aby))


def _point_in_polygon(px, py, polygon):
    """Ray casting, counting crossings of a horizontal ray to +x."""
    inside = False
    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        if (ay > py) != (by > py):
            x_cross = ax + (py - ay) * (bx - ax) / (by - ay)
            if px < x_cross:
                inside = not inside
    return inside


def signed_clearance(px, py, polygon, robot_radius=0.0):
    """Margin between the robot's edge and the corridor boundary.

    Distance from ``(px, py)`` to the nearest polygon edge, signed positive
    inside and negative outside, minus ``robot_radius``. ``< 0`` means the
    circular footprint touches or crosses the boundary.
    """
    polygon = [(float(x), float(y)) for x, y in polygon]
    if len(polygon) < 3:
        raise ValueError(f"a polygon needs at least 3 vertices, got {len(polygon)}")
    dist = min(
        _point_segment_distance(px, py, *polygon[i], *polygon[(i + 1) % len(polygon)])
        for i in range(len(polygon))
    )
    if not _point_in_polygon(px, py, polygon):
        dist = -dist
    return dist - float(robot_radius)


def corridor_from_centerline(points, width, end_extension=None):
    """Build a corridor polygon by offsetting a centerline by ``+-width/2``.

    Both ends are extended by ``end_extension`` (default: ``width``) *before*
    offsetting. Without that, a robot sitting exactly on the first centerline
    point is flush against the end cap and reads a negative clearance from the
    first sample.

    Returns the left side walked forward followed by the right side reversed,
    so the result is a simple closed polygon in the order the walls appear.
    """
    pts = [(float(x), float(y)) for x, y in points]
    pts = [p for i, p in enumerate(pts) if i == 0 or p != pts[i - 1]]
    if len(pts) < 2:
        raise ValueError("a centerline needs at least 2 distinct points")

    width = float(width)
    if width <= 0.0:
        raise ValueError(f"width must be positive, got {width}")
    ext = width if end_extension is None else float(end_extension)

    if ext > 0.0:
        head = _unit(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1])
        tail = _unit(pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1])
        pts = (
            [(pts[0][0] + head[0] * ext, pts[0][1] + head[1] * ext)]
            + pts
            + [(pts[-1][0] + tail[0] * ext, pts[-1][1] + tail[1] * ext)]
        )

    half = width / 2.0
    left, right = [], []
    for i, (x, y) in enumerate(pts):
        prev = pts[max(i - 1, 0)]
        nxt = pts[min(i + 1, len(pts) - 1)]
        tx, ty = _unit(nxt[0] - prev[0], nxt[1] - prev[1])
        nx, ny = -ty, tx  # left normal of the travel direction
        left.append((x + nx * half, y + ny * half))
        right.append((x - nx * half, y - ny * half))

    return left + right[::-1]


def _unit(dx, dy):
    norm = math.hypot(dx, dy)
    if norm <= 0.0:
        return (1.0, 0.0)
    return (dx / norm, dy / norm)


def _wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# --------------------------------------------------------------------------
# formatting helpers
# --------------------------------------------------------------------------

def _fmt(value):
    """CSV cell: empty for missing, 6 significant digits for floats."""
    if value is None:
        return ""
    if isinstance(value, bool):
        return str(int(value))
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return ""
        return f"{value:.6g}"
    return str(value)


def _round6(value):
    """Same 6-significant-digit contract, for JSON."""
    if value is None:
        return None
    value = float(value)
    if math.isnan(value) or math.isinf(value):
        return None
    return float(f"{value:.6g}")


def _git_commit(root):
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(root),
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if out.returncode != 0:
        return None
    return out.stdout.strip() or None


class _Stream:
    """A file handle plus its flush budget."""

    def __init__(self, path, header=None, immediate=False):
        self.path = path
        self.immediate = immediate
        self.fh = open(path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.fh) if header is not None else None
        self._last_flush = time.monotonic()
        if header is not None:
            self.writer.writerow(header)
            self.fh.flush()

    def row(self, values):
        self.writer.writerow([_fmt(v) for v in values])
        self._maybe_flush()

    def line(self, obj):
        self.fh.write(json.dumps(obj, ensure_ascii=False, default=str) + "\n")
        self._maybe_flush()

    def _maybe_flush(self):
        now = time.monotonic()
        if self.immediate or now - self._last_flush >= _FLUSH_PERIOD_S:
            self.fh.flush()
            self._last_flush = now

    def close(self):
        try:
            self.fh.flush()
            self.fh.close()
        except (OSError, ValueError):
            pass


class _LLMCallHandle:
    """Yielded by :meth:`TestLogger.llm_call`."""

    def __init__(self, call_idx, tag, prompt, mono_start):
        self.call_idx = call_idx
        self.tag = tag
        self.prompt = prompt
        self.response = None
        self.ttft_ms = None
        self._mono_start = mono_start

    def set_response(self, text):
        """Record the model's answer. Safe to call more than once."""
        self.response = text
        return text

    def first_token(self):
        """Mark the arrival of the first streamed token; returns ttft in ms."""
        if self.ttft_ms is None:
            self.ttft_ms = (time.monotonic() - self._mono_start) * 1000.0
        return self.ttft_ms


# --------------------------------------------------------------------------
# the logger
# --------------------------------------------------------------------------

class TestLogger:
    """Log one test run into its own folder.

    Parameters
    ----------
    prompt:
        Either the ``prompt_num`` from ``prompts.yaml`` or the exact prompt
        text. Anything not in the table is an error -- folder names are never
        derived from free text.
    root:
        The ``f1tenth_more`` folder; see :func:`find_root`.
    campaign:
        Campaign folder name under ``root``.
    robot_radius:
        Footprint radius in metres, used for every clearance.
    mpc_ok_statuses:
        Solver status strings that count as solved. Everything else --
        ``infeasible``, ``max_iter``, ``timeout``, ``solved_inaccurate`` --
        counts as not solved. The set is written to ``meta.json`` so the
        export computes feasibility with the same rule the run used.
    extra_meta:
        Copied verbatim into ``meta.json``. ``robot``/``robot_name`` and
        ``llm_model`` from it seed ``campaign.json`` when that file is created.
    """

    #: This is not a pytest test class, despite the name (pytest collects
    #: anything called Test*; without this it warns on every run).
    __test__ = False

    def __init__(
        self,
        prompt,
        root=None,
        campaign=DEFAULT_CAMPAIGN,
        robot_radius=0.0,
        extra_meta=None,
        mpc_ok_statuses=("solved",),
    ):
        self._lock = threading.RLock()
        self._finished = False
        self._closed = False

        self.root = find_root(root)
        self.campaign_dir = self.root / str(campaign)
        self.robot_radius = float(robot_radius)
        self.extra_meta = dict(extra_meta or {})
        self.mpc_ok_statuses = frozenset(str(x) for x in mpc_ok_statuses)
        if not self.mpc_ok_statuses:
            raise ValueError("mpc_ok_statuses must name at least one status")

        self.campaign_dir.mkdir(parents=True, exist_ok=True)
        self.git_commit = _git_commit(self.root)
        self._ensure_campaign_json()

        entry = self._lookup_prompt(prompt)
        self.prompt_num = int(entry["prompt_num"])
        self.mission = str(entry["mission"])
        self.prompt_text = str(entry["text"])
        self.success_criterion = str(entry.get("success_criterion", "") or "")
        self.prompt_hash = hashlib.sha256(
            self.prompt_text.encode("utf-8")
        ).hexdigest()

        self.mission_dir = self.campaign_dir / self.mission
        self.mission_dir.mkdir(parents=True, exist_ok=True)
        self._ensure_mission_json()

        self.prompt_changed = self._check_prompt_hash()

        # Wall clock for the record, monotonic for every t.
        self._start_wall = datetime.now()
        self._t0 = time.monotonic()
        self.repetition, self.test_id, self.dir = self._create_test_dir()

        self._streams = {
            "kinematics": _Stream(self.dir / "kinematics.csv", _KINEMATICS_COLS),
            "imu": _Stream(self.dir / "imu.csv", _IMU_COLS),
            "commands": _Stream(self.dir / "commands.csv", _COMMAND_COLS),
            "mpc": _Stream(self.dir / "mpc.csv", _MPC_COLS),
            "llm": _Stream(self.dir / "llm_calls.csv", _LLM_COLS, immediate=True),
            "llm_jsonl": _Stream(self.dir / "llm_calls.jsonl", immediate=True),
            "corridors": _Stream(self.dir / "corridors.jsonl", immediate=True),
            "events": _Stream(self.dir / "events.jsonl"),
        }

        # running state for finite differences
        self._last_pose = None      # (t, x, y)
        self._last_vel = None       # (t, vx, vy)
        self._last_yaw = None       # (t, yaw)
        self._corridor = None       # current polygon
        self.last_corridor_clearance = None

        # accumulators for the summary
        self._n_pose = 0
        self._n_imu = 0
        self._n_cmd = 0
        self._span = {}             # stream -> [t_first, t_last]
        self._path_length = 0.0
        self._max_speed = None
        self._max_acc = None
        self._min_corr_clear = None
        self._min_obst_clear = None
        self._max_imu_horiz_acc = None
        self._max_abs_yaw_rate = None
        self._n_corridors = 0
        self._llm_latencies = []
        self._n_llm = 0
        self._n_mpc = 0
        self._n_events = 0
        self._plans = []            # one record per save_plan(), for meta.json

        self.summary = None
        self.auto_outcome = None
        self.reason = ""
        self.duration_s = None
        self._end_wall = None

        atexit.register(self._atexit)
        self.log_event(
            "test_start",
            mission=self.mission,
            prompt_num=self.prompt_num,
            repetition=self.repetition,
            prompt_hash=self.prompt_hash,
        )

    # -- campaign / mission bookkeeping ------------------------------------

    def _ensure_campaign_json(self):
        path = self.campaign_dir / "campaign.json"
        if path.exists():
            return
        meta = self.extra_meta
        payload = {
            "created": datetime.now().isoformat(timespec="seconds"),
            "robot": meta.get("robot", meta.get("robot_name"))
            or os.environ.get("F1TENTH_ROBOT_NAME"),
            "llm_model": meta.get("llm_model")
            or os.environ.get("F1TENTH_LLM_MODEL"),
            "git_commit": self.git_commit,
        }
        _write_json_atomic(path, payload)

    def _lookup_prompt(self, prompt):
        path = self.campaign_dir / "prompts.yaml"
        if not path.exists():
            raise FileNotFoundError(
                f"no prompt table at {path}. Write it first: a list of "
                f"{{prompt_num, mission, text, success_criterion}} entries."
            )
        with open(path, encoding="utf-8") as fh:
            table = yaml.safe_load(fh)
        if not isinstance(table, list) or not table:
            raise ValueError(f"{path} must hold a non-empty list of prompt entries")

        for entry in table:
            if not isinstance(entry, dict):
                raise ValueError(f"{path}: every entry must be a mapping, got {entry!r}")
            for key in ("prompt_num", "mission", "text"):
                if key not in entry:
                    raise ValueError(f"{path}: entry {entry!r} is missing {key!r}")

        if isinstance(prompt, bool):
            raise TypeError("prompt must be a prompt_num or the prompt text")
        if isinstance(prompt, int):
            for entry in table:
                if int(entry["prompt_num"]) == prompt:
                    return entry
            known = sorted(int(e["prompt_num"]) for e in table)
            raise KeyError(f"prompt_num {prompt} is not in {path} (known: {known})")

        text = str(prompt)
        for entry in table:
            if str(entry["text"]) == text:
                return entry
        raise KeyError(
            f"this exact prompt text is not in {path}; add it to the table "
            f"rather than logging an untabulated prompt:\n  {text!r}"
        )

    def _ensure_mission_json(self):
        path = self.mission_dir / "mission.json"
        if path.exists():
            return
        _write_json_atomic(
            path,
            {
                "mission": self.mission,
                "prompt_num": self.prompt_num,
                "prompt_text": self.prompt_text,
                "success_criterion": self.success_criterion,
            },
        )

    def _existing_tests(self):
        """(repetition, dir) for every valid test folder of this prompt."""
        found = []
        for child in self.mission_dir.iterdir():
            if not child.is_dir():
                continue
            try:
                parsed = parse_test_id(child.name)
            except ValueError:
                continue
            if parsed["prompt_num"] == self.prompt_num:
                found.append((parsed["repetition"], child))
        return found

    def _check_prompt_hash(self):
        """Warn if this prompt's text changed since an earlier repetition."""
        for _, child in self._existing_tests():
            meta_path = child / "meta.json"
            if not meta_path.exists():
                continue
            try:
                with open(meta_path, encoding="utf-8") as fh:
                    earlier = json.load(fh)
            except (OSError, json.JSONDecodeError):
                continue
            previous = (earlier.get("summary") or {}).get("prompt_hash")
            if previous and previous != self.prompt_hash:
                print(
                    f"WARNING: prompt {self.prompt_num} text changed since "
                    f"{child.name} ({previous[:12]} -> {self.prompt_hash[:12]}); "
                    f"repetitions of this prompt are no longer comparable",
                    file=sys.stderr,
                )
                return True
        return False

    def _create_test_dir(self):
        """Next repetition, created with exist_ok=False. Never overwrites."""
        existing = self._existing_tests()
        rep = max((r for r, _ in existing), default=0) + 1
        for _ in range(1000):
            if rep > 999:
                raise RuntimeError(
                    f"prompt {self.prompt_num} already has repetition 999 in "
                    f"{self.mission_dir}; start a new campaign"
                )
            test_id = make_test_id(self.prompt_num, rep, self._start_wall)
            path = self.mission_dir / test_id
            try:
                path.mkdir(exist_ok=False)
            except FileExistsError:
                # another process took this repetition; step over it
                rep += 1
                continue
            return rep, test_id, path
        raise RuntimeError(f"could not allocate a test folder in {self.mission_dir}")

    # -- clock --------------------------------------------------------------

    def now(self):
        """Seconds since the test started."""
        return time.monotonic() - self._t0

    def _stamp(self, t):
        return self.now() if t is None else float(t)

    def _note(self, stream, t):
        span = self._span.get(stream)
        if span is None:
            self._span[stream] = [t, t]
        else:
            span[1] = t

    # -- logging ------------------------------------------------------------

    def log_state(
        self,
        x,
        y,
        yaw=None,
        vx=None,
        vy=None,
        ax=None,
        ay=None,
        yaw_rate=None,
        obstacle_clearance=None,
        t=None,
    ):
        """Log one pose sample, filling in what the estimator did not provide.

        ``obstacle_clearance`` is measured by perception from the robot's
        **edge** to the nearest obstacle, so it is negative on contact.
        """
        with self._lock:
            if self._closed:
                return None
            t = self._stamp(t)
            x, y = float(x), float(y)

            if self._last_pose is not None:
                t_prev, x_prev, y_prev = self._last_pose
                dt = t - t_prev
                step = math.hypot(x - x_prev, y - y_prev)
                self._path_length += step
                if dt > 0.0:
                    if vx is None:
                        vx = (x - x_prev) / dt
                    if vy is None:
                        vy = (y - y_prev) / dt
            self._last_pose = (t, x, y)

            vx = None if vx is None else float(vx)
            vy = None if vy is None else float(vy)
            speed = None
            if vx is not None and vy is not None:
                speed = math.hypot(vx, vy)

            if vx is not None and vy is not None:
                if self._last_vel is not None:
                    t_prev, vx_prev, vy_prev = self._last_vel
                    dt = t - t_prev
                    if dt > 0.0:
                        if ax is None:
                            ax = (vx - vx_prev) / dt
                        if ay is None:
                            ay = (vy - vy_prev) / dt
                self._last_vel = (t, vx, vy)

            ax = None if ax is None else float(ax)
            ay = None if ay is None else float(ay)
            acc = None
            if ax is not None and ay is not None:
                acc = math.hypot(ax, ay)

            if yaw is not None:
                yaw = float(yaw)
                if yaw_rate is None and self._last_yaw is not None:
                    t_prev, yaw_prev = self._last_yaw
                    dt = t - t_prev
                    if dt > 0.0:
                        # wrap the difference, not the angles: the +-pi
                        # crossing must not read as a huge rate
                        yaw_rate = _wrap_pi(yaw - yaw_prev) / dt
                self._last_yaw = (t, yaw)
            yaw_rate = None if yaw_rate is None else float(yaw_rate)

            corridor_clearance = None
            if self._corridor is not None:
                corridor_clearance = signed_clearance(
                    x, y, self._corridor, self.robot_radius
                )
            self.last_corridor_clearance = corridor_clearance

            obstacle_clearance = (
                None if obstacle_clearance is None else float(obstacle_clearance)
            )

            self._max_speed = _max_opt(self._max_speed, speed)
            self._max_acc = _max_opt(self._max_acc, acc)
            self._min_corr_clear = _min_opt(self._min_corr_clear, corridor_clearance)
            self._min_obst_clear = _min_opt(self._min_obst_clear, obstacle_clearance)
            if yaw_rate is not None:
                self._max_abs_yaw_rate = _max_opt(
                    self._max_abs_yaw_rate, abs(yaw_rate)
                )

            self._n_pose += 1
            self._note("kinematics", t)
            self._streams["kinematics"].row(
                [t, x, y, yaw, yaw_rate, vx, vy, speed, ax, ay, acc,
                 corridor_clearance, obstacle_clearance]
            )
            return corridor_clearance

    def log_imu(
        self, ax, ay, az, gx, gy, gz, imu_yaw=None, sensor_stamp=None, t=None
    ):
        """Log one IMU message. Call for every message -- no downsampling."""
        with self._lock:
            if self._closed:
                return
            t = self._stamp(t)
            ax, ay, az = float(ax), float(ay), float(az)
            gx, gy, gz = float(gx), float(gy), float(gz)

            self._max_imu_horiz_acc = _max_opt(
                self._max_imu_horiz_acc, math.hypot(ax, ay)
            )
            self._max_abs_yaw_rate = _max_opt(self._max_abs_yaw_rate, abs(gz))

            self._n_imu += 1
            self._note("imu", t)
            self._streams["imu"].row(
                [t, sensor_stamp, ax, ay, az, gx, gy, gz, imu_yaw]
            )

    def log_command(
        self,
        cmd_speed=None,
        cmd_steer=None,
        cmd_yaw_rate=None,
        cmd_throttle=None,
        cmd_brake=None,
        source="controller",
        t=None,
    ):
        """Log one actuation command (source: controller / teleop / estop / ...)."""
        with self._lock:
            if self._closed:
                return
            t = self._stamp(t)
            self._n_cmd += 1
            self._note("commands", t)
            self._streams["commands"].row(
                [t, cmd_speed, cmd_steer, cmd_yaw_rate, cmd_throttle, cmd_brake,
                 source]
            )

    def log_mpc(self, status, solve_time_ms=None, cost=None, iterations=None,
                t=None):
        """Log one MPC solve. Call after **every** solve, solved or not.

        ``status`` is the solver's own status string; whether it counts as
        solved is decided by ``mpc_ok_statuses``, not by this method.
        """
        with self._lock:
            if self._closed:
                return
            t = self._stamp(t)
            self._n_mpc += 1
            self._note("mpc", t)
            self._streams["mpc"].row(
                [t, status, solve_time_ms, cost, iterations]
            )

    def record_llm_call(self, prompt, response=None, tag="", t_sent=None,
                        t_received=None, latency_ms=None, ttft_ms=None,
                        ok=1, error="", llm_raw=None, rejections=None,
                        translated_plan=None, plan_file=None, plan_hash=None):
        """Record a call **someone else** timed, e.g. the planner node.

        :meth:`llm_call` is for code that makes the request itself. When the
        request happened in another process and arrived as a message, this
        writes the same row from the numbers that message carried.
        ``t_sent``/``t_received`` are in this test's time base and may be
        negative -- the prompt is normally sent before the test folder exists.

        ``llm_raw`` (what the model wrote), ``rejections`` (the attempts the
        planner refused before this one) and ``translated_plan`` (the
        translator's output) become their own fields in ``llm_calls.jsonl``;
        ``plan_file``/``plan_hash`` name the copy :meth:`save_plan` wrote.
        With a ``translated_plan``, ``response`` is kept only for older
        readers and marked deprecated in the record.
        """
        with self._lock:
            if self._closed:
                return None
            call_idx = self._n_llm
            self._n_llm += 1
            if t_sent is None:
                t_sent = self.now()
            if t_received is None:
                t_received = self.now()
            if latency_ms is None:
                latency_ms = (float(t_received) - float(t_sent)) * 1000.0
            latency_ms = float(latency_ms)
            self._llm_latencies.append(latency_ms)
            self._streams["llm"].row(
                [
                    call_idx,
                    tag,
                    t_sent,
                    t_received,
                    latency_ms,
                    ttft_ms,
                    len(prompt) if prompt is not None else 0,
                    len(response) if response is not None else 0,
                    ok,
                    error,
                ]
            )
            record = {
                "call_idx": call_idx,
                "tag": tag,
                "t_sent": _round6(t_sent),
                "t_received": _round6(t_received),
                "latency_ms": _round6(latency_ms),
                "ttft_ms": _round6(ttft_ms),
                "ok": ok,
                "error": error,
                "prompt": prompt,
                "response": response,
                "llm_raw": llm_raw,
                "rejections": list(rejections or []),
                "translated_plan": translated_plan,
                "plan_file": plan_file,
                "plan_hash": plan_hash,
            }
            if translated_plan is not None:
                record["deprecated"] = {"response": "translated_plan"}
            self._streams["llm_jsonl"].line(record)
            return call_idx

    def save_plan(self, plan, tag="initial", reference_path=None, plan_id=None):
        """Write the plan into the test folder and check it against the loader's.

        ``plan.json`` for the initial plan, ``plan_replan_<N>.json`` for the
        N-th replan, in the planner's own format (2-space JSON). The hash is
        :func:`plan_hash`. ``reference_path`` is the file the mission loader
        was given; if it cannot be read, or hashes differently, a
        ``plan_file_mismatch`` event records both sides. Returns the record
        that also lands in ``meta.json`` under ``plans``, or None once closed.
        """
        with self._lock:
            if self._closed:
                return None
            if tag == "replan":
                n = 1 + sum(1 for p in self._plans if p["tag"] == "replan")
                name = f"plan_replan_{n}.json"
            else:
                name = "plan.json"
                if (self.dir / name).exists():
                    # a second 'initial' plan inside one test never overwrites
                    n = 1 + sum(1 for p in self._plans if p["file"].startswith("plan_initial_"))
                    name = f"plan_initial_{n}.json"
            _write_json_atomic(self.dir / name, plan)
            digest = plan_hash(plan)

            reference_hash, problem = None, None
            if reference_path:
                try:
                    with open(reference_path, encoding="utf-8") as fh:
                        reference_hash = plan_hash(json.load(fh))
                except FileNotFoundError:
                    problem = "reference file not found"
                except (OSError, json.JSONDecodeError) as exc:
                    problem = f"reference file unreadable: {type(exc).__name__}: {exc}"
                else:
                    if reference_hash != digest:
                        problem = "content differs"
            else:
                problem = "no reference file to compare with"

            record = {
                "tag": tag,
                "file": name,
                "plan_id": plan_id,
                "plan_hash": digest,
                "reference": None if reference_path is None else str(reference_path),
                "reference_hash": reference_hash,
                "match": problem is None,
            }
            self._plans.append(record)
            if problem is not None:
                self.log_event(
                    "plan_file_mismatch",
                    file=name,
                    plan_id=plan_id,
                    plan_hash=digest,
                    reference=record["reference"],
                    reference_hash=reference_hash,
                    reason=problem,
                )
            return record

    @contextmanager
    def llm_call(self, prompt, tag=""):
        """Time one LLM request. Wrap **only** the request itself.

        ``with logger.llm_call(prompt, tag="plan") as call:`` yields a handle
        with ``set_response(text)`` and ``first_token()``. An exception inside
        the block is recorded as ``ok=0`` with its message, then re-raised.
        """
        with self._lock:
            call_idx = self._n_llm
            self._n_llm += 1
        mono_start = time.monotonic()
        t_sent = self.now()
        handle = _LLMCallHandle(call_idx, tag, prompt, mono_start)
        ok, error = 1, ""
        try:
            yield handle
        except BaseException as exc:  # noqa: BLE001 - recorded, then re-raised
            ok = 0
            error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            latency_ms = (time.monotonic() - mono_start) * 1000.0
            t_received = self.now()
            response = handle.response
            with self._lock:
                if not self._closed:
                    self._llm_latencies.append(latency_ms)
                    self._streams["llm"].row(
                        [
                            call_idx,
                            tag,
                            t_sent,
                            t_received,
                            latency_ms,
                            handle.ttft_ms,
                            len(prompt) if prompt is not None else 0,
                            len(response) if response is not None else 0,
                            ok,
                            error,
                        ]
                    )
                    self._streams["llm_jsonl"].line(
                        {
                            "call_idx": call_idx,
                            "tag": tag,
                            "t_sent": _round6(t_sent),
                            "t_received": _round6(t_received),
                            "latency_ms": _round6(latency_ms),
                            "ttft_ms": _round6(handle.ttft_ms),
                            "ok": ok,
                            "error": error,
                            "prompt": prompt,
                            "response": response,
                        }
                    )

    def log_corridor(self, polygon, corridor_id=None, source="llm", meta=None, t=None):
        """Record a corridor and make it the current one for clearance."""
        poly = [(float(px), float(py)) for px, py in polygon]
        if len(poly) < 3:
            raise ValueError(f"a corridor needs at least 3 vertices, got {len(poly)}")
        with self._lock:
            if self._closed:
                return None
            t = self._stamp(t)
            if corridor_id is None:
                corridor_id = self._n_corridors
            self._corridor = poly
            self._n_corridors += 1
            self._streams["corridors"].line(
                {
                    "t": _round6(t),
                    "id": corridor_id,
                    "source": source,
                    "polygon": [[_round6(px), _round6(py)] for px, py in poly],
                    "meta": meta or {},
                }
            )
            if self._last_pose is not None:
                _, x, y = self._last_pose
                self.last_corridor_clearance = signed_clearance(
                    x, y, poly, self.robot_radius
                )
            return corridor_id

    def log_event(self, name, t=None, **data):
        """Free-form event, one JSON object per line.

        Two names are reserved and read by the export: ``"contact"`` (the
        robot touched something) and ``"estop"`` (emergency stop), both of
        which should carry ``cause="..."``.
        """
        with self._lock:
            if self._closed:
                return
            t = self._stamp(t)
            record = {"t": _round6(t), "event": str(name)}
            record.update(data)
            self._n_events += 1
            self._streams["events"].line(record)

    def stream_counts(self):
        """Rows written so far, per stream -- the recorder's status line."""
        with self._lock:
            return {
                "kinematics": self._n_pose,
                "imu": self._n_imu,
                "commands": self._n_cmd,
                "mpc": self._n_mpc,
                "corridors": self._n_corridors,
                "llm_calls": self._n_llm,
                "events": self._n_events,
            }

    # -- teardown -----------------------------------------------------------

    def finish(self, outcome, reason=""):
        """Close the run. ``outcome`` is ``"completed"`` or ``"aborted"``.

        This is only the **automatic hint**: it lands in ``meta.json`` and
        ``results.csv`` as ``auto_outcome`` / ``auto_success``. The verdict
        that counts is the ``success`` column a human fills in
        ``campaign_results.csv``.

        Idempotent: later calls (including the atexit hook) are ignored.
        """
        with self._lock:
            if self._finished:
                return self.summary
            if outcome not in ("completed", "aborted"):
                raise ValueError(
                    f"outcome must be 'completed' or 'aborted', got {outcome!r}"
                )
            self._finished = True
            self.auto_outcome = outcome
            self.reason = str(reason or "")
            self._end_wall = datetime.now()
            self.duration_s = self.now()

            self.log_event("test_end", auto_outcome=outcome, reason=self.reason)

            summary = self._build_summary()
            self.summary = summary
            meta = {
                "summary": summary,
                "prompt_text": self.prompt_text,
                "success_criterion": self.success_criterion,
                "robot_radius": self.robot_radius,
                "start_time": self._start_wall.isoformat(timespec="microseconds"),
                "end_time": self._end_wall.isoformat(timespec="microseconds"),
                "prompt_changed": self.prompt_changed,
                "mpc_ok_statuses": sorted(self.mpc_ok_statuses),
                "extra_meta": self.extra_meta,
                "campaign_dir": str(self.campaign_dir),
                "plans": list(self._plans),
            }
            meta.update({k: summary[k] for k in SUMMARY_FIELDS})
            _write_json_atomic(self.dir / "meta.json", meta)
            self._append_results_row(summary)

            self._closed = True
            for stream in self._streams.values():
                stream.close()
            try:
                atexit.unregister(self._atexit)
            except Exception:  # pragma: no cover - unregister is best effort
                pass
            return summary

    def _build_summary(self):
        latencies = list(self._llm_latencies)
        return {
            "test_id": self.test_id,
            "mission": self.mission,
            "prompt_num": self.prompt_num,
            "repetition": self.repetition,
            "prompt_hash": self.prompt_hash,
            "git_commit": self.git_commit,
            "start_time": self._start_wall.isoformat(timespec="microseconds"),
            "duration_s": _round6(self.duration_s),
            "auto_outcome": self.auto_outcome,
            "auto_success": 1 if self.auto_outcome == "completed" else 0,
            "reason": self.reason,
            "n_llm_calls": len(latencies),
            "llm_latency_mean_ms": _round6(
                sum(latencies) / len(latencies) if latencies else None
            ),
            "llm_latency_max_ms": _round6(max(latencies) if latencies else None),
            "n_corridors": self._n_corridors,
            "min_corridor_clearance": _round6(self._min_corr_clear),
            "min_obstacle_clearance": _round6(self._min_obst_clear),
            "max_speed": _round6(self._max_speed),
            "max_acc": _round6(self._max_acc),
            "path_length": _round6(self._path_length),
            "max_imu_horiz_acc": _round6(self._max_imu_horiz_acc),
            "max_abs_yaw_rate": _round6(self._max_abs_yaw_rate),
            "pose_rate_hz": _round6(self._rate("kinematics", self._n_pose)),
            "imu_rate_hz": _round6(self._rate("imu", self._n_imu)),
            "cmd_rate_hz": _round6(self._rate("commands", self._n_cmd)),
            "n_imu": self._n_imu,
            "n_cmd": self._n_cmd,
        }

    def _rate(self, stream, count):
        """(n-1) / (t_last - t_first), from the stream's own timestamps."""
        span = self._span.get(stream)
        if span is None or count < 2:
            return None
        elapsed = span[1] - span[0]
        if elapsed <= 0.0:
            return None
        return (count - 1) / elapsed

    def _append_results_row(self, summary):
        path = self.campaign_dir / "results.csv"
        new = not path.exists() or path.stat().st_size == 0
        with open(path, "a", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            if new:
                writer.writerow(SUMMARY_FIELDS)
            writer.writerow([_fmt(summary[k]) for k in SUMMARY_FIELDS])
            fh.flush()

    def _atexit(self):
        if not self._finished:
            self.finish("aborted", "process exited without an outcome")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc_type is not None:
            if not self._finished:
                self.finish("aborted", f"exception: {exc_type.__name__}: {exc}")
            return False  # re-raise
        if not self._finished:
            self.finish("aborted", "no outcome recorded")
        return False

    def __repr__(self):
        state = self.auto_outcome or ("open" if not self._finished else "closed")
        return f"<TestLogger {self.test_id} {self.mission} {state}>"


def _write_json_atomic(path, payload):
    tmp = Path(str(path) + ".tmp")
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, ensure_ascii=False, default=str)
        fh.write("\n")
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def _max_opt(current, value):
    if value is None:
        return current
    return value if current is None else max(current, value)


def _min_opt(current, value):
    if value is None:
        return current
    return value if current is None else min(current, value)
