"""A campaign test folder -> per-topic structs. The primary source.

Everything the test-campaign logger wrote for one test, for the whole test
(pre-roll, drive, post-roll). Times in the folder are seconds since the test
opened; ``meta.json``'s ``extra_meta.ros_start_time`` is that instant in ROS
time, so ``t_ros = ros_start_time + t``. ``t_rel`` is measured from the
drive start: the ``mission_started`` event, or the first ``commands.csv``
row when a test has no such event.

``time_source`` per topic: kinematics, imu and commands carry the message
header stamp (the logger falls back to its clock only for a stamp more than
5 s off); everything else is stamped with the logger's clock on receipt.
"""

import csv
import json
import math

import numpy as np

from f1tenth_logger.matlab_export.convert import (
    cell, column, is_number, mat_name, records_to_struct, str_cell)

SOURCE = "campaign"
RESERVED = ("source", "time_source", "topic", "t_ros", "t_rel", "info")

#: struct name -> (file, time_source)
CSV_TOPICS = {
    "kin": ("kinematics.csv", "header"),
    "imu": ("imu.csv", "header"),
    "cmd": ("commands.csv", "header"),
    "mpc": ("mpc.csv", "receive"),
}
#: struct name -> (file, time_source)
JSONL_TOPICS = {
    "horizon": ("horizon.jsonl", "receive"),
    "corridor": ("corridors.jsonl", "receive"),
    "corridor_dbg": ("corridor_debug.jsonl", "receive"),
    "tracks": ("tracks.jsonl", "receive"),
}


def read_jsonl(path):
    records = []
    if not path.exists():
        return records
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(record, dict):
                records.append(record)
    return records


def read_csv(path):
    """``{column: list of raw strings}`` in file order; {} if absent."""
    if not path.exists():
        return {}
    with open(path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        rows = list(reader)
        names = reader.fieldnames or []
    return {name: [row.get(name) for row in rows] for name in names}


def _parse(text):
    if text is None or text.strip() == "":
        return None
    try:
        return float(text)
    except ValueError:
        return text


def csv_column(raw):
    """Raw strings -> double column (empty = NaN) or, if any is text, a cell."""
    values = [_parse(v) for v in raw]
    if all(v is None or is_number(v) for v in values):
        return column(values)
    return str_cell(["" if v is None else str(v) for v in raw])


def times(values):
    return np.array([math.nan if v is None else float(v) for v in values], dtype=float)


def make_topic(name, t_test, fields, timing, time_source, topic, source=SOURCE, info=None):
    """The common topic layout: source, time_source, topic, t_ros, t_rel, fields."""
    t_test = np.asarray(t_test, dtype=float).reshape(-1, 1)
    out = {
        "source": source,
        "time_source": time_source,
        "topic": topic,
        "t_ros": timing["ros_start"] + t_test,
        "t_rel": timing["ros_start"] + t_test - timing["drive_start_ros"],
    }
    for key, value in fields.items():
        key = f"{name}_{key}" if key in RESERVED else key
        out[mat_name(key, out)] = value
    if info:
        out["info"] = info
    return out


def load_meta(test_dir):
    try:
        with open(test_dir / "meta.json", encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, json.JSONDecodeError):
        return {}


def first_event_times(events):
    first = {}
    for event in events:
        name = event.get("event")
        t = event.get("t")
        if name and is_number(t) and name not in first:
            first[name] = float(t)
    return first


def timing_for(test_dir, meta=None, events=None):
    """The test's time anchors, all in seconds.

    ``ros_start`` (ROS time of the test's t = 0), ``drive_start_ros`` and
    the source it came from, ``drive_end_rel`` and ``test_end_rel``.
    """
    meta = load_meta(test_dir) if meta is None else meta
    events = read_jsonl(test_dir / "events.jsonl") if events is None else events
    extra = meta.get("extra_meta") or {}
    ros_start = extra.get("ros_start_time")
    if not is_number(ros_start):
        raise ValueError(f"{test_dir.name}: meta.json has no extra_meta.ros_start_time")
    first = first_event_times(events)
    if "mission_started" in first:
        drive_start, how = first["mission_started"], "mission_started"
    else:
        cmd_t = [v for v in (_parse(x) for x in read_csv(test_dir / "commands.csv").get("t", []))
                 if is_number(v)]
        if cmd_t:
            drive_start, how = cmd_t[0], "first_cmd"
        else:
            drive_start, how = 0.0, "test_start"
    end = first.get("mission_finished", first.get("test_end"))
    return {
        "ros_start": float(ros_start),
        "drive_start_ros": float(ros_start) + drive_start,
        "drive_start_source": how,
        "drive_end_rel": math.nan if end is None else end - drive_start,
        "test_end_rel": first.get("test_end", math.nan) - drive_start
        if "test_end" in first else math.nan,
        "events": first,
    }


def csv_topic(test_dir, name, filename, time_source, timing):
    raw = read_csv(test_dir / filename)
    if "t" not in raw:
        return None
    fields = {}
    for key, values in raw.items():
        if key == "t":
            continue
        if name == "cmd" and key.startswith("cmd_"):
            key = key[4:]
        fields[key] = csv_column(values)
    return make_topic(name, times(_parse(v) for v in raw["t"]), fields, timing,
                      time_source, filename)


def jsonl_topic(test_dir, name, filename, time_source, timing):
    records = read_jsonl(test_dir / filename)
    if not records:
        return None
    fields = records_to_struct(records, skip=("t",))
    return make_topic(name, times(r.get("t") for r in records), fields, timing,
                      time_source, filename)


def events_topic(events, timing):
    if not events:
        return None
    rest = [json.dumps({k: v for k, v in e.items() if k not in ("t", "event")}) for e in events]
    return make_topic("events", times(e.get("t") for e in events),
                      {"event": str_cell([e.get("event") for e in events]),
                       "fields_json": str_cell(rest)},
                      timing, "receive", "events.jsonl")


def llm_topic(test_dir, timing):
    records = read_jsonl(test_dir / "llm_calls.jsonl")
    if not records:
        return None
    fields = records_to_struct(records, skip=("t_sent",))
    fields["translated_plan_json"] = str_cell(
        [json.dumps(r.get("translated_plan")) if r.get("translated_plan") is not None else ""
         for r in records])
    return make_topic("llm", times(r.get("t_sent") for r in records), fields, timing,
                      "receive", "llm_calls.jsonl")


def plan_topic(test_dir, meta, timing):
    """Every plan file meta.json lists (initial + replans), as JSON text."""
    plans = meta.get("plans") or [{"tag": "initial", "file": "plan.json"}]
    rows = []
    for entry in plans:
        path = test_dir / str(entry.get("file") or "")
        if not path.is_file():
            continue
        rows.append((entry, path.read_text(encoding="utf-8")))
    if not rows:
        return None
    loaded = timing["events"].get("mission_loaded")
    t = [loaded if (e.get("tag") == "initial" and loaded is not None) else None
         for e, _ in rows]
    fields = {
        "tag": str_cell([e.get("tag") for e, _ in rows]),
        "file": str_cell([e.get("file") for e, _ in rows]),
        "plan_id": str_cell([e.get("plan_id") for e, _ in rows]),
        "plan_hash": str_cell([e.get("plan_hash") for e, _ in rows]),
        "text": cell([text for _, text in rows]),
    }
    return make_topic("plan", times(t), fields, timing, "receive", "plan.json")


def load_campaign_topics(test_dir):
    """``(topics, timing, meta)`` for one test folder. Missing files are skipped."""
    meta = load_meta(test_dir)
    events = read_jsonl(test_dir / "events.jsonl")
    timing = timing_for(test_dir, meta, events)
    topics = {}
    for name, (filename, tsrc) in CSV_TOPICS.items():
        topic = csv_topic(test_dir, name, filename, tsrc, timing)
        if topic is not None:
            topics[name] = topic
    for name, (filename, tsrc) in JSONL_TOPICS.items():
        topic = jsonl_topic(test_dir, name, filename, tsrc, timing)
        if topic is not None:
            topics[name] = topic
    for name, topic in (("events", events_topic(events, timing)),
                        ("llm", llm_topic(test_dir, timing)),
                        ("plan", plan_topic(test_dir, meta, timing))):
        if topic is not None:
            topics[name] = topic
    return topics, timing, meta
