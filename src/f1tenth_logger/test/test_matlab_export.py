"""export_matlab: flattening on a tiny synthetic bag, the db root, a whole run.

The bag is written with rosbags in a temp dir; no ROS runtime is involved.
"""

import csv
import json
import os
from pathlib import Path

import numpy as np
import pytest

rosbags = pytest.importorskip("rosbags")
scipy_io = pytest.importorskip("scipy.io")

from f1tenth_logger.matlab_export import bag as bagmod  # noqa: E402
from f1tenth_logger.matlab_export import convert, db_root, exporter  # noqa: E402
from f1tenth_logger.matlab_export import EXPORTER_VERSION  # noqa: E402
from f1tenth_logger.test_campaign.robot_logger import find_root  # noqa: E402

T0 = 1_790_000_000.0          # ROS time of the test's t = 0
DRIVE = 3.0                   # mission_started, test time


@pytest.fixture(scope="module")
def store():
    return bagmod.make_typestore(find_root())[0]


def _t(store, seconds):
    sec = int(seconds)
    return store.types["builtin_interfaces/msg/Time"](
        sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


def _hdr(store, seconds, frame="odom"):
    return store.types["std_msgs/msg/Header"](stamp=_t(store, seconds), frame_id=frame)


def _pose(store, x, y, yaw):
    T = store.types
    return T["geometry_msgs/msg/Pose"](
        position=T["geometry_msgs/msg/Point"](x=x, y=y, z=0.0),
        orientation=T["geometry_msgs/msg/Quaternion"](
            x=0.0, y=0.0, z=float(np.sin(yaw / 2)), w=float(np.cos(yaw / 2))))


def _vec(store, x=0.0, y=0.0, z=0.0):
    return store.types["geometry_msgs/msg/Vector3"](x=x, y=y, z=z)


def make_messages(store):
    """``[(topic, msgtype, receive_s, msg)]`` covering every extractor kind."""
    T = store.types
    out = []
    for i in range(3):
        t = T0 + DRIVE + 0.1 * i
        odom = T["nav_msgs/msg/Odometry"](
            header=_hdr(store, t), child_frame_id="base_link",
            pose=T["geometry_msgs/msg/PoseWithCovariance"](
                pose=_pose(store, 1.0 * i, 2.0, 0.5), covariance=np.zeros(36)),
            twist=T["geometry_msgs/msg/TwistWithCovariance"](
                twist=T["geometry_msgs/msg/Twist"](linear=_vec(store, 0.4), angular=_vec(
                    store, z=0.1)), covariance=np.zeros(36)))
        out.append(("/odometry/filtered", "nav_msgs/msg/Odometry", t + 0.01, odom))
        drive = T["ackermann_msgs/msg/AckermannDriveStamped"](
            header=_hdr(store, t, "base_link"),
            drive=T["ackermann_msgs/msg/AckermannDrive"](
                steering_angle=0.1 * i, steering_angle_velocity=0.0, speed=0.5,
                acceleration=0.0, jerk=0.0))
        out.append(("/drive", "ackermann_msgs/msg/AckermannDriveStamped", t + 0.01, drive))
        scan = T["sensor_msgs/msg/LaserScan"](
            header=_hdr(store, t, "laser"), angle_min=-1.0, angle_max=1.0,
            angle_increment=0.5, time_increment=0.0, scan_time=0.025, range_min=0.1,
            range_max=20.0, ranges=np.array([1.0, 2.0, 3.0, 4.0, 5.0 + i], dtype=np.float32),
            intensities=np.array([], dtype=np.float32))
        out.append(("/scan", "sensor_msgs/msg/LaserScan", t + 0.01, scan))
        out.append(("/costmap/front_clearance", "std_msgs/msg/Float32", t + 0.02,
                    T["std_msgs/msg/Float32"](data=1.5 - 0.1 * i)))
        obstacles = [T["f1tenth_messages/msg/Obstacle2D"](x=float(k), y=1.0, r=0.2)
                     for k in range(i)]          # 0, 1, 2 obstacles
        out.append(("/perception/obstacles_2d", "f1tenth_messages/msg/Obstacle2DArray",
                    t + 0.02, T["f1tenth_messages/msg/Obstacle2DArray"](
                        header=_hdr(store, t), obstacles=obstacles)))
        pred = np.arange(4, dtype=np.float32) + i
        out.append(("/mpc/solver_status", "f1tenth_messages/msg/MpcSolverStatus", t + 0.02,
                    T["f1tenth_messages/msg/MpcSolverStatus"](
                        header=_hdr(store, t), success=True, status=0,
                        status_message="ok", solve_dt_sec=0.03, control_period_sec=0.1,
                        cost=10.0 + i, solver="acados", n_boundary_constraints=4,
                        n_obstacles=i, prediction_frame_id="odom", pred_x=pred,
                        pred_y=pred, pred_yaw=pred, pred_v=pred)))
    for k, width in enumerate((2, 3)):           # two maps: only the last is kept
        grid = T["nav_msgs/msg/OccupancyGrid"](
            header=_hdr(store, T0 + DRIVE + k, "map"),
            info=T["nav_msgs/msg/MapMetaData"](
                map_load_time=_t(store, 0.0), resolution=0.05, width=width, height=2,
                origin=_pose(store, -1.0, -2.0, 0.0)),
            data=np.arange(2 * width, dtype=np.int8))
        out.append(("/slam/map", "nav_msgs/msg/OccupancyGrid", T0 + DRIVE + k, grid))
    out.append(("/tf", "tf2_msgs/msg/TFMessage", T0 + DRIVE,
                T["tf2_msgs/msg/TFMessage"](transforms=[])))
    return out


def write_bag(path, store, messages):
    from rosbags.rosbag2 import Writer

    conns = {}
    with Writer(path, version=8) as writer:
        for topic, msgtype, received, msg in messages:
            if topic not in conns:
                conns[topic] = writer.add_connection(topic, msgtype, typestore=store)
            writer.write(conns[topic], int(received * 1e9), store.serialize_cdr(msg, msgtype))


@pytest.fixture(scope="module")
def bag_dir(tmp_path_factory, store):
    path = tmp_path_factory.mktemp("bag") / "bag"
    write_bag(path, store, make_messages(store))
    return path


TIMING = {"ros_start": T0, "drive_start_ros": T0 + DRIVE}


def test_typed_extractors_and_header_time(bag_dir, store):
    topics, table, errors = bagmod.load_bag_topics(bag_dir, TIMING, store)
    assert errors == []
    odom = topics["ekf_local"]
    assert odom["source"] == "bag" and odom["time_source"] == "header"
    np.testing.assert_allclose(odom["x"].ravel(), [0.0, 1.0, 2.0])
    np.testing.assert_allclose(odom["yaw"].ravel(), [0.5] * 3, atol=1e-6)
    np.testing.assert_allclose(odom["vx"].ravel(), [0.4] * 3, atol=1e-6)
    np.testing.assert_allclose(odom["t_rel"].ravel(), [0.0, 0.1, 0.2], atol=1e-6)
    assert odom["info"]["child_frame_id"] == "base_link"
    drive = topics["drive_mpc"]
    np.testing.assert_allclose(drive["steering_angle"].ravel(), [0.0, 0.1, 0.2], atol=1e-6)


def test_header_less_topic_falls_back_to_receive_time(bag_dir, store):
    topics, _, _ = bagmod.load_bag_topics(bag_dir, TIMING, store)
    clear = topics["front_clear"]
    assert clear["time_source"] == "receive"
    np.testing.assert_allclose(clear["t_rel"].ravel(), [0.02, 0.12, 0.22], atol=1e-6)
    np.testing.assert_allclose(clear["data"].ravel(), [1.5, 1.4, 1.3], atol=1e-6)


def test_generic_flattening_of_scalars_arrays_and_sequences(bag_dir, store):
    topics, _, _ = bagmod.load_bag_topics(bag_dir, TIMING, store)
    solver = topics["solver"]
    assert solver["pred_x"].shape == (3, 4)            # fixed length -> matrix
    np.testing.assert_allclose(solver["cost"].ravel(), [10.0, 11.0, 12.0])
    assert solver["status_message"].shape == (3, 1)    # strings -> cell column
    assert solver["status_message"][0, 0] == "ok"
    assert not any(k.startswith("x__") for k in solver)  # no rosbags internals
    obstacles = topics["obstacles2d"]["obstacles"]
    assert obstacles.shape == (3, 1)                   # sequences -> one cell per message
    assert obstacles[0, 0].size == 0                   # an empty sequence is []
    np.testing.assert_allclose(obstacles[2, 0]["x"].ravel(), [0.0, 1.0])
    np.testing.assert_allclose(obstacles[2, 0]["r"].ravel(), [0.2, 0.2], atol=1e-6)


def test_scan_matrix_decimation_and_last_map(bag_dir, store):
    topics, table, _ = bagmod.load_bag_topics(bag_dir, TIMING, store, scan_decimate=2)
    scan = topics["scan"]
    assert scan["ranges"].dtype == np.float32 and scan["ranges"].shape == (2, 5)
    assert scan["info"]["angle_increment"] == 0.5
    assert "intensities" not in scan                    # empty intensities are dropped
    grid = topics["map"]
    assert grid["grid"].dtype == np.int8 and grid["grid"].shape == (2, 3)
    assert grid["origin_x"] == -1.0 and grid["resolution"] == pytest.approx(0.05)
    assert "tf" not in topics                           # excluded, but listed
    assert {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 1,
            "struct": ""} in table


def test_convert_rules():
    assert convert.mat_name("meta.ref_step") == "meta_ref_step"
    assert convert.mat_name("1abc") == "x1abc"
    assert convert.mat_name("a", taken={"a"}) == "a_1"
    np.testing.assert_array_equal(convert.to_mat([[0, 1], [2, 3]]), [[0, 1], [2, 3]])
    soa = convert.to_mat([{"id": 1, "label": "a"}, {"id": None, "label": "b"}])
    assert np.isnan(soa["id"][1, 0]) and soa["label"][1, 0] == "b"
    rec = convert.records_to_struct([{"t": 0, "a": {"b": 1}, "p": [1, 2]},
                                     {"t": 1, "a": {"b": None}, "p": [3]}], skip=("t",))
    assert list(rec) == ["a_b", "p"]
    assert np.isnan(rec["a_b"][1, 0])
    assert rec["p"].shape == (2, 1) and rec["p"][1, 0].shape == (1, 1)


def test_db_root_precedence(tmp_path):
    yaml_path = tmp_path / "logger.yaml"
    yaml_path.write_text("test_campaign_logger:\n  ros__parameters:\n"
                         f"    matlab_export:\n      db_root: {tmp_path / 'from_yaml'}\n")
    env = {db_root.ENV_VAR: str(tmp_path / "from_env")}
    assert db_root.resolve_db_root(str(tmp_path / "cli"), env, yaml_path)[0] == tmp_path / "cli"
    assert db_root.resolve_db_root(None, env, yaml_path)[0] == tmp_path / "from_env"
    assert db_root.resolve_db_root(None, {}, yaml_path)[0] == tmp_path / "from_yaml"
    path, how = db_root.resolve_db_root(None, {}, tmp_path / "missing.yaml")
    assert path == Path("~/matlab_data").expanduser().resolve() and how == "default"
    root, _ = db_root.get_db_root(str(tmp_path / "new"), {}, None)
    assert (root / "runs").is_dir() and (root / "index").is_dir()


def test_db_root_that_cannot_be_written_fails_loudly(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("x")
    with pytest.raises(SystemExit, match="not writable"):
        db_root.get_db_root(str(blocker / "db"), {}, None)


# --------------------------------------------------------------------------
# a whole run: campaign folder + archive + bag -> .mat + runs.csv
# --------------------------------------------------------------------------

def _write_csv(path, header, rows):
    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        writer.writerows(rows)


def make_campaign(root):
    test_dir = root / "campaign" / "M04_person" / "P004-R016-20261001T112933"
    test_dir.mkdir(parents=True)
    (test_dir / "meta.json").write_text(json.dumps({
        "summary": {"test_id": test_dir.name, "mission": "M04_person", "prompt_num": 4,
                    "repetition": 16, "auto_outcome": "completed"},
        "prompt_text": "go to the person", "extra_meta": {"ros_start_time": T0},
        "plans": [{"tag": "initial", "file": "plan.json", "plan_id": "llm_x"}]}))
    events = [{"t": 0.0, "event": "test_start"}, {"t": 0.1, "event": "mission_loaded"},
              {"t": DRIVE, "event": "mission_started", "plan_id": "llm_x"},
              {"t": DRIVE + 1.0, "event": "mission_finished"}, {"t": 5.0, "event": "test_end"}]
    (test_dir / "events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in events))
    _write_csv(test_dir / "kinematics.csv",
               ["t", "x", "y", "yaw", "speed", "corridor_clearance"],
               [[-1.0, 0, 0, 0, 0, ""], [3.0, 0, 0, 0, 0.1, 0.2], [3.5, 0.2, 0, 0, 0.4, ""]])
    _write_csv(test_dir / "imu.csv", ["t", "sensor_stamp", "ax", "gz"],
               [[3.0, T0 + 3.0, 0.1, 0.0]])
    _write_csv(test_dir / "commands.csv", ["t", "cmd_speed", "cmd_steer", "source"],
               [[3.1, 0.4, 0.0, "mpc"]])
    _write_csv(test_dir / "mpc.csv", ["t", "status", "solve_time_ms"], [[3.1, "solved", 30]])
    (test_dir / "horizon.jsonl").write_text(json.dumps(
        {"t": 3.1, "i": 0, "n": 2, "x": [0, 1], "y": [0, 0]}) + "\n")
    (test_dir / "corridors.jsonl").write_text(json.dumps(
        {"t": 3.05, "id": 1, "source": "llm", "polygon": [[0, 0], [1, 0], [1, 1]],
         "meta": {"schema": 2, "definition": {"type": "straight", "C0": [0, 0]}}}) + "\n")
    (test_dir / "plan.json").write_text('{"mission_id": "llm_x", "moves": []}')
    (test_dir / "llm_calls.jsonl").write_text(json.dumps(
        {"call_idx": 0, "t_sent": 0.05, "t_received": 0.06, "latency_ms": 10.0,
         "prompt": "go", "response": "{}", "translated_plan": {"moves": []}}) + "\n")
    return root / "campaign", test_dir


def make_archive(root, store, start_offset):
    run = root / "archive" / "complete" / "2026-10-01T09-29-36_mission-llm_x"
    run.mkdir(parents=True)
    from datetime import datetime, timezone
    start = datetime.fromtimestamp(T0 + DRIVE + start_offset, tz=timezone.utc)
    (run / "2026-10-01T09-29-36_mission-llm_x.manifest.json").write_text(json.dumps(
        {"run_id": run.name, "start_time": start.isoformat()}))
    write_bag(run / "bag", store, make_messages(store))
    return root / "archive"


def _load(path):
    return scipy_io.loadmat(str(path), squeeze_me=True, struct_as_record=False)


def test_whole_run_export_index_and_incremental_skip(tmp_path, store, capsys):
    campaign, test_dir = make_campaign(tmp_path)
    archive = make_archive(tmp_path, store, start_offset=0.03)
    db = tmp_path / "db"
    args = [str(campaign), "--db-root", str(db), "--archive", str(archive)]
    assert exporter.main(args) == 0
    mat = db / "runs" / f"{test_dir.name}.mat"
    data = _load(mat)
    meta = data["meta"]
    assert meta.exporter_version == EXPORTER_VERSION
    assert meta.bag_status == "attached" and meta.has_bag == 1
    assert meta.t_drive_start_source == "mission_started"
    assert meta.bag_coverage_pct == pytest.approx(100.0)
    kin = data["kin"]
    assert kin.source == "campaign"
    np.testing.assert_allclose(kin.t_rel, [-4.0, 0.0, 0.5])
    assert np.isnan(kin.corridor_clearance[0])            # empty stays NaN, never 0
    assert data["cmd"].cmd_source == "mpc"               # 'source' column renamed
    assert data["ekf_local"].source == "bag"
    assert data["llm"].prompt == "go"
    assert "moves" in data["plan"].text

    with open(db / "index" / "runs.csv", newline="") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 1 and rows[0]["test_id"] == test_dir.name
    assert rows[0]["has_bag"] == "1" and rows[0]["bag_status"] == "attached"
    assert rows[0]["success"] == ""                      # manual and unset: empty, not 0
    assert rows[0]["mat_file"] == f"runs/{mat.name}"

    mtime = mat.stat().st_mtime_ns
    assert exporter.main(args) == 0
    assert "skipped 1" in capsys.readouterr().out
    assert mat.stat().st_mtime_ns == mtime
    assert exporter.main(args + ["--force"]) == 0
    assert "exported 1" in capsys.readouterr().out


def test_a_bag_more_than_two_seconds_off_is_not_attached(tmp_path, store, capsys):
    campaign, test_dir = make_campaign(tmp_path)
    archive = make_archive(tmp_path, store, start_offset=2.5)
    db = tmp_path / "db"
    assert exporter.main([str(campaign), "--db-root", str(db), "--archive", str(archive)]) == 0
    meta = _load(db / "runs" / f"{test_dir.name}.mat")["meta"]
    assert meta.bag_status == "no_match" and meta.has_bag == 0
    assert "no_match" in capsys.readouterr().out


def test_one_failing_run_does_not_stop_the_batch(tmp_path, store, capsys):
    campaign, test_dir = make_campaign(tmp_path)
    broken = test_dir.parent / "P004-R017-20261001T113120"
    broken.mkdir()
    (broken / "meta.json").write_text("{}")             # no ros_start_time
    archive = make_archive(tmp_path, store, start_offset=0.0)
    db = tmp_path / "db"
    assert exporter.main([str(campaign), "--db-root", str(db), "--archive", str(archive)]) == 1
    out = capsys.readouterr().out
    assert "exported 1" in out and "failed 1" in out
    assert (db / "runs" / f"{test_dir.name}.mat").exists()
    assert not (db / "runs" / f"{broken.name}.mat").exists()
    assert not any(p.suffix == ".tmp" for p in (db / "runs").iterdir())


def test_runs_filter_uses_the_shared_parser(tmp_path, store):
    campaign, _ = make_campaign(tmp_path)
    archive = make_archive(tmp_path, store, start_offset=0.0)
    with pytest.raises(SystemExit, match="R099"):
        exporter.main([str(campaign), "--db-root", str(tmp_path / "db"),
                       "--archive", str(archive), "--runs", "16-99"])
    assert os.path.isdir(tmp_path / "db" / "runs")
