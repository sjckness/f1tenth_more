"""
wall_tracker_impl:=py|rust (R6): the dispatch, the refusals and the Rust tracker's mirror.

wall_tracker_backend.py's own contract. The tracker's BEHAVIOUR is the other
wall tests' business, and they run against both backends
(F1TENTH_WALL_TRACKER_IMPL=rust colcon test, output/rust_main/r6.md); this
file pins what only the switch has: the default and its test hook, the refusal
of an unknown value or an unloadable library, and the mirror -- the same Track
objects call after call, Python edits pushed back, in-place writes refused, a
lockstep run equal field for field with the Python. The Rust tests skip (in a
fixture, not at import: a module-level skip would hide the py tests too) when
libf1tenth_wall_tracker_core.so is not built.
"""

import math

from corridor_perception.geometry import Pose2D
from f1tenth_perception import multi_wall_tracker as mwt
from f1tenth_perception import wall_fusion
from f1tenth_perception import wall_tracker_backend as wtb
from f1tenth_perception import wall_tracker_native as wtn
import numpy as np
import pytest

LASER = (0.305, 0.0, 0.0)
N_BEAMS = 1081
ANGLE_MIN = -math.radians(135.0)
INC = math.radians(0.25)
ROOM = [((0, 0), (6, 0)), ((6, 0), (6, 6)), ((6, 6), (0, 6)), ((0, 6), (0, 0))]


@pytest.fixture
def rust():
    if not wtn.available():
        pytest.skip(f'f1tenth_wall_tracker_core not loadable: {wtn.load_error()}')
    return 'rust'


def raycast(car, segments, rng):
    laser = car.compose(Pose2D(*LASER))
    ang = laser.theta + ANGLE_MIN + np.arange(N_BEAMS) * INC
    d = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    best = np.full(N_BEAMS, np.inf)
    for (ax, ay), (bx, by) in segments:
        e = np.array([bx - ax, by - ay], dtype=float)
        w = np.array([ax, ay], dtype=float) - laser.t
        den = d[:, 0] * (-e[1]) - d[:, 1] * (-e[0])
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (w[0] * (-e[1]) - w[1] * (-e[0])) / den
            u = (d[:, 0] * w[1] - d[:, 1] * w[0]) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(hit & (t < best), t, best)
    return np.where(np.isfinite(best), best + rng.normal(0.0, 0.01, N_BEAMS), 65.533)


def cycle(car, rng):
    """One scan of ROOM: (observations, scan tuple)."""
    ranges = raycast(car, ROOM, rng)
    lines = mwt.extract_lines(ranges, ANGLE_MIN, INC, 0.06, 30.0, LASER)
    obs = [wall_fusion.Observation(mwt.line_to_wall_line(ln), 0.0, 2.5, False,
                                   wall_fusion.SOURCE_LIDAR, 0.8) for ln in lines]
    r, b, _a, cut = mwt.scan_in_base_link(ranges, ANGLE_MIN, INC, 0.06, 30.0, LASER, (),
                                          with_cut=True)
    return obs, (r, b, LASER[:2], cut)


def poses(n):
    for i in range(n):
        yield i * 0.1, Pose2D(1.5 + 0.03 * i, 2.5 + 0.01 * i, 0.02 * i)


def _nan_free(d):
    """Return the dict with NaN floats as None, so == compares them (NaN != NaN)."""
    return {k: (None if isinstance(v, float) and math.isnan(v) else v) for k, v in d.items()}


def state(trk):
    """Every field of every track and the tracker attributes, comparable."""
    out = {'next_id': trk._next_id, 'last_stamp': trk._last_stamp, 'odo': trk._odometer,
           'counts': (trk.n_short_dropped, trk.n_negative_killed, trk.n_gap_rejected,
                      trk.n_merge_gap_refused, trk.n_stereo_near_rejected,
                      trk.n_camera_rejected),
           'rejections': list(trk.rejections),
           'strip': (trk.strip.range_m, trk.strip.n_returns, trk.strip.n_cut,
                     trk.strip.n_no_return),
           'tracks': []}
    for t in trk.tracks:
        d = {k: getattr(t, k) for k in (
            'id', 'hits', 't0', 't1', 'born', 'last_seen', 'ever_confirmed', 'state', 'source',
            'role', 'verify', 'glass', 'miss', 'odo_at_last_seen', 'occluded',
            'strip_verdict', 'strip_miss', 'contradicted', 'confidence')}
        d['x'] = tuple(float(v) for v in t.x)
        d['P'] = tuple(float(v) for v in np.asarray(t.P).ravel())
        d['confirm_obs'] = tuple(t.confirm_obs)
        d['lidar_hist'] = tuple(t.lidar_hist)
        d['conf_hist'] = tuple(t.conf_hist)
        d['obs_ext'] = tuple((s, tuple(np.asarray(p).ravel())) for s, p in t.obs_ext)
        d['strip_pred_m'] = None if math.isnan(t.strip_pred_m) else t.strip_pred_m
        out['tracks'].append(d)
    return out


# ---- the switch ------------------------------------------------------------

def test_the_default_is_py_and_builds_the_python_class(monkeypatch):
    monkeypatch.delenv(wtb.ENV_IMPL, raising=False)
    assert wtb.default_impl() == 'py'
    monkeypatch.setattr(wtb, 'DEFAULT_IMPL', 'py')
    trk = mwt.MultiWallTracker()
    assert type(trk) is mwt.MultiWallTracker
    assert type(mwt.MultiWallTracker(impl='py')) is mwt.MultiWallTracker


def test_the_environment_hook_overrides_the_stack_default(monkeypatch):
    monkeypatch.setenv(wtb.ENV_IMPL, 'rust')
    assert wtb.default_impl() == 'rust'
    monkeypatch.setenv(wtb.ENV_IMPL, 'fast')
    with pytest.raises(ValueError):
        wtb.default_impl()


def test_unknown_values_are_refused():
    for bad in ('', 'c', 'Rust', None):
        with pytest.raises(ValueError):
            wtb.check_impl(bad)
    with pytest.raises(ValueError):
        mwt.MultiWallTracker(impl='cpp')


def test_rust_without_the_library_refuses_rather_than_falling_back(monkeypatch):
    monkeypatch.setattr(wtn, 'available', lambda: False)
    monkeypatch.setattr(wtn, 'load_error', lambda: 'OSError: no such file')
    with pytest.raises(RuntimeError, match='not usable: OSError'):
        wtb.require('rust')
    with pytest.raises(RuntimeError):
        mwt.MultiWallTracker(impl='rust')
    assert wtb.require('py') == 'py'


# ---- the Rust tracker and its mirror ---------------------------------------

def test_rust_is_a_multi_wall_tracker_and_describes_itself(rust):
    trk = mwt.MultiWallTracker(impl=rust)
    assert isinstance(trk, mwt.MultiWallTracker)
    assert type(trk) is wtb.NativeMultiWallTracker
    assert 'f1tenth_wall_tracker_core' in wtb.describe('rust')
    assert 'opt-level 3' in wtb.describe('rust')


def test_a_lockstep_run_is_identical_to_the_python(rust):
    py, rs = mwt.MultiWallTracker(impl='py'), mwt.MultiWallTracker(impl=rust)
    rng_py, rng_rs = np.random.default_rng(7), np.random.default_rng(7)
    for t, car in poses(30):
        o1, s1 = cycle(car, rng_py)
        o2, s2 = cycle(car, rng_rs)
        a = py.step(o1, t, car, scan=s1)
        b = rs.step(o2, t, car, scan=s2)
        assert [x.id for x in a] == [y.id for y in b]
        assert state(py) == state(rs), t
        fa, fb = py.front_track(car, 0.405), rs.front_track(car, 0.405)
        assert (fa is None) == (fb is None)
        assert _nan_free(py.front_status) == _nan_free(rs.front_status)
    assert any(t.ever_confirmed for t in rs.tracks)


def test_tracks_keep_their_identity_and_front_is_a_published_track(rust):
    trk = mwt.MultiWallTracker(impl=rust)
    rng = np.random.default_rng(3)
    first = None
    for t, car in poses(12):
        obs, scan = cycle(car, rng)
        tracks = trk.step(obs, t, car, scan=scan)
        if first is None and trk.tracks:
            first = trk.tracks[0]
    assert first is trk.track(first.id)
    front = trk.front_track(car, 0.405)
    if front is not None:
        assert any(front is x for x in trk.published(car))
        assert any(front is x for x in tracks)


def test_python_edits_are_pushed_and_in_place_writes_refused(rust):
    py, rs = mwt.MultiWallTracker(impl='py'), mwt.MultiWallTracker(impl=rust)
    rng_py, rng_rs = np.random.default_rng(5), np.random.default_rng(5)
    for t, car in poses(8):
        for trk, rng in ((py, rng_py), (rs, rng_rs)):
            obs, scan = cycle(car, rng)
            trk.step(obs, t, car, scan=scan)
    for trk in (py, rs):
        trk.tracks[0].glass = True
        trk.tracks[0].miss = 3
        trk.tracks[-1].role = ''
        trk.p.max_published = 2
    for t, car in poses(14):
        if t <= 0.75:
            continue
        for trk, rng in ((py, rng_py), (rs, rng_rs)):
            obs, scan = cycle(car, rng)
            trk.step(obs, t, car, scan=scan)
        assert state(py) == state(rs), t
        assert len(rs.published(car)) <= 2
    with pytest.raises(ValueError):
        rs.tracks[0].x[0] = 1.0     # read-only: an in-place edit cannot go unnoticed
    with pytest.raises(ValueError):
        rs.tracks[0].P[0, 0] = 1.0


def test_spawn_and_merge_called_directly_agree_with_the_python(rust):
    car = Pose2D(0.0, 0.0, 0.0)
    out = []
    for impl in ('py', rust):
        trk = mwt.MultiWallTracker(impl=impl)
        wl = mwt.WallLine(1.0, 0.0, 2.0, -1.0, 1.0, 40, 0.005, 'lidar')
        o = wall_fusion.Observation(wl, 0.0, 2.5, False, wall_fusion.SOURCE_LIDAR, 0.8)
        for k in range(3):
            trk.step([o], 0.1 * k, car)
        piece = wall_fusion.Observation(
            mwt.WallLine(1.0, 0.0, 2.0, 0.5, 1.5, 40, 0.005, 'lidar'), 0.0, 2.5, False,
            wall_fusion.SOURCE_LIDAR, 0.8)
        trk._spawn(mwt.observation_line(piece, car, trk.p), 0.25)
        n_spawned = len(trk.tracks)
        trk._merge()
        out.append((n_spawned, state(trk)))
    assert out[0] == out[1]
    assert out[0][0] == 2 and len(out[0][1]['tracks']) == 1


def test_a_hand_made_track_is_adopted_not_copied(rust):
    trk = mwt.MultiWallTracker(impl=rust)
    mine = mwt.Track(7, np.array([2.0, 0.0]), np.eye(2) * 0.01, -1.0, 1.0, 0.0, 0.0,
                     ever_confirmed=True, state=mwt.STATE_CONFIRMED)
    trk.tracks.append(mine)
    car = Pose2D(0.0, 0.0, 0.0)
    trk.update_roles(car)
    assert trk.tracks[0] is mine
    assert mine.role == 'front'


def test_the_rust_rectangle_check_equals_the_python():
    if not wtn.available():
        pytest.skip(f'f1tenth_wall_tracker_core not loadable: {wtn.load_error()}')
    rng = np.random.default_rng(11)
    labels = rng.choice([2, 3, 6], size=(180, 320), p=[0.1, 0.3, 0.6]).astype(np.uint8)
    depth = rng.uniform(500, 9000, size=(180, 320)).astype(np.uint16)
    depth[rng.uniform(size=depth.shape) < 0.1] = 0
    rot = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    trans = np.array([0.2, 0.0, 0.5])
    segs = [((3.0, -1.0), (3.0, 1.0)), ((1.0, 2.0), (6.0, 2.0)), ((9.0, -3.0), (12.0, 4.0))]
    p = wall_fusion.RectParams()
    a = wall_fusion.verify_rectangles(segs, labels, depth, 131.2, 131.2, 160.0, 90.0, rot,
                                      trans, laser_z=0.19, p=p, depth_scale=0.001)
    b = wtb.verify_rectangles(segs, labels, depth, 131.2, 131.2, 160.0, 90.0, rot, trans,
                              laser_z=0.19, p=p, depth_scale=0.001)
    assert a == b
    assert any(v['coverage'] > 0 for v in a)


# ---- the node parameter ------------------------------------------------------

@pytest.fixture(scope='module')
def ros():
    import rclpy
    owned = not rclpy.ok()
    if owned:
        rclpy.init()
    yield rclpy
    if owned and rclpy.ok():
        rclpy.shutdown()


def _node(**params):
    from rclpy.parameter import Parameter

    from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode
    overrides = [Parameter('enable_scene_labels', value=True),
                 Parameter('use_wall_tracker', value=True)]
    overrides += [Parameter(k, value=v) for k, v in params.items()]
    return LidarFrontWallNode(parameter_overrides=overrides)


def test_the_node_defaults_to_the_stack_value(ros):
    n = _node()
    try:
        assert n.wall_tracker_impl == wtb.default_impl()
        native = n.wall_tracker_impl == 'rust'
        assert (type(n.wall_tracker) is wtb.NativeMultiWallTracker) == native
        assert n._native_fusion == native
    finally:
        n.destroy_node()


def test_the_node_refuses_an_unknown_impl(ros):
    with pytest.raises(ValueError):
        _node(wall_tracker_impl='cpp')


def test_the_node_runs_rust_when_asked_and_the_switch_is_read_only(ros, rust):
    from rclpy.parameter import Parameter
    n = _node(wall_tracker_impl=rust)
    try:
        assert type(n.wall_tracker) is wtb.NativeMultiWallTracker
        assert n._native_fusion
        result = n.set_parameters([Parameter('wall_tracker_impl', value='py')])[0]
        assert not result.successful
        assert n.wall_tracker_impl == 'rust'
    finally:
        n.destroy_node()
