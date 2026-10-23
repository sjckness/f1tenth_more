"""
wall_tracker_backend.py -- which implementation runs lidar_front_wall_node's wall tracker.

wall_tracker_impl (stack_params.yaml wall_tracker_impl; the node parameter,
lidar_front_wall.launch.py and supervisor_bringup.launch.py wall_tracker_impl:=py|rust,
the supervisor passing it on to the lidar_front_wall component; stack_bringup.launch.py
does not start that node):

  'py'    multi_wall_tracker.MultiWallTracker and wall_fusion, the reference
          (the default);
  'rust'  f1tenth_wall_tracker_core (src/rust/f1tenth_wall_tracker_core)
          through wall_tracker_native.py: the same tracker and the same
          per-cycle fusion ported to Rust, bit-identical on linus
          (output/rust/parity_wall_tracker.md).

THE CLASS. `NativeMultiWallTracker` IS a `MultiWallTracker` (a subclass): the
library holds the tracker state and runs the mutating steps (step, update_roles,
apply_rect_verdicts, published, front_track, _spawn, _merge), and after each one
the state is MIRRORED into real `multi_wall_tracker.Track` objects, so every
reader -- the node's publishers and debug payload, door_detection, wall_markers,
the replay tools, the tests -- reads the same attributes it always did, and the
tracker's own read-only methods (front_reason, is_front, front_confidence, label,
confirm_reason, coasting, ...) run unchanged on that mirror. A mirrored track
keeps its identity from call to call (`front is tracks[0]` holds as in Python).

PYTHON MAY CHANGE THE STATE. Tests set track fields, append tracks, call
_spawn / _merge directly, change `tracker.p`. Every assignment to a mirrored
track (or a mutation of one of its deques) marks the mirror dirty, and the next
library call first pushes the whole Python state back; the parameters are
compared field by field before every call; the tracker-level attributes
(laser_xy, camera_xy, camera_fresh, _next_id, _last_stamp, _odometer, ...) are
pushed on every call. The mirrored `x` and `P` arrays (and the obs_ext points)
are read-only: an in-place write raises instead of going unnoticed. With
F1TENTH_WALL_TRACKER_VERIFY_SYNC=1 every call also checks that the mirror still
equals the library state (the parity harness runs with it).

THE DEFAULT. DEFAULT_IMPL is what a caller without an explicit impl gets:
`multi_wall_tracker.MultiWallTracker(params, rect)` returns a
NativeMultiWallTracker when it is 'rust'. It is the stack_params.yaml value,
unless the environment variable F1TENTH_WALL_TRACKER_IMPL is set: that is the
test hook that runs every tracker test against the Rust library
(`F1TENTH_WALL_TRACKER_IMPL=rust colcon test ...`). The node takes its parameter
default from default_impl() too; an explicit wall_tracker_impl parameter always
wins (it is passed as `impl=`).

RUST REQUESTED BUT NOT LOADABLE IS AN ERROR at construction (require()), never
a silent fall back to the Python.

DIFFERENCES FROM THE PYTHON, all outside what any caller does:
- a Track pushed from Python whose deques were replaced by lists comes back as
  deques; a hand-made track with an id the tracker later issues is a different
  Python object in the reference but the same mirror object here;
- an overflowing `x ** 2` (|x| > 1e154) raises OverflowError in Python and is
  inf here (the squared quantities are sigmas and turn rates);
- a 4-tuple scan whose cut mask is None: the Python's negative evidence raises,
  this treats it as no cut (as the strip and the open-gap tests do in both).
"""

import collections
import os
import weakref

from f1tenth_params.param_defaults import get_value
import numpy as np

from f1tenth_perception import multi_wall_tracker as mwt
from f1tenth_perception import scene_mask, wall_fusion
from f1tenth_perception import wall_tracker_native as wtn

#: The accepted wall_tracker_impl values.
IMPLS = ('py', 'rust')

#: Test hook: overrides the stack_params.yaml default (see the module docstring).
ENV_IMPL = 'F1TENTH_WALL_TRACKER_IMPL'
#: Parity aid: check the mirror against the library state on every call.
ENV_VERIFY = 'F1TENTH_WALL_TRACKER_VERIFY_SYNC'

#: The stack_params.yaml key (unprefixed, like corridor_impl: the node
#: parameter and the launch arguments all carry this name).
STACK_KEY = 'wall_tracker_impl'


def check_impl(impl):
    """Raise ValueError for a value wall_tracker_impl does not accept."""
    if impl not in IMPLS:
        raise ValueError(f'wall_tracker_impl must be one of {IMPLS}, got {impl!r}')
    return impl


def default_impl():
    """F1TENTH_WALL_TRACKER_IMPL if set, else stack_params.yaml's value (py)."""
    env = os.environ.get(ENV_IMPL, '').strip()
    if env:
        return check_impl(env)
    try:
        return check_impl(str(get_value(STACK_KEY)))
    except KeyError:
        return 'py'


#: Read once, at import. Tests may assign it; MultiWallTracker reads it per call.
DEFAULT_IMPL = default_impl()


def require(impl):
    """
    For a node's startup: a valid impl whose backend is usable, or raise.

    ValueError for an unknown value; RuntimeError for 'rust' when
    libf1tenth_wall_tracker_core.so does not load, with the loader's reason.
    """
    check_impl(impl)
    if impl == 'rust' and not wtn.available():
        raise RuntimeError(
            'wall_tracker_impl:=rust but the Rust wall tracker library is not usable: '
            f'{wtn.load_error()}. Build it (colcon build --packages-select '
            'f1tenth_wall_tracker_core) or run wall_tracker_impl:=py.')
    return impl


def describe(impl):
    """One line for the startup log: which tracker runs, and from where."""
    if impl == 'rust':
        return (f'rust (f1tenth_wall_tracker_core {wtn.version()}, {wtn.library_path()}, '
                f'{wtn.build_info()})')
    return 'py (f1tenth_perception.multi_wall_tracker)'


def native_class_for(impl=None):
    """Return NativeMultiWallTracker if `impl` (default DEFAULT_IMPL) is rust, else None."""
    impl = DEFAULT_IMPL if impl is None else impl
    if require(impl) == 'rust':
        return NativeMultiWallTracker
    return None


# ============================================================================
# The mirror
# ============================================================================

ROLE_NAMES = ('', 'front', 'back', 'left', 'right', mwt.ROLE_UNKNOWN, mwt.LEGACY_ROLE_OTHER)
ROLE_CODES = {name: code for code, name in enumerate(ROLE_NAMES)}
STRIP_NAMES = ('', mwt.STRIP_SUPPORTED, mwt.STRIP_CONTRADICTED, mwt.STRIP_BLOCKED,
               mwt.STRIP_NO_RETURNS)
STRIP_CODES = {name: code for code, name in enumerate(STRIP_NAMES)}
FRONT_NAMES = ('', 'unconfirmed', 'contradicted', 'role_other', 'off_strip', 'coast_expired',
               'coast_contradicted', 'low_confidence', 'unverified', 'stereo_oblique', 'front',
               'no_candidate')
REJECT_NAMES = {1: 'gap_assoc', 2: 'gap_merge', 3: 'stereo_near', 4: 'stereo_short',
                5: 'stereo_rms', 6: 'stereo_flat', 7: 'camera_radial'}

_TRACK_FIELDS = [f.split(':')[0] for f in wtn.FIELDS[wtn.STRUCT_TRACK].split(',')]
_TI = {name: k for k, name in enumerate(_TRACK_FIELDS)}
_OBS_DTYPE = wtn._dtype(wtn.FIELDS[wtn.STRUCT_OBSERVATION])
_LINE_DTYPE = wtn._dtype(wtn.FIELDS[wtn.STRUCT_LINE])
_TRACK_DTYPE = wtn._dtype(wtn.FIELDS[wtn.STRUCT_TRACK])
_SCALAR_DTYPE = wtn._dtype(wtn.FIELDS[wtn.STRUCT_SCALARS])


def _frozen(a):
    a = np.array(a, dtype=float)
    a.flags.writeable = False
    return a


class _WatchedDeque(collections.deque):
    """A deque that marks its mirrored track's tracker dirty on any mutation."""

    __slots__ = ('_wt_owner',)

    def _touch(self):
        ref = getattr(self, '_wt_owner', None)
        owner = ref() if ref is not None else None
        if owner is not None:
            owner._dirty = True


def _watch(method):
    def wrapped(self, *a, **kw):
        self._touch()
        return method(self, *a, **kw)
    wrapped.__name__ = method.__name__
    return wrapped


for _name in ('append', 'appendleft', 'extend', 'extendleft', 'pop', 'popleft', 'remove',
              'rotate', 'clear', 'insert', '__setitem__', '__delitem__', '__iadd__',
              '__imul__', 'reverse'):
    setattr(_WatchedDeque, _name, _watch(getattr(collections.deque, _name)))


def _deque(items, maxlen, owner_ref):
    d = _WatchedDeque(items, maxlen)
    d._wt_owner = owner_ref
    return d


class _MirrorTrack(mwt.Track):
    """
    A multi_wall_tracker.Track whose fields mirror the library state.

    Same dataclass, same fields, same methods; an attribute assignment marks
    the owning tracker dirty so the change is pushed before its next call.
    """

    def __setattr__(self, name, value):
        object.__setattr__(self, name, value)
        ref = self.__dict__.get('_wt_owner')
        owner = ref() if ref is not None else None
        if owner is not None:
            owner._dirty = True


def _maxlen_code(d):
    m = getattr(d, 'maxlen', None)
    return -1 if m is None else int(m)


class NativeMultiWallTracker(mwt.MultiWallTracker):
    """multi_wall_tracker.MultiWallTracker, the mutating steps in Rust. See the module."""

    def __new__(cls, *args, **kwargs):
        return object.__new__(cls)

    def __init__(self, params=None, rect_params=None, impl=None):
        super().__init__(params, rect_params)
        pf, pi, rf, ri = self._param_arrays()
        self._native = wtn.Handle(pf, pi, rf, ri)
        self._param_key = self._params_snapshot()
        self._mirror = {}         # track id -> _MirrorTrack
        self._hashes = {}         # track id -> library hash of the mirrored state
        self._ext_by_serial = {}  # obs_ext serial -> (stamp, points) tuple
        self._serial_of = {}      # id(tuple) -> serial
        self._exported_list = self.tracks
        self._exported_ids = []
        self._dirty = False
        self._self_ref = weakref.ref(self)
        self._verify = os.environ.get(ENV_VERIFY, '') not in ('', '0', 'false', 'False')
        self._sc_row = np.zeros(1, dtype=_SCALAR_DTYPE)

    # ---- parameters ------------------------------------------------------

    def _param_arrays(self):
        p, r = self.p, self.rect
        pf = [float(getattr(p, k)) for k in wtn.PARAM_F64]
        pi = [int(bool(p.confirm_require_lidar)) if k == 'confirm_require_lidar'
              else int(getattr(p, k)) for k in wtn.PARAM_I64]
        rf = [float(getattr(r, k)) for k in wtn.RECT_F64]
        ri = [int(getattr(r, k)) for k in wtn.RECT_I64]
        return pf, pi, rf, ri

    def _params_snapshot(self):
        p, r = self.p, self.rect
        return (tuple(getattr(p, k) for k in wtn.PARAM_F64 + wtn.PARAM_I64)
                + tuple(getattr(r, k) for k in wtn.RECT_F64 + wtn.RECT_I64))

    # ---- push ------------------------------------------------------------

    def _push_scalars(self):
        row = self._sc_row
        s = row[0]
        lx = self.laser_xy
        s['has_laser'] = int(lx is not None)
        if lx is not None:
            s['laser_x'], s['laser_y'] = float(lx[0]), float(lx[1])
        cx = self.camera_xy
        s['has_camera'] = int(cx is not None)
        if cx is not None:
            s['camera_x'], s['camera_y'] = float(cx[0]), float(cx[1])
        s['camera_fresh'] = int(bool(self.camera_fresh))
        s['next_id'] = int(self._next_id)
        s['has_last_stamp'] = int(self._last_stamp is not None)
        if self._last_stamp is not None:
            s['last_stamp'] = float(self._last_stamp)
        s['has_last_yaw'] = int(self._last_yaw is not None)
        if self._last_yaw is not None:
            s['last_yaw'] = float(self._last_yaw)
        s['odometer'] = float(self._odometer)
        t = self._last_car_t
        s['has_last_car_t'] = int(t is not None)
        if t is not None:
            s['last_car_x'], s['last_car_y'] = float(t[0]), float(t[1])
        for k in ('n_short_dropped', 'n_negative_killed', 'n_gap_rejected',
                  'n_merge_gap_refused', 'n_stereo_near_rejected', 'n_camera_rejected'):
            s[k] = int(getattr(self, k))
        self._native.set_scalars(row)

    def _tracks_changed(self):
        if self._dirty or self.tracks is not self._exported_list:
            return True
        ids = self._exported_ids
        if len(ids) != len(self.tracks):
            return True
        return any(id(t) != i for t, i in zip(self.tracks, ids))

    def _push_tracks(self):
        rows = np.zeros(len(self.tracks), dtype=_TRACK_DTYPE)
        confirm, lidar, conf, ext, serials = [], [], [], [], []
        for k, trk in enumerate(self.tracks):
            if type(trk) is mwt.Track:
                # Adopt a hand-made track so the caller's reference IS the mirror.
                trk.__class__ = _MirrorTrack
            r = rows[k]
            r['id'], r['hits'] = int(trk.id), int(trk.hits)
            r['state'], r['source'], r['verify'] = int(trk.state), int(trk.source), int(trk.verify)
            role = trk.role if trk.role is not None else ''
            if role not in ROLE_CODES:
                raise ValueError(f'track {trk.id}: role {role!r} is not one the tracker knows')
            r['role'] = ROLE_CODES[role]
            if trk.strip_verdict not in STRIP_CODES:
                raise ValueError(f'track {trk.id}: strip_verdict {trk.strip_verdict!r} unknown')
            r['strip_verdict'] = STRIP_CODES[trk.strip_verdict]
            for name in ('rect_reject_streak', 'miss', 'strip_miss'):
                r[name] = int(getattr(trk, name))
            for name in ('ever_confirmed', 'has_lidar', 'has_both', 'height_measured', 'glass',
                         'rect_informative', 'occluded', 'contradicted'):
                r[name] = int(bool(getattr(trk, name)))
            x = np.asarray(trk.x, dtype=float).reshape(2)
            P = np.asarray(trk.P, dtype=float).reshape(2, 2)
            r['x0'], r['x1'] = x
            r['p00'], r['p01'], r['p10'], r['p11'] = P.ravel()
            for name in ('t0', 't1', 'born', 'last_seen', 'z_min', 'z_max', 'confidence',
                         'rect_coverage', 'rect_support', 'rect_beyond_share',
                         'odo_at_last_seen', 'strip_pred_m'):
                r[name] = float(getattr(trk, name))
            r['confirm_maxlen'] = _maxlen_code(trk.confirm_obs)
            r['confirm_off'], r['confirm_len'] = len(confirm), len(trk.confirm_obs)
            for c in trk.confirm_obs:
                confirm.append((0.0, 0.0, 0.0) if c is None else (1.0, float(c[0]), float(c[1])))
            r['lidar_maxlen'] = _maxlen_code(trk.lidar_hist)
            r['lidar_off'], r['lidar_len'] = len(lidar), len(trk.lidar_hist)
            lidar.extend(int(bool(v)) for v in trk.lidar_hist)
            r['conf_maxlen'] = _maxlen_code(trk.conf_hist)
            r['conf_off'], r['conf_len'] = len(conf), len(trk.conf_hist)
            conf.extend(float(v) for v in trk.conf_hist)
            r['ext_maxlen'] = _maxlen_code(trk.obs_ext)
            r['ext_off'], r['ext_len'] = len(ext), len(trk.obs_ext)
            for e in trk.obs_ext:
                pts = np.asarray(e[1], dtype=float).reshape(2, 2)
                ext.append((float(e[0]), pts[0, 0], pts[0, 1], pts[1, 0], pts[1, 1]))
                serials.append(self._serial_of.get(id(e), 0))
            # The pushed object becomes (or stays) the mirror of its id.
            self._mirror[int(trk.id)] = trk
            self._hashes[int(trk.id)] = None
        self._native.import_tracks(rows, np.array(confirm, dtype=float).reshape(-1, 3),
                                   np.array(lidar, dtype=np.int64), np.array(conf, dtype=float),
                                   np.array(ext, dtype=float).reshape(-1, 5),
                                   np.array(serials, dtype=np.int64))

    def _sync_in(self):
        """Push what Python may have changed. True when the tracks were pushed."""
        key = self._params_snapshot()
        if key != self._param_key:
            self._native.set_params(*self._param_arrays())
            self._param_key = key
        self._push_scalars()
        if self._tracks_changed():
            self._push_tracks()
            return True
        return False

    # ---- pull ------------------------------------------------------------

    def _pull_scalars(self):
        s = self._native.get_scalars()
        self._next_id = int(s['next_id'])
        self._last_stamp = float(s['last_stamp']) if s['has_last_stamp'] else None
        self._last_yaw = float(s['last_yaw']) if s['has_last_yaw'] else None
        self._odometer = float(s['odometer'])
        self._last_car_t = (np.array([float(s['last_car_x']), float(s['last_car_y'])])
                            if s['has_last_car_t'] else None)
        for k in ('n_short_dropped', 'n_negative_killed', 'n_gap_rejected',
                  'n_merge_gap_refused', 'n_stereo_near_rejected', 'n_camera_rejected'):
            setattr(self, k, int(s[k]))
        self.strip = mwt.StripReading(
            float(s['strip_range_m']) if s['strip_has_range'] else None,
            int(s['strip_n_returns']), int(s['strip_n_cut']), int(s['strip_n_no_return']))
        self.rejections = [(REJECT_NAMES[int(code)], int(tid) if has else None,
                            round(float(value), 3))
                           for code, has, tid, value in self._native.rejections()]
        return s

    def _ext_entry(self, serial, row):
        entry = self._ext_by_serial.get(serial)
        if entry is None:
            pts = np.array([[row[1], row[2]], [row[3], row[4]]], dtype=float)
            pts.flags.writeable = False
            entry = (float(row[0]), pts)
            self._ext_by_serial[serial] = entry
            self._serial_of[id(entry)] = serial
        return entry

    def _pull(self):
        self._pull_scalars()
        tracks, confirm, lidar, conf, ext, serials = self._native.export_tracks()
        rows = tracks.tolist()
        ti = _TI
        owner = self._self_ref
        out, seen_serials = [], set()
        ext_rows = ext.tolist()
        ser = serials.tolist()
        conf_l = conf.tolist()
        lidar_l = lidar.tolist()
        confirm_l = confirm.tolist()
        for row in rows:
            tid = row[ti['id']]
            e0, en = row[ti['ext_off']], row[ti['ext_len']]
            ext_serials = ser[e0:e0 + en]
            seen_serials.update(ext_serials)
            obj = self._mirror.get(tid)
            if obj is not None and self._hashes.get(tid) == row[ti['hash']]:
                out.append(obj)
                continue
            if obj is None:
                obj = object.__new__(_MirrorTrack)
            c0, cn = row[ti['confirm_off']], row[ti['confirm_len']]
            l0, ln = row[ti['lidar_off']], row[ti['lidar_len']]
            f0, fn = row[ti['conf_off']], row[ti['conf_len']]

            def ml(code):
                return None if code < 0 else code
            fields = {
                'id': tid, 'x': _frozen([row[ti['x0']], row[ti['x1']]]),
                'P': _frozen([[row[ti['p00']], row[ti['p01']]], [row[ti['p10']], row[ti['p11']]]]),
                't0': row[ti['t0']], 't1': row[ti['t1']], 'born': row[ti['born']],
                'last_seen': row[ti['last_seen']], 'hits': row[ti['hits']],
                'confirm_obs': _deque(
                    [None if v == 0.0 else (r, a) for v, r, a in confirm_l[c0:c0 + cn]],
                    ml(row[ti['confirm_maxlen']]), owner),
                'ever_confirmed': bool(row[ti['ever_confirmed']]), 'state': row[ti['state']],
                'source': row[ti['source']], 'has_lidar': bool(row[ti['has_lidar']]),
                'has_both': bool(row[ti['has_both']]), 'z_min': row[ti['z_min']],
                'z_max': row[ti['z_max']], 'height_measured': bool(row[ti['height_measured']]),
                'confidence': row[ti['confidence']], 'role': ROLE_NAMES[row[ti['role']]],
                'verify': row[ti['verify']], 'glass': bool(row[ti['glass']]),
                'rect_coverage': row[ti['rect_coverage']],
                'rect_support': row[ti['rect_support']],
                'rect_beyond_share': row[ti['rect_beyond_share']],
                'rect_informative': bool(row[ti['rect_informative']]),
                'rect_reject_streak': row[ti['rect_reject_streak']], 'miss': row[ti['miss']],
                'odo_at_last_seen': row[ti['odo_at_last_seen']],
                'occluded': bool(row[ti['occluded']]),
                'obs_ext': _deque([self._ext_entry(s, er) for s, er in
                                   zip(ext_serials, ext_rows[e0:e0 + en])],
                                  ml(row[ti['ext_maxlen']]), owner),
                'lidar_hist': _deque([bool(v) for v in lidar_l[l0:l0 + ln]],
                                     ml(row[ti['lidar_maxlen']]), owner),
                'conf_hist': _deque(conf_l[f0:f0 + fn], ml(row[ti['conf_maxlen']]), owner),
                'strip_verdict': STRIP_NAMES[row[ti['strip_verdict']]],
                'strip_pred_m': row[ti['strip_pred_m']], 'strip_miss': row[ti['strip_miss']],
                'contradicted': bool(row[ti['contradicted']]),
            }
            obj.__dict__.update(fields)
            obj.__dict__['_wt_owner'] = owner
            self._mirror[tid] = obj
            self._hashes[tid] = row[ti['hash']]
            out.append(obj)
        live = {t.id for t in out}
        for tid in [k for k in self._mirror if k not in live]:
            del self._mirror[tid]
            self._hashes.pop(tid, None)
        for serial in [s for s in self._ext_by_serial if s not in seen_serials]:
            entry = self._ext_by_serial.pop(serial)
            self._serial_of.pop(id(entry), None)
        self.tracks = out
        self._exported_list = out
        self._exported_ids = [id(t) for t in out]
        self._dirty = False
        if self._verify:
            self._verify_mirror()

    def _verify_mirror(self):
        """Check that the mirror re-exports to the library's own state (ENV_VERIFY)."""
        before = self._native.export_tracks()
        saved = (dict(self._hashes),)
        self._push_tracks()
        after = self._native.export_tracks()
        self._hashes = saved[0]
        a, b = before[0].tolist(), after[0].tolist()
        if len(a) != len(b) or any(x[_TI['hash']] != y[_TI['hash']] for x, y in zip(a, b)):
            raise AssertionError('wall tracker mirror diverged from the library state')

    # ---- calls -----------------------------------------------------------

    def _call(self, fn, mutating=True):
        pushed = self._sync_in()
        try:
            return fn()
        finally:
            if mutating or pushed:
                self._pull()

    def _by_ids(self, ids):
        return [self._mirror[i] for i in ids]

    @staticmethod
    def _car(car_pose):
        return float(car_pose.x), float(car_pose.y), float(car_pose.theta)

    def step(self, observations, stamp, car_pose, scan=None, defer_roles=False):
        obs = _observation_array(observations)
        if scan is None:
            native_scan = None
        else:
            r = wtn.f64(scan[0])
            b = wtn.f64(scan[1])
            lxy = np.asarray(scan[2], dtype=float).reshape(2)
            cut = scan[3] if len(scan) > 3 else None
            cut = None if cut is None else np.ascontiguousarray(cut, dtype=np.uint8)
            native_scan = (r, b, (float(lxy[0]), float(lxy[1])), cut)
        car = self._car(car_pose)
        ids = self._call(lambda: self._native.step(obs, float(stamp), car, native_scan,
                                                   defer_roles))
        return [] if defer_roles else self._by_ids(ids)

    def update_roles(self, car_pose):
        car = self._car(car_pose)
        self._call(lambda: self._native.update_roles(car))

    def published(self, car_pose):
        car = self._car(car_pose)
        return self._by_ids(self._call(lambda: self._native.published(car), mutating=False))

    def front_track(self, car_pose, front_x):
        car = self._car(car_pose)
        fid, rejects = self._call(lambda: self._native.front_track(car, float(front_x)),
                                  mutating=False)
        s = self._native.get_scalars()
        best = None if fid is None else self._mirror[fid]
        self.front_status = {
            'id': fid,
            'reason': FRONT_NAMES[int(s['front_reason'])],
            'distance_m': float(s['front_distance_m']),
            'rejects': {tid: FRONT_NAMES[code] for tid, code in rejects}}
        return best

    def apply_rect_verdicts(self, verdicts):
        ids = list(verdicts)
        values = [(float(verdicts[i]['coverage']), float(verdicts[i]['support']),
                   float(verdicts[i]['beyond_share'])) for i in ids]
        self._call(lambda: self._native.apply_rect_verdicts(
            np.array(ids, dtype=np.int64), np.array(values, dtype=float).reshape(-1, 3)))

    def _spawn(self, line, stamp, obs=None):
        row = np.zeros(1, dtype=_LINE_DTYPE)
        r = row[0]
        cov = np.asarray(line.cov, dtype=float).reshape(2, 2)
        r['rho'], r['alpha'] = float(line.rho), float(line.alpha)
        r['cov00'], r['cov01'], r['cov10'], r['cov11'] = cov.ravel()
        r['sx'], r['sy'] = np.asarray(line.p_start, dtype=float).reshape(2)
        r['ex'], r['ey'] = np.asarray(line.p_end, dtype=float).reshape(2)
        r['n_points'] = int(line.n_points)
        orow = None if obs is None else _observation_array([obs])
        self._call(lambda: self._native.spawn(row, float(stamp), orow))

    def _merge(self):
        self._call(self._native.merge)


def _observation_array(observations):
    """[wall_fusion.Observation] as the library's structs."""
    out = np.zeros(len(observations), dtype=_OBS_DTYPE)
    for k, o in enumerate(observations):
        wl = o.line
        out[k] = (float(wl.nx), float(wl.ny), float(wl.d), float(wl.t0), float(wl.t1),
                  int(wl.inliers), float(wl.rms), float(o.z_min), float(o.z_max),
                  int(bool(o.height_measured)), int(o.source), float(o.confidence),
                  float(getattr(o, 'stereo_weight', 1.0)))
    return out


# ============================================================================
# The per-cycle fusion (wall_fusion), in the library
# ============================================================================

def verify_rectangles(segments, labels, depth, fx, fy, cx, cy, rotation, translation,
                      laser_z=0.0, p=None, grid=None, object_mask=None, wall_class=None,
                      object_class=None, depth_scale=1.0, detail=None, detail_stride=1):
    """
    wall_fusion.verify_rectangles, computed by the library.

    Same signature and result. The library takes the node's case -- a uint8
    class image, a uint16 depth image, no separate object mask, no `detail`
    -- and anything else goes to the Python unchanged.
    """
    lab, dep = np.asarray(labels), np.asarray(depth)
    if (object_mask is not None or detail is not None or lab.ndim != 2 or lab.dtype != np.uint8
            or dep.dtype != np.uint16 or dep.shape[:2] != lab.shape):
        return wall_fusion.verify_rectangles(
            segments, labels, depth, fx, fy, cx, cy, rotation, translation, laser_z=laser_z,
            p=p, grid=grid, object_mask=object_mask, wall_class=wall_class,
            object_class=object_class, depth_scale=depth_scale, detail=detail,
            detail_stride=detail_stride)
    p = p or wall_fusion.RectParams()
    if len(segments) == 0:
        return []
    grid = grid if grid is not None else wall_fusion.rect_grid(p)
    segs = [[float(a[0]), float(a[1]), float(b[0]), float(b[1])] for a, b in segments]
    rot = np.asarray(rotation, dtype=float).reshape(3, 3)
    trans = np.asarray(translation, dtype=float).reshape(3)
    cam = [float(fx), float(fy), float(cx), float(cy), *rot.ravel().tolist(),
           *trans.tolist(), float(laser_z)]
    prm = [float(p.t_step_min_m), float(int(p.t_steps_max)), float(p.tol_base_m),
           float(p.tol_quad_per_m2), float(p.min_depth_m), float(p.max_range_m)]
    rows = wtn.verify_rectangles(
        segs, lab, dep, cam, prm, grid[0], grid[1], depth_scale,
        scene_mask.CLASS_WALL if wall_class is None else wall_class,
        scene_mask.CLASS_OBJECT if object_class is None else object_class)
    return [{'coverage': c, 'support': s, 'beyond_share': b, 'n_samples': ns, 'n_in_image': ni,
             'at': at, 'beyond': be, 'against': ag}
            for c, s, b, ns, ni, at, be, ag in rows.tolist()]


_FUSION_IN_DTYPE = wtn._dtype(wtn.FIELDS[wtn.STRUCT_FUSION_IN])
_ROT_NAMES = ('r00', 'r01', 'r02', 'r10', 'r11', 'r12', 'r20', 'r21', 'r22')


def fusion_cycle(lines, pts, laser_z, camera, width, distance_m, z_min, z_max, compensation,
                 cloud, *, tol_base_m, tol_quad_per_m2, disagree_reject_frac, stereo_weight,
                 default_height, conf_range_ref_m, conf_min_inlier_ref, conf_max_rms_scale,
                 stereo_min_columns, inlier_m, min_span_m, inlier_ref=40, rms_ref=0.03,
                 min_points=40, max_pairs=300):
    """
    lidar_front_wall_node._publish_observations' fused branch, by the library.

    The same computation as `wall_fusion.cross_check_profile` +
    `observations_from_lines` + `stereo_only_observations` with the node's
    projection and odometry compensation (`camera` = (fx, fy, cx, cy, R, t);
    `compensation` = (pose_scan, pose_image) or None; `cloud` the WALL
    points, (M, 3), or None). Returns (observations, n_rejected, check), the
    observations wall_fusion.Observation objects: the LiDAR ones carry the
    very WallLine objects of `lines`, the stereo-only ones a new WallLine.
    """
    from f1tenth_perception.scene_labels import WallLine
    fx, fy, cx, cy, rot, trans = camera
    rot = np.asarray(rot, dtype=float).reshape(3, 3)
    trans = np.asarray(trans, dtype=float).reshape(3)
    row = np.zeros(1, dtype=_FUSION_IN_DTYPE)
    r = row[0]
    r['fx'], r['fy'], r['cx'], r['cy'] = float(fx), float(fy), float(cx), float(cy)
    for name, v in zip(_ROT_NAMES, rot.ravel().tolist()):
        r[name] = v
    r['t0'], r['t1'], r['t2'] = trans.tolist()
    r['laser_z'] = float(laser_z)
    r['width'] = int(width)
    if compensation is not None:
        ps, pim = compensation
        r['has_comp'] = 1
        r['scan_x'], r['scan_y'], r['scan_theta'] = float(ps.x), float(ps.y), float(ps.theta)
        r['image_x'], r['image_y'] = float(pim.x), float(pim.y)
        r['image_theta'] = float(pim.theta)
    r['tol_base'], r['tol_quad'] = float(tol_base_m), float(tol_quad_per_m2)
    r['disagree_reject_frac'] = float(disagree_reject_frac)
    r['stereo_weight'] = float(stereo_weight)
    r['inlier_ref'], r['rms_ref'] = float(inlier_ref), float(rms_ref)
    r['range_ref_m'] = float(conf_range_ref_m)
    r['min_inlier_ref'] = float(conf_min_inlier_ref)
    r['max_rms_scale'] = float(conf_max_rms_scale)
    r['min_span_columns'] = int(stereo_min_columns)
    r['inlier_m'] = float(inlier_m)
    r['min_points'] = int(min_points)
    r['min_span_m'] = float(min_span_m)
    r['default_height'] = float(default_height)
    r['max_pairs'] = int(max_pairs)
    line_rows = [(float(wl.nx), float(wl.ny), float(wl.d), float(wl.t0), float(wl.t1),
                  float(int(wl.inliers)), float(wl.rms)) for wl in lines]
    status, lidar_m, stereo_m, tol_m, obs, rejected = wtn.fusion_cycle(
        row, np.array(line_rows, dtype=float).reshape(-1, 7),
        np.asarray(pts, dtype=float).reshape(-1, 2), distance_m, z_min, z_max, cloud)
    out = []
    for (li, nx, ny, d, t0, t1, inl, rms, zlo, zhi, hm, src, conf, c0, c1, na, nlo, nso, nd,
         sw) in obs.tolist():
        wl = lines[li] if li >= 0 else WallLine(nx, ny, d, t0, t1, inl, rms, 'stereo')
        out.append(wall_fusion.Observation(
            wl, zlo, zhi, bool(hm), src, conf, col_begin=c0, col_end=c1,
            counts={'agree': na, 'lidar_only': nlo, 'stereo_only': nso, 'disagree': nd},
            stereo_weight=sw))
    check = {'status': status, 'lidar_m': lidar_m, 'stereo_m': stereo_m,
             'n_px': np.zeros(int(width), dtype=np.int64), 'tol_m': tol_m}
    return out, rejected, check
