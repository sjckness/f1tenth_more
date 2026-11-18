r"""pytest plugin: every corridor call the tests make, replayed through Rust in process.

    PYTHONPATH=tools/rust_parity/corridor:$PYTHONPATH \\
    F1TENTH_CORRIDOR_PARITY_OUT=<dir> F1TENTH_CORRIDOR_PARITY_MODE=capture \\
    python3 -m pytest -p corridor_capture_plugin <test dirs>

MODE capture (default). The Python reference runs exactly as it always does
and its result is what the caller gets: the plugin only wraps
f1tenth_params.corridor_geometry.corridor_curves / corridor_curves_to_pose
and mpc_controller.mpc_solver's four corridor queries (nearest_centerline_index,
corridor_lateral_coordinates, corridor_heading_at, _nearest_corridor_frame).
After each Python call the SAME inputs go through the Rust library
(mpc_native) and the two results are compared on the spot
(corridor_compare.compare_dicts: bit identity counted, 1e-12 the gate). An
exception must be the same type with the same message on both sides. The
wrappers are installed at pytest_configure, before any test module is
imported, so `from ... import corridor_curves` in a test or a rig binds the
wrapper too. Every geometry call is kept (deduplicated) in corpus.pkl for an
offline replay (parity_corridor.py --replay); query calls are compared all and
kept as a sample.

MODE substitute. The Python functions are REPLACED by the Rust ones (the same
seven names), so the whole suite -- test_corridor_geometry.py,
reverse_manoeuvre, corridor_def, the solver's own queries, every rig -- runs on
the Rust library. Used to show the test outcomes do not change
(run_corridor_tests.sh). Calls are counted, nothing is compared.

Output (F1TENTH_CORRIDOR_PARITY_OUT): summary.json, corpus.pkl (capture).
"""

import collections
import json
import os
import pickle
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import corridor_compare as cc  # noqa: E402, I100

MODE = os.environ.get('F1TENTH_CORRIDOR_PARITY_MODE', 'capture')
OUT = os.environ.get('F1TENTH_CORRIDOR_PARITY_OUT', '')
#: Query calls kept in the corpus (all are compared).
QUERY_SAMPLE_FIRST = 3000
QUERY_SAMPLE_EVERY = 500
GEOMETRY_CORPUS_CAP = 300_000

_STATE = {
    'started': None,
    'geometry': collections.defaultdict(cc.Tally),       # site -> Tally
    'geometry_calls': collections.Counter(),             # name -> calls
    'query': collections.defaultdict(cc.Tally),          # kind -> Tally
    'query_calls': collections.Counter(),
    'query_py_raised': collections.Counter(),
    'query_dtypes': collections.Counter(),
    'substituted_calls': collections.Counter(),
    'corpus_geometry': {},     # pickled call -> (name, args, kwargs, site, count)
    'corpus_query': [],
    'mismatch': [],
}
_SKIP_FRAMES = ('corridor_backend.py', 'corridor_capture_plugin.py', 'corridor_geometry.py',
                'mpc_solver.py')


def _site():
    """file:function of the first caller outside the backend and this plugin."""
    f = sys._getframe(2)
    while f is not None and os.path.basename(f.f_code.co_filename) in _SKIP_FRAMES[:2]:
        f = f.f_back
    if f is None:
        return '?'
    path = f.f_code.co_filename
    marker = os.sep + 'src' + os.sep
    if marker in path:
        path = path.split(marker, 1)[1]
    return f'{path}:{f.f_code.co_name}'


def _geometry_wrapper(name, py_fn, rs_fn):
    def wrapped(*args, **kwargs):
        site = _site()
        _STATE['geometry_calls'][name] += 1
        try:
            result = py_fn(*args, **kwargs)
        except Exception as exc:
            _after_geometry(name, site, args, kwargs, None, exc, rs_fn)
            raise
        _after_geometry(name, site, args, kwargs, result, None, rs_fn)
        return result
    wrapped.__wrapped__ = py_fn
    wrapped.__name__ = name
    return wrapped


def _after_geometry(name, site, args, kwargs, py, py_exc, rs_fn):
    tally = _STATE['geometry'][f'{name} @ {site}']
    rs = rs_exc = None
    try:
        rs = rs_fn(*args, **kwargs)
    except Exception as exc:          # noqa: BLE001 - compared below
        rs_exc = exc
    if py_exc is not None or rs_exc is not None:
        if cc.exc_key(py_exc) == cc.exc_key(rs_exc):
            tally.exceptions_matched += 1
        else:
            tally.exception_mismatch.append(
                (repr(args), repr(kwargs), cc.exc_key(py_exc), cc.exc_key(rs_exc)))
            _STATE['mismatch'].append(('geometry', name, site, repr(args), repr(kwargs),
                                       cc.exc_key(py_exc), cc.exc_key(rs_exc)))
    else:
        result = cc.compare_dicts(py, rs)
        cc.add_to(tally, f'{name}{args!r}{kwargs!r}', result)
        if not result['ok']:
            _STATE['mismatch'].append(('geometry', name, site, repr(args), repr(kwargs),
                                       result['reasons']))
    try:
        key = pickle.dumps((name, args, sorted(kwargs.items())))
    except Exception:                 # noqa: BLE001 - an unpicklable input is just not kept
        return
    corpus = _STATE['corpus_geometry']
    if key in corpus:
        corpus[key][4] += 1
    elif len(corpus) < GEOMETRY_CORPUS_CAP:
        corpus[key] = [name, args, kwargs, site, 1]


def _query_inputs(p, corridor):
    keys = ('xc', 'yc', 'nx', 'ny', 'halfWidth')
    return (np.array(p, dtype=float, copy=True),
            {k: np.array(corridor[k], copy=True) for k in keys if k in corridor})


def _query_wrapper(kind, py_fn):
    def wrapped(p, corridor, *rest, **kw):
        result = py_fn(p, corridor, *rest, **kw)
        _after_query(kind, p, corridor, result)
        return result
    wrapped.__wrapped__ = py_fn
    wrapped.__name__ = py_fn.__name__
    return wrapped


def _rs_query(kind, p, corridor):
    from mpc_controller import mpc_native
    if kind == 'nearest_centerline_index':
        return mpc_native.nearest_centerline_index(p, corridor)
    f = mpc_native.corridor_frame(p, corridor)
    if kind == 'corridor_lateral_coordinates':
        return (f['d_lat'], f['half_w'])
    if kind == 'corridor_heading_at':
        return f['heading']
    return (f['pc'], f['n'], f['half_w'])


def _after_query(kind, p, corridor, py):
    _STATE['query_calls'][kind] += 1
    n = _STATE['query_calls'][kind]
    dtypes = tuple(str(getattr(corridor.get(k), 'dtype', type(corridor.get(k)).__name__))
                   for k in ('xc', 'yc'))
    _STATE['query_dtypes'][dtypes] += 1
    tally = _STATE['query'][kind]
    try:
        rs = _rs_query(kind, p, corridor)
    except Exception as exc:          # noqa: BLE001
        tally.exception_mismatch.append((kind, cc.exc_key(None), cc.exc_key(exc)))
        _STATE['mismatch'].append(('query', kind, repr(exc)))
        return
    result = cc.compare_dicts(py, rs)
    cc.add_to(tally, kind, result)
    keep = n <= QUERY_SAMPLE_FIRST or n % QUERY_SAMPLE_EVERY == 0 or not result['ok']
    if not result['ok']:
        _STATE['mismatch'].append(('query', kind, result['reasons']))
    if keep:
        try:
            _STATE['corpus_query'].append((kind,) + _query_inputs(p, corridor))
        except Exception:             # noqa: BLE001
            pass


def _substitute_wrapper(name, rs_fn):
    def wrapped(*args, **kwargs):
        _STATE['substituted_calls'][name] += 1
        return rs_fn(*args, **kwargs)
    wrapped.__name__ = name
    return wrapped


def install():
    from f1tenth_params import corridor_geometry as cg
    from mpc_controller import mpc_native, mpc_solver
    if not mpc_native.available():
        raise RuntimeError(
            f'corridor parity: Rust library not loadable: {mpc_native.load_error()}')
    _STATE['started'] = time.time()
    _STATE['library'] = mpc_native.library_path()
    _STATE['build'] = mpc_native.build_info()
    if MODE == 'substitute':
        cg.corridor_curves = _substitute_wrapper('corridor_curves', mpc_native.corridor_curves)
        cg.corridor_curves_to_pose = _substitute_wrapper(
            'corridor_curves_to_pose', mpc_native.corridor_curves_to_pose)
        for kind in ('nearest_centerline_index', 'corridor_lateral_coordinates',
                     'corridor_heading_at', '_nearest_corridor_frame'):
            setattr(mpc_solver, kind, _substitute_wrapper(
                kind, lambda p, corridor, _k=kind: _rs_query(_k, p, corridor)))
        return
    if MODE != 'capture':
        raise RuntimeError(f'F1TENTH_CORRIDOR_PARITY_MODE must be capture or substitute, '
                           f'got {MODE!r}')
    cg.corridor_curves = _geometry_wrapper(
        'corridor_curves', cg.corridor_curves, mpc_native.corridor_curves)
    cg.corridor_curves_to_pose = _geometry_wrapper(
        'corridor_curves_to_pose', cg.corridor_curves_to_pose,
        mpc_native.corridor_curves_to_pose)
    for kind in ('nearest_centerline_index', 'corridor_lateral_coordinates',
                 'corridor_heading_at', '_nearest_corridor_frame'):
        setattr(mpc_solver, kind, _query_wrapper(kind, getattr(mpc_solver, kind)))


def summary():
    def tallies(d):
        return {k: v.as_dict() for k, v in sorted(d.items())}
    return {
        'mode': MODE,
        'library': _STATE.get('library'),
        'build': _STATE.get('build'),
        'seconds': None if _STATE['started'] is None else time.time() - _STATE['started'],
        'geometry_calls': dict(_STATE['geometry_calls']),
        'geometry_distinct': len(_STATE['corpus_geometry']),
        'geometry': tallies(_STATE['geometry']),
        'query_calls': dict(_STATE['query_calls']),
        'query': tallies(_STATE['query']),
        'query_dtypes': {' / '.join(k): v for k, v in _STATE['query_dtypes'].items()},
        'substituted_calls': dict(_STATE['substituted_calls']),
        'mismatches': len(_STATE['mismatch']),
        'mismatch_examples': [repr(m)[:400] for m in _STATE['mismatch'][:30]],
    }


def write(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'summary.json'), 'w') as f:
        json.dump(summary(), f, indent=1, default=str)
    if MODE == 'capture':
        corpus = {
            'geometry': [tuple(v) for v in _STATE['corpus_geometry'].values()],
            'query': _STATE['corpus_query'],
        }
        with open(os.path.join(out_dir, 'corpus.pkl'), 'wb') as f:
            pickle.dump(corpus, f, protocol=pickle.HIGHEST_PROTOCOL)


def pytest_configure(config):
    install()


def pytest_unconfigure(config):
    if OUT:
        write(OUT)
