"""Captured MPC RTI ticks for the R2 solver parity: file format, writer, reader.

One file, ``tools/rust_parity/mpc/data/rti_ticks.jsonl.gz``, written by
``profile_rti.py``. It holds every input ``mpc_solver.solve_mpc_step`` reads on
the RTI path and every output it returns, per tick, so a later parity layer
can feed the identical tick to the Python and to the Rust core and diff them.

FORMAT (version 1)
------------------
gzip (mtime 0, no file name, level 9) of UTF-8 JSON Lines: the same inputs
always give the same bytes.

Line 1 is the header object:
    format       "f1tenth_mpc_rti_ticks"
    version      1
    created_by   "tools/rust_parity/mpc/profile_rti.py"
    git_head     commit the capture ran on (provenance, read-only git)
    sources      sha256 of mpc_solver.py, vehicle_model.py, object_geometry.py
    solver_front_point_offsets_m   SOLVER_FRONT_POINT_OFFSETS_M at capture
    stack_params the stack_params.yaml values the scenarios read
    osqp_settings  every OSQPSettings field as OSQP C received it
    osqp         {"python": "1.1.3", "c": "1.0.0", "infty": 1e30, ...}
    numpy, scipy versions; count; groups {group: n}

Lines 2.. are one tick each:
    id       unique: "matrix/<speed>/obs<n>/walls<n>/<tick>",
             "approach/<scenario>/<tick>" or
             "edge/<speed>/<variant>/from<source matrix tick>"
    group    "matrix" (closed loop, the profiled matrix), "approach" (the
             object-approach rig: real corridor builder, legacy speed model)
             or "edge" (a matrix tick with one input changed to reach a
             branch the closed loops do not: no or bad warm start, hard rows,
             infeasible rows, a saturated slack, psiRefTurn, a target line,
             8 slots, ts 0.2 with a 0.1 s control period)
    config   {"speed_model": "legacy"|"offset_free", "obstacles": n,
              "walls": n, "scenario": str, ...} (descriptive only)
    tick     index of the solve within its closed-loop run
    inputs   solve_mpc_step's keyword arguments, ALL of them explicit
             (defaults filled in): x0 (4 or 5), last_u, pref_nom, corridor,
             horizon, ts, params {L, lr}, limits (the 10 keys), weights
             (UNSCALED, as handed to solve_mpc_step; it applies
             scale_stage_weights itself), obstacles [[x, y, r], ...], dmin,
             vdes, solver "rti", boundaries [[nx, ny, offset], ...],
             boundary_max_sources, boundary_hard, boundary_slack_weight,
             boundary_slack_max, warm_start_z (2N list or null),
             control_period, stage_spec (always null here), speed_model
             (null, or {alpha, delay, d, c_lo, c_hi, v_hi (null or N, may
             hold inf), slack_weight}).
             corridor carries only the keys the solver reads: xc, yc, nx,
             ny, halfWidth (arrays), psiRef, psiRefTurn, psiStart,
             targetLine ({p, psi} or null), obstacles_world, car_radius,
             avoidance_margin. A key absent at capture is absent here
             (the solver's .get() defaults then apply, as they did).
    outputs  u0, success, status, status_message, message, cost,
             iterations, zopt (2N), x_pred (N x 4: x_1..x_N of the true-model
             rollout, x0 not included), boundary_slack, and with a speed
             model v_cmd_pred (N) and speed_slack.
    osqp     what the QP looked like: n, m, nnz_P (upper triangle as OSQP
             C took it), nnz_A, iter, status_val, status_polish, rho_updates,
             rho_estimate, obj_val, prim_res, dual_res, qp_sha256.
             qp_sha256 hashes, in this order, P.indptr, P.indices (int64 LE),
             P.data, q, A.indptr, A.indices (int64 LE), A.data, l, u (float64
             LE) exactly as osqp-python handed them to OSQP C (P upper
             triangle, zeros dropped, sorted indices, l/u clamped to
             +-OSQP_INFTY).

NUMBERS. Every float is Python's repr (shortest round-trip), so it reads back
bit-identical in Python's json and in Rust's serde_json with the
``float_roundtrip`` feature (its default parser is not correctly rounded).
Strict JSON has no inf/nan: a non-finite float is the object
``{"$float": "inf"}`` / ``"-inf"`` / ``"nan"``, nothing else uses that key.

Read with ``load(path)`` -> (header, records) and ``solve_kwargs(record)``
-> the keyword arguments for ``solve_mpc_step``, arrays as numpy float64
exactly as the rigs passed them. ``compare_outputs(a, b)`` diffs two output
dicts (bit-identical flag plus the largest absolute difference).
"""

import gzip
import io
import json
import math

import numpy as np

FORMAT = 'f1tenth_mpc_rti_ticks'
VERSION = 1

# corridor keys _solve_rti and planner_cost_corridor read (mpc_solver.py).
CORRIDOR_ARRAY_KEYS = ('xc', 'yc', 'nx', 'ny', 'halfWidth')
CORRIDOR_SCALAR_KEYS = ('psiRef', 'psiRefTurn', 'psiStart', 'car_radius',
                        'avoidance_margin')
CORRIDOR_KEYS = CORRIDOR_ARRAY_KEYS + CORRIDOR_SCALAR_KEYS + (
    'targetLine', 'obstacles_world')

_ARRAY_INPUTS = ('x0', 'last_u', 'pref_nom', 'warm_start_z')
_SPEED_MODEL_ARRAYS = ('c_lo', 'c_hi', 'v_hi')
_TUPLE_LISTS = ('obstacles', 'boundaries')
_OUTPUT_ARRAYS = ('u0', 'zopt', 'x_pred', 'v_cmd_pred')


# ---------------------------------------------------------------- encoding
def _float(v):
    v = float(v)
    if math.isfinite(v):
        return v
    return {'$float': 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')}


def jsonable(obj):
    """Return obj with numpy types, tuples and non-finite floats made JSON-safe."""
    if obj is None or isinstance(obj, (bool, str)):
        return obj
    if isinstance(obj, (np.bool_,)):
        return bool(obj)
    if isinstance(obj, (int, np.integer)):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        return _float(obj)
    if isinstance(obj, np.ndarray):
        return jsonable(obj.tolist())
    if isinstance(obj, dict):
        return {str(k): jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [jsonable(v) for v in obj]
    if hasattr(obj, 'value') and hasattr(obj, 'name'):   # enum (osqp status)
        return int(obj.value)
    raise TypeError(f'cannot encode {type(obj)!r}')


def _hook(d):
    if len(d) == 1 and '$float' in d:
        return float(d['$float'])
    return d


def corridor_view(corridor):
    """The solver-visible part of a corridor dict (keys absent stay absent)."""
    return {k: corridor[k] for k in CORRIDOR_KEYS if k in corridor}


def encode_inputs(kwargs):
    """solve_mpc_step kwargs (all explicit) -> JSON-safe dict."""
    out = {}
    for key, value in kwargs.items():
        if key == 'corridor':
            value = corridor_view(value)
        out[key] = jsonable(value)
    return out


def encode_outputs(u0, info):
    """(u0, info) of solve_mpc_step -> JSON-safe dict."""
    out = {'u0': jsonable(np.asarray(u0, dtype=float))}
    for key in ('success', 'status', 'status_message', 'message', 'cost',
                'iterations', 'zopt', 'x_pred', 'boundary_slack', 'v_cmd_pred',
                'speed_slack'):
        if key in info:
            out[key] = jsonable(info[key])
    return out


def roundtrip(obj):
    """obj through JSON text and back: exactly what a reader of the file sees."""
    return json.loads(json.dumps(jsonable(obj), allow_nan=False), object_hook=_hook)


def outputs_of(u0, info):
    """(u0, info) as a reader of the file would see them (for compare_outputs)."""
    return roundtrip(encode_outputs(u0, info))


def write(path, header, records):
    """Write header + records as gzip JSON Lines (non-finite floats encoded)."""
    buf = io.BytesIO()
    with gzip.GzipFile(filename='', mode='wb', fileobj=buf, mtime=0,
                       compresslevel=9) as gz:
        for obj in [header] + list(records):
            line = json.dumps(jsonable(obj), separators=(',', ':'), allow_nan=False)
            gz.write(line.encode('utf-8'))
            gz.write(b'\n')
    data = buf.getvalue()
    with open(path, 'wb') as fh:
        fh.write(data)
    return data


# ---------------------------------------------------------------- decoding
def load(path):
    """Return (header, [records]) from a fixture file."""
    with gzip.open(path, 'rt', encoding='utf-8') as fh:
        lines = [json.loads(line, object_hook=_hook) for line in fh if line.strip()]
    header, records = lines[0], lines[1:]
    if header.get('format') != FORMAT or header.get('version') != VERSION:
        raise ValueError(f'{path}: not a {FORMAT} v{VERSION} file')
    if header.get('count') != len(records):
        raise ValueError(f'{path}: header count {header.get("count")} != {len(records)}')
    return header, records


def _arr(v):
    return None if v is None else np.asarray(v, dtype=float)


def solve_kwargs(record):
    """Return solve_mpc_step's keyword arguments for one record (fresh objects)."""
    src = record['inputs']
    kw = {}
    for key, value in src.items():
        if key in _ARRAY_INPUTS:
            kw[key] = _arr(value)
        elif key in _TUPLE_LISTS:
            kw[key] = None if value is None else [tuple(float(x) for x in row)
                                                  for row in value]
        elif key == 'corridor':
            corridor = {}
            for ck, cv in value.items():
                if ck in CORRIDOR_ARRAY_KEYS:
                    corridor[ck] = _arr(cv)
                elif ck == 'obstacles_world':
                    corridor[ck] = [tuple(float(x) for x in row) for row in cv]
                elif ck == 'targetLine' and cv is not None:
                    corridor[ck] = {'p': [float(x) for x in cv['p']],
                                    'psi': float(cv['psi'])}
                else:
                    corridor[ck] = cv
            kw[key] = corridor
        elif key == 'speed_model' and value is not None:
            sm = dict(value)
            for sk in _SPEED_MODEL_ARRAYS:
                if sk in sm and sm[sk] is not None:
                    sm[sk] = _arr(sm[sk])
            kw[key] = sm
        elif isinstance(value, dict):
            kw[key] = dict(value)
        else:
            kw[key] = value
    return kw


def _flat(v):
    if v is None:
        return None
    if isinstance(v, (list, tuple, np.ndarray)):
        return np.asarray(v, dtype=float).ravel()
    return None


def compare_outputs(a, b):
    """Diff two encoded-or-raw output dicts.

    Returns {'identical': bool, 'max_abs': float, 'fields': {name: max_abs}}:
    identical means every array element and every scalar is bit-equal (NaN
    equal to NaN) and every non-float field (status, strings, success) equal.
    """
    fields = {}
    identical = True
    for key in sorted(set(a) | set(b)):
        va, vb = a.get(key), b.get(key)
        fa, fb = _flat(va), _flat(vb)
        if fa is not None or fb is not None:
            if fa is None or fb is None or fa.shape != fb.shape:
                fields[key] = math.inf
                identical = False
                continue
            same = np.array_equal(fa.view(np.uint64), fb.view(np.uint64))
            diff = np.abs(fa - fb)
            diff = diff[np.isfinite(diff)]
            fields[key] = float(diff.max()) if diff.size else 0.0
            identical &= bool(same)
        elif isinstance(va, float) or isinstance(vb, float):
            try:
                fa_, fb_ = float(va), float(vb)
            except (TypeError, ValueError):
                fields[key] = math.inf
                identical = False
                continue
            same = (np.float64(fa_).view(np.uint64) == np.float64(fb_).view(np.uint64))
            fields[key] = 0.0 if same else abs(fa_ - fb_)
            identical &= bool(same)
        else:
            same = va == vb
            fields[key] = 0.0 if same else math.inf
            identical &= bool(same)
    finite = [v for v in fields.values() if math.isfinite(v)]
    return {'identical': identical, 'max_abs': max(finite) if finite else 0.0,
            'fields': fields}
