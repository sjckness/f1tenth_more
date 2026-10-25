"""
Exact, pickle-free (de)serialisation of solve_mpc_step() inputs/outputs.

Runs on: the Orin host (native Humble, Python 3.10), the harness container
(Python 3.12), linus. Written by mpc_capture_node.py (under ROS), read by
mpc_timing.py. Plain Python 3 + NumPy only, no f-string or NumPy features
newer than Python 3.6 / NumPy 1.17, so either side can load what the other
wrote.

The value trees solve_mpc_step takes (x0 arrays, the corridor dict, params/
limits/weights dicts, obstacle and boundary tuple lists, the warm-start
vector) are encoded as JSON with every leaf kept exact:
  float / numpy floating -> {"f": float.hex(x)}     (bit-exact, NaN/inf safe)
  numpy ndarray          -> {"nd": name}             (array stored in the .npz)
  bool / int / str / None-> as is (numpy integers/bools -> Python)
  tuple                  -> {"t": [...]},  list -> [...]
  dict                   -> {"d": [[key, value], ...]}   (keeps key order)
Each tick's JSON is stored as a uint8 array (fixed-width unicode arrays would
pad every tick to the longest one).
"""
import json

import numpy as np


def _enc(v, arrays, prefix, counter):
    if isinstance(v, np.ndarray):
        name = '%s_a%d' % (prefix, counter[0])
        counter[0] += 1
        arrays[name] = np.array(v, copy=True)
        return {'nd': name}
    if isinstance(v, (bool, np.bool_)):
        return bool(v)
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        return {'f': float(v).hex()}
    if v is None or isinstance(v, str):
        return v
    if isinstance(v, tuple):
        return {'t': [_enc(x, arrays, prefix, counter) for x in v]}
    if isinstance(v, list):
        return [_enc(x, arrays, prefix, counter) for x in v]
    if isinstance(v, dict):
        return {'d': [[k, _enc(x, arrays, prefix, counter)] for k, x in v.items()]}
    # Anything else (a solver result object, a dataclass): kept as its repr so a
    # capture never fails on an info field newer than this encoder. Lossy, and
    # decoded as that string.
    return {'repr': '%s: %r' % (type(v).__name__, v)}


def _dec(v, arrays):
    if isinstance(v, list):
        return [_dec(x, arrays) for x in v]
    if isinstance(v, dict):
        if 'f' in v:
            return float.fromhex(v['f'])
        if 'nd' in v:
            return np.array(arrays[v['nd']], copy=True)
        if 't' in v:
            return tuple(_dec(x, arrays) for x in v['t'])
        if 'd' in v:
            return {k: _dec(x, arrays) for k, x in v['d']}
        if 'repr' in v:
            return v['repr']
    return v


def encode(value, arrays, prefix):
    """
    Encode `value` into a JSON uint8 array.

    ndarray leaves go into `arrays` under names starting with `prefix`.
    Returns the uint8 array.
    """
    tree = _enc(value, arrays, prefix, [0])
    return np.frombuffer(json.dumps(tree).encode('utf-8'), dtype=np.uint8).copy()


def decode(json_u8, arrays):
    return _dec(json.loads(bytes(bytearray(json_u8)).decode('utf-8')), arrays)


def save_ticks(path, ticks, meta):
    """
    Save ticks (a list of {name: value}) and meta ({key: str}) as one .npz.

    Each named value is encoded separately as '<name>_json_<i>'; meta is
    stored as 'meta_<key>'.
    """
    arrays = {}
    for i, tick in enumerate(ticks):
        for name, value in tick.items():
            arrays['%s_json_%d' % (name, i)] = encode(value, arrays, '%s_%d' % (name, i))
    arrays['n_ticks'] = np.array(len(ticks), dtype=np.int64)
    for k, v in meta.items():
        arrays['meta_' + k] = np.array(str(v))
    np.savez_compressed(path, **arrays)


def load_ticks(path, names):
    """-> (list of {name: value}, {meta_key: str}, raw npz dict)."""
    raw = dict(np.load(path, allow_pickle=False))
    n = int(raw['n_ticks'])
    ticks = []
    for i in range(n):
        tick = {}
        for name in names:
            key = '%s_json_%d' % (name, i)
            if key in raw:
                tick[name] = decode(raw[key], raw)
        ticks.append(tick)
    meta = {k[5:]: str(v) for k, v in raw.items() if k.startswith('meta_')}
    return ticks, meta, raw
