"""Python values -> what scipy.io.savemat writes as clean MATLAB types.

The rules, everywhere in the exporter:

* numbers (int, float, bool) become double; a missing number is NaN, never 0;
* a dict becomes a struct, its keys made into valid MATLAB names;
* a list of numbers becomes a column vector, a list of equal-length number
  lists a matrix (polylines: n x 2);
* a list of dicts that share their keys becomes ONE struct of columns
  ("struct of arrays"), not a cell of structs, so ``s.x`` is a vector;
* a list of strings, or of anything mixed, becomes a cell column;
* ``None`` on its own becomes ``[]``.

``records_to_struct`` applies the same rules across many records (one per
logged message): scalar fields become column vectors, everything
variable-length becomes a cell column with one entry per record, and nested
dicts are flattened into ``outer_inner`` names.
"""

import keyword
import math
import re

import numpy as np

MAX_NAME = 63  # MATLAB namelengthmax


def mat_name(key, taken=()):
    """A valid MATLAB identifier for ``key``, not colliding with ``taken``."""
    name = re.sub(r"[^0-9A-Za-z_]", "_", str(key))
    if not name or not name[0].isalpha():
        name = "x" + name
    if keyword.iskeyword(name):
        name += "_"
    name = name[:MAX_NAME]
    base, n = name, 1
    while name in taken:
        suffix = f"_{n}"
        name = base[:MAX_NAME - len(suffix)] + suffix
        n += 1
    return name


def is_number(value):
    return (isinstance(value, (bool, int, float, np.integer, np.floating, np.bool_))
            and not isinstance(value, str))


def _num(value):
    return math.nan if value is None else float(value)


def empty_column():
    return np.zeros((0, 1))


def cell(items):
    """A cell column (numpy object array, shape n x 1)."""
    out = np.empty((len(items), 1), dtype=object)
    for i, item in enumerate(items):
        out[i, 0] = item
    return out


def str_cell(values):
    return cell(["" if v is None else str(v) for v in values])


def column(values):
    """One field across n records -> n x 1 double, n x 1 cell, or n x k."""
    values = list(values)
    if not values:
        return empty_column()
    if all(v is None or is_number(v) for v in values):
        return np.array([_num(v) for v in values], dtype=float).reshape(-1, 1)
    if all(v is None or isinstance(v, str) for v in values):
        return str_cell(values)
    return cell([to_mat(v) for v in values])


def to_mat(value):
    """One Python/JSON value -> its savemat form (see module docstring)."""
    if value is None:
        return np.zeros((0, 0))
    if isinstance(value, str):
        return value
    if is_number(value):
        return float(value)
    if isinstance(value, np.ndarray):
        if value.dtype.kind in "biuf":
            return value.astype(float).reshape(-1, 1) if value.ndim == 1 else value.astype(float)
        return cell([to_mat(v) for v in value.ravel()])
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            out[mat_name(key, out)] = to_mat(item)
        return out
    if isinstance(value, (list, tuple)):
        items = list(value)
        if not items:
            return empty_column()
        if all(v is None or is_number(v) for v in items):
            return np.array([_num(v) for v in items], dtype=float).reshape(-1, 1)
        if all(isinstance(v, (list, tuple)) and v and all(x is None or is_number(x) for x in v)
               for v in items) and len({len(v) for v in items}) == 1:
            return np.array([[_num(x) for x in v] for v in items], dtype=float)
        if all(isinstance(v, dict) for v in items) and len({tuple(v) for v in items}) == 1:
            keys = list(items[0])
            out = {}
            for key in keys:
                out[mat_name(key, out)] = column([v.get(key) for v in items])
            return out
        if all(isinstance(v, str) for v in items):
            return str_cell(items)
        return cell([to_mat(v) for v in items])
    return str(value)


def flatten_dict(record, prefix=""):
    """``{"a": {"b": 1}, "c": [..]}`` -> ``{"a_b": 1, "c": [..]}``; lists stay whole."""
    flat = {}
    for key, value in record.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict) and value:
            flat.update(flatten_dict(value, name + "_"))
        else:
            flat[name] = value
    return flat


def records_to_struct(records, skip=()):
    """Many JSON records (one per message) -> one struct of n-row columns.

    A field that is a scalar (or missing) in every record becomes a double or
    string column; anything else becomes a cell column, one entry per record.
    Field order follows first appearance.
    """
    flats = [flatten_dict(r) for r in records]
    keys = []
    for flat in flats:
        for key in flat:
            if key not in keys and key not in skip:
                keys.append(key)
    out = {}
    for key in keys:
        values = [flat.get(key) for flat in flats]
        if all(v is None or is_number(v) for v in values):
            col = np.array([_num(v) for v in values], dtype=float).reshape(-1, 1)
        elif all(v is None or isinstance(v, str) for v in values):
            col = str_cell(values)
        else:
            col = cell([to_mat(v) for v in values])
        out[mat_name(key, out)] = col
    return out
