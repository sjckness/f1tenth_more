"""The diagnostics wire format: the two halves of one round-trip contract.

``DiagnosticArray`` values are strings, and :mod:`go_to_object.replay` reads
its tick schedule out of them. A float that is serialized with anything less
than full precision -- ``%.4f``, a bare f-string format spec added for
readability, ``str()`` on a path that happens to shorten -- makes replay
*approximately* identical instead of bit-identical. The exactness check in
``compare`` then reports serialization artifacts as divergences, and the
first bag sends someone hunting a node bug that does not exist.

So the serializer and the parser live together, in one small module both the
node and the replay import, and the round trip is tested rather than assumed.

``repr`` is exact for Python floats: CPython emits the shortest string that
round-trips to the identical double, which is what ``float()`` then recovers.
``%.17g`` would also be exact but is longer and no more correct.

The one value that does not survive an equality check is NaN, and it is not a
precision problem: ``repr(nan)`` -> ``'nan'`` -> ``float('nan')`` reproduces
a NaN faithfully, but ``nan != nan`` by IEEE-754, so a comparison will flag it
whatever we do. Negative zero *does* round-trip with its sign intact, though
``-0.0 == 0.0`` means a comparison will not notice either way.

Readability of ``ros2 topic echo`` is not a reason to lose precision here.
Anything that wants a rounded value gets its own separate field.
"""

from __future__ import annotations

__all__ = ['NONE', 'format_float', 'parse_float',
           'format_optional_float', 'parse_optional_float']

NONE = 'none'
"""Sentinel for a field that had no value on this tick."""


def format_float(value) -> str:
    """Serialize exactly.

    The ``float()`` coercion is deliberate: a numpy scalar reaching this by
    accident would otherwise be serialized by numpy's own ``repr``, which is
    bare digits under numpy 1.x but ``np.float64(0.1)`` under numpy 2.x --
    unparseable, and a silent break on a library upgrade rather than at the
    call site.
    """
    return repr(float(value))


def parse_float(text: str) -> float:
    return float(text)


def format_optional_float(value) -> str:
    return NONE if value is None else format_float(value)


def parse_optional_float(text: str) -> float | None:
    return None if text in ('', NONE) else parse_float(text)
