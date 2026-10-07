"""Pick tests out of a campaign with one short string: ``--runs "1-3-4-67-89"``.

Shared by every campaign tool that takes ``--runs`` (the MATLAB exporter now,
the plot tools later), so the syntax means the same thing everywhere::

    "1-3-4-67-89"                        R001 R003 R004 R067 R089
    "R014-R015"                          the same, with the R spelled out
    "P004-R016"                          one prompt's repetition
    "2-P003-R009-20260928T170138-5"      full test IDs mixed in

Tokens are separated by ``-``, ``,`` or whitespace. A full test ID and the
``Pnnn-Rnnn`` short form are recognised as one token even though they contain
``-`` themselves. A bare number or ``Rnnn`` matches that repetition of every
prompt still in the candidate list, so narrow the list first (``--mission``)
when two prompts share repetition numbers.

The result keeps the order the tokens were given in, without duplicates. A
token that matches nothing is an error, and the error names every missing
token together with the runs that were available.
"""

import re

from f1tenth_logger.test_campaign.robot_logger import TEST_ID_RE

_TOKEN_RE = re.compile(
    r"(?P<full>P\d{3}-R\d{3}-\d{8}T\d{6})"
    r"|(?P<short>P\d{3}-R\d{3})"
    r"|(?P<rep>R?\d+)",
    re.IGNORECASE,
)
_SEPARATORS = set("-, \t\n")


class RunSelectionError(ValueError):
    """A ``--runs`` string that is malformed or names runs that do not exist."""


def short_id(test_id):
    """``P003-R009-20260928T170138`` -> ``P003-R009``."""
    match = TEST_ID_RE.match(str(test_id).strip())
    if match is None:
        raise ValueError(f"not a test ID: {test_id!r}")
    return f"P{match.group(1)}-R{match.group(2)}"


def tokenize(spec):
    """Split a ``--runs`` string into ``(kind, text)`` tokens, in order."""
    text = str(spec).strip()
    if not text:
        raise RunSelectionError("empty --runs selection")
    tokens = []
    pos = 0
    while pos < len(text):
        if text[pos] in _SEPARATORS:
            pos += 1
            continue
        match = _TOKEN_RE.match(text, pos)
        end = match.end() if match else pos
        # A token must end at a separator or at the end of the string, so
        # "12x" or "P003-R0091" is rejected rather than half-read.
        if match is None or (end < len(text) and text[end] not in _SEPARATORS):
            stop = pos
            while stop < len(text) and text[stop] not in _SEPARATORS:
                stop += 1
            raise RunSelectionError(
                f"cannot read {text[pos:stop]!r} in --runs {text!r}: expected a "
                f"repetition number (14, R014), a short ID (P004-R014) or a "
                f"full test ID (P004-R014-20261001T112202)"
            )
        tokens.append((match.lastgroup, match.group(0).upper()))
        pos = end
    return tokens


def select_runs(spec, available):
    """The test IDs in ``available`` that ``spec`` names, in ``spec`` order.

    ``available`` is an iterable of full test IDs (folder names). Raises
    RunSelectionError for a malformed string or for tokens that match nothing.
    """
    available = list(available)
    parsed = {}
    for test_id in available:
        match = TEST_ID_RE.match(test_id)
        if match:
            parsed[test_id] = (int(match.group(1)), int(match.group(2)))
    selected = []
    seen = set()
    missing = []
    for kind, token in tokenize(spec):
        if kind == "full":
            hits = [t for t in available if t.upper() == token]
        elif kind == "short":
            hits = [t for t in parsed if short_id(t) == token]
        else:
            rep = int(token.lstrip("R"))
            hits = [t for t, (_, r) in parsed.items() if r == rep]
        if not hits:
            missing.append(f"R{int(token.lstrip('R')):03d}" if kind == "rep" else token)
            continue
        for hit in hits:
            if hit not in seen:
                seen.add(hit)
                selected.append(hit)
    if missing:
        listing = ", ".join(sorted(available)) or "(none)"
        raise RunSelectionError(
            f"--runs names {len(missing)} run(s) that do not exist: "
            f"{', '.join(missing)}\navailable runs: {listing}"
        )
    return selected
