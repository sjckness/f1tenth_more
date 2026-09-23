"""Read corridors.jsonl, of either schema, and re-evaluate the v2 ones.

TWO SCHEMAS, and a reader has to tell them apart or it will report confident
nonsense:

``v1`` (everything logged before the schema change, including all 13 runs in
first_test_campaing/) carries ONLY a sampled boundary polygon. The two walls
are in it; the centreline is not, and neither are the parameters the curves
were built from. A v1 corridor can be drawn as a filled band and measured
against, and that is all -- its centreline is genuinely not recoverable, and
for a wall_turn of 180 degrees or more not even the direction of the bend is
(``psi_ref`` alone cannot say which way round; see MPC_corr's own note).

``mpc_corr/v2`` carries the function definition at full float precision, so
:func:`evaluate` reproduces the planner's own arrays exactly.

"Exactly" means AT THE STORED ``corr_N``. The centreline is a cumsum Riemann
sum, so re-evaluating at a higher n gives a different -- more accurate, but
different -- curve, which is not what the car drove. :func:`evaluate` never
resamples it. See corridor_geometry's docstring.

The evaluator lives in f1tenth_params, the dependency-free leaf package both
this package and mpc_controller already depend on. It cannot live in
mpc_controller: that package test_depends on llm and f1tenth_behavior, which
depend on f1tenth_logger, so an f1tenth_logger -> mpc_controller edge closes a
cycle and colcon refuses to order the workspace. Same rule, and same leaf, as
f1tenth_params.object_geometry.

numpy and the evaluator are imported lazily, inside :func:`evaluate`, so that
merely LOADING a corridors.jsonl (which is what the export does, on a machine
that may have neither) stays stdlib-only.
"""

from __future__ import annotations

import json

__all__ = [
    'CorridorRecord',
    'SCHEMA_V1',
    'SCHEMA_V2',
    'evaluate',
    'load_corridors',
    'schema_of',
]

SCHEMA_V1 = 'v1'
SCHEMA_V2 = 'mpc_corr/v2'

#: Keys corridor_geometry.corridor_curves cannot be called without.
_REQUIRED = ('C0', 'psiStart', 'psiEnd', 'L', 'corr_N')


def schema_of(record):
    """``SCHEMA_V2`` if the record carries a usable definition, else ``SCHEMA_V1``.

    Structural, not a version-string comparison: a record that claims v2 but is
    missing a parameter cannot be evaluated, so it is treated as v1 rather than
    being allowed to fail later inside the evaluator.
    """
    defn = record.get('definition') or (record.get('meta') or {}).get('definition')
    if not isinstance(defn, dict):
        return SCHEMA_V1
    if any(defn.get(key) is None for key in _REQUIRED):
        return SCHEMA_V1
    return SCHEMA_V2


class CorridorRecord:
    """One line of corridors.jsonl, with the two schemas normalised.

    robot_logger writes the publisher's payload split across the top level
    (``t``, ``id``, ``source``, ``polygon``) and a ``meta`` blob holding
    whatever else the publisher sent, so ``definition`` can arrive in either
    place depending on which side was updated first. Both are accepted.
    """

    __slots__ = ('t', 'id', 'source', 'polygon', 'meta', 'definition', 'schema')

    def __init__(self, raw):
        meta = raw.get('meta') or {}
        self.t = raw.get('t')
        self.id = raw.get('id')
        self.source = raw.get('source') or meta.get('source')
        self.polygon = [(float(x), float(y)) for x, y in (raw.get('polygon') or [])]
        self.meta = meta
        self.schema = schema_of(raw)
        self.definition = (
            raw.get('definition') or meta.get('definition')
            if self.schema == SCHEMA_V2 else None
        )

    @property
    def length_m(self):
        if self.definition:
            return self.definition.get('L')
        return self.meta.get('length_m')

    @property
    def dpsi(self):
        """Signed rotation the corridor asks for, or None on a v1 record.

        v1 records cannot answer this: psi_ref is an absolute heading and the
        start heading was never logged. Recovering it from the polygon is
        possible to within its rounding but is a different number from the one
        the planner used, so it is not done here.
        """
        return None if self.definition is None else self.definition.get('dpsi')

    @property
    def object_mode(self):
        return bool(self.meta.get('object_mode', False))

    @property
    def object_shape(self):
        """'arc', 'straight' or 'none'; 'unknown' on a record that predates it.

        NOT inferred from object_mode: a v1 object corridor was always straight
        but did not say so, and guessing 'straight' for it would make the
        campaign columns claim a shape the log never recorded.
        """
        return str(self.meta.get('object_shape') or 'unknown')

    @property
    def corridor_mode(self):
        """The object_corridor_mode this corridor was built under, or None.

        The MODE is the setting ('off' | 'arc' | 'arc_far'); object_shape is
        what that setting produced on this rebuild ('straight' | 'arc'). They
        are not the same fact and the campaign carries both: 'arc_far' that
        stayed straight all run is a different thing from 'off', and only the
        mode distinguishes them. Read from the v2 definition, so None on a v1
        record and on anything logged before the mode existed.
        """
        if not self.definition:
            return None
        mode = self.definition.get('object_corridor_mode')
        return str(mode) if mode else None

    @property
    def cut(self):
        """True when the length cap shortened this corridor. None on v1.

        A cut corridor does NOT end where it was aiming: Pend is a waypoint on
        the way to the target rather than the target's standoff. Counting cuts
        is how a campaign sees that regime at all -- the length alone cannot
        say whether 3.00 m is the nominal corridor or a 4.63 m one truncated.
        """
        if not self.definition:
            return None
        return bool(self.definition.get('cut', False))

    @property
    def ref_step(self):
        """How far the reference moved since the previous rebuild, or {}.

        Empty on v1 and on the first corridor of a move (nothing to difference
        against), so a caller must treat a missing value as unmeasured rather
        than as zero.
        """
        step = self.meta.get('ref_step')
        return dict(step) if isinstance(step, dict) else {}

    def __repr__(self):
        return (f'<CorridorRecord id={self.id} t={self.t} schema={self.schema} '
                f'{len(self.polygon)} boundary samples>')


def load_corridors(path):
    """Every well-formed line of a corridors.jsonl, as CorridorRecords.

    A malformed line is skipped rather than raising: the file is appended to
    live by a running node, so a truncated last line is expected, not a fault.
    A missing file is an empty list -- a test that logged no corridor is a
    real outcome the plot has to render, not an error.
    """
    records = []
    try:
        handle = open(path, 'r', encoding='utf-8')
    except (FileNotFoundError, NotADirectoryError, IsADirectoryError):
        return records
    with handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                raw = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(raw, dict):
                records.append(CorridorRecord(raw))
    return records


def evaluate(record, n=None):
    """The planner's own curves, re-derived from a v2 record's definition.

    Returns corridor_geometry.corridor_curves' dict, or None for a v1 record.

    ``n`` overrides the sample count and exists for tests that want to show the
    cumsum's dependence on it. LEAVE IT NONE for anything that claims to show
    what the planner computed: at any other n the centreline is a different
    curve.
    """
    if record.schema != SCHEMA_V2:
        return None
    from f1tenth_params.corridor_geometry import (  # noqa: PLC0415 - see docstring
        CORRIDOR_HANDLE_FRAC, POSE_HANDLE_FRAC, corridor_curves,
        corridor_curves_to_pose,
    )
    d = record.definition
    c0 = d['C0']
    # TWO KINDS OF CENTRELINE, and the record says which. 'bezier' ends ON the
    # pose in C1 and so carries no usable L (its length is an output of the
    # fit); 'ramp', the only kind before the object branch learned to end on
    # its target, is the integrated heading ramp of length L. A record with no
    # 'centreline' key predates the distinction and can only be a ramp.
    if d.get('centreline') == 'bezier':
        c1 = d['C1']
        return corridor_curves_to_pose(
            float(c0[0]), float(c0[1]), float(d['psiStart']),
            float(c1[0]), float(c1[1]), float(d['psiEnd']),
            int(d['corr_N'] if n is None else n),
            handle_a=float(d.get('handle_a') or POSE_HANDLE_FRAC),
            handle_b=float(d.get('handle_b') or POSE_HANDLE_FRAC),
            w0=float(d.get('w0', 0.4333)),
            w1=float(d.get('w1', 0.7667)),
            handle_frac=float(d.get('handle_frac', CORRIDOR_HANDLE_FRAC)),
        )
    # dpsi is passed from the record rather than left to the evaluator's own
    # wrap: it is the RESOLVED value the planner used, which for a wall_turn is
    # signed and unwrapped and so is not equal to wrap(psiEnd - psiStart) past
    # half a turn. Falling back to None reproduces the non-turn branches.
    dpsi = d.get('dpsi')
    if dpsi is None:
        dpsi = d.get('psiRefTurn')
    return corridor_curves(
        float(c0[0]), float(c0[1]),
        float(d['psiStart']), float(d['psiEnd']),
        float(d['L']), int(d['corr_N'] if n is None else n),
        dpsi=(None if dpsi is None else float(dpsi)),
        u_start=float(d.get('u_start', 0.0)),
        u_end=float(d.get('u_end', 0.40)),
        w0=float(d.get('w0', 0.4333)),
        w1=float(d.get('w1', 0.7667)),
        handle_frac=float(d.get('handle_frac', CORRIDOR_HANDLE_FRAC)),
    )
