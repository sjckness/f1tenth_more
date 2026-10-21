"""
LLM2's MPC-parameter updates, the controller side (two-llm-arch).

llm2_mpc_param_node (f1tenth_intelligence/llm) publishes f1tenth_messages/
MpcParamUpdate on /llm2/mpc_params: parallel names/values, the names being
mpc_corr's own ROS parameter names (mpc_w_term, ...). With llm2_params_enabled
true MPC_corr subscribes to it (MPC_corr._llm2_setup); this module decides what
may be applied and holds it until the control loop applies it.

  - THE WHITELIST is three parallel ROS parameters, llm2_param_whitelist,
    llm2_param_min and llm2_param_max (stack_params.yaml). The bounds are in
    the DECLARED, unscaled units -- the number `ros2 param get` shows, before
    mpc_solver.scale_stage_weights multiplies the stage terms by 7/N.
  - ONLY COST WEIGHTS can be whitelisted at all: a name has to be one of
    MPC_corr's mpc_w_* parameters, i.e. a key of its self.weights. A whitelist
    naming anything else -- car_radius, avoidance_margin, the steering limits
    and rate bounds, the boundary flags, corridor geometry, vdes -- refuses
    the node's start. Such a weight only reshapes the objective: the input
    and speed boxes and the corridor's lateral bound stay hard rows of the
    QP. NOT mpc_w_obs: the obstacle cost has no hard row behind it (the
    costmap boundary rows are soft by default), so w_obs near 0 would switch
    obstacle avoidance off. It is refused like a safety parameter
    (NEVER_TUNABLE_WEIGHTS).
  - WITH allow_reverse ON, an update may not raise mpc_w_v past
    w_rev / (2 * vdes_default): reverse_mpc relies on w_rev >= 2 * w_v * vdes
    so that reversing never costs less than standing still on the speed term
    alone. Such an update is rejected whole like any other bad value
    (validate_update's w_v_limit).
  - AN UPDATE IS ALL OR NOTHING. One name that is not whitelisted, one value
    that is not a finite number or lies outside its [min, max], rejects the
    WHOLE message. Nothing is clamped: a clamped value is a value nobody
    asked for.
  - THE CALLBACK ONLY PARKS a valid update (PendingUpdate). The control loop
    takes it at the top of its next tick, so a solve never sees the weights
    change half-way through, and one tick sees one weight set.

rclpy-free on purpose, so all of it tests without ROS.
"""

import math
import threading
from typing import Dict, List, Mapping, NamedTuple, Optional, Tuple

#: What llm2_mpc_param_node publishes, and where MPC_corr echoes every update
#: it applied (same seq, the values now in force).
LLM2_PARAMS_TOPIC = '/llm2/mpc_params'
APPLIED_PARAMS_TOPIC = '/mpc/applied_params'

#: self.weights keys that are cost weights but must never be tuned: w_obs, the
#: obstacle cost, has no hard obstacle row behind it.
NEVER_TUNABLE_WEIGHTS = frozenset({'w_obs'})


class Update(NamedTuple):
    """One validated update: the sender's seq and source, {parameter name: value}."""

    seq: int
    source: str
    values: Dict[str, float]


def tunable_weights(weights: Mapping[str, float]) -> Dict[str, str]:
    """
    Map every ROS parameter LLM2 could ever change to its self.weights key.

    The cost weights and nothing else, NEVER_TUNABLE_WEIGHTS left out:
    MPC_corr declares weights[key] as the parameter 'mpc_' + key
    (w_term -> mpc_w_term).
    """
    return {'mpc_' + key: key for key in weights if key not in NEVER_TUNABLE_WEIGHTS}


def build_whitelist(names, mins, maxs,
                    tunable: Mapping[str, str]) -> Dict[str, Tuple[float, float]]:
    """
    Return {name: (min, max)} from the three parallel whitelist parameters.

    Raises ValueError on a whitelist that cannot be applied as written:
    unpaired lists, an empty list, a repeated name, a name that is not a cost
    weight or is a NEVER_TUNABLE_WEIGHTS one, a bound that is not finite, a
    negative minimum (a negative weight rewards the error it should charge,
    and the QP stops being convex), or min > max.
    """
    names, mins, maxs = list(names), list(mins), list(maxs)
    if not len(names) == len(mins) == len(maxs):
        raise ValueError(
            f'llm2_param_whitelist/_min/_max are not paired: {len(names)} names, '
            f'{len(mins)} minima, {len(maxs)} maxima')
    if not names:
        raise ValueError('llm2_param_whitelist is empty: no update could ever be applied')
    whitelist = {}
    for name, lo, hi in zip(names, mins, maxs):
        if name in whitelist:
            raise ValueError(f'{name!r} is listed twice in llm2_param_whitelist')
        if name in {'mpc_' + key for key in NEVER_TUNABLE_WEIGHTS}:
            raise ValueError(
                f'{name!r} is never tunable: the obstacle cost has no hard obstacle '
                f'row behind it, so a low value switches obstacle avoidance off')
        if name not in tunable:
            raise ValueError(
                f'{name!r} is not an MPC cost weight; only these can be '
                f'whitelisted: {sorted(tunable)}')
        lo, hi = float(lo), float(hi)
        if not (math.isfinite(lo) and math.isfinite(hi)):
            raise ValueError(f'{name}: bounds [{lo}, {hi}] are not finite')
        if lo < 0.0:
            raise ValueError(f'{name}: minimum {lo:g} is negative')
        if lo > hi:
            raise ValueError(f'{name}: minimum {lo:g} is above maximum {hi:g}')
        whitelist[name] = (lo, hi)
    return whitelist


def reverse_w_v_limit(w_rev: Optional[float], vdes: Optional[float]) -> Optional[float]:
    """
    Return the largest mpc_w_v reverse_mpc's invariant allows, w_rev / (2 * vdes).

    reverse_mpc relies on w_rev >= 2 * w_v * vdes: reversing never costs less
    than standing still on the speed term alone. None when there is no
    reverse planner (w_rev None) or no positive vdes: nothing to hold.
    """
    if w_rev is None or not vdes or vdes <= 0.0:
        return None
    return float(w_rev) / (2.0 * float(vdes))


def validate_update(names, values, whitelist: Mapping[str, Tuple[float, float]],
                    w_v_limit: Optional[float] = None) -> Dict[str, float]:
    """
    Return {name: value} for an update that may be applied as a whole.

    Raises ValueError, saying why, for anything else: unpaired names/values,
    no names, a repeated name, a name not in the whitelist, a value that is
    not a number (bool included), not finite, or outside its [min, max]
    (bounds inclusive), or -- w_v_limit given, i.e. allow_reverse on -- an
    mpc_w_v above w_v_limit (reverse_w_v_limit). One bad entry rejects the
    whole update.
    """
    names, values = list(names), list(values)
    if len(names) != len(values):
        raise ValueError(f'{len(names)} names but {len(values)} values')
    if not names:
        raise ValueError('no names: nothing to apply')
    update = {}
    for name, value in zip(names, values):
        if name in update:
            raise ValueError(f'{name!r} appears twice')
        if name not in whitelist:
            raise ValueError(f'{name!r} is not in llm2_param_whitelist')
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{name}={value!r} is not a number')
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f'{name}={value} is not finite')
        lo, hi = whitelist[name]
        if not lo <= value <= hi:
            raise ValueError(f'{name}={value:g} is outside [{lo:g}, {hi:g}]')
        if name == 'mpc_w_v' and w_v_limit is not None and value > w_v_limit:
            raise ValueError(
                f'{name}={value:g} is above {w_v_limit:g} = w_rev / (2 * vdes): with '
                f'allow_reverse on, reversing would cost less than standing still')
        update[name] = value
    return update


def startup_warnings(whitelist: Mapping[str, Tuple[float, float]],
                     current: Mapping[str, float],
                     w_rev: Optional[float] = None,
                     vdes: Optional[float] = None) -> List[str]:
    """
    Return what is valid in a whitelist but worth an operator's attention.

    - A current value outside its own range: an update echoing it unchanged
      (the mock) would be rejected every time.
    - With allow_reverse's planner on (w_rev and vdes given): a whitelisted
      mpc_w_v maximum above reverse_w_v_limit. Not refused -- validate_update
      rejects every update past the limit -- but the whitelist range is then
      not the range that applies. And a current mpc_w_v already above it:
      every update carrying mpc_w_v (the mock's echo too) is rejected.
    """
    warnings = []
    for name, (lo, hi) in whitelist.items():
        if name in current and not lo <= current[name] <= hi:
            warnings.append(
                f'{name} is {current[name]:g}, outside its whitelist range '
                f'[{lo:g}, {hi:g}]: an update repeating it would be rejected')
    limit = reverse_w_v_limit(w_rev, vdes)
    if limit is not None and 'mpc_w_v' in whitelist:
        if whitelist['mpc_w_v'][1] > limit:
            warnings.append(
                f'allow_reverse is on: an update with mpc_w_v above w_rev / (2 * vdes) '
                f'= {limit:g} is rejected whole, although the whitelist allows up to '
                f'{whitelist["mpc_w_v"][1]:g} (past {limit:g}, reversing would cost '
                f'less than standing still)')
        if current.get('mpc_w_v', 0.0) > limit:
            warnings.append(
                f'allow_reverse is on and mpc_w_v is {current["mpc_w_v"]:g}, already above '
                f'w_rev / (2 * vdes) = {limit:g}: every update carrying mpc_w_v is rejected')
    return warnings


class PendingUpdate:
    """
    The update waiting for the next control tick; thread-safe.

    put() is called from the subscription callback, take() from the control
    loop. A second update put before the first was taken is MERGED over it
    (per name the newer value wins; seq and source are the newer one's): each
    passed validation on its own and the ranges are per name, so the merge is
    valid too, and a partial update is not lost to the one that follows it.
    The older seq then never appears on /mpc/applied_params; put() returns it
    so the caller can log it.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._update = None

    def put(self, update: Update) -> Optional[int]:
        """Park `update`; return the seq of the unapplied one it merged over, else None."""
        with self._lock:
            previous = self._update
            if previous is None:
                self._update = update
                return None
            self._update = Update(update.seq, update.source,
                                  {**previous.values, **update.values})
            return previous.seq

    def take(self) -> Optional[Update]:
        """Return the parked update and clear the slot; None when nothing is parked."""
        with self._lock:
            update, self._update = self._update, None
            return update
