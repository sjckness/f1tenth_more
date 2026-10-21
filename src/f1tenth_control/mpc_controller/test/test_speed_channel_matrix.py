"""
The robustness matrix: offset-free speed against plants that are not its model.

Every cell is one closed-loop run of speed_channel_harness.run (the real QP,
the real SpeedChannel, stack_params weights) on a straight drive at vdes 0.5
or 0.4, against a Plant whose gain, lag, delay and static friction are each
set independently of what the solver assumes (speed_lag_tau_s,
speed_delay_ticks), with sigma 0.02 m/s noise on the measured speed in every
cell:

  gain      0.6, 0.76, 1.0, 1.15 constant, and the speed-dependent curve
            fitted to the archived runs (harness.FITTED_GAIN)
  tau       0.5x, 1x, 2x the nominal
  delay     0, 1, 2 control ticks
  stiction  off, 0.15 m/s (from rest the car does not move below it)

PASS, per cell:
  * steady-state error |v - vdes| < 0.02 m/s on the TRUE speed (mean over
    8-12 s), whenever vdes is reachable under the command ceiling
    (max_forward_speed_mps, 0.5: the operator's decision, see
    SpeedChannel.ceiling). Where it is not -- vdes / gain > 0.5, e.g. gain
    0.6 or 0.76 at vdes 0.5 -- the car must instead settle at what the
    ceiling delivers, within 0.02, with the command AT the ceiling and d not
    wound past what the ceiling can deliver;
  * overshoot < 10 % of vdes wherever the plant is no further from the
    model than the data allow (gain <= 1.0 or the fitted curve, delay <= 1
    tick), < 15 % in the stress corner beyond it (gain 1.15, which nothing
    measured approaches -- the fitted gain never exceeds 0.89 up to 0.65 m/s
    -- or two ticks of delay against a measured dead time under 20 ms). The
    corner reaches 10-14 %: the car over-delivers, or answers 0.2 s late,
    before the observer can see it. Relaxed there on the operator's call
    (2026-10-21, "a small overshoot is not a problem") rather than bought
    with a slower rise everywhere -- a model tau of 0.26 s plus a fading
    start-up margin brought every cell under 10 % at 0.3 s more rise time;
  * the command never above the ceiling.
Rise time (10 -> 90 % of the target) and the command peak are reported, not judged.

Print the table:  python3 test/speed_channel_harness.py
"""

import functools
import itertools
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import speed_channel_harness as harness  # noqa: E402

TAU_NOMINAL = harness.TAU_IDENTIFIED
GAINS = (0.6, 0.76, 1.0, 1.15, 'fitted')
TAU_FACTORS = (0.5, 1.0, 2.0)
DELAYS = (0, 1, 2)
STICTIONS = (0.0, 0.15)
VDES = (0.5, 0.4)
NOISE = 0.02
SS_TOL = 0.02
OVERSHOOT_MAX = 0.10
OVERSHOOT_MAX_STRESS = 0.15

CELLS = list(itertools.product(GAINS, TAU_FACTORS, DELAYS, STICTIONS, VDES))


def _gain(label):
    return harness.FITTED_GAIN if label == 'fitted' else float(label)


def _reachable(gain, vdes, ceiling):
    """Steady speed the ceiling can deliver, and whether vdes is within it."""
    plant = harness.Plant(gain=gain, noise=0.0)
    at_ceiling = plant.steady(ceiling)
    return at_ceiling, at_ceiling >= vdes - 1e-6


@functools.lru_cache(maxsize=None)
def run_cell(gain_label, tau_factor, delay, stiction, vdes, seed=7):
    gain = _gain(gain_label)
    plant = harness.Plant(gain=gain, tau=TAU_NOMINAL * tau_factor, delay=delay,
                          stiction=stiction, noise=NOISE, seed=seed)
    trace = harness.run(mode='offset_free', vdes=vdes, plant=plant, duration=12.0)
    at_ceiling, reachable = _reachable(gain, vdes, harness.MAX_FORWARD)
    m = harness.metrics(trace, vdes, target=vdes if reachable else at_ceiling)
    m.update(reachable=reachable, v_at_ceiling=at_ceiling,
             target=vdes if reachable else at_ceiling,
             d_hat_max_abs=float(np.max(np.abs(trace.arrays()['d_hat']))))
    return m


def run_matrix(verbose=False):
    """Every cell, as rows; with verbose, the markdown table the report quotes."""
    rows = []
    for cell in CELLS:
        m = run_cell(*cell)
        gain, tau_f, delay, stiction, vdes = cell
        rows.append(dict(gain=gain, tau_factor=tau_f, delay=delay, stiction=stiction,
                         vdes=vdes, **m))
    if verbose:
        print('| gain | tau | delay | stiction | vdes | v_ss | error | ok | rise s | '
              'overshoot | cmd peak | cmd ss | d ss |')
        print('|---|---|---|---|---|---|---|---|---|---|---|---|---|')
        for r in rows:
            err = r['v_ss'] - r['target']
            ok = 'yes' if abs(err) < SS_TOL else 'NO'
            if not r['reachable']:
                ok += ' (ceiling)'
            print(f"| {r['gain']} | {r['tau_factor']:.1f}x | {r['delay']} | {r['stiction']:.2f} | "
                  f"{r['vdes']:.1f} | {r['v_ss']:.3f} | {err:+.3f} | {ok} | "
                  f"{r['rise_s']:.2f} | {100 * r['overshoot']:.1f} % | {r['v_cmd_peak']:.3f} | "
                  f"{r['v_cmd_ss']:.3f} | {r['d_hat_ss']:+.3f} |")
    return rows


def _id(cell):
    gain, tau_f, delay, stiction, vdes = cell
    return f'K{gain}-tau{tau_f}x-d{delay}-s{stiction}-v{vdes}'


@pytest.mark.parametrize('cell', CELLS, ids=[_id(c) for c in CELLS])
def test_the_cell(cell):
    m = run_cell(*cell)
    err = m['v_ss'] - m['target']
    assert abs(err) < SS_TOL, (
        f"steady state {m['v_ss']:.3f} vs target {m['target']:.3f} "
        f"({'vdes' if m['reachable'] else 'ceiling-limited'})")
    gain, delay = cell[0], cell[2]
    stress = delay > 1 or (gain != 'fitted' and float(gain) > 1.0)
    limit = OVERSHOOT_MAX_STRESS if stress else OVERSHOOT_MAX
    assert m['overshoot'] < limit, (
        f"overshoot {100 * m['overshoot']:.1f} % (limit {100 * limit:.0f} %)")
    assert m['v_cmd_peak'] <= harness.MAX_FORWARD + 1e-9
    if not m['reachable']:
        # Pinned at the ceiling, and d between "enough to put the command
        # there" (vdes - ceiling) and the deficit the ceiling actually shows
        # (v - c at the ceiling): anti-windup stops it integrating any further
        # once the command sits at the limit.
        vdes = cell[-1]
        assert m['v_cmd_ss'] == pytest.approx(harness.MAX_FORWARD, abs=0.01)
        assert m['d_hat_ss'] <= vdes - harness.MAX_FORWARD + 0.02
        assert m['d_hat_ss'] >= m['v_at_ceiling'] - harness.MAX_FORWARD - 0.03
    assert math.isfinite(m['rise_s']), f"never reached 90 % of {m['target']:.3f}"
