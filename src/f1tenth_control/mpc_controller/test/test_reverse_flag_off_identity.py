"""
allow_reverse false is bit-identical to 868ec8a on the existing closed-loop scenarios.

The golden file (test/data/closed_loop_golden.json) was written on 868ec8a,
before the reverse work touched mpc_solver.py: one SHA-256 per run over the
exact bytes of every solve's inputs and outputs (closed_loop_golden.py). These
tests route every one of those solves through the code the node now runs and
require the same digests -- every tick of every run, bit for bit.
"""

from types import SimpleNamespace

import closed_loop_golden as golden
from mpc_controller.MPC_corr import MPCController
from mpc_controller.mpc_solver import solve_mpc_step
from mpc_controller.reverse_mpc import ReverseConfig, ReversePlanner
import pytest


@pytest.fixture(scope='module')
def recorded():
    return golden.load_golden()


def test_the_golden_file_was_recorded_before_the_reverse_work(recorded):
    import json
    with open(golden.GOLDEN_PATH, encoding='utf-8') as fh:
        assert json.load(fh)['recorded_at'] == '868ec8a'
    assert len(recorded) == 20


def test_solve_mpc_step_without_stage_spec_is_unchanged(recorded):
    assert golden.digests() == recorded


def test_the_node_dispatch_with_allow_reverse_false_is_the_same_call(recorded):
    # MPCController._mpc_solve on a stand-in carrying only what it reads.
    node = SimpleNamespace(allow_reverse=False, reverse_planner=None)
    assert golden.digests(
        solve=lambda **kw: MPCController._mpc_solve(node, **kw)) == recorded


def test_the_planner_with_allow_reverse_false_is_the_same_call(recorded):
    planner = ReversePlanner(ReverseConfig(allow_reverse=False))
    assert golden.digests(
        solve=lambda **kw: planner.step(now_sec=0.0, **kw)) == recorded


def test_a_disabled_planner_object_is_never_consulted(recorded):
    # allow_reverse false with a planner present (e.g. after a parameter
    # change) must still take the plain path.
    class Exploding:
        def step(self, **_kw):
            raise AssertionError('the reverse planner ran with allow_reverse false')
    node = SimpleNamespace(allow_reverse=False, reverse_planner=Exploding())
    out = golden.digests(solve=lambda **kw: MPCController._mpc_solve(node, **kw),
                         runs=('direct/',))
    assert out == {k: v for k, v in recorded.items() if k.startswith('direct/')}


def test_solve_mpc_step_is_what_the_dispatch_calls():
    assert MPCController._mpc_solve.__globals__['solve_mpc_step'] is solve_mpc_step
