"""
allow_reverse false is bit-identical to the plain solve on the existing closed-loop scenarios.

The golden file (test/data/closed_loop_golden.json) holds one SHA-256 per run
over the exact bytes of every solve's inputs and outputs
(closed_loop_golden.py). These tests route every one of those solves through
the code the node now runs and require the same digests -- every tick of every
run, bit for bit.

It was first written on 868ec8a, before the reverse work touched
mpc_solver.py, and RE-RECORDED on branch mpc-centerline-target: every run
feeds compute_local_target's pref_nom into its solves, and that branch removed
the target's obstacle deflection, its coast and its exponential smoother, so
all 20 input streams changed by design (the obstacle-free rows too, through
the smoother). The solver and the reverse dispatch did not change; what the
digests pin -- that the allow_reverse-false paths are the plain call -- is the
same property against the new reference.
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


def test_the_golden_file_is_the_centerline_target_recording(recorded):
    import json
    with open(golden.GOLDEN_PATH, encoding='utf-8') as fh:
        assert json.load(fh)['recorded_at'] == 'mpc-centerline-target'
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
