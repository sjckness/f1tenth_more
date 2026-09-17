"""An operator's words to object_reached, without hardware or a model.

The chain, every link the shipped code except the LLM and the plant:

  intent JSON          the committed golden intent of the command (llm's
                       test/golden), standing in for the model's answer
  translate()          llm.plan_translate, go_to_enabled=True
  mission file         written by LLMPlannerNode._write_mission_file, the
                       planner's own writer, into a temporary share directory
  loader               mission_config.load_mission_file with stack_params'
                       gap limits
  handler + MPC        test_go_to_object_end_to_end's chain: fake semantic
                       tracks -> GoToObject -> ObjectGoal -> MPC_corr's object
                       mode (stop latch, 0.4 m/s floor) -> solve_mpc_step ->
                       /drive clamp at its stack_params defaults -> nominal
                       plant with the measured braking model ->
                       CheckStopCondition on the real ObjectApproachStatus

Run standalone: python3 -m pytest test/test_llm_go_to_end_to_end.py -v
"""

import json
import math
from pathlib import Path
from unittest.mock import patch

from f1tenth_params.param_defaults import get_value
import pytest

pytest.importorskip('py_trees')

from llm.llm_planner_node import LLMPlannerNode  # noqa: E402
from llm.plan_translate import translate  # noqa: E402

import test_go_to_object_end_to_end as e2e  # noqa: E402

GOLDEN = (Path(__file__).resolve().parents[3]
          / 'f1tenth_intelligence' / 'llm' / 'test' / 'golden')


class _Logger:
    def info(self, *_a, **_k):
        pass


class _Planner:
    def get_logger(self):
        return _Logger()


def _plan_and_run(golden_name, share_dir):
    case = json.loads((GOLDEN / f'{golden_name}.json').read_text())
    result = translate(case['intent'])
    with patch('llm.llm_planner_node.get_package_share_directory',
               return_value=str(share_dir)):
        path = LLMPlannerNode._write_mission_file(_Planner(), result.mission)
    limits = (float(get_value('max_forward_speed_mps')),
              float(get_value('max_reverse_speed_mps')))
    return case, result, path, e2e._run_go_to_person(mission_path=path, speed_limits=limits)


@pytest.fixture(scope='module')
def default_gap(tmp_path_factory):
    return _plan_and_run('ex7_go_to_person', tmp_path_factory.mktemp('share'))


@pytest.fixture(scope='module')
def clamped_gap(tmp_path_factory):
    return _plan_and_run('ex10_go_to_person_gap_clamped', tmp_path_factory.mktemp('share'))


def _final_gap(run):
    d = math.hypot(run.person_odom[0] - run.x[0], run.person_odom[1] - run.x[1])
    return d - run.spec.nose_reach_m - e2e.PERSON_RADIUS


class TestVaiDallaPersona:

    def test_the_command_is_a_single_go_to_person_move(self, default_gap):
        case, _result, path, _run = default_gap
        assert case['command'] == 'vai dalla persona'
        written = json.loads(Path(path).read_text())
        assert written == case['mission']
        assert Path(path).parent.name == 'llm_generated'

    def test_object_reached_fires(self, default_gap):
        run = default_gap[3]
        assert run.reached_at is not None, 'object_reached never fired'
        assert run.state.object_record.outcome == 'reached'
        assert run.state.last_stop_reason == 'stop_condition:object_reached'

    def test_it_rests_just_outside_the_default_gap(self, default_gap):
        """The stop latches past the gap and brakes in: see e2e.rig.stop_window."""
        run = default_gap[3]
        assert run.spec.gap_m == pytest.approx(0.5)
        lo, hi = e2e.rig.stop_window()
        assert run.spec.gap_m + lo <= _final_gap(run) <= run.spec.gap_m + hi

    def test_the_car_never_reverses_or_exceeds_the_forward_limit(self, default_gap):
        speeds = default_gap[3].speeds
        assert min(speeds) >= 0.0
        assert max(speeds) <= float(get_value('max_forward_speed_mps')) + 1e-9

    def test_every_goal_carried_the_translated_moves_wire_id(self, default_gap):
        _case, result, _path, run = default_gap
        assert run.wire.startswith(result.mission['mission_id'] + '#')
        assert run.wire.endswith('/move_0_go_to')
        assert {m.move_id for m in run.goto.goal_pub.msgs} == {run.wire}


class TestFermatiADieciCentimetri:

    def test_the_clamped_gap_is_what_the_loader_and_handler_use(self, clamped_gap):
        _case, result, _path, run = clamped_gap
        assert result.notes, 'the operator was told'
        assert run.spec.gap_m == pytest.approx(0.42)
        assert run.spec.gap_m >= run.spec.gap_min_m - 1e-9

    def test_object_reached_fires_at_the_clamped_gap(self, clamped_gap):
        run = clamped_gap[3]
        assert run.reached_at is not None, 'object_reached never fired at gap_min'
        assert run.state.object_record.outcome == 'reached'
        lo, hi = e2e.rig.stop_window()
        assert run.spec.gap_m + lo <= _final_gap(run) <= run.spec.gap_m + hi
