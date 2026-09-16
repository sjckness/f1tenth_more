"""The planner_path wire-in: path pairing, retries, display and the writer.

These exercise LLMPlannerNode's methods unbound, against a stub `self`, rather
than constructing the node: its __init__ waits on a live llama-server and
creates ROS service clients, neither of which belongs in a unit test.
"""

import json
import os
from unittest.mock import patch

import pytest

from llm import llm_planner_node
from llm.llm_planner_node import (
    DEFAULT_PLANNER_PATH,
    LLMPlannerNode,
    MAX_INTENT_RETRIES,
    PLANNER_PATHS,
    PLANNER_PATH_SPEC,
    describe_mission,
)
from llm.plan_translate import IntentSchemaError, translate


class _StubLogger:
    """Collects log lines so a test can assert what was reported."""

    def __init__(self):
        self.lines = {'info': [], 'warn': [], 'error': []}

    def info(self, msg):
        self.lines['info'].append(str(msg))

    def warn(self, msg):
        self.lines['warn'].append(str(msg))

    def error(self, msg):
        self.lines['error'].append(str(msg))


class _StubNode:
    """Just enough of LLMPlannerNode for the unbound methods under test."""

    def __init__(self, prompt='PROMPT', go_to_enabled=True):
        self._system_prompt = prompt
        self._go_to_enabled = go_to_enabled
        self._logger = _StubLogger()

    def get_logger(self):
        return self._logger


# ---------------------------------------------------------------------------
# the pairing: the two halves must never mix
# ---------------------------------------------------------------------------

def test_every_path_is_specified_exactly_once():
    """PLANNER_PATH_SPEC is the single place the pairing is declared."""
    assert set(PLANNER_PATH_SPEC) == set(PLANNER_PATHS)
    for path, spec in PLANNER_PATH_SPEC.items():
        assert set(spec) == {'prompt', 'validation', 'translator'}, path


def test_the_default_path_is_v2():
    """v2 is the path on which the model cannot author speed or timeouts."""
    assert DEFAULT_PLANNER_PATH == 'v2'
    assert DEFAULT_PLANNER_PATH in PLANNER_PATHS


def test_a_legacy_response_through_translate_raises():
    """A bare phase list is not an intent, and must not translate to anything.

    The legacy vocabulary's guard names overlap the intent's, so a list of
    straight phases is close enough to look translatable. It is not: the
    schema requires an object, so this is a named rejection rather than a
    plausible mission built from the wrong semantics.
    """
    legacy_plan = [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0,
                    'stop_at_distance': 3.0}]
    with pytest.raises(IntentSchemaError):
        translate(legacy_plan)


@patch('llm.llm_planner_node.requests.post')
def test_a_v2_response_through_the_legacy_path_raises(mock_post):
    """get_plan_from_llm would happily have extracted intent["plan"].

    That is the dangerous direction: the dict branch pulls out any list it can
    find, so a v2 intent would have yielded a phase list and a mission built
    from a prompt with different semantics. It is refused by name instead.
    """
    class _Resp:
        status_code = 200

        def raise_for_status(self):
            return None

        def json(self):
            return {'content': json.dumps({
                'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0}],
                'unsupported': [],
            })}

    mock_post.return_value = _Resp()
    with pytest.raises(ValueError, match='intent v2'):
        llm_planner_node.get_plan_from_llm('vai dritto')


# ---------------------------------------------------------------------------
# retries
# ---------------------------------------------------------------------------

def _good_intent():
    return {'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0}],
            'unsupported': []}


def test_an_invalid_intent_is_retried_then_succeeds():
    """The validator's error text is fed back, and the retry is logged."""
    node = _StubNode()
    responses = [{'plan': [{'mode': 'straight'}], 'unsupported': []}, _good_intent()]
    seen = []

    def fake_get(command, prompt, feedback=None):
        seen.append(feedback)
        return responses[len(seen) - 1]

    with patch('llm.llm_planner_node.get_intent_from_llm', side_effect=fake_get):
        planned = LLMPlannerNode._plan_v2(node, 'vai dritto')

    assert planned is not None
    mission, unsupported, _dt, attempts, _notes = planned
    assert attempts == 2
    assert seen[0] is None and seen[1] is not None
    assert node.get_logger().lines['warn'], 'the retry must be logged'
    assert mission['moves'][0]['stop_condition']['type'] == 'front_clearance'


def test_an_intent_that_never_validates_emits_no_mission():
    """After MAX_INTENT_RETRIES extra tries it gives up -- it does not patch around."""
    node = _StubNode()
    bad = {'plan': [{'mode': 'straight'}], 'unsupported': []}
    with patch('llm.llm_planner_node.get_intent_from_llm', return_value=bad) as mock_get:
        assert LLMPlannerNode._plan_v2(node, 'x') is None
    assert mock_get.call_count == MAX_INTENT_RETRIES + 1
    assert node.get_logger().lines['error']


def test_an_empty_plan_is_not_retried():
    """Ambiguity is the correct answer, not a model error to insist against."""
    node = _StubNode()
    empty = {'plan': [], 'unsupported': ['ambiguo: distanza non specificata']}
    with patch('llm.llm_planner_node.get_intent_from_llm', return_value=empty) as mock_get:
        assert LLMPlannerNode._plan_v2(node, 'gira') is None
    assert mock_get.call_count == 1


def test_a_v2_failure_never_falls_back_to_legacy():
    """A silent downgrade would run a mission from a different prompt."""
    node = _StubNode()
    bad = {'plan': [{'mode': 'straight'}], 'unsupported': []}
    with patch('llm.llm_planner_node.get_intent_from_llm', return_value=bad):
        with patch('llm.llm_planner_node.get_plan_from_llm') as legacy:
            assert LLMPlannerNode._plan_v2(node, 'x') is None
    legacy.assert_not_called()


def test_unsupported_is_carried_out_of_the_planner():
    """The flag the operator confirmation depends on must survive the call."""
    node = _StubNode()
    intent = {'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0}],
              'unsupported': ['saltare sopra la scatola']}
    with patch('llm.llm_planner_node.get_intent_from_llm', return_value=intent):
        _mission, unsupported, _dt, _attempts, _notes = LLMPlannerNode._plan_v2(node, 'x')
    assert unsupported == ('saltare sopra la scatola',)


# ---------------------------------------------------------------------------
# the display renders the translated mission
# ---------------------------------------------------------------------------

def test_the_display_shows_speed_timeout_and_terminal():
    """Speed and timeout were absent from the old display entirely."""
    mission = translate({'plan': [{'mode': 'straight', 'guard': 'distance', 'thresh': 2.0}],
                         'unsupported': []}).mission
    text = describe_mission(mission)
    assert 'velocita' in text
    assert 'm/s' in text
    assert 'timeout' in text
    assert 'TERMINALE' in text
    assert 'percorsi 2.0 m' in text


def test_a_post_turn_straight_is_not_rendered_as_a_turn():
    """The exact disagreement that motivated rewriting the display.

    "vai dritto, al muro gira a destra e avanza 2 metri": the final move is a
    post-turn continuation, which the translator emits as drive.mode
    "straight". The old describe() rendered the LLM's own phase and said "gira
    a destra" for it. Rendering the translated document cannot disagree,
    because it IS the document.
    """
    mission = translate({'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0},
                                  {'mode': 'turn', 'dir': 'right'},
                                  {'mode': 'straight', 'guard': 'distance', 'thresh': 2.0}],
                         'unsupported': []}).mission
    lines = describe_mission(mission).split('  2. ')
    assert len(lines) == 2, 'expected a third move in the summary'
    assert lines[1].startswith('vai dritto')
    assert 'gira' not in lines[1]
    assert describe_mission(mission).count('gira a destra') == 1


def test_a_turn_is_rendered_with_its_magnitude_once():
    """Magnitude appears as a number with units, not duplicated per field."""
    mission = translate({'plan': [{'mode': 'turn', 'dir': 'left'}],
                         'unsupported': []}).mission
    text = describe_mission(mission)
    assert 'gira a sinistra di 90.0 gradi' in text
    assert 'ruotato di 90.0 gradi' in text


# ---------------------------------------------------------------------------
# the writer
# ---------------------------------------------------------------------------

def _write(node, mission, share_dir):
    with patch('llm.llm_planner_node.get_package_share_directory',
               return_value=str(share_dir)):
        return LLMPlannerNode._write_mission_file(node, mission)


def test_plans_are_written_to_their_own_directory_never_missions(tmp_path):
    """Hand-written missions live in missions/; generated ones never go there."""
    node = _StubNode()
    mission = translate(
        {'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0}],
         'unsupported': []}).mission
    path = _write(node, mission, tmp_path)
    assert os.path.dirname(path).endswith(os.path.join('missions', 'llm_generated'))
    assert os.path.basename(path) == f'{mission["mission_id"]}.json'


def test_rewriting_an_identical_mission_reuses_the_file(tmp_path):
    """Deterministic ids make a repeated command hit the same name every time.

    That is the normal case now, not a collision, so it must not error.
    """
    node = _StubNode()
    mission = translate(
        {'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0}],
         'unsupported': []}).mission
    first = _write(node, mission, tmp_path)
    second = _write(node, mission, tmp_path)
    assert first == second
    assert any('riusata' in line for line in node.get_logger().lines['info'])


def test_a_different_mission_on_the_same_id_refuses_to_overwrite(tmp_path):
    """Losing a previous plan silently is the worst available outcome."""
    node = _StubNode()
    mission = translate(
        {'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0}],
         'unsupported': []}).mission
    _write(node, mission, tmp_path)
    tampered = json.loads(json.dumps(mission))
    tampered['moves'][0]['drive']['speed'] = 0.9
    with pytest.raises(FileExistsError):
        _write(node, tampered, tmp_path)
