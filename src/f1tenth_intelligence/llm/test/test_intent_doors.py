"""The enter_door intent phase: the schema, the translation, and the prompt.

The door twin of test_intent_go_to.py. What is NOT here, deliberately: how a
side is resolved against real doors, which is f1tenth_behavior's question and
is tested where it is computed (test_enter_door.py). This file is about the
only thing the LLM authors -- WHICH door -- and about the three places that
have to agree on how it is spelled: intent_v1.json's `side` enum,
plan_translate's MISSION_DOOR_* constants, and mission_config's own
DOOR_NEAREST / DOOR_SIDES.
"""

import json
from types import SimpleNamespace

import jsonschema

from llm.plan_translate import (
    DOOR_MISSION_SCHEMA_VERSION,
    doors_in_prompt,
    INTENT_PROMPT_DOORS_MARKER,
    intent_prompt_examples,
    INTENT_PROMPT_NO_GO_TO_FILENAME,
    IntentPromptError,
    IntentSchemaError,
    load_intent_prompt,
    load_intent_schema,
    MISSION_DOOR_NEAREST,
    MISSION_DOOR_SIDES,
    MISSION_SCHEMA_VERSION,
    OBJECT_MISSION_SCHEMA_VERSION,
    translate,
    TranslatorConfig,
    UnsupportedIntentDoorError,
)

# Third party, alphabetical: jsonschema, llm, pytest. ament_flake8 wants
# `llm` between the other two (it is not in application-import-names), and
# tidying pytest back up next to jsonschema is what re-introduces the I100.
import pytest


def _intent(*plan, unsupported=()):
    return {'plan': list(plan), 'unsupported': list(unsupported)}


def _door_branch():
    """Return the enter_door branch of intent_v1.json's phase oneOf."""
    branches = load_intent_schema()['properties']['plan']['items']['oneOf']
    door = [b for b in branches if b['properties']['mode'].get('const') == 'enter_door']
    assert len(door) == 1, 'exactly one enter_door branch'
    return door[0]


# ---------------------------------------------------------------------------
# the three spellings of a door selector must be one spelling
# ---------------------------------------------------------------------------

def test_the_translator_and_the_loader_spell_a_selector_the_same_way():
    """plan_translate duplicates these rather than importing them; pinned here.

    The duplication is deliberate -- plan_translate keeps its "imports in
    isolation, no ament index required" property and mission_config lives in
    another package -- so this is the test that stops the two copies drifting.
    Skipped, not failed, where f1tenth_behavior is not importable, for the
    same reason the duplication exists.
    """
    mission_config = pytest.importorskip(
        'f1tenth_behavior.mission.mission_config',
        reason='needs f1tenth_behavior on the path')
    assert MISSION_DOOR_NEAREST == mission_config.DOOR_NEAREST
    assert tuple(MISSION_DOOR_SIDES) == tuple(mission_config.DOOR_SIDES)


def test_the_schema_offers_exactly_the_sides_the_mission_accepts():
    """intent_v1.json's enum and the two mission words are one list."""
    assert _door_branch()['properties']['side']['enum'] == list(MISSION_DOOR_SIDES)


# ---------------------------------------------------------------------------
# the schema
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('phase', [
    {'mode': 'enter_door'},
    {'mode': 'enter_door', 'side': 'left'},
    {'mode': 'enter_door', 'side': 'right'},
])
def test_a_door_phase_validates(phase):
    """The three shapes the model may send: bare, left, right."""
    jsonschema.validate(_intent(phase), load_intent_schema())


@pytest.mark.parametrize('phase', [
    {'mode': 'enter_door', 'side': 'ahead'},        # not a side
    {'mode': 'enter_door', 'side': 'sinistra'},     # the Italian word, not the enum
    {'mode': 'enter_door', 'side': 'LEFT'},
    {'mode': 'enter_door', 'side': ''},
    {'mode': 'enter_door', 'side': ['left']},
    {'mode': 'enter_door', 'thresh': 1.0},          # no distance to author
    {'mode': 'enter_door', 'guard': 'wall', 'thresh': 1.0},
    {'mode': 'enter_door', 'target': 'chair'},      # not a go_to
    {'mode': 'enter_door', 'door': 'door_7'},       # naming one is not the LLM's job
    {'mode': 'door'},                               # not the mode name
    {'mode': 'enter_door', 'side': 'left', 'ordinal': 2},
])
def test_an_invented_door_field_or_side_is_rejected(phase):
    """additionalProperties: false, plus the side enum.

    A thresh is the interesting one: it would be the model authoring the
    arrival test, which is two tolerances in config, not a distance.
    """
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(_intent(phase), load_intent_schema())


def test_the_schema_says_why_an_ordinal_is_not_expressible():
    """The model reads the schema too; the reason belongs in it, not only here."""
    text = _door_branch()['description']
    assert 'ordinal' in text.lower()
    assert 'unsupported' in text


# ---------------------------------------------------------------------------
# the translation
# ---------------------------------------------------------------------------

def test_no_side_means_the_nearest_door():
    """An omitted side is the mission's "nearest" selector, not an error."""
    move = translate(_intent({'mode': 'enter_door'})).mission['moves'][0]
    assert move['enter_door']['door'] == MISSION_DOOR_NEAREST
    assert move['stop_condition'] == {'type': 'door_entered'}
    assert move['id'] == 'move_0_enter_door'
    assert move['terminal'] is True


@pytest.mark.parametrize('side', MISSION_DOOR_SIDES)
def test_a_side_is_passed_through_untouched(side):
    """The intent and the mission spell a side with the same word."""
    move = translate(_intent({'mode': 'enter_door', 'side': side})).mission['moves'][0]
    assert move['enter_door']['door'] == side


def test_the_tolerances_come_from_config_and_nothing_else():
    """The whole emitted body, against a config with four odd values."""
    cfg = TranslatorConfig(door_xy_tol_m=0.2, door_yaw_tol_deg=15.0,
                           door_acquire_timeout_sec=7.0, door_approach_dist_m=1.1)
    body = translate(_intent({'mode': 'enter_door'}), config=cfg).mission['moves'][0]
    assert body['enter_door'] == {
        'type': 'enter_door', 'door': MISSION_DOOR_NEAREST, 'xy_tol': 0.2,
        'yaw_tol_deg': 15.0, 'acquire_timeout_sec': 7.0, 'approach_dist_m': 1.1}


def test_the_selector_is_INTENT_and_the_rest_is_CONFIG():
    """The provenance of a door move, field by field.

    test_intent_translate.py already asserts that every leaf HAS a
    provenance entry; this asserts which source each one names, because
    "CONFIG" on the selector or "INTENT" on a tolerance would both be the
    invention the provenance rule exists to prevent.
    """
    prov = translate(_intent({'mode': 'enter_door', 'side': 'right'}),
                     explain=True).provenance
    assert prov['moves[0].enter_door.door'][0] == 'INTENT'
    for field in ('xy_tol', 'yaw_tol_deg', 'acquire_timeout_sec', 'approach_dist_m'):
        assert prov[f'moves[0].enter_door.{field}'][0] == 'CONFIG', field
    assert prov['moves[0].enter_door.type'][0] == 'CONST'
    assert prov['moves[0].stop_condition.type'][0] == 'CONST'
    # And with no side, the selector is a constant, not an intent field.
    bare = translate(_intent({'mode': 'enter_door'}), explain=True).provenance
    assert bare['moves[0].enter_door.door'][0] == 'CONST'


@pytest.mark.parametrize('plan, version', [
    ([{'mode': 'straight', 'guard': 'wall', 'thresh': 2.0}], MISSION_SCHEMA_VERSION),
    ([{'mode': 'go_to', 'target': 'chair'}], OBJECT_MISSION_SCHEMA_VERSION),
    ([{'mode': 'enter_door'}], DOOR_MISSION_SCHEMA_VERSION),
    ([{'mode': 'enter_door', 'side': 'left'}], DOOR_MISSION_SCHEMA_VERSION),
    # Both: the versions are cumulative, so the door one wins.
    ([{'mode': 'enter_door'}, {'mode': 'go_to', 'target': 'chair'}],
     DOOR_MISSION_SCHEMA_VERSION),
])
def test_the_schema_version_says_which_loader_can_read_the_document(plan, version):
    """3.0, 5.0 or 6.1, by what the plan carries."""
    assert translate(_intent(*plan)).mission['schema_version'] == version


def test_a_changed_door_tolerance_gives_a_new_mission_id():
    """Otherwise two different missions would collide on one id.

    Every field of a door body except the selector is config, so the id has
    to hash the resolved body and not only the intent -- the same reason a
    go_to's gap_m is hashed. llm_planner_node._write_mission_file refuses to
    overwrite a file whose id matches and whose content does not, so a
    collision here is a mission that cannot be written at all.
    """
    intent = _intent({'mode': 'enter_door', 'side': 'left'})
    a = translate(intent).mission
    b = translate(intent, config=TranslatorConfig(door_xy_tol_m=0.2)).mission
    assert a['mission_id'] != b['mission_id']
    # ... and the same intent under the same config is still byte-identical.
    assert translate(intent).mission == a


def test_the_two_sides_are_two_missions():
    """Three selectors, three ids: the selector is part of the hash."""
    left = translate(_intent({'mode': 'enter_door', 'side': 'left'})).mission
    right = translate(_intent({'mode': 'enter_door', 'side': 'right'})).mission
    bare = translate(_intent({'mode': 'enter_door'})).mission
    assert len({left['mission_id'], right['mission_id'], bare['mission_id']}) == 3


def test_a_door_move_is_timed_against_an_unknown_distance():
    """Open-ended like go_to: the doorway's distance is not known when written."""
    cfg = TranslatorConfig()
    door = translate(_intent({'mode': 'enter_door'}), config=cfg).mission['moves'][0]
    straight = translate(_intent({'mode': 'straight', 'guard': 'wall', 'thresh': 2.0}),
                         config=cfg).mission['moves'][0]
    assert door['timeout_sec'] == straight['timeout_sec']
    assert door['on_timeout'] == 'abort'


def test_the_door_move_loads_in_the_real_loader():
    """The translator's own output check, stated out loud.

    _validate_output() already runs parse_mission() inside translate(), so
    this cannot fail on its own; it is here to name the guarantee.

    A translator that emitted a selector the loader rejects would be caught
    here rather than on the car -- the whole point of validating the output
    against mission_config instead of against a second copy of the schema.
    """
    mission_config = pytest.importorskip(
        'f1tenth_behavior.mission.mission_config',
        reason='needs f1tenth_behavior on the path')
    mission = translate(_intent({'mode': 'enter_door', 'side': 'right'},
                                {'mode': 'straight', 'guard': 'distance',
                                 'thresh': 2.0})).mission
    config = mission_config.parse_mission(mission)
    assert config.moves[0].enter_door.door == 'right'
    assert config.moves[1].terminal


# ---------------------------------------------------------------------------
# enter_door_enabled=False: refused, and fed back to the model
# ---------------------------------------------------------------------------

def test_a_door_phase_is_refused_when_doors_are_off():
    """The refusal text is written for the MODEL, not for a log reader."""
    with pytest.raises(UnsupportedIntentDoorError) as exc:
        translate(_intent({'mode': 'enter_door', 'side': 'left'}), enter_door_enabled=False)
    message = str(exc.value)
    assert 'unsupported' in message            # what the model must do instead
    assert 'enter_door' in message
    # Written for the MODEL: it is appended to the next prompt verbatim, so it
    # is in the prompt's language, and it names the substitution to avoid.
    assert '"wall"' in message


def test_the_refusal_is_retryable_feedback_not_a_crash():
    """The refusal subclasses the error _plan_v2 feeds back to the model.

    A bare KeyError -- which is what a door phase falling through to the
    straight branch would raise on phase['guard'] -- is not caught there at
    all and would surface as a traceback.
    """
    assert issubclass(UnsupportedIntentDoorError, IntentSchemaError)


def test_the_other_phases_still_translate_with_doors_off():
    """Switching doors off touches nothing else."""
    mission = translate(_intent({'mode': 'straight', 'guard': 'wall', 'thresh': 2.0}),
                        enter_door_enabled=False).mission
    assert mission['schema_version'] == MISSION_SCHEMA_VERSION


# ---------------------------------------------------------------------------
# the prompt
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('go_to, doors, expected', [
    (True, True, True),
    (True, False, False),
    (False, True, False),      # the prompt without go_to has no door section
    (False, False, False),
])
def test_doors_need_the_go_to_prompt(go_to, doors, expected):
    """The truth table the node resolves its one door switch from."""
    assert doors_in_prompt(go_to, doors) is expected


@pytest.mark.parametrize('doors_enabled', [True, False])
def test_the_marker_is_always_substituted(doors_enabled):
    """A prompt that still carried {{PORTE}} would send the model a literal."""
    prompt = load_intent_prompt(doors_enabled=doors_enabled)
    assert INTENT_PROMPT_DOORS_MARKER not in prompt
    assert 'PORTE' in prompt


def test_the_on_section_teaches_the_phase_and_the_two_sides():
    """What the model is actually shown when doors are on."""
    prompt = load_intent_prompt(doors_enabled=True)
    assert '{"mode":"enter_door"}' in prompt
    for side in MISSION_DOOR_SIDES:
        assert f'{{"mode":"enter_door","side":"{side}"}}' in prompt
    # The one thing an operator gets wrong: entered is not through.
    assert "DENTRO LA PORTA NON E' OLTRE LA PORTA" in prompt
    assert 'DUE fasi' in prompt


def test_the_off_section_sends_every_door_to_unsupported():
    """And it never mentions the phase, so the model cannot learn it."""
    prompt = load_intent_prompt(doors_enabled=False)
    assert 'enter_door' not in prompt
    assert 'PORTE - NON SUPPORTATE' in prompt
    examples = dict(intent_prompt_examples(doors_enabled=False))
    assert examples['vai dalla porta'] == {'plan': [], 'unsupported': ['vai dalla porta']}


def test_the_on_sections_examples_are_what_the_translator_does():
    """Every door example, run through the translator it is teaching.

    A worked example is the model's strongest signal, so an example whose
    intent the translator would refuse teaches refused output directly.
    """
    examples = dict(intent_prompt_examples(doors_enabled=True))
    assert examples['entra nella porta a destra']['plan'] == [
        {'mode': 'enter_door', 'side': 'right'}]
    assert examples['vai dalla porta']['plan'] == [{'mode': 'enter_door'}]
    assert examples['attraversa la porta a sinistra e avanza due metri']['plan'] == [
        {'mode': 'enter_door', 'side': 'left'},
        {'mode': 'straight', 'guard': 'distance', 'thresh': 2.0}]
    assert examples['prendi la seconda porta a destra'] == {
        'plan': [], 'unsupported': ['prendi la seconda porta a destra']}
    for command, intent in examples.items():
        if 'porta' not in command:
            continue
        if intent['plan']:
            assert translate(intent).mission['moves'], command
        else:
            assert intent['unsupported'] == [command], command


def test_the_legacy_prompt_cannot_be_asked_to_teach_doors():
    """It has no substitution point, and silently dropping doors is the bug.

    The node computes doors_in_prompt() precisely so this never happens; the
    raise is what makes a wiring mistake loud instead of leaving the model
    untaught while the translator accepts doors.
    """
    with pytest.raises(IntentPromptError):
        load_intent_prompt(INTENT_PROMPT_NO_GO_TO_FILENAME, doors_enabled=True)
    legacy = load_intent_prompt(INTENT_PROMPT_NO_GO_TO_FILENAME, doors_enabled=False)
    assert 'enter_door' not in legacy


# ---------------------------------------------------------------------------
# the planner, end to end with the LLM call patched
# ---------------------------------------------------------------------------

class _Stub:
    """The attributes _plan_v2 touches on self. See test_intent_go_to._Stub."""

    def __init__(self, enter_door_enabled):
        self._system_prompt = 'unused: the LLM call is patched'
        self._go_to_enabled = True
        self._enter_door_enabled = enter_door_enabled
        self._planner_path = 'v2'
        self._logger = SimpleNamespace(
            info=lambda m: None, warn=lambda m: self.warned.append(str(m)),
            error=lambda m: None)
        self.warned = []
        self.opts = SimpleNamespace(dry_run=True, confirm=False)
        self.written = None

    def get_logger(self):
        return self._logger

    def _note_llm_sent(self):
        pass

    def _note_llm_received(self):
        pass

    def _write_mission_file(self, mission):
        self.written = mission
        return '/nonexistent/unused.json'


def _run(monkeypatch, response, command, enter_door_enabled):
    from llm import llm_planner_node
    from llm.llm_planner_node import LLMPlannerNode
    monkeypatch.setattr(llm_planner_node, 'get_intent_from_llm',
                        lambda *a, **k: response)
    stub = _Stub(enter_door_enabled)
    return LLMPlannerNode._plan_v2(stub, command), stub


def test_a_door_command_plans_first_time(monkeypatch):
    """The whole planner path, with the LLM call patched."""
    result, stub = _run(monkeypatch, _intent({'mode': 'enter_door', 'side': 'right'}),
                        'entra nella porta a destra', True)
    mission, unsupported, _dt, attempts, notes = result
    assert attempts == 1
    assert mission['moves'][0]['enter_door']['door'] == 'right'
    assert mission['schema_version'] == DOOR_MISSION_SCHEMA_VERSION
    assert unsupported == () and notes == ()
    assert not stub.warned
    # The preview an operator confirms is describe_mission's, printed by
    # process_command and not by _plan_v2 -- see the parametrized test below.


def test_a_door_command_with_doors_off_ends_as_unsupported(monkeypatch, capsys):
    """The model complies on the retry; the operator sees the banner.

    MAX_INTENT_RETRIES + 1 attempts of a door intent would otherwise look
    like exhaustion ("the model never produced valid output"), which is a
    different failure from "the robot cannot do this".
    """
    from llm.llm_planner_node import MAX_INTENT_RETRIES
    result, stub = _run(monkeypatch, _intent({'mode': 'enter_door'}),
                        'entra nella porta', False)
    assert result is None
    out = capsys.readouterr().out
    assert 'RICHIESTA NON SUPPORTATA' in out
    assert 'entra nella porta' in out
    assert len([line for line in stub.warned
                if 'prompt-non-rispettato' in line]) == MAX_INTENT_RETRIES + 1


def test_the_refusal_banner_advertises_doors_when_they_are_on(monkeypatch, capsys):
    """What IS supported, printed under the banner, must mention them."""
    _run(monkeypatch, _intent(unsupported=['vai da Marco']), 'vai da Marco', True)
    out = capsys.readouterr().out
    assert 'entra nella porta a destra' in out
    assert 'non la seconda' in out
    _run(monkeypatch, _intent(unsupported=['vai da Marco']), 'vai da Marco', False)
    assert 'entra nella porta' not in capsys.readouterr().out


def test_an_intent_the_model_invented_a_door_field_in_is_retried(monkeypatch):
    """A bogus side is a schema rejection, so it is fed back, not run."""
    from llm.llm_planner_node import MAX_INTENT_RETRIES
    calls = []

    def fake(command_text, system_prompt, feedback=None):
        calls.append(feedback)
        return {'plan': [{'mode': 'enter_door', 'side': 'ahead'}], 'unsupported': []}

    from llm import llm_planner_node
    from llm.llm_planner_node import LLMPlannerNode
    monkeypatch.setattr(llm_planner_node, 'get_intent_from_llm', fake)
    assert LLMPlannerNode._plan_v2(_Stub(True), 'entra nella porta davanti') is None
    assert len(calls) == MAX_INTENT_RETRIES + 1
    assert calls[0] is None and all(c for c in calls[1:])
    assert any('side' in c for c in calls[1:] if c)


# The operator's preview of a door move is tested with the other
# describe_mission cases, in test_planner_path.py: it renders a mission
# document and does not care how the document was authored.

def test_the_generated_mission_is_json_serialisable_as_written():
    """_write_mission_file json.dumps() it; a non-serialisable leaf is a crash."""
    mission = translate(_intent({'mode': 'enter_door', 'side': 'left'})).mission
    assert json.loads(json.dumps(mission)) == mission
