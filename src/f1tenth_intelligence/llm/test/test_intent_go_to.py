"""The go_to intent branch: schema, translation to go_to_object, and the switch.

go_to_enabled (stack_params.yaml, read by llm_planner_node) chooses between
two planners:

  true   the prompt teaches go_to and translate() turns a go_to phase into a
         go_to_object move (mission schema 5.0), clamping a distance below
         the reachable minimum with an operator note
  false  the planner before go_to_object: the prompt without go_to, the
         UnsupportedIntentModeError refusal fed back as retry text, and the
         RICHIESTA NON SUPPORTATA banner -- pinned here text for text

Both halves are tested, because switching back has to be a single value.
"""

import json
import math
import pathlib
from types import SimpleNamespace

import jsonschema
import pytest

from llm.plan_translate import (
    CONST,
    DERIVED,
    INTENT,
    INTENT_PROMPT_FILENAME,
    INTENT_PROMPT_NO_GO_TO_FILENAME,
    IntentSchemaError,
    OBJECT_MISSION_SCHEMA_VERSION,
    TranslatorConfig,
    UnsupportedIntentModeError,
    intent_prompt_examples,
    intent_prompt_filename,
    load_intent_prompt,
    translate,
)

SCHEMA_PATH = (pathlib.Path(__file__).resolve().parents[1]
               / 'schemas' / 'intent_v1.json')
SCHEMA = json.loads(SCHEMA_PATH.read_text())
TARGETS = next(
    b for b in SCHEMA['properties']['plan']['items']['oneOf']
    if b['properties']['mode'].get('const') == 'go_to'
)['properties']['target']['enum']

STRAIGHT = {'mode': 'straight', 'guard': 'wall', 'thresh': 2.0}
TURN = {'mode': 'turn', 'dir': 'left'}
GO_TO = {'mode': 'go_to', 'target': 'bottle', 'thresh': 1.0}

# The refusal text the planner fed back before go_to_object existed, verbatim.
REFUSAL_BEFORE_GO_TO = (
    'fase {i}: "go_to" non e\' eseguibile da questo robot. '
    'Non sostituirlo con "front_object": quello si ferma alla cosa '
    'piu\' vicina, non a quella nominata. '
    'Rimetti la richiesta in "unsupported".')

# The "what is supported" line under the RICHIESTA NON SUPPORTATA banner,
# verbatim from the planner before go_to_object existed.
SUPPORTED_BEFORE_GO_TO = (
    '\nSupportato: andare dritto e fermarsi al muro, a una '
    'distanza percorsa, o prima di cio\' che si trova davanti '
    'senza nominarlo. Esempio: "vai dritto e fermati prima '
    'dell\'ostacolo".')


def _limits(gap_min=0.42, class_margin=0.0):
    """Return a gap_limits_for whose gap_min is `gap_min`."""
    from f1tenth_params.object_geometry import GapLimits
    return lambda target_class: GapLimits(
        target_class=target_class, car_radius=0.20,
        avoidance_margin=gap_min - 0.30 - class_margin, class_margin=class_margin,
        settle_buffer=0.10, wheelbase=0.305)


def validate(*phases, unsupported=()):
    jsonschema.validate({'plan': list(phases), 'unsupported': list(unsupported)},
                        SCHEMA)


def rejects(*phases):
    with pytest.raises(jsonschema.ValidationError):
        validate(*phases)


def _intent(*phases, unsupported=()):
    return {'plan': list(phases), 'unsupported': list(unsupported)}


# -- schema: each branch validates its own shape --------------------------

@pytest.mark.parametrize('phase', [STRAIGHT, TURN, GO_TO], ids=['straight', 'turn', 'go_to'])
def test_each_branch_validates_its_own_shape(phase):
    validate(phase)


def test_all_three_branches_compose_in_one_plan():
    validate(STRAIGHT, TURN, GO_TO)


def test_go_to_rejects_a_guard_key():
    """guard front_object is class-blind and bearing-blind: never on a go_to."""
    rejects({**GO_TO, 'guard': 'front_object'})
    rejects({**GO_TO, 'guard': 'distance'})


def test_go_to_rejects_a_dir_key():
    rejects({**GO_TO, 'dir': 'left'})


def test_straight_and_turn_reject_a_target_key():
    rejects({**STRAIGHT, 'target': 'bottle'})
    rejects({**TURN, 'target': 'bottle'})


def test_go_to_requires_mode_and_target_and_thresh_is_optional():
    validate({'mode': 'go_to', 'target': 'bottle'})
    rejects({'mode': 'go_to', 'thresh': 1.0})             # no target
    rejects({'target': 'bottle', 'thresh': 1.0})          # no mode


def test_additional_properties_stays_closed():
    rejects({**GO_TO, 'speed': 0.4})
    rejects({**GO_TO, 'timeout_sec': 30})
    rejects({**GO_TO, 'gap_m': 1.0})


def test_target_is_an_enum_not_a_free_string():
    assert isinstance(TARGETS, list) and len(TARGETS) > 10
    assert 'bottle' in TARGETS and 'person' in TARGETS and 'chair' in TARGETS


@pytest.mark.parametrize('bogus', ['unicorn', 'Bottle', 'bottle ', '', 'bottiglia', 'door'])
def test_a_class_the_detector_never_publishes_is_rejected(bogus):
    rejects({**GO_TO, 'target': bogus})


def test_every_enum_member_validates():
    for name in TARGETS:
        validate({**GO_TO, 'target': name})


def test_the_enum_is_sorted_and_free_of_duplicates():
    assert TARGETS == sorted(TARGETS)
    assert len(TARGETS) == len(set(TARGETS))


@pytest.mark.parametrize('thresh', [0.0, 0.1, 0.2, 1.0, 19.999, 20.0])
def test_go_to_thresh_accepts_zero_to_twenty(thresh):
    """Below gap_min must validate, so the translator can clamp it with a note."""
    validate({**GO_TO, 'thresh': thresh})


@pytest.mark.parametrize('thresh', [-0.01, -1.0, 20.01, 100.0])
def test_go_to_thresh_bounds_are_enforced(thresh):
    rejects({**GO_TO, 'thresh': thresh})


def test_the_straight_thresh_minimum_is_unchanged():
    rejects({**STRAIGHT, 'guard': 'distance', 'thresh': 0.1})


def test_thresh_must_be_a_number():
    rejects({**GO_TO, 'thresh': '1.0'})


def test_the_target_enum_matches_the_configured_detector_checkpoint():
    """The enum is a snapshot of the checkpoint's classes; drift must fail.

    Regenerate with tools/gen_intent_target_enum.py.
    """
    pytest.importorskip('torch', reason='needed to read the checkpoint')

    repo = SCHEMA_PATH.resolve().parents[4]
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'gen_enum', repo / 'tools' / 'gen_intent_target_enum.py')
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)

    model = gen.configured_model_name()
    checkpoint = gen.MODELS / model
    if not checkpoint.exists() or checkpoint.suffix != '.pt':
        pytest.skip(f'configured model {model!r} is not a readable .pt checkpoint')

    assert TARGETS == gen.class_names(checkpoint), (
        f'schema enum has drifted from {model}; '
        'rerun tools/gen_intent_target_enum.py')


# -- translator: go_to -> go_to_object ------------------------------------

class TestTranslatorFields:

    def test_a_go_to_phase_becomes_one_go_to_object_move(self):
        result = translate(_intent(GO_TO), gap_limits_for=_limits())
        mission = result.mission
        assert mission['schema_version'] == OBJECT_MISSION_SCHEMA_VERSION == '5.0'
        [move] = mission['moves']
        assert set(move) == {'id', 'go_to_object', 'stop_condition', 'timeout_sec',
                             'on_timeout', 'terminal'}
        assert move['stop_condition'] == {'type': 'object_reached'}
        assert move['on_timeout'] == 'abort'
        assert move['terminal'] is True
        assert result.requires_confirmation is False
        assert result.notes == ()

    def test_target_class_is_the_intent_target_and_gap_is_thresh(self):
        cfg = TranslatorConfig()
        body = translate(_intent({'mode': 'go_to', 'target': 'chair', 'thresh': 1.0}),
                         gap_limits_for=_limits()).mission['moves'][0]['go_to_object']
        assert body == {
            'target_class': 'chair', 'gap_m': 1.0, 'speed': cfg.speed_go_to,
            'acquire_timeout_sec': cfg.go_to_acquire_timeout_sec,
            'lost_grace_sec': cfg.go_to_lost_grace_sec,
        }

    def test_no_thresh_gives_the_class_default_gap(self):
        move = translate(_intent({'mode': 'go_to', 'target': 'person'}),
                         gap_limits_for=_limits(gap_min=0.42)).mission['moves'][0]
        assert move['go_to_object']['gap_m'] == 0.5

    def test_the_default_gap_follows_the_class_margin(self):
        result = translate(_intent({'mode': 'go_to', 'target': 'person'}),
                           gap_limits_for=_limits(gap_min=0.72, class_margin=0.3))
        assert result.mission['moves'][0]['go_to_object']['gap_m'] == 0.8

    def test_the_default_gap_comes_from_stack_params_without_an_override(self):
        from f1tenth_params.object_geometry import gap_limits
        mission = translate(_intent({'mode': 'go_to', 'target': 'person'})).mission
        assert mission['moves'][0]['go_to_object']['gap_m'] == pytest.approx(
            gap_limits('person').default_gap)

    def test_thresh_is_rounded_to_centimetres(self):
        result = translate(_intent({**GO_TO, 'thresh': 1.234}), gap_limits_for=_limits())
        assert result.mission['moves'][0]['go_to_object']['gap_m'] == 1.23

    def test_the_timeout_is_sized_like_an_open_guard(self):
        """open_guard_max_distance / speed_go_to, same formula as front_object."""
        cfg = TranslatorConfig()
        move = translate(_intent(GO_TO), gap_limits_for=_limits()).mission['moves'][0]
        expected_s = cfg.open_guard_max_distance / cfg.speed_go_to
        assert move['timeout_sec'] == math.ceil(
            expected_s * cfg.timeout_factor + cfg.timeout_floor)

    def test_the_gap_and_target_name_their_source(self):
        prov = translate(_intent(GO_TO), explain=True,
                         gap_limits_for=_limits()).provenance
        assert prov['moves[0].go_to_object.gap_m'][0] == INTENT
        assert prov['moves[0].go_to_object.target_class'][0] == INTENT
        assert prov['moves[0].stop_condition.type'][0] == CONST
        default = translate(_intent({'mode': 'go_to', 'target': 'bottle'}), explain=True,
                            gap_limits_for=_limits()).provenance
        assert default['moves[0].go_to_object.gap_m'][0] == DERIVED

    def test_go_to_never_falls_through_to_a_straight_move(self):
        """The old failure: a go_to landing in the guard branch."""
        move = translate(_intent(GO_TO), gap_limits_for=_limits()).mission['moves'][0]
        assert 'drive' not in move
        assert move['stop_condition']['type'] != 'obstacle_distance_below'

    def test_a_plan_without_go_to_keeps_schema_3_0(self):
        assert translate(_intent(STRAIGHT, TURN)).mission['schema_version'] == '3.0'


class TestClamp:

    def test_a_gap_below_gap_min_is_raised_to_it_with_an_italian_note(self):
        result = translate(_intent({'mode': 'go_to', 'target': 'person', 'thresh': 0.1}),
                           explain=True, gap_limits_for=_limits(gap_min=0.42))
        assert result.mission['moves'][0]['go_to_object']['gap_m'] == 0.42
        [note] = result.notes
        assert 'distanza richiesta 0.10 m' in note
        assert 'applicata 0.42 m' in note
        assert '"person"' in note
        assert result.provenance['moves[0].go_to_object.gap_m'][0] == DERIVED
        assert result.requires_confirmation is False, 'a larger gap is the safe side'

    def test_zero_is_clamped_too(self):
        result = translate(_intent({**GO_TO, 'thresh': 0.0}), gap_limits_for=_limits())
        assert result.mission['moves'][0]['go_to_object']['gap_m'] == 0.42
        assert len(result.notes) == 1

    def test_the_clamp_rounds_up_so_the_loader_accepts_it(self):
        """gap_min 0.425 must clamp to 0.43, not round to 0.42 below the minimum."""
        result = translate(_intent({**GO_TO, 'thresh': 0.2}),
                           gap_limits_for=_limits(gap_min=0.425))
        assert result.mission['moves'][0]['go_to_object']['gap_m'] == 0.43

    def test_exactly_gap_min_is_not_clamped(self):
        result = translate(_intent({**GO_TO, 'thresh': 0.42}),
                           gap_limits_for=_limits(gap_min=0.42))
        assert result.mission['moves'][0]['go_to_object']['gap_m'] == 0.42
        assert result.notes == ()

    def test_one_note_per_clamped_phase(self):
        result = translate(
            _intent({**GO_TO, 'thresh': 0.1}, TURN, {**GO_TO, 'target': 'chair', 'thresh': 0.3}),
            gap_limits_for=_limits())
        assert [n.split(',')[0] for n in result.notes] == ['fase 0', 'fase 2']


class TestMultiPhase:

    def test_go_to_then_turn_is_two_moves_with_the_terminal_on_the_turn(self):
        moves = translate(_intent({'mode': 'go_to', 'target': 'bottle'},
                                  {'mode': 'turn', 'dir': 'right'}),
                          gap_limits_for=_limits()).mission['moves']
        assert [m['id'] for m in moves] == ['move_0_go_to', 'move_1_turn']
        assert 'terminal' not in moves[0]
        assert moves[1]['terminal'] is True
        assert moves[1]['drive']['turn_sign'] == -1.0

    def test_move_ids_and_wire_ids_are_unique_with_two_go_tos(self):
        from f1tenth_behavior.mission.object_handler import object_move_wire_id
        mission = translate(_intent(STRAIGHT, GO_TO, TURN, {**GO_TO, 'target': 'chair'}),
                            gap_limits_for=_limits()).mission
        ids = [m['id'] for m in mission['moves']]
        assert ids == ['move_0_straight', 'move_1_go_to', 'move_2_turn', 'move_3_go_to']
        wires = {object_move_wire_id(mission['mission_id'], 1, i) for i in ids}
        assert len(wires) == len(ids)

    def test_the_loader_accepts_the_multi_phase_mission(self):
        from f1tenth_behavior.mission.mission_config import parse_mission
        config = parse_mission(translate(_intent(STRAIGHT, GO_TO, TURN)).mission)
        assert config.moves[1].go_to_object.target_class == 'bottle'
        assert config.moves[1].go_to_object.gap_m == 1.0
        assert config.moves[2].terminal is True


# -- go_to_enabled = false: the refusal before go_to_object, text for text --

class TestDisabledTranslator:

    def test_translate_refuses_go_to_with_the_text_it_always_had(self):
        with pytest.raises(UnsupportedIntentModeError) as excinfo:
            translate(_intent(GO_TO), go_to_enabled=False)
        assert isinstance(excinfo.value, IntentSchemaError), 'still fed back as retry text'
        assert str(excinfo.value) == REFUSAL_BEFORE_GO_TO.format(i=0)

    def test_the_refusal_survives_being_mixed_with_executable_phases(self):
        with pytest.raises(UnsupportedIntentModeError) as excinfo:
            translate(_intent(STRAIGHT, GO_TO), go_to_enabled=False)
        assert str(excinfo.value) == REFUSAL_BEFORE_GO_TO.format(i=1)

    def test_executable_intents_are_unaffected_by_the_switch(self):
        on = translate(_intent(STRAIGHT, TURN))
        off = translate(_intent(STRAIGHT, TURN), go_to_enabled=False)
        assert on == off

    def test_the_prompt_file_follows_the_switch(self):
        assert intent_prompt_filename(True) == INTENT_PROMPT_FILENAME
        assert intent_prompt_filename(False) == INTENT_PROMPT_NO_GO_TO_FILENAME


def test_go_to_enabled_ships_true_in_stack_params():
    """Coupling: the default the floor test runs with."""
    from llm.llm_planner_node import go_to_enabled_default
    assert go_to_enabled_default() is True


# -- the planner node, both values of the switch --------------------------

class _Logger:
    """Captures what the node logged, by level."""

    def __init__(self):
        self.warn_lines = []
        self.error_lines = []
        self.info_lines = []

    def warn(self, message):
        self.warn_lines.append(str(message))

    def error(self, message):
        self.error_lines.append(str(message))

    def info(self, message):
        self.info_lines.append(str(message))


class _Stub:
    """The attributes _plan_v2 and process_command touch on self."""

    def __init__(self, go_to_enabled):
        self._system_prompt = 'unused: the LLM call is patched'
        self._logger = _Logger()
        self._go_to_enabled = go_to_enabled
        self._planner_path = 'v2'
        self.opts = SimpleNamespace(dry_run=True, confirm=False)
        self.written = None

    def get_logger(self):
        return self._logger

    def _plan_v2(self, command_text):
        from llm.llm_planner_node import LLMPlannerNode
        return LLMPlannerNode._plan_v2(self, command_text)

    def _write_mission_file(self, mission):
        self.written = mission
        return '/nonexistent/unused.json'


def _run_plan_v2(monkeypatch, capsys, response, command, go_to_enabled):
    from llm import llm_planner_node
    from llm.llm_planner_node import LLMPlannerNode

    calls = []

    def fake(command_text, system_prompt, feedback=None):
        calls.append(feedback)
        return response

    monkeypatch.setattr(llm_planner_node, 'get_intent_from_llm', fake)
    stub = _Stub(go_to_enabled)
    result = LLMPlannerNode._plan_v2(stub, command)
    return result, calls, stub._logger, capsys.readouterr().out


class TestDisabledPlanner:
    """go_to_enabled=False: exactly the planner before go_to_object."""

    def test_a_go_to_answer_terminates_as_unsupported_not_exhaustion(
            self, monkeypatch, capsys):
        from llm.llm_planner_node import MAX_INTENT_RETRIES

        command = 'vai verso la bottiglia e fermati a 1 metro'
        result, calls, logger, out = _run_plan_v2(
            monkeypatch, capsys, _intent(GO_TO), command, go_to_enabled=False)

        assert result is None, 'no mission may be emitted'
        assert len(calls) == MAX_INTENT_RETRIES + 1, 'the whole budget is spent'
        assert calls[0] is None
        assert calls[1] == REFUSAL_BEFORE_GO_TO.format(i=0), 'the refusal is fed back'
        assert 'RICHIESTA NON SUPPORTATA -- nessuna missione emessa.' in out
        assert f'  richiesta: {command}' in out
        assert any('non supportata' in line for line in logger.error_lines)
        assert any('[prompt-non-rispettato]' in line for line in logger.warn_lines)

    def test_exhaustion_does_not_fall_back_to_the_last_valid_plan(self, monkeypatch):
        from llm import llm_planner_node
        from llm.llm_planner_node import LLMPlannerNode

        executable = _intent({'mode': 'straight', 'guard': 'front_object', 'thresh': 1.0})
        stub = _Stub(go_to_enabled=False)
        monkeypatch.setattr(llm_planner_node, 'get_intent_from_llm',
                            lambda *a, **k: executable)
        assert LLMPlannerNode._plan_v2(stub, 'vai verso la bottiglia') is not None
        monkeypatch.setattr(llm_planner_node, 'get_intent_from_llm',
                            lambda *a, **k: _intent(GO_TO))
        assert LLMPlannerNode._plan_v2(stub, 'vai verso la bottiglia') is None

    def test_a_complied_refusal_prints_the_banner_it_always_printed(
            self, monkeypatch, capsys):
        _, _, logger, out = _run_plan_v2(
            monkeypatch, capsys, _intent(unsupported=['vai verso la bottiglia']),
            'vai verso la bottiglia', go_to_enabled=False)
        expected = ('\nRICHIESTA NON SUPPORTATA -- il robot non sa farlo, '
                    'e non e\' un errore del planner:\n'
                    '  - vai verso la bottiglia\n'
                    + SUPPORTED_BEFORE_GO_TO + '\n')
        assert out == expected
        assert not any('[prompt-non-rispettato]' in line for line in logger.warn_lines)


class TestEnabledPlanner:
    """go_to_enabled=True: the new branch, and the banner for what is still unsupported."""

    def test_a_go_to_answer_plans_first_time(self, monkeypatch, capsys):
        result, calls, logger, _ = _run_plan_v2(
            monkeypatch, capsys, _intent({'mode': 'go_to', 'target': 'person'}),
            'vai dalla persona', go_to_enabled=True)
        mission, unsupported, _dt, attempts, notes = result
        assert attempts == 1 and calls == [None]
        assert mission['moves'][0]['go_to_object']['target_class'] == 'person'
        assert unsupported == () and notes == ()
        assert not logger.warn_lines

    def test_the_clamp_note_reaches_the_operator(self, monkeypatch, capsys):
        from llm import llm_planner_node
        from llm.llm_planner_node import LLMPlannerNode

        stub = _Stub(go_to_enabled=True)
        monkeypatch.setattr(llm_planner_node, 'get_intent_from_llm', lambda *a, **k: _intent(
            {'mode': 'go_to', 'target': 'person', 'thresh': 0.1}))
        started = LLMPlannerNode.process_command(
            stub, 'fermati a dieci centimetri dalla persona')
        assert started is False, 'dry run'
        out = capsys.readouterr().out
        assert 'NOTA: fase 0, go_to "person": distanza richiesta 0.10 m, applicata' in out
        assert 'vai verso "person" (il piu\' vicino)' in out
        assert stub.written['moves'][0]['go_to_object']['gap_m'] >= 0.42

    def test_a_still_unsupported_request_keeps_the_banner(self, monkeypatch, capsys):
        result, calls, _, out = _run_plan_v2(
            monkeypatch, capsys, _intent(unsupported=['vai da Marco']),
            'vai da Marco', go_to_enabled=True)
        assert result is None and len(calls) == 1
        assert out.startswith('\nRICHIESTA NON SUPPORTATA -- il robot non sa farlo')
        assert '  - vai da Marco' in out
        assert SUPPORTED_BEFORE_GO_TO in out, 'what was supported still is'
        assert '"vai dalla persona"' in out

    def test_ambiguity_and_unsupported_do_not_share_a_message(self, monkeypatch, capsys):
        _, _, _, ambiguous_out = _run_plan_v2(
            monkeypatch, capsys, _intent(unsupported=['ambiguo: distanza non specificata']),
            "gira e vai avanti un po'", go_to_enabled=True)
        assert 'ambiguo' in ambiguous_out
        assert 'RICHIESTA NON SUPPORTATA' not in ambiguous_out


# -- the prompts are part of the contract ---------------------------------

class TestPromptWithGoTo:

    @pytest.fixture(scope='class')
    def prompt(self):
        return load_intent_prompt()

    def test_it_teaches_go_to(self, prompt):
        assert '{"mode":"go_to","target":<classe>}' in prompt

    def test_it_lists_exactly_the_schema_classes(self, prompt):
        """The in/out-of-enum rule needs the enum; a stale list teaches the wrong one."""
        block = prompt.split('\nCLASSI:\n', 1)[1].split('\n\n', 1)[0]
        listed = [c.strip() for c in block.replace('\n', ' ').split(',')]
        assert listed == TARGETS

    @pytest.mark.parametrize('italian, coco', [
        ('persona', 'person'), ('sedia', 'chair'), ('bottiglia', 'bottle'),
        ('tavolo', 'dining table'), ('divano', 'couch'), ('zaino', 'backpack')])
    def test_it_maps_italian_names_to_coco_classes(self, prompt, italian, coco):
        assert f'{italian} -> "{coco}"' in prompt

    def test_it_names_the_three_unsupported_kinds(self, prompt):
        assert '"vai dalla porta"' in prompt           # not a class
        assert '"vai da Marco"' in prompt              # an individual
        assert '"la seconda sedia"' in prompt          # a selection
        assert "PIU' VICINO" in prompt

    def test_front_object_stays_the_generic_guard(self, prompt):
        assert 'NON usare "front_object" per una cosa nominata' in prompt
        assert 'COMPLETATA' in prompt
        assert "fermati prima dell'ostacolo" in prompt

    def test_its_examples_cover_the_work_order_commands(self):
        examples = dict(intent_prompt_examples())
        assert examples['vai dalla persona']['plan'] == [{'mode': 'go_to', 'target': 'person'}]
        assert examples['raggiungi la sedia e fermati a un metro']['plan'] == [
            {'mode': 'go_to', 'target': 'chair', 'thresh': 1.0}]
        assert examples['vai verso la bottiglia e poi gira a destra']['plan'] == [
            {'mode': 'go_to', 'target': 'bottle'}, {'mode': 'turn', 'dir': 'right'}]
        assert examples['fermati a dieci centimetri dalla persona']['plan'] == [
            {'mode': 'go_to', 'target': 'person', 'thresh': 0.1}]
        assert examples['vai dalla porta'] == _intent(unsupported=['vai dalla porta'])
        assert examples['vai da Marco'] == _intent(unsupported=['vai da Marco'])


@pytest.mark.parametrize('prompt_file', [INTENT_PROMPT_FILENAME, INTENT_PROMPT_NO_GO_TO_FILENAME])
def test_no_prompt_example_pairs_a_named_object_with_front_object(prompt_file):
    """Examples are the model's strongest signal; this pairing is the forbidden one."""
    named = ['sedia', 'bottiglia', 'tavolo', 'persona', 'chair', 'bottle']
    for command, intent in intent_prompt_examples(prompt_file):
        if any(p.get('guard') == 'front_object' for p in intent['plan']):
            assert not any(word in command.lower() for word in named), command


class TestPromptWithoutGoTo:
    """The prompt go_to_enabled=False loads: the one before go_to was taught."""

    @pytest.fixture(scope='class')
    def prompt(self):
        return load_intent_prompt(INTENT_PROMPT_NO_GO_TO_FILENAME)

    def test_it_never_advertises_go_to(self, prompt):
        assert 'go_to' not in prompt

    def test_it_names_the_unsupported_request_class(self, prompt):
        for phrase in ['vai verso la bottiglia', 'avvicinati alla persona',
                       'segui la sedia', 'portati davanti al tavolo',
                       'go to the bottle', 'drive toward the person',
                       'approach the chair', 'follow me',
                       'stop one metre from the table']:
            assert phrase in prompt, f'{phrase!r} missing from the prompt'

    def test_it_forbids_the_substitution_and_says_why(self, prompt):
        assert 'NON sostituirli con "front_object"' in prompt
        assert 'PIU\' VICINO' in prompt
        assert 'COMPLETATA' in prompt
        assert 'soglia indovinata' in prompt
        assert '"turn" + "straight"' in prompt

    def test_it_still_permits_the_unnamed_case(self, prompt):
        assert 'SUPPORTATO' in prompt
        assert "fermati prima dell'ostacolo" in prompt
        assert 'se il comando nomina una cosa specifica' in prompt

    def test_its_refusal_examples_preserve_the_operator_wording(self):
        refusals = [(c, i) for c, i in intent_prompt_examples(INTENT_PROMPT_NO_GO_TO_FILENAME)
                    if i['unsupported'] and 'bottiglia' in c]
        assert refusals
        for command, intent in refusals:
            assert any('bottiglia' in entry for entry in intent['unsupported']), command


def test_a_changed_gap_limit_gives_a_new_mission_id():
    """The planner refuses to overwrite a mission file whose id matches but content does not."""
    intent = _intent({'mode': 'go_to', 'target': 'person'})
    a = translate(intent, gap_limits_for=_limits(gap_min=0.42)).mission
    b = translate(intent, gap_limits_for=_limits(gap_min=0.72, class_margin=0.3)).mission
    assert a['moves'][0]['go_to_object']['gap_m'] != b['moves'][0]['go_to_object']['gap_m']
    assert a['mission_id'] != b['mission_id']
    assert translate(intent, gap_limits_for=_limits(gap_min=0.42)).mission == a
