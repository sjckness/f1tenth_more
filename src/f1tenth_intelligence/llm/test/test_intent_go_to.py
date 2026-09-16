"""The go_to intent branch: shape, strictness, and the wall it hits.

go_to is in intent_v1.json and is NOT executable by this stack. The schema
branch exists so the vocabulary is written down in one place and so a model
emitting it gets a precise rejection; the translator refuses it explicitly so
that refusal is a handled failure rather than a traceback. Both halves are
pinned here, because the dangerous state is a schema that accepts something
the runtime silently mishandles.
"""

import json
import pathlib

import jsonschema
import pytest

from llm.plan_translate import (
    IntentSchemaError,
    UnsupportedIntentModeError,
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


def validate(*phases, unsupported=()):
    jsonschema.validate({'plan': list(phases), 'unsupported': list(unsupported)},
                        SCHEMA)


def rejects(*phases):
    with pytest.raises(jsonschema.ValidationError):
        validate(*phases)


# -- each branch validates its own shape ----------------------------------

@pytest.mark.parametrize('phase', [STRAIGHT, TURN, GO_TO], ids=['straight', 'turn', 'go_to'])
def test_each_branch_validates_its_own_shape(phase):
    validate(phase)


def test_all_three_branches_compose_in_one_plan():
    validate(STRAIGHT, TURN, GO_TO)


# -- and rejects the other branches' fields -------------------------------

def test_go_to_rejects_a_guard_key():
    """The specific confusion the branch exists to prevent.

    guard 'front_object' maps to obstacle_distance_below over a class-blind,
    bearing-blind forward scalar. Accepting it on a go_to would mean steering
    at the target and stopping at whatever is nearest, reporting success.
    """
    rejects({**GO_TO, 'guard': 'front_object'})
    rejects({**GO_TO, 'guard': 'distance'})


def test_go_to_rejects_a_dir_key():
    rejects({**GO_TO, 'dir': 'left'})


def test_straight_and_turn_reject_a_target_key():
    rejects({**STRAIGHT, 'target': 'bottle'})
    rejects({**TURN, 'target': 'bottle'})


def test_go_to_requires_every_one_of_its_fields():
    rejects({'mode': 'go_to', 'target': 'bottle'})        # no thresh
    rejects({'mode': 'go_to', 'thresh': 1.0})             # no target
    rejects({'target': 'bottle', 'thresh': 1.0})          # no mode


def test_additional_properties_stays_closed():
    rejects({**GO_TO, 'speed': 0.4})
    rejects({**GO_TO, 'timeout_sec': 30})


# -- target enum ----------------------------------------------------------

def test_target_is_an_enum_not_a_free_string():
    assert isinstance(TARGETS, list) and len(TARGETS) > 10
    assert 'bottle' in TARGETS and 'person' in TARGETS and 'chair' in TARGETS


@pytest.mark.parametrize('bogus', ['unicorn', 'Bottle', 'bottle ', '', 'bottiglia'])
def test_a_class_the_detector_never_publishes_is_rejected(bogus):
    """A free string would validate and then never fire. This is the point."""
    rejects({**GO_TO, 'target': bogus})


def test_every_enum_member_validates():
    for name in TARGETS:
        validate({**GO_TO, 'target': name})


def test_the_enum_is_sorted_and_free_of_duplicates():
    assert TARGETS == sorted(TARGETS)
    assert len(TARGETS) == len(set(TARGETS))


# -- thresh bounds --------------------------------------------------------

@pytest.mark.parametrize('thresh', [0.2, 1.0, 19.999, 20.0])
def test_thresh_accepts_its_documented_range(thresh):
    validate({**GO_TO, 'thresh': thresh})


@pytest.mark.parametrize('thresh', [0.19, 0.0, -1.0, 20.01, 100.0])
def test_thresh_bounds_are_enforced(thresh):
    rejects({**GO_TO, 'thresh': thresh})


def test_thresh_must_be_a_number():
    rejects({**GO_TO, 'thresh': '1.0'})


# -- the translator refuses it, and refuses it *cleanly* ------------------

def test_translate_refuses_go_to_as_a_handled_failure():
    """Not a KeyError.

    Without the explicit guard the phase falls through to the straight
    branch and dies on phase['guard']. _plan_v2 catches only
    PlanTranslationError subclasses, so a KeyError would surface as a
    traceback in the planner instead of as retry feedback.
    """
    with pytest.raises(UnsupportedIntentModeError) as excinfo:
        translate({'plan': [GO_TO], 'unsupported': []})

    # Still an IntentSchemaError, so _plan_v2 keeps feeding it back as retry
    # text; a distinct subclass so the node can log it as a prompt-compliance
    # failure rather than a malformed plan.
    assert isinstance(excinfo.value, IntentSchemaError)

    message = str(excinfo.value)
    assert 'go_to' in message
    assert 'front_object' in message, 'must warn off the false-success shortcut'
    assert 'unsupported' in message, 'must tell the model where to put it'
    assert len(message) < 300, 'retry text competes with the system prompt'
    assert 'DRIVE_MODES' not in message, 'model-facing, not log-facing'


def test_the_refusal_survives_being_mixed_with_executable_phases():
    with pytest.raises(UnsupportedIntentModeError):
        translate({'plan': [STRAIGHT, GO_TO], 'unsupported': []})


def test_executable_intents_are_unaffected_by_the_new_branch():
    """The branch must not perturb the two modes that do work."""
    result = translate({'plan': [STRAIGHT, TURN], 'unsupported': []})

    assert len(result.mission['moves']) == 2
    assert result.mission['moves'][0]['drive']['mode'] == 'straight'


# -- the enum is a snapshot, so drift must fail a test --------------------

def test_the_target_enum_matches_the_configured_detector_checkpoint():
    """The class vocabulary is not defined as code anywhere in this repo.

    yolo_detector_node._resolve_class_names() reads it out of the YOLO
    checkpoint at runtime, so which strings exist depends on
    stack_params.yaml's `yolo_model`. The two checkpoints in the tree share
    only 10 classes: yolo26s-seg.pt has 80 COCO names, yolo26_office.pt has
    58 office names, 48 of them not in COCO. A static enum is therefore a
    snapshot, and this test is what stops the snapshot going stale --
    regenerate with tools/gen_intent_target_enum.py.
    """
    pytest.importorskip('torch', reason='needed to read the checkpoint')

    repo = SCHEMA_PATH.resolve().parents[4]
    sys_path = repo / 'tools'
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'gen_enum', sys_path / 'gen_intent_target_enum.py')
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)

    model = gen.configured_model_name()
    checkpoint = gen.MODELS / model
    if not checkpoint.exists() or checkpoint.suffix != '.pt':
        pytest.skip(f'configured model {model!r} is not a readable .pt checkpoint')

    assert TARGETS == gen.class_names(checkpoint), (
        f'schema enum has drifted from {model}; '
        'rerun tools/gen_intent_target_enum.py')


# -- B1/B3: how the retry budget terminates -------------------------------

class _Logger:
    """Captures what the node logged, by level."""

    def __init__(self):
        self.warn_lines = []
        self.error_lines = []

    def warn(self, message):
        self.warn_lines.append(str(message))

    def error(self, message):
        self.error_lines.append(str(message))

    def info(self, message):
        pass


class _Stub:
    """The only two attributes _plan_v2 touches on self."""

    def __init__(self):
        self._system_prompt = 'unused: the LLM call is patched'
        self._logger = _Logger()

    def get_logger(self):
        return self._logger


def _run_plan_v2(monkeypatch, capsys, response, command):
    from llm import llm_planner_node
    from llm.llm_planner_node import LLMPlannerNode

    calls = []

    def fake(command_text, system_prompt, feedback=None):
        calls.append(feedback)
        return response

    monkeypatch.setattr(llm_planner_node, 'get_intent_from_llm', fake)
    stub = _Stub()
    result = LLMPlannerNode._plan_v2(stub, command)
    return result, calls, stub._logger, capsys.readouterr().out


def test_an_unsupported_request_terminates_as_unsupported_not_exhaustion(
        monkeypatch, capsys):
    """B1. The budget must end in a stated refusal carrying the request.

    A model that keeps re-emitting go_to burns every retry. What the operator
    must not get is a bare "planning failed", a traceback, or -- worst --
    whatever plan last validated, because for a request the robot cannot
    perform that is exactly the wrong-object substitution.
    """
    from llm.llm_planner_node import MAX_INTENT_RETRIES

    command = 'vai verso la bottiglia e fermati a 1 metro'
    result, calls, logger, out = _run_plan_v2(
        monkeypatch, capsys,
        {'plan': [GO_TO], 'unsupported': []}, command)

    assert result is None, 'no mission may be emitted'
    assert len(calls) == MAX_INTENT_RETRIES + 1, 'the whole budget is spent'
    assert calls[0] is None and calls[1] is not None, 'the refusal is fed back'

    assert 'RICHIESTA NON SUPPORTATA' in out
    assert command in out, "the operator's own words must appear"
    assert any('non supportata' in line for line in logger.error_lines)
    assert not any('Traceback' in line for line in logger.error_lines)


def test_exhaustion_does_not_fall_back_to_the_last_valid_plan(
        monkeypatch, capsys):
    """The specific silent failure B1 rules out.

    The model emits a perfectly executable front_object plan first, then
    go_to. If exhaustion fell back to "the last thing that validated", the
    car would drive off and stop at the nearest object of any kind.
    """
    from llm import llm_planner_node
    from llm.llm_planner_node import LLMPlannerNode

    executable = {'plan': [{'mode': 'straight', 'guard': 'front_object',
                            'thresh': 1.0}],
                  'unsupported': []}
    refused = {'plan': [GO_TO], 'unsupported': []}
    seen = []

    def fake(command_text, system_prompt, feedback=None):
        seen.append(feedback)
        # First answer translates cleanly; every later one does not.
        return executable if len(seen) == 1 else refused

    monkeypatch.setattr(llm_planner_node, 'get_intent_from_llm', fake)
    stub = _Stub()

    first = LLMPlannerNode._plan_v2(stub, 'vai verso la bottiglia')
    assert first is not None, 'sanity: a translatable answer is accepted'

    # Now the same node, with only untranslatable answers.
    seen.clear()

    def only_refused(command_text, system_prompt, feedback=None):
        seen.append(feedback)
        return refused

    monkeypatch.setattr(llm_planner_node, 'get_intent_from_llm', only_refused)
    capsys.readouterr()

    assert LLMPlannerNode._plan_v2(stub, 'vai verso la bottiglia') is None, (
        'no mission may be returned, and in particular not the one that '
        'validated on an earlier command')


def test_the_two_refusal_outcomes_are_distinguishable_in_logs(
        monkeypatch, capsys):
    """B3. Ignoring the prompt and complying with it must not look alike.

    A model that authored go_to ignored the system prompt, which is a
    prompt-quality signal worth counting. A model that routed the request to
    "unsupported" did what it was told and never reaches the translator.
    """
    _, _, ignored_logger, _ = _run_plan_v2(
        monkeypatch, capsys, {'plan': [GO_TO], 'unsupported': []},
        'vai verso la bottiglia')

    _, _, complied_logger, complied_out = _run_plan_v2(
        monkeypatch, capsys,
        {'plan': [], 'unsupported': ['vai verso la bottiglia']},
        'vai verso la bottiglia')

    assert any('[prompt-non-rispettato]' in line
               for line in ignored_logger.warn_lines)
    assert not any('[prompt-non-rispettato]' in line
                   for line in complied_logger.warn_lines), 'compliance is not a fault'
    # Compliance now prints the "not supported" banner, not the ambiguity
    # one: an operator reading "comando ambiguo" for a correctly refused
    # request concludes the planner is broken.
    assert 'RICHIESTA NON SUPPORTATA' in complied_out
    assert 'non e\' un errore del planner' in complied_out


# -- A1-A4: the prompt text itself is part of the contract ----------------

def _prompt():
    from llm.plan_translate import load_intent_prompt
    return load_intent_prompt()


def test_the_prompt_never_advertises_go_to():
    """A1. The branch is in the schema; the model is not told it exists.

    Advertising a mode that is always refused is a contradictory contract:
    the model would author it and be rejected every time. The prompt flips
    when a runtime exists.
    """
    assert 'go_to' not in _prompt()


def test_the_prompt_names_the_unsupported_request_class():
    """A2, in both languages the operators actually use."""
    prompt = _prompt()

    for phrase in ['vai verso la bottiglia', 'avvicinati alla persona',
                   'segui la sedia', 'portati davanti al tavolo',
                   'go to the bottle', 'drive toward the person',
                   'approach the chair', 'follow me',
                   'stop one metre from the table']:
        assert phrase in prompt, f'{phrase!r} missing from the prompt'


def test_the_prompt_forbids_the_substitution_and_says_why():
    """A3. A bare prohibition is complied with less reliably than a reason."""
    prompt = _prompt()

    assert 'NON sostituirli con "front_object"' in prompt
    assert 'PIU\' VICINO' in prompt, 'must say front_object measures the nearest'
    assert 'COMPLETATA' in prompt, 'must say the mission reports success'
    assert 'soglia indovinata' in prompt, 'must forbid a guessed distance'
    assert '"turn" + "straight"' in prompt, 'must forbid the aiming sequence'


def test_the_prompt_still_permits_the_unnamed_case():
    """A4. Over-refusal is a regression too."""
    prompt = _prompt()

    assert 'SUPPORTATO' in prompt
    assert "fermati prima dell'ostacolo" in prompt
    assert 'se il comando nomina una cosa specifica' in prompt


def test_no_prompt_example_pairs_a_named_object_with_front_object():
    """The example that used to teach exactly the forbidden substitution.

    "vai avanti e fermati davanti alla sedia" -> front_object was a worked
    example, and examples are the model's strongest signal. It now reads
    "prima dell'ostacolo".
    """
    from llm.plan_translate import intent_prompt_examples

    named = ['sedia', 'bottiglia', 'tavolo', 'persona', 'chair', 'bottle']
    for command, intent in intent_prompt_examples():
        uses_front_object = any(
            p.get('guard') == 'front_object' for p in intent['plan'])
        if not uses_front_object:
            continue
        assert not any(word in command.lower() for word in named), (
            f'{command!r} teaches the forbidden pairing')


def test_the_refusal_examples_preserve_the_operator_wording():
    """A2 requires the original text, not a paraphrase, in unsupported."""
    from llm.plan_translate import intent_prompt_examples

    refusals = [(c, i) for c, i in intent_prompt_examples()
                if i['unsupported'] and 'bottiglia' in c]
    assert refusals, 'the prompt must show at least one named-object refusal'

    for command, intent in refusals:
        assert any('bottiglia' in entry for entry in intent['unsupported']), (
            f'{command!r} loses the operator wording: {intent["unsupported"]}')


def test_ambiguity_and_unsupported_do_not_share_a_message(monkeypatch, capsys):
    """An operator who reads "comando ambiguo" concludes the planner is broken.

    Both outcomes come through EmptyPlanError, and before the named-object
    rule landed almost every one of them really was an ambiguity. Now the
    common case is a request the robot cannot perform, and saying "ambiguo"
    for it sends the operator looking for a fault that is not there. The
    prompt already marks true ambiguity with an "ambiguo:" prefix, so that is
    what splits them.
    """
    _, _, _, ambiguous_out = _run_plan_v2(
        monkeypatch, capsys,
        {'plan': [], 'unsupported': ['ambiguo: distanza non specificata']},
        "gira e vai avanti un po'")

    _, _, _, unsupported_out = _run_plan_v2(
        monkeypatch, capsys,
        {'plan': [], 'unsupported': ['vai avanti e fermati davanti alla sedia']},
        'vai avanti e fermati davanti alla sedia')

    assert 'ambiguo' in ambiguous_out
    assert 'RICHIESTA NON SUPPORTATA' not in ambiguous_out

    assert 'RICHIESTA NON SUPPORTATA' in unsupported_out
    assert "fermati prima dell'ostacolo" in unsupported_out, (
        'the refusal must say what IS supported, or the operator is stuck')
