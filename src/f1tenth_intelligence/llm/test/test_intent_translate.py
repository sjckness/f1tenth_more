"""Tests for plan_translate.translate(): intent v1 -> mission v3.0.

The one that matters most is test_output_always_validates. Every other test
here checks a specific case; that one checks the INVARIANT -- that no
schema-valid intent can make the translator emit a mission the stack's own
loader rejects. A translator that can emit an invalid mission is worse than no
translator, because the failure surfaces later and somewhere else.
"""

import json
import math
import pathlib
import random

import pytest

import jsonschema

from llm.plan_translate import (
    CONFIG,
    CONST,
    DERIVED,
    EmptyPlanError,
    GUARD_THRESH_RANGE,
    INTENT,
    IntentRangeError,
    IntentSchemaError,
    TIMEOUT_MAX_SEC,
    TIMEOUT_MIN_SEC,
    TURN_SIGN,
    TranslatorConfig,
    TranslatorOutputError,
    intent_prompt_examples,
    load_intent_schema,
    translate,
)

GOLDEN_DIR = pathlib.Path(__file__).parent / 'golden'
MISSIONS_DIR = (pathlib.Path(__file__).resolve().parents[3]
                / 'f1tenth_behavior' / 'missions')
SOURCES = {INTENT, CONFIG, DERIVED, CONST}


def _golden(name):
    """Load one committed golden fixture by file stem."""
    with open(GOLDEN_DIR / f'{name}.json', encoding='utf-8') as fh:
        return json.load(fh)


def _golden_names():
    """Every golden fixture that carries a mission (i.e. not the empty-plan one)."""
    return sorted(p.stem for p in GOLDEN_DIR.glob('*.json') if 'mission' in _golden(p.stem))


def _leaf_paths(node, prefix=''):
    """Every scalar leaf of a mission document, as a dotted/indexed path.

    `moves` is indexed as moves[i] to match the provenance keys translate()
    records; everything else is a plain attribute path.
    """
    if isinstance(node, dict):
        for key, value in node.items():
            yield from _leaf_paths(value, f'{prefix}.{key}' if prefix else key)
    elif isinstance(node, list):
        for i, value in enumerate(node):
            yield from _leaf_paths(value, f'{prefix}[{i}]')
    else:
        yield prefix


# ---------------------------------------------------------------------------
# 1. provenance completeness
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('name', _golden_names())
def test_every_emitted_leaf_has_a_provenance_entry(name):
    """No field may be emitted without a recorded source.

    This is the test that keeps invention out. A new field added to translate()
    without a provenance entry fails here, which is the point: the four sources
    are the whole contract, and "it seemed like a sensible default" is not one
    of them.
    """
    case = _golden(name)
    result = translate(case['intent'], explain=True)
    missing = [p for p in _leaf_paths(result.mission) if p not in result.provenance]
    assert missing == [], f'{name}: leaves with no provenance: {missing}'


@pytest.mark.parametrize('name', _golden_names())
def test_every_provenance_entry_names_one_of_the_four_sources(name):
    """A provenance entry is only meaningful if its source is one of the four."""
    result = translate(_golden(name)['intent'], explain=True)
    for path, (source, detail) in result.provenance.items():
        assert source in SOURCES, f'{path}: {source!r} is not one of {sorted(SOURCES)}'
        assert detail, f'{path}: empty provenance detail'


def test_provenance_is_absent_unless_asked_for():
    """explain=False must not pay for the provenance structure."""
    assert translate(_golden('ex2_pass_obstacle')['intent']).provenance is None


# ---------------------------------------------------------------------------
# 2. determinism
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('name', _golden_names())
def test_the_same_intent_translates_byte_identically(name):
    """Two runs of one intent produce the same document, mission_id included.

    mission_id is a sha1 over the canonical intent, never a timestamp, so this
    holds across processes and across days -- which is what makes the golden
    fixtures below stable at all.
    """
    intent = _golden(name)['intent']
    first = json.dumps(translate(intent).mission, sort_keys=True)
    second = json.dumps(translate(intent).mission, sort_keys=True)
    assert first == second


def test_mission_id_changes_when_the_intent_changes():
    """Determinism must not mean collision: a different intent is a different id."""
    a = translate(_golden('ex2_pass_obstacle')['intent']).mission['mission_id']
    b = translate(_golden('ex3_stop_at_chair')['intent']).mission['mission_id']
    assert a != b


# ---------------------------------------------------------------------------
# 3. golden pairs -- the five prompt examples
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('name', _golden_names())
def test_golden_pair_still_translates_to_its_committed_mission(name):
    """The prompt's own examples, with their expected v3.0 output committed.

    A diff here is a deliberate behaviour change and the fixture should be
    regenerated in the same commit -- or it is a regression.
    """
    case = _golden(name)
    result = translate(case['intent'])
    assert result.mission == case['mission']
    assert result.requires_confirmation == case['requires_confirmation']


def test_prompt_example_five_emits_no_mission_at_all():
    """"gira e vai avanti un po'" is ambiguous: the correct output is nothing."""
    case = _golden('ex5_ambiguous_no_mission')
    with pytest.raises(EmptyPlanError):
        translate(case['intent'])


def test_no_golden_mission_carries_the_bug_that_motivated_this_work():
    """74.48451336700703 was degrees(1.3), and 1.3 came from the old prompt.

    Not a hypothetical: llm_planner_node.SYSTEM_PROMPT instructs the model to
    use `guard "turned" thresh 1.3`, and the legacy path rendered the
    conversion at full float precision. The intent format has no turn
    magnitude at all, so every turn is exactly turn_magnitude_deg.
    """
    for name in _golden_names():
        for move in _golden(name)['mission']['moves']:
            mag = move['drive'].get('turn_mag_deg')
            if mag is not None and move['drive']['mode'] == 'wall_turn':
                assert mag == TranslatorConfig().turn_magnitude_deg
                assert round(mag, 1) == mag


# ---------------------------------------------------------------------------
# 4. reference match against the known-good hand-written mission
# ---------------------------------------------------------------------------

def _reference_wall_turn():
    """missions/wall_turn.json -- hand-written, and run on hardware 2026-09-14."""
    with open(MISSIONS_DIR / 'wall_turn.json', encoding='utf-8') as fh:
        return json.load(fh)


REFERENCE_INTENT = {
    'plan': [{'mode': 'straight', 'guard': 'distance', 'thresh': 2.0},
             {'mode': 'turn', 'dir': 'right'}],
    'unsupported': [],
}


@pytest.mark.skipif(not (MISSIONS_DIR / 'wall_turn.json').is_file(),
                    reason='f1tenth_behavior sources not beside this package')
def test_the_reference_mission_is_reproduced_where_it_is_reproducible():
    """Hand-written wall_turn.json vs the intent that means the same thing.

    drive and stop_condition must match field for field: if the translator
    cannot reproduce the mission that actually drove the car, it is missing
    something real.
    """
    reference = _reference_wall_turn()
    produced = translate(REFERENCE_INTENT).mission
    assert len(produced['moves']) == len(reference['moves'])
    for produced_move, reference_move in zip(produced['moves'], reference['moves']):
        assert produced_move['drive'] == reference_move['drive']
        assert produced_move['stop_condition'] == reference_move['stop_condition']
        assert produced_move.get('terminal') == reference_move.get('terminal')
        assert produced_move['on_timeout'] == reference_move['on_timeout']


@pytest.mark.skipif(not (MISSIONS_DIR / 'wall_turn.json').is_file(),
                    reason='f1tenth_behavior sources not beside this package')
def test_the_reference_mission_differs_only_in_ids_and_timeouts():
    """The two places it does NOT match, recorded rather than hidden.

    ids: wall_turn.json uses human names ("runup_2m"); the translator derives
    move_{i}_{mode}, because an LLM has no business naming things the operator
    will read back.

    timeout_sec: the hand-written 30 and 60 are round numbers someone picked.
    The derived ones come from the documented formula, and for the turn the
    derived value is the better one -- the observed turn took 7.37 s, so 60 s
    would let a completely stuck turn run for the best part of a minute.
    """
    reference = _reference_wall_turn()
    produced = translate(REFERENCE_INTENT).mission
    assert [m['id'] for m in reference['moves']] == ['runup_2m', 'wall_turn']
    assert [m['id'] for m in produced['moves']] == ['move_0_straight', 'move_1_turn']
    assert [m['timeout_sec'] for m in reference['moves']] == [30, 60]
    assert [m['timeout_sec'] for m in produced['moves']] == [20, 27]


# ---------------------------------------------------------------------------
# 5. every rejection path
# ---------------------------------------------------------------------------

def test_schema_violation_raises_intent_schema_error():
    """A shape the schema forbids is a named rejection, not a KeyError."""
    with pytest.raises(IntentSchemaError):
        translate({'plan': [{'mode': 'straight', 'guard': 'wall'}], 'unsupported': []})


def test_unknown_top_level_key_raises_intent_schema_error():
    """additionalProperties: false, so an invented key is caught at the door."""
    with pytest.raises(IntentSchemaError):
        translate({'plan': [], 'unsupported': [], 'speed': 2.0})


def test_turn_phase_may_not_carry_a_threshold():
    """The model has no say over turn magnitude; a thresh on a turn is refused."""
    with pytest.raises(IntentSchemaError):
        translate({'plan': [{'mode': 'turn', 'dir': 'left', 'thresh': 1.3}],
                   'unsupported': []})


@pytest.mark.parametrize('guard,bad', [('wall', 4.5), ('wall', 0.4),
                                       ('front_object', 5.5), ('front_object', 0.4)])
def test_per_guard_range_violation_raises_intent_range_error(guard, bad):
    """The per-guard range the blanket schema range cannot express.

    Only wall and front_object appear here. The "distance" guard's per-guard
    range is 0.2-20.0, which is EXACTLY the blanket range in intent_v1.json, so
    no value exists that the schema accepts and GUARD_THRESH_RANGE rejects --
    see test_distance_out_of_range_is_caught_by_the_schema_instead.
    """
    with pytest.raises(IntentRangeError) as excinfo:
        translate({'plan': [{'mode': 'straight', 'guard': guard, 'thresh': bad}],
                   'unsupported': []})
    assert excinfo.value.guard == guard
    assert excinfo.value.value == bad
    assert excinfo.value.range == GUARD_THRESH_RANGE[guard]


@pytest.mark.parametrize('bad', [20.5, 0.1])
def test_distance_out_of_range_is_caught_by_the_schema_instead(bad):
    """Which layer rejects a bad "distance" thresh, stated rather than assumed.

    The claim "the per-guard range is enforced exactly once, in the translator"
    is true for wall and front_object and NOT for distance, whose per-guard
    range coincides with the schema's blanket one. The rejection still happens
    and is still named; it is just IntentSchemaError, not IntentRangeError.
    """
    with pytest.raises(IntentSchemaError):
        translate({'plan': [{'mode': 'straight', 'guard': 'distance', 'thresh': bad}],
                   'unsupported': []})


def test_empty_plan_raises_empty_plan_error():
    """No executable phase means no mission, not an empty one."""
    with pytest.raises(EmptyPlanError):
        translate({'plan': [], 'unsupported': ['ambiguo: distanza non specificata']})


def test_translator_output_error_carries_the_offending_document():
    """A translator bug must surface the whole document, not just a message."""
    broken = TranslatorConfig(turn_magnitude_deg=0.0)
    with pytest.raises(TranslatorOutputError) as excinfo:
        translate({'plan': [{'mode': 'turn', 'dir': 'left'}], 'unsupported': []},
                  config=broken)
    assert excinfo.value.mission['moves'][0]['drive']['turn_mag_deg'] == 0.0


# ---------------------------------------------------------------------------
# 6. unsupported gates the run
# ---------------------------------------------------------------------------

def test_unsupported_sets_requires_confirmation():
    """A partly-executable command produces a mission that must not auto-run."""
    result = translate(_golden('ex4_unsupported_jump')['intent'])
    assert result.requires_confirmation is True
    assert result.unsupported == ('saltare sopra la scatola',)


def test_empty_unsupported_does_not_require_confirmation():
    """The ordinary case stays ordinary."""
    result = translate(_golden('ex1_wall_right_distance')['intent'])
    assert result.requires_confirmation is False
    assert result.unsupported == ()


def test_requires_confirmation_does_not_leak_into_the_mission():
    """The flag belongs to the result, not to the document the stack loads."""
    result = translate(_golden('ex4_unsupported_jump')['intent'])
    assert 'requires_confirmation' not in result.mission
    assert 'unsupported' not in result.mission


# ---------------------------------------------------------------------------
# 7. timeout bounds
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('name', _golden_names())
def test_every_timeout_is_inside_the_clamp(name):
    """No emitted timeout may be born expired or effectively infinite."""
    for move in translate(_golden(name)['intent']).mission['moves']:
        assert TIMEOUT_MIN_SEC <= move['timeout_sec'] <= TIMEOUT_MAX_SEC


@pytest.mark.parametrize('guard', ['wall', 'front_object'])
def test_an_open_guard_still_gets_a_finite_timeout(guard):
    """The case the pipeline had no protection for at all.

    wall and front_object have no bounded travel: if the wall is never seen,
    nothing in the move ends it. The generated missions before this work
    carried no timeout_sec at all, so that move ran until a person intervened.
    """
    intent = {'plan': [{'mode': 'straight', 'guard': guard, 'thresh': 2.0}],
              'unsupported': []}
    timeout = translate(intent).mission['moves'][0]['timeout_sec']
    assert TIMEOUT_MIN_SEC <= timeout <= TIMEOUT_MAX_SEC
    cfg = TranslatorConfig()
    expected = math.ceil(cfg.open_guard_max_distance / cfg.speed_straight
                         * cfg.timeout_factor + cfg.timeout_floor)
    assert timeout == expected


def test_an_absurd_config_cannot_push_a_timeout_past_the_ceiling():
    """The clamp is the guarantee, not the formula."""
    cfg = TranslatorConfig(nominal_yaw_rate=1e-6)
    intent = {'plan': [{'mode': 'turn', 'dir': 'left'}], 'unsupported': []}
    assert translate(intent, config=cfg).mission['moves'][0]['timeout_sec'] == TIMEOUT_MAX_SEC


# ---------------------------------------------------------------------------
# 8. the invariant: output always validates
# ---------------------------------------------------------------------------

def _random_intent(rng):
    """A uniformly random intent that satisfies the schema AND the per-guard ranges."""
    plan = []
    for _ in range(rng.randint(1, 8)):
        if rng.random() < 0.35:
            plan.append({'mode': 'turn', 'dir': rng.choice(['left', 'right'])})
        else:
            guard = rng.choice(sorted(GUARD_THRESH_RANGE))
            low, high = GUARD_THRESH_RANGE[guard]
            plan.append({'mode': 'straight', 'guard': guard,
                         'thresh': round(rng.uniform(low, high), 2)})
    unsupported = ['x' * rng.randint(1, 20) for _ in range(rng.randint(0, 3))]
    return {'plan': plan, 'unsupported': unsupported}


def test_output_always_validates():
    """Over 400 random in-range intents, translate() never emits an invalid mission.

    translate() validates its own output through mission_config.parse_mission()
    and raises TranslatorOutputError rather than returning, so reaching the end
    of this loop without an exception IS the assertion. Seeded, so a failure is
    reproducible.
    """
    rng = random.Random(20260914)
    for _ in range(400):
        intent = _random_intent(rng)
        result = translate(intent, explain=True)
        assert result.mission['moves'][-1]['terminal'] is True
        assert sum(1 for m in result.mission['moves'] if m.get('terminal')) == 1
        ids = [m['id'] for m in result.mission['moves']]
        assert len(set(ids)) == len(ids)
        missing = [p for p in _leaf_paths(result.mission) if p not in result.provenance]
        assert missing == [], f'leaves with no provenance: {missing} for {intent}'


# ---------------------------------------------------------------------------
# config must not drift from stack_params.yaml
# ---------------------------------------------------------------------------

def test_dataclass_defaults_match_stack_params():
    """TranslatorConfig duplicates the yaml defaults; they must stay equal.

    Skipped rather than failed when stack_params is not reachable, so the rest
    of this file still runs outside a sourced workspace -- which is the whole
    reason the defaults are duplicated in the first place.
    """
    yaml = pytest.importorskip('yaml')
    candidates = [pathlib.Path(__file__).resolve().parents[3]
                  / 'f1tenth_params' / 'config' / 'stack_params.yaml']
    path = next((p for p in candidates if p.is_file()), None)
    if path is None:
        pytest.skip('stack_params.yaml not reachable from the source tree')
    with open(path, encoding='utf-8') as fh:
        params = yaml.safe_load(fh)
    cfg = TranslatorConfig()
    for field_name in cfg.__dataclass_fields__:
        key = f'mission_translator_{field_name}'
        assert key in params, f'{key} missing from stack_params.yaml'
        assert params[key]['default'] == getattr(cfg, field_name), (
            f'{key}: yaml has {params[key]["default"]!r}, '
            f'TranslatorConfig has {getattr(cfg, field_name)!r}')


def test_turn_sign_mapping_is_the_one_the_loader_validates():
    """left/right -> +1.0/-1.0, the convention mission_config states outright."""
    assert TURN_SIGN == {'left': 1.0, 'right': -1.0}


# ---------------------------------------------------------------------------
# the prompt's own examples must be things the schema and translator accept
# ---------------------------------------------------------------------------

def test_the_prompt_ships_all_five_worked_examples():
    """A silently dropped example is a silently weakened prompt."""
    assert len(intent_prompt_examples()) == 5


def test_every_prompt_example_is_a_valid_intent():
    """The examples are the model's strongest signal, so they are held to the schema.

    A worked example that the schema would reject teaches the model to produce
    rejected output, and nothing else in the pipeline would catch it -- the
    prompt is data, not code, so no import or call site ever touches it.
    """
    for command, intent in intent_prompt_examples():
        jsonschema.validate(intent, load_intent_schema()), command


def test_every_executable_prompt_example_translates():
    """And the ones with a non-empty plan must survive the whole translator."""
    for command, intent in intent_prompt_examples():
        if not intent['plan']:
            with pytest.raises(EmptyPlanError):
                translate(intent)
            continue
        result = translate(intent, explain=True)
        assert result.mission['moves'], command
        assert result.requires_confirmation == bool(intent['unsupported']), command


def test_no_prompt_example_teaches_a_turn_threshold():
    """The old prompt's `guard "turned" thresh 1.3` is what produced 74.48 degrees.

    The intent vocabulary has no turn threshold at all, so this asserts the
    replacement prompt never reintroduces one by example.
    """
    for command, intent in intent_prompt_examples():
        for phase in intent['plan']:
            if phase['mode'] == 'turn':
                assert set(phase) == {'mode', 'dir'}, command
