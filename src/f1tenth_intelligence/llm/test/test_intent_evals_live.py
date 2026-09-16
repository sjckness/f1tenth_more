"""Evals from operator utterances, against the live model.

Every other test in this package starts from a hand-written intent. These
start from what an operator actually types, which is the only level at which
a *prompt* change can be verified at all.

That distinction matters more than it looks. The failure this suite exists to
catch -- answering "vai verso la bottiglia" with
``{"mode":"straight","guard":"front_object","thresh":1.0}`` -- produces a
perfectly valid intent. It passes the schema, it translates, it loads, it
drives, and it reports success at whatever happened to be nearest. **No
amount of code can catch it**, because nothing downstream of the model knows
the utterance named a specific object. The system prompt is the only defence,
so the prompt is what has to be tested, and it can only be tested by running
a model.

Running these:
    llama-server up (see llm.launch.py)  ->  pytest -m llm
    otherwise                            ->  skipped, with a reason

In CI they run only where a server is provisioned; `pytest -m "not llm"` is
the default elsewhere, which is why the offline suite in
test_intent_go_to.py covers the schema and translator halves separately.
Sampling is temperature 0.0, so a pass here is reproducible rather than lucky.
"""

import socket
import urllib.parse

import pytest

from llm import llm_planner_node
from llm.llm_planner_node import get_intent_from_llm, load_intent_prompt

pytestmark = pytest.mark.llm


def _server_is_up() -> bool:
    try:
        parsed = urllib.parse.urlparse(llm_planner_node.LLAMA_URL)
        sock = socket.socket()
        sock.settimeout(1.0)
        reachable = sock.connect_ex((parsed.hostname, parsed.port)) == 0
        sock.close()
        return reachable
    except Exception:
        return False


pytest.mark.skipif  # noqa: B018 - referenced for readers of the decorator below
requires_server = pytest.mark.skipif(
    not _server_is_up(),
    reason=f'no llama-server on {llm_planner_node.LLAMA_URL}; run with -m llm '
           'once one is up')


@pytest.fixture(scope='module')
def prompt():
    return load_intent_prompt()


def plan_for(prompt, utterance):
    return get_intent_from_llm(utterance, prompt)


def guards(intent):
    return [p.get('guard') for p in intent['plan'] if p['mode'] == 'straight']


def assert_refused(intent, utterance):
    """The request must be declined, and declined for the right reason."""
    assert intent['unsupported'], (
        f'{utterance!r} produced no unsupported entry: {intent}')
    # The assertion that actually matters. Checking only `unsupported` would
    # pass for the wrong reason -- a model that refused everything, or that
    # refused *and* also emitted the substitution, would both slip through.
    assert 'front_object' not in guards(intent), (
        f'{utterance!r} fell back to the wrong-object substitution: {intent}')
    assert 'distance' not in guards(intent), (
        f'{utterance!r} guessed a distance threshold: {intent}')


ITALIAN_OBJECT = [
    'vai verso la bottiglia e fermati a 1 metro',
    'vai verso la bottiglia',
    'segui la sedia',
    'portati davanti al tavolo',
]
ITALIAN_PERSON = ['avvicinati alla persona']
ENGLISH_OBJECT = ['go to the bottle', 'approach the chair']
ENGLISH_PERSON = ['drive toward the person', 'follow me']


@requires_server
@pytest.mark.parametrize('utterance', ITALIAN_OBJECT)
def test_italian_named_object_requests_are_refused(prompt, utterance):
    assert_refused(plan_for(prompt, utterance), utterance)


@requires_server
@pytest.mark.parametrize('utterance', ITALIAN_PERSON + ENGLISH_PERSON)
def test_named_person_requests_are_refused(prompt, utterance):
    """Phrased differently from object targets, so covered separately."""
    assert_refused(plan_for(prompt, utterance), utterance)


@requires_server
@pytest.mark.parametrize('utterance', ENGLISH_OBJECT)
def test_english_named_object_requests_are_refused(prompt, utterance):
    assert_refused(plan_for(prompt, utterance), utterance)


@requires_server
@pytest.mark.parametrize('utterance', [
    "vai dritto e fermati prima dell'ostacolo",
    'vai avanti e fermati prima di quello che trovi',
])
def test_an_unnamed_obstacle_still_plans(prompt, utterance):
    """Over-refusal guard.

    A prompt that fixed the substitution by refusing everything would be a
    regression, and the refusal tests above could not tell the difference.
    The line is whether the utterance names a specific thing.
    """
    intent = plan_for(prompt, utterance)

    assert intent['plan'], f'{utterance!r} should still be planned: {intent}'
    assert 'front_object' in guards(intent), (
        f'{utterance!r} is exactly what front_object is for: {intent}')
    assert not intent['unsupported'], f'nothing to refuse here: {intent}'


@requires_server
def test_a_compound_request_is_planned_in_part_and_refused_in_part(prompt):
    """Documented decision: PARTIAL, never silent.

    "gira a destra poi vai verso la bottiglia" has an expressible half. The
    prompt's existing AZIONI NON ESPRIMIBILI rule already says to emit the
    executable part and list the rest, and ex4 (the jump example) has always
    behaved that way, so refusing the whole thing would be the inconsistent
    choice.

    Partial is only defensible because it cannot be silent: a non-empty
    `unsupported` forces an explicit operator confirmation in
    process_command() before anything is loaded, whether or not --confirm was
    passed. That gate is what makes this safe, not the model's judgement.
    """
    utterance = 'gira a destra poi vai verso la bottiglia'
    intent = plan_for(prompt, utterance)

    assert intent['unsupported'], f'the approach half must be declined: {intent}'
    assert 'front_object' not in guards(intent), (
        f'and must not become the substitution: {intent}')

    turns = [p for p in intent['plan'] if p['mode'] == 'turn']
    assert turns and turns[0]['dir'] == 'right', (
        f'the expressible half should survive: {intent}')


@requires_server
def test_the_model_never_authors_go_to(prompt):
    """go_to is in the schema and deliberately not in the prompt.

    If this ever fails, the model invented the mode from somewhere, and the
    translator's UnsupportedIntentModeError is the net that catches it.
    """
    for utterance in ITALIAN_OBJECT[:2] + ENGLISH_OBJECT[:1]:
        intent = plan_for(prompt, utterance)
        assert all(p['mode'] != 'go_to' for p in intent['plan']), intent
