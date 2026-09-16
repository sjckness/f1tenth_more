"""Evals from operator utterances, against the live model.

Every other test in this package starts from a hand-written intent. These
start from what an operator actually types, which is the only level at which
a *prompt* change can be verified at all.

The failure that matters most -- answering a named object with
``{"mode":"straight","guard":"front_object","thresh":1.0}`` -- produces a
perfectly valid intent that translates, loads, drives, and reports success at
whatever happened to be nearest. Nothing downstream of the model knows the
utterance named a specific thing, so the system prompt is the only defence,
and it can only be tested by running a model.

Each command is planned the way LLMPlannerNode._plan_v2 plans it: the
translator's rejection text is fed back for up to MAX_INTENT_RETRIES extra
attempts, and the attempt count is part of the result. The accepted intent
is then judged against the command's expectation.

Running these:
    llama-server up (see llm.launch.py)  ->  pytest -m llm
    otherwise                            ->  skipped, with a reason

Sampling is temperature 0.0, so a pass here is reproducible rather than lucky.
docs/analysis/go_to_live_eval.py runs the same cases and prints the table.
"""

import socket
import urllib.parse

import pytest

from llm import llm_planner_node
from llm.llm_planner_node import MAX_INTENT_RETRIES, get_intent_from_llm
from llm.plan_translate import (
    INTENT_PROMPT_NO_GO_TO_FILENAME,
    EmptyPlanError,
    IntentRangeError,
    IntentSchemaError,
    load_intent_prompt,
    translate,
)

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


requires_server = pytest.mark.skipif(
    not _server_is_up(),
    reason=f'no llama-server on {llm_planner_node.LLAMA_URL}; run with -m llm '
           'once one is up')


def go_to(target, thresh=None):
    phase = {'mode': 'go_to', 'target': target}
    if thresh is not None:
        phase['thresh'] = thresh
    return phase


def straight(guard, thresh):
    return {'mode': 'straight', 'guard': guard, 'thresh': thresh}


def turn(direction):
    return {'mode': 'turn', 'dir': direction}


REFUSED = 'refused'

# (command, expected plan or REFUSED, why). A plan must match exactly and
# carry no `unsupported`; REFUSED means an empty plan with the request in
# `unsupported`, and in particular no go_to and no front_object/distance guess.
CASES = [
    ('vai dalla persona', [go_to('person')], 'class, no distance'),
    ('raggiungi la sedia e fermati a un metro', [go_to('chair', 1.0)], 'class, distance'),
    ('vai verso la bottiglia e poi gira a destra', [go_to('bottle'), turn('right')],
     'multi-phase'),
    ('fermati a dieci centimetri dalla persona', [go_to('person', 0.1)],
     'distance below gap_min, written as said'),
    ('vai dalla porta', REFUSED, 'not a class'),
    ('vai da Marco', REFUSED, 'an individual'),
    ("fermati prima dell'ostacolo", [straight('front_object', 1.0)], 'generic guard'),
    ('avvicinati al divano', [go_to('couch')], 'Italian -> COCO'),
    ('vai verso lo zaino e fermati a mezzo metro', [go_to('backpack', 0.5)], 'Italian -> COCO'),
    ('vai al tavolo', [go_to('dining table')], 'two-word class'),
    ('vai dalla persona a sinistra', REFUSED, 'selection'),
    ('raggiungi la seconda sedia', REFUSED, 'selection'),
    ('vai verso la finestra', REFUSED, 'not a class'),
    ('vai dalla mia collega', REFUSED, 'an individual'),
    ('vai verso il portatile', [go_to('laptop')], 'class not in the examples'),
    ('gira a sinistra e poi vai dalla sedia', [turn('left'), go_to('chair')], 'multi-phase'),
    ('vai dritto e fermati a due metri dal muro', [straight('wall', 2.0)],
     'wall stays a guard'),
    ('vai dritto, al muro gira a destra e avanza 2 metri',
     [straight('wall', 3.0), turn('right'), straight('distance', 2.0)], 'unchanged'),
    ('vai avanti e fermati prima di quello che trovi', [straight('front_object', 1.0)],
     'generic guard'),
    ('vai dalla bottiglia e fermati a due metri', [go_to('bottle', 2.0)], 'class, distance'),
]

# The go_to_enabled=False prompt: named objects refused, go_to never authored.
NO_GO_TO_CASES = [
    ('vai dalla persona', REFUSED, 'no go_to in this prompt'),
    ('vai verso la bottiglia e fermati a 1 metro', REFUSED, 'no go_to in this prompt'),
    ("vai avanti e fermati prima dell'ostacolo", [straight('front_object', 1.0)],
     'generic guard'),
]


def plan_like_the_node(prompt, command, go_to_enabled=True):
    """Return (intent, attempts, error) the way _plan_v2 reaches its answer."""
    feedback = None
    intent = None
    for attempt in range(1, MAX_INTENT_RETRIES + 2):
        try:
            intent = get_intent_from_llm(command, prompt, feedback)
        except Exception as exc:  # the node gives up on these without retrying
            return intent, attempt, f'generation failed: {exc}'
        try:
            translate(intent, go_to_enabled=go_to_enabled)
        except EmptyPlanError:
            return intent, attempt, None
        except (IntentSchemaError, IntentRangeError) as exc:
            feedback = str(exc)
            continue
        return intent, attempt, None
    return intent, MAX_INTENT_RETRIES + 1, f'retries exhausted: {feedback}'


def judge(intent, expected):
    """Return None when the intent meets the expectation, else the reason."""
    if intent is None:
        return 'no intent'
    plan = intent.get('plan', [])
    if expected == REFUSED:
        if plan:
            return f'expected a refusal, got plan {plan}'
        if not intent.get('unsupported'):
            return 'empty plan without an unsupported entry'
        return None
    if intent.get('unsupported'):
        return f'unexpected unsupported {intent["unsupported"]}'
    if plan != expected:
        return f'plan {plan} != expected {expected}'
    return None


def run_case(prompt, command, expected, go_to_enabled=True):
    intent, attempts, error = plan_like_the_node(prompt, command, go_to_enabled)
    reason = error or judge(intent, expected)
    return {'command': command, 'intent': intent, 'attempts': attempts,
            'retries': attempts - 1, 'passed': reason is None, 'reason': reason}


@pytest.fixture(scope='module')
def prompt():
    return load_intent_prompt()


@pytest.fixture(scope='module')
def no_go_to_prompt():
    return load_intent_prompt(INTENT_PROMPT_NO_GO_TO_FILENAME)


def test_there_are_at_least_fifteen_italian_commands():
    assert len(CASES) >= 15


@requires_server
@pytest.mark.parametrize('command, expected, why', CASES, ids=[c[0] for c in CASES])
def test_go_to_prompt(prompt, command, expected, why):
    result = run_case(prompt, command, expected)
    assert result['passed'], (why, result)


@requires_server
@pytest.mark.parametrize('command, expected, why', NO_GO_TO_CASES,
                         ids=[c[0] for c in NO_GO_TO_CASES])
def test_no_go_to_prompt(no_go_to_prompt, command, expected, why):
    result = run_case(no_go_to_prompt, command, expected, go_to_enabled=False)
    assert result['passed'], (why, result)
