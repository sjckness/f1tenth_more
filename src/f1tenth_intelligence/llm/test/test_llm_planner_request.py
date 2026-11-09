"""Unit tests for get_plan_from_llm()'s HTTP request-building (the
llama-server backend switch) -- mocks requests.post, no live llama-server
needed. Confirms the payload shape matches llm_mpc_tuner_node.ask_llm()'s
own pattern (Step 1 finding of that switch, at the time still a real,
present file in this package -- since removed outright, see git history):
a raw prompt string posted to llama-server's /completion, n_predict/
temperature only, no chat/messages structure, no grammar/json_schema
constraint -- and that the existing list/{"plan":[...]}/fallback response
parsing is unchanged.

Also covers the raw_decode() JSON-parsing fix (fixes a live bug: the model
occasionally continues past the closing '}' with hallucinated extra
comando:/risposta: pairs, which used to raise json.JSONDecodeError: Extra
data via json.loads() even though the plan itself was valid) --
test_accepts_valid_json_with_trailing_garbage below.

Also covers the readiness/warm-up pass (fixes a live bug: a raw
ConnectionError/urllib3 traceback used to reach the operator wrapped in
"traduzione fallita", with no actionable guidance, and the still-open
cold-start item -- first /completion call against a freshly-started
llama-server observed taking ~58s against a 60s timeout): TestWaitFor
LlamaServer below covers _wait_for_llama_server()'s retry/backoff/timeout
behavior, and TestConnectionErrorHandling covers get_plan_from_llm()'s own
now-distinct ConnectionError message (server was reachable at startup but
got restarted/crashed mid-session -- a different case from "never came up",
which _wait_for_llama_server() alone handles).

The payload now also carries cache_prompt and stop (request-cost pass:
neither changes what the model sees). TestWarmPrompt covers the startup
request that loads the model and caches the real system prompt in one go,
TestStopAndCeiling how a generation ends (LLAMA_STOP, LLAMA_N_PREDICT, the
truncation warning), TestStopStringsNeverCutAValidAnswer that no known valid
answer contains a stop string, and TestPromptPrefixStability that the retry
feedback never lands before the cached prefix.

Run standalone: python3 -m pytest test/test_llm_planner_request.py -v
"""

import json
import logging
import os
from unittest.mock import MagicMock, patch

import pytest
import requests

from llm import llm_planner_node, llm_warmup, plan_translate


class _FakeResponse:
    """Just enough of requests.Response for get_plan_from_llm()'s own use
    of it (.raise_for_status(), .json())."""

    def __init__(self, content_field, status_code=200, extra=None):
        self._content_field = content_field
        self.status_code = status_code
        self.text = content_field
        self._extra = extra or {}

    def raise_for_status(self):
        pass

    def json(self):
        return dict(self._extra, content=self._content_field)


class TestRequestShape:

    # Both planner paths, because the payload assertion below is the test
    # that catches a prompt silently not being sent -- and after the v2
    # wire-in there are two prompts that could fail to be sent. Parameterised
    # rather than weakened to something both paths happen to satisfy.
    LEGACY_RESPONSE = json.dumps({
        'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0,
                  'stop_at_distance': 3.0}],
    })
    V2_RESPONSE = json.dumps({
        'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0}],
        'unsupported': [],
    })

    @pytest.mark.parametrize('path', ['legacy', 'v2'])
    @patch('llm.llm_planner_node.requests.post')
    def test_posts_raw_prompt_no_chat_structure(self, mock_post, path):
        """Raw /completion payload, and the RIGHT prompt for the selected path.

        legacy asserts exactly what it asserted before the wire-in; v2 asserts
        the payload begins with the CONTENTS OF THE PROMPT FILE, so a packaging
        break that leaves the file unread fails here rather than silently
        sending the model nothing to go on.
        """
        if path == 'legacy':
            mock_post.return_value = _FakeResponse(self.LEGACY_RESPONSE)
            expected_prompt = llm_planner_node.SYSTEM_PROMPT
            result = llm_planner_node.get_plan_from_llm('vai dritto')
            assert result == [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0,
                               'stop_at_distance': 3.0}]
        else:
            mock_post.return_value = _FakeResponse(self.V2_RESPONSE)
            expected_prompt = plan_translate.load_intent_prompt()
            result = llm_planner_node.get_intent_from_llm('vai dritto', expected_prompt)
            assert result == {'plan': [{'mode': 'straight', 'guard': 'wall',
                                        'thresh': 3.0}],
                              'unsupported': []}

        assert mock_post.call_count == 1
        args, kwargs = mock_post.call_args
        # positional URL arg, matching llm_mpc_tuner_node.ask_llm()'s own
        # requests.post(self.mpc_url, json=..., timeout=...) call shape.
        assert args[0] == llm_planner_node.LLAMA_URL

        payload = kwargs['json']
        # Raw /completion payload only -- no "messages" array, no chat
        # roles, nothing OpenAI-chat-specific. cache_prompt and stop change
        # the cost of the request, not what the model sees.
        assert set(payload.keys()) == {'prompt', 'n_predict', 'temperature',
                                       'cache_prompt', 'stop'}
        assert isinstance(payload['prompt'], str)
        assert 'messages' not in payload
        assert payload['temperature'] == pytest.approx(0.0)
        assert payload['n_predict'] == llm_planner_node.LLAMA_N_PREDICT
        assert payload['cache_prompt'] is True
        assert payload['stop'] == llm_planner_node.LLAMA_STOP

        # No chat/ChatML templating -- the prompt and the command both appear
        # verbatim, concatenated, in the same "comando: ...\nrisposta:" shape
        # the prompts' own few-shot examples use.
        assert payload['prompt'].startswith(expected_prompt)
        assert 'comando: "vai dritto"' in payload['prompt']
        assert '<|im_start|>' not in payload['prompt']

        assert kwargs['timeout'] == llm_planner_node.LLAMA_TIMEOUT

    def test_the_two_prompts_are_actually_different(self):
        """Guards the test above: identical prompts would make it prove nothing."""
        assert llm_planner_node.SYSTEM_PROMPT != plan_translate.load_intent_prompt()

    @patch('llm.llm_planner_node.requests.post')
    def test_accepts_bare_array_response(self, mock_post):
        """Fallback parsing (accepts a bare JSON array, not just {"plan":[...]})
        is unchanged from before the backend switch."""
        mock_post.return_value = _FakeResponse(json.dumps(
            [{'mode': 'straight', 'guard': 'distance', 'thresh': 1.0,
              'stop_at_distance': 1.0}]
        ))
        result = llm_planner_node.get_plan_from_llm('x')
        assert result == [{'mode': 'straight', 'guard': 'distance', 'thresh': 1.0,
                           'stop_at_distance': 1.0}]

    @patch('llm.llm_planner_node.requests.post')
    def test_raises_on_http_error(self, mock_post):
        """No internal try/except -- HTTP errors propagate to the caller,
        same as llm_mpc_tuner_node.ask_llm()'s own error handling (relies on
        raise_for_status() + the caller's own try/except)."""
        resp = MagicMock()
        resp.raise_for_status.side_effect = RuntimeError('HTTP 500')
        mock_post.return_value = resp
        with pytest.raises(RuntimeError):
            llm_planner_node.get_plan_from_llm('x')

    @patch('llm.llm_planner_node.requests.post')
    def test_raises_on_malformed_json_response(self, mock_post):
        mock_post.return_value = _FakeResponse('this is not json')
        with pytest.raises(json.JSONDecodeError):
            llm_planner_node.get_plan_from_llm('x')

    @patch('llm.llm_planner_node.requests.post')
    def test_accepts_valid_json_with_trailing_garbage(self, mock_post):
        """Reproduces the exact failure mode found earlier: the model
        continues past the closing '}' with hallucinated extra
        comando:/risposta: pairs. json.loads() would reject the whole
        response with json.JSONDecodeError: Extra data, even though the
        plan itself, up to that point, was perfectly valid -- raw_decode()
        should parse just the first valid JSON value and ignore the rest."""
        valid_plan = {
            'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0,
                      'stop_at_distance': 3.0}],
        }
        trailing_garbage = (
            '\ncomando: "gira a sinistra"\nrisposta: {"plan":[{"mode":"wall_turn"}]}'
        )
        mock_post.return_value = _FakeResponse(json.dumps(valid_plan) + trailing_garbage)

        result = llm_planner_node.get_plan_from_llm('vai dritto')

        assert result == valid_plan['plan']

    @patch('llm.llm_planner_node.requests.post')
    def test_raises_clear_message_on_connection_error(self, mock_post):
        """Distinct from test_raises_on_malformed_json_response (JSON
        parsing failure) and from test_raises_on_http_error (generic
        catch-all path): a ConnectionError here means the server was
        reachable earlier (readiness/warm-up already passed at startup, see
        TestWaitForLlamaServer below) but is no longer -- restarted or
        crashed mid-session. Gets its own clear, actionable RuntimeError
        instead of the raw ConnectionError/urllib3 traceback that used to
        reach the operator wrapped in "traduzione fallita"."""
        mock_post.side_effect = requests.exceptions.ConnectionError(
            'Connection refused')

        with pytest.raises(RuntimeError) as exc_info:
            llm_planner_node.get_plan_from_llm('vai dritto')

        message = str(exc_info.value)
        assert 'llama-server' in message
        assert 'riavviato' in message or 'crashato' in message
        # the raw requests/urllib3 message must NOT leak through verbatim --
        # that's exactly the bad failure mode being fixed here.
        assert 'Connection refused' not in message
        # exception chaining preserved (from e) -- not silently dropped.
        assert isinstance(exc_info.value.__cause__, requests.exceptions.ConnectionError)


def _system_prompt(path):
    """Return the system prompt the given planner path sends."""
    if path == 'legacy':
        return llm_planner_node.SYSTEM_PROMPT
    if path == 'no_go_to':
        return plan_translate.load_intent_prompt(
            plan_translate.INTENT_PROMPT_NO_GO_TO_FILENAME)
    return plan_translate.load_intent_prompt()


class TestPromptPrefixStability:
    """The cached prefix is the system prompt: nothing may be inserted before it."""

    @patch('llm.llm_planner_node.requests.post')
    def test_retry_feedback_keeps_system_prompt_as_exact_prefix(self, mock_post):
        """The retry feedback goes after the command, so the cache still matches."""
        mock_post.return_value = _FakeResponse(TestRequestShape.V2_RESPONSE)
        system_prompt = plan_translate.load_intent_prompt()

        llm_planner_node.get_intent_from_llm('vai dritto', system_prompt, feedback='x')

        prompt = mock_post.call_args[1]['json']['prompt']
        assert prompt.startswith(system_prompt + '\ncomando: "vai dritto')
        assert 'rifiutato: x.' in prompt[len(system_prompt):]
        assert prompt.endswith('\nrisposta:')


class TestWarmPrompt:
    """_wait_for_llama_server(warm_prompt=...): one request loads the model AND the prefix."""

    @pytest.mark.parametrize('path', ['legacy', 'v2'])
    @patch('llm.llm_planner_node.requests.post')
    def test_warm_prompt_is_posted_with_cache_prompt(self, mock_post, path):
        """Given a warm_prompt, it replaces 'ciao' and asks the server to cache it."""
        mock_post.return_value = _FakeResponse('{')
        warm = llm_planner_node._build_prompt(_system_prompt(path), 'vai dritto')

        llm_planner_node._wait_for_llama_server('http://x/completion', warm_prompt=warm)

        assert mock_post.call_count == 1
        args, kwargs = mock_post.call_args
        assert args[0] == 'http://x/completion'
        assert kwargs['json'] == {'prompt': warm, 'n_predict': 8, 'temperature': 0.0,
                                  'cache_prompt': True}
        assert kwargs['timeout'] == llm_planner_node.LLAMA_TIMEOUT

    @patch('llm.llm_planner_node.requests.post')
    def test_without_warm_prompt_the_ciao_request_is_unchanged(self, mock_post):
        """No warm_prompt: exactly the request the node always sent."""
        mock_post.return_value = _FakeResponse('{')

        llm_planner_node._wait_for_llama_server('http://x/completion')

        assert mock_post.call_args[1]['json'] == {
            'prompt': 'ciao', 'n_predict': 8, 'temperature': 0.0}

    @pytest.mark.parametrize('path', ['legacy', 'v2', 'no_go_to'])
    @patch('llm.llm_planner_node.requests.post')
    def test_warm_up_and_real_request_share_the_system_prompt_prefix(self, mock_post, path):
        """The prefix the warm-up caches is the one a real command reuses."""
        mock_post.return_value = _FakeResponse(TestRequestShape.V2_RESPONSE)
        system_prompt = _system_prompt(path)

        llm_planner_node._wait_for_llama_server(
            'http://x/completion',
            warm_prompt=llm_planner_node._build_prompt(system_prompt, 'vai dritto'))
        warm = mock_post.call_args[1]['json']['prompt']
        llm_planner_node._completion(system_prompt, 'gira a destra e vai al muro')
        real = mock_post.call_args[1]['json']['prompt']

        common = len(os.path.commonprefix([warm, real]))
        assert common >= len(system_prompt)
        # The same tail format too, up to the command itself.
        assert warm[:common].endswith('\ncomando: "')

    def test_llm_warmup_sends_the_same_prompt_as_the_planner(self):
        """llm_warmup (started by llm.launch.py) primes the exact prefix the node uses."""
        with patch('llm.llm_warmup.requests.post') as mock_post:
            mock_post.return_value = MagicMock(json=lambda: {'timings': {}})
            llm_warmup.warm_up('http://x', 'SYSTEM', timeout_s=1.0)
        payload = mock_post.call_args[1]['json']
        assert payload['prompt'] == llm_planner_node._build_prompt(
            'SYSTEM', llm_warmup.WARMUP_COMMAND)
        assert payload['cache_prompt'] is True

    def test_warm_prefix_is_gone(self):
        """The second warm-up request is folded into _wait_for_llama_server."""
        assert not hasattr(llm_planner_node, '_warm_prefix')


class TestStopAndCeiling:
    """LLAMA_STOP / LLAMA_N_PREDICT: how a generation ends, and what is done with it."""

    @patch('llm.llm_planner_node.requests.post')
    def test_stopping_word_is_put_back_before_parsing(self, mock_post):
        """llama-server cuts the stop string, '}' included: the answer still parses."""
        answer = {'plan': [], 'unsupported': ['vai dalla persona']}
        text = ' ' + json.dumps(answer, separators=(',', ':'))
        for stop in llm_planner_node.LLAMA_STOP:
            cut = text[:-1]          # what the server returns: the closing '}' is gone
            mock_post.return_value = _FakeResponse(
                cut, extra={'stop_type': 'word', 'stopping_word': stop})

            assert llm_planner_node._completion('SYSTEM', 'vai dalla persona') == answer

    @patch('llm.llm_planner_node.requests.post')
    def test_limit_logs_a_truncation_warning(self, mock_post):
        """stop_type 'limit' -> one WARNING with n_predict and the tokens generated."""
        mock_post.return_value = _FakeResponse(
            TestRequestShape.V2_RESPONSE + ' ' + TestRequestShape.V2_RESPONSE,
            extra={'stop_type': 'limit', 'timings': {'predicted_n': 1184}})
        logger = logging.getLogger('llm.llm_planner_node')
        with patch.object(logger, 'warning') as mock_warning:
            llm_planner_node._completion('SYSTEM', 'x')

        assert mock_warning.call_count == 1
        message = mock_warning.call_args[0][0]
        assert 'risposta troncata dal tetto n_predict=' in message
        assert str(llm_planner_node.LLAMA_N_PREDICT) in message
        assert '1184 token' in message

    @patch('llm.llm_planner_node.requests.post')
    def test_truncated_json_still_raises_with_llm_raw(self, mock_post):
        """A cut-off answer is not repaired: same JSONDecodeError, llm_raw attached."""
        mock_post.return_value = _FakeResponse(
            '{"plan":[{"mode":"go_to","target":"pers',
            extra={'stop_type': 'limit', 'timings': {'predicted_n': 1184}})
        logger = logging.getLogger('llm.llm_planner_node')
        with patch.object(logger, 'warning'):
            with pytest.raises(json.JSONDecodeError) as exc_info:
                llm_planner_node._completion('SYSTEM', 'x')
        assert exc_info.value.llm_raw == '{"plan":[{"mode":"go_to","target":"pers'

    @patch('llm.llm_planner_node.requests.post')
    def test_eos_and_word_do_not_warn(self, mock_post):
        """Only 'limit' is a truncation."""
        logger = logging.getLogger('llm.llm_planner_node')
        for extra in ({'stop_type': 'eos'}, {'stop_type': 'word', 'stopping_word': '} '}):
            mock_post.return_value = _FakeResponse(TestRequestShape.V2_RESPONSE, extra=extra)
            with patch.object(logger, 'warning') as mock_warning:
                llm_planner_node._completion('SYSTEM', 'x')
            assert mock_warning.call_count == 0

    def test_n_predict_is_a_multiple_of_32_above_the_worst_case(self):
        """789 tokens measured for the schema's longest answer (see LLAMA_N_PREDICT)."""
        assert llm_planner_node.LLAMA_N_PREDICT % 32 == 0
        assert llm_planner_node.LLAMA_N_PREDICT >= 789 * 1.5


def _known_answers():
    """Yield (source, intent) for every answer this package knows to be valid."""
    here = os.path.dirname(os.path.abspath(__file__))
    golden_dir = os.path.join(here, 'golden')
    for name in sorted(os.listdir(golden_dir)):
        if name.endswith('.json'):
            with open(os.path.join(golden_dir, name), encoding='utf-8') as fh:
                yield f'golden/{name}', json.load(fh)['intent']
    import test_intent_evals_live as evals
    for cases in (evals.CASES, evals.NO_GO_TO_CASES):
        for command, expected, _why in cases:
            if expected == evals.REFUSED:
                yield f'eval {command!r}', {'plan': [], 'unsupported': [command]}
            else:
                yield f'eval {command!r}', {'plan': expected, 'unsupported': []}
    prompts = {
        'legacy': llm_planner_node.SYSTEM_PROMPT,
        'v2': plan_translate.load_intent_prompt(),
        'no_go_to': plan_translate.load_intent_prompt(
            plan_translate.INTENT_PROMPT_NO_GO_TO_FILENAME),
    }
    for prompt_name, text in prompts.items():
        for line in text.splitlines():
            line = line[len('risposta:'):].strip() if line.startswith('risposta:') else line
            if line.startswith('{'):
                yield f'{prompt_name} prompt example', json.loads(line)


class TestStopStringsNeverCutAValidAnswer:
    """No answer this package knows to be valid contains a LLAMA_STOP string."""

    def test_known_answers_exist(self):
        """Guards the test below: an empty corpus would prove nothing."""
        sources = {s.split(' ')[0] for s, _ in _known_answers()}
        assert {'legacy', 'v2', 'no_go_to', 'eval'} <= sources
        assert any(s.startswith('golden/') for s in sources)

    def test_no_known_answer_contains_a_stop_string(self):
        """Compact (what the model writes) and spaced (json.dumps default) forms."""
        for source, intent in _known_answers():
            for text in (json.dumps(intent, separators=(',', ':'), ensure_ascii=False),
                         json.dumps(intent, ensure_ascii=False)):
                assert '\n' not in text, source
                for stop in llm_planner_node.LLAMA_STOP:
                    assert stop not in text, (source, stop, text)


class TestWaitForLlamaServer:
    """_wait_for_llama_server() -- the startup readiness+warm-up check
    (readiness/warm-up pass, see module docstring). Mocks requests.post and
    time.sleep, no live llama-server or real waiting needed."""

    @patch('llm.llm_planner_node.requests.post')
    def test_succeeds_on_first_attempt(self, mock_post):
        mock_post.return_value = _FakeResponse('{"plan":[]}')

        elapsed = llm_planner_node._wait_for_llama_server('http://x/completion')

        assert mock_post.call_count == 1
        # the warm-up request itself is a real /completion call, not a bare
        # ping -- same payload shape get_plan_from_llm() uses (prompt/
        # n_predict/temperature), just a trivial prompt and small n_predict.
        args, kwargs = mock_post.call_args
        assert args[0] == 'http://x/completion'
        assert set(kwargs['json'].keys()) == {'prompt', 'n_predict', 'temperature'}
        assert kwargs['json']['n_predict'] <= 8
        assert isinstance(elapsed, float)
        assert elapsed >= 0.0

    @patch('llm.llm_planner_node.time.sleep')
    @patch('llm.llm_planner_node.requests.post')
    def test_succeeds_after_n_connection_errors(self, mock_post, mock_sleep):
        """Confirms the retry/backoff loop actually retries: two simulated
        ConnectionErrors (server process not listening yet) followed by a
        successful attempt."""
        mock_post.side_effect = [
            requests.exceptions.ConnectionError('refused'),
            requests.exceptions.ConnectionError('refused'),
            _FakeResponse('{"plan":[]}'),
        ]

        elapsed = llm_planner_node._wait_for_llama_server(
            'http://x/completion', max_wait_s=10.0)

        assert mock_post.call_count == 3
        assert mock_sleep.call_count == 2  # backed off twice before succeeding
        assert isinstance(elapsed, float)

    @patch('llm.llm_planner_node.time.sleep')
    @patch('llm.llm_planner_node.requests.post')
    def test_retries_while_server_answers_503_loading_model(self, mock_post, mock_sleep):
        """llama-server answers 503 while the model loads (measured on the Thor
        between the port opening and /health 200): retried like a refused
        connection, not raised as an HTTP error."""
        mock_post.side_effect = [
            _FakeResponse('{"error":"Loading model"}', status_code=503),
            _FakeResponse('{"plan":[]}'),
        ]

        elapsed = llm_planner_node._wait_for_llama_server(
            'http://x/completion', max_wait_s=10.0)

        assert mock_post.call_count == 2
        assert mock_sleep.call_count == 1
        assert isinstance(elapsed, float)

    @patch('llm.llm_planner_node.time.sleep')
    @patch('llm.llm_planner_node.time.time')
    @patch('llm.llm_planner_node.requests.post')
    def test_raises_clear_error_when_max_wait_exhausted(
            self, mock_post, mock_time, mock_sleep):
        """max_wait_s exhausted -> LlamaServerUnreachableError (not a raw
        requests exception), with an actionable message telling the
        operator exactly what to do."""
        mock_post.side_effect = requests.exceptions.ConnectionError('refused')
        # t0=100.0, then the post-attempt elapsed check reads 200.0 --
        # 100s elapsed >= max_wait_s=5.0, so it gives up after one attempt.
        mock_time.side_effect = [100.0, 200.0]

        with pytest.raises(llm_planner_node.LlamaServerUnreachableError) as exc_info:
            llm_planner_node._wait_for_llama_server(
                'http://x/completion', max_wait_s=5.0)

        assert mock_post.call_count == 1
        message = str(exc_info.value)
        assert 'http://x/completion' in message
        assert 'ros2 launch llm llm.launch.py' in message


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
