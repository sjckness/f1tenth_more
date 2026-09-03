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

Run standalone: python3 -m pytest test/test_llm_planner_request.py -v
"""

import json
from unittest.mock import MagicMock, patch

import pytest
import requests

from llm import llm_planner_node


class _FakeResponse:
    """Just enough of requests.Response for get_plan_from_llm()'s own use
    of it (.raise_for_status(), .json())."""

    def __init__(self, content_field):
        self._content_field = content_field

    def raise_for_status(self):
        pass

    def json(self):
        return {'content': self._content_field}


class TestRequestShape:

    @patch('llm.llm_planner_node.requests.post')
    def test_posts_raw_prompt_no_chat_structure(self, mock_post):
        mock_post.return_value = _FakeResponse(json.dumps({
            'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0,
                      'stop_at_distance': 3.0}],
        }))

        result = llm_planner_node.get_plan_from_llm('vai dritto')

        assert mock_post.call_count == 1
        args, kwargs = mock_post.call_args
        # positional URL arg, matching llm_mpc_tuner_node.ask_llm()'s own
        # requests.post(self.mpc_url, json=..., timeout=...) call shape.
        assert args[0] == llm_planner_node.LLAMA_URL

        payload = kwargs['json']
        # Raw /completion payload only -- no "messages" array, no chat
        # roles, nothing OpenAI-chat-specific.
        assert set(payload.keys()) == {'prompt', 'n_predict', 'temperature'}
        assert isinstance(payload['prompt'], str)
        assert 'messages' not in payload
        assert payload['temperature'] == pytest.approx(0.0)

        # No chat/ChatML templating -- SYSTEM_PROMPT and the command both
        # appear verbatim, concatenated, in the same "comando: ...\nrisposta:"
        # shape SYSTEM_PROMPT's own few-shot examples already use.
        assert payload['prompt'].startswith(llm_planner_node.SYSTEM_PROMPT)
        assert 'comando: "vai dritto"' in payload['prompt']
        assert '<|im_start|>' not in payload['prompt']

        assert kwargs['timeout'] == llm_planner_node.LLAMA_TIMEOUT

        assert result == [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0,
                           'stop_at_distance': 3.0}]

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
