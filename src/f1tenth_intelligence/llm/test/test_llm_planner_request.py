"""Unit tests for get_plan_from_llm()'s HTTP request-building (the
llama-server backend switch) -- mocks requests.post, no live llama-server
needed. Confirms the payload shape matches llm_mpc_tuner_node.ask_llm()'s
own pattern (Step 1 finding of that switch): a raw prompt string posted to
llama-server's /completion, n_predict/temperature only, no chat/messages
structure, no grammar/json_schema constraint -- and that the existing
list/{"plan":[...]}/fallback response parsing is unchanged.

Run standalone: python3 -m pytest test/test_llm_planner_request.py -v
"""

import json
from unittest.mock import MagicMock, patch

import pytest

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


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
