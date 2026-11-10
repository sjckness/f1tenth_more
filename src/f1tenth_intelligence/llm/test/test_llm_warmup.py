"""llm_warmup: waits for /health, then one planner-shaped completion (mocked HTTP)."""
from unittest.mock import patch

from llm import llm_warmup
from llm.plan_translate import INTENT_PROMPT_FILENAME, INTENT_PROMPT_NO_GO_TO_FILENAME

import pytest

import requests


class _Resp:
    def __init__(self, status_code=200, payload=None):
        self.status_code = status_code
        self._payload = payload or {}

    def raise_for_status(self):
        if self.status_code >= 400:
            raise requests.exceptions.HTTPError(str(self.status_code))

    def json(self):
        return self._payload


@patch('llm.llm_warmup.time.sleep')
@patch('llm.llm_warmup.requests.get')
def test_wait_healthy_retries_refused_and_503(mock_get, _sleep):
    """Refused connection and 503 (model loading) are waited out until 200."""
    mock_get.side_effect = [requests.exceptions.ConnectionError('refused'), _Resp(503), _Resp(200)]
    assert llm_warmup.wait_healthy('http://x', max_wait_s=60.0) >= 0.0
    assert mock_get.call_count == 3
    assert mock_get.call_args[0][0] == 'http://x/health'


@patch('llm.llm_warmup.requests.post')
def test_warm_up_sends_the_planner_request_shape(mock_post):
    """Same raw-prompt request as the planner, n_predict 1."""
    mock_post.return_value = _Resp(200, {'timings': {'prompt_n': 2443}})
    t = llm_warmup.warm_up('http://x', 'SYSTEM', timeout_s=10.0)
    args, kwargs = mock_post.call_args
    assert args[0] == 'http://x/completion'
    # same raw-prompt contract as llm_planner_node._completion, only n_predict differs
    assert set(kwargs['json']) == {'prompt', 'n_predict', 'temperature'}
    assert kwargs['json']['prompt'].startswith('SYSTEM\ncomando: "')
    assert kwargs['json']['prompt'].endswith('"\nrisposta:')
    assert kwargs['json']['n_predict'] == 1 and kwargs['json']['temperature'] == 0.0
    assert t['prompt_n'] == 2443 and 'wall_s' in t


@patch('llm.llm_warmup.wait_healthy', side_effect=TimeoutError('not ready'))
def test_main_returns_1_when_server_never_ready(_wait):
    """A server that never gets ready is reported with exit code 1."""
    assert llm_warmup.main(['--url', 'http://x/completion', '--max-wait', '1']) == 1


@pytest.mark.parametrize('go_to, doors, expect_file, expect_doors', [
    (True, True, INTENT_PROMPT_FILENAME, True),
    (True, False, INTENT_PROMPT_FILENAME, False),
    # go_to off: the prompt without go_to has no door section to render at
    # all, so asking for one would raise IntentPromptError -- which is what
    # this used to do, because load_intent_prompt defaults to doors_enabled.
    (False, True, INTENT_PROMPT_NO_GO_TO_FILENAME, False),
    (False, False, INTENT_PROMPT_NO_GO_TO_FILENAME, False),
])
def test_the_warm_up_renders_the_prompt_the_planner_will_send(
        monkeypatch, go_to, doors, expect_file, expect_doors):
    """Byte-identical to the planner's, or the prefix cache buys nothing.

    llama-server caches a prompt prefix per slot: a warm-up that renders the
    door section while the planner renders the refusal one pays the full
    prompt_ms twice and the warm-up has bought nothing. Both sides must
    resolve go_to_enabled AND enter_door_enabled the same way.
    """
    from llm import llm_warmup
    from llm.plan_translate import load_intent_prompt
    monkeypatch.setattr(llm_warmup, 'go_to_enabled', lambda: go_to)
    monkeypatch.setattr(llm_warmup, 'enter_door_enabled', lambda: doors)
    prompt_file, rendered_doors, prompt = llm_warmup.planner_prompt()
    assert prompt_file == expect_file
    assert rendered_doors is expect_doors
    assert prompt == load_intent_prompt(expect_file, doors_enabled=expect_doors)
    assert '{{PORTE}}' not in prompt
    assert ('enter_door' in prompt) is expect_doors
