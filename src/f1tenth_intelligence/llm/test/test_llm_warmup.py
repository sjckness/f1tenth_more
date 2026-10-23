"""llm_warmup: waits for /health, then one planner-shaped completion (mocked HTTP)."""
from unittest.mock import patch

from llm import llm_warmup

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
