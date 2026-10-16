#!/usr/bin/env python3
"""Warm llama-server up when the intelligence component starts.

llm.launch.py starts this next to llama-server (start_server:=true). It waits
for the server's /health to answer 200 (503 while the model loads, connection
refused before the port is open), then sends ONE /completion with the v2
planner system prompt and a dummy command, n_predict 1. That request pays the
first-request cost (model pages, GPU warm-up: up to 58 s observed on a freshly
booted Thor) and leaves the prompt prefix in the server's slot cache, so the
operator's first `ros2 run llm llm_planner_node "..."` gets the same ~2 s
answer as every later one instead of racing LLAMA_TIMEOUT.

The request is the planner's own shape (raw prompt, temperature 0.0, see
llm_planner_node._completion); only n_predict differs. It never touches the
mission services. Exit code 0 when warm, 1 when the server did not get ready
within --max-wait (the server itself is left alone either way).

    ros2 run llm llm_warmup [--url http://127.0.0.1:8083] [--max-wait 300]
"""

import argparse
import sys
import time

from llm.plan_translate import intent_prompt_filename, load_intent_prompt

import requests

DEFAULT_URL = 'http://127.0.0.1:8083'
WARMUP_COMMAND = 'vai dritto'


def _log(msg: str) -> None:
    print(f'[llm_warmup] {msg}', flush=True)


def wait_healthy(base_url: str, max_wait_s: float, poll_s: float = 0.5) -> float:
    """Seconds until GET /health answers 200; raises TimeoutError after max_wait_s."""
    t0 = time.monotonic()
    last = None
    while True:
        try:
            code = requests.get(f'{base_url}/health', timeout=5.0).status_code
        except requests.exceptions.RequestException as exc:
            code = type(exc).__name__
        if code != last:
            _log(f'/health -> {code} after {time.monotonic() - t0:.1f} s')
            last = code
        if code == 200:
            return time.monotonic() - t0
        if time.monotonic() - t0 >= max_wait_s:
            raise TimeoutError(
                f'{base_url}/health not 200 after {max_wait_s:.0f} s (last: {code})')
        time.sleep(poll_s)


def warm_up(base_url: str, system_prompt: str, timeout_s: float) -> dict:
    """One planner-shaped /completion (n_predict 1); returns the server's timings."""
    prompt = system_prompt + f'\ncomando: "{WARMUP_COMMAND}"\nrisposta:'
    t0 = time.monotonic()
    r = requests.post(f'{base_url}/completion',
                      json={'prompt': prompt, 'n_predict': 1, 'temperature': 0.0},
                      timeout=timeout_s)
    r.raise_for_status()
    timings = r.json().get('timings', {}) or {}
    timings['wall_s'] = time.monotonic() - t0
    return timings


def go_to_enabled() -> bool:
    """stack_params go_to_enabled: picks the same prompt file as the planner."""
    try:
        from f1tenth_params.param_defaults import get_value
        return bool(get_value('go_to_enabled'))
    except Exception:  # noqa: BLE001 - no params package: the default prompt
        return True


def main(argv=None) -> int:
    """Wait for the server, warm it up; 0 when warm, 1 otherwise."""
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--url', default=DEFAULT_URL, help='llama-server base URL (no /completion)')
    p.add_argument('--max-wait', type=float, default=300.0, help='seconds to wait for /health')
    p.add_argument('--timeout', type=float, default=180.0,
                   help='seconds for the warm-up completion')
    a, _ros = p.parse_known_args(argv)
    base = a.url.rstrip('/')
    if base.endswith('/completion'):
        base = base[:-len('/completion')]
    try:
        ready = wait_healthy(base, a.max_wait)
        prompt_file = intent_prompt_filename(go_to_enabled())
        t = warm_up(base, load_intent_prompt(prompt_file), a.timeout)
    except (TimeoutError, requests.exceptions.RequestException) as exc:
        _log(f'FAILED: {exc}')
        return 1
    _log(f'ready after {ready:.1f} s; warm-up completion ({prompt_file}) {t["wall_s"]:.2f} s, '
         f'prompt_n={t.get("prompt_n")} prompt_ms={t.get("prompt_ms")} cache_n={t.get("cache_n")}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
