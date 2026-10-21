"""
A fake llama-server for the LLM2 tests: a /completion endpoint on 127.0.0.1.

Fixtures:
  fake_server(answer=None, delay_s=0.0) -> FakeLlamaServer, closed after the test.
      answer(body) -> (http_status, reply dict); None answers a perfect echo.
  echo_answer -> the perfect-echo answer function, to compose other answers.
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading

from llm.llm2_protocol import PROMPT_PREFIX
import pytest

#: llama-server's timings in every echo reply (plus one key the node ignores).
ECHO_TIMINGS = {'prompt_n': 1, 'prompt_ms': 12.5, 'predicted_n': 82, 'predicted_ms': 880.0,
                'predicted_per_second': 93.2}


def _echo_answer(body):
    """Answer like llama-server for a perfect echo: the prompt's JSON back."""
    prompt = body['prompt']
    sent = prompt[len(PROMPT_PREFIX):prompt.index('<|im_end|>', len(PROMPT_PREFIX))]
    return 200, {'content': sent, 'timings': dict(ECHO_TIMINGS), 'tokens_cached': 214}


class FakeLlamaServer:
    """
    A /completion endpoint on 127.0.0.1 that answers the way it is told.

    Every request body is kept in `bodies`; `max_active` is the most requests
    it ever had open at once. A delayed reply returns at once when the server
    is closed.
    """

    def __init__(self, answer, delay_s):
        self.answer = answer
        self.delay_s = delay_s
        self.bodies = []
        self.active = 0
        self.max_active = 0
        self.release = threading.Event()
        self._lock = threading.Lock()
        fake = self

        class Handler(BaseHTTPRequestHandler):

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                with fake._lock:
                    fake.bodies.append(body)
                    fake.active += 1
                    fake.max_active = max(fake.max_active, fake.active)
                try:
                    fake.release.wait(fake.delay_s)
                    status, reply = fake.answer(body)
                    data = json.dumps(reply).encode()
                    self.send_response(status)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError):
                    pass    # the client timed out and hung up
                finally:
                    with fake._lock:
                        fake.active -= 1

            def log_message(self, *args):
                pass

        self._server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self._server.daemon_threads = True
        self.url = f'http://127.0.0.1:{self._server.server_address[1]}'
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

    def close(self):
        self.release.set()
        self._server.shutdown()
        self._server.server_close()


@pytest.fixture
def echo_answer():
    return _echo_answer


@pytest.fixture
def fake_server():
    servers = []

    def start(answer=None, delay_s=0.0):
        server = FakeLlamaServer(answer or _echo_answer, delay_s)
        servers.append(server)
        return server

    yield start
    for server in servers:
        server.close()
