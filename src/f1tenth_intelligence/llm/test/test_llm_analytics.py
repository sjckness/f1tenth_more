"""Offline tests for llm_analytics: the recording proxy and the analyser.

Nothing here needs llama-server, ROS or the network beyond a fake upstream
bound to 127.0.0.1:0 -- including the end-to-end proxy test, which is the one
that matters most: it asserts that the bytes the node would have sent come out
the other side unchanged AND land in the event log with the prompt, timings and
raw output the analyser is supposed to report.

The airlock tests use the real plan_translate.translate() against the committed
golden fixture. translate() validates through f1tenth_behavior's REAL
mission_config, so f1tenth_behavior must be importable: from a sourced workspace
it already is. They skip themselves only when llm.plan_translate itself cannot
be imported -- a missing f1tenth_behavior is NOT a skip, it is a failure.

Run standalone, from src/f1tenth_intelligence/llm:

    PYTHONPATH=$PWD:$PWD/../../f1tenth_behavior \
        python3 -m pytest test/test_llm_analytics.py -v

83 tests. Run without f1tenth_behavior on the path and 81 pass: the two airlock
verdicts (TestAirlock.test_accepts_an_intent_the_golden_fixture_says_translates
and TestExecFile.test_the_airlock_verdict_lands_in_the_log) report
"TranslatorOutputError: ... f1tenth_behavior is not importable", which is an
import-path problem, not a defect in llm_analytics or in the translator.

TestExecFile is the --exec-file batch: it asserts the request is the node's
(n_predict, temperature, schema, no stop list) and that a sweep records every
line, keeps going after a malformed answer or a dead upstream, and reports the
exit code. It needs the shipped prompts, so it skips only if llm.plan_translate
itself cannot be imported.

TestInputSnapshot is the copy the tool keeps of its own input: --exec-file lands
beside --events byte for byte (comments and line endings intact, so the copy can
be fed back to the same flag), --exec lands one command per line in the format
load_command_file() reads, both are named in report.md with a count and a
sha256, and nothing is written when there is no folder (--no-events) or no input
(idle recorder). One test runs the whole path through main().
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hashlib
import json
import pathlib
import sys
import threading
import urllib.error
import urllib.request

from llm import llm_analytics as analytics
import pytest

GOLDEN_DIR = pathlib.Path(__file__).parent / 'golden'


def _prompt(command, system='SYSTEM PROMPT', feedback=None):
    """Build the prompt llm_planner_node._completion() builds, byte for byte."""
    text = command
    if feedback:
        text = (f"{command}\n(il tentativo precedente e' stato rifiutato: "
                f'{feedback}. Correggi e rispondi di nuovo solo con il JSON.)')
    return f'{system}\ncomando: "{text}"\nrisposta:'


def _request(command='vai dritto', n_predict=512, feedback=None):
    """Build the JSON body llm_planner_node posts to /completion."""
    return json.dumps({
        'prompt': _prompt(command, feedback=feedback),
        'n_predict': n_predict,
        'temperature': 0.0,
        'json_schema': {'type': 'object'},
    }).encode('utf-8')


def _response(content='{"plan": [], "unsupported": []}', prompt_n=512,
              predicted_n=96, stop_type='word', n_predict=512, timings=True):
    """Build a llama.cpp /completion response with the fields this tool reads."""
    payload = {
        'content': content,
        'stop_type': stop_type,
        'truncated': stop_type == 'limit',
        'stopped_eos': stop_type == 'eos',
        'tokens_evaluated': prompt_n,
        'tokens_predicted': predicted_n,
        'tokens_cached': 0,
        'model': 'test-model',
        'generation_settings': {'n_predict': n_predict, 'n_ctx': 4096,
                                'temperature': 0.0},
    }
    if timings:
        payload['timings'] = {
            'prompt_n': prompt_n,
            'prompt_ms': 630.0,
            'prompt_per_second': round(prompt_n / 0.63, 2),
            'predicted_n': predicted_n,
            'predicted_ms': 2330.0,
            'predicted_per_second': round(predicted_n / 2.33, 2),
        }
    return json.dumps(payload).encode('utf-8')


class TestParsePrompt:
    """The prompt is the one thing the node builds; splitting it must be exact."""

    def test_splits_system_prompt_and_command(self):
        parsed = analytics.parse_prompt(_prompt('gira a destra'))
        assert parsed['well_formed'] is True
        assert parsed['command'] == 'gira a destra'
        assert parsed['system_prompt_chars'] == len('SYSTEM PROMPT')
        assert parsed['retry_feedback'] is None

    def test_splits_the_retry_feedback_from_the_command(self):
        parsed = analytics.parse_prompt(
            _prompt('gira', feedback="mode 'turn' non supportato"))
        assert parsed['command'] == 'gira'
        assert parsed['retry_feedback'] == "mode 'turn' non supportato"

    def test_takes_the_last_marker_so_a_system_prompt_can_mention_comando(self):
        system = 'regole\ncomando: "esempio"\nrisposta: niente da dire'
        parsed = analytics.parse_prompt(_prompt('vai', system=system))
        assert parsed['command'] == 'vai'
        assert parsed['system_prompt_sha256'] == analytics._sha256(system)

    def test_rejects_anything_that_is_not_a_planning_prompt(self):
        assert analytics.parse_prompt('ciao')['well_formed'] is False
        assert analytics.parse_prompt(None)['well_formed'] is False
        assert analytics.parse_prompt('')['well_formed'] is False


class TestClassifyExchange:
    """Warm-up, completion and everything else must not be confused."""

    def test_warmup_is_recognised_by_its_prompt(self):
        body = json.dumps({'prompt': 'ciao', 'n_predict': 8}).encode()
        assert analytics.classify_exchange(analytics._decode_json(body),
                                           '/completion') == analytics.KIND_WARMUP

    def test_planning_prompt_is_a_completion(self):
        assert analytics.classify_exchange(
            analytics._decode_json(_request()), '/completion') == \
            analytics.KIND_COMPLETION

    def test_other_traffic_is_neither(self):
        assert analytics.classify_exchange({}, '/props') == analytics.KIND_OTHER
        assert analytics.classify_exchange(None, '/completion') == \
            analytics.KIND_COMPLETION


class TestTimings:
    """Prompt-eval vs generation, from the server's numbers plus our wall clock."""

    def test_reports_both_speeds_and_the_overhead(self):
        record = analytics.analyse_exchange(
            1, analytics.KIND_COMPLETION, _request(), 200, _response(), 3100.0)
        timings = record['timings']
        assert timings['prompt_n'] == 512
        assert timings['predicted_n'] == 96
        assert timings['prompt_per_second'] == pytest.approx(812.7, abs=0.1)
        assert timings['predicted_per_second'] == pytest.approx(41.2, abs=0.1)
        assert timings['server_ms'] == pytest.approx(2960.0)
        # 3100 ms measured by the proxy minus 2960 ms the server spent working.
        assert timings['overhead_ms'] == pytest.approx(140.0)
        assert timings['prompt_share'] == pytest.approx(0.2128, abs=0.001)

    def test_derives_speeds_when_the_build_omits_them(self):
        timings = analytics.summarise_timings(
            {'prompt_n': 500, 'prompt_ms': 500.0,
             'predicted_n': 50, 'predicted_ms': 1000.0}, wall_ms=1600.0)
        assert timings['prompt_per_second'] == pytest.approx(1000.0)
        assert timings['predicted_per_second'] == pytest.approx(50.0)
        assert timings['overhead_ms'] == pytest.approx(100.0)

    def test_missing_timings_become_nulls_not_an_exception(self):
        record = analytics.analyse_exchange(
            1, analytics.KIND_COMPLETION, _request(), 200, _response(timings=False),
            3100.0)
        assert record['timings']['prompt_ms'] is None
        assert record['timings']['overhead_ms'] is None
        assert record['wall_ms'] == 3100.0

    def test_an_unparseable_body_still_yields_a_record(self):
        record = analytics.analyse_exchange(
            2, analytics.KIND_COMPLETION, _request(), 502, b'<html>nope</html>', 5.0)
        assert record['response']['http_status'] == 502
        assert record['thought']['json_valid'] is False
        assert 'response body is not a JSON object' in record['warnings']


class TestLoad:
    """How much of the generation budget the answer used."""

    def test_budget_used_is_tokens_over_the_applied_cap(self):
        record = analytics.analyse_exchange(
            1, analytics.KIND_COMPLETION, _request(), 200, _response(predicted_n=96),
            3000.0)
        assert record['load']['n_predict_requested'] == 512
        assert record['load']['n_predict_applied'] == 512
        assert record['load']['budget_used'] == pytest.approx(0.188)
        assert record['load']['hit_token_limit'] is False

    def test_a_capped_answer_is_flagged_as_hitting_the_limit(self):
        record = analytics.analyse_exchange(
            1, analytics.KIND_COMPLETION, _request(), 200,
            _response(predicted_n=512, stop_type='limit'), 3000.0)
        assert record['load']['hit_token_limit'] is True
        assert record['load']['truncated'] is True

    def test_the_requested_cap_is_reported_even_if_the_server_clamped_it(self):
        record = analytics.analyse_exchange(
            1, analytics.KIND_COMPLETION, _request(n_predict=512), 200,
            _response(n_predict=256, predicted_n=10), 3000.0)
        assert record['load']['n_predict_requested'] == 512
        assert record['load']['n_predict_applied'] == 256
        assert record['load']['json_constrained'] is True


class TestThought:
    """The raw output, recorded before the airlock sees it."""

    def test_keeps_the_raw_output_verbatim(self):
        content = '{"plan": [{"mode": "straight", "guard": "wall"}], "unsupported": []}'
        thought = analytics.parse_thought(content)
        assert thought['raw_output'] == content
        assert thought['json_valid'] is True
        assert thought['moves'] == 1
        assert thought['trailing_chars'] == 0

    def test_counts_the_trailing_text_raw_decode_discards(self):
        content = '{"plan": [], "unsupported": []} comando: "altro" risposta: bla'
        thought = analytics.parse_thought(content)
        assert thought['json_valid'] is True
        assert thought['trailing_chars'] == len(
            'comando: "altro" risposta: bla')

    def test_output_that_is_not_json_is_kept_and_flagged(self):
        thought = analytics.parse_thought('Mi dispiace, non posso}')
        assert thought['json_valid'] is False
        assert thought['raw_output'] == 'Mi dispiace, non posso}'
        assert thought['empty'] is False

    def test_empty_output_is_marked_empty(self):
        thought = analytics.parse_thought('')
        assert thought['empty'] is True
        assert thought['json_valid'] is False


def _golden_intent(name='ex1_wall_right_distance'):
    """Load the intent half of a committed golden fixture (a translate() input)."""
    with open(GOLDEN_DIR / f'{name}.json', encoding='utf-8') as handle:
        return json.load(handle)['intent']


def _airlock(content, go_to_enabled=True):
    """Run the airlock, skipping the test where plan_translate is unavailable."""
    verdict, warning = analytics.airlock_verdict(content, go_to_enabled)
    if verdict is None:
        pytest.skip(f'airlock unavailable here: {warning}')
    return verdict


class TestAirlock:
    """The predicted verdict of plan_translate on the raw model output."""

    def test_accepts_an_intent_the_golden_fixture_says_translates(self):
        verdict = _airlock(json.dumps(_golden_intent()))
        assert verdict == 'accepted:3 moves'

    def test_rejects_a_schema_violation_by_exception_name(self):
        intent = _golden_intent()
        intent['plan'][0]['guard'] = 'banana'
        verdict = _airlock(json.dumps(intent))
        assert verdict.startswith('rejected:IntentSchemaError:')

    def test_rejects_output_that_is_not_json(self):
        verdict = _airlock('non ho capito')
        assert verdict.startswith('rejected:JSONDecodeError:')

    def test_rejects_json_that_is_not_an_object(self):
        verdict = _airlock('[{"mode": "straight"}]')
        assert verdict.startswith('rejected:ValueError:')


METRICS_TEXT = """# HELP llamacpp:kv_cache_usage_ratio KV cache usage
# TYPE llamacpp:kv_cache_usage_ratio gauge
llamacpp:kv_cache_usage_ratio 0.375
llamacpp:tokens_predicted_total 1234
llamacpp:tokens_evaluated_total 98765
llamacpp:n_decode_total 12
process_resident_memory_bytes{pid="42"} 4.5e+09
garbage line without a value
"""


class TestPrometheus:
    """The /metrics parser: a flat dict, and nothing it cannot read."""

    def test_reads_gauges_and_counters(self):
        metrics = analytics.parse_prometheus(METRICS_TEXT)
        assert metrics['llamacpp:kv_cache_usage_ratio'] == pytest.approx(0.375)
        assert metrics['llamacpp:tokens_predicted_total'] == 1234.0
        assert metrics['process_resident_memory_bytes'] == pytest.approx(4.5e9)

    def test_skips_comments_and_unparseable_lines(self):
        metrics = analytics.parse_prometheus(METRICS_TEXT)
        assert 'garbage' not in metrics
        assert len(metrics) == 5

    def test_finds_a_metric_by_suffix_across_builds(self):
        for text in ('llamacpp:kv_cache_usage_ratio 0.5\n',
                     'llama_kv_cache_usage_ratio 0.5\n',
                     'kv_cache_usage_ratio 0.5\n'):
            metrics = analytics.parse_prometheus(text)
            assert analytics.metric_value(metrics, 'kv_cache_usage_ratio') == 0.5

    def test_missing_metric_is_none(self):
        assert analytics.metric_value({'other': 1.0}, 'kv_cache_usage_ratio') is None


class TestSlots:
    """KV-cache occupancy, from /slots alone (no --metrics needed)."""

    def test_sums_tokens_and_capacity_and_counts_busy_slots(self):
        slots = [
            {'id': 0, 'n_ctx': 2048, 'n_past': 512, 'is_processing': True},
            {'id': 1, 'n_ctx': 2048, 'n_past': 512, 'is_processing': False},
        ]
        kv = analytics.kv_from_slots(slots)
        assert kv['kv_tokens'] == 1024
        assert kv['kv_capacity'] == 4096
        assert kv['kv_ratio'] == pytest.approx(0.25)
        assert kv['processing_slots'] == 1

    def test_slots_endpoint_being_something_else_yields_nothing(self):
        assert analytics.kv_from_slots({'error': 'slots disabled'}) == {}
        assert analytics.kv_from_slots(None) == {}

    def test_a_slot_that_reports_no_occupancy_is_unknown_not_zero(self):
        # n_ctx alone: llama-server knows the size but not what is in it. An
        # unknown must not be rendered as "0% used" (it used to be).
        kv = analytics.kv_from_slots([{'id': 0, 'n_ctx': 4096}])
        assert kv['kv_capacity'] == 4096
        assert kv['kv_tokens'] is None
        assert kv['kv_ratio'] is None

    def test_an_idle_slot_reporting_zero_stays_zero(self):
        kv = analytics.kv_from_slots([{'id': 0, 'n_ctx': 4096, 'n_past': 0}])
        assert kv['kv_tokens'] == 0
        assert kv['kv_ratio'] == 0.0

    def test_reads_occupancy_nested_under_next_token(self):
        # Newer llama.cpp builds put n_ctx/n_past inside next_token.
        kv = analytics.kv_from_slots([
            {'id': 0, 'is_processing': True,
             'next_token': {'has_next_token': True, 'n_ctx': 4096, 'n_past': 1024}},
        ])
        assert kv['kv_tokens'] == 1024
        assert kv['kv_capacity'] == 4096
        assert kv['kv_ratio'] == pytest.approx(0.25)


def _fake_proc(tmp_path, pid=4242, argv=('llama-server', '--port', '8083'),
               rss_kb=4 * 1024 * 1024):
    """Create a /proc tree with one llama-server in it."""
    root = tmp_path / 'proc'
    (root / str(pid)).mkdir(parents=True)
    (root / str(pid) / 'cmdline').write_bytes(
        b'\0'.join(arg.encode() for arg in argv) + b'\0')
    (root / str(pid) / 'status').write_text(
        'Name:\tllama-server\n'
        'VmSize:\t9000000 kB\n'
        f'VmRSS:\t{rss_kb} kB\n'
        'VmHWM:\t5000000 kB\n', encoding='utf-8')
    return root


class TestProc:
    """Finding the server process and reading its memory, from /proc only."""

    def test_finds_the_server_by_its_command_line(self, tmp_path):
        root = _fake_proc(tmp_path)
        assert analytics.find_server_pid(8083, str(root)) == 4242

    def test_does_not_pick_a_server_on_another_port(self, tmp_path):
        root = _fake_proc(tmp_path)
        assert analytics.find_server_pid(8084, str(root)) is None

    def test_reads_rss_peak_and_size_in_mb(self, tmp_path):
        root = _fake_proc(tmp_path)
        memory = analytics.read_proc_memory(4242, str(root))
        assert memory['rss_mb'] == pytest.approx(4096.0)
        assert memory['rss_peak_mb'] == pytest.approx(4882.8, abs=0.1)
        assert memory['vm_size_mb'] == pytest.approx(8789.1, abs=0.1)

    def test_a_vanished_process_reads_as_empty(self, tmp_path):
        root = _fake_proc(tmp_path)
        assert analytics.read_proc_memory(9999, str(root)) == {}

    def test_falls_back_to_the_listening_socket(self, tmp_path):
        root = tmp_path / 'proc'
        (root / '1337' / 'fd').mkdir(parents=True)
        (root / 'net').mkdir()
        # 0x1F93 == 8083, state 0A == LISTEN, socket inode in field 10.
        (root / 'net' / 'tcp').write_text(
            '  sl  local_address rem_address   st tx_queue rx_queue tr '
            'tm->when retrnsmt   uid  timeout inode\n'
            '   0: 0100007F:1F93 00000000:0000 0A 00000000:00000000 '
            '00:00000000 00000000  1000        0 12345 1 0000000000000000 '
            '100 0 0 10 0\n', encoding='utf-8')
        (root / '1337' / 'fd' / '3').symlink_to('socket:[12345]')
        assert analytics.find_server_pid(8083, str(root)) == 1337


class TestPoller:
    """The memory series and its aggregates, without a thread or a server."""

    def _poller(self):
        return analytics.MemoryPoller('http://127.0.0.1:9', port=None, interval=0.1)

    def test_summary_reports_rss_movement_and_kv_peak(self):
        poller = self._poller()
        poller.samples = [
            {'t': 1.0, 'rss_mb': 4000.0, 'kv_ratio': 0.1},
            {'t': 2.0, 'rss_mb': 4100.0, 'kv_ratio': 0.3},
        ]
        poller.pid = 42
        summary = poller.summary()
        assert summary['samples'] == 2
        assert summary['pid'] == 42
        assert summary['rss_baseline_mb'] == 4000.0
        assert summary['rss_peak_mb'] == 4100.0
        assert summary['rss_delta_mb'] == 100.0
        assert summary['kv_ratio_peak'] == 0.3
        assert summary['metrics_available'] is False

    def test_snapshot_picks_the_sample_nearest_the_request(self):
        poller = self._poller()
        poller.samples = [{'t': 1.0, 'rss_mb': 1.0}, {'t': 5.0, 'rss_mb': 5.0}]
        assert poller.snapshot(4.0)['rss_mb'] == 5.0
        assert poller.snapshot(None)['rss_mb'] == 5.0
        assert poller.snapshot(1.5)['rss_mb'] == 1.0

    def test_no_samples_yields_an_empty_snapshot(self):
        assert self._poller().snapshot() == {}


class _FakeLlama:
    """A stand-in llama-server: canned /completion, /props, /slots, /metrics."""

    def __init__(self, body=None, status=200, slots=None, metrics=True):
        self.body = body if body is not None else _response()
        self.status = status
        self.slots = slots if slots is not None else [
            {'id': 0, 'n_ctx': 4096, 'n_past': 1024, 'is_processing': False}]
        self.metrics = metrics
        self.requests = []
        fake = self

        class _Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def _send(self, status, payload, content_type='application/json'):
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def do_POST(self):
                length = int(self.headers.get('Content-Length') or 0)
                fake.requests.append((self.path, self.rfile.read(length)))
                self._send(fake.status, fake.body)

            def do_GET(self):
                if self.path == '/props':
                    payload = json.dumps({
                        'model_path': '/models/test.gguf',
                        'total_slots': 1,
                        'default_generation_settings': {'n_ctx': 4096},
                    }).encode()
                    self._send(200, payload)
                elif self.path == '/slots':
                    self._send(200, json.dumps(fake.slots).encode())
                elif fake.metrics:
                    self._send(200, b'llamacpp:kv_cache_usage_ratio 0.25\n',
                               'text/plain; version=0.0.4')
                else:
                    self._send(404, b'404 page not found', 'text/plain')

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), _Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f'http://127.0.0.1:{self.server.server_address[1]}'

    @property
    def port(self):
        return self.server.server_address[1]

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=5.0)


def _serve(upstream_url, session, timeout=10.0):
    """Start a recording proxy in front of `upstream_url`; returns (proxy, thread)."""
    proxy = analytics.CompletionProxy(('127.0.0.1', 0), upstream_url, session,
                                      timeout=timeout)
    thread = threading.Thread(target=proxy.serve_forever, daemon=True)
    thread.start()
    return proxy, thread


def _stop(proxy, thread):
    """Shut a proxy down and wait for it."""
    proxy.shutdown()
    proxy.server_close()
    thread.join(timeout=5.0)


class TestProxy:
    """The end-to-end contract: bytes through unchanged, and recorded."""

    def test_forwards_the_bytes_and_records_the_exchange(self, tmp_path):
        upstream = _FakeLlama()
        events = tmp_path / 'events.jsonl'
        session = analytics.Session(upstream.url, 'listen', events_path=str(events))
        proxy, thread = _serve(upstream.url, session)
        body = _request('gira a destra al muro')
        try:
            request = urllib.request.Request(
                proxy.url, data=body,
                headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(request, timeout=10) as response:
                forwarded = response.read()
        finally:
            _stop(proxy, thread)
            upstream.close()
            session.close()
        assert forwarded == upstream.body
        assert upstream.requests == [('/completion', body)]
        assert len(session.records) == 1
        record = session.records[0]
        assert record['kind'] == analytics.KIND_COMPLETION
        assert record['input']['command'] == 'gira a destra al muro'
        assert record['input']['well_formed'] is True
        assert record['timings']['prompt_n'] == 512
        assert record['thought']['json_valid'] is True
        assert analytics.read_events(events) == session.records

    def test_a_dead_upstream_becomes_502_and_a_warning(self, tmp_path):
        dead = 'http://127.0.0.1:1'
        session = analytics.Session(dead, 'listen',
                                    events_path=str(tmp_path / 'events.jsonl'))
        proxy, thread = _serve(dead, session, timeout=2.0)
        try:
            request = urllib.request.Request(
                proxy.url, data=_request(),
                headers={'Content-Type': 'application/json'})
            with pytest.raises(urllib.error.HTTPError) as failure:
                urllib.request.urlopen(request, timeout=10)
        finally:
            _stop(proxy, thread)
            session.close()
        assert failure.value.code == 502
        assert session.records[0]['response']['http_status'] == 502
        assert any('upstream error' in warning for warning in session.warnings)

    def test_the_url_it_advertises_is_what_the_node_needs(self):
        upstream = _FakeLlama()
        session = analytics.Session(upstream.url, 'listen')
        proxy, thread = _serve(upstream.url, session)
        try:
            assert proxy.url.startswith('http://127.0.0.1:')
            assert proxy.url.endswith('/completion')
            assert str(proxy.server_address[1]) in proxy.url
        finally:
            _stop(proxy, thread)
            upstream.close()


class TestPollerAgainstServer:
    """The poller against a llama-server that does not serve /metrics."""

    def test_missing_metrics_warns_once_and_stops_polling_them(self):
        upstream = _FakeLlama(metrics=False)
        warnings = []
        poller = analytics.MemoryPoller(upstream.url, interval=0.1,
                                        on_warning=warnings.append)
        try:
            poller._sample_once()
            poller._sample_once()
        finally:
            upstream.close()
        assert poller.metrics_enabled is False
        assert len(warnings) == 1
        assert '--metrics' in warnings[0]
        assert poller.samples[0]['kv_tokens'] == 1024
        assert poller.samples[0]['kv_capacity'] == 4096

    def test_metrics_that_do_exist_are_kept(self):
        upstream = _FakeLlama()
        poller = analytics.MemoryPoller(upstream.url, interval=0.1)
        try:
            poller._sample_once()
        finally:
            upstream.close()
        assert poller.metrics_enabled is True
        assert poller.samples[0]['kv_ratio_metrics'] == 0.25
        assert poller.summary()['kv_ratio_metrics_last'] == pytest.approx(0.25)


def _records():
    """Build a warm-up and two completions (the second one a retry)."""
    warmup_body = json.dumps({'prompt': 'ciao', 'n_predict': 8}).encode()
    content = '{"plan": [], "unsupported": []}'
    return [
        analytics.analyse_exchange(
            1, analytics.KIND_WARMUP, warmup_body, 200,
            _response(content='ok', prompt_n=30, predicted_n=8), 12000.0),
        analytics.analyse_exchange(
            2, analytics.KIND_COMPLETION, _request('gira'), 200,
            _response(content=content), 3100.0, occurrence=1),
        analytics.analyse_exchange(
            3, analytics.KIND_COMPLETION,
            _request('gira', feedback='IntentSchemaError: guard sconosciuto'),
            200, _response(content=content), 2900.0, occurrence=2),
    ]


class TestSummary:
    """The aggregate view: what an operator reads first."""

    def test_counts_kinds_and_keeps_the_warmup_out_of_the_averages(self):
        summary = analytics.build_summary(_records())
        assert summary['exchanges'] == 3
        assert summary['by_kind'][analytics.KIND_WARMUP] == 1
        assert summary['by_kind'][analytics.KIND_COMPLETION] == 2
        assert summary['wall_ms']['n'] == 2
        assert summary['wall_ms']['mean'] == pytest.approx(3000.0)
        assert summary['warmup_wall_ms']['max'] == pytest.approx(12000.0)
        assert summary['retries'] == 1
        assert summary['prompt_parsed'] == 2
        assert summary['json_valid'] == 2

    def test_totals_tokens_and_commands(self):
        summary = analytics.build_summary(_records())
        assert summary['tokens']['evaluated'] == 1024
        assert summary['tokens']['predicted'] == 192
        assert summary['commands'] == {'gira': 2}

    def test_tallies_the_airlock_verdicts(self):
        records = _records()
        records[1]['thought']['airlock'] = 'rejected:IntentSchemaError: bad guard'
        records[2]['thought']['airlock'] = 'accepted:2 moves'
        summary = analytics.build_summary(records)
        assert summary['airlock'] == {
            'accepted': 1, 'rejected': 1, 'by_error': {'IntentSchemaError': 1}}

    def test_counts_failed_exchanges(self):
        records = _records()
        records[1]['response']['http_status'] = 502
        assert analytics.build_summary(records)['failures'] == 1

    def test_an_empty_capture_summarises_to_zeros(self):
        summary = analytics.build_summary([])
        assert summary['exchanges'] == 0
        assert 'wall_ms' not in summary

    def test_format_summary_renders_without_a_server(self):
        text = '\n'.join(analytics.format_summary(analytics.build_summary(_records())))
        assert 'completion 2' in text
        assert 'prompt eval' in text
        assert 'generation' in text

    def test_format_summary_renders_the_server_block(self):
        server = {
            'pid': 42,
            'props': {'n_ctx': 4096, 'total_slots': 1, 'model_path': '/m/x.gguf'},
            'rss_baseline_mb': 4000.0, 'rss_peak_mb': 4100.0,
            'rss_last_mb': 4100.0, 'rss_delta_mb': 100.0,
            'kv_ratio_peak': 0.25, 'metrics_available': False,
        }
        text = '\n'.join(analytics.format_summary({}, server))
        assert 'pid 42' in text
        assert 'kv cache peak' in text
        assert 'not served' in text


class TestReportAndReplay:
    """The files: one JSON report, its Markdown twin, one JSONL line per exchange."""

    def test_the_report_round_trips_through_the_event_file(self, tmp_path):
        session = analytics.Session('http://127.0.0.1:8083', 'listen',
                                    events_path=str(tmp_path / 'events.jsonl'))
        session.add('/completion', _request('vai'), 200, _response(), 3000.0)
        payload = session.report()
        session.close()
        path = analytics.write_report(str(tmp_path / 'report.json'), payload)
        with open(path, encoding='utf-8') as handle:
            reloaded = json.load(handle)
        assert reloaded['summary'] == payload['summary']
        assert reloaded['exchanges'] == payload['exchanges']
        events = analytics.read_events(tmp_path / 'events.jsonl')
        assert events == payload['exchanges']

    def test_replay_writes_a_report_and_a_summary(self, tmp_path):
        events = tmp_path / 'events.jsonl'
        session = analytics.Session('http://127.0.0.1:8083', 'listen',
                                    events_path=str(events))
        session.add('/completion', _request('vai'), 200, _response(), 3000.0)
        session.close()
        out = tmp_path / 'replayed.json'
        assert analytics._replay(str(events), str(out)) == 0
        with open(out, encoding='utf-8') as handle:
            payload = json.load(handle)
        assert payload['session']['replayed_from'] == str(events)
        assert payload['summary']['exchanges'] == 1

    def test_the_markdown_twin_sits_beside_the_json(self):
        assert analytics.markdown_path('run/report.json') == 'run/report.md'
        assert analytics.markdown_path('run/report') == 'run/report.md'
        assert analytics.markdown_path('run/report.md') == 'run/report.md'

    def test_replay_writes_the_table_next_to_the_json(self, tmp_path):
        events = tmp_path / 'events.jsonl'
        session = analytics.Session('http://127.0.0.1:8083', 'listen',
                                    events_path=str(events))
        session.add('/completion', _request('vai'), 200, _response(), 3000.0)
        session.close()
        out = tmp_path / 'replayed.json'
        assert analytics._replay(str(events), str(out)) == 0
        markdown = (tmp_path / 'replayed.md').read_text(encoding='utf-8')
        header = ('| Command Prompt | Temp | JSON Valid | Airlock Verdict '
                  '| Latency | Tokens | Tok/s |')
        assert header in markdown
        assert ('| vai | 0.0 | Pass | not run (no verdict recorded) '
                '| 3000.0 | 608 | 41.2 |') in markdown
        assert markdown.startswith('# llm_analytics report')
        assert f'- replayed from: `{events}`' in markdown
        assert '## Summary' in markdown

    def test_verdicts_are_readable_and_text_cannot_break_the_row(self):
        records = _records()
        records[1]['thought']['airlock'] = 'accepted:3 moves'
        records[2]['thought']['airlock'] = (
            'rejected:IntentSchemaError: guard | soglia non validi')
        records[2]['input']['command'] = 'gira | poi\nvai'
        table = '\n'.join(analytics.markdown_table(records))
        assert '| Accepted: 3 moves |' in table
        assert 'guard \\| soglia non validi |' in table
        assert '| gira \\| poi<br>vai |' in table
        assert '| (warm-up) |' in table

    def test_a_sparse_record_renders_instead_of_raising(self):
        table = analytics.markdown_table([{'index': 1}])
        assert len(table) == 3
        assert '(prompt not parsed)' in table[-1]
        assert '| Fail |' in table[-1]
        assert table[-1].count('|') == 8

    def test_write_reports_creates_both_files_in_a_new_directory(
            self, tmp_path):
        target = tmp_path / 'run' / 'report.json'
        payload = {'session': {}, 'summary': {}, 'server': {},
                   'exchanges': [], 'runs': []}
        json_path, markdown_file = analytics.write_reports(
            str(target), payload)
        assert json_path == str(target)
        assert markdown_file == str(tmp_path / 'run' / 'report.md')
        text = pathlib.Path(markdown_file).read_text(encoding='utf-8')
        assert '| Command Prompt |' in text
        assert '## Exchanges' in text

    def test_a_corrupt_event_line_is_skipped_not_fatal(self, tmp_path):
        path = tmp_path / 'events.jsonl'
        path.write_text('{"index": 1}\nnot json at all\n', encoding='utf-8')
        assert analytics.read_events(str(path)) == [{'index': 1}]

    def test_replay_of_a_missing_file_fails_cleanly(self, tmp_path):
        assert analytics._replay(str(tmp_path / 'nope.jsonl'),
                                 str(tmp_path / 'out.json')) == 2


class TestInputSnapshot:
    """The commands a run measured are kept in that run's own folder.

    `analytics_runs/` is untracked and every artifact in it is truncated on
    purpose, so a capture without its commands is a measurement nobody can
    repeat. The tool copies the input before the first command goes out and
    names the copy in the report.
    """

    def test_the_snapshot_sits_beside_the_capture(self):
        assert (analytics.commands_snapshot_path('run/events.jsonl')
                == 'run/commands.txt')
        assert (analytics.commands_snapshot_path('events.jsonl')
                == 'commands.txt')

    def test_an_exec_file_is_copied_byte_for_byte(self, tmp_path):
        source = _write_commands(
            tmp_path, '# il giro di prova\n\nvai dritto\n  gira a destra  \n',
            name='sweep.txt')
        fields = analytics.write_commands_snapshot(
            str(tmp_path / 'run' / 'events.jsonl'), exec_file=str(source))
        copy = tmp_path / 'run' / 'commands.txt'
        assert fields['commands_snapshot'] == str(copy)
        assert fields['commands_source'] == 'exec-file'
        assert fields['commands_count'] == 2
        # Hand it straight back to --exec-file: the bytes have to be the same,
        # or the next run is a different run that looks identical.
        assert copy.read_bytes() == source.read_bytes()
        assert fields['commands_sha256'] == hashlib.sha256(
            source.read_bytes()).hexdigest()

    def test_exec_commands_are_written_one_per_line(self, tmp_path):
        fields = analytics.write_commands_snapshot(
            str(tmp_path / 'events.jsonl'),
            commands=['vai dritto', 'gira a destra'])
        copy = tmp_path / 'commands.txt'
        assert fields['commands_source'] == 'exec'
        assert copy.read_text(encoding='utf-8') == 'vai dritto\ngira a destra\n'
        # The format is the one the tool reads back.
        assert analytics.load_command_file(str(copy)) == [
            'vai dritto', 'gira a destra']

    def test_nothing_to_copy_or_nowhere_to_put_it(self, tmp_path):
        # An idle recorder has no input; --no-events has no capture folder.
        assert analytics.write_commands_snapshot(
            str(tmp_path / 'events.jsonl')) == {}
        assert analytics.write_commands_snapshot(None, commands=['vai']) == {}
        assert list(tmp_path.iterdir()) == []

    def test_a_rerun_replaces_the_old_copy_rather_than_appending(self, tmp_path):
        target = str(tmp_path / 'events.jsonl')
        analytics.write_commands_snapshot(target, commands=['a', 'b'])
        analytics.write_commands_snapshot(target, commands=['c'])
        assert (tmp_path / 'commands.txt').read_text(encoding='utf-8') == 'c\n'

    def test_the_markdown_names_the_input_beside_the_prompt(self, tmp_path):
        payload = {
            'session': {
                'commands_snapshot': str(tmp_path / 'commands.txt'),
                'commands_count': 20,
                'commands_sha256': 'a' * 64,
            },
            'summary': {}, 'server': {}, 'exchanges': [], 'runs': [],
        }
        text = analytics.format_markdown_report(payload)
        assert ('- input: `commands.txt`  (20 commands, '
                'sha256 `aaaaaaaaaaaa`)') in text

    def test_a_sweep_keeps_its_commands_in_the_run_folder(self, tmp_path):
        source = _write_commands(
            tmp_path, '# prova\nvai dritto\ngira a destra\n', name='sweep.txt')
        run = tmp_path / 'run'
        run.mkdir()
        code = analytics.main([
            '--exec-file', str(source),
            '--upstream', 'http://127.0.0.1:1',
            '--listen', '127.0.0.1:0',
            '--timeout', '2',
            '--no-memory',
            '--events', str(run / 'events.jsonl'),
            '--out', str(run / 'report.json'),
        ])
        # Dead upstream: every line is recorded, nothing is fatal.
        assert code == 1
        assert (run / 'commands.txt').read_bytes() == source.read_bytes()
        payload = json.loads((run / 'report.json').read_text(encoding='utf-8'))
        session = payload['session']
        assert session['commands_source'] == 'exec-file'
        assert session['commands_count'] == 2
        markdown = (run / 'report.md').read_text(encoding='utf-8')
        assert f'- exec file: {source}' in markdown
        assert '- input: `commands.txt`  (2 commands, sha256 `' in markdown


class TestNodeCommand:
    """--exec: the node is run as a subprocess, pointed at the proxy."""

    URL = 'http://127.0.0.1:8084/completion'

    def test_points_the_node_at_the_proxy_and_dry_runs_by_default(self, monkeypatch):
        monkeypatch.setattr(analytics.shutil, 'which', lambda name: '/usr/bin/ros2')
        argv = analytics.build_node_command('gira a destra', self.URL)
        assert argv[:4] == ['ros2', 'run', 'llm', 'llm_planner_node']
        assert argv[4] == 'gira a destra'
        assert '--dry-run' in argv
        assert argv.index('--ros-args') > argv.index('--dry-run')
        assert f'llm_url:={self.URL}' in argv
        assert not any(arg.startswith('go_to_enabled') for arg in argv)

    def test_live_and_go_to_are_both_opt_in(self, monkeypatch):
        monkeypatch.setattr(analytics.shutil, 'which', lambda name: '/usr/bin/ros2')
        argv = analytics.build_node_command('gira', self.URL, dry_run=False,
                                            go_to=False)
        assert '--dry-run' not in argv
        assert 'go_to_enabled:=false' in argv

    def test_sources_the_workspace_when_ros2_is_not_on_path(self, monkeypatch,
                                                            tmp_path):
        monkeypatch.setattr(analytics.shutil, 'which', lambda name: None)
        setup = str(tmp_path / 'setup.bash')
        monkeypatch.setattr(analytics, '_setup_bash', lambda: setup)
        argv = analytics.build_node_command('gira', self.URL)
        assert argv[:2] == ['bash', '-c']
        assert setup in argv[2]
        assert 'llm_planner_node' in argv[2]

    def test_says_so_when_it_cannot_find_ros(self, monkeypatch):
        monkeypatch.setattr(analytics.shutil, 'which', lambda name: None)
        monkeypatch.setattr(analytics, '_setup_bash', lambda: None)
        with pytest.raises(RuntimeError, match='ros2 is not on PATH'):
            analytics.build_node_command('gira', self.URL)

    def test_parses_the_node_result_line(self):
        stdout = ('noise\nRESULT {"status": "ok", "attempts": 1}\nmore noise\n')
        assert analytics.parse_result_line(stdout) == {'status': 'ok',
                                                       'attempts': 1}
        assert analytics.parse_result_line('no result here') is None
        assert analytics.parse_result_line('RESULT not json') is None

    def test_runs_a_node_command_and_parses_its_result(self):
        script = ('import json; '
                  'print("RESULT " + json.dumps({"status": "refused"}))')
        run = analytics.run_node_command([sys.executable, '-c', script],
                                         timeout_s=30.0)
        assert run['exit_code'] == 0
        assert run['timed_out'] is False
        assert run['result'] == {'status': 'refused'}
        assert 'RESULT' in run['stdout_tail']

    def test_a_node_that_never_ends_is_killed_and_reported(self):
        script = 'import time; time.sleep(30)'
        run = analytics.run_node_command([sys.executable, '-c', script],
                                         timeout_s=0.5)
        assert run['timed_out'] is True
        assert run['duration_s'] < 10.0


def _write_commands(tmp_path, text, name='commands.txt'):
    """Write a --exec-file and return its path."""
    path = tmp_path / name
    path.write_text(text, encoding='utf-8')
    return path


def _batch_args(path):
    """Parse the real command line, so the flag itself is covered too."""
    return analytics.build_parser().parse_args(['--exec-file', str(path)])


class TestExecFile:
    """--exec-file: the node's own request, batched, and never a dying loop."""

    def test_the_request_is_the_one_the_node_posts(self):
        request = analytics.build_intent_request('gira a destra', 'PROMPT')
        parsed = analytics.parse_prompt(request['prompt'])
        assert request['prompt'] == 'PROMPT\ncomando: "gira a destra"\nrisposta:'
        assert parsed['well_formed'] is True
        assert parsed['command'] == 'gira a destra'
        assert parsed['system_prompt_sha256'] == analytics._sha256('PROMPT')
        assert request['n_predict'] == analytics.N_PREDICT == 512
        assert request['temperature'] == analytics.TEMPERATURE == 0.0
        assert request['json_schema']['required'] == ['plan', 'unsupported']
        # The node sends no stop list; a batch must not add one for tidiness.
        assert 'stop' not in request

    def test_the_prompt_is_a_file_and_go_to_picks_which(self):
        try:
            with_go_to, filename = analytics.load_system_prompt(True)
            without, other = analytics.load_system_prompt(False)
        except RuntimeError as error:  # ex RuntimeError
            pytest.skip(f'plan_translate unavailable here: {error}')
        assert filename != other
        assert with_go_to != without
        # One prompt teaches go_to, the other deliberately never mentions it:
        # that pairing with the translator is what the node relies on.
        assert 'go_to' in with_go_to
        assert 'go_to' not in without

    def test_blank_lines_and_comments_are_skipped(self, tmp_path):
        path = _write_commands(
            tmp_path, '\n# a comment\n  vai dritto  \n\n#altro\ngira a destra\n')
        assert analytics.load_command_file(path) == ['vai dritto', 'gira a destra']

    def test_a_missing_or_empty_file_stops_before_anything_is_sent(self, tmp_path):
        session = analytics.Session('http://127.0.0.1:1', 'listen')
        missing = _batch_args(tmp_path / 'nope.txt')
        empty = _batch_args(_write_commands(tmp_path, '# only a comment\n\n'))
        # server=None on purpose: both must fail before touching a socket.
        assert analytics._run_batch(missing, session, None) == 2
        assert analytics._run_batch(empty, session, None) == 2
        assert session.records == []

    def test_live_with_only_a_command_file_is_refused(self, tmp_path):
        path = _write_commands(tmp_path, 'vai dritto\n')
        assert analytics.main(['--exec-file', str(path), '--live']) == 2

    def test_every_line_is_recorded_with_latency_speed_and_json_verdict(
            self, tmp_path):
        upstream = _FakeLlama()
        events = tmp_path / 'events.jsonl'
        session = analytics.Session(upstream.url, 'listen',
                                    events_path=str(events))
        opts = _batch_args(_write_commands(
            tmp_path, 'vai dritto al muro\ngira a destra\n'))
        proxy, thread = _serve(upstream.url, session)
        try:
            assert analytics._run_batch(opts, session, proxy) == 0
        finally:
            _stop(proxy, thread)
            upstream.close()
            session.close()
        assert [record['input']['command'] for record in session.records] == [
            'vai dritto al muro', 'gira a destra']
        assert [record['kind'] for record in session.records] == [
            analytics.KIND_COMPLETION, analytics.KIND_COMPLETION]
        for record in session.records:
            assert record['input']['well_formed'] is True
            assert record['request']['params']['n_predict'] == 512
            assert record['tokens']['predicted'] == 96
            assert record['thought']['json_valid'] is True
            # latency: the server's own, plus the wall clock that includes us
            assert record['timings']['predicted_ms'] == 2330.0
            assert record['timings']['overhead_ms'] is not None
            assert record['wall_ms'] is not None
            # generation speed, as llama-server reports it
            assert record['timings']['predicted_per_second'] == 41.2
        assert [path for path, _ in upstream.requests] == [
            '/completion', '/completion']
        assert analytics.read_events(events) == session.records

    def test_a_malformed_output_is_flagged_and_the_batch_continues(self, tmp_path):
        upstream = _FakeLlama(body=_response(content='non ho capito'))
        session = analytics.Session(upstream.url, 'listen',
                                    events_path=str(tmp_path / 'events.jsonl'))
        opts = _batch_args(_write_commands(tmp_path, 'vai dritto\ngira\n'))
        proxy, thread = _serve(upstream.url, session)
        try:
            assert analytics._run_batch(opts, session, proxy) == 1
        finally:
            _stop(proxy, thread)
            upstream.close()
            session.close()
        # Both lines went out and both were recorded: the loop did not stop at 1.
        assert len(session.records) == 2
        assert [record['thought']['json_valid']
                for record in session.records] == [False, False]
        assert any('no JSON value' in warning for warning in session.warnings)

    def test_a_dead_upstream_does_not_end_the_batch(self, tmp_path):
        dead = 'http://127.0.0.1:1'
        session = analytics.Session(dead, 'listen',
                                    events_path=str(tmp_path / 'events.jsonl'))
        opts = _batch_args(_write_commands(tmp_path, 'vai dritto\ngira\n'))
        proxy, thread = _serve(dead, session, timeout=2.0)
        try:
            assert analytics._run_batch(opts, session, proxy) == 1
        finally:
            _stop(proxy, thread)
            session.close()
        assert len(session.records) == 2
        assert [record['response']['http_status']
                for record in session.records] == [502, 502]
        assert session.records[0]['response']['error']

    def test_the_airlock_verdict_lands_in_the_log(self, tmp_path):
        try:
            import llm.plan_translate  # noqa: F401
        except Exception as error:  # noqa: BLE001 -- the airlock is optional
            pytest.skip(f'plan_translate unavailable here: {error}')
        upstream = _FakeLlama(body=_response(content=json.dumps(_golden_intent())))
        session = analytics.Session(upstream.url, 'listen', airlock=True,
                                    events_path=str(tmp_path / 'events.jsonl'))
        opts = _batch_args(_write_commands(
            tmp_path, 'vai dritto al muro e gira a destra\n'))
        proxy, thread = _serve(upstream.url, session)
        try:
            assert analytics._run_batch(opts, session, proxy) == 0
        finally:
            _stop(proxy, thread)
            upstream.close()
            session.close()
        assert session.records[0]['thought']['airlock'] == 'accepted:3 moves'
