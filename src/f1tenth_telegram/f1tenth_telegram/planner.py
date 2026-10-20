"""
Plain-text messages as commands for the LLM planner (llm_planner_node).

The planner runs on the car in interactive mode and PLAN-ONLY (its own
--dry-run flag: it never loads or starts a mission), its stdin on a FIFO:
`f1tenth_ctl send planner "<text>"` writes one line and answers the log's
byte offset at that moment; `f1tenth_ctl planner-log --since <offset>`
returns what the planner printed after it. An Exchange is one command: it
sends, then polls the log until the planner's end-of-answer line.

The end-of-answer line: llm_planner_node._emit_result() prints exactly one
`RESULT {json}` per command, always the last line (status ok, llm_error,
translator_error, refused or node_error). The generated mission is printed
after a `JSON (<mission_id>):` line as indented JSON; it is shown as a code
block. The interactive prompt `comando> ` has no newline, so it sticks to
the start of the next line in the log and is removed.
"""

import html
import json
import re
import time
from typing import List, Optional

MAX_LEN = 500                  # as f1tenth_ctl's units.MAX_LINE
RESULT_PREFIX = 'RESULT '      # llm_planner_node.RESULT_PREFIX
PROMPT = 'comando> '           # llm_planner_node._input_loop's input() prompt
JSON_HEAD = re.compile(r'^JSON \((?P<id>.*)\):\s*$')
MAX_CHARS = 3500               # a Telegram message holds 4096
POLL_S = 2.0
MAX_S = 60.0


def validate(text: str):
    """
    Return (line, None) for a sendable command, else (None, why) (pure).

    Line breaks and runs of spaces collapse to one space (a typed message or
    a transcript); what is left must be 1-500 characters.
    """
    if text is None:
        return None, 'empty text'
    line = ' '.join(text.split())
    if not line:
        return None, 'empty text'
    if len(line) > MAX_LEN:
        return None, f'too long: {len(line)} characters, at most {MAX_LEN}'
    return line, None


def clean(line: str) -> str:
    while line.startswith(PROMPT):
        line = line[len(PROMPT):]
    return line.rstrip()


def parse(lines: List[str]) -> dict:
    """Split the planner's output into body, mission JSON and RESULT (pure)."""
    body, mission, mission_id, result = [], None, None, None
    lines = [clean(x) for x in lines]
    i = 0
    while i < len(lines):
        line = lines[i]
        head = JSON_HEAD.match(line)
        if head and mission is None and i + 1 < len(lines) and lines[i + 1] == '{':
            block = []
            j = i + 1
            while j < len(lines):
                block.append(lines[j])
                if lines[j] == '}':
                    break
                j += 1
            if block[-1] == '}':
                mission, mission_id = '\n'.join(block), head.group('id')
                i = j + 1
                continue
        if line.startswith(RESULT_PREFIX):
            raw = line[len(RESULT_PREFIX):]
            try:
                result = json.loads(raw)
            except ValueError:
                result = {'status': 'unparsed', 'raw': raw[:300]}
            i += 1
            continue
        body.append(line)
        i += 1
    while body and not body[0].strip():
        body.pop(0)
    while body and not body[-1].strip():
        body.pop()
    return {'body': body, 'mission_json': mission, 'mission_id': mission_id,
            'result': result, 'done': result is not None}


def _tail(text: str, budget: int) -> str:
    if len(text) <= budget:
        return text
    return '…' + text[-max(0, budget - 1):]


def result_line(result: Optional[dict]) -> Optional[str]:
    if result is None:
        return None
    status = result.get('status', '?')
    if status == 'ok':
        return '✅ result: ok'
    detail = result.get('error_message') or result.get('raw') or ''
    kind = result.get('error_type')
    detail = f'{kind}: {detail}' if kind else detail
    icon = '⚠️' if status == 'refused' else '❌'
    return f'{icon} result: {status}' + (f' ({detail[:200]})' if detail else '')


def render(text: str, lines: List[str], state: str, elapsed_s: float = 0.0,
           max_chars: int = MAX_CHARS, as_html: bool = True) -> str:
    """
    One message: the command, the state, the RESULT, the output, the mission.

    The output is trimmed from the start (the end matters most), the mission
    JSON keeps up to 2000 characters, and everything fits in max_chars.
    """
    esc = html.escape if as_html else (lambda s: s)
    p = parse(lines)
    head = [f'📝 «{esc(text)}» → planner', esc(state.replace('{s}', str(int(elapsed_s))))]
    res = result_line(p['result'])
    if res:
        head.append(esc(res))
    out = '\n'.join(head)
    mission = p['mission_json']
    mission_part = ''
    if mission:
        title = f"mission {p['mission_id']}" if p['mission_id'] else 'mission'
        body = _tail(mission, min(len(mission), 2000))
        mission_part = (f'\n\n{esc(title)}:\n<pre><code class="language-json">{esc(body)}'
                        '</code></pre>' if as_html else f'\n\n{title}:\n{body}')
    budget = max_chars - len(out) - len(mission_part) - 4
    body = '\n'.join(p['body'])
    if body and budget > 20:
        out += '\n\n' + esc(_tail(body, budget))
    return out + mission_part


class Exchange:
    """One planner command: send it, then follow the log to its RESULT line."""

    WAITING = '⏳ waiting for the planner… {s} s'
    DONE = '🏁 planner answered in {s} s'
    TIMEOUT = '⌛ no RESULT line after {s} s: stopped following the log'

    def __init__(self, text: str, clock=time.monotonic, max_s: float = MAX_S):
        self.text = text
        self.clock = clock
        self.max_s = max_s
        self.cursor = None
        self.lines: List[str] = []
        self.results: List[dict] = []     # f1tenth_ctl's RESULT entries (stored missions)
        self.started = None
        self.finished = None
        self.done = False
        self.state = self.WAITING
        self.error = None

    def send(self, transport, timeout: float) -> bool:
        res = transport.ctl(['send', 'planner', self.text], timeout)
        if not res.get('ok'):
            self.done = True
            self.error = res.get('error') or 'not sent'
            self.state = f'❌ not sent: {self.error}'
            return False
        self.cursor = res.get('cursor', 0)
        self.started = self.clock()
        return True

    def elapsed(self) -> float:
        if self.started is None:
            return 0.0
        return (self.finished or self.clock()) - self.started

    def _finish(self, state: str) -> bool:
        self.done, self.state = True, state
        self.finished = self.clock()
        return True

    def poll(self, transport, timeout: float) -> bool:
        """Fetch the new lines; return whether the message has to change."""
        if self.done:
            return False
        res = transport.ctl(['planner-log', '--since', str(self.cursor)], timeout)
        changed = False
        if res.get('transport_error') or not res.get('ok'):
            self.error = res.get('error') or 'planner-log failed'
        else:
            self.cursor = res.get('cursor', self.cursor)
            self.results.extend(res.get('results') or [])
            new = res.get('lines') or []
            if new:
                self.lines.extend(new)
                self.lines = self.lines[-400:]
                changed = True
            if parse(self.lines)['done']:
                return self._finish(self.DONE)
            if res.get('running') is False:
                return self._finish('❌ the planner is not running any more')
        if self.elapsed() >= self.max_s:
            return self._finish(self.TIMEOUT)
        return changed

    def render(self, as_html: bool = True) -> str:
        return render(self.text, self.lines, self.state, self.elapsed(), as_html=as_html)

    def final_result(self) -> Optional[dict]:
        """Return f1tenth_ctl's entry for this command's RESULT line, if any."""
        return self.results[-1] if self.results else None


def _fmt_speed(v) -> str:
    return f'{v:g} m/s' if v else 'controller default'


def render_offer(text: str, entry: dict, as_html: bool = True) -> str:
    """
    Render the plan for a human to decide: summary first, the JSON collapsed.

    entry is f1tenth_ctl's RESULT entry with a stored mission: `stored` (id,
    summary, unsupported, notes, plan_seconds) and `mission`.
    """
    esc = html.escape if as_html else (lambda s: s)
    rec = entry['stored']
    sm = rec.get('summary') or {}
    lines = [f'🧭 Plan for «{esc(text)}»'
             + (f' ({rec["plan_seconds"]:.1f} s)' if rec.get('plan_seconds') else ''),
             esc(f"mission {sm.get('mission_id')} · id {rec['id']}"),
             esc(f"{sm.get('n_moves')} move(s) · max speed {_fmt_speed(sm.get('max_speed_mps'))}"
                 f" · reverse: {'yes' if sm.get('reverse') else 'no'}")]
    if sm.get('speed_note'):
        lines.append(esc(f"({sm['speed_note']})"))
    if sm.get('targets'):
        lines.append(esc('target: ' + ', '.join(str(t) for t in sm['targets'])))
    length = (f"~{sm['distance_m']:g} m" if sm.get('distance_m') is not None
              else 'not known ahead (it stops on a condition)')
    dur = f", at most {sm['max_duration_s']:g} s (timeouts)" if sm.get('max_duration_s') else ''
    lines.append(esc(f'length: {length}{dur}'))
    for m in sm.get('moves') or []:
        lines.append(esc(f"{m['i'] + 1}. {m['action']} at {_fmt_speed(m.get('speed_mps'))}, "
                         f"until {m['until']} ({m.get('timeout_s'):g} s → {m.get('on_timeout')})"
                         if m.get('timeout_s') is not None else
                         f"{m['i'] + 1}. {m['action']}, until {m['until']}"))
    for item in rec.get('unsupported') or []:
        lines.append(esc(f'⚠️ NOT translated, will not happen: {item}'))
    for note in rec.get('notes') or []:
        lines.append(esc(f'note: {note}'))
    body = json.dumps(entry.get('mission') or {}, indent=2, ensure_ascii=False)
    body = body if len(body) <= 2000 else body[:1990] + '\n…'
    if as_html:
        lines.append(f'<blockquote expandable>{esc(body)}</blockquote>')
    else:
        lines.append(body)
    lines.append(esc('▶️ Start mission asks once more. A plan older than 5 min cannot be '
                     'started: plan again.'))
    return '\n'.join(lines)


def render_failure(ex: 'Exchange', as_html: bool = True) -> str:
    """No mission: the planner's reason, from its RESULT line, and its last output."""
    entry = ex.final_result()
    head = None
    if entry is not None and entry.get('status') != 'ok':
        head = result_line({k: entry.get(k) for k in ('status', 'error_type',
                                                      'error_message')})
    elif entry is not None:
        head = '❌ the planner answered ok but no mission was stored'
    out = ex.render(as_html)
    if head and head not in out:
        out = (html.escape(head) if as_html else head) + '\n' + out
    return out + ('\n' if out else '') + 'No mission, nothing to start.'
