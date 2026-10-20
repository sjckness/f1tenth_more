"""
The Telegram glue with the real python-telegram-bot, never contacting Telegram.

run_polling is replaced, the handlers are taken from the Application and fed
stand-in updates. Skipped where python-telegram-bot is not installed (the
ROS workspace's colcon test); run them in the bot's venv.
"""

import asyncio
from datetime import datetime, timedelta, timezone
from types import SimpleNamespace

from f1tenth_telegram import config, tg
from f1tenth_telegram.controller import Controller
import pytest

from test_controller import FakeTransport

OWNER = 111
TOKEN = '123456:ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghi'


class Bot:
    def __init__(self):
        self.sent, self.edits, self.next_id = [], [], 100

    async def send_message(self, chat_id, text, reply_markup=None):
        self.next_id += 1
        self.sent.append((chat_id, text, reply_markup))
        return SimpleNamespace(message_id=self.next_id, chat_id=chat_id)

    async def edit_message_reply_markup(self, chat_id, message_id, reply_markup=None):
        self.edits.append((chat_id, message_id, None, reply_markup))

    async def edit_message_text(self, text, chat_id, message_id, reply_markup=None,
                                parse_mode=None):
        self.edits.append((chat_id, message_id, text, reply_markup))
        self.parse_modes = getattr(self, 'parse_modes', []) + [parse_mode]


@pytest.fixture
def app(monkeypatch):
    # Skipped per test, not at module level: launch_testing's collection hook
    # turns a module-level skip into a skip of the whole directory.
    ext = pytest.importorskip('telegram.ext')
    captured = {}

    def fake_run_polling(self, **kwargs):
        captured['app'] = self
        captured['kwargs'] = kwargs

    monkeypatch.setattr(ext.Application, 'run_polling', fake_run_polling)
    cfg = config.from_dict({'telegram': {'allowed_user_ids': [OWNER]}})
    fake = FakeTransport()
    controller = Controller(cfg, fake)
    from test_transcribe import make
    voice = make()
    voice.start(background=False)
    tg.run_bot(cfg, controller, TOKEN, transcriber=voice)
    handlers = captured['app'].handlers[0]
    by_kind = {}
    for h in handlers:
        if isinstance(h, ext.CommandHandler):
            for c in h.commands:
                by_kind[c] = h.callback
        elif isinstance(h, ext.CallbackQueryHandler):
            by_kind['button'] = h.callback
        elif isinstance(h, ext.MessageHandler):
            by_kind[h.callback.__name__.replace('on_', '')] = h.callback
    return SimpleNamespace(handlers=by_kind, fake=fake, bot=Bot(), kwargs=captured['kwargs'],
                           application=captured['app'], controller=controller, voice=voice)


def message_update(user_id, text, age_s=0.0, chat_id=5, voice=None):
    replies = Replies()

    async def reply_text(t, reply_markup=None):
        replies.append(t)
        replies.markups = getattr(replies, 'markups', []) + [reply_markup]
        return SimpleNamespace(message_id=900 + len(replies), chat_id=chat_id)

    msg = SimpleNamespace(date=datetime.now(timezone.utc) - timedelta(seconds=age_s),
                          text=text, reply_text=reply_text, voice=voice, audio=None)
    upd = SimpleNamespace(effective_user=SimpleNamespace(id=user_id, username='u'),
                          effective_message=msg, effective_chat=SimpleNamespace(id=chat_id))
    return upd, replies


class Replies(list):
    markups = []


def button_update(user_id, data, message_id, chat_id=5):
    answers = []

    async def answer(text=None, show_alert=False):
        answers.append(text)

    q = SimpleNamespace(data=data, answer=answer,
                        message=SimpleNamespace(chat_id=chat_id, message_id=message_id))
    upd = SimpleNamespace(effective_user=SimpleNamespace(id=user_id, username='u'),
                          callback_query=q, effective_message=None,
                          effective_chat=SimpleNamespace(id=chat_id))
    return upd, answers


def run(coro):
    return asyncio.run(coro)


def test_polling_setup(app):
    assert app.kwargs['drop_pending_updates'] is True
    assert set(app.kwargs['allowed_updates']) == {'message', 'callback_query'}
    assert {'start', 'panel', 'estop', 'button'} <= set(app.handlers)
    assert app.application.concurrent_updates


def test_panel_for_the_owner_only(app):
    ctx = SimpleNamespace(bot=app.bot)
    upd, _ = message_update(999, '/panel')
    run(app.handlers['panel'](upd, ctx))
    assert app.bot.sent == []
    upd, _ = message_update(OWNER, '/panel')
    run(app.handlers['panel'](upd, ctx))
    assert len(app.bot.sent) == 1 and '🟢 online' in app.bot.sent[0][1]
    labels = [b.text for row in app.bot.sent[0][2].inline_keyboard for b in row]
    assert labels == ['🔌 Connect', '🔄']


def test_stale_message_is_dropped(app):
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, '/estop', age_s=30)
    run(app.handlers['estop'](upd, ctx))
    assert app.fake.calls == [] and 'Dropped' in replies[0]


def test_buttons_edit_the_one_panel(app):
    ctx = SimpleNamespace(bot=app.bot)
    upd, _ = message_update(OWNER, '/panel')
    run(app.handlers['panel'](upd, ctx))
    panel_id = app.bot.next_id
    upd, answers = button_update(OWNER, 'connect', panel_id)
    run(app.handlers['button'](upd, ctx))
    assert ['connect'] in app.fake.calls
    assert app.bot.edits and all(e[1] == panel_id for e in app.bot.edits)
    assert len(app.bot.sent) == 1
    labels = [b.text for row in app.bot.edits[-1][3].inline_keyboard for b in row]
    assert '🛑 EMERGENCY' in labels and '▶️ Stack' in labels


def test_button_of_an_old_panel_is_refused(app):
    ctx = SimpleNamespace(bot=app.bot)
    upd, _ = message_update(OWNER, '/panel')
    run(app.handlers['panel'](upd, ctx))
    upd, answers = button_update(OWNER, 'killall', app.bot.next_id - 7)
    run(app.handlers['button'](upd, ctx))
    assert app.fake.calls == [] and 'Old panel' in answers[0]


def test_button_from_a_stranger_does_nothing(app):
    ctx = SimpleNamespace(bot=app.bot)
    upd, _ = message_update(OWNER, '/panel')
    run(app.handlers['panel'](upd, ctx))
    upd, answers = button_update(999, 'estop', app.bot.next_id)
    run(app.handlers['button'](upd, ctx))
    assert app.fake.calls == [] and answers == [None]


def test_estop_command(app):
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, '/estop')
    run(app.handlers['estop'](upd, ctx))
    assert app.fake.calls == [['estop']] and 'car held ✅' in replies[0]
    assert len(replies[0].splitlines()) == 4   # result + one line per step


def test_requests_are_ipv4_with_the_configured_timeout():
    pytest.importorskip('telegram.request')
    httpx = pytest.importorskip('httpx')
    cfg = config.from_dict({'telegram': {'timeout_s': 12}})
    req = tg.make_request(cfg, 32)
    assert req.read_timeout == 12
    transport = req._client._transport
    assert isinstance(transport, httpx.AsyncHTTPTransport)
    assert transport._pool._local_address == '0.0.0.0'
    plain = tg.make_request(config.from_dict({'telegram': {'force_ipv4': False}}), 1)
    assert getattr(plain._client._transport._pool, '_local_address', None) is None


def _buttons(markup):
    return [b.callback_data for row in markup.inline_keyboard for b in row]


def _plan_and_offer(app, monkeypatch, text='vai dritto per due metri'):
    """Text → Plan? → 🧭 Plan: return (callback data of ▶️ Start, message id)."""
    from f1tenth_telegram import planner
    monkeypatch.setattr(planner, 'POLL_S', 0)
    app.fake.state['planner'] = 'running'
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, text)
    run(app.handlers['text'](upd, ctx))
    assert replies[-1] == f'Plan: «{text}»?'
    go, no = _buttons(replies.markups[-1])
    assert go.startswith('pl:go:') and no.startswith('pl:no:')
    run(app.handlers['button'](button_update(OWNER, go, message_id=901)[0], ctx))
    final = app.bot.edits[-1]
    return final, ctx


def test_text_plans_but_never_starts_by_itself(app, monkeypatch):
    final, ctx = _plan_and_offer(app, monkeypatch)
    assert app.fake.sent == ['vai dritto per due metri']
    assert final[1] == 901 and app.bot.parse_modes[-1] == 'HTML'
    assert '🧭 Plan for «vai dritto per due metri»' in final[2]
    assert '<blockquote expandable>' in final[2]
    start, discard = _buttons(final[3])
    assert start.startswith('ms:offer:') and discard.startswith('ms:drop:')
    assert not any(c[0] in ('mission-load', 'mission-start') for c in app.fake.calls)


def test_start_needs_the_second_confirmation(app, monkeypatch):
    final, ctx = _plan_and_offer(app, monkeypatch)
    start = _buttons(final[3])[0]
    run(app.handlers['button'](button_update(OWNER, start, message_id=901)[0], ctx))
    chat, question, markup = app.bot.sent[-1]
    assert question.startswith('🚗 The car will drive. Start?')
    yes, no = _buttons(markup)
    assert yes.startswith('ms:go:') and no.startswith('ms:no:')
    assert not any(c[0] == 'mission-start' for c in app.fake.calls)
    run(app.handlers['button'](button_update(OWNER, yes, message_id=app.bot.next_id)[0], ctx))
    assert ['mission-start', '--id', '20261020T153012-abc123'] in app.fake.calls
    assert any('started: it begins after the 15 s countdown' in (e[2] or '')
               for e in app.bot.edits)


def test_no_and_discard_start_nothing(app, monkeypatch):
    final, ctx = _plan_and_offer(app, monkeypatch)
    start, discard = _buttons(final[3])
    run(app.handlers['button'](button_update(OWNER, start, message_id=901)[0], ctx))
    no = _buttons(app.bot.sent[-1][2])[1]
    run(app.handlers['button'](button_update(OWNER, no, message_id=app.bot.next_id)[0], ctx))
    run(app.handlers['button'](button_update(OWNER, discard, message_id=901)[0], ctx))
    upd, answers = button_update(OWNER, start, message_id=901)
    run(app.handlers['button'](upd, ctx))
    assert 'discarded or replaced' in answers[-1]
    assert not any(c[0] in ('mission-load', 'mission-start') for c in app.fake.calls)


def test_plan_question_expires(app, monkeypatch):
    from f1tenth_telegram import guard
    app.fake.state['planner'] = 'running'
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, 'gira a destra')
    run(app.handlers['text'](upd, ctx))
    go = _buttons(replies.markups[-1])[0]
    monkeypatch.setattr(guard, 'is_stale', lambda *a: True)
    run(app.handlers['button'](button_update(OWNER, go, message_id=901)[0], ctx))
    assert app.fake.sent == [] and app.bot.edits[-1][2].startswith('⌛ Not planned')


def test_plan_cancelled(app):
    app.fake.state['planner'] = 'running'
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, 'gira a destra')
    run(app.handlers['text'](upd, ctx))
    no = _buttons(replies.markups[-1])[1]
    run(app.handlers['button'](button_update(OWNER, no, message_id=901)[0], ctx))
    assert app.fake.sent == [] and app.bot.edits[-1][2] == '❌ Not planned.'


def test_text_when_the_planner_is_not_running_sends_a_panel(app):
    app.controller.handle('connect')
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, 'go ahead')
    run(app.handlers['text'](upd, ctx))
    assert app.fake.sent == [] and replies[-1].startswith('📝 The planner is not running')
    chat, text, markup = app.bot.sent[-1]
    assert 'planner not running: tap ▶️ Planner' in text
    assert 'start:planner' in _buttons(markup)


def test_bad_foreign_or_stale_text_is_not_planned(app):
    app.fake.state['planner'] = 'running'
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, 'x' * 501)
    run(app.handlers['text'](upd, ctx))
    assert 'at most 500' in replies[-1]
    upd, replies = message_update(999, 'go ahead')
    run(app.handlers['text'](upd, ctx))
    assert replies == []
    upd, replies = message_update(OWNER, 'go ahead', age_s=30)
    run(app.handlers['text'](upd, ctx))
    assert replies[-1].startswith('Dropped')
    app.fake.mission = {'state': 'running', 'id': 'm1'}
    upd, replies = message_update(OWNER, 'go ahead')
    run(app.handlers['text'](upd, ctx))
    assert replies[-1].startswith('⏳ Not planned: a mission is running (m1)')
    assert app.fake.sent == []


def _voice(duration=3):
    async def download_as_bytearray():
        return bytearray(b'OggS')

    async def get_file():
        return SimpleNamespace(download_as_bytearray=download_as_bytearray)
    return SimpleNamespace(duration=duration, get_file=get_file)


def test_voice_is_transcribed_into_the_same_plan_question(app):
    app.fake.state['planner'] = 'running'
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, None, voice=_voice())
    run(app.handlers['voice'](upd, ctx))
    assert replies == ['🎙 transcribing…']
    edited = app.bot.edits[-1]
    assert edited[2].startswith('🎙 «vai dritto per due metri»\n(wrong? type the command')
    assert 'Plan: «vai dritto per due metri»?' in edited[2]
    assert _buttons(edited[3])[0].startswith('pl:go:')
    assert app.fake.sent == []


def test_voice_too_long_or_model_not_ready(app):
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, None, voice=_voice(duration=75))
    run(app.handlers['voice'](upd, ctx))
    assert replies == ['🎙 the audio is 75 s long, at most 60 s.']
    app.voice.state = 'loading'
    upd, replies = message_update(OWNER, None, voice=_voice())
    run(app.handlers['voice'](upd, ctx))
    assert 'still loading' in replies[-1]
    app.voice.state, app.voice.reason = 'unavailable', 'ImportError: faster_whisper'
    upd, replies = message_update(OWNER, None, voice=_voice())
    run(app.handlers['voice'](upd, ctx))
    assert replies[-1] == '🎙 transcription unavailable on this bot.'
