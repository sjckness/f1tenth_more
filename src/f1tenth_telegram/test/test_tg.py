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
        return SimpleNamespace(message_id=self.next_id)

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
    tg.run_bot(cfg, controller, TOKEN)
    handlers = captured['app'].handlers[0]
    by_kind = {}
    for h in handlers:
        if isinstance(h, ext.CommandHandler):
            for c in h.commands:
                by_kind[c] = h.callback
        elif isinstance(h, ext.CallbackQueryHandler):
            by_kind['button'] = h.callback
        elif isinstance(h, ext.MessageHandler):
            by_kind['text'] = h.callback
    return SimpleNamespace(handlers=by_kind, fake=fake, bot=Bot(), kwargs=captured['kwargs'],
                           application=captured['app'], controller=controller)


def message_update(user_id, text, age_s=0.0, chat_id=5):
    replies = Replies()

    async def reply_text(t, reply_markup=None):
        replies.append(t)
        replies.markups = getattr(replies, 'markups', []) + [reply_markup]
        return SimpleNamespace(message_id=900 + len(replies))

    msg = SimpleNamespace(date=datetime.now(timezone.utc) - timedelta(seconds=age_s),
                          text=text, reply_text=reply_text)
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


def _plan_buttons(replies):
    return [b.callback_data for row in replies.markups[-1].inline_keyboard for b in row]


def test_text_is_a_planner_command_after_a_confirmation(app, monkeypatch):
    from f1tenth_telegram import planner
    monkeypatch.setattr(planner, 'POLL_S', 0)
    app.fake.state['planner'] = 'running'
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, 'go ahead 2 meters')
    run(app.handlers['text'](upd, ctx))
    assert replies[-1] == 'Send to planner: «go ahead 2 meters»?'
    send, cancel = _plan_buttons(replies)
    assert send.startswith('plan:send:') and cancel.startswith('plan:cancel:')
    assert app.fake.sent == []
    upd, answers = button_update(OWNER, send, message_id=901)
    run(app.handlers['button'](upd, ctx))
    assert app.fake.sent == ['go ahead 2 meters']
    final = app.bot.edits[-1]
    assert final[1] == 901 and app.bot.parse_modes[-1] == 'HTML'
    assert '✅ result: ok' in final[2] and '<pre><code class="language-json">' in final[2]
    # The confirmation was used up: a second tap sends nothing.
    run(app.handlers['button'](button_update(OWNER, send, message_id=901)[0], ctx))
    assert app.fake.sent == ['go ahead 2 meters']


def test_planner_confirmation_expires(app, monkeypatch):
    from f1tenth_telegram import guard
    app.fake.state['planner'] = 'running'
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, 'turn left')
    run(app.handlers['text'](upd, ctx))
    send = _plan_buttons(replies)[0]
    monkeypatch.setattr(guard, 'is_stale', lambda *a: True)
    run(app.handlers['button'](button_update(OWNER, send, message_id=901)[0], ctx))
    assert app.fake.sent == [] and app.bot.edits[-1][2].startswith('⌛ Not sent')


def test_planner_command_cancelled(app):
    app.fake.state['planner'] = 'running'
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, 'turn left')
    run(app.handlers['text'](upd, ctx))
    cancel = _plan_buttons(replies)[1]
    run(app.handlers['button'](button_update(OWNER, cancel, message_id=901)[0], ctx))
    assert app.fake.sent == [] and app.bot.edits[-1][2] == '❌ Not sent to the planner.'


def test_text_when_the_planner_is_not_running_sends_a_panel(app):
    app.controller.handle('connect')
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, 'go ahead')
    run(app.handlers['text'](upd, ctx))
    assert replies == [] and app.fake.sent == []
    chat, text, markup = app.bot.sent[-1]
    assert 'planner not running: tap ▶️ Planner' in text
    assert 'start:planner' in [b.callback_data for row in markup.inline_keyboard for b in row]


def test_bad_or_foreign_text_is_not_sent(app):
    app.fake.state['planner'] = 'running'
    ctx = SimpleNamespace(bot=app.bot)
    upd, replies = message_update(OWNER, 'line one\nline two')
    run(app.handlers['text'](upd, ctx))
    assert 'one line only' in replies[-1]
    upd, replies = message_update(OWNER, 'x' * 501)
    run(app.handlers['text'](upd, ctx))
    assert 'at most 500' in replies[-1]
    upd, replies = message_update(999, 'go ahead')
    run(app.handlers['text'](upd, ctx))
    assert replies == [] and app.fake.sent == []
    upd, replies = message_update(OWNER, 'go ahead', age_s=30)
    run(app.handlers['text'](upd, ctx))
    assert replies[-1].startswith('Dropped') and app.fake.sent == []
