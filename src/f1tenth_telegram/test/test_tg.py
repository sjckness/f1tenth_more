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

    async def edit_message_text(self, text, chat_id, message_id, reply_markup=None):
        self.edits.append((chat_id, message_id, text, reply_markup))


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
        else:
            by_kind['button'] = h.callback
    return SimpleNamespace(handlers=by_kind, fake=fake, bot=Bot(), kwargs=captured['kwargs'],
                           application=captured['app'])


def message_update(user_id, text, age_s=0.0, chat_id=5):
    replies = []

    async def reply_text(t):
        replies.append(t)

    msg = SimpleNamespace(date=datetime.now(timezone.utc) - timedelta(seconds=age_s),
                          text=text, reply_text=reply_text)
    upd = SimpleNamespace(effective_user=SimpleNamespace(id=user_id, username='u'),
                          effective_message=msg, effective_chat=SimpleNamespace(id=chat_id))
    return upd, replies


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
