"""
The Telegram front end: python-telegram-bot (v21+), long polling.

One status message per chat (/panel or /start sends it), edited in place
with edit_message_text after every action. /estop works without the panel.
Updates are handled concurrently so 🛑 is never stuck behind a running
killall; the controller still runs one other action at a time.

Any other plain text is a command for the LLM planner (planner.py): asked
for confirmation, sent, then the planner's answer is followed in one message
that is edited as its output arrives.

Imported only when the bot really talks to Telegram: --cli does not need
the library.
"""

import asyncio
import dataclasses
import logging
import time

from . import planner, view
from .config import Config
from .controller import Controller, summary_estop
from .guard import allowed, Confirmations, is_stale

log = logging.getLogger('f1tenth_telegram')


def make_request(cfg: Config, pool_size: int):
    """
    Build an HTTPXRequest for the Bot API (python-telegram-bot 22.x).

    One timeout for connect, read, write and pool. With force_ipv4, an httpx
    transport bound to 0.0.0.0, so every connection is IPv4: linus's IPv6
    route is broken, and getMe timed out on it. A custom transport ignores
    the client's own limits, so the pool size is set on it too.
    """
    from telegram.request import HTTPXRequest
    t = cfg.timeout_s
    kwargs = {}
    if cfg.force_ipv4:
        import httpx
        kwargs['httpx_kwargs'] = {'transport': httpx.AsyncHTTPTransport(
            local_address='0.0.0.0', limits=httpx.Limits(max_connections=pool_size))}
    return HTTPXRequest(connection_pool_size=pool_size, connect_timeout=t, read_timeout=t,
                        write_timeout=t, pool_timeout=t, **kwargs)


def startup_line(cfg: Config) -> str:
    """Return the INFO line logged at start: never the token, never the user ids."""
    return (f'starting: ipv4 forced {"yes" if cfg.force_ipv4 else "no"}, '
            f'timeout {cfg.timeout_s:g} s, target {cfg.target.name} ({cfg.target.host}), '
            f'{len(cfg.allowed_user_ids)} whitelisted user(s)')


def run_bot(cfg: Config, controller: Controller, token: str) -> None:
    from telegram import InlineKeyboardButton, InlineKeyboardMarkup, Update
    from telegram.error import BadRequest
    from telegram.ext import (Application, CallbackQueryHandler, CommandHandler,
                              filters, MessageHandler)

    panels = {}          # chat id -> message id of the current panel
    # Planner texts waiting for ✅ Send; they expire like any confirmation.
    plan_confirms = Confirmations(cfg.max_command_age_s)
    plan_busy = {'text': None}   # one planner command followed at a time

    def markup(rows):
        return InlineKeyboardMarkup(
            [[InlineKeyboardButton(b.label, callback_data=b.action) for b in row]
             for row in rows])

    async def show(bot, chat_id, panel, new=False):
        text, rows = view.render(panel)
        if not new and chat_id in panels:
            try:
                await bot.edit_message_text(text, chat_id=chat_id, message_id=panels[chat_id],
                                            reply_markup=markup(rows))
                return
            except BadRequest as exc:
                if 'not modified' in str(exc).lower():
                    return
                log.warning('edit failed (%s): sending a new panel', exc)
        msg = await bot.send_message(chat_id, text, reply_markup=markup(rows))
        panels[chat_id] = msg.message_id

    async def edit(bot, chat_id, message_id, text, rows=None, parse_mode=None):
        try:
            await bot.edit_message_text(text, chat_id=chat_id, message_id=message_id,
                                        reply_markup=markup(rows) if rows else None,
                                        parse_mode=parse_mode)
        except BadRequest as exc:
            if 'not modified' not in str(exc).lower():
                log.warning('edit failed: %s', exc)

    def permitted(update) -> bool:
        user = update.effective_user
        if allowed(user.id if user else None, cfg.allowed_user_ids):
            return True
        log.warning('ignored an update from user id %s (@%s): not in allowed_user_ids',
                    user.id if user else None, user.username if user else None)
        return False

    def fresh(update) -> bool:
        msg = update.effective_message
        if msg is None or msg.date is None:
            return True
        if is_stale(msg.date.timestamp(), time.time(), cfg.max_command_age_s):
            log.warning('dropped "%s": sent %.0f s ago (max %g s)', msg.text,
                        time.time() - msg.date.timestamp(), cfg.max_command_age_s)
            return False
        return True

    async def cmd_panel(update, context):
        if not permitted(update):
            return
        if not fresh(update):
            await update.effective_message.reply_text(
                f'Dropped: older than {cfg.max_command_age_s:g} s.')
            return
        log.info('user %s: /panel', update.effective_user.id)
        panel = await asyncio.to_thread(controller.handle, 'refresh')
        await show(context.bot, update.effective_chat.id, panel, new=True)

    async def cmd_estop(update, context):
        if not permitted(update):
            return
        if not fresh(update):
            await update.effective_message.reply_text(
                f'Dropped: older than {cfg.max_command_age_s:g} s. Send /estop again.')
            return
        log.warning('user %s: /estop', update.effective_user.id)
        panel = await asyncio.to_thread(controller.handle, 'estop')
        await update.effective_message.reply_text(panel.last or summary_estop({}))
        if update.effective_chat.id in panels:
            await show(context.bot, update.effective_chat.id, panel)

    async def on_text(update, context):
        """Treat a plain text message as a planner command, after a confirmation."""
        if not permitted(update):
            return
        msg = update.effective_message
        if not fresh(update):
            await msg.reply_text(f'Dropped: older than {cfg.max_command_age_s:g} s. '
                                 'Send it again.')
            return
        text, why = planner.validate(msg.text)
        if why:
            await msg.reply_text(f'❌ Not sent to the planner: {why}.')
            return
        res = await asyncio.to_thread(controller.t.ctl, ['planner-log'], cfg.timeout('status'))
        if res.get('transport_error') or not res.get('ok'):
            await msg.reply_text(f'❌ Cannot reach the planner: {res.get("error")}')
            return
        if not res.get('running'):
            # A fresh panel, with ▶️ Planner on it once connected.
            panel = await asyncio.to_thread(controller.handle, 'status')
            controller.note('📝 planner not running: tap ▶️ Planner, then send the command '
                            'again')
            await show(context.bot, update.effective_chat.id, panel, new=True)
            return
        nonce = plan_confirms.ask(text)
        await msg.reply_text(
            f'Send to planner: «{text}»?',
            reply_markup=markup([[view.Button('✅ Send', f'plan:send:{nonce}'),
                                  view.Button('❌ Cancel', f'plan:cancel:{nonce}')]]))

    async def on_plan_button(update, context):
        """Handle ✅ Send / ❌ Cancel under a planner command, then follow the answer."""
        q = update.callback_query
        _, verb, nonce = (q.data.split(':', 2) + ['', ''])[:3]
        chat_id, mid = q.message.chat_id, q.message.message_id
        if verb != 'send':
            plan_confirms.cancel(nonce)
            await q.answer('Cancelled')
            await edit(context.bot, chat_id, mid, '❌ Not sent to the planner.')
            return
        text, why = plan_confirms.take(nonce)
        if text is None:
            await q.answer()
            await edit(context.bot, chat_id, mid, f'⌛ Not sent: {why}.')
            return
        if plan_busy['text'] is not None:
            await q.answer()
            await edit(context.bot, chat_id, mid, '⏳ Not sent: the planner is still answering '
                       f'«{plan_busy["text"]}». Send it again afterwards.')
            return
        await q.answer('Sending…')
        plan_busy['text'] = text
        try:
            ex = planner.Exchange(text)
            if await asyncio.to_thread(ex.send, controller.t, cfg.timeout('default')):
                log.info('user %s: planner command %r', update.effective_user.id, text)
            else:
                log.warning('user %s: planner command %r NOT sent: %s',
                            update.effective_user.id, text, ex.error)
            await edit(context.bot, chat_id, mid, ex.render(), parse_mode='HTML')
            while not ex.done:
                await asyncio.sleep(planner.POLL_S)
                await asyncio.to_thread(ex.poll, controller.t, cfg.timeout('status'))
                # Every poll: the elapsed time on the message moves too.
                await edit(context.bot, chat_id, mid, ex.render(), parse_mode='HTML')
        finally:
            plan_busy['text'] = None

    async def on_button(update, context):
        q = update.callback_query
        if not permitted(update):
            await q.answer()
            return
        if (q.data or '').startswith('plan:'):
            await on_plan_button(update, context)
            return
        chat_id = q.message.chat_id
        if panels.get(chat_id) != q.message.message_id:
            await q.answer('Old panel: use the newest one, or send /panel.', show_alert=True)
            return
        action = q.data
        await q.answer()
        log.info('user %s: %s', update.effective_user.id, action)
        idle = controller.panel.busy is None
        # The action starts first (an e-stop must not wait for an edit), then
        # the panel shows it as running.
        work = asyncio.ensure_future(asyncio.to_thread(controller.handle, action))
        if idle or action == 'estop':
            await show(context.bot, chat_id,
                       dataclasses.replace(controller.panel, busy=action, confirm=None))
        panel = await work
        await show(context.bot, chat_id, panel)

    # httpx logs every request URL at INFO, and a Bot API URL contains the
    # token: keep it out of the journal.
    logging.getLogger('httpx').setLevel(logging.WARNING)
    log.info(startup_line(cfg))
    app = (Application.builder().token(token)
           .request(make_request(cfg, 32))
           # Long polling holds one connection; its read timeout gets the poll
           # timeout added by get_updates itself.
           .get_updates_request(make_request(cfg, 1))
           .concurrent_updates(True).build())
    app.add_handler(CommandHandler(['start', 'panel'], cmd_panel))
    app.add_handler(CommandHandler('estop', cmd_estop))
    app.add_handler(CallbackQueryHandler(on_button))
    app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, on_text))
    if not cfg.allowed_user_ids:
        log.warning('allowed_user_ids is empty: every update is ignored, and logged '
                    'with its user id')
    # Pending updates are dropped: nothing sent while the bot was down runs.
    # bootstrap_retries=-1: getMe and the start of polling are retried forever,
    # so a network that is not up yet at boot does not end the service.
    app.run_polling(drop_pending_updates=True, bootstrap_retries=-1,
                    allowed_updates=[Update.MESSAGE, Update.CALLBACK_QUERY])
