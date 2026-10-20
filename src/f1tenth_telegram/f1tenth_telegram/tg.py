"""
The Telegram front end: python-telegram-bot (v21+), long polling.

One status message per chat (/panel or /start sends it), edited in place
with edit_message_text after every action. /estop works without the panel.
Updates are handled concurrently so 🛑 is never stuck behind a running
killall; the controller still runs one other action at a time.

Any other plain text, or a voice message once transcribed (transcribe.py),
is a command for the LLM planner, in two steps (flow.py): "Plan: «…»?" →
the planner, plan-only, answers in one message edited as its output arrives
→ the plan with [▶️ Start mission] → "🚗 The car will drive. Start?" →
mission-load + mission-start. Never a start straight from a message.

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
from .flow import Flow
from .guard import allowed, is_stale
from .transcribe import Transcriber

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


def run_bot(cfg: Config, controller: Controller, token: str,
            transcriber: Transcriber = None) -> None:
    from telegram import InlineKeyboardButton, InlineKeyboardMarkup, Update
    from telegram.error import BadRequest
    from telegram.ext import (Application, CallbackQueryHandler, CommandHandler,
                              filters, MessageHandler)

    panels = {}          # chat id -> message id of the current panel
    flow = Flow(controller.t, cfg)
    transcriber = transcriber or Transcriber(cfg.transcription)

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
        """Edit a message; "not modified" is not an error."""
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

    async def propose(bot, chat_id, text, reply=None, message_id=None, prefix=''):
        """Step 1: checks, then "Plan: «text»?" (a reply, or an edit of message_id)."""
        r = await asyncio.to_thread(flow.propose, text)
        rows = None
        if r['kind'] == 'not_running':
            panel = await asyncio.to_thread(controller.handle, 'status')
            controller.note('📝 planner not running: tap ▶️ Planner, then send the command '
                            'again')
            await show(bot, chat_id, panel, new=True)
            body = '📝 The planner is not running: see the panel.'
        elif r['kind'] == 'ask':
            body = f'Plan: «{r["text"]}»?'
            rows = [[view.Button('🧭 Plan', f'pl:go:{r["nonce"]}'),
                     view.Button('❌ Cancel', f'pl:no:{r["nonce"]}')]]
        elif r['kind'] == 'invalid':
            body = f'❌ Not a planner command: {r["why"]}.'
        elif r['kind'] == 'refused':
            body = f'⏳ Not planned: {r["why"]}.'
        else:
            body = f'❌ Cannot reach the planner: {r.get("why")}'
        body = prefix + body
        if message_id is not None:
            await edit(bot, chat_id, message_id, body, rows)
        elif reply is not None:
            await reply(body, reply_markup=markup(rows) if rows else None)

    async def on_text(update, context):
        """Treat a plain text message as a planner command (two steps)."""
        if not permitted(update):
            return
        msg = update.effective_message
        if not fresh(update):
            await msg.reply_text(f'Dropped: older than {cfg.max_command_age_s:g} s. '
                                 'Send it again.')
            return
        await propose(context.bot, update.effective_chat.id, msg.text, reply=msg.reply_text)

    async def on_voice(update, context):
        """Transcribe a voice note or an audio file, then the same two steps."""
        if not permitted(update):
            return
        msg = update.effective_message
        if not fresh(update):
            await msg.reply_text(f'Dropped: older than {cfg.max_command_age_s:g} s. '
                                 'Send it again.')
            return
        audio = msg.voice or msg.audio
        why = transcriber.unavailable_message() or transcriber.too_long(
            getattr(audio, 'duration', None))
        if why:
            await msg.reply_text(f'🎙 {why}.')
            return
        status = await msg.reply_text('🎙 transcribing…')
        tg_file = await audio.get_file()
        data = bytes(await tg_file.download_as_bytearray())
        suffix = '.ogg' if msg.voice else ('.' + (audio.file_name or 'a.ogg').rsplit('.', 1)[-1])

        def write(path):
            with open(path, 'wb') as f:
                f.write(data)
        try:
            res = await asyncio.to_thread(transcriber.transcribe_bytes, write, suffix)
        except Exception as exc:  # noqa: BLE001 - a bad file must not reach the user raw
            log.warning('user %s: transcription failed: %s', update.effective_user.id, exc)
            await edit(context.bot, status.chat_id, status.message_id,
                       '🎙 Could not transcribe it: type the command as text.')
            return
        finally:
            data = None
        log.info('user %s: voice %.1f s, transcribed in %.2f s: %r', update.effective_user.id,
                 res['duration_s'], res['transcribe_s'], res['text'])
        why = transcriber.too_long(res['duration_s'])
        if why:
            await edit(context.bot, status.chat_id, status.message_id, f'🎙 {why}.')
            return
        await propose(context.bot, status.chat_id, res['text'], message_id=status.message_id,
                      prefix=f'🎙 «{res["text"]}»\n(wrong? type the command as text)\n\n')

    async def on_plan_button(update, context, verb, nonce):
        """Step 2: 🧭 Plan / ❌ Cancel. The planner answers in this same message."""
        q = update.callback_query
        chat_id, mid = q.message.chat_id, q.message.message_id
        if verb != 'go':
            flow.cancel_plan(nonce)
            await q.answer('Cancelled')
            await edit(context.bot, chat_id, mid, '❌ Not planned.')
            return
        await q.answer('Planning…')
        ex, why = await asyncio.to_thread(flow.begin_plan, nonce)
        if ex is None:
            await edit(context.bot, chat_id, mid, f'⌛ Not planned: {why}.')
            return
        log.info('user %s: planner command %r%s', update.effective_user.id, ex.text,
                 '' if ex.error is None else f' NOT sent: {ex.error}')
        offer = None
        try:
            await edit(context.bot, chat_id, mid, ex.render(), parse_mode='HTML')
            while not ex.done:
                await asyncio.sleep(planner.POLL_S)
                await asyncio.to_thread(ex.poll, controller.t, cfg.timeout('status'))
                await edit(context.bot, chat_id, mid, ex.render(), parse_mode='HTML')
        finally:
            offer = flow.finish_plan(ex)
        if offer is None:
            await edit(context.bot, chat_id, mid, planner.render_failure(ex), parse_mode='HTML')
            return
        log.info('user %s: plan %s for %r', update.effective_user.id, offer.id, offer.text)
        await edit(context.bot, chat_id, mid, planner.render_offer(offer.text, offer.entry),
                   [[view.Button('▶️ Start mission', f'ms:offer:{offer.token}'),
                     view.Button('❌ Discard', f'ms:drop:{offer.token}')]], parse_mode='HTML')

    async def on_mission_button(update, context, verb, arg):
        """Handle steps 3 and 4: ▶️ Start → "🚗 … Start?" → load + start."""
        q = update.callback_query
        chat_id, mid = q.message.chat_id, q.message.message_id
        if verb == 'drop':
            flow.discard(arg)
            await q.answer('Discarded')
            await context.bot.edit_message_reply_markup(chat_id=chat_id, message_id=mid,
                                                        reply_markup=None)
            return
        if verb == 'offer':
            r = flow.offer_start(arg)
            if r['kind'] != 'ask':
                await q.answer(r['why'], show_alert=True)
                if r['kind'] in ('expired', 'gone'):
                    await context.bot.edit_message_reply_markup(
                        chat_id=chat_id, message_id=mid, reply_markup=None)
                return
            await q.answer()
            offer = r['offer']
            sm = offer.entry['stored'].get('summary') or {}
            await context.bot.send_message(
                chat_id, f'🚗 The car will drive. Start?\n{offer.id}: «{offer.text}», '
                f"{sm.get('n_moves')} move(s), max {planner._fmt_speed(sm.get('max_speed_mps'))}",
                reply_markup=markup([[view.Button('✅ Yes, start', f'ms:go:{r["nonce"]}'),
                                      view.Button('❌ No', f'ms:no:{r["nonce"]}')]]))
            return
        if verb == 'no':
            flow.cancel_start(arg)
            await q.answer('Not started')
            await edit(context.bot, chat_id, mid, '❌ Not started.')
            return
        await q.answer('Starting…')
        res = await asyncio.to_thread(flow.start, arg)
        log.warning('user %s: mission start %s: %s', update.effective_user.id,
                    res.get('id'), 'started' if res['ok'] else f'refused: {res["why"]}')
        if res['ok']:
            text = (f'🚗 Mission {res["id"]} started: it begins after the '
                    f'{res.get("countdown_s") or 15:g} s countdown. 🛑 EMERGENCY is on '
                    'the panel.')
        else:
            text = f'❌ Not started: {res["why"]}'
        await edit(context.bot, chat_id, mid, text)
        if res['ok']:
            panel = await asyncio.to_thread(controller.handle, 'status')
            await show(context.bot, chat_id, panel, new=chat_id not in panels)

    async def on_button(update, context):
        q = update.callback_query
        if not permitted(update):
            await q.answer()
            return
        prefix, _, rest = (q.data or '').partition(':')
        if prefix in ('pl', 'ms'):
            verb, _, arg = rest.partition(':')
            handler = on_plan_button if prefix == 'pl' else on_mission_button
            await handler(update, context, verb, arg)
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
    app.add_handler(MessageHandler(filters.VOICE | filters.AUDIO, on_voice))
    # The speech model loads in the background: start-up is not blocked.
    transcriber.start()
    if not cfg.allowed_user_ids:
        log.warning('allowed_user_ids is empty: every update is ignored, and logged '
                    'with its user id')
    # Pending updates are dropped: nothing sent while the bot was down runs.
    # bootstrap_retries=-1: getMe and the start of polling are retried forever,
    # so a network that is not up yet at boot does not end the service.
    app.run_polling(drop_pending_updates=True, bootstrap_retries=-1,
                    allowed_updates=[Update.MESSAGE, Update.CALLBACK_QUERY])
