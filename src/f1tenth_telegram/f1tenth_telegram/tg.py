"""
The Telegram front end: python-telegram-bot (v21+), long polling.

One status message per chat (/panel or /start sends it), edited in place
with edit_message_text after every action. /estop works without the panel.
Updates are handled concurrently so 🛑 is never stuck behind a running
killall; the controller still runs one other action at a time.

Imported only when the bot really talks to Telegram: --cli does not need
the library.
"""

import asyncio
import dataclasses
import logging
import time

from . import view
from .config import Config
from .controller import Controller, summary_estop
from .guard import allowed, is_stale

log = logging.getLogger('f1tenth_telegram')


def run_bot(cfg: Config, controller: Controller, token: str) -> None:
    from telegram import InlineKeyboardButton, InlineKeyboardMarkup, Update
    from telegram.error import BadRequest
    from telegram.ext import Application, CallbackQueryHandler, CommandHandler

    panels = {}          # chat id -> message id of the current panel

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

    async def on_button(update, context):
        q = update.callback_query
        if not permitted(update):
            await q.answer()
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

    app = Application.builder().token(token).concurrent_updates(True).build()
    app.add_handler(CommandHandler(['start', 'panel'], cmd_panel))
    app.add_handler(CommandHandler('estop', cmd_estop))
    app.add_handler(CallbackQueryHandler(on_button))
    if not cfg.allowed_user_ids:
        log.warning('allowed_user_ids is empty: every update is ignored, and logged '
                    'with its user id')
    log.info('polling as target %s (%s)', cfg.target.name, cfg.target.host)
    # Pending updates are dropped: nothing sent while the bot was down runs.
    app.run_polling(drop_pending_updates=True,
                    allowed_updates=[Update.MESSAGE, Update.CALLBACK_QUERY])
