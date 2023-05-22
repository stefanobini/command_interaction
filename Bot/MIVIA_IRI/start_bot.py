"""
#doc: https://python-telegram-bot.readthedocs.io/en/stable/
#github rep: https://github.com/python-telegram-bot/python-telegram-bot

https://pythontelegramrobot.readthedocs.io/en/latest/telegram.inlinekeyboardbutton.html#
"""

import os
import time
from datetime import datetime, timedelta
import logging

import telegram.error
from telegram.ext import Updater, CommandHandler, MessageHandler, ConversationHandler, CallbackQueryHandler

from utils import callbacks
import utils.states as STATES
from utils.utils import get_curr_dir, SchedulerTimer, error_handler, is_connected

#TOKEN = r"1905358159:AAHLRxV694LmBhtb6GH3aqCoG9vc3v4nd7Y" #Telegram bot token
TOKEN = "6272694271:AAFw-2nWm4zg2hOY7tD0Rp2wZ58B8CZxKIk"    # Beis token (MIVIA-IRI)

def main():

    def create_bot():
        updater = Updater(TOKEN)
        updater.dispatcher.add_handler(conv_handler)
        updater.dispatcher.add_handler(CommandHandler('reset', callbacks.reset))
        updater.dispatcher.add_error_handler(error_handler)
        return updater

    os.chdir(get_curr_dir(__file__))

    """Enabling logger"""
    logging.basicConfig(format='%(name)s - %(levelname)s - %(message)s', level=logging.INFO)
    logger = logging.getLogger(__name__)
    logging.getLogger('apscheduler.scheduler').setLevel(logging.ERROR)
    logging.getLogger('apscheduler.executors.default').setLevel(logging.ERROR)
    logging.getLogger('telegram.vendor.ptb_urllib3.urllib3.connectionpool').setLevel(logging.ERROR)

    """Define the Conversational State in the interaction between bot and user."""
    states = {}
    states[STATES.HELLO] = [CallbackQueryHandler(callbacks.hello)]
    states[STATES.ASK_INTERACTION_LANGUAGE] = [CallbackQueryHandler(callbacks.ask_int_lang)]
    states[STATES.GET_INTERACTION_LANGUAGE] = [CallbackQueryHandler(callbacks.get_int_lang)]
    states[STATES.ASK_GDPR] = [CallbackQueryHandler(callbacks.ask_gdpr)]
    states[STATES.GET_GDPR] = [CallbackQueryHandler(callbacks.get_gdpr)]
    states[STATES.ASK_GENDER] = [CallbackQueryHandler(callbacks.ask_gender)]
    states[STATES.GET_GENDER] = [CallbackQueryHandler(callbacks.get_gender)]
    states[STATES.ASK_AGE] = [CallbackQueryHandler(callbacks.ask_age)]
    states[STATES.GET_AGE] = [CallbackQueryHandler(callbacks.get_age)]
    states[STATES.ASK_ACQUISITION_LANGUAGES] = [CallbackQueryHandler(callbacks.ask_acq_langs)]
    states[STATES.GET_ACQUISITION_LANGUAGES] = [CallbackQueryHandler(callbacks.get_acq_langs)]
    states[STATES.ACQUISITION_INFO] = [CallbackQueryHandler(callbacks.give_acq_info)]
    states[STATES.START_ACQUISITION] = [CallbackQueryHandler(callbacks.start_acquisition)]
    states[STATES.ASK_INTENT_ACQUISITION] = [CallbackQueryHandler(callbacks.ask_intent)]
    states[STATES.GET_INTENT_ACQUISITION] = [CallbackQueryHandler(callbacks.get_intent)]

    conv_handler = ConversationHandler(
        entry_points=[CommandHandler('start', callbacks.run_bot)],
        states=states,
        fallbacks=[CommandHandler('reset', callbacks.reset)],
        conversation_timeout=None
    )

    # Start the Bot
    # Create the Updater and pass it your bot's token.
    updater = create_bot()
    logger.info(datetime.now().strftime("%d-%m-%Y %H:%M"))

    try:
        updater.start_polling()
    except telegram.error.NetworkError:
        while not is_connected(updater) :
            updater = create_bot()
            time.sleep(60)

    #Start timer
    TIME_CHECK = 45 #minutes
    scheduler_timer = SchedulerTimer(timedelta(minutes=TIME_CHECK).total_seconds(), updater.bot)

    scheduler_timer.start()

    # Run the bot until the user presses Ctrl-C or the process receives SIGINT,
    # SIGTERM or SIGABRT
    updater.idle()
    scheduler_timer.stop()

if __name__ == "__main__":
    main()


