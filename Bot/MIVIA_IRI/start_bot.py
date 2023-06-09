"""
#doc: https://python-telegram-bot.readthedocs.io/en/stable/
#github rep: https://github.com/python-telegram-bot/python-telegram-bot

https://pythontelegramrobot.readthedocs.io/en/latest/telegram.inlinekeyboardbutton.html#

python3 start_bot.py 2>&1 | tee log_file.txt
"""

import os
import time
from datetime import datetime, timedelta
import logging

import telegram.error
from telegram.ext import Updater, CommandHandler, MessageHandler, ConversationHandler, CallbackQueryHandler, Filters

from utils import callbacks
import utils.states as STATES
from utils.utils import get_curr_dir, SchedulerTimer, error_handler, is_connected

#TOKEN = r"1905358159:AAHLRxV694LmBhtb6GH3aqCoG9vc3v4nd7Y" #Telegram bot token
TOKEN = "6272694271:AAFw-2nWm4zg2hOY7tD0Rp2wZ58B8CZxKIk"    # Beis token (MIVIA-IRI)

def main():

    def create_bot(conv_handler:ConversationHandler):
        updater = Updater(TOKEN)
        updater.dispatcher.add_handler(conv_handler)
        updater.dispatcher.add_handler(CommandHandler(command='reset', callback=callbacks.reset))
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
    states[STATES.PRESENTATION] = [CallbackQueryHandler(callback=callbacks.presentation)]   # Handler to be modified
    states[STATES.INTERACTION_LANGUAGE] = [CallbackQueryHandler(callback=callbacks.get_int_lang)]
    states[STATES.GDPR] = [CallbackQueryHandler(callback=callbacks.get_gdpr)]
    states[STATES.GENDER] = [CallbackQueryHandler(callback=callbacks.get_gender)]
    states[STATES.AGE] = [CallbackQueryHandler(callback=callbacks.get_age)]
    states[STATES.ACQUISITION_LANGUAGES] = [CallbackQueryHandler(callback=callbacks.get_acq_langs)]
    states[STATES.START_ACQUISITION] = [CallbackQueryHandler(callback=callbacks.start_acquisition)]
    states[STATES.INTENT_ACQUISITION] = [MessageHandler(filters=Filters.voice, callback=callbacks.get_intent)]    # Replace with MessageHandler using a voice filter
  
    conv_handler = ConversationHandler(
        entry_points=[CommandHandler(command="start", callback=callbacks.presentation),
                      CallbackQueryHandler(callbacks.conversation_handler),
                      MessageHandler(filters=Filters.voice, callback=callbacks.get_intent)],
        states=states,
        fallbacks=[CommandHandler('reset', callbacks.reset)]
    )

    # Start the Bot
    # Create the Updater and pass it your bot's token.
    updater = create_bot(conv_handler=conv_handler)
    logger.info(datetime.now().strftime("%d-%m-%Y %H:%M"))

    try:
        updater.start_polling()
    except telegram.error.NetworkError:
        while not is_connected(updater):
            updater = create_bot(conv_handler=conv_handler)
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


