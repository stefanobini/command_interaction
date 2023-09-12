#doc: https://python-telegram-bot.readthedocs.io/en/stable/
#github rep: https://github.com/python-telegram-bot/python-telegram-bot

import logging
import time
import telegram.error
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters, ConversationHandler, CallbackQueryHandler
from callbacks import start, restart_conversation, receive, get_gender, get_age, conv_pause, select_query, get_lang, reset, \
    GENDER_STATE, AGE_STATE, STATE, PAUSE, LANG_STATE
from utils import get_curr_dir, SchedulerTimer, error_handler, is_connected
import os
from datetime import datetime, timedelta

from tqdm import tqdm
import colorama
from colorama import Back, Fore
colorama.init(autoreset=True)


TOKEN = r"1905358159:AAHLRxV694LmBhtb6GH3aqCoG9vc3v4nd7Y" #Telegram bot token
# TOKEN = r"5347906074:AAGOrW93EKhB5ghBltX16xcNERr8wt1C_pA" #Telegram bot token (test)

EFFECTIVE_USER_PATH = os.path.join("info", "effective_users")
BLOCK_USER_PATH = os.path.join("info", "block_users")
MESSAGE = "Ciao, sono di nuovo Speech Command bot. \nTi chiedo scusa per il disturbo, ma per far in modo da poter utilizzare i dati che abbiamo raccolto per aiutare la ricerca scientifica ho bisogno che compili il questionario al link Google di seguito. Richiede pochi minuti. \nGrazie infinite della tua disponibilità. \nhttps://docs.google.com/forms/d/e/1FAIpQLScTSPx1nHk8c06vOQs5-jxOEcVFfWawO-TxHk8MNlZKP-ogww/viewform?vc=0&c=0&w=1&flr=0"
RECORDED_USER_PATH = "saves"

def main():

    def create_bot():
        updater = Updater(TOKEN)
        updater.dispatcher.add_error_handler(error_handler)
        return updater

    os.chdir(get_curr_dir(__file__))

    # Enabling logger
    logging.basicConfig(format='%(name)s - %(levelname)s - %(message)s', level=logging.INFO)
    logger = logging.getLogger(__name__)
    logging.getLogger('apscheduler.scheduler').setLevel(logging.ERROR)
    logging.getLogger('apscheduler.executors.default').setLevel(logging.ERROR)
    logging.getLogger('telegram.vendor.ptb_urllib3.urllib3.connectionpool').setLevel(logging.ERROR)

    # Start the Bot
    # Create the Updater and pass it your bot's token.
    updater = create_bot()
    logger.info(datetime.now().strftime("%d-%m-%Y %H:%M"))

    effective_users = list()
    blocked_users = list()

    try:
        user_iter = tqdm(os.listdir(RECORDED_USER_PATH))
        # user_iter = tqdm(["268005350", "1254765304"])
        for telegram_id in user_iter:
            try:
                updater.bot.send_message(chat_id=telegram_id, text=MESSAGE)
                effective_users.append(telegram_id)
            except Exception:
                blocked_users.append(telegram_id)
            
            user_iter.set_description(Fore.GREEN + "Sending messages")

        with open(EFFECTIVE_USER_PATH + "_" + str(len(effective_users)) + ".txt", "w") as fout:
            fout.write('\n'.join(effective_users))
        with open(BLOCK_USER_PATH + "_" + str(len(blocked_users)) + ".txt", "w") as fout:
            fout.write('\n'.join(blocked_users))

        # print(Back.GREEN + Fore.WHITE + "FINISH!!!")

    except telegram.error.NetworkError:
        while not is_connected(updater) :
            updater = create_bot()
            time.sleep(60)

if __name__ == "__main__":
    main()


