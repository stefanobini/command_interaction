#doc: https://python-telegram-bot.readthedocs.io/en/stable/
#github rep: https://github.com/python-telegram-bot/python-telegram-bot
"""
python3 utils/send_reminder.py 
"""

import logging
import time
import telegram.error
from telegram.ext import Updater
from utils import get_curr_dir, error_handler, is_connected
import os
from datetime import datetime

from tqdm import tqdm
import colorama
from colorama import Back, Fore
colorama.init(autoreset=True)
import pandas
import json

from bot_messages import BOT_MESSAGES


TOKEN = "6272694271:AAFw-2nWm4zg2hOY7tD0Rp2wZ58B8CZxKIk"    # Beis token (MIVIA-IRI)
# TOKEN = r"5347906074:AAGOrW93EKhB5ghBltX16xcNERr8wt1C_pA" #Telegram bot token (test)

EFFECTIVE_USER_PATH = os.path.join("..", "info", "effective_users")
BLOCK_USER_PATH = os.path.join("..", "info", "block_users")
RECORDED_USER_PATH = "recordings"
LANGS = ["eng", "esp", "ita"]
N_REPETITION_X_INTENT = 3

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
        #user_iter = tqdm(["268005350", "1254765304"])
        for telegram_id in user_iter:
            user_end = True
            user_path = os.path.join(RECORDED_USER_PATH, telegram_id)
            json_path = os.path.join(user_path, "info.json")
            info_path = os.path.join(user_path, "database.csv")
            info_json = None
            with open(json_path) as f:
                info_json = json.load(f)
            speaker_lang = info_json["int_lang"]
            info_df = pandas.read_csv(info_path, sep=',')
            columns = info_df.columns.tolist()
            if len(columns) > 1:
                langs = columns[1:]
                i = 0
                while i < len(info_df.index) and user_end:
                    j = 0
                    while j < len(langs) and user_end:
                        if info_df[langs[j]][i] != N_REPETITION_X_INTENT:
                            user_end = False
                        j += 1
                    i += 1
            else:
                try:
                    updater.bot.send_message(chat_id=telegram_id, text=BOT_MESSAGES["remember"][speaker_lang])
                    effective_users.append(telegram_id)
                except Exception:
                    blocked_users.append(telegram_id)
            if not user_end:
                try:
                    updater.bot.send_message(chat_id=telegram_id, text=BOT_MESSAGES["reminder"][speaker_lang])
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


