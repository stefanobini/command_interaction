import traceback
from pathlib import Path
import json
import pandas as pd
from datetime import datetime, timedelta
import random
import time
from apscheduler.schedulers.background import BackgroundScheduler
from apscheduler.triggers.interval import BaseTrigger
import os
import logging
from threading import Event, Timer, Thread, Lock
import random
from telegram.ext import CallbackContext
import telegram

from colorama import Back, Fore, Style, init
init(autoreset=True)

from utils.bot_messages import BOT_MESSAGES
from utils.intents import INTENTS_DICT, INTENT_REPETITIONS

logger = logging.getLogger(__name__)
BASE_PATH = "audio_samples"

'''
UTILITY FUNCTIONS
'''
def get_curr_dir(file):
    path = os.path.abspath(os.path.realpath(os.path.dirname(file)))
    return path

def error_handler(update, context):
    if context is None or context.error is None: return
    user = None if update is None else update.effective_user
    text = f"Problem with: {user} - {datetime.now()} - {context.error}"
    logger.error(msg=text, stack_info=True, stacklevel=2)

def get_age_set():
    age_str = []
    for i in range(9):
        s = fr"{10 + 10 * i}-{20 + 10 * i}"
        age_str.append([s])
    return age_str

def is_connected(updater):
    """Check if the server is connected to Internet"""
    import requests
    try:
        requests.get("http://google.com")
        updater.start_polling()
        logger.info(f"Connected - {datetime.now()}")
        return True
    except Exception:
        logger.warn(f"No internet connection - {datetime.now()}")
        return False


class State:
    def __init__(self, userid):
        """This class stores all the user info.
        When a user starts the conversation, first checks if him already exists on server, using his telegram userid.
        If not exists this class creates the user directory on disk and initializes the commands database.
        If exists it loads the  database."""
        self.id = userid.id if not type(userid) is str else userid
        self.path = Path(fr"{get_curr_dir(__file__)}/recordings/{self.id}")
        self.info_path = self.path.joinpath("info.json")
        self.database = pd.DataFrame({"int": range(0, len(INTENTS_DICT))})
        self.database.set_index("int", inplace=True)
        if type(userid) is str:
            self.info = {}
        else:
            self.info:dict = {"id": self.id,
                              "name": userid.name,
                              "link": userid.link,
                              "presentation": False,
                              "int_lang": None,
                              "gdpr": None,
                              "gender": None,
                              "age": None,
                              "acq_langs": None,
                              "init_complete": False,
                              "miss_intent": dict(),
                              "acquiring_intent": None,
                              "acquiring_lang": None,
                              "acquiring_version": None,
                              "last_int_date": str(datetime.now())
                              }
        self._load()
        Scheduler.set_scheduled_hour(self)

    @property
    def intent_number(self):
        """Total number of intents summing each language"""
        return len(INTENTS_DICT) * len(self.info["acq_langs"])
    
    @property
    def recordable_intent_number(self):
        """Total number of intents summing each language"""
        return len(INTENTS_DICT) * len(self.info["acq_langs"] * INTENT_REPETITIONS)

    @property
    def recorded_intents(self):
        """Dictionary containing the number of the acquired intent for each language."""
        rec_int:dict = dict()
        for lang in self.info["acq_langs"]:
            rec_int[lang] = self.database[lang].sum()
        return rec_int


    def set_acq_langs(self, langs:list) -> None:
        def add_acq_langs() -> None:
            """Update the database of the acquired commands adding a column for each acquisition language."""
            for lang in langs:
                self.database[lang] = [0] * len(INTENTS_DICT)
            self._save_database()
        add_acq_langs()
        for lang in langs:
            self.info["miss_intent"][lang] = list(INTENTS_DICT.keys())
        self.info["acq_langs"] = langs

    def get_message(self, label:str) -> str:
        """Return the string related to a specific state in the interaction language of the user."""
        message = ""
        if self.info["int_lang"] is None:
            for _, msg in BOT_MESSAGES[label].items():
                message += "{}\n".format(msg)
        else:
            message = BOT_MESSAGES[label][self.info["int_lang"]]
        return message

    def remember(self, bot:telegram.Bot):
        """Send a reminder to the inactive user.
        
        Parameters
        ----------
        context:    telegram-ext.CallbackContext
            Context related to the interaction between the bot and this user.
        """
        try:
            self._load()
            logger.info("I am activated reminder function for <{}>".format(self.id))
            if not self.info["init_complete"]:
                bot.send_message(chat_id=self.id, text=self.get_message("remember"))
            elif self.info["last_int_date"] is None or (Scheduler.user_is_sleeping(self) and not self.check_end()):
                logger.info(msg="Send reminder to {}".format(self.id))
                bot.send_message(chat_id=self.id, text=self.get_message("remember_intent"))
                time.sleep(1)
                self.send_next_intent_msg(bot=bot)
        except Exception as e:
            # print(traceback.format_exc())
            logger.warn(f"error: {e}; in remember for {self.id} - {datetime.now()}")
        return self.id

    def _save_info(self):
        """Saves the user info on disk"""
        with open(self.info_path, "w") as fil:
            json.dump(self.info, fil, indent=4)

    def _save_commands_count_last_date(self):
        """Updates the the date of the last interaction with the user. Then saves the info on disk."""
        now = datetime.now()
        self.info["last_int_date"] = str(now)
        self._save_info()

    def _save_database(self):
        """Saves the user database on disk"""
        self.database.to_csv(self.path.joinpath("database.csv"))

    def check_end(self):
        """Checks if user has recorded all intents."""
        if self.info["acq_langs"] == None:
            return False
        for lang in self.info["acq_langs"]:
            for e in self.database[lang]:
                if int(e) != INTENT_REPETITIONS:
                    return False
        return True

    def _load(self):
        """Loads the commands database and the user info if the user folder exist, create it otherwise."""
        if self.info_path.exists():
            with open(self.info_path) as fil:
                self.info = json.load(fil)
                self.database = pd.read_csv(self.path.joinpath("database.csv"), index_col=0)
        else:
            self.path.mkdir(parents=True, exist_ok=False)
            with open(self.info_path, "w") as fil:
                json.dump(self.info, fil, indent=4)
            self._save_database()

    def is_over(self, lang:str) -> bool:
        """Return True there is no intent to acquire for the passed language, False otherwise."""
        return self.recorded_intents[lang] == (len(INTENTS_DICT) * INTENT_REPETITIONS)

    def get_next_intent(self):
        """Return a tuple containing information about the next intent to acquire, with this format: intent ID, language, version ID, sentence.
        
        Returns
        -------
        int
            Intent identifier
        str
            Language label
        int
            Version identifier
        """
        # Chose the language
        if self.check_end():
            print(Back + "ERROR: Before to call <get_next_intent> of <State> class in <utils.py> you should chack if the acquisition is completed.")
        if self.info["acquiring_lang"] == None:
            self.info["acquiring_lang"] = self.info["acq_langs"][0]
        elif self.is_over(self.info["acquiring_lang"]):
            self.info["acquiring_lang"] = self.info["acq_langs"][self.info["acq_langs"].index(self.info["acquiring_lang"]) + 1]
        lang = self.info["acquiring_lang"]
        # Chose the intent
        candidated_intents = self.info["miss_intent"][lang]
        intent_id = candidated_intents[random.randint(0, len(candidated_intents)-1)]
        self.info["acquiring_intent"] = intent_id
        '''
        if self.database[lang][intent_id] < INTENT_REPETITIONS:
            self.database[lang][intent_id] += 1
        else:
            self.info["miss_intent"][lang].remove(intent_id)
        '''
        # Chose the version
        candidated_versions = list(INTENTS_DICT[intent_id][lang].keys())
        version_id = candidated_versions[random.randint(0, len(candidated_versions)-1)]
        self.info["acquiring_version"] = version_id
        # Save status
        self._save_info()
        self._save_database()
        return intent_id, lang, version_id

    def save(self, audio):
        """Saves on disk an audio file and set the cell of this command in the user database as acquired."""
        intent = self.info["acquiring_intent"]
        lang = self.info["acquiring_lang"]
        version = self.info["acquiring_version"]

        path = self.path.joinpath(lang)
        path.mkdir(parents=True, exist_ok=True)
        i = 0
        filename = fr"{intent}_{version}_{i}.ogg"
        dst_path = path.joinpath(filename)
        while os.path.exists(dst_path):
            i += 1
            filename = fr"{intent}_{version}_{i}.ogg"
            dst_path = path.joinpath(filename)
        audio.download(dst_path)
        
        self.database[lang][intent] += 1
        if self.database[lang][intent] >= INTENT_REPETITIONS:
            self.info["miss_intent"][lang].remove(intent)

        #self.database[intent][lang] += 1    # To record more than one sample
        self._save_database()
        self._save_commands_count_last_date()

    def send_next_intent_msg(self, bot:telegram.Bot) -> None:
        """Send to the user the text message and the sample audio related to the current intent to record.
        
        Parameters
        ----------
        bot:    telegram.Bot
            Instance of the Telegram bot interacting with the user.
        """
        intent_id, lang, version = self.get_next_intent()
        intent = INTENTS_DICT[intent_id][lang][version]
        exp_intent, imp_intent = intent.replace(')', '').split(" (")
        msg = self.get_message("int_acq").replace("**", self.get_message(lang)).replace("''", exp_intent).replace("<>", imp_intent)
        audio_name = "{}_{}.wav".format(intent_id, version)
        audio_path = os.path.join(BASE_PATH, lang, audio_name)
        with open(audio_path, "rb") as audio_file:
            bot.send_audio(chat_id=self.id, audio=audio_file, caption=msg)

class Scheduler:
    class Trigger(BaseTrigger):
        def __init__(self, delay):
            self.delay = delay
            self.first = True
        def get_next_fire_time(self, previous_fire_time, now):
            if self.first:
                self.first = False
                return now + timedelta(seconds=self.delay)
            else: return  None

    def __init__(self, context):
        """This class implements a scheduler. Its job is to remind the user to complete the data collection. Takes in input the bot instance"""
        self.scheduler = BackgroundScheduler()
        self.context = context
        self.set_userid = set()
        self.db_path = Path(fr"{get_curr_dir(__file__)}/scheduler.db")
        self.lock = Lock()
        self.scheduler.start()

    def _get_delay(self, state: State):
        """If the user has not done a command or he did not send for long time. This function computes the delay from the current datetime to the nearest target datetime"""
        if state.info["last_int_date"] is None or Scheduler.user_is_sleeping(state):
            now = datetime.now()
            target_date = datetime.strptime(fr"{now.year}-{now.month}-{now.day}-19:30", "%Y-%m-%d-%H:%M")
            delay = (target_date - now).seconds
            delay = delay + timedelta(seconds=len(self.set_userid)).total_seconds()
            return delay

    @staticmethod
    def user_is_sleeping(state: State):
        """Returns True if the user does not send commands for a long time."""
        last_date = state.info["last_int_date"]
        scheduled_hours = Scheduler.get_scheduled_hour(state)
        assert scheduled_hours is not None
        assert last_date is not None
        last_date = datetime.strptime(last_date, "%Y-%m-%d %H:%M:%S.%f")
        scheduled_hours = timedelta(seconds=float(scheduled_hours))
        now = datetime.now()
        diff = now - last_date
        return True if diff >= scheduled_hours else False

    @staticmethod
    def set_scheduled_hour(state: State):
        """Computes the delay before to start to send reminders."""
        hour = timedelta(hours=random.randint(15, 50)).total_seconds()
        path = Path(get_curr_dir(__file__)).joinpath("recordings", str(state.id), "scheduler.json")
        data = {"scheduled_hour": str(hour)}
        with open(path, "w") as fil:
            json.dump(data, fil, indent=4)

    @staticmethod
    def get_scheduled_hour(state):
        path = Path(get_curr_dir(__file__)).joinpath("recordings", state.id, "scheduler.json")
        with open(path) as fil:
            data = json.load(fil)
            return data["scheduled_hour"]

    def check(self):
        """Schedules the user reminder."""
        curr_dir = get_curr_dir(__file__)
        path = Path(fr"{curr_dir}/recordings/")
        path.mkdir(parents=True, exist_ok=True)
        for directory in os.listdir(path):
            userid = directory
            state = State(userid)
            if state.check_end(): continue
            delay = self._get_delay(state)
            if delay is not None and userid not in self.set_userid:
                self.lock.acquire()
                self.add(delay, state)
                Scheduler.set_scheduled_hour(state)
                self.lock.release()
        log = f"scheduled: {str(self.set_userid)} - {datetime.now()}"
        logger.info(log)

    def _decorator(self, func):
        """Function decorator. Removes a user by the reminder set"""
        def wrapper(**kwargs):
            userid = func(**kwargs)
            self.lock.acquire()
            self.set_userid.remove(userid)
            self.lock.release()
            logger.info(f"Removing: {str(userid)}")
        return wrapper

    def add(self, delay, state: State):
        """Adds a user to reminder set"""
        self.set_userid.add(state.id)
        self.scheduler.add_job(self._decorator(state.remember), trigger=self.Trigger(delay), kwargs={"context": self.context},
                               timezone="Europe/Rome")
        logger.info(f"Adding: {state.id} at {datetime.now() + timedelta(seconds=delay)}")


class SchedulerTimer(Thread):

    def __init__(self, interval, context):
        """This class implement a Python Timer.
        Checks if a user needs of a reminder every time interval"""
        Thread.__init__(self)
        self.scheduler = Scheduler(context=context)
        self.interval = interval
        self.function = self._decorator(self.scheduler.check)
        self.event = Event()
        self.timer: Timer = None
        self.stop_flag = False

    def _decorator(self, function):
        def wrapper():
            function()
            self.event.set()
        return wrapper

    def stop(self):
        if self.timer is not None:
            self.timer.cancel()
        logger.info(f"Shutdown scheduler - {datetime.now()}")
        self.scheduler.scheduler.shutdown()
        self.stop_flag = True
        self.event.set()

    def run(self):
        self.function()
        while not self.stop_flag:
            self.event.clear()
            self.timer = Timer(self.interval, self.function)
            self.timer.start()
            self.event.wait()