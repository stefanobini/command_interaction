import shutil
import time
from datetime import datetime
from pathlib import Path
import logging
import numpy as np

from telegram.ext import CallbackContext, ConversationHandler
from telegram import Update, InlineKeyboardMarkup, InlineKeyboardButton

import utils.states as STATES
from utils.utils import State, get_age_set, get_curr_dir


logger = logging.getLogger(__name__)


def hello(update: Update, context: CallbackContext) -> int:
    print("hello")
    state: State = context.user_data["state"] 
    time.sleep(1)
    context.bot.send_message(state.id, state.get_message("hello"))
    return STATES.ASK_INTERACTION_LANGUAGE


def ask_int_lang(update: Update, context: CallbackContext) -> int:
    """INTERACTION LANGUAGES state.
    Ask the interaction language to the user."""
    print("ask_int_lang")
    # Ask to the user in which languages he/she wants acquire data.
    state: State = context.user_data["state"]
    keyword = [InlineKeyboardButton("English", callback_data="lang_eng"),
                InlineKeyboardButton("Español", callback_data="lang_esp"),
                InlineKeyboardButton("Italiano", callback_data="lang_ita")]
    time.sleep(1)
    context.bot.send_message(state.id, state.get_message("int_lang"), reply_markup=InlineKeyboardMarkup(keyword))
    return STATES.GET_ACQUISITION_LANGUAGES

def get_int_lang(update: Update, context: CallbackContext) -> int:
    """INTERACTION LANGUAGE state.
    Receive the interaction language from the user."""
    update.callback_query.answer()
    state: State = context.user_data["state"]
    lang = update.callback_query.data.split("_")[1]
    state.info["int_lang"] = lang
    state._save_info()
    return STATES.GET_INTERACTION_LANGUAGE


def ask_gdpr(update: Update, context: CallbackContext) -> int:
    """GDPR state.
    Ask the privacy Consent to the user."""
    state: State = context.user_data["state"]
    context.bot.send_message(state.id, state.get_message("gdpr_link"))    # send this link with some istructions
    time.sleep(10)

    keyword = [[InlineKeyboardButton(state.get_message("agree"), callback_data="gdpr_yes"),
                InlineKeyboardButton(state.get_message("not_agree"), callback_data="gdpr_no")]]
    time.sleep(1)
    context.bot.send_message(state.id, state.get_message("gdpr_agreement"), reply_markup=InlineKeyboardMarkup(keyword))
    return STATES.ASK_GDPR

def get_gdpr(update: Update, context: CallbackContext) -> int:
    """GDPR state.
    Receive the Privacy Consent answer from the user."""
    update.callback_query.answer()
    state: State = context.user_data["state"]
    gdpr = update.callback_query.data.split("_")[1]
    state.info["GDPR"] = gdpr
    state._save_info()
    if gdpr == "no":
        context.bot.send_message(state.id, state.get_message("refuse_gdpr"))
        reset(update=update, context=context)
    return STATES.GET_GDPR


def ask_gender(update: Update, context: CallbackContext) -> int:
    """GENDER state.
    Ask gender to the user."""
    state: State = context.user_data["state"] 
    keyword = [[InlineKeyboardButton(state.get_message("male"), callback_data="gender_Male"),
                InlineKeyboardButton(state.get_message("female"), callback_data="gender_Female")]]
    time.sleep(1)
    context.bot.send_message(state.id, state.get_message("gender"), reply_markup=InlineKeyboardMarkup(keyword))
    return STATES.ASK_GENDER

def get_gender(update: Update, context: CallbackContext) -> int:
    """GENDER state.
    Receive gender from the user."""
    update.callback_query.answer()
    state: State = context.user_data["state"]
    gender = update.callback_query.data.split("_")[1]
    state.info["gender"] = gender
    state._save_info()
    return STATES.ASK_AGE


def ask_age(update: Update, context: CallbackContext) -> int:
    """AGE state.
    Ask age range to the user."""
    state: State = context.user_data["state"]
    age_set = get_age_set()
    for i, e in enumerate(age_set):
        age_set[i] = InlineKeyboardButton(str(e[0]), callback_data=f"age_{e}")
    age_set = np.array(age_set)
    age_set = age_set.reshape((-1, 3))
    keyboard = InlineKeyboardMarkup(age_set)
    time.sleep(1)
    context.bot.send_message(state.id, state.get_message("age"), reply_markup=keyboard)
    return STATES.GET_AGE

def get_age(update: Update, context: CallbackContext) -> int:
    """AGE state.
    Receive age range from the user."""
    # Get user answer (age)
    update.callback_query.answer()
    state: State = context.user_data["state"]
    age = update.callback_query.data.split("_")[1]
    state.info["age"] = age
    state._save_info()
    return STATES.ASK_ACQUISITION_LANGUAGES


def ask_acq_langs(update: Update, context: CallbackContext) -> int:
    """ACQUISITION LANGUAGES state.
    Ask the acquisition languages to the user."""
    # Ask to the user in which languages he/she wants acquire data.
    state: State = context.user_data["state"]
    keyword = [[InlineKeyboardButton("English - Español - Italiano", callback_data="lang_eng-esp-ita")],

               [InlineKeyboardButton("English - Español", callback_data="lang_eng-esp"),
                InlineKeyboardButton("English - Italiano", callback_data="lang_eng-ita"),
                InlineKeyboardButton("Español - Italiano", callback_data="lang_esp-ita")],

                [InlineKeyboardButton("English", callback_data="lang_eng"),
                 InlineKeyboardButton("Español", callback_data="lang_esp"),
                 InlineKeyboardButton("Italiano", callback_data="lang_ita")]]
    time.sleep(1)
    context.bot.send_message(state.id, state.get_message("acq_lang"), reply_markup=InlineKeyboardMarkup(keyword))
    return STATES.GET_ACQUISITION_LANGUAGES

def get_acq_langs(update: Update, context: CallbackContext) -> int:
    """ACQUISITION LANGUAGES state.
    Receive acquisition languages from the user"""
    # Get user answer (interaction language)
    update.callback_query.answer()
    state: State = context.user_data["state"]
    langs = update.callback_query.data.split("_")[1]
    lang_list = langs.split("-")
    state.set_acq_langs(langs=lang_list)
    state._save_info()
    return STATES.ACQUISITION_INFO


def give_acq_info(update: Update, context: CallbackContext) -> int:
    """INTENT ACQUISITION state.
    Send instruction to the user before starting the acquisition loop.""" 
    state: State = context.user_data["state"]
    context.bot.send_message(state.id, state.get_message("start_acq"))

    keyword = [[InlineKeyboardButton(state.get_message("yes"), callback_data="start_yes"),
                InlineKeyboardButton(state.get_message("no"), callback_data="start_no")]]
    time.sleep(1)
    context.bot.send_message(state.id, state.get_message("start_acq"), reply_markup=InlineKeyboardMarkup(keyword))
    return STATES.START_ACQUISITION

def start_acquisition(update: Update, context: CallbackContext) -> int:
    """INTENT ACQUISITION state.
    Check if the user is ready to start the acquisition loop.""" 
    update.callback_query.answer()
    state: State = context.user_data["state"]
    start = update.callback_query.data.split("_")[1]
    if start == "yes":
        state.info["init_complete"] = True
        state._save_info()
        return STATES.ASK_INTENT_ACQUISITION
    else:
        return STATES.ACQUISITION_INFO


def ask_intent(update: Update, context: CallbackContext) -> int:
    """INTENT ACQUISITION state.
    Send the text related to the intent to record and an audio sample for understand the tone to use."""
    state = State(update.effective_user)
    context.user_data["state"] = state
    state.send_next_intent_msg(context=context)
    return STATES.GET_INTENT_ACQUISITION

def get_intent(update: Update, context: CallbackContext) -> int:
    """INTENT ACQUISITION state.
    Receive the recorded intent from the user."""
    state = State(update.effective_user)
    context.user_data["state"] = state
    state.save(update.message.voice.get_file())
    if state.check_end():
        update.message.reply_text(state.get_message("end"))
        return ConversationHandler.END
    else:
        return STATES.ASK_INTENT_ACQUISITION

'''
This callback is classed when the user start the bot (/start command).
Checks if the user has already started the bot: if no, starts a new conversation else continues the previous conversation.
If the user has already done the data collection terminates the conversation
'''
def run_bot(update: Update, context: CallbackContext) -> int:
    state = State(update.effective_user)
    context.user_data["state"] = state

    log_text = '{' + fr"{update.effective_user.full_name}, {update.effective_user.link}, {update.effective_user.id}, complete: {state.check_end()}" + '}'
    logger.info(fr"Entra: {log_text} - {datetime.now().strftime('%d-%m-%Y %H:%M:%S')}")

    if state.check_end():
        update.message.reply_text(state.get_message("end"))
        return ConversationHandler.END

    if state.info["hello"] == False:
        print("run_bot, hello")
        return STATES.HELLO
    elif state.info["int_lang"] == None:
        print("run_bot, int_lang")
        return STATES.ASK_INTERACTION_LANGUAGE
    elif state.info["gdpr"] == None:
        return STATES.ASK_GDPR
    elif state.info["gender"] == None:
        return STATES.ASK_GENDER
    elif state.info["age"] == None:
        return STATES.ASK_AGE
    elif state.info["acq_langs"] == None:
        return STATES.ASK_ACQUISITION_LANGUAGES
    else:
        return STATES.ASK_INTENT_ACQUISITION
    

# THESE METHODS SHOULD BE USELESS
'''
If the conversation ended but the user did not complete the task, 
this callback continues the conversation when the user make a query using the Inline keyboard
'''
def select_query(update: Update, context: CallbackContext):
    query_data = update.callback_query.data
    state = State(update.effective_user)
    context.user_data["state"] = state
    identifier, data = query_data.split("_")[0], query_data.split("_")[1]
    if identifier == "gender" and state.info["gender"] is None:
        return get_gender(update, context)
    elif identifier == "age" and state.info["age"] is None:
        return get_age(update, context)
    elif identifier == "int_lang" and state.info["gender"] is None:
        return get_int_lang(update, context)
    else:
        update.callback_query.answer()
        return run_bot(update, context)

'''
Deletes all user files
'''
def reset(update: Update, context: CallbackContext):
    state = State(update.effective_user)
    userid = update.effective_user["id"]
    path = Path(get_curr_dir(__file__)).joinpath("recordings", str(userid))
    if path.exists():
        shutil.rmtree(path, ignore_errors=True)
    update.message.reply_text(state.get_message(label="restart"))
    logger.info(f"user {userid} has resetted profile - {datetime.now()}")
    return ConversationHandler.END
