import shutil
import time
from datetime import datetime
from pathlib import Path
import logging
import numpy as np
from colorama import Back, Fore, Style, init
init(autoreset=True)

from telegram.ext import CallbackContext, ConversationHandler
from telegram import Update, InlineKeyboardMarkup, InlineKeyboardButton

import utils.states as STATES
from utils.utils import State, get_age_set, get_curr_dir


LANGS = ["eng", "esp", "ita"]
GENDERS = ["male", "female", "no"]
COMB_LANGS = ["eng-esp-ita", "eng-esp", "eng-ita", "esp-ita", "eng", "esp", "ita"]
logger = logging.getLogger(__name__)


def presentation(update: Update, context: CallbackContext) -> int:
    #print(Fore.GREEN + "PRESENTATION")
    state = State(update.effective_user)
    context.user_data["state"] = state
    # Set logger
    log_text = '{' + fr"{update.effective_user.full_name}, {update.effective_user.link}, {update.effective_user.id}, complete: {state.check_end()}" + '}'
    logger.info(fr"Entra: {log_text} - {datetime.now().strftime('%d-%m-%Y %H:%M:%S')}")
    time.sleep(0.5)
    # Ask for the interaction language
    ask_int_lang(update=update, context=context)
    # Go to the INTERACTION LANGUAGE state
    return STATES.INTERACTION_LANGUAGE


def ask_int_lang(update: Update, context: CallbackContext) -> None:
    state: State = context.user_data["state"]
    state.info["presentation"] = True
    state._save_info()
    context.bot.send_message(state.id, state.get_message("hello"))
    # update.message.reply_text(state.get_message("hello"))
    keyword = [[InlineKeyboardButton("English", callback_data="lang_eng"),
                InlineKeyboardButton("Español", callback_data="lang_esp"),
                InlineKeyboardButton("Italiano", callback_data="lang_ita")]]
    time.sleep(5)   # set 5s
    context.bot.send_message(chat_id=state.id, text=state.get_message("int_lang"), reply_markup=InlineKeyboardMarkup(keyword))

def get_int_lang(update: Update, context: CallbackContext) -> int:
    """INTERACTION LANGUAGE state.
    Receive the interaction language from the user, then ask the privacy Consent to the user."""
    #print(Fore.GREEN + "INTERACTION LANGUAGE")
    # Receive the interaction language from the user
    update.callback_query.answer()
    state: State = context.user_data["state"]
    lang = update.callback_query.data.split("_")[1]
    # Check if the selection of the user is valid
    if lang not in LANGS:
        context.bot.send_message(chat_id=state.id, text=state.get_message("selection_error"))
        time.sleep(2)
        ask_int_lang(update=update, context=context)
        return STATES.INTERACTION_LANGUAGE
    state.info["int_lang"] = lang
    state._save_info()
    # Ask the privacy Consent to the user
    ask_gdpr(update=update, context=context)    
    # Go to the GDPR state
    return STATES.GDPR


def ask_gdpr(update: Update, context: CallbackContext) -> None:
    state: State = context.user_data["state"]
    context.bot.send_message(state.id, state.get_message("gdpr_link").replace("<>", str(state.id)))    # send this link with some istructions
    time.sleep(15)   # set 15s
    keyword = [[InlineKeyboardButton(state.get_message("agree"), callback_data="gdpr_yes"),
                InlineKeyboardButton(state.get_message("not_agree"), callback_data="gdpr_no")]]
    #time.sleep(0.5)
    context.bot.send_message(state.id, state.get_message("gdpr_agreement"), reply_markup=InlineKeyboardMarkup(keyword))

def get_gdpr(update: Update, context: CallbackContext) -> int:
    """GDPR state.
    Receive the Privacy Consent answer from the user, then ask gender to the user."""
    #print(Fore.GREEN + "GDPR")
    # Receive the Privacy Consent answer from the user
    update.callback_query.answer()
    state: State = context.user_data["state"]
    gdpr = update.callback_query.data.split("_")[1]
    state.info["gdpr"] = gdpr
    state._save_info()
    if gdpr == "no":
        reset(update=update, context=context)
        return ConversationHandler.END
    elif gdpr == "yes":
        # Ask gender to the user
        ask_gender(update=update, context=context)
        # Go to GENDER state
        return STATES.GENDER
    else:
        # Invalid selection of the user
        context.bot.send_message(chat_id=state.id, text=state.get_message("selection_error"))
        time.sleep(2)
        ask_gdpr(update=update, context=context)
        return STATES.GDPR


def ask_gender(update: Update, context: CallbackContext) -> None:
    state: State = context.user_data["state"]
    keyword = [[InlineKeyboardButton(state.get_message("male"), callback_data="gender_male"),
                InlineKeyboardButton(state.get_message("female"), callback_data="gender_female")],

               [InlineKeyboardButton(state.get_message("not_declared"), callback_data="gender_no")]]
    #time.sleep(0.5)
    context.bot.send_message(state.id, state.get_message("gender"), reply_markup=InlineKeyboardMarkup(keyword))
    
def get_gender(update: Update, context: CallbackContext) -> int:
    """GENDER state.
    Receive gender from the user, then ask age range to the user."""
    #print(Fore.GREEN + "GENDER")
    # Receive gender from the user
    update.callback_query.answer()
    state: State = context.user_data["state"]
    gender = update.callback_query.data.split("_")[1]
    # Check if the selection of the user is valid
    if gender not in GENDERS:
        context.bot.send_message(chat_id=state.id, text=state.get_message("selection_error"))
        time.sleep(2)
        ask_gender(update=update, context=context)
        return STATES.GENDER
    state.info["gender"] = gender
    state._save_info()
    # Ask age range to the user
    ask_age(update=update, context=context)
    # Go to the AGE state
    return STATES.AGE


def ask_age(update: Update, context: CallbackContext) -> None:
    state: State = context.user_data["state"]
    age_set = get_age_set()
    tmp_list, final_list = list(), list()
    for i, e in enumerate(age_set):
        tmp_list.append(InlineKeyboardButton(str(e[0]), callback_data=f"age_{e}"))
        if (i+1)%3 == 0:
            final_list.append(tmp_list)
            tmp_list = list()
    final_list.append([InlineKeyboardButton(state.get_message("not_declared"), callback_data="age_no")])
    keyboard = InlineKeyboardMarkup(final_list)
    #time.sleep(0.5)
    context.bot.send_message(state.id, state.get_message("age"), reply_markup=keyboard)

def get_age(update: Update, context: CallbackContext) -> int:
    """AGE state.
    Receive age range from the user, then ask the acquisition languages to the user."""
    # Get user answer (age)
    update.callback_query.answer()
    state: State = context.user_data["state"]
    age = update.callback_query.data.split("_")[1]
    # Check if the selection of the user is valid
    if [age[2:-2]] not in get_age_set() and age != "no":
        context.bot.send_message(chat_id=state.id, text=state.get_message("selection_error"))
        time.sleep(2)
        ask_age(update=update, context=context)
        return STATES.AGE
    state.info["age"] = age
    state._save_info()
    # Ask the acquisition languages to the user
    ask_acq_langs(update=update, context=context)
    # Go to the ACQUISITION LANGUAGE state
    return STATES.ACQUISITION_LANGUAGES


def ask_acq_langs(update: Update, context: CallbackContext) -> None:
    state: State = context.user_data["state"]
    keyword = [[InlineKeyboardButton("English - Español - Italiano", callback_data="lang_eng-esp-ita")],
               
               [InlineKeyboardButton("English - Español", callback_data="lang_eng-esp")],
               [InlineKeyboardButton("English - Italiano", callback_data="lang_eng-ita")],
               [InlineKeyboardButton("Español - Italiano", callback_data="lang_esp-ita")],
               
               [InlineKeyboardButton("English", callback_data="lang_eng"),
                InlineKeyboardButton("Español", callback_data="lang_esp"),
                InlineKeyboardButton("Italiano", callback_data="lang_ita")]]
    #time.sleep(0.5)
    context.bot.send_message(state.id, state.get_message("acq_lang"), reply_markup=InlineKeyboardMarkup(keyword))
    
def get_acq_langs(update: Update, context: CallbackContext) -> int:
    """ACQUISITION LANGUAGES state.
    Receive acquisition languages from the user, then send info for start the acquisition."""
    #print(Fore.GREEN + "ACQUISITION_LANGUAGES")
    # Receive acquisition languages from the user
    update.callback_query.answer()
    state: State = context.user_data["state"]
    langs = update.callback_query.data.split("_")[1]
    # Check if the selection of the user is valid
    if langs not in COMB_LANGS:
        context.bot.send_message(chat_id=state.id, text=state.get_message("selection_error"))
        time.sleep(2)
        ask_acq_langs(update=update, context=context)
        return STATES.ACQUISITION_LANGUAGES
    lang_list = langs.split("-")
    state.set_acq_langs(langs=lang_list)
    state._save_info()
    #update.message.reply_text(state.get_message("acq_info"))
    context.bot.send_message(state.id, state.get_message("acq_info"))
    time.sleep(5)   # set 5s
    # Ask to the user if he is ready
    ask_ready(update=update, context=context)
    # Go to the START ACQUISITION state
    return STATES.START_ACQUISITION


def ask_ready(update: Update, context: CallbackContext) -> None:
    state: State = context.user_data["state"]
    keyword = [[InlineKeyboardButton(state.get_message("yes"), callback_data="start_yes"),
                InlineKeyboardButton(state.get_message("no"), callback_data="start_no")]]
    #time.sleep(0.5)
    context.bot.send_message(state.id, state.get_message("start_acq"), reply_markup=InlineKeyboardMarkup(keyword))
    
def start_acquisition(update: Update, context: CallbackContext) -> int:
    """INTENT ACQUISITION state.
    Check if the user is ready to start the acquisition loop."""
    #print(Fore.GREEN + "START ACQUISITION")
    update.callback_query.answer()
    state: State = context.user_data["state"]
    start = update.callback_query.data.split("_")[1]
    if start == "yes":
        state.info["init_complete"] = True
        state._save_info()
        # Send the text related to the intent to record and an audio sample for understand the tone to use
        state.send_next_intent_msg(bot=context.bot)
        # Go to the INTENT ACQUISITION state
        return STATES.INTENT_ACQUISITION
    elif start == "no":
        # Check if the user is ready to start the acquisition loop
        ask_ready(update=update, context=context)
        # Go to the START ACQUISITION state
        return STATES.START_ACQUISITION
    else:
        # Check if the selection of the user is valid
        context.bot.send_message(chat_id=state.id, text=state.get_message("selection_error"))
        time.sleep(10)
        ask_ready(update=update, context=context)
        return STATES.START_ACQUISITION

def get_intent(update: Update, context: CallbackContext) -> int:
    """INTENT ACQUISITION state.
    Send the text related to the intent to record and an audio sample for understand the tone to use, then receive the recorded intent from the user."""
    #print(Fore.GREEN + "INTENT ACQUISITION")
    # Receive the recorded intent from user.
    state = State(update.effective_user)
    context.user_data["state"] = state
    state.save(update.message.voice.get_file())
    if state.check_end():
        update.message.reply_text(state.get_message("end"))
        time.sleep(3)
        update.message.reply_text(state.get_message("delete_data").replace("<>", str(state.id)))
        return ConversationHandler.END
    else:
        # Send the text related to the intent to record and an audio sample for understand the tone to use
        state.send_next_intent_msg(bot=context.bot)
        return STATES.INTENT_ACQUISITION


def conversation_handler(update: Update, context: CallbackContext) -> int:
    #print(Fore.GREEN + "CONVERSATION HANDLER")
    state = State(update.effective_user)
    context.user_data["state"] = state

    log_text = '{' + fr"{update.effective_user.full_name}, {update.effective_user.link}, {update.effective_user.id}, complete: {state.check_end()}" + '}'
    logger.info(fr"Entra: {log_text} - {datetime.now().strftime('%d-%m-%Y %H:%M:%S')}")

    if state.check_end():
        update.message.reply_text(state.get_message("end"))
        time.sleep(3)
        update.message.reply_text(state.get_message("delete_data"))
        return ConversationHandler.END

    if state.info["presentation"] == False:
        # I cannot use this branch, due to the state Handler
        return STATES.PRESENTATION
    elif state.info["int_lang"] == None:
        #ask_int_lang(update==update, context=context)
        return STATES.INTERACTION_LANGUAGE
    elif state.info["gdpr"] == None:
        #ask_gdpr(update==update, context=context)
        return STATES.GDPR
    elif state.info["gender"] == None:
        #ask_gender(update==update, context=context)
        return STATES.GENDER
    elif state.info["age"] == None:
        #ask_age(update==update, context=context)
        return STATES.AGE
    elif state.info["acq_langs"] == None:
        #ask_acq_langs(update==update, context=context)
        return STATES.ACQUISITION_LANGUAGES
    else:
        print(Back.RED + "ERROR - Conversation ends!")
        return ConversationHandler.END
    

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

    if identifier == "presentation" and state.info["presentation"] == False:
        return presentation(update=update, context=context)
    elif state.info["int_lang"] == None:
        return ask_int_lang(update=update, context=context)
    elif state.info["gdpr"] == None:
        return ask_gdpr(update=update, context=context)
    elif state.info["gender"] == None:
        return ask_gender(update=update, context=context)
    elif state.info["age"] == None:
        return ask_age(update=update, context=context)
    elif state.info["acq_langs"] == None:
        return ask_acq_langs(update=update, context=context)
    else:
        return ask_acq_langs(update=update, context=context)


'''
Deletes all user files
'''
def reset(update: Update, context: CallbackContext):
    state = State(update.effective_user)
    userid = update.effective_user["id"]
    path = Path(get_curr_dir(__file__)).joinpath("recordings", str(userid))
    if path.exists():
        shutil.rmtree(path, ignore_errors=True)
    if state.info["gdpr"] == "no":
        context.bot.send_message(state.id, state.get_message("refuse_gdpr"))
    else:
        context.bot.send_message(state.id, state.get_message("restart"))
    logger.info(f"user {userid} has resetted profile - {datetime.now()}")
    return ConversationHandler.END
