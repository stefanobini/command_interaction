#VARS
"""
command_eng = {
    0: "Increase the illumination",
    1: "Decrease the illumination",
    2: "Increase the height",
    3: "Decrease the height",
    4: "Increase the inclination",
    5: "Decrease the inclination",
    6: "Bring me the gun screwdriver",
    7: "Take the gun screwdriver",
    8: "Bring me the elbow screwdriver",
    9: "Take the elbow screwdriver",
    10: "Bring me the hammer",
    11: "Take the hammer",
    12: "Bring me the screwdriver",
    13: "Take the screwdriver",
    14: "Bring me the lever",
    15: "Take the lever",
    16: "Come here",
    17: "Go",
    18: "Start",
    19: "Stop",
    20: "Move to the right",
    21: "Move to the left",
    22: "Move up",
    23: "Move down",
    24: "Move forward",
    25: "Move backward",
    26: "Bring me the mostrina comandi",
    27: "Take the mostrina comandi",
    28: "Bring me the rearview mirror",
    29: "Take the rearview mirror",
    30: "Release",
    31: "Bring me the windows control panel",
    32: "Take the windows control panel"
}

command_ita = {
    0: "Più luce",
    1: "Meno luce",
    2: "Più alto",
    3: "Più basso",
    4: "Più inclinato",
    5: "Meno inclinato",
    6: "Portami l'avvitatore elettrico",
    7: "Prendi l'avvitatore elettrico",
    8: "Portami l'avvitatore a gomito",
    9: "Prendi l'avvitatore a gomito",
    10: "Portami il martello",
    11: "Prendi il martello",
    12: "Portami il cacciavite",
    13: "Prendi il cacciavite",
    14: "Portami la leva",
    15: "Prendi la leva",
    16: "Vieni",
    17: "Libero",
    18: "Start",
    19: "Stop",
    20: "Vai a destra",
    21: "Vai a sinistra",
    22: "Vai su",
    23: "Vai giù",
    24: "Vai avanti",
    25: "Vai indietro",
    26: "Portami la mostrina comandi",
    27: "Prendi la mostrina comandi",
    28: "Portami lo specchietto retrovisore",
    29: "Prendi lo specchietto retrovisore",
    30: "Rilascia",
    31: "Portami la mostrina comandi",
    32: "Prendi la mostrina comandi"
}
"""

command_eng = {
    0: "Increase the height",
    1: "Decrease the height",
    2: "Increase the inclination",
    3: "Decrease the inclination",
    4: "Increase the illumination",
    5: "Decrease the illumination",
    6: "Bring me the gun screwdriver",
    7: "Take the gun screwdriver",
    8: "Bring me the elbow screwdriver one",
    9: "Take the elbow screwdriver one",
    10: "Bring me the elbow screwdriver two",
    11: "Take the elbow screwdriver two",
    12: "Bring me the window control panel",
    13: "Take the window control panel",
    14: "Bring me the window frame",
    15: "Take the window frame",
    16: "Bring me the speaker",
    17: "Take the speaker",
    18: "Bring me the object holder",
    19: "Take the object holder",
    20: "Bring me the speaker frame",
    21: "Take the speaker frame",
    22: "Open the gripper",
    23: "Close the gripper",
    24: "Go to the line side",
    25: "Back home"
}

command_ita = {
    0: "Piu alto",
    1: "Piu basso",
    2: "Piu inclinato",
    3: "Meno inclinato",
    4: "Piu luce",
    5: "Meno luce",
    6: "Portami l'avvitatore elettrico",
    7: "Prendi l'avvitatore elettrico",
    8: "Portami l'avvitatore a gomito uno",
    9: "Prendi l'avvitatore a gomito uno",
    10: "Portami l'avvitatore a gomito due",
    11: "Prendi l'avvitatore a gomito due",
    12: "Portami la mostrina comandi",
    13: "Prendi la mostrina comandi",
    14: "Portami il voletto",
    15: "Prendi il voletto",
    16: "Portami l'altoparlante",
    17: "Prendi l'altoparlante",
    18: "Portami il porta oggetti",
    19: "Prendi il porta oggetti",
    20: "Portami il telaio altoparlante",
    21: "Prendi il telaio altoparlante",
    22: "Apri la pinza",
    23: "Chiudi la pinza",
    24: "Libero",
    25: "Torna a casa"
}

ITA_STR = {
    0: "Inserisci la tua fascia di età:",
    1: "ATTENZIONE❗❗❗\nDopo che hai avviato la registrazione del messaggio vocale, prima di iniziare a parlare, "
           "attendi mezzo secondo. Poichè Telegram taglia i primi decimi di secondo del messaggio",
    2: "Ok procediamo, ripeti e registra i comandi che ti invierò:",
    4: "Ti sottolineerò i comandi che devi registrare",
    5: "Assicurati di trovarti in un ambiente silenzioso, privo di rumori di sottofondo",
    6: "Iniziamo, sei maschio o femmina?",
    7: "Per avviare la conversazione scrivi: /start",
    8: "Hai già partecipato",
    9: "Grazie per aver contribuito 😊",
    10: "Vuoi fare una pausa?",
    11: "Si",
    12: "No",
    13: "Ok facciamo una pausa, quando vuoi ricominciare registrami il seguente comando:",
    14: "Ok continuiamo:",
    15: "Maschio",
    16: "Femmina",
    18: "Registra e ripeti il seguente comando:",
    19: "Hey non dimenticarti di me, ho ancora bisogno del tuo aiuto 🥺\n"
                       "Puoi rispondere alla domanda che ti ho fatto sopra? ☝☝☝",
    20: "Hey non dimenticarti di me, ho ancora bisogno del tuo aiuto 🥺",
    21: "Puoi registrare il seguente comando?",
    22: "Congratulazioni🥳\nHai ottenuto {} premi su {}\n",
    23: "Ok continuiamo, ripeti e registra:",
    24: "Ecco il premio:"
}

ENG_STR = {
    0: "Enter your age range:",
    1: "ATTENTION❗❗❗After you have started recording the voice message, before starting to speak, "
        "wait half a second. Because Telegram cuts the first tenths of a second of the message",
    2: "Ok let's go on, repeat and record the commands that I will send you:",
    4: "I will underline the commands you need to record",
    5: "Make sure you are in a quiet environment with no background noise",
    6: "Let's get started, are you male or female?",
    7: "To start the conversation type: / start",
    8: "You have already participated",
    9: "Thanks for contributing 😊",
    10: "Do you want to take a break?",
    11: "Yes",
    12: "No",
    13: "Ok let's take a break, when you want to start again record me the following command:",
    14: "Ok let's continue:",
    15: "Male",
    16: "Female",
    18: "Record and repeat the following command:",
    19: "Hey don't forget me, I still need your help 🥺\n"
        "Can you answer the question I asked you above? ☝☝☝",
    20: "Hey don't forget me, I still need your help 🥺",
    21: "Can you record the following command?",
    22: "Congratulations🥳\nYou got {} rewards on {}\n",
    23: "Ok let's continue, repeat and record:",
    24: "Here is the reward for you:"
}

WELCOME_STR = {
    0: "Ciao, sono Speech Command bot, il bot Telegream del Laboratorio MIVIA dell'Università degli Studi di Salerno e ti assisterò durante tutta la procedura.😊\n" \
       "Ti invierò una serie di comandi, il tuo compito sarà quello di registrare tali comandi ed inviarmeli.\n"
       "Inoltre, man mano che prosegui ti invierò dei premi per rigraziarti del tuo impegno 🥳\n"
       "Nel caso tu voglia partecipare alla procedura si preoccupi di compilare il questionario al seguente link prima di procedere.\n"
       "https://docs.google.com/forms/d/e/1FAIpQLScTSPx1nHk8c06vOQs5-jxOEcVFfWawO-TxHk8MNlZKP-ogww/viewform?vc=0&c=0&w=1&flr=0\n\n"
       "Hi, I'm Speech Command bot and I will assist you throughout the procedure.😊\n"
       "I will send you a series of commands, your task will be to record these commands and send them to me.\n"
       "Also, as you proceed, I'll send you some rewards to thank you for your effort🥳\n"
       "If you want to participate in the procedure, fill out the questionnaire at the following link before proceeding.\n"
       "https://docs.google.com/forms/d/e/1FAIpQLSc_5ZZexxQ4rRtyV3jXpU8OaFumn1BozWHG7kQoohU0TSDXSQ/viewform?vc=0&c=0&w=1&flr=0\n\n",
    1: "Prima di iniziare dimmi se vuoi ricevere i comandi in lingua italiana, inglese o entrambe.\n"
       "Se selezioni entrambe le lingue avrai un maggior numero di premi\n\n"
       "Before starting tell me if you want to receive the commands in Italian, English or both.\n"
       "If you select both languages you will get more rewards",
    2: "Seleziona la lingua dei comandi:\n\nSelect the language of the commands:"
}