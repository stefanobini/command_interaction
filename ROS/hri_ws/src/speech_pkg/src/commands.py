# coding=utf-8

AVAILABLE_LANGS = ["eng", "ita"]

DEMO3 = {
    "eng":{
        0: "Increase the height",
        1: "Decrease the height",
        2: "Increase the inclination",
        3: "Decrease the inclination",
        4: "Increase the illumination",
        5: "Increase the illumination",
        6: "No command"
    },
    "ita":{
        0: "Piu alto",
        1: "Piu basso",
        2: "Piu inclinato",
        3: "Meno inclinato",
        4: "Piu luce",
        5: "Meno luce",
        6: "Nessun comando"
    }
}
DEMO3_MAPPING = {
    0:0,
    1:1,
    2:2,
    3:3,
    4:4,
    5:5,
    6:26
}

""" DEMO_7 = {
    "eng":{
        0: "Bring me the gun screwdriver",
        1: "Take the gun screwdriver",
        2: "Bring me the elbow screwdriver one",
        3: "Take the elbow screwdriver one",
        4: "Bring me the elbow screwdriver two",
        5: "Take the elbow screwdriver two",
        6: "Bring me the window control panel",
        7: "Take the window control panel",
        8: "Bring me the window frame",
        9: "Take the window frame",
        10: "Bring me the speaker",
        11: "Take the speaker",
        12: "Bring me the object holder",
        13: "Take the object holder",
        14: "Bring me the speaker frame",
        15: "Take the speaker frame",
        16: "Open the gripper",
        17: "Close the gripper",
        18: "Go to the line side",
        19: "Back home",
        20: "No command"
    },
    "ita":{
        0: "Portami l avvitatore elettrico",
        1: "Prendi l avvitatore elettrico",
        2: "Portami l avvitatore a gomito uno",
        3: "Prendi l avvitatore a gomito uno",
        4: "Portami l avvitatore a gomito due",
        5: "Prendi l avvitatore a gomito due",
        6: "Portami la mostrina comandi",
        7: "Prendi la mostrina comandi",
        8: "Portami il voletto",
        9: "Prendi il voletto",
        10: "Portami l altoparlante",
        11: "Prendi l altoparlante",
        12: "Portami il porta oggetti",
        13: "Prendi il porta oggetti",
        14: "Portami il telaio altoparlante",
        15: "Prendi il telaio altoparlante",
        16: "Apri la pinza",
        17: "Chiudi la pinza",
        18: "Libero",
        19: "Torna a casa",
        20: "Nessun comando"
    }
} """
DEMO7 = {
    "eng":{
        0: "Bring me the gun screwdriver",
        1: "Take the gun screwdriver",
        2: "Bring me the elbow screwdriver one",
        3: "Take the elbow screwdriver one",
        4: "Bring me the elbow screwdriver two",
        5: "Take the elbow screwdriver two",
        6: "Bring me the window control panel",
        7: "Take the window control panel",
        8: "Open the gripper",
        9: "Close the gripper",
        10: "Go to the line side",
        11: "Back home",
        12: "No command"
    },
    "ita":{
        0: "Portami l avvitatore elettrico",
        1: "Prendi l avvitatore elettrico",
        2: "Portami l avvitatore a gomito uno",
        3: "Prendi l avvitatore a gomito uno",
        4: "Portami l avvitatore a gomito due",
        5: "Prendi l avvitatore a gomito due",
        6: "Portami la mostrina comandi",
        7: "Prendi la mostrina comandi",
        8: "Apri la pinza",
        9: "Chiudi la pinza",
        10: "Libero",
        11: "Torna a casa",
        12: "Nessun comando"
    }
}
DEMO7_MAPPING = {

}
DEMO7_PHASE2_MAPPING = {
    0:  6,  # id_model: id_felice (for orchestrator)
    1:  7,
    2:  8,
    3:  9,
    4:  10,
    5:  11,
    6:  12,
    7:  13,
    8:  22,
    9:  23,
    10: 24,
    11: 25,
    12: 26
}

DEMO_FULL = {
    "eng":{
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
        25: "Back home",
        26: "No command"
    },
    "ita":{
        0: "Piu alto",
        1: "Piu basso",
        2: "Piu inclinato",
        3: "Meno inclinato",
        4: "Piu luce",
        5: "Meno luce",
        6: "Portami l avvitatore elettrico",
        7: "Prendi l avvitatore elettrico",
        8: "Portami l avvitatore a gomito uno",
        9: "Prendi l avvitatore a gomito uno",
        10: "Portami l avvitatore a gomito due",
        11: "Prendi l avvitatore a gomito due",
        12: "Portami la mostrina comandi",
        13: "Prendi la mostrina comandi",
        14: "Portami il voletto",
        15: "Prendi il voletto",
        16: "Portami l altoparlante",
        17: "Prendi l altoparlante",
        18: "Portami il porta oggetti",
        19: "Prendi il porta oggetti",
        20: "Portami il telaio altoparlante",
        21: "Prendi il telaio altoparlante",
        22: "Apri la pinza",
        23: "Chiudi la pinza",
        24: "Libero",
        25: "Torna a casa",
        26: "Nessun comando"
    }
}
DEMOFULL_MAPPING = {
    0:  0,  # id_model: id_felice (for orchestrator)
    1:  1,
    2:  2,
    3:  3,
    4:  4,
    5:  5,
    6:  6,
    7:  7,
    8:  8,
    9:  9,
    10: 10,
    11: 11,
    12: 12,
    13: 13,
    14: 14,
    15: 15,
    16: 16,
    17: 17,
    18: 18,
    19: 19,
    20: 20,
    21: 21,
    22: 22,
    23: 23,
    24: 24,
    25: 25,
    26: 26,
    27: 26,
    28: 26,
    29: 26,
    30: 26
}

MAPPING = {
    "3":        DEMO3_MAPPING,
    "3_phase2": DEMO3_MAPPING,
    "7":        DEMO7_MAPPING,
    "7_phase2": DEMO7_PHASE2_MAPPING,
    "full": 	DEMOFULL_MAPPING
}
