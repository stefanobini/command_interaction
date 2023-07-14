INTENTS:dict = {
    0:  {
        "text":{
            "eng":"start (not urgent)",
            "esp":"comienza (no urgente)",
            "ita":"inizia (non urgente)"},
        "explicit":{
            "eng":{0:{"id":0, "text":"start"}},
            "esp":{0:{"id":0, "text":"comienza"}},
            "ita":{0:{"id":0, "text":"inizia"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"no urgente"},
            "ita":{"id":0, "text":"non urgente"}
            }
        },
    1:  {
        "text":{
            "eng":"start (urgent)",
            "esp":"comienza (urgente)",
            "ita":"inizia (urgente)"},
        "explicit":{
            "eng":{0:{"id":0, "text":"start"},
                   1:{"id":1, "text":"start immediately"},
                   2:{"id":2, "text":"start now"},
                   3:{"id":3, "text":"start quickly"}},
            "esp":{0:{"id":0, "text":"comienza"},
                   1:{"id":1, "text":"comienza inmediatamente"},
                   2:{"id":2, "text":"inizia ahora"}},
            "ita":{0:{"id":0, "text":"inizia"},
                   1:{"id":1, "text":"inizia immediatamente"},
                   2:{"id":2, "text":"inizia ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    2:  {
        "text":{
            "eng":"ahead (not urgent)",
            "esp":"adelante (no urgente)",
            "ita":"avanti (non urgente)"},
        "explicit": {
            "eng":{0:{"id":4, "text":"go ahead"},
                   1:{"id":5, "text":"ahead"}},
            "esp":{0:{"id":3, "text":"adelante"}},
            "ita":{0:{"id":3, "text":"vai avanti"},
                   1:{"id":4, "text":"avanti"}}
            },
        "implicit": {
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    3:  {
        "text":{
            "eng":"ahead (urgent)",
            "esp":"adelante (urgente)",
            "ita":"avanti (urgente)"},
        "explicit": {
            "eng":{0:{"id":4, "text":"go ahead"},
                   1:{"id":5, "text":"ahead"},
                   2:{"id":6, "text":"go ahead immediately"},
                   3:{"id":7, "text":"ahead immediately"},
                   4:{"id":8, "text":"go ahead now"},
                   5:{"id":9, "text":"ahead now"},
                   6:{"id":10, "text":"go ahead quickly"},
                   7:{"id":11, "text":"ahead quickly"}},
            "esp":{0:{"id":3, "text":"adelante"},
                   1:{"id":4, "text":"adelante inmediatamente"},
                   2:{"id":5, "text":"adelante ahora"}},
            "ita":{0:{"id":3, "text":"vai avanti"},
                   1:{"id":4, "text":"avanti"},
                   2:{"id":5, "text":"vai avanti immediatamente"},
                   3:{"id":6, "text":"avanti immediatamente"},
                   4:{"id":7, "text":"vai avanti ora"},
                   5:{"id":8, "text":"avanti ora"}}
            },
        "implicit": {
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    4:  {
        "text":{
            "eng":"back (not urgent)",
            "esp":"atrás (no urgente)",
            "ita":"indietro (non urgente)"},
        "explicit": {
            "eng":{0:{"id":12, "text":"go back"},
                   1:{"id":13, "text":"back"}},
            "esp":{0:{"id":6, "text":"regresa"},
                   1:{"id":7, "text":"ve para atrás"},
                   2:{"id":8, "text":"atrás"}},
            "ita":{0:{"id":9, "text":"vai indietro"},
                   1:{"id":10, "text":"indietro"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    5:  {
        "text":{
            "eng":"back (urgent)",
            "esp":"atrás (urgente)",
            "ita":"indietro (urgente)"},
        "explicit":{
            "eng":{0:{"id":12, "text":"go back"},
                   1:{"id":13, "text":"back"},
                   2:{"id":14, "text":"go back immediately"},
                   3:{"id":15, "text":"back immediately"},
                   4:{"id":16, "text":"go back now"},
                   5:{"id":17, "text":"back now"},
                   6:{"id":18, "text":"go back quickly"},
                   7:{"id":19, "text":"back quickly"}},
            "esp":{0:{"id":6, "text":"regresa"},
                   1:{"id":7, "text":"ve para atrás"},
                   2:{"id":8, "text":"atrás"},
                   3:{"id":9, "text":"regresa inmediatamente"},
                   4:{"id":10, "text":"ve para atrás inmediatamente"},
                   5:{"id":11, "text":"atrás inmediatamente"},
                   6:{"id":12, "text":"regresa ahora"},
                   7:{"id":13, "text":"ve para atrás ahora"},
                   8:{"id":14, "text":"atrás ahora"}},
            "ita":{0:{"id":9, "text":"vai indietro"},
                   1:{"id":10, "text":"indietro"},
                   2:{"id":11, "text":"vai indietro immediatamente"},
                   3:{"id":12, "text":"indietro immediatamente"},
                   4:{"id":13, "text":"vai indietro ora"},
                   5:{"id":14, "text":"indietro ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    6:  {
        "text":{
            "eng":"left (not urgent)",
            "esp":"izquierda (no urgente)",
            "ita":"sinistra (non urgente)"},
        "explicit":{
            "eng":{0:{"id":20, "text":"go left"},
                   1:{"id":21, "text":"left"}},
            "esp":{0:{"id":15, "text":"ve a la izquierda"},
                   1:{"id":16, "text":"izquierda"}},
            "ita":{0:{"id":15, "text":"vai a sinistra"},
                   1:{"id":16, "text":"sinistra"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    7:  {
        "text":{
            "eng":"left (not urgent)",
            "esp":"izquierda (no urgente)",
            "ita":"sinistra (non urgente)"},
        "explicit":{
            "eng":{0:{"id":20, "text":"go left"},
                   1:{"id":21, "text":"left"},
                   2:{"id":22, "text":"go left immediately"},
                   3:{"id":23, "text":"left immediately"},
                   4:{"id":24, "text":"go left now"},
                   5:{"id":25, "text":"left now"},
                   6:{"id":26, "text":"go left quickly"},
                   7:{"id":27, "text":"left quickly"}},
            "esp":{0:{"id":15, "text":"ve a la izquierda"},
                   1:{"id":16, "text":"izquierda"},
                   2:{"id":17, "text":"ve a la izquierda inmediatamente"},
                   3:{"id":18, "text":"izquierda inmediatamente"},
                   4:{"id":19, "text":"ve a la izquierda ahora"},
                   5:{"id":20, "text":"izquierda ahora"}},
            "ita":{0:{"id":15, "text":"vai a sinistra"},
                   1:{"id":16, "text":"sinistra"},
                   2:{"id":17, "text":"vai a sinistra immediatamente"},
                   3:{"id":18, "text":"sinistra immediatamente"},
                   4:{"id":19, "text":"vai a sinistra ora"},
                   5:{"id":20, "text":"sinistra ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    8:  {
        "text":{
            "eng":"right (not urgent)",
            "esp":"derecha (no urgente)",
            "ita":"destra (non urgente)"},
        "explicit":{
            "eng":{0:{"id":28, "text":"go right"},
                   1:{"id":29, "text":"right"}},
            "esp":{0:{"id":21, "text":"ve a la derecha"},
                   1:{"id":22, "text":"derecha"}},
            "ita":{0:{"id":21, "text":"vai a destra"},
                   1:{"id":22, "text":"destra"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    9:  {
        "text":{
            "eng":"right (urgent)",
            "esp":"derecha (urgente)",
            "ita":"destra (urgente)"},
        "explicit":{
            "eng":{0:{"id":28, "text":"go right"},
                   1:{"id":29, "text":"right"},
                   2:{"id":30, "text":"go right immediately"},
                   3:{"id":31, "text":"right immediately"},
                   4:{"id":32, "text":"go right now"},
                   5:{"id":33, "text":"right now"},
                   6:{"id":34, "text":"go right quickly"},
                   7:{"id":35, "text":"right quickly"}},
            "esp":{0:{"id":21, "text":"ve a la derecha"},
                   1:{"id":22, "text":"derecha"},
                   2:{"id":23, "text":"ve a la derecha inmediatamente"},
                   3:{"id":24, "text":"derecha inmediatamente"},
                   4:{"id":25, "text":"ve a la derecha ahora"},
                   5:{"id":26, "text":"derecha ahora"}},
            "ita":{0:{"id":21, "text":"vai a destra"},
                   1:{"id":22, "text":"destra"},
                   2:{"id":23, "text":"vai a destra immediatamente"},
                   3:{"id":24, "text":"destra immediatamente"},
                   4:{"id":25, "text":"vai a destra ora"},
                   5:{"id":26, "text":"destra ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    10: {
        "text":{
            "eng":"stop (not urgent)",
            "esp":"detente (no urgente)",
            "ita":"fermati (non urgente)"},
        "explicit":{
            "eng":{0:{"id":36, "text":"stop"}},
            "esp":{0:{"id":27, "text":"stop"},
                   1:{"id":28, "text":"detente"}},
            "ita":{0:{"id":27, "text":"stop"},
                   1:{"id":28, "text":"fermati"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    11: {
        "text":{
            "eng":"stop (urgent)",
            "esp":"detente (urgente)",
            "ita":"fermati (urgente)"},
        "explicit":{
            "eng":{0:{"id":36, "text":"stop"},
                   1:{"id":37, "text":"stop immediately"},
                   2:{"id":38, "text":"stop now"},
                   3:{"id":39, "text":"stop quickly"}},
            "esp":{0:{"id":27, "text":"stop"},
                   1:{"id":28, "text":"detente"},
                   2:{"id":29, "text":"stop inmediatamente"},
                   3:{"id":30, "text":"detente inmediatamente"},
                   4:{"id":31, "text":"stop ahora"},
                   5:{"id":32, "text":"detente ahora"}},
            "ita":{0:{"id":27, "text":"stop"},
                   1:{"id":28, "text":"fermati"},
                   2:{"id":29, "text":"stop immediatamente"},
                   3:{"id":30, "text":"fermati immediatamente"},
                   4:{"id":31, "text":"stop ora"},
                   5:{"id":32, "text":"fermati ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    12: {
        "text":{
            "eng":"come (not urgent)",
            "esp":"acercate (no urgente)",
            "ita":"vieni (non urgente)"},
        "explicit":{
            "eng":{0:{"id":40, "text":"come"},
                   1:{"id":41, "text":"come here"}},
            "esp":{0:{"id":33, "text":"ven aquì"},
                   1:{"id":34, "text":"acercate"}},
            "ita":{0:{"id":33, "text":"vieni"},
                   1:{"id":34, "text":"vieni qui"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    13: {
        "text":{
            "eng":"come (urgent)",
            "esp":"acercate (urgente)",
            "ita":"vieni (urgente)"},
        "explicit":{
            "eng":{0:{"id":40, "text":"come"},
                   1:{"id":41, "text":"come here"},
                   2:{"id":42, "text":"come immediately"},
                   3:{"id":43, "text":"come here immediately"},
                   4:{"id":44, "text":"come now"},
                   5:{"id":45, "text":"come here now"},
                   6:{"id":46, "text":"come quickly"},
                   7:{"id":47, "text":"come here quickly"}},
            "esp":{0:{"id":33, "text":"ven aqui"},
                   1:{"id":34, "text":"acercate"},
                   2:{"id":35, "text":"ven aquì inmediatamente"},
                   3:{"id":36, "text":"acercate inmediatamente"},
                   4:{"id":37, "text":"ven aquì ahora"},
                   5:{"id":38, "text":"acercate ahora"}},
            "ita":{0:{"id":33, "text":"vieni"},
                   1:{"id":34, "text":"vieni qui"},
                   2:{"id":35, "text":"vieni immediatamente"},
                   3:{"id":36, "text":"vieni qui immediatamente"},
                   4:{"id":37, "text":"vieni qui ora"},
                   5:{"id":38, "text":"vieni ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    14: {
        "text":{
            "eng":"release (not urgent)",
            "esp":"suelta (no urgente)",
            "ita":"rilascia (non urgente)"},
        "explicit":{
            "eng":{0:{"id":48, "text":"release"},
                   1:{"id":49, "text":"drop"}},
            "esp":{0:{"id":39, "text":"suelta"},
                   1:{"id":40, "text":"deja"}},
            "ita":{0:{"id":39, "text":"rilascia"},
                   1:{"id":40, "text":"lascia"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    15: {
        "text":{
            "eng":"release (urgent)",
            "esp":"suelta (urgente)",
            "ita":"rilascia (urgente)"},
        "explicit":{
            "eng":{0:{"id":48, "text":"release"},
                   1:{"id":49, "text":"drop"},
                   2:{"id":50, "text":"release immediately"},
                   3:{"id":51, "text":"drop immediately"},
                   4:{"id":52, "text":"release now"},
                   5:{"id":53, "text":"drop now"},
                   6:{"id":54, "text":"release quickly"},
                   7:{"id":55, "text":"drop quickly"}},
            "esp":{0:{"id":39, "text":"suelta"},
                   1:{"id":40, "text":"deja"},
                   2:{"id":41, "text":"suelta inmediatamente"},
                   3:{"id":42, "text":"deja inmediatamente"},
                   4:{"id":43, "text":"suelta ahora"},
                   5:{"id":44, "text":"deja ahora"}},
            "ita":{0:{"id":39, "text":"rilascia"},
                   1:{"id":40, "text":"lascia"},
                   2:{"id":41, "text":"rilascia immediatamente"},
                   3:{"id":42, "text":"lascia immediatamente"},
                   4:{"id":43, "text":"rilascia ora"},
                   5:{"id":44, "text":"lascia ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    16: {
        "text":{
            "eng":"fast (not urgent)",
            "esp":"rapido (no urgente)",
            "ita":"veloce (non urgente)"},
        "explicit":{
            "eng":{0:{"id":56, "text":"fast"},
                   1:{"id":57, "text":"faster"},
                   2:{"id":58, "text":"quickly"}},
            "esp":{0:{"id":45, "text":"rapido"},
                   1:{"id":46, "text":"más rapido"},
                   2:{"id":47, "text":"rapidamente"}},
            "ita":{0:{"id":45, "text":"veloce"},
                   1:{"id":46, "text":"più veloce"},
                   2:{"id":47, "text":"velocemente"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    17: {
        "text":{
            "eng":"slow (not urgent)",
            "esp":"lento (no urgente)",
            "ita":"lento (non urgente)"},
        "explicit":{
            "eng":{0:{"id":59, "text":"slow"},
                   1:{"id":60, "text":"slower"},
                   2:{"id":61, "text":"slowly"}},
            "ita":{0:{"id":48, "text":"lento"},
                   1:{"id":49, "text":"più lento"},
                   2:{"id":50, "text":"lentamente"}},
            "esp":{0:{"id":48, "text":"lento"},
                   1:{"id":49, "text":"más lento"},
                   2:{"id":50, "text":"lentamente"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    18: {
        "text":{
            "eng":"okay (not urgent)",
            "esp":"vale (no urgente)",
            "ita":"va bene (non urgente)"},
        "explicit":{
            "eng":{0:{"id":62, "text":"yes"},
                   1:{"id":63, "text":"okay"}},
            "esp":{0:{"id":51, "text":"si"},
                   1:{"id":52, "text":"vale"}},
            "ita":{0:{"id":51, "text":"si"},
                   1:{"id":52, "text":"va bene"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    19: {
        "text":{
            "eng":"no (not urgent)",
            "esp":"no (no urgente)",
            "ita":"no (non urgente)"},
        "explicit":{
            "eng":{0:{"id":64, "text":"no"}},
            "esp":{0:{"id":53, "text":"no"}},
            "ita":{0:{"id":53, "text":"no"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    20: {
        "text":{
            "eng":"i don't know (not urgent)",
            "esp":"no sé (no urgente)",
            "ita":"non lo so (non urgente)"},
        "explicit":{
            "eng":{0:{"id":65, "text":"maybe"},
                   1:{"id":66, "text":"i don't know"}},
            "esp":{0:{"id":54, "text":"tal vez"},
                   1:{"id":55, "text":"no sé"}},
            "ita":{0:{"id":54, "text":"forse"},
                   1:{"id":55, "text":"non lo so"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    21: {
        "text":{
            "eng":"help me (not urgent)",
            "esp":"ayùdame (no urgente)",
            "ita":"aiutami (non urgente)"},
        "explicit":{
            "eng":{0:{"id":67, "text":"help me"}},
            "esp":{0:{"id":56, "text":"ayùdame"}},
            "ita":{0:{"id":56, "text":"aiutami"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    22: {
        "text":{
            "eng":"help me (urgent)",
            "esp":"ayùdame (urgente)",
            "ita":"aiutami (urgente)"},
        "explicit":{
            "eng":{0:{"id":67, "text":"help me"},
                   1:{"id":68, "text":"help me immediately"},
                   2:{"id":69, "text":"help me now"},
                   3:{"id":70, "text":"help me quickly"}},
            "esp":{0:{"id":56, "text":"ayùdame"},
                   1:{"id":57, "text":"ayùdame inmediatamente"},
                   2:{"id":58, "text":"ayùdame ahora"}},
            "ita":{0:{"id":56, "text":"aiutami"},
                   1:{"id":57, "text":"aiutami immediatamente"},
                   2:{"id":58, "text":"aiutami ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        }
}

EXPLICIT_INTENTS:dict = {
    "eng":  {
        0:"start",
        1:"start immediately",
        2:"start now",
        3:"start quickly",
        4:"go ahead",
        5:"ahead",
        6:"go ahead immediately",
        7:"ahead immediately",
        8:"go ahead now",
        9:"ahead now",
        10:"go ahead quickly",
        11:"ahead quickly",
        12:"go back",
        13:"back",
        14:"go back immediately",
        15:"back immediately",
        16:"go back now",
        17:"back now",
        18:"go back quickly",
        19:"back quickly",
        20:"go left",
        21:"left",
        22:"go left immediately",
        23:"left immediately",
        24:"go left now",
        25:"left now",
        26:"go left quickly",
        27:"left quickly",
        28:"go right",
        29:"right",
        30:"go right immediately",
        31:"right immediately",
        32:"go right now",
        33:"right now",
        34:"go right quickly",
        35:"right quickly",
        36:"stop",
        37:"stop immediately",
        38:"stop now",
        39:"stop quickly",
        40:"come",
        41:"come here",
        42:"come immediately",
        43:"come here immediately",
        44:"come now",
        45:"come here now",
        46:"come quickly",
        47:"come here quickly",
        48:"release",
        49:"drop",
        50:"release immediately",
        51:"drop immediately",
        52:"release now",
        53:"drop now",
        54:"release quickly",
        55:"drop quickly",
        56:"fast",
        57:"faster",
        58:"quickly",
        59:"slow",
        60:"slower",
        61:"slowly",
        62:"yes",
        63:"okay",
        64:"no",
        65:"maybe",
        66:"i don't know",
        67:"help me",
        68:"help me immediately",
        69:"help me now",
        70:"help me quickly"
    },
    "esp":  {
        0:"comienza",
        1:"comienza inmediatamente",
        2:"comienza ahora",
        3:"adelante",
        4:"adelante inmediatamente",
        5:"adelante ahora",
        6:"regresa",
        7:"ve para atrás",
        8:"atrás",
        9:"regresa inmediatamente",
        10:"ve para atrás inmediatamente",
        11:"atrás inmediatamente",
        12:"regresa ahora",
        13:"ve para atrás ahora",
        14:"atrás ahora",
        15:"ve a la izquierda",
        16:"izquierda",
        17:"ve a la izquierda inmediatamente",
        18:"izquierda inmediatamente",
        19:"ve a la izquierda ahora",
        20:"izquierda ahora",
        21:"ve a la derecha",
        22:"derecha",
        23:"ve a la derecha inmediatamente",
        24:"derecha inmediatamente",
        25:"ve a la derecha ahora",
        26:"derecha ahora",
        27:"stop",
        28:"detente",
        29:"stop inmediatamente",
        30:"detente inmediatamente",
        31:"stop ahora",
        32:"detente ahora",
        33:"acercate",
        34:"ven aquì",
        35:"acercate inmediatamente",
        36:"ven aquì inmediatamente",
        37:"acercate ahora",
        38:"ven aquì ahora",
        39:"suelta",
        40:"deja",
        41:"suelta inmediatamente",
        42:"deja inmediatamente",
        43:"suelta ahora",
        44:"deja ahora",
        45:"rapido",
        46:"mas rapido",
        47:"rapidamente",
        48:"lento",
        49:"mas lento",
        50:"lentamente",
        51:"si",
        52:"vale",
        53:"no",
        54:"tal vez",
        55:"no sé",
        56:"ayùdame",
        57:"ayùdame inmediatamente",
        58:"ayùdame ahora"
    },
    "ita":  {
        0:"inizia",
        1:"inizia immediatamente",
        2:"inizia ora",
        3:"vai avanti",
        4:"avanti",
        5:"vai avanti immediatamente",
        6:"avanti immediatamente",
        7:"vai avanti ora",
        8:"avanti ora",
        9:"vai indietro",
        10:"indietro",
        11:"vai indietro immediatamente",
        12:"indietro immediatamente",
        13:"vai indietro ora",
        14:"indietro ora",
        15:"vai a sinistra",
        16:"sinistra",
        17:"vai a sinistra immediatamente",
        18:"sinistra immediatamente",
        19:"vai a sinistra ora",
        20:"sinistra ora",
        21:"vai a destra",
        22:"destra",
        23:"vai a destra immediatamente",
        24:"destra immediatamente",
        25:"vai a destra ora",
        26:"destra ora",
        27:"stop",
        28:"fermati",
        29:"stop immediatamente",
        30:"fermati immediatamente",
        31:"stop ora",
        32:"fermati ora",
        33:"vieni",
        34:"vieni qui",
        35:"vieni immediatamente",
        36:"vieni qui immediatamente",
        37:"vieni ora",
        38:"vieni qui ora",
        39:"rilascia",
        40:"lascia",
        41:"rilascia immediatamente",
        42:"lascia immediatamente",
        43:"rilascia ora",
        44:"lascia ora",
        45:"veloce",
        46:"più veloce",
        47:"velocemente",
        48:"lento",
        49:"più lento",
        50:"lentamente",
        51:"si",
        52:"va bene",
        53:"no",
        54:"forse",
        55:"non lo so",
        56:"aiutami",
        57:"aiutami immediatamente",
        58:"aiutami ora"
    }
}

IMPLICIT_INTENTS:dict = {
    0:{
        "eng":"not urgent",
        "esp":"no urgente",
        "ita":"non urgente"},
    1:{
        "eng":"urgent",
        "esp":"urgente",
        "ita":"urgente"}
}