from math import pi


INTENTS = {
    0:  {
        "explicit":{
            "eng":{0:{"id":0, "text":"start"}},
            "esp":{0:{"id":0, "text":"comienza"}},
            "ita":{0:{"id":0, "text":"inizia"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    1:  {
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
        "explicit": {
            "eng":{0:{"id":12, "text":"go back"},
                   1:{"id":13, "text":"back"}},
            "esp":{0:{"id":6, "text":"regresa"},
                   1:{"id":7, "text":"ve para atras"},
                   2:{"id":8, "text":"atras"}},
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
                   1:{"id":7, "text":"ve para atras"},
                   2:{"id":8, "text":"atras"},
                   3:{"id":9, "text":"regresa inmediatamente"},
                   4:{"id":10, "text":"ve para atras inmediatamente"},
                   5:{"id":11, "text":"atras inmediatamente"},
                   6:{"id":12, "text":"regresa ahora"},
                   7:{"id":13, "text":"ve para atras ahora"},
                   8:{"id":14, "text":"atras ahora"}},
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
        "explicit":{
            "eng":{0:{"id":40, "text":"come"},
                   1:{"id":41, "text":"come here"}},
            "esp":{0:{"id":33, "text":"ven aqui"},
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
                   2:{"id":35, "text":"ven aqui inmediatamente"},
                   3:{"id":36, "text":"acercate inmediatamente"},
                   4:{"id":37, "text":"ven aqui ahora"},
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
        "explicit":{
            "eng":{0:{"id":56, "text":"fast"},
                   1:{"id":57, "text":"faster"},
                   2:{"id":58, "text":"quickly"}},
            "esp":{0:{"id":45, "text":"rapido"},
                   1:{"id":46, "text":"mas rapido"},
                   2:{"id":47, "text":"rapidamente"}},
            "ita":{0:{"id":45, "text":"veloce"},
                   1:{"id":46, "text":"piu veloce"},
                   2:{"id":47, "text":"velocemente"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    17: {
        "explicit":{
            "eng":{0:{"id":59, "text":"slow"},
                   1:{"id":60, "text":"slower"},
                   2:{"id":61, "text":"slowly"}},
            "ita":{0:{"id":48, "text":"lento"},
                   1:{"id":49, "text":"piu lento"},
                   2:{"id":50, "text":"lentamente"}},
            "esp":{0:{"id":48, "text":"lento"},
                   1:{"id":49, "text":"mas lento"},
                   2:{"id":50, "text":"lentamente"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    18: {
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
        "explicit":{
            "eng":{0:{"id":65, "text":"maybe"},
                   1:{"id":66, "text":"i don't know"}},
            "esp":{0:{"id":54, "text":"tal vez"},
                   1:{"id":55, "text":"no se"}},
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
        "explicit":{
            "eng":{0:{"id":67, "text":"help me"}},
            "esp":{0:{"id":56, "text":"ayudame"}},
            "ita":{0:{"id":56, "text":"aiutami"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    22: {
        "explicit":{
            "eng":{0:{"id":67, "text":"help me"},
                   1:{"id":68, "text":"help me immediately"},
                   2:{"id":69, "text":"help me now"},
                   3:{"id":70, "text":"help me quickly"}},
            "esp":{0:{"id":56, "text":"ayudame"},
                   1:{"id":57, "text":"ayudame inmediatamente"},
                   2:{"id":58, "text":"ayudame ahora"}},
            "ita":{0:{"id":56, "text":"aiutami"},
                   1:{"id":57, "text":"aiutami immediatamente"},
                   2:{"id":58, "text":"aiutami ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    23: {
        "explicit":{
            "eng":{0:{"id":71, "text":"chatter"}},
            "esp":{0:{"id":59, "text":"charla"}},
            "ita":{0:{"id":59, "text":"chiacchiere"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"no urgente"},
            "ita":{"id":0, "text":"non urgente"}
            }
    }
}
EXPLICIT_INTENTS = {
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
        7:"ve para atras",
        8:"atras",
        9:"regresa inmediatamente",
        10:"ve para atras inmediatamente",
        11:"atras inmediatamente",
        12:"regresa ahora",
        13:"ve para atras ahora",
        14:"atras ahora",
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
        34:"ven aqui",
        35:"acercate inmediatamente",
        36:"ven aqui inmediatamente",
        37:"acercate ahora",
        38:"ven aqui ahora",
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
        55:"no se",
        56:"ayudame",
        57:"ayudame inmediatamente",
        58:"ayudame ahora"
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
        46:"piu veloce",
        47:"velocemente",
        48:"lento",
        49:"piu lento",
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
IMPLICIT_INTENTS = {
    0:  {"eng":"not urgent",
         "esp":"not urgente",
         "ita":"not urgente"},
    1:  {"eng":"urgent",
         "esp":"urgente",
         "ita":"urgente"}
}


INTENTS_MSIEXP0 = {
    0   :   {"eng": {0:"go left",
                     1:"left",
                     2:"go left immediately",
                     3:"left immediately",
                     4:"go left now",
                     5:"left now",
                     6:"go left quickly",
                     7:"left quickly"},
             "ita": {0:"vai a sinistra",
                     1:"sinistra",
                     2:"vai a sinistra immediatamente",
                     3:"sinistra immediatamente",
                     4:"vai a sinistra ora",
                     5:"sinistra ora"},
             "esp": {0:"ve a la izquierda",
                     1:"izquierda",
                     2:"ve a la izquierda inmediatamente",
                     3:"izquierda inmediatamente",
                     4:"ve a la izquierda ahora",
                     5:"izquierda ahora"}},
    1   :   {"eng": {0:"go ahead",
                     1:"ahead",
                     2:"go ahead immediately",
                     3:"ahead immediately",
                     4:"go ahead now",
                     5:"ahead now",
                     6:"go ahead quickly",
                     7:"ahead quickly"},
             "ita": {0:"vai avanti",
                     1:"avanti",
                     2:"vai avanti immediatamente",
                     3:"avanti immediatamente",
                     4:"vai avanti ora",
                     5:"avanti ora"},
             "esp": {0:"adelante",
                     1:"adelante inmediatamente",
                     2:"adelante ahora"}},
    2   :   {"eng": {0:"go right",
                     1:"right",
                     2:"go right immediately",
                     3:"right immediately",
                     4:"go right now",
                     5:"right now",
                     6:"go right quickly",
                     7:"right quickly"},
             "ita": {0:"vai a destra",
                     1:"destra",
                     2:"vai a destra immediatamente",
                     3:"destra immediatamente",
                     4:"vai a destra ora",
                     5:"destra ora"},
             "esp": {0:"ve a la derecha",
                     1:"derecha",
                     2:"ve a la derecha inmediatamente",
                     3:"derecha inmediatamente",
                     4:"ve a la derecha ahora",
                     5:"derecha ahora"}},
    3  :   {"eng": {0:"stop",
                     1:"stop immediately",
                     2:"stop now",
                     3:"stop quickly"},
             "ita": {0:"stop",
                     1:"fermati",
                     2:"stop immediatamente",
                     3:"fermati immediatamente",
                     4:"stop ora",
                     5:"fermati ora"},
             "esp": {0:"stop",
                     1:"detente",
                     2:"stop inmediatamente",
                     3:"detente inmediatamente",
                     4:"stop ahora",
                     5:"detente ahora"}},
    4   :  {"eng":  {0:"chatter"},
            "esp":  {0:"charla"},
            "ita":  {0:"chiacchiere"}}}
CONVERSION_DICT_MSIEXP0 = {
    6:0,
    7:0,
    2:1,
    3:1,
    8:2,
    9:2,
    10:3,
    11:3
}


INTENTS_MSIEXP1 = {
    0:  {
        "text":{"eng":"ahead (not urgent)",
                "esp":"adelante (no urgente)",
                "ita":"avanti (non urgente)"},
        "explicit": {
            "eng":{0:{"id":0, "text":"go ahead"},
                   1:{"id":1, "text":"ahead"}},
            "esp":{0:{"id":0, "text":"adelante"}},
            "ita":{0:{"id":0, "text":"vai avanti"},
                   1:{"id":1, "text":"avanti"}}
            },
        "implicit": {
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    1:  {
        "text":{"eng":"ahead (urgent)",
                "esp":"adelante (urgente)",
                "ita":"avanti (urgente)"},
        "explicit": {
            "eng":{0:{"id":0, "text":"go ahead"},
                   1:{"id":1, "text":"ahead"},
                   2:{"id":2, "text":"go ahead immediately"},
                   3:{"id":3, "text":"ahead immediately"},
                   4:{"id":4, "text":"go ahead now"},
                   5:{"id":5, "text":"ahead now"},
                   6:{"id":6, "text":"go ahead quickly"},
                   7:{"id":7, "text":"ahead quickly"}},
            "esp":{0:{"id":0, "text":"adelante"},
                   1:{"id":1, "text":"adelante inmediatamente"},
                   2:{"id":2, "text":"adelante ahora"}},
            "ita":{0:{"id":0, "text":"vai avanti"},
                   1:{"id":1, "text":"avanti"},
                   2:{"id":2, "text":"vai avanti immediatamente"},
                   3:{"id":3, "text":"avanti immediatamente"},
                   4:{"id":4, "text":"vai avanti ora"},
                   5:{"id":5, "text":"avanti ora"}}
            },
        "implicit": {
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    2:  {
        "text":{"eng":"back (not urgent)",
                "esp":"atras (no urgente)",
                "ita":"indietro (non urgente)"},
        "explicit": {
            "eng":{0:{"id":8, "text":"go back"},
                   1:{"id":9, "text":"back"}},
            "esp":{0:{"id":3, "text":"regresa"},
                   1:{"id":4, "text":"ve para atras"},
                   2:{"id":5, "text":"atras"}},
            "ita":{0:{"id":6, "text":"vai indietro"},
                   1:{"id":7, "text":"indietro"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    3:  {
        "text":{"eng":"back (urgent)",
                "esp":"atras (urgente)",
                "ita":"indietro (urgente)"},
        "explicit":{
            "eng":{0:{"id":8, "text":"go back"},
                   1:{"id":9, "text":"back"},
                   2:{"id":10, "text":"go back immediately"},
                   3:{"id":11, "text":"back immediately"},
                   4:{"id":12, "text":"go back now"},
                   5:{"id":13, "text":"back now"},
                   6:{"id":14, "text":"go back quickly"},
                   7:{"id":15, "text":"back quickly"}},
            "esp":{0:{"id":3, "text":"regresa"},
                   1:{"id":4, "text":"ve para atras"},
                   2:{"id":5, "text":"atras"},
                   3:{"id":6, "text":"regresa inmediatamente"},
                   4:{"id":7, "text":"ve para atras inmediatamente"},
                   5:{"id":8, "text":"atras inmediatamente"},
                   6:{"id":9, "text":"regresa ahora"},
                   7:{"id":10, "text":"ve para atras ahora"},
                   8:{"id":11, "text":"atras ahora"}},
            "ita":{0:{"id":6, "text":"vai indietro"},
                   1:{"id":7, "text":"indietro"},
                   2:{"id":8, "text":"vai indietro immediatamente"},
                   3:{"id":9, "text":"indietro immediatamente"},
                   4:{"id":10, "text":"vai indietro ora"},
                   5:{"id":11, "text":"indietro ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    4:  {
        "text":{"eng":"left (not urgent)",
                "esp":"izquierda (no urgente)",
                "ita":"sinistra (non urgente)"},
        "explicit":{
            "eng":{0:{"id":16, "text":"go left"},
                   1:{"id":17, "text":"left"}},
            "esp":{0:{"id":12, "text":"ve a la izquierda"},
                   1:{"id":13, "text":"izquierda"}},
            "ita":{0:{"id":12, "text":"vai a sinistra"},
                   1:{"id":13, "text":"sinistra"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    5:  {
        "text":{"eng":"left (urgent)",
                "esp":"izquierda (urgente)",
                "ita":"sinistra (urgente)"},
        "explicit":{
            "eng":{0:{"id":16, "text":"go left"},
                   1:{"id":17, "text":"left"},
                   2:{"id":18, "text":"go left immediately"},
                   3:{"id":19, "text":"left immediately"},
                   4:{"id":20, "text":"go left now"},
                   5:{"id":21, "text":"left now"},
                   6:{"id":22, "text":"go left quickly"},
                   7:{"id":23, "text":"left quickly"}},
            "esp":{0:{"id":12, "text":"ve a la izquierda"},
                   1:{"id":13, "text":"izquierda"},
                   2:{"id":14, "text":"ve a la izquierda inmediatamente"},
                   3:{"id":15, "text":"izquierda inmediatamente"},
                   4:{"id":16, "text":"ve a la izquierda ahora"},
                   5:{"id":17, "text":"izquierda ahora"}},
            "ita":{0:{"id":12, "text":"vai a sinistra"},
                   1:{"id":13, "text":"sinistra"},
                   2:{"id":14, "text":"vai a sinistra immediatamente"},
                   3:{"id":15, "text":"sinistra immediatamente"},
                   4:{"id":16, "text":"vai a sinistra ora"},
                   5:{"id":17, "text":"sinistra ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    6:  {
        "text":{"eng":"right (not urgent)",
                "esp":"derecha (no urgente)",
                "ita":"destra (non urgente)"},
        "explicit":{
            "eng":{0:{"id":24, "text":"go right"},
                   1:{"id":25, "text":"right"}},
            "esp":{0:{"id":18, "text":"ve a la derecha"},
                   1:{"id":19, "text":"derecha"}},
            "ita":{0:{"id":18, "text":"vai a destra"},
                   1:{"id":19, "text":"destra"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    7:  {
        "text":{"eng":"right (urgent)",
                "esp":"derecha (urgente)",
                "ita":"destra (urgente)"},
        "explicit":{
            "eng":{0:{"id":24, "text":"go right"},
                   1:{"id":25, "text":"right"},
                   2:{"id":26, "text":"go right immediately"},
                   3:{"id":27, "text":"right immediately"},
                   4:{"id":28, "text":"go right now"},
                   5:{"id":29, "text":"right now"},
                   6:{"id":30, "text":"go right quickly"},
                   7:{"id":31, "text":"right quickly"}},
            "esp":{0:{"id":18, "text":"ve a la derecha"},
                   1:{"id":19, "text":"derecha"},
                   2:{"id":20, "text":"ve a la derecha inmediatamente"},
                   3:{"id":21, "text":"derecha inmediatamente"},
                   4:{"id":22, "text":"ve a la derecha ahora"},
                   5:{"id":23, "text":"derecha ahora"}},
            "ita":{0:{"id":18, "text":"vai a destra"},
                   1:{"id":19, "text":"destra"},
                   2:{"id":20, "text":"vai a destra immediatamente"},
                   3:{"id":21, "text":"destra immediatamente"},
                   4:{"id":22, "text":"vai a destra ora"},
                   5:{"id":23, "text":"destra ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    8: {
        "text":{"eng":"stop (not urgent)",
                "esp":"stop (no urgente)",
                "ita":"stop (non urgente)"},
        "explicit":{
            "eng":{0:{"id":32, "text":"stop"}},
            "esp":{0:{"id":24, "text":"stop"},
                   1:{"id":25, "text":"detente"}},
            "ita":{0:{"id":24, "text":"stop"},
                   1:{"id":25, "text":"fermati"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    9: {
        "text":{"eng":"stop (urgent)",
                "esp":"stop (urgente)",
                "ita":"stop (urgente)"},
        "explicit":{
            "eng":{0:{"id":32, "text":"stop"},
                   1:{"id":33, "text":"stop immediately"},
                   2:{"id":34, "text":"stop now"},
                   3:{"id":35, "text":"stop quickly"}},
            "esp":{0:{"id":24, "text":"stop"},
                   1:{"id":25, "text":"detente"},
                   2:{"id":26, "text":"stop inmediatamente"},
                   3:{"id":27, "text":"detente inmediatamente"},
                   4:{"id":28, "text":"stop ahora"},
                   5:{"id":29, "text":"detente ahora"}},
            "ita":{0:{"id":24, "text":"stop"},
                   1:{"id":25, "text":"fermati"},
                   2:{"id":26, "text":"stop immediatamente"},
                   3:{"id":27, "text":"fermati immediatamente"},
                   4:{"id":28, "text":"stop ora"},
                   5:{"id":29, "text":"fermati ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    10: {
        "text":{"eng":"come (not urgent)",
                "esp":"acercate (no urgente)",
                "ita":"vieni qui (non urgente)"},
        "explicit":{
            "eng":{0:{"id":36, "text":"come"},
                   1:{"id":37, "text":"come here"}},
            "esp":{0:{"id":30, "text":"ven aqui"},
                   1:{"id":31, "text":"acercate"}},
            "ita":{0:{"id":30, "text":"vieni"},
                   1:{"id":31, "text":"vieni qui"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"non urgente"},
            "ita":{"id":0, "text":"no urgente"}
            }
        },
    11: {
        "text":{"eng":"come (urgent)",
                "esp":"acercate (urgente)",
                "ita":"vieni qui (urgente)"},
        "explicit":{
            "eng":{0:{"id":36, "text":"come"},
                   1:{"id":37, "text":"come here"},
                   2:{"id":38, "text":"come immediately"},
                   3:{"id":39, "text":"come here immediately"},
                   4:{"id":40, "text":"come now"},
                   5:{"id":41, "text":"come here now"},
                   6:{"id":42, "text":"come quickly"},
                   7:{"id":43, "text":"come here quickly"}},
            "esp":{0:{"id":30, "text":"ven aqui"},
                   1:{"id":31, "text":"acercate"},
                   2:{"id":32, "text":"ven aqui inmediatamente"},
                   3:{"id":33, "text":"acercate inmediatamente"},
                   4:{"id":34, "text":"ven aqui ahora"},
                   5:{"id":35, "text":"acercate ahora"}},
            "ita":{0:{"id":30, "text":"vieni"},
                   1:{"id":31, "text":"vieni qui"},
                   2:{"id":32, "text":"vieni immediatamente"},
                   3:{"id":33, "text":"vieni qui immediatamente"},
                   4:{"id":34, "text":"vieni qui ora"},
                   5:{"id":35, "text":"vieni ora"}}
            },
        "implicit":{
            "eng":{"id":1, "text":"urgent"},
            "esp":{"id":1, "text":"urgente"},
            "ita":{"id":1, "text":"urgente"}
            }
        },
    12: {
        "text":{"eng":"chatter",
                "esp":"charla",
                "ita":"chiacchiere"},
        "explicit":{
            "eng":{0:{"id":44, "text":"chatter"}},
            "esp":{0:{"id":36, "text":"charla"}},
            "ita":{0:{"id":36, "text":"chiacchiere"}}
            },
        "implicit":{
            "eng":{"id":0, "text":"not urgent"},
            "esp":{"id":0, "text":"no urgente"},
            "ita":{"id":0, "text":"non urgente"}
            }
    }
}
EXPLICIT_INTENTS_MSIEXP1 = {
    "eng":  {
        0:"go ahead",
        1:"ahead",
        2:"go ahead immediately",
        3:"ahead immediately",
        4:"go ahead now",
        5:"ahead now",
        6:"go ahead quickly",
        7:"ahead quickly",
        8:"go back",
        9:"back",
        10:"go back immediately",
        11:"back immediately",
        12:"go back now",
        13:"back now",
        14:"go back quickly",
        15:"back quickly",
        16:"go left",
        17:"left",
        18:"go left immediately",
        19:"left immediately",
        20:"go left now",
        21:"left now",
        22:"go left quickly",
        23:"left quickly",
        24:"go right",
        25:"right",
        26:"go right immediately",
        27:"right immediately",
        28:"go right now",
        29:"right now",
        30:"go right quickly",
        31:"right quickly",
        32:"stop",
        33:"stop immediately",
        34:"stop now",
        35:"stop quickly",
        36:"come",
        37:"come here",
        38:"come immediately",
        39:"come here immediately",
        40:"come now",
        41:"come here now",
        42:"come quickly",
        43:"come here quickly",
        44:"chatter"
    },
    "esp":  {
        0:"adelante",
        1:"adelante inmediatamente",
        2:"adelante ahora",
        3:"regresa",
        4:"ve para atras",
        5:"atras",
        6:"regresa inmediatamente",
        7:"ve para atras inmediatamente",
        8:"atras inmediatamente",
        9:"regresa ahora",
        10:"ve para atras ahora",
        11:"atras ahora",
        12:"ve a la izquierda",
        13:"izquierda",
        14:"ve a la izquierda inmediatamente",
        15:"izquierda inmediatamente",
        16:"ve a la izquierda ahora",
        17:"izquierda ahora",
        18:"ve a la derecha",
        19:"derecha",
        20:"ve a la derecha inmediatamente",
        21:"derecha inmediatamente",
        22:"ve a la derecha ahora",
        23:"derecha ahora",
        24:"stop",
        25:"detente",
        26:"stop inmediatamente",
        27:"detente inmediatamente",
        28:"stop ahora",
        29:"detente ahora",
        30:"acercate",
        31:"ven aqui",
        32:"acercate inmediatamente",
        33:"ven aqui inmediatamente",
        34:"acercate ahora",
        35:"ven aqui ahora",
        36:"charla"
    },
    "ita":  {
        0:"vai avanti",
        1:"avanti",
        2:"vai avanti immediatamente",
        3:"avanti immediatamente",
        4:"vai avanti ora",
        5:"avanti ora",
        6:"vai indietro",
        7:"indietro",
        8:"vai indietro immediatamente",
        9:"indietro immediatamente",
        10:"vai indietro ora",
        11:"indietro ora",
        12:"vai a sinistra",
        13:"sinistra",
        14:"vai a sinistra immediatamente",
        15:"sinistra immediatamente",
        16:"vai a sinistra ora",
        17:"sinistra ora",
        18:"vai a destra",
        19:"destra",
        20:"vai a destra immediatamente",
        21:"destra immediatamente",
        22:"vai a destra ora",
        23:"destra ora",
        24:"stop",
        25:"fermati",
        26:"stop immediatamente",
        27:"fermati immediatamente",
        28:"stop ora",
        29:"fermati ora",
        30:"vieni",
        31:"vieni qui",
        32:"vieni immediatamente",
        33:"vieni qui immediatamente",
        34:"vieni ora",
        35:"vieni qui ora",
        36:"chiacchiere"
    }
}
CONVERSION_INTENT_MSIEXP1 = {
    2:0,
    3:1,
    4:2,
    5:3,
    6:4,
    7:5,
    8:6,
    9:7,
    10:8,
    11:9,
    12:10,
    13:11
}
CONVERSION_EXPLICIT_MSIEXP1 = {
    "eng":[4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47],
    "esp":[3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38],
    "ita":[3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38]
}
INTENT_TO_ACTION = {
    0   :{"x":1.5,  "y":0.,     "speed":0.25,   "theta":0.0,    "eng":"I'm going ahead",            "ita":"Sto andando avanti"},                # ahead (not urgent)
    1   :{"x":1.5,  "y":0.,     "speed":1.,     "theta":0.0,    "eng":"I'm going ahead quickly",    "ita":"Sto andando avanti rapidamente"},    # ahead (urgent)
    2   :{"x":-1.5, "y":0.,     "speed":0.25,   "theta":pi,     "eng":"I'm going back",             "ita":"Sto andando indietro"},              # back (not urgent)
    3   :{"x":1.5,  "y":0.,     "speed":1.,     "theta":pi,     "eng":"I'm going back quickly",     "ita":"Sto andando indietro rapidamente"},  # back (urgent)
    4   :{"x":0.,   "y":1.5,    "speed":0.25,   "theta":pi/2,   "eng":"I'm going to left",          "ita":"Sto andando a sinistra"},            # left (not urgent)
    5   :{"x":0.,   "y":1.5,    "speed":1.,     "theta":pi/2,   "eng":"I'm going to left quickly",  "ita":"Sto andando a sinistra rapidamente"},# left (urgent)
    6   :{"x":0.,   "y":-1.5,   "speed":0.25,   "theta":-pi/2,  "eng":"I'm going to right",         "ita":"Sto andando a destra"},              # right (not urgent)
    7   :{"x":0.,   "y":-1.5,   "speed":1.,     "theta":-pi/2,  "eng":"I'm going to right quickly", "ita":"Sto andando a destra rapidamente"},  # right (urgent)
    8   :{"x":0.,   "y":0.,     "speed":0.25,   "theta":0.0,    "eng":"I'm stopping",               "ita":"Mi sto fermando"},                   # stop (not urgent)
    9   :{"x":0.,   "y":0.,     "speed":1.,     "theta":0.0,    "eng":"I'm stopping now",           "ita":"Mi sto fermando rapidamente"},       # stop (urgent)
    10  :{"x":0.,   "y":0.,     "speed":0.25,   "theta":0.0,    "eng":"I'm coming",                 "ita":"Sto venendo"},                       # come here
    11  :{"x":0.,   "y":0.,     "speed":1.,     "theta":0.0,    "eng":"I'm coming quickly",         "ita":"Sto venendo rapidamente"}            # come here (urgent)
}