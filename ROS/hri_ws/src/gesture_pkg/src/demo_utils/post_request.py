import json
import requests
from datetime import datetime
from colorama import Fore

from commands import GESTURE_COMMANDS

CB_HEADER = {'Content-Type': 'application/json; charset=utf-8'}


class MyRequestPost:
    def __init__(self, instance_uuid, entity, msg_type, address, port=1026):
        self.entity = entity + ":" + str(instance_uuid)
        self.msg_type = msg_type
        self.address = address
        self.port = port
        self.CB_BASE_URL = "http://{}:{}/v2/".format(self.address, self.port)

        # entity instantiation message
        msg_create = """{
                "id": "UNISA.SpeechGestureAnalysis.Gesture",
                "type": "Gesture",
                "timestamp": {
                        "type": "DateTime",
                        "value": "1995-02-22T06:30:22.12"
                },
                "command": {
                        "type": "Number",
                        "value": 13,
                        "metadata":{
                                "english":{
                                        "type": "String",
                                        "value": "No command"
                                },
                                "italian":{
                                        "type": "String",
                                        "value": "Nessun comando"
                                }
                        }
                },
                "confidence": {
                        "type": "float",
                        "value": "0.0"
                }
        }"""

        self.json_create = json.loads(msg_create)

        msg_update = """{
                "timestamp": {
                        "type": "DateTime",
                        "value": "1995-02-22T06:30:22.12"
                },
                "command": {
                        "type": "Number",
                        "value": 13,
                        "metadata":{
                                "english":{
                                        "type": "String",
                                        "value": "No command"
                                },
                                "italian":{
                                        "type": "String",
                                        "value": "Nessun comando"
                                }
                        }
                },
                "confidence": {
                        "type": "float",
                        "value": "0.0"
                }
        }"""

        # convert strint to json
        self.json_update = json.loads(msg_update)


    def create_entity(self):
        response = None
        msg = None

        self.json_create['id'] = self.entity
        self.json_create['type'] = self.msg_type
        self.json_create['timestamp']['value'] = datetime.now().isoformat()
        
        msg = json.dumps(self.json_create)
        #print(msg)

        # send request
        response = requests.post(self.CB_BASE_URL+"entities/", data=msg, headers=CB_HEADER)

        """
        if response.ok: # response successful
            print("CB response -> status " + response.status_code.__str__())
        else: # response ko
            print("CB response -> " + response.text)
        """

        
    def send_command(self, command_id, confidence):
        response = None
        msg = None

        self.json_update['timestamp']['value'] = datetime.now().isoformat()
        self.json_update['command']['value'] = int(command_id)
        self.json_update['command']['metadata']['english']['value'] = GESTURE_COMMANDS[command_id]["eng"]
        self.json_update['command']['metadata']['italian']['value'] = GESTURE_COMMANDS[command_id]["ita"]
        self.json_update['confidence']['value'] = confidence

        res_str = Fore.CYAN + '#'*6 + ' SPEECH CHUNCK  ' + '#'*6 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(GESTURE_COMMANDS[command_id]["eng"], confidence) + Fore.CYAN + ' #\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(GESTURE_COMMANDS[command_id]["ita"], confidence) + Fore.CYAN + ' #\n' + '#'* 44 + Fore.RESET + '\n'
        print(res_str)

        msg = json.dumps(self.json_update)
        #print(msg)

        # send request
        response = requests.post(self.CB_BASE_URL+"entities/{}/attrs".format(self.entity), data=msg, headers=CB_HEADER)

        """
        if response.ok: # response successful
            print("CB response (COMMAND msg) -> status " + response.status_code.__str__())
        else: # response ko
            print("CB response (COMMAND msg) -> " + response.text)
        #"""