import json
import requests
import datetime

CB_HEADER = {'Content-Type': 'application/json; charset=utf-8'}
CB_BASE_URL = None
address = None
port = None
json_create= None
json_update = None
MSG_FILE = './_requests/entity_message.json'
CB_PORT = '1026'
CB_ADDRESS = '25.45.111.204'

commands = {
    13: {'label':13, 'english':'Bring me the screwdriver', 'italian':'Portami il cacciavite'},
    21: {'label':21, 'english':'Release', 'italian':'Rilascia'},
    24: {'label':24, 'english':'Start', 'italian':'Start'}
}

# url create instance
# 'entities/.turtle1.cmd_vel/attrs'

# url update instance
# 'entities/.turtle1.cmd_vel/attrs'


class MyRequestPost:
    def __init__(self, address, port=1026, entities=dict()):
        self.address = address
        self.port = port
        self.CB_BASE_URL = "http://{}:{}/v2/".format(self.address, self.port)
        self.entities = entities

        # entity instantiation message
        msg_create = """{
                "id": "UNISA.SpeechGestureAnalysis.Speech",
                "type": "Speech",
                "timestamp": {
                        "type": "DateTime",
                        "value": "1995-02-22T06:30:22.12+01:00"
                },
                "command": {
                        "type": "Number",
                        "value": 29,
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
                        "value": "1995-02-22T06:30:22.12+01:00"
                },
                "command": {
                        "type": "Number",
                        "value": 29,
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


    def create_entity(self, command):
        self.json_create['id'] = self.entities[command]['entity']
        self.json_create['type'] = self.entities[command]['type']
        
        msg = json.dumps(self.json_create)

        # send request
        response = requests.post(self.CB_BASE_URL+"entities/", data = msg, headers = CB_HEADER)

        if response.ok: # response successful
            print("CB response -> status " + response.status_code.__str__())
        else: # response ko
            print("CB response -> " + response.text)


    def send_command(self, command_id, confidence):
        response = None
        msg = None
        entity = self.json_create['id']
        
        self.json_update['timestamp']['value'] = datetime.datetime.now().isoformat()
        self.json_update['command']['value'] = commands[command_id]['label']
        self.json_update['command']['metadata']['english']['value'] = commands[command_id]['english']
        self.json_update['command']['metadata']['italian']['value'] = commands[command_id]['italian']
        self.json_update['confidence']['value'] = confidence

        msg = json.dumps(self.json_update)

        # send request
        response = requests.post(self.CB_BASE_URL+"entities/{}/attrs".format(entity), data = msg, headers = CB_HEADER)

        if response.ok: # response successful
            print("CB response -> status " + response.status_code.__str__())
        else: # response ko
            print("CB response -> " + response.text)