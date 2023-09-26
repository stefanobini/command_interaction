import json
import requests
from datetime import datetime


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
                "id": "UNISA.SpeechGestureAnalysis.SystemHealth",
                "type": "Speech",
                "timestamp": {
                        "type": "DateTime",
                        "value": "1995-02-22T06:30:22.12"
                },
                "status": {
                        "type": "string",
                        "value": "Ok"
                }
        }"""

        self.json_create = json.loads(msg_create)

        msg_update = """{
                "timestamp": {
                        "type": "DateTime",
                        "value": "1995-02-22T06:30:22.12"
                },
                "status": {
                        "type": "string",
                        "value": "Ok"
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
        self.json_update['status']['value'] = "Ok"
        
        msg = json.dumps(self.json_create)
        #print(msg)

        # send request
        response = requests.post(self.CB_BASE_URL+"entities/", data=msg, headers=CB_HEADER)

        """
        if response.ok: # response successful
            #print("CB response -> status " + response.status_code.__str__())
        else: # response ko
            #print("CB response -> " + response.text)
        """
        
        
    def send_alive(self, status):
        response = None
        msg = None
        
        self.json_update['timestamp']['value'] = datetime.now().isoformat()
        self.json_update['status']['value'] = status

        msg = json.dumps(self.json_update)
        #print(msg)

        # send request
        response = requests.post(self.CB_BASE_URL+"entities/{}/attrs".format(self.entity), data=msg, headers=CB_HEADER)

        """
        if response.ok: # response successful
            #print("CB response (ALIVE msg) -> status " + response.status_code.__str__())
        else: # response ko
            #print("CB response (ALIVE msg) -> " + response.text)
        """