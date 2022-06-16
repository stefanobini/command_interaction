import json
import requests
import time

CB_HEADER = {'Content-Type': 'application/json'}
CB_BASE_URL = None
address = None
port = None
json_create= None
json_update = None

commands = {
    1: {'label':1, 'english':'Increase the height', 'italian':'Più alto'},
    2: {'label':2, 'english':'Decrease the height', 'italian':'Più basso'},
    3: {'label':3, 'english':'Bring me the screwdriver', 'italian':'Portami il cacciavite'},
    4: {'label':4, 'english':'Take the screwdriver', 'italian':'Prendi il cacciavite'},
    5: {'label':5, 'english':'Bring me the windows control panel', 'italian':'Portami la mostrina comandi'},
    6: {'label':6, 'english':'Take the windows control panel', 'italian':'Prendi la mostrina comandi'},
    7: {'label':7, 'english':'Bring me the rearview mirror', 'italian':'Portami lo specchietto retrovisore'},
    8: {'label':8, 'english':'Take the rearview mirror', 'italian':'Prendi lo specchietto retrovisore'},
    9: {'label':9, 'english':'Release', 'italian':'Rilascia'},
}

# url create instance
# 'entities/.turtle1.cmd_vel/attrs'

# url update instance
# 'entities/.turtle1.cmd_vel/attrs'


class MyRequestPost:
    def __init__(self, address, port=1026):
        self.address = address
        self.port = port
        self.CB_BASE_URL = "http://{}:{}/v2/".format(self.address, self.port)

        # entity instantiation message
        msg_create = """{
                \"id\": {
                        \"type\": \"string\",
                        \"value\": \"UNISA.SpeechGestureAnalysis.Speech\",
                },
                \"type\": {
                        \"type\": \"string\",
                        \"value\": \"Speech\",
                },
                \"timestamp\": {
                        \"type\": \"DateTime\",
                        \"value\": \"1995-02-22T06:30:22.08+01:00\",
                },
                \"command\": {
                        \"type\": {
                                \"label\": \"LabelType\",
                                \"english\": \"EnglishType\",
                                \"italian\": \"ItalianType\",
                        },
                        \"value\": {
                                \"label\": 29,
                                \"english\": \"No command\",
                                \"italian\": \"Nessun comando\",
                        },
                },
                \"confidence\": {
                        \"type\": \"float\",
                        \"value\": \"0.0\",
                },
        }"""

        self.json_create = json.loads(msg_create)

        msg_update = """{
                \"timestamp\": {
                        \"type\": \"DateTime\",
                        \"value\": \"1995-02-22T06:30:22.08+01:00\",
                },
                \"command\": {
                        \"type\": {
                                \"label\": \"LabelType\",
                                \"english\": \"EnglishType\",
                                \"italian\": \"ItalianType\",
                        },
                        \"value\": {
                                \"label\": 29,
                                \"english\": \"No command\",
                                \"italian\": \"Nessun comando\",
                        },
                },
                \"confidence\": {
                        \"type\": \"float\",
                        \"value\": \"0.0\",
                },
        }"""

        # convert strint to json
        self.json_update = json.loads(msg_update)

        '''
        msg_create = "{\n  \"id\": \".turtle1.cmd_vel\",\n  \"type\": \"geometry_msgs%2FTwist\",\n  \"linear\": {\n   " \
                     " \"type\": \"geometry_msgs%2FVector3\",\n    \"value\": {\n      \"y\": {\n        \"type\": " \
                     "\"number\",\n        \"value\": \"0\"\n      },\n      \"x\": {\n        \"type\": \"number\"," \
                     "\n        \"value\": \"0\"\n      },\n      \"z\": {\n        \"type\": \"number\"," \
                     "\n        \"value\": \"0\"\n      }\n    },\n    \"metadata\": {\n      \"dataType\": {\n       " \
                     " \"type\": \"dataType\",\n        \"value\": {\n          \"y\": \"float64\",\n          \"x\": " \
                     "\"float64\",\n          \"z\": \"float64\"\n        }\n      }\n    }\n  },\n  \"angular\": {\n " \
                     "   \"type\": \"geometry_msgs%2FVector3\",\n    \"value\": {\n      \"y\": {\n        \"type\": " \
                     "\"number\",\n        \"value\": \"0\"\n      },\n      \"x\": {\n        \"type\": \"number\"," \
                     "\n        \"value\": \"0\"\n      },\n      \"z\": {\n        \"type\": \"number\"," \
                     "\n        \"value\": \"0\"\n      }\n    },\n    \"metadata\": {\n      \"dataType\": {\n       " \
                     " \"type\": \"dataType\",\n        \"value\": {\n          \"y\": \"float64\",\n          \"x\": " \
                     "\"float64\",\n          \"z\": \"float64\"\n        }\n      }\n    }\n  }\n} "
        

        self.json_create = json.loads(msg_create)

        # entity update message
        msg_update = "{\n  \"linear\": {\n    \"type\": \"geometry_msgs%2FVector3\",\n    \"value\": {\n      \"y\": " \
                     "{\n        \"type\": \"number\",\n        \"value\": \"0\"\n      },\n      \"x\": {\n        " \
                     "\"type\": \"number\",\n        \"value\": \"0\"\n      },\n      \"z\": {\n        \"type\": " \
                     "\"number\",\n        \"value\": \"0\"\n      }\n    },\n    \"metadata\": {\n      " \
                     "\"dataType\": {\n        \"type\": \"dataType\",\n        \"value\": {\n          \"y\": " \
                     "\"float64\",\n          \"x\": \"float64\",\n          \"z\": \"float64\"\n        }\n      }\n " \
                     "   }\n  },\n  \"angular\": {\n    \"type\": \"geometry_msgs%2FVector3\",\n    \"value\": {\n    " \
                     "  \"y\": {\n        \"type\": \"number\",\n        \"value\": \"0\"\n      },\n      \"x\": {\n " \
                     "       \"type\": \"number\",\n        \"value\": \"0\"\n      },\n      \"z\": {\n        " \
                     "\"type\": \"number\",\n        \"value\": \"0\"\n      }\n    },\n    \"metadata\": {\n      " \
                     "\"dataType\": {\n        \"type\": \"dataType\",\n        \"value\": {\n          \"y\": " \
                     "\"float64\",\n          \"x\": \"float64\",\n          \"z\": \"float64\"\n        }\n      }\n " \
                     "   }\n  }\n} "


        # convert strint to json
        self.json_update = json.loads(msg_update)
        '''
    def send_command(self, command_id, confidence):
        response = None
        msg = None
        
        self.json_create['timestamp'] = time.time()
        self.json_create['command']['label'] = commands[command_id]['label']
        self.json_create['command']['english'] = commands[command_id]['english']
        self.json_create['command']['italian'] = commands[command_id]['italian']
        self.json_create['confidence'] = commands[confidence]

        msg = json.dumps(self.json_create);

        # send request
        response = requests.post(self.CB_BASE_URL+"entities/", data = msg, headers = CB_HEADER)

        if response.ok: # response successful
            print("CB response -> status " + response.status_code.__str__())
        else: # response ko
            print("CB response -> " + response.text)

    # function move left (y = 1) or right (y = -1)
    def move_left_right(self, isFirst, value):
        response = None
        msg = None
        if isFirst :
            self.json_create['linear']['value']['x']['value'] = 0
            self.json_create['linear']['value']['y']['value'] = value
            self.json_create['linear']['value']['z']['value'] = 0

            self.json_create['angular']['value']['x']['value'] = 0
            self.json_create['angular']['value']['y']['value'] = 0
            self.json_create['angular']['value']['z']['value'] = 0

            msg = json.dumps(self.json_create);

            # send request
            response = requests.post(self.CB_BASE_URL+"entities/", data = msg, headers = CB_HEADER)
        else:
            self.json_update['linear']['value']['x']['value'] = 0
            self.json_update['linear']['value']['y']['value'] = value
            self.json_update['linear']['value']['z']['value'] = 0

            self.json_update['angular']['value']['x']['value'] = 0
            self.json_update['angular']['value']['y']['value'] = 0
            self.json_update['angular']['value']['z']['value'] = 0

            msg = json.dumps(self.json_update);

            # send request
            response = requests.post(self.CB_BASE_URL + "entities/.turtle1.cmd_vel/attrs", data = msg, headers = CB_HEADER)
        if response.ok: # response successful
            print("CB response -> status " + response.status_code.__str__())
        else: # response ko
            print("CB response -> " + response.text)