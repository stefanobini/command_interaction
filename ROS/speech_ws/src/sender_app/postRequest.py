import json
import requests
import time
from demo_utils.io.request_msgs import initial_msg, update_msg

CB_HEADER = {'Content-Type': 'application/json'}
#CB_BASE_URL = None

commands = {
    0: {'label':0, 'english':'Bring me the screwdriver', 'italian':'Portami il cacciavite'},
    1: {'label':1, 'english':'Take the screwdriver', 'italian':'Prendi il cacciavite'},
    2: {'label':2, 'english':'Bring me the windows control panel', 'italian':'Portami la mostrina comandi'},
    3: {'label':3, 'english':'Take the windows control panel', 'italian':'Prendi la mostrina comandi'},
    4: {'label':4, 'english':'Bring me the rearview mirror', 'italian':'Portami lo specchietto retrovisore'},
    5: {'label':5, 'english':'Take the rearview mirror', 'italian':'Prendi lo specchietto retrovisore'},
    6: {'label':6, 'english':'Release', 'italian':'Rilascia'},
    7: {'label':7, 'english':'No command', 'italian':'Nessun comando'}
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
        self.json_create = json.load(initial_msg)
        self.json_update = json.load(msg_update)

        
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