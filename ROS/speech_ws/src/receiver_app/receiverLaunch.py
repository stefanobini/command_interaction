import sys
import signal
import requests


from logger import Log, initLog
from Receiver.myReceiver import MyReceiver

CB_HEADER = {'Content-Type': 'application/json'}
CB_BASE_URL = None
selection_port = None
selection_address = None

initLog()

# Input data acquisition

selection_port = input("CB Subscription\n"\
                  "Enter your server port number : ") #port my Receiver

selection_address = input("Enter your server ip address : ") #address ip my Receiver

selection_port_CB = input("Enter CB port : ") #port Context Broker

selection_address_CB = input("Enter CB ip address : ") #address ip Context Broker

#subscription message
'''
msg = "{\n  \"throttling\": 1, \n  \"description\": \"Cal-Tek Subscription\",\n  \"subject\": {\n    \"entities\": [" \
      "\n      {\n          \"id\": \".turtle1.pose\",\n          \"type\": \"turtlesim%2FPose\"\n      }\n    ]\n  " \
      "},\n  \"notification\": \n  {\n    \"http\": \n    {\n      \"url\": \"http:\/\/"+selection_address+":"+selection_port+"\",\n      " \
      "\"method\" : \"POST\",\n      \"headers\" : \n        {\n " \
      "           \"Content-Type\" : \"application\/json\"\n        }\n    " \
      "}\n  }\n} "
'''

msg = """{
    \"throttling\": 1,
    \"description\": \"UNISA Subscription\",
    \"subject\": {
        \"entities\": [
        {
            \"id\": \"UNISA.SpeechGestureAnalysis.Speech\",
            \"type\": \"Speech\"
        }
        ]
    },
    \"notification\": 
    {
        \"http\": 
        {
        \"url\": \"http:\/\/"+selection_address+":"+selection_port+"\",
                    \"Content-Type\" : \"application\/json\"
            }
        }
    }
}"""


CB_BASE_URL = "http://{}:{}/v2/".format(selection_address_CB, selection_port_CB) #url send notification

Log("INFO", "Send subcription")
Log("INFO", msg)

response = requests.post(CB_BASE_URL+"subscriptions/", data = msg, headers = CB_HEADER) #send request to Context Broker
if response.ok: #positive response, notification accepted
    print("CB response -> status " + response.status_code.__str__())
else: #error response
    print("CB response -> " + response.text)

Log("INFO", "Initialized")

#Start server, receive message
try:
    server = MyReceiver(selection_address, int(selection_port))
except Exception as ex:
    raise Exception("Unable to create a Receiver")
else: # close application and server
    def signal_handler(signal, frame):
        Log("INFO", ('\nExiting from the application'))
        server.close()
        Log("INFO", ('\nExit'))
        sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

Log("INFO", "\nStarting...")
Log("INFO", "---------------------------------\n")
server.start()