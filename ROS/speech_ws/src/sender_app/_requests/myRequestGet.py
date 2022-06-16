import json
import requests

CB_HEADER = {'Content-Type': 'application/json'}
CB_BASE_URL = None


class MyRequestGet:

    def __init__(self, address, port=1026):
        self.CB_BASE_URL = "http://{}:{}/v2/".format(address, port)

    # call entities cb
    def entities(self):
        print(self.CB_BASE_URL + "entities")
        # send request
        response = requests.get(self.CB_BASE_URL + "entities")
        obj = json.loads(response.text)
        # print response
        print("CB Enitities:\n" + json.dumps(obj, indent=4, sort_keys=True) + "\n+++++++++++++++++++++++++++++\n")

    # call subscriptions cb
    def subscriptions(self):
        print(self.CB_BASE_URL + "subscriptions")
        # send request
        response = requests.get(self.CB_BASE_URL + "subscriptions")
        obj = json.loads(response.text)
        # print response
        print("CB Subscriptions:\n" + json.dumps(obj, indent=4, sort_keys=True) + "\n+++++++++++++++++++++++++++++\n")

    # call types cb
    def types(self):
        print(self.CB_BASE_URL + "types")
        # send request
        response = requests.get(self.CB_BASE_URL + "types")
        obj = json.loads(response.text)
        # print response
        print("Types:\n" + json.dumps(obj, indent=4, sort_keys=True) + "\n+++++++++++++++++++++++++++++\n")

#    def registrations(self):
#        print(self.CB_BASE_URL + "registrations")
#        response = requests.get(self.CB_BASE_URL + "registrations")
#        obj = json.loads(response.text)
#        print("CB Registrations:\n" + json.dumps(obj, indent=4, sort_keys=True) + "\n+++++++++++++++++++++++++++++\n")