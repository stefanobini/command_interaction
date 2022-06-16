import json

from http.server import BaseHTTPRequestHandler
from logger import Log

#Intercepts incoming messages
class RequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        datalen = int(self.headers['Content-Length']) # size receive message
        data = self.rfile.read(datalen) # read receive messages
        obj = json.loads(data) #convert message to json
        Log("INFO", json.dumps(obj, indent=4, sort_keys=True)) # print receive messages
        self.send_response(200)
        self.end_headers()
