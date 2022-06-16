from http.server import HTTPServer
from Receiver.requestHandler import RequestHandler
from logger import Log


class MyReceiver:
    def __init__(self, address="0.0.0.0", port=8080):
        self.address = address
        self.port = port
        self.stopped = False
        Protocol = "HTTP/1.0"

        # set ip and port my server
        server_address = (self.address, self.port)

        # initialize RequestHandler
        RequestHandler.protocol_version = Protocol
        self.httpd = HTTPServer(server_address, RequestHandler)

    def start(self):  # Start server method
        sa = self.httpd.socket.getsockname()
        Log("INFO", "\nServing HTTP on", sa[0], "port", sa[1], "...")

        while not self.stopped:
            self.httpd.handle_request()

    def close(self):  # Stop server method
        self.stopped = True
