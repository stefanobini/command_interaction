import qi
from std_msgs.msg import Bool

class HandlerSuper:
    def start_threads(self):
        '''
        Override this method if you need to start some threads
        '''
        raise NotImplementedError

    def loop(self):
        '''
        Start the loop of the node
        '''
        raise NotImplementedError

class HandlerSuperQI(HandlerSuper):
    def __init__(self, ip, port):
        '''
        Superclass that takes as input the ip and port of the robot and sets the qi Session as an attribute of the class

        :param ip (str): IP of the robot
        :param port (int): port of the robot used to establish the connection
        '''

        self.ip = ip
        '''
        IP set
        '''

        self.port = port
        '''
        Port set
        '''

        self.session = self.connect_to_pepper(ip, port)
        '''
        Session object
        '''

    def create_move_service(self):
        '''
        Uses the session object to get a ALMotion object
        '''
        return self.session.service("ALMotion")

    def create_memory_service(self):
        '''
        Uses the session object to get a ALMemory object
        '''
        return self.session.service("ALMemory")

    def emergency_handler(self, data):
        '''
        Saves the emergency state given as input in an attribute variable
        '''
        self.emergency = data.data

    def create_led_service(self):
        '''
        Uses the session object to get a ALLeds object
        '''
        return self.session.service("ALLeds")

    def connect_to_pepper(self, ip, port):
        '''
        This function take in input the ip and the port of the robot and start the connection using the qi library

        :param ip (str): IP of the robot
        :param port (int): Port
        :return: qi Session
        '''
        session = qi.Session()
        session.connect("tcp://" + ip + ":" + str(port))
        return session

    def reconnect(self):
        '''
        This function re-creates a qi Session to Pepper robot.
        The session is added as an attribute of class
        '''

        self.session = self.connect_to_pepper(self.ip, self.port)