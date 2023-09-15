#!/usr/bin/python3
# coding=utf-8

import rospy
import qi
from threading import Thread
from pepper_handler import HandlerSuperQI
from settings.pepper import STAND_INIT, CROUCH

class Handler(HandlerSuperQI):

    def __init__(self, ip, port):
        super(Handler, self).__init__(ip, port)
        self.HEAD_TIME = float(rospy.get_param("head_time", 1))
        '''
        Time to wait before to set send a command to controller the Pepper's head
        '''

        self.BODY_TIME = float(rospy.get_param("body_time", 2))
        '''
        Time to wait before to set send a command to controller the Pepper's body
        '''
        self.create_posture_service(STAND_INIT)
        self.create_move_service()
        self.create_background_movement_service()
        self.motion_service.moveInit()
        self.start_threads()

    def create_move_service(self):
        '''
        Creates the ALMotion service.
        Sets the head stiffnesses to the maximum (1.0) and disables the smart stiffnesses.
        Deactivates arm movement, in this way Pepper will not move them to re-balance its self and the torso will remain straight.
        '''
        self.motion_service = self.session.service("ALMotion")
        self.motion_service.setStiffnesses("Head", 1.0)
        self.motion_service.setSmartStiffnessEnabled(False)
        self.motion_service.setMoveArmsEnabled(False, False)

    def create_background_movement_service(self):
        '''
        Creates the ALBackgroundMovement service.
        Set the background movement to True, in this way Pepper positions her arms along her torso and starts the animation
        of the finger movement
        '''
        self.background_service = self.session.service("ALBackgroundMovement")
        self.background_service.setEnabled(True)

    def create_posture_service(self, posture):
        '''
        Creates the ALRobotPosture service.
        Sets the Pepper position to posture given as parameter.
        The only two positions allowed are `StandInit` and `Crouch`.
        Pepper will reach the assigned position in with a velocity set to 0.5
        '''
        assert posture == "StandInit" or posture == "Crouch"
        self.posture_service = self.session.service("ALRobotPosture")
        self.posture_service.goToPosture(posture, 0.5)
        rospy.loginfo("Posture: {}".format(posture))

    def wakeUp(self):
        '''
        Wake up the robot
        '''
        self.motion_service.wakeUp()


    def rest(self):
        '''
        Sets the robot to rest position
        '''
        self.motion_service.rest()

    def start_threads(self):
        '''
        Starts the `head_thread` and `body_thread` threads
        '''

        def head_thread():
            '''
            This thread forces the robot to look in front.
            The service to set the head position is called every `HEAD_TIME` seconds; this parameter (HEAD_TIME)
            is set by the user on the ROS parameter server.
            '''
            def head_position_callback():
                while not rospy.is_shutdown():
                    names = ["HeadYaw", "HeadPitch"]
                    angles = [0.0, 0.0]
                    max_speed = 0.1
                    try:
                        self.motion_service.setAngles(names, angles, max_speed)
                    except RuntimeError:
                        rospy.logwarn("qi session died")
                        self.reconnect()
                    rospy.sleep(self.HEAD_TIME)
            Thread(target=head_position_callback, daemon=True).start()

        def body_thread():
            '''
            This thread forces the robot to remain in an upright position .
            The service to set the body position is called every  `BODY_TIME` seconds; this parameter (BODY_TIME)
            is set by the user on the ROS parameter server.
            '''
            def body():
                while not rospy.is_shutdown():
                    try:
                        self.motion_service.setAngles("HipRoll", 0.0, 0.2)
                    except RuntimeError:
                        rospy.logwarn("qi session died")
                        self.reconnect()
                    rospy.sleep(self.BODY_TIME)
            Thread(target=body, daemon=True).start()

        head_thread()
        body_thread()

if __name__ == "__main__":
    NODE_NAME = "pepper_posture_node"
    rospy.init_node(NODE_NAME)
    rospy.loginfo("{} started".format(NODE_NAME))
    ip = rospy.get_param("/pepper_ip")
    port = rospy.get_param("/port", "9559")
    handler = Handler(ip, port)
    rospy.spin()