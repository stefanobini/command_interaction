#!/usr/bin/env python3
from cv_bridge import CvBridge
from gesture_pkg.srv import *
from settings import demo_settings
from demo_utils.video import RealSenseImageSource 
from sensor_msgs.msg import Image
import cv2
import rospy

NODE_NAME = "image_input_node"

class ServiceCallback:

    def __init__(self, image_source):
        self.response = CameraInfoResponse()
        self.response.width = image_source.width
        self.response.height = image_source.height
        self.response.hfov, self.response.vfov = image_source.get_fov()

    def __call__(self, msg):
        return self.response

class PublisherCallback:



    def __init__(self, publisher, image_source):
        self.bridge = CvBridge()
        self.image_source = image_source
        self.image_publisher = publisher

    def __call__(self):
        frame = self.image_source.get_color_frame()
        #frame = self.image_source.get_depth_frame()
        #frame = cv2.flip(frame,1)
        frame = self.crop_image_to_square(frame)
        frame = cv2.resize(frame, (320, 320))
        if frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame)
            msg.header.stamp = rospy.Time.now()
            self.image_publisher.publish(msg)


    def crop_image_to_square(self, image):

        height = image.shape[0]
        width = image.shape[1]
        crop_size = min(height, width)
        new_height = min(height, crop_size)
        new_width = min(width, crop_size)
        offset_height = max(height - crop_size, 0)
        offset_width = max(width - crop_size, 0)
        #GET THE TOP AND LEFT COORDINATES OF A CENTRAL CROP
        top = int(offset_height/2)
        left = int(offset_width/2)
        image_arr = image[top : top + crop_size, left : left + crop_size]

        return image_arr

class ImageInputNode:
    '''ImageInputNode implements a ROS interface for the different image sources.

    The node has not subscription to any topic.

    The node publishes on the following topics:

    - **in_rgb** : Image representing the acquired frame.

    The node is a server for the following services:
    
    - **camera_info** : returns the information about camera resolution and FOV

    The available methods are:
    
    - **\_\_init\_\_(self)**: constructor
    - **start(self)**: starts the ros node instance
    '''

    def start(self):
        rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
        pub = rospy.Publisher('in_rgb', Image, queue_size=1)
        
        image_source = RealSenseImageSource(fps=demo_settings.io.camera.fps)
        callback_function = PublisherCallback(pub, image_source)
        service_callback = ServiceCallback(image_source)
        rospy.Service('camera_info', CameraInfo, service_callback)

        while not rospy.is_shutdown():
            callback_function()

if __name__ == "__main__":
    try:
        image_input = ImageInputNode()
        image_input.start()
    except rospy.ROSInterruptException:
        pass
        