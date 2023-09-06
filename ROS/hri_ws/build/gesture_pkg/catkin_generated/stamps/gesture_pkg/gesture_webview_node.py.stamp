#!/usr/bin/env python
#from demo_pkg.msg import AggregationState
#from demo_pkg.msg import TaggedFaceArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Log
from vision_msgs.msg import Detection2DArray
import base64
import cv2
import json
import message_filters
import numpy as np
import os
import random
import re
import rospy
import subprocess
import argparse
import time
from PIL import Image as ImageRGB
from pepper_utils import Pepper
#from demo_pkg.msg import TaggedFaceArray, ObjectsDetected,Object, Detections
from settings import demo_settings
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Log
import base64
import cv2
import json
import message_filters
import numpy as np
import os
import random
import re
import rospy
import subprocess
import argparse
import time
import ifcfg


NODE_NAME = "console_webview_node"

COLORS = ["red", "green", "yellow", "cyan", "white"]
FONT = cv2.FONT_HERSHEY_SIMPLEX
SCALE = 0.5
THICKNESS = 1

video_script = 'document.getElementById("rtv").src="data:image/png;base64, {}";'

class ImageCallback:

    def __init__(self, tablet_service):
        self.tablet_service = tablet_service
        self.events_publisher = rospy.Publisher("events", String, queue_size=1)
        self.pub_ss = rospy.Publisher("text_2_speech", String, queue_size=1)
        self.timestamp = time.time()
        self.count = 0
        self.previous_label = None

        ##init
        #saves_path = "/home/felice/speech-command_interaction/ROS/speech_ws/src/gesture_pkg/src/saves"
        #self.out = cv2.VideoWriter('{}/filename.avi'.format(saves_path), cv2.VideoWriter_fourcc(*'MJPG'), 6, (320,320))

    def __call__(self, msg_img, msg_state):
        ## acquire image
        img = np.ndarray(shape=(msg_img.height, msg_img.width, 3), dtype=np.uint8, buffer=msg_img.data) # RAW image
        self.count += 1
        ## compute frame rate
        previous_timestamp = self.timestamp
        self.timestamp = time.time()
        frame_rate = round(1 / (self.timestamp - previous_timestamp), 2)
        print(frame_rate)
        (w_fps, h_fps), _ = cv2.getTextSize(str(frame_rate), cv2.FONT_HERSHEY_SIMPLEX, 1e-3 * 3 * 200, 2)
        x_fps, y_fps = 10, 10
        offset = 5
        #cv2.rectangle(img, (x_fps-offset, y_fps-offset), (x_fps+w_fps+offset, y_fps+h_fps+offset),  (0, 0, 0), 2)
        #cv2.putText(img, str(frame_rate), (x_fps,  y_fps+h_fps), cv2.FONT_HERSHEY_SIMPLEX, 1e-3 * 3 * 200, (0, 0, 0), 2)

        #response.header.stamp = msg.header.stamp
        #frame = self.bridge.imgmsg_to_cv2(msg)
        # rects = list()
        
        ## show detections
        for detection in msg_state.detections:

            if detection.header.frame_id == "NoGesture":
                continue

            right = int(detection.bbox.center.x + detection.bbox.size_x//2)
            bottom = int(detection.bbox.center.y + detection.bbox.size_y//2)
            left = int(detection.bbox.center.x - detection.bbox.size_x//2)
            top = int(detection.bbox.center.y - detection.bbox.size_y//2)
            cv2.rectangle(img, (left, top), (right, bottom),  (0, 255, 0), 2)
            #cv2.putText(img, '{} ({:.2f})'.format(detection.header.frame_id, detection.results[0].score), (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 1e-3 * 3 * 200, (0, 0, 255), 2)
            cv2.putText(img, '{}'.format(detection.header.frame_id), (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 1e-3 * 3 * 200, (0, 0, 255), 2)
            print(detection.header.frame_id))
            
            
            #cv2.putText(img, "{}".format(detection.results[0].id), (int(detection.bbox.center.x)-10, int(detection.bbox.center.y)-10), cv2.FONT_HERSHEY_SIMPLEX, 1e-3 * 3 * 200, (255, 0, 0), 3)
            #cv2.circle(img, (int(detection.bbox.center.x), int(detection.bbox.center.y)),radius=5, color= (255,0,0), thickness=-1)      
            # rects.append((int(left), int(top), int(right), int(bottom)))
            
        ## update centroidTracker object
        # objects_centroid, _ = self.hand_tracker.update(rects=rects)

        ## image tracking visualization
        # for (objectID, centroid) in objects_centroid.items():
        #     text = "ID {}".format(objectID)
        #     #print (text)
        #     cv2.putText(img, text,(centroid[0]-10, centroid[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 1e-3 * 3 * 300, (0, 0, 255), 3)
        #     cv2.circle(img, (centroid[0], centroid[1]),radius=5, color= (0,0,255), thickness=-1)      

        
        #self.out.write(img)
        if not detection.header.frame_id == "NoGesture" and self.count > 10:
            if self.previous_label != detection.header.frame_id or self.count > 20:
                self.count = 0
                self.previous_label = detection.header.frame_id
                message = String()
                message.data = detection.header.frame_id
                self.pub_ss.publish(message) 
        ret, buffer = cv2.imencode('.jpg', img)
        if ret:
            try:
                self.tablet_service.executeJS(video_script.format(base64.b64encode(buffer)))
            except Exception as e:
                self.events_publisher.publish("WatchDog/TabletFault") 

        


class ConsoleWebviewNode:
    '''ConsoleWebviewNode implements a ROS interface to show data on the Pepper's tablet through the web interface.
    The node subscribes to the following topics:
    
    - **sync_rgb**: raw images from the sync node
    - **rosout**: All the loggers publish here
    - **tracking_state**: all the data about the people in the scene
    The node does not publish on any topic.
    The available methods are:
    
    - **\_\_init\_\_(self)**: constructor
    - **start(self)**: starts the ros node instance
    '''

    def __init__(self):
        Pepper.disableAutonomousLife(Pepper(demo_settings.pepper.ip))
        self.tablet_service = Pepper(demo_settings.pepper.ip).session.service("ALTabletService")
        

        net = ifcfg.interfaces()
        if "eth0" in net and "inet" in net["eth0"] and net["eth0"]["inet"] is not None:
            webview_ip = net["eth0"]["inet"]
            print("ETH0", webview_ip)
        elif "wlan0" in net and "inet" in net["wlan0"] and net["wlan0"]["inet"] is not None:
            webview_ip = net["wlan0"]["inet"]
            print("WLAN0", webview_ip)
        else:
            webview_ip = demo_settings.io.webview.ip
            print("DEFAULT", webview_ip)
        url = "http://{}:{}".format(webview_ip, demo_settings.io.webview.port)
        self.tablet_service.resetTablet()
        check = self.tablet_service.showWebview(url)
        self.events_publisher = rospy.Publisher("events", String, queue_size=1)
        print("###################### TABLET OK ######################")

    def stt_callback(self, data):
        # Add to tablet
        try:
            self.tablet_service.executeJS("lastReceivedMessage(\"%s\")"%data.data)
            self.tablet_service.executeJS("addUserMessage(\"%s\")"%data.data)
        except:
            self.events_publisher.publish("WatchDog/TabletFault")

    def pepper_listening(self, data):
        # Add to tablet
        try:
            if data.data == "FieraMain/TextSpokenDone":
                time.sleep(1)
                self.tablet_service.executeJS("hidePopup();")
        except:
            self.events_publisher.publish("WatchDog/TabletFault")


    def start(self):
        rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
        image_callback = ImageCallback(self.tablet_service)
        
        raw_image = message_filters.Subscriber('in_rgb', Image)
        #tracking_state = message_filters.Subscriber('tracking_state', AggregationState)
        #objdet_topic = message_filters.Subscriber("detected_object", Detections)
        #ts = message_filters.TimeSynchronizer([raw_image, tracking_state, objdet_topic], 18)
        #ts = message_filters.TimeSynchronizer([raw_image, tracking_state], 2)
        gesture = message_filters.Subscriber('hand_gesture_recognition', Detection2DArray)
        ts = message_filters.TimeSynchronizer([raw_image, gesture], 100)  #50 
        #rospy.Subscriber("stt_listening", String, self.stt_callback)
        # rospy.Subscriber("pepper_talking", String, self.pepper_talking)
        ts.registerCallback(image_callback)
        
        rospy.spin()


if __name__ == "__main__":
    
    webserver = None
    try:
        webserver_path = "{}/webserver.py".format(os.path.dirname(os.path.abspath(__file__)))
        webserver = subprocess.Popen("python3 {}".format(webserver_path).split(), stdout=subprocess.PIPE)
        while "Debug" not in webserver.stdout.readline():
            time.sleep(2)
        console = ConsoleWebviewNode()
        console.start()
    except rospy.ROSInterruptException:
        pass
    finally:
        if webserver is not None:
            webserver.kill()